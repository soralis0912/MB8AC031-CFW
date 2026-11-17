/* armcore_host.c -- run the ARM image's queue and services on the host.
 *
 * tests/test_arm.py compiles this together with src/armcore/armmain.c,
 * ipc.c, svc_sys.c, svc_watch.c, and svc_card.c, built with -DARM_HOSTED (DTCM is the
 * array below; src/armcore/core.h).  diag.c is not built: its accesses go
 * to real addresses, so this file stands in for it with a small fake memory
 * map.  The main FR's half of the queue is played here with plain array
 * writes, the way src/main/arm_ipc.c does it through the window.
 *
 * What this cannot tell: anything about the real core.  It checks the C.
 *
 * Prints one line per check, then "armcore_host: PASS" or FAIL.
 */
#include <stdio.h>

#include "../src/armcore/core.h"
#include "mb86m01.h"
#include "regs_dione.h"

unsigned arm_hosted_dtcm[ARM_DTCM_SIZE / 4u];

/* --- stand-ins for diag.c and vectors.S -------------------------------- */

#define REG_A       0x00f40000u     /* a word the tests change            */
#define REG_B       0x00f40004u
#define NO_ANSWER   0x80000000u     /* reads abort                        */

static unsigned reg_a, reg_b;

void diag_init(void)
{
    MB(ARM_MB_LOOPS) = 0;
}

void diag_poll(void)
{
    MB(ARM_MB_LOOPS) = MB(ARM_MB_LOOPS) + 1u;
}

unsigned arm_cp15(unsigned which)
{
    return which == ARM_CP_ID ? 0x41069265u : 0u;
}

unsigned arm_access(unsigned op, unsigned addr, unsigned val, unsigned *out)
{
    unsigned *r = addr == REG_A ? &reg_a : addr == REG_B ? &reg_b : 0;
    int write = op == ARM_OP_WR32 || op == ARM_OP_WR16 || op == ARM_OP_WR8;

    if (addr >= NO_ANSWER || r == 0)
        return ARM_ST_ABORT;
    if (write)
        *r = val;
    *out = *r;
    return ARM_ST_OK;
}

/* --- a card block (svc_card.c's RD / WR) --------------------------------
 * Registers as plain words, plus a card: once VCC is on and RST goes up,
 * the ATR arrives a few bytes per look at CARD_MODE; after GO the
 * answer block does the same.  `silent` makes the card say nothing. */

static const unsigned char ATR[] = {
    0x3b, 0xf0, 0x12, 0x00, 0xff, 0x91, 0x81, 0xb1, 0x7c, 0x45, 0x1f, 0x03,
    0x99};
static const unsigned char IFS_REQ[] = {0x00, 0xc1, 0x01, 0xfe, 0x3e};
static const unsigned char IFS_RES[] = {0x00, 0xe1, 0x01, 0xfe, 0x1e};

static struct {
    unsigned reg[0x40 / 4];
    const unsigned char *out;       /* bytes the card will send          */
    unsigned out_n, out_at, ready;  /* how many, taken, arrived so far    */
    unsigned char tx[300];
    unsigned tx_n, sent_n, writes, silent, extra;
} card;

#define CREG(a)     card.reg[((a) - CARD_EN) / 4u]

static void card_reset(void)
{
    unsigned i;

    for (i = 0; i < sizeof(card.reg) / 4u; i++)
        card.reg[i] = 0;
    card.out = 0;
    card.out_n = card.out_at = card.ready = card.tx_n = card.sent_n = card.writes = 0;
    card.silent = card.extra = 0;
}

static void card_talk(const unsigned char *b, unsigned n)
{
    if (card.silent)
        return;
    card.out = b;
    card.out_n = n + card.extra;    /* `extra`: one block too many bytes */
    card.out_at = card.ready = 0;
}

unsigned arm_hosted_rd(unsigned addr)
{
    if (addr == REG_A || addr == REG_B)     /* svc_sys.c's SUM */
        return addr == REG_A ? reg_a : reg_b;
    if (addr < CARD_EN || addr >= CARD_EN + sizeof(card.reg))
        return 0xbadu;
    if (addr == CARD_MODE) {
        if (card.ready < card.out_n)
            card.ready += card.ready + 3u <= card.out_n ? 3u
                          : card.out_n - card.ready;
        return (CREG(CARD_MODE) & 0xffffu)
               | ((card.ready - card.out_at) << 16);
    }
    if (addr == CARD_RX) {
        unsigned i = card.out_at;

        if (i >= card.ready)
            return 0xeeu;           /* a read past what arrived */
        card.out_at++;
        return i < card.out_n - card.extra ? card.out[i] : 0x55u;
    }
    return CREG(addr);
}

void arm_hosted_wr(unsigned addr, unsigned val)
{
    card.writes++;
    if (addr == CARD_TX) {
        if (card.tx_n < sizeof(card.tx))
            card.tx[card.tx_n++] = (unsigned char)val;
        return;
    }
    if (addr == CARD_GO) {
        if (card.tx_n == sizeof(IFS_REQ))
            card_talk(IFS_RES, sizeof(IFS_RES));
        card.sent_n = card.tx_n;
        card.tx_n = 0;
        return;
    }
    CREG(addr) = val;
    if (addr == CARD_PINS && (val & CARD_PIN_RST) && CREG(CARD_VCC)
        && CREG(CARD_EN) == CARD_EN_ON)
        card_talk(ATR, sizeof(ATR));
}

/* --- the main FR's half ------------------------------------------------ */

#define W(off)      DTCM_WORD(ARM_IPC + (off))

static unsigned call(unsigned svc, unsigned op, unsigned a0, unsigned a1,
                     unsigned d0, unsigned d1)
{
    unsigned head = W(IPC_REQ_HEAD);
    unsigned s = IPC_REQ + (head % IPC_SLOTS) * IPC_SLOT_BYTES;

    W(s + IPC_Q_SEQ) = head + 1u;
    W(s + IPC_Q_CALL) = IPC_CALL(svc, op);
    W(s + IPC_Q_ARG0) = a0;
    W(s + IPC_Q_ARG1) = a1;
    W(s + IPC_Q_DATA) = d0;
    W(s + IPC_Q_DATA + 4u) = d1;
    W(s + IPC_Q_DATA + 8u) = 0xd2u;
    W(s + IPC_Q_DATA + 12u) = 0xd3u;
    W(IPC_REQ_HEAD) = head + 1u;
    return head + 1u;
}

struct got {
    unsigned seq, call, status, val, d0;
};

/* -> 1 with *g, 0 if no reply waits. */
static int reap(struct got *g)
{
    unsigned tail = W(IPC_REP_TAIL);
    unsigned s = IPC_REP + (tail % IPC_SLOTS) * IPC_SLOT_BYTES;

    if (W(IPC_REP_HEAD) == tail)
        return 0;
    g->seq = W(s + IPC_R_SEQ);
    g->call = W(s + IPC_R_CALL);
    g->status = W(s + IPC_R_STATUS);
    g->val = W(s + IPC_R_VAL);
    g->d0 = W(s + IPC_R_DATA);
    W(IPC_REP_TAIL) = tail + 1u;
    return 1;
}

/* BUF_TX / BUF_RX bytes, packed as armipc.h says. */
static void buf_put(const unsigned char *b, unsigned n)
{
    unsigned i;

    for (i = 0; i < n; i++) {
        unsigned a = ARM_BUF + BUF_TX + (i & ~3u), sh = 24u - 8u * (i & 3u);

        DTCM_WORD(a) = (DTCM_WORD(a) & ~(0xffu << sh)) | ((unsigned)b[i] << sh);
    }
}

static int buf_is(const unsigned char *b, unsigned n)
{
    unsigned i;

    for (i = 0; i < n; i++) {
        unsigned w = DTCM_WORD(ARM_BUF + BUF_RX + (i & ~3u));

        if (((w >> (24u - 8u * (i & 3u))) & 0xffu) != b[i])
            return 0;
    }
    return 1;
}

/* Run until a reply comes or `max` passes go by.  -> passes run, or ~0. */
static unsigned until_reply(struct got *g, unsigned max)
{
    unsigned n;

    for (n = 0; n < max; n++) {
        arm_pass();
        if (reap(g))
            return n + 1u;
    }
    return ~0u;
}

static void cancel(unsigned what)
{
    W(IPC_CANCEL_WHAT) = what;
    W(IPC_CANCEL_REQ) = W(IPC_CANCEL_REQ) + 1u;
}

static void passes(unsigned n)
{
    while (n--)
        arm_pass();
}

static int fails;

static void ok(int cond, const char *label)
{
    printf("  [%s] %s\n", cond ? "ok " : "NG ", label);
    if (!cond)
        fails++;
}

int main(void)
{
    struct got g, g2;
    unsigned s1, s2, s3, n, i;

    /* What arm.c's ARM_RUN leaves: a zeroed header. */
    arm_start();
    ok(W(IPC_MAGIC) == IPC_MAGIC_VAL && MB(ARM_MB_ALIVE) == ARM_ALIVE_MAGIC,
       "start-up: 'IPC1' and 'ARM1'");

    /* SYS */
    s1 = call(SVC_SYS, SYS_ECHO, 0x00ff00ffu, 0, 0xa0u, 0xa1u);
    passes(1);
    ok(reap(&g) && g.seq == s1 && g.status == IPC_ST_OK
       && g.val == 0xff00ff00u && g.d0 == 0xa0u, "ECHO: ~arg0, data back");
    ok(W(IPC_REQ_TAIL) == 1u, "ECHO: the request was taken (tail 1)");
    call(SVC_SYS, SYS_INFO, 0, 0, 0, 0);
    passes(1);
    ok(reap(&g) && g.val == IPC_VERSION && g.status == IPC_ST_OK,
       "INFO: version");
    reg_a = 0x1234u;
    call(SVC_SYS, SYS_PEEK, REG_A, 32, 0, 0);
    call(SVC_SYS, SYS_POKE, REG_B, 16, 0x77u, 0);
    call(SVC_SYS, SYS_PEEK, NO_ANSWER, 32, 0, 0);
    call(SVC_SYS, SYS_PEEK, REG_A, 12, 0, 0);
    passes(4);
    ok(reap(&g) && g.val == 0x1234u, "PEEK reads");
    ok(reap(&g) && g.val == 0x77u && reg_b == 0x77u, "POKE writes, reads back");
    ok(reap(&g) && g.status == IPC_ST_ABORT, "PEEK of a dead address: ABORT");
    ok(reap(&g) && g.status == IPC_ST_ARG, "PEEK width 12: ARG");
    call(7, 0, 0, 0, 0, 0);
    call(SVC_SYS, 9, 0, 0, 0, 0);
    passes(2);
    ok(reap(&g) && g.status == IPC_ST_BADSVC, "service 7: BADSVC");
    ok(reap(&g) && g.status == IPC_ST_BADOP, "SYS op 9: BADOP");
    reg_a = 1;
    reg_b = 3;
    s1 = call(SVC_SYS, SYS_SUM, REG_A, 2, 0, 0);
    s2 = call(SVC_SYS, SYS_SUM, REG_A, 2, 0, 0);
    call(SVC_SYS, SYS_SUM, REG_A + 2u, 1, 0, 0);
    call(SVC_SYS, SYS_SUM, NO_ANSWER, 1, 0, 0);
    passes(5);
    /* Two words take one pass, so the second SUM finds the first done. */
    ok(reap(&g) && g.seq == s1 && g.status == IPC_ST_OK && g.val == 1u
       && g.d0 == 1u, "SUM: words folded, 1 pass");
    ok(reap(&g) && g.seq == s2 && g.status == IPC_ST_OK,
       "SUM again once the first is done");
    ok(reap(&g) && g.status == IPC_ST_ARG, "SUM misaligned: ARG");
    ok(reap(&g) && g.status == IPC_ST_ABORT, "SUM of a dead address: ABORT");
    call(SVC_SYS, SYS_EPTEST, 0, 0, 0, 0);
    call(SVC_SYS, SYS_EPTEST, 0, 129, 0, 0);
    call(SVC_SYS, SYS_EPTEST, 0, 128, 0, 0);
    passes(3);
    ok(reap(&g) && g.status == IPC_ST_ARG, "EPTEST 0 words: ARG");
    ok(reap(&g) && g.status == IPC_ST_ARG, "EPTEST 129 words: ARG");
    ok(reap(&g) && g.status == IPC_ST_ABORT,
       "EPTEST with no USB block (the host model): ABORT, nothing written");

    /* WAIT answers later, and does not hold up what comes behind it. */
    reg_a = 0;
    s1 = call(SVC_WATCH, WATCH_WAIT, REG_A, 0, 0xffu, 0x5au);
    s2 = call(SVC_SYS, SYS_ECHO, 1, 0, 0, 0);
    passes(5);
    ok(reap(&g) && g.seq == s2, "ECHO behind a WAIT comes back first");
    ok(!reap(&g), "WAIT: nothing while the condition is false");
    reg_a = 0x125au;
    passes(1);
    ok(reap(&g) && g.seq == s1 && g.status == IPC_ST_OK && g.val == 0x125au
       && !(g.call & IPC_EVENT), "WAIT: answers once ([a] & mask) == value");
    /* The pass that takes the request also makes its first step. */
    s1 = call(SVC_WATCH, WATCH_WAIT, REG_A, 5, 0xffu, 0x00u);
    passes(4);
    ok(!reap(&g), "WAIT with timeout 5: nothing after 4 steps");
    passes(1);
    ok(reap(&g) && g.seq == s1 && g.status == IPC_ST_TIMEOUT,
       "WAIT: TIMEOUT on the 5th step");
    call(SVC_WATCH, WATCH_WAIT, NO_ANSWER, 0, 1, 1);
    passes(1);
    ok(reap(&g) && g.status == IPC_ST_ABORT, "WAIT on a dead address: ABORT");

    /* START / events / STOP */
    reg_b = 0x10u;
    s1 = call(SVC_WATCH, WATCH_START, REG_B, 0, 0xf0u, 0);
    passes(1);
    ok(reap(&g) && g.seq == s1 && g.status == IPC_ST_OK && g.d0 == 0x10u,
       "START: watch number and the value now, at once");
    n = g.val;
    passes(3);
    ok(!reap(&g), "START: no event while nothing changes");
    reg_b = 0x13u;
    passes(2);
    ok(!reap(&g), "START: no event for bits outside the mask");
    reg_b = 0x20u;
    passes(1);
    ok(reap(&g) && g.seq == s1 && (g.call & IPC_EVENT) && g.val == 0x20u
       && g.d0 == 0x10u, "event: new value, old value, the START's seq");
    s2 = call(SVC_WATCH, WATCH_STOP, n, 0, 0, 0);
    passes(2);
    ok(reap(&g) && g.seq == s2 && g.status == IPC_ST_OK, "STOP: OK at once");
    ok(reap(&g) && g.seq == s1 && (g.call & IPC_EVENT)
       && g.status == IPC_ST_STOPPED, "STOP: the watch's last event STOPPED");
    call(SVC_WATCH, WATCH_STOP, n, 0, 0, 0);
    passes(1);
    ok(reap(&g) && g.status == IPC_ST_ARG, "STOP of a stopped watch: ARG");

    /* Four watches at most. */
    for (i = 0; i < WATCH_MAX; i++)
        call(SVC_WATCH, WATCH_START, REG_A, 0, 1, 0);
    s3 = call(SVC_WATCH, WATCH_START, REG_A, 0, 1, 0);
    passes(WATCH_MAX + 1u);
    for (i = 0; i < WATCH_MAX; i++)
        reap(&g);
    ok(reap(&g) && g.seq == s3 && g.status == IPC_ST_BUSY, "5th watch: BUSY");
    for (i = 0; i < WATCH_MAX; i++)
        call(SVC_WATCH, WATCH_STOP, i, 0, 0, 0);
    passes(WATCH_MAX * 2u);
    while (reap(&g))
        ;

    /* A full reply ring stops the taking, and nothing is lost. */
    for (i = 0; i < IPC_SLOTS; i++)
        call(SVC_SYS, SYS_ECHO, i, 0, 0, 0);
    passes(IPC_SLOTS);
    s1 = call(SVC_SYS, SYS_ECHO, 99, 0, 0, 0);
    passes(3);
    ok(W(IPC_REP_HEAD) - W(IPC_REP_TAIL) == IPC_SLOTS
       && W(IPC_REQ_HEAD) - W(IPC_REQ_TAIL) == 1u,
       "reply ring full: the next request waits");
    ok(reap(&g) && g.val == ~0u, "the first of 16 is there");
    passes(1);
    for (i = 0; reap(&g2); i++)
        g = g2;
    ok(i == IPC_SLOTS && g.seq == s1 && g.val == ~99u,
       "with room again: taken and answered, in order");

    /* The cancel words: they get through when the rings do not. */
    while (reap(&g))
        ;
    reg_a = 0;
    s1 = call(SVC_WATCH, WATCH_WAIT, REG_A, 0, 1, 1);         /* never true */
    s2 = call(SVC_WATCH, WATCH_START, REG_B, 0, 0xffffffffu, 0);
    passes(2);
    while (reap(&g))
        ;
    for (i = 0; i < 2u * IPC_SLOTS; i++)        /* fill both rings */
        call(SVC_SYS, SYS_ECHO, i, 0, 0, 0);
    passes(3u * IPC_SLOTS);
    ok(W(IPC_REP_HEAD) - W(IPC_REP_TAIL) == IPC_SLOTS
       && W(IPC_REQ_HEAD) - W(IPC_REQ_TAIL) == IPC_SLOTS,
       "both rings full: a STOP could not get in");
    cancel(s1);
    passes(1);
    ok(W(IPC_CANCEL_ACK) == W(IPC_CANCEL_REQ) && W(IPC_CANCEL_DONE) == 1u
       && W(IPC_CANCEL_DROPPED) == 0u,
       "cancel by SEQ with full rings: the WAIT ended, acknowledged");
    cancel(s1);
    passes(1);
    ok(W(IPC_CANCEL_DONE) == 0u, "cancel of a SEQ already ended: 0 ended");
    cancel(IPC_CANCEL_ALL);
    passes(1);
    ok(W(IPC_CANCEL_DONE) == 1u && W(IPC_CANCEL_DROPPED) == IPC_SLOTS
       && W(IPC_REQ_HEAD) == W(IPC_REQ_TAIL),
       "cancel all: the START ended, 16 queued requests dropped");
    for (i = 0; reap(&g); i++)
        ;
    reg_b = 0x99u;
    passes(5);
    ok(i == IPC_SLOTS && !reap(&g),
       "the replies already out stay; no event from the cancelled START");
    s3 = call(SVC_SYS, SYS_ECHO, 5, 0, 0, 0);
    passes(1);
    ok(reap(&g) && g.seq == s3 && g.val == ~5u, "the queue works again");
    (void)s2;

    /* SVC_CARD */
    card_reset();
    while (reap(&g))
        ;
    call(SVC_CARD, CARDSVC_STATUS, 0, 0, 0, 0);
    call(SVC_CARD, CARDSVC_PARAM, (372u << 16) | 1u, 0, 0, 0);
    call(SVC_CARD, CARDSVC_XFER, 5, 0, 0, 0);
    call(SVC_CARD, CARDSVC_ON, 2, 0, 0, 0);
    call(SVC_CARD, 9, 0, 0, 0, 0);
    passes(5);
    ok(reap(&g) && g.status == IPC_ST_OK && g.val == CARDST_OFF,
       "card STATUS: off");
    ok(reap(&g) && g.status == IPC_ST_STATE, "card PARAM before the ATR: STATE");
    ok(reap(&g) && g.status == IPC_ST_STATE, "card XFER before the ATR: STATE");
    ok(reap(&g) && g.status == IPC_ST_ARG, "card ON polarity 2: ARG");
    ok(reap(&g) && g.status == IPC_ST_BADOP, "card op 9: BADOP");

    s1 = call(SVC_CARD, CARDSVC_ON, 0, 0, 0, 0);
    passes(10);
    s2 = call(SVC_CARD, CARDSVC_ON, 0, 0, 0, 0);
    passes(1);
    ok(reap(&g) && g.seq == s2 && g.status == IPC_ST_BUSY,
       "a second ON while one runs: BUSY");
    n = until_reply(&g, 2000000u);
    ok(n != ~0u && g.seq == s1 && g.status == IPC_ST_OK
       && g.val == sizeof(ATR) && buf_is(ATR, sizeof(ATR))
       && g.d0 == 0x3bf01200u, "ON: the ATR, in BUF_RX and the reply");
    ok(n + 11u >= (10u + 1u) * IPC_PASSES_PER_MS,
       "ON: the power-up waits are kept (passes >= 11 ms)");
    ok(CREG(CARD_VCC) == 1u
       && CREG(CARD_PINS) == (CARD_PIN_CLK | CARD_PIN_RST),
       "ON: powered, CLK and RST up, polarity 0");
    s1 = call(SVC_CARD, CARDSVC_ON, 0, 0, 0, 0);
    passes(1);
    ok(reap(&g) && g.seq == s1 && g.status == IPC_ST_OK
       && g.val == sizeof(ATR), "ON when up: the same answer at once");

    call(SVC_CARD, CARDSVC_PARAM, (372u << 16) | 1u, (4u << 8) | 4u, 0, 0);
    call(SVC_CARD, CARDSVC_PARAM, (372u << 16) | 0u, 0, 0, 0);
    passes(2);
    ok(reap(&g) && g.status == IPC_ST_OK
       && CREG(CARD_BAUD) == (372u << 7)
       && CREG(CARD_CWT) == 11u + 16u && CREG(CARD_BWT) == 11u + 960u * 16u,
       "PARAM: baud, CWT, BWT as card.c");
    ok(reap(&g) && g.status == IPC_ST_ARG, "PARAM D = 0: ARG");

    buf_put(IFS_REQ, sizeof(IFS_REQ));
    s1 = call(SVC_CARD, CARDSVC_XFER, sizeof(IFS_REQ), 0, 0, 0);
    n = until_reply(&g, 100000u);
    ok(card.sent_n == sizeof(IFS_REQ) && card.tx[0] == 0x00u
       && card.tx[1] == 0xc1u && card.tx[4] == 0x3eu,
       "XFER: the block from BUF_TX went out");
    ok(n != ~0u && g.seq == s1 && g.status == IPC_ST_OK
       && g.val == sizeof(IFS_RES) && buf_is(IFS_RES, sizeof(IFS_RES)),
       "XFER: the answer block in BUF_RX");

    card.silent = 1;
    s1 = call(SVC_CARD, CARDSVC_XFER, sizeof(IFS_REQ), 3, 0, 0);
    n = until_reply(&g, 100000u);
    ok(n != ~0u && g.seq == s1 && g.status == IPC_ST_TIMEOUT && g.val == 0u
       && n >= 3u * IPC_PASSES_PER_MS, "XFER, silent card: TIMEOUT after 3 ms");

    card.silent = 0;
    card.extra = 1;
    s1 = call(SVC_CARD, CARDSVC_XFER, sizeof(IFS_REQ), 0, 0, 0);
    n = until_reply(&g, 100000u);
    ok(n != ~0u && g.seq == s1 && g.status == IPC_ST_IO,
       "XFER, one byte past the block: IO");
    card.extra = 0;

    card.silent = 1;
    s1 = call(SVC_CARD, CARDSVC_XFER, sizeof(IFS_REQ), 0, 0, 0);
    passes(10);
    s2 = call(SVC_CARD, CARDSVC_OFF, 0, 0, 0, 0);
    passes(2);
    ok(reap(&g) && g.seq == s2 && g.status == IPC_ST_OK
       && reap(&g2) && g2.seq == s1 && g2.status == IPC_ST_STOPPED,
       "OFF during XFER: OK, and the XFER answers STOPPED");
    ok(CREG(CARD_VCC) == 0u && CREG(CARD_PINS) == 0u,
       "OFF: power down as card.c");

    card.silent = 1;
    s1 = call(SVC_CARD, CARDSVC_ON, 1, 2, 0, 0);
    n = until_reply(&g, 2000000u);
    ok(n != ~0u && g.seq == s1 && g.status == IPC_ST_TIMEOUT,
       "ON, silent card: TIMEOUT");
    ok(CREG(CARD_PINS) & CARD_PIN_POL, "ON polarity 1: the POL pin");

    card.silent = 0;
    s1 = call(SVC_CARD, CARDSVC_ON, 0, 0, 0, 0);
    passes(1000);
    cancel(s1);
    passes(1);
    ok(W(IPC_CANCEL_DONE) == 1u && CREG(CARD_VCC) == 0u,
       "cancel an ON: ended, and powered down");
    passes(1000000);
    ok(!reap(&g), "cancelled ON: no reply");
    s1 = call(SVC_CARD, CARDSVC_STATUS, 0, 0, 0, 0);
    passes(1);
    ok(reap(&g) && g.seq == s1 && g.val == CARDST_OFF, "and the stage is off");

    printf(fails ? "armcore_host: FAIL (%d)\n" : "armcore_host: PASS\n", fails);
    return fails != 0;
}
