/* src/armcore/svc_card.c -- SVC_CARD: the card interface (ISO 7816) on the
 * ARM.  armipc.h has the calls; ../../docs/card.md the block.
 *
 * WHY HERE.  The card is mostly waiting: power-up steps with milliseconds
 * between them, then bytes that trickle in.  On the main FR each wait is a
 * frame the host has to send again, and card frames queue behind the TS
 * frames on EP0 (src/main/ts_dione.c).  Here the waiting runs on its own,
 * and the host asks once and collects one reply.
 *
 * WHAT IS KNOWN
 *   The register order is the one measured from the host's window and then
 *   run from the main FR (src/main/card.c, which this follows step for
 *   step).  The ARM reaches the card block at the main FR's addresses
 *   (docs/arm.md, read measured on a GV-M2TV).
 *
 * WHAT IS NOT
 *   - Whether writes from the ARM reach the block the same way.  Reads did.
 *   - Time: passes of the loop, IPC_PASSES_PER_MS (armipc.h).  Waits are
 *     at least as long as asked; they may be longer.
 *
 * SHAPE
 *   One task at a time, one step per pass, never a wait inside a step (the
 *   loop's rule, armmain.c).  The main FR's card.c and this file must not
 *   both drive the block: the main FR keeps track of who has it
 *   (src/main/arm_ipc.c) and refuses the other.
 */
#include "core.h"
#include "mb86m01.h"

#ifdef ARM_HOSTED
unsigned arm_hosted_rd(unsigned addr);
void arm_hosted_wr(unsigned addr, unsigned val);
#define RD(a)       arm_hosted_rd(a)
#define WR(a, v)    arm_hosted_wr((a), (v))
#else
#define RD(a)       (*(volatile unsigned *)(a))
#define WR(a, v)    (*(volatile unsigned *)(a) = (v))
#endif

#define BUF(off)    DTCM_WORD(ARM_BUF + (off))

#define WAIT_VCC        10u     /* ms for VCC to settle before the clock   */
#define WAIT_CLK        1u      /* ms of clock before RST (ISO: 400 clocks) */
#define POLL_MS         1u      /* ms between looks at the receive count   */
#define ATR_MS_DEF      1000u
#define XFER_MS_DEF     2000u

#define ATR_MAX         33u
#define BLOCK_MAX       259u    /* NAD PCB LEN + 254 + LRC, and one spare  */
#define SEND_MAX        255u    /* under 256: the only size measured       */

#define T_NONE          0u
#define T_ON            1u
#define T_XFER          2u

static unsigned stage, pol, atr_len, rx_len;
static unsigned task, seq, call;
static unsigned wait;           /* passes until the next step              */
static unsigned left;           /* passes until the task times out         */
static unsigned pending;        /* 1 = `out` is waiting for room           */
static struct rep out;

/* --- bytes in the buffer (armipc.h, THE BUFFER) ------------------------ */
static unsigned tx_byte(unsigned i)
{
    return (BUF(BUF_TX + (i & ~3u)) >> (24u - 8u * (i & 3u))) & 0xffu;
}

static unsigned rx_byte(unsigned i)
{
    return (BUF(BUF_RX + (i & ~3u)) >> (24u - 8u * (i & 3u))) & 0xffu;
}

static void rx_put(unsigned i, unsigned c)
{
    unsigned sh = 24u - 8u * (i & 3u);
    unsigned a = BUF_RX + (i & ~3u);

    BUF(a) = (BUF(a) & ~(0xffu << sh)) | (c << sh);
}

static void rx_clear(void)
{
    unsigned i;

    for (i = 0; i < BUF_HALF; i += 4u)
        BUF(BUF_RX + i) = 0;
}

/* No divide instruction on this core, and no libgcc in the image. */
static unsigned udiv(unsigned n, unsigned d)
{
    unsigned q = 0, bit = 1;

    if (d == 0u)
        return 0;
    while (d < n && !(d & 0x80000000u)) {
        d <<= 1;
        bit <<= 1;
    }
    while (bit) {
        if (n >= d) {
            n -= d;
            q |= bit;
        }
        d >>= 1;
        bit >>= 1;
    }
    return q;
}

static unsigned passes(unsigned ms)
{
    return ms * IPC_PASSES_PER_MS;
}

/* --- the block (card.c has the same steps; docs/card.md the table) ----- */
/* Unread bytes outlive a power-down (card.c says more): drop them. */
static void discard(void)
{
    unsigned n = RD(CARD_MODE) >> 16;

    while (n--)
        (void)RD(CARD_RX);
}

static unsigned power_step(unsigned step)
{
    switch (step) {
    case 1:
        WR(CARD_PINS, pol);
        WR(CARD_EN, CARD_EN_ON);
        WR(CARD_BAUD, CARD_FD_ATR << CARD_BAUD_SHIFT);
        WR(CARD_VCC, 1);
        return WAIT_VCC;
    case 2:
        WR(CARD_PINS, pol | CARD_PIN_CLK);
        return WAIT_CLK;
    case 3:
        discard();
        WR(CARD_PINS, pol | CARD_PIN_CLK | CARD_PIN_RST);
        return POLL_MS;
    default:
        return 0;
    }
}

static void power_down(void)
{
    WR(CARD_PINS, pol | CARD_PIN_CLK);
    WR(CARD_PINS, pol);
    WR(CARD_VCC, 0);
    stage = CARDST_OFF;
    atr_len = 0;
    rx_len = 0;
}

/* Take exactly the bytes CARD_MODE says are waiting: one more read would
 * take a byte that has not arrived.  -> 0, or 1 if more than `cap`. */
static int drain(unsigned cap)
{
    unsigned n = RD(CARD_MODE) >> 16;

    while (n--) {
        unsigned c = RD(CARD_RX) & 0xffu;

        if (rx_len >= cap)
            return 1;
        rx_put(rx_len++, c);
    }
    return 0;
}

/* How long the ATR is, once enough of it is here to say.  0 = not yet. */
static unsigned atr_length(void)
{
    unsigned i = 2, y, k, tck = 0, n = 0;

    if (rx_len < 2u)
        return 0;
    y = rx_byte(1) >> 4;
    k = rx_byte(1) & 0x0fu;
    for (;;) {
        if (y & 1u) i++;
        if (y & 2u) i++;
        if (y & 4u) i++;
        if (!(y & 8u))
            break;
        if (rx_len <= i)
            return 0;
        if ((rx_byte(i) & 0x0fu) != 0u)
            tck = 1;
        y = rx_byte(i) >> 4;
        i++;
        if (++n > 8u)
            return ATR_MAX + 1u;
    }
    return i + k + tck;
}

/* --- answers ----------------------------------------------------------- */
static void first_words(unsigned *d)
{
    unsigned i;

    for (i = 0; i < IPC_DATA_WORDS; i++)
        d[i] = BUF(BUF_RX + i * 4u);
}

/* The task's one reply; sent from card_tick when there is room. */
static void finish(unsigned status, unsigned val, unsigned d0)
{
    out.seq = seq;
    out.call = call;
    out.status = status;
    out.val = val;
    first_words(out.data);
    if (status != IPC_ST_OK)
        out.data[0] = d0;
    task = T_NONE;
    pending = 1;
}

static void step_on(void)
{
    unsigned need;

    if (stage < CARDST_STEPS) {
        stage++;
        wait = passes(power_step(stage));
        if (stage == CARDST_STEPS)
            stage = CARDST_ATR;
        return;
    }
    if (drain(ATR_MAX)) {
        finish(IPC_ST_IO, rx_len, 0);
        return;
    }
    need = atr_length();
    if (need > ATR_MAX || (need && rx_len > need)) {
        finish(IPC_ST_IO, rx_len, need);
        return;
    }
    if (need && rx_len == need) {
        atr_len = rx_len;
        stage = CARDST_READY;
        finish(IPC_ST_OK, atr_len, 0);
        return;
    }
    if (--left == 0u) {
        finish(IPC_ST_TIMEOUT, rx_len, need);
        return;
    }
    wait = passes(POLL_MS);
}

static void step_xfer(void)
{
    unsigned need = 0;
    int over = drain(BLOCK_MAX);

    if (rx_len >= 3u)
        need = 3u + rx_byte(2) + 1u;
    if (over || (need && rx_len > need)) {
        stage = CARDST_READY;
        finish(IPC_ST_IO, rx_len, need);
        return;
    }
    if (need && rx_len == need) {
        stage = CARDST_GOT;
        finish(IPC_ST_OK, rx_len, 0);
        return;
    }
    if (--left == 0u) {
        stage = CARDST_READY;
        finish(IPC_ST_TIMEOUT, rx_len, need);
        return;
    }
    wait = passes(POLL_MS);
}

/* The left counter is decremented once per look, so it counts looks, not
 * passes: timeout ms / POLL_MS looks. */
static unsigned looks(unsigned ms)
{
    return ms / POLL_MS + 1u;
}

static void start(const struct req *q, unsigned kind, unsigned ms)
{
    task = kind;
    seq = q->seq;
    call = q->call;
    wait = 0;
    left = looks(ms);
    pending = 0;
}

static int param(const struct req *q)
{
    unsigned fi = q->arg0 >> 16, di = q->arg0 & 0xffffu;
    unsigned cwi = (q->arg1 >> 8) & 0xffu, bwi = q->arg1 & 0xffu, n;

    if (fi == 0u || di == 0u || di > 64u || di > fi || cwi > 15u || bwi > 9u)
        return 0;
    n = udiv(fi, di);
    if (n > 0x1ffu)
        return 0;
    WR(CARD_BAUD, n << CARD_BAUD_SHIFT);
    WR(CARD_CWT, 11u + (1u << cwi));
    WR(CARD_BWT, 11u + (udiv(960u * 372u * di, fi) << bwi));
    return 1;
}

static void send(unsigned n)
{
    unsigned i;

    discard();
    for (i = 0; i < n; i++)
        WR(CARD_TX, tx_byte(i));
    WR(CARD_GO, 1);
    rx_len = 0;
    atr_len = 0;                /* the receive half is reused */
    rx_clear();
    stage = CARDST_SENT;
}

/* --- the service ------------------------------------------------------- */
int card_call(const struct req *q, struct rep *r)
{
    unsigned op = IPC_CALL_OP(q->call);

    if ((op == CARDSVC_ON || op == CARDSVC_XFER) && (task || pending)) {
        r->status = IPC_ST_BUSY;
        return 1;
    }
    switch (op) {
    case CARDSVC_STATUS:
        r->val = stage;
        r->data[0] = RD(CARD_EVENT);
        r->data[1] = RD(CARD_PINS);
        r->data[2] = RD(CARD_MODE);
        r->data[3] = atr_len;
        return 1;
    case CARDSVC_ON:
        if (q->arg0 > 1u) {
            r->status = IPC_ST_ARG;
            return 1;
        }
        if (stage >= CARDST_READY && atr_len) {
            r->val = atr_len;
            first_words(r->data);
            return 1;
        }
        pol = q->arg0 ? CARD_PIN_POL : 0u;
        stage = CARDST_OFF;
        atr_len = 0;
        rx_len = 0;
        rx_clear();
        start(q, T_ON, q->arg1 ? q->arg1 : ATR_MS_DEF);
        return 0;
    case CARDSVC_OFF:
        if (task)               /* val = the stage it had reached */
            finish(IPC_ST_STOPPED, stage, 0);
        power_down();
        return 1;
    case CARDSVC_PARAM:
        if (stage < CARDST_READY) {
            r->status = IPC_ST_STATE;
            return 1;
        }
        if (!param(q))
            r->status = IPC_ST_ARG;
        return 1;
    case CARDSVC_XFER:
        if (stage < CARDST_READY) {
            r->status = IPC_ST_STATE;
            return 1;
        }
        if (q->arg0 == 0u || q->arg0 > SEND_MAX) {
            r->status = IPC_ST_ARG;
            return 1;
        }
        start(q, T_XFER, q->arg1 ? q->arg1 : XFER_MS_DEF);
        send(q->arg0);
        wait = passes(POLL_MS);
        return 0;
    default:
        r->status = IPC_ST_BADOP;
        return 1;
    }
}

/* The cancel words: end the task with no reply.  An ON that is cut off
 * leaves the card half powered, so it is powered down. */
unsigned card_cancel(unsigned what)
{
    if (!task && !pending)
        return 0;
    if (what != IPC_CANCEL_ALL && what != seq)
        return 0;
    if (task == T_ON)
        power_down();
    else if (task == T_XFER)
        stage = CARDST_READY;
    task = T_NONE;
    pending = 0;
    return 1;
}

void card_tick(void)
{
    if (task && !pending) {
        if (wait)
            wait--;
        else if (task == T_ON)
            step_on();
        else
            step_xfer();
    }
    if (pending && ipc_put(&out))
        pending = 0;
}
