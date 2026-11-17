/* test_t1.c -- t1.h against a simulated card.  No hardware.
 *
 *     make test
 *
 * The simulated card speaks T=1 the card's way round: it reassembles
 * chained I-blocks, answers with its own chain, asks for a waiting time
 * extension, and the test damages or drops blocks on the way so the retry
 * paths run.  The answer to an APDU is the APDU reversed plus 90 00.
 * Also checks atr_parse() on the GV-M2TV card's ATR.
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "t1.h"

struct sim {
    unsigned ifsc;              /* what the card takes */
    unsigned ifsd;              /* what the host takes (from S(IFS)) */
    unsigned ns, nr;
    uint8_t cmd[1024];
    size_t ncmd;
    uint8_t resp[1024];
    size_t nresp, rsent;
    uint8_t last[T1_BLOCK_MAX]; /* the card's last block, for resends */
    size_t nlast;
    /* faults */
    int wtx_before_answer;
    int corrupt_at;             /* corrupt the n-th block out, 1-based */
    int drop_at;                /* drop the n-th block out */
    int out_count;
    int blocks_in;
};

static size_t sim_chunk(struct sim *c)
{
    size_t k = c->nresp - c->rsent;

    if (k > c->ifsd)
        k = c->ifsd;
    c->nlast = t1_build(c->last, T1_I(c->ns, c->rsent + k < c->nresp),
                        c->resp + c->rsent, k);
    c->rsent += k;
    return c->nlast;
}

static void sim_answer(struct sim *c)
{
    size_t i;

    c->nresp = 0;
    for (i = 0; i < c->ncmd; i++)
        c->resp[c->nresp++] = c->cmd[c->ncmd - 1 - i];
    c->resp[c->nresp++] = 0x90;
    c->resp[c->nresp++] = 0x00;
    c->rsent = 0;
    c->ncmd = 0;
    sim_chunk(c);
}

/* The card's reaction to one block from the host. */
static void sim_card(struct sim *c, const uint8_t *b, size_t n)
{
    uint8_t pcb = b[1];

    c->blocks_in++;
    if (t1_check(b, n) != 0) {
        c->nlast = t1_build(c->last, T1_R(c->nr, 1u), NULL, 0);
        return;
    }
    if (T1_IS_S(pcb)) {
        if (pcb == T1_S_IFS) {
            c->ifsd = b[3];
            c->nlast = t1_build(c->last, T1_S_IFS | T1_S_RESPONSE, b + 3, 1);
        } else if (pcb == (T1_S_WTX | T1_S_RESPONSE)) {
            sim_answer(c);
        }
        return;
    }
    if (T1_IS_R(pcb)) {
        unsigned nr = (pcb >> 4) & 1u;

        if (c->rsent > 0 && c->rsent < c->nresp && nr != c->ns) {
            c->ns ^= 1u;                /* our block was taken */
            sim_chunk(c);
        }
        /* else: resend c->last as it is */
        return;
    }
    /* I-block */
    if (((pcb >> 6) & 1u) != c->nr) {
        c->nlast = t1_build(c->last, T1_R(c->nr, 2u), NULL, 0);
        return;
    }
    if (c->rsent > 0) {                 /* host's new APDU acks our last */
        c->ns ^= 1u;
        c->rsent = 0;
        c->nresp = 0;
    }
    c->nr ^= 1u;
    memcpy(c->cmd + c->ncmd, b + 3, b[2]);
    c->ncmd += b[2];
    if (pcb & 0x20u) {
        c->nlast = t1_build(c->last, T1_R(c->nr, 0u), NULL, 0);
        return;
    }
    if (c->wtx_before_answer) {
        uint8_t m = 3;

        c->wtx_before_answer = 0;
        c->nlast = t1_build(c->last, T1_S_WTX, &m, 1);
        return;
    }
    sim_answer(c);
}

static int sim_xfer(void *ctx, const uint8_t *tx, size_t ntx, uint8_t *rx,
                    size_t *nrx, unsigned wait_ms)
{
    struct sim *c = (struct sim *)ctx;

    (void)wait_ms;
    sim_card(c, tx, ntx);
    c->out_count++;
    if (c->out_count == c->drop_at)
        return T1_XFER_TIMEOUT;
    memcpy(rx, c->last, c->nlast);
    *nrx = c->nlast;
    if (c->out_count == c->corrupt_at)
        rx[*nrx - 1] ^= 0x55u;
    return T1_XFER_OK;
}

static int run(struct t1 *t, size_t n)
{
    uint8_t apdu[600], resp[1024];
    size_t nresp, i;

    for (i = 0; i < n; i++)
        apdu[i] = (uint8_t)(i * 7u + 1u);
    if (t1_transceive(t, apdu, n, resp, sizeof(resp), &nresp) != 0) {
        printf("  transceive %zu: %s\n", n, t->err);
        return 1;
    }
    if (nresp != n + 2u || resp[n] != 0x90 || resp[n + 1] != 0x00)
        return printf("  transceive %zu: bad length %zu\n", n, nresp), 1;
    for (i = 0; i < n; i++)
        if (resp[i] != apdu[n - 1 - i])
            return printf("  transceive %zu: byte %zu\n", n, i), 1;
    return 0;
}

static void test_atr(void)
{
    static const uint8_t m2tv[] = {
        0x3B, 0xF0, 0x12, 0x00, 0xFF, 0x91, 0x81, 0xB1, 0x7C, 0x45, 0x1F,
        0x03, 0x99
    };
    static const uint8_t t0only[] = { 0x3B, 0x02, 0x14, 0x50 };
    /* T=1 with TC3 = 01: CRC.  The handler refuses it at power-up. */
    static const uint8_t t1crc[] = { 0x3B, 0x80, 0x81, 0x41, 0x01, 0x41 };
    struct atr_info ai;
    uint8_t bad[sizeof(m2tv)];

    assert(atr_parse(m2tv, sizeof(m2tv), &ai) == NULL);
    assert(ai.fi == 372 && ai.di == 2);
    assert(ai.ifsc == 124 && ai.cwi == 5 && ai.bwi == 4);
    assert(ai.specific && (ai.ta2 & 0x0Fu) == 1u && !ai.crc);
    assert(ai.protocols == ((1u << 1) | (1u << 15)));
    assert(atr_parse(m2tv, sizeof(m2tv) - 1, &ai) != NULL);
    memcpy(bad, m2tv, sizeof(m2tv));
    bad[sizeof(bad) - 1] ^= 1u;
    assert(atr_parse(bad, sizeof(bad), &ai) != NULL);
    assert(atr_parse(t0only, sizeof(t0only), &ai) == NULL);
    assert(ai.protocols == 1u && ai.fi == 372 && ai.di == 1);
    assert(atr_parse(t1crc, sizeof(t1crc), &ai) == NULL);
    assert(ai.crc && (ai.protocols & 2u));
    printf("atr: ok (T=1 LRC; T=0-only and CRC are flagged)\n");
}

int main(void)
{
    static const struct {
        const char *name;
        size_t n;
        unsigned ifsd;
        int wtx, corrupt, drop;
    } cases[] = {
        { "short",                 5, 251, 0, 0, 0 },
        { "chain out",            40, 251, 0, 0, 0 },
        { "chain both ways",     300,  32, 0, 0, 0 },
        { "wtx",                  10, 251, 1, 0, 0 },
        { "corrupt first answer", 10, 251, 0, 2, 0 },
        { "drop in chain",       100,  32, 0, 0, 4 },
        { "corrupt in chain",    100,  32, 0, 5, 0 },
    };
    struct atr_info ai;
    unsigned i;
    int bad = 0;

    test_atr();
    memset(&ai, 0, sizeof(ai));
    for (i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        struct sim c;
        struct t1 t;
        int r;

        memset(&c, 0, sizeof(c));
        c.ifsc = 16;
        c.ifsd = 32;
        ai.ifsc = c.ifsc;
        ai.bwi = 4;
        t1_init(&t, sim_xfer, &c, &ai, cases[i].ifsd);
        r = t1_set_ifsd(&t);
        if (r != T1_XFER_OK) {
            printf("%s: S(IFS) %s\n", cases[i].name, t.err);
            bad++;
            continue;
        }
        c.out_count = 0;
        c.wtx_before_answer = cases[i].wtx;
        c.corrupt_at = cases[i].corrupt;
        c.drop_at = cases[i].drop;
        r = run(&t, cases[i].n);
        /* A second APDU on the same state checks the sequence bits. */
        c.corrupt_at = c.drop_at = 0;
        r |= run(&t, 7);
        printf("%s: %s (%d blocks)\n", cases[i].name, r ? "FAIL" : "ok",
               c.blocks_in);
        bad += r;
    }
    printf(bad ? "FAILED\n" : "all ok\n");
    return bad ? 1 : 0;
}
