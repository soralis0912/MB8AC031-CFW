/* t1.h -- ATR parsing and the T=1 block protocol (ISO/IEC 7816-3), host
 * side.  Header-only, no dependencies, no I/O of its own.
 *
 * The firmware moves bytes and nothing else (../../docs/card.md): it sends
 * one block the host built, LRC included, and hands back one block.  Every
 * rule above that -- sequence numbers, chaining, retransmission, waiting
 * time extension, IFS -- lives here.  The transport is a callback, so the
 * same code runs against the real card (card_link.h) and against the
 * simulated card in test_t1.c.
 *
 * Scope: T=1 with LRC.  CRC (TC3 bit 0) is refused at ATR time rather than
 * half-supported.  NAD is always 0.
 */
#ifndef T1_H
#define T1_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* --- ATR --------------------------------------------------------------- */

#define ATR_MAX         33u

struct atr_info {
    unsigned fi, di;            /* F and D to run at (372, 1 if unknown)   */
    unsigned ifsc;              /* TA3 for T=1, else 32                    */
    unsigned cwi, bwi;          /* TB3 for T=1, else 13 and 4              */
    unsigned protocols;         /* bit n = T=n offered                     */
    int specific;               /* TA2 present: no PPS, run as told        */
    unsigned ta2;               /* TA2 itself (bit 4 = implicit params)    */
    int crc;                    /* TC3 bit0: T=1 uses CRC                  */
    unsigned hist_len;
};

static const unsigned t1_fi_table[16] = {
    372, 372, 558, 744, 1116, 1488, 1860, 0, 0, 512, 768, 1024, 1536, 2048,
    0, 0
};
static const unsigned t1_di_table[16] = {
    0, 1, 2, 4, 8, 16, 32, 64, 12, 20, 0, 0, 0, 0, 0, 0
};

/* Parse an ATR.  Returns NULL when it is good, else why not.  The length
 * must be exact: the firmware already cut it at the end the TDi chain says. */
static inline const char *atr_parse(const uint8_t *a, size_t n,
                                    struct atr_info *out)
{
    size_t i = 2;
    unsigned y, k, level = 1, ta1 = 0x11u, t1_level = 0;
    int have_ta1 = 0, need_tck = 0;

    memset(out, 0, sizeof(*out));
    out->fi = 372;
    out->di = 1;
    out->ifsc = 32;
    out->cwi = 13;
    out->bwi = 4;

    if (n < 2u)
        return "ATR shorter than TS T0";
    if (a[0] != 0x3Bu && a[0] != 0x3Fu)
        return "TS is neither 3B nor 3F";
    y = a[1] >> 4;
    k = a[1] & 0x0Fu;
    out->hist_len = k;
    for (;;) {
        unsigned td = 0;
        int have_td = 0, spec = level == t1_level;

        if (y & 1u) {                   /* TAi */
            if (i >= n)
                return "ATR truncated in TA";
            if (level == 1u) {
                ta1 = a[i];
                have_ta1 = 1;
            } else if (level == 2u) {
                out->specific = 1;
                out->ta2 = a[i];
            } else if (spec) {
                out->ifsc = a[i];
            }
            i++;
        }
        if (y & 2u) {                   /* TBi */
            if (i >= n)
                return "ATR truncated in TB";
            if (spec) {
                out->cwi = a[i] & 0x0Fu;
                out->bwi = a[i] >> 4;
            }
            i++;
        }
        if (y & 4u) {                   /* TCi */
            if (i >= n)
                return "ATR truncated in TC";
            if (spec)
                out->crc = a[i] & 1;
            i++;
        }
        if (y & 8u) {                   /* TDi */
            if (i >= n)
                return "ATR truncated in TD";
            td = a[i++];
            have_td = 1;
        }
        if (!have_td)
            break;
        out->protocols |= 1u << (td & 0x0Fu);
        if ((td & 0x0Fu) != 0u)
            need_tck = 1;
        /* The T=1 specific bytes are TA/TB/TCi+1 after the first TDi
         * (i >= 2) that names T=1. */
        if ((td & 0x0Fu) == 1u && level >= 2u && t1_level == 0u)
            t1_level = level + 1u;
        y = td >> 4;
        level++;
    }
    if (out->protocols == 0u)
        out->protocols = 1u;            /* no TD1: T=0 only */
    if (i + k + (need_tck ? 1u : 0u) != n)
        return "ATR length disagrees with T0 and TDi";
    if (need_tck) {
        uint8_t x = 0;
        size_t j;

        for (j = 1; j < n; j++)
            x ^= a[j];
        if (x != 0u)
            return "TCK check failed";
    }
    /* TA1 applies when TA2 says so (specific mode, bit 5 clear); with no
     * TA2 it would need a PPS, which this reader does not do, so it runs at
     * the default F and D. */
    if (have_ta1 && out->specific && !(out->ta2 & 0x10u)) {
        unsigned f = t1_fi_table[ta1 >> 4], d = t1_di_table[ta1 & 0x0Fu];

        if (f == 0u || d == 0u)
            return "TA1 names a reserved F or D";
        out->fi = f;
        out->di = d;
    }
    if (out->ifsc == 0u || out->ifsc == 255u)
        return "TA3 names an invalid IFSC";
    return NULL;
}

/* --- T=1 --------------------------------------------------------------- */

#define T1_INF_MAX      254u
#define T1_BLOCK_MAX    (3u + T1_INF_MAX + 1u)
#define T1_RETRIES      3

/* PCB */
#define T1_I(ns, more)  (uint8_t)(((ns) ? 0x40u : 0u) | ((more) ? 0x20u : 0u))
#define T1_R(nr, err)   (uint8_t)(0x80u | ((nr) ? 0x10u : 0u) | (err))
#define T1_S_RESYNCH    0xC0u
#define T1_S_IFS        0xC1u
#define T1_S_ABORT      0xC2u
#define T1_S_WTX        0xC3u
#define T1_S_RESPONSE   0x20u

#define T1_IS_I(p)      (((p) & 0x80u) == 0u)
#define T1_IS_R(p)      (((p) & 0xC0u) == 0x80u)
#define T1_IS_S(p)      (((p) & 0xC0u) == 0xC0u)

/* What the transport returns. */
#define T1_XFER_OK       0
#define T1_XFER_TIMEOUT (-1)    /* no whole block within the time given   */
#define T1_XFER_IO      (-2)    /* transport broke: give up at once       */

/* Send `tx` (a whole block, LRC on), collect one whole block into `rx`.
 * `wait_ms` is how long the card may take; the transport decides how to
 * poll.  A block that arrived damaged is still returned OK: checking it is
 * this file's job. */
typedef int (*t1_xfer_fn)(void *ctx, const uint8_t *tx, size_t ntx,
                          uint8_t *rx, size_t *nrx, unsigned wait_ms);

struct t1 {
    t1_xfer_fn xfer;
    void *ctx;
    unsigned ns;                /* our next I-block's N(S)                */
    unsigned nr;                /* the N(S) we expect from the card       */
    unsigned ifsc;              /* what the card takes                    */
    unsigned ifsd;              /* what we take                           */
    unsigned bwt_ms;
    char err[96];
};

/* Block waiting time.  BWT = 11 etu + 2^BWI x 960 x 372 / f.  The card
 * clock of a given board is not measured (docs/card.md), so f is taken as
 * low as 1 MHz-ish (357 ms per BWI unit) rather than the usual 3.57 MHz:
 * waiting too long only costs time on a dead card, too short breaks a slow
 * one. */
static inline unsigned t1_bwt_ms(unsigned bwi)
{
    if (bwi > 9u)
        bwi = 9u;
    return (1u << bwi) * 360u + 500u;
}

static inline void t1_init(struct t1 *t, t1_xfer_fn xfer, void *ctx,
                           const struct atr_info *ai, unsigned ifsd)
{
    memset(t, 0, sizeof(*t));
    t->xfer = xfer;
    t->ctx = ctx;
    t->ifsc = ai->ifsc;
    t->ifsd = ifsd;
    t->bwt_ms = t1_bwt_ms(ai->bwi);
}

static inline size_t t1_build(uint8_t *b, uint8_t pcb, const uint8_t *inf,
                              size_t n)
{
    uint8_t x;
    size_t i;

    b[0] = 0;
    b[1] = pcb;
    b[2] = (uint8_t)n;
    if (n)
        memcpy(b + 3, inf, n);
    x = 0;
    for (i = 0; i < 3u + n; i++)
        x ^= b[i];
    b[3 + n] = x;
    return 4u + n;
}

/* 0 when `b` is one well-formed block. */
static inline int t1_check(const uint8_t *b, size_t n)
{
    uint8_t x = 0;
    size_t i;

    if (n < 4u || b[0] != 0u || b[2] == 255u || n != 4u + b[2])
        return -1;
    for (i = 0; i < n; i++)
        x ^= b[i];
    return x == 0u ? 0 : -1;
}

/* One exchange with the retry rule: a damaged or missing answer earns an
 * R-block (or, if `tx` was not answered at all the first time, a resend
 * of `tx` itself is what ISO 7816-3 11.6.3.2 rule 7.1 allows only for
 * S(RESYNCH); everything else asks with R).  Returns T1_XFER_OK with a
 * good block in rx, or a failure after T1_RETRIES. */
static inline int t1_exchange(struct t1 *t, const uint8_t *tx, size_t ntx,
                              uint8_t *rx, size_t *nrx, unsigned wait_ms)
{
    uint8_t rb[4];
    const uint8_t *out = tx;
    size_t nout = ntx;
    int tries;

    for (tries = 0; tries <= T1_RETRIES; tries++) {
        int rc = t->xfer(t->ctx, out, nout, rx, nrx, wait_ms);

        if (rc == T1_XFER_IO) {
            snprintf(t->err, sizeof(t->err), "transport failed");
            return rc;
        }
        if (rc == T1_XFER_OK && t1_check(rx, *nrx) == 0)
            return T1_XFER_OK;
        /* Ask again: R(N(R) = what we expect, error = 1 EDC / 2 other). */
        nout = t1_build(rb, T1_R(t->nr, rc == T1_XFER_OK ? 1u : 2u),
                        NULL, 0);
        out = rb;
    }
    snprintf(t->err, sizeof(t->err), "no valid block after %d retries",
             T1_RETRIES);
    return T1_XFER_TIMEOUT;
}

/* Answer an S request in place.  Returns 1 if it was one we answer (WTX
 * and IFS), with *wait_ms adjusted for WTX; 0 for anything else. */
static inline int t1_s_answer(struct t1 *t, const uint8_t *rx, uint8_t *tx,
                              size_t *ntx, unsigned *wait_ms)
{
    uint8_t pcb = rx[1];

    if (pcb == T1_S_WTX && rx[2] == 1u) {
        unsigned m = rx[3] ? rx[3] : 1u;

        *ntx = t1_build(tx, T1_S_WTX | T1_S_RESPONSE, rx + 3, 1);
        *wait_ms = t->bwt_ms * m;
        return 1;
    }
    if (pcb == T1_S_IFS && rx[2] == 1u && rx[3] != 0u && rx[3] != 255u) {
        t->ifsc = rx[3];
        *ntx = t1_build(tx, T1_S_IFS | T1_S_RESPONSE, rx + 3, 1);
        *wait_ms = t->bwt_ms;
        return 1;
    }
    return 0;
}

/* S(IFS request) for our IFSD.  Card must echo the value. */
static inline int t1_set_ifsd(struct t1 *t)
{
    uint8_t tx[T1_BLOCK_MAX], rx[T1_BLOCK_MAX];
    uint8_t v = (uint8_t)t->ifsd;
    size_t ntx, nrx;
    int rc;

    ntx = t1_build(tx, T1_S_IFS, &v, 1);
    rc = t1_exchange(t, tx, ntx, rx, &nrx, t->bwt_ms);
    if (rc != T1_XFER_OK)
        return rc;
    if (rx[1] != (T1_S_IFS | T1_S_RESPONSE) || rx[2] != 1u || rx[3] != v) {
        snprintf(t->err, sizeof(t->err), "S(IFS) answered with PCB %02X",
                 rx[1]);
        return T1_XFER_TIMEOUT;
    }
    return T1_XFER_OK;
}

/* Send an APDU, collect the response.  0 on success. */
static inline int t1_transceive(struct t1 *t, const uint8_t *apdu,
                                size_t napdu, uint8_t *resp, size_t cap,
                                size_t *nresp)
{
    uint8_t tx[T1_BLOCK_MAX], rx[T1_BLOCK_MAX];
    size_t ntx, nrx, sent = 0, got = 0;
    unsigned wait = t->bwt_ms;
    int sending = 1, guard = 0;

    *nresp = 0;
    if (napdu == 0u) {
        snprintf(t->err, sizeof(t->err), "empty APDU");
        return -1;
    }

    /* First chunk. */
    {
        size_t k = napdu < t->ifsc ? napdu : t->ifsc;

        ntx = t1_build(tx, T1_I(t->ns, k < napdu), apdu, k);
        sent = k;
    }

    for (;;) {
        int rc;
        uint8_t pcb;

        if (++guard > 1024) {
            snprintf(t->err, sizeof(t->err), "too many blocks");
            return -1;
        }
        rc = t1_exchange(t, tx, ntx, rx, &nrx, wait);
        if (rc != T1_XFER_OK)
            return -1;
        wait = t->bwt_ms;
        pcb = rx[1];

        if (T1_IS_S(pcb)) {
            if (t1_s_answer(t, rx, tx, &ntx, &wait))
                continue;
            snprintf(t->err, sizeof(t->err), "card sent S-block %02X",
                     pcb);
            return -1;
        }

        if (T1_IS_R(pcb)) {
            unsigned nr = (pcb >> 4) & 1u;

            if (sending && sent < napdu && nr != t->ns) {
                /* Our chained block is acknowledged: next chunk. */
                size_t k = napdu - sent;

                t->ns ^= 1u;
                if (k > t->ifsc)
                    k = t->ifsc;
                ntx = t1_build(tx, T1_I(t->ns, sent + k < napdu),
                               apdu + sent, k);
                sent += k;
                continue;
            }
            /* Otherwise the card wants the last block again: tx still
             * holds it.  (If tx was an R of ours, it is resent too.) */
            continue;
        }

        /* I-block. */
        if (((pcb >> 6) & 1u) != t->nr) {
            /* Not the sequence we expect: ask for the right one. */
            ntx = t1_build(tx, T1_R(t->nr, 2u), NULL, 0);
            continue;
        }
        if (sending) {
            if (sent < napdu) {
                snprintf(t->err, sizeof(t->err),
                         "card answered before the chain ended");
                return -1;
            }
            t->ns ^= 1u;                /* our last block is acknowledged */
            sending = 0;
        }
        t->nr ^= 1u;
        if (got + rx[2] > cap) {
            snprintf(t->err, sizeof(t->err), "response longer than %zu",
                     cap);
            return -1;
        }
        memcpy(resp + got, rx + 3, rx[2]);
        got += rx[2];
        if (pcb & 0x20u) {              /* more: acknowledge and go on */
            ntx = t1_build(tx, T1_R(t->nr, 0u), NULL, 0);
            continue;
        }
        *nresp = got;
        return 0;
    }
}

#endif
