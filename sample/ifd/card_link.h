/* card_link.h -- the firmware's card frames (band 0x50..0x57) over the
 * 0xBC window.  Header-only; sits on ../chipio.h and ../frq1.h.
 *
 * ../../docs/card.md is the specification.  The firmware has no clock, so
 * every wait is the host's: CARD_ON and CARD_RECV answer FW_EAGAIN until
 * they are done, and this file sleeps and asks again, up to a deadline of
 * its own.
 *
 * card_xfer() has the t1_xfer_fn shape (t1.h), so the T=1 engine runs on
 * the real card through it.
 *
 * WHICH MACHINE.  CFW_CARD_DEV=m2tv or dt295 picks one (../cfwdev.h);
 * without it the first of the table that opens is used.  This uses the
 * SECOND mailbox (0x000C5040 on the GV-M2TV, 0x003B0040 on the PIX-DT295):
 * a frame takes several control transfers, and ts_grab uses
 * the first one, so with one mailbox each the card can be used while a
 * stream is recorded.  A CFW older than the mailboxes does not answer
 * there (the reply is our own request, or zeros: CARD_SILENT).
 *
 * THE ARM PATH.  With CFW_CARD_ARM (or the older name M2TV_CARD_ARM) =0
 * (ARM-E) or 1 (ARM-C) in the
 * environment, the same functions drive the card through our ARM image
 * instead (../../src/armcore/svc_card.c): one ARM_CALL, the waiting done
 * on the ARM, one reply collected with ARM_REAP, the bytes moved with
 * ARM_PUT / ARM_GET.  The image has to be running (../arm_probe hold,
 * image, run), and this program has to be the only one using that core's
 * queue: replies for other requests are dropped here.
 */
#ifndef CARD_LINK_H
#define CARD_LINK_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "../chipio.h"
#include "../frq1.h"
#include "../cfwdev.h"
#include "t1.h"
#include "../../src/shared/armipc.h"

#define CARD_STAGING        (l->dev->card_box)
#define CARD_HOOK           (l->dev->hook)
#define CARD_ENTRY          0x000E1000u /* frc1_entry, both loaders (frc1.md) */

/* Report offsets inside the data field (docs/card.md). */
#define CARD_RP_STAGE       0x01u
#define CARD_RP_WAIT        0x02u
#define CARD_RP_ATRLEN      0x03u
#define CARD_RP_EVENT       0x04u
#define CARD_RP_PINS        0x08u
#define CARD_RP_MODE        0x0Cu
#define CARD_RP_ATR         0x10u
#define CARD_RP_ATR_INLINE  24u
#define CARD_RV_GOT         0x02u
#define CARD_RV_NEED        0x04u
#define CARD_RV_BYTES       0x08u
#define CARD_RV_INLINE      32u

#define CARD_EVENT_PRESENT  0x400u  /* bit 10; see docs/card.md */

/* card_call() results below zero that are not firmware statuses. */
#define CARD_XPORT          (-100)  /* the USB transfer failed            */
#define CARD_SILENT         (-101)  /* nothing of ours answered            */

struct card_link {
    struct chipio io;
    const struct cfwdev *dev;   /* which machine (../cfwdev.h)        */
    int open;
    uint32_t seq;
    int arm;                    /* -1: card.c's frames; else the ARM core */
    uint32_t arm_st;            /* IPC_ST_* of the last reply, ~0 if none */
    char err[256];              /* room for chipio's 192 and a prefix */
};

static inline unsigned card_get16(const uint8_t *b, unsigned off)
{
    return ((unsigned)b[off] << 8) | b[off + 1];
}

static inline uint64_t card_now_ms(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000u + (uint64_t)ts.tv_nsec / 1000000u;
}

/* CFW_CARD_ARM, or the older M2TV_CARD_ARM; NULL when neither is set. */
static inline const char *card_arm_env(void)
{
    const char *v = getenv("CFW_CARD_ARM");

    return v != NULL ? v : getenv("M2TV_CARD_ARM");
}

static inline int card_open(struct card_link *l)
{
    uint32_t seq = l->seq;
    const char *arm = card_arm_env();
    const char *want = getenv("CFW_CARD_DEV");
    unsigned i;

    memset(l, 0, sizeof(*l));
    l->seq = seq ? seq : (uint32_t)card_now_ms();
    l->arm = -1;
    if (arm != NULL && (strcmp(arm, "0") == 0 || strcmp(arm, "1") == 0))
        l->arm = arm[0] - '0';
    if (want != NULL && cfwdev_find(want) == NULL) {
        snprintf(l->err, sizeof(l->err), "CFW_CARD_DEV=%s: m2tv or dt295",
                 want);
        return -1;
    }
    for (i = 0; i < CFWDEV_COUNT; i++) {
        const struct cfwdev *d = &cfwdevs[i];

        if (want != NULL && strcmp(want, d->name) != 0)
            continue;
        if (chipio_open(&l->io, d->vid, d->pid) == 0) {
            l->dev = d;
            l->open = 1;
            return 0;
        }
        snprintf(l->err, sizeof(l->err), "%s: %s", d->name, l->io.err);
    }
    return -1;
}

static inline void card_close(struct card_link *l)
{
    if (l->open)
        chipio_close(&l->io);
    l->open = 0;
}

/* Is a stub of ours in the hook?  Read only: it decides whether a frame may
 * be sent at all.  Without CFW the 0xC1 ring lands in the resident loader,
 * and while frc1_load is half way through writing the hook a ring could
 * land in neither (frc1.md, "the order of installing").  The stub's form is
 * ldi:20 #entry, r12 / jmp @r12 / nop, and the entry must be ours: other
 * firmware hooked the same way (one measured jumps to 0x000E14FC)
 * keeps other data where our mailboxes are, and a frame written there
 * breaks it.  1 hooked, 0 not, -1 the read
 * failed. */
static inline int card_hooked(struct card_link *l)
{
    uint8_t h[8];

    if (chipio_read(&l->io, CARD_HOOK, h, sizeof(h)) != 0) {
        snprintf(l->err, sizeof(l->err), "read hook: %s", l->io.err);
        return -1;
    }
    if (h[0] == 0x9Bu && h[1] == (uint8_t)(((CARD_ENTRY >> 16) << 4) | 12u) &&
        h[2] == (uint8_t)(CARD_ENTRY >> 8) && h[3] == (uint8_t)CARD_ENTRY &&
        h[4] == 0x97u && h[5] == 0x0Cu && h[6] == 0x9Fu && h[7] == 0xA0u)
        return 1;
    snprintf(l->err, sizeof(l->err),
             "CFW is not hooked at 0x%08X (run frc1_load install)",
             CARD_HOOK);
    return 0;
}

/* One frame.  Returns the firmware's status (FRQ1_ST_OK, FRQ1_E_*), or
 * CARD_XPORT / CARD_SILENT.  `data` is the 40-byte data field of the
 * reply (may be NULL). */
static inline int card_call(struct card_link *l, uint32_t cmd, uint32_t a0,
                            uint32_t a1, const uint8_t *in, size_t nin,
                            uint8_t *data)
{
    uint8_t req[FRQ1_BYTES], rep[FRQ1_BYTES];
    const char *why;

    l->seq++;
    frq1_build(req, l->seq, cmd, a0, a1, in, nin);
    if (chipio_dispatch(&l->io, CARD_STAGING, req, rep) != 0) {
        snprintf(l->err, sizeof(l->err), "%s: %s", frq1_command_name(cmd),
                 l->io.err);
        return CARD_XPORT;
    }
    why = frq1_reply_check(rep, l->seq, cmd);
    if (why != NULL) {
        snprintf(l->err, sizeof(l->err), "%s: %s (is CFW installed?)",
                 frq1_command_name(cmd), why);
        return CARD_SILENT;
    }
    if (data != NULL)
        memcpy(data, rep + FRQ1_FR_DATA, FRQ1_DATA_BYTES);
    return frq1_get32s(rep, FRQ1_FR_STATUS);
}

static inline const char *card_rc_name(int rc)
{
    if (rc == CARD_XPORT)
        return "transport failed";
    if (rc == CARD_SILENT)
        return "CFW did not answer";
    return frq1_status_name(rc);
}

/* --- the ARM path ------------------------------------------------------ */

/* An IPC_ST_* as the FRQ1 status the callers already handle. */
static inline int card_arm_st(uint32_t st)
{
    switch (st) {
    case IPC_ST_OK:         return FRQ1_ST_OK;
    case IPC_ST_TIMEOUT:    return FRQ1_E_TIMEOUT;
    case IPC_ST_ARG:        return FRQ1_E_INVAL;
    case IPC_ST_BUSY:
    case IPC_ST_STATE:      return FRQ1_E_AGAIN;
    default:                return FRQ1_E_IO;
    }
}

/* Queue one SVC_CARD request and collect its reply within `limit_ms`.
 * -> FRQ1 status of the frames (FRQ1_E_IO: the image is not listening) or,
 * once the reply is in, card_arm_st of it.  *val and d[4] from the reply.
 * If no reply comes, the task is cancelled (the cancel words). */
static inline int card_arm_ask(struct card_link *l, unsigned op, uint32_t a0,
                               uint32_t a1, unsigned limit_ms, uint32_t *val,
                               uint32_t d[4])
{
    uint8_t in[20] = {0}, r[FRQ1_DATA_BYTES];
    uint32_t seq;
    uint64_t end;
    int rc;
    unsigned i;

    frq1_put32(in, 0, a1);
    rc = card_call(l, FRQ1_CMD_ARM_CALL,
                   (uint32_t)l->arm | (IPC_CALL(SVC_CARD, op) << 8), a0, in,
                   sizeof in, r);
    if (rc != FRQ1_ST_OK) {
        snprintf(l->err, sizeof(l->err), "ARM_CALL: %s%s", card_rc_name(rc),
                 rc == FRQ1_E_IO ? " (is the ARM image running?)" : "");
        return rc;
    }
    seq = frq1_get32(r, 4);
    end = card_now_ms() + limit_ms;
    for (;;) {
        rc = card_call(l, FRQ1_CMD_ARM_REAP, (uint32_t)l->arm, 0, NULL, 0, r);
        if (rc == FRQ1_ST_OK) {
            if (frq1_get32(r, 4) != seq || (frq1_get32(r, 8) & IPC_EVENT))
                continue;               /* not ours */
            if (val)
                *val = frq1_get32(r, 16);
            if (d)
                for (i = 0; i < 4; i++)
                    d[i] = frq1_get32(r, 20u + i * 4u);
            l->arm_st = frq1_get32(r, 12);
            rc = card_arm_st(l->arm_st);
            if (rc != FRQ1_ST_OK)
                snprintf(l->err, sizeof(l->err), "ARM card op %u: status %u",
                         op, (unsigned)frq1_get32(r, 12));
            return rc;
        }
        if (rc != FRQ1_E_AGAIN) {
            snprintf(l->err, sizeof(l->err), "ARM_REAP: %s", card_rc_name(rc));
            return rc;
        }
        if (card_now_ms() > end) {
            card_call(l, FRQ1_CMD_ARM_CANCEL, (uint32_t)l->arm, seq, NULL, 0,
                      NULL);
            snprintf(l->err, sizeof(l->err),
                     "ARM card op %u: no reply in %u ms (cancelled)", op,
                     limit_ms);
            return FRQ1_E_TIMEOUT;
        }
        chipio_nap(2);
    }
}

/* n bytes of BUF_RX into b.  The reply already carried the first 16 (w,
 * the 4 data words), so ARM_GET only fetches the rest: one frame less,
 * and a frame is what waits behind the TS frames on EP0. */
static inline int card_arm_get(struct card_link *l, uint8_t *b, size_t n,
                               const uint32_t w[4])
{
    uint8_t d[FRQ1_DATA_BYTES];
    size_t off, i;
    int rc;

    for (i = 0; i < n && i < 16u; i++)
        b[i] = (uint8_t)(w[i / 4u] >> (24u - 8u * (i % 4u)));
    for (off = 16u; off < n; off += FRQ1_DATA_BYTES) {
        size_t k = n - off < FRQ1_DATA_BYTES ? n - off : FRQ1_DATA_BYTES;

        rc = card_call(l, FRQ1_CMD_ARM_GET, (uint32_t)l->arm, (uint32_t)off,
                       NULL, 0, d);
        if (rc != FRQ1_ST_OK) {
            snprintf(l->err, sizeof(l->err), "ARM_GET: %s", card_rc_name(rc));
            return rc;
        }
        memcpy(b + off, d, k);
    }
    return FRQ1_ST_OK;
}

static inline int card_arm_xfer(struct card_link *l, const uint8_t *tx,
                                size_t ntx, uint8_t *rx, size_t *nrx,
                                unsigned wait_ms)
{
    size_t off;
    uint32_t got = 0, d[4];
    int rc;

    l->arm_st = ~0u;
    *nrx = 0;
    for (off = 0; off < ntx; off += FRQ1_DATA_BYTES) {
        size_t k = ntx - off < FRQ1_DATA_BYTES ? ntx - off : FRQ1_DATA_BYTES;

        rc = card_call(l, FRQ1_CMD_ARM_PUT, (uint32_t)l->arm | (uint32_t)k << 8,
                       (uint32_t)off, tx + off, k, NULL);
        if (rc != FRQ1_ST_OK) {
            snprintf(l->err, sizeof(l->err), "ARM_PUT: %s", card_rc_name(rc));
            return T1_XFER_IO;
        }
    }
    rc = card_arm_ask(l, CARDSVC_XFER, (uint32_t)ntx, wait_ms, wait_ms + 1000u,
                      &got, d);
    if (rc == FRQ1_E_TIMEOUT || l->arm_st == IPC_ST_IO)
        return T1_XFER_TIMEOUT;         /* the card: silent or garbled */
    if (rc != FRQ1_ST_OK)
        return T1_XFER_IO;
    if (got < 4u || got > T1_BLOCK_MAX) {
        snprintf(l->err, sizeof(l->err), "ARM card XFER: %u bytes",
                 (unsigned)got);
        return T1_XFER_TIMEOUT;
    }
    if (card_arm_get(l, rx, got, d) != FRQ1_ST_OK)
        return T1_XFER_IO;
    *nrx = got;
    return T1_XFER_OK;
}

/* CARD_STATUS: the stage and CARD_EVENT, read only. */
static inline int card_status(struct card_link *l, unsigned *stage,
                              uint32_t *event)
{
    uint8_t d[FRQ1_DATA_BYTES];
    int rc;

    if (l->arm >= 0) {
        uint32_t st = 0, w[4] = {0};

        rc = card_arm_ask(l, CARDSVC_STATUS, 0, 0, 1000u, &st, w);
        if (rc == FRQ1_ST_OK) {
            if (stage)
                *stage = st;
            if (event)
                *event = w[0];
        }
        return rc;
    }
    rc = card_call(l, FRQ1_CMD_CARD_STATUS, 0, 0, NULL, 0, d);
    if (rc == FRQ1_ST_OK) {
        if (stage)
            *stage = d[CARD_RP_STAGE];
        if (event)
            *event = frq1_get32(d, CARD_RP_EVENT);
    }
    return rc;
}

static inline int card_off(struct card_link *l)
{
    if (l->arm >= 0)
        return card_arm_ask(l, CARDSVC_OFF, 0, 0, 1000u, NULL, NULL);
    return card_call(l, FRQ1_CMD_CARD_OFF, 0, 0, NULL, 0, NULL);
}

/* CARD_ON until the ATR is in, or `limit_ms` passes.  The ATR goes into
 * atr[ATR_MAX]. */
static inline int card_on(struct card_link *l, uint8_t *atr, size_t *natr,
                          unsigned limit_ms)
{
    uint64_t end = card_now_ms() + limit_ms;
    uint8_t d[FRQ1_DATA_BYTES];
    int rc;

    *natr = 0;
    if (l->arm >= 0) {      /* polarity: the firmware takes the row's */
        uint32_t n = 0, w[4];

        rc = card_arm_ask(l, CARDSVC_ON, 0, limit_ms, limit_ms + 1000u, &n,
                          w);
        if (rc != FRQ1_ST_OK)
            return rc;
        if (n == 0u || n > ATR_MAX) {
            snprintf(l->err, sizeof(l->err), "ARM card ON: ATR length %u",
                     (unsigned)n);
            return FRQ1_E_IO;
        }
        rc = card_arm_get(l, atr, n, w);
        if (rc == FRQ1_ST_OK)
            *natr = n;
        return rc;
    }
    for (;;) {
        rc = card_call(l, FRQ1_CMD_CARD_ON, 0, 0, NULL, 0, d);
        if (rc != FRQ1_E_AGAIN)
            break;
        if (card_now_ms() > end) {
            snprintf(l->err, sizeof(l->err),
                     "no ATR within %u ms (stage %u)", limit_ms,
                     d[CARD_RP_STAGE]);
            return FRQ1_E_TIMEOUT;
        }
        chipio_nap(d[CARD_RP_WAIT]);
    }
    if (rc != FRQ1_ST_OK) {
        snprintf(l->err, sizeof(l->err), "CARD_ON: %s", card_rc_name(rc));
        return rc;
    }
    {
        size_t n = d[CARD_RP_ATRLEN], i;

        if (n == 0u || n > ATR_MAX) {
            snprintf(l->err, sizeof(l->err), "CARD_ON: ATR length %zu", n);
            return FRQ1_E_IO;
        }
        for (i = 0; i < n && i < CARD_RP_ATR_INLINE; i++)
            atr[i] = d[CARD_RP_ATR + i];
        if (n > CARD_RP_ATR_INLINE) {
            rc = card_call(l, FRQ1_CMD_CARD_GET, CARD_RP_ATR_INLINE, 0,
                           NULL, 0, d);
            if (rc != FRQ1_ST_OK) {
                snprintf(l->err, sizeof(l->err), "CARD_GET: %s",
                         card_rc_name(rc));
                return rc;
            }
            memcpy(atr + CARD_RP_ATR_INLINE, d, n - CARD_RP_ATR_INLINE);
        }
        *natr = n;
    }
    return FRQ1_ST_OK;
}

static inline int card_param(struct card_link *l, unsigned fi, unsigned di,
                             unsigned cwi, unsigned bwi)
{
    if (l->arm >= 0)
        return card_arm_ask(l, CARDSVC_PARAM, (fi << 16) | di,
                            (cwi << 8) | bwi, 1000u, NULL, NULL);
    return card_call(l, FRQ1_CMD_CARD_PARAM, (fi << 16) | di,
                     (cwi << 8) | bwi, NULL, 0, NULL);
}

/* t1_xfer_fn: PUT the block 40 bytes at a time, SEND, RECV until a whole
 * block is in, GET whatever did not fit the RECV report. */
static inline int card_xfer(void *ctx, const uint8_t *tx, size_t ntx,
                            uint8_t *rx, size_t *nrx, unsigned wait_ms)
{
    struct card_link *l = (struct card_link *)ctx;
    uint8_t d[FRQ1_DATA_BYTES];
    uint64_t end;
    size_t off, need, got;
    int rc;

    if (l->arm >= 0)
        return card_arm_xfer(l, tx, ntx, rx, nrx, wait_ms);
    *nrx = 0;
    for (off = 0; off < ntx; off += FRQ1_DATA_BYTES) {
        size_t k = ntx - off < FRQ1_DATA_BYTES ? ntx - off
                                               : FRQ1_DATA_BYTES;

        rc = card_call(l, FRQ1_CMD_CARD_PUT, (uint32_t)off, (uint32_t)k,
                       tx + off, k, NULL);
        if (rc != FRQ1_ST_OK)
            goto broke;
    }
    rc = card_call(l, FRQ1_CMD_CARD_SEND, (uint32_t)ntx, 0, NULL, 0, NULL);
    if (rc != FRQ1_ST_OK)
        goto broke;

    end = card_now_ms() + wait_ms;
    for (;;) {
        rc = card_call(l, FRQ1_CMD_CARD_RECV, 0, 0, NULL, 0, d);
        if (rc == FRQ1_ST_OK)
            break;
        if (rc == FRQ1_E_IO) {          /* more than one block came */
            snprintf(l->err, sizeof(l->err), "CARD_RECV: too many bytes");
            return T1_XFER_TIMEOUT;
        }
        if (rc != FRQ1_E_AGAIN)
            goto broke;
        if (card_now_ms() > end) {
            snprintf(l->err, sizeof(l->err),
                     "CARD_RECV: %u of %u bytes after %u ms",
                     (unsigned)card_get16(d, CARD_RV_GOT),
                     (unsigned)card_get16(d, CARD_RV_NEED), wait_ms);
            return T1_XFER_TIMEOUT;
        }
        chipio_nap(2);
    }
    got = card_get16(d, CARD_RV_GOT);
    need = card_get16(d, CARD_RV_NEED);
    if (need != got || got > T1_BLOCK_MAX || got < 4u) {
        snprintf(l->err, sizeof(l->err), "CARD_RECV: %zu bytes, need %zu",
                 got, need);
        return T1_XFER_TIMEOUT;
    }
    memcpy(rx, d + CARD_RV_BYTES,
           got < CARD_RV_INLINE ? got : CARD_RV_INLINE);
    for (off = CARD_RV_INLINE; off < got; off += FRQ1_DATA_BYTES) {
        size_t k = got - off < FRQ1_DATA_BYTES ? got - off
                                               : FRQ1_DATA_BYTES;

        rc = card_call(l, FRQ1_CMD_CARD_GET, (uint32_t)off, 0, NULL, 0, d);
        if (rc != FRQ1_ST_OK)
            goto broke;
        memcpy(rx + off, d, k);
    }
    *nrx = got;
    return T1_XFER_OK;

broke:
    /* card_call() already said why for the transport failures. */
    if (rc != CARD_XPORT && rc != CARD_SILENT)
        snprintf(l->err, sizeof(l->err), "card frame: %s",
                 card_rc_name(rc));
    return T1_XFER_IO;
}

#endif
