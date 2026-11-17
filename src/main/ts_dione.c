/* ts_dione.c -- the transport stream on the dione generation (GV-M2TV).
 *
 * WHY A SEPARATE PATH.  selene's capture blocks (tsif_selene.c, ring_selene.c) live at
 * 0x0190xxxx, which reads as the unmapped pattern on dione.  dione captures
 * through the 0x00D0 band instead, into a ring in DRAM, and its ring has
 * different rules: plain 188-byte packets back to back, no record header.
 *
 * WHAT IS KNOWN (driven from the host's 0xBC window on a live GV-M2TV, with
 * the CPU doing the copy -- 60 s at 2.16 MB/s, PAT decoded):
 *   - the 0x00D0 band is off in the loader state; two writes to the memory
 *     controller bring it up (dione_ts_start), and the pin gate's bit2 must
 *     be set or the band takes no write at all
 *   - the TS setup and ring values (regs_dione.h), in that order
 *   - the ring is a circle with a read offset, as on selene: the DMA goes
 *     back to the base at the limit's read-back value and stops in front of
 *     the read offset (regs_dione.h)
 *   - the CPU reads the ring at 0x02000000 and pushes it into EP 0x81's FIFO
 *     (the same burst as usbep.c, but the burst that reaches the end of the
 *     ring may be short)
 *
 * MEASURED FROM INSIDE (GV-M2TV): this file, in the FRC1 image, brought
 * the band up, armed the ring and streamed at 2.15 MB/s with the PAT
 * decoded.  Unlike the I2C controller (docs/i2c.md), the 0x00D0 band takes
 * writes from this CPU.
 *
 * TUNING IS NOT HERE.  The I2C comes up from inside (fe_dione.c), but this
 * tree has no tuning sequence of its own, so the host sends one a transfer
 * at a time with I2CXFER, then sends STREAM.
 *
 * SHAPE.  One STREAM frame moves up to arg0 bursts and returns; the state
 * (where we are in the ring) stays in .bss for the next frame.
 * The host reads EP 0x81 continuously on another thread: a frame waits for
 * room in the FIFO, so nothing moves unless the host is reading.
 */
#include "fw.h"
#include "mb86m01.h"
#include "regs_dione.h"

/* The dione link (link_dione.ld) defines this symbol; the selene link does
 * not, so its address is 0 there.  Weak, so the compiler cannot assume it
 * is non-null and fold the test away. */
extern const char fw_dione_image[] __attribute__((weak));

int fw_is_dione(void)
{
    return fw_dione_image != 0;
}

/* Bringing the band up: two writes to the memory controller (measured on a
 * GV-M2TV from a cold start, each write taken out on its own and the TS
 * tried again):
 *   - DN_MEMC_CTL = DN_MEMC_CTL_RUN: without it no TS arrives
 *   - DN_MEMC_CFG = DN_MEMC_CFG_ON: without it (the loader leaves the
 *     register cleared) the sub-blocks at 0x00D01xxx / 0x00D02xxx do not
 *     decode, and it is the value the readback below checks
 * Other writes to this band seen elsewhere (to 0x00D03xxx..0x00D61xxx, and
 * two more to the memory controller) were taken out with these two kept,
 * and the TS still ran with no sync loss or continuity jump. */

/* STREAM's data byte 0: which bulk IN endpoints carry the TS.  Pair 0 is
 * EP 0x81, pair 1 is EP 0x84 (the same burst, measured on a GV-M2TV:
 * docs/arm.md).  Both on sends the same bytes to each, and a burst waits
 * until both have room. */
#define DS_OUT_EP81     0x01u
#define DS_OUT_EP84     0x02u

/* Spins of the inner wait (a status read and a few compares) before a frame
 * gives up and returns; the host just sends the next frame. */
#define SPIN_BUDGET     500000u
/* 64 bursts (32 KiB) by default: the frame holds EP0 while it runs, and
 * another host program's frames (the card reader driver's, in its own
 * mailbox) wait for it.  The bigger the count, the longer a card frame
 * waits during a recording; with 64 a card frame takes what it takes idle,
 * and 64 still keeps up with the stream (measured on a GV-M2TV). */
#define BLOCKS_DEF      64u
#define BLOCKS_MAX      8192u

/* The band does not need a pause between these writes: back-to-back
 * writes bring it up (measured on a GV-M2TV). */
static void put(unsigned reg, unsigned v)
{
    MMIO(reg) = v;
}

/* .bss: kept between frames. */
static unsigned running;
static unsigned wrap;           /* where the DMA goes back to the base */
static unsigned pos;            /* next byte to send, CPU address */
static unsigned laps;           /* times the reader went back to the base */
static unsigned peak;           /* most bytes seen waiting in the ring */
static unsigned fulls;          /* times the ring was found full        */
static unsigned was_full;
static unsigned packets;        /* packet starts sent that held 0x47    */
static unsigned misses;         /* packet starts that did not           */
static unsigned teis;           /* of those with 0x47, how many had TEI */
static unsigned tei_next;       /* the last start's byte 1 is in the    */
                                /* next word                            */
static unsigned sync_left;      /* bytes to the next packet start       */
static unsigned hunting;        /* looking for a 0x47 to start from     */
static unsigned outs;           /* bulk IN endpoints, set by each frame */
/* The first read-back that disagreed in the last failed start, so the
 * report can say where the band stopped taking writes, not just FW_EIO. */
static unsigned bad_reg;
static unsigned bad_val;

/* The pin registers are the high half of their word; the window wrote the
 * whole word and kept the low half, so this does too. */
static void set_high(unsigned reg, unsigned v)
{
    put(reg, (v << 16) | (MMIO(reg) & 0xffffu));
}

static unsigned get_high(unsigned reg)
{
    return MMIO(reg) >> 16;
}

/* Read reg back and keep it if (value & mask) is not want.  Returns 0 when
 * it matched.  Only the first disagreement of a start is kept. */
static int differs(unsigned reg, unsigned mask, unsigned want)
{
    unsigned v = MMIO(reg);

    if ((v & mask) == want)
        return 0;
    if (!bad_reg) {
        bad_reg = reg;
        bad_val = v;
    }
    return 1;
}

int dione_ts_start(unsigned length)
{
    if (running)
        return FW_OK;
    if (length < DN_RING_MIN || length > DN_RING_MAX || (length & 0xffffu))
        return FW_EINVAL;
    bad_reg = 0;
    bad_val = 0;

    /* Wake the band, before anything that touches 0x00D01xxx/0x00D02xxx:
     * those do not decode until it is up. */
    set_high(DN_PIN_ROUTE, DN_PIN_ROUTE_TS);
    set_high(DN_PIN_GATE, DN_PIN_GATE_ALL);
    put(DN_MEMC_CTL, DN_MEMC_CTL_RUN);
    put(DN_MEMC_CFG, DN_MEMC_CFG_ON);
    set_high(DN_PIN_GATE, get_high(DN_PIN_GATE) | DN_PIN_GATE_MEM);
    /* The top byte reads back with bits set (0x05 on a GV-M2TV) that were
     * not written; only the low 24 bits are ours to compare. */
    if (differs(DN_MEMC_CFG, DN_MEMC_CFG_MASK, DN_MEMC_CFG_ON))
        return FW_EIO;          /* the band did not take the writes */

    /* TS input, then the ring. */
    put(DN_TSIN_020, DN_RING_AT);
    put(DN_TSIN_004, 0x04000000u);
    put(DN_TSIN_CTL, 0x020f0000u);
    set_high(DN_TS_GATE, 0);
    put(DN_RING_BASE, DN_RING_AT);
    put(DN_RING_LIMIT, length - 0x20u);
    put(DN_RING_MODE, DN_RING_MODE_RUN);
    put(DN_TSIN_020, 0);
    put(DN_TSIN_RUN, 0x01000000u);
    put(DN_RING_WRITE, 0);
    put(DN_RING_READ, 0);
    put(DN_RING_038, 0);
    /* The limit reads back in 64 B steps; that value is where the DMA goes
     * back to the base. */
    wrap = MMIO(DN_RING_LIMIT);
    if (differs(DN_RING_MODE, ~0u, DN_RING_MODE_RUN) ||
        differs(DN_RING_BASE, ~0u, DN_RING_AT) ||
        differs(DN_RING_LIMIT, ~0u, (length - 0x20u) & ~0x3fu))
        return FW_EIO;

    pos = DN_RING_AT;
    laps = 0;
    peak = 0;
    fulls = 0;
    was_full = 0;
    packets = 0;
    misses = 0;
    teis = 0;
    tei_next = 0;
    sync_left = 0;
    hunting = 1;
    MMIO(DN_RING_KICK) = DN_RING_AT;
    running = 1u;
    return FW_OK;
}

int dione_ts_running(void)
{
    return running != 0u;
}

int dione_ts_stop(void)
{
    if (!running)
        return FW_OK;
    MMIO(DN_TSIN_RUN) = 0;
    running = 0;
    return FW_OK;
}

#define TS_PACKET       188u
#define TS_SYNC         0x47u
#define TS_TEI          0x80u   /* byte 1: the demod could not correct it */
#define WORD_BYTE(w, k) (((w) >> (24u - 8u * (k))) & 0xffu)

/* Follow the packet starts through the words as they go out: count the
 * ones that hold 0x47, and when one does not, count it and look for the
 * next 0x47 to go on from.  Nothing is held back -- the host gets the bytes
 * either way; this only says whether what left was in step.
 *
 * Of the starts that hold 0x47, also count those whose transport error
 * indicator (bit7 of byte 1) is set: the packets the demodulator handed on
 * without being able to correct them.  Byte 1 is in the next word when the
 * start is the last byte of this one. */
static void sync_word(unsigned w)
{
    unsigned k = 0;

    if (tei_next) {
        tei_next = 0;
        if (WORD_BYTE(w, 0) & TS_TEI)
            teis++;
    }
    for (;;) {
        if (hunting) {
            while (k < 4u && WORD_BYTE(w, k) != TS_SYNC)
                k++;
            if (k == 4u)
                return;
            hunting = 0;
            sync_left = k;
        }
        if (sync_left >= 4u) {
            sync_left -= 4u;
            return;
        }
        if (WORD_BYTE(w, sync_left) == TS_SYNC) {
            packets++;
            if (sync_left == 3u)
                tei_next = 1;
            else if (WORD_BYTE(w, sync_left + 1u) & TS_TEI)
                teis++;
            sync_left += TS_PACKET;
            continue;
        }
        misses++;
        hunting = 1;
        k = sync_left + 1u;
    }
}

/* Every endpoint in outs has room for a burst. */
static int outs_ready(void)
{
    if ((outs & DS_OUT_EP81) && !(MMIO(USB_EP0_STATUS) & USB_EP_READY))
        return 0;
    if ((outs & DS_OUT_EP84) && !(MMIO(USB_EP1_STATUS) & USB_EP_READY))
        return 0;
    return 1;
}

/* One burst of `words` words from pos, to each endpoint in outs (the same
 * bytes to both when both are on), then hand the bytes back to the DMA
 * through the read offset.  The caller has seen room. */
static void burst(unsigned words)
{
    unsigned i, off, w;

    for (i = 0; i < words; i++) {
        w = MMIO(pos);
        if (outs & DS_OUT_EP81)
            MMIO(USB_EP0_FIFO) = w;
        if (outs & DS_OUT_EP84)
            MMIO(USB_EP1_FIFO) = w;
        sync_word(w);
        pos += 4u;
    }
    MMIO(USB_EP_GUARD) = USB_EP_OPEN;
    if (outs & DS_OUT_EP81)
        MMIO(USB_EP0_STATUS) = USB_EP_COMMIT;
    if (outs & DS_OUT_EP84)
        MMIO(USB_EP1_STATUS) = USB_EP_COMMIT;
    MMIO(USB_EP_GUARD) = USB_EP_CLOSED;

    off = pos - DN_RING_AT;
    if (off >= wrap) {
        off = 0;
        pos = DN_RING_AT;
        laps++;
    }
    MMIO(DN_RING_READ) = off;
}

/* How many words can go now, 0 if none.  Up to the write offset, whole
 * 512 B bursts only; at the end of the ring, what is left before the wrap
 * (it may be short), so the next burst starts at the base. */
static unsigned ready_words(void)
{
    unsigned wp = MMIO(DN_RING_WRITE);
    unsigned rd = pos - DN_RING_AT;
    unsigned n;

    if (wp >= wrap)
        return 0;               /* not a value the DMA writes */
    n = wp >= rd ? wp - rd : wrap - rd + wp;
    if (n > peak)
        peak = n;
    /* Full: the DMA has stopped DN_RING_FULL_GAP short of us.  Counted
     * once per time it is found so, not once per look. */
    if (wrap - n <= DN_RING_FULL_GAP) {
        if (!was_full)
            fulls++;
        was_full = 1;
    } else {
        was_full = 0;
    }
    if (wp >= rd) {
        if (n < USB_EP_BURST)
            return 0;
        return USB_EP_BURST / 4u;
    }
    n = wrap - rd;              /* the DMA has gone round: finish the end */
    if (n > USB_EP_BURST)
        n = USB_EP_BURST;
    return n / 4u;
}

/* Move up to `max` bursts.  Returns how many; *bytes gets the byte count. */
static unsigned pump(unsigned max, unsigned *bytes)
{
    unsigned sent = 0, spin = SPIN_BUDGET, words;

    *bytes = 0;
    while (sent < max && spin) {
        if (!outs_ready()) {
            spin--;
            continue;           /* the host has not read the FIFO yet */
        }
        words = ready_words();
        if (!words) {
            spin--;
            continue;           /* the capture has not got that far yet */
        }
        burst(words);
        sent++;
        *bytes += words * 4u;
        spin = SPIN_BUDGET;
    }
    return sent;
}

/* The report STREAM leaves in FR_DATA on dione.  40 bytes, big-endian. */
#define DR_VERSION      0x00    /* u8  = 3 (1 had no 0x02, 0x03, 0x24;   */
                                /*     2 had the ring length at 0x1c)     */
#define DR_FLAGS        0x01    /* u8  bit0 running, bit2 ran out of time, */
                                /*     bit4 EP 0x81 on, bit5 EP 0x84 on    */
#define DR_FULLS        0x02    /* u8  times the ring was found full (DMA */
                                /*     stopped, input lost), at most 255  */
#define DR_MISSES       0x03    /* u8  packet starts without 0x47, <= 255 */
#define DR_BURSTS       0x04    /* u32 bursts this frame                  */
#define DR_BYTES        0x08    /* u32 bytes this frame                   */
#define DR_WRITE        0x0c    /* u32 the capture's write offset         */
#define DR_RC           0x10    /* s32 why the frame failed, 0 if it did not */
#define DR_LAPS         0x14    /* u32 times the reader wrapped           */
#define DR_READ         0x18    /* u32 our read offset                    */
#define DR_TEIS         0x1c    /* u32 of DR_PACKETS, those with TEI set  */
#define DR_PEAK         0x20    /* u32 most bytes seen waiting in the ring */
#define DR_PACKETS      0x24    /* u32 packet starts sent that held 0x47  */
#define DR_BAD_VAL      0x1c
/* When the start failed with FW_EIO, DR_READ and DR_BAD_VAL (0 then anyway)
 * carry the register whose read-back disagreed and the value it read. */

/* arg0: bursts to try this frame (0 = 64, at most 8192).
 * arg1: ring length in MiB for the first frame (0 = 16, 1..16); ignored
 *       once running.
 * data byte 0: the endpoints (DS_OUT_*; 0 = EP 0x81).  Read every frame,
 *       so a recording can move or add an endpoint between frames. */
int dione_stream(frame_t f)
{
    unsigned want = frq1_be32(f, FR_ARG0);
    unsigned mib = frq1_be32(f, FR_ARG1);
    unsigned sel = f[FR_DATA];
    unsigned sent = 0, bytes = 0, flags = 0, i;
    int rc;

    if (sel & ~(DS_OUT_EP81 | DS_OUT_EP84))
        return FW_EINVAL;
    outs = sel ? sel : DS_OUT_EP81;

    for (i = 0; i < 40u; i++)
        f[FR_DATA + i] = 0;
    f[FR_DATA + DR_VERSION] = 3;

    if (!mib)
        mib = DN_RING_MAX >> 20;
    rc = dione_ts_start(mib << 20);
    if (rc == FW_OK) {
        flags |= 0x01u;
        if (!want)
            want = BLOCKS_DEF;
        if (want > BLOCKS_MAX)
            want = BLOCKS_MAX;
        sent = pump(want, &bytes);
        if (sent < want)
            flags |= 0x04u;
    }

    if (outs & DS_OUT_EP81)
        flags |= 0x10u;
    if (outs & DS_OUT_EP84)
        flags |= 0x20u;
    f[FR_DATA + DR_FLAGS] = (unsigned char)flags;
    frq1_put32(f, FR_DATA + DR_BURSTS, sent);
    frq1_put32(f, FR_DATA + DR_BYTES, bytes);
    frq1_put32(f, FR_DATA + DR_WRITE, running ? MMIO(DN_RING_WRITE) : 0u);
    frq1_put32(f, FR_DATA + DR_RC, (unsigned)rc);
    frq1_put32(f, FR_DATA + DR_LAPS, laps);
    if (running) {
        frq1_put32(f, FR_DATA + DR_READ, pos - DN_RING_AT);
        frq1_put32(f, FR_DATA + DR_TEIS, teis);
        frq1_put32(f, FR_DATA + DR_PEAK, peak);
        frq1_put32(f, FR_DATA + DR_PACKETS, packets);
        f[FR_DATA + DR_FULLS] = (unsigned char)(fulls > 255u ? 255u : fulls);
        f[FR_DATA + DR_MISSES] =
            (unsigned char)(misses > 255u ? 255u : misses);
    } else if (rc == FW_EIO) {
        frq1_put32(f, FR_DATA + DR_READ, bad_reg);
        frq1_put32(f, FR_DATA + DR_BAD_VAL, bad_val);
    }
    return rc;
}
