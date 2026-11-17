/* fe_m2tv.h -- the GV-M2TV front end (terrestrial), tuned by the host
 * over I2CXFER.  Header only, like fe_dt295.h.
 *
 * Every write was kept because taking it out, or flipping one of its bits,
 * changed something on a real GV-M2TV running this firmware (lock, C/N or
 * the TS); the comment on each write says what.
 *
 * The parts: a demodulator at 7-bit 0x18 and, behind it, a tuner.  A write
 * to the demodulator that starts FE C0 is passed on to the tuner, so
 * [FE C0 reg value] writes one tuner register.
 *
 * Method, on a GV-M2TV running this firmware: every write below was taken
 * out on its own, and then each bit of each remaining write was put back to
 * its power-up value, one at a time.  Before each trial every demodulator
 * register the sequence writes was put back to its power-up value (the
 * demodulator has no reset we know of) and the tuner was reset.  The writes
 * left are the ones needed for the same lock time, the same C/N and a clean
 * TS (no resync, no continuity jump, no TEI, PAT present), checked on
 * the UHF channels this file tunes (13..50, up to M2TV_KHZ_MAX).
 * The comment on each write says what happens without it.  "Free" bits
 * changed nothing when flipped; the read-modify-writes leave them as the
 * chip has them.
 */
#ifndef FE_M2TV_H
#define FE_M2TV_H

#include <stddef.h>
#include <stdint.h>

/* One I2C transfer: write wn bytes, then read rn.  0 ok, -1 failed.  The
 * same shape as fe_dt295.h's. */
typedef int (*m2tv_xfer_fn)(void *ctx, unsigned addr7, const uint8_t *w,
                            unsigned wn, uint8_t *r, unsigned rn);

#define M2TV_DEMOD      0x18u   /* 7-bit                                  */
#define M2TV_KHZ_MIN    470000u
#define M2TV_KHZ_MAX    700000u /* above it: not measured, refused        */

#define M2TV__DEMOD    M2TV_DEMOD

static inline int m2tv__dw(m2tv_xfer_fn x, void *ctx, unsigned reg,
                           unsigned v)
{
    uint8_t b[2] = { (uint8_t)reg, (uint8_t)v };
    return x(ctx, M2TV__DEMOD, b, 2, NULL, 0);
}

static inline int m2tv__dr(m2tv_xfer_fn x, void *ctx, unsigned reg,
                           uint8_t *out, unsigned n)
{
    uint8_t b = (uint8_t)reg;
    return x(ctx, M2TV__DEMOD, &b, 1, out, n);
}

/* Read, clear `clear`, set `set`, write back. */
static inline int m2tv__dmod(m2tv_xfer_fn x, void *ctx, unsigned reg,
                             unsigned clear, unsigned set)
{
    uint8_t v;
    if (m2tv__dr(x, ctx, reg, &v, 1) != 0)
        return -1;
    return m2tv__dw(x, ctx, reg, (v & ~clear & 0xFFu) | set);
}

/* One tuner register, through the demodulator. */
static inline int m2tv__tw(m2tv_xfer_fn x, void *ctx, unsigned reg,
                           unsigned v)
{
    uint8_t b[4] = { 0xFE, 0xC0, (uint8_t)reg, (uint8_t)v };
    return x(ctx, M2TV__DEMOD, b, 4, NULL, 0);
}

static inline int m2tv_init(m2tv_xfer_fn x, void *ctx)
{
    static const uint8_t reset[3] = { 0xFE, 0xC0, 0xFF };

    /* Tuner register 0xFF, no data: a reset.  Read back afterwards, the
     * tuner registers are at their power-up values (all but 0x00 and
     * 0xD5..0xD8, which move on their own), so the writes below start from
     * known values. */
    if (x(ctx, M2TV__DEMOD, reset, 3, NULL, 0) != 0)
        return -1;

    /* Demod 0x11, 0x12, 0x13: bits 2..0 of each must be 6, 4 and 3
     * (power-up 1, 3 and 4).  Any one bit flipped back: no lock on either
     * channel.  Bits 7..3 are kept. */
    if (m2tv__dmod(x, ctx, 0x11, 0x07, 0x06) != 0 ||
        m2tv__dmod(x, ctx, 0x12, 0x07, 0x04) != 0 ||
        m2tv__dmod(x, ctx, 0x13, 0x07, 0x03) != 0)
        return -1;
    /* Demod 0x14: bit 6 must be 1 (power-up 0x0A has it 0: no lock).  Bits
     * 3 and 1 free. */
    if (m2tv__dmod(x, ctx, 0x14, 0x00, 0x40) != 0)
        return -1;
    /* Demod 0x16: bits 1..0 must be 0 (power-up 0x03; either one set: no
     * lock). */
    if (m2tv__dmod(x, ctx, 0x16, 0x03, 0x00) != 0)
        return -1;
    /* Demod 0x30: bit 3 must be 0 (power-up 0x28; set: no lock).  Bit 5
     * free. */
    if (m2tv__dmod(x, ctx, 0x30, 0x08, 0x00) != 0)
        return -1;
    /* Demod 0x31: bit 1 must be 0 (power-up 0x0F; set: no lock). */
    if (m2tv__dmod(x, ctx, 0x31, 0x02, 0x00) != 0)
        return -1;
    /* Demod 0x34: bits 5..4 must not both be 1.  Power-up 0x3F has both:
     * the C/N drops by several dB (TS with errors), and a weaker channel
     * does not lock.  Either one alone is fine; bit 5 is cleared. */
    if (m2tv__dmod(x, ctx, 0x34, 0x20, 0x00) != 0)
        return -1;
    /* Demod 0x1C: bit 6 must be 0.  The power-up 0x55 has it set: lock
     * takes seconds with a C/N of a few dB, or does not come at all.
     * Other bits free. */
    if (m2tv__dmod(x, ctx, 0x1C, 0x40, 0x00) != 0)
        return -1;
    /* Tuner 0x02: bit 2 must be 0 (power-up 0x06; set: no lock).  Bit 1
     * free; kept as it comes up. */
    if (m2tv__tw(x, ctx, 0x02, 0x02) != 0)
        return -1;
    /* Tuner 0x01: 1 (power-up 0): without it, no lock. */
    if (m2tv__tw(x, ctx, 0x01, 0x01) != 0)
        return -1;
    /* Demod 0x1E: bits 4 and 2 must be 0; demod 0x1F: bits 6 and 4 must be
     * 0 (power-up 0x54 each).  Any one of them set: the demodulator locks
     * and data comes out, but not a TS (no PAT).  Other bits free. */
    if (m2tv__dmod(x, ctx, 0x1E, 0x14, 0x00) != 0 ||
        m2tv__dmod(x, ctx, 0x1F, 0x50, 0x00) != 0)
        return -1;
    /* Demod 0x75: bit 3 set leaves out most of the null packets.  Clear
     * (power-up 0x00), every channel comes at the full rate with the
     * padding as null packets.  The same programme packets either way
     * (same count, no continuity jump).
     * Demod 0x76 bit 3 does the same; one is enough.  Bit 5 of 0x75: no
     * effect.  Not needed for lock or a clean TS: kept for the USB
     * bandwidth and the size of a recording. */
    if (m2tv__dmod(x, ctx, 0x75, 0x00, 0x08) != 0)
        return -1;
    return 0;
}

static inline unsigned m2tv_freq_word(unsigned khz)
{
    /* The tuner takes the frequency in 1/64 MHz, rounded. */
    return (unsigned)((((uint64_t)khz * 64u) + 500u) / 1000u) & 0xFFFFu;
}

static inline int m2tv_tune(m2tv_xfer_fn x, void *ctx, unsigned khz)
{
    unsigned n = m2tv_freq_word(khz);

    /* Tuner 0x0D / 0x0E: the frequency word, low byte then high. */
    if (m2tv__tw(x, ctx, 0x0D, n & 0xFFu) != 0 ||
        m2tv__tw(x, ctx, 0x0E, n >> 8) != 0)
        return -1;
    /* Tuner 0x0F = 1 makes the tuner take the new frequency (without it,
     * no lock).  Writing 0 first was not needed: a scan retunes with the
     * 1 alone. */
    if (m2tv__tw(x, ctx, 0x0F, 0x01) != 0)
        return -1;
    /* Demod 0x01 = 0x40 starts the demodulator on the new signal (power-up
     * 0; without it, no lock on one channel and 2.3 s on the other). */
    if (m2tv__dw(x, ctx, 0x01, 0x40) != 0)
        return -1;
    return 0;
}

static inline int m2tv_status(m2tv_xfer_fn x, void *ctx, int *locked,
                              uint32_t *cnr)
{
    uint8_t s, q, t, c[3];

    /* Locked when demod 0x80 has neither bit 5 nor bit 3 set, the low
     * nibble of 0xB0 is 8 or more, and 0x96 has one of bits 7..5 (before
     * lock: 0x80 = 0xAF, 0xB0 = 0xA0, 0x96 = 0x00; locked: 0x80 =
     * 0x00..0x07, 0xB0 = 0xA9, 0x96 = 0x40..0xC0).  0x96 comes up last:
     * with 0x80 and 0xB0 alone, polled every 20 ms, the TS starts with
     * about 0.5 s of packets the demodulator could not correct. */
    if (m2tv__dr(x, ctx, 0x80, &s, 1) != 0 ||
        m2tv__dr(x, ctx, 0xB0, &q, 1) != 0 ||
        m2tv__dr(x, ctx, 0x96, &t, 1) != 0)
        return -1;
    /* Demod 0x8B..0x8D: the C/N, 24 bits.  Smaller is better. */
    if (m2tv__dr(x, ctx, 0x8B, c, 3) != 0)
        return -1;
    if (locked)
        *locked = (s & 0x28u) == 0 && (q & 0x0Fu) >= 8u && (t & 0xE0u) != 0;
    if (cnr)
        *cnr = ((uint32_t)c[0] << 16) | ((uint32_t)c[1] << 8) | c[2];
    return 0;
}

#endif /* FE_M2TV_H */
