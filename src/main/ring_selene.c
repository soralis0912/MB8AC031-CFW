/* ring_selene.c -- the ring the captured transport stream lands in.
 *
 * NOT a software ring.  The producer is a hardware capture block: it writes
 * fixed-size records into SDRAM and publishes its write pointer in a
 * register, and the only thing this side does is move the read pointer.  So
 * there is no head/tail pair to own here -- there are two registers, and the
 * work is the arithmetic between them and finding where a record starts.
 *
 * WHAT THE CODE RELIES ON.  regs_selene.h lists the registers.  The ring's
 * shape, as this code takes it:
 *
 *   * the record is 192 bytes with a 188-byte TS packet at its front
 *   * used = (write - read) mod length
 *   * a record boundary is found by looking for 0x47 at the same offset in
 *     three consecutive records, stepping the offset over 0..191; so the
 *     search waits for three whole records (TS_SYNC_MIN), and a ring
 *     shorter than four records is refused (TS_LENGTH_MIN)
 *   * sending n packets from offset k needs (n-1)*192 + 188 + k bytes
 *
 * The TS has streamed through this code on a PIX-DT295 (tsout_selene.c).
 *
 * WHAT IS CHECKED.  tsif_selene.c fills the ring's registers, and if they
 * still read zeros ring_init() says FW_EIO.  That is the honest answer and
 * it is loud.
 *
 * A DESIGN DECISION.  This code keeps the sync offset apart from the read
 * pointer and advances the pointer by whole records, which preserves the
 * invariant it just established: if read+k is a record boundary then so is
 * read + n*192 + k.  The one case that breaks it is a ring whose length is
 * not a multiple of the record size -- wrapping then shifts every boundary
 * -- so a wrap on such a ring drops the sync and makes the next caller look
 * again.
 */
#include "fw.h"
#include "mb86m01.h"
#include "regs_selene.h"

static unsigned cpu_base;       /* the ring, as this CPU addresses it */
static unsigned length;
static unsigned sync_off;       /* where a record starts, 0..191 */
static unsigned aligned;        /* is sync_off established? */
static unsigned whole;          /* is the length a whole number of records? */
static unsigned ready;

static unsigned wrap(unsigned a)
{
    return (a >= length) ? a - length : a;
}

int ring_init(void)
{
    unsigned base = MMIO(TSCAP_BASE);
    unsigned len = MMIO(TSCAP_LENGTH);
    unsigned rem = 0;

    ready = aligned = sync_off = whole = 0;
    cpu_base = length = 0;

    /* Does what the capture block reports make sense at all? */
    if (len <= TS_LENGTH_MIN - 1u)
        return FW_EIO;
    if (MMIO(TSCAP_READ) >= len)
        return FW_EIO;
    if (!base)
        return FW_EIO;

    cpu_base = base | TSCAP_CPU_VIEW;
    length = len;
    (void)fw_udiv(len, TS_RECORD, &rem);
    whole = (rem == 0u);
    ready = 1u;
    return FW_OK;
}

/* How far the producer is ahead of us. */
unsigned ring_used(void)
{
    unsigned wp, rp;

    if (!ready)
        return 0;
    wp = MMIO(TSCAP_WRITE);
    rp = MMIO(TSCAP_READ);
    if (wp >= length || rp >= length)
        return 0;               /* the block is not making sense */
    return (wp >= rp) ? (wp - rp) : (wp + length - rp);
}

/* Find where a record starts.
 *
 * FW_OK once it is known, FW_EAGAIN while there is not enough in the ring to
 * tell or no candidate matched this round -- not an error, just not yet.  A
 * round that finds nothing throws one record away, which is what makes the
 * next round look at different bytes.
 */
int ring_sync(void)
{
    unsigned rp, k, hit, i;

    if (!ready)
        return FW_EIO;
    if (aligned)
        return FW_OK;
    if (ring_used() < TS_SYNC_MIN)
        return FW_EAGAIN;

    rp = MMIO(TSCAP_READ);
    for (k = 0; k < TS_RECORD; k++) {
        hit = 0;
        for (i = 0; i < TS_SYNC_PROBES; i++)
            if (MMIO8(cpu_base + wrap(rp + k + i * TS_RECORD)) == TS_SYNC)
                hit++;
        if (hit == TS_SYNC_PROBES) {
            sync_off = k;
            aligned = 1u;
            return FW_OK;
        }
    }
    MMIO(TSCAP_READ) = wrap(rp + TS_RECORD);
    return FW_EAGAIN;
}

/* Whole TS packets that can be taken right now. */
unsigned ring_avail(void)
{
    unsigned used = ring_used();
    unsigned need = TS_PACKET + sync_off;

    if (!aligned || used < need)
        return 0;
    /* fw_udiv, not `/`: there is no libgcc here (util.c says why). */
    return 1u + fw_udiv(used - need, TS_RECORD, 0);
}

/* Where packet i's 188 bytes begin, counted from the read pointer. */
unsigned ring_offset(unsigned i)
{
    return sync_off + i * TS_RECORD;
}

/* Copy n bytes from off past the read pointer, wrapping at the end. */
int ring_read(unsigned off, unsigned char *dst, unsigned n)
{
    unsigned a, i;

    if (!ready)
        return FW_EIO;
    if (!dst || !n || n > length || off >= length)
        return FW_EINVAL;
    a = wrap(MMIO(TSCAP_READ) + off);

    /* The fast case: no wrap inside the span, and source and destination
     * share an alignment.  A packet starts wherever the capture put the
     * first record (2 mod 4 on a PIX-DT295, measured), so half-words are
     * the usual case; a byte at a time made the stream CPU-bound at about
     * 1.2 MB/s (measured). */
    if (a + n <= length) {
        unsigned src = cpu_base + a;
        unsigned d = (unsigned)dst;

        if (!((src | d | n) & 3u)) {
            for (i = 0; i < n; i += 4u)
                *(unsigned *)(d + i) = MMIO(src + i);
            return FW_OK;
        }
        if (!((src | d | n) & 1u)) {
            for (i = 0; i < n; i += 2u)
                *(unsigned short *)(d + i) = MMIO16(src + i);
            return FW_OK;
        }
    }
    for (i = 0; i < n; i++) {
        dst[i] = MMIO8(cpu_base + a);
        a++;
        if (a >= length)
            a = 0;
    }
    return FW_OK;
}

/* Push n bytes (a multiple of 4) from off past the read pointer straight
 * into an endpoint FIFO, a word per store.  No copy in between: a packet
 * starts 2 mod 4 on a PIX-DT295 (measured), so the usual case is two aligned
 * word loads and a shift per word.  The other cases assemble from bytes.
 * Loads only -- nothing is stored into the SDRAM, whose narrow stores are
 * known to repaint neighbouring bytes (measured). */
int ring_to_fifo(unsigned off, unsigned n, unsigned fifo)
{
    unsigned a, src, i, w0, w1;

    if (!ready)
        return FW_EIO;
    if (!n || (n & 3u) || n > length || off >= length || !fifo)
        return FW_EINVAL;
    a = wrap(MMIO(TSCAP_READ) + off);
    src = cpu_base + a;

    if (a + n <= length && !(src & 3u)) {
        for (i = 0; i < n; i += 4u)
            MMIO(fifo) = MMIO(src + i);
        return FW_OK;
    }
    if (a + n + 2u <= length && (src & 3u) == 2u) {
        /* The word before and the word after cover it.  a + n + 2 stays
         * inside the ring, so the last load does too. */
        src -= 2u;
        w0 = MMIO(src);
        for (i = 0; i < n; i += 4u) {
            w1 = MMIO(src + i + 4u);
            MMIO(fifo) = (w0 << 16) | (w1 >> 16);
            w0 = w1;
        }
        return FW_OK;
    }
    for (i = 0; i < n; i += 4u) {
        unsigned k, w = 0;
        for (k = 0; k < 4u; k++) {
            w = (w << 8) | MMIO8(cpu_base + a);
            if (++a >= length)
                a = 0;
        }
        MMIO(fifo) = w;
    }
    return FW_OK;
}

/* Give n packets back to the producer and publish the pointer. */
int ring_consume(unsigned n)
{
    unsigned rp, step;

    if (!ready)
        return FW_EIO;
    if (!aligned)
        return FW_EIO;
    if (!n)
        return FW_EINVAL;
    if (n > ring_avail())
        return FW_EINVAL;
    /* ring_avail() counts whole 188-byte PACKETS, and this advances by whole
     * 192-byte RECORDS.  If the producer has written only the packet and not
     * the 4 bytes behind it, those two disagree by 4 and the read pointer
     * would step past the write pointer -- which reads back as a nearly full
     * ring.  A producer that writes whole records never gets here; one that
     * can stop mid-record waits 4 bytes instead of wrapping.
     */
    if (ring_used() < n * TS_RECORD)
        return FW_EAGAIN;

    rp = MMIO(TSCAP_READ);
    step = n * TS_RECORD;
    if (rp + step >= length && !whole)
        aligned = 0u;           /* the wrap moved every boundary */
    MMIO(TSCAP_READ) = wrap(rp + step);
    return FW_OK;
}
