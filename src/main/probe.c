/* probe.c -- questions asked of the chip from inside the firmware: read and
 * write any address at a chosen width, arm the capture ring, scan an I2C
 * bus, start an endpoint DMA.  Nothing else in the firmware calls these.
 */
#include "fw.h"
#include "mb86m01.h"
#include "regs_selene.h"
#include "fjcore.h"

/* ---------------------------------------------------------------- peek */

#define PEEK_MAX        8u      /* 8 words is what the 40-byte frame holds */

#define W_WORD          0u
#define W_HALF          1u
#define W_BYTE          2u

static unsigned stride_of(unsigned width)
{
    return width == W_WORD ? 4u : (width == W_HALF ? 2u : 1u);
}

static unsigned rd(unsigned a, unsigned width)
{
    return width == W_WORD ? MMIO(a)
         : (width == W_HALF ? (unsigned)MMIO16(a) : (unsigned)MMIO8(a));
}

static void wr(unsigned a, unsigned width, unsigned v)
{
    if (width == W_WORD)
        MMIO(a) = v;
    else if (width == W_HALF)
        MMIO16(a) = (unsigned short)v;
    else
        MMIO8(a) = (unsigned char)v;
}

/* PEEK (0x40).  arg0 = address, arg1 = width << 8 | count.
 *
 * Both are argument words of the frame (docs/frq1.md).
 *
 * The loader's bank aperture already reads any 32-bit address with a CPU
 * load, so the new thing here is not reach, it is WIDTH: this does lduh and
 * ldub as well, and the chip has registers that only answer at their own
 * width. */
int probe_peek(frame_t f)
{
    unsigned addr = frq1_be32(f, FR_ARG0);
    unsigned ctl = frq1_be32(f, FR_ARG1);
    unsigned n = ctl & 0xffu;
    unsigned width = (ctl >> 8) & 3u;
    unsigned stride, i;

    if (n == 0u || n > PEEK_MAX)
        return FW_EINVAL;
    stride = stride_of(width);
    /* A misaligned access on FR does not fault, it reads the wrong place.
     * Refuse instead of reporting a lie. */
    if (addr & (stride - 1u))
        return FW_EINVAL;

    f[FR_DATA] = 1;                             /* version                */
    f[FR_DATA + 1] = (unsigned char)n;
    f[FR_DATA + 2] = (unsigned char)width;
    f[FR_DATA + 3] = 0;
    frq1_put32(f, FR_DATA + 4, addr);
    for (i = 0; i < n; i++)
        frq1_put32(f, (int)(FR_DATA + 8u + i * 4u),
                   rd(addr + i * stride, width));
    return FW_OK;
}

/* POKE (0x41).  arg0 = address, arg1 = width << 8 | flags, FIRST DATA WORD =
 * the value.  flags bit0 asks for the old value to be put back after the
 * read-back.
 *
 * The value stays in the data area on purpose: it is what the command
 * carries, not how it is addressed.
 *
 * WHY A WRITE AND NOT JUST A READ.  A block that reads 0 is either asleep or
 * awake with a reset value of 0, and on this chip we have both: once
 * 0x00F10118 bit4 is set, 0x01900404 answers but 0x01902000 still reads 0.
 * Only a write that sticks tells them apart.
 *
 * Reports the old value, what was written, what came back, and -- when it
 * restored -- what the address reads after that, so the host can see the
 * whole transaction rather than trust one number. */
int probe_poke(frame_t f)
{
    unsigned addr = frq1_be32(f, FR_ARG0);
    unsigned ctl = frq1_be32(f, FR_ARG1);
    unsigned val = frq1_be32(f, FR_DATA);
    unsigned width = (ctl >> 8) & 3u;
    unsigned restore = ctl & 1u;
    unsigned old, back, after = 0;

    if (addr & (stride_of(width) - 1u))
        return FW_EINVAL;

    old = rd(addr, width);
    wr(addr, width, val);
    back = rd(addr, width);
    if (restore) {
        wr(addr, width, old);
        after = rd(addr, width);
    }

    f[FR_DATA] = 1;                             /* version                */
    f[FR_DATA + 1] = (unsigned char)width;
    f[FR_DATA + 2] = (unsigned char)restore;
    f[FR_DATA + 3] = 0;
    frq1_put32(f, FR_DATA + 4, addr);
    frq1_put32(f, FR_DATA + 8, old);
    frq1_put32(f, FR_DATA + 12, val);
    frq1_put32(f, FR_DATA + 16, back);
    frq1_put32(f, FR_DATA + 20, after);
    return FW_OK;
}

/* RING (0x44).  arg0 bit0 = arm, otherwise just report.
 *
 * With the capture block alive, does arming it actually make the HARDWARE
 * PRODUCER move?
 *
 * It arms through tsif_start(), so it does so only on a board whose row has
 * a measured TS sequence; anywhere else tsif_start() refuses and the report
 * says so (byte 2 = 1) with the registers as they were.
 *
 * Nothing here feeds the block, so a write pointer that stays put is the
 * expected answer and not a failure; what the report is for is telling
 * "armed and idle" apart from "the registers did not even take".
 */
int probe_ring(frame_t f)
{
    unsigned arm = frq1_be32(f, FR_ARG0) & 1u;
    int rc = FW_OK;

    if (arm) {
        rc = tsif_start();
        if (rc == FW_OK)
            rc = ring_init();
    }

    f[FR_DATA] = 1;                             /* version                */
    f[FR_DATA + 1] = (unsigned char)arm;
    f[FR_DATA + 2] = (unsigned char)(rc == FW_OK ? 0 : 1);
    f[FR_DATA + 3] = 0;
    frq1_put32(f, FR_DATA + 4, MMIO(TSCAP_MODE));
    frq1_put32(f, FR_DATA + 8, MMIO(TSCAP_BASE));
    frq1_put32(f, FR_DATA + 12, MMIO(TSCAP_LENGTH));
    frq1_put32(f, FR_DATA + 16, MMIO(TSCAP_WRITE));
    frq1_put32(f, FR_DATA + 20, MMIO(TSCAP_READ));
    frq1_put32(f, FR_DATA + 24, ring_used());
    frq1_put32(f, FR_DATA + 28, MMIO(TSCAP_R010));
    /* The arming result is reported in the payload, not as the frame's
     * status: "the registers read back as X" is the answer either way. */
    return FW_OK;
}

/* I2CSCAN (0x45).  arg0 = the first 7-bit address.  arg1: bits 0-7 how many
 * (0 means 16, at most 32), bit8 picks the 0x00FD8FE0 instance instead of
 * 0x01684FE0.
 *
 * Two words rather than three fields packed into one: the frame has them,
 * and "where to start" and "how much, on which controller" are different
 * questions.
 *
 * WHY IT HAS TO BE HERE AND NOT ON THE HOST.  The same controller can be
 * driven from the host, but only with an untouched loader -- and on an
 * untouched FZ2 the 0x01684FE0 controller is dead, because the unit that
 * owns it is not powered.  Waking it needs our firmware installed, which
 * is exactly the state the host-side route cannot be in.  So the scan
 * comes in here, where it can run once the firmware has woken it.
 *
 * A WINDOW AT A TIME.  We are inside the loader's control transfer while this
 * runs: 128 addresses, each with the core's bounded spin, is long enough to
 * look like a hang from the host.  The host asks for a window and loops.
 *
 * This goes straight to the core with an explicit base, deliberately
 * bypassing the board's transport choice (config/devices/fz2.json says
 * fz2 is "aux-mailbox", and that transport has no implementation).  A
 * scan is not a driver: it asks the hardware who is there.
 */
int probe_i2cscan(frame_t f)
{
    unsigned first = frq1_be32(f, FR_ARG0) & 0x7fu;
    unsigned ctl = frq1_be32(f, FR_ARG1);
    unsigned n = ctl & 0xffu;
    unsigned which = (ctl >> 8) & 1u;
    unsigned base = which ? I2C_FD8F_BASE : I2C_1684_BASE;
    unsigned acks = 0, i;
    int setup_rc, last_rc = FW_OK;

    if (n == 0u)
        n = 16u;
    if (n > 32u || first + n > 0x80u)
        return FW_EINVAL;

    /* The 0x00FD8FE0 instance does not answer a write until its block is
     * clocked and out of reset; the 0x01684FE0 one is documented as needing
     * neither (mb86m01.h).  fj_setup() alone fails with FW_EIO on the
     * USB-side instance even though that whole region is live (measured).
     *
     * Read-modify-write, one bit each, because writing the whole word here
     * makes every USB control transfer time out -- which is the transfer we
     * are answering on (the DANGER note in mb86m01.h). */
    if (which && fw_is_dione()) {
        /* dione also needs front-end power, its own pins and the block
         * out of reset (fe_dione.c).  Clock and reset alone were measured
         * to leave fj_setup() at FW_EIO there. */
        setup_rc = dione_fe_up();
    } else {
        if (which) {
            MMIO(SYS_REG_010) = MMIO(SYS_REG_010) | I2C_CLK_BIT;
            MMIO(SYS_REG_014) = MMIO(SYS_REG_014) & ~I2C_RESET_BIT;
        }
        setup_rc = fj_setup(base);
    }
    if (setup_rc == FW_OK) {
        for (i = 0; i < n; i++) {
            unsigned char b = 0;
            last_rc = fj_xfer(base, first + i, (const unsigned char *)0, 0,
                              &b, 1);
            if (last_rc == FW_OK)
                acks |= 1u << i;
        }
    }

    f[FR_DATA] = 1;                             /* version                */
    f[FR_DATA + 1] = (unsigned char)which;
    f[FR_DATA + 2] = (unsigned char)n;
    f[FR_DATA + 3] = (unsigned char)first;
    frq1_put32(f, FR_DATA + 4, base);
    frq1_put32(f, FR_DATA + 8, (unsigned)setup_rc);
    frq1_put32(f, FR_DATA + 12, acks);
    frq1_put32(f, FR_DATA + 16, (unsigned)last_rc);
    /* The setup result is in the payload, not the frame's status: "the
     * controller did not come up" is an answer about the chip, and the host
     * wants to see which base it was asked about either way. */
    return FW_OK;
}

/* EPDMA (0x47).  arg0 = source address, arg1 = length | k << 16 | steps << 24.
 *
 * THE ONE THING THE HOST CANNOT DO.  Everything else about the memory-to-USB
 * block (mb86m01.h's USB_DMA_* block) can be driven over 0xBC from outside,
 * and that was measured not to move a byte.  The endpoint's own start bit
 * cannot: it sits in the low byte of USB_EP_CONFIG(k), and that write needs
 * USB_EP_GUARD set to 8 in the SAME sequence -- the host's writes are one
 * control transfer each and the resident loader puts the guard back to 0xF
 * when each one completes.  Writing the whole word with all four bytes
 * enabled would clear the 0xFF60 the loader left in the middle two, which is
 * the write that killed EP0 on an FZ2.  So the sequence lives here.
 *
 * STEPWISE.  Every step is one bit of arg1's top byte, so the
 * host walks it up and sees which one changed what -- or which one wedged
 * the chip.  Step 0 performs nothing and only reports, which is the "before"
 * reading.
 *
 *     bit0   stop:  RUN = 0, CMD = 0
 *     bit1   arm:   CTL = 0x40001000, LEN = length, ADDR = source
 *     bit2   start: CMD = 2, RUN = 1
 *     bit3   the endpoint's start bit, guard 8 around it   <-- the point
 *     bit4   the pipe: USB_PIPE_CTL |= 1, then |= 0x00010000
 *     bit5   the kick: USB_PIPE_KICK = 0x00010000
 *
 * Reports the five DMA registers, the cause word and the endpoint's pair,
 * read AFTER the steps.  Whether it worked is one comparison on the host:
 * ADDR advanced by LEN, or the cause bit is set, or bytes came out.
 *
 * NOT A FEATURE.  This is a question asked of the chip; nothing in this
 * firmware calls it.
 */
int probe_epdma(frame_t f)
{
    unsigned src = frq1_be32(f, FR_ARG0);
    unsigned ctl = frq1_be32(f, FR_ARG1);
    unsigned len = ctl & 0xffffu;
    unsigned k = (ctl >> 16) & 0xfu;
    unsigned steps = (ctl >> 24) & 0x3fu;

    /* The block moves words, and a misaligned source would be read from the
     * wrong place rather than faulting. */
    if (src & 3u)
        return FW_EINVAL;
    if (steps & 0x02u) {
        if (len == 0u || (len & 3u))
            return FW_EINVAL;
    }

    if (steps & 0x01u) {
        MMIO(USB_DMA_RUN) = 0;
        MMIO(USB_DMA_CMD) = 0;
    }
    if (steps & 0x02u) {
        MMIO(USB_DMA_CTL) = USB_DMA_CTL_RUN;
        MMIO(USB_DMA_LEN) = len;
        MMIO(USB_DMA_ADDR) = src;
    }
    if (steps & 0x04u) {
        MMIO(USB_DMA_CMD) = 2u;
        MMIO(USB_DMA_RUN) = 1u;
    }
    if (steps & 0x08u) {
        /* The guard selects which bytes of the word the write reaches, and
         * bit0 of the enable is the TOP byte, so 8 is the bottom one -- the
         * byte the start bit is in.  Same shape as usb_ep_burst's commit. */
        MMIO(USB_EP_GUARD) = USB_EP_OPEN;
        MMIO(USB_EP_CONFIG(k)) = 1u;
        MMIO(USB_EP_GUARD) = USB_EP_CLOSED;
    }
    if (steps & 0x10u) {
        MMIO(USB_PIPE_CTL) = MMIO(USB_PIPE_CTL) | 1u;
        MMIO(USB_PIPE_CTL) = MMIO(USB_PIPE_CTL) | 0x00010000u;
    }
    if (steps & 0x20u)
        MMIO(USB_PIPE_KICK) = 0x00010000u;

    f[FR_DATA] = 1;                             /* version                */
    f[FR_DATA + 1] = (unsigned char)steps;
    f[FR_DATA + 2] = (unsigned char)k;
    f[FR_DATA + 3] = 0;
    frq1_put32(f, FR_DATA + 4, MMIO(USB_DMA_CMD));
    frq1_put32(f, FR_DATA + 8, MMIO(USB_DMA_RUN));
    frq1_put32(f, FR_DATA + 12, MMIO(USB_DMA_ADDR));
    frq1_put32(f, FR_DATA + 16, MMIO(USB_DMA_LEN));
    frq1_put32(f, FR_DATA + 20, MMIO(USB_DMA_CTL));
    frq1_put32(f, FR_DATA + 24, MMIO(USB_DMA_CAUSE));
    frq1_put32(f, FR_DATA + 28, MMIO(USB_EP_CONFIG(k)));
    frq1_put32(f, FR_DATA + 32, MMIO(USB_EP_STATUS(k)));
    return FW_OK;
}
