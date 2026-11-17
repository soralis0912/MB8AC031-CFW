/* src/auxcore/auxmain.c -- I2C request handler on the auxiliary unit.
 *
 * This program runs on a different CPU. It is a separate image from the
 * main FR CFW, built for the same FR ISA with the same toolchain. See
 * README.md for why it is needed.
 *
 * What it does:
 *
 *   1. Open the controller (clock and reset; cpwin_setup).
 *   2. Bring up the controller with fj_setup.
 *   3. Poll the request slot; when a request arrives, run fj_xfer and
 *      write the result.
 *
 * There is no state machine here. fj_xfer in ../shared/fjcore.c takes the
 * base address as an argument, so the same code that the main-side tests
 * (tests/test_i2c.py) check also runs here. A second hand-written
 * byte-by-byte sequence would only give one more thing that could be wrong.
 *
 * No doorbell (0x01700080) and no interrupts. The peer is our own
 * program, so there is no interface to match. Polling a slot works without
 * knowing how the interrupt vectors are laid out, so it keeps the unknowns
 * down.
 */
#include "fw.h"
#include "mb86m01.h"
#include "fjcore.h"

/* --- Request slot -----------------------------------------------------
 * Lives in aux-side SRAM. The main FR can read and write that range
 * (measured: wrote deadbeef to 0x017A3F80 and read it back), so the main
 * side fills and reads the slot directly. No doorbell is needed.
 *
 * All fields are big-endian words.
 *
 * Addresses are as seen from the auxiliary core (local)
 * = main-side address - 0x01700000.  Linking at main-side addresses makes
 * the core crash as soon as it runs.
 *
 * The values are written as subtractions so that the same numbers as the
 * `AUX_*` in the main side's `mb86m01.h` are visible here, which makes a
 * mismatch easier to spot. Peripheral registers (`0x01684xxx`) have the
 * same address from both sides, so nothing is subtracted for them
 * (src/auxcore/README.md). */
#define AX_UNIT_BASE    0x01700000u

#define AX_SLOT         (0x017a3f00u - AX_UNIT_BASE)

#define AX_MAGIC        0x00u   /* 'AXI2'. Slots without it are ignored */
#define AX_SEQ          0x04u   /* changes per request. 0 is not used   */
#define AX_OP           0x08u   /* 1 = write / 2 = write then read      */
#define AX_ADDR         0x0cu   /* 7-bit slave address                  */
#define AX_WLEN         0x10u
#define AX_RLEN         0x14u
#define AX_RESULT       0x18u   /* written by us. Signed FW_* code      */
#define AX_ACK          0x1cu   /* written by us. Equals SEQ when done  */
#define AX_WDATA        0x20u   /* 40 bytes                             */
#define AX_RDATA        0x48u   /* 40 bytes                             */

#define AX_MAGIC_VALUE  0x41584932u     /* 'AXI2' */

/* --- Alive marker -----------------------------------------------------
 * Answers "did the core run?" with a single word.
 *
 * Without it, every attempt to find the start-up mechanism would mean
 * sending an I2C request and waiting for the ack. With no I2C device
 * present no ack comes back either, so "the core did not run" and "the
 * core ran but I2C failed" could not be told apart.
 *
 * Our marker lives at our own address, away from AUX_ALIVE in
 * `mb86m01.h`.
 *
 * The start-up marker is written once; the loop counter on every pass.
 * A growing counter also shows that the polling loop is running.
 */
#define AX_ALIVE        (0x017a3f70u - AX_UNIT_BASE)     /* 'AX1C', written once            */
#define AX_ALIVE_MAGIC  0x41583143u     /* 'AX1C' */
#define AX_LOOPS        (0x017a3f74u - AX_UNIT_BASE)     /* loop count, +1 per pass         */
#define AX_SETUP_RC     (0x017a3f78u - AX_UNIT_BASE)
/* Expose the I2C registers we see to the main side. It cannot see them
 * itself: 0x01684xxx is on the auxiliary unit's bus and is not in the
 * main FR's map (measured). So to work out on the main side why fj_setup
 * returned -2, we have to copy them out here. */
#define AX_I2C_CLK_RB   (0x017a3f7cu - AX_UNIT_BASE)
#define AX_I2C_RST_RB   (0x017a3f80u - AX_UNIT_BASE)
#define AX_I2C_DIV_RB   (0x017a3f84u - AX_UNIT_BASE)
#define AX_I2C_ST_RB    (0x017a3f88u - AX_UNIT_BASE)     /* fj_setup result, signed         */
#define AX_OP_WRITE     1u
#define AX_OP_WRITE_READ 2u

/* --- How to reach the block --------------------------------------------
 * The main FR writes this word before it starts the core, and it is read
 * once.  Two ways are known, and they differ per chip:
 *
 *   AX_MODE_GATE (0, the default; what the container leaves behind)
 *       Drive the controller directly, with no window.  Not seen working
 *       on hardware.
 *
 *   AX_MODE_CPWIN ('CPW1')
 *       Measured on a cold PIX-DT295.  The 0x0168 band reads 0 until the
 *       control-plane half-word 0x00F1010C is set to 0x000B from THIS
 *       core; set back to its previous value, the band reads 0
 *       again.  The main FR must not write that word: doing so drops USB
 *       (measured).  So every access to the band happens between cp_open()
 *       and cp_close(), here.  Inside the band, the I2C sub-block needs
 *       bit4 of +0x10 set and bit4 of +0x14 cleared (cpwin_setup).
 */
#define AX_MODE         (0x017a3f90u - AX_UNIT_BASE)
#define AX_MODE_GATE    0u
#define AX_MODE_CPWIN   0x43505731u     /* 'CPW1' */

/* The slot is plain memory shared with the main FR, so do not assume
 * alignment. The main side's frq1.h does the same for the same reason. */
typedef volatile unsigned char *slot_t;

static unsigned ax_be32(slot_t s, unsigned off)
{
    return ((unsigned)s[off] << 24) | ((unsigned)s[off + 1] << 16)
         | ((unsigned)s[off + 2] << 8) | (unsigned)s[off + 3];
}

/* Write whole words. Do not write bytes.
 *
 * The slot is in aux SRAM, where a byte write becomes a read-modify-write
 * of a wider unit, and that read is stale. So writing one byte paints the
 * neighbouring bytes back with old values (measured: with byte writes, a
 * word that should read `0x00000003` read back as `0xF000F003`).
 *
 * On hardware, with byte writes the first request of a scan (seq 1) goes
 * through and the second (seq 2) stalls. The first one matches only
 * because the stale byte happens to be 0. The aux side does answer; the
 * main side reads a corrupted `ACK`.
 *
 * Every field offset in the slot is a multiple of 4, so one word per field
 * is enough. */
static void ax_put32(slot_t s, unsigned off, unsigned v)
{
    *(volatile unsigned *)(s + off) = v;
}

#define AUX_I2C_BLOCK_CLK   0x01684010u
#define AUX_I2C_BLOCK_RST   0x01684014u

static void publish_regs(void)
{
    MMIO(AX_I2C_CLK_RB) = MMIO(AUX_I2C_BLOCK_CLK);
    MMIO(AX_I2C_RST_RB) = MMIO(AUX_I2C_BLOCK_RST);
    MMIO(AX_I2C_DIV_RB) = MMIO(I2C_1684_BASE + 0x08u);
    MMIO(AX_I2C_ST_RB) = MMIO(I2C_1684_BASE + 0x00u);
}

/* --- The control-plane window (AX_MODE_CPWIN) ------------------------- */
#define CP_WORD         0x00f1010cu     /* half-word */
#define CP_OPEN         0x000bu
#define I2C_PIN_SEL     0x00f10114u     /* half-word; bits 5:4 = 0b10 */
#define I2C_PIN_MASK    0xffcfu
#define I2C_PIN_VAL     0x0020u
#define BAND_C0         0x016840c0u
#define BAND_BIT4       0x00000010u

static unsigned short cp_saved;

static void cp_open(void)
{
    cp_saved = MMIO16(CP_WORD);
    MMIO16(CP_WORD) = CP_OPEN;
}

static void cp_close(void)
{
    MMIO16(CP_WORD) = cp_saved;
}

/* The bring-up, measured on a cold PIX-DT295 by leaving each write out in
 * turn.  Call between cp_open() and cp_close().
 *
 *   - 0x00F10114 bits 5:4 = 0b10 routes the pins to this controller;
 *     without it every transfer times out.
 *   - 0x016840C0 = 0 lets the I2C sub-block run; left as it comes up,
 *     fj_setup fails (FW_EIO).
 *   - bit4 of +0x10 set (clock) and bit4 of +0x14 cleared (reset); without
 *     either, fj_setup fails (FW_EIO).
 *
 * Not needed (each left out, the demodulator still locked): bit20 of
 * +0x10 / +0x14, bit0 of 0x016840B0, and the 0x01684FF4 / 0x01684FF8
 * words.  This is the same set the main FR's 0x00FD8FE0 instance needs
 * (src/main/i2c/fujitsu.c). */
static int cpwin_setup(void)
{
    MMIO16(I2C_PIN_SEL) = (unsigned short)
        ((MMIO16(I2C_PIN_SEL) & I2C_PIN_MASK) | I2C_PIN_VAL);
    MMIO(BAND_C0) = 0u;
    MMIO(AUX_I2C_BLOCK_CLK) = MMIO(AUX_I2C_BLOCK_CLK) | BAND_BIT4;
    MMIO(AUX_I2C_BLOCK_RST) = MMIO(AUX_I2C_BLOCK_RST) & ~BAND_BIT4;
    return fj_setup(I2C_1684_BASE);
}

/* --- One request ------------------------------------------------------ */

static int serve(slot_t s)
{
    unsigned char wb[FJ_MAX_XFER], rb[FJ_MAX_XFER];
    unsigned op = ax_be32(s, AX_OP);
    unsigned addr = ax_be32(s, AX_ADDR);
    unsigned wn = ax_be32(s, AX_WLEN);
    unsigned rn = ax_be32(s, AX_RLEN);
    unsigned i;

    if (op != AX_OP_WRITE && op != AX_OP_WRITE_READ)
        return FW_EINVAL;
    if (op == AX_OP_WRITE)
        rn = 0;
    /* fj_xfer's limit is the total per request. Refuse anything larger;
     * truncating and reporting success would be worse. */
    if (wn + rn > FJ_MAX_XFER || addr > 0x7fu)
        return FW_EINVAL;

    for (i = 0; i < wn; i++)
        wb[i] = s[AX_WDATA + i];
    for (i = 0; i < rn; i++)
        rb[i] = 0;

    i = (unsigned)fj_xfer(I2C_1684_BASE, addr, wb, wn, rb, rn);
    /* Store the read bytes as words too (same reason as ax_put32 above).
     * The tail is padded with 0, so up to 3 bytes past rn in the 40-byte
     * RDATA become 0. Harmless: the main side reads only rlen bytes. */
    for (unsigned k = 0; k < rn; k += 4u) {
        unsigned w = 0, j;
        for (j = 0; j < 4u; j++)
            w = (w << 8) | (k + j < rn ? (unsigned)rb[k + j] : 0u);
        ax_put32(s, AX_RDATA + k, w);
    }
    return (int)i;
}

/* --- Polling loop ----------------------------------------------------- */

/* Bring the controller up the way AX_MODE says, and publish what the
 * registers read (the main FR cannot read the band itself). */
static int bring_up(unsigned mode)
{
    int rc;

    if (mode == AX_MODE_CPWIN) {
        cp_open();
        rc = cpwin_setup();
        publish_regs();
        cp_close();
        return rc;
    }
    rc = fj_setup(I2C_1684_BASE);
    publish_regs();
    return rc;
}

void aux_main(void)
{
    slot_t s = (slot_t)AX_SLOT;
    unsigned served = 0;
    unsigned mode = MMIO(AX_MODE);
    int setup;

    if (mode != AX_MODE_CPWIN)
        mode = AX_MODE_GATE;
    setup = bring_up(mode);

    /* Reaching here means the core is running. Whatever I2C does is a
     * separate matter, so the marker does not depend on fj_setup's
     * result. */
    MMIO(AX_ALIVE) = AX_ALIVE_MAGIC;
    MMIO(AX_SETUP_RC) = (unsigned)setup;
    MMIO(AX_LOOPS) = 0;

    for (;;) {
        unsigned seq;

        MMIO(AX_LOOPS) = MMIO(AX_LOOPS) + 1u;

        if (ax_be32(s, AX_MAGIC) != AX_MAGIC_VALUE)
            continue;
        seq = ax_be32(s, AX_SEQ);
        if (seq == 0u || seq == served)
            continue;

        /* If the controller is not up, retry setup on every request. If
         * it still fails, answer each request with that value.
         *
         * Giving up after one try at start-up, on hardware `fj_setup`
         * returns -2 (the "aux-side fj_setup" field of the main-side
         * report) and every request gets -2 even though the mailbox
         * works. That is why a 112-address scan got no ACK at all, not
         * because no device was there.
         *
         * The block's gate may open after we start: the main FR may wake
         * unit 0x016 after we are already running. So keep retrying. */
        if (setup != FW_OK) {
            setup = bring_up(mode);
            MMIO(AX_SETUP_RC) = (unsigned)setup;
        }
        if (setup != FW_OK) {
            ax_put32(s, AX_RESULT, (unsigned)setup);
        } else if (mode == AX_MODE_CPWIN) {
            int rc;

            cp_open();
            rc = serve(s);
            cp_close();
            ax_put32(s, AX_RESULT, (unsigned)rc);
        } else {
            ax_put32(s, AX_RESULT, (unsigned)serve(s));
        }
        /* ACK goes last. The host reads the result after it sees ACK, so
         * the reverse order would let it read a stale result. */
        ax_put32(s, AX_ACK, seq);
        served = seq;
    }
}
