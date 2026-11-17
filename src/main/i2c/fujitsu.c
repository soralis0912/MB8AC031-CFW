/* i2c/fujitsu.c -- the controller inside the USB core block, 0x00FD8FE0.
 *
 * The transport of the GV-M2TV (dione) and the PIX-DT295 (selene), both
 * measured.  The state machine is in fjcore.c; what is here is everything
 * specific to this instance: the clock and reset gating of the block, then
 * the I2C pins and the front-end reset.
 *
 * On a PIX-DT295 this is the whole route from a cold start: the main FR
 * reaches the demodulators 0x10 / 0x11 through this instance with nothing
 * running on the auxiliary core (measured).
 *
 * ORDER MATTERS AND IT IS NOT OPTIONAL.  Nothing written to this I2C block
 * lands until the block is clocked and out of reset -- the divider written
 * beforehand reads back 0.  That is why the gating comes before fj_setup and why i2c.c
 * owns a cached, idempotent init: a driver that reached fj_xfer() without
 * this having run would write to a dead block and then wait forever for a
 * bit0 that cannot arrive.  tests/test_i2c.py catches a driver that does
 * that.
 *
 * Only what the bus needs is written (measured on a GV-M2TV and a
 * PIX-DT295, leaving out one write at a time from a cold start): the
 * clock, the reset, the two reset bits of 0x00FD80C0, then fj_setup's
 * divider and control.  0x00FD80B0 bit0 and the registers at +0x14 / +0x18
 * changed nothing on either board and are left alone.
 *
 * This firmware does not save the previous values and put them back when
 * done: it is the resident, not a guest in someone else's session, and
 * there is nothing after it to hand the block back to.
 */
#include "fw.h"
#include "drivers.h"
#include "mb86m01.h"
#include "regs_dione.h"
#include "fjcore.h"

static int fujitsu_init(const struct board *b)
{
    int rc;

    (void)b;

    /* The block first, then what is outside it: the block can be set up
     * and read back with the pins and the front end still as they came
     * up, and the bus is not used before this function returns.  Checked
     * in this order from a power-up on both boards. */

    /* Clock the block and release its reset.  Without either, nothing
     * written to the block lands and every transfer times out (measured
     * on both boards).  Read-modify-write on the clock register -- see the
     * warning in mb86m01.h about what a blind write there costs.  bit20 of
     * 0x00FD8010 is not needed (measured on a PIX-DT295: the demodulators
     * answer with it set or clear), so it is left alone.
     */
    MMIO(SYS_REG_010) |= I2C_CLK_BIT;
    MMIO(SYS_REG_014) &= ~(unsigned)I2C_RESET_BIT;
    /* 0x00FD80C0 comes up as 3.  Both bits hold the block back: with either
     * one left at 1 the divider and control do not take and the bus does
     * not work (measured on both boards).  The other bits are left as
     * they are. */
    MMIO(SYS_REG_0C0) &= ~(unsigned)I2C_HOLD_BITS;
    /* The divider and the control register.  fj_setup reads both back,
     * which is the liveness check. */
    rc = fj_setup(I2C_FD8F_BASE);
    if (rc != FW_OK)
        return rc;

    if (fw_is_dione()) {
        /* The I2C pins: bit3:2 of the high half-word (regs_dione.h).  The
         * front-end power on dione is fe_dione.c's, not this file's. */
        MMIO(DN_I2C_PIN) = (MMIO(DN_I2C_PIN) & ~DN_I2C_PIN_MASK) |
                           DN_I2C_PIN_ON;
    } else {
        /* The I2C pins: bit5:4 of the high half-word = 0b10.  Without it
         * the setup passes but no device ACKs (measured on a PIX-DT295). */
        MMIO(RF_ENABLE_114) = (MMIO(RF_ENABLE_114) &
                               ~(unsigned)RF_EN_CLEAR) | RF_EN_BIT;
        /* Release the front-end reset: 0x00F10018 bit1 (the level), then
         * 0x00F10014 bit1 (drive it), half-word read-modify-write.  Without
         * it no device ACKs; clearing 0x00F10014 bit1 again stops every
         * ACK (measured on a PIX-DT295).  The level is set before the pin
         * is driven so that the pin never drives the reset level. */
        MMIO16(FE_RESET_DAT) = (unsigned short)(MMIO16(FE_RESET_DAT) |
                                                FE_RESET_BIT);
        MMIO16(FE_RESET_DIR) = (unsigned short)(MMIO16(FE_RESET_DIR) |
                                                FE_RESET_BIT);
    }
    return FW_OK;
}

static int fujitsu_xfer(const struct board *b, unsigned addr,
                        const unsigned char *wb, unsigned wn,
                        unsigned char *rb, unsigned rn)
{
    int rc;

    (void)b;
    /* On dione, only the shapes measured on a GV-M2TV: a write ending in
     * STOP, and a 1-byte read ending in STOP.  That is all the host's
     * tuning tool ever does (register number in one transfer, the byte in
     * the next).  MEASURED: STATUS reading the demodulator with the
     * combined shape -- register number, repeated START, then 1 byte, and
     * 3 bytes for C/N -- disagreed with the host's reads on a locked
     * channel, and the next transfer found the bus busy (E_TIMEOUT) until
     * the device was replugged.  Setting the block up before each transfer
     * did not change that.  So the combined shape is split into two
     * transfers here, and a read of more than one byte is refused until
     * someone measures it.  selene is left as it was. */
    if (fw_is_dione()) {
        if (rn > 1u)
            return FW_ENOTIMPL;
        if (wn && rn) {
            rc = fj_xfer(I2C_FD8F_BASE, addr, wb, wn, 0, 0u);
            if (rc != FW_OK)
                return rc;
            return fj_xfer(I2C_FD8F_BASE, addr, 0, 0u, rb, rn);
        }
    }
    return fj_xfer(I2C_FD8F_BASE, addr, wb, wn, rb, rn);
}

const struct i2c_ops i2c_fujitsu_ops = {
    "fujitsu-fd8f",
    fujitsu_init,
    fujitsu_xfer,
};
