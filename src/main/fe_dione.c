/* fe_dione.c -- the dione front end: power, and I2C transfers for the host.
 *
 * On the GV-M2TV the demodulator (7-bit 0x18) and, behind its gateway
 * register, the tuner sit on the 0x00FD8FE0 controller.  Bringing it up
 * from inside the chip takes three things (docs/i2c.md):
 *
 *   1. front-end power (DN_FE_SEL / DN_FE_DAT): without it nobody ACKs
 *   2. the I2C pins (DN_I2C_PIN) and the block out of reset
 *      (SYS_REG_0C0 bit1:0): i2c/fujitsu.c does both
 *   3. the controller setup (fj_setup, also in i2c/fujitsu.c)
 *
 * WHAT IS NOT HERE: the register sequence that tunes the module.  This
 * tree has no such sequence of its own.  So tuning stays on the host, which sends one I2C transfer per
 * frame (I2CXFER, docs/frq1.md), and this file only moves the bytes.
 */
#include "fw.h"
#include "frq1.h"
#include "drivers.h"
#include "mb86m01.h"
#include "regs_dione.h"
#include "fjcore.h"

/* dione reaches I2C one way only (0x00FD8FE0), whatever the row says. */
extern const struct i2c_ops i2c_fujitsu_ops;

/* About 50 ms after power-on: one read of the endpoint status takes about
 * 185 ns (measured in the TS copy loop), so 5000 reads are about 1 ms.  The host tool waits 50 ms here; whether the
 * module needs it has not been tried.  The read has no side effect. */
#define POWER_SETTLE_READS  (50u * 5000u)

static unsigned get_high(unsigned reg)
{
    return MMIO(reg) >> 16;
}

static void set_high(unsigned reg, unsigned v)
{
    MMIO(reg) = (v << 16) | (MMIO(reg) & 0xffffu);
}

/* Power on the front end if it is off, then bring the I2C block up.
 * Safe to call on every frame: power is only written when it reads off,
 * and the I2C setup is the same writes again. */
int dione_fe_up(void)
{
    unsigned i;

    if (!fw_is_dione())
        return FW_ENOTIMPL;

    if (get_high(DN_FE_SEL) != DN_FE_SEL_ON ||
        get_high(DN_FE_DAT) != DN_FE_DAT_ON) {
        set_high(DN_FE_SEL, DN_FE_SEL_ON);
        set_high(DN_FE_DAT, DN_FE_DAT_ON);
        for (i = 0; i < POWER_SETTLE_READS; i++)
            (void)MMIO(USB_EP0_STATUS);
    }
    return i2c_fujitsu_ops.init(&fw_board);
}

/* I2CXFER (0x46).  arg0: bits 6:0 the 7-bit address, bits 15:8 how many
 * bytes to write (taken from the data field), bits 23:16 how many to read
 * after a restart.  Written + read is 1..40.  The bytes read come back at
 * the start of the data field.
 *
 * One transfer per frame, on purpose: the host keeps the sequence and the
 * timing (between frames it can wait as long as the module wants).
 */
int dione_i2cxfer(frame_t f)
{
    unsigned a0 = frq1_be32(f, FR_ARG0);
    unsigned addr = a0 & 0x7fu;
    unsigned wn = (a0 >> 8) & 0xffu;
    unsigned rn = (a0 >> 16) & 0xffu;
    /* The bytes are sent from the frame and read back into it: no copy.
     * When the stack was in the 0x000E band, where a store narrower than a
     * word writes the whole word (docs/image.md), a copy on the stack
     * turned "fe c0 0d 49" into "00 00 00 49" and the module ended up
     * holding SDA low.  The stack has moved to the 0x000C band since, but
     * there is still no reason to copy.  fj_xfer() sends all of the bytes
     * before it reads any.
     */
    unsigned char *data = (unsigned char *)&f[FR_DATA];
    unsigned i;
    int rc;

    if (!fw_is_dione())
        return FW_ENOTIMPL;
    if ((a0 & 0xff000080u) || wn + rn == 0u || wn + rn > FJ_MAX_XFER)
        return FW_EINVAL;

    rc = dione_fe_up();
    if (rc != FW_OK)
        return rc;
    rc = fj_xfer(I2C_FD8F_BASE, addr, data, wn, data, rn);
    if (rc != FW_OK)
        return rc;

    for (i = rn; i < FJ_MAX_XFER; i++)
        f[FR_DATA + i] = 0u;
    return FW_OK;
}
