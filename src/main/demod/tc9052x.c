/* demod/tc9052x.c -- the Toshiba TC9052x demodulator family, over I2C.
 *
 * One demodulator driver, several parts: TC90512 on the one-tuner modules,
 * TC90522 in pairs on the two-tuner ones, TC90517 on the GV-MVP/FZ2's
 * SMT-MJ111, TC90532XBG on the PIX-DT295 family.  They share a register
 * interface, which is why config/devices/ points all of them here and
 * carries only what differs: the bus address, and how sure we are of it.
 *
 * WHAT IS KNOWN.  That the part is in this family, from board markings
 * (those identifications are inferred, not confirmed
 * in silicon, and the table says so per board).
 * That it is an I2C device, and on a PIX-DT295 answers at 0x10 for
 * terrestrial and 0x11 for satellite, measured.  That the
 * register access is a one-byte register number followed by data, with
 * reads done as a separate transfer from writes.  That the C/N figure comes
 * out as a raw register value and is converted to dB on the host, not here.
 *
 * LOCK AND C/N REGISTER NUMBERS.
 *
 *     terrestrial lock   0x80, fail if (v & 0x28); then 0xB0, lock needs
 *                        (v & 0x0F) >= 8
 *     satellite lock     0xC3, locked when !(v & 0x10)
 *     terrestrial C/N    0x8B..0x8D, a 24-bit big-endian raw value
 *     satellite C/N      0xBC..0xBD, a 16-bit raw value
 *
 * The terrestrial pair is measured: on a GV-M2TV it agrees with the host's
 * own lock test on the UHF channels.  The satellite pair is not measured
 * yet.  The addresses are 0x10/0x12 terrestrial and 0x11/0x13 satellite,
 * two units' worth; config/devices/ records them.  0x10/0x11 are measured
 * (above); the second unit is not confirmed on hardware.  All four are
 * READS: this driver writes nothing it is not sure of.
 *
 * BRING-UP IS THE HOST'S.  init() proves only that something answers at
 * the address and returns FW_ENOTIMPL, on both sides.  Whatever the part
 * needs written before it locks, the host sends through I2CXFER along with
 * the tuning sequence.

 * STILL NOT KNOWN.  The tuner's own registers, though the way to reach
 * it is known: register 0xFE on this part is an I2C repeater.  Writing
 * [0xFE, tuner_addr << 1, ...] to the demodulator forwards to the tuner, and
 * [0xFE, (tuner_addr << 1) | 1] followed by a read reads from it.  What to
 * say through it is the part that is missing.
 * docs/demod.md has the whole picture.
 */
#include "fw.h"
#include "drivers.h"
#include "mb86m01.h"

/* The transport is reached through i2c.c, not through b->i2c directly.  That
 * matters: the front door owns the setup sequence and runs it once.  On the
 * Fujitsu controller nothing written to the I2C block lands until the block
 * is clocked, so a transfer that bypassed init() would put nothing on the bus
 * and then wait for a byte that is never going to finish.
 * tests/test_i2c.py catches it.
 */
static unsigned char board_demod_addr(const struct board *b, unsigned source)
{
    return source < FW_SRC_COUNT ? b->demod_addr[source] : 0u;
}

static int reg_write(const struct board *b, unsigned source,
                     unsigned char reg, unsigned char val)
{
    unsigned char addr = board_demod_addr(b, source);

    if (!addr)
        return FW_ENOTIMPL;
    return i2c_reg_write(addr, reg, val);
}

static int reg_read(const struct board *b, unsigned source,
                    unsigned char reg, unsigned char *val)
{
    unsigned char addr = board_demod_addr(b, source);

    if (!addr)
        return FW_ENOTIMPL;
    return i2c_reg_read(addr, reg, val);
}

/* The register numbers -- see the header for which are measured. */
#define T_SYNC          0x80u   /* terrestrial: must have no bit of 0x28  */
#define T_SYNC_BAD      0x28u
#define T_LOCK          0xb0u   /* terrestrial: low nibble >= 8 is locked */
#define T_LOCK_MASK     0x0fu
#define T_LOCK_MIN      8u
#define S_LOCK          0xc3u   /* satellite: bit4 clear is locked        */
#define S_LOCK_BIT      0x10u
#define T_CNR           0x8bu   /* terrestrial: 3 bytes, big endian       */
#define T_CNR_LEN       3u
#define S_CNR           0xbcu   /* satellite: 2 bytes                     */
#define S_CNR_LEN       2u
#define DEMOD_GATEWAY   0xfeu   /* the I2C repeater to the tuner          */

/* Read n consecutive registers, one transfer pair per register.
 */
static int regs_read(const struct board *b, unsigned source,
                     unsigned char reg, unsigned char *buf, unsigned n)
{
    unsigned char addr = board_demod_addr(b, source);

    unsigned i;
    int rc;

    if (!addr)
        return FW_ENOTIMPL;
    if (!buf || !n)
        return FW_EINVAL;
    /* One register at a time.  A multi-byte read has not been measured on
     * every transport (dione refuses it, i2c/fujitsu.c); a register number
     * per byte is the shape the GV-M2TV's tuning tool uses. */
    for (i = 0; i < n; i++) {
        unsigned char r = (unsigned char)(reg + i);

        rc = i2c_xfer_rw(addr, &r, 1u, &buf[i], 1u);
        if (rc != FW_OK)
            return rc;
    }
    return FW_OK;
}

static int tc9052x_init(const struct board *b, unsigned source)
{
    unsigned char addr = board_demod_addr(b, source);
    unsigned char junk;
    int rc;

    if (!addr)
        return FW_ENOTIMPL;         /* this board's address is not known */

    /* Does anything answer?  A bare read: no register is selected and
     * nothing is written, so this cannot disturb the part.
     */
    rc = i2c_read(addr, &junk, 1u);
    if (rc != FW_OK)
        return rc;

    /* Bring-up is the host's (header): stop having proved only that
     * something answers.
     */
    return FW_ENOTIMPL;
}

/* Two reads for terrestrial, one for satellite.  Nothing is written, and a
 * read that fails is reported rather than turned into "not locked" -- the
 * host needs to tell a dead bus from a weak signal.
 */
static int tc9052x_locked(const struct board *b, unsigned source, int *locked)
{
    unsigned char v;
    int rc;

    if (!locked)
        return FW_EINVAL;
    *locked = 0;

    if (FW_SRC_IS_SAT(source)) {
        rc = reg_read(b, source, S_LOCK, &v);
        if (rc != FW_OK)
            return rc;
        *locked = (v & S_LOCK_BIT) ? 0 : 1;
        return FW_OK;
    }

    rc = reg_read(b, source, T_SYNC, &v);
    if (rc != FW_OK)
        return rc;
    if (v & T_SYNC_BAD)
        return FW_OK;               /* answered, and says not synchronised */
    rc = reg_read(b, source, T_LOCK, &v);
    if (rc != FW_OK)
        return rc;
    if ((v & T_LOCK_MASK) >= T_LOCK_MIN)
        *locked = 1;
    return FW_OK;
}

/* The raw register value, not dB.  The conversion belongs on the host: it
 * is a table and a curve per part, and getting it wrong here would hide the
 * number the host could have converted correctly.
 */
static int tc9052x_cnr_raw(const struct board *b, unsigned source,
                           unsigned *raw)
{
    unsigned char v[T_CNR_LEN];
    int rc;

    if (!raw)
        return FW_EINVAL;
    *raw = 0;

    if (FW_SRC_IS_SAT(source)) {
        rc = regs_read(b, source, S_CNR, v, S_CNR_LEN);
        if (rc != FW_OK)
            return rc;
        *raw = ((unsigned)v[0] << 8) | v[1];
        return FW_OK;
    }

    rc = regs_read(b, source, T_CNR, v, T_CNR_LEN);
    if (rc != FW_OK)
        return rc;
    *raw = ((unsigned)v[0] << 16) | ((unsigned)v[1] << 8) | v[2];
    return FW_OK;
}

/* The tuner on this family sits behind this part's I2C repeater, register
 * DEMOD_GATEWAY.  These are the register access with the board's own address
 * filled in from the table; demod_tc9052x_gateway() is the repeater itself.
 */
int demod_tc9052x_reg_write(const struct board *b, unsigned source,
                            unsigned char reg, unsigned char val)
{
    return reg_write(b, source, reg, val);
}

int demod_tc9052x_reg_read(const struct board *b, unsigned source,
                           unsigned char reg, unsigned char *val)
{
    return reg_read(b, source, reg, val);
}

/* Talk to whatever is behind the repeater.
 *
 * Writing [0xFE, addr << 1, ...] to the demodulator forwards the rest to the
 * device at `addr`; [0xFE, (addr << 1) | 1] then a read reads from it.  Both
 * halves go in one transfer here, which is what the transport gives us.
 *
 * NOT USED YET, and that is deliberate: the host tunes through this gateway
 * with I2CXFER (that works on a GV-M2TV), and nothing in this tree says what
 * the tuner wants to hear once it is open.  Opening it with nothing to say would
 * be a blind write for no gain.  src/main/tuner/ is where this gets called
 * from when the tuner's own registers turn up.
 */
int demod_tc9052x_gateway(const struct board *b, unsigned source,
                          unsigned char addr, const unsigned char *wb,
                          unsigned wn, unsigned char *rb, unsigned rn)
{
    unsigned char buf[FW_I2C_MAX];
    unsigned char dst = board_demod_addr(b, source);
    unsigned i;

    if (!dst)
        return FW_ENOTIMPL;
    if (wn + rn + 2u > FW_I2C_MAX)
        return FW_EINVAL;

    buf[0] = DEMOD_GATEWAY;
    buf[1] = (unsigned char)((addr & 0x7fu) << 1);
    if (rn)
        buf[1] |= 1u;
    for (i = 0; i < wn; i++)
        buf[2u + i] = wb[i];
    if (rn)
        return i2c_xfer_rw(dst, buf, 2u + wn, rb, rn);
    return i2c_write(dst, buf, 2u + wn);
}

const struct demod_ops demod_tc9052x_ops = {
    "tc9052x",
    tc9052x_init,
    tc9052x_locked,
    tc9052x_cnr_raw,
};
