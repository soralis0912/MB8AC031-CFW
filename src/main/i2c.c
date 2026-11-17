/* i2c.c -- the I2C front door.
 *
 * There is no single I2C controller on this family, so this file is not a
 * driver.  It is the dispatcher: it takes the transport config/devices/
 * gave this image's board, and offers the shape the rest of the firmware
 * wants (write, read, one register in, one register out) on top of the one
 * primitive a transport has to provide -- write n bytes then read m.
 *
 * The transports are in src/main/i2c/.  Today there are three: the Fujitsu
 * controller at 0x00FD8FE0 (fujitsu.c, which the PIX-DT295 family uses), the
 * same controller at 0x01684FE0 (aux684.c, which the main FR cannot reach on
 * a live FZ2), and asking the auxiliary unit (mailbox.c, which is the FZ2's
 * path and is not finished).  A board whose transport is not known gets FW_ENOTIMPL from
 * every call here rather than a default that happens to be wrong.
 *
 * init() is idempotent: the transport is set up once and the result
 * remembered, because the setup sequence writes clock and reset registers
 * and running it twice under a caller that retries is not something to find
 * out about on hardware.
 */
#include "fw.h"
#include "drivers.h"


static int ready;               /* 0 untried, 1 up, negative = the error */

int i2c_init(void)
{
    const struct i2c_ops *o = fw_board.i2c;
    int rc;

    if (ready > 0)
        return FW_OK;
    if (ready < 0)
        return ready;
    if (!o || !o->init)
        return FW_ENOTIMPL;

    rc = o->init(&fw_board);
    ready = (rc == FW_OK) ? 1 : rc;
    return rc;
}

static int xfer(unsigned addr, const unsigned char *wb, unsigned wn,
                unsigned char *rb, unsigned rn)
{
    const struct i2c_ops *o = fw_board.i2c;
    int rc;

    if (!o || !o->xfer)
        return FW_ENOTIMPL;
    if (wn + rn > FW_I2C_MAX)
        return FW_EINVAL;
    rc = i2c_init();
    if (rc != FW_OK)
        return rc;
    return o->xfer(&fw_board, addr, wb, wn, rb, rn);
}

/* Write n bytes then, with a restart, read m.  The general shape; the four
 * below are the cases the rest of the firmware actually wants.  Exposed
 * because the demodulator's I2C repeater needs a multi-byte write followed
 * by a read, which none of the four covers (demod/tc9052x.c).
 */
int i2c_xfer_rw(unsigned addr, const unsigned char *wb, unsigned wn,
                unsigned char *rb, unsigned rn)
{
    return xfer(addr, wb, wn, rb, rn);
}

int i2c_write(unsigned addr, const unsigned char *buf, unsigned n)
{
    if (!buf || !n)
        return FW_EINVAL;
    return xfer(addr, buf, n, 0, 0u);
}

int i2c_read(unsigned addr, unsigned char *buf, unsigned n)
{
    if (!buf || !n)
        return FW_EINVAL;
    return xfer(addr, 0, 0u, buf, n);
}

int i2c_reg_write(unsigned addr, unsigned char reg, unsigned char val)
{
    unsigned char w[2];

    w[0] = reg;
    w[1] = val;
    return xfer(addr, w, 2u, 0, 0u);
}

/* Register number out, value back, in one transfer with a restart between.
 * The demodulators on this family want the read as its own transfer, which
 * is what the restart in the transport gives.
 */
int i2c_reg_read(unsigned addr, unsigned char reg, unsigned char *val)
{
    if (!val)
        return FW_EINVAL;
    return xfer(addr, &reg, 1u, val, 1u);
}

/* I2CXFER (0x46) on an image that is not dione: one transfer through this
 * board's transport.  The frame's shape is dione_i2cxfer's (fe_dione.c):
 * arg0 bits 6:0 the 7-bit address, bits 15:8 bytes to write (from the data
 * field), bits 23:16 bytes to read after a restart; 1..40 in all.  The
 * bytes read come back at the start of the data field, the rest is 0.
 *
 * Reads go into a buffer here and are copied out after the transfer, so a
 * transport that fills `rb` before it has sent all of `wb` cannot clobber
 * the bytes still to be sent.  The buffer is a word array: byte stores on
 * the stack are not safe in every band (docs/image.md).
 */
int i2c_frame_xfer(frame_t f)
{
    unsigned a0 = frq1_be32(f, FR_ARG0);
    unsigned addr = a0 & 0x7fu;
    unsigned wn = (a0 >> 8) & 0xffu;
    unsigned rn = (a0 >> 16) & 0xffu;
    unsigned rw[(FW_I2C_MAX + 3u) / 4u];
    unsigned char *rb = (unsigned char *)rw;
    unsigned i;
    int rc;

    if ((a0 & 0xff000080u) || wn + rn == 0u || wn + rn > FW_I2C_MAX)
        return FW_EINVAL;
    for (i = 0; i < (FW_I2C_MAX + 3u) / 4u; i++)
        rw[i] = 0u;
    rc = xfer(addr, (const unsigned char *)&f[FR_DATA], wn, rn ? rb : 0, rn);
    if (rc != FW_OK)
        return rc;
    for (i = 0; i < FW_I2C_MAX; i++)
        f[FR_DATA + i] = i < rn ? rb[i] : 0u;
    return FW_OK;
}
