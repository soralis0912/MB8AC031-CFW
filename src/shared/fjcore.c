/* shared/fjcore.c -- the Fujitsu I2C controller's state machine.
 *
 * Shared by both transports in config/drivers.json, because both are this
 * same controller at different addresses (mb86m01.h explains why there are
 * two).  Taking the base as an argument is what lets the auxiliary-unit
 * route (src/auxcore/) run the same code at the other address.
 *
 * One transfer is: wait for the bus to be free, send the address with
 * START, then the bytes to write, then (after a repeated START if there was
 * a write) the bytes to read, ACKing all but the last, and end with STOP.
 * Every byte ends the same way: clear DONE, write the shadow, wait for DONE.
 *
 * WHAT IS MEASURED.  Driven like this, from the host on the GV-M2TV and
 * PIX-DT295 and from inside the chip on both, the demodulators and tuners
 * answer (tests/test_i2c.py models the same handshake).  The spin limit of
 * I2C_SPIN reads per byte is a generous bound, not a tuned one.
 *
 * TWO CHOICES OF OURS.  NACK is looked at only after an address; a NACK
 * on a data byte is not seen by this code (none has been observed from the
 * devices we drive, and none was looked for).  And STOP drops START, RESTART, ACK and DONE from the
 * shadow in one write, so the next transfer starts from the setup value
 * with nothing left over from this one.
 *
 * RUNNING THE LOOP FROM INSIDE THE CHIP IS MEASURED (GV-M2TV, PIX-DT295).
 * The byte wait has one more step: **before waiting for bit0 to come up, wait for it to be
 * 0**, up to I2C_SETTLE reads.  Driven from the host that step is invisible,
 * because every access is separated by a ~60 us control transfer and the bit
 * has always settled; from in here the reads come back-to-back, and without
 * it the previous byte's "done" is still standing and gets read as this
 * one's.
 *
 * This code still never assumes a wait succeeded: every spin is bounded, a
 * timeout leaves the bus with a STOP on it, and nothing retries silently.
 *
 * The control register is kept as a shadow copy rather than read back,
 * because several of its bits do not read back as written.
 */
#include "fw.h"
#include "mb86m01.h"
#include "fjcore.h"

#define STATUS(b)       MMIO((b) + I2C_O_STATUS)
#define CONTROL(b)      MMIO((b) + I2C_O_CONTROL)
#define DIVIDER(b)      MMIO((b) + I2C_O_DIVIDER)
#define DATA(b)         MMIO((b) + I2C_O_DATA)

static unsigned ctl;            /* shadow of the control register */

/* Spin until the controller says the byte is done.  Returns FW_EIO on a bus
 * error, FW_ETIMEOUT if it never came.
 */
static int byte_done(unsigned base)
{
    unsigned i;

    /* Let the bit we just cleared actually go down.  Bounded and NOT an
     * error if it does not: spend up to I2C_SETTLE reads here and then carry
     * on regardless.
     */
    for (i = 0; i < I2C_SETTLE; i++) {
        unsigned v = CONTROL(base);

        if (v & I2C_CT_ERROR)
            return FW_EIO;
        if (!(v & I2C_CT_DONE))
            break;
    }

    for (i = 0; i < I2C_SPIN; i++) {
        unsigned v = CONTROL(base);

        if (v & I2C_CT_ERROR)
            return FW_EIO;
        if (v & I2C_CT_DONE)
            return FW_OK;
    }
    return FW_ETIMEOUT;
}

static int wait_idle(unsigned base)
{
    unsigned i;

    for (i = 0; i < I2C_SPIN; i++)
        if (!(STATUS(base) & I2C_ST_BUSY))
            return FW_OK;
    return FW_ETIMEOUT;
}

/* STOP.  Bits 0, 3, 4 and 5 all go down together. */
static void stop(unsigned base)
{
    ctl &= ~(unsigned)(I2C_CT_DONE | I2C_CT_ACK | I2C_CT_START |
                       I2C_CT_RESTART);
    CONTROL(base) = ctl;
}

/* Put one address byte on the bus and check it was answered. */
static int address(unsigned base, unsigned addr, unsigned reading,
                   unsigned restart)
{
    int rc;

    DATA(base) = ((addr & 0x7fu) << 1) | (reading ? 1u : 0u);
    if (restart)
        ctl |= I2C_CT_RESTART;
    else
        ctl |= I2C_CT_START;
    ctl &= ~(unsigned)I2C_CT_DONE;
    CONTROL(base) = ctl;

    rc = byte_done(base);
    /* RESTART is a one-shot request: left standing in the shadow, every
     * read-byte write after it asks for another repeated START instead of
     * clocking a byte in, DATA still holds the address (0x21 for 0x10
     * reading), and the STOP lands while the slave drives SDA.  Measured on
     * a PIX-DT295: a tuner read through the demodulator's gate returned
     * 0x21 and the bus stayed busy until a replug. */
    ctl &= ~(unsigned)I2C_CT_RESTART;
    if (rc != FW_OK)
        return rc;
    /* The ACK is only meaningful here, right after an address. */
    if (STATUS(base) & I2C_ST_NACK)
        return FW_EIO;
    return FW_OK;
}

int fj_setup(unsigned base)
{
    DIVIDER(base) = I2C_DIV_SETUP;
    ctl = I2C_CT_INITIAL;
    CONTROL(base) = ctl;

    /* Did it take?  bit0 of the control register is a status bit, so it is
     * masked off; the divider reads back with bit7 added to what was
     * written.
     */
    if ((CONTROL(base) & ~(unsigned)I2C_CT_DONE) != I2C_CT_INITIAL)
        return FW_EIO;
    if ((DIVIDER(base) & I2C_DIV_MASK) != I2C_DIV_SETUP)
        return FW_EIO;
    return FW_OK;
}

int fj_xfer(unsigned base, unsigned addr, const unsigned char *wb, unsigned wn,
            unsigned char *rb, unsigned rn)
{
    unsigned i;
    int rc;

    if (wn + rn > FJ_MAX_XFER || (wn && !wb) || (rn && !rb))
        return FW_EINVAL;
    if (!wn && !rn)
        return FW_EINVAL;

    rc = wait_idle(base);
    if (rc != FW_OK)
        return rc;

    rc = address(base, addr, wn ? 0u : 1u, 0u);
    if (rc != FW_OK)
        goto fail;

    for (i = 0; i < wn; i++) {
        DATA(base) = wb[i];
        ctl &= ~(unsigned)I2C_CT_DONE;
        CONTROL(base) = ctl;
        rc = byte_done(base);
        if (rc != FW_OK)
            goto fail;
    }

    if (rn) {
        /* Turning the bus around after a write needs a restart. */
        if (wn) {
            rc = address(base, addr, 1u, 1u);
            if (rc != FW_OK)
                goto fail;
        }
        for (i = 0; i < rn; i++) {
            /* ACK every byte except the last one. */
            if (i + 1u < rn)
                ctl |= I2C_CT_ACK;
            else
                ctl &= ~(unsigned)I2C_CT_ACK;
            ctl &= ~(unsigned)I2C_CT_DONE;
            CONTROL(base) = ctl;
            rc = byte_done(base);
            if (rc != FW_OK)
                goto fail;
            rb[i] = (unsigned char)(DATA(base) & 0xffu);
        }
    }

    stop(base);
    return FW_OK;

fail:
    stop(base);
    return rc;
}
