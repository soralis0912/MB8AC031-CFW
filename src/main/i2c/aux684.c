/* i2c/aux684.c -- the controller the auxiliary unit drives, 0x01684FE0.
 *
 * The MB8AC031 transport, and the reason this file exists rather than
 * i2c/mailbox.c being finished.
 *
 * WHY THIS BLOCK.  On an FZ2, driving the 0x00FD8FE0 controller from the
 * host gets through setup but never completes a transfer (i2c/mailbox.c),
 * so that is not how that board reaches its tuner.  The assumption here
 * (not confirmed on hardware) is that auxiliary unit 0x017 has a controller
 * of the same design at 0x01684FE0, with the same register layout and setup
 * values as the 0x00FD8FE0 one, so fjcore.c drives it unchanged.
 *
 * This firmware drives the controller itself rather than asking the
 * auxiliary unit over the mailbox.  That is a deliberate choice, not a
 * shortcut: the mailbox request cannot be built yet (see i2c/mailbox.c),
 * while the controller's own registers are described.
 *
 * WHAT IS MEASURED, and it is the one thing that matters here: FROM THE MAIN
 * FR, 0x01684FE0 READS 0 AND TAKES NO WRITES on a live FZ2 (the loader path;
 * PEEK / POKE, docs/frq1.md).  The block sits on the auxiliary unit's bus,
 * not on the main FR's map.  So fj_setup() below fails its read-back and
 * init reports FW_EIO, and a transfer reports FW_ENOTIMPL instead of
 * driving a bus that is not there.
 *
 * NO CLOCK GATING HERE.  This block is assumed to be clocked and out of
 * reset already (not confirmed on hardware) -- unlike the 0x00FD8FE0
 * instance, which needs four gates opened first (i2c/fujitsu.c).  The two
 * setup registers that instance writes, +0x14 and +0x18, are left alone.
 */
#include "fw.h"
#include "drivers.h"
#include "mb86m01.h"
#include "fjcore.h"

static int aux684_init(const struct board *b)
{
    (void)b;

    /* The setup order assumed here (not confirmed on hardware): a preset
     * into the control register, then the divider, then the real control
     * value.  fj_setup() writes the last two and checks them.
     */
    MMIO(I2C_1684_BASE + I2C_O_CONTROL) = I2C_CT_PRESET;
    return fj_setup(I2C_1684_BASE);
}

static int aux684_xfer(const struct board *b, unsigned addr,
                       const unsigned char *wb, unsigned wn,
                       unsigned char *rb, unsigned rn)
{
    (void)b;
    (void)addr;
    (void)wb;
    (void)wn;
    (void)rb;
    (void)rn;

    /* RF_ENABLE_114 IS NOT RAISED HERE: it is unproven on this product
     * (mb86m01.h), and this
     * driver has nothing to gain by it -- the block it drives reads back
     * zeros and takes no writes from the main FR on a live FZ2, which is
     * measured and is why no board selects this transport.
     *
     * The semaphore at RF_I2C_SEM is left alone too -- mb86m01.h says why.
     */
    return FW_ENOTIMPL;
}

const struct i2c_ops i2c_aux684_ops = {
    "fujitsu-1684",
    aux684_init,
    aux684_xfer,
};
