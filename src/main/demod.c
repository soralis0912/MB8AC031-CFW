/* demod.c -- the demodulator front door.
 *
 * This LSI does not demodulate.  The demodulator sits in a module outside
 * the chip and which one it is depends on the product, so this file is a
 * dispatcher, not a driver: it hands the call to the demodulator driver
 * that config/devices/ gave this image's board.
 *
 * The drivers are in src/main/demod/.  Today there is one, tc9052x.c, covering
 * the Toshiba family that every identified board in this family carries --
 * but the TC90512, 90517, 90522 and 90532XBG are not the same part, so the
 * table keeps each board's bus address and how sure we are of it, and a
 * board with no demodulator driver gets FW_ENOTIMPL from here.
 *
 * `source` is FW_SRC_T or FW_SRC_S.  A board with no address for one of
 * them does not have that side established, and says so.
 */
#include "fw.h"
#include "drivers.h"

int demod_init(unsigned source)
{
    const struct demod_ops *o = fw_board.demod;

    if (source >= FW_SRC_COUNT)
        return FW_EINVAL;
    if (!o || !o->init)
        return FW_ENOTIMPL;
    return o->init(&fw_board, source);
}

int demod_locked(unsigned source, int *locked)
{
    const struct demod_ops *o = fw_board.demod;

    if (locked)
        *locked = 0;
    if (source >= FW_SRC_COUNT)
        return FW_EINVAL;
    if (!o || !o->locked)
        return FW_ENOTIMPL;
    return o->locked(&fw_board, source, locked);
}

int demod_cnr_raw(unsigned source, unsigned *raw)
{
    const struct demod_ops *o = fw_board.demod;

    if (raw)
        *raw = 0;
    if (source >= FW_SRC_COUNT)
        return FW_EINVAL;
    if (!o || !o->cnr_raw)
        return FW_ENOTIMPL;
    return o->cnr_raw(&fw_board, source, raw);
}
