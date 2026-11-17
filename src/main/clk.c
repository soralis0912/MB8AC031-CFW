/* clk.c -- clock and reset.
 *
 * clk_init is a stub: by the time we run, the loader has already brought the
 * clocks up for the USB path it was using, and re-initialising them blind
 * would be a good way to lose the connection we are answering on.  What this
 * module is really for is the per-block clocks the TS and RF paths need, and
 * those are not identified yet.
 */
#include "fw.h"

int clk_init(void)
{
    return FW_ENOTIMPL;
}
