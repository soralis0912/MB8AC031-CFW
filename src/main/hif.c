/* hif.c -- the window the host reads.
 *
 * Measured on a live FZ2 while the loader was answering, so the contents at
 * power-on are known exactly:
 *
 *     0x00F00000/04  'MB8A' 'C031'   the chip's name, as the host reads it
 *     0x00F00014                     not known what it means
 *     0x00F00020     0x82000000      an address; what it is for is a guess
 *                                    (where the loader starts code)
 *     0x00F00094     "V13012900"+1a  the loader's version, in ASCII
 *     0x00F00220                     a block the host and the chip share
 *
 * The important subtlety: 0x00F00004 is the identifier while the loader is
 * answering and a host command word once a firmware is running.  The same
 * address has two roles either side of boot, which is why a firmware must
 * not assume what it reads there.
 *
 * NOT WRITTEN YET.  What this module should do is publish our own version
 * string and take over the command word, and neither can be done without
 * deciding what we want the host to see -- which is a protocol decision, not
 * a register one.
 */
#include "fw.h"
#include "mb86m01.h"

int hif_init(void)
{
    return FW_ENOTIMPL;
}
