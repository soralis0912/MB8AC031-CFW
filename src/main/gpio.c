/* gpio.c -- straps and pins.
 *
 * **IT IS A HALF-WORD.** Measured on a live GV-MVP/FZ2: the half-word at
 * 0x00F10004 is 0x8A01, so `half & 0x0C00` is 0x0800.  Read as a 32-bit
 * word the same register gives 0x8A010000, and every field lands 16 bits
 * out.  So this reads it with `lduh`.
 *
 * The fields:
 *
 *     0x00F10004 bit12-13  thought to pick the signature the loader puts
 *                          at 0x00F00000/04: 0 gives 'MB8A' 'C031' (0 on
 *                          the FZ2 above); the
 *                          other values (FSL0801, FSL5540, FSL2886) are
 *                          not confirmed on hardware
 *     0x00F10004 bit10-11  differs between boards (0x0800 on the FZ2
 *                          above).
 *                          This firmware does not read it: each image is
 *                          built for one board
 *
 * The external-bus timing registers at 0x00F1010C..0x00F10134 hold
 * board-dependent values that are already set by the time we run; we do
 * not touch them.
 */
#include "fw.h"
#include "mb86m01.h"

unsigned gpio_straps(void)
{
    return MMIO16(GP_STRAPS);
}
