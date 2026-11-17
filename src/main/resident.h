/* resident.h -- the 256 KiB the firmware lives in.
 *
 * A memory map, nothing more.  Nothing outside this firmware checks it;
 * the path that works on hardware is FRC1 (docs/frc1.md).
 *
 * The layout is kept because it is still the right one.  Our code goes at
 * the bottom, the things that change at run time go above RES_IMMUT_END with
 * the stack, and RES_META is a known-safe ceiling for them.  Keeping statics
 * out of the code region also means a build can be checked by diffing memory
 * after it runs, which tests/test_frq1.py does.
 */
#ifndef RESIDENT_H
#define RESIDENT_H

#define RES_BASE        0x80380000u     /* where the code goes              */
#define RES_IMMUT_END   0x803b0000u     /* code below here, .bss above      */
#define RES_META        0x803befe0u     /* ceiling for .bss                 */
#define RES_STACK_TOP   0x803bfff8u     /* above .bss, so they never meet   */

#endif
