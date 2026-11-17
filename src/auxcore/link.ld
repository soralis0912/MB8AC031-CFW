/* Placement of the auxiliary unit's image.
 *
 * It goes in the RAM window of aux SRAM (0x017A2D00). Our container
 * carries it there.
 *
 * Why it goes there, and the map of the range, are at AUX_MAIN_BASE below.
 *
 * No .data. There is no mechanism here to deliver initialised writable
 * data, so the ASSERT rejects it.
 */
OUTPUT_FORMAT("elf32-fr30")
OUTPUT_ARCH(fr30)
ENTRY(aux_entry)

/* Addresses are as seen from the auxiliary core (local)
 * = main-side address - AUX_UNIT_BASE.  Linking at main-side addresses
 * makes the core crash as soon as it runs. Peripheral registers
 * (0x01684xxx) have the same address from both sides, so the C code uses
 * them as is. */
AUX_UNIT_BASE = 0x01700000;

/* Placed in the RAM window for one reason: we can check that it landed.
 *
 *     0x017A0000..0x017B0000   readable and writable. 16 KiB mirrored
 *                              every 0x4000 (0x017A0000 / 0x017A4000 /
 *                              0x017A8000 / 0x017B0000 return the same
 *                              value; measured). A loaded image is
 *                              visible from the main FR (measured)
 *     0x017E0000               closed to both reads and writes from the
 *                              main FR. Returns 0 before and after the
 *                              unit is woken, even with code loaded there
 *                              (measured)
 *
 * In code space we could never check whether the image landed.  In the
 * RAM window we can read it back.
 *
 * The range, as this image uses it:
 *     0x017A0000..0x017A2CD0   left alone
 *     0x017A2CD0..0x017A3900   free, 3,120 B  <- here
 *     0x017A39xx               AUX_ALIVE / AUX_* words (mb86m01.h), left
 *                              alone
 *     0x017A3F00..0x017A3F8C   our request slot and alive marker (read
 *                              from the main side)
 */
AUX_MAIN_BASE = 0x017a2d00;             /* where the container puts it
                                         * (main-side address)          */
AUX_BASE  = AUX_MAIN_BASE - AUX_UNIT_BASE;      /* local 0x000A2D00    */
/* Stack top.  This is the only place the value is written: entry.S loads
 * this symbol, and config/auxcore.json and tests/test_aux.py refer here.
 *
 * Main side 0x017A38F8 = local 0x000A38F8: the top of the free range, 8
 * bytes below the AUX_ALIVE word at 0x017A3900.  The first two words of our
 * image read back from 0x017A2D00 as 0x9F8F000A 0x38F89F8C, i.e. this
 * value (measured).  We take the whole free range. */
AUX_STACK = 0x017a38f8 - AUX_UNIT_BASE;
AUX_CAP   = AUX_STACK;                  /* code must not reach the stack */

SECTIONS
{
    . = AUX_BASE;
    .text : { *(.text*) *(.rodata*) }
    _text_end = .;

    .bss (NOLOAD) : { *(.bss*) *(COMMON) }
    _bss_end = .;

    /DISCARD/ : { *(.comment) *(.note*) }

    ASSERT(SIZEOF(.data) == 0,
           "aux: .data is not empty. Make it const or assign it at run time")
    /* Code and stack share the same RAM window, so what we guard is the
     * gap between them. The end of the code must not reach the stack
     * (AUX_STACK is the top; the stack grows down). */
    ASSERT(_bss_end <= AUX_CAP,
           "aux: image reaches the stack area")
    ASSERT(AUX_BASE >= 0x000a2cd0,
           "aux: overlaps the range below 0x017A2CD0 that is left alone")
    ASSERT(AUX_STACK < 0x000a3900,
           "aux: stack top reaches the AUX_ALIVE word at 0x017A3900")
}
