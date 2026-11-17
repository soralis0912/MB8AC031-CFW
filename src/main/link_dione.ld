/* link_dione.ld -- where the `dione` FRC1 image lands.
 *
 * Same two ways in as link.ld, different memory.  On this generation
 * (MB8AC018 / MB86H5x, measured on a GV-M2TV) the loader state has the
 * external memory controller turned off, and the 0xBC window does not reach
 * DRAM.  So the body cannot live at 0x80380000 the way it does on the
 * MB8AC031 parts.  It goes into the free internal RAM above the gate's body
 * area:
 *
 *     0x000C5000  the mailbox for the command frame (64 bytes)
 *     0x000C5100  .bss
 *     0x000C5FF8  stack top (the stack grows down towards .bss)
 *
 *     0x000E0000  the reset stub (not used by this path)
 *     0x000E1000  .boot: the gate the hook jumps to (capacity 0x800)
 *     0x000E1800  .text: our body, up to 0x000E9000
 *     0x000E9000  the plaintext loader -- MUST NOT BE TOUCHED
 *
 * WHY WHAT IS WRITTEN LIVES IN THE 0x000C BAND.  In the 0x000E band a store
 * narrower than a word writes the whole word: a byte or half-word store
 * leaves the rest of its word zeroed or garbled (measured on a GV-M2TV,
 * docs/image.md).  Code there is fine -- it is only read -- but .bss and the
 * stack are written with byte and half-word stores the compiler chooses on
 * its own.  In the 0x000C band such stores behave (measured), and the page
 * above the mailbox held only power-on noise when read on a live GV-M2TV,
 * so nothing of the loader's is there.
 *
 * 0x000E0100-0x000E9000 is uninitialised internal RAM (writing it and
 * reading it back on the live GV-M2TV left the loader intact).  The mailbox
 * the host uses for the command frame is separate, at 0x000C5000.
 *
 * The flat image is still taken with `objcopy -j .text`; .boot is a separate
 * section far above it, exactly as on the selene layout.
 */
OUTPUT_FORMAT("elf32-fr30")
OUTPUT_ARCH(fr30)
ENTRY(_start)

BOOT_BASE     = 0x000e1000;
BOOT_CAP      = 0x800;
RES_BASE      = 0x000e1800;     /* body starts just past the gate's area  */
RES_IMMUT_END = 0x000e9000;     /* code below here: the loader above     */
RES_BSS       = 0x000c5100;     /* .bss                                   */
RES_META      = 0x000c5800;     /* ceiling for .bss; 2 KiB of stack above */
RES_STACK_TOP = 0x000c5ff8;     /* top of the mailbox page                */

/* Marks this image as dione's.  ts_dione.c tests its address; the selene
 * link leaves it undefined, so there it is 0. */
fw_dione_image = 1;

SECTIONS
{
  . = BOOT_BASE;
  .boot : { *(.boot) }
  _boot_end = .;

  . = RES_BASE;
  _text_start = .;
  .text : { *(.text.entry) *(.text.dispatch)
            *(.text) *(.text.*) *(.rodata) *(.rodata.*) }
  ASSERT(frq1_dispatch - _start < 2048,
         "frq1_dispatch is more than 2 KB from _start: keep .text.dispatch right after .text.entry")
  _text_end = .;

  . = RES_BSS;
  .bss (NOLOAD) : { *(.bss) *(COMMON) }
  _bss_end = .;

  .data : { *(.data) *(.data.*) }

  ASSERT(_boot_end <= RES_BASE,
         ".boot overruns into RES_BASE; the dione body starts right after the gate area")
  ASSERT(_boot_end - BOOT_BASE <= BOOT_CAP,
         ".boot overruns the body area the hook can reach")
  ASSERT(_text_end <= RES_IMMUT_END,
         "code overruns into the loader")
  ASSERT(SIZEOF(.data) == 0,
         "this image cannot carry .data -- use const or assign at run time")
  ASSERT(_bss_end <= RES_META,
         ".bss overruns into the self-description (RES_META)")
}
