/* Where the C build lands inside the 256 KiB the firmware owns.
 *
 * Two sections, far apart, because there are two ways in.
 *
 * .text at RES_BASE is the body: code and rodata.  .bss goes ABOVE
 * RES_IMMUT_END, with the stack, not mixed into .text.  Nothing outside
 * this firmware depends on that split, but it is kept: it is what lets a
 * run be checked by diffing memory afterwards, and the FRC1 path has to
 * declare .bss as an explicit zero segment anyway and wants it in one
 * piece.
 *
 * .bss is NOLOAD: the bytes are not in the flat output.  The FRC1 target
 * writes a zero segment of exactly _bss_end - RES_IMMUT_END bytes, so they
 * really are zero on the device -- that path writes only declared bytes, so
 * without it statics would start at whatever the SRAM held.
 *
 * .boot is the other way in: eight bytes at the loader's hook (0x000EA6FC)
 * are replaced with a jump to it, and that stub's immediate is 20 bits, so
 * it has to sit below 0x100000.  The body area at BOOT_BASE is BOOT_CAP
 * bytes long, which is why the size is checked
 * here.  It is a separate section at a far lower address than .text, so the
 * flat image is taken with `objcopy -j .text` -- without that, the output
 * would span from 0xE1000 to 0x80381000.
 *
 * .data has to stay empty.  There is no loader step here that could copy
 * initialised data from one place to another, and anything with an
 * initialiser would have to sit in .text to be loaded at all -- where it
 * must not be written.  Use const (which goes to .rodata in .text) or assign
 * at run time.  The ASSERT below is the "fails loudly".
 */
OUTPUT_FORMAT("elf32-fr30")
OUTPUT_ARCH(fr30)
ENTRY(_start)

BOOT_BASE     = 0x000e1000;     /* where the hook can reach           */
BOOT_CAP      = 0x800;          /* the body area's capacity           */
RES_BASE      = 0x80380000;
RES_IMMUT_END = 0x803b0000;
RES_META      = 0x803befe0;

SECTIONS
{
  . = BOOT_BASE;
  .boot : { *(.boot) }
  _boot_end = .;

  . = RES_BASE;
  /* The dispatcher goes right after _start. `call` is R_FR30_12_PCREL and
   * reaches only +-2 KB, so frq1_dispatch sits in .text.dispatch right
   * after .text.entry and the body can grow freely behind it. */
  _text_start = .;
  .text : { *(.text.entry) *(.text.dispatch)
            *(.text) *(.text.*) *(.rodata) *(.rodata.*) }

  ASSERT(frq1_dispatch - _start < 2048, "frq1_dispatch is more than 2 KB from _start: keep .text.dispatch right after .text.entry")
  _text_end = .;

  /* Anything that gets written lives outside .text. */
  . = RES_IMMUT_END;
  .bss (NOLOAD) : { KEEP(*(.bss.mbox)) *(.bss) *(COMMON) }
  _bss_end = .;

  /* The FRC1 entry's four mailboxes (entry_frc1_selene.S) open .bss, and
   * the entry names them by their uncached view.  Only that build has
   * them. */
  ASSERT(!DEFINED(frc1_mbox) || frc1_mbox == RES_IMMUT_END,
         "the mailboxes must open .bss")
  ASSERT(!DEFINED(frc1_mbox) || (frc1_mbox & 0x3fffffff) == FRC1_MBOX,
         "FRC1_MBOX in entry_frc1_selene.S is not the mailboxes' uncached view")

  .data : { *(.data) *(.data.*) }

  ASSERT(_boot_end - BOOT_BASE <= BOOT_CAP,
         ".boot overruns the body area the hook can reach")
  ASSERT(_text_end <= RES_IMMUT_END,
         "code overruns into the area .bss and the stack use")
  ASSERT(SIZEOF(.data) == 0,
         "this image cannot carry .data -- use const or assign at run time")
  ASSERT(_bss_end <= RES_META,
         ".bss overruns into the self-description (RES_META)")
}
