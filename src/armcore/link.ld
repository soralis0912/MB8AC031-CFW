/* Placement of our ARM image, in the core's own addresses.
 *
 *     ITCM  0x00000000, 64 KiB   vectors, code, constants
 *     DTCM  0x00100000, 64 KiB   .bss, stack, the work queue and our mailbox
 *                                at the top
 *
 * The main FR sees the same two at its window (+0 and +0x80000).  These
 * addresses hold on a GV-M2TV (measured; docs/arm.md).
 *
 * No .data.  Initial values for DTCM could be carried (the main FR could
 * write them into the window's DTCM band), but nothing here needs them yet,
 * so the ASSERT keeps the image to ITCM and a zeroed .bss.
 */
OUTPUT_FORMAT("elf32-bigarm")
OUTPUT_ARCH(arm)
ENTRY(arm_reset)

ARM_ITCM      = 0x00000000;
ARM_ITCM_SIZE = 0x00010000;
ARM_DTCM      = 0x00100000;
ARM_DTCM_SIZE = 0x00010000;

/* The mailbox is the last 256 bytes of DTCM (src/shared/armcore.h,
 * ARM_MBOX), the work queue the 0x500 bytes below it (src/shared/armipc.h,
 * ARM_IPC), the buffer the 0x400 bytes below that (ARM_BUF), and the stack
 * grows down from just below the buffer.  This is the
 * only place the stack top is written: vectors.S loads this symbol. */
ARM_MBOX      = ARM_DTCM + ARM_DTCM_SIZE - 0x100;
ARM_IPC       = ARM_MBOX - 0x500;
ARM_BUF       = ARM_IPC - 0x400;
ARM_STACK_TOP = ARM_BUF;
ARM_STACK_MIN = 0x800;          /* bytes kept free for the stack */

SECTIONS
{
    . = ARM_ITCM;
    /* One output section, so build.py's `objcopy -j .text` takes all of
     * it, vectors first. */
    .text : {
        KEEP(*(.vectors))
        *(.text*)
        *(.rodata*)
        . = ALIGN(4);
    }
    _text_end = .;

    .data : { *(.data*) }

    . = ARM_DTCM;
    .bss (NOLOAD) : {
        _bss_start = .;
        *(.bss*) *(COMMON)
        . = ALIGN(4);
        _bss_end = .;
    }

    /DISCARD/ : { *(.comment) *(.note*) *(.ARM.attributes) *(.ARM.exidx*) }

    ASSERT(SIZEOF(.data) == 0,
           "arm: .data is not empty. Make it const or assign it at run time")
    ASSERT(_text_end <= ARM_ITCM + ARM_ITCM_SIZE,
           "arm: image does not fit in ITCM")
    ASSERT(_bss_end + ARM_STACK_MIN <= ARM_STACK_TOP,
           "arm: .bss leaves too little stack below the work queue")
}
