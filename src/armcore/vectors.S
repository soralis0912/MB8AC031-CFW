/* src/armcore/vectors.S -- the exception vectors and the reset path of our
 * ARM image.
 *
 * This is ARM code (ARM926EJ-S class, ARMv5TE, big-endian), not FR.  It is
 * built with arm-none-eabi and placed at the core's address 0 (ITCM), which
 * the main FR sees at the window's start (src/shared/armcore.h).
 *
 * After reset the core runs from address 0 in supervisor mode with IRQ and
 * FIQ masked, MMU and caches off.  We keep it that way: no interrupts, no
 * MMU, no caches.  Polling a mailbox needs none of them.
 *
 * Reset does four things:
 *   1. turn DTCM on at 0x00100000 (CP15 c9, c1, 0: base in bits 31:12,
 *      enable in bit 0; the size field is read-only and kept)
 *   2. set the stack (ARM_STACK_TOP from link.ld, below the work queue)
 *   3. clear .bss
 *   4. call arm_main, which does not return
 *
 * Every other vector records which one it was in the mailbox and stops.
 * A fault is the one thing the main side cannot otherwise see.
 *
 * The one exception is a data abort while arm_main has set PROBING in the
 * mailbox: then the handler records FAR/FSR, clears PROBING and returns to
 * the instruction after the faulting one, so a probe of an address that
 * does not answer costs one request, not the image.  The handler has no
 * stack: sp_abt is used as a base register and r0 is parked in the
 * mailbox's SCRATCH word.
 */
#include "armcore.h"

	.syntax	unified
	.arm
	.section .vectors, "ax", %progbits
	.globl	arm_vectors
arm_vectors:
	b	arm_reset		/* 0x00 reset                 */
	b	arm_undef		/* 0x04 undefined instruction */
	b	arm_swi			/* 0x08 software interrupt    */
	b	arm_pabt		/* 0x0c prefetch abort        */
	b	arm_dabt		/* 0x10 data abort            */
	b	arm_rsvd		/* 0x14 reserved              */
	b	arm_irq			/* 0x18 IRQ (never enabled)   */
	b	arm_fiq			/* 0x1c FIQ (never enabled)   */

	.text
	.globl	arm_reset
arm_reset:
	/* 1. DTCM on at ARM_DTCM.  Keep bits 11:1 (size and reserved), replace
	 * the base, set enable. */
	mrc	p15, 0, r0, c9, c1, 0
	ldr	r1, =0x00000ffe
	and	r0, r0, r1
	ldr	r1, =(ARM_DTCM | 1)
	orr	r0, r0, r1
	mcr	p15, 0, r0, c9, c1, 0

	/* 2. stack */
	ldr	sp, =ARM_STACK_TOP

	/* 3. .bss (word aligned by link.ld) */
	ldr	r0, =_bss_start
	ldr	r1, =_bss_end
	mov	r2, #0
1:	cmp	r0, r1
	strlo	r2, [r0], #4
	blo	1b

	/* CP15 reads for the OP_CP15 request: r0 = which, returns r0. */
	b	4f
	.globl	arm_cp15
arm_cp15:
	cmp	r0, #0
	mrceq	p15, 0, r0, c0, c0, 0
	bxeq	lr
	cmp	r0, #1
	mrceq	p15, 0, r0, c0, c0, 1
	bxeq	lr
	cmp	r0, #2
	mrceq	p15, 0, r0, c0, c0, 2
	bxeq	lr
	cmp	r0, #3
	mrceq	p15, 0, r0, c1, c0, 0
	bxeq	lr
	cmp	r0, #4
	mrceq	p15, 0, r0, c9, c1, 0
	bxeq	lr
	cmp	r0, #5
	mrceq	p15, 0, r0, c9, c1, 1
	bxeq	lr
	mvn	r0, #0
	bx	lr

4:	/* 4. C */
	bl	arm_main
	b	arm_halt

/* Traps: r0 = vector index, then record and stop. */
arm_undef:	mov	r0, #1
		b	arm_trap
arm_swi:	mov	r0, #2
		b	arm_trap
arm_pabt:	mov	r0, #3
		b	arm_trap
arm_dabt:
	ldr	sp, =ARM_MBOX			/* sp_abt: banked, free here */
	str	r0, [sp, #ARM_MB_SCRATCH]
	ldr	r0, [sp, #ARM_MB_PROBING]
	cmp	r0, #0
	beq	1f
	mrc	p15, 0, r0, c6, c0, 0		/* FAR */
	str	r0, [sp, #ARM_MB_ABORT_FAR]
	mrc	p15, 0, r0, c5, c0, 0		/* FSR */
	str	r0, [sp, #ARM_MB_ABORT_FSR]
	mov	r0, #0
	str	r0, [sp, #ARM_MB_PROBING]	/* 0 = it aborted */
	ldr	r0, [sp, #ARM_MB_SCRATCH]
	subs	pc, lr, #4			/* skip the faulting access */
1:	ldr	r0, [sp, #ARM_MB_SCRATCH]
	mov	r0, #4
	b	arm_trap
arm_rsvd:	mov	r0, #5
		b	arm_trap
arm_irq:	mov	r0, #6
		b	arm_trap
arm_fiq:	mov	r0, #7
		b	arm_trap

	.globl	arm_trap
arm_trap:
	ldr	r1, =ARM_MBOX
	ldr	r2, =ARM_FAULT_BASE
	orr	r2, r2, r0
	str	r2, [r1, #ARM_MB_FAULT]
	str	lr, [r1, #ARM_MB_FAULT_LR]
	/* fall through */

	.globl	arm_halt
arm_halt:
	mov	r0, #0
	mcr	p15, 0, r0, c7, c0, 4	/* wait for interrupt (none will come) */
	b	arm_halt

	.ltorg
