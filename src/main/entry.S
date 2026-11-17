/* entry.S -- the only assembly in this firmware.
 *
 * The servicer `call`s us at RES_BASE with r4 = frame and nothing else set
 * up (assumed; this entry path is not confirmed on hardware -- the path
 * that runs there is FRC1, entry_frc1_*.S).  C needs a stack, so
 * this switches to the image's own, calls in, and puts sp back.  Everything
 * else is C.
 *
 * The stack top is inside the image but ABOVE the immutable region
 * ([0x80380000, 0x803B0000), whose word sum the servicer is assumed to
 * check), so stack
 * traffic cannot invalidate the check word.  Keep it in step with
 * RES_STACK_TOP in resident.h.
 */
	.equ	RES_STACK_TOP, 0x803bfff8

	.section .text.entry,"ax",@progbits
	.globl	_start
_start:
	st	rp, @-r15		; the servicer's RP, on the servicer's stack
	ldi:32	#RES_STACK_TOP, r1
	mov	r15, r0			; keep the caller's sp ...
	mov	r1, r15			; ... and run C on ours
	st	r0, @-r15
	call	frq1_dispatch		; r4 is already the frame
	ld	@r15+, r0
	mov	r0, r15			; back to the caller's stack
	ld	@r15+, rp
	ret
