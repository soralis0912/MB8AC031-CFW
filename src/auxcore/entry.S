/* src/auxcore/entry.S -- entry point when the auxiliary unit's core is released.
 *
 * It does two things: set up the stack and call C. Nothing else.
 *
 * The stack top is AUX_STACK, a symbol defined only in link.ld. It is not
 * repeated here: ldi:32 with a symbol is still one instruction, and a
 * second copy is how the stack address ended up different in three places.
 * tests/test_aux.py checks that the first instruction loads that symbol.
 *
 * aux_main never returns (it is an endless loop polling for requests).
 * aux_halt is still there so that, if it ever does return, execution does
 * not fall into an address nobody catches.
 *
 * aux_halt is also the default target in the vector table. The FR reset
 * vector is the last word of its own space (0x017FFFFC for this unit).
 * build.py builds an 80-byte table ending there from these two symbols:
 * 19 default handlers, then the reset entry as the last word.
 */
	.section .text,"ax",@progbits
	.globl	aux_entry
	.globl	aux_halt
aux_entry:
	ldi:32	#AUX_STACK, r15		/* from link.ld (local address) */
	ldi:32	#aux_main, r12
	call	@r12
aux_halt:
	bra	aux_halt
