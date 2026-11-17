/* entry_frc1_selene.S -- the way in on the plaintext (FRC1) path.
 *
 * TWO WAYS IN, ONE FIRMWARE.  entry.S is the servicer's: a resident servicer
 * body `call`s RES_BASE with r4 = frame.  This one is the loader's: eight
 * bytes at its hook, 0x000EA6FC, are replaced with a
 * jump to here, so a host control transfer reaches us with no servicer in
 * the picture at all.  Both end up in the same frq1_dispatch.
 *
 * This path is the one that is known to work on real hardware, which is why
 * the firmware is packaged for it.
 *
 * WHERE THIS LIVES.  The hook stub is `ldi:20 #target,r12`, a 20-bit
 * immediate, so whatever it jumps to must sit below 0x100000 -- the main
 * image at 0x80380000 is out of reach.  The body area at 0x000E1000 is 0x800
 * bytes, so this section goes there and
 * reaches the rest with a 32-bit load.  link_selene.ld places it and checks the size.
 *
 * ENTERED WITH:
 *     r10  the USB setup packet              must be preserved
 *     r11  the loader's request descriptor   must be preserved
 *     r11+0x00  the address the host asked for
 *     r11+0x46  wValue's low byte (the sub-opcode)
 *     r11+0x48  wLength
 *
 * TWO WAYS TO HAND US A FRAME.
 *
 *   staging   a 64-byte OUT aimed at 0x000C4620, the loader's own buffer.
 *             The frame arrives in the transfer itself and is answered in
 *             place.  There is one such buffer, so two host programs that
 *             talk to the chip at once overwrite each other's reply between
 *             the OUT and the read-back.
 *   mailbox   the host writes the 64-byte frame into one of four mailboxes
 *             of ours with plain writes, then rings our sub-opcode
 *             (0xC1) with exactly four bytes aimed at that mailbox.  The frame is run
 *             and answered in that mailbox.  One mailbox per program, and
 *             they do not disturb each other (the frames still run one at a
 *             time, inside the doorbell's transfer).
 *
 * THE MAILBOXES are the first 256 bytes of .bss (frc1_mbox, the linker
 * script puts it at RES_IMMUT_END), so nothing else can be placed there and
 * the FRC1 zero segment clears them at install.  The host reaches that RAM
 * only through the uncached view 0x003Bxxxx, which is also the view the
 * container's segments are written through, so the host names the mailbox
 * by that address and we run the frame from it too: no cache stands between
 * what the host wrote and what frq1_dispatch reads.
 *
 * THE GATE.  A stray OUT must not be taken for a command, so where it is
 * aimed, its length and its sub-opcode all have to agree before we touch
 * anything: 0x40 bytes at the staging address, or 4 bytes at one of the
 * four mailboxes, and in both cases sub-opcode 0xC1.  The loader itself
 * gives only 0 and 1 a meaning (the plain window and read-modify-write),
 * so with no firmware of ours a 0xC1 request is a plain write to the same
 * place and does no harm.  The frame's own magic is the fourth check and
 * frq1_dispatch already does it -- it answers 0 when it handled the frame
 * and 1 when the frame was not ours, so the magic check is not duplicated
 * here.
 *
 * WHAT IS NOT OURS GOES BACK.  Those eight bytes were the loader's request
 * handling, so if we do not make its decision here, nothing does: the host's plain
 * window would stop working the moment this firmware is installed.
 * Sub-opcode 1 is the read-modify-write path, everything else the plain
 * window path.  We only reach the resume below with a sub-opcode that is not
 * ours, and ours is not 1, so the plain path is the right one.
 *
 * THE ADDRESSES ARE CONSTANTS HERE.  staging, complete and resume_plain are
 * per-target fields of the FRC1 container, but every selene target we
 * build for uses the same values.  build.py refuses to pack a
 * container whose targets disagree with these three numbers, so they cannot
 * drift apart silently.
 */
	.equ	SUBOP,          0xc1		/* wValue's low byte: ours   */
	.equ	WLENGTH,        0x40		/* the frame is 64 bytes     */
	.equ	DOORBELL,       0x4		/* a mailbox's doorbell      */
	.equ	STAGING,        0x000c4620	/* the 64-byte OUT buffer    */
	.equ	FRC1_MBOX,      0x003b0000	/* mailbox 0, uncached view  */
	.equ	MBOX_PICK,      0xffffff3f	/* clears the mailbox number */
	.equ	COMPLETE,       0x000ea8c8	/* finish the transfer       */
	.equ	RESUME_PLAIN,   0x000ea75a	/* the loader's plain path  */
	.equ	RES_STACK_TOP,  0x803bfff8	/* keep in step with resident.h */

	.section .boot,"ax",@progbits
	.globl	frc1_entry
frc1_entry:
	/* Only r0, r1, r2 and r13 are touched before we know the request is
	 * ours; r12 only on the way out. */
	ld	@r11, r0		; r0 = where the host aimed: the frame
	ldi:8	#0x48, r13
	lduh	@(r13, r11), r2		; r2 = wLength

	ldi:32	#STAGING, r1		; aimed at staging: 64 bytes,
	cmp	r1, r0
	beq	frc1_at_staging
	cmp	#DOORBELL, r2		; else a mailbox: 4 bytes,
	bne	frc1_resume
	ldi:32	#MBOX_PICK, r1
	and	r0, r1			; r1 = the address, mailbox number cleared
	ldi:32	#FRC1_MBOX, r2
	cmp	r2, r1			; at one of ours and nowhere else
	bne	frc1_resume
	bra	frc1_check_subop
frc1_at_staging:
	ldi:8	#WLENGTH, r1
	cmp	r1, r2
	bne	frc1_resume

frc1_check_subop:
	ldi:8	#0x46, r13
	ldub	@(r13, r11), r1		; and, last, our sub-opcode
	ldi:8	#SUBOP, r13
	cmp	r13, r1
	bne	frc1_resume


	/* Ours to look at.  Run C on our own stack: the loader's has no
	 * documented headroom, and ours is above the immutable region so
	 * nothing it writes can invalidate the resident's check word. */
	ldi:32	#RES_STACK_TOP, r1
	mov	r15, r2
	mov	r1, r15
	st	r2, @-r15		; the loader's sp
	st	r10, @-r15		; the setup packet, for the resume
	st	r11, @-r15		; the request descriptor
	st	rp, @-r15

	mov	r0, r4			; the frame: staging or the mailbox
	ldi:32	#frq1_dispatch, r12
	call	@r12

	ld	@r15+, rp
	ld	@r15+, r11
	ld	@r15+, r10
	ld	@r15+, r2
	mov	r2, r15			; back on the loader's stack

	cmp	#0, r4			; 0 = answered, 1 = not our frame
	bne	frc1_resume
	ldi:32	#COMPLETE, r12
	jmp	@r12

frc1_resume:
	ldi:32	#RESUME_PLAIN, r12
	jmp	@r12

	/* The four mailboxes.  .bss.mbox is placed first in .bss; the linker
	 * script checks that it starts at RES_IMMUT_END and that FRC1_MBOX is
	 * its uncached view. */
	.globl	FRC1_MBOX
	.section .bss.mbox,"aw",@nobits
	.globl	frc1_mbox
	.balign	4
frc1_mbox:
	.space	0x100
