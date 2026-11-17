/* entry_frc1_dione.S -- the way in on the plaintext (FRC1) path, `dione` loader.
 *
 * Same job as entry_frc1_selene.S, different resident loader.  The MB8AC031
 * generation (`selene`) and the MB86H5x / MB8AC018 one (`dione`) both answer
 * the 0xBC vendor request, but their plaintext loaders are separate builds:
 *
 *     selene   hook 0x000EA6FC   descriptor in r11
 *              address +0x00, sub-opcode +0x46, wLength +0x48
 *     dione    hook 0x000E9F80   descriptor in r10, setup packet in r11
 *              address +0x44, sub-opcode +0x40, wLength +0x48
 *              (bRequest is the byte at r11+1)
 *
 * The dione values are measured on a GV-M2TV (04BB:053A), loader
 * "Dione boot loader copyright FML 2010" / V10091300.
 *
 * THE MAILBOX, NOT THE LOADER'S BUFFER.  This loader's EP0 OUT reader
 * scatters a multi-word payload: word i lands at buffer + 16*i, so a 64-byte
 * transfer carries four usable words and walks over the fields that follow.
 * The host therefore puts the 64-byte FRQ1 frame in a mailbox of our own
 * one word at a time and rings our sub-opcode, 0xC1, with a four-byte payload
 * aimed at that mailbox as the doorbell.  That is why wLength is checked
 * against 4, not 0x40.
 *
 * FOUR MAILBOXES.  0x000C5000, 0x000C5040, 0x000C5080 and 0x000C50C0 are
 * all ours, and the frame is answered in the one the doorbell names.  A
 * frame takes some 34 control transfers, so two host programs sharing one
 * mailbox (a stream recorder and the card reader driver, say) overwrite
 * each other's frames between transfers.  With one mailbox each they do
 * not: the frames themselves run one at a time, inside the doorbell's own
 * transfer.
 *
 * WHAT IS NOT OURS GOES BACK.  The hook replaces eight bytes of the
 * loader's request handling, so its decision is made here: a request that
 * is not 0xBC finishes the transfer (COMPLETE); sub-opcode 1 goes to the
 * loader's read-modify-write path; everything else to its plain window path.
 *
 * WHERE THIS LIVES.  The hook stub is `ldi:20 #target,r12`, a 20-bit
 * immediate, so the gate has to sit below 0x100000.  It goes at
 * 0x000E1000.  The body is linked into the free internal RAM above the body
 * area (0x000E1800), because on this generation the loader state has the
 * external memory controller turned off -- a body
 * at 0x80380000 would not be reachable from the host and would not fetch.
 * src/main/link_dione.ld places both and checks the sizes.
 *
 * ENTERED WITH:
 *     r10  the loader's request descriptor   must be preserved
 *     r11  the USB setup packet              must be preserved
 *     r10+0x40  wValue's low byte (the sub-opcode)
 *     r10+0x44  the address the host asked for
 *     r10+0x48  wLength
 *     r11+1     bRequest
 */
	.equ	SUBOP,          0xc1		/* wValue's low byte: ours   */
	.equ	WLENGTH,        0x4		/* the doorbell is 4 bytes   */
	.equ	STAGING,        0x000c5000	/* the first of four mailboxes */
	.equ	MBOX_PICK,      0xffffff3f	/* clears the mailbox number   */
	.equ	COMPLETE,       0x000e9ff0	/* finish the transfer       */
	.equ	RESUME_PLAIN,   0x000e9fa6	/* the loader's plain path   */
	.equ	RESUME_MASK,    0x000e9f8a	/* the loader's RMW path     */
	.equ	RES_STACK_TOP,  0x000c5ff8	/* keep in step with link_dione.ld */

	.section .boot,"ax",@progbits
	.globl	frc1_entry
frc1_entry:
	ldi:8	#0x01, r13
	ldub	@(r13, r11), r0		; bRequest
	ldi:8	#0xbc, r1
	cmp	r1, r0
	bne	dione_complete

	ldi:8	#0x44, r13
	ld	@(r13, r10), r0		; r0 = where the host aimed: the mailbox
	ldi:32	#MBOX_PICK, r1
	and	r0, r1			; r1 = the address, mailbox number cleared
	ldi:32	#STAGING, r2
	cmp	r2, r1			; one of our four and nowhere else
	bne	dione_resume

	ldi:8	#0x48, r13
	lduh	@(r13, r10), r1		; the doorbell is 4 bytes
	cmp	#WLENGTH, r1
	bne	dione_resume

	ldi:8	#0x40, r13
	ldub	@(r13, r10), r1		; and, last, our sub-opcode
	ldi:8	#SUBOP, r13
	cmp	r13, r1
	bne	dione_resume

	/* Ours to look at.  Run C on our own stack: the loader's has no
	 * documented headroom.  r10/r11 are needed again for the resume
	 * decision, so they are saved around the call. */
	ldi:32	#RES_STACK_TOP, r1
	mov	r15, r2
	mov	r1, r15
	st	r2, @-r15		; the loader's sp
	st	r10, @-r15		; the request descriptor
	st	r11, @-r15		; the setup packet
	st	rp, @-r15

	mov	r0, r4			; the frame: the mailbox asked for
	ldi:32	#frq1_dispatch, r12
	call	@r12

	ld	@r15+, rp
	ld	@r15+, r11
	ld	@r15+, r10
	ld	@r15+, r2
	mov	r2, r15			; back on the loader's stack

	cmp	#0, r4			; 0 = answered, 1 = not our frame
	bne	dione_resume
	ldi:32	#COMPLETE, r12
	jmp	@r12

dione_resume:
	ldi:8	#0x40, r13
	ldub	@(r13, r10), r0		; sub-opcode 1 is read-modify-write
	cmp	#1, r0
	beq	dione_resume_mask
	ldi:32	#RESUME_PLAIN, r12
	jmp	@r12

dione_resume_mask:
	ldi:32	#RESUME_MASK, r12
	jmp	@r12

dione_complete:
	ldi:32	#COMPLETE, r12
	jmp	@r12
