/* armcore.h -- the two ARM cores: where they are, and our mailbox.
 *
 * Included by BOTH sides: the main FR (src/main/arm.c, built with fr30-elf)
 * and our ARM image (src/armcore/, built with arm-none-eabi).  So: plain
 * #defines, no types, nothing that depends on the compiler.
 *
 * THE TWO VIEWS.  The main FR sees each core through a window (ARM-E at
 * 0x00E00000, ARM-C at 0x00C00000).  The core itself sees its code at 0
 * (ITCM) and its data RAM at 0x00100000 (DTCM).  The window's 0x80000 band
 * is that DTCM.  Both views hold on a GV-M2TV: our image ran from ITCM 0
 * with its mailbox in DTCM, seen through the window (measured; docs/arm.md
 * has what is unconfirmed).
 *
 *     ARM side             main side (ARM-E / ARM-C)
 *     0x00000000 ITCM      0x00E00000 / 0x00C00000     code, 64 KiB
 *     0x00100000 DTCM      0x00E80000 / 0x00C80000     data and stack, 64 KiB
 *
 * Every value here is big-endian, like the rest of the chip: the ARM image
 * is built with -mbig-endian, so a word means the same thing on both sides.
 */
#ifndef CFW_ARMCORE_H
#define CFW_ARMCORE_H

/* --- which core ------------------------------------------------------- */
#define ARM_CORE_E          0u      /* window 0x00E00000                   */
#define ARM_CORE_C          1u      /* window 0x00C00000                   */
#define ARM_CORES           2u

/* --- the main FR's view ----------------------------------------------- */
#define ARM_E_WIN           0x00e00000u
#define ARM_C_WIN           0x00c00000u

/* Offsets from the window.  The control band is 16-bit registers.  On a
 * GV-M2TV (dione) driven from the host's 0xBC window (measured):
 *   - with 0x00F10118 = 0xFFFF, CTL_EN reads 0x0001 and CTL_RST takes a
 *     write: 0xFFFF reads back as 0x0FFF
 *   - CTL_EN bit0 STOPS the core: with it set the mailbox's loop counter
 *     freezes, with it clear the core runs.  CTL_RST = 0x0FFF alone does
 *     not stop a running core
 *   - stopped with CTL_RST = 0x0FFF, then CTL_RST = 0x0104 and CTL_EN = 0:
 *     the core starts from ITCM 0 (our image ran; docs/arm.md)
 *   - ITCM takes word stores from the main FR but always reads back 0:
 *     it is write-only from this side
 *   - DTCM (64 KiB) and 0x40000..0x57FFF (96 KiB) take writes and read back;
 *     DTCM also reads while the core runs with CTL_RST = 0x0104 (it read 0
 *     with CTL_RST = 0) */
#define ARM_ITCM_OFF        0x00000u
#define ARM_RAM_OFF         0x40000u    /* 0x40000..0x57FFF, plain RAM     */
#define ARM_CTL58_OFF       0x58000u    /* control word both sides touch   */
#define ARM_CTL_EN_OFF      0x70000u
#define ARM_CTL_RST_OFF     0x70004u
#define ARM_CTL_CMD_OFF     0x70050u    /* not confirmed; not used here    */
#define ARM_CTL_404_OFF     0x70404u
#define ARM_DTCM_OFF        0x80000u

#define ARM_RST_HOLD        0x0fffu     /* what 0xFFFF reads back as       */
#define ARM_RST_RUN         0x0104u     /* runs the loaded image (measured) */
#define ARM_EN_STOP         0x0001u     /* CTL_EN bit0: the core is stopped */

/* --- the core's own view ---------------------------------------------- */
#define ARM_ITCM            0x00000000u
#define ARM_ITCM_SIZE       0x00010000u
#define ARM_DTCM            0x00100000u
#define ARM_DTCM_SIZE       0x00010000u

/* --- our mailbox --------------------------------------------------------
 * The last 256 bytes of DTCM.  Our own layout.  The ARM image's stack
 * starts right below it (src/armcore/link.ld).
 *
 * The ARM side writes ALIVE, LOOPS and FAULT; the main side writes REQ_*
 * and reads the rest.  One word per field: stores narrower than a word
 * into RAM shared with another bus master are not trusted here (on dione
 * the main FR's own 0x000E band already showed that, docs/image.md). */
#define ARM_MBOX            (ARM_DTCM + ARM_DTCM_SIZE - 0x100u)  /* ARM side */
#define ARM_MBOX_OFF        (ARM_DTCM_OFF + ARM_DTCM_SIZE - 0x100u) /* main  */

#define ARM_MB_ALIVE        0x00u   /* ARM_ALIVE_MAGIC once arm_main runs  */
#define ARM_MB_LOOPS        0x04u   /* counts up while the ARM is idle     */
#define ARM_MB_FAULT        0x08u   /* 0, or ARM_FAULT_BASE + vector index */
#define ARM_MB_FAULT_LR     0x0cu   /* lr at the fault                     */
#define ARM_MB_REQ_SEQ      0x10u   /* main side: new value = new request  */
#define ARM_MB_REQ_ARG      0x14u
#define ARM_MB_ACK_SEQ      0x18u   /* ARM side: REQ_SEQ once answered     */
#define ARM_MB_ACK_VAL      0x1cu
#define ARM_MB_REQ_OP       0x20u   /* main side: ARM_OP_*                 */
#define ARM_MB_REQ_VAL      0x24u   /* main side: the value to write       */
#define ARM_MB_ACK_ST       0x28u   /* ARM side: ARM_ST_*                  */
#define ARM_MB_PROBING      0x2cu   /* ARM side: 1 while a probe access    */
#define ARM_MB_SCRATCH      0x30u   /* ARM side: the abort handler's r0    */
#define ARM_MB_ABORT_FAR    0x34u   /* ARM side: CP15 FAR of the last abort */
#define ARM_MB_ABORT_FSR    0x38u   /* ARM side: CP15 FSR of the last abort */
#define ARM_MB_BYTES        0x40u

/* Requests (REQ_OP).  The main side writes REQ_OP, REQ_ARG, REQ_VAL, then a
 * new REQ_SEQ; the ARM side answers ACK_VAL and ACK_ST, then ACK_SEQ.  A
 * data abort during a probe access is caught: ACK_ST = ARM_ST_ABORT and
 * ABORT_FAR/FSR say why; the image keeps running. */
#define ARM_OP_NOT          0u      /* ACK_VAL = ~REQ_ARG                  */
#define ARM_OP_RD32         1u      /* ACK_VAL = [REQ_ARG]                 */
#define ARM_OP_WR32         2u      /* [REQ_ARG] = REQ_VAL, ACK_VAL = read */
#define ARM_OP_RD16         3u
#define ARM_OP_WR16         4u
#define ARM_OP_RD8          5u
#define ARM_OP_WR8          6u
#define ARM_OP_CP15         7u      /* ACK_VAL = a CP15 register, ARM_CP_* */

#define ARM_CP_ID           0u      /* c0, c0, 0  main ID                  */
#define ARM_CP_CACHE        1u      /* c0, c0, 1  cache type               */
#define ARM_CP_TCM          2u      /* c0, c0, 2  TCM status               */
#define ARM_CP_CTRL         3u      /* c1, c0, 0  control                  */
#define ARM_CP_DTCM         4u      /* c9, c1, 0  DTCM region              */
#define ARM_CP_ITCM         5u      /* c9, c1, 1  ITCM region              */

#define ARM_ST_OK           0u
#define ARM_ST_ABORT        1u      /* the access took a data abort        */
#define ARM_ST_BADOP        2u

#define ARM_ALIVE_MAGIC     0x41524d31u     /* 'ARM1' */
#define ARM_FAULT_BASE      0x46000000u     /* 'F' in the top byte         */

#endif
