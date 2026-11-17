/* regs_selene.h -- the selene generation (MB8AC031 / MB86M0x: GV-MVP/FZ2,
 * PIX-DT295): the TS input and capture blocks.
 *
 * These read as the unmapped pattern on dione, which captures through
 * another band (regs_dione.h).  Registers both generations share stay in
 * mb86m01.h.
 */
#ifndef CFW_REGS_SELENE_H
#define CFW_REGS_SELENE_H

/* --- TS input: the two blocks in front of the capture ring --------------
 * What each register MEANS is not known, and this header does not pretend
 * otherwise -- the names are the addresses.  What is known is an order and
 * a set of values that bring the block up so the ring fills (PIX-DT295,
 * measured; not confirmed on an FZ2).  That is enough to bring the block up
 * and no more, which is why tsif_selene.c is a table of writes rather than a
 * driver with opinions.
 *
 * The one part that is NOT common is the pin multiplexing either side of
 * this (0x00F1011C, 0x00F10134): the loader has already set those for the
 * board, and the two products differ there.  tsif_selene.c leaves them alone.
 */
#define TSB0_000        0x01900000u
#define TSB0_004        0x01900004u
#define TSB0_404        0x01900404u     /* half-word */
#define TSB1_000        0x01901000u     /* board-dependent: the row's ts_in0 */
#define TSB1_004        0x01901004u     /* board-dependent: the row's ts_in1 */
#define TSB1_040        0x01901040u

/* --- the unit gate, before starting the ARMs ----------------------------
 * Not confirmed on hardware on selene: the half-word is opened to 0xFFFF,
 * bit2 of every unit's +0x404 is set, and the half-word is written again.
 * The same register opens the ARM bands on dione (measured, armcore.h).
 * arm_selene.c uses it for the ARM bands only.
 */
#define SL_UNIT_GATE        0x00f10118u /* half-word                      */
#define SL_UNIT_GATE_ALL    0xffffu
#define SL_UNIT_404_ON      0x0004u     /* unit + 0x404, bit2             */

/* --- TS capture: the ring the stream lands in --------------------------
 * The producer is hardware, not code: a capture block writes fixed-size
 * records into SDRAM and publishes its write pointer in a register.  The
 * consumer moves the read pointer.
 *
 *   0x01902020   ring base, OR'd with 0x80000000 to get the CPU view
 *   0x01902024   length
 *   0x01902028   write pointer, always < length
 *   0x0190202C   read pointer, written back when a burst is done
 *   used = (wp - rp) mod length
 *
 * Each record is 192 bytes with a 188-byte TS packet in front.  The reader
 * waits for more than 0x2FB bytes, then looks for 0x47 in three records 192
 * apart, stepping the start over 0..0xBF.  Sending n packets from offset k
 * needs (n-1)*192 + 188 + k bytes.  The record shape was seen in the ring on
 * a PIX-DT295 (measured, ring_selene.c).
 *
 * What fills these registers is tsif_selene.c (base, length and the two
 * pointers); the mode word at 0x01902000 is not written there, and what it
 * means is not known.  ring_selene.c reads what it is given.
 */
#define TSCAP_MODE      0x01902000u     /* mode word -- meaning not known  */
#define TSCAP_BASE      0x01902020u     /* ring base, PHYSICAL             */
#define TSCAP_LENGTH    0x01902024u     /* ring length in bytes            */
#define TSCAP_WRITE     0x01902028u     /* producer's pointer (hardware)   */
#define TSCAP_READ      0x0190202cu     /* consumer's pointer (ours)       */
#define TSCAP_R010      0x01902010u     /* armed with the ring base; the
                                         * teardown polls it back to 0     */
#define TSCAP_SPIN      0x20000u        /* bound on the teardown's poll    */
#define TSCAP_CPU_VIEW  0x80000000u     /* base | this = the CPU address   */

/* The length register keeps only multiples of 64 (measured, GV-MVP/FZ2):
 * a length that is not a multiple of the 192-byte record reads back
 * rounded down, which is why ring_selene.c reads the length back from the
 * register rather than trusting a constant, and re-finds the record
 * boundary after a wrap.
 */
#define TS_RECORD       192u            /* one record in the ring          */
#define TS_PACKET       188u            /* the TS packet at its front      */
#define TS_SYNC         0x47u           /* the TS sync byte                */
#define TS_SYNC_PROBES  3u              /* records a candidate must match  */
/* The search reads offset k of TS_SYNC_PROBES records in a row, k up to
 * TS_RECORD - 1, so it needs that many whole records in the ring.  The ring
 * must hold one record more than that, or a full ring could never be
 * searched (used stays below length). */
#define TS_SYNC_MIN     (TS_SYNC_PROBES * TS_RECORD)
#define TS_LENGTH_MIN   (TS_SYNC_MIN + TS_RECORD)

#endif
