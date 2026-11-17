/* regs_dione.h -- the dione generation (MB8AC018 / MB86H5x, the GV-M2TV):
 * the TS input and capture.  selene's are in regs_selene.h; the two
 * generations share none of them.
 *
 * Everything here was driven from the host's 0xBC window on a live GV-M2TV
 * and produced a transport stream (PAT decoded).  What each register MEANS is mostly not known;
 * the names say what the writes were seen to do, the values are the ones
 * that worked.  docs/ts.md has the order and the reasons.
 *
 * selene's TS blocks (0x0190xxxx) read as the unmapped pattern on this
 * generation, so none of tsif_selene.c / ring_selene.c applies here.
 */
#ifndef CFW_DIONE_H
#define CFW_DIONE_H

/* --- pins -------------------------------------------------------------
 * Both are the HIGH half-word of their word.  The window wrote them as a
 * whole word, keeping the low half; ts_dione.c does the same.
 */
#define DN_PIN_GATE     0x00f10118u     /* high half: gates; bit2 = DRAM side */
#define DN_PIN_ROUTE    0x00f10128u     /* high half: TS input routing        */
#define DN_PIN_GATE_ALL 0xffffu
#define DN_PIN_GATE_MEM 0x0004u         /* without it 0x00D0xxxx takes no write */
#define DN_PIN_ROUTE_TS 0x6100u         /* bit15 set stops the stream          */

/* --- the 0x00D0 band: memory controller and capture -------------------
 * The loader state leaves it switched off (0x00D00004 reads cleared).
 * Two memory-controller writes bring it up (ts_dione.c, measured); with
 * 0x00001100 in 0x00D00004 instead of DN_MEMC_CFG_ON the sub-blocks at
 * 0x00D01xxx / 0x00D02xxx do not decode (measured).  They run before the
 * TS setup.
 */
#define DN_MEMC_CTL     0x00d00000u
#define DN_MEMC_CTL_RUN 0x00020000u     /* without it no TS (measured)        */
#define DN_MEMC_CFG     0x00d00004u
#define DN_MEMC_CFG_ON  0x00043305u     /* what 0x00D00004 ends at when awake */
#define DN_MEMC_CFG_MASK 0x00ffffffu    /* it reads back as 0x05043305: the top
                                         * byte is not ours                   */

/* TS input stage */
/* 0x00D01000, measured on a GV-M2TV by changing one bit mid-capture
 * (docs/ts.md): bit0 reverses the bit order in each byte, bit17 set gives
 * 188-byte records and clear gives 204, bits 25:24 = 10 runs the input (11
 * and 00 stop it).  The other bits either do not stick or change nothing.
 * 0x00D01044..0x00D0104C changed nothing in any form tried. */
#define DN_TSIN_CTL     0x00d01000u
#define DN_TSIN_004     0x00d01004u
#define DN_TSIN_020     0x00d01020u
#define DN_TSIN_RUN     0x00d01040u     /* 0x01000000 runs, 0 stops           */
#define DN_TS_GATE      0x00f12788u     /* high half; 0 before the ring       */

/* The capture ring.  The pointers are byte offsets from the base. */
#define DN_RING_MODE    0x00d02000u
#define DN_RING_KICK    0x00d02010u     /* written with the base to start     */
#define DN_RING_BASE    0x00d02020u
#define DN_RING_LIMIT   0x00d02024u     /* length - 0x20; read back in 64 B   */
#define DN_RING_WRITE   0x00d02028u     /* producer offset (hardware)         */
#define DN_RING_READ    0x00d0202cu     /* consumer offset (ours)             */
#define DN_RING_038     0x00d02038u
#define DN_RING_MODE_RUN 0x03020300u

/* Where the ring goes: DRAM that nothing else here uses.  The CPU reads
 * it at this address directly (measured). */
#define DN_RING_AT      0x02000000u
#define DN_RING_MAX     0x01000000u     /* 16 MiB; larger stops at 16 MiB    */
#define DN_RING_MIN     0x00100000u

/* The ring is a circle, as on selene (measured on a GV-M2TV, the CPU
 * reading the DRAM):
 *
 *   - the DMA goes back to the base at the offset DN_RING_LIMIT reads back
 *     (length - 0x40), and the stream continues there byte for byte
 *   - it stops in front of DN_RING_READ and waits; it does not write over
 *     what the reader has not taken
 *
 * So the reader writes DN_RING_READ as it consumes.  If it never does, the
 * read offset stays 0 and the DMA stops one lap in, just short of the end.
 *
 * Where it stops: DN_RING_FULL_GAP bytes short of the read offset (a 1 MiB
 * ring left alone: write 0x2FF40 against read 0x30000, and it stayed there).
 * What came in meanwhile is lost at the input; once the reader moves on,
 * the stream goes on with one gap (one resync on the host).
 */
#define DN_RING_FULL_GAP 0xc0u

/* --- the front end and its I2C ----------------------------------------
 * The I2C controller is the 0x00FD8FE0 instance (I2C_FD8F_BASE).  Driven
 * from inside the chip on a live GV-M2TV, it took writes and the
 * demodulator at 7-bit 0x18 ACKed once the following were set
 * (docs/i2c.md).
 *
 * Front-end power: two pin words, high half.  With both at 0 nothing on
 * the bus answers.  With these values the demodulator ACKed (measured).
 */
#define DN_FE_SEL       0x00f10014u     /* high half: direction / enable      */
#define DN_FE_DAT       0x00f10018u     /* high half: levels                  */
#define DN_FE_SEL_ON    0x02f4u
#define DN_FE_DAT_ON    0x03ffu

/* I2C pins: the high half-word of 0x00F10114, bit3:2 = 0b11, which is
 * word bit19:18.  selene boards use bit5:4 of the same half-word
 * (RF_ENABLE_114 in mb86m01.h); the two are not interchangeable. */
#define DN_I2C_PIN      0x00f10114u
#define DN_I2C_PIN_MASK 0x000c0000u
#define DN_I2C_PIN_ON   0x000c0000u

/* 0x00FD80C0 (SYS_REG_0C0) comes up as 3, and both bits hold the I2C block
 * back (I2C_HOLD_BITS in mb86m01.h; measured on a GV-M2TV and a PIX-DT295).
 * With bit0 at 1 the block ignores writes to control and divider, and
 * +0x14 / +0x18 read their reset values (0 and 1).  With bit0 cleared but
 * bit1 at 1 every register of the block reads 0x80 and the bus stays busy.
 * Clearing both lets the setup take. */

#endif
