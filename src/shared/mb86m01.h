/* mb86m01.h -- chip registers, for the feature modules to grow into.
 *
 * Each group says whether it was measured on hardware and on which device.
 * Anything that is not confirmed on hardware says so.
 */
#ifndef MB86M01_H
#define MB86M01_H

#define MMIO(a)         (*(volatile unsigned *)(a))
#define MMIO8(a)        (*(volatile unsigned char *)(a))
#define MMIO16(a)       (*(volatile unsigned short *)(a))

/* --- HIF: the window the host reads -------------------------------------
 * Measured on a live FZ2 while the loader was answering.
 */
#define HIF_SIGNATURE0  0x00f00000u     /* reads 'MB8A'                    */
#define HIF_SIGNATURE1  0x00f00004u     /* reads 'C031' under the loader;
                                           the host's command word once our
                                           firmware runs                   */
#define HIF_REG_014     0x00f00014u     /* meaning not known               */
#define HIF_START_ADDR  0x00f00020u     /* holds 0x82000000 at power-on    */
#define HIF_VERSION_STR 0x00f00094u     /* 9 ASCII bytes, then 0x1a        */
#define HIF_MAILBOX     0x00f00220u

/* --- GPIO / straps ------------------------------------------------------
 * **A HALF-WORD.** Read it as one: on a live FZ2 the half-word is 0x8A01,
 * the word is 0x8A010000, and every field below lands 16 bits out if you
 * take the word.  gpio_straps() is the accessor.  The field meanings below
 * are not confirmed on hardware beyond the FZ2 value.
 */
#define GP_STRAPS       0x00f10004u     /* half-word                       */
#define STRAP_FZ2_SEEN  0x8a01u         /* measured, a live GV-MVP/FZ2     */
#define STRAP_SIGSEL_MASK 0x3000u       /* bits 13..12: pick the signature
                                           in HIF_SIGNATURE0/1 (assumed)   */
#define STRAP_ID_MASK   0x0c00u         /* bits 11..10: tells boards apart
                                           (assumed)                       */

/* --- I2C: one controller design, two instances in the chip --------------
 * Fujitsu I2C, read and written 32 bits at a time: status, control, data.
 * Measured from the host, one register access at a
 * time on a PIX-DT295.
 *
 * THERE ARE TWO OF THESE, AND WHICH ONE A BOARD USES IS THE DIFFERENCE
 * BETWEEN THE TWO TRANSPORTS IN config/drivers.json.
 *
 *   0x00FD8FE0   inside the USB core block.  The main FR drives this one
 *                on the GV-M2TV (MB8AC018) and on the PIX-DT295 (MB8AC031),
 *                both measured; the DT295 also needs the front-end reset
 *                released first (FE_RESET_* below).
 *   0x01684FE0   inside a second block of the same type, reached from the
 *                auxiliary unit 0x017.  The DT295's demodulators answer on
 *                this one too, from our auxiliary-unit program (measured;
 *                i2c/auxslot.c, which no device row selects now).  Driving
 *                0x00FD8FE0 from the host on an FZ2 gets through setup but
 *                never completes a transfer (measured).
 *
 * The two blocks are treated as the same design: the I2C group at
 * +0xFE0..+0xFF8 takes the same setup and the same byte handshake at both
 * bases (measured on a PIX-DT295).  So one driver core takes the base
 * address as an argument (src/shared/fjcore.c) and each transport is a thin
 * wrapper over it.
 */
#define I2C_FD8F_BASE   0x00fd8fe0u     /* the USB core block's instance   */
#define I2C_1684_BASE   0x01684fe0u     /* the auxiliary unit's instance   */

#define I2C_O_STATUS    0x00u           /* I2C_ST_* below                   */
#define I2C_O_CONTROL   0x04u           /* I2C_CT_* below                   */
#define I2C_O_DIVIDER   0x08u           /* bus clock; I2C_DIV_SETUP         */
#define I2C_O_DATA      0x10u           /* the byte going out or come in    */
/* +0x14 / +0x18 are not written: on both instances the bus worked with
 * them left as they come up (measured on the PIX-DT295 and GV-M2TV for
 * 0x00FD8FE0, on the PIX-DT295 for 0x01684FE0). */

/* The names the 0x00FD8FE0 instance was written against, kept so the
 * measured sequence still reads literally.
 */
#define I2C_STATUS      (I2C_FD8F_BASE + I2C_O_STATUS)
#define I2C_CONTROL     (I2C_FD8F_BASE + I2C_O_CONTROL)
#define I2C_DIVIDER     (I2C_FD8F_BASE + I2C_O_DIVIDER)
#define I2C_DATA        (I2C_FD8F_BASE + I2C_O_DATA)

/* The 0x00FD8FE0 instance takes no write until its block is clocked and
 * out of reset: a divider written before that reads back 0 (measured).
 *
 * The 0x01684FE0 instance has its own gate (src/auxcore/auxmain.c), not
 * these.  The gates are therefore in i2c/fujitsu.c, not in the shared core.
 *
 * These words hold other blocks' clocks and resets too, the USB core's
 * among them.  Only bit4 is changed, read-modify-write; nothing else in
 * them is written.
 */
#define SYS_REG_010     0x00fd8010u     /* |= 0x10 : clock the I2C block   */
#define SYS_REG_014     0x00fd8014u     /* &= ~0x10 : release its reset    */
#define SYS_REG_0C0     0x00fd80c0u     /* bit1:0 hold the I2C block back  */
#define I2C_HOLD_BITS   0x3u
#define I2C_CLK_BIT     0x10u
#define I2C_RESET_BIT   0x10u
#define I2C_CT_INITIAL  0x42u           /* the control register's setup value */
#define I2C_CT_PRESET   0x40u           /* written first on the FZ2 path
                                         * (i2c/aux684.c); not confirmed  */
#define I2C_DIV_SETUP   0x3cu
#define I2C_DIV_MASK    0x7fu           /* bit7 is status, not what we wrote */

/* Bit5 of the half-word here set, bit4 cleared, before touching I2C.  It
 * is the pin setting for the I2C lines (assumed) and was measured safe on a
 * DT295, so it goes in the sequence that drives this controller.
 *
 * TREAT IT AS UNPROVEN ON AN FZ2.  This register is pin assignment and the
 * two products' values are nothing alike (FZ2 0xFA40, DT295 0x8000, both
 * read on hardware).  Nobody has measured the write on an FZ2, so the two
 * FZ2 transports do not write it: a transport that cannot carry a transfer
 * anyway has no reason to touch the chip on the way to saying so.
 */
#define RF_ENABLE_114   0x00f10114u
#define RF_EN_BIT       0x00200000u     /* word bit21 = half-word bit5     */
#define RF_EN_CLEAR     0x00300000u     /* clear bit5:4 before setting bit5 */

/* The front-end reset on selene: bit1 of two half-words, set in this order
 * (0x00F10018 first, then 0x00F10014), read-modify-write.  Measured on a
 * PIX-DT295 from cold: without them the I2C controller sets up but no
 * device on the bus ACKs, on either instance; clearing 0x00F10014 bit1 again
 * afterwards stops every ACK.  i2c/fujitsu.c (0x00FD8FE0) and i2c/auxslot.c
 * (0x01684FE0) both do this first.
 *
 * dione has its own front-end power at the same two addresses with other
 * values (DN_FE_SEL / DN_FE_DAT in regs_dione.h); the two do not mix.
 */
#define FE_RESET_DIR    0x00f10014u     /* half-word                       */
#define FE_RESET_DAT    0x00f10018u     /* half-word                       */
#define FE_RESET_BIT    0x0002u

/* Status: read only. */
#define I2C_ST_BUSY     0x80u   /* the bus is held; wait before a START    */
#define I2C_ST_NACK     0x10u   /* the address we sent was not answered    */
/* Control: the bits fjcore.c sets and clears in its shadow copy. */
#define I2C_CT_ERROR    0x80u   /* reads 1 after a bus error               */
#define I2C_CT_RESTART  0x20u   /* send a repeated START next              */
#define I2C_CT_START    0x10u   /* 1 = START and hold the bus, 0 = STOP    */
#define I2C_CT_ACK      0x08u   /* answer the byte being read with ACK     */
#define I2C_CT_DONE     0x01u   /* reads 1 when a byte is through; writing
                                 * 0 lets the next one go                 */

/* NOT USED, ON PURPOSE.  bit0 of the half-word at 0x00F10090 may be a
 * semaphore over the I2C block between two CPUs (not confirmed on
 * hardware).  This firmware does not touch it, because:
 *
 *   - reading 0x00F10080-0x00F100FF from this side is the one access that is
 *     measured to break things: it makes the resident loader time out, and
 *   - only one party drives the bus at a time in this design: our own
 *     auxiliary-unit program serves one request at a time, and nothing else
 *     is taking the bus while it has it.
 *
 * If a board ever turns out to need two parties on the bus at once, this is
 * the first thing to look at.
 */
#define RF_I2C_SEM      0x00f10090u     /* half-word, bit0 -- see above    */

/* How many reads to spin on one byte before giving up.  0x20000 is a
 * generous bound, not tuned on hardware: hitting it means the bus is stuck.
 */
#define I2C_SPIN        0x20000u

/* Reads to spend letting the control register's "byte done" bit go back down
 * after we write it, BEFORE starting to wait for it to come up again.
 *
 * THIS IS THE ONE THING THAT ONLY SHOWS UP FROM INSIDE THE CHIP.  Driven
 * from the host every access was separated by a ~60 us control transfer, so
 * the bit had always settled; the in-chip sequencer on a PIX-DT295 has to
 * look for bit0 to be 0 first, up to 64 reads, or it reads the PREVIOUS
 * byte's "done" and thinks this one has already finished.  If it has not
 * gone down within 64 reads we carry on anyway.
 */
#define I2C_SETTLE      64u

/* --- I2C: a mailbox on the auxiliary unit (command 9) -------------------
 * A request interface where the main CPU fills these words and wakes the
 * auxiliary unit, which runs the transfer and answers.  It only works while
 * a program on the auxiliary unit serves it, and this firmware does not
 * load one that speaks this layout: our own auxiliary-unit program uses its
 * own slot (src/auxcore/auxmain.c).  i2c/mailbox.c is written against these
 * words and stays a stub.
 *
 * NOT CONFIRMED ON HARDWARE, except where a line says measured.  The words
 * are kept so the layout is in one place.
 *
 * Address views: the auxiliary unit is FR.  Its code is seen at local
 * 0x000E0000 and its RAM at local 0x000Axxxx, which the main CPU sees at
 * 0x017E0000 and 0x017Axxxx; peripherals have the same address from both.
 *
 * AUX_MBOX: the high bits name the request, the low bits select which
 * blocks to bring up (assumed).
 */
#define AUX_ALIVE       0x017a3900u     /* 'DTI2' when the aux mailbox is up */
#define AUX_ALIVE_MAGIC 0x44544932u
#define AUX_MBOX        0x017a3904u     /* request bit (high) + block bits   */
#define AUX_FIELD_391C  0x017a391cu     /* expected to be 1 (assumed)        */
#define AUX_BELL        0x01700080u     /* = 0x20000000 : wake the aux unit */
#define AUX_BELL_RING   0x20000000u
/* The data buffers.  Both are in the 0x00F0 page, which a live FZ2 answers
 * from the main FR (measured), so every part of a transfer would be
 * reachable from here.  0x017A3940 is the serving program's own scratch,
 * not part of the request.
 */
#define AUX_I2C_TX      0x00f000c0u     /* bytes to send                    */
#define AUX_I2C_RX      0x00f023c0u     /* bytes received                   */
#define AUX_I2C_RES0    0x00f023beu     /* result flag, written at the end  */
#define AUX_I2C_RES1    0x00f023bfu
#define AUX_I2C_STATE_P 0x017a3940u     /* the aux side's own scratch       */

/* The whole request, as this layout assumes it:
 *
 *   1. bytes to send -> 0x00F000C0
 *   2. [0x017A3908] = 9 ; [0x017A390C] = kind ; [0x017A3910] = addr<<8 | len
 *   3. [0x01700080] = 0x20000000                     ring
 *   4. wait for [0x00D00080] == 0x0100               half-word
 *   5. read 0x00F023C0, and the result flags at 0x00F023BE/BF
 *
 * What IS measured: the aux SRAM is live and writable from the main FR, and
 * the controller at 0x01684FE0 answers only the auxiliary unit's bus.  So
 * the way in is our own aux-side program (src/auxcore/).
 */

#define AUX_CMD         0x017a3908u     /* 9 = I2C, 0xE = another path      */
#define AUX_CMD_I2C     9u
#define AUX_KIND        0x017a390cu     /* low byte: kind 0..4              */
#define AUX_ADDR_LEN    0x017a3910u     /* high byte addr<<8 | length (<=0x40) */
#define AUX_RETURN      0x017a3914u     /* 0x80 for kind 3, else 0          */
/* The done flag.  0x0100 is the I2C request's; 8 belongs to a different
 * command (assumed).  A multi-byte transfer's data lives in the 0x00F0
 * page (AUX_I2C_TX / AUX_I2C_RX above), not here. */
#define AUX_DONE        0x00d00080u     /* aux writes a half-word when done */
#define AUX_DONE_FLAG   0x0100u         /* the I2C request's                */
#define AUX_DONE_FLAG8  0x0008u         /* another command's                */
#define AUX_MAX_LEN     0x40u

/* --- TS input and capture: per generation ------------------------------
 * selene's are in regs_selene.h, dione's in regs_dione.h.  The two
 * generations share none of them.
 */

/* --- the USB side of a bulk burst ---------------------------------------
 * One burst takes a pair -- a status register and a FIFO -- and there are
 * two of them, chosen by the caller.  The sequence is: check bit2 of the
 * status, write 128 words into the FIFO (each assembled from 4 bytes, so the
 * source may be unaligned), then 8 / commit / 0xF around the guard.  TS
 * streams this way on a live FZ2 and GV-M2TV (measured).
 *
 * 0x00FD814C looks like a four-bit byte enable for the endpoint register
 * block, with bit0 the MOST significant byte (not confirmed on hardware
 * beyond the two values below):
 *
 *     0x0F  all four bytes      0x08  lowest byte only
 *     0x07  top three bytes     0x06  the two middle bytes
 *     0x03  top two bytes       0x02  bits 23:16 only
 *     0x01  top byte only
 *
 * So 0x0F would be the resting state, not "closed".  The two values this
 * firmware uses hold either way: 8 writes only the lowest byte (the commit
 * bit), 0xF restores all four.
 */
#define USB_EP_GUARD    0x00fd814cu
#define USB_EP_OPEN     8u              /* what the burst writes before    */
#define USB_EP_CLOSED   0xfu            /* ... and after the commit        */
#define USB_EP_BURST    0x200u          /* 512 bytes = 128 words           */
#define USB_EP_READY    4u              /* bit2 of the status register     */
#define USB_EP_COMMIT   4u              /* written to the status register   */

/* The two pairs.  TS streams on pair 0, so that is this firmware's
 * default; pair 1 is here so the register map is complete.
 *
 * WHICH ENDPOINT EACH PAIR REACHES, measured on a live FZ2: the same
 * 512-byte report went out of both pairs with the pair number inside it,
 * and the host read it back:
 *
 *     pair 0 (0x00FDBD0C / 0x00FDBD84)  ->  EP 0x81
 *     pair 1 (0x00FDBD24 / 0x00FDBD90)  ->  EP 0x84
 *
 * Both delivered, repeatedly, with no endpoint configuration of our own --
 * on a path where the resident loader is gone.  So the burst alone is
 * enough, on both pairs.
 *
 * The GV-M2TV's descriptor has the same four endpoints (0x81, 0x02, 0x83
 * interrupt, 0x84; bulk ones 512 bytes), and an ARM core's burst into
 * pair 1 came out of EP 0x84 with the resident loader in place (svc_sys.c,
 * EPTEST).  Its status read 0x00111004 before the first burst and
 * 0x00511004 after, the same as pair 0 in use.
 */
#define USB_EP0_STATUS  0x00fdbd0cu
#define USB_EP0_FIFO    0x00fdbd84u
#define USB_EP1_STATUS  0x00fdbd24u
#define USB_EP1_FIFO    0x00fdbd90u

/* The block that moves bytes between memory and USB.  The USB -> memory
 * direction lands a downloaded container at 0x82000000 in 0xFFC0 chunks,
 * the address advancing by the length per chunk (assumed from the register
 * shapes; not driven by this firmware).
 *
 * WHETHER IT GOES THE OTHER WAY IS THE OPEN QUESTION.  If memory -> endpoint
 * works, it replaces pushing words through usb_ep_burst, which is what caps
 * TS at about 1.2 MB/s today.  Driving these five from the HOST does not
 * work: measured on a GV-M2TV, twelve combinations left the address at its
 * start, the cause bit clear and EP 0x81 silent.  What the host cannot do is
 * the endpoint's own start bit, because that write needs the byte enable set
 * in the same transfer -- hence probe_epdma(), which does it from in here.
 */
#define USB_DMA_CMD     0x00fd8500u     /* 0 then 2; reads back 2 only     */
#define USB_DMA_RUN     0x00fd8504u     /* write 1 to start                */
#define USB_DMA_ADDR    0x00fd8508u     /* memory address; advances by LEN  */
#define USB_DMA_LEN     0x00fd8510u     /* one chunk, e.g. 0xFFC0           */
#define USB_DMA_CTL     0x00fd8514u     /* 0x40001000 (assumed)             */
#define USB_DMA_CAUSE   0x00fd8094u     /* bit2 = one chunk finished        */
#define USB_DMA_CTL_RUN 0x40001000u
#define USB_PIPE_CTL    0x00fdd000u     /* OR 1 then 0x00010000 (assumed)   */
#define USB_PIPE_KICK   0x00fdc00cu     /* ... and then writes 0x00010000   */
#define USB_EP_CONFIG(k) (0x00fdbd08u + 8u * (k))  /* bit0 starts a transfer */
#define USB_EP_STATUS(k) (0x00fdbd0cu + 8u * (k))

/* NOT USED.  What goes in an endpoint's configuration word is not known.
 * It does not have to be: the resident loader publishes the descriptors
 * before any firmware is downloaded, and TS streams with the burst alone
 * (measured, above).  So this firmware does the burst alone.
 */
#define USB_EP0_CONFIG  0x00fdbd08u
#define USB_EP1_CONFIG  0x00fdbd20u

/* Demodulator I2C addresses, measured on a PIX-DT295.
 * They are NOT assumed to hold on other boards -- config/devices/ carries
 * each product's own, and leaves it 0 where it has not been established.
 */
#define I2C_ADDR_TERR   0x10u
#define I2C_ADDR_SAT    0x11u

/* --- the card interface (ISO 7816), docs/card.md -------------------------
 * Measured on a GV-M2TV, from the host's 0xBC window and from this firmware.
 * Every register is a 32-bit word.  CARD_RX advances on every read,
 * CARD_GO and CARD_TX act on every write, CARD_EVENT clears the bits
 * written as 1 -- never read-back-and-restore these.
 */
#define CARD_EN         0x00fea000u     /* 7 = on; see CARD_EN_ON           */
#define CARD_PINS       0x00fea004u     /* the card's lines (below)        */
#define CARD_BAUD       0x00fea008u     /* bits 15:7 = clocks per etu      */
#define CARD_GO         0x00fea00cu     /* write 1: send what is in CARD_TX */
#define CARD_CTL        0x00fea010u     /* reads 0x200 after power-up      */
#define CARD_MODE       0x00fea014u     /* read: bytes waiting << 16       */
#define CARD_IRQEN      0x00fea018u     /* 0: we poll                      */
#define CARD_EVENT      0x00fea01cu     /* status bits; write 1s to clear  */
#define CARD_CWT        0x00fea020u     /* character wait, in etu (guess)  */
#define CARD_BWT        0x00fea024u     /* block wait, in etu (guess)      */
#define CARD_TX         0x00fea028u     /* one byte per write              */
#define CARD_RX         0x00fea02cu     /* one byte per read               */
#define CARD_VCC        0x00fea034u     /* 1: the card is powered          */
#define CARD_TXLRC      0x00fea038u     /* XOR of the bytes sent           */
#define CARD_RXLRC      0x00fea03cu     /* XOR of the bytes received       */

/* CARD_EN: 7 receives and sends.  4 and 6 got no ATR, 5 got the ATR but no
 * answer to a block, and after 5 the block took no card until the device
 * was replugged (measured), so nothing but 7 is ever written. */
#define CARD_EN_ON      7u

#define CARD_PIN_CLKSEL 0x01u           /* clock select; either worked     */
#define CARD_PIN_CLK    0x02u           /* clock out                       */
#define CARD_PIN_RST    0x04u           /* RST high                        */
#define CARD_PIN_POL    0x08u           /* polarity, from the board row    */

/* CARD_EVENT bits (measured): bit10 card present, bit6 set when the answer
 * started later than CARD_BWT, bit7 when a gap between bytes was longer
 * than CARD_CWT.  Neither wait stops the receiver. */
#define CARD_EV_PRESENT 0x400u
#define CARD_EV_BWT     0x040u
#define CARD_EV_CWT     0x080u

/* CARD_BAUD: bits 15:7 are F/D, the card clocks per etu; a value off by 3%
 * still worked, a stale one (F/D 186 for an ATR at 372) gave noise.  Bits
 * 6:0 made no difference at 0..0x7F (measured), so they are written 0. */
#define CARD_BAUD_SHIFT 7u
#define CARD_FD_ATR     372u            /* the ATR is always at F/D = 372  */

/* --- what the loader publishes before the firmware runs ----------------
 * Measured on a live device.  The descriptor word here carries the board
 * identity.  This firmware does not need it: each image is built for one
 * product.  On the FRC1 path the word holds
 * noise anyway (measured on a GV-MVP/FZ2).
 */
#define SHM_DESCRIPTORS 0x017a3d00u     /* the USB descriptor set          */
#define SHM_IDVENDOR    0x017a3d08u     /* idVendor:idProduct, one word    */
#define SHM_CFG_PARAMS  0x00f4b400u     /* 20 bytes from the board         */

#endif
