/* frq1.h -- the FRQ1 command frame.
 *
 * OURS.  The layout, the numbering and the error space are our design,
 * specified in ../../docs/frq1.md.
 *
 * 64 bytes, big-endian throughout: 24 bytes of header, 40 of data.
 *
 *     +0x00  'FRQ1'      magic                       untouched on the way back
 *     +0x04  sequence                                untouched
 *     +0x08  command                                 untouched -- the echo
 *     +0x0c  status      in: 0, out: +1 or negative  THE ONLY FIELD WE WRITE
 *     +0x10  arg0
 *     +0x14  arg1
 *     +0x18  data, 40 bytes
 *
 * THE MAGIC IS SHARED, ON PURPOSE.  Another firmware of ours answers
 * 'FRQ1' at the same staging address on the same sub-opcode, with its own
 * four commands.  Both are firmware we wrote, so both are 'FRQ1'; WHICH one
 * is settled by the command number, and the two spaces are disjoint for
 * good: 0x01..0x0F is the other one's and is never allocated here, so a
 * frame that reaches the wrong firmware is refused instead of running a
 * write or a call.  A host tells the two apart by the echo: the other one
 * writes its status over the command word, we do not.
 */
#ifndef FRQ1_H
#define FRQ1_H

/* The staging area is plain memory the host reads back, so a frame is
 * touched through a volatile pointer and one byte at a time -- nothing here
 * may assume alignment. */
typedef volatile unsigned char *frame_t;

#define FR_MAGIC        0x00    /* 'FRQ1'                      (untouched) */
#define FR_SEQUENCE     0x04    /* sequence                    (untouched) */
#define FR_COMMAND      0x08    /* command                     (untouched) */
#define FR_STATUS       0x0c    /* out: ST_OK or a negative FW_* code      */
#define FR_ARG0         0x10
#define FR_ARG1         0x14
#define FR_DATA         0x18    /* 40 bytes                               */
#define FR_BYTES        0x40

#define FRQ1_MAGIC      0x46525131u
#define FRQ1_VERSION    1u      /* reported by IDENTIFY                   */

/* The host sends ST_PENDING and we always overwrite it, so "the status is
 * no longer 0" is what makes a frame count as answered.  Failures are the
 * module's own FW_* codes (fw.h) -- one space, no translation. */
#define ST_PENDING      0
#define ST_OK           1

/* Commands.  Grouped by role; see ../../docs/frq1.md for the bands.
 *
 *   0x00        not a command, so a zeroed frame cannot be one
 *   0x01..0x0F  RESERVED (above).  Do not allocate here.
 */
#define CMD_IDENTIFY            0x10u   /* who answered, and what it can do */
#define CMD_START               0x11u
#define CMD_STOP                0x12u
#define CMD_SUSPEND             0x13u
#define CMD_HEALTH              0x14u   /* phase, flags, first failure per part */

/* The front end.  There is no tune command: the host tunes by I2CXFER
 * and reads the lock here (docs/frq1.md). */
#define CMD_STATUS              0x20u
#define CMD_CNR                 0x21u

#define CMD_STREAM              0x30u   /* the stream                      */

/* Ours for bring-up work (main/probe.c): read and write the chip, arm
 * the capture ring, ask the I2C bus who is there.  Nothing on the host side but our own tools
 * sends these. */
#define CMD_PEEK                0x40u
#define CMD_POKE                0x41u
/* 0x42 / 0x43 are taken (a unit bring-up tried on the GV-MVP/FZ2); this
 * image does not answer them. */
#define CMD_RING                0x44u
#define CMD_I2CSCAN             0x45u
#define CMD_I2CXFER             0x46u   /* dione: one I2C transfer (fe_dione.c) */
#define CMD_EPDMA               0x47u   /* arm 0x00FD85xx and start an endpoint */

/* The card interface (main/card.c, docs/card.md).  The host keeps the time
 * and the T=1 protocol; these move the card's lines and bytes. */
#define CMD_CARD_STATUS         0x50u
#define CMD_CARD_ON             0x51u   /* steps; FW_EAGAIN until ATR      */
#define CMD_CARD_OFF            0x52u
#define CMD_CARD_PARAM          0x53u
#define CMD_CARD_PUT            0x54u
#define CMD_CARD_SEND           0x55u
#define CMD_CARD_RECV           0x56u   /* FW_EAGAIN until a whole block   */
#define CMD_CARD_GET            0x57u

/* The two ARM cores (main/arm.c, docs/arm.md).  The host loads our ARM image
 * a few words at a time and reads the core's state and our mailbox. */
#define CMD_ARM_STATUS          0x60u   /* reads only                      */
#define CMD_ARM_HOLD            0x61u   /* wake the band, hold the core    */
#define CMD_ARM_LOAD            0x62u   /* write words at an ARM address   */
#define CMD_ARM_RUN             0x63u   /* start the image in ITCM         */
#define CMD_ARM_CALL            0x64u   /* queue a request (armipc.h)      */
#define CMD_ARM_REAP            0x65u   /* take a reply, FW_EAGAIN if none */
#define CMD_ARM_CANCEL          0x66u   /* stop a task, around the rings   */
#define CMD_ARM_PUT             0x67u   /* bytes into the buffer's TX half */
#define CMD_ARM_GET             0x68u   /* bytes from the buffer's RX half */

/* frame access (frq1.c).  There is no public "fail the frame" call: a
 * module reports by returning an FW_* code and the router writes it. */
unsigned frq1_be32(frame_t f, int off);
void frq1_put32(frame_t f, int off, unsigned v);

/* command handlers.  Each returns nothing and leaves the frame answered;
 * the router has already written ST_OK, so a handler only overrides. */
int dev_identify(frame_t f);

/* the entry point entry.S calls */
int frq1_dispatch(frame_t f);

#endif
