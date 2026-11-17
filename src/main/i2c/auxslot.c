/* i2c/auxslot.c -- the 0x01684FE0 controller, driven by our own program on
 * the auxiliary core.
 *
 * NO DEVICE ROW SELECTS THIS TODAY.  The PIX-DT295 reaches its
 * demodulators from the main FR over 0x00FD8FE0 (i2c/fujitsu.c, measured),
 * which needs no program on the auxiliary core.  This transport stays as
 * the description of the other route, which was also measured on that
 * device; like every driver it is compiled into every image.
 *
 * WHY THIS ROUTE.  On a cold PIX-DT295 (measured) the band at
 * 0x01684000 answers only to the auxiliary core, and only while that core
 * holds the control-plane half-word 0x00F1010C at 0x000B.  The main FR
 * cannot do the same: writing that word from here drops USB.  So the
 * transfer itself runs on the auxiliary core (src/auxcore/auxmain.c,
 * AX_MODE_CPWIN), and this file is the main FR's half: it places that
 * program, starts the core, and passes requests through the slot.
 *
 * WHAT init() DOES, in order:
 *
 *   1. Release the front-end reset: 0x00F10018 bit1, then 0x00F10014 bit1
 *      (half-words; FE_RESET_* in mb86m01.h).  Without them the
 *      controller comes up but the demodulators do not answer (FW_EIO).  These two bits are the only
 *      main-FR writes the bring-up needs; the gates at 0x00F10118 /
 *      0x00F1011C, the unit resets at 0x00F101B4 / 0x00F10078 and
 *      0x00F001F0 are not (measured from cold, one write at a time).
 *   2. Copy the program carried in this image (build/auxcore_image.c) to
 *      its address in the aux RAM window, word by word with read-back, and
 *      zero its .bss, the slot and the liveness words.
 *   3. Write the mode word (AX_MODE_CPWIN) and the reset vector
 *      0x017FFFFC = the program's local entry.  The vector reads back 0
 *      from here, but the core takes it (measured).
 *   4. Start the core: 0x01700004 = 0, then 0x01700000 = 0xFFFCFFFF
 *      (bits 17:16 clear).  Leaving out either, the core never writes
 *      'AX1C' (measured on the DT295, one write left out at a time).
 *      Setting both words to all ones first, or 0x01700004 = 0x00800000,
 *      made no difference and is not done.
 *   5. Wait for 'AX1C' and take the program's own setup result.
 *
 * The slot protocol is src/auxcore/auxmain.c's; the offsets below must
 * match it.  Every field is written as a whole word: byte stores into the
 * aux RAM paint neighbouring bytes back with stale values (measured; see
 * auxmain.c, ax_put32).
 */
#include "fw.h"
#include "drivers.h"
#include "mb86m01.h"

/* build/auxcore_image.c */
extern const unsigned char aux_image[];
extern const unsigned aux_image_len;
extern const unsigned aux_image_bss;
extern const unsigned aux_image_base;
extern const unsigned aux_image_entry;

#define U17_CTL0        0x01700000u
#define U17_CTL4        0x01700004u
#define U17_VECTOR      0x017ffffcu

#define SLOT            0x017a3f00u
#define SLOT_BYTES      0x70u
#define O_MAGIC         0x00u
#define O_SEQ           0x04u
#define O_OP            0x08u
#define O_ADDR          0x0cu
#define O_WLEN          0x10u
#define O_RLEN          0x14u
#define O_RESULT        0x18u
#define O_ACK           0x1cu
#define O_WDATA         0x20u
#define O_RDATA         0x48u
#define SLOT_MAX        40u
#define MAGIC           0x41584932u     /* 'AXI2' */
#define OP_WRITE        1u
#define OP_WRITE_READ   2u

#define ALIVE           0x017a3f70u     /* 'AX1C' */
#define ALIVE_SIG       0x41583143u
#define SETUP_RC        0x017a3f78u
#define STATUS_END      0x017a3f8cu     /* ALIVE .. the published registers */
#define MODE            0x017a3f90u
#define MODE_CPWIN      0x43505731u     /* 'CPW1' */

/* Bounded waits, in polls of a word in the aux RAM.  The core answers
 * within a few hundred polls; a transfer of 40 bytes at the divider's
 * rate takes far less than the ack bound. */
#define ALIVE_POLLS     200000u
#define ACK_POLLS       2000000u

static unsigned seq;

static int place(void)
{
    unsigned i, n = aux_image_len;

    if (n == 0u || (n & 3u))
        return FW_ENOTIMPL;
    for (i = 0; i < n; i += 4u) {
        unsigned w = ((unsigned)aux_image[i] << 24)
                   | ((unsigned)aux_image[i + 1u] << 16)
                   | ((unsigned)aux_image[i + 2u] << 8)
                   | (unsigned)aux_image[i + 3u];
        MMIO(aux_image_base + i) = w;
        if (MMIO(aux_image_base + i) != w)
            return FW_EIO;
    }
    for (i = 0; i < aux_image_bss; i += 4u)
        MMIO(aux_image_base + n + i) = 0u;
    for (i = 0; i < SLOT_BYTES; i += 4u)
        MMIO(SLOT + i) = 0u;
    for (i = ALIVE; i < STATUS_END; i += 4u)
        MMIO(i) = 0u;
    return FW_OK;
}

static int auxslot_init(const struct board *b)
{
    unsigned i;
    int rc;

    (void)b;

    MMIO16(FE_RESET_DAT) = (unsigned short)(MMIO16(FE_RESET_DAT) |
                                            FE_RESET_BIT);
    MMIO16(FE_RESET_DIR) = (unsigned short)(MMIO16(FE_RESET_DIR) |
                                            FE_RESET_BIT);

    rc = place();
    if (rc != FW_OK)
        return rc;
    MMIO(MODE) = MODE_CPWIN;
    MMIO(U17_VECTOR) = aux_image_entry;

    MMIO(U17_CTL4) = 0u;
    MMIO(U17_CTL0) = 0xfffcffffu;

    for (i = 0; i < ALIVE_POLLS; i++)
        if (MMIO(ALIVE) == ALIVE_SIG)
            return (int)MMIO(SETUP_RC);
    return FW_ETIMEOUT;
}

static int auxslot_xfer(const struct board *b, unsigned addr,
                        const unsigned char *wb, unsigned wn,
                        unsigned char *rb, unsigned rn)
{
    unsigned i, j;

    (void)b;
    if (wn + rn > SLOT_MAX || addr > 0x7fu)
        return FW_EINVAL;

    for (i = 0; i < wn; i += 4u) {
        unsigned w = 0;
        for (j = 0; j < 4u; j++)
            w = (w << 8) | (i + j < wn ? (unsigned)wb[i + j] : 0u);
        MMIO(SLOT + O_WDATA + i) = w;
    }
    MMIO(SLOT + O_OP) = rn ? OP_WRITE_READ : OP_WRITE;
    MMIO(SLOT + O_ADDR) = addr;
    MMIO(SLOT + O_WLEN) = wn;
    MMIO(SLOT + O_RLEN) = rn;
    MMIO(SLOT + O_MAGIC) = MAGIC;
    /* The sequence number goes last: it is what the core waits for. */
    if (++seq == 0u)
        seq = 1u;
    MMIO(SLOT + O_SEQ) = seq;

    for (i = 0; i < ACK_POLLS; i++)
        if (MMIO(SLOT + O_ACK) == seq)
            break;
    if (i == ACK_POLLS)
        return FW_ETIMEOUT;

    for (i = 0; i < rn; i += 4u) {
        unsigned w = MMIO(SLOT + O_RDATA + i);
        for (j = 0; j < 4u && i + j < rn; j++)
            rb[i + j] = (unsigned char)(w >> (24u - 8u * j));
    }
    return (int)MMIO(SLOT + O_RESULT);
}

const struct i2c_ops i2c_auxslot_ops = {
    "aux-slot",
    auxslot_init,
    auxslot_xfer,
};
