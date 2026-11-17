/* i2c/mailbox.c -- reaching the tuner by asking the auxiliary unit.
 *
 * The second transport, meant for the GV-MVP/FZ2: driving the 0x00FD8F..
 * controller from the host on an FZ2 gets through setup but never
 * completes a transfer.  Whatever the FZ2 uses, it is not the same block at
 * the same address.
 *
 * WHAT IS ASSUMED (none of it confirmed on hardware).  The auxiliary unit
 * takes a command in a register file, and command 9 is I2C.  The words:
 *
 *     AUX_CMD        9 = I2C
 *     AUX_KIND       low byte, kind 0..4
 *     AUX_ADDR_LEN   high byte 7-bit address, low byte length (<= 0x40)
 *     AUX_RETURN     0x80 for kind 3, 0 otherwise
 *
 * The main CPU wakes it by writing a word to AUX_MBOX and then
 * AUX_BELL_RING to AUX_BELL, and the unit answers by writing AUX_DONE_FLAG
 * as a half-word to AUX_DONE.  The receiver sits in idle until woken, which
 * is why nothing happens to a chip that is otherwise set up correctly.
 *
 * THE HANDSHAKE WORDS.
 *
 *     The data bytes go through the 0x00F0 page: AUX_I2C_TX to send,
 *     AUX_I2C_RX to receive (mb86m01.h).
 *
 *     0x017A3900 is a SIGNATURE: the mailbox is taken to be alive only when
 *     it reads 'DTI2' (0x44544932), with 0x017A3904 == 2 and
 *     0x017A391C == 1.  So that word is how a host asks "is the auxiliary
 *     mailbox alive at all" -- a measurable signal, and on a state-0 FZ2 it
 *     is not there (measured: the aux SRAM holds power-on junk).
 *
 *     0x017A3904 is the word that goes in before the bell: a single high
 *     bit naming the request, e.g. [0x017A3904] = 0x80000000 then
 *     [0x01700080] = 0x20000000, and 0x10000000 for another.  Bits 16/17/20
 *     of the same word are taken to be block-enable flags, so it carries
 *     both.
 *
 * WHAT IS NOT, and why xfer() refuses.  Which high bit reaches the command-9
 * handler is not known, and that handler would be an auxiliary program this
 * repository does not load (mb86m01.h, "WHAT STILL STOPS i2c/mailbox.c").
 * The way in is our own auxiliary program (src/auxcore/): it serves
 * requests left in its slot at 0x017A3F00, a protocol of our own, not
 * command 9.  This file does not use that slot yet.
 *
 * So this file sets up what it can verify and stops.  Ringing a bell for a
 * request that is half filled in is worse than not ringing it: the auxiliary
 * unit would run a transfer against an address and a length it was given and
 * a payload it was not.
 *
 * To finish it: measure 0x017A3900-0x017A3918 from the host's window on
 * hardware, or move the transfer to our own auxiliary program's slot.
 */
#include "fw.h"
#include "drivers.h"
#include "mb86m01.h"

static int mailbox_init(const struct board *b)
{
    (void)b;

    /* **THIS TOUCHES NO REGISTER, ON PURPOSE.**
     *
     * Not even RF_ENABLE_114: nothing measures it safe on this product, and
     * mb86m01.h explains why the register is more delicate here than on a
     * DT295.  And this transport cannot carry a transfer anyway -- so writing
     * to the chip on the way to returning FW_ENOTIMPL buys nothing and risks
     * something.
     */
    return FW_ENOTIMPL;
}

static int mailbox_xfer(const struct board *b, unsigned addr,
                        const unsigned char *wb, unsigned wn,
                        unsigned char *rb, unsigned rn)
{
    (void)b; (void)addr; (void)wb; (void)wn; (void)rb; (void)rn;
    return FW_ENOTIMPL;
}

const struct i2c_ops i2c_mailbox_ops = {
    "aux-mailbox",
    mailbox_init,
    mailbox_xfer,
};
