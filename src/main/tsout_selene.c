/* tsout_selene.c -- the ring to the host, one 512-byte bulk burst at a time.
 *
 * THERE IS AN ENDPOINT TO WRITE TO.  The loader holds EP0 and this firmware
 * sets up no bulk endpoint, but a burst written straight into the
 * endpoint's FIFO goes out.  One burst is the whole answer --
 *
 *     r1 = [status]; if (!(r1 & 4)) return busy
 *     for (p = buf; p != buf + 0x200; p += 4)
 *         [fifo] = p[0]<<24 | p[1]<<16 | p[2]<<8 | p[3]
 *     [0x00FD814C] = 8; [status] = 4; [0x00FD814C] = 0x0F
 *
 * So: 512 bytes a burst, the ready bit is bit2 of the status register, and
 * the commit is the guard pair around a 4 into the status.  Byte-at-a-time
 * word assembly because the source is not promised to be aligned.  On a
 * PIX-DT295 the TS streams on this pair from inside the loader's hook with
 * nothing else set up, which is the measured evidence that **the endpoint
 * is already configured** -- the loader publishes the descriptors before any firmware
 * is downloaded, and the burst alone is enough.  This firmware therefore
 * configures nothing (mb86m01.h says what that would need and why we do not
 * have it).
 *
 * NO BUFFER.  A record in the ring is 192 bytes with a 188-byte TS packet
 * in front, and the burst is 512.  188 does not divide 512, so a packet gets
 * split across two bursts: the stream is a byte stream of packets back to
 * back, which is what the host expects of TS over bulk.  188 is a multiple
 * of 4, so every split falls on a word, and the words go from the ring
 * straight into the FIFO (ring_to_fifo).  Copying into a 512-byte buffer
 * first, one byte at a time, caps the stream at about 1.2 MB/s on a
 * PIX-DT295 (measured).
 *
 * The 4 bytes at the end of each record are not sent.  ring_offset() plus a
 * 188-byte read is what drops them.  What they are is not known.
 *
 * MEASURED on a PIX-DT295 (above).  Not measured on any other board.
 */
#include "fw.h"
#include "mb86m01.h"
#include "regs_selene.h"

static unsigned taken;          /* bytes of the current packet already sent */
static unsigned status;
static unsigned fifo;

int tsout_init(void)
{
    /* Pair 0 is the one TS streams on.  Nothing else to do: no
     * configuration, by the reasoning in the header.
     */
    status = USB_EP0_STATUS;
    fifo = USB_EP0_FIFO;
    taken = 0;
    return FW_OK;
}

int tsout_ready(int *ready)
{
    if (!ready)
        return FW_EINVAL;
    if (!status)
        return FW_EIO;
    *ready = (MMIO(status) & USB_EP_READY) ? 1 : 0;
    return FW_OK;
}

/* One burst, straight from the ring into the FIFO.  *bytes gets what went
 * out, 0 if nothing did.
 *
 * Nothing goes into the FIFO unless a whole burst can follow: a FIFO
 * cannot take words back, so the burst starts only when the endpoint is
 * ready and the ring holds 512 bytes of packets.
 */
int tsout_push(unsigned *bytes)
{
    unsigned avail, sent = 0, done = 0, want;
    int rc;

    if (bytes)
        *bytes = 0;
    if (!status)
        return FW_EIO;
    if (!usb_ep_ready(status))
        return FW_EAGAIN;

    /* Whole packets that can also be given back as whole records. */
    avail = ring_avail();
    if (avail && ring_used() < avail * TS_RECORD)
        avail--;
    if (avail * TS_PACKET < taken + USB_EP_BURST)
        return FW_EAGAIN;

    while (sent < USB_EP_BURST) {
        want = TS_PACKET - taken;
        if (want > USB_EP_BURST - sent)
            want = USB_EP_BURST - sent;
        rc = ring_to_fifo(ring_offset(done) + taken, want, fifo);
        if (rc != FW_OK)
            return rc;          /* the ring stopped making sense */
        sent += want;
        taken += want;
        if (taken == TS_PACKET) {
            done++;
            taken = 0;
        }
    }
    usb_ep_commit(status);
    if (done) {
        rc = ring_consume(done);
        if (rc != FW_OK)
            return rc;
    }
    if (bytes)
        *bytes = USB_EP_BURST;
    return FW_OK;
}
