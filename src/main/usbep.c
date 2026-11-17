/* usbep.c -- one burst out of one endpoint FIFO.
 *
 * tsout_selene.c sends TS from the ring through here.  It takes five
 * steps, and the shape is measured (mb86m01.h's USB_EP_* block): check bit2
 * of the status register, write 128 words into the FIFO, then 8 / commit /
 * 0xF around the guard at 0x00FD814C.
 *
 * ONE IMPLEMENTATION ON PURPOSE.  The guard pair is the part that would be
 * easy to get subtly wrong in a copy, so every caller shares this one.
 *
 * THE SOURCE MAY BE UNALIGNED.  Words are assembled from four bytes because
 * the TS ring's records are not word aligned; the cost is nothing here and
 * it means a caller never has to think about it.
 */
#include "fw.h"
#include "mb86m01.h"

/* The two halves of a burst, for a caller that fills the FIFO itself
 * (tsout_selene.c, straight from the ring).  Same steps, same order. */
int usb_ep_ready(unsigned status_reg)
{
    return (MMIO(status_reg) & USB_EP_READY) ? 1 : 0;
}

void usb_ep_commit(unsigned status_reg)
{
    MMIO(USB_EP_GUARD) = USB_EP_OPEN;
    MMIO(status_reg) = USB_EP_COMMIT;
    MMIO(USB_EP_GUARD) = USB_EP_CLOSED;
}

int usb_ep_burst(unsigned status_reg, unsigned fifo_reg,
                 const volatile unsigned char *src)
{
    unsigned i;

    if (!status_reg || !fifo_reg || !src)
        return FW_EINVAL;

    /* The endpoint's own flow control.  A busy endpoint is not an error:
     * its buffer is full and the caller should come back. */
    if (!(MMIO(status_reg) & USB_EP_READY))
        return FW_EAGAIN;

    /* Big-endian, so an aligned word load is the same word. */
    if (!((unsigned)src & 3u)) {
        const volatile unsigned *w = (const volatile unsigned *)src;
        for (i = 0; i < USB_EP_BURST / 4u; i++)
            MMIO(fifo_reg) = w[i];
    } else {
        for (i = 0; i < USB_EP_BURST; i += 4u)
            MMIO(fifo_reg) =
                ((unsigned)src[i] << 24) | ((unsigned)src[i + 1u] << 16) |
                ((unsigned)src[i + 2u] << 8) | (unsigned)src[i + 3u];
    }

    usb_ep_commit(status_reg);
    return FW_OK;
}
