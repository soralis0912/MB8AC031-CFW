/* device.c -- the one command a host asks before anything else.
 *
 * FRQ1 has one such command, IDENTIFY (docs/frq1.md).
 */
#include "fw.h"
#include "drivers.h"

/* IDENTIFY (0x10).  Nothing on the host side parses this payload beyond the
 * signature: it checks that, and that the status changed.  So the shape is
 * ours.  Keep it self-describing so a dump says whose runtime answered and
 * which version of the frame it speaks.
 *
 * The receive-source mask is 0 on purpose: this firmware does not tune
 * (there is no tune command; the host tunes by I2CXFER), and claiming a
 * source would make a host expect it to.  Which bands the host can tune is
 * on IDENTIFY page 1.  What the build CAN do is
 * in the capability word instead.
 *
 * The board word is the USB ID of the product this image was built for
 * (its row, fw_board), or 0 for an image built for no product.  A host can
 * check here that it installed the right container.
 *
 * The build word is the CRC-32 of this image's own .text, worked out the
 * first time it is asked.  .text is the container's `main` segment byte for
 * byte, and the image has no .data, so the host can compare it with the
 * segment's crc32 in the .frc it has (frc1_load info) and know whether the
 * build that answers is that one.
 */
#define ID_BOARD        8u      /* u32 vid<<16 | pid of the row, 0 = none */
#define ID_CAPS         12u     /* u32 ID_CAP_* */
#define ID_BUILD        16u     /* u32 CRC-32 of .text                    */

#define ID_CAP_DIONE    0x01u   /* the dione image: STREAM is ts_dione.c, */
                                /*     I2CXFER is there                   */
#define ID_CAP_CARD     0x02u   /* the row lets the card commands run     */

extern const unsigned char _text_start[], _text_end[];

/* CRC-32 as zlib has it (reflected 0xEDB88320, inverted in and out), a
 * nibble at a time.  The words are read whole, as the code band is. */
static const unsigned crc_nib[16] = {
    0x00000000u, 0x1db71064u, 0x3b6e20c8u, 0x26d930acu,
    0x76dc4190u, 0x6b6b51f4u, 0x4db26158u, 0x5005713cu,
    0xedb88320u, 0xf00f9344u, 0xd6d6a3e8u, 0xcb61b38cu,
    0x9b64c2b0u, 0x86d3d2d4u, 0xa00ae278u, 0xbdbdf21cu,
};

static unsigned crc_byte(unsigned c, unsigned b)
{
    c ^= b;
    c = (c >> 4) ^ crc_nib[c & 15u];
    return (c >> 4) ^ crc_nib[c & 15u];
}

static unsigned build_id;
static unsigned char have_build_id;

static unsigned text_crc(void)
{
    const volatile unsigned *w = (const volatile unsigned *)_text_start;
    unsigned n = (unsigned)(_text_end - _text_start), c = ~0u, v, i;

    for (i = 0; i + 4u <= n; i += 4u) {
        v = *w++;                       /* big-endian: byte 0 is the top */
        c = crc_byte(c, v >> 24);
        c = crc_byte(c, (v >> 16) & 0xffu);
        c = crc_byte(c, (v >> 8) & 0xffu);
        c = crc_byte(c, v & 0xffu);
    }
    for (; i < n; i++)
        c = crc_byte(c, _text_start[i]);
    return ~c;
}

/* Page 1: the board's receive bands, from its row (config/devices/,
 * "bands").  One record per band:
 *
 *   +0x00  u8   page report version (1)
 *   +0x01  u8   page (1)
 *   +0x02  u8   bands that follow
 *   +0x04  per band, 16 bytes:
 *            u8   source (0 T, 1 T2, 2 S, 3 S2)
 *            u8   flags: 0x01 host tunes it, 0x02 a lock was measured
 *            u16  record length in bytes
 *            u32  lowest centre frequency, kHz
 *            u32  highest centre frequency, kHz
 *            u32  the tuner's step, Hz
 */
#define ID_BANDS        4
#define ID_BAND_BYTES   16

static void identify_bands(frame_t f)
{
    unsigned i, n = fw_board.nbands;

    if (n > FW_BANDS)
        n = FW_BANDS;
    f[FR_DATA + 0] = 1;
    f[FR_DATA + 1] = 1;
    f[FR_DATA + 2] = (unsigned char)n;
    for (i = 0; i < n; i++) {
        const struct band *b = &fw_board.bands[i];
        int o = FR_DATA + ID_BANDS + ID_BAND_BYTES * (int)i;

        f[o] = b->source;
        f[o + 1] = b->flags;
        f[o + 2] = (unsigned char)(b->record >> 8);
        f[o + 3] = (unsigned char)b->record;
        frq1_put32(f, o + 4, b->lo_khz);
        frq1_put32(f, o + 8, b->hi_khz);
        frq1_put32(f, o + 12, b->step_hz);
    }
}

int dev_identify(frame_t f)
{
    unsigned caps = 0;
    unsigned page = frq1_be32(f, FR_ARG0);
    unsigned i;

    /* Older hosts send arg0 = 0 and get page 0, as before. */
    if (page > 1u)
        return FW_EINVAL;
    for (i = 0; i < 40u; i++)
        f[FR_DATA + i] = 0;
    if (page == 1u) {
        identify_bands(f);
        return FW_OK;
    }

    if (fw_is_dione())
        caps |= ID_CAP_DIONE;
    if (card_available())
        caps |= ID_CAP_CARD;

    if (!have_build_id) {
        build_id = text_crc();
        have_build_id = 1;
    }
    f[FR_DATA + 0] = 3;                         /* report version        */
    f[FR_DATA + 1] = (unsigned char)FRQ1_VERSION;
    f[FR_DATA + 2] = 1;                         /* receive chains        */
    f[FR_DATA + 3] = 0;                         /* usable source mask    */
    frq1_put32(f, FR_DATA + 4, 0x46523031u);    /* 'FR01'                */
    frq1_put32(f, FR_DATA + ID_BOARD, fw_board.usb);
    frq1_put32(f, FR_DATA + ID_CAPS, caps);
    frq1_put32(f, FR_DATA + ID_BUILD, build_id);
    return FW_OK;
}
