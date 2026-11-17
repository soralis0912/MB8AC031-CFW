/* health.c -- what state the firmware is in, and what last went wrong.
 * Owns CMD_HEALTH.
 *
 * The router hands every answered frame to health_note(), so each part
 * keeps its FIRST failure (the one that usually explains the rest) plus a
 * count, until the host clears them.  FW_EAGAIN is not a failure: it is
 * how a handler with a wait in it says "not yet".
 *
 * The report (docs/frq1.md), in the frame's data:
 *
 *   +0x00  u8   version (HR_REPORT_V)
 *   +0x01  u8   phase: 0 loaded, 1 started, 2 streaming
 *   +0x02  u16  flags (HF_*)
 *   +0x04  u32  frames answered since load (the HEALTH frame itself too)
 *   +0x08  per part, 4 bytes each, HP_PARTS of them:
 *            s8   first failure code (0 = none)
 *            u8   command that gave it
 *            u16  failures since the last clear, saturating
 *   +0x24  s8   the first part's failure in that order (device first),
 *               so a host that wants one answer reads one byte
 *   +0x25  u8   that part, 0xff if none
 *   +0x26  u8   its command
 *
 * arg0 bit0: clear the failures after this report.
 */
#include "fw.h"

#define HR_REPORT_V     1u
#define HR_PARTS        0x08
#define HR_FIRST        0x24

#define HF_STARTED      0x0001u /* START answered, no STOP since          */
#define HF_TS           0x0002u /* the capture is running                 */
#define HF_CARD_ON      0x0004u /* card.c has the card powered            */
#define HF_CARD_ARM     0x0008u /* an ARM core owns the card block        */
#define HF_DIONE        0x0010u /* this image was linked for dione        */
#define HF_FAILED       0x0020u /* some part holds a failure              */

/* Parts follow the command bands (src/shared/frq1.h). */
#define HP_DEVICE       0u      /* 0x1x */
#define HP_FE           1u      /* 0x2x */
#define HP_TS           2u      /* 0x3x */
#define HP_PROBE        3u      /* 0x4x, I2C included */
#define HP_CARD         4u      /* 0x5x */
#define HP_ARM          5u      /* 0x6x */
#define HP_OTHER        6u      /* a command nobody defined */
#define HP_PARTS        7u

/* .bss: kept between frames. */
static unsigned started;
static unsigned frames;
static int first_rc[HP_PARTS];
static unsigned first_cmd[HP_PARTS];
static unsigned count[HP_PARTS];

static unsigned part_of(unsigned cmd)
{
    switch (cmd >> 4) {
    case 1u: return HP_DEVICE;
    case 2u: return HP_FE;
    case 3u: return HP_TS;
    case 4u: return HP_PROBE;
    case 5u: return HP_CARD;
    case 6u: return HP_ARM;
    default: return HP_OTHER;
    }
}

void health_started(int on)
{
    started = on ? 1u : 0u;
}

void health_note(unsigned cmd, int rc)
{
    unsigned p;

    frames++;
    if (rc >= 0 || rc == FW_EAGAIN)
        return;
    p = part_of(cmd);
    if (!first_rc[p]) {
        first_rc[p] = rc;
        first_cmd[p] = cmd & 0xffu;
    }
    if (count[p] < 0xffffu)
        count[p]++;
}

int health_report(frame_t f)
{
    unsigned flags = 0;
    unsigned phase = 0;
    unsigned p, i;

    for (i = 0; i < 40u; i++)
        f[FR_DATA + i] = 0;

    if (started)
        flags |= HF_STARTED;
    if (stream_running())
        flags |= HF_TS;
    if (card_available() && !card_idle())
        flags |= HF_CARD_ON;
    if (arm_has_card())
        flags |= HF_CARD_ARM;
    if (fw_is_dione())
        flags |= HF_DIONE;
    for (p = 0; p < HP_PARTS; p++)
        if (first_rc[p])
            flags |= HF_FAILED;

    if (flags & HF_TS)
        phase = 2u;
    else if (flags & HF_STARTED)
        phase = 1u;

    f[FR_DATA + 0] = (unsigned char)HR_REPORT_V;
    f[FR_DATA + 1] = (unsigned char)phase;
    f[FR_DATA + 2] = (unsigned char)(flags >> 8);
    f[FR_DATA + 3] = (unsigned char)flags;
    /* This frame is counted by the router after it is answered, so add it
     * here to make the number the same as the host's own count. */
    frq1_put32(f, FR_DATA + 4, frames + 1u);
    for (p = 0; p < HP_PARTS; p++) {
        int o = FR_DATA + HR_PARTS + 4 * (int)p;

        f[o] = (unsigned char)first_rc[p];
        f[o + 1] = (unsigned char)first_cmd[p];
        f[o + 2] = (unsigned char)(count[p] >> 8);
        f[o + 3] = (unsigned char)count[p];
    }

    f[FR_DATA + HR_FIRST + 1] = 0xffu;
    for (p = 0; p < HP_PARTS; p++)
        if (first_rc[p]) {
            f[FR_DATA + HR_FIRST] = (unsigned char)first_rc[p];
            f[FR_DATA + HR_FIRST + 1] = (unsigned char)p;
            f[FR_DATA + HR_FIRST + 2] = (unsigned char)first_cmd[p];
            break;
        }

    if (frq1_be32(f, FR_ARG0) & 1u)
        for (p = 0; p < HP_PARTS; p++) {
            first_rc[p] = 0;
            first_cmd[p] = 0;
            count[p] = 0;
        }
    return FW_OK;
}
