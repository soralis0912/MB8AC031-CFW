/* pm.c -- SUSPEND: put down what this firmware owns.  Owns CMD_SUSPEND.
 *
 * A LIGHT SUSPEND.  The chip's own low-power path is not known, so this does
 * not touch power or clocks.  It stops the TS capture (stream_stop: tsif on
 * selene, the dione TS path) and nothing else.
 *
 * THE CARD IS LEFT ALONE.  Its power belongs to whoever opened it: the
 * PC/SC driver powers it down itself.  On a PIX-DT295, powering it down
 * here loses the card block's insertion bit (CARD_EVENT bit10) after the
 * next power-up until the card is pulled and pushed again, so after
 * SUSPEND the driver would see no card with the card in (docs/card.md).
 *
 * THE RF SIDE IS THE HOST'S.  The host tunes the tuner and the demodulator
 * by I2CXFER, so it also puts them to sleep the same way, before or after
 * this command.  The firmware writes neither.
 *
 * There is no RESUME.  Coming back is START, then the host tunes again,
 * then STREAM.
 *
 * The answer, in FR_DATA (big-endian):
 *   +0x00  u8   report version (1)
 *   +0x01  u8   what was put down: bit0 the TS capture was running and is
 *               stopped
 *   +0x04  s32  stream_stop()'s result
 *
 * The status is FW_OK unless stopping the capture failed.
 *
 * WHAT IS NOT HERE.  0x00FDD004 and 0x00FDC00C look like a low-power
 * path, but that page is not a power controller: the live dump shows 0x00010001 / 1 / 1 and
 * then 253 words of high-entropy data from +0x0C, a key or random store on
 * the secure side.  On dione the same addresses are on the bulk-OUT path
 * and read 0 on a GV-M2TV.  Writing them risks the endpoint, so nothing
 * here does.
 */
#include "fw.h"
#include "frq1.h"

#define PM_VERSION      0x00    /* u8  = 1                                */
#define PM_DOWN         0x01    /* u8  PM_D_*                             */
#define PM_STREAM_RC    0x04    /* s32                                    */

#define PM_D_STREAM     0x01u

int pm_suspend(frame_t f)
{
    unsigned down = 0;
    int rc = FW_OK;
    unsigned i;

    for (i = 0; i < 40u; i++)
        f[FR_DATA + i] = 0;

    if (stream_running()) {
        rc = stream_stop();
        if (rc == FW_OK)
            down |= PM_D_STREAM;
    }

    f[FR_DATA + PM_VERSION] = 1u;
    f[FR_DATA + PM_DOWN] = (unsigned char)down;
    frq1_put32(f, FR_DATA + PM_STREAM_RC, (unsigned)rc);
    return rc;
}
