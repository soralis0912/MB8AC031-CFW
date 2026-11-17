/* fe.c -- the frontend: what the host can be told about the signal.
 *
 * Owns CMD_STATUS, CMD_CNR and CMD_STOP's frontend half.
 *
 * HOW LONG TO WAIT FOR LOCK IS NOT DECIDED HERE.  This firmware has no
 * clock, and the frame carries no millisecond budget, because a number this
 * firmware cannot honour is a lie in the header (docs/frq1.md).  The host
 * keeps the clock.
 *
 * So this file does not wait.  It answers what is true at the moment it is
 * asked, and the host polls.  A weak channel is the host's patience to
 * spend rather than a constant compiled in here.
 *
 * WHAT WORKS: reporting.  demod.c can read lock and C/N now
 * (docs/demod.md), so STATUS and CNR return real values.
 *
 * WHAT IS NOT HERE: tuning.  There is no tune command.  The host writes the
 * tuner and the demodulator itself, one I2C transfer a frame (I2CXFER), and
 * polls STATUS -- the same split as the clock above: the sequence, like the
 * patience, lives on the host.  IDENTIFY page 1 marks every band
 * BAND_HOST_TUNED (docs/frq1.md).
 */
#include "fw.h"
#include "drivers.h"

/* The report CMD_STATUS writes into FR_DATA:
 * 40 bytes, big-endian, version first so it can grow.
 */
#define FE_VERSION      0x00    /* u8  = 1                               */
#define FE_FLAGS        0x01    /* u8  see below                         */
#define FE_SOURCE       0x02    /* u8  the device index, 0..3             */
#define FE_LOCKED       0x04    /* u32 0 or 1                            */
#define FE_CNR          0x08    /* u32 raw register value, not dB        */
#define FE_LOCK_RC      0x0c    /* s32 result of the lock read           */
#define FE_CNR_RC       0x10    /* s32 result of the C/N read            */

#define FE_F_LOCK_OK    0x01u   /* the lock read answered                */
#define FE_F_CNR_OK     0x02u   /* the C/N read answered                 */
#define FE_F_PRESENT    0x04u   /* something answered at this address    */
#define FE_F_NOINIT     0x80u   /* CMD_START has not run                 */

#define FE_REPORT_V     1u

static unsigned ready;
static unsigned char present[FW_SRC_COUNT];

/* arg0's low two bits are the device index: 0 and 1 are the
 * terrestrial demodulators, 2 and 3 the satellite ones (drivers.h).
 */
static unsigned source_of(frame_t f)
{
    return frq1_be32(f, FR_ARG0) & 3u;
}

/* "Initialised" here means the bus is up and we know which demodulators
 * answer.  It does NOT mean they are configured: that needs a bring-up
 * register sequence this tree does not have (docs/demod.md).  Saying FW_OK
 * for the part that did happen, and letting the report carry the rest, beats
 * one return value standing for both.
 */
int fe_init(void)
{
    unsigned s;
    int rc;
    int locked;

    ready = 0;
    for (s = 0; s < FW_SRC_COUNT; s++)
        present[s] = 0;

    rc = i2c_init();
    if (rc != FW_OK)
        return rc;

    /* The lock read is the probe: it answers FW_OK only if the part on the
     * bus replied, FW_ENOTIMPL if this board has no address for that side,
     * and an I/O error if nothing is there.
     */
    for (s = 0; s < FW_SRC_COUNT; s++) {
        rc = demod_locked(s, &locked);
        present[s] = (rc == FW_OK) ? 1u : 0u;
    }
    ready = 1;
    return FW_OK;
}

/* Nothing is running, so there is nothing to stop.  Reporting success for
 * that is honest -- the postcondition the caller wants (not tuned, not
 * streaming) holds.
 */
int fe_stop(void)
{
    return FW_OK;
}

int fe_channel_info(frame_t f)
{
    unsigned source = source_of(f);
    unsigned flags = 0;
    unsigned raw = 0;
    int locked = 0;
    int lock_rc, cnr_rc;
    unsigned i;

    for (i = 0; i < 40u; i++)
        f[FR_DATA + i] = 0;

    f[FR_DATA + FE_VERSION] = (unsigned char)FE_REPORT_V;
    f[FR_DATA + FE_SOURCE] = (unsigned char)source;
    if (!ready)
        flags |= FE_F_NOINIT;
    else if (present[source])
        flags |= FE_F_PRESENT;

    lock_rc = demod_locked(source, &locked);
    if (lock_rc == FW_OK)
        flags |= FE_F_LOCK_OK;
    cnr_rc = demod_cnr_raw(source, &raw);
    if (cnr_rc == FW_OK)
        flags |= FE_F_CNR_OK;

    f[FR_DATA + FE_FLAGS] = (unsigned char)flags;
    frq1_put32(f, FR_DATA + FE_LOCKED, locked ? 1u : 0u);
    frq1_put32(f, FR_DATA + FE_CNR, raw);
    frq1_put32(f, FR_DATA + FE_LOCK_RC, (unsigned)lock_rc);
    frq1_put32(f, FR_DATA + FE_CNR_RC, (unsigned)cnr_rc);

    /* The frame is answered either way: a host that asked for the state gets
     * the state, and the per-read results inside say which halves are real.
     * Only a request this firmware cannot answer at all fails.
     */
    if (lock_rc == FW_ENOTIMPL && cnr_rc == FW_ENOTIMPL)
        return FW_ENOTIMPL;
    return FW_OK;
}

/* CMD_CNR: the raw value on its own, for a host that polls it often.  Same
 * numbers as the report, fewer bytes to read back.
 */
int fe_cnr(frame_t f)
{
    unsigned source = source_of(f);
    unsigned raw = 0;
    int rc;
    unsigned i;

    for (i = 0; i < 40u; i++)
        f[FR_DATA + i] = 0;

    rc = demod_cnr_raw(source, &raw);
    f[FR_DATA + FE_VERSION] = (unsigned char)FE_REPORT_V;
    f[FR_DATA + FE_SOURCE] = (unsigned char)source;
    if (rc == FW_OK)
        f[FR_DATA + FE_FLAGS] = (unsigned char)FE_F_CNR_OK;
    frq1_put32(f, FR_DATA + FE_CNR, raw);
    frq1_put32(f, FR_DATA + FE_CNR_RC, (unsigned)rc);
    return rc;
}
