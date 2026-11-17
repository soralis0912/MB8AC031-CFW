/* stream.c -- hand the TS to the host, one request's worth at a time.
 *
 * Owns the streaming half of CMD_STREAM and CMD_STOP.
 *
 * HOW THE STREAM LEAVES THE CHIP.  Through an endpoint that is already
 * configured when we run: the burst goes out on the pair (0x00FDBD0C,
 * 0x00FDBD84), and the loader publishes the descriptors before
 * any firmware is downloaded.  tsout_selene.c is that burst (measured on a
 * PIX-DT295).
 *
 * SO WHAT IS LEFT IS THE SHAPE OF A REQUEST, and the shape is forced by one
 * fact: **there is no main loop here.** A frame arrives, we are called, we
 * return.  Nothing runs between requests -- on the FRC1 path we are inside
 * the loader's handling of one control transfer, and while we are in
 * here no other control transfer reaches the device.  So a request cannot
 * mean "stream until I say stop".  It means "move what you can now, and
 * tell me what you did", and the host comes back.
 *
 * The host therefore sets the pace.  It says how many bursts to try in one
 * request (arg0), which is how it trades latency against the number of
 * control transfers; we stop early the moment there is nothing ready,
 * because spinning on an empty ring would hold off the request that wants
 * to stop us.
 *
 * A COUNT, NOT A DEADLINE.  A millisecond budget is something this
 * firmware cannot honour: there is no clock here (fe.c has the
 * measurement).  A count we can honour exactly, so FRQ1 has no budget field
 * (docs/frq1.md).
 *
 * WHAT IS MEASURED.  The burst, on a PIX-DT295 (tsout_selene.c).  The
 * capture block's bring-up says for itself what is measured
 * (tsif_selene.c).
 */
#include "fw.h"
#include "mb86m01.h"

/* The report CMD_STREAM leaves in FR_DATA.  Ours; 40 bytes, big-endian. */
#define SR_VERSION      0x00    /* u8  = 1                               */
#define SR_FLAGS        0x01    /* u8  see below                         */
#define SR_BURSTS       0x04    /* u32 bursts that went out              */
#define SR_BYTES        0x08    /* u32 bytes that went out               */
#define SR_USED         0x0c    /* u32 bytes still in the ring           */
#define SR_RC           0x10    /* s32 why we stopped (0 = ran out of
                                 *     requested bursts)                 */
#define SR_REPORT_V     1u

#define SR_F_RUNNING    0x01u   /* the capture block is up               */
#define SR_F_SYNCED     0x02u   /* the record boundary is known          */
#define SR_F_DRAINED    0x04u   /* stopped because nothing was ready     */

#define SR_BURSTS_MAX   64u     /* 32 KiB a request, the host's ceiling  */
#define SR_BURSTS_DEF   8u      /* what a 0 argument means               */

static unsigned running;

/* Bring the path up, bottom first.  Each step is idempotent, so a host that
 * sends STREAM twice does not restart the capture block under itself.
 */
int stream_init(void)
{
    int rc;

    if (running)
        return FW_OK;

    rc = tsif_init();
    if (rc != FW_OK)
        return rc;
    rc = tsif_start();
    if (rc != FW_OK)
        return rc;

    /* ring_init reads the base and the length back out of the registers
     * tsif_start just wrote, so it goes after, not before.
     */
    rc = ring_init();
    if (rc != FW_OK) {
        (void)tsif_stop();
        return rc;
    }
    rc = tsout_init();
    if (rc != FW_OK) {
        (void)tsif_stop();
        return rc;
    }
    running = 1u;
    return FW_OK;
}

int stream_running(void)
{
    if (fw_is_dione())
        return dione_ts_running();
    return running != 0u;
}

int stream_stop(void)
{
    int rc;

    if (fw_is_dione())
        return dione_ts_stop();
    if (!running)
        return FW_OK;           /* stopping what never started is not a fault */
    rc = tsif_stop();
    running = 0u;
    return rc;
}

/* One request's worth.
 *
 * The frame is answered with what happened either way: zero bursts because
 * nothing had arrived yet is not a failure, and a host that is polling needs
 * to tell that apart from a path that is broken.  Only a path that cannot be
 * brought up at all fails.
 */
int stream_start(frame_t f)
{
    unsigned want = frq1_be32(f, FR_ARG0);
    unsigned bursts = 0;
    unsigned bytes = 0;
    unsigned flags = 0;
    unsigned sent;
    int rc, last = FW_OK;
    unsigned i;

    /* dione has its own capture block and its own ring rules. */
    if (fw_is_dione())
        return dione_stream(f);

    for (i = 0; i < 40u; i++)
        f[FR_DATA + i] = 0;
    f[FR_DATA + SR_VERSION] = (unsigned char)SR_REPORT_V;

    rc = stream_init();
    if (rc != FW_OK) {
        frq1_put32(f, FR_DATA + SR_RC, (unsigned)rc);
        return rc;
    }
    flags |= SR_F_RUNNING;

    if (!want)
        want = SR_BURSTS_DEF;
    if (want > SR_BURSTS_MAX)
        want = SR_BURSTS_MAX;

    /* Find the record boundary if it is not known yet.  FW_EAGAIN here means
     * the ring has not got enough in it to look -- nothing is wrong, the
     * host should come back.
     */
    rc = ring_sync();
    if (rc == FW_OK)
        flags |= SR_F_SYNCED;
    else
        last = rc;

    if (rc == FW_OK) {
        for (i = 0; i < want; i++) {
            rc = tsout_push(&sent);
            if (rc != FW_OK) {
                last = rc;
                if (rc == FW_EAGAIN)
                    flags |= SR_F_DRAINED;
                break;
            }
            bursts++;
            bytes += sent;
        }
    }

    f[FR_DATA + SR_FLAGS] = (unsigned char)flags;
    frq1_put32(f, FR_DATA + SR_BURSTS, bursts);
    frq1_put32(f, FR_DATA + SR_BYTES, bytes);
    frq1_put32(f, FR_DATA + SR_USED, ring_used());
    frq1_put32(f, FR_DATA + SR_RC, (unsigned)last);
    return FW_OK;
}
