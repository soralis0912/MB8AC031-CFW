/* src/armcore/armmain.c -- what our ARM image does once it runs.
 *
 * WHAT IS KNOWN
 *   - It runs on ARM-E and ARM-C of a GV-M2TV from ITCM 0, with DTCM at
 *     0x00100000 (measured; docs/arm.md).
 *   - The main FR reads and writes DTCM through its window (0x00E80000 for
 *     ARM-E), so the mailbox and the work queue there are its way in and
 *     out.
 *
 * WHAT IS NOT
 *   - The card service (svc_card.c) has not run on hardware yet.  The
 *     queue, SVC_SYS and SVC_WATCH have (docs/arm.md).
 *   - Any timer on this core.  Waits count passes of the loop below.
 *
 * SHAPE
 *   One loop, no interrupts (README.md says why).  Each pass:
 *     0. the cancel words (ipc.c): checked before anything else, so a
 *        cancel gets through even when both rings are full
 *     1. the mailbox (diag.c): LOOPS counts, a raw request is answered
 *     2. at most one request from the queue: a service answers it now, or
 *        keeps it to answer later
 *     3. every service with something pending gets a step (svc_sys.c,
 *        svc_watch.c, svc_card.c)
 *   No step waits inside itself.  That is what makes stopping work without
 *   interrupts: whatever a task is doing, the next pass comes within a
 *   microsecond or so and sees the cancel.
 *   ALIVE is written last in start-up, so one read says "it ran" and the
 *   queue is ready when it says so.
 */
#include "core.h"

static void serve(void)
{
    struct req q;
    struct rep r;
    unsigned i, now = 1;

    if (!ipc_take(&q))
        return;
    r.seq = q.seq;
    r.call = q.call;
    r.status = IPC_ST_OK;
    r.val = 0;
    for (i = 0; i < IPC_DATA_WORDS; i++)
        r.data[i] = 0;

    switch (IPC_CALL_SVC(q.call)) {
    case SVC_SYS:
        now = (unsigned)sys_call(&q, &r);
        break;
    case SVC_WATCH:
        now = (unsigned)watch_call(&q, &r);
        break;
    case SVC_CARD:
        now = (unsigned)card_call(&q, &r);
        break;
    default:
        r.status = IPC_ST_BADSVC;
        break;
    }
    /* ipc_take took it only with a reply slot free, so this fits. */
    if (now)
        ipc_put(&r);
}

void arm_start(void)
{
    diag_init();
    ipc_init();
    MB(ARM_MB_ALIVE) = ARM_ALIVE_MAGIC;
}

void arm_pass(void)
{
    ipc_cancel_poll();          /* first: it must not depend on the rings */
    diag_poll();
    serve();
    sys_tick();
    watch_tick();
    card_tick();
}

void arm_main(void)
{
    arm_start();
    for (;;)
        arm_pass();
}
