/* src/armcore/ipc.c -- the ARM's end of the work queue (armipc.h).
 *
 * The ARM is the consumer of the request ring and the producer of the reply
 * ring, so it writes REQ_TAIL, REP_HEAD and EVENTS and only reads the other
 * two.  A request is taken only when a reply slot is free, so a service that
 * answers at once always has room; a service that answers later retries
 * until there is room (svc_watch.c).
 */
#include "core.h"

static unsigned served;

static volatile unsigned *slot(unsigned ring, unsigned count)
{
    return &DTCM_WORD(ARM_IPC + ring + (count % IPC_SLOTS) * IPC_SLOT_BYTES);
}

/* The main FR zeroed the header before starting us (arm.c), so the counts
 * start at 0 on both sides.  MAGIC last: from then on the main FR queues. */
void ipc_init(void)
{
    IPC(IPC_REQ_TAIL) = IPC(IPC_REQ_HEAD);
    IPC(IPC_REP_HEAD) = 0;
    IPC(IPC_EVENTS) = 0;
    IPC(IPC_CANCEL_DONE) = 0;
    IPC(IPC_CANCEL_DROPPED) = 0;
    IPC(IPC_CANCEL_ACK) = IPC(IPC_CANCEL_REQ);
    BARRIER();
    IPC(IPC_MAGIC) = IPC_MAGIC_VAL;
}

static int room(void)
{
    return IPC(IPC_REP_HEAD) - IPC(IPC_REP_TAIL) < IPC_SLOTS;
}

/* -> 1 with *q filled and the request taken, 0 if none (or no room to
 * answer it). */
int ipc_take(struct req *q)
{
    unsigned tail = IPC(IPC_REQ_TAIL);
    volatile unsigned *s;
    unsigned i;

    if (IPC(IPC_REQ_HEAD) == tail || !room())
        return 0;
    BARRIER();
    s = slot(IPC_REQ, tail);
    q->seq = s[IPC_Q_SEQ / 4u];
    q->call = s[IPC_Q_CALL / 4u];
    q->arg0 = s[IPC_Q_ARG0 / 4u];
    q->arg1 = s[IPC_Q_ARG1 / 4u];
    for (i = 0; i < IPC_DATA_WORDS; i++)
        q->data[i] = s[IPC_Q_DATA / 4u + i];
    BARRIER();
    IPC(IPC_REQ_TAIL) = tail + 1u;
    served++;
    return 1;
}

/* -> 1 queued, 0 the ring is full (try again later). */
int ipc_put(const struct rep *r)
{
    unsigned head = IPC(IPC_REP_HEAD);
    volatile unsigned *s;
    unsigned i;

    if (!room())
        return 0;
    s = slot(IPC_REP, head);
    s[IPC_R_SEQ / 4u] = r->seq;
    s[IPC_R_CALL / 4u] = r->call;
    s[IPC_R_STATUS / 4u] = r->status;
    s[IPC_R_VAL / 4u] = r->val;
    for (i = 0; i < IPC_DATA_WORDS; i++)
        s[IPC_R_DATA / 4u + i] = r->data[i];
    BARRIER();
    IPC(IPC_REP_HEAD) = head + 1u;
    if (r->call & IPC_EVENT)
        IPC(IPC_EVENTS) = IPC(IPC_EVENTS) + 1u;
    return 1;
}

unsigned ipc_served(void)
{
    return served;
}

/* The cancel words (armipc.h).  Called first on every pass, so a cancel
 * gets through whatever the rings hold.  REQ is read before WHAT; if the
 * main FR were to change WHAT in between, the next pass sees REQ != ACK
 * again and handles it again, and ending a task twice ends it once. */
void ipc_cancel_poll(void)
{
    unsigned req = IPC(IPC_CANCEL_REQ);
    unsigned what, dropped = 0;

    if (req == IPC(IPC_CANCEL_ACK))
        return;
    BARRIER();
    what = IPC(IPC_CANCEL_WHAT);
    IPC(IPC_CANCEL_DONE) = watch_cancel(what) + card_cancel(what)
                           + sys_cancel(what);
    if (what == IPC_CANCEL_ALL) {
        unsigned head = IPC(IPC_REQ_HEAD);

        dropped = head - IPC(IPC_REQ_TAIL);
        IPC(IPC_REQ_TAIL) = head;
    }
    IPC(IPC_CANCEL_DROPPED) = dropped;
    BARRIER();
    IPC(IPC_CANCEL_ACK) = req;
}
