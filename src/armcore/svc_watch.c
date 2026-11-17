/* src/armcore/svc_watch.c -- SVC_WATCH: waiting, which is what the ARM is
 * here for (README.md).
 *
 * The main FR has no loop of its own: it runs only while the host's frame
 * is being answered, so anything that waits has to be cut into steps the
 * host drives.  This service waits here instead.  WAIT answers once a
 * condition holds; START answers at once and then sends an EVENT each time
 * a value changes.  The host collects both with ARM_REAP whenever it likes.
 *
 * Time is counted in passes of the main loop (armmain.c), not in seconds:
 * no timer on this core is known yet.
 *
 * A reply that finds the reply ring full is kept and sent on a later pass.
 * While a watch holds such a reply it does not sample, so changes that
 * happen meanwhile arrive as one.
 */
#include "core.h"

#define K_FREE      0u
#define K_WAIT      1u
#define K_START     2u

struct watch {
    unsigned kind;
    unsigned seq, call;
    unsigned addr, mask, value;
    unsigned left;              /* WAIT: passes left, 0 = no timeout */
    unsigned last;              /* START: the last value reported    */
    unsigned pending;           /* 1 = `out` is waiting for room     */
    unsigned final;             /* 1 = free the watch once sent      */
    struct rep out;
};

static struct watch w[WATCH_MAX];

static int find_free(void)
{
    unsigned i;

    for (i = 0; i < WATCH_MAX; i++)
        if (w[i].kind == K_FREE)
            return (int)i;
    return -1;
}

static void queue(struct watch *x, unsigned status, unsigned val,
                  unsigned d0, unsigned event, unsigned final)
{
    unsigned i;

    x->out.seq = x->seq;
    x->out.call = x->call | (event ? IPC_EVENT : 0u);
    x->out.status = status;
    x->out.val = val;
    x->out.data[0] = d0;
    for (i = 1; i < IPC_DATA_WORDS; i++)
        x->out.data[i] = 0;
    x->pending = 1;
    x->final = final;
}

int watch_call(const struct req *q, struct rep *r)
{
    unsigned v, n;
    int i;

    switch (IPC_CALL_OP(q->call)) {
    case WATCH_WAIT:
    case WATCH_START:
        i = find_free();
        if (i < 0) {
            r->status = IPC_ST_BUSY;
            return 1;
        }
        if (arm_access(ARM_OP_RD32, q->arg0, 0, &v) != ARM_ST_OK) {
            r->status = IPC_ST_ABORT;
            return 1;
        }
        w[i].seq = q->seq;
        w[i].call = q->call;
        w[i].addr = q->arg0;
        w[i].mask = q->data[0];
        w[i].value = q->data[1];
        w[i].left = q->arg1;
        w[i].last = v & q->data[0];
        w[i].pending = 0;
        if (IPC_CALL_OP(q->call) == WATCH_WAIT) {
            w[i].kind = K_WAIT;
            return 0;           /* watch_tick answers */
        }
        w[i].kind = K_START;
        r->val = (unsigned)i;
        r->data[0] = w[i].last;
        return 1;
    case WATCH_STOP:
        n = q->arg0;
        if (n >= WATCH_MAX || w[n].kind != K_START) {
            r->status = IPC_ST_ARG;
            return 1;
        }
        /* Replaces an event still waiting for room: STOPPED says the
         * watch is over, and its value is the last one seen. */
        queue(&w[n], IPC_ST_STOPPED, w[n].last, 0, 1, 1);
        return 1;
    default:
        r->status = IPC_ST_BADOP;
        return 1;
    }
}

static void step(struct watch *x)
{
    unsigned v, m;

    if (arm_access(ARM_OP_RD32, x->addr, 0, &v) != ARM_ST_OK) {
        queue(x, IPC_ST_ABORT, 0, 0, x->kind == K_START, 1);
        return;
    }
    m = v & x->mask;
    if (x->kind == K_WAIT) {
        if (m == x->value)
            queue(x, IPC_ST_OK, v, 0, 0, 1);
        else if (x->left && --x->left == 0)
            queue(x, IPC_ST_TIMEOUT, v, 0, 0, 1);
        return;
    }
    if (m != x->last) {
        queue(x, IPC_ST_OK, m, x->last, 1, 0);
        x->last = m;
    }
}

/* End the task a request started, or all of them, with no reply: the
 * cancel words (armipc.h) are for when replies cannot get out.  A reply
 * still waiting for room goes with it. */
unsigned watch_cancel(unsigned seq)
{
    unsigned i, n = 0;

    for (i = 0; i < WATCH_MAX; i++) {
        if (w[i].kind == K_FREE)
            continue;
        if (seq == IPC_CANCEL_ALL || w[i].seq == seq) {
            w[i].kind = K_FREE;
            w[i].pending = 0;
            n++;
        }
    }
    return n;
}

void watch_tick(void)
{
    unsigned i;

    for (i = 0; i < WATCH_MAX; i++) {
        struct watch *x = &w[i];

        if (x->kind == K_FREE)
            continue;
        if (!x->pending)
            step(x);
        if (x->pending && ipc_put(&x->out)) {
            x->pending = 0;
            if (x->final)
                x->kind = K_FREE;
        }
    }
}
