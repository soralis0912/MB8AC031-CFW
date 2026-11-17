/* src/armcore/svc_sys.c -- SVC_SYS (armipc.h).  ECHO and INFO show the
 * queue works; PEEK and POKE are the mailbox's raw access, through the
 * queue.  SUM reads a range and is answered later: it is how fast this
 * core reads memory (the TS ring in DRAM, for one), timed by the host.
 */
#include "core.h"
#include "mb86m01.h"

#define SUM_CHUNK   256u        /* words a pass: about what a pass costs */

#ifdef ARM_HOSTED
unsigned arm_hosted_rd(unsigned addr);
void arm_hosted_wr(unsigned addr, unsigned val);
#define RD(a)       arm_hosted_rd(a)
#define WR(a, v)    arm_hosted_wr((a), (v))
#else
#define RD(a)       (*(volatile unsigned *)(a))
#define WR(a, v)    (*(volatile unsigned *)(a) = (v))
#endif

/* EPTEST: one burst into EP 0x81 from this core (armipc.h).  The status
 * register is read through arm_access first, so a block the ARM cannot
 * reach comes back as ABORT and not as a fault.
 *
 * GAP_READS: a FIFO write straight after a status read loses the first word
 * of the burst, and one stray byte comes out at its end (GV-M2TV, measured:
 * 5 or 6 bursts in 20).  Any other access in between cures it -- one DTCM
 * read gave 20 in 20, and so did 4, 16, 64, 256.  Repeated status reads
 * just before the write made it worse (0 in 20).  Why is not known. */
#define GAP_READS   4u
static void ep_test(const struct req *q, struct rep *r)
{
    unsigned st, i, n = q->arg1 & 0xffffu, spin = (q->arg1 >> 16) & 0x7fffu;
    unsigned dtcm = q->arg1 >> 31;
    unsigned sreg = (q->arg0 & 1u) ? USB_EP1_STATUS : USB_EP0_STATUS;
    unsigned fifo = (q->arg0 & 1u) ? USB_EP1_FIFO : USB_EP0_FIFO;

    if (n == 0u || n > USB_EP_BURST / 4u) {
        r->status = IPC_ST_ARG;
        return;
    }
    if (arm_access(ARM_OP_RD32, sreg, 0, &st) != ARM_ST_OK) {
        r->status = IPC_ST_ABORT;
        return;
    }
    r->val = st;
    if (!(st & USB_EP_READY)) {
        r->status = IPC_ST_BUSY;
        return;
    }
    for (i = 0; i < GAP_READS; i++)
        (void)MB(ARM_MB_LOOPS);
    while (spin--) {                    /* the experiment's delay */
        if (dtcm)
            (void)MB(ARM_MB_LOOPS);
        else
            (void)RD(sreg);
    }
    for (i = 0; i < n; i++)
        WR(fifo, (q->arg0 & 0xffff0000u) | i);
    WR(USB_EP_GUARD, USB_EP_OPEN);
    WR(sreg, USB_EP_COMMIT);
    WR(USB_EP_GUARD, USB_EP_CLOSED);
    r->data[0] = RD(sreg);
    r->data[1] = RD(USB_EP_GUARD);
}

static struct {
    unsigned on, seq, call, addr, left, acc, passes, pending;
    struct rep out;
} sum;

static unsigned op_for(unsigned width, int write)
{
    if (width == 32u)
        return write ? ARM_OP_WR32 : ARM_OP_RD32;
    if (width == 16u)
        return write ? ARM_OP_WR16 : ARM_OP_RD16;
    if (width == 8u)
        return write ? ARM_OP_WR8 : ARM_OP_RD8;
    return ~0u;
}

/* SUM's first and last word through arm_access: a range that aborts is
 * refused before the plain reads below could take the abort. */
static int sum_start(const struct req *q, struct rep *r)
{
    unsigned v;

    if (sum.on || sum.pending) {
        r->status = IPC_ST_BUSY;
        return 1;
    }
    if ((q->arg0 & 3u) || q->arg1 == 0u || q->arg1 > 0x01000000u) {
        r->status = IPC_ST_ARG;
        return 1;
    }
    if (arm_access(ARM_OP_RD32, q->arg0, 0, &v) != ARM_ST_OK
        || arm_access(ARM_OP_RD32, q->arg0 + (q->arg1 - 1u) * 4u, 0, &v)
           != ARM_ST_OK) {
        r->status = IPC_ST_ABORT;
        return 1;
    }
    sum.on = 1;
    sum.seq = q->seq;
    sum.call = q->call;
    sum.addr = q->arg0;
    sum.left = q->arg1;
    sum.acc = 0;
    sum.passes = 0;
    return 0;
}

void sys_tick(void)
{
    unsigned n, i;

    if (sum.on) {
        n = sum.left < SUM_CHUNK ? sum.left : SUM_CHUNK;
        for (i = 0; i < n; i++) {
            sum.acc = (sum.acc << 1 | sum.acc >> 31) ^ RD(sum.addr);
            sum.addr += 4u;
        }
        sum.left -= n;
        sum.passes++;
        if (sum.left == 0u) {
            sum.out.seq = sum.seq;
            sum.out.call = sum.call;
            sum.out.status = IPC_ST_OK;
            sum.out.val = sum.acc;
            sum.out.data[0] = sum.passes;
            for (i = 1; i < IPC_DATA_WORDS; i++)
                sum.out.data[i] = 0;
            sum.on = 0;
            sum.pending = 1;
        }
    }
    if (sum.pending && ipc_put(&sum.out))
        sum.pending = 0;
}

unsigned sys_cancel(unsigned seq)
{
    if (!(sum.on || sum.pending) || (seq != IPC_CANCEL_ALL && seq != sum.seq))
        return 0;
    sum.on = 0;
    sum.pending = 0;
    return 1;
}

int sys_call(const struct req *q, struct rep *r)
{
    unsigned i, op;

    switch (IPC_CALL_OP(q->call)) {
    case SYS_ECHO:
        r->val = ~q->arg0;
        for (i = 0; i < IPC_DATA_WORDS; i++)
            r->data[i] = q->data[i];
        break;
    case SYS_INFO:
        r->val = IPC_VERSION;
        r->data[0] = MB(ARM_MB_LOOPS);
        r->data[1] = arm_cp15(ARM_CP_ID);
        r->data[2] = ipc_served();
        r->data[3] = IPC(IPC_EVENTS);
        break;
    case SYS_PEEK:
    case SYS_POKE:
        op = op_for(q->arg1, IPC_CALL_OP(q->call) == SYS_POKE);
        if (op == ~0u) {
            r->status = IPC_ST_ARG;
            break;
        }
        if (arm_access(op, q->arg0, q->data[0], &r->val) != ARM_ST_OK)
            r->status = IPC_ST_ABORT;
        break;
    case SYS_SUM:
        return sum_start(q, r);
    case SYS_EPTEST:
        ep_test(q, r);
        break;
    default:
        r->status = IPC_ST_BADOP;
        break;
    }
    return 1;
}
