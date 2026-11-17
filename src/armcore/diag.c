/* src/armcore/diag.c -- the mailbox: is the image alive, did it fault, and
 * one raw access at a time.
 *
 * This is the part that was measured first (docs/arm.md): the main FR
 * reads ALIVE, LOOPS and FAULT through its window, and the host uses the
 * REQ_* / ACK_* words for single reads and writes (sample/arm_probe).  It
 * stays separate from the work queue (ipc.c) on purpose: it is what still
 * answers when the queue is the thing that broke.
 */
#include "core.h"

static unsigned seen;

/* One access with the abort handler armed.  -> ARM_ST_*.  A write is
 * checked before its read-back: after an abort PROBING is 0, and a second
 * abort would then be a real fault.  The services use this too, so a watch
 * on an address that does not answer ends with IPC_ST_ABORT. */
unsigned arm_access(unsigned op, unsigned addr, unsigned val, unsigned *out)
{
    unsigned v = 0;

    MB(ARM_MB_PROBING) = 1;
    BARRIER();
    switch (op) {
    case ARM_OP_WR32: *(volatile unsigned *)addr = val; break;
    case ARM_OP_WR16: *(volatile unsigned short *)addr = (unsigned short)val; break;
    case ARM_OP_WR8:  *(volatile unsigned char *)addr = (unsigned char)val; break;
    default: break;
    }
    BARRIER();
    if (MB(ARM_MB_PROBING) == 0)
        return ARM_ST_ABORT;
    switch (op) {
    case ARM_OP_RD32: case ARM_OP_WR32: v = *(volatile unsigned *)addr; break;
    case ARM_OP_RD16: case ARM_OP_WR16: v = *(volatile unsigned short *)addr; break;
    default:                            v = *(volatile unsigned char *)addr; break;
    }
    BARRIER();
    if (MB(ARM_MB_PROBING) == 0)
        return ARM_ST_ABORT;
    MB(ARM_MB_PROBING) = 0;
    *out = v;
    return ARM_ST_OK;
}

void diag_init(void)
{
    MB(ARM_MB_FAULT) = 0;
    MB(ARM_MB_FAULT_LR) = 0;
    MB(ARM_MB_LOOPS) = 0;
    MB(ARM_MB_PROBING) = 0;
    seen = MB(ARM_MB_REQ_SEQ);
    MB(ARM_MB_ACK_SEQ) = seen;
}

/* Answer the mailbox's request, if there is a new one.  Answer before
 * sequence: the main side reads ACK_SEQ first and then trusts ACK_VAL and
 * ACK_ST. */
void diag_poll(void)
{
    unsigned seq = MB(ARM_MB_REQ_SEQ);
    unsigned op, arg, v = 0, st = ARM_ST_OK;

    MB(ARM_MB_LOOPS) = MB(ARM_MB_LOOPS) + 1;
    if (seq == seen)
        return;
    op = MB(ARM_MB_REQ_OP);
    arg = MB(ARM_MB_REQ_ARG);
    if (op == ARM_OP_NOT)
        v = ~arg;
    else if (op == ARM_OP_CP15)
        v = arm_cp15(arg);
    else if (op <= ARM_OP_WR8)
        st = arm_access(op, arg, MB(ARM_MB_REQ_VAL), &v);
    else
        st = ARM_ST_BADOP;
    MB(ARM_MB_ACK_VAL) = v;
    MB(ARM_MB_ACK_ST) = st;
    MB(ARM_MB_ACK_SEQ) = seq;
    seen = seq;
}
