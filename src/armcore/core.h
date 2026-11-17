/* core.h -- what the files of our ARM image share.  Not seen by the main
 * FR: the wire is armcore.h (mailbox) and armipc.h (work queue).
 */
#ifndef CFW_ARMCORE_CORE_H
#define CFW_ARMCORE_CORE_H

#include "armcore.h"
#include "armipc.h"

/* DTCM by the core's own address.  ARM_HOSTED builds the same C on the
 * host (tests/armcore_host.c), with DTCM in an array. */
#ifdef ARM_HOSTED
extern unsigned arm_hosted_dtcm[ARM_DTCM_SIZE / 4u];
#define DTCM_WORD(a) (arm_hosted_dtcm[((a) - ARM_DTCM) / 4u])
#else
#define DTCM_WORD(a) (*(volatile unsigned *)(a))
#endif

#define MB(off)     DTCM_WORD(ARM_MBOX + (off))
#define IPC(off)    DTCM_WORD(ARM_IPC + (off))
#define BARRIER()   __asm__ volatile("" ::: "memory")

/* One request or reply, as the services see it (armipc.h has the slots). */
struct req {
    unsigned seq, call, arg0, arg1;
    unsigned data[IPC_DATA_WORDS];
};

struct rep {
    unsigned seq, call, status, val;
    unsigned data[IPC_DATA_WORDS];
};

/* vectors.S */
unsigned arm_cp15(unsigned which);

/* diag.c -- the mailbox: liveness, faults and raw probes */
void diag_init(void);
void diag_poll(void);
unsigned arm_access(unsigned op, unsigned addr, unsigned val, unsigned *out);

/* ipc.c -- the two rings */
void ipc_init(void);
int ipc_take(struct req *q);
int ipc_put(const struct rep *r);
unsigned ipc_served(void);
void ipc_cancel_poll(void);

/* armmain.c -- start-up, and one pass of the loop */
void arm_start(void);
void arm_pass(void);

/* svc_sys.c -- answered at once, but SUM later (sys_tick) */
int sys_call(const struct req *q, struct rep *r);
void sys_tick(void);
unsigned sys_cancel(unsigned seq);

/* svc_watch.c -- waiting.  watch_call -> 1 answer now, 0 answered later
 * from watch_tick. */
int watch_call(const struct req *q, struct rep *r);
void watch_tick(void);
unsigned watch_cancel(unsigned seq);   /* -> tasks ended; IPC_CANCEL_ALL */

/* svc_card.c -- the card interface.  Same shape as svc_watch.c. */
int card_call(const struct req *q, struct rep *r);
void card_tick(void);
unsigned card_cancel(unsigned seq);

#endif
