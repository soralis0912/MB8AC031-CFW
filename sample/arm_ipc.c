/* arm_ipc.c -- give our ARM image work through the queue, and collect the
 * answers.  The host side of ARM_CALL / ARM_REAP (../src/shared/armipc.h);
 * the design is ../src/armcore/README.md.
 *
 *     ./arm_ipc info                         the image's version and counters
 *     ./arm_ipc echo <x>                     ~x back, through the queue
 *     ./arm_ipc peek <addr> [--w 32|16|8]    the ARM reads
 *     ./arm_ipc poke <addr> <val> [--w ..]   the ARM writes and reads back
 *     ./arm_ipc wait <addr> <mask> <value> [--timeout passes] [--for ms]
 *                                            the ARM waits for a condition
 *     ./arm_ipc watch <addr> [--mask m] [--for ms]
 *                                            the ARM reports every change
 *     ./arm_ipc demo                         a wait, an echo and an info at
 *                                            once: the quick ones come back
 *                                            first
 *     ./arm_ipc cancel <seq|all>             stop a task around the rings
 *     ./arm_ipc jam                          fill both rings, show STOP
 *                                            cannot get in, get out with a
 *                                            cancel
 *
 * --core 0|1 picks ARM-E (default) or ARM-C.  Addresses are the ARM's own.
 *
 * The image has to be running first; arm_probe puts it there:
 *
 *     ./arm_probe hold
 *     ./arm_probe image ../build/armcore.bin
 *     ./arm_probe run
 *
 * Every request is one ARM_CALL, which returns at once with the SEQ the
 * reply will carry.  Replies are collected with ARM_REAP, which also returns
 * at once (E_AGAIN when nothing is there yet).  Nothing here waits inside the
 * chip: the waiting is the ARM's (the point of the design), and this program
 * only polls.  --dev m2tv (default) or dt295, like arm_probe.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "chipio.h"
#include "frq1.h"
#include "cfwdev.h"
#include "armipc.h"

/* The machine (--dev, default m2tv).  The ARM tools have mailbox 2 of 4
 * (cfwdev.h), so ts_grab (mailbox 0) can run at once. */
static const struct cfwdev *g_dev;
#define STAGING     (g_dev->arm_box)
#define HOOK        (g_dev->hook)
#define POLL_MS     20u

struct link {
    struct chipio io;
    uint32_t seq;
};

static unsigned g_core;

static int call(struct link *l, uint32_t cmd, uint32_t arg0, uint32_t arg1,
                const uint8_t *data, size_t ndata, uint8_t reply[64],
                int32_t *status)
{
    uint8_t request[64];
    const char *why;

    l->seq++;
    frq1_build(request, l->seq, cmd, arg0, arg1, data, ndata);
    if (chipio_dispatch(&l->io, STAGING, request, reply) != 0) {
        fprintf(stderr, "%s: %s\n", frq1_command_name(cmd), l->io.err);
        return -1;
    }
    why = frq1_reply_check(reply, l->seq, cmd);
    if (why != NULL) {
        fprintf(stderr, "%s: %s\n", frq1_command_name(cmd), why);
        return -1;
    }
    *status = frq1_get32s(reply, FRQ1_FR_STATUS);
    return 0;
}

/* --- the two commands -------------------------------------------------- */

/* Queue one request.  -> the status (FRQ1_E_AGAIN: the ring is full),
 * *seq set on FRQ1_ST_OK; -1000 if the frame itself failed. */
static int32_t ipc_try(struct link *l, unsigned svc, unsigned op,
                       uint32_t arg0, uint32_t arg1, const uint32_t d[4],
                       uint32_t *seq)
{
    uint8_t data[20], r[64];
    int32_t st;
    unsigned i;

    frq1_put32(data, 0, arg1);
    for (i = 0; i < 4; i++)
        frq1_put32(data, 4u + i * 4u, d ? d[i] : 0u);
    if (call(l, FRQ1_CMD_ARM_CALL, g_core | (IPC_CALL(svc, op) << 8), arg0,
             data, sizeof data, r, &st) != 0)
        return -1000;
    if (st == FRQ1_ST_OK)
        *seq = frq1_get32(r + FRQ1_FR_DATA, 4);
    return st;
}

/* Queue one request.  -> its SEQ, or 0 on failure. */
static uint32_t ipc_call(struct link *l, unsigned svc, unsigned op,
                         uint32_t arg0, uint32_t arg1, const uint32_t d[4])
{
    uint32_t seq = 0;
    int32_t st = ipc_try(l, svc, op, arg0, arg1, d, &seq);

    if (st == -1000)
        return 0;
    if (st != FRQ1_ST_OK) {
        fprintf(stderr, "ARM_CALL: %s%s\n", frq1_status_name(st),
                st == FRQ1_E_IO ? " -- is the image running? (arm_probe run)"
                                : "");
        return 0;
    }
    return seq;
}

struct reply {
    uint32_t seq, call, status, val, data[4];
};

/* Take one reply.  -> 1 got one, 0 none yet, -1 failure. */
static int ipc_reap(struct link *l, struct reply *out)
{
    uint8_t r[64];
    const uint8_t *d = r + FRQ1_FR_DATA;
    int32_t st;
    unsigned i;

    if (call(l, FRQ1_CMD_ARM_REAP, g_core, 0, NULL, 0, r, &st) != 0)
        return -1;
    if (st == FRQ1_E_AGAIN)
        return 0;
    if (st != FRQ1_ST_OK) {
        fprintf(stderr, "ARM_REAP: %s\n", frq1_status_name(st));
        return -1;
    }
    out->seq = frq1_get32(d, 4);
    out->call = frq1_get32(d, 8);
    out->status = frq1_get32(d, 12);
    out->val = frq1_get32(d, 16);
    for (i = 0; i < 4; i++)
        out->data[i] = frq1_get32(d, 20u + i * 4u);
    return 1;
}

struct cancel {
    uint32_t req, ack, done, dropped, reqs, reps;
};

static int cancel_frame(struct link *l, uint32_t what, struct cancel *c,
                        int32_t *st)
{
    uint8_t r[64];
    const uint8_t *d = r + FRQ1_FR_DATA;

    if (call(l, FRQ1_CMD_ARM_CANCEL, g_core, what, NULL, 0, r, st) != 0)
        return -1;
    if (*st == FRQ1_ST_OK) {
        c->req = frq1_get32(d, 4);
        c->ack = frq1_get32(d, 8);
        c->done = frq1_get32(d, 12);
        c->dropped = frq1_get32(d, 16);
        c->reqs = frq1_get32(d, 20);
        c->reps = frq1_get32(d, 24);
    }
    return 0;
}

/* Ask for a cancel and wait (here, on the host) until the ARM has handled
 * it.  -> 0 handled, 1 not. */
static int ipc_cancel(struct link *l, uint32_t what, struct cancel *c)
{
    int32_t st;
    unsigned t;

    for (t = 0; t <= 1000; t += POLL_MS) {
        if (cancel_frame(l, what, c, &st) != 0)
            return 1;
        if (st == FRQ1_ST_OK)
            break;
        if (st != FRQ1_E_AGAIN) {
            fprintf(stderr, "ARM_CANCEL: %s\n", frq1_status_name(st));
            return 1;
        }
        chipio_nap(POLL_MS);            /* the last one is still pending */
    }
    for (t = 0; t <= 1000; t += POLL_MS) {
        uint32_t req = c->req;

        if (c->ack == req) {
            printf("  cancel %u handled: %u task(s) ended, %u request(s) "
                   "dropped; now %u request(s), %u reply(s) waiting\n",
                   (unsigned)req, (unsigned)c->done, (unsigned)c->dropped,
                   (unsigned)c->reqs, (unsigned)c->reps);
            return 0;
        }
        chipio_nap(POLL_MS);
        if (cancel_frame(l, 0, c, &st) != 0 || st != FRQ1_ST_OK)
            return 1;
        c->req = req;
    }
    printf("  cancel %u: not handled in 1 s\n", (unsigned)c->req);
    return 1;
}

/* --- printing ---------------------------------------------------------- */

static const char *call_name(uint32_t c)
{
    switch (c & ~IPC_EVENT) {
    case IPC_CALL(SVC_SYS, SYS_ECHO):       return "SYS.ECHO";
    case IPC_CALL(SVC_SYS, SYS_INFO):       return "SYS.INFO";
    case IPC_CALL(SVC_SYS, SYS_PEEK):       return "SYS.PEEK";
    case IPC_CALL(SVC_SYS, SYS_POKE):       return "SYS.POKE";
    case IPC_CALL(SVC_SYS, SYS_SUM):        return "SYS.SUM";
    case IPC_CALL(SVC_SYS, SYS_EPTEST):     return "SYS.EPTEST";
    case IPC_CALL(SVC_WATCH, WATCH_WAIT):   return "WATCH.WAIT";
    case IPC_CALL(SVC_WATCH, WATCH_START):  return "WATCH.START";
    case IPC_CALL(SVC_WATCH, WATCH_STOP):   return "WATCH.STOP";
    case IPC_CALL(SVC_CARD, CARDSVC_STATUS): return "CARD.STATUS";
    case IPC_CALL(SVC_CARD, CARDSVC_ON):    return "CARD.ON";
    case IPC_CALL(SVC_CARD, CARDSVC_OFF):   return "CARD.OFF";
    case IPC_CALL(SVC_CARD, CARDSVC_PARAM): return "CARD.PARAM";
    case IPC_CALL(SVC_CARD, CARDSVC_XFER):  return "CARD.XFER";
    default:                                return "?";
    }
}

static const char *st_name(uint32_t s)
{
    static const char *n[] = { "OK", "BADSVC", "BADOP", "ABORT", "BUSY",
                               "TIMEOUT", "ARG", "STOPPED", "IO",
                               "STATE" };

    return s < sizeof n / sizeof n[0] ? n[s] : "?";
}

static void show(const struct reply *p, unsigned ms)
{
    printf("  %5u ms  seq %-3u %-5s %-11s %-7s val %08x  data %08x %08x "
           "%08x %08x\n", ms, (unsigned)p->seq,
           p->call & IPC_EVENT ? "event" : "reply", call_name(p->call),
           st_name(p->status), (unsigned)p->val, (unsigned)p->data[0],
           (unsigned)p->data[1], (unsigned)p->data[2], (unsigned)p->data[3]);
}

/* Print replies until the one for `seq` that is not an event (seq 0: none
 * in particular), or until `ms` have passed.  -> 0 it came, 1 not. */
static int collect(struct link *l, uint32_t seq, unsigned ms, struct reply *got)
{
    unsigned t = 0;
    struct reply p;
    int k;

    for (;;) {
        while ((k = ipc_reap(l, &p)) == 1) {
            show(&p, t);
            if (seq && p.seq == seq && !(p.call & IPC_EVENT)) {
                if (got)
                    *got = p;
                return 0;
            }
        }
        if (k < 0 || t >= ms)
            return 1;
        chipio_nap(POLL_MS);
        t += POLL_MS;
    }
}

static int ask(struct link *l, unsigned svc, unsigned op, uint32_t a0,
               uint32_t a1, const uint32_t d[4], unsigned ms)
{
    struct reply p;
    uint32_t seq = ipc_call(l, svc, op, a0, a1, d);

    if (seq == 0)
        return 1;
    printf("  queued  seq %u  %s\n", (unsigned)seq, call_name(IPC_CALL(svc, op)));
    if (collect(l, seq, ms, &p) != 0) {
        printf("  no reply for seq %u in %u ms\n", (unsigned)seq, ms);
        return 1;
    }
    return p.status == IPC_ST_OK ? 0 : 1;
}

/* --- commands ---------------------------------------------------------- */

static int cmd_watch(struct link *l, uint32_t addr, uint32_t mask, unsigned ms)
{
    uint32_t d[4] = { mask, 0, 0, 0 };
    uint32_t seq = ipc_call(l, SVC_WATCH, WATCH_START, addr, 0, d);
    struct reply p;
    uint32_t n;

    if (seq == 0)
        return 1;
    if (collect(l, seq, 1000, &p) != 0 || p.status != IPC_ST_OK)
        return 1;
    n = p.val;
    printf("  watch %u on %08x & %08x; events for %u ms:\n", (unsigned)n,
           (unsigned)addr, (unsigned)mask, ms);
    collect(l, 0, ms, NULL);
    seq = ipc_call(l, SVC_WATCH, WATCH_STOP, n, 0, NULL);
    if (seq == 0)
        return 1;
    /* STOP's own reply, then the watch's STOPPED event. */
    collect(l, seq, 1000, NULL);
    collect(l, 0, 200, NULL);
    return 0;
}

/* Three requests in a row, then collect.  The WAIT is on a word that does
 * not change, so it ends by its timeout -- after the ECHO and INFO queued
 * behind it have been answered.  The main FR never waited for any of it. */
static int cmd_demo(struct link *l)
{
    uint32_t d[4] = { 0xffffffffu, 0x12345678u, 0, 0 };
    uint32_t s1, s2, s3;

    /* The mailbox's ALIVE word: 'ARM1' while the image runs, never the
     * value asked for. */
    s1 = ipc_call(l, SVC_WATCH, WATCH_WAIT, ARM_MBOX + ARM_MB_ALIVE,
                  2000000u, d);
    s2 = ipc_call(l, SVC_SYS, SYS_ECHO, 0x00ff00ffu, 0, NULL);
    s3 = ipc_call(l, SVC_SYS, SYS_INFO, 0, 0, NULL);
    if (!s1 || !s2 || !s3)
        return 1;
    printf("  queued  seq %u WAIT (times out), %u ECHO, %u INFO\n",
           (unsigned)s1, (unsigned)s2, (unsigned)s3);
    return collect(l, s1, 10000, NULL);
}

/* The hole the cancel words are for, on the real chip.  A watch that keeps
 * sending events, then requests until ARM_CALL says the ring is full,
 * without reading a reply: the reply ring fills, the ARM stops taking, the
 * request ring fills.  A STOP cannot get in.  A cancel can. */
static int cmd_jam(struct link *l)
{
    uint32_t d[4] = { 0x00100000u, 0, 0, 0 };   /* LOOPS bit20: ~0.6 s */
    uint32_t seq = 0, wseq;
    struct reply p;
    struct cancel c;
    unsigned n = 0, events = 0, k;
    int32_t st;

    wseq = ipc_call(l, SVC_WATCH, WATCH_START, ARM_MBOX + ARM_MB_LOOPS, 0, d);
    if (wseq == 0 || collect(l, wseq, 1000, &p) != 0)
        return 1;
    printf("  watch %u started (seq %u)\n", (unsigned)p.val, (unsigned)wseq);
    while ((st = ipc_try(l, SVC_SYS, SYS_ECHO, n, 0, NULL, &seq)) == FRQ1_ST_OK)
        n++;
    if (st != FRQ1_E_AGAIN)
        return 1;
    printf("  %u ECHOs queued, then ARM_CALL: %s\n", n, frq1_status_name(st));
    chipio_nap(100);
    st = ipc_try(l, SVC_WATCH, WATCH_STOP, 0, 0, NULL, &seq);
    printf("  STOP through the ring: %s\n", frq1_status_name(st));
    if (ipc_cancel(l, IPC_CANCEL_ALL, &c) != 0)
        return 1;
    for (k = 0; ipc_reap(l, &p) == 1; k++)
        if (p.call & IPC_EVENT)
            events++;
    printf("  reaped %u reply(s) that were already out (%u event(s))\n", k,
           events);
    chipio_nap(1500);
    for (k = 0; ipc_reap(l, &p) == 1; k++)
        ;
    printf("  1.5 s later: %u more reply(s) -- the watch is gone\n", k);
    return (c.done == 1 && k == 0 && ask(l, SVC_SYS, SYS_INFO, 0, 0, NULL,
                                         1000) == 0) ? 0 : 1;
}

static double now_ms(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

/* SYS.SUM over [addr, addr + 4 * words), timed from here: the ARM's read
 * speed, frame latency included (a few ms; use a big range). */
static int cmd_sum(struct link *l, uint32_t addr, uint32_t words)
{
    struct reply p;
    uint32_t seq;
    double t0, t;
    int k;

    t0 = now_ms();
    seq = ipc_call(l, SVC_SYS, SYS_SUM, addr, words, NULL);
    if (seq == 0)
        return 1;
    for (;;) {
        k = ipc_reap(l, &p);
        if (k < 0)
            return 1;
        if (k == 1 && p.seq == seq)
            break;
        if (now_ms() - t0 > 30000.0) {
            printf("  no reply in 30 s\n");
            return 1;
        }
        chipio_nap(1);
    }
    t = now_ms() - t0;
    show(&p, (unsigned)t);
    if (p.status != IPC_ST_OK)
        return 1;
    printf("  %u words (%.1f MiB) in %.1f ms: %.1f MB/s, %u passes\n",
           (unsigned)words, words * 4.0 / 1048576.0, t,
           words * 4.0 / (t * 1000.0), (unsigned)p.data[0]);
    return 0;
}

/* SYS.EPTEST: the ARM writes one burst into EP 0x81, and this reads it.
 * Nothing may be streaming (ts_grab).  What is already in the endpoint is
 * drained first, so a leftover is not taken for ours. */
static unsigned g_spin, g_quiet, g_ep = 0x81;

static int cmd_epburst(struct link *l, unsigned words)
{
    uint8_t buf[2048];
    struct reply p;
    uint32_t seq, seed = 0xA5C30000u;
    int got = 0, rc, k;
    unsigned i, bad = 0, drained = 0;

    while ((rc = chipio_bulk(&l->io, g_ep, buf, sizeof buf, 50, &got)) == 0
           && got > 0)
        drained += (unsigned)got;
    if (rc < 0) {
        fprintf(stderr, "%s\n", l->io.err);
        return 1;
    }
    printf("  drained %u byte(s) left in ep %#x\n", drained, g_ep);
    seq = ipc_call(l, SVC_SYS, SYS_EPTEST, seed | (g_ep == 0x84 ? 1u : 0u),
                   words | g_spin << 16, NULL);
    if (seq == 0)
        return 1;
    if (g_quiet)                        /* no EP0 traffic while it writes */
        chipio_nap(g_quiet);
    for (k = 0; k < 500; k++) {
        int r = ipc_reap(l, &p);

        if (r < 0)
            return 1;
        if (r == 1 && p.seq == seq)
            break;
        chipio_nap(2);
    }
    if (k == 500) {
        printf("  no reply\n");
        return 1;
    }
    show(&p, 0);
    printf("  status before %08x, after %08x, guard after %08x\n",
           (unsigned)p.val, (unsigned)p.data[0], (unsigned)p.data[1]);
    if (p.status != IPC_ST_OK)
        return 1;
    rc = chipio_bulk(&l->io, g_ep, buf, sizeof buf, 1000, &got);
    if (rc < 0) {
        fprintf(stderr, "%s\n", l->io.err);
        return 1;
    }
    printf("  ep %#x gave %d byte(s) (want %u):", g_ep, got, words * 4u);
    for (i = 0; i < 12u && (int)i < got; i++)
        printf(" %02x", buf[i]);
    printf(" ..");
    for (i = got > 8 ? (unsigned)got - 8u : 0u; (int)i < got; i++)
        printf(" %02x", buf[i]);
    printf("\n");
    for (i = 0; i < words && (int)(i * 4u + 3u) < got; i++)
        if (frq1_get32(buf, i * 4u) != (seed | i))
            bad++;
    if (got > 0)
        printf("  first words %08x %08x %08x ... last %08x\n",
               (unsigned)frq1_get32(buf, 0), (unsigned)frq1_get32(buf, 4),
               (unsigned)frq1_get32(buf, 8),
               got >= 4 ? (unsigned)frq1_get32(buf, (unsigned)(got - 4) & ~3u)
                        : 0u);
    printf("  %s: %u of %u word(s) differ\n",
           got == (int)(words * 4u) && bad == 0 ? "MATCH" : "NO MATCH", bad,
           words);
    return got == (int)(words * 4u) && bad == 0 ? 0 : 1;
}

static void usage(const char *me)
{
    fprintf(stderr,
        "usage: %s <command> [--core 0|1] [--dev m2tv|dt295]   0 = ARM-E (default), 1 = ARM-C\n"
        "       %s info\n"
        "       %s echo <x>\n"
        "       %s peek <addr> [--w 32|16|8]\n"
        "       %s poke <addr> <val> [--w 32|16|8]\n"
        "       %s wait <addr> <mask> <value> [--timeout passes] [--for ms]\n"
        "       %s watch <addr> [--mask m] [--for ms]\n"
        "       %s demo\n"
        "       %s cancel <seq|all>\n"
        "       %s jam\n"
        "       %s sum <addr> <words>          ARM read speed\n"
        "       %s epburst [words] [--ep 0x84] ARM writes one burst to ep 0x81/0x84\n"
        "The ARM image has to be running: arm_probe hold, image, run.\n",
        me, me, me, me, me, me, me, me, me, me, me, me);
}

int main(int argc, char **argv)
{
    struct link l;
    const char *cmd, *pos[4];
    unsigned npos = 0, w = 32, ms = 3000;
    uint32_t mask = 0xffffffffu, timeout = 0;
    uint8_t stub[8];
    int i, rc;

    if (argc < 2) {
        usage(argv[0]);
        return 2;
    }
    cmd = argv[1];
    for (i = 2; i < argc; i++) {
        if (strcmp(argv[i], "--dev") == 0 && i + 1 < argc) {
            g_dev = cfwdev_find(argv[++i]);
            if (!g_dev) {
                fprintf(stderr, "--dev: m2tv or dt295\n");
                return 2;
            }
            continue;
        }
        if (strcmp(argv[i], "--core") == 0 && i + 1 < argc)
            g_core = (unsigned)strtoul(argv[++i], NULL, 0);
        else if (strcmp(argv[i], "--w") == 0 && i + 1 < argc)
            w = (unsigned)strtoul(argv[++i], NULL, 0);
        else if (strcmp(argv[i], "--mask") == 0 && i + 1 < argc)
            mask = (uint32_t)strtoul(argv[++i], NULL, 0);
        else if (strcmp(argv[i], "--timeout") == 0 && i + 1 < argc)
            timeout = (uint32_t)strtoul(argv[++i], NULL, 0);
        else if (strcmp(argv[i], "--quiet") == 0 && i + 1 < argc)
            g_quiet = (unsigned)strtoul(argv[++i], NULL, 0);
        else if (strcmp(argv[i], "--ep") == 0 && i + 1 < argc)
            g_ep = (unsigned)strtoul(argv[++i], NULL, 0) == 0x84 ? 0x84u : 0x81u;
        else if (strcmp(argv[i], "--spin") == 0 && i + 1 < argc)
            g_spin = (unsigned)strtoul(argv[++i], NULL, 0);
        else if (strcmp(argv[i], "--for") == 0 && i + 1 < argc)
            ms = (unsigned)strtoul(argv[++i], NULL, 0);
        else if (argv[i][0] != '-' && npos < 4)
            pos[npos++] = argv[i];
        else {
            usage(argv[0]);
            return 2;
        }
    }
    if (!(strcmp(cmd, "info") == 0 || strcmp(cmd, "demo") == 0 ||
          strcmp(cmd, "jam") == 0 || (strcmp(cmd, "cancel") == 0 && npos == 1) ||
          (strcmp(cmd, "echo") == 0 && npos == 1) ||
          (strcmp(cmd, "peek") == 0 && npos == 1) ||
          (strcmp(cmd, "poke") == 0 && npos == 2) ||
          (strcmp(cmd, "wait") == 0 && npos == 3) ||
          (strcmp(cmd, "watch") == 0 && npos == 1) ||
          (strcmp(cmd, "sum") == 0 && npos == 2) ||
          (strcmp(cmd, "epburst") == 0 && npos <= 1)) ||
        (w != 32 && w != 16 && w != 8) || g_core > 1) {
        usage(argv[0]);
        return 2;
    }

    memset(&l, 0, sizeof l);
    if (!g_dev)
        g_dev = cfwdev_find("m2tv");
    if (chipio_open(&l.io, g_dev->vid, g_dev->pid) != 0) {
        fprintf(stderr, "%s\n", l.io.err);
        return 1;
    }
    /* Read before write: is a firmware of ours behind the hook? */
    if (chipio_read(&l.io, HOOK, stub, 8u) != 0 || stub[0] != 0x9b ||
        stub[4] != 0x97 || stub[5] != 0x0c) {
        fprintf(stderr, "no firmware of ours at the hook 0x%08x; nothing "
                "sent.\n  ./frc1_load install ../build/frc1-%s.frc\n",
                HOOK, g_dev->name);
        chipio_close(&l.io);
        return 1;
    }

    if (strcmp(cmd, "info") == 0) {
        rc = ask(&l, SVC_SYS, SYS_INFO, 0, 0, NULL, 1000);
    } else if (strcmp(cmd, "echo") == 0) {
        uint32_t d[4] = { 1, 2, 3, 4 };

        rc = ask(&l, SVC_SYS, SYS_ECHO, (uint32_t)strtoul(pos[0], NULL, 0), 0,
                 d, 1000);
    } else if (strcmp(cmd, "peek") == 0) {
        rc = ask(&l, SVC_SYS, SYS_PEEK, (uint32_t)strtoul(pos[0], NULL, 0), w,
                 NULL, 1000);
    } else if (strcmp(cmd, "poke") == 0) {
        uint32_t d[4] = { (uint32_t)strtoul(pos[1], NULL, 0), 0, 0, 0 };

        rc = ask(&l, SVC_SYS, SYS_POKE, (uint32_t)strtoul(pos[0], NULL, 0), w,
                 d, 1000);
    } else if (strcmp(cmd, "wait") == 0) {
        uint32_t d[4] = { (uint32_t)strtoul(pos[1], NULL, 0),
                          (uint32_t)strtoul(pos[2], NULL, 0), 0, 0 };

        rc = ask(&l, SVC_WATCH, WATCH_WAIT, (uint32_t)strtoul(pos[0], NULL, 0),
                 timeout, d, ms);
    } else if (strcmp(cmd, "watch") == 0) {
        rc = cmd_watch(&l, (uint32_t)strtoul(pos[0], NULL, 0), mask, ms);
    } else if (strcmp(cmd, "cancel") == 0) {
        struct cancel c;

        rc = ipc_cancel(&l, strcmp(pos[0], "all") == 0 ? IPC_CANCEL_ALL
                        : (uint32_t)strtoul(pos[0], NULL, 0), &c);
    } else if (strcmp(cmd, "sum") == 0) {
        rc = cmd_sum(&l, (uint32_t)strtoul(pos[0], NULL, 0),
                     (uint32_t)strtoul(pos[1], NULL, 0));
    } else if (strcmp(cmd, "epburst") == 0) {
        rc = cmd_epburst(&l, npos ? (unsigned)strtoul(pos[0], NULL, 0) : 128u);
    } else if (strcmp(cmd, "jam") == 0) {
        rc = cmd_jam(&l);
    } else {
        rc = cmd_demo(&l);
    }
    chipio_close(&l.io);
    return rc;
}
