/* ts_grab.c -- getting a transport stream out, as a worked example.
 *
 * The other half of ../sample/: frc1_load.c puts the firmware on the chip,
 * this talks to it.  Two things, which is all a recorder really needs:
 *
 *     ./ts_grab probe  [--dev m2tv]              is our firmware answering?
 *     ./ts_grab save   out.ts [--seconds 10]
 *     ./ts_grab stop                             STOP, and drop the capture
 *
 * TUNING IS THE HOST'S JOB.  The firmware has no tune command: the host
 * writes the tuner and the demodulator itself with I2CXFER (one transfer a
 * frame) and polls STATUS for the lock (../docs/frq1.md).  This program
 * does that on the GV-M2TV (dione) and the PIX-DT295 with their own
 * measured sequences (fe_m2tv.h, fe_dt295.h).  On other selene machines,
 * tune with your own sequence first and then `save`.
 *
 * On the PIX-DT295 (--dev dt295), no table file:
 *
 *     ./ts_grab tune 27 --dev dt295          also BS1/TS1, BS15/0x40F1, CS2
 *     ./ts_grab scan --dev dt295             UHF 13..50; --sat for BS / CS
 *     ./ts_grab save out.ts --ch BS1 --dev dt295
 *
 * On dione (--dev m2tv) `save` takes a different path, cmd_save_dione()
 * below: a reader thread and big frames:
 *
 *     ./ts_grab tune 27 --dev m2tv           UHF 13..50 (below 700 MHz)
 *     ./ts_grab scan --dev m2tv
 *     ./ts_grab save out.ts --ch 27 --dev m2tv
 *
 * THE WIRE IS TWO THINGS AT ONCE.  Commands go out as a 64-byte `FRQ1`
 * frame through the staging dispatch on vendor request 0xBC (frq1.h for the
 * frame, chipio.h for the window, ../docs/frq1.md for the meaning).  The
 * stream itself does NOT come back that way: the firmware pushes 512-byte
 * bursts into an endpoint FIFO (../docs/ts.md) and they arrive on a bulk IN
 * endpoint.  So `save` alternates -- one STREAM frame, then drain the
 * endpoint -- and that shape is forced by the firmware's own:
 *
 *   there is no main loop on the chip.  On the FRC1 path the firmware runs
 *   inside the resident loader's handling of one control transfer, and while
 *   it is in there no other control transfer reaches the device.  A request
 *   cannot mean "stream until I say stop"; it means "move what you can now
 *   and tell me what you did", and the host comes back.  The host sets the
 *   pace with arg0, the burst count (../src/main/stream.c).
 *
 * A REAL RECORDER WOULD KEEP READS OUTSTANDING.  This one does a synchronous
 * STREAM and then a synchronous drain, so between the two there is a window
 * where the endpoint is not being read and a burst finds it busy -- the
 * firmware answers FW_EAGAIN and sets SR_F_DRAINED, which is reported rather
 * than hidden.  libusb_submit_transfer with a few buffers in flight is the
 * fix and it is deliberately not here: it would double the size of the file
 * and bury the part worth reading.
 *
 */
/* clock_gettime and pthreads, also in the build without libusb. */
#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif
#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "chipio.h"
#include "frq1.h"
#include "fe_dt295.h"
#include "fe_m2tv.h"

#define STAGING_DEFAULT 0x000C4620u     /* the selene loader's buffer      */
#define HOOK_DEFAULT    0x000EA6FCu     /* selene -- see entry_frc1_selene.S       */
#define EP_DEFAULT      0x81u           /* the first bulk IN               */

/* dione STREAM's data byte 0: the bulk IN endpoints the firmware fills
 * (bit0 EP 0x81, bit1 EP 0x84; 0 = EP 0x81).  main() sets it from --ep and
 * --mirror. */
static uint8_t g_outs;

/* `save -` writes the TS to standard output (for piping into a recorder).
 * main() moves the original stdout here and points fd 1 at stderr, so the
 * progress lines cannot mix with the TS. */
static int g_ts_fd = -1;

/* SIGINT / SIGTERM during a dione `save`: stop at the next frame, send
 * STOP, and finish the summary.  A recorder ends a tuner command this way. */
static volatile sig_atomic_t g_quit;

static void on_quit(int sig)
{
    (void)sig;
    g_quit = 1;
}

static FILE *open_ts(const char *path)
{
    if (strcmp(path, "-") == 0)
        return g_ts_fd >= 0 ? fdopen(g_ts_fd, "wb") : NULL;
    return fopen(path, "wb");
}

#define BURSTS_DEFAULT  32u
#define DRAIN_MS        200u

/* --- is our firmware even there? ---------------------------------------
 *
 * EVERY COMMAND BELOW STARTS WITH A WRITE: the staging dispatch is an OUT
 * transfer, 64 bytes to the staging address.  On a machine running our
 * firmware that buffer belongs to it and the write is the request.  On any
 * other machine it is 64 bytes into an address chosen for a different
 * loader -- measured once on a GV-M2TV (MB8AC018), where
 * 0x000C4620 read back as something other than what went in.
 *
 * So the first thing is a read, not a write.  frc1_load puts
 * `ldi:20 #entry,r12 ; jmp @r12 ; nop` at the hook, which is eight bytes
 * with six fixed positions, and those six are what this looks for.  If they
 * are not there, no firmware of ours is in the path and there is nothing to
 * talk to -- so nothing is sent.
 */
static int hook_installed(struct chipio *io, uint32_t hook, uint32_t *entry,
                          uint8_t out[8])
{
    if (chipio_read(io, hook, out, 8u) != 0)
        return -1;
    *entry = (uint32_t)((out[1] >> 4) & 0x0fu) << 16 |
             (uint32_t)out[2] << 8 | out[3];
    return (out[0] == 0x9b && (out[1] & 0x0fu) == 12u &&   /* ldi:20 ,r12 */
            out[4] == 0x97 && out[5] == 0x0c &&            /* jmp @r12    */
            out[6] == 0x9f && out[7] == 0xa0) ? 1 : 0;     /* nop         */
}

/* --- the link ---------------------------------------------------------- */

struct link {
    struct chipio io;
    uint32_t staging;
    uint32_t seq;
    int quiet;
};

#define CALL_OK         0       /* answered, status OK                    */
#define CALL_FAIL       1       /* answered, status FAIL (detail says why)*/
#define CALL_XPORT     (-1)     /* the transfer itself failed             */
#define CALL_SILENT    (-2)     /* nothing on the chip answered           */

/* One command, one answer.  The reply buffer is always filled when this
 * returns anything but CALL_XPORT, so a caller can report the status. */
static int call(struct link *l, uint32_t cmd, uint32_t arg0, uint32_t arg1,
                const uint8_t *data, size_t ndata, uint8_t reply[64])
{
    uint8_t request[64];
    const char *why;
    int32_t status;

    l->seq++;
    frq1_build(request, l->seq, cmd, arg0, arg1, data, ndata);
    if (chipio_dispatch(&l->io, l->staging, request, reply) != 0) {
        fprintf(stderr, "%s: %s\n", frq1_command_name(cmd), l->io.err);
        return CALL_XPORT;
    }

    /* Four checks make a reply, and frq1_reply_check() is all of them
     * (frq1.h): magic, the command echo, the sequence, and a status that is
     * no longer PENDING.  Each one catches a different way of being alone
     * on the wire.
     *
     * With no firmware of ours installed, the resident loader treats
     * sub-opcode 0xC1 like a plain window write and stores our own 64 bytes
     * AT the staging address; the read-back is then our request verbatim
     * and the status is still PENDING.  A reply whose magic matches but
     * whose command word is not our command (something else answering) is
     * caught by the echo check.  And on a GV-M2TV, where the staging
     * region is not linear RAM, the read-back can carry our own magic at
     * the right offset with the rest of the frame something else (measured):
     * the sequence check is what catches that one. */
    why = frq1_reply_check(reply, l->seq, cmd);
    if (why != NULL) {
        if (!l->quiet)
            printf("  %-13s %s\n", frq1_command_name(cmd), why);
        return CALL_SILENT;
    }

    status = frq1_get32s(reply, FRQ1_FR_STATUS);
    if (!l->quiet)
        printf("  %-13s status %s\n", frq1_command_name(cmd),
               frq1_status_name(status));
    return status == FRQ1_ST_OK ? CALL_OK : CALL_FAIL;
}

static void explain_silence(const struct link *l)
{
    fflush(stdout);
    fprintf(stderr,
        "nothing of ours answered at the staging address 0x%08x.\n"
        "  Either no firmware of ours is hooked -- the resident loader then\n"
        "  stores a sub-opcode 0xC1 request at that address and the\n"
        "  read-back is the request itself -- or the staging\n"
        "  address is wrong for this machine, in which case the read-back is\n"
        "  whatever else lives there.  On a dione (MB86H5x) part the frame\n"
        "  goes into that address as a mailbox and only a four-byte doorbell\n"
        "  is sent: with no firmware the doorbell clears the magic, so look\n"
        "  for a read-back that starts with zeros rather than the request.\n"
        "Install a firmware:\n"
        "      ./frc1_load install ../build/frc1-<id>.frc\n"
        "  or point --staging at this machine's buffer.\n",
        l->staging);
}

/* --- probe ------------------------------------------------------------- */

static int cmd_probe(struct link *l)
{
    uint8_t r[64];
    int rc;

    printf("staging %#010x on %s\n", l->staging, chipio_backend());

    /* One round trip.  IDENTIFY says who answered and what it can do
     * (../docs/frq1.md). */
    rc = call(l, FRQ1_CMD_IDENTIFY, 0, 0, NULL, 0, r);
    if (rc == CALL_SILENT) {
        explain_silence(l);
        return 1;
    }
    if (rc == CALL_XPORT)
        return 1;
    if (rc == CALL_OK) {
        const uint8_t *d = r + FRQ1_FR_DATA;
        unsigned sources = d[FRQ1_ID_SOURCES];

        printf("     signature '%.4s'   report v%u  frame v%u  %u "
               "receiver(s)\n", (const char *)(d + FRQ1_ID_SIGNATURE),
               d[FRQ1_ID_VERSION], d[FRQ1_ID_PROTOCOL], d[FRQ1_ID_RECEIVERS]);
        printf("     sources   %#04x%s\n", sources,
               sources == 0 ? "   (this build does not turn a tuner)" : "");
        if (d[FRQ1_ID_VERSION] >= 2) {
            uint32_t board = frq1_get32(r, FRQ1_FR_DATA + FRQ1_ID_BOARD);
            uint32_t caps = frq1_get32(r, FRQ1_FR_DATA + FRQ1_ID_CAPS);

            if (board)
                printf("     board     %04x:%04x", board >> 16,
                       board & 0xffffu);
            else
                printf("     board     none (an image built for no product)");
            printf("   caps %#x%s%s\n", caps,
                   caps & FRQ1_ID_CAP_DIONE ? " dione" : "",
                   caps & FRQ1_ID_CAP_CARD ? " card" : "");
        }
        if (d[FRQ1_ID_VERSION] >= 3)
            printf("     build     %#010x   (crc32 of .text: frc1_load info "
                   "shows the main segment's)\n",
                   frq1_get32(r, FRQ1_FR_DATA + FRQ1_ID_BUILD));
        if (d[FRQ1_ID_PROTOCOL] != FRQ1_VERSION)
            printf("     NOTE: the firmware speaks frame version %u, this "
                   "host %u\n", d[FRQ1_ID_PROTOCOL], FRQ1_VERSION);
    }
    return 0;
}

/* --- save -------------------------------------------------------------- */

/* Every 188th byte should be 0x47: the firmware hands over a byte stream
 * that starts at a packet boundary, because ring_sync() found the record
 * head before the first burst went out (../docs/ts.md). */
static void sync_report(const uint8_t *buf, size_t n)
{
    size_t packets = n / FRQ1_TS_PACKET;
    size_t good = 0;
    size_t i;

    for (i = 0; i < packets; i++)
        if (buf[i * FRQ1_TS_PACKET] == FRQ1_TS_SYNC)
            good++;
    printf("  sync      %zu of %zu packet starts carry 0x47%s\n", good,
           packets, (packets && good == packets) ? "  (aligned)" : "");
}

static int cmd_save(struct link *l, const char *path, unsigned seconds, unsigned bursts, unsigned ep,
                    unsigned src)
{
    uint8_t r[64];
    uint8_t chunk[16384];
    uint8_t head[188 * 8];
    size_t head_got = 0;
    unsigned long total = 0;
    unsigned long reported = 0;
    unsigned requests = 0, drained = 0, empty = 0;
    time_t end;
    FILE *out;
    int rc;

    if (bursts > FRQ1_SR_BURSTS_MAX)
        bursts = FRQ1_SR_BURSTS_MAX;

    out = open_ts(path);
    if (out == NULL) {
        fprintf(stderr, "%s: cannot write\n", path);
        return 1;
    }

    rc = call(l, FRQ1_CMD_START, src, 0, NULL, 0, r);
    if (rc == CALL_SILENT) {
        explain_silence(l);
        fclose(out);
        return 1;
    }
    if (rc == CALL_XPORT) {
        fclose(out);
        return 1;
    }

    printf("\nreading ep %#02x into %s for %u s, %u burst(s) a request\n",
           ep, path, seconds, bursts);
    end = time(NULL) + (time_t)seconds;

    while (time(NULL) < end) {
        uint32_t flags, bytes, used;
        int got;

        rc = call(l, FRQ1_CMD_STREAM, bursts, 0, NULL, 0, r);
        if (rc == CALL_XPORT)
            break;
        if (rc == CALL_SILENT) {
            explain_silence(l);
            break;
        }
        requests++;
        if (rc == CALL_FAIL) {
            fprintf(stderr, "STREAM failed: %s, rc %d -- the capture "
                    "path did not come up\n",
                    frq1_status_name(frq1_get32s(r, FRQ1_FR_STATUS)),
                    frq1_get32s(r, FRQ1_FR_DATA + FRQ1_SR_RC));
            break;
        }
        flags = r[FRQ1_FR_DATA + FRQ1_SR_FLAGS];
        bytes = frq1_get32(r, FRQ1_FR_DATA + FRQ1_SR_BYTES);
        used = frq1_get32(r, FRQ1_FR_DATA + FRQ1_SR_USED);
        reported += bytes;
        if (flags & FRQ1_SR_F_DRAINED)
            drained++;
        if (bytes == 0)
            empty++;

        /* Drain whatever the bursts put in the endpoint.  A timeout with
         * nothing in hand is the normal end of a round, not a failure. */
        for (;;) {
            rc = chipio_bulk(&l->io, ep, chunk, (int)sizeof(chunk), DRAIN_MS,
                             &got);
            if (rc < 0) {
                fprintf(stderr, "%s\n", l->io.err);
                break;
            }
            if (got > 0) {
                if (head_got < sizeof(head)) {
                    size_t room = sizeof(head) - head_got;
                    size_t take = (size_t)got < room ? (size_t)got : room;

                    memcpy(head + head_got, chunk, take);
                    head_got += take;
                }
                if (fwrite(chunk, 1, (size_t)got, out) != (size_t)got) {
                    fprintf(stderr, "%s: short write\n", path);
                    rc = -1;
                    break;
                }
                total += (unsigned long)got;
            }
            if (rc == 1 || got == 0)
                break;          /* nothing more ready right now */
        }
        if (rc < 0)
            break;
        if ((requests % 8u) == 0u) {
            printf("  %u requests  %lu bytes on the wire  ring %u byte(s) "
                   "left%s\n", requests, total, used,
                   (flags & FRQ1_SR_F_SYNCED) ? "  synced" : "");
            fflush(stdout);
        }
    }

    (void)call(l, FRQ1_CMD_STOP, 0, 0, NULL, 0, r);
    fclose(out);

    printf("\n%s\n", path);
    printf("  requests  %u   of which %u said the ring was empty, %u hit a "
           "busy endpoint\n", requests, empty, drained);
    printf("  firmware  %lu byte(s) pushed\n", reported);
    printf("  host      %lu byte(s) written (%lu packet(s))\n", total,
           total / FRQ1_TS_PACKET);
    if (head_got)
        sync_report(head, head_got);
    if (total == 0) {
        printf("\nNo bytes arrived.  In order, the things that stop a stream:\n"
               "  * STREAM answering a failure -- tsif_init() refused the "
               "strap, or\n    ring_init() found the capture registers at 0 "
               "(../docs/ts.md)\n"
               "  * STREAM OK with 0 bursts every time -- the capture block "
               "is up but\n    nothing is being captured, which is what an "
               "untuned frontend looks like\n"
               "  * bursts going out but nothing on ep %#02x -- try the other "
               "bulk IN\n    endpoint with --ep (the firmware's burst "
               "routine knows two pairs)\n", ep);
        return 1;
    }
    return 0;
}

/* --- save on dione ---------------------------------------------------- *
 *
 * THE SAME COMMAND, A DIFFERENT SHAPE.  On dione (the GV-M2TV) one STREAM
 * frame moves up to 64 bursts (32 KiB) by default, and a burst waits until the
 * endpoint FIFO has room -- so a frame only finishes if the host is reading
 * the endpoint WHILE the frame runs.  The alternation above cannot do that:
 * the drain would start after the frame it is supposed to feed.  So here a
 * second thread reads the endpoint the whole time, and the main thread only
 * sends frames.  libusb allows a synchronous bulk read on one thread and
 * control transfers on another.
 *
 * WHAT IS WRITTEN IS WHOLE PACKETS.  The stream starts in the middle of
 * a packet, so the reader finds 0x47 first and writes only complete
 * 188-byte packets, counting what it dropped.  If sync is lost later it
 * finds 0x47 again (with the firmware's ring read offset written back,
 * that does not happen; ../docs/ts.md).  It also keeps
 * the continuity counter of every PID, because a jump there is a lost
 * packet that resynchronising cannot show.
 *
 * TUNING IS SEPARATE.  `save --ch` runs cmd_tune_dione() below first; this
 * function records whatever is tuned.
 */
#define DN_CHUNK        65536
#define DN_RING_LIMIT_ADDR 0x00D02024u  /* then write, read (docs/ts.md)   */
#define DN_READ_MS      200u
#define DN_STALL_S      2

struct ts_sink {
    FILE *out;
    uint8_t buf[DN_CHUNK + 2 * 188];
    size_t have;
    int synced;
    unsigned long packets, dropped, resyncs, cc_jumps, pat;
    unsigned long teis;                 /* packets with TEI set       */
    uint8_t cc[8192];                   /* 0xff: not seen yet */
    atomic_int failed;                  /* set by the reader, read by main */
    int failed_errno;                   /* why the write failed            */
    /* For `bench`: when the reader saw what, on now_s()'s clock.  0 = not
     * yet.  bench_mark() clears them; the reader only ever sets them. */
    _Atomic double t_byte, t_pat_new, t_pmt, t_pat_old, t_err;
    atomic_uint base_tsid;              /* a PAT with this TSID is "old"  */
    atomic_uint last_tsid;              /* 0x10000: no PAT yet            */
    atomic_uint pmt_pid;                /* the new PAT's first programme  */
};

static double now_s(void);

/* Where a packet's section starts (after any adaptation field and the
 * pointer field), or NULL when it has none. */
static const uint8_t *ts_section(const uint8_t *p)
{
    unsigned at = 4;

    if (!(p[1] & 0x40u) || !(p[3] & 0x10u))
        return NULL;                    /* no section start, or no payload */
    if (p[3] & 0x20u)
        at += 1u + p[4];                /* the adaptation field           */
    if (at >= FRQ1_TS_PACKET)
        return NULL;
    at += 1u + p[at];                   /* the pointer field              */
    return at < FRQ1_TS_PACKET ? p + at : NULL;
}

/* MPEG-2's CRC-32 (poly 0x04C11DB7, MSB first, no final xor): over a whole
 * section including its CRC it comes to 0. */
static uint32_t crc_mpeg(const uint8_t *p, size_t n)
{
    uint32_t c = 0xffffffffu;
    unsigned b;

    while (n--) {
        c ^= (uint32_t)*p++ << 24;
        for (b = 0; b < 8u; b++)
            c = (c & 0x80000000u) ? (c << 1) ^ 0x04c11db7u : c << 1;
    }
    return c;
}

/* A PAT's start: note its TSID, and the PMT PID of its first programme.
 * Only a section that fits in the packet and passes its CRC counts: around
 * a retune the demodulator hands over damaged packets too. */
static void sink_pat(struct ts_sink *s, const uint8_t *p)
{
    const uint8_t *sec, *end, *e;
    unsigned tsid, len, pmt = 0;

    sec = ts_section(p);
    end = p + FRQ1_TS_PACKET;
    if (sec == NULL || sec + 8 > end || sec[0] != 0x00u)
        return;
    len = ((unsigned)(sec[1] & 0x0fu) << 8) | sec[2];
    if (len < 9u || sec + 3 + len > end || crc_mpeg(sec, 3u + len) != 0u)
        return;
    tsid = ((unsigned)sec[3] << 8) | sec[4];
    for (e = sec + 8; e + 4 <= end && e + 4 <= sec + 3 + len - 4; e += 4)
        if ((e[0] | e[1]) != 0u) {      /* programme 0 is the NIT        */
            pmt = ((unsigned)(e[2] & 0x1fu) << 8) | e[3];
            break;
        }
    if (tsid == atomic_load(&s->base_tsid)) {
        atomic_store(&s->t_pat_old, now_s());
    } else if (atomic_load(&s->t_pat_new) == 0.0) {
        /* A new multiplex: what the old one's PIDs counted to, and what
         * damaged packets before the lock left, say nothing about it. */
        memset(s->cc, 0xff, sizeof(s->cc));
        atomic_store(&s->pmt_pid, pmt);
        atomic_store(&s->t_pat_new, now_s());
    }
    atomic_store(&s->last_tsid, tsid);
}

static void sink_packet(struct ts_sink *s, const uint8_t *p)
{
    unsigned pid = ((unsigned)(p[1] & 0x1fu) << 8) | p[2];
    unsigned cc = p[3] & 0x0fu;

    /* TEI: the demodulator could not correct this packet. */
    if (p[1] & 0x80u)
        s->teis++;
    if (pid == 0) {
        s->pat++;
        sink_pat(s, p);
    } else if (atomic_load(&s->t_pat_new) != 0.0 &&
               atomic_load(&s->t_pmt) == 0.0 &&
               pid == atomic_load(&s->pmt_pid) && ts_section(p) != NULL &&
               ts_section(p)[0] == 0x02u) {
        atomic_store(&s->t_pmt, now_s());
    }
    /* Only packets with a payload advance the counter, and the null PID
     * has none worth counting.  A repeat of the last value is allowed. */
    if (pid != 0x1fffu && (p[3] & 0x10u)) {
        unsigned last = s->cc[pid];

        if (last != 0xffu && cc != last && cc != ((last + 1u) & 0x0fu)) {
            s->cc_jumps++;
            atomic_store(&s->t_err, now_s());
        }
        s->cc[pid] = (uint8_t)cc;
    }
    if (!atomic_load(&s->failed) &&
        fwrite(p, 1, FRQ1_TS_PACKET, s->out) != FRQ1_TS_PACKET) {
        s->failed_errno = errno;
        atomic_store(&s->failed, 1);
    }
    s->packets++;
}

/* s->buf[0 .. s->have) holds the unread bytes.  A packet is taken when its
 * first byte is 0x47 -- and, when out of step, when the next one is too. */
static void sink_feed(struct ts_sink *s)
{
    const uint8_t *p = s->buf;
    size_t i = 0;

    for (;;) {
        size_t left = s->have - i;

        if (left < FRQ1_TS_PACKET)
            break;
        if (p[i] == FRQ1_TS_SYNC) {
            if (s->synced) {
                sink_packet(s, p + i);
                i += FRQ1_TS_PACKET;
                continue;
            }
            if (left < 2u * FRQ1_TS_PACKET)
                break;                  /* cannot confirm yet */
            if (p[i + FRQ1_TS_PACKET] == FRQ1_TS_SYNC) {
                s->synced = 1;
                continue;
            }
        } else if (s->synced) {
            s->synced = 0;
            s->resyncs++;
            atomic_store(&s->t_err, now_s());
        }
        s->dropped++;
        i++;
    }
    memmove(s->buf, s->buf + i, s->have - i);
    s->have -= i;
}

struct reader {
    struct chipio *io;
    unsigned ep;
    struct ts_sink *sink;
    atomic_int stop;
    atomic_ulong bytes;
    atomic_int failed;
    FILE *raw;                          /* env TS_RAW: every byte as read */
    char err[256];
};

static void *reader_main(void *arg)
{
    struct reader *r = arg;
    struct ts_sink *s = r->sink;

    while (!atomic_load(&r->stop)) {
        int got = 0;
        int rc = chipio_bulk(r->io, r->ep, s->buf + s->have, DN_CHUNK,
                             DN_READ_MS, &got);

        if (rc < 0) {
            snprintf(r->err, sizeof(r->err), "%s", r->io->err);
            atomic_store(&r->failed, 1);
            break;
        }
        if (got > 0) {
            if (r->raw)
                (void)fwrite(s->buf + s->have, 1, (size_t)got, r->raw);
            if (atomic_load(&s->t_byte) == 0.0)
                atomic_store(&s->t_byte, now_s());
            s->have += (size_t)got;
            atomic_fetch_add(&r->bytes, (unsigned long)got);
            sink_feed(s);
        }
    }
    return NULL;
}

static double now_s(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

/* What one endpoint's reader got, for the summary. */
static void sink_summary(const char *path, unsigned ep,
                         const struct ts_sink *s, unsigned long total)
{
    printf("  ep %#02x    %s\n", ep, strcmp(path, "-") ? path : "(stdout)");
    printf("  host      %lu byte(s) read, %lu packet(s) written\n", total,
           s->packets);
    printf("  sync      %lu byte(s) dropped, %lu resync(s)\n", s->dropped,
           s->resyncs);
    printf("  cc        %lu jump(s)   PAT %lu packet(s)\n", s->cc_jumps,
           s->pat);
    printf("  tei       %lu packet(s) the demod could not correct",
           s->teis);
    if (s->packets)
        printf(" (%.2e of the packets)", (double)s->teis / s->packets);
    printf("\n");
}

/* mirror: when not NULL, the firmware sends the same bursts to EP 0x84
 * too, and a second reader writes them there. */
static int cmd_save_dione(struct link *l, const char *path,
                          unsigned seconds, unsigned bursts, unsigned ring,
                          unsigned ep, const char *mirror)
{
    static struct ts_sink sink;         /* 72 KiB: not for the stack */
    static struct ts_sink sink2;
    struct reader rd, rd2;
    pthread_t th, th2;
    struct sigaction sa;
    char span[48];
    uint8_t r[64];
    const uint8_t *d = r + FRQ1_FR_DATA;
    unsigned frames = 0, shorts = 0, laps = 0, peak = 0, ring_len = 0;
    unsigned fulls = 0, misses = 0, packets = 0, teis = 0, dsr_ver = 0;
    unsigned long seen = 0, total;
    double t0, last_print, moved, pause_at = 0.0, pause_s = 0.0;
    int rc = 0, got;

    if (bursts > FRQ1_DSR_BURSTS_MAX)
        bursts = FRQ1_DSR_BURSTS_MAX;
    if (ring > FRQ1_DSR_RING_MAX)
        ring = FRQ1_DSR_RING_MAX;

    /* A recorder stops a tuner command with SIGTERM, or by closing the
     * pipe: either way, send STOP before leaving.  No SA_RESTART, so a
     * signal ends a wait early. */
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_quit;
    sigemptyset(&sa.sa_mask);
    (void)sigaction(SIGINT, &sa, NULL);
    (void)sigaction(SIGTERM, &sa, NULL);
    signal(SIGPIPE, SIG_IGN);           /* a closed pipe is EPIPE, not death */

    if (chipio_claim(&l->io) != 0) {
        fprintf(stderr, "%s\n", l->io.err);
        return 1;
    }
    /* Whatever an earlier run left in the FIFO is not this recording. */
    while (chipio_bulk(&l->io, ep, sink.buf, DN_CHUNK, 60u, &got) == 0 &&
           got > 0)
        ;
    if (mirror != NULL)
        while (chipio_bulk(&l->io, 0x84u, sink.buf, DN_CHUNK, 60u,
                           &got) == 0 && got > 0)
            ;

    memset(&sink, 0, sizeof(sink));
    memset(sink.cc, 0xff, sizeof(sink.cc));
    sink.out = open_ts(path);
    if (sink.out == NULL) {
        fprintf(stderr, "%s: cannot write\n", path);
        return 1;
    }
    if (mirror != NULL) {
        memset(&sink2, 0, sizeof(sink2));
        memset(sink2.cc, 0xff, sizeof(sink2.cc));
        sink2.out = open_ts(mirror);
        if (sink2.out == NULL) {
            fprintf(stderr, "%s: cannot write\n", mirror);
            fclose(sink.out);
            return 1;
        }
    }

    memset(&rd, 0, sizeof(rd));
    rd.io = &l->io;
    rd.ep = ep;
    rd.sink = &sink;
    if (getenv("TS_RAW") != NULL)       /* for finding where bytes went */
        rd.raw = fopen(getenv("TS_RAW"), "wb");     /* flushed at exit */
    atomic_init(&rd.stop, 0);
    atomic_init(&rd.bytes, 0ul);
    atomic_init(&rd.failed, 0);
    if (pthread_create(&th, NULL, reader_main, &rd) != 0) {
        fprintf(stderr, "cannot start the reader thread\n");
        fclose(sink.out);
        return 1;
    }
    if (mirror != NULL) {
        memset(&rd2, 0, sizeof(rd2));
        rd2.io = &l->io;
        rd2.ep = 0x84u;
        rd2.sink = &sink2;
        atomic_init(&rd2.stop, 0);
        atomic_init(&rd2.bytes, 0ul);
        atomic_init(&rd2.failed, 0);
        if (pthread_create(&th2, NULL, reader_main, &rd2) != 0) {
            fprintf(stderr, "cannot start the second reader thread\n");
            atomic_store(&rd.stop, 1);
            pthread_join(th, NULL);
            fclose(sink.out);
            fclose(sink2.out);
            return 1;
        }
        printf("and ep 0x84 into %s (the same bursts)\n", mirror);
    }

    if (seconds)
        snprintf(span, sizeof(span), "for %u s", seconds);
    else
        snprintf(span, sizeof(span), "until stopped (SIGINT / SIGTERM)");
    printf("reading ep %#02x into %s %s, %u burst(s) a frame, "
           "ring %u MiB\n", ep, strcmp(path, "-") ? path : "stdout", span,
           bursts ? bursts : FRQ1_DSR_BURSTS_DEF,
           ring ? ring : FRQ1_DSR_RING_MAX);
    t0 = last_print = moved = now_s();
    l->quiet = 1;
    /* env TS_PAUSE=at,secs: send no STREAM for `secs` from `at` seconds in,
     * reading the ring's limit / write / read offsets every 100 ms instead.
     * For seeing what the capture does when the ring fills. */
    if (getenv("TS_PAUSE") != NULL &&
        sscanf(getenv("TS_PAUSE"), "%lf,%lf", &pause_at, &pause_s) != 2)
        pause_at = pause_s = 0.0;
    while ((seconds == 0 || now_s() - t0 < (double)seconds) && !g_quit &&
           !atomic_load(&rd.failed) && !atomic_load(&sink.failed) &&
           !(mirror != NULL && (atomic_load(&rd2.failed) ||
                                atomic_load(&sink2.failed)))) {
        double t;

        if (pause_s > 0.0 && now_s() - t0 >= pause_at &&
            now_s() - t0 < pause_at + pause_s) {
            if (call(l, FRQ1_CMD_PEEK, DN_RING_LIMIT_ADDR,
                     FRQ1_PEEK_CTL(FRQ1_W_WORD, 3), NULL, 0, r) == CALL_OK)
                printf("  %5.2f s  paused: limit %#x write %#x read %#x\n",
                       now_s() - t0, frq1_get32(d, FRQ1_PK_VALUES),
                       frq1_get32(d, FRQ1_PK_VALUES + 4),
                       frq1_get32(d, FRQ1_PK_VALUES + 8));
            chipio_nap(100u);
            moved = now_s();
            continue;
        }
        rc = call(l, FRQ1_CMD_STREAM, bursts, ring, &g_outs, 1, r);
        if (rc == CALL_XPORT)
            break;
        if (rc == CALL_SILENT) {
            explain_silence(l);
            break;
        }
        frames++;
        if (rc == CALL_FAIL) {
            fprintf(stderr, "STREAM failed: %s, rc %d -- the capture did "
                    "not come up\n",
                    frq1_status_name(frq1_get32s(r, FRQ1_FR_STATUS)),
                    frq1_get32s(d, FRQ1_DSR_RC));
            if (frq1_get32s(d, FRQ1_DSR_RC) == FRQ1_E_IO &&
                frq1_get32(d, FRQ1_DSR_BAD_REG))
                fprintf(stderr, "  %#010x read back %#010x\n",
                        frq1_get32(d, FRQ1_DSR_BAD_REG),
                        frq1_get32(d, FRQ1_DSR_BAD_VAL));
            break;
        }
        if (d[FRQ1_DSR_FLAGS] & FRQ1_DSR_F_SHORT)
            shorts++;
        laps = frq1_get32(d, FRQ1_DSR_LAPS);
        peak = frq1_get32(d, FRQ1_DSR_PEAK);
        dsr_ver = d[FRQ1_DSR_VERSION];
        /* From version 3 the ring length's word carries the TEI count; the
         * ring is the length this tool asked for. */
        if (dsr_ver >= 3) {
            ring_len = (ring ? ring : FRQ1_DSR_RING_MAX) << 20;
            teis = frq1_get32(d, FRQ1_DSR_TEIS);
        } else {
            ring_len = frq1_get32(d, FRQ1_DSR_LENGTH);
        }
        if (dsr_ver >= 2) {
            fulls = d[FRQ1_DSR_FULLS];
            misses = d[FRQ1_DSR_MISSES];
            packets = frq1_get32(d, FRQ1_DSR_PACKETS);
        }

        t = now_s();
        total = atomic_load(&rd.bytes);
        if (total != seen) {
            seen = total;
            moved = t;
        } else if (t - moved > DN_STALL_S) {
            fprintf(stderr, "nothing on ep %#02x for %d s\n", ep,
                    DN_STALL_S);
            rc = CALL_FAIL;
            break;
        }
        if (t - last_print >= 1.0) {
            last_print = t;
            printf("  %5.1f s  %8.1f MB  (%.2f MB/s)  laps %u  write %#x  "
                   "read %#x  tei %u\n", t - t0, (double)total / 1e6,
                   (double)total / (t - t0) / 1e6, laps,
                   frq1_get32(d, FRQ1_DSR_WRITE),
                   frq1_get32(d, FRQ1_DSR_READ), teis);
            fflush(stdout);
        }
    }
    l->quiet = 0;

    /* STOP, then let the reader take what is still in flight. */
    (void)call(l, FRQ1_CMD_STOP, 0, 0, NULL, 0, r);
    chipio_nap(250u);
    atomic_store(&rd.stop, 1);
    pthread_join(th, NULL);
    fclose(sink.out);
    total = atomic_load(&rd.bytes);
    if (mirror != NULL) {
        atomic_store(&rd2.stop, 1);
        pthread_join(th2, NULL);
        fclose(sink2.out);
    }

    printf("\n  frames    %u   of which %u ran out of time, %u lap(s)\n",
           frames, shorts, laps);
    if (ring_len)
        printf("  ring      peak %u of %u byte(s) waiting (%.1f%%)\n", peak,
               ring_len, 100.0 * peak / ring_len);
    if (dsr_ver >= 2)
        printf("  firmware  %u packet(s) sent, %u start(s) without 0x47, "
               "ring found full %u time(s)\n", packets, misses, fulls);
    if (dsr_ver >= 3)
        printf("  firmware  %u packet(s) with TEI set\n", teis);
    sink_summary(path, ep, &sink, total);
    if (mirror != NULL)
        sink_summary(mirror, 0x84u, &sink2, atomic_load(&rd2.bytes));
    if (atomic_load(&rd.failed)) {
        fprintf(stderr, "%s\n", rd.err);
        return 1;
    }
    if (mirror != NULL && atomic_load(&rd2.failed)) {
        fprintf(stderr, "%s\n", rd2.err);
        return 1;
    }
    if (g_quit)
        printf("  stopped by a signal\n");
    /* The reader of a pipe went away: the end of a recording, not a fault. */
    if (atomic_load(&sink.failed) && sink.failed_errno == EPIPE &&
        mirror == NULL) {
        printf("  %s closed (EPIPE): stopped\n",
               strcmp(path, "-") ? path : "stdout");
        return total ? 0 : 1;
    }
    if (atomic_load(&sink.failed) ||
        (mirror != NULL && atomic_load(&sink2.failed))) {
        fprintf(stderr, "short write: %s\n",
                strerror(atomic_load(&sink.failed) ? sink.failed_errno
                                                   : sink2.failed_errno));
        return 1;
    }
    if (total == 0) {
        printf("\nNo bytes arrived.  Is a channel tuned?  STREAM moves only "
               "what the\ndemodulator hands over (../docs/ts.md).\n");
        return 1;
    }
    return rc == CALL_OK ? 0 : 1;
}

/* --- tuning on dione -------------------------------------------------- *
 *
 * The firmware has no TUNE on dione, but it does I2C (I2CXFER, one transfer
 * a frame), so the host tunes: it sends the tuning sequence one transfer
 * at a time and then reads the demodulator's lock registers.  The sequence
 * is fe_m2tv.h's (about 25 frames the first time, 4 for each channel
 * after).
 */
/* bench's zap-i: a STREAM frame after every transfer, so the TS keeps
 * moving while the tuning sequence goes out. */
static int g_pump;
static unsigned g_pump_bursts, g_pump_ring;

static int dn_xfer(void *ctx, unsigned addr7, const uint8_t *w, unsigned wn,
                   uint8_t *r, unsigned rn)
{
    struct link *l = ctx;
    uint8_t reply[64];
    int quiet = l->quiet;
    int rc;

    l->quiet = 1;
    rc = call(l, FRQ1_CMD_I2CXFER, FRQ1_I2CX_ARG(addr7, wn, rn), 0, w, wn,
              reply);
    if (rc == CALL_OK && rn)
        memcpy(r, reply + FRQ1_FR_DATA, rn);
    if (rc == CALL_OK && g_pump) {
        uint8_t pr[64];

        (void)call(l, FRQ1_CMD_STREAM, g_pump_bursts, g_pump_ring, &g_outs, 1,
                   pr);
    }
    l->quiet = quiet;
    if (rc != CALL_OK) {
        if (rc == CALL_FAIL)
            fprintf(stderr, "I2CXFER to 0x%02x: %s\n", addr7,
                    frq1_status_name(frq1_get32s(reply, FRQ1_FR_STATUS)));
        else if (rc == CALL_SILENT)
            explain_silence(l);
        return -1;
    }
    return 0;
}

/* A dione channel spec to kHz:
 *   "27" or "U27"   UHF 13..62, 6 MHz apart from 473,143 kHz (ch 13)
 *   "557143k"       a frequency in kHz, 90,000..770,000
 * 0 when it is neither.  Whether the sequence is measured there is
 * dn_measured()'s question, not this one's. */
static unsigned dn_channel_khz(const char *spec)
{
    const char *p = spec;
    char *end;
    unsigned long n;

    if (*p == 'U' || *p == 'u')
        p++;
    n = strtoul(p, &end, 10);
    if (end == p)
        return 0;
    if (p == spec && (*end == 'k' || *end == 'K') && end[1] == '\0')
        return n >= 90000ul && n <= 770000ul ? (unsigned)n : 0u;
    if (*end != '\0' || n < 13ul || n > 62ul)
        return 0;
    return 473143u + 6000u * (unsigned)(n - 13ul);
}

/* The demodulator's three lock bytes (0x80, 0xB0, 0x96), only to be
 * printed; fe_m2tv.h's m2tv_status() makes the lock decision. */
static int dn_lock_bytes(struct link *l, uint8_t out[3])
{
    static const uint8_t reg[3] = { 0x80, 0xB0, 0x96 };
    unsigned i;

    for (i = 0; i < 3u; i++)
        if (dn_xfer(l, M2TV_DEMOD, &reg[i], 1u, &out[i], 1u) != 0)
            return -1;
    return 0;
}

/* st gets the lock bytes to print.  1 locked, 0 not, -1 the bus. */
static int dn_poll(struct link *l, uint8_t st[3])
{
    int locked;

    /* decide first, then read what is printed: the other way round the
     * line can show the bytes from just before the lock */
    if (m2tv_status(dn_xfer, l, &locked, NULL) != 0 ||
        dn_lock_bytes(l, st) != 0)
        return -1;
    return locked;
}

/* The built-in sequence is measured on UHF below 700 MHz only. */
static int dn_measured(unsigned khz)
{
    return khz >= M2TV_KHZ_MIN && khz <= M2TV_KHZ_MAX;
}

/* Returns 1 locked, 0 not locked within wait_ms, -1 the bus failed. */
static int dn_tune(struct link *l, unsigned khz, unsigned wait_ms, int init,
                   uint8_t st[3])
{
    unsigned waited = 0;
    int rc;

    if (init && m2tv_init(dn_xfer, l) != 0)
        return -1;
    if (m2tv_tune(dn_xfer, l, khz) != 0)
        return -1;
    for (;;) {
        rc = dn_poll(l, st);
        if (rc != 0)
            return rc;
        if (waited >= wait_ms)
            return 0;
        chipio_nap(20u);
        waited += 20u;
    }
}

/* 0 locked, 1 no lock, -1 the bus failed.  The channel was checked in
 * main(). */
/* The firmware's own reading of the demodulator (STATUS, fe.c), to set
 * beside the host's.  START first, once: STATUS reports "not initialised"
 * until fe_init has run.  Prints one short field; never fails the caller. */
static void show_fw_status(struct link *l, int *started)
{
    uint8_t r[64];
    int rc;

    l->quiet = 1;
    if (!*started) {
        (void)call(l, FRQ1_CMD_START, 0, 0, NULL, 0, r);
        *started = 1;
    }
    rc = call(l, FRQ1_CMD_STATUS, 0, 0, NULL, 0, r);
    l->quiet = 0;
    if (rc != CALL_OK) {
        printf("  fw: STATUS %s", rc == CALL_FAIL
               ? frq1_status_name(frq1_get32s(r, FRQ1_FR_STATUS))
               : "no answer");
        return;
    }
    printf("  fw: %s cnr %#08x flags %#04x",
           frq1_get32(r, FRQ1_FR_DATA + FRQ1_FE_LOCKED) ? "locked" : "no lock",
           frq1_get32(r, FRQ1_FR_DATA + FRQ1_FE_CNR),
           r[FRQ1_FR_DATA + FRQ1_FE_FLAGS]);
}

static int cmd_tune_dione(struct link *l, const char *spec)
{
    int started = 0;
    uint8_t st[3];
    unsigned khz = dn_channel_khz(spec);
    int rc;

    if (!dn_measured(khz)) {
        fprintf(stderr, "%s = %u kHz: the built-in sequence is measured on "
                "%u..%u kHz only\n", spec, khz, M2TV_KHZ_MIN, M2TV_KHZ_MAX);
        return -1;
    }
    printf("tuning %s = %u kHz\n", spec, khz);
    rc = dn_tune(l, khz, 2000u, 1, st);
    if (rc < 0)
        return -1;
    printf("  demod 0x80=%02x 0xb0=%02x 0x96=%02x   %s", st[0], st[1],
           st[2], rc ? "locked" : "no lock");
    show_fw_status(l, &started);
    printf("\n");
    return rc ? 0 : 1;
}

/* UHF first..last.  Initialises once, then tunes each channel and waits
 * up to 1.2 s for lock. */
static int cmd_scan_dione(struct link *l, unsigned first, unsigned last)
{
    unsigned locked = 0, tried = 0;
    unsigned c;
    int init = 1, started = 0;

    printf("\nch     freq         0x80 0xb0 0x96  host    firmware (STATUS)\n");
    for (c = first; c <= last; c++) {
        char spec[8];
        uint8_t st[3];
        unsigned khz;
        int rc;

        snprintf(spec, sizeof(spec), "%u", c);
        khz = dn_channel_khz(spec);
        if (!dn_measured(khz)) {
            printf("%-5s %7u kHz    (not measured)\n", spec, khz);
            continue;
        }
        rc = dn_tune(l, khz, 1200u, init, st);
        init = 0;
        if (rc < 0)
            return 1;
        tried++;
        locked += (unsigned)rc;
        printf("%-5s %7u kHz    %02x   %02x   %02x  %-6s", spec, khz, st[0],
               st[1], st[2], rc ? "locked" : "");
        show_fw_status(l, &started);
        printf("\n");
        fflush(stdout);
    }
    printf("\n%u of %u channel(s) locked\n", locked, tried);
    return 0;
}

/* --- tuning on the PIX-DT295 ------------------------------------------ *
 *
 * selene has no TUNE either; the host writes both demodulators and both
 * tuners with I2CXFER (fe_dt295.h, every write measured on the device).
 * dt295_init runs once per run of this program, before the first tune.
 */
static int g_dt_inited;

/* Returns 1 locked, 0 not locked within wait_ms, -1 the bus failed.
 * *tsid: the TSID locked to (satellite), else 0.  *cnr: the raw C/N. */
static int dt_tune(struct link *l, int sat, unsigned khz, unsigned ts,
                   unsigned wait_ms, unsigned *tsid, uint32_t *cnr)
{
    unsigned waited = 0, picked = 0;
    int locked = 0;

    *tsid = 0;
    *cnr = 0;
    if (!g_dt_inited) {
        if (dt295_init(dn_xfer, l) != 0)
            return -1;
        g_dt_inited = 1;
    }
    if ((sat ? dt295_sat_tune(dn_xfer, l, khz)
             : dt295_tune(dn_xfer, l, khz)) != 0)
        return -1;
    for (;;) {
        if (sat) {
            if (dt295_sat_poll(dn_xfer, l, ts, &picked, &locked) != 0)
                return -1;
        } else if (dt295_status(dn_xfer, l, &locked, NULL) != 0) {
            return -1;
        }
        if (locked)
            break;
        if (waited >= wait_ms)
            return 0;
        chipio_nap(sat ? 10u : 20u);
        waited += sat ? 10u : 20u;
    }
    *tsid = picked;
    if ((sat ? dt295_sat_status(dn_xfer, l, NULL, cnr)
             : dt295_status(dn_xfer, l, NULL, cnr)) != 0)
        return -1;
    return 1;
}

/* 0 locked, 1 no lock, -1 the bus failed.  The channel was checked in
 * main(). */
static int cmd_tune_dt295(struct link *l, const char *spec)
{
    unsigned khz, ts, tsid;
    uint32_t cnr;
    int sat, rc;

    (void)dt295_channel(spec, &sat, &khz, &ts);
    printf("tuning %s = %s%u kHz\n", spec, sat ? "IF " : "", khz);
    rc = dt_tune(l, sat, khz, ts, 2000u, &tsid, &cnr);
    if (rc < 0)
        return -1;
    if (rc && sat)
        printf("  locked, TSID 0x%04x, C/N raw 0x%04x (bigger is worse)\n",
               tsid, (unsigned)cnr);
    else if (rc)
        printf("  locked, C/N raw 0x%06x (bigger is worse)\n",
               (unsigned)cnr);
    else
        printf("  no lock\n");
    return rc ? 0 : 1;
}

/* UHF first..last (13..50 at most), or with --sat BS1..BS23 / CS2..CS24. */
static int cmd_scan_dt295(struct link *l, unsigned first, unsigned last,
                          int sat)
{
    unsigned locked = 0, tried = 0, c;
    unsigned lo = sat ? 1u : first, hi = sat ? 24u : last;

    printf("\nch       freq            lock    TSID    C/N raw\n");
    for (c = lo; c <= hi; c++) {
        char spec[8];
        unsigned khz, ts, tsid;
        uint32_t cnr;
        int is_sat, rc;

        if (sat)
            snprintf(spec, sizeof(spec), "%s%u", c & 1u ? "BS" : "CS", c);
        else
            snprintf(spec, sizeof(spec), "%u", c);
        if (dt295_channel(spec, &is_sat, &khz, &ts) != 0) {
            printf("%-6s   (not on this model: not measured)\n", spec);
            continue;
        }
        rc = dt_tune(l, is_sat, khz, ts, sat ? 1500u : 1200u, &tsid, &cnr);
        if (rc < 0)
            return 1;
        tried++;
        locked += (unsigned)rc;
        if (rc && is_sat)
            printf("%-6s %s%7u kHz  locked  0x%04x  0x%04x\n", spec, "IF ",
                   khz, tsid, (unsigned)cnr);
        else if (rc)
            printf("%-6s    %7u kHz  locked          0x%06x\n", spec, khz,
                   (unsigned)cnr);
        else
            printf("%-6s %s%7u kHz  -\n", spec, is_sat ? "IF " : "   ", khz);
        fflush(stdout);
    }
    printf("\n%u of %u channel(s) locked\n", locked, tried);
    return 0;
}

/* --- save on selene, with the endpoint read in a thread ----------------
 *
 * cmd_save() above sends STREAM and only then reads the endpoint, so while
 * the firmware pushes nothing on the host is waiting for the data: the
 * endpoint fills after a burst or two and the rest of the frame is
 * FW_EAGAIN (measured on a PIX-DT295: nearly every frame hits a busy
 * endpoint, and the stream comes to a few KiB per second).  Here a thread
 * keeps a read waiting on
 * the endpoint the whole time, as for dione, and this loop only sends
 * STREAM frames back to back.
 */
static int cmd_save_selene(struct link *l, const char *path,
                           unsigned seconds, unsigned bursts, unsigned ep)
{
    static struct ts_sink sink;
    struct reader rd;
    pthread_t th;
    struct sigaction sa;
    uint8_t r[64];
    const uint8_t *d = r + FRQ1_FR_DATA;
    unsigned frames = 0, empty = 0, busy = 0, unsynced = 0;
    unsigned long pushed = 0, total, seen = 0;
    unsigned used = 0, peak = 0;
    double t0, last_print, moved;
    int rc = 0, got;

    if (bursts == 0 || bursts > FRQ1_SR_BURSTS_MAX)
        bursts = FRQ1_SR_BURSTS_MAX;

    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_quit;
    sigemptyset(&sa.sa_mask);
    (void)sigaction(SIGINT, &sa, NULL);
    (void)sigaction(SIGTERM, &sa, NULL);
    signal(SIGPIPE, SIG_IGN);

    if (chipio_claim(&l->io) != 0) {
        fprintf(stderr, "%s\n", l->io.err);
        return 1;
    }
    while (chipio_bulk(&l->io, ep, sink.buf, DN_CHUNK, 60u, &got) == 0 &&
           got > 0)
        ;
    memset(&sink, 0, sizeof(sink));
    memset(sink.cc, 0xff, sizeof(sink.cc));
    sink.out = open_ts(path);
    if (sink.out == NULL) {
        fprintf(stderr, "%s: cannot write\n", path);
        return 1;
    }

    rc = call(l, FRQ1_CMD_START, 0, 0, NULL, 0, r);
    if (rc == CALL_SILENT) {
        explain_silence(l);
        fclose(sink.out);
        return 1;
    }
    if (rc == CALL_XPORT) {
        fclose(sink.out);
        return 1;
    }

    memset(&rd, 0, sizeof(rd));
    rd.io = &l->io;
    rd.ep = ep;
    rd.sink = &sink;
    atomic_init(&rd.stop, 0);
    atomic_init(&rd.bytes, 0ul);
    atomic_init(&rd.failed, 0);
    if (pthread_create(&th, NULL, reader_main, &rd) != 0) {
        fprintf(stderr, "cannot start the reader thread\n");
        fclose(sink.out);
        return 1;
    }

    printf("reading ep %#02x into %s for %u s, %u burst(s) a frame, "
           "endpoint read in a thread\n", ep,
           strcmp(path, "-") ? path : "stdout", seconds, bursts);
    t0 = last_print = moved = now_s();
    l->quiet = 1;
    rc = 0;
    while ((seconds == 0 || now_s() - t0 < (double)seconds) && !g_quit &&
           !atomic_load(&rd.failed) && !atomic_load(&sink.failed)) {
        unsigned flags;
        double t;
        int c = call(l, FRQ1_CMD_STREAM, bursts, 0, NULL, 0, r);

        if (c == CALL_XPORT || c == CALL_SILENT) {
            rc = 1;
            break;
        }
        frames++;
        if (c == CALL_FAIL) {
            fprintf(stderr, "STREAM failed: %s, rc %d -- the capture "
                    "path did not come up\n",
                    frq1_status_name(frq1_get32s(r, FRQ1_FR_STATUS)),
                    frq1_get32s(d, FRQ1_SR_RC));
            rc = 1;
            break;
        }
        flags = d[FRQ1_SR_FLAGS];
        pushed += frq1_get32(d, FRQ1_SR_BYTES);
        used = frq1_get32(d, FRQ1_SR_USED);
        if (used > peak)
            peak = used;
        if (!(flags & FRQ1_SR_F_SYNCED))
            unsynced++;
        if (frq1_get32(d, FRQ1_SR_BYTES) == 0)
            empty++;
        if (flags & FRQ1_SR_F_DRAINED)
            busy++;             /* ring empty or endpoint busy: E_AGAIN */

        t = now_s();
        total = atomic_load(&rd.bytes);
        if (total != seen) {
            seen = total;
            moved = t;
        } else if (t - moved > DN_STALL_S) {
            fprintf(stderr, "nothing on ep %#02x for %d s\n", ep,
                    DN_STALL_S);
            rc = 1;
            break;
        }
        if (t - last_print >= 1.0) {
            last_print = t;
            fprintf(stderr, "  %5.1f s  %8.2f MB  (%.2f MB/s)  ring %u "
                    "waiting\n", t - t0, (double)total / 1e6,
                    (double)total / (t - t0) / 1e6, used);
        }
    }
    l->quiet = 0;

    (void)call(l, FRQ1_CMD_STOP, 0, 0, NULL, 0, r);
    chipio_nap(250u);
    atomic_store(&rd.stop, 1);
    pthread_join(th, NULL);
    fclose(sink.out);
    total = atomic_load(&rd.bytes);

    printf("\n  frames    %u   %u pushed nothing, %u stopped on E_AGAIN "
           "(ring empty or endpoint busy), %u not synced\n", frames, empty, busy, unsynced);
    printf("  firmware  %lu byte(s) pushed; ring peak %u byte(s) waiting\n",
           pushed, peak);
    sink_summary(path, ep, &sink, total);
    if (atomic_load(&rd.failed))
        fprintf(stderr, "reader: %s\n", rd.err);
    return rc;
}

/* --- bench: how long until the TS of a channel arrives -------------------
 *
 * Four cases, each run `runs` times, alternating between two channels:
 *
 *   cold     STOP, then the init and tuning sequences, then STREAM
 *   retune   STOP, then the tuning sequence only, then STREAM
 *   zap      streaming one channel; the tuning sequence for the other with
 *            no STREAM frames in between, then STREAM again
 *   zap-i    the same, with a STREAM frame after every I2C transfer
 *
 * Every time is from t0, the first frame of the sequence, to:
 *   sent   the last frame of the tuning sequence answered
 *   lock   the demodulator's registers first say locked (polled every
 *          20 ms between STREAM frames)
 *   byte   the first byte on the bulk endpoint (cold, retune)
 *   PAT    the first PAT with a TSID other than the old channel's
 *   PMT    the first PMT start of that PAT's first programme
 *   old    the last PAT of the old channel the host read (zap: the
 *          backlog that still came out after t0)
 *   clean  the last resync or continuity jump, watching for 1 s after the
 *          PMT: from here on the stream had no errors
 */
#define BENCH_LIMIT_S   6.0
#define BENCH_NONE      0x10000u
#define BENCH_KINDS     4
#define BENCH_RUNS_MAX  50

static const char *const BENCH_KIND[BENCH_KINDS] = {
    "cold", "retune", "zap", "zap-i"
};

static void bench_mark(struct ts_sink *s, unsigned base)
{
    atomic_store(&s->base_tsid, base);
    atomic_store(&s->t_pat_old, 0.0);
    atomic_store(&s->t_pmt, 0.0);
    atomic_store(&s->pmt_pid, BENCH_NONE);
    atomic_store(&s->t_pat_new, 0.0);
    atomic_store(&s->t_byte, 0.0);
    atomic_store(&s->t_err, 0.0);
}

/* STREAM frames for `secs`. */
static int bench_stream(struct link *l, double secs, unsigned bursts,
                        unsigned ring)
{
    uint8_t r[64];
    double t0 = now_s();

    while (now_s() - t0 < secs)
        if (call(l, FRQ1_CMD_STREAM, bursts, ring, &g_outs, 1, r) != CALL_OK)
            return -1;
    return 0;
}

/* STREAM frames until the new PMT has come and the demodulator has said
 * locked.  -> 0, 1 when BENCH_LIMIT_S ran out, -1 when a frame failed. */
static int bench_follow(struct link *l, struct ts_sink *s, double t0,
                        unsigned bursts, unsigned ring, double *t_lock)
{
    uint8_t r[64], st[3];
    double next = 0.0, t;

    *t_lock = 0.0;
    for (;;) {
        t = now_s();
        if (t - t0 > BENCH_LIMIT_S)
            return 1;
        if (*t_lock == 0.0 && t >= next) {
            int lk = dn_poll(l, st);
            if (lk < 0)
                return -1;
            if (lk)
                *t_lock = now_s();
            next = t + 0.02;
        }
        if (*t_lock != 0.0 && atomic_load(&s->t_pmt) != 0.0)
            return 0;
        if (call(l, FRQ1_CMD_STREAM, bursts, ring, &g_outs, 1, r) != CALL_OK)
            return -1;
    }
}

static double ms(double t, double t0)
{
    return t == 0.0 ? -1.0 : (t - t0) * 1e3;
}

static void bench_cell(double v)
{
    if (v < 0.0)
        printf("     -");
    else
        printf(" %5.0f", v);
}

static int cmp_d(const void *a, const void *b)
{
    double x = *(const double *)a, y = *(const double *)b;

    return x < y ? -1 : x > y;
}

/* "median (min..max)" of the values that are there. */
static void bench_sum(const char *name, double *v, unsigned n)
{
    double k[BENCH_RUNS_MAX];
    unsigned i, m = 0;

    for (i = 0; i < n; i++)
        if (v[i] >= 0.0)
            k[m++] = v[i];
    if (m == 0)
        return;
    qsort(k, m, sizeof(k[0]), cmp_d);
    printf("  %-5s %5.0f ms  (%4.0f..%4.0f, %u run(s))\n", name,
           m % 2 ? k[m / 2] : (k[m / 2 - 1] + k[m / 2]) / 2.0, k[0],
           k[m - 1], m);
}

static int cmd_bench_dione(struct link *l, const char *a, const char *b,
                           unsigned runs, unsigned bursts, unsigned ring,
                           unsigned ep)
{
    static struct ts_sink sink;
    static double col[BENCH_KINDS][7][BENCH_RUNS_MAX];
    static const char *const COL[7] = {
        "sent", "lock", "byte", "PAT", "PMT", "old", "clean"
    };
    struct reader rd;
    pthread_t th;
    uint8_t r[64], st[3];
    unsigned k, i, c, fails = 0;
    int rc = 0, got;

    if (runs > BENCH_RUNS_MAX)
        runs = BENCH_RUNS_MAX;
    for (k = 0; k < BENCH_KINDS; k++)   /* a run cut short leaves "none" */
        for (c = 0; c < 7u; c++)
            for (i = 0; i < BENCH_RUNS_MAX; i++)
                col[k][c][i] = -1.0;
    if (chipio_claim(&l->io) != 0) {
        fprintf(stderr, "%s\n", l->io.err);
        return 1;
    }
    (void)call(l, FRQ1_CMD_STOP, 0, 0, NULL, 0, r);
    while (chipio_bulk(&l->io, ep, sink.buf, DN_CHUNK, 60u, &got) == 0 &&
           got > 0)
        ;
    memset(&sink, 0, sizeof(sink));
    memset(sink.cc, 0xff, sizeof(sink.cc));
    atomic_store(&sink.last_tsid, BENCH_NONE);
    bench_mark(&sink, BENCH_NONE);
    sink.out = fopen("/dev/null", "wb");
    if (sink.out == NULL)
        return 1;
    memset(&rd, 0, sizeof(rd));
    rd.io = &l->io;
    rd.ep = ep;
    rd.sink = &sink;
    atomic_init(&rd.stop, 0);
    atomic_init(&rd.bytes, 0ul);
    atomic_init(&rd.failed, 0);
    if (pthread_create(&th, NULL, reader_main, &rd) != 0) {
        fclose(sink.out);
        return 1;
    }
    l->quiet = 1;
    g_pump_bursts = bursts;
    g_pump_ring = ring;

    printf("bench %s <-> %s, %u run(s) a case, times in ms from the first "
           "frame of the sequence\n\n", a, b, runs);
    printf("case    from  to     sent  lock  byte   PAT   PMT   old clean  TSID\n");
    for (k = 0; k < BENCH_KINDS && rc == 0; k++) {
        int zap = k >= 2;

        /* Start every case on `a`, locked; the zaps also streaming. */
        if (dn_tune(l, dn_channel_khz(a), 3000u, 1, st) != 1) {
            fprintf(stderr, "%s did not lock\n", a);
            rc = 1;
            break;
        }
        for (i = 0; i < runs; i++) {
            const char *to = i % 2 ? a : b, *from = i % 2 ? b : a;
            double t0, sent, lock;
            int f;

            if (zap) {
                if (bench_stream(l, 1.0, bursts, ring) != 0) {
                    rc = 1;
                    break;
                }
                bench_mark(&sink, atomic_load(&sink.last_tsid));
            } else {
                (void)call(l, FRQ1_CMD_STOP, 0, 0, NULL, 0, r);
                chipio_nap(300u);
                bench_mark(&sink, BENCH_NONE);
            }
            t0 = now_s();
            if (k == 0 && m2tv_init(dn_xfer, l) != 0) {
                rc = 1;
                break;
            }
            g_pump = k == 3;
            f = m2tv_tune(dn_xfer, l, dn_channel_khz(to));
            g_pump = 0;
            if (f != 0) {
                rc = 1;
                break;
            }
            sent = now_s();
            f = bench_follow(l, &sink, t0, bursts, ring, &lock);
            if (f < 0) {
                rc = 1;
                break;
            }
            fails += (unsigned)f;
            if (bench_stream(l, 1.0, bursts, ring) != 0) {
                rc = 1;
                break;
            }
            col[k][0][i] = ms(sent, t0);
            col[k][1][i] = ms(lock, t0);
            col[k][2][i] = zap ? -1.0 : ms(atomic_load(&sink.t_byte), t0);
            col[k][3][i] = ms(atomic_load(&sink.t_pat_new), t0);
            col[k][4][i] = ms(atomic_load(&sink.t_pmt), t0);
            col[k][5][i] = zap ? ms(atomic_load(&sink.t_pat_old), t0) : -1.0;
            col[k][6][i] = ms(atomic_load(&sink.t_err), t0);
            printf("%-7s %-5s %-5s", BENCH_KIND[k], from, to);
            for (c = 0; c < 7u; c++)
                bench_cell(col[k][c][i]);
            printf("  %#06x%s\n", atomic_load(&sink.last_tsid),
                   f ? "  (ran out of time)" : "");
            fflush(stdout);
        }
    }
    l->quiet = 0;
    (void)call(l, FRQ1_CMD_STOP, 0, 0, NULL, 0, r);
    chipio_nap(250u);
    atomic_store(&rd.stop, 1);
    pthread_join(th, NULL);
    fclose(sink.out);

    for (k = 0; k < BENCH_KINDS; k++) {
        printf("\n%s\n", BENCH_KIND[k]);
        for (c = 0; c < 7u; c++)
            bench_sum(COL[c], col[k][c], runs);
    }
    printf("\nsync %lu resync(s), %lu cc jump(s) over the whole bench "
           "(each tune makes some)\n", sink.resyncs, sink.cc_jumps);
    if (fails)
        printf("%u run(s) ran out of time (%.0f s)\n", fails, BENCH_LIMIT_S);
    return rc;
}

/* --- main -------------------------------------------------------------- */

static const struct { const char *name; unsigned vid, pid; } KNOWN[] = {
    {"dt295", 0x06B8, 0x103B},      /* PIXELA PIX-DT295      MB8AC031    */
    {"m2tv",  0x04BB, 0x053A},      /* I-O DATA GV-M2TV      MB8AC018    */
};

static int parse_dev(const char *s, unsigned *vid, unsigned *pid)
{
    size_t i;

    for (i = 0; i < sizeof(KNOWN) / sizeof(KNOWN[0]); i++) {
        if (strcmp(s, KNOWN[i].name) == 0) {
            *vid = KNOWN[i].vid;
            *pid = KNOWN[i].pid;
            return 0;
        }
    }
    if (sscanf(s, "%x:%x", vid, pid) == 2)
        return 0;
    return -1;
}

static void usage(const char *me)
{
    fprintf(stderr,
        "usage: %s probe|stop [options]\n"
        "       %s save <out.ts> [options]\n"
        "       %s tune <ch> [options]              m2tv, dt295\n"
        "       %s scan [options]                   m2tv, dt295\n"
        "       %s bench --ch <a> --ch2 <b> [--runs n]\n"
        "                                      dione: time to the TS of a channel\n"
        "\n"
        "  --dev <vid:pid|dt295|m2tv>         default dt295\n"
        "  --staging <addr>                    default by device (dt295 0x003b0000,\n"
        "                                      m2tv 0x000c5000, others 0x000c4620)\n"
        "  --hook <addr>                       default by device (m2tv 0x000e9f80,\n"
        "                                      others 0x000ea6fc)\n"
        "  --force                             send even with no hook in place\n"
        "  save:  --seconds <n>  --bursts <1..%u>  --ep <0x81>\n"
        "         (--seconds 0: until SIGINT/SIGTERM or a closed pipe)\n"
        "         (selene: tune first; the endpoint is read in a thread;\n"
        "         --sync for the old one-frame-then-drain loop, which\n"
        "         needs --seconds above 0)\n"
        "         --src <0..3>    with --sync: demodulator index, default 0\n"
        "  save - writes the TS to stdout (progress goes to stderr)\n"
        "  scan:  --first/--last  UHF range, default 13..62 (m2tv, dt295)\n"
        "\n"
        "  on dt295, tuning is built in: <ch> is 13..50 (UHF, below\n"
        "  700 MHz), BS1..BS23[/TS<n>|/0x<TSID>], CS2..CS24; scan --sat\n"
        "  for BS / CS (instead of UHF).  The dish's LNB power is not ours.\n"
        "  save:  --ch <ch>\n"
        "\n"
        "  on m2tv (dione), tuning is built in: <ch> is 27 / U27 (UHF\n"
        "  13..50, below 700 MHz) or 557143k.\n"
        "  save:  --ch <ch>  --bursts <1..%u>  --ring <MiB 1..%u>\n"
        "         --ep 0x84         the TS on EP 0x84 instead of 0x81\n"
        "         --mirror <file>   the same TS on EP 0x84 too, into <file>\n",
        me, me, me, me, me, FRQ1_SR_BURSTS_MAX,
        FRQ1_DSR_BURSTS_MAX, FRQ1_DSR_RING_MAX);
}

int main(int argc, char **argv)
{
    struct link l;
    unsigned vid = 0x06B8, pid = 0x103B;   /* dt295 */
    unsigned staging = STAGING_DEFAULT;
    unsigned hook = HOOK_DEFAULT;
    unsigned first = 13, last = 62, src = 0;
    unsigned seconds = 10, bursts = BURSTS_DEFAULT, ep = EP_DEFAULT;
    unsigned ring = 0;
    const char *ch_spec = NULL, *ch2_spec = NULL;
    unsigned runs = 10;
    int sat = 0, is295;
    const char *out = NULL;
    const char *mirror = NULL;
    int sync_save = 0;
    const char *cmd;
    uint8_t stub[8];
    uint32_t entry = 0;
    int force = 0;
    int staging_set = 0, hook_set = 0, bursts_set = 0;
    int i, rc;

    if (argc < 2) {
        usage(argv[0]);
        return 2;
    }
    cmd = argv[1];

    for (i = 2; i < argc; i++) {
        const char *a = argv[i];
        const char *v = (i + 1 < argc) ? argv[i + 1] : NULL;

        if (a[0] != '-' || a[1] == '\0') {   /* "-" is stdout */
            if (out == NULL)
                out = a;
            continue;
        }
        if (strcmp(a, "--force") == 0) {
            force = 1;
            continue;
        }
        if (strcmp(a, "--sat") == 0) {
            sat = 1;
            continue;
        }
        if (strcmp(a, "--sync") == 0) {
            sync_save = 1;
            continue;
        }
        if (v == NULL) {
            fprintf(stderr, "%s needs a value\n", a);
            usage(argv[0]);
            return 2;
        }
        if (strcmp(a, "--dev") == 0) {
            if (parse_dev(v, &vid, &pid) != 0) {
                fprintf(stderr, "%s: not a device\n", v);
                return 2;
            }
        } else if (strcmp(a, "--staging") == 0) {
            staging = (unsigned)strtoul(v, NULL, 0);
            staging_set = 1;
        } else if (strcmp(a, "--hook") == 0) {
            hook = (unsigned)strtoul(v, NULL, 0);
            hook_set = 1;
        } else if (strcmp(a, "--src") == 0) {
            src = (unsigned)strtoul(v, NULL, 0) & 3u;
        } else if (strcmp(a, "--first") == 0) {
            first = (unsigned)strtoul(v, NULL, 0);
        } else if (strcmp(a, "--last") == 0) {
            last = (unsigned)strtoul(v, NULL, 0);
        } else if (strcmp(a, "--ch") == 0) {
            ch_spec = v;
        } else if (strcmp(a, "--ch2") == 0) {
            ch2_spec = v;
        } else if (strcmp(a, "--runs") == 0) {
            runs = (unsigned)strtoul(v, NULL, 0);
        } else if (strcmp(a, "--seconds") == 0) {
            seconds = (unsigned)strtoul(v, NULL, 0);
        } else if (strcmp(a, "--bursts") == 0) {
            bursts = (unsigned)strtoul(v, NULL, 0);
            bursts_set = 1;
        } else if (strcmp(a, "--ring") == 0) {
            ring = (unsigned)strtoul(v, NULL, 0);
        } else if (strcmp(a, "--ep") == 0) {
            ep = (unsigned)strtoul(v, NULL, 0);
        } else if (strcmp(a, "--mirror") == 0) {
            mirror = v;
        } else {
            fprintf(stderr, "unknown option %s\n", a);
            usage(argv[0]);
            return 2;
        }
        i++;
    }

    /* The command frame goes to a mailbox of the device's own where it has
     * one, and dione hooks a different loader address.  These follow the
     * device, default or --dev, unless the operator named them explicitly. */
    if (!staging_set && chipio_default_mbox(vid, pid))
        staging = chipio_default_mbox(vid, pid);
    if (chipio_is_dione(vid, pid) && !hook_set)
        hook = 0x000E9F80u;

    if (first < 13u || last > 62u || first > last) {
        fprintf(stderr, "the UHF channels are 13..62\n");
        return 2;
    }
    if (strcmp(cmd, "save") == 0 && out == NULL) {
        fprintf(stderr, "save needs a file to write\n");
        return 2;
    }
    /* dione's endpoint byte.  The firmware fills EP 0x81 or EP 0x84 (both
     * with --mirror); any other --ep is read as given, for selene. */
    if (ep == 0x84u)
        g_outs = FRQ1_DSO_EP84;
    if (mirror != NULL) {
        if (ep != 0x81u) {
            fprintf(stderr, "--mirror reads EP 0x84 beside EP 0x81: leave "
                    "--ep at 0x81\n");
            return 2;
        }
        g_outs = FRQ1_DSO_EP81 | FRQ1_DSO_EP84;
    }
    if (out != NULL && strcmp(cmd, "save") == 0 && strcmp(out, "-") == 0) {
        if (mirror != NULL && strcmp(mirror, "-") == 0) {
            fprintf(stderr, "only one stream can go to stdout\n");
            return 2;
        }
        /* The TS gets the real stdout; everything printed goes to stderr. */
        fflush(stdout);
        g_ts_fd = dup(1);
        if (g_ts_fd < 0 || dup2(2, 1) < 0) {
            fprintf(stderr, "cannot set stdout aside for the TS\n");
            return 1;
        }
    } else if (mirror != NULL && strcmp(mirror, "-") == 0) {
        fprintf(stderr, "--mirror -: put the main stream on stdout "
                "instead (save -)\n");
        return 2;
    }
    if (strcmp(cmd, "tune") == 0) {
        if (out == NULL) {
            fprintf(stderr, "tune needs a channel\n");
            return 2;
        }
        ch_spec = out;
    }
    is295 = vid == 0x06B8 && pid == 0x103B;
    if (ch_spec != NULL && is295) {
        int s_;
        unsigned k_, t_;
        if (dt295_channel(ch_spec, &s_, &k_, &t_) != 0) {
            fprintf(stderr, "%s: not a channel on the PIX-DT295 (13..50, "
                    "BS1..BS23[/TS<n>|/0x<TSID>], CS2..CS24)\n", ch_spec);
            return 2;
        }
    }
    if (ch_spec != NULL && chipio_is_dione(vid, pid) &&
        dn_channel_khz(ch_spec) == 0) {
        fprintf(stderr, "%s: not a channel (27, U27, or 557143k)\n",
                ch_spec);
        return 2;
    }

    memset(&l, 0, sizeof(l));
    l.staging = staging;
    if (chipio_open(&l.io, vid, pid) != 0) {
        fprintf(stderr, "%s\n", l.io.err);
        return 1;
    }

    /* Read before write: see hook_installed() above. */
    rc = hook_installed(&l.io, hook, &entry, stub);
    if (rc < 0) {
        fprintf(stderr, "%s\n", l.io.err);
        chipio_close(&l.io);
        return 1;
    }
    if (rc == 0) {
        fprintf(stderr,
            "the hook at 0x%08x does not hold a jump of ours:\n"
            "  %02x%02x%02x%02x%02x%02x%02x%02x\n"
            "No firmware of ours is in the path, so nothing was sent.  "
            "Install one:\n"
            "      ./frc1_load install ../build/frc1-<id>.frc\n"
            "If the hook lives somewhere else on this machine, say where "
            "with --hook;\n--force sends anyway, which writes 64 bytes to "
            "0x%08x on a chip whose\nloader this has not identified.\n",
            hook, stub[0], stub[1], stub[2], stub[3], stub[4], stub[5],
            stub[6], stub[7], staging);
        if (!force) {
            chipio_close(&l.io);
            return 1;
        }
    } else {
        printf("hook 0x%08x -> entry 0x%08x\n", hook, entry);
    }

    if (strcmp(cmd, "probe") == 0) {
        rc = cmd_probe(&l);
    } else if (strcmp(cmd, "tune") == 0 && l.io.dione) {
        rc = cmd_tune_dione(&l, ch_spec) == 0 ? 0 : 1;
    } else if (strcmp(cmd, "tune") == 0 && is295) {
        rc = cmd_tune_dt295(&l, ch_spec) == 0 ? 0 : 1;
    } else if (strcmp(cmd, "scan") == 0 && is295) {
        rc = cmd_scan_dt295(&l, first, last > 50u ? 50u : last, sat);
    } else if (strcmp(cmd, "save") == 0 && is295 && ch_spec) {
        rc = cmd_tune_dt295(&l, ch_spec);
        if (rc == 1) {
            printf("     (recording anyway: no lock)\n");
            rc = 0;
        }
        if (rc == 0)
            rc = sync_save ? cmd_save(&l, out, seconds, bursts, ep, src)
                           : cmd_save_selene(&l, out, seconds,
                                             bursts_set ? bursts : 0u, ep);
        else
            rc = 1;
    } else if (!l.io.dione && (strcmp(cmd, "tune") == 0 ||
                               strcmp(cmd, "scan") == 0 ||
                               strcmp(cmd, "bench") == 0 ||
                               (strcmp(cmd, "save") == 0 && ch_spec))) {
        fprintf(stderr, "%s: the firmware does not tune, and this program "
                "only knows the GV-M2TV's\n(--dev m2tv) and the PIX-DT295's (--dev dt295).  "
                "On other selene machines, tune with your own sequence\n"
                "through I2CXFER, then "
                "save without --ch.\n", strcmp(cmd, "save") == 0 ? "--ch" : cmd);
        rc = 2;
    } else if (strcmp(cmd, "scan") == 0) {
        rc = cmd_scan_dione(&l, first, last);
    } else if (strcmp(cmd, "bench") == 0 && l.io.dione) {
        if (ch_spec == NULL || ch2_spec == NULL ||
            dn_channel_khz(ch2_spec) == 0 || runs == 0) {
            fprintf(stderr, "bench needs --ch <a> --ch2 <b> (and --runs n)\n");
            rc = 2;
        } else {
            rc = cmd_bench_dione(&l, ch_spec, ch2_spec, runs,
                                 bursts_set ? bursts : 0u, ring, ep);
        }
    } else if (strcmp(cmd, "save") == 0 && l.io.dione) {
        /* 0 lets the firmware pick (64 bursts, short enough that the card
         * driver's frames get through during a recording). */
        rc = 0;
        if (ch_spec != NULL) {
            rc = cmd_tune_dione(&l, ch_spec);
            if (rc == 1) {
                printf("     (recording anyway: no lock)\n");
                rc = 0;
            } else if (rc < 0) {
                rc = 1;
            }
        }
        if (rc == 0)
            rc = cmd_save_dione(&l, out, seconds,
                                bursts_set ? bursts : 0u, ring, ep, mirror);
    } else if (strcmp(cmd, "save") == 0 && !sync_save) {
        rc = cmd_save_selene(&l, out, seconds, bursts_set ? bursts : 0u, ep);
    } else if (strcmp(cmd, "save") == 0) {
        rc = cmd_save(&l, out, seconds, bursts, ep, src);
    } else if (strcmp(cmd, "stop") == 0) {
        uint8_t r[64];

        rc = call(&l, FRQ1_CMD_STOP, 0, 0, NULL, 0, r);
        if (rc == CALL_SILENT)
            explain_silence(&l);
        rc = (rc == CALL_OK) ? 0 : 1;
    } else {
        usage(argv[0]);
        rc = 2;
    }

    chipio_close(&l.io);
    return rc;
}
