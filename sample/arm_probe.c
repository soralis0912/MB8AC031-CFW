/* arm_probe.c -- the host side of the 0x60 band: the ARM cores.
 *
 *     ./arm_probe status [--core c]          read only
 *     ./arm_probe hold                       wake the band, hold ARM-E
 *     ./arm_probe load <arm addr> <word>...  up to 8 words, read back
 *     ./arm_probe image <file> [--at addr] [--blind]
 *                                            a whole image, 32 bytes a time
 *     ./arm_probe run                        ARM_RUN, then wait for 'ARM1'
 *     ./arm_probe ask <value>                one mailbox request (~value)
 *     ./arm_probe rd <addr> [--w 32|16|8]    the ARM reads an address
 *     ./arm_probe wr <addr> <val> [--w ..]   the ARM writes and reads back
 *     ./arm_probe cp15                       the ARM's ID, cache, TCM regs
 *     ./arm_probe start                      a fixed CTL write order
 *
 * --dev picks the machine (cfwdev.h): m2tv (dione, the default) or dt295
 * (selene).  The
 * addresses given to `load` and `image` are the ARM core's own (ITCM at 0,
 * DTCM at 0x00100000); the firmware maps them to its window.  ../docs/arm.md
 * has the layout and the order to try things in.
 *
 * --blind keeps going when a word does not read back.  A word that reads
 * back 0 from ITCM may still be there for the core; only running it
 * tells.
 *
 * `run` is the way to start our image: hold, image, run.  ARM_RUN does
 * CTL_RST = 0x0104 and CTL_EN &= ~3 (measured to start the core from
 * ITCM 0), and this program reads ARM_STATUS until ALIVE = 'ARM1'.
 *
 * `start` writes a fixed order with half-word POKEs (CTL_RST = 0x0FFF,
 * CTL58 = 3, 1, CTL_EN = 7, CTL_RST = 0).  Measured: it does not start a
 * core, because CTL_EN bit0 stays set and that bit stops the core.  It is
 * kept to repeat that measurement.
 *
 * `ask` writes REQ_ARG and then a new REQ_SEQ through POKE and reads the
 * answer: our image answers with the bitwise NOT of the argument.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "chipio.h"
#include "frq1.h"
#include "cfwdev.h"

/* The machine (--dev, default m2tv).  The ARM tools have mailbox 2 of 4
 * (cfwdev.h), so ts_grab (mailbox 0) can run at once. */
static const struct cfwdev *g_dev;
#define STAGING     (g_dev->arm_box)
#define HOOK        (g_dev->hook)

#define CTL58       0x58000u
#define CTL_EN      0x70000u
#define CTL_RST     0x70004u
#define MBOX_ARM    0x0010ff00u     /* the mailbox, as the core sees it  */
#define LOAD_MAX    32u
#define ALIVE_MAGIC 0x41524d31u
/* The main FR's view of the chosen core: ARM-E 0x00E00000, ARM-C 0x00C00000. */
static unsigned g_core;
#define WIN         (g_core ? 0x00c00000u : 0x00e00000u)
#define MBOX_MAIN   (WIN + 0x8ff00u)    /* the mailbox, from the main FR     */

struct link {
    struct chipio io;
    uint32_t seq;
};

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

static void show_status(const uint8_t *r)
{
    const uint8_t *d = r + FRQ1_FR_DATA;
    unsigned fl = d[2];
    uint32_t enrst = frq1_get32(d, 8), c58 = frq1_get32(d, 12);
    uint32_t alive = frq1_get32(d, 16);

    printf("     report v%u  core %s  flags 0x%02x%s%s%s%s%s\n", d[0],
           d[1] == 0 ? "ARM-E" : "ARM-C", fl,
           fl & 1u ? " awake" : "", fl & 2u ? " held" : "",
           fl & 0x10u ? " running" : "",
           fl & 4u ? " ALIVE" : "", fl & 8u ? " FAULT" : "");
    printf("     0x00F10118 hi   %04x\n", (unsigned)frq1_get32(d, 4));
    printf("     CTL_EN %04x  CTL_RST %04x  CTL58 %04x  +0x404 %04x\n",
           (unsigned)(enrst >> 16), (unsigned)(enrst & 0xffffu),
           (unsigned)(c58 >> 16), (unsigned)(c58 & 0xffffu));
    printf("     mailbox  alive %08x%s  loops %08x\n", (unsigned)alive,
           alive == ALIVE_MAGIC ? " ('ARM1')" : "",
           (unsigned)frq1_get32(d, 20));
    printf("              fault %08x  lr %08x  ack %08x/%08x\n",
           (unsigned)frq1_get32(d, 24), (unsigned)frq1_get32(d, 28),
           (unsigned)frq1_get32(d, 32), (unsigned)frq1_get32(d, 36));
}

static int simple(struct link *l, uint32_t cmd, unsigned core)
{
    uint8_t r[64];
    int32_t st;

    if (call(l, cmd, core, 0, NULL, 0, r, &st) != 0)
        return 1;
    printf("  %-11s status %s\n", frq1_command_name(cmd),
           frq1_status_name(st));
    if (st == FRQ1_ST_OK || st == FRQ1_E_IO)
        show_status(r);
    return st == FRQ1_ST_OK ? 0 : 1;
}

/* One ARM_LOAD.  Prints only on failure unless verbose. */
static int load(struct link *l, uint32_t addr, const uint8_t *bytes,
                unsigned n, int verbose)
{
    uint8_t r[64];
    const uint8_t *d = r + FRQ1_FR_DATA;
    int32_t st;

    if (call(l, FRQ1_CMD_ARM_LOAD, g_core | (uint32_t)n << 8, addr, bytes, n, r,
             &st) != 0)
        return 1;
    if (verbose || st != FRQ1_ST_OK)
        printf("  ARM_LOAD    0x%08x  %u bytes  status %s\n", (unsigned)addr,
               n, frq1_status_name(st));
    if (st == FRQ1_ST_OK || st == FRQ1_E_IO) {
        if (d[3] != 0xffu)
            printf("     word %u (0x%08x) read back %08x\n", d[3],
                   (unsigned)(addr + d[3] * 4u), (unsigned)frq1_get32(d, 8));
        else if (verbose)
            printf("     %u words read back as written\n", d[2]);
    }
    return st == FRQ1_ST_OK ? 0 : 1;
}

static int cmd_image(struct link *l, const char *path, uint32_t at,
                     int blind)
{
    uint8_t buf[0x10000];
    size_t n, off;
    unsigned bad = 0;
    FILE *f = fopen(path, "rb");

    if (f == NULL) {
        perror(path);
        return 1;
    }
    n = fread(buf, 1, sizeof buf, f);
    fclose(f);
    while (n & 3u)
        buf[n++] = 0;
    printf("  %s: %zu bytes to 0x%08x\n", path, n, (unsigned)at);
    for (off = 0; off < n; off += LOAD_MAX) {
        unsigned chunk = (unsigned)(n - off < LOAD_MAX ? n - off : LOAD_MAX);

        if (load(l, at + (uint32_t)off, buf + off, chunk, 0) != 0) {
            bad++;
            if (!blind) {
                printf("  stopped at +0x%zx\n", off);
                return 1;
            }
        }
    }
    if (bad)
        printf("  written blind: %u of %zu chunks did not read back\n", bad,
               (n + LOAD_MAX - 1u) / LOAD_MAX);
    else
        printf("  all %zu bytes read back as written\n", n);
    return bad ? 1 : 0;
}

static int poke16(struct link *l, uint32_t addr, uint32_t v)
{
    uint8_t data[4], r[64];
    const uint8_t *d = r + FRQ1_FR_DATA;
    int32_t st;

    frq1_put32(data, 0, v);
    if (call(l, FRQ1_CMD_POKE, addr, FRQ1_POKE_CTL(FRQ1_W_HALF, 0u), data,
             sizeof data, r, &st) != 0)
        return 1;
    printf("  POKE half 0x%08x = %04x  was %04x  reads %04x  %s\n",
           (unsigned)addr, (unsigned)v,
           (unsigned)frq1_get32(d, FRQ1_PO_OLD) & 0xffffu,
           (unsigned)frq1_get32(d, FRQ1_PO_READBACK) & 0xffffu,
           frq1_status_name(st));
    return st == FRQ1_ST_OK ? 0 : 1;
}

static int poke32(struct link *l, uint32_t addr, uint32_t v)
{
    uint8_t data[4], r[64];
    int32_t st;

    frq1_put32(data, 0, v);
    if (call(l, FRQ1_CMD_POKE, addr, FRQ1_POKE_CTL(FRQ1_W_WORD, 0u), data,
             sizeof data, r, &st) != 0)
        return 1;
    return st == FRQ1_ST_OK ? 0 : 1;
}

static int cmd_run(struct link *l)
{
    uint8_t r[64];
    int32_t st;
    int k;

    if (call(l, FRQ1_CMD_ARM_RUN, g_core, 0, NULL, 0, r, &st) != 0)
        return 1;
    printf("  ARM_RUN     status %s\n", frq1_status_name(st));
    if (st != FRQ1_ST_OK)
        return 1;
    show_status(r);
    for (k = 0; k < 20; k++) {
        const uint8_t *d = r + FRQ1_FR_DATA;

        chipio_nap(50);
        if (call(l, FRQ1_CMD_ARM_STATUS, g_core, 0, NULL, 0, r, &st) != 0)
            return 1;
        if (d[2] & 0x0cu) {
            printf("  after %d ms:\n", (k + 1) * 50);
            show_status(r);
            return (d[2] & 0x04u) ? 0 : 1;
        }
    }
    printf("  no 'ARM1' after 1 s\n");
    show_status(r);
    return 1;
}

static int peek32(struct link *l, uint32_t addr, unsigned n, uint32_t *out)
{
    uint8_t r[64];
    const uint8_t *d = r + FRQ1_FR_DATA;
    int32_t st;
    unsigned i;

    if (call(l, FRQ1_CMD_PEEK, addr, FRQ1_PEEK_CTL(FRQ1_W_WORD, n), NULL, 0,
             r, &st) != 0 || st != FRQ1_ST_OK)
        return 1;
    for (i = 0; i < n; i++)
        out[i] = frq1_get32(d, FRQ1_PK_VALUES + i * 4u);
    return 0;
}

/* One mailbox request.  -> 0 answered (val, st set), 1 not. */
static int request(struct link *l, uint32_t op, uint32_t arg, uint32_t val,
                   uint32_t *ack_val, uint32_t *ack_st)
{
    uint32_t m[5], seq;
    int k;

    /* ACK_SEQ (+0x18), ACK_VAL, REQ_OP, REQ_VAL, ACK_ST (+0x28) */
    if (peek32(l, MBOX_MAIN + 0x18u, 5, m))
        return 1;
    seq = m[0] + 1u;
    if (poke32(l, MBOX_MAIN + 0x20u, op) || poke32(l, MBOX_MAIN + 0x14u, arg)
        || poke32(l, MBOX_MAIN + 0x24u, val)
        || poke32(l, MBOX_MAIN + 0x10u, seq))
        return 1;
    for (k = 0; k < 20; k++) {
        chipio_nap(10);
        if (peek32(l, MBOX_MAIN + 0x18u, 5, m))
            return 1;
        if (m[0] == seq) {
            *ack_val = m[1];
            *ack_st = m[4];
            return 0;
        }
    }
    printf("  request %08x not answered\n", (unsigned)seq);
    return 1;
}

static const char *st_name(uint32_t st)
{
    return st == 0 ? "ok" : st == 1 ? "DATA ABORT" : st == 2 ? "bad op" : "?";
}

static int show_abort(struct link *l)
{
    uint32_t m[2];

    if (peek32(l, MBOX_MAIN + 0x34u, 2, m))
        return 1;
    printf("     FAR %08x  FSR %08x (status %x, domain %x)\n", (unsigned)m[0],
           (unsigned)m[1], (unsigned)(m[1] & 0xfu),
           (unsigned)((m[1] >> 4) & 0xfu));
    return 0;
}

static int cmd_ask(struct link *l, uint32_t arg)
{
    uint32_t v, st;

    if (request(l, 0, arg, 0, &v, &st))
        return 1;
    printf("  NOT %08x -> %08x  %s\n", (unsigned)arg, (unsigned)v,
           v == ~arg ? "(= ~arg, ok)" : "(NOT ~arg)");
    return v == ~arg ? 0 : 1;
}

/* op: 1 rd32, 2 wr32, 3 rd16, 4 wr16, 5 rd8, 6 wr8 */
static int cmd_rw(struct link *l, int write, uint32_t addr, uint32_t val,
                  unsigned w)
{
    uint32_t op = (w == 32 ? 1u : w == 16 ? 3u : 5u) + (write ? 1u : 0u);
    uint32_t v, st;

    if (request(l, op, addr, val, &v, &st))
        return 1;
    if (write)
        printf("  ARM wr%-2u %08x = %08x  -> reads %08x  %s\n", w,
               (unsigned)addr, (unsigned)val, (unsigned)v, st_name(st));
    else
        printf("  ARM rd%-2u %08x  -> %08x  %s\n", w, (unsigned)addr,
               (unsigned)v, st_name(st));
    if (st == 1)
        show_abort(l);
    return st == 0 ? 0 : 1;
}

static int cmd_cp15(struct link *l)
{
    static const char *const name[] = {
        "c0,c0,0 main ID", "c0,c0,1 cache type", "c0,c0,2 TCM status",
        "c1,c0,0 control", "c9,c1,0 DTCM region", "c9,c1,1 ITCM region"
    };
    uint32_t i, v, st;

    for (i = 0; i < 6; i++) {
        if (request(l, 7, i, 0, &v, &st))
            return 1;
        printf("  %-22s %08x\n", name[i], (unsigned)v);
    }
    return 0;
}

static int cmd_start(struct link *l)
{
    static const uint8_t zero[4] = {0, 0, 0, 0};
    static const struct { uint32_t off, v; } order[] = {
        {CTL_RST, 0x0fffu}, {CTL58, 3u}, {CTL58, 1u}, {CTL_EN, 7u},
        {CTL_RST, 0u}
    };
    size_t i;
    int k;

    if (load(l, MBOX_ARM, zero, 4, 1) != 0)
        return 1;
    for (i = 0; i < sizeof order / sizeof order[0]; i++)
        if (poke16(l, WIN + order[i].off, order[i].v) != 0)
            return 1;
    for (k = 0; k < 3; k++) {
        chipio_nap(100);
        if (simple(l, FRQ1_CMD_ARM_STATUS, g_core) != 0)
            return 1;
    }
    return 0;
}

static void usage(const char *me)
{
    fprintf(stderr,
        "usage: %s <command> [--core 0|1] [--dev m2tv|dt295]   0 = ARM-E (default), 1 = ARM-C\n"
        "       %s status\n"
        "       %s hold\n"
        "       %s load <arm addr> <word> [word...]   up to 8 words\n"
        "       %s image <file> [--at <arm addr>] [--blind]  default 0 (ITCM)\n"
        "       %s run                                 ARM_RUN, wait for 'ARM1'\n"
        "       %s ask <value>                         one mailbox request\n"
        "       %s rd <addr> [--w 32|16|8]             the ARM reads\n"
        "       %s wr <addr> <val> [--w 32|16|8]       the ARM writes\n"
        "       %s cp15                                ARM CP15 ID/TCM regs\n"
        "       %s start                               fixed CTL write order\n"
        "--dev m2tv (default, 04bb:053a) or dt295 (06b8:103b).  A firmware of\n"
        "ours has to be on the chip:\n"
        "  ./frc1_load install ../build/frc1-m2tv.frc\n",
        me, me, me, me, me, me, me, me, me, me, me);
}

int main(int argc, char **argv)
{
    struct link l;
    const char *cmd, *pos[9];
    unsigned npos = 0, core = 0;
    uint32_t at = 0;
    uint8_t stub[8];
    int i, rc, blind = 0;
    unsigned w = 32;

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
        if (strcmp(argv[i], "--blind") == 0) {
            blind = 1;
        } else if (strcmp(argv[i], "--core") == 0 && i + 1 < argc) {
            core = (unsigned)strtoul(argv[++i], NULL, 0);
        } else if (strcmp(argv[i], "--w") == 0 && i + 1 < argc) {
            w = (unsigned)strtoul(argv[++i], NULL, 0);
        } else if (strcmp(argv[i], "--at") == 0 && i + 1 < argc) {
            at = (uint32_t)strtoul(argv[++i], NULL, 0);
        } else if (argv[i][0] != '-' && npos < 9) {
            pos[npos++] = argv[i];
        } else {
            usage(argv[0]);
            return 2;
        }
    }
    if (!(strcmp(cmd, "status") == 0 || strcmp(cmd, "hold") == 0 ||
          (strcmp(cmd, "load") == 0 && npos >= 2) ||
          (strcmp(cmd, "image") == 0 && npos == 1) ||
          strcmp(cmd, "start") == 0 || strcmp(cmd, "run") == 0 ||
          (strcmp(cmd, "ask") == 0 && npos == 1) ||
          (strcmp(cmd, "rd") == 0 && npos == 1) ||
          (strcmp(cmd, "wr") == 0 && npos == 2) ||
          strcmp(cmd, "cp15") == 0) || (w != 32 && w != 16 && w != 8)) {
        usage(argv[0]);
        return 2;
    }

    g_core = core;
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

    if (strcmp(cmd, "status") == 0) {
        rc = simple(&l, FRQ1_CMD_ARM_STATUS, core);
    } else if (strcmp(cmd, "hold") == 0) {
        rc = simple(&l, FRQ1_CMD_ARM_HOLD, core);
    } else if (strcmp(cmd, "load") == 0) {
        uint8_t data[LOAD_MAX];
        unsigned k, n = npos - 1u;

        if (n > LOAD_MAX / 4u)
            n = LOAD_MAX / 4u;
        for (k = 0; k < n; k++)
            frq1_put32(data, k * 4u, (uint32_t)strtoul(pos[k + 1], NULL, 0));
        rc = load(&l, (uint32_t)strtoul(pos[0], NULL, 0), data, n * 4u, 1);
    } else if (strcmp(cmd, "image") == 0) {
        rc = cmd_image(&l, pos[0], at, blind);
    } else if (strcmp(cmd, "run") == 0) {
        rc = cmd_run(&l);
    } else if (strcmp(cmd, "rd") == 0 || strcmp(cmd, "wr") == 0) {
        rc = cmd_rw(&l, cmd[0] == 'w', (uint32_t)strtoul(pos[0], NULL, 0),
                    npos > 1 ? (uint32_t)strtoul(pos[1], NULL, 0) : 0u, w);
    } else if (strcmp(cmd, "cp15") == 0) {
        rc = cmd_cp15(&l);
    } else if (strcmp(cmd, "ask") == 0) {
        rc = cmd_ask(&l, (uint32_t)strtoul(pos[0], NULL, 0));
    } else {
        rc = cmd_start(&l);
    }
    chipio_close(&l.io);
    return rc;
}
