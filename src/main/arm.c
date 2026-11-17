/* arm.c -- the two ARM cores, from the main FR's side.  docs/arm.md.
 *
 * The chip has two ARM cores next to the FR (ARM-E and ARM-C).  This
 * module is the main FR's half of putting OUR program on one of them; the
 * other half is src/armcore/.
 *
 * WHAT IS KNOWN
 *   On a GV-M2TV (dione), measured with this module and src/armcore/
 *   (docs/arm.md):
 *   - 0x00F10118 = 0xFFFF wakes both control bands: CTL_EN reads 0x0001
 *   - CTL_EN bit0 stops the core; clearing it lets the core run.  CTL_RST
 *     alone does not stop a running core
 *   - ITCM takes word stores from the main FR but reads back 0 (write-only
 *     from here).  DTCM takes writes and reads back
 *   - stopped, CTL_RST = 0x0FFF, image written to ITCM, then CTL_RST =
 *     0x0104 and CTL_EN &= ~3: the core runs our image from ITCM 0 (the
 *     mailbox says 'ARM1' and answers requests), and DTCM stays readable
 *     while it runs
 *   - the same on ARM-C, through its own window (0x00C00000); both cores
 *     run at once
 *   The other order, CTL_EN = 7 then CTL_RST = 0, leaves bit0 set, and it
 *   never started a core of ours.  0x00FD0038/0x00FD0048 are taken to be
 *   ARM-E's interrupt status and enable towards the FR, not a clock gate
 *   (not confirmed on hardware).
 *
 * WHAT IS NOT
 *   - What the other CTL_RST bits do (only 0x0FFF and 0x0104 are used)
 *   - Whether all of it works on selene.  On a PIX-DT295 the band wakes
 *     and our image loads into ITCM and runs (arm_selene.c); the rest of
 *     the window, CTL_EN, CTL_RST and the start and stop orders is assumed
 *     to be the same as on dione.  Only waking the band differs.
 *   - ARM -> FR interrupts.  FR -> ARM works by polling (a write to
 *     win + 0x50 shows in the ARM's 0x00E70034), but what raises
 *     0x00FD0038 from the ARM side is not known (docs/arm.md).
 *
 * SHAPE
 *   This file is the part both generations share.  Waking the band, and
 *   reading the register that wakes it, are per generation: arm_dione.c
 *   and arm_selene.c, picked with fw_is_dione() as stream.c does.
 *
 *   The host walks it: STATUS (reads only), HOLD, LOAD a few words at a
 *   time, RUN, then STATUS until ALIVE.  No command waits.  The mailbox
 *   (armcore.h) is read through the window, so STATUS also says whether
 *   our ARM image ran.
 */
#include "fw.h"
#include "mb86m01.h"
#include "armcore.h"

#define REPORT_VERSION  1u
#define LOAD_MAX        32u     /* bytes per ARM_LOAD; fits the data area */
#define NO_MISMATCH     0xffu

/* STATUS flags (report byte 2) */
#define ST_AWAKE        0x01u   /* CTL_EN or CTL_RST reads non-zero       */
#define ST_HELD         0x02u   /* stopped (CTL_EN bit0) and CTL_RST held */
#define ST_ALIVE        0x04u   /* the mailbox says our image ran         */
#define ST_FAULT        0x08u   /* the mailbox has a fault recorded       */
#define ST_RUNNING      0x10u   /* CTL_EN bit0 clear: the core runs       */

static unsigned window(unsigned core)
{
    return core == ARM_CORE_E ? ARM_E_WIN : ARM_C_WIN;
}

static unsigned rd16(unsigned a)
{
    return MMIO16(a) & 0xffffu;
}

/* The register that wakes the bands, for the report. */
static unsigned gate_get(void)
{
    return fw_is_dione() ? dione_arm_gate() : selene_arm_gate();
}

static void wake(unsigned win)
{
    if (fw_is_dione())
        dione_arm_wake(win);
    else
        selene_arm_wake(win);
}

static int held(unsigned win)
{
    return (rd16(win + ARM_CTL_EN_OFF) & ARM_EN_STOP)
        && (rd16(win + ARM_CTL_RST_OFF) & ARM_RST_HOLD) == ARM_RST_HOLD;
}

/* ARM-side address -> offset in the main FR's window, or -1. */
static int win_off(unsigned arm, unsigned n)
{
    if (arm < ARM_ITCM + ARM_ITCM_SIZE && n <= ARM_ITCM + ARM_ITCM_SIZE - arm)
        return (int)(ARM_ITCM_OFF + (arm - ARM_ITCM));
    if (arm >= ARM_DTCM && arm < ARM_DTCM + ARM_DTCM_SIZE
        && n <= ARM_DTCM + ARM_DTCM_SIZE - arm)
        return (int)(ARM_DTCM_OFF + (arm - ARM_DTCM));
    return -1;
}

/* The common checks for every command of the 0x60 band (arm_ipc.c uses it
 * too).  -> the window, or 0 with *rc set. */
unsigned arm_pick(frame_t f, int *rc)
{
    unsigned core = frq1_be32(f, FR_ARG0) & 0xffu;

    *rc = FW_OK;
    if (core >= ARM_CORES) {
        *rc = FW_EINVAL;
        return 0;
    }
    return window(core);
}

/* The STATUS report, also written by HOLD.
 *
 *   +0  version, core, flags (ST_*), 0
 *   +4  0x00F10118 (arm_dione.c / arm_selene.c say how it is read)
 *   +8  CTL_EN << 16 | CTL_RST
 *   +12 CTL58 << 16 | CTL_404
 *   +16 mailbox: ALIVE, LOOPS, FAULT, FAULT_LR, ACK_SEQ, ACK_VAL
 */
static void report(frame_t f, unsigned win)
{
    unsigned mb = win + ARM_MBOX_OFF;
    unsigned en = rd16(win + ARM_CTL_EN_OFF);
    unsigned rst = rd16(win + ARM_CTL_RST_OFF);
    unsigned alive = MMIO(mb + ARM_MB_ALIVE);
    unsigned fault = MMIO(mb + ARM_MB_FAULT);
    unsigned flags = 0;

    /* CTL_EN reads 0 while the core runs, so either register says awake. */
    if (en | rst)
        flags |= ST_AWAKE;
    if ((en & ARM_EN_STOP) && (rst & ARM_RST_HOLD) == ARM_RST_HOLD)
        flags |= ST_HELD;
    if ((en | rst) && !(en & ARM_EN_STOP))
        flags |= ST_RUNNING;
    if (alive == ARM_ALIVE_MAGIC)
        flags |= ST_ALIVE;
    if ((fault & 0xff000000u) == ARM_FAULT_BASE)
        flags |= ST_FAULT;

    f[FR_DATA + 0] = (unsigned char)REPORT_VERSION;
    f[FR_DATA + 1] = (unsigned char)(win == ARM_E_WIN ? ARM_CORE_E : ARM_CORE_C);
    f[FR_DATA + 2] = (unsigned char)flags;
    f[FR_DATA + 3] = 0;
    frq1_put32(f, FR_DATA + 4, gate_get());
    frq1_put32(f, FR_DATA + 8, (en << 16) | rst);
    frq1_put32(f, FR_DATA + 12, (rd16(win + ARM_CTL58_OFF) << 16)
                                | rd16(win + ARM_CTL_404_OFF));
    frq1_put32(f, FR_DATA + 16, alive);
    frq1_put32(f, FR_DATA + 20, MMIO(mb + ARM_MB_LOOPS));
    frq1_put32(f, FR_DATA + 24, fault);
    frq1_put32(f, FR_DATA + 28, MMIO(mb + ARM_MB_FAULT_LR));
    frq1_put32(f, FR_DATA + 32, MMIO(mb + ARM_MB_ACK_SEQ));
    frq1_put32(f, FR_DATA + 36, MMIO(mb + ARM_MB_ACK_VAL));
}

/* ARM_STATUS (0x60).  arg0 = core.  Reads only. */
int arm_status(frame_t f)
{
    int rc;
    unsigned win = arm_pick(f, &rc);

    if (rc)
        return rc;
    report(f, win);
    return FW_OK;
}

/* ARM_HOLD (0x61).  arg0 = core.
 *
 * Wake the band (arm_dione.c / arm_selene.c), stop the core (CTL_EN bit0) and hold it in reset:
 * the state ARM_LOAD needs, and the safe state to leave a core in.  Stop
 * first: CTL_RST alone does not stop a running core.  On dione 0x00F10118 =
 * 0xFFFF is the value ts_dione.c already uses for the TS path, so it is not
 * a new state for the chip.  FW_EIO if the core does not read back as held
 * -- on selene that is also what a band that did not wake looks like. */
int arm_hold(frame_t f)
{
    int rc;
    unsigned win = arm_pick(f, &rc);

    if (rc)
        return rc;
    wake(win);
    MMIO16(win + ARM_CTL_EN_OFF) =
        (unsigned short)(rd16(win + ARM_CTL_EN_OFF) | ARM_EN_STOP);
    MMIO16(win + ARM_CTL_RST_OFF) = 0xffffu;
    arm_ipc_reset(win);         /* a stopped core does not listen */
    report(f, win);
    return held(win) ? FW_OK : FW_EIO;
}

/* ARM_LOAD (0x62).  arg0 = core | bytes << 8, arg1 = ARM-side address,
 * data = the words, big-endian.
 *
 * bytes is 4..LOAD_MAX in steps of 4, the address word aligned and inside
 * ITCM or DTCM as the core sees them (armcore.h).  The core must be held
 * (ARM_HOLD first, else FW_EAGAIN).  Writes one word at a time, then reads
 * DTCM back.  ITCM is write-only from the main FR (it reads 0), so ITCM
 * words are not read back; only running the image tells.
 *
 *   +0  version, core, words written, first word that read back wrong
 *       (NO_MISMATCH if none)
 *   +4  ARM-side address
 *   +8  what that word read back as (0 if none)
 *
 * FW_EIO when a word did not read back. */
int arm_load(frame_t f)
{
    int rc, off;
    unsigned arg = frq1_be32(f, FR_ARG0);
    unsigned addr = frq1_be32(f, FR_ARG1);
    unsigned n = (arg >> 8) & 0xffu;
    unsigned win = arm_pick(f, &rc);
    unsigned i, bad = NO_MISMATCH, got = 0;
    unsigned w[LOAD_MAX / 4u];

    if (rc)
        return rc;
    if (n == 0u || n > LOAD_MAX || (n & 3u) || (addr & 3u))
        return FW_EINVAL;
    off = win_off(addr, n);
    if (off < 0)
        return FW_EINVAL;
    if (!held(win))
        return FW_EAGAIN;

    for (i = 0; i < n / 4u; i++)
        w[i] = frq1_be32(f, (int)(FR_DATA + i * 4u));
    for (i = 0; i < n / 4u; i++)
        MMIO(win + (unsigned)off + i * 4u) = w[i];
    for (i = 0; addr >= ARM_DTCM && i < n / 4u; i++) {
        unsigned v = MMIO(win + (unsigned)off + i * 4u);

        if (v != w[i]) {
            bad = i;
            got = v;
            break;
        }
    }

    f[FR_DATA + 0] = (unsigned char)REPORT_VERSION;
    f[FR_DATA + 1] = (unsigned char)(win == ARM_E_WIN ? ARM_CORE_E : ARM_CORE_C);
    f[FR_DATA + 2] = (unsigned char)(n / 4u);
    f[FR_DATA + 3] = (unsigned char)bad;
    frq1_put32(f, FR_DATA + 4, addr);
    frq1_put32(f, FR_DATA + 8, got);
    return bad == NO_MISMATCH ? FW_OK : FW_EIO;
}

/* ARM_RUN (0x63).  arg0 = core.
 *
 * Start the image in ITCM from its reset vector, the way it was measured to
 * run: the core must be held (ARM_HOLD, then ARM_LOAD; else FW_EAGAIN).
 * Clears the mailbox's ALIVE and FAULT and the work queue's header first,
 * since DTCM keeps old contents, then CTL_RST = ARM_RST_RUN and CTL_EN &= ~3.  Does not wait:
 * the host reads ARM_STATUS until ST_ALIVE (or ST_FAULT).  The report is
 * the STATUS report, taken right after the start. */
int arm_run(frame_t f)
{
    int rc;
    unsigned win = arm_pick(f, &rc);
    unsigned mb;

    if (rc)
        return rc;
    if (!held(win))
        return FW_EAGAIN;
    mb = win + ARM_MBOX_OFF;
    MMIO(mb + ARM_MB_ALIVE) = 0;
    MMIO(mb + ARM_MB_FAULT) = 0;
    arm_ipc_reset(win);
    MMIO16(win + ARM_CTL_RST_OFF) = ARM_RST_RUN;
    MMIO16(win + ARM_CTL_EN_OFF) = (unsigned short)(rd16(win + ARM_CTL_EN_OFF) & ~3u);
    report(f, win);
    return FW_OK;
}
