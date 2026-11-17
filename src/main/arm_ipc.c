/* arm_ipc.c -- the main FR's end of the work queue to our ARM image.
 * src/shared/armipc.h is the wire; src/armcore/README.md the design.
 *
 * WHY THE MAIN FR ONLY RELAYS.  This firmware has no loop of its own: it
 * runs while the host's frame is answered and then returns.  So it cannot
 * wait for anything.  The ARM can.  Here the main FR does the two quick
 * halves -- put a request in, take a reply out -- and the ARM does the
 * waiting in between (src/armcore/svc_watch.c).
 *
 * The main FR is the producer of the request ring and the consumer of the
 * reply ring: it writes REQ_HEAD and REP_TAIL and only reads the others.
 * Every access is a whole word, through the window's DTCM band.
 *
 * WHAT IS NOT KNOWN
 *   - Whether it works on hardware.  The window and DTCM it uses were
 *     measured (docs/arm.md); the queue itself has only run in the
 *     emulator (tests/test_arm.py).
 */
#include "fw.h"
#include "frq1.h"
#include "mb86m01.h"
#include "armipc.h"

#define REPORT_VERSION  1u
#define PUT_MAX         40u     /* the frame's data area */

/* Which core has the card block: 0 none, else ARM_CORE_* + 1.  Set when a
 * CARD ON is queued, cleared when a CARD OFF is queued or that core is
 * stopped or restarted (arm_ipc_reset).  Kept here, not read from the
 * ARM: the main FR sees every request go by, and card.c must be able to
 * ask without touching a window whose band may be asleep. */
static unsigned char card_core;

int arm_has_card(void)
{
    return card_core != 0u;
}

static unsigned base(unsigned win)
{
    return win + ARM_IPC_OFF;
}

static unsigned core_of(unsigned win)
{
    return win == ARM_E_WIN ? ARM_CORE_E : ARM_CORE_C;
}

/* Zero the header, so both sides start at count 0 and the ARM's MAGIC is
 * gone until it writes it again.  arm.c calls this with the core stopped
 * (ARM_HOLD) or about to start (ARM_RUN). */
void arm_ipc_reset(unsigned win)
{
    unsigned i;

    for (i = 0; i < IPC_HEADER_WORDS; i++)
        MMIO(base(win) + i * 4u) = 0;
    if (card_core == core_of(win) + 1u)
        card_core = 0;          /* the image that had it is gone */
}

/* SVC_CARD requests change who has the card.  -> FW_OK, or FW_EBUSY if the
 * block is someone else's: card.c has it up, or the other core has it. */
static int card_owner(unsigned win, unsigned callw)
{
    unsigned me = core_of(win) + 1u, op = IPC_CALL_OP(callw);

    if (IPC_CALL_SVC(callw) != SVC_CARD || op == CARDSVC_STATUS)
        return FW_OK;
    /* The same rule as card.c: a row that does not describe the card
     * means the block is not touched, by either side. */
    if (op != CARDSVC_OFF && card_polarity() < 0)
        return FW_ENOTIMPL;
    if (card_core != 0u && card_core != me)
        return FW_EBUSY;
    if (card_core == 0u && op != CARDSVC_OFF && !card_idle())
        return FW_EBUSY;
    return FW_OK;
}

/* The checks both commands make.  FW_EIO if the image is not listening
 * (no MAGIC: not started, stopped, or an image without the queue) or the
 * counts make no sense. */
static int ready(unsigned b)
{
    if (MMIO(b + IPC_MAGIC) != IPC_MAGIC_VAL)
        return FW_EIO;
    if (MMIO(b + IPC_REQ_HEAD) - MMIO(b + IPC_REQ_TAIL) > IPC_SLOTS
        || MMIO(b + IPC_REP_HEAD) - MMIO(b + IPC_REP_TAIL) > IPC_SLOTS)
        return FW_EIO;
    return FW_OK;
}

/* ARM_CALL (0x64).  Queue one request.
 *
 *   arg0  core | IPC_CALL(service, op) << 8
 *   arg1  the request's ARG0
 *   data  +0 the request's ARG1, +4..+19 its 4 data words
 *
 * FW_EAGAIN if the ring is full.  The report:
 *
 *   +0  version, core, 0, 0
 *   +4  SEQ given to the request (the reply carries it)
 *   +8  requests waiting, this one included
 *   +12 replies waiting */
int arm_call(frame_t f)
{
    int rc;
    unsigned win = arm_pick(f, &rc);
    unsigned b, head, s, i, callw;

    if (rc)
        return rc;
    b = base(win);
    rc = ready(b);
    if (rc)
        return rc;
    callw = (frq1_be32(f, FR_ARG0) >> 8) & 0xffffu;
    rc = card_owner(win, callw);
    if (rc)
        return rc;
    head = MMIO(b + IPC_REQ_HEAD);
    if (head - MMIO(b + IPC_REQ_TAIL) >= IPC_SLOTS)
        return FW_EAGAIN;

    s = b + IPC_REQ + (head % IPC_SLOTS) * IPC_SLOT_BYTES;
    MMIO(s + IPC_Q_SEQ) = head + 1u;
    MMIO(s + IPC_Q_CALL) = callw;
    /* CARD ON's arg0 is the polarity.  The board row says what it is
     * (card.c reads the same field); the host's value is not used. */
    if (IPC_CALL_SVC(callw) == SVC_CARD && IPC_CALL_OP(callw) == CARDSVC_ON)
        MMIO(s + IPC_Q_ARG0) = (unsigned)card_polarity();
    else
        MMIO(s + IPC_Q_ARG0) = frq1_be32(f, FR_ARG1);
    MMIO(s + IPC_Q_ARG1) = frq1_be32(f, FR_DATA);
    for (i = 0; i < IPC_DATA_WORDS; i++)
        MMIO(s + IPC_Q_DATA + i * 4u) = frq1_be32(f, (int)(FR_DATA + 4u + i * 4u));
    MMIO(b + IPC_REQ_HEAD) = head + 1u;     /* last: the slot is whole */
    if (IPC_CALL_SVC(callw) == SVC_CARD) {
        if (IPC_CALL_OP(callw) == CARDSVC_ON)
            card_core = (unsigned char)(core_of(win) + 1u);
        else if (IPC_CALL_OP(callw) == CARDSVC_OFF)
            card_core = 0;
    }

    f[FR_DATA + 0] = (unsigned char)REPORT_VERSION;
    f[FR_DATA + 1] = (unsigned char)(win == ARM_E_WIN ? ARM_CORE_E : ARM_CORE_C);
    f[FR_DATA + 2] = 0;
    f[FR_DATA + 3] = 0;
    frq1_put32(f, FR_DATA + 4, head + 1u);
    frq1_put32(f, FR_DATA + 8, head + 1u - MMIO(b + IPC_REQ_TAIL));
    frq1_put32(f, FR_DATA + 12, MMIO(b + IPC_REP_HEAD) - MMIO(b + IPC_REP_TAIL));
    return FW_OK;
}

/* ARM_REAP (0x65).  Take the oldest reply.  arg0 = core.
 *
 * FW_EAGAIN if there is none yet.  The report:
 *
 *   +0  version, core, 0, 0
 *   +4  SEQ (of the request; an EVENT carries the one that set it up)
 *   +8  CALL (| IPC_EVENT)
 *   +12 STATUS (IPC_ST_*)
 *   +16 VAL
 *   +20..+35 the 4 data words
 *   +36 replies still waiting after this one */
int arm_reap(frame_t f)
{
    int rc;
    unsigned win = arm_pick(f, &rc);
    unsigned b, tail, s, i;

    if (rc)
        return rc;
    b = base(win);
    rc = ready(b);
    if (rc)
        return rc;
    tail = MMIO(b + IPC_REP_TAIL);
    if (MMIO(b + IPC_REP_HEAD) == tail)
        return FW_EAGAIN;

    s = b + IPC_REP + (tail % IPC_SLOTS) * IPC_SLOT_BYTES;
    f[FR_DATA + 0] = (unsigned char)REPORT_VERSION;
    f[FR_DATA + 1] = (unsigned char)(win == ARM_E_WIN ? ARM_CORE_E : ARM_CORE_C);
    f[FR_DATA + 2] = 0;
    f[FR_DATA + 3] = 0;
    frq1_put32(f, FR_DATA + 4, MMIO(s + IPC_R_SEQ));
    frq1_put32(f, FR_DATA + 8, MMIO(s + IPC_R_CALL));
    frq1_put32(f, FR_DATA + 12, MMIO(s + IPC_R_STATUS));
    frq1_put32(f, FR_DATA + 16, MMIO(s + IPC_R_VAL));
    for (i = 0; i < IPC_DATA_WORDS; i++)
        frq1_put32(f, (int)(FR_DATA + 20u + i * 4u), MMIO(s + IPC_R_DATA + i * 4u));
    MMIO(b + IPC_REP_TAIL) = tail + 1u;     /* last: the slot is read */
    frq1_put32(f, FR_DATA + 36, MMIO(b + IPC_REP_HEAD) - (tail + 1u));
    return FW_OK;
}

/* ARM_CANCEL (0x66).  The cancel words (armipc.h): stop a task even when
 * the rings are full.  arg0 = core, arg1 = what:
 *
 *   a SEQ            end the task that request started, with no reply
 *   IPC_CANCEL_ALL   end every task, drop every request not yet taken
 *   0                ask nothing; only report (SEQs start at 1)
 *
 * Does not wait for the ARM.  FW_EAGAIN if the last cancel is not yet
 * acknowledged (call again with 0 to see when it is); FW_EIO if the image
 * is not listening -- then ARM_HOLD is the way to stop it.  The report:
 *
 *   +0  version, core, 0, 0
 *   +4  CANCEL_REQ   (the number of this cancel, if one was asked)
 *   +8  CANCEL_ACK   (equal to CANCEL_REQ once the ARM has handled it)
 *   +12 CANCEL_DONE  tasks the last handled cancel ended
 *   +16 CANCEL_DROPPED  requests it dropped
 *   +20 requests waiting, +24 replies waiting */
int arm_cancel(frame_t f)
{
    int rc;
    unsigned win = arm_pick(f, &rc);
    unsigned b, what, req;

    if (rc)
        return rc;
    b = base(win);
    /* Only MAGIC: this must work when the counts are the problem. */
    if (MMIO(b + IPC_MAGIC) != IPC_MAGIC_VAL)
        return FW_EIO;
    what = frq1_be32(f, FR_ARG1);
    req = MMIO(b + IPC_CANCEL_REQ);
    if (what != 0u) {
        if (req != MMIO(b + IPC_CANCEL_ACK))
            return FW_EAGAIN;
        MMIO(b + IPC_CANCEL_WHAT) = what;
        req = req + 1u;
        MMIO(b + IPC_CANCEL_REQ) = req;     /* last: WHAT is in place */
    }

    f[FR_DATA + 0] = (unsigned char)REPORT_VERSION;
    f[FR_DATA + 1] = (unsigned char)(win == ARM_E_WIN ? ARM_CORE_E : ARM_CORE_C);
    f[FR_DATA + 2] = 0;
    f[FR_DATA + 3] = 0;
    frq1_put32(f, FR_DATA + 4, req);
    frq1_put32(f, FR_DATA + 8, MMIO(b + IPC_CANCEL_ACK));
    frq1_put32(f, FR_DATA + 12, MMIO(b + IPC_CANCEL_DONE));
    frq1_put32(f, FR_DATA + 16, MMIO(b + IPC_CANCEL_DROPPED));
    frq1_put32(f, FR_DATA + 20, MMIO(b + IPC_REQ_HEAD) - MMIO(b + IPC_REQ_TAIL));
    frq1_put32(f, FR_DATA + 24, MMIO(b + IPC_REP_HEAD) - MMIO(b + IPC_REP_TAIL));
    return FW_OK;
}

/* ARM_PUT (0x67).  Bytes into the buffer's TX half (armipc.h, THE BUFFER),
 * for a request that names them (a CARD XFER).  arg0 = core | n << 8
 * (1..40), arg1 = offset in the TX half (a multiple of 4); data = the
 * bytes.  Packed big-endian by value; a last part word keeps its other
 * bytes (only the main FR writes this half).  FW_EIO if the image is not
 * listening.  Write before ARM_CALL: the ARM reads the half only once it
 * has taken the request. */
int arm_put(frame_t f)
{
    int rc;
    unsigned win = arm_pick(f, &rc);
    unsigned n = (frq1_be32(f, FR_ARG0) >> 8) & 0xffu;
    unsigned off = frq1_be32(f, FR_ARG1), b, i;

    if (rc)
        return rc;
    if (n == 0u || n > PUT_MAX || (off & 3u) || off >= BUF_HALF
        || n > BUF_HALF - off)
        return FW_EINVAL;
    rc = ready(base(win));
    if (rc)
        return rc;
    b = win + ARM_BUF_OFF + BUF_TX + off;
    for (i = 0; i < n; i += 4u) {
        unsigned w = 0, k, a = b + i;

        if (n - i < 4u)
            w = MMIO(a);
        for (k = 0; k < 4u && i + k < n; k++) {
            unsigned sh = 24u - 8u * k;

            w = (w & ~(0xffu << sh)) | ((unsigned)f[FR_DATA + i + k] << sh);
        }
        MMIO(a) = w;
    }
    return FW_OK;
}

/* ARM_GET (0x68).  40 bytes of the buffer's RX half.  arg0 = core, arg1 =
 * offset (a multiple of 4); zero past the half's end.  Read after the
 * reply that says the bytes are there. */
int arm_get(frame_t f)
{
    int rc;
    unsigned win = arm_pick(f, &rc);
    unsigned off = frq1_be32(f, FR_ARG1), b, i;

    if (rc)
        return rc;
    if ((off & 3u) || off >= BUF_HALF)
        return FW_EINVAL;
    rc = ready(base(win));
    if (rc)
        return rc;
    b = win + ARM_BUF_OFF + BUF_RX;
    for (i = 0; i < PUT_MAX; i += 4u)
        frq1_put32(f, (int)(FR_DATA + i),
                   off + i < BUF_HALF ? MMIO(b + off + i) : 0u);
    return FW_OK;
}
