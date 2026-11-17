/* frq1.c -- validate the frame, route the command, answer.
 *
 * Contract (src/shared/frq1.h): entered with a pointer to the 64-byte frame.
 * Returns 0 when we answered and 1 when the frame was not ours.
 */
#include "fw.h"

unsigned frq1_be32(frame_t f, int off)
{
    return ((unsigned)f[off] << 24) | ((unsigned)f[off + 1] << 16)
         | ((unsigned)f[off + 2] << 8) | (unsigned)f[off + 3];
}

void frq1_put32(frame_t f, int off, unsigned v)
{
    f[off]     = (unsigned char)(v >> 24);
    f[off + 1] = (unsigned char)(v >> 16);
    f[off + 2] = (unsigned char)(v >> 8);
    f[off + 3] = (unsigned char)v;
}

/* A failure is the module's own code, signed, in the one status word.  No
 * second error space and no translation table: the host reads the same
 * numbers fw.h defines (docs/frq1.md). */
static void frq1_fail(frame_t f, int code)
{
    frq1_put32(f, FR_STATUS, (unsigned)code);
}

/* Turn a module's return into the frame.  FW_OK leaves the ST_OK the router
 * already wrote; anything else becomes the status. */
static void answer(frame_t f, int rc)
{
    if (rc != FW_OK)
        frq1_fail(f, rc);
}

/* The dispatcher goes right after _start. The `call` in entry.S reaches
 * only +-2 KB. Switching entry.S to absolute addresses makes _start 8 bytes
 * longer, shifts everything after it, and breaks the container path
 * (src/main/link_selene.ld). So its address is fixed. */
__attribute__((section(".text.dispatch")))
int frq1_dispatch(frame_t f)
{
    unsigned cmd;

    /* The staging area is read back by the host, so a frame without the
     * magic is not ours: leave every byte of it alone.  Someone else's
     * frame goes back untouched through here. */
    if (frq1_be32(f, FR_MAGIC) != FRQ1_MAGIC)
        return 1;

    cmd = frq1_be32(f, FR_COMMAND);

    /* Answer OK first, into the one field we are allowed to write.  Every
     * path below then leaves a frame the host accepts, and a handler only
     * has to override.  The other five words keep the request's values,
     * which is what makes the echo free -- and the command echo is how a
     * host knows the answer came from THIS firmware and not from the other
     * one that speaks the same magic at the same address (frq1.h). */
    frq1_put32(f, FR_STATUS, (unsigned)ST_OK);

    /* Each command goes to the module that owns it.  A module that is not
     * written yet returns FW_ENOTIMPL and that becomes the frame's status --
     * so adding a module is a matter of making its function work, not of
     * touching this switch. */
    switch (cmd) {
    case CMD_IDENTIFY:
        answer(f, dev_identify(f));
        break;
    case CMD_START:
        /* Bring up what can be brought up -- the bus, and which
         * demodulators answer (fe.c).  **Its result is deliberately not the
         * frame's status.** The runtime IS up, and saying so is what lets a
         * host keep using the commands that work on a board whose RF side
         * this firmware cannot drive.  Which halves are real is reported by
         * CMD_STATUS's flags, where a host can act on it.
         */
        (void)fe_init();
        health_started(1);
        break;
    case CMD_STOP:
        fe_stop();
        stream_stop();
        health_started(0);
        /* Stopping something that never started is not an error. */
        break;
    case CMD_SUSPEND:
        answer(f, pm_suspend(f));
        break;
    case CMD_HEALTH:
        answer(f, health_report(f));
        break;
    case CMD_STATUS:
        answer(f, fe_channel_info(f));
        break;
    case CMD_CNR:
        answer(f, fe_cnr(f));
        break;
    case CMD_STREAM:
        answer(f, stream_start(f));
        break;
    case CMD_PEEK:
        answer(f, probe_peek(f));
        break;
    case CMD_POKE:
        answer(f, probe_poke(f));
        break;
    case CMD_RING:
        answer(f, probe_ring(f));
        break;
    case CMD_I2CSCAN:
        answer(f, probe_i2cscan(f));
        break;
    case CMD_I2CXFER:
        answer(f, fw_is_dione() ? dione_i2cxfer(f) : i2c_frame_xfer(f));
        break;
    case CMD_EPDMA:
        answer(f, probe_epdma(f));
        break;
    case CMD_CARD_STATUS:
        answer(f, card_status(f));
        break;
    case CMD_CARD_ON:
        answer(f, card_on(f));
        break;
    case CMD_CARD_OFF:
        answer(f, card_off(f));
        break;
    case CMD_CARD_PARAM:
        answer(f, card_param(f));
        break;
    case CMD_CARD_PUT:
        answer(f, card_put(f));
        break;
    case CMD_CARD_SEND:
        answer(f, card_send(f));
        break;
    case CMD_CARD_RECV:
        answer(f, card_recv(f));
        break;
    case CMD_CARD_GET:
        answer(f, card_get(f));
        break;
    case CMD_ARM_STATUS:
        answer(f, arm_status(f));
        break;
    case CMD_ARM_HOLD:
        answer(f, arm_hold(f));
        break;
    case CMD_ARM_LOAD:
        answer(f, arm_load(f));
        break;
    case CMD_ARM_RUN:
        answer(f, arm_run(f));
        break;
    case CMD_ARM_CALL:
        answer(f, arm_call(f));
        break;
    case CMD_ARM_REAP:
        answer(f, arm_reap(f));
        break;
    case CMD_ARM_CANCEL:
        answer(f, arm_cancel(f));
        break;
    case CMD_ARM_PUT:
        answer(f, arm_put(f));
        break;
    case CMD_ARM_GET:
        answer(f, arm_get(f));
        break;
    default:
        /* The one failure that happens outside a module.  Not FW_ENOTIMPL:
         * that says "we know this command but have not written it", and this
         * is a number nobody has defined. */
        frq1_fail(f, FW_EBADCMD);
        break;
    }
    /* Whatever the frame now says, the part that owns the command keeps it
     * if it is the first failure since the host last cleared (health.c). */
    health_note(cmd, (int)frq1_be32(f, FR_STATUS));
    return 0;
}
