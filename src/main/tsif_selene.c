/* tsif_selene.c -- TS input: bring the capture block up so the ring fills.
 *
 * ONLY MEASURED BOARDS.  A board row with ts.evidence = measured carries the
 * values this file needs (ts_in0 / ts_in1), and the sequence below is the
 * one that filled the ring with a clean TS on such a board.  Any other board
 * gets FW_ENOTIMPL from tsif_init() and tsif_start() before anything is
 * written: starting a DMA engine whose input was never configured is worse
 * than a block that is still off.
 *
 * NAMES ARE ADDRESSES.  What most of these registers do is not known and
 * this file does not guess.  regs_selene.h lists the registers.  What is
 * known is what leaving each write out did, and that is written next to it.
 *
 * NOT ON AN FZ2 YET.  At state 0 the whole 0x01900000 block reads zeros and
 * ignores writes from the main FR, as do 0x01684000 and 0x00D00000.  What
 * powers those units up on that board is not known.
 *
 * NOT TOUCHED: the pin multiplexing either side of this sequence
 * (0x00F1011C, 0x00F10134).  It is assumed to be set for the board already
 * when we run.
 *
 * The teardown's poll is bounded, because a loop waiting on a bit that
 * never clears would hang the only path the host has into this firmware.
 */
#include "fw.h"
#include "drivers.h"
#include "mb86m01.h"
#include "regs_selene.h"

/* THE MEASURED PATH (a board row with ts.evidence = measured).
 *
 * MEASURED ON A PIX-DT295: with the demodulator locked, the writes in
 * m_start() fill the ring with 192-byte records carrying a clean TS (sync
 * on every record, no continuity error, a PAT that parses).  Each one was
 * left out in turn, both on the first session after power-up and on a
 * later one; the comment on each says what leaving it out did.  What the registers are is otherwise not known, so
 * the names are the addresses.
 *
 * Writes that changed nothing were dropped (measured the same way): the
 * mode word 0x01902000, the half-word at
 * 0x00F12788, and the zeros and fill patterns elsewhere in 0x01901000 and
 * at 0x01902030.
 *
 * Before any of this, m_init() opens the unit gate 0x00F10118 (half-word) to
 * 0xFFFF -- with it at its cold value the capture block reads zeros and
 * ignores writes -- then bit2 of the capture block's +0x404 half-word.
 * Two words in unit 0x016 (0x01600000, 0x01600004) were written here
 * before; leaving out either changed nothing on the first session after
 * power-up (measured on a PIX-DT295), so they are not.
 *
 * THE RING IS 4 MiB AT 0x02000000, 21,845 whole records (0x003FFFC0).
 * Nothing runs here between two host frames, so the ring has to cover a
 * host stall.  192 KiB holds 52 ms of a UHF stream (3.72 MB/s) and
 * overflows on a UHF channel; 4 MiB (about 1.1 s) does not (measured).  On a
 * PIX-DT295 the window reads "MB8AC031" at 0x02000000, 0xF0F0F0F0 fill
 * from 0x02100000, and 0x02400000 reads the same as 0x02000000 (a 4 MiB
 * window that wraps, not established).
 */
#define M_RING_PHYS     0x02000000u
#define M_RING_LEN      0x003fffc0u     /* 21845 records of 192 bytes */
#define M_RUN           0x02000000u
#define M_STOP          0x04000000u

static int measured(void)
{
    return fw_board.ev_ts == EV_MEASURED;
}

static int m_init(void)
{
    MMIO16(SL_UNIT_GATE) = (unsigned short)SL_UNIT_GATE_ALL;
    MMIO16(TSB0_404) = (unsigned short)((MMIO16(TSB0_404) & 0xffffu) |
                                        SL_UNIT_404_ON);
    /* The block is asleep if that did not take, and nothing below would
     * stick: say so rather than start a DMA engine with no input. */
    if (!(MMIO16(TSB0_404) & SL_UNIT_404_ON))
        return FW_EIO;
    return FW_OK;
}

static void m_start(void)
{
    /* The head of the block, then the ring, the input and the packet,
     * then the run word and the start.  The values are the measured ones;
     * the grouping is ours.  Checked in this order on a PIX-DT295 from a
     * power-up and on a later session. */

    /* The two words at the head of the 0x01900000 block.  Without either,
     * nothing reaches the ring.  They come first: with them written last
     * (just before the start), nothing reached the ring on the first
     * session after power-up, and a later session worked (measured on a
     * PIX-DT295). */
    MMIO(TSB0_000) = 0x00020000u;
    MMIO(TSB0_004) = 0x00001100u;

    /* The ring.  Without the base or the length nothing reaches it.
     * Without zeroing the two pointers, the next session starts where the
     * last one stopped and sends old records first (continuity errors). */
    MMIO(TSCAP_BASE) = M_RING_PHYS;
    MMIO(TSCAP_LENGTH) = M_RING_LEN;
    MMIO(TSCAP_WRITE) = 0u;
    MMIO(TSCAP_READ) = 0u;

    /* The input.  The board's input selection (ts_in0, high half-word):
     * without it, nothing reaches the ring.  The board's ts_in1: without
     * it nothing synchronises on the first session after power-up (the
     * ring fills, but no record starts with 0x47); a later session works
     * without it, because nothing clears it. */
    MMIO16(TSB1_000) = (unsigned short)(fw_board.ts_in0 >> 16);
    MMIO(TSB1_004) = fw_board.ts_in1;

    /* The packet.  0xBC is 188, the TS packet length (assumed from the
     * value).  Without it, or without 0x01901080, nothing reaches the
     * ring. */
    MMIO(0x01901044u) = 0x00bc0000u;
    MMIO(0x01901080u) = 0x01000000u;

    /* Without it, nothing reaches the ring.  m_stop() clears it again. */
    MMIO(TSB1_040) = 0x03000000u;
    /* Start. */
    MMIO(TSCAP_R010) = M_RUN;
}

static int m_stop(void)
{
    unsigned i;

    /* Without it the engine is still running when the next session starts
     * (0x01902010 stays at the run value), and that session's TS has a
     * continuity error. */
    MMIO(TSCAP_R010) = M_STOP;
    /* m_start() sets this to 0x03000000.  Without clearing it here, the
     * next session starts with it already set, and often only about 64% of
     * the stream reaches the host: the ring fills and the TS has thousands
     * of continuity errors (measured on a PIX-DT295). */
    MMIO(TSB1_040) = 0u;
    for (i = 0; i < TSCAP_SPIN; i++)
        if (!MMIO(TSCAP_R010))
            return FW_OK;
    return FW_ETIMEOUT;
}

static int started;

int tsif_init(void)
{
    if (!measured())
        return FW_ENOTIMPL;     /* no measured sequence for this board */
    return m_init();
}

/* Point the block at the ring and start it.  ring_init() comes after this:
 * it reads the base and the length back out of the registers written here.
 */
int tsif_start(void)
{
    if (!measured())
        return FW_ENOTIMPL;
    m_start();
    started = 1;
    return FW_OK;
}

int tsif_stop(void)
{
    if (!started)
        return FW_OK;
    started = 0;
    return m_stop();
}
