/* card.c -- the built-in card interface (ISO 7816).  docs/card.md.
 *
 * WHAT IS KNOWN
 *   On a GV-M2TV, driven from the host's 0xBC window (measured): the
 *   registers at 0x00FEA000 (mb86m01.h), polarity 0 for that board, and
 *   which writes the power-up, the power-down and a block need.  Each write
 *   below was taken out on its own and the ATR and a T=1 S(IFS) exchange
 *   were tried again; what is here is what was needed, in the order of an
 *   ISO 7816-3 activation and deactivation.  No interrupts: everything polls.
 *
 * WHAT IS NOT
 *   - Whether the block looks the same from INSIDE this firmware.  On dione
 *     the I2C window answered the host and not the CPU; check CARD_STATUS
 *     against the window's values first (docs/card.md, the section on the order of checks).
 *   - Card insertion.  CARD_EVENT bit10 is 1 with the M2TV's card in, and
 *     that card cannot be pulled.
 *   - The units of CARD_CWT / CARD_BWT.  They only raise flags in
 *     CARD_EVENT and stop nothing (measured), so they are set from the
 *     ISO 7816-3 definitions in etu and nothing here reads the flags.
 *   - Blocks of 256 bytes or more (another CARD_CTL shape, not measured).
 *     CARD_SEND refuses them; a host keeps IFSD at 251 or less.
 *   - Any board but the M2TV.  The board row's card entry decides: unknown
 *     there means FW_ENOTIMPL here, CARD_STATUS included.
 *
 * SHAPE
 *   The host keeps the time and the T=1 protocol.  There is no clock here
 *   and a frame is one call in and one return out, so anything with a wait
 *   in it does one step, says how long to wait, and returns FW_EAGAIN; the
 *   host waits and sends the same command again.  Giving up is the host's
 *   call too, and CARD_OFF is always safe to send.
 *
 *   The block can be handed to an ARM core instead (src/armcore/svc_card.c
 *   does the same steps with its own waits).  One owner at a time: while
 *   an ARM core has it, every command here but CARD_STATUS is FW_EBUSY,
 *   and the ARM gets it only while this file has it powered down
 *   (arm_ipc.c keeps the score).
 */
#include "fw.h"
#include "drivers.h"
#include "mb86m01.h"

/* Where a power-up / exchange stands.  Reported as is (docs/card.md). */
#define STAGE_OFF       0u      /* 1..3: that many power-up steps done    */
#define STAGE_STEPS     3u
#define STAGE_ATR       4u      /* RST is up; collecting the ATR           */
#define STAGE_READY     5u      /* the ATR is complete                     */
#define STAGE_SENT      6u      /* a block went out; collecting the answer */
#define STAGE_GOT       7u      /* one whole block came back               */

#define WAIT_VCC        10u     /* ms for VCC to settle before the clock   */
#define WAIT_CLK        1u      /* ms of clock before RST (ISO: 400 clocks) */
#define WAIT_POLL       10u     /* ms between polls for bytes              */

#define ATR_MAX         33u
#define BLOCK_MAX       259u    /* NAD PCB LEN + 254 + LRC, and one spare  */
#define SEND_MAX        255u    /* under 256: the only size measured       */
#define DATA_MAX        40u     /* the frame's data area                  */

/* .bss: kept between frames, cleared when the image is installed. */
static unsigned char stage;
static unsigned char atr_len;
static unsigned short rx_len;
static unsigned char tx_buf[BLOCK_MAX];
static unsigned char rx_buf[BLOCK_MAX];

/* --- the board ---------------------------------------------------------
 * The polarity is the one per-board fact the sequence needs.  The bytes at
 * 0x00F00220.. read 0 in the loader state, and following them is wrong on
 * the M2TV (measured), so the board row says instead.  A row that does not say means "do not touch the block".
 */
static const struct board *card_board(void)
{
    const struct board *b = &fw_board;

    if (b->ev_card == EV_UNKNOWN || b->card_polarity > 1u)
        return 0;
    return b;
}

/* For arm_ipc.c: the card may be handed to an ARM core only while it is
 * powered down here. */
int card_idle(void)
{
    return stage == STAGE_OFF;
}

/* Every command but CARD_STATUS: the block has one owner at a time, and
 * while an ARM core has it (src/armcore/svc_card.c) this file keeps off. */
static const struct board *card_mine(int *rc)
{
    const struct board *b = card_board();

    *rc = FW_OK;
    if (!b)
        *rc = FW_ENOTIMPL;
    else if (arm_has_card())
        *rc = FW_EBUSY;
    return *rc ? 0 : b;
}

/* For IDENTIFY: whether the card commands would run on this board. */
int card_available(void)
{
    return card_board() != 0;
}

/* For arm_ipc.c: this board's card polarity (0 or 1), or -1 when the row
 * does not let the block be touched.  The ARM's CARD ON takes it from
 * here, not from the host, so the row is the one place it is written. */
int card_polarity(void)
{
    const struct board *b = card_board();

    return b ? (int)b->card_polarity : -1;
}

static unsigned pol_bits(const struct board *b)
{
    return b->card_polarity ? CARD_PIN_POL : 0u;
}

/* --- the report -------------------------------------------------------- */
static void put8(frame_t f, unsigned off, unsigned v)
{
    f[FR_DATA + off] = (unsigned char)v;
}

static void put16(frame_t f, unsigned off, unsigned v)
{
    f[FR_DATA + off] = (unsigned char)(v >> 8);
    f[FR_DATA + off + 1u] = (unsigned char)v;
}

static void clear_data(frame_t f)
{
    unsigned i;

    for (i = 0; i < DATA_MAX; i++)
        f[FR_DATA + i] = 0;
}

/* CARD_STATUS / CARD_ON / CARD_OFF share one shape. */
static void report(frame_t f, unsigned wait)
{
    unsigned i;

    clear_data(f);
    put8(f, 0, 1);
    put8(f, 1, stage);
    put8(f, 2, wait);
    put8(f, 3, atr_len);
    frq1_put32(f, FR_DATA + 4, MMIO(CARD_EVENT));
    frq1_put32(f, FR_DATA + 8, MMIO(CARD_PINS));
    frq1_put32(f, FR_DATA + 12, MMIO(CARD_MODE));
    for (i = 0; i < atr_len && i < 24u; i++)
        put8(f, 16u + i, rx_buf[i]);
}

/* --- receiving ---------------------------------------------------------
 * CARD_MODE's top half is how many bytes are waiting; each CARD_RX read
 * takes one.  Read exactly that many and no more -- an extra read would
 * take a byte that has not arrived.
 */
static int drain(unsigned cap)
{
    unsigned n = MMIO(CARD_MODE) >> 16;

    while (n--) {
        unsigned char c = (unsigned char)MMIO(CARD_RX);

        if (rx_len >= cap)
            return FW_EIO;      /* more than any legal answer */
        rx_buf[rx_len++] = c;
    }
    return FW_OK;
}

/* Bytes left in the receive FIFO outlive a power-down (measured: an unread
 * ATR was still there, ahead of the next one, after VCC off and on), and
 * no register write tried empties it.  So whatever is waiting is read and
 * dropped before RST goes up and before a block goes out. */
static void discard(void)
{
    unsigned n = MMIO(CARD_MODE) >> 16;

    while (n--)
        (void)MMIO(CARD_RX);
}

/* How long the ATR in rx_buf is, once enough of it is here to say: the TDi
 * chain decides how many interface bytes follow, T0 how many historical
 * bytes, and any protocol other than T=0 adds TCK.  0 = not known yet.
 */
static unsigned atr_length(void)
{
    unsigned i = 2, y, k, tck = 0, n = 0;

    if (rx_len < 2u)
        return 0;
    y = rx_buf[1] >> 4;
    k = rx_buf[1] & 0x0fu;
    for (;;) {
        if (y & 1u) i++;
        if (y & 2u) i++;
        if (y & 4u) i++;
        if (!(y & 8u))
            break;
        if (rx_len <= i)
            return 0;           /* TDi not here yet */
        if ((rx_buf[i] & 0x0fu) != 0u)
            tck = 1;
        y = rx_buf[i] >> 4;
        i++;
        if (++n > 8u)
            return ATR_MAX + 1u;        /* nonsense; the caller refuses */
    }
    return i + k + tck;
}

/* --- powering up -------------------------------------------------------
 * An ISO 7816-3 activation, one step per call; returns the wait in ms
 * before the next one.  Measured on a GV-M2TV (docs/card.md, the table):
 *   1  lines low (RST low, no clock) before VCC; the block on (CARD_EN 7,
 *      without it no ATR); the ATR rate, F/D 372 (left at a T=1 rate from
 *      the last session the ATR comes out as noise); then VCC (without it
 *      no ATR).  Lines low first was not needed in the test; it is the
 *      ISO order.
 *   2  the clock.
 *   3  drop what is in the receive FIFO (discard()), then RST up: the ATR
 *      starts (without it no ATR).
 * Other writes once thought part of this -- clearing CARD_EVENT, the
 * receive mode, CARD_RXLRC, CARD_CWT / CARD_BWT, CARD_IRQEN, a second
 * CARD_EN 0 -> 7 -- were each taken out and the ATR still came.
 */
static unsigned power_step(unsigned step, unsigned p)
{
    switch (step) {
    case 1:
        MMIO(CARD_PINS) = p;
        MMIO(CARD_EN) = CARD_EN_ON;
        MMIO(CARD_BAUD) = CARD_FD_ATR << CARD_BAUD_SHIFT;
        MMIO(CARD_VCC) = 1;
        return WAIT_VCC;
    case 2:
        MMIO(CARD_PINS) = p | CARD_PIN_CLK;
        return WAIT_CLK;
    case 3:
        discard();
        MMIO(CARD_PINS) = p | CARD_PIN_CLK | CARD_PIN_RST;
        return WAIT_POLL;
    default:
        return 0;
    }
}

/* An ISO 7816-3 deactivation: RST low, then the clock, then VCC.  With
 * these three the next power-up works, ten times in a row (measured). */
static void power_down(unsigned p)
{
    MMIO(CARD_PINS) = p | CARD_PIN_CLK;
    MMIO(CARD_PINS) = p;
    MMIO(CARD_VCC) = 0;
}

/* --- the commands ------------------------------------------------------ */

int card_status(frame_t f)
{
    if (!card_board())
        return FW_ENOTIMPL;
    report(f, 0);
    return FW_OK;
}

int card_on(frame_t f)
{
    unsigned wait, need;
    int rc;
    const struct board *b = card_mine(&rc);

    if (!b)
        return rc;
    if (stage >= STAGE_READY) {         /* already up: say so again */
        report(f, 0);
        return FW_OK;
    }
    if (stage < STAGE_STEPS) {
        if (stage == STAGE_OFF) {
            atr_len = 0;
            rx_len = 0;
        }
        stage++;
        wait = power_step(stage, pol_bits(b));
        if (stage == STAGE_STEPS)
            stage = STAGE_ATR;
        report(f, wait);
        return FW_EAGAIN;
    }

    /* STAGE_ATR */
    rc = drain(ATR_MAX);
    need = atr_length();
    if (rc != FW_OK || need > ATR_MAX) {
        report(f, 0);
        return FW_EIO;
    }
    if (need && rx_len >= need) {
        if (rx_len > need) {            /* bytes past the ATR: not ours */
            report(f, 0);
            return FW_EIO;
        }
        atr_len = (unsigned char)rx_len;
        stage = STAGE_READY;
        report(f, 0);
        return FW_OK;
    }
    report(f, WAIT_POLL);
    return FW_EAGAIN;
}

int card_off(frame_t f)
{
    int own;
    const struct board *b = card_mine(&own);

    if (!b)
        return own;
    power_down(pol_bits(b));
    stage = STAGE_OFF;
    atr_len = 0;
    rx_len = 0;
    report(f, 0);
    return FW_OK;
}

/* arg0 = F << 16 | D, arg1 = CWI << 8 | BWI, from the ATR.  The rate is
 * F/D clocks per etu (mb86m01.h).  The waits are the ISO 7816-3 ones in
 * etu: CWT = 11 + 2^CWI, BWT = 11 + 2^BWI * 960 * 372 * D / F.  The block
 * takes the waits only as flags (measured), so a wrong unit there costs
 * nothing but a flag. */
int card_param(frame_t f)
{
    unsigned a0 = frq1_be32(f, FR_ARG0), a1 = frq1_be32(f, FR_ARG1);
    unsigned fi = a0 >> 16, di = a0 & 0xffffu;
    unsigned cwi = (a1 >> 8) & 0xffu, bwi = a1 & 0xffu;
    unsigned n;
    int own;

    if (!card_mine(&own))
        return own;
    if (stage < STAGE_READY)
        return FW_EAGAIN;               /* no ATR yet: nothing to set */
    if (fi == 0u || di == 0u || di > 64u || di > fi || cwi > 15u || bwi > 9u)
        return FW_EINVAL;
    n = fw_udiv(fi, di, 0);
    if (n > 0x1ffu)                     /* does not fit bits 15:7 */
        return FW_EINVAL;
    MMIO(CARD_BAUD) = n << CARD_BAUD_SHIFT;
    MMIO(CARD_CWT) = 11u + (1u << cwi);
    /* divide first: 960 * 372 * D fits, the shift by BWI after it too */
    MMIO(CARD_BWT) = 11u + (fw_udiv(960u * 372u * di, fi, 0) << bwi);
    return FW_OK;
}

/* arg0 = where in the send buffer, arg1 = how many bytes (1..40). */
int card_put(frame_t f)
{
    unsigned off = frq1_be32(f, FR_ARG0), n = frq1_be32(f, FR_ARG1);
    unsigned i;
    int own;

    if (!card_mine(&own))
        return own;
    if (n == 0u || n > DATA_MAX || off >= BLOCK_MAX || n > BLOCK_MAX - off)
        return FW_EINVAL;
    for (i = 0; i < n; i++)
        tx_buf[off + i] = f[FR_DATA + i];
    return FW_OK;
}

/* arg0 = how many bytes of the send buffer make the block (the host has put
 * the LRC at the end).  Stale bytes are dropped first (discard()), then the
 * bytes go into CARD_TX and CARD_GO sends them;
 * the receiver needs nothing else (measured: the receive mode, CARD_RXLRC,
 * CARD_TXLRC, CARD_EVENT and CARD_CTL were each left out and the answer
 * still came). */
int card_send(frame_t f)
{
    unsigned n = frq1_be32(f, FR_ARG0), i;
    int own;

    if (!card_mine(&own))
        return own;
    if (n == 0u || n > SEND_MAX)
        return FW_EINVAL;
    if (stage < STAGE_READY)
        return FW_EAGAIN;               /* not powered up */
    discard();
    for (i = 0; i < n; i++)
        MMIO(CARD_TX) = tx_buf[i];
    MMIO(CARD_GO) = 1;
    rx_len = 0;
    atr_len = 0;                        /* the receive buffer is reused */
    stage = STAGE_SENT;
    return FW_OK;
}

/* One T=1 block: NAD PCB LEN, then LEN bytes and the LRC. */
int card_recv(frame_t f)
{
    unsigned need = 0, i;
    int rc;
    int own;

    if (!card_mine(&own))
        return own;
    if (stage != STAGE_SENT && stage != STAGE_GOT)
        return FW_EAGAIN;               /* nothing was sent */
    rc = stage == STAGE_SENT ? drain(BLOCK_MAX) : FW_OK;
    if (rx_len >= 3u)
        need = 3u + rx_buf[2] + 1u;
    if (rc == FW_OK && need && rx_len > need)
        rc = FW_EIO;                    /* more than one block */
    if (rc == FW_OK && need && rx_len == need)
        stage = STAGE_GOT;

    clear_data(f);
    put8(f, 0, 1);
    put8(f, 1, stage);
    put16(f, 2, rx_len);
    put16(f, 4, need);
    for (i = 0; i < rx_len && i < 32u; i++)
        put8(f, 8u + i, rx_buf[i]);

    if (rc != FW_OK)
        return rc;
    return stage == STAGE_GOT ? FW_OK : FW_EAGAIN;
}

/* arg0 = where in the receive buffer.  40 bytes, zero past the end. */
int card_get(frame_t f)
{
    unsigned off = frq1_be32(f, FR_ARG0), i;
    int own;

    if (!card_mine(&own))
        return own;
    if (off >= BLOCK_MAX)
        return FW_EINVAL;
    for (i = 0; i < DATA_MAX; i++)
        put8(f, i, off + i < rx_len ? rx_buf[off + i] : 0u);
    return FW_OK;
}
