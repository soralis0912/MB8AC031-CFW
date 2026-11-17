/* frq1.h -- the host side of the 64-byte `FRQ1` frame.  Header-only.
 *
 * The frame is specified in ../docs/frq1.md; the firmware's side is
 * ../src/shared/frq1.h.  This is the other end of the same wire, meant to
 * be copied into whatever host program wants to talk to the firmware.
 *
 * EVERY FIELD IS BIG-ENDIAN, so on a little-endian host the raw 64 bytes
 * are NOT this struct: nothing here casts the buffer to it.  The accessors
 * take the buffer and an offset and assemble a byte at a time, exactly as
 * the firmware does, because neither end may assume the frame is aligned --
 * it lives at a staging address the host picked.
 *
 * FOUR CHECKS MAKE A REPLY.  The staging area is plain memory the host
 * reads back, so "my request is still sitting there" and "an answer came
 * back" are told apart by looking:
 *
 *     1. the magic is 'FRQ1'
 *     2. THE COMMAND WORD IS STILL THE COMMAND WE SENT   <- who answered
 *     3. the sequence is still ours
 *     4. the status word is no longer FRQ1_ST_PENDING
 *
 * frq1_reply_check() does all four.  Check 2 is load-bearing: a read-back
 * with the right magic but a changed command word is something other than
 * our firmware answering, and it is never mistaken for ours.  Our command
 * numbers start at 0x10.  ../docs/frq1.md has the table.
 *
 * The report layouts at the bottom are the firmware's own
 * (../src/main/fe.c and ../src/main/stream.c).  They carry a version
 * byte first so they can grow.
 *
 * Needs C11 for _Static_assert.  No dependencies.
 */
#ifndef FRQ1_HOST_H
#define FRQ1_HOST_H

#include <stdint.h>
#include <string.h>

#define FRQ1_MAGIC          0x46525131u     /* 'FRQ1'                     */
#define FRQ1_BYTES          64u
#define FRQ1_DATA_BYTES     40u
#define FRQ1_VERSION        1u              /* this frame's own version   */

/* Field offsets in the frame.  Same names as the firmware's FR_*. */
#define FRQ1_FR_MAGIC       0x00u   /* echoed: the firmware never writes it */
#define FRQ1_FR_SEQUENCE    0x04u   /* echoed                              */
#define FRQ1_FR_COMMAND     0x08u   /* echoed -- this is check 2 above     */
#define FRQ1_FR_STATUS      0x0cu   /* out: +1 OK, negative = why not      */
#define FRQ1_FR_ARG0        0x10u
#define FRQ1_FR_ARG1        0x14u
#define FRQ1_FR_DATA        0x18u

/* The status word (FRQ1_FR_STATUS).  The host writes PENDING on the way
 * out; the firmware always overwrites it, with OK or with a reason. */
#define FRQ1_ST_PENDING     0
#define FRQ1_ST_OK          1

/* Negative statuses are the firmware's own FW_* codes (src/shared/fw.h),
 * one space with no translation layer.  Only E_BADCMD is the frame's own:
 * it is the one failure that happens outside a module. */
#define FRQ1_E_NOTIMPL      (-1)    /* known command, no implementation    */
#define FRQ1_E_IO           (-2)    /* the hardware did not answer         */
#define FRQ1_E_TIMEOUT      (-3)
#define FRQ1_E_INVAL        (-4)    /* bad argument                        */
#define FRQ1_E_AGAIN        (-5)    /* nothing wrong, just not enough yet  */
#define FRQ1_E_BADCMD       (-6)    /* unknown command                    */
#define FRQ1_E_BUSY         (-7)    /* another core has that block        */

/* Commands.  Grouped by what they are for:
 *
 *     0x00        not a command (so a zeroed frame cannot be one)
 *     0x01..0x0F  RESERVED.  NEVER allocate here -- a stray frame with
 *                 a small command number must not run anything of ours.
 *     0x10..0x1F  this firmware itself
 *     0x20..0x2F  the front end (RF)
 *     0x30..0x3F  the stream
 *     0x40..0x4F  chip probing, ours (../src/main/probe.c)
 *     0x50..0x5F  the card interface (../docs/card.md)
 */
enum frq1_command {
    FRQ1_CMD_IDENTIFY  = 0x10,
    FRQ1_CMD_START     = 0x11,
    FRQ1_CMD_STOP      = 0x12,
    FRQ1_CMD_SUSPEND   = 0x13,
    FRQ1_CMD_HEALTH    = 0x14,
    FRQ1_CMD_STATUS    = 0x20,
    FRQ1_CMD_CNR       = 0x21,
    FRQ1_CMD_STREAM    = 0x30,
    FRQ1_CMD_PEEK      = 0x40,
    FRQ1_CMD_POKE      = 0x41,
    FRQ1_CMD_BRINGUP   = 0x42,
    FRQ1_CMD_BRINGDOWN = 0x43,
    FRQ1_CMD_RING      = 0x44,
    FRQ1_CMD_I2CSCAN   = 0x45,
    FRQ1_CMD_I2CXFER   = 0x46,
    FRQ1_CMD_EPDMA     = 0x47,
    FRQ1_CMD_CARD_STATUS = 0x50,    /* card I/F (../docs/card.md)         */
    FRQ1_CMD_CARD_ON     = 0x51,
    FRQ1_CMD_CARD_OFF    = 0x52,
    FRQ1_CMD_CARD_PARAM  = 0x53,
    FRQ1_CMD_CARD_PUT    = 0x54,
    FRQ1_CMD_CARD_SEND   = 0x55,
    FRQ1_CMD_CARD_RECV   = 0x56,
    FRQ1_CMD_CARD_GET    = 0x57,
    FRQ1_CMD_ARM_STATUS  = 0x60,    /* the ARM cores (../docs/arm.md)     */
    FRQ1_CMD_ARM_HOLD    = 0x61,
    FRQ1_CMD_ARM_LOAD    = 0x62,
    FRQ1_CMD_ARM_RUN     = 0x63,
    FRQ1_CMD_ARM_CALL    = 0x64,    /* the work queue (../src/shared/armipc.h) */
    FRQ1_CMD_ARM_REAP    = 0x65,
    FRQ1_CMD_ARM_CANCEL  = 0x66,
    FRQ1_CMD_ARM_PUT     = 0x67,    /* the buffer (armipc.h, THE BUFFER)  */
    FRQ1_CMD_ARM_GET     = 0x68
};

/* The shape of the frame, for reference and for the size check.  Read it,
 * do not point it at the buffer. */
struct frq1_frame {
    uint32_t magic;                 /* +0x00 */
    uint32_t sequence;              /* +0x04 */
    uint32_t command;               /* +0x08 */
    int32_t  status;                /* +0x0c */
    uint32_t arg0;                  /* +0x10 */
    uint32_t arg1;                  /* +0x14 */
    uint8_t  data[FRQ1_DATA_BYTES]; /* +0x18 */
};
_Static_assert(sizeof(struct frq1_frame) == FRQ1_BYTES, "frq1_frame");

/* --- bytes ------------------------------------------------------------ */

static inline uint32_t frq1_get32(const uint8_t *f, unsigned off)
{
    return ((uint32_t)f[off] << 24) | ((uint32_t)f[off + 1u] << 16) |
           ((uint32_t)f[off + 2u] << 8) | (uint32_t)f[off + 3u];
}

static inline void frq1_put32(uint8_t *f, unsigned off, uint32_t v)
{
    f[off] = (uint8_t)(v >> 24);
    f[off + 1u] = (uint8_t)(v >> 16);
    f[off + 2u] = (uint8_t)(v >> 8);
    f[off + 3u] = (uint8_t)v;
}

static inline int32_t frq1_get32s(const uint8_t *f, unsigned off)
{
    uint32_t v = frq1_get32(f, off);

    /* Not a cast: a value above INT32_MAX is implementation-defined that
     * way, and these fields really are two's complement on the wire. */
    return (v & 0x80000000u) ? -(int32_t)(~v + 1u) : (int32_t)v;
}

/* Fill a request.  `data` may be NULL; whatever is not given is zero.
 * The status word goes out as FRQ1_ST_PENDING (zero), which is what makes
 * "answered" detectable at all. */
static inline void frq1_build(uint8_t f[FRQ1_BYTES], uint32_t sequence,
                              uint32_t command, uint32_t arg0, uint32_t arg1,
                              const uint8_t *data, size_t n)
{
    memset(f, 0, FRQ1_BYTES);
    frq1_put32(f, FRQ1_FR_MAGIC, FRQ1_MAGIC);
    frq1_put32(f, FRQ1_FR_SEQUENCE, sequence);
    frq1_put32(f, FRQ1_FR_COMMAND, command);
    frq1_put32(f, FRQ1_FR_STATUS, (uint32_t)FRQ1_ST_PENDING);
    frq1_put32(f, FRQ1_FR_ARG0, arg0);
    frq1_put32(f, FRQ1_FR_ARG1, arg1);
    if (data != NULL && n != 0u)
        memcpy(f + FRQ1_FR_DATA, data, n > FRQ1_DATA_BYTES
               ? FRQ1_DATA_BYTES : n);
}

/* Why a read-back is not a reply to (sequence, command).  NULL when it is
 * one, so a caller can print this straight. */
static inline const char *frq1_reply_check(const uint8_t *f,
                                           uint32_t sequence,
                                           uint32_t command)
{
    if (frq1_get32(f, FRQ1_FR_MAGIC) != FRQ1_MAGIC)
        return "no FRQ1 magic in the staging read-back";
    if (frq1_get32(f, FRQ1_FR_COMMAND) != command)
        return "the command word came back changed -- something else "
               "answered";
    if (frq1_get32(f, FRQ1_FR_SEQUENCE) != sequence)
        return "sequence mismatch: this is not the answer to our request";
    if (frq1_get32s(f, FRQ1_FR_STATUS) == FRQ1_ST_PENDING)
        return "the status is still PENDING: our request was read back "
               "unanswered";
    return NULL;
}

static inline const char *frq1_status_name(int32_t st)
{
    switch (st) {
    case FRQ1_ST_PENDING:   return "PENDING (unanswered)";
    case FRQ1_ST_OK:        return "OK";
    case FRQ1_E_NOTIMPL:    return "E_NOTIMPL (known command, no body)";
    case FRQ1_E_IO:         return "E_IO (the hardware did not answer)";
    case FRQ1_E_TIMEOUT:    return "E_TIMEOUT";
    case FRQ1_E_INVAL:      return "E_INVAL (bad argument)";
    case FRQ1_E_AGAIN:      return "E_AGAIN (not enough yet)";
    case FRQ1_E_BADCMD:     return "E_BADCMD (unknown command)";
    case FRQ1_E_BUSY:       return "E_BUSY (another core has it)";
    default:                return st > 0 ? "? (positive)" : "? (failed)";
    }
}

static inline const char *frq1_command_name(uint32_t cmd)
{
    switch (cmd) {
    case FRQ1_CMD_IDENTIFY:  return "IDENTIFY";
    case FRQ1_CMD_START:     return "START";
    case FRQ1_CMD_STOP:      return "STOP";
    case FRQ1_CMD_SUSPEND:   return "SUSPEND";
    case FRQ1_CMD_HEALTH:    return "HEALTH";
    case FRQ1_CMD_STATUS:    return "STATUS";
    case FRQ1_CMD_CNR:       return "CNR";
    case FRQ1_CMD_STREAM:    return "STREAM";
    case FRQ1_CMD_PEEK:      return "PEEK";
    case FRQ1_CMD_POKE:      return "POKE";
    case FRQ1_CMD_BRINGUP:   return "BRINGUP";
    case FRQ1_CMD_BRINGDOWN: return "BRINGDOWN";
    case FRQ1_CMD_RING:      return "RING";
    case FRQ1_CMD_I2CSCAN:   return "I2CSCAN";
    case FRQ1_CMD_I2CXFER:   return "I2CXFER";
    case FRQ1_CMD_EPDMA:     return "EPDMA";
    case FRQ1_CMD_CARD_STATUS: return "CARD_STATUS";
    case FRQ1_CMD_CARD_ON:     return "CARD_ON";
    case FRQ1_CMD_CARD_OFF:    return "CARD_OFF";
    case FRQ1_CMD_CARD_PARAM:  return "CARD_PARAM";
    case FRQ1_CMD_CARD_PUT:    return "CARD_PUT";
    case FRQ1_CMD_CARD_SEND:   return "CARD_SEND";
    case FRQ1_CMD_CARD_RECV:   return "CARD_RECV";
    case FRQ1_CMD_CARD_GET:    return "CARD_GET";
    case FRQ1_CMD_ARM_STATUS:  return "ARM_STATUS";
    case FRQ1_CMD_ARM_HOLD:    return "ARM_HOLD";
    case FRQ1_CMD_ARM_LOAD:    return "ARM_LOAD";
    case FRQ1_CMD_ARM_RUN:     return "ARM_RUN";
    case FRQ1_CMD_ARM_CALL:    return "ARM_CALL";
    case FRQ1_CMD_ARM_REAP:    return "ARM_REAP";
    case FRQ1_CMD_ARM_CANCEL:  return "ARM_CANCEL";
    case FRQ1_CMD_ARM_PUT:     return "ARM_PUT";
    case FRQ1_CMD_ARM_GET:     return "ARM_GET";
    default:                 return "?";
    }
}

/* --- IDENTIFY, ../src/main/device.c ------------------------------------ */

#define FRQ1_ID_VERSION     0x00u   /* u8  report version = 3 (1 had no   */
                                    /*     board / caps, 2 no build)      */
#define FRQ1_ID_PROTOCOL    0x01u   /* u8  the frame's version            */
#define FRQ1_ID_RECEIVERS   0x02u   /* u8  receive chains this build has   */
#define FRQ1_ID_SOURCES     0x03u   /* u8  mask of usable receive sources  */
#define FRQ1_ID_SIGNATURE   0x04u   /* u32 'FR01' -- which build answered  */
#define FRQ1_ID_BOARD       0x08u   /* u32 vid<<16|pid of the board row   */
                                    /*     built for, 0 = no product      */
#define FRQ1_ID_CAPS        0x0cu   /* u32 FRQ1_ID_CAP_*                  */
#define FRQ1_ID_BUILD       0x10u   /* u32 CRC-32 of the image's .text =  */
                                    /*     the .frc's `main` segment crc   */

#define FRQ1_ID_CAP_DIONE   0x01u   /* dione image (ts_dione.c, I2CXFER)  */
#define FRQ1_ID_CAP_CARD    0x02u   /* the card commands run on this row  */

#define FRQ1_ID_FR01        0x46523031u

/* IDENTIFY with arg0 = 1: the board's receive bands (its config row). */
#define FRQ1_IDB_VERSION    0x00u   /* u8  = 1                            */
#define FRQ1_IDB_PAGE       0x01u   /* u8  = 1                            */
#define FRQ1_IDB_COUNT      0x02u   /* u8  bands that follow, up to 2     */
#define FRQ1_IDB_FIRST      0x04u   /* the first band; each is 16 bytes:  */
#define FRQ1_IDB_BYTES      16u
#define FRQ1_IDB_SOURCE     0x00u   /* u8  0 T, 1 T2, 2 S, 3 S2           */
#define FRQ1_IDB_FLAGS      0x01u   /* u8  FRQ1_IDB_F_*                   */
#define FRQ1_IDB_RECORD     0x02u   /* u16 bytes per record (188 = TS)    */
#define FRQ1_IDB_LO_KHZ     0x04u   /* u32 lowest centre frequency        */
#define FRQ1_IDB_HI_KHZ     0x08u   /* u32 highest centre frequency       */
#define FRQ1_IDB_STEP_HZ    0x0cu   /* u32 the tuner's step               */
#define FRQ1_IDB_F_HOST     0x01u   /* the host tunes it (I2CXFER)        */
#define FRQ1_IDB_F_LOCK     0x02u   /* a lock was measured inside it      */

/* --- HEALTH, ../src/main/health.c -------------------------------------- */

#define FRQ1_HR_VERSION     0x00u   /* u8  = 1                            */
#define FRQ1_HR_PHASE       0x01u   /* u8  0 loaded, 1 started, 2 stream  */
#define FRQ1_HR_FLAGS       0x02u   /* u16 FRQ1_HR_F_*                    */
#define FRQ1_HR_FRAMES      0x04u   /* u32 frames answered since load     */
#define FRQ1_HR_PARTS       0x08u   /* 7 x 4 bytes: s8 first failure,     */
                                    /*   u8 its command, u16 count        */
#define FRQ1_HR_NPARTS      7u      /* device fe ts probe card arm other  */
#define FRQ1_HR_FIRST       0x24u   /* s8 the first part's failure in     */
                                    /*   that order, u8 part (0xff none), */
                                    /*   u8 its command                   */
#define FRQ1_HR_F_STARTED   0x0001u
#define FRQ1_HR_F_TS        0x0002u
#define FRQ1_HR_F_CARD_ON   0x0004u
#define FRQ1_HR_F_CARD_ARM  0x0008u
#define FRQ1_HR_F_DIONE     0x0010u
#define FRQ1_HR_F_FAILED    0x0020u
#define FRQ1_HR_CLEAR       0x01u   /* arg0: clear after reporting        */

/* --- STATUS / CNR, ../src/main/fe.c ------------------------------------ */

#define FRQ1_FE_VERSION     0x00u   /* u8  = 1                            */
#define FRQ1_FE_FLAGS       0x01u   /* u8                                 */
#define FRQ1_FE_SOURCE      0x02u   /* u8  the device index, 0..3          */
#define FRQ1_FE_LOCKED      0x04u   /* u32 0 or 1                         */
#define FRQ1_FE_CNR         0x08u   /* u32 RAW register value, not dB     */
#define FRQ1_FE_LOCK_RC     0x0cu   /* s32                                */
#define FRQ1_FE_CNR_RC      0x10u   /* s32                                */

#define FRQ1_FE_F_LOCK_OK   0x01u   /* the lock read answered             */
#define FRQ1_FE_F_CNR_OK    0x02u   /* the C/N read answered              */
#define FRQ1_FE_F_PRESENT   0x04u   /* something is at that address       */
#define FRQ1_FE_F_NOINIT    0x80u   /* START has not run                  */

/* --- STREAM, ../src/main/stream.c -------------------------------------- */

#define FRQ1_SR_VERSION     0x00u   /* u8  = 1                            */
#define FRQ1_SR_FLAGS       0x01u   /* u8                                 */
#define FRQ1_SR_BURSTS      0x04u   /* u32 bursts that went out           */
#define FRQ1_SR_BYTES       0x08u   /* u32 bytes that went out            */
#define FRQ1_SR_USED        0x0cu   /* u32 bytes still in the ring        */
#define FRQ1_SR_RC          0x10u   /* s32 why it stopped                 */

#define FRQ1_SR_F_RUNNING   0x01u   /* the capture block is up            */
#define FRQ1_SR_F_SYNCED    0x02u   /* the record boundary is known       */
#define FRQ1_SR_F_DRAINED   0x04u   /* stopped: nothing was ready         */

#define FRQ1_SR_BURSTS_MAX  64u     /* the firmware's ceiling, 32 KiB     */
#define FRQ1_SR_BURST_BYTES 512u    /* one burst out of the endpoint FIFO */

/* STREAM on dione (the GV-M2TV), ../src/main/ts_dione.c.  Same command, a
 * different report: dione captures into a circular DRAM ring, and the
 * firmware hands each burst back through the ring's read offset.
 *
 * arg0 = bursts to move in this frame (0 = 64, at most 8192).
 * arg1 = ring length in MiB, read by the first frame only (0 = 16, 1..16).
 *
 * A frame waits for room in the endpoint FIFO, so the host must already be
 * reading the bulk IN endpoint on another thread when it sends one. */
#define FRQ1_DSR_VERSION    0x00u   /* u8  = 3 (1 had no FULLS, MISSES,   */
                                    /*     PACKETS; 2 had LENGTH, not     */
                                    /*     TEIS)                          */
#define FRQ1_DSR_FLAGS      0x01u   /* u8                                 */
#define FRQ1_DSR_FULLS      0x02u   /* u8  times the ring was found full: */
                                    /*     the DMA stopped, input lost    */
#define FRQ1_DSR_MISSES     0x03u   /* u8  packet starts without 0x47     */
#define FRQ1_DSR_BURSTS     0x04u   /* u32 bursts in this frame           */
#define FRQ1_DSR_BYTES      0x08u   /* u32 bytes in this frame            */
#define FRQ1_DSR_WRITE      0x0cu   /* u32 the capture's write offset     */
#define FRQ1_DSR_RC         0x10u   /* s32 why the frame failed, 0 if not */
#define FRQ1_DSR_LAPS       0x14u   /* u32 times the reader wrapped       */
#define FRQ1_DSR_READ       0x18u   /* u32 the firmware's read offset     */
#define FRQ1_DSR_LENGTH     0x1cu   /* u32 version 2: the ring length     */
#define FRQ1_DSR_TEIS       0x1cu   /* u32 version 3: of PACKETS, those   */
                                    /*     with TEI set (the demod could  */
                                    /*     not correct them)              */
#define FRQ1_DSR_PEAK       0x20u   /* u32 most bytes seen waiting in the */
                                    /*     ring since the start           */
#define FRQ1_DSR_PACKETS    0x24u   /* u32 packet starts sent with 0x47   */
/* When the frame failed with -2 (the band did not come up), READ and LENGTH
 * instead carry the register whose read-back disagreed and what it read. */
#define FRQ1_DSR_BAD_REG    FRQ1_DSR_READ
#define FRQ1_DSR_BAD_VAL    FRQ1_DSR_LENGTH

#define FRQ1_DSR_F_RUNNING  0x01u   /* the capture is up                  */
#define FRQ1_DSR_F_EP81     0x10u   /* the TS goes out of EP 0x81         */
#define FRQ1_DSR_F_EP84     0x20u   /* the TS goes out of EP 0x84         */
/* dione STREAM's request data byte 0: the endpoints (0 = EP 0x81). */
#define FRQ1_DSO_EP81       0x01u
#define FRQ1_DSO_EP84       0x02u
#define FRQ1_DSR_F_SHORT    0x04u   /* fewer bursts than asked: the FIFO  */
                                    /*     stayed full too long           */

#define FRQ1_DSR_BURSTS_DEF 64u
#define FRQ1_DSR_BURSTS_MAX 8192u
#define FRQ1_DSR_RING_MAX   16u     /* MiB                                */

/* --- the 0x40 band: chip probing, ../src/main/probe.c ------------------ */

/* These are ours and nobody else's: a frame the firmware answers only
 * because it is the firmware.  Each report starts with a version byte, and
 * the one thing a host must not do is assume a layout without reading it. */

#define FRQ1_W_WORD         0u          /* ld   -- 4-byte alignment       */
#define FRQ1_W_HALF         1u          /* lduh -- 2-byte alignment       */
#define FRQ1_W_BYTE         2u          /* ldub                           */

/* A misaligned address is refused with E_INVAL rather than read from the
 * wrong place, so a host can ask freely. */
static inline unsigned frq1_width_stride(unsigned width)
{
    return width == FRQ1_W_WORD ? 4u : (width == FRQ1_W_HALF ? 2u : 1u);
}

/* PEEK (0x40): arg0 = address, arg1 = width << 8 | count (1..8). */
#define FRQ1_PEEK_MAX       8u
#define FRQ1_PEEK_CTL(width, count) (((uint32_t)(width) << 8) | (uint32_t)(count))

#define FRQ1_PK_VERSION     0x00u   /* u8  = 1                            */
#define FRQ1_PK_COUNT       0x01u   /* u8  values that follow             */
#define FRQ1_PK_WIDTH       0x02u   /* u8  FRQ1_W_*                       */
#define FRQ1_PK_ADDR        0x04u   /* u32 the address, echoed            */
#define FRQ1_PK_VALUES      0x08u   /* u32 x 8, narrow widths in the low  */
                                    /*     bits                           */

/* POKE (0x41): arg0 = address, arg1 = width << 8 | flags, and the FIRST
 * DATA WORD carries the value.  Flag bit0 asks for the old value to be put
 * back after the read-back. */
#define FRQ1_POKE_RESTORE   0x01u
#define FRQ1_POKE_CTL(width, flags) (((uint32_t)(width) << 8) | (uint32_t)(flags))

#define FRQ1_PO_VERSION     0x00u   /* u8  = 1                            */
#define FRQ1_PO_WIDTH       0x01u   /* u8  FRQ1_W_*                       */
#define FRQ1_PO_RESTORED    0x02u   /* u8  whether it put the old back    */
#define FRQ1_PO_ADDR        0x04u   /* u32                                */
#define FRQ1_PO_OLD         0x08u   /* u32 before the write               */
#define FRQ1_PO_WROTE       0x0cu   /* u32 what went out                  */
#define FRQ1_PO_READBACK    0x10u   /* u32 what came back                 */
#define FRQ1_PO_AFTER       0x14u   /* u32 after the restore, else 0      */

/* BRINGUP (0x42) / BRINGDOWN (0x43): not in the images built from this
 * tree, which answer FRQ1_E_BADCMD.  arg0 = bitmask of the steps to play,
 * 0 for a report with nothing played.  The reply says how many steps the
 * firmware's table has, so a host never has to hold that number. */
#define FRQ1_BU_VERSION     0x00u   /* u8  = 1                            */
#define FRQ1_BU_DONE        0x01u   /* u8  steps actually played          */
#define FRQ1_BU_STEPS       0x02u   /* u8  steps in the firmware's table  */
#define FRQ1_BU_PROBES      0x03u   /* u8  observation points reported    */
#define FRQ1_BU_MASK        0x04u   /* u32 the mask, echoed               */
#define FRQ1_BU_VALUES      0x08u   /* u32 x 8, read AFTER the writes     */
#define FRQ1_BU_MAX         8u

/* RING (0x44): arg0 bit0 arms the capture block, 0 reads the registers. */
#define FRQ1_RING_ARM       0x01u

#define FRQ1_RG_VERSION     0x00u   /* u8  = 1                            */
#define FRQ1_RG_ARMED       0x01u   /* u8  whether it was asked to arm    */
#define FRQ1_RG_FAILED      0x02u   /* u8  nonzero: arming did not take   */
/* EPDMA (0x47).  arg1 packs the three numbers; the report is six DMA words
 * then the endpoint's pair. */
#define FRQ1_EPD_ARG(len, ep, steps) \
    (((unsigned)(len) & 0xffffu) | (((unsigned)(ep) & 0xfu) << 16) \
     | (((unsigned)(steps) & 0x3fu) << 24))
#define FRQ1_EPD_STEPS_ALL  0x3fu
#define FRQ1_EPD_CMD        0x04u   /* u32 0x00FD8500 after the steps      */
#define FRQ1_EPD_RUN        0x08u   /* u32 0x00FD8504                      */
#define FRQ1_EPD_ADDR       0x0cu   /* u32 0x00FD8508 -- advanced?         */
#define FRQ1_EPD_LEN        0x10u   /* u32 0x00FD8510                      */
#define FRQ1_EPD_CTL        0x14u   /* u32 0x00FD8514                      */
#define FRQ1_EPD_CAUSE      0x18u   /* u32 0x00FD8094 -- bit2 = a chunk    */
#define FRQ1_EPD_EPCFG      0x1cu   /* u32 the endpoint's control          */
#define FRQ1_EPD_EPSTAT     0x20u   /* u32 the endpoint's status           */

#define FRQ1_RG_MODE        0x04u   /* u32 the mode word                  */
#define FRQ1_RG_BASE        0x08u   /* u32 ring base, PHYSICAL            */
#define FRQ1_RG_LENGTH      0x0cu   /* u32 ring length in bytes           */
#define FRQ1_RG_WRITE       0x10u   /* u32 the producer's pointer         */
#define FRQ1_RG_READ        0x14u   /* u32 the consumer's pointer         */
#define FRQ1_RG_USED        0x18u   /* u32 bytes the firmware thinks are  */
                                    /*     ready                          */
#define FRQ1_RG_R010        0x1cu   /* u32 armed with the ring base       */

/* I2CSCAN (0x45): arg0 = the first 7-bit address, arg1 = count in the low
 * byte (0 means 16, at most 32) with bit8 picking the 0x00FD8FE0 instance
 * instead of 0x01684FE0.  One window per request on purpose: 128 addresses
 * in one go does not come back. */
#define FRQ1_SCAN_MAX       32u
#define FRQ1_SCAN_FD8F      0x100u
#define FRQ1_SCAN_CTL(count, fd8f) \
    ((uint32_t)(count) | ((fd8f) ? (uint32_t)FRQ1_SCAN_FD8F : 0u))

#define FRQ1_SC_VERSION     0x00u   /* u8  = 1                            */
#define FRQ1_SC_WHICH       0x01u   /* u8  1 = the 0x00FD8FE0 instance    */
#define FRQ1_SC_COUNT       0x02u   /* u8  addresses tried                */
#define FRQ1_SC_FIRST       0x03u   /* u8  the first of them              */
#define FRQ1_SC_BASE        0x04u   /* u32 the controller's base          */
#define FRQ1_SC_SETUP_RC    0x08u   /* s32 fj_setup()'s result            */
#define FRQ1_SC_ACKS        0x0cu   /* u32 bit i set = first + i answered */
#define FRQ1_SC_LAST_RC     0x10u   /* s32 the last transfer's result     */

/* I2CXFER (0x46), dione only: one transfer on the 0x00FD8FE0 instance.
 * arg0 = 7-bit address | write count << 8 | read count << 16.  The bytes to
 * write are the data field; the bytes read come back at its start, after a
 * repeated START.  Write + read is 1..40.  The first frame also powers the
 * front end and brings the I2C block up.  On selene: E_NOTIMPL. */
#define FRQ1_I2CX_MAX       40u
#define FRQ1_I2CX_ARG(addr, wn, rn) \
    (((uint32_t)(addr) & 0x7fu) | ((uint32_t)(wn) << 8) | \
     ((uint32_t)(rn) << 16))

/* --- the stream itself ------------------------------------------------- */

#define FRQ1_TS_PACKET      188u    /* what the host gets                 */
#define FRQ1_TS_RECORD      192u    /* what the capture block writes       */
#define FRQ1_TS_SYNC        0x47u

#endif /* FRQ1_HOST_H */
