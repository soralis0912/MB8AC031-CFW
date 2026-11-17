/* chipio.h -- the host side of the chip's USB window.  Header-only.
 *
 * Both sample programs sit on this: frc1_load.c writes an FRC1 container
 * into the chip through it, ts_grab.c talks to the firmware that container
 * installed.  It is the libusb side of the window, with limits that come
 * from measurements on the devices.
 *
 * THE WINDOW.  Vendor request 0xBC is a 16-bit
 * register window with a 24-bit address:
 *
 *     wValue = ((addr >> 16) & 0xFFFF) << 8 | sub-opcode
 *     wIndex = addr & 0xFFFF
 *
 * so it reaches the whole 16 MB space and nothing above it.  wValue's LOW
 * byte is a sub-opcode, not padding: 0 is the plain window, 1 is
 * read-modify-write, and 0xC1 is the 64-byte staging dispatch that our own
 * firmware answers (../src/main/entry_frc1_selene.S gates on exactly that).
 *
 * TWO LOADER GENERATIONS, TWO SETS OF RULES.  `selene` (MB8AC031 / MB86M0x)
 * takes a block write of up to 64 bytes and carries the frame in the OUT
 * transfer.  `dione` (MB8AC018 / MB86H5x, measured on a GV-M2TV) advances a
 * word per transfer whatever wLength says, scatters a multi-word OUT, and
 * cannot reach DRAM from the loader state; so every write is one aligned
 * word, and a command frame goes into a mailbox of our own and is rung with
 * a four-byte sub-opcode 0xC1.  `struct chipio.dione` picks the rules, and
 * only the USB id measured for it sets that flag.
 *
 * THE LIMITS ARE NOT TASTE.  Reads are capped at 32 bytes because this
 * project recorded a hard hang -- one needing a physical replug -- on
 * register reads past 1024.  Block writes are capped at 64 bytes, 4-byte
 * aligned.  Transfers retry with a backoff because this device returns
 * LIBUSB_ERROR_NOT_FOUND now and then, and a run that dies half way through
 * writing a segment leaves the chip holding half an image.
 *
 * BUILDING WITHOUT LIBUSB.  Define nothing and every call fails with a
 * message saying so, which keeps frc1_load.c's offline commands (and
 * ../tests/test_frc1.py, which compiles it with just a C compiler) working
 * with no dependency at all.  The Makefile defines CHIPIO_HAVE_LIBUSB when
 * pkg-config finds libusb-1.0, and then the same file really installs.
 *
 * Needs C11.  With CHIPIO_HAVE_LIBUSB it also needs _POSIX_C_SOURCE for
 * nanosleep; the Makefile passes it.
 */
#ifndef CHIPIO_H
#define CHIPIO_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifdef CHIPIO_HAVE_LIBUSB
#include <time.h>
#include <libusb.h>
#endif

#define CHIPIO_REQ_LSI0     0xBCu   /* the main LSI's window              */
#define CHIPIO_REQ_LSI1     0xBDu   /* the second one, where there is one  */

#define CHIPIO_READ_MAX     32u     /* never raise this                   */
#define CHIPIO_BLOCK_MAX    64u     /* a block write's ceiling            */
#define CHIPIO_TRIES        6       /* then give up                       */

#define CHIPIO_SUBOP_PLAIN    0x00u
#define CHIPIO_SUBOP_MASK16   0x01u
#define CHIPIO_SUBOP_DISPATCH 0xC1u

/* Two registers worth naming: the chip says what container magic it eats,
 * and whether one has booted.  Both are read-only here. */
#define CHIPIO_REG_CHIPID   0x00080000u
#define CHIPIO_REG_STATE    0x00082008u

/* The one `dione` (MB86H5x) machine whose window rules have been measured:
 * a GV-M2TV.  Only this id gets them -- guessing from a generation would
 * let an unmeasured part be written with the wrong transfer size. */
#define CHIPIO_DIONE_VID    0x04BBu
#define CHIPIO_DIONE_PID    0x053Au

static inline int chipio_is_dione(unsigned vid, unsigned pid)
{
    return vid == CHIPIO_DIONE_VID && pid == CHIPIO_DIONE_PID;
}

/* Where a frame goes on the selene loader (MB8AC031).
 *
 *   CHIPIO_SELENE_STAGING  the loader's own 64-byte OUT buffer.  One for
 *                          everybody: two programs at once overwrite each
 *                          other's reply between the OUT and the read-back
 *   CHIPIO_SELENE_MBOX     four mailboxes of our CFW, + 0x40 * k, in the
 *                          uncached view of our .bss.  The frame goes in
 *                          with plain writes and a four-byte doorbell
 *                          runs it, as on dione
 *
 * Measured on the PIX-DT295 only, so only that id defaults to a mailbox. */
#define CHIPIO_SELENE_STAGING 0x000C4620u
#define CHIPIO_SELENE_MBOX    0x003B0000u
#define CHIPIO_DT295_VID      0x06B8u
#define CHIPIO_DT295_PID      0x103Bu

/* -> the mailbox a stream/probe tool should use by default on this id, or
 * 0 for "the staging buffer". */
static inline unsigned chipio_default_mbox(unsigned vid, unsigned pid)
{
    if (vid == CHIPIO_DIONE_VID && pid == CHIPIO_DIONE_PID)
        return 0x000C5000u;
    if (vid == CHIPIO_DT295_VID && pid == CHIPIO_DT295_PID)
        return CHIPIO_SELENE_MBOX;
    return 0;
}

struct chipio {
    void       *handle;         /* libusb_device_handle *                 */
    void       *ctx;            /* libusb_context *                       */
    unsigned    request;        /* CHIPIO_REQ_LSI0 or _LSI1               */
    unsigned    vid, pid;
    int         claimed;        /* interface 0 is held (bulk needs it)    */
    int         dione;          /* 1 = MB8AC018/MB86H5x window rules      */
    char        err[192];
};

/* ---------------------------------------------------------------- no libusb */
#ifndef CHIPIO_HAVE_LIBUSB

static inline int chipio_unavailable(struct chipio *c)
{
    snprintf(c->err, sizeof(c->err),
             "built without libusb: no transport.  Install libusb-1.0 and "
             "rebuild (see README.md)");
    return -1;
}

static inline int chipio_open(struct chipio *c, unsigned vid, unsigned pid)
{
    memset(c, 0, sizeof(*c));
    c->vid = vid;
    c->pid = pid;
    c->dione = chipio_is_dione(vid, pid);
    return chipio_unavailable(c);
}

static inline void chipio_close(struct chipio *c) { (void)c; }

static inline int chipio_read(struct chipio *c, uint32_t a, uint8_t *b,
                              uint32_t n)
{
    (void)a; (void)b; (void)n;
    return chipio_unavailable(c);
}

static inline int chipio_write(struct chipio *c, uint32_t a,
                               const uint8_t *b, uint32_t n)
{
    (void)a; (void)b; (void)n;
    return chipio_unavailable(c);
}

static inline int chipio_dispatch(struct chipio *c, uint32_t staging,
                                  const uint8_t *out, uint8_t *in)
{
    (void)staging; (void)out; (void)in;
    return chipio_unavailable(c);
}

static inline int chipio_hook_write(struct chipio *c, uint32_t addr,
                                    const uint8_t *b, uint32_t n,
                                    int restoring)
{
    (void)addr; (void)b; (void)n; (void)restoring;
    return chipio_unavailable(c);
}

static inline int chipio_bulk(struct chipio *c, unsigned ep, uint8_t *buf,
                              int len, unsigned ms, int *got)
{
    (void)ep; (void)buf; (void)len; (void)ms; (void)got;
    return chipio_unavailable(c);
}

static inline int chipio_claim(struct chipio *c) { return chipio_unavailable(c); }

static inline void chipio_nap(unsigned ms) { (void)ms; }

static inline const char *chipio_backend(void) { return "none (no libusb)"; }

#else  /* ------------------------------------------------------- with libusb */

static inline const char *chipio_backend(void) { return "libusb-1.0"; }

static inline void chipio_nap(unsigned ms)
{
    struct timespec ts;

    ts.tv_sec = (time_t)(ms / 1000u);
    ts.tv_nsec = (long)(ms % 1000u) * 1000000L;
    (void)nanosleep(&ts, NULL);
}

static inline int chipio_open(struct chipio *c, unsigned vid, unsigned pid)
{
    libusb_context *ctx = NULL;
    libusb_device_handle *h;
    int rc;

    memset(c, 0, sizeof(*c));
    c->request = CHIPIO_REQ_LSI0;
    c->vid = vid;
    c->pid = pid;
    c->dione = chipio_is_dione(vid, pid);

    rc = libusb_init(&ctx);
    if (rc != 0) {
        snprintf(c->err, sizeof(c->err), "libusb_init: %s",
                 libusb_error_name(rc));
        return -1;
    }
    h = libusb_open_device_with_vid_pid(ctx, (uint16_t)vid, (uint16_t)pid);
    if (h == NULL) {
        snprintf(c->err, sizeof(c->err),
                 "%04x:%04x is not there, or this user cannot open it",
                 vid, pid);
        libusb_exit(ctx);
        return -1;
    }
    c->ctx = ctx;
    c->handle = h;
    return 0;
}

static inline void chipio_close(struct chipio *c)
{
    if (c->handle != NULL) {
        if (c->claimed)
            (void)libusb_release_interface((libusb_device_handle *)c->handle,
                                           0);
        libusb_close((libusb_device_handle *)c->handle);
        c->handle = NULL;
    }
    if (c->ctx != NULL) {
        libusb_exit((libusb_context *)c->ctx);
        c->ctx = NULL;
    }
    c->claimed = 0;
}

/* Bulk needs the interface; EP0 does not, so this is only called by the
 * paths that read a stream.  Keeping it out of chipio_open() means a
 * firmware install still works on a machine where something else holds the
 * interface. */
static inline int chipio_claim(struct chipio *c)
{
    int rc;

    if (c->claimed)
        return 0;
    if (c->handle == NULL) {
        snprintf(c->err, sizeof(c->err), "no device is open");
        return -1;
    }
    rc = libusb_claim_interface((libusb_device_handle *)c->handle, 0);
    if (rc != 0) {
        snprintf(c->err, sizeof(c->err),
                 "cannot claim interface 0: %s%s", libusb_error_name(rc),
                 rc == LIBUSB_ERROR_ACCESS || rc == LIBUSB_ERROR_BUSY
                 ? " (a driver for this device is loaded)" : "");
        return -1;
    }
    c->claimed = 1;
    return 0;
}

/* One windowed control transfer, with the retry the device needs. */
static inline int chipio_xfer(struct chipio *c, uint8_t dir, uint32_t addr,
                              unsigned subop, uint8_t *data, uint16_t len,
                              unsigned timeout)
{
    uint16_t wvalue = (uint16_t)((((addr >> 16) & 0xFFu) << 8) | subop);
    uint16_t windex = (uint16_t)(addr & 0xFFFFu);
    int rc = LIBUSB_ERROR_OTHER;
    int i;

    if (c->handle == NULL) {
        snprintf(c->err, sizeof(c->err), "no device is open");
        return -1;
    }
    for (i = 0; i < CHIPIO_TRIES; i++) {
        rc = libusb_control_transfer((libusb_device_handle *)c->handle, dir,
                                     (uint8_t)c->request, wvalue, windex,
                                     data, len, timeout);
        if (rc >= 0)
            break;
        chipio_nap(50u * (unsigned)(i + 1));
    }
    if (rc < 0) {
        snprintf(c->err, sizeof(c->err), "%s at %#010x (sub-opcode %#x): %s",
                 dir == 0xC0 ? "read" : "write", addr, subop,
                 libusb_error_name(rc));
        return -1;
    }
    if ((uint16_t)rc != len) {
        snprintf(c->err, sizeof(c->err),
                 "%s at %#010x moved %d of %u bytes",
                 dir == 0xC0 ? "read" : "write", addr, rc, len);
        return -1;
    }
    return 0;
}

static inline int chipio_read(struct chipio *c, uint32_t addr, uint8_t *buf,
                              uint32_t n)
{
    uint32_t done = 0;

    /* The dione loader's read decodes words from the address it was given:
     * an unaligned request returns the word that contains it, and the
     * loader always moves a whole word.  Read the containing word and trim,
     * never write the rounded-up tail into the caller's buffer. */
    if (c->dione) {
        if ((addr & 3u) != 0u) {
            uint8_t tmp[4];
            uint32_t skew = addr & 3u;
            uint32_t take = 4u - skew;

            if (take > n)
                take = n;
            if (chipio_xfer(c, 0xC0, addr - skew, CHIPIO_SUBOP_PLAIN, tmp,
                            4u, 800u) != 0)
                return -1;
            memcpy(buf, tmp + skew, take);
            done = take;
        }
        while (done < n) {
            uint32_t left = n - done;

            if (left >= 4u) {
                uint32_t k = left < CHIPIO_READ_MAX ? left & ~3u
                                                    : CHIPIO_READ_MAX;

                if (chipio_xfer(c, 0xC0, addr + done, CHIPIO_SUBOP_PLAIN,
                                buf + done, (uint16_t)k, 800u) != 0)
                    return -1;
                done += k;
            } else {
                uint8_t tmp[4];

                if (chipio_xfer(c, 0xC0, addr + done, CHIPIO_SUBOP_PLAIN,
                                tmp, 4u, 800u) != 0)
                    return -1;
                memcpy(buf + done, tmp, left);
                done += left;
            }
        }
        return 0;
    }

    while (done < n) {
        uint32_t k = n - done < CHIPIO_READ_MAX ? n - done : CHIPIO_READ_MAX;

        if (chipio_xfer(c, 0xC0, addr + done, CHIPIO_SUBOP_PLAIN, buf + done,
                        (uint16_t)k, 800u) != 0)
            return -1;
        done += k;
    }
    return 0;
}

/* Blocks where the alignment allows it, 16-bit words where it does not.
 * The loader's handler takes whatever length the transfer carries as long as
 * the address is 4-byte aligned and the length a multiple of 4; a 64-byte
 * block is 32x fewer transfers than a word at a time, which is the whole
 * difference between a few seconds and a few minutes for a real image.
 *
 * ON dione IT IS ALWAYS ONE WORD PER TRANSFER.  That loader's copy loop
 * advances four bytes whatever wLength says, and its EP0 OUT reader scatters
 * a multi-word payload (word i lands at buffer + 16*i), so an 8-byte write
 * moves word 0 and leaves stale buffer content in the next four bytes --
 * measured: it half-writes a hook.  A partial word is merged with what is
 * already there rather than zero-filled, so a call cannot clear a neighbour. */
static inline int chipio_write(struct chipio *c, uint32_t addr,
                               const uint8_t *buf, uint32_t n)
{
    uint8_t tmp[CHIPIO_BLOCK_MAX];
    uint32_t done = 0;

    if (c->dione) {
        while (done < n) {
            uint32_t here = addr + done;
            uint32_t off = here & 3u;
            uint32_t left = n - done;
            uint32_t take = 4u - off;

            if (take > left)
                take = left;
            if (off == 0u && take == 4u) {
                memcpy(tmp, buf + done, 4);
            } else {
                if (chipio_read(c, here - off, tmp, 4u) != 0)
                    return -1;
                memcpy(tmp + off, buf + done, take);
            }
            if (chipio_xfer(c, 0x40, here - off, CHIPIO_SUBOP_PLAIN, tmp,
                            4u, 1000u) != 0)
                return -1;
            done += take;
        }
        return 0;
    }

    while (done < n) {
        uint32_t left = n - done;
        uint32_t here = addr + done;
        uint32_t k;

        if ((here & 3u) == 0u && left >= 4u) {
            k = left < CHIPIO_BLOCK_MAX ? left & ~3u : CHIPIO_BLOCK_MAX;
            memcpy(tmp, buf + done, k);
            if (chipio_xfer(c, 0x40, here, CHIPIO_SUBOP_PLAIN, tmp,
                            (uint16_t)k, 1000u) != 0)
                return -1;
            done += k;
            continue;
        }
        if ((here & 1u) != 0u) {
            snprintf(c->err, sizeof(c->err),
                     "%#010x is not 2-byte aligned; the window writes words",
                     here);
            return -1;
        }
        /* A trailing odd byte would mean a read-modify-write on a half we
         * were not asked to touch, so it is refused rather than guessed. */
        if (left < 2u) {
            snprintf(c->err, sizeof(c->err),
                     "a single trailing byte at %#010x cannot be written "
                     "without touching its neighbour", here);
            return -1;
        }
        tmp[0] = buf[done];
        tmp[1] = buf[done + 1];
        if (chipio_xfer(c, 0x40, here, CHIPIO_SUBOP_PLAIN, tmp, 2u, 800u) != 0)
            return -1;
        done += 2u;
    }
    return 0;
}

/* The 64-byte staging dispatch (sub-opcode 0xC1).  With no firmware of ours
 * hooked, the resident loader treats it like the plain window and stores our
 * 64 bytes AT the staging address, which is why a reply that is our own
 * request verbatim means nothing answered.  ts_grab.c checks for exactly
 * that rather than reporting a reply that is really an echo.
 *
 * Only the OUT half retries -- chipio_read already retries, and a failure
 * here is libusb refusing the submit, so the firmware never saw the packet
 * and resending is safe even for a command that must not run twice. */
static inline int chipio_dispatch(struct chipio *c, uint32_t staging,
                                  const uint8_t *out, uint8_t *in)
{
    uint8_t packet[64];
    uint8_t door[4] = { 0, 0, 0, 0 };

    /* A mailbox: the frame goes in with plain writes, then sub-opcode 0xC1
     * with exactly four bytes aimed at that address rings it (what
     * entry_frc1_dione.S and entry_frc1_selene.S check).  dione has nothing
     * else: that loader scatters a multi-word OUT payload and walks over the
     * fields behind its buffer.  On selene anything but the loader's own
     * buffer is one of our mailboxes. */
    if (c->dione || staging != CHIPIO_SELENE_STAGING) {
        if (chipio_write(c, staging, out, 64u) != 0)
            return -1;
        if (chipio_xfer(c, 0x40, staging, CHIPIO_SUBOP_DISPATCH, door,
                        (uint16_t)sizeof(door), 20000u) != 0)
            return -1;
        return chipio_read(c, staging, in, 64u);
    }

    memcpy(packet, out, sizeof(packet));
    if (chipio_xfer(c, 0x40, staging, CHIPIO_SUBOP_DISPATCH, packet,
                    (uint16_t)sizeof(packet), 20000u) != 0)
        return -1;
    return chipio_read(c, staging, in, 64u);
}

/* Write the eight bytes of a hook so that the chip never executes a state
 * that is neither the old dispatcher nor the new stub.
 *
 * On selene that is one 8-byte block write.  On dione the two words cannot go
 * in one transfer, and between them the chip runs whatever those 8 bytes
 * say for the second write's own transfer.  Exactly one intermediate is
 * safe:
 *
 *     9b ec 10 00   ldi:20 #target,r12     the stub's first word
 *     (unchanged)   the loader's original second word
 *
 * with which the loader still performs the plain window write the second
 * transfer asks for -- as long as the hook word's second byte is not 0x01
 * (which would pick the read-modify-write path).  The install/restore round
 * trip on a GV-M2TV goes through exactly this state.  Installing therefore writes word 0 first; restoring writes word 1
 * first.  `restoring` selects which. */
static inline int chipio_hook_write(struct chipio *c, uint32_t addr,
                                    const uint8_t *b, uint32_t n,
                                    int restoring)
{
    uint32_t order[2];
    int i;

    if (!c->dione)
        return chipio_write(c, addr, b, n);
    if (n != 8u) {
        snprintf(c->err, sizeof(c->err),
                 "a hook is 8 bytes, not %u", n);
        return -1;
    }
    order[0] = restoring ? 4u : 0u;
    order[1] = restoring ? 0u : 4u;
    for (i = 0; i < 2; i++) {
        uint32_t k = order[i];

        if (b[k + 1u] == 0x01u) {
            snprintf(c->err, sizeof(c->err),
                     "hook word at +%u is %02x%02x%02x%02x: its second byte "
                     "0x01 would send the intermediate state down the "
                     "read-modify-write path", k, b[k], b[k + 1u], b[k + 2u],
                     b[k + 3u]);
            return -1;
        }
        if (chipio_xfer(c, 0x40, addr + k, CHIPIO_SUBOP_PLAIN,
                        (uint8_t *)(b + k), 4u, 1000u) != 0)
            return -1;
    }
    return 0;
}

/* One bulk read.  A timeout with bytes in hand is not an error: that is what
 * a stream that has nothing more right now looks like. */
static inline int chipio_bulk(struct chipio *c, unsigned ep, uint8_t *buf,
                              int len, unsigned ms, int *got)
{
    int moved = 0;
    int rc;

    *got = 0;
    if (chipio_claim(c) != 0)
        return -1;
    rc = libusb_bulk_transfer((libusb_device_handle *)c->handle,
                              (unsigned char)ep, buf, len, &moved, ms);
    *got = moved;
    if (rc == 0 || (rc == LIBUSB_ERROR_TIMEOUT && moved > 0))
        return 0;
    if (rc == LIBUSB_ERROR_TIMEOUT)
        return 1;               /* nothing ready, not a failure */
    snprintf(c->err, sizeof(c->err), "bulk read on ep %#02x: %s", ep,
             libusb_error_name(rc));
    return -1;
}

#endif /* CHIPIO_HAVE_LIBUSB */

/* --- things both programs want to say about a chip --------------------- */

/* The eight bytes at CHIPIO_REG_CHIPID are the container magic this part
 * accepts: "MB8AC031" on the MB86M0x generation, "MB8AC018" on the MB86H5x
 * one.  Printable, so it is worth showing
 * as text. */
static inline void chipio_magic_text(const uint8_t *b, char out[9])
{
    int i;

    for (i = 0; i < 8; i++)
        out[i] = (b[i] >= 32 && b[i] < 127) ? (char)b[i] : '.';
    out[8] = 0;
}

#endif /* CHIPIO_H */
