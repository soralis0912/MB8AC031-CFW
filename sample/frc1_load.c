/* frc1_load.c -- the firmware loader.  Reads an `FRC1` container and puts it
 * on the chip.
 *
 * The format is in ../docs/frc1.md and the reader in frc1.h.  This is the
 * other half: what a host actually does with one.
 *
 *     ./frc1_load info    <file.frc>          what is in it
 *     ./frc1_load verify  <file.frc>          every check, nothing written
 *     ./frc1_load fields  <file.frc>          every field, one per line
 *     ./frc1_load stub    <file.frc> [N]      the eight bytes the hook becomes
 *     ./frc1_load plan    <file.frc> [N]      the install, step by step, dry
 *
 *     ./frc1_load probe   [vid:pid]           what chip is on the other end
 *     ./frc1_load peek    <vid:pid> <addr> [n]
 *     ./frc1_load install <file.frc> [N]      **writes to the chip**
 *     ./frc1_load restore <file.frc> [N]      put the vendor's 8 bytes back
 *
 * THE TRANSPORT is behind `struct frc1_io` and there are two
 * implementations: a dry run that prints what it would do, and chipio.h's
 * control transfers on vendor request 0xBC.  `plan` uses the first,
 * `install` the second.  The part of this file worth copying is the parsing
 * and the ORDER.
 *
 * Built without libusb it still does everything above the line -- that is
 * what ../tests/test_frc1.py compiles, with a C compiler and nothing else --
 * and the commands below it say so instead of pretending.
 *
 * The order in install() is not a preference.  Every step is there because
 * leaving it out breaks something, and one of them was learned by wedging a
 * device hard enough to need a physical replug -- see ../docs/frc1.md.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "frc1.h"
#include "chipio.h"

/* --- the transport -----------------------------------------------------
 *
 * Four operations are enough for the whole install.  A real one does each
 * with a vendor control transfer on request 0xBC; see
 * ../docs/frc1.md for the window and the 64-byte chunking.
 */
struct frc1_io {
    const char *name;
    int (*read_mem)(void *ctx, uint32_t addr, uint8_t *buf, uint32_t n);
    int (*write_mem)(void *ctx, uint32_t addr, const uint8_t *buf,
                     uint32_t n);
    /* Optional: the eight hook bytes with the order that keeps the chip out
     * of a half-written dispatcher.  `restoring` says which way.  NULL means
     * use write_mem. */
    int (*write_hook)(void *ctx, uint32_t addr, const uint8_t *buf,
                      uint32_t n, int restoring);
    int (*find_device)(void *ctx, uint32_t vid, uint32_t pid);
    void *ctx;
    int live;                   /* 1 = the writes land on a chip          */
};

/* The dry-run transport: says what would happen, writes nothing.  Reads
 * answer with the bytes the container expects, so a dry run gets all the way
 * to the end and shows the whole sequence. */
struct dry {
    const struct frc1 *c;
    const struct frc1_target *t;
    int steps;
};

static int dry_read(void *ctx, uint32_t addr, uint8_t *buf, uint32_t n)
{
    struct dry *d = ctx;
    const struct frc1 *c = d->c;
    uint32_t i;

    d->steps++;
    printf("  read  %#010x %6u  ", addr, n);
    memset(buf, 0, n);
    if (addr == d->t->state) {
        printf("state (must be 0)\n");
        return 0;                               /* zeros = installable */
    }
    if (addr == d->t->loader_base) {
        printf("the running loader, for its crc32\n");
        /* A dry run cannot know the loader's bytes, so it answers with
         * whatever makes the check pass and says so. */
        return -2;
    }
    if (addr == d->t->hook) {
        /* A dry run cannot know the vendor's bytes -- the container only
         * carries their fingerprint -- so it says so, like the loader. */
        printf("the hook, before touching it\n");
        return -2;
    }
    for (i = 0; i < c->h.segment_count; i++) {
        struct frc1_segment s;

        frc1_segment_at(c, i, &s);
        if (s.address == addr && s.bytes == n) {
            memcpy(buf, frc1_segment_data(c, &s), n);
            printf("read back '%s'\n", frc1_string(c, s.name));
            return 0;
        }
    }
    printf("\n");
    return 0;
}

static int dry_write(void *ctx, uint32_t addr, const uint8_t *buf, uint32_t n)
{
    struct dry *d = ctx;

    (void)buf;
    d->steps++;
    printf("  write %#010x %6u%s\n", addr, n,
           frc1_needs_window(addr) ? "  (needs the bank window)" : "");
    return 0;
}

static int dry_find(void *ctx, uint32_t vid, uint32_t pid)
{
    (void)ctx;
    printf("  look for %04x:%04x\n", vid, pid);
    return 0;
}

/* The live transport: the same four operations, on the chip.  Progress is
 * printed in the same shape as the dry run so the two can be compared line
 * by line after a real install. */
static int usb_read(void *ctx, uint32_t addr, uint8_t *buf, uint32_t n)
{
    struct chipio *io = ctx;

    printf("  read  %#010x %6u  ", addr, n);
    fflush(stdout);
    if (chipio_read(io, addr, buf, n) != 0) {
        printf("FAILED\n");
        fprintf(stderr, "  %s\n", io->err);
        return -1;
    }
    printf("ok\n");
    return 0;
}

static int usb_write(void *ctx, uint32_t addr, const uint8_t *buf, uint32_t n)
{
    struct chipio *io = ctx;

    printf("  write %#010x %6u%s  ", addr, n,
           frc1_needs_window(addr) ? "  (needs the bank window)" : "");
    fflush(stdout);
    if (chipio_write(io, addr, buf, n) != 0) {
        printf("FAILED\n");
        fprintf(stderr, "  %s\n", io->err);
        return -1;
    }
    printf("ok\n");
    return 0;
}

static int usb_write_hook(void *ctx, uint32_t addr, const uint8_t *buf,
                          uint32_t n, int restoring)
{
    struct chipio *io = ctx;

    printf("  hook  %#010x %6u  %s  ", addr, n,
           restoring ? "vendor bytes first" : "stub first");
    fflush(stdout);
    if (chipio_hook_write(io, addr, buf, n, restoring) != 0) {
        printf("FAILED\n");
        fprintf(stderr, "  %s\n", io->err);
        return -1;
    }
    printf("ok\n");
    return 0;
}

static int write_hook_bytes(struct frc1_io *io, uint32_t addr,
                            const uint8_t *buf, uint32_t n, int restoring)
{
    if (io->write_hook != NULL)
        return io->write_hook(io->ctx, addr, buf, n, restoring);
    return io->write_mem(io->ctx, addr, buf, n);
}

/* THE VENDOR'S EIGHT BYTES ARE NOT IN THE CONTAINER.  Only their crc32 is,
 * so this tree carries none of the loader's code.  They are read off the chip
 * before the hook goes in, and kept in "<file>.hook" beside the container so
 * that a `restore` in a LATER run has them.  Without that file, a replug
 * restores them by itself: the resident loader lives in RAM, so the vendor's
 * bytes come back from flash at every power-on.
 */
static int hook_side_path(const char *frc, char *out, size_t n)
{
    int k = snprintf(out, n, "%s.hook", frc);

    return (k > 0 && (size_t)k < n) ? 0 : -1;
}

static void hook_side_save(const char *frc, const uint8_t *b, uint32_t n)
{
    char path[1024];
    FILE *f;

    if (hook_side_path(frc, path, sizeof(path)) != 0)
        return;
    f = fopen(path, "wb");
    if (f == NULL) {
        printf("     (could not write %s; a replug also restores them)\n",
               path);
        return;
    }
    (void)fwrite(b, 1, n, f);
    (void)fclose(f);
    printf("     kept the vendor's %u bytes in %s\n", n, path);
}

static int hook_side_load(const char *frc, uint8_t *b, uint32_t n,
                          uint32_t want_crc)
{
    char path[1024];
    FILE *f;
    size_t got;

    if (hook_side_path(frc, path, sizeof(path)) != 0)
        return -1;
    f = fopen(path, "rb");
    if (f == NULL)
        return -1;
    got = fread(b, 1, n, f);
    (void)fclose(f);
    if (got != n)
        return -1;
    return frc1_crc32(0, b, n) == want_crc ? 0 : -1;
}

/* The device is opened before install() runs, so this only checks that the
 * one that is open is the one the container names.  Opening the target's own
 * ids is what makes a container for another machine fail here rather than
 * three steps later with a write in flight. */
static int usb_find(void *ctx, uint32_t vid, uint32_t pid)
{
    struct chipio *io = ctx;

    printf("  %04x:%04x is open\n", vid, pid);
    if (io->vid != vid || io->pid != pid) {
        fprintf(stderr, "opened %04x:%04x but the target wants %04x:%04x\n",
                io->vid, io->pid, vid, pid);
        return -1;
    }
    return 0;
}

/* 1 when the hook already jumps to this container's entry and every code
 * segment reads back as built: the same build is in, and installing again
 * would only rewrite the same bytes.  0 when anything differs, -1 when a
 * read failed.  Code segments only: .bss changes as the firmware runs. */
static int same_build_in(const struct frc1 *c, uint32_t which,
                         const struct frc1_target *t, struct frc1_io *io)
{
    uint8_t stub[8], buf[8];
    uint32_t i;

    if (t->entry >= 0x100000u)
        return 0;
    frc1_hook_stub(t->entry, stub);
    if (io->read_mem(io->ctx, t->hook, buf, sizeof(stub)) != 0)
        return -1;
    if (memcmp(buf, stub, sizeof(stub)) != 0)
        return 0;
    for (i = 0; i < c->h.segment_count; i++) {
        struct frc1_segment s;
        uint8_t *back;
        int same;

        frc1_segment_at(c, i, &s);
        if (!(s.target_mask & (1u << which)) || !(s.flags & FRC1_SEG_CODE))
            continue;
        back = malloc(s.bytes ? s.bytes : 1);
        if (back == NULL)
            return -1;
        if (io->read_mem(io->ctx, s.address, back, s.bytes) != 0) {
            free(back);
            return -1;
        }
        same = memcmp(back, frc1_segment_data(c, &s), s.bytes) == 0;
        free(back);
        if (!same)
            return 0;
    }
    return 1;
}

/* --- the install -------------------------------------------------------
 *
 * The steps are numbered as in ../docs/frc1.md.  Read that before changing
 * the order.
 */
static int install(const char *frc_path, const struct frc1 *c,
                   uint32_t which, struct frc1_io *io,
                   int force)
{
    struct frc1_target t;
    uint8_t buf[256];
    uint8_t vhook[8];
    int have_vhook = 0;
    uint32_t i;
    int rc;

    frc1_target_at(c, which, &t);
    printf("target %u: %s  %04x:%04x\n", which, frc1_string(c, t.name),
           t.usb_vendor, t.usb_product);

    if (io->find_device(io->ctx, t.usb_vendor, t.usb_product) != 0) {
        fprintf(stderr, "that device is not plugged in\n");
        return -1;
    }

    /* 1. state must read 0.  Anything else means a vendor container has
     *    booted and 0x000E9000 no longer holds the plaintext loader. */
    printf("1. state\n");
    if (io->read_mem(io->ctx, t.state, buf, 4) != 0)
        return -1;
    if (frc1_be32(buf) != 0) {
        fprintf(stderr, "state is %#010x, not 0: a container has booted "
                "-- power-cycle and do not let it\n", frc1_be32(buf));
        return -1;
    }

    /* 1b. the vendor's eight bytes.  The container has only their crc32,
     *     so they come off the chip -- or out of the sidecar if a hook of
     *     ours is already in place. */
    printf("1b. the vendor's eight bytes\n");
    if (io->live) {
        if (io->read_mem(io->ctx, t.hook, vhook, t.hook_bytes) != 0)
            return -1;
        if (frc1_crc32(0, vhook, t.hook_bytes) == t.hook_crc32) {
            have_vhook = 1;
            printf("     on the chip, fingerprint %#010x as built\n",
                   t.hook_crc32);
            hook_side_save(frc_path, vhook, t.hook_bytes);
        } else if (hook_side_load(frc_path, vhook, t.hook_bytes,
                                  t.hook_crc32) == 0) {
            have_vhook = 1;
            printf("     a hook is in place; took them from %s.hook\n",
                   frc_path);
        } else {
            fprintf(stderr, "the hook does not hold the vendor's bytes "
                    "(fingerprint %#010x) and there is no %s.hook to take "
                    "them from.  Replug: the loader is in RAM, so a "
                    "power-cycle brings the vendor's bytes back\n",
                    t.hook_crc32, frc_path);
            return -1;
        }
    } else {
        printf("     dry run: cannot read the chip, want %#010x\n",
               t.hook_crc32);
    }

    /* 2. the running loader must be the one this was built against.  Two
     *    selene loaders can differ by a single word, and the payload jumps
     *    to addresses inside it. */
    printf("2. the loader's crc32\n");
    if (t.loader_crc32 == 0) {
        /* On a dry run this is just a line of output.  On a chip it is the
         * only thing standing between step 3 and writing eight bytes on top
         * of a dispatcher we have never seen, so a live install stops here
         * unless the operator insists. */
        if (io->live && !force) {
            fprintf(stderr, "the container does not say which loader it was "
                    "built against, so step 3 would write the vendor bytes "
                    "on top of code this has not identified.  Refusing; "
                    "--force overrides\n");
            return -1;
        }
        printf("     0 in the container: not checked\n");
    } else {
        uint8_t *image = malloc(t.loader_bytes);
        uint8_t hook_bytes[8];
        int hook_ok = 0;
        uint32_t got;

        if (image == NULL)
            return -1;
        rc = io->read_mem(io->ctx, t.loader_base, image, t.loader_bytes);
        /* The hook lies INSIDE the loader image, so hashing RAM as-is gives
         * a different answer depending on what is installed right now --
         * an install over an existing hook would reject its own loader.
         * Put the vendor bytes back in the copy before hashing; the question
         * is which loader this is, not which hook is on it. */
        if (t.hook_bytes <= sizeof(hook_bytes))
            hook_ok = io->read_mem(io->ctx, t.hook, hook_bytes,
                                   t.hook_bytes) == 0;
        if (rc == 0 && hook_ok && have_vhook && t.hook >= t.loader_base &&
            t.hook + t.hook_bytes <= t.loader_base + t.loader_bytes)
            memcpy(image + (t.hook - t.loader_base), vhook, t.hook_bytes);
        got = frc1_crc32(0, image, t.loader_bytes);
        free(image);
        if (rc == -2) {
            printf("     dry run: cannot read the chip, want %#010x\n",
                   t.loader_crc32);
        } else if (rc != 0) {
            return -1;
        } else if (got != t.loader_crc32) {
            fprintf(stderr, "the loader is %#010x, this container was built "
                    "against %#010x -- wrong machine or wrong firmware "
                    "version\n", got, t.loader_crc32);
            return -1;
        } else {
            printf("     %#010x, as built\n", got);
        }
    }

    /* 2b. the same build already in: nothing to write.  --force installs
     *     anyway (a fresh .bss, say). */
    if (io->live && !force) {
        rc = same_build_in(c, which, &t, io);
        if (rc < 0)
            return -1;
        if (rc == 1) {
            printf("2b. this build is already in (hook and code read back as "
                   "built); nothing written\n    --force installs it "
                   "again\n");
            return 0;
        }
    }

    /* 3. put the vendor's eight bytes back FIRST.  If a hook is already in
     *    place, the code it points at is what is servicing this very
     *    transfer; overwriting it wedges EP0. */
    printf("3. the vendor dispatcher goes back first\n");
    rc = io->read_mem(io->ctx, t.hook, buf, t.hook_bytes);
    /* -2 is the dry run saying it cannot know these bytes (the container
     * carries only their fingerprint), which is not a failure. */
    if (rc != 0 && rc != -2)
        return -1;
    if (!have_vhook) {
        printf("     dry run: cannot put bytes it has not read\n");
    } else if (memcmp(buf, vhook, t.hook_bytes) == 0) {
        printf("     already the vendor's bytes, nothing to undo\n");
    } else {
        printf("     a hook is installed; restoring before writing anything\n");
        if (write_hook_bytes(io, t.hook, vhook, t.hook_bytes, 1) != 0)
            return -1;
    }

    /* 4. the segments, then read them back. */
    printf("4. segments\n");
    for (i = 0; i < c->h.segment_count; i++) {
        struct frc1_segment s;
        const uint8_t *data;

        frc1_segment_at(c, i, &s);
        if (!(s.target_mask & (1u << which)))
            continue;
        data = frc1_segment_data(c, &s);
        if (io->write_mem(io->ctx, s.address, data, s.bytes) != 0)
            return -1;
        if (s.flags & FRC1_SEG_VERIFY) {
            uint8_t *back = malloc(s.bytes ? s.bytes : 1);

            if (back == NULL)
                return -1;
            rc = io->read_mem(io->ctx, s.address, back, s.bytes);
            if (rc == 0 && memcmp(back, data, s.bytes) != 0) {
                fprintf(stderr, "segment '%s' read back different\n",
                        frc1_string(c, s.name));
                rc = -1;
            }
            free(back);
            if (rc != 0)
                return -1;
        }
    }

    /* 5. the hook last, so the entry is never pointed at half-written code.
     *    Reading it back goes through the new firmware's gate, so an answer
     *    at all means the gate works. */
    printf("5. the hook, last\n");
    {
        uint8_t stub[8];

        if (t.entry >= 0x100000u) {
            fprintf(stderr, "entry %#010x does not fit the 20-bit "
                    "immediate\n", t.entry);
            return -1;
        }
        frc1_hook_stub(t.entry, stub);
        if (write_hook_bytes(io, t.hook, stub, sizeof(stub), 0) != 0)
            return -1;
        rc = io->read_mem(io->ctx, t.hook, buf, sizeof(stub));
        if (rc != 0 && rc != -2)        /* -2 is the dry run (step 1b) */
            return -1;
        printf("     answered through the new gate\n");
    }
    printf("installed\n");
    return 0;
}

/* Undo step 5: the vendor's eight bytes go back, so the plain window and
 * the plain window work again.  The segments are left where they
 * are -- nothing runs them once the hook no longer points at them, and
 * rewriting that much RAM to no effect is the sort of thing that goes
 * wrong. */
static int restore(const char *frc_path, const struct frc1 *c,
                   uint32_t which, struct chipio *io)
{
    struct frc1_target t;
    uint8_t buf[8] = {0};
    uint8_t vhook[8];

    frc1_target_at(c, which, &t);
    printf("target %u: %s  %04x:%04x\n", which, frc1_string(c, t.name),
           t.usb_vendor, t.usb_product);
    if (usb_find(io, t.usb_vendor, t.usb_product) != 0)
        return -1;
    if (usb_read(io, t.hook, buf, t.hook_bytes) != 0)
        return -1;
    if (frc1_crc32(0, buf, t.hook_bytes) == t.hook_crc32) {
        printf("already the vendor's bytes; nothing to do\n");
        return 0;
    }
    /* The container does not carry them, so they come from the sidecar the
     * install wrote.  Without it, a replug is the restore. */
    if (hook_side_load(frc_path, vhook, t.hook_bytes, t.hook_crc32) != 0) {
        fprintf(stderr, "no %s.hook with the vendor's eight bytes (the "
                "container carries only their fingerprint %#010x).  Replug: "
                "the loader is in RAM, so a power-cycle restores them\n",
                frc_path, t.hook_crc32);
        return -1;
    }
    if (usb_write_hook(io, t.hook, vhook, t.hook_bytes, 1) != 0)
        return -1;
    if (usb_read(io, t.hook, buf, t.hook_bytes) != 0)
        return -1;
    if (memcmp(buf, vhook, t.hook_bytes) != 0) {
        fprintf(stderr, "the hook did not read back as the vendor's bytes\n");
        return -1;
    }
    printf("restored\n");
    return 0;
}

/* --- reporting -------------------------------------------------------- */

static const char *flag_names(uint32_t f, char *out, size_t n)
{
    static const struct { uint32_t bit; const char *name; } tab[] = {
        {FRC1_SEG_CODE, "code"}, {FRC1_SEG_VERIFY, "verify"},
        {FRC1_SEG_WINDOW, "window"}, {FRC1_SEG_ENTRY, "entry"},
    };
    size_t i, used = 0;

    out[0] = 0;
    for (i = 0; i < sizeof(tab) / sizeof(tab[0]); i++) {
        if (!(f & tab[i].bit))
            continue;
        used += (size_t)snprintf(out + used, n - used, "%s%s",
                                 used ? "|" : "", tab[i].name);
        if (used >= n)
            break;
    }
    return out;
}

static void info(const struct frc1 *c)
{
    char flags[64];
    uint32_t i;

    printf("%s  build %u  %u target(s), %u segment(s), %zu bytes\n",
           frc1_string(c, c->h.name), c->h.build_id, c->h.target_count,
           c->h.segment_count, c->len);
    for (i = 0; i < c->h.target_count; i++) {
        struct frc1_target t;

        frc1_target_at(c, i, &t);
        printf("\ntarget %u: %s  usb %04x:%04x\n", i, frc1_string(c, t.name),
               t.usb_vendor, t.usb_product);
        printf("  entry       %#09x   hook %#09x   its original bytes "
               "%#010x\n", t.entry, t.hook, t.hook_crc32);
        printf("  resume      plain %#09x  mask %#09x  complete %#09x\n",
               t.resume_plain, t.resume_mask, t.complete);
        printf("  staging     %#09x   state %#09x\n", t.staging, t.state);
        printf("  loader      %#09x + %#x  crc32 %#010x\n", t.loader_base,
               t.loader_bytes, t.loader_crc32);
    }
    for (i = 0; i < c->h.segment_count; i++) {
        struct frc1_segment s;

        frc1_segment_at(c, i, &s);
        printf("\nsegment %u: %-10s %#09x + %#6x  crc32 %#010x  [%s]\n", i,
               frc1_string(c, s.name), s.address, s.bytes, s.crc32,
               flag_names(s.flags, flags, sizeof(flags)));
    }
}

/* Every field, one per line.  ../tests/test_frc1.py compares this with what
 * the Python writer put in, so the two implementations cannot drift. */
static void fields(const struct frc1 *c)
{
    uint32_t i;

    printf("h.magic=%.4s\n", c->h.magic);
    printf("h.version=%u\nh.header_bytes=%u\nh.total_bytes=%u\n",
           c->h.version, c->h.header_bytes, c->h.total_bytes);
    printf("h.flags=%u\nh.crc32=%#010x\n", c->h.flags, c->h.crc32);
    printf("h.target_offset=%u\nh.target_count=%u\n", c->h.target_offset,
           c->h.target_count);
    printf("h.segment_offset=%u\nh.segment_count=%u\n", c->h.segment_offset,
           c->h.segment_count);
    printf("h.string_offset=%u\nh.string_bytes=%u\n", c->h.string_offset,
           c->h.string_bytes);
    printf("h.build_id=%u\nh.name=%s\n", c->h.build_id,
           frc1_string(c, c->h.name));
    for (i = 0; i < c->h.target_count; i++) {
        struct frc1_target t;

        frc1_target_at(c, i, &t);
        printf("t%u.name=%s\n", i, frc1_string(c, t.name));
        printf("t%u.usb_vendor=%#06x\nt%u.usb_product=%#06x\n", i,
               t.usb_vendor, i, t.usb_product);
        printf("t%u.hook=%#010x\nt%u.hook_bytes=%u\nt%u.hook_crc32=%#010x\n",
               i, t.hook, i, t.hook_bytes, i, t.hook_crc32);
        printf("t%u.entry=%#010x\n", i, t.entry);
        printf("t%u.resume_plain=%#010x\nt%u.resume_mask=%#010x\n", i,
               t.resume_plain, i, t.resume_mask);
        printf("t%u.complete=%#010x\nt%u.staging=%#010x\nt%u.state=%#010x\n",
               i, t.complete, i, t.staging, i, t.state);
        printf("t%u.loader_base=%#010x\nt%u.loader_bytes=%#010x\n", i,
               t.loader_base, i, t.loader_bytes);
        printf("t%u.loader_crc32=%#010x\n", i, t.loader_crc32);
    }
    for (i = 0; i < c->h.segment_count; i++) {
        struct frc1_segment s;

        frc1_segment_at(c, i, &s);
        printf("s%u.name=%s\n", i, frc1_string(c, s.name));
        printf("s%u.address=%#010x\ns%u.bytes=%u\n", i, s.address, i, s.bytes);
        printf("s%u.file_offset=%u\ns%u.crc32=%#010x\n", i, s.file_offset, i,
               s.crc32);
        printf("s%u.flags=%u\ns%u.target_mask=%#010x\n", i, s.flags, i,
               s.target_mask);
        printf("s%u.needs_window=%d\n", i, frc1_needs_window(s.address));
    }
}

/* --- the chip, with no container in hand ------------------------------ */

static const struct { const char *name; unsigned vid, pid; } KNOWN[] = {
    {"dt295", 0x06B8, 0x103B},      /* PIXELA PIX-DT295      MB8AC031    */
    {"m2tv",  0x04BB, 0x053A},      /* I-O DATA GV-M2TV      MB8AC018    */
};

/* "04bb:053a", or one of the short names above. */
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

static void dump_bytes(uint32_t addr, const uint8_t *b, uint32_t n)
{
    uint32_t k, j;

    for (k = 0; k < n; k += 16) {
        printf("  %08x ", addr + k);
        for (j = 0; j < 16; j++) {
            if (k + j < n)
                printf(" %02x", b[k + j]);
            else
                printf("   ");
        }
        printf("  |");
        for (j = 0; j < 16 && k + j < n; j++)
            putchar(b[k + j] >= 32 && b[k + j] < 127 ? b[k + j] : '.');
        printf("|\n");
    }
}

/* Read-only.  Says what is on the other end and whether a container could
 * go in right now: the magic names the generation, state says whether a
 * vendor container has already booted over the plaintext loader. */
static int probe(unsigned vid, unsigned pid)
{
    struct chipio io;
    uint8_t buf[32] = {0};
    char magic[9];
    uint32_t state;

    if (chipio_open(&io, vid, pid) != 0) {
        fprintf(stderr, "%s\n", io.err);
        return 1;
    }
    printf("%04x:%04x open on %s, request %#x\n", vid, pid, chipio_backend(),
           io.request);

    if (chipio_read(&io, CHIPIO_REG_CHIPID, buf, 8) != 0) {
        fprintf(stderr, "%s\n", io.err);
        chipio_close(&io);
        return 1;
    }
    chipio_magic_text(buf, magic);
    printf("  0x%08x  container magic  %-8s  %02x%02x%02x%02x%02x%02x%02x%02x\n",
           CHIPIO_REG_CHIPID, magic, buf[0], buf[1], buf[2], buf[3], buf[4],
           buf[5], buf[6], buf[7]);

    if (chipio_read(&io, CHIPIO_REG_STATE, buf, 4) != 0) {
        fprintf(stderr, "%s\n", io.err);
        chipio_close(&io);
        return 1;
    }
    state = frc1_be32(buf);
    printf("  0x%08x  state            0x%08x  %s\n", CHIPIO_REG_STATE, state,
           state == 0 ? "no container has booted: a container can go in"
                      : "a container is running: power-cycle before installing");

    /* The loader and its dispatcher.  Which addresses those are depends on
     * the generation: selene (MB8AC031) hooks 0x000EA6FC, dione (MB86H5x)
     * hooks 0x000E9F80.  Printed, not judged. */
    if (chipio_read(&io, 0x000E9000u, buf, 16) == 0) {
        printf("  0x000e9000  the plaintext loader\n");
        dump_bytes(0x000E9000u, buf, 16);
    }
    {
        uint32_t hook = io.dione ? 0x000E9F80u : 0x000EA6FCu;

        if (chipio_read(&io, hook, buf, 8) == 0) {
            printf("  0x%08x  the sub-opcode dispatcher (%s, hook)\n", hook,
                   io.dione ? "dione" : "selene");
            dump_bytes(hook, buf, 8);
        }
    }
    chipio_close(&io);
    return 0;
}

static int peek(unsigned vid, unsigned pid, uint32_t addr, uint32_t n)
{
    struct chipio io;
    uint8_t *buf;

    if (n == 0 || n > 4096u) {
        fprintf(stderr, "read 1..4096 bytes\n");
        return 2;
    }
    buf = calloc(n, 1);
    if (buf == NULL)
        return 1;
    if (chipio_open(&io, vid, pid) != 0) {
        fprintf(stderr, "%s\n", io.err);
        free(buf);
        return 1;
    }
    if (chipio_read(&io, addr, buf, n) != 0) {
        fprintf(stderr, "%s\n", io.err);
        chipio_close(&io);
        free(buf);
        return 1;
    }
    dump_bytes(addr, buf, n);
    chipio_close(&io);
    free(buf);
    return 0;
}

static uint8_t *slurp(const char *path, size_t *len)
{
    FILE *f = fopen(path, "rb");
    uint8_t *blob;
    long n;

    if (f == NULL)
        return NULL;
    if (fseek(f, 0, SEEK_END) != 0 || (n = ftell(f)) < 0) {
        fclose(f);
        return NULL;
    }
    rewind(f);
    blob = malloc((size_t)n ? (size_t)n : 1);
    if (blob == NULL || fread(blob, 1, (size_t)n, f) != (size_t)n) {
        free(blob);
        fclose(f);
        return NULL;
    }
    fclose(f);
    *len = (size_t)n;
    return blob;
}

static void usage(const char *me)
{
    fprintf(stderr,
            "usage: %s info|verify|fields|stub|plan <file.frc> [target]\n"
            "       %s install|restore <file.frc> [target] [--force]\n"
            "       %s probe [vid:pid|dt295|m2tv]\n"
            "       %s peek  <vid:pid|name> <addr> [bytes]\n",
            me, me, me, me);
}

int main(int argc, char **argv)
{
    char err[256];
    struct frc1 c;
    uint8_t *blob;
    size_t len;
    uint32_t which = 0;
    unsigned vid = 0, pid = 0;
    int force = 0;
    int rc = 0;
    int i;

    if (argc < 2) {
        usage(argv[0]);
        return 2;
    }

    for (i = 2; i < argc; i++)
        if (strcmp(argv[i], "--force") == 0)
            force = 1;

    /* --- the commands that need no container ------------------------- */
    if (strcmp(argv[1], "probe") == 0) {
        vid = 0x06B8;
        pid = 0x103B;                   /* dt295 */
        if (argc > 2 && parse_dev(argv[2], &vid, &pid) != 0) {
            fprintf(stderr, "%s: not a device\n", argv[2]);
            return 2;
        }
        return probe(vid, pid);
    }
    if (strcmp(argv[1], "peek") == 0) {
        uint32_t addr, n = 32u;

        if (argc < 4) {
            usage(argv[0]);
            return 2;
        }
        if (parse_dev(argv[2], &vid, &pid) != 0) {
            fprintf(stderr, "%s: not a device\n", argv[2]);
            return 2;
        }
        addr = (uint32_t)strtoul(argv[3], NULL, 0);
        if (argc > 4)
            n = (uint32_t)strtoul(argv[4], NULL, 0);
        return peek(vid, pid, addr, n);
    }

    /* --- everything else reads a container first --------------------- */
    if (argc < 3) {
        usage(argv[0]);
        return 2;
    }
    blob = slurp(argv[2], &len);
    if (blob == NULL) {
        fprintf(stderr, "%s: cannot read\n", argv[2]);
        return 2;
    }
    if (frc1_open(&c, blob, len, err, sizeof(err)) != 0) {
        fprintf(stderr, "%s: %s\n", argv[2], err);
        free(blob);
        return 1;
    }
    if (argc > 3 && argv[3][0] != '-')
        which = (uint32_t)strtoul(argv[3], NULL, 0);
    if (which >= c.h.target_count &&
        (strcmp(argv[1], "info") != 0 && strcmp(argv[1], "fields") != 0 &&
         strcmp(argv[1], "verify") != 0)) {
        fprintf(stderr, "there is no target %u\n", which);
        free(blob);
        return 2;
    }

    if (strcmp(argv[1], "info") == 0) {
        info(&c);
    } else if (strcmp(argv[1], "fields") == 0) {
        fields(&c);
    } else if (strcmp(argv[1], "stub") == 0) {
        struct frc1_target t;
        uint8_t stub[8];
        int k;

        frc1_target_at(&c, which, &t);
        if (frc1_hook_stub(t.entry, stub) != 0) {
            fprintf(stderr, "entry %#010x does not fit the 20-bit "
                    "immediate\n", t.entry);
            free(blob);
            return 1;
        }
        for (k = 0; k < 8; k++)
            printf("%02x", stub[k]);
        printf("\n");
    } else if (strcmp(argv[1], "verify") == 0) {
        printf("%s: valid FRC1, %zu bytes, %u target(s), %u segment(s)\n",
               argv[2], len, c.h.target_count, c.h.segment_count);
    } else if (strcmp(argv[1], "plan") == 0) {
        struct frc1_target t;
        struct dry d;
        struct frc1_io io;

        frc1_target_at(&c, which, &t);
        d.c = &c;
        d.t = &t;
        d.steps = 0;
        io.name = "dry run";
        io.read_mem = dry_read;
        io.write_mem = dry_write;
        io.write_hook = NULL;
        io.find_device = dry_find;
        io.ctx = &d;
        io.live = 0;
        printf("dry run -- nothing is written\n\n");
        rc = install(argv[2], &c, which, &io, force) == 0 ? 0 : 1;
        printf("\n%d transfer group(s)\n", d.steps);
    } else if (strcmp(argv[1], "install") == 0) {
        struct frc1_target t;
        struct chipio chip;
        struct frc1_io io;

        frc1_target_at(&c, which, &t);
        if (chipio_open(&chip, t.usb_vendor, t.usb_product) != 0) {
            fprintf(stderr, "%s\n", chip.err);
            free(blob);
            return 1;
        }
        io.name = "usb";
        io.read_mem = usb_read;
        io.write_mem = usb_write;
        io.write_hook = usb_write_hook;
        io.find_device = usb_find;
        io.ctx = &chip;
        io.live = 1;
        printf("installing on %s -- this writes to the chip\n\n",
               chipio_backend());
        rc = install(argv[2], &c, which, &io, force) == 0 ? 0 : 1;
        chipio_close(&chip);
    } else if (strcmp(argv[1], "restore") == 0) {
        struct frc1_target t;
        struct chipio chip;

        frc1_target_at(&c, which, &t);
        if (chipio_open(&chip, t.usb_vendor, t.usb_product) != 0) {
            fprintf(stderr, "%s\n", chip.err);
            free(blob);
            return 1;
        }
        rc = restore(argv[2], &c, which, &chip) == 0 ? 0 : 1;
        chipio_close(&chip);
    } else {
        fprintf(stderr, "unknown command %s\n", argv[1]);
        rc = 2;
    }
    free(blob);
    return rc;
}
