/* frc1.h -- reading an `FRC1` container.  Header-only, no dependencies.
 *
 * The format is specified in ../docs/frc1.md; this is a reference reader for
 * it, meant to be copied into whatever host program needs to install one.
 * ../tests/test_frc1.py runs it against the Python writer field by field, so
 * if the two ever disagree, that test fails rather than a chip getting the
 * wrong bytes.
 *
 * THE STRUCTS MIRROR THE FILE.  Every field is at the offset the spec gives
 * and the _Static_asserts check that no padding crept in.  But the file is
 * big-endian, so the raw bytes are NOT these structs on a little-endian
 * host: frc1_open() decodes field by field, and after it the structs hold
 * host-order values.  Nothing here casts a pointer into the blob to a
 * struct, and neither should you -- the file is also not guaranteed to be
 * aligned.
 *
 * Needs C11 for _Static_assert.  Everything else is C99.
 */
#ifndef FRC1_H
#define FRC1_H

#include <stddef.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define FRC1_MAGIC          "FRC1"
#define FRC1_VERSION        1u
#define FRC1_HEADER_BYTES   64u
#define FRC1_TARGET_BYTES   64u
#define FRC1_SEGMENT_BYTES  32u
#define FRC1_MAX_TARGETS    32u     /* target_mask is 32 bits */

#define FRC1_SEG_CODE       1u
#define FRC1_SEG_VERIFY     2u
#define FRC1_SEG_WINDOW     4u
#define FRC1_SEG_ENTRY      8u

struct frc1_header {
    char     magic[4];          /* +0x00 */
    uint32_t version;           /* +0x04 */
    uint32_t header_bytes;      /* +0x08 */
    uint32_t total_bytes;       /* +0x0c */
    uint32_t flags;             /* +0x10 */
    uint32_t crc32;             /* +0x14  of the whole file, this field 0 */
    uint32_t target_offset;     /* +0x18 */
    uint32_t target_count;      /* +0x1c */
    uint32_t segment_offset;    /* +0x20 */
    uint32_t segment_count;     /* +0x24 */
    uint32_t string_offset;     /* +0x28 */
    uint32_t string_bytes;      /* +0x2c */
    uint32_t build_id;          /* +0x30 */
    uint32_t name;              /* +0x34 */
    uint32_t reserved[2];       /* +0x38 */
};
_Static_assert(sizeof(struct frc1_header) == FRC1_HEADER_BYTES, "frc1_header");

struct frc1_target {
    uint32_t usb_vendor;        /* +0x00 */
    uint32_t usb_product;       /* +0x04 */
    uint32_t hook;              /* +0x08 */
    uint32_t hook_bytes;        /* +0x0c */
    uint32_t hook_crc32;        /* +0x10  crc32 of the original 8 bytes */
    uint32_t reserved0;         /* +0x14 */
    uint32_t entry;             /* +0x18 */
    uint32_t resume_plain;      /* +0x1c */
    uint32_t resume_mask;       /* +0x20 */
    uint32_t complete;          /* +0x24 */
    uint32_t staging;           /* +0x28 */
    uint32_t state;             /* +0x2c  must read 0 to install */
    uint32_t loader_base;       /* +0x30 */
    uint32_t loader_bytes;      /* +0x34 */
    uint32_t loader_crc32;      /* +0x38  0 = do not check */
    uint32_t name;              /* +0x3c */
};
_Static_assert(sizeof(struct frc1_target) == FRC1_TARGET_BYTES, "frc1_target");

struct frc1_segment {
    uint32_t address;           /* +0x00 */
    uint32_t bytes;             /* +0x04 */
    uint32_t file_offset;       /* +0x08 */
    uint32_t crc32;             /* +0x0c */
    uint32_t flags;             /* +0x10 */
    uint32_t target_mask;       /* +0x14 */
    uint32_t name;              /* +0x18 */
    uint32_t reserved;          /* +0x1c */
};
_Static_assert(sizeof(struct frc1_segment) == FRC1_SEGMENT_BYTES,
               "frc1_segment");

struct frc1 {
    const uint8_t      *blob;
    size_t              len;
    struct frc1_header  h;
};

/* --- bytes ------------------------------------------------------------ */

static uint32_t frc1_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

/* crc32 as zlib computes it, so the two implementations agree.  Bitwise so
 * there is no table to get wrong; a real loader should use zlib's. */
static uint32_t frc1_crc32(uint32_t crc, const uint8_t *p, size_t n)
{
    size_t i;
    int k;

    crc = ~crc;
    for (i = 0; i < n; i++) {
        crc ^= p[i];
        for (k = 0; k < 8; k++)
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1)));
    }
    return ~crc;
}

/* The stored crc32 covers the whole file with its own field zeroed, so it
 * is computed in three pieces rather than over a modified copy. */
static uint32_t frc1_file_crc32(const uint8_t *blob, size_t len)
{
    static const uint8_t zero[4] = {0, 0, 0, 0};
    uint32_t crc;

    if (len < 0x18)
        return 0;
    crc = frc1_crc32(0, blob, 0x14);
    crc = frc1_crc32(crc, zero, 4);
    return frc1_crc32(crc, blob + 0x18, len - 0x18);
}

/* --- reading ---------------------------------------------------------- */

static const char *frc1_string(const struct frc1 *c, uint32_t off)
{
    const uint8_t *s;
    size_t room;

    if (off >= c->h.string_bytes)
        return "";
    s = c->blob + c->h.string_offset + off;
    room = c->h.string_bytes - off;
    if (memchr(s, 0, room) == NULL)
        return "";              /* not terminated inside the table */
    return (const char *)s;
}

static void frc1_target_at(const struct frc1 *c, uint32_t i,
                           struct frc1_target *t)
{
    const uint8_t *p = c->blob + c->h.target_offset + i * FRC1_TARGET_BYTES;

    t->usb_vendor = frc1_be32(p + 0x00);
    t->usb_product = frc1_be32(p + 0x04);
    t->hook = frc1_be32(p + 0x08);
    t->hook_bytes = frc1_be32(p + 0x0c);
    t->hook_crc32 = frc1_be32(p + 0x10);
    t->reserved0 = frc1_be32(p + 0x14);
    t->entry = frc1_be32(p + 0x18);
    t->resume_plain = frc1_be32(p + 0x1c);
    t->resume_mask = frc1_be32(p + 0x20);
    t->complete = frc1_be32(p + 0x24);
    t->staging = frc1_be32(p + 0x28);
    t->state = frc1_be32(p + 0x2c);
    t->loader_base = frc1_be32(p + 0x30);
    t->loader_bytes = frc1_be32(p + 0x34);
    t->loader_crc32 = frc1_be32(p + 0x38);
    t->name = frc1_be32(p + 0x3c);
}

static void frc1_segment_at(const struct frc1 *c, uint32_t j,
                            struct frc1_segment *s)
{
    const uint8_t *p = c->blob + c->h.segment_offset + j * FRC1_SEGMENT_BYTES;

    s->address = frc1_be32(p + 0x00);
    s->bytes = frc1_be32(p + 0x04);
    s->file_offset = frc1_be32(p + 0x08);
    s->crc32 = frc1_be32(p + 0x0c);
    s->flags = frc1_be32(p + 0x10);
    s->target_mask = frc1_be32(p + 0x14);
    s->name = frc1_be32(p + 0x18);
    s->reserved = frc1_be32(p + 0x1c);
}

static const uint8_t *frc1_segment_data(const struct frc1 *c,
                                        const struct frc1_segment *s)
{
    return c->blob + s->file_offset;
}

/* True when the plain 24-bit window cannot reach this address. */
static int frc1_needs_window(uint32_t address)
{
    return address >= 0x1000000u || (address >> 28) == 0x8u;
}

/* The eight bytes the hook becomes: `ldi:20 #entry, r12 ; jmp @r12 ; nop`.
 *
 * ldi:20 puts the immediate's top nibble in the HIGH nibble of byte 1 and
 * the register in the low one, then the low 16 bits follow -- which is why
 * `entry` has to be below 0x100000.  The nop only fills the eight bytes the
 * hook occupies; the jump leaves before reaching it.
 *
 * Checked against fr30-elf-as by ../tests/test_frc1.py, because these eight
 * bytes go on top of the running loader's code and getting them wrong needs a
 * physical replug to recover.
 */
static int frc1_hook_stub(uint32_t entry, uint8_t out[8])
{
    if (entry >= 0x100000u)
        return -1;
    out[0] = 0x9b;
    out[1] = (uint8_t)((((entry >> 16) & 0x0fu) << 4) | 12u);   /* r12 */
    out[2] = (uint8_t)(entry >> 8);
    out[3] = (uint8_t)entry;
    out[4] = 0x97;              /* jmp @r12 */
    out[5] = 0x0c;
    out[6] = 0x9f;              /* nop      */
    out[7] = 0xa0;
    return 0;
}

/* Every check the spec lists, in order.  Returns 0 and fills `c` on success,
 * or -1 with a reason in `err`.  Nothing is written anywhere before this
 * has passed -- a container that fails here must not reach a chip. */
static int frc1_open(struct frc1 *c, const uint8_t *blob, size_t len,
                     char *err, size_t errlen)
{
    struct frc1_header *h = &c->h;
    uint32_t i, end;
    int bad = 0;

#define FRC1_FAIL(...) do { snprintf(err, errlen, __VA_ARGS__); return -1; \
                          } while (0)

    memset(c, 0, sizeof(*c));
    if (len < FRC1_HEADER_BYTES)
        FRC1_FAIL("%zu bytes is shorter than the header", len);
    c->blob = blob;
    c->len = len;

    memcpy(h->magic, blob, 4);
    h->version = frc1_be32(blob + 0x04);
    h->header_bytes = frc1_be32(blob + 0x08);
    h->total_bytes = frc1_be32(blob + 0x0c);
    h->flags = frc1_be32(blob + 0x10);
    h->crc32 = frc1_be32(blob + 0x14);
    h->target_offset = frc1_be32(blob + 0x18);
    h->target_count = frc1_be32(blob + 0x1c);
    h->segment_offset = frc1_be32(blob + 0x20);
    h->segment_count = frc1_be32(blob + 0x24);
    h->string_offset = frc1_be32(blob + 0x28);
    h->string_bytes = frc1_be32(blob + 0x2c);
    h->build_id = frc1_be32(blob + 0x30);
    h->name = frc1_be32(blob + 0x34);
    h->reserved[0] = frc1_be32(blob + 0x38);
    h->reserved[1] = frc1_be32(blob + 0x3c);

    if (memcmp(h->magic, FRC1_MAGIC, 4) != 0)
        FRC1_FAIL("not an FRC1 container");
    if (h->version != FRC1_VERSION)
        FRC1_FAIL("format version %u, expected %u", h->version, FRC1_VERSION);
    if (h->header_bytes != FRC1_HEADER_BYTES)
        FRC1_FAIL("header is %u bytes, expected %u", h->header_bytes,
                  FRC1_HEADER_BYTES);
    if (h->total_bytes != len)
        FRC1_FAIL("header says %u bytes, the file is %zu", h->total_bytes, len);
    if (frc1_file_crc32(blob, len) != h->crc32)
        FRC1_FAIL("crc32 %#010x, computed %#010x", h->crc32,
                  frc1_file_crc32(blob, len));
    if (h->target_count < 1 || h->target_count > FRC1_MAX_TARGETS)
        FRC1_FAIL("%u targets, the mask holds at most %u", h->target_count,
                  FRC1_MAX_TARGETS);

    if (h->target_offset + h->target_count * FRC1_TARGET_BYTES > len)
        FRC1_FAIL("the target table runs past the end");
    if (h->segment_offset + h->segment_count * FRC1_SEGMENT_BYTES > len)
        FRC1_FAIL("the segment table runs past the end");
    if (h->string_offset + h->string_bytes > len)
        FRC1_FAIL("the string table runs past the end");

    for (i = 0; i < h->target_count; i++) {
        struct frc1_target t;

        frc1_target_at(c, i, &t);
        if (t.hook_bytes != 8)
            FRC1_FAIL("target %u: hook is %u bytes, expected 8", i,
                      t.hook_bytes);
    }

    for (i = 0; i < h->segment_count; i++) {
        struct frc1_segment s;

        frc1_segment_at(c, i, &s);
        end = s.file_offset + s.bytes;
        if (end > len || end < s.file_offset)
            FRC1_FAIL("segment %u runs past the end", i);
        if (frc1_crc32(0, frc1_segment_data(c, &s), s.bytes) != s.crc32)
            FRC1_FAIL("segment %u: crc32 does not match its bytes", i);
    }

    /* Two segments wanted by the same target must not overlap: the writes
     * would race and the read-back of whichever went first would fail. */
    for (i = 0; i < h->segment_count; i++) {
        struct frc1_segment a;
        uint32_t j;

        frc1_segment_at(c, i, &a);
        for (j = i + 1; j < h->segment_count; j++) {
            struct frc1_segment b;

            frc1_segment_at(c, j, &b);
            if (!(a.target_mask & b.target_mask))
                continue;
            if (a.address < b.address + b.bytes &&
                b.address < a.address + a.bytes)
                bad = 1;
        }
    }
    if (bad)
        FRC1_FAIL("two segments for the same target overlap");

#undef FRC1_FAIL
    return 0;
}

#endif
