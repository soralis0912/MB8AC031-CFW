"""`FRC1` -- the firmware container for the MB8AC031 / MB86M0x plaintext path.

The device's own `MB8AC031` container path is signed and we do not have
the key, so nothing we build boots that way.  The plaintext path needs no container: it
writes bytes into RAM through the 0xBC window and redirects 8 bytes of the
resident loader.  But "write these bytes to these addresses, on this chip,
and only if the loader is the one I built against" is what a container is
for, so FRC1 is one.

Deliberately NOT signed, encrypted, compressed or position independent.
Segments carry absolute load addresses because the host writes them straight
there, and the linker resolves cross-segment symbols, so there is no
relocation list.

Everything is big-endian, matching the chip.

    +0x00  4  magic "FRC1"
    +0x04  4  format version (1)
    +0x08  4  header bytes (64)
    +0x0c  4  total size (= file size)
    +0x10  4  flags (0)
    +0x14  4  crc32 of the whole file with THIS field zeroed
    +0x18  4  target table offset
    +0x1c  4  target count
    +0x20  4  segment table offset
    +0x24  4  segment count
    +0x28  4  string table offset
    +0x2c  4  string table bytes
    +0x30  4  build id (unix seconds)
    +0x34  4  name (offset into the string table)
    +0x38  8  reserved, zero

    target[i], 64 bytes -- one supported chip, and every address on it that
    a payload has to know.  These are facts about the DEVICE's resident
    loader, measured per device, not about our code, which is why they live here and not in the sources.
    +0x00  4  usb vendor id
    +0x04  4  usb product id
    +0x08  4  hook address            (0x000EA6FC on selene)
    +0x0c  4  hook bytes              (8)
    +0x10  4  crc32 of the hook's original 8 bytes  (refuse anything else)
    +0x14  4  reserved (0)
    +0x18  4  entry address           (where the hook jumps -- one of our segments)
    +0x1c  4  resume_plain            (0x000EA75A: where sub-opcode 0 resumes)
    +0x20  4  resume_mask             (0x000EA704: where sub-opcode 1 resumes)
    +0x24  4  complete                (0x000EA8C8: finish the control transfer)
    +0x28  4  staging                 (0x000C4620: the 64-byte OUT buffer)
    +0x2c  4  state                   (0x00082008: must read 0 to install)
    +0x30  4  loader base             (0x000E9000)
    +0x34  4  loader bytes            (0x3910)
    +0x38  4  loader crc32            (0 = do not check)
    +0x3c  4  name (string table offset)

    segment[j], 32 bytes
    +0x00  4  chip address
    +0x04  4  byte count
    +0x08  4  file offset
    +0x0c  4  crc32 of those bytes
    +0x10  4  flags (see SEG_*)
    +0x14  4  target mask, bit i = target[i]
    +0x18  4  name (string table offset)
    +0x1c  4  reserved, zero

The string table is NUL-terminated UTF-8; offset 0 is always the empty string.

    python frcontainer.py info <file.frc>
    python frcontainer.py verify <file.frc>
"""
import argparse
import struct
import sys
import zlib

MAGIC = b"FRC1"
VERSION = 1
HEADER_BYTES = 64
TARGET_BYTES = 64
SEGMENT_BYTES = 32
M32 = 0xFFFFFFFF

SEG_CODE = 1        # instructions, not data (only affects reporting)
SEG_VERIFY = 2      # read back and compare after writing
SEG_WINDOW = 4      # needs the bank window: >= 0x1000000
SEG_ENTRY = 8       # holds a target's entry point

SEG_FLAG_NAMES = [(SEG_CODE, "code"), (SEG_VERIFY, "verify"),
                  (SEG_WINDOW, "window"), (SEG_ENTRY, "entry")]

HEADER_FMT = ">4s13I"
TARGET_FMT = ">2I2I2I10I"
SEGMENT_FMT = ">8I"
TARGET_KEYS = ("usb_vendor", "usb_product", "hook", "hook_bytes",
               "hook_crc32", "_reserved", "entry", "resume_plain",
               "resume_mask", "complete", "staging", "state", "loader_base",
               "loader_bytes", "loader_crc32")


class FormatError(Exception):
    pass


class Strings:
    """Builder for the string table; offset 0 is always the empty string."""

    def __init__(self):
        self.blob = bytearray(b"\0")
        self.seen = {"": 0}

    def add(self, text):
        text = text or ""
        if text not in self.seen:
            self.seen[text] = len(self.blob)
            self.blob += text.encode("utf-8") + b"\0"
        return self.seen[text]


def read_string(blob, off):
    if off >= len(blob):
        return ""
    end = blob.find(b"\0", off)
    return blob[off:end if end >= 0 else len(blob)].decode("utf-8", "replace")


def crc32(b):
    return zlib.crc32(bytes(b)) & M32


def needs_window(address):
    """True when the plain 24-bit 0xBC window cannot reach this address."""
    return address >= 0x1000000


def build(name, targets, segments, build_id=0):
    """Pack a container.

    `targets` is a list of dicts keyed by the names in the layout above
    (pass `name` as text, not an offset).  `segments` is a list of dicts with
    `address`, `data`, `flags`, `targets` (a mask) and `name`.
    """
    if not targets:
        raise FormatError("a container needs at least one target")
    if len(targets) > 32:
        raise FormatError("the target mask is 32 bits, so at most 32 targets")
    strings = Strings()
    name_off = strings.add(name)
    target_names = [strings.add(t.get("name", "")) for t in targets]
    segment_names = [strings.add(s.get("name", "")) for s in segments]

    target_table = HEADER_BYTES
    segment_table = target_table + len(targets) * TARGET_BYTES
    string_table = segment_table + len(segments) * SEGMENT_BYTES
    payload = string_table + len(strings.blob)
    payload += -payload % 4

    blobs, offsets = bytearray(), []
    for s in segments:
        offsets.append(payload + len(blobs))
        blobs += s["data"]
        blobs += b"\0" * (-len(s["data"]) % 4)
    total = payload + len(blobs)

    out = bytearray(total)
    struct.pack_into(HEADER_FMT, out, 0, MAGIC, VERSION, HEADER_BYTES, total,
                     0, 0, target_table, len(targets), segment_table,
                     len(segments), string_table, len(strings.blob), build_id,
                     name_off)
    for i, (t, noff) in enumerate(zip(targets, target_names)):
        if not t.get("hook_crc32"):
            raise FormatError("hook_crc32 is the fingerprint of the hook's "
                              "eight original bytes and cannot be 0")
        struct.pack_into(TARGET_FMT, out, target_table + i * TARGET_BYTES,
                         t["usb_vendor"], t["usb_product"], t["hook"],
                         t.get("hook_bytes", 8), t["hook_crc32"], 0,
                         t["entry"],
                         t["resume_plain"], t["resume_mask"], t["complete"],
                         t["staging"], t["state"], t["loader_base"],
                         t["loader_bytes"], t.get("loader_crc32", 0), noff)
    for j, (s, off, noff) in enumerate(zip(segments, offsets, segment_names)):
        struct.pack_into(SEGMENT_FMT, out, segment_table + j * SEGMENT_BYTES,
                         s["address"], len(s["data"]), off, crc32(s["data"]),
                         s.get("flags", 0),
                         s.get("targets", (1 << len(targets)) - 1), noff, 0)
    out[string_table:string_table + len(strings.blob)] = strings.blob
    out[payload:payload + len(blobs)] = blobs
    struct.pack_into(">I", out, 0x14, crc32(out))
    return bytes(out)


def parse(blob):
    """Unpack and fully validate a container.  Raises FormatError, never lies."""
    if len(blob) < HEADER_BYTES:
        raise FormatError(f"{len(blob)} bytes is shorter than a header")
    (magic, version, header, total, flags, crc, target_table, target_count,
     segment_table, segment_count, string_table, string_bytes, build_id,
     name_off) = struct.unpack_from(HEADER_FMT, blob, 0)
    if magic != MAGIC:
        raise FormatError(f"magic {magic!r}, not {MAGIC!r}")
    if version != VERSION:
        raise FormatError(f"format version {version}, this tool speaks {VERSION}")
    if header != HEADER_BYTES:
        raise FormatError(f"header says {header} bytes, expected {HEADER_BYTES}")
    if total != len(blob):
        raise FormatError(f"header total {total:#x} != file size {len(blob):#x}")
    bare = bytearray(blob)
    struct.pack_into(">I", bare, 0x14, 0)
    actual = crc32(bare)
    if actual != crc:
        raise FormatError(f"crc32 {actual:#010x} != stored {crc:#010x}")

    def span(off, length, what):
        if off > len(blob) or length > len(blob) - off:
            raise FormatError(f"{what} at {off:#x}+{length:#x} runs past the file")

    span(target_table, target_count * TARGET_BYTES, "target table")
    span(segment_table, segment_count * SEGMENT_BYTES, "segment table")
    span(string_table, string_bytes, "string table")
    if not target_count:
        raise FormatError("no targets")
    if target_count > 32:
        raise FormatError(f"{target_count} targets; the mask is 32 bits")
    strings = blob[string_table:string_table + string_bytes]

    targets = []
    for i in range(target_count):
        f = struct.unpack_from(TARGET_FMT, blob, target_table + i * TARGET_BYTES)
        t = dict(zip(TARGET_KEYS, f))
        t["name"] = read_string(strings, f[15])
        if t["hook_bytes"] != 8:
            raise FormatError(f"target {i}: hook is {t['hook_bytes']} bytes, "
                              "expected 8")
        targets.append(t)

    segments = []
    for j in range(segment_count):
        (address, count, off, scrc, sflags, mask, noff, _) = struct.unpack_from(
            SEGMENT_FMT, blob, segment_table + j * SEGMENT_BYTES)
        span(off, count, f"segment {j}")
        data = blob[off:off + count]
        if crc32(data) != scrc:
            raise FormatError(f"segment {j} crc32 mismatch")
        if not count:
            raise FormatError(f"segment {j} is empty")
        if mask >> target_count:
            raise FormatError(f"segment {j} names target bits that do not exist")
        if bool(sflags & SEG_WINDOW) != needs_window(address):
            raise FormatError(f"segment {j} at {address:#x}: the window flag "
                              "does not match the address")
        segments.append({"address": address, "data": data, "file_offset": off,
                         "crc32": scrc, "flags": sflags, "targets": mask,
                         "name": read_string(strings, noff)})

    for i, t in enumerate(targets):
        if not any(s["targets"] & (1 << i)
                   and s["address"] <= t["entry"] < s["address"] + len(s["data"])
                   for s in segments):
            raise FormatError(f"target {i} ({t['name']}): entry {t['entry']:#x}"
                              " is not inside any segment that applies to it")
        if t["entry"] >= 0x100000:
            raise FormatError(f"target {i}: entry {t['entry']:#x} cannot be "
                              "reached by the 8-byte ldi:20 hook stub")
    check_overlaps(segments)
    return {"flags": flags, "build_id": build_id,
            "name": read_string(strings, name_off),
            "targets": targets, "segments": segments}


def check_overlaps(segments):
    """Raise FormatError if two segments that can apply to the same chip overlap."""
    for i, a in enumerate(segments):
        for b in segments[:i]:
            if not (a["targets"] & b["targets"]):
                continue
            if (a["address"] < b["address"] + len(b["data"])
                    and b["address"] < a["address"] + len(a["data"])):
                raise FormatError(
                    f"segments {b['name']!r} and {a['name']!r} overlap at "
                    f"{max(a['address'], b['address']):#x} on a shared target")


def describe(c):
    out = [f"{c['name'] or '(unnamed)'}  build {c['build_id']}  "
           f"{len(c['targets'])} target(s), {len(c['segments'])} segment(s)"]
    for i, t in enumerate(c["targets"]):
        out.append(f"\ntarget {i}: {t['name']}  usb "
                   f"{t['usb_vendor']:04x}:{t['usb_product']:04x}")
        out.append(f"  entry       {t['entry']:#09x}   "
                   f"hook {t['hook']:#09x}  its original bytes "
                   f"{t['hook_crc32']:#010x}")
        out.append(f"  resume      plain {t['resume_plain']:#09x}  "
                   f"mask {t['resume_mask']:#09x}  complete {t['complete']:#09x}")
        out.append(f"  staging     {t['staging']:#09x}   state {t['state']:#09x}")
        out.append(f"  loader      {t['loader_base']:#09x} + {t['loader_bytes']:#x}"
                   "  crc32 " + (f"{t['loader_crc32']:#010x}"
                                 if t["loader_crc32"] else "not checked"))
    out.append("")
    for j, s in enumerate(c["segments"]):
        names = ", ".join(t["name"] for i, t in enumerate(c["targets"])
                          if s["targets"] & (1 << i))
        flags = "|".join(n for b, n in SEG_FLAG_NAMES if s["flags"] & b) or "-"
        out.append(f"segment {j}: {s['name']:<10s} {s['address']:#09x} + "
                   f"{len(s['data']):#6x}  crc32 {s['crc32']:#010x}  "
                   f"[{flags}]  {names}")
        out.append(f"             first bytes {s['data'][:16].hex(' ')}")
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["info", "verify"])
    ap.add_argument("file")
    a = ap.parse_args()
    with open(a.file, "rb") as f:
        blob = f.read()
    try:
        c = parse(blob)
    except FormatError as e:
        print(f"{a.file}: NOT a valid FRC1 container: {e}", file=sys.stderr)
        return 1
    if a.cmd == "verify":
        print(f"{a.file}: valid FRC1, {len(blob)} bytes, "
              f"{len(c['targets'])} target(s), {len(c['segments'])} segment(s)")
        return 0
    print(describe(c))
    return 0


if __name__ == "__main__":
    sys.exit(main())
