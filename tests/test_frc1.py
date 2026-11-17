#!/usr/bin/env python3
"""Checks for the FRC1 container and the structs published in docs.

The spec is written twice (the Python writer, and the C reader published
in docs/frc1.md and implemented in sample/frc1.h), so any mismatch must
fail here. If a mismatch first shows up on the chip, you only notice it
once you need to physically unplug the device.

What this checks

  * The built container passes the frcontainer.py checks
  * Every field the C reader reads matches the value the Python writer stored
  * The 8 bytes written to the hook match fr30-elf-as output. A mistake here
    would overwrite running loader code with something else
  * The C reader rejects broken containers (magic, crc32, length,
    segment crc32, overlapping segments)
  * The structs published in docs compile and their _Static_assert holds.
    This checks the spec's structs are not wrong at the time they are published
  * The entry is at an address that fits a 20-bit immediate
  * boot is reachable from the hook (below 0x100000) and fits its slot
"""
import argparse
import os
import shutil
import re
import struct
import subprocess
import sys
import tempfile
import zlib

_HERE = os.path.dirname(os.path.abspath(__file__))
_FW = os.path.dirname(_HERE)
sys.path.insert(0, os.path.join(_FW, "host"))
import frcontainer                                              # noqa: E402
import build as fwbuild                                         # noqa: E402

SAMPLE = os.path.join(_FW, "sample")
BOOT_CAP = 0x800
CC = os.environ.get("CC", "cc")


def make_sample(tmp):
    """Compile the sample. _Static_assert passing is part of the check."""
    exe = os.path.join(tmp, "frc1_load")
    r = subprocess.run([CC, "-std=c11", "-O2", "-Wall", "-Wextra",
                        "-pedantic", "-Werror",
                        os.path.join(SAMPLE, "frc1_load.c"), "-o", exe],
                       capture_output=True, text=True, cwd=SAMPLE)
    if r.returncode:
        raise SystemExit("sample does not compile:\n" +
                         (r.stderr or r.stdout))
    return exe


def run(exe, *args):
    return subprocess.run([exe, *args], capture_output=True, text=True)


def fields(exe, path):
    r = run(exe, "fields", path)
    if r.returncode:
        raise SystemExit("fields failed: " + (r.stderr or r.stdout))
    out = {}
    for line in r.stdout.splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            out[k] = v
    return out


def asm_stub(entry, tool):
    """Have fr30-elf-as assemble the same thing. This is the reference."""
    with tempfile.TemporaryDirectory() as d:
        src = os.path.join(d, "stub.S")
        obj = os.path.join(d, "stub.o")
        bin_ = os.path.join(d, "stub.bin")
        with open(src, "w", encoding="utf-8", newline="\n") as f:
            f.write(f"\t.text\n\tldi:20\t#{entry:#x}, r12\n\tjmp\t@r12\n"
                    f"\tnop\n")
        subprocess.run([tool("as"), src, "-o", obj], check=True,
                       capture_output=True)
        subprocess.run([tool("objcopy"), "-O", "binary", obj, bin_],
                       check=True, capture_output=True)
        return open(bin_, "rb").read()


def corrupt(blob, how):
    b = bytearray(blob)
    if how == "magic":
        b[0:4] = b"FRC2"
    elif how == "crc32":
        b[0x30] ^= 1                    # flip 1 bit of build_id
    elif how == "total":
        struct.pack_into(">I", b, 0x0c, len(b) + 4)
    elif how == "segment":
        # Change 1 byte of the content, then fix the whole-file crc32.
        # Otherwise the file crc32 catches it first and the per-segment
        # crc32 is never tested
        off = struct.unpack_from(">I", b, 0x20)[0]
        data = struct.unpack_from(">I", b, off + 8)[0]
        b[data] ^= 0xFF
        struct.pack_into(">I", b, 0x14, 0)
        struct.pack_into(">I", b, 0x14, zlib.crc32(bytes(b)) & 0xFFFFFFFF)
    elif how == "truncated":
        return bytes(b[:len(b) // 2])
    elif how == "version":
        struct.pack_into(">I", b, 0x04, 2)
    return bytes(b)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args()
    fails = []

    def ok(cond, label, detail=""):
        print(f"  [{'ok ' if cond else 'NG '}] {label:<42}{detail}")
        if not cond:
            fails.append(label)

    m = fwbuild.load()
    code, elf, tool = fwbuild.compile_all(m)
    outs = fwbuild.target_frc1(m, code, tool)
    syms = fwbuild.symbols(elf, tool)

    tmp = tempfile.mkdtemp()
    try:
        exe = make_sample(tmp)

        # One per board, one container per product
        # (docs/frc1.md). So every check runs on each one, leaving no room to
        # produce a container that is right for only one of them.
        ok(len(outs) >= 1, "at least 1 FRC1 is produced", f"{len(outs)}")
        for suffix, blob, note in outs:
            c = frcontainer.parse(blob)
            path = os.path.join(tmp, f"CFW{suffix}.frc")
            with open(path, "wb") as f:
                f.write(blob)
            print(f"\n=== {suffix.lstrip('-')}: {len(blob)} bytes / {note}")
            ok(len(c["targets"]) == 1,
               "only 1 board per container", f"{len(c['targets'])} targets")

            # --- the format itself ---
            print("\n-- container --")
            try:
                frcontainer.check_overlaps(c["segments"])
                overlap = ""
            except frcontainer.FormatError as e:
                overlap = str(e)
            ok(not overlap, "segments do not overlap", overlap)
            r = run(exe, "verify", path)
            ok(r.returncode == 0, "the C reader accepts it",
               r.stdout.strip().split(": ", 1)[-1])

            names = [s["name"] for s in c["segments"]]
            ok(names == ["boot", "main", "bss"], "segments are boot, main, bss",
               str(names))
            boot = c["segments"][0]
            ok(boot["address"] < 0x100000,
               "boot is reachable from the hook", f"{boot['address']:#x}")
            ok(len(boot["data"]) <= BOOT_CAP, "boot fits its slot",
               f"{len(boot['data'])} / {BOOT_CAP} bytes")
            ok(all(t["entry"] == syms.get("frc1_entry")
                   for t in c["targets"]),
               "entry is frc1_entry for every board",
               f"{syms.get('frc1_entry', 0):#x}")
            bss = c["segments"][2]
            ok(set(bss["data"]) == {0} and len(bss["data"]) >= 4,
               "bss is zero and not empty", f"{len(bss['data'])} bytes")

            # --- C reader vs Python writer ---
            print("\n-- values read by the C reader vs values stored by the Python writer --")
            f = fields(exe, path)
            want = {
                "h.magic": "FRC1", "h.version": "1", "h.header_bytes": "64",
                "h.total_bytes": str(len(blob)), "h.name": c["name"],
                "h.target_count": str(len(c["targets"])),
                "h.segment_count": str(len(c["segments"])),
                "h.build_id": str(c["build_id"]), "h.flags": str(c["flags"]),
            }
            bad = {k: (f.get(k), v) for k, v in want.items() if f.get(k) != v}
            ok(not bad, "all header fields", "" if not bad else str(bad))

            bad = {}
            for i, t in enumerate(c["targets"]):
                for key in ("usb_vendor", "usb_product", "hook", "entry",
                            "resume_plain", "resume_mask", "complete", "staging",
                            "state", "loader_base", "loader_bytes",
                            "loader_crc32", "hook_crc32"):
                    got = f.get(f"t{i}.{key}")
                    exp = (f"{t[key]:#06x}" if key.startswith("usb")
                           else f"{t[key]:#010x}")
                    if got != exp:
                        bad[f"t{i}.{key}"] = (got, exp)
                if f.get(f"t{i}.name") != t["name"]:
                    bad[f"t{i}.name"] = (f.get(f"t{i}.name"), t["name"])
            ok(not bad, f"all fields of {len(c['targets'])} target rows",
               "" if not bad else str(bad))

            bad = {}
            for j, s in enumerate(c["segments"]):
                pairs = {"address": f"{s['address']:#010x}",
                         "bytes": str(len(s["data"])),
                         "file_offset": str(s["file_offset"]),
                         "crc32": f"{s['crc32']:#010x}",
                         "flags": str(s["flags"]),
                         "target_mask": f"{s['targets']:#010x}",
                         "name": s["name"]}
                for key, exp in pairs.items():
                    if f.get(f"s{j}.{key}") != exp:
                        bad[f"s{j}.{key}"] = (f.get(f"s{j}.{key}"), exp)
                exp = "1" if frcontainer.needs_window(s["address"]) else "0"
                if f.get(f"s{j}.needs_window") != exp:
                    bad[f"s{j}.needs_window"] = (f.get(f"s{j}.needs_window"), exp)
            ok(not bad, f"all fields of {len(c['segments'])} segment rows",
               "" if not bad else str(bad))

            # --- the 8 hook bytes ---
            print("\n-- the 8 bytes written to the hook --")
            for i, t in enumerate(c["targets"]):
                r = run(exe, "stub", path, str(i))
                want_hex = asm_stub(t["entry"], tool).hex()
                got = r.stdout.strip()
                ok(got == want_hex, f"target {i}: matches the assembler output",
                   got if got == want_hex else f"{got} != {want_hex}")

            # --- broken containers ---
            print("\n-- rejects broken containers --")
            for how in ("magic", "version", "crc32", "total", "segment",
                        "truncated"):
                p = os.path.join(tmp, f"bad-{how}.frc")
                with open(p, "wb") as fh:
                    fh.write(corrupt(blob, how))
                r = run(exe, "verify", p)
                ok(r.returncode != 0, f"rejects a container with broken {how}",
                   r.stderr.strip().split(": ", 1)[-1][:46])

            print("\n-- load order --")
            r = run(exe, "plan", path, "0")
            hook = f"{c['targets'][0]['hook']:#010x}"
            steps = [l for l in r.stdout.splitlines()
                     if l[:2] in ("1.", "2.", "3.", "4.", "5.")]
            ok(r.returncode == 0 and len(steps) == 5,
               "all 5 steps pass", f"{len(steps)} steps")
            lines = r.stdout.splitlines()
            hook_w = [i for i, l in enumerate(lines)
                      if l.startswith("  write") and hook in l]
            seg_w = [i for i, l in enumerate(lines)
                     if l.startswith("  write") and hook not in l]
            ok(bool(hook_w) and bool(seg_w) and hook_w[-1] > max(seg_w),
               "hook is written after the segments",
               f"hook={hook_w} ({hook}) segments={seg_w}")

        # --- the other sample ---
        # They must pass -Werror even without libusb. This is the first place
        # the header-only frq1.h (including the 0x40-band report layout) is compiled.
        print("\n-- sample compile --")
        for src in ("ts_grab.c",):
            r = subprocess.run([CC, "-std=c11", "-O2", "-Wall", "-Wextra",
                                "-pedantic", "-Werror", "-fsyntax-only",
                                "-I" + os.path.join(SAMPLE, "..", "src", "shared"),
                                os.path.join(SAMPLE, src)],
                               capture_output=True, text=True, cwd=SAMPLE)
            ok(r.returncode == 0, f"{src} compiles without libusb",
               "" if r.returncode == 0
               else (r.stderr or r.stdout).strip().splitlines()[0][:60])

        # --- structs published in docs ---
        print("\n-- structs published in docs --")
        for doc in ("docs/frc1.md", "docs/frq1.md", "docs/devices.md"):
            text = open(os.path.join(_FW, doc), encoding="utf-8").read()
            blocks = [b for b in re.findall(r"```c\n(.*?)```", text, re.S)
                      if "struct" in b or "_Static_assert" in b]
            bad = []
            for n, b in enumerate(blocks):
                src = ("#include <stdint.h>\n#include <stddef.h>\n"
                       "#include <stdio.h>\n#include <string.h>\n" + b +
                       "\nint main(void){return 0;}\n")
                cpath = os.path.join(tmp, f"doc{n}.c")
                with open(cpath, "w", encoding="utf-8", newline="\n") as fh:
                    fh.write(src)
                r = subprocess.run([CC, "-std=c11", "-Wall", "-Wextra",
                                    "-Wno-unused-function", "-Werror",
                                    "-fsyntax-only", cpath],
                                   capture_output=True, text=True)
                if r.returncode:
                    bad.append(f"block {n + 1}: " +
                               r.stderr.strip().splitlines()[0][:60])
            ok(blocks and not bad,
               f"structs in {os.path.basename(doc)} compile",
               f"{len(blocks)} blocks" if not bad else "; ".join(bad))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print("\ntest_frc1: " + ("PASS" if not fails else f"FAIL ({len(fails)})"))
    for x in fails:
        print("  - " + x)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
