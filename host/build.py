"""Build CFW, the firmware for MB8AC031 / MB86M0x.

    python host/build.py                 # every target
    python host/build.py frc1            # just one
    python host/build.py --list
    python host/build.py --map           # what landed where

Targets come from `config/fw.json`:

| target     | what it makes | how it runs |
|---|---|---|
| `frc1`     | the `FRC1` container | the resident loader's hook -- the only path that works on hardware |

Tests want the firmware as memory rather than as a container: `loaded()`
returns the bands to hand to `fremu`.

Everything apart from the entry assembly is C, so this needs the `fr30-elf`
toolchain (docs/toolchain.md has the recipe).  It is looked for in
`FR30_TOOLCHAIN`, then `PATH`, then the usual places.
"""
import argparse
import json
import os
import re
import shutil
import subprocess
import sys

# _HERE is the CFW/ root.  config/, src/, build/ and the `image` paths in the
# device files are all relative to it
_HOST = os.path.dirname(os.path.abspath(__file__))
_HERE = os.path.dirname(_HOST)

# The container code (frcontainer.py) lives in
# host/.  Looking in exactly one place keeps it clear which version built what
sys.path.insert(0, _HOST)

import frcontainer   # noqa: E402
import genboards     # noqa: E402

MANIFEST = os.path.join(_HERE, "config", "fw.json")
AUX_MANIFEST = os.path.join(_HERE, "config", "auxcore.json")
ARM_MANIFEST = os.path.join(_HERE, "config", "armcore.json")
BUILD = os.path.join(_HERE, "build")

# Searched when nothing is configured; FR30_TOOLCHAIN and PATH come first
TOOLCHAIN_DIRS = [os.path.expanduser("~/opt/fr30-elf/bin"),
                  "/usr/local/fr30-elf/bin",
                  "/opt/fr30-elf/bin"]

# The ARM image (src/armcore/) needs a second toolchain, arm-none-eabi.  It
# is optional: without it the ARM image is skipped and everything else builds
ARM_TOOLCHAIN_DIRS = [os.path.expanduser("~/opt/arm-none-eabi/bin"),
                      "/usr/local/arm-none-eabi/bin",
                      "/opt/arm-none-eabi/bin",
                      "/opt/homebrew/bin"]

LAYOUT_KEYS = ("RES_BASE", "RES_IMMUT_END", "RES_META", "RES_STACK_TOP")


class NoToolchain(Exception):
    """The fr30-elf (or, for the ARM image, arm-none-eabi) toolchain is not
    installed."""


def _num(v):
    """Accept both a JSON "0x..." string and a number."""
    return int(str(v), 0)


def _read_json(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def _read_bytes(path):
    with open(path, "rb") as f:
        return f.read()


def load():
    """Read config/fw.json and the device tables.

    What differs per device (how I2C is reached, demodulator, tuner, card)
    is not written in C.  Each image is linked for one device, with that
    device's row: genboards.py writes it as build/board_<id>.c.  Every driver
    source is compiled into every image, so a driver no device selects
    cannot rot silently.
    """
    m = _read_json(MANIFEST)
    d = genboards.load(_HERE)
    m["sources"] = list(m["sources"]) + [s for s in genboards.driver_sources(d)
                                         if s not in m["sources"]]
    m["_tables"] = d
    m["_boards"] = d["boards"]
    m["_drivers"] = d["drivers"]
    m["_none"] = d["none"]
    return m


def _find_toolchain(prefix, env, dirs):
    """-> a function returning the path of <prefix>-<tool>"""
    cands = []
    pre = os.environ.get(env)
    if pre:
        cands.append(os.path.join(pre, "bin"))
    cands.append(None)                      # PATH
    cands += dirs
    for d in cands:
        gcc = (os.path.join(d, f"{prefix}-gcc") if d
               else shutil.which(f"{prefix}-gcc"))
        if gcc and os.path.exists(gcc):
            base = os.path.dirname(gcc)
            return lambda t: os.path.join(base, f"{prefix}-{t}")
    raise NoToolchain(
        f"{prefix}-gcc not found.  Point {env} at its prefix, or put"
        f" it on PATH or in {dirs[0]}.  See docs/toolchain.md")


def toolchain():
    """-> a function returning the path of fr30-elf-<tool>"""
    return _find_toolchain("fr30-elf", "FR30_TOOLCHAIN", TOOLCHAIN_DIRS)


def arm_toolchain():
    """-> a function returning the path of arm-none-eabi-<tool>"""
    return _find_toolchain("arm-none-eabi", "ARM_TOOLCHAIN",
                           ARM_TOOLCHAIN_DIRS)


def _run(cmd, what, **kw):
    """Run a tool; on failure stop with `what` in front of its output."""
    r = subprocess.run(cmd, capture_output=True, text=True, **kw)
    if r.returncode:
        raise SystemExit(f"{what}:\n{(r.stderr or r.stdout).rstrip()}")
    return r


def _link(tool, objs, script, stem, ldflags=()):
    """Link and flatten .text.  -> (.text bytes, elf path)

    Only -j .text: .boot sits at 0x000E1000, and flattening everything would
    give a file spanning 0xE1000 to 0x80381000.
    """
    elf = os.path.join(BUILD, stem + ".elf")
    _run([tool("ld")] + list(ldflags) + ["-T", os.path.join(_HERE, script),
          "-Map", os.path.join(BUILD, stem + ".map"), "-o", elf] + objs,
         f"ld ({stem})")
    raw = os.path.join(BUILD, stem + ".text.bin")
    _run([tool("objcopy"), "-O", "binary", "-j", ".text", elf, raw],
         f"objcopy ({stem})")
    return _read_bytes(raw), elf


def _compile(m, tool, rel, obj):
    src = os.path.join(_HERE, rel)
    if not os.path.exists(src):
        raise SystemExit(f"{rel} is missing")
    if rel.endswith(".c") or (rel.endswith(".S") and m.get("asm_cpp")):
        cmd = [tool("gcc")] + m.get("cflags", "-O2").split() + ["-c", src,
                                                                "-o", obj]
    else:
        cmd = [tool("as"), src, "-o", obj]
    r = _run(cmd, rel, cwd=_HERE)
    if r.stderr.strip():
        print(f"  {rel}:\n{r.stderr.rstrip()}")
    return obj


def board_obj(m, tool, bid):
    """Compile that device's row (build/board_<id>.c).  -> object path"""
    cache = m.setdefault("_row_obj", {})
    if bid not in cache:
        src = genboards.write_row(_HERE, m["_tables"], bid)
        cache[bid] = _compile(m, tool, os.path.relpath(src, _HERE),
                              os.path.join(BUILD, f"board_{bid}.o"))
    return cache[bid]


def compile_all(m, stem="resident", tool=None):
    """Compile and link every source.  -> (flat bytes, elf path, tool)

    `stem` names the intermediates.  The main FR image and the auxiliary
    unit's image (config/auxcore.json) share the toolchain and the ISA, so they
    are built by the same function (the auxiliary one reuses
    src/shared/fjcore.c rather than a copy).

    The main image is linked here with the row that names no device
    (`unknown`): it is what most tests run.  board_image() and frc1_variant() link it for a device.

    The ARM image passes its own `tool` (arm_toolchain).  Its manifest sets
    `asm_cpp`, so its .S files go through gcc (they include armcore.h), and
    `ldflags`.
    """
    tool = tool or toolchain()
    os.makedirs(BUILD, exist_ok=True)
    obj_of = {}
    for rel in m["sources"]:
        obj_of[rel] = _compile(m, tool, rel, os.path.join(
            BUILD, stem + "_" + rel.replace("/", "_").replace("\\", "_") + ".o"))
    if stem == "resident":
        # The main image carries the auxiliary unit's program, so that a
        # transport that needs it (i2c/auxslot.c) can place it on the frc1
        # path, where the host's window cannot reach the aux RAM.
        rel = aux_blob(tool)
        obj_of[rel] = _compile(m, tool, rel, os.path.join(
            BUILD, stem + "_" + rel.replace("/", "_") + ".o"))
    m["_obj_of"] = obj_of

    objs = list(obj_of.values())
    if "_tables" in m:
        objs.append(board_obj(m, tool, m["_none"]))
    code, elf = _link(tool, objs, m["link"], stem,
                      m.get("ldflags", "").split())
    m["_elf"] = elf
    return code, elf, tool


def board_image(m, tool, bid):
    """The selene-layout image linked with one device's row.

    -> (flat bytes, elf path).  compile_all() must have run.
    """
    objs = list(m["_obj_of"].values()) + [board_obj(m, tool, bid)]
    return _link(tool, objs, m["link"], f"resident-{bid}")


def section(elf, tool, name):
    """-> that section as flat bytes"""
    out = os.path.join(BUILD, f"section{name}.bin")
    _run([tool("objcopy"), "-O", "binary", "-j", name, elf, out],
         f"objcopy {name}")
    return _read_bytes(out)


def symbols(elf, tool):
    out = _run([tool("nm"), elf], "nm").stdout
    return {f[2]: int(f[0], 16)
            for f in (line.split() for line in out.splitlines()) if len(f) == 3}


def _need_layout(out, where):
    for k in LAYOUT_KEYS:
        if k not in out:
            raise SystemExit(f"{k} is missing from {where}")
    return out


def bss_base(lay):
    """Where .bss starts: RES_BSS if the layout has one (dione keeps it in
    another band), otherwise right above the code."""
    return lay.get("RES_BSS", lay["RES_IMMUT_END"])


def layout(root=_HERE):
    """Read RES_* from src/main/resident.h (one source for the addresses)."""
    out = {}
    path = os.path.join(root, "src", "main", "resident.h")
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            f = line.split()
            if len(f) >= 3 and f[0] == "#define" and f[1].startswith("RES_"):
                out[f[1]] = int(f[2].rstrip("u"), 0)
    return _need_layout(out, "src/main/resident.h")


def loaded(m, code, tool, elf=None):
    """The firmware as memory after loading.

    -> [(address, bytes), ...], which is what the tests feed the emulator.
    `elf` is the image `code` came from (default: compile_all's).

    .bss is NOLOAD, so it is not in the flat image, but on the device it is
    zero (the FRC1 container carries a zero segment of exactly that length).
    So it is added here as a zero band too.  Without it statics would start
    as 0xFF and the tests would check something the hardware never runs.
    """
    lay = layout()
    syms = symbols(elf or m["_elf"], tool)
    bss = syms.get("_bss_end", lay["RES_IMMUT_END"]) - lay["RES_IMMUT_END"]
    bands = [(lay["RES_BASE"], code)]
    if bss > 0:
        bands.append((lay["RES_IMMUT_END"], b"\0" * bss))
    return bands


PLAIN = 0x3FFFFFFF      # 0x8xxxxxxx is the cached view; strip it


def _equ(path):
    """Read the .equ lines of an assembly file (one source for the addresses)."""
    out = {}
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            f = line.split()
            if len(f) >= 3 and f[0] == ".equ":
                out[f[1].rstrip(",")] = int(f[2].rstrip(","), 0)
    return out


# The loader generation decides the entry and the placement.  selene
# (MB8AC031 / MB86M0x) and dione (MB8AC018 / MB86H5x) answer the same 0xBC
# request, but with different loaders that take the request differently.
# Each config `loader` picks one with `family`.
#
# These are choices about where our code goes, not facts about the loader.
# The loader's addresses and bytes live in the config rows (_loaders).
FRC1_ENTRY = {
    "selene": "src/main/entry_frc1_selene.S",
    "dione":  "src/main/entry_frc1_dione.S",
}
FRC1_LINK = {
    "selene": "src/main/link_selene.ld",
    "dione":  "src/main/link_dione.ld",
}


def _ld_layout(path):
    """Read the RES_* assignments of a linker script (dione's placement)."""
    out = {}
    pat = re.compile(r"^\s*(RES_[A-Z_]+)\s*=\s*(0x[0-9a-fA-F]+|\d+)\s*;")
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            m = pat.match(line)
            if m:
                out[m.group(1)] = int(m.group(2), 0)
    return _need_layout(out, os.path.relpath(path, _HERE))


def family_layout(family):
    """Code, .bss and stack addresses for that generation."""
    if family not in FRC1_LINK:
        raise SystemExit(f"unknown loader family '{family}'; "
                         f"FRC1_ENTRY has {list(FRC1_ENTRY)}")
    if family == "selene":
        return layout()
    return _ld_layout(os.path.join(_HERE, FRC1_LINK[family]))


LOADER_ADDRS = ("hook", "boot", "resume_plain", "resume_mask", "complete",
                "staging", "state")


def _loaders(boards):
    """Device `loader` entries -> the targets passed to frcontainer.build.

    These are facts about the DEVICE's resident loader, measured per
    device, not about our code.

    The values are kept per device on purpose.  Two selene devices can
    agree on all ten, but that is a measurement that happened to match on
    those two devices, not a property of the generation.  Folding them into a shared
    spec would let unmeasured devices inherit them silently (never put
    inferred values in the table).

    The USB ID comes from the row's `usb` (not from `loader`).  A device with
    a null `loader` is not a target: there is no loader crc32 and the hook
    address has not been measured.

    `crc32` is the fingerprint of the first `bytes` bytes of that device's
    resident loader, stored as a number.  The loader image itself is not in
    this tree; the install check needs only the fingerprint.  To work one
    out for a new device, dump its loader and take zlib.crc32 of the first `bytes` bytes.
    Null disables the check.
    """
    out = []
    for b in boards:
        ld = b.get("loader")
        if not ld:
            continue
        where = f"{b.get('_file', b['id'])} loader"
        for key in ("name", "family", "base", "bytes", "hook_crc32",
                    "crc32") + LOADER_ADDRS:
            if key not in ld:
                raise SystemExit(f"{where}: '{key}' is missing")
        if ld["family"] not in FRC1_ENTRY:
            raise SystemExit(f"{where}: family is '{ld['family']}'; "
                             f"known families are {list(FRC1_ENTRY)}")
        if not b.get("usb"):
            raise SystemExit(f"{where}: a row with usb null cannot be a"
                             " target (it names no device)")
        vid, pid = (int(x, 16) for x in b["usb"].split(":"))
        base, size = _num(ld["base"]), _num(ld["bytes"])
        crc = 0
        if ld["crc32"] is not None:
            crc = _num(ld["crc32"]) & 0xFFFFFFFF
            if crc == 0:
                raise SystemExit(f"{where}: crc32 is 0; write null to say"
                                 " the loader has not been fingerprinted")
        hook_crc = _num(ld["hook_crc32"]) & 0xFFFFFFFF
        if hook_crc == 0:
            raise SystemExit(f"{where}: hook_crc32 cannot be 0; it is the"
                             " fingerprint of the hook's eight bytes")
        t = {"_id": b["id"], "family": ld["family"], "name": ld["name"],
             "usb_vendor": vid, "usb_product": pid, "hook_bytes": 8,
             "hook_crc32": hook_crc,
             "loader_base": base, "loader_bytes": size, "loader_crc32": crc}
        t.update({k: _num(ld[k]) for k in LOADER_ADDRS})
        out.append(t)
    if not out:
        raise SystemExit(
            f"no FRC1 targets: no row in {genboards.DEVICES}/ has a loader"
            " (only devices whose loader was measured qualify)")
    return out


def frc1_variant(m, family, tool, bid):
    """(code, boot, syms, layout) for one device, linked with that
    generation's entry and that device's row.

    The C objects are shared; only the entry .S, the linker script and the
    row change.  Instructions carry absolute addresses, so the flat `.text`
    is extracted again for each image.
    """
    key = f"_frc1_{family}_{bid}"
    if key in m:
        return m[key]
    if family not in FRC1_ENTRY:
        raise SystemExit(f"unknown loader family '{family}'; "
                         f"FRC1_ENTRY has {list(FRC1_ENTRY)}")
    entry_obj = os.path.join(BUILD, "resident_entry_" + family + ".o")
    ekey = "_entry_" + family
    if ekey not in m:
        _run([tool("as"), os.path.join(_HERE, FRC1_ENTRY[family]), "-o",
              entry_obj], FRC1_ENTRY[family], cwd=_HERE)
        m[ekey] = entry_obj
    objs = [o for rel, o in m["_obj_of"].items()
            if rel not in FRC1_ENTRY.values()]
    objs += [board_obj(m, tool, bid), m[ekey]]
    code, elf = _link(tool, objs, FRC1_LINK[family],
                      f"resident_{family}-{bid}")
    m[key] = (code, section(elf, tool, ".boot"), symbols(elf, tool),
              family_layout(family))
    return m[key]


def target_frc1(m, code, tool):
    """`FRC1` -- the container for the plaintext path.

    This is the path that works on real hardware: the host writes the
    segments into RAM through the 0xBC window and redirects eight bytes of
    the resident loader to our boot stub.  No signature, no servicer.

    Three segments.  `boot` is where the hook lands; the stub's immediate is
    20 bits, so it must be below 0x100000.  `main` is the firmware.  `bss` is
    zeros: only declared bytes are written, so statics must be declared or
    they start with whatever the SRAM held at power-on.

    The generation picks the entry.  A `dione` loader passes the request
    descriptor in r10 and cannot have the body at 0x80380000 (external
    memory is off in the loader state), so `family` selects
    entry_frc1_dione.S and link_dione.ld.

    One container per device (`build/frc1-<id>.frc`), and the image inside is linked with that device's row: the
    file names the device, so installing cannot pick the wrong device's hook,
    and the firmware never has to work out which device it is on.

    -> [(suffix, bytes, note), ...]
    """
    spec = m["targets"]["frc1"]
    if not m.get("_elf"):
        raise SystemExit("no ELF")
    only = m.get("_only")
    outs = []
    for t in _loaders(m["_boards"]):
        if only and t["_id"] not in only:
            continue
        family = t["family"]
        code_f, boot, syms, lay = frc1_variant(m, family, tool, t["_id"])
        entry_src = FRC1_ENTRY[family]
        equ = _equ(os.path.join(_HERE, entry_src))

        # The entry stub has addresses baked in.  If it could be built while
        # disagreeing with the table, the device would jump somewhere else.
        # Check only what the entry actually hard-codes (entry_frc1_selene.S
        # leaves the read/write path to the loader and does not have them;
        # entry_frc1_dione.S does)
        for key in ("staging", "complete", "resume_plain", "resume_mask"):
            want = equ.get(key.upper())
            if want is not None and t[key] != want:
                raise SystemExit(
                    f"{t['name']}: {key} is {t[key]:#010x}, but "
                    f"{entry_src} hard-codes {want:#010x}; fix both")
        if lay["RES_STACK_TOP"] != equ.get("RES_STACK_TOP"):
            raise SystemExit(
                f"{family}: stack top {lay['RES_STACK_TOP']:#010x} disagrees "
                f"with {equ.get('RES_STACK_TOP', 0):#010x} in {entry_src}")
        if "frc1_entry" not in syms:
            raise SystemExit(f"{family}: frc1_entry is missing")

        # The boot address comes from this generation's link.  A device
        # file's `boot` states that device's free space, so check they agree
        boot_base = syms["frc1_entry"]
        if t["boot"] != boot_base:
            raise SystemExit(
                f"{t['name']}: boot is {t['boot']:#010x}, but .boot is "
                f"linked at {boot_base:#010x} (BOOT_BASE in "
                f"{FRC1_LINK[family]}); this device needs its own link")

        segs = [
            {"name": "boot", "address": boot_base, "data": boot,
             "flags": frcontainer.SEG_CODE | frcontainer.SEG_VERIFY |
                      frcontainer.SEG_ENTRY},
            {"name": "main", "address": lay["RES_BASE"] & PLAIN, "data": code_f,
             "flags": frcontainer.SEG_CODE | frcontainer.SEG_VERIFY},
        ]
        bss_at = bss_base(lay)
        bss = syms.get("_bss_end", bss_at) - bss_at
        if bss > 0:
            bss += -bss % 4
            segs.append({"name": "bss", "address": bss_at & PLAIN,
                         "data": b"\0" * bss,
                         "flags": frcontainer.SEG_VERIFY})

        t["entry"] = t["boot"]
        blob = frcontainer.build(m["name"], [t], segs,
                                 build_id=int(spec.get("build_id", 0)))
        # Read back what was built; parse checks the crc32s and overlaps
        try:
            frcontainer.parse(blob)
        except frcontainer.FormatError as e:
            raise SystemExit(f"{t['_id']}: the built container is broken: {e}")
        note = (f"{t['name']} ({family}), {len(segs)} segments "
                f"({', '.join(s['name'] for s in segs)}), read back and checked")
        outs.append((f"-{t['_id']}", blob, note))
    return outs


AUX_UNIT_BASE = 0x01700000      # start of the auxiliary unit's band; local = main - this



BUILDERS = {"frc1-container": target_frc1}


def show_list(m):
    print(f"{m['name']}  base {m['base']}")
    for name, t in m["targets"].items():
        print(f"  {name:10s} {t['kind']:20s} {t.get('comment', '')}")
    print()
    # The side-by-side device table (genboards.py prints the same one)
    print(f"device table ({genboards.DEVICES}/ and {genboards.UNTESTED}/, "
          f"drivers in {genboards.DRIVERS}):")
    genboards.table(m["_boards"], m["_drivers"],
                    out=lambda line: print("  " + line))
    for b in m["_boards"]:
        print(f"  {b['id']:<10} {b['name']}")
    # The table says how each device is built, not that it works
    print("  " + genboards.ran_line(m["_boards"]))
    un = genboards.untested(m["_tables"])
    if un:
        print(f"  {len(un)} in {genboards.UNTESTED}/ (never run on hardware):"
              f" {', '.join(un)} -- once one runs, move it to"
              f" {genboards.DEVICES}/ and fill in ran")
    print("\n  sources (in every image, with the device's row):")
    for rel in m["sources"]:
        print(f"    {rel}")
    print("\n  Every module is in place; the empty ones return FW_ENOTIMPL."
          "\n  Status of each module: docs/modules.md.")


AUX_BLOB = "build/auxcore_image.c"


def aux_blob(tool):
    """Build src/auxcore/ and write it as C for the main image.  -> path

    The file defines `aux_image` (the flat .text), its length, the .bss
    bytes that follow it, the main-side address it is linked for and its
    entry as the auxiliary core sees it.  i2c/auxslot.c copies it to that
    address and points the core's reset vector at the entry.
    """
    am = _read_json(AUX_MANIFEST)
    acode, aelf, _t = compile_all(am, stem="auxcore", tool=tool)
    sym = symbols(aelf, tool)
    for name in ("aux_entry", "_text_end", "_bss_end"):
        if name not in sym:
            raise SystemExit(f"aux: {name} not found")
    base = _num(am["base"])
    if sym["aux_entry"] != base - AUX_UNIT_BASE:
        raise SystemExit(f"aux: aux_entry {sym['aux_entry']:#x} is not the "
                         f"start of the image ({base - AUX_UNIT_BASE:#x})")
    acode += bytes(-len(acode) % 4)
    bss = sym["_bss_end"] - sym["_text_end"]
    bss += -bss % 4
    lines = ["/* Generated by host/build.py from src/auxcore/. Do not edit. */",
             "const unsigned char aux_image[] __attribute__((aligned(4))) = {"]
    for i in range(0, len(acode), 12):
        lines.append("    " + ", ".join(f"0x{b:02x}" for b in acode[i:i + 12])
                     + ",")
    lines += ["};",
              f"const unsigned aux_image_len = {len(acode)}u;",
              f"const unsigned aux_image_bss = {bss}u;",
              f"const unsigned aux_image_base = {base:#010x}u;",
              f"const unsigned aux_image_entry = {sym['aux_entry']:#010x}u;",
              ""]
    path = os.path.join(_HERE, AUX_BLOB)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    return AUX_BLOB


def build_aux():
    """Build the auxiliary unit's image into build/auxcore.bin.  -> success

    It is a separate program for a separate CPU, so it has its own manifest
    and build.  It shares the ISA and reuses src/shared/fjcore.c, so it goes
    through compile_all.
    """
    if not os.path.exists(AUX_MANIFEST):
        return True
    am = _read_json(AUX_MANIFEST)
    try:
        acode, _elf, _tool = compile_all(am, stem="auxcore")
    except SystemExit as e:
        print(f"  aux: {e}")
        return False
    dest = os.path.join(BUILD, "auxcore.bin")
    with open(dest, "wb") as f:
        f.write(acode)
    print(f"  {'aux':10s} {len(acode):7d} bytes -> "
          f"{os.path.relpath(dest, _HERE)}   "
          f"auxiliary unit @ {_num(am['base']):#010x} (src/auxcore/README.md)")
    return True


def build_arm():
    """Build our ARM image into build/armcore.bin.  -> success

    A program for the ARM cores (src/armcore/), with its own manifest and its
    own toolchain.  Without arm-none-eabi it is skipped, not failed: nothing
    else depends on it, and the main image carries no copy of it (the host
    loads it with ARM_LOAD, docs/arm.md).
    """
    if not os.path.exists(ARM_MANIFEST):
        return True
    am = _read_json(ARM_MANIFEST)
    try:
        atool = arm_toolchain()
    except NoToolchain as e:
        print(f"  {'arm':10s} skipped: {e}")
        return True
    try:
        acode, _elf, _tool = compile_all(am, stem="armcore", tool=atool)
    except SystemExit as e:
        print(f"  arm: {e}")
        return False
    dest = os.path.join(BUILD, "armcore.bin")
    with open(dest, "wb") as f:
        f.write(acode)
    print(f"  {'arm':10s} {len(acode):7d} bytes -> "
          f"{os.path.relpath(dest, _HERE)}   ARM core @ ITCM 0 "
          f"(src/armcore/README.md)")
    return True


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("targets", nargs="*")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--map", action="store_true")
    ap.add_argument("--boards", metavar="ID,ID",
                    help="build the FRC1 container only for these devices"
                         " (default: every device with a loader); ID is the"
                         " file name of config/devices/<ID>.json")
    a = ap.parse_args()
    m = load()
    if a.boards:
        m["_only"] = [genboards.board(m["_tables"], i)["id"]
                      for i in a.boards.split(",")]

    if a.list:
        show_list(m)
        return 0

    try:
        code, elf, tool = compile_all(m)
    except NoToolchain as e:
        print(f"cannot build: {e}")
        return 1
    print(f"{m['name']}: {len(code)} bytes @ {_num(m['base']):#010x}  "
          f"({len(m['sources'])} sources)")
    if a.map:
        r = subprocess.run([tool("nm"), "-n", elf], capture_output=True,
                           text=True)
        for line in r.stdout.splitlines():
            if line.strip():
                print("  " + line)
        r = subprocess.run([tool("size"), elf], capture_output=True, text=True)
        print("  " + "\n  ".join(r.stdout.rstrip().splitlines()))

    rc = 0 if build_aux() else 1
    if not build_arm():
        rc = 1
    for name in a.targets or list(m["targets"]):
        t = m["targets"].get(name)
        if t is None:
            print(f"  {name}: no such target")
            rc = 1
            continue
        fn = BUILDERS.get(t["kind"])
        if fn is None:
            print(f"  {name}: no builder for kind {t['kind']}")
            rc = 1
            continue
        try:
            outs = fn(m, code, tool)
        except SystemExit as e:
            print(f"  {name}: {e}")
            rc = 1
            continue
        # frc1 makes one file per device (build/frc1-<id>.frc)
        for suffix, blob, note in outs:
            dest = os.path.join(BUILD, name + suffix + t.get("ext", ".bin"))
            with open(dest, "wb") as f:
                f.write(blob)
            print(f"  {name + suffix:10s} {len(blob):7d} bytes -> "
                  f"{os.path.relpath(dest, _HERE)}   {note}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
