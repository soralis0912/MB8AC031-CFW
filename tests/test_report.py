#!/usr/bin/env python3
"""Check, in the machine code, that no store goes through a base added to itself.

    python tests/test_report.py

Why: reading the C is not enough.  GCC 14.2.0's fr30 back end sometimes
loads a displacement into the register that holds the base address and adds
the register to itself (`ldi:32 imm,rX` then `addn rX,rX`), so a store goes
near 0xFFFFFFxx instead of where the C says.  The C is correct; the code is
not.  This looks for that pattern anywhere in build/resident.elf.

Needs the fr30-elf toolchain (skipped if missing).
"""
import os
import re
import subprocess
import sys

_FW = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_FW, "host"))

import build as fwbuild                                          # noqa: E402

ELF = os.path.join(_FW, "build", "resident.elf")

_LDI = re.compile(r"^[0-9a-f]+:\s+ldi:32 (0x[0-9a-f]+),(r\d+)$")
_ADDN = re.compile(r"^[0-9a-f]+:\s+addn (r\d+),(r\d+)$")
_R4_OTHER = re.compile(r"^[0-9a-f]+:\s+(?:mov r\d+,r4|ld @.*,r4|addn .*,r4)$")


def disasm(tool):
    r = subprocess.run([tool("objdump"), "-d", "--no-show-raw-insn", ELF],
                       capture_output=True, text=True)
    if r.returncode:
        raise SystemExit(f"objdump failed: {r.stderr.strip()}")
    return [line.strip() for line in r.stdout.splitlines()]


def self_add_sites(lines):
    """Every `ldi:32 imm,rX` directly followed by `addn rX,rX`."""
    out = []
    for i in range(len(lines) - 1):
        a, b = _LDI.match(lines[i]), _ADDN.match(lines[i + 1])
        if a and b and b.group(1) == b.group(2) == a.group(2):
            after = lines[i + 2] if i + 2 < len(lines) else ""
            out.append((lines[i], lines[i + 1], after))
    return out


def main():
    try:
        tool = fwbuild.toolchain()
    except fwbuild.NoToolchain as e:
        print(f"test_report: SKIP ({e})")
        return 0

    r = subprocess.run([sys.executable, os.path.join(_FW, "host", "build.py")],
                       capture_output=True, text=True, cwd=_FW)
    if r.returncode or not os.path.exists(ELF):
        print((r.stderr or r.stdout).rstrip())
        print("test_report: FAIL (build failed)")
        return 1

    lines = disasm(tool)
    ok = True

    print("-- 1. no store through a base added to itself --")
    bad = self_add_sites(lines)
    ok = ok and not bad
    print(f"  [{'ok ' if not bad else 'BAD'}] `ldi:32 imm,rX` + `addn rX,rX`:"
          f" {len(bad)} site(s)")
    for a, b, c in bad:
        print(f"        {a} | {b} | {c}")

    print(f"\ntest_report: {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
