#!/usr/bin/env python3
"""Check that this tree builds on its own, by actually copying it out and building it.

Copy the whole repository (except `build/`) to an empty place with nothing
above it, run `build.py` there, and compare the outputs with those built here.
You cannot claim "builds standalone" until you have tried it.

Needs: the fr30-elf toolchain (skipped if missing), arm-none-eabi for the
ARM image (compared only when present).
"""
import hashlib
import os
import shutil
import subprocess
import sys
import tempfile

_FW = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_FW, "host"))

import build as fwbuild                                          # noqa: E402
import genboards                                                 # noqa: E402


def md5(path):
    with open(path, "rb") as f:
        return hashlib.md5(f.read()).hexdigest()


def artifacts():
    """Outputs a build should produce. One frc1 per board."""
    d = genboards.load(_FW)
    frc1 = [f"frc1-{b['id']}.frc" for b in d["boards"] if b.get("loader")]
    out = ["resident.text.bin", "auxcore.bin"] + sorted(frc1)
    # The ARM image (src/armcore/) needs arm-none-eabi; build.py skips it
    # without one, so it is only compared when the toolchain is there.
    try:
        fwbuild.arm_toolchain()
        out.append("armcore.bin")
    except fwbuild.NoToolchain as e:
        print(f"  [-- ] armcore.bin not compared: {e}")
    return out


def main():
    fails = []

    def ok(cond, label, detail=""):
        print(f"  [{'ok ' if cond else 'NG '}] {label:<40}{detail}")
        if not cond:
            fails.append(label)

    print("\n-- copy out and build --")
    try:
        fwbuild.toolchain()
    except fwbuild.NoToolchain as e:
        print(f"  [-- ] no toolchain, skipping: {e}")
        print("\ntest_host: SKIP")
        return 0

    want = artifacts()
    here = subprocess.run([sys.executable, os.path.join(_FW, "host", "build.py")],
                          capture_output=True, text=True, cwd=_FW)
    ok(here.returncode == 0, "builds in this tree",
       (here.stderr or here.stdout).strip().splitlines()[-1][:40]
       if here.returncode else "")

    tmp = tempfile.mkdtemp()
    try:
        # Copy to a place with nothing above it. Fails if the build looks outside
        dst = os.path.join(tmp, "cfw")
        shutil.copytree(_FW, dst, ignore=shutil.ignore_patterns(
            "build", "__pycache__", "*.pyc", ".git"))
        ok(sorted(os.listdir(tmp)) == ["cfw"], "nothing else next to the copy")
        alone = subprocess.run([sys.executable,
                                os.path.join(dst, "host", "build.py")],
                               capture_output=True, text=True, cwd=tmp)
        detail = (alone.stderr or alone.stdout).strip().splitlines()
        ok(alone.returncode == 0, "builds from this tree alone",
           detail[-1][:46] if alone.returncode else "")

        print("\n-- same output bytes --")
        for name in want:
            a = os.path.join(_FW, "build", name)
            b = os.path.join(dst, "build", name)
            if not (os.path.exists(a) and os.path.exists(b)):
                ok(False, f"{name} exists in both",
                   f"here={os.path.exists(a)} alone={os.path.exists(b)}")
                continue
            ok(md5(a) == md5(b), f"{name} has the same bytes", md5(a))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print("\ntest_host: " + ("PASS" if not fails else f"FAIL ({len(fails)})"))
    for x in fails:
        print("  - " + x)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
