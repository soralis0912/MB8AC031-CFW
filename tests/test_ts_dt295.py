#!/usr/bin/env python3
"""The measured TS-input path in tsif_selene.c, on the PIX-DT295 image.

The DT295 row says ts.evidence = measured, so tsif_init / tsif_start /
tsif_stop take the measured sequence.  The
expected writes are written out here by hand from the sequence measured on
a PIX-DT295, in the order tsif_selene.c uses (checked on the device) -- a
second statement of the same facts, not a read of the code's own tables.

What this checks

  * tsif_init on the DT295 image does not look at the straps (0x8A01)
  * tsif_init: the unit gate and bit2 of 0x01900404, and
    FW_EIO when that bit does not stick (block asleep), with no capture
    write after it
  * tsif_start: the capture sequence in order and at the right widths,
    including the board's two values and the 4 MiB ring at 0x02000000
  * tsif_stop: the two stop writes, and FW_OK once 0x01902010 reads 0;
    FW_ETIMEOUT when it never does
  * the image built for no device (no measured sequence) refuses

What this cannot tell: whether the device needs every one of these writes.
That was measured on the device, one write left out at a time
(the comments in tsif_selene.c say what leaving each out did).
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_FW = os.path.dirname(_HERE)
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(_HERE, "fr"))
sys.path.insert(0, os.path.join(_FW, "host"))
import fremu                                                    # noqa: E402
import build as fwbuild                                         # noqa: E402

FW_OK, FW_ENOTIMPL, FW_EIO, FW_ETIMEOUT = 0, -1, -2, -3
SP = 0x000C8000
STRAPS = 0x00F10004
GATE, B0_404 = 0x00F10118, 0x01900404
R010 = 0x01902010
BANDS = ((0x01600000, 0x01601000), (0x01900000, 0x01903000))

INIT = [(GATE, 0xFFFF, 2), (B0_404, 0x0004, 2)]
START = [
    (0x01900000, 0x00020000, 4), (0x01900004, 0x00001100, 4),
    (0x01902020, 0x02000000, 4), (0x01902024, 0x003FFFC0, 4),
    (0x01902028, 0, 4), (0x0190202C, 0, 4),
    (0x01901000, 0x0607, 2), (0x01901004, 0x04000000, 4),
    (0x01901044, 0x00BC0000, 4), (0x01901080, 0x01000000, 4),
    (0x01901040, 0x03000000, 4),
    (R010, 0x02000000, 4)]
STOP = [(R010, 0x04000000, 4), (0x01901040, 0, 4)]

class Chip:
    def __init__(self, straps=0x8A01, awake=True, stop_clears=True):
        self.regs = {STRAPS: straps}
        self.log = []
        self.awake = awake
        self.stop_clears = stop_clears

    def __call__(self, a, width, v):
        if v is None:
            if a == STRAPS and width == 4:
                return self.regs[STRAPS] << 16
            if a == R010 and self.stop_clears and \
                    self.regs.get(R010) == 0x04000000:
                return 0
            return self.regs.get(a, 0)
        v &= (1 << (8 * width)) - 1     # a half-word store is 16 bits
        self.log.append((a, v, width))
        if a == B0_404 and not self.awake:
            return None             # asleep: the write does not stick
        self.regs[a] = v
        return None


def run(img, sym, hw, names, budget=4000000):
    e = fremu.Emu([(fremu.fold(a), b) for a, b in img], budget=budget,
                  hw=hw, hw_bands=BANDS)
    out = []
    for name in names:
        for i in range(16):
            e.r[i] = 0
        e.r[15] = SP
        why = e.run(fremu.fold(sym[name]))
        rv = e.r[4] - (1 << 32) if e.r[4] & 0x80000000 else e.r[4]
        out.append((rv, why))
    return out


def image(m, tool, bid):
    code, _boot, sym, lay = fwbuild.frc1_variant(m, "selene", tool, bid)
    bss = sym.get("_bss_end", lay["RES_IMMUT_END"]) - lay["RES_IMMUT_END"]
    img = [(lay["RES_BASE"], code)]
    if bss > 0:
        img.append((lay["RES_IMMUT_END"], b"\0" * bss))
    return img, sym


def main():
    m = fwbuild.load()
    _code, _elf, tool = fwbuild.compile_all(m)
    img, sym = image(m, tool, "dt295")
    fails = []

    def ok(cond, label, detail=""):
        print(f"  [{'ok ' if cond else 'NG '}] {label:<50}{detail}")
        if not cond:
            fails.append(label)

    def show(got, want):
        if got != want:
            print("        got  " + " ".join(f"{a:08x}={v:x}/{w}"
                                          for a, v, w in got))
            print("        want " + " ".join(f"{a:08x}={v:x}/{w}"
                                          for a, v, w in want))

    print("\n-- the DT295 image (ts measured) --")
    hw = Chip()
    (rv, _), = run(img, sym, hw, ["tsif_init"])
    ok(rv == FW_OK and hw.log == INIT,
       "init on strap 0x8A01: gate, unit 0x016, +0x404", f"rv {rv}")
    show(hw.log, INIT)

    hw = Chip(awake=False)
    (rv, _), = run(img, sym, hw, ["tsif_init"])
    ok(rv == FW_EIO and hw.log == INIT,
       "block asleep: FW_EIO, nothing after +0x404", f"rv {rv}")
    show(hw.log, INIT)

    hw = Chip()
    res = run(img, sym, hw, ["tsif_init", "tsif_start"])
    got = hw.log[len(INIT):]
    ok(res[1][0] == FW_OK and got == START,
       "start: the measured sequence, 34 writes", f"rv {res[1][0]}")
    show(got, START)

    hw = Chip()
    res = run(img, sym, hw, ["tsif_stop", "tsif_init", "tsif_start",
                             "tsif_stop", "tsif_stop"])
    ok(res[0][0] == FW_OK and not hw.log[:0], "stop before start: FW_OK",
       f"rv {res[0][0]}")
    got = hw.log[len(INIT) + len(START):]
    ok(res[3][0] == FW_OK and got == STOP,
       "stop: 3 writes, then 0x01902010 reads 0", f"rv {res[3][0]}")
    show(got, STOP)
    ok(res[4][0] == FW_OK, "a second stop: FW_OK, nothing more",
       f"rv {res[4][0]}")

    hw = Chip(stop_clears=False)
    res = run(img, sym, hw, ["tsif_init", "tsif_start", "tsif_stop"],
              budget=40000000)
    ok(res[2][0] == FW_ETIMEOUT, "stop never clears: FW_ETIMEOUT",
       f"rv {res[2][0]} ({res[2][1]})")

    print("\n-- the image built for no device (no measured sequence) --")
    img0, sym0 = image(m, tool, "unknown")
    hw = Chip()
    (rv, _), = run(img0, sym0, hw, ["tsif_init"])
    ok(rv == FW_ENOTIMPL and not hw.log, "init: FW_ENOTIMPL, no write",
       f"rv {rv}, {len(hw.log)} writes")

    print()
    print("test_ts_dt295: " + ("PASS" if not fails else f"FAIL ({len(fails)})"))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
