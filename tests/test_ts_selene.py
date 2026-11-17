#!/usr/bin/env python3
"""Check the selene TS input (tsif_selene.c) and output (tsout_selene.c).

This checks that the code does what the docs say, not that the docs are
right.

What this checks

  tsif_selene.c, on the image built for no device (no measured TS sequence)
  * tsif_init: FW_ENOTIMPL and not one write, whatever the straps read
  * tsif_start: FW_ENOTIMPL and not one write
  * tsif_stop: FW_OK and not one write (nothing was started)
  (The measured sequence itself is test_ts_dt295.py's.)
  tsout_selene.c
  * A burst is the ring's packets back to back, 188 bytes each, the 4-byte
    tails dropped, split across the 512-byte boundary
  * The burst ends with the guard pair around the commit: 8, then 4 into
    the status, then 0x0F
  * A busy endpoint gives FW_EAGAIN and writes nothing to the FIFO
  * The second burst carries on where the first one split a packet
  stream.c, through a STREAM frame (frq1_dispatch), on the PIX-DT295 image
  * The first frame brings the path up once and sends the bursts asked
    for; the report says running, synced, the bursts and the bytes, and
    the FIFO got the packets
  * A second frame does not bring the path up again
  * STOP runs the stop once
  * On the image built for no device, STREAM answers FW_ENOTIMPL and
    writes nothing to the TS blocks
"""
import struct
import argparse
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_FW = os.path.dirname(_HERE)
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(_HERE, "fr"))
sys.path.insert(0, os.path.join(_FW, "host"))
import fremu                                                    # noqa: E402
import build as fwbuild                                         # noqa: E402
import test_ring_selene                                         # noqa: E402
import test_ts_dt295                                            # noqa: E402
from test_ring_selene import Capture                            # noqa: E402
from test_ring_selene import (ring_bytes, LENGTH,               # noqa: E402
                              RECORD, PACKET)

# Both TS blocks and the capture block, not only the capture block.
SELENE_BANDS = ((0x01900000, 0x01903000),)


class Run(test_ring_selene.Run):
    def __init__(self, img, sym, hw, ring=None, budget=8000000):
        saved = test_ring_selene.HW_BANDS
        test_ring_selene.HW_BANDS = SELENE_BANDS
        try:
            test_ring_selene.Run.__init__(self, img, sym, hw, ring=ring,
                                          budget=budget)
        finally:
            test_ring_selene.HW_BANDS = saved

FW_OK, FW_ENOTIMPL, FW_ETIMEOUT, FW_EAGAIN = 0, -1, -3, -5

# Register addresses (src/shared/regs_selene.h, mb86m01.h)
TSB0_000, TSB0_004 = 0x01900000, 0x01900004
TSB1_000, TSB1_004 = 0x01901000, 0x01901004
CAP_MODE, CAP_R010, CAP_BASE, CAP_LEN = (0x01902000, 0x01902010,
                                         0x01902020, 0x01902024)
CAP_WRITE, CAP_READ = 0x01902028, 0x0190202C
STRAPS = 0x00F10004
GUARD, EP_STATUS, EP_FIFO = 0x00FD814C, 0x00FDBD0C, 0x00FDBD84
M_RING_PHYS, M_RING_LEN = 0x02000000, 0x003FFFC0
M_STOP = 0x04000000
TS_BLOCKS = (0x00F10118, 0x01600000, 0x01600004, 0x01900000, 0x01900004,
             0x01900404, 0x01901000, 0x01901004)


class Producer(Capture):
    """The capture block as the hardware runs it: the write pointer is the
    block's, so a write to it does not move it; and 0x01902010 reads 0 once
    the stop has been asked for."""

    def __call__(self, a, width, v):
        if v is None:
            if a == CAP_R010 and self.regs.get(a) == M_STOP:
                return 0
            return self.regs.get(a, 0)
        self.writes.append((a, v & 0xFFFFFFFF))
        if a != CAP_WRITE:
            self.regs[a] = v & 0xFFFFFFFF
        return None


class Block:
    """The registers both files touch, as plain memory with a write log."""

    def __init__(self, straps=0, ready=True, capture=None):
        self.regs = {STRAPS: straps, EP_STATUS: 4 if ready else 0}
        self.log = []
        self.capture = capture          # test_ring_selene.Capture, or None

    def __call__(self, a, width, v):
        if self.capture is not None and 0x01902000 <= a < 0x01903000:
            return self.capture(a, width, v)
        if v is None:
            if a == STRAPS and width == 4:
                return self.regs[STRAPS] << 16
            return self.regs.get(a, 0)
        v &= (1 << (8 * width)) - 1
        self.regs[a] = v
        self.log.append((a, v, width))
        return None


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args()

    m = fwbuild.load()
    code, elf, tool = fwbuild.compile_all(m)
    img = fwbuild.loaded(m, code, tool)
    sym = fwbuild.symbols(elf, tool)
    fails = []

    def ok(cond, label, detail=""):
        print(f"  [{'ok ' if cond else 'NG '}] {label:<46}{detail}")
        if not cond:
            fails.append(label)

    def show(got, want):
        if a.verbose or got != want:
            print("        got  " + " ".join(f"{x:08x}={v:x}/{w}"
                                          for x, v, w in got))
            print("        want " + " ".join(f"{x:08x}={v:x}/{w}"
                                          for x, v, w in want))

    print("\n-- tsif_selene.c (no measured TS sequence) --")
    for straps in (0, 0x100, 0x1C0, 0x8A01):
        hw = Block(straps=straps)
        rv, _, _ = Run(img, sym, hw).call("tsif_init")
        ok(rv == FW_ENOTIMPL and not hw.log,
           f"init, strap {straps:#06x}: FW_ENOTIMPL, no write",
           f"rv {rv}, {len(hw.log)} writes")
    hw = Block()
    r = Run(img, sym, hw)
    rv, _, _ = r.call("tsif_start")
    ok(rv == FW_ENOTIMPL and not hw.log, "start: FW_ENOTIMPL, no write",
       f"rv {rv}, {len(hw.log)} writes")
    rv, _, _ = r.call("tsif_stop")
    ok(rv == FW_OK and not hw.log, "stop: FW_OK, no write",
       f"rv {rv}, {len(hw.log)} writes")

    print("\n-- tsout_selene.c --")
    nrec = 12
    ring = ring_bytes(LENGTH, 0, n_records=nrec)
    packets = b"".join(ring[i * RECORD:i * RECORD + PACKET]
                       for i in range(nrec))

    def burst_run(ready=True):
        cap = Capture(wp=nrec * RECORD)
        hw = Block(ready=ready, capture=cap)
        r = Run(img, sym, hw, ring=ring)
        for name in ("ring_init", "ring_sync", "tsout_init"):
            rv, _, _ = r.call(name)
            if rv != FW_OK:
                return r, hw, f"{name} {rv}"
        return r, hw, None

    def fifo_bytes(log):
        return b"".join(v.to_bytes(4, "big") for x, v, w in log
                        if x == EP_FIFO)

    r, hw, err = burst_run(ready=False)
    rv = r.call("tsout_push", [0])[0] if not err else err
    ok(rv == FW_EAGAIN and not fifo_bytes(hw.log),
       "busy endpoint: FW_EAGAIN, FIFO untouched", f"{rv}")

    r, hw, err = burst_run()
    if err:
        ok(False, "ring set up", err)
    else:
        sent = b""
        for k in range(2):
            hw.log.clear()
            rv, _, _ = r.call("tsout_push", [0])
            got = fifo_bytes(hw.log)
            want = packets[512 * k:512 * (k + 1)]
            tail = [(x, v) for x, v, w in hw.log if x != EP_FIFO]
            ok(rv == FW_OK and got == want,
               f"burst {k + 1}: 512 B, packets back to back",
               f"rv {rv}, {len(got)} B")
            ok(tail == [(GUARD, 8), (EP_STATUS, 4), (GUARD, 0x0F)],
               f"burst {k + 1}: guard 8, commit 4, guard 0x0F",
               " ".join(f"{x:08x}={v:x}" for x, v in tail))
            sent += got
        ok(sent == packets[:1024], "two bursts carry on across the split",
           f"{len(sent)} B")

    # Where the first record starts decides how ring_to_fifo loads: 0 is
    # word aligned, 186 is what a PIX-DT295 gave (2 mod 4: two words and a
    # shift), 1 takes the byte path.  And a ring that holds less than a
    # burst sends nothing.
    for shift in (186, 1):
        ring_k = ring_bytes(LENGTH, shift, n_records=nrec)
        pk = b"".join(ring_k[shift + i * RECORD:shift + i * RECORD + PACKET]
                      for i in range(nrec))
        cap = Capture(wp=shift + nrec * RECORD)
        hw = Block(capture=cap)
        r = Run(img, sym, hw, ring=ring_k)
        for name in ("ring_init", "ring_sync", "tsout_init"):
            r.call(name)
        sent = b""
        for k in range(2):
            hw.log.clear()
            rv, _, _ = r.call("tsout_push", [0])
            sent += fifo_bytes(hw.log)
        ok(rv == FW_OK and sent == pk[:1024],
           f"first record at {shift}: two bursts, the same bytes",
           f"rv {rv}, {len(sent)} B")

    cap = Capture(wp=2 * RECORD)
    hw = Block(capture=cap)
    r = Run(img, sym, hw, ring=ring_bytes(LENGTH, 0, n_records=2))
    for name in ("ring_init", "ring_sync", "tsout_init"):
        r.call(name)
    rv, _, _ = r.call("tsout_push", [0])
    ok(rv == FW_EAGAIN and not fifo_bytes(hw.log),
       "under a burst in the ring: FW_EAGAIN, FIFO untouched", f"rv {rv}")

    print("\n-- stream.c through a STREAM frame (PIX-DT295 image) --")
    img_d, sym_d = test_ts_dt295.image(m, tool, "dt295")
    nrec2 = 20
    big = ring_bytes(M_RING_LEN, 0, n_records=nrec2)
    pk2 = b"".join(big[i * RECORD:i * RECORD + PACKET] for i in range(nrec2))
    FRAME = 0x000CA000

    def stream_rig(img_x, sym_x):
        cap = Producer(base=0, length=0, wp=nrec2 * RECORD)
        hw = Block(straps=0x8A01, capture=cap)
        r = Run(img_x, sym_x, hw)
        r.e.write(M_RING_PHYS, big)
        return r, hw, cap

    def frame(r, cmd, arg0=0):
        f = bytearray(64)
        struct.pack_into(">6I", f, 0, 0x46525131, 1, cmd, 0, arg0, 0)
        r.e.write(FRAME, bytes(f))
        r.call("frq1_dispatch", [FRAME])
        return bytes(r.e.ld(FRAME + i, 1) for i in range(64))

    r, hw, cap = stream_rig(img_d, sym_d)
    resp = frame(r, 0x30, 2)
    st = struct.unpack_from(">i", resp, 12)[0]
    d = resp[24:]
    nb, nby = struct.unpack_from(">II", d, 4)
    modes = [w for w in cap.writes if w == (CAP_R010, 0x02000000)]
    ok(st == 1 and d[0] == 1 and d[1] & 3 == 3 and nb == 2 and nby == 1024,
       "first STREAM: running, synced, 2 bursts, 1024 B",
       f"status {st} flags {d[1]:#x} bursts {nb} bytes {nby}")
    ok(fifo_bytes(hw.log) == pk2[:1024], "  the FIFO got the packets",
       f"{len(fifo_bytes(hw.log))} B")
    ok(len(modes) == 1, "  the path came up once",
       f"{len(modes)} run writes")
    hw.log.clear()
    cap.writes.clear()
    resp = frame(r, 0x30, 1)
    d = resp[24:]
    again = [w for w in hw.log if w[0] in TS_BLOCKS] + \
        [w for w in cap.writes if w == (CAP_R010, 0x02000000)]
    ok(struct.unpack_from(">I", d, 4)[0] == 1 and not again,
       "second STREAM: 1 burst, no bring-up again",
       f"{len(again)} bring-up writes")
    ok(fifo_bytes(hw.log) == pk2[1024:1536], "  carries on from the first",
       "")
    hw.log.clear()
    cap.writes.clear()
    frame(r, 0x12)
    stops = [w for w in cap.writes if w == (CAP_R010, M_STOP)]
    ok(len(stops) == 1, "STOP: the stop asked for once",
       f"{len(cap.writes)} capture writes")

    r, hw, cap = stream_rig(img, sym)
    resp = frame(r, 0x30, 2)
    st = struct.unpack_from(">i", resp, 12)[0]
    touched = [w for w in hw.log if w[0] in TS_BLOCKS] + cap.writes
    ok(st == FW_ENOTIMPL and not touched,
       "no measured sequence: STREAM is FW_ENOTIMPL, no write",
       f"status {st}, {len(touched)} writes")

    print(f"\ntest_ts_selene: {'PASS' if not fails else f'FAIL ({len(fails)})'}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
