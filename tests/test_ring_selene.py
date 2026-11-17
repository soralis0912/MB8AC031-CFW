#!/usr/bin/env python3
"""Test the capture ring reader against a producer written in Python.

The ring's producer is hardware, not code, and the write position is a
register. So here too we model the hardware side and run the real
firmware against it.

The 192-byte records (a 188-byte packet each) in the ring at 0x0138EC00
were measured on a PIX-DT295. The rest (0x47 over 3 records,
the three-record threshold, the gates) are the rules this reader is written to,
with addresses in `src/shared/regs_selene.h`. This test checks that our
code follows them.

What this checks

  * Registers at 0 (= tsif_selene.c not there yet) give FW_EIO. No silent running
  * Two gates (length of four records or more, read position < length)
  * Fill = (write - read) mod length, including wraparound
  * No sync search below three records, 0x240 (FW_EAGAIN)
  * The record head is found at the smallest k in 0..191, checking 0x47 over 3 records
  * If not found, drop 1 record and return FW_EAGAIN (the next pass looks at other bytes)
  * The packet count matches the formula (n-1)*192 + 188 + k
  * Reads cross the end of the ring
  * Consuming advances the read position by whole records. If the length is
    not a multiple of the record size, sync is dropped on wraparound
"""
import argparse
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_FW = os.path.dirname(_HERE)
sys.path.insert(0, os.path.join(_HERE, "fr"))
sys.path.insert(0, os.path.join(_FW, "host"))
import fremu                                                    # noqa: E402
import build as fwbuild                                         # noqa: E402

SP = 0x000C8000
SCRATCH = 0x000C9000

R_MODE, R_BASE, R_LEN = 0x01902000, 0x01902020, 0x01902024
R_WRITE, R_READ = 0x01902028, 0x0190202C
HW_BANDS = ((0x01902000, 0x01903000),)

RECORD, PACKET, SYNC, SYNC_MIN = 192, 188, 0x47, 3 * 192
FW_OK, FW_ENOTIMPL, FW_EIO, FW_ETIMEOUT, FW_EINVAL, FW_EAGAIN = \
    0, -1, -2, -3, -4, -5
NAMES = {0: "FW_OK", -1: "FW_ENOTIMPL", -2: "FW_EIO", -3: "FW_ETIMEOUT",
         -4: "FW_EINVAL", -5: "FW_EAGAIN"}

BASE = 0x0138EC00               # measured on a PIX-DT295. From the CPU, | 0x80000000
LENGTH = 0x30000                # 192 KiB = exactly 1024 records


class Capture:
    """Capture block registers. The ring itself lives in a memory band."""

    def __init__(self, base=BASE, length=LENGTH, wp=0, rp=0):
        self.regs = {R_MODE: 0, R_BASE: base, R_LEN: length,
                     R_WRITE: wp, R_READ: rp}
        self.writes = []

    def __call__(self, a, width, v):
        if v is None:
            return self.regs.get(a, 0)
        self.regs[a] = v & 0xFFFFFFFF
        self.writes.append((a, v & 0xFFFFFFFF))
        return None


def ring_bytes(length, k, n_records=None, first=0):
    """Sequence of 192-byte records shifted by k bytes. Each record starts with 0x47."""
    out = bytearray(b"\x00" * length)
    i = k
    seq = first
    while i + RECORD <= length:
        out[i] = SYNC
        out[i + 1] = seq & 0xFF
        for j in range(2, PACKET):
            out[i + j] = (seq + j) & 0xFF
        i += RECORD
        seq += 1
        if n_records is not None and seq - first >= n_records:
            break
    return bytes(out)


class Run:
    """Keep one Emu around so functions can be called in turn on the same state.

    ring_selene.c remembers things in statics (base, length, sync), so calling init,
    sync and consume on separate Emus would be meaningless.
    """

    def __init__(self, img, sym, hw, ring=None, ring_base=BASE,
                 budget=8000000):
        bands = [(fremu.fold(a), b) for a, b in img]
        if ring is not None:
            bands.append((ring_base, ring))
        self.e = fremu.Emu(bands, budget=budget, hw=hw, hw_bands=HW_BANDS)
        self.sym, self.hw = sym, hw

    def call(self, name, args=(), out_len=0):
        e = self.e
        for i in range(16):
            e.r[i] = 0
        for i, v in enumerate(args):
            e.r[4 + i] = v
        e.r[15] = SP
        why = e.run(fremu.fold(self.sym[name]))
        rv = e.r[4]
        if rv & 0x80000000:
            rv -= 1 << 32
        got = bytes(e.ld(SCRATCH + i, 1) for i in range(out_len)) \
            if out_len else b""
        return rv, got, why


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-v", "--verbose", action="store_true")
    ap.parse_args()

    m = fwbuild.load()
    code, elf, tool = fwbuild.compile_all(m)
    # Take it as memory after loading (including the .bss zeros)
    img = fwbuild.loaded(m, code, tool)
    sym = fwbuild.symbols(elf, tool)
    fails = []

    def ok(cond, label, detail=""):
        print(f"  [{'ok ' if cond else 'NG '}] {label:<40}{detail}")
        if not cond:
            fails.append(label)

    print(f"\nCFW {len(code)} bytes")

    # --- gates ---
    print("\n-- register gates --")
    r = Run(img, sym, Capture(base=0, length=0))
    rv, _, _ = r.call("ring_init")
    ok(rv == FW_EIO, "FW_EIO if the block is not set up", NAMES.get(rv, rv))

    r = Run(img, sym, Capture(length=0x2FF))
    rv, _, _ = r.call("ring_init")
    ok(rv == FW_EIO, "FW_EIO if length <= 0x2FF (under four records)",
       NAMES.get(rv, rv))

    r = Run(img, sym, Capture(rp=LENGTH))
    rv, _, _ = r.call("ring_init")
    ok(rv == FW_EIO, "FW_EIO if read position >= length", NAMES.get(rv, rv))

    r = Run(img, sym, Capture(length=0x300))
    rv, _, _ = r.call("ring_init")
    ok(rv == FW_OK, "length 0x300 passes", NAMES.get(rv, rv))

    # --- fill ---
    print("\n-- fill --")
    for wp, rp, want in ((0x1000, 0x400, 0x0C00),
                         (0x100, 0x2F000, LENGTH - 0x2F000 + 0x100),
                         (0x500, 0x500, 0)):
        r = Run(img, sym, Capture(wp=wp, rp=rp))
        r.call("ring_init")
        used, _, _ = r.call("ring_used")
        ok(used == want, f"fill wp={wp:#x} rp={rp:#x}", f"{used:#x} / {want:#x}")

    r = Run(img, sym, Capture(wp=LENGTH + 4))
    r.call("ring_init")
    used, _, _ = r.call("ring_used")
    ok(used == 0, "0 if write position >= length (makes no sense)", f"{used}")

    # --- sync ---
    print("\n-- find the record head --")
    r = Run(img, sym, Capture(wp=SYNC_MIN - 1), ring=ring_bytes(LENGTH, 0))
    r.call("ring_init")
    rv, _, _ = r.call("ring_sync")
    ok(rv == FW_EAGAIN, "no search below three records", NAMES.get(rv, rv))

    for k in (0, 1, 3, 100, 191):
        cap = Capture(wp=LENGTH - RECORD)
        r = Run(img, sym, cap, ring=ring_bytes(LENGTH, k))
        r.call("ring_init")
        rv, _, _ = r.call("ring_sync")
        off, _, _ = r.call("ring_offset", [0])
        ok(rv == FW_OK and off == k, f"finds k={k}",
           f"{NAMES.get(rv, rv)} off={off}")

    cap = Capture(wp=LENGTH - RECORD)
    r = Run(img, sym, cap, ring=b"\x00" * LENGTH)
    r.call("ring_init")
    rv, _, _ = r.call("ring_sync")
    ok(rv == FW_EAGAIN, "FW_EAGAIN if there is no 0x47", NAMES.get(rv, rv))
    ok(cap.regs[R_READ] == RECORD, "drops 1 record if not found",
       f"read position {cap.regs[R_READ]}")

    # Sync is remembered (the second call does not search)
    cap = Capture(wp=LENGTH - RECORD)
    r = Run(img, sym, cap, ring=ring_bytes(LENGTH, 7))
    r.call("ring_init")
    r.call("ring_sync")
    before = len(cap.writes)
    rv, _, _ = r.call("ring_sync")
    ok(rv == FW_OK and len(cap.writes) == before,
       "second sync does not touch registers", NAMES.get(rv, rv))

    # --- packet count follows the formula ---
    print("\n-- packet count --")
    def avail_want(used, k):
        need = PACKET + k
        return 0 if used < need else 1 + (used - need) // RECORD

    for k in (0, 5, 191):
        for used in (SYNC_MIN, SYNC_MIN + 1, 0x1000, 0x2F000):
            cap = Capture(wp=used)
            r = Run(img, sym, cap, ring=ring_bytes(LENGTH, k))
            r.call("ring_init")
            if r.call("ring_sync")[0] != FW_OK:
                ok(False, f"cannot sync k={k} used={used:#x}")
                continue
            n, _, _ = r.call("ring_avail")
            w = avail_want(used, k)
            ok(n == w, f"count k={k} fill={used:#x}", f"{n} / {w}")
            # Check the formula's boundary directly
            if n:
                ok(used >= (n - 1) * RECORD + PACKET + k and
                   used < n * RECORD + PACKET + k,
                   f"  (n-1)*192+188+k boundary k={k} fill={used:#x}")

    # --- read ---
    print("\n-- read --")
    ring = ring_bytes(LENGTH, 3)
    cap = Capture(wp=0x2000)
    r = Run(img, sym, cap, ring=ring)
    r.call("ring_init")
    r.call("ring_sync")
    off, _, _ = r.call("ring_offset", [2])
    rv, got, _ = r.call("ring_read", [off, SCRATCH, PACKET], out_len=PACKET)
    want = ring[3 + 2 * RECORD: 3 + 2 * RECORD + PACKET]
    ok(rv == FW_OK and got == want, "the 3rd packet's 188 bytes match",
       f"{NAMES.get(rv, rv)} head {got[:2].hex()}")

    # Read across the end
    cap = Capture(wp=0x400, rp=LENGTH - 100)
    r = Run(img, sym, cap, ring=ring)
    r.call("ring_init")
    rv, got, _ = r.call("ring_read", [0, SCRATCH, 200], out_len=200)
    want = ring[LENGTH - 100:] + ring[:100]
    ok(rv == FW_OK and got == want, "can read across the end of the ring",
       NAMES.get(rv, rv))

    rv, _, _ = r.call("ring_read", [LENGTH, SCRATCH, 4])
    ok(rv == FW_EINVAL, "offset >= length is FW_EINVAL", NAMES.get(rv, rv))

    # --- consume ---
    print("\n-- consume --")
    cap = Capture(wp=0x2000)
    r = Run(img, sym, cap, ring=ring_bytes(LENGTH, 0))
    r.call("ring_init")
    r.call("ring_sync")
    n, _, _ = r.call("ring_avail")
    rv, _, _ = r.call("ring_consume", [3])
    ok(rv == FW_OK and cap.regs[R_READ] == 3 * RECORD,
       "3 packets advance the read position by 3 records",
       f"{NAMES.get(rv, rv)} {cap.regs[R_READ]}")
    rv, _, _ = r.call("ring_consume", [n + 1])
    ok(rv == FW_EINVAL, "cannot consume more than available", NAMES.get(rv, rv))
    rv, _, _ = r.call("ring_consume", [0])
    ok(rv == FW_EINVAL, "0 packets is FW_EINVAL", NAMES.get(rv, rv))

    # Cannot consume before sync
    cap = Capture(wp=0x2000)
    r = Run(img, sym, cap, ring=ring_bytes(LENGTH, 0))
    r.call("ring_init")
    rv, _, _ = r.call("ring_consume", [1])
    ok(rv == FW_EIO, "consume before sync is FW_EIO", NAMES.get(rv, rv))

    # If the length is not a multiple of the record size, sync is dropped on wraparound.
    # The producer lays records out counting from the read position (and keeps
    # going straight through the wrap), so after the wrap record heads drift off the 192 grid.
    odd = LENGTH + 64                   # not a multiple of 192
    rp0 = odd - 2 * RECORD
    ring = bytearray(b"\x00" * odd)
    for j in range(odd // RECORD + 2):
        ring[(rp0 + j * RECORD) % odd] = SYNC
    cap = Capture(length=odd, wp=0x2000, rp=rp0)
    r = Run(img, sym, cap, ring=bytes(ring), budget=20000000)
    rv, _, _ = r.call("ring_init")
    ok(rv == FW_OK, "a non-multiple length passes", NAMES.get(rv, rv))
    rv, _, _ = r.call("ring_sync")
    ok(rv == FW_OK, "syncs with a non-multiple length", NAMES.get(rv, rv))
    rv, _, _ = r.call("ring_consume", [3])
    ok(rv == FW_OK and cap.regs[R_READ] == (rp0 + 3 * RECORD) % odd,
       "read position advances across the wrap", f"{cap.regs[R_READ]}")
    n, _, _ = r.call("ring_avail")
    ok(n == 0, "drops sync on wraparound (search starts over)", f"{n}")
    rv, _, _ = r.call("ring_consume", [1])
    ok(rv == FW_EIO, "cannot consume after sync is dropped", NAMES.get(rv, rv))

    print("\ntest_ring_selene: " + ("PASS" if not fails else f"FAIL ({len(fails)})"))
    for f in fails:
        print("  - " + f)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
