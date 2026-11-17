#!/usr/bin/env python3
"""The dione TS path (src/main/ts_dione.c) against a model of the hardware.

There is no hardware here, so the capture block and the endpoint are built
in Python the way docs/ts.md describes them, and the real dione image runs
frame by frame on the FR emulator.  .bss carries the state between frames,
so one emulator is reused.

What this can show is that the code follows the steps: what it writes,
in what order, and that the bytes it pushes are the bytes the capture
wrote, in order, across the seams.  Timing and the real seam loss are for
the hardware.

What it checks

  * the image is the dione one (fw_is_dione) and STREAM goes to ts_dione.c
  * the wake-up list runs with the pin gate's bit2 set, and nothing in
    0x00D0xxxx is written before it (the model drops such writes, as the
    hardware does)
  * a band that does not come up is FW_EIO, not a silent empty stream
  * the TS setup and ring are written, then the kick
  * the stream that leaves EP 0x81 is exactly the stream the capture wrote,
    over two or more trips round the ring, with nothing lost: the reader
    writes the read offset back, the capture wraps at the limit's
    read-back value and never writes over what has not been sent
  * the write offset is never reset by the reader
  * only the burst that reaches the end of the ring may be short
  * nothing is read past the write offset: unwritten DRAM holds the
    emulator's fill pattern, so reading it would break the in-order match
  * a stalled capture makes the frame return (bit2 of the flags) instead
    of hanging
  * STOP stops the input, and a bad ring length is FW_EINVAL
"""
import argparse
import os
import struct
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_FW = os.path.dirname(_HERE)
sys.path.insert(0, os.path.join(_HERE, "fr"))
sys.path.insert(0, os.path.join(_FW, "host"))
import fremu                                                    # noqa: E402
import build as fwbuild                                         # noqa: E402

FRAME = 0x000C5000
SP = 0x000C5FF0
CMD_STOP, CMD_STREAM = 0x12, 0x30
# A frame that succeeded carries +1 in its status word (frq1.c answer()).
ST_OK, FW_EIO, FW_EINVAL = 1, -2, -4
NAMES = {1: "ok (+1)", -1: "FW_ENOTIMPL", -2: "FW_EIO", -4: "FW_EINVAL",
         -5: "FW_EAGAIN"}

PIN_GATE, PIN_ROUTE, TS_GATE = 0x00F10118, 0x00F10128, 0x00F12788
MEMC_CFG = 0x00D00004
TSIN_RUN = 0x00D01040
RING_MODE, RING_KICK, RING_BASE = 0x00D02000, 0x00D02010, 0x00D02020
RING_LIMIT, RING_WRITE, RING_READ = 0x00D02024, 0x00D02028, 0x00D0202C
EP_STATUS, EP_FIFO, EP_GUARD = 0x00FDBD0C, 0x00FDBD84, 0x00FD814C
EP1_STATUS, EP1_FIFO = 0x00FDBD24, 0x00FDBD90     # pair 1: EP 0x84
OUT_EP81, OUT_EP84 = 1, 2                          # STREAM data byte 0
RING_AT = 0x02000000
LAG = 0xC0                      # the write offset lags the data by this
HW_BANDS = ((0x00D00000, 0x00E00000),)


TEI_EVERY = 7                   # every 7th packet has TEI set


def ts_stream(n, lead=b""):
    """n bytes of TS: 188-byte packets with a counter in them, every
    TEI_EVERY-th with the transport error indicator set, after `lead`."""
    out = bytearray(lead)
    k = 0
    while len(out) < n:
        b1 = 0x81 if k % TEI_EVERY == TEI_EVERY - 1 else 0x01
        out += bytes([0x47, b1, 0x00, 0x10 | (k & 15)]) + \
            struct.pack(">I", k) * 46
        k += 1
    return bytes(out[:n])


def teis_in(got, lead=0):
    """Whole packets in `got` (after `lead` bytes) whose TEI is set."""
    return sum(1 for i in range(lead, len(got) - 1, 188)
               if got[i] == 0x47 and got[i + 1] & 0x80)


class Hw:
    """The 0x00D0 band, the pin words, the capture and EP 0x81.

    * 0x00D0xxxx takes writes only once the pin gate's bit2 is set
    * after the kick, every read of the write offset lets the capture put
      `step` more bytes into the ring; the offset reads LAG behind
    * the limit reads back in 64 B steps, and that is where the capture
      goes back to the base; it stops 0x40 short of the read offset
    * the FIFO is always ready; a commit (status <- 4 inside the guard
      pair) takes the words written since the last one as one packet
    """

    def __init__(self, stream, step=0x1000, band_works=True):
        self.r = {}
        self.order = []          # (addr, value) writes
        self.early_band = 0      # 0x00D0 writes before the gate bit
        self.stream = stream
        self.sent = 0            # stream bytes handed to the ring
        self.fill = 0            # bytes the capture has written, in all
        self.resets = 0          # writes of 0 to the write offset after the kick
        self.reads_set = 0       # writes to the read offset after the kick
        self.kicked = False
        self.step = step
        self.band_works = band_works
        self.stall_at = None
        self.words = []
        self.packets = []
        self.words1 = []
        self.packets1 = []       # EP 0x84
        self.guard = 0xF
        self.bad_commit = 0
        self.wp_reads = 0
        self.emu = None

    def gate_open(self):
        return bool((self.r.get(PIN_GATE, 0) >> 16) & 4)

    def wrap(self):
        return self.r.get(RING_LIMIT, 0) & ~0x3F

    def produce(self):
        if not self.kicked or not self.r.get(TSIN_RUN):
            return
        w = self.wrap()
        at = self.fill % w
        room = (self.r.get(RING_READ, 0) - 0x40 - at) % w
        n = min(self.step, room, len(self.stream) - self.sent)
        if self.stall_at is not None:
            n = min(n, self.stall_at - self.sent)
        while n > 0:
            k = min(n, w - at)
            self.emu.write(RING_AT + at, self.stream[self.sent:self.sent + k])
            self.fill += k
            self.sent += k
            n -= k
            at = self.fill % w

    def __call__(self, addr, width, val):
        if val is None:
            if addr == RING_WRITE:
                self.wp_reads += 1
                self.produce()
                return (max(0, self.fill - LAG) % self.wrap()) & ~0x3F
            if addr == RING_LIMIT:
                return self.r.get(addr, 0) & ~0x3F
            if addr in (EP_STATUS, EP1_STATUS):
                return 4
            if 0x00D00000 <= addr < 0x00E00000 and not self.band_works:
                return 0
            if addr == MEMC_CFG and self.r.get(addr) == 0x00043305:
                return 0x05043305        # the top byte reads back set (measured)
            return self.r.get(addr, 0)
        self.order.append((addr, val))
        if 0x00D00000 <= addr < 0x00E00000:
            if not self.gate_open() or not self.band_works:
                self.early_band += not self.gate_open()
                return 0
        if addr == RING_WRITE and val == 0 and self.kicked:
            self.resets += 1
        if addr == RING_READ and self.kicked:
            self.reads_set += 1
        if addr == RING_KICK:
            self.kicked = True
            self.fill = 0
        if addr == EP_FIFO:
            self.words.append(val)
            return 0
        if addr == EP1_FIFO:
            self.words1.append(val)
            return 0
        if addr == EP1_STATUS:
            if val == 4:
                if self.guard != 8:
                    self.bad_commit += 1
                self.packets1.append(b"".join(struct.pack(">I", w)
                                              for w in self.words1))
                self.words1 = []
            return 0
        if addr == EP_GUARD:
            self.guard = val
            return 0
        if addr == EP_STATUS:
            if val == 4:
                if self.guard != 8:
                    self.bad_commit += 1
                self.packets.append(b"".join(struct.pack(">I", w)
                                             for w in self.words))
                self.words = []
            return 0
        self.r[addr] = val
        return 0


class Fw:
    def __init__(self, bands, hw, symbols):
        self.e = fremu.Emu(bands, budget=60_000_000, hw=hw, hw_bands=HW_BANDS)
        hw.emu = self.e
        self.hw = hw
        self.sym = symbols
        self.seq = 0

    def frame(self, cmd, arg0=0, arg1=0, entry="frq1_dispatch", data=b""):
        self.seq += 1
        f = struct.pack(">6I", 0x46525131, self.seq, cmd, 0, arg0, arg1)
        e = self.e
        e.write(FRAME, f + data + bytes(40 - len(data)))
        e.r[4] = FRAME
        e.r[15] = SP
        e.steps = 0
        why = e.run(self.sym[entry])
        if why == "budget":
            raise RuntimeError("ran out of steps")
        st = struct.unpack(">i", bytes(e.ld(FRAME + 12 + i, 1)
                                       for i in range(4)))[0]
        data = bytes(e.ld(FRAME + 0x18 + i, 1) for i in range(40))
        return st, data


def report(d):
    keys = ("burst", "bytes", "write", "rc", "laps", "read", "teis", "peak")
    v = struct.unpack(">8I", d[4:36])
    # on FW_EIO, the TEI word carries the value that read back wrong
    return dict(zip(keys, v), flags=d[1], ver=d[0], fulls=d[2], misses=d[3],
                packets=struct.unpack(">I", d[36:40])[0], bad_val=v[6])


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args()

    m = fwbuild.load()
    code, elf, tool = fwbuild.compile_all(m)
    sel_sym = fwbuild.symbols(elf, tool)
    dcode, _boot, dsym, dlay = fwbuild.frc1_variant(m, "dione", tool, "m2tv")
    bss_at = fwbuild.bss_base(dlay)
    bss_len = dsym["_bss_end"] - bss_at
    bands = [(dlay["RES_BASE"], dcode),
             (bss_at, b"\0" * (bss_len + (-bss_len % 4)))]
    fails = []

    def ok(cond, label, detail=""):
        print(f"  [{'ok ' if cond else 'NG '}] {label:<46}{detail}")
        if not cond:
            fails.append(label)

    print("\nWhich image")
    for name, sym in (("selene", sel_sym), ("dione", dsym)):
        if "fw_is_dione" not in sym:
            print("  fw_is_dione is missing")
            return 1
    e = fremu.Emu([(fremu.fold(a), b) for a, b in fwbuild.loaded(m, code, tool)],
                  budget=1000)
    e.r[15] = 0x000C8000
    e.run(fremu.fold(sel_sym["fw_is_dione"]))
    ok(e.r[4] == 0, "selene image: fw_is_dione() = 0", str(e.r[4]))
    hw = Hw(b"")
    fw = Fw(bands, hw, dsym)
    fw.e.run(dsym["fw_is_dione"])
    ok(fw.e.r[4] == 1, "dione image: fw_is_dione() = 1", str(fw.e.r[4]))

    print("\nA band that does not come up")
    hw = Hw(ts_stream(0x1000), band_works=False)
    fw = Fw(bands, hw, dsym)
    st, d = fw.frame(CMD_STREAM, 16, 1)
    ok(st == FW_EIO, "STREAM says FW_EIO", NAMES.get(st, st))
    ok(not hw.packets, "nothing went out of EP 0x81", f"{len(hw.packets)}")
    r = report(d)
    ok(r["read"] == MEMC_CFG and r["bad_val"] & 0xFFFFFF != 0x043305,
       "the report names 0x00D00004 and what it read",
       f"{r['read']:#010x} {r['bad_val']:#010x}")

    print("\nA bad ring length")
    hw = Hw(b"")
    fw = Fw(bands, hw, dsym)
    st, _ = fw.frame(CMD_STREAM, 16, 17)
    ok(st == FW_EINVAL, "17 MiB is FW_EINVAL", NAMES.get(st, st))
    ok(not hw.order, "and nothing was written", f"{len(hw.order)} writes")

    print("\nStreaming over laps (1 MiB ring)")
    ring = 1 << 20
    total = 2 * ring + 0x12345
    src = ts_stream(total)
    hw = Hw(src, step=0x1000)
    fw = Fw(bands, hw, dsym)
    frames = 0
    reports = []
    while hw.sent < total and frames < 200:
        st, d = fw.frame(CMD_STREAM, 512, 1)
        frames += 1
        reports.append(report(d))
        if st != ST_OK:
            break
    r0 = reports[0]
    ok(st == ST_OK and r0["ver"] == 3 and r0["flags"] & 1,
       "STREAM succeeds, running", NAMES.get(st, st))
    ok(hw.early_band == 0, "no 0x00D0 write before the gate bit",
       f"{hw.early_band}")
    gate_w = [v for a, v in hw.order if a == PIN_GATE]
    ok(gate_w and (gate_w[-1] >> 16) & 4, "pin gate bit2 set",
       hex(gate_w[-1]) if gate_w else "")
    route_w = [v for a, v in hw.order if a == PIN_ROUTE]
    ok(route_w and route_w[0] >> 16 == 0x6100, "TS route 0x6100 (high half)",
       hex(route_w[0]) if route_w else "")
    ok(hw.r.get(MEMC_CFG) == 0x00043305, "0x00D00004 ends at 0x00043305",
       hex(hw.r.get(MEMC_CFG, 0)))
    ok(hw.r.get(RING_BASE) == RING_AT and hw.r.get(RING_LIMIT) == ring - 0x20
       and hw.r.get(RING_MODE) == 0x03020300, "ring base / limit / mode",
       f"{hw.r.get(RING_BASE, 0):#x} {hw.r.get(RING_LIMIT, 0):#x}")
    kick = [i for i, (a, _v) in enumerate(hw.order) if a == RING_KICK]
    run = [i for i, (a, v) in enumerate(hw.order) if a == TSIN_RUN and v]
    ok(kick and run and run[0] < kick[0], "input running, then the kick")
    ok(hw.bad_commit == 0, "every commit is inside the guard pair",
       f"{hw.bad_commit}")
    # drain what the capture already wrote but has not been pushed:
    # stall the capture and let frames run until they come back empty
    hw.stall_at = hw.sent
    for _ in range(20):
        st, d = fw.frame(CMD_STREAM, 4096, 1)
        r = report(d)
        if r["burst"] == 0:
            break
    ok(r["flags"] & 4 and r["burst"] == 0,
       "a stalled capture: the frame returns (flag bit2)",
       f"flags {r['flags']:#x}")
    got = b"".join(hw.packets)
    laps = r["laps"]
    sizes = [len(p) for p in hw.packets]
    short = [i for i, s in enumerate(sizes) if s != 512]
    ok(laps >= 2, "round the ring twice or more", f"{laps}")
    # what is still behind the lagging offset, or short of a whole burst,
    # has not gone out yet; everything before it has
    ok(src.startswith(got) and len(got) >= hw.sent - LAG - 0x200,
       "EP 0x81 carried the capture's bytes, in order, none lost",
       f"{len(got)} of {hw.sent} B")
    ok(len(short) <= laps, "short bursts only at the end of the ring",
       f"{len(short)} short, {laps} laps")
    peak = max(x["peak"] for x in reports)
    ok(0x200 <= peak < ring, "peak ring level is reported, below the ring",
       f"{peak:#x} of {ring:#x}")
    # the stream is whole packets from byte 0, so every packet start that
    # went out holds 0x47
    starts = (len(got) + 187) // 188
    ok(r["misses"] == 0 and r["packets"] == starts,
       "packet starts counted, none without 0x47",
       f"{r['packets']} of {starts}, {r['misses']} miss(es)")
    want = teis_in(got)
    ok(want and r["teis"] == want, "packets with TEI set are counted",
       f"{r['teis']} of {want}")
    ok(r["fulls"] <= laps, "times found full are reported",
       f"{max(x['fulls'] for x in reports)}")
    ok(hw.resets == 0, "the write offset is never reset", f"{hw.resets}")
    ok(hw.reads_set >= len(hw.packets), "the read offset follows every burst",
       f"{hw.reads_set} writes, {len(hw.packets)} bursts")

    print("\nSTOP")
    st, _ = fw.frame(CMD_STOP)
    ok(st == ST_OK and hw.r.get(TSIN_RUN) == 0, "STOP clears the input run",
       NAMES.get(st, st))

    print("\nThe endpoint byte (data byte 0)")
    ok(r0["flags"] & 0x30 == 0x10, "0 means EP 0x81 (flag bit4 only)",
       f"flags {r0['flags']:#x}")
    ok(not hw.packets1, "and nothing went to EP 0x84",
       f"{len(hw.packets1)} bursts")
    hw = Hw(b"")
    fw = Fw(bands, hw, dsym)
    st, _ = fw.frame(CMD_STREAM, 16, 1, data=bytes([4]))
    ok(st == FW_EINVAL and not hw.order,
       "an unknown endpoint bit is FW_EINVAL, nothing written",
       NAMES.get(st, st))
    for sel, name in ((OUT_EP84, "EP 0x84 alone"),
                      (OUT_EP81 | OUT_EP84, "both endpoints")):
        src = ts_stream(0x40000)
        hw = Hw(src, step=0x1000)
        fw = Fw(bands, hw, dsym)
        # frames until one runs out of input (flag bit2): that one has
        # also sent what was left, so it is the only frame that idles
        for _ in range(200):
            st, d = fw.frame(CMD_STREAM, 64, 1, data=bytes([sel]))
            if st != ST_OK or report(d)["flags"] & 4:
                break
        r = report(d)
        g0, g1 = b"".join(hw.packets), b"".join(hw.packets1)
        ok(st == ST_OK and (r["flags"] >> 4) & 3 == sel,
           f"{name}: flags say so", f"flags {r['flags']:#x}")
        ok(src.startswith(g1) and len(g1) >= len(src) - LAG - 0x200,
           f"{name}: EP 0x84 carried the bytes, in order",
           f"{len(g1)} of {len(src)} B")
        if sel & OUT_EP81:
            ok(g0 == g1 and len(hw.packets) == len(hw.packets1),
               f"{name}: EP 0x81 got the same bursts",
               f"{len(g0)} vs {len(g1)} B")
        else:
            ok(not g0, f"{name}: nothing on EP 0x81", f"{len(g0)} B")
        ok(hw.bad_commit == 0, f"{name}: commits inside the guard pair",
           f"{hw.bad_commit}")
        fw.frame(CMD_STOP)

    print("\nTEI when byte 1 is in the next word")
    # 3 bytes before the first packet: every start is the last byte of a
    # word, so every byte 1 is the first byte of the next
    src = ts_stream(0x20000, lead=b"\0\0\0")
    hw = Hw(src, step=0x1000)
    fw = Fw(bands, hw, dsym)
    for _ in range(200):
        st, d = fw.frame(CMD_STREAM, 64, 1)
        if st != ST_OK or report(d)["flags"] & 4:
            break
    r = report(d)
    got = b"".join(hw.packets)
    want = teis_in(got, 3)
    # the last start may have gone out without its byte 1
    ok(st == ST_OK and want and want <= r["teis"] <= want + 1,
       "packets with TEI set are counted", f"{r['teis']} of {want}")
    ok(r["misses"] == 0, "the 3 lead bytes are no miss", f"{r['misses']}")
    fw.frame(CMD_STOP)

    print()
    print("test_ts_dione: " + ("PASS" if not fails else f"FAIL ({len(fails)})"))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
