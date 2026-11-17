"""Drive src/main/arm.c on a modelled ARM band, and check our ARM image's layout.

    python tests/test_arm.py

The model is what was measured on a GV-M2TV (docs/arm.md):

  * The ARM bands answer only once 0x00F10118 (high half) is 0xFFFF.
    Then CTL_EN reads 1 (bit0 = the core is stopped) and CTL_RST keeps the
    low 12 bits of a write
  * DTCM (window + 0x80000) takes writes and reads back
  * ITCM (window + 0) takes word stores but always reads 0 (write-only)

The selene model is what was measured on a PIX-DT295: a band
answers once bit2 of its +0x404 was set while the 0x00F10118 half-word was 0xFFFF, and
only while that half-word stays 0xFFFF (put back, the band reads 0).  The
rest of the band is modelled as on dione.

What this checks

  * selene: STATUS only reads; HOLD opens 0x00F10118, sets +0x404 bit2,
    leaves 0x00F10118 open, holds the core; never touches 0x00F10080..0x00F100FF
    or 0x00F1011C; LOAD and RUN as on dione
  * STATUS only reads; HOLD wakes the band and holds the core
  * LOAD refuses bad arguments and an unheld core, maps ARM addresses to the
    window (0 -> +0, 0x00100000 -> +0x80000), writes words, reads DTCM back
    and does not read ITCM back
  * RUN refuses an unheld core, clears ALIVE/FAULT, then CTL_RST = 0x0104
    and CTL_EN &= ~3; ARM-C goes through its own window (0x00C00000)
  * the mailbox is read through the window (ALIVE / FAULT flags)
  * the cancel words (ARM_CANCEL): WHAT then REQ, refused until the last
    one is acknowledged, 0 only reports, works when the counts are nonsense
  * the work queue (arm_ipc.c, src/shared/armipc.h): HOLD and RUN zero its
    header; CALL refuses until the ARM's 'IPC1', fills a slot and then the
    head, says FW_EAGAIN when 16 are waiting; REAP says FW_EAGAIN when
    empty, hands out a reply and moves the tail; nonsense counts -> FW_EIO
  * with arm-none-eabi: the ARM image starts with 8 branches at 0, and its
    stack and mailbox match src/shared/armcore.h
  * with a host C compiler: the ARM image's queue and services, built for
    the host with -DARM_HOSTED and driven by tests/armcore_host.c

What this cannot tell: whether the real core runs.
"""
import os
import shutil
import struct
import subprocess
import sys
import tempfile

_HERE = os.path.dirname(os.path.abspath(__file__))
_FW = os.path.dirname(_HERE)
sys.path.insert(0, os.path.join(_HERE, "fr"))
sys.path.insert(0, os.path.join(_FW, "host"))
sys.path.insert(0, _HERE)
import fremu                                                    # noqa: E402
import build as fwbuild                                         # noqa: E402
from test_card import Fw as CardFw, dione_image                 # noqa: E402

FW_OK, FW_ENOTIMPL, FW_EIO, FW_EINVAL, FW_EAGAIN = 0, -1, -2, -4, -5
FW_EBUSY = -7
NAMES = {0: "FW_OK", -1: "FW_ENOTIMPL", -2: "FW_EIO", -3: "FW_ETIMEOUT",
         -4: "FW_EINVAL", -5: "FW_EAGAIN", -7: "FW_EBUSY"}
FUNC = {"status": "arm_status", "hold": "arm_hold", "load": "arm_load",
        "run": "arm_run", "call": "arm_call", "reap": "arm_reap",
        "cancel": "arm_cancel", "put": "arm_put", "get": "arm_get",
        "card_status": "card_status", "card_on": "card_on",
        "card_off": "card_off"}
CMD = {"status": 0x60, "hold": 0x61, "load": 0x62, "run": 0x63,
       "call": 0x64, "reap": 0x65, "cancel": 0x66, "put": 0x67, "get": 0x68,
       "card_status": 0x50, "card_on": 0x51, "card_off": 0x52}
SVC_CARD, CARD_STATUS, CARD_ON, CARD_OFF = 2, 0, 1, 2
BUF_OFF = 0x8F600                              # ARM_BUF_OFF, armipc.h

GATE = 0x00F10118
WIN = {0: 0x00E00000, 1: 0x00C00000}
EN, RST, CTL58, CTL404 = 0x70000, 0x70004, 0x58000, 0x70404
DTCM_OFF, MBOX_OFF = 0x80000, 0x8FF00
ALIVE = 0x41524D31
IPC_OFF, IPC_MAGIC = 0x8FA00, 0x49504331     # src/shared/armipc.h
REQ_HEAD, REQ_TAIL, REP_HEAD, REP_TAIL = 0x04, 0x08, 0x0C, 0x10
CAN_REQ, CAN_WHAT, CAN_ACK, CAN_DONE, CAN_DROP = 0x18, 0x1C, 0x20, 0x24, 0x28
REQ, REP, SLOT, SLOTS = 0x040, 0x240, 0x20, 16


def header(name, hdr="armcore.h"):
    """#define values from src/shared/<hdr>"""
    with open(os.path.join(_FW, "src", "shared", hdr),
              encoding="utf-8") as f:
        for line in f:
            p = line.split()
            if len(p) >= 3 and p[0] == "#define" and p[1] == name:
                return p[2]
    raise KeyError(name)


class ArmBand:
    """The two ARM bands as measured from the host (see the top)."""

    def __init__(self, chip="dione"):
        self.chip = chip
        # dione: the cold value, high half of the word.  selene: the
        # half-word as the container boot leaves it.
        self.gate = 0x60100000 if chip == "dione" else 0xFC77
        self.woke = set()               # selene: cores whose +0x404 bit2 took
        self.reg = {}                   # (core, off) -> 16-bit value
        self.ram = {}                   # word address -> value
        self.writes = []                # (addr, width, value)
        self.reads = 0

    def awake(self, core=0):
        if self.chip == "selene":
            return core in self.woke and self.gate == 0xFFFF
        return (self.gate >> 16) == 0xFFFF

    def where(self, a):
        for core, w in WIN.items():
            if w <= a < w + 0x100000:
                return core, a - w
        return None, None

    def __call__(self, a, width, val):
        if val is not None:
            val &= (1 << (8 * width)) - 1     # a store keeps its width
            self.writes.append((a, width, val))
        else:
            self.reads += 1
        if a == GATE:
            if val is None:
                return self.gate
            self.gate = val
            return 0
        core, off = self.where(a)
        if core is None:
            return 0
        if (self.chip == "selene" and off == CTL404 and val is not None
                and width == 2):
            if self.gate == 0xFFFF and val & 4:
                self.woke.add(core)
            if core not in self.woke:
                return 0
        if not self.awake(core):
            return 0
        if off in (EN, RST, CTL58, CTL404):
            if val is None:
                v = self.reg.get((core, off), 1 if off == EN else 0)
                return v << 16 if width == 4 else v
            if width == 2:
                self.reg[(core, off)] = val & (0x0FFF if off == RST else 0xFFFF)
            return 0
        dtcm = DTCM_OFF <= off < DTCM_OFF + 0x10000
        itcm = off < 0x10000
        if val is None:
            return self.ram.get(a & ~3, 0) if dtcm else 0
        if (dtcm or itcm) and width == 4:
            self.ram[a] = val
        return 0

    def held(self, core=0):
        return (self.reg.get((core, EN), 1) & 1
                and self.reg.get((core, RST), 0) == 0x0FFF)


class DeafBand(ArmBand):
    """selene: a band whose +0x404 never takes (it did not wake)."""

    def __call__(self, a, width, val):
        if a in (WIN[0] + CTL404, WIN[1] + CTL404) and val is not None:
            self.writes.append((a, width, val))
            return 0
        return super().__call__(a, width, val)


class Fw(CardFw):
    def cmd(self, name, arg0=0, arg1=0, data=b""):
        self.seq += 1
        f = bytearray(64)
        struct.pack_into(">6I", f, 0, 0x46525131, self.seq, CMD[name], 0,
                         arg0, arg1)
        f[0x18:0x18 + len(data)] = data
        e = self.e
        e.write(self.frame, bytes(f))
        e.r[4] = self.frame
        e.r[15] = self.sp
        e.steps = 0
        why = e.run(fremu.fold(self.sym[FUNC[name]]))
        if why not in (None, "ret", "return"):
            raise RuntimeError(f"{FUNC[name]}: {why}")
        rv = e.r[4]
        if rv & 0x80000000:
            rv -= 1 << 32
        return rv, bytes(e.ld(self.frame + 0x18 + i, 1) for i in range(40))


def make(img, sym, hw, where=None):
    fw = Fw(img, sym, hw, where)
    fw.e.hw_bands = fw.e.hw_bands + ((0x00C00000, 0x00F00000),)
    return fw


def main():
    m = fwbuild.load()
    code, elf, tool = fwbuild.compile_all(m)
    img = fwbuild.loaded(m, code, tool)
    sym = fwbuild.symbols(elf, tool)
    fails = []

    def ok(cond, label, detail=""):
        print(f"  [{'ok ' if cond else 'NG '}] {label:<44}{detail}")
        if not cond:
            fails.append(label)

    def nm(rv):
        return NAMES.get(rv, str(rv))

    for f in FUNC.values():
        if f not in sym:
            print(f"symbol {f} not found")
            return 1

    print("\nselene image (model from a PIX-DT295)")
    hw = ArmBand("selene")
    fw = make(img, sym, hw)
    rv, d = fw.cmd("status", 0)
    ok(rv == FW_OK and not hw.writes and d[2] == 0, "STATUS reads only, asleep",
       f"{nm(rv)} flags {d[2]:#x}")
    rv, _ = fw.cmd("load", 0 | 4 << 8, 0x00100000, b"\x11\x22\x33\x44")
    ok(rv == FW_EAGAIN and not hw.writes, "LOAD before HOLD -> FW_EAGAIN", nm(rv))
    rv, d = fw.cmd("hold", 0)
    ok(rv == FW_OK and hw.held(), "HOLD -> FW_OK, core held", nm(rv))
    gw = [(w, v) for a, w, v in hw.writes if a == GATE]
    ok(gw == [(2, 0xFFFF)],
       "HOLD: gate half-word 0xFFFF, left open (DT295)", str(gw))
    order = [a for a, _w, _v in hw.writes]
    ok(order.index(WIN[0] + CTL404) == 1
       and hw.reg.get((0, CTL404), 0) & 4,
       "HOLD: +0x404 bit2 set while the gate is open", str([hex(a) for a in order[:3]]))
    ok(not any(0x00F10080 <= a < 0x00F10100 or a == 0x00F1011C
               for a, _w, _v in hw.writes),
       "HOLD: no semaphore, no 0x00F1011C")
    ok(d[2] & 3 == 3 and struct.unpack(">I", d[4:8])[0] == 0xFFFF,
       "HOLD report: awake, held, gate half-word", f"flags {d[2]:#x}")
    words = struct.pack(">2I", 0xE3A00001, 0xE12FFF1E)
    rv, d = fw.cmd("load", 0 | 8 << 8, 0x00100040, words)
    ok(rv == FW_OK and hw.ram.get(0x00E80040) == 0xE3A00001,
       "LOAD DTCM -> FW_OK", nm(rv))
    rv, d = fw.cmd("run", 0)
    ok(rv == FW_OK and hw.reg.get((0, RST)) == 0x0104
       and hw.reg.get((0, EN)) & 3 == 0, "RUN: CTL_RST 0x0104, CTL_EN &= ~3",
       nm(rv))
    rv, d = fw.cmd("hold", 1)
    ok(rv == FW_OK and hw.held(1) and 1 in hw.woke, "HOLD ARM-C wakes its own band",
       nm(rv))

    hw = DeafBand("selene")
    hw.gate = 0x9000
    fw = make(img, sym, hw)
    rv, _ = fw.cmd("hold", 0)
    gw = [v for a, _w, v in hw.writes if a == GATE]
    ok(rv == FW_EIO and gw == [0xFFFF],
       "HOLD on a band that stays asleep -> FW_EIO, gate left open",
       f"{nm(rv)} {[hex(v) for v in gw]}")

    print("\ndione image")
    dimg, dsym, where = dione_image(m, tool)

    hw = ArmBand()
    fw = make(dimg, dsym, hw, where)
    rv, d = fw.cmd("status", 0)
    ok(rv == FW_OK and not hw.writes, "STATUS reads only", nm(rv))
    ok(d[0] == 1 and d[2] == 0, "STATUS asleep: no flags", f"flags {d[2]:#x}")
    rv, _ = fw.cmd("status", 2)
    ok(rv == FW_EINVAL, "STATUS core 2 -> FW_EINVAL", nm(rv))
    rv, _ = fw.cmd("load", 0 | 4 << 8, 0x00100000, b"\x11\x22\x33\x44")
    ok(rv == FW_EAGAIN and not hw.writes, "LOAD before HOLD -> FW_EAGAIN", nm(rv))

    rv, d = fw.cmd("hold", 0)
    ok(rv == FW_OK, "HOLD -> FW_OK", nm(rv))
    ok(hw.gate >> 16 == 0xFFFF and hw.gate & 0xFFFF == 0,
       "HOLD: gate high half 0xFFFF, low half kept", f"{hw.gate:#010x}")
    ok(hw.held(), "HOLD: stopped (CTL_EN bit0) and CTL_RST held",
       f"EN {hw.reg.get((0, EN), 1):#06x} RST {hw.reg.get((0, RST), 0):#06x}")
    ok(d[2] & 3 == 3, "HOLD report: awake and held", f"flags {d[2]:#x}")

    for arg0, arg1, why in ((0 | 0 << 8, 0x00100000, "0 bytes"),
                            (0 | 6 << 8, 0x00100000, "6 bytes"),
                            (0 | 36 << 8, 0x00100000, "36 bytes"),
                            (0 | 4 << 8, 0x00100002, "misaligned"),
                            (0 | 4 << 8, 0x00010000, "past ITCM"),
                            (0 | 8 << 8, 0x0010FFFC, "runs past DTCM"),
                            (0 | 4 << 8, 0x00E80000, "main-side address")):
        rv, _ = fw.cmd("load", arg0, arg1, b"\0" * 8)
        ok(rv == FW_EINVAL, f"LOAD {why} -> FW_EINVAL", nm(rv))

    words = struct.pack(">4I", 0xE3A00001, 0xE12FFF1E, 0xDEADBEEF, 0x01020304)
    n0 = len(hw.writes)
    rv, d = fw.cmd("load", 0 | 16 << 8, 0x00100010, words)
    ok(rv == FW_OK, "LOAD 16 bytes into DTCM -> FW_OK", nm(rv))
    got = [hw.ram.get(0x00E80010 + 4 * i) for i in range(4)]
    ok(got == list(struct.unpack(">4I", words)),
       "LOAD: DTCM 0x00100010 lands at 0x00E80010", str([hex(x or 0) for x in got]))
    ok(all(w == 4 for _, w, _v in hw.writes[n0:]), "LOAD: word stores only")
    ok(d[2] == 4 and d[3] == 0xFF, "LOAD report: 4 words, no mismatch",
       f"{d[2]} {d[3]:#x}")

    rv, d = fw.cmd("load", 0 | 8 << 8, 0x00000000, words[:8])
    ok(rv == FW_OK and d[3] == 0xFF and hw.ram.get(0x00E00000) == 0xE3A00001,
       "LOAD into write-only ITCM -> FW_OK, not read back", nm(rv))

    hw.ram[0x00E00000 + MBOX_OFF] = 0x12345678
    hw.ram[0x00E00000 + MBOX_OFF + 8] = 0x46000001
    rv, d = fw.cmd("run", 0)
    ok(rv == FW_OK, "RUN after HOLD -> FW_OK", nm(rv))
    ok(hw.reg.get((0, RST)) == 0x0104 and hw.reg.get((0, EN)) & 3 == 0,
       "RUN: CTL_RST 0x0104, CTL_EN bits 1:0 clear",
       f"RST {hw.reg.get((0, RST), 0):#06x} EN {hw.reg.get((0, EN), 1):#06x}")
    ok(hw.ram.get(0x00E00000 + MBOX_OFF) == 0
       and hw.ram.get(0x00E00000 + MBOX_OFF + 8) == 0,
       "RUN clears ALIVE and FAULT first")
    ok(d[2] & 0x12 == 0x10, "RUN report: running, not held", f"flags {d[2]:#x}")
    rv, _ = fw.cmd("run", 0)
    ok(rv == FW_EAGAIN, "RUN while running -> FW_EAGAIN", nm(rv))
    rv, _ = fw.cmd("load", 0 | 4 << 8, 0x00100000, words[:4])
    ok(rv == FW_EAGAIN, "LOAD while running -> FW_EAGAIN", nm(rv))

    print("\n  work queue")
    q = 0x00E00000 + IPC_OFF

    def w(off):
        return hw.ram.get(q + off, 0)
    for i in range(16):
        hw.ram[q + i * 4] = 0x5A5A0000 + i      # stale words from before
    hw.reg[(0, EN)] = 1
    hw.reg[(0, RST)] = 0x0FFF
    fw.cmd("run", 0)
    ok(all(w(i * 4) == 0 for i in range(16)), "RUN zeroed the 16-word queue header")
    rv, _ = fw.cmd("call", 0 | 0x0001 << 8, 1)
    ok(rv == FW_EIO, "CALL before the ARM says 'IPC1' -> FW_EIO", nm(rv))
    hw.ram[q] = IPC_MAGIC                       # what ipc_init writes
    data = struct.pack(">5I", 0x55, 0xA0, 0xA1, 0xA2, 0xA3)
    rv, d = fw.cmd("call", 0 | 0x0102 << 8, 0x1234, data)
    seq, waiting = struct.unpack(">2I", d[4:12])
    ok(rv == FW_OK and seq == 1 and waiting == 1, "CALL -> FW_OK, seq 1",
       f"{nm(rv)} seq {seq} waiting {waiting}")
    slot = [w(REQ + i * 4) for i in range(8)]
    ok(slot[:7] == [1, 0x0102, 0x1234, 0x55, 0xA0, 0xA1, 0xA2]
       and slot[7] == 0xA3 and w(REQ_HEAD) == 1,
       "CALL: slot 0 = seq, call, arg0, arg1, data; head 1",
       " ".join(f"{x:x}" for x in slot))
    heads = [i for i, (a, _w, _v) in enumerate(hw.writes) if a == q + REQ_HEAD]
    slots = [i for i, (a, _w, _v) in enumerate(hw.writes)
             if q + REQ <= a < q + REQ + SLOT]
    ok(heads and slots and max(slots) < heads[-1], "CALL writes the head last")
    ok(all(wd == 4 for a, wd, _v in hw.writes if q <= a < q + 0x500),
       "queue: word stores only")
    for _ in range(SLOTS - 1):
        fw.cmd("call", 0 | 0x0000 << 8, 0)
    rv, _ = fw.cmd("call", 0, 0)
    ok(rv == FW_EAGAIN and w(REQ_HEAD) == SLOTS,
       "CALL with 16 waiting -> FW_EAGAIN", nm(rv))
    hw.ram[q + REQ_TAIL] = 3                    # the ARM took three
    rv, d = fw.cmd("call", 0, 0)
    ok(rv == FW_OK and struct.unpack(">I", d[4:8])[0] == 17
       and w(REQ + 0) == 17, "CALL after the ARM took some: slot 16 % 16 = 0",
       nm(rv))

    rv, _ = fw.cmd("reap", 0)
    ok(rv == FW_EAGAIN, "REAP with nothing -> FW_EAGAIN", nm(rv))
    for i, v in enumerate((1, 0x80000101, 0, 0xCAFE, 1, 2, 3, 4)):
        hw.ram[q + REP + i * 4] = v
    hw.ram[q + REP_HEAD] = 1
    rv, d = fw.cmd("reap", 0)
    got = struct.unpack(">8I", d[4:36])
    ok(rv == FW_OK and got == (1, 0x80000101, 0, 0xCAFE, 1, 2, 3, 4)
       and w(REP_TAIL) == 1, "REAP hands out the reply, tail 1",
       f"{nm(rv)} {[hex(x) for x in got]}")
    rv, _ = fw.cmd("reap", 0)
    ok(rv == FW_EAGAIN, "REAP again -> FW_EAGAIN", nm(rv))
    hw.ram[q + REP_HEAD] = 40
    rv, _ = fw.cmd("reap", 0)
    ok(rv == FW_EIO, "REAP with nonsense counts -> FW_EIO", nm(rv))

    print("\n  cancel words")
    rv, d = fw.cmd("cancel", 0, 7)
    req, ack = struct.unpack(">2I", d[4:12])
    ok(rv == FW_OK and w(CAN_WHAT) == 7 and w(CAN_REQ) == 1 and req == 1
       and ack == 0, "CANCEL 7 even with nonsense counts: WHAT 7, REQ 1",
       f"{nm(rv)} req {req} ack {ack}")
    order = [a for a, _w, _v in hw.writes if a in (q + CAN_WHAT, q + CAN_REQ)]
    ok(order[-2:] == [q + CAN_WHAT, q + CAN_REQ], "CANCEL writes WHAT, then REQ")
    hw.ram[q + REP_HEAD] = 1
    rv, _ = fw.cmd("cancel", 0, 0xFFFFFFFF)
    ok(rv == FW_EAGAIN and w(CAN_REQ) == 1,
       "CANCEL before the ARM acknowledged the last -> FW_EAGAIN", nm(rv))
    hw.ram[q + CAN_ACK] = 1
    hw.ram[q + CAN_DONE] = 1
    hw.ram[q + CAN_DROP] = 3
    n0 = len(hw.writes)
    rv, d = fw.cmd("cancel", 0, 0)
    req, ack, done, drop = struct.unpack(">4I", d[4:20])
    ok(rv == FW_OK and (req, ack, done, drop) == (1, 1, 1, 3)
       and not [a for a, _w, _v in hw.writes[n0:] if q <= a < q + 0x40],
       "CANCEL 0 only reports: acknowledged, 1 ended, 3 dropped",
       f"{req} {ack} {done} {drop}")
    rv, _ = fw.cmd("cancel", 0, 0xFFFFFFFF)
    ok(rv == FW_OK and w(CAN_WHAT) == 0xFFFFFFFF and w(CAN_REQ) == 2,
       "CANCEL all: WHAT 0xFFFFFFFF, REQ 2", nm(rv))

    print("\n  the buffer")
    tx, rx = 0x00E00000 + BUF_OFF, 0x00E00000 + BUF_OFF + 0x200
    rv, _ = fw.cmd("put", 0 | 6 << 8, 0, bytes([1, 2, 3, 4, 5, 6]))
    ok(rv == FW_OK and hw.ram.get(tx) == 0x01020304
       and hw.ram.get(tx + 4) == 0x05060000,
       "PUT 6 bytes: packed big-endian by value", nm(rv))
    hw.ram[tx + 8] = 0x11223344
    rv, _ = fw.cmd("put", 0 | 2 << 8, 8, bytes([0x77, 0x88]))
    ok(rv == FW_OK and hw.ram.get(tx + 8) == 0x77883344,
       "PUT 2 bytes into a word: its other bytes kept",
       f"{hw.ram.get(tx + 8, 0):#010x}")
    for arg0, arg1, why in ((0 | 0 << 8, 0, "0 bytes"), (0 | 41 << 8, 0, "41 bytes"),
                            (0 | 4 << 8, 2, "misaligned"),
                            (0 | 4 << 8, 0x200, "past the half"),
                            (0 | 8 << 8, 0x1FC, "runs past the half")):
        rv, _ = fw.cmd("put", arg0, arg1, b"\0" * 8)
        ok(rv == FW_EINVAL, f"PUT {why} -> FW_EINVAL", nm(rv))
    for i in range(0x80):
        hw.ram[rx + 4 * i] = 0xA0000000 + i
    rv, d = fw.cmd("get", 0, 0x1F0)
    got = struct.unpack(">10I", d)
    ok(rv == FW_OK and got[:4] == tuple(0xA0000000 + i for i in range(0x7C, 0x80))
       and got[4:] == (0,) * 6, "GET near the end: the last 16 bytes, then 0",
       nm(rv))
    rv, _ = fw.cmd("get", 0, 0x200)
    ok(rv == FW_EINVAL, "GET past the half -> FW_EINVAL", nm(rv))
    ok(all(wd == 4 for a, wd, _v in hw.writes if tx <= a < tx + 0x400),
       "buffer: word stores only")

    print("\n  the card has one owner")

    def take():                                 # the ARM took what was queued
        hw.ram[q + REQ_TAIL] = w(REQ_HEAD)

    def card(core, op):
        take()
        return fw.cmd("call", core | (SVC_CARD << 8 | op) << 8, 0)[0]
    take()
    rv = fw.cmd("call", 0 | (SVC_CARD << 8 | CARD_ON) << 8, 1)[0]
    head = w(REQ_HEAD)
    arg0 = w(REQ + ((head - 1) % SLOTS) * SLOT + 0x08)
    ok(rv == FW_OK and arg0 == 0,
       "ARM-E: card ON queued, polarity from the row (0), not the host's 1",
       f"{nm(rv)} arg0 {arg0}")
    rv, _ = fw.cmd("card_off", 0)
    ok(rv == FW_EBUSY, "then card.c's CARD_OFF -> FW_EBUSY", nm(rv))
    rv, _ = fw.cmd("card_status", 0)
    ok(rv == FW_OK, "card.c's CARD_STATUS still answers", nm(rv))
    hw.ram[0x00C00000 + IPC_OFF] = IPC_MAGIC
    rv = card(1, CARD_ON)
    ok(rv == FW_EBUSY, "ARM-C: card ON -> FW_EBUSY (ARM-E has it)", nm(rv))
    ok(card(0, CARD_OFF) == FW_OK, "ARM-E: card OFF queued")
    rv, _ = fw.cmd("card_off", 0)
    ok(rv == FW_OK, "then card.c's CARD_OFF -> FW_OK", nm(rv))
    rv, _ = fw.cmd("card_on", 0)
    ok(rv == FW_EAGAIN, "card.c powers up (step 1)", nm(rv))
    rv = card(0, CARD_ON)
    ok(rv == FW_EBUSY, "ARM-E: card ON while card.c has it -> FW_EBUSY", nm(rv))
    rv = card(0, CARD_STATUS)
    ok(rv == FW_OK, "ARM-E: card STATUS is fine", nm(rv))
    fw.cmd("card_off", 0)
    ok(card(0, CARD_ON) == FW_OK, "card.c off again: ARM-E takes it")

    rv, d = fw.cmd("hold", 0)
    ok(rv == FW_OK and hw.held(), "HOLD stops it again", nm(rv))
    rv, _ = fw.cmd("card_off", 0)
    ok(rv == FW_OK, "after HOLD the card is free: card.c's CARD_OFF -> FW_OK",
       nm(rv))
    rv, _ = fw.cmd("call", 0, 0)
    ok(rv == FW_EIO and hw.ram.get(q) == 0, "after HOLD: no 'IPC1', CALL -> FW_EIO",
       nm(rv))
    rv, _ = fw.cmd("cancel", 0, 1)
    ok(rv == FW_EIO, "after HOLD: CANCEL -> FW_EIO (HOLD is the stop now)", nm(rv))

    rv, d = fw.cmd("hold", 1)
    ok(rv == FW_OK and hw.held(1) and d[1] == 1, "HOLD ARM-C -> FW_OK, ARM-C held",
       nm(rv))
    rv, d = fw.cmd("load", 1 | 4 << 8, 0x00100020, words[:4])
    ok(rv == FW_OK and hw.ram.get(0x00C80020) == 0xE3A00001 and d[1] == 1,
       "LOAD ARM-C: DTCM lands at 0x00C80020", nm(rv))
    rv, _ = fw.cmd("status", 1)
    ok(rv == FW_OK, "STATUS ARM-C -> FW_OK", nm(rv))

    hw.ram[0x00E00000 + MBOX_OFF] = ALIVE
    hw.ram[0x00E00000 + MBOX_OFF + 4] = 1234
    hw.ram[0x00E00000 + MBOX_OFF + 8] = 0x46000004
    rv, d = fw.cmd("status", 0)
    alive, loops, fault = struct.unpack(">3I", d[16:28])
    ok(d[2] & 0x0C == 0x0C and alive == ALIVE and loops == 1234
       and fault == 0x46000004,
       "STATUS reads the mailbox through the window",
       f"flags {d[2]:#x} loops {loops}")

    narrow = fw.e.narrow_e
    ok(narrow == 0, "dione: no byte or half-word store in 0x000E", str(narrow))

    print("\nheader and linker script agree")
    with open(os.path.join(_FW, "src", "armcore", "link.ld"),
              encoding="utf-8") as f:
        ld = f.read()
    ok(header("ARM_DTCM") == "0x00100000u" and "ARM_DTCM      = 0x00100000;" in ld,
       "DTCM at 0x00100000 in both")
    with open(os.path.join(_FW, "src", "shared", "armcore.h"),
              encoding="utf-8") as f:
        hdr = f.read()
    ok("ARM_MBOX      = ARM_DTCM + ARM_DTCM_SIZE - 0x100;" in ld
       and "(ARM_DTCM + ARM_DTCM_SIZE - 0x100u)" in hdr,
       "mailbox is the last 256 bytes of DTCM in both")
    ok("ARM_IPC       = ARM_MBOX - 0x500;" in ld
       and "#define ARM_IPC             (ARM_MBOX - 0x500u)" in open(
           os.path.join(_FW, "src", "shared", "armipc.h"), encoding="utf-8").read()
       and header("ARM_IPC_BYTES", "armipc.h") == "0x500u",
       "work queue is the 0x500 bytes below the mailbox in both")
    ok(int(header("IPC_END", "armipc.h").rstrip("u"), 16) <= 0x500
       and int(header("IPC_REP", "armipc.h").rstrip("u"), 16)
       + SLOTS * SLOT == int(header("IPC_END", "armipc.h").rstrip("u"), 16),
       "work queue: two rings of 16 x 32 B fit")
    ok("ARM_BUF       = ARM_IPC - 0x400;" in ld
       and "ARM_STACK_TOP = ARM_BUF;" in ld
       and "#define ARM_BUF             (ARM_IPC - 0x400u)" in open(
           os.path.join(_FW, "src", "shared", "armipc.h"), encoding="utf-8").read(),
       "buffer is the 0x400 bytes below the queue in both, stack below it")

    print("\nARM image")
    try:
        atool = fwbuild.arm_toolchain()
    except fwbuild.NoToolchain as e:
        print(f"  [-- ] no arm-none-eabi, skipping: {e}")
        atool = None
    if atool:
        am = fwbuild._read_json(fwbuild.ARM_MANIFEST)
        acode, aelf, _ = fwbuild.compile_all(am, stem="armcore", tool=atool)
        asym = fwbuild.symbols(aelf, atool)
        vec = struct.unpack(">8I", acode[:32])
        ok(all(w >> 24 == 0xEA for w in vec), "8 vectors at 0, all `b`",
           " ".join(f"{w:08x}" for w in vec))
        ok(asym.get("arm_reset", 1 << 32) < 0x10000, "arm_reset in ITCM",
           f"{asym.get('arm_reset', 0):#x}")
        ok(asym.get("ARM_STACK_TOP") == 0x0010F600, "stack top below the buffer",
           f"{asym.get('ARM_STACK_TOP', 0):#x}")
        ok(len(acode) <= 0x10000, "fits ITCM", f"{len(acode)} bytes")

    print("\nARM side on the host (tests/armcore_host.c)")
    cc = shutil.which(os.environ.get("CC", "cc"))
    if not cc:
        print("  [-- ] no host C compiler, skipping")
    else:
        src = [os.path.join(_HERE, "armcore_host.c")] + [
            os.path.join(_FW, "src", "armcore", n)
            for n in ("armmain.c", "ipc.c", "svc_sys.c", "svc_watch.c",
                      "svc_card.c")]
        with tempfile.TemporaryDirectory() as tmp:
            exe = os.path.join(tmp, "armcore_host")
            c = subprocess.run([cc, "-std=c11", "-Wall", "-Wextra", "-Werror",
                                "-DARM_HOSTED",
                                "-I" + os.path.join(_FW, "src", "shared"),
                                "-I" + os.path.join(_FW, "src", "armcore"),
                                "-o", exe] + src,
                               capture_output=True, text=True)
            if c.returncode:
                ok(False, "armcore_host builds", c.stderr.strip()[:200])
            else:
                r = subprocess.run([exe], capture_output=True, text=True)
                for line in r.stdout.splitlines()[:-1]:
                    print("  " + line)
                ok(r.returncode == 0, "armcore_host: all checks")

    print()
    if fails:
        print(f"test_arm: FAIL ({len(fails)})")
        return 1
    print("test_arm: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
