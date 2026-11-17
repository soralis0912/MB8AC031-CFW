#!/usr/bin/env python3
"""Check that the card I/F (src/main/card.c) follows the procedure in docs/card.md.

With no device, we build in Python a card block that behaves exactly as
docs/card.md describes, and call the real firmware one frame at a time.
.bss (stage, receive buffer) persists across frames, so one emulator is
created and reused.

This only shows that the code writes and reads in the documented order.
The real register behaviour (how fast bytes arrive, the unit of the waits)
can only be learned on the device.

What this checks

  * In the image built for no product (the `unknown` row), all 8 commands
    return FW_ENOTIMPL and never touch the card block
  * CARD_ON writes the 3 steps, one step per frame, in the docs' order and
    values, and reports the waits; bytes left in the receive FIFO are
    dropped before RST goes up
  * The ATR is collected even when it arrives piecemeal, and its length is
    derived correctly from TDi
  * CARD_PARAM writes the rate and the ISO 7816-3 waits from the ATR values (F372 D2 CWI5 BWI4)
  * CARD_PUT / SEND / RECV / GET do one S(IFS) round trip (measured messages)
  * Out-of-range args give FW_EINVAL, wrong order gives FW_EAGAIN, extra bytes give FW_EIO
  * CARD_OFF writes RST low, clock off, VCC=0
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
sys.path.insert(0, _HERE)

SP = 0x000C8000
FRAME = 0x000C4620

FW_OK, FW_ENOTIMPL, FW_EIO, FW_EINVAL, FW_EAGAIN = 0, -1, -2, -4, -5
NAMES = {0: "FW_OK", -1: "FW_ENOTIMPL", -2: "FW_EIO", -3: "FW_ETIMEOUT",
         -4: "FW_EINVAL", -5: "FW_EAGAIN"}

CMD = {"status": 0x50, "on": 0x51, "off": 0x52, "param": 0x53,
       "put": 0x54, "send": 0x55, "recv": 0x56, "get": 0x57}
FUNC = {k: "card_" + k for k in CMD}
CMD["health"] = 0x14             # health.c reads card_idle() for its flag
FUNC["health"] = "health_report"
HF_CARD_ON = 0x0004
CMD["suspend"] = 0x13            # pm.c powers the card down too
FUNC["suspend"] = "pm_suspend"


BASE = 0x00FEA000
REG = {"EN": 0x00, "PINS": 0x04, "BAUD": 0x08, "GO": 0x0C, "CTL": 0x10,
       "MODE": 0x14, "IRQEN": 0x18, "EVENT": 0x1C, "CWT": 0x20, "BWT": 0x24,
       "TX": 0x28, "RX": 0x2C, "VCC": 0x34, "TXLRC": 0x38, "RXLRC": 0x3C}
NAME = {BASE + o: n for n, o in REG.items()}

# ATR and S(IFS) round trip measured with a card on the GV-M2TV
ATR = bytes.fromhex("3BF01200FF9181B17C451F0399")
IFS_REQ = bytes.fromhex("00C101FE3E")
IFS_RES = bytes.fromhex("00E101FE1E")

# The power-up from docs/card.md (polarity 0): one list of writes per frame
POWER_UP = [[("PINS", 0), ("EN", 7), ("BAUD", 372 << 7), ("VCC", 1)],
            [("PINS", 2)],
            [("PINS", 6)]]
WAITS = [10, 1, 10]
POWER_DOWN = [("PINS", 2), ("PINS", 0), ("VCC", 0)]
STAGE_ATR, STAGE_READY, STAGE_SENT, STAGE_GOT = 4, 5, 6, 7


class CardBlock:
    """A card block shaped like the registers in docs/card.md, with a card inserted.

    * When RST rises with VCC=1, EN=7 and polarity 0, it emits the ATR
    * Bytes "arrive" `chunk` bytes at a time, each time MODE is read
    * The upper 16 bits of MODE are the count of arrived but unread bytes.
      Reading RX decrements it by 1
    * On GO=1 it emits the answer to the block sent
    * Bytes not read stay in the FIFO across a power-down (`stale`)
    """

    def __init__(self, chunk=4, extra=b"", stale=b""):
        self.r = {n: 0 for n in REG}
        self.writes = []           # (name, value)
        self.reads = 0
        self.pending = bytearray()  # yet to arrive
        self.ready = bytearray(stale)  # arrived, unread
        self.tx = bytearray()
        self.chunk = chunk
        self.extra = extra          # extra bytes emitted after the ATR
        self.underflow = 0
        self.sent = []

    def __call__(self, addr, width, val):
        n = NAME.get(addr)
        if n is None:
            return 0
        if val is None:
            self.reads += 1
            if n == "MODE":
                self.ready += self.pending[:self.chunk]
                del self.pending[:self.chunk]
                return (len(self.ready) << 16) | (self.r["MODE"] & 0xFFFF)
            if n == "RX":
                if not self.ready:
                    self.underflow += 1
                    return 0
                return self.ready.pop(0)
            if n == "EVENT":
                return 0x400
            return self.r[n]
        self.writes.append((n, val))
        old = self.r[n]
        self.r[n] = val
        if n == "PINS" and (val & 4) and not (old & 4):
            if self.r["VCC"] == 1 and self.r["EN"] == 7 and not (val & 8):
                self.pending += ATR + self.extra
        elif n == "TX":
            self.tx.append(val & 0xFF)
        elif n == "GO" and val == 1:
            self.sent.append(bytes(self.tx))
            if bytes(self.tx) == IFS_REQ:
                self.pending += IFS_RES
            self.tx = bytearray()
        return 0


class Fw:
    """Emulator with one real image loaded. .bss persists across calls.

    `where` is (frame, stack, emulator class): selene's by default, dione's
    from dione_image()."""

    def __init__(self, img, sym, hw, where=None):
        if where is None:
            where = (FRAME, SP, fremu.Emu)
        self.frame, self.sp, emu = where
        self.e = emu([(fremu.fold(a), b) for a, b in img], budget=2000000,
                     hw=hw)
        self.sym = sym
        self.seq = 0

    def cmd(self, name, arg0=0, arg1=0, data=b""):
        """Send one frame. -> (return value, 40 bytes of the frame data area)"""
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

    def identify(self):
        """Run IDENTIFY's handler. -> (board word, capability word)"""
        f = bytearray(64)
        struct.pack_into(">6I", f, 0, 0x46525131, 0, 0x10, 0, 0, 0)
        e = self.e
        e.write(self.frame, bytes(f))
        e.r[4] = self.frame
        e.r[15] = self.sp
        e.steps = 0
        why = e.run(fremu.fold(self.sym["dev_identify"]))
        if why not in (None, "ret", "return"):
            raise RuntimeError(f"dev_identify: {why}")
        d = bytes(e.ld(self.frame + 0x18 + i, 1) for i in range(16))
        return struct.unpack(">II", d[8:16])


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args()

    m = fwbuild.load()
    code, elf, tool = fwbuild.compile_all(m)
    img = fwbuild.loaded(m, code, tool)
    sym = fwbuild.symbols(elf, tool)
    fails = []

    def ok(cond, label, detail=""):
        print(f"  [{'ok ' if cond else 'NG '}] {label:<40}{detail}")
        if not cond:
            fails.append(label)

    def nm(rv):
        return NAMES.get(rv, str(rv))

    for f in FUNC.values():
        if f not in sym:
            print(f"symbol {f} not found")
            return 1

    # ------------------------------------------------ image for no product
    print("\nimage built for no product (the unknown row)")
    hw = CardBlock()
    fw = Fw(img, sym, hw)
    rvs = {k: fw.cmd(k, 0, 1, b"\0")[0] for k in CMD
           if k not in ("health", "suspend")}
    ok(all(v == FW_ENOTIMPL for v in rvs.values()), "all 8 commands FW_ENOTIMPL",
       " ".join(f"{k}={nm(v)}" for k, v in rvs.items() if v != FW_ENOTIMPL))
    ok(not hw.writes and hw.reads == 0, "does not touch the card block",
       f"writes {len(hw.writes)} reads {hw.reads}")
    rv, d = fw.cmd("suspend")
    ok(rv == FW_OK and d[0] == 1 and d[1] == 0 and not hw.writes,
       "SUSPEND: OK, nothing put down, no write", f"{nm(rv)} {d[:2].hex()}")
    board, caps = fw.identify()
    ok(board == 0 and caps == 0, "IDENTIFY: no product, no card",
       f"board {board:#x} caps {caps:#x}")

    mcode, melf = fwbuild.board_image(m, tool, "m2tv")
    mimg = fwbuild.loaded(m, mcode, tool, melf)
    msym = fwbuild.symbols(melf, tool)
    m2tv_checks(lambda hw: Fw(mimg, msym, hw), ok, nm, "selene image")

    # The card was measured on a GV-M2TV, which is dione: run the same
    # sequence on the dione image, with .bss and the stack where that image
    # puts them and the 0x000E band's store quirk modelled.
    dimg, dsym, where = dione_image(m, tool)
    emu_box = []

    def dione_fw(hw):
        fw = Fw(dimg, dsym, hw, where)
        emu_box.append(fw.e)
        return fw
    m2tv_checks(dione_fw, ok, nm, "dione image")
    narrow = sum(e.narrow_e for e in emu_box)
    ok(narrow == 0, "dione: no byte or half-word store in 0x000E",
       f"{narrow}")

    print()
    if fails:
        print(f"test_card: FAIL ({len(fails)})")
        return 1
    print("test_card: PASS")
    return 0


def dione_image(m, tool):
    """The GV-M2TV FRC1 image as loaded memory, and where it keeps things."""
    from test_i2c_dione import DioneEmu
    code, _boot, dsym, lay = fwbuild.frc1_variant(m, "dione", tool, "m2tv")
    bss_at = fwbuild.bss_base(lay)
    n = dsym["_bss_end"] - bss_at
    img = [(lay["RES_BASE"], code), (bss_at, b"\0" * (n + (-n % 4)))]
    where = (0x000C5000, lay["RES_STACK_TOP"] - 8, DioneEmu)
    return img, dsym, where


def m2tv_checks(make_fw, ok, nm, label):
    """The GV-M2TV sequence, on whichever image make_fw builds."""
    print(f"\nGV-M2TV (row m2tv, polarity 0), {label}")
    hw = CardBlock(chunk=4)
    fw = make_fw(hw)

    board, caps = fw.identify()
    want = 0x03 if "dione" in label else 0x02
    ok(board == 0x04BB053A and caps == want,
       "IDENTIFY: the m2tv row, card usable",
       f"board {board:#x} caps {caps:#x}")

    rv, d = fw.cmd("status")
    ok(rv == FW_OK and d[0] == 1 and d[1] == 0, "STATUS: version 1, stage 0",
       f"{nm(rv)} {d[:4].hex()}")

    rv, _ = fw.cmd("param", (372 << 16) | 2, (5 << 8) | 4)
    ok(rv == FW_EAGAIN, "PARAM before ATR is FW_EAGAIN", nm(rv))
    rv, _ = fw.cmd("send", 5)
    ok(rv == FW_EAGAIN, "SEND before ATR is FW_EAGAIN", nm(rv))
    rv, _ = fw.cmd("recv")
    ok(rv == FW_EAGAIN, "RECV before sending is FW_EAGAIN", nm(rv))
    hw.writes.clear()

    # Power-up: 3 steps, one per frame
    waits, stages, per = [], [], []
    rv = FW_EAGAIN
    for _ in range(len(POWER_UP)):
        n0 = len(hw.writes)
        rv, d = fw.cmd("on")
        waits.append(d[2])
        stages.append(d[1])
        per.append(hw.writes[n0:])
        if rv != FW_EAGAIN:
            break
    ok(per == POWER_UP, "writes the 3 steps in docs order and values (1 per frame)",
       "" if per == POWER_UP else str(per))
    ok(waits == WAITS, "waits: 10 ms after VCC, 1 after the clock, 10 last",
       "" if waits == WAITS else str(waits))
    ok(stages == [1, 2, STAGE_ATR], "stages 1, 2, then 4 (waiting for ATR)",
       str(stages))

    # Collect the ATR
    polls = 0
    while rv == FW_EAGAIN and polls < 20:
        rv, d = fw.cmd("on")
        polls += 1
    ok(rv == FW_OK, "collects the whole ATR, FW_OK", f"{nm(rv)} ({polls} polls)")
    ok(polls > 1, "collects a piecemeal ATR over several frames", f"{polls} frames")
    ok(d[1] == STAGE_READY and d[3] == len(ATR), "stage 5, ATR length 13", f"{d[1]} {d[3]}")
    ok(d[16:16 + len(ATR)] == ATR, "ATR at report +0x10", d[16:29].hex())
    ok(struct.unpack(">I", d[4:8])[0] == 0x400, "EVENT at report +4", d[4:8].hex())
    ok(struct.unpack(">I", d[8:12])[0] == 6, "PINS at report +8 (CLK|RST)",
       d[8:12].hex())
    ok(hw.underflow == 0, "never reads bytes that have not arrived", f"{hw.underflow} times")

    rv, d = fw.cmd("on")
    ok(rv == FW_OK and d[1] == STAGE_READY and not hw.pending, "ON after power-up writes nothing, OK",
       nm(rv))

    # PARAM
    hw.writes.clear()
    rv, _ = fw.cmd("param", (372 << 16) | 2, (5 << 8) | 4)
    w = dict(hw.writes)
    ok(rv == FW_OK, "PARAM F372 D2 CWI5 BWI4", nm(rv))
    ok(w.get("BAUD") == 186 << 7, "BAUD = F/D 186 << 7", hex(w.get("BAUD", 0)))
    ok(w.get("CWT") == 11 + 32, "CWT = 11 + 2^5 etu", str(w.get("CWT")))
    ok(w.get("BWT") == 11 + 16 * 960 * 2, "BWT = 11 + 2^4 * 960 * 372 * 2 / 372 etu",
       str(w.get("BWT")))
    ok("CTL" not in w, "CTL is not written")
    bad = [(0, 1 << 8), (372 << 16, 0), ((372 << 16) | 2, 16 << 8),
           ((1 << 16) | 2, 0), ((2048 << 16) | 1, 0), ((372 << 16) | 2, 10)]
    rvs = [fw.cmd("param", a0, a1)[0] for a0, a1 in bad]
    ok(all(v == FW_EINVAL for v in rvs), "bad F, D, CWI or BWI is FW_EINVAL",
       " ".join(nm(v) for v in rvs))

    # PUT
    rvs = [fw.cmd("put", 0, 0)[0], fw.cmd("put", 0, 41)[0],
           fw.cmd("put", 259, 1)[0], fw.cmd("put", 250, 10)[0]]
    ok(all(v == FW_EINVAL for v in rvs), "out-of-range PUT is FW_EINVAL",
       " ".join(nm(v) for v in rvs))
    rv, _ = fw.cmd("put", 0, len(IFS_REQ), IFS_REQ)
    ok(rv == FW_OK, "PUT places S(IFS)", nm(rv))

    # SEND
    rvs = [fw.cmd("send", 0)[0], fw.cmd("send", 256)[0]]
    ok(all(v == FW_EINVAL for v in rvs), "SEND 0 and 256 are FW_EINVAL",
       " ".join(nm(v) for v in rvs))
    hw.writes.clear()
    rv, _ = fw.cmd("send", len(IFS_REQ))
    order = [n for n, _ in hw.writes]
    want = ["TX"] * 5 + ["GO"]
    ok(rv == FW_OK and order == want, "SEND write order (as in docs)",
       "" if order == want else str(order))
    ok(hw.sent == [IFS_REQ], "bytes sent are exactly S(IFS)",
       " ".join(s.hex() for s in hw.sent))

    # RECV
    polls, rv = 0, FW_EAGAIN
    while rv == FW_EAGAIN and polls < 10:
        rv, d = fw.cmd("recv")
        polls += 1
    cnt, need = struct.unpack(">HH", d[2:6])
    ok(rv == FW_OK and d[1] == STAGE_GOT, "RECV collects 1 block, OK (stage 7)",
       f"{nm(rv)} stage {d[1]} ({polls} frames)")
    ok(cnt == 5 and need == 5, "received 5, block length 5", f"{cnt} {need}")
    ok(d[8:13] == IFS_RES, "S(IFS) answer at report +8", d[8:13].hex())
    rv, d2 = fw.cmd("recv")
    ok(rv == FW_OK and d2 == d, "RECV again gives the same answer", nm(rv))

    # GET
    rv, d = fw.cmd("get", 0)
    ok(rv == FW_OK and d[:5] == IFS_RES and d[5:] == bytes(35),
       "GET 0 gives the answer, zeros after it", d[:8].hex())
    rv, d = fw.cmd("get", 3)
    ok(rv == FW_OK and d[:2] == IFS_RES[3:], "GET 3 starts at the 4th byte", d[:4].hex())
    rv, _ = fw.cmd("get", 259)
    ok(rv == FW_EINVAL, "GET 259 is FW_EINVAL", nm(rv))
    ok(hw.underflow == 0, "never reads bytes that have not arrived (send/receive)",
       f"{hw.underflow} times")

    # HEALTH sees the powered card
    rv, d = fw.cmd("health")
    hf = (d[2] << 8) | d[3]
    ok(rv == FW_OK and hf & HF_CARD_ON, "HEALTH: card-on flag while powered",
       f"flags {hf:#06x}")

    # OFF
    hw.writes.clear()
    rv, d = fw.cmd("off")
    ok(rv == FW_OK and hw.writes == POWER_DOWN, "OFF writes RST low, clock off, VCC=0",
       "" if hw.writes == POWER_DOWN else str(hw.writes))
    ok(d[1] == 0 and d[3] == 0, "after OFF: stage 0, ATR length 0", f"{d[1]} {d[3]}")
    rv, d = fw.cmd("health")
    hf = (d[2] << 8) | d[3]
    ok(rv == FW_OK and not hf & HF_CARD_ON, "HEALTH: no card-on flag after OFF",
       f"flags {hf:#06x}")
    rv, _ = fw.cmd("param", (372 << 16) | 2, 5 << 8)
    ok(rv == FW_EAGAIN, "PARAM after OFF is FW_EAGAIN", nm(rv))

    # A second power-up also gets the ATR (.bss is reset correctly)
    for _ in range(40):
        rv, d = fw.cmd("on")
        if rv != FW_EAGAIN:
            break
    ok(rv == FW_OK and d[16:29] == ATR, "ATR is read again after re-insertion", nm(rv))

    # SUSPEND leaves a powered card alone: no write, still stage 5
    hw.writes.clear()
    rv, d = fw.cmd("suspend")
    ok(rv == FW_OK and d[0] == 1 and d[1] == 0 and not hw.writes,
       "SUSPEND: the powered card is left alone",
       f"{nm(rv)} {d[:2].hex()} writes {len(hw.writes)}")
    rv, d = fw.cmd("status")
    ok(d[1] == STAGE_READY, "after SUSPEND: still stage 5 (ready)", f"{d[1]}")

    # ------------------------------------------------ stale bytes
    print(f"\nbytes left in the FIFO from before, {label}")
    hw = CardBlock(chunk=16, stale=ATR[:7])
    fw = make_fw(hw)
    for _ in range(40):
        rv, d = fw.cmd("on")
        if rv != FW_EAGAIN:
            break
    ok(rv == FW_OK and d[16:29] == ATR, "stale bytes dropped before RST: the ATR alone",
       f"{nm(rv)} {d[16:29].hex()}")
    rv, _ = fw.cmd("put", 0, len(IFS_REQ), IFS_REQ)
    hw.ready += b"\x99\x98"
    rv, _ = fw.cmd("send", len(IFS_REQ))
    for _ in range(10):
        rv, d = fw.cmd("recv")
        if rv != FW_EAGAIN:
            break
    ok(rv == FW_OK and d[8:13] == IFS_RES, "stale bytes dropped before SEND: the answer alone",
       f"{nm(rv)} {d[8:13].hex()}")
    fw.cmd("off")

    # ------------------------------------------------ extra bytes
    print(f"\ncard that sends extra bytes after the ATR, {label}")
    hw = CardBlock(chunk=16, extra=b"\x55")
    fw = make_fw(hw)
    for _ in range(40):
        rv, d = fw.cmd("on")
        if rv != FW_EAGAIN:
            break
    ok(rv == FW_EIO, "FW_EIO if more than the ATR arrives", nm(rv))
    rv, _ = fw.cmd("off")
    ok(rv == FW_OK and hw.r["VCC"] == 0, "OFF can still power it down", nm(rv))


if __name__ == "__main__":
    sys.exit(main())
