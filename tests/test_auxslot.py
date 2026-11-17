"""Run the main FR's half of the aux-slot transport against a model of the
auxiliary core.

    python tests/test_auxslot.py

src/main/i2c/auxslot.c places our auxiliary program (carried in the main
image as build/auxcore_image.c), starts the core and passes I2C requests
through the slot at 0x017A3F00.  Here the auxiliary unit's band is a Python
object: plain RAM, plus a core that "boots" on the last reset write and
answers requests from a table.

No device row selects this transport: the PIX-DT295 reaches its
demodulators over 0x00FD8FE0 from the main FR (test_i2c.py).  So this test
links an image with a row of its own, ROW below: the DT295's row with
"i2c": "aux-slot".  It exists only in this test's build/.

What this checks

  * The DT295 image does not select this transport; the test row does, and
    the image carries the aux program
  * init releases the front-end reset (0x00F10018 bit1, then 0x00F10014
    bit1) and writes nothing else in 0x00F10xxx: the gates, the unit resets
    and 0x00F1010C stay untouched (the last one drops USB from the main FR)
  * The program lands at 0x017A2D00 byte for byte, the mode word is 'CPW1'
    and the reset vector is the program's local entry
  * The core is reset in the measured order, and init returns the
    program's own setup result
  * A transfer goes through the slot as words, sequence number last, and
    the bytes and result come back; a core that never answers gives
    FW_ETIMEOUT instead of a hang

What this cannot tell: whether the device starts the core this way from
the frc1 path with such an image (it did with the experiment monitor).
"""
import os
import struct
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_FW = os.path.dirname(_HERE)
sys.path.insert(0, os.path.join(_HERE, "fr"))
sys.path.insert(0, os.path.join(_FW, "host"))
sys.path.insert(0, _HERE)
import fremu                                                    # noqa: E402
import build as fwbuild                                         # noqa: E402
import genboards                                                # noqa: E402
import test_i2c                                                 # noqa: E402
from test_i2c import call, image, NAMES, SCRATCH                # noqa: E402

FW_OK, FW_ETIMEOUT = 0, -3
ROW = "auxslot"             # the test's own row (see above)
U17_LO, U17_HI = 0x01700000, 0x01800000
SLOT = 0x017A3F00
ALIVE, SETUP_RC, MODE = 0x017A3F70, 0x017A3F78, 0x017A3F90
VECTOR = 0x017FFFFC
CTL0, CTL4 = 0x01700000, 0x01700004
FE_DIR, FE_DAT = 0x00F10014, 0x00F10018
# The two writes the core needs to start (measured on the PIX-DT295: leaving
# out either, it never wrote 'AX1C').
RESET_ORDER = [(CTL4, 0), (CTL0, 0xFFFCFFFF)]


class AuxModel:
    """Unit 0x017's band and the main FR's 0x00F10xxx page.

    The aux core boots when 0x01700000 gets its last reset value, if the
    reset vector was written; then it publishes 'AX1C' and `setup_rc`.  A
    write to the slot's sequence word is served at once from `answers`
    ({(addr, tuple(wbytes)): (rc, rbytes)}).  The vector reads back 0, as
    on the device.
    """

    def __init__(self, answers, setup_rc=0, alive=True, answer=True):
        self.mem = {}
        self.half = {}
        self.f10 = []
        self.ctl = []
        self.answers = answers
        self.setup_rc = setup_rc & 0xFFFFFFFF
        self.alive = alive
        self.answer = answer
        self.vector = None
        self.slot_writes = []

    def word(self, a):
        return self.mem.get(a, 0)

    def __call__(self, a, width, v):
        if 0x00F10000 <= a < 0x00F11000:
            if v is None:
                h = self.half.get(a, 0)
                return h if width == 2 else h << 16
            self.f10.append((a, width, v & 0xFFFF if width == 2 else v))
            if width == 2:
                self.half[a] = v & 0xFFFF
            return None
        if not U17_LO <= a < U17_HI:
            return 0
        if v is None:
            if a == VECTOR:
                return 0
            if width == 4:
                return self.word(a)
            w = self.word(a & ~3)
            sh = (4 - width - (a & 3)) * 8
            return (w >> sh) & ((1 << (8 * width)) - 1)
        if width != 4:
            raise AssertionError(f"{width}-byte store to {a:#x} in aux RAM")
        v &= 0xFFFFFFFF
        if a == VECTOR:
            self.vector = v
            return None
        if a in (CTL0, CTL4):
            self.ctl.append((a, v))
            if self.ctl[-len(RESET_ORDER):] == RESET_ORDER and \
                    self.vector is not None and self.alive:
                self.mem[ALIVE] = 0x41583143
                self.mem[SETUP_RC] = self.setup_rc
            return None
        self.mem[a] = v
        if SLOT <= a < SLOT + 0x70:
            self.slot_writes.append(a - SLOT)
        if a == SLOT + 4 and v and self.answer:
            self.serve(v)
        return None

    def slot_bytes(self, off, n):
        out = b""
        for i in range(0, n, 4):
            out += struct.pack(">I", self.word(SLOT + off + i))
        return out[:n]

    def serve(self, seq):
        f = [self.word(SLOT + o) for o in range(0, 0x18, 4)]
        magic, _seq, op, addr, wn, rn = f
        assert magic == 0x41584932, "magic not written before seq"
        wb = tuple(self.slot_bytes(0x20, wn))
        rc, rb = self.answers.get((addr, wb), (-2, b""))
        rb = rb[:rn] if op == 2 else b""
        rb += bytes(-len(rb) % 4)
        for i in range(0, len(rb), 4):
            self.mem[SLOT + 0x48 + i] = struct.unpack(">I", rb[i:i + 4])[0]
        self.mem[SLOT + 0x18] = rc & 0xFFFFFFFF
        self.mem[SLOT + 0x1C] = seq


def main():
    m = fwbuild.load()
    code, _elf, tool = fwbuild.compile_all(m)
    row = dict(genboards.board(m["_tables"], "dt295"))
    row.update(id=ROW, i2c="aux-slot", _file="tests/test_auxslot.py")
    m["_tables"]["boards"].append(row)

    def build_image(bid):
        c, elf = fwbuild.board_image(m, tool, bid)
        return fwbuild.loaded(m, c, tool, elf), fwbuild.symbols(elf, tool)
    test_i2c._IMAGE["build"] = build_image
    test_i2c.HW_BANDS = ((U17_LO, U17_HI),)
    fails = []

    def ok(cond, label, detail=""):
        print(f"  [{'ok ' if cond else 'NG '}] {label:<44}{detail}")
        if not cond:
            fails.append(label)

    blob = open(os.path.join(_FW, fwbuild.AUX_BLOB), encoding="utf-8").read()
    aux = open(os.path.join(fwbuild.BUILD, "auxcore.bin"), "rb").read() \
        if os.path.exists(os.path.join(fwbuild.BUILD, "auxcore.bin")) else None
    img, sym = image("dt295")
    e = fremu.Emu([(fremu.fold(a), b) for a, b in img], budget=1)
    ok(e.ld(fremu.fold(sym["fw_board"]) + 4, 4) == sym["i2c_fujitsu_ops"],
       "the DT295 image selects fujitsu-fd8f, not this")
    img, sym = image(ROW)
    e = fremu.Emu([(fremu.fold(a), b) for a, b in img], budget=1)
    ok(e.ld(fremu.fold(sym["fw_board"]) + 4, 4) == sym["i2c_auxslot_ops"],
       "the test row selects aux-slot")
    n = e.ld(fremu.fold(sym["aux_image_len"]), 4)
    carried = bytes(e.ld(fremu.fold(sym["aux_image"]) + i, 1) for i in range(n))
    ok(n > 0 and n % 4 == 0 and "aux_image" in blob, "the image carries the aux program",
       f"{n} bytes")
    if aux is not None:
        ok(carried[:len(aux)] == aux, "  and it is build/auxcore.bin")

    print("\n-- init --")
    hw = AuxModel({})
    rv, _, _, why = call(ROW, "i2c_init", [], hw, budget=20000000)
    ok(rv == FW_OK, "init returns the program's setup result", NAMES.get(rv, rv))
    ok([(a, v) for a, _w, v in hw.f10] == [(FE_DAT, 0x0002), (FE_DIR, 0x0002)],
       "only the front-end reset bits in 0x00F10xxx",
       " ".join(f"{a:08x}={v:#x}" for a, _w, v in hw.f10))
    base = 0x017A2D00
    placed = b"".join(struct.pack(">I", hw.word(base + i)) for i in range(0, n, 4))
    ok(placed == carried, "the program is at 0x017A2D00, byte for byte")
    ok(hw.word(MODE) == 0x43505731, "mode word is 'CPW1'")
    ok(hw.vector == base - 0x01700000, "reset vector is the local entry",
       f"{hw.vector if hw.vector is None else hex(hw.vector)}")
    ok(hw.ctl == RESET_ORDER, "the core is started with the two measured writes",
       " ".join(f"{a & 0xF:x}={v:#x}" for a, v in hw.ctl))

    hw = AuxModel({}, setup_rc=-2)
    rv, _, _, _ = call(ROW, "i2c_init", [], hw, budget=20000000)
    ok(rv == -2, "the program's own setup failure comes back", NAMES.get(rv, rv))
    hw = AuxModel({}, alive=False)
    rv, _, _, why = call(ROW, "i2c_init", [], hw, budget=60000000)
    ok(rv == FW_ETIMEOUT, "a core that never starts: FW_ETIMEOUT",
       f"{NAMES.get(rv, rv)} ({why})")

    print("\n-- transfers --")
    hw = AuxModel({(0x10, (0x80,)): (0, b"\x21")})
    call(ROW, "i2c_init", [], hw, budget=20000000)
    rv, got, _, _ = call(ROW, "i2c_reg_read", [0x10, 0x80, SCRATCH], hw,
                         budget=20000000, out_len=1)
    ok(rv == FW_OK and got == b"\x21", "register read through the slot",
       f"{NAMES.get(rv, rv)} {got.hex()}")
    last = hw.slot_writes[-1] if hw.slot_writes else None
    ok(last == 4, "the sequence number is written last", f"last +{last}")
    ok(hw.word(SLOT + 8) == 2 and hw.word(SLOT + 0x10) == 1
       and hw.word(SLOT + 0x14) == 1, "op 2, wlen 1, rlen 1")

    hw = AuxModel({(0x11, (0x01, 0x02, 0x03, 0x04, 0x05)): (0, b"")})
    call(ROW, "i2c_init", [], hw, budget=20000000)
    rv, _, _, _ = call(ROW, "i2c_reg_write", [0x11, 0x01, 0x02], hw,
                       budget=20000000)
    ok(rv == -2 and hw.word(SLOT + 8) == 1 and hw.word(SLOT + 0x0C) == 0x11
       and hw.slot_bytes(0x20, 2) == b"\x01\x02",
       "a write: op 1, address and bytes, the peer's NACK back",
       NAMES.get(rv, rv))

    hw = AuxModel({}, answer=False)
    call(ROW, "i2c_init", [], hw, budget=20000000)
    rv, _, _, why = call(ROW, "i2c_reg_read", [0x10, 0x80, SCRATCH], hw,
                         budget=200000000, out_len=1)
    ok(rv == FW_ETIMEOUT, "no answer: FW_ETIMEOUT", f"{NAMES.get(rv, rv)} ({why})")

    print(f"\ntest_auxslot: {'PASS' if not fails else 'FAIL'}")
    for f in fails:
        print(f"   - {f}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
