"""Run the auxiliary unit image on top of the modelled I2C controller.

    python tests/test_aux.py

src/auxcore/ is a program for a different CPU (the 0x017 unit). It is needed
because 0x01684FE0 answers on that unit's bus, not the main FR's
(src/auxcore/README.md). Here we run it in the FR interpreter, with the
controller model from tests/test_i2c.py attached at base 0x01684FE0.

What this checks

  * Addresses are linked local (as seen from the auxiliary core)
  * The image runs from its entry and does not crash
  * It serves a request placed in the slot: bytes go out, come back, and the
    result and ACK are written, ACK last
  * It refuses requests it cannot serve, without truncating them
  * In the CPW1 mode ('CPW1' in the mode word; the route measured on a
    PIX-DT295 and started by i2c/auxslot.c, which no device row selects
    now -- the DT295 row uses 0x00FD8FE0), every access to the
    0x0168 band happens while 0x00F1010C holds 0x000B, the word is put
    back afterwards, and the I2C sub-block is brought up with 0x016840C0 = 0
    and bit4 of +0x10 / +0x14 (the model below answers only then, as the
    device did)

What this cannot tell: real bus timing, and how the peer chip answers.
"""
import json
import os
import struct
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_FW = os.path.dirname(_HERE)
sys.path.insert(0, os.path.join(_FW, "host"))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(_HERE, "fr"))

import build as fwbuild                   # noqa: E402
import fremu                              # noqa: E402
from test_i2c import Fujitsu, Slave, BASE_1684, HW_BANDS, NAMES   # noqa: E402

# Addresses are as seen from the auxiliary core (local) = main address - 0x01700000.
# src/auxcore/link.ld links local
AUX_UNIT_BASE = 0x01700000
with open(os.path.join(_FW, "config", "auxcore.json"), encoding="utf-8") as _f:
    AUX_MANIFEST = json.load(_f)
# base in auxcore.json is the address as seen from main
AUX_MAIN_BASE = int(AUX_MANIFEST["base"], 0)
AUX_BASE = AUX_MAIN_BASE - AUX_UNIT_BASE
SLOT = 0x017A3F00 - AUX_UNIT_BASE
RAM_LO, RAM_HI = 0x000A0000, 0x000A4000         # 16 KiB window (local)
MAGIC = 0x41584932                      # 'AXI2'
OP_WRITE, OP_WRITE_READ = 1, 2
FW_OK, FW_EINVAL = 0, -4

O_MAGIC, O_SEQ, O_OP, O_ADDR = 0x00, 0x04, 0x08, 0x0C
O_WLEN, O_RLEN, O_RESULT, O_ACK = 0x10, 0x14, 0x18, 0x1C
O_WDATA, O_RDATA = 0x20, 0x48
ALIVE, ALIVE_MAGIC = 0x017A3F70 - AUX_UNIT_BASE, 0x41583143   # 'AX1C'
LOOPS = 0x017A3F74 - AUX_UNIT_BASE
SETUP_RC = 0x017A3F78 - AUX_UNIT_BASE

BLOCK_CLK, BLOCK_RST, BLOCK_C0 = 0x01684010, 0x01684014, 0x016840C0


def image():
    """-> (flat bytes, AUX_STACK from link.ld or None)"""
    code, elf, tool = fwbuild.compile_all(dict(AUX_MANIFEST), stem="auxcore")
    return code, fwbuild.symbols(elf, tool).get("AUX_STACK")


def request(seq, op, addr, wdata=b"", rlen=0):
    s = bytearray(0x70)
    struct.pack_into(">6I", s, 0, MAGIC, seq, op, addr, len(wdata), rlen)
    s[O_WDATA:O_WDATA + len(wdata)] = wdata
    return bytes(s)


MODE, MODE_CPWIN = 0x017A3F90 - AUX_UNIT_BASE, 0x43505731     # 'CPW1'
CP_WORD, CP_COLD, CP_OPEN = 0x00F1010C, 0x0001, 0x000B
PIN_114 = 0x00F10114
BAND_LO, BAND_HI = 0x01680000, 0x01690000
BIT4 = 0x00000010


class CpWindow:
    """The DT295 band as measured, around a controller model.

    0x0168xxxx reads 0 and drops writes unless 0x00F1010C (a half-word)
    holds 0x000B.  The I2C registers inside answer only once 0x016840C0
    reads 0, bit4 of 0x01684010 is set and bit4 of 0x01684014 is clear.  Any
    band access with the window shut is recorded in `shut`.
    """

    def __init__(self, inner):
        self.inner = inner
        self.cp = CP_COLD
        self.pin = 0
        self.regs = {BLOCK_CLK: 0x00000004, BLOCK_RST: 0x0F3F001B,
                     BLOCK_C0: 0x00000003}
        self.shut = []
        self.cp_writes = []

    def _up(self):
        return self.regs[BLOCK_CLK] & BIT4 and \
            not self.regs[BLOCK_RST] & BIT4 and not self.regs[BLOCK_C0]

    def __call__(self, a, width, v):
        if a == CP_WORD:
            if v is None:
                return self.cp << 16 if width == 4 else self.cp
            self.cp = v & 0xFFFF
            self.cp_writes.append(self.cp)
            return None
        if a == PIN_114:
            if v is None:
                return self.pin
            self.pin = v & 0xFFFF
            return None
        if BAND_LO <= a < BAND_HI:
            if self.cp != CP_OPEN:
                self.shut.append((a, v))
                return 0 if v is None else None
            if a < BASE_1684:
                if v is None:
                    return self.regs.get(a, 0)
                self.regs[a] = v & 0xFFFFFFFF
                return None
            if not self._up():
                return 0 if v is None else None
        return self.inner(a, width, v)


def run(code, slot, hw, budget=4000000, mode=0):
    # Code, stack and the request slot all live in the same 16 KiB RAM window,
    # so the bands split the window into 3. They must not overlap: a zero band
    # written later would wipe the code
    assert RAM_LO <= AUX_BASE and AUX_BASE + len(code) < RAM_HI
    bands = [(RAM_LO, bytes(AUX_BASE - RAM_LO)),
             (AUX_BASE, code),
             (AUX_BASE + len(code), bytes(RAM_HI - AUX_BASE - len(code)))]
    e = fremu.Emu(bands, budget=budget, hw=hw, hw_bands=HW_BANDS)
    e.write(SLOT, slot)
    e.write(MODE, struct.pack(">I", mode))
    why = e.run(AUX_BASE)
    back = bytes(e.ld(SLOT + i, 1) for i in range(0x70))
    return back, e, why


def field(s, off):
    return struct.unpack_from(">I", s, off)[0]


def signed(s, off):
    return struct.unpack_from(">i", s, off)[0]


def main():
    ok = True
    try:
        code, want_stack = image()
    except fwbuild.NoToolchain as e:
        print(f"cannot build, so cannot test: {e}")
        return 1
    print(f"aux: {len(code)} bytes @ local {AUX_BASE:#010x}"
          f" (from main {AUX_MAIN_BASE:#010x})")

    print("\n  -- address view is local --")
    # The aux core runs its image at local addresses, so ours must be linked
    # local. If main-side addresses appear in the image, the core crashes as
    # soon as it runs
    first = int.from_bytes(code[0:2], "big")
    if first == 0x9F8F:                 # ldi:32 #imm,r15
        stack = int.from_bytes(code[2:6], "big")
    elif (first & 0xFF00) == 0x9B00 and (code[1] & 0x0F) == 0x0F:
        stack = ((code[1] >> 4) << 16) | int.from_bytes(code[2:4], "big")
    else:
        stack = None
    if stack is None:
        print(f"  [BAD] first instruction does not set the stack: {code[:6].hex(' ')}")
        ok = False
    elif stack >= AUX_UNIT_BASE:
        print(f"  [BAD] stack is a main-side address {stack:#010x}. "
              "Link it local")
        ok = False
    elif not RAM_LO <= stack <= RAM_HI:
        print(f"  [BAD] stack {stack:#010x} is outside the RAM window")
        ok = False
    elif stack != want_stack:
        # link.ld owns the value. A second copy in entry.S is how the stack
        # address once differed in three places
        print(f"  [BAD] stack {stack:#010x} is not AUX_STACK in link.ld "
              f"({want_stack if want_stack is None else hex(want_stack)})")
        ok = False
    elif not AUX_BASE + len(code) <= stack < SLOT:
        print(f"  [BAD] stack {stack:#010x} is not between the end of the "
              f"image {AUX_BASE + len(code):#010x} and the slot {SLOT:#010x}")
        ok = False
    else:
        print(f"  [ok ] stack is AUX_STACK from link.ld, local {stack:#010x}"
              f" (from main {stack + AUX_UNIT_BASE:#010x})")

    # Check that no main-side address (0x017xxxxx) leaked into the image.
    # Peripheral registers (0x01684xxx / 0x01700xxx) look the same from both sides, so skip them
    bad = []
    for i in range(0, len(code) - 3):
        w = int.from_bytes(code[i:i + 4], "big")
        if 0x017A0000 <= w < 0x017B0000 or 0x017E0000 <= w < 0x017F0000:
            bad.append((i, w))
    if bad:
        print(f"  [BAD] {len(bad)} main-side RAM/code addresses in the image: "
              + ", ".join(f"+{o:#x}={w:#010x}" for o, w in bad[:4]))
        ok = False
    else:
        print("  [ok ] no main-side RAM/code addresses in the image")

    print(f"  [ok ] auxcore.json base {AUX_MAIN_BASE:#010x} "
          f"= local {AUX_BASE:#010x} + {AUX_UNIT_BASE:#010x}")

    hw = Fujitsu({0x10: Slave({0x02: 0xAB, 0x03: 0xCD})}, base=BASE_1684,
                 gated=False)
    back, e, why = run(code, request(1, OP_WRITE_READ, 0x10, b"\x02", 2), hw)

    print("\n  -- alive marker (one word telling whether the core ran) --")
    live = [("'AX1C' is set", e.ld(ALIVE, 4) == ALIVE_MAGIC),
            ("loop count increases", e.ld(LOOPS, 4) > 1),
            ("fj_setup result is stored too", e.ld(SETUP_RC, 4) == 0)]
    for name, good in live:
        ok = ok and good
        print(f"  [{'ok ' if good else 'BAD'}] {name}")
    print(f"        alive={e.ld(ALIVE, 4):#010x} loops={e.ld(LOOPS, 4)} "
          f"setup={struct.unpack('>i', struct.pack('>I', e.ld(SETUP_RC, 4)))[0]}")

    print("\n  -- serves one request (write then read) --")
    checks = [
        ("ACK is the request seq", field(back, O_ACK) == 1),
        ("result is FW_OK", signed(back, O_RESULT) == FW_OK),
        ("the 2 bytes read are the peer's values", back[O_RDATA:O_RDATA + 2] == b"\xAB\xCD"),
        ("slot input fields untouched", field(back, O_SEQ) == 1
         and field(back, O_ADDR) == 0x10 and back[O_WDATA] == 0x02),
    ]
    for name, good in checks:
        ok = ok and good
        print(f"  [{'ok ' if good else 'BAD'}] {name}")
    print(f"        result={NAMES.get(signed(back, O_RESULT), signed(back, O_RESULT))} "
          f"ack={field(back, O_ACK)} "
          f"read={back[O_RDATA:O_RDATA+2].hex(' ')} steps={e.steps} ({why})")

    print("\n  -- requests that must be refused --")
    for seq, (op, addr, wdata, rlen, label) in enumerate(
            [(9, 0x10, b"", 0, "unknown op 9"),
             (OP_WRITE_READ, 0x10, b"x" * 30, 20, "total 50 exceeds limit 40"),
             (OP_WRITE, 0x80, b"\x00", 0, "address does not fit in 7 bits")], 1):
        back2, _e, _w = run(code, request(seq, op, addr, wdata, rlen), hw)
        good = (field(back2, O_ACK) == seq
                and signed(back2, O_RESULT) == FW_EINVAL)
        ok = ok and good
        print(f"  [{'ok ' if good else 'BAD'}] {label}: "
              f"result={NAMES.get(signed(back2, O_RESULT), signed(back2, O_RESULT))} "
              f"ack={field(back2, O_ACK)}")

    print("\n  -- does nothing when there is no slot --")
    empty = bytearray(request(1, OP_WRITE_READ, 0x10, b"\x02", 2))
    struct.pack_into(">I", empty, O_MAGIC, 0xDEADBEEF)
    back3, _e, _w = run(code, bytes(empty), hw, budget=400000)
    good = field(back3, O_ACK) == 0 and signed(back3, O_RESULT) == 0
    ok = ok and good
    print(f"  [{'ok ' if good else 'BAD'}] wrong magic: ack={field(back3, O_ACK)} "
          f"result={signed(back3, O_RESULT)}")

    print("\n  -- DT295 mode: the band only through the 0x00F1010C window --")
    # HW_BANDS covers only the I2C page; the window model needs the band
    # around it (0x00F1010C / 0x00F10114 are in fremu's default bands)
    bands = ((BAND_LO, BAND_HI),)
    global HW_BANDS
    saved, HW_BANDS = HW_BANDS, bands
    try:
        win = CpWindow(Fujitsu({0x10: Slave({0x80: 0x21})}, base=BASE_1684,
                               gated=False))
        back, e, why = run(code, request(1, OP_WRITE_READ, 0x10, b"\x80", 1),
                           win, mode=MODE_CPWIN)
        checks = [
            ("setup result is FW_OK", e.ld(SETUP_RC, 4) == 0),
            ("ACK is the request seq", field(back, O_ACK) == 1),
            ("result is FW_OK", signed(back, O_RESULT) == FW_OK),
            ("the byte read is the peer's", back[O_RDATA] == 0x21),
            ("no band access with the window shut", win.shut == []),
            ("0x00F1010C opened with 0x000B", CP_OPEN in win.cp_writes),
            ("0x00F1010C put back to the cold value", win.cp == CP_COLD),
            ("C0 = 0, bit4 set in +0x10, cleared in +0x14", win._up()),
            ("0x00F10114 bits 5:4 = 0b10", (win.pin & 0x30) == 0x20),
            ("+0x14 / +0x18 not written",
             win.inner.ff4 == 0 and win.inner.ff8 == 0),
        ]
        for name, good in checks:
            ok = ok and good
            print(f"  [{'ok ' if good else 'BAD'}] {name}")
        print(f"        result={NAMES.get(signed(back, O_RESULT), signed(back, O_RESULT))} "
              f"cp writes={[hex(x) for x in win.cp_writes]} shut={len(win.shut)} "
              f"steps={e.steps} ({why})")

        # Without the mode word the program drives the controller with no window, which
        # this band does not answer: the mode really is what opens it
        win = CpWindow(Fujitsu({0x10: Slave({0x80: 0x21})}, base=BASE_1684,
                               gated=False))
        back, e, why = run(code, request(1, OP_WRITE_READ, 0x10, b"\x80", 1),
                           win, mode=0)
        good = (field(back, O_ACK) == 1 and signed(back, O_RESULT) != FW_OK
                and win.cp_writes == [])
        ok = ok and good
        print(f"  [{'ok ' if good else 'BAD'}] without 'CPW1' the window stays shut "
              f"and the request fails: result="
              f"{NAMES.get(signed(back, O_RESULT), signed(back, O_RESULT))}")
    finally:
        HW_BANDS = saved

    print(f"\ntest_aux: {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
