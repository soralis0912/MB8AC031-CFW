#!/usr/bin/env python3
"""The dione I2C path (src/main/fe_dione.c, i2c/fujitsu.c) against a model.

The controller model is the one tests/test_i2c.py builds.  Around it this
adds what dione needs before the controller takes a write, as measured on
a GV-M2TV (docs/i2c.md):

  * the block is held back while bit0 or bit1 of 0x00FD80C0 is 1, and also needs
    the clock (0x00FD8010 bit4) and reset (0x00FD8014 bit4 clear); while
    any of the three is wrong, writes to the controller are dropped
  * the demodulator at 7-bit 0x18 answers only with front-end power on
    (high halves of 0x00F10014 / 0x00F10018 = 0x02F4 / 0x03FF)

The model starts in the hardware's cold state: 0x00FD80C0 = 3 (with bit8
added, to see the other bits kept),
clock off, reset on, power off.

What it checks

  * I2CXFER brings everything up by itself: power, the dione pins
    (0x00F10114 bit19:18, and not selene's bit21:20), 0x00FD80C0 bit1:0
    cleared and its other bits kept
  * power is written before anything goes on the bus, the low halves of the
    power words are kept, and a second frame does not write power again
  * a write lands in the demodulator's register; a write-then-read and a
    plain read return the register's value in the data field
  * a peer that is not there is FW_EIO; bad counts and a reserved bit are
    FW_EINVAL and put nothing on the bus
  * I2CSCAN on the 0x00FD8FE0 instance finds 0x18 on dione
"""
import argparse
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
from test_i2c import Fujitsu, Slave                             # noqa: E402

FRAME = 0x000C5000
SP = 0x000C5FF0
CMD_I2CSCAN, CMD_I2CXFER = 0x45, 0x46
CMD_START, CMD_STATUS = 0x11, 0x20
ST_OK, FW_ENOTIMPL, FW_EIO, FW_EINVAL = 1, -1, -2, -4
NAMES = {1: "ok (+1)", -1: "FW_ENOTIMPL", -2: "FW_EIO", -3: "FW_ETIMEOUT",
         -4: "FW_EINVAL"}

FE_SEL, FE_DAT = 0x00F10014, 0x00F10018
PIN = 0x00F10114
CLK, RST, B0, C0 = 0x00FD8010, 0x00FD8014, 0x00FD80B0, 0x00FD80C0
BASE = 0x00FD8FE0
DEMOD = 0x18


class Dione:
    """The gate, the power and a Fujitsu controller behind them."""

    def __init__(self):
        self.r = {CLK: 0x00100004, RST: 0x0F2F001B, C0: 0x103, B0: 0,
                  PIN: 0xBA000000, FE_SEL: 0x00001234, FE_DAT: 0x00005678}
        self.demod = Slave({0x80: 0x00, 0xB0: 0x0A, 0x96: 0xE0})
        self.fj = Fujitsu({}, base=BASE)
        self.order = []          # every write, in order
        self.dropped = 0         # controller writes while the block was dead

    def live(self):
        return (self.r[CLK] & 0x10 and not self.r[RST] & 0x10
                and not self.r[C0] & 3)

    def powered(self):
        return (self.r[FE_SEL] >> 16 == 0x02F4
                and self.r[FE_DAT] >> 16 == 0x03FF)

    def __call__(self, addr, width, val):
        mine = BASE <= addr < BASE + 0x20
        if val is None:
            if mine:
                if not self.live():
                    return 0
                return self.fj(addr, width, None)
            return self.r.get(addr, 0)
        self.order.append((addr, val))
        if mine:
            if not self.live():
                self.dropped += 1
                return None
            self.fj.clocked = True
            self.fj.slaves = {DEMOD: self.demod} if self.powered() else {}
            return self.fj(addr, width, val)
        self.r[addr] = val
        return None


class DioneEmu(fremu.Emu):
    """dione's memory quirk: in the 0x000E band a byte or half-word store
    writes its whole word, and the rest of the word does not survive
    (measured, docs/image.md).  Modelled as zeros around the stored value.
    Every such store is also counted: nothing of ours should make one."""

    narrow_e = 0

    def st(self, a, width, val):
        if width in (1, 2) and 0x000E0000 <= fremu.fold(a) < 0x000F0000:
            self.narrow_e += 1
            a = fremu.fold(a)
            shift = 8 * (4 - width - (a & 3))
            word = (val & ((1 << (8 * width)) - 1)) << shift
            return super().st(a & ~3, 4, word)
        return super().st(a, width, val)


class Fw:
    def __init__(self, bands, hw, symbols):
        self.e = DioneEmu(bands, budget=60_000_000, hw=hw)
        self.hw = hw
        self.sym = symbols
        self.seq = 0

    def frame(self, cmd, arg0=0, arg1=0, data=b""):
        self.seq += 1
        f = struct.pack(">6I", 0x46525131, self.seq, cmd, 0, arg0, arg1)
        e = self.e
        e.write(FRAME, f + bytes(data) + bytes(40 - len(data)))
        e.r[4] = FRAME
        e.r[15] = SP
        e.steps = 0
        why = e.run(self.sym["frq1_dispatch"])
        if why == "budget":
            raise RuntimeError("ran out of steps")
        st = struct.unpack(">i", bytes(e.ld(FRAME + 12 + i, 1)
                                       for i in range(4)))[0]
        out = bytes(e.ld(FRAME + 0x18 + i, 1) for i in range(40))
        return st, out


def xfer_arg(addr, wn=0, rn=0):
    return (addr & 0x7F) | (wn << 8) | (rn << 16)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.parse_args()

    m = fwbuild.load()
    _code, _elf, tool = fwbuild.compile_all(m)
    dcode, _boot, dsym, dlay = fwbuild.frc1_variant(m, "dione", tool, "m2tv")
    bss_at = fwbuild.bss_base(dlay)
    bss_len = dsym["_bss_end"] - bss_at
    bands = [(dlay["RES_BASE"], dcode),
             (bss_at, b"\0" * (bss_len + (-bss_len % 4)))]
    fails = []

    def ok(cond, label, detail=""):
        print(f"  [{'ok ' if cond else 'NG '}] {label:<52}{detail}")
        if not cond:
            fails.append(label)

    print("\nOne write from the cold state")
    hw = Dione()
    fw = Fw(bands, hw, dsym)
    st, _ = fw.frame(CMD_I2CXFER, xfer_arg(DEMOD, wn=2), 0, b"\x01\x23")
    ok(st == ST_OK, "I2CXFER succeeds", NAMES.get(st, st))
    ok(hw.demod.regs.get(0x01) == 0x23, "the byte landed in register 0x01",
       hex(hw.demod.regs.get(0x01, -1)))
    ok(hw.dropped == 0, "no controller write while the block was dead",
       f"{hw.dropped}")
    ok(hw.r[C0] == 0x100, "0x00FD80C0 bit1:0 cleared, other bits kept",
       hex(hw.r[C0]))
    ok(hw.r[PIN] == 0xBA0C0000, "pins: bit19:18 set, bit21:20 untouched",
       hex(hw.r[PIN]))
    ok(hw.r[FE_SEL] == 0x02F41234 and hw.r[FE_DAT] == 0x03FF5678,
       "power on, low halves kept",
       f"{hw.r[FE_SEL]:#010x} {hw.r[FE_DAT]:#010x}")
    first_bus = next(i for i, (a, _v) in enumerate(hw.order)
                     if BASE <= a < BASE + 0x20)
    power_at = max(i for i, (a, _v) in enumerate(hw.order)
                   if a in (FE_SEL, FE_DAT))
    ok(power_at < first_bus, "power before the controller is touched")
    starts = [t for t in hw.fj.trace if t[0] == "start"]
    ok(starts == [("start", DEMOD, "w")], "one START, to 0x18 for writing",
       str(starts))

    print("\nThe next frames")
    power_writes = sum(1 for a, _v in hw.order if a in (FE_SEL, FE_DAT))
    hw.demod.regs[0xB0] = 0x5A
    st, d = fw.frame(CMD_I2CXFER, xfer_arg(DEMOD, wn=1, rn=1), 0, b"\xb0")
    ok(st == ST_OK and d[0] == 0x5A and d[1:] == bytes(39),
       "write 0xB0 then read: 0x5A, rest zero", d[:2].hex(" "))
    st, d = fw.frame(CMD_I2CXFER, xfer_arg(DEMOD, rn=2))
    ok(st == ST_OK and d[:2] == bytes((hw.demod.regs.get(0xB1, 0xFF),
                                       hw.demod.regs.get(0xB2, 0xFF))),
       "a plain read continues from the pointer", d[:2].hex(" "))
    again = sum(1 for a, _v in hw.order if a in (FE_SEL, FE_DAT))
    ok(again == power_writes, "power is not written again",
       f"{power_writes} -> {again}")
    st, d = fw.frame(CMD_I2CXFER, xfer_arg(DEMOD, wn=36), 0, bytes(range(36)))
    ok(st == ST_OK, "a 36-byte write (the longest record) goes",
       NAMES.get(st, st))
    sent = [v for tag, v, _x in hw.fj.trace[-38:] if tag == "wr"]
    ok(sent == list(range(36)), "and every byte went out as given "
       "(byte stores in 0x000E)", bytes(sent[:6]).hex(" "))
    record = bytes.fromhex("fec00d490e8b")
    st, d = fw.frame(CMD_I2CXFER, xfer_arg(DEMOD, wn=6), 0, record)
    sent = bytes(v for tag, v, _x in hw.fj.trace[-8:] if tag == "wr")
    ok(st == ST_OK and sent == record, "a 6-byte record goes out intact",
       sent.hex(" "))
    hw.demod.regs.update({0x40: 0x11, 0x41: 0x22, 0x42: 0x33, 0x43: 0x44,
                          0x44: 0x55})
    st, d = fw.frame(CMD_I2CXFER, xfer_arg(DEMOD, wn=1, rn=5), 0, b"\x40")
    ok(st == ST_OK and d[:5] == bytes.fromhex("1122334455"),
       "a 5-byte read comes back intact", d[:5].hex(" "))

    print("\nRefusals")
    before = len(hw.fj.trace)
    st, _ = fw.frame(CMD_I2CXFER, xfer_arg(0x19, wn=1), 0, b"\x00")
    ok(st == FW_EIO, "nobody at 0x19: FW_EIO", NAMES.get(st, st))
    ok(hw.fj.trace[-1][0] == "stop", "and the bus got a STOP")
    before = len(hw.fj.trace)
    for label, arg in (("nothing to do", xfer_arg(DEMOD)),
                       ("41 bytes", xfer_arg(DEMOD, wn=40, rn=1)),
                       ("address bit7", 0x80 | xfer_arg(DEMOD, wn=1)),
                       ("reserved top byte", 0x01000000 | xfer_arg(DEMOD, wn=1))):
        st, _ = fw.frame(CMD_I2CXFER, arg, 0, b"\x00")
        ok(st == FW_EINVAL, f"{label}: FW_EINVAL", NAMES.get(st, st))
    ok(len(hw.fj.trace) == before, "and nothing went on the bus",
       f"{len(hw.fj.trace) - before}")

    narrow = fw.e.narrow_e

    print("\nI2CSCAN on dione")
    hw = Dione()
    fw = Fw(bands, hw, dsym)
    st, d = fw.frame(CMD_I2CSCAN, 0x10, 0x100 | 16)
    base, setup, acks = struct.unpack(">I i I", d[4:16])
    ok(st == ST_OK and base == BASE and setup == 0,
       "scan sets the block up from cold", f"setup {setup}")
    ok(acks == 1 << (DEMOD - 0x10), "only 0x18 answers", f"{acks:#06x}")
    narrow += fw.e.narrow_e

    print("\nSTATUS on dione (the firmware's own reads)")
    hw = Dione()
    hw.demod.regs.update({0x8B: 0x12, 0x8C: 0x34, 0x8D: 0x56})
    fw = Fw(bands, hw, dsym)
    fw.frame(CMD_I2CXFER, xfer_arg(DEMOD, wn=2), 0, b"\x01\x00")  # power
    st, _ = fw.frame(CMD_START)
    ok(st == ST_OK, "START", NAMES.get(st, st))
    before = len(hw.fj.trace)
    st, d = fw.frame(CMD_STATUS, 0)
    locked, cnr = struct.unpack(">II", d[4:12])
    ok(st == ST_OK and d[1] & 0x03 == 0x03 and locked == 1,
       "lock read answers: locked", f"flags {d[1]:#04x} locked {locked}")
    ok(cnr == 0x123456, "C/N 0x8B..0x8D, one register at a time", f"{cnr:#x}")
    tr = hw.fj.trace[before:]
    restarts = [t for t in tr if t[0] == "restart"]
    ok(not restarts, "no repeated START: register number, STOP, then read",
       str(restarts[:2]))
    reads = [t for t in tr if t[0] == "rd"]
    ok(reads and all(t[2] == "nak" for t in reads),
       "every read is one byte, NAKed", f"{len(reads)} reads")
    starts = sum(1 for t in tr if t[0] == "start")
    stops = sum(1 for t in tr if t[0] == "stop")
    ok(starts == stops, "every transfer ends in STOP", f"{starts} / {stops}")
    narrow += fw.e.narrow_e

    print("\nThe 0x000E band")
    ok(narrow == 0, "no byte or half-word store lands there", f"{narrow}")
    ok(not 0x000E0000 <= bss_at < 0x000F0000 and not 0x000E0000 <= SP < 0x000F0000,
       ".bss and the stack are outside it", f"{bss_at:#010x} {SP:#010x}")

    print()
    print("test_i2c_dione: " + ("PASS" if not fails else f"FAIL ({len(fails)})"))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
