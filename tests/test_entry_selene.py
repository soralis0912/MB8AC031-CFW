#!/usr/bin/env python3
"""The selene gate (src/main/entry_frc1_selene.S) on the FR emulator.

The real boot stub and image run with a request descriptor built the way
the selene loader hands it over (r11: address +0x00, sub-opcode +0x46,
wLength +0x48; r10 the setup packet).

What it checks

  * the staging buffer still works: a 64-byte OUT aimed at 0x000C4620 runs
    the frame there and finishes the transfer
  * a four-byte doorbell aimed at each of the four mailboxes
    (0x003B0000 + 0x40 * k) runs the frame found THERE, answers it in that
    mailbox, leaves the others and the staging buffer alone, and finishes
    the transfer
  * the mailboxes are the start of .bss: frc1_mbox is 0x803B0000, whose
    uncached view is 0x003B0000 (the emulator folds the two together, as
    the chip does)
  * anything else goes back to the loader's plain path untouched: an
    address off the mailboxes, the cached view 0x803B0000, a 64-byte OUT
    at a mailbox, a four-byte one at the staging buffer, another
    sub-opcode

What this cannot tell: whether the loader hands a four-byte OUT to the
hook the same way it hands a 64-byte one.  That is for the device.
"""
import os
import struct
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_FW = os.path.dirname(_HERE)
sys.path.insert(0, os.path.join(_HERE, "fr"))
sys.path.insert(0, os.path.join(_FW, "host"))
import fremu                                                    # noqa: E402
import build as fwbuild                                         # noqa: E402

MAGIC = 0x46525131
CMD_IDENTIFY = 0x10
SUBOP = 0xC1
STAGING = 0x000C4620
MBOX0 = 0x003B0000
LOW = 0x000C4000                # staging, descriptor and setup live here
DESC = 0x000C5A00
SETUP = 0x000C5B00
COMPLETE, PLAIN = 0x000EA8C8, 0x000EA75A


def frame(seq, cmd):
    return struct.pack(">6I", MAGIC, seq, cmd, 0, 0, 0) + bytes(40)


def main():
    m = fwbuild.load()
    _code, _elf, tool = fwbuild.compile_all(m)
    code, boot, sym, lay = fwbuild.frc1_variant(m, "selene", tool, "dt295")
    entry = sym["frc1_entry"]
    fails = []

    def ok(cond, label, detail=""):
        print(f"  [{'ok ' if cond else 'NG '}] {label:<52}{detail}")
        if not cond:
            fails.append(label)

    def ring(frames, addr, subop=SUBOP, wlength=4):
        """frames: {address: 64 bytes}.  Returns (exit, emu)."""
        low = bytearray(0x2000)
        high = bytearray(0x10000)           # 0x003B0000.., .bss and the stack
        for at, f in frames.items():
            if at >= MBOX0:
                high[at - MBOX0:at - MBOX0 + 64] = f
            else:
                low[at - LOW:at - LOW + 64] = f
        e = fremu.Emu([(fremu.fold(lay["RES_BASE"]), code), (entry, boot),
                       (LOW, bytes(low)), (MBOX0, bytes(high))],
                      budget=2_000_000)
        desc = bytearray(0x50)
        desc[0x00:0x04] = struct.pack(">I", addr)
        desc[0x46] = subop
        desc[0x48:0x4A] = struct.pack(">H", wlength)
        e.write(DESC, bytes(desc))
        e.write(SETUP, bytes([0x40, 0xBC, subop, 0, 0, 0, wlength, 0]))
        e.r[10], e.r[11], e.r[15] = SETUP, DESC, 0x000C5F00
        why = e.run(entry)
        if why == "budget":
            raise RuntimeError("ran out of steps")
        return (e.externs[-1] if e.externs else None), e

    def at64(e, at):
        return bytes(e.ld(at + i, 1) for i in range(64))

    def status(e, at):
        return struct.unpack(">i", at64(e, at)[12:16])[0]

    print("\nWhere the mailboxes are")
    ok(sym.get("frc1_mbox") == 0x803B0000 and
       sym.get("frc1_mbox") == lay["RES_IMMUT_END"],
       "frc1_mbox opens .bss at 0x803B0000",
       f"{sym.get('frc1_mbox', 0):#x}")

    print("\nThe staging buffer, as before")
    f = frame(5, CMD_IDENTIFY)
    out, e = ring({STAGING: f}, STAGING, wlength=0x40)
    ok(out == COMPLETE and status(e, STAGING) == 1,
       "64 bytes at 0x000C4620: answered in place",
       f"exit {out:#x} status {status(e, STAGING)}" if out else "no exit")

    print("\nEach of the four mailboxes")
    boxes = [MBOX0 + 0x40 * k for k in range(4)]
    for k, at in enumerate(boxes):
        frames = {b: frame(100 + i, CMD_IDENTIFY) for i, b in enumerate(boxes)}
        frames[STAGING] = frame(99, CMD_IDENTIFY)
        out, e = ring(frames, at)
        others = all(at64(e, b) == frames[b] for b in frames if b != at)
        ok(out == COMPLETE and status(e, at) == 1 and others,
           f"doorbell at {at:#010x}: answered there only",
           f"exit {out:#x} status {status(e, at)}" if out else "no exit")

    print("\nTwo mailboxes, two different frames")
    frames = {boxes[1]: frame(7, CMD_IDENTIFY), boxes[2]: frame(8, 0x7777)}
    _out, e = ring(frames, boxes[2])
    st2 = status(e, boxes[2])
    _out, e = ring(frames, boxes[1])
    ok(st2 < 0 and status(e, boxes[1]) == 1,
       "bad command in 0x003B0080, IDENTIFY in 0x003B0040",
       f"{st2} / {status(e, boxes[1])}")

    print("\nNot ours: back to the loader untouched")
    f = frame(1, CMD_IDENTIFY)
    for label, kw in (
            ("address 0x003B0100 (past the mailboxes)", {"addr": 0x003B0100}),
            ("address 0x003B0004 (inside a mailbox)", {"addr": 0x003B0004}),
            ("address 0x803B0000 (the cached view)", {"addr": 0x803B0000}),
            ("address 0x003AFFC0 (below)", {"addr": 0x003AFFC0}),
            ("wLength 0x40 at a mailbox", {"addr": MBOX0, "wlength": 0x40}),
            ("wLength 4 at the staging buffer", {"addr": STAGING}),
            ("sub-opcode 0x00", {"addr": MBOX0, "subop": 0}),
            ("sub-opcode 0x01", {"addr": MBOX0, "subop": 1})):
        out, e = ring({MBOX0: f, STAGING: f}, **kw)
        ok(out == PLAIN and at64(e, MBOX0) == f and at64(e, STAGING) == f,
           label, f"exit {out:#x}" if out else "no exit")

    print()
    print("test_entry_selene: " + ("PASS" if not fails else f"FAIL ({len(fails)})"))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
