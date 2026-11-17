#!/usr/bin/env python3
"""The dione doorbell (src/main/entry_frc1_dione.S) on the FR emulator.

The real boot stub and image run with a request descriptor and a setup
packet built the way the loader hands them over (r10, r11).

What it checks

  * a doorbell aimed at each of the four mailboxes (0x000C5000 + 0x40 * k)
    runs the frame found THERE, answers it in that mailbox, leaves the
    other mailboxes alone, and finishes the transfer
  * frames in two mailboxes are answered independently: an IDENTIFY in one
    and a bad command in another each get their own answer
  * anything else falls through to the loader's path untouched: an address
    off the mailboxes (0x000C5100, 0x000C5004, 0x000C4FC0), another
    sub-opcode, another wLength, another bRequest; sub-opcode 1 goes to the
    loader's read-modify-write path
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
MBOX0 = 0x000C5000
DESC = 0x000C5A00               # where this test puts the loader's descriptor
SETUP = 0x000C5B00              # and the setup packet
COMPLETE, PLAIN, MASK = 0x000E9FF0, 0x000E9FA6, 0x000E9F8A


def frame(seq, cmd):
    return struct.pack(">6I", MAGIC, seq, cmd, 0, 0, 0) + bytes(40)


def main():
    m = fwbuild.load()
    _code, _elf, tool = fwbuild.compile_all(m)
    dcode, boot, dsym, dlay = fwbuild.frc1_variant(m, "dione", tool, "m2tv")
    entry = dsym["frc1_entry"]
    fails = []

    def ok(cond, label, detail=""):
        print(f"  [{'ok ' if cond else 'NG '}] {label:<52}{detail}")
        if not cond:
            fails.append(label)

    def ring(frames, addr, subop=SUBOP, wlength=4, request=0xBC):
        """frames: {mailbox address: 64 bytes}.  Returns (exit, emu)."""
        page = bytearray(0x1000)            # 0x000C5000..0x000C5FFF
        for at, f in frames.items():
            page[at - MBOX0:at - MBOX0 + 64] = f
        e = fremu.Emu([(dlay["RES_BASE"], dcode), (entry, boot),
                       (MBOX0, bytes(page))], budget=2_000_000)
        desc = bytearray(0x50)
        desc[0x40] = subop
        desc[0x44:0x48] = struct.pack(">I", addr)
        desc[0x48:0x4A] = struct.pack(">H", wlength)
        e.write(DESC, bytes(desc))
        e.write(SETUP, bytes([0x40, request, 0, 0, 0, 0, 4, 0]))
        e.r[10], e.r[11], e.r[15] = DESC, SETUP, 0x000C7000
        why = e.run(entry)
        if why == "budget":
            raise RuntimeError("ran out of steps")
        return (e.externs[-1] if e.externs else None), e

    def mailbox(e, at):
        return bytes(e.ld(at + i, 1) for i in range(64))

    def status(e, at):
        return struct.unpack(">i", mailbox(e, at)[12:16])[0]

    print("\nEach of the four mailboxes")
    boxes = [MBOX0 + 0x40 * k for k in range(4)]
    for k, at in enumerate(boxes):
        frames = {b: frame(100 + i, CMD_IDENTIFY) for i, b in enumerate(boxes)}
        out, e = ring(frames, at)
        others = all(mailbox(e, b) == frames[b] for b in boxes if b != at)
        ok(out == COMPLETE and status(e, at) == 1 and others,
           f"doorbell at {at:#010x}: answered there only",
           f"exit {out:#x} status {status(e, at)}" if out else "no exit")

    print("\nTwo mailboxes, two different frames")
    frames = {boxes[0]: frame(7, CMD_IDENTIFY), boxes[1]: frame(8, 0x7777)}
    _out, e = ring(frames, boxes[1])
    st1 = status(e, boxes[1])
    _out, e2 = ring({boxes[0]: frames[boxes[0]]}, boxes[0])
    ok(st1 < 0 and status(e2, boxes[0]) == 1,
       "bad command in 0x000C5040, IDENTIFY in 0x000C5000",
       f"{st1} / {status(e2, boxes[0])}")

    print("\nNot ours: back to the loader untouched")
    f = frame(1, CMD_IDENTIFY)
    for label, kw, want in (
            ("address 0x000C5100 (.bss)", {"addr": 0x000C5100}, PLAIN),
            ("address 0x000C5004 (inside a mailbox)", {"addr": 0x000C5004}, PLAIN),
            ("address 0x000C4FC0 (below)", {"addr": 0x000C4FC0}, PLAIN),
            ("sub-opcode 0x00", {"addr": MBOX0, "subop": 0}, PLAIN),
            ("sub-opcode 0x01 (read-modify-write)", {"addr": MBOX0, "subop": 1}, MASK),
            ("wLength 0x40", {"addr": MBOX0, "wlength": 0x40}, PLAIN),
            ("bRequest 0xBD", {"addr": MBOX0, "request": 0xBD}, COMPLETE)):
        out, e = ring({MBOX0: f, 0x000C5100 - 0x40: f}, **kw)
        ok(out == want and mailbox(e, MBOX0) == f,
           label, f"exit {out:#x}" if out else "no exit")

    print()
    print("test_entry_dione: " + ("PASS" if not fails else f"FAIL ({len(fails)})"))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
