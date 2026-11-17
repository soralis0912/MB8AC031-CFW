"""Hand the CFW real-shaped FRQ1 frames and inspect the answers without a device.

    python tests/test_frq1.py
    python tests/test_frq1.py -v

Two kinds of checks.

1. The answers. Run the image in `tests/fr/fremu.py` (an independently
   written FR interpreter), hand it a frame and look at the bytes returned
2. Whether the code region changed. Diff memory per command. A static that
   landed in .text instead of .bss shows up here

What this cannot tell: whether the chip boots the image, or whether TV reception works.

The firmware is C, so the fr30-elf toolchain is needed (docs/toolchain.md).
"""
import argparse
import os
import struct
import sys
import zlib

_HERE = os.path.dirname(os.path.abspath(__file__))
_FW = os.path.dirname(_HERE)
sys.path.insert(0, os.path.join(_FW, "host"))
sys.path.insert(0, os.path.join(_HERE, "fr"))

import build as fwbuild   # noqa: E402
import fremu              # noqa: E402

FRAME = 0x000C4620          # servicer staging buffer
PACKET = 64
MAGIC = 0x46525131          # 'FRQ1'
ST_PENDING, ST_OK = 0, 1
E_NOTIMPL, E_INVAL, E_BADCMD = -1, -4, -6
CMD = {"IDENTIFY": 0x10, "START": 0x11, "STOP": 0x12, "SUSPEND": 0x13,
       "HEALTH": 0x14,
       "STATUS": 0x20, "CNR": 0x21,
       "STREAM": 0x30,
       "PEEK": 0x40, "POKE": 0x41,
       "RING": 0x44, "I2CSCAN": 0x45, "EPDMA": 0x47,
       "CARD_STATUS": 0x50, "CARD_ON": 0x51, "CARD_OFF": 0x52,
       "CARD_PARAM": 0x53, "CARD_PUT": 0x54, "CARD_SEND": 0x55,
       "CARD_RECV": 0x56, "CARD_GET": 0x57}


def image():
    """Build and return it as memory after loading (including the zeroed .bss)."""
    m = fwbuild.load()
    code, _elf, tool = fwbuild.compile_all(m)
    return fwbuild.loaded(m, code, tool), len(code)


LAYOUT = fwbuild.layout()


def request(command, sequence=1, arg0=0, arg1=0, value=None):
    f = bytearray(PACKET)
    # magic, sequence, command, status (0 on the way in), 2 args
    struct.pack_into(">6I", f, 0, MAGIC, sequence, command, ST_PENDING,
                     arg0, arg1)
    # The value POKE writes is the first word of the data area (payload, not an arg)
    if value is not None:
        struct.pack_into(">I", f, 24, value)
    return bytes(f)


def field(resp, off):
    return struct.unpack_from(">I", resp, off)[0]


def status(resp):
    """Status field (+0x0C). +1 is success, negatives are FW_* from fw.h."""
    return struct.unpack_from(">i", resp, 12)[0]


class HwPage:
    """Replace fremu's MMIO band with plain memory that distinguishes widths.

    fremu reads 0 from `HW` bands and drops writes, so read-modify-write there
    cannot be checked by reading back. This is byte-wise memory, so a halfword
    `|=` only matches if it really is a halfword `|=`.
    """

    def __init__(self):
        self.b = {}

    def __call__(self, a, width, val):
        if val is None:
            v = 0
            for k in range(width):
                v = (v << 8) | self.b.get(a + k, 0)
            return v
        for k in range(width):
            self.b[a + width - 1 - k] = (val >> (8 * k)) & 0xFF
        return 0


class HwLog(HwPage):
    """HwPage that also keeps the order of the writes.

    EPDMA's whole point is that one write happens while the byte enable is
    open, so the final values are not enough: the sequence is the claim.
    """

    def __init__(self):
        HwPage.__init__(self)
        self.log = []

    def __call__(self, a, width, val):
        if val is not None:
            self.log.append((a, val))
        return HwPage.__call__(self, a, width, val)


SCRATCH_SP = 0x000C8000         # scratch stack below the immutable region


def run(img, frame, entry=None, sp=None, extra=(), hw=None,
        budget=2000000):
    bands = [(fremu.fold(a), b) for a, b in img]
    bands += list(extra)
    e = fremu.Emu(bands, hw=hw, budget=budget)
    e.write(FRAME, frame)
    e.r[4] = FRAME
    # C pushes onto the stack, so set sp
    e.r[15] = sp or SCRATCH_SP
    why = e.run(entry if entry is not None
                else fremu.fold(LAYOUT["RES_BASE"]))
    return bytes(e.ld(FRAME + i, 1) for i in range(PACKET)), e, why


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args()
    ok = True
    try:
        img, ncode = image()
    except fwbuild.NoToolchain as e:
        print(f"cannot build, so cannot test: {e}")
        return 1
    print(f"CFW: {ncode} bytes of code, "
          f"{len(img)} bands ({', '.join(f'{a:#010x}+{len(b)}' for a, b in img)})")

    print("\n  -- implemented commands --")
    # Only the status and data fields may change on the way back. Magic,
    # sequence and command keep their inbound values (the command echo marks "who answered")
    # IDENTIFY's arg0 is the page (0 or 1), so it gets one it accepts.
    for name in ("IDENTIFY", "START", "STOP"):
        a0 = 0x00000001 if name == "IDENTIFY" else 0x11112222
        resp, e, why = run(img, request(CMD[name], sequence=0x1234,
                                        arg0=a0, arg1=0x33334444))
        good = (why == "return" and field(resp, 0) == MAGIC
                and field(resp, 4) == 0x1234
                and field(resp, 8) == CMD[name]
                and status(resp) == ST_OK
                and field(resp, 16) == a0
                and field(resp, 20) == 0x33334444)
        ok = ok and good
        print(f"  [{'ok ' if good else 'BAD'}] {name:14s} status={status(resp)} "
              f"echo={field(resp,8):#x} args_unchanged="
              f"{field(resp,16) == a0 and field(resp,20) == 0x33334444} "
              f"steps={e.steps}")
        if a.verbose or not good:
            print(f"        resp {resp[:32].hex(' ')}")
    # IDENTIFY content: report version, protocol version, receiver, source
    # mask, signature, then the board row (0: this image is built for no
    # product) and the capability word (selene image, no card on that row)
    resp, _e, _w = run(img, request(CMD["IDENTIFY"]))
    good = resp[24:40] == bytes.fromhex("03010100465230310000000000000000")
    # then the build word: the CRC-32 of .text, which is the image's code band
    text = dict(img)[LAYOUT["RES_BASE"]]
    build = int.from_bytes(resp[40:44], "big")
    print(f"  [{'ok ' if build == zlib.crc32(text) else 'BAD'}] IDENTIFY build "
          f"{build:#010x}, crc32 of .text {zlib.crc32(text):#010x}")
    good = good and build == zlib.crc32(text)
    ok = ok and good
    print(f"  [{'ok ' if good else 'BAD'}] IDENTIFY content {resp[24:40].hex(' ')}")
    # Page 1: the receive bands of the row.  The row for no product has none.
    resp, _e, _w = run(img, request(CMD["IDENTIFY"], arg0=1))
    good = status(resp) == ST_OK and resp[24:28] == bytes([1, 1, 0, 0]) \
        and resp[28:64] == bytes(36)
    ok = ok and good
    print(f"  [{'ok ' if good else 'BAD'}] IDENTIFY page 1  {resp[24:28].hex(' ')}"
          " (v1, page 1, 0 bands on the no-product row)")
    resp, _e, _w = run(img, request(CMD["IDENTIFY"], arg0=2))
    good = status(resp) == E_INVAL
    ok = ok and good
    print(f"  [{'ok ' if good else 'BAD'}] IDENTIFY page 2  status={status(resp)}"
          " (E_INVAL)")

    print("\n  -- HEALTH (0x14): phase, flags, first failure per part --")

    def again(e, frame):
        """Another frame on the same emulator, so .bss carries over."""
        e.write(FRAME, frame)
        e.r[4] = FRAME
        e.r[15] = SCRATCH_SP
        e.budget = 2000000
        why = e.run(fremu.fold(LAYOUT["RES_BASE"]))
        return bytes(e.ld(FRAME + k, 1) for k in range(PACKET)), why

    def parts(resp):
        return [(struct.unpack_from(">b", resp, 32 + 4 * p)[0], resp[33 + 4 * p],
                 struct.unpack_from(">H", resp, 34 + 4 * p)[0])
                for p in range(7)]

    resp, e, _w = run(img, request(CMD["HEALTH"]))
    good = (status(resp) == ST_OK and resp[24] == 1 and resp[25] == 0
            and resp[26:28] == b"\0\0" and field(resp, 28) == 1
            and parts(resp) == [(0, 0, 0)] * 7)
    ok = ok and good
    print(f"  [{'ok ' if good else 'BAD'}] fresh: phase {resp[25]} flags "
          f"{resp[26:28].hex()} frames {field(resp, 28)} no failures")
    again(e, request(CMD["START"]))
    again(e, request(CMD["CNR"]))           # E_NOTIMPL, front end
    again(e, request(CMD["CNR"]))           # counted, first kept
    again(e, request(0x7F))                 # E_BADCMD, nobody's part
    again(e, request(CMD["PEEK"], arg0=1, arg1=4))   # E_INVAL, probe part
    resp, _w = again(e, request(CMD["HEALTH"], arg0=1))
    pt = parts(resp)
    want = [(0, 0, 0), (E_NOTIMPL, 0x21, 2), (0, 0, 0), (E_INVAL, 0x40, 1),
            (0, 0, 0), (0, 0, 0), (E_BADCMD, 0x7F, 1)]
    flags = struct.unpack_from(">H", resp, 26)[0]
    good = (status(resp) == ST_OK and resp[25] == 1 and flags == 0x0021
            and field(resp, 28) == 7 and pt == want
            and struct.unpack_from(">b", resp, 60)[0] == E_NOTIMPL
            and resp[61] == 1 and resp[62] == 0x21)
    ok = ok and good
    print(f"  [{'ok ' if good else 'BAD'}] after START, 2x CNR, 0x7F, a bad PEEK:"
          f" phase {resp[25]} flags {flags:#06x} frames {field(resp, 28)}")
    if not good or a.verbose:
        print(f"        parts {pt}")
    resp, _w = again(e, request(CMD["STOP"]))
    resp, _w = again(e, request(CMD["HEALTH"]))
    flags = struct.unpack_from(">H", resp, 26)[0]
    good = (resp[25] == 0 and flags == 0 and parts(resp) == [(0, 0, 0)] * 7
            and resp[60:63] == bytes([0, 0xff, 0]))
    ok = ok and good
    print(f"  [{'ok ' if good else 'BAD'}] cleared by arg0 bit0, STOP: phase "
          f"{resp[25]} flags {flags:#06x}")

    print("\n  -- commands not implemented yet (return a fixed failure) --")
    resp, _e, why = run(img, request(CMD["SUSPEND"]))
    d = resp[24:64]
    good = why == "return" and status(resp) == ST_OK and d[0] == 1 and d[1] == 0
    ok = ok and good
    print(f"  [{'ok ' if good else 'BAD'}] SUSPEND        status={status(resp)}"
          f" v{d[0]} down {d[1]:#04x} (nothing running)")
    for name in ("STREAM", "CNR",
                 "CARD_STATUS", "CARD_ON", "CARD_OFF", "CARD_PARAM",
                 "CARD_PUT", "CARD_SEND", "CARD_RECV", "CARD_GET"):
        resp, e, why = run(img, request(CMD[name]))
        good = why == "return" and status(resp) == E_NOTIMPL
        ok = ok and good
        print(f"  [{'ok ' if good else 'BAD'}] {name:14s} status={status(resp)}")
    # Unknown commands give E_BADCMD (not E_NOTIMPL).
    # 0x55 is now in the card band (CARD_SEND), so use the free 0x7F
    resp, _e, _w = run(img, request(0x7F))
    good = status(resp) == E_BADCMD
    ok = ok and good
    print(f"  [{'ok ' if good else 'BAD'}] unknown 0x7F  status={status(resp)}"
          " (E_BADCMD)")
    # There is no tune command (the host tunes by I2CXFER): 0x22, the last
    # front-end number before STATUS and CNR moved down, is free again.
    resp, _e, _w = run(img, request(0x22))
    good = status(resp) == E_BADCMD
    ok = ok and good
    print(f"  [{'ok ' if good else 'BAD'}] free 0x22     status={status(resp)}"
          " (E_BADCMD, no tune command)")
    # The reserved band is 0x01..0x0F. Ours are 0x10 and up, so even if they get
    # mixed up, the other side's commands do not run (docs/frq1.md)
    for cmd, label in ((1, "PING"), (2, "READ"), (3, "WRITE"), (4, "CALL")):
        resp, _e, _w = run(img, request(cmd, arg0=0x80380000, arg1=4))
        good = status(resp) == E_BADCMD
        ok = ok and good
        print(f"  [{'ok ' if good else 'BAD'}] rejects reserved-band {label} "
              f"({cmd})  status={status(resp)}")

    print("\n  -- frames that are not ours --")
    junk = bytearray(request(CMD["IDENTIFY"]))
    struct.pack_into(">I", junk, 0, 0xDEADBEEF)
    resp, e, _w = run(img, bytes(junk))
    good = resp == bytes(junk) and e.r[4] == 1
    ok = ok and good
    print(f"  [{'ok ' if good else 'BAD'}] wrong magic: unchanged={resp==bytes(junk)} "
          f"r4={e.r[4]} (1 = not handled)")

    print("\n  -- PEEK (0x40): read our own image and compare --")
    base = fremu.fold(LAYOUT["RES_BASE"])
    want = dict(img)[LAYOUT["RES_BASE"]][:16]
    for width, stride, label in ((0, 4, "word"), (1, 2, "half"), (2, 1, "byte")):
        resp, _e, _w = run(img, request(CMD["PEEK"], arg0=base,
                                        arg1=(width << 8) | 4))
        got = [field(resp, 24 + 8 + 4 * i) for i in range(4)]
        exp = [int.from_bytes(want[i * stride:(i + 1) * stride], "big")
               for i in range(4)]
        good = (status(resp) == ST_OK and resp[24] == 1
                and resp[25] == 4 and resp[26] == width
                and field(resp, 24 + 4) == base and got == exp)
        ok = ok and good
        print(f"  [{'ok ' if good else 'BAD'}] {label:4s} x4 @ {base:#x}  "
              + " ".join(f"{v:#x}" for v in got))
    # Requests to refuse. For a misaligned address, pretending the read worked is the worst outcome
    for arg, ctl, why in ((base, 0x0000, "count 0"),
                          (base, 0x0009, "count 9 (max 8)"),
                          (base + 1, 0x0004, "odd address for a word"),
                          (base + 1, 0x0104, "odd address for a halfword")):
        resp, _e, _w = run(img, request(CMD["PEEK"], arg0=arg, arg1=ctl))
        good = status(resp) == E_INVAL
        ok = ok and good
        print(f"  [{'ok ' if good else 'BAD'}] refuses: {why}  "
              f"status={status(resp)}")

    # 0x42 / 0x43 (a unit bring-up tried on another board) are not in this image
    print("\n  -- 0x42 / 0x43 are not answered --")
    for code in (0x42, 0x43):
        resp, _e, _w = run(img, request(code))
        good = status(resp) == E_BADCMD
        ok = ok and good
        print(f"  [{'ok ' if good else 'BAD'}] {code:#04x}: status={status(resp)}")

    # Zero the bands the probes touch. Otherwise fremu fills unwritten memory
    # with 0xFF, and "0 before the write" cannot be seen
    ZEROED = [(a, bytes(0x1000)) for a in (0x00D00000, 0x00D01000,
                                           0x01900000, 0x01901000,
                                           0x01902000, 0x01684000,
                                           0x01600000)]
    # POKE (0x41): a tool to tell "cannot read" from "cannot write". Whether an
    # address returning 0 is asleep or just resets to 0 can only be told by writing it.
    print("\n  -- POKE (0x41): write, read back, restore --")
    for addr, val, width, restore, label in (
            (0x01902024, 0xDEADBEEF, 0, 1, "word restore"),
            (0x01902024, 0xDEADBEEF, 0, 0, "word keep"),
            (0x00D00404, 0x0000BEEF, 1, 1, "half restore")):
        resp, e, _w = run(img, request(CMD["POKE"], arg0=addr,
                                       arg1=(width << 8) | restore,
                                       value=val),
                          extra=ZEROED, hw=HwPage())
        old_v, wrote, back, after = (field(resp, 24 + 8), field(resp, 24 + 12),
                                     field(resp, 24 + 16), field(resp, 24 + 20))
        mask = (0xFFFFFFFF, 0xFFFF, 0xFF)[width]
        good = (status(resp) == ST_OK and resp[24] == 1
                and resp[25] == width and resp[26] == restore
                and field(resp, 24 + 4) == addr
                and old_v == 0 and wrote == val
                and back == (val & mask) and after == 0
                and e.ld(addr, 4) == (0 if restore else (val & mask)))
        ok = ok and good
        print(f"  [{'ok ' if good else 'BAD'}] {label:12s} {addr:#x}  "
              f"old {old_v:#x} wrote {wrote:#x} read {back:#x} after {after:#x}")
    for addr, width, why in ((0x01902025, 0, "odd address for a word"),
                             (0x01902025, 1, "odd address for a halfword")):
        resp, _e, _w = run(img, request(CMD["POKE"], arg0=addr,
                                        arg1=(width << 8) | 1, value=1),
                           extra=ZEROED, hw=HwPage())
        good = status(resp) == E_INVAL
        ok = ok and good
        print(f"  [{'ok ' if good else 'BAD'}] refuses: {why}")

    # RING (0x44): arm the capture block through tsif_start() and read the
    # registers back. This image has no measured TS sequence, so arming is
    # refused (byte 2 = 1) and the registers stay as they were.
    print("\n  -- RING (0x44): arm, refused here, registers read back --")
    resp, e, _w = run(img, request(CMD["RING"], arg0=0),
                      extra=ZEROED, hw=HwPage())
    got = [field(resp, 24 + 4 + 4 * i) for i in range(7)]
    good = (status(resp) == ST_OK and resp[24] == 1 and resp[25] == 0
            and resp[26] == 0 and all(v == 0 for v in got))
    ok = ok and good
    print(f"  [{'ok ' if good else 'BAD'}] before setup: all 0  "
          + " ".join(f"{v:#x}" for v in got))
    resp, e, _w = run(img, request(CMD["RING"], arg0=1),
                      extra=ZEROED, hw=HwPage())
    mode, base, length, wptr, rptr, used, r010 = \
        [field(resp, 24 + 4 + 4 * i) for i in range(7)]
    good = (status(resp) == ST_OK and resp[25] == 1 and resp[26] == 1
            and mode == 0 and base == 0 and length == 0 and wptr == 0
            and rptr == 0 and used == 0 and r010 == 0)
    ok = ok and good
    print(f"  [{'ok ' if good else 'BAD'}] arm refused, nothing written: mode {mode:#x} "
          f"base {base:#x} len {length:#x} w {wptr:#x} r {rptr:#x} "
          f"used {used} r010 {r010:#x}")

    # I2CSCAN (0x45): which instance it drives, and whether it refuses bad windows.
    # On the bench with no peer, fj_setup fails to read back. That too is an
    # "answer about the chip", so it is returned in the content, not in status.
    print("\n  -- I2CSCAN (0x45): instance selection and window checks --")
    # Only 1 address in the window. With no peer, each transfer spins up to
    # I2C_SPIN (0x20000), and 16 addresses would exhaust the emulator's step budget.
    # On the bench we only want: which instance was driven, the report shape,
    # and whether bad windows are refused.
    for which, base, label in ((0, 0x01684FE0, "default (aux)"),
                               (1, 0x00FD8FE0, "USB side")):
        resp, _e, why = run(img, request(CMD["I2CSCAN"], arg0=0x10,
                                         arg1=(which << 8) | 1),
                            extra=ZEROED, hw=HwPage(), budget=40000000)
        got_base = field(resp, 24 + 4)
        setup_rc = struct.unpack_from(">i", resp, 24 + 8)[0]
        last_rc = struct.unpack_from(">i", resp, 24 + 16)[0]
        good = (why == "return" and status(resp) == ST_OK and resp[24] == 1
                and resp[25] == which and resp[26] == 1 and resp[27] == 0x10
                and got_base == base and setup_rc == 0
                and field(resp, 24 + 12) == 0 and last_rc != 0)
        ok = ok and good
        print(f"  [{'ok ' if good else 'BAD'}] {label:14s} base {got_base:#010x} "
              f"setup={setup_rc} ack=0 xfer={last_rc}"
              " (no peer, so failure is correct)")
    for first, n, why_s in ((0x10, 33, "window 33 (max 32)"),
                            (0x78, 16, "16 from 0x78 goes past 0x80")):
        resp, _e, _w = run(img, request(CMD["I2CSCAN"], arg0=first, arg1=n),
                           extra=ZEROED, hw=HwPage())
        good = status(resp) == E_INVAL
        ok = ok and good
        print(f"  [{'ok ' if good else 'BAD'}] refuses: {why_s}")

    # After running, has even 1 word of the code region changed?
    # A static that landed in .text instead of .bss shows up here (that is why
    # link_selene.ld places .bss above RES_IMMUT_END).
    # -- EPDMA (0x47): the sequence, not the result -------------------
    print("\n-- EPDMA --")
    GUARD, OPEN, CLOSED = 0x00FD814C, 8, 0xF
    DMA = {"cmd": 0x00FD8500, "run": 0x00FD8504, "addr": 0x00FD8508,
           "len": 0x00FD8510, "ctl": 0x00FD8514}
    SRC, LEN, K = 0x000EC000, 0x40, 1

    def epdma(src, ctl, hw):
        return run(img, request(CMD["EPDMA"], arg0=src, arg1=ctl), hw=hw)

    for name, src, ctl in (("src not word aligned", SRC + 2, 0x3F000000 | LEN),
                           ("arm with length 0", SRC, 0x02000000),
                           ("arm with odd length", SRC, 0x02000000 | 0x41)):
        resp, _e, _w = epdma(src, ctl, HwPage())
        good = status(resp) == E_INVAL
        ok = ok and good
        print(f"  [{'ok ' if good else 'BAD'}] {name:22s} E_INVAL")

    hw = HwLog()
    resp, _e, _w = epdma(SRC, 0x00000000 | LEN, hw)
    good = status(resp) == ST_OK and not hw.log
    ok = ok and good
    print(f"  [{'ok ' if good else 'BAD'}] mask 0: writes nothing          "
          f"writes={len(hw.log)}")

    hw = HwLog()
    resp, _e, _w = epdma(SRC, 0x3F000000 | (K << 16) | LEN, hw)
    cfg = 0x00FDBD08 + 8 * K
    want = [(DMA["run"], 0), (DMA["cmd"], 0),
            (DMA["ctl"], 0x40001000), (DMA["len"], LEN), (DMA["addr"], SRC),
            (DMA["cmd"], 2), (DMA["run"], 1),
            (GUARD, OPEN), (cfg, 1), (GUARD, CLOSED),
            (0x00FDD000, 1), (0x00FDD000, 0x00010001),
            (0x00FDC00C, 0x00010000)]
    good = status(resp) == ST_OK and hw.log == want
    ok = ok and good
    print(f"  [{'ok ' if good else 'BAD'}] full mask: the measured order   "
          f"{len(hw.log)} writes")
    if not good:
        for i, (g, w) in enumerate(zip(hw.log, want)):
            if g != w:
                print(f"        {i}: got {g[0]:#x}={g[1]:#x} "
                      f"want {w[0]:#x}={w[1]:#x}")
                break
    # The endpoint write must sit between the two guard writes, with the
    # guard open -- that is the thing the host cannot do.
    try:
        i = hw.log.index((cfg, 1))
        good = hw.log[i - 1] == (GUARD, OPEN) and hw.log[i + 1] == (GUARD, CLOSED)
    except (ValueError, IndexError):
        good = False
    ok = ok and good
    print(f"  [{'ok ' if good else 'BAD'}] the endpoint write is guarded")

    rep = [field(resp, 24 + 4 + 4 * i) for i in range(8)]
    good = (rep[0] == 2 and rep[1] == 1 and rep[2] == SRC and rep[3] == LEN
            and rep[4] == 0x40001000 and rep[6] == 1)
    ok = ok and good
    print(f"  [{'ok ' if good else 'BAD'}] reports what it left behind     "
          f"cmd={rep[0]} run={rep[1]} addr={rep[2]:#x} len={rep[3]:#x}")

    print("\n  -- after running, has even 1 word of the code region changed --")
    lo, hi = fremu.fold(LAYOUT["RES_BASE"]), \
        fremu.fold(LAYOUT["RES_IMMUT_END"])
    for name in ("IDENTIFY", "START", "STATUS", "CNR"):
        _resp, e, _why = run(img, request(CMD[name]))
        moved = sorted(a for a in e.diff() if lo <= a < hi)
        good = not moved
        ok = ok and good
        where = "" if good else (f"  changed: " +
                                 ", ".join(f"{a + 0x80000000:#x}"
                                           for a in moved[:4]))
        print(f"  [{'ok ' if good else 'BAD'}] {name:14s} "
              f"changed words {len(moved)}{where}")

    print(f"\ntest_frq1: {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
