#!/usr/bin/env python3
"""Check that the per-board paths really split and work.

With no device, we build in Python a controller that behaves exactly as
documented and run the real firmware against it. This only shows that the
code follows the procedure. Device timing (how bit0 rises, how bus errors
appear) cannot be seen this way; only the device can fill that in.

What this checks

  * Each image is linked with one device's row, and carries that row's USB
    ID and transport.  The 0x00FD8FE0 instance is driven through the test
    rows in tests/boards/ (selene layout, terrestrial and satellite
    demodulators) and the PIX-DT295 row
  * On the DT295 image, init releases the front-end reset (0x00F10018 bit1,
    then 0x00F10014 bit1) before the I2C pins and before fj_setup, and
    touches nothing in 0x0168.... or 0x017..... (no auxiliary core)
  * One transfer (START, address, ACK, restart, drop ACK on the last byte
    only, STOP) comes out in the documented order
  * With no peer, FW_EIO; if bit0 never comes, FW_ETIMEOUT instead of spinning
    forever. Either way it leaves a STOP on the bus before returning
  * The image built for no product (the `unknown` row) puts nothing on the bus
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
import genboards                                                # noqa: E402

SP = 0x000C8000
SCRATCH = 0x000C9000            # output area
STRAP = 0x00F10004
# A value read from a selene board. As a halfword it is 0x8A01; read as a word it
# becomes 0x8A010000, so the model returns by width (to catch word-read bugs)
STRAP_HALF = 0x8A01


def strap_value(width):
    return STRAP_HALF if width == 2 else (STRAP_HALF << 16)


BASE_FD8F, BASE_1684 = 0x00FD8FE0, 0x01684FE0
O_STATUS, O_CONTROL, O_DIV, O_DATA, O_FF4, O_FF8 = 0, 4, 8, 0x10, 0x14, 0x18
# Names on the 0x00FD8FE0 side (gate registers exist only here, so keep absolute addresses)
R_STATUS, R_CONTROL, R_DIV = 0x00FD8FE0, 0x00FD8FE4, 0x00FD8FE8
R_DATA, R_FF4, R_FF8 = 0x00FD8FF0, 0x00FD8FF4, 0x00FD8FF8
GATES = (0x00FD8010, 0x00FD8014, 0x00FD80B0, 0x00FD80C0)
R_114 = 0x00F10114
# The selene front-end reset: half-words, bit1 (FE_RESET_* in mb86m01.h)
FE_DIR, FE_DAT, FE_BIT = 0x00F10014, 0x00F10018, 0x0002
CT_DONE, CT_ACK, CT_START, CT_RESTART, CT_ERROR = 0x01, 0x08, 0x10, 0x20, 0x80
ST_BUSY, ST_NACK = 0x80, 0x10
FW_OK, FW_ENOTIMPL, FW_EIO, FW_ETIMEOUT, FW_EINVAL = 0, -1, -2, -3, -4
NAMES = {0: "FW_OK", -1: "FW_ENOTIMPL", -2: "FW_EIO", -3: "FW_ETIMEOUT",
         -4: "FW_EINVAL"}


class Slave:
    """I2C peer. Takes a 1-byte register number, then reads or writes."""

    def __init__(self, regs=None):
        self.regs = dict(regs or {})
        self.ptr = 0
        self.got_ptr = False

    def start(self, reading):
        self.got_ptr = reading     # if it starts with a read, no register number is sent

    def write(self, b):
        if not self.got_ptr:
            self.ptr, self.got_ptr = b, True
        else:
            self.regs[self.ptr] = b
            self.ptr = (self.ptr + 1) & 0xFF

    def read(self):
        v = self.regs.get(self.ptr, 0xFF)
        self.ptr = (self.ptr + 1) & 0xFF
        return v


class Fujitsu:
    """Model of the Fujitsu I2C controller, as described from measurements.

    Takes a base address. This chip has 2 instances of the same design
    (0x00FD8FE0 and 0x01684FE0), and which one is driven depends on the board.

    `gated=False` is the 0x01684FE0 side. The auxiliary unit setup does not
    touch clock or reset, so that instance runs from the start.

    Raising `done_after` makes it take several reads before bit0 rises (on the
    device one byte takes a few µs). `stuck=True` gives a peer that never sets it.

    `foreign` collects writes to addresses that are neither our instance nor
    0x00F10114.
    """

    def __init__(self, slaves, done_after=0, stuck=False, base=BASE_FD8F,
                 gated=True, stale=0):
        self.slaves, self.done_after, self.stuck = slaves, done_after, stuck
        # After a control write, the previous value (with the done mark) can be
        # read for a while. A measured device quirk. With this in place, a driver
        # that does not wait for it to settle mistakes the previous byte's mark for the current one
        self.stale = stale
        self.stale_left = 0
        self.prev_ctl = 0
        self.violations = []
        self.base = base
        self.r_status = base + O_STATUS
        self.r_control = base + O_CONTROL
        self.r_div = base + O_DIV
        self.r_data = base + O_DATA
        self.r_ff4 = base + O_FF4
        self.r_ff8 = base + O_FF8
        self.mine = (self.r_status, self.r_control, self.r_div, self.r_data,
                     self.r_ff4, self.r_ff8)
        self.gated = gated
        self.ctl = 0                # control value held by the device
        self.status = 0
        self.data = 0
        self.div = self.ff4 = self.ff8 = 0
        self.setup = []             # order touched during setup
        self.trace = []             # what went out on the bus
        self.foreign = []           # writes outside our instance
        self.open = False
        self.reading = False
        self.cur = None
        self.pending = 0
        self.clocked = not gated
        self.half = {}              # the front-end reset half-words

    # --- bus contents ---
    def _addr_phase(self, tag):
        addr, rd = self.data >> 1, self.data & 1
        self.trace.append((tag, addr, "r" if rd else "w"))
        self.cur = self.slaves.get(addr)
        self.open, self.reading = True, bool(rd)
        self.status |= ST_BUSY
        if self.cur is None:
            self.status |= ST_NACK
        else:
            self.status &= ~ST_NACK
            self.cur.start(bool(rd))
        self._arm()

    def _next_byte(self, ack):
        if self.reading:
            v = self.cur.read() if self.cur else 0xFF
            self.data = v
            self.trace.append(("rd", v, "ack" if ack else "nak"))
        else:
            if self.cur:
                self.cur.write(self.data)
            self.trace.append(("wr", self.data, None))
        self._arm()

    def _arm(self):
        self.pending = None if self.stuck else self.done_after
        if self.pending == 0:
            self.ctl |= CT_DONE
            self.pending = -1

    def _stop(self):
        self.trace.append(("stop", None, None))
        self.open = False
        self.status &= ~ST_BUSY
        self.cur = None

    # --- registers ---
    def __call__(self, a, width, v):
        if v is None:
            return self._read(a, width)
        if a in (FE_DIR, FE_DAT):
            h = v & 0xFFFF if width == 2 else (v >> 16) & 0xFFFF
            self.half[a] = h
            self.setup.append((a, h))
            return None
        self._write(a, v)
        return None

    def _read(self, a, width=4):
        if a == STRAP:
            return strap_value(width)
        if a in (FE_DIR, FE_DAT):
            h = self.half.get(a, 0)
            return h if width == 2 else h << 16
        if a == self.r_control:
            if self.stale_left > 0:
                self.stale_left -= 1
                return self.prev_ctl
            if self.pending is not None and self.pending > 0:
                self.pending -= 1
                if self.pending == 0:
                    self.ctl |= CT_DONE
                    self.pending = -1
            return self.ctl
        if a == self.r_status:
            return self.status
        if a == self.r_data:
            return self.data
        if a == self.r_div:
            return self.div | 0x80       # reads back the written value with bit7 set
        if a == self.r_ff4:
            return self.ff4
        if a == self.r_ff8:
            return self.ff8
        return 0

    def _write(self, a, v):
        v &= 0xFFFFFFFF
        if a in GATES or a == R_114:
            self.setup.append((a, v))
            if a == GATES[0]:
                self.clocked = True
            if a in GATES and not self.gated:
                self.foreign.append((a, v))
            return
        if a == STRAP:
            return
        if a not in self.mine:
            self.foreign.append((a, v))
            return
        # Writes to the I2C block do not take effect before the clock is on
        if not self.clocked:
            self.setup.append((a, "dropped"))
            return
        if a == self.r_data:
            self.data = v & 0xFF
            return
        if a in (self.r_div, self.r_ff4, self.r_ff8):
            self.setup.append((a, v))
            if a == self.r_div:
                self.div = v & 0xFF
            elif a == self.r_ff4:
                self.ff4 = v & 0xFF
            else:
                self.ff8 = v & 0xFF
            return
        if a != self.r_control:
            return

        prev, new = self.ctl, v & 0xFF
        if self.stale and (prev & CT_DONE) and not (new & CT_DONE):
            self.prev_ctl, self.stale_left = prev, self.stale
        if not (new & (CT_DONE | CT_ACK | CT_START | CT_RESTART)):
            self.ctl = new
            if self.open:
                self._stop()
            else:
                self.setup.append((a, new))     # the 0x42 during setup
            return
        self.ctl = new
        if (new & CT_RESTART) and not (prev & CT_RESTART):
            self._addr_phase("restart")
        elif (new & CT_START) and not self.open:
            self._addr_phase("start")
        elif self.open and (prev & CT_DONE) and not (new & CT_DONE):
            # Record it if it moves on before the current byte is done
            if self.pending not in (-1, None):
                self.violations.append(("too early", len(self.trace)))
            # RESTART still standing on a byte step: on the PIX-DT295 the
            # controller took it as another repeated START, handed back the
            # address byte (0x21) as data and left the bus busy (measured).
            if new & CT_RESTART:
                self.violations.append(("restart held", len(self.trace)))
            self._next_byte(bool(new & CT_ACK))


# The 0x01684FE0 instance is not in fremu's default MMIO bands (this chip's
# registers are not in one place). Add a band to route it to the model.
HW_BANDS = ((BASE_1684 & ~0xFFF, (BASE_1684 & ~0xFFF) + 0x1000),)


FRAME = 0x000C4620          # servicer staging. Where the frame goes
FR_ARG0, FR_DATA = 0x10, 0x18


def frame(command, arg0=0):
    """FRQ1 frame (docs/frq1.md): magic, sequence, command, status, 2 args."""
    f = bytearray(64)
    struct.pack_into(">6I", f, 0, 0x46525131, 1, command, 0, arg0, 0)
    return bytes(f)


# Images by device id, linked on first use (main() sets "build")
_IMAGE = {}


def image(bid):
    """-> (memory bands, symbols) of the image linked with that device's row."""
    if bid not in _IMAGE:
        _IMAGE[bid] = _IMAGE["build"](bid)
    return _IMAGE[bid]


def call(board, fn, args, hw, budget=4000000, out_len=0, fr=None):
    """Call one fw function in that board's image.

    -> (return value, output, Emu, stop reason)

    `board` is a device id: every image is linked for one device, so the
    board is picked by which image runs.  If `fr` is given, place the frame
    at FRAME before the call, and with out_len=0 read the frame back and
    return it.
    """
    img, sym = image(board)
    bands = [(fremu.fold(a), b) for a, b in img]
    e = fremu.Emu(bands, budget=budget, hw=hw, hw_bands=HW_BANDS)
    if fr is not None:
        e.write(FRAME, fr)
    for i, v in enumerate(args):
        e.r[4 + i] = v
    e.r[15] = SP
    why = e.run(fremu.fold(sym[fn]))
    rv = e.r[4]
    if rv & 0x80000000:
        rv -= 1 << 32
    if out_len:
        got = bytes(e.ld(SCRATCH + i, 1) for i in range(out_len))
    elif fr is not None:
        got = bytes(e.ld(FRAME + i, 1) for i in range(64))
    else:
        got = b""
    return rv, got, e, why


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args()

    # The fixture rows (tests/boards/) must be in the tables before load()
    genboards.EXTRA_DIRS.append(os.path.join("tests", "boards"))
    m = fwbuild.load()
    code, _elf, tool = fwbuild.compile_all(m)

    def build_image(bid):
        c, elf = fwbuild.board_image(m, tool, bid)
        # Take it as memory after loading (including the .bss zeros)
        return fwbuild.loaded(m, c, tool, elf), fwbuild.symbols(elf, tool)
    _IMAGE["build"] = build_image
    fails = []

    def ok(cond, label, detail=""):
        print(f"  [{'ok ' if cond else 'NG '}] {label:<34}{detail}")
        if not cond:
            fails.append(label)

    print(f"\nCFW {len(code)} bytes")

    # --- the device tables ---
    print("\n-- device tables (config/devices/ + config/drivers.json) --")
    d = genboards.load(_FW)
    byid = {b["id"]: b for b in d["boards"]}
    ok(d["none"] == "unknown", "the row for no product is unknown",
       f"{len(byid)} rows")
    ok(byid["dt295"]["i2c"] == "fujitsu-fd8f" and
       byid["m2tv"]["i2c"] == "fujitsu-fd8f",
       "DT295 and M2TV use 0x00FD8FE0",
       f"{byid['dt295']['i2c']} / {byid['m2tv']['i2c']}")
    drv = d["drivers"]["i2c"]
    ok(drv["fujitsu-fd8f"]["requires"] == ["src/shared/fjcore.c"],
       "fd8f runs the shared state machine")
    ok(not any(b["i2c"] == "aux-slot" for b in d["boards"]) and
       "src/main/i2c/auxslot.c" in genboards.driver_sources(d),
       "no row selects aux-slot; it is still compiled")

    # --- each image carries its own row ---
    print("\n-- the row linked into each image --")
    for bid, usb, i2c in (("fd8f1", 0xFFFF0001, "i2c_fujitsu_ops"),
                          ("dt295", 0x06B8103B, "i2c_fujitsu_ops"),
                          ("m2tv", 0x04BB053A, "i2c_fujitsu_ops"),
                          ("unknown", 0, None)):
        img, sym = image(bid)
        e = fremu.Emu([(fremu.fold(a), b) for a, b in img], budget=1)
        got_usb = e.ld(fremu.fold(sym["fw_board"]), 4)
        got_i2c = e.ld(fremu.fold(sym["fw_board"]) + 4, 4)
        want_i2c = sym[i2c] if i2c else 0
        ok(got_usb == usb and got_i2c == want_i2c,
           f"{bid}: its USB ID and transport",
           f"usb {got_usb:#010x} i2c {got_i2c:#010x}")

    # --- setup ---
    want = [0x00FD8010, 0x00FD8014, 0x00FD80C0, R_DIV, R_CONTROL,
            0x00F10114, FE_DAT, FE_DIR]
    for bid in ("dt295", "fd8f1"):
        print(f"\n-- setup ({bid} row selected) --")
        hw = Fujitsu({0x10: Slave()})
        rv, _, _, why = call(bid, "i2c_init", [], hw)
        ok(rv == FW_OK and why == "return", "i2c_init", NAMES.get(rv, rv))
        order = [a for a, _ in hw.setup]
        ok(order == want, "sequence order as documented",
           "" if order == want else f"got {[hex(x) for x in order]}")
        ok(not any(v == "dropped" for _, v in hw.setup),
           "writes to I2C only after the clock")
        ok(hw.half.get(FE_DAT) == FE_BIT and hw.half.get(FE_DIR) == FE_BIT,
           "front-end reset: bit1, half-words",
           f"{hw.half.get(FE_DAT, 0):#06x} {hw.half.get(FE_DIR, 0):#06x}")

    # The DT295: the whole route is on the main FR.  The block is set up
    # first (fj_setup: divider, control), then the I2C pins (0x00F10114),
    # then the front-end reset is released -- and nothing in the auxiliary core's bands
    # (0x0168...., 0x017.....) is read or written.
    print("\n-- DT295: no auxiliary core --")

    class Watch(Fujitsu):
        def __init__(self, *args, **kw):
            Fujitsu.__init__(self, *args, **kw)
            self.aux = []
            self.f10 = []

        def __call__(self, a, width, v):
            if 0x01680000 <= a < 0x01690000 or 0x01700000 <= a < 0x01800000:
                self.aux.append((a, width, v))
                return 0 if v is None else None
            if v is not None and 0x00F10000 <= a < 0x00F11000:
                self.f10.append(a)
            return Fujitsu.__call__(self, a, width, v)

    global HW_BANDS
    keep = HW_BANDS
    HW_BANDS = keep + ((0x01680000, 0x01690000), (0x01700000, 0x01800000))
    try:
        hw = Watch({0x10: Slave({0x80: 0x21}), 0x11: Slave()})
        rv, _, _, _ = call("dt295", "i2c_init", [], hw)
        rv2, got, _, _ = call("dt295", "i2c_reg_read", [0x10, 0x80, SCRATCH],
                              hw, out_len=1)
    finally:
        HW_BANDS = keep
    order = [a for a, _ in hw.setup]

    def at(a):
        return order.index(a) if a in order else len(order)
    ok(rv == FW_OK and rv2 == FW_OK and got == b"\x21",
       "init and a register read pass", f"{NAMES.get(rv, rv)} "
       f"{NAMES.get(rv2, rv2)} {got.hex()}")
    ok(at(R_114) < at(FE_DAT) < at(FE_DIR),
       "0x00F10114, then FE reset 18, 14",
       f"{at(R_114)} {at(FE_DAT)} {at(FE_DIR)}")
    ok(at(R_CONTROL) < at(R_114),
       "fj_setup before the pins and the front-end reset")
    ok(hw.aux == [], "nothing in 0x0168.. / 0x017..",
       " ".join(f"{a:08x}" for a, _w, _v in hw.aux[:4]))
    ok(sorted(set(hw.f10)) == [FE_DIR, FE_DAT, R_114],
       "0x00F10xxx: FE reset and pins only",
       " ".join(f"{a:08x}" for a in sorted(set(hw.f10))))

    # --- one transfer: register write ---
    print("\n-- one transfer --")
    sl = Slave()
    hw = Fujitsu({0x10: sl})
    rv, _, _, _ = call("fd8f1", "i2c_reg_write", [0x10, 0x5A, 0x3C], hw)
    ok(rv == FW_OK, "i2c_reg_write", NAMES.get(rv, rv))
    ok(sl.regs.get(0x5A) == 0x3C, "reached the peer's register",
       f"0x5A = {sl.regs.get(0x5A)}")
    want = [("start", 0x10, "w"), ("wr", 0x5A, None), ("wr", 0x3C, None),
            ("stop", None, None)]
    ok(hw.trace == want, "bus order (write)",
       "" if hw.trace == want else f"got {hw.trace}")

    # --- one transfer: register read (restart to change direction) ---
    hw = Fujitsu({0x10: Slave({0x5A: 0x3C})}, done_after=3)
    rv, got, _, _ = call("fd8f1", "i2c_reg_read", [0x10, 0x5A, SCRATCH], hw, out_len=1)
    ok(rv == FW_OK and got == b"\x3c", "i2c_reg_read",
       f"{NAMES.get(rv, rv)} val={got.hex()}")
    want = [("start", 0x10, "w"), ("wr", 0x5A, None), ("restart", 0x10, "r"),
            ("rd", 0x3C, "nak"), ("stop", None, None)]
    ok(hw.trace == want, "bus order (read, no ACK on the last byte)",
       "" if hw.trace == want else f"got {hw.trace}")

    # --- how failures look ---
    print("\n-- failures --")
    hw = Fujitsu({0x11: Slave()})           # nobody at 0x10
    rv, _, _, _ = call("fd8f1", "i2c_reg_write", [0x10, 0, 0], hw)
    ok(rv == FW_EIO, "FW_EIO if no answer", NAMES.get(rv, rv))
    ok(bool(hw.trace) and hw.trace[-1][0] == "stop",
       "leaves a STOP on the bus even on failure", f"{hw.trace[-1:]}")

    hw = Fujitsu({0x10: Slave()}, stuck=True)
    rv, _, _, why = call("fd8f1", "i2c_reg_write", [0x10, 0, 0], hw, budget=20000000)
    ok(rv == FW_ETIMEOUT and why == "return",
       "FW_ETIMEOUT if bit0 never comes", f"{NAMES.get(rv, rv)} / {why}")
    ok(bool(hw.trace) and hw.trace[-1][0] == "stop",
       "leaves a STOP on the bus even on timeout", f"{hw.trace[-1:]}")

    rv, _, _, _ = call("fd8f1", "i2c_write", [0x10, SCRATCH, 41],
                       Fujitsu({0x10: Slave()}))
    ok(rv == FW_EINVAL, "FW_EINVAL above 40 bytes", NAMES.get(rv, rv))

    # --- no product: nothing on the bus ---
    print("\n-- an image for no product --")
    hw = Fujitsu({0x10: Slave()})
    rv, _, _, _ = call("unknown", "i2c_reg_write", [0x10, 0, 0], hw)
    ok(rv == FW_ENOTIMPL, "an image for no product: FW_ENOTIMPL", NAMES.get(rv, rv))
    ok(hw.trace == [] and hw.setup == [],
       "and it puts nothing on the bus",
       "" if not (hw.trace or hw.setup) else f"{hw.trace} {hw.setup}")

    # --- the demodulator uses the address from the table ---
    print("\n-- demodulator (uses the table address) --")
    hw = Fujitsu({0x10: Slave()})
    rv, _, _, _ = call("fd8f1", "demod_init", [0], hw)
    ok(rv == FW_ENOTIMPL, "FW_ENOTIMPL even when present (registers not known)",
       NAMES.get(rv, rv))
    ok(hw.trace and hw.trace[0] == ("start", 0x10, "r"),
       "probed table address 0x10 with a plain read",
       "" if hw.trace else "nothing on the bus")

    hw = Fujitsu({0x11: Slave()})
    rv, _, _, _ = call("fd8f1", "demod_init", [0], hw)
    ok(rv == FW_EIO, "FW_EIO if absent", NAMES.get(rv, rv))


    rv, _, _, _ = call("fd8f1", "demod_init", [4], Fujitsu({}))
    ok(rv == FW_EINVAL, "FW_EINVAL if source is out of range (4 or more)",
       NAMES.get(rv, rv))

    # --- lock and C/N (docs/demod.md) ---
    print("\n-- lock --")

    def word(got):
        return int.from_bytes(got, "big") if len(got) == 4 else None

    # Terrestrial: locked if 0x80 has none of the 0x28 bits and the low nibble of 0xB0 is 8 or more
    for regs, want, label in (
            ({0x80: 0x00, 0xB0: 0x08}, 1, "0x80=00 0xB0=08 -> locked"),
            ({0x80: 0x00, 0xB0: 0x0F}, 1, "0x80=00 0xB0=0F -> locked"),
            ({0x80: 0x00, 0xB0: 0x07}, 0, "0xB0 below 8 -> unlocked"),
            ({0x80: 0x28, 0xB0: 0x0F}, 0, "0x28 in 0x80 -> unlocked"),
            ({0x80: 0x08, 0xB0: 0x0F}, 0, "bit3 in 0x80 -> unlocked")):
        hw = Fujitsu({0x10: Slave(dict(regs))})
        rv, got, _, _ = call("fd8f1", "demod_locked", [0, SCRATCH], hw, out_len=4)
        ok(rv == FW_OK and word(got) == want, f"terr {label}",
           f"{NAMES.get(rv, rv)} locked={word(got)}")

    # If 0x80 already says no, 0xB0 is not read
    hw = Fujitsu({0x10: Slave({0x80: 0x28, 0xB0: 0x0F})})
    call("fd8f1", "demod_locked", [0, SCRATCH], hw, out_len=4)
    sent = [v for tag, v, _ in hw.trace if tag == "wr"]
    ok(sent == [0x80], "0xB0 not read when 0x80 already fails", f"{sent}")

    # Satellite: locked if bit4 of 0xC3 is clear
    for v, want in ((0x00, 1), (0x10, 0), (0xEF, 1), (0xFF, 0)):
        hw = Fujitsu({0x11: Slave({0xC3: v})})
        rv, got, _, _ = call("fd8f1", "demod_locked", [2, SCRATCH], hw, out_len=4)
        ok(rv == FW_OK and word(got) == want, f"sat 0xC3={v:#04x}",
           f"locked={word(got)}")

    hw = Fujitsu({0x11: Slave()})       # nobody at 0x10
    rv, _, _, _ = call("fd8f1", "demod_locked", [0, SCRATCH], hw)
    ok(rv == FW_EIO, "FW_EIO if no answer (not rounded to unlocked)",
       NAMES.get(rv, rv))

    print("\n-- raw C/N --")
    hw = Fujitsu({0x10: Slave({0x8B: 0x12, 0x8C: 0x34, 0x8D: 0x56})})
    rv, got, _, _ = call("fd8f1", "demod_cnr_raw", [0, SCRATCH], hw, out_len=4)
    ok(rv == FW_OK and word(got) == 0x123456,
       "terr 0x8B..0x8D, 24 bit", f"{word(got):#x}" if got else "")
    sent = [v for tag, v, _ in hw.trace if tag == "wr"]
    ok(sent == [0x8B, 0x8C, 0x8D], "one register number per byte read",
       f"{sent}")
    nread = len([1 for tag, _, _ in hw.trace if tag == "rd"])
    ok(nread == 3, "reads 3 bytes", f"{nread}")

    hw = Fujitsu({0x11: Slave({0xBC: 0xAB, 0xBD: 0xCD})})
    rv, got, _, _ = call("fd8f1", "demod_cnr_raw", [2, SCRATCH], hw, out_len=4)
    ok(rv == FW_OK and word(got) == 0xABCD, "sat 0xBC..0xBD, 16 bit",
       f"{word(got):#x}" if got else "")


    # No writes at all (this driver only reads)
    hw = Fujitsu({0x10: Slave({0x80: 0, 0xB0: 8, 0x8B: 1, 0x8C: 2, 0x8D: 3})})
    call("fd8f1", "demod_locked", [0, SCRATCH], hw)
    call("fd8f1", "demod_cnr_raw", [0, SCRATCH], hw)
    wrote = [v for tag, v, _ in hw.trace if tag == "wr"]
    ok(wrote == [0x80, 0xB0, 0x8B, 0x8C, 0x8D],
       "only register numbers are written to the bus (no values)",
       f"{[hex(x) for x in wrote]}")

    # --- bring-up: probe only, on both sides ---
    print("\n-- bring-up --")
    sl = Slave()
    hw = Fujitsu({0x11: sl})
    rv, _, _, _ = call("fd8f1", "demod_init", [2], hw)
    ok(rv == FW_ENOTIMPL, "satellite: probe only, so FW_ENOTIMPL", NAMES.get(rv, rv))
    ok(sl.regs == {}, "writes not a single word to satellite", f"{sl.regs}")

    # Terrestrial: the same
    sl = Slave()
    hw = Fujitsu({0x10: sl})
    rv, _, _, _ = call("fd8f1", "demod_init", [0], hw)
    ok(rv == FW_ENOTIMPL, "terrestrial: probe only, so FW_ENOTIMPL", NAMES.get(rv, rv))
    ok(sl.regs == {}, "writes not a single word to terrestrial", f"{sl.regs}")

    # Absent: no writes
    hw = Fujitsu({0x10: Slave()})
    rv, _, _, _ = call("fd8f1", "demod_init", [2], hw)
    ok(rv == FW_EIO, "FW_EIO if the satellite demod is absent", NAMES.get(rv, rv))
    ok([v for tag, v, _ in hw.trace if tag == "wr"] == [],
       "absent: writes not a single word")

    # --- device quirk: wait for it to settle ---
    print("\n-- per-byte wait quirk (only seen when run inside the hardware) --")
    sl = Slave()
    hw = Fujitsu({0x10: sl}, done_after=3, stale=5)
    rv, _, _, _ = call("fd8f1", "i2c_reg_write", [0x10, 0x5A, 0x3C], hw)
    ok(rv == FW_OK, "one transfer passes even if the mark drops late", NAMES.get(rv, rv))
    ok(hw.violations == [], "never moves on before the hardware is done",
       "" if not hw.violations else f"{hw.violations}")
    ok(sl.regs.get(0x5A) == 0x3C,
       "the value byte is not lost (lost without waiting to settle)",
       f"0x5A = {sl.regs.get(0x5A)}")
    want = [("start", 0x10, "w"), ("wr", 0x5A, None), ("wr", 0x3C, None),
            ("stop", None, None)]
    ok(hw.trace == want, "bus order has nothing missing",
       "" if hw.trace == want else f"got {hw.trace}")

    # Same for reads
    hw = Fujitsu({0x10: Slave({0x5A: 0x3C})}, done_after=3, stale=5)
    rv, got, _, _ = call("fd8f1", "i2c_reg_read", [0x10, 0x5A, SCRATCH], hw, out_len=1)
    ok(rv == FW_OK and got == b"\x3c" and hw.violations == [],
       "reads do not pick up a wrong value either", f"{NAMES.get(rv, rv)} {got.hex()}")

    # Even if settling takes longer than I2C_SETTLE, keep going instead of giving up
    hw = Fujitsu({0x10: Slave()}, done_after=3, stale=200)
    rv, _, _, _ = call("fd8f1", "i2c_reg_write", [0x10, 0x5A, 0x3C], hw)
    ok(rv == FW_OK, "moves on even if the mark is still up after 64 reads",
       NAMES.get(rv, rv))

    # --- second unit (device numbers 0..3; 0x12 / 0x13 inferred) ---
    print("\n-- second unit addresses --")
    for src, addr, label in ((0, 0x10, "device 0 -> 0x10 terr unit 1"),
                             (1, 0x12, "device 1 -> 0x12 terr unit 2"),
                             (2, 0x11, "device 2 -> 0x11 sat unit 1"),
                             (3, 0x13, "device 3 -> 0x13 sat unit 2")):
        regs = {0x80: 0x00, 0xB0: 0x08} if src < 2 else {0xC3: 0x00}
        hw = Fujitsu({addr: Slave(dict(regs))})
        rv, got, _, _ = call("fd8f2", "demod_locked", [src, SCRATCH], hw, out_len=4)
        ok(rv == FW_OK and int.from_bytes(got, "big") == 1, label,
           f"{NAMES.get(rv, rv)}")
        ok(hw.trace and hw.trace[0][1] == addr, f"  probed {addr:#04x}",
           "" if hw.trace else "nothing on the bus")

    # Single-tuner boards have no second-unit addresses
    for src in (1, 3):
        hw = Fujitsu({0x10: Slave(), 0x11: Slave(), 0x12: Slave(),
                      0x13: Slave()})
        rv, _, _, _ = call("fd8f1", "demod_locked", [src, SCRATCH], hw)
        ok(rv == FW_ENOTIMPL, f"fd8f1 (1 unit) has no device {src}",
           NAMES.get(rv, rv))
        ok(hw.trace == [], "  nothing on the bus")

    # --- fe.c report (queried by frame) ---
    print("\n-- status queried by frame (fe.c) --")
    CMD_STATUS, CMD_CNR = 0x20, 0x21
    D = FR_DATA

    def rep_field(fr, off, n=4):
        return int.from_bytes(fr[D + off:D + off + n], "big")

    regs = {0x80: 0x00, 0xB0: 0x0C, 0x8B: 0x01, 0x8C: 0x02, 0x8D: 0x03}
    hw = Fujitsu({0x10: Slave(dict(regs))})
    rv, fr, _, _ = call("fd8f1", "fe_channel_info", [FRAME], hw,
                        fr=frame(CMD_STATUS, 0))
    ok(rv == FW_OK, "fe_channel_info terrestrial", NAMES.get(rv, rv))
    ok(fr[D] == 1 and fr[D + 2] == 0, "version 1, source 0", f"{fr[D]} {fr[D+2]}")
    ok(rep_field(fr, 4) == 1, "lock is 1", f"{rep_field(fr, 4)}")
    ok(rep_field(fr, 8) == 0x010203, "C/N comes in as 24 bit",
       f"{rep_field(fr, 8):#x}")
    ok(rep_field(fr, 12) == 0 and rep_field(fr, 16) == 0,
       "both reads 0 (success)")
    ok(fr[D + 1] & 0x03 == 0x03, "flags have the lock and C/N valid bits",
       f"{fr[D+1]:#04x}")
    ok(fr[D + 1] & 0x80, "fe_init not called, so the uninitialized bit is set",
       f"{fr[D+1]:#04x}")

    # Satellite side
    hw = Fujitsu({0x11: Slave({0xC3: 0x10, 0xBC: 0xAB, 0xBD: 0xCD})})
    rv, fr, _, _ = call("fd8f1", "fe_channel_info", [FRAME], hw,
                        fr=frame(CMD_STATUS, 2))
    ok(rv == FW_OK and fr[D + 2] == 2, "fe_channel_info satellite",
       f"{NAMES.get(rv, rv)} source={fr[D+2]}")
    ok(rep_field(fr, 4) == 0 and rep_field(fr, 8) == 0xABCD,
       "C/N is returned even when unlocked", f"{rep_field(fr, 8):#x}")

    # The frame is answered even if one side is dead
    hw = Fujitsu({0x11: Slave()})       # nobody at 0x10
    rv, fr, _, _ = call("fd8f1", "fe_channel_info", [FRAME], hw,
                        fr=frame(CMD_STATUS, 0))
    ok(rv == FW_OK, "frame is answered even with no peer", NAMES.get(rv, rv))
    ok(fr[D + 1] & 0x03 == 0, "valid bits not set", f"{fr[D+1]:#04x}")
    ok(rep_field(fr, 12) == (FW_EIO & 0xFFFFFFFF),
       "read result holds FW_EIO", f"{rep_field(fr, 12):#x}")

    # A board with no address has nothing to answer
    hw = Fujitsu({0x10: Slave()})
    rv, _, _, _ = call("unknown", "fe_channel_info", [FRAME], hw,
                       fr=frame(CMD_STATUS, 0))
    ok(rv == FW_ENOTIMPL, "FW_ENOTIMPL if both are FW_ENOTIMPL",
       NAMES.get(rv, rv))

    # CMD_CNR is the value only
    hw = Fujitsu({0x10: Slave({0x8B: 0x11, 0x8C: 0x22, 0x8D: 0x33})})
    rv, fr, _, _ = call("fd8f1", "fe_cnr", [FRAME], hw,
                        fr=frame(CMD_CNR, 0))
    ok(rv == FW_OK and rep_field(fr, 8) == 0x112233, "fe_cnr",
       f"{NAMES.get(rv, rv)} {rep_field(fr, 8):#x}")

    # fe_init remembers present/absent
    hw = Fujitsu({0x10: Slave({0x80: 0, 0xB0: 8})})
    rv, _, _, _ = call("fd8f1", "fe_init", [], hw)
    ok(rv == FW_OK, "fe_init", NAMES.get(rv, rv))

    print("\ntest_i2c: " + ("PASS" if not fails else f"FAIL ({len(fails)})"))
    for f in fails:
        print("  - " + f)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
