"""Fujitsu FR interpreter. This is how the tests in this tree actually run
the built firmware.

  * Control moves the PC plainly. `call` just sets `rp` and jumps; `ret` is
    `pc = rp`
  * Delay slots are handled at run time (evaluate the condition -> run the
    slot -> transfer)
  * Flags are always fully computed
  * `Emu(..., hw=...)` routes MMIO bands to a device model, so drivers that
    poke registers (sequence, wait strategy) can be tested without a device

Only the instruction encoding is shared with the neighbouring `frdis`. The
semantics are written independently here. Use `Emu` as a library.
"""
import os as _os
import sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))

import frdis

PAGE = 0x1000
FILL = 0xFF

# Bands treated as MMIO. This chip's registers are not in one place, so to test
# a driver that drives a band not listed here (such as the I2C at 0x01684FE0
# run by the auxiliary unit), add it with Emu(..., hw_bands=((lo, hi), ...)) (the caller adds it).
HW = ((0x00F00000, 0x01000000), (0x01200000, 0x01210000))

RET, CALL, JMP, COND, SEQ, TRAP = (frdis.RET, frdis.CALL, frdis.JMP,
                                   frdis.COND, frdis.SEQ, frdis.TRAP)


def fold(a):
    # `0x8xxxxxxx` and `0x4xxxxxxx` are different views of the same physical
    # address (`0x8xxxxxxx` is the cached side). Fold to the low 28 bits here
    a &= 0xFFFFFFFF
    if 0x80000000 <= a < 0x90000000 or 0x40000000 <= a < 0x50000000:
        a &= 0x0FFFFFFF
    return a


def _s(v, bits):
    m = 1 << (bits - 1)
    return (v ^ m) - m


def _m4(v):
    """4-bit immediate of ADD2 / ADDN2 / CMP2. Minus extended, -16..-1."""
    return (v & 0xF) - 16


class Stop(Exception):
    pass


class Emu(object):
    def __init__(self, bands_bytes, budget=2000000, hw=None, hw_bands=None):
        """bands_bytes: [(addr, bytes), ...]

        If `hw` is given, reads and writes in MMIO bands (`HW` and `hw_bands`)
        go to it. This is for writing a device model to drive a driver:

            hw(addr, width, None)   -> read. Returns the value
            hw(addr, width, value)  -> write. Return value is ignored

        Without it, reads give 0 and writes are dropped. A driver polling a
        register either spins forever or never loops when reads are always 0,
        so a model is needed to test its logic.
        """
        self.hw = hw
        self.hw_bands = tuple(HW) + tuple(hw_bands or ())
        self.mem = {}
        self.r = [0] * 16
        self.rp = self.tbr = self.ssp = self.usp = self.mdh = self.mdl = 0
        self.n = self.z = self.v = self.c = 0
        self.s = self.i = self.ilm = 0
        self.d0 = self.d1 = 0
        self.budget = budget
        self.steps = 0
        self.depth = 0
        self.frames = []    # return addresses of nested calls
        self.rp_skew = 0    # times rp and the return address differed at ret
        self.externs = []
        self.traps = []
        self.hw_reads = self.hw_writes = 0
        self.code = {}
        for a, blob in bands_bytes:
            self.write(a, blob)
            self.code[(a, a + len(blob))] = blob
        self.bands = [(a, a + len(b)) for a, b in bands_bytes]
        self.base = dict((k, bytearray(v)) for k, v in self.mem.items())

    # ------------------------------------------------------------ memory
    def _page(self, a, alloc):
        p = a >> 12
        pg = self.mem.get(p)
        if pg is None and alloc:
            pg = bytearray([FILL]) * PAGE
            self.mem[p] = pg
        return pg

    def write(self, a, blob):
        a = fold(a)
        for k, b in enumerate(blob):
            x = (a + k) & 0xFFFFFFFF
            self._page(x, True)[x & (PAGE - 1)] = b

    def _is_hw(self, a):
        for lo, hi in self.hw_bands:
            if lo <= a < hi:
                return True
        return False

    def ld(self, a, width):
        a = fold(a)
        if self._is_hw(a):
            self.hw_reads += 1
            return 0 if self.hw is None else (self.hw(a, width, None) & 0xFFFFFFFF)
        v = 0
        for k in range(width):
            x = (a + k) & 0xFFFFFFFF      # wrap at 32 bits (same as the C side)
            pg = self._page(x, False)
            v = (v << 8) | (FILL if pg is None else pg[x & (PAGE - 1)])
        return v

    def st(self, a, width, val):
        a = fold(a)
        if self._is_hw(a):
            self.hw_writes += 1
            if self.hw is not None:
                self.hw(a, width, val & 0xFFFFFFFF)
            return
        for k in range(width - 1, -1, -1):
            x = (a + k) & 0xFFFFFFFF      # wrap at 32 bits (same as the C side)
            self._page(x, True)[x & (PAGE - 1)] = val & 0xFF
            val >>= 8

    def inside(self, a):
        return any(lo <= a < hi for lo, hi in self.bands)

    def fetch(self, a):
        for (lo, hi), blob in self.code.items():
            if lo <= a < hi:
                return blob, a - lo, lo
        return None, 0, 0

    def diff(self):
        """After a run, return the 32-bit words changed since the image was loaded."""
        out = {}
        for p, pg in self.mem.items():
            b = self.base.get(p)
            for off in range(0, PAGE, 4):
                cur = bytes(pg[off:off + 4])
                old = bytes(b[off:off + 4]) if b else b"\xff\xff\xff\xff"
                if cur != old:
                    out[(p << 12) | off] = int.from_bytes(cur, "big")
        return out

    # ------------------------------------------------------------- flags
    def _nz(self, r):
        self.n = (r >> 31) & 1
        self.z = 1 if (r & 0xFFFFFFFF) == 0 else 0

    def _add(self, a, b, ci=0):
        w = (a & 0xFFFFFFFF) + (b & 0xFFFFFFFF) + ci
        r = w & 0xFFFFFFFF
        self.c = (w >> 32) & 1
        self.v = ((~(a ^ b) & (a ^ r)) >> 31) & 1
        self._nz(r)
        return r

    def _sub(self, a, b, ci=0):
        a &= 0xFFFFFFFF
        b &= 0xFFFFFFFF
        r = (a - b - ci) & 0xFFFFFFFF
        self.c = 1 if (a - b - ci) < 0 else 0
        self.v = (((a ^ b) & (a ^ r)) >> 31) & 1
        self._nz(r)
        return r

    def _shift(self, a, s, kind):
        a &= 0xFFFFFFFF
        s &= 31
        if s == 0:
            self.c = 0
            r = a
        elif kind == "l":
            self.c = (a >> (32 - s)) & 1
            r = (a << s) & 0xFFFFFFFF
        elif kind == "r":
            self.c = (a >> (s - 1)) & 1
            r = a >> s
        else:
            self.c = (a >> (s - 1)) & 1
            r = (_s(a, 32) >> s) & 0xFFFFFFFF
        self._nz(r)
        return r

    def ccr(self):
        return (self.c | (self.v << 1) | (self.z << 2) | (self.n << 3) |
                (self.i << 4) | (self.s << 5))

    def set_ccr(self, v):
        self.c = v & 1
        self.v = (v >> 1) & 1
        self.z = (v >> 2) & 1
        self.n = (v >> 3) & 1
        self.i = (v >> 4) & 1
        self.s = (v >> 5) & 1

    def ps(self):
        return self.ccr() | ((self.ilm & 0x1F) << 16)

    def set_ps(self, v):
        self.set_ccr(v & 0xFF)
        self.ilm = (v >> 16) & 0x1F

    # ------------------------------------------------------------- execution
    DR = {0: "tbr", 1: "rp", 2: "ssp", 3: "usp", 4: "mdh", 5: "mdl"}

    def getdr(self, n):
        return getattr(self, self.DR[n & 0xF])

    def setdr(self, n, v):
        setattr(self, self.DR[n & 0xF], v & 0xFFFFFFFF)

    def push(self, v):
        self.r[15] = (self.r[15] - 4) & 0xFFFFFFFF
        self.st(self.r[15], 4, v)

    def pop(self):
        v = self.ld(self.r[15], 4)
        self.r[15] = (self.r[15] + 4) & 0xFFFFFFFF
        return v

    def exec1(self, ins):
        """Semantics of one instruction. Branch transfer is done by the caller (run)."""
        raw = ins.raw
        w = (raw[0] << 8) | raw[1]
        hi = w >> 8
        j = (w >> 4) & 0xF
        i = w & 0xF
        u8 = w & 0xFF
        r = self.r

        def put(n, v):
            r[n & 0xF] = v & 0xFFFFFFFF

        # ---- load / store
        if hi in (0x00, 0x01, 0x02):
            width = {0x00: 4, 0x01: 2, 0x02: 1}[hi]
            put(i, self.ld(r[13] + r[j], width)); return
        if hi in (0x04, 0x05, 0x06):
            width = {0x04: 4, 0x05: 2, 0x06: 1}[hi]
            put(i, self.ld(r[j], width)); return
        if hi in (0x10, 0x11, 0x12):
            width = {0x10: 4, 0x11: 2, 0x12: 1}[hi]
            self.st(r[13] + r[j], width, r[i]); return
        if hi in (0x14, 0x15, 0x16):
            width = {0x14: 4, 0x15: 2, 0x16: 1}[hi]
            self.st(r[j], width, r[i]); return

        # ---- logic / arithmetic (register to register)
        if hi == 0x82:
            put(i, r[i] & r[j]); self._nz(r[i]); return
        if hi == 0x92:
            put(i, r[i] | r[j]); self._nz(r[i]); return
        if hi == 0x9A:
            put(i, r[i] ^ r[j]); self._nz(r[i]); return
        if hi in (0x84, 0x85, 0x86, 0x94, 0x95, 0x96, 0x9C, 0x9D, 0x9E):
            width = {0x84: 4, 0x85: 2, 0x86: 1,
                     0x94: 4, 0x95: 2, 0x96: 1,
                     0x9C: 4, 0x9D: 2, 0x9E: 1}[hi]
            op = "&" if hi in (0x84, 0x85, 0x86) else (
                 "|" if hi in (0x94, 0x95, 0x96) else "^")
            cur = self.ld(r[i], width)
            val = (cur & r[j]) if op == "&" else (
                  (cur | r[j]) if op == "|" else (cur ^ r[j]))
            mask = (1 << (width * 8)) - 1
            self.st(r[i], width, val & mask)
            self._nz(val & mask)
            return
        if hi == 0x8A:
            t = self.ld(r[j], 1)
            self.st(r[j], 1, r[i] & 0xFF)
            put(i, t); return
        if hi == 0x8B:
            put(i, r[j]); return
        if hi == 0xA2:
            put(i, r[i] + r[j]); return
        if hi == 0xAE:
            put(i, r[i] - r[j]); return
        if hi == 0xA6:
            put(i, self._add(r[i], r[j])); return
        if hi == 0xA7:
            put(i, self._add(r[i], r[j], self.c)); return
        if hi == 0xAC:
            put(i, self._sub(r[i], r[j])); return
        if hi == 0xAD:
            put(i, self._sub(r[i], r[j], self.c)); return
        if hi == 0xAA:
            self._sub(r[i], r[j]); return
        if hi in (0xAB, 0xAF):      # mulu / mul
            if hi == 0xAF:
                p = _s(r[i], 32) * _s(r[j], 32)
            else:
                p = r[i] * r[j]
            self.mdh = (p >> 32) & 0xFFFFFFFF
            self.mdl = p & 0xFFFFFFFF
            self.n = (p >> 63) & 1 if hi == 0xAB else (1 if p < 0 else 0)
            self.z = 1 if p == 0 else 0
            self.v = 0 if -0x80000000 <= p <= 0xFFFFFFFF else 1
            return
        if hi in (0xBB, 0xBF):      # muluh / mulh
            if hi == 0xBF:
                p = _s(r[i] & 0xFFFF, 16) * _s(r[j] & 0xFFFF, 16)
            else:
                p = (r[i] & 0xFFFF) * (r[j] & 0xFFFF)
            self.mdl = p & 0xFFFFFFFF
            self.n = (self.mdl >> 31) & 1
            self.z = 1 if p == 0 else 0
            self.v = 0
            return
        if hi == 0xB2:
            put(i, self._shift(r[i], r[j], "r")); return
        if hi == 0xB6:
            put(i, self._shift(r[i], r[j], "l")); return
        if hi == 0xBA:
            put(i, self._shift(r[i], r[j], "a")); return

        # ---- @(fp,disp)
        if 0x20 <= hi <= 0x7F:
            kind = hi >> 4
            width = {0x2: 4, 0x3: 4, 0x4: 2, 0x5: 2, 0x6: 1, 0x7: 1}[kind]
            d = _s((w >> 4) & 0xFF, 8) * width
            a = (r[14] + d) & 0xFFFFFFFF
            if kind in (0x3, 0x5, 0x7):
                self.st(a, width, r[i])
            else:
                put(i, self.ld(a, width))
            return

        # ---- @(sp,udisp)
        if hi == 0x03:
            put(i, self.ld(r[15] + j * 4, 4)); return
        if hi == 0x13:
            self.st(r[15] + j * 4, 4, r[i]); return

        # ---- stack / PS
        if hi == 0x07:
            if j == 0:
                put(i, self.pop()); return
            if j == 1:
                self.set_ps(r[i]); return
            if j == 8:
                self.setdr(i, self.pop()); return
            if j == 9 and i == 0:
                self.set_ps(self.pop()); return
        if hi == 0x17:
            if j == 0:
                self.push(r[i]); return
            if j == 1:
                put(i, self.ps()); return
            if j == 8:
                self.push(self.getdr(i)); return
            if j == 9 and i == 0:
                self.push(self.ps()); return

        # ---- 8-bit immediate
        if hi == 0x08:
            r[13] = self.ld(u8 * 4, 4); return
        if hi == 0x09:
            r[13] = self.ld(u8 * 2, 2); return
        if hi == 0x0A:
            r[13] = self.ld(u8, 1); return
        if hi == 0x0B:
            self.push(self.ld(u8 * 4, 4)); return
        if hi in (0x0C, 0x0D, 0x0E):
            width = {0x0C: 4, 0x0D: 2, 0x0E: 1}[hi]
            self.st(r[13], width, self.ld(u8 * width, width))
            r[13] = (r[13] + width) & 0xFFFFFFFF; return
        if hi == 0x0F:                          # enter #u10
            u10 = u8 * 4
            r[15] = (r[15] - 4) & 0xFFFFFFFF
            self.st(r[15], 4, r[14])
            r[14] = r[15]
            r[15] = (r[15] - (u10 - 4)) & 0xFFFFFFFF
            return
        if hi in (0x18, 0x19, 0x1A):
            width = {0x18: 4, 0x19: 2, 0x1A: 1}[hi]
            self.st(u8 * width, width, r[13]); return
        if hi == 0x1B:
            self.st(u8 * 4, 4, self.pop()); return
        if hi in (0x1C, 0x1D, 0x1E):
            width = {0x1C: 4, 0x1D: 2, 0x1E: 1}[hi]
            self.st(u8 * width, width, self.ld(r[13], width))
            r[13] = (r[13] + width) & 0xFFFFFFFF; return
        if hi == 0x1F:
            self.traps.append(("int", u8)); return
        if hi == 0x83:
            self.set_ccr(self.ccr() & u8); return
        if hi == 0x87:
            self.ilm = u8 & 0x1F; return
        if hi == 0x93:
            self.set_ccr(self.ccr() | u8); return
        if hi == 0xA3:
            r[15] = (r[15] + _s(u8, 8) * 4) & 0xFFFFFFFF; return

        # ---- 4-bit immediate
        if hi in (0x80, 0x81, 0x90, 0x91, 0x98, 0x99):
            cur = self.ld(r[i], 1)
            if hi == 0x80:
                nv = cur & (0xF0 | j)
            elif hi == 0x81:
                nv = cur & ((j << 4) | 0x0F)
            elif hi == 0x90:
                nv = cur | j
            elif hi == 0x91:
                nv = cur | (j << 4)
            elif hi == 0x98:
                nv = cur ^ j
            else:
                nv = cur ^ (j << 4)
            self.st(r[i], 1, nv & 0xFF); return
        if hi in (0x88, 0x89):
            mask = j if hi == 0x88 else (j << 4)
            val = self.ld(r[i], 1) & mask
            self.z = 1 if val == 0 else 0
            self.n = ((val >> 7) & 1) if hi == 0x89 else 0
            return
        # ADDN2 / ADD2 / CMP2 immediates are "minus extended", -16..-1. Not sign
        # extended (-8..7). See `f-m4` in binutils cpu/fr30.cpu.
        if hi in (0xA0, 0xA1):
            v = _m4(j) if hi == 0xA1 else j
            put(i, r[i] + v); return
        if hi in (0xA4, 0xA5):
            v = _m4(j) if hi == 0xA5 else j
            put(i, self._add(r[i], v & 0xFFFFFFFF)); return
        if hi in (0xA8, 0xA9):
            v = _m4(j) if hi == 0xA9 else j
            self._sub(r[i], v & 0xFFFFFFFF); return
        if hi in (0xB0, 0xB1, 0xB4, 0xB5, 0xB8, 0xB9):
            sh = j + (16 if hi in (0xB1, 0xB5, 0xB9) else 0)
            kind = "r" if hi in (0xB0, 0xB1) else (
                   "l" if hi in (0xB4, 0xB5) else "a")
            put(i, self._shift(r[i], sh, kind)); return

        # ---- multiple load / store
        if hi in (0x8C, 0x8D, 0x8E, 0x8F):
            store = hi in (0x8E, 0x8F)
            base = 8 if hi in (0x8D, 0x8F) else 0
            regs = sorted(base + (k if not store else 7 - k)
                          for k in range(8) if u8 & (1 << k))
            if store:
                for x in reversed(regs):
                    self.push(r[x])
            else:
                for x in regs:
                    r[x] = self.pop()
            return

        # ---- dedicated registers
        if hi == 0xB3:
            self.setdr(j, r[i]); return
        if hi == 0xB7:
            put(i, self.getdr(j)); return

        # ---- coprocessor
        if hi == 0xBC:
            self.ld(r[i], 4)
            put(i, r[i] + 4)
            self.traps.append(("res", j)); return
        if hi == 0xBD:
            put(i, r[i] + 4)
            self.traps.append(("res", j)); return

        # ---- 0x97 / 0x9F unary and immediate
        if hi == 0x97:
            if j == 3 and i == 0:
                self.traps.append(("reti", 0)); return
            if j == 4:                              # div0s Ri
                self.d0 = (self.mdl >> 31) & 1
                self.d1 = ((self.mdl >> 31) ^ (r[i] >> 31)) & 1
                self.mdh = 0xFFFFFFFF if (self.mdl >> 31) else 0
                return
            if j == 5:                              # div0u Ri
                self.d0 = self.d1 = 0
                self.mdh = 0
                return
            if j == 6:                              # div1 Ri
                # One step. Direction is D1; taken when C == (D0 ^ D1)
                want = self.d0 ^ self.d1
                self.mdh = ((self.mdh << 1) | (self.mdl >> 31)) & 0xFFFFFFFF
                self.mdl = (self.mdl << 1) & 0xFFFFFFFF
                if not self.d1:
                    tmp = (self.mdh - r[i]) & 0xFFFFFFFF
                    self.c = 1 if self.mdh < r[i] else 0
                else:
                    tmp = (self.mdh + r[i]) & 0xFFFFFFFF
                    self.c = 1 if tmp < self.mdh else 0
                if self.c == want:
                    self.mdh = tmp
                    self.mdl |= 1
                self.z = 1 if self.mdh == 0 else 0
                return
            if j == 7:                              # div2 Ri
                if not self.d1:
                    tmp = (self.mdh - r[i]) & 0xFFFFFFFF
                    self.c = 1 if self.mdh < r[i] else 0
                else:
                    tmp = (self.mdh + r[i]) & 0xFFFFFFFF
                    self.c = 1 if tmp < self.mdh else 0
                self.z = 1 if tmp == 0 else 0
                if self.z:
                    self.mdh = tmp
                return
            if j == 8:
                put(i, _s(r[i] & 0xFF, 8) & 0xFFFFFFFF); return
            if j == 9:
                put(i, r[i] & 0xFF); return
            if j == 0xA:
                put(i, _s(r[i] & 0xFFFF, 16) & 0xFFFFFFFF); return
            if j == 0xB:
                put(i, r[i] & 0xFFFF); return
        if hi == 0x9F:
            if j == 3 and i == 0:
                self.traps.append(("inte", 0)); return
            if j == 9 and i == 0:                   # leave
                r[15] = (r[14] + 4) & 0xFFFFFFFF
                r[14] = self.ld(r[15] - 4, 4); return
            if j == 6 and i == 0:                   # div3
                if self.z:
                    self.mdl = (self.mdl + 1) & 0xFFFFFFFF
                return
            if j == 7 and i == 0:                   # div4s
                if self.d1:
                    self.mdl = (-self.mdl) & 0xFFFFFFFF
                return
            if j == 0xA and i == 0:
                return
            if j == 8:
                put(i, ins.imm); return
            if j in (0xC, 0xD, 0xE, 0xF):
                self.traps.append(("cop", j)); return
        if hi == 0x9B or 0xC0 <= hi <= 0xCF:
            put(i, ins.imm); return

        # Control instructions (jmp/call/ret/branches) are handled by run()
        if hi in (0x97, 0x9F) and j in (0, 1, 2):
            return
        if 0xD0 <= hi <= 0xFF:
            return
        raise Stop("unimplemented encoding %#04x at %#08x" % (w, ins.addr))

    COND = {
        0x2: lambda e: e.z, 0x3: lambda e: not e.z,
        0x4: lambda e: e.c, 0x5: lambda e: not e.c,
        0x6: lambda e: e.n, 0x7: lambda e: not e.n,
        0x8: lambda e: e.v, 0x9: lambda e: not e.v,
        0xA: lambda e: e.v ^ e.n, 0xB: lambda e: not (e.v ^ e.n),
        0xC: lambda e: (e.v ^ e.n) | e.z, 0xD: lambda e: not ((e.v ^ e.n) | e.z),
        0xE: lambda e: e.c | e.z, 0xF: lambda e: not (e.c | e.z),
    }

    def decode_at(self, a):
        blob, off, lo = self.fetch(a)
        if blob is None:
            raise Stop("pc %#08x left the image" % a)
        ins = frdis.decode(blob, off, a)
        if ins is None or not ins.ok:
            raise Stop("undecodable at %#08x" % a)
        return ins

    def run(self, entry):
        """Enter at entry and run until the `ret` that leaves it.

        `call` just sets rp and jumps. `ret` is pc = rp. Stops when depth goes
        below the entry level. Returns the stop reason.
        """
        pc = fold(entry)
        self.depth = 0
        self.frames = []
        self.rp_skew = 0
        while True:
            self.steps += 1
            if self.steps > self.budget:
                return "budget"
            ins = self.decode_at(pc)
            size = max(ins.size, 2)
            hi = (ins.raw[0] << 8 | ins.raw[1]) >> 8
            flow = ins.flow

            # Decide the target and whether to transfer *before* running the delay slot
            taken, target = False, None
            if flow == COND:
                taken = bool(self.COND[hi & 0xF](self))
                target = ins.target
            elif flow == JMP:
                taken, target = True, (ins.target if ins.target is not None
                                       else self.r[ins.raw[1] & 0xF])
            elif flow == CALL:
                taken, target = True, (ins.target if ins.target is not None
                                       else self.r[ins.raw[1] & 0xF])
            elif flow == RET:
                taken = True

            self.exec1(ins)

            nxt = pc + size
            if ins.delay:
                d = self.decode_at(nxt)
                self.steps += 1
                self.exec1(d)
                nxt = nxt + max(d.size, 2)

            if flow == CALL:
                self.rp = nxt & 0xFFFFFFFF
                t = fold(target)
                if not self.inside(t):
                    self.externs.append(t)     # outside the image: treat as called and move on
                    pc = nxt
                else:
                    self.depth += 1
                    self.frames.append(nxt & 0xFFFFFFFF)
                    pc = t
                continue
            if flow == RET:
                # The return target is the top of frames, not `rp`. When a
                # function is entered on a synthetic stack, as in the tests,
                # `ld @sp+,rp` reads unwritten 0xFF memory and jumps to a bogus
                # address. Instead, count the times `rp` and frames differ in rp_skew
                self.depth -= 1
                back = self.frames.pop() if self.frames else None
                if back is not None and fold(self.rp) != back:
                    self.rp_skew += 1
                if self.depth < 0:
                    return "return"
                pc = back if back is not None else fold(self.rp)
                continue
            if flow in (JMP, COND) and taken:
                t = fold(target)
                if not self.inside(t):
                    # A jump outside the image is treated as a tail call and
                    # returns from the current function. The return target is the
                    # top of frames (the return address when this function was
                    # called). `rp` is not used, as a call mid-function may have overwritten it
                    self.externs.append(t)
                    self.depth -= 1
                    if self.depth < 0 or not self.frames:
                        return "return"
                    pc = self.frames.pop()
                    continue
                pc = t
                continue
            pc = nxt
