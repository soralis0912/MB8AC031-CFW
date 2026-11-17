#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Fujitsu FR (FR30 / FR60 / FR80) disassembler.

The ISA is fixed 16-bit, big-endian, with general registers R0..R15
(R13=AC, R14=FP, R15=SP) and dedicated registers TBR/RP/SSP/USP/MDH/MDL.
The only variable-length instructions are LDI:32 (6 B) and LDI:20 (4 B).

As a library, use `decode()` / `sweep()`.
"""

# ---------------------------------------------------------------- registers

RNAME = ["r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7",
         "r8", "r9", "r10", "r11", "r12", "ac", "fp", "sp"]

DRNAME = {0: "tbr", 1: "rp", 2: "ssp", 3: "usp", 4: "mdh", 5: "mdl"}


def _r(n):
    return RNAME[n & 0xF]


def _dr(n):
    return DRNAME.get(n & 0xF, "cr%d" % (n & 0xF))


def _s(v, bits):
    """Interpret as a signed value of `bits` bits."""
    m = 1 << (bits - 1)
    return (v ^ m) - m


def _m4(v):
    """4-bit immediate of ADD2 / ADDN2 / CMP2 ("minus extended").

    This is not sign extension. Only these 3 FR instructions interpret the
    immediate with the upper bits filled with 1, so the range is -16..-1,
    not -8..7. binutils `cpu/fr30.cpu` defines `f-m4` "4 bit minus extended"
    as `(or WI value (const -16))`.
    """
    return (v & 0xF) - 16


def _h(v):
    return ("-0x%x" % -v) if v < 0 else ("0x%x" % v)


# ---------------------------------------------------------------- instructions

# Flow kinds. Used to stop disassembly ranges and for recursive descent.
SEQ, RET, CALL, JMP, COND, TRAP = "seq", "ret", "call", "jmp", "cond", "trap"


class Insn(object):
    __slots__ = ("addr", "size", "mnem", "ops", "raw", "flow",
                 "target", "imm", "delay", "ok")

    def __init__(self, addr, size, mnem, ops="", raw=b"", flow=SEQ,
                 target=None, imm=None, delay=False, ok=True):
        self.addr = addr
        self.size = size
        self.mnem = mnem
        self.ops = ops
        self.raw = raw
        self.flow = flow
        self.target = target      # target of a direct branch or direct call
        self.imm = imm            # LDI-family immediate (for xref)
        self.delay = delay        # delayed branch (the next instruction is the delay slot)
        self.ok = ok              # decoded as a known encoding

    def text(self):
        return ("%-8s %s" % (self.mnem, self.ops)).rstrip()

    def __repr__(self):
        return "<%08x %s>" % (self.addr, self.text())


# Conditional branches. 0xE_ is non-delayed, 0xF_ is delayed.
CC = ["ra", "no", "eq", "ne", "c", "nc", "n", "p",
      "v", "nv", "lt", "ge", "le", "gt", "ls", "hi"]

# Two-operand ops fully determined by the high byte (op8 | Rj | Ri).
FMT_A = {
    0x00: ("ld",   "@(ac,%J),%I"),
    0x01: ("lduh", "@(ac,%J),%I"),
    0x02: ("ldub", "@(ac,%J),%I"),
    0x04: ("ld",   "@%J,%I"),
    0x05: ("lduh", "@%J,%I"),
    0x06: ("ldub", "@%J,%I"),
    0x10: ("st",   "%I,@(ac,%J)"),
    0x11: ("sth",  "%I,@(ac,%J)"),
    0x12: ("stb",  "%I,@(ac,%J)"),
    0x14: ("st",   "%I,@%J"),
    0x15: ("sth",  "%I,@%J"),
    0x16: ("stb",  "%I,@%J"),
    0x82: ("and",  "%J,%I"),
    0x84: ("and",  "%J,@%I"),
    0x85: ("andh", "%J,@%I"),
    0x86: ("andb", "%J,@%I"),
    0x8A: ("xchb", "@%J,%I"),
    0x8B: ("mov",  "%J,%I"),
    0x92: ("or",   "%J,%I"),
    0x94: ("or",   "%J,@%I"),
    0x95: ("orh",  "%J,@%I"),
    0x96: ("orb",  "%J,@%I"),
    0x9A: ("eor",  "%J,%I"),
    0x9C: ("eor",  "%J,@%I"),
    0x9D: ("eorh", "%J,@%I"),
    0x9E: ("eorb", "%J,@%I"),
    0xA2: ("addn", "%J,%I"),
    0xA6: ("add",  "%J,%I"),
    0xA7: ("addc", "%J,%I"),
    0xAA: ("cmp",  "%J,%I"),
    0xAB: ("mulu", "%J,%I"),
    0xAC: ("sub",  "%J,%I"),
    0xAD: ("subc", "%J,%I"),
    0xAE: ("subn", "%J,%I"),
    0xAF: ("mul",  "%J,%I"),
    0xB2: ("lsr",  "%J,%I"),
    0xB6: ("lsl",  "%J,%I"),
    0xBA: ("asr",  "%J,%I"),
    0xBB: ("muluh", "%J,%I"),
    0xBF: ("mulh", "%J,%I"),
}

# 4-bit immediate and Ri (op8 | u4/m4 | Ri). Value is (mnem, immediate kind).
# "u" = zero extended (0..15), "m" = minus extended (-16..-1, `_m4`).
FMT_B = {
    0x80: ("bandl", 0), 0x81: ("bandh", 0),
    0x88: ("btstl", 0), 0x89: ("btsth", 0),
    0x90: ("borl",  0), 0x91: ("borh",  0),
    0x98: ("beorl", 0), 0x99: ("beorh", 0),
    0xA0: ("addn",  0), 0xA1: ("addn2", "m"),
    0xA4: ("add",   0), 0xA5: ("add2",  "m"),
    0xA8: ("cmp",   0), 0xA9: ("cmp2",  "m"),
    0xB0: ("lsr",   0), 0xB1: ("lsr2",  0),
    0xB4: ("lsl",   0), 0xB5: ("lsl2",  0),
    0xB8: ("asr",   0), 0xB9: ("asr2",  0),
}
# Bit operations take "@Ri", the rest take "Ri".
FMT_B_MEM = {0x80, 0x81, 0x88, 0x89, 0x90, 0x91, 0x98, 0x99}

# 8-bit immediate (op8 | u8). (mnem, template, scale, signed)
FMT_D = {
    0x08: ("dmov",  "@%A,ac",    4, 0),
    0x09: ("dmovh", "@%A,ac",    2, 0),
    0x0A: ("dmovb", "@%A,ac",    1, 0),
    0x0B: ("dmov",  "@%A,@-sp",  4, 0),
    0x0C: ("dmov",  "@%A,@ac+",  4, 0),
    0x0D: ("dmovh", "@%A,@ac+",  2, 0),
    0x0E: ("dmovb", "@%A,@ac+",  1, 0),
    0x0F: ("enter", "#%A",       4, 0),
    0x18: ("dmov",  "ac,@%A",    4, 0),
    0x19: ("dmovh", "ac,@%A",    2, 0),
    0x1A: ("dmovb", "ac,@%A",    1, 0),
    0x1B: ("dmov",  "@sp+,@%A",  4, 0),
    0x1C: ("dmov",  "@ac+,@%A",  4, 0),
    0x1D: ("dmovh", "@ac+,@%A",  2, 0),
    0x1E: ("dmovb", "@ac+,@%A",  1, 0),
    0x1F: ("int",   "#%A",       1, 0),
    0x83: ("andccr", "#%A",      1, 0),
    0x87: ("stilm",  "#%A",      1, 0),
    0x93: ("orccr",  "#%A",      1, 0),
    0xA3: ("addsp",  "#%A",      4, 1),
}

# @(fp,disp) forms determined by the high nibble. (mnem, is store, scale)
FMT_FP = {
    0x2: ("ld",   0, 4),
    0x3: ("st",   1, 4),
    0x4: ("lduh", 0, 2),
    0x5: ("sth",  1, 2),
    0x6: ("ldub", 0, 1),
    0x7: ("stb",  1, 1),
}

# Sub-opcodes of 0x97 / 0x9F (j == subtype, with i == 0)
SUB_97 = {0x2: ("ret", RET), 0x3: ("reti", RET)}
SUB_9F = {0x2: ("ret:d", RET), 0x3: ("inte", TRAP),
          0x6: ("div3", SEQ), 0x7: ("div4s", SEQ),
          0x9: ("leave", SEQ), 0xA: ("nop", SEQ)}
# 0x97 forms that take Ri
SUB_97_RI = {0x4: "div0s", 0x5: "div0u", 0x6: "div1", 0x7: "div2",
             0x8: "extsb", 0x9: "extub", 0xA: "extsh", 0xB: "extuh"}


def _reglist(mask, base, store):
    """LDM/STM register list. Note that STM and LDM use opposite bit orders.

    STM: bit7 = R(base+0). LDM: bit0 = R(base+0).
    In real firmware, 41 `stm1 0xf0` pair with 42 `ldm1 0x0f`, 13 `stm1 0xc0`
    pair with 13 `ldm1 0x03`, and so on. Reading both as R8..R11 / R8,R9 makes
    prologues and epilogues match up.
    Both are shown in ascending register order (memory order).
    """
    regs = [base + (k if not store else 7 - k)
            for k in range(8) if mask & (1 << k)]
    regs.sort()
    return "(%s)" % ",".join(_r(x) for x in regs)


def decode(data, off, addr=0):
    """Decode one instruction at data[off:] and return an Insn. None if it runs past the end."""
    n = len(data)
    if off + 2 > n:
        return None
    w = (data[off] << 8) | data[off + 1]
    hi = w >> 8
    j = (w >> 4) & 0xF
    i = w & 0xF
    u8 = w & 0xFF

    def mk(size, mnem, ops="", **kw):
        return Insn(addr, size, mnem, ops, bytes(data[off:off + size]), **kw)

    def sub(t):
        return t.replace("%J", _r(j)).replace("%I", _r(i))

    # --- two-operand ops (op8 | Rj | Ri)
    if hi in FMT_A:
        m, t = FMT_A[hi]
        return mk(2, m, sub(t))

    # --- @(fp,disp) forms (determined by the high nibble)
    if 0x20 <= hi <= 0x7F:
        m, store, mul = FMT_FP[hi >> 4]
        d = _s((w >> 4) & 0xFF, 8) * mul
        mem = "@(fp,%s)" % _h(d)
        return mk(2, m, ("%s,%s" % (_r(i), mem)) if store
                  else ("%s,%s" % (mem, _r(i))))

    # --- 8-bit immediate
    if hi in FMT_D:
        m, t, mul, sg = FMT_D[hi]
        v = (_s(u8, 8) if sg else u8) * mul
        return mk(2, m, t.replace("%A", _h(v)))

    # --- 4-bit immediate
    if hi in FMT_B:
        m, sg = FMT_B[hi]
        v = _m4(j) if sg == "m" else j
        tgt = ("@" + _r(i)) if hi in FMT_B_MEM else _r(i)
        return mk(2, m, "#%s,%s" % (_h(v), tgt))

    # --- @(sp,udisp) forms
    if hi == 0x03:
        return mk(2, "ld", "@(sp,%s),%s" % (_h(j * 4), _r(i)))
    if hi == 0x13:
        return mk(2, "st", "%s,@(sp,%s)" % (_r(i), _h(j * 4)))

    # --- stack pop/push and PS save/restore
    if hi == 0x07:
        if j == 0:
            return mk(2, "ld", "@sp+,%s" % _r(i))
        if j == 1:
            return mk(2, "mov", "%s,ps" % _r(i))
        if j == 8:
            return mk(2, "ld", "@sp+,%s" % _dr(i))
        if j == 9 and i == 0:
            return mk(2, "ld", "@sp+,ps")
    if hi == 0x17:
        if j == 0:
            return mk(2, "st", "%s,@-sp" % _r(i))
        if j == 1:
            return mk(2, "mov", "ps,%s" % _r(i))
        if j == 8:
            return mk(2, "st", "%s,@-sp" % _dr(i))
        if j == 9 and i == 0:
            return mk(2, "st", "ps,@-sp")

    # --- multiple load/store
    if hi in (0x8C, 0x8D, 0x8E, 0x8F):
        store = hi in (0x8E, 0x8F)
        base = 8 if hi in (0x8D, 0x8F) else 0
        m = ("stm" if store else "ldm") + ("1" if base else "0")
        return mk(2, m, _reglist(u8, base, store))

    # --- MOV with dedicated registers
    if hi == 0xB3:
        return mk(2, "mov", "%s,%s" % (_r(i), _dr(j)))
    if hi == 0xB7:
        return mk(2, "mov", "%s,%s" % (_dr(j), _r(i)))

    # --- coprocessor (resource)
    if hi == 0xBC:
        return mk(2, "ldres", "@%s+,#%s" % (_r(i), _h(j)))
    if hi == 0xBD:
        return mk(2, "stres", "#%s,@%s+" % (_h(j), _r(i)))

    # --- sub-opcodes of 0x97 / 0x9F
    if hi in (0x97, 0x9F):
        delayed = (hi == 0x9F)
        tab = SUB_9F if delayed else SUB_97
        if j == 0:
            return mk(2, "jmp:d" if delayed else "jmp", "@" + _r(i),
                      flow=JMP, delay=delayed)
        if j == 1:
            return mk(2, "call:d" if delayed else "call", "@" + _r(i),
                      flow=CALL, delay=delayed)
        if j in tab and i == 0:
            m, fl = tab[j]
            return mk(2, m, flow=fl, delay=(fl == RET and delayed))
        if not delayed and j in SUB_97_RI:
            return mk(2, SUB_97_RI[j], _r(i))
        if delayed and j == 8:                      # LDI:32 #i32,Ri
            if off + 6 > n:
                return None
            v = int.from_bytes(data[off + 2:off + 6], "big")
            return mk(6, "ldi:32", "#0x%08x,%s" % (v, _r(i)), imm=v)
        if delayed and j in (0xC, 0xD, 0xE, 0xF):   # COPOP / COPLD / COPST / COPSV
            if off + 4 > n:
                return None
            m = {0xC: "copop", 0xD: "copld", 0xE: "copst", 0xF: "copsv"}[j]
            w2 = (data[off + 2] << 8) | data[off + 3]
            return mk(4, m, "#%s,#%s,cr%d,cr%d"
                      % (_h(i), _h(w2 >> 8), (w2 >> 4) & 0xF, w2 & 0xF))
        return mk(2, ".word", "0x%04x" % w, ok=False)

    # --- LDI:20 / LDI:8
    if hi == 0x9B:
        if off + 4 > n:
            return None
        v = (j << 16) | ((data[off + 2] << 8) | data[off + 3])
        return mk(4, "ldi:20", "#0x%05x,%s" % (v, _r(i)), imm=v)
    if 0xC0 <= hi <= 0xCF:
        v = (w >> 4) & 0xFF
        return mk(2, "ldi:8", "#0x%02x,%s" % (v, _r(i)), imm=v)

    # --- direct call (label12)
    if 0xD0 <= hi <= 0xDF:
        d = _s(w & 0x7FF, 11) * 2
        return mk(2, "call:d" if (hi & 0x08) else "call", "0x%x" % (addr + 2 + d),
                  flow=CALL, target=addr + 2 + d, delay=bool(hi & 0x08))

    # --- conditional branch (label9)
    if 0xE0 <= hi <= 0xFF:
        cc = hi & 0x0F
        delayed = bool(hi & 0x10)
        d = _s(u8, 8) * 2
        tgt = addr + 2 + d
        m = "b" + CC[cc] + (":d" if delayed else "")
        fl = JMP if cc == 0 else (SEQ if cc == 1 else COND)
        return mk(2, m, "0x%x" % tgt, flow=fl, target=tgt, delay=delayed)

    return mk(2, ".word", "0x%04x" % w, ok=False)


def sweep(data, base, start=None, end=None):
    """Walk [start, end) linearly and yield Insns in order."""
    start = base if start is None else start
    end = base + len(data) if end is None else end
    a = start
    while a < end:
        ins = decode(data, a - base, a)
        if ins is None:
            break
        yield ins
        a += ins.size
