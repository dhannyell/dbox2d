"""Straight-line AVX2 kernel builder with a Belady register allocator.

A kernel is written as SSA operations over named values. A value lives in a
YMM register, in memory (a constraint field, a constant or a spill slot), or
both. The allocator keeps sixteen YMM registers and spills the value whose
next use is furthest away, so the emitted Go assembly keeps the operation
order of the Go source it transcribes, which keeps float results identical.
"""

NREGS = 16


class Value:
    def __init__(self, name, mem=None):
        self.name = name
        self.mem = mem          # memory operand string holding a valid copy
        self.reg = None
        self.uses = []          # op indices that read it


class Kernel:
    def __init__(self, spill_base=0):
        self.ops = []
        self.values = {}
        self.count = 0
        self.spill_base = spill_base

    # ----- value constructors -----
    def _new(self, prefix, mem=None):
        self.count += 1
        v = Value('%s%d' % (prefix, self.count), mem)
        self.values[v.name] = v
        return v

    def mem(self, operand):
        """A value already held in memory: a field or a constant."""
        return self._new('m', operand)

    # ----- operations -----
    # Each op is (kind, dst, srcs, reg_srcs, text) where text formats the
    # instruction with {d} and {s0}.. placeholders.
    def _op(self, text, srcs, reg_srcs, dst=True, pre=None):
        d = self._new('v') if dst else None
        self.ops.append(dict(text=text, dst=d, srcs=srcs, reg=reg_srcs, pre=pre))
        return d

    def bin(self, instr, a, b):
        # Go operand order: instr src2, src1, dst computes src1 op src2.
        return self._op(instr + ' {s1}, {s0}, {d}', [a, b], [0])

    def add(self, a, b): return self.bin('VADDPS', a, b)
    def sub(self, a, b): return self.bin('VSUBPS', a, b)
    def mul(self, a, b): return self.bin('VMULPS', a, b)
    def div(self, a, b): return self.bin('VDIVPS', a, b)
    # VMAXPS src1, src2 returns src1 > src2 ? src1 : src2, laneW.Max exactly.
    def max(self, a, b): return self.bin('VMAXPS', a, b)
    # VMINPS returns src1 < src2 ? src1 : src2, laneW.Min exactly.
    def min(self, a, b): return self.bin('VMINPS', a, b)
    def and_(self, a, b): return self.bin('VANDPS', a, b)

    def cmp(self, pred, a, b):
        return self._op('VCMPPS $%d, {s1}, {s0}, {d}' % pred, [a, b], [0])

    def gt(self, a, b): return self.cmp(0x1E, a, b)   # GT_OQ

    def blend(self, m, a, b):
        """m ? a : b. VBLENDVPS takes src2 where the mask is set."""
        return self._op('VBLENDVPS {s0}, {s1}, {s2}, {d}', [m, a, b], [0, 2])

    def shufps(self, imm, a, b):
        return self._op('VSHUFPS $0x%02x, {s1}, {s0}, {d}' % imm, [a, b], [0])

    def perm128(self, imm, a, b):
        return self._op('VPERM2F128 $0x%02x, {s1}, {s0}, {d}' % imm, [a, b], [0])

    def load(self, pre, operand):
        """Run GP code, then load a vector from operand into a register."""
        return self._op('VMOVUPS ' + operand + ', {d}', [], [], pre=pre)

    def store(self, a, operand, home=True):
        """Store a to operand; afterwards operand holds a valid copy of a."""
        self._op('VMOVUPS {s0}, ' + operand, [a], [0], dst=False)
        self.ops[-1]['store'] = operand if home else None
        self.ops[-1]['clobber'] = operand

    def store_gp(self, a, pre, operand):
        self._op('VMOVUPS {s0}, ' + operand, [a], [0], dst=False, pre=pre)

    def raw(self, text, srcs=(), reg_srcs=None):
        """Raw text with register sources; produces no value."""
        srcs = list(srcs)
        self._op(text, srcs, list(range(len(srcs))) if reg_srcs is None else reg_srcs, dst=False)

    def raw_def(self, text, srcs=(), reg_srcs=None):
        srcs = list(srcs)
        return self._op(text, srcs, list(range(len(srcs))) if reg_srcs is None else reg_srcs)

    # ----- allocation and emission -----
    def emit(self):
        for i, op in enumerate(self.ops):
            for s in op['srcs']:
                s.uses.append(i)
        regs = [None] * NREGS
        free_slots = []
        nslots = [0]
        out = []

        def next_use(v, i):
            for u in v.uses:
                if u >= i:
                    return u
            return None

        def slot():
            if free_slots:
                return free_slots.pop()
            off = self.spill_base + 32 * nslots[0]
            nslots[0] += 1
            return '%d(SP)' % off

        spill_homes = {}

        def release(v):
            if v.reg is not None:
                regs[v.reg] = None
                v.reg = None
            if v.name in spill_homes:
                free_slots.append(spill_homes.pop(v.name))

        def get_reg(i, keep):
            for r in range(NREGS):
                if regs[r] is None:
                    return r
            # Evict the register value used furthest ahead.
            best, bestd = None, -1
            for r in range(NREGS):
                v = regs[r]
                if v in keep:
                    continue
                nu = next_use(v, i)
                d = 1 << 30 if nu is None else nu
                if d > bestd:
                    best, bestd = r, d
            v = regs[best]
            if v.mem is None and next_use(v, i) is not None:
                s = slot()
                spill_homes[v.name] = s
                v.mem = s
                out.append('\tVMOVUPS Y%d, %s' % (best, s))
            regs[best] = None
            v.reg = None
            return best

        for i, op in enumerate(self.ops):
            srcs = op['srcs']
            keep = set(srcs)
            if op['pre']:
                out.extend('\t' + l for l in op['pre'])
            ops_txt = []
            for k, s in enumerate(srcs):
                if s.reg is None and k in op['reg']:
                    assert s.mem is not None, s.name
                    r = get_reg(i, keep)
                    out.append('\tVMOVUPS %s, Y%d' % (s.mem, r))
                    s.reg = r
                    regs[r] = s
            for s in srcs:
                ops_txt.append('Y%d' % s.reg if s.reg is not None else s.mem)
            # A store overwrites memory other values may still call home.
            clob = op.get('clobber')
            if clob:
                for v in self.values.values():
                    if v.mem == clob and v not in srcs:
                        if v.reg is None and next_use(v, i + 1) is not None:
                            raise RuntimeError('store clobbers live %s' % v.name)
                        v.mem = None
            # Sources that die here free their registers before the result.
            for s in set(srcs):
                if next_use(s, i + 1) is None:
                    release(s)
            d = op['dst']
            fmt = {'s%d' % k: t for k, t in enumerate(ops_txt)}
            if d is not None:
                r = get_reg(i, keep)
                d.reg = r
                regs[r] = d
                fmt['d'] = 'Y%d' % r
                if not d.uses:
                    release(d)
            out.append('\t' + op['text'].format(**fmt))
            if op.get('store'):
                srcs[0].mem = op['store']
        frame = self.spill_base + 32 * nslots[0]
        return out, frame
