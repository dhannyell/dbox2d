"""Generates contact_solver_wide_amd64.s.

Run from the repository root:  python tools/asmgen/main.py
Each kernel transcribes its Go counterpart in contact_solver_simd.go one
operation at a time, in the same order, so every lane rounds as the Go code
does. Without FMA the float results are identical bit for bit.
"""
import os
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(__file__))
from regalloc import Kernel  # noqa: E402

CW = 'contactConstraintWide_'
PW = 'contactPointWide_'
NULL = -1


def cf(field, sub=''):
    return '(%s%s%s)(CX)' % (CW, field, sub)


def pf(point, field, sub=''):
    base = CW + 'points'
    if point:
        base += '+contactPointWide__size'
    return '(%s+%s%s%s)(CX)' % (base, PW, field, sub)


X = '+vec2W_x'
Y = '+vec2W_y'

ZERO = '·laneConstZero(SB)'
NEG1 = '·laneConstNegOne(SB)'
# Frame: 0 invH, 32 the scatter sink for null lanes, 64 and 96 the other
# broadcast parameters, then the spill slots.
INVH = '0(SP)'
PARAM1 = '64(SP)'
PARAM2 = '96(SP)'
SPILL_BASE = 128


def gather(k, index_field):
    """gatherBodyW: eight rows, then the 8x8 transpose of simd_lane_amd64.go."""
    rows = []
    for j in range(8):
        off = '(%s%s+%d)(CX)' % (CW, index_field, 8 * j)
        pre = [
            'MOVQ %s, AX' % off,
            'MOVQ AX, BX',
            'SHLQ $5, AX',
            'ADDQ DI, AX',
            'CMPQ BX, $%d' % NULL,
            'CMOVQEQ DX, AX',
        ]
        rows.append(k.load(pre, '(AX)'))
    return transpose(k, rows)


def transpose(k, r):
    t = [k.shufps(0x44 if j % 2 == 0 else 0xEE, r[j & ~1], r[(j & ~1) + 1]) for j in range(8)]
    c0 = k.shufps(0x88, t[0], t[2])
    c1 = k.shufps(0xDD, t[0], t[2])
    c2 = k.shufps(0x88, t[1], t[3])
    c3 = k.shufps(0xDD, t[1], t[3])
    d0 = k.shufps(0x88, t[4], t[6])
    d1 = k.shufps(0xDD, t[4], t[6])
    d2 = k.shufps(0x88, t[5], t[7])
    d3 = k.shufps(0xDD, t[5], t[7])
    return [k.perm128(0x20, c0, d0), k.perm128(0x20, c1, d1),
            k.perm128(0x20, c2, d2), k.perm128(0x20, c3, d3),
            k.perm128(0x31, c0, d0), k.perm128(0x31, c1, d1),
            k.perm128(0x31, c2, d2), k.perm128(0x31, c3, d3)]


def scatter(k, store_field, cols):
    """scatterBodyW: transpose back and store the rows of real bodies."""
    rows = transpose(k, cols)
    for j in range(8):
        off = '(%s%s+%d)(CX)' % (CW, store_field, 8 * j)
        pre = [
            'MOVQ %s, AX' % off,
            'MOVQ AX, BX',
            'SHLQ $5, AX',
            'ADDQ DI, AX',
            'CMPQ BX, $%d' % NULL,
            'CMOVQEQ R8, AX',
        ]
        k.store_gp(rows[j], pre, '(AX)')


def cross(k, ax, ay, bx, by):
    return k.sub(k.mul(ax, by), k.mul(ay, bx))


def relax(k, rolling):
    """solveContactsTaskWide, one constraint; rolling selects the variant
    whose lane groups hold rolling resistance."""
    zero = k.mem(ZERO)
    neg1 = k.mem(NEG1)
    invH = k.mem(INVH)

    def neg(a):
        return k.mul(a, neg1)

    A = gather(k, 'indexA')
    B = gather(k, 'indexB')
    vAx, vAy, wA0, flA, dpAx, dpAy, dqAs, dqAc = A
    vBx, vBy, wB0, flB, dpBx, dpBy, dqBs, dqBc = B

    dpx = k.sub(dpBx, dpAx)
    dpy = k.sub(dpBy, dpAy)
    mA, iA = k.mem(cf('invMassA')), k.mem(cf('invIA'))
    mB, iB = k.mem(cf('invMassB')), k.mem(cf('invIB'))
    nx, ny = k.mem(cf('normal', X)), k.mem(cf('normal', Y))
    tx = ny
    ty = neg(nx)

    P = []
    for p in range(2):
        P.append(dict(
            aAx=k.mem(pf(p, 'anchorA', X)), aAy=k.mem(pf(p, 'anchorA', Y)),
            aBx=k.mem(pf(p, 'anchorB', X)), aBy=k.mem(pf(p, 'anchorB', Y)),
            base=k.mem(pf(p, 'baseSeparation')),
            ni=k.mem(pf(p, 'normalImpulse')), ti=k.mem(pf(p, 'tangentImpulse')),
            tni=k.mem(pf(p, 'totalNormalImpulse')),
            nm=k.mem(pf(p, 'normalMass')), tm=k.mem(pf(p, 'tangentMass'))))
    c1, c2 = P

    def sep(c):
        rsAx = k.sub(k.mul(dqAc, c['aAx']), k.mul(dqAs, c['aAy']))
        rsAy = k.add(k.mul(dqAs, c['aAx']), k.mul(dqAc, c['aAy']))
        rsBx = k.sub(k.mul(dqBc, c['aBx']), k.mul(dqBs, c['aBy']))
        rsBy = k.add(k.mul(dqBs, c['aBx']), k.mul(dqBc, c['aBy']))
        return k.add(k.add(k.mul(nx, k.add(dpx, k.sub(rsBx, rsAx))),
                           k.mul(ny, k.add(dpy, k.sub(rsBy, rsAy)))), c['base'])

    s1 = sep(c1)
    s2 = sep(c2)
    vb1 = k.max(k.mul(s1, invH), zero)
    vb2 = k.max(k.mul(s2, invH), zero)

    rnA1 = cross(k, c1['aAx'], c1['aAy'], nx, ny)
    rnB1 = cross(k, c1['aBx'], c1['aBy'], nx, ny)
    rnA2 = cross(k, c2['aAx'], c2['aAy'], nx, ny)
    rnB2 = cross(k, c2['aBx'], c2['aBy'], nx, ny)
    rtA1 = cross(k, c1['aAx'], c1['aAy'], tx, ty)
    rtB1 = cross(k, c1['aBx'], c1['aBy'], tx, ty)
    rtA2 = cross(k, c2['aAx'], c2['aAy'], tx, ty)
    rtB2 = cross(k, c2['aBx'], c2['aBy'], tx, ty)

    wnA1 = k.mul(iA, rnA1)
    wnB1 = k.mul(iB, rnB1)
    wnA2 = k.mul(iA, rnA2)
    wnB2 = k.mul(iB, rnB2)
    wtA1 = k.mul(iA, rtA1)
    wtB1 = k.mul(iB, rtB1)

    invMass = k.add(mA, mB)
    k21 = k.add(invMass, k.add(k.mul(wnA1, rnA2), k.mul(wnB1, rnB2)))
    k31 = k.add(k.mul(wnA1, rtA1), k.mul(wnB1, rtB1))
    k32 = k.add(k.mul(wnA2, rtA1), k.mul(wnB2, rtB1))
    k41 = k.add(k.mul(wnA1, rtA2), k.mul(wnB1, rtB2))
    k42 = k.add(k.mul(wnA2, rtA2), k.mul(wnB2, rtB2))
    k43 = k.add(invMass, k.add(k.mul(wtA1, rtA2), k.mul(wtB1, rtB2)))

    wA, wB = wA0, wB0
    dvx = k.sub(vBx, vAx)
    dvy = k.sub(vBy, vAy)
    dn = k.add(k.mul(dvx, nx), k.mul(dvy, ny))
    dt = k.sub(k.add(k.mul(dvx, tx), k.mul(dvy, ty)), k.mem(cf('tangentSpeed')))

    vn1 = k.add(dn, k.sub(k.mul(wB, rnB1), k.mul(wA, rnA1)))
    vn2 = k.add(dn, k.sub(k.mul(wB, rnB2), k.mul(wA, rnA2)))
    vt1 = k.add(dt, k.sub(k.mul(wB, rtB1), k.mul(wA, rtA1)))
    vt2 = k.add(dt, k.sub(k.mul(wB, rtB2), k.mul(wA, rtA2)))

    ni1 = k.max(neg(c1['ni']), neg(k.mul(c1['nm'], k.add(vn1, vb1))))
    nn1 = k.add(c1['ni'], ni1)
    k.store(nn1, pf(0, 'normalImpulse'))
    k.store(k.add(c1['tni'], ni1), pf(0, 'totalNormalImpulse'))

    vn2 = k.add(vn2, k.mul(k21, ni1))

    ni2 = k.max(neg(c2['ni']), neg(k.mul(c2['nm'], k.add(vn2, vb2))))
    nn2 = k.add(c2['ni'], ni2)
    k.store(nn2, pf(1, 'normalImpulse'))
    k.store(k.add(c2['tni'], ni2), pf(1, 'totalNormalImpulse'))

    vt1 = k.add(k.add(vt1, k.mul(k31, ni1)), k.mul(k32, ni2))
    vt2 = k.add(k.add(vt2, k.mul(k41, ni1)), k.mul(k42, ni2))

    angA = k.add(k.mul(ni1, rnA1), k.mul(ni2, rnA2))
    angB = k.add(k.mul(ni1, rnB1), k.mul(ni2, rnB2))

    if rolling:
        active = k.raw_def('VMOVUPS (R9), {d}')
        relW = k.sub(k.sub(wA, wB), k.add(k.mul(k.add(wnA1, wnB1), ni1), k.mul(k.add(wnA2, wnB2), ni2)))
        kk = k.add(iA, iB)
        q = k.div(relW, kk)
        dl = k.and_(k.gt(kk, zero), q)
        lam = k.mem(cf('rollingImpulse'))
        maxL = k.mul(k.mem(cf('rollingResistance')), k.add(nn1, nn2))
        # laneClamp(a, lo, hi) = lo > a ? lo : (a > hi ? hi : a). The outer
        # test reads a itself, which matters when lo > hi.
        a = k.add(lam, dl)
        lo = neg(maxL)
        ri = k.blend(k.gt(lo, a), lo, k.min(maxL, a))
        dl = k.sub(ri, lam)
        wtA2 = k.mul(iA, rtA2)
        wtB2 = k.mul(iB, rtB2)
        k.store(k.blend(active, ri, lam), cf('rollingImpulse'), home=False)
        vt1 = k.blend(active, k.add(vt1, k.mul(k.add(wtA1, wtB1), dl)), vt1)
        vt2 = k.blend(active, k.add(vt2, k.mul(k.add(wtA2, wtB2), dl)), vt2)
        angA = k.blend(active, k.add(angA, dl), angA)
        angB = k.blend(active, k.add(angB, dl), angB)

    friction = k.mem(cf('friction'))
    mf1 = k.mul(friction, nn1)
    ti1 = neg(k.mul(c1['tm'], vt1))
    ti1 = k.max(k.min(k.sub(mf1, c1['ti']), ti1), k.sub(neg(mf1), c1['ti']))
    k.store(k.add(c1['ti'], ti1), pf(0, 'tangentImpulse'), home=False)

    vt2 = k.add(vt2, k.mul(k43, ti1))

    mf2 = k.mul(friction, nn2)
    ti2 = neg(k.mul(c2['tm'], vt2))
    ti2 = k.max(k.min(k.sub(mf2, c2['ti']), ti2), k.sub(neg(mf2), c2['ti']))
    k.store(k.add(c2['ti'], ti2), pf(1, 'tangentImpulse'), home=False)

    angA = k.add(angA, k.add(k.mul(ti1, rtA1), k.mul(ti2, rtA2)))
    angB = k.add(angB, k.add(k.mul(ti1, rtB1), k.mul(ti2, rtB2)))

    nI = k.add(ni1, ni2)
    tI = k.add(ti1, ti2)
    px = k.add(k.mul(nI, nx), k.mul(tI, tx))
    py = k.add(k.mul(nI, ny), k.mul(tI, ty))

    vAx = k.sub(vAx, k.mul(mA, px))
    vAy = k.sub(vAy, k.mul(mA, py))
    wA = k.sub(wA, k.mul(iA, angA))
    vBx = k.add(vBx, k.mul(mB, px))
    vBy = k.add(vBy, k.mul(mB, py))
    wB = k.add(wB, k.mul(iB, angB))

    scatter(k, 'storeA', [vAx, vAy, wA, flA, dpAx, dpAy, dqAs, dqAc])
    scatter(k, 'storeB', [vBx, vBy, wB, flB, dpBx, dpBy, dqBs, dqBc])


HEADER = '''// Code generated by tools/asmgen; DO NOT EDIT.

//go:build dbox2d_simd && !dbox2d_fixed && goexperiment.simd && go1.27 && amd64

#include "go_asm.h"
#include "textflag.h"

DATA ·laneConstZero+0(SB)/8, $0
DATA ·laneConstZero+8(SB)/8, $0
DATA ·laneConstZero+16(SB)/8, $0
DATA ·laneConstZero+24(SB)/8, $0
GLOBL ·laneConstZero(SB), RODATA|NOPTR, $32

DATA ·laneConstNegOne+0(SB)/8, $0xbf800000bf800000
DATA ·laneConstNegOne+8(SB)/8, $0xbf800000bf800000
DATA ·laneConstNegOne+16(SB)/8, $0xbf800000bf800000
DATA ·laneConstNegOne+24(SB)/8, $0xbf800000bf800000
GLOBL ·laneConstNegOne(SB), RODATA|NOPTR, $32

DATA ·laneConstGroup+0(SB)/8, $0
DATA ·laneConstGroup+8(SB)/8, $0
DATA ·laneConstGroup+16(SB)/8, $0
DATA ·laneConstGroup+24(SB)/8, $0
DATA ·laneConstGroup+32(SB)/8, $-1
DATA ·laneConstGroup+40(SB)/8, $-1
DATA ·laneConstGroup+48(SB)/8, $0
DATA ·laneConstGroup+56(SB)/8, $0
DATA ·laneConstGroup+64(SB)/8, $0
DATA ·laneConstGroup+72(SB)/8, $0
DATA ·laneConstGroup+80(SB)/8, $-1
DATA ·laneConstGroup+88(SB)/8, $-1
DATA ·laneConstGroup+96(SB)/8, $-1
DATA ·laneConstGroup+104(SB)/8, $-1
DATA ·laneConstGroup+112(SB)/8, $-1
DATA ·laneConstGroup+120(SB)/8, $-1
GLOBL ·laneConstGroup(SB), RODATA|NOPTR, $128

DATA ·laneConstIdentityRow+0(SB)/8, $0
DATA ·laneConstIdentityRow+8(SB)/8, $0
DATA ·laneConstIdentityRow+16(SB)/8, $0
DATA ·laneConstIdentityRow+24(SB)/8, $0x3f80000000000000
GLOBL ·laneConstIdentityRow(SB), RODATA|NOPTR, $32
'''


def warm(k, rolling):
    """warmStartContactsTaskWide, one constraint; rolling applies the rolling
    impulse, as constraint.hasRolling does."""
    neg1 = k.mem(NEG1)
    A = gather(k, 'indexA')
    B = gather(k, 'indexB')
    vAx, vAy, wA, flA, dpAx, dpAy, dqAs, dqAc = A
    vBx, vBy, wB, flB, dpBx, dpBy, dqBs, dqBc = B
    mA, iA = k.mem(cf('invMassA')), k.mem(cf('invIA'))
    mB, iB = k.mem(cf('invMassB')), k.mem(cf('invIB'))
    nx, ny = k.mem(cf('normal', X)), k.mem(cf('normal', Y))
    tx = ny
    ty = k.mul(nx, neg1)
    for p in range(2):
        ni = k.mem(pf(p, 'normalImpulse'))
        ti = k.mem(pf(p, 'tangentImpulse'))
        px = k.add(k.mul(ni, nx), k.mul(ti, tx))
        py = k.add(k.mul(ni, ny), k.mul(ti, ty))
        vAx = k.sub(vAx, k.mul(mA, px))
        vAy = k.sub(vAy, k.mul(mA, py))
        wA = k.sub(wA, k.mul(iA, cross(k, k.mem(pf(p, 'anchorA', X)), k.mem(pf(p, 'anchorA', Y)), px, py)))
        vBx = k.add(vBx, k.mul(mB, px))
        vBy = k.add(vBy, k.mul(mB, py))
        wB = k.add(wB, k.mul(iB, cross(k, k.mem(pf(p, 'anchorB', X)), k.mem(pf(p, 'anchorB', Y)), px, py)))
        k.store(k.add(k.mem(pf(p, 'totalNormalImpulse')), ni), pf(p, 'totalNormalImpulse'), home=False)
    if rolling:
        ri = k.mem(cf('rollingImpulse'))
        wA = k.sub(wA, k.mul(iA, ri))
        wB = k.add(wB, k.mul(iB, ri))
    scatter(k, 'storeA', [vAx, vAy, wA, flA, dpAx, dpAy, dqAs, dqAc])
    scatter(k, 'storeB', [vBx, vBy, wB, flB, dpBx, dpBy, dqBs, dqBc])


def push(k, _):
    """pushContactsTaskWide, one constraint."""
    zero = k.mem(ZERO)
    neg1 = k.mem(NEG1)
    invH = k.mem(INVH)
    negCS = k.mem(PARAM1)
    one = k.mem(PARAM2)
    A = gather(k, 'indexA')
    B = gather(k, 'indexB')
    vAx, vAy, wA, flA, dpAx, dpAy, dqAs, dqAc = A
    vBx, vBy, wB, flB, dpBx, dpBy, dqBs, dqBc = B
    dpx = k.sub(dpBx, dpAx)
    dpy = k.sub(dpBy, dpAy)
    mA, iA = k.mem(cf('invMassA')), k.mem(cf('invIA'))
    mB, iB = k.mem(cf('invMassB')), k.mem(cf('invIB'))
    nx, ny = k.mem(cf('normal', X)), k.mem(cf('normal', Y))
    dn = k.add(k.mul(k.sub(vBx, vAx), nx), k.mul(k.sub(vBy, vAy), ny))
    rnA, rnB, imp = [], [], []
    for j in range(2):
        aAx, aAy = k.mem(pf(j, 'anchorA', X)), k.mem(pf(j, 'anchorA', Y))
        aBx, aBy = k.mem(pf(j, 'anchorB', X)), k.mem(pf(j, 'anchorB', Y))
        rsAx = k.sub(k.mul(dqAc, aAx), k.mul(dqAs, aAy))
        rsAy = k.add(k.mul(dqAs, aAx), k.mul(dqAc, aAy))
        rsBx = k.sub(k.mul(dqBc, aBx), k.mul(dqBs, aBy))
        rsBy = k.add(k.mul(dqBs, aBx), k.mul(dqBc, aBy))
        s = k.add(k.add(k.mul(nx, k.add(dpx, k.sub(rsBx, rsAx))),
                        k.mul(ny, k.add(dpy, k.sub(rsBy, rsAy)))), k.mem(pf(j, 'baseSeparation')))
        sep = k.gt(s, zero)
        vbias = k.blend(sep, k.mul(s, invH), k.max(negCS, k.mul(k.mem(cf('pushRate')), s)))
        mscale = k.blend(sep, one, k.mem(cf('massScale')))
        iscale = k.blend(sep, zero, k.mem(cf('impulseScale')))
        rnA.append(cross(k, aAx, aAy, nx, ny))
        rnB.append(cross(k, aBx, aBy, nx, ny))
        vn = k.add(dn, k.sub(k.mul(wB, rnB[j]), k.mul(wA, rnA[j])))
        if j == 1:
            kk = k.add(k.add(mA, mB), k.add(k.mul(k.mul(iA, rnA[0]), rnA[1]), k.mul(k.mul(iB, rnB[0]), rnB[1])))
            vn = k.add(vn, k.mul(kk, imp[0]))
        ni = k.mem(pf(j, 'normalImpulse'))
        negImp = k.add(k.mul(k.mem(pf(j, 'normalMass')), k.add(k.mul(mscale, vn), vbias)), k.mul(iscale, ni))
        impulse = k.max(k.mul(ni, neg1), k.mul(negImp, neg1))
        k.store(k.add(ni, impulse), pf(j, 'normalImpulse'), home=False)
        k.store(k.add(k.mem(pf(j, 'totalNormalImpulse')), impulse), pf(j, 'totalNormalImpulse'), home=False)
        imp.append(impulse)
    impulse = k.add(imp[0], imp[1])
    px = k.mul(impulse, nx)
    py = k.mul(impulse, ny)
    angA = k.add(k.mul(imp[0], rnA[0]), k.mul(imp[1], rnA[1]))
    angB = k.add(k.mul(imp[0], rnB[0]), k.mul(imp[1], rnB[1]))
    vAx = k.sub(vAx, k.mul(mA, px))
    vAy = k.sub(vAy, k.mul(mA, py))
    wA = k.sub(wA, k.mul(iA, angA))
    vBx = k.add(vBx, k.mul(mB, px))
    vBy = k.add(vBy, k.mul(mB, py))
    wB = k.add(wB, k.mul(iB, angB))
    scatter(k, 'storeA', [vAx, vAy, wA, flA, dpAx, dpAy, dqAs, dqAc])
    scatter(k, 'storeB', [vBx, vBy, wB, flB, dpBx, dpBy, dqBs, dqBc])


def build(fn, variant):
    k = Kernel(spill_base=SPILL_BASE)
    fn(k, variant)
    return k.emit()


def gen_func(name, sig, argsize, params, fn, select=None):
    """One TEXT symbol looping over n constraints. select emits the code
    that jumps to label <tag>_alt for the second variant; without it the
    kernel has one variant."""
    tag = name
    bodies = [build(fn, False)]
    if select:
        bodies.append(build(fn, True))
    frame = max(f for _, f in bodies)
    s = ['', '// func %s%s' % (name, sig), 'TEXT ·%s(SB), 0, $%d-%d' % (name, frame, argsize)]
    s += ['\tMOVQ c+0(FP), CX', '\tMOVQ n+8(FP), SI', '\tMOVQ states+16(FP), DI']
    for arg, slot in params:
        s += ['\tVBROADCASTSS %s, Y0' % arg, '\tVMOVUPS Y0, %s' % slot]
    s += ['\tLEAQ ·laneConstIdentityRow(SB), DX', '\tLEAQ 32(SP), R8',
          '\tTESTQ SI, SI', '\tJLE %s_done' % tag, '%s_loop:' % tag]
    if select:
        s += select(tag)
    s += bodies[0][0]
    if select:
        s += ['\tJMP %s_next' % tag, '%s_alt:' % tag]
        s += bodies[1][0]
    s += ['%s_next:' % tag, '\tADDQ $contactConstraintWide__size, CX', '\tDECQ SI',
          '\tJNE %s_loop' % tag, '%s_done:' % tag, '\tVZEROUPPER', '\tRET']
    return s


def select_rolling(tag):
    # The rolling variant when any lane of rollingResistance is nonzero, as
    # laneGroupMask; its active mask row goes to R9:
    # group = (bits&0x0f != 0) | (bits&0xf0 != 0)<<1.
    return [
            '\tVMOVUPS %s, Y0' % cf('rollingResistance'), '\tVXORPS Y1, Y1, Y1',
            '\tVCMPPS $4, Y1, Y0, Y0', '\tVMOVMSKPS Y0, AX', '\tTESTL AX, AX',
            '\tJE %s_plain' % tag,
            '\tXORL BX, BX', '\tTESTL $0x0f, AX', '\tSETNE BL',
            '\tXORL R9, R9', '\tTESTL $0xf0, AX', '\tSETNE R9B',
            '\tLEAQ (BX)(R9*2), R9', '\tSHLQ $5, R9',
            '\tLEAQ ·laneConstGroup(SB), BX', '\tADDQ BX, R9',
            '\tJMP %s_alt' % tag, '%s_plain:' % tag]


def select_has_rolling(tag):
    return ['\tCMPB %s, $0' % cf('hasRolling'), '\tJNE %s_alt' % tag]


def gen_relax():
    return gen_func('solveContactsWideAVX2',
                    '(c *contactConstraintWide, n int, states *bodyState, invH float32)', 28,
                    [('invH+24(FP)', INVH)], relax, select_rolling)


def gen_warm():
    return gen_func('warmStartContactsWideAVX2',
                    '(c *contactConstraintWide, n int, states *bodyState)', 24,
                    [], warm, select_has_rolling)


def gen_push():
    return gen_func('pushContactsWideAVX2',
                    '(c *contactConstraintWide, n int, states *bodyState, invH, negContactSpeed, one float32)', 36,
                    [('invH+24(FP)', INVH), ('negContactSpeed+28(FP)', PARAM1), ('one+32(FP)', PARAM2)],
                    push)


def main():
    lines = HEADER.split('\n') + gen_relax() + gen_warm() + gen_push()
    path = 'contact_solver_wide_amd64.s'
    with open(path, 'w', newline='', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')
    print('wrote', path, len(lines), 'lines')


if __name__ == '__main__':
    main()
