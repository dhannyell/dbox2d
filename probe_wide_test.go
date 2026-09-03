package dbox2d

import (
	"github.com/dhannyell/fixed"
)

// The wide row runs the lanes row of probe_q16_test.go through the batch
// functions of fixed. The constraints are colored so that no two in one
// color share a dynamic body, as the graph coloring of the library does;
// each color then runs as slices, one lane per constraint, with the
// bodies gathered before and scattered after. The batch functions floor
// each product, so the row uses the floor lanes. Nothing here enters the
// library.
//
// The wide solver has no branch per lane. The speculative test s > 0
// becomes an indicator built from clamps, and the friction bound, which
// differs per lane, becomes two clamps around a subtraction.

type q16 = fixed.Q16

// wideConstant is the scale that turns any positive Q16 into a value
// above one half in one product, so a second clamp yields exactly one.
var wideConstant = fixed.Q16MaxValue()

// wideSlab holds the per-point lanes of one manifold point index.
type wideSlab struct {
	rAx, rAy, rBx, rBy []q16
	base               []q16
	negNMass, negTMass []q16
	ni, ti             []q16
}

// probeWide holds the lanes of one step. Lane i is constraint order[i];
// color c is lanes [colors[c], colors[c+1]).
type probeWide struct {
	order  []int
	colors []int
	used   [][]bool

	// Per constraint.
	nx, ny, tx, ty      []q16
	mA, iA, mB, iB      []q16
	friction            []q16
	biasRate            []q16
	softMassLoss        []q16
	softImpulse         []q16
	pts                 [2]wideSlab
	vAx, vAy, wA        []q16
	vBx, vBy, wB        []q16
	vAx0, vAy0, wA0     []q16
	vBx0, vBy0, wB0     []q16
	dpx, dpy            []q16
	dqAc, dqAs          []q16
	dqBc, dqBs          []q16
	ones, zeros, ulps   []q16
	ks, invHs           []q16
	t                   [8][]q16
	vrAx, vrAy          []q16
	vrBx, vrBy          []q16
	dvx, dvy, vn        []q16
	s, bias             []q16
	massScale, impScale []q16
	imp, Px, Py, maxF   []q16
}

func (w *probeWide) grow(n int) {
	if len(w.order) >= n {
		return
	}
	w.order = make([]int, n)
	slabs := []*[]q16{
		&w.nx, &w.ny, &w.tx, &w.ty, &w.mA, &w.iA, &w.mB, &w.iB, &w.friction,
		&w.biasRate, &w.softMassLoss, &w.softImpulse,
		&w.vAx, &w.vAy, &w.wA, &w.vBx, &w.vBy, &w.wB,
		&w.vAx0, &w.vAy0, &w.wA0, &w.vBx0, &w.vBy0, &w.wB0,
		&w.dpx, &w.dpy, &w.dqAc, &w.dqAs, &w.dqBc, &w.dqBs,
		&w.ones, &w.zeros, &w.ulps, &w.ks, &w.invHs,
		&w.vrAx, &w.vrAy, &w.vrBx, &w.vrBy, &w.dvx, &w.dvy, &w.vn,
		&w.s, &w.bias, &w.massScale, &w.impScale, &w.imp, &w.Px, &w.Py, &w.maxF,
	}
	for j := range w.pts {
		sl := &w.pts[j]
		slabs = append(slabs, &sl.rAx, &sl.rAy, &sl.rBx, &sl.rBy, &sl.base, &sl.negNMass, &sl.negTMass, &sl.ni, &sl.ti)
	}
	for i := range w.t {
		slabs = append(slabs, &w.t[i])
	}
	for _, s := range slabs {
		*s = make([]q16, n)
	}
}

// wideBuild colors the prepared constraints and fills the lanes.
func (p *probePyramid) wideBuild(count int) {
	if p.f.fracBits != 16 {
		panic("the wide row needs the Q16 grid")
	}
	w := &p.wide
	w.grow(count)
	for c := range w.used {
		clear(w.used[c])
	}
	colorOf := w.t[0][:count] // scratch, holds the color index as raw
	for i := range count {
		cs := &p.constraints[i]
		color := 0
		for {
			if color == len(w.used) {
				w.used = append(w.used, make([]bool, len(p.states)))
			}
			free := (cs.indexA < 0 || !w.used[color][cs.indexA]) && (cs.indexB < 0 || !w.used[color][cs.indexB])
			if free {
				break
			}
			color++
		}
		if cs.indexA >= 0 {
			w.used[color][cs.indexA] = true
		}
		if cs.indexB >= 0 {
			w.used[color][cs.indexB] = true
		}
		colorOf[i] = fixed.Q16FromRaw(int32(color))
	}

	w.colors = w.colors[:0]
	for range len(w.used) + 1 {
		w.colors = append(w.colors, 0)
	}
	for i := range count {
		w.colors[colorOf[i].Raw()+1]++
	}
	for c := 1; c < len(w.colors); c++ {
		w.colors[c] += w.colors[c-1]
	}
	fill := w.t[1][:len(w.colors)]
	for c := range w.colors {
		fill[c] = fixed.Q16FromRaw(int32(w.colors[c]))
	}
	for i := range count {
		c := colorOf[i].Raw()
		lane := fill[c].Raw()
		fill[c] = fixed.Q16FromRaw(lane + 1)
		w.order[lane] = i
	}

	q := fixed.Q16FromRaw
	for lane := range count {
		cs := &p.constraints[w.order[lane]]
		w.nx[lane], w.ny[lane] = q(cs.normal.x), q(cs.normal.y)
		w.tx[lane], w.ty[lane] = q(cs.normal.y), q(-cs.normal.x)
		w.mA[lane], w.iA[lane] = q(cs.invMassA), q(cs.invIA)
		w.mB[lane], w.iB[lane] = q(cs.invMassB), q(cs.invIB)
		w.friction[lane] = q(cs.friction)
		w.biasRate[lane] = q(cs.soft.biasRate)
		w.softMassLoss[lane] = q(p.f.one - cs.soft.massScale)
		w.softImpulse[lane] = q(cs.soft.impulseScale)
		w.ones[lane], w.zeros[lane], w.ulps[lane] = fixed.Q16One(), fixed.Q16Zero(), q(1)
		w.ks[lane], w.invHs[lane] = wideConstant, q(p.invH)
		for j := range w.pts {
			sl := &w.pts[j]
			if j >= cs.pointCount {
				// An absent point has no mass, so its lane applies nothing.
				sl.rAx[lane], sl.rAy[lane], sl.rBx[lane], sl.rBy[lane] = q(0), q(0), q(0), q(0)
				sl.base[lane], sl.negNMass[lane], sl.negTMass[lane] = q(0), q(0), q(0)
				sl.ni[lane], sl.ti[lane] = q(0), q(0)
				continue
			}
			cp := &cs.points[j]
			sl.rAx[lane], sl.rAy[lane] = q(cp.rA.x), q(cp.rA.y)
			sl.rBx[lane], sl.rBy[lane] = q(cp.rB.x), q(cp.rB.y)
			sl.base[lane] = q(cp.baseSeparation)
			sl.negNMass[lane], sl.negTMass[lane] = q(-cp.normalMass), q(-cp.tangentMass)
			sl.ni[lane], sl.ti[lane] = cp.ni32.ToQ16(), cp.ti32.ToQ16()
		}
	}
}

// wideStore writes the impulse sums back to the constraints for store.
func (p *probePyramid) wideStore(count int) {
	w := &p.wide
	for lane := range count {
		cs := &p.constraints[w.order[lane]]
		for j := range cs.pointCount {
			cp := &cs.points[j]
			cp.ni32 = w.pts[j].ni[lane].ToQ32()
			cp.ti32 = w.pts[j].ti[lane].ToQ32()
			cp.tni32 = fixed.Q32Zero()
		}
	}
}

// wideGather reads the bodies of one color as the floor lanes row does.
func (p *probePyramid) wideGather(lo, hi int, pose bool) {
	w := &p.wide
	tau := fixed.Q16FromRaw(p.tau)
	for lane := lo; lane < hi; lane++ {
		cs := &p.constraints[w.order[lane]]
		stateA, stateB := p.stateOf(cs.indexA), p.stateOf(cs.indexB)
		w.vAx[lane], w.vAy[lane] = stateA.v32.X.ToQ16(), stateA.v32.Y.ToQ16()
		w.wA[lane] = stateA.w32.ToQ16().Mul(tau)
		w.vBx[lane], w.vBy[lane] = stateB.v32.X.ToQ16(), stateB.v32.Y.ToQ16()
		w.wB[lane] = stateB.w32.ToQ16().Mul(tau)
		if pose {
			dp := stateB.dp32.Sub(stateA.dp32)
			w.dpx[lane], w.dpy[lane] = dp.X.ToQ16(), dp.Y.ToQ16()
			w.dqAc[lane], w.dqAs[lane] = stateA.dq32.Cos.ToQ16(), stateA.dq32.Sin.ToQ16()
			w.dqBc[lane], w.dqBs[lane] = stateB.dq32.Cos.ToQ16(), stateB.dq32.Sin.ToQ16()
		}
	}
	copy(w.vAx0[lo:hi], w.vAx[lo:hi])
	copy(w.vAy0[lo:hi], w.vAy[lo:hi])
	copy(w.wA0[lo:hi], w.wA[lo:hi])
	copy(w.vBx0[lo:hi], w.vBx[lo:hi])
	copy(w.vBy0[lo:hi], w.vBy[lo:hi])
	copy(w.wB0[lo:hi], w.wB[lo:hi])
}

// wideScatter adds the velocity deltas of one color to the Q32 state.
func (p *probePyramid) wideScatter(lo, hi int) {
	w := &p.wide
	tau := fixed.Q16FromRaw(p.tau)
	for lane := lo; lane < hi; lane++ {
		cs := &p.constraints[w.order[lane]]
		stateA, stateB := p.stateOf(cs.indexA), p.stateOf(cs.indexB)
		stateA.v32 = stateA.v32.Add(Vec2{X: w.vAx[lane].Sub(w.vAx0[lane]).ToQ32(), Y: w.vAy[lane].Sub(w.vAy0[lane]).ToQ32()})
		stateA.w32 = stateA.w32.Add(w.wA[lane].Sub(w.wA0[lane]).Div(tau).ToQ32())
		stateB.v32 = stateB.v32.Add(Vec2{X: w.vBx[lane].Sub(w.vBx0[lane]).ToQ32(), Y: w.vBy[lane].Sub(w.vBy0[lane]).ToQ32()})
		stateB.w32 = stateB.w32.Add(w.wB[lane].Sub(w.wB0[lane]).Div(tau).ToQ32())
	}
}

// wideApply adds the impulse P of one point to both bodies.
func (w *probeWide) wideApply(lo, hi int, sl *wideSlab) {
	add, sub, mul := fixed.BatchAdd16, fixed.BatchSub16, fixed.BatchMul16
	r := func(s []q16) []q16 { return s[lo:hi] }
	t0, t1 := r(w.t[0]), r(w.t[1])
	Px, Py := r(w.Px), r(w.Py)

	mul(t0, r(w.mA), Px)
	sub(r(w.vAx), r(w.vAx), t0)
	mul(t0, r(w.mA), Py)
	sub(r(w.vAy), r(w.vAy), t0)
	mul(t0, r(sl.rAx), Py)
	mul(t1, r(sl.rAy), Px)
	sub(t0, t0, t1)
	mul(t0, r(w.iA), t0)
	sub(r(w.wA), r(w.wA), t0)

	mul(t0, r(w.mB), Px)
	add(r(w.vBx), r(w.vBx), t0)
	mul(t0, r(w.mB), Py)
	add(r(w.vBy), r(w.vBy), t0)
	mul(t0, r(sl.rBx), Py)
	mul(t1, r(sl.rBy), Px)
	sub(t0, t0, t1)
	mul(t0, r(w.iB), t0)
	add(r(w.wB), r(w.wB), t0)
}

// wideRelative computes the relative velocity of one point at the anchors.
func (w *probeWide) wideRelative(lo, hi int, sl *wideSlab) {
	add, sub, mul := fixed.BatchAdd16, fixed.BatchSub16, fixed.BatchMul16
	r := func(s []q16) []q16 { return s[lo:hi] }
	t0 := r(w.t[0])
	mul(t0, r(w.wA), r(sl.rAy))
	sub(r(w.vrAx), r(w.vAx), t0)
	mul(t0, r(w.wA), r(sl.rAx))
	add(r(w.vrAy), r(w.vAy), t0)
	mul(t0, r(w.wB), r(sl.rBy))
	sub(r(w.vrBx), r(w.vBx), t0)
	mul(t0, r(w.wB), r(sl.rBx))
	add(r(w.vrBy), r(w.vBy), t0)
	sub(r(w.dvx), r(w.vrBx), r(w.vrAx))
	sub(r(w.dvy), r(w.vrBy), r(w.vrAy))
}

// wideWarmStart mirrors warmStart over the colors.
func (p *probePyramid) wideWarmStart() {
	w := &p.wide
	add, mul := fixed.BatchAdd16, fixed.BatchMul16
	for c := 0; c+1 < len(w.colors); c++ {
		lo, hi := w.colors[c], w.colors[c+1]
		if lo == hi {
			continue
		}
		r := func(s []q16) []q16 { return s[lo:hi] }
		p.wideGather(lo, hi, false)
		for j := range w.pts {
			sl := &w.pts[j]
			t0 := r(w.t[0])
			mul(r(w.Px), r(w.nx), r(sl.ni))
			mul(t0, r(w.tx), r(sl.ti))
			add(r(w.Px), r(w.Px), t0)
			mul(r(w.Py), r(w.ny), r(sl.ni))
			mul(t0, r(w.ty), r(sl.ti))
			add(r(w.Py), r(w.Py), t0)
			w.wideApply(lo, hi, sl)
		}
		p.wideScatter(lo, hi)
	}
}

// wideSolve mirrors solve over the colors, with and without the bias.
func (p *probePyramid) wideSolve(useBias bool) {
	w := &p.wide
	add, sub, mul, clamp := fixed.BatchAdd16, fixed.BatchSub16, fixed.BatchMul16, fixed.BatchClamp16
	zero, one := fixed.Q16Zero(), fixed.Q16One()
	minV, maxV := fixed.Q16MinValue(), fixed.Q16MaxValue()
	pushout := fixed.Q16FromRaw(p.pushout)
	for c := 0; c+1 < len(w.colors); c++ {
		lo, hi := w.colors[c], w.colors[c+1]
		if lo == hi {
			continue
		}
		r := func(s []q16) []q16 { return s[lo:hi] }
		t0, t1, t2, t3, t4 := r(w.t[0]), r(w.t[1]), r(w.t[2]), r(w.t[3]), r(w.t[4])
		s, bias, massScale, impScale := r(w.s), r(w.bias), r(w.massScale), r(w.impScale)
		imp, Px, Py, maxF := r(w.imp), r(w.Px), r(w.Py), r(w.maxF)
		p.wideGather(lo, hi, true)

		for j := range w.pts {
			sl := &w.pts[j]

			// Separation at the current sub-step pose.
			mul(t0, r(w.dqBc), r(sl.rBx))
			mul(t1, r(w.dqBs), r(sl.rBy))
			sub(t0, t0, t1)
			mul(t1, r(w.dqBs), r(sl.rBx))
			mul(t2, r(w.dqBc), r(sl.rBy))
			add(t1, t1, t2)
			mul(t2, r(w.dqAc), r(sl.rAx))
			mul(t3, r(w.dqAs), r(sl.rAy))
			sub(t2, t2, t3)
			mul(t3, r(w.dqAs), r(sl.rAx))
			mul(t4, r(w.dqAc), r(sl.rAy))
			add(t3, t3, t4)
			add(t0, r(w.dpx), t0)
			sub(t0, t0, t2)
			add(t1, r(w.dpy), t1)
			sub(t1, t1, t3)
			mul(t0, t0, r(w.nx))
			mul(t1, t1, r(w.ny))
			add(t0, t0, t1)
			add(s, r(sl.base), t0)

			// Speculative bias when s > 0; soft bias otherwise.
			mul(bias, s, r(w.invHs))
			clamp(bias, bias, zero, maxV)
			if useBias {
				sub(t0, r(w.ulps), s)
				clamp(t0, t0, zero, one)
				mul(t0, t0, r(w.ks))
				clamp(t0, t0, zero, one)
				mul(t0, t0, r(w.ks))
				clamp(t0, t0, zero, one)
				mul(t1, t0, r(w.softMassLoss))
				sub(massScale, r(w.ones), t1)
				mul(impScale, t0, r(w.softImpulse))
				mul(t1, r(w.biasRate), s)
				clamp(t1, t1, pushout.Neg(), zero)
				add(bias, bias, t1)
			} else {
				copy(massScale, r(w.ones))
				copy(impScale, r(w.zeros))
			}

			w.wideRelative(lo, hi, sl)
			mul(t0, r(w.dvx), r(w.nx))
			mul(t1, r(w.dvy), r(w.ny))
			add(r(w.vn), t0, t1)

			add(t0, r(w.vn), bias)
			mul(t1, r(sl.negNMass), massScale)
			mul(t0, t1, t0)
			mul(t1, impScale, r(sl.ni))
			sub(imp, t0, t1)
			add(t0, r(sl.ni), imp)
			clamp(t0, t0, zero, maxV)
			sub(imp, t0, r(sl.ni))
			copy(r(sl.ni), t0)

			mul(Px, r(w.nx), imp)
			mul(Py, r(w.ny), imp)
			w.wideApply(lo, hi, sl)
		}

		for j := range w.pts {
			sl := &w.pts[j]
			w.wideRelative(lo, hi, sl)
			mul(t0, r(w.dvx), r(w.tx))
			mul(t1, r(w.dvy), r(w.ty))
			add(t0, t0, t1)
			mul(imp, r(sl.negTMass), t0)

			// Clamp to [-maxF, maxF] with a bound that differs per lane.
			mul(maxF, r(w.friction), r(sl.ni))
			add(t0, r(sl.ti), imp)
			sub(t1, t0, maxF)
			clamp(t1, t1, minV, zero)
			add(t0, maxF, t1)
			add(t1, t0, maxF)
			clamp(t1, t1, zero, maxV)
			sub(t0, t1, maxF)
			sub(imp, t0, r(sl.ti))
			copy(r(sl.ti), t0)

			mul(Px, r(w.tx), imp)
			mul(Py, r(w.ty), imp)
			w.wideApply(lo, hi, sl)
		}

		p.wideScatter(lo, hi)
	}
}
