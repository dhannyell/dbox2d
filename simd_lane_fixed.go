//go:build dbox2d_simd && dbox2d_fixed

package dbox2d

import "github.com/dhannyell/fixed"

const (
	// wideWidth is the number of Q16 lanes fixed exposes on this path.
	wideWidth = fixed.LaneWidth
	// wideShift is log2 of wideWidth; fixed lanes are four or eight wide.
	wideShift = 2 + wideWidth/8
)

// Fails to compile unless wideShift matches wideWidth.
const (
	_ = uint(1<<wideShift - wideWidth)
	_ = uint(wideWidth - 1<<wideShift)
)

// laneScalar is the element type the lane loads and stores.
type laneScalar = fixed.Q16

// laneScalarFromQ rounds a scalar-mode value to the lane grid, to nearest like
// the scalar contact stages.
func laneScalarFromQ(q Q) laneScalar { return q.ToQ16Round() }

// laneScalarToQ widens a lane element back to a scalar-mode value.
func laneScalarToQ(s laneScalar) Q { return s.ToQ32() }

// accScalar is the element type the accumulator stores.
type accScalar = fixed.Q48

// accScalarFromQ rounds a scalar-mode value to the accumulator grid, to
// nearest like the scalar contact stages.
func accScalarFromQ(q Q) accScalar { return q.ToQ48Round() }

// accScalarToQ converts an accumulator element back to a scalar-mode value.
func accScalarToQ(s accScalar) Q { return s.ToQ32() }

// signedZeroSurvives is false because fixed lanes have one zero: a block that
// only adds zero can be skipped.
const signedZeroSurvives = false

// laneW carries constraint values; fixed selects the lane path per ISA.
type laneW struct {
	v fixed.Lane16
}

// maskW carries lane-wise comparison results.
type maskW struct {
	v fixed.Mask16
}

// accW carries velocities and impulse totals in Q48.16: the lane grid with
// more integer range.
type accW struct {
	v fixed.Lane48
}

// vec2W stores two wide vectors.
type vec2W struct {
	x, y laneW
}

// rotW stores a wide cosine and sine pair.
type rotW struct {
	c, s laneW
}

// wideAvailable reports whether the lane path may be used.
func wideAvailable() bool { return fixed.LanesAvailable() }

// widePath reports the lane implementation fixed compiled.
func widePath() string { return fixed.LanePath() }

// laneZero returns a lane filled with zero.
func laneZero() laneW { return laneW{v: fixed.SplatLane16(fixed.Q16Zero())} }

// laneSplat fills a lane with one scalar value.
func laneSplat(q Q) laneW { return laneW{v: fixed.SplatLane16(laneScalarFromQ(q))} }

// laneLoad loads one aligned-width lane-element array.
func laneLoad(p *[wideWidth]laneScalar) laneW { return laneW{v: fixed.LoadLane16(p)} }

// store writes the lane to one aligned-width lane-element array.
func (a laneW) store(p *[wideWidth]laneScalar) { a.v.Store(p) }

// Add returns the lane-wise sum, with Q16 saturation.
func (a laneW) Add(b laneW) laneW { return laneW{v: a.v.Add(b.v)} }

// Sub returns the lane-wise difference, with Q16 saturation.
func (a laneW) Sub(b laneW) laneW { return laneW{v: a.v.Sub(b.v)} }

// Neg returns the lane-wise negation. Fixed has one zero, so 0 - x is exact
// and avoids a widening multiply.
func (a laneW) Neg() laneW { return laneW{v: a.v.Neg()} }

// Mul returns the lane-wise product, rounded to nearest like the scalar
// contact stages.
func (a laneW) Mul(b laneW) laneW { return laneW{v: a.v.MulRound(b.v)} }

// MulAdd returns a plus the separately rounded product of b and c.
func (a laneW) MulAdd(b, c laneW) laneW { return a.Add(b.Mul(c)) }

// MulSub returns a minus the separately rounded product of b and c.
func (a laneW) MulSub(b, c laneW) laneW { return a.Sub(b.Mul(c)) }

// Min returns the lane-wise minimum.
func (a laneW) Min(b laneW) laneW { return laneW{v: a.v.Min(b.v)} }

// Max returns the lane-wise maximum.
func (a laneW) Max(b laneW) laneW { return laneW{v: a.v.Max(b.v)} }

// Greater compares corresponding lane values.
func (a laneW) Greater(b laneW) maskW { return maskW{v: a.v.Greater(b.v)} }

// Equals compares corresponding lane values for equality.
func (a laneW) Equals(b laneW) maskW { return maskW{v: a.v.Equals(b.v)} }

// Or returns the lane-wise mask union.
func (m maskW) Or(n maskW) maskW { return maskW{v: m.v.Or(n.v)} }

// AllZero reports whether no mask lane is set.
func (m maskW) AllZero() bool { return m.v.AllZero() }

// laneBlend selects a where the mask is set and b otherwise.
func laneBlend(m maskW, a, b laneW) laneW {
	return laneW{v: fixed.BlendLane16(m.v, a.v, b.v)}
}

// toAcc widens a lane into the accumulator grid. The conversion is exact.
func (a laneW) toAcc() accW { return accW{v: a.v.ToLane48()} }

// toLane narrows an accumulator back to a lane, with Q16 saturation.
func (a accW) toLane() laneW { return laneW{v: a.v.ToLane16()} }

// accLoad loads one aligned-width accumulator-element array.
func accLoad(p *[wideWidth]accScalar) accW { return accW{v: fixed.LoadLane48(p)} }

// store writes the accumulator without narrowing it.
func (a accW) store(p *[wideWidth]accScalar) { a.v.Store(p) }

// Add returns the lane-wise accumulator sum, with Q48 saturation.
func (a accW) Add(b accW) accW { return accW{v: a.v.Add(b.v)} }

// Sub returns the lane-wise accumulator difference, with Q48 saturation.
func (a accW) Sub(b accW) accW { return accW{v: a.v.Sub(b.v)} }

// AddBounded adds without the overflow check. Each stage reloads the
// velocities from the Q32 body state and adds a few Q16 terms, and a total
// gains a few per substep, so no sum nears the Q48 range.
func (a accW) AddBounded(b accW) accW { return accW{v: a.v.AddWrap(b.v)} }

// SubBounded subtracts under the same budget as AddBounded.
func (a accW) SubBounded(b accW) accW { return accW{v: a.v.SubWrap(b.v)} }

// rollingBoundW returns rr·total per lane through the scalar rollingBound:
// the lanes have no floor, and the split of the total must not saturate.
func rollingBoundW(rr laneW, total accW) laneW {
	var rrs [wideWidth]laneScalar
	var totals [wideWidth]accScalar
	var bounds [wideWidth]laneScalar
	rr.store(&rrs)
	total.store(&totals)
	for j := range wideWidth {
		bounds[j] = rollingBound(qc{rrs[j]}, qa{totals[j]}).v
	}
	return laneLoad(&bounds)
}

// bodyScratchW carries the gathered scalars between the body states and the
// lanes. The velocities travel in the accumulator grid, as in the scalar
// contact stages; only the deltas are narrowed to the lane grid.
type bodyScratchW struct {
	vx, vy, w          [wideWidth]accScalar
	dpx, dpy, dqc, dqs [wideWidth]laneScalar
}

// gatherBodies fills the scratch; null lanes get the identity. The tau scaling
// runs in Q32 before the rounding, as in the scalar contact stages.
func gatherBodies(states []bodyState, indices *[wideWidth]int, s *bodyScratchW) {
	zero := fixed.Q16Zero()
	for j := range wideWidth {
		idx := indices[j]
		if idx == nullIndex {
			s.vx[j], s.vy[j], s.w[j] = fixed.Q48Zero(), fixed.Q48Zero(), fixed.Q48Zero()
			s.dpx[j], s.dpy[j], s.dqs[j] = zero, zero, zero
			s.dqc[j] = fixed.Q16One()
			continue
		}
		b := &states[idx]
		s.vx[j] = accScalarFromQ(b.linearVelocity.X)
		s.vy[j] = accScalarFromQ(b.linearVelocity.Y)
		s.w[j] = accScalarFromQ(b.angularVelocity.Mul(tau))
		s.dpx[j] = laneScalarFromQ(b.deltaPosition.X)
		s.dpy[j] = laneScalarFromQ(b.deltaPosition.Y)
		s.dqc[j] = laneScalarFromQ(b.deltaRotation.Cos)
		s.dqs[j] = laneScalarFromQ(b.deltaRotation.Sin)
	}
}

// loadBodyVelocityW builds the velocity accumulators from the scratch.
func loadBodyVelocityW(s *bodyScratchW, b *bodyStateW) {
	b.v.x = accLoad(&s.vx)
	b.v.y = accLoad(&s.vy)
	b.w = accLoad(&s.w)
}

// loadBodyDeltaW builds the delta position and delta rotation lanes.
func loadBodyDeltaW(s *bodyScratchW, b *bodyStateW) {
	b.dp = vec2W{x: laneLoad(&s.dpx), y: laneLoad(&s.dpy)}
	b.dq = rotW{c: laneLoad(&s.dqc), s: laneLoad(&s.dqs)}
}

// storeBodyW writes the velocity accumulators to the scratch.
func storeBodyW(b *bodyStateW, s *bodyScratchW) {
	b.v.x.store(&s.vx)
	b.v.y.store(&s.vy)
	b.w.store(&s.w)
}

// scatterBodies writes the real-lane velocities back. The tau division runs
// in Q32, as in the scalar store.
func scatterBodies(states []bodyState, indices *[wideWidth]int, s *bodyScratchW) {
	for j := range wideWidth {
		idx := indices[j]
		if idx == nullIndex {
			continue
		}
		b := &states[idx]
		b.linearVelocity.X = accScalarToQ(s.vx[j])
		b.linearVelocity.Y = accScalarToQ(s.vy[j])
		b.angularVelocity = accScalarToQ(s.w[j]).Div(tau)
	}
}

// gatherBodyW loads one constraint's bodies. tauW is unused: gatherBodies
// already scaled by tau.
func gatherBodyW(states []bodyState, indices *[wideWidth]int, tauW laneW, b *bodyStateW) {
	var scratch bodyScratchW
	gatherBodies(states, indices, &scratch)
	loadBodyVelocityW(&scratch, b)
	loadBodyDeltaW(&scratch, b)
	b.flags = laneZero()
}

// scatterBodyW writes one constraint's velocities back. tauW is unused:
// scatterBodies divides by tau.
func scatterBodyW(states []bodyState, indices *[wideWidth]int, tauW laneW, b *bodyStateW) {
	var scratch bodyScratchW
	storeBodyW(b, &scratch)
	scatterBodies(states, indices, &scratch)
}
