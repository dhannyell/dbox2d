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
