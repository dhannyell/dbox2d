//go:build dbox2d_simd && !dbox2d_fixed && goexperiment.simd && go1.27 && (amd64 || arm64 || wasm)

package dbox2d

// The archsimd float paths differ only in the vector width and in a few
// constructors; the per-architecture files supply those, and this file holds
// the lane operations over laneData.

// laneW is the float32 SIMD lane type.
type laneW struct {
	v laneData
}

// maskW is the SIMD comparison-mask type.
type maskW struct {
	v maskData
}

// laneZero returns a lane filled with positive zero.
func laneZero() laneW { return laneW{v: laneBroadcast(0)} }

// laneSplat fills a lane with a scalar float32 value.
func laneSplat(q Q) laneW { return laneW{v: laneBroadcast(laneScalarFromQ(q))} }

// laneLoad loads one aligned-width lane-element array.
func laneLoad(p *[wideWidth]laneScalar) laneW { return laneW{v: laneLoadData(p)} }

// store writes the lane to one aligned-width lane-element array.
func (a laneW) store(p *[wideWidth]laneScalar) { a.v.StoreArray(p) }

// Add returns the lane-wise sum.
func (a laneW) Add(b laneW) laneW { return laneW{v: a.v.Add(b.v)} }

// Sub returns the lane-wise difference.
func (a laneW) Sub(b laneW) laneW { return laneW{v: a.v.Sub(b.v)} }

// Neg returns the lane-wise negation. It multiplies by -1 so a zero flips its
// sign, as the scalar Neg does; 0 - x would not.
func (a laneW) Neg() laneW { return laneW{v: a.v.Mul(laneBroadcast(-1))} }

// Mul returns the lane-wise product.
func (a laneW) Mul(b laneW) laneW { return laneW{v: a.v.Mul(b.v)} }

// Div returns the lane-wise quotient, rounded once like the scalar division.
func (a laneW) Div(b laneW) laneW { return laneW{v: a.v.Div(b.v)} }

// MulAdd returns a plus the separately rounded product of b and c.
func (a laneW) MulAdd(b, c laneW) laneW { return a.Add(b.Mul(c)) }

// MulSub returns a minus the separately rounded product of b and c.
func (a laneW) MulSub(b, c laneW) laneW { return a.Sub(b.Mul(c)) }

// Min returns the lane-wise minimum, selecting b on equality.
// The hardware min/max do not honor the tie rule on signed zeros; compare and select do.
func (a laneW) Min(b laneW) laneW { less := b.v.Greater(a.v); return laneW{v: a.v.IfElse(less, b.v)} }

// Max returns the lane-wise maximum, selecting b on equality.
// The hardware min/max do not honor the tie rule on signed zeros; compare and select do.
func (a laneW) Max(b laneW) laneW {
	greater := a.v.Greater(b.v)
	return laneW{v: a.v.IfElse(greater, b.v)}
}

// Greater compares corresponding lane values.
func (a laneW) Greater(b laneW) maskW { return maskW{v: a.v.Greater(b.v)} }

// Equals compares corresponding lane values for equality.
func (a laneW) Equals(b laneW) maskW { return maskW{v: a.v.Equal(b.v)} }

// Or returns the lane-wise mask union.
func (m maskW) Or(n maskW) maskW { return maskW{v: m.v.Or(n.v)} }

// laneBlend selects a where the mask is set and b otherwise.
func laneBlend(m maskW, a, b laneW) laneW {
	return laneW{v: a.v.IfElse(m.v, b.v)}
}

// toAcc preserves a lane while naming the accumulation conversion.
func (a laneW) toAcc() laneW { return a }

// toLane preserves a lane while naming the lane conversion.
func (a laneW) toLane() laneW { return a }
