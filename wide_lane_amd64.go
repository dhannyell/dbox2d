//go:build dbox2d_simd && dbox2d_float && goexperiment.simd && go1.27 && amd64

package dbox2d

import "simd/archsimd"

const (
	// wideWidth is the number of float32 lanes in the SIMD vector.
	wideWidth = 8
	// wideShift is the base-two logarithm of wideWidth.
	wideShift = 3
)

// laneW is the amd64 float32 SIMD lane type.
type laneW struct {
	v archsimd.Float32x8
}

// maskW is the amd64 SIMD comparison-mask type.
type maskW struct {
	v archsimd.Mask32x8
}

// accW is the accumulation lane; on this path it is the same type as laneW.
type accW = laneW

// vec2W stores two wide vectors.
type vec2W struct {
	x, y laneW
}

// rotW stores a wide cosine and sine pair.
type rotW struct {
	c, s laneW
}

// wideAvailable reports whether the required amd64 feature is available.
func wideAvailable() bool { return archsimd.X86.AVX2() }

// widePath reports the selected amd64 SIMD path.
func widePath() string { return "avx2" }

// laneZero returns a lane filled with positive zero.
func laneZero() laneW { return laneW{v: archsimd.BroadcastFloat32x8(0)} }

// laneSplat fills a lane with a scalar float32 value.
func laneSplat(q Q) laneW {
	return laneW{v: archsimd.BroadcastFloat32x8(q.v)}
}

// laneLoad loads one aligned-width float32 array.
func laneLoad(p *[wideWidth]float32) laneW {
	return laneW{v: archsimd.LoadFloat32x8Array(p)}
}

// store writes the lane to one aligned-width float32 array.
func (a laneW) store(p *[wideWidth]float32) { a.v.StoreArray(p) }

// Add returns the lane-wise sum.
func (a laneW) Add(b laneW) laneW { return laneW{v: a.v.Add(b.v)} }

// Sub returns the lane-wise difference.
func (a laneW) Sub(b laneW) laneW { return laneW{v: a.v.Sub(b.v)} }

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

// AllZero reports whether no mask lane is set.
func (m maskW) AllZero() bool { return m.v.ToBits() == 0 }

// laneBlend selects a where the mask is set and b otherwise.
func laneBlend(m maskW, a, b laneW) laneW {
	return laneW{v: a.v.IfElse(m.v, b.v)}
}

// toAcc preserves a lane while naming the accumulation conversion.
func (a laneW) toAcc() laneW { return a }

// toLane preserves a lane while naming the lane conversion.
func (a laneW) toLane() laneW { return a }
