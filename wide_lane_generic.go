//go:build dbox2d_wide && dbox2d_float && (!goexperiment.simd || !go1.27 || (!amd64 && !arm64))

package dbox2d

const (
	// wideWidth is the number of float32 lanes in the generic vector.
	wideWidth = 4
	// wideShift is the base-two logarithm of wideWidth.
	wideShift = 2
)

// laneW is the pure-Go float32 lane type.
type laneW struct {
	v [4]float32
}

// maskW is the pure-Go comparison-mask type.
type maskW struct {
	v [4]bool
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

// wideAvailable reports that the generic path is available.
func wideAvailable() bool { return true }

// widePath reports the selected generic path.
func widePath() string { return "generic" }

// laneZero returns a lane filled with positive zero.
func laneZero() laneW { return laneW{} }

// laneSplat fills a lane with a scalar float32 value.
func laneSplat(q Q) laneW {
	var a laneW
	for i := range a.v {
		a.v[i] = q.v
	}
	return a
}

// laneLoad loads one aligned-width float32 array.
func laneLoad(p *[wideWidth]float32) laneW { return laneW{v: *p} }

// store writes the lane to one aligned-width float32 array.
func (a laneW) store(p *[wideWidth]float32) { *p = a.v }

// Add returns the lane-wise sum.
func (a laneW) Add(b laneW) laneW {
	var out laneW
	for i := range out.v {
		out.v[i] = a.v[i] + b.v[i]
	}
	return out
}

// Sub returns the lane-wise difference.
func (a laneW) Sub(b laneW) laneW {
	var out laneW
	for i := range out.v {
		out.v[i] = a.v[i] - b.v[i]
	}
	return out
}

// Mul returns the lane-wise product.
func (a laneW) Mul(b laneW) laneW {
	var out laneW
	for i := range out.v {
		out.v[i] = a.v[i] * b.v[i]
	}
	return out
}

// MulAdd returns a plus the separately rounded product of b and c.
func (a laneW) MulAdd(b, c laneW) laneW { return a.Add(b.Mul(c)) }

// MulSub returns a minus the separately rounded product of b and c.
func (a laneW) MulSub(b, c laneW) laneW { return a.Sub(b.Mul(c)) }

// Min returns the lane-wise minimum, selecting b on equality.
func (a laneW) Min(b laneW) laneW {
	var out laneW
	for i := range out.v {
		if a.v[i] < b.v[i] {
			out.v[i] = a.v[i]
		} else {
			out.v[i] = b.v[i]
		}
	}
	return out
}

// Max returns the lane-wise maximum, selecting b on equality.
func (a laneW) Max(b laneW) laneW {
	var out laneW
	for i := range out.v {
		if a.v[i] > b.v[i] {
			out.v[i] = a.v[i]
		} else {
			out.v[i] = b.v[i]
		}
	}
	return out
}

// Greater compares corresponding lane values.
func (a laneW) Greater(b laneW) maskW {
	var out maskW
	for i := range out.v {
		out.v[i] = a.v[i] > b.v[i]
	}
	return out
}

// Equals compares corresponding lane values for equality.
func (a laneW) Equals(b laneW) maskW {
	var out maskW
	for i := range out.v {
		out.v[i] = a.v[i] == b.v[i]
	}
	return out
}

// Or returns the lane-wise mask union.
func (m maskW) Or(n maskW) maskW {
	var out maskW
	for i := range out.v {
		out.v[i] = m.v[i] || n.v[i]
	}
	return out
}

// AllZero reports whether no mask lane is set.
func (m maskW) AllZero() bool {
	for _, set := range m.v {
		if set {
			return false
		}
	}
	return true
}

// laneBlend selects a where the mask is set and b otherwise.
func laneBlend(m maskW, a, b laneW) laneW {
	var out laneW
	for i, set := range m.v {
		if set {
			out.v[i] = a.v[i]
		} else {
			out.v[i] = b.v[i]
		}
	}
	return out
}

// toAcc preserves a lane while naming the accumulation conversion.
func (a laneW) toAcc() laneW { return a }

// toLane preserves a lane while naming the lane conversion.
func (a laneW) toLane() laneW { return a }
