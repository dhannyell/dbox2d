//go:build dbox2d_simd && dbox2d_float && (!goexperiment.simd || !go1.27 || (!amd64 && !arm64 && !wasm))

package dbox2d

const (
	// wideWidth is the number of float32 lanes in the generic vector.
	wideWidth = 4
	// wideShift is the base-two logarithm of wideWidth.
	wideShift = 2
)

// laneScalar is the element type the lane loads and stores.
type laneScalar = float32

// laneScalarFromQ converts a scalar-mode value to a lane element.
func laneScalarFromQ(q Q) laneScalar { return q.v }

// laneScalarToQ converts a lane element back to a scalar-mode value.
func laneScalarToQ(s laneScalar) Q { return Q{v: s} }

// signedZeroSurvives is true because float lanes keep a sign on zero:
// skipping a block that only adds zero could change a stored bit.
const signedZeroSurvives = true

// laneW is the pure-Go float32 lane type.
type laneW struct {
	l0, l1, l2, l3 laneScalar
}

// maskW is the pure-Go comparison-mask type.
type maskW struct {
	m0, m1, m2, m3 bool
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
	s := laneScalarFromQ(q)
	return laneW{l0: s, l1: s, l2: s, l3: s}
}

// laneLoad loads one aligned-width lane-element array.
func laneLoad(p *[wideWidth]laneScalar) laneW {
	return laneW{l0: p[0], l1: p[1], l2: p[2], l3: p[3]}
}

// store writes the lane to one aligned-width lane-element array.
func (a laneW) store(p *[wideWidth]laneScalar) {
	p[0], p[1], p[2], p[3] = a.l0, a.l1, a.l2, a.l3
}

// Add returns the lane-wise sum.
func (a laneW) Add(b laneW) laneW {
	return laneW{l0: a.l0 + b.l0, l1: a.l1 + b.l1, l2: a.l2 + b.l2, l3: a.l3 + b.l3}
}

// Sub returns the lane-wise difference.
func (a laneW) Sub(b laneW) laneW {
	return laneW{l0: a.l0 - b.l0, l1: a.l1 - b.l1, l2: a.l2 - b.l2, l3: a.l3 - b.l3}
}

// Neg returns the lane-wise negation. It multiplies by -1 so a zero flips its
// sign, as the scalar Neg does; 0 - x would not.
func (a laneW) Neg() laneW {
	return laneW{
		l0: laneScalar(-1 * a.l0),
		l1: laneScalar(-1 * a.l1),
		l2: laneScalar(-1 * a.l2),
		l3: laneScalar(-1 * a.l3),
	}
}

// Mul returns the lane-wise product. The conversions round each product
// explicitly, which forbids the compiler to fuse it with a later add.
func (a laneW) Mul(b laneW) laneW {
	return laneW{
		l0: laneScalar(a.l0 * b.l0),
		l1: laneScalar(a.l1 * b.l1),
		l2: laneScalar(a.l2 * b.l2),
		l3: laneScalar(a.l3 * b.l3),
	}
}

// Div returns the lane-wise quotient, rounded once like the scalar division.
func (a laneW) Div(b laneW) laneW {
	return laneW{l0: a.l0 / b.l0, l1: a.l1 / b.l1, l2: a.l2 / b.l2, l3: a.l3 / b.l3}
}

// MulAdd returns a plus the separately rounded product of b and c.
func (a laneW) MulAdd(b, c laneW) laneW { return a.Add(b.Mul(c)) }

// MulSub returns a minus the separately rounded product of b and c.
func (a laneW) MulSub(b, c laneW) laneW { return a.Sub(b.Mul(c)) }

// Min returns the lane-wise minimum, selecting b on equality.
func (a laneW) Min(b laneW) laneW {
	var out laneW
	if a.l0 < b.l0 {
		out.l0 = a.l0
	} else {
		out.l0 = b.l0
	}
	if a.l1 < b.l1 {
		out.l1 = a.l1
	} else {
		out.l1 = b.l1
	}
	if a.l2 < b.l2 {
		out.l2 = a.l2
	} else {
		out.l2 = b.l2
	}
	if a.l3 < b.l3 {
		out.l3 = a.l3
	} else {
		out.l3 = b.l3
	}
	return out
}

// Max returns the lane-wise maximum, selecting b on equality.
func (a laneW) Max(b laneW) laneW {
	var out laneW
	if a.l0 > b.l0 {
		out.l0 = a.l0
	} else {
		out.l0 = b.l0
	}
	if a.l1 > b.l1 {
		out.l1 = a.l1
	} else {
		out.l1 = b.l1
	}
	if a.l2 > b.l2 {
		out.l2 = a.l2
	} else {
		out.l2 = b.l2
	}
	if a.l3 > b.l3 {
		out.l3 = a.l3
	} else {
		out.l3 = b.l3
	}
	return out
}

// Greater compares corresponding lane values.
func (a laneW) Greater(b laneW) maskW {
	return maskW{m0: a.l0 > b.l0, m1: a.l1 > b.l1, m2: a.l2 > b.l2, m3: a.l3 > b.l3}
}

// Equals compares corresponding lane values for equality.
func (a laneW) Equals(b laneW) maskW {
	return maskW{m0: a.l0 == b.l0, m1: a.l1 == b.l1, m2: a.l2 == b.l2, m3: a.l3 == b.l3}
}

// Or returns the lane-wise mask union.
func (m maskW) Or(n maskW) maskW {
	return maskW{m0: m.m0 || n.m0, m1: m.m1 || n.m1, m2: m.m2 || n.m2, m3: m.m3 || n.m3}
}

// AllZero reports whether no mask lane is set.
func (m maskW) AllZero() bool {
	return !m.m0 && !m.m1 && !m.m2 && !m.m3
}

// laneBlend selects a where the mask is set and b otherwise.
func laneBlend(m maskW, a, b laneW) laneW {
	var out laneW
	if m.m0 {
		out.l0 = a.l0
	} else {
		out.l0 = b.l0
	}
	if m.m1 {
		out.l1 = a.l1
	} else {
		out.l1 = b.l1
	}
	if m.m2 {
		out.l2 = a.l2
	} else {
		out.l2 = b.l2
	}
	if m.m3 {
		out.l3 = a.l3
	} else {
		out.l3 = b.l3
	}
	return out
}

// toAcc preserves a lane while naming the accumulation conversion.
func (a laneW) toAcc() laneW { return a }

// toLane preserves a lane while naming the lane conversion.
func (a laneW) toLane() laneW { return a }

// Float lanes have no overflow check to skip, so the bounded forms are the
// plain ones.

// AddBounded returns the accumulator sum.
func (a accW) AddBounded(b accW) accW { return a.Add(b) }

// SubBounded returns the accumulator difference.
func (a accW) SubBounded(b accW) accW { return a.Sub(b) }
