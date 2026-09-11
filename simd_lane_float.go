//go:build dbox2d_simd && !dbox2d_fixed

package dbox2d

// The float lane paths share these element types and conversions; each path
// adds its own laneW.

// laneScalar is the element type the lane loads and stores.
type laneScalar = float32

// laneScalarFromQ converts a scalar-mode value to a lane element.
func laneScalarFromQ(q Q) laneScalar { return q.v }

// laneScalarToQ converts a lane element back to a scalar-mode value.
func laneScalarToQ(s laneScalar) Q { return Q{v: s} }

// accScalar is the element type the accumulator stores.
type accScalar = laneScalar

// accScalarToQ converts an accumulator element back to a scalar-mode value.
func accScalarToQ(s accScalar) Q { return laneScalarToQ(s) }

// signedZeroSurvives is true because float lanes keep a sign on zero:
// skipping a block that only adds zero could change a stored bit.
const signedZeroSurvives = true

// accW is the accumulation lane; on the float paths it is the same type as laneW.
type accW = laneW

// vec2W stores two wide vectors.
type vec2W struct {
	x, y laneW
}

// rotW stores a wide cosine and sine pair.
type rotW struct {
	c, s laneW
}

// Float lanes have no overflow check to skip, so the bounded forms are the
// plain ones.

// AddBounded returns the accumulator sum.
func (a accW) AddBounded(b accW) accW { return a.Add(b) }

// SubBounded returns the accumulator difference.
func (a accW) SubBounded(b accW) accW { return a.Sub(b) }
