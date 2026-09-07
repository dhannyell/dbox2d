//go:build !dbox2d_float

package dbox2d

import "github.com/dhannyell/fixed"

// The library has one scalar per build mode. This file is the fixed-point
// mode: Q32.32 from github.com/dhannyell/fixed. It is the only file of the
// library that imports that module; every other file builds its numbers
// through the constructors below.

type (
	// Q is a signed Q32.32 fixed-point number.
	Q = fixed.Q32

	// Vec2 is a 2D vector. It represents a point or a free vector.
	Vec2 = fixed.Vec2

	// Rot is a 2D rotation, stored as a sine and cosine pair.
	Rot = fixed.Rot
)

// QZero returns zero.
func QZero() Q { return fixed.Q32Zero() }

// QOne returns one.
func QOne() Q { return fixed.Q32One() }

// QHalf returns one half.
func QHalf() Q { return fixed.Q32Half() }

// QFromInt returns the integer i as a scalar.
func QFromInt(i int) Q { return fixed.Q32FromInt(i) }

// QFromRatio returns num/den, rounded the way the scalar rounds a division.
func QFromRatio(num, den int) Q { return fixed.Q32FromRatio(num, den) }

// QMustParse returns the decimal literal s as a scalar. It panics when s is
// not a decimal number.
func QMustParse(s string) Q { return fixed.Q32MustParse(s) }

// QMaxValue returns the largest scalar. A computation that leaves the
// representable range saturates to it; see IsValidQ.
func QMaxValue() Q { return fixed.Q32MaxValue() }

// QMinValue returns the smallest scalar.
func QMinValue() Q { return fixed.Q32MinValue() }

// RotIdentity returns the rotation by zero turns.
func RotIdentity() Rot { return fixed.RotIdentity() }

// MakeRot returns the rotation by the angle t, in turns.
func MakeRot(t Q) Rot { return fixed.RotFromTurns(t) }

// atan2Turns returns the angle of (x, y), in turns.
func atan2Turns(y, x Q) Q { return fixed.Atan2Turns(y, x) }

var (
	// The fixed mode uses zero as its rounding-noise threshold.
	scalarEpsilon = QZero()

	// The fixed mode squares its rounding-noise threshold.
	scalarEpsilonSq = scalarEpsilon.Mul(scalarEpsilon)

	// upstream 100.0f * FLT_EPSILON, about 1.2e-5. One raw unit is 2^-32.
	normalizedTolerance = QFromRatio(1, 1<<16)
)

// belowEpsilon reports whether a non-negative x is below the rounding noise
// of the fixed-point format.
func belowEpsilon(x Q) bool { return x.Eq(QZero()) }

// belowEpsilonSq reports whether a squared length is below fixed-point
// rounding noise.
func belowEpsilonSq(x Q) bool { return x.Eq(QZero()) }

// sensorOverlaps reports whether a sensor overlap distance is below the
// fixed-point format's rounding noise.
func sensorOverlaps(distance Q) bool { return distance.Eq(QZero()) }

// IsValidQ reports whether a is a usable value. Fixed-point arithmetic has
// no NaN and no infinity; it saturates instead, so a saturated value is the
// signal that a computation left the representable range.
func IsValidQ(a Q) bool {
	return !a.Eq(QMinValue()) && !a.Eq(QMaxValue())
}

// qBits returns the bits the checksum folds.
func qBits(q Q) uint64 { return uint64(q.Raw()) }

// QFromFloat64 converts a presentation value, such as a camera value, to a
// scalar. It must never be used by simulation code.
func QFromFloat64(f float64) Q {
	return fixed.Q32FromRaw(int64(f * (1 << 32)))
}

// QToFloat64 converts a scalar to a presentation value, such as a camera
// value. It must never be used by simulation code.
func QToFloat64(q Q) float64 { return float64(q.Raw()) / (1 << 32) }
