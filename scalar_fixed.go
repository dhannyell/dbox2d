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

// scalarEpsilon is the tolerance of a comparison with zero. Fixed-point
// arithmetic is exact, so the tolerance is zero; a float mode substitutes
// the epsilon of the reference.
var scalarEpsilon = QZero()

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
