//go:build !dbox2d_fixed

package dbox2d

import "math"

// The trace stores radians; each mode reaches them on its own path, so the 0-ulp budget of atan2.txt holds.
func conformanceAtan2Radians(y, x Q) Q { return atan2Radians(y, x) }

// conformanceRotFromRadians builds the rotation of b2MakeRot from its radian
// angle, in [-pi, pi], as the reference scene does.
func conformanceRotFromRadians(radians float32) Rot {
	return makeRotRadians(QFromFloat64(float64(radians)))
}

// radiansF64 returns a joint angle in radians. Float mode keeps radians.
func radiansF64(a Q) float64 { return qToF64(a) }

// qUlps returns an absolute tolerance of n float32 ulps at one.
func qUlps(n int64) Q { return scalarEpsilon.Mul(QFromInt(int(n))) }

// The float mode solves contacts in Q, so the contact grid adds no rounding.
func contactRounding() Q { return QZero() }

// mirrorTolerance has a 2e-4 floor plus 2e-5 per unit of magnitude. The
// measured maximum error is 2.4379e-4 absolute at |want| > 12, inside the
// relative term, and 5.4911e-4 relative on small values, inside the floor.
func mirrorTolerance(_ float64, want float64) float64 {
	const floor = 2e-4
	const rel = 2e-5
	return math.Max(floor, rel*math.Abs(want))
}

// Float mode has no saturation; a later step may add a NaN check.
func resetSaturationCount() {}

func saturationCount() uint64 { return 0 }
