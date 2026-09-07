//go:build dbox2d_float

package dbox2d

import "math"

// qUlps returns an absolute tolerance of n float32 ulps at one.
func qUlps(n int64) Q { return scalarEpsilon.Mul(QFromInt(int(n))) }

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
