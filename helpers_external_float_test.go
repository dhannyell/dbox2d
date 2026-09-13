//go:build !dbox2d_fixed

package b2_test

import (
	"math"

	"github.com/dhannyell/dbox2d"
)

func qBits(q b2.Q) uint64 {
	return uint64(math.Float32bits(float32(b2.QToFloat64(q))))
}

// qUlps returns n float32 ulps at one.
func qUlps(n int64) b2.Q {
	return b2.QMustParse("1.1920929e-7").Mul(b2.QFromInt(int(n)))
}

// withinQ reports whether a and b differ by at most limit.
func withinQ(a, b, limit b2.Q) bool {
	return !limit.Less(a.Sub(b).Abs())
}

// Float mode has no saturation; a later step may add a NaN check.
func resetSaturationCount() {}

func saturationCount() uint64 { return 0 }
