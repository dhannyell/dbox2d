//go:build !dbox2d_fixed

package dbox2d_test

import (
	"math"

	"github.com/dhannyell/dbox2d"
)

func qBits(q dbox2d.Q) uint64 {
	return uint64(math.Float32bits(float32(dbox2d.QToFloat64(q))))
}

// qUlps returns n float32 ulps at one.
func qUlps(n int64) dbox2d.Q {
	return dbox2d.QMustParse("1.1920929e-7").Mul(dbox2d.QFromInt(int(n)))
}

// withinQ reports whether a and b differ by at most limit.
func withinQ(a, b, limit dbox2d.Q) bool {
	return !limit.Less(a.Sub(b).Abs())
}

// Float mode has no saturation; a later step may add a NaN check.
func resetSaturationCount() {}

func saturationCount() uint64 { return 0 }
