//go:build dbox2d_fixed

package dbox2d_test

import (
	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/fixed"
)

func qBits(q dbox2d.Q) uint64 { return uint64(q.Raw()) }

// qUlps returns n raw Q32.32 units, where one ulp is 2^-32.
func qUlps(n int64) dbox2d.Q { return fixed.Q32FromRaw(n) }

// withinQ reports whether a and b differ by at most limit.
func withinQ(a, b, limit dbox2d.Q) bool {
	return !limit.Less(a.Sub(b).Abs())
}

func resetSaturationCount() { fixed.ResetSaturationCount() }

func saturationCount() uint64 { return fixed.SaturationCount() }
