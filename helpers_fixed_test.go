//go:build dbox2d_fixed

package dbox2d

import (
	"math"

	"github.com/dhannyell/fixed"
)

// The trace stores radians; each mode reaches them on its own path, so the 0-ulp budget of atan2.txt holds.
func conformanceAtan2Radians(y, x Q) Q { return atan2Turns(y, x).Mul(tau) }

// conformanceRotFromRadians builds the rotation by a radian angle through
// the turn of MakeRot.
func conformanceRotFromRadians(radians float32) Rot {
	return MakeRot(QFromFloat64(float64(radians) / (2 * QToFloat64(Pi()))))
}

// radiansF64 returns a joint angle in radians. Fixed mode keeps turns.
func radiansF64(a Q) float64 { return qToF64(a) * 2 * math.Pi }

// qUlps returns n raw Q32.32 units, where one ulp is 2^-32.
func qUlps(n int64) Q { return fixed.Q32FromRaw(n) }

// contactRounding is the largest error of one rounding to the contact grid.
func contactRounding() Q { return fixed.Q32FromRaw(1 << 15) }

// mirrorTolerance preserves the fixed-mode absolute mirror tolerance.
func mirrorTolerance(fixedTolerance, _ float64) float64 { return fixedTolerance }

func resetSaturationCount() { fixed.ResetSaturationCount() }

func saturationCount() uint64 { return fixed.SaturationCount() }
