//go:build !dbox2d_float

package dbox2d

import "github.com/dhannyell/fixed"

func conformanceAtan2Radians(y, x Q) Q { return atan2Turns(y, x).Mul(tau) }

// qUlps returns n raw Q32.32 units, where one ulp is 2^-32.
func qUlps(n int64) Q { return fixed.Q32FromRaw(n) }

// mirrorTolerance preserves the fixed-mode absolute mirror tolerance.
func mirrorTolerance(fixedTolerance, _ float64) float64 { return fixedTolerance }

func resetSaturationCount() { fixed.ResetSaturationCount() }

func saturationCount() uint64 { return fixed.SaturationCount() }
