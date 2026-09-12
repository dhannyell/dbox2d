//go:build dbox2d_fixed

package dbox2d

import (
	"math"
	"testing"

	"github.com/dhannyell/fixed"
)

// TestQFromFloat64Rounds checks that a float64 converts as QMustParse rounds a
// decimal: to the nearest raw value, with halves away from zero, saturating
// outside the range.
func TestQFromFloat64Rounds(t *testing.T) {
	const ulp = 1.0 / (1 << 32)
	cases := []struct {
		name string
		f    float64
		want Q
	}{
		{"decimal", 0.35, QMustParse("0.35")},
		{"negative decimal", -1000.125, QMustParse("-1000.125")},
		{"half up", 1.5 * ulp, fixed.Q32FromRaw(2)},
		{"half down", -1.5 * ulp, fixed.Q32FromRaw(-2)},
		{"below half", 0.4 * ulp, QZero()},
		{"largest", math.Inf(1), QMaxValue()},
		{"smallest", math.Inf(-1), QMinValue()},
		{"too large", 1 << 40, QMaxValue()},
		{"too small", -(1 << 40), QMinValue()},
	}
	for _, tc := range cases {
		if got := QFromFloat64(tc.f); got.Raw() != tc.want.Raw() {
			t.Errorf("%s: QFromFloat64(%v) raw = %d, want %d", tc.name, tc.f, got.Raw(), tc.want.Raw())
		}
	}
	if got := QToFloat64(QFromFloat64(0.375)); got != 0.375 {
		t.Errorf("round trip = %v, want 0.375", got)
	}
	requirePanic(t, func() { QFromFloat64(math.NaN()) })
}
