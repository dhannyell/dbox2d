//go:build !dbox2d_fixed

package b2_test

import (
	"math"
	"testing"

	"github.com/dhannyell/dbox2d"
)

// TestIsValidRayRejectsANaNOrigin keeps the float-only invalid scalar case.
func TestIsValidRayRejectsANaNOrigin(t *testing.T) {
	input := ray(pt("0", "0"), pt("1", "0"))
	input.Origin = b2.Vec2{X: b2.QFromFloat64(math.NaN()), Y: b2.QZero()}
	if b2.IsValidRay(&input) {
		t.Error("IsValidRay accepts a NaN origin")
	}
}
