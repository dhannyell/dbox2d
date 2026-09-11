//go:build !dbox2d_fixed

package dbox2d_test

import (
	"math"
	"testing"

	"github.com/dhannyell/dbox2d"
)

// TestIsValidRayRejectsANaNOrigin keeps the float-only invalid scalar case.
func TestIsValidRayRejectsANaNOrigin(t *testing.T) {
	input := ray(pt("0", "0"), pt("1", "0"))
	input.Origin = dbox2d.Vec2{X: dbox2d.QFromFloat64(math.NaN()), Y: dbox2d.QZero()}
	if dbox2d.IsValidRay(&input) {
		t.Error("IsValidRay accepts a NaN origin")
	}
}
