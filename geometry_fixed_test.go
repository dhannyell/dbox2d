//go:build !dbox2d_float

package dbox2d_test

import (
	"testing"

	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/fixed"
)

// TestRayCastCapsuleHitsTheSide pins the Cramer solve with an oblique ray.
// Its determinant is not reciprocal-exact in Q32.32.
func TestRayCastCapsuleHitsTheSide(t *testing.T) {
	capsule := dbox2d.Capsule{Center1: pt("-1", "0"), Center2: pt("1", "0"), Radius: dbox2d.QHalf()}
	translation := pt("-9.5", "-9.5")
	wantFraction := dbox2d.QFromRatio(3, 5)
	target := pt("0", "0.5")
	origin := target.Sub(translation.Mul(wantFraction))
	input := ray(origin, translation)

	output := dbox2d.RayCastCapsule(&input, &capsule)

	if !output.Hit {
		t.Fatalf("the ray misses the capsule")
	}
	if !output.Fraction.Eq(wantFraction) {
		t.Errorf("fraction = %v, want %v", output.Fraction, wantFraction)
	}
	// The reference order of operations floors the x coordinate two raw
	// units below zero.
	wantPoint := dbox2d.Vec2{X: fixed.Q32FromRaw(-2), Y: dbox2d.QHalf()}
	if output.Point != wantPoint {
		t.Errorf("point = %v, want %v", output.Point, wantPoint)
	}
	if output.Normal != pt("0", "1") {
		t.Errorf("normal = %v, want (0, 1)", output.Normal)
	}
}

// TestIsValidRayRejectsASaturatedOrigin keeps the fixed-only range check.
func TestIsValidRayRejectsASaturatedOrigin(t *testing.T) {
	input := ray(pt("0", "0"), pt("1", "0"))
	input.Origin = dbox2d.Vec2{X: dbox2d.QMaxValue(), Y: dbox2d.QZero()}
	if dbox2d.IsValidRay(&input) {
		t.Error("IsValidRay accepts a saturated origin")
	}
}
