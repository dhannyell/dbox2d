//go:build dbox2d_fixed

package b2_test

import (
	"testing"

	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/fixed"
)

// TestRayCastCapsuleHitsTheSide pins the Cramer solve with an oblique ray.
// Its determinant is not reciprocal-exact in Q32.32.
func TestRayCastCapsuleHitsTheSide(t *testing.T) {
	capsule := b2.Capsule{Center1: pt("-1", "0"), Center2: pt("1", "0"), Radius: b2.QHalf()}
	translation := pt("-9.5", "-9.5")
	wantFraction := b2.QFromRatio(3, 5)
	target := pt("0", "0.5")
	origin := target.Sub(translation.Mul(wantFraction))
	input := ray(origin, translation)

	output := b2.RayCastCapsule(&input, &capsule)

	if !output.Hit {
		t.Fatalf("the ray misses the capsule")
	}
	if !output.Fraction.Eq(wantFraction) {
		t.Errorf("fraction = %v, want %v", output.Fraction, wantFraction)
	}
	// The reference order of operations floors the x coordinate two raw
	// units below zero.
	wantPoint := b2.Vec2{X: fixed.Q32FromRaw(-2), Y: b2.QHalf()}
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
	input.Origin = b2.Vec2{X: b2.QMaxValue(), Y: b2.QZero()}
	if b2.IsValidRay(&input) {
		t.Error("IsValidRay accepts a saturated origin")
	}
}
