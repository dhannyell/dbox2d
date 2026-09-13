//go:build dbox2d_fixed

package b2_test

import (
	"testing"

	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/fixed"
)

// TestSaturationMarksAValueInvalid covers the range edge. Fixed point has no
// infinity, so a computation that leaves the range saturates, and the
// validity check is what notices.
func TestSaturationMarksAValueInvalid(t *testing.T) {
	// One hundred kilometres, the largest coordinate the reference accepts.
	big := b2.QFromInt(100000)
	if !b2.IsValidQ(big) {
		t.Fatalf("the largest accepted coordinate is outside the range")
	}

	fixed.ResetSaturationCount()
	over := big.Mul(big)

	// The counter only runs under fixed_satcounter. The saturation itself,
	// which IsValidQ notices below, happens either way.
	if fixed.SaturationCountingEnabled && fixed.SaturationCount() == 0 {
		t.Errorf("the product of two huge values did not saturate")
	}
	if b2.IsValidQ(over) {
		t.Errorf("IsValidQ accepted a saturated value")
	}
	if b2.IsValidVec2(b2.Vec2{X: over}) {
		t.Errorf("IsValidVec2 accepted a saturated component")
	}
}

// TestNegationComesBeforeTheProduct pins the order of operations. The
// reference writes -s * x, which negates the operand. A product in Q32.32
// floors, so negating the product instead shifts the result by one raw unit.
func TestNegationComesBeforeTheProduct(t *testing.T) {
	// Three raw units times one half is one and a half raw units. The floor
	// of that is 1, and the floor of its negative is -2.
	s := fixed.Q32FromRaw(3)
	v := b2.Vec2{X: b2.QHalf(), Y: b2.QHalf()}

	if got := b2.CrossVS(v, s).Y.Raw(); got != -2 {
		t.Errorf("CrossVS y = %d raw, want -2: the product was negated", got)
	}
	if got := b2.CrossSV(s, v).X.Raw(); got != -2 {
		t.Errorf("CrossSV x = %d raw, want -2: the product was negated", got)
	}

	// A normalized rotation with sin equal to one half exercises the same
	// order in the inverse rotation and transform helpers.
	q := b2.Rot{Sin: b2.QHalf(), Cos: b2.QMustParse("0.8660254038")}
	p := b2.Vec2{X: fixed.Q32FromRaw(3)}
	if got := b2.InvRotateVector(q, p).Y.Raw(); got != -2 {
		t.Errorf("InvRotateVector y = %d raw, want -2: the product was negated", got)
	}
	transform := b2.Transform{Q: q}
	if got := b2.InvTransformPoint(transform, p).Y.Raw(); got != -2 {
		t.Errorf("InvTransformPoint y = %d raw, want -2: the product was negated", got)
	}
}

// TestNormalizeKeepsAShortVector is the evidence for the divergence that
// replaced the reciprocal. A vector of a few raw units would collapse to
// zero if the port multiplied by one over its length.
func TestNormalizeKeepsAShortVector(t *testing.T) {
	v := b2.Vec2{X: fixed.Q32FromRaw(3), Y: fixed.Q32FromRaw(4)}

	length, unit := b2.GetLengthAndNormalize(v)
	if got := length.Raw(); got != 5 {
		t.Errorf("length = %d raw, want 5", got)
	}
	if !b2.IsNormalized(unit) {
		t.Errorf("the unit vector of a short vector is not normalized: %v", unit)
	}
}
