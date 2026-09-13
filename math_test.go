package b2_test

import (
	"testing"

	"github.com/dhannyell/dbox2d"
)

// near reports whether a and b differ by less than limit.
func near(a, b, limit b2.Q) bool {
	return a.Sub(b).Abs().Less(limit)
}

// tol returns num/den as a tolerance value.
func tol(num, den int) b2.Q {
	return b2.QFromRatio(num, den)
}

// TestConstantsMatchTheReference checks the values that the whole solver is
// tuned around. A wrong slop changes every contact.
func TestConstantsMatchTheReference(t *testing.T) {
	if got, want := b2.LinearSlop(), b2.QMustParse("0.005"); !got.Eq(want) {
		t.Errorf("LinearSlop = %v, want %v", got, want)
	}
	// The reference derives the speculative distance from the slop, so the
	// port derives it too. It sits one raw unit below the nearest 0.02.
	if got, want := b2.SpeculativeDistance(), b2.LinearSlop().Mul(b2.QFromInt(4)); !got.Eq(want) {
		t.Errorf("SpeculativeDistance = %v, want %v", got, want)
	}
	if got, want := b2.AABBMargin(), b2.QMustParse("0.05"); !got.Eq(want) {
		t.Errorf("AABBMargin = %v, want %v", got, want)
	}
	// The reference limits a step to 0.25 * pi radians, which is 0.125 turns.
	if got, want := b2.MaxRotation(), b2.QMustParse("0.125"); !got.Eq(want) {
		t.Errorf("MaxRotation = %v, want %v", got, want)
	}
	if got := b2.ReferenceVersion(); got.Major != 3 || got.Minor != 1 || got.Revision != 1 {
		t.Errorf("ReferenceVersion = %+v, want 3.1.1", got)
	}
}

// TestTransformRoundTrip is the conversion test between local and world
// space. Every shape query depends on the pair agreeing.
func TestTransformRoundTrip(t *testing.T) {
	xf := b2.Transform{
		P: b2.Vec2{X: b2.QFromInt(3), Y: b2.QMustParse("-7.25")},
		Q: b2.MakeRot(b2.QMustParse("0.3")),
	}
	p := b2.Vec2{X: b2.QMustParse("1.5"), Y: b2.QMustParse("2.125")}

	world := b2.TransformPoint(xf, p)
	back := b2.InvTransformPoint(xf, world)

	limit := tol(1, 1000000)
	if !near(back.X, p.X, limit) || !near(back.Y, p.Y, limit) {
		t.Errorf("round trip = %v, want %v", back, p)
	}
}

// TestInvMulTransformsUndoesMulTransforms checks the transform composition
// against its inverse, which the solver uses for every joint frame.
func TestInvMulTransformsUndoesMulTransforms(t *testing.T) {
	a := b2.Transform{
		P: b2.Vec2{X: b2.QFromInt(2), Y: b2.QFromInt(5)},
		Q: b2.MakeRot(b2.QMustParse("0.1")),
	}
	b := b2.Transform{
		P: b2.Vec2{X: b2.QMustParse("-1.5"), Y: b2.QMustParse("0.75")},
		Q: b2.MakeRot(b2.QMustParse("0.4")),
	}

	got := b2.InvMulTransforms(a, b2.MulTransforms(a, b))

	limit := tol(1, 100000)
	if !near(got.P.X, b.P.X, limit) || !near(got.P.Y, b.P.Y, limit) {
		t.Errorf("position = %v, want %v", got.P, b.P)
	}
	if !near(got.Q.Sin, b.Q.Sin, limit) || !near(got.Q.Cos, b.Q.Cos, limit) {
		t.Errorf("rotation = %v, want %v", got.Q, b.Q)
	}
}

// TestIntegrateRotationCompletesATurn checks the angle unit. The reference
// integrates radians; this package integrates turns, so a wrong scale factor
// shows up as a rotation that is off by two pi.
func TestIntegrateRotationCompletesATurn(t *testing.T) {
	const steps = 360
	q := b2.RotIdentity()
	delta := b2.QFromRatio(1, steps)
	for range steps {
		q = b2.IntegrateRotation(q, delta)
	}

	if !b2.IsNormalizedRot(q) {
		t.Fatalf("rotation left the unit circle: %v", q)
	}
	// One full turn returns to the identity.
	if angle := b2.RotGetAngle(q); !near(angle, b2.QZero(), tol(1, 1000)) {
		t.Errorf("angle after one turn = %v, want 0", angle)
	}
}

// TestComputeAngularVelocityInvertsIntegration checks that the solver can
// recover the velocity it used to advance a rotation.
func TestComputeAngularVelocityInvertsIntegration(t *testing.T) {
	h := b2.QFromRatio(1, 60)
	invH := b2.QFromInt(60)
	omega := b2.QMustParse("0.25") // turns per second

	q1 := b2.MakeRot(b2.QMustParse("0.2"))
	q2 := b2.IntegrateRotation(q1, omega.Mul(h))

	if got := b2.ComputeAngularVelocity(q1, q2, invH); !near(got, omega, tol(1, 1000)) {
		t.Errorf("angular velocity = %v, want %v", got, omega)
	}
}

// TestUnwindAngleReducesToHalfTurn checks the reduction that replaces the
// remainder of two pi. In turns the reduction is exact.
func TestUnwindAngleReducesToHalfTurn(t *testing.T) {
	half := b2.QHalf()
	for _, in := range []string{"0.25", "1.25", "-1.25", "7.5", "-3.75"} {
		got := b2.UnwindAngle(b2.QMustParse(in))
		if half.Less(got.Abs()) {
			t.Errorf("UnwindAngle(%s) = %v, outside [-0.5, 0.5]", in, got)
		}
		// The reduced angle names the same direction.
		if a, b := b2.MakeRot(got), b2.MakeRot(b2.QMustParse(in)); !a.Cos.Eq(b.Cos) || !a.Sin.Eq(b.Sin) {
			t.Errorf("UnwindAngle(%s) = %v, which is a different rotation", in, got)
		}
	}
}

// TestCrossAndPerpAgree checks the identities that the reference documents:
// the perpendiculars are cross products with one.
func TestCrossAndPerpAgree(t *testing.T) {
	v := b2.Vec2{X: b2.QFromInt(3), Y: b2.QMustParse("-4.5")}
	one := b2.QOne()

	if got, want := b2.CrossSV(one, v), b2.LeftPerp(v); got != want {
		t.Errorf("CrossSV(1, v) = %v, want %v", got, want)
	}
	if got, want := b2.CrossVS(v, one), b2.RightPerp(v); got != want {
		t.Errorf("CrossVS(v, 1) = %v, want %v", got, want)
	}
	// A vector is parallel to itself, so the cross product is zero.
	if got := b2.Cross(v, v); !got.Eq(b2.QZero()) {
		t.Errorf("Cross(v, v) = %v, want 0", got)
	}
}

// TestLerpHitsBothEnds checks the endpoint behaviour that decided the
// formula. The weighted form returns each end exactly.
func TestLerpHitsBothEnds(t *testing.T) {
	a := b2.Vec2{X: b2.QFromInt(1), Y: b2.QFromInt(2)}
	b := b2.Vec2{X: b2.QFromInt(9), Y: b2.QMustParse("-3.5")}

	if got := b2.Lerp(a, b, b2.QZero()); got != a {
		t.Errorf("Lerp at 0 = %v, want %v", got, a)
	}
	if got := b2.Lerp(a, b, b2.QOne()); got != b {
		t.Errorf("Lerp at 1 = %v, want %v", got, b)
	}
}

// TestSolve22SolvesTheSystem checks the 2-by-2 solver, and that a
// singular matrix returns zero instead of dividing by zero.
func TestSolve22SolvesTheSystem(t *testing.T) {
	m := b2.Mat22{
		Cx: b2.Vec2{X: b2.QFromInt(4), Y: b2.QFromInt(1)},
		Cy: b2.Vec2{X: b2.QFromInt(2), Y: b2.QFromInt(3)},
	}
	b := b2.Vec2{X: b2.QFromInt(10), Y: b2.QFromInt(8)}

	x := b2.Solve22(m, b)
	got := b2.MulMV(m, x)

	limit := tol(1, 100000)
	if !near(got.X, b.X, limit) || !near(got.Y, b.Y, limit) {
		t.Errorf("m * x = %v, want %v", got, b)
	}

	singular := b2.Mat22{
		Cx: b2.Vec2{X: b2.QFromInt(1), Y: b2.QFromInt(2)},
		Cy: b2.Vec2{X: b2.QFromInt(2), Y: b2.QFromInt(4)},
	}
	if got := b2.Solve22(singular, b); got != (b2.Vec2{}) {
		t.Errorf("singular solve = %v, want the zero vector", got)
	}
	if got := b2.GetInverse22(singular); got != (b2.Mat22{}) {
		t.Errorf("singular inverse = %v, want the zero matrix", got)
	}
}

// TestSpringDamperRemovesEnergy checks the implicit spring: a body at rest
// away from zero gains a velocity that points back to zero.
func TestSpringDamperRemovesEnergy(t *testing.T) {
	hertz := b2.QFromInt(4)
	damping := b2.QOne()
	position := b2.QFromInt(2)
	step := b2.QFromRatio(1, 60)

	v := b2.SpringDamper(hertz, damping, position, b2.QZero(), step)
	if !v.Less(b2.QZero()) {
		t.Errorf("velocity = %v, want a value below zero", v)
	}

	// A zero stiffness leaves the velocity alone.
	kept := b2.QMustParse("1.5")
	if got := b2.SpringDamper(b2.QZero(), damping, position, kept, step); !got.Eq(kept) {
		t.Errorf("velocity at zero hertz = %v, want %v", got, kept)
	}
}

// TestNormalizedChecksAcceptAUnitPair guards the tolerances that replaced
// the float epsilons of the reference.
func TestNormalizedChecksAcceptAUnitPair(t *testing.T) {
	for _, turns := range []string{"0", "0.125", "0.3", "-0.4"} {
		q := b2.MakeRot(b2.QMustParse(turns))
		if !b2.IsNormalizedRot(q) {
			t.Errorf("rotation at %s turns is not normalized: %v", turns, q)
		}
		if !b2.IsNormalized(b2.RotGetXAxis(q)) {
			t.Errorf("x axis at %s turns is not a unit vector", turns)
		}
	}

	if b2.IsNormalized(b2.Vec2{X: b2.QFromInt(2)}) {
		t.Errorf("IsNormalized accepted a vector of length two")
	}
}

// TestNormalizeRotKeepsAZeroRotation checks that an invalid rotation stays
// invalid. The identity would hide the bad state from every later check.
func TestNormalizeRotKeepsAZeroRotation(t *testing.T) {
	got := b2.NormalizeRot(b2.Rot{})

	if got != (b2.Rot{}) {
		t.Errorf("NormalizeRot of a zero rotation = %v, want the zero rotation", got)
	}
	if b2.IsValidRotation(got) {
		t.Errorf("IsValidRotation accepted a zero rotation")
	}
	if !b2.IsValidRotation(b2.RotIdentity()) {
		t.Errorf("IsValidRotation rejected the identity")
	}
}

// TestComputeRotationBetweenUnitVectors covers the assertion of the
// reference, which this port keeps as a panic in every build.
func TestComputeRotationBetweenUnitVectors(t *testing.T) {
	x := b2.Vec2{X: b2.QOne()}
	y := b2.Vec2{Y: b2.QOne()}

	got := b2.ComputeRotationBetweenUnitVectors(x, y)
	if angle := b2.RotGetAngle(got); !near(angle, b2.QMustParse("0.25"), tol(1, 1000)) {
		t.Errorf("angle from the x axis to the y axis = %v, want 0.25 turns", angle)
	}

	defer func() {
		if recover() == nil {
			t.Errorf("a vector of length two did not panic")
		}
	}()
	b2.ComputeRotationBetweenUnitVectors(x, b2.Vec2{X: b2.QFromInt(2)})
}
