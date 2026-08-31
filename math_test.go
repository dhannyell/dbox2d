package dbox2d_test

import (
	"testing"

	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/fixed"
)

// near reports whether a and b differ by less than limit.
func near(a, b, limit dbox2d.Q) bool {
	return a.Sub(b).Abs().Less(limit)
}

// tol returns num/den as a tolerance value.
func tol(num, den int) dbox2d.Q {
	return fixed.FromRatio(num, den)
}

// TestConstantsMatchTheReference checks the values that the whole solver is
// tuned around. A wrong slop changes every contact.
func TestConstantsMatchTheReference(t *testing.T) {
	if got, want := dbox2d.LinearSlop(), fixed.MustParse("0.005"); !got.Eq(want) {
		t.Errorf("LinearSlop = %v, want %v", got, want)
	}
	// The reference derives the speculative distance from the slop, so the
	// port derives it too. It sits one raw unit below the nearest 0.02.
	if got, want := dbox2d.SpeculativeDistance(), dbox2d.LinearSlop().Mul(fixed.FromInt(4)); !got.Eq(want) {
		t.Errorf("SpeculativeDistance = %v, want %v", got, want)
	}
	if got, want := dbox2d.AABBMargin(), fixed.MustParse("0.05"); !got.Eq(want) {
		t.Errorf("AABBMargin = %v, want %v", got, want)
	}
	// The reference limits a step to 0.25 * pi radians, which is 0.125 turns.
	if got, want := dbox2d.MaxRotation(), fixed.MustParse("0.125"); !got.Eq(want) {
		t.Errorf("MaxRotation = %v, want %v", got, want)
	}
	if got := dbox2d.ReferenceVersion(); got.Major != 3 || got.Minor != 1 || got.Revision != 1 {
		t.Errorf("ReferenceVersion = %+v, want 3.1.1", got)
	}
}

// TestTransformRoundTrip is the conversion test between local and world
// space. Every shape query depends on the pair agreeing.
func TestTransformRoundTrip(t *testing.T) {
	xf := dbox2d.Transform{
		P: dbox2d.Vec2{X: fixed.FromInt(3), Y: fixed.MustParse("-7.25")},
		Q: dbox2d.MakeRot(fixed.MustParse("0.3")),
	}
	p := dbox2d.Vec2{X: fixed.MustParse("1.5"), Y: fixed.MustParse("2.125")}

	world := dbox2d.TransformPoint(xf, p)
	back := dbox2d.InvTransformPoint(xf, world)

	limit := tol(1, 1000000)
	if !near(back.X, p.X, limit) || !near(back.Y, p.Y, limit) {
		t.Errorf("round trip = %v, want %v", back, p)
	}
}

// TestInvMulTransformsUndoesMulTransforms checks the transform composition
// against its inverse, which the solver uses for every joint frame.
func TestInvMulTransformsUndoesMulTransforms(t *testing.T) {
	a := dbox2d.Transform{
		P: dbox2d.Vec2{X: fixed.FromInt(2), Y: fixed.FromInt(5)},
		Q: dbox2d.MakeRot(fixed.MustParse("0.1")),
	}
	b := dbox2d.Transform{
		P: dbox2d.Vec2{X: fixed.MustParse("-1.5"), Y: fixed.MustParse("0.75")},
		Q: dbox2d.MakeRot(fixed.MustParse("0.4")),
	}

	got := dbox2d.InvMulTransforms(a, dbox2d.MulTransforms(a, b))

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
	q := dbox2d.RotIdentity()
	delta := fixed.FromRatio(1, steps)
	for range steps {
		q = dbox2d.IntegrateRotation(q, delta)
	}

	if !dbox2d.IsNormalizedRot(q) {
		t.Fatalf("rotation left the unit circle: %v", q)
	}
	// One full turn returns to the identity.
	if angle := dbox2d.RotGetAngle(q); !near(angle, fixed.Zero(), tol(1, 1000)) {
		t.Errorf("angle after one turn = %v, want 0", angle)
	}
}

// TestComputeAngularVelocityInvertsIntegration checks that the solver can
// recover the velocity it used to advance a rotation.
func TestComputeAngularVelocityInvertsIntegration(t *testing.T) {
	h := fixed.FromRatio(1, 60)
	invH := fixed.FromInt(60)
	omega := fixed.MustParse("0.25") // turns per second

	q1 := dbox2d.MakeRot(fixed.MustParse("0.2"))
	q2 := dbox2d.IntegrateRotation(q1, omega.Mul(h))

	if got := dbox2d.ComputeAngularVelocity(q1, q2, invH); !near(got, omega, tol(1, 1000)) {
		t.Errorf("angular velocity = %v, want %v", got, omega)
	}
}

// TestUnwindAngleReducesToHalfTurn checks the reduction that replaces the
// remainder of two pi. In turns the reduction is exact.
func TestUnwindAngleReducesToHalfTurn(t *testing.T) {
	half := fixed.Half()
	for _, in := range []string{"0.25", "1.25", "-1.25", "7.5", "-3.75"} {
		got := dbox2d.UnwindAngle(fixed.MustParse(in))
		if half.Less(got.Abs()) {
			t.Errorf("UnwindAngle(%s) = %v, outside [-0.5, 0.5]", in, got)
		}
		// The reduced angle names the same direction.
		if a, b := dbox2d.MakeRot(got), dbox2d.MakeRot(fixed.MustParse(in)); !a.Cos.Eq(b.Cos) || !a.Sin.Eq(b.Sin) {
			t.Errorf("UnwindAngle(%s) = %v, which is a different rotation", in, got)
		}
	}
}

// TestCrossAndPerpAgree checks the identities that the reference documents:
// the perpendiculars are cross products with one.
func TestCrossAndPerpAgree(t *testing.T) {
	v := dbox2d.Vec2{X: fixed.FromInt(3), Y: fixed.MustParse("-4.5")}
	one := fixed.One()

	if got, want := dbox2d.CrossSV(one, v), dbox2d.LeftPerp(v); got != want {
		t.Errorf("CrossSV(1, v) = %v, want %v", got, want)
	}
	if got, want := dbox2d.CrossVS(v, one), dbox2d.RightPerp(v); got != want {
		t.Errorf("CrossVS(v, 1) = %v, want %v", got, want)
	}
	// A vector is parallel to itself, so the cross product is zero.
	if got := dbox2d.Cross(v, v); !got.Eq(fixed.Zero()) {
		t.Errorf("Cross(v, v) = %v, want 0", got)
	}
}

// TestLerpHitsBothEnds checks the endpoint behaviour that decided the
// formula. The weighted form returns each end exactly.
func TestLerpHitsBothEnds(t *testing.T) {
	a := dbox2d.Vec2{X: fixed.FromInt(1), Y: fixed.FromInt(2)}
	b := dbox2d.Vec2{X: fixed.FromInt(9), Y: fixed.MustParse("-3.5")}

	if got := dbox2d.Lerp(a, b, fixed.Zero()); got != a {
		t.Errorf("Lerp at 0 = %v, want %v", got, a)
	}
	if got := dbox2d.Lerp(a, b, fixed.One()); got != b {
		t.Errorf("Lerp at 1 = %v, want %v", got, b)
	}
}

// TestSolve22SolvesTheSystem checks the 2-by-2 solver, and that a
// singular matrix returns zero instead of dividing by zero.
func TestSolve22SolvesTheSystem(t *testing.T) {
	m := dbox2d.Mat22{
		Cx: dbox2d.Vec2{X: fixed.FromInt(4), Y: fixed.FromInt(1)},
		Cy: dbox2d.Vec2{X: fixed.FromInt(2), Y: fixed.FromInt(3)},
	}
	b := dbox2d.Vec2{X: fixed.FromInt(10), Y: fixed.FromInt(8)}

	x := dbox2d.Solve22(m, b)
	got := dbox2d.MulMV(m, x)

	limit := tol(1, 100000)
	if !near(got.X, b.X, limit) || !near(got.Y, b.Y, limit) {
		t.Errorf("m * x = %v, want %v", got, b)
	}

	singular := dbox2d.Mat22{
		Cx: dbox2d.Vec2{X: fixed.FromInt(1), Y: fixed.FromInt(2)},
		Cy: dbox2d.Vec2{X: fixed.FromInt(2), Y: fixed.FromInt(4)},
	}
	if got := dbox2d.Solve22(singular, b); got != (dbox2d.Vec2{}) {
		t.Errorf("singular solve = %v, want the zero vector", got)
	}
	if got := dbox2d.GetInverse22(singular); got != (dbox2d.Mat22{}) {
		t.Errorf("singular inverse = %v, want the zero matrix", got)
	}
}

// TestSpringDamperRemovesEnergy checks the implicit spring: a body at rest
// away from zero gains a velocity that points back to zero.
func TestSpringDamperRemovesEnergy(t *testing.T) {
	hertz := fixed.FromInt(4)
	damping := fixed.One()
	position := fixed.FromInt(2)
	step := fixed.FromRatio(1, 60)

	v := dbox2d.SpringDamper(hertz, damping, position, fixed.Zero(), step)
	if !v.Less(fixed.Zero()) {
		t.Errorf("velocity = %v, want a value below zero", v)
	}

	// A zero stiffness leaves the velocity alone.
	kept := fixed.MustParse("1.5")
	if got := dbox2d.SpringDamper(fixed.Zero(), damping, position, kept, step); !got.Eq(kept) {
		t.Errorf("velocity at zero hertz = %v, want %v", got, kept)
	}
}

// TestSaturationMarksAValueInvalid covers the range edge. Fixed point has no
// infinity, so a computation that leaves the range saturates, and the
// validity check is what notices.
func TestSaturationMarksAValueInvalid(t *testing.T) {
	// One hundred kilometres, the largest coordinate the reference accepts.
	big := fixed.FromInt(100000)
	if !dbox2d.IsValidQ(big) {
		t.Fatalf("the largest accepted coordinate is outside the range")
	}

	fixed.ResetSaturationCount()
	over := big.Mul(big)

	if fixed.SaturationCount() == 0 {
		t.Errorf("the product of two huge values did not saturate")
	}
	if dbox2d.IsValidQ(over) {
		t.Errorf("IsValidQ accepted a saturated value")
	}
	if dbox2d.IsValidVec2(dbox2d.Vec2{X: over}) {
		t.Errorf("IsValidVec2 accepted a saturated component")
	}
}

// TestNormalizedChecksAcceptAUnitPair guards the tolerances that replaced
// the float epsilons of the reference.
func TestNormalizedChecksAcceptAUnitPair(t *testing.T) {
	for _, turns := range []string{"0", "0.125", "0.3", "-0.4"} {
		q := dbox2d.MakeRot(fixed.MustParse(turns))
		if !dbox2d.IsNormalizedRot(q) {
			t.Errorf("rotation at %s turns is not normalized: %v", turns, q)
		}
		if !dbox2d.IsNormalized(dbox2d.RotGetXAxis(q)) {
			t.Errorf("x axis at %s turns is not a unit vector", turns)
		}
	}

	if dbox2d.IsNormalized(dbox2d.Vec2{X: fixed.FromInt(2)}) {
		t.Errorf("IsNormalized accepted a vector of length two")
	}
}

// TestNegationComesBeforeTheProduct pins the order of operations. The
// reference writes -s * x, which negates the operand. A product in Q32.32
// floors, so negating the product instead shifts the result by one raw unit.
func TestNegationComesBeforeTheProduct(t *testing.T) {
	// Three raw units times one half is one and a half raw units. The floor
	// of that is 1, and the floor of its negative is -2.
	s := fixed.FromRaw(3)
	v := dbox2d.Vec2{X: fixed.Half(), Y: fixed.Half()}

	if got := dbox2d.CrossVS(v, s).Y.Raw(); got != -2 {
		t.Errorf("CrossVS y = %d raw, want -2: the product was negated", got)
	}
	if got := dbox2d.CrossSV(s, v).X.Raw(); got != -2 {
		t.Errorf("CrossSV x = %d raw, want -2: the product was negated", got)
	}

	// A normalized rotation with sin equal to one half exercises the same
	// order in the inverse rotation and transform helpers.
	q := dbox2d.Rot{Sin: fixed.Half(), Cos: fixed.MustParse("0.8660254038")}
	p := dbox2d.Vec2{X: fixed.FromRaw(3)}
	if got := dbox2d.InvRotateVector(q, p).Y.Raw(); got != -2 {
		t.Errorf("InvRotateVector y = %d raw, want -2: the product was negated", got)
	}
	transform := dbox2d.Transform{Q: q}
	if got := dbox2d.InvTransformPoint(transform, p).Y.Raw(); got != -2 {
		t.Errorf("InvTransformPoint y = %d raw, want -2: the product was negated", got)
	}
}

// TestNormalizeRotKeepsAZeroRotation checks that an invalid rotation stays
// invalid. The identity would hide the bad state from every later check.
func TestNormalizeRotKeepsAZeroRotation(t *testing.T) {
	got := dbox2d.NormalizeRot(dbox2d.Rot{})

	if got != (dbox2d.Rot{}) {
		t.Errorf("NormalizeRot of a zero rotation = %v, want the zero rotation", got)
	}
	if dbox2d.IsValidRotation(got) {
		t.Errorf("IsValidRotation accepted a zero rotation")
	}
	if !dbox2d.IsValidRotation(dbox2d.RotIdentity()) {
		t.Errorf("IsValidRotation rejected the identity")
	}
}

// TestComputeRotationBetweenUnitVectors covers the assertion of the
// reference, which this port keeps as a panic in every build.
func TestComputeRotationBetweenUnitVectors(t *testing.T) {
	x := dbox2d.Vec2{X: fixed.One()}
	y := dbox2d.Vec2{Y: fixed.One()}

	got := dbox2d.ComputeRotationBetweenUnitVectors(x, y)
	if angle := dbox2d.RotGetAngle(got); !near(angle, fixed.MustParse("0.25"), tol(1, 1000)) {
		t.Errorf("angle from the x axis to the y axis = %v, want 0.25 turns", angle)
	}

	defer func() {
		if recover() == nil {
			t.Errorf("a vector of length two did not panic")
		}
	}()
	dbox2d.ComputeRotationBetweenUnitVectors(x, dbox2d.Vec2{X: fixed.FromInt(2)})
}

// TestNormalizeKeepsAShortVector is the evidence for the divergence that
// replaced the reciprocal. A vector of a few raw units would collapse to
// zero if the port multiplied by one over its length.
func TestNormalizeKeepsAShortVector(t *testing.T) {
	v := dbox2d.Vec2{X: fixed.FromRaw(3), Y: fixed.FromRaw(4)}

	length, unit := dbox2d.GetLengthAndNormalize(v)
	if got := length.Raw(); got != 5 {
		t.Errorf("length = %d raw, want 5", got)
	}
	if !dbox2d.IsNormalized(unit) {
		t.Errorf("the unit vector of a short vector is not normalized: %v", unit)
	}
}
