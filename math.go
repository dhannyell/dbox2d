package dbox2d

import "github.com/dhannyell/fixed"

// The scalar, the vector and the rotation come from the fixed-point module.
// This package adds only the shapes and the operations that module lacks.
type (
	// Q is a signed Q32.32 fixed-point number.
	Q = fixed.Q

	// Vec2 is a 2D vector. It represents a point or a free vector.
	Vec2 = fixed.Vec2

	// Rot is a 2D rotation, stored as a sine and cosine pair.
	Rot = fixed.Rot
)

// Transform is a 2D rigid transform.
type Transform struct {
	P Vec2
	Q Rot
}

// Mat22 is a 2-by-2 matrix, stored as its columns.
type Mat22 struct {
	Cx, Cy Vec2
}

// Plane separates space. The separation of a point is
// dot(normal, point) - offset.
type Plane struct {
	Normal Vec2
	Offset Q
}

var (
	// upstream B2_PI 3.14159265359f
	pi = fixed.MustParse("3.14159265359")

	// One turn in radians. An angle is a turn here, so the integrators
	// that the reference writes in radians scale by this factor.
	tau = pi.Add(pi)

	// upstream 100.0f * FLT_EPSILON, about 1.2e-5. One raw unit is 2^-32.
	normalizedTolerance = fixed.FromRaw(1 << 16)

	// upstream 0.0006f, kept as it is written
	rotNormalizedTolerance = fixed.MustParse("0.0006")
)

// Pi is the ratio of a circumference to its diameter. An angle is a turn in
// this package; Pi serves the formulas that the reference writes in radians.
func Pi() Q { return pi }

// Vec2Zero returns the zero vector.
func Vec2Zero() Vec2 { return Vec2{} }

// RotIdentity returns the rotation by zero turns.
func RotIdentity() Rot { return fixed.RotIdentity() }

// TransformIdentity returns the transform that moves and rotates nothing.
func TransformIdentity() Transform {
	return Transform{P: Vec2{}, Q: fixed.RotIdentity()}
}

// Mat22Zero returns the zero matrix.
func Mat22Zero() Mat22 { return Mat22{} }

// IsValidQ reports whether a is a usable value. Fixed-point arithmetic has
// no NaN and no infinity; it saturates instead, so a saturated value is the
// signal that a computation left the representable range.
func IsValidQ(a Q) bool {
	return !a.Eq(fixed.MinValue()) && !a.Eq(fixed.MaxValue())
}

// IsValidVec2 reports whether v is a usable vector.
func IsValidVec2(v Vec2) bool {
	return IsValidQ(v.X) && IsValidQ(v.Y)
}

// IsValidRotation reports whether q is a usable and normalized rotation.
func IsValidRotation(q Rot) bool {
	if !IsValidQ(q.Sin) || !IsValidQ(q.Cos) {
		return false
	}
	return IsNormalizedRot(q)
}

// IsValidPlane reports whether a has a unit normal and a usable offset.
func IsValidPlane(a Plane) bool {
	return IsValidVec2(a.Normal) && IsNormalized(a.Normal) && IsValidQ(a.Offset)
}

// ClampInt returns a limited to the range [lower, upper].
func ClampInt(a, lower, upper int) int {
	return min(max(a, lower), upper)
}

// Cross returns the 2D cross product, which is a scalar.
func Cross(a, b Vec2) Q {
	return a.X.Mul(b.Y).Sub(a.Y.Mul(b.X))
}

// CrossVS returns the cross product of a vector and a scalar.
func CrossVS(v Vec2, s Q) Vec2 {
	return Vec2{X: s.Mul(v.Y), Y: s.Neg().Mul(v.X)}
}

// CrossSV returns the cross product of a scalar and a vector.
func CrossSV(s Q, v Vec2) Vec2 {
	return Vec2{X: s.Neg().Mul(v.Y), Y: s.Mul(v.X)}
}

// LeftPerp returns the left pointing perpendicular of v. It equals
// CrossSV(one, v).
func LeftPerp(v Vec2) Vec2 {
	return Vec2{X: v.Y.Neg(), Y: v.X}
}

// RightPerp returns the right pointing perpendicular of v. It equals
// CrossVS(v, one).
func RightPerp(v Vec2) Vec2 {
	return Vec2{X: v.Y, Y: v.X.Neg()}
}

// Neg returns the negation of a.
func Neg(a Vec2) Vec2 {
	return Vec2{X: a.X.Neg(), Y: a.Y.Neg()}
}

// Lerp interpolates between a and b by t.
//
// It weighs both ends, unlike the Lerp method of the fixed-point module,
// which adds a scaled difference. The two round differently.
func Lerp(a, b Vec2, t Q) Vec2 {
	omt := fixed.One().Sub(t)
	return Vec2{
		X: omt.Mul(a.X).Add(t.Mul(b.X)),
		Y: omt.Mul(a.Y).Add(t.Mul(b.Y)),
	}
}

// Mul returns the component-wise product of a and b.
func Mul(a, b Vec2) Vec2 {
	return Vec2{X: a.X.Mul(b.X), Y: a.Y.Mul(b.Y)}
}

// MulAdd returns a + s * b.
func MulAdd(a Vec2, s Q, b Vec2) Vec2 {
	return Vec2{X: a.X.Add(s.Mul(b.X)), Y: a.Y.Add(s.Mul(b.Y))}
}

// MulSub returns a - s * b.
func MulSub(a Vec2, s Q, b Vec2) Vec2 {
	return Vec2{X: a.X.Sub(s.Mul(b.X)), Y: a.Y.Sub(s.Mul(b.Y))}
}

// Abs returns the component-wise absolute value of a.
func Abs(a Vec2) Vec2 {
	return Vec2{X: a.X.Abs(), Y: a.Y.Abs()}
}

// Min returns the component-wise minimum of a and b.
func Min(a, b Vec2) Vec2 {
	return Vec2{X: a.X.Min(b.X), Y: a.Y.Min(b.Y)}
}

// Max returns the component-wise maximum of a and b.
func Max(a, b Vec2) Vec2 {
	return Vec2{X: a.X.Max(b.X), Y: a.Y.Max(b.Y)}
}

// Clamp returns v limited component-wise to the box [a, b].
func Clamp(v, a, b Vec2) Vec2 {
	return Vec2{X: v.X.Clamp(a.X, b.X), Y: v.Y.Clamp(a.Y, b.Y)}
}

// IsNormalized reports whether a has unit length.
func IsNormalized(a Vec2) bool {
	aa := a.Dot(a)
	return fixed.One().Sub(aa).Abs().Less(normalizedTolerance)
}

// GetLengthAndNormalize returns the length of v and the unit vector with the
// same direction. The zero vector returns a zero length and the zero vector.
func GetLengthAndNormalize(v Vec2) (Q, Vec2) {
	return v.Len(), v.Normalize()
}

// NormalizeRot rescales q to unit length.
//
// A zero q returns a zero rotation, as the reference does. The zero rotation
// is invalid, so IsValidRotation reports the bad state instead of hiding it
// behind the identity.
func NormalizeRot(q Rot) Rot {
	zero := fixed.Zero()
	if q.Sin.Eq(zero) && q.Cos.Eq(zero) {
		return Rot{}
	}
	return q.Normalize()
}

// IntegrateRotation advances q1 by the angular displacement deltaAngle, in
// turns, and renormalizes. The first order step below needs radians, so the
// displacement scales by one turn.
func IntegrateRotation(q1 Rot, deltaAngle Q) Rot {
	// dc/dt = -omega * sin(t)
	// ds/dt = omega * cos(t)
	// c2 = c1 - omega * h * s1
	// s2 = s1 + omega * h * c1
	d := deltaAngle.Mul(tau)
	q2 := Rot{
		Cos: q1.Cos.Sub(d.Mul(q1.Sin)),
		Sin: q1.Sin.Add(d.Mul(q1.Cos)),
	}
	return NormalizeRot(q2)
}

// MakeRot returns the rotation by the angle t, in turns.
func MakeRot(t Q) Rot {
	return fixed.RotFromTurns(t)
}

// ComputeRotationBetweenUnitVectors returns the rotation that carries the
// unit vector v1 onto the unit vector v2. It panics when either vector is not
// a unit vector, because the result would be a rotation of the wrong scale.
func ComputeRotationBetweenUnitVectors(v1, v2 Vec2) Rot {
	if !IsNormalized(v1) || !IsNormalized(v2) {
		panic("dbox2d: ComputeRotationBetweenUnitVectors needs two unit vectors")
	}

	rot := Rot{Cos: v1.Dot(v2), Sin: Cross(v1, v2)}
	return NormalizeRot(rot)
}

// IsNormalizedRot reports whether q has unit length.
func IsNormalizedRot(q Rot) bool {
	qq := q.Sin.Mul(q.Sin).Add(q.Cos.Mul(q.Cos))
	one := fixed.One()
	return one.Sub(rotNormalizedTolerance).Less(qq) && qq.Less(one.Add(rotNormalizedTolerance))
}

// NLerp interpolates between q1 and q2 by t and renormalizes.
func NLerp(q1, q2 Rot, t Q) Rot {
	omt := fixed.One().Sub(t)
	q := Rot{
		Cos: omt.Mul(q1.Cos).Add(t.Mul(q2.Cos)),
		Sin: omt.Mul(q1.Sin).Add(t.Mul(q2.Sin)),
	}
	return NormalizeRot(q)
}

// ComputeAngularVelocity returns the angular velocity, in turns per second,
// that rotates q1 into q2 over one step. invH is the inverse time step.
func ComputeAngularVelocity(q1, q2 Rot, invH Q) Q {
	// ds/dt = omega * cos(t)
	// dc/dt = -omega * sin(t)
	// s2 = s1 + omega * h * c1
	// c2 = c1 - omega * h * s1

	// omega * h * s1 = c1 - c2
	// omega * h * c1 = s2 - s1
	// omega * h = (c1 - c2) * s1 + (s2 - s1) * c1
	// omega * h = s1 * c1 - c2 * s1 + s2 * c1 - s1 * c1
	// omega * h = s2 * c1 - c2 * s1 = sin(a2 - a1), about a2 - a1 for a small delta
	omega := invH.Mul(q2.Sin.Mul(q1.Cos).Sub(q2.Cos.Mul(q1.Sin)))
	return omega.Div(tau)
}

// RotGetAngle returns the angle of q in turns, in the range [-0.5, 0.5].
func RotGetAngle(q Rot) Q {
	return fixed.Atan2Turns(q.Sin, q.Cos)
}

// RotGetXAxis returns the x axis of q.
func RotGetXAxis(q Rot) Vec2 {
	return Vec2{X: q.Cos, Y: q.Sin}
}

// RotGetYAxis returns the y axis of q.
func RotGetYAxis(q Rot) Vec2 {
	return Vec2{X: q.Sin.Neg(), Y: q.Cos}
}

// MulRot returns the rotation q * r.
//
//	[qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
//	[qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
//	s(q + r) = qs * rc + qc * rs
//	c(q + r) = qc * rc - qs * rs
func MulRot(q, r Rot) Rot {
	return q.Mul(r)
}

// InvMulRot returns the rotation transpose(q) * r.
//
//	[ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
//	[-qs qc]   [rs  rc]   [-qs*rc+qc*rs qs*rs+qc*rc]
//	s(q - r) = qc * rs - qs * rc
//	c(q - r) = qc * rc + qs * rs
func InvMulRot(q, r Rot) Rot {
	return Rot{
		Sin: q.Cos.Mul(r.Sin).Sub(q.Sin.Mul(r.Cos)),
		Cos: q.Cos.Mul(r.Cos).Add(q.Sin.Mul(r.Sin)),
	}
}

// RelativeAngle returns the angle of b relative to a, in turns.
func RelativeAngle(b, a Rot) Q {
	// sin(b - a) = bs * ac - bc * as
	// cos(b - a) = bc * ac + bs * as
	s := b.Sin.Mul(a.Cos).Sub(b.Cos.Mul(a.Sin))
	c := b.Cos.Mul(a.Cos).Add(b.Sin.Mul(a.Sin))
	return fixed.Atan2Turns(s, c)
}

// UnwindAngle reduces an angle in turns to the range [-0.5, 0.5]. One turn
// is one unit here, so the reduction is exact and needs no remainder of pi.
func UnwindAngle(t Q) Q {
	return t.Sub(t.Round())
}

// RotateVector rotates v by q.
func RotateVector(q Rot, v Vec2) Vec2 {
	return q.Apply(v)
}

// InvRotateVector rotates v by the inverse of q.
func InvRotateVector(q Rot, v Vec2) Vec2 {
	return Vec2{
		X: q.Cos.Mul(v.X).Add(q.Sin.Mul(v.Y)),
		Y: q.Sin.Neg().Mul(v.X).Add(q.Cos.Mul(v.Y)),
	}
}

// TransformPoint carries the point p from the frame of t into the parent
// frame, for example from local space into world space.
func TransformPoint(t Transform, p Vec2) Vec2 {
	x := t.Q.Cos.Mul(p.X).Sub(t.Q.Sin.Mul(p.Y)).Add(t.P.X)
	y := t.Q.Sin.Mul(p.X).Add(t.Q.Cos.Mul(p.Y)).Add(t.P.Y)
	return Vec2{X: x, Y: y}
}

// InvTransformPoint carries the point p from the parent frame into the frame
// of t, for example from world space into local space.
func InvTransformPoint(t Transform, p Vec2) Vec2 {
	vx := p.X.Sub(t.P.X)
	vy := p.Y.Sub(t.P.Y)
	return Vec2{
		X: t.Q.Cos.Mul(vx).Add(t.Q.Sin.Mul(vy)),
		Y: t.Q.Sin.Neg().Mul(vx).Add(t.Q.Cos.Mul(vy)),
	}
}

// MulTransforms composes two transforms. Applied to a point local to frame
// b, the result converts it to a point local to frame a, then to a point in
// the world frame.
//
//	v2 = a.q.Rot(b.q.Rot(v1) + b.p) + a.p
//	   = (a.q * b.q).Rot(v1) + a.q.Rot(b.p) + a.p
func MulTransforms(a, b Transform) Transform {
	var c Transform
	c.Q = MulRot(a.Q, b.Q)
	c.P = RotateVector(a.Q, b.P).Add(a.P)
	return c
}

// InvMulTransforms returns the transform that converts a point local to
// frame b into a point local to frame a.
//
//	v2 = inv(a.q) * (b.q * v1 + b.p - a.p)
//	   = inv(a.q) * b.q * v1 + inv(a.q) * (b.p - a.p)
func InvMulTransforms(a, b Transform) Transform {
	var c Transform
	c.Q = InvMulRot(a.Q, b.Q)
	c.P = InvRotateVector(a.Q, b.P.Sub(a.P))
	return c
}

// MulMV returns the product of the matrix m and the vector v.
func MulMV(m Mat22, v Vec2) Vec2 {
	return Vec2{
		X: m.Cx.X.Mul(v.X).Add(m.Cy.X.Mul(v.Y)),
		Y: m.Cx.Y.Mul(v.X).Add(m.Cy.Y.Mul(v.Y)),
	}
}

// GetInverse22 returns the inverse of m. A singular matrix returns the zero
// matrix.
//
// Each entry divides by the determinant, because a fixed-point reciprocal
// loses the precision that the division keeps. See DIVERGENCES.md.
func GetInverse22(m Mat22) Mat22 {
	a, b, c, d := m.Cx.X, m.Cy.X, m.Cx.Y, m.Cy.Y
	det := a.Mul(d).Sub(b.Mul(c))
	if det.Eq(fixed.Zero()) {
		return Mat22{}
	}

	return Mat22{
		Cx: Vec2{X: d.Div(det), Y: c.Neg().Div(det)},
		Cy: Vec2{X: b.Neg().Div(det), Y: a.Div(det)},
	}
}

// Solve22 solves m * x = b for the column vector b. It is cheaper than the
// inverse for a one-shot case. A singular matrix returns the zero vector.
func Solve22(m Mat22, b Vec2) Vec2 {
	a11, a12, a21, a22 := m.Cx.X, m.Cy.X, m.Cx.Y, m.Cy.Y
	det := a11.Mul(a22).Sub(a12.Mul(a21))
	if det.Eq(fixed.Zero()) {
		return Vec2{}
	}

	return Vec2{
		X: a22.Mul(b.X).Sub(a12.Mul(b.Y)).Div(det),
		Y: a11.Mul(b.Y).Sub(a21.Mul(b.X)).Div(det),
	}
}

// PlaneSeparation returns the signed distance of a point from a plane.
func PlaneSeparation(plane Plane, point Vec2) Q {
	return plane.Normal.Dot(point).Sub(plane.Offset)
}

// SpringDamper simulates a one dimensional mass-spring-damper and returns
// the new velocity. The caller then computes the new position:
//
//	position += timeStep * newVelocity
//
// It drives towards a zero position. Implicit integration makes the solution
// stable and free of transcendental functions.
func SpringDamper(hertz, dampingRatio, position, velocity, timeStep Q) Q {
	omega := tau.Mul(hertz)
	omegaH := omega.Mul(timeStep)
	num := velocity.Sub(omega.Mul(omegaH).Mul(position))
	den := fixed.One().Add(fixed.FromInt(2).Mul(dampingRatio).Mul(omegaH)).Add(omegaH.Mul(omegaH))
	if den.Eq(fixed.Zero()) {
		return velocity
	}
	return num.Div(den)
}
