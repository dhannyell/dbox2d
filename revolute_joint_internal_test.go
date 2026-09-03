package dbox2d

import (
	"math"
	"testing"

	"github.com/dhannyell/fixed"
)

// The float64 mirror of the revolute solve follows src/revolute_joint.c
// of the reference line by line, in radians. It takes the prepared joint
// data and one body state per side and runs one solve iteration.

// f64State mirrors the body state fields that the solve reads, with the
// angular velocity in radians per second.
type f64State struct {
	v   f64Vec
	w   float64
	dp  f64Vec
	dqc float64
	dqs float64
}

func f64Rotate(c, s float64, v f64Vec) f64Vec {
	return f64Vec{c*v.x - s*v.y, s*v.x + c*v.y}
}

// f64RelativeAngle mirrors b2RelativeAngle: the angle of b relative to a.
func f64RelativeAngle(bc, bs, ac, as float64) float64 {
	s := bs*ac - bc*as
	c := bc*ac + bs*as
	return math.Atan2(s, c)
}

func f64UnwindAngle(radians float64) float64 {
	if radians < -math.Pi {
		return radians + 2*math.Pi
	} else if radians > math.Pi {
		return radians - 2*math.Pi
	}
	return radians
}

type f64RevoluteJoint struct {
	mA, mB, iA, iB     float64
	constraintSoftness f64Softness

	linearImpulse                                           f64Vec
	springImpulse, motorImpulse, lowerImpulse, upperImpulse float64
	hertz, dampingRatio, targetAngle, maxMotorTorque        float64
	motorSpeed, referenceAngle, lowerAngle, upperAngle      float64
	anchorA, anchorB, deltaCenter                           f64Vec
	deltaAngle, axialMass                                   float64
	springSoftness                                          f64Softness
	enableSpring, enableMotor, enableLimit                  bool
}

// f64SolveRevoluteJoint mirrors b2SolveRevoluteJoint with useBias.
func f64SolveRevoluteJoint(joint *f64RevoluteJoint, stateA, stateB *f64State, h, invH float64, useBias bool) {
	mA, mB, iA, iB := joint.mA, joint.mB, joint.iA, joint.iB

	vA, wA := stateA.v, stateA.w
	vB, wB := stateB.v, stateB.w

	fixedRotation := iA+iB == 0

	if joint.enableSpring && !fixedRotation {
		jointAngle := f64RelativeAngle(stateB.dqc, stateB.dqs, stateA.dqc, stateA.dqs) + joint.deltaAngle
		jointAngleDelta := f64UnwindAngle(jointAngle - joint.targetAngle)

		C := jointAngleDelta
		bias := joint.springSoftness.biasRate * C
		massScale := joint.springSoftness.massScale
		impulseScale := joint.springSoftness.impulseScale

		Cdot := wB - wA
		impulse := -massScale*joint.axialMass*(Cdot+bias) - impulseScale*joint.springImpulse
		joint.springImpulse += impulse

		wA -= iA * impulse
		wB += iB * impulse
	}

	if joint.enableMotor && !fixedRotation {
		Cdot := wB - wA - joint.motorSpeed
		impulse := -joint.axialMass * Cdot
		oldImpulse := joint.motorImpulse
		maxImpulse := h * joint.maxMotorTorque
		joint.motorImpulse = math.Max(-maxImpulse, math.Min(joint.motorImpulse+impulse, maxImpulse))
		impulse = joint.motorImpulse - oldImpulse

		wA -= iA * impulse
		wB += iB * impulse
	}

	if joint.enableLimit && !fixedRotation {
		jointAngle := f64RelativeAngle(stateB.dqc, stateB.dqs, stateA.dqc, stateA.dqs) + joint.deltaAngle - joint.referenceAngle
		jointAngle = f64UnwindAngle(jointAngle)

		{
			C := jointAngle - joint.lowerAngle
			bias := 0.0
			massScale := 1.0
			impulseScale := 0.0
			if C > 0 {
				bias = C * invH
			} else if useBias {
				bias = joint.constraintSoftness.biasRate * C
				massScale = joint.constraintSoftness.massScale
				impulseScale = joint.constraintSoftness.impulseScale
			}

			Cdot := wB - wA
			oldImpulse := joint.lowerImpulse
			impulse := -massScale*joint.axialMass*(Cdot+bias) - impulseScale*oldImpulse
			joint.lowerImpulse = math.Max(oldImpulse+impulse, 0)
			impulse = joint.lowerImpulse - oldImpulse

			wA -= iA * impulse
			wB += iB * impulse
		}

		{
			C := joint.upperAngle - jointAngle
			bias := 0.0
			massScale := 1.0
			impulseScale := 0.0
			if C > 0 {
				bias = C * invH
			} else if useBias {
				bias = joint.constraintSoftness.biasRate * C
				massScale = joint.constraintSoftness.massScale
				impulseScale = joint.constraintSoftness.impulseScale
			}

			Cdot := wA - wB
			oldImpulse := joint.upperImpulse
			impulse := -massScale*joint.axialMass*(Cdot+bias) - impulseScale*oldImpulse
			joint.upperImpulse = math.Max(oldImpulse+impulse, 0)
			impulse = joint.upperImpulse - oldImpulse

			wA += iA * impulse
			wB -= iB * impulse
		}
	}

	{
		rA := f64Rotate(stateA.dqc, stateA.dqs, joint.anchorA)
		rB := f64Rotate(stateB.dqc, stateB.dqs, joint.anchorB)

		Cdot := f64Vec{vB.x - wB*rB.y - (vA.x - wA*rA.y), vB.y + wB*rB.x - (vA.y + wA*rA.x)}

		bias := f64Vec{}
		massScale := 1.0
		impulseScale := 0.0
		if useBias {
			dcA := stateA.dp
			dcB := stateB.dp
			separation := f64Vec{dcB.x - dcA.x + rB.x - rA.x + joint.deltaCenter.x, dcB.y - dcA.y + rB.y - rA.y + joint.deltaCenter.y}
			bias = f64Vec{joint.constraintSoftness.biasRate * separation.x, joint.constraintSoftness.biasRate * separation.y}
			massScale = joint.constraintSoftness.massScale
			impulseScale = joint.constraintSoftness.impulseScale
		}

		k11 := mA + mB + rA.y*rA.y*iA + rB.y*rB.y*iB
		k12 := -rA.y*rA.x*iA - rB.y*rB.x*iB
		k22 := mA + mB + rA.x*rA.x*iA + rB.x*rB.x*iB
		det := k11*k22 - k12*k12
		if det != 0 {
			det = 1 / det
		}
		rhs := f64Vec{Cdot.x + bias.x, Cdot.y + bias.y}
		b := f64Vec{det * (k22*rhs.x - k12*rhs.y), det * (k11*rhs.y - k12*rhs.x)}

		impulse := f64Vec{-massScale*b.x - impulseScale*joint.linearImpulse.x, -massScale*b.y - impulseScale*joint.linearImpulse.y}
		joint.linearImpulse.x += impulse.x
		joint.linearImpulse.y += impulse.y

		vA = f64Vec{vA.x - mA*impulse.x, vA.y - mA*impulse.y}
		wA -= iA * (rA.x*impulse.y - rA.y*impulse.x)
		vB = f64Vec{vB.x + mB*impulse.x, vB.y + mB*impulse.y}
		wB += iB * (rB.x*impulse.y - rB.y*impulse.x)
	}

	stateA.v, stateA.w = vA, wA
	stateB.v, stateB.w = vB, wB
}

func vecToF64(v Vec2) f64Vec { return f64Vec{qToF64(v.X), qToF64(v.Y)} }

func stateToF64(s *bodyState) f64State {
	return f64State{
		v:   vecToF64(s.linearVelocity),
		w:   qToF64(s.angularVelocity) * 2 * math.Pi,
		dp:  vecToF64(s.deltaPosition),
		dqc: qToF64(s.deltaRotation.Cos),
		dqs: qToF64(s.deltaRotation.Sin),
	}
}

// revoluteMirrorCase builds the mirror pair: a joint with the spring, the
// motor and both limits armed, on two rotated and displaced states. The
// bench shares it with the mirror test.
type revoluteMirrorCase struct {
	js             *jointSim
	context        *stepContext
	stateA, stateB *bodyState
	mirror         *f64RevoluteJoint
	fA, fB         f64State
}

func makeRevoluteMirrorCase(tb testing.TB) revoluteMirrorCase {
	tb.Helper()
	worldId := createTestWorld(tb)
	w := getWorldFromId(worldId)

	makeBody := func(position Vec2, turns string) BodyId {
		def := DefaultBodyDef()
		def.Type = DynamicBody
		def.Position = position
		def.Rotation = fixed.RotFromTurns(fixed.Q32MustParse(turns))
		id := CreateBody(worldId, &def)
		shapeDef := DefaultShapeDef()
		box := MakeBox(fixed.Q32MustParse("0.75"), fixed.Q32Half())
		CreatePolygonShape(id, &shapeDef, &box)
		return id
	}
	idA := makeBody(qv("0", "0"), "0.03")
	idB := makeBody(qv("1.5", "0.5"), "-0.04")

	def := DefaultRevoluteJointDef()
	def.BodyIdA, def.BodyIdB = idA, idB
	def.LocalAnchorA = qv("0.5", "0.25")
	def.LocalAnchorB = qv("-1", "0.25")
	def.EnableSpring = true
	def.Hertz = fixed.Q32FromInt(2)
	def.DampingRatio = fixed.Q32Half()
	def.TargetAngle = fixed.Q32MustParse("0.05")
	def.EnableLimit = true
	def.LowerAngle = fixed.Q32MustParse("-0.1")
	def.UpperAngle = fixed.Q32MustParse("0.1")
	def.EnableMotor = true
	def.MotorSpeed = fixed.Q32Half()
	def.MaxMotorTorque = fixed.Q32FromInt(3)
	jointId := CreateRevoluteJoint(worldId, &def)
	j := getJointFullId(w, jointId)
	js := getJointSim(w, j)

	bodyA := getBodyFullId(w, idA)
	bodyB := getBodyFullId(w, idB)
	stateA := getBodyState(w, bodyA)
	stateB := getBodyState(w, bodyB)
	stateA.linearVelocity = qv("1", "-2")
	stateA.angularVelocity = fixed.Q32MustParse("0.3")
	stateA.deltaPosition = qv("0.01", "0.02")
	stateA.deltaRotation = fixed.RotFromTurns(fixed.Q32MustParse("0.01"))
	stateB.linearVelocity = qv("-0.5", "1")
	stateB.angularVelocity = fixed.Q32MustParse("-0.2")
	stateB.deltaPosition = qv("-0.03", "0.01")
	stateB.deltaRotation = fixed.RotFromTurns(fixed.Q32MustParse("-0.02"))

	context := jointContext(w)
	prepareJoint(js, context)

	h := qToF64(context.h)
	invH := qToF64(context.invH)
	r := &js.revoluteJoint
	mirror := &f64RevoluteJoint{
		mA: qToF64(js.invMassA), mB: qToF64(js.invMassB), iA: qToF64(js.invIA), iB: qToF64(js.invIB),
		constraintSoftness: makeSoftF64(math.Min(qToF64(js.constraintHertz), 0.25*invH), qToF64(js.constraintDampingRatio), h),
		hertz:              qToF64(r.hertz),
		dampingRatio:       qToF64(r.dampingRatio),
		targetAngle:        qToF64(r.targetAngle) * 2 * math.Pi,
		maxMotorTorque:     qToF64(r.maxMotorTorque),
		motorSpeed:         qToF64(r.motorSpeed) * 2 * math.Pi,
		referenceAngle:     qToF64(r.referenceAngle) * 2 * math.Pi,
		lowerAngle:         qToF64(r.lowerAngle) * 2 * math.Pi,
		upperAngle:         qToF64(r.upperAngle) * 2 * math.Pi,
		anchorA:            vecToF64(r.anchorA),
		anchorB:            vecToF64(r.anchorB),
		deltaCenter:        vecToF64(r.deltaCenter),
		deltaAngle:         qToF64(r.deltaAngle) * 2 * math.Pi,
		enableSpring:       true,
		enableMotor:        true,
		enableLimit:        true,
	}
	mirror.axialMass = 1 / (mirror.iA + mirror.iB)
	mirror.springSoftness = makeSoftF64(mirror.hertz, mirror.dampingRatio, h)
	fA := stateToF64(stateA)
	fB := stateToF64(stateB)
	return revoluteMirrorCase{js, context, stateA, stateB, mirror, fA, fB}
}

// TestSolveRevoluteJointTracksTheFloat64Mirror runs one biased solve on
// the mirror case. The fixed atan2 is exact to 2^-20 turn, which the
// spring and limit biases amplify to a few 1e-6, so the bound is 1e-5.
func TestSolveRevoluteJointTracksTheFloat64Mirror(t *testing.T) {
	c := makeRevoluteMirrorCase(t)
	js, context, stateA, stateB, mirror, fA, fB := c.js, c.context, c.stateA, c.stateB, c.mirror, c.fA, c.fB
	r := &js.revoluteJoint

	solveRevoluteJoint(js, context, true)
	f64SolveRevoluteJoint(mirror, &fA, &fB, qToF64(context.h), qToF64(context.invH), true)

	const limit = 1e-5
	check := func(name string, got Q, want float64) {
		t.Helper()
		if diff := math.Abs(qToF64(got) - want); diff > limit {
			t.Errorf("%s: Q %v, float64 %v, diff %g", name, qToF64(got), want, diff)
		}
	}
	check("vA.x", stateA.linearVelocity.X, fA.v.x)
	check("vA.y", stateA.linearVelocity.Y, fA.v.y)
	check("wA", stateA.angularVelocity.Mul(tau), fA.w)
	check("vB.x", stateB.linearVelocity.X, fB.v.x)
	check("vB.y", stateB.linearVelocity.Y, fB.v.y)
	check("wB", stateB.angularVelocity.Mul(tau), fB.w)
	check("springImpulse", r.springImpulse, mirror.springImpulse)
	check("motorImpulse", r.motorImpulse, mirror.motorImpulse)
	check("lowerImpulse", r.lowerImpulse, mirror.lowerImpulse)
	check("upperImpulse", r.upperImpulse, mirror.upperImpulse)
	check("linearImpulse.x", r.linearImpulse.X, mirror.linearImpulse.x)
	check("linearImpulse.y", r.linearImpulse.Y, mirror.linearImpulse.y)
}
