package b2

import (
	"math"
	"testing"
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
	return f64Atan2(s, c)
}

func f64UnwindAngle(radians float64) float64 {
	if radians < -math.Pi {
		return radians + 2*math.Pi
	} else if radians > math.Pi {
		return radians - 2*math.Pi
	}
	return radians
}

// f64Frame mirrors a joint frame: a position and a rotation.
type f64Frame struct {
	p    f64Vec
	c, s float64
}

func frameToF64(t Transform) f64Frame {
	return f64Frame{vecToF64(t.P), qToF64(t.Q.Cos), qToF64(t.Q.Sin)}
}

// f64MulRot mirrors b2MulRot: the rotation q * r.
func f64MulRot(qc, qs, rc, rs float64) (float64, float64) {
	return qc*rc - qs*rs, qs*rc + qc*rs
}

// f64FrameAngle mirrors b2Rot_GetAngle(b2InvMulRot(qA, qB)) with
// qA = dqA * frameA.q and qB = dqB * frameB.q.
func f64FrameAngle(stateA, stateB *f64State, frameA, frameB f64Frame) float64 {
	ac, as := f64MulRot(stateA.dqc, stateA.dqs, frameA.c, frameA.s)
	bc, bs := f64MulRot(stateB.dqc, stateB.dqs, frameB.c, frameB.s)
	return f64RelativeAngle(bc, bs, ac, as)
}

type f64RevoluteJoint struct {
	mA, mB, iA, iB     float64
	constraintSoftness f64Softness

	linearImpulse                                           f64Vec
	springImpulse, motorImpulse, lowerImpulse, upperImpulse float64
	hertz, dampingRatio, targetAngle, maxMotorTorque        float64
	motorSpeed, lowerAngle, upperAngle                      float64
	frameA, frameB                                          f64Frame
	deltaCenter                                             f64Vec
	axialMass                                               float64
	springSoftness                                          f64Softness
	enableSpring, enableMotor, enableLimit                  bool
}

// f64SolveRevoluteJoint mirrors b2SolveRevoluteJoint of v3.2: the axial
// impulses feed forward through angularCdot and land with the point
// constraint.
func f64SolveRevoluteJoint(joint *f64RevoluteJoint, stateA, stateB *f64State, h, invH float64, useBias bool) {
	mA, mB, iA, iB := joint.mA, joint.mB, joint.iA, joint.iB

	vA, wA := stateA.v, stateA.w
	vB, wB := stateB.v, stateB.w

	jointAngle := f64FrameAngle(stateA, stateB, joint.frameA, joint.frameB)

	rA := f64Rotate(stateA.dqc, stateA.dqs, joint.frameA.p)
	rB := f64Rotate(stateB.dqc, stateB.dqs, joint.frameB.p)

	linearCdot := f64Sub(f64Add(vB, f64CrossSV(wB, rB)), f64Add(vA, f64CrossSV(wA, rA)))
	angularCdot := wB - wA
	axialImpulse := 0.0

	k := iA + iB
	fixedRotation := k == 0

	if joint.enableSpring && !fixedRotation {
		C := f64UnwindAngle(jointAngle - joint.targetAngle)
		bias := joint.springSoftness.biasRate * C
		massScale := joint.springSoftness.massScale
		impulseScale := joint.springSoftness.impulseScale

		impulse := -massScale*joint.axialMass*(angularCdot+bias) - impulseScale*joint.springImpulse
		joint.springImpulse += impulse

		angularCdot += k * impulse
		axialImpulse += impulse
	}

	if joint.enableMotor && !fixedRotation {
		Cdot := angularCdot - joint.motorSpeed
		oldImpulse := joint.motorImpulse
		maxImpulse := h * joint.maxMotorTorque
		impulse := math.Max(-maxImpulse-oldImpulse, math.Min(-joint.axialMass*Cdot, maxImpulse-oldImpulse))
		joint.motorImpulse = oldImpulse + impulse

		angularCdot += k * impulse
		axialImpulse += impulse
	}

	if joint.enableLimit && !fixedRotation {
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

			oldImpulse := joint.lowerImpulse
			impulse := math.Max(-massScale*joint.axialMass*(angularCdot+bias)-impulseScale*oldImpulse, -oldImpulse)
			joint.lowerImpulse = oldImpulse + impulse

			angularCdot += k * impulse
			axialImpulse += impulse
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

			oldImpulse := joint.upperImpulse
			impulse := math.Max(-massScale*joint.axialMass*(-angularCdot+bias)-impulseScale*oldImpulse, -oldImpulse)
			joint.upperImpulse = oldImpulse + impulse

			axialImpulse -= impulse
		}
	}

	{
		Cdot := f64Add(linearCdot, f64Scale(axialImpulse, f64Add(f64CrossSV(iA, rA), f64CrossSV(iB, rB))))

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
		wA -= iA * (rA.x*impulse.y - rA.y*impulse.x + axialImpulse)
		vB = f64Vec{vB.x + mB*impulse.x, vB.y + mB*impulse.y}
		wB += iB * (rB.x*impulse.y - rB.y*impulse.x + axialImpulse)
	}

	stateA.v, stateA.w = vA, wA
	stateB.v, stateB.w = vB, wB
}

func vecToF64(v Vec2) f64Vec { return f64Vec{qToF64(v.X), qToF64(v.Y)} }

func stateToF64(s *bodyState) f64State {
	return f64State{
		v:   vecToF64(s.linearVelocity),
		w:   qToF64(s.angularVelocity),
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
		def.Rotation = MakeRot(QMustParse(turns))
		id := CreateBody(worldId, &def)
		shapeDef := DefaultShapeDef()
		box := MakeBox(QMustParse("0.75"), QHalf())
		CreatePolygonShape(id, &shapeDef, &box)
		return id
	}
	idA := makeBody(qv("0", "0"), "0.03")
	idB := makeBody(qv("1.5", "0.5"), "-0.04")

	def := DefaultRevoluteJointDef()
	def.Base.BodyIdA, def.Base.BodyIdB = idA, idB
	def.Base.LocalFrameA.P = qv("0.5", "0.25")
	def.Base.LocalFrameB.P = qv("-1", "0.25")
	def.EnableSpring = true
	def.Hertz = QFromInt(2)
	def.DampingRatio = QHalf()
	def.TargetAngle = QMustParse("0.05")
	def.EnableLimit = true
	def.LowerAngle = QMustParse("-0.1")
	def.UpperAngle = QMustParse("0.1")
	def.EnableMotor = true
	def.MotorSpeed = QHalf()
	def.MaxMotorTorque = QFromInt(3)
	jointId := CreateRevoluteJoint(worldId, &def)
	j := getJointFullId(w, jointId)
	js := getJointSim(w, j)

	bodyA := getBodyFullId(w, idA)
	bodyB := getBodyFullId(w, idB)
	stateA := getBodyState(w, bodyA)
	stateB := getBodyState(w, bodyB)
	stateA.linearVelocity = qv("1", "-2")
	stateA.angularVelocity = QMustParse("0.3")
	stateA.deltaPosition = qv("0.01", "0.02")
	stateA.deltaRotation = MakeRot(QMustParse("0.01"))
	stateB.linearVelocity = qv("-0.5", "1")
	stateB.angularVelocity = QMustParse("-0.2")
	stateB.deltaPosition = qv("-0.03", "0.01")
	stateB.deltaRotation = MakeRot(QMustParse("-0.02"))

	context := jointContext(w)
	prepareJoint(js, context)

	h := qToF64(context.h)
	invH := qToF64(context.invH)
	r := js.revolute()
	mirror := &f64RevoluteJoint{
		mA: qToF64(js.invMassA), mB: qToF64(js.invMassB), iA: qToF64(js.invIA), iB: qToF64(js.invIB),
		constraintSoftness: makeSoftF64(math.Min(qToF64(js.constraintHertz), 0.25*invH), qToF64(js.constraintDampingRatio), h),
		hertz:              qToF64(r.hertz),
		dampingRatio:       qToF64(r.dampingRatio),
		targetAngle:        radiansF64(r.targetAngle),
		maxMotorTorque:     qToF64(r.maxMotorTorque),
		motorSpeed:         radiansF64(r.motorSpeed),
		lowerAngle:         radiansF64(r.lowerAngle),
		upperAngle:         radiansF64(r.upperAngle),
		frameA:             frameToF64(r.frameA),
		frameB:             frameToF64(r.frameB),
		deltaCenter:        vecToF64(r.deltaCenter),
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
	r := js.revolute()

	solveRevoluteJoint(js, context, true)
	f64SolveRevoluteJoint(mirror, &fA, &fB, qToF64(context.h), qToF64(context.invH), true)

	const limit = 1e-5
	check := func(name string, got Q, want float64) {
		t.Helper()
		diff := math.Abs(qToF64(got) - want)
		if diff > mirrorTolerance(limit, want) {
			t.Errorf("%s: Q %v, float64 %v, diff %g", name, qToF64(got), want, diff)
		}
	}
	check("vA.x", stateA.linearVelocity.X, fA.v.x)
	check("vA.y", stateA.linearVelocity.Y, fA.v.y)
	check("wA", stateA.angularVelocity, fA.w)
	check("vB.x", stateB.linearVelocity.X, fB.v.x)
	check("vB.y", stateB.linearVelocity.Y, fB.v.y)
	check("wB", stateB.angularVelocity, fB.w)
	check("springImpulse", r.springImpulse, mirror.springImpulse)
	check("motorImpulse", r.motorImpulse, mirror.motorImpulse)
	check("lowerImpulse", r.lowerImpulse, mirror.lowerImpulse)
	check("upperImpulse", r.upperImpulse, mirror.upperImpulse)
	check("linearImpulse.x", r.linearImpulse.X, mirror.linearImpulse.x)
	check("linearImpulse.y", r.linearImpulse.Y, mirror.linearImpulse.y)
}
