package dbox2d

import (
	"math"
	"testing"

	"github.com/dhannyell/fixed"
)

// This file tests the prismatic joint solver with hand-computed cases and
// with a float64 mirror of src/prismatic_joint.c. The composite slider
// lives in step_test.go.

// slidingBox creates a static ground at the origin and a unit box of mass
// one (inverse inertia 6) at the position, on a prismatic joint along the
// x axis between the two body origins. The effective axial mass is one.
func slidingBox(t *testing.T, worldId WorldId, position Vec2, def *PrismaticJointDef) (*body, *joint) {
	t.Helper()
	w := getWorldFromId(worldId)

	groundDef := DefaultBodyDef()
	groundId := CreateBody(worldId, &groundDef)

	boxDef := DefaultBodyDef()
	boxDef.Type = DynamicBody
	boxDef.Position = position
	boxId := CreateBody(worldId, &boxDef)
	shapeDef := DefaultShapeDef()
	unit := MakeBox(fixed.Q32Half(), fixed.Q32Half())
	CreatePolygonShape(boxId, &shapeDef, &unit)

	def.BodyIdA = groundId
	def.BodyIdB = boxId
	jointId := CreatePrismaticJoint(worldId, def)
	return getBodyFullId(w, boxId), getJointFullId(w, jointId)
}

// TestPrismaticBlockHoldsTheLine pins the block solver: the box at the
// origin moves across the axis at one unit per second and spins at one
// radian per second. With s1 = s2 = 0:
//
//	K = [1 0; 0 6], Cdot = (1, 1), b = (1, 1/6), impulse = (-1, -1/6)
//	vB = 0, wB = 1 + 6 * (-1/6) = 0
func TestPrismaticBlockHoldsTheLine(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	def := DefaultPrismaticJointDef()
	box, j := slidingBox(t, worldId, v2(0, 0), &def)
	state := getBodyState(w, box)
	state.linearVelocity = Vec2{Y: fixed.Q32One()}
	state.angularVelocity = fixed.Q32One().Div(tau)

	context := jointContext(w)
	prepareJoints(context, j.colorIndex)
	solveJoints(context, j.colorIndex, false)

	tolerance := fixed.Q32FromRaw(1 << 12)
	js := getJointSim(w, j)
	impulse := js.prismaticJoint.impulse
	if !withinQ(impulse.X, fixed.Q32One().Neg(), tolerance) || !withinQ(impulse.Y, fixed.Q32FromRatio(-1, 6), tolerance) {
		t.Errorf("impulse is %v, want (-1, -1/6)", impulse)
	}
	if !withinQ(state.linearVelocity.Y, fixed.Q32Zero(), tolerance) || !withinQ(state.angularVelocity, fixed.Q32Zero(), tolerance) {
		t.Errorf("the box still moves: v %v, w %v", state.linearVelocity, state.angularVelocity)
	}

	// The torque report divides the angular impulse by the sub-step: -40.
	w.invH = context.invH
	if torque := getPrismaticJointTorque(w, js); !withinQ(torque, fixed.Q32FromInt(-40), tolerance.Mul(context.invH)) {
		t.Errorf("the joint torque is %v, want -40", torque)
	}
}

// TestPrismaticUpperLimitStopsTheSlide pins the limit branch: the box sits
// at the upper translation one and slides outward at one unit per second.
//
//	lower: C = 1 > 0, bias = 240, impulse clamped to 0
//	upper: C = 0, Cdot = -1, impulse = 1, vB.x -= 1 = 0
func TestPrismaticUpperLimitStopsTheSlide(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	def := DefaultPrismaticJointDef()
	def.EnableLimit = true
	def.LowerTranslation = fixed.Q32Zero()
	def.UpperTranslation = fixed.Q32One()
	box, j := slidingBox(t, worldId, v2(1, 0), &def)
	state := getBodyState(w, box)
	state.linearVelocity = Vec2{X: fixed.Q32One()}

	context := jointContext(w)
	prepareJoints(context, j.colorIndex)
	solveJoints(context, j.colorIndex, false)

	tolerance := fixed.Q32FromRaw(1 << 12)
	js := getJointSim(w, j)
	if !js.prismaticJoint.lowerImpulse.Eq(fixed.Q32Zero()) || !withinQ(js.prismaticJoint.upperImpulse, fixed.Q32One(), tolerance) {
		t.Errorf("the limit impulses are %v and %v, want 0 and 1", js.prismaticJoint.lowerImpulse, js.prismaticJoint.upperImpulse)
	}
	if !withinQ(state.linearVelocity.X, fixed.Q32Zero(), tolerance) {
		t.Errorf("vB.x is %v, want 0", state.linearVelocity.X)
	}

	// The force report divides the axial impulse by the sub-step: -240 on x.
	w.invH = context.invH
	force := getPrismaticJointForce(w, js)
	if !withinQ(force.X, fixed.Q32FromInt(-240), tolerance.Mul(context.invH)) {
		t.Errorf("the joint force is %v, want (-240, 0)", force)
	}
}

// TestPrismaticMotorSaturatesAtTheForce pins the motor branch: the motor
// asks for one unit per second with axial mass one, and the force limit
// clamps the impulse at h * maxMotorForce = 100/240 = 5/12.
func TestPrismaticMotorSaturatesAtTheForce(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	def := DefaultPrismaticJointDef()
	def.EnableMotor = true
	def.MotorSpeed = fixed.Q32One()
	def.MaxMotorForce = fixed.Q32FromInt(100)
	box, j := slidingBox(t, worldId, v2(0, 0), &def)
	state := getBodyState(w, box)

	context := jointContext(w)
	prepareJoints(context, j.colorIndex)
	solveJoints(context, j.colorIndex, false)

	tolerance := fixed.Q32FromRaw(1 << 12)
	js := getJointSim(w, j)
	twelfths := fixed.Q32FromRatio(5, 12)
	if !withinQ(js.prismaticJoint.motorImpulse, twelfths, tolerance) {
		t.Errorf("motorImpulse is %v, want 5/12", js.prismaticJoint.motorImpulse)
	}
	if !withinQ(state.linearVelocity.X, twelfths, tolerance) {
		t.Errorf("vB.x is %v, want 5/12", state.linearVelocity.X)
	}

	// The warm start applies the stored impulse again on a fresh state.
	state.linearVelocity = Vec2Zero()
	warmStartJoints(context, j.colorIndex)
	if !withinQ(state.linearVelocity.X, twelfths, tolerance) {
		t.Errorf("the warm start gives vB.x %v, want 5/12", state.linearVelocity.X)
	}
}

// The float64 mirror follows b2SolvePrismaticJoint line by line, in
// radians.

type f64PrismaticJoint struct {
	mA, mB, iA, iB     float64
	constraintSoftness f64Softness

	impulse                                                 f64Vec
	springImpulse, motorImpulse, lowerImpulse, upperImpulse float64
	hertz, dampingRatio, targetTranslation, maxMotorForce   float64
	motorSpeed, lowerTranslation, upperTranslation          float64
	anchorA, anchorB, axisA, deltaCenter                    f64Vec
	deltaAngle, axialMass                                   float64
	springSoftness                                          f64Softness
	enableSpring, enableLimit, enableMotor                  bool
}

func f64SolvePrismaticJoint(joint *f64PrismaticJoint, stateA, stateB *f64State, h, invH float64, useBias bool) {
	mA, mB, iA, iB := joint.mA, joint.mB, joint.iA, joint.iB

	vA, wA := stateA.v, stateA.w
	vB, wB := stateB.v, stateB.w

	rA := f64Rotate(stateA.dqc, stateA.dqs, joint.anchorA)
	rB := f64Rotate(stateB.dqc, stateB.dqs, joint.anchorB)

	d := f64Add(f64Add(f64Sub(stateB.dp, stateA.dp), joint.deltaCenter), f64Sub(rB, rA))
	axisA := f64Rotate(stateA.dqc, stateA.dqs, joint.axisA)
	translation := f64Dot(axisA, d)

	a1 := f64Cross(f64Add(d, rA), axisA)
	a2 := f64Cross(rB, axisA)

	axial := func(impulse float64) {
		P := f64Scale(impulse, axisA)
		vA = f64Sub(vA, f64Scale(mA, P))
		wA -= iA * impulse * a1
		vB = f64Add(vB, f64Scale(mB, P))
		wB += iB * impulse * a2
	}
	axialCdot := func() float64 { return f64Dot(axisA, f64Sub(vB, vA)) + a2*wB - a1*wA }

	if joint.enableSpring {
		C := translation - joint.targetTranslation
		bias := joint.springSoftness.biasRate * C
		massScale := joint.springSoftness.massScale
		impulseScale := joint.springSoftness.impulseScale

		Cdot := axialCdot()
		deltaImpulse := -massScale*joint.axialMass*(Cdot+bias) - impulseScale*joint.springImpulse
		joint.springImpulse += deltaImpulse
		axial(deltaImpulse)
	}

	if joint.enableMotor {
		Cdot := axialCdot()
		impulse := joint.axialMass * (joint.motorSpeed - Cdot)
		oldImpulse := joint.motorImpulse
		maxImpulse := h * joint.maxMotorForce
		joint.motorImpulse = math.Max(-maxImpulse, math.Min(joint.motorImpulse+impulse, maxImpulse))
		impulse = joint.motorImpulse - oldImpulse
		axial(impulse)
	}

	if joint.enableLimit {
		{
			C := translation - joint.lowerTranslation
			bias, massScale, impulseScale := 0.0, 1.0, 0.0
			if C > 0 {
				bias = C * invH
			} else if useBias {
				bias = joint.constraintSoftness.biasRate * C
				massScale = joint.constraintSoftness.massScale
				impulseScale = joint.constraintSoftness.impulseScale
			}
			oldImpulse := joint.lowerImpulse
			Cdot := axialCdot()
			impulse := -joint.axialMass*massScale*(Cdot+bias) - impulseScale*oldImpulse
			joint.lowerImpulse = math.Max(oldImpulse+impulse, 0)
			impulse = joint.lowerImpulse - oldImpulse
			axial(impulse)
		}
		{
			C := joint.upperTranslation - translation
			bias, massScale, impulseScale := 0.0, 1.0, 0.0
			if C > 0 {
				bias = C * invH
			} else if useBias {
				bias = joint.constraintSoftness.biasRate * C
				massScale = joint.constraintSoftness.massScale
				impulseScale = joint.constraintSoftness.impulseScale
			}
			oldImpulse := joint.upperImpulse
			Cdot := -axialCdot()
			impulse := -joint.axialMass*massScale*(Cdot+bias) - impulseScale*oldImpulse
			joint.upperImpulse = math.Max(oldImpulse+impulse, 0)
			impulse = joint.upperImpulse - oldImpulse
			axial(-impulse)
		}
	}

	{
		perpA := f64Vec{-axisA.y, axisA.x}
		s1 := f64Cross(f64Add(d, rA), perpA)
		s2 := f64Cross(rB, perpA)

		Cdot := f64Vec{f64Dot(perpA, f64Sub(vB, vA)) + s2*wB - s1*wA, wB - wA}

		bias := f64Vec{}
		massScale, impulseScale := 1.0, 0.0
		if useBias {
			C := f64Vec{f64Dot(perpA, d), f64RelativeAngle(stateB.dqc, stateB.dqs, stateA.dqc, stateA.dqs) + joint.deltaAngle}
			bias = f64Scale(joint.constraintSoftness.biasRate, C)
			massScale = joint.constraintSoftness.massScale
			impulseScale = joint.constraintSoftness.impulseScale
		}

		k11 := mA + mB + iA*s1*s1 + iB*s2*s2
		k12 := iA*s1 + iB*s2
		k22 := iA + iB
		if k22 == 0 {
			k22 = 1
		}
		det := k11*k22 - k12*k12
		if det != 0 {
			det = 1 / det
		}
		rhs := f64Add(Cdot, bias)
		b := f64Vec{det * (k22*rhs.x - k12*rhs.y), det * (k11*rhs.y - k12*rhs.x)}

		impulse := f64Vec{-massScale*b.x - impulseScale*joint.impulse.x, -massScale*b.y - impulseScale*joint.impulse.y}
		joint.impulse = f64Add(joint.impulse, impulse)

		P := f64Scale(impulse.x, perpA)
		LA := impulse.x*s1 + impulse.y
		LB := impulse.x*s2 + impulse.y

		vA = f64Sub(vA, f64Scale(mA, P))
		wA -= iA * LA
		vB = f64Add(vB, f64Scale(mB, P))
		wB += iB * LB
	}

	stateA.v, stateA.w = vA, wA
	stateB.v, stateB.w = vB, wB
}

// TestSolvePrismaticJointTracksTheFloat64Mirror runs one biased solve on a
// joint with the spring, the motor and both limits armed, on two rotated
// and displaced states. The angular error goes through the fixed atan2,
// exact to 2^-20 turn, so the bound is 1e-5 as for the revolute joint.
func TestSolvePrismaticJointTracksTheFloat64Mirror(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	idA, idB := mirrorBodies(t, worldId)

	def := DefaultPrismaticJointDef()
	def.BodyIdA, def.BodyIdB = idA, idB
	def.LocalAnchorA = qv("0.5", "0.25")
	def.LocalAnchorB = qv("-1", "0.25")
	def.LocalAxisA = qv("1", "0.5")
	def.ReferenceAngle = fixed.Q32MustParse("0.02")
	def.EnableSpring = true
	def.Hertz = fixed.Q32FromInt(2)
	def.DampingRatio = fixed.Q32Half()
	def.TargetTranslation = fixed.Q32MustParse("0.3")
	def.EnableLimit = true
	def.LowerTranslation = fixed.Q32MustParse("0.6")
	def.UpperTranslation = fixed.Q32MustParse("0.8")
	def.EnableMotor = true
	def.MotorSpeed = fixed.Q32Half()
	def.MaxMotorForce = fixed.Q32FromInt(30)
	jointId := CreatePrismaticJoint(worldId, &def)
	js := getJointSim(w, getJointFullId(w, jointId))
	stateA := getBodyState(w, getBodyFullId(w, idA))
	stateB := getBodyState(w, getBodyFullId(w, idB))

	context := jointContext(w)
	prepareJoint(js, context)

	h := qToF64(context.h)
	invH := qToF64(context.invH)
	p := &js.prismaticJoint
	mirror := &f64PrismaticJoint{
		mA: qToF64(js.invMassA), mB: qToF64(js.invMassB), iA: qToF64(js.invIA), iB: qToF64(js.invIB),
		constraintSoftness: makeSoftF64(math.Min(qToF64(js.constraintHertz), 0.25*invH), qToF64(js.constraintDampingRatio), h),
		hertz:              qToF64(p.hertz),
		dampingRatio:       qToF64(p.dampingRatio),
		targetTranslation:  qToF64(p.targetTranslation),
		maxMotorForce:      qToF64(p.maxMotorForce),
		motorSpeed:         qToF64(p.motorSpeed),
		lowerTranslation:   qToF64(p.lowerTranslation),
		upperTranslation:   qToF64(p.upperTranslation),
		anchorA:            vecToF64(p.anchorA),
		anchorB:            vecToF64(p.anchorB),
		axisA:              vecToF64(p.axisA),
		deltaCenter:        vecToF64(p.deltaCenter),
		deltaAngle:         qToF64(p.deltaAngle) * 2 * math.Pi,
		axialMass:          qToF64(p.axialMass),
		enableSpring:       true,
		enableLimit:        true,
		enableMotor:        true,
	}
	mirror.springSoftness = makeSoftF64(mirror.hertz, mirror.dampingRatio, h)
	fA := stateToF64(stateA)
	fB := stateToF64(stateB)

	solvePrismaticJoint(js, context, true)
	f64SolvePrismaticJoint(mirror, &fA, &fB, h, invH, true)

	const limit = 1e-5
	checkMirror(t, "vA.x", stateA.linearVelocity.X, fA.v.x, limit)
	checkMirror(t, "vA.y", stateA.linearVelocity.Y, fA.v.y, limit)
	checkMirror(t, "wA", stateA.angularVelocity.Mul(tau), fA.w, limit)
	checkMirror(t, "vB.x", stateB.linearVelocity.X, fB.v.x, limit)
	checkMirror(t, "vB.y", stateB.linearVelocity.Y, fB.v.y, limit)
	checkMirror(t, "wB", stateB.angularVelocity.Mul(tau), fB.w, limit)
	checkMirror(t, "springImpulse", p.springImpulse, mirror.springImpulse, limit)
	checkMirror(t, "motorImpulse", p.motorImpulse, mirror.motorImpulse, limit)
	checkMirror(t, "lowerImpulse", p.lowerImpulse, mirror.lowerImpulse, limit)
	checkMirror(t, "upperImpulse", p.upperImpulse, mirror.upperImpulse, limit)
	checkMirror(t, "impulse.x", p.impulse.X, mirror.impulse.x, limit)
	checkMirror(t, "impulse.y", p.impulse.Y, mirror.impulse.y, limit)
}
