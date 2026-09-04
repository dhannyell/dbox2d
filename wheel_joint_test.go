package dbox2d

import (
	"math"
	"testing"

	"github.com/dhannyell/fixed"
)

// This file tests the wheel joint solver with hand-computed cases and with
// a float64 mirror of src/wheel_joint.c. The composite suspension lives in
// step_test.go.

// suspendedBox creates a static ground at the origin and a unit box of
// mass one (inverse inertia 6) at the position, on a wheel joint along the
// default vertical axis between the two body origins. The effective axial
// and perpendicular masses are one and the motor mass is 1/6.
func suspendedBox(t *testing.T, worldId WorldId, position Vec2, def *WheelJointDef) (*body, *joint) {
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
	jointId := CreateWheelJoint(worldId, def)
	return getBodyFullId(w, boxId), getJointFullId(w, jointId)
}

// TestWheelLineHoldsTheBox pins the point-to-line constraint: the box at
// the origin moves across the vertical axis at one unit per second. With
// perpMass = 1, Cdot = -1 gives the impulse 1 on the perpendicular (-1, 0)
// and the box stops. The spring is off, so the free axis keeps its velocity.
func TestWheelLineHoldsTheBox(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	def := DefaultWheelJointDef()
	def.EnableSpring = false
	box, j := suspendedBox(t, worldId, v2(0, 0), &def)
	state := getBodyState(w, box)
	state.linearVelocity = Vec2{X: fixed.Q32One(), Y: fixed.Q32One()}

	context := jointContext(w)
	prepareJoints(context, j.colorIndex)
	solveJoints(context, j.colorIndex, false)

	tolerance := fixed.Q32FromRaw(1 << 12)
	js := getJointSim(w, j)
	if !withinQ(js.wheelJoint.perpImpulse, fixed.Q32One(), tolerance) {
		t.Errorf("perpImpulse is %v, want 1", js.wheelJoint.perpImpulse)
	}
	if !withinQ(state.linearVelocity.X, fixed.Q32Zero(), tolerance) || !withinQ(state.linearVelocity.Y, fixed.Q32One(), tolerance) {
		t.Errorf("vB is %v, want (0, 1)", state.linearVelocity)
	}

	// The force report divides the impulse by the sub-step: 240 on the
	// perpendicular (-1, 0).
	w.invH = context.invH
	force := getWheelJointForce(w, js)
	if !withinQ(force.X, fixed.Q32FromInt(-240), tolerance.Mul(context.invH)) {
		t.Errorf("the joint force is %v, want (-240, 0)", force)
	}
}

// TestWheelUpperLimitStopsTheTravel pins the limit branch: the box sits at
// the upper translation one and moves up at one unit per second.
//
//	lower: C = 1 > 0, bias = 240, impulse clamped to 0
//	upper: C = 0, Cdot = -1, impulse = 1, vB.y -= 1 = 0
func TestWheelUpperLimitStopsTheTravel(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	def := DefaultWheelJointDef()
	def.EnableSpring = false
	def.EnableLimit = true
	def.LowerTranslation = fixed.Q32Zero()
	def.UpperTranslation = fixed.Q32One()
	box, j := suspendedBox(t, worldId, v2(0, 1), &def)
	state := getBodyState(w, box)
	state.linearVelocity = Vec2{Y: fixed.Q32One()}

	context := jointContext(w)
	prepareJoints(context, j.colorIndex)
	solveJoints(context, j.colorIndex, false)

	tolerance := fixed.Q32FromRaw(1 << 12)
	js := getJointSim(w, j)
	if !js.wheelJoint.lowerImpulse.Eq(fixed.Q32Zero()) || !withinQ(js.wheelJoint.upperImpulse, fixed.Q32One(), tolerance) {
		t.Errorf("the limit impulses are %v and %v, want 0 and 1", js.wheelJoint.lowerImpulse, js.wheelJoint.upperImpulse)
	}
	if !withinQ(state.linearVelocity.Y, fixed.Q32Zero(), tolerance) {
		t.Errorf("vB.y is %v, want 0", state.linearVelocity.Y)
	}
}

// TestWheelMotorSaturatesAtTheTorque pins the motor branch and the unit
// of the motor speed: one turn per second is tau radians per second, the
// motor mass 1/6 asks for tau/6, and the torque limit clamps the impulse
// at h * maxMotorTorque = 100/240 = 5/12, so wB = 6 * 5/12 = 2.5 rad/s.
func TestWheelMotorSaturatesAtTheTorque(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	def := DefaultWheelJointDef()
	def.EnableSpring = false
	def.EnableMotor = true
	def.MotorSpeed = fixed.Q32One()
	def.MaxMotorTorque = fixed.Q32FromInt(100)
	box, j := suspendedBox(t, worldId, v2(0, 0), &def)
	state := getBodyState(w, box)

	context := jointContext(w)
	prepareJoints(context, j.colorIndex)
	solveJoints(context, j.colorIndex, false)

	tolerance := fixed.Q32FromRaw(1 << 12)
	js := getJointSim(w, j)
	if !withinQ(js.wheelJoint.motorImpulse, fixed.Q32FromRatio(5, 12), tolerance) {
		t.Errorf("motorImpulse is %v, want 5/12", js.wheelJoint.motorImpulse)
	}
	wB := state.angularVelocity.Mul(tau)
	if !withinQ(wB, fixed.Q32FromRatio(5, 2), tolerance) {
		t.Errorf("wB is %v rad/s, want 2.5", wB)
	}

	// The torque report is the saturated motor torque.
	w.invH = context.invH
	if torque := getWheelJointTorque(w, js); !withinQ(torque, fixed.Q32FromInt(100), tolerance.Mul(context.invH)) {
		t.Errorf("the joint torque is %v, want 100", torque)
	}

	// The warm start applies the stored impulse again on a fresh state.
	state.angularVelocity = fixed.Q32Zero()
	warmStartJoints(context, j.colorIndex)
	wB = state.angularVelocity.Mul(tau)
	if !withinQ(wB, fixed.Q32FromRatio(5, 2), tolerance) {
		t.Errorf("the warm start gives wB %v rad/s, want 2.5", wB)
	}
}

// The float64 mirror follows b2SolveWheelJoint line by line, in radians.

type f64WheelJoint struct {
	mA, mB, iA, iB     float64
	constraintSoftness f64Softness

	perpImpulse, motorImpulse, springImpulse float64
	lowerImpulse, upperImpulse               float64
	maxMotorTorque, motorSpeed               float64
	lowerTranslation, upperTranslation       float64
	hertz, dampingRatio                      float64
	anchorA, anchorB, axisA, deltaCenter     f64Vec
	perpMass, motorMass, axialMass           float64
	springSoftness                           f64Softness
	enableSpring, enableMotor, enableLimit   bool
}

func f64SolveWheelJoint(joint *f64WheelJoint, stateA, stateB *f64State, h, invH float64, useBias bool) {
	mA, mB, iA, iB := joint.mA, joint.mB, joint.iA, joint.iB

	vA, wA := stateA.v, stateA.w
	vB, wB := stateB.v, stateB.w

	fixedRotation := iA+iB == 0

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

	if joint.enableMotor && !fixedRotation {
		Cdot := wB - wA - joint.motorSpeed
		impulse := -joint.motorMass * Cdot
		oldImpulse := joint.motorImpulse
		maxImpulse := h * joint.maxMotorTorque
		joint.motorImpulse = math.Max(-maxImpulse, math.Min(joint.motorImpulse+impulse, maxImpulse))
		impulse = joint.motorImpulse - oldImpulse

		wA -= iA * impulse
		wB += iB * impulse
	}

	if joint.enableSpring {
		C := translation
		bias := joint.springSoftness.biasRate * C
		massScale := joint.springSoftness.massScale
		impulseScale := joint.springSoftness.impulseScale

		Cdot := axialCdot()
		impulse := -massScale*joint.axialMass*(Cdot+bias) - impulseScale*joint.springImpulse
		joint.springImpulse += impulse
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
			Cdot := axialCdot()
			impulse := -massScale*joint.axialMass*(Cdot+bias) - impulseScale*joint.lowerImpulse
			oldImpulse := joint.lowerImpulse
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
			Cdot := -axialCdot()
			impulse := -massScale*joint.axialMass*(Cdot+bias) - impulseScale*joint.upperImpulse
			oldImpulse := joint.upperImpulse
			joint.upperImpulse = math.Max(oldImpulse+impulse, 0)
			impulse = joint.upperImpulse - oldImpulse
			axial(-impulse)
		}
	}

	{
		perpA := f64Vec{-axisA.y, axisA.x}

		bias, massScale, impulseScale := 0.0, 1.0, 0.0
		if useBias {
			C := f64Dot(perpA, d)
			bias = joint.constraintSoftness.biasRate * C
			massScale = joint.constraintSoftness.massScale
			impulseScale = joint.constraintSoftness.impulseScale
		}

		s1 := f64Cross(f64Add(d, rA), perpA)
		s2 := f64Cross(rB, perpA)
		Cdot := f64Dot(perpA, f64Sub(vB, vA)) + s2*wB - s1*wA

		impulse := -massScale*joint.perpMass*(Cdot+bias) - impulseScale*joint.perpImpulse
		joint.perpImpulse += impulse

		P := f64Scale(impulse, perpA)
		vA = f64Sub(vA, f64Scale(mA, P))
		wA -= iA * impulse * s1
		vB = f64Add(vB, f64Scale(mB, P))
		wB += iB * impulse * s2
	}

	stateA.v, stateA.w = vA, wA
	stateB.v, stateB.w = vB, wB
}

// TestSolveWheelJointTracksTheFloat64Mirror runs one biased solve on a
// joint with the spring, the motor and both limits armed, on two rotated
// and displaced states. No angle enters the error, so the bound is 1e-6.
func TestSolveWheelJointTracksTheFloat64Mirror(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	idA, idB := mirrorBodies(t, worldId)

	def := DefaultWheelJointDef()
	def.BodyIdA, def.BodyIdB = idA, idB
	def.LocalAnchorA = qv("0.5", "0.25")
	def.LocalAnchorB = qv("-1", "0.25")
	def.LocalAxisA = qv("1", "0.5")
	def.EnableSpring = true
	def.Hertz = fixed.Q32FromInt(2)
	def.DampingRatio = fixed.Q32Half()
	def.EnableLimit = true
	def.LowerTranslation = fixed.Q32MustParse("0.6")
	def.UpperTranslation = fixed.Q32MustParse("0.8")
	def.EnableMotor = true
	def.MotorSpeed = fixed.Q32Half()
	def.MaxMotorTorque = fixed.Q32FromInt(3)
	jointId := CreateWheelJoint(worldId, &def)
	js := getJointSim(w, getJointFullId(w, jointId))
	stateA := getBodyState(w, getBodyFullId(w, idA))
	stateB := getBodyState(w, getBodyFullId(w, idB))

	context := jointContext(w)
	prepareJoint(js, context)

	h := qToF64(context.h)
	invH := qToF64(context.invH)
	wj := &js.wheelJoint
	mirror := &f64WheelJoint{
		mA: qToF64(js.invMassA), mB: qToF64(js.invMassB), iA: qToF64(js.invIA), iB: qToF64(js.invIB),
		constraintSoftness: makeSoftF64(math.Min(qToF64(js.constraintHertz), 0.25*invH), qToF64(js.constraintDampingRatio), h),
		maxMotorTorque:     qToF64(wj.maxMotorTorque),
		motorSpeed:         qToF64(wj.motorSpeed) * 2 * math.Pi,
		lowerTranslation:   qToF64(wj.lowerTranslation),
		upperTranslation:   qToF64(wj.upperTranslation),
		hertz:              qToF64(wj.hertz),
		dampingRatio:       qToF64(wj.dampingRatio),
		anchorA:            vecToF64(wj.anchorA),
		anchorB:            vecToF64(wj.anchorB),
		axisA:              vecToF64(wj.axisA),
		deltaCenter:        vecToF64(wj.deltaCenter),
		perpMass:           qToF64(wj.perpMass),
		motorMass:          qToF64(wj.motorMass),
		axialMass:          qToF64(wj.axialMass),
		enableSpring:       true,
		enableMotor:        true,
		enableLimit:        true,
	}
	mirror.springSoftness = makeSoftF64(mirror.hertz, mirror.dampingRatio, h)
	fA := stateToF64(stateA)
	fB := stateToF64(stateB)

	solveWheelJoint(js, context, true)
	f64SolveWheelJoint(mirror, &fA, &fB, h, invH, true)

	const limit = 1e-6
	checkMirror(t, "vA.x", stateA.linearVelocity.X, fA.v.x, limit)
	checkMirror(t, "vA.y", stateA.linearVelocity.Y, fA.v.y, limit)
	checkMirror(t, "wA", stateA.angularVelocity.Mul(tau), fA.w, limit)
	checkMirror(t, "vB.x", stateB.linearVelocity.X, fB.v.x, limit)
	checkMirror(t, "vB.y", stateB.linearVelocity.Y, fB.v.y, limit)
	checkMirror(t, "wB", stateB.angularVelocity.Mul(tau), fB.w, limit)
	checkMirror(t, "perpImpulse", wj.perpImpulse, mirror.perpImpulse, limit)
	checkMirror(t, "motorImpulse", wj.motorImpulse, mirror.motorImpulse, limit)
	checkMirror(t, "springImpulse", wj.springImpulse, mirror.springImpulse, limit)
	checkMirror(t, "lowerImpulse", wj.lowerImpulse, mirror.lowerImpulse, limit)
	checkMirror(t, "upperImpulse", wj.upperImpulse, mirror.upperImpulse, limit)
}
