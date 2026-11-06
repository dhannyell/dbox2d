package b2

import (
	"math"
	"testing"
)

// This file tests the motor joint solver with hand-computed cases and with
// a float64 mirror of src/motor_joint.c. The composite drive lives in
// step_test.go.

// drivenBox creates a static ground at the origin and a unit box of mass
// one (inverse inertia 6) at the origin, on a motor joint between the two
// body origins. The linear mass is the identity and the angular mass 1/6.
func drivenBox(t *testing.T, worldId WorldId, def *MotorJointDef) (*body, *joint) {
	t.Helper()
	w := getWorldFromId(worldId)

	groundDef := DefaultBodyDef()
	groundId := CreateBody(worldId, &groundDef)

	boxDef := DefaultBodyDef()
	boxDef.Type = DynamicBody
	boxId := CreateBody(worldId, &boxDef)
	shapeDef := DefaultShapeDef()
	unit := MakeBox(QHalf(), QHalf())
	CreatePolygonShape(boxId, &shapeDef, &unit)

	def.Base.BodyIdA = groundId
	def.Base.BodyIdB = boxId
	jointId := CreateMotorJoint(worldId, def)
	return getBodyFullId(w, boxId), getJointFullId(w, jointId)
}

// TestMotorJointAccessorsRoundTrip pins the motor joint settings through
// their public setters and getters without involving the solver.
func TestMotorJointAccessorsRoundTrip(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	def := DefaultMotorJointDef()
	_, j := drivenBox(t, worldId, &def)
	jointId := makeJointId(w, jointPair{joint: j, jointSim: getJointSim(w, j)})

	linearVelocity := Vec2{X: QFromRatio(3, 2), Y: QFromRatio(-2, 3)}
	tests := []struct {
		name string
		set  func()
		get  func() any
		want any
	}{
		{"linear velocity", func() { jointId.SetLinearVelocity(linearVelocity) }, func() any { return jointId.GetLinearVelocity() }, linearVelocity},
		{"angular velocity", func() { jointId.SetAngularVelocity(QFromRatio(1, 4)) }, func() any { return jointId.GetAngularVelocity() }, QFromRatio(1, 4)},
		{"max velocity force", func() { jointId.SetMaxVelocityForce(QFromInt(7)) }, func() any { return jointId.GetMaxVelocityForce() }, QFromInt(7)},
		{"max velocity torque", func() { jointId.SetMaxVelocityTorque(QFromInt(11)) }, func() any { return jointId.GetMaxVelocityTorque() }, QFromInt(11)},
		{"linear hertz", func() { jointId.SetLinearHertz(QFromInt(3)) }, func() any { return jointId.GetLinearHertz() }, QFromInt(3)},
		{"linear damping ratio", func() { jointId.SetLinearDampingRatio(QFromRatio(2, 5)) }, func() any { return jointId.GetLinearDampingRatio() }, QFromRatio(2, 5)},
		{"max spring force", func() { jointId.SetMaxSpringForce(QFromInt(13)) }, func() any { return jointId.GetMaxSpringForce() }, QFromInt(13)},
		{"angular hertz", func() { jointId.SetAngularHertz(QFromInt(5)) }, func() any { return jointId.GetAngularHertz() }, QFromInt(5)},
		{"angular damping ratio", func() { jointId.SetAngularDampingRatio(QFromRatio(3, 5)) }, func() any { return jointId.GetAngularDampingRatio() }, QFromRatio(3, 5)},
		{"max spring torque", func() { jointId.SetMaxSpringTorque(QFromInt(17)) }, func() any { return jointId.GetMaxSpringTorque() }, QFromInt(17)},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			tt.set()
			switch want := tt.want.(type) {
			case Vec2:
				got, ok := tt.get().(Vec2)
				if !ok || !got.X.Eq(want.X) || !got.Y.Eq(want.Y) {
					t.Errorf("got %v, want %v", got, want)
				}
			case Q:
				// The angular velocity goes through radians in float mode.
				got, ok := tt.get().(Q)
				if !ok || !withinQ(got, want, qUlps(1<<12)) {
					t.Errorf("got %v, want %v", got, want)
				}
			default:
				t.Fatalf("unsupported accessor type %T", want)
			}
		})
	}
}

// TestMotorDrivesAtTheLinearVelocity pins the linear velocity row and the
// force clamp: the target (1, 0) asks for the impulse (1, 0), and the clamp
// h * maxVelocityForce = 100/240 = 5/12 keeps (5/12, 0).
func TestMotorDrivesAtTheLinearVelocity(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	def := DefaultMotorJointDef()
	def.LinearVelocity = Vec2{X: QOne()}
	def.MaxVelocityForce = QFromInt(100)
	box, j := drivenBox(t, worldId, &def)
	state := getBodyState(w, box)

	context := jointContext(w)
	prepareJointsTask(0, len(context.joints), context)
	solveJointsTask(0, len(context.graph.colors[j.colorIndex].jointSims), context, j.colorIndex, false, 0)

	tolerance := qUlps(1 << 12)
	js := getJointSim(w, j)
	twelfths := QFromRatio(5, 12)
	impulse := js.motor().linearVelocityImpulse
	if !withinQ(impulse.X, twelfths, tolerance) || !withinQ(impulse.Y, QZero(), tolerance) {
		t.Errorf("linearVelocityImpulse is %v, want (5/12, 0)", impulse)
	}
	if !withinQ(state.linearVelocity.X, twelfths, tolerance) {
		t.Errorf("vB.x is %v, want 5/12", state.linearVelocity.X)
	}

	// The force report is the saturated motor force.
	w.invH = context.invH
	force := getMotorJointForce(w, js)
	if !withinQ(force.X, QFromInt(100), tolerance.Mul(context.invH)) {
		t.Errorf("the joint force is %v, want (100, 0)", force)
	}

	// The warm start applies the stored impulse again on a fresh state.
	state.linearVelocity = Vec2Zero()
	warmStartJointsTask(0, len(context.graph.colors[j.colorIndex].jointSims), context, j.colorIndex)
	if !withinQ(state.linearVelocity.X, twelfths, tolerance) {
		t.Errorf("the warm start gives vB.x %v, want 5/12", state.linearVelocity.X)
	}
}

// TestMotorTurnsAtTheAngularVelocity pins the angular velocity row and its
// unit: a quarter turn per second is tau/4 rad/s, below the torque clamp,
// so the box reaches it in one solve with the impulse tau/24.
func TestMotorTurnsAtTheAngularVelocity(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	def := DefaultMotorJointDef()
	def.AngularVelocity = QMustParse("0.25")
	def.MaxVelocityTorque = QFromInt(100)
	box, j := drivenBox(t, worldId, &def)
	state := getBodyState(w, box)

	context := jointContext(w)
	prepareJointsTask(0, len(context.joints), context)
	solveJointsTask(0, len(context.graph.colors[j.colorIndex].jointSims), context, j.colorIndex, false, 0)

	js := getJointSim(w, j)
	const limit = 1e-4
	checkMirror(t, "angularVelocityImpulse", js.motor().angularVelocityImpulse, math.Pi/12, limit)
	checkMirror(t, "wB", state.angularVelocity, math.Pi/2, limit)

	// The torque report is the velocity impulse over the step.
	w.invH = context.invH
	checkMirror(t, "torque", getMotorJointTorque(w, js), math.Pi/12*qToF64(context.invH), 1e-2)
}

// The float64 mirror follows b2SolveMotorJoint of v3.2 line by line, in
// radians.

type f64MotorJoint struct {
	mA, mB, iA, iB                      float64
	linearVelocity                      f64Vec
	angularVelocity                     float64
	maxVelocityForce, maxVelocityTorque float64
	maxSpringForce, maxSpringTorque     float64
	linearHertz, angularHertz           float64
	linearSpring, angularSpring         f64Softness

	linearVelocityImpulse, linearSpringImpulse   f64Vec
	angularVelocityImpulse, angularSpringImpulse float64

	frameA, frameB f64Frame
	deltaCenter    f64Vec
	angularMass    float64
}

// f64MotorSolve applies the inverse of the point constraint mass to rhs.
func f64MotorSolve(mA, mB, iA, iB float64, rA, rB, rhs f64Vec) f64Vec {
	k11 := mA + mB + rA.y*rA.y*iA + rB.y*rB.y*iB
	k12 := -rA.y*rA.x*iA - rB.y*rB.x*iB
	k22 := mA + mB + rA.x*rA.x*iA + rB.x*rB.x*iB
	det := k11*k22 - k12*k12
	if det != 0 {
		det = 1 / det
	}
	return f64Vec{det * (k22*rhs.x - k12*rhs.y), det * (k11*rhs.y - k12*rhs.x)}
}

// f64ClampLength scales v down to maxLength.
func f64ClampLength(v f64Vec, maxLength float64) f64Vec {
	if lenSq := f64Dot(v, v); lenSq > maxLength*maxLength {
		return f64Scale(maxLength/math.Sqrt(lenSq), v)
	}
	return v
}

func f64SolveMotorJoint(joint *f64MotorJoint, stateA, stateB *f64State, h float64) {
	mA, mB, iA, iB := joint.mA, joint.mB, joint.iA, joint.iB

	vA, wA := stateA.v, stateA.w
	vB, wB := stateB.v, stateB.w

	if joint.maxSpringTorque > 0 && joint.angularHertz > 0 {
		C := f64FrameAngle(stateA, stateB, joint.frameA, joint.frameB)
		bias := joint.angularSpring.biasRate * C
		Cdot := wB - wA
		maxImpulse := h * joint.maxSpringTorque
		oldImpulse := joint.angularSpringImpulse
		impulse := -joint.angularSpring.massScale*joint.angularMass*(Cdot+bias) - joint.angularSpring.impulseScale*oldImpulse
		joint.angularSpringImpulse = math.Max(-maxImpulse, math.Min(oldImpulse+impulse, maxImpulse))
		impulse = joint.angularSpringImpulse - oldImpulse
		wA -= iA * impulse
		wB += iB * impulse
	}

	if joint.maxVelocityTorque > 0 {
		Cdot := wB - wA - joint.angularVelocity
		impulse := -joint.angularMass * Cdot
		maxImpulse := h * joint.maxVelocityTorque
		oldImpulse := joint.angularVelocityImpulse
		joint.angularVelocityImpulse = math.Max(-maxImpulse, math.Min(oldImpulse+impulse, maxImpulse))
		impulse = joint.angularVelocityImpulse - oldImpulse
		wA -= iA * impulse
		wB += iB * impulse
	}

	rA := f64Rotate(stateA.dqc, stateA.dqs, joint.frameA.p)
	rB := f64Rotate(stateB.dqc, stateB.dqs, joint.frameB.p)

	apply := func(impulse f64Vec) {
		vA = f64Sub(vA, f64Scale(mA, impulse))
		wA -= iA * f64Cross(rA, impulse)
		vB = f64Add(vB, f64Scale(mB, impulse))
		wB += iB * f64Cross(rB, impulse)
	}

	if joint.maxSpringForce > 0 && joint.linearHertz > 0 {
		C := f64Add(f64Add(f64Sub(stateB.dp, stateA.dp), f64Sub(rB, rA)), joint.deltaCenter)
		bias := f64Scale(joint.linearSpring.biasRate, C)
		Cdot := f64Sub(f64Add(vB, f64CrossSV(wB, rB)), f64Add(vA, f64CrossSV(wA, rA)))
		b := f64MotorSolve(mA, mB, iA, iB, rA, rB, f64Add(Cdot, bias))
		oldImpulse := joint.linearSpringImpulse
		ms, is := joint.linearSpring.massScale, joint.linearSpring.impulseScale
		impulse := f64Vec{-ms*b.x - is*oldImpulse.x, -ms*b.y - is*oldImpulse.y}
		joint.linearSpringImpulse = f64ClampLength(f64Add(oldImpulse, impulse), h*joint.maxSpringForce)
		apply(f64Sub(joint.linearSpringImpulse, oldImpulse))
	}

	if joint.maxVelocityForce > 0 {
		Cdot := f64Sub(f64Sub(f64Add(vB, f64CrossSV(wB, rB)), f64Add(vA, f64CrossSV(wA, rA))), joint.linearVelocity)
		b := f64MotorSolve(mA, mB, iA, iB, rA, rB, Cdot)
		oldImpulse := joint.linearVelocityImpulse
		joint.linearVelocityImpulse = f64ClampLength(f64Sub(oldImpulse, b), h*joint.maxVelocityForce)
		apply(f64Sub(joint.linearVelocityImpulse, oldImpulse))
	}

	stateA.v, stateA.w = vA, wA
	stateB.v, stateB.w = vB, wB
}

// TestSolveMotorJointTracksTheFloat64Mirror runs one solve with all four
// rows armed and below their clamps, on two rotated and displaced states
// with offset frames. The angular spring goes through the fixed atan2,
// exact to 2^-20 turn, so the bound is 1e-4.
func TestSolveMotorJointTracksTheFloat64Mirror(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	idA, idB := mirrorBodies(t, worldId)

	def := DefaultMotorJointDef()
	def.Base.BodyIdA, def.Base.BodyIdB = idA, idB
	def.Base.LocalFrameA = Transform{P: qv("0.4", "-0.2"), Q: MakeRot(QMustParse("0.05"))}
	def.Base.LocalFrameB = Transform{P: qv("-0.3", "0.25"), Q: MakeRot(QMustParse("-0.07"))}
	def.LinearVelocity = qv("0.6", "-0.4")
	def.AngularVelocity = QMustParse("0.1")
	def.MaxVelocityForce = QFromInt(1000)
	def.MaxVelocityTorque = QFromInt(1000)
	def.LinearHertz = QFromInt(4)
	def.LinearDampingRatio = QMustParse("0.7")
	def.MaxSpringForce = QFromInt(1000)
	def.AngularHertz = QFromInt(3)
	def.AngularDampingRatio = QMustParse("0.5")
	def.MaxSpringTorque = QFromInt(1000)
	jointId := CreateMotorJoint(worldId, &def)
	js := getJointSim(w, getJointFullId(w, jointId))
	stateA := getBodyState(w, getBodyFullId(w, idA))
	stateB := getBodyState(w, getBodyFullId(w, idB))

	context := jointContext(w)
	prepareJoint(js, context)

	h := qToF64(context.h)
	m := js.motor()
	mirror := &f64MotorJoint{
		mA: qToF64(js.invMassA), mB: qToF64(js.invMassB), iA: qToF64(js.invIA), iB: qToF64(js.invIB),
		linearVelocity:    vecToF64(m.linearVelocity),
		angularVelocity:   radiansF64(m.angularVelocity),
		maxVelocityForce:  qToF64(m.maxVelocityForce),
		maxVelocityTorque: qToF64(m.maxVelocityTorque),
		maxSpringForce:    qToF64(m.maxSpringForce),
		maxSpringTorque:   qToF64(m.maxSpringTorque),
		linearHertz:       qToF64(m.linearHertz),
		angularHertz:      qToF64(m.angularHertz),
		linearSpring:      makeSoftF64(qToF64(m.linearHertz), qToF64(m.linearDampingRatio), h),
		angularSpring:     makeSoftF64(qToF64(m.angularHertz), qToF64(m.angularDampingRatio), h),
		frameA:            frameToF64(m.frameA),
		frameB:            frameToF64(m.frameB),
		deltaCenter:       vecToF64(m.deltaCenter),
	}
	mirror.angularMass = 1 / (mirror.iA + mirror.iB)
	fA := stateToF64(stateA)
	fB := stateToF64(stateB)

	solveMotorJoint(js, context)
	f64SolveMotorJoint(mirror, &fA, &fB, h)

	const limit = 1e-4
	checkMirror(t, "vA.x", stateA.linearVelocity.X, fA.v.x, limit)
	checkMirror(t, "vA.y", stateA.linearVelocity.Y, fA.v.y, limit)
	checkMirror(t, "wA", stateA.angularVelocity, fA.w, limit)
	checkMirror(t, "vB.x", stateB.linearVelocity.X, fB.v.x, limit)
	checkMirror(t, "vB.y", stateB.linearVelocity.Y, fB.v.y, limit)
	checkMirror(t, "wB", stateB.angularVelocity, fB.w, limit)
	checkMirror(t, "angularSpringImpulse", m.angularSpringImpulse, mirror.angularSpringImpulse, limit)
	checkMirror(t, "angularVelocityImpulse", m.angularVelocityImpulse, mirror.angularVelocityImpulse, limit)
	checkMirror(t, "linearSpringImpulse.x", m.linearSpringImpulse.X, mirror.linearSpringImpulse.x, limit)
	checkMirror(t, "linearSpringImpulse.y", m.linearSpringImpulse.Y, mirror.linearSpringImpulse.y, limit)
	checkMirror(t, "linearVelocityImpulse.x", m.linearVelocityImpulse.X, mirror.linearVelocityImpulse.x, limit)
	checkMirror(t, "linearVelocityImpulse.y", m.linearVelocityImpulse.Y, mirror.linearVelocityImpulse.y, limit)
}
