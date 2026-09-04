package dbox2d

import (
	"math"
	"testing"

	"github.com/dhannyell/fixed"
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
	unit := MakeBox(fixed.Q32Half(), fixed.Q32Half())
	CreatePolygonShape(boxId, &shapeDef, &unit)

	def.BodyIdA = groundId
	def.BodyIdB = boxId
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

	linearOffset := Vec2{X: fixed.Q32FromRatio(3, 2), Y: fixed.Q32FromRatio(-2, 3)}
	tests := []struct {
		name string
		set  func()
		get  func() any
		want any
	}{
		{
			name: "linear offset",
			set:  func() { jointId.SetLinearOffset(linearOffset) },
			get:  func() any { return jointId.GetLinearOffset() },
			want: linearOffset,
		},
		{
			name: "angular offset",
			set:  func() { jointId.SetAngularOffset(fixed.Q32FromRatio(1, 4)) },
			get:  func() any { return jointId.GetAngularOffset() },
			want: fixed.Q32FromRatio(1, 4),
		},
		{
			name: "max force",
			set:  func() { jointId.SetMaxForce(fixed.Q32FromInt(7)) },
			get:  func() any { return jointId.GetMaxForce() },
			want: fixed.Q32FromInt(7),
		},
		{
			name: "max torque",
			set:  func() { jointId.SetMaxTorque(fixed.Q32FromInt(11)) },
			get:  func() any { return jointId.GetMaxTorque() },
			want: fixed.Q32FromInt(11),
		},
		{
			name: "correction factor",
			set:  func() { jointId.SetCorrectionFactor(fixed.Q32FromRatio(2, 5)) },
			get:  func() any { return jointId.GetCorrectionFactor() },
			want: fixed.Q32FromRatio(2, 5),
		},
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
				got, ok := tt.get().(Q)
				if !ok || !got.Eq(want) {
					t.Errorf("got %v, want %v", got, want)
				}
			default:
				t.Fatalf("unsupported accessor type %T", want)
			}
		})
	}

	t.Run("mouse max force dispatch", func(t *testing.T) {
		groundDef := DefaultBodyDef()
		groundId := CreateBody(worldId, &groundDef)
		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody
		bodyId := CreateBody(worldId, &bodyDef)
		mouseDef := DefaultMouseJointDef()
		mouseDef.BodyIdA = groundId
		mouseDef.BodyIdB = bodyId
		mouseDef.Target = Vec2Zero()
		mouseId := CreateMouseJoint(worldId, &mouseDef)
		want := fixed.Q32FromInt(13)
		mouseId.SetMaxForce(want)
		if got := mouseId.GetMaxForce(); !got.Eq(want) {
			t.Errorf("got %v, want %v", got, want)
		}
	})
}

// TestMotorJointAngularOffsetClampsToHalfTurn pins the port's half-turn
// bound for angular offsets.
func TestMotorJointAngularOffsetClampsToHalfTurn(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	def := DefaultMotorJointDef()
	_, j := drivenBox(t, worldId, &def)
	jointId := makeJointId(w, jointPair{joint: j, jointSim: getJointSim(w, j)})

	jointId.SetAngularOffset(fixed.Q32FromRatio(3, 4))
	if got := jointId.GetAngularOffset(); !got.Eq(fixed.Q32Half()) {
		t.Errorf("got %v, want %v", got, fixed.Q32Half())
	}
}

// TestMotorDrivesTowardTheOffset pins the linear row and the force clamp:
// the offset (1, 0) gives the separation (-1, 0) and the bias
// 240 * 0.3 * (-1, 0) = (-72, 0), so the impulse asks for (72, 0) and the
// clamp h * maxForce = 100/240 = 5/12 keeps (5/12, 0).
func TestMotorDrivesTowardTheOffset(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	def := DefaultMotorJointDef()
	def.LinearOffset = Vec2{X: fixed.Q32One()}
	def.MaxForce = fixed.Q32FromInt(100)
	box, j := drivenBox(t, worldId, &def)
	state := getBodyState(w, box)

	context := jointContext(w)
	prepareJoints(context, j.colorIndex)
	solveJoints(context, j.colorIndex, false)

	tolerance := fixed.Q32FromRaw(1 << 12)
	js := getJointSim(w, j)
	twelfths := fixed.Q32FromRatio(5, 12)
	impulse := js.motorJoint.linearImpulse
	if !withinQ(impulse.X, twelfths, tolerance) || !withinQ(impulse.Y, fixed.Q32Zero(), tolerance) {
		t.Errorf("linearImpulse is %v, want (5/12, 0)", impulse)
	}
	if !withinQ(state.linearVelocity.X, twelfths, tolerance) {
		t.Errorf("vB.x is %v, want 5/12", state.linearVelocity.X)
	}

	// The force report is the saturated motor force.
	w.invH = context.invH
	force := getMotorJointForce(w, js)
	if !withinQ(force.X, fixed.Q32FromInt(100), tolerance.Mul(context.invH)) {
		t.Errorf("the joint force is %v, want (100, 0)", force)
	}

	// The warm start applies the stored impulse again on a fresh state.
	state.linearVelocity = Vec2Zero()
	warmStartJoints(context, j.colorIndex)
	if !withinQ(state.linearVelocity.X, twelfths, tolerance) {
		t.Errorf("the warm start gives vB.x %v, want 5/12", state.linearVelocity.X)
	}
}

// TestMotorTurnsTowardTheAngularOffset pins the angular row and the unit
// of the offset: a quarter turn is tau/4 radians, so the bias is
// 240 * 0.3 * (-tau/4) and the impulse asks for more than the clamp
// h * maxTorque = 5/12. With inverse inertia 6, wB = 2.5 rad/s.
func TestMotorTurnsTowardTheAngularOffset(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	def := DefaultMotorJointDef()
	def.AngularOffset = fixed.Q32MustParse("0.25")
	def.MaxTorque = fixed.Q32FromInt(100)
	box, j := drivenBox(t, worldId, &def)
	state := getBodyState(w, box)

	context := jointContext(w)
	prepareJoints(context, j.colorIndex)
	solveJoints(context, j.colorIndex, false)

	tolerance := fixed.Q32FromRaw(1 << 12)
	js := getJointSim(w, j)
	if !withinQ(js.motorJoint.angularImpulse, fixed.Q32FromRatio(5, 12), tolerance) {
		t.Errorf("angularImpulse is %v, want 5/12", js.motorJoint.angularImpulse)
	}
	if wB := state.angularVelocity.Mul(tau); !withinQ(wB, fixed.Q32FromRatio(5, 2), tolerance) {
		t.Errorf("wB is %v rad/s, want 2.5", wB)
	}

	w.invH = context.invH
	if torque := getMotorJointTorque(w, js); !withinQ(torque, fixed.Q32FromInt(100), tolerance.Mul(context.invH)) {
		t.Errorf("the joint torque is %v, want 100", torque)
	}
}

// The float64 mirror follows b2SolveMotorJoint line by line, in radians.

type f64MotorJoint struct {
	mA, mB, iA, iB                float64
	linearImpulse                 f64Vec
	angularImpulse                float64
	maxForce, maxTorque           float64
	correctionFactor              float64
	anchorA, anchorB, deltaCenter f64Vec
	deltaAngle                    float64
	k11, k12, k22                 float64
	angularMass                   float64
}

func f64SolveMotorJoint(joint *f64MotorJoint, stateA, stateB *f64State, h, invH float64) {
	mA, mB, iA, iB := joint.mA, joint.mB, joint.iA, joint.iB

	vA, wA := stateA.v, stateA.w
	vB, wB := stateB.v, stateB.w

	{
		angularSeparation := f64RelativeAngle(stateB.dqc, stateB.dqs, stateA.dqc, stateA.dqs) + joint.deltaAngle
		angularSeparation = f64UnwindAngle(angularSeparation)

		angularBias := invH * joint.correctionFactor * angularSeparation

		Cdot := wB - wA
		impulse := -joint.angularMass * (Cdot + angularBias)

		oldImpulse := joint.angularImpulse
		maxImpulse := h * joint.maxTorque
		joint.angularImpulse = math.Max(-maxImpulse, math.Min(joint.angularImpulse+impulse, maxImpulse))
		impulse = joint.angularImpulse - oldImpulse

		wA -= iA * impulse
		wB += iB * impulse
	}

	{
		rA := f64Rotate(stateA.dqc, stateA.dqs, joint.anchorA)
		rB := f64Rotate(stateB.dqc, stateB.dqs, joint.anchorB)

		ds := f64Add(f64Sub(stateB.dp, stateA.dp), f64Sub(rB, rA))
		linearSeparation := f64Add(joint.deltaCenter, ds)
		linearBias := f64Scale(invH*joint.correctionFactor, linearSeparation)

		Cdot := f64Sub(f64Add(vB, f64CrossSV(wB, rB)), f64Add(vA, f64CrossSV(wA, rA)))
		rhs := f64Add(Cdot, linearBias)
		det := joint.k11*joint.k22 - joint.k12*joint.k12
		if det != 0 {
			det = 1 / det
		}
		// linearMass = inverse(K) applied to rhs.
		b := f64Vec{det * (joint.k22*rhs.x - joint.k12*rhs.y), det * (joint.k11*rhs.y - joint.k12*rhs.x)}
		impulse := f64Vec{-b.x, -b.y}

		oldImpulse := joint.linearImpulse
		maxImpulse := h * joint.maxForce
		joint.linearImpulse = f64Add(joint.linearImpulse, impulse)

		if lenSq := f64Dot(joint.linearImpulse, joint.linearImpulse); lenSq > maxImpulse*maxImpulse {
			joint.linearImpulse = f64Scale(maxImpulse/math.Sqrt(lenSq), joint.linearImpulse)
		}

		impulse = f64Sub(joint.linearImpulse, oldImpulse)

		vA = f64Sub(vA, f64Scale(mA, impulse))
		wA -= iA * f64Cross(rA, impulse)
		vB = f64Add(vB, f64Scale(mB, impulse))
		wB += iB * f64Cross(rB, impulse)
	}

	stateA.v, stateA.w = vA, wA
	stateB.v, stateB.w = vB, wB
}

// TestSolveMotorJointTracksTheFloat64Mirror runs one solve with both rows
// below their clamps, on two rotated and displaced states. The angular
// separation goes through the fixed atan2, exact to 2^-20 turn, and the
// bias scales it by 240 * 0.3, so the bound is 1e-4.
func TestSolveMotorJointTracksTheFloat64Mirror(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	idA, idB := mirrorBodies(t, worldId)

	def := DefaultMotorJointDef()
	def.BodyIdA, def.BodyIdB = idA, idB
	def.LinearOffset = qv("1.4", "0.6")
	def.AngularOffset = fixed.Q32MustParse("-0.05")
	def.MaxForce = fixed.Q32FromInt(1000)
	def.MaxTorque = fixed.Q32FromInt(1000)
	jointId := CreateMotorJoint(worldId, &def)
	js := getJointSim(w, getJointFullId(w, jointId))
	stateA := getBodyState(w, getBodyFullId(w, idA))
	stateB := getBodyState(w, getBodyFullId(w, idB))

	context := jointContext(w)
	prepareJoint(js, context)

	h := qToF64(context.h)
	invH := qToF64(context.invH)
	m := &js.motorJoint
	mirror := &f64MotorJoint{
		mA: qToF64(js.invMassA), mB: qToF64(js.invMassB), iA: qToF64(js.invIA), iB: qToF64(js.invIB),
		maxForce:         qToF64(m.maxForce),
		maxTorque:        qToF64(m.maxTorque),
		correctionFactor: qToF64(m.correctionFactor),
		anchorA:          vecToF64(m.anchorA),
		anchorB:          vecToF64(m.anchorB),
		deltaCenter:      vecToF64(m.deltaCenter),
		deltaAngle:       qToF64(m.deltaAngle) * 2 * math.Pi,
	}
	rA, rB := mirror.anchorA, mirror.anchorB
	mirror.k11 = mirror.mA + mirror.mB + rA.y*rA.y*mirror.iA + rB.y*rB.y*mirror.iB
	mirror.k12 = -rA.y*rA.x*mirror.iA - rB.y*rB.x*mirror.iB
	mirror.k22 = mirror.mA + mirror.mB + rA.x*rA.x*mirror.iA + rB.x*rB.x*mirror.iB
	mirror.angularMass = 1 / (mirror.iA + mirror.iB)
	fA := stateToF64(stateA)
	fB := stateToF64(stateB)

	solveMotorJoint(js, context, true)
	f64SolveMotorJoint(mirror, &fA, &fB, h, invH)

	const limit = 1e-4
	checkMirror(t, "vA.x", stateA.linearVelocity.X, fA.v.x, limit)
	checkMirror(t, "vA.y", stateA.linearVelocity.Y, fA.v.y, limit)
	checkMirror(t, "wA", stateA.angularVelocity.Mul(tau), fA.w, limit)
	checkMirror(t, "vB.x", stateB.linearVelocity.X, fB.v.x, limit)
	checkMirror(t, "vB.y", stateB.linearVelocity.Y, fB.v.y, limit)
	checkMirror(t, "wB", stateB.angularVelocity.Mul(tau), fB.w, limit)
	checkMirror(t, "angularImpulse", m.angularImpulse, mirror.angularImpulse, limit)
	checkMirror(t, "linearImpulse.x", m.linearImpulse.X, mirror.linearImpulse.x, limit)
	checkMirror(t, "linearImpulse.y", m.linearImpulse.Y, mirror.linearImpulse.y, limit)
}
