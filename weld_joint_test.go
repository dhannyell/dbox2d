package dbox2d

import (
	"math"
	"testing"

	"github.com/dhannyell/fixed"
)

// This file tests the weld joint solver with hand-computed cases and with
// a float64 mirror of src/weld_joint.c. The composite weld lives in
// step_test.go.

// weldedBox creates a static ground at the origin and a unit box of mass
// one (inverse inertia 6) at the position, welded to the ground at the
// world origin. It returns the box body and the joint.
func weldedBox(t *testing.T, worldId WorldId, position Vec2, def *WeldJointDef) (*body, *joint) {
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
	def.LocalAnchorB = Vec2{X: position.X.Neg(), Y: position.Y.Neg()}
	jointId := CreateWeldJoint(worldId, def)
	return getBodyFullId(w, boxId), getJointFullId(w, jointId)
}

// TestWeldHoldsTheAnchor pins the linear block: the box hangs one unit to
// the right of the weld and falls at one unit per second without spin.
// The angular row sees Cdot = 0, so the linear row alone acts, as in the
// revolute case: impulse (0, 1/7), vB = (0, -6/7), wB = -6/7.
func TestWeldHoldsTheAnchor(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	def := DefaultWeldJointDef()
	box, j := weldedBox(t, worldId, v2(1, 0), &def)
	state := getBodyState(w, box)
	state.linearVelocity = Vec2{Y: fixed.Q32One().Neg()}

	context := jointContext(w)
	prepareJoints(context, j.colorIndex)
	solveJoints(context, j.colorIndex, false)

	tolerance := fixed.Q32FromRaw(1 << 12)
	js := getJointSim(w, j)
	seventh := fixed.Q32FromRatio(1, 7)
	if !withinQ(js.weldJoint.linearImpulse.X, fixed.Q32Zero(), tolerance) || !withinQ(js.weldJoint.linearImpulse.Y, seventh, tolerance) {
		t.Errorf("linearImpulse is %v, want (0, 1/7)", js.weldJoint.linearImpulse)
	}
	if !withinQ(state.angularVelocity.Mul(tau), fixed.Q32FromRatio(-6, 7), tolerance) {
		t.Errorf("wB is %v rad/s, want -6/7", state.angularVelocity.Mul(tau))
	}

	// The force report divides the impulse by the sub-step: 240/7.
	w.invH = context.invH
	force := getWeldJointForce(w, js)
	if !withinQ(force.Y, fixed.Q32FromRatio(240, 7), tolerance.Mul(context.invH)) {
		t.Errorf("the joint force is %v, want (0, 240/7)", force)
	}
}

// TestWeldStopsTheSpin pins the angular row: the box at the origin spins
// at one radian per second. With axialMass = 1/6 the impulse is -1/6 and
// wB drops to zero; the torque report is -40.
func TestWeldStopsTheSpin(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	def := DefaultWeldJointDef()
	box, j := weldedBox(t, worldId, v2(0, 0), &def)
	state := getBodyState(w, box)
	state.angularVelocity = fixed.Q32One().Div(tau)

	context := jointContext(w)
	prepareJoints(context, j.colorIndex)
	solveJoints(context, j.colorIndex, false)

	tolerance := fixed.Q32FromRaw(1 << 12)
	js := getJointSim(w, j)
	if !withinQ(js.weldJoint.angularImpulse, fixed.Q32FromRatio(-1, 6), tolerance) {
		t.Errorf("angularImpulse is %v, want -1/6", js.weldJoint.angularImpulse)
	}
	if !withinQ(state.angularVelocity, fixed.Q32Zero(), tolerance) {
		t.Errorf("wB is %v turns/s, want 0", state.angularVelocity)
	}

	w.invH = context.invH
	if torque := getWeldJointTorque(w, js); !withinQ(torque, fixed.Q32FromInt(-40), tolerance.Mul(context.invH)) {
		t.Errorf("the joint torque is %v, want -40", torque)
	}

	// The warm start applies the stored impulse again on a fresh state.
	state.angularVelocity = fixed.Q32One().Div(tau)
	warmStartJoints(context, j.colorIndex)
	if !withinQ(state.angularVelocity, fixed.Q32Zero(), tolerance) {
		t.Errorf("the warm start gives wB %v turns/s, want 0", state.angularVelocity)
	}
}

// The float64 mirror follows b2SolveWeldJoint line by line, in radians.

type f64WeldJoint struct {
	mA, mB, iA, iB                  float64
	linearHertz, angularHertz       float64
	linearSoftness, angularSoftness f64Softness
	linearImpulse                   f64Vec
	angularImpulse                  float64
	anchorA, anchorB, deltaCenter   f64Vec
	deltaAngle, axialMass           float64
}

func f64SolveWeldJoint(joint *f64WeldJoint, stateA, stateB *f64State, useBias bool) {
	mA, mB, iA, iB := joint.mA, joint.mB, joint.iA, joint.iB

	vA, wA := stateA.v, stateA.w
	vB, wB := stateB.v, stateB.w

	{
		bias, massScale, impulseScale := 0.0, 1.0, 0.0
		if useBias || joint.angularHertz > 0 {
			C := f64RelativeAngle(stateB.dqc, stateB.dqs, stateA.dqc, stateA.dqs) + joint.deltaAngle
			bias = joint.angularSoftness.biasRate * C
			massScale = joint.angularSoftness.massScale
			impulseScale = joint.angularSoftness.impulseScale
		}

		Cdot := wB - wA
		impulse := -massScale*joint.axialMass*(Cdot+bias) - impulseScale*joint.angularImpulse
		joint.angularImpulse += impulse

		wA -= iA * impulse
		wB += iB * impulse
	}

	{
		rA := f64Rotate(stateA.dqc, stateA.dqs, joint.anchorA)
		rB := f64Rotate(stateB.dqc, stateB.dqs, joint.anchorB)

		bias := f64Vec{}
		massScale, impulseScale := 1.0, 0.0
		if useBias || joint.linearHertz > 0 {
			C := f64Add(f64Add(f64Sub(stateB.dp, stateA.dp), f64Sub(rB, rA)), joint.deltaCenter)
			bias = f64Scale(joint.linearSoftness.biasRate, C)
			massScale = joint.linearSoftness.massScale
			impulseScale = joint.linearSoftness.impulseScale
		}

		Cdot := f64Sub(f64Add(vB, f64CrossSV(wB, rB)), f64Add(vA, f64CrossSV(wA, rA)))

		k11 := mA + mB + rA.y*rA.y*iA + rB.y*rB.y*iB
		k12 := -rA.y*rA.x*iA - rB.y*rB.x*iB
		k22 := mA + mB + rA.x*rA.x*iA + rB.x*rB.x*iB
		det := k11*k22 - k12*k12
		if det != 0 {
			det = 1 / det
		}
		rhs := f64Add(Cdot, bias)
		b := f64Vec{det * (k22*rhs.x - k12*rhs.y), det * (k11*rhs.y - k12*rhs.x)}

		impulse := f64Vec{-massScale*b.x - impulseScale*joint.linearImpulse.x, -massScale*b.y - impulseScale*joint.linearImpulse.y}
		joint.linearImpulse = f64Add(joint.linearImpulse, impulse)

		vA = f64Sub(vA, f64Scale(mA, impulse))
		wA -= iA * f64Cross(rA, impulse)
		vB = f64Add(vB, f64Scale(mB, impulse))
		wB += iB * f64Cross(rB, impulse)
	}

	stateA.v, stateA.w = vA, wA
	stateB.v, stateB.w = vB, wB
}

// TestSolveWeldJointTracksTheFloat64Mirror runs one biased solve on a
// soft weld with both springs armed, on two rotated and displaced states.
// The angular error goes through the fixed atan2, exact to 2^-20 turn, so
// the bound is 1e-5.
func TestSolveWeldJointTracksTheFloat64Mirror(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	idA, idB := mirrorBodies(t, worldId)

	def := DefaultWeldJointDef()
	def.BodyIdA, def.BodyIdB = idA, idB
	def.LocalAnchorA = qv("0.5", "0.25")
	def.LocalAnchorB = qv("-1", "0.25")
	def.ReferenceAngle = fixed.Q32MustParse("0.02")
	def.LinearHertz = fixed.Q32FromInt(2)
	def.LinearDampingRatio = fixed.Q32Half()
	def.AngularHertz = fixed.Q32FromInt(3)
	def.AngularDampingRatio = fixed.Q32MustParse("0.7")
	jointId := CreateWeldJoint(worldId, &def)
	js := getJointSim(w, getJointFullId(w, jointId))
	stateA := getBodyState(w, getBodyFullId(w, idA))
	stateB := getBodyState(w, getBodyFullId(w, idB))

	context := jointContext(w)
	prepareJoint(js, context)

	h := qToF64(context.h)
	wj := &js.weldJoint
	mirror := &f64WeldJoint{
		mA: qToF64(js.invMassA), mB: qToF64(js.invMassB), iA: qToF64(js.invIA), iB: qToF64(js.invIB),
		linearHertz:  qToF64(wj.linearHertz),
		angularHertz: qToF64(wj.angularHertz),
		anchorA:      vecToF64(wj.anchorA),
		anchorB:      vecToF64(wj.anchorB),
		deltaCenter:  vecToF64(wj.deltaCenter),
		deltaAngle:   qToF64(wj.deltaAngle) * 2 * math.Pi,
	}
	mirror.axialMass = 1 / (mirror.iA + mirror.iB)
	mirror.linearSoftness = makeSoftF64(mirror.linearHertz, qToF64(wj.linearDampingRatio), h)
	mirror.angularSoftness = makeSoftF64(mirror.angularHertz, qToF64(wj.angularDampingRatio), h)
	fA := stateToF64(stateA)
	fB := stateToF64(stateB)

	solveWeldJoint(js, context, true)
	f64SolveWeldJoint(mirror, &fA, &fB, true)

	const limit = 1e-5
	checkMirror(t, "vA.x", stateA.linearVelocity.X, fA.v.x, limit)
	checkMirror(t, "vA.y", stateA.linearVelocity.Y, fA.v.y, limit)
	checkMirror(t, "wA", stateA.angularVelocity.Mul(tau), fA.w, limit)
	checkMirror(t, "vB.x", stateB.linearVelocity.X, fB.v.x, limit)
	checkMirror(t, "vB.y", stateB.linearVelocity.Y, fB.v.y, limit)
	checkMirror(t, "wB", stateB.angularVelocity.Mul(tau), fB.w, limit)
	checkMirror(t, "angularImpulse", wj.angularImpulse, mirror.angularImpulse, limit)
	checkMirror(t, "linearImpulse.x", wj.linearImpulse.X, mirror.linearImpulse.x, limit)
	checkMirror(t, "linearImpulse.y", wj.linearImpulse.Y, mirror.linearImpulse.y, limit)
}
