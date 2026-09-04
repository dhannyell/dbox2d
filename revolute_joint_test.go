package dbox2d

import (
	"testing"

	"github.com/dhannyell/fixed"
)

// This file tests the revolute joint solver with hand-computed cases: one
// prepare and one solve iteration with a known sub-step. The composite
// pendulum lives in step_test.go.

// jointContext returns the step context of one 4 sub-step frame at 60 Hz
// over the awake set of the world.
func jointContext(w *world) *stepContext {
	context := &stepContext{world: w}
	context.dt = fixed.Q32FromRatio(1, 60)
	context.subStepCount = 4
	context.invDt = fixed.Q32FromInt(60)
	context.h = fixed.Q32FromRatio(1, 240)
	context.invH = fixed.Q32FromInt(240)
	context.maxLinearVelocity = w.maxLinearSpeed
	context.enableWarmStarting = w.enableWarmStarting
	context.graph = &w.constraintGraph
	awake := &w.solverSets[awakeSet]
	context.sims = awake.bodySims
	context.states = awake.bodyStates
	return context
}

// pinnedBox creates a static ground at the origin and a unit box of mass
// one (inverse inertia 6) at the position, pinned to the ground by a
// revolute joint at the world origin. It returns the box body and the
// joint.
func pinnedBox(t *testing.T, worldId WorldId, position Vec2, def *RevoluteJointDef) (*body, *joint) {
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
	jointId := CreateRevoluteJoint(worldId, def)
	return getBodyFullId(w, boxId), getJointFullId(w, jointId)
}

// TestRevoluteHoldsTheAnchor pins the point constraint: the box hangs one
// unit to the right of the pivot and falls at one unit per second. With
// rB = (-1, 0), mB = 1, iB = 6:
//
//	K = [1 0; 0 1 + 6] and Cdot = (0, -1), so b = (0, -1/7)
//	impulse = (0, 1/7), vB = (0, -6/7), wB = 6 * cross(rB, impulse) = -6/7
//
// and the anchor velocity vB + wB x rB is zero.
func TestRevoluteHoldsTheAnchor(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	def := DefaultRevoluteJointDef()
	box, j := pinnedBox(t, worldId, v2(1, 0), &def)
	state := getBodyState(w, box)
	state.linearVelocity = Vec2{Y: fixed.Q32One().Neg()}

	context := jointContext(w)
	prepareJoints(context, j.colorIndex)
	solveJoints(context, j.colorIndex, false)

	tolerance := fixed.Q32FromRaw(1 << 12)
	js := getJointSim(w, j)
	seventh := fixed.Q32FromRatio(1, 7)
	if !withinQ(js.revoluteJoint.linearImpulse.X, fixed.Q32Zero(), tolerance) || !withinQ(js.revoluteJoint.linearImpulse.Y, seventh, tolerance) {
		t.Errorf("linearImpulse is %v, want (0, 1/7)", js.revoluteJoint.linearImpulse)
	}
	wB := state.angularVelocity.Mul(tau)
	if !withinQ(wB, fixed.Q32FromRatio(-6, 7), tolerance) {
		t.Errorf("wB is %v rad/s, want -6/7", wB)
	}
	anchorVelocity := state.linearVelocity.Add(CrossSV(wB, Vec2{X: fixed.Q32One().Neg()}))
	if !withinQ(anchorVelocity.X, fixed.Q32Zero(), tolerance) || !withinQ(anchorVelocity.Y, fixed.Q32Zero(), tolerance) {
		t.Errorf("the anchor moves at %v", anchorVelocity)
	}

	// The force report divides the impulse by the sub-step: 240/7.
	w.invH = context.invH
	force := getRevoluteJointForce(w, js)
	if !withinQ(force.Y, fixed.Q32FromRatio(240, 7), tolerance.Mul(context.invH)) {
		t.Errorf("the joint force is %v, want (0, 240/7)", force)
	}
}

// TestRevoluteLimitStopsTheSpin pins the limit branch: with lower and
// upper angle both zero and the box spinning at one radian per second,
// the upper limit absorbs the spin. With axialMass = 1/6:
//
//	lower: Cdot = wB = 1, impulse = -1/6, clamped to 0
//	upper: Cdot = -wB = -1, impulse = 1/6, wB -= 6 * 1/6 = 0
//
// A sign swap between the two limits leaves wB at 2.
func TestRevoluteLimitStopsTheSpin(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	def := DefaultRevoluteJointDef()
	def.EnableLimit = true
	box, j := pinnedBox(t, worldId, v2(0, 0), &def)
	state := getBodyState(w, box)
	state.angularVelocity = fixed.Q32One().Div(tau)

	context := jointContext(w)
	prepareJoints(context, j.colorIndex)
	solveJoints(context, j.colorIndex, false)

	tolerance := fixed.Q32FromRaw(1 << 12)
	js := getJointSim(w, j)
	sixth := fixed.Q32FromRatio(1, 6)
	if !js.revoluteJoint.lowerImpulse.Eq(fixed.Q32Zero()) || !withinQ(js.revoluteJoint.upperImpulse, sixth, tolerance) {
		t.Errorf("the limit impulses are %v and %v, want 0 and 1/6", js.revoluteJoint.lowerImpulse, js.revoluteJoint.upperImpulse)
	}
	if !withinQ(state.angularVelocity, fixed.Q32Zero(), tolerance) {
		t.Errorf("wB is %v turns/s, want 0", state.angularVelocity)
	}
}

// TestRevoluteMotorSaturatesAtTheTorque pins the motor branch and the
// unit of the motor speed. The motor asks for one turn per second, which
// is tau radians per second, and the axial mass 1/6 asks for the impulse
// tau/6. The torque limit clamps it at h * maxMotorTorque = 100/240:
//
//	motorImpulse = 5/12, wB = 6 * 5/12 = 2.5 rad/s
func TestRevoluteMotorSaturatesAtTheTorque(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	def := DefaultRevoluteJointDef()
	def.EnableMotor = true
	def.MotorSpeed = fixed.Q32One()
	def.MaxMotorTorque = fixed.Q32FromInt(100)
	box, j := pinnedBox(t, worldId, v2(0, 0), &def)
	state := getBodyState(w, box)

	context := jointContext(w)
	prepareJoints(context, j.colorIndex)
	solveJoints(context, j.colorIndex, false)

	tolerance := fixed.Q32FromRaw(1 << 12)
	js := getJointSim(w, j)
	if !withinQ(js.revoluteJoint.motorImpulse, fixed.Q32FromRatio(5, 12), tolerance) {
		t.Errorf("motorImpulse is %v, want 5/12", js.revoluteJoint.motorImpulse)
	}
	wB := state.angularVelocity.Mul(tau)
	if !withinQ(wB, fixed.Q32FromRatio(5, 2), tolerance) {
		t.Errorf("wB is %v rad/s, want 2.5", wB)
	}

	// The torque report is the saturated motor torque.
	w.invH = context.invH
	if torque := getRevoluteJointTorque(w, js); !withinQ(torque, fixed.Q32FromInt(100), tolerance.Mul(context.invH)) {
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
