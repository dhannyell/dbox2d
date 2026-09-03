package dbox2d

import (
	"math"
	"testing"

	"github.com/dhannyell/fixed"
)

// This file tests the mouse joint solver with a hand-computed case and
// with a float64 mirror of src/mouse_joint.c. The composite drag lives in
// step_test.go.

// draggedBox creates a static ground at the origin and a unit box of mass
// one (inverse inertia 6) at the origin, on a mouse joint anchored at the
// box origin. The linear mass is the identity.
func draggedBox(t *testing.T, worldId WorldId, def *MouseJointDef) (*body, *joint) {
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
	jointId := CreateMouseJoint(worldId, def)
	return getBodyFullId(w, boxId), getJointFullId(w, jointId)
}

// TestMouseJointTargetRoundTrip pins the public target accessors without
// involving the solver.
func TestMouseJointTargetRoundTrip(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	def := DefaultMouseJointDef()
	_, j := draggedBox(t, worldId, &def)
	jointId := makeJointId(w, jointPair{joint: j, jointSim: getJointSim(w, j)})

	want := Vec2{X: fixed.Q32FromRatio(3, 2), Y: fixed.Q32FromRatio(-2, 3)}
	jointId.SetTarget(want)
	got := jointId.GetTarget()
	if !got.X.Eq(want.X) || !got.Y.Eq(want.Y) {
		t.Errorf("got %v, want %v", got, want)
	}
}

// TestMousePullsTowardTheTarget pins the linear row and the force clamp:
// the joint grabs the box at its origin, then the target moves to (1, 0),
// which gives the separation (-1, 0). With the default four hertz the
// soft impulse asks for about 2.16 along x, and the clamp
// h * maxForce = 100/240 = 5/12 keeps (5/12, 0).
func TestMousePullsTowardTheTarget(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	def := DefaultMouseJointDef()
	def.MaxForce = fixed.Q32FromInt(100)
	box, j := draggedBox(t, worldId, &def)
	state := getBodyState(w, box)
	getJointSim(w, j).mouseJoint.targetA = Vec2{X: fixed.Q32One()}

	context := jointContext(w)
	prepareJoints(context, j.colorIndex)
	solveJoints(context, j.colorIndex, false)

	tolerance := fixed.Q32FromRaw(1 << 12)
	js := getJointSim(w, j)
	twelfths := fixed.Q32FromRatio(5, 12)
	impulse := js.mouseJoint.linearImpulse
	if !withinQ(impulse.X, twelfths, tolerance) || !withinQ(impulse.Y, fixed.Q32Zero(), tolerance) {
		t.Errorf("linearImpulse is %v, want (5/12, 0)", impulse)
	}
	if !withinQ(state.linearVelocity.X, twelfths, tolerance) {
		t.Errorf("vB.x is %v, want 5/12", state.linearVelocity.X)
	}

	// The force report is the saturated force; the torque is zero because
	// the box does not spin.
	w.invH = context.invH
	force := getMouseJointForce(w, js)
	if !withinQ(force.X, fixed.Q32FromInt(100), tolerance.Mul(context.invH)) {
		t.Errorf("the joint force is %v, want (100, 0)", force)
	}
	if torque := getMouseJointTorque(w, js); !torque.Eq(fixed.Q32Zero()) {
		t.Errorf("the joint torque is %v, want 0", torque)
	}

	// The warm start applies the stored impulse again on a fresh state.
	state.linearVelocity = Vec2Zero()
	warmStartJoints(context, j.colorIndex)
	if !withinQ(state.linearVelocity.X, twelfths, tolerance) {
		t.Errorf("the warm start gives vB.x %v, want 5/12", state.linearVelocity.X)
	}
}

// The float64 mirror follows b2SolveMouseJoint line by line, in radians.

type f64MouseJoint struct {
	mB, iB                          float64
	maxForce                        float64
	linearImpulse                   f64Vec
	angularImpulse                  float64
	linearSoftness, angularSoftness f64Softness
	anchorB, deltaCenter            f64Vec
	k11, k12, k22                   float64
}

func f64SolveMouseJoint(joint *f64MouseJoint, stateB *f64State, h float64) {
	mB, iB := joint.mB, joint.iB

	vB, wB := stateB.v, stateB.w

	{
		massScale := joint.angularSoftness.massScale
		impulseScale := joint.angularSoftness.impulseScale

		impulse := 0.0
		if iB > 0 {
			impulse = -wB / iB
		}
		impulse = massScale*impulse - impulseScale*joint.angularImpulse
		joint.angularImpulse += impulse

		wB += iB * impulse
	}

	maxImpulse := joint.maxForce * h

	{
		rB := f64Rotate(stateB.dqc, stateB.dqs, joint.anchorB)
		Cdot := f64Add(vB, f64CrossSV(wB, rB))

		separation := f64Add(f64Add(stateB.dp, rB), joint.deltaCenter)
		bias := f64Scale(joint.linearSoftness.biasRate, separation)

		massScale := joint.linearSoftness.massScale
		impulseScale := joint.linearSoftness.impulseScale

		rhs := f64Add(Cdot, bias)
		det := joint.k11*joint.k22 - joint.k12*joint.k12
		if det != 0 {
			det = 1 / det
		}
		b := f64Vec{det * (joint.k22*rhs.x - joint.k12*rhs.y), det * (joint.k11*rhs.y - joint.k12*rhs.x)}

		impulse := f64Vec{-massScale*b.x - impulseScale*joint.linearImpulse.x, -massScale*b.y - impulseScale*joint.linearImpulse.y}

		oldImpulse := joint.linearImpulse
		joint.linearImpulse = f64Add(joint.linearImpulse, impulse)

		if mag := math.Hypot(joint.linearImpulse.x, joint.linearImpulse.y); mag > maxImpulse {
			joint.linearImpulse = f64Scale(maxImpulse/mag, joint.linearImpulse)
		}

		impulse = f64Sub(joint.linearImpulse, oldImpulse)

		vB = f64Add(vB, f64Scale(mB, impulse))
		wB += iB * f64Cross(rB, impulse)
	}

	stateB.v, stateB.w = vB, wB
}

// TestSolveMouseJointTracksTheFloat64Mirror runs one solve below the force
// clamp on a rotated and displaced body B with an off-center anchor. No
// angle enters the error, so the bound is 1e-6.
func TestSolveMouseJointTracksTheFloat64Mirror(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	idA, idB := mirrorBodies(t, worldId)

	def := DefaultMouseJointDef()
	def.BodyIdA, def.BodyIdB = idA, idB
	def.Target = qv("1.2", "0.9")
	def.Hertz = fixed.Q32FromInt(5)
	def.DampingRatio = fixed.Q32MustParse("0.7")
	def.MaxForce = fixed.Q32FromInt(1000)
	jointId := CreateMouseJoint(worldId, &def)
	js := getJointSim(w, getJointFullId(w, jointId))
	stateB := getBodyState(w, getBodyFullId(w, idB))

	context := jointContext(w)
	prepareJoint(js, context)

	h := qToF64(context.h)
	m := &js.mouseJoint
	mirror := &f64MouseJoint{
		mB: qToF64(js.invMassB), iB: qToF64(js.invIB),
		maxForce:        qToF64(m.maxForce),
		linearSoftness:  makeSoftF64(qToF64(m.hertz), qToF64(m.dampingRatio), h),
		angularSoftness: makeSoftF64(0.5, 0.1, h),
		anchorB:         vecToF64(m.anchorB),
		deltaCenter:     vecToF64(m.deltaCenter),
	}
	rB := mirror.anchorB
	mirror.k11 = mirror.mB + mirror.iB*rB.y*rB.y
	mirror.k12 = -mirror.iB * rB.x * rB.y
	mirror.k22 = mirror.mB + mirror.iB*rB.x*rB.x
	fB := stateToF64(stateB)

	solveMouseJoint(js, context)
	f64SolveMouseJoint(mirror, &fB, h)

	const limit = 1e-6
	checkMirror(t, "vB.x", stateB.linearVelocity.X, fB.v.x, limit)
	checkMirror(t, "vB.y", stateB.linearVelocity.Y, fB.v.y, limit)
	checkMirror(t, "wB", stateB.angularVelocity.Mul(tau), fB.w, limit)
	checkMirror(t, "angularImpulse", m.angularImpulse, mirror.angularImpulse, limit)
	checkMirror(t, "linearImpulse.x", m.linearImpulse.X, mirror.linearImpulse.x, limit)
	checkMirror(t, "linearImpulse.y", m.linearImpulse.Y, mirror.linearImpulse.y, limit)
}
