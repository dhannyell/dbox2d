package dbox2d

import (
	"math"
	"testing"

	"github.com/dhannyell/fixed"
)

// This file tests the distance joint solver with hand-computed cases and
// with a float64 mirror of src/distance_joint.c. The composite rope lives
// in step_test.go.

// ropedBox creates a static ground at the origin and a unit box of mass
// one at the position, tied to the ground origin by a distance joint
// between the two body origins. The axis is the x axis and the effective
// mass is one.
func ropedBox(t *testing.T, worldId WorldId, position Vec2, def *DistanceJointDef) (*body, *joint) {
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
	jointId := CreateDistanceJoint(worldId, def)
	return getBodyFullId(w, boxId), getJointFullId(w, jointId)
}

// TestDistanceRigidStopsTheBox pins the rigid branch: the box at (2, 0)
// moves outward at one unit per second on a rope of length two. With
// axialMass = 1, Cdot = 1 gives impulse = -1 and vB = 0.
func TestDistanceRigidStopsTheBox(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	def := DefaultDistanceJointDef()
	def.Length = fixed.Q32FromInt(2)
	box, j := ropedBox(t, worldId, v2(2, 0), &def)
	state := getBodyState(w, box)
	state.linearVelocity = Vec2{X: fixed.Q32One()}

	context := jointContext(w)
	prepareJoints(context, j.colorIndex)
	solveJoints(context, j.colorIndex, false)

	tolerance := fixed.Q32FromRaw(1 << 12)
	js := getJointSim(w, j)
	if !withinQ(js.distanceJoint.impulse, fixed.Q32One().Neg(), tolerance) {
		t.Errorf("impulse is %v, want -1", js.distanceJoint.impulse)
	}
	if !withinQ(state.linearVelocity.X, fixed.Q32Zero(), tolerance) {
		t.Errorf("vB.x is %v, want 0", state.linearVelocity.X)
	}

	// The force report divides the impulse by the sub-step: -240 on x.
	w.invH = context.invH
	force := getDistanceJointForce(w, js)
	if !withinQ(force.X, fixed.Q32FromInt(-240), tolerance.Mul(context.invH)) {
		t.Errorf("the joint force is %v, want (-240, 0)", force)
	}
}

// TestDistanceUpperLimitHoldsTheRope pins the limit branch: the spring is
// on with zero hertz, so only the limits act. The box sits at the max
// length three and moves outward at one unit per second:
//
//	lower: C = 2 > 0, bias = 480, impulse clamped to 0
//	upper: C = 0, Cdot = -1, impulse = 1, vB.x -= 1 = 0
func TestDistanceUpperLimitHoldsTheRope(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	def := DefaultDistanceJointDef()
	def.Length = fixed.Q32FromInt(2)
	def.EnableSpring = true
	def.EnableLimit = true
	def.MinLength = fixed.Q32One()
	def.MaxLength = fixed.Q32FromInt(3)
	box, j := ropedBox(t, worldId, v2(3, 0), &def)
	state := getBodyState(w, box)
	state.linearVelocity = Vec2{X: fixed.Q32One()}

	context := jointContext(w)
	prepareJoints(context, j.colorIndex)
	solveJoints(context, j.colorIndex, false)

	tolerance := fixed.Q32FromRaw(1 << 12)
	js := getJointSim(w, j)
	if !js.distanceJoint.lowerImpulse.Eq(fixed.Q32Zero()) || !withinQ(js.distanceJoint.upperImpulse, fixed.Q32One(), tolerance) {
		t.Errorf("the limit impulses are %v and %v, want 0 and 1", js.distanceJoint.lowerImpulse, js.distanceJoint.upperImpulse)
	}
	if !withinQ(state.linearVelocity.X, fixed.Q32Zero(), tolerance) {
		t.Errorf("vB.x is %v, want 0", state.linearVelocity.X)
	}
}

// TestDistanceMotorSaturatesAtTheForce pins the motor branch: the motor
// asks for one unit per second and the axial mass one asks for the
// impulse one. The force limit clamps it at h * maxMotorForce = 100/240:
//
//	motorImpulse = 5/12, vB.x = 5/12
func TestDistanceMotorSaturatesAtTheForce(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	def := DefaultDistanceJointDef()
	def.Length = fixed.Q32FromInt(2)
	def.EnableSpring = true
	def.EnableMotor = true
	def.MotorSpeed = fixed.Q32One()
	def.MaxMotorForce = fixed.Q32FromInt(100)
	box, j := ropedBox(t, worldId, v2(2, 0), &def)
	state := getBodyState(w, box)

	context := jointContext(w)
	prepareJoints(context, j.colorIndex)
	solveJoints(context, j.colorIndex, false)

	tolerance := fixed.Q32FromRaw(1 << 12)
	js := getJointSim(w, j)
	twelfths := fixed.Q32FromRatio(5, 12)
	if !withinQ(js.distanceJoint.motorImpulse, twelfths, tolerance) {
		t.Errorf("motorImpulse is %v, want 5/12", js.distanceJoint.motorImpulse)
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

// The float64 mirror follows b2SolveDistanceJoint line by line, in
// radians.

type f64DistanceJoint struct {
	mA, mB, iA, iB     float64
	constraintSoftness f64Softness

	length, hertz, dampingRatio, minLength, maxLength float64
	maxMotorForce, motorSpeed                         float64
	impulse, lowerImpulse, upperImpulse, motorImpulse float64
	anchorA, anchorB, deltaCenter                     f64Vec
	distanceSoftness                                  f64Softness
	axialMass                                         float64
	enableSpring, enableLimit, enableMotor            bool
}

func f64Cross(a, b f64Vec) float64 { return a.x*b.y - a.y*b.x }

func f64CrossSV(s float64, v f64Vec) f64Vec { return f64Vec{-s * v.y, s * v.x} }

func f64Add(a, b f64Vec) f64Vec { return f64Vec{a.x + b.x, a.y + b.y} }

func f64Sub(a, b f64Vec) f64Vec { return f64Vec{a.x - b.x, a.y - b.y} }

func f64Scale(s float64, v f64Vec) f64Vec { return f64Vec{s * v.x, s * v.y} }

func f64Dot(a, b f64Vec) float64 { return a.x*b.x + a.y*b.y }

// f64ApplyAxial applies the impulse P at rA and rB to the four velocities.
func f64ApplyAxial(j *f64DistanceJoint, P, rA, rB f64Vec, vA, vB *f64Vec, wA, wB *float64) {
	*vA = f64Sub(*vA, f64Scale(j.mA, P))
	*wA -= j.iA * f64Cross(rA, P)
	*vB = f64Add(*vB, f64Scale(j.mB, P))
	*wB += j.iB * f64Cross(rB, P)
}

func f64SolveDistanceJoint(joint *f64DistanceJoint, stateA, stateB *f64State, h, invH float64, useBias bool) {
	vA, wA := stateA.v, stateA.w
	vB, wB := stateB.v, stateB.w

	rA := f64Rotate(stateA.dqc, stateA.dqs, joint.anchorA)
	rB := f64Rotate(stateB.dqc, stateB.dqs, joint.anchorB)

	ds := f64Add(f64Sub(stateB.dp, stateA.dp), f64Sub(rB, rA))
	separation := f64Add(joint.deltaCenter, ds)
	length := math.Hypot(separation.x, separation.y)
	axis := f64Scale(1/length, separation)

	relative := func() float64 {
		vr := f64Add(f64Sub(vB, vA), f64Sub(f64CrossSV(wB, rB), f64CrossSV(wA, rA)))
		return f64Dot(axis, vr)
	}

	if joint.enableSpring && (joint.minLength < joint.maxLength || !joint.enableLimit) {
		if joint.hertz > 0 {
			Cdot := relative()
			C := length - joint.length
			bias := joint.distanceSoftness.biasRate * C
			m := joint.distanceSoftness.massScale * joint.axialMass
			impulse := -m*(Cdot+bias) - joint.distanceSoftness.impulseScale*joint.impulse
			joint.impulse += impulse
			f64ApplyAxial(joint, f64Scale(impulse, axis), rA, rB, &vA, &vB, &wA, &wB)
		}

		if joint.enableLimit {
			{
				Cdot := relative()
				C := length - joint.minLength
				bias, massCoeff, impulseCoeff := 0.0, 1.0, 0.0
				if C > 0 {
					bias = C * invH
				} else if useBias {
					bias = joint.constraintSoftness.biasRate * C
					massCoeff = joint.constraintSoftness.massScale
					impulseCoeff = joint.constraintSoftness.impulseScale
				}
				impulse := -massCoeff*joint.axialMass*(Cdot+bias) - impulseCoeff*joint.lowerImpulse
				newImpulse := math.Max(0, joint.lowerImpulse+impulse)
				impulse = newImpulse - joint.lowerImpulse
				joint.lowerImpulse = newImpulse
				f64ApplyAxial(joint, f64Scale(impulse, axis), rA, rB, &vA, &vB, &wA, &wB)
			}
			{
				Cdot := -relative()
				C := joint.maxLength - length
				bias, massScale, impulseScale := 0.0, 1.0, 0.0
				if C > 0 {
					bias = C * invH
				} else if useBias {
					bias = joint.constraintSoftness.biasRate * C
					massScale = joint.constraintSoftness.massScale
					impulseScale = joint.constraintSoftness.impulseScale
				}
				impulse := -massScale*joint.axialMass*(Cdot+bias) - impulseScale*joint.upperImpulse
				newImpulse := math.Max(0, joint.upperImpulse+impulse)
				impulse = newImpulse - joint.upperImpulse
				joint.upperImpulse = newImpulse
				f64ApplyAxial(joint, f64Scale(-impulse, axis), rA, rB, &vA, &vB, &wA, &wB)
			}
		}

		if joint.enableMotor {
			Cdot := relative()
			impulse := joint.axialMass * (joint.motorSpeed - Cdot)
			oldImpulse := joint.motorImpulse
			maxImpulse := h * joint.maxMotorForce
			joint.motorImpulse = math.Max(-maxImpulse, math.Min(joint.motorImpulse+impulse, maxImpulse))
			impulse = joint.motorImpulse - oldImpulse
			f64ApplyAxial(joint, f64Scale(impulse, axis), rA, rB, &vA, &vB, &wA, &wB)
		}
	} else {
		Cdot := relative()
		C := length - joint.length
		bias, massScale, impulseScale := 0.0, 1.0, 0.0
		if useBias {
			bias = joint.constraintSoftness.biasRate * C
			massScale = joint.constraintSoftness.massScale
			impulseScale = joint.constraintSoftness.impulseScale
		}
		impulse := -massScale*joint.axialMass*(Cdot+bias) - impulseScale*joint.impulse
		joint.impulse += impulse
		f64ApplyAxial(joint, f64Scale(impulse, axis), rA, rB, &vA, &vB, &wA, &wB)
	}

	stateA.v, stateA.w = vA, wA
	stateB.v, stateB.w = vB, wB
}

// mirrorBodies creates two rotated dynamic boxes for the mirror tests and
// gives their states a velocity, a delta position and a delta rotation.
func mirrorBodies(t *testing.T, worldId WorldId) (BodyId, BodyId) {
	t.Helper()
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

	stateA := getBodyState(w, getBodyFullId(w, idA))
	stateB := getBodyState(w, getBodyFullId(w, idB))
	stateA.linearVelocity = qv("1", "-2")
	stateA.angularVelocity = fixed.Q32MustParse("0.3")
	stateA.deltaPosition = qv("0.01", "0.02")
	stateA.deltaRotation = fixed.RotFromTurns(fixed.Q32MustParse("0.01"))
	stateB.linearVelocity = qv("-0.5", "1")
	stateB.angularVelocity = fixed.Q32MustParse("-0.2")
	stateB.deltaPosition = qv("-0.03", "0.01")
	stateB.deltaRotation = fixed.RotFromTurns(fixed.Q32MustParse("-0.02"))
	return idA, idB
}

// checkMirror compares one Q result against the float64 mirror.
func checkMirror(t *testing.T, name string, got Q, want, limit float64) {
	t.Helper()
	if diff := math.Abs(qToF64(got) - want); diff > limit {
		t.Errorf("%s: Q %v, float64 %v, diff %g", name, qToF64(got), want, diff)
	}
}

// TestSolveDistanceJointTracksTheFloat64Mirror runs one biased solve on a
// soft joint with the spring, the motor and both limits armed. The limits
// are tight so that both act with the constraint softness.
func TestSolveDistanceJointTracksTheFloat64Mirror(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	idA, idB := mirrorBodies(t, worldId)

	def := DefaultDistanceJointDef()
	def.BodyIdA, def.BodyIdB = idA, idB
	def.LocalAnchorA = qv("0.5", "0.25")
	def.LocalAnchorB = qv("-1", "0.25")
	def.Length = fixed.Q32MustParse("1.9")
	def.EnableSpring = true
	def.Hertz = fixed.Q32FromInt(2)
	def.DampingRatio = fixed.Q32Half()
	def.EnableLimit = true
	def.MinLength = fixed.Q32MustParse("2.1")
	def.MaxLength = fixed.Q32MustParse("2.2")
	def.EnableMotor = true
	def.MotorSpeed = fixed.Q32Half()
	def.MaxMotorForce = fixed.Q32FromInt(30)
	jointId := CreateDistanceJoint(worldId, &def)
	js := getJointSim(w, getJointFullId(w, jointId))
	stateA := getBodyState(w, getBodyFullId(w, idA))
	stateB := getBodyState(w, getBodyFullId(w, idB))

	context := jointContext(w)
	prepareJoint(js, context)

	h := qToF64(context.h)
	invH := qToF64(context.invH)
	d := &js.distanceJoint
	mirror := &f64DistanceJoint{
		mA: qToF64(js.invMassA), mB: qToF64(js.invMassB), iA: qToF64(js.invIA), iB: qToF64(js.invIB),
		constraintSoftness: makeSoftF64(math.Min(qToF64(js.constraintHertz), 0.25*invH), qToF64(js.constraintDampingRatio), h),
		length:             qToF64(d.length),
		hertz:              qToF64(d.hertz),
		dampingRatio:       qToF64(d.dampingRatio),
		minLength:          qToF64(d.minLength),
		maxLength:          qToF64(d.maxLength),
		maxMotorForce:      qToF64(d.maxMotorForce),
		motorSpeed:         qToF64(d.motorSpeed),
		anchorA:            vecToF64(d.anchorA),
		anchorB:            vecToF64(d.anchorB),
		deltaCenter:        vecToF64(d.deltaCenter),
		axialMass:          qToF64(d.axialMass),
		enableSpring:       true,
		enableLimit:        true,
		enableMotor:        true,
	}
	mirror.distanceSoftness = makeSoftF64(mirror.hertz, mirror.dampingRatio, h)
	fA := stateToF64(stateA)
	fB := stateToF64(stateB)

	solveDistanceJoint(js, context, true)
	f64SolveDistanceJoint(mirror, &fA, &fB, h, invH, true)

	const limit = 1e-6
	checkMirror(t, "vA.x", stateA.linearVelocity.X, fA.v.x, limit)
	checkMirror(t, "vA.y", stateA.linearVelocity.Y, fA.v.y, limit)
	checkMirror(t, "wA", stateA.angularVelocity.Mul(tau), fA.w, limit)
	checkMirror(t, "vB.x", stateB.linearVelocity.X, fB.v.x, limit)
	checkMirror(t, "vB.y", stateB.linearVelocity.Y, fB.v.y, limit)
	checkMirror(t, "wB", stateB.angularVelocity.Mul(tau), fB.w, limit)
	checkMirror(t, "impulse", d.impulse, mirror.impulse, limit)
	checkMirror(t, "lowerImpulse", d.lowerImpulse, mirror.lowerImpulse, limit)
	checkMirror(t, "upperImpulse", d.upperImpulse, mirror.upperImpulse, limit)
	checkMirror(t, "motorImpulse", d.motorImpulse, mirror.motorImpulse, limit)
}
