package dbox2d

import (
	"testing"

	"github.com/dhannyell/fixed"
)

// This file tests the joint bookkeeping: the body lists, the set choice,
// the graph color and the contact filter. The solver stages have their
// own files.

// createEachJoint creates one joint of every type between two bodies and
// returns the ids in JointType order.
func createEachJoint(t *testing.T, worldId WorldId, idA, idB BodyId) [8]JointId {
	t.Helper()
	var ids [8]JointId

	distance := DefaultDistanceJointDef()
	distance.BodyIdA, distance.BodyIdB = idA, idB
	ids[DistanceJoint] = CreateDistanceJoint(worldId, &distance)

	filter := DefaultFilterJointDef()
	filter.BodyIdA, filter.BodyIdB = idA, idB
	ids[FilterJoint] = CreateFilterJoint(worldId, &filter)

	motor := DefaultMotorJointDef()
	motor.BodyIdA, motor.BodyIdB = idA, idB
	ids[MotorJoint] = CreateMotorJoint(worldId, &motor)

	mouse := DefaultMouseJointDef()
	mouse.BodyIdA, mouse.BodyIdB = idA, idB
	ids[MouseJoint] = CreateMouseJoint(worldId, &mouse)

	prismatic := DefaultPrismaticJointDef()
	prismatic.BodyIdA, prismatic.BodyIdB = idA, idB
	ids[PrismaticJoint] = CreatePrismaticJoint(worldId, &prismatic)

	revolute := DefaultRevoluteJointDef()
	revolute.BodyIdA, revolute.BodyIdB = idA, idB
	ids[RevoluteJoint] = CreateRevoluteJoint(worldId, &revolute)

	weld := DefaultWeldJointDef()
	weld.BodyIdA, weld.BodyIdB = idA, idB
	ids[WeldJoint] = CreateWeldJoint(worldId, &weld)

	wheel := DefaultWheelJointDef()
	wheel.BodyIdA, wheel.BodyIdB = idA, idB
	ids[WheelJoint] = CreateWheelJoint(worldId, &wheel)

	return ids
}

// TestEachJointTypeTakesItsOwnColor pins the creation path of every type:
// the joint lands in the awake set, in a color of its own, with the sim
// type set and both body lists linked.
func TestEachJointTypeTakesItsOwnColor(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	idA := addDynamicCircle(t, worldId, v2(0, 0))
	idB := addDynamicCircle(t, worldId, v2(3, 0))

	ids := createEachJoint(t, worldId, idA, idB)

	seen := make(map[int]bool)
	for jointType, jointId := range ids {
		j := getJointFullId(w, jointId)
		if j.jointType != JointType(jointType) {
			t.Errorf("joint %d has type %d", jointType, j.jointType)
		}
		if j.setIndex != awakeSet || j.colorIndex == nullIndex || j.colorIndex == overflowIndex {
			t.Errorf("joint %d is in set %d color %d", jointType, j.setIndex, j.colorIndex)
		}
		if seen[j.colorIndex] {
			t.Errorf("joint %d shares color %d", jointType, j.colorIndex)
		}
		seen[j.colorIndex] = true
		js := getJointSim(w, j)
		if js.jointType != JointType(jointType) || js.jointId != j.jointId {
			t.Errorf("the sim of joint %d has type %d and id %d", jointType, js.jointType, js.jointId)
		}
	}

	bodyA := getBodyFullId(w, idA)
	bodyB := getBodyFullId(w, idB)
	if bodyA.jointCount != 8 || bodyB.jointCount != 8 {
		t.Errorf("the bodies count %d and %d joints, want 8", bodyA.jointCount, bodyB.jointCount)
	}
	if bodyA.islandId != bodyB.islandId {
		t.Errorf("the joints did not merge the islands")
	}
	validateWorld(w)

	for _, jointId := range ids {
		DestroyJoint(jointId)
	}
	if w.jointIdPool.idCount() != 0 || bodyA.jointCount != 0 || bodyB.jointCount != 0 {
		t.Errorf("%d joints survive; the bodies count %d and %d", w.jointIdPool.idCount(), bodyA.jointCount, bodyB.jointCount)
	}
	validateWorld(w)
}

// TestJointToAStaticBodyKeepsTheColorFree pins the color rule for joints
// with a static body: the joint goes to the awake set and only the
// dynamic body takes a bit in the color.
func TestJointToAStaticBodyKeepsTheColorFree(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	groundId := addStaticCircle(t, worldId, v2(0, -1))
	idA := addDynamicCircle(t, worldId, v2(0, 2))

	def := DefaultRevoluteJointDef()
	def.BodyIdA, def.BodyIdB = groundId, idA
	jointId := CreateRevoluteJoint(worldId, &def)

	j := getJointFullId(w, jointId)
	ground := getBodyFullId(w, groundId)
	bodyA := getBodyFullId(w, idA)
	if j.setIndex != awakeSet || j.colorIndex == nullIndex {
		t.Fatalf("the joint is in set %d color %d", j.setIndex, j.colorIndex)
	}
	color := &w.constraintGraph.colors[j.colorIndex]
	if color.bodySet.getBit(ground.id) || !color.bodySet.getBit(bodyA.id) {
		t.Errorf("the color bits are wrong: ground %v, body %v", color.bodySet.getBit(ground.id), color.bodySet.getBit(bodyA.id))
	}
	if j.islandId != bodyA.islandId {
		t.Errorf("the joint is in island %d, the body in %d", j.islandId, bodyA.islandId)
	}
	validateWorld(w)
}

// TestJointBetweenStaticBodiesHasNoIsland pins the static branch: the
// joint lives in the static set and joins no island.
func TestJointBetweenStaticBodiesHasNoIsland(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	idA := addStaticCircle(t, worldId, v2(0, 0))
	idB := addStaticCircle(t, worldId, v2(2, 0))

	def := DefaultWeldJointDef()
	def.BodyIdA, def.BodyIdB = idA, idB
	jointId := CreateWeldJoint(worldId, &def)

	j := getJointFullId(w, jointId)
	if j.setIndex != staticSet || j.colorIndex != nullIndex || j.islandId != nullIndex {
		t.Errorf("the joint is in set %d color %d island %d", j.setIndex, j.colorIndex, j.islandId)
	}
	if len(w.solverSets[staticSet].jointSims) != 1 {
		t.Errorf("the static set holds %d joints", len(w.solverSets[staticSet].jointSims))
	}
	validateWorld(w)

	DestroyJoint(jointId)
	validateWorld(w)
}

// TestJointWakesTheSleepingBody pins the awake branch: a joint between an
// awake body and a sleeping body wakes the sleeping set.
func TestJointWakesTheSleepingBody(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	idA, _, sleepIndex := sleepPair(t, w, worldId)
	idC := addDynamicCircle(t, worldId, v2(5, 0))

	def := DefaultDistanceJointDef()
	def.BodyIdA, def.BodyIdB = idC, idA
	jointId := CreateDistanceJoint(worldId, &def)

	j := getJointFullId(w, jointId)
	bodyA := getBodyFullId(w, idA)
	if bodyA.setIndex != awakeSet {
		t.Errorf("body A stays in set %d", bodyA.setIndex)
	}
	if j.setIndex != awakeSet || j.colorIndex == nullIndex {
		t.Errorf("the joint is in set %d color %d", j.setIndex, j.colorIndex)
	}
	if sleepIndex < len(w.solverSets) && w.solverSets[sleepIndex].setIndex == sleepIndex {
		t.Errorf("the sleeping set %d survives", sleepIndex)
	}
	validateWorld(w)
}

// TestJointToASleepingBodySleeps pins the sleeping branch: a joint between
// a static body and a sleeping body joins the sleeping set, the removal
// fixes the moved sim, and DestroyJoint wakes the bodies.
func TestJointToASleepingBodySleeps(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	groundId := addStaticCircle(t, worldId, v2(0, -2))
	idA, idB, sleepIndex := sleepPair(t, w, worldId)

	defA := DefaultRevoluteJointDef()
	defA.BodyIdA, defA.BodyIdB = groundId, idA
	jointIdA := CreateRevoluteJoint(worldId, &defA)
	defB := DefaultRevoluteJointDef()
	defB.BodyIdA, defB.BodyIdB = groundId, idB
	jointIdB := CreateRevoluteJoint(worldId, &defB)

	jA := getJointFullId(w, jointIdA)
	jB := getJointFullId(w, jointIdB)
	bodyA := getBodyFullId(w, idA)
	if jA.setIndex != sleepIndex || jA.localIndex != 0 || jB.localIndex != 1 {
		t.Fatalf("the joints are in set %d at %d and %d", jA.setIndex, jA.localIndex, jB.localIndex)
	}
	if jA.islandId != bodyA.islandId || bodyA.setIndex != sleepIndex {
		t.Errorf("the joint is in island %d and the body in %d, set %d", jA.islandId, bodyA.islandId, bodyA.setIndex)
	}
	validateWorld(w)

	// The first removal swaps the second sim into slot zero.
	destroyJointInternal(w, jA, false)
	if jB.localIndex != 0 || w.solverSets[sleepIndex].jointSims[0].jointId != jB.jointId {
		t.Errorf("the moved joint is at %d", jB.localIndex)
	}
	validateWorld(w)

	DestroyJoint(jointIdB)
	if bodyA.setIndex != awakeSet {
		t.Errorf("DestroyJoint left body A in set %d", bodyA.setIndex)
	}
	validateWorld(w)
}

// TestJointWithoutCollisionRemovesTheContact pins collideConnected: the
// joint destroys the contact that exists and shouldBodiesCollide rejects
// the pair so the broad phase creates no new one.
func TestJointWithoutCollisionRemovesTheContact(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	boxId := boxOnGround(t, worldId, fixed.Q32Zero())
	box := getBodyFullId(w, boxId)
	groundId := BodyId{index1: 1, world0: w.worldId, generation: w.bodies[0].generation}

	dt := stepDt()
	Step(worldId, dt, 4)
	if w.contactIdPool.idCount() != 1 {
		t.Fatalf("the contact did not survive the first step")
	}

	def := DefaultRevoluteJointDef()
	def.BodyIdA, def.BodyIdB = groundId, boxId
	CreateRevoluteJoint(worldId, &def)

	if w.contactIdPool.idCount() != 0 || box.contactCount != 0 {
		t.Errorf("the contact survives the joint: %d contacts", w.contactIdPool.idCount())
	}
	if shouldBodiesCollide(w, &w.bodies[0], box) {
		t.Errorf("shouldBodiesCollide accepts the filtered pair")
	}
	Step(worldId, dt, 4)
	if w.contactIdPool.idCount() != 0 {
		t.Errorf("the next step recreated the contact")
	}
	validateWorld(w)
}

// TestDestroyBodyDestroysItsJoints pins the body hook: the joints go away
// with the body and the other bodies count zero joints.
func TestDestroyBodyDestroysItsJoints(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	idA := addDynamicCircle(t, worldId, v2(0, 0))
	idB := addDynamicCircle(t, worldId, v2(3, 0))
	idC := addDynamicCircle(t, worldId, v2(6, 0))

	defAB := DefaultWeldJointDef()
	defAB.BodyIdA, defAB.BodyIdB = idA, idB
	CreateWeldJoint(worldId, &defAB)
	defAC := DefaultWeldJointDef()
	defAC.BodyIdA, defAC.BodyIdB = idA, idC
	CreateWeldJoint(worldId, &defAC)

	DestroyBody(idA)

	bodyB := getBodyFullId(w, idB)
	bodyC := getBodyFullId(w, idC)
	if w.jointIdPool.idCount() != 0 || bodyB.jointCount != 0 || bodyC.jointCount != 0 {
		t.Errorf("%d joints survive; B counts %d, C counts %d", w.jointIdPool.idCount(), bodyB.jointCount, bodyC.jointCount)
	}
	if bodyB.headJointKey != nullIndex || bodyC.headJointKey != nullIndex {
		t.Errorf("the body lists still point at a joint")
	}
	validateWorld(w)
}

// TestRevoluteRejectsAFullTurnLimit pins the angle unit of the limit
// check: the reference bound of 0.99 pi is 0.495 turns.
func TestRevoluteRejectsAFullTurnLimit(t *testing.T) {
	worldId := createTestWorld(t)
	idA := addDynamicCircle(t, worldId, v2(0, 0))
	idB := addDynamicCircle(t, worldId, v2(3, 0))

	def := DefaultRevoluteJointDef()
	def.BodyIdA, def.BodyIdB = idA, idB
	def.EnableLimit = true
	def.UpperAngle = fixed.Q32Half()
	requirePanic(t, func() { CreateRevoluteJoint(worldId, &def) })

	def.UpperAngle = fixed.Q32MustParse("0.495")
	CreateRevoluteJoint(worldId, &def)
}
