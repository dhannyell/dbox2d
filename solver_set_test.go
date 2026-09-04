package dbox2d

import (
	"testing"

	"github.com/dhannyell/fixed"
)

// sleepPair builds two touching dynamic bodies, merges their island and
// puts it to sleep. It returns the body ids and the sleeping set index.
func sleepPair(t *testing.T, w *world, worldId WorldId) (idA, idB BodyId, sleepIndex int) {
	t.Helper()
	idA = addDynamicCircle(t, worldId, v2(0, 0))
	idB = addDynamicCircle(t, worldId, v2(1, 0))
	startTouching(t, w, idA, idB)
	mergeAwakeIslands(w)

	bodyA := getBodyFullId(w, idA)
	trySleepIsland(w, bodyA.islandId)
	return idA, idB, bodyA.setIndex
}

// addStaticCircle creates a static body with a circle of radius one half.
func addStaticCircle(t *testing.T, worldId WorldId, position Vec2) BodyId {
	t.Helper()
	bodyDef := DefaultBodyDef()
	bodyDef.Position = position
	bodyId := CreateBody(worldId, &bodyDef)
	shapeDef := DefaultShapeDef()
	circle := Circle{Radius: fixed.Q32Half()}
	CreateCircleShape(bodyId, &shapeDef, &circle)
	return bodyId
}

// TestSleepMovesTheIsland pins trySleepIsland: bodies, touching contacts
// and the island move to a fresh set, the graph frees its bits, and a
// non-touching contact with a sleeping body lands in the disabled set.
func TestSleepMovesTheIsland(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)

	groundId := addStaticCircle(t, worldId, v2(0, -1))
	idA, idB, sleepIndex := sleepPair(t, w, worldId)
	bodyA := getBodyFullId(w, idA)
	bodyB := getBodyFullId(w, idB)

	// A non-touching contact with the ground appears while asleep and
	// lands in the disabled set, as createContact places it.
	createContact(w, firstShape(w, groundId), firstShape(w, idA))
	ground := &w.contacts[len(w.contacts)-1]

	if sleepIndex < firstSleepingSet {
		t.Fatalf("the island sleeps in set %d, want a sleeping set", sleepIndex)
	}
	if bodyB.setIndex != sleepIndex {
		t.Errorf("body B is in set %d, want %d", bodyB.setIndex, sleepIndex)
	}
	sleepSet := &w.solverSets[sleepIndex]
	if len(sleepSet.bodySims) != 2 || len(sleepSet.contactSims) != 1 || len(sleepSet.islandSims) != 1 {
		t.Errorf("the sleeping set holds %d bodies, %d contacts and %d islands, want 2, 1 and 1", len(sleepSet.bodySims), len(sleepSet.contactSims), len(sleepSet.islandSims))
	}
	if w.islands[bodyA.islandId].setIndex != sleepIndex {
		t.Errorf("the island stays in set %d", w.islands[bodyA.islandId].setIndex)
	}
	touching := &w.contacts[sleepSet.contactSims[0].contactId]
	if touching.colorIndex != nullIndex || touching.setIndex != sleepIndex {
		t.Errorf("the touching contact keeps color %d in set %d", touching.colorIndex, touching.setIndex)
	}
	if w.constraintGraph.colors[0].bodySet.getBit(bodyA.id) || len(w.constraintGraph.colors[0].contactSims) != 0 {
		t.Errorf("color 0 keeps the sleeping contact")
	}
	if ground.setIndex != disabledSet {
		t.Errorf("the non-touching contact is in set %d, want the disabled set", ground.setIndex)
	}
	awake := &w.solverSets[awakeSet]
	if len(awake.bodySims) != 0 || len(awake.bodyStates) != 0 || len(awake.islandSims) != 0 {
		t.Errorf("the awake set is not empty")
	}
	validateSolverSets(w)

	// A pending split keeps the island awake.
	wakeSolverSet(w, sleepIndex)
	w.islands[bodyA.islandId].constraintRemoveCount = 1
	trySleepIsland(w, bodyA.islandId)
	if bodyA.setIndex != awakeSet {
		t.Errorf("an island with a pending split fell asleep")
	}
}

// TestSleepDisablesNonTouchingContactOfSleepingPair pins the ownership
// rule of trySleepIsland: an awake non-touching contact moves to the
// disabled set only when the other body is not awake.
func TestSleepDisablesNonTouchingContactOfSleepingPair(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)

	idA := addDynamicCircle(t, worldId, v2(0, 0))
	idB := addDynamicCircle(t, worldId, v2(3, 0))
	createContact(w, firstShape(w, idA), firstShape(w, idB))
	c := &w.contacts[len(w.contacts)-1]

	bodyA := getBodyFullId(w, idA)
	bodyB := getBodyFullId(w, idB)

	trySleepIsland(w, bodyA.islandId)
	if c.setIndex != awakeSet {
		t.Fatalf("the contact left the awake set while B is awake")
	}

	trySleepIsland(w, bodyB.islandId)
	if c.setIndex != disabledSet {
		t.Errorf("the contact is in set %d, want the disabled set", c.setIndex)
	}
	validateSolverSets(w)
}

// TestWakeRestoresTheAwakeSet pins wakeSolverSet: everything returns to
// the awake set with a fresh state and a zero sleep time, the touching
// contact re-enters the graph, and the disabled contact follows.
func TestWakeRestoresTheAwakeSet(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)

	groundId := addStaticCircle(t, worldId, v2(0, -1))
	idA, idB, sleepIndex := sleepPair(t, w, worldId)
	bodyA := getBodyFullId(w, idA)
	bodyB := getBodyFullId(w, idB)
	bodyA.sleepTime = fixed.Q32One()

	createContact(w, firstShape(w, groundId), firstShape(w, idA))
	ground := &w.contacts[len(w.contacts)-1]

	wakeSolverSet(w, sleepIndex)

	if bodyA.setIndex != awakeSet || bodyB.setIndex != awakeSet {
		t.Fatalf("the bodies are in sets %d and %d, want the awake set", bodyA.setIndex, bodyB.setIndex)
	}
	if bodyA.sleepTime != (Q{}) {
		t.Errorf("the sleep time is %v, want zero", bodyA.sleepTime)
	}
	if getBodyState(w, bodyA) == nil {
		t.Errorf("the woken body has no state")
	}
	if w.solverSets[sleepIndex].setIndex != nullIndex {
		t.Errorf("the sleeping set survives the wake")
	}
	awake := &w.solverSets[awakeSet]
	if len(awake.islandSims) != 1 || w.islands[bodyA.islandId].setIndex != awakeSet {
		t.Errorf("the island did not return to the awake set")
	}
	touching := &w.contacts[w.constraintGraph.colors[0].contactSims[0].contactId]
	if touching.colorIndex != 0 || touching.setIndex != awakeSet {
		t.Errorf("the touching contact has color %d in set %d, want color 0 in the awake set", touching.colorIndex, touching.setIndex)
	}
	if ground.setIndex != awakeSet || len(w.solverSets[disabledSet].contactSims) != 0 {
		t.Errorf("the non-touching contact is in set %d, want the awake set", ground.setIndex)
	}
	validateSolverSets(w)
}

// TestDestroyTouchingContactWakesTheSet pins the wake branch of
// destroyContact and wakeBody: a sleeping set wakes when a touching
// contact of one of its bodies goes away with wakeBodies set.
func TestDestroyTouchingContactWakesTheSet(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)

	idA, _, sleepIndex := sleepPair(t, w, worldId)
	bodyA := getBodyFullId(w, idA)

	c := &w.contacts[w.solverSets[sleepIndex].contactSims[0].contactId]
	destroyContact(w, c, true)

	if bodyA.setIndex != awakeSet {
		t.Fatalf("body A is in set %d, want the awake set", bodyA.setIndex)
	}
	if w.solverSets[sleepIndex].setIndex != nullIndex {
		t.Errorf("the sleeping set survives")
	}
	if wakeBody(w, bodyA) {
		t.Errorf("wakeBody reports a wake on an awake body")
	}
	validateSolverSets(w)
}

// TestMergeSolverSetsMovesTheSmallerSet pins mergeSolverSets: the set
// with fewer bodies moves into the other, and the freed slot returns to
// the pool.
func TestMergeSolverSetsMovesTheSmallerSet(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)

	idA, _, pairIndex := sleepPair(t, w, worldId)

	soloId := addDynamicCircle(t, worldId, v2(5, 0))
	solo := getBodyFullId(w, soloId)
	trySleepIsland(w, solo.islandId)
	soloIndex := solo.setIndex

	mergeSolverSets(w, soloIndex, pairIndex)

	if solo.setIndex != pairIndex {
		t.Fatalf("the solo body is in set %d, want %d", solo.setIndex, pairIndex)
	}
	if w.solverSets[soloIndex].setIndex != nullIndex {
		t.Errorf("the smaller set survives the merge")
	}
	merged := &w.solverSets[pairIndex]
	if len(merged.bodySims) != 3 || len(merged.islandSims) != 2 {
		t.Errorf("the merged set holds %d bodies and %d islands, want 3 and 2", len(merged.bodySims), len(merged.islandSims))
	}
	if w.islands[solo.islandId].localIndex != 1 || getBodyFullId(w, idA).setIndex != pairIndex {
		t.Errorf("the merge broke an index")
	}
	validateSolverSets(w)
}

// TestJointSleepsAndWakesWithTheIsland pins the joint sections of
// trySleepIsland and wakeSolverSet: the joint sim leaves its color for the
// sleeping set, the color frees the body bits, and the wake returns the
// sim to a color.
func TestJointSleepsAndWakesWithTheIsland(t *testing.T) {
	worldDef := DefaultWorldDef()
	worldDef.Gravity = Vec2Zero()
	worldId := CreateWorld(&worldDef)
	t.Cleanup(func() { DestroyWorld(worldId) })
	w := getWorldFromId(worldId)

	idA := addDynamicCircle(t, worldId, v2(0, 0))
	idB := addDynamicCircle(t, worldId, v2(3, 0))
	def := DefaultRevoluteJointDef()
	def.BodyIdA, def.BodyIdB = idA, idB
	jointId := CreateRevoluteJoint(worldId, &def)
	j := getJointFullId(w, jointId)
	bodyA := getBodyFullId(w, idA)
	colorIndex := j.colorIndex

	// The bodies rest; the island sleeps after timeToSleep.
	dt := stepDt()
	for range 40 {
		Step(worldId, dt, 4)
	}
	sleepIndex := bodyA.setIndex
	if sleepIndex < firstSleepingSet {
		t.Fatalf("the island is in set %d, want a sleeping set", sleepIndex)
	}
	sleepSet := &w.solverSets[sleepIndex]
	if j.setIndex != sleepIndex || j.colorIndex != nullIndex || j.localIndex != 0 {
		t.Errorf("the joint is in set %d color %d at %d", j.setIndex, j.colorIndex, j.localIndex)
	}
	if len(sleepSet.jointSims) != 1 || sleepSet.jointSims[0].jointId != j.jointId {
		t.Errorf("the sleeping set holds %d joints", len(sleepSet.jointSims))
	}
	color := &w.constraintGraph.colors[colorIndex]
	if len(color.jointSims) != 0 || color.bodySet.getBit(bodyA.id) {
		t.Errorf("color %d keeps the sleeping joint", colorIndex)
	}
	validateWorld(w)

	wakeSolverSet(w, sleepIndex)
	if j.setIndex != awakeSet || j.colorIndex == nullIndex {
		t.Errorf("the woken joint is in set %d color %d", j.setIndex, j.colorIndex)
	}
	js := getJointSim(w, j)
	if js.jointId != j.jointId || js.jointType != RevoluteJoint {
		t.Errorf("the woken sim has id %d and type %d", js.jointId, js.jointType)
	}
	validateWorld(w)
}

// TestJointBetweenSleepingSetsMergesThem pins the joint section of
// mergeSolverSets: a joint between two sleeping islands moves one set into
// the other, and the joint sim lands in the merged set.
func TestJointBetweenSleepingSetsMergesThem(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)

	idA1, _, set1 := sleepPair(t, w, worldId)
	groundId := addStaticCircle(t, worldId, v2(0, -3))
	holdDef := DefaultRevoluteJointDef()
	holdDef.BodyIdA, holdDef.BodyIdB = groundId, idA1
	holdId := CreateRevoluteJoint(worldId, &holdDef)

	idA2 := addDynamicCircle(t, worldId, v2(5, 0))
	bodyA2 := getBodyFullId(w, idA2)
	trySleepIsland(w, bodyA2.islandId)
	set2 := bodyA2.setIndex

	def := DefaultDistanceJointDef()
	def.BodyIdA, def.BodyIdB = idA1, idA2
	jointId := CreateDistanceJoint(worldId, &def)

	j := getJointFullId(w, jointId)
	hold := getJointFullId(w, holdId)
	bodyA1 := getBodyFullId(w, idA1)
	if bodyA1.setIndex != bodyA2.setIndex || bodyA1.setIndex < firstSleepingSet {
		t.Fatalf("the bodies are in sets %d and %d", bodyA1.setIndex, bodyA2.setIndex)
	}
	merged := &w.solverSets[bodyA1.setIndex]
	if j.setIndex != bodyA1.setIndex || hold.setIndex != bodyA1.setIndex {
		t.Errorf("the joints are in sets %d and %d, want %d", j.setIndex, hold.setIndex, bodyA1.setIndex)
	}
	if len(merged.jointSims) != 2 || merged.jointSims[j.localIndex].jointId != j.jointId || merged.jointSims[hold.localIndex].jointId != hold.jointId {
		t.Errorf("the merged set holds %d joints with broken indices", len(merged.jointSims))
	}
	if w.solverSets[set1].setIndex == set1 && w.solverSets[set2].setIndex == set2 {
		t.Errorf("both sets survive the merge")
	}
	validateWorld(w)
}

// TestTransferJointMovesBetweenSets pins transferJoint in both directions:
// from a color into a set and back into a color.
func TestTransferJointMovesBetweenSets(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)

	_, _, sleepIndex := sleepPair(t, w, worldId)
	idA := addDynamicCircle(t, worldId, v2(5, 0))
	idB := addDynamicCircle(t, worldId, v2(8, 0))
	def := DefaultWeldJointDef()
	def.BodyIdA, def.BodyIdB = idA, idB
	jointId := CreateWeldJoint(worldId, &def)
	j := getJointFullId(w, jointId)
	colorIndex := j.colorIndex

	awake := &w.solverSets[awakeSet]
	sleepSet := &w.solverSets[sleepIndex]
	transferJoint(w, sleepSet, awake, j)
	if j.setIndex != sleepIndex || j.colorIndex != nullIndex || j.localIndex != 0 {
		t.Fatalf("the joint is in set %d color %d at %d", j.setIndex, j.colorIndex, j.localIndex)
	}
	if len(sleepSet.jointSims) != 1 || len(w.constraintGraph.colors[colorIndex].jointSims) != 0 {
		t.Errorf("the sim did not move")
	}

	transferJoint(w, awake, sleepSet, j)
	if j.setIndex != awakeSet || j.colorIndex == nullIndex || len(sleepSet.jointSims) != 0 {
		t.Fatalf("the joint is in set %d color %d", j.setIndex, j.colorIndex)
	}
	if getJointSim(w, j).jointId != j.jointId {
		t.Errorf("the color sim has another id")
	}
	validateWorld(w)
}
