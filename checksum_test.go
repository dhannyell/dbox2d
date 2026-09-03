package dbox2d

import (
	"testing"

	"github.com/dhannyell/fixed"
)

// TestChecksumIsOrderIndependent pins the commutative fold: two worlds with
// the same bodies, created in opposite orders, share one checksum before and
// after stepping.
func TestChecksumIsOrderIndependent(t *testing.T) {
	positions := []Vec2{v2(0, 10), v2(5, 10), v2(10, 10)}

	build := func(reversed bool) WorldId {
		worldId := createTestWorld(t)
		for i := range positions {
			p := positions[i]
			if reversed {
				p = positions[len(positions)-1-i]
			}
			addDynamicBox(t, worldId, p)
		}
		return worldId
	}

	world1 := build(false)
	world2 := build(true)

	if Checksum(world1) != Checksum(world2) {
		t.Fatalf("checksums differ before stepping: %d and %d", Checksum(world1), Checksum(world2))
	}

	dt := stepDt()
	for range 60 {
		world1.Step(dt, 4)
		world2.Step(dt, 4)
	}
	if Checksum(world1) != Checksum(world2) {
		t.Errorf("checksums differ after stepping: %d and %d", Checksum(world1), Checksum(world2))
	}
}

// TestChecksumContactsIgnoreCreationOrder checks the contact graph as well as
// the body and shape folds. The second world reverses both object creation and
// contact orientation, but represents the same physical state.
func TestChecksumContactsIgnoreCreationOrder(t *testing.T) {
	positions := [3]Vec2{v2(0, 0), {X: fixed.Q32MustParse("0.75")}, v2(4, 0)}
	build := func(order [3]int, reverseContact bool) WorldId {
		worldId := createTestWorld(t)
		var bodies [3]BodyId
		for _, index := range order {
			bodies[index] = addDynamicCircle(t, worldId, positions[index])
		}

		w := getWorldFromId(worldId)
		shapeA := firstShape(w, bodies[0])
		shapeB := firstShape(w, bodies[1])
		if reverseContact {
			shapeA, shapeB = shapeB, shapeA
		}
		createContact(w, shapeA, shapeB)
		c := &w.contacts[0]
		cs := getContactSim(w, c)
		xfA := getBodyTransformQuick(w, &w.bodies[shapeA.bodyId])
		xfB := getBodyTransformQuick(w, &w.bodies[shapeB.bodyId])
		updateContact(w, cs, shapeA, xfA, Vec2Zero(), shapeB, xfB, Vec2Zero())
		return worldId
	}

	world1 := build([3]int{0, 1, 2}, false)
	world2 := build([3]int{2, 1, 0}, true)
	if got, want := Checksum(world2), Checksum(world1); got != want {
		t.Fatalf("contact checksum after reversed creation = %d, want %d", got, want)
	}
}

// TestChecksumSeesContactState checks that adding and updating a contact, and
// then changing a stored impulse, each changes the witness.
func TestChecksumSeesContactState(t *testing.T) {
	worldId := createTestWorld(t)
	idA := addDynamicCircle(t, worldId, v2(0, 0))
	idB := addDynamicCircle(t, worldId, Vec2{X: fixed.Q32MustParse("0.75")})
	w := getWorldFromId(worldId)
	shapeA := firstShape(w, idA)
	shapeB := firstShape(w, idB)

	withoutContact := Checksum(worldId)
	createContact(w, shapeA, shapeB)
	withContact := Checksum(worldId)
	if withContact == withoutContact {
		t.Fatal("adding a contact did not change the checksum")
	}

	c := &w.contacts[0]
	cs := getContactSim(w, c)
	xfA := getBodyTransformQuick(w, getBodyFullId(w, idA))
	xfB := getBodyTransformQuick(w, getBodyFullId(w, idB))
	updateContact(w, cs, shapeA, xfA, Vec2Zero(), shapeB, xfB, Vec2Zero())
	withManifold := Checksum(worldId)
	if withManifold == withContact {
		t.Fatal("updating the contact manifold did not change the checksum")
	}

	cs.manifold.Points[0].NormalImpulse = fixed.Q32One()
	if Checksum(worldId) == withManifold {
		t.Fatal("a stored contact impulse did not change the checksum")
	}
}

// TestChecksumSeesAStateChange pins sensitivity: a moved body changes the
// checksum, and a sleeping body still contributes its transform.
func TestChecksumSeesAStateChange(t *testing.T) {
	worldId := createTestWorld(t)
	bodyId := addDynamicBox(t, worldId, v2(0, 10))
	before := Checksum(worldId)

	w := getWorldFromId(worldId)
	b := getBodyFullId(w, bodyId)
	getBodyState(w, b).linearVelocity = Vec2{X: fixed.Q32One()}
	if Checksum(worldId) == before {
		t.Errorf("a velocity change did not change the checksum")
	}
	afterVelocity := Checksum(worldId)

	// A body outside the awake set still contributes its complete body state.
	sleeperDef := DefaultBodyDef()
	sleeperDef.Type = DynamicBody
	sleeperDef.IsAwake = false
	sleeperDef.Position = v2(20, 0)
	CreateBody(worldId, &sleeperDef)
	if Checksum(worldId) == afterVelocity {
		t.Errorf("a new sleeping body did not change the checksum")
	}
}

// TestChecksumSeesFutureBehaviour distinguishes worlds that look the same
// now but will integrate differently on the next step.
func TestChecksumSeesFutureBehaviour(t *testing.T) {
	build := func(gravity Vec2, damping Q) WorldId {
		def := DefaultWorldDef()
		def.Gravity = gravity
		worldId := CreateWorld(&def)
		t.Cleanup(func() { DestroyWorld(worldId) })

		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody
		bodyDef.LinearDamping = damping
		bodyId := CreateBody(worldId, &bodyDef)
		shapeDef := DefaultShapeDef()
		box := MakeSquare(fixed.Q32One())
		CreatePolygonShape(bodyId, &shapeDef, &box)
		return worldId
	}

	base := build(v2(0, -10), fixed.Q32Zero())
	differentGravity := build(v2(0, -9), fixed.Q32Zero())
	differentDamping := build(v2(0, -10), fixed.Q32One())
	if Checksum(base) == Checksum(differentGravity) {
		t.Errorf("world gravity did not change the checksum")
	}
	if Checksum(base) == Checksum(differentDamping) {
		t.Errorf("body damping did not change the checksum")
	}
}

// TestChecksumSeesAPendingSplit pins the island state that the bodies and
// contacts do not show: an island with a removed constraint splits on the
// next step and cannot sleep, so two worlds with the same visible state
// must not share a checksum.
func TestChecksumSeesAPendingSplit(t *testing.T) {
	build := func(chain bool) (WorldId, BodyId) {
		worldId := createTestWorld(t)
		w := getWorldFromId(worldId)
		idA := addDynamicCircle(t, worldId, v2(0, 0))
		idB := addDynamicCircle(t, worldId, v2(1, 0))
		idC := addDynamicCircle(t, worldId, v2(2, 0))
		startTouching(t, w, idA, idB)
		if chain {
			c := startTouching(t, w, idB, idC)
			mergeAwakeIslands(w)
			destroyContact(w, c, false)
		}
		return worldId, idA
	}

	pending, idA := build(true)
	settled, _ := build(false)
	w := getWorldFromId(pending)
	if w.islands[getBodyFullId(w, idA).islandId].constraintRemoveCount == 0 {
		t.Fatalf("the chain world has no pending split")
	}
	if Checksum(pending) == Checksum(settled) {
		t.Errorf("a pending island split did not change the checksum")
	}
}

// TestChecksumMatchesDeterministicWitness pins one value across processes and
// architectures. Every CI target must produce the same integer.
func TestChecksumMatchesDeterministicWitness(t *testing.T) {
	worldId := createTestWorld(t)
	var bodies [5]BodyId
	for i := range 5 {
		position := v2(i*3, i)
		if i == 1 {
			position = v2(1, 0)
		}
		id := addDynamicBox(t, worldId, position)
		bodies[i] = id
		w := getWorldFromId(worldId)
		b := getBodyFullId(w, id)
		getBodyState(w, b).angularVelocity = fixed.Q32MustParse("0.1")
	}

	// Only the boxes 0 and 1 overlap, and the broadphase pairs them on the
	// first step.
	dt := stepDt()
	for range 120 {
		worldId.Step(dt, 4)
	}

	// Rebased when the joint count and sum entered the hash.
	const want uint64 = 4734897736241209759
	if got := Checksum(worldId); got != want {
		t.Errorf("checksum = %d, want %d", got, want)
	}
}

// TestChecksumIgnoresTheTreeTopology pins D-013: a pyramid whose trees
// are rebuilt from scratch before the first step yields the checksum of
// the pyramid whose trees grew by insertion, because the pair order does
// not depend on the tree.
func TestChecksumIgnoresTheTreeTopology(t *testing.T) {
	run := func(rebuild bool) uint64 {
		worldId := createTestWorld(t)
		w := getWorldFromId(worldId)
		buildPyramid(worldId, 6)
		if rebuild {
			for i := range BodyTypeCount {
				w.broadPhase.trees[i].rebuild(true)
				w.broadPhase.trees[i].validate()
			}
		}
		dt := stepDt()
		for range 60 {
			worldId.Step(dt, 4)
		}
		validateWorld(w)
		return Checksum(worldId)
	}

	grown := run(false)
	rebuilt := run(true)
	if grown != rebuilt {
		t.Errorf("the rebuilt trees give %d, the grown trees %d", rebuilt, grown)
	}
}
