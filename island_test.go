package dbox2d

import "testing"

// linkTouching creates a contact between the first shapes of two bodies,
// marks it touching and links it into the island graph. The narrowphase
// sets the flag in a step; the test sets it by hand.
func linkTouching(t *testing.T, w *world, idA, idB BodyId) *contact {
	t.Helper()
	createContact(w, firstShape(w, idA), firstShape(w, idB))
	c := &w.contacts[len(w.contacts)-1]
	c.flags |= contactTouchingFlag
	linkContact(w, c)
	return c
}

// TestBodyOwnsAnIsland pins the island lifecycle of one body: a dynamic
// body is born in its own island, a static body has none, and the island
// dies with the body.
func TestBodyOwnsAnIsland(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)

	dynamicId := addDynamicCircle(t, worldId, v2(0, 0))
	staticDef := DefaultBodyDef()
	staticId := CreateBody(worldId, &staticDef)

	dynamic := getBodyFullId(w, dynamicId)
	static := getBodyFullId(w, staticId)
	if dynamic.islandId == nullIndex {
		t.Fatalf("the dynamic body has no island")
	}
	if static.islandId != nullIndex {
		t.Errorf("the static body has island %d", static.islandId)
	}

	isl := &w.islands[dynamic.islandId]
	if isl.bodyCount != 1 || isl.headBody != dynamic.id || isl.tailBody != dynamic.id {
		t.Errorf("the island lists %d bodies with head %d and tail %d", isl.bodyCount, isl.headBody, isl.tailBody)
	}
	validateIsland(w, dynamic.islandId)
	validateSolverSets(w)

	DestroyBody(dynamicId)
	if w.islandIdPool.idCount() != 0 {
		t.Errorf("%d islands survive the body", w.islandIdPool.idCount())
	}
	validateSolverSets(w)
}

// TestLinkContactJoinsIslands pins the union-find rule: the root of body A
// is always the parent, and the merge appends the child at the tail.
func TestLinkContactJoinsIslands(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)

	idA := addDynamicCircle(t, worldId, v2(0, 0))
	idB := addDynamicCircle(t, worldId, v2(1, 0))
	idC := addDynamicCircle(t, worldId, v2(2, 0))
	bodyA := getBodyFullId(w, idA)
	bodyB := getBodyFullId(w, idB)
	bodyC := getBodyFullId(w, idC)
	rootId := bodyA.islandId

	// B-C first, then A-B: the root of A must win over the root of B.
	cBC := linkTouching(t, w, idB, idC)
	if w.islands[bodyC.islandId].parentIsland != bodyB.islandId {
		t.Fatalf("the island of C has parent %d, want the island of B", w.islands[bodyC.islandId].parentIsland)
	}
	if cBC.islandId != bodyB.islandId {
		t.Errorf("the contact B-C sits in island %d, want the island of B", cBC.islandId)
	}

	cAB := linkTouching(t, w, idA, idB)
	if w.islands[bodyB.islandId].parentIsland != rootId {
		t.Fatalf("the island of B has parent %d, want the island of A", w.islands[bodyB.islandId].parentIsland)
	}
	if cAB.islandId != rootId {
		t.Errorf("the contact A-B sits in island %d, want the island of A", cAB.islandId)
	}

	mergeAwakeIslands(w)

	if w.islandIdPool.idCount() != 1 {
		t.Fatalf("%d islands survive the merge, want 1", w.islandIdPool.idCount())
	}
	for _, b := range []*body{bodyA, bodyB, bodyC} {
		if b.islandId != rootId {
			t.Errorf("body %d sits in island %d, want %d", b.id, b.islandId, rootId)
		}
	}
	isl := &w.islands[rootId]
	if isl.parentIsland != nullIndex {
		t.Errorf("the root island has parent %d", isl.parentIsland)
	}
	// The merge walks the awake islands in reverse, so C joins before B.
	if isl.headBody != bodyA.id || isl.tailBody != bodyB.id || isl.bodyCount != 3 {
		t.Errorf("the merged body list runs %d..%d over %d bodies, want A..B over 3", isl.headBody, isl.tailBody, isl.bodyCount)
	}
	if isl.contactCount != 2 || isl.headContact != cAB.contactId || isl.tailContact != cBC.contactId {
		t.Errorf("the merged contact list runs %d..%d over %d contacts, want A-B..B-C over 2", isl.headContact, isl.tailContact, isl.contactCount)
	}
	validateIsland(w, rootId)
	validateSolverSets(w)
}

// TestUnlinkAndSplitSeparateIslands pins the split: an unlinked contact
// marks the island, and the depth first search rebuilds one island per
// connected component.
func TestUnlinkAndSplitSeparateIslands(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)

	idA := addDynamicCircle(t, worldId, v2(0, 0))
	idB := addDynamicCircle(t, worldId, v2(1, 0))
	idC := addDynamicCircle(t, worldId, v2(2, 0))
	bodyA := getBodyFullId(w, idA)
	bodyB := getBodyFullId(w, idB)
	bodyC := getBodyFullId(w, idC)

	cAB := linkTouching(t, w, idA, idB)
	cBC := linkTouching(t, w, idB, idC)
	mergeAwakeIslands(w)
	baseId := bodyA.islandId

	// The narrowphase clears the flag before it unlinks; the split trusts
	// the flag.
	cBC.flags &^= contactTouchingFlag
	unlinkContact(w, cBC)
	isl := &w.islands[baseId]
	if isl.constraintRemoveCount != 1 || isl.contactCount != 1 {
		t.Fatalf("the unlink left %d removed and %d live contacts, want 1 and 1", isl.constraintRemoveCount, isl.contactCount)
	}
	if cBC.islandId != nullIndex {
		t.Errorf("the unlinked contact keeps island %d", cBC.islandId)
	}
	validateIsland(w, baseId)

	splitIsland(w, baseId)

	if w.islandIdPool.idCount() != 2 {
		t.Fatalf("%d islands survive the split, want 2", w.islandIdPool.idCount())
	}
	if bodyA.islandId != bodyB.islandId {
		t.Errorf("A and B split apart: islands %d and %d", bodyA.islandId, bodyB.islandId)
	}
	if bodyC.islandId == bodyA.islandId {
		t.Errorf("C stays in the island of A")
	}
	if cAB.islandId != bodyA.islandId {
		t.Errorf("the contact A-B sits in island %d, want %d", cAB.islandId, bodyA.islandId)
	}
	if w.islands[bodyC.islandId].contactCount != 0 {
		t.Errorf("the island of C has %d contacts", w.islands[bodyC.islandId].contactCount)
	}
	if w.arena.allocation != 0 {
		t.Errorf("the split leaves %d bytes in the arena", w.arena.allocation)
	}
	validateIsland(w, bodyA.islandId)
	validateIsland(w, bodyC.islandId)
	validateSolverSets(w)
}

// TestDestroyContactLeavesTheIsland pins the destroy path: a touching
// contact unlinks from its island before its sim goes away.
func TestDestroyContactLeavesTheIsland(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)

	idA := addDynamicCircle(t, worldId, v2(0, 0))
	idB := addDynamicCircle(t, worldId, v2(1, 0))
	bodyA := getBodyFullId(w, idA)

	c := startTouching(t, w, idA, idB)
	mergeAwakeIslands(w)
	islandId := bodyA.islandId

	destroyContact(w, c, false)

	isl := &w.islands[islandId]
	if isl.contactCount != 0 || isl.constraintRemoveCount != 1 {
		t.Errorf("the island keeps %d contacts with %d removed, want 0 and 1", isl.contactCount, isl.constraintRemoveCount)
	}
	validateIsland(w, islandId)
	validateSolverSets(w)
}
