package b2

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
	if len(isl.bodies) != 1 || isl.bodies[0] != dynamic.id || dynamic.islandIndex != 0 {
		t.Errorf("the island lists bodies %v, body island index %d", isl.bodies, dynamic.islandIndex)
	}
	validateIsland(w, dynamic.islandId)
	validateSolverSets(w)

	DestroyBody(dynamicId)
	if w.islandIdPool.idCount() != 0 {
		t.Errorf("%d islands survive the body", w.islandIdPool.idCount())
	}
	validateSolverSets(w)
}

// TestLinkContactJoinsIslands pins the merge rule: a link merges the
// islands at once, the island with more bodies survives (the island of
// body A on a tie), and the bodies of the smaller island append in order.
func TestLinkContactJoinsIslands(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)

	idA := addDynamicCircle(t, worldId, v2(0, 0))
	idB := addDynamicCircle(t, worldId, v2(1, 0))
	idC := addDynamicCircle(t, worldId, v2(2, 0))
	bodyA := getBodyFullId(w, idA)
	bodyB := getBodyFullId(w, idB)
	bodyC := getBodyFullId(w, idC)
	islandOfB := bodyB.islandId

	// B-C first: a tie keeps the island of B.
	cBC := linkTouching(t, w, idB, idC)
	if bodyC.islandId != islandOfB || cBC.islandId != islandOfB {
		t.Fatalf("C and B-C sit in islands %d and %d, want the island of B %d", bodyC.islandId, cBC.islandId, islandOfB)
	}

	// A-B next: the island of B is bigger and survives.
	cAB := linkTouching(t, w, idA, idB)
	if w.islandIdPool.idCount() != 1 {
		t.Fatalf("%d islands survive the merge, want 1", w.islandIdPool.idCount())
	}
	for _, b := range []*body{bodyA, bodyB, bodyC} {
		if b.islandId != islandOfB {
			t.Errorf("body %d sits in island %d, want %d", b.id, b.islandId, islandOfB)
		}
	}
	isl := &w.islands[islandOfB]
	if len(isl.bodies) != 3 || isl.bodies[0] != bodyB.id || isl.bodies[1] != bodyC.id || isl.bodies[2] != bodyA.id {
		t.Errorf("the merged island lists bodies %v, want B, C, A", isl.bodies)
	}
	if len(isl.contacts) != 2 || isl.contacts[0].contactId != cBC.contactId || isl.contacts[1].contactId != cAB.contactId {
		t.Errorf("the merged island lists contacts %v, want B-C, A-B", isl.contacts)
	}
	validateIsland(w, islandOfB)
	validateSolverSets(w)
}

// TestUnlinkAndSplitSeparateIslands pins the split: an unlinked contact
// marks the island, and the union-find pass rebuilds one island per
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
	baseId := bodyA.islandId

	cBC.flags &^= contactTouchingFlag
	unlinkContact(w, cBC)
	isl := &w.islands[baseId]
	if isl.constraintRemoveCount != 1 || len(isl.contacts) != 1 {
		t.Fatalf("the unlink left %d removed and %d live contacts, want 1 and 1", isl.constraintRemoveCount, len(isl.contacts))
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
	if n := len(w.islands[bodyC.islandId].contacts); n != 0 {
		t.Errorf("the island of C has %d contacts", n)
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
	islandId := bodyA.islandId

	destroyContact(w, c, false)

	isl := &w.islands[islandId]
	if len(isl.contacts) != 0 || isl.constraintRemoveCount != 1 {
		t.Errorf("the island keeps %d contacts with %d removed, want 0 and 1", len(isl.contacts), isl.constraintRemoveCount)
	}
	validateIsland(w, islandId)
	validateSolverSets(w)
}

// linkHandJoint builds a filter joint by hand between two bodies, wires it
// into the body lists and the graph, and links it into the island graph.
// The creation path lands with the joint API; the test needs only the
// bookkeeping.
func linkHandJoint(t *testing.T, w *world, idA, idB BodyId) *joint {
	t.Helper()
	bodyA := getBodyFullId(w, idA)
	bodyB := getBodyFullId(w, idB)

	jointId := w.jointIdPool.allocId()
	if jointId == len(w.joints) {
		w.joints = append(w.joints, joint{})
	}
	j := &w.joints[jointId]
	*j = joint{
		jointId:     jointId,
		setIndex:    awakeSet,
		colorIndex:  nullIndex,
		localIndex:  nullIndex,
		islandId:    nullIndex,
		islandIndex: nullIndex,
		jointType:   FilterJoint,
	}

	j.edges[0] = jointEdge{bodyId: bodyA.id, prevKey: nullIndex, nextKey: bodyA.headJointKey}
	keyA := jointId << 1
	if bodyA.headJointKey != nullIndex {
		head := &w.joints[bodyA.headJointKey>>1]
		head.edges[bodyA.headJointKey&1].prevKey = keyA
	}
	bodyA.headJointKey = keyA
	bodyA.jointCount += 1

	j.edges[1] = jointEdge{bodyId: bodyB.id, prevKey: nullIndex, nextKey: bodyB.headJointKey}
	keyB := jointId<<1 | 1
	if bodyB.headJointKey != nullIndex {
		head := &w.joints[bodyB.headJointKey>>1]
		head.edges[bodyB.headJointKey&1].prevKey = keyB
	}
	bodyB.headJointKey = keyB
	bodyB.jointCount += 1

	js := createJointInGraph(w, j)
	js.jointId = jointId
	js.bodyIdA = bodyA.id
	js.bodyIdB = bodyB.id
	js.jointType = FilterJoint

	linkJoint(w, j)
	return j
}

// TestLinkJointJoinsIslands pins the joint rule: a joint merges the
// islands of its bodies at once, and a joint to a static body stays in
// the island of the dynamic body.
func TestLinkJointJoinsIslands(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)

	idA := addDynamicCircle(t, worldId, v2(0, 0))
	idB := addDynamicCircle(t, worldId, v2(1, 0))
	groundDef := DefaultBodyDef()
	groundId := CreateBody(worldId, &groundDef)
	bodyA := getBodyFullId(w, idA)
	bodyB := getBodyFullId(w, idB)

	jAB := linkHandJoint(t, w, idA, idB)
	if bodyA.islandId != bodyB.islandId {
		t.Fatalf("A and B stay apart: islands %d and %d", bodyA.islandId, bodyB.islandId)
	}
	if jAB.islandId != bodyA.islandId {
		t.Errorf("the joint sits in island %d, want %d", jAB.islandId, bodyA.islandId)
	}

	jGA := linkHandJoint(t, w, groundId, idA)
	isl := &w.islands[bodyA.islandId]
	if len(isl.joints) != 2 || isl.joints[0].jointId != jAB.jointId || isl.joints[1].jointId != jGA.jointId {
		t.Errorf("the island lists joints %v, want A-B, ground-A", isl.joints)
	}
	if w.islandIdPool.idCount() != 1 {
		t.Errorf("%d islands survive the merge, want 1", w.islandIdPool.idCount())
	}
	validateIsland(w, bodyA.islandId)
	validateConnectivity(w)
	validateSolverSets(w)
}

// TestUnlinkJointSplitsTheIsland pins the split over joints: an unlinked
// joint counts as a removed constraint, and the search follows the joints
// that remain.
func TestUnlinkJointSplitsTheIsland(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)

	idA := addDynamicCircle(t, worldId, v2(0, 0))
	idB := addDynamicCircle(t, worldId, v2(1, 0))
	idC := addDynamicCircle(t, worldId, v2(2, 0))
	bodyA := getBodyFullId(w, idA)
	bodyB := getBodyFullId(w, idB)
	bodyC := getBodyFullId(w, idC)

	jAB := linkHandJoint(t, w, idA, idB)
	jBC := linkHandJoint(t, w, idB, idC)
	baseId := bodyA.islandId

	unlinkJoint(w, jBC)
	isl := &w.islands[baseId]
	if isl.constraintRemoveCount != 1 || len(isl.joints) != 1 {
		t.Fatalf("the unlink left %d removed and %d live joints, want 1 and 1", isl.constraintRemoveCount, len(isl.joints))
	}
	validateIsland(w, baseId)

	// The split reads the island links only; the joint still leaves the
	// body lists, as a destroyed joint does.
	removeHandJointEdges(w, jBC)

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
	if jAB.islandId != bodyA.islandId {
		t.Errorf("the joint A-B sits in island %d, want %d", jAB.islandId, bodyA.islandId)
	}
	validateIsland(w, bodyA.islandId)
	validateIsland(w, bodyC.islandId)
}

// removeHandJointEdges takes a hand-built joint out of the body lists,
// the graph and the pool.
func removeHandJointEdges(w *world, j *joint) {
	for edgeIndex := range 2 {
		edge := &j.edges[edgeIndex]
		b := &w.bodies[edge.bodyId]
		key := j.jointId<<1 | edgeIndex
		if edge.prevKey != nullIndex {
			prev := &w.joints[edge.prevKey>>1]
			prev.edges[edge.prevKey&1].nextKey = edge.nextKey
		}
		if edge.nextKey != nullIndex {
			next := &w.joints[edge.nextKey>>1]
			next.edges[edge.nextKey&1].prevKey = edge.prevKey
		}
		if b.headJointKey == key {
			b.headJointKey = edge.nextKey
		}
		b.jointCount -= 1
	}
	removeJointFromGraph(w, j.edges[0].bodyId, j.edges[1].bodyId, j.colorIndex, j.localIndex)
	w.jointIdPool.freeId(j.jointId)
	j.jointId = nullIndex
	j.setIndex = nullIndex
}
