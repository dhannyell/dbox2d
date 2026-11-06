package b2

import (
	"math/rand"
	"slices"
	"testing"
)

// TestBroadPhaseMarksTheMovedProxies pins the moved marks: a dynamic
// proxy is marked on creation, a static proxy only when forced, a move
// marks the proxy, and the key packs the type below the id.
func TestBroadPhaseMarksTheMovedProxies(t *testing.T) {
	var bp broadPhase
	createBroadPhase(&bp)

	dyn := bp.createProxy(DynamicBody, box(0, 0, 1, 1), DefaultCategoryBits, 7, false)
	if proxyTypeOf(dyn) != DynamicBody || proxyIdOf(dyn) != 0 || bp.getShapeIndex(dyn) != 7 {
		t.Errorf("the dynamic key %d does not unpack", dyn)
	}
	if !bp.trees[DynamicBody].hasMoved() {
		t.Errorf("a new dynamic proxy is not marked moved")
	}

	static := bp.createProxy(StaticBody, box(0, 0, 1, 1), DefaultCategoryBits, 8, false)
	if bp.trees[StaticBody].hasMoved() {
		t.Errorf("an unforced static proxy is marked moved")
	}
	forced := bp.createProxy(StaticBody, box(5, 5, 6, 6), DefaultCategoryBits, 9, true)
	if !bp.trees[StaticBody].hasMoved() {
		t.Errorf("a forced static proxy is not marked moved")
	}
	moved := bp.trees[StaticBody].gatherMovedProxies(nil)
	if len(moved) != 1 || moved[0] != proxyIdOf(forced) {
		t.Errorf("the static tree reports %v moved, want the forced proxy", moved)
	}

	bp.trees[StaticBody].clearMoved()
	bp.moveProxy(static, box(2, 2, 3, 3))
	moved = bp.trees[StaticBody].gatherMovedProxies(nil)
	if len(moved) != 1 || moved[0] != proxyIdOf(static) {
		t.Errorf("the move marked %v, want the static proxy", moved)
	}
	if bp.trees[StaticBody].getProxyBounds(proxyIdOf(static)) != box(2, 2, 3, 3) {
		t.Errorf("the move did not set the box")
	}

	bp.trees[StaticBody].clearMoved()
	bp.markProxyMovedSerial(forced)
	moved = bp.trees[StaticBody].gatherMovedProxies(nil)
	if len(moved) != 1 || moved[0] != proxyIdOf(forced) {
		t.Errorf("the serial mark marked %v, want the forced proxy", moved)
	}

	bp.destroyProxy(dyn)
	if bp.trees[DynamicBody].getProxyCount() != 0 || bp.trees[DynamicBody].hasMoved() {
		t.Errorf("the destroy left the dynamic tree marked")
	}
	bp.trees[StaticBody].validate()

	destroyBroadPhase(&bp)
}

// updatePairs runs the pair update and joins the tree rebuild it starts.
func updatePairs(w *world) {
	updateBroadPhasePairs(w, &w.solverContext)
	w.treeTask.wait()
}

// TestBroadPhasePairsFollowTheRules pins the candidate filter: overlapping
// shapes gain one contact each, two static shapes none, a rejecting
// filter none, two shapes of one body none, and an existing pair is not
// created twice.
func TestBroadPhasePairsFollowTheRules(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)

	a := addDynamicCircle(t, worldId, v2(0, 0))
	b := addDynamicCircle(t, worldId, v2(0, 0))
	groundA := addStaticCircle(t, worldId, v2(0, 0))
	groundB := addStaticCircle(t, worldId, v2(0, 0))

	// A second shape on body a must not pair with the first one.
	shapeDef := DefaultShapeDef()
	circle := Circle{Radius: QHalf()}
	CreateCircleShape(a, &shapeDef, &circle)

	// A shape that no mask accepts.
	shapeDef.Filter.MaskBits = 0
	c := addDynamicCircle(t, worldId, v2(0, 0))
	sc := firstShape(w, c)
	sc.filter.MaskBits = 0

	// Every shape is marked from its creation: the default definition
	// invokes contact creation, so the static shapes join the dynamic
	// ones.
	if got := len(w.broadPhase.trees[StaticBody].gatherMovedProxies(nil)); got != 2 {
		t.Fatalf("the static tree marks %d proxies, want 2", got)
	}
	if got := len(w.broadPhase.trees[DynamicBody].gatherMovedProxies(nil)); got != 4 {
		t.Fatalf("the dynamic tree marks %d proxies, want 4", got)
	}
	_, _, _ = b, groundA, groundB

	updatePairs(w)
	validateWorld(w)
	w.broadPhase.validate()
	w.broadPhase.validateNoMoved()

	// a-b, a-groundA, a-groundB, b-groundA, b-groundB, second-b,
	// second-groundA, second-groundB.
	if got := w.contactIdPool.idCount(); got != 8 {
		t.Fatalf("the update created %d contacts, want 8", got)
	}

	for i := range 8 {
		ct := &w.contacts[i]
		if w.shapes[ct.shapeIdA].bodyId == w.shapes[ct.shapeIdB].bodyId {
			t.Errorf("contact %d pairs two shapes of one body", i)
		}
	}

	// A second update with every proxy marked again creates nothing new.
	for i := range w.shapes {
		if w.shapes[i].proxyKey != nullIndex {
			w.broadPhase.markProxyMovedSerial(w.shapes[i].proxyKey)
		}
	}
	updatePairs(w)
	if got := w.contactIdPool.idCount(); got != 8 {
		t.Errorf("the second update raised the contact count to %d", got)
	}
}

// TestBroadPhasePairsAreSortedByShapeId pins the pair order: the update
// creates the contacts in ascending shape pair key order, and a static
// tree rebuilt into another topology yields the same sequence.
func TestBroadPhasePairsAreSortedByShapeId(t *testing.T) {
	sequence := func(rebuild bool) [][2]int {
		worldId := createTestWorld(t)
		w := getWorldFromId(worldId)

		addDynamicBox(t, worldId, v2(0, 0))
		addDynamicBox(t, worldId, v2(0, 0))
		for _, k := range []int{5, 2, 6, 0, 3, 1, 4} {
			// Every circle overlaps the boxes; the spread only changes the
			// tree.
			position := Vec2{X: QFromInt(k).Div(QFromInt(16))}
			addStaticCircle(t, worldId, position)
		}

		if rebuild {
			worldId.RebuildStaticTree()
			w.broadPhase.trees[StaticBody].validate()
			w.broadPhase.validate()
		}

		updatePairs(w)

		// createContact may flip a pair to its primary register, so the
		// pair is rebuilt as the broadphase saw it: the smaller shape id
		// is A.
		count := w.contactIdPool.idCount()
		pairs := make([][2]int, 0, count)
		for i := range count {
			ct := &w.contacts[i]
			pairs = append(pairs, [2]int{min(ct.shapeIdA, ct.shapeIdB), max(ct.shapeIdA, ct.shapeIdB)})
		}
		return pairs
	}

	plain := sequence(false)
	if len(plain) != 15 {
		t.Fatalf("the update created %d contacts, want 15", len(plain))
	}
	for i := 1; i < len(plain); i++ {
		p, q := plain[i-1], plain[i]
		if p[0] > q[0] || (p[0] == q[0] && p[1] >= q[1]) {
			t.Errorf("contact %d %v comes after %v", i, q, p)
		}
	}

	rebuilt := sequence(true)
	for i := range plain {
		if i >= len(rebuilt) || rebuilt[i] != plain[i] {
			t.Fatalf("the rebuilt tree gave %v, the incremental tree %v", rebuilt, plain)
		}
	}
}

// TestRadixSortKeysSorts compares radixSortKeys with a library sort over
// keys that share digits, as shape pair keys do.
func TestRadixSortKeysSorts(t *testing.T) {
	rng := rand.New(rand.NewSource(5))
	for _, count := range []int{1, 2, 3, 64, 1000} {
		keys := make([]uint64, count)
		for i := range keys {
			keys[i] = shapePairKey(rng.Intn(300), rng.Intn(300))
		}
		want := slices.Clone(keys)
		slices.Sort(want)
		radixSortKeys(keys, make([]uint64, count))
		if !slices.Equal(keys, want) {
			t.Fatalf("radix sort of %d keys differs from the library sort", count)
		}
	}
}

// TestShouldShapesCollideAppliesTheGroupRule pins the filter rules: a
// shared positive group always collides, a shared negative group never,
// and different groups fall back to the masks.
func TestShouldShapesCollideAppliesTheGroupRule(t *testing.T) {
	a := DefaultFilter()
	b := DefaultFilter()
	if !shouldShapesCollide(a, b) {
		t.Errorf("the default filters do not collide")
	}

	a.MaskBits = 0
	if shouldShapesCollide(a, b) || shouldShapesCollide(b, a) {
		t.Errorf("an empty mask still collides")
	}

	a.GroupIndex, b.GroupIndex = 3, 3
	if !shouldShapesCollide(a, b) {
		t.Errorf("a shared positive group does not override the mask")
	}
	a.MaskBits = DefaultMaskBits
	a.GroupIndex, b.GroupIndex = -3, -3
	if shouldShapesCollide(a, b) {
		t.Errorf("a shared negative group collides")
	}
	b.GroupIndex = -4
	if !shouldShapesCollide(a, b) {
		t.Errorf("different groups do not fall back to the masks")
	}

	query := DefaultQueryFilter()
	if !shouldQueryCollide(b, query) {
		t.Errorf("the default query filter rejects a default shape")
	}
	query.MaskBits = 2
	if shouldQueryCollide(b, query) {
		t.Errorf("the query mask does not reject the shape category")
	}
}

// TestShouldBodiesCollideRejectsTwoNonDynamicBodies pins the body rule:
// at least one body of a pair must be dynamic.
func TestShouldBodiesCollideRejectsTwoNonDynamicBodies(t *testing.T) {
	w := &world{}
	static := &body{bodyType: StaticBody, headJointKey: nullIndex}
	kinematic := &body{bodyType: KinematicBody, headJointKey: nullIndex}
	dynamic := &body{bodyType: DynamicBody, headJointKey: nullIndex}

	if shouldBodiesCollide(w, static, kinematic) || shouldBodiesCollide(w, static, static) {
		t.Errorf("two non-dynamic bodies collide")
	}
	if !shouldBodiesCollide(w, static, dynamic) || !shouldBodiesCollide(w, dynamic, kinematic) {
		t.Errorf("a dynamic body does not collide")
	}
}

// TestBroadPhasePairsMatchBruteForce compares updateBroadPhasePairs with
// the enumeration of a hundred circles over the three body types and
// random filters: one contact for every pair that the rules accept.
func TestBroadPhasePairsMatchBruteForce(t *testing.T) {
	rng := rand.New(rand.NewSource(2))
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)

	const count = 100
	types := [3]BodyType{DynamicBody, StaticBody, KinematicBody}
	circle := Circle{Radius: QHalf()}
	for i := range count {
		bodyDef := DefaultBodyDef()
		bodyDef.Type = types[i%3]
		bodyDef.Position = Vec2{X: QFromRatio(rng.Intn(80), 4), Y: QFromRatio(rng.Intn(80), 4)}
		bodyId := CreateBody(worldId, &bodyDef)
		shapeDef := DefaultShapeDef()
		shapeDef.Filter.CategoryBits = 1 << uint(rng.Intn(3))
		shapeDef.Filter.MaskBits = uint64(rng.Intn(8))
		CreateCircleShape(bodyId, &shapeDef, &circle)
	}

	updatePairs(w)
	validateWorld(w)
	w.broadPhase.validate()

	want := map[uint64]bool{}
	for i := range count {
		for j := i + 1; j < count; j++ {
			sa, sb := &w.shapes[i], &w.shapes[j]
			if !AABBOverlaps(sa.fatAABB, sb.fatAABB) || sa.bodyId == sb.bodyId {
				continue
			}
			if !shouldBodiesCollide(w, &w.bodies[sa.bodyId], &w.bodies[sb.bodyId]) {
				continue
			}
			if !shouldShapesCollide(sa.filter, sb.filter) {
				continue
			}
			want[shapePairKey(i, j)] = true
		}
	}
	if len(want) == 0 {
		t.Fatal("the scene has no pair to check")
	}

	got := map[uint64]bool{}
	for i := range w.contacts {
		ct := &w.contacts[i]
		if ct.setIndex == nullIndex {
			continue
		}
		key := shapePairKey(ct.shapeIdA, ct.shapeIdB)
		if got[key] {
			t.Fatalf("shapes %d and %d have two contacts", ct.shapeIdA, ct.shapeIdB)
		}
		got[key] = true
	}
	if len(got) != len(want) {
		t.Fatalf("the update created %d contacts, brute force wants %d", len(got), len(want))
	}
	for key := range want {
		if !got[key] {
			t.Fatalf("the update missed the pair with key %d", key)
		}
	}
}
