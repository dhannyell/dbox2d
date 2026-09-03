package dbox2d

import (
	"math/rand"
	"testing"

	"github.com/dhannyell/fixed"
)

// TestBroadPhaseBuffersTheMovedProxies pins the move buffer: a dynamic
// proxy joins it, a static proxy only when forced, a destroy leaves it,
// and the key packs the type below the id.
func TestBroadPhaseBuffersTheMovedProxies(t *testing.T) {
	var bp broadPhase
	createBroadPhase(&bp)

	dyn := bp.createProxy(DynamicBody, box(0, 0, 1, 1), DefaultCategoryBits, 7, false)
	if proxyTypeOf(dyn) != DynamicBody || proxyIdOf(dyn) != 0 || bp.getShapeIndex(dyn) != 7 {
		t.Errorf("the dynamic key %d does not unpack", dyn)
	}

	static := bp.createProxy(StaticBody, box(0, 0, 1, 1), DefaultCategoryBits, 8, false)
	forced := bp.createProxy(StaticBody, box(5, 5, 6, 6), DefaultCategoryBits, 9, true)
	if len(bp.moveArray) != 2 || bp.moveArray[0] != dyn || bp.moveArray[1] != forced {
		t.Fatalf("the move array is %v, want the dynamic and the forced key", bp.moveArray)
	}
	if !bp.testOverlap(dyn, static) || bp.testOverlap(dyn, forced) {
		t.Errorf("testOverlap does not follow the boxes")
	}

	bp.destroyProxy(dyn)
	if len(bp.moveArray) != 1 || bp.moveArray[0] != forced || bp.moveSet.count != 1 {
		t.Errorf("the destroy left the move array as %v", bp.moveArray)
	}

	bp.moveProxy(static, box(2, 2, 3, 3))
	if len(bp.moveArray) != 2 || bp.moveArray[1] != static {
		t.Errorf("the move did not buffer the static key")
	}
	bp.moveProxy(static, box(3, 3, 4, 4))
	if len(bp.moveArray) != 2 {
		t.Errorf("a second move buffered the key twice")
	}

	requirePanic(t, func() { bp.enlargeProxy(static, box(0, 0, 9, 9)) })

	destroyBroadPhase(&bp)
}

// TestBroadPhasePairsFollowTheRules pins pairQueryCallback: overlapping
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
	circle := Circle{Radius: fixed.Q32Half()}
	CreateCircleShape(a, &shapeDef, &circle)

	// A shape that no mask accepts.
	shapeDef.Filter.MaskBits = 0
	c := addDynamicCircle(t, worldId, v2(0, 0))
	sc := firstShape(w, c)
	sc.filter.MaskBits = 0

	// Every shape owns a proxy from its creation and sits in the move
	// buffer: the default definition invokes contact creation, so the
	// static shapes join the dynamic ones.
	if len(w.broadPhase.moveArray) != 6 {
		t.Fatalf("the move buffer holds %d proxies, want 6", len(w.broadPhase.moveArray))
	}
	_, _, _ = b, groundA, groundB

	updateBroadPhasePairs(w)
	validateWorld(w)
	w.broadPhase.validate()
	w.broadPhase.validateNoEnlarged()

	// a-b, a-groundA, a-groundB, b-groundA, b-groundB, second-b,
	// second-groundA, second-groundB.
	if got := w.contactIdPool.idCount(); got != 8 {
		t.Fatalf("the update created %d contacts, want 8", got)
	}
	if len(w.broadPhase.moveArray) != 0 || w.broadPhase.moveSet.count != 0 {
		t.Errorf("the update left the move buffer full")
	}

	for i := range 8 {
		ct := &w.contacts[i]
		if w.shapes[ct.shapeIdA].bodyId == w.shapes[ct.shapeIdB].bodyId {
			t.Errorf("contact %d pairs two shapes of one body", i)
		}
	}

	// A second update with every proxy buffered again creates nothing new.
	for i := range w.shapes {
		if w.shapes[i].proxyKey != nullIndex {
			w.broadPhase.bufferMove(w.shapes[i].proxyKey)
		}
	}
	updateBroadPhasePairs(w)
	if got := w.contactIdPool.idCount(); got != 8 {
		t.Errorf("the second update raised the contact count to %d", got)
	}
}

// TestBroadPhasePairsAreSortedByShapeId pins D-013: the contacts of one
// moved proxy come out in ascending shape pair order, and a static tree
// rebuilt into another topology yields the same sequence.
func TestBroadPhasePairsAreSortedByShapeId(t *testing.T) {
	sequence := func(rebuild bool) [][2]int {
		worldId := createTestWorld(t)
		w := getWorldFromId(worldId)

		mover := addDynamicBox(t, worldId, v2(0, 0))
		statics := make([]BodyId, 0, 7)
		for _, k := range []int{5, 2, 6, 0, 3, 1, 4} {
			// Every circle overlaps the box; the spread only changes the tree.
			position := Vec2{X: fixed.Q32FromInt(k).Div(fixed.Q32FromInt(16))}
			statics = append(statics, addStaticCircle(t, worldId, position))
		}

		_, _ = mover, statics
		if rebuild {
			w.broadPhase.trees[StaticBody].rebuild(true)
			w.broadPhase.trees[StaticBody].validate()
			w.broadPhase.rebuildTrees()
			w.broadPhase.validate()
		}

		updateBroadPhasePairs(w)

		// createContact may flip a pair to its primary register, so the
		// pair is rebuilt as the broadphase saw it: the smaller proxy key
		// is A.
		count := w.contactIdPool.idCount()
		pairs := make([][2]int, 0, count)
		for i := range count {
			ct := &w.contacts[i]
			pair := [2]int{ct.shapeIdA, ct.shapeIdB}
			if w.shapes[pair[1]].proxyKey < w.shapes[pair[0]].proxyKey {
				pair[0], pair[1] = pair[1], pair[0]
			}
			pairs = append(pairs, pair)
		}
		return pairs
	}

	plain := sequence(false)
	if len(plain) != 7 {
		t.Fatalf("the update created %d contacts, want 7", len(plain))
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
	circle := Circle{Radius: fixed.Q32Half()}
	for i := range count {
		bodyDef := DefaultBodyDef()
		bodyDef.Type = types[i%3]
		bodyDef.Position = Vec2{X: fixed.Q32FromRatio(rng.Intn(80), 4), Y: fixed.Q32FromRatio(rng.Intn(80), 4)}
		bodyId := CreateBody(worldId, &bodyDef)
		shapeDef := DefaultShapeDef()
		shapeDef.Filter.CategoryBits = 1 << uint(rng.Intn(3))
		shapeDef.Filter.MaskBits = uint64(rng.Intn(8))
		CreateCircleShape(bodyId, &shapeDef, &circle)
	}

	updateBroadPhasePairs(w)
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
