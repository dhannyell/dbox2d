package dbox2d

import (
	"math/rand"
	"testing"

	"github.com/dhannyell/fixed"
)

// This file tests the world state as one unit: the handles, the id pools,
// the solver sets and the mass pipeline. The reference validates the same
// invariants with b2ValidateSolverSets in its validation builds.

// createTestWorld returns a world and destroys it on cleanup.
func createTestWorld(t *testing.T) WorldId {
	t.Helper()
	def := DefaultWorldDef()
	worldId := CreateWorld(&def)
	if worldId.IsNull() {
		t.Fatal("CreateWorld returned the null id")
	}
	t.Cleanup(func() {
		if worldId.IsValid() {
			DestroyWorld(worldId)
		}
	})
	return worldId
}

func v2(x, y int) Vec2 {
	return Vec2{X: fixed.Q32FromInt(x), Y: fixed.Q32FromInt(y)}
}

// TestIdReuseInvalidatesTheOldHandle pins the generation scheme: a destroyed
// id never validates again, even after its slot is reused.
func TestIdReuseInvalidatesTheOldHandle(t *testing.T) {
	t.Run("world", func(t *testing.T) {
		def := DefaultWorldDef()
		oldId := CreateWorld(&def)
		DestroyWorld(oldId)
		if oldId.IsValid() {
			t.Errorf("a destroyed WorldId is still valid")
		}

		newId := CreateWorld(&def)
		defer DestroyWorld(newId)
		if oldId.IsValid() {
			t.Errorf("the old WorldId became valid after slot reuse")
		}
		if !newId.IsValid() {
			t.Errorf("the new WorldId is not valid")
		}
	})

	t.Run("body", func(t *testing.T) {
		worldId := createTestWorld(t)

		bodyDef := DefaultBodyDef()
		oldId := CreateBody(worldId, &bodyDef)
		DestroyBody(oldId)
		if oldId.IsValid() {
			t.Errorf("a destroyed BodyId is still valid")
		}

		// The pool reuses the freed index, so only the generation differs.
		newId := CreateBody(worldId, &bodyDef)
		if newId.index1 != oldId.index1 {
			t.Fatalf("the new body has index %d, want the reused index %d", newId.index1, oldId.index1)
		}
		if oldId.IsValid() {
			t.Errorf("the old BodyId became valid after slot reuse")
		}
		if !newId.IsValid() {
			t.Errorf("the new BodyId is not valid")
		}

		validateSolverSets(getWorldFromId(worldId))
	})

	t.Run("shape", func(t *testing.T) {
		worldId := createTestWorld(t)

		bodyDef := DefaultBodyDef()
		bodyId := CreateBody(worldId, &bodyDef)

		shapeDef := DefaultShapeDef()
		circle := Circle{Radius: fixed.Q32One()}
		oldId := CreateCircleShape(bodyId, &shapeDef, &circle)
		DestroyShape(oldId, true)
		if oldId.IsValid() {
			t.Errorf("a destroyed ShapeId is still valid")
		}

		newId := CreateCircleShape(bodyId, &shapeDef, &circle)
		if newId.index1 != oldId.index1 {
			t.Fatalf("the new shape has index %d, want the reused index %d", newId.index1, oldId.index1)
		}
		if oldId.IsValid() {
			t.Errorf("the old ShapeId became valid after slot reuse")
		}
		if !newId.IsValid() {
			t.Errorf("the new ShapeId is not valid")
		}

		validateSolverSets(getWorldFromId(worldId))
	})
}

// TestSensorRejectionLeavesBodyUnchanged guards the unsupported sensor path.
// A recovered panic must not leave an unreachable shape attached to the body.
func TestSensorRejectionLeavesBodyUnchanged(t *testing.T) {
	worldId := createTestWorld(t)

	bodyDef := DefaultBodyDef()
	bodyId := CreateBody(worldId, &bodyDef)
	w := getWorldFromId(worldId)

	shapeDef := DefaultShapeDef()
	shapeDef.IsSensor = true
	circle := Circle{Radius: fixed.Q32One()}

	panicked := false
	func() {
		defer func() {
			panicked = recover() != nil
		}()
		CreateCircleShape(bodyId, &shapeDef, &circle)
	}()

	if !panicked {
		t.Fatal("creating a sensor shape did not panic")
	}
	if got := bodyId.ShapeCount(); got != 0 {
		t.Errorf("shape count after the panic = %d, want 0", got)
	}
	if got := w.shapeIdPool.idCount(); got != 0 {
		t.Errorf("allocated shape ids after the panic = %d, want 0", got)
	}
	if got := len(w.shapes); got != 0 {
		t.Errorf("shape slots after the panic = %d, want 0", got)
	}
	validateSolverSets(w)
}

// TestCreateAndDestroyOrdersProduceTheSameWorld builds the same survivors
// through two different creation and destruction orders. Every observable of
// each survivor must match, and both worlds must pass validation.
func TestCreateAndDestroyOrdersProduceTheSameWorld(t *testing.T) {
	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	shapeDef := DefaultShapeDef()
	box := MakeSquare(fixed.Q32One())

	addBody := func(worldId WorldId, position Vec2) BodyId {
		def := bodyDef
		def.Position = position
		bodyId := CreateBody(worldId, &def)
		CreatePolygonShape(bodyId, &shapeDef, &box)
		return bodyId
	}

	p1, p2 := v2(1, 2), v2(3, 4)
	temp := v2(9, 9)

	// World 1: the temporary body sits between the survivors.
	world1 := createTestWorld(t)
	a1 := addBody(world1, p1)
	doomed1 := addBody(world1, temp)
	b1 := addBody(world1, p2)
	DestroyBody(doomed1)

	// World 2: the temporary body comes last.
	world2 := createTestWorld(t)
	a2 := addBody(world2, p1)
	b2 := addBody(world2, p2)
	doomed2 := addBody(world2, temp)
	DestroyBody(doomed2)

	pairs := []struct {
		name   string
		s1, s2 BodyId
	}{
		{"first survivor", a1, a2},
		{"second survivor", b1, b2},
	}
	for _, pair := range pairs {
		if pair.s1.Position() != pair.s2.Position() {
			t.Errorf("%s: position %v, want %v", pair.name, pair.s2.Position(), pair.s1.Position())
		}
		if pair.s1.Rotation() != pair.s2.Rotation() {
			t.Errorf("%s: rotation %v, want %v", pair.name, pair.s2.Rotation(), pair.s1.Rotation())
		}
		if !pair.s1.Mass().Eq(pair.s2.Mass()) {
			t.Errorf("%s: mass %v, want %v", pair.name, pair.s2.Mass(), pair.s1.Mass())
		}
		if pair.s1.ShapeCount() != pair.s2.ShapeCount() {
			t.Errorf("%s: shape count %d, want %d", pair.name, pair.s2.ShapeCount(), pair.s1.ShapeCount())
		}
	}

	w1, w2 := getWorldFromId(world1), getWorldFromId(world2)
	if w1.bodyIdPool.idCount() != w2.bodyIdPool.idCount() {
		t.Errorf("body counts differ: %d and %d", w1.bodyIdPool.idCount(), w2.bodyIdPool.idCount())
	}
	validateSolverSets(w1)
	validateSolverSets(w2)

	// A handle rebuilt from the raw index equals the original handle.
	if makeBodyId(w1, int(a1.index1)-1) != a1 {
		t.Errorf("makeBodyId does not rebuild the original handle")
	}
}

// TestBodyMassComesFromItsShapes pins the world mass pipeline against the
// direct mass functions, the capsule weld and the velocity correction of a
// moving center of mass.
func TestBodyMassComesFromItsShapes(t *testing.T) {
	worldId := createTestWorld(t)

	// Half a turn per second, so the velocity correction is visible.
	omega := fixed.Q32Half()

	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = v2(5, 0)
	bodyDef.AngularVelocity = omega
	bodyId := CreateBody(worldId, &bodyDef)

	shapeDef := DefaultShapeDef()
	box := MakeSquare(fixed.Q32One())
	CreatePolygonShape(bodyId, &shapeDef, &box)

	wantBox := ComputePolygonMass(&box, shapeDef.Density)
	if !bodyId.Mass().Eq(wantBox.Mass) {
		t.Fatalf("mass with one box = %v, want %v", bodyId.Mass(), wantBox.Mass)
	}

	// Record the state before the second shape. The box centroid carries a
	// rounding residue, so the center is near the position, not equal.
	w := getWorldFromId(worldId)
	b := getBodyFullId(w, bodyId)
	centerBefore := getBodySim(w, b).center
	velocityBefore := getBodyState(w, b).linearVelocity

	// A degenerate capsule welds into a circle at its midpoint.
	capsule := Capsule{Center1: v2(2, 0), Center2: v2(2, 0), Radius: fixed.Q32One()}
	weldedId := CreateCapsuleShape(bodyId, &shapeDef, &capsule)

	welded := getShape(w, weldedId)
	if welded.shapeType != CircleShape {
		t.Fatalf("the welded capsule has type %d, want CircleShape", welded.shapeType)
	}

	wantCircle := ComputeCircleMass(&welded.circle, shapeDef.Density)
	wantTotal := wantBox.Mass.Add(wantCircle.Mass)
	if !bodyId.Mass().Eq(wantTotal) {
		t.Errorf("mass with two shapes = %v, want %v", bodyId.Mass(), wantTotal)
	}

	// The center of mass moved, so the linear velocity gains the cross of
	// the angular velocity, in radians per second, with the displacement.
	sim := getBodySim(w, b)
	state := getBodyState(w, b)
	wantVelocity := velocityBefore.Add(CrossSV(tau.Mul(omega), sim.center.Sub(centerBefore)))
	if !state.linearVelocity.X.Eq(wantVelocity.X) || !state.linearVelocity.Y.Eq(wantVelocity.Y) {
		t.Errorf("velocity after the mass update = %v, want %v", state.linearVelocity, wantVelocity)
	}

	// The stored inverse comes from one division, as in the reference.
	if !sim.invMass.Eq(fixed.Q32One().Div(b.mass)) {
		t.Errorf("invMass = %v, want 1 / mass", sim.invMass)
	}

	// Destroying the circle with a mass update restores the box mass.
	DestroyShape(weldedId, true)
	if !bodyId.Mass().Eq(wantBox.Mass) {
		t.Errorf("mass after the destroy = %v, want %v", bodyId.Mass(), wantBox.Mass)
	}
	if bodyId.ShapeCount() != 1 {
		t.Errorf("shape count after the destroy = %d, want 1", bodyId.ShapeCount())
	}

	validateSolverSets(w)
}

// TestSleepingBodyGetsItsOwnSolverSet pins the set lifecycle: a body born
// asleep receives a fresh sleeping set, a transfer wakes it, and the orphan
// set dies.
func TestSleepingBodyGetsItsOwnSolverSet(t *testing.T) {
	worldId := createTestWorld(t)

	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.IsAwake = false
	bodyId := CreateBody(worldId, &bodyDef)

	w := getWorldFromId(worldId)
	b := getBodyFullId(w, bodyId)
	if b.setIndex != firstSleepingSet {
		t.Fatalf("the sleeping body is in set %d, want %d", b.setIndex, firstSleepingSet)
	}
	if getBodyState(w, b) != nil {
		t.Errorf("a sleeping body has a solver state")
	}
	validateSolverSets(w)

	// Wake the body by hand: transfer it to the awake set, then destroy the
	// orphan set, as the wake operation of the reference does.
	sleepIndex := b.setIndex
	transferBody(w, &w.solverSets[awakeSet], &w.solverSets[sleepIndex], b)
	if b.setIndex != awakeSet {
		t.Fatalf("the transferred body is in set %d, want the awake set", b.setIndex)
	}
	if getBodyState(w, b) == nil {
		t.Errorf("an awake body has no solver state")
	}
	if len(w.solverSets[sleepIndex].bodySims) != 0 {
		t.Fatalf("the sleeping set still has sims")
	}

	// The island follows its body, as the wake operation does.
	isl := &w.islands[b.islandId]
	isl.setIndex = awakeSet
	isl.localIndex = len(w.solverSets[awakeSet].islandSims)
	w.solverSets[awakeSet].islandSims = append(w.solverSets[awakeSet].islandSims, w.solverSets[sleepIndex].islandSims[0])

	destroySolverSet(w, sleepIndex)
	validateSolverSets(w)

	// The next sleeping body reuses the freed set slot.
	secondId := CreateBody(worldId, &bodyDef)
	second := getBodyFullId(w, secondId)
	if second.setIndex != sleepIndex {
		t.Errorf("the second sleeper is in set %d, want the reused set %d", second.setIndex, sleepIndex)
	}
	validateSolverSets(w)
}

// TestOverlapAABBReportsTheFatBounds pins OverlapAABB: the query reports
// exactly the shapes whose fat bounds cross the box, across the three
// trees, a false return stops the walk of one tree, an empty mask hides every shape, and a
// locked world panics.
func TestOverlapAABBReportsTheFatBounds(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)

	addStaticCircle(t, worldId, v2(0, 0))
	kinematicDef := DefaultBodyDef()
	kinematicDef.Type = KinematicBody
	kinematicDef.Position = v2(5, 0)
	kinematic := CreateBody(worldId, &kinematicDef)
	shapeDef := DefaultShapeDef()
	circle := Circle{Radius: fixed.Q32Half()}
	CreateCircleShape(kinematic, &shapeDef, &circle)
	addDynamicBox(t, worldId, v2(10, 0))
	addDynamicBox(t, worldId, v2(10, 0))

	query := box(-1, -1, 11, 1)
	expected := map[ShapeId]bool{}
	for i := range w.shapes {
		s := &w.shapes[i]
		if s.id != nullIndex && AABBOverlaps(s.fatAABB, query) {
			expected[shapeIdOf(w, s)] = true
		}
	}
	if len(expected) != 4 {
		t.Fatalf("the fixture has %d shapes in the box, want 4", len(expected))
	}

	got := map[ShapeId]bool{}
	stats := OverlapAABB(worldId, query, DefaultQueryFilter(), func(id ShapeId) bool {
		got[id] = true
		return true
	})
	if len(got) != len(expected) || stats.LeafVisits < 3 {
		t.Fatalf("the query reported %d shapes with %d leaf visits", len(got), stats.LeafVisits)
	}
	for id := range expected {
		if !got[id] {
			t.Errorf("the query missed a shape in the box")
		}
	}

	// A false return ends the current tree; the next tree still runs, so
	// the two dynamic boxes yield one call and the trees three in total.
	calls := 0
	OverlapAABB(worldId, query, DefaultQueryFilter(), func(ShapeId) bool {
		calls++
		return false
	})
	if calls != 3 {
		t.Errorf("a false return did not stop the tree walk: %d calls", calls)
	}

	filter := DefaultQueryFilter()
	filter.MaskBits = 0
	OverlapAABB(worldId, query, filter, func(ShapeId) bool {
		t.Errorf("an empty mask reported a shape")
		return true
	})

	w.locked = true
	requirePanic(t, func() { OverlapAABB(worldId, query, DefaultQueryFilter(), func(ShapeId) bool { return true }) })
	w.locked = false
}

// TestCastRayClipsAcrossTheTrees pins CastRay: the static hit clips the
// ray before the dynamic tree, the closest hit wins, a -1 return skips
// the shape, an empty mask hides every shape, an origin inside a shape
// reports a zero fraction and stops, and a locked world panics.
func TestCastRayClipsAcrossTheTrees(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)

	near := shapeIdOf(w, firstShape(w, addDynamicBox(t, worldId, v2(0, 0))))
	far := shapeIdOf(w, firstShape(w, addDynamicBox(t, worldId, v2(4, 0))))
	static := shapeIdOf(w, firstShape(w, addStaticCircle(t, worldId, v2(8, 0))))

	origin := v2(-10, 0)
	translation := v2(30, 0)
	filter := DefaultQueryFilter()

	var hit ShapeId
	var point Vec2
	closest := func(id ShapeId, p, _ Vec2, fraction Q) Q {
		hit, point = id, p
		return fraction
	}
	stats := CastRay(worldId, origin, translation, filter, closest)
	if hit != near || stats.LeafVisits < 2 {
		t.Fatalf("the closest hit is not the near box (%d leaf visits)", stats.LeafVisits)
	}
	// The near box spans [-1, 1]; the hit sits on its left face.
	eps := fixed.Q32One().Div(fixed.Q32FromInt(1024))
	diff := point.X.Add(fixed.Q32One())
	if diff.Less(eps.Neg()) || eps.Less(diff) {
		t.Errorf("the hit point x is %v, want -1", point.X)
	}

	hit = ShapeId{}
	CastRay(worldId, origin, translation, filter, func(id ShapeId, p, n Vec2, fraction Q) Q {
		if id == near {
			return fixed.Q32One().Neg()
		}
		return closest(id, p, n, fraction)
	})
	if hit != far {
		t.Errorf("the skip did not fall through to the far box")
	}

	masked := DefaultQueryFilter()
	masked.MaskBits = 0
	CastRay(worldId, origin, translation, masked, func(ShapeId, Vec2, Vec2, Q) Q {
		t.Errorf("an empty mask reported a hit")
		return fixed.Q32One()
	})

	calls := 0
	CastRay(worldId, v2(8, 0), translation, filter, func(id ShapeId, _, _ Vec2, fraction Q) Q {
		calls++
		if id != static || !fraction.Eq(fixed.Q32Zero()) {
			t.Errorf("the inside origin did not report the static circle at fraction zero")
		}
		return fraction
	})
	if calls != 1 {
		t.Errorf("the zero fraction did not stop the cast: %d calls", calls)
	}

	w.locked = true
	requirePanic(t, func() { CastRay(worldId, origin, translation, filter, closest) })
	w.locked = false
}

// hundredShapeWorld fills a world with a hundred shapes spread over the
// three trees with random category bits, for the brute-force oracles.
func hundredShapeWorld(t *testing.T, rng *rand.Rand) WorldId {
	t.Helper()
	worldId := createTestWorld(t)
	for i := range 100 {
		bodyDef := DefaultBodyDef()
		bodyDef.Type = BodyType(i % 3)
		bodyDef.Position = v2(rng.Intn(60), rng.Intn(60))
		bodyDef.Rotation = MakeRot(fixed.Q32FromRatio(rng.Intn(8), 8))
		bodyId := CreateBody(worldId, &bodyDef)
		shapeDef := DefaultShapeDef()
		shapeDef.Filter.CategoryBits = 1 << uint(rng.Intn(3))
		switch rng.Intn(3) {
		case 0:
			circle := Circle{Radius: fixed.Q32FromRatio(1+rng.Intn(4), 2)}
			CreateCircleShape(bodyId, &shapeDef, &circle)
		case 1:
			capsule := Capsule{Center1: v2(-1, 0), Center2: v2(1, 0), Radius: fixed.Q32Half()}
			CreateCapsuleShape(bodyId, &shapeDef, &capsule)
		default:
			box := MakeBox(fixed.Q32FromRatio(1+rng.Intn(4), 2), fixed.Q32FromRatio(1+rng.Intn(4), 2))
			CreatePolygonShape(bodyId, &shapeDef, &box)
		}
	}
	return worldId
}

// TestShapeQueriesMatchBruteForce runs OverlapShape, CastShape and
// CastRayClosest against the enumeration of a hundred shapes across the
// three trees, with random masks.
func TestShapeQueriesMatchBruteForce(t *testing.T) {
	rng := rand.New(rand.NewSource(3))
	worldId := hundredShapeWorld(t, rng)
	w := getWorldFromId(worldId)
	one := fixed.Q32One()
	zero := fixed.Q32Zero()
	tolerance := linearSlop.Div(fixed.Q32FromInt(10))

	for range 50 {
		filter := DefaultQueryFilter()
		filter.MaskBits = uint64(1 + rng.Intn(7))
		unit := MakeBox(one, fixed.Q32Half())
		origin := v2(rng.Intn(60), rng.Intn(60))
		proxy := MakeOffsetProxy(unit.Vertices[:unit.Count], fixed.Q32FromRatio(rng.Intn(3), 4), origin, RotIdentity())
		translation := v2(rng.Intn(61)-30, rng.Intn(61)-30)

		// Overlap: the distance solver decides for every shape.
		wantOverlap := map[ShapeId]bool{}
		wantCast := ShapeId{}
		wantCastFraction := one
		wantRay := ShapeId{}
		wantRayFraction := one
		for i := range w.shapes {
			s := &w.shapes[i]
			if s.id == nullIndex || !shouldQueryCollide(s.filter, filter) {
				continue
			}
			transform := getBodyTransformQuick(w, &w.bodies[s.bodyId])

			input := DistanceInput{ProxyA: proxy, ProxyB: makeShapeDistanceProxy(s), TransformA: TransformIdentity(), TransformB: transform, UseRadii: true}
			var cache SimplexCache
			if out := ShapeDistance(&input, &cache, nil); !tolerance.Less(out.Distance) {
				wantOverlap[shapeIdOf(w, s)] = true
			}

			castInput := ShapeCastInput{Proxy: proxy, Translation: translation, MaxFraction: one}
			if out := shapeCastShape(&castInput, s, transform); out.Hit && out.Fraction.Less(wantCastFraction) {
				wantCast, wantCastFraction = shapeIdOf(w, s), out.Fraction
			}

			rayInput := RayCastInput{Origin: origin, Translation: translation, MaxFraction: one}
			if out := rayCastShape(&rayInput, s, transform); out.Hit && zero.Less(out.Fraction) && out.Fraction.Less(wantRayFraction) {
				wantRay, wantRayFraction = shapeIdOf(w, s), out.Fraction
			}
		}

		gotOverlap := map[ShapeId]bool{}
		OverlapShape(worldId, &proxy, filter, func(id ShapeId) bool {
			gotOverlap[id] = true
			return true
		})
		if len(gotOverlap) != len(wantOverlap) {
			t.Fatalf("OverlapShape at %v with mask %d found %d shapes, brute force %d", origin, filter.MaskBits, len(gotOverlap), len(wantOverlap))
		}
		for id := range wantOverlap {
			if !gotOverlap[id] {
				t.Fatalf("OverlapShape missed a shape at %v", origin)
			}
		}

		gotCast := ShapeId{}
		gotCastFraction := one
		CastShape(worldId, &proxy, translation, filter, func(id ShapeId, _, _ Vec2, fraction Q) Q {
			if fraction.Less(gotCastFraction) {
				gotCast, gotCastFraction = id, fraction
			}
			return fraction
		})
		if gotCast != wantCast || !gotCastFraction.Eq(wantCastFraction) {
			t.Fatalf("CastShape from %v by %v found %v at %v, brute force %v at %v", origin, translation, gotCast, gotCastFraction, wantCast, wantCastFraction)
		}

		ray := CastRayClosest(worldId, origin, translation, filter)
		if ray.Hit != (wantRay != ShapeId{}) || (ray.Hit && (ray.ShapeId != wantRay || !ray.Fraction.Eq(wantRayFraction))) {
			t.Fatalf("CastRayClosest from %v by %v found %+v, brute force %v at %v", origin, translation, ray, wantRay, wantRayFraction)
		}
	}
}

// TestCastMoverStopsAtTheWall sweeps a capsule at a static box: it stops
// within a slop of the face, a shape it already overlaps does not stop
// it, and a thin capsule panics.
func TestCastMoverStopsAtTheWall(t *testing.T) {
	worldId := createTestWorld(t)
	wallDef := DefaultBodyDef()
	wallDef.Position = v2(10, 0)
	wall := CreateBody(worldId, &wallDef)
	shapeDef := DefaultShapeDef()
	wallBox := MakeBox(fixed.Q32One(), fixed.Q32FromInt(5))
	CreatePolygonShape(wall, &shapeDef, &wallBox)

	mover := Capsule{Center1: v2(0, -1), Center2: v2(0, 1), Radius: fixed.Q32Half()}
	fraction := CastMover(worldId, &mover, v2(20, 0), DefaultQueryFilter())

	// The capsule surface reaches the face at x = 9 after 8.5 units of
	// the 20; the sweep targets the radius less a slop.
	exact := fixed.Q32MustParse("0.425")
	if !fraction.Sub(exact).Abs().Less(fixed.Q32MustParse("0.001")) {
		t.Fatalf("mover fraction %v, want near 0.425", fraction)
	}

	// A circle around the start overlaps the mover and does not stop it.
	addStaticCircle(t, worldId, v2(0, 0))
	if again := CastMover(worldId, &mover, v2(20, 0), DefaultQueryFilter()); !again.Eq(fraction) {
		t.Fatalf("an overlapping shape changed the fraction to %v", again)
	}

	thin := mover
	thin.Radius = linearSlop
	requirePanic(t, func() { CastMover(worldId, &thin, v2(20, 0), DefaultQueryFilter()) })
}
