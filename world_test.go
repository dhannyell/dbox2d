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
func createTestWorld(t testing.TB) WorldId {
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

func TestWorldAccessorsRoundTrip(t *testing.T) {
	worldId := createTestWorld(t)
	worldId.SetGravity(v2(3, -4))
	worldId.SetRestitutionThreshold(fixed.Q32FromInt(2))
	worldId.SetHitEventThreshold(fixed.Q32FromInt(3))
	worldId.SetContactTuning(fixed.Q32FromInt(4), fixed.Q32Half(), fixed.Q32FromInt(5))
	worldId.SetMaximumLinearSpeed(fixed.Q32FromInt(6))
	worldId.SetUserData("world")
	worldId.EnableSleeping(false)
	worldId.EnableContinuous(false)
	worldId.EnableWarmStarting(false)
	worldId.EnableSpeculative(false)

	checks := []struct {
		name string
		got  Q
		want Q
	}{
		{"restitution threshold", worldId.GetRestitutionThreshold(), fixed.Q32FromInt(2)},
		{"hit event threshold", worldId.GetHitEventThreshold(), fixed.Q32FromInt(3)},
		{"maximum linear speed", worldId.GetMaximumLinearSpeed(), fixed.Q32FromInt(6)},
	}
	for _, check := range checks {
		if !check.got.Eq(check.want) {
			t.Errorf("%s = %v, want %v", check.name, check.got, check.want)
		}
	}
	if got := worldId.GetGravity(); got != v2(3, -4) {
		t.Errorf("gravity = %v, want %v", got, v2(3, -4))
	}
	if worldId.IsSleepingEnabled() || worldId.IsContinuousEnabled() || worldId.IsWarmStartingEnabled() {
		t.Error("one of the boolean accessors remained enabled")
	}
	if got := worldId.GetUserData(); got != "world" {
		t.Errorf("user data = %v, want world", got)
	}
}

func TestWorldGetCounters(t *testing.T) {
	worldId := createTestWorld(t)
	bodyA := addDynamicBox(t, worldId, v2(0, 0))
	bodyB := addDynamicBox(t, worldId, v2(1, 0))
	addDynamicBox(t, worldId, v2(2, 0))

	jointDef := DefaultDistanceJointDef()
	jointDef.BodyIdA = bodyA
	jointDef.BodyIdB = bodyB
	CreateDistanceJoint(worldId, &jointDef)
	worldId.Step(stepDt(), 4)

	counters := worldId.GetCounters()
	if counters.BodyCount == 0 || counters.ShapeCount == 0 || counters.JointCount == 0 || counters.ContactCount == 0 {
		t.Fatalf("counters = %+v, want bodies, shapes, joints and contacts", counters)
	}
	if counters.IslandCount == 0 || counters.TreeHeight == 0 {
		t.Fatalf("counters = %+v, want islands and a dynamic tree", counters)
	}
}

// TestStepFillsTheProfile pins that GetProfile reports non-negative parts
// that sum consistently, and that a zero time step leaves it zeroed.
func TestStepFillsTheProfile(t *testing.T) {
	worldId := createTestWorld(t)

	groundDef := DefaultBodyDef()
	groundId := CreateBody(worldId, &groundDef)
	groundShapeDef := DefaultShapeDef()
	ground := MakeBox(fixed.Q32FromInt(50), fixed.Q32One())
	CreatePolygonShape(groundId, &groundShapeDef, &ground)

	// A stack of boxes, not a handful: the timer resolution of some hosts
	// is coarse, so the step needs enough work to read back above zero.
	for i := range 200 {
		addDynamicBox(t, worldId, v2(i%10, 3+2*(i/10)))
	}

	for range 10 {
		worldId.Step(stepDt(), 4)
	}

	p := worldId.GetProfile()
	if p.Step <= 0 {
		t.Fatalf("Step = %v, want > 0", p.Step)
	}
	parts := []float64{
		p.Pairs, p.Collide, p.Solve, p.MergeIslands, p.PrepareStages, p.SolveConstraints,
		p.PrepareConstraints, p.IntegrateVelocities, p.WarmStart, p.SolveImpulses, p.IntegratePositions,
		p.RelaxImpulses, p.ApplyRestitution, p.StoreImpulses, p.SplitIslands, p.Transforms, p.HitEvents,
		p.Refit, p.Bullets, p.SleepIslands, p.Sensors,
	}
	for _, part := range parts {
		if part < 0 {
			t.Fatalf("profile = %+v, want no negative part", p)
		}
	}

	const slack = 0.5
	if top := p.Pairs + p.Collide + p.Solve + p.Sensors; top > p.Step+slack {
		t.Errorf("Pairs+Collide+Solve+Sensors = %v, want <= Step (%v) + %v", top, p.Step, slack)
	}
	if sub := p.MergeIslands + p.PrepareStages + p.SolveConstraints + p.Transforms + p.HitEvents +
		p.Refit + p.Bullets + p.SleepIslands; sub > p.Solve+slack {
		t.Errorf("solve parts sum = %v, want <= Solve (%v) + %v", sub, p.Solve, slack)
	}

	worldId.Step(fixed.Q32Zero(), 4)
	if got := worldId.GetProfile(); got != (Profile{}) {
		t.Errorf("GetProfile() after a zero time step = %+v, want zero", got)
	}
}

func TestSetFrictionCallbackAffectsNextContact(t *testing.T) {
	worldId := createTestWorld(t)
	constant := fixed.Q32MustParse("0.75")
	worldId.SetFrictionCallback(func(Q, int, Q, int) Q { return constant })
	addDynamicCircle(t, worldId, v2(0, 0))
	addDynamicCircle(t, worldId, v2(1, 0))
	worldId.Step(stepDt(), 4)

	w := getWorldFromId(worldId)
	if len(w.contacts) == 0 {
		t.Fatal("the overlapping circles created no contact")
	}
	if got := getContactSim(w, &w.contacts[0]).friction; !got.Eq(constant) {
		t.Errorf("contact friction = %v, want %v", got, constant)
	}
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
		if pair.s1.GetPosition() != pair.s2.GetPosition() {
			t.Errorf("%s: position %v, want %v", pair.name, pair.s2.GetPosition(), pair.s1.GetPosition())
		}
		if pair.s1.GetRotation() != pair.s2.GetRotation() {
			t.Errorf("%s: rotation %v, want %v", pair.name, pair.s2.GetRotation(), pair.s1.GetRotation())
		}
		if !pair.s1.GetMass().Eq(pair.s2.GetMass()) {
			t.Errorf("%s: mass %v, want %v", pair.name, pair.s2.GetMass(), pair.s1.GetMass())
		}
		if pair.s1.GetShapeCount() != pair.s2.GetShapeCount() {
			t.Errorf("%s: shape count %d, want %d", pair.name, pair.s2.GetShapeCount(), pair.s1.GetShapeCount())
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
	if !bodyId.GetMass().Eq(wantBox.Mass) {
		t.Fatalf("mass with one box = %v, want %v", bodyId.GetMass(), wantBox.Mass)
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
	if !bodyId.GetMass().Eq(wantTotal) {
		t.Errorf("mass with two shapes = %v, want %v", bodyId.GetMass(), wantTotal)
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
	if !bodyId.GetMass().Eq(wantBox.Mass) {
		t.Errorf("mass after the destroy = %v, want %v", bodyId.GetMass(), wantBox.Mass)
	}
	if bodyId.GetShapeCount() != 1 {
		t.Errorf("shape count after the destroy = %d, want 1", bodyId.GetShapeCount())
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
	stats := worldId.OverlapAABB(query, DefaultQueryFilter(), func(id ShapeId) bool {
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
	worldId.OverlapAABB(query, DefaultQueryFilter(), func(ShapeId) bool {
		calls++
		return false
	})
	if calls != 3 {
		t.Errorf("a false return did not stop the tree walk: %d calls", calls)
	}

	filter := DefaultQueryFilter()
	filter.MaskBits = 0
	worldId.OverlapAABB(query, filter, func(ShapeId) bool {
		t.Errorf("an empty mask reported a shape")
		return true
	})

	w.locked = true
	requirePanic(t, func() { worldId.OverlapAABB(query, DefaultQueryFilter(), func(ShapeId) bool { return true }) })
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
	stats := worldId.CastRay(origin, translation, filter, closest)
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
	worldId.CastRay(origin, translation, filter, func(id ShapeId, p, n Vec2, fraction Q) Q {
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
	worldId.CastRay(origin, translation, masked, func(ShapeId, Vec2, Vec2, Q) Q {
		t.Errorf("an empty mask reported a hit")
		return fixed.Q32One()
	})

	calls := 0
	worldId.CastRay(v2(8, 0), translation, filter, func(id ShapeId, _, _ Vec2, fraction Q) Q {
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
	requirePanic(t, func() { worldId.CastRay(origin, translation, filter, closest) })
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
		worldId.OverlapShape(&proxy, filter, func(id ShapeId) bool {
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
		worldId.CastShape(&proxy, translation, filter, func(id ShapeId, _, _ Vec2, fraction Q) Q {
			if fraction.Less(gotCastFraction) {
				gotCast, gotCastFraction = id, fraction
			}
			return fraction
		})
		if gotCast != wantCast || !gotCastFraction.Eq(wantCastFraction) {
			t.Fatalf("CastShape from %v by %v found %v at %v, brute force %v at %v", origin, translation, gotCast, gotCastFraction, wantCast, wantCastFraction)
		}

		ray := worldId.CastRayClosest(origin, translation, filter)
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
	fraction := worldId.CastMover(&mover, v2(20, 0), DefaultQueryFilter())

	// The capsule surface reaches the face at x = 9 after 8.5 units of
	// the 20; the sweep targets the radius less a slop.
	exact := fixed.Q32MustParse("0.425")
	if !fraction.Sub(exact).Abs().Less(fixed.Q32MustParse("0.001")) {
		t.Fatalf("mover fraction %v, want near 0.425", fraction)
	}

	// A circle around the start overlaps the mover and does not stop it.
	addStaticCircle(t, worldId, v2(0, 0))
	if again := worldId.CastMover(&mover, v2(20, 0), DefaultQueryFilter()); !again.Eq(fraction) {
		t.Fatalf("an overlapping shape changed the fraction to %v", again)
	}

	thin := mover
	thin.Radius = linearSlop
	requirePanic(t, func() { worldId.CastMover(&thin, v2(20, 0), DefaultQueryFilter()) })
}

// TestCustomFilterRejectsPair locks down that a callback returning false
// keeps the pair from ever becoming a contact.
func TestCustomFilterRejectsPair(t *testing.T) {
	worldId := createTestWorld(t)
	worldId.SetCustomFilterCallback(func(ShapeId, ShapeId) bool { return false })

	addDynamicBox(t, worldId, v2(0, 0))
	addDynamicBox(t, worldId, v2(0, 0))

	dt := stepDt()
	for range 10 {
		worldId.Step(dt, 4)
	}

	if events := worldId.GetContactEvents(); len(events.BeginEvents) != 0 {
		t.Fatalf("got %d begin-touch events, want 0", len(events.BeginEvents))
	}
	if got := worldId.GetCounters().ContactCount; got != 0 {
		t.Fatalf("contact count = %d, want 0", got)
	}
}

// TestCustomFilterAcceptingKeepsWitness locks down that an always-true
// callback does not perturb the simulation: the checksum of a scene run
// with the callback matches the same scene run with no callback at all.
func TestCustomFilterAcceptingKeepsWitness(t *testing.T) {
	buildScene := func(worldId WorldId) {
		addDynamicBox(t, worldId, v2(0, 5))
		addDynamicBox(t, worldId, v2(0, 6))
	}

	dt := stepDt()

	plainId := createTestWorld(t)
	buildScene(plainId)
	for range 30 {
		plainId.Step(dt, 4)
	}
	want := Checksum(plainId)

	filteredId := createTestWorld(t)
	filteredId.SetCustomFilterCallback(func(ShapeId, ShapeId) bool { return true })
	buildScene(filteredId)
	for range 30 {
		filteredId.Step(dt, 4)
	}
	if got := Checksum(filteredId); got != want {
		t.Fatalf("checksum with an always-true filter = %d, want %d", got, want)
	}
}

// TestExplodeImpulseByDistance pins the falloff shape of b2World_Explode.
// Every case uses addDynamicBox: a unit half-width square, density 1, so
// mass = 4 and invMass = 0.25. The box sits on the x-axis so its closest
// edge faces the explosion head-on, giving getShapeProjectedPerimeter a
// perimeter of exactly 2 (the two y-extents, no polygon radius).
func TestExplodeImpulseByDistance(t *testing.T) {
	radius := fixed.Q32FromInt(4)
	falloff := fixed.Q32FromInt(2)
	impulsePerLength := fixed.Q32FromInt(8)

	def := func() ExplosionDef {
		return ExplosionDef{
			MaskBits:         DefaultMaskBits,
			Position:         v2(0, 0),
			Radius:           radius,
			Falloff:          falloff,
			ImpulsePerLength: impulsePerLength,
		}
	}

	t.Run("full impulse inside the radius", func(t *testing.T) {
		// distance = radius/2 = 2, closest edge at x=1: box center at x=3.
		// magnitude = impulsePerLength * perimeter * scale = 8*2*1 = 16.
		// velocity = invMass * magnitude = 0.25*16 = 4, along +x.
		worldId := createTestWorld(t)
		bodyId := addDynamicBox(t, worldId, v2(3, 0))

		explosionDef := def()
		worldId.Explode(&explosionDef)

		want := v2(4, 0)
		if v := bodyId.GetLinearVelocity(); !v.X.Eq(want.X) || !v.Y.Eq(want.Y) {
			t.Fatalf("velocity at half radius = %v, want %v", v, want)
		}
	})

	t.Run("half impulse in the falloff band", func(t *testing.T) {
		// distance = radius + falloff/2 = 5, closest edge at x=5: center at x=6.
		// scale = (radius+falloff-distance)/falloff = (6-5)/2 = 0.5.
		// magnitude = 8*2*0.5 = 8. velocity = 0.25*8 = 2, along +x.
		worldId := createTestWorld(t)
		bodyId := addDynamicBox(t, worldId, v2(6, 0))

		explosionDef := def()
		worldId.Explode(&explosionDef)

		want := v2(2, 0)
		if v := bodyId.GetLinearVelocity(); !v.X.Eq(want.X) || !v.Y.Eq(want.Y) {
			t.Fatalf("velocity in the falloff band = %v, want %v", v, want)
		}
	})

	t.Run("no impulse beyond the falloff, sleeper stays asleep", func(t *testing.T) {
		// distance = radius+falloff+1 = 7, closest edge at x=7: center at x=8.
		worldId := createTestWorld(t)
		bodyId := addDynamicBox(t, worldId, v2(8, 0))
		bodyId.SetAwake(false)

		explosionDef := def()
		worldId.Explode(&explosionDef)

		if v := bodyId.GetLinearVelocity(); !v.X.Eq(fixed.Q32Zero()) || !v.Y.Eq(fixed.Q32Zero()) {
			t.Fatalf("velocity beyond the falloff = %v, want zero", v)
		}
		if bodyId.IsAwake() {
			t.Fatalf("a shape beyond the falloff should not wake its body")
		}
	})

	t.Run("a sleeper inside the radius wakes up", func(t *testing.T) {
		worldId := createTestWorld(t)
		bodyId := addDynamicBox(t, worldId, v2(3, 0))
		bodyId.SetAwake(false)

		explosionDef := def()
		worldId.Explode(&explosionDef)

		if !bodyId.IsAwake() {
			t.Fatalf("a shape inside the radius should wake its sleeping body")
		}
	})

	t.Run("center on the explosion point falls back to +x", func(t *testing.T) {
		// The box's local centroid is its body origin, so the D-012
		// zero-distance branch picks a closest point equal to the
		// explosion position: the direction is zero-length too, and
		// b2World_Explode falls back to (1, 0) rather than the shape
		// centroid direction.
		worldId := createTestWorld(t)
		bodyId := addDynamicBox(t, worldId, v2(0, 0))

		explosionDef := def()
		worldId.Explode(&explosionDef)

		v := bodyId.GetLinearVelocity()
		if !v.Y.Eq(fixed.Q32Zero()) || !fixed.Q32Zero().Less(v.X) {
			t.Fatalf("velocity at the explosion center = %v, want a positive x and zero y", v)
		}
	})
}
