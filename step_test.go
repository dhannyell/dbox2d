package dbox2d

import (
	"testing"

	"github.com/dhannyell/fixed"
)

// This file tests the Step pipeline: exact integration, sleep tracking, the
// zero-allocation contract and bit-for-bit reproducibility.

// stepDt returns the recommended fixed time step of 1/60 second.
func stepDt() Q {
	return fixed.Q32One().Div(fixed.Q32FromInt(60))
}

// addDynamicBox creates a dynamic body with a unit box at the position.
func addDynamicBox(t *testing.T, worldId WorldId, position Vec2) BodyId {
	t.Helper()
	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = position
	bodyId := CreateBody(worldId, &bodyDef)
	shapeDef := DefaultShapeDef()
	box := MakeSquare(fixed.Q32One())
	CreatePolygonShape(bodyId, &shapeDef, &box)
	return bodyId
}

func requirePanic(t *testing.T, f func()) {
	t.Helper()
	defer func() {
		if recover() == nil {
			t.Errorf("call did not panic")
		}
	}()
	f()
}

// TestStepAppliesGravityExactly pins the integration order: with no damping
// and no rotation, the velocity is the exact sum of the sub-step impulses
// and the position is the exact sum of the sub-step displacements.
func TestStepAppliesGravityExactly(t *testing.T) {
	worldId := createTestWorld(t)
	bodyId := addDynamicBox(t, worldId, v2(0, 10))

	dt := stepDt()
	const subStepCount = 4
	const stepCount = 8

	h := dt.Div(fixed.Q32FromInt(subStepCount))
	gravity := worldId.Gravity()

	// The scalar mirror of the loop: velocity gains h*g per sub-step and the
	// position gains h*v after each velocity update.
	wantVelocityY := fixed.Q32Zero()
	wantPositionY := bodyId.Position().Y
	for range stepCount {
		deltaY := fixed.Q32Zero()
		for range subStepCount {
			wantVelocityY = wantVelocityY.Add(h.Mul(gravity.Y))
			deltaY = deltaY.Add(h.Mul(wantVelocityY))
		}
		wantPositionY = wantPositionY.Add(deltaY)
	}

	for range stepCount {
		Step(worldId, dt, subStepCount)
	}

	w := getWorldFromId(worldId)
	b := getBodyFullId(w, bodyId)
	state := getBodyState(w, b)
	if !state.linearVelocity.Y.Eq(wantVelocityY) {
		t.Errorf("velocity y = %v, want %v", state.linearVelocity.Y, wantVelocityY)
	}
	if !bodyId.Position().Y.Eq(wantPositionY) {
		t.Errorf("position y = %v, want %v", bodyId.Position().Y, wantPositionY)
	}
	if !bodyId.Position().X.Eq(fixed.Q32Zero()) {
		t.Errorf("position x moved to %v", bodyId.Position().X)
	}
}

// TestStepAppliesDampingByDivision covers the fixed-point form of the Pade
// damping factor. Multiplying by a rounded reciprocal would lose more bits.
func TestStepAppliesDampingByDivision(t *testing.T) {
	def := DefaultWorldDef()
	def.Gravity = Vec2Zero()
	worldId := CreateWorld(&def)
	t.Cleanup(func() { DestroyWorld(worldId) })

	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.LinearVelocity = Vec2{X: fixed.Q32FromInt(12), Y: fixed.Q32FromInt(-7)}
	bodyDef.AngularVelocity = fixed.Q32MustParse("0.25")
	bodyDef.LinearDamping = fixed.Q32MustParse("0.4")
	bodyDef.AngularDamping = fixed.Q32MustParse("0.7")
	bodyId := CreateBody(worldId, &bodyDef)
	shapeDef := DefaultShapeDef()
	box := MakeSquare(fixed.Q32One())
	CreatePolygonShape(bodyId, &shapeDef, &box)

	w := getWorldFromId(worldId)
	b := getBodyFullId(w, bodyId)
	initialState := *getBodyState(w, b)
	dt := stepDt()
	Step(worldId, dt, 1)

	state := getBodyState(w, b)
	linearDenominator := fixed.Q32One().Add(dt.Mul(bodyDef.LinearDamping))
	angularDenominator := fixed.Q32One().Add(dt.Mul(bodyDef.AngularDamping))
	wantLinear := Vec2{
		X: initialState.linearVelocity.X.Div(linearDenominator),
		Y: initialState.linearVelocity.Y.Div(linearDenominator),
	}
	wantAngular := initialState.angularVelocity.Div(angularDenominator)
	if state.linearVelocity != wantLinear {
		t.Errorf("linear velocity = %v, want %v", state.linearVelocity, wantLinear)
	}
	if !state.angularVelocity.Eq(wantAngular) {
		t.Errorf("angular velocity = %v, want %v", state.angularVelocity, wantAngular)
	}
}

// TestStepConvertsTorqueAndArcSpeedToTurns covers both solver sites where an
// angular velocity changes units between radians and turns.
func TestStepConvertsTorqueAndArcSpeedToTurns(t *testing.T) {
	def := DefaultWorldDef()
	def.Gravity = Vec2Zero()
	worldId := CreateWorld(&def)
	t.Cleanup(func() { DestroyWorld(worldId) })

	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.AngularVelocity = fixed.Q32MustParse("0.02")
	bodyDef.SleepThreshold = fixed.Q32MustParse("0.1")
	bodyId := CreateBody(worldId, &bodyDef)
	shapeDef := DefaultShapeDef()
	box := MakeSquare(fixed.Q32One())
	CreatePolygonShape(bodyId, &shapeDef, &box)

	w := getWorldFromId(worldId)
	b := getBodyFullId(w, bodyId)
	sim := getBodySim(w, b)
	sim.torque = tau
	b.sleepTime = fixed.Q32One()

	dt := stepDt()
	wantAngular := bodyDef.AngularVelocity.Add(dt.Mul(sim.invInertia).Mul(sim.torque).Div(tau))
	Step(worldId, dt, 1)

	state := getBodyState(w, b)
	if !state.angularVelocity.Eq(wantAngular) {
		t.Errorf("angular velocity = %v, want %v", state.angularVelocity, wantAngular)
	}
	if !b.sleepTime.Eq(fixed.Q32Zero()) {
		t.Errorf("sleep time = %v, want zero for the rotating body", b.sleepTime)
	}
}

// TestStepRefreshesFastBodyBoundsWithoutCCD keeps the discrete world
// coherent while the continuous collision stage is deferred.
func TestStepRefreshesFastBodyBoundsWithoutCCD(t *testing.T) {
	def := DefaultWorldDef()
	def.Gravity = Vec2Zero()
	worldId := CreateWorld(&def)
	t.Cleanup(func() { DestroyWorld(worldId) })

	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.LinearVelocity = Vec2{X: fixed.Q32FromInt(100)}
	bodyId := CreateBody(worldId, &bodyDef)
	shapeDef := DefaultShapeDef()
	box := MakeSquare(fixed.Q32One())
	shapeId := CreatePolygonShape(bodyId, &shapeDef, &box)

	w := getWorldFromId(worldId)
	s := getShape(w, shapeId)
	before := s.aabb
	Step(worldId, stepDt(), 4)

	b := getBodyFullId(w, bodyId)
	sim := getBodySim(w, b)
	want := computeShapeAABB(s, sim.transform)
	want.LowerBound.X = want.LowerBound.X.Sub(speculativeDistance)
	want.LowerBound.Y = want.LowerBound.Y.Sub(speculativeDistance)
	want.UpperBound.X = want.UpperBound.X.Add(speculativeDistance)
	want.UpperBound.Y = want.UpperBound.Y.Add(speculativeDistance)
	if s.aabb != want {
		t.Errorf("shape AABB = %+v, want %+v", s.aabb, want)
	}
	if s.aabb == before {
		t.Errorf("shape AABB did not move with the fast body")
	}
	if sim.center0 != sim.center || sim.rotation0 != sim.transform.Q {
		t.Errorf("previous transform was not finalized without CCD")
	}
}

// TestStepRejectsInvalidInput covers the public assertions retained as
// panics in every build.
func TestStepRejectsInvalidInput(t *testing.T) {
	worldId := createTestWorld(t)

	t.Run("saturated time step", func(t *testing.T) {
		requirePanic(t, func() { Step(worldId, fixed.Q32MaxValue(), 4) })
	})
	t.Run("non-positive sub-step count", func(t *testing.T) {
		requirePanic(t, func() { Step(worldId, stepDt(), 0) })
	})
	t.Run("locked world", func(t *testing.T) {
		w := getWorldFromId(worldId)
		w.locked = true
		defer func() { w.locked = false }()
		requirePanic(t, func() { Step(worldId, stepDt(), 4) })
	})
}

// TestStepTracksSleepTime pins the finalize branch: a body at rest
// accumulates sleep time, a moving body resets it.
func TestStepTracksSleepTime(t *testing.T) {
	def := DefaultWorldDef()
	def.Gravity = Vec2Zero()
	worldId := CreateWorld(&def)
	t.Cleanup(func() { DestroyWorld(worldId) })

	restingId := addDynamicBox(t, worldId, v2(0, 0))

	movingId := addDynamicBox(t, worldId, v2(10, 0))

	dt := stepDt()
	const stepCount = 10
	for range stepCount {
		Step(worldId, dt, 4)
	}

	w := getWorldFromId(worldId)
	resting := getBodyFullId(w, restingId)
	wantSleep := dt.Mul(fixed.Q32FromInt(stepCount))
	if !resting.sleepTime.Eq(wantSleep) {
		t.Errorf("sleep time at rest = %v, want %v", resting.sleepTime, wantSleep)
	}

	// A push above the sleep threshold resets the sleep time.
	moving := getBodyFullId(w, movingId)
	getBodyState(w, moving).linearVelocity = Vec2{X: fixed.Q32One()}
	Step(worldId, dt, 4)
	if !moving.sleepTime.Eq(fixed.Q32Zero()) {
		t.Errorf("sleep time in motion = %v, want zero", moving.sleepTime)
	}
}

// TestStepAllocatesNothing pins the hot-path contract: after the first
// step that activates a contact, a step allocates nothing. The first step
// grows the graph colors, the arena and the event buffers once, as the
// reference does. Sleep stays off so the contact keeps solving.
func TestStepAllocatesNothing(t *testing.T) {
	def := DefaultWorldDef()
	def.EnableSleep = false
	worldId := CreateWorld(&def)
	t.Cleanup(func() { DestroyWorld(worldId) })
	boxOnGround(t, worldId, fixed.Q32Zero())
	for i := range 8 {
		addDynamicBox(t, worldId, v2(10+i*3, 0))
	}
	w := getWorldFromId(worldId)

	dt := stepDt()
	Step(worldId, dt, 4)
	if len(w.constraintGraph.colors[1].contactSims) != 1 {
		t.Fatalf("the warm-up step did not activate the contact")
	}

	allocs := testing.AllocsPerRun(10, func() {
		Step(worldId, dt, 4)
	})
	if allocs != 0 {
		t.Errorf("Step allocates %v times per call, want 0", allocs)
	}
}

// TestStepIsReproducibleBitForBit pins determinism: two identical builds
// produce the same checksum at every step.
func TestStepIsReproducibleBitForBit(t *testing.T) {
	build := func() WorldId {
		worldId := createTestWorld(t)
		for i := range 5 {
			id := addDynamicBox(t, worldId, v2(i*3, i))
			w := getWorldFromId(worldId)
			b := getBodyFullId(w, id)
			getBodyState(w, b).angularVelocity = fixed.Q32MustParse("0.1")
		}
		return worldId
	}

	world1 := build()
	world2 := build()

	dt := stepDt()
	for step := range 120 {
		Step(world1, dt, 4)
		Step(world2, dt, 4)
		c1, c2 := Checksum(world1), Checksum(world2)
		if c1 != c2 {
			t.Fatalf("checksums diverge at step %d: %d and %d", step, c1, c2)
		}
	}
}

// TestStepPutsARestingIslandToSleep pins the sleep tail of solve: a body
// at rest for the sleep time leaves the awake set, and a moving body stays.
func TestStepPutsARestingIslandToSleep(t *testing.T) {
	def := DefaultWorldDef()
	def.Gravity = Vec2Zero()
	worldId := CreateWorld(&def)
	t.Cleanup(func() { DestroyWorld(worldId) })

	restingId := addDynamicBox(t, worldId, v2(0, 0))
	movingId := addDynamicBox(t, worldId, v2(10, 0))
	w := getWorldFromId(worldId)
	moving := getBodyFullId(w, movingId)
	getBodyState(w, moving).linearVelocity = Vec2{X: fixed.Q32One()}

	dt := stepDt()
	// The sleep time is one half second. A rounded 1/60 times 30 falls one
	// bit short, so the 31st step crosses it.
	for range 31 {
		Step(worldId, dt, 4)
	}

	resting := getBodyFullId(w, restingId)
	if resting.setIndex < firstSleepingSet {
		t.Fatalf("the resting body is in set %d, want a sleeping set", resting.setIndex)
	}
	if moving.setIndex != awakeSet {
		t.Errorf("the moving body is in set %d, want the awake set", moving.setIndex)
	}
	if len(w.solverSets[awakeSet].islandSims) != 1 {
		t.Errorf("the awake set holds %d islands, want 1", len(w.solverSets[awakeSet].islandSims))
	}
	validateSolverSets(w)
}

// TestStepSplitsTheIslandBeforeItSleeps pins the split candidate: an
// island with a removed constraint cannot sleep, so the finalize picks it
// for a split and the next step splits it into two sleeping islands.
func TestStepSplitsTheIslandBeforeItSleeps(t *testing.T) {
	def := DefaultWorldDef()
	def.Gravity = Vec2Zero()
	worldId := CreateWorld(&def)
	t.Cleanup(func() { DestroyWorld(worldId) })

	idA := addDynamicBox(t, worldId, v2(0, 0))
	idB := addDynamicBox(t, worldId, v2(10, 0))
	w := getWorldFromId(worldId)

	// Join the islands, then remove the link so the split is pending.
	c := startTouching(t, w, idA, idB)
	mergeAwakeIslands(w)
	c.flags &^= contactTouchingFlag
	destroyContact(w, c, false)

	bodyA := getBodyFullId(w, idA)
	if w.islands[bodyA.islandId].constraintRemoveCount != 1 {
		t.Fatalf("the island has no pending split")
	}

	dt := stepDt()
	for range 31 {
		Step(worldId, dt, 4)
	}
	if w.splitIslandId == nullIndex {
		t.Fatalf("the finalize did not pick the split candidate")
	}
	if bodyA.setIndex != awakeSet {
		t.Fatalf("the island slept with a pending split")
	}

	Step(worldId, dt, 4)
	bodyB := getBodyFullId(w, idB)
	if bodyA.islandId == bodyB.islandId {
		t.Errorf("the bodies still share island %d", bodyA.islandId)
	}
	if bodyA.setIndex < firstSleepingSet || bodyB.setIndex < firstSleepingSet {
		t.Errorf("the split islands are in sets %d and %d, want sleeping sets", bodyA.setIndex, bodyB.setIndex)
	}
	if w.splitIslandId != nullIndex {
		t.Errorf("the split candidate is still set")
	}
	validateSolverSets(w)
}

// The tests below cover the collide block of Step: the touch state changes, the
// contact events and the move events. The broad-phase has not landed, so
// each test creates its contact by hand, as the pair update will.

// boxOnGround builds a static ground with its top at y = 0 and a unit box
// of mass one just above it, with one contact between them. The box sits
// at the height y, so a small positive height starts a short fall.
func boxOnGround(t *testing.T, worldId WorldId, height Q) BodyId {
	t.Helper()
	w := getWorldFromId(worldId)

	groundDef := DefaultBodyDef()
	groundDef.Position = Vec2{Y: fixed.Q32Half().Neg()}
	groundId := CreateBody(worldId, &groundDef)
	shapeDef := DefaultShapeDef()
	shapeDef.EnableContactEvents = true
	shapeDef.EnableHitEvents = true
	ground := MakeBox(fixed.Q32FromInt(5), fixed.Q32Half())
	CreatePolygonShape(groundId, &shapeDef, &ground)

	boxDef := DefaultBodyDef()
	boxDef.Type = DynamicBody
	boxDef.Position = Vec2{Y: fixed.Q32Half().Add(height)}
	boxId := CreateBody(worldId, &boxDef)
	unit := MakeBox(fixed.Q32Half(), fixed.Q32Half())
	CreatePolygonShape(boxId, &shapeDef, &unit)

	createContact(w, firstShape(w, groundId), firstShape(w, boxId))
	return boxId
}

// TestStepLandsAFallingBox pins the whole pipeline: the collide pass
// starts the touch, the solver holds the box on the ground and the sleep
// tail puts it to sleep.
func TestStepLandsAFallingBox(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	boxId := boxOnGround(t, worldId, fixed.Q32MustParse("0.05"))
	box := getBodyFullId(w, boxId)

	dt := stepDt()
	for range 120 {
		Step(worldId, dt, 4)
	}

	sim := getBodySim(w, box)
	tolerance := fixed.Q32MustParse("0.01")
	if !withinQ(sim.center.Y, fixed.Q32Half(), tolerance) {
		t.Errorf("the box rests at y %v, want 0.5", sim.center.Y)
	}
	if box.setIndex < firstSleepingSet {
		t.Errorf("the box is in set %d, want a sleeping set", box.setIndex)
	}
	c := &w.contacts[0]
	if c.flags&contactTouchingFlag == 0 || c.islandId == nullIndex {
		t.Errorf("the contact is not touching inside an island")
	}
	validateSolverSets(w)
}

// TestStepReportsBeginAndEndTouch pins the contact events: the begin
// event arrives on the step of the touch with a manifold without
// impulses, and the end event arrives on the step of the separation.
func TestStepReportsBeginAndEndTouch(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	boxId := boxOnGround(t, worldId, fixed.Q32Zero())
	box := getBodyFullId(w, boxId)

	dt := stepDt()
	Step(worldId, dt, 4)

	events := GetContactEvents(worldId)
	if len(events.BeginEvents) != 1 || len(events.EndEvents) != 0 || len(events.HitEvents) != 0 {
		t.Fatalf("the first step reports %d begin, %d end and %d hit events, want 1, 0 and 0", len(events.BeginEvents), len(events.EndEvents), len(events.HitEvents))
	}
	begin := events.BeginEvents[0]
	if begin.ShapeIdB != shapeIdOf(w, firstShape(w, boxId)) {
		t.Errorf("the begin event names the wrong shape")
	}
	if begin.Manifold.PointCount == 0 || !begin.Manifold.Points[0].NormalImpulse.Eq(fixed.Q32Zero()) {
		t.Errorf("the begin manifold has %d points and impulse %v, want points and zero", begin.Manifold.PointCount, begin.Manifold.Points[0].NormalImpulse)
	}

	Step(worldId, dt, 4)
	if len(GetContactEvents(worldId).BeginEvents) != 0 {
		t.Errorf("the second step repeats the begin event")
	}

	// Lift the box out of the speculative margin. The bounds refresh on
	// the next finalize, so the contact survives that step and separates.
	sim := getBodySim(w, box)
	lift := fixed.Q32One()
	sim.center.Y = sim.center.Y.Add(lift)
	sim.transform.P.Y = sim.transform.P.Y.Add(lift)
	getBodyState(w, box).linearVelocity = Vec2{Y: fixed.Q32FromInt(5)}
	Step(worldId, dt, 4)

	events = GetContactEvents(worldId)
	if len(events.EndEvents) != 1 {
		t.Fatalf("the lift step reports %d end events, want 1", len(events.EndEvents))
	}
	if events.EndEvents[0].ShapeIdB != shapeIdOf(w, firstShape(w, boxId)) {
		t.Errorf("the end event names the wrong shape")
	}
	c := &w.contacts[0]
	if c.flags&contactTouchingFlag != 0 || c.colorIndex != nullIndex || c.islandId != nullIndex {
		t.Errorf("the contact still touches after the lift")
	}
	validateSolverSets(w)
}

// TestStepDestroysADisjointContact pins the disjoint branch: once the fat
// bounds stop overlapping, the collide pass destroys the contact.
func TestStepDestroysADisjointContact(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	boxId := boxOnGround(t, worldId, fixed.Q32Zero())
	box := getBodyFullId(w, boxId)

	dt := stepDt()
	Step(worldId, dt, 4)
	if w.contactIdPool.idCount() != 1 {
		t.Fatalf("the contact did not survive the first step")
	}

	// Carry the box far away. The finalize of this step refreshes the
	// bounds; the collide of the next step sees no overlap.
	sim := getBodySim(w, box)
	far := fixed.Q32FromInt(20)
	sim.center.Y = sim.center.Y.Add(far)
	sim.transform.P.Y = sim.transform.P.Y.Add(far)
	getBodyState(w, box).linearVelocity = Vec2{Y: fixed.Q32FromInt(5)}
	Step(worldId, dt, 4)
	Step(worldId, dt, 4)

	if w.contactIdPool.idCount() != 0 || w.pairSet.count != 0 {
		t.Errorf("the disjoint contact survives: %d contacts and %d pairs", w.contactIdPool.idCount(), w.pairSet.count)
	}
	if len(GetContactEvents(worldId).EndEvents) != 0 {
		t.Errorf("a disjoint contact that never touched reported an end event")
	}
	validateSolverSets(w)
}

// TestStepReportsMoveEvents pins the body events: every awake body reports
// its transform, and the step that puts it to sleep flags the event.
func TestStepReportsMoveEvents(t *testing.T) {
	def := DefaultWorldDef()
	def.Gravity = Vec2Zero()
	worldId := CreateWorld(&def)
	t.Cleanup(func() { DestroyWorld(worldId) })
	w := getWorldFromId(worldId)

	restingId := addDynamicBox(t, worldId, v2(3, 0))
	resting := getBodyFullId(w, restingId)

	dt := stepDt()
	Step(worldId, dt, 4)
	events := GetBodyEvents(worldId)
	if len(events.MoveEvents) != 1 {
		t.Fatalf("the step reports %d move events, want 1", len(events.MoveEvents))
	}
	move := events.MoveEvents[0]
	if move.BodyId != restingId || move.FellAsleep || move.Transform.P != v2(3, 0) {
		t.Errorf("the move event is %+v", move)
	}

	for range 30 {
		Step(worldId, dt, 4)
	}
	events = GetBodyEvents(worldId)
	if len(events.MoveEvents) != 1 || !events.MoveEvents[0].FellAsleep {
		t.Fatalf("the sleep step reports %+v", events.MoveEvents)
	}
	if resting.setIndex < firstSleepingSet {
		t.Errorf("the body is in set %d, want a sleeping set", resting.setIndex)
	}

	Step(worldId, dt, 4)
	if len(GetBodyEvents(worldId).MoveEvents) != 0 {
		t.Errorf("a sleeping body reports a move event")
	}
}

// TestStepKeepsAZeroContactFrequencyFinite pins the D-006 guard: a zero
// contact frequency gives a rigid constraint and a zero contact speed,
// where the reference would divide by zero.
func TestStepKeepsAZeroContactFrequencyFinite(t *testing.T) {
	def := DefaultWorldDef()
	def.ContactHertz = fixed.Q32Zero()
	worldId := CreateWorld(&def)
	t.Cleanup(func() { DestroyWorld(worldId) })
	w := getWorldFromId(worldId)
	boxOnGround(t, worldId, fixed.Q32Zero())

	Step(worldId, stepDt(), 4)
	if !w.contactSpeed.Eq(fixed.Q32Zero()) {
		t.Errorf("the contact speed is %v, want zero", w.contactSpeed)
	}
}

// validateWorld runs every validator of the reference that the port has.
func validateWorld(w *world) {
	validateSolverSets(w)
	validateConnectivity(w)
	validateContacts(w)
}

// TestStepReportsAHitEvent pins the hit event: a box that lands faster
// than the threshold reports one hit on the contact normal. The box starts
// inside the fat margin with its fall speed, because a contact outside the
// margin dies before the broad phase can recreate it.
func TestStepReportsAHitEvent(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	boxId := boxOnGround(t, worldId, fixed.Q32MustParse("0.05"))
	getBodyState(w, getBodyFullId(w, boxId)).linearVelocity = Vec2{Y: fixed.Q32FromInt(-3)}

	dt := stepDt()
	for range 10 {
		Step(worldId, dt, 4)
		events := GetContactEvents(worldId)
		if len(events.HitEvents) == 0 {
			continue
		}
		if len(events.HitEvents) != 1 {
			t.Fatalf("the landing step reports %d hit events, want 1", len(events.HitEvents))
		}
		hit := events.HitEvents[0]
		if hit.ShapeIdB != shapeIdOf(w, firstShape(w, boxId)) {
			t.Errorf("the hit event names the wrong shape")
		}
		if !w.hitEventThreshold.Less(hit.ApproachSpeed) {
			t.Errorf("approach speed = %v, want above the threshold %v", hit.ApproachSpeed, w.hitEventThreshold)
		}
		if !hit.Normal.Y.Eq(fixed.Q32One()) || !withinQ(hit.Point.Y, fixed.Q32Zero(), fixed.Q32MustParse("0.05")) {
			t.Errorf("hit normal %v at %v, want the ground normal near y = 0", hit.Normal, hit.Point)
		}
		validateWorld(w)
		return
	}
	t.Fatal("no hit event in 10 steps")
}

// TestStepOrdersEventsByContactId pins the event order: begin, hit and end
// events follow the contact id, which follows the creation order.
func TestStepOrdersEventsByContactId(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)

	groundDef := DefaultBodyDef()
	groundDef.Position = Vec2{Y: fixed.Q32Half().Neg()}
	groundId := CreateBody(worldId, &groundDef)
	shapeDef := DefaultShapeDef()
	shapeDef.EnableContactEvents = true
	shapeDef.EnableHitEvents = true
	ground := MakeBox(fixed.Q32FromInt(5), fixed.Q32Half())
	CreatePolygonShape(groundId, &shapeDef, &ground)

	unit := MakeBox(fixed.Q32Half(), fixed.Q32Half())
	// The right box is created first, so the creation order of the
	// contacts, not the position, decides the event order. Both start
	// inside the fat margin with their fall speed.
	var bodies [2]BodyId
	var shapes [2]ShapeId
	for i, x := range []int{3, 0} {
		boxDef := DefaultBodyDef()
		boxDef.Type = DynamicBody
		boxDef.Position = Vec2{X: fixed.Q32FromInt(x), Y: fixed.Q32MustParse("0.55")}
		bodies[i] = CreateBody(worldId, &boxDef)
		shapes[i] = CreatePolygonShape(bodies[i], &shapeDef, &unit)
		getBodyState(w, getBodyFullId(w, bodies[i])).linearVelocity = Vec2{Y: fixed.Q32FromInt(-3)}
	}
	for i := range bodies {
		createContact(w, firstShape(w, groundId), firstShape(w, bodies[i]))
	}

	dt := stepDt()
	events := GetContactEvents(worldId)
	for range 10 {
		Step(worldId, dt, 4)
		events = GetContactEvents(worldId)
		if len(events.BeginEvents) == 2 {
			break
		}
	}
	if len(events.BeginEvents) != 2 || events.BeginEvents[0].ShapeIdB != shapes[0] || events.BeginEvents[1].ShapeIdB != shapes[1] {
		t.Fatalf("the begin events do not follow the contact ids")
	}
	if len(events.HitEvents) != 2 || events.HitEvents[0].ShapeIdB != shapes[0] || events.HitEvents[1].ShapeIdB != shapes[1] {
		t.Fatalf("the hit events do not follow the contact ids: %d events", len(events.HitEvents))
	}

	// Lift both boxes out of the speculative margin.
	for i := range bodies {
		b := getBodyFullId(w, bodies[i])
		sim := getBodySim(w, b)
		sim.center.Y = sim.center.Y.Add(fixed.Q32One())
		sim.transform.P.Y = sim.transform.P.Y.Add(fixed.Q32One())
		getBodyState(w, b).linearVelocity = Vec2{Y: fixed.Q32FromInt(5)}
	}
	Step(worldId, dt, 4)
	events = GetContactEvents(worldId)
	if len(events.EndEvents) != 2 || events.EndEvents[0].ShapeIdB != shapes[0] || events.EndEvents[1].ShapeIdB != shapes[1] {
		t.Fatalf("the end events do not follow the contact ids: %d events", len(events.EndEvents))
	}
	validateWorld(w)
}

// TestSmallPyramidStaysStable pins the scene of the plan: a five-row pyramid
// keeps its top box in place for one hundred steps, falls asleep, passes
// every validator and yields the same checksum in a second build.
func TestSmallPyramidStaysStable(t *testing.T) {
	const rows = 5
	run := func() (uint64, Q, int) {
		worldId := createTestWorld(t)
		w := getWorldFromId(worldId)
		topId := buildPyramid(worldId, rows)
		createBruteForcePairs(w)
		dt := stepDt()
		for range 100 {
			Step(worldId, dt, 4)
		}
		validateWorld(w)
		top := getBodyFullId(w, topId)
		return Checksum(worldId), getBodySim(w, top).center.Y, top.setIndex
	}

	first, topY, setIndex := run()
	second, _, _ := run()
	if first != second {
		t.Errorf("two builds of the pyramid differ: %d and %d", first, second)
	}
	want := fixed.Q32Half().Mul(fixed.Q32FromInt(2*rows - 1))
	if !withinQ(topY, want, fixed.Q32MustParse("0.02")) {
		t.Errorf("the top box rests at y = %v, want %v", topY, want)
	}
	if setIndex < firstSleepingSet {
		t.Errorf("the pyramid is in set %d, want a sleeping set", setIndex)
	}
}
