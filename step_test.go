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
		worldId.Step(dt, subStepCount)
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
	worldId.Step(dt, 1)

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
	worldId.Step(dt, 1)

	state := getBodyState(w, b)
	if !state.angularVelocity.Eq(wantAngular) {
		t.Errorf("angular velocity = %v, want %v", state.angularVelocity, wantAngular)
	}
	if !b.sleepTime.Eq(fixed.Q32Zero()) {
		t.Errorf("sleep time = %v, want zero for the rotating body", b.sleepTime)
	}
}

// TestStepRefreshesFastBodyBoundsWithoutAHit pins the no-hit path of the
// continuous stage: the shape keeps the tight bounds of the end transform,
// as the reference does, the fat bounds still contain them, and the
// previous transform advances.
func TestStepRefreshesFastBodyBoundsWithoutAHit(t *testing.T) {
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
	worldId.Step(stepDt(), 4)

	b := getBodyFullId(w, bodyId)
	sim := getBodySim(w, b)
	if !sim.isFast {
		t.Fatal("the body is not marked fast")
	}
	if want := computeShapeAABB(s, sim.transform); s.aabb != want {
		t.Errorf("shape AABB = %+v, want %+v", s.aabb, want)
	}
	if s.aabb == before {
		t.Errorf("shape AABB did not move with the fast body")
	}
	if !AABBContains(s.fatAABB, s.aabb) {
		t.Errorf("fat AABB %+v does not contain %+v", s.fatAABB, s.aabb)
	}
	if sim.center0 != sim.center || sim.rotation0 != sim.transform.Q {
		t.Errorf("previous transform was not advanced")
	}
}

// TestStepRejectsInvalidInput covers the public assertions retained as
// panics in every build.
func TestStepRejectsInvalidInput(t *testing.T) {
	worldId := createTestWorld(t)

	t.Run("saturated time step", func(t *testing.T) {
		requirePanic(t, func() { worldId.Step(fixed.Q32MaxValue(), 4) })
	})
	t.Run("non-positive sub-step count", func(t *testing.T) {
		requirePanic(t, func() { worldId.Step(stepDt(), 0) })
	})
	t.Run("locked world", func(t *testing.T) {
		w := getWorldFromId(worldId)
		w.locked = true
		defer func() { w.locked = false }()
		requirePanic(t, func() { worldId.Step(stepDt(), 4) })
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
		worldId.Step(dt, 4)
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
	worldId.Step(dt, 4)
	if !moving.sleepTime.Eq(fixed.Q32Zero()) {
		t.Errorf("sleep time in motion = %v, want zero", moving.sleepTime)
	}
}

// TestStepAllocatesNothing pins the hot-path contract: after the first
// step that activates a contact, a step allocates nothing. The first step
// grows the graph colors, the arena and the event buffers once, as the
// reference does. Sleep stays off so the contact and the joint keep
// solving.
func TestStepAllocatesNothing(t *testing.T) {
	def := DefaultWorldDef()
	def.EnableSleep = false
	worldId := CreateWorld(&def)
	t.Cleanup(func() { DestroyWorld(worldId) })
	boxOnGround(t, worldId, fixed.Q32Zero())
	var boxIds [8]BodyId
	for i := range 8 {
		boxIds[i] = addDynamicBox(t, worldId, v2(10+i*3, 0))
	}
	jointDef := DefaultRevoluteJointDef()
	jointDef.BodyIdA, jointDef.BodyIdB = boxIds[0], boxIds[1]
	jointDef.LocalAnchorA = Vec2{X: fixed.Q32FromInt(2)}
	jointDef.LocalAnchorB = Vec2{X: fixed.Q32FromInt(-1)}
	CreateRevoluteJoint(worldId, &jointDef)
	w := getWorldFromId(worldId)

	dt := stepDt()
	worldId.Step(dt, 4)
	if len(w.constraintGraph.colors[1].contactSims) != 1 {
		t.Fatalf("the warm-up step did not activate the contact")
	}

	allocs := testing.AllocsPerRun(10, func() {
		worldId.Step(dt, 4)
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
		world1.Step(dt, 4)
		world2.Step(dt, 4)
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
		worldId.Step(dt, 4)
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
		worldId.Step(dt, 4)
	}
	if w.splitIslandId == nullIndex {
		t.Fatalf("the finalize did not pick the split candidate")
	}
	if bodyA.setIndex != awakeSet {
		t.Fatalf("the island slept with a pending split")
	}

	worldId.Step(dt, 4)
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
// contact events and the move events. The pair update of the broadphase
// creates every contact.

// boxOnGround builds a static ground with its top at y = 0 and a unit box
// of mass one above it. The box sits at the height y, so a positive height
// starts a fall.
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

	_ = w
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
		worldId.Step(dt, 4)
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
	worldId.Step(dt, 4)

	events := worldId.GetContactEvents()
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

	worldId.Step(dt, 4)
	if len(worldId.GetContactEvents().BeginEvents) != 0 {
		t.Errorf("the second step repeats the begin event")
	}

	// Lift the box out of the speculative margin. The bounds refresh on
	// the next finalize, so the contact survives that step and separates.
	sim := getBodySim(w, box)
	lift := fixed.Q32One()
	sim.center.Y = sim.center.Y.Add(lift)
	sim.transform.P.Y = sim.transform.P.Y.Add(lift)
	getBodyState(w, box).linearVelocity = Vec2{Y: fixed.Q32FromInt(5)}
	worldId.Step(dt, 4)

	events = worldId.GetContactEvents()
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
	worldId.Step(dt, 4)
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
	worldId.Step(dt, 4)
	worldId.Step(dt, 4)

	if w.contactIdPool.idCount() != 0 || w.broadPhase.pairSet.count != 0 {
		t.Errorf("the disjoint contact survives: %d contacts and %d pairs", w.contactIdPool.idCount(), w.broadPhase.pairSet.count)
	}
	if len(worldId.GetContactEvents().EndEvents) != 0 {
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
	worldId.Step(dt, 4)
	events := worldId.GetBodyEvents()
	if len(events.MoveEvents) != 1 {
		t.Fatalf("the step reports %d move events, want 1", len(events.MoveEvents))
	}
	move := events.MoveEvents[0]
	if move.BodyId != restingId || move.FellAsleep || move.Transform.P != v2(3, 0) {
		t.Errorf("the move event is %+v", move)
	}

	for range 30 {
		worldId.Step(dt, 4)
	}
	events = worldId.GetBodyEvents()
	if len(events.MoveEvents) != 1 || !events.MoveEvents[0].FellAsleep {
		t.Fatalf("the sleep step reports %+v", events.MoveEvents)
	}
	if resting.setIndex < firstSleepingSet {
		t.Errorf("the body is in set %d, want a sleeping set", resting.setIndex)
	}

	worldId.Step(dt, 4)
	if len(worldId.GetBodyEvents().MoveEvents) != 0 {
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

	worldId.Step(stepDt(), 4)
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

// TestStepReportsAHitEvent pins the hit event: a box that falls from half
// a metre and lands faster than the threshold reports one hit on the
// contact normal. The broadphase creates the contact on the way down.
func TestStepReportsAHitEvent(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	boxId := boxOnGround(t, worldId, fixed.Q32Half())
	getBodyState(w, getBodyFullId(w, boxId)).linearVelocity = Vec2{Y: fixed.Q32FromInt(-3)}

	dt := stepDt()
	for range 30 {
		worldId.Step(dt, 4)
		events := worldId.GetContactEvents()
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
	t.Fatal("no hit event in 30 steps")
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
	// contacts, not the position, decides the event order. Both fall from
	// half a metre with the same speed, so the broadphase creates their
	// contacts in one step, in body order.
	var bodies [2]BodyId
	var shapes [2]ShapeId
	for i, x := range []int{3, 0} {
		boxDef := DefaultBodyDef()
		boxDef.Type = DynamicBody
		boxDef.Position = Vec2{X: fixed.Q32FromInt(x), Y: fixed.Q32One()}
		bodies[i] = CreateBody(worldId, &boxDef)
		shapes[i] = CreatePolygonShape(bodies[i], &shapeDef, &unit)
		getBodyState(w, getBodyFullId(w, bodies[i])).linearVelocity = Vec2{Y: fixed.Q32FromInt(-3)}
	}

	dt := stepDt()
	events := worldId.GetContactEvents()
	for range 30 {
		worldId.Step(dt, 4)
		events = worldId.GetContactEvents()
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
	worldId.Step(dt, 4)
	events = worldId.GetContactEvents()
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
		dt := stepDt()
		for range 100 {
			worldId.Step(dt, 4)
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

// TestStepLandsABoxOnAChainSegment runs the chain segment against polygon
// pair through the whole pipeline: the box falls on a chain floor, rests
// on it and the world stays valid.
func TestStepLandsABoxOnAChainSegment(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)

	// The chain normal points to the right of p1->p2, so the floor runs
	// from right to left to face up.
	groundDef := DefaultBodyDef()
	groundId := CreateBody(worldId, &groundDef)
	ground := getBodyFullId(w, groundId)
	shapeDef := DefaultShapeDef()
	floor := ChainSegment{
		Ghost1:  Vec2{X: fixed.Q32FromInt(6)},
		Segment: Segment{Point1: Vec2{X: fixed.Q32FromInt(5)}, Point2: Vec2{X: fixed.Q32FromInt(-5)}},
		Ghost2:  Vec2{X: fixed.Q32FromInt(-6)},
	}
	createShapeInternal(w, ground, getBodyTransformQuick(w, ground), &shapeDef, &floor, ChainSegmentShape)

	boxId := addDynamicBox(t, worldId, Vec2{Y: fixed.Q32FromInt(2)})
	box := getBodyFullId(w, boxId)

	dt := stepDt()
	for range 120 {
		worldId.Step(dt, 4)
		validateWorld(w)
	}

	// The helper box has a half extent of one, so it rests with its
	// center one unit above the floor.
	sim := getBodySim(w, box)
	tolerance := fixed.Q32MustParse("0.01")
	if !sim.center.Y.Sub(fixed.Q32One()).Abs().Less(tolerance) {
		t.Fatalf("box center %v, want y near 1", sim.center)
	}
	if box.contactCount != 1 {
		t.Fatalf("contact count %d, want 1", box.contactCount)
	}
}

// continuousWorld builds a world without gravity so a fast body keeps
// its line of flight.
func continuousWorld(t *testing.T, enableContinuous bool) WorldId {
	t.Helper()
	def := DefaultWorldDef()
	def.Gravity = Vec2Zero()
	def.EnableContinuous = enableContinuous
	worldId := CreateWorld(&def)
	t.Cleanup(func() { DestroyWorld(worldId) })
	return worldId
}

// addPlate creates a body with a thin box: a tenth of a metre wide and
// two metres tall.
func addPlate(t *testing.T, worldId WorldId, bodyType BodyType, x int) BodyId {
	t.Helper()
	bodyDef := DefaultBodyDef()
	bodyDef.Type = bodyType
	bodyDef.Position = v2(x, 0)
	bodyId := CreateBody(worldId, &bodyDef)
	shapeDef := DefaultShapeDef()
	plate := MakeBox(fixed.Q32FromRatio(1, 20), fixed.Q32One())
	CreatePolygonShape(bodyId, &shapeDef, &plate)
	return bodyId
}

// addProjectile creates a small dynamic box that crosses more than three
// metres per step, well over half its extent.
func addProjectile(t *testing.T, worldId WorldId, isBullet bool) BodyId {
	t.Helper()
	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = Vec2{X: fixed.Q32MustParse("3.5")}
	bodyDef.LinearVelocity = v2(200, 0)
	bodyDef.IsBullet = isBullet
	bodyId := CreateBody(worldId, &bodyDef)
	shapeDef := DefaultShapeDef()
	box := MakeSquare(fixed.Q32FromRatio(1, 10))
	CreatePolygonShape(bodyId, &shapeDef, &box)
	return bodyId
}

func bodyX(worldId WorldId, bodyId BodyId) Q {
	w := getWorldFromId(worldId)
	return getBodyTransformQuick(w, getBodyFullId(w, bodyId)).P.X
}

// TestStepStopsAFastBodyAtAStaticPlate pins the continuous stage: a fast
// box that would cross a thin static plate in one step stops at its face
// instead. With the stage disabled the same box tunnels through.
func TestStepStopsAFastBodyAtAStaticPlate(t *testing.T) {
	// The plate face sits at x = 4.95 and the box half extent is 0.1.
	face := fixed.Q32MustParse("4.85")

	t.Run("continuous", func(t *testing.T) {
		worldId := continuousWorld(t, true)
		addPlate(t, worldId, StaticBody, 5)
		boxId := addProjectile(t, worldId, false)

		worldId.Step(stepDt(), 4)
		w := getWorldFromId(worldId)
		validateWorld(w)

		x := bodyX(worldId, boxId)
		if !x.Less(face) || !fixed.Q32MustParse("4.8").Less(x) {
			t.Fatalf("x %v, want just under %v", x, face)
		}
		if sim := getBodySim(w, getBodyFullId(w, boxId)); !sim.isFast {
			t.Fatal("the box is not marked fast")
		}

		// The next step lands the speculative contact and the box rests.
		worldId.Step(stepDt(), 4)
		validateWorld(w)
		if x := bodyX(worldId, boxId); !x.Less(face) {
			t.Fatalf("x %v after the second step, want under %v", x, face)
		}
	})

	t.Run("discrete", func(t *testing.T) {
		worldId := continuousWorld(t, false)
		addPlate(t, worldId, StaticBody, 5)
		boxId := addProjectile(t, worldId, false)

		worldId.Step(stepDt(), 4)

		if x := bodyX(worldId, boxId); !fixed.Q32FromInt(6).Less(x) {
			t.Fatalf("x %v, want past the plate", x)
		}
	})
}

// TestStepBulletStopsAtADynamicPlate pins the bullet path: only a bullet
// sweeps the dynamic tree, so a plain fast body passes a dynamic plate
// while a bullet stops at it and pushes it.
func TestStepBulletStopsAtADynamicPlate(t *testing.T) {
	t.Run("bullet", func(t *testing.T) {
		worldId := continuousWorld(t, true)
		plateId := addPlate(t, worldId, DynamicBody, 5)
		boxId := addProjectile(t, worldId, true)

		worldId.Step(stepDt(), 4)
		w := getWorldFromId(worldId)
		validateWorld(w)

		x := bodyX(worldId, boxId)
		if !x.Less(fixed.Q32MustParse("4.85")) || !fixed.Q32MustParse("4.8").Less(x) {
			t.Fatalf("x %v, want just under 4.85", x)
		}

		// The contact forms on the next step and the plate takes the hit.
		worldId.Step(stepDt(), 4)
		worldId.Step(stepDt(), 4)
		validateWorld(w)
		if plateX := bodyX(worldId, plateId); !fixed.Q32FromInt(5).Less(plateX) {
			t.Fatalf("plate x %v, want pushed past 5", plateX)
		}
		if x := bodyX(worldId, boxId); !x.Less(bodyX(worldId, plateId)) {
			t.Fatalf("box x %v passed the plate", x)
		}
	})

	t.Run("not a bullet", func(t *testing.T) {
		worldId := continuousWorld(t, true)
		addPlate(t, worldId, DynamicBody, 5)
		boxId := addProjectile(t, worldId, false)

		worldId.Step(stepDt(), 4)

		if x := bodyX(worldId, boxId); !fixed.Q32FromInt(6).Less(x) {
			t.Fatalf("x %v, want past the plate", x)
		}
	})
}

// TestStepFastBodyCrossesAChainJunction pins the side test of the
// continuous stage: a fast box that lands on a chain from above stops at
// the floor, then slides across the junction of two collinear segments.
// A wrong sign in the side test lets the box tunnel through the floor.
func TestStepFastBodyCrossesAChainJunction(t *testing.T) {
	worldId := continuousWorld(t, true)
	w := getWorldFromId(worldId)

	// The chain normal points to the right of p1->p2, so each floor
	// segment runs from right to left to face up. The junction is x = 8.
	groundDef := DefaultBodyDef()
	groundId := CreateBody(worldId, &groundDef)
	ground := getBodyFullId(w, groundId)
	shapeDef := DefaultShapeDef()
	xf := getBodyTransformQuick(w, ground)
	right := ChainSegment{
		Ghost1:  Vec2{X: fixed.Q32FromInt(30)},
		Segment: Segment{Point1: Vec2{X: fixed.Q32FromInt(20)}, Point2: Vec2{X: fixed.Q32FromInt(8)}},
		Ghost2:  Vec2{X: fixed.Q32FromInt(-10)},
	}
	left := ChainSegment{
		Ghost1:  Vec2{X: fixed.Q32FromInt(20)},
		Segment: Segment{Point1: Vec2{X: fixed.Q32FromInt(8)}, Point2: Vec2{X: fixed.Q32FromInt(-10)}},
		Ghost2:  Vec2{X: fixed.Q32FromInt(-20)},
	}
	createShapeInternal(w, ground, xf, &shapeDef, &right, ChainSegmentShape)
	createShapeInternal(w, ground, xf, &shapeDef, &left, ChainSegmentShape)

	// The box has a half extent of 0.1. It crosses more than three metres
	// and falls one metre per step, so it reaches the floor two metres
	// before the junction.
	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = Vec2{X: fixed.Q32FromInt(6), Y: fixed.Q32MustParse("0.6")}
	bodyDef.LinearVelocity = v2(200, -60)
	boxId := CreateBody(worldId, &bodyDef)
	boxShape := MakeSquare(fixed.Q32FromRatio(1, 10))
	CreatePolygonShape(boxId, &shapeDef, &boxShape)
	box := getBodyFullId(w, boxId)

	junction := fixed.Q32FromInt(8)
	for range 3 {
		worldId.Step(stepDt(), 4)
		validateWorld(w)
	}

	if x := bodyX(worldId, boxId); !junction.Less(x) {
		t.Fatalf("x %v, want past the junction at %v", x, junction)
	}
	if y := getBodyTransformQuick(w, box).P.Y; !fixed.Q32Zero().Less(y) {
		t.Fatalf("y %v, want the box above the floor", y)
	}
	state := getBodyState(w, box)
	if !fixed.Q32FromInt(100).Less(state.linearVelocity.X) {
		t.Fatalf("velocity x %v, want the box still moving", state.linearVelocity.X)
	}
}

// TestStepSwingsAPendulum pins the joint stages inside Step: a box on a
// revolute joint one unit from a static pivot swings under gravity, the
// arm keeps its length within two linear slops on every step, and no
// operation saturates.
func TestStepSwingsAPendulum(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)

	pivotDef := DefaultBodyDef()
	pivotId := CreateBody(worldId, &pivotDef)

	bobDef := DefaultBodyDef()
	bobDef.Type = DynamicBody
	bobDef.Position = Vec2{X: fixed.Q32One()}
	bobId := CreateBody(worldId, &bobDef)
	shapeDef := DefaultShapeDef()
	bob := MakeSquare(fixed.Q32MustParse("0.1"))
	CreatePolygonShape(bobId, &shapeDef, &bob)

	def := DefaultRevoluteJointDef()
	def.BodyIdA = pivotId
	def.BodyIdB = bobId
	def.LocalAnchorB = Vec2{X: fixed.Q32One().Neg()}
	CreateRevoluteJoint(worldId, &def)

	fixed.ResetSaturationCount()
	body := getBodyFullId(w, bobId)
	slack := linearSlop.Add(linearSlop)
	one := fixed.Q32One()
	lowest := fixed.Q32Zero()
	dt := stepDt()
	for i := range 120 {
		worldId.Step(dt, 4)
		center := getBodySim(w, body).center
		arm := center.Len()
		if !withinQ(arm, one, slack) {
			t.Fatalf("step %d: the arm is %v long, want 1", i, arm)
		}
		lowest = lowest.Min(center.Y)
	}
	if !lowest.Less(fixed.Q32MustParse("-0.9")) {
		t.Errorf("the bob only fell to y = %v", lowest)
	}
	if n := fixed.SaturationCount(); n != 0 {
		t.Errorf("%d operations saturated", n)
	}
	validateWorld(w)
}

// TestStepPullsTheRopeTight pins the distance joint inside Step: two
// circles two units apart on a rigid rope of length one converge to one
// unit and no operation saturates.
func TestStepPullsTheRopeTight(t *testing.T) {
	worldDef := DefaultWorldDef()
	worldDef.Gravity = Vec2Zero()
	worldId := CreateWorld(&worldDef)
	t.Cleanup(func() { DestroyWorld(worldId) })
	w := getWorldFromId(worldId)

	idA := addDynamicCircle(t, worldId, v2(0, 0))
	idB := addDynamicCircle(t, worldId, v2(2, 0))
	def := DefaultDistanceJointDef()
	def.BodyIdA = idA
	def.BodyIdB = idB
	def.Length = fixed.Q32One()
	CreateDistanceJoint(worldId, &def)

	fixed.ResetSaturationCount()
	bodyA := getBodyFullId(w, idA)
	bodyB := getBodyFullId(w, idB)
	dt := stepDt()
	for range 60 {
		worldId.Step(dt, 4)
	}
	gap := getBodySim(w, bodyB).center.Sub(getBodySim(w, bodyA).center).Len()
	if !withinQ(gap, fixed.Q32One(), linearSlop.Add(linearSlop)) {
		t.Errorf("the rope is %v long, want 1", gap)
	}
	if n := fixed.SaturationCount(); n != 0 {
		t.Errorf("%d operations saturated", n)
	}
	validateWorld(w)
}

// TestStepSlidesToTheStop pins the prismatic joint inside Step: a box on
// the x axis of a static ground, limited to [0, 1], slides under a lateral
// gravity and stops at one, still on the axis, and no operation saturates.
func TestStepSlidesToTheStop(t *testing.T) {
	worldDef := DefaultWorldDef()
	worldDef.Gravity = Vec2{X: fixed.Q32FromInt(10)}
	worldId := CreateWorld(&worldDef)
	t.Cleanup(func() { DestroyWorld(worldId) })
	w := getWorldFromId(worldId)

	groundDef := DefaultBodyDef()
	groundId := CreateBody(worldId, &groundDef)
	boxId := addDynamicCircle(t, worldId, v2(0, 0))

	def := DefaultPrismaticJointDef()
	def.BodyIdA = groundId
	def.BodyIdB = boxId
	def.EnableLimit = true
	def.LowerTranslation = fixed.Q32Zero()
	def.UpperTranslation = fixed.Q32One()
	CreatePrismaticJoint(worldId, &def)

	fixed.ResetSaturationCount()
	body := getBodyFullId(w, boxId)
	slack := linearSlop.Add(linearSlop)
	dt := stepDt()
	for range 120 {
		worldId.Step(dt, 4)
	}
	center := getBodySim(w, body).center
	if !withinQ(center.X, fixed.Q32One(), slack) || !withinQ(center.Y, fixed.Q32Zero(), slack) {
		t.Errorf("the box rests at %v, want (1, 0)", center)
	}
	if n := fixed.SaturationCount(); n != 0 {
		t.Errorf("%d operations saturated", n)
	}
	validateWorld(w)
}

// TestStepSettlesTheSuspension pins the wheel joint inside Step: a circle
// half a unit above the ground origin, on a vertical spring of two hertz
// without gravity, settles back onto the anchor within two linear slops,
// and no operation saturates.
func TestStepSettlesTheSuspension(t *testing.T) {
	worldDef := DefaultWorldDef()
	worldDef.Gravity = Vec2Zero()
	worldId := CreateWorld(&worldDef)
	t.Cleanup(func() { DestroyWorld(worldId) })
	w := getWorldFromId(worldId)

	groundDef := DefaultBodyDef()
	groundId := CreateBody(worldId, &groundDef)
	wheelId := addDynamicCircle(t, worldId, Vec2{Y: fixed.Q32Half()})

	def := DefaultWheelJointDef()
	def.BodyIdA = groundId
	def.BodyIdB = wheelId
	def.Hertz = fixed.Q32FromInt(2)
	CreateWheelJoint(worldId, &def)

	fixed.ResetSaturationCount()
	body := getBodyFullId(w, wheelId)
	slack := linearSlop.Add(linearSlop)
	dt := stepDt()
	for range 240 {
		worldId.Step(dt, 4)
	}
	center := getBodySim(w, body).center
	if !withinQ(center.X, fixed.Q32Zero(), slack) || !withinQ(center.Y, fixed.Q32Zero(), slack) {
		t.Errorf("the wheel rests at %v, want (0, 0)", center)
	}
	if n := fixed.SaturationCount(); n != 0 {
		t.Errorf("%d operations saturated", n)
	}
	validateWorld(w)
}

// TestStepWeldsTwoBoxes pins the weld joint inside Step: two small boxes
// welded side by side fall together while the second one starts with a
// spin. After 60 steps the relative angle is below 1e-4 turn, the centers
// stay one unit apart within two linear slops, and no operation saturates.
func TestStepWeldsTwoBoxes(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)

	makeBox := func(position Vec2) BodyId {
		def := DefaultBodyDef()
		def.Type = DynamicBody
		def.Position = position
		id := CreateBody(worldId, &def)
		shapeDef := DefaultShapeDef()
		box := MakeSquare(fixed.Q32MustParse("0.25"))
		CreatePolygonShape(id, &shapeDef, &box)
		return id
	}
	idA := makeBox(v2(0, 0))
	idB := makeBox(v2(1, 0))

	def := DefaultWeldJointDef()
	def.BodyIdA = idA
	def.BodyIdB = idB
	def.LocalAnchorA = Vec2{X: fixed.Q32Half()}
	def.LocalAnchorB = Vec2{X: fixed.Q32Half().Neg()}
	CreateWeldJoint(worldId, &def)

	bodyA := getBodyFullId(w, idA)
	bodyB := getBodyFullId(w, idB)
	getBodyState(w, bodyB).angularVelocity = fixed.Q32Half()

	fixed.ResetSaturationCount()
	dt := stepDt()
	for range 60 {
		worldId.Step(dt, 4)
	}
	simA := getBodySim(w, bodyA)
	simB := getBodySim(w, bodyB)
	angle := RelativeAngle(simB.transform.Q, simA.transform.Q).Abs()
	if !angle.Less(fixed.Q32MustParse("0.0001")) {
		t.Errorf("the relative angle is %v turn, want below 1e-4", angle)
	}
	gap := simB.center.Sub(simA.center).Len()
	if !withinQ(gap, fixed.Q32One(), linearSlop.Add(linearSlop)) {
		t.Errorf("the centers are %v apart, want 1", gap)
	}
	if n := fixed.SaturationCount(); n != 0 {
		t.Errorf("%d operations saturated", n)
	}
	validateWorld(w)
}

// TestStepDrivesToTheOffset pins the motor joint inside Step: a circle on
// a motor joint to the ground with the linear offset (1, 0) and the
// default correction factor converges to x = 1 within two linear slops in
// 120 steps, and no operation saturates.
func TestStepDrivesToTheOffset(t *testing.T) {
	worldDef := DefaultWorldDef()
	worldDef.Gravity = Vec2Zero()
	worldId := CreateWorld(&worldDef)
	t.Cleanup(func() { DestroyWorld(worldId) })
	w := getWorldFromId(worldId)

	groundDef := DefaultBodyDef()
	groundId := CreateBody(worldId, &groundDef)
	circleId := addDynamicCircle(t, worldId, v2(0, 0))

	def := DefaultMotorJointDef()
	def.BodyIdA = groundId
	def.BodyIdB = circleId
	def.LinearOffset = Vec2{X: fixed.Q32One()}
	def.MaxForce = fixed.Q32FromInt(1000)
	def.MaxTorque = fixed.Q32FromInt(1000)
	CreateMotorJoint(worldId, &def)

	fixed.ResetSaturationCount()
	body := getBodyFullId(w, circleId)
	dt := stepDt()
	for range 120 {
		worldId.Step(dt, 4)
	}
	center := getBodySim(w, body).center
	slack := linearSlop.Add(linearSlop)
	if !withinQ(center.X, fixed.Q32One(), slack) || !withinQ(center.Y, fixed.Q32Zero(), slack) {
		t.Errorf("the circle rests at %v, want (1, 0)", center)
	}
	if n := fixed.SaturationCount(); n != 0 {
		t.Errorf("%d operations saturated", n)
	}
	validateWorld(w)
}

// TestStepDragsToTheTarget pins the mouse joint inside Step: a five hertz
// mouse joint grabs a circle at its center, the target moves to (1, 0),
// and the circle gets within 0.05 of it in 60 steps without saturation.
func TestStepDragsToTheTarget(t *testing.T) {
	worldDef := DefaultWorldDef()
	worldDef.Gravity = Vec2Zero()
	worldId := CreateWorld(&worldDef)
	t.Cleanup(func() { DestroyWorld(worldId) })
	w := getWorldFromId(worldId)

	groundDef := DefaultBodyDef()
	groundId := CreateBody(worldId, &groundDef)
	circleId := addDynamicCircle(t, worldId, v2(0, 0))

	def := DefaultMouseJointDef()
	def.BodyIdA = groundId
	def.BodyIdB = circleId
	def.Hertz = fixed.Q32FromInt(5)
	def.MaxForce = fixed.Q32FromInt(1000)
	jointId := CreateMouseJoint(worldId, &def)
	target := Vec2{X: fixed.Q32One()}
	getJointSim(w, getJointFullId(w, jointId)).mouseJoint.targetA = target

	fixed.ResetSaturationCount()
	body := getBodyFullId(w, circleId)
	dt := stepDt()
	for range 60 {
		worldId.Step(dt, 4)
	}
	center := getBodySim(w, body).center
	if gap := center.Sub(target).Len(); !gap.Less(fixed.Q32MustParse("0.05")) {
		t.Errorf("the circle rests at %v, %v from the target", center, gap)
	}
	if n := fixed.SaturationCount(); n != 0 {
		t.Errorf("%d operations saturated", n)
	}
	validateWorld(w)
}
