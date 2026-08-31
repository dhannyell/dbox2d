package dbox2d

import (
	"testing"

	"github.com/dhannyell/fixed"
)

// This file tests the Step pipeline: exact integration, sleep tracking, the
// zero-allocation contract and bit-for-bit reproducibility.

// stepDt returns the recommended fixed time step of 1/60 second.
func stepDt() Q {
	return fixed.One().Div(fixed.FromInt(60))
}

// addDynamicBox creates a dynamic body with a unit box at the position.
func addDynamicBox(t *testing.T, worldId WorldId, position Vec2) BodyId {
	t.Helper()
	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = position
	bodyId := CreateBody(worldId, &bodyDef)
	shapeDef := DefaultShapeDef()
	box := MakeSquare(fixed.One())
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

	h := dt.Div(fixed.FromInt(subStepCount))
	gravity := worldId.Gravity()

	// The scalar mirror of the loop: velocity gains h*g per sub-step and the
	// position gains h*v after each velocity update.
	wantVelocityY := fixed.Zero()
	wantPositionY := bodyId.Position().Y
	for range stepCount {
		deltaY := fixed.Zero()
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
	if !bodyId.Position().X.Eq(fixed.Zero()) {
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
	bodyDef.LinearVelocity = Vec2{X: fixed.FromInt(12), Y: fixed.FromInt(-7)}
	bodyDef.AngularVelocity = fixed.MustParse("0.25")
	bodyDef.LinearDamping = fixed.MustParse("0.4")
	bodyDef.AngularDamping = fixed.MustParse("0.7")
	bodyId := CreateBody(worldId, &bodyDef)
	shapeDef := DefaultShapeDef()
	box := MakeSquare(fixed.One())
	CreatePolygonShape(bodyId, &shapeDef, &box)

	w := getWorldFromId(worldId)
	b := getBodyFullId(w, bodyId)
	initialState := *getBodyState(w, b)
	dt := stepDt()
	Step(worldId, dt, 1)

	state := getBodyState(w, b)
	linearDenominator := fixed.One().Add(dt.Mul(bodyDef.LinearDamping))
	angularDenominator := fixed.One().Add(dt.Mul(bodyDef.AngularDamping))
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
	bodyDef.AngularVelocity = fixed.MustParse("0.02")
	bodyDef.SleepThreshold = fixed.MustParse("0.1")
	bodyId := CreateBody(worldId, &bodyDef)
	shapeDef := DefaultShapeDef()
	box := MakeSquare(fixed.One())
	CreatePolygonShape(bodyId, &shapeDef, &box)

	w := getWorldFromId(worldId)
	b := getBodyFullId(w, bodyId)
	sim := getBodySim(w, b)
	sim.torque = tau
	b.sleepTime = fixed.One()

	dt := stepDt()
	wantAngular := bodyDef.AngularVelocity.Add(dt.Mul(sim.invInertia).Mul(sim.torque).Div(tau))
	Step(worldId, dt, 1)

	state := getBodyState(w, b)
	if !state.angularVelocity.Eq(wantAngular) {
		t.Errorf("angular velocity = %v, want %v", state.angularVelocity, wantAngular)
	}
	if !b.sleepTime.Eq(fixed.Zero()) {
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
	bodyDef.LinearVelocity = Vec2{X: fixed.FromInt(100)}
	bodyId := CreateBody(worldId, &bodyDef)
	shapeDef := DefaultShapeDef()
	box := MakeSquare(fixed.One())
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
		requirePanic(t, func() { Step(worldId, fixed.MaxValue(), 4) })
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
	wantSleep := dt.Mul(fixed.FromInt(stepCount))
	if !resting.sleepTime.Eq(wantSleep) {
		t.Errorf("sleep time at rest = %v, want %v", resting.sleepTime, wantSleep)
	}

	// A push above the sleep threshold resets the sleep time.
	moving := getBodyFullId(w, movingId)
	getBodyState(w, moving).linearVelocity = Vec2{X: fixed.One()}
	Step(worldId, dt, 4)
	if !moving.sleepTime.Eq(fixed.Zero()) {
		t.Errorf("sleep time in motion = %v, want zero", moving.sleepTime)
	}
}

// TestStepAllocatesNothing pins the hot-path contract: a step allocates
// nothing after the world is built.
func TestStepAllocatesNothing(t *testing.T) {
	worldId := createTestWorld(t)
	for i := range 8 {
		addDynamicBox(t, worldId, v2(i*3, 0))
	}

	dt := stepDt()
	Step(worldId, dt, 4)

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
			getBodyState(w, b).angularVelocity = fixed.MustParse("0.1")
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
