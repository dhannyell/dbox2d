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
