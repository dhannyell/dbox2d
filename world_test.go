package dbox2d

import (
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
