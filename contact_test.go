package dbox2d

import (
	"testing"

	"github.com/dhannyell/fixed"
)

// addDynamicCircle creates a dynamic body with a circle of radius one half.
func addDynamicCircle(t *testing.T, worldId WorldId, position Vec2) BodyId {
	t.Helper()
	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = position
	bodyId := CreateBody(worldId, &bodyDef)
	shapeDef := DefaultShapeDef()
	circle := Circle{Radius: fixed.Q32Half()}
	CreateCircleShape(bodyId, &shapeDef, &circle)
	return bodyId
}

// firstShape returns the shape at the head of the shape list of a body.
func firstShape(w *world, bodyId BodyId) *shape {
	b := getBodyFullId(w, bodyId)
	return &w.shapes[b.headShapeId]
}

// TestCreateContactLinksBothBodies pins the bookkeeping of one contact:
// the set placement, the edge links, the body counters and the pair set.
func TestCreateContactLinksBothBodies(t *testing.T) {
	worldId := createTestWorld(t)
	idA := addDynamicCircle(t, worldId, v2(0, 0))
	idB := addDynamicCircle(t, worldId, v2(1, 0))

	w := getWorldFromId(worldId)
	shapeA := firstShape(w, idA)
	shapeB := firstShape(w, idB)

	createContact(w, shapeA, shapeB)

	c := &w.contacts[0]
	if c.setIndex != awakeSet || c.localIndex != 0 {
		t.Fatalf("contact set %d local %d, want the awake set at zero", c.setIndex, c.localIndex)
	}
	if c.edges[0].bodyId != shapeA.bodyId || c.edges[1].bodyId != shapeB.bodyId {
		t.Errorf("the edges do not point at the bodies of the shapes")
	}

	bodyA := getBodyFullId(w, idA)
	bodyB := getBodyFullId(w, idB)
	if bodyA.contactCount != 1 || bodyB.contactCount != 1 {
		t.Errorf("contact counts %d and %d, want 1 and 1", bodyA.contactCount, bodyB.contactCount)
	}
	if bodyA.headContactKey != 0 || bodyB.headContactKey != 1 {
		t.Errorf("head keys %d and %d, want 0 and 1", bodyA.headContactKey, bodyB.headContactKey)
	}

	if !w.broadPhase.pairSet.containsKey(shapePairKey(shapeA.id, shapeB.id)) {
		t.Errorf("the pair set does not have the shape pair")
	}

	cs := getContactSim(w, c)
	if cs.contactId != c.contactId || cs.shapeIdA != shapeA.id || cs.shapeIdB != shapeB.id {
		t.Errorf("the contact sim ids do not match the contact")
	}
	if cs.manifold.PointCount != 0 {
		t.Errorf("a new contact starts with a manifold")
	}
}

// TestCreateContactFlipsReversedTypes checks that the dispatch table stores a
// mixed pair in the primary order expected by its manifold function.
func TestCreateContactFlipsReversedTypes(t *testing.T) {
	worldId := createTestWorld(t)
	circleBody := addDynamicCircle(t, worldId, v2(0, 0))
	polygonBody := addDynamicBox(t, worldId, v2(1, 0))
	w := getWorldFromId(worldId)
	circle := firstShape(w, circleBody)
	polygon := firstShape(w, polygonBody)

	createContact(w, circle, polygon)

	c := &w.contacts[0]
	if c.shapeIdA != polygon.id || c.shapeIdB != circle.id {
		t.Fatal("the reversed pair was not stored in primary dispatch order")
	}
}

// TestCreateContactUsesDisabledSetForSleepingBodies checks the storage rule
// for a non-touching contact whose bodies are both asleep.
func TestCreateContactUsesDisabledSetForSleepingBodies(t *testing.T) {
	worldId := createTestWorld(t)
	addSleepingCircle := func(position Vec2) BodyId {
		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody
		bodyDef.Position = position
		bodyDef.IsAwake = false
		bodyId := CreateBody(worldId, &bodyDef)
		shapeDef := DefaultShapeDef()
		circle := Circle{Radius: fixed.Q32Half()}
		CreateCircleShape(bodyId, &shapeDef, &circle)
		return bodyId
	}

	idA := addSleepingCircle(v2(0, 0))
	idB := addSleepingCircle(v2(1, 0))
	w := getWorldFromId(worldId)
	createContact(w, firstShape(w, idA), firstShape(w, idB))

	c := &w.contacts[0]
	if c.setIndex != disabledSet || len(w.solverSets[disabledSet].contactSims) != 1 {
		t.Fatal("the sleeping contact is not stored in the disabled set")
	}
}

// TestUpdateContactCarriesTheStoredImpulse pins the id matching: the second
// update finds the point of the first update and copies its impulse into
// the warm start.
func TestUpdateContactCarriesTheStoredImpulse(t *testing.T) {
	worldId := createTestWorld(t)
	idA := addDynamicCircle(t, worldId, v2(0, 0))
	idB := addDynamicCircle(t, worldId, Vec2{X: fixed.Q32MustParse("0.75")})

	w := getWorldFromId(worldId)
	shapeA := firstShape(w, idA)
	shapeB := firstShape(w, idB)
	createContact(w, shapeA, shapeB)

	c := &w.contacts[0]
	cs := getContactSim(w, c)
	xfA := getBodyTransformQuick(w, getBodyFullId(w, idA))
	xfB := getBodyTransformQuick(w, getBodyFullId(w, idB))
	if temporary := computeManifold(shapeA, xfA, shapeB, xfB); temporary.PointCount != 1 {
		t.Fatalf("temporary manifold point count %d, want 1", temporary.PointCount)
	}

	if !updateContact(w, cs, shapeA, xfA, Vec2Zero(), shapeB, xfB, Vec2Zero()) {
		t.Fatal("the overlapping circles do not touch")
	}
	if cs.simFlags&simTouchingFlag == 0 {
		t.Errorf("the touching sim flag is not set")
	}
	if cs.manifold.PointCount != 1 {
		t.Fatalf("point count %d, want 1", cs.manifold.PointCount)
	}
	if cs.manifold.Points[0].Persisted {
		t.Errorf("the first update reports a persisted point")
	}

	cs.manifold.Points[0].NormalImpulse = fixed.Q32One()
	updateContact(w, cs, shapeA, xfA, Vec2Zero(), shapeB, xfB, Vec2Zero())

	p := &cs.manifold.Points[0]
	if !p.Persisted {
		t.Errorf("the matched point is not persisted")
	}
	if !p.NormalImpulse.Eq(fixed.Q32One()) {
		t.Errorf("the stored impulse did not carry over")
	}
}

// TestDestroyContactRepairsTheMovedSim removes the first of two contacts, so
// the swap moves the second sim and its contact must follow.
func TestDestroyContactRepairsTheMovedSim(t *testing.T) {
	worldId := createTestWorld(t)
	idA := addDynamicCircle(t, worldId, v2(0, 0))
	idB := addDynamicCircle(t, worldId, v2(1, 0))
	idC := addDynamicCircle(t, worldId, v2(2, 0))

	w := getWorldFromId(worldId)
	shapeA := firstShape(w, idA)
	shapeB := firstShape(w, idB)
	shapeC := firstShape(w, idC)
	createContact(w, shapeA, shapeB)
	createContact(w, shapeB, shapeC)

	destroyContact(w, &w.contacts[0], false)

	if w.broadPhase.pairSet.containsKey(shapePairKey(shapeA.id, shapeB.id)) {
		t.Errorf("the destroyed pair is still in the pair set")
	}

	moved := &w.contacts[1]
	if moved.localIndex != 0 {
		t.Errorf("the moved contact local index is %d, want 0", moved.localIndex)
	}
	cs := getContactSim(w, moved)
	if cs.contactId != moved.contactId {
		t.Errorf("the moved sim does not belong to the moved contact")
	}

	bodyA := getBodyFullId(w, idA)
	bodyB := getBodyFullId(w, idB)
	if bodyA.contactCount != 0 || bodyA.headContactKey != nullIndex {
		t.Errorf("body A still holds the destroyed contact")
	}
	if bodyB.contactCount != 1 || bodyB.headContactKey != moved.contactId<<1 {
		t.Errorf("body B does not hold only the surviving contact")
	}
}

// TestDestroyBodyDestroysItsContacts pins the contact walk of DestroyBody:
// every contact of the body goes away with it.
func TestDestroyBodyDestroysItsContacts(t *testing.T) {
	worldId := createTestWorld(t)
	idA := addDynamicCircle(t, worldId, v2(0, 0))
	idB := addDynamicCircle(t, worldId, v2(1, 0))
	idC := addDynamicCircle(t, worldId, v2(2, 0))

	w := getWorldFromId(worldId)
	createContact(w, firstShape(w, idA), firstShape(w, idB))
	createContact(w, firstShape(w, idB), firstShape(w, idC))

	DestroyBody(idB)

	bodyA := getBodyFullId(w, idA)
	bodyC := getBodyFullId(w, idC)
	if bodyA.contactCount != 0 || bodyA.headContactKey != nullIndex {
		t.Errorf("body A still holds a contact of the destroyed body")
	}
	if bodyC.contactCount != 0 || bodyC.headContactKey != nullIndex {
		t.Errorf("body C still holds a contact of the destroyed body")
	}
	if w.broadPhase.pairSet.count != 0 {
		t.Errorf("the pair set still holds %d pairs", w.broadPhase.pairSet.count)
	}
	if len(w.solverSets[awakeSet].contactSims) != 0 {
		t.Errorf("the awake set still holds contact sims")
	}
}

// TestDestroyShapeDestroysItsContacts checks the direct shape path. A freed
// shape id must not remain in the pair set or in either body's contact list.
func TestDestroyShapeDestroysItsContacts(t *testing.T) {
	worldId := createTestWorld(t)

	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyA := CreateBody(worldId, &bodyDef)
	bodyDef.Position = v2(1, 0)
	bodyB := CreateBody(worldId, &bodyDef)
	shapeDef := DefaultShapeDef()
	circle := Circle{Radius: fixed.Q32Half()}
	shapeIdA := CreateCircleShape(bodyA, &shapeDef, &circle)
	CreateCircleShape(bodyB, &shapeDef, &circle)

	w := getWorldFromId(worldId)
	shapeA := firstShape(w, bodyA)
	shapeB := firstShape(w, bodyB)
	pairKey := shapePairKey(shapeA.id, shapeB.id)
	createContact(w, shapeA, shapeB)

	DestroyShape(shapeIdA, false)

	if shapeIdA.IsValid() {
		t.Fatal("the destroyed shape id is still valid")
	}
	for _, bodyId := range []BodyId{bodyA, bodyB} {
		b := getBodyFullId(w, bodyId)
		if b.contactCount != 0 || b.headContactKey != nullIndex {
			t.Fatal("a body still holds the destroyed shape contact")
		}
	}
	if w.broadPhase.pairSet.containsKey(pairKey) || len(w.solverSets[awakeSet].contactSims) != 0 {
		t.Fatal("the destroyed shape contact remains in contact storage")
	}
}
