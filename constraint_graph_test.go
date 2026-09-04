package dbox2d

import (
	"testing"

	"github.com/dhannyell/fixed"
)

// startTouching creates a contact between the first shapes of two bodies
// and moves it into the constraint graph, as the started-touching branch
// of the collide pass does. The test fakes one manifold point.
func startTouching(t *testing.T, w *world, idA, idB BodyId) *contact {
	t.Helper()
	createContact(w, firstShape(w, idA), firstShape(w, idB))
	c := &w.contacts[len(w.contacts)-1]
	if c.setIndex != awakeSet {
		t.Fatalf("the contact is in set %d, want the awake set", c.setIndex)
	}
	localIndex := c.localIndex

	awake := &w.solverSets[awakeSet]
	cs := &awake.contactSims[localIndex]
	cs.manifold.PointCount = 1
	cs.simFlags |= simTouchingFlag
	c.flags |= contactTouchingFlag
	linkContact(w, c)

	cs = &awake.contactSims[localIndex]
	addContactToGraph(w, cs, c)

	// Remove the non-touching sim from the awake set.
	var movedIndex int
	awake.contactSims, movedIndex = removeSwap(awake.contactSims, localIndex)
	if movedIndex != nullIndex {
		movedContact := &w.contacts[awake.contactSims[localIndex].contactId]
		movedContact.localIndex = localIndex
	}
	return c
}

// TestGraphColorsSeparateSharedBodies pins the color rule: two contacts
// that share a dynamic body take distinct colors, and the sim moves to the
// color with the inverse masses of the bodies.
func TestGraphColorsSeparateSharedBodies(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)

	idA := addDynamicCircle(t, worldId, v2(0, 0))
	idB := addDynamicCircle(t, worldId, v2(1, 0))
	idC := addDynamicCircle(t, worldId, v2(2, 0))

	cAB := startTouching(t, w, idA, idB)
	cBC := startTouching(t, w, idB, idC)

	if cAB.colorIndex != 0 || cBC.colorIndex != 1 {
		t.Fatalf("the contacts took colors %d and %d, want 0 and 1", cAB.colorIndex, cBC.colorIndex)
	}
	if len(w.solverSets[awakeSet].contactSims) != 0 {
		t.Errorf("the awake set keeps %d non-touching sims", len(w.solverSets[awakeSet].contactSims))
	}

	cs := getContactSim(w, cAB)
	bodyA := getBodyFullId(w, idA)
	simA := getBodySim(w, bodyA)
	if cs.contactId != cAB.contactId || cs.bodySimIndexA != bodyA.localIndex {
		t.Errorf("the graph sim does not point at contact %d and body sim %d", cAB.contactId, bodyA.localIndex)
	}
	if cs.invMassA != simA.invMass || cs.invIA != simA.invInertia {
		t.Errorf("the graph sim copies invMass %v and invI %v, want %v and %v", cs.invMassA, cs.invIA, simA.invMass, simA.invInertia)
	}

	// The bits of a color follow its contacts.
	color0 := &w.constraintGraph.colors[0]
	if !color0.bodySet.getBit(bodyA.id) || color0.bodySet.getBit(getBodyFullId(w, idC).id) {
		t.Errorf("color 0 marks the wrong bodies")
	}

	destroyContact(w, cAB, false)
	if color0.bodySet.getBit(bodyA.id) || len(color0.contactSims) != 0 {
		t.Errorf("the destroy left color 0 with bits or sims")
	}
	if cBC.colorIndex != 1 || getContactSim(w, cBC).contactId != cBC.contactId {
		t.Errorf("the destroy moved the other contact")
	}
	validateSolverSets(w)
}

// TestStaticContactSkipsColorZero pins the static rule: a contact with a
// static body starts at color 1 and marks only the dynamic body; the
// static side gets no body sim and zero inverse mass.
func TestStaticContactSkipsColorZero(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)

	groundDef := DefaultBodyDef()
	groundId := CreateBody(worldId, &groundDef)
	shapeDef := DefaultShapeDef()
	box := MakeBox(fixed.Q32FromInt(1), fixed.Q32FromInt(1))
	CreatePolygonShape(groundId, &shapeDef, &box)

	idA := addDynamicCircle(t, worldId, v2(0, 1))

	c := startTouching(t, w, groundId, idA)
	if c.colorIndex != 1 {
		t.Fatalf("the static contact took color %d, want 1", c.colorIndex)
	}

	cs := getContactSim(w, c)
	if cs.bodySimIndexA != nullIndex || cs.invMassA != (Q{}) || cs.invIA != (Q{}) {
		t.Errorf("the static side has body sim %d and inverse mass %v", cs.bodySimIndexA, cs.invMassA)
	}
	if cs.bodySimIndexB == nullIndex || cs.invMassB == (Q{}) {
		t.Errorf("the dynamic side has no body sim or no inverse mass")
	}

	ground := getBodyFullId(w, groundId)
	if w.constraintGraph.colors[1].bodySet.getBit(ground.id) {
		t.Errorf("color 1 marks the static body")
	}
}

// TestGraphOverflowTakesTheRest pins the overflow: a body with more
// contacts than colors sends the rest to the last color, which keeps no
// body set.
func TestGraphOverflowTakesTheRest(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)

	hubId := addDynamicCircle(t, worldId, v2(0, 0))
	contacts := make([]*contact, 0, overflowIndex+2)
	for i := range overflowIndex + 2 {
		otherId := addDynamicCircle(t, worldId, v2(i+1, 0))
		contacts = append(contacts, startTouching(t, w, hubId, otherId))
	}

	for i, c := range contacts {
		want := min(i, overflowIndex)
		if c.colorIndex != want {
			t.Errorf("contact %d took color %d, want %d", i, c.colorIndex, want)
		}
	}
	overflow := &w.constraintGraph.colors[overflowIndex]
	if len(overflow.contactSims) != 2 || overflow.bodySet.bits != nil {
		t.Errorf("the overflow holds %d sims and a body set of %d blocks", len(overflow.contactSims), len(overflow.bodySet.bits))
	}
	validateSolverSets(w)
}

// TestJointColorsFollowTheBodies pins the joint color rules: a joint to a
// static body may take color zero, two joints that share a dynamic body
// take distinct colors, and the sim moves out of a color with the joint
// that filled its slot.
func TestJointColorsFollowTheBodies(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)

	groundDef := DefaultBodyDef()
	groundId := CreateBody(worldId, &groundDef)
	idA := addDynamicCircle(t, worldId, v2(0, 1))
	idB := addDynamicCircle(t, worldId, v2(1, 1))
	ground := getBodyFullId(w, groundId)
	bodyA := getBodyFullId(w, idA)

	jGA := linkHandJoint(t, w, groundId, idA)
	if jGA.colorIndex != 0 {
		t.Fatalf("the static joint took color %d, want 0", jGA.colorIndex)
	}
	if w.constraintGraph.colors[0].bodySet.getBit(ground.id) || !w.constraintGraph.colors[0].bodySet.getBit(bodyA.id) {
		t.Errorf("color 0 marks the wrong body")
	}

	jAB := linkHandJoint(t, w, idA, idB)
	if jAB.colorIndex != 1 {
		t.Fatalf("the shared joint took color %d, want 1", jAB.colorIndex)
	}
	validateSolverSets(w)

	// Remove the first joint of color 0 after a second one enters it.
	idC := addDynamicCircle(t, worldId, v2(2, 1))
	jGC := linkHandJoint(t, w, groundId, idC)
	if jGC.colorIndex != 0 || jGC.localIndex != 1 {
		t.Fatalf("the third joint took color %d slot %d, want 0 and 1", jGC.colorIndex, jGC.localIndex)
	}
	unlinkJoint(w, jGA)
	removeHandJointEdges(w, jGA)
	if jGC.localIndex != 0 || getJointSim(w, jGC).jointId != jGC.jointId {
		t.Errorf("the moved joint sits at slot %d", jGC.localIndex)
	}
	validateSolverSets(w)
}

// TestJointOverflowTakesTheRest pins the joint overflow: a body with more
// joints than colors sends the rest to the last color.
func TestJointOverflowTakesTheRest(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)

	hubId := addDynamicCircle(t, worldId, v2(0, 0))
	for i := range graphColorCount {
		otherId := addDynamicCircle(t, worldId, v2(i+1, 0))
		j := linkHandJoint(t, w, hubId, otherId)
		want := i
		if i >= overflowIndex {
			want = overflowIndex
		}
		if j.colorIndex != want {
			t.Fatalf("joint %d took color %d, want %d", i, j.colorIndex, want)
		}
	}
	validateSolverSets(w)
}
