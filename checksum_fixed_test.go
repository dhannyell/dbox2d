//go:build dbox2d_fixed

package b2

import "testing"

// Rebased when the body state took radians (D-004) and the colored contacts
// took the normal impulse of b2SolveContactsTask (D-018), and when the shape
// stopped folding the retired enlargedAABB flag (v3.2 phase 2), and when the
// body flags became one word, the shape folded its AABB margin and its
// custom filtering flag, and the body its safety factor (v3.2 phase 3), and
// when the solver, contacts and islands switched to v3.2 (v3.2 phase 5).
const checksumWitness uint64 = 9118052756229121518

// TestChecksumContactsIgnoreCreationOrder checks the contact graph as well as
// the body and shape folds. The second world reverses both object creation and
// contact orientation, but represents the same physical state. Only the
// fixed mode promises the same bits from both sides: the float mode rounds
// the mirrored manifold differently.
func TestChecksumContactsIgnoreCreationOrder(t *testing.T) {
	positions := [3]Vec2{v2(0, 0), {X: QMustParse("0.75")}, v2(4, 0)}
	build := func(order [3]int, reverseContact bool) WorldId {
		worldId := createTestWorld(t)
		var bodies [3]BodyId
		for _, index := range order {
			bodies[index] = addDynamicCircle(t, worldId, positions[index])
		}

		w := getWorldFromId(worldId)
		shapeA := firstShape(w, bodies[0])
		shapeB := firstShape(w, bodies[1])
		if reverseContact {
			shapeA, shapeB = shapeB, shapeA
		}
		createContact(w, shapeA, shapeB)
		c := &w.contacts[0]
		cs := getContactSim(w, c)
		xfA := getBodyTransformQuick(w, &w.bodies[shapeA.bodyId])
		xfB := getBodyTransformQuick(w, &w.bodies[shapeB.bodyId])
		updateContact(w, cs, shapeA, xfA, Vec2Zero(), shapeB, xfB, Vec2Zero())
		return worldId
	}

	world1 := build([3]int{0, 1, 2}, false)
	world2 := build([3]int{2, 1, 0}, true)
	if got, want := Checksum(world2), Checksum(world1); got != want {
		t.Fatalf("contact checksum after reversed creation = %d, want %d", got, want)
	}
}
