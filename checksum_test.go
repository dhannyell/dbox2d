package dbox2d

import (
	"testing"

	"github.com/dhannyell/fixed"
)

// TestChecksumIsOrderIndependent pins the commutative fold: two worlds with
// the same bodies, created in opposite orders, share one checksum before and
// after stepping.
func TestChecksumIsOrderIndependent(t *testing.T) {
	positions := []Vec2{v2(0, 10), v2(5, 10), v2(10, 10)}

	build := func(reversed bool) WorldId {
		worldId := createTestWorld(t)
		for i := range positions {
			p := positions[i]
			if reversed {
				p = positions[len(positions)-1-i]
			}
			addDynamicBox(t, worldId, p)
		}
		return worldId
	}

	world1 := build(false)
	world2 := build(true)

	if Checksum(world1) != Checksum(world2) {
		t.Fatalf("checksums differ before stepping: %d and %d", Checksum(world1), Checksum(world2))
	}

	dt := stepDt()
	for range 60 {
		Step(world1, dt, 4)
		Step(world2, dt, 4)
	}
	if Checksum(world1) != Checksum(world2) {
		t.Errorf("checksums differ after stepping: %d and %d", Checksum(world1), Checksum(world2))
	}
}

// TestChecksumSeesAStateChange pins sensitivity: a moved body changes the
// checksum, and a sleeping body still contributes its transform.
func TestChecksumSeesAStateChange(t *testing.T) {
	worldId := createTestWorld(t)
	bodyId := addDynamicBox(t, worldId, v2(0, 10))
	before := Checksum(worldId)

	w := getWorldFromId(worldId)
	b := getBodyFullId(w, bodyId)
	getBodyState(w, b).linearVelocity = Vec2{X: fixed.One()}
	if Checksum(worldId) == before {
		t.Errorf("a velocity change did not change the checksum")
	}

	// A body outside the awake set folds only its transform.
	sleeperDef := DefaultBodyDef()
	sleeperDef.Type = DynamicBody
	sleeperDef.IsAwake = false
	sleeperDef.Position = v2(20, 0)
	CreateBody(worldId, &sleeperDef)
	if Checksum(worldId) == before {
		t.Errorf("a new sleeping body did not change the checksum")
	}
}
