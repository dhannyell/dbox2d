//go:build dbox2d_fixed

package dbox2d

import "testing"

// TestStepRejectsASaturatedTimeStep keeps the fixed-only saturation case.
func TestStepRejectsASaturatedTimeStep(t *testing.T) {
	worldId := createTestWorld(t)
	requirePanic(t, func() { worldId.Step(QMaxValue(), 4) })
}

// TestStepPartitionsContactsByTheLaneWindow sends the contact of a 200000 kg
// box to the Q32 path and keeps the contact of a unit box in the lane.
func TestStepPartitionsContactsByTheLaneWindow(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	boxOnGround(t, worldId, QZero())
	heavyBox(worldId, Vec2{X: QFromInt(2), Y: QHalf()}, Vec2{})

	worldId.Step(stepDt(), 4)

	if w.contactCount32 != 1 {
		t.Fatalf("the step solved %d contacts in Q32, want 1", w.contactCount32)
	}
	for i := range w.contacts {
		c := &w.contacts[i]
		if c.setIndex != awakeSet {
			continue
		}
		cs := getContactSim(w, c)
		heavy := !fitsLaneWindow(cs.invMassA) || !fitsLaneWindow(cs.invMassB)
		if heavy != !contactFitsLane(cs) {
			t.Errorf("contact %d fits the lane %t with inverse masses %v and %v", i, contactFitsLane(cs), cs.invMassA, cs.invMassB)
		}
	}
}
