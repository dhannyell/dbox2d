package samples

import (
	"testing"

	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/fixed"
)

// TestHumanHelpers drives the human controls the scenes rely on: a spawned
// human has every bone, the scale moves the bones away from the hip, and
// the joint controls accept zero and non-zero values.
func TestHumanHelpers(t *testing.T) {
	worldDef := dbox2d.DefaultWorldDef()
	worldId := dbox2d.CreateWorld(&worldDef)
	defer dbox2d.DestroyWorld(worldId)

	h := createHuman(worldId, dbox2d.Vec2{}, fixed.Q32One(), fixed.Q32MustParse("0.03"),
		fixed.Q32FromInt(5), fixed.Q32Half(), 1, nil, false)
	if !h.isSpawned {
		t.Fatalf("the human is not spawned")
	}
	for i := range int(boneCount) {
		if h.bones[i].bodyId.IsNull() {
			t.Fatalf("bone %d has no body", i)
		}
	}

	h.setVelocity(dbox2d.Vec2{X: fixed.Q32One()})
	if got := h.bones[boneTorso].bodyId.GetLinearVelocity(); got.X != fixed.Q32One() {
		t.Errorf("the torso velocity is %s, want 1", got.X)
	}
	h.applyRandomAngularImpulse(fixed.Q32One())
	h.setJointFrictionTorque(fixed.Q32Zero())
	h.setJointFrictionTorque(fixed.Q32One())
	h.setJointSpringHertz(fixed.Q32Zero())
	h.setJointSpringHertz(fixed.Q32FromInt(5))
	h.setJointDampingRatio(fixed.Q32Half())
	h.enableSensorEvents(true)

	before := h.bones[boneHead].bodyId.GetPosition().Y
	h.setScale(fixed.Q32FromInt(2))
	after := h.bones[boneHead].bodyId.GetPosition().Y
	if !after.Greater(before) {
		t.Errorf("the head is at %s after the scale, want above %s", after, before)
	}

	h.destroy()
	if h.isSpawned || !h.bones[boneHead].bodyId.IsNull() {
		t.Errorf("the human survives destroy")
	}
}
