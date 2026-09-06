package samples

import (
	"testing"

	"github.com/dhannyell/dbox2d"
)

// TestHumanHelpers drives the human controls the scenes rely on: a spawned
// human has every bone, the scale moves the bones away from the hip, and
// the joint controls accept zero and non-zero values.
func TestHumanHelpers(t *testing.T) {
	worldDef := dbox2d.DefaultWorldDef()
	worldId := dbox2d.CreateWorld(&worldDef)
	defer dbox2d.DestroyWorld(worldId)

	h := createHuman(worldId, dbox2d.Vec2{}, dbox2d.QOne(), dbox2d.QMustParse("0.03"),
		dbox2d.QFromInt(5), dbox2d.QHalf(), 1, nil, false)
	if !h.isSpawned {
		t.Fatalf("the human is not spawned")
	}
	for i := range int(boneCount) {
		if h.bones[i].bodyId.IsNull() {
			t.Fatalf("bone %d has no body", i)
		}
	}

	h.setVelocity(dbox2d.Vec2{X: dbox2d.QOne()})
	if got := h.bones[boneTorso].bodyId.GetLinearVelocity(); got.X != dbox2d.QOne() {
		t.Errorf("the torso velocity is %s, want 1", got.X)
	}
	h.applyRandomAngularImpulse(dbox2d.QOne())
	h.setJointFrictionTorque(dbox2d.QZero())
	h.setJointFrictionTorque(dbox2d.QOne())
	h.setJointSpringHertz(dbox2d.QZero())
	h.setJointSpringHertz(dbox2d.QFromInt(5))
	h.setJointDampingRatio(dbox2d.QHalf())
	h.enableSensorEvents(true)

	before := h.bones[boneHead].bodyId.GetPosition().Y
	h.setScale(dbox2d.QFromInt(2))
	after := h.bones[boneHead].bodyId.GetPosition().Y
	if !after.Greater(before) {
		t.Errorf("the head is at %s after the scale, want above %s", after, before)
	}

	h.destroy()
	if h.isSpawned || !h.bones[boneHead].bodyId.IsNull() {
		t.Errorf("the human survives destroy")
	}
}
