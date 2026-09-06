package samples

import (
	"testing"

	"github.com/dhannyell/dbox2d"
)

// Drives car, truck, donut and doohickey end to end. No scene spawns a
// truck yet, so this test is what keeps it exercised.
func TestVehicleHelpersSpawnAndDespawn(t *testing.T) {
	worldDef := dbox2d.DefaultWorldDef()
	worldId := dbox2d.CreateWorld(&worldDef)
	t.Cleanup(func() { dbox2d.DestroyWorld(worldId) })

	var c car
	var tr truck
	var d donut
	var dh doohickey

	c.spawn(worldId, dbox2d.Vec2{X: dbox2d.QFromInt(0)}, dbox2d.QOne(), dbox2d.QFromInt(5), dbox2d.QMustParse("0.7"), dbox2d.QFromInt(5), nil)
	tr.spawn(worldId, dbox2d.Vec2{X: dbox2d.QFromInt(20)}, dbox2d.QOne(), dbox2d.QFromInt(5), dbox2d.QMustParse("0.7"), dbox2d.QFromInt(5), dbox2d.QOne(), nil)
	d.create(worldId, dbox2d.Vec2{X: dbox2d.QFromInt(40)}, dbox2d.QOne(), 0, false, nil)
	dh.spawn(worldId, dbox2d.Vec2{X: dbox2d.QFromInt(60)}, dbox2d.QOne())

	if !c.isSpawned || !tr.isSpawned || !d.isSpawned || !dh.isSpawned {
		t.Fatal("expected all helpers to report isSpawned after spawning")
	}

	c.setSpeed(dbox2d.QFromInt(10))
	c.setTorque(dbox2d.QFromInt(3))
	c.setHertz(dbox2d.QFromInt(4))
	c.setDampingRatio(dbox2d.QMustParse("0.5"))
	tr.setSpeed(dbox2d.QFromInt(10))
	tr.setTorque(dbox2d.QFromInt(3))
	tr.setHertz(dbox2d.QFromInt(4))
	tr.setDampingRatio(dbox2d.QMustParse("0.5"))

	for range 10 {
		worldId.Step(dbox2d.QFromRatio(1, 60), 4)
	}

	c.despawn()
	tr.despawn()
	d.destroy()
	dh.despawn()

	if c.isSpawned || tr.isSpawned || d.isSpawned || dh.isSpawned {
		t.Fatal("expected all helpers to report !isSpawned after despawning")
	}
}
