package samples

import (
	"testing"

	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/fixed"
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

	c.spawn(worldId, dbox2d.Vec2{X: fixed.Q32FromInt(0)}, fixed.Q32One(), fixed.Q32FromInt(5), fixed.Q32MustParse("0.7"), fixed.Q32FromInt(5), nil)
	tr.spawn(worldId, dbox2d.Vec2{X: fixed.Q32FromInt(20)}, fixed.Q32One(), fixed.Q32FromInt(5), fixed.Q32MustParse("0.7"), fixed.Q32FromInt(5), fixed.Q32One(), nil)
	d.create(worldId, dbox2d.Vec2{X: fixed.Q32FromInt(40)}, fixed.Q32One(), 0, false, nil)
	dh.spawn(worldId, dbox2d.Vec2{X: fixed.Q32FromInt(60)}, fixed.Q32One())

	if !c.isSpawned || !tr.isSpawned || !d.isSpawned || !dh.isSpawned {
		t.Fatal("expected all helpers to report isSpawned after spawning")
	}

	c.setSpeed(fixed.Q32FromInt(10))
	c.setTorque(fixed.Q32FromInt(3))
	c.setHertz(fixed.Q32FromInt(4))
	c.setDampingRatio(fixed.Q32MustParse("0.5"))
	tr.setSpeed(fixed.Q32FromInt(10))
	tr.setTorque(fixed.Q32FromInt(3))
	tr.setHertz(fixed.Q32FromInt(4))
	tr.setDampingRatio(fixed.Q32MustParse("0.5"))

	for range 10 {
		worldId.Step(fixed.Q32FromRatio(1, 60), 4)
	}

	c.despawn()
	tr.despawn()
	d.destroy()
	dh.despawn()

	if c.isSpawned || tr.isSpawned || d.isSpawned || dh.isSpawned {
		t.Fatal("expected all helpers to report !isSpawned after despawning")
	}
}
