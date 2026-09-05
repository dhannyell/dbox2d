// Ported from samples/doohickey.cpp of Box2D v3.1.1

package samples

import (
	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/fixed"
)

type doohickey struct {
	wheelId1, wheelId2 dbox2d.BodyId
	barId1, barId2     dbox2d.BodyId
	axleId1, axleId2   dbox2d.JointId
	sliderId           dbox2d.JointId
	isSpawned          bool
}

func (d *doohickey) spawn(worldId dbox2d.WorldId, position dbox2d.Vec2, scale dbox2d.Q) {
	if d.isSpawned {
		panic("samples: doohickey is already spawned")
	}

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody

	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Material.RollingResistance = fixed.Q32MustParse("0.1")

	circle := dbox2d.Circle{Radius: scale}
	capsule := dbox2d.Capsule{
		Center1: dbox2d.Vec2{X: fixed.Q32FromRatio(-7, 2).Mul(scale)},
		Center2: dbox2d.Vec2{X: fixed.Q32FromRatio(7, 2).Mul(scale)},
		Radius:  fixed.Q32MustParse("0.15").Mul(scale),
	}

	bodyDef.Position = dbox2d.MulAdd(position, scale, dbox2d.Vec2{
		X: fixed.Q32FromInt(-5),
		Y: fixed.Q32FromInt(3),
	})
	d.wheelId1 = dbox2d.CreateBody(worldId, &bodyDef)
	dbox2d.CreateCircleShape(d.wheelId1, &shapeDef, &circle)

	bodyDef.Position = dbox2d.MulAdd(position, scale, dbox2d.Vec2{
		X: fixed.Q32FromInt(5),
		Y: fixed.Q32FromInt(3),
	})
	d.wheelId2 = dbox2d.CreateBody(worldId, &bodyDef)
	dbox2d.CreateCircleShape(d.wheelId2, &shapeDef, &circle)

	bodyDef.Position = dbox2d.MulAdd(position, scale, dbox2d.Vec2{
		X: fixed.Q32MustParse("-1.5"),
		Y: fixed.Q32FromInt(3),
	})
	d.barId1 = dbox2d.CreateBody(worldId, &bodyDef)
	dbox2d.CreateCapsuleShape(d.barId1, &shapeDef, &capsule)

	bodyDef.Position = dbox2d.MulAdd(position, scale, dbox2d.Vec2{
		X: fixed.Q32MustParse("1.5"),
		Y: fixed.Q32FromInt(3),
	})
	d.barId2 = dbox2d.CreateBody(worldId, &bodyDef)
	dbox2d.CreateCapsuleShape(d.barId2, &shapeDef, &capsule)

	revoluteDef := dbox2d.DefaultRevoluteJointDef()
	revoluteDef.BodyIdA = d.wheelId1
	revoluteDef.BodyIdB = d.barId1
	revoluteDef.LocalAnchorA = dbox2d.Vec2{}
	revoluteDef.LocalAnchorB = dbox2d.Vec2{X: fixed.Q32MustParse("-3.5").Mul(scale)}
	revoluteDef.EnableMotor = true
	revoluteDef.MaxMotorTorque = fixed.Q32FromInt(2).Mul(scale)
	d.axleId1 = dbox2d.CreateRevoluteJoint(worldId, &revoluteDef)

	revoluteDef.BodyIdA = d.wheelId2
	revoluteDef.BodyIdB = d.barId2
	revoluteDef.LocalAnchorA = dbox2d.Vec2{}
	revoluteDef.LocalAnchorB = dbox2d.Vec2{X: fixed.Q32MustParse("3.5").Mul(scale)}
	revoluteDef.EnableMotor = true
	revoluteDef.MaxMotorTorque = fixed.Q32FromInt(2).Mul(scale)
	d.axleId2 = dbox2d.CreateRevoluteJoint(worldId, &revoluteDef)

	prismaticDef := dbox2d.DefaultPrismaticJointDef()
	prismaticDef.BodyIdA = d.barId1
	prismaticDef.BodyIdB = d.barId2
	prismaticDef.LocalAxisA = dbox2d.Vec2{X: fixed.Q32One()}
	prismaticDef.LocalAnchorA = dbox2d.Vec2{X: fixed.Q32FromInt(2).Mul(scale)}
	prismaticDef.LocalAnchorB = dbox2d.Vec2{X: fixed.Q32FromInt(-2).Mul(scale)}
	prismaticDef.LowerTranslation = fixed.Q32FromInt(-2).Mul(scale)
	prismaticDef.UpperTranslation = fixed.Q32FromInt(2).Mul(scale)
	prismaticDef.EnableLimit = true
	prismaticDef.EnableMotor = true
	prismaticDef.MaxMotorForce = fixed.Q32FromInt(2).Mul(scale)
	prismaticDef.EnableSpring = true
	prismaticDef.Hertz = fixed.Q32One()
	prismaticDef.DampingRatio = fixed.Q32Half()
	d.sliderId = dbox2d.CreatePrismaticJoint(worldId, &prismaticDef)

	d.isSpawned = true
}

func (d *doohickey) despawn() {
	if !d.isSpawned {
		panic("samples: doohickey is not spawned")
	}

	dbox2d.DestroyJoint(d.axleId1)
	dbox2d.DestroyJoint(d.axleId2)
	dbox2d.DestroyJoint(d.sliderId)
	dbox2d.DestroyBody(d.wheelId1)
	dbox2d.DestroyBody(d.wheelId2)
	dbox2d.DestroyBody(d.barId1)
	dbox2d.DestroyBody(d.barId2)
	d.isSpawned = false
}
