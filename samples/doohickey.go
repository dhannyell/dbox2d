// Ported from samples/doohickey.cpp of Box2D v3.1.1

package samples

import (
	"github.com/dhannyell/dbox2d"
)

type doohickey struct {
	wheelId1, wheelId2 b2.BodyId
	barId1, barId2     b2.BodyId
	axleId1, axleId2   b2.JointId
	sliderId           b2.JointId
	isSpawned          bool
}

func (d *doohickey) spawn(worldId b2.WorldId, position b2.Vec2, scale b2.Q) {
	if d.isSpawned {
		panic("samples: doohickey is already spawned")
	}

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody

	shapeDef := b2.DefaultShapeDef()
	shapeDef.Material.RollingResistance = b2.F(0.1)

	circle := b2.Circle{Radius: scale}
	capsule := b2.Capsule{
		Center1: b2.Vec2{X: b2.QFromRatio(-7, 2).Mul(scale)},
		Center2: b2.Vec2{X: b2.QFromRatio(7, 2).Mul(scale)},
		Radius:  b2.F(0.15).Mul(scale),
	}

	bodyDef.Position = b2.MulAdd(position, scale, b2.Vec2{
		X: b2.F(-5),
		Y: b2.F(3),
	})
	d.wheelId1 = b2.CreateBody(worldId, &bodyDef)
	b2.CreateCircleShape(d.wheelId1, &shapeDef, &circle)

	bodyDef.Position = b2.MulAdd(position, scale, b2.Vec2{
		X: b2.F(5),
		Y: b2.F(3),
	})
	d.wheelId2 = b2.CreateBody(worldId, &bodyDef)
	b2.CreateCircleShape(d.wheelId2, &shapeDef, &circle)

	bodyDef.Position = b2.MulAdd(position, scale, b2.Vec2{
		X: b2.F(-1.5),
		Y: b2.F(3),
	})
	d.barId1 = b2.CreateBody(worldId, &bodyDef)
	b2.CreateCapsuleShape(d.barId1, &shapeDef, &capsule)

	bodyDef.Position = b2.MulAdd(position, scale, b2.Vec2{
		X: b2.F(1.5),
		Y: b2.F(3),
	})
	d.barId2 = b2.CreateBody(worldId, &bodyDef)
	b2.CreateCapsuleShape(d.barId2, &shapeDef, &capsule)

	revoluteDef := b2.DefaultRevoluteJointDef()
	revoluteDef.BodyIdA = d.wheelId1
	revoluteDef.BodyIdB = d.barId1
	revoluteDef.LocalAnchorA = b2.Vec2{}
	revoluteDef.LocalAnchorB = b2.Vec2{X: b2.F(-3.5).Mul(scale)}
	revoluteDef.EnableMotor = true
	revoluteDef.MaxMotorTorque = b2.F(2).Mul(scale)
	d.axleId1 = b2.CreateRevoluteJoint(worldId, &revoluteDef)

	revoluteDef.BodyIdA = d.wheelId2
	revoluteDef.BodyIdB = d.barId2
	revoluteDef.LocalAnchorA = b2.Vec2{}
	revoluteDef.LocalAnchorB = b2.Vec2{X: b2.F(3.5).Mul(scale)}
	revoluteDef.EnableMotor = true
	revoluteDef.MaxMotorTorque = b2.F(2).Mul(scale)
	d.axleId2 = b2.CreateRevoluteJoint(worldId, &revoluteDef)

	prismaticDef := b2.DefaultPrismaticJointDef()
	prismaticDef.BodyIdA = d.barId1
	prismaticDef.BodyIdB = d.barId2
	prismaticDef.LocalAxisA = b2.Vec2{X: b2.QOne()}
	prismaticDef.LocalAnchorA = b2.Vec2{X: b2.F(2).Mul(scale)}
	prismaticDef.LocalAnchorB = b2.Vec2{X: b2.F(-2).Mul(scale)}
	prismaticDef.LowerTranslation = b2.F(-2).Mul(scale)
	prismaticDef.UpperTranslation = b2.F(2).Mul(scale)
	prismaticDef.EnableLimit = true
	prismaticDef.EnableMotor = true
	prismaticDef.MaxMotorForce = b2.F(2).Mul(scale)
	prismaticDef.EnableSpring = true
	prismaticDef.Hertz = b2.QOne()
	prismaticDef.DampingRatio = b2.QHalf()
	d.sliderId = b2.CreatePrismaticJoint(worldId, &prismaticDef)

	d.isSpawned = true
}

func (d *doohickey) despawn() {
	if !d.isSpawned {
		panic("samples: doohickey is not spawned")
	}

	b2.DestroyJoint(d.axleId1)
	b2.DestroyJoint(d.axleId2)
	b2.DestroyJoint(d.sliderId)
	b2.DestroyBody(d.wheelId1)
	b2.DestroyBody(d.wheelId2)
	b2.DestroyBody(d.barId1)
	b2.DestroyBody(d.barId2)
	d.isSpawned = false
}
