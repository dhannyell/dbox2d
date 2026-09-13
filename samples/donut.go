// Ported from samples/donut.cpp of Box2D v3.1.1

package samples

import (
	"github.com/dhannyell/dbox2d"
)

const donutSideCount = 7

type donut struct {
	bodyIds   [donutSideCount]b2.BodyId
	jointIds  [donutSideCount]b2.JointId
	isSpawned bool
}

func (d *donut) create(worldId b2.WorldId, position b2.Vec2, scale b2.Q, groupIndex int, enableSensorEvents bool, userData any) {
	if d.isSpawned {
		panic("samples: donut is already spawned")
	}
	for i := range donutSideCount {
		if !d.bodyIds[i].IsNull() || !d.jointIds[i].IsNull() {
			panic("samples: donut has non-null ids before creation")
		}
	}

	radius := scale
	length := b2.Pi().Mul(b2.F(2)).Mul(radius).Div(b2.F(donutSideCount))
	capsule := b2.Capsule{
		Center1: b2.Vec2{Y: length.Mul(b2.QHalf()).Neg()},
		Center2: b2.Vec2{Y: length.Mul(b2.QHalf())},
		Radius:  b2.F(0.25).Mul(scale),
	}

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.UserData = userData

	shapeDef := b2.DefaultShapeDef()
	shapeDef.EnableSensorEvents = enableSensorEvents
	shapeDef.Filter.GroupIndex = -groupIndex
	shapeDef.Material.Friction = b2.F(0.3)

	for i := range donutSideCount {
		rot := b2.MakeRot(b2.QFromRatio(i, donutSideCount))
		bodyDef.Position = b2.Vec2{
			X: radius.Mul(rot.Cos).Add(position.X),
			Y: radius.Mul(rot.Sin).Add(position.Y),
		}
		bodyDef.Rotation = rot
		d.bodyIds[i] = b2.CreateBody(worldId, &bodyDef)
		b2.CreateCapsuleShape(d.bodyIds[i], &shapeDef, &capsule)
	}

	weldDef := b2.DefaultWeldJointDef()
	weldDef.AngularHertz = b2.F(5)
	weldDef.AngularDampingRatio = b2.QZero()
	weldDef.LocalAnchorA = b2.Vec2{Y: length.Mul(b2.QHalf())}
	weldDef.LocalAnchorB = b2.Vec2{Y: length.Mul(b2.QHalf()).Neg()}
	prevBodyId := d.bodyIds[donutSideCount-1]
	for i := range donutSideCount {
		weldDef.BodyIdA = prevBodyId
		weldDef.BodyIdB = d.bodyIds[i]
		rotA := prevBodyId.GetRotation()
		rotB := d.bodyIds[i].GetRotation()
		weldDef.ReferenceAngle = b2.RelativeAngle(rotB, rotA)
		d.jointIds[i] = b2.CreateWeldJoint(worldId, &weldDef)
		prevBodyId = weldDef.BodyIdB
	}
	d.isSpawned = true
}

func (d *donut) destroy() {
	if !d.isSpawned {
		panic("samples: donut is not spawned")
	}
	for i := range donutSideCount {
		b2.DestroyBody(d.bodyIds[i])
		d.bodyIds[i] = b2.BodyId{}
		d.jointIds[i] = b2.JointId{}
	}
	d.isSpawned = false
}
