// Ported from samples/donut.cpp of Box2D v3.1.1

package samples

import (
	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/fixed"
)

const donutSideCount = 7

type donut struct {
	bodyIds   [donutSideCount]dbox2d.BodyId
	jointIds  [donutSideCount]dbox2d.JointId
	isSpawned bool
}

func (d *donut) create(worldId dbox2d.WorldId, position dbox2d.Vec2, scale dbox2d.Q, groupIndex int, enableSensorEvents bool, userData any) {
	if d.isSpawned {
		panic("samples: donut is already spawned")
	}
	for i := range donutSideCount {
		if !d.bodyIds[i].IsNull() || !d.jointIds[i].IsNull() {
			panic("samples: donut has non-null ids before creation")
		}
	}

	radius := scale
	length := dbox2d.Pi().Mul(fixed.Q32FromInt(2)).Mul(radius).Div(fixed.Q32FromInt(donutSideCount))
	capsule := dbox2d.Capsule{
		Center1: dbox2d.Vec2{Y: length.Mul(fixed.Q32Half()).Neg()},
		Center2: dbox2d.Vec2{Y: length.Mul(fixed.Q32Half())},
		Radius:  fixed.Q32MustParse("0.25").Mul(scale),
	}

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	bodyDef.UserData = userData

	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.EnableSensorEvents = enableSensorEvents
	shapeDef.Filter.GroupIndex = -groupIndex
	shapeDef.Material.Friction = fixed.Q32MustParse("0.3")

	for i := range donutSideCount {
		rot := dbox2d.MakeRot(fixed.Q32FromRatio(i, donutSideCount))
		bodyDef.Position = dbox2d.Vec2{
			X: radius.Mul(rot.Cos).Add(position.X),
			Y: radius.Mul(rot.Sin).Add(position.Y),
		}
		bodyDef.Rotation = rot
		d.bodyIds[i] = dbox2d.CreateBody(worldId, &bodyDef)
		dbox2d.CreateCapsuleShape(d.bodyIds[i], &shapeDef, &capsule)
	}

	weldDef := dbox2d.DefaultWeldJointDef()
	weldDef.AngularHertz = fixed.Q32FromInt(5)
	weldDef.AngularDampingRatio = fixed.Q32Zero()
	weldDef.LocalAnchorA = dbox2d.Vec2{Y: length.Mul(fixed.Q32Half())}
	weldDef.LocalAnchorB = dbox2d.Vec2{Y: length.Mul(fixed.Q32Half()).Neg()}
	prevBodyId := d.bodyIds[donutSideCount-1]
	for i := range donutSideCount {
		weldDef.BodyIdA = prevBodyId
		weldDef.BodyIdB = d.bodyIds[i]
		rotA := prevBodyId.GetRotation()
		rotB := d.bodyIds[i].GetRotation()
		weldDef.ReferenceAngle = dbox2d.RelativeAngle(rotB, rotA)
		d.jointIds[i] = dbox2d.CreateWeldJoint(worldId, &weldDef)
		prevBodyId = weldDef.BodyIdB
	}
	d.isSpawned = true
}

func (d *donut) destroy() {
	if !d.isSpawned {
		panic("samples: donut is not spawned")
	}
	for i := range donutSideCount {
		dbox2d.DestroyBody(d.bodyIds[i])
		d.bodyIds[i] = dbox2d.BodyId{}
		d.jointIds[i] = dbox2d.JointId{}
	}
	d.isSpawned = false
}
