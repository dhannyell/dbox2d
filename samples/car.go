// Ported from samples/car.cpp of Box2D v3.1.1

package samples

import (
	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/fixed"
)

type car struct {
	chassisId, rearWheelId, frontWheelId dbox2d.BodyId
	rearAxleId, frontAxleId              dbox2d.JointId
	isSpawned                            bool
}

func (c *car) spawn(worldId dbox2d.WorldId, position dbox2d.Vec2, scale, hertz, dampingRatio, torque dbox2d.Q, userData any) {
	vertices := []dbox2d.Vec2{
		{X: fixed.Q32MustParse("-1.5"), Y: fixed.Q32MustParse("-0.5")},
		{X: fixed.Q32MustParse("1.5"), Y: fixed.Q32MustParse("-0.5")},
		{X: fixed.Q32MustParse("1.5"), Y: fixed.Q32Zero()},
		{X: fixed.Q32Zero(), Y: fixed.Q32MustParse("0.9")},
		{X: fixed.Q32MustParse("-1.15"), Y: fixed.Q32MustParse("0.9")},
		{X: fixed.Q32MustParse("-1.5"), Y: fixed.Q32MustParse("0.2")},
	}
	vertexScale := fixed.Q32MustParse("0.85").Mul(scale)
	for i := range vertices {
		vertices[i].X = vertices[i].X.Mul(vertexScale)
		vertices[i].Y = vertices[i].Y.Mul(vertexScale)
	}

	hull := dbox2d.ComputeHull(vertices)
	chassis := dbox2d.MakePolygon(&hull, fixed.Q32MustParse("0.15").Mul(scale))

	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Density = fixed.Q32One().Div(scale)
	shapeDef.Material.Friction = fixed.Q32MustParse("0.2")

	circle := dbox2d.Circle{Center: dbox2d.Vec2{}, Radius: fixed.Q32MustParse("0.4").Mul(scale)}

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	bodyDef.Position = dbox2d.Vec2{Y: scale}.Add(position)
	c.chassisId = dbox2d.CreateBody(worldId, &bodyDef)
	dbox2d.CreatePolygonShape(c.chassisId, &shapeDef, &chassis)

	shapeDef.Density = fixed.Q32FromInt(2).Div(scale)
	shapeDef.Material.Friction = fixed.Q32MustParse("1.5")
	shapeDef.Material.RollingResistance = fixed.Q32MustParse("0.1")

	bodyDef.Position = dbox2d.Vec2{X: scale.Neg(), Y: fixed.Q32MustParse("0.35").Mul(scale)}.Add(position)
	bodyDef.AllowFastRotation = true
	c.rearWheelId = dbox2d.CreateBody(worldId, &bodyDef)
	dbox2d.CreateCircleShape(c.rearWheelId, &shapeDef, &circle)

	bodyDef.Position = dbox2d.Vec2{X: scale, Y: fixed.Q32MustParse("0.4").Mul(scale)}.Add(position)
	bodyDef.AllowFastRotation = true
	c.frontWheelId = dbox2d.CreateBody(worldId, &bodyDef)
	dbox2d.CreateCircleShape(c.frontWheelId, &shapeDef, &circle)

	axis := dbox2d.Vec2{Y: fixed.Q32One()}
	pivot := c.rearWheelId.GetPosition()
	jointDef := dbox2d.DefaultWheelJointDef()
	jointDef.BodyIdA = c.chassisId
	jointDef.BodyIdB = c.rearWheelId
	jointDef.LocalAxisA = jointDef.BodyIdA.GetLocalVector(axis)
	jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
	jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
	jointDef.MotorSpeed = fixed.Q32Zero()
	jointDef.MaxMotorTorque = torque
	jointDef.EnableMotor = true
	jointDef.Hertz = hertz
	jointDef.DampingRatio = dampingRatio
	jointDef.LowerTranslation = fixed.Q32MustParse("-0.25").Mul(scale)
	jointDef.UpperTranslation = fixed.Q32MustParse("0.25").Mul(scale)
	jointDef.EnableLimit = true
	c.rearAxleId = dbox2d.CreateWheelJoint(worldId, &jointDef)

	pivot = c.frontWheelId.GetPosition()
	jointDef.BodyIdA = c.chassisId
	jointDef.BodyIdB = c.frontWheelId
	jointDef.LocalAxisA = jointDef.BodyIdA.GetLocalVector(axis)
	jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
	jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
	jointDef.MotorSpeed = fixed.Q32Zero()
	jointDef.MaxMotorTorque = torque
	jointDef.EnableMotor = true
	jointDef.Hertz = hertz
	jointDef.DampingRatio = dampingRatio
	jointDef.LowerTranslation = fixed.Q32MustParse("-0.25").Mul(scale)
	jointDef.UpperTranslation = fixed.Q32MustParse("0.25").Mul(scale)
	jointDef.EnableLimit = true
	c.frontAxleId = dbox2d.CreateWheelJoint(worldId, &jointDef)
	c.isSpawned = true
}

func (c *car) despawn() {
	dbox2d.DestroyJoint(c.rearAxleId)
	dbox2d.DestroyJoint(c.frontAxleId)
	dbox2d.DestroyBody(c.rearWheelId)
	dbox2d.DestroyBody(c.frontWheelId)
	dbox2d.DestroyBody(c.chassisId)
	c.isSpawned = false
}

func (c *car) setSpeed(speed dbox2d.Q) {
	// rad/s to turns/s.
	turnsPerSecond := speed.Div(dbox2d.Pi().Mul(fixed.Q32FromInt(2)))
	c.rearAxleId.SetMotorSpeed(turnsPerSecond)
	c.frontAxleId.SetMotorSpeed(turnsPerSecond)
	c.rearAxleId.WakeBodies()
}

func (c *car) setTorque(torque dbox2d.Q) {
	c.rearAxleId.SetMaxMotorTorque(torque)
	c.frontAxleId.SetMaxMotorTorque(torque)
}

func (c *car) setHertz(hertz dbox2d.Q) {
	c.rearAxleId.SetSpringHertz(hertz)
	c.frontAxleId.SetSpringHertz(hertz)
}

func (c *car) setDampingRatio(dampingRatio dbox2d.Q) {
	c.rearAxleId.SetSpringDampingRatio(dampingRatio)
	c.frontAxleId.SetSpringDampingRatio(dampingRatio)
}

type truck struct {
	chassisId, rearWheelId, frontWheelId dbox2d.BodyId
	rearAxleId, frontAxleId              dbox2d.JointId
	isSpawned                            bool
}

func (t *truck) spawn(worldId dbox2d.WorldId, position dbox2d.Vec2, scale, hertz, dampingRatio, torque, density dbox2d.Q, userData any) {
	vertices := []dbox2d.Vec2{
		{X: fixed.Q32MustParse("-0.65"), Y: fixed.Q32MustParse("-0.4")},
		{X: fixed.Q32MustParse("1.5"), Y: fixed.Q32MustParse("-0.4")},
		{X: fixed.Q32MustParse("1.5"), Y: fixed.Q32Zero()},
		{X: fixed.Q32Zero(), Y: fixed.Q32MustParse("0.9")},
		{X: fixed.Q32MustParse("-0.65"), Y: fixed.Q32MustParse("0.9")},
	}
	vertexScale := fixed.Q32MustParse("0.85").Mul(scale)
	for i := range vertices {
		vertices[i].X = vertices[i].X.Mul(vertexScale)
		vertices[i].Y = vertices[i].Y.Mul(vertexScale)
	}

	hull := dbox2d.ComputeHull(vertices)
	chassis := dbox2d.MakePolygon(&hull, fixed.Q32MustParse("0.15").Mul(scale))

	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Density = density
	shapeDef.Material.Friction = fixed.Q32MustParse("0.2")
	shapeDef.Material.CustomColor = uint32(dbox2d.ColorHotPink)

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	bodyDef.Position = dbox2d.Vec2{Y: scale}.Add(position)
	t.chassisId = dbox2d.CreateBody(worldId, &bodyDef)
	dbox2d.CreatePolygonShape(t.chassisId, &shapeDef, &chassis)

	box := dbox2d.MakeOffsetBox(fixed.Q32MustParse("1.25").Mul(scale), fixed.Q32MustParse("0.1").Mul(scale), dbox2d.Vec2{X: fixed.Q32MustParse("-2.05").Mul(scale), Y: fixed.Q32MustParse("-0.275").Mul(scale)}, dbox2d.RotIdentity())
	box.Radius = fixed.Q32MustParse("0.1").Mul(scale)
	dbox2d.CreatePolygonShape(t.chassisId, &shapeDef, &box)

	box = dbox2d.MakeOffsetBox(fixed.Q32MustParse("0.05").Mul(scale), fixed.Q32MustParse("0.35").Mul(scale), dbox2d.Vec2{X: fixed.Q32MustParse("-3.25").Mul(scale), Y: fixed.Q32MustParse("0.375").Mul(scale)}, dbox2d.RotIdentity())
	box.Radius = fixed.Q32MustParse("0.1").Mul(scale)
	dbox2d.CreatePolygonShape(t.chassisId, &shapeDef, &box)

	shapeDef.Density = fixed.Q32FromInt(2).Mul(density)
	shapeDef.Material.Friction = fixed.Q32MustParse("2.5")
	shapeDef.Material.CustomColor = uint32(dbox2d.ColorSilver)

	circle := dbox2d.Circle{Center: dbox2d.Vec2{}, Radius: fixed.Q32MustParse("0.4").Mul(scale)}
	bodyDef.Position = dbox2d.Vec2{X: fixed.Q32MustParse("-2.75").Mul(scale), Y: fixed.Q32MustParse("0.3").Mul(scale)}.Add(position)
	t.rearWheelId = dbox2d.CreateBody(worldId, &bodyDef)
	dbox2d.CreateCircleShape(t.rearWheelId, &shapeDef, &circle)

	bodyDef.Position = dbox2d.Vec2{X: fixed.Q32MustParse("0.8").Mul(scale), Y: fixed.Q32MustParse("0.3").Mul(scale)}.Add(position)
	t.frontWheelId = dbox2d.CreateBody(worldId, &bodyDef)
	dbox2d.CreateCircleShape(t.frontWheelId, &shapeDef, &circle)

	axis := dbox2d.Vec2{Y: fixed.Q32One()}
	pivot := t.rearWheelId.GetPosition()
	jointDef := dbox2d.DefaultWheelJointDef()
	jointDef.BodyIdA = t.chassisId
	jointDef.BodyIdB = t.rearWheelId
	jointDef.LocalAxisA = jointDef.BodyIdA.GetLocalVector(axis)
	jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
	jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
	jointDef.MotorSpeed = fixed.Q32Zero()
	jointDef.MaxMotorTorque = torque
	jointDef.EnableMotor = true
	jointDef.Hertz = hertz
	jointDef.DampingRatio = dampingRatio
	jointDef.LowerTranslation = fixed.Q32MustParse("-0.25").Mul(scale)
	jointDef.UpperTranslation = fixed.Q32MustParse("0.25").Mul(scale)
	jointDef.EnableLimit = true
	t.rearAxleId = dbox2d.CreateWheelJoint(worldId, &jointDef)

	pivot = t.frontWheelId.GetPosition()
	jointDef.BodyIdA = t.chassisId
	jointDef.BodyIdB = t.frontWheelId
	jointDef.LocalAxisA = jointDef.BodyIdA.GetLocalVector(axis)
	jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
	jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
	jointDef.MotorSpeed = fixed.Q32Zero()
	jointDef.MaxMotorTorque = torque
	jointDef.EnableMotor = true
	jointDef.Hertz = hertz
	jointDef.DampingRatio = dampingRatio
	jointDef.LowerTranslation = fixed.Q32MustParse("-0.25").Mul(scale)
	jointDef.UpperTranslation = fixed.Q32MustParse("0.25").Mul(scale)
	jointDef.EnableLimit = true
	t.frontAxleId = dbox2d.CreateWheelJoint(worldId, &jointDef)
	t.isSpawned = true
}

func (t *truck) despawn() {
	dbox2d.DestroyJoint(t.rearAxleId)
	dbox2d.DestroyJoint(t.frontAxleId)
	dbox2d.DestroyBody(t.rearWheelId)
	dbox2d.DestroyBody(t.frontWheelId)
	dbox2d.DestroyBody(t.chassisId)
	t.isSpawned = false
}

func (t *truck) setSpeed(speed dbox2d.Q) {
	// rad/s to turns/s.
	turnsPerSecond := speed.Div(dbox2d.Pi().Mul(fixed.Q32FromInt(2)))
	t.rearAxleId.SetMotorSpeed(turnsPerSecond)
	t.frontAxleId.SetMotorSpeed(turnsPerSecond)
	t.rearAxleId.WakeBodies()
}

func (t *truck) setTorque(torque dbox2d.Q) {
	t.rearAxleId.SetMaxMotorTorque(torque)
	t.frontAxleId.SetMaxMotorTorque(torque)
}

func (t *truck) setHertz(hertz dbox2d.Q) {
	t.rearAxleId.SetSpringHertz(hertz)
	t.frontAxleId.SetSpringHertz(hertz)
}

func (t *truck) setDampingRatio(dampingRatio dbox2d.Q) {
	t.rearAxleId.SetSpringDampingRatio(dampingRatio)
	t.frontAxleId.SetSpringDampingRatio(dampingRatio)
}
