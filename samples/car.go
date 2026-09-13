// Ported from samples/car.cpp of Box2D v3.1.1

package samples

import (
	"github.com/dhannyell/dbox2d"
)

type car struct {
	chassisId, rearWheelId, frontWheelId b2.BodyId
	rearAxleId, frontAxleId              b2.JointId
	isSpawned                            bool
}

func (c *car) spawn(worldId b2.WorldId, position b2.Vec2, scale, hertz, dampingRatio, torque b2.Q, userData any) {
	vertices := []b2.Vec2{
		{X: b2.F(-1.5), Y: b2.F(-0.5)},
		{X: b2.F(1.5), Y: b2.F(-0.5)},
		{X: b2.F(1.5), Y: b2.QZero()},
		{X: b2.QZero(), Y: b2.F(0.9)},
		{X: b2.F(-1.15), Y: b2.F(0.9)},
		{X: b2.F(-1.5), Y: b2.F(0.2)},
	}
	vertexScale := b2.F(0.85).Mul(scale)
	for i := range vertices {
		vertices[i].X = vertices[i].X.Mul(vertexScale)
		vertices[i].Y = vertices[i].Y.Mul(vertexScale)
	}

	hull := b2.ComputeHull(vertices)
	chassis := b2.MakePolygon(&hull, b2.F(0.15).Mul(scale))

	shapeDef := b2.DefaultShapeDef()
	shapeDef.Density = b2.QOne().Div(scale)
	shapeDef.Material.Friction = b2.F(0.2)

	circle := b2.Circle{Center: b2.Vec2{}, Radius: b2.F(0.4).Mul(scale)}

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.Position = b2.Vec2{Y: scale}.Add(position)
	c.chassisId = b2.CreateBody(worldId, &bodyDef)
	b2.CreatePolygonShape(c.chassisId, &shapeDef, &chassis)

	shapeDef.Density = b2.F(2).Div(scale)
	shapeDef.Material.Friction = b2.F(1.5)
	shapeDef.Material.RollingResistance = b2.F(0.1)

	bodyDef.Position = b2.Vec2{X: scale.Neg(), Y: b2.F(0.35).Mul(scale)}.Add(position)
	bodyDef.AllowFastRotation = true
	c.rearWheelId = b2.CreateBody(worldId, &bodyDef)
	b2.CreateCircleShape(c.rearWheelId, &shapeDef, &circle)

	bodyDef.Position = b2.Vec2{X: scale, Y: b2.F(0.4).Mul(scale)}.Add(position)
	bodyDef.AllowFastRotation = true
	c.frontWheelId = b2.CreateBody(worldId, &bodyDef)
	b2.CreateCircleShape(c.frontWheelId, &shapeDef, &circle)

	axis := b2.Vec2{Y: b2.QOne()}
	pivot := c.rearWheelId.GetPosition()
	jointDef := b2.DefaultWheelJointDef()
	jointDef.BodyIdA = c.chassisId
	jointDef.BodyIdB = c.rearWheelId
	jointDef.LocalAxisA = jointDef.BodyIdA.GetLocalVector(axis)
	jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
	jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
	jointDef.MotorSpeed = b2.QZero()
	jointDef.MaxMotorTorque = torque
	jointDef.EnableMotor = true
	jointDef.Hertz = hertz
	jointDef.DampingRatio = dampingRatio
	jointDef.LowerTranslation = b2.F(-0.25).Mul(scale)
	jointDef.UpperTranslation = b2.F(0.25).Mul(scale)
	jointDef.EnableLimit = true
	c.rearAxleId = b2.CreateWheelJoint(worldId, &jointDef)

	pivot = c.frontWheelId.GetPosition()
	jointDef.BodyIdA = c.chassisId
	jointDef.BodyIdB = c.frontWheelId
	jointDef.LocalAxisA = jointDef.BodyIdA.GetLocalVector(axis)
	jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
	jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
	jointDef.MotorSpeed = b2.QZero()
	jointDef.MaxMotorTorque = torque
	jointDef.EnableMotor = true
	jointDef.Hertz = hertz
	jointDef.DampingRatio = dampingRatio
	jointDef.LowerTranslation = b2.F(-0.25).Mul(scale)
	jointDef.UpperTranslation = b2.F(0.25).Mul(scale)
	jointDef.EnableLimit = true
	c.frontAxleId = b2.CreateWheelJoint(worldId, &jointDef)
	c.isSpawned = true
}

func (c *car) despawn() {
	b2.DestroyJoint(c.rearAxleId)
	b2.DestroyJoint(c.frontAxleId)
	b2.DestroyBody(c.rearWheelId)
	b2.DestroyBody(c.frontWheelId)
	b2.DestroyBody(c.chassisId)
	c.isSpawned = false
}

func (c *car) setSpeed(speed b2.Q) {
	// rad/s to turns/s.
	turnsPerSecond := speed.Div(b2.Pi().Mul(b2.F(2)))
	c.rearAxleId.SetMotorSpeed(turnsPerSecond)
	c.frontAxleId.SetMotorSpeed(turnsPerSecond)
	c.rearAxleId.WakeBodies()
}

func (c *car) setTorque(torque b2.Q) {
	c.rearAxleId.SetMaxMotorTorque(torque)
	c.frontAxleId.SetMaxMotorTorque(torque)
}

func (c *car) setHertz(hertz b2.Q) {
	c.rearAxleId.SetSpringHertz(hertz)
	c.frontAxleId.SetSpringHertz(hertz)
}

func (c *car) setDampingRatio(dampingRatio b2.Q) {
	c.rearAxleId.SetSpringDampingRatio(dampingRatio)
	c.frontAxleId.SetSpringDampingRatio(dampingRatio)
}

type truck struct {
	chassisId, rearWheelId, frontWheelId b2.BodyId
	rearAxleId, frontAxleId              b2.JointId
	isSpawned                            bool
}

func (t *truck) spawn(worldId b2.WorldId, position b2.Vec2, scale, hertz, dampingRatio, torque, density b2.Q, userData any) {
	vertices := []b2.Vec2{
		{X: b2.F(-0.65), Y: b2.F(-0.4)},
		{X: b2.F(1.5), Y: b2.F(-0.4)},
		{X: b2.F(1.5), Y: b2.QZero()},
		{X: b2.QZero(), Y: b2.F(0.9)},
		{X: b2.F(-0.65), Y: b2.F(0.9)},
	}
	vertexScale := b2.F(0.85).Mul(scale)
	for i := range vertices {
		vertices[i].X = vertices[i].X.Mul(vertexScale)
		vertices[i].Y = vertices[i].Y.Mul(vertexScale)
	}

	hull := b2.ComputeHull(vertices)
	chassis := b2.MakePolygon(&hull, b2.F(0.15).Mul(scale))

	shapeDef := b2.DefaultShapeDef()
	shapeDef.Density = density
	shapeDef.Material.Friction = b2.F(0.2)
	shapeDef.Material.CustomColor = uint32(b2.ColorHotPink)

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.Position = b2.Vec2{Y: scale}.Add(position)
	t.chassisId = b2.CreateBody(worldId, &bodyDef)
	b2.CreatePolygonShape(t.chassisId, &shapeDef, &chassis)

	box := b2.MakeOffsetBox(b2.F(1.25).Mul(scale), b2.F(0.1).Mul(scale), b2.Vec2{X: b2.F(-2.05).Mul(scale), Y: b2.F(-0.275).Mul(scale)}, b2.RotIdentity())
	box.Radius = b2.F(0.1).Mul(scale)
	b2.CreatePolygonShape(t.chassisId, &shapeDef, &box)

	box = b2.MakeOffsetBox(b2.F(0.05).Mul(scale), b2.F(0.35).Mul(scale), b2.Vec2{X: b2.F(-3.25).Mul(scale), Y: b2.F(0.375).Mul(scale)}, b2.RotIdentity())
	box.Radius = b2.F(0.1).Mul(scale)
	b2.CreatePolygonShape(t.chassisId, &shapeDef, &box)

	shapeDef.Density = b2.F(2).Mul(density)
	shapeDef.Material.Friction = b2.F(2.5)
	shapeDef.Material.CustomColor = uint32(b2.ColorSilver)

	circle := b2.Circle{Center: b2.Vec2{}, Radius: b2.F(0.4).Mul(scale)}
	bodyDef.Position = b2.Vec2{X: b2.F(-2.75).Mul(scale), Y: b2.F(0.3).Mul(scale)}.Add(position)
	t.rearWheelId = b2.CreateBody(worldId, &bodyDef)
	b2.CreateCircleShape(t.rearWheelId, &shapeDef, &circle)

	bodyDef.Position = b2.Vec2{X: b2.F(0.8).Mul(scale), Y: b2.F(0.3).Mul(scale)}.Add(position)
	t.frontWheelId = b2.CreateBody(worldId, &bodyDef)
	b2.CreateCircleShape(t.frontWheelId, &shapeDef, &circle)

	axis := b2.Vec2{Y: b2.QOne()}
	pivot := t.rearWheelId.GetPosition()
	jointDef := b2.DefaultWheelJointDef()
	jointDef.BodyIdA = t.chassisId
	jointDef.BodyIdB = t.rearWheelId
	jointDef.LocalAxisA = jointDef.BodyIdA.GetLocalVector(axis)
	jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
	jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
	jointDef.MotorSpeed = b2.QZero()
	jointDef.MaxMotorTorque = torque
	jointDef.EnableMotor = true
	jointDef.Hertz = hertz
	jointDef.DampingRatio = dampingRatio
	jointDef.LowerTranslation = b2.F(-0.25).Mul(scale)
	jointDef.UpperTranslation = b2.F(0.25).Mul(scale)
	jointDef.EnableLimit = true
	t.rearAxleId = b2.CreateWheelJoint(worldId, &jointDef)

	pivot = t.frontWheelId.GetPosition()
	jointDef.BodyIdA = t.chassisId
	jointDef.BodyIdB = t.frontWheelId
	jointDef.LocalAxisA = jointDef.BodyIdA.GetLocalVector(axis)
	jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
	jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
	jointDef.MotorSpeed = b2.QZero()
	jointDef.MaxMotorTorque = torque
	jointDef.EnableMotor = true
	jointDef.Hertz = hertz
	jointDef.DampingRatio = dampingRatio
	jointDef.LowerTranslation = b2.F(-0.25).Mul(scale)
	jointDef.UpperTranslation = b2.F(0.25).Mul(scale)
	jointDef.EnableLimit = true
	t.frontAxleId = b2.CreateWheelJoint(worldId, &jointDef)
	t.isSpawned = true
}

func (t *truck) despawn() {
	b2.DestroyJoint(t.rearAxleId)
	b2.DestroyJoint(t.frontAxleId)
	b2.DestroyBody(t.rearWheelId)
	b2.DestroyBody(t.frontWheelId)
	b2.DestroyBody(t.chassisId)
	t.isSpawned = false
}

func (t *truck) setSpeed(speed b2.Q) {
	// rad/s to turns/s.
	turnsPerSecond := speed.Div(b2.Pi().Mul(b2.F(2)))
	t.rearAxleId.SetMotorSpeed(turnsPerSecond)
	t.frontAxleId.SetMotorSpeed(turnsPerSecond)
	t.rearAxleId.WakeBodies()
}

func (t *truck) setTorque(torque b2.Q) {
	t.rearAxleId.SetMaxMotorTorque(torque)
	t.frontAxleId.SetMaxMotorTorque(torque)
}

func (t *truck) setHertz(hertz b2.Q) {
	t.rearAxleId.SetSpringHertz(hertz)
	t.frontAxleId.SetSpringHertz(hertz)
}

func (t *truck) setDampingRatio(dampingRatio b2.Q) {
	t.rearAxleId.SetSpringDampingRatio(dampingRatio)
	t.frontAxleId.SetSpringDampingRatio(dampingRatio)
}
