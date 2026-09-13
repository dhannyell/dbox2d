// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/sample_bodies.cpp of Box2D v3.1.1

package samples

import (
	"math"

	"github.com/dhannyell/dbox2d"
)

func init() {
	RegisterSample("Bodies", "Body Type", NewBodyType)
	RegisterSample("Bodies", "Weeble", NewWeeble)
	RegisterSample("Bodies", "Sleep", NewSleepBodies)
	RegisterSample("Bodies", "Bad", NewBadBody)
	RegisterSample("Bodies", "Pivot", NewPivot)
	RegisterSample("Bodies", "Kinematic", NewKinematicBody)
}

// groundSegment creates a static body with one segment from (-halfWidth, 0)
// to (halfWidth, 0), the ground most Bodies scenes share.
func groundSegment(worldId b2.WorldId, shapeDef *b2.ShapeDef, halfWidth int) (b2.BodyId, b2.ShapeId) {
	bodyDef := b2.DefaultBodyDef()
	groundId := b2.CreateBody(worldId, &bodyDef)

	segment := b2.Segment{
		Point1: b2.Vec2{X: b2.F(-halfWidth)},
		Point2: b2.Vec2{X: b2.F(halfWidth)},
	}
	shapeId := b2.CreateSegmentShape(groundId, shapeDef, &segment)
	return groundId, shapeId
}

// BodyType switches a platform and its neighbors between static, kinematic
// and dynamic, and enables or disables them.
type BodyType struct {
	Base
	attachmentId       b2.BodyId
	secondAttachmentId b2.BodyId
	platformId         b2.BodyId
	secondPayloadId    b2.BodyId
	touchingBodyId     b2.BodyId
	floatingBodyId     b2.BodyId
	bodyType           b2.BodyType
	speed              b2.Q
	isEnabled          bool
}

// NewBodyType builds the body type scene.
func NewBodyType(ctx *SampleContext) Sample {
	s := &BodyType{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0.8, Y: 6.4}
		ctx.Camera.Zoom = 25.0 * 0.4
	}

	s.bodyType = b2.DynamicBody
	s.isEnabled = true

	groundShapeDef := b2.DefaultShapeDef()
	groundId, _ := groundSegment(s.WorldId, &groundShapeDef, 20)

	// Define attachment
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.V2(-2, 3)
		s.attachmentId = b2.CreateBody(s.WorldId, &bodyDef)

		box := b2.MakeBox(b2.QHalf(), b2.F(2))
		shapeDef := b2.DefaultShapeDef()
		shapeDef.Density = b2.QOne()
		b2.CreatePolygonShape(s.attachmentId, &shapeDef, &box)
	}

	// Define second attachment
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = s.bodyType
		bodyDef.IsEnabled = s.isEnabled
		bodyDef.Position = b2.V2(3, 3)
		s.secondAttachmentId = b2.CreateBody(s.WorldId, &bodyDef)

		box := b2.MakeBox(b2.QHalf(), b2.F(2))
		shapeDef := b2.DefaultShapeDef()
		shapeDef.Density = b2.QOne()
		b2.CreatePolygonShape(s.secondAttachmentId, &shapeDef, &box)
	}

	// Define platform
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = s.bodyType
		bodyDef.IsEnabled = s.isEnabled
		bodyDef.Position = b2.V2(-4, 5)
		s.platformId = b2.CreateBody(s.WorldId, &bodyDef)

		// A quarter turn is the reference's 0.5 * pi.
		box := b2.MakeOffsetBox(b2.QHalf(), b2.F(4), b2.V2(4, 0), b2.MakeRot(b2.QFromRatio(1, 4)))

		shapeDef := b2.DefaultShapeDef()
		shapeDef.Density = b2.F(2)
		b2.CreatePolygonShape(s.platformId, &shapeDef, &box)

		revoluteDef := b2.DefaultRevoluteJointDef()
		pivot := b2.V2(-2, 5)
		revoluteDef.BodyIdA = s.attachmentId
		revoluteDef.BodyIdB = s.platformId
		revoluteDef.LocalAnchorA = s.attachmentId.GetLocalPoint(pivot)
		revoluteDef.LocalAnchorB = s.platformId.GetLocalPoint(pivot)
		revoluteDef.MaxMotorTorque = b2.F(50)
		revoluteDef.EnableMotor = true
		b2.CreateRevoluteJoint(s.WorldId, &revoluteDef)

		pivot = b2.V2(3, 5)
		revoluteDef.BodyIdA = s.secondAttachmentId
		revoluteDef.BodyIdB = s.platformId
		revoluteDef.LocalAnchorA = s.secondAttachmentId.GetLocalPoint(pivot)
		revoluteDef.LocalAnchorB = s.platformId.GetLocalPoint(pivot)
		revoluteDef.MaxMotorTorque = b2.F(50)
		revoluteDef.EnableMotor = true
		b2.CreateRevoluteJoint(s.WorldId, &revoluteDef)

		prismaticDef := b2.DefaultPrismaticJointDef()
		anchor := b2.V2(0, 5)
		prismaticDef.BodyIdA = groundId
		prismaticDef.BodyIdB = s.platformId
		prismaticDef.LocalAnchorA = groundId.GetLocalPoint(anchor)
		prismaticDef.LocalAnchorB = s.platformId.GetLocalPoint(anchor)
		prismaticDef.LocalAxisA = b2.Vec2{X: b2.QOne()}
		prismaticDef.MaxMotorForce = b2.F(1000)
		prismaticDef.MotorSpeed = b2.QZero()
		prismaticDef.EnableMotor = true
		prismaticDef.LowerTranslation = b2.F(-10)
		prismaticDef.UpperTranslation = b2.F(10)
		prismaticDef.EnableLimit = true
		b2.CreatePrismaticJoint(s.WorldId, &prismaticDef)

		s.speed = b2.F(3)
	}

	// Create a payload
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.V2(-3, 8)
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)

		box := b2.MakeBox(b2.QFromRatio(3, 4), b2.QFromRatio(3, 4))
		shapeDef := b2.DefaultShapeDef()
		shapeDef.Density = b2.F(2)
		b2.CreatePolygonShape(bodyId, &shapeDef, &box)
	}

	// Create a second payload
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = s.bodyType
		bodyDef.IsEnabled = s.isEnabled
		bodyDef.Position = b2.V2(2, 8)
		s.secondPayloadId = b2.CreateBody(s.WorldId, &bodyDef)

		box := b2.MakeBox(b2.QFromRatio(3, 4), b2.QFromRatio(3, 4))
		shapeDef := b2.DefaultShapeDef()
		shapeDef.Density = b2.F(2)
		b2.CreatePolygonShape(s.secondPayloadId, &shapeDef, &box)
	}

	// Create a separate body on the ground
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = s.bodyType
		bodyDef.IsEnabled = s.isEnabled
		bodyDef.Position = b2.V2(8.0, 0.2)
		s.touchingBodyId = b2.CreateBody(s.WorldId, &bodyDef)

		capsule := b2.Capsule{
			Center2: b2.Vec2{X: b2.QOne()},
			Radius:  b2.QFromRatio(1, 4),
		}
		shapeDef := b2.DefaultShapeDef()
		shapeDef.Density = b2.F(2)
		b2.CreateCapsuleShape(s.touchingBodyId, &shapeDef, &capsule)
	}

	// Create a separate floating body
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = s.bodyType
		bodyDef.IsEnabled = s.isEnabled
		bodyDef.Position = b2.V2(-8, 12)
		bodyDef.GravityScale = b2.QZero()
		s.floatingBodyId = b2.CreateBody(s.WorldId, &bodyDef)

		circle := b2.Circle{Center: b2.Vec2{Y: b2.QHalf()}, Radius: b2.QFromRatio(1, 4)}
		shapeDef := b2.DefaultShapeDef()
		shapeDef.Density = b2.F(2)
		b2.CreateCircleShape(s.floatingBodyId, &shapeDef, &circle)
	}

	return s
}

func (s *BodyType) setType(bodyType b2.BodyType) {
	s.bodyType = bodyType
	s.platformId.SetType(bodyType)
	if bodyType == b2.KinematicBody {
		s.platformId.SetLinearVelocity(b2.Vec2{X: s.speed.Neg()})
		s.platformId.SetAngularVelocity(b2.QZero())
	}
	s.secondAttachmentId.SetType(bodyType)
	s.secondPayloadId.SetType(bodyType)
	s.touchingBodyId.SetType(bodyType)
	s.floatingBodyId.SetType(bodyType)
}

// UpdateGui provides the body type radio buttons and the enable checkbox.
func (s *BodyType) UpdateGui() {
	height := 140
	gui := s.Context.Gui
	gui.Begin("Body Type", 10, s.Context.Camera.Height-height-50, 180, height)

	if gui.RadioButton("Static", s.bodyType == b2.StaticBody) {
		s.setType(b2.StaticBody)
	}

	if gui.RadioButton("Kinematic", s.bodyType == b2.KinematicBody) {
		s.setType(b2.KinematicBody)
	}

	if gui.RadioButton("Dynamic", s.bodyType == b2.DynamicBody) {
		s.setType(b2.DynamicBody)
	}

	if gui.Checkbox("Enable", &s.isEnabled) {
		if s.isEnabled {
			s.platformId.Enable()
			s.secondAttachmentId.Enable()
			s.secondPayloadId.Enable()
			s.touchingBodyId.Enable()
			s.floatingBodyId.Enable()

			if s.bodyType == b2.KinematicBody {
				s.platformId.SetLinearVelocity(b2.Vec2{X: s.speed.Neg()})
				s.platformId.SetAngularVelocity(b2.QZero())
			}
		} else {
			s.platformId.Disable()
			s.secondAttachmentId.Disable()
			s.secondPayloadId.Disable()
			s.touchingBodyId.Disable()
			s.floatingBodyId.Disable()
		}
	}

	gui.End()
}

// Step drives the kinematic platform back and forth, then advances the scene.
func (s *BodyType) Step() {
	if s.bodyType == b2.KinematicBody {
		p := s.platformId.GetPosition()
		v := s.platformId.GetLinearVelocity()

		if (p.X.Less(b2.F(-14)) && v.X.Less(b2.QZero())) ||
			(p.X.Greater(b2.F(6)) && v.X.Greater(b2.QZero())) {
			v.X = v.X.Neg()
			s.platformId.SetLinearVelocity(v)
		}
	}

	s.Base.Step()
}

func weebleFriction(b2.Q, int, b2.Q, int) b2.Q {
	return b2.QFromRatio(1, 10)
}

func weebleRestitution(b2.Q, int, b2.Q, int) b2.Q {
	return b2.QOne()
}

// Weeble is a capsule with its center of mass moved below the shape, so it
// always rights itself.
type Weeble struct {
	Base
	weebleId           b2.BodyId
	explosionPosition  b2.Vec2
	explosionRadius    b2.Q
	explosionMagnitude float64
}

// NewWeeble builds the weeble scene.
func NewWeeble(ctx *SampleContext) Sample {
	s := &Weeble{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 2.3, Y: 10.0}
		ctx.Camera.Zoom = 25.0 * 0.5
	}

	// Test friction and restitution callbacks
	s.WorldId.SetFrictionCallback(weebleFriction)
	s.WorldId.SetRestitutionCallback(weebleRestitution)

	groundShapeDef := b2.DefaultShapeDef()
	groundSegment(s.WorldId, &groundShapeDef, 20)

	// Build weeble
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.V2(0, 3)
		// An eighth of a turn is the reference's 0.25 * pi.
		bodyDef.Rotation = b2.MakeRot(b2.QFromRatio(1, 8))
		s.weebleId = b2.CreateBody(s.WorldId, &bodyDef)

		capsule := b2.Capsule{
			Center1: b2.V2(0, -1),
			Center2: b2.Vec2{Y: b2.QOne()},
			Radius:  b2.QOne(),
		}
		shapeDef := b2.DefaultShapeDef()
		b2.CreateCapsuleShape(s.weebleId, &shapeDef, &capsule)

		mass := s.weebleId.GetMass()
		inertiaTensor := s.weebleId.GetRotationalInertia()

		offset := b2.F(1.5)

		// See: https://en.wikipedia.org/wiki/Parallel_axis_theorem
		inertiaTensor = inertiaTensor.Add(mass.Mul(offset).Mul(offset))

		massData := b2.MassData{Mass: mass, Center: b2.Vec2{Y: offset.Neg()}, RotationalInertia: inertiaTensor}
		s.weebleId.SetMassData(massData)
	}

	s.explosionPosition = b2.Vec2{}
	s.explosionRadius = b2.F(2)
	s.explosionMagnitude = 8.0

	return s
}

// UpdateGui provides the teleport and explosion controls.
func (s *Weeble) UpdateGui() {
	height := 120
	gui := s.Context.Gui
	gui.Begin("Weeble", 10, s.Context.Camera.Height-height-50, 200, height)
	if gui.Button("Teleport") {
		// 0.475 turns is the reference's 0.95 * pi.
		s.weebleId.SetTransform(b2.V2(0, 5), b2.MakeRot(b2.F(0.475)))
	}

	if gui.Button("Explode") {
		def := b2.DefaultExplosionDef()
		def.Position = s.explosionPosition
		def.Radius = s.explosionRadius
		def.Falloff = b2.QFromRatio(1, 10)
		def.ImpulsePerLength = FromFloat64(s.explosionMagnitude)
		s.WorldId.Explode(&def)
	}

	gui.SliderFloat("Magnitude", &s.explosionMagnitude, -100.0, 100.0)

	gui.End()
}

// Step advances the scene and draws the velocity of a point on the weeble.
func (s *Weeble) Step() {
	s.Base.Step()

	s.Context.Draw.DrawCircle(s.explosionPosition, s.explosionRadius, b2.ColorAzure)

	// This shows how to get the velocity of a point on a body
	localPoint := b2.V2(0, 2)
	worldPoint := s.weebleId.GetWorldPoint(localPoint)

	v1 := s.weebleId.GetLocalPointVelocity(localPoint)
	v2 := s.weebleId.GetWorldPointVelocity(worldPoint)

	offset := b2.Vec2{X: b2.QFromRatio(1, 20)}
	s.Context.Draw.DrawSegment(worldPoint, worldPoint.Add(v1), b2.ColorRed)
	s.Context.Draw.DrawSegment(worldPoint.Add(offset), worldPoint.Add(v2).Add(offset), b2.ColorGreen)
}

// SleepBodies tests sleeping bodies, sensors on sleeping bodies, sleep
// tuning and waking on contact creation and destruction. It is the
// reference's Sleep; the name Sleep belongs to the benchmark.
type SleepBodies struct {
	Base
	pendulumId     b2.BodyId
	staticBodyId   b2.BodyId
	groundShapeId  b2.ShapeId
	sensorIds      [2]b2.ShapeId
	sensorTouching [2]bool
}

// NewSleepBodies builds the sleep scene.
func NewSleepBodies(ctx *SampleContext) Sample {
	s := &SleepBodies{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 3.0, Y: 50.0}
		ctx.Camera.Zoom = 25.0 * 2.2
	}

	groundShapeDef := b2.DefaultShapeDef()
	groundShapeDef.EnableSensorEvents = true
	groundId, groundShapeId := groundSegment(s.WorldId, &groundShapeDef, 40)
	s.groundShapeId = groundShapeId

	// Sleeping body with sensors
	for i := range 2 {
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.Vec2{X: b2.F(-4), Y: b2.F(3 + 2*i)}
		bodyDef.IsAwake = false
		bodyDef.EnableSleep = true
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)

		capsule := b2.Capsule{
			Center1: b2.Vec2{Y: b2.QOne()},
			Center2: b2.Vec2{X: b2.QOne(), Y: b2.QOne()},
			Radius:  b2.QFromRatio(3, 4),
		}
		shapeDef := b2.DefaultShapeDef()
		b2.CreateCapsuleShape(bodyId, &shapeDef, &capsule)

		shapeDef.IsSensor = true
		shapeDef.EnableSensorEvents = true
		capsule.Radius = b2.QOne()
		s.sensorIds[i] = b2.CreateCapsuleShape(bodyId, &shapeDef, &capsule)
		s.sensorTouching[i] = false
	}

	// Sleeping body but sleep is disabled
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.V2(0, 3)
		bodyDef.IsAwake = false
		bodyDef.EnableSleep = false
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)

		circle := b2.Circle{Center: b2.Vec2{X: b2.QOne(), Y: b2.QOne()}, Radius: b2.QOne()}
		shapeDef := b2.DefaultShapeDef()
		b2.CreateCircleShape(bodyId, &shapeDef, &circle)
	}

	// Awake body and sleep is disabled
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.V2(5, 3)
		bodyDef.IsAwake = true
		bodyDef.EnableSleep = false
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)

		box := b2.MakeOffsetBox(b2.QOne(), b2.QOne(), b2.Vec2{Y: b2.QOne()}, b2.MakeRot(b2.QFromRatio(1, 8)))
		shapeDef := b2.DefaultShapeDef()
		b2.CreatePolygonShape(bodyId, &shapeDef, &box)
	}

	// A sleeping body to test waking on collision
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.Vec2{X: b2.F(5), Y: b2.QOne()}
		bodyDef.IsAwake = false
		bodyDef.EnableSleep = true
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)

		box := b2.MakeSquare(b2.QOne())
		shapeDef := b2.DefaultShapeDef()
		b2.CreatePolygonShape(bodyId, &shapeDef, &box)
	}

	// A long pendulum
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.V2(0, 100)
		bodyDef.AngularDamping = b2.QHalf()
		bodyDef.SleepThreshold = b2.QFromRatio(1, 20)
		s.pendulumId = b2.CreateBody(s.WorldId, &bodyDef)

		capsule := b2.Capsule{
			Center2: b2.V2(90, 0),
			Radius:  b2.QFromRatio(1, 4),
		}
		shapeDef := b2.DefaultShapeDef()
		b2.CreateCapsuleShape(s.pendulumId, &shapeDef, &capsule)

		pivot := bodyDef.Position
		jointDef := b2.DefaultRevoluteJointDef()
		jointDef.BodyIdA = groundId
		jointDef.BodyIdB = s.pendulumId
		jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
		jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
		b2.CreateRevoluteJoint(s.WorldId, &jointDef)
	}

	// A sleeping body to test waking on contact destroyed
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.Vec2{X: b2.F(-10), Y: b2.QOne()}
		bodyDef.IsAwake = false
		bodyDef.EnableSleep = true
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)

		box := b2.MakeSquare(b2.QOne())
		shapeDef := b2.DefaultShapeDef()
		b2.CreatePolygonShape(bodyId, &shapeDef, &box)
	}

	s.staticBodyId = b2.BodyId{}
	return s
}

func (s *SleepBodies) toggleInvoker() {
	if s.staticBodyId.IsNull() {
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Position = b2.V2(-10.5, 3.0)
		s.staticBodyId = b2.CreateBody(s.WorldId, &bodyDef)

		box := b2.MakeOffsetBox(b2.F(2), b2.QFromRatio(1, 10), b2.Vec2{}, b2.MakeRot(b2.QFromRatio(1, 8)))
		shapeDef := b2.DefaultShapeDef()
		shapeDef.InvokeContactCreation = true
		b2.CreatePolygonShape(s.staticBodyId, &shapeDef, &box)
	} else {
		b2.DestroyBody(s.staticBodyId)
		s.staticBodyId = b2.BodyId{}
	}
}

// UpdateGui provides the pendulum tuning and the invoker toggle. The
// reference's separator line is not drawn.
func (s *SleepBodies) UpdateGui() {
	height := 160
	gui := s.Context.Gui
	gui.Begin("Sleep", 10, s.Context.Camera.Height-height-50, 240, height)

	gui.Text("Pendulum Tuning")

	sleepVelocity := ToFloat64(s.pendulumId.GetSleepThreshold())
	if gui.SliderFloat("sleep velocity", &sleepVelocity, 0.0, 1.0) {
		s.pendulumId.SetSleepThreshold(FromFloat64(sleepVelocity))
		s.pendulumId.SetAwake(true)
	}

	angularDamping := ToFloat64(s.pendulumId.GetAngularDamping())
	if gui.SliderFloat("angular damping", &angularDamping, 0.0, 2.0) {
		s.pendulumId.SetAngularDamping(FromFloat64(angularDamping))
	}

	label := "Destroy"
	if s.staticBodyId.IsNull() {
		label = "Create"
	}
	if gui.Button(label) {
		s.toggleInvoker()
	}

	gui.End()
}

// Step advances the scene and reports which sensors touch the ground.
func (s *SleepBodies) Step() {
	s.Base.Step()

	// Detect sensors touching the ground
	sensorEvents := s.WorldId.GetSensorEvents()

	for _, event := range sensorEvents.BeginEvents {
		if event.VisitorShapeId == s.groundShapeId {
			if event.SensorShapeId == s.sensorIds[0] {
				s.sensorTouching[0] = true
			} else if event.SensorShapeId == s.sensorIds[1] {
				s.sensorTouching[1] = true
			}
		}
	}

	for _, event := range sensorEvents.EndEvents {
		if event.VisitorShapeId == s.groundShapeId {
			if event.SensorShapeId == s.sensorIds[0] {
				s.sensorTouching[0] = false
			} else if event.SensorShapeId == s.sensorIds[1] {
				s.sensorTouching[1] = false
			}
		}
	}

	for i := range 2 {
		s.DrawTextLine("sensor touch %d = %t", i, s.sensorTouching[i])
	}
}

// BadBody shows a dynamic body with no mass next to a normal one.
type BadBody struct {
	Base
	badBodyId b2.BodyId
}

// NewBadBody builds the bad body scene.
func NewBadBody(ctx *SampleContext) Sample {
	s := &BadBody{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 2.3, Y: 10.0}
		ctx.Camera.Zoom = 25.0 * 0.5
	}

	groundShapeDef := b2.DefaultShapeDef()
	groundSegment(s.WorldId, &groundShapeDef, 20)

	capsule := b2.Capsule{
		Center1: b2.V2(0, -1),
		Center2: b2.Vec2{Y: b2.QOne()},
		Radius:  b2.QOne(),
	}

	// Build a bad body
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.V2(0, 3)
		bodyDef.AngularVelocity = radiansToTurns(0.5)
		bodyDef.Rotation = b2.MakeRot(b2.QFromRatio(1, 8))

		s.badBodyId = b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()

		// density set to zero intentionally to create a bad body
		shapeDef.Density = b2.QZero()
		b2.CreateCapsuleShape(s.badBodyId, &shapeDef, &capsule)
	}

	// Build a normal body
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.V2(2, 3)
		bodyDef.Rotation = b2.MakeRot(b2.QFromRatio(1, 8))

		bodyId := b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		b2.CreateCapsuleShape(bodyId, &shapeDef, &capsule)
	}

	return s
}

// Step advances the scene and pushes the bad body up.
func (s *BadBody) Step() {
	s.Base.Step()

	s.DrawTextLine("A bad body is a dynamic body with no mass and behaves like a kinematic body.")
	s.DrawTextLine("Bad bodies are considered invalid and a user bug. Behavior is not guaranteed.")

	// For science
	s.badBodyId.ApplyForceToCenter(b2.V2(0, 10), true)
}

// Pivot sets the initial angular velocity so that the bottom end of a lever
// stays at rest.
type Pivot struct {
	Base
	bodyId b2.BodyId
	lever  b2.Q
}

// NewPivot builds the pivot scene.
func NewPivot(ctx *SampleContext) Sample {
	s := &Pivot{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0.8, Y: 6.4}
		ctx.Camera.Zoom = 25.0 * 0.4
	}

	groundShapeDef := b2.DefaultShapeDef()
	groundSegment(s.WorldId, &groundShapeDef, 20)

	// Create a separate body on the ground
	{
		v := b2.V2(5, 0)

		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.V2(0, 3)
		bodyDef.GravityScale = b2.QOne()
		bodyDef.LinearVelocity = v

		s.bodyId = b2.CreateBody(s.WorldId, &bodyDef)

		s.lever = b2.F(3)
		r := b2.Vec2{Y: s.lever.Neg()}

		// The reference's omega is in radians per second; the body takes turns.
		omega := b2.Cross(v, r).Div(r.Dot(r))
		s.bodyId.SetAngularVelocity(omega.Div(b2.Pi().Mul(b2.F(2))))

		box := b2.MakeBox(b2.QFromRatio(1, 10), s.lever)
		shapeDef := b2.DefaultShapeDef()
		b2.CreatePolygonShape(s.bodyId, &shapeDef, &box)
	}

	return s
}

// Step advances the scene and reports the velocity of the pivot point.
func (s *Pivot) Step() {
	s.Base.Step()

	v := s.bodyId.GetLinearVelocity()
	omega := s.bodyId.GetAngularVelocity().Mul(b2.Pi().Mul(b2.F(2)))
	r := s.bodyId.GetWorldVector(b2.Vec2{Y: s.lever.Neg()})

	vp := v.Add(b2.CrossSV(omega, r))
	s.DrawTextLine("pivot velocity = (%g, %g)", ToFloat64(vp.X), ToFloat64(vp.Y))
}

// KinematicBody drives a kinematic body along a figure eight with target
// transforms. It is the reference's Kinematic; the name Kinematic belongs
// to the benchmark.
type KinematicBody struct {
	Base
	bodyId    b2.BodyId
	amplitude float64
	time      float64
}

// NewKinematicBody builds the kinematic scene.
func NewKinematicBody(ctx *SampleContext) Sample {
	s := &KinematicBody{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0.0, Y: 0.0}
		ctx.Camera.Zoom = 4.0
	}

	s.amplitude = 2.0

	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.KinematicBody
		bodyDef.Position.X = FromFloat64(2.0 * s.amplitude)

		s.bodyId = b2.CreateBody(s.WorldId, &bodyDef)

		box := b2.MakeBox(b2.QFromRatio(1, 10), b2.QOne())
		shapeDef := b2.DefaultShapeDef()
		b2.CreatePolygonShape(s.bodyId, &shapeDef, &box)
	}

	s.time = 0.0
	return s
}

// Step sets the target transform for this step, then advances the scene.
func (s *KinematicBody) Step() {
	timeStep := 0.0
	if s.Context.Settings.Hertz > 0 {
		timeStep = 1.0 / s.Context.Settings.Hertz
	}
	if s.Context.Settings.Pause && !s.Context.Settings.SingleStep {
		timeStep = 0.0
	}

	if timeStep > 0.0 {
		point := b2.Vec2{
			X: FromFloat64(2.0 * s.amplitude * math.Cos(s.time)),
			Y: FromFloat64(s.amplitude * math.Sin(2.0*s.time)),
		}
		// The reference rotates by 2 * time radians.
		rotation := b2.MakeRot(FromFloat64(s.time / math.Pi))

		axis := b2.RotateVector(rotation, b2.Vec2{Y: b2.QOne()})
		halfAxis := axis.Mul(b2.QHalf())
		s.Context.Draw.DrawSegment(point.Sub(halfAxis), point.Add(halfAxis), b2.ColorPlum)
		s.Context.Draw.DrawPoint(point, b2.F(10), b2.ColorPlum)

		s.bodyId.SetTargetTransform(b2.Transform{P: point, Q: rotation}, FromFloat64(timeStep))
	}

	s.Base.Step()

	s.time += timeStep
}
