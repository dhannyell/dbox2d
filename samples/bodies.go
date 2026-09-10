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
func groundSegment(worldId dbox2d.WorldId, shapeDef *dbox2d.ShapeDef, halfWidth int) (dbox2d.BodyId, dbox2d.ShapeId) {
	bodyDef := dbox2d.DefaultBodyDef()
	groundId := dbox2d.CreateBody(worldId, &bodyDef)

	segment := dbox2d.Segment{
		Point1: dbox2d.Vec2{X: dbox2d.QFromInt(-halfWidth)},
		Point2: dbox2d.Vec2{X: dbox2d.QFromInt(halfWidth)},
	}
	shapeId := dbox2d.CreateSegmentShape(groundId, shapeDef, &segment)
	return groundId, shapeId
}

// BodyType switches a platform and its neighbors between static, kinematic
// and dynamic, and enables or disables them.
type BodyType struct {
	Base
	attachmentId       dbox2d.BodyId
	secondAttachmentId dbox2d.BodyId
	platformId         dbox2d.BodyId
	secondPayloadId    dbox2d.BodyId
	touchingBodyId     dbox2d.BodyId
	floatingBodyId     dbox2d.BodyId
	bodyType           dbox2d.BodyType
	speed              dbox2d.Q
	isEnabled          bool
}

// NewBodyType builds the body type scene.
func NewBodyType(ctx *SampleContext) Sample {
	s := &BodyType{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0.8, Y: 6.4}
		ctx.Camera.Zoom = 25.0 * 0.4
	}

	s.bodyType = dbox2d.DynamicBody
	s.isEnabled = true

	groundShapeDef := dbox2d.DefaultShapeDef()
	groundId, _ := groundSegment(s.WorldId, &groundShapeDef, 20)

	// Define attachment
	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = dbox2d.Vec2{X: dbox2d.QFromInt(-2), Y: dbox2d.QFromInt(3)}
		s.attachmentId = dbox2d.CreateBody(s.WorldId, &bodyDef)

		box := dbox2d.MakeBox(dbox2d.QHalf(), dbox2d.QFromInt(2))
		shapeDef := dbox2d.DefaultShapeDef()
		shapeDef.Density = dbox2d.QOne()
		dbox2d.CreatePolygonShape(s.attachmentId, &shapeDef, &box)
	}

	// Define second attachment
	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = s.bodyType
		bodyDef.IsEnabled = s.isEnabled
		bodyDef.Position = dbox2d.Vec2{X: dbox2d.QFromInt(3), Y: dbox2d.QFromInt(3)}
		s.secondAttachmentId = dbox2d.CreateBody(s.WorldId, &bodyDef)

		box := dbox2d.MakeBox(dbox2d.QHalf(), dbox2d.QFromInt(2))
		shapeDef := dbox2d.DefaultShapeDef()
		shapeDef.Density = dbox2d.QOne()
		dbox2d.CreatePolygonShape(s.secondAttachmentId, &shapeDef, &box)
	}

	// Define platform
	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = s.bodyType
		bodyDef.IsEnabled = s.isEnabled
		bodyDef.Position = dbox2d.Vec2{X: dbox2d.QFromInt(-4), Y: dbox2d.QFromInt(5)}
		s.platformId = dbox2d.CreateBody(s.WorldId, &bodyDef)

		// A quarter turn is the reference's 0.5 * pi.
		box := dbox2d.MakeOffsetBox(dbox2d.QHalf(), dbox2d.QFromInt(4), dbox2d.Vec2{X: dbox2d.QFromInt(4)}, dbox2d.MakeRot(dbox2d.QFromRatio(1, 4)))

		shapeDef := dbox2d.DefaultShapeDef()
		shapeDef.Density = dbox2d.QFromInt(2)
		dbox2d.CreatePolygonShape(s.platformId, &shapeDef, &box)

		revoluteDef := dbox2d.DefaultRevoluteJointDef()
		pivot := dbox2d.Vec2{X: dbox2d.QFromInt(-2), Y: dbox2d.QFromInt(5)}
		revoluteDef.BodyIdA = s.attachmentId
		revoluteDef.BodyIdB = s.platformId
		revoluteDef.LocalAnchorA = s.attachmentId.GetLocalPoint(pivot)
		revoluteDef.LocalAnchorB = s.platformId.GetLocalPoint(pivot)
		revoluteDef.MaxMotorTorque = dbox2d.QFromInt(50)
		revoluteDef.EnableMotor = true
		dbox2d.CreateRevoluteJoint(s.WorldId, &revoluteDef)

		pivot = dbox2d.Vec2{X: dbox2d.QFromInt(3), Y: dbox2d.QFromInt(5)}
		revoluteDef.BodyIdA = s.secondAttachmentId
		revoluteDef.BodyIdB = s.platformId
		revoluteDef.LocalAnchorA = s.secondAttachmentId.GetLocalPoint(pivot)
		revoluteDef.LocalAnchorB = s.platformId.GetLocalPoint(pivot)
		revoluteDef.MaxMotorTorque = dbox2d.QFromInt(50)
		revoluteDef.EnableMotor = true
		dbox2d.CreateRevoluteJoint(s.WorldId, &revoluteDef)

		prismaticDef := dbox2d.DefaultPrismaticJointDef()
		anchor := dbox2d.Vec2{Y: dbox2d.QFromInt(5)}
		prismaticDef.BodyIdA = groundId
		prismaticDef.BodyIdB = s.platformId
		prismaticDef.LocalAnchorA = groundId.GetLocalPoint(anchor)
		prismaticDef.LocalAnchorB = s.platformId.GetLocalPoint(anchor)
		prismaticDef.LocalAxisA = dbox2d.Vec2{X: dbox2d.QOne()}
		prismaticDef.MaxMotorForce = dbox2d.QFromInt(1000)
		prismaticDef.MotorSpeed = dbox2d.QZero()
		prismaticDef.EnableMotor = true
		prismaticDef.LowerTranslation = dbox2d.QFromInt(-10)
		prismaticDef.UpperTranslation = dbox2d.QFromInt(10)
		prismaticDef.EnableLimit = true
		dbox2d.CreatePrismaticJoint(s.WorldId, &prismaticDef)

		s.speed = dbox2d.QFromInt(3)
	}

	// Create a payload
	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = dbox2d.Vec2{X: dbox2d.QFromInt(-3), Y: dbox2d.QFromInt(8)}
		bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		box := dbox2d.MakeBox(dbox2d.QFromRatio(3, 4), dbox2d.QFromRatio(3, 4))
		shapeDef := dbox2d.DefaultShapeDef()
		shapeDef.Density = dbox2d.QFromInt(2)
		dbox2d.CreatePolygonShape(bodyId, &shapeDef, &box)
	}

	// Create a second payload
	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = s.bodyType
		bodyDef.IsEnabled = s.isEnabled
		bodyDef.Position = dbox2d.Vec2{X: dbox2d.QFromInt(2), Y: dbox2d.QFromInt(8)}
		s.secondPayloadId = dbox2d.CreateBody(s.WorldId, &bodyDef)

		box := dbox2d.MakeBox(dbox2d.QFromRatio(3, 4), dbox2d.QFromRatio(3, 4))
		shapeDef := dbox2d.DefaultShapeDef()
		shapeDef.Density = dbox2d.QFromInt(2)
		dbox2d.CreatePolygonShape(s.secondPayloadId, &shapeDef, &box)
	}

	// Create a separate body on the ground
	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = s.bodyType
		bodyDef.IsEnabled = s.isEnabled
		bodyDef.Position = dbox2d.Vec2{X: dbox2d.QFromInt(8), Y: dbox2d.QMustParse("0.2")}
		s.touchingBodyId = dbox2d.CreateBody(s.WorldId, &bodyDef)

		capsule := dbox2d.Capsule{
			Center2: dbox2d.Vec2{X: dbox2d.QOne()},
			Radius:  dbox2d.QFromRatio(1, 4),
		}
		shapeDef := dbox2d.DefaultShapeDef()
		shapeDef.Density = dbox2d.QFromInt(2)
		dbox2d.CreateCapsuleShape(s.touchingBodyId, &shapeDef, &capsule)
	}

	// Create a separate floating body
	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = s.bodyType
		bodyDef.IsEnabled = s.isEnabled
		bodyDef.Position = dbox2d.Vec2{X: dbox2d.QFromInt(-8), Y: dbox2d.QFromInt(12)}
		bodyDef.GravityScale = dbox2d.QZero()
		s.floatingBodyId = dbox2d.CreateBody(s.WorldId, &bodyDef)

		circle := dbox2d.Circle{Center: dbox2d.Vec2{Y: dbox2d.QHalf()}, Radius: dbox2d.QFromRatio(1, 4)}
		shapeDef := dbox2d.DefaultShapeDef()
		shapeDef.Density = dbox2d.QFromInt(2)
		dbox2d.CreateCircleShape(s.floatingBodyId, &shapeDef, &circle)
	}

	return s
}

func (s *BodyType) setType(bodyType dbox2d.BodyType) {
	s.bodyType = bodyType
	s.platformId.SetType(bodyType)
	if bodyType == dbox2d.KinematicBody {
		s.platformId.SetLinearVelocity(dbox2d.Vec2{X: s.speed.Neg()})
		s.platformId.SetAngularVelocity(dbox2d.QZero())
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

	if gui.RadioButton("Static", s.bodyType == dbox2d.StaticBody) {
		s.setType(dbox2d.StaticBody)
	}

	if gui.RadioButton("Kinematic", s.bodyType == dbox2d.KinematicBody) {
		s.setType(dbox2d.KinematicBody)
	}

	if gui.RadioButton("Dynamic", s.bodyType == dbox2d.DynamicBody) {
		s.setType(dbox2d.DynamicBody)
	}

	if gui.Checkbox("Enable", &s.isEnabled) {
		if s.isEnabled {
			s.platformId.Enable()
			s.secondAttachmentId.Enable()
			s.secondPayloadId.Enable()
			s.touchingBodyId.Enable()
			s.floatingBodyId.Enable()

			if s.bodyType == dbox2d.KinematicBody {
				s.platformId.SetLinearVelocity(dbox2d.Vec2{X: s.speed.Neg()})
				s.platformId.SetAngularVelocity(dbox2d.QZero())
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
	if s.bodyType == dbox2d.KinematicBody {
		p := s.platformId.GetPosition()
		v := s.platformId.GetLinearVelocity()

		if (p.X.Less(dbox2d.QFromInt(-14)) && v.X.Less(dbox2d.QZero())) ||
			(p.X.Greater(dbox2d.QFromInt(6)) && v.X.Greater(dbox2d.QZero())) {
			v.X = v.X.Neg()
			s.platformId.SetLinearVelocity(v)
		}
	}

	s.Base.Step()
}

func weebleFriction(dbox2d.Q, int, dbox2d.Q, int) dbox2d.Q {
	return dbox2d.QFromRatio(1, 10)
}

func weebleRestitution(dbox2d.Q, int, dbox2d.Q, int) dbox2d.Q {
	return dbox2d.QOne()
}

// Weeble is a capsule with its center of mass moved below the shape, so it
// always rights itself.
type Weeble struct {
	Base
	weebleId           dbox2d.BodyId
	explosionPosition  dbox2d.Vec2
	explosionRadius    dbox2d.Q
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

	groundShapeDef := dbox2d.DefaultShapeDef()
	groundSegment(s.WorldId, &groundShapeDef, 20)

	// Build weeble
	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = dbox2d.Vec2{Y: dbox2d.QFromInt(3)}
		// An eighth of a turn is the reference's 0.25 * pi.
		bodyDef.Rotation = dbox2d.MakeRot(dbox2d.QFromRatio(1, 8))
		s.weebleId = dbox2d.CreateBody(s.WorldId, &bodyDef)

		capsule := dbox2d.Capsule{
			Center1: dbox2d.Vec2{Y: dbox2d.QFromInt(-1)},
			Center2: dbox2d.Vec2{Y: dbox2d.QOne()},
			Radius:  dbox2d.QOne(),
		}
		shapeDef := dbox2d.DefaultShapeDef()
		dbox2d.CreateCapsuleShape(s.weebleId, &shapeDef, &capsule)

		mass := s.weebleId.GetMass()
		inertiaTensor := s.weebleId.GetRotationalInertia()

		offset := dbox2d.QMustParse("1.5")

		// See: https://en.wikipedia.org/wiki/Parallel_axis_theorem
		inertiaTensor = inertiaTensor.Add(mass.Mul(offset).Mul(offset))

		massData := dbox2d.MassData{Mass: mass, Center: dbox2d.Vec2{Y: offset.Neg()}, RotationalInertia: inertiaTensor}
		s.weebleId.SetMassData(massData)
	}

	s.explosionPosition = dbox2d.Vec2{}
	s.explosionRadius = dbox2d.QFromInt(2)
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
		s.weebleId.SetTransform(dbox2d.Vec2{Y: dbox2d.QFromInt(5)}, dbox2d.MakeRot(dbox2d.QMustParse("0.475")))
	}

	if gui.Button("Explode") {
		def := dbox2d.DefaultExplosionDef()
		def.Position = s.explosionPosition
		def.Radius = s.explosionRadius
		def.Falloff = dbox2d.QFromRatio(1, 10)
		def.ImpulsePerLength = FromFloat64(s.explosionMagnitude)
		s.WorldId.Explode(&def)
	}

	gui.SliderFloat("Magnitude", &s.explosionMagnitude, -100.0, 100.0)

	gui.End()
}

// Step advances the scene and draws the velocity of a point on the weeble.
func (s *Weeble) Step() {
	s.Base.Step()

	s.Context.Draw.DrawCircle(s.explosionPosition, s.explosionRadius, dbox2d.ColorAzure)

	// This shows how to get the velocity of a point on a body
	localPoint := dbox2d.Vec2{Y: dbox2d.QFromInt(2)}
	worldPoint := s.weebleId.GetWorldPoint(localPoint)

	v1 := s.weebleId.GetLocalPointVelocity(localPoint)
	v2 := s.weebleId.GetWorldPointVelocity(worldPoint)

	offset := dbox2d.Vec2{X: dbox2d.QFromRatio(1, 20)}
	s.Context.Draw.DrawSegment(worldPoint, worldPoint.Add(v1), dbox2d.ColorRed)
	s.Context.Draw.DrawSegment(worldPoint.Add(offset), worldPoint.Add(v2).Add(offset), dbox2d.ColorGreen)
}

// SleepBodies tests sleeping bodies, sensors on sleeping bodies, sleep
// tuning and waking on contact creation and destruction. It is the
// reference's Sleep; the name Sleep belongs to the benchmark.
type SleepBodies struct {
	Base
	pendulumId     dbox2d.BodyId
	staticBodyId   dbox2d.BodyId
	groundShapeId  dbox2d.ShapeId
	sensorIds      [2]dbox2d.ShapeId
	sensorTouching [2]bool
}

// NewSleepBodies builds the sleep scene.
func NewSleepBodies(ctx *SampleContext) Sample {
	s := &SleepBodies{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 3.0, Y: 50.0}
		ctx.Camera.Zoom = 25.0 * 2.2
	}

	groundShapeDef := dbox2d.DefaultShapeDef()
	groundShapeDef.EnableSensorEvents = true
	groundId, groundShapeId := groundSegment(s.WorldId, &groundShapeDef, 40)
	s.groundShapeId = groundShapeId

	// Sleeping body with sensors
	for i := range 2 {
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = dbox2d.Vec2{X: dbox2d.QFromInt(-4), Y: dbox2d.QFromInt(3 + 2*i)}
		bodyDef.IsAwake = false
		bodyDef.EnableSleep = true
		bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		capsule := dbox2d.Capsule{
			Center1: dbox2d.Vec2{Y: dbox2d.QOne()},
			Center2: dbox2d.Vec2{X: dbox2d.QOne(), Y: dbox2d.QOne()},
			Radius:  dbox2d.QFromRatio(3, 4),
		}
		shapeDef := dbox2d.DefaultShapeDef()
		dbox2d.CreateCapsuleShape(bodyId, &shapeDef, &capsule)

		shapeDef.IsSensor = true
		shapeDef.EnableSensorEvents = true
		capsule.Radius = dbox2d.QOne()
		s.sensorIds[i] = dbox2d.CreateCapsuleShape(bodyId, &shapeDef, &capsule)
		s.sensorTouching[i] = false
	}

	// Sleeping body but sleep is disabled
	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = dbox2d.Vec2{Y: dbox2d.QFromInt(3)}
		bodyDef.IsAwake = false
		bodyDef.EnableSleep = false
		bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		circle := dbox2d.Circle{Center: dbox2d.Vec2{X: dbox2d.QOne(), Y: dbox2d.QOne()}, Radius: dbox2d.QOne()}
		shapeDef := dbox2d.DefaultShapeDef()
		dbox2d.CreateCircleShape(bodyId, &shapeDef, &circle)
	}

	// Awake body and sleep is disabled
	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = dbox2d.Vec2{X: dbox2d.QFromInt(5), Y: dbox2d.QFromInt(3)}
		bodyDef.IsAwake = true
		bodyDef.EnableSleep = false
		bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		box := dbox2d.MakeOffsetBox(dbox2d.QOne(), dbox2d.QOne(), dbox2d.Vec2{Y: dbox2d.QOne()}, dbox2d.MakeRot(dbox2d.QFromRatio(1, 8)))
		shapeDef := dbox2d.DefaultShapeDef()
		dbox2d.CreatePolygonShape(bodyId, &shapeDef, &box)
	}

	// A sleeping body to test waking on collision
	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = dbox2d.Vec2{X: dbox2d.QFromInt(5), Y: dbox2d.QOne()}
		bodyDef.IsAwake = false
		bodyDef.EnableSleep = true
		bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		box := dbox2d.MakeSquare(dbox2d.QOne())
		shapeDef := dbox2d.DefaultShapeDef()
		dbox2d.CreatePolygonShape(bodyId, &shapeDef, &box)
	}

	// A long pendulum
	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = dbox2d.Vec2{Y: dbox2d.QFromInt(100)}
		bodyDef.AngularDamping = dbox2d.QHalf()
		bodyDef.SleepThreshold = dbox2d.QFromRatio(1, 20)
		s.pendulumId = dbox2d.CreateBody(s.WorldId, &bodyDef)

		capsule := dbox2d.Capsule{
			Center2: dbox2d.Vec2{X: dbox2d.QFromInt(90)},
			Radius:  dbox2d.QFromRatio(1, 4),
		}
		shapeDef := dbox2d.DefaultShapeDef()
		dbox2d.CreateCapsuleShape(s.pendulumId, &shapeDef, &capsule)

		pivot := bodyDef.Position
		jointDef := dbox2d.DefaultRevoluteJointDef()
		jointDef.BodyIdA = groundId
		jointDef.BodyIdB = s.pendulumId
		jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
		jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
		dbox2d.CreateRevoluteJoint(s.WorldId, &jointDef)
	}

	// A sleeping body to test waking on contact destroyed
	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = dbox2d.Vec2{X: dbox2d.QFromInt(-10), Y: dbox2d.QOne()}
		bodyDef.IsAwake = false
		bodyDef.EnableSleep = true
		bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		box := dbox2d.MakeSquare(dbox2d.QOne())
		shapeDef := dbox2d.DefaultShapeDef()
		dbox2d.CreatePolygonShape(bodyId, &shapeDef, &box)
	}

	s.staticBodyId = dbox2d.BodyId{}
	return s
}

func (s *SleepBodies) toggleInvoker() {
	if s.staticBodyId.IsNull() {
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Position = dbox2d.Vec2{X: dbox2d.QMustParse("-10.5"), Y: dbox2d.QFromInt(3)}
		s.staticBodyId = dbox2d.CreateBody(s.WorldId, &bodyDef)

		box := dbox2d.MakeOffsetBox(dbox2d.QFromInt(2), dbox2d.QFromRatio(1, 10), dbox2d.Vec2{}, dbox2d.MakeRot(dbox2d.QFromRatio(1, 8)))
		shapeDef := dbox2d.DefaultShapeDef()
		shapeDef.InvokeContactCreation = true
		dbox2d.CreatePolygonShape(s.staticBodyId, &shapeDef, &box)
	} else {
		dbox2d.DestroyBody(s.staticBodyId)
		s.staticBodyId = dbox2d.BodyId{}
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
	badBodyId dbox2d.BodyId
}

// NewBadBody builds the bad body scene.
func NewBadBody(ctx *SampleContext) Sample {
	s := &BadBody{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 2.3, Y: 10.0}
		ctx.Camera.Zoom = 25.0 * 0.5
	}

	groundShapeDef := dbox2d.DefaultShapeDef()
	groundSegment(s.WorldId, &groundShapeDef, 20)

	capsule := dbox2d.Capsule{
		Center1: dbox2d.Vec2{Y: dbox2d.QFromInt(-1)},
		Center2: dbox2d.Vec2{Y: dbox2d.QOne()},
		Radius:  dbox2d.QOne(),
	}

	// Build a bad body
	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = dbox2d.Vec2{Y: dbox2d.QFromInt(3)}
		bodyDef.AngularVelocity = radiansToTurns(0.5)
		bodyDef.Rotation = dbox2d.MakeRot(dbox2d.QFromRatio(1, 8))

		s.badBodyId = dbox2d.CreateBody(s.WorldId, &bodyDef)

		shapeDef := dbox2d.DefaultShapeDef()

		// density set to zero intentionally to create a bad body
		shapeDef.Density = dbox2d.QZero()
		dbox2d.CreateCapsuleShape(s.badBodyId, &shapeDef, &capsule)
	}

	// Build a normal body
	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = dbox2d.Vec2{X: dbox2d.QFromInt(2), Y: dbox2d.QFromInt(3)}
		bodyDef.Rotation = dbox2d.MakeRot(dbox2d.QFromRatio(1, 8))

		bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		shapeDef := dbox2d.DefaultShapeDef()
		dbox2d.CreateCapsuleShape(bodyId, &shapeDef, &capsule)
	}

	return s
}

// Step advances the scene and pushes the bad body up.
func (s *BadBody) Step() {
	s.Base.Step()

	s.DrawTextLine("A bad body is a dynamic body with no mass and behaves like a kinematic body.")
	s.DrawTextLine("Bad bodies are considered invalid and a user bug. Behavior is not guaranteed.")

	// For science
	s.badBodyId.ApplyForceToCenter(dbox2d.Vec2{Y: dbox2d.QFromInt(10)}, true)
}

// Pivot sets the initial angular velocity so that the bottom end of a lever
// stays at rest.
type Pivot struct {
	Base
	bodyId dbox2d.BodyId
	lever  dbox2d.Q
}

// NewPivot builds the pivot scene.
func NewPivot(ctx *SampleContext) Sample {
	s := &Pivot{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0.8, Y: 6.4}
		ctx.Camera.Zoom = 25.0 * 0.4
	}

	groundShapeDef := dbox2d.DefaultShapeDef()
	groundSegment(s.WorldId, &groundShapeDef, 20)

	// Create a separate body on the ground
	{
		v := dbox2d.Vec2{X: dbox2d.QFromInt(5)}

		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = dbox2d.Vec2{Y: dbox2d.QFromInt(3)}
		bodyDef.GravityScale = dbox2d.QOne()
		bodyDef.LinearVelocity = v

		s.bodyId = dbox2d.CreateBody(s.WorldId, &bodyDef)

		s.lever = dbox2d.QFromInt(3)
		r := dbox2d.Vec2{Y: s.lever.Neg()}

		// The reference's omega is in radians per second; the body takes turns.
		omega := dbox2d.Cross(v, r).Div(r.Dot(r))
		s.bodyId.SetAngularVelocity(omega.Div(dbox2d.Pi().Mul(dbox2d.QFromInt(2))))

		box := dbox2d.MakeBox(dbox2d.QFromRatio(1, 10), s.lever)
		shapeDef := dbox2d.DefaultShapeDef()
		dbox2d.CreatePolygonShape(s.bodyId, &shapeDef, &box)
	}

	return s
}

// Step advances the scene and reports the velocity of the pivot point.
func (s *Pivot) Step() {
	s.Base.Step()

	v := s.bodyId.GetLinearVelocity()
	omega := s.bodyId.GetAngularVelocity().Mul(dbox2d.Pi().Mul(dbox2d.QFromInt(2)))
	r := s.bodyId.GetWorldVector(dbox2d.Vec2{Y: s.lever.Neg()})

	vp := v.Add(dbox2d.CrossSV(omega, r))
	s.DrawTextLine("pivot velocity = (%g, %g)", ToFloat64(vp.X), ToFloat64(vp.Y))
}

// KinematicBody drives a kinematic body along a figure eight with target
// transforms. It is the reference's Kinematic; the name Kinematic belongs
// to the benchmark.
type KinematicBody struct {
	Base
	bodyId    dbox2d.BodyId
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
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.KinematicBody
		bodyDef.Position.X = FromFloat64(2.0 * s.amplitude)

		s.bodyId = dbox2d.CreateBody(s.WorldId, &bodyDef)

		box := dbox2d.MakeBox(dbox2d.QFromRatio(1, 10), dbox2d.QOne())
		shapeDef := dbox2d.DefaultShapeDef()
		dbox2d.CreatePolygonShape(s.bodyId, &shapeDef, &box)
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
		point := dbox2d.Vec2{
			X: FromFloat64(2.0 * s.amplitude * math.Cos(s.time)),
			Y: FromFloat64(s.amplitude * math.Sin(2.0*s.time)),
		}
		// The reference rotates by 2 * time radians.
		rotation := dbox2d.MakeRot(FromFloat64(s.time / math.Pi))

		axis := dbox2d.RotateVector(rotation, dbox2d.Vec2{Y: dbox2d.QOne()})
		halfAxis := axis.Mul(dbox2d.QHalf())
		s.Context.Draw.DrawSegment(point.Sub(halfAxis), point.Add(halfAxis), dbox2d.ColorPlum)
		s.Context.Draw.DrawPoint(point, dbox2d.QFromInt(10), dbox2d.ColorPlum)

		s.bodyId.SetTargetTransform(dbox2d.Transform{P: point, Q: rotation}, FromFloat64(timeStep))
	}

	s.Base.Step()

	s.time += timeStep
}
