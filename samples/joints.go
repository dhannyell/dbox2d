// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/sample_joints.cpp of Box2D v3.1.1. Debug-only sizes
// use the release values.

package samples

import (
	"fmt"
	"math"

	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/dbox2d/internal/shared"
)

func init() {
	RegisterSample("Joints", "Distance Joint", NewDistanceJoint)
	RegisterSample("Joints", "Motor Joint", NewMotorJoint)
	RegisterSample("Joints", "Top Down Friction", NewTopDownFriction)
	RegisterSample("Joints", "Filter Joint", NewFilterJoint)
	RegisterSample("Joints", "Revolute", NewRevoluteJoint)
	RegisterSample("Joints", "Prismatic", NewPrismaticJoint)
	RegisterSample("Joints", "Wheel", NewWheelJoint)
	RegisterSample("Joints", "Bridge", NewBridge)
	RegisterSample("Joints", "Ball & Chain", NewBallAndChain)
	RegisterSample("Joints", "Cantilever", NewCantilever)
	RegisterSample("Joints", "Motion Locks", NewMotionLocks)
	RegisterSample("Joints", "Breakable", NewBreakableJoint)
	RegisterSample("Joints", "Separation", NewJointSeparation)
	RegisterSample("Joints", "User Constraint", NewUserConstraint)
	RegisterSample("Joints", "Driving", NewDriving)
	RegisterSample("Joints", "Ragdoll", NewRagdoll)
	RegisterSample("Joints", "Soft Body", NewSoftBody)
	RegisterSample("Joints", "Doohickey", NewDoohickeyFarm)
	RegisterSample("Joints", "Scissor Lift", NewScissorLift)
	RegisterSample("Joints", "Gear Lift", NewGearLift)
	RegisterSample("Joints", "Door", NewDoor)
	RegisterSample("Joints", "Scale Ragdoll", NewScaleRagdoll)
	RegisterSample("Joints", "Theo Jansen", NewTheoJansen)
	RegisterSample("Joints", "Desk Lamp", NewDeskLamp)
}

// DistanceJoint demonstrates a configurable chain of bodies connected by distance joints.
type DistanceJoint struct {
	Base

	groundId b2.BodyId
	bodyIds  [10]b2.BodyId
	jointIds [10]b2.JointId
	count    int
	hertz    float64
	damping  float64
	length   float64
	// tensionForce and compressionForce bound the spring force.
	tensionForce     float64
	compressionForce float64
	minLength        float64
	maxLength        float64
	spring           bool
	limit            bool
}

// NewDistanceJoint builds the distance-joint chain scene.
func NewDistanceJoint(ctx *SampleContext) Sample {
	s := &DistanceJoint{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0.0, Y: 12.0}
		ctx.Camera.Zoom = 25.0 * 0.35
	}

	bodyDef := b2.DefaultBodyDef()
	s.groundId = b2.CreateBody(s.WorldId, &bodyDef)

	s.count = 0
	s.hertz = 5.0
	s.damping = 0.5
	s.tensionForce = 2000.0
	s.compressionForce = 100.0
	s.length = 1.0
	s.minLength = s.length
	s.maxLength = s.length
	s.spring = false
	s.limit = false

	for i := range s.bodyIds {
		s.bodyIds[i] = b2.BodyId{}
		s.jointIds[i] = b2.JointId{}
	}

	s.createScene(1)
	return s
}

func (s *DistanceJoint) createScene(newCount int) {
	// Joints must be destroyed before their attached bodies.
	for i := 0; i < s.count; i++ {
		b2.DestroyJoint(s.jointIds[i])
		s.jointIds[i] = b2.JointId{}
	}

	for i := 0; i < s.count; i++ {
		b2.DestroyBody(s.bodyIds[i])
		s.bodyIds[i] = b2.BodyId{}
	}

	s.count = newCount

	circle := b2.Circle{Radius: b2.QFromRatio(1, 4)}
	shapeDef := b2.DefaultShapeDef()
	shapeDef.Density = b2.F(20)
	yOffset := b2.F(20)

	jointDef := b2.DefaultDistanceJointDef()
	jointDef.Hertz = FromFloat64(s.hertz)
	jointDef.DampingRatio = FromFloat64(s.damping)
	jointDef.Length = FromFloat64(s.length)
	jointDef.MinLength = FromFloat64(s.minLength)
	jointDef.MaxLength = FromFloat64(s.maxLength)
	jointDef.EnableSpring = s.spring
	jointDef.LowerSpringForce = FromFloat64(-s.tensionForce)
	jointDef.UpperSpringForce = FromFloat64(s.compressionForce)
	jointDef.EnableLimit = s.limit

	prevBodyId := s.groundId
	for i := 0; i < s.count; i++ {
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.AngularDamping = b2.F(1)
		bodyDef.Position = b2.Vec2{
			X: FromFloat64(s.length * float64(i+1)),
			Y: yOffset,
		}
		s.bodyIds[i] = b2.CreateBody(s.WorldId, &bodyDef)
		b2.CreateCircleShape(s.bodyIds[i], &shapeDef, &circle)

		pivotA := b2.Vec2{X: FromFloat64(s.length * float64(i)), Y: yOffset}
		pivotB := b2.Vec2{X: FromFloat64(s.length * float64(i+1)), Y: yOffset}
		jointDef.Base.BodyIdA = prevBodyId
		jointDef.Base.BodyIdB = s.bodyIds[i]
		jointDef.Base.LocalFrameA.P = prevBodyId.GetLocalPoint(pivotA)
		jointDef.Base.LocalFrameB.P = s.bodyIds[i].GetLocalPoint(pivotB)
		s.jointIds[i] = b2.CreateDistanceJoint(s.WorldId, &jointDef)

		prevBodyId = s.bodyIds[i]
	}
}

// UpdateGui exposes the distance, spring, limit, and count controls.
func (s *DistanceJoint) UpdateGui() {
	height := 240
	gui := s.Context.Gui
	gui.Begin("Distance Joint", 10, s.Context.Camera.Height-height-50, 180, height)

	if gui.SliderFloat("Length", &s.length, 0.1, 4.0) {
		for i := 0; i < s.count; i++ {
			s.jointIds[i].SetRestLength(FromFloat64(s.length))
			s.jointIds[i].WakeBodies()
		}
	}

	if gui.Checkbox("Spring", &s.spring) {
		for i := 0; i < s.count; i++ {
			s.jointIds[i].EnableSpring(s.spring)
			s.jointIds[i].WakeBodies()
		}
	}

	if s.spring {
		if gui.SliderFloat("Hertz", &s.hertz, 0.0, 15.0) {
			for i := 0; i < s.count; i++ {
				s.jointIds[i].SetSpringHertz(FromFloat64(s.hertz))
				s.jointIds[i].WakeBodies()
			}
		}

		if gui.SliderFloat("Damping", &s.damping, 0.0, 4.0) {
			for i := 0; i < s.count; i++ {
				s.jointIds[i].SetSpringDampingRatio(FromFloat64(s.damping))
				s.jointIds[i].WakeBodies()
			}
		}

		if gui.SliderFloat("Tension", &s.tensionForce, 0.0, 4000.0) {
			for i := 0; i < s.count; i++ {
				s.jointIds[i].SetSpringForceRange(FromFloat64(-s.tensionForce), FromFloat64(s.compressionForce))
				s.jointIds[i].WakeBodies()
			}
		}

		if gui.SliderFloat("Compression", &s.compressionForce, 0.0, 200.0) {
			for i := 0; i < s.count; i++ {
				s.jointIds[i].SetSpringForceRange(FromFloat64(-s.tensionForce), FromFloat64(s.compressionForce))
				s.jointIds[i].WakeBodies()
			}
		}
	}

	if gui.Checkbox("Limit", &s.limit) {
		for i := 0; i < s.count; i++ {
			s.jointIds[i].EnableLimit(s.limit)
			s.jointIds[i].WakeBodies()
		}
	}

	if s.limit {
		if gui.SliderFloat("Min Length", &s.minLength, 0.1, 4.0) {
			for i := 0; i < s.count; i++ {
				s.jointIds[i].SetLimits(FromFloat64(s.minLength), FromFloat64(s.maxLength))
				s.jointIds[i].WakeBodies()
			}
		}

		if gui.SliderFloat("Max Length", &s.maxLength, 0.1, 4.0) {
			for i := 0; i < s.count; i++ {
				s.jointIds[i].SetLimits(FromFloat64(s.minLength), FromFloat64(s.maxLength))
				s.jointIds[i].WakeBodies()
			}
		}
	}

	count := s.count
	if gui.SliderInt("Count", &count, 1, 10) {
		s.createScene(count)
	}

	gui.End()
}

// MotorJoint demonstrates a motorized body following a kinematic target.
type MotorJoint struct {
	Base

	targetId  b2.BodyId
	bodyId    b2.BodyId
	jointId   b2.JointId
	transform b2.Transform
	time      float64
	speed     float64
	maxForce  float64
	maxTorque float64
}

// NewMotorJoint builds the motor-joint scene.
func NewMotorJoint(ctx *SampleContext) Sample {
	s := &MotorJoint{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0.0, Y: 7.0}
		ctx.Camera.Zoom = 25.0 * 0.4
	}

	groundBodyDef := b2.DefaultBodyDef()
	groundId := b2.CreateBody(s.WorldId, &groundBodyDef)
	groundShapeDef := b2.DefaultShapeDef()
	groundSegment := b2.Segment{
		Point1: b2.Vec2{X: FromFloat64(-20.0), Y: FromFloat64(0.0)},
		Point2: b2.Vec2{X: FromFloat64(20.0), Y: FromFloat64(0.0)},
	}
	b2.CreateSegmentShape(groundId, &groundShapeDef, &groundSegment)

	s.transform = b2.Transform{P: b2.Vec2{X: FromFloat64(0.0), Y: FromFloat64(8.0)}, Q: b2.RotIdentity()}

	// Define a target body
	targetDef := b2.DefaultBodyDef()
	targetDef.Type = b2.KinematicBody
	targetDef.Position = s.transform.P
	s.targetId = b2.CreateBody(s.WorldId, &targetDef)

	// Define motorized body
	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.Position = s.transform.P
	s.bodyId = b2.CreateBody(s.WorldId, &bodyDef)

	box := b2.MakeBox(FromFloat64(2.0), FromFloat64(0.5))
	shapeDef := b2.DefaultShapeDef()
	b2.CreatePolygonShape(s.bodyId, &shapeDef, &box)

	s.maxForce = 5000.0
	s.maxTorque = 500.0

	jointDef := b2.DefaultMotorJointDef()
	jointDef.Base.BodyIdA = s.targetId
	jointDef.Base.BodyIdB = s.bodyId
	jointDef.LinearHertz = FromFloat64(4.0)
	jointDef.LinearDampingRatio = FromFloat64(0.7)
	jointDef.AngularHertz = FromFloat64(4.0)
	jointDef.AngularDampingRatio = FromFloat64(0.7)
	jointDef.MaxSpringForce = FromFloat64(s.maxForce)
	jointDef.MaxSpringTorque = FromFloat64(s.maxTorque)
	s.jointId = b2.CreateMotorJoint(s.WorldId, &jointDef)

	// Define spring body
	springBodyDef := b2.DefaultBodyDef()
	springBodyDef.Type = b2.DynamicBody
	springBodyDef.Position = b2.Vec2{X: FromFloat64(-2.0), Y: FromFloat64(2.0)}
	springId := b2.CreateBody(s.WorldId, &springBodyDef)

	square := b2.MakeSquare(FromFloat64(0.5))
	springShapeDef := b2.DefaultShapeDef()
	b2.CreatePolygonShape(springId, &springShapeDef, &square)

	corner := b2.Vec2{X: FromFloat64(0.25), Y: FromFloat64(0.25)}
	springDef := b2.DefaultMotorJointDef()
	springDef.Base.BodyIdA = groundId
	springDef.Base.BodyIdB = springId
	springDef.Base.LocalFrameA.P = springBodyDef.Position.Add(corner)
	springDef.Base.LocalFrameB.P = corner
	springDef.LinearHertz = FromFloat64(7.5)
	springDef.LinearDampingRatio = FromFloat64(0.7)
	springDef.AngularHertz = FromFloat64(7.5)
	springDef.AngularDampingRatio = FromFloat64(0.7)
	springDef.MaxSpringForce = FromFloat64(500.0)
	springDef.MaxSpringTorque = FromFloat64(10.0)
	b2.CreateMotorJoint(s.WorldId, &springDef)

	s.speed = 1.0
	s.time = 0.0
	return s
}

func (s *MotorJoint) UpdateGui() {
	gui := s.Context.Gui
	height := 180
	gui.Begin("Motor Joint", 10, s.Context.Camera.Height-height-50, 240, height)

	gui.SliderFloat("Speed", &s.speed, -5.0, 5.0)
	if gui.SliderFloat("Max Force", &s.maxForce, 0.0, 10000.0) {
		s.jointId.SetMaxSpringForce(FromFloat64(s.maxForce))
	}
	if gui.SliderFloat("Max Torque", &s.maxTorque, 0.0, 10000.0) {
		s.jointId.SetMaxSpringTorque(FromFloat64(s.maxTorque))
	}
	if gui.Button("Apply Impulse") {
		s.bodyId.ApplyLinearImpulseToCenter(b2.Vec2{X: FromFloat64(100.0), Y: FromFloat64(0.0)}, true)
	}

	gui.End()
}

func (s *MotorJoint) Step() {
	timeStep := 0.0
	if s.Context.Settings.Hertz > 0 {
		timeStep = 1.0 / s.Context.Settings.Hertz
	}
	if s.Context.Settings.Pause && !s.Context.Settings.SingleStep {
		timeStep = 0.0
	}

	if timeStep > 0.0 {
		s.time += s.speed * timeStep

		linearOffset := b2.Vec2{
			X: FromFloat64(6.0 * math.Sin(2.0*s.time)),
			Y: FromFloat64(8.0 + 4.0*math.Sin(1.0*s.time)),
		}
		angularOffsetTurns := FromFloat64(2.0 * s.time / (2.0 * math.Pi))
		s.transform = b2.Transform{P: linearOffset, Q: b2.MakeRot(angularOffsetTurns)}

		s.targetId.SetTargetTransform(s.transform, FromFloat64(timeStep), true)
	}

	s.Context.Draw.DrawTransformFcn(s.transform)

	s.Base.Step()

	force := s.jointId.GetConstraintForce()
	torque := s.jointId.GetConstraintTorque()
	s.DrawTextLine("force = {%3.f, %3.f}, torque = %3.f", ToFloat64(force.X), ToFloat64(force.Y), ToFloat64(torque))
}

// FilterJoint demonstrates disabling collision between two bodies.
type FilterJoint struct {
	Base
}

// NewFilterJoint builds the filter-joint scene.
func NewFilterJoint(ctx *SampleContext) Sample {
	s := &FilterJoint{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0.0, Y: 7.0}
		ctx.Camera.Zoom = 25.0 * 0.4
	}

	groundBodyDef := b2.DefaultBodyDef()
	groundId := b2.CreateBody(s.WorldId, &groundBodyDef)
	groundShapeDef := b2.DefaultShapeDef()
	groundSegment := b2.Segment{
		Point1: b2.Vec2{X: FromFloat64(-20.0), Y: FromFloat64(0.0)},
		Point2: b2.Vec2{X: FromFloat64(20.0), Y: FromFloat64(0.0)},
	}
	b2.CreateSegmentShape(groundId, &groundShapeDef, &groundSegment)

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.Position = b2.Vec2{X: FromFloat64(-4.0), Y: FromFloat64(2.0)}
	body1 := b2.CreateBody(s.WorldId, &bodyDef)
	box := b2.MakeSquare(FromFloat64(2.0))
	shapeDef := b2.DefaultShapeDef()
	b2.CreatePolygonShape(body1, &shapeDef, &box)

	bodyDef.Position = b2.Vec2{X: FromFloat64(4.0), Y: FromFloat64(2.0)}
	body2 := b2.CreateBody(s.WorldId, &bodyDef)
	b2.CreatePolygonShape(body2, &shapeDef, &box)

	jointDef := b2.DefaultFilterJointDef()
	jointDef.Base.BodyIdA = body1
	jointDef.Base.BodyIdB = body2
	b2.CreateFilterJoint(s.WorldId, &jointDef)

	return s
}

// Revolute demonstrates limits, motors, and springs on revolute joints.
type RevoluteJoint struct {
	Base

	ball          b2.BodyId
	jointId1      b2.JointId
	jointId2      b2.JointId
	motorSpeed    float64
	motorTorque   float64
	hertz         float64
	dampingRatio  float64
	targetDegrees float64
	enableSpring  bool
	enableMotor   bool
	enableLimit   bool
}

// NewRevoluteJoint builds the revolute-joint scene.
func NewRevoluteJoint(ctx *SampleContext) Sample {
	s := &RevoluteJoint{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 15.5}
		ctx.Camera.Zoom = 25 * 0.7
	}

	var groundID b2.BodyId
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Position = b2.V2(0, -1)
		groundID = b2.CreateBody(s.WorldId, &bodyDef)

		box := b2.MakeBox(b2.F(40), b2.F(1))
		shapeDef := b2.DefaultShapeDef()
		b2.CreatePolygonShape(groundID, &shapeDef, &box)
	}

	s.enableSpring = false
	s.enableLimit = false
	s.enableMotor = false
	s.hertz = 2
	s.dampingRatio = 0.5
	s.targetDegrees = 45
	s.motorSpeed = 1
	s.motorTorque = 1000

	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.V2(-10, 20)
		bodyID := b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		shapeDef.Density = b2.QOne()
		capsule := b2.Capsule{
			Center1: b2.V2(0, -1),
			Center2: b2.V2(0, 6),
			Radius:  b2.QHalf(),
		}
		b2.CreateCapsuleShape(bodyID, &shapeDef, &capsule)

		pivot := b2.V2(-10.0, 20.5)
		jointDef := b2.DefaultRevoluteJointDef()
		jointDef.Base.BodyIdA = groundID
		jointDef.Base.BodyIdB = bodyID
		jointDef.Base.LocalFrameA.P = groundID.GetLocalPoint(pivot)
		jointDef.Base.LocalFrameB.P = bodyID.GetLocalPoint(pivot)
		// Convert the GUI's degrees to the public API's turns.
		jointDef.TargetAngle = FromFloat64(s.targetDegrees).Div(b2.F(360))
		jointDef.EnableSpring = s.enableSpring
		jointDef.Hertz = FromFloat64(s.hertz)
		jointDef.DampingRatio = FromFloat64(s.dampingRatio)
		jointDef.MotorSpeed = radiansToTurns(s.motorSpeed)
		jointDef.MaxMotorTorque = FromFloat64(s.motorTorque)
		jointDef.EnableMotor = s.enableMotor
		// 0.5*PI radians is 1/4 turn.
		jointDef.Base.LocalFrameA.Q = b2.MakeRot(b2.QFromRatio(1, 4))
		// -0.5*PI radians is -1/4 turn.
		jointDef.LowerAngle = b2.QFromRatio(-1, 4)
		// 0.05*PI radians is 1/40 turn.
		jointDef.UpperAngle = b2.QFromRatio(1, 40)
		jointDef.EnableLimit = s.enableLimit

		s.jointId1 = b2.CreateRevoluteJoint(s.WorldId, &jointDef)

		s.jointId1.SetConstraintTuning(b2.F(60), b2.F(20))
	}

	{
		circle := b2.Circle{Radius: b2.F(2)}
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.V2(5, 30)
		s.ball = b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		shapeDef.Density = b2.QOne()
		b2.CreateCircleShape(s.ball, &shapeDef, &circle)
	}

	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Position = b2.V2(20, 10)
		bodyDef.Type = b2.DynamicBody
		bodyID := b2.CreateBody(s.WorldId, &bodyDef)

		box := b2.MakeOffsetBox(
			b2.F(10), b2.F(0.5),
			b2.V2(-10, 0), b2.RotIdentity(),
		)
		shapeDef := b2.DefaultShapeDef()
		shapeDef.Density = b2.QOne()
		b2.CreatePolygonShape(bodyID, &shapeDef, &box)

		pivot := b2.V2(19, 10)
		jointDef := b2.DefaultRevoluteJointDef()
		jointDef.Base.BodyIdA = groundID
		jointDef.Base.BodyIdB = bodyID
		jointDef.Base.LocalFrameA.P = groundID.GetLocalPoint(pivot)
		jointDef.Base.LocalFrameB.P = bodyID.GetLocalPoint(pivot)
		// -0.25*PI radians is -1/8 turn; the upper limit is zero.
		jointDef.LowerAngle = b2.QFromRatio(-1, 8)
		jointDef.UpperAngle = b2.QZero()
		jointDef.EnableLimit = true
		jointDef.EnableMotor = true
		jointDef.MotorSpeed = b2.QZero()
		jointDef.MaxMotorTorque = FromFloat64(s.motorTorque)

		s.jointId2 = b2.CreateRevoluteJoint(s.WorldId, &jointDef)
	}

	return s
}

// UpdateGui updates the revolute-joint controls.
func (s *RevoluteJoint) UpdateGui() {
	height := 220
	gui := s.Context.Gui
	gui.Begin("Revolute Joint", 10, s.Context.Camera.Height-height-50, 240, height)

	if gui.Checkbox("Limit", &s.enableLimit) {
		s.jointId1.EnableLimit(s.enableLimit)
		s.jointId1.WakeBodies()
	}

	if gui.Checkbox("Motor", &s.enableMotor) {
		s.jointId1.EnableMotor(s.enableMotor)
		s.jointId1.WakeBodies()
	}

	if s.enableMotor {
		if gui.SliderFloat("Max Torque", &s.motorTorque, 0, 5000) {
			s.jointId1.SetMaxMotorTorque(FromFloat64(s.motorTorque))
			s.jointId1.WakeBodies()
		}

		if gui.SliderFloat("Speed", &s.motorSpeed, -20, 20) {
			s.jointId1.SetTargetMotorSpeed(radiansToTurns(s.motorSpeed))
			s.jointId1.WakeBodies()
		}
	}

	if gui.Checkbox("Spring", &s.enableSpring) {
		s.jointId1.EnableSpring(s.enableSpring)
		s.jointId1.WakeBodies()
	}

	if s.enableSpring {
		if gui.SliderFloat("Hertz", &s.hertz, 0, 30) {
			s.jointId1.SetSpringHertz(FromFloat64(s.hertz))
			s.jointId1.WakeBodies()
		}

		if gui.SliderFloat("Damping", &s.dampingRatio, 0, 2) {
			s.jointId1.SetSpringDampingRatio(FromFloat64(s.dampingRatio))
			s.jointId1.WakeBodies()
		}

		if gui.SliderFloat("Degrees", &s.targetDegrees, -180, 180) {
			// Convert the GUI's degrees to the public API's turns.
			targetAngle := FromFloat64(s.targetDegrees).Div(b2.F(360))
			s.jointId1.SetTargetAngle(targetAngle)
			s.jointId1.WakeBodies()
		}
	}

	gui.End()
}

// Step advances the scene and reports the joint angles and motor torques.
func (s *RevoluteJoint) Step() {
	s.Base.Step()

	angle1 := ToFloat64(s.jointId1.GetCurrentAngle()) * 360
	s.DrawTextLine("Angle (Deg) 1 = %2.1f", angle1)

	torque1 := ToFloat64(s.jointId1.GetCurrentMotorTorque())
	s.DrawTextLine("Motor Torque 1 = %4.1f", torque1)

	torque2 := ToFloat64(s.jointId2.GetCurrentMotorTorque())
	s.DrawTextLine("Motor Torque 2 = %4.1f", torque2)
}

// PrismaticJoint demonstrates limits, motors, and springs on prismatic joints.
type PrismaticJoint struct {
	Base

	jointId      b2.JointId
	motorSpeed   float64
	motorForce   float64
	hertz        float64
	dampingRatio float64
	translation  float64
	enableSpring bool
	enableMotor  bool
	enableLimit  bool
}

// NewPrismaticJoint builds the prismatic-joint scene.
func NewPrismaticJoint(ctx *SampleContext) Sample {
	s := &PrismaticJoint{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 8}
		ctx.Camera.Zoom = 25 * 0.5
	}

	var groundID b2.BodyId
	{
		bodyDef := b2.DefaultBodyDef()
		groundID = b2.CreateBody(s.WorldId, &bodyDef)
	}

	s.enableSpring = false
	s.enableLimit = true
	s.enableMotor = false
	s.motorSpeed = 2
	s.motorForce = 25
	s.hertz = 1
	s.dampingRatio = 0.5
	s.translation = 0

	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Position = b2.V2(0, 10)
		bodyDef.Type = b2.DynamicBody
		bodyID := b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		box := b2.MakeBox(b2.F(0.5), b2.F(2))
		b2.CreatePolygonShape(bodyID, &shapeDef, &box)

		pivot := b2.V2(0, 9)
		axis := (b2.Vec2{X: b2.QOne(), Y: b2.QOne()}).Normalize()
		jointDef := b2.DefaultPrismaticJointDef()
		jointDef.Base.BodyIdA = groundID
		jointDef.Base.BodyIdB = bodyID
		jointDef.Base.LocalFrameA.Q = b2.MakeRotFromUnitVector(groundID.GetLocalVector(axis))
		jointDef.Base.LocalFrameB.Q = jointDef.Base.LocalFrameA.Q
		jointDef.Base.LocalFrameA.P = groundID.GetLocalPoint(pivot)
		jointDef.Base.LocalFrameB.P = bodyID.GetLocalPoint(pivot)
		jointDef.Base.DrawScale = b2.F(2)
		jointDef.MotorSpeed = FromFloat64(s.motorSpeed)
		jointDef.MaxMotorForce = FromFloat64(s.motorForce)
		jointDef.EnableMotor = s.enableMotor
		jointDef.LowerTranslation = b2.F(-10)
		jointDef.UpperTranslation = b2.F(10)
		jointDef.EnableLimit = s.enableLimit
		jointDef.EnableSpring = s.enableSpring
		jointDef.Hertz = FromFloat64(s.hertz)
		jointDef.DampingRatio = FromFloat64(s.dampingRatio)

		s.jointId = b2.CreatePrismaticJoint(s.WorldId, &jointDef)
	}

	return s
}

// UpdateGui updates the prismatic-joint controls.
func (s *PrismaticJoint) UpdateGui() {
	height := 240
	gui := s.Context.Gui
	gui.Begin("Prismatic Joint", 10, s.Context.Camera.Height-height-50, 240, height)

	if gui.Checkbox("Limit", &s.enableLimit) {
		s.jointId.EnableLimit(s.enableLimit)
		s.jointId.WakeBodies()
	}

	if gui.Checkbox("Motor", &s.enableMotor) {
		s.jointId.EnableMotor(s.enableMotor)
		s.jointId.WakeBodies()
	}

	if s.enableMotor {
		if gui.SliderFloat("Max Force", &s.motorForce, 0, 200) {
			s.jointId.SetMaxMotorForce(FromFloat64(s.motorForce))
			s.jointId.WakeBodies()
		}

		if gui.SliderFloat("Speed", &s.motorSpeed, -40, 40) {
			s.jointId.SetTargetMotorSpeed(FromFloat64(s.motorSpeed))
			s.jointId.WakeBodies()
		}
	}

	if gui.Checkbox("Spring", &s.enableSpring) {
		s.jointId.EnableSpring(s.enableSpring)
		s.jointId.WakeBodies()
	}

	if s.enableSpring {
		if gui.SliderFloat("Hertz", &s.hertz, 0, 10) {
			s.jointId.SetSpringHertz(FromFloat64(s.hertz))
			s.jointId.WakeBodies()
		}

		if gui.SliderFloat("Damping", &s.dampingRatio, 0, 2) {
			s.jointId.SetSpringDampingRatio(FromFloat64(s.dampingRatio))
			s.jointId.WakeBodies()
		}

		if gui.SliderFloat("Translation", &s.translation, -15, 15) {
			s.jointId.SetTargetTranslation(FromFloat64(s.translation))
			s.jointId.WakeBodies()
		}
	}

	gui.End()
}

// Step advances the scene and reports the motor force and translation state.
func (s *PrismaticJoint) Step() {
	s.Base.Step()

	force := ToFloat64(s.jointId.GetCurrentMotorForce())
	s.DrawTextLine("Motor Force = %4.1f", force)

	translation := ToFloat64(s.jointId.GetCurrentTranslation())
	s.DrawTextLine("Translation = %4.1f", translation)

	speed := ToFloat64(s.jointId.GetCurrentSpeed())
	s.DrawTextLine("Speed = %4.8f", speed)
}

// WheelJoint demonstrates a wheel joint with configurable suspension, limits, and motor.
type WheelJoint struct {
	Base

	jointID      b2.JointId
	hertz        float64
	dampingRatio float64
	motorSpeed   float64
	motorTorque  float64
	enableSpring bool
	enableMotor  bool
	enableLimit  bool
}

// NewWheelJoint builds the wheel joint scene.
func NewWheelJoint(ctx *SampleContext) Sample {
	s := &WheelJoint{
		Base: NewBase(ctx),
	}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0.0, Y: 10.0}
		ctx.Camera.Zoom = 25.0 * 0.15
	}

	groundDef := b2.DefaultBodyDef()
	groundID := b2.CreateBody(s.WorldId, &groundDef)

	s.enableSpring = true
	s.enableLimit = true
	s.enableMotor = true
	s.motorSpeed = 2.0
	s.motorTorque = 5.0
	s.hertz = 1.0
	s.dampingRatio = 0.7

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Position = b2.V2(0.0, 10.25)
	bodyDef.Type = b2.DynamicBody
	bodyID := b2.CreateBody(s.WorldId, &bodyDef)

	shapeDef := b2.DefaultShapeDef()
	capsule := b2.Capsule{
		Center1: b2.Vec2{Y: b2.QHalf().Neg()},
		Center2: b2.Vec2{Y: b2.QHalf()},
		Radius:  b2.QHalf(),
	}
	b2.CreateCapsuleShape(bodyID, &shapeDef, &capsule)

	pivot := b2.V2(0, 10)
	axis := b2.Vec2{X: b2.QOne(), Y: b2.QOne()}.Normalize()
	jointDef := b2.DefaultWheelJointDef()
	jointDef.Base.BodyIdA = groundID
	jointDef.Base.BodyIdB = bodyID
	jointDef.Base.LocalFrameA.Q = b2.MakeRotFromUnitVector(groundID.GetLocalVector(axis))
	jointDef.Base.LocalFrameA.P = groundID.GetLocalPoint(pivot)
	jointDef.Base.LocalFrameB.P = bodyID.GetLocalPoint(pivot)
	jointDef.MotorSpeed = radiansToTurns(s.motorSpeed)
	jointDef.MaxMotorTorque = FromFloat64(s.motorTorque)
	jointDef.EnableMotor = s.enableMotor
	jointDef.LowerTranslation = b2.F(-3)
	jointDef.UpperTranslation = b2.F(3)
	jointDef.EnableLimit = s.enableLimit
	jointDef.Hertz = FromFloat64(s.hertz)
	jointDef.DampingRatio = FromFloat64(s.dampingRatio)

	s.jointID = b2.CreateWheelJoint(s.WorldId, &jointDef)

	return s
}

// UpdateGui exposes the wheel joint controls.
func (s *WheelJoint) UpdateGui() {
	gui := s.Context.Gui
	const height = 220
	if gui.Begin("Wheel Joint", 10, s.Context.Camera.Height-height-50, 240, height) {
		if gui.Checkbox("Limit", &s.enableLimit) {
			s.jointID.EnableLimit(s.enableLimit)
		}

		if gui.Checkbox("Motor", &s.enableMotor) {
			s.jointID.EnableMotor(s.enableMotor)
		}

		if s.enableMotor {
			if gui.SliderFloat("Torque", &s.motorTorque, 0.0, 20.0) {
				s.jointID.SetMaxMotorTorque(FromFloat64(s.motorTorque))
			}

			if gui.SliderFloat("Speed", &s.motorSpeed, -20.0, 20.0) {
				s.jointID.SetTargetMotorSpeed(radiansToTurns(s.motorSpeed))
			}
		}

		if gui.Checkbox("Spring", &s.enableSpring) {
			s.jointID.EnableSpring(s.enableSpring)
		}

		if s.enableSpring {
			if gui.SliderFloat("Hertz", &s.hertz, 0.0, 10.0) {
				s.jointID.SetSpringHertz(FromFloat64(s.hertz))
			}

			if gui.SliderFloat("Damping", &s.dampingRatio, 0.0, 2.0) {
				s.jointID.SetSpringDampingRatio(FromFloat64(s.dampingRatio))
			}
		}
	}
	gui.End()
}

// Step advances the world and reports the motor torque.
func (s *WheelJoint) Step() {
	s.Base.Step()

	torque := s.jointID.GetCurrentMotorTorque()
	s.DrawTextLine("Motor Torque = %4.1f", ToFloat64(torque))
}

const bridgeCount = 160

// Bridge demonstrates a suspension bridge made from revolute-jointed boxes.
type Bridge struct {
	Base
	bodyIDs                [bridgeCount]b2.BodyId
	jointIDs               [bridgeCount + 1]b2.JointId
	frictionTorque         float64
	constraintHertz        float64
	constraintDampingRatio float64
	springHertz            float64
	springDampingRatio     float64
}

// NewBridge builds the suspension bridge scene.
func NewBridge(ctx *SampleContext) Sample {
	s := &Bridge{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Zoom = 25.0 * 2.5
	}

	groundDef := b2.DefaultBodyDef()
	groundID := b2.CreateBody(s.WorldId, &groundDef)

	s.constraintHertz = 60.0
	s.constraintDampingRatio = 0.0
	s.springHertz = 2.0
	s.springDampingRatio = 0.7
	s.frictionTorque = 200.0

	box := b2.MakeBox(b2.QHalf(), b2.QFromRatio(1, 8))
	shapeDef := b2.DefaultShapeDef()
	shapeDef.Density = b2.F(20)

	jointDef := b2.DefaultRevoluteJointDef()
	jointDef.EnableMotor = true
	jointDef.MaxMotorTorque = FromFloat64(s.frictionTorque)
	jointDef.EnableSpring = true
	jointDef.Hertz = FromFloat64(s.springHertz)
	jointDef.DampingRatio = FromFloat64(s.springDampingRatio)

	jointIndex := 0
	xbase := b2.F(-80)
	prevBodyID := groundID
	for i := range bridgeCount {
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.Vec2{
			X: xbase.Add(b2.QHalf()).Add(b2.F(i)),
			Y: b2.F(20),
		}
		bodyDef.LinearDamping = b2.F(0.1)
		bodyDef.AngularDamping = b2.F(0.1)

		s.bodyIDs[i] = b2.CreateBody(s.WorldId, &bodyDef)
		b2.CreatePolygonShape(s.bodyIDs[i], &shapeDef, &box)

		pivot := b2.Vec2{X: xbase.Add(b2.F(i)), Y: b2.F(20)}
		jointDef.Base.BodyIdA = prevBodyID
		jointDef.Base.BodyIdB = s.bodyIDs[i]
		jointDef.Base.LocalFrameA.P = jointDef.Base.BodyIdA.GetLocalPoint(pivot)
		jointDef.Base.LocalFrameB.P = jointDef.Base.BodyIdB.GetLocalPoint(pivot)
		s.jointIDs[jointIndex] = b2.CreateRevoluteJoint(s.WorldId, &jointDef)

		prevBodyID = s.bodyIDs[i]
		jointIndex++
	}

	pivot := b2.Vec2{X: xbase.Add(b2.F(bridgeCount)), Y: b2.F(20)}
	jointDef.Base.BodyIdA = prevBodyID
	jointDef.Base.BodyIdB = groundID
	jointDef.Base.LocalFrameA.P = jointDef.Base.BodyIdA.GetLocalPoint(pivot)
	jointDef.Base.LocalFrameB.P = jointDef.Base.BodyIdB.GetLocalPoint(pivot)
	s.jointIDs[jointIndex] = b2.CreateRevoluteJoint(s.WorldId, &jointDef)

	for i := range 2 {
		vertices := []b2.Vec2{
			{X: b2.F(-0.5)},
			{X: b2.F(0.5)},
			{Y: b2.F(1.5)},
		}
		hull := b2.ComputeHull(vertices)
		triangle := b2.MakePolygon(&hull, b2.QZero())

		shapeDef := b2.DefaultShapeDef()
		shapeDef.Density = b2.F(20)
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.Vec2{X: b2.F(-8 + 8*i), Y: b2.F(22)}
		bodyID := b2.CreateBody(s.WorldId, &bodyDef)
		b2.CreatePolygonShape(bodyID, &shapeDef, &triangle)
	}

	for i := range 3 {
		circle := b2.Circle{Radius: b2.QHalf()}
		shapeDef := b2.DefaultShapeDef()
		shapeDef.Density = b2.F(20)
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.Vec2{X: b2.F(-6 + 6*i), Y: b2.F(25)}
		bodyID := b2.CreateBody(s.WorldId, &bodyDef)
		b2.CreateCircleShape(bodyID, &shapeDef, &circle)
	}

	return s
}

func (s *Bridge) UpdateGui() {
	const height = 180
	gui := s.Context.Gui
	gui.Begin("Bridge", 10, s.Context.Camera.Height-height-50, 320, height)
	if gui.SliderFloat("Joint Friction", &s.frictionTorque, 0.0, 10000.0) {
		for i := range s.jointIDs {
			s.jointIDs[i].SetMaxMotorTorque(FromFloat64(s.frictionTorque))
		}
	}
	if gui.SliderFloat("Spring hertz", &s.springHertz, 0.0, 30.0) {
		for i := range s.jointIDs {
			s.jointIDs[i].SetSpringHertz(FromFloat64(s.springHertz))
		}
	}
	if gui.SliderFloat("Spring damping", &s.springDampingRatio, 0.0, 2.0) {
		for i := range s.jointIDs {
			s.jointIDs[i].SetSpringDampingRatio(FromFloat64(s.springDampingRatio))
		}
	}
	if gui.SliderFloat("Constraint hertz", &s.constraintHertz, 15.0, 240.0) {
		for i := range s.jointIDs {
			s.jointIDs[i].SetConstraintTuning(FromFloat64(s.constraintHertz), FromFloat64(s.constraintDampingRatio))
		}
	}
	if gui.SliderFloat("Constraint damping", &s.constraintDampingRatio, 0.0, 10.0) {
		for i := range s.jointIDs {
			s.jointIDs[i].SetConstraintTuning(FromFloat64(s.constraintHertz), FromFloat64(s.constraintDampingRatio))
		}
	}
	gui.End()
}

const ballAndChainCount = 30

// BallAndChain demonstrates a capsule chain ending in a large ball.
type BallAndChain struct {
	Base
	jointIDs       [ballAndChainCount + 1]b2.JointId
	frictionTorque float64
}

// NewBallAndChain builds the ball and chain scene.
func NewBallAndChain(ctx *SampleContext) Sample {
	s := &BallAndChain{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0.0, Y: -8.0}
		ctx.Camera.Zoom = 27.5
	}

	groundDef := b2.DefaultBodyDef()
	groundID := b2.CreateBody(s.WorldId, &groundDef)

	s.frictionTorque = 100.0

	hx := b2.QHalf()
	capsule := b2.Capsule{
		Center1: b2.Vec2{X: hx.Neg()},
		Center2: b2.Vec2{X: hx},
		Radius:  b2.F(0.125),
	}

	shapeDef := b2.DefaultShapeDef()
	shapeDef.Density = b2.F(20)
	shapeDef.Filter.CategoryBits = 0x1
	shapeDef.Filter.MaskBits = 0x2
	jointDef := b2.DefaultRevoluteJointDef()

	jointIndex := 0
	prevBodyID := groundID
	for i := range ballAndChainCount {
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.Vec2{
			X: b2.F(1 + 2*i).Mul(hx),
			Y: b2.F(ballAndChainCount).Mul(hx),
		}
		bodyID := b2.CreateBody(s.WorldId, &bodyDef)
		b2.CreateCapsuleShape(bodyID, &shapeDef, &capsule)

		pivot := b2.Vec2{
			X: b2.F(2 * i).Mul(hx),
			Y: b2.F(ballAndChainCount).Mul(hx),
		}
		jointDef.Base.BodyIdA = prevBodyID
		jointDef.Base.BodyIdB = bodyID
		jointDef.Base.LocalFrameA.P = jointDef.Base.BodyIdA.GetLocalPoint(pivot)
		jointDef.Base.LocalFrameB.P = jointDef.Base.BodyIdB.GetLocalPoint(pivot)
		jointDef.EnableMotor = true
		jointDef.MaxMotorTorque = FromFloat64(s.frictionTorque)
		jointDef.EnableSpring = i > 0
		jointDef.Hertz = b2.F(4)
		s.jointIDs[jointIndex] = b2.CreateRevoluteJoint(s.WorldId, &jointDef)
		jointIndex++

		prevBodyID = bodyID
	}

	circle := b2.Circle{Radius: b2.F(4)}
	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.Position = b2.Vec2{
		X: b2.F(1 + 2*ballAndChainCount).Mul(hx).Add(circle.Radius).Sub(hx),
		Y: b2.F(ballAndChainCount).Mul(hx),
	}
	bodyID := b2.CreateBody(s.WorldId, &bodyDef)

	shapeDef.Filter.CategoryBits = 0x2
	shapeDef.Filter.MaskBits = 0x1
	b2.CreateCircleShape(bodyID, &shapeDef, &circle)

	pivot := b2.Vec2{
		X: b2.F(2 * ballAndChainCount).Mul(hx),
		Y: b2.F(ballAndChainCount).Mul(hx),
	}
	jointDef.Base.BodyIdA = prevBodyID
	jointDef.Base.BodyIdB = bodyID
	jointDef.Base.LocalFrameA.P = jointDef.Base.BodyIdA.GetLocalPoint(pivot)
	jointDef.Base.LocalFrameB.P = jointDef.Base.BodyIdB.GetLocalPoint(pivot)
	jointDef.EnableMotor = true
	jointDef.MaxMotorTorque = FromFloat64(s.frictionTorque)
	jointDef.EnableSpring = true
	jointDef.Hertz = b2.F(4)
	s.jointIDs[jointIndex] = b2.CreateRevoluteJoint(s.WorldId, &jointDef)

	return s
}

func (s *BallAndChain) UpdateGui() {
	const height = 60
	gui := s.Context.Gui
	gui.Begin("Ball and Chain", 10, s.Context.Camera.Height-height-50, 240, height)
	if gui.SliderFloat("Joint Friction", &s.frictionTorque, 0.0, 1000.0) {
		for i := range s.jointIDs {
			s.jointIDs[i].SetMaxMotorTorque(FromFloat64(s.frictionTorque))
		}
	}
	gui.End()
}

const cantileverCount = 8

// Cantilever builds a chain of capsule links attached to a static ground body.
type Cantilever struct {
	Base

	linearHertz         float64
	linearDampingRatio  float64
	angularHertz        float64
	angularDampingRatio float64
	gravityScale        float64
	collideConnected    bool

	tipID    b2.BodyId
	bodyIDs  [cantileverCount]b2.BodyId
	jointIDs [cantileverCount]b2.JointId
}

// NewCantilever builds the cantilever scene.
func NewCantilever(ctx *SampleContext) Sample {
	s := &Cantilever{
		Base:                NewBase(ctx),
		linearHertz:         15.0,
		linearDampingRatio:  0.5,
		angularHertz:        5.0,
		angularDampingRatio: 0.5,
		gravityScale:        1.0,
	}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{}
		ctx.Camera.Zoom = 25.0 * 0.35
	}

	groundDef := b2.DefaultBodyDef()
	groundID := b2.CreateBody(s.WorldId, &groundDef)

	hx := b2.QHalf()
	capsule := b2.Capsule{
		Center1: b2.Vec2{X: hx.Neg()},
		Center2: b2.Vec2{X: hx},
		Radius:  b2.F(0.125),
	}
	shapeDef := b2.DefaultShapeDef()
	shapeDef.Density = b2.F(20)

	jointDef := b2.DefaultWeldJointDef()
	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.IsAwake = false

	previousID := groundID
	for i := range cantileverCount {
		bodyDef.Position = b2.Vec2{
			X: b2.F(1 + 2*i).Mul(hx),
		}
		s.bodyIDs[i] = b2.CreateBody(s.WorldId, &bodyDef)
		b2.CreateCapsuleShape(s.bodyIDs[i], &shapeDef, &capsule)

		pivot := b2.Vec2{X: b2.F(2 * i).Mul(hx)}
		jointDef.Base.BodyIdA = previousID
		jointDef.Base.BodyIdB = s.bodyIDs[i]
		jointDef.Base.LocalFrameA.P = previousID.GetLocalPoint(pivot)
		jointDef.Base.LocalFrameB.P = s.bodyIDs[i].GetLocalPoint(pivot)
		jointDef.LinearHertz = FromFloat64(s.linearHertz)
		jointDef.LinearDampingRatio = FromFloat64(s.linearDampingRatio)
		jointDef.AngularHertz = FromFloat64(s.angularHertz)
		jointDef.AngularDampingRatio = FromFloat64(s.angularDampingRatio)
		jointDef.Base.CollideConnected = s.collideConnected
		s.jointIDs[i] = b2.CreateWeldJoint(s.WorldId, &jointDef)

		// Experimental tuning
		s.jointIDs[i].SetConstraintTuning(b2.F(120), b2.F(10))

		previousID = s.bodyIDs[i]
	}
	s.tipID = previousID

	return s
}

// UpdateGui exposes the live weld and gravity parameters.
func (s *Cantilever) UpdateGui() {
	gui := s.Context.Gui
	const height = 180
	if gui.Begin("Cantilever", 10, s.Context.Camera.Height-height-50, 240, height) {
		if gui.SliderFloat("Linear Hertz", &s.linearHertz, 0, 20) {
			for i := range s.jointIDs {
				s.jointIDs[i].SetLinearHertz(FromFloat64(s.linearHertz))
			}
		}
		if gui.SliderFloat("Linear Damping Ratio", &s.linearDampingRatio, 0, 10) {
			for i := range s.jointIDs {
				s.jointIDs[i].SetLinearDampingRatio(FromFloat64(s.linearDampingRatio))
			}
		}
		if gui.SliderFloat("Angular Hertz", &s.angularHertz, 0, 20) {
			for i := range s.jointIDs {
				s.jointIDs[i].SetAngularHertz(FromFloat64(s.angularHertz))
			}
		}
		if gui.SliderFloat("Angular Damping Ratio", &s.angularDampingRatio, 0, 10) {
			for i := range s.jointIDs {
				s.jointIDs[i].SetAngularDampingRatio(FromFloat64(s.angularDampingRatio))
			}
		}
		if gui.Checkbox("Collide Connected", &s.collideConnected) {
			for i := range s.jointIDs {
				s.jointIDs[i].SetCollideConnected(s.collideConnected)
			}
		}
		if gui.SliderFloat("Gravity Scale", &s.gravityScale, -1, 1) {
			for i := range s.bodyIDs {
				s.bodyIDs[i].SetGravityScale(FromFloat64(s.gravityScale))
			}
		}
	}
	gui.End()
}

// Step advances the world and reports the tip height.
func (s *Cantilever) Step() {
	s.Base.Step()

	position := s.tipID.GetPosition()
	s.DrawTextLine("tip-y = %.2f", ToFloat64(position.Y))
}

const motionLocksCount = 6

// MotionLocks mirrors the Motion Locks sample of sample_joints.cpp. It
// ensures joints work correctly with bodies that have motion locks, and lets
// the locks be toggled live.
type MotionLocks struct {
	Base

	bodyIDs     [motionLocksCount]b2.BodyId
	motionLocks b2.MotionLocks
}

// NewMotionLocks builds the motion-lock joint comparison scene.
func NewMotionLocks(ctx *SampleContext) Sample {
	s := &MotionLocks{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{Y: 8}
		ctx.Camera.Zoom = 25.0 * 0.7
	}

	groundDef := b2.DefaultBodyDef()
	groundID := b2.CreateBody(s.WorldId, &groundDef)

	s.motionLocks = b2.MotionLocks{AngularZ: true}

	position := b2.Vec2{X: b2.QFromRatio(-25, 2), Y: b2.F(10)}
	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.MotionLocks = s.motionLocks
	box := b2.MakeBox(b2.QOne(), b2.QOne())

	index := 0

	// Distance joint.
	bodyDef.Position = position
	s.bodyIDs[index] = b2.CreateBody(s.WorldId, &bodyDef)
	shapeDef := b2.DefaultShapeDef()
	b2.CreatePolygonShape(s.bodyIDs[index], &shapeDef, &box)
	length := b2.F(2)
	pivot1 := b2.Vec2{X: position.X, Y: position.Y.Add(b2.QOne()).Add(length)}
	pivot2 := b2.Vec2{X: position.X, Y: position.Y.Add(b2.QOne())}
	distanceDef := b2.DefaultDistanceJointDef()
	distanceDef.Base.BodyIdA = groundID
	distanceDef.Base.BodyIdB = s.bodyIDs[index]
	distanceDef.Base.LocalFrameA.P = groundID.GetLocalPoint(pivot1)
	distanceDef.Base.LocalFrameB.P = s.bodyIDs[index].GetLocalPoint(pivot2)
	distanceDef.Length = length
	b2.CreateDistanceJoint(s.WorldId, &distanceDef)

	position.X = position.X.Add(b2.F(5))
	index++

	// Motor joint.
	bodyDef.Position = position
	s.bodyIDs[index] = b2.CreateBody(s.WorldId, &bodyDef)
	shapeDef = b2.DefaultShapeDef()
	b2.CreatePolygonShape(s.bodyIDs[index], &shapeDef, &box)
	motorDef := b2.DefaultMotorJointDef()
	motorDef.Base.BodyIdA = groundID
	motorDef.Base.BodyIdB = s.bodyIDs[index]
	motorDef.Base.LocalFrameA.P = position
	motorDef.MaxVelocityForce = b2.F(200)
	motorDef.MaxVelocityTorque = b2.F(200)
	b2.CreateMotorJoint(s.WorldId, &motorDef)

	position.X = position.X.Add(b2.F(5))
	index++

	// Prismatic joint.
	bodyDef.Position = position
	s.bodyIDs[index] = b2.CreateBody(s.WorldId, &bodyDef)
	shapeDef = b2.DefaultShapeDef()
	b2.CreatePolygonShape(s.bodyIDs[index], &shapeDef, &box)
	pivot := b2.Vec2{X: position.X.Sub(b2.QOne()), Y: position.Y}
	prismaticDef := b2.DefaultPrismaticJointDef()
	prismaticDef.Base.BodyIdA = groundID
	prismaticDef.Base.BodyIdB = s.bodyIDs[index]
	prismaticDef.Base.LocalFrameA.P = groundID.GetLocalPoint(pivot)
	prismaticDef.Base.LocalFrameB.P = s.bodyIDs[index].GetLocalPoint(pivot)
	b2.CreatePrismaticJoint(s.WorldId, &prismaticDef)

	position.X = position.X.Add(b2.F(5))
	index++

	// Revolute joint.
	bodyDef.Position = position
	s.bodyIDs[index] = b2.CreateBody(s.WorldId, &bodyDef)
	shapeDef = b2.DefaultShapeDef()
	b2.CreatePolygonShape(s.bodyIDs[index], &shapeDef, &box)
	pivot = b2.Vec2{X: position.X.Sub(b2.QOne()), Y: position.Y}
	revoluteDef := b2.DefaultRevoluteJointDef()
	revoluteDef.Base.BodyIdA = groundID
	revoluteDef.Base.BodyIdB = s.bodyIDs[index]
	revoluteDef.Base.LocalFrameA.P = groundID.GetLocalPoint(pivot)
	revoluteDef.Base.LocalFrameB.P = s.bodyIDs[index].GetLocalPoint(pivot)
	b2.CreateRevoluteJoint(s.WorldId, &revoluteDef)

	position.X = position.X.Add(b2.F(5))
	index++

	// Weld joint.
	bodyDef.Position = position
	s.bodyIDs[index] = b2.CreateBody(s.WorldId, &bodyDef)
	shapeDef = b2.DefaultShapeDef()
	b2.CreatePolygonShape(s.bodyIDs[index], &shapeDef, &box)
	pivot = b2.Vec2{X: position.X.Sub(b2.QOne()), Y: position.Y}
	weldDef := b2.DefaultWeldJointDef()
	weldDef.Base.BodyIdA = groundID
	weldDef.Base.BodyIdB = s.bodyIDs[index]
	weldDef.Base.LocalFrameA.P = groundID.GetLocalPoint(pivot)
	weldDef.Base.LocalFrameB.P = s.bodyIDs[index].GetLocalPoint(pivot)
	weldDef.AngularHertz = b2.QOne()
	weldDef.AngularDampingRatio = b2.QHalf()
	weldDef.LinearHertz = b2.QOne()
	weldDef.LinearDampingRatio = b2.QHalf()
	b2.CreateWeldJoint(s.WorldId, &weldDef)

	position.X = position.X.Add(b2.F(5))
	index++

	// Wheel joint.
	bodyDef.Position = position
	s.bodyIDs[index] = b2.CreateBody(s.WorldId, &bodyDef)
	shapeDef = b2.DefaultShapeDef()
	b2.CreatePolygonShape(s.bodyIDs[index], &shapeDef, &box)
	pivot = b2.Vec2{X: position.X.Sub(b2.QOne()), Y: position.Y}
	wheelDef := b2.DefaultWheelJointDef()
	wheelDef.Base.BodyIdA = groundID
	wheelDef.Base.BodyIdB = s.bodyIDs[index]
	wheelDef.Base.LocalFrameA.P = groundID.GetLocalPoint(pivot)
	wheelDef.Base.LocalFrameB.P = s.bodyIDs[index].GetLocalPoint(pivot)
	wheelDef.Hertz = b2.QOne()
	wheelDef.DampingRatio = b2.QFromRatio(7, 10)
	wheelDef.LowerTranslation = b2.F(-1)
	wheelDef.UpperTranslation = b2.QOne()
	wheelDef.EnableLimit = true
	wheelDef.EnableMotor = true
	wheelDef.MaxMotorTorque = b2.F(10)
	wheelDef.MotorSpeed = radiansToTurns(1)
	b2.CreateWheelJoint(s.WorldId, &wheelDef)

	return s
}

// UpdateGui toggles the motion locks on all bodies and applies an impulse
// to the first body while L is held.
func (s *MotionLocks) UpdateGui() {
	gui := s.Context.Gui
	apply := false
	apply = gui.Checkbox("Lock Linear X", &s.motionLocks.LinearX) || apply
	apply = gui.Checkbox("Lock Linear Y", &s.motionLocks.LinearY) || apply
	apply = gui.Checkbox("Lock Angular Z", &s.motionLocks.AngularZ) || apply
	if apply {
		for i := range motionLocksCount {
			s.bodyIDs[i].SetMotionLocks(s.motionLocks)
			s.bodyIDs[i].SetAwake(true)
		}
	}

	if s.keyDown(KeyL) {
		s.bodyIDs[0].ApplyLinearImpulseToCenter(b2.Vec2{X: b2.F(100)}, true)
	}
}

const breakableJointCount = 6

// BreakableJoint demonstrates breaking joints when their constraint force is exceeded.
type BreakableJoint struct {
	Base
	jointIDs   [breakableJointCount]b2.JointId
	breakForce float64
}

// NewBreakableJoint builds the breakable-joint scene.
func NewBreakableJoint(ctx *SampleContext) Sample {
	s := &BreakableJoint{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 8}
		ctx.Camera.Zoom = 25 * 0.7
	}

	bodyDef := b2.DefaultBodyDef()
	groundID := b2.CreateBody(s.WorldId, &bodyDef)
	shapeDef := b2.DefaultShapeDef()
	segment := b2.Segment{Point1: b2.V2(-40, 0), Point2: b2.V2(40, 0)}
	b2.CreateSegmentShape(groundID, &shapeDef, &segment)

	box := b2.MakeBox(b2.QOne(), b2.QOne())
	bodyDef.Type = b2.DynamicBody
	bodyDef.EnableSleep = false
	position := b2.V2(-12.5, 10.0)
	index := 0

	bodyDef.Position = position
	bodyID := b2.CreateBody(s.WorldId, &bodyDef)
	b2.CreatePolygonShape(bodyID, &shapeDef, &box)
	length := b2.F(2)
	pivot1 := b2.Vec2{X: position.X, Y: position.Y.Add(b2.QOne()).Add(length)}
	pivot2 := b2.Vec2{X: position.X, Y: position.Y.Add(b2.QOne())}
	distanceDef := b2.DefaultDistanceJointDef()
	distanceDef.Base.BodyIdA, distanceDef.Base.BodyIdB = groundID, bodyID
	distanceDef.Base.LocalFrameA.P = groundID.GetLocalPoint(pivot1)
	distanceDef.Base.LocalFrameB.P = bodyID.GetLocalPoint(pivot2)
	distanceDef.Length = length
	distanceDef.Base.CollideConnected = true
	s.jointIDs[index] = b2.CreateDistanceJoint(s.WorldId, &distanceDef)

	position.X = position.X.Add(b2.F(5))
	index++
	bodyDef.Position = position
	bodyID = b2.CreateBody(s.WorldId, &bodyDef)
	b2.CreatePolygonShape(bodyID, &shapeDef, &box)
	motorDef := b2.DefaultMotorJointDef()
	motorDef.Base.BodyIdA, motorDef.Base.BodyIdB = groundID, bodyID
	motorDef.Base.LocalFrameA.P = position
	motorDef.MaxVelocityForce = b2.F(1000)
	motorDef.MaxVelocityTorque = b2.F(20)
	motorDef.Base.CollideConnected = true
	s.jointIDs[index] = b2.CreateMotorJoint(s.WorldId, &motorDef)

	for kind := range 4 {
		position.X = position.X.Add(b2.F(5))
		index++
		bodyDef.Position = position
		bodyID = b2.CreateBody(s.WorldId, &bodyDef)
		b2.CreatePolygonShape(bodyID, &shapeDef, &box)
		pivot := b2.Vec2{X: position.X.Sub(b2.QOne()), Y: position.Y}
		switch kind {
		case 0:
			def := b2.DefaultPrismaticJointDef()
			def.Base.BodyIdA, def.Base.BodyIdB = groundID, bodyID
			def.Base.LocalFrameA.P, def.Base.LocalFrameB.P = groundID.GetLocalPoint(pivot), bodyID.GetLocalPoint(pivot)
			def.Base.LocalFrameA.Q = b2.MakeRotFromUnitVector(groundID.GetLocalVector(b2.Vec2{X: b2.QOne()}))
			def.Base.LocalFrameB.Q = def.Base.LocalFrameA.Q
			def.Base.CollideConnected = true
			s.jointIDs[index] = b2.CreatePrismaticJoint(s.WorldId, &def)
		case 1:
			def := b2.DefaultRevoluteJointDef()
			def.Base.BodyIdA, def.Base.BodyIdB = groundID, bodyID
			def.Base.LocalFrameA.P, def.Base.LocalFrameB.P = groundID.GetLocalPoint(pivot), bodyID.GetLocalPoint(pivot)
			def.Base.CollideConnected = true
			s.jointIDs[index] = b2.CreateRevoluteJoint(s.WorldId, &def)
		case 2:
			def := b2.DefaultWeldJointDef()
			def.Base.BodyIdA, def.Base.BodyIdB = groundID, bodyID
			def.Base.LocalFrameA.P, def.Base.LocalFrameB.P = groundID.GetLocalPoint(pivot), bodyID.GetLocalPoint(pivot)
			def.Base.CollideConnected = true
			s.jointIDs[index] = b2.CreateWeldJoint(s.WorldId, &def)
		case 3:
			def := b2.DefaultWheelJointDef()
			def.Base.BodyIdA, def.Base.BodyIdB = groundID, bodyID
			def.Base.LocalFrameA.P, def.Base.LocalFrameB.P = groundID.GetLocalPoint(pivot), bodyID.GetLocalPoint(pivot)
			def.Base.LocalFrameA.Q = b2.MakeRotFromUnitVector(groundID.GetLocalVector(b2.Vec2{X: b2.QOne()}))
			def.Hertz, def.DampingRatio = b2.QOne(), b2.F(0.7)
			def.LowerTranslation, def.UpperTranslation = b2.F(-1), b2.QOne()
			def.EnableLimit, def.EnableMotor = true, true
			def.MaxMotorTorque, def.MotorSpeed = b2.F(10), radiansToTurns(1)
			def.Base.CollideConnected = true
			s.jointIDs[index] = b2.CreateWheelJoint(s.WorldId, &def)
		}
	}
	s.breakForce = 1000
	return s
}

func (s *BreakableJoint) UpdateGui() {
	height := 100
	gui := s.Context.Gui
	gui.Begin("Breakable Joint", 10, s.Context.Camera.Height-height-50, 240, height)
	gui.SliderFloat("break force", &s.breakForce, 0, 10000)
	gravity := s.WorldId.GetGravity()
	gravityY := ToFloat64(gravity.Y)
	if gui.SliderFloat("gravity", &gravityY, -50, 50) {
		gravity.Y = FromFloat64(gravityY)
		s.WorldId.SetGravity(gravity)
	}
	gui.End()
}

func (s *BreakableJoint) Step() {
	breakForce := FromFloat64(s.breakForce)
	threshold := breakForce.Mul(breakForce)
	for i := range s.jointIDs {
		if s.jointIDs[i].IsNull() {
			continue
		}
		force := s.jointIDs[i].GetConstraintForce()
		if force.LenSq().Greater(threshold) {
			b2.DestroyJoint(s.jointIDs[i])
			s.jointIDs[i] = b2.JointId{}
		} else {
			point := s.jointIDs[i].GetLocalFrameA().P
			s.Context.Draw.DrawStringFcn(point, fmt.Sprintf("(%.1f, %.1f)", ToFloat64(force.X), ToFloat64(force.Y)), b2.ColorWhite)
		}
	}
	s.Base.Step()
}

const jointSeparationCount = 5

// JointSeparation displays unresolved constraint separation for several joint types.
type JointSeparation struct {
	Base
	bodyIDs           [jointSeparationCount]b2.BodyId
	jointIDs          [jointSeparationCount]b2.JointId
	impulse           float64
	jointHertz        float64
	jointDampingRatio float64
}

// NewJointSeparation builds the joint-separation scene.
func NewJointSeparation(ctx *SampleContext) Sample {
	s := &JointSeparation{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 8}
		ctx.Camera.Zoom = 25
	}

	bodyDef := b2.DefaultBodyDef()
	groundID := b2.CreateBody(s.WorldId, &bodyDef)
	shapeDef := b2.DefaultShapeDef()
	segment := b2.Segment{Point1: b2.V2(-40, 0), Point2: b2.V2(40, 0)}
	b2.CreateSegmentShape(groundID, &shapeDef, &segment)
	bodyDef.Type = b2.DynamicBody
	bodyDef.EnableSleep = false
	box := b2.MakeBox(b2.QOne(), b2.QOne())
	position := b2.V2(-20, 10)
	for i := range s.bodyIDs {
		bodyDef.Position = position
		s.bodyIDs[i] = b2.CreateBody(s.WorldId, &bodyDef)
		b2.CreatePolygonShape(s.bodyIDs[i], &shapeDef, &box)
		pivot := b2.Vec2{X: position.X.Sub(b2.QOne()), Y: position.Y}
		switch i {
		case 0:
			def := b2.DefaultDistanceJointDef()
			def.Base.BodyIdA, def.Base.BodyIdB = groundID, s.bodyIDs[i]
			def.Base.LocalFrameA.P = groundID.GetLocalPoint(b2.Vec2{X: position.X, Y: position.Y.Add(b2.QOne()).Add(b2.F(2))})
			def.Base.LocalFrameB.P = s.bodyIDs[i].GetLocalPoint(b2.Vec2{X: position.X, Y: position.Y.Add(b2.QOne())})
			def.Length, def.Base.CollideConnected = b2.F(2), true
			s.jointIDs[i] = b2.CreateDistanceJoint(s.WorldId, &def)
		case 1:
			def := b2.DefaultPrismaticJointDef()
			def.Base.BodyIdA, def.Base.BodyIdB = groundID, s.bodyIDs[i]
			def.Base.LocalFrameA.P, def.Base.LocalFrameB.P = groundID.GetLocalPoint(pivot), s.bodyIDs[i].GetLocalPoint(pivot)
			def.Base.LocalFrameA.Q = b2.MakeRotFromUnitVector(groundID.GetLocalVector(b2.Vec2{X: b2.QOne()}))
			def.Base.LocalFrameB.Q = def.Base.LocalFrameA.Q
			def.Base.CollideConnected = true
			s.jointIDs[i] = b2.CreatePrismaticJoint(s.WorldId, &def)
		case 2:
			def := b2.DefaultRevoluteJointDef()
			def.Base.BodyIdA, def.Base.BodyIdB = groundID, s.bodyIDs[i]
			def.Base.LocalFrameA.P, def.Base.LocalFrameB.P = groundID.GetLocalPoint(pivot), s.bodyIDs[i].GetLocalPoint(pivot)
			def.Base.CollideConnected = true
			s.jointIDs[i] = b2.CreateRevoluteJoint(s.WorldId, &def)
		case 3:
			def := b2.DefaultWeldJointDef()
			def.Base.BodyIdA, def.Base.BodyIdB = groundID, s.bodyIDs[i]
			def.Base.LocalFrameA.P, def.Base.LocalFrameB.P = groundID.GetLocalPoint(pivot), s.bodyIDs[i].GetLocalPoint(pivot)
			def.Base.CollideConnected = true
			s.jointIDs[i] = b2.CreateWeldJoint(s.WorldId, &def)
		case 4:
			def := b2.DefaultWheelJointDef()
			def.Base.BodyIdA, def.Base.BodyIdB = groundID, s.bodyIDs[i]
			def.Base.LocalFrameA.P, def.Base.LocalFrameB.P = groundID.GetLocalPoint(pivot), s.bodyIDs[i].GetLocalPoint(pivot)
			def.Base.LocalFrameA.Q = b2.MakeRotFromUnitVector(groundID.GetLocalVector(b2.Vec2{X: b2.QOne()}))
			def.Hertz, def.DampingRatio = b2.QOne(), b2.F(0.7)
			def.LowerTranslation, def.UpperTranslation = b2.F(-1), b2.QOne()
			def.EnableLimit, def.EnableMotor = true, true
			def.MaxMotorTorque, def.MotorSpeed = b2.F(10), radiansToTurns(1)
			def.Base.CollideConnected = true
			s.jointIDs[i] = b2.CreateWheelJoint(s.WorldId, &def)
		}
		position.X = position.X.Add(b2.F(10))
	}
	s.impulse, s.jointHertz, s.jointDampingRatio = 500, 60, 2
	return s
}

func (s *JointSeparation) UpdateGui() {
	height := 180
	gui := s.Context.Gui
	gui.Begin("Joint Separation", 10, s.Context.Camera.Height-height-50, 260, height)
	gravity := s.WorldId.GetGravity()
	gravityY := ToFloat64(gravity.Y)
	if gui.SliderFloat("gravity", &gravityY, -500, 500) {
		gravity.Y = FromFloat64(gravityY)
		s.WorldId.SetGravity(gravity)
	}
	if gui.Button("impulse") {
		impulse := FromFloat64(s.impulse)
		for i := range s.bodyIDs {
			point := s.bodyIDs[i].GetWorldPoint(b2.Vec2{X: b2.QOne(), Y: b2.QOne()})
			s.bodyIDs[i].ApplyLinearImpulse(b2.Vec2{X: impulse, Y: impulse.Neg()}, point, true)
		}
	}
	gui.SliderFloat("magnitude", &s.impulse, 0, 1000)
	if gui.SliderFloat("hertz", &s.jointHertz, 15, 120) {
		for i := range s.jointIDs {
			s.jointIDs[i].SetConstraintTuning(FromFloat64(s.jointHertz), FromFloat64(s.jointDampingRatio))
		}
	}
	if gui.SliderFloat("damping", &s.jointDampingRatio, 0, 10) {
		for i := range s.jointIDs {
			s.jointIDs[i].SetConstraintTuning(FromFloat64(s.jointHertz), FromFloat64(s.jointDampingRatio))
		}
	}
	gui.End()
}

func (s *JointSeparation) Step() {
	for i := range s.jointIDs {
		if s.jointIDs[i].IsNull() {
			continue
		}
		linear := ToFloat64(s.jointIDs[i].GetLinearSeparation())
		angular := ToFloat64(s.jointIDs[i].GetAngularSeparation())
		point := s.jointIDs[i].GetLocalFrameA().P
		text := fmt.Sprintf("%.2f m, %.1f deg", linear, 360*angular)
		s.Context.Draw.DrawStringFcn(point, text, b2.ColorWhite)
	}
	s.Base.Step()
}

// UserConstraint hand-solves two soft constraints from one body to a fixed anchor.
type UserConstraint struct {
	Base
	bodyId   b2.BodyId
	impulses [2]b2.Q
}

// NewUserConstraint builds the hand-solved user-constraint scene.
func NewUserConstraint(ctx *SampleContext) Sample {
	s := &UserConstraint{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 3, Y: -1}
		ctx.Camera.Zoom = 25 * 0.15
	}

	box := b2.MakeBox(b2.QOne(), b2.QHalf())
	shapeDef := b2.DefaultShapeDef()
	shapeDef.Density = b2.F(20)

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.GravityScale = b2.QOne()
	bodyDef.AngularDamping = b2.QHalf()
	bodyDef.LinearDamping = b2.F(0.2)
	s.bodyId = b2.CreateBody(s.WorldId, &bodyDef)
	b2.CreatePolygonShape(s.bodyId, &shapeDef, &box)

	s.impulses = [2]b2.Q{}
	return s
}

// Step advances the scene and applies the hand-solved soft constraints.
func (s *UserConstraint) Step() {
	s.Base.Step()

	s.Context.Draw.DrawTransformFcn(b2.TransformIdentity())
	if s.Context.Settings.Pause {
		return
	}

	timeStep := b2.QZero()
	if s.Context.Settings.Hertz > 0 {
		timeStep = FromFloat64(1.0 / s.Context.Settings.Hertz)
	}
	if timeStep.Eq(b2.QZero()) {
		return
	}

	invTimeStep := FromFloat64(s.Context.Settings.Hertz)
	constraintHertz := b2.F(3)
	dampingRatioZeta := b2.F(0.7)
	maxForce := b2.F(1000)
	one := b2.QOne()
	zero := b2.QZero()
	omega := b2.Pi().Mul(b2.F(2)).Mul(constraintHertz)
	sigma := b2.F(2).Mul(dampingRatioZeta).Add(timeStep.Mul(omega))
	softness := timeStep.Mul(omega).Mul(sigma)
	impulseCoefficient := one.Div(one.Add(softness))
	massCoefficient := softness.Mul(impulseCoefficient)
	biasCoefficient := omega.Div(sigma)

	localAnchors := [2]b2.Vec2{
		{X: one, Y: b2.QHalf().Neg()},
		{X: one, Y: b2.QHalf()},
	}
	mass := s.bodyId.GetMass()
	threshold := b2.F(0.0001)
	invMass := zero
	if !mass.Less(threshold) {
		invMass = one.Div(mass)
	}
	inertiaTensor := s.bodyId.GetRotationalInertia()
	invI := zero
	if !inertiaTensor.Less(threshold) {
		invI = one.Div(inertiaTensor)
	}

	vB := s.bodyId.GetLinearVelocity()
	omegaB := s.bodyId.GetAngularVelocity()
	// The angular velocity is turns/s; the cross terms need rad/s.
	turnRadians := b2.Pi().Mul(b2.F(2))
	omegaRad := omegaB.Mul(turnRadians)
	pB := s.bodyId.GetWorldCenter()

	for i := range localAnchors {
		anchorA := b2.V2(3, 0)
		anchorB := s.bodyId.GetWorldPoint(localAnchors[i])
		deltaAnchor := anchorB.Sub(anchorA)

		slackLength := one
		length := deltaAnchor.Len()
		constraintError := length.Sub(slackLength)
		if constraintError.Less(zero) || length.Less(b2.F(0.001)) {
			s.Context.Draw.DrawLineFcn(anchorA, anchorB, b2.ColorLightCyan)
			s.impulses[i] = zero
			continue
		}

		s.Context.Draw.DrawLineFcn(anchorA, anchorB, b2.ColorViolet)
		axis := deltaAnchor.Normalize()
		rB := anchorB.Sub(pB)
		Jb := b2.Cross(rB, axis)
		K := invMass.Add(Jb.Mul(invI).Mul(Jb))
		invK := zero
		if !K.Less(threshold) {
			invK = one.Div(K)
		}

		Cdot := vB.Dot(axis).Add(Jb.Mul(omegaRad))
		impulse := massCoefficient.Mul(invK).Mul(Cdot.Add(biasCoefficient.Mul(constraintError))).Neg()
		appliedImpulse := impulse.Clamp(maxForce.Mul(timeStep).Neg(), zero)

		vB = b2.MulAdd(vB, invMass.Mul(appliedImpulse), axis)
		omegaRad = omegaRad.Add(appliedImpulse.Mul(invI).Mul(Jb))
		s.impulses[i] = appliedImpulse
	}

	s.bodyId.SetLinearVelocity(vB)
	omegaB = omegaRad.Div(turnRadians)
	s.bodyId.SetAngularVelocity(omegaB)
	s.DrawTextLine("forces = %s, %s", s.impulses[0].Mul(invTimeStep), s.impulses[1].Mul(invTimeStep))
}

// Door demonstrates a revolute joint with a spring, limit, and impulse controls.
type Door struct {
	Base
	doorId            b2.BodyId
	jointId           b2.JointId
	impulse           b2.Q
	impulseFloat      float64
	translationError  b2.Q
	jointHertz        b2.Q
	jointHertzFloat   float64
	jointDampingRatio b2.Q
	dampingRatioFloat float64
	enableLimit       bool
}

// NewDoor builds the revolute-joint door scene.
func NewDoor(ctx *SampleContext) Sample {
	s := &Door{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 0}
		ctx.Camera.Zoom = 4
	}

	groundDef := b2.DefaultBodyDef()
	groundId := b2.CreateBody(s.WorldId, &groundDef)

	s.enableLimit = true
	s.impulse = b2.F(50000)
	s.impulseFloat = 50000
	s.translationError = b2.QZero()
	s.jointHertz = b2.F(240)
	s.jointHertzFloat = 240
	s.jointDampingRatio = b2.QOne()
	s.dampingRatioFloat = 1

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.Position = b2.V2(0.0, 1.5)
	bodyDef.GravityScale = b2.QZero()
	s.doorId = b2.CreateBody(s.WorldId, &bodyDef)

	shapeDef := b2.DefaultShapeDef()
	shapeDef.Density = b2.F(1000)
	box := b2.MakeBox(b2.F(0.1), b2.F(1.5))
	b2.CreatePolygonShape(s.doorId, &shapeDef, &box)

	jointDef := b2.DefaultRevoluteJointDef()
	jointDef.Base.BodyIdA = groundId
	jointDef.Base.BodyIdB = s.doorId
	jointDef.Base.LocalFrameA.P = b2.Vec2{}
	jointDef.Base.LocalFrameB.P = b2.V2(0.0, -1.5)
	jointDef.TargetAngle = b2.QZero()
	jointDef.EnableSpring = true
	jointDef.Hertz = b2.QOne()
	jointDef.DampingRatio = b2.QHalf()
	jointDef.MotorSpeed = b2.QZero()
	jointDef.MaxMotorTorque = b2.QZero()
	jointDef.EnableMotor = false
	jointDef.LowerAngle = b2.QFromRatio(-1, 4)
	jointDef.UpperAngle = b2.QFromRatio(1, 4)
	jointDef.EnableLimit = s.enableLimit
	s.jointId = b2.CreateRevoluteJoint(s.WorldId, &jointDef)
	s.jointId.SetConstraintTuning(s.jointHertz, s.jointDampingRatio)

	return s
}

// UpdateGui exposes the door impulse, limit, and spring controls.
func (s *Door) UpdateGui() {
	const height = 220
	gui := s.Context.Gui
	gui.Begin("Door", 10, s.Context.Camera.Height-height-50, 240, height)

	if gui.Button("impulse") {
		point := s.doorId.GetWorldPoint(b2.V2(0.0, 1.5))
		s.doorId.ApplyLinearImpulse(b2.Vec2{X: s.impulse}, point, true)
		s.translationError = b2.QZero()
	}

	if gui.SliderFloat("magnitude", &s.impulseFloat, 1000, 100000) {
		s.impulse = FromFloat64(s.impulseFloat)
	}

	if gui.Checkbox("limit", &s.enableLimit) {
		s.jointId.EnableLimit(s.enableLimit)
	}

	if gui.SliderFloat("hertz", &s.jointHertzFloat, 15, 480) {
		s.jointHertz = FromFloat64(s.jointHertzFloat)
		s.jointId.SetConstraintTuning(s.jointHertz, s.jointDampingRatio)
	}

	if gui.SliderFloat("damping", &s.dampingRatioFloat, 0, 10) {
		s.jointDampingRatio = FromFloat64(s.dampingRatioFloat)
		s.jointId.SetConstraintTuning(s.jointHertz, s.jointDampingRatio)
	}

	gui.End()
}

// Step advances the door scene and reports its maximum translation error.
func (s *Door) Step() {
	s.Base.Step()

	point := s.doorId.GetWorldPoint(b2.V2(0.0, 1.5))
	s.Context.Draw.DrawPointFcn(point, b2.F(5), b2.ColorDarkKhaki)
	s.Context.Draw.DrawTransformFcn(b2.TransformIdentity())

	translationError := s.jointId.GetLinearSeparation()
	s.translationError = s.translationError.Max(translationError)
	s.DrawTextLine("translation error = %s", s.translationError)
}

// Ragdoll exposes the human joint tuning controls.
type Ragdoll struct {
	Base
	jointFrictionTorque      b2.Q
	jointFrictionTorqueFloat float64
	jointHertz               b2.Q
	jointHertzFloat          float64
	jointDampingRatio        b2.Q
	jointDampingRatioFloat   float64
	human                    shared.Human
}

// NewRagdoll builds the ragdoll scene.
func NewRagdoll(ctx *SampleContext) Sample {
	s := &Ragdoll{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 12}
		ctx.Camera.Zoom = 16
	}

	groundDef := b2.DefaultBodyDef()
	groundId := b2.CreateBody(s.WorldId, &groundDef)
	shapeDef := b2.DefaultShapeDef()
	segment := b2.Segment{
		Point1: b2.V2(-20, 0),
		Point2: b2.V2(20, 0),
	}
	b2.CreateSegmentShape(groundId, &shapeDef, &segment)

	s.jointFrictionTorque = b2.F(0.03)
	s.jointFrictionTorqueFloat = 0.03
	s.jointHertz = b2.F(5)
	s.jointHertzFloat = 5
	s.jointDampingRatio = b2.QHalf()
	s.jointDampingRatioFloat = 0.5

	s.Spawn()
	s.WorldId.SetContactTuning(b2.F(240), b2.QZero(), b2.F(2))
	return s
}

func (s *Ragdoll) Spawn() {
	s.human = shared.CreateHuman(
		s.WorldId,
		b2.V2(0, 25),
		b2.QOne(),
		s.jointFrictionTorque,
		s.jointHertz,
		s.jointDampingRatio,
		1,
		nil,
		false,
	)
}

// UpdateGui exposes the ragdoll joint controls.
func (s *Ragdoll) UpdateGui() {
	const height = 140
	gui := s.Context.Gui
	gui.Begin("Ragdoll", 10, s.Context.Camera.Height-height-50, 180, height)

	if gui.SliderFloat("Friction", &s.jointFrictionTorqueFloat, 0, 1) {
		s.jointFrictionTorque = FromFloat64(s.jointFrictionTorqueFloat)
		s.human.SetJointFrictionTorque(s.jointFrictionTorque)
	}
	if gui.SliderFloat("Hertz", &s.jointHertzFloat, 0, 10) {
		s.jointHertz = FromFloat64(s.jointHertzFloat)
		s.human.SetJointSpringHertz(s.jointHertz)
	}
	if gui.SliderFloat("Damping", &s.jointDampingRatioFloat, 0, 4) {
		s.jointDampingRatio = FromFloat64(s.jointDampingRatioFloat)
		s.human.SetJointDampingRatio(s.jointDampingRatio)
	}
	if gui.Button("Respawn") {
		s.human.Destroy()
		s.Spawn()
	}

	gui.End()
}

// ScaleRagdoll exposes the human scale control.
type ScaleRagdoll struct {
	Base
	scale      b2.Q
	scaleFloat float64
	human      shared.Human
}

// NewScaleRagdoll builds the scalable ragdoll scene.
func NewScaleRagdoll(ctx *SampleContext) Sample {
	s := &ScaleRagdoll{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 4.5}
		ctx.Camera.Zoom = 6
	}

	groundDef := b2.DefaultBodyDef()
	groundId := b2.CreateBody(s.WorldId, &groundDef)
	shapeDef := b2.DefaultShapeDef()
	box := b2.MakeOffsetBox(
		b2.F(20),
		b2.QOne(),
		b2.V2(0, -1),
		b2.RotIdentity(),
	)
	b2.CreatePolygonShape(groundId, &shapeDef, &box)

	s.scale = b2.QOne()
	s.scaleFloat = 1
	s.Spawn()
	return s
}

func (s *ScaleRagdoll) Spawn() {
	jointFrictionTorque := b2.F(0.03)
	jointHertz := b2.QOne()
	jointDampingRatio := b2.QHalf()
	s.human = shared.CreateHuman(
		s.WorldId,
		b2.V2(0, 5),
		s.scale,
		jointFrictionTorque,
		jointHertz,
		jointDampingRatio,
		1,
		nil,
		false,
	)
	s.human.ApplyRandomAngularImpulse(b2.QFromRatio(1, 10))
}

// UpdateGui exposes the ragdoll scale control.
func (s *ScaleRagdoll) UpdateGui() {
	const height = 60
	gui := s.Context.Gui
	gui.Begin("Scale Ragdoll", 10, s.Context.Camera.Height-height-50, 260, height)

	if gui.SliderFloat("Scale", &s.scaleFloat, 0.1, 10) {
		s.scale = FromFloat64(s.scaleFloat)
		s.human.SetScale(s.scale)
	}

	gui.End()
}

type Driving struct {
	Base
	car          car
	throttle     float64
	hertz        float64
	dampingRatio float64
	torque       float64
	speed        float64
}

func NewDriving(ctx *SampleContext) Sample {
	s := &Driving{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center.Y = 5
		ctx.Camera.Zoom = 25 * 0.4
		ctx.Settings.DrawJoints = false
	}

	groundDef := b2.DefaultBodyDef()
	groundID := b2.CreateBody(s.WorldId, &groundDef)
	{
		zero := b2.QZero()
		minusTwenty := b2.F(-20)
		twenty := b2.F(20)
		dx := b2.F(5)
		x := twenty
		hs := []b2.Q{
			b2.F(0.25), b2.QOne(), b2.F(4), zero, zero,
			b2.F(-1), b2.F(-2), b2.F(-2),
			b2.F(-1.25), zero,
		}
		// Filled in reverse to match the line list convention, as the reference.
		var points [25]b2.Vec2
		count := 24
		put := func(px, py b2.Q) {
			points[count] = b2.Vec2{X: px, Y: py}
			count--
		}
		put(minusTwenty, minusTwenty)
		put(minusTwenty, zero)
		put(twenty, zero)
		for range 2 {
			for i := range hs {
				put(x.Add(dx), hs[i])
				x = x.Add(dx)
			}
		}
		put(x.Add(b2.F(40)), zero)
		put(x.Add(b2.F(40)), minusTwenty)

		chainDef := b2.DefaultChainDef()
		chainDef.Points = points[:]
		chainDef.IsLoop = true
		b2.CreateChain(groundID, &chainDef)

		x = x.Add(b2.F(80))
		shapeDef := b2.DefaultShapeDef()
		segment := b2.Segment{
			Point1: b2.Vec2{X: x},
			Point2: b2.Vec2{X: x.Add(b2.F(40))},
		}
		b2.CreateSegmentShape(groundID, &shapeDef, &segment)

		x = x.Add(b2.F(40))
		segment = b2.Segment{
			Point1: b2.Vec2{X: x},
			Point2: b2.Vec2{X: x.Add(b2.F(10)), Y: b2.F(5)},
		}
		b2.CreateSegmentShape(groundID, &shapeDef, &segment)

		x = x.Add(b2.F(20))
		segment = b2.Segment{
			Point1: b2.Vec2{X: x},
			Point2: b2.Vec2{X: x.Add(b2.F(40))},
		}
		b2.CreateSegmentShape(groundID, &shapeDef, &segment)

		x = x.Add(b2.F(40))
		segment = b2.Segment{
			Point1: b2.Vec2{X: x},
			Point2: b2.Vec2{X: x, Y: b2.F(20)},
		}
		b2.CreateSegmentShape(groundID, &shapeDef, &segment)
	}

	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Position = b2.Vec2{X: b2.F(140), Y: b2.QOne()}
		bodyDef.AngularVelocity = radiansToTurns(1)
		bodyDef.Type = b2.DynamicBody
		bodyID := b2.CreateBody(s.WorldId, &bodyDef)
		shapeDef := b2.DefaultShapeDef()
		box := b2.MakeBox(b2.F(10), b2.F(0.25))
		b2.CreatePolygonShape(bodyID, &shapeDef, &box)

		jointDef := b2.DefaultRevoluteJointDef()
		jointDef.Base.BodyIdA = groundID
		jointDef.Base.BodyIdB = bodyID
		jointDef.Base.LocalFrameA.P = groundID.GetLocalPoint(bodyDef.Position)
		jointDef.Base.LocalFrameB.P = bodyID.GetLocalPoint(bodyDef.Position)
		jointDef.LowerAngle = b2.QFromRatio(-8, 360)
		jointDef.UpperAngle = b2.QFromRatio(8, 360)
		jointDef.EnableLimit = true
		b2.CreateRevoluteJoint(s.WorldId, &jointDef)
	}

	{
		shapeDef := b2.DefaultShapeDef()
		capsule := b2.Capsule{
			Center1: b2.V2(-1, 0),
			Center2: b2.Vec2{X: b2.QOne()},
			Radius:  b2.F(0.125),
		}
		jointDef := b2.DefaultRevoluteJointDef()
		prevBodyID := groundID
		for i := range 20 {
			bodyDef := b2.DefaultBodyDef()
			bodyDef.Type = b2.DynamicBody
			bodyDef.Position = b2.Vec2{
				X: b2.F(161).Add(b2.F(2 * i)),
				Y: b2.F(-0.125),
			}
			bodyID := b2.CreateBody(s.WorldId, &bodyDef)
			b2.CreateCapsuleShape(bodyID, &shapeDef, &capsule)

			pivot := b2.Vec2{
				X: b2.F(160).Add(b2.F(2 * i)),
				Y: b2.F(-0.125),
			}
			jointDef.Base.BodyIdA = prevBodyID
			jointDef.Base.BodyIdB = bodyID
			jointDef.Base.LocalFrameA.P = prevBodyID.GetLocalPoint(pivot)
			jointDef.Base.LocalFrameB.P = bodyID.GetLocalPoint(pivot)
			b2.CreateRevoluteJoint(s.WorldId, &jointDef)
			prevBodyID = bodyID
		}

		pivot := b2.V2(200.0, -0.125)
		jointDef.Base.BodyIdA = prevBodyID
		jointDef.Base.BodyIdB = groundID
		jointDef.Base.LocalFrameA.P = prevBodyID.GetLocalPoint(pivot)
		jointDef.Base.LocalFrameB.P = groundID.GetLocalPoint(pivot)
		jointDef.EnableMotor = true
		jointDef.MaxMotorTorque = b2.F(50)
		b2.CreateRevoluteJoint(s.WorldId, &jointDef)
	}

	{
		box := b2.MakeBox(b2.QHalf(), b2.QHalf())
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		shapeDef := b2.DefaultShapeDef()
		shapeDef.Material.Friction = b2.F(0.25)
		shapeDef.Material.Restitution = b2.F(0.25)
		shapeDef.Density = b2.F(0.25)
		for i := range 5 {
			bodyDef.Position = b2.Vec2{X: b2.F(230), Y: b2.F(i).Add(b2.QHalf())}
			bodyID := b2.CreateBody(s.WorldId, &bodyDef)
			b2.CreatePolygonShape(bodyID, &shapeDef, &box)
		}
	}

	s.throttle = 0
	s.speed = 35
	s.torque = 5
	s.hertz = 5
	s.dampingRatio = 0.7
	s.car.spawn(s.WorldId, b2.Vec2{}, b2.QOne(), FromFloat64(s.hertz), FromFloat64(s.dampingRatio), FromFloat64(s.torque), nil)
	return s
}

func (s *Driving) UpdateGui() {
	const height = 140
	gui := s.Context.Gui
	gui.Begin("Driving", 10, s.Context.Camera.Height-height-50, 200, height)
	if gui.SliderFloat("Spring Hertz", &s.hertz, 0, 20) {
		s.car.setHertz(FromFloat64(s.hertz))
	}
	if gui.SliderFloat("Damping Ratio", &s.dampingRatio, 0, 10) {
		s.car.setDampingRatio(FromFloat64(s.dampingRatio))
	}
	if gui.SliderFloat("Speed", &s.speed, 0, 50) {
		s.car.setSpeed(FromFloat64(s.throttle * s.speed))
	}
	if gui.SliderFloat("Torque", &s.torque, 0, 10) {
		s.car.setTorque(FromFloat64(s.torque))
	}
	gui.End()
}

func (s *Driving) Step() {
	if s.keyDown(KeyA) {
		s.throttle = 1
		s.car.setSpeed(FromFloat64(s.speed))
	}
	if s.keyDown(KeyS) {
		s.throttle = 0
		s.car.setSpeed(b2.QZero())
	}
	if s.keyDown(KeyD) {
		s.throttle = -1
		s.car.setSpeed(FromFloat64(-s.speed))
	}

	s.DrawTextLine("Keys: left = a, brake = s, right = d")
	linearVelocity := s.car.chassisId.GetLinearVelocity()
	kph := linearVelocity.X.Mul(b2.F(3.6))
	s.DrawTextLine("speed in kph: %.2g", ToFloat64(kph))
	s.Context.Camera.Center.X = ToFloat64(s.car.chassisId.GetPosition().X)
	s.Base.Step()
}

type SoftBody struct {
	Base
	donut donut
}

func NewSoftBody(ctx *SampleContext) Sample {
	s := &SoftBody{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 5}
		ctx.Camera.Zoom = 25 * 0.25
	}

	groundDef := b2.DefaultBodyDef()
	groundID := b2.CreateBody(s.WorldId, &groundDef)
	shapeDef := b2.DefaultShapeDef()
	segment := b2.Segment{
		Point1: b2.V2(-20, 0),
		Point2: b2.V2(20, 0),
	}
	b2.CreateSegmentShape(groundID, &shapeDef, &segment)

	s.donut.create(
		s.WorldId,
		b2.V2(0, 10),
		b2.F(2),
		0,
		false,
		nil,
	)
	return s
}

type DoohickeyFarm struct {
	Base
}

func NewDoohickeyFarm(ctx *SampleContext) Sample {
	s := &DoohickeyFarm{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 5}
		ctx.Camera.Zoom = 25 * 0.35
	}

	groundDef := b2.DefaultBodyDef()
	groundID := b2.CreateBody(s.WorldId, &groundDef)
	shapeDef := b2.DefaultShapeDef()
	segment := b2.Segment{
		Point1: b2.V2(-20, 0),
		Point2: b2.V2(20, 0),
	}
	b2.CreateSegmentShape(groundID, &shapeDef, &segment)

	box := b2.MakeOffsetBox(
		b2.QOne(),
		b2.QOne(),
		b2.Vec2{Y: b2.QOne()},
		b2.RotIdentity(),
	)
	b2.CreatePolygonShape(groundID, &shapeDef, &box)

	y := b2.F(4)
	for range 4 {
		d := doohickey{}
		d.spawn(s.WorldId, b2.Vec2{Y: y}, b2.QHalf())
		y = y.Add(b2.F(2))
	}

	return s
}

// ScissorLift builds a three-link lift with a distance-joint motor.
type ScissorLift struct {
	Base

	liftJointID b2.JointId
	motorForce  float64
	motorSpeed  float64
	enableMotor bool
}

// NewScissorLift builds the scissor-lift scene.
func NewScissorLift(ctx *SampleContext) Sample {
	s := &ScissorLift{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 9}
		ctx.Camera.Zoom = 25 * 0.4
	}

	// Need 8 sub-steps for smoother operation.
	ctx.Settings.SubStepCount = 8

	constraintDampingRatio := b2.F(20)
	constraintHertz := b2.F(240)

	groundDef := b2.DefaultBodyDef()
	groundID := b2.CreateBody(s.WorldId, &groundDef)
	shapeDef := b2.DefaultShapeDef()
	segment := b2.Segment{
		Point1: b2.V2(-20, 0),
		Point2: b2.V2(20, 0),
	}
	b2.CreateSegmentShape(groundID, &shapeDef, &segment)

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.SleepThreshold = b2.F(0.01)
	capsule := b2.Capsule{
		Center1: b2.V2(-2.5, 0.0),
		Center2: b2.V2(2.5, 0.0),
		Radius:  b2.F(0.15),
	}

	baseID1 := groundID
	baseID2 := groundID
	baseAnchor1 := b2.V2(-2.5, 0.2)
	baseAnchor2 := b2.V2(2.5, 0.2)
	y := b2.F(0.5)
	var linkID1 b2.BodyId

	for i := range 3 {
		bodyDef.Position = b2.Vec2{Y: y}
		bodyDef.Rotation = b2.MakeRot(radiansToTurns(0.15))
		bodyID1 := b2.CreateBody(s.WorldId, &bodyDef)
		b2.CreateCapsuleShape(bodyID1, &shapeDef, &capsule)

		bodyDef.Position = b2.Vec2{Y: y}
		bodyDef.Rotation = b2.MakeRot(radiansToTurns(0.15).Neg())
		bodyID2 := b2.CreateBody(s.WorldId, &bodyDef)
		b2.CreateCapsuleShape(bodyID2, &shapeDef, &capsule)

		if i == 1 {
			linkID1 = bodyID2
		}

		revoluteDef := b2.DefaultRevoluteJointDef()
		revoluteDef.Base.BodyIdA = baseID1
		revoluteDef.Base.BodyIdB = bodyID1
		revoluteDef.Base.LocalFrameA.P = baseAnchor1
		revoluteDef.Base.LocalFrameB.P = b2.V2(-2.5, 0.0)
		revoluteDef.Base.CollideConnected = i == 0
		revoluteDef.Base.ConstraintDampingRatio = constraintDampingRatio
		revoluteDef.Base.ConstraintHertz = constraintHertz
		b2.CreateRevoluteJoint(s.WorldId, &revoluteDef)

		if i == 0 {
			wheelDef := b2.DefaultWheelJointDef()
			wheelDef.Base.BodyIdA = baseID2
			wheelDef.Base.BodyIdB = bodyID2
			wheelDef.Base.LocalFrameA.Q = b2.MakeRotFromUnitVector(b2.Vec2{X: b2.QOne()})
			wheelDef.Base.LocalFrameA.P = baseAnchor2
			wheelDef.Base.LocalFrameB.P = b2.V2(2.5, 0.0)
			wheelDef.EnableSpring = false
			wheelDef.Base.CollideConnected = true
			wheelDef.Base.ConstraintDampingRatio = constraintDampingRatio
			wheelDef.Base.ConstraintHertz = constraintHertz
			b2.CreateWheelJoint(s.WorldId, &wheelDef)
		} else {
			revoluteDef.Base.BodyIdA = baseID2
			revoluteDef.Base.BodyIdB = bodyID2
			revoluteDef.Base.LocalFrameA.P = baseAnchor2
			revoluteDef.Base.LocalFrameB.P = b2.V2(2.5, 0.0)
			revoluteDef.Base.CollideConnected = false
			revoluteDef.Base.ConstraintDampingRatio = constraintDampingRatio
			revoluteDef.Base.ConstraintHertz = constraintHertz
			b2.CreateRevoluteJoint(s.WorldId, &revoluteDef)
		}

		revoluteDef.Base.BodyIdA = bodyID1
		revoluteDef.Base.BodyIdB = bodyID2
		revoluteDef.Base.LocalFrameA.P = b2.Vec2{}
		revoluteDef.Base.LocalFrameB.P = b2.Vec2{}
		revoluteDef.Base.CollideConnected = false
		revoluteDef.Base.ConstraintDampingRatio = constraintDampingRatio
		revoluteDef.Base.ConstraintHertz = constraintHertz
		b2.CreateRevoluteJoint(s.WorldId, &revoluteDef)

		baseID1 = bodyID2
		baseID2 = bodyID1
		baseAnchor1 = b2.V2(-2.5, 0.0)
		baseAnchor2 = b2.V2(2.5, 0.0)
		y = y.Add(b2.QOne())
	}

	bodyDef.Position = b2.Vec2{Y: y}
	bodyDef.Rotation = b2.RotIdentity()
	platformID := b2.CreateBody(s.WorldId, &bodyDef)
	box := b2.MakeBox(b2.F(3), b2.F(0.2))
	b2.CreatePolygonShape(platformID, &shapeDef, &box)

	revoluteDef := b2.DefaultRevoluteJointDef()
	revoluteDef.Base.BodyIdA = platformID
	revoluteDef.Base.BodyIdB = baseID1
	revoluteDef.Base.LocalFrameA.P = b2.V2(-2.5, -0.4)
	revoluteDef.Base.LocalFrameB.P = baseAnchor1
	revoluteDef.Base.CollideConnected = true
	revoluteDef.Base.ConstraintDampingRatio = constraintDampingRatio
	revoluteDef.Base.ConstraintHertz = constraintHertz
	b2.CreateRevoluteJoint(s.WorldId, &revoluteDef)

	wheelDef := b2.DefaultWheelJointDef()
	wheelDef.Base.BodyIdA = platformID
	wheelDef.Base.BodyIdB = baseID2
	wheelDef.Base.LocalFrameA.Q = b2.MakeRotFromUnitVector(b2.Vec2{X: b2.QOne()})
	wheelDef.Base.LocalFrameA.P = b2.V2(2.5, -0.4)
	wheelDef.Base.LocalFrameB.P = baseAnchor2
	wheelDef.EnableSpring = false
	wheelDef.Base.CollideConnected = true
	wheelDef.Base.ConstraintDampingRatio = constraintDampingRatio
	wheelDef.Base.ConstraintHertz = constraintHertz
	b2.CreateWheelJoint(s.WorldId, &wheelDef)

	s.enableMotor = false
	s.motorSpeed = 0.25
	s.motorForce = 2000

	distanceDef := b2.DefaultDistanceJointDef()
	distanceDef.Base.BodyIdA = groundID
	distanceDef.Base.BodyIdB = linkID1
	distanceDef.Base.LocalFrameA.P = b2.V2(-2.5, 0.2)
	distanceDef.Base.LocalFrameB.P = b2.V2(0.5, 0.0)
	distanceDef.EnableSpring = true
	distanceDef.MinLength = b2.F(0.2)
	distanceDef.MaxLength = b2.F(5.5)
	distanceDef.EnableLimit = true
	distanceDef.EnableMotor = s.enableMotor
	distanceDef.MotorSpeed = FromFloat64(s.motorSpeed)
	distanceDef.MaxMotorForce = FromFloat64(s.motorForce)
	s.liftJointID = b2.CreateDistanceJoint(s.WorldId, &distanceDef)

	var decoration car
	decoration.spawn(s.WorldId, b2.Vec2{X: b2.QZero(), Y: y.Add(b2.F(2))},
		b2.QOne(), b2.F(3), b2.F(0.7), b2.QZero(), nil)
	return s
}

// UpdateGui exposes the lift motor controls.
func (s *ScissorLift) UpdateGui() {
	const height = 140
	gui := s.Context.Gui
	gui.Begin("Scissor Lift", 10, s.Context.Camera.Height-height-50, 240, height)
	if gui.Checkbox("Motor", &s.enableMotor) {
		s.liftJointID.EnableMotor(s.enableMotor)
		s.liftJointID.WakeBodies()
	}
	if gui.SliderFloat("Max Force", &s.motorForce, 0, 3000) {
		s.liftJointID.SetMaxMotorForce(FromFloat64(s.motorForce))
		s.liftJointID.WakeBodies()
	}
	if gui.SliderFloat("Speed", &s.motorSpeed, -0.3, 0.3) {
		s.liftJointID.SetTargetMotorSpeed(FromFloat64(s.motorSpeed))
		s.liftJointID.WakeBodies()
	}
	gui.End()
}

// GearLift builds a geared lift with a hanging door.
type GearLift struct {
	Base

	driverId    b2.JointId
	motorTorque float64
	motorSpeed  float64
	enableMotor bool
}

// NewGearLift builds the gear-lift scene.
func NewGearLift(ctx *SampleContext) Sample {
	g := &GearLift{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 6}
		ctx.Camera.Zoom = 7
		ctx.Settings.DrawJoints = false
	}

	groundDef := b2.DefaultBodyDef()
	groundId := b2.CreateBody(g.WorldId, &groundDef)
	material := b2.DefaultSurfaceMaterial()
	material.CustomColor = uint32(b2.ColorDarkSeaGreen)
	chainDef := b2.DefaultChainDef()
	path := "m 63.500002,201.08333 103.187498,0 1e-5,-37.04166 h -2.64584 l 0,34.39583 h -42.33333 v -2.64583 l " +
		"-2.64584,-1e-5 v -2.64583 h -2.64583 v -2.64584 h -2.64584 v -2.64583 H 111.125 v -2.64583 h -2.64583 v " +
		"-2.64583 h -2.64583 v -2.64584 l -2.64584,1e-5 v -2.64583 l -2.64583,-1e-5 V 174.625 h -2.645834 v -2.64584 l " +
		"-2.645833,1e-5 v -2.64584 H 92.60417 v -2.64583 h -2.645834 v -2.64583 l -26.458334,0 0,37.04166"
	offset := b2.V2(-120, -200)
	chainDef.Points = parsePath(path, offset, 64, b2.F(0.2))
	chainDef.IsLoop = true
	chainDef.Materials = []b2.SurfaceMaterial{material}
	b2.CreateChain(groundId, &chainDef)

	gearRadius := b2.F(1)
	toothHalfWidth := b2.F(0.09)
	toothHalfHeight := b2.F(0.06)
	toothRadius := b2.F(0.03)
	linkHalfLength := b2.F(0.07)
	linkRadius := b2.F(0.05)
	doorHalfHeight := b2.F(1.5)
	gearPosition1 := b2.V2(-4.25, 9.75)
	gearPosition2 := gearPosition1.Add(b2.V2(2, 1))
	linkAttachPosition := gearPosition2.Add(b2.Vec2{X: gearRadius.Add(toothHalfWidth.Mul(b2.F(2))).Add(toothRadius)})
	doorPosition := linkAttachPosition.Sub(b2.Vec2{Y: linkHalfLength.Mul(b2.F(2)).Mul(b2.F(40)).Add(doorHalfHeight)})

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.Position = gearPosition1
	driverBodyId := b2.CreateBody(g.WorldId, &bodyDef)
	shapeDef := b2.DefaultShapeDef()
	shapeDef.Material.Friction = b2.F(0.1)
	shapeDef.Material.CustomColor = uint32(b2.ColorSaddleBrown)
	circle := b2.Circle{Center: b2.Vec2{}, Radius: gearRadius}
	b2.CreateCircleShape(driverBodyId, &shapeDef, &circle)
	dq := b2.MakeRot(b2.QFromRatio(1, 16))
	center := b2.Vec2{X: gearRadius.Add(toothHalfHeight)}
	rotation := b2.RotIdentity()
	for range 16 {
		tooth := b2.MakeOffsetRoundedBox(toothHalfWidth, toothHalfHeight, center, rotation, toothRadius)
		shapeDef.Material.CustomColor = uint32(b2.ColorGray)
		b2.CreatePolygonShape(driverBodyId, &shapeDef, &tooth)
		rotation = b2.MulRot(dq, rotation)
		center = b2.RotateVector(rotation, b2.Vec2{X: gearRadius.Add(toothHalfHeight)})
	}
	revoluteDef := b2.DefaultRevoluteJointDef()
	g.motorTorque = 80
	g.motorSpeed = 0
	g.enableMotor = true
	revoluteDef.Base.BodyIdA = groundId
	revoluteDef.Base.BodyIdB = driverBodyId
	revoluteDef.Base.LocalFrameA.P = groundId.GetLocalPoint(gearPosition1)
	revoluteDef.Base.LocalFrameB.P = b2.Vec2{}
	revoluteDef.EnableMotor = g.enableMotor
	revoluteDef.MaxMotorTorque = FromFloat64(g.motorTorque)
	revoluteDef.MotorSpeed = radiansToTurns(g.motorSpeed)
	g.driverId = b2.CreateRevoluteJoint(g.WorldId, &revoluteDef)

	bodyDef.Position = gearPosition2
	followerId := b2.CreateBody(g.WorldId, &bodyDef)
	shapeDef.Material.Friction = b2.F(0.1)
	shapeDef.Material.CustomColor = uint32(b2.ColorSaddleBrown)
	b2.CreateCircleShape(followerId, &shapeDef, &circle)
	center = b2.Vec2{X: gearRadius.Add(toothHalfWidth)}
	rotation = b2.RotIdentity()
	for range 16 {
		tooth := b2.MakeOffsetRoundedBox(toothHalfWidth, toothHalfHeight, center, rotation, toothRadius)
		shapeDef.Material.CustomColor = uint32(b2.ColorGray)
		b2.CreatePolygonShape(followerId, &shapeDef, &tooth)
		rotation = b2.MulRot(dq, rotation)
		center = b2.RotateVector(rotation, b2.Vec2{X: gearRadius.Add(toothHalfWidth)})
	}
	revoluteDef = b2.DefaultRevoluteJointDef()
	revoluteDef.Base.BodyIdA = groundId
	revoluteDef.Base.BodyIdB = followerId
	revoluteDef.Base.LocalFrameA.P = groundId.GetLocalPoint(gearPosition2)
	revoluteDef.Base.LocalFrameB.P = b2.Vec2{}
	revoluteDef.EnableMotor = true
	revoluteDef.MaxMotorTorque = b2.F(0.5)
	revoluteDef.Base.LocalFrameA.Q = b2.MakeRot(b2.QFromRatio(1, 8))
	revoluteDef.LowerAngle = b2.QFromRatio(-15, 100)
	revoluteDef.UpperAngle = b2.QFromRatio(4, 10)
	revoluteDef.EnableLimit = true
	b2.CreateRevoluteJoint(g.WorldId, &revoluteDef)

	capsule := b2.Capsule{Center1: b2.Vec2{Y: linkHalfLength.Neg()}, Center2: b2.Vec2{Y: linkHalfLength}, Radius: linkRadius}
	shapeDef = b2.DefaultShapeDef()
	shapeDef.Density = b2.F(2)
	shapeDef.Material.CustomColor = uint32(b2.ColorLightSteelBlue)
	jointDef := b2.DefaultRevoluteJointDef()
	jointDef.MaxMotorTorque = b2.F(0.05)
	jointDef.EnableMotor = true
	bodyDef = b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	position := linkAttachPosition.Add(b2.Vec2{Y: linkHalfLength.Neg()})
	prevBodyId := followerId
	var lastLinkId b2.BodyId
	for range 40 {
		bodyDef.Position = position
		bodyId := b2.CreateBody(g.WorldId, &bodyDef)
		b2.CreateCapsuleShape(bodyId, &shapeDef, &capsule)
		pivot := b2.Vec2{X: position.X, Y: position.Y.Add(linkHalfLength)}
		jointDef.Base.BodyIdA = prevBodyId
		jointDef.Base.BodyIdB = bodyId
		jointDef.Base.LocalFrameA.P = prevBodyId.GetLocalPoint(pivot)
		jointDef.Base.LocalFrameB.P = bodyId.GetLocalPoint(pivot)
		jointDef.Base.DrawScale = b2.F(0.2)
		b2.CreateRevoluteJoint(g.WorldId, &jointDef)
		position.Y = position.Y.Sub(linkHalfLength.Mul(b2.F(2)))
		prevBodyId = bodyId
		lastLinkId = bodyId
	}

	bodyDef = b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.Position = doorPosition
	doorBodyId := b2.CreateBody(g.WorldId, &bodyDef)
	box := b2.MakeBox(b2.F(0.15), doorHalfHeight)
	shapeDef = b2.DefaultShapeDef()
	shapeDef.Material.Friction = b2.F(0.1)
	shapeDef.Material.CustomColor = uint32(b2.ColorDarkCyan)
	b2.CreatePolygonShape(doorBodyId, &shapeDef, &box)
	pivot := doorPosition.Add(b2.Vec2{Y: doorHalfHeight})
	revoluteDef = b2.DefaultRevoluteJointDef()
	revoluteDef.Base.BodyIdA = lastLinkId
	revoluteDef.Base.BodyIdB = doorBodyId
	revoluteDef.Base.LocalFrameA.P = lastLinkId.GetLocalPoint(pivot)
	revoluteDef.Base.LocalFrameB.P = b2.Vec2{Y: doorHalfHeight}
	revoluteDef.EnableMotor = true
	revoluteDef.MaxMotorTorque = b2.F(0.05)
	b2.CreateRevoluteJoint(g.WorldId, &revoluteDef)
	prismaticDef := b2.DefaultPrismaticJointDef()
	prismaticDef.Base.BodyIdA = groundId
	prismaticDef.Base.BodyIdB = doorBodyId
	prismaticDef.Base.LocalFrameA.P = groundId.GetLocalPoint(doorPosition)
	prismaticDef.Base.LocalFrameB.P = b2.Vec2{}
	prismaticDef.Base.LocalFrameA.Q = b2.MakeRotFromUnitVector(b2.Vec2{Y: b2.QOne()})
	prismaticDef.Base.LocalFrameB.Q = prismaticDef.Base.LocalFrameA.Q
	prismaticDef.MaxMotorForce = b2.F(0.2)
	prismaticDef.EnableMotor = true
	prismaticDef.Base.CollideConnected = true
	b2.CreatePrismaticJoint(g.WorldId, &prismaticDef)

	bodyDef = b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	shapeDef = b2.DefaultShapeDef()
	shapeDef.Material.RollingResistance = b2.F(0.3)
	colors := [5]uint32{uint32(b2.ColorGray), uint32(b2.ColorGainsboro), uint32(b2.ColorLightGray), uint32(b2.ColorLightSlateGray), uint32(b2.ColorDarkGray)}
	y := b2.F(4.25)
	for range 20 {
		x := b2.F(-3.15)
		for range 10 {
			bodyDef.Position = b2.Vec2{X: x, Y: y}
			bodyId := b2.CreateBody(g.WorldId, &bodyDef)
			poly := shared.RandomPolygon(b2.F(0.1))
			poly.Radius = shared.RandomFloatRange(b2.F(0.01), b2.F(0.02))
			shapeDef.Material.CustomColor = colors[shared.RandomIntRange(0, 4)]
			b2.CreatePolygonShape(bodyId, &shapeDef, &poly)
			x = x.Add(b2.F(0.2))
		}
		y = y.Add(b2.F(0.2))
	}

	return g
}

// UpdateGui exposes the gear-lift motor controls.
func (g *GearLift) UpdateGui() {
	const height = 120
	gui := g.Context.Gui
	gui.Begin("Gear Lift", 10, g.Context.Camera.Height-height-25, 240, height)
	if gui.Checkbox("Motor", &g.enableMotor) {
		g.driverId.EnableMotor(g.enableMotor)
		g.driverId.WakeBodies()
	}
	if gui.SliderFloat("Max Torque", &g.motorTorque, 0, 100) {
		g.driverId.SetMaxMotorTorque(FromFloat64(g.motorTorque))
		g.driverId.WakeBodies()
	}
	if gui.SliderFloat("Speed", &g.motorSpeed, -0.3, 0.3) {
		g.driverId.SetTargetMotorSpeed(radiansToTurns(g.motorSpeed))
		g.driverId.WakeBodies()
	}
	gui.End()
}

// Step advances the gear-lift simulation.
func (g *GearLift) Step() {
	if g.keyDown(KeyA) {
		g.motorSpeed = max(-0.3, g.motorSpeed-0.01)
		g.driverId.SetTargetMotorSpeed(radiansToTurns(g.motorSpeed))
		g.driverId.WakeBodies()
	}
	if g.keyDown(KeyD) {
		g.motorSpeed = min(0.3, g.motorSpeed+0.01)
		g.driverId.SetTargetMotorSpeed(radiansToTurns(g.motorSpeed))
		g.driverId.WakeBodies()
	}
	g.Base.Step()
}

// TopDownFriction drives bodies in a top-down world with motor joints that
// act as friction against the ground.
type TopDownFriction struct {
	Base
}

// NewTopDownFriction builds the top-down friction scene.
func NewTopDownFriction(ctx *SampleContext) Sample {
	s := &TopDownFriction{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 10}
		ctx.Camera.Zoom = 12
	}

	groundDef := b2.DefaultBodyDef()
	groundId := b2.CreateBody(s.WorldId, &groundDef)
	groundShapeDef := b2.DefaultShapeDef()
	for _, segment := range []b2.Segment{
		{Point1: b2.V2(-10, 0), Point2: b2.V2(10, 0)},
		{Point1: b2.V2(-10, 0), Point2: b2.V2(-10, 20)},
		{Point1: b2.V2(10, 0), Point2: b2.V2(10, 20)},
		{Point1: b2.V2(-10, 20), Point2: b2.V2(10, 20)},
	} {
		b2.CreateSegmentShape(groundId, &groundShapeDef, &segment)
	}

	jointDef := b2.DefaultMotorJointDef()
	jointDef.Base.BodyIdA = groundId
	jointDef.Base.CollideConnected = true
	jointDef.MaxVelocityForce = b2.F(10)
	jointDef.MaxVelocityTorque = b2.F(10)

	capsule := b2.Capsule{Center1: b2.V2(-0.25, 0), Center2: b2.V2(0.25, 0), Radius: b2.F(0.25)}
	circle := b2.Circle{Center: b2.Vec2{}, Radius: b2.F(0.35)}
	square := b2.MakeSquare(b2.F(0.35))

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.GravityScale = b2.QZero()
	shapeDef := b2.DefaultShapeDef()
	shapeDef.Material.Restitution = b2.F(0.8)

	const n = 10
	x, y := b2.F(-5), b2.F(15)
	for i := range n {
		for j := range n {
			bodyDef.Position = b2.Vec2{X: x, Y: y}
			bodyId := b2.CreateBody(s.WorldId, &bodyDef)

			switch (n*i + j) % 4 {
			case 0:
				b2.CreateCapsuleShape(bodyId, &shapeDef, &capsule)
			case 1:
				b2.CreateCircleShape(bodyId, &shapeDef, &circle)
			case 2:
				b2.CreatePolygonShape(bodyId, &shapeDef, &square)
			default:
				poly := shared.RandomPolygon(b2.F(0.75))
				poly.Radius = b2.F(0.1)
				b2.CreatePolygonShape(bodyId, &shapeDef, &poly)
			}

			jointDef.Base.BodyIdB = bodyId
			b2.CreateMotorJoint(s.WorldId, &jointDef)

			x = x.Add(b2.QOne())
		}

		x = b2.F(-5)
		y = y.Sub(b2.QOne())
	}

	return s
}

// UpdateGui provides the explosion button.
func (s *TopDownFriction) UpdateGui() {
	const height = 60
	gui := s.Context.Gui
	gui.Begin("Top Down Friction", 10, s.Context.Camera.Height-height-25, 200, height)
	if gui.Button("Explode") {
		def := b2.DefaultExplosionDef()
		def.Position = b2.V2(0, 10)
		def.Radius = b2.F(10)
		def.Falloff = b2.F(5)
		def.ImpulsePerLength = b2.F(10)
		s.WorldId.Explode(&def)

		s.Context.Draw.DrawCircleFcn(def.Position, b2.F(10), b2.ColorWhite)
	}
	gui.End()
}

// TheoJansen is Theo Jansen's walking machine, inspired by a contribution
// from roman_m. Dimensions scooped from APE (http://www.cove.org/ape/index.htm).
type TheoJansen struct {
	Base

	offset       b2.Vec2
	chassisId    b2.BodyId
	wheelId      b2.BodyId
	motorJointId b2.JointId
	motorSpeed   float64
	enableMotor  bool
}

// NewTheoJansen builds the walking machine.
func NewTheoJansen(ctx *SampleContext) Sample {
	s := &TheoJansen{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 6}
		ctx.Camera.Zoom = 25 * 0.5
	}

	s.offset = b2.V2(0, 8)
	s.motorSpeed = 2
	s.enableMotor = true
	pivot := b2.V2(0.0, 0.8)

	// Ground
	{
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(s.WorldId, &bodyDef)
		shapeDef := b2.DefaultShapeDef()

		for _, segment := range []b2.Segment{
			{Point1: b2.V2(-50, 0), Point2: b2.V2(50, 0)},
			{Point1: b2.V2(-50, 0), Point2: b2.V2(-50, 10)},
			{Point1: b2.V2(50, 0), Point2: b2.V2(50, 10)},
		} {
			b2.CreateSegmentShape(groundId, &shapeDef, &segment)
		}
	}

	// Balls
	for i := range 40 {
		circle := b2.Circle{Center: b2.Vec2{}, Radius: b2.F(0.25)}

		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.Vec2{X: b2.F(-40 + 2*i), Y: b2.F(0.5)}
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		shapeDef.Density = b2.QOne()
		b2.CreateCircleShape(bodyId, &shapeDef, &circle)
	}

	// Chassis
	{
		box := b2.MakeBox(b2.F(2.5), b2.QOne())

		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = pivot.Add(s.offset)
		s.chassisId = b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		shapeDef.Density = b2.QOne()
		shapeDef.Filter.GroupIndex = -1
		b2.CreatePolygonShape(s.chassisId, &shapeDef, &box)
	}

	// Wheel
	{
		circle := b2.Circle{Center: b2.Vec2{}, Radius: b2.F(1.6)}

		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = pivot.Add(s.offset)
		s.wheelId = b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		shapeDef.Density = b2.QOne()
		shapeDef.Filter.GroupIndex = -1
		b2.CreateCircleShape(s.wheelId, &shapeDef, &circle)
	}

	{
		jointDef := b2.DefaultRevoluteJointDef()
		jointDef.Base.BodyIdA = s.wheelId
		jointDef.Base.BodyIdB = s.chassisId

		motorPivot := pivot.Add(s.offset)
		jointDef.Base.LocalFrameA.P = jointDef.Base.BodyIdA.GetLocalPoint(motorPivot)
		jointDef.Base.LocalFrameB.P = jointDef.Base.BodyIdB.GetLocalPoint(motorPivot)

		jointDef.EnableMotor = s.enableMotor
		jointDef.MotorSpeed = radiansToTurns(s.motorSpeed)
		jointDef.MaxMotorTorque = b2.F(400)
		s.motorJointId = b2.CreateRevoluteJoint(s.WorldId, &jointDef)
	}

	wheelAnchor := pivot.Add(b2.V2(0, -0.8))

	s.createLeg(-1, wheelAnchor)
	s.createLeg(1, wheelAnchor)

	// Stagger three leg pairs around the wheel so some feet always touch the ground.
	// 120 degrees is a third of a turn.
	s.wheelId.SetTransform(s.wheelId.GetPosition(), b2.MakeRot(b2.QFromRatio(1, 3)))
	s.createLeg(-1, wheelAnchor)
	s.createLeg(1, wheelAnchor)

	s.wheelId.SetTransform(s.wheelId.GetPosition(), b2.MakeRot(b2.QFromRatio(-1, 3)))
	s.createLeg(-1, wheelAnchor)
	s.createLeg(1, wheelAnchor)

	return s
}

func (s *TheoJansen) createLeg(sign float64, wheelAnchor b2.Vec2) {
	p1 := b2.V2(5.4*sign, -6.1)
	p2 := b2.V2(7.2*sign, -1.2)
	p3 := b2.V2(4.3*sign, -1.9)
	p4 := b2.V2(3.1*sign, 0.8)
	p5 := b2.V2(6.0*sign, 1.5)
	p6 := b2.V2(2.5*sign, 3.7)

	// The hull computation handles the mirrored winding for sign < 0.
	hull1 := b2.ComputeHull([]b2.Vec2{p1, p2, p3})
	hull2 := b2.ComputeHull([]b2.Vec2{{}, p5.Sub(p4), p6.Sub(p4)})
	poly1 := b2.MakePolygon(&hull1, b2.QZero())
	poly2 := b2.MakePolygon(&hull2, b2.QZero())

	bodyDef1 := b2.DefaultBodyDef()
	bodyDef1.Type = b2.DynamicBody
	bodyDef1.Position = s.offset
	bodyDef1.AngularDamping = b2.F(10)
	body1 := b2.CreateBody(s.WorldId, &bodyDef1)

	bodyDef2 := b2.DefaultBodyDef()
	bodyDef2.Type = b2.DynamicBody
	bodyDef2.Position = p4.Add(s.offset)
	bodyDef2.AngularDamping = b2.F(10)
	body2 := b2.CreateBody(s.WorldId, &bodyDef2)

	// Negative group index keeps the legs from colliding with the chassis and wheel.
	shapeDef := b2.DefaultShapeDef()
	shapeDef.Density = b2.QOne()
	shapeDef.Filter.GroupIndex = -1
	b2.CreatePolygonShape(body1, &shapeDef, &poly1)
	b2.CreatePolygonShape(body2, &shapeDef, &poly2)

	{
		// Soft distance constraints reduce jitter and act like a suspension system.
		jointDef := b2.DefaultDistanceJointDef()
		jointDef.EnableSpring = true
		jointDef.Hertz = b2.F(10)
		jointDef.DampingRatio = b2.F(0.5)

		jointDef.Base.BodyIdA = body1
		jointDef.Base.BodyIdB = body2

		anchorA := p2.Add(s.offset)
		anchorB := p5.Add(s.offset)
		jointDef.Base.LocalFrameA.P = body1.GetLocalPoint(anchorA)
		jointDef.Base.LocalFrameB.P = body2.GetLocalPoint(anchorB)
		jointDef.Length = anchorA.Sub(anchorB).Len()
		b2.CreateDistanceJoint(s.WorldId, &jointDef)

		anchorA = p3.Add(s.offset)
		anchorB = p4.Add(s.offset)
		jointDef.Base.LocalFrameA.P = body1.GetLocalPoint(anchorA)
		jointDef.Base.LocalFrameB.P = body2.GetLocalPoint(anchorB)
		jointDef.Length = anchorA.Sub(anchorB).Len()
		b2.CreateDistanceJoint(s.WorldId, &jointDef)

		jointDef.Base.BodyIdB = s.wheelId
		anchorB = wheelAnchor.Add(s.offset)
		jointDef.Base.LocalFrameB.P = s.wheelId.GetLocalPoint(anchorB)
		jointDef.Length = anchorA.Sub(anchorB).Len()
		b2.CreateDistanceJoint(s.WorldId, &jointDef)

		jointDef.Base.BodyIdA = body2
		anchorA = p6.Add(s.offset)
		jointDef.Base.LocalFrameA.P = body2.GetLocalPoint(anchorA)
		jointDef.Length = anchorA.Sub(anchorB).Len()
		b2.CreateDistanceJoint(s.WorldId, &jointDef)
	}

	{
		jointDef := b2.DefaultRevoluteJointDef()
		jointDef.Base.BodyIdA = body2
		jointDef.Base.BodyIdB = s.chassisId

		legPivot := p4.Add(s.offset)
		jointDef.Base.LocalFrameA.P = body2.GetLocalPoint(legPivot)
		jointDef.Base.LocalFrameB.P = s.chassisId.GetLocalPoint(legPivot)
		b2.CreateRevoluteJoint(s.WorldId, &jointDef)
	}
}

// UpdateGui exposes the motor controls.
func (s *TheoJansen) UpdateGui() {
	const height = 80
	gui := s.Context.Gui
	gui.Begin("Theo Jansen", 10, s.Context.Camera.Height-height-25, 200, height)
	if gui.Checkbox("Motor", &s.enableMotor) {
		s.motorJointId.EnableMotor(s.enableMotor)
	}
	if gui.SliderFloat("Speed", &s.motorSpeed, -5, 5) {
		s.motorJointId.SetTargetMotorSpeed(radiansToTurns(s.motorSpeed))
	}
	gui.End()
}

func (s *TheoJansen) Step() {
	s.DrawTextLine("Keys: left = a, brake = s, right = d, toggle motor = f")
	s.Base.Step()
}

func (s *TheoJansen) Keyboard(key Key) {
	switch key {
	case KeyA:
		s.motorJointId.SetTargetMotorSpeed(radiansToTurns(-s.motorSpeed))
	case KeyS:
		s.motorJointId.SetTargetMotorSpeed(b2.QZero())
	case KeyD:
		s.motorJointId.SetTargetMotorSpeed(radiansToTurns(s.motorSpeed))
	case KeyF:
		s.enableMotor = !s.motorJointId.IsMotorEnabled()
		s.motorJointId.EnableMotor(s.enableMotor)
	default:
		s.Base.Keyboard(key)
	}
}

const deskLampSpringCount = 2

// DeskLamp is the Anglepoise Original 1227 desk lamp.
// https://www.anglepoise.com/usa/product-category/original-1227/
type DeskLamp struct {
	Base

	springJointIds [deskLampSpringCount]b2.JointId
	postJointId    b2.JointId
	elbowJointId   b2.JointId
	headJointId    b2.JointId
	topLinkId      b2.BodyId
	springHertz    [deskLampSpringCount]float64
	springDamping  float64
	lowerFriction  float64
	elbowFriction  float64
	headAngle      float64
}

// NewDeskLamp builds the desk lamp.
func NewDeskLamp(ctx *SampleContext) Sample {
	s := &DeskLamp{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0.2, Y: 1.42}
		ctx.Camera.Zoom = 1.6
	}

	// The parallelogram is narrow, so it wants the extra resolution.
	ctx.Settings.SubStepCount = 8

	{
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(s.WorldId, &bodyDef)
		shapeDef := b2.DefaultShapeDef()
		segment := b2.Segment{Point1: b2.V2(-20, 0), Point2: b2.V2(20, 0)}
		b2.CreateSegmentShape(groundId, &shapeDef, &segment)
	}

	// Dimensions taken off an Original 1227 in meters, then scaled up. At life size the silver
	// link is two centimeters, which is far too close to the linear slop.
	scale := b2.F(5)
	sc := func(v string) b2.Q { return b2.QMustParse(v).Mul(scale) }

	baseHalfWidth := sc("0.070")
	baseHalfHeight := sc("0.018")
	postHalfWidth := sc("0.008")
	pivotHeight := sc("0.185")
	springAnchorHeight := sc("0.095")
	topLinkLength := sc("0.296")
	silverFoot := sc("0.024")
	topLeverLength := sc("0.019")
	lowerLeverLength := sc("0.028")
	upperArmLength := sc("0.324")
	tubeRadius := sc("0.0036")
	shadeLength := sc("0.161")
	shadeNeckRadius := sc("0.020")
	shadeMouthRadius := sc("0.061")

	// The silver link measured in the top link frame. The same span separates the two pins on the
	// upper arm, which is what closes the parallelogram.
	silverSpan := b2.Vec2{X: sc("0.0212"), Y: sc("0.0109")}
	silverLength := silverSpan.Len()

	topLinkAngle := -0.878
	upperArmAngle := 1.192
	shadeTilt := -0.41

	// The arm spring balances the elevation, the head spring balances the upper arm through the
	// parallelogram. That is what the real lamp uses two springs for.
	s.springHertz[0] = 5
	s.springHertz[1] = 5
	s.springDamping = 4
	s.lowerFriction = 100
	s.elbowFriction = 100
	s.headAngle = 0
	topRotation := rotFromRadians(topLinkAngle)
	upperRotation := rotFromRadians(upperArmAngle)

	topAxis := b2.RotGetYAxis(topRotation)
	silverWorld := b2.RotateVector(topRotation, silverSpan)

	postPivot := b2.Pos{X: b2.QZero(), Y: pivotHeight}
	upperPin := b2.OffsetPos(postPivot, topAxis.Mul(topLinkLength))
	silverBase := b2.OffsetPos(postPivot, topAxis.Mul(silverFoot))
	lowerFoot := b2.OffsetPos(silverBase, silverWorld)
	lowerPin := b2.OffsetPos(upperPin, silverWorld)

	// The lower link is parallel to the top link and shorter by the silver link foot.
	lowerLinkLength := topLinkLength.Sub(silverFoot)
	silverRotation := b2.MakeRotFromUnitVector(b2.RightPerp(silverWorld.Normalize()))

	headPivot := b2.OffsetPos(upperPin, b2.RotGetYAxis(upperRotation).Mul(upperArmLength))

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody

	shapeDef := b2.DefaultShapeDef()

	var baseId b2.BodyId
	{
		// The base is static so the linkage is all that moves.
		bodyDef.Type = b2.StaticBody
		bodyDef.Position = b2.Vec2{}
		bodyDef.Rotation = b2.RotIdentity()
		baseId = b2.CreateBody(s.WorldId, &bodyDef)

		plinth := b2.MakeOffsetBox(baseHalfWidth, baseHalfHeight, b2.Vec2{Y: baseHalfHeight}, b2.RotIdentity())
		postHalfHeight := b2.QHalf().Mul(pivotHeight.Sub(baseHalfHeight.Mul(b2.F(2))))
		post := b2.MakeOffsetBox(postHalfWidth, postHalfHeight, b2.Vec2{Y: pivotHeight.Sub(postHalfHeight)}, b2.RotIdentity())

		b2.CreatePolygonShape(baseId, &shapeDef, &plinth)
		b2.CreatePolygonShape(baseId, &shapeDef, &post)
	}

	bodyDef.Type = b2.DynamicBody
	shapeDef.Density = b2.F(20)

	// Both long links run past their feet into the levers the springs pull on.
	{
		bodyDef.Position = postPivot
		bodyDef.Rotation = topRotation
		s.topLinkId = b2.CreateBody(s.WorldId, &bodyDef)

		capsule := b2.Capsule{Center1: b2.Vec2{Y: topLeverLength.Neg()}, Center2: b2.Vec2{Y: topLinkLength}, Radius: tubeRadius}
		b2.CreateCapsuleShape(s.topLinkId, &shapeDef, &capsule)
	}

	var lowerLinkId b2.BodyId
	{
		bodyDef.Position = lowerFoot
		bodyDef.Rotation = topRotation
		lowerLinkId = b2.CreateBody(s.WorldId, &bodyDef)

		capsule := b2.Capsule{Center1: b2.Vec2{Y: lowerLeverLength.Neg()}, Center2: b2.Vec2{Y: lowerLinkLength}, Radius: tubeRadius}
		b2.CreateCapsuleShape(lowerLinkId, &shapeDef, &capsule)
	}

	var silverId b2.BodyId
	{
		bodyDef.Position = silverBase
		bodyDef.Rotation = silverRotation
		silverId = b2.CreateBody(s.WorldId, &bodyDef)

		capsule := b2.Capsule{Center1: b2.Vec2{}, Center2: b2.Vec2{Y: silverLength}, Radius: b2.F(1.4).Mul(tubeRadius)}
		shapeDef.Material.CustomColor = uint32(b2.ColorSilver)
		b2.CreateCapsuleShape(silverId, &shapeDef, &capsule)
		shapeDef.Material.CustomColor = 0
	}

	var upperArmId b2.BodyId
	{
		bodyDef.Position = upperPin
		bodyDef.Rotation = upperRotation
		upperArmId = b2.CreateBody(s.WorldId, &bodyDef)

		capsule := b2.Capsule{Center1: b2.Vec2{}, Center2: b2.Vec2{Y: upperArmLength}, Radius: b2.F(0.8).Mul(tubeRadius)}
		b2.CreateCapsuleShape(upperArmId, &shapeDef, &capsule)

		// The coupler span reaches back to the lower link pin.
		bracket := b2.Capsule{Center1: b2.Vec2{}, Center2: upperArmId.GetLocalPoint(lowerPin), Radius: tubeRadius}
		b2.CreateCapsuleShape(upperArmId, &shapeDef, &bracket)
	}

	pinDef := b2.DefaultRevoluteJointDef()
	pinDef.Base.DrawScale = b2.F(0.03).Mul(scale)
	pinDef.Base.ConstraintHertz = b2.F(240)
	pinDef.Base.ConstraintDampingRatio = b2.F(2)

	// Only the top link reaches the post. The springs carry it, so it needs little friction.
	pinDef.Base.BodyIdA = baseId
	pinDef.Base.BodyIdB = s.topLinkId
	pinDef.Base.LocalFrameA.P = baseId.GetLocalPoint(postPivot)
	pinDef.Base.LocalFrameB.P = b2.Vec2{}
	pinDef.EnableMotor = true
	pinDef.MaxMotorTorque = FromFloat64(s.lowerFriction)
	s.postJointId = b2.CreateRevoluteJoint(s.WorldId, &pinDef)

	pinDef.EnableMotor = false

	// The silver link hangs off the top link and carries the foot of the lower link.
	pinDef.Base.BodyIdA = s.topLinkId
	pinDef.Base.BodyIdB = silverId
	pinDef.Base.LocalFrameA.P = b2.Vec2{Y: silverFoot}
	pinDef.Base.LocalFrameB.P = b2.Vec2{}
	b2.CreateRevoluteJoint(s.WorldId, &pinDef)

	pinDef.Base.BodyIdA = silverId
	pinDef.Base.BodyIdB = lowerLinkId
	pinDef.Base.LocalFrameA.P = b2.Vec2{Y: silverLength}
	pinDef.Base.LocalFrameB.P = b2.Vec2{}
	b2.CreateRevoluteJoint(s.WorldId, &pinDef)

	// Both long links reach the upper arm, at points a silver link apart.
	pinDef.Base.BodyIdA = lowerLinkId
	pinDef.Base.BodyIdB = upperArmId
	pinDef.Base.LocalFrameA.P = b2.Vec2{Y: lowerLinkLength}
	pinDef.Base.LocalFrameB.P = upperArmId.GetLocalPoint(lowerPin)
	b2.CreateRevoluteJoint(s.WorldId, &pinDef)

	// No spring reaches the elbow, so it is a friction pivot like the real one.
	pinDef.Base.BodyIdA = s.topLinkId
	pinDef.Base.BodyIdB = upperArmId
	pinDef.Base.LocalFrameA.P = b2.Vec2{Y: topLinkLength}
	pinDef.Base.LocalFrameB.P = b2.Vec2{}
	pinDef.EnableMotor = true
	pinDef.MaxMotorTorque = FromFloat64(s.elbowFriction)
	s.elbowJointId = b2.CreateRevoluteJoint(s.WorldId, &pinDef)

	springDef := b2.DefaultDistanceJointDef()
	springDef.Base.DrawScale = b2.F(0.03).Mul(scale)
	springDef.EnableSpring = true
	springDef.DampingRatio = FromFloat64(s.springDamping)

	// Carwardine's trick. With almost no free length the spring torque about the pivot is
	// k times cross(lever, anchor), which tracks the gravity torque as the arm swings, so the
	// arm balances over its whole range instead of at one angle.
	springDef.Length = b2.F(0.01)

	springBase := b2.Pos{X: b2.QZero(), Y: springAnchorHeight}
	leverIds := [deskLampSpringCount]b2.BodyId{s.topLinkId, lowerLinkId}
	leverLengths := [deskLampSpringCount]b2.Q{topLeverLength, lowerLeverLength}

	for k := range deskLampSpringCount {
		springDef.Base.BodyIdA = baseId
		springDef.Base.BodyIdB = leverIds[k]
		springDef.Base.LocalFrameA.P = baseId.GetLocalPoint(springBase)
		springDef.Base.LocalFrameB.P = b2.Vec2{Y: leverLengths[k].Neg()}
		springDef.Hertz = FromFloat64(s.springHertz[k])
		s.springJointIds[k] = b2.CreateDistanceJoint(s.WorldId, &springDef)
	}

	{
		bodyDef.Position = headPivot
		bodyDef.Rotation = rotFromRadians(shadeTilt)
		headId := b2.CreateBody(s.WorldId, &bodyDef)

		// The pivot is on the shade center line, where the arm capsule ends. The shade opens along
		// the local negative y.
		neck := b2.F(0.45).Mul(shadeLength)
		mouth := neck.Sub(shadeLength)
		points := []b2.Vec2{
			{X: shadeNeckRadius.Neg(), Y: neck},
			{X: shadeNeckRadius, Y: neck},
			{X: shadeMouthRadius, Y: mouth},
			{X: shadeMouthRadius.Neg(), Y: mouth},
		}
		hull := b2.ComputeHull(points)
		shade := b2.MakePolygon(&hull, b2.F(0.01).Mul(scale))

		// The shade is a thin shell.
		shapeDef.Density = b2.QOne()
		b2.CreatePolygonShape(headId, &shapeDef, &shade)

		// Offset the reference frame so the head slider reads zero at the pose above.
		pinDef.Base.BodyIdA = upperArmId
		pinDef.Base.BodyIdB = headId
		pinDef.Base.LocalFrameA.P = b2.Vec2{Y: upperArmLength}
		pinDef.Base.LocalFrameA.Q = b2.InvMulRot(upperRotation, rotFromRadians(shadeTilt))
		pinDef.Base.LocalFrameB.P = b2.Vec2{}
		pinDef.EnableMotor = false
		pinDef.MaxMotorTorque = b2.F(10)
		pinDef.EnableSpring = true
		pinDef.Hertz = b2.F(8)
		pinDef.DampingRatio = b2.F(2)
		pinDef.TargetAngle = radiansToTurns(s.headAngle)
		pinDef.EnableLimit = true
		pinDef.LowerAngle = radiansToTurns(-1.2)
		pinDef.UpperAngle = radiansToTurns(1.2)
		s.headJointId = b2.CreateRevoluteJoint(s.WorldId, &pinDef)
	}

	return s
}

// UpdateGui exposes the spring and friction controls.
func (s *DeskLamp) UpdateGui() {
	const height = 200
	gui := s.Context.Gui
	gui.Begin("Desk Lamp", 10, s.Context.Camera.Height-height-25, 240, height)

	if gui.SliderFloat("Arm Spring", &s.springHertz[0], 0, 20) {
		s.springJointIds[0].SetSpringHertz(FromFloat64(s.springHertz[0]))
		s.springJointIds[0].WakeBodies()
	}

	if gui.SliderFloat("Head Spring", &s.springHertz[1], 0, 20) {
		s.springJointIds[1].SetSpringHertz(FromFloat64(s.springHertz[1]))
		s.springJointIds[1].WakeBodies()
	}

	if gui.SliderFloat("Spring Damping", &s.springDamping, 0, 5) {
		for k := range deskLampSpringCount {
			s.springJointIds[k].SetSpringDampingRatio(FromFloat64(s.springDamping))
			s.springJointIds[k].WakeBodies()
		}
	}

	if gui.SliderFloat("Post Friction", &s.lowerFriction, 0, 20) {
		s.postJointId.SetMaxMotorTorque(FromFloat64(s.lowerFriction))
		s.postJointId.WakeBodies()
	}

	if gui.SliderFloat("Elbow Friction", &s.elbowFriction, 0, 60) {
		s.elbowJointId.SetMaxMotorTorque(FromFloat64(s.elbowFriction))
		s.elbowJointId.WakeBodies()
	}

	if gui.SliderFloat("Head", &s.headAngle, -1.2, 1.2) {
		s.headJointId.SetTargetAngle(radiansToTurns(s.headAngle))
		s.headJointId.WakeBodies()
	}

	gui.End()
}

func (s *DeskLamp) Step() {
	s.Base.Step()

	postTorque := ToFloat64(s.postJointId.GetCurrentMotorTorque())
	elbowTorque := ToFloat64(s.elbowJointId.GetCurrentMotorTorque())
	s.DrawTextLine("friction carrying: post %.2f, elbow %.2f N m", postTorque, elbowTorque)

	spring1 := ToFloat64(s.springJointIds[0].GetSpringForce())
	spring2 := ToFloat64(s.springJointIds[1].GetSpringForce())
	s.DrawTextLine("spring forces: %.2f, %.2f N", spring1, spring2)
}
