// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/sample_joints.cpp of Box2D v3.1.1. Debug-only sizes
// use the release values.

package samples

import (
	"fmt"
	"math"

	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/fixed"
)

func init() {
	RegisterSample("Joints", "Distance Joint", NewDistanceJoint)
	RegisterSample("Joints", "Motor Joint", NewMotorJoint)
	RegisterSample("Joints", "Filter Joint", NewFilterJoint)
	RegisterSample("Joints", "Revolute", NewRevoluteJoint)
	RegisterSample("Joints", "Prismatic", NewPrismaticJoint)
	RegisterSample("Joints", "Wheel", NewWheelJoint)
	RegisterSample("Joints", "Bridge", NewBridge)
	RegisterSample("Joints", "Ball & Chain", NewBallAndChain)
	RegisterSample("Joints", "Cantilever", NewCantilever)
	RegisterSample("Joints", "Fixed Rotation", NewFixedRotation)
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
}

// DistanceJoint demonstrates a configurable chain of bodies connected by distance joints.
type DistanceJoint struct {
	Base

	groundId  dbox2d.BodyId
	bodyIds   [10]dbox2d.BodyId
	jointIds  [10]dbox2d.JointId
	count     int
	hertz     float64
	damping   float64
	length    float64
	minLength float64
	maxLength float64
	spring    bool
	limit     bool
}

// NewDistanceJoint builds the distance-joint chain scene.
func NewDistanceJoint(ctx *SampleContext) Sample {
	s := &DistanceJoint{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0.0, Y: 12.0}
		ctx.Camera.Zoom = 25.0 * 0.35
	}

	bodyDef := dbox2d.DefaultBodyDef()
	s.groundId = dbox2d.CreateBody(s.WorldId, &bodyDef)

	s.count = 0
	s.hertz = 2.0
	s.damping = 0.5
	s.length = 1.0
	s.minLength = s.length
	s.maxLength = s.length
	s.spring = false
	s.limit = false

	for i := range s.bodyIds {
		s.bodyIds[i] = dbox2d.BodyId{}
		s.jointIds[i] = dbox2d.JointId{}
	}

	s.createScene(1)
	return s
}

func (s *DistanceJoint) createScene(newCount int) {
	// Joints must be destroyed before their attached bodies.
	for i := 0; i < s.count; i++ {
		dbox2d.DestroyJoint(s.jointIds[i])
		s.jointIds[i] = dbox2d.JointId{}
	}

	for i := 0; i < s.count; i++ {
		dbox2d.DestroyBody(s.bodyIds[i])
		s.bodyIds[i] = dbox2d.BodyId{}
	}

	s.count = newCount

	circle := dbox2d.Circle{Radius: fixed.Q32FromRatio(1, 4)}
	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Density = fixed.Q32FromInt(20)
	yOffset := fixed.Q32FromInt(20)

	jointDef := dbox2d.DefaultDistanceJointDef()
	jointDef.Hertz = FromFloat64(s.hertz)
	jointDef.DampingRatio = FromFloat64(s.damping)
	jointDef.Length = FromFloat64(s.length)
	jointDef.MinLength = FromFloat64(s.minLength)
	jointDef.MaxLength = FromFloat64(s.maxLength)
	jointDef.EnableSpring = s.spring
	jointDef.EnableLimit = s.limit

	prevBodyId := s.groundId
	for i := 0; i < s.count; i++ {
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.AngularDamping = fixed.Q32MustParse("0.1")
		bodyDef.Position = dbox2d.Vec2{
			X: FromFloat64(s.length * float64(i+1)),
			Y: yOffset,
		}
		s.bodyIds[i] = dbox2d.CreateBody(s.WorldId, &bodyDef)
		dbox2d.CreateCircleShape(s.bodyIds[i], &shapeDef, &circle)

		pivotA := dbox2d.Vec2{X: FromFloat64(s.length * float64(i)), Y: yOffset}
		pivotB := dbox2d.Vec2{X: FromFloat64(s.length * float64(i+1)), Y: yOffset}
		jointDef.BodyIdA = prevBodyId
		jointDef.BodyIdB = s.bodyIds[i]
		jointDef.LocalAnchorA = prevBodyId.GetLocalPoint(pivotA)
		jointDef.LocalAnchorB = s.bodyIds[i].GetLocalPoint(pivotB)
		s.jointIds[i] = dbox2d.CreateDistanceJoint(s.WorldId, &jointDef)

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
			s.jointIds[i].SetLength(FromFloat64(s.length))
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
				s.jointIds[i].SetLengthRange(FromFloat64(s.minLength), FromFloat64(s.maxLength))
				s.jointIds[i].WakeBodies()
			}
		}

		if gui.SliderFloat("Max Length", &s.maxLength, 0.1, 4.0) {
			for i := 0; i < s.count; i++ {
				s.jointIds[i].SetLengthRange(FromFloat64(s.minLength), FromFloat64(s.maxLength))
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

// MotorJoint demonstrates a motorized body following a changing offset.
type MotorJoint struct {
	Base

	bodyId           dbox2d.BodyId
	jointId          dbox2d.JointId
	time             float64
	maxForce         float64
	maxTorque        float64
	correctionFactor float64
	goEnabled        bool
}

// NewMotorJoint builds the motor-joint scene.
func NewMotorJoint(ctx *SampleContext) Sample {
	s := &MotorJoint{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0.0, Y: 7.0}
		ctx.Camera.Zoom = 25.0 * 0.4
	}

	groundBodyDef := dbox2d.DefaultBodyDef()
	groundId := dbox2d.CreateBody(s.WorldId, &groundBodyDef)
	groundShapeDef := dbox2d.DefaultShapeDef()
	groundSegment := dbox2d.Segment{
		Point1: dbox2d.Vec2{X: FromFloat64(-20.0), Y: FromFloat64(0.0)},
		Point2: dbox2d.Vec2{X: FromFloat64(20.0), Y: FromFloat64(0.0)},
	}
	dbox2d.CreateSegmentShape(groundId, &groundShapeDef, &groundSegment)

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	bodyDef.Position = dbox2d.Vec2{X: FromFloat64(0.0), Y: FromFloat64(8.0)}
	s.bodyId = dbox2d.CreateBody(s.WorldId, &bodyDef)

	box := dbox2d.MakeBox(FromFloat64(2.0), FromFloat64(0.5))
	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Density = FromFloat64(1.0)
	dbox2d.CreatePolygonShape(s.bodyId, &shapeDef, &box)

	s.maxForce = 500.0
	s.maxTorque = 500.0
	s.correctionFactor = 0.3
	jointDef := dbox2d.DefaultMotorJointDef()
	jointDef.BodyIdA = groundId
	jointDef.BodyIdB = s.bodyId
	jointDef.MaxForce = FromFloat64(s.maxForce)
	jointDef.MaxTorque = FromFloat64(s.maxTorque)
	jointDef.CorrectionFactor = FromFloat64(s.correctionFactor)
	s.jointId = dbox2d.CreateMotorJoint(s.WorldId, &jointDef)

	s.goEnabled = true
	s.time = 0.0
	return s
}

func (s *MotorJoint) UpdateGui() {
	gui := s.Context.Gui
	height := 180
	gui.Begin("Motor Joint", 10, s.Context.Camera.Height-height-50, 240, height)

	gui.Checkbox("Go", &s.goEnabled)
	if gui.SliderFloat("Max Force", &s.maxForce, 0.0, 10000.0) {
		s.jointId.SetMaxForce(FromFloat64(s.maxForce))
	}
	if gui.SliderFloat("Max Torque", &s.maxTorque, 0.0, 10000.0) {
		s.jointId.SetMaxTorque(FromFloat64(s.maxTorque))
	}
	if gui.SliderFloat("Correction", &s.correctionFactor, 0.0, 1.0) {
		s.jointId.SetCorrectionFactor(FromFloat64(s.correctionFactor))
	}
	if gui.Button("Apply Impulse") {
		s.bodyId.ApplyLinearImpulseToCenter(dbox2d.Vec2{X: FromFloat64(100.0), Y: FromFloat64(0.0)}, true)
	}

	gui.End()
}

func (s *MotorJoint) Step() {
	if s.goEnabled && s.Context.Settings.Hertz > 0 {
		s.time += 1.0 / s.Context.Settings.Hertz
	}

	linearOffset := dbox2d.Vec2{
		X: FromFloat64(6.0 * math.Sin(2.0*s.time)),
		Y: FromFloat64(8.0 + 4.0*math.Sin(1.0*s.time)),
	}
	angularOffsetRadians := 2.0 * s.time
	angularOffsetTurns := FromFloat64(angularOffsetRadians / (2.0 * math.Pi))

	s.jointId.SetLinearOffset(linearOffset)
	s.jointId.SetAngularOffset(angularOffsetTurns)
	s.Context.Draw.DrawTransform(dbox2d.Transform{P: linearOffset, Q: dbox2d.MakeRot(angularOffsetTurns)})

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

	groundBodyDef := dbox2d.DefaultBodyDef()
	groundId := dbox2d.CreateBody(s.WorldId, &groundBodyDef)
	groundShapeDef := dbox2d.DefaultShapeDef()
	groundSegment := dbox2d.Segment{
		Point1: dbox2d.Vec2{X: FromFloat64(-20.0), Y: FromFloat64(0.0)},
		Point2: dbox2d.Vec2{X: FromFloat64(20.0), Y: FromFloat64(0.0)},
	}
	dbox2d.CreateSegmentShape(groundId, &groundShapeDef, &groundSegment)

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	bodyDef.Position = dbox2d.Vec2{X: FromFloat64(-4.0), Y: FromFloat64(2.0)}
	body1 := dbox2d.CreateBody(s.WorldId, &bodyDef)
	box := dbox2d.MakeSquare(FromFloat64(2.0))
	shapeDef := dbox2d.DefaultShapeDef()
	dbox2d.CreatePolygonShape(body1, &shapeDef, &box)

	bodyDef.Position = dbox2d.Vec2{X: FromFloat64(4.0), Y: FromFloat64(2.0)}
	body2 := dbox2d.CreateBody(s.WorldId, &bodyDef)
	dbox2d.CreatePolygonShape(body2, &shapeDef, &box)

	jointDef := dbox2d.DefaultFilterJointDef()
	jointDef.BodyIdA = body1
	jointDef.BodyIdB = body2
	dbox2d.CreateFilterJoint(s.WorldId, &jointDef)

	return s
}

// Revolute demonstrates limits, motors, and springs on revolute joints.
type RevoluteJoint struct {
	Base

	ball          dbox2d.BodyId
	jointId1      dbox2d.JointId
	jointId2      dbox2d.JointId
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

	var groundID dbox2d.BodyId
	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Position = dbox2d.Vec2{Y: fixed.Q32FromInt(-1)}
		groundID = dbox2d.CreateBody(s.WorldId, &bodyDef)

		box := dbox2d.MakeBox(fixed.Q32FromInt(40), fixed.Q32FromInt(1))
		shapeDef := dbox2d.DefaultShapeDef()
		dbox2d.CreatePolygonShape(groundID, &shapeDef, &box)
	}

	s.enableSpring = false
	s.enableLimit = true
	s.enableMotor = false
	s.hertz = 2
	s.dampingRatio = 0.5
	s.targetDegrees = 45
	s.motorSpeed = 1
	s.motorTorque = 1000

	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = dbox2d.Vec2{X: fixed.Q32FromInt(-10), Y: fixed.Q32FromInt(20)}
		bodyID := dbox2d.CreateBody(s.WorldId, &bodyDef)

		shapeDef := dbox2d.DefaultShapeDef()
		shapeDef.Density = fixed.Q32One()
		capsule := dbox2d.Capsule{
			Center1: dbox2d.Vec2{Y: fixed.Q32FromInt(-1)},
			Center2: dbox2d.Vec2{Y: fixed.Q32FromInt(6)},
			Radius:  fixed.Q32Half(),
		}
		dbox2d.CreateCapsuleShape(bodyID, &shapeDef, &capsule)

		pivot := dbox2d.Vec2{X: fixed.Q32FromInt(-10), Y: fixed.Q32MustParse("20.5")}
		jointDef := dbox2d.DefaultRevoluteJointDef()
		jointDef.BodyIdA = groundID
		jointDef.BodyIdB = bodyID
		jointDef.LocalAnchorA = groundID.GetLocalPoint(pivot)
		jointDef.LocalAnchorB = bodyID.GetLocalPoint(pivot)
		// Convert the GUI's degrees to the public API's turns.
		jointDef.TargetAngle = FromFloat64(s.targetDegrees).Div(fixed.Q32FromInt(360))
		jointDef.EnableSpring = s.enableSpring
		jointDef.Hertz = FromFloat64(s.hertz)
		jointDef.DampingRatio = FromFloat64(s.dampingRatio)
		jointDef.MotorSpeed = radiansToTurns(s.motorSpeed)
		jointDef.MaxMotorTorque = FromFloat64(s.motorTorque)
		jointDef.EnableMotor = s.enableMotor
		// 0.5*PI radians is 1/4 turn.
		jointDef.ReferenceAngle = fixed.Q32FromRatio(1, 4)
		jointDef.LowerAngle = fixed.Q32FromRatio(-1, 4)
		jointDef.UpperAngle = fixed.Q32FromRatio(3, 8)
		jointDef.EnableLimit = s.enableLimit

		s.jointId1 = dbox2d.CreateRevoluteJoint(s.WorldId, &jointDef)
	}

	{
		circle := dbox2d.Circle{Radius: fixed.Q32FromInt(2)}
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = dbox2d.Vec2{X: fixed.Q32FromInt(5), Y: fixed.Q32FromInt(30)}
		s.ball = dbox2d.CreateBody(s.WorldId, &bodyDef)

		shapeDef := dbox2d.DefaultShapeDef()
		shapeDef.Density = fixed.Q32One()
		dbox2d.CreateCircleShape(s.ball, &shapeDef, &circle)
	}

	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Position = dbox2d.Vec2{X: fixed.Q32FromInt(20), Y: fixed.Q32FromInt(10)}
		bodyDef.Type = dbox2d.DynamicBody
		bodyID := dbox2d.CreateBody(s.WorldId, &bodyDef)

		box := dbox2d.MakeOffsetBox(
			fixed.Q32FromInt(10), fixed.Q32MustParse("0.5"),
			dbox2d.Vec2{X: fixed.Q32FromInt(-10)}, dbox2d.RotIdentity(),
		)
		shapeDef := dbox2d.DefaultShapeDef()
		shapeDef.Density = fixed.Q32One()
		dbox2d.CreatePolygonShape(bodyID, &shapeDef, &box)

		pivot := dbox2d.Vec2{X: fixed.Q32FromInt(19), Y: fixed.Q32FromInt(10)}
		jointDef := dbox2d.DefaultRevoluteJointDef()
		jointDef.BodyIdA = groundID
		jointDef.BodyIdB = bodyID
		jointDef.LocalAnchorA = groundID.GetLocalPoint(pivot)
		jointDef.LocalAnchorB = bodyID.GetLocalPoint(pivot)
		jointDef.LowerAngle = fixed.Q32FromRatio(-1, 8)
		jointDef.UpperAngle = fixed.Q32FromRatio(1, 20)
		jointDef.EnableLimit = true
		jointDef.EnableMotor = true
		jointDef.MotorSpeed = fixed.Q32Zero()
		jointDef.MaxMotorTorque = FromFloat64(s.motorTorque)

		s.jointId2 = dbox2d.CreateRevoluteJoint(s.WorldId, &jointDef)
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
			s.jointId1.SetMotorSpeed(radiansToTurns(s.motorSpeed))
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
			targetAngle := FromFloat64(s.targetDegrees).Div(fixed.Q32FromInt(360))
			s.jointId1.SetTargetAngle(targetAngle)
			s.jointId1.WakeBodies()
		}
	}

	gui.End()
}

// Step advances the scene and reports the joint angles and motor torques.
func (s *RevoluteJoint) Step() {
	s.Base.Step()

	angle1 := ToFloat64(s.jointId1.GetAngle()) * 360
	s.DrawTextLine("Angle (Deg) 1 = %2.1f", angle1)

	torque1 := ToFloat64(s.jointId1.GetMotorTorque())
	s.DrawTextLine("Motor Torque 1 = %4.1f", torque1)

	torque2 := ToFloat64(s.jointId2.GetMotorTorque())
	s.DrawTextLine("Motor Torque 2 = %4.1f", torque2)
}

// PrismaticJoint demonstrates limits, motors, and springs on prismatic joints.
type PrismaticJoint struct {
	Base

	jointId      dbox2d.JointId
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

	var groundID dbox2d.BodyId
	{
		bodyDef := dbox2d.DefaultBodyDef()
		groundID = dbox2d.CreateBody(s.WorldId, &bodyDef)
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
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Position = dbox2d.Vec2{Y: fixed.Q32FromInt(10)}
		bodyDef.Type = dbox2d.DynamicBody
		bodyID := dbox2d.CreateBody(s.WorldId, &bodyDef)

		shapeDef := dbox2d.DefaultShapeDef()
		box := dbox2d.MakeBox(fixed.Q32MustParse("0.5"), fixed.Q32FromInt(2))
		dbox2d.CreatePolygonShape(bodyID, &shapeDef, &box)

		pivot := dbox2d.Vec2{Y: fixed.Q32FromInt(9)}
		axis := (dbox2d.Vec2{X: fixed.Q32One(), Y: fixed.Q32One()}).Normalize()
		jointDef := dbox2d.DefaultPrismaticJointDef()
		jointDef.BodyIdA = groundID
		jointDef.BodyIdB = bodyID
		jointDef.LocalAxisA = groundID.GetLocalVector(axis)
		jointDef.LocalAnchorA = groundID.GetLocalPoint(pivot)
		jointDef.LocalAnchorB = bodyID.GetLocalPoint(pivot)
		jointDef.MotorSpeed = FromFloat64(s.motorSpeed)
		jointDef.MaxMotorForce = FromFloat64(s.motorForce)
		jointDef.EnableMotor = s.enableMotor
		jointDef.LowerTranslation = fixed.Q32FromInt(-10)
		jointDef.UpperTranslation = fixed.Q32FromInt(10)
		jointDef.EnableLimit = s.enableLimit
		jointDef.EnableSpring = s.enableSpring
		jointDef.Hertz = FromFloat64(s.hertz)
		jointDef.DampingRatio = FromFloat64(s.dampingRatio)

		s.jointId = dbox2d.CreatePrismaticJoint(s.WorldId, &jointDef)
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
			s.jointId.SetMotorSpeed(FromFloat64(s.motorSpeed))
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

		if gui.SliderFloat("Translation", &s.translation, -5, 5) {
			s.jointId.SetTargetTranslation(FromFloat64(s.translation))
			s.jointId.WakeBodies()
		}
	}

	gui.End()
}

// Step advances the scene and reports the motor force and translation state.
func (s *PrismaticJoint) Step() {
	s.Base.Step()

	force := ToFloat64(s.jointId.GetMotorForce())
	s.DrawTextLine("Motor Force = %4.1f", force)

	translation := ToFloat64(s.jointId.GetTranslation())
	s.DrawTextLine("Translation = %4.1f", translation)

	speed := ToFloat64(s.jointId.GetSpeed())
	s.DrawTextLine("Speed = %4.8f", speed)
}

// WheelJoint demonstrates a wheel joint with configurable suspension, limits, and motor.
type WheelJoint struct {
	Base

	jointID      dbox2d.JointId
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

	groundDef := dbox2d.DefaultBodyDef()
	groundID := dbox2d.CreateBody(s.WorldId, &groundDef)

	s.enableSpring = true
	s.enableLimit = true
	s.enableMotor = true
	s.motorSpeed = 2.0
	s.motorTorque = 5.0
	s.hertz = 1.0
	s.dampingRatio = 0.7

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Position = dbox2d.Vec2{Y: fixed.Q32MustParse("10.25")}
	bodyDef.Type = dbox2d.DynamicBody
	bodyID := dbox2d.CreateBody(s.WorldId, &bodyDef)

	shapeDef := dbox2d.DefaultShapeDef()
	capsule := dbox2d.Capsule{
		Center1: dbox2d.Vec2{Y: fixed.Q32Half().Neg()},
		Center2: dbox2d.Vec2{Y: fixed.Q32Half()},
		Radius:  fixed.Q32Half(),
	}
	dbox2d.CreateCapsuleShape(bodyID, &shapeDef, &capsule)

	pivot := dbox2d.Vec2{Y: fixed.Q32FromInt(10)}
	axis := dbox2d.Vec2{X: fixed.Q32One(), Y: fixed.Q32One()}.Normalize()
	jointDef := dbox2d.DefaultWheelJointDef()
	jointDef.BodyIdA = groundID
	jointDef.BodyIdB = bodyID
	jointDef.LocalAxisA = groundID.GetLocalVector(axis)
	jointDef.LocalAnchorA = groundID.GetLocalPoint(pivot)
	jointDef.LocalAnchorB = bodyID.GetLocalPoint(pivot)
	jointDef.MotorSpeed = radiansToTurns(s.motorSpeed)
	jointDef.MaxMotorTorque = FromFloat64(s.motorTorque)
	jointDef.EnableMotor = s.enableMotor
	jointDef.LowerTranslation = fixed.Q32FromInt(-3)
	jointDef.UpperTranslation = fixed.Q32FromInt(3)
	jointDef.EnableLimit = s.enableLimit
	jointDef.Hertz = FromFloat64(s.hertz)
	jointDef.DampingRatio = FromFloat64(s.dampingRatio)

	s.jointID = dbox2d.CreateWheelJoint(s.WorldId, &jointDef)

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
				s.jointID.SetMotorSpeed(radiansToTurns(s.motorSpeed))
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

	torque := s.jointID.GetMotorTorque()
	s.DrawTextLine("Motor Torque = %4.1f", ToFloat64(torque))
}

const bridgeCount = 160

// Bridge demonstrates a suspension bridge made from revolute-jointed boxes.
type Bridge struct {
	Base
	bodyIDs                [bridgeCount]dbox2d.BodyId
	jointIDs               [bridgeCount + 1]dbox2d.JointId
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

	groundDef := dbox2d.DefaultBodyDef()
	groundID := dbox2d.CreateBody(s.WorldId, &groundDef)

	s.constraintHertz = 60.0
	s.constraintDampingRatio = 0.0
	s.springHertz = 2.0
	s.springDampingRatio = 0.7
	s.frictionTorque = 200.0

	box := dbox2d.MakeBox(fixed.Q32Half(), fixed.Q32FromRatio(1, 8))
	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Density = fixed.Q32FromInt(20)

	jointDef := dbox2d.DefaultRevoluteJointDef()
	jointDef.EnableMotor = true
	jointDef.MaxMotorTorque = FromFloat64(s.frictionTorque)
	jointDef.EnableSpring = true
	jointDef.Hertz = FromFloat64(s.springHertz)
	jointDef.DampingRatio = FromFloat64(s.springDampingRatio)

	jointIndex := 0
	xbase := fixed.Q32FromInt(-80)
	prevBodyID := groundID
	for i := range bridgeCount {
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = dbox2d.Vec2{
			X: xbase.Add(fixed.Q32Half()).Add(fixed.Q32FromInt(i)),
			Y: fixed.Q32FromInt(20),
		}
		bodyDef.LinearDamping = fixed.Q32MustParse("0.1")
		bodyDef.AngularDamping = fixed.Q32MustParse("0.1")

		s.bodyIDs[i] = dbox2d.CreateBody(s.WorldId, &bodyDef)
		dbox2d.CreatePolygonShape(s.bodyIDs[i], &shapeDef, &box)

		pivot := dbox2d.Vec2{X: xbase.Add(fixed.Q32FromInt(i)), Y: fixed.Q32FromInt(20)}
		jointDef.BodyIdA = prevBodyID
		jointDef.BodyIdB = s.bodyIDs[i]
		jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
		jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
		s.jointIDs[jointIndex] = dbox2d.CreateRevoluteJoint(s.WorldId, &jointDef)

		prevBodyID = s.bodyIDs[i]
		jointIndex++
	}

	pivot := dbox2d.Vec2{X: xbase.Add(fixed.Q32FromInt(bridgeCount)), Y: fixed.Q32FromInt(20)}
	jointDef.BodyIdA = prevBodyID
	jointDef.BodyIdB = groundID
	jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
	jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
	s.jointIDs[jointIndex] = dbox2d.CreateRevoluteJoint(s.WorldId, &jointDef)

	for i := range 2 {
		vertices := []dbox2d.Vec2{
			{X: fixed.Q32MustParse("-0.5")},
			{X: fixed.Q32MustParse("0.5")},
			{Y: fixed.Q32MustParse("1.5")},
		}
		hull := dbox2d.ComputeHull(vertices)
		triangle := dbox2d.MakePolygon(&hull, fixed.Q32Zero())

		shapeDef := dbox2d.DefaultShapeDef()
		shapeDef.Density = fixed.Q32FromInt(20)
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = dbox2d.Vec2{X: fixed.Q32FromInt(-8 + 8*i), Y: fixed.Q32FromInt(22)}
		bodyID := dbox2d.CreateBody(s.WorldId, &bodyDef)
		dbox2d.CreatePolygonShape(bodyID, &shapeDef, &triangle)
	}

	for i := range 3 {
		circle := dbox2d.Circle{Radius: fixed.Q32Half()}
		shapeDef := dbox2d.DefaultShapeDef()
		shapeDef.Density = fixed.Q32FromInt(20)
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = dbox2d.Vec2{X: fixed.Q32FromInt(-6 + 6*i), Y: fixed.Q32FromInt(25)}
		bodyID := dbox2d.CreateBody(s.WorldId, &bodyDef)
		dbox2d.CreateCircleShape(bodyID, &shapeDef, &circle)
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
	jointIDs       [ballAndChainCount + 1]dbox2d.JointId
	frictionTorque float64
}

// NewBallAndChain builds the ball and chain scene.
func NewBallAndChain(ctx *SampleContext) Sample {
	s := &BallAndChain{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0.0, Y: -8.0}
		ctx.Camera.Zoom = 27.5
	}

	groundDef := dbox2d.DefaultBodyDef()
	groundID := dbox2d.CreateBody(s.WorldId, &groundDef)

	s.frictionTorque = 100.0

	hx := fixed.Q32Half()
	capsule := dbox2d.Capsule{
		Center1: dbox2d.Vec2{X: hx.Neg()},
		Center2: dbox2d.Vec2{X: hx},
		Radius:  fixed.Q32MustParse("0.125"),
	}

	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Density = fixed.Q32FromInt(20)
	shapeDef.Filter.CategoryBits = 0x1
	shapeDef.Filter.MaskBits = 0x2
	jointDef := dbox2d.DefaultRevoluteJointDef()

	jointIndex := 0
	prevBodyID := groundID
	for i := range ballAndChainCount {
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = dbox2d.Vec2{
			X: fixed.Q32FromInt(1 + 2*i).Mul(hx),
			Y: fixed.Q32FromInt(ballAndChainCount).Mul(hx),
		}
		bodyID := dbox2d.CreateBody(s.WorldId, &bodyDef)
		dbox2d.CreateCapsuleShape(bodyID, &shapeDef, &capsule)

		pivot := dbox2d.Vec2{
			X: fixed.Q32FromInt(2 * i).Mul(hx),
			Y: fixed.Q32FromInt(ballAndChainCount).Mul(hx),
		}
		jointDef.BodyIdA = prevBodyID
		jointDef.BodyIdB = bodyID
		jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
		jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
		jointDef.EnableMotor = true
		jointDef.MaxMotorTorque = FromFloat64(s.frictionTorque)
		jointDef.EnableSpring = i > 0
		jointDef.Hertz = fixed.Q32FromInt(4)
		s.jointIDs[jointIndex] = dbox2d.CreateRevoluteJoint(s.WorldId, &jointDef)
		jointIndex++

		prevBodyID = bodyID
	}

	circle := dbox2d.Circle{Radius: fixed.Q32FromInt(4)}
	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	bodyDef.Position = dbox2d.Vec2{
		X: fixed.Q32FromInt(1 + 2*ballAndChainCount).Mul(hx).Add(circle.Radius).Sub(hx),
		Y: fixed.Q32FromInt(ballAndChainCount).Mul(hx),
	}
	bodyID := dbox2d.CreateBody(s.WorldId, &bodyDef)

	shapeDef.Filter.CategoryBits = 0x2
	shapeDef.Filter.MaskBits = 0x1
	dbox2d.CreateCircleShape(bodyID, &shapeDef, &circle)

	pivot := dbox2d.Vec2{
		X: fixed.Q32FromInt(2 * ballAndChainCount).Mul(hx),
		Y: fixed.Q32FromInt(ballAndChainCount).Mul(hx),
	}
	jointDef.BodyIdA = prevBodyID
	jointDef.BodyIdB = bodyID
	jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
	jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
	jointDef.EnableMotor = true
	jointDef.MaxMotorTorque = FromFloat64(s.frictionTorque)
	jointDef.EnableSpring = true
	jointDef.Hertz = fixed.Q32FromInt(4)
	s.jointIDs[jointIndex] = dbox2d.CreateRevoluteJoint(s.WorldId, &jointDef)

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

	tipID    dbox2d.BodyId
	bodyIDs  [cantileverCount]dbox2d.BodyId
	jointIDs [cantileverCount]dbox2d.JointId
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

	groundDef := dbox2d.DefaultBodyDef()
	groundID := dbox2d.CreateBody(s.WorldId, &groundDef)

	hx := fixed.Q32Half()
	capsule := dbox2d.Capsule{
		Center1: dbox2d.Vec2{X: hx.Neg()},
		Center2: dbox2d.Vec2{X: hx},
		Radius:  fixed.Q32MustParse("0.125"),
	}
	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Density = fixed.Q32FromInt(20)

	jointDef := dbox2d.DefaultWeldJointDef()
	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	bodyDef.IsAwake = false

	previousID := groundID
	for i := range cantileverCount {
		bodyDef.Position = dbox2d.Vec2{
			X: fixed.Q32FromInt(1 + 2*i).Mul(hx),
		}
		s.bodyIDs[i] = dbox2d.CreateBody(s.WorldId, &bodyDef)
		dbox2d.CreateCapsuleShape(s.bodyIDs[i], &shapeDef, &capsule)

		pivot := dbox2d.Vec2{X: fixed.Q32FromInt(2 * i).Mul(hx)}
		jointDef.BodyIdA = previousID
		jointDef.BodyIdB = s.bodyIDs[i]
		jointDef.LocalAnchorA = previousID.GetLocalPoint(pivot)
		jointDef.LocalAnchorB = s.bodyIDs[i].GetLocalPoint(pivot)
		jointDef.LinearHertz = FromFloat64(s.linearHertz)
		jointDef.LinearDampingRatio = FromFloat64(s.linearDampingRatio)
		jointDef.AngularHertz = FromFloat64(s.angularHertz)
		jointDef.AngularDampingRatio = FromFloat64(s.angularDampingRatio)
		jointDef.CollideConnected = s.collideConnected
		s.jointIDs[i] = dbox2d.CreateWeldJoint(s.WorldId, &jointDef)

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

const fixedRotationCount = 6

// FixedRotation demonstrates six joint types with a shared fixed-rotation setting.
type FixedRotation struct {
	Base

	groundID      dbox2d.BodyId
	bodyIDs       [fixedRotationCount]dbox2d.BodyId
	jointIDs      [fixedRotationCount]dbox2d.JointId
	fixedRotation bool
}

// NewFixedRotation builds the fixed-rotation joint comparison scene.
func NewFixedRotation(ctx *SampleContext) Sample {
	s := &FixedRotation{
		Base:          NewBase(ctx),
		fixedRotation: true,
	}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{Y: 8}
		ctx.Camera.Zoom = 25.0 * 0.7
	}

	groundDef := dbox2d.DefaultBodyDef()
	s.groundID = dbox2d.CreateBody(s.WorldId, &groundDef)
	s.CreateScene()

	return s
}

// CreateScene rebuilds all six bodies and joints using the current setting.
func (s *FixedRotation) CreateScene() {
	for i := range fixedRotationCount {
		if !s.jointIDs[i].IsNull() {
			dbox2d.DestroyJoint(s.jointIDs[i])
			s.jointIDs[i] = dbox2d.JointId{}
		}
		if !s.bodyIDs[i].IsNull() {
			dbox2d.DestroyBody(s.bodyIDs[i])
			s.bodyIDs[i] = dbox2d.BodyId{}
		}
	}

	position := dbox2d.Vec2{X: fixed.Q32FromRatio(-25, 2), Y: fixed.Q32FromInt(10)}
	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	bodyDef.FixedRotation = s.fixedRotation
	box := dbox2d.MakeBox(fixed.Q32One(), fixed.Q32One())

	index := 0

	// Distance joint.
	bodyDef.Position = position
	s.bodyIDs[index] = dbox2d.CreateBody(s.WorldId, &bodyDef)
	shapeDef := dbox2d.DefaultShapeDef()
	dbox2d.CreatePolygonShape(s.bodyIDs[index], &shapeDef, &box)
	length := fixed.Q32FromInt(2)
	pivot1 := dbox2d.Vec2{X: position.X, Y: position.Y.Add(fixed.Q32One()).Add(length)}
	pivot2 := dbox2d.Vec2{X: position.X, Y: position.Y.Add(fixed.Q32One())}
	distanceDef := dbox2d.DefaultDistanceJointDef()
	distanceDef.BodyIdA = s.groundID
	distanceDef.BodyIdB = s.bodyIDs[index]
	distanceDef.LocalAnchorA = s.groundID.GetLocalPoint(pivot1)
	distanceDef.LocalAnchorB = s.bodyIDs[index].GetLocalPoint(pivot2)
	distanceDef.Length = length
	s.jointIDs[index] = dbox2d.CreateDistanceJoint(s.WorldId, &distanceDef)

	position.X = position.X.Add(fixed.Q32FromInt(5))
	index++

	// Motor joint.
	bodyDef.Position = position
	s.bodyIDs[index] = dbox2d.CreateBody(s.WorldId, &bodyDef)
	shapeDef = dbox2d.DefaultShapeDef()
	dbox2d.CreatePolygonShape(s.bodyIDs[index], &shapeDef, &box)
	motorDef := dbox2d.DefaultMotorJointDef()
	motorDef.BodyIdA = s.groundID
	motorDef.BodyIdB = s.bodyIDs[index]
	motorDef.LinearOffset = position
	motorDef.MaxForce = fixed.Q32FromInt(200)
	motorDef.MaxTorque = fixed.Q32FromInt(20)
	s.jointIDs[index] = dbox2d.CreateMotorJoint(s.WorldId, &motorDef)

	position.X = position.X.Add(fixed.Q32FromInt(5))
	index++

	// Prismatic joint.
	bodyDef.Position = position
	s.bodyIDs[index] = dbox2d.CreateBody(s.WorldId, &bodyDef)
	shapeDef = dbox2d.DefaultShapeDef()
	dbox2d.CreatePolygonShape(s.bodyIDs[index], &shapeDef, &box)
	pivot := dbox2d.Vec2{X: position.X.Sub(fixed.Q32One()), Y: position.Y}
	prismaticDef := dbox2d.DefaultPrismaticJointDef()
	prismaticDef.BodyIdA = s.groundID
	prismaticDef.BodyIdB = s.bodyIDs[index]
	prismaticDef.LocalAnchorA = s.groundID.GetLocalPoint(pivot)
	prismaticDef.LocalAnchorB = s.bodyIDs[index].GetLocalPoint(pivot)
	prismaticDef.LocalAxisA = s.groundID.GetLocalVector(dbox2d.Vec2{X: fixed.Q32One()})
	s.jointIDs[index] = dbox2d.CreatePrismaticJoint(s.WorldId, &prismaticDef)

	position.X = position.X.Add(fixed.Q32FromInt(5))
	index++

	// Revolute joint.
	bodyDef.Position = position
	s.bodyIDs[index] = dbox2d.CreateBody(s.WorldId, &bodyDef)
	shapeDef = dbox2d.DefaultShapeDef()
	dbox2d.CreatePolygonShape(s.bodyIDs[index], &shapeDef, &box)
	pivot = dbox2d.Vec2{X: position.X.Sub(fixed.Q32One()), Y: position.Y}
	revoluteDef := dbox2d.DefaultRevoluteJointDef()
	revoluteDef.BodyIdA = s.groundID
	revoluteDef.BodyIdB = s.bodyIDs[index]
	revoluteDef.LocalAnchorA = s.groundID.GetLocalPoint(pivot)
	revoluteDef.LocalAnchorB = s.bodyIDs[index].GetLocalPoint(pivot)
	s.jointIDs[index] = dbox2d.CreateRevoluteJoint(s.WorldId, &revoluteDef)

	position.X = position.X.Add(fixed.Q32FromInt(5))
	index++

	// Weld joint.
	bodyDef.Position = position
	s.bodyIDs[index] = dbox2d.CreateBody(s.WorldId, &bodyDef)
	shapeDef = dbox2d.DefaultShapeDef()
	dbox2d.CreatePolygonShape(s.bodyIDs[index], &shapeDef, &box)
	pivot = dbox2d.Vec2{X: position.X.Sub(fixed.Q32One()), Y: position.Y}
	weldDef := dbox2d.DefaultWeldJointDef()
	weldDef.BodyIdA = s.groundID
	weldDef.BodyIdB = s.bodyIDs[index]
	weldDef.LocalAnchorA = s.groundID.GetLocalPoint(pivot)
	weldDef.LocalAnchorB = s.bodyIDs[index].GetLocalPoint(pivot)
	weldDef.AngularHertz = fixed.Q32One()
	weldDef.AngularDampingRatio = fixed.Q32Half()
	weldDef.LinearHertz = fixed.Q32One()
	weldDef.LinearDampingRatio = fixed.Q32Half()
	s.jointIDs[index] = dbox2d.CreateWeldJoint(s.WorldId, &weldDef)

	position.X = position.X.Add(fixed.Q32FromInt(5))
	index++

	// Wheel joint.
	bodyDef.Position = position
	s.bodyIDs[index] = dbox2d.CreateBody(s.WorldId, &bodyDef)
	shapeDef = dbox2d.DefaultShapeDef()
	dbox2d.CreatePolygonShape(s.bodyIDs[index], &shapeDef, &box)
	pivot = dbox2d.Vec2{X: position.X.Sub(fixed.Q32One()), Y: position.Y}
	wheelDef := dbox2d.DefaultWheelJointDef()
	wheelDef.BodyIdA = s.groundID
	wheelDef.BodyIdB = s.bodyIDs[index]
	wheelDef.LocalAnchorA = s.groundID.GetLocalPoint(pivot)
	wheelDef.LocalAnchorB = s.bodyIDs[index].GetLocalPoint(pivot)
	wheelDef.LocalAxisA = s.groundID.GetLocalVector(dbox2d.Vec2{X: fixed.Q32One()})
	wheelDef.Hertz = fixed.Q32One()
	wheelDef.DampingRatio = fixed.Q32FromRatio(7, 10)
	wheelDef.LowerTranslation = fixed.Q32FromInt(-1)
	wheelDef.UpperTranslation = fixed.Q32One()
	wheelDef.EnableLimit = true
	wheelDef.EnableMotor = true
	wheelDef.MaxMotorTorque = fixed.Q32FromInt(10)
	wheelDef.MotorSpeed = radiansToTurns(1)
	s.jointIDs[index] = dbox2d.CreateWheelJoint(s.WorldId, &wheelDef)
}

// UpdateGui toggles fixed rotation on all existing bodies without rebuilding.
func (s *FixedRotation) UpdateGui() {
	const height = 60
	if s.Context.Gui.Begin("Fixed Rotation", 10, s.Context.Camera.Height-height-50, 180, height) {
		if s.Context.Gui.Checkbox("Fixed Rotation", &s.fixedRotation) {
			for i := range fixedRotationCount {
				s.bodyIDs[i].SetFixedRotation(s.fixedRotation)
			}
		}
	}
	s.Context.Gui.End()
}

const breakableJointCount = 6

// BreakableJoint demonstrates breaking joints when their constraint force is exceeded.
type BreakableJoint struct {
	Base
	jointIDs   [breakableJointCount]dbox2d.JointId
	breakForce float64
}

// NewBreakableJoint builds the breakable-joint scene.
func NewBreakableJoint(ctx *SampleContext) Sample {
	s := &BreakableJoint{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 8}
		ctx.Camera.Zoom = 25 * 0.7
	}

	bodyDef := dbox2d.DefaultBodyDef()
	groundID := dbox2d.CreateBody(s.WorldId, &bodyDef)
	shapeDef := dbox2d.DefaultShapeDef()
	segment := dbox2d.Segment{Point1: dbox2d.Vec2{X: fixed.Q32FromInt(-40)}, Point2: dbox2d.Vec2{X: fixed.Q32FromInt(40)}}
	dbox2d.CreateSegmentShape(groundID, &shapeDef, &segment)

	box := dbox2d.MakeBox(fixed.Q32One(), fixed.Q32One())
	bodyDef.Type = dbox2d.DynamicBody
	bodyDef.EnableSleep = false
	position := dbox2d.Vec2{X: fixed.Q32MustParse("-12.5"), Y: fixed.Q32FromInt(10)}
	index := 0

	bodyDef.Position = position
	bodyID := dbox2d.CreateBody(s.WorldId, &bodyDef)
	dbox2d.CreatePolygonShape(bodyID, &shapeDef, &box)
	length := fixed.Q32FromInt(2)
	pivot1 := dbox2d.Vec2{X: position.X, Y: position.Y.Add(fixed.Q32One()).Add(length)}
	pivot2 := dbox2d.Vec2{X: position.X, Y: position.Y.Add(fixed.Q32One())}
	distanceDef := dbox2d.DefaultDistanceJointDef()
	distanceDef.BodyIdA, distanceDef.BodyIdB = groundID, bodyID
	distanceDef.LocalAnchorA = groundID.GetLocalPoint(pivot1)
	distanceDef.LocalAnchorB = bodyID.GetLocalPoint(pivot2)
	distanceDef.Length = length
	distanceDef.CollideConnected = true
	s.jointIDs[index] = dbox2d.CreateDistanceJoint(s.WorldId, &distanceDef)

	position.X = position.X.Add(fixed.Q32FromInt(5))
	index++
	bodyDef.Position = position
	bodyID = dbox2d.CreateBody(s.WorldId, &bodyDef)
	dbox2d.CreatePolygonShape(bodyID, &shapeDef, &box)
	motorDef := dbox2d.DefaultMotorJointDef()
	motorDef.BodyIdA, motorDef.BodyIdB = groundID, bodyID
	motorDef.LinearOffset = position
	motorDef.MaxForce = fixed.Q32FromInt(1000)
	motorDef.MaxTorque = fixed.Q32FromInt(20)
	motorDef.CollideConnected = true
	s.jointIDs[index] = dbox2d.CreateMotorJoint(s.WorldId, &motorDef)

	for kind := range 4 {
		position.X = position.X.Add(fixed.Q32FromInt(5))
		index++
		bodyDef.Position = position
		bodyID = dbox2d.CreateBody(s.WorldId, &bodyDef)
		dbox2d.CreatePolygonShape(bodyID, &shapeDef, &box)
		pivot := dbox2d.Vec2{X: position.X.Sub(fixed.Q32One()), Y: position.Y}
		switch kind {
		case 0:
			def := dbox2d.DefaultPrismaticJointDef()
			def.BodyIdA, def.BodyIdB = groundID, bodyID
			def.LocalAnchorA, def.LocalAnchorB = groundID.GetLocalPoint(pivot), bodyID.GetLocalPoint(pivot)
			def.LocalAxisA = groundID.GetLocalVector(dbox2d.Vec2{X: fixed.Q32One()})
			def.CollideConnected = true
			s.jointIDs[index] = dbox2d.CreatePrismaticJoint(s.WorldId, &def)
		case 1:
			def := dbox2d.DefaultRevoluteJointDef()
			def.BodyIdA, def.BodyIdB = groundID, bodyID
			def.LocalAnchorA, def.LocalAnchorB = groundID.GetLocalPoint(pivot), bodyID.GetLocalPoint(pivot)
			def.CollideConnected = true
			s.jointIDs[index] = dbox2d.CreateRevoluteJoint(s.WorldId, &def)
		case 2:
			def := dbox2d.DefaultWeldJointDef()
			def.BodyIdA, def.BodyIdB = groundID, bodyID
			def.LocalAnchorA, def.LocalAnchorB = groundID.GetLocalPoint(pivot), bodyID.GetLocalPoint(pivot)
			def.AngularHertz, def.AngularDampingRatio = fixed.Q32FromInt(2), fixed.Q32Half()
			def.LinearHertz, def.LinearDampingRatio = fixed.Q32FromInt(2), fixed.Q32Half()
			def.CollideConnected = true
			s.jointIDs[index] = dbox2d.CreateWeldJoint(s.WorldId, &def)
		case 3:
			def := dbox2d.DefaultWheelJointDef()
			def.BodyIdA, def.BodyIdB = groundID, bodyID
			def.LocalAnchorA, def.LocalAnchorB = groundID.GetLocalPoint(pivot), bodyID.GetLocalPoint(pivot)
			def.LocalAxisA = groundID.GetLocalVector(dbox2d.Vec2{X: fixed.Q32One()})
			def.Hertz, def.DampingRatio = fixed.Q32One(), fixed.Q32MustParse("0.7")
			def.LowerTranslation, def.UpperTranslation = fixed.Q32FromInt(-1), fixed.Q32One()
			def.EnableLimit, def.EnableMotor = true, true
			def.MaxMotorTorque, def.MotorSpeed = fixed.Q32FromInt(10), radiansToTurns(1)
			def.CollideConnected = true
			s.jointIDs[index] = dbox2d.CreateWheelJoint(s.WorldId, &def)
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
			dbox2d.DestroyJoint(s.jointIDs[i])
			s.jointIDs[i] = dbox2d.JointId{}
		} else {
			point := s.jointIDs[i].GetLocalAnchorA()
			s.Context.Draw.DrawString(point, fmt.Sprintf("(%.1f, %.1f)", ToFloat64(force.X), ToFloat64(force.Y)), dbox2d.ColorWhite)
		}
	}
	s.Base.Step()
}

const jointSeparationCount = 5

// JointSeparation displays unresolved constraint separation for several joint types.
type JointSeparation struct {
	Base
	bodyIDs           [jointSeparationCount]dbox2d.BodyId
	jointIDs          [jointSeparationCount]dbox2d.JointId
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

	bodyDef := dbox2d.DefaultBodyDef()
	groundID := dbox2d.CreateBody(s.WorldId, &bodyDef)
	shapeDef := dbox2d.DefaultShapeDef()
	segment := dbox2d.Segment{Point1: dbox2d.Vec2{X: fixed.Q32FromInt(-40)}, Point2: dbox2d.Vec2{X: fixed.Q32FromInt(40)}}
	dbox2d.CreateSegmentShape(groundID, &shapeDef, &segment)
	bodyDef.Type = dbox2d.DynamicBody
	bodyDef.EnableSleep = false
	box := dbox2d.MakeBox(fixed.Q32One(), fixed.Q32One())
	position := dbox2d.Vec2{X: fixed.Q32FromInt(-20), Y: fixed.Q32FromInt(10)}
	for i := range s.bodyIDs {
		bodyDef.Position = position
		s.bodyIDs[i] = dbox2d.CreateBody(s.WorldId, &bodyDef)
		dbox2d.CreatePolygonShape(s.bodyIDs[i], &shapeDef, &box)
		pivot := dbox2d.Vec2{X: position.X.Sub(fixed.Q32One()), Y: position.Y}
		switch i {
		case 0:
			def := dbox2d.DefaultDistanceJointDef()
			def.BodyIdA, def.BodyIdB = groundID, s.bodyIDs[i]
			def.LocalAnchorA = groundID.GetLocalPoint(dbox2d.Vec2{X: position.X, Y: position.Y.Add(fixed.Q32One()).Add(fixed.Q32FromInt(2))})
			def.LocalAnchorB = s.bodyIDs[i].GetLocalPoint(dbox2d.Vec2{X: position.X, Y: position.Y.Add(fixed.Q32One())})
			def.Length, def.CollideConnected = fixed.Q32FromInt(2), true
			s.jointIDs[i] = dbox2d.CreateDistanceJoint(s.WorldId, &def)
		case 1:
			def := dbox2d.DefaultPrismaticJointDef()
			def.BodyIdA, def.BodyIdB = groundID, s.bodyIDs[i]
			def.LocalAnchorA, def.LocalAnchorB = groundID.GetLocalPoint(pivot), s.bodyIDs[i].GetLocalPoint(pivot)
			def.LocalAxisA, def.CollideConnected = groundID.GetLocalVector(dbox2d.Vec2{X: fixed.Q32One()}), true
			s.jointIDs[i] = dbox2d.CreatePrismaticJoint(s.WorldId, &def)
		case 2:
			def := dbox2d.DefaultRevoluteJointDef()
			def.BodyIdA, def.BodyIdB = groundID, s.bodyIDs[i]
			def.LocalAnchorA, def.LocalAnchorB = groundID.GetLocalPoint(pivot), s.bodyIDs[i].GetLocalPoint(pivot)
			def.CollideConnected = true
			s.jointIDs[i] = dbox2d.CreateRevoluteJoint(s.WorldId, &def)
		case 3:
			def := dbox2d.DefaultWeldJointDef()
			def.BodyIdA, def.BodyIdB = groundID, s.bodyIDs[i]
			def.LocalAnchorA, def.LocalAnchorB = groundID.GetLocalPoint(pivot), s.bodyIDs[i].GetLocalPoint(pivot)
			def.CollideConnected = true
			s.jointIDs[i] = dbox2d.CreateWeldJoint(s.WorldId, &def)
		case 4:
			def := dbox2d.DefaultWheelJointDef()
			def.BodyIdA, def.BodyIdB = groundID, s.bodyIDs[i]
			def.LocalAnchorA, def.LocalAnchorB = groundID.GetLocalPoint(pivot), s.bodyIDs[i].GetLocalPoint(pivot)
			def.LocalAxisA = groundID.GetLocalVector(dbox2d.Vec2{X: fixed.Q32One()})
			def.Hertz, def.DampingRatio = fixed.Q32One(), fixed.Q32MustParse("0.7")
			def.LowerTranslation, def.UpperTranslation = fixed.Q32FromInt(-1), fixed.Q32One()
			def.EnableLimit, def.EnableMotor = true, true
			def.MaxMotorTorque, def.MotorSpeed = fixed.Q32FromInt(10), radiansToTurns(1)
			def.CollideConnected = true
			s.jointIDs[i] = dbox2d.CreateWheelJoint(s.WorldId, &def)
		}
		position.X = position.X.Add(fixed.Q32FromInt(10))
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
			point := s.bodyIDs[i].GetWorldPoint(dbox2d.Vec2{X: fixed.Q32One(), Y: fixed.Q32One()})
			s.bodyIDs[i].ApplyLinearImpulse(dbox2d.Vec2{X: impulse, Y: impulse.Neg()}, point, true)
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
		point := s.jointIDs[i].GetLocalAnchorA()
		text := fmt.Sprintf("%.2f m, %.1f deg", linear, 360*angular)
		s.Context.Draw.DrawString(point, text, dbox2d.ColorWhite)
	}
	s.Base.Step()
}

// UserConstraint hand-solves two soft constraints from one body to a fixed anchor.
type UserConstraint struct {
	Base
	bodyId   dbox2d.BodyId
	impulses [2]dbox2d.Q
}

// NewUserConstraint builds the hand-solved user-constraint scene.
func NewUserConstraint(ctx *SampleContext) Sample {
	s := &UserConstraint{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 3, Y: -1}
		ctx.Camera.Zoom = 25 * 0.15
	}

	box := dbox2d.MakeBox(fixed.Q32One(), fixed.Q32Half())
	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Density = fixed.Q32FromInt(20)

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	bodyDef.GravityScale = fixed.Q32One()
	bodyDef.AngularDamping = fixed.Q32Half()
	bodyDef.LinearDamping = fixed.Q32MustParse("0.2")
	s.bodyId = dbox2d.CreateBody(s.WorldId, &bodyDef)
	dbox2d.CreatePolygonShape(s.bodyId, &shapeDef, &box)

	s.impulses = [2]dbox2d.Q{}
	return s
}

// Step advances the scene and applies the hand-solved soft constraints.
func (s *UserConstraint) Step() {
	s.Base.Step()

	s.Context.Draw.DrawTransform(dbox2d.TransformIdentity())
	if s.Context.Settings.Pause {
		return
	}

	timeStep := fixed.Q32Zero()
	if s.Context.Settings.Hertz > 0 {
		timeStep = FromFloat64(1.0 / s.Context.Settings.Hertz)
	}
	if timeStep.Eq(fixed.Q32Zero()) {
		return
	}

	invTimeStep := FromFloat64(s.Context.Settings.Hertz)
	constraintHertz := fixed.Q32FromInt(3)
	dampingRatioZeta := fixed.Q32MustParse("0.7")
	maxForce := fixed.Q32FromInt(1000)
	one := fixed.Q32One()
	zero := fixed.Q32Zero()
	omega := dbox2d.Pi().Mul(fixed.Q32FromInt(2)).Mul(constraintHertz)
	sigma := fixed.Q32FromInt(2).Mul(dampingRatioZeta).Add(timeStep.Mul(omega))
	softness := timeStep.Mul(omega).Mul(sigma)
	impulseCoefficient := one.Div(one.Add(softness))
	massCoefficient := softness.Mul(impulseCoefficient)
	biasCoefficient := omega.Div(sigma)

	localAnchors := [2]dbox2d.Vec2{
		{X: one, Y: fixed.Q32Half().Neg()},
		{X: one, Y: fixed.Q32Half()},
	}
	mass := s.bodyId.GetMass()
	threshold := fixed.Q32MustParse("0.0001")
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
	turnRadians := dbox2d.Pi().Mul(fixed.Q32FromInt(2))
	omegaRad := omegaB.Mul(turnRadians)
	pB := s.bodyId.GetWorldCenterOfMass()

	for i := range localAnchors {
		anchorA := dbox2d.Vec2{X: fixed.Q32FromInt(3)}
		anchorB := s.bodyId.GetWorldPoint(localAnchors[i])
		deltaAnchor := anchorB.Sub(anchorA)

		slackLength := one
		length := deltaAnchor.Len()
		constraintError := length.Sub(slackLength)
		if constraintError.Less(zero) || length.Less(fixed.Q32MustParse("0.001")) {
			s.Context.Draw.DrawSegment(anchorA, anchorB, dbox2d.ColorLightCyan)
			s.impulses[i] = zero
			continue
		}

		s.Context.Draw.DrawSegment(anchorA, anchorB, dbox2d.ColorViolet)
		axis := deltaAnchor.Normalize()
		rB := anchorB.Sub(pB)
		Jb := dbox2d.Cross(rB, axis)
		K := invMass.Add(Jb.Mul(invI).Mul(Jb))
		invK := zero
		if !K.Less(threshold) {
			invK = one.Div(K)
		}

		Cdot := vB.Dot(axis).Add(Jb.Mul(omegaRad))
		impulse := massCoefficient.Mul(invK).Mul(Cdot.Add(biasCoefficient.Mul(constraintError))).Neg()
		appliedImpulse := impulse.Clamp(maxForce.Mul(timeStep).Neg(), zero)

		vB = dbox2d.MulAdd(vB, invMass.Mul(appliedImpulse), axis)
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
	doorId            dbox2d.BodyId
	jointId           dbox2d.JointId
	impulse           dbox2d.Q
	impulseFloat      float64
	translationError  dbox2d.Q
	jointHertz        dbox2d.Q
	jointHertzFloat   float64
	jointDampingRatio dbox2d.Q
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

	groundDef := dbox2d.DefaultBodyDef()
	groundId := dbox2d.CreateBody(s.WorldId, &groundDef)

	s.enableLimit = true
	s.impulse = fixed.Q32FromInt(50000)
	s.impulseFloat = 50000
	s.translationError = fixed.Q32Zero()
	s.jointHertz = fixed.Q32FromInt(240)
	s.jointHertzFloat = 240
	s.jointDampingRatio = fixed.Q32One()
	s.dampingRatioFloat = 1

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	bodyDef.Position = dbox2d.Vec2{Y: fixed.Q32MustParse("1.5")}
	bodyDef.GravityScale = fixed.Q32Zero()
	s.doorId = dbox2d.CreateBody(s.WorldId, &bodyDef)

	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Density = fixed.Q32FromInt(1000)
	box := dbox2d.MakeBox(fixed.Q32MustParse("0.1"), fixed.Q32MustParse("1.5"))
	dbox2d.CreatePolygonShape(s.doorId, &shapeDef, &box)

	jointDef := dbox2d.DefaultRevoluteJointDef()
	jointDef.BodyIdA = groundId
	jointDef.BodyIdB = s.doorId
	jointDef.LocalAnchorA = dbox2d.Vec2{}
	jointDef.LocalAnchorB = dbox2d.Vec2{Y: fixed.Q32MustParse("-1.5")}
	jointDef.TargetAngle = fixed.Q32Zero()
	jointDef.EnableSpring = true
	jointDef.Hertz = fixed.Q32One()
	jointDef.DampingRatio = fixed.Q32Half()
	jointDef.MotorSpeed = fixed.Q32Zero()
	jointDef.MaxMotorTorque = fixed.Q32Zero()
	jointDef.EnableMotor = false
	jointDef.ReferenceAngle = fixed.Q32Zero()
	jointDef.LowerAngle = fixed.Q32FromRatio(-1, 4)
	jointDef.UpperAngle = fixed.Q32FromRatio(1, 4)
	jointDef.EnableLimit = s.enableLimit
	s.jointId = dbox2d.CreateRevoluteJoint(s.WorldId, &jointDef)
	s.jointId.SetConstraintTuning(s.jointHertz, s.jointDampingRatio)

	return s
}

// UpdateGui exposes the door impulse, limit, and spring controls.
func (s *Door) UpdateGui() {
	const height = 220
	gui := s.Context.Gui
	gui.Begin("Door", 10, s.Context.Camera.Height-height-50, 240, height)

	if gui.Button("impulse") {
		point := s.doorId.GetWorldPoint(dbox2d.Vec2{Y: fixed.Q32MustParse("1.5")})
		s.doorId.ApplyLinearImpulse(dbox2d.Vec2{X: s.impulse}, point, true)
		s.translationError = fixed.Q32Zero()
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

	point := s.doorId.GetWorldPoint(dbox2d.Vec2{Y: fixed.Q32MustParse("1.5")})
	s.Context.Draw.DrawPoint(point, fixed.Q32FromInt(5), dbox2d.ColorDarkKhaki)
	s.Context.Draw.DrawTransform(dbox2d.TransformIdentity())

	translationError := s.jointId.GetLinearSeparation()
	s.translationError = s.translationError.Max(translationError)
	s.DrawTextLine("translation error = %s", s.translationError)
}

// Ragdoll exposes the human joint tuning controls.
type Ragdoll struct {
	Base
	jointFrictionTorque      dbox2d.Q
	jointFrictionTorqueFloat float64
	jointHertz               dbox2d.Q
	jointHertzFloat          float64
	jointDampingRatio        dbox2d.Q
	jointDampingRatioFloat   float64
	human                    human
}

// NewRagdoll builds the ragdoll scene.
func NewRagdoll(ctx *SampleContext) Sample {
	s := &Ragdoll{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 12}
		ctx.Camera.Zoom = 16
	}

	groundDef := dbox2d.DefaultBodyDef()
	groundId := dbox2d.CreateBody(s.WorldId, &groundDef)
	shapeDef := dbox2d.DefaultShapeDef()
	segment := dbox2d.Segment{
		Point1: dbox2d.Vec2{X: fixed.Q32FromInt(-20)},
		Point2: dbox2d.Vec2{X: fixed.Q32FromInt(20)},
	}
	dbox2d.CreateSegmentShape(groundId, &shapeDef, &segment)

	s.jointFrictionTorque = fixed.Q32MustParse("0.03")
	s.jointFrictionTorqueFloat = 0.03
	s.jointHertz = fixed.Q32FromInt(5)
	s.jointHertzFloat = 5
	s.jointDampingRatio = fixed.Q32Half()
	s.jointDampingRatioFloat = 0.5

	s.Spawn()
	s.WorldId.SetContactTuning(fixed.Q32FromInt(240), fixed.Q32Zero(), fixed.Q32FromInt(2))
	return s
}

func (s *Ragdoll) Spawn() {
	s.human = createHuman(
		s.WorldId,
		dbox2d.Vec2{Y: fixed.Q32FromInt(25)},
		fixed.Q32One(),
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
		s.human.setJointFrictionTorque(s.jointFrictionTorque)
	}
	if gui.SliderFloat("Hertz", &s.jointHertzFloat, 0, 10) {
		s.jointHertz = FromFloat64(s.jointHertzFloat)
		s.human.setJointSpringHertz(s.jointHertz)
	}
	if gui.SliderFloat("Damping", &s.jointDampingRatioFloat, 0, 4) {
		s.jointDampingRatio = FromFloat64(s.jointDampingRatioFloat)
		s.human.setJointDampingRatio(s.jointDampingRatio)
	}
	if gui.Button("Respawn") {
		s.human.destroy()
		s.Spawn()
	}

	gui.End()
}

// ScaleRagdoll exposes the human scale control.
type ScaleRagdoll struct {
	Base
	scale      dbox2d.Q
	scaleFloat float64
	human      human
}

// NewScaleRagdoll builds the scalable ragdoll scene.
func NewScaleRagdoll(ctx *SampleContext) Sample {
	s := &ScaleRagdoll{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 4.5}
		ctx.Camera.Zoom = 6
	}

	groundDef := dbox2d.DefaultBodyDef()
	groundId := dbox2d.CreateBody(s.WorldId, &groundDef)
	shapeDef := dbox2d.DefaultShapeDef()
	box := dbox2d.MakeOffsetBox(
		fixed.Q32FromInt(20),
		fixed.Q32One(),
		dbox2d.Vec2{Y: fixed.Q32FromInt(-1)},
		dbox2d.RotIdentity(),
	)
	dbox2d.CreatePolygonShape(groundId, &shapeDef, &box)

	s.scale = fixed.Q32One()
	s.scaleFloat = 1
	s.Spawn()
	return s
}

func (s *ScaleRagdoll) Spawn() {
	jointFrictionTorque := fixed.Q32MustParse("0.03")
	jointHertz := fixed.Q32One()
	jointDampingRatio := fixed.Q32Half()
	s.human = createHuman(
		s.WorldId,
		dbox2d.Vec2{Y: fixed.Q32FromInt(5)},
		s.scale,
		jointFrictionTorque,
		jointHertz,
		jointDampingRatio,
		1,
		nil,
		false,
	)
	s.human.applyRandomAngularImpulse(fixed.Q32FromInt(10))
}

// UpdateGui exposes the ragdoll scale control.
func (s *ScaleRagdoll) UpdateGui() {
	const height = 60
	gui := s.Context.Gui
	gui.Begin("Scale Ragdoll", 10, s.Context.Camera.Height-height-50, 260, height)

	if gui.SliderFloat("Scale", &s.scaleFloat, 0.1, 10) {
		s.scale = FromFloat64(s.scaleFloat)
		s.human.setScale(s.scale)
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

	groundDef := dbox2d.DefaultBodyDef()
	groundID := dbox2d.CreateBody(s.WorldId, &groundDef)
	{
		zero := fixed.Q32Zero()
		minusTwenty := fixed.Q32FromInt(-20)
		twenty := fixed.Q32FromInt(20)
		dx := fixed.Q32FromInt(5)
		x := twenty
		hs := []dbox2d.Q{
			fixed.Q32MustParse("0.25"), fixed.Q32One(), fixed.Q32FromInt(4), zero, zero,
			fixed.Q32FromInt(-1), fixed.Q32FromInt(-2), fixed.Q32FromInt(-2),
			fixed.Q32MustParse("-1.25"), zero,
		}
		// Filled in reverse to match the line list convention, as the reference.
		var points [25]dbox2d.Vec2
		count := 24
		put := func(px, py dbox2d.Q) {
			points[count] = dbox2d.Vec2{X: px, Y: py}
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
		put(x.Add(fixed.Q32FromInt(40)), zero)
		put(x.Add(fixed.Q32FromInt(40)), minusTwenty)

		chainDef := dbox2d.DefaultChainDef()
		chainDef.Points = points[:]
		chainDef.IsLoop = true
		dbox2d.CreateChain(groundID, &chainDef)

		x = x.Add(fixed.Q32FromInt(80))
		shapeDef := dbox2d.DefaultShapeDef()
		segment := dbox2d.Segment{
			Point1: dbox2d.Vec2{X: x},
			Point2: dbox2d.Vec2{X: x.Add(fixed.Q32FromInt(40))},
		}
		dbox2d.CreateSegmentShape(groundID, &shapeDef, &segment)

		x = x.Add(fixed.Q32FromInt(40))
		segment = dbox2d.Segment{
			Point1: dbox2d.Vec2{X: x},
			Point2: dbox2d.Vec2{X: x.Add(fixed.Q32FromInt(10)), Y: fixed.Q32FromInt(5)},
		}
		dbox2d.CreateSegmentShape(groundID, &shapeDef, &segment)

		x = x.Add(fixed.Q32FromInt(20))
		segment = dbox2d.Segment{
			Point1: dbox2d.Vec2{X: x},
			Point2: dbox2d.Vec2{X: x.Add(fixed.Q32FromInt(40))},
		}
		dbox2d.CreateSegmentShape(groundID, &shapeDef, &segment)

		x = x.Add(fixed.Q32FromInt(40))
		segment = dbox2d.Segment{
			Point1: dbox2d.Vec2{X: x},
			Point2: dbox2d.Vec2{X: x, Y: fixed.Q32FromInt(20)},
		}
		dbox2d.CreateSegmentShape(groundID, &shapeDef, &segment)
	}

	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Position = dbox2d.Vec2{X: fixed.Q32FromInt(140), Y: fixed.Q32One()}
		bodyDef.AngularVelocity = radiansToTurns(1)
		bodyDef.Type = dbox2d.DynamicBody
		bodyID := dbox2d.CreateBody(s.WorldId, &bodyDef)
		shapeDef := dbox2d.DefaultShapeDef()
		box := dbox2d.MakeBox(fixed.Q32FromInt(10), fixed.Q32MustParse("0.25"))
		dbox2d.CreatePolygonShape(bodyID, &shapeDef, &box)

		jointDef := dbox2d.DefaultRevoluteJointDef()
		jointDef.BodyIdA = groundID
		jointDef.BodyIdB = bodyID
		jointDef.LocalAnchorA = groundID.GetLocalPoint(bodyDef.Position)
		jointDef.LocalAnchorB = bodyID.GetLocalPoint(bodyDef.Position)
		jointDef.LowerAngle = fixed.Q32FromRatio(-8, 360)
		jointDef.UpperAngle = fixed.Q32FromRatio(8, 360)
		jointDef.EnableLimit = true
		dbox2d.CreateRevoluteJoint(s.WorldId, &jointDef)
	}

	{
		shapeDef := dbox2d.DefaultShapeDef()
		capsule := dbox2d.Capsule{
			Center1: dbox2d.Vec2{X: fixed.Q32FromInt(-1)},
			Center2: dbox2d.Vec2{X: fixed.Q32One()},
			Radius:  fixed.Q32MustParse("0.125"),
		}
		jointDef := dbox2d.DefaultRevoluteJointDef()
		prevBodyID := groundID
		for i := range 20 {
			bodyDef := dbox2d.DefaultBodyDef()
			bodyDef.Type = dbox2d.DynamicBody
			bodyDef.Position = dbox2d.Vec2{
				X: fixed.Q32FromInt(161).Add(fixed.Q32FromInt(2 * i)),
				Y: fixed.Q32MustParse("-0.125"),
			}
			bodyID := dbox2d.CreateBody(s.WorldId, &bodyDef)
			dbox2d.CreateCapsuleShape(bodyID, &shapeDef, &capsule)

			pivot := dbox2d.Vec2{
				X: fixed.Q32FromInt(160).Add(fixed.Q32FromInt(2 * i)),
				Y: fixed.Q32MustParse("-0.125"),
			}
			jointDef.BodyIdA = prevBodyID
			jointDef.BodyIdB = bodyID
			jointDef.LocalAnchorA = prevBodyID.GetLocalPoint(pivot)
			jointDef.LocalAnchorB = bodyID.GetLocalPoint(pivot)
			dbox2d.CreateRevoluteJoint(s.WorldId, &jointDef)
			prevBodyID = bodyID
		}

		pivot := dbox2d.Vec2{X: fixed.Q32FromInt(200), Y: fixed.Q32MustParse("-0.125")}
		jointDef.BodyIdA = prevBodyID
		jointDef.BodyIdB = groundID
		jointDef.LocalAnchorA = prevBodyID.GetLocalPoint(pivot)
		jointDef.LocalAnchorB = groundID.GetLocalPoint(pivot)
		jointDef.EnableMotor = true
		jointDef.MaxMotorTorque = fixed.Q32FromInt(50)
		dbox2d.CreateRevoluteJoint(s.WorldId, &jointDef)
	}

	{
		box := dbox2d.MakeBox(fixed.Q32Half(), fixed.Q32Half())
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		shapeDef := dbox2d.DefaultShapeDef()
		shapeDef.Material.Friction = fixed.Q32MustParse("0.25")
		shapeDef.Material.Restitution = fixed.Q32MustParse("0.25")
		shapeDef.Density = fixed.Q32MustParse("0.25")
		for i := range 5 {
			bodyDef.Position = dbox2d.Vec2{X: fixed.Q32FromInt(230), Y: fixed.Q32FromInt(i).Add(fixed.Q32Half())}
			bodyID := dbox2d.CreateBody(s.WorldId, &bodyDef)
			dbox2d.CreatePolygonShape(bodyID, &shapeDef, &box)
		}
	}

	s.throttle = 0
	s.speed = 35
	s.torque = 5
	s.hertz = 5
	s.dampingRatio = 0.7
	s.car.spawn(s.WorldId, dbox2d.Vec2{}, fixed.Q32One(), FromFloat64(s.hertz), FromFloat64(s.dampingRatio), FromFloat64(s.torque), nil)
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
		s.car.setSpeed(fixed.Q32Zero())
	}
	if s.keyDown(KeyD) {
		s.throttle = -1
		s.car.setSpeed(FromFloat64(-s.speed))
	}

	s.DrawTextLine("Keys: left = a, brake = s, right = d")
	linearVelocity := s.car.chassisId.GetLinearVelocity()
	kph := linearVelocity.X.Mul(fixed.Q32MustParse("3.6"))
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

	groundDef := dbox2d.DefaultBodyDef()
	groundID := dbox2d.CreateBody(s.WorldId, &groundDef)
	shapeDef := dbox2d.DefaultShapeDef()
	segment := dbox2d.Segment{
		Point1: dbox2d.Vec2{X: fixed.Q32FromInt(-20)},
		Point2: dbox2d.Vec2{X: fixed.Q32FromInt(20)},
	}
	dbox2d.CreateSegmentShape(groundID, &shapeDef, &segment)

	s.donut.create(
		s.WorldId,
		dbox2d.Vec2{Y: fixed.Q32FromInt(10)},
		fixed.Q32FromInt(2),
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

	groundDef := dbox2d.DefaultBodyDef()
	groundID := dbox2d.CreateBody(s.WorldId, &groundDef)
	shapeDef := dbox2d.DefaultShapeDef()
	segment := dbox2d.Segment{
		Point1: dbox2d.Vec2{X: fixed.Q32FromInt(-20)},
		Point2: dbox2d.Vec2{X: fixed.Q32FromInt(20)},
	}
	dbox2d.CreateSegmentShape(groundID, &shapeDef, &segment)

	box := dbox2d.MakeOffsetBox(
		fixed.Q32One(),
		fixed.Q32One(),
		dbox2d.Vec2{Y: fixed.Q32One()},
		dbox2d.RotIdentity(),
	)
	dbox2d.CreatePolygonShape(groundID, &shapeDef, &box)

	y := fixed.Q32FromInt(4)
	for range 4 {
		d := doohickey{}
		d.spawn(s.WorldId, dbox2d.Vec2{Y: y}, fixed.Q32Half())
		y = y.Add(fixed.Q32FromInt(2))
	}

	return s
}

// ScissorLift builds a three-link lift with a distance-joint motor.
type ScissorLift struct {
	Base

	liftJointID dbox2d.JointId
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

	groundDef := dbox2d.DefaultBodyDef()
	groundID := dbox2d.CreateBody(s.WorldId, &groundDef)
	shapeDef := dbox2d.DefaultShapeDef()
	segment := dbox2d.Segment{
		Point1: dbox2d.Vec2{X: fixed.Q32FromInt(-20)},
		Point2: dbox2d.Vec2{X: fixed.Q32FromInt(20)},
	}
	dbox2d.CreateSegmentShape(groundID, &shapeDef, &segment)

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	bodyDef.SleepThreshold = fixed.Q32MustParse("0.01")
	capsule := dbox2d.Capsule{
		Center1: dbox2d.Vec2{X: fixed.Q32MustParse("-2.5")},
		Center2: dbox2d.Vec2{X: fixed.Q32MustParse("2.5")},
		Radius:  fixed.Q32MustParse("0.15"),
	}

	baseID1 := groundID
	baseID2 := groundID
	baseAnchor1 := dbox2d.Vec2{X: fixed.Q32MustParse("-2.5"), Y: fixed.Q32MustParse("0.2")}
	baseAnchor2 := dbox2d.Vec2{X: fixed.Q32MustParse("2.5"), Y: fixed.Q32MustParse("0.2")}
	y := fixed.Q32MustParse("0.5")
	var linkID1 dbox2d.BodyId

	for i := range 3 {
		bodyDef.Position = dbox2d.Vec2{Y: y}
		bodyDef.Rotation = dbox2d.MakeRot(radiansToTurns(0.15))
		bodyID1 := dbox2d.CreateBody(s.WorldId, &bodyDef)
		dbox2d.CreateCapsuleShape(bodyID1, &shapeDef, &capsule)

		bodyDef.Position = dbox2d.Vec2{Y: y}
		bodyDef.Rotation = dbox2d.MakeRot(radiansToTurns(0.15).Neg())
		bodyID2 := dbox2d.CreateBody(s.WorldId, &bodyDef)
		dbox2d.CreateCapsuleShape(bodyID2, &shapeDef, &capsule)

		if i == 1 {
			linkID1 = bodyID2
		}

		revoluteDef := dbox2d.DefaultRevoluteJointDef()
		revoluteDef.BodyIdA = baseID1
		revoluteDef.BodyIdB = bodyID1
		revoluteDef.LocalAnchorA = baseAnchor1
		revoluteDef.LocalAnchorB = dbox2d.Vec2{X: fixed.Q32MustParse("-2.5")}
		revoluteDef.CollideConnected = i == 0
		dbox2d.CreateRevoluteJoint(s.WorldId, &revoluteDef)

		if i == 0 {
			wheelDef := dbox2d.DefaultWheelJointDef()
			wheelDef.BodyIdA = baseID2
			wheelDef.BodyIdB = bodyID2
			wheelDef.LocalAxisA = dbox2d.Vec2{X: fixed.Q32One()}
			wheelDef.LocalAnchorA = baseAnchor2
			wheelDef.LocalAnchorB = dbox2d.Vec2{X: fixed.Q32MustParse("2.5")}
			wheelDef.EnableSpring = false
			wheelDef.CollideConnected = true
			dbox2d.CreateWheelJoint(s.WorldId, &wheelDef)
		} else {
			revoluteDef.BodyIdA = baseID2
			revoluteDef.BodyIdB = bodyID2
			revoluteDef.LocalAnchorA = baseAnchor2
			revoluteDef.LocalAnchorB = dbox2d.Vec2{X: fixed.Q32MustParse("2.5")}
			revoluteDef.CollideConnected = false
			dbox2d.CreateRevoluteJoint(s.WorldId, &revoluteDef)
		}

		revoluteDef.BodyIdA = bodyID1
		revoluteDef.BodyIdB = bodyID2
		revoluteDef.LocalAnchorA = dbox2d.Vec2{}
		revoluteDef.LocalAnchorB = dbox2d.Vec2{}
		revoluteDef.CollideConnected = false
		dbox2d.CreateRevoluteJoint(s.WorldId, &revoluteDef)

		baseID1 = bodyID2
		baseID2 = bodyID1
		baseAnchor1 = dbox2d.Vec2{X: fixed.Q32MustParse("-2.5")}
		baseAnchor2 = dbox2d.Vec2{X: fixed.Q32MustParse("2.5")}
		y = y.Add(fixed.Q32One())
	}

	bodyDef.Position = dbox2d.Vec2{Y: y}
	bodyDef.Rotation = dbox2d.RotIdentity()
	platformID := dbox2d.CreateBody(s.WorldId, &bodyDef)
	box := dbox2d.MakeBox(fixed.Q32FromInt(3), fixed.Q32MustParse("0.2"))
	dbox2d.CreatePolygonShape(platformID, &shapeDef, &box)

	revoluteDef := dbox2d.DefaultRevoluteJointDef()
	revoluteDef.BodyIdA = platformID
	revoluteDef.BodyIdB = baseID1
	revoluteDef.LocalAnchorA = dbox2d.Vec2{X: fixed.Q32MustParse("-2.5"), Y: fixed.Q32MustParse("-0.4")}
	revoluteDef.LocalAnchorB = baseAnchor1
	revoluteDef.CollideConnected = true
	dbox2d.CreateRevoluteJoint(s.WorldId, &revoluteDef)

	wheelDef := dbox2d.DefaultWheelJointDef()
	wheelDef.BodyIdA = platformID
	wheelDef.BodyIdB = baseID2
	wheelDef.LocalAxisA = dbox2d.Vec2{X: fixed.Q32One()}
	wheelDef.LocalAnchorA = dbox2d.Vec2{X: fixed.Q32MustParse("2.5"), Y: fixed.Q32MustParse("-0.4")}
	wheelDef.LocalAnchorB = baseAnchor2
	wheelDef.EnableSpring = false
	wheelDef.CollideConnected = true
	dbox2d.CreateWheelJoint(s.WorldId, &wheelDef)

	s.enableMotor = false
	s.motorSpeed = 0.25
	s.motorForce = 2000

	distanceDef := dbox2d.DefaultDistanceJointDef()
	distanceDef.BodyIdA = groundID
	distanceDef.BodyIdB = linkID1
	distanceDef.LocalAnchorA = dbox2d.Vec2{X: fixed.Q32MustParse("-2.5"), Y: fixed.Q32MustParse("0.2")}
	distanceDef.LocalAnchorB = dbox2d.Vec2{X: fixed.Q32MustParse("0.5")}
	distanceDef.EnableSpring = true
	distanceDef.MinLength = fixed.Q32MustParse("0.2")
	distanceDef.MaxLength = fixed.Q32MustParse("5.5")
	distanceDef.EnableLimit = true
	distanceDef.EnableMotor = s.enableMotor
	distanceDef.MotorSpeed = FromFloat64(s.motorSpeed)
	distanceDef.MaxMotorForce = FromFloat64(s.motorForce)
	s.liftJointID = dbox2d.CreateDistanceJoint(s.WorldId, &distanceDef)

	var decoration car
	decoration.spawn(s.WorldId, dbox2d.Vec2{X: fixed.Q32Zero(), Y: y.Add(fixed.Q32FromInt(2))},
		fixed.Q32One(), fixed.Q32FromInt(3), fixed.Q32MustParse("0.7"), fixed.Q32Zero(), nil)
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
		s.liftJointID.SetMotorSpeed(FromFloat64(s.motorSpeed))
		s.liftJointID.WakeBodies()
	}
	gui.End()
}

// GearLift builds a geared lift with a hanging door.
type GearLift struct {
	Base

	driverId    dbox2d.JointId
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
	}

	groundDef := dbox2d.DefaultBodyDef()
	groundId := dbox2d.CreateBody(g.WorldId, &groundDef)
	material := dbox2d.DefaultSurfaceMaterial()
	material.CustomColor = uint32(dbox2d.ColorDarkSeaGreen)
	chainDef := dbox2d.DefaultChainDef()
	path := "m 63.500002,201.08333 103.187498,0 1e-5,-37.04166 h -2.64584 l 0,34.39583 h -42.33333 v -2.64583 l " +
		"-2.64584,-1e-5 v -2.64583 h -2.64583 v -2.64584 h -2.64584 v -2.64583 H 111.125 v -2.64583 h -2.64583 v " +
		"-2.64583 h -2.64583 v -2.64584 l -2.64584,1e-5 v -2.64583 l -2.64583,-1e-5 V 174.625 h -2.645834 v -2.64584 l " +
		"-2.645833,1e-5 v -2.64584 H 92.60417 v -2.64583 h -2.645834 v -2.64583 l -26.458334,0 0,37.04166"
	offset := dbox2d.Vec2{X: fixed.Q32FromInt(-120), Y: fixed.Q32FromInt(-200)}
	chainDef.Points = parsePath(path, offset, 64, fixed.Q32MustParse("0.2"))
	chainDef.IsLoop = true
	chainDef.Materials = []dbox2d.SurfaceMaterial{material}
	dbox2d.CreateChain(groundId, &chainDef)

	gearRadius := fixed.Q32FromInt(1)
	toothHalfWidth := fixed.Q32MustParse("0.09")
	toothHalfHeight := fixed.Q32MustParse("0.06")
	toothRadius := fixed.Q32MustParse("0.03")
	linkHalfLength := fixed.Q32MustParse("0.07")
	linkRadius := fixed.Q32MustParse("0.05")
	doorHalfHeight := fixed.Q32MustParse("1.5")
	gearPosition1 := dbox2d.Vec2{X: fixed.Q32MustParse("-4.25"), Y: fixed.Q32MustParse("9.75")}
	gearPosition2 := gearPosition1.Add(dbox2d.Vec2{X: fixed.Q32FromInt(2), Y: fixed.Q32FromInt(1)})
	linkAttachPosition := gearPosition2.Add(dbox2d.Vec2{X: gearRadius.Add(toothHalfWidth.Mul(fixed.Q32FromInt(2))).Add(toothRadius)})
	doorPosition := linkAttachPosition.Sub(dbox2d.Vec2{Y: linkHalfLength.Mul(fixed.Q32FromInt(2)).Mul(fixed.Q32FromInt(40)).Add(doorHalfHeight)})

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	bodyDef.Position = gearPosition1
	driverBodyId := dbox2d.CreateBody(g.WorldId, &bodyDef)
	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Material.Friction = fixed.Q32MustParse("0.1")
	shapeDef.Material.CustomColor = uint32(dbox2d.ColorSaddleBrown)
	circle := dbox2d.Circle{Center: dbox2d.Vec2{}, Radius: gearRadius}
	dbox2d.CreateCircleShape(driverBodyId, &shapeDef, &circle)
	dq := dbox2d.MakeRot(fixed.Q32FromRatio(1, 16))
	center := dbox2d.Vec2{X: gearRadius.Add(toothHalfHeight)}
	rotation := dbox2d.RotIdentity()
	for range 16 {
		tooth := dbox2d.MakeOffsetRoundedBox(toothHalfWidth, toothHalfHeight, center, rotation, toothRadius)
		shapeDef.Material.CustomColor = uint32(dbox2d.ColorGray)
		dbox2d.CreatePolygonShape(driverBodyId, &shapeDef, &tooth)
		rotation = dbox2d.MulRot(dq, rotation)
		center = dbox2d.RotateVector(rotation, dbox2d.Vec2{X: gearRadius.Add(toothHalfHeight)})
	}
	revoluteDef := dbox2d.DefaultRevoluteJointDef()
	g.motorTorque = 80
	g.motorSpeed = 0
	g.enableMotor = true
	revoluteDef.BodyIdA = groundId
	revoluteDef.BodyIdB = driverBodyId
	revoluteDef.LocalAnchorA = groundId.GetLocalPoint(gearPosition1)
	revoluteDef.LocalAnchorB = dbox2d.Vec2{}
	revoluteDef.EnableMotor = g.enableMotor
	revoluteDef.MaxMotorTorque = FromFloat64(g.motorTorque)
	revoluteDef.MotorSpeed = radiansToTurns(g.motorSpeed)
	g.driverId = dbox2d.CreateRevoluteJoint(g.WorldId, &revoluteDef)

	bodyDef.Position = gearPosition2
	followerId := dbox2d.CreateBody(g.WorldId, &bodyDef)
	shapeDef.Material.Friction = fixed.Q32MustParse("0.1")
	shapeDef.Material.CustomColor = uint32(dbox2d.ColorSaddleBrown)
	dbox2d.CreateCircleShape(followerId, &shapeDef, &circle)
	center = dbox2d.Vec2{X: gearRadius.Add(toothHalfWidth)}
	rotation = dbox2d.RotIdentity()
	for range 16 {
		tooth := dbox2d.MakeOffsetRoundedBox(toothHalfWidth, toothHalfHeight, center, rotation, toothRadius)
		shapeDef.Material.CustomColor = uint32(dbox2d.ColorGray)
		dbox2d.CreatePolygonShape(followerId, &shapeDef, &tooth)
		rotation = dbox2d.MulRot(dq, rotation)
		center = dbox2d.RotateVector(rotation, dbox2d.Vec2{X: gearRadius.Add(toothHalfWidth)})
	}
	revoluteDef = dbox2d.DefaultRevoluteJointDef()
	revoluteDef.BodyIdA = groundId
	revoluteDef.BodyIdB = followerId
	revoluteDef.LocalAnchorA = groundId.GetLocalPoint(gearPosition2)
	revoluteDef.LocalAnchorB = dbox2d.Vec2{}
	revoluteDef.EnableMotor = true
	revoluteDef.MaxMotorTorque = fixed.Q32MustParse("0.5")
	revoluteDef.ReferenceAngle = fixed.Q32FromRatio(1, 8)
	revoluteDef.LowerAngle = fixed.Q32FromRatio(-15, 100)
	revoluteDef.UpperAngle = fixed.Q32FromRatio(4, 10)
	revoluteDef.EnableLimit = true
	dbox2d.CreateRevoluteJoint(g.WorldId, &revoluteDef)

	capsule := dbox2d.Capsule{Center1: dbox2d.Vec2{Y: linkHalfLength.Neg()}, Center2: dbox2d.Vec2{Y: linkHalfLength}, Radius: linkRadius}
	shapeDef = dbox2d.DefaultShapeDef()
	shapeDef.Density = fixed.Q32FromInt(2)
	shapeDef.Material.CustomColor = uint32(dbox2d.ColorLightSteelBlue)
	jointDef := dbox2d.DefaultRevoluteJointDef()
	jointDef.MaxMotorTorque = fixed.Q32MustParse("0.05")
	jointDef.EnableMotor = true
	bodyDef = dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	position := linkAttachPosition.Add(dbox2d.Vec2{Y: linkHalfLength.Neg()})
	prevBodyId := followerId
	var lastLinkId dbox2d.BodyId
	for range 40 {
		bodyDef.Position = position
		bodyId := dbox2d.CreateBody(g.WorldId, &bodyDef)
		dbox2d.CreateCapsuleShape(bodyId, &shapeDef, &capsule)
		pivot := dbox2d.Vec2{X: position.X, Y: position.Y.Add(linkHalfLength)}
		jointDef.BodyIdA = prevBodyId
		jointDef.BodyIdB = bodyId
		jointDef.LocalAnchorA = prevBodyId.GetLocalPoint(pivot)
		jointDef.LocalAnchorB = bodyId.GetLocalPoint(pivot)
		dbox2d.CreateRevoluteJoint(g.WorldId, &jointDef)
		position.Y = position.Y.Sub(linkHalfLength.Mul(fixed.Q32FromInt(2)))
		prevBodyId = bodyId
		lastLinkId = bodyId
	}

	bodyDef = dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	bodyDef.Position = doorPosition
	doorBodyId := dbox2d.CreateBody(g.WorldId, &bodyDef)
	box := dbox2d.MakeBox(fixed.Q32MustParse("0.15"), doorHalfHeight)
	shapeDef = dbox2d.DefaultShapeDef()
	shapeDef.Material.Friction = fixed.Q32MustParse("0.1")
	shapeDef.Material.CustomColor = uint32(dbox2d.ColorDarkCyan)
	dbox2d.CreatePolygonShape(doorBodyId, &shapeDef, &box)
	pivot := doorPosition.Add(dbox2d.Vec2{Y: doorHalfHeight})
	revoluteDef = dbox2d.DefaultRevoluteJointDef()
	revoluteDef.BodyIdA = lastLinkId
	revoluteDef.BodyIdB = doorBodyId
	revoluteDef.LocalAnchorA = lastLinkId.GetLocalPoint(pivot)
	revoluteDef.LocalAnchorB = dbox2d.Vec2{Y: doorHalfHeight}
	revoluteDef.EnableMotor = true
	revoluteDef.MaxMotorTorque = fixed.Q32MustParse("0.05")
	dbox2d.CreateRevoluteJoint(g.WorldId, &revoluteDef)
	prismaticDef := dbox2d.DefaultPrismaticJointDef()
	prismaticDef.BodyIdA = groundId
	prismaticDef.BodyIdB = doorBodyId
	prismaticDef.LocalAnchorA = groundId.GetLocalPoint(doorPosition)
	prismaticDef.LocalAnchorB = dbox2d.Vec2{}
	prismaticDef.LocalAxisA = dbox2d.Vec2{Y: fixed.Q32One()}
	prismaticDef.MaxMotorForce = fixed.Q32MustParse("0.2")
	prismaticDef.EnableMotor = true
	prismaticDef.CollideConnected = true
	dbox2d.CreatePrismaticJoint(g.WorldId, &prismaticDef)

	bodyDef = dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	shapeDef = dbox2d.DefaultShapeDef()
	shapeDef.Material.RollingResistance = fixed.Q32MustParse("0.3")
	colors := [5]uint32{uint32(dbox2d.ColorGray), uint32(dbox2d.ColorGainsboro), uint32(dbox2d.ColorLightGray), uint32(dbox2d.ColorLightSlateGray), uint32(dbox2d.ColorDarkGray)}
	y := fixed.Q32MustParse("4.25")
	for range 20 {
		x := fixed.Q32MustParse("-3.15")
		for range 10 {
			bodyDef.Position = dbox2d.Vec2{X: x, Y: y}
			bodyId := dbox2d.CreateBody(g.WorldId, &bodyDef)
			poly := randomPolygon(fixed.Q32MustParse("0.1"))
			poly.Radius = randomFloatRange(fixed.Q32MustParse("0.01"), fixed.Q32MustParse("0.02"))
			shapeDef.Material.CustomColor = colors[randomIntRange(0, 4)]
			dbox2d.CreatePolygonShape(bodyId, &shapeDef, &poly)
			x = x.Add(fixed.Q32MustParse("0.2"))
		}
		y = y.Add(fixed.Q32MustParse("0.2"))
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
		g.driverId.SetMotorSpeed(radiansToTurns(g.motorSpeed))
		g.driverId.WakeBodies()
	}
	gui.End()
}

// Step advances the gear-lift simulation.
func (g *GearLift) Step() {
	if g.keyDown(KeyA) {
		g.motorSpeed = max(-0.3, g.motorSpeed-0.01)
		g.driverId.SetMotorSpeed(radiansToTurns(g.motorSpeed))
		g.driverId.WakeBodies()
	}
	if g.keyDown(KeyD) {
		g.motorSpeed = min(0.3, g.motorSpeed+0.01)
		g.driverId.SetMotorSpeed(radiansToTurns(g.motorSpeed))
		g.driverId.WakeBodies()
	}
	g.Base.Step()
}
