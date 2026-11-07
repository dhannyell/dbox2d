// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/sample_character.cpp of Box2D v3.2.0

package samples

import (
	"math"
	"sync/atomic"

	"github.com/dhannyell/dbox2d"
)

func init() {
	RegisterSample("Character", "Geometric Mover", NewGeometricMoverSample)
	RegisterSample("Character", "Dynamic Mover", NewDynamicMoverSample)
	RegisterSample("Character", "Ghost Culling", NewGhostCulling)
}

// Collision bits of the mover scenes.
const (
	moverStaticBit  uint64 = 0x0001
	moverMoverBit   uint64 = 0x0002
	moverDynamicBit uint64 = 0x0004

	// moverAllBits is the reference's ~0u, a 32-bit mask.
	moverAllBits uint64 = 0xFFFFFFFF
)

type moverCastResult struct {
	point    b2.Vec2
	normal   b2.Vec2
	bodyId   b2.BodyId
	fraction b2.Q
	hit      bool
}

var (
	moverElevatorBase      = b2.V2(112, 10)
	moverElevatorAmplitude = 4.0
)

// moverDrawPogo draws the ground probe of a mover.
func moverDrawPogo(draw *b2.DebugDraw, origin, translation b2.Vec2, fraction b2.Q, hit bool) {
	color := b2.ColorGray
	if hit {
		color = b2.ColorPlum
	}
	delta := translation.Mul(fraction)

	draw.DrawLineFcn(b2.ToPos(origin), b2.ToPos(origin.Add(delta)), b2.ColorGray)
	draw.DrawPointFcn(b2.ToPos(origin.Add(delta)), b2.F(10), color)
}

// moverChain creates a looped chain from an SVG path on a new static body.
func moverChain(worldId b2.WorldId, bodyDef *b2.BodyDef, path string, offset b2.Vec2) b2.BodyId {
	groundId := b2.CreateBody(worldId, bodyDef)
	points := parsePath(path, offset, 64, b2.QFromRatio(1, 5))

	chainDef := b2.DefaultChainDef()
	chainDef.Points = points
	chainDef.IsLoop = true
	b2.CreateChain(groundId, &chainDef)
	return groundId
}

const moverGroundPath2 = "M 2.6458333,201.08333 H 399.52085 v -23.8125 h -23.8125 l 21.16667,18.52084 -232.83335,-1e-5 -0.0575,2.64584 h " +
	"-5.29166 v -2.64583 l -7.86855,-1e-5 -0.0114,-2.64583 h -2.64583 l -2.64583,2.64584 h -7.9375 l " +
	"-2.64584,2.64583 -2.58891,-2.64584 -10.64031,2e-5 v -2.64583 l -7.9375,0 v -2.64584 l -7.9375,0 v -2.64583 l " +
	"-7.9375,0 v -2.64583 l -7.9375,0 v -2.64583 l -7.937501,-1e-5 v -2.64584 l -7.9375,1e-5 v -2.64583 l " +
	"-7.937501,-2e-5 V 174.625 l -7.937501,1e-5 v -2.64584 l -5.291669,0 -7.9375,-2.64584 -7.9375,-2.64583 " +
	"-5.291667,-5.29167 H 21.166667 L 13.229167,158.75 5.2916668,153.45833 H 2.6458334 l -10e-8,47.625"

// moverBridge creates the plank bridge between the two ground bodies.
func moverBridge(worldId b2.WorldId, groundId1, groundId2 b2.BodyId) {
	box := b2.MakeBox(b2.F(0.55), b2.QFromRatio(1, 8))

	shapeDef := b2.DefaultShapeDef()

	jointDef := b2.DefaultRevoluteJointDef()
	jointDef.MaxMotorTorque = b2.F(10)
	jointDef.EnableMotor = true
	jointDef.Hertz = b2.F(3)
	jointDef.DampingRatio = b2.F(0.8)
	jointDef.EnableSpring = true

	xBase := b2.F(48.7)
	yBase := b2.F(9.2)
	count := 50
	prevBodyId := groundId1
	for i := range count {
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.ToPos(b2.Vec2{X: xBase.Add(b2.QHalf()).Add(b2.F(i)), Y: yBase})
		bodyDef.AngularDamping = b2.F(0.2)
		bodyId := b2.CreateBody(worldId, &bodyDef)
		b2.CreatePolygonShape(bodyId, &shapeDef, &box)

		pivot := b2.Vec2{X: xBase.Add(b2.F(i)), Y: yBase}
		jointDef.Base.BodyIdA = prevBodyId
		jointDef.Base.BodyIdB = bodyId
		jointDef.Base.LocalFrameA.P = jointDef.Base.BodyIdA.GetLocalPoint(b2.ToPos(pivot))
		jointDef.Base.LocalFrameB.P = jointDef.Base.BodyIdB.GetLocalPoint(b2.ToPos(pivot))
		b2.CreateRevoluteJoint(worldId, &jointDef)

		prevBodyId = bodyId
	}

	pivot := b2.Vec2{X: xBase.Add(b2.F(count)), Y: yBase}
	jointDef.Base.BodyIdA = prevBodyId
	jointDef.Base.BodyIdB = groundId2
	jointDef.Base.LocalFrameA.P = jointDef.Base.BodyIdA.GetLocalPoint(b2.ToPos(pivot))
	jointDef.Base.LocalFrameB.P = jointDef.Base.BodyIdB.GetLocalPoint(b2.ToPos(pivot))
	b2.CreateRevoluteJoint(worldId, &jointDef)
}

// moverBalls drops five bouncing balls next to the start.
func moverBalls(worldId b2.WorldId) {
	for i := range 5 {
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.ToPos(b2.V2(7, 7+0.5*float64(i)))
		bodyId := b2.CreateBody(worldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		shapeDef.Filter = b2.Filter{CategoryBits: moverDynamicBit, MaskBits: moverAllBits}
		shapeDef.Material.Restitution = b2.F(0.7)
		shapeDef.Material.RollingResistance = b2.F(0.2)

		circle := b2.Circle{Radius: b2.QFromRatio(1, 4)}
		b2.CreateCircleShape(bodyId, &shapeDef, &circle)
	}
}

// moverElevator creates the kinematic elevator; userData may be nil.
func moverElevator(worldId b2.WorldId, userData *moverShapeUserData) b2.BodyId {
	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.KinematicBody
	bodyDef.Position = b2.ToPos(b2.Vec2{
		X: moverElevatorBase.X,
		Y: moverElevatorBase.Y.Sub(FromFloat64(moverElevatorAmplitude)),
	})
	elevatorId := b2.CreateBody(worldId, &bodyDef)

	shapeDef := b2.DefaultShapeDef()
	shapeDef.Filter = b2.Filter{CategoryBits: moverDynamicBit, MaskBits: moverAllBits}
	if userData != nil {
		shapeDef.UserData = userData
	}

	box := b2.MakeBox(b2.F(2), b2.QFromRatio(1, 10))
	b2.CreatePolygonShape(elevatorId, &shapeDef, &box)
	return elevatorId
}

// moverStack creates the tower of ten small boxes.
func moverStack(worldId b2.WorldId) {
	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	setPosX(&bodyDef.Position, b2.F(140))
	a := b2.QFromRatio(1, 4)
	square := b2.MakeSquare(a)
	shapeDef := b2.DefaultShapeDef()
	shapeDef.Filter = b2.Filter{CategoryBits: moverDynamicBit, MaskBits: moverAllBits}
	for i := range 10 {
		setPosY(&bodyDef.Position, b2.F(2*i).Add(b2.QOne()).Mul(a).Add(b2.QOne()))
		bodyId := b2.CreateBody(worldId, &bodyDef)
		b2.CreatePolygonShape(bodyId, &shapeDef, &square)
	}
}

// moverPole creates the motorized pole hinged on the second ground body.
func moverPole(worldId b2.WorldId, groundId2 b2.BodyId, y float64) {
	bodyDef := b2.DefaultBodyDef()
	bodyDef.Position = b2.ToPos(b2.V2(160, y))
	bodyDef.Type = b2.DynamicBody
	body := b2.CreateBody(worldId, &bodyDef)

	box := b2.MakeBox(b2.QFromRatio(1, 10), b2.F(4))
	shapeDef := b2.DefaultShapeDef()
	shapeDef.Density = b2.QOne()
	b2.CreatePolygonShape(body, &shapeDef, &box)

	pivot := b2.OffsetPos(bodyDef.Position, b2.V2(0, 4))
	jointDef := b2.DefaultRevoluteJointDef()
	jointDef.Base.BodyIdA = groundId2
	jointDef.Base.BodyIdB = body
	jointDef.Base.LocalFrameA.P = jointDef.Base.BodyIdA.GetLocalPoint(pivot)
	jointDef.Base.LocalFrameB.P = jointDef.Base.BodyIdB.GetLocalPoint(pivot)
	jointDef.EnableMotor = true
	jointDef.MotorSpeed = radiansToTurns(1.0)
	jointDef.MaxMotorTorque = b2.F(500)

	b2.CreateRevoluteJoint(worldId, &jointDef)
}

// moverTimeStep returns the frame time step and whether the sample is paused,
// as the Step overrides of the mover samples compute it.
func (b *Base) moverTimeStep() (float64, bool) {
	pause := false
	if b.Context.Settings.Pause {
		pause = !b.Context.Settings.SingleStep
	}

	timeStep := 0.0
	if b.Context.Settings.Hertz > 0 {
		timeStep = 1.0 / b.Context.Settings.Hertz
	}
	if pause {
		timeStep = 0.0
	}
	return timeStep, pause
}

// moverThrottle reads the A and D keys.
func (b *Base) moverThrottle() b2.Q {
	throttle := b2.QZero()
	if b.keyDown(KeyA) {
		throttle = throttle.Sub(b2.QOne())
	}
	if b.keyDown(KeyD) {
		throttle = throttle.Add(b2.QOne())
	}
	return throttle
}

// moverMoveElevator drives the kinematic elevator at the given time.
func moverMoveElevator(elevatorId b2.BodyId, time, timeStep float64) {
	point := b2.Vec2{
		X: moverElevatorBase.X,
		Y: FromFloat64(moverElevatorAmplitude * math.Cos(1.0*time+math.Pi)).Add(moverElevatorBase.Y),
	}

	elevatorId.SetTargetTransform(b2.MakeWorldTransform(b2.Transform{P: point, Q: b2.RotIdentity()}), FromFloat64(timeStep), true)
}

// GeometricMoverSample is the "Geometric Mover" sample of
// sample_character.cpp (GeometryMoverSample): a capsule moved by shape casts
// and plane solving rather than by the rigid body solver.
type GeometricMoverSample struct {
	Base

	mover         *geometricMover
	elevatorId    b2.BodyId
	friendlyShape moverShapeUserData
	elevatorShape moverShapeUserData
	time          float64
	jumpReleased  bool
	lockCamera    bool
}

// NewGeometricMoverSample builds the geometric mover scene.
func NewGeometricMoverSample(ctx *SampleContext) Sample {
	s := &GeometricMoverSample{Base: NewBase(ctx), mover: newGeometricMover()}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 20.0, Y: 9.0}
		ctx.Camera.Zoom = 10.0
	}

	ctx.Settings.DrawJoints = false

	moverDef := defaultGeometricMoverDef()
	moverDef.position = b2.V2(0, 8)
	moverDef.capsule = b2.Capsule{Center1: b2.V2(0, -0.5), Center2: b2.V2(0, 0.5), Radius: b2.F(0.3)}
	moverDef.filter = b2.QueryFilter{CategoryBits: moverMoverBit, MaskBits: moverStaticBit | moverDynamicBit | moverMoverBit}
	s.mover.create(s.WorldId, &moverDef)

	var groundId1 b2.BodyId
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Name = "svg"
		bodyDef.Position = b2.ToPos(b2.V2(0, 0))

		path := "M -34.395834,201.08333 H 293.68751 v -47.625 h -2.64584 l -10.58333,7.9375 -13.22916,7.9375 -13.24648,5.29167 " +
			"-31.73269,7.9375 -21.16667,2.64583 -23.8125,10.58333 -15.875,2e-5 v -3.96875 l -17.19792,2e-5 v 2.64582 l " +
			"-17.19791,0 v -1.32292 h -3.96875 v -1.32292 h -3.96875 v -1.32292 h -3.96875 v -1.32291 h -3.96875 v -1.32292 " +
			"h -3.96875 v -1.32292 h -3.96875 v -1.32291 l -3.968754,0 v -1.32292 h -3.96875 v -1.32292 h -3.968751 v " +
			"-1.32291 h -3.96875 v -1.32292 h -3.96875 v -1.32292 h -3.96875 v -1.32291 h -3.96875 v -1.32292 l -3.96875,0 v " +
			"-1.32292 h -3.968751 v -1.32291 h -3.96875 l -2e-6,-1.32294 H 52.916669 L 39.6875,177.27083 h -5.291667 l " +
			"-7.937499,5.29167 H 15.875001 l -47.625002,-50.27083 v -26.45834 h -2.645834 l 10e-7,95.25"

		groundId1 = moverChain(s.WorldId, &bodyDef, path, b2.V2(-50, -200))
	}

	var groundId2 b2.BodyId
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Position = b2.ToPos(b2.V2(98, 0))
		groundId2 = moverChain(s.WorldId, &bodyDef, moverGroundPath2, b2.V2(0, -200))
	}

	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Position = b2.ToPos(b2.V2(32.0, 4.5))

		shapeDef := b2.DefaultShapeDef()
		s.friendlyShape.maxPush = b2.F(0.025)
		s.friendlyShape.clipVelocity = false

		shapeDef.Filter = b2.Filter{CategoryBits: moverMoverBit, MaskBits: moverAllBits}
		shapeDef.UserData = &s.friendlyShape
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)
		b2.CreateCapsuleShape(bodyId, &shapeDef, &s.mover.capsule)
	}

	moverBridge(s.WorldId, groundId1, groundId2)
	moverBalls(s.WorldId)

	s.elevatorShape = moverShapeUserData{
		maxPush:      b2.QFromRatio(1, 10),
		clipVelocity: true,
	}
	s.elevatorId = moverElevator(s.WorldId, &s.elevatorShape)

	moverStack(s.WorldId)
	moverPole(s.WorldId, groundId2, 5)

	s.jumpReleased = true
	s.lockCamera = true
	s.time = 0.0

	return s
}

// UpdateGui provides the movement tuning.
func (s *GeometricMoverSample) UpdateGui() {
	height := 350
	gui := s.Context.Gui
	gui.Begin("Mover", 10, s.Context.Camera.Height-height-25, 340, height)

	m := s.mover
	gui.SliderFloat("Jump Speed", &m.jumpSpeed, 0.0, 40.0)
	gui.SliderFloat("Min Speed", &m.minSpeed, 0.0, 1.0)
	gui.SliderFloat("Max Speed", &m.maxSpeed, 0.0, 20.0)
	gui.SliderFloat("Stop Speed", &m.stopSpeed, 0.0, 10.0)
	gui.SliderFloat("Accelerate", &m.accelerate, 0.0, 100.0)
	gui.SliderFloat("Friction", &m.friction, 0.0, 10.0)
	gui.SliderFloat("Gravity", &m.gravity, 0.0, 100.0)
	gui.SliderFloat("Air Steer", &m.airSteer, 0.0, 1.0)
	gui.SliderFloat("Pogo Hertz", &m.pogoHertz, 0.0, 30.0)
	gui.SliderFloat("Pogo Damping", &m.pogoDampingRatio, 0.0, 4.0)

	gui.Checkbox("Push Bodies", &m.pushBodies)
	gui.Checkbox("Lock Camera", &s.lockCamera)

	gui.End()
}

// Step moves the elevator, advances the world, then moves the character
// with the A, D and space keys.
func (s *GeometricMoverSample) Step() {
	s.DrawTextLine("left/right/jump = A/D/Space")

	timeStep, pause := s.moverTimeStep()

	if timeStep > 0.0 {
		moverMoveElevator(s.elevatorId, s.time, timeStep)
	}

	s.time += timeStep

	s.Base.Step()

	m := s.mover
	if !pause {
		throttle := s.moverThrottle()

		if s.keyDown(KeySpace) {
			if s.jumpReleased && m.jump() {
				s.jumpReleased = false
			}
		} else {
			s.jumpReleased = true
		}

		m.update(FromFloat64(timeStep), throttle)
	}

	draw := &s.Context.Draw
	moverDrawPogo(draw, m.pogoOrigin, m.pogoTranslation, m.pogoFraction, m.pogoHit)

	// The transform axes at the mover position, 0.25 long.
	axis := b2.QFromRatio(1, 4)
	draw.DrawLineFcn(b2.ToPos(m.position), b2.ToPos(m.position.Add(b2.Vec2{X: axis})), b2.ColorRed)
	draw.DrawLineFcn(b2.ToPos(m.position), b2.ToPos(m.position.Add(b2.Vec2{Y: axis})), b2.ColorGreen)

	capsule := m.capsule

	for i := range m.planeCount {
		plane := m.planes[i].Plane
		p1 := b2.MulAdd(m.position, plane.Offset.Sub(capsule.Radius), plane.Normal)
		p2 := b2.MulAdd(p1, b2.QFromRatio(1, 10), plane.Normal)
		draw.DrawPointFcn(b2.ToPos(p1), b2.F(5), b2.ColorYellow)
		draw.DrawLineFcn(b2.ToPos(p1), b2.ToPos(p2), b2.ColorYellow)
	}

	p1 := m.position.Add(capsule.Center1)
	p2 := m.position.Add(capsule.Center2)

	color := b2.ColorAquamarine
	if m.onGround {
		color = b2.ColorOrange
	}
	draw.DrawSolidCapsuleFcn(b2.ToPos(p1), b2.ToPos(p2), capsule.Radius, color)
	draw.DrawLineFcn(b2.ToPos(m.position), b2.ToPos(m.position.Add(m.velocity)), b2.ColorPurple)

	p := m.position
	s.DrawTextLine("position %.2f %.2f", ToFloat64(p.X), ToFloat64(p.Y))
	s.DrawTextLine("velocity %.2f %.2f", ToFloat64(m.velocity.X), ToFloat64(m.velocity.Y))
	s.DrawTextLine("iterations %d", m.totalIterations)

	if s.lockCamera {
		s.Context.Camera.Center.X = ToFloat64(m.position.X)
	}
}

// DynamicMoverSample is the "Dynamic Mover" sample of sample_character.cpp
// (DynamicMoverSample): a capsule rigid body steered by a mover joint and
// held up by a pogo joint.
type DynamicMoverSample struct {
	Base

	mover        *dynamicMover
	elevatorId   b2.BodyId
	time         float64
	jumpReleased bool
	lockCamera   bool
}

// NewDynamicMoverSample builds the dynamic mover scene.
func NewDynamicMoverSample(ctx *SampleContext) Sample {
	s := &DynamicMoverSample{Base: NewBase(ctx), mover: newDynamicMover()}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 20.0, Y: 9.0}
		ctx.Camera.Zoom = 10.0
	}

	ctx.Settings.DrawJoints = false

	// This is the mover the player controls
	{
		def := defaultDynamicMoverDef()
		def.position = b2.V2(0, 8)
		def.capsule = b2.Capsule{Center1: b2.V2(0, -0.5), Center2: b2.V2(0, 0.5), Radius: b2.F(0.3)}
		def.filter = b2.Filter{CategoryBits: moverMoverBit, MaskBits: moverStaticBit | moverDynamicBit}
		def.enablePreSolveEvents = true
		s.mover.create(s.WorldId, &def)
	}

	var groundId1 b2.BodyId
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Name = "svg"
		bodyDef.Position = b2.ToPos(b2.V2(0, 0))

		path := "M -34.395834,201.08333 H 293.68751 v -47.625 h -2.64584 l -10.58333,7.9375 -13.22916,7.9375 -13.24648,5.29167 " +
			"-31.73269,7.9375 -21.16667,2.64583 -23.8125,13.22919 -15.875,2e-5 0,-6.61461 -17.19792,2e-5 v 2.64582 h " +
			"-17.19791 v -1.32292 h -3.96875 v -1.32292 h -3.96875 v -1.32292 h -3.96875 v -1.32291 h -3.96875 v -1.32292 h " +
			"-3.96875 v -1.32292 h -3.96875 v -1.32291 h -3.968754 v -1.32292 h -3.96875 v -1.32292 h -3.968751 v -1.32291 h " +
			"-3.96875 v -1.32292 h -3.96875 v -1.32292 h -3.96875 v -1.32291 h -3.96875 v -1.32292 h -3.96875 v -1.32292 h " +
			"-3.968751 v -1.32291 h -3.96875 l -2e-6,-1.32294 H 52.916669 L 39.6875,177.27083 h -5.291667 l " +
			"-7.937499,5.29167 H 15.875001 l -47.625002,-50.27083 v -26.45834 h -2.645834 l 10e-7,95.25"

		groundId1 = moverChain(s.WorldId, &bodyDef, path, b2.V2(-50, -200))
	}

	var groundId2 b2.BodyId
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Position = b2.ToPos(b2.V2(98, 0))
		groundId2 = moverChain(s.WorldId, &bodyDef, moverGroundPath2, b2.V2(0, -200))
	}

	moverBridge(s.WorldId, groundId1, groundId2)
	moverBalls(s.WorldId)
	s.elevatorId = moverElevator(s.WorldId, nil)
	moverStack(s.WorldId)
	moverPole(s.WorldId, groundId2, 4)

	// This is used for one-way collision on the kinematic elevator.
	s.WorldId.SetPreSolveCallback(s.preSolve, s.preContinuous)

	s.jumpReleased = true
	s.lockCamera = true
	s.time = 0.0

	return s
}

// preSolve turns the elevator into a one-way platform for the mover.
func (s *DynamicMoverSample) preSolve(shapeIdA, shapeIdB b2.ShapeId, manifold *b2.Manifold) {
	idA := shapeIdA.GetBody()
	idB := shapeIdB.GetBody()

	if idA == s.elevatorId && idB == s.mover.moverId {
		if manifold.Normal.Y.Less(b2.QZero()) {
			manifold.PointCount = 0
		}
	} else if idA == s.mover.moverId && idB == s.elevatorId {
		if manifold.Normal.Y.Greater(b2.QZero()) {
			manifold.PointCount = 0
		}
	}
}

// preContinuous is the one-way platform rule for the continuous stage.
func (s *DynamicMoverSample) preContinuous(shapeIdA, shapeIdB b2.ShapeId, point b2.Pos, normal b2.Vec2) bool {
	idA := shapeIdA.GetBody()
	idB := shapeIdB.GetBody()

	if idA == s.elevatorId && idB == s.mover.moverId {
		return !normal.Y.Less(b2.QZero())
	}

	if idA == s.mover.moverId && idB == s.elevatorId {
		return !normal.Y.Greater(b2.QZero())
	}

	return true
}

// UpdateGui provides the movement tuning.
func (s *DynamicMoverSample) UpdateGui() {
	height := 350
	gui := s.Context.Gui
	gui.Begin("Mover", 10, s.Context.Camera.Height-height-25, 340, height)

	m := s.mover
	gui.SliderFloat("Jump Speed", &m.jumpSpeed, 0.0, 40.0)
	gui.SliderFloat("Min Speed", &m.minSpeed, 0.0, 1.0)
	gui.SliderFloat("Max Speed", &m.maxSpeed, 0.0, 20.0)
	gui.SliderFloat("Stop Speed", &m.stopSpeed, 0.0, 10.0)
	gui.SliderFloat("Accelerate", &m.accelerate, 0.0, 100.0)
	gui.SliderFloat("Friction", &m.friction, 0.0, 10.0)

	gravityScale := m.gravityScale
	if gui.SliderFloat("Gravity Scale", &gravityScale, 0.0, 4.0) {
		m.setGravityScale(gravityScale)
	}

	gui.SliderFloat("Air Steer", &m.airSteer, 0.0, 1.0)
	gui.SliderFloat("Pogo Hertz", &m.pogoHertz, 0.0, 30.0)
	gui.SliderFloat("Pogo Damping", &m.pogoDampingRatio, 0.0, 4.0)

	gui.Checkbox("Lock Camera", &s.lockCamera)

	gui.End()
}

// Step moves the elevator, updates the mover, then advances the world.
func (s *DynamicMoverSample) Step() {
	s.DrawTextLine("left/right/jump = A/D/Space")

	timeStep, pause := s.moverTimeStep()

	if timeStep > 0.0 && !s.elevatorId.IsNull() {
		moverMoveElevator(s.elevatorId, s.time, timeStep)
	}

	s.time += timeStep

	m := s.mover
	if !pause {
		throttle := s.moverThrottle()

		if s.keyDown(KeySpace) {
			if s.jumpReleased && m.jump() {
				s.jumpReleased = false
			}
		} else {
			s.jumpReleased = true
		}

		m.update(FromFloat64(timeStep), throttle)
	}

	s.Base.Step()

	moverDrawPogo(&s.Context.Draw, m.pogoOrigin, m.pogoTranslation, m.pogoFraction, m.pogoHit)

	position := m.moverId.GetPosition()

	color := b2.ColorAquamarine
	if m.onGround {
		color = b2.ColorOrange
	}
	s.Context.Draw.DrawLineFcn(position, b2.OffsetPos(position, m.velocity), color)

	s.DrawTextLine("position %.2f %.2f", ToFloat64(b2.ToVec2(position).X), ToFloat64(b2.ToVec2(position).Y))
	s.DrawTextLine("velocity %.2f %.2f", ToFloat64(m.velocity.X), ToFloat64(m.velocity.Y))
	s.DrawTextLine("pogo: impulse %.3f, velocity %.3f", ToFloat64(m.pogoImpulse), ToFloat64(m.pogoVelocity))
	s.DrawTextLine("pogo: length = %.2f/%.2f", ToFloat64(m.pogoLength), m.pogoRestLength)
	s.DrawTextLine("on ground: %d", characterBoolInt(m.onGround))
	normalY := b2.QZero()
	if m.castResult.hit {
		normalY = m.castResult.normal.Y
	}
	s.DrawTextLine("walkable: %d, normal y: %.3f/%.3f", characterBoolInt(m.walkable), ToFloat64(normalY), m.minGroundNormalY)
	s.DrawTextLine("jumping/ticks: %d/%d", characterBoolInt(m.jumping), m.jumpTicks)

	if s.lockCamera {
		s.Context.Camera.Center.X = ToFloat64(b2.ToVec2(position).X)
	}
}

// characterBoolToInt is the C++ (int)bool cast.
func characterBoolInt(b bool) int {
	if b {
		return 1
	}
	return 0
}

// ghostCullingFloorMaterial marks the horizontal floor tiles.
const ghostCullingFloorMaterial uint64 = 42

// GhostCulling is the "Ghost Culling" sample of sample_character.cpp. It
// shows how to mitigate ghost collisions using the presolve callback, based
// on https://briansemrau.github.io/dealing-with-ghost-collisions/
type GhostCulling struct {
	Base

	playerIds     [2]b2.BodyId
	rejectedCount atomic.Int32
	speed         float64
}

// NewGhostCulling builds the ghost culling scene.
func NewGhostCulling(ctx *SampleContext) Sample {
	s := &GhostCulling{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0.0, Y: 1.5}
		ctx.Camera.Zoom = 12.0
	}

	groundBodyDef := b2.DefaultBodyDef()
	groundId := b2.CreateBody(s.WorldId, &groundBodyDef)

	surfaceHeights := [2]float64{3.0, 0.0}
	for _, surfaceHeight := range surfaceHeights {
		for i := range 10 {
			x := -9.0 + 2.0*float64(i)
			tile := b2.MakeOffsetBox(b2.QOne(), b2.QFromRatio(1, 4), b2.V2(x, surfaceHeight-0.25), b2.RotIdentity())

			shapeDef := b2.DefaultShapeDef()
			shapeDef.Material.Friction = b2.QZero()

			// Flag the horizontal tiles with a special material.
			shapeDef.Material.UserMaterialId = ghostCullingFloorMaterial

			if i%2 == 0 {
				shapeDef.Material.CustomColor = uint32(b2.ColorGray)
			} else {
				shapeDef.Material.CustomColor = uint32(b2.ColorDarkGray)
			}

			b2.CreatePolygonShape(groundId, &shapeDef, &tile)
		}

		wall := b2.MakeOffsetBox(b2.QFromRatio(1, 4), b2.F(1.25), b2.V2(10.25, surfaceHeight+1.25), b2.RotIdentity())
		shapeDef := b2.DefaultShapeDef()
		shapeDef.Material.Friction = b2.QZero()
		shapeDef.Material.CustomColor = uint32(b2.ColorDarkGray)
		b2.CreatePolygonShape(groundId, &shapeDef, &wall)
	}

	s.speed = 20.0
	s.playerIds[0] = b2.BodyId{}
	s.playerIds[1] = b2.BodyId{}
	s.rejectedCount.Store(0)

	s.WorldId.SetPreSolveCallback(s.preSolve, s.preContinuous)
	s.launch()

	return s
}

// launch (re)creates the two players and sends them along the floors.
func (s *GhostCulling) launch() {
	s.rejectedCount.Store(0)

	surfaceHeights := [2]float64{3.0, 0.0}
	for i := range 2 {
		if !s.playerIds[i].IsNull() {
			b2.DestroyBody(s.playerIds[i])
		}

		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.ToPos(b2.V2(-9.0, surfaceHeights[i]+0.5))
		bodyDef.LinearVelocity = b2.Vec2{X: FromFloat64(s.speed)}
		bodyDef.MotionLocks.AngularZ = true

		// Pre-solve must see a fresh manifold at every tile boundary.
		bodyDef.EnableContactRecycling = false

		s.playerIds[i] = b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		shapeDef.Density = b2.QOne()
		shapeDef.Material.Friction = b2.QZero()
		shapeDef.EnablePreSolveEvents = i == 1

		player := b2.MakeBox(b2.QHalf(), b2.QHalf())
		b2.CreatePolygonShape(s.playerIds[i], &shapeDef, &player)
	}
}

// preSolve rejects shallow face contacts at tile seams.
func (s *GhostCulling) preSolve(shapeIdA, shapeIdB b2.ShapeId, manifold *b2.Manifold) {
	if manifold.PointCount != 2 {
		return
	}

	normal := manifold.Normal
	bodyIdA := shapeIdA.GetBody()
	if bodyIdA != s.playerIds[1] {
		bodyIdB := shapeIdB.GetBody()
		if bodyIdB != s.playerIds[1] {
			return
		}

		normal = b2.Neg(normal)
	}

	velocity := s.playerIds[1].GetLinearVelocity()
	if normal.Dot(velocity).Less(b2.QZero()) {
		return
	}

	// A short face contact is overlap slop at a tile seam, not a wall.
	contactArea := manifold.Points[0].AnchorA.Distance(manifold.Points[1].AnchorA)
	if contactArea.Less(b2.F(4).Mul(b2.LinearSlop())) {
		manifold.PointCount = 0

		// PreSolve is called from a worker and must not race.
		s.rejectedCount.Add(1)
	}
}

// preContinuous disables horizontal continuous collision between the player
// and the floor tiles. Continuous collision in Box2D doesn't affect velocity,
// but it does cause time loss, which may be perceived as stuttering or
// non-smooth movement.
//
// CCD doesn't get the full manifold so the contact area cannot be used.
// Instead the CCD event is disabled if the normal is horizontal and the other
// shape is a horizontal tile, identified by material id.
func (s *GhostCulling) preContinuous(shapeIdA, shapeIdB b2.ShapeId, point b2.Pos, normal b2.Vec2) bool {
	// Only filter horizontal hits.
	if normal.Y.Abs().Greater(b2.QFromRatio(1, 10)) {
		return true
	}

	idA := shapeIdA.GetBody()

	// Determine if this is player[1] and determine the other shape.
	var floorShapeId b2.ShapeId
	if idA == s.playerIds[1] {
		floorShapeId = shapeIdB
	} else {
		idB := shapeIdB.GetBody()
		if idB == s.playerIds[1] {
			floorShapeId = shapeIdA
		} else {
			return true
		}
	}

	// A floor tile does not take part in CCD, anything else does.
	return floorShapeId.GetUserMaterial() != ghostCullingFloorMaterial
}

// UpdateGui provides the launch speed and the launch button.
func (s *GhostCulling) UpdateGui() {
	height := 90
	gui := s.Context.Gui
	gui.Begin("Ghost Culling", 10, s.Context.Camera.Height-height-25, 200, height)

	gui.SliderFloat("Speed", &s.speed, 1.0, 100.0)

	if gui.Button("Launch") {
		s.launch()
	}

	gui.End()
}

// Step advances the world and labels the two floors.
func (s *GhostCulling) Step() {
	s.Base.Step()

	draw := &s.Context.Draw
	draw.DrawStringFcn(b2.ToPos(b2.V2(-9.8, 4.2)), "unfiltered", b2.ColorWhite)
	draw.DrawStringFcn(b2.ToPos(b2.V2(-9.8, 1.2)), "shallow contacts rejected", b2.ColorWhite)
	s.DrawTextLine("Rejected contacts: %d", s.rejectedCount.Load())

	for i := range 2 {
		v := s.playerIds[i].GetLinearVelocity()
		s.DrawTextLine("player %d velocity.x = %.3f", i, ToFloat64(v.X))
	}
}
