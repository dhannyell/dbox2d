// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/sample_character.cpp of Box2D v3.1.1. Only the first
// Mover is ported; the second sits in an #if 0 block upstream.

package samples

import (
	"math"

	"github.com/dhannyell/dbox2d"
)

func init() {
	RegisterSample("Character", "Mover", NewMover)
}

// moverShapeUserData tunes how a shape pushes the mover.
type moverShapeUserData struct {
	maxPush      b2.Q
	clipVelocity bool
}

// Collision bits of the mover scene.
const (
	moverStaticBit  uint64 = 0x0001
	moverMoverBit   uint64 = 0x0002
	moverDynamicBit uint64 = 0x0004
	moverDebrisBit  uint64 = 0x0008

	// moverAllBits is the reference's ~0u, a 32-bit mask.
	moverAllBits uint64 = 0xFFFFFFFF
)

const (
	pogoPoint = iota
	pogoCircle
	pogoSegment
)

type moverCastResult struct {
	point    b2.Vec2
	normal   b2.Vec2
	bodyId   b2.BodyId
	fraction b2.Q
	hit      bool
}

const moverPlaneCapacity = 8

var (
	moverElevatorBase      = b2.V2(112, 10)
	moverElevatorAmplitude = 4.0
)

// Mover is a kinematic character controller: a capsule moved by plane
// solving and shape casts rather than by the rigid body solver.
type Mover struct {
	Base

	jumpSpeed        float64
	maxSpeed         float64
	minSpeed         float64
	stopSpeed        float64
	accelerate       float64
	airSteer         float64
	friction         float64
	gravity          float64
	pogoHertz        float64
	pogoDampingRatio float64

	pogoShape       int
	transform       b2.Transform
	velocity        b2.Vec2
	capsule         b2.Capsule
	elevatorId      b2.BodyId
	ballId          b2.ShapeId
	friendlyShape   moverShapeUserData
	elevatorShape   moverShapeUserData
	planes          [moverPlaneCapacity]b2.CollisionPlane
	planeCount      int
	totalIterations int
	pogoVelocity    b2.Q
	time            float64
	onGround        bool
	jumpReleased    bool
	lockCamera      bool
}

// NewMover builds the mover scene.
func NewMover(ctx *SampleContext) Sample {
	s := &Mover{
		Base:             NewBase(ctx),
		jumpSpeed:        10.0,
		maxSpeed:         6.0,
		minSpeed:         0.1,
		stopSpeed:        3.0,
		accelerate:       20.0,
		airSteer:         0.2,
		friction:         8.0,
		gravity:          30.0,
		pogoHertz:        5.0,
		pogoDampingRatio: 0.8,
		pogoShape:        pogoSegment,
	}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 20.0, Y: 9.0}
		ctx.Camera.Zoom = 10.0
	}

	ctx.Settings.DrawJoints = false
	s.transform = b2.Transform{P: b2.V2(2, 8), Q: b2.RotIdentity()}
	s.velocity = b2.Vec2{}
	s.capsule = b2.Capsule{
		Center1: b2.Vec2{Y: b2.QHalf().Neg()},
		Center2: b2.Vec2{Y: b2.QHalf()},
		Radius:  b2.F(0.3),
	}

	var groundId1 b2.BodyId
	{
		bodyDef := b2.DefaultBodyDef()
		groundId1 = b2.CreateBody(s.WorldId, &bodyDef)

		path := "M 2.6458333,201.08333 H 293.68751 v -47.625 h -2.64584 l -10.58333,7.9375 -13.22916,7.9375 -13.24648,5.29167 " +
			"-31.73269,7.9375 -21.16667,2.64583 -23.8125,10.58333 H 142.875 v -5.29167 h -5.29166 v 5.29167 H 119.0625 v " +
			"-2.64583 h -2.64583 v -2.64584 h -2.64584 v -2.64583 H 111.125 v -2.64583 H 84.666668 v -2.64583 h -5.291666 v " +
			"-2.64584 h -5.291667 v -2.64583 H 68.791668 V 174.625 h -5.291666 v -2.64584 H 52.916669 L 39.6875,177.27083 H " +
			"34.395833 L 23.8125,185.20833 H 15.875 L 5.2916669,187.85416 V 153.45833 H 2.6458333 v 47.625"

		offset := b2.V2(-50, -200)
		points := parsePath(path, offset, 64, b2.QFromRatio(1, 5))

		chainDef := b2.DefaultChainDef()
		chainDef.Points = points
		chainDef.IsLoop = true
		b2.CreateChain(groundId1, &chainDef)
	}

	var groundId2 b2.BodyId
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Position = b2.V2(98, 0)
		groundId2 = b2.CreateBody(s.WorldId, &bodyDef)

		path := "M 2.6458333,201.08333 H 293.68751 l 0,-23.8125 h -23.8125 l 21.16667,21.16667 h -23.8125 l -39.68751,-13.22917 " +
			"-26.45833,7.9375 -23.8125,2.64583 h -13.22917 l -0.0575,2.64584 h -5.29166 v -2.64583 l -7.86855,-1e-5 " +
			"-0.0114,-2.64583 h -2.64583 l -2.64583,2.64584 h -7.9375 l -2.64584,2.64583 -2.58891,-2.64584 h -13.28609 v " +
			"-2.64583 h -2.64583 v -2.64584 l -5.29167,1e-5 v -2.64583 h -2.64583 v -2.64583 l -5.29167,-1e-5 v -2.64583 h " +
			"-2.64583 v -2.64584 h -5.291667 v -2.64583 H 92.60417 V 174.625 h -5.291667 v -2.64584 l -34.395835,1e-5 " +
			"-7.9375,-2.64584 -7.9375,-2.64583 -5.291667,-5.29167 H 21.166667 L 13.229167,158.75 5.2916668,153.45833 H " +
			"2.6458334 l -10e-8,47.625"

		offset := b2.V2(0, -200)
		points := parsePath(path, offset, 64, b2.QFromRatio(1, 5))

		chainDef := b2.DefaultChainDef()
		chainDef.Points = points
		chainDef.IsLoop = true
		b2.CreateChain(groundId2, &chainDef)
	}

	{
		box := b2.MakeBox(b2.QHalf(), b2.QFromRatio(1, 8))

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
			bodyDef.Position = b2.Vec2{X: xBase.Add(b2.QHalf()).Add(b2.F(i)), Y: yBase}
			bodyDef.AngularDamping = b2.F(0.2)
			bodyId := b2.CreateBody(s.WorldId, &bodyDef)
			b2.CreatePolygonShape(bodyId, &shapeDef, &box)

			pivot := b2.Vec2{X: xBase.Add(b2.F(i)), Y: yBase}
			jointDef.BodyIdA = prevBodyId
			jointDef.BodyIdB = bodyId
			jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
			jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
			b2.CreateRevoluteJoint(s.WorldId, &jointDef)

			prevBodyId = bodyId
		}

		pivot := b2.Vec2{X: xBase.Add(b2.F(count)), Y: yBase}
		jointDef.BodyIdA = prevBodyId
		jointDef.BodyIdB = groundId2
		jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
		jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
		b2.CreateRevoluteJoint(s.WorldId, &jointDef)
	}

	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Position = b2.V2(32.0, 4.5)

		shapeDef := b2.DefaultShapeDef()
		s.friendlyShape.maxPush = b2.F(0.025)
		s.friendlyShape.clipVelocity = false

		shapeDef.Filter = b2.Filter{CategoryBits: moverMoverBit, MaskBits: moverAllBits}
		shapeDef.UserData = &s.friendlyShape
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)
		b2.CreateCapsuleShape(bodyId, &shapeDef, &s.capsule)
	}

	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.V2(7, 7)
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		shapeDef.Filter = b2.Filter{CategoryBits: moverDebrisBit, MaskBits: moverAllBits}
		shapeDef.Material.Restitution = b2.F(0.7)
		shapeDef.Material.RollingResistance = b2.F(0.2)

		circle := b2.Circle{Radius: b2.F(0.3)}
		s.ballId = b2.CreateCircleShape(bodyId, &shapeDef, &circle)
	}

	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.KinematicBody
		bodyDef.Position = b2.Vec2{
			X: moverElevatorBase.X,
			Y: moverElevatorBase.Y.Sub(FromFloat64(moverElevatorAmplitude)),
		}
		s.elevatorId = b2.CreateBody(s.WorldId, &bodyDef)

		s.elevatorShape = moverShapeUserData{
			maxPush:      b2.QFromRatio(1, 10),
			clipVelocity: true,
		}
		shapeDef := b2.DefaultShapeDef()
		shapeDef.Filter = b2.Filter{CategoryBits: moverDynamicBit, MaskBits: moverAllBits}
		shapeDef.UserData = &s.elevatorShape

		box := b2.MakeBox(b2.F(2), b2.QFromRatio(1, 10))
		b2.CreatePolygonShape(s.elevatorId, &shapeDef, &box)
	}

	s.totalIterations = 0
	s.pogoVelocity = b2.QZero()
	s.onGround = false
	s.jumpReleased = true
	s.lockCamera = true
	s.planeCount = 0
	s.time = 0.0

	return s
}

// solveMove moves the capsule for one step.
// https://github.com/id-Software/Quake/blob/master/QW/client/pmove.c#L390
func (s *Mover) solveMove(timeStep, throttle b2.Q) {
	zero := b2.QZero()
	one := b2.QOne()
	minSpeed := FromFloat64(s.minSpeed)
	maxSpeed := FromFloat64(s.maxSpeed)
	stopSpeed := FromFloat64(s.stopSpeed)
	draw := &s.Context.Draw

	// Friction
	speed, _ := b2.GetLengthAndNormalize(s.velocity)
	if speed.Less(minSpeed) {
		s.velocity.X = zero
		s.velocity.Y = zero
	} else if s.onGround {
		// Linear damping above stopSpeed and fixed reduction below stopSpeed
		control := speed
		if speed.Less(stopSpeed) {
			control = stopSpeed
		}

		// friction has units of 1/time
		drop := control.Mul(FromFloat64(s.friction)).Mul(timeStep)
		newSpeed := speed.Sub(drop)
		if newSpeed.Less(zero) {
			newSpeed = zero
		}
		s.velocity = s.velocity.Mul(newSpeed.Div(speed))
	}

	desiredVelocity := b2.Vec2{X: maxSpeed.Mul(throttle)}
	desiredSpeed, desiredDirection := b2.GetLengthAndNormalize(desiredVelocity)

	if desiredSpeed.Greater(maxSpeed) {
		desiredSpeed = maxSpeed
	}

	if s.onGround {
		s.velocity.Y = zero
	}

	// Accelerate
	currentSpeed := s.velocity.Dot(desiredDirection)
	addSpeed := desiredSpeed.Sub(currentSpeed)
	if addSpeed.Greater(zero) {
		steer := one
		if !s.onGround {
			steer = FromFloat64(s.airSteer)
		}
		accelSpeed := steer.Mul(FromFloat64(s.accelerate)).Mul(maxSpeed).Mul(timeStep)
		if accelSpeed.Greater(addSpeed) {
			accelSpeed = addSpeed
		}

		s.velocity = b2.MulAdd(s.velocity, accelSpeed, desiredDirection)
	}

	s.velocity.Y = s.velocity.Y.Sub(FromFloat64(s.gravity).Mul(timeStep))

	radius := s.capsule.Radius
	pogoRestLength := b2.F(3).Mul(radius)
	rayLength := pogoRestLength.Add(radius)
	origin := b2.TransformPoint(s.transform, s.capsule.Center1)
	circle := b2.Circle{Center: origin, Radius: b2.QHalf().Mul(radius)}
	segmentOffset := b2.Vec2{X: b2.QFromRatio(3, 4).Mul(radius)}
	segment := b2.Segment{
		Point1: origin.Sub(segmentOffset),
		Point2: origin.Add(segmentOffset),
	}

	var proxy b2.ShapeProxy
	var translation b2.Vec2
	pogoFilter := b2.QueryFilter{CategoryBits: moverMoverBit, MaskBits: moverStaticBit | moverDynamicBit}
	var castResult moverCastResult

	switch s.pogoShape {
	case pogoPoint:
		proxy = b2.MakeProxy([]b2.Vec2{origin}, zero)
		translation = b2.Vec2{Y: rayLength.Neg()}
	case pogoCircle:
		proxy = b2.MakeProxy([]b2.Vec2{origin}, circle.Radius)
		translation = b2.Vec2{Y: rayLength.Neg().Add(circle.Radius)}
	default:
		proxy = b2.MakeProxy([]b2.Vec2{segment.Point1, segment.Point2}, zero)
		translation = b2.Vec2{Y: rayLength.Neg()}
	}

	s.WorldId.CastShape(&proxy, translation, pogoFilter, func(shapeId b2.ShapeId, point, normal b2.Vec2, fraction b2.Q) b2.Q {
		castResult.point = point
		castResult.normal = normal
		castResult.bodyId = shapeId.GetBody()
		castResult.fraction = fraction
		castResult.hit = true
		return fraction
	})

	// Avoid snapping to ground if still going up
	if !s.onGround {
		s.onGround = castResult.hit && !s.velocity.Y.Greater(b2.QFromRatio(1, 100))
	} else {
		s.onGround = castResult.hit
	}

	if !castResult.hit {
		s.pogoVelocity = zero

		delta := translation
		draw.DrawSegment(origin, origin.Add(delta), b2.ColorGray)

		switch s.pogoShape {
		case pogoPoint:
			draw.DrawPoint(origin.Add(delta), b2.F(10), b2.ColorGray)
		case pogoCircle:
			draw.DrawCircle(origin.Add(delta), circle.Radius, b2.ColorGray)
		default:
			draw.DrawSegment(segment.Point1.Add(delta), segment.Point2.Add(delta), b2.ColorGray)
		}
	} else {
		pogoCurrentLength := castResult.fraction.Mul(rayLength)

		offset := pogoCurrentLength.Sub(pogoRestLength)
		s.pogoVelocity = b2.SpringDamper(FromFloat64(s.pogoHertz), FromFloat64(s.pogoDampingRatio), offset, s.pogoVelocity, timeStep)

		delta := translation.Mul(castResult.fraction)
		draw.DrawSegment(origin, origin.Add(delta), b2.ColorGray)

		switch s.pogoShape {
		case pogoPoint:
			draw.DrawPoint(origin.Add(delta), b2.F(10), b2.ColorPlum)
		case pogoCircle:
			draw.DrawCircle(origin.Add(delta), circle.Radius, b2.ColorPlum)
		default:
			draw.DrawSegment(segment.Point1.Add(delta), segment.Point2.Add(delta), b2.ColorPlum)
		}

		castResult.bodyId.ApplyForce(b2.V2(0, -50), castResult.point, true)
	}

	target := s.transform.P.Add(s.velocity.Mul(timeStep)).Add(b2.Vec2{Y: timeStep.Mul(s.pogoVelocity)})

	// Mover overlap filter
	collideFilter := b2.QueryFilter{CategoryBits: moverMoverBit, MaskBits: moverStaticBit | moverDynamicBit | moverMoverBit}

	// Movers don't sweep against other movers, allows for soft collision
	castFilter := b2.QueryFilter{CategoryBits: moverMoverBit, MaskBits: moverStaticBit | moverDynamicBit}

	s.totalIterations = 0
	tolerance := b2.QFromRatio(1, 100)

	for range 5 {
		s.planeCount = 0

		mover := b2.Capsule{
			Center1: b2.TransformPoint(s.transform, s.capsule.Center1),
			Center2: b2.TransformPoint(s.transform, s.capsule.Center2),
			Radius:  s.capsule.Radius,
		}

		s.WorldId.CollideMover(&mover, collideFilter, s.planeResult)
		result := b2.SolvePlanes(target.Sub(s.transform.P), s.planes[:s.planeCount])

		s.totalIterations += result.IterationCount

		fraction := s.WorldId.CastMover(&mover, result.Translation, castFilter)

		delta := result.Translation.Mul(fraction)
		s.transform.P = s.transform.P.Add(delta)

		if delta.Dot(delta).Less(tolerance.Mul(tolerance)) {
			break
		}
	}

	s.velocity = b2.ClipVector(s.velocity, s.planes[:s.planeCount])
}

// UpdateGui provides the movement tuning and the pogo shape.
func (s *Mover) UpdateGui() {
	height := 350
	gui := s.Context.Gui
	gui.Begin("Mover", 10, s.Context.Camera.Height-height-25, 340, height)

	gui.SliderFloat("Jump Speed", &s.jumpSpeed, 0.0, 40.0)
	gui.SliderFloat("Min Speed", &s.minSpeed, 0.0, 1.0)
	gui.SliderFloat("Max Speed", &s.maxSpeed, 0.0, 20.0)
	gui.SliderFloat("Stop Speed", &s.stopSpeed, 0.0, 10.0)
	gui.SliderFloat("Accelerate", &s.accelerate, 0.0, 100.0)
	gui.SliderFloat("Friction", &s.friction, 0.0, 10.0)
	gui.SliderFloat("Gravity", &s.gravity, 0.0, 100.0)
	gui.SliderFloat("Air Steer", &s.airSteer, 0.0, 1.0)
	gui.SliderFloat("Pogo Hertz", &s.pogoHertz, 0.0, 30.0)
	gui.SliderFloat("Pogo Damping", &s.pogoDampingRatio, 0.0, 4.0)

	gui.Text("Pogo Shape")
	if gui.RadioButton("Point", s.pogoShape == pogoPoint) {
		s.pogoShape = pogoPoint
	}
	if gui.RadioButton("Circle", s.pogoShape == pogoCircle) {
		s.pogoShape = pogoCircle
	}
	if gui.RadioButton("Segment", s.pogoShape == pogoSegment) {
		s.pogoShape = pogoSegment
	}

	gui.Checkbox("Lock Camera", &s.lockCamera)

	gui.End()
}

// planeResult collects the collision planes of the mover, with the push
// tuning of the shape it hits.
func (s *Mover) planeResult(shapeId b2.ShapeId, planeResult *b2.PlaneResult) bool {
	maxPush := b2.Huge
	clipVelocity := true
	if userData, ok := shapeId.GetUserData().(*moverShapeUserData); ok && userData != nil {
		maxPush = userData.maxPush
		clipVelocity = userData.clipVelocity
	}

	if s.planeCount < moverPlaneCapacity {
		s.planes[s.planeCount] = b2.CollisionPlane{
			Plane:        planeResult.Plane,
			PushLimit:    maxPush,
			Push:         b2.QZero(),
			ClipVelocity: clipVelocity,
		}
		s.planeCount++
	}

	return true
}

// kick pushes a dynamic body away from the mover and up.
func (s *Mover) kick(shapeId b2.ShapeId) bool {
	bodyId := shapeId.GetBody()
	if bodyId.GetType() != b2.DynamicBody {
		return true
	}

	center := bodyId.GetWorldCenterOfMass()
	_, direction := b2.GetLengthAndNormalize(center.Sub(s.transform.P))
	impulse := b2.Vec2{X: b2.F(2).Mul(direction.X), Y: b2.F(2)}
	bodyId.ApplyLinearImpulseToCenter(impulse, true)

	return true
}

// Keyboard kicks the debris near the mover's feet with K.
func (s *Mover) Keyboard(key Key) {
	if key == KeyK {
		point := b2.TransformPoint(s.transform, b2.Vec2{Y: s.capsule.Center1.Y.Sub(b2.F(3).Mul(s.capsule.Radius))})
		circle := b2.Circle{Center: point, Radius: b2.QHalf()}
		proxy := b2.MakeProxy([]b2.Vec2{circle.Center}, circle.Radius)
		filter := b2.QueryFilter{CategoryBits: moverMoverBit, MaskBits: moverDebrisBit}
		s.WorldId.OverlapShape(&proxy, filter, s.kick)
		s.Context.Draw.DrawCircle(circle.Center, circle.Radius, b2.ColorGoldenRod)
	}

	s.Base.Keyboard(key)
}

// Step moves the elevator, advances the world, then moves the character
// with the A, D and space keys.
func (s *Mover) Step() {
	pause := false
	if s.Context.Settings.Pause {
		pause = !s.Context.Settings.SingleStep
	}

	timeStep := 0.0
	if s.Context.Settings.Hertz > 0 {
		timeStep = 1.0 / s.Context.Settings.Hertz
	}
	if pause {
		timeStep = 0.0
	}

	if timeStep > 0.0 {
		point := b2.Vec2{
			X: moverElevatorBase.X,
			Y: FromFloat64(moverElevatorAmplitude * math.Cos(1.0*s.time+math.Pi)).Add(moverElevatorBase.Y),
		}

		s.elevatorId.SetTargetTransform(b2.Transform{P: point, Q: b2.RotIdentity()}, FromFloat64(timeStep))
	}

	s.time += timeStep

	s.Base.Step()

	if !pause {
		throttle := b2.QZero()

		if s.keyDown(KeyA) {
			throttle = throttle.Sub(b2.QOne())
		}

		if s.keyDown(KeyD) {
			throttle = throttle.Add(b2.QOne())
		}

		if s.keyDown(KeySpace) {
			if s.onGround && s.jumpReleased {
				s.velocity.Y = FromFloat64(s.jumpSpeed)
				s.onGround = false
				s.jumpReleased = false
			}
		} else {
			s.jumpReleased = true
		}

		s.solveMove(FromFloat64(timeStep), throttle)
	}

	draw := &s.Context.Draw
	radius := s.capsule.Radius
	for i := range s.planeCount {
		plane := s.planes[i].Plane
		p1 := b2.MulAdd(s.transform.P, plane.Offset.Sub(radius), plane.Normal)
		p2 := b2.MulAdd(p1, b2.QFromRatio(1, 10), plane.Normal)
		draw.DrawPoint(p1, b2.F(5), b2.ColorYellow)
		draw.DrawSegment(p1, p2, b2.ColorYellow)
	}

	p1 := b2.TransformPoint(s.transform, s.capsule.Center1)
	p2 := b2.TransformPoint(s.transform, s.capsule.Center2)

	color := b2.ColorAquamarine
	if s.onGround {
		color = b2.ColorOrange
	}
	draw.DrawSolidCapsule(p1, p2, radius, color)
	draw.DrawSegment(s.transform.P, s.transform.P.Add(s.velocity), b2.ColorPurple)

	p := s.transform.P
	s.DrawTextLine("position %.2f %.2f", ToFloat64(p.X), ToFloat64(p.Y))
	s.DrawTextLine("velocity %.2f %.2f", ToFloat64(s.velocity.X), ToFloat64(s.velocity.Y))
	s.DrawTextLine("iterations %d", s.totalIterations)

	if s.lockCamera {
		s.Context.Camera.Center.X = ToFloat64(s.transform.P.X)
	}
}
