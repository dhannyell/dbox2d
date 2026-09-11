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
	maxPush      dbox2d.Q
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
	point    dbox2d.Vec2
	normal   dbox2d.Vec2
	bodyId   dbox2d.BodyId
	fraction dbox2d.Q
	hit      bool
}

const moverPlaneCapacity = 8

var (
	moverElevatorBase      = dbox2d.Vec2{X: dbox2d.QFromInt(112), Y: dbox2d.QFromInt(10)}
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
	transform       dbox2d.Transform
	velocity        dbox2d.Vec2
	capsule         dbox2d.Capsule
	elevatorId      dbox2d.BodyId
	ballId          dbox2d.ShapeId
	friendlyShape   moverShapeUserData
	elevatorShape   moverShapeUserData
	planes          [moverPlaneCapacity]dbox2d.CollisionPlane
	planeCount      int
	totalIterations int
	pogoVelocity    dbox2d.Q
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
	s.transform = dbox2d.Transform{P: dbox2d.Vec2{X: dbox2d.QFromInt(2), Y: dbox2d.QFromInt(8)}, Q: dbox2d.RotIdentity()}
	s.velocity = dbox2d.Vec2{}
	s.capsule = dbox2d.Capsule{
		Center1: dbox2d.Vec2{Y: dbox2d.QHalf().Neg()},
		Center2: dbox2d.Vec2{Y: dbox2d.QHalf()},
		Radius:  dbox2d.QMustParse("0.3"),
	}

	var groundId1 dbox2d.BodyId
	{
		bodyDef := dbox2d.DefaultBodyDef()
		groundId1 = dbox2d.CreateBody(s.WorldId, &bodyDef)

		path := "M 2.6458333,201.08333 H 293.68751 v -47.625 h -2.64584 l -10.58333,7.9375 -13.22916,7.9375 -13.24648,5.29167 " +
			"-31.73269,7.9375 -21.16667,2.64583 -23.8125,10.58333 H 142.875 v -5.29167 h -5.29166 v 5.29167 H 119.0625 v " +
			"-2.64583 h -2.64583 v -2.64584 h -2.64584 v -2.64583 H 111.125 v -2.64583 H 84.666668 v -2.64583 h -5.291666 v " +
			"-2.64584 h -5.291667 v -2.64583 H 68.791668 V 174.625 h -5.291666 v -2.64584 H 52.916669 L 39.6875,177.27083 H " +
			"34.395833 L 23.8125,185.20833 H 15.875 L 5.2916669,187.85416 V 153.45833 H 2.6458333 v 47.625"

		offset := dbox2d.Vec2{X: dbox2d.QFromInt(-50), Y: dbox2d.QFromInt(-200)}
		points := parsePath(path, offset, 64, dbox2d.QFromRatio(1, 5))

		chainDef := dbox2d.DefaultChainDef()
		chainDef.Points = points
		chainDef.IsLoop = true
		dbox2d.CreateChain(groundId1, &chainDef)
	}

	var groundId2 dbox2d.BodyId
	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Position = dbox2d.Vec2{X: dbox2d.QFromInt(98)}
		groundId2 = dbox2d.CreateBody(s.WorldId, &bodyDef)

		path := "M 2.6458333,201.08333 H 293.68751 l 0,-23.8125 h -23.8125 l 21.16667,21.16667 h -23.8125 l -39.68751,-13.22917 " +
			"-26.45833,7.9375 -23.8125,2.64583 h -13.22917 l -0.0575,2.64584 h -5.29166 v -2.64583 l -7.86855,-1e-5 " +
			"-0.0114,-2.64583 h -2.64583 l -2.64583,2.64584 h -7.9375 l -2.64584,2.64583 -2.58891,-2.64584 h -13.28609 v " +
			"-2.64583 h -2.64583 v -2.64584 l -5.29167,1e-5 v -2.64583 h -2.64583 v -2.64583 l -5.29167,-1e-5 v -2.64583 h " +
			"-2.64583 v -2.64584 h -5.291667 v -2.64583 H 92.60417 V 174.625 h -5.291667 v -2.64584 l -34.395835,1e-5 " +
			"-7.9375,-2.64584 -7.9375,-2.64583 -5.291667,-5.29167 H 21.166667 L 13.229167,158.75 5.2916668,153.45833 H " +
			"2.6458334 l -10e-8,47.625"

		offset := dbox2d.Vec2{Y: dbox2d.QFromInt(-200)}
		points := parsePath(path, offset, 64, dbox2d.QFromRatio(1, 5))

		chainDef := dbox2d.DefaultChainDef()
		chainDef.Points = points
		chainDef.IsLoop = true
		dbox2d.CreateChain(groundId2, &chainDef)
	}

	{
		box := dbox2d.MakeBox(dbox2d.QHalf(), dbox2d.QFromRatio(1, 8))

		shapeDef := dbox2d.DefaultShapeDef()

		jointDef := dbox2d.DefaultRevoluteJointDef()
		jointDef.MaxMotorTorque = dbox2d.QFromInt(10)
		jointDef.EnableMotor = true
		jointDef.Hertz = dbox2d.QFromInt(3)
		jointDef.DampingRatio = dbox2d.QMustParse("0.8")
		jointDef.EnableSpring = true

		xBase := dbox2d.QMustParse("48.7")
		yBase := dbox2d.QMustParse("9.2")
		count := 50
		prevBodyId := groundId1
		for i := range count {
			bodyDef := dbox2d.DefaultBodyDef()
			bodyDef.Type = dbox2d.DynamicBody
			bodyDef.Position = dbox2d.Vec2{X: xBase.Add(dbox2d.QHalf()).Add(dbox2d.QFromInt(i)), Y: yBase}
			bodyDef.AngularDamping = dbox2d.QMustParse("0.2")
			bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)
			dbox2d.CreatePolygonShape(bodyId, &shapeDef, &box)

			pivot := dbox2d.Vec2{X: xBase.Add(dbox2d.QFromInt(i)), Y: yBase}
			jointDef.BodyIdA = prevBodyId
			jointDef.BodyIdB = bodyId
			jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
			jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
			dbox2d.CreateRevoluteJoint(s.WorldId, &jointDef)

			prevBodyId = bodyId
		}

		pivot := dbox2d.Vec2{X: xBase.Add(dbox2d.QFromInt(count)), Y: yBase}
		jointDef.BodyIdA = prevBodyId
		jointDef.BodyIdB = groundId2
		jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
		jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
		dbox2d.CreateRevoluteJoint(s.WorldId, &jointDef)
	}

	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Position = dbox2d.Vec2{X: dbox2d.QFromInt(32), Y: dbox2d.QMustParse("4.5")}

		shapeDef := dbox2d.DefaultShapeDef()
		s.friendlyShape.maxPush = dbox2d.QMustParse("0.025")
		s.friendlyShape.clipVelocity = false

		shapeDef.Filter = dbox2d.Filter{CategoryBits: moverMoverBit, MaskBits: moverAllBits}
		shapeDef.UserData = &s.friendlyShape
		bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)
		dbox2d.CreateCapsuleShape(bodyId, &shapeDef, &s.capsule)
	}

	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = dbox2d.Vec2{X: dbox2d.QFromInt(7), Y: dbox2d.QFromInt(7)}
		bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		shapeDef := dbox2d.DefaultShapeDef()
		shapeDef.Filter = dbox2d.Filter{CategoryBits: moverDebrisBit, MaskBits: moverAllBits}
		shapeDef.Material.Restitution = dbox2d.QMustParse("0.7")
		shapeDef.Material.RollingResistance = dbox2d.QMustParse("0.2")

		circle := dbox2d.Circle{Radius: dbox2d.QMustParse("0.3")}
		s.ballId = dbox2d.CreateCircleShape(bodyId, &shapeDef, &circle)
	}

	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.KinematicBody
		bodyDef.Position = dbox2d.Vec2{
			X: moverElevatorBase.X,
			Y: moverElevatorBase.Y.Sub(FromFloat64(moverElevatorAmplitude)),
		}
		s.elevatorId = dbox2d.CreateBody(s.WorldId, &bodyDef)

		s.elevatorShape = moverShapeUserData{
			maxPush:      dbox2d.QFromRatio(1, 10),
			clipVelocity: true,
		}
		shapeDef := dbox2d.DefaultShapeDef()
		shapeDef.Filter = dbox2d.Filter{CategoryBits: moverDynamicBit, MaskBits: moverAllBits}
		shapeDef.UserData = &s.elevatorShape

		box := dbox2d.MakeBox(dbox2d.QFromInt(2), dbox2d.QFromRatio(1, 10))
		dbox2d.CreatePolygonShape(s.elevatorId, &shapeDef, &box)
	}

	s.totalIterations = 0
	s.pogoVelocity = dbox2d.QZero()
	s.onGround = false
	s.jumpReleased = true
	s.lockCamera = true
	s.planeCount = 0
	s.time = 0.0

	return s
}

// solveMove moves the capsule for one step.
// https://github.com/id-Software/Quake/blob/master/QW/client/pmove.c#L390
func (s *Mover) solveMove(timeStep, throttle dbox2d.Q) {
	zero := dbox2d.QZero()
	one := dbox2d.QOne()
	minSpeed := FromFloat64(s.minSpeed)
	maxSpeed := FromFloat64(s.maxSpeed)
	stopSpeed := FromFloat64(s.stopSpeed)
	draw := &s.Context.Draw

	// Friction
	speed, _ := dbox2d.GetLengthAndNormalize(s.velocity)
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

	desiredVelocity := dbox2d.Vec2{X: maxSpeed.Mul(throttle)}
	desiredSpeed, desiredDirection := dbox2d.GetLengthAndNormalize(desiredVelocity)

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

		s.velocity = dbox2d.MulAdd(s.velocity, accelSpeed, desiredDirection)
	}

	s.velocity.Y = s.velocity.Y.Sub(FromFloat64(s.gravity).Mul(timeStep))

	radius := s.capsule.Radius
	pogoRestLength := dbox2d.QFromInt(3).Mul(radius)
	rayLength := pogoRestLength.Add(radius)
	origin := dbox2d.TransformPoint(s.transform, s.capsule.Center1)
	circle := dbox2d.Circle{Center: origin, Radius: dbox2d.QHalf().Mul(radius)}
	segmentOffset := dbox2d.Vec2{X: dbox2d.QFromRatio(3, 4).Mul(radius)}
	segment := dbox2d.Segment{
		Point1: origin.Sub(segmentOffset),
		Point2: origin.Add(segmentOffset),
	}

	var proxy dbox2d.ShapeProxy
	var translation dbox2d.Vec2
	pogoFilter := dbox2d.QueryFilter{CategoryBits: moverMoverBit, MaskBits: moverStaticBit | moverDynamicBit}
	var castResult moverCastResult

	switch s.pogoShape {
	case pogoPoint:
		proxy = dbox2d.MakeProxy([]dbox2d.Vec2{origin}, zero)
		translation = dbox2d.Vec2{Y: rayLength.Neg()}
	case pogoCircle:
		proxy = dbox2d.MakeProxy([]dbox2d.Vec2{origin}, circle.Radius)
		translation = dbox2d.Vec2{Y: rayLength.Neg().Add(circle.Radius)}
	default:
		proxy = dbox2d.MakeProxy([]dbox2d.Vec2{segment.Point1, segment.Point2}, zero)
		translation = dbox2d.Vec2{Y: rayLength.Neg()}
	}

	s.WorldId.CastShape(&proxy, translation, pogoFilter, func(shapeId dbox2d.ShapeId, point, normal dbox2d.Vec2, fraction dbox2d.Q) dbox2d.Q {
		castResult.point = point
		castResult.normal = normal
		castResult.bodyId = shapeId.GetBody()
		castResult.fraction = fraction
		castResult.hit = true
		return fraction
	})

	// Avoid snapping to ground if still going up
	if !s.onGround {
		s.onGround = castResult.hit && !s.velocity.Y.Greater(dbox2d.QFromRatio(1, 100))
	} else {
		s.onGround = castResult.hit
	}

	if !castResult.hit {
		s.pogoVelocity = zero

		delta := translation
		draw.DrawSegment(origin, origin.Add(delta), dbox2d.ColorGray)

		switch s.pogoShape {
		case pogoPoint:
			draw.DrawPoint(origin.Add(delta), dbox2d.QFromInt(10), dbox2d.ColorGray)
		case pogoCircle:
			draw.DrawCircle(origin.Add(delta), circle.Radius, dbox2d.ColorGray)
		default:
			draw.DrawSegment(segment.Point1.Add(delta), segment.Point2.Add(delta), dbox2d.ColorGray)
		}
	} else {
		pogoCurrentLength := castResult.fraction.Mul(rayLength)

		offset := pogoCurrentLength.Sub(pogoRestLength)
		s.pogoVelocity = dbox2d.SpringDamper(FromFloat64(s.pogoHertz), FromFloat64(s.pogoDampingRatio), offset, s.pogoVelocity, timeStep)

		delta := translation.Mul(castResult.fraction)
		draw.DrawSegment(origin, origin.Add(delta), dbox2d.ColorGray)

		switch s.pogoShape {
		case pogoPoint:
			draw.DrawPoint(origin.Add(delta), dbox2d.QFromInt(10), dbox2d.ColorPlum)
		case pogoCircle:
			draw.DrawCircle(origin.Add(delta), circle.Radius, dbox2d.ColorPlum)
		default:
			draw.DrawSegment(segment.Point1.Add(delta), segment.Point2.Add(delta), dbox2d.ColorPlum)
		}

		castResult.bodyId.ApplyForce(dbox2d.Vec2{Y: dbox2d.QFromInt(-50)}, castResult.point, true)
	}

	target := s.transform.P.Add(s.velocity.Mul(timeStep)).Add(dbox2d.Vec2{Y: timeStep.Mul(s.pogoVelocity)})

	// Mover overlap filter
	collideFilter := dbox2d.QueryFilter{CategoryBits: moverMoverBit, MaskBits: moverStaticBit | moverDynamicBit | moverMoverBit}

	// Movers don't sweep against other movers, allows for soft collision
	castFilter := dbox2d.QueryFilter{CategoryBits: moverMoverBit, MaskBits: moverStaticBit | moverDynamicBit}

	s.totalIterations = 0
	tolerance := dbox2d.QFromRatio(1, 100)

	for range 5 {
		s.planeCount = 0

		mover := dbox2d.Capsule{
			Center1: dbox2d.TransformPoint(s.transform, s.capsule.Center1),
			Center2: dbox2d.TransformPoint(s.transform, s.capsule.Center2),
			Radius:  s.capsule.Radius,
		}

		s.WorldId.CollideMover(&mover, collideFilter, s.planeResult)
		result := dbox2d.SolvePlanes(target.Sub(s.transform.P), s.planes[:s.planeCount])

		s.totalIterations += result.IterationCount

		fraction := s.WorldId.CastMover(&mover, result.Translation, castFilter)

		delta := result.Translation.Mul(fraction)
		s.transform.P = s.transform.P.Add(delta)

		if delta.Dot(delta).Less(tolerance.Mul(tolerance)) {
			break
		}
	}

	s.velocity = dbox2d.ClipVector(s.velocity, s.planes[:s.planeCount])
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
func (s *Mover) planeResult(shapeId dbox2d.ShapeId, planeResult *dbox2d.PlaneResult) bool {
	maxPush := dbox2d.Huge
	clipVelocity := true
	if userData, ok := shapeId.GetUserData().(*moverShapeUserData); ok && userData != nil {
		maxPush = userData.maxPush
		clipVelocity = userData.clipVelocity
	}

	if s.planeCount < moverPlaneCapacity {
		s.planes[s.planeCount] = dbox2d.CollisionPlane{
			Plane:        planeResult.Plane,
			PushLimit:    maxPush,
			Push:         dbox2d.QZero(),
			ClipVelocity: clipVelocity,
		}
		s.planeCount++
	}

	return true
}

// kick pushes a dynamic body away from the mover and up.
func (s *Mover) kick(shapeId dbox2d.ShapeId) bool {
	bodyId := shapeId.GetBody()
	if bodyId.GetType() != dbox2d.DynamicBody {
		return true
	}

	center := bodyId.GetWorldCenterOfMass()
	_, direction := dbox2d.GetLengthAndNormalize(center.Sub(s.transform.P))
	impulse := dbox2d.Vec2{X: dbox2d.QFromInt(2).Mul(direction.X), Y: dbox2d.QFromInt(2)}
	bodyId.ApplyLinearImpulseToCenter(impulse, true)

	return true
}

// Keyboard kicks the debris near the mover's feet with K.
func (s *Mover) Keyboard(key Key) {
	if key == KeyK {
		point := dbox2d.TransformPoint(s.transform, dbox2d.Vec2{Y: s.capsule.Center1.Y.Sub(dbox2d.QFromInt(3).Mul(s.capsule.Radius))})
		circle := dbox2d.Circle{Center: point, Radius: dbox2d.QHalf()}
		proxy := dbox2d.MakeProxy([]dbox2d.Vec2{circle.Center}, circle.Radius)
		filter := dbox2d.QueryFilter{CategoryBits: moverMoverBit, MaskBits: moverDebrisBit}
		s.WorldId.OverlapShape(&proxy, filter, s.kick)
		s.Context.Draw.DrawCircle(circle.Center, circle.Radius, dbox2d.ColorGoldenRod)
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
		point := dbox2d.Vec2{
			X: moverElevatorBase.X,
			Y: FromFloat64(moverElevatorAmplitude * math.Cos(1.0*s.time+math.Pi)).Add(moverElevatorBase.Y),
		}

		s.elevatorId.SetTargetTransform(dbox2d.Transform{P: point, Q: dbox2d.RotIdentity()}, FromFloat64(timeStep))
	}

	s.time += timeStep

	s.Base.Step()

	if !pause {
		throttle := dbox2d.QZero()

		if s.keyDown(KeyA) {
			throttle = throttle.Sub(dbox2d.QOne())
		}

		if s.keyDown(KeyD) {
			throttle = throttle.Add(dbox2d.QOne())
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
		p1 := dbox2d.MulAdd(s.transform.P, plane.Offset.Sub(radius), plane.Normal)
		p2 := dbox2d.MulAdd(p1, dbox2d.QFromRatio(1, 10), plane.Normal)
		draw.DrawPoint(p1, dbox2d.QFromInt(5), dbox2d.ColorYellow)
		draw.DrawSegment(p1, p2, dbox2d.ColorYellow)
	}

	p1 := dbox2d.TransformPoint(s.transform, s.capsule.Center1)
	p2 := dbox2d.TransformPoint(s.transform, s.capsule.Center2)

	color := dbox2d.ColorAquamarine
	if s.onGround {
		color = dbox2d.ColorOrange
	}
	draw.DrawSolidCapsule(p1, p2, radius, color)
	draw.DrawSegment(s.transform.P, s.transform.P.Add(s.velocity), dbox2d.ColorPurple)

	p := s.transform.P
	s.DrawTextLine("position %.2f %.2f", ToFloat64(p.X), ToFloat64(p.Y))
	s.DrawTextLine("velocity %.2f %.2f", ToFloat64(s.velocity.X), ToFloat64(s.velocity.Y))
	s.DrawTextLine("iterations %d", s.totalIterations)

	if s.lockCamera {
		s.Context.Camera.Center.X = ToFloat64(s.transform.P.X)
	}
}
