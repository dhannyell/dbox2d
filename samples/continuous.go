// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/sample_continuous.cpp of Box2D v3.1.1

package samples

import (
	"fmt"

	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/dbox2d/internal/shared"
)

func init() {
	RegisterSample("Continuous", "Bounce House", NewBounceHouse)
	RegisterSample("Continuous", "Bounce Humans", NewBounceHumans)
	RegisterSample("Continuous", "Chain Drop", NewChainDrop)
	RegisterSample("Continuous", "Chain Slide", NewChainSlide)
	RegisterSample("Continuous", "Segment Slide", NewSegmentSlide)
	RegisterSample("Continuous", "Skinny Box", NewSkinnyBox)
	RegisterSample("Continuous", "Ghost Bumps", NewGhostBumps)
	RegisterSample("Continuous", "Speculative Fallback", NewSpeculativeFallback)
	RegisterSample("Continuous", "Speculative Sliver", NewSpeculativeSliver)
	RegisterSample("Continuous", "Speculative Ghost", NewSpeculativeGhost)
	RegisterSample("Continuous", "Pixel Imperfect", NewPixelImperfect)
	RegisterSample("Continuous", "Restitution Threshold", NewRestitutionThreshold)
	RegisterSample("Continuous", "Drop", NewDrop)
	RegisterSample("Continuous", "Pinball", NewPinball)
	RegisterSample("Continuous", "Wedge", NewWedge)
}

// radiansQToTurns converts a scalar angle of the reference to turns.
func radiansQToTurns(radians b2.Q) b2.Q {
	return radians.Div(b2.F(2).Mul(b2.Pi()))
}

// createBoxWalls adds the four walls of a 20 by 20 room centered on the
// origin to the body, as Bounce House and Bounce Humans do.
func createBoxWalls(bodyId b2.BodyId, shapeDef *b2.ShapeDef) {
	corners := [4]b2.Vec2{b2.V2(-10, -10), b2.V2(10, -10), b2.V2(10, 10), b2.V2(-10, 10)}
	for i := range corners {
		segment := b2.Segment{Point1: corners[i], Point2: corners[(i+1)%4]}
		b2.CreateSegmentShape(bodyId, shapeDef, &segment)
	}
}

// The launched shapes of Bounce House and Ghost Bumps.
const (
	launchCircle = iota
	launchCapsule
	launchBox
)

var launchShapeNames = []string{"Circle", "Capsule", "Box"}

type bounceHitEvent struct {
	point     b2.Vec2
	speed     b2.Q
	stepIndex int
}

// BounceHouse bounces a fast body inside a closed room.
type BounceHouse struct {
	Base

	hitEvents       [4]bounceHitEvent
	bodyId          b2.BodyId
	shapeType       int
	enableHitEvents bool
}

func NewBounceHouse(ctx *SampleContext) Sample {
	s := &BounceHouse{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 0}
		ctx.Camera.Zoom = 25 * 0.45
	}

	bodyDef := b2.DefaultBodyDef()
	groundId := b2.CreateBody(s.WorldId, &bodyDef)

	shapeDef := b2.DefaultShapeDef()
	createBoxWalls(groundId, &shapeDef)

	s.shapeType = launchBox
	s.bodyId = b2.BodyId{}
	s.enableHitEvents = true

	s.launch()
	return s
}

func (s *BounceHouse) launch() {
	if !s.bodyId.IsNull() {
		b2.DestroyBody(s.bodyId)
	}

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.LinearVelocity = b2.V2(10, 20)
	bodyDef.Position = b2.V2(0, 0)
	bodyDef.GravityScale = b2.QZero()

	// Circle shapes centered on the body can spin fast without risk of tunnelling.
	bodyDef.AllowFastRotation = s.shapeType == launchCircle

	s.bodyId = b2.CreateBody(s.WorldId, &bodyDef)

	shapeDef := b2.DefaultShapeDef()
	shapeDef.Density = b2.QOne()
	shapeDef.Material.Restitution = b2.F(1.2)
	shapeDef.Material.Friction = b2.F(0.3)
	shapeDef.EnableHitEvents = s.enableHitEvents

	if s.shapeType == launchCircle {
		circle := b2.Circle{Radius: b2.QHalf()}
		b2.CreateCircleShape(s.bodyId, &shapeDef, &circle)
	} else if s.shapeType == launchCapsule {
		capsule := b2.Capsule{Center1: b2.V2(-0.5, 0.0), Center2: b2.V2(0.5, 0.0), Radius: b2.F(0.25)}
		b2.CreateCapsuleShape(s.bodyId, &shapeDef, &capsule)
	} else {
		h := b2.F(0.1)
		box := b2.MakeBox(b2.F(20).Mul(h), h)
		b2.CreatePolygonShape(s.bodyId, &shapeDef, &box)
	}
}

func (s *BounceHouse) UpdateGui() {
	gui := s.Context.Gui
	height := 100
	gui.Begin("Bounce House", 10, s.Context.Camera.Height-height-50, 240, height)

	if gui.Combo("Shape", &s.shapeType, launchShapeNames) {
		s.launch()
	}

	if gui.Checkbox("hit events", &s.enableHitEvents) {
		s.bodyId.EnableHitEvents(s.enableHitEvents)
	}

	gui.End()
}

func (s *BounceHouse) Step() {
	s.Base.Step()

	events := s.WorldId.GetContactEvents()
	for i := range events.HitEvents {
		event := &events.HitEvents[i]

		e := &s.hitEvents[0]
		for j := 1; j < 4; j++ {
			if s.hitEvents[j].stepIndex < e.stepIndex {
				e = &s.hitEvents[j]
			}
		}

		e.point = event.Point
		e.speed = event.ApproachSpeed
		e.stepIndex = s.StepCount
	}

	draw := &s.Context.Draw
	for i := range s.hitEvents {
		e := &s.hitEvents[i]
		if e.stepIndex > 0 && s.StepCount <= e.stepIndex+30 {
			draw.DrawCircle(e.point, b2.F(0.1), b2.ColorOrangeRed)
			draw.DrawString(e.point, fmt.Sprintf("%.1f", ToFloat64(e.speed)), b2.ColorWhite)
		}
	}
}

// BounceHumans bounces ragdolls in a room whose gravity turns.
type BounceHumans struct {
	Base

	humans     [5]shared.Human
	humanCount int
	countDown  b2.Q
	time       b2.Q
}

func NewBounceHumans(ctx *SampleContext) Sample {
	s := &BounceHumans{Base: NewBase(ctx)}
	ctx.Camera.Center = Vec2f{X: 0, Y: 0}
	ctx.Camera.Zoom = 12

	bodyDef := b2.DefaultBodyDef()
	groundId := b2.CreateBody(s.WorldId, &bodyDef)

	shapeDef := b2.DefaultShapeDef()
	shapeDef.Material.Restitution = b2.F(1.3)
	shapeDef.Material.Friction = b2.F(0.1)

	createBoxWalls(groundId, &shapeDef)

	circle := b2.Circle{Radius: b2.F(2)}
	shapeDef.Material.Restitution = b2.F(2)
	b2.CreateCircleShape(groundId, &shapeDef, &circle)
	return s
}

func (s *BounceHumans) Step() {
	if s.humanCount < 5 && !s.countDown.Greater(b2.QZero()) {
		jointFrictionTorque := b2.QZero()
		jointHertz := b2.QOne()
		jointDampingRatio := b2.F(0.1)

		s.humans[s.humanCount] = shared.CreateHuman(s.WorldId, b2.V2(0, 5), b2.QOne(), jointFrictionTorque, jointHertz,
			jointDampingRatio, 1, nil, true)

		s.countDown = b2.F(2)
		s.humanCount += 1
	}

	timeStep := b2.QFromRatio(1, 60)
	cs1 := b2.MakeRot(radiansQToTurns(b2.QHalf().Mul(s.time)))
	cs2 := b2.MakeRot(radiansQToTurns(s.time))
	gravity := b2.F(10)
	gravityVec := b2.Vec2{X: gravity.Mul(cs1.Sin), Y: gravity.Mul(cs2.Cos)}
	three := b2.F(3)
	s.Context.Draw.DrawSegment(b2.Vec2{}, b2.Vec2{X: three.Mul(cs1.Sin), Y: three.Mul(cs2.Cos)}, b2.ColorWhite)
	s.time = s.time.Add(timeStep)
	s.countDown = s.countDown.Sub(timeStep)
	s.WorldId.SetGravity(gravityVec)

	s.Base.Step()
}

// ChainDrop drops a fast circle on a closed chain.
type ChainDrop struct {
	Base

	bodyId  b2.BodyId
	shapeId b2.ShapeId
	yOffset float64
	speed   float64
}

func NewChainDrop(ctx *SampleContext) Sample {
	s := &ChainDrop{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 0}
		ctx.Camera.Zoom = 25 * 0.35
	}

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Position = b2.V2(0, -6)
	groundId := b2.CreateBody(s.WorldId, &bodyDef)

	points := []b2.Vec2{b2.V2(-10, -2), b2.V2(10, -2), b2.V2(10, 1), b2.V2(-10, 1)}

	chainDef := b2.DefaultChainDef()
	chainDef.Points = points
	chainDef.IsLoop = true

	b2.CreateChain(groundId, &chainDef)

	s.bodyId = b2.BodyId{}
	s.yOffset = -0.1
	s.speed = -42

	s.launch()
	return s
}

func (s *ChainDrop) launch() {
	if !s.bodyId.IsNull() {
		b2.DestroyBody(s.bodyId)
	}

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.LinearVelocity = b2.Vec2{Y: FromFloat64(s.speed)}
	bodyDef.Position = b2.Vec2{Y: b2.F(10).Add(FromFloat64(s.yOffset))}
	bodyDef.Rotation = b2.MakeRot(b2.QFromRatio(1, 4))
	bodyDef.FixedRotation = true
	s.bodyId = b2.CreateBody(s.WorldId, &bodyDef)

	shapeDef := b2.DefaultShapeDef()

	circle := b2.Circle{Radius: b2.QHalf()}
	s.shapeId = b2.CreateCircleShape(s.bodyId, &shapeDef, &circle)
}

func (s *ChainDrop) UpdateGui() {
	gui := s.Context.Gui
	height := 140
	gui.Begin("Chain Drop", 10, s.Context.Camera.Height-height-50, 240, height)

	gui.SliderFloat("Speed", &s.speed, -100, 0)
	gui.SliderFloat("Y Offset", &s.yOffset, -1, 1)

	if gui.Button("Launch") {
		s.launch()
	}

	gui.End()
}

// ChainSlide slides a fast circle along the inside of a closed chain.
type ChainSlide struct {
	Base
}

func NewChainSlide(ctx *SampleContext) Sample {
	s := &ChainSlide{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 10}
		ctx.Camera.Zoom = 15
	}

	{
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(s.WorldId, &bodyDef)

		const count = 80
		points := make([]b2.Vec2, count)

		w := b2.F(2)
		h := b2.QOne()
		x, y := b2.F(20), b2.QZero()
		for i := 0; i < 20; i++ {
			points[i] = b2.Vec2{X: x, Y: y}
			x = x.Sub(w)
		}

		for i := 20; i < 40; i++ {
			points[i] = b2.Vec2{X: x, Y: y}
			y = y.Add(h)
		}

		for i := 40; i < 60; i++ {
			points[i] = b2.Vec2{X: x, Y: y}
			x = x.Add(w)
		}

		for i := 60; i < 80; i++ {
			points[i] = b2.Vec2{X: x, Y: y}
			y = y.Sub(h)
		}

		chainDef := b2.DefaultChainDef()
		chainDef.Points = points
		chainDef.IsLoop = true

		b2.CreateChain(groundId, &chainDef)
	}

	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.LinearVelocity = b2.V2(100, 0)
		bodyDef.Position = b2.V2(-19.5, 0.5)
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		shapeDef.Material.Friction = b2.QZero()
		circle := b2.Circle{Radius: b2.QHalf()}
		b2.CreateCircleShape(bodyId, &shapeDef, &circle)
	}
	return s
}

// SegmentSlide slides a fast circle along a segment into a wall.
type SegmentSlide struct {
	Base
}

func NewSegmentSlide(ctx *SampleContext) Sample {
	s := &SegmentSlide{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 10}
		ctx.Camera.Zoom = 15
	}

	{
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		segment := b2.Segment{Point1: b2.V2(-40, 0), Point2: b2.V2(40, 0)}
		b2.CreateSegmentShape(groundId, &shapeDef, &segment)

		segment = b2.Segment{Point1: b2.V2(40, 0), Point2: b2.V2(40, 10)}
		b2.CreateSegmentShape(groundId, &shapeDef, &segment)
	}

	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.LinearVelocity = b2.V2(100, 0)
		bodyDef.Position = b2.V2(-20.0, 0.7)
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		circle := b2.Circle{Radius: b2.QHalf()}
		b2.CreateCircleShape(bodyId, &shapeDef, &circle)
	}
	return s
}

// SkinnyBox drops a fast spinning thin box on a thin post.
type SkinnyBox struct {
	Base

	bodyId, bulletId b2.BodyId
	angularVelocity  b2.Q
	x                b2.Q
	capsule          bool
	autoTest         bool
	bullet           bool
}

func NewSkinnyBox(ctx *SampleContext) Sample {
	s := &SkinnyBox{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 1, Y: 5}
		ctx.Camera.Zoom = 25 * 0.25
	}

	{
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(s.WorldId, &bodyDef)

		segment := b2.Segment{Point1: b2.V2(-10, 0), Point2: b2.V2(10, 0)}
		shapeDef := b2.DefaultShapeDef()
		shapeDef.Material.Friction = b2.F(0.9)
		b2.CreateSegmentShape(groundId, &shapeDef, &segment)

		box := b2.MakeOffsetBox(b2.F(0.1), b2.QOne(), b2.V2(0, 1), b2.RotIdentity())
		b2.CreatePolygonShape(groundId, &shapeDef, &box)
	}

	s.autoTest = false
	s.bullet = false
	s.capsule = false

	s.bodyId = b2.BodyId{}
	s.bulletId = b2.BodyId{}

	s.launch()
	return s
}

func (s *SkinnyBox) launch() {
	if !s.bodyId.IsNull() {
		b2.DestroyBody(s.bodyId)
	}

	if !s.bulletId.IsNull() {
		b2.DestroyBody(s.bulletId)
	}

	// The reference draws radians per second; the body takes turns.
	s.angularVelocity = shared.RandomFloatRange(b2.F(-50), b2.F(50))

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.Position = b2.V2(0, 8)
	bodyDef.AngularVelocity = radiansQToTurns(s.angularVelocity)
	bodyDef.LinearVelocity = b2.V2(0, -100)

	shapeDef := b2.DefaultShapeDef()
	shapeDef.Density = b2.QOne()
	shapeDef.Material.Friction = b2.F(0.9)

	s.bodyId = b2.CreateBody(s.WorldId, &bodyDef)

	if s.capsule {
		capsule := b2.Capsule{Center1: b2.V2(0, -1), Center2: b2.V2(0, 1), Radius: b2.F(0.1)}
		b2.CreateCapsuleShape(s.bodyId, &shapeDef, &capsule)
	} else {
		polygon := b2.MakeBox(b2.F(2), b2.F(0.05))
		b2.CreatePolygonShape(s.bodyId, &shapeDef, &polygon)
	}

	if s.bullet {
		polygon := b2.MakeBox(b2.F(0.25), b2.F(0.25))
		s.x = shared.RandomFloatRange(b2.F(-1), b2.QOne())
		bodyDef.Position = b2.Vec2{X: s.x, Y: b2.F(10)}
		bodyDef.LinearVelocity = b2.V2(0, -50)
		s.bulletId = b2.CreateBody(s.WorldId, &bodyDef)
		b2.CreatePolygonShape(s.bulletId, &shapeDef, &polygon)
	}
}

func (s *SkinnyBox) UpdateGui() {
	gui := s.Context.Gui
	height := 110
	gui.Begin("Skinny Box", 10, s.Context.Camera.Height-height-50, 140, height)

	gui.Checkbox("Capsule", &s.capsule)

	if gui.Button("Launch") {
		s.launch()
	}

	gui.Checkbox("Auto Test", &s.autoTest)

	gui.End()
}

func (s *SkinnyBox) Step() {
	s.Base.Step()

	if s.autoTest && s.StepCount%60 == 0 {
		s.launch()
	}
}

// GhostBumps shows ghost bumps.
type GhostBumps struct {
	Base

	groundId  b2.BodyId
	bodyId    b2.BodyId
	shapeId   b2.ShapeId
	shapeType int
	round     float64
	friction  float64
	bevel     float64
	useChain  bool
}

func NewGhostBumps(ctx *SampleContext) Sample {
	s := &GhostBumps{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 1.5, Y: 16}
		ctx.Camera.Zoom = 25 * 0.8
	}

	s.groundId = b2.BodyId{}
	s.bodyId = b2.BodyId{}
	s.shapeId = b2.ShapeId{}
	s.shapeType = launchCircle
	s.round = 0
	s.friction = 0.2
	s.bevel = 0
	s.useChain = true

	s.createScene()
	s.launch()
	return s
}

func (s *GhostBumps) createScene() {
	if !s.groundId.IsNull() {
		b2.DestroyBody(s.groundId)
	}

	s.shapeId = b2.ShapeId{}

	bodyDef := b2.DefaultBodyDef()
	s.groundId = b2.CreateBody(s.WorldId, &bodyDef)

	one := b2.QOne()
	two := b2.F(2)
	sqrt2 := two.Sqrt()
	m := one.Div(sqrt2)
	mm := two.Mul(sqrt2.Sub(one))
	hx, hy := b2.F(4), b2.F(0.25)
	friction := FromFloat64(s.friction)

	if s.useChain {
		// step returns p moved by (x, y).
		step := func(p b2.Vec2, x, y b2.Q) b2.Vec2 { return p.Add(b2.Vec2{X: x, Y: y}) }

		hxm := two.Mul(hx).Mul(m)
		hym := two.Mul(hy).Mul(m)
		corner := hxm.Add(two.Mul(hy).Mul(one.Sub(m)))
		flat := two.Mul(hx)
		flatBevel := flat.Add(hy.Mul(mm))

		var points [20]b2.Vec2
		points[0] = b2.Vec2{X: b2.F(-3).Mul(hx), Y: hy}
		points[1] = step(points[0], hxm.Neg(), hxm)
		points[2] = step(points[1], hxm.Neg(), hxm)
		points[3] = step(points[2], hxm.Neg(), hxm)
		points[4] = step(points[3], hym.Neg(), hym.Neg())
		points[5] = step(points[4], hxm, hxm.Neg())
		points[6] = step(points[5], hxm, hxm.Neg())
		points[7] = step(points[6], corner, corner.Neg())
		points[8] = step(points[7], flatBevel, b2.QZero())
		points[9] = step(points[8], flat, b2.QZero())
		points[10] = step(points[9], flatBevel, b2.QZero())
		points[11] = step(points[10], corner, corner)
		points[12] = step(points[11], hxm, hxm)
		points[13] = step(points[12], hxm, hxm)
		points[14] = step(points[13], hym.Neg(), hym)
		points[15] = step(points[14], hxm.Neg(), hxm.Neg())
		points[16] = step(points[15], hxm.Neg(), hxm.Neg())
		points[17] = step(points[16], hxm.Neg(), hxm.Neg())
		points[18] = step(points[17], flat.Neg(), b2.QZero())
		points[19] = step(points[18], flat.Neg(), b2.QZero())

		material := b2.SurfaceMaterial{Friction: friction}

		chainDef := b2.DefaultChainDef()
		chainDef.Points = points[:]
		chainDef.IsLoop = true
		chainDef.Materials = []b2.SurfaceMaterial{material}

		b2.CreateChain(s.groundId, &chainDef)
	} else {
		shapeDef := b2.DefaultShapeDef()
		shapeDef.Material.Friction = friction

		var hull b2.Hull

		if s.bevel > 0 {
			hb := FromFloat64(s.bevel)
			d := b2.F(0.05)
			vs := []b2.Vec2{
				{X: hx.Add(hb), Y: hy.Sub(d)}, {X: hx, Y: hy}, {X: hx.Neg(), Y: hy}, {X: hx.Neg().Sub(hb), Y: hy.Sub(d)},
				{X: hx.Neg().Sub(hb), Y: hy.Neg().Add(d)}, {X: hx.Neg(), Y: hy.Neg()}, {X: hx, Y: hy.Neg()}, {X: hx.Add(hb), Y: hy.Neg().Add(d)},
			}
			hull = b2.ComputeHull(vs)
		} else {
			vs := []b2.Vec2{{X: hx, Y: hy}, {X: hx.Neg(), Y: hy}, {X: hx.Neg(), Y: hy.Neg()}, {X: hx, Y: hy.Neg()}}
			hull = b2.ComputeHull(vs)
		}

		// place adds three polygons along (dx, dy) from (x, y).
		place := func(x, y, dx, dy b2.Q, q b2.Rot) {
			for range 3 {
				polygon := b2.MakeOffsetPolygon(&hull, b2.Vec2{X: x, Y: y}, q)
				b2.CreatePolygonShape(s.groundId, &shapeDef, &polygon)
				x = x.Add(dx)
				y = y.Add(dy)
			}
		}

		slopeStep := two.Mul(m).Mul(hx)
		slopeY := hy.Add(m.Mul(hx)).Sub(m.Mul(hy))
		slopeX := b2.F(3).Mul(hx).Add(m.Mul(hx)).Add(m.Mul(hy))

		// Left slope
		place(slopeX.Neg(), slopeY, slopeStep.Neg(), slopeStep, b2.MakeRot(b2.QFromRatio(-1, 8)))

		place(two.Mul(hx).Neg(), b2.QZero(), two.Mul(hx), b2.QZero(), b2.MakeRot(b2.QZero()))

		place(slopeX, slopeY, slopeStep, slopeStep, b2.MakeRot(b2.QFromRatio(1, 8)))
	}
}

func (s *GhostBumps) launch() {
	if !s.bodyId.IsNull() {
		b2.DestroyBody(s.bodyId)
		s.shapeId = b2.ShapeId{}
	}

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.Position = b2.V2(-28, 18)
	bodyDef.LinearVelocity = b2.V2(0, 0)
	s.bodyId = b2.CreateBody(s.WorldId, &bodyDef)

	shapeDef := b2.DefaultShapeDef()
	shapeDef.Density = b2.QOne()
	shapeDef.Material.Friction = FromFloat64(s.friction)

	if s.shapeType == launchCircle {
		circle := b2.Circle{Radius: b2.QHalf()}
		s.shapeId = b2.CreateCircleShape(s.bodyId, &shapeDef, &circle)
	} else if s.shapeType == launchCapsule {
		capsule := b2.Capsule{Center1: b2.V2(-0.5, 0.0), Center2: b2.V2(0.5, 0.0), Radius: b2.F(0.25)}
		s.shapeId = b2.CreateCapsuleShape(s.bodyId, &shapeDef, &capsule)
	} else {
		round := FromFloat64(s.round)
		h := b2.QHalf().Sub(round)
		box := b2.MakeRoundedBox(h, b2.F(2).Mul(h), round)
		s.shapeId = b2.CreatePolygonShape(s.bodyId, &shapeDef, &box)
	}
}

func (s *GhostBumps) UpdateGui() {
	gui := s.Context.Gui
	height := 140
	gui.Begin("Ghost Bumps", 10, s.Context.Camera.Height-height-50, 180, height)

	if gui.Checkbox("Chain", &s.useChain) {
		s.createScene()
	}

	if !s.useChain {
		if gui.SliderFloat("Bevel", &s.bevel, 0, 1) {
			s.createScene()
		}
	}

	gui.Combo("Shape", &s.shapeType, launchShapeNames)

	if s.shapeType == launchBox {
		gui.SliderFloat("Round", &s.round, 0, 0.4)
	}

	if gui.SliderFloat("Friction", &s.friction, 0, 1) {
		if !s.shapeId.IsNull() {
			s.shapeId.SetFriction(FromFloat64(s.friction))
		}

		s.createScene()
	}

	if gui.Button("Launch") {
		s.launch()
	}

	gui.End()
}

// SpeculativeFallback is a speculative collision failure case suggested by
// Dirk Gregorius. This uses a simple fallback scheme to prevent tunneling.
type SpeculativeFallback struct {
	Base
}

func NewSpeculativeFallback(ctx *SampleContext) Sample {
	s := &SpeculativeFallback{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 1, Y: 5}
		ctx.Camera.Zoom = 25 * 0.25
	}

	{
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		segment := b2.Segment{Point1: b2.V2(-10, 0), Point2: b2.V2(10, 0)}
		b2.CreateSegmentShape(groundId, &shapeDef, &segment)

		points := []b2.Vec2{b2.V2(-2, 4), b2.V2(2, 4), b2.V2(2.0, 4.1), b2.V2(-0.5, 4.2), b2.V2(-2.0, 4.2)}
		hull := b2.ComputeHull(points)
		poly := b2.MakePolygon(&hull, b2.QZero())
		b2.CreatePolygonShape(groundId, &shapeDef, &poly)
	}

	// Fast moving skinny box. Also testing a large shape offset.
	{
		offset := b2.F(8)
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.Vec2{X: offset, Y: b2.F(12)}
		bodyDef.LinearVelocity = b2.V2(0, -100)
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		box := b2.MakeOffsetBox(b2.F(2), b2.F(0.05), b2.Vec2{X: offset.Neg()}, b2.MakeRot(b2.QHalf()))
		b2.CreatePolygonShape(bodyId, &shapeDef, &box)
	}
	return s
}

// SpeculativeSliver drops a fast sliver on a segment.
type SpeculativeSliver struct {
	Base
}

func NewSpeculativeSliver(ctx *SampleContext) Sample {
	s := &SpeculativeSliver{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 1.75}
		ctx.Camera.Zoom = 2.5
	}

	{
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		segment := b2.Segment{Point1: b2.V2(-10, 0), Point2: b2.V2(10, 0)}
		b2.CreateSegmentShape(groundId, &shapeDef, &segment)
	}

	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.V2(0, 12)
		bodyDef.LinearVelocity = b2.V2(0, -100)
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		points := []b2.Vec2{b2.V2(-2, 0), b2.V2(-1, 0), b2.V2(2.0, 0.5)}
		hull := b2.ComputeHull(points)
		poly := b2.MakePolygon(&hull, b2.QZero())
		b2.CreatePolygonShape(bodyId, &shapeDef, &poly)
	}
	return s
}

// SpeculativeGhost shows that while Box2D uses speculative collision, it
// does not lead to speculative ghost collisions at small distances.
type SpeculativeGhost struct {
	Base
}

func NewSpeculativeGhost(ctx *SampleContext) Sample {
	s := &SpeculativeGhost{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 1.75}
		ctx.Camera.Zoom = 2
	}

	{
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		segment := b2.Segment{Point1: b2.V2(-10, 0), Point2: b2.V2(10, 0)}
		b2.CreateSegmentShape(groundId, &shapeDef, &segment)

		box := b2.MakeOffsetBox(b2.QOne(), b2.F(0.1), b2.V2(0.0, 0.9), b2.RotIdentity())
		b2.CreatePolygonShape(groundId, &shapeDef, &box)
	}

	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody

		// The speculative distance is 0.02 meters, so this avoid it
		bodyDef.Position = b2.V2(0.015, 2.515)
		speed := b2.F(0.125).Mul(FromFloat64(ctx.Settings.Hertz))
		bodyDef.LinearVelocity = b2.Vec2{X: speed, Y: speed.Neg()}
		bodyDef.GravityScale = b2.QZero()
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		box := b2.MakeSquare(b2.F(0.25))
		b2.CreatePolygonShape(bodyId, &shapeDef, &box)
	}
	return s
}

// pixels converts a pixel measure of Pixel Imperfect and Restitution
// Threshold to meters, at 30 pixels per meter.
func pixels(n int) b2.Q { return b2.QFromRatio(n, 30) }

// PixelImperfect shows that Box2D does not have pixel perfect collision.
type PixelImperfect struct {
	Base

	ballId b2.BodyId
}

func NewPixelImperfect(ctx *SampleContext) Sample {
	s := &PixelImperfect{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 7, Y: 5}
		ctx.Camera.Zoom = 6
	}

	{
		block4BodyDef := b2.DefaultBodyDef()
		block4BodyDef.Type = b2.StaticBody
		block4BodyDef.Position = b2.Vec2{X: pixels(175), Y: pixels(150)}
		block4BodyId := b2.CreateBody(s.WorldId, &block4BodyDef)
		block4Shape := b2.MakeBox(pixels(20), pixels(10))
		block4ShapeDef := b2.DefaultShapeDef()
		block4ShapeDef.Material.Friction = b2.QZero()
		b2.CreatePolygonShape(block4BodyId, &block4ShapeDef, &block4Shape)
	}

	{
		ballBodyDef := b2.DefaultBodyDef()
		ballBodyDef.Type = b2.DynamicBody
		ballBodyDef.Position = b2.Vec2{X: pixels(200), Y: pixels(275)}
		ballBodyDef.GravityScale = b2.QZero()

		s.ballId = b2.CreateBody(s.WorldId, &ballBodyDef)
		// Ball shape
		ballShape := b2.MakeRoundedBox(pixels(4), pixels(4), b2.F(0.9).Div(b2.F(30)))
		ballShapeDef := b2.DefaultShapeDef()
		ballShapeDef.Material.Friction = b2.QZero()
		b2.CreatePolygonShape(s.ballId, &ballShapeDef, &ballShape)
		s.ballId.SetLinearVelocity(b2.V2(0, -5))
		s.ballId.SetFixedRotation(true)
	}
	return s
}

func (s *PixelImperfect) Step() {
	var data [1]b2.ContactData
	s.ballId.GetContactData(data[:])

	p := s.ballId.GetPosition()
	v := s.ballId.GetLinearVelocity()
	s.DrawTextLine("p.x = %.9f, v.y = %.9f", ToFloat64(p.X), ToFloat64(v.Y))

	s.Base.Step()
}

// RestitutionThreshold bounces a slow ball off a slope, which needs a low
// restitution threshold.
type RestitutionThreshold struct {
	Base

	ballId b2.BodyId
}

func NewRestitutionThreshold(ctx *SampleContext) Sample {
	s := &RestitutionThreshold{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 7, Y: 5}
		ctx.Camera.Zoom = 6
	}

	// With the default threshold the ball will not bounce.
	s.WorldId.SetRestitutionThreshold(b2.F(0.1))

	{
		block0BodyDef := b2.DefaultBodyDef()
		block0BodyDef.Type = b2.StaticBody
		block0BodyDef.Position = b2.Vec2{X: pixels(205), Y: pixels(120)}
		// The reference writes 70 degrees with pi as 3.14.
		block0BodyDef.Rotation = b2.MakeRot(radiansToTurns(70 * 3.14 / 180))
		block0BodyId := b2.CreateBody(s.WorldId, &block0BodyDef)
		block0Shape := b2.MakeBox(pixels(50), pixels(5))
		block0ShapeDef := b2.DefaultShapeDef()
		block0ShapeDef.Material.Friction = b2.QZero()
		b2.CreatePolygonShape(block0BodyId, &block0ShapeDef, &block0Shape)
	}

	{
		// Make a ball
		ballBodyDef := b2.DefaultBodyDef()
		ballBodyDef.Type = b2.DynamicBody
		ballBodyDef.Position = b2.Vec2{X: pixels(200), Y: pixels(250)}
		s.ballId = b2.CreateBody(s.WorldId, &ballBodyDef)

		ballShape := b2.Circle{Radius: pixels(5)}
		ballShapeDef := b2.DefaultShapeDef()
		ballShapeDef.Material.Friction = b2.QZero()
		ballShapeDef.Material.Restitution = b2.QOne()
		b2.CreateCircleShape(s.ballId, &ballShapeDef, &ballShape)

		s.ballId.SetLinearVelocity(b2.V2(0.0, -2.9)) // Initial velocity
		s.ballId.SetFixedRotation(true)              // Do not rotate a ball
	}
	return s
}

func (s *RestitutionThreshold) Step() {
	var data [1]b2.ContactData
	s.ballId.GetContactData(data[:])

	p := s.ballId.GetPosition()
	v := s.ballId.GetLinearVelocity()
	s.DrawTextLine("p.x = %.9f, v.y = %.9f", ToFloat64(p.X), ToFloat64(v.Y))

	s.Base.Step()
}

// Drop switches between four drop scenes with the number keys.
type Drop struct {
	Base

	groundIds   []b2.BodyId
	bodyIds     []b2.BodyId
	human       shared.Human
	frameSkip   int
	frameCount  int
	continuous  bool
	speculative bool
}

func NewDrop(ctx *SampleContext) Sample {
	s := &Drop{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 1.5}
		ctx.Camera.Zoom = 3
		ctx.Settings.EnableSleep = false
		ctx.Settings.DrawJoints = false
	}

	s.human = shared.Human{}
	s.frameSkip = 0
	s.frameCount = 0
	s.continuous = true
	s.speculative = true

	s.scene1()
	return s
}

func (s *Drop) clear() {
	for _, bodyId := range s.bodyIds {
		b2.DestroyBody(bodyId)
	}

	s.bodyIds = s.bodyIds[:0]

	if s.human.IsSpawned {
		s.human.Destroy()
	}
}

// resetGround destroys the ground bodies and creates a fresh one.
func (s *Drop) resetGround() b2.BodyId {
	for _, groundId := range s.groundIds {
		b2.DestroyBody(groundId)
	}
	s.groundIds = s.groundIds[:0]

	bodyDef := b2.DefaultBodyDef()
	groundId := b2.CreateBody(s.WorldId, &bodyDef)
	s.groundIds = append(s.groundIds, groundId)
	return groundId
}

const (
	dropGroundWidth = 0.25
	dropGroundCount = 40
)

func (s *Drop) createGround1() {
	groundId := s.resetGround()

	shapeDef := b2.DefaultShapeDef()

	w := b2.F(dropGroundWidth)
	half := b2.QHalf().Mul(b2.F(dropGroundCount)).Mul(w)
	segment := b2.Segment{Point1: b2.Vec2{X: half.Neg()}, Point2: b2.Vec2{X: half}}
	b2.CreateSegmentShape(groundId, &shapeDef, &segment)
}

func (s *Drop) createGround2() {
	groundId := s.resetGround()

	shapeDef := b2.DefaultShapeDef()

	w := b2.F(dropGroundWidth)

	x := b2.QHalf().Mul(b2.F(dropGroundCount)).Mul(w).Neg()
	h := b2.F(0.05)
	for j := 0; j <= dropGroundCount; j++ {
		box := b2.MakeOffsetBox(b2.QHalf().Mul(w), h, b2.Vec2{X: x}, b2.RotIdentity())
		b2.CreatePolygonShape(groundId, &shapeDef, &box)
		x = x.Add(w)
	}
}

func (s *Drop) createGround3() {
	groundId := s.resetGround()

	shapeDef := b2.DefaultShapeDef()

	w := b2.F(dropGroundWidth)
	half := b2.QHalf().Mul(b2.F(dropGroundCount)).Mul(w)
	segment := b2.Segment{Point1: b2.Vec2{X: half.Neg()}, Point2: b2.Vec2{X: half}}
	b2.CreateSegmentShape(groundId, &shapeDef, &segment)
	segment = b2.Segment{Point1: b2.V2(3, 0), Point2: b2.V2(3, 8)}
	b2.CreateSegmentShape(groundId, &shapeDef, &segment)
}

// scene1 drops a ball.
func (s *Drop) scene1() {
	s.clear()
	s.createGround2()

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.Position = b2.V2(0, 4)
	bodyDef.LinearVelocity = b2.V2(0, -100)

	bodyId := b2.CreateBody(s.WorldId, &bodyDef)

	shapeDef := b2.DefaultShapeDef()
	circle := b2.Circle{Radius: b2.F(0.125)}
	b2.CreateCircleShape(bodyId, &shapeDef, &circle)

	s.bodyIds = append(s.bodyIds, bodyId)
	s.frameCount = 1
}

// scene2 drops a ruler.
func (s *Drop) scene2() {
	s.clear()
	s.createGround1()

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.Position = b2.V2(0, 4)
	bodyDef.Rotation = b2.MakeRot(b2.QFromRatio(1, 4))
	bodyDef.LinearVelocity = b2.V2(0, 0)
	bodyDef.AngularVelocity = radiansQToTurns(b2.F(-0.5))

	bodyId := b2.CreateBody(s.WorldId, &bodyDef)

	shapeDef := b2.DefaultShapeDef()
	box := b2.MakeBox(b2.F(0.75), b2.F(0.01))
	b2.CreatePolygonShape(bodyId, &shapeDef, &box)

	s.bodyIds = append(s.bodyIds, bodyId)
	s.frameCount = 1
}

// scene3 drops a ragdoll.
func (s *Drop) scene3() {
	s.clear()
	s.createGround2()

	jointFrictionTorque := b2.F(0.03)
	jointHertz := b2.QOne()
	jointDampingRatio := b2.QHalf()

	s.human = shared.CreateHuman(s.WorldId, b2.V2(0, 40), b2.QOne(), jointFrictionTorque, jointHertz, jointDampingRatio, 1, nil,
		true)

	s.frameCount = 1
}

// scene4 shoots a bullet into a stack.
func (s *Drop) scene4() {
	s.clear()
	s.createGround3()

	a := b2.F(0.25)
	box := b2.MakeSquare(a)

	shapeDef := b2.DefaultShapeDef()

	offset := b2.F(0.01)

	for i := 0; i < 5; i++ {
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody

		shift := offset
		if i%2 == 0 {
			shift = offset.Neg()
		}
		bodyDef.Position = b2.Vec2{X: b2.F(2.5).Add(shift), Y: a.Add(b2.F(2 * i).Mul(a))}
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)

		s.bodyIds = append(s.bodyIds, bodyId)
		b2.CreatePolygonShape(bodyId, &shapeDef, &box)
	}

	circle := b2.Circle{Radius: b2.F(0.125)}
	shapeDef.Density = b2.F(4)

	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.V2(-7.7, 1.9)
		bodyDef.LinearVelocity = b2.V2(200, 0)
		bodyDef.IsBullet = true

		bodyId := b2.CreateBody(s.WorldId, &bodyDef)
		b2.CreateCircleShape(bodyId, &shapeDef, &circle)
		s.bodyIds = append(s.bodyIds, bodyId)
	}

	s.frameCount = 1
}

func (s *Drop) Keyboard(key Key) {
	switch key {
	case Key1:
		s.scene1()

	case Key2:
		s.scene2()

	case Key3:
		s.scene3()

	case Key4:
		s.scene4()

	case KeyC:
		s.clear()
		s.continuous = !s.continuous

	case KeyV:
		s.clear()
		s.speculative = !s.speculative
		s.WorldId.EnableSpeculative(s.speculative)

	case KeyS:
		if s.frameSkip > 0 {
			s.frameSkip = 0
		} else {
			s.frameSkip = 60
		}

	default:
		s.Base.Keyboard(key)
	}
}

func (s *Drop) Step() {
	settings := &s.Context.Settings
	settings.EnableContinuous = s.continuous

	if (s.frameSkip == 0 || s.frameCount%s.frameSkip == 0) && !settings.Pause {
		s.Base.Step()
	} else {
		pause := settings.Pause
		settings.Pause = true
		s.Base.Step()
		settings.Pause = pause
	}

	s.frameCount += 1
}

// Pinball shows a fast moving body that uses continuous collision versus
// static and dynamic bodies. This is achieved by setting the ball body as
// a bullet.
type Pinball struct {
	Base

	leftJointId  b2.JointId
	rightJointId b2.JointId
	ballId       b2.BodyId
}

func NewPinball(ctx *SampleContext) Sample {
	s := &Pinball{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 9}
		ctx.Camera.Zoom = 25 * 0.5
	}

	ctx.Settings.DrawJoints = false

	// Ground body
	var groundId b2.BodyId
	{
		bodyDef := b2.DefaultBodyDef()
		groundId = b2.CreateBody(s.WorldId, &bodyDef)

		vs := []b2.Vec2{b2.V2(-8, 6), b2.V2(-8, 20), b2.V2(8, 20), b2.V2(8, 6), b2.V2(0, -2)}

		chainDef := b2.DefaultChainDef()
		chainDef.Points = vs
		chainDef.IsLoop = true
		b2.CreateChain(groundId, &chainDef)
	}

	// Flippers
	{
		p1, p2 := b2.V2(-2, 0), b2.V2(2, 0)

		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.EnableSleep = false

		bodyDef.Position = p1
		leftFlipperId := b2.CreateBody(s.WorldId, &bodyDef)

		bodyDef.Position = p2
		rightFlipperId := b2.CreateBody(s.WorldId, &bodyDef)

		box := b2.MakeBox(b2.F(1.75), b2.F(0.2))

		shapeDef := b2.DefaultShapeDef()

		b2.CreatePolygonShape(leftFlipperId, &shapeDef, &box)
		b2.CreatePolygonShape(rightFlipperId, &shapeDef, &box)

		jointDef := b2.DefaultRevoluteJointDef()
		jointDef.BodyIdA = groundId
		jointDef.LocalAnchorB = b2.Vec2{}
		jointDef.EnableMotor = true
		jointDef.MaxMotorTorque = b2.F(1000)
		jointDef.EnableLimit = true

		// 30 degrees is 1/12 turn and 5 degrees is 1/72 turn.
		jointDef.MotorSpeed = b2.QZero()
		jointDef.LocalAnchorA = p1
		jointDef.BodyIdB = leftFlipperId
		jointDef.LowerAngle = b2.QFromRatio(-1, 12)
		jointDef.UpperAngle = b2.QFromRatio(1, 72)
		s.leftJointId = b2.CreateRevoluteJoint(s.WorldId, &jointDef)

		jointDef.MotorSpeed = b2.QZero()
		jointDef.LocalAnchorA = p2
		jointDef.BodyIdB = rightFlipperId
		jointDef.LowerAngle = b2.QFromRatio(-1, 72)
		jointDef.UpperAngle = b2.QFromRatio(1, 12)
		s.rightJointId = b2.CreateRevoluteJoint(s.WorldId, &jointDef)
	}

	// Spinners
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.V2(-4, 17)

		bodyId := b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		box1 := b2.MakeBox(b2.F(1.5), b2.F(0.125))
		box2 := b2.MakeBox(b2.F(0.125), b2.F(1.5))

		b2.CreatePolygonShape(bodyId, &shapeDef, &box1)
		b2.CreatePolygonShape(bodyId, &shapeDef, &box2)

		jointDef := b2.DefaultRevoluteJointDef()
		jointDef.BodyIdA = groundId
		jointDef.BodyIdB = bodyId
		jointDef.LocalAnchorA = bodyDef.Position
		jointDef.LocalAnchorB = b2.Vec2{}
		jointDef.EnableMotor = true
		jointDef.MaxMotorTorque = b2.F(0.1)
		b2.CreateRevoluteJoint(s.WorldId, &jointDef)

		bodyDef.Position = b2.V2(4, 8)
		bodyId = b2.CreateBody(s.WorldId, &bodyDef)
		b2.CreatePolygonShape(bodyId, &shapeDef, &box1)
		b2.CreatePolygonShape(bodyId, &shapeDef, &box2)
		jointDef.LocalAnchorA = bodyDef.Position
		jointDef.BodyIdB = bodyId
		b2.CreateRevoluteJoint(s.WorldId, &jointDef)
	}

	// Bumpers
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Position = b2.V2(-4, 8)

		bodyId := b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		shapeDef.Material.Restitution = b2.F(1.5)

		circle := b2.Circle{Radius: b2.QOne()}
		b2.CreateCircleShape(bodyId, &shapeDef, &circle)

		bodyDef.Position = b2.V2(4, 17)
		bodyId = b2.CreateBody(s.WorldId, &bodyDef)
		b2.CreateCircleShape(bodyId, &shapeDef, &circle)
	}

	// Ball
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Position = b2.V2(1, 15)
		bodyDef.Type = b2.DynamicBody
		bodyDef.IsBullet = true

		s.ballId = b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		circle := b2.Circle{Radius: b2.F(0.2)}
		b2.CreateCircleShape(s.ballId, &shapeDef, &circle)
	}
	return s
}

func (s *Pinball) Step() {
	s.Base.Step()

	if s.keyDown(KeySpace) {
		s.leftJointId.SetMotorSpeed(radiansToTurns(20))
		s.rightJointId.SetMotorSpeed(radiansToTurns(-20))
	} else {
		s.leftJointId.SetMotorSpeed(radiansToTurns(-10))
		s.rightJointId.SetMotorSpeed(radiansToTurns(10))
	}
}

// Wedge shows the importance of secondary collisions in continuous
// physics. This also shows a difficult setup for the solver with an acute
// angle.
type Wedge struct {
	Base
}

func NewWedge(ctx *SampleContext) Sample {
	s := &Wedge{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 5.5}
		ctx.Camera.Zoom = 6
	}

	{
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(s.WorldId, &bodyDef)
		shapeDef := b2.DefaultShapeDef()
		segment := b2.Segment{Point1: b2.V2(-4, 8), Point2: b2.V2(0, 0)}
		b2.CreateSegmentShape(groundId, &shapeDef, &segment)
		segment = b2.Segment{Point1: b2.V2(0, 0), Point2: b2.V2(0, 8)}
		b2.CreateSegmentShape(groundId, &shapeDef, &segment)
	}

	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.V2(-0.45, 10.75)
		bodyDef.LinearVelocity = b2.V2(0, -200)

		bodyId := b2.CreateBody(s.WorldId, &bodyDef)

		circle := b2.Circle{Radius: b2.F(0.3)}
		shapeDef := b2.DefaultShapeDef()
		shapeDef.Material.Friction = b2.F(0.2)
		b2.CreateCircleShape(bodyId, &shapeDef, &circle)
	}
	return s
}
