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
func radiansQToTurns(radians dbox2d.Q) dbox2d.Q {
	return radians.Div(dbox2d.QFromInt(2).Mul(dbox2d.Pi()))
}

// createBoxWalls adds the four walls of a 20 by 20 room centered on the
// origin to the body, as Bounce House and Bounce Humans do.
func createBoxWalls(bodyId dbox2d.BodyId, shapeDef *dbox2d.ShapeDef) {
	corners := [4]dbox2d.Vec2{qv("-10", "-10"), qv("10", "-10"), qv("10", "10"), qv("-10", "10")}
	for i := range corners {
		segment := dbox2d.Segment{Point1: corners[i], Point2: corners[(i+1)%4]}
		dbox2d.CreateSegmentShape(bodyId, shapeDef, &segment)
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
	point     dbox2d.Vec2
	speed     dbox2d.Q
	stepIndex int
}

// BounceHouse bounces a fast body inside a closed room.
type BounceHouse struct {
	Base

	hitEvents       [4]bounceHitEvent
	bodyId          dbox2d.BodyId
	shapeType       int
	enableHitEvents bool
}

func NewBounceHouse(ctx *SampleContext) Sample {
	s := &BounceHouse{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 0}
		ctx.Camera.Zoom = 25 * 0.45
	}

	bodyDef := dbox2d.DefaultBodyDef()
	groundId := dbox2d.CreateBody(s.WorldId, &bodyDef)

	shapeDef := dbox2d.DefaultShapeDef()
	createBoxWalls(groundId, &shapeDef)

	s.shapeType = launchBox
	s.bodyId = dbox2d.BodyId{}
	s.enableHitEvents = true

	s.launch()
	return s
}

func (s *BounceHouse) launch() {
	if !s.bodyId.IsNull() {
		dbox2d.DestroyBody(s.bodyId)
	}

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	bodyDef.LinearVelocity = qv("10", "20")
	bodyDef.Position = qv("0", "0")
	bodyDef.GravityScale = dbox2d.QZero()

	// Circle shapes centered on the body can spin fast without risk of tunnelling.
	bodyDef.AllowFastRotation = s.shapeType == launchCircle

	s.bodyId = dbox2d.CreateBody(s.WorldId, &bodyDef)

	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Density = dbox2d.QOne()
	shapeDef.Material.Restitution = qs("1.2")
	shapeDef.Material.Friction = qs("0.3")
	shapeDef.EnableHitEvents = s.enableHitEvents

	if s.shapeType == launchCircle {
		circle := dbox2d.Circle{Radius: dbox2d.QHalf()}
		dbox2d.CreateCircleShape(s.bodyId, &shapeDef, &circle)
	} else if s.shapeType == launchCapsule {
		capsule := dbox2d.Capsule{Center1: qv("-0.5", "0"), Center2: qv("0.5", "0"), Radius: qs("0.25")}
		dbox2d.CreateCapsuleShape(s.bodyId, &shapeDef, &capsule)
	} else {
		h := qs("0.1")
		box := dbox2d.MakeBox(dbox2d.QFromInt(20).Mul(h), h)
		dbox2d.CreatePolygonShape(s.bodyId, &shapeDef, &box)
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
			draw.DrawCircle(e.point, qs("0.1"), dbox2d.ColorOrangeRed)
			draw.DrawString(e.point, fmt.Sprintf("%.1f", ToFloat64(e.speed)), dbox2d.ColorWhite)
		}
	}
}

// BounceHumans bounces ragdolls in a room whose gravity turns.
type BounceHumans struct {
	Base

	humans     [5]shared.Human
	humanCount int
	countDown  dbox2d.Q
	time       dbox2d.Q
}

func NewBounceHumans(ctx *SampleContext) Sample {
	s := &BounceHumans{Base: NewBase(ctx)}
	ctx.Camera.Center = Vec2f{X: 0, Y: 0}
	ctx.Camera.Zoom = 12

	bodyDef := dbox2d.DefaultBodyDef()
	groundId := dbox2d.CreateBody(s.WorldId, &bodyDef)

	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Material.Restitution = qs("1.3")
	shapeDef.Material.Friction = qs("0.1")

	createBoxWalls(groundId, &shapeDef)

	circle := dbox2d.Circle{Radius: dbox2d.QFromInt(2)}
	shapeDef.Material.Restitution = dbox2d.QFromInt(2)
	dbox2d.CreateCircleShape(groundId, &shapeDef, &circle)
	return s
}

func (s *BounceHumans) Step() {
	if s.humanCount < 5 && !s.countDown.Greater(dbox2d.QZero()) {
		jointFrictionTorque := dbox2d.QZero()
		jointHertz := dbox2d.QOne()
		jointDampingRatio := qs("0.1")

		s.humans[s.humanCount] = shared.CreateHuman(s.WorldId, qv("0", "5"), dbox2d.QOne(), jointFrictionTorque, jointHertz,
			jointDampingRatio, 1, nil, true)

		s.countDown = dbox2d.QFromInt(2)
		s.humanCount += 1
	}

	timeStep := dbox2d.QFromRatio(1, 60)
	cs1 := dbox2d.MakeRot(radiansQToTurns(dbox2d.QHalf().Mul(s.time)))
	cs2 := dbox2d.MakeRot(radiansQToTurns(s.time))
	gravity := dbox2d.QFromInt(10)
	gravityVec := dbox2d.Vec2{X: gravity.Mul(cs1.Sin), Y: gravity.Mul(cs2.Cos)}
	three := dbox2d.QFromInt(3)
	s.Context.Draw.DrawSegment(dbox2d.Vec2{}, dbox2d.Vec2{X: three.Mul(cs1.Sin), Y: three.Mul(cs2.Cos)}, dbox2d.ColorWhite)
	s.time = s.time.Add(timeStep)
	s.countDown = s.countDown.Sub(timeStep)
	s.WorldId.SetGravity(gravityVec)

	s.Base.Step()
}

// ChainDrop drops a fast circle on a closed chain.
type ChainDrop struct {
	Base

	bodyId  dbox2d.BodyId
	shapeId dbox2d.ShapeId
	yOffset float64
	speed   float64
}

func NewChainDrop(ctx *SampleContext) Sample {
	s := &ChainDrop{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 0}
		ctx.Camera.Zoom = 25 * 0.35
	}

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Position = qv("0", "-6")
	groundId := dbox2d.CreateBody(s.WorldId, &bodyDef)

	points := []dbox2d.Vec2{qv("-10", "-2"), qv("10", "-2"), qv("10", "1"), qv("-10", "1")}

	chainDef := dbox2d.DefaultChainDef()
	chainDef.Points = points
	chainDef.IsLoop = true

	dbox2d.CreateChain(groundId, &chainDef)

	s.bodyId = dbox2d.BodyId{}
	s.yOffset = -0.1
	s.speed = -42

	s.launch()
	return s
}

func (s *ChainDrop) launch() {
	if !s.bodyId.IsNull() {
		dbox2d.DestroyBody(s.bodyId)
	}

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	bodyDef.LinearVelocity = dbox2d.Vec2{Y: FromFloat64(s.speed)}
	bodyDef.Position = dbox2d.Vec2{Y: dbox2d.QFromInt(10).Add(FromFloat64(s.yOffset))}
	bodyDef.Rotation = dbox2d.MakeRot(dbox2d.QFromRatio(1, 4))
	bodyDef.FixedRotation = true
	s.bodyId = dbox2d.CreateBody(s.WorldId, &bodyDef)

	shapeDef := dbox2d.DefaultShapeDef()

	circle := dbox2d.Circle{Radius: dbox2d.QHalf()}
	s.shapeId = dbox2d.CreateCircleShape(s.bodyId, &shapeDef, &circle)
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
		bodyDef := dbox2d.DefaultBodyDef()
		groundId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		const count = 80
		points := make([]dbox2d.Vec2, count)

		w := dbox2d.QFromInt(2)
		h := dbox2d.QOne()
		x, y := dbox2d.QFromInt(20), dbox2d.QZero()
		for i := 0; i < 20; i++ {
			points[i] = dbox2d.Vec2{X: x, Y: y}
			x = x.Sub(w)
		}

		for i := 20; i < 40; i++ {
			points[i] = dbox2d.Vec2{X: x, Y: y}
			y = y.Add(h)
		}

		for i := 40; i < 60; i++ {
			points[i] = dbox2d.Vec2{X: x, Y: y}
			x = x.Add(w)
		}

		for i := 60; i < 80; i++ {
			points[i] = dbox2d.Vec2{X: x, Y: y}
			y = y.Sub(h)
		}

		chainDef := dbox2d.DefaultChainDef()
		chainDef.Points = points
		chainDef.IsLoop = true

		dbox2d.CreateChain(groundId, &chainDef)
	}

	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.LinearVelocity = qv("100", "0")
		bodyDef.Position = qv("-19.5", "0.5")
		bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		shapeDef := dbox2d.DefaultShapeDef()
		shapeDef.Material.Friction = dbox2d.QZero()
		circle := dbox2d.Circle{Radius: dbox2d.QHalf()}
		dbox2d.CreateCircleShape(bodyId, &shapeDef, &circle)
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
		bodyDef := dbox2d.DefaultBodyDef()
		groundId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		shapeDef := dbox2d.DefaultShapeDef()
		segment := dbox2d.Segment{Point1: qv("-40", "0"), Point2: qv("40", "0")}
		dbox2d.CreateSegmentShape(groundId, &shapeDef, &segment)

		segment = dbox2d.Segment{Point1: qv("40", "0"), Point2: qv("40", "10")}
		dbox2d.CreateSegmentShape(groundId, &shapeDef, &segment)
	}

	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.LinearVelocity = qv("100", "0")
		bodyDef.Position = qv("-20", "0.7")
		bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		shapeDef := dbox2d.DefaultShapeDef()
		circle := dbox2d.Circle{Radius: dbox2d.QHalf()}
		dbox2d.CreateCircleShape(bodyId, &shapeDef, &circle)
	}
	return s
}

// SkinnyBox drops a fast spinning thin box on a thin post.
type SkinnyBox struct {
	Base

	bodyId, bulletId dbox2d.BodyId
	angularVelocity  dbox2d.Q
	x                dbox2d.Q
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
		bodyDef := dbox2d.DefaultBodyDef()
		groundId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		segment := dbox2d.Segment{Point1: qv("-10", "0"), Point2: qv("10", "0")}
		shapeDef := dbox2d.DefaultShapeDef()
		shapeDef.Material.Friction = qs("0.9")
		dbox2d.CreateSegmentShape(groundId, &shapeDef, &segment)

		box := dbox2d.MakeOffsetBox(qs("0.1"), dbox2d.QOne(), qv("0", "1"), dbox2d.RotIdentity())
		dbox2d.CreatePolygonShape(groundId, &shapeDef, &box)
	}

	s.autoTest = false
	s.bullet = false
	s.capsule = false

	s.bodyId = dbox2d.BodyId{}
	s.bulletId = dbox2d.BodyId{}

	s.launch()
	return s
}

func (s *SkinnyBox) launch() {
	if !s.bodyId.IsNull() {
		dbox2d.DestroyBody(s.bodyId)
	}

	if !s.bulletId.IsNull() {
		dbox2d.DestroyBody(s.bulletId)
	}

	// The reference draws radians per second; the body takes turns.
	s.angularVelocity = shared.RandomFloatRange(dbox2d.QFromInt(-50), dbox2d.QFromInt(50))

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	bodyDef.Position = qv("0", "8")
	bodyDef.AngularVelocity = radiansQToTurns(s.angularVelocity)
	bodyDef.LinearVelocity = qv("0", "-100")

	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Density = dbox2d.QOne()
	shapeDef.Material.Friction = qs("0.9")

	s.bodyId = dbox2d.CreateBody(s.WorldId, &bodyDef)

	if s.capsule {
		capsule := dbox2d.Capsule{Center1: qv("0", "-1"), Center2: qv("0", "1"), Radius: qs("0.1")}
		dbox2d.CreateCapsuleShape(s.bodyId, &shapeDef, &capsule)
	} else {
		polygon := dbox2d.MakeBox(dbox2d.QFromInt(2), qs("0.05"))
		dbox2d.CreatePolygonShape(s.bodyId, &shapeDef, &polygon)
	}

	if s.bullet {
		polygon := dbox2d.MakeBox(qs("0.25"), qs("0.25"))
		s.x = shared.RandomFloatRange(dbox2d.QFromInt(-1), dbox2d.QOne())
		bodyDef.Position = dbox2d.Vec2{X: s.x, Y: dbox2d.QFromInt(10)}
		bodyDef.LinearVelocity = qv("0", "-50")
		s.bulletId = dbox2d.CreateBody(s.WorldId, &bodyDef)
		dbox2d.CreatePolygonShape(s.bulletId, &shapeDef, &polygon)
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

	groundId  dbox2d.BodyId
	bodyId    dbox2d.BodyId
	shapeId   dbox2d.ShapeId
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

	s.groundId = dbox2d.BodyId{}
	s.bodyId = dbox2d.BodyId{}
	s.shapeId = dbox2d.ShapeId{}
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
		dbox2d.DestroyBody(s.groundId)
	}

	s.shapeId = dbox2d.ShapeId{}

	bodyDef := dbox2d.DefaultBodyDef()
	s.groundId = dbox2d.CreateBody(s.WorldId, &bodyDef)

	one := dbox2d.QOne()
	two := dbox2d.QFromInt(2)
	sqrt2 := two.Sqrt()
	m := one.Div(sqrt2)
	mm := two.Mul(sqrt2.Sub(one))
	hx, hy := dbox2d.QFromInt(4), qs("0.25")
	friction := FromFloat64(s.friction)

	if s.useChain {
		// step returns p moved by (x, y).
		step := func(p dbox2d.Vec2, x, y dbox2d.Q) dbox2d.Vec2 { return p.Add(dbox2d.Vec2{X: x, Y: y}) }

		hxm := two.Mul(hx).Mul(m)
		hym := two.Mul(hy).Mul(m)
		corner := hxm.Add(two.Mul(hy).Mul(one.Sub(m)))
		flat := two.Mul(hx)
		flatBevel := flat.Add(hy.Mul(mm))

		var points [20]dbox2d.Vec2
		points[0] = dbox2d.Vec2{X: dbox2d.QFromInt(-3).Mul(hx), Y: hy}
		points[1] = step(points[0], hxm.Neg(), hxm)
		points[2] = step(points[1], hxm.Neg(), hxm)
		points[3] = step(points[2], hxm.Neg(), hxm)
		points[4] = step(points[3], hym.Neg(), hym.Neg())
		points[5] = step(points[4], hxm, hxm.Neg())
		points[6] = step(points[5], hxm, hxm.Neg())
		points[7] = step(points[6], corner, corner.Neg())
		points[8] = step(points[7], flatBevel, dbox2d.QZero())
		points[9] = step(points[8], flat, dbox2d.QZero())
		points[10] = step(points[9], flatBevel, dbox2d.QZero())
		points[11] = step(points[10], corner, corner)
		points[12] = step(points[11], hxm, hxm)
		points[13] = step(points[12], hxm, hxm)
		points[14] = step(points[13], hym.Neg(), hym)
		points[15] = step(points[14], hxm.Neg(), hxm.Neg())
		points[16] = step(points[15], hxm.Neg(), hxm.Neg())
		points[17] = step(points[16], hxm.Neg(), hxm.Neg())
		points[18] = step(points[17], flat.Neg(), dbox2d.QZero())
		points[19] = step(points[18], flat.Neg(), dbox2d.QZero())

		material := dbox2d.SurfaceMaterial{Friction: friction}

		chainDef := dbox2d.DefaultChainDef()
		chainDef.Points = points[:]
		chainDef.IsLoop = true
		chainDef.Materials = []dbox2d.SurfaceMaterial{material}

		dbox2d.CreateChain(s.groundId, &chainDef)
	} else {
		shapeDef := dbox2d.DefaultShapeDef()
		shapeDef.Material.Friction = friction

		var hull dbox2d.Hull

		if s.bevel > 0 {
			hb := FromFloat64(s.bevel)
			d := qs("0.05")
			vs := []dbox2d.Vec2{
				{X: hx.Add(hb), Y: hy.Sub(d)}, {X: hx, Y: hy}, {X: hx.Neg(), Y: hy}, {X: hx.Neg().Sub(hb), Y: hy.Sub(d)},
				{X: hx.Neg().Sub(hb), Y: hy.Neg().Add(d)}, {X: hx.Neg(), Y: hy.Neg()}, {X: hx, Y: hy.Neg()}, {X: hx.Add(hb), Y: hy.Neg().Add(d)},
			}
			hull = dbox2d.ComputeHull(vs)
		} else {
			vs := []dbox2d.Vec2{{X: hx, Y: hy}, {X: hx.Neg(), Y: hy}, {X: hx.Neg(), Y: hy.Neg()}, {X: hx, Y: hy.Neg()}}
			hull = dbox2d.ComputeHull(vs)
		}

		// place adds three polygons along (dx, dy) from (x, y).
		place := func(x, y, dx, dy dbox2d.Q, q dbox2d.Rot) {
			for range 3 {
				polygon := dbox2d.MakeOffsetPolygon(&hull, dbox2d.Vec2{X: x, Y: y}, q)
				dbox2d.CreatePolygonShape(s.groundId, &shapeDef, &polygon)
				x = x.Add(dx)
				y = y.Add(dy)
			}
		}

		slopeStep := two.Mul(m).Mul(hx)
		slopeY := hy.Add(m.Mul(hx)).Sub(m.Mul(hy))
		slopeX := dbox2d.QFromInt(3).Mul(hx).Add(m.Mul(hx)).Add(m.Mul(hy))

		// Left slope
		place(slopeX.Neg(), slopeY, slopeStep.Neg(), slopeStep, dbox2d.MakeRot(dbox2d.QFromRatio(-1, 8)))

		place(two.Mul(hx).Neg(), dbox2d.QZero(), two.Mul(hx), dbox2d.QZero(), dbox2d.MakeRot(dbox2d.QZero()))

		place(slopeX, slopeY, slopeStep, slopeStep, dbox2d.MakeRot(dbox2d.QFromRatio(1, 8)))
	}
}

func (s *GhostBumps) launch() {
	if !s.bodyId.IsNull() {
		dbox2d.DestroyBody(s.bodyId)
		s.shapeId = dbox2d.ShapeId{}
	}

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	bodyDef.Position = qv("-28", "18")
	bodyDef.LinearVelocity = qv("0", "0")
	s.bodyId = dbox2d.CreateBody(s.WorldId, &bodyDef)

	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Density = dbox2d.QOne()
	shapeDef.Material.Friction = FromFloat64(s.friction)

	if s.shapeType == launchCircle {
		circle := dbox2d.Circle{Radius: dbox2d.QHalf()}
		s.shapeId = dbox2d.CreateCircleShape(s.bodyId, &shapeDef, &circle)
	} else if s.shapeType == launchCapsule {
		capsule := dbox2d.Capsule{Center1: qv("-0.5", "0"), Center2: qv("0.5", "0"), Radius: qs("0.25")}
		s.shapeId = dbox2d.CreateCapsuleShape(s.bodyId, &shapeDef, &capsule)
	} else {
		round := FromFloat64(s.round)
		h := dbox2d.QHalf().Sub(round)
		box := dbox2d.MakeRoundedBox(h, dbox2d.QFromInt(2).Mul(h), round)
		s.shapeId = dbox2d.CreatePolygonShape(s.bodyId, &shapeDef, &box)
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
		bodyDef := dbox2d.DefaultBodyDef()
		groundId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		shapeDef := dbox2d.DefaultShapeDef()
		segment := dbox2d.Segment{Point1: qv("-10", "0"), Point2: qv("10", "0")}
		dbox2d.CreateSegmentShape(groundId, &shapeDef, &segment)

		points := []dbox2d.Vec2{qv("-2", "4"), qv("2", "4"), qv("2", "4.1"), qv("-0.5", "4.2"), qv("-2", "4.2")}
		hull := dbox2d.ComputeHull(points)
		poly := dbox2d.MakePolygon(&hull, dbox2d.QZero())
		dbox2d.CreatePolygonShape(groundId, &shapeDef, &poly)
	}

	// Fast moving skinny box. Also testing a large shape offset.
	{
		offset := dbox2d.QFromInt(8)
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = dbox2d.Vec2{X: offset, Y: dbox2d.QFromInt(12)}
		bodyDef.LinearVelocity = qv("0", "-100")
		bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		shapeDef := dbox2d.DefaultShapeDef()
		box := dbox2d.MakeOffsetBox(dbox2d.QFromInt(2), qs("0.05"), dbox2d.Vec2{X: offset.Neg()}, dbox2d.MakeRot(dbox2d.QHalf()))
		dbox2d.CreatePolygonShape(bodyId, &shapeDef, &box)
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
		bodyDef := dbox2d.DefaultBodyDef()
		groundId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		shapeDef := dbox2d.DefaultShapeDef()
		segment := dbox2d.Segment{Point1: qv("-10", "0"), Point2: qv("10", "0")}
		dbox2d.CreateSegmentShape(groundId, &shapeDef, &segment)
	}

	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = qv("0", "12")
		bodyDef.LinearVelocity = qv("0", "-100")
		bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		shapeDef := dbox2d.DefaultShapeDef()
		points := []dbox2d.Vec2{qv("-2", "0"), qv("-1", "0"), qv("2", "0.5")}
		hull := dbox2d.ComputeHull(points)
		poly := dbox2d.MakePolygon(&hull, dbox2d.QZero())
		dbox2d.CreatePolygonShape(bodyId, &shapeDef, &poly)
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
		bodyDef := dbox2d.DefaultBodyDef()
		groundId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		shapeDef := dbox2d.DefaultShapeDef()
		segment := dbox2d.Segment{Point1: qv("-10", "0"), Point2: qv("10", "0")}
		dbox2d.CreateSegmentShape(groundId, &shapeDef, &segment)

		box := dbox2d.MakeOffsetBox(dbox2d.QOne(), qs("0.1"), qv("0", "0.9"), dbox2d.RotIdentity())
		dbox2d.CreatePolygonShape(groundId, &shapeDef, &box)
	}

	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody

		// The speculative distance is 0.02 meters, so this avoid it
		bodyDef.Position = qv("0.015", "2.515")
		speed := qs("0.125").Mul(FromFloat64(ctx.Settings.Hertz))
		bodyDef.LinearVelocity = dbox2d.Vec2{X: speed, Y: speed.Neg()}
		bodyDef.GravityScale = dbox2d.QZero()
		bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		shapeDef := dbox2d.DefaultShapeDef()
		box := dbox2d.MakeSquare(qs("0.25"))
		dbox2d.CreatePolygonShape(bodyId, &shapeDef, &box)
	}
	return s
}

// pixels converts a pixel measure of Pixel Imperfect and Restitution
// Threshold to meters, at 30 pixels per meter.
func pixels(n int) dbox2d.Q { return dbox2d.QFromRatio(n, 30) }

// PixelImperfect shows that Box2D does not have pixel perfect collision.
type PixelImperfect struct {
	Base

	ballId dbox2d.BodyId
}

func NewPixelImperfect(ctx *SampleContext) Sample {
	s := &PixelImperfect{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 7, Y: 5}
		ctx.Camera.Zoom = 6
	}

	{
		block4BodyDef := dbox2d.DefaultBodyDef()
		block4BodyDef.Type = dbox2d.StaticBody
		block4BodyDef.Position = dbox2d.Vec2{X: pixels(175), Y: pixels(150)}
		block4BodyId := dbox2d.CreateBody(s.WorldId, &block4BodyDef)
		block4Shape := dbox2d.MakeBox(pixels(20), pixels(10))
		block4ShapeDef := dbox2d.DefaultShapeDef()
		block4ShapeDef.Material.Friction = dbox2d.QZero()
		dbox2d.CreatePolygonShape(block4BodyId, &block4ShapeDef, &block4Shape)
	}

	{
		ballBodyDef := dbox2d.DefaultBodyDef()
		ballBodyDef.Type = dbox2d.DynamicBody
		ballBodyDef.Position = dbox2d.Vec2{X: pixels(200), Y: pixels(275)}
		ballBodyDef.GravityScale = dbox2d.QZero()

		s.ballId = dbox2d.CreateBody(s.WorldId, &ballBodyDef)
		// Ball shape
		ballShape := dbox2d.MakeRoundedBox(pixels(4), pixels(4), qs("0.9").Div(dbox2d.QFromInt(30)))
		ballShapeDef := dbox2d.DefaultShapeDef()
		ballShapeDef.Material.Friction = dbox2d.QZero()
		dbox2d.CreatePolygonShape(s.ballId, &ballShapeDef, &ballShape)
		s.ballId.SetLinearVelocity(qv("0", "-5"))
		s.ballId.SetFixedRotation(true)
	}
	return s
}

func (s *PixelImperfect) Step() {
	var data [1]dbox2d.ContactData
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

	ballId dbox2d.BodyId
}

func NewRestitutionThreshold(ctx *SampleContext) Sample {
	s := &RestitutionThreshold{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 7, Y: 5}
		ctx.Camera.Zoom = 6
	}

	// With the default threshold the ball will not bounce.
	s.WorldId.SetRestitutionThreshold(qs("0.1"))

	{
		block0BodyDef := dbox2d.DefaultBodyDef()
		block0BodyDef.Type = dbox2d.StaticBody
		block0BodyDef.Position = dbox2d.Vec2{X: pixels(205), Y: pixels(120)}
		// The reference writes 70 degrees with pi as 3.14.
		block0BodyDef.Rotation = dbox2d.MakeRot(radiansToTurns(70 * 3.14 / 180))
		block0BodyId := dbox2d.CreateBody(s.WorldId, &block0BodyDef)
		block0Shape := dbox2d.MakeBox(pixels(50), pixels(5))
		block0ShapeDef := dbox2d.DefaultShapeDef()
		block0ShapeDef.Material.Friction = dbox2d.QZero()
		dbox2d.CreatePolygonShape(block0BodyId, &block0ShapeDef, &block0Shape)
	}

	{
		// Make a ball
		ballBodyDef := dbox2d.DefaultBodyDef()
		ballBodyDef.Type = dbox2d.DynamicBody
		ballBodyDef.Position = dbox2d.Vec2{X: pixels(200), Y: pixels(250)}
		s.ballId = dbox2d.CreateBody(s.WorldId, &ballBodyDef)

		ballShape := dbox2d.Circle{Radius: pixels(5)}
		ballShapeDef := dbox2d.DefaultShapeDef()
		ballShapeDef.Material.Friction = dbox2d.QZero()
		ballShapeDef.Material.Restitution = dbox2d.QOne()
		dbox2d.CreateCircleShape(s.ballId, &ballShapeDef, &ballShape)

		s.ballId.SetLinearVelocity(qv("0", "-2.9")) // Initial velocity
		s.ballId.SetFixedRotation(true)             // Do not rotate a ball
	}
	return s
}

func (s *RestitutionThreshold) Step() {
	var data [1]dbox2d.ContactData
	s.ballId.GetContactData(data[:])

	p := s.ballId.GetPosition()
	v := s.ballId.GetLinearVelocity()
	s.DrawTextLine("p.x = %.9f, v.y = %.9f", ToFloat64(p.X), ToFloat64(v.Y))

	s.Base.Step()
}

// Drop switches between four drop scenes with the number keys.
type Drop struct {
	Base

	groundIds   []dbox2d.BodyId
	bodyIds     []dbox2d.BodyId
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
		dbox2d.DestroyBody(bodyId)
	}

	s.bodyIds = s.bodyIds[:0]

	if s.human.IsSpawned {
		s.human.Destroy()
	}
}

// resetGround destroys the ground bodies and creates a fresh one.
func (s *Drop) resetGround() dbox2d.BodyId {
	for _, groundId := range s.groundIds {
		dbox2d.DestroyBody(groundId)
	}
	s.groundIds = s.groundIds[:0]

	bodyDef := dbox2d.DefaultBodyDef()
	groundId := dbox2d.CreateBody(s.WorldId, &bodyDef)
	s.groundIds = append(s.groundIds, groundId)
	return groundId
}

const (
	dropGroundWidth = "0.25"
	dropGroundCount = 40
)

func (s *Drop) createGround1() {
	groundId := s.resetGround()

	shapeDef := dbox2d.DefaultShapeDef()

	w := qs(dropGroundWidth)
	half := dbox2d.QHalf().Mul(dbox2d.QFromInt(dropGroundCount)).Mul(w)
	segment := dbox2d.Segment{Point1: dbox2d.Vec2{X: half.Neg()}, Point2: dbox2d.Vec2{X: half}}
	dbox2d.CreateSegmentShape(groundId, &shapeDef, &segment)
}

func (s *Drop) createGround2() {
	groundId := s.resetGround()

	shapeDef := dbox2d.DefaultShapeDef()

	w := qs(dropGroundWidth)

	x := dbox2d.QHalf().Mul(dbox2d.QFromInt(dropGroundCount)).Mul(w).Neg()
	h := qs("0.05")
	for j := 0; j <= dropGroundCount; j++ {
		box := dbox2d.MakeOffsetBox(dbox2d.QHalf().Mul(w), h, dbox2d.Vec2{X: x}, dbox2d.RotIdentity())
		dbox2d.CreatePolygonShape(groundId, &shapeDef, &box)
		x = x.Add(w)
	}
}

func (s *Drop) createGround3() {
	groundId := s.resetGround()

	shapeDef := dbox2d.DefaultShapeDef()

	w := qs(dropGroundWidth)
	half := dbox2d.QHalf().Mul(dbox2d.QFromInt(dropGroundCount)).Mul(w)
	segment := dbox2d.Segment{Point1: dbox2d.Vec2{X: half.Neg()}, Point2: dbox2d.Vec2{X: half}}
	dbox2d.CreateSegmentShape(groundId, &shapeDef, &segment)
	segment = dbox2d.Segment{Point1: qv("3", "0"), Point2: qv("3", "8")}
	dbox2d.CreateSegmentShape(groundId, &shapeDef, &segment)
}

// scene1 drops a ball.
func (s *Drop) scene1() {
	s.clear()
	s.createGround2()

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	bodyDef.Position = qv("0", "4")
	bodyDef.LinearVelocity = qv("0", "-100")

	bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)

	shapeDef := dbox2d.DefaultShapeDef()
	circle := dbox2d.Circle{Radius: qs("0.125")}
	dbox2d.CreateCircleShape(bodyId, &shapeDef, &circle)

	s.bodyIds = append(s.bodyIds, bodyId)
	s.frameCount = 1
}

// scene2 drops a ruler.
func (s *Drop) scene2() {
	s.clear()
	s.createGround1()

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	bodyDef.Position = qv("0", "4")
	bodyDef.Rotation = dbox2d.MakeRot(dbox2d.QFromRatio(1, 4))
	bodyDef.LinearVelocity = qv("0", "0")
	bodyDef.AngularVelocity = radiansQToTurns(qs("-0.5"))

	bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)

	shapeDef := dbox2d.DefaultShapeDef()
	box := dbox2d.MakeBox(qs("0.75"), qs("0.01"))
	dbox2d.CreatePolygonShape(bodyId, &shapeDef, &box)

	s.bodyIds = append(s.bodyIds, bodyId)
	s.frameCount = 1
}

// scene3 drops a ragdoll.
func (s *Drop) scene3() {
	s.clear()
	s.createGround2()

	jointFrictionTorque := qs("0.03")
	jointHertz := dbox2d.QOne()
	jointDampingRatio := dbox2d.QHalf()

	s.human = shared.CreateHuman(s.WorldId, qv("0", "40"), dbox2d.QOne(), jointFrictionTorque, jointHertz, jointDampingRatio, 1, nil,
		true)

	s.frameCount = 1
}

// scene4 shoots a bullet into a stack.
func (s *Drop) scene4() {
	s.clear()
	s.createGround3()

	a := qs("0.25")
	box := dbox2d.MakeSquare(a)

	shapeDef := dbox2d.DefaultShapeDef()

	offset := qs("0.01")

	for i := 0; i < 5; i++ {
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody

		shift := offset
		if i%2 == 0 {
			shift = offset.Neg()
		}
		bodyDef.Position = dbox2d.Vec2{X: qs("2.5").Add(shift), Y: a.Add(dbox2d.QFromInt(2 * i).Mul(a))}
		bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		s.bodyIds = append(s.bodyIds, bodyId)
		dbox2d.CreatePolygonShape(bodyId, &shapeDef, &box)
	}

	circle := dbox2d.Circle{Radius: qs("0.125")}
	shapeDef.Density = dbox2d.QFromInt(4)

	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = qv("-7.7", "1.9")
		bodyDef.LinearVelocity = qv("200", "0")
		bodyDef.IsBullet = true

		bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)
		dbox2d.CreateCircleShape(bodyId, &shapeDef, &circle)
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

	leftJointId  dbox2d.JointId
	rightJointId dbox2d.JointId
	ballId       dbox2d.BodyId
}

func NewPinball(ctx *SampleContext) Sample {
	s := &Pinball{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 9}
		ctx.Camera.Zoom = 25 * 0.5
	}

	ctx.Settings.DrawJoints = false

	// Ground body
	var groundId dbox2d.BodyId
	{
		bodyDef := dbox2d.DefaultBodyDef()
		groundId = dbox2d.CreateBody(s.WorldId, &bodyDef)

		vs := []dbox2d.Vec2{qv("-8", "6"), qv("-8", "20"), qv("8", "20"), qv("8", "6"), qv("0", "-2")}

		chainDef := dbox2d.DefaultChainDef()
		chainDef.Points = vs
		chainDef.IsLoop = true
		dbox2d.CreateChain(groundId, &chainDef)
	}

	// Flippers
	{
		p1, p2 := qv("-2", "0"), qv("2", "0")

		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.EnableSleep = false

		bodyDef.Position = p1
		leftFlipperId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		bodyDef.Position = p2
		rightFlipperId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		box := dbox2d.MakeBox(qs("1.75"), qs("0.2"))

		shapeDef := dbox2d.DefaultShapeDef()

		dbox2d.CreatePolygonShape(leftFlipperId, &shapeDef, &box)
		dbox2d.CreatePolygonShape(rightFlipperId, &shapeDef, &box)

		jointDef := dbox2d.DefaultRevoluteJointDef()
		jointDef.BodyIdA = groundId
		jointDef.LocalAnchorB = dbox2d.Vec2{}
		jointDef.EnableMotor = true
		jointDef.MaxMotorTorque = dbox2d.QFromInt(1000)
		jointDef.EnableLimit = true

		// 30 degrees is 1/12 turn and 5 degrees is 1/72 turn.
		jointDef.MotorSpeed = dbox2d.QZero()
		jointDef.LocalAnchorA = p1
		jointDef.BodyIdB = leftFlipperId
		jointDef.LowerAngle = dbox2d.QFromRatio(-1, 12)
		jointDef.UpperAngle = dbox2d.QFromRatio(1, 72)
		s.leftJointId = dbox2d.CreateRevoluteJoint(s.WorldId, &jointDef)

		jointDef.MotorSpeed = dbox2d.QZero()
		jointDef.LocalAnchorA = p2
		jointDef.BodyIdB = rightFlipperId
		jointDef.LowerAngle = dbox2d.QFromRatio(-1, 72)
		jointDef.UpperAngle = dbox2d.QFromRatio(1, 12)
		s.rightJointId = dbox2d.CreateRevoluteJoint(s.WorldId, &jointDef)
	}

	// Spinners
	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = qv("-4", "17")

		bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		shapeDef := dbox2d.DefaultShapeDef()
		box1 := dbox2d.MakeBox(qs("1.5"), qs("0.125"))
		box2 := dbox2d.MakeBox(qs("0.125"), qs("1.5"))

		dbox2d.CreatePolygonShape(bodyId, &shapeDef, &box1)
		dbox2d.CreatePolygonShape(bodyId, &shapeDef, &box2)

		jointDef := dbox2d.DefaultRevoluteJointDef()
		jointDef.BodyIdA = groundId
		jointDef.BodyIdB = bodyId
		jointDef.LocalAnchorA = bodyDef.Position
		jointDef.LocalAnchorB = dbox2d.Vec2{}
		jointDef.EnableMotor = true
		jointDef.MaxMotorTorque = qs("0.1")
		dbox2d.CreateRevoluteJoint(s.WorldId, &jointDef)

		bodyDef.Position = qv("4", "8")
		bodyId = dbox2d.CreateBody(s.WorldId, &bodyDef)
		dbox2d.CreatePolygonShape(bodyId, &shapeDef, &box1)
		dbox2d.CreatePolygonShape(bodyId, &shapeDef, &box2)
		jointDef.LocalAnchorA = bodyDef.Position
		jointDef.BodyIdB = bodyId
		dbox2d.CreateRevoluteJoint(s.WorldId, &jointDef)
	}

	// Bumpers
	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Position = qv("-4", "8")

		bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		shapeDef := dbox2d.DefaultShapeDef()
		shapeDef.Material.Restitution = qs("1.5")

		circle := dbox2d.Circle{Radius: dbox2d.QOne()}
		dbox2d.CreateCircleShape(bodyId, &shapeDef, &circle)

		bodyDef.Position = qv("4", "17")
		bodyId = dbox2d.CreateBody(s.WorldId, &bodyDef)
		dbox2d.CreateCircleShape(bodyId, &shapeDef, &circle)
	}

	// Ball
	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Position = qv("1", "15")
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.IsBullet = true

		s.ballId = dbox2d.CreateBody(s.WorldId, &bodyDef)

		shapeDef := dbox2d.DefaultShapeDef()
		circle := dbox2d.Circle{Radius: qs("0.2")}
		dbox2d.CreateCircleShape(s.ballId, &shapeDef, &circle)
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
		bodyDef := dbox2d.DefaultBodyDef()
		groundId := dbox2d.CreateBody(s.WorldId, &bodyDef)
		shapeDef := dbox2d.DefaultShapeDef()
		segment := dbox2d.Segment{Point1: qv("-4", "8"), Point2: qv("0", "0")}
		dbox2d.CreateSegmentShape(groundId, &shapeDef, &segment)
		segment = dbox2d.Segment{Point1: qv("0", "0"), Point2: qv("0", "8")}
		dbox2d.CreateSegmentShape(groundId, &shapeDef, &segment)
	}

	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = qv("-0.45", "10.75")
		bodyDef.LinearVelocity = qv("0", "-200")

		bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		circle := dbox2d.Circle{Radius: qs("0.3")}
		shapeDef := dbox2d.DefaultShapeDef()
		shapeDef.Material.Friction = qs("0.2")
		dbox2d.CreateCircleShape(bodyId, &shapeDef, &circle)
	}
	return s
}
