// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/sample_stacking.cpp of Box2D v3.1.1

package samples

import (
	"github.com/dhannyell/dbox2d"
)

func init() {
	RegisterSample("Stacking", "Single Box", NewSingleBox)
	RegisterSample("Stacking", "Tilted Stack", NewTiltedStack)
	RegisterSample("Stacking", "Vertical Stack", NewVerticalStack)
	RegisterSample("Stacking", "Circle Stack", NewCircleStack)
	RegisterSample("Stacking", "Capsule Stack", NewCapsuleStack)
	RegisterSample("Stacking", "Cliff", NewCliff)
	RegisterSample("Stacking", "Arch", NewArch)
	RegisterSample("Stacking", "Double Domino", NewDoubleDomino)
	RegisterSample("Stacking", "Confined", NewConfined)
	RegisterSample("Stacking", "Card House", NewCardHouse)
}

// SingleBox drops a box with sideways velocity onto a long ground segment.
type SingleBox struct {
	Base
	bodyId dbox2d.BodyId
}

// NewSingleBox builds the scene, matching the reference SingleBox
// constructor.
func NewSingleBox(ctx *SampleContext) Sample {
	s := &SingleBox{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 2.5}
		ctx.Camera.Zoom = 3.5
	}

	extent := dbox2d.QOne()

	bodyDef := dbox2d.DefaultBodyDef()
	groundId := dbox2d.CreateBody(s.WorldId, &bodyDef)

	groundWidth := dbox2d.QFromInt(66).Mul(extent)
	shapeDef := dbox2d.DefaultShapeDef()

	segment := dbox2d.Segment{
		Point1: dbox2d.Vec2{X: groundWidth.Neg(), Y: dbox2d.QZero()},
		Point2: dbox2d.Vec2{X: groundWidth, Y: dbox2d.QZero()},
	}
	dbox2d.CreateSegmentShape(groundId, &shapeDef, &segment)

	bodyDef.Type = dbox2d.DynamicBody
	box := dbox2d.MakeBox(extent, extent)
	bodyDef.Position = dbox2d.Vec2{X: dbox2d.QZero(), Y: dbox2d.QOne()}
	bodyDef.LinearVelocity = dbox2d.Vec2{X: dbox2d.QFromInt(5), Y: dbox2d.QZero()}
	s.bodyId = dbox2d.CreateBody(s.WorldId, &bodyDef)
	dbox2d.CreatePolygonShape(s.bodyId, &shapeDef, &box)

	return s
}

// Step advances the scene and reports the box position.
func (s *SingleBox) Step() {
	s.Base.Step()

	position := s.bodyId.GetPosition()
	s.DrawTextLine("(x, y) = (%.2g, %.2g)", ToFloat64(position.X), ToFloat64(position.Y))
}

// TiltedStack creates ten columns of ten rounded boxes with a small offset.
type TiltedStack struct {
	Base
	bodies [tiltedStackRows * tiltedStackColumns]dbox2d.BodyId
}

const (
	tiltedStackColumns = 10
	tiltedStackRows    = 10
)

// NewTiltedStack builds the tilted stack scene.
func NewTiltedStack(ctx *SampleContext) Sample {
	s := &TiltedStack{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 7.5, Y: 7.5}
		ctx.Camera.Zoom = 20.0
	}

	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Position = dbox2d.Vec2{Y: dbox2d.QFromInt(-1)}
		groundID := dbox2d.CreateBody(s.WorldId, &bodyDef)

		box := dbox2d.MakeBox(dbox2d.QFromInt(1000), dbox2d.QOne())
		shapeDef := dbox2d.DefaultShapeDef()
		dbox2d.CreatePolygonShape(groundID, &shapeDef, &box)
	}

	for i := range s.bodies {
		s.bodies[i] = dbox2d.BodyId{}
	}

	box := dbox2d.MakeRoundedBox(dbox2d.QMustParse("0.45"), dbox2d.QMustParse("0.45"), dbox2d.QMustParse("0.05"))
	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Density = dbox2d.QOne()
	shapeDef.Material.Friction = dbox2d.QMustParse("0.3")

	offset := dbox2d.QMustParse("0.2")
	dx := dbox2d.QFromInt(5)
	xroot := dx.Mul(dbox2d.QFromRatio(-1*(tiltedStackColumns-1), 2))

	for j := range tiltedStackColumns {
		x := xroot.Add(dx.Mul(dbox2d.QFromInt(j)))

		for i := range tiltedStackRows {
			bodyDef := dbox2d.DefaultBodyDef()
			bodyDef.Type = dbox2d.DynamicBody

			n := j*tiltedStackRows + i
			bodyDef.Position = dbox2d.Vec2{
				X: x.Add(offset.Mul(dbox2d.QFromInt(i))),
				Y: dbox2d.QFromRatio(1, 2).Add(dbox2d.QFromInt(i)),
			}
			bodyID := dbox2d.CreateBody(s.WorldId, &bodyDef)

			s.bodies[n] = bodyID
			dbox2d.CreatePolygonShape(bodyID, &shapeDef, &box)
		}
	}

	return s
}

const (
	verticalStackMaxColumns = 10
	verticalStackMaxRows    = 15
	verticalStackMaxBullets = 8
)

// VerticalStack demonstrates continuous collision with configurable stacks
// and fast bullets.
type VerticalStack struct {
	Base
	bullets [verticalStackMaxBullets]dbox2d.BodyId
	bodies  [verticalStackMaxRows * verticalStackMaxColumns]dbox2d.BodyId

	columnCount int
	rowCount    int
	bulletCount int
	shapeType   int
	bulletType  int
}

const (
	verticalStackCircleShape = iota
	verticalStackBoxShape
)

// NewVerticalStack builds the configurable vertical stack scene.
func NewVerticalStack(ctx *SampleContext) Sample {
	s := &VerticalStack{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: -7.0, Y: 9.0}
		ctx.Camera.Zoom = 14.0
	}

	{
		bodyDef := dbox2d.DefaultBodyDef()
		groundID := dbox2d.CreateBody(s.WorldId, &bodyDef)
		shapeDef := dbox2d.DefaultShapeDef()

		segment := dbox2d.Segment{
			Point1: dbox2d.Vec2{X: dbox2d.QFromInt(10)},
			Point2: dbox2d.Vec2{X: dbox2d.QFromInt(10), Y: dbox2d.QFromInt(20)},
		}
		dbox2d.CreateSegmentShape(groundID, &shapeDef, &segment)

		segment = dbox2d.Segment{
			Point1: dbox2d.Vec2{X: dbox2d.QFromInt(-30)},
			Point2: dbox2d.Vec2{X: dbox2d.QFromInt(30)},
		}
		dbox2d.CreateSegmentShape(groundID, &shapeDef, &segment)
	}

	for i := range s.bodies {
		s.bodies[i] = dbox2d.BodyId{}
	}
	for i := range s.bullets {
		s.bullets[i] = dbox2d.BodyId{}
	}

	s.shapeType = verticalStackBoxShape
	s.rowCount = 12
	s.columnCount = 1
	s.bulletCount = 1
	s.bulletType = verticalStackCircleShape

	s.createStacks()
	return s
}

func (s *VerticalStack) createStacks() {
	for i := range s.bodies {
		if !s.bodies[i].IsNull() {
			dbox2d.DestroyBody(s.bodies[i])
			s.bodies[i] = dbox2d.BodyId{}
		}
	}

	circle := dbox2d.Circle{Radius: dbox2d.QHalf()}
	box := dbox2d.MakeRoundedBox(dbox2d.QMustParse("0.45"), dbox2d.QMustParse("0.45"), dbox2d.QMustParse("0.05"))
	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Density = dbox2d.QOne()
	shapeDef.Material.Friction = dbox2d.QMustParse("0.3")

	offset := dbox2d.QFromRatio(1, 100)
	if s.shapeType == verticalStackCircleShape {
		offset = dbox2d.QZero()
	}

	dx := dbox2d.QFromInt(-3)
	xroot := dbox2d.QFromInt(8)

	for j := range s.columnCount {
		x := xroot.Add(dx.Mul(dbox2d.QFromInt(j)))

		for i := range s.rowCount {
			bodyDef := dbox2d.DefaultBodyDef()
			bodyDef.Type = dbox2d.DynamicBody

			n := j*s.rowCount + i
			shift := offset
			if i%2 == 0 {
				shift = offset.Neg()
			}
			bodyDef.Position = dbox2d.Vec2{
				X: x.Add(shift),
				Y: dbox2d.QFromRatio(1, 2).Add(dbox2d.QFromInt(i)),
			}
			bodyID := dbox2d.CreateBody(s.WorldId, &bodyDef)

			s.bodies[n] = bodyID
			if s.shapeType == verticalStackCircleShape {
				dbox2d.CreateCircleShape(bodyID, &shapeDef, &circle)
			} else {
				dbox2d.CreatePolygonShape(bodyID, &shapeDef, &box)
			}
		}
	}
}

func (s *VerticalStack) destroyBody() {
	for j := range s.columnCount {
		for i := range s.rowCount {
			n := j*s.rowCount + i
			if !s.bodies[n].IsNull() {
				dbox2d.DestroyBody(s.bodies[n])
				s.bodies[n] = dbox2d.BodyId{}
				break
			}
		}
	}
}

func (s *VerticalStack) destroyBullets() {
	for i := range s.bullets {
		if !s.bullets[i].IsNull() {
			dbox2d.DestroyBody(s.bullets[i])
			s.bullets[i] = dbox2d.BodyId{}
		}
	}
}

func (s *VerticalStack) fireBullets() {
	circle := dbox2d.Circle{Radius: dbox2d.QFromRatio(1, 4)}
	box := dbox2d.MakeBox(dbox2d.QFromRatio(1, 4), dbox2d.QFromRatio(1, 4))
	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Density = dbox2d.QFromInt(4)

	for i := range s.bulletCount {
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = dbox2d.Vec2{
			X: dbox2d.QMustParse("-26.7").Sub(dbox2d.QFromInt(i)),
			Y: dbox2d.QFromInt(6),
		}
		// Bullet speed is linear velocity, so it is not converted to turns.
		speed := randomFloatRange(dbox2d.QFromInt(200), dbox2d.QFromInt(300))
		bodyDef.LinearVelocity = dbox2d.Vec2{X: speed}
		bodyDef.IsBullet = true

		bulletID := dbox2d.CreateBody(s.WorldId, &bodyDef)
		if s.bulletType == verticalStackBoxShape {
			dbox2d.CreatePolygonShape(bulletID, &shapeDef, &box)
		} else {
			dbox2d.CreateCircleShape(bulletID, &shapeDef, &circle)
		}
		s.bullets[i] = bulletID
	}
}

// UpdateGui exposes the stack and bullet controls in reference order.
func (s *VerticalStack) UpdateGui() {
	height := 230
	gui := s.Context.Gui
	gui.Begin("Vertical Stack", 10, s.Context.Camera.Height-height-50, 240, height)

	changed := false
	shapeTypes := []string{"Circle", "Box"}
	changed = changed || gui.Combo("Shape", &s.shapeType, shapeTypes)
	changed = changed || gui.SliderInt("Rows", &s.rowCount, 1, verticalStackMaxRows)
	changed = changed || gui.SliderInt("Columns", &s.columnCount, 1, verticalStackMaxColumns)
	gui.SliderInt("Bullets", &s.bulletCount, 1, verticalStackMaxBullets)
	gui.Combo("Bullet Shape", &s.bulletType, shapeTypes)

	if gui.Button("Fire Bullets") || s.keyDown(KeyB) {
		s.destroyBullets()
		s.fireBullets()
	}

	if gui.Button("Destroy Body") {
		s.destroyBody()
	}

	changed = changed || gui.Button("Reset Stack")
	if changed {
		s.destroyBullets()
		s.createStacks()
	}

	gui.End()
}

// CircleStack collects hit points and displays the shape indices involved.
type CircleStack struct {
	Base
	events []circleStackEvent
}

type circleStackEvent struct {
	indexA int
	indexB int
}

// NewCircleStack builds the hit-event circle stack.
func NewCircleStack(ctx *SampleContext) Sample {
	s := &CircleStack{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0.0, Y: 5.0}
		ctx.Camera.Zoom = 6.0
	}

	shapeIndex := 0
	{
		bodyDef := dbox2d.DefaultBodyDef()
		groundID := dbox2d.CreateBody(s.WorldId, &bodyDef)
		shapeDef := dbox2d.DefaultShapeDef()
		shapeDef.UserData = shapeIndex
		shapeIndex++

		segment := dbox2d.Segment{
			Point1: dbox2d.Vec2{X: dbox2d.QFromInt(-10)},
			Point2: dbox2d.Vec2{X: dbox2d.QFromInt(10)},
		}
		dbox2d.CreateSegmentShape(groundID, &shapeDef, &segment)
	}

	s.WorldId.SetGravity(dbox2d.Vec2{Y: dbox2d.QFromInt(-20)})
	s.WorldId.SetContactTuning(dbox2d.QFromInt(90), dbox2d.QFromInt(10), dbox2d.QFromInt(3))

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	circle := dbox2d.Circle{Radius: dbox2d.QHalf()}
	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.EnableHitEvents = true
	shapeDef.Material.Friction = dbox2d.QZero()

	y := dbox2d.QFromRatio(3, 4)
	for i := range 10 {
		bodyDef.Position.Y = y
		bodyID := dbox2d.CreateBody(s.WorldId, &bodyDef)
		shapeDef.UserData = shapeIndex
		shapeDef.Density = dbox2d.QFromInt(1 + 4*i)
		shapeIndex++
		dbox2d.CreateCircleShape(bodyID, &shapeDef, &circle)
		y = y.Add(dbox2d.QFromRatio(5, 4))
	}

	return s
}

func (s *CircleStack) Step() {
	s.Base.Step()

	events := s.WorldId.GetContactEvents()
	for _, event := range events.HitEvents {
		indexA, _ := event.ShapeIdA.GetUserData().(int)
		indexB, _ := event.ShapeIdB.GetUserData().(int)
		s.Context.Draw.DrawPoint(event.Point, dbox2d.QFromInt(10), dbox2d.ColorWhite)
		s.events = append(s.events, circleStackEvent{indexA: indexA, indexB: indexB})
	}

	for _, event := range s.events {
		s.DrawTextLine("%d, %d", event.indexA, event.indexB)
	}
}

// CapsuleStack drops twenty capsules onto a wide ground box.
type CapsuleStack struct {
	Base
}

// NewCapsuleStack builds the capsule stack scene.
func NewCapsuleStack(ctx *SampleContext) Sample {
	s := &CapsuleStack{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0.0, Y: 5.0}
		ctx.Camera.Zoom = 6.0
	}

	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Position = dbox2d.Vec2{Y: dbox2d.QFromInt(-1)}
		groundID := dbox2d.CreateBody(s.WorldId, &bodyDef)

		shapeDef := dbox2d.DefaultShapeDef()
		polygon := dbox2d.MakeBox(dbox2d.QFromInt(10), dbox2d.QOne())
		dbox2d.CreatePolygonShape(groundID, &shapeDef, &polygon)
	}

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	a := dbox2d.QFromRatio(1, 4)
	capsule := dbox2d.Capsule{
		Center1: dbox2d.Vec2{X: dbox2d.QFromInt(-4).Mul(a)},
		Center2: dbox2d.Vec2{X: dbox2d.QFromInt(4).Mul(a)},
		Radius:  a,
	}
	shapeDef := dbox2d.DefaultShapeDef()

	y := dbox2d.QFromInt(2).Mul(a)
	for range 20 {
		bodyDef.Position.Y = y
		bodyID := dbox2d.CreateBody(s.WorldId, &bodyDef)
		dbox2d.CreateCapsuleShape(bodyID, &shapeDef, &capsule)
		y = y.Add(dbox2d.QFromInt(3).Mul(a))
	}

	return s
}

// Cliff runs moving shapes across a stepped set of static surfaces.
type Cliff struct {
	Base
	bodyIDs [9]dbox2d.BodyId
	flip    bool
}

// NewCliff builds the cliff scene.
func NewCliff(ctx *SampleContext) Sample {
	s := &Cliff{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Zoom = 25.0 * 0.5
		ctx.Camera.Center = Vec2f{X: 0.0, Y: 5.0}
	}

	{
		bodyDef := dbox2d.DefaultBodyDef()
		groundID := dbox2d.CreateBody(s.WorldId, &bodyDef)
		shapeDef := dbox2d.DefaultShapeDef()
		box := dbox2d.MakeOffsetBox(dbox2d.QFromInt(100), dbox2d.QOne(), dbox2d.Vec2{Y: dbox2d.QFromInt(-1)}, dbox2d.RotIdentity())
		dbox2d.CreatePolygonShape(groundID, &shapeDef, &box)

		segment := dbox2d.Segment{
			Point1: dbox2d.Vec2{X: dbox2d.QFromInt(-14), Y: dbox2d.QFromInt(4)},
			Point2: dbox2d.Vec2{X: dbox2d.QFromInt(-8), Y: dbox2d.QFromInt(4)},
		}
		dbox2d.CreateSegmentShape(groundID, &shapeDef, &segment)

		box = dbox2d.MakeOffsetBox(dbox2d.QFromInt(3), dbox2d.QFromRatio(1, 2), dbox2d.Vec2{Y: dbox2d.QFromInt(4)}, dbox2d.RotIdentity())
		dbox2d.CreatePolygonShape(groundID, &shapeDef, &box)

		capsule := dbox2d.Capsule{
			Center1: dbox2d.Vec2{X: dbox2d.QMustParse("8.5"), Y: dbox2d.QFromInt(4)},
			Center2: dbox2d.Vec2{X: dbox2d.QMustParse("13.5"), Y: dbox2d.QFromInt(4)},
			Radius:  dbox2d.QHalf(),
		}
		dbox2d.CreateCapsuleShape(groundID, &shapeDef, &capsule)
	}

	s.flip = false
	for i := range s.bodyIDs {
		s.bodyIDs[i] = dbox2d.BodyId{}
	}
	s.createBodies()

	return s
}

func (s *Cliff) createBodies() {
	for i := range s.bodyIDs {
		if !s.bodyIDs[i].IsNull() {
			dbox2d.DestroyBody(s.bodyIDs[i])
			s.bodyIDs[i] = dbox2d.BodyId{}
		}
	}

	sign := dbox2d.QOne()
	if s.flip {
		sign = sign.Neg()
	}

	capsule := dbox2d.Capsule{
		Center1: dbox2d.Vec2{X: dbox2d.QFromRatio(-1, 4)},
		Center2: dbox2d.Vec2{X: dbox2d.QFromRatio(1, 4)},
		Radius:  dbox2d.QFromRatio(1, 4),
	}
	circle := dbox2d.Circle{Radius: dbox2d.QHalf()}
	square := dbox2d.MakeSquare(dbox2d.QHalf())

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody

	{
		shapeDef := dbox2d.DefaultShapeDef()
		shapeDef.Material.Friction = dbox2d.QFromRatio(1, 100)
		bodyDef.LinearVelocity = dbox2d.Vec2{X: dbox2d.QFromInt(2).Mul(sign)}

		offset := dbox2d.QZero()
		if s.flip {
			offset = dbox2d.QFromInt(-4)
		}

		bodyDef.Position = dbox2d.Vec2{X: dbox2d.QFromInt(-9).Add(offset), Y: dbox2d.QMustParse("4.25")}
		s.bodyIDs[0] = dbox2d.CreateBody(s.WorldId, &bodyDef)
		dbox2d.CreateCapsuleShape(s.bodyIDs[0], &shapeDef, &capsule)

		bodyDef.Position = dbox2d.Vec2{X: dbox2d.QFromInt(2).Add(offset), Y: dbox2d.QMustParse("4.75")}
		s.bodyIDs[1] = dbox2d.CreateBody(s.WorldId, &bodyDef)
		dbox2d.CreateCapsuleShape(s.bodyIDs[1], &shapeDef, &capsule)

		bodyDef.Position = dbox2d.Vec2{X: dbox2d.QFromInt(13).Add(offset), Y: dbox2d.QMustParse("4.75")}
		s.bodyIDs[2] = dbox2d.CreateBody(s.WorldId, &bodyDef)
		dbox2d.CreateCapsuleShape(s.bodyIDs[2], &shapeDef, &capsule)
	}

	{
		shapeDef := dbox2d.DefaultShapeDef()
		shapeDef.Material.Friction = dbox2d.QFromRatio(1, 100)
		bodyDef.LinearVelocity = dbox2d.Vec2{X: dbox2d.QMustParse("2.5").Mul(sign)}

		bodyDef.Position = dbox2d.Vec2{X: dbox2d.QFromInt(-11), Y: dbox2d.QMustParse("4.5")}
		s.bodyIDs[3] = dbox2d.CreateBody(s.WorldId, &bodyDef)
		dbox2d.CreatePolygonShape(s.bodyIDs[3], &shapeDef, &square)

		bodyDef.Position = dbox2d.Vec2{Y: dbox2d.QFromInt(5)}
		s.bodyIDs[4] = dbox2d.CreateBody(s.WorldId, &bodyDef)
		dbox2d.CreatePolygonShape(s.bodyIDs[4], &shapeDef, &square)

		bodyDef.Position = dbox2d.Vec2{X: dbox2d.QFromInt(11), Y: dbox2d.QFromInt(5)}
		s.bodyIDs[5] = dbox2d.CreateBody(s.WorldId, &bodyDef)
		dbox2d.CreatePolygonShape(s.bodyIDs[5], &shapeDef, &square)
	}

	{
		shapeDef := dbox2d.DefaultShapeDef()
		shapeDef.Material.Friction = dbox2d.QFromRatio(1, 5)
		bodyDef.LinearVelocity = dbox2d.Vec2{X: dbox2d.QMustParse("1.5").Mul(sign)}

		offset := dbox2d.QZero()
		if s.flip {
			offset = dbox2d.QFromInt(4)
		}

		bodyDef.Position = dbox2d.Vec2{X: dbox2d.QFromInt(-13).Add(offset), Y: dbox2d.QMustParse("4.5")}
		s.bodyIDs[6] = dbox2d.CreateBody(s.WorldId, &bodyDef)
		dbox2d.CreateCircleShape(s.bodyIDs[6], &shapeDef, &circle)

		bodyDef.Position = dbox2d.Vec2{X: dbox2d.QFromInt(-2).Add(offset), Y: dbox2d.QFromInt(5)}
		s.bodyIDs[7] = dbox2d.CreateBody(s.WorldId, &bodyDef)
		dbox2d.CreateCircleShape(s.bodyIDs[7], &shapeDef, &circle)

		bodyDef.Position = dbox2d.Vec2{X: dbox2d.QFromInt(9).Add(offset), Y: dbox2d.QFromInt(5)}
		s.bodyIDs[8] = dbox2d.CreateBody(s.WorldId, &bodyDef)
		dbox2d.CreateCircleShape(s.bodyIDs[8], &shapeDef, &circle)
	}
}

// UpdateGui provides the reference Flip control.
func (s *Cliff) UpdateGui() {
	height := 60
	gui := s.Context.Gui
	gui.Begin("Cliff", 10, s.Context.Camera.Height-height-50, 160, height)
	if gui.Button("Flip") {
		s.flip = !s.flip
		s.createBodies()
	}
	gui.End()
}

// Arch builds two symmetric polygon arches with a keystone and four boxes.
type Arch struct {
	Base
}

// NewArch builds the arch scene from the reference vertex tables.
func NewArch(ctx *SampleContext) Sample {
	s := &Arch{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0.0, Y: 8.0}
		ctx.Camera.Zoom = 25.0 * 0.35
	}

	ps1 := [9]dbox2d.Vec2{
		{X: dbox2d.QMustParse("16.0"), Y: dbox2d.QMustParse("0.0")},
		{X: dbox2d.QMustParse("14.93803712795643"), Y: dbox2d.QMustParse("5.133601056842984")},
		{X: dbox2d.QMustParse("13.79871746027416"), Y: dbox2d.QMustParse("10.24928069555078")},
		{X: dbox2d.QMustParse("12.56252963284711"), Y: dbox2d.QMustParse("15.34107019122473")},
		{X: dbox2d.QMustParse("11.20040987372525"), Y: dbox2d.QMustParse("20.39856541571217")},
		{X: dbox2d.QMustParse("9.66521217819836"), Y: dbox2d.QMustParse("25.40369899225096")},
		{X: dbox2d.QMustParse("7.87179930638133"), Y: dbox2d.QMustParse("30.3179337000085")},
		{X: dbox2d.QMustParse("5.635199558196225"), Y: dbox2d.QMustParse("35.03820717801641")},
		{X: dbox2d.QMustParse("2.405937953536585"), Y: dbox2d.QMustParse("39.09554102558315")},
	}
	ps2 := [9]dbox2d.Vec2{
		{X: dbox2d.QMustParse("24.0"), Y: dbox2d.QMustParse("0.0")},
		{X: dbox2d.QMustParse("22.33619528222415"), Y: dbox2d.QMustParse("6.02299846205841")},
		{X: dbox2d.QMustParse("20.54936888969905"), Y: dbox2d.QMustParse("12.00964361211476")},
		{X: dbox2d.QMustParse("18.60854610798073"), Y: dbox2d.QMustParse("17.9470321677465")},
		{X: dbox2d.QMustParse("16.46769273811807"), Y: dbox2d.QMustParse("23.81367936585418")},
		{X: dbox2d.QMustParse("14.05325025774858"), Y: dbox2d.QMustParse("29.57079353071012")},
		{X: dbox2d.QMustParse("11.23551045834022"), Y: dbox2d.QMustParse("35.13775818285372")},
		{X: dbox2d.QMustParse("7.752568160730571"), Y: dbox2d.QMustParse("40.30450679009583")},
		{X: dbox2d.QMustParse("3.016931552701656"), Y: dbox2d.QMustParse("44.28891593799322")},
	}

	scale := dbox2d.QFromRatio(1, 4)
	for i := range ps1 {
		ps1[i].X = ps1[i].X.Mul(scale)
		ps1[i].Y = ps1[i].Y.Mul(scale)
		ps2[i].X = ps2[i].X.Mul(scale)
		ps2[i].Y = ps2[i].Y.Mul(scale)
	}

	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Material.Friction = dbox2d.QMustParse("0.6")

	{
		bodyDef := dbox2d.DefaultBodyDef()
		groundID := dbox2d.CreateBody(s.WorldId, &bodyDef)
		segment := dbox2d.Segment{
			Point1: dbox2d.Vec2{X: dbox2d.QFromInt(-100)},
			Point2: dbox2d.Vec2{X: dbox2d.QFromInt(100)},
		}
		dbox2d.CreateSegmentShape(groundID, &shapeDef, &segment)
	}

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	for i := range 8 {
		bodyID := dbox2d.CreateBody(s.WorldId, &bodyDef)
		points := []dbox2d.Vec2{ps1[i], ps2[i], ps2[i+1], ps1[i+1]}
		hull := dbox2d.ComputeHull(points)
		polygon := dbox2d.MakePolygon(&hull, dbox2d.QZero())
		dbox2d.CreatePolygonShape(bodyID, &shapeDef, &polygon)
	}

	for i := range 8 {
		bodyID := dbox2d.CreateBody(s.WorldId, &bodyDef)
		points := []dbox2d.Vec2{
			{X: ps2[i].X.Neg(), Y: ps2[i].Y},
			{X: ps1[i].X.Neg(), Y: ps1[i].Y},
			{X: ps1[i+1].X.Neg(), Y: ps1[i+1].Y},
			{X: ps2[i+1].X.Neg(), Y: ps2[i+1].Y},
		}
		hull := dbox2d.ComputeHull(points)
		polygon := dbox2d.MakePolygon(&hull, dbox2d.QZero())
		dbox2d.CreatePolygonShape(bodyID, &shapeDef, &polygon)
	}

	{
		bodyID := dbox2d.CreateBody(s.WorldId, &bodyDef)
		points := []dbox2d.Vec2{
			ps1[8], ps2[8],
			{X: ps2[8].X.Neg(), Y: ps2[8].Y},
			{X: ps1[8].X.Neg(), Y: ps1[8].Y},
		}
		hull := dbox2d.ComputeHull(points)
		polygon := dbox2d.MakePolygon(&hull, dbox2d.QZero())
		dbox2d.CreatePolygonShape(bodyID, &shapeDef, &polygon)
	}

	box := dbox2d.MakeBox(dbox2d.QFromInt(2), dbox2d.QHalf())
	for i := range 4 {
		bodyDef.Position = dbox2d.Vec2{
			Y: dbox2d.QHalf().Add(ps2[8].Y).Add(dbox2d.QFromInt(i)),
		}
		bodyID := dbox2d.CreateBody(s.WorldId, &bodyDef)
		dbox2d.CreatePolygonShape(bodyID, &shapeDef, &box)
	}

	return s
}

// DoubleDomino builds a row of dominoes and nudges the first one.
type DoubleDomino struct {
	Base
}

// NewDoubleDomino builds the double domino scene.
func NewDoubleDomino(ctx *SampleContext) Sample {
	s := &DoubleDomino{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0.0, Y: 4.0}
		ctx.Camera.Zoom = 25.0 * 0.25
	}

	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Position = dbox2d.Vec2{Y: dbox2d.QFromInt(-1)}
		groundID := dbox2d.CreateBody(s.WorldId, &bodyDef)
		box := dbox2d.MakeBox(dbox2d.QFromInt(100), dbox2d.QFromInt(1))
		shapeDef := dbox2d.DefaultShapeDef()
		dbox2d.CreatePolygonShape(groundID, &shapeDef, &box)
	}

	box := dbox2d.MakeBox(dbox2d.QFromRatio(1, 8), dbox2d.QHalf())
	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Material.Friction = dbox2d.QMustParse("0.6")
	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	count := 15
	x := dbox2d.QFromRatio(-count, 2)
	for i := range count {
		bodyDef.Position = dbox2d.Vec2{X: x, Y: dbox2d.QHalf()}
		bodyID := dbox2d.CreateBody(s.WorldId, &bodyDef)
		dbox2d.CreatePolygonShape(bodyID, &shapeDef, &box)
		if i == 0 {
			bodyID.ApplyLinearImpulse(
				dbox2d.Vec2{X: dbox2d.QMustParse("0.2")},
				dbox2d.Vec2{X: x, Y: dbox2d.QFromInt(1)},
				true,
			)
		}
		x = x.Add(dbox2d.QOne())
	}

	return s
}

const (
	confinedGridCount = 25
	confinedMaxCount  = confinedGridCount * confinedGridCount
)

// Confined fills a capsule-framed box with zero-gravity circles.
type Confined struct {
	Base
	row    int
	column int
	count  int
}

// NewConfined builds the confined circle grid.
func NewConfined(ctx *SampleContext) Sample {
	s := &Confined{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0.0, Y: 10.0}
		ctx.Camera.Zoom = 25.0 * 0.5
	}

	{
		bodyDef := dbox2d.DefaultBodyDef()
		groundID := dbox2d.CreateBody(s.WorldId, &bodyDef)
		shapeDef := dbox2d.DefaultShapeDef()
		capsule := dbox2d.Capsule{
			Center1: dbox2d.Vec2{X: dbox2d.QMustParse("-10.5")},
			Center2: dbox2d.Vec2{X: dbox2d.QMustParse("10.5")},
			Radius:  dbox2d.QHalf(),
		}
		dbox2d.CreateCapsuleShape(groundID, &shapeDef, &capsule)
		capsule = dbox2d.Capsule{
			Center1: dbox2d.Vec2{X: dbox2d.QMustParse("-10.5")},
			Center2: dbox2d.Vec2{X: dbox2d.QMustParse("-10.5"), Y: dbox2d.QMustParse("20.5")},
			Radius:  dbox2d.QHalf(),
		}
		dbox2d.CreateCapsuleShape(groundID, &shapeDef, &capsule)
		capsule = dbox2d.Capsule{
			Center1: dbox2d.Vec2{X: dbox2d.QMustParse("10.5")},
			Center2: dbox2d.Vec2{X: dbox2d.QMustParse("10.5"), Y: dbox2d.QMustParse("20.5")},
			Radius:  dbox2d.QHalf(),
		}
		dbox2d.CreateCapsuleShape(groundID, &shapeDef, &capsule)
		capsule = dbox2d.Capsule{
			Center1: dbox2d.Vec2{X: dbox2d.QMustParse("-10.5"), Y: dbox2d.QMustParse("20.5")},
			Center2: dbox2d.Vec2{X: dbox2d.QMustParse("10.5"), Y: dbox2d.QMustParse("20.5")},
			Radius:  dbox2d.QHalf(),
		}
		dbox2d.CreateCapsuleShape(groundID, &shapeDef, &capsule)
	}

	s.row = 0
	s.column = 0
	s.count = 0
	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	bodyDef.GravityScale = dbox2d.QZero()
	shapeDef := dbox2d.DefaultShapeDef()
	circle := dbox2d.Circle{Radius: dbox2d.QHalf()}
	step := dbox2d.QFromRatio(18, confinedGridCount)
	for s.count < confinedMaxCount {
		s.row = 0
		for range confinedGridCount {
			x := dbox2d.QMustParse("-8.75").Add(dbox2d.QFromInt(s.column).Mul(step))
			y := dbox2d.QMustParse("1.5").Add(dbox2d.QFromInt(s.row).Mul(step))
			bodyDef.Position = dbox2d.Vec2{X: x, Y: y}
			bodyID := dbox2d.CreateBody(s.WorldId, &bodyDef)
			dbox2d.CreateCircleShape(bodyID, &shapeDef, &circle)
			s.count++
			s.row++
		}
		s.column++
	}

	return s
}

// CardHouse stacks alternating cards with a flat card at each tier.
type CardHouse struct {
	Base
}

// NewCardHouse builds the card house scene.
func NewCardHouse(ctx *SampleContext) Sample {
	s := &CardHouse{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0.75, Y: 0.9}
		ctx.Camera.Zoom = 25.0 * 0.05
	}

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Position = dbox2d.Vec2{Y: dbox2d.QFromInt(-2)}
	groundID := dbox2d.CreateBody(s.WorldId, &bodyDef)
	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Material.Friction = dbox2d.QMustParse("0.7")
	groundBox := dbox2d.MakeBox(dbox2d.QFromInt(40), dbox2d.QFromInt(2))
	dbox2d.CreatePolygonShape(groundID, &shapeDef, &groundBox)

	cardHeight := dbox2d.QMustParse("0.2")
	cardThickness := dbox2d.QMustParse("0.001")
	angle0 := dbox2d.QFromRatio(5, 72)
	angle1 := angle0.Neg()
	angle2 := dbox2d.QFromRatio(1, 4)
	cardBox := dbox2d.MakeBox(cardThickness, cardHeight)
	bodyDef.Type = dbox2d.DynamicBody

	nb := 5
	z0 := dbox2d.QZero()
	y := cardHeight.Sub(dbox2d.QMustParse("0.02"))
	for nb > 0 {
		z := z0
		for i := range nb {
			if i != nb-1 {
				bodyDef.Position = dbox2d.Vec2{
					X: z.Add(dbox2d.QMustParse("0.25")),
					Y: y.Add(cardHeight).Sub(dbox2d.QMustParse("0.015")),
				}
				bodyDef.Rotation = dbox2d.MakeRot(angle2)
				bodyID := dbox2d.CreateBody(s.WorldId, &bodyDef)
				dbox2d.CreatePolygonShape(bodyID, &shapeDef, &cardBox)
			}

			bodyDef.Position = dbox2d.Vec2{X: z, Y: y}
			bodyDef.Rotation = dbox2d.MakeRot(angle1)
			bodyID := dbox2d.CreateBody(s.WorldId, &bodyDef)
			dbox2d.CreatePolygonShape(bodyID, &shapeDef, &cardBox)

			z = z.Add(dbox2d.QMustParse("0.175"))
			bodyDef.Position = dbox2d.Vec2{X: z, Y: y}
			bodyDef.Rotation = dbox2d.MakeRot(angle0)
			bodyID = dbox2d.CreateBody(s.WorldId, &bodyDef)
			dbox2d.CreatePolygonShape(bodyID, &shapeDef, &cardBox)

			z = z.Add(dbox2d.QMustParse("0.175"))
		}
		y = y.Add(cardHeight.Mul(dbox2d.QFromInt(2))).Sub(dbox2d.QMustParse("0.03"))
		z0 = z0.Add(dbox2d.QMustParse("0.175"))
		nb--
	}

	return s
}
