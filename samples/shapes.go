// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/sample_shapes.cpp of Box2D v3.1.1

package samples

import (
	"fmt"
	"math"

	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/dbox2d/internal/shared"
)

func init() {
	RegisterSample("Shapes", "Chain Shape", NewChainShape)
	RegisterSample("Shapes", "Compound Shapes", NewCompoundShapes)
	RegisterSample("Shapes", "Filter", NewShapeFilter)
	RegisterSample("Shapes", "Custom Filter", NewCustomFilter)
	RegisterSample("Shapes", "Restitution", NewRestitution)
	RegisterSample("Shapes", "Friction", NewFriction)
	RegisterSample("Shapes", "Rolling Resistance", NewRollingResistance)
	RegisterSample("Shapes", "Conveyor Belt", NewConveyorBelt)
	RegisterSample("Shapes", "Tangent Speed", NewTangentSpeed)
	RegisterSample("Shapes", "Modify Geometry", NewModifyGeometry)
	RegisterSample("Shapes", "Chain Link", NewChainLink)
	RegisterSample("Shapes", "Rounded", NewRoundedShapes)
	RegisterSample("Shapes", "Ellipse", NewEllipseShape)
	RegisterSample("Shapes", "Offset", NewOffsetShapes)
	RegisterSample("Shapes", "Explosion", NewExplosion)
	RegisterSample("Shapes", "Recreate Static", NewRecreateStatic)
}

// qvs builds a point list from coordinate pairs.
func qvs(coords ...float64) []b2.Vec2 {
	points := make([]b2.Vec2, len(coords)/2)
	for i := range points {
		points[i] = b2.V2(coords[2*i], coords[2*i+1])
	}
	return points
}

// ChainShape rolls a shape along a closed chain.
type ChainShape struct {
	Base

	groundId    b2.BodyId
	bodyId      b2.BodyId
	chainId     b2.ChainId
	shapeType   int
	shapeId     b2.ShapeId
	restitution float64
	friction    float64
}

const (
	chainCircleShape = iota
	chainCapsuleShape
	chainBoxShape
)

func NewChainShape(ctx *SampleContext) Sample {
	s := &ChainShape{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 0}
		ctx.Camera.Zoom = 25 * 1.75
	}

	s.shapeType = chainCircleShape
	s.restitution = 0
	s.friction = 0.2

	s.createScene()
	s.launch()
	return s
}

func (s *ChainShape) createScene() {
	if !s.groundId.IsNull() {
		b2.DestroyBody(s.groundId)
	}

	// https://betravis.github.io/shape-tools/path-to-polygon/
	points := qvs(
		-56.885498, 12.8985004, -56.885498, 16.2057495, 56.885498, 16.2057495, 56.885498, -16.2057514,
		51.5935059, -16.2057514, 43.6559982, -10.9139996, 35.7184982, -10.9139996, 27.7809982, -10.9139996,
		21.1664963, -14.2212505, 11.9059982, -16.2057514, 0, -16.2057514, -10.5835037, -14.8827496,
		-17.1980019, -13.5597477, -21.1665001, -12.2370014, -25.1355019, -9.5909977, -31.75, -3.63799858,
		-38.3644981, 6.2840004, -42.3334999, 9.59125137, -47.625, 11.5755005, -56.885498, 12.8985004,
	)

	material := b2.SurfaceMaterial{}
	material.Friction = b2.F(0.2)
	material.CustomColor = uint32(b2.ColorSteelBlue)
	material.UserMaterialId = 42

	chainDef := b2.DefaultChainDef()
	chainDef.Points = points
	chainDef.Materials = []b2.SurfaceMaterial{material}
	chainDef.IsLoop = true

	bodyDef := b2.DefaultBodyDef()
	s.groundId = b2.CreateBody(s.WorldId, &bodyDef)

	s.chainId = b2.CreateChain(s.groundId, &chainDef)
}

func (s *ChainShape) launch() {
	if !s.bodyId.IsNull() {
		b2.DestroyBody(s.bodyId)
	}

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.Position = b2.V2(-55.0, 13.5)
	s.bodyId = b2.CreateBody(s.WorldId, &bodyDef)

	shapeDef := b2.DefaultShapeDef()
	shapeDef.Density = b2.QOne()
	shapeDef.Material.Friction = FromFloat64(s.friction)
	shapeDef.Material.Restitution = FromFloat64(s.restitution)

	switch s.shapeType {
	case chainCircleShape:
		circle := b2.Circle{Radius: b2.QHalf()}
		s.shapeId = b2.CreateCircleShape(s.bodyId, &shapeDef, &circle)

	case chainCapsuleShape:
		capsule := b2.Capsule{Center1: b2.V2(-0.5, 0.0), Center2: b2.V2(0.5, 0.0), Radius: b2.F(0.25)}
		s.shapeId = b2.CreateCapsuleShape(s.bodyId, &shapeDef, &capsule)

	default:
		h := b2.QHalf()
		box := b2.MakeBox(h, h)
		s.shapeId = b2.CreatePolygonShape(s.bodyId, &shapeDef, &box)
	}

	s.StepCount = 0
}

func (s *ChainShape) UpdateGui() {
	height := 155
	gui := s.Context.Gui
	gui.Begin("Chain Shape", 10, s.Context.Camera.Height-height-50, 240, height)

	shapeTypes := []string{"Circle", "Capsule", "Box"}
	if gui.Combo("Shape", &s.shapeType, shapeTypes) {
		s.launch()
	}

	if gui.SliderFloat("Friction", &s.friction, 0, 1) {
		s.shapeId.SetFriction(FromFloat64(s.friction))
		s.chainId.SetFriction(FromFloat64(s.friction))
	}

	if gui.SliderFloat("Restitution", &s.restitution, 0, 2) {
		s.shapeId.SetRestitution(FromFloat64(s.restitution))
	}

	if gui.Button("Launch") {
		s.launch()
	}

	gui.End()
}

func (s *ChainShape) Step() {
	s.Base.Step()

	s.Context.Draw.DrawSegment(b2.Vec2{}, b2.V2(0.5, 0.0), b2.ColorRed)
	s.Context.Draw.DrawSegment(b2.Vec2{}, b2.V2(0.0, 0.5), b2.ColorGreen)
}

// CompoundShapes shows how careful creation of compound shapes leads to
// better simulation and avoids objects getting stuck. It also shows how to
// get the combined AABB for the body.
type CompoundShapes struct {
	Base

	table1Id      b2.BodyId
	table2Id      b2.BodyId
	ship1Id       b2.BodyId
	ship2Id       b2.BodyId
	drawBodyAABBs bool
	aabbVertices  [4]b2.Vec2
}

func NewCompoundShapes(ctx *SampleContext) Sample {
	s := &CompoundShapes{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 6}
		ctx.Camera.Zoom = 25 * 0.5
	}

	{
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(s.WorldId, &bodyDef)
		shapeDef := b2.DefaultShapeDef()
		segment := b2.Segment{Point1: b2.V2(50, 0), Point2: b2.V2(-50, 0)}
		b2.CreateSegmentShape(groundId, &shapeDef, &segment)
	}

	// Table 1
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.V2(-15, 1)
		s.table1Id = b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		top := b2.MakeOffsetBox(b2.F(3), b2.QHalf(), b2.V2(0.0, 3.5), b2.RotIdentity())
		leftLeg := b2.MakeOffsetBox(b2.QHalf(), b2.F(1.5), b2.V2(-2.5, 1.5), b2.RotIdentity())
		rightLeg := b2.MakeOffsetBox(b2.QHalf(), b2.F(1.5), b2.V2(2.5, 1.5), b2.RotIdentity())

		b2.CreatePolygonShape(s.table1Id, &shapeDef, &top)
		b2.CreatePolygonShape(s.table1Id, &shapeDef, &leftLeg)
		b2.CreatePolygonShape(s.table1Id, &shapeDef, &rightLeg)
	}

	// Table 2
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.V2(-5, 1)
		s.table2Id = b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		top := b2.MakeOffsetBox(b2.F(3), b2.QHalf(), b2.V2(0.0, 3.5), b2.RotIdentity())
		leftLeg := b2.MakeOffsetBox(b2.QHalf(), b2.F(2), b2.V2(-2.5, 2.0), b2.RotIdentity())
		rightLeg := b2.MakeOffsetBox(b2.QHalf(), b2.F(2), b2.V2(2.5, 2.0), b2.RotIdentity())

		b2.CreatePolygonShape(s.table2Id, &shapeDef, &top)
		b2.CreatePolygonShape(s.table2Id, &shapeDef, &leftLeg)
		b2.CreatePolygonShape(s.table2Id, &shapeDef, &rightLeg)
	}

	// Spaceship 1
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.V2(5, 1)
		s.ship1Id = b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		var vertices [3]b2.Vec2

		vertices[0] = b2.V2(-2, 0)
		vertices[1] = b2.Vec2{Y: b2.QFromRatio(4, 3)}
		vertices[2] = b2.V2(0, 4)
		hull := b2.ComputeHull(vertices[:])
		left := b2.MakePolygon(&hull, b2.QZero())

		vertices[0] = b2.V2(2, 0)
		vertices[1] = b2.Vec2{Y: b2.QFromRatio(4, 3)}
		vertices[2] = b2.V2(0, 4)
		hull = b2.ComputeHull(vertices[:])
		right := b2.MakePolygon(&hull, b2.QZero())

		b2.CreatePolygonShape(s.ship1Id, &shapeDef, &left)
		b2.CreatePolygonShape(s.ship1Id, &shapeDef, &right)
	}

	// Spaceship 2
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.V2(15, 1)
		s.ship2Id = b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		var vertices [3]b2.Vec2

		vertices[0] = b2.V2(-2, 0)
		vertices[1] = b2.V2(1, 2)
		vertices[2] = b2.V2(0, 4)
		hull := b2.ComputeHull(vertices[:])
		left := b2.MakePolygon(&hull, b2.QZero())

		vertices[0] = b2.V2(2, 0)
		vertices[1] = b2.V2(-1, 2)
		vertices[2] = b2.V2(0, 4)
		hull = b2.ComputeHull(vertices[:])
		right := b2.MakePolygon(&hull, b2.QZero())

		b2.CreatePolygonShape(s.ship2Id, &shapeDef, &left)
		b2.CreatePolygonShape(s.ship2Id, &shapeDef, &right)
	}

	s.drawBodyAABBs = false
	return s
}

func (s *CompoundShapes) spawn() {
	// Table 1 obstruction
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = s.table1Id.GetPosition()
		bodyDef.Rotation = s.table1Id.GetRotation()
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		box := b2.MakeOffsetBox(b2.F(4), b2.F(0.1), b2.V2(0, 3), b2.RotIdentity())
		b2.CreatePolygonShape(bodyId, &shapeDef, &box)
	}

	// Table 2 obstruction
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = s.table2Id.GetPosition()
		bodyDef.Rotation = s.table2Id.GetRotation()
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		box := b2.MakeOffsetBox(b2.F(4), b2.F(0.1), b2.V2(0, 3), b2.RotIdentity())
		b2.CreatePolygonShape(bodyId, &shapeDef, &box)
	}

	// Ship 1 obstruction
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = s.ship1Id.GetPosition()
		bodyDef.Rotation = s.ship1Id.GetRotation()
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		circle := b2.Circle{Center: b2.V2(0, 2), Radius: b2.QHalf()}
		b2.CreateCircleShape(bodyId, &shapeDef, &circle)
	}

	// Ship 2 obstruction
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = s.ship2Id.GetPosition()
		bodyDef.Rotation = s.ship2Id.GetRotation()
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		circle := b2.Circle{Center: b2.V2(0, 2), Radius: b2.QHalf()}
		b2.CreateCircleShape(bodyId, &shapeDef, &circle)
	}
}

func (s *CompoundShapes) UpdateGui() {
	height := 100
	gui := s.Context.Gui
	gui.Begin("Compound Shapes", 10, s.Context.Camera.Height-height-50, 180, height)

	if gui.Button("Intrude") {
		s.spawn()
	}

	gui.Checkbox("Body AABBs", &s.drawBodyAABBs)

	gui.End()
}

// drawAABB draws a box outline, like the reference Draw::DrawAABB.
func (s *CompoundShapes) drawAABB(box b2.AABB, color b2.HexColor) {
	v := &s.aabbVertices
	v[0] = box.LowerBound
	v[1] = b2.Vec2{X: box.UpperBound.X, Y: box.LowerBound.Y}
	v[2] = box.UpperBound
	v[3] = b2.Vec2{X: box.LowerBound.X, Y: box.UpperBound.Y}
	s.Context.Draw.DrawPolygon(v[:], color)
}

func (s *CompoundShapes) Step() {
	s.Base.Step()

	if s.drawBodyAABBs {
		s.drawAABB(s.table1Id.ComputeAABB(), b2.ColorYellow)
		s.drawAABB(s.table2Id.ComputeAABB(), b2.ColorYellow)
		s.drawAABB(s.ship1Id.ComputeAABB(), b2.ColorYellow)
		s.drawAABB(s.ship2Id.ComputeAABB(), b2.ColorYellow)
	}
}

// Collision bits of the Filter sample.
const (
	filterGround uint64 = 0x00000001
	filterTeam1  uint64 = 0x00000002
	filterTeam2  uint64 = 0x00000004
	filterTeam3  uint64 = 0x00000008

	// filterAllBits is the reference's ~0u, a 32-bit all-ones value.
	filterAllBits uint64 = 0xFFFFFFFF
)

// ShapeFilter lets each player pick the teams it collides with.
type ShapeFilter struct {
	Base

	player1Id b2.BodyId
	player2Id b2.BodyId
	player3Id b2.BodyId

	shape1Id b2.ShapeId
	shape2Id b2.ShapeId
	shape3Id b2.ShapeId

	// Checkbox state; it is refreshed from the filters every frame.
	collides [3][2]bool
}

func NewShapeFilter(ctx *SampleContext) Sample {
	s := &ShapeFilter{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Zoom = 25 * 0.5
		ctx.Camera.Center = Vec2f{X: 0, Y: 5}
	}

	{
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(s.WorldId, &bodyDef)
		segment := b2.Segment{Point1: b2.V2(-20, 0), Point2: b2.V2(20, 0)}

		shapeDef := b2.DefaultShapeDef()
		shapeDef.Filter.CategoryBits = filterGround
		shapeDef.Filter.MaskBits = filterAllBits

		b2.CreateSegmentShape(groundId, &shapeDef, &segment)
	}

	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody

		bodyDef.Position = b2.V2(0, 2)
		s.player1Id = b2.CreateBody(s.WorldId, &bodyDef)

		bodyDef.Position = b2.V2(0, 5)
		s.player2Id = b2.CreateBody(s.WorldId, &bodyDef)

		bodyDef.Position = b2.V2(0, 8)
		s.player3Id = b2.CreateBody(s.WorldId, &bodyDef)

		box := b2.MakeBox(b2.F(2), b2.QOne())

		shapeDef := b2.DefaultShapeDef()

		shapeDef.Filter.CategoryBits = filterTeam1
		shapeDef.Filter.MaskBits = filterGround | filterTeam2 | filterTeam3
		s.shape1Id = b2.CreatePolygonShape(s.player1Id, &shapeDef, &box)

		shapeDef.Filter.CategoryBits = filterTeam2
		shapeDef.Filter.MaskBits = filterGround | filterTeam1 | filterTeam3
		s.shape2Id = b2.CreatePolygonShape(s.player2Id, &shapeDef, &box)

		shapeDef.Filter.CategoryBits = filterTeam3
		shapeDef.Filter.MaskBits = filterGround | filterTeam1 | filterTeam2
		s.shape3Id = b2.CreatePolygonShape(s.player3Id, &shapeDef, &box)
	}

	return s
}

// teamCheckbox shows one mask bit of a shape filter as a checkbox and
// writes the filter back when it is toggled.
func teamCheckbox(gui Gui, label string, state *bool, shapeId b2.ShapeId, team uint64) {
	filter := shapeId.GetFilter()
	*state = filter.MaskBits&team == team
	if gui.Checkbox(label, state) {
		if *state {
			filter.MaskBits |= team
		} else {
			filter.MaskBits &^= team
		}

		shapeId.SetFilter(filter)
	}
}

func (s *ShapeFilter) UpdateGui() {
	height := 240
	gui := s.Context.Gui
	gui.Begin("Shape Filter", 10, s.Context.Camera.Height-height-50, 240, height)

	gui.Text("Player 1 Collides With")
	teamCheckbox(gui, "Team 2", &s.collides[0][0], s.shape1Id, filterTeam2)
	teamCheckbox(gui, "Team 3", &s.collides[0][1], s.shape1Id, filterTeam3)

	gui.Text("Player 2 Collides With")
	teamCheckbox(gui, "Team 1", &s.collides[1][0], s.shape2Id, filterTeam1)
	teamCheckbox(gui, "Team 3", &s.collides[1][1], s.shape2Id, filterTeam3)

	gui.Text("Player 3 Collides With")
	teamCheckbox(gui, "Team 1", &s.collides[2][0], s.shape3Id, filterTeam1)
	teamCheckbox(gui, "Team 2", &s.collides[2][1], s.shape3Id, filterTeam2)

	gui.End()
}

func (s *ShapeFilter) Step() {
	s.Base.Step()

	p1 := s.player1Id.GetPosition()
	s.Context.Draw.DrawString(b2.Vec2{X: p1.X.Sub(b2.QHalf()), Y: p1.Y}, "player 1", b2.ColorWhite)

	p2 := s.player2Id.GetPosition()
	s.Context.Draw.DrawString(b2.Vec2{X: p2.X.Sub(b2.QHalf()), Y: p2.Y}, "player 2", b2.ColorWhite)

	p3 := s.player3Id.GetPosition()
	s.Context.Draw.DrawString(b2.Vec2{X: p3.X.Sub(b2.QHalf()), Y: p3.Y}, "player 3", b2.ColorWhite)
}

const customFilterCount = 10

// CustomFilter shows how to use custom filtering.
type CustomFilter struct {
	Base

	bodyIds  [customFilterCount]b2.BodyId
	shapeIds [customFilterCount]b2.ShapeId
}

func NewCustomFilter(ctx *SampleContext) Sample {
	s := &CustomFilter{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 5}
		ctx.Camera.Zoom = 10
	}

	// Register custom filter
	s.WorldId.SetCustomFilterCallback(s.shouldCollide)

	{
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(s.WorldId, &bodyDef)
		segment := b2.Segment{Point1: b2.V2(-40, 0), Point2: b2.V2(40, 0)}

		shapeDef := b2.DefaultShapeDef()

		b2.CreateSegmentShape(groundId, &shapeDef, &segment)
	}

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	shapeDef := b2.DefaultShapeDef()
	box := b2.MakeSquare(b2.QOne())
	x := b2.F(-customFilterCount)

	for i := range customFilterCount {
		bodyDef.Position = b2.Vec2{X: x, Y: b2.F(5)}
		s.bodyIds[i] = b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef.UserData = i + 1
		s.shapeIds[i] = b2.CreatePolygonShape(s.bodyIds[i], &shapeDef, &box)
		x = x.Add(b2.F(2))
	}

	return s
}

func (s *CustomFilter) Step() {
	s.DrawTextLine("Custom filter disables collision between odd and even shapes")

	s.Base.Step()

	for i := range customFilterCount {
		p := s.bodyIds[i].GetPosition()
		s.Context.Draw.DrawString(p, fmt.Sprintf("%d", i), b2.ColorWhite)
	}
}

func (s *CustomFilter) shouldCollide(shapeIdA, shapeIdB b2.ShapeId) bool {
	userDataA := shapeIdA.GetUserData()
	userDataB := shapeIdB.GetUserData()

	if userDataA == nil || userDataB == nil {
		return true
	}

	indexA := userDataA.(int)
	indexB := userDataB.(int)

	return (indexA&1)+(indexB&1) != 1
}

const restitutionCount = 40

// Restitution is approximate since Box2D uses speculative collision.
type Restitution struct {
	Base

	bodyIds   [restitutionCount]b2.BodyId
	shapeType int
}

const (
	restitutionCircleShape = iota
	restitutionBoxShape
)

func NewRestitution(ctx *SampleContext) Sample {
	s := &Restitution{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 4, Y: 17}
		ctx.Camera.Zoom = 27.5
	}

	{
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(s.WorldId, &bodyDef)

		h := b2.F(restitutionCount)
		segment := b2.Segment{Point1: b2.Vec2{X: h.Neg()}, Point2: b2.Vec2{X: h}}
		shapeDef := b2.DefaultShapeDef()
		b2.CreateSegmentShape(groundId, &shapeDef, &segment)
	}

	s.shapeType = restitutionCircleShape

	s.createBodies()
	return s
}

func (s *Restitution) createBodies() {
	for i := range restitutionCount {
		if !s.bodyIds[i].IsNull() {
			b2.DestroyBody(s.bodyIds[i])
			s.bodyIds[i] = b2.BodyId{}
		}
	}

	circle := b2.Circle{Radius: b2.QHalf()}

	box := b2.MakeBox(b2.QHalf(), b2.QHalf())

	shapeDef := b2.DefaultShapeDef()
	shapeDef.Density = b2.QOne()
	shapeDef.Material.Restitution = b2.QZero()

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody

	dr := b2.QFromRatio(1, max(restitutionCount-1, 1))
	x := b2.QFromInt(-(restitutionCount - 1))
	dx := b2.F(2)

	for i := range restitutionCount {
		bodyDef.Position = b2.Vec2{X: x, Y: b2.F(40)}
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)

		s.bodyIds[i] = bodyId

		if s.shapeType == restitutionCircleShape {
			b2.CreateCircleShape(bodyId, &shapeDef, &circle)
		} else {
			b2.CreatePolygonShape(bodyId, &shapeDef, &box)
		}

		shapeDef.Material.Restitution = shapeDef.Material.Restitution.Add(dr)
		x = x.Add(dx)
	}
}

func (s *Restitution) UpdateGui() {
	height := 100
	gui := s.Context.Gui
	gui.Begin("Restitution", 10, s.Context.Camera.Height-height-50, 240, height)

	changed := false
	shapeTypes := []string{"Circle", "Box"}

	changed = changed || gui.Combo("Shape", &s.shapeType, shapeTypes)

	changed = changed || gui.Button("Reset")

	if changed {
		s.createBodies()
	}

	gui.End()
}

// Friction slides boxes of decreasing friction down a set of ramps.
type Friction struct {
	Base
}

func NewFriction(ctx *SampleContext) Sample {
	s := &Friction{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 14}
		ctx.Camera.Zoom = 25 * 0.6
	}

	{
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		shapeDef.Material.Friction = b2.F(0.2)

		segment := b2.Segment{Point1: b2.V2(-40, 0), Point2: b2.V2(40, 0)}
		b2.CreateSegmentShape(groundId, &shapeDef, &segment)

		// The ramp angles are 0.25 radians.
		box := b2.MakeOffsetBox(b2.F(13), b2.F(0.25), b2.V2(-4, 22), b2.MakeRot(radiansToTurns(-0.25)))
		b2.CreatePolygonShape(groundId, &shapeDef, &box)

		box = b2.MakeOffsetBox(b2.F(0.25), b2.QOne(), b2.V2(10.5, 19.0), b2.RotIdentity())
		b2.CreatePolygonShape(groundId, &shapeDef, &box)

		box = b2.MakeOffsetBox(b2.F(13), b2.F(0.25), b2.V2(4, 14), b2.MakeRot(radiansToTurns(0.25)))
		b2.CreatePolygonShape(groundId, &shapeDef, &box)

		box = b2.MakeOffsetBox(b2.F(0.25), b2.QOne(), b2.V2(-10.5, 11.0), b2.RotIdentity())
		b2.CreatePolygonShape(groundId, &shapeDef, &box)

		box = b2.MakeOffsetBox(b2.F(13), b2.F(0.25), b2.V2(-4, 6), b2.MakeRot(radiansToTurns(-0.25)))
		b2.CreatePolygonShape(groundId, &shapeDef, &box)
	}

	{
		box := b2.MakeBox(b2.QHalf(), b2.QHalf())

		shapeDef := b2.DefaultShapeDef()
		shapeDef.Density = b2.F(25)

		friction := [5]b2.Q{b2.F(0.75), b2.F(0.5), b2.F(0.35), b2.F(0.1), b2.F(0)}

		for i := range 5 {
			bodyDef := b2.DefaultBodyDef()
			bodyDef.Type = b2.DynamicBody
			bodyDef.Position = b2.Vec2{X: b2.F(-15 + 4*i), Y: b2.F(28)}
			bodyId := b2.CreateBody(s.WorldId, &bodyDef)

			shapeDef.Material.Friction = friction[i]
			b2.CreatePolygonShape(bodyId, &shapeDef, &box)
		}
	}

	return s
}

// RollingResistance rolls balls of increasing rolling resistance.
type RollingResistance struct {
	Base

	resistScale b2.Q
	lift        b2.Q
}

func NewRollingResistance(ctx *SampleContext) Sample {
	s := &RollingResistance{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 5, Y: 20}
		ctx.Camera.Zoom = 27.5
	}

	s.lift = b2.QZero()
	s.resistScale = b2.F(0.02)
	s.createScene()
	return s
}

func (s *RollingResistance) createScene() {
	circle := b2.Circle{Radius: b2.QHalf()}

	shapeDef := b2.DefaultShapeDef()

	for i := range 20 {
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(s.WorldId, &bodyDef)

		y := b2.F(2 * i)
		segment := b2.Segment{
			Point1: b2.Vec2{X: b2.F(-40), Y: y},
			Point2: b2.Vec2{X: b2.F(40), Y: y.Add(s.lift)},
		}
		b2.CreateSegmentShape(groundId, &shapeDef, &segment)

		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.Vec2{X: b2.F(-39.5), Y: y.Add(b2.F(0.75))}
		bodyDef.AngularVelocity = radiansToTurns(-10)
		bodyDef.LinearVelocity = b2.V2(5, 0)

		bodyId := b2.CreateBody(s.WorldId, &bodyDef)
		shapeDef.Material.RollingResistance = s.resistScale.Mul(b2.F(i))
		b2.CreateCircleShape(bodyId, &shapeDef, &circle)
	}
}

func (s *RollingResistance) Keyboard(key Key) {
	switch key {
	case Key1:
		s.lift = b2.QZero()
		s.CreateWorld()
		s.createScene()

	case Key2:
		s.lift = b2.F(5)
		s.CreateWorld()
		s.createScene()

	case Key3:
		s.lift = b2.F(-5)
		s.CreateWorld()
		s.createScene()

	default:
		s.Base.Keyboard(key)
	}
}

func (s *RollingResistance) Step() {
	s.Base.Step()

	for i := range 20 {
		p := b2.Vec2{X: b2.F(-41.5), Y: b2.F(2*i + 1)}
		text := fmt.Sprintf("%.2f", ToFloat64(s.resistScale.Mul(b2.F(i))))
		s.Context.Draw.DrawString(p, text, b2.ColorWhite)
	}
}

// ConveyorBelt carries boxes on a platform with a tangent speed.
type ConveyorBelt struct {
	Base
}

func NewConveyorBelt(ctx *SampleContext) Sample {
	s := &ConveyorBelt{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 2, Y: 7.5}
		ctx.Camera.Zoom = 12
	}

	// Ground
	{
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		segment := b2.Segment{Point1: b2.V2(-20, 0), Point2: b2.V2(20, 0)}
		b2.CreateSegmentShape(groundId, &shapeDef, &segment)
	}

	// Platform
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Position = b2.V2(-5, 5)
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)

		box := b2.MakeRoundedBox(b2.F(10), b2.F(0.25), b2.F(0.25))

		shapeDef := b2.DefaultShapeDef()
		shapeDef.Material.Friction = b2.F(0.8)
		shapeDef.Material.TangentSpeed = b2.F(2)

		b2.CreatePolygonShape(bodyId, &shapeDef, &box)
	}

	// Boxes
	shapeDef := b2.DefaultShapeDef()
	cube := b2.MakeSquare(b2.QHalf())
	for i := range 5 {
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.Vec2{X: b2.F(-10 + 2*i), Y: b2.F(7)}
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)

		b2.CreatePolygonShape(bodyId, &shapeDef, &cube)
	}

	return s
}

const tangentSpeedTotalCount = 200

// TangentSpeed drops balls onto a chain whose segments have increasing
// tangent speeds.
type TangentSpeed struct {
	Base

	bodyIds           []b2.BodyId
	friction          float64
	rollingResistance float64
}

func NewTangentSpeed(ctx *SampleContext) Sample {
	s := &TangentSpeed{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 60, Y: -15}
		ctx.Camera.Zoom = 38
	}

	{
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(s.WorldId, &bodyDef)

		path := "m 613.8334,185.20833 -42.33338,0 h -37.04166 l -34.39581,0 -29.10417,-2.64583 -26.45834,-7.9375 " +
			"-26.45833,-13.22917 -23.81251,-21.16666 h -13.22916 v 44.97916 H 68.791712 V 0 h -21.16671 v " +
			"206.375 l 566.208398,-1e-5 z"

		offset := b2.V2(-47.375002, 0.25)

		scale := b2.F(0.2)
		points := parsePath(path, offset, 20, scale)
		count := len(points)

		var materials [20]b2.SurfaceMaterial
		for i := range materials {
			materials[i].Friction = b2.F(0.6)
		}

		materials[0].TangentSpeed = b2.F(-10)
		materials[0].CustomColor = uint32(b2.ColorDarkBlue)
		materials[1].TangentSpeed = b2.F(-20)
		materials[1].CustomColor = uint32(b2.ColorDarkCyan)
		materials[2].TangentSpeed = b2.F(-30)
		materials[2].CustomColor = uint32(b2.ColorDarkGoldenRod)
		materials[3].TangentSpeed = b2.F(-40)
		materials[3].CustomColor = uint32(b2.ColorDarkGray)
		materials[4].TangentSpeed = b2.F(-50)
		materials[4].CustomColor = uint32(b2.ColorDarkGreen)
		materials[5].TangentSpeed = b2.F(-60)
		materials[5].CustomColor = uint32(b2.ColorDarkKhaki)
		materials[6].TangentSpeed = b2.F(-70)
		materials[6].CustomColor = uint32(b2.ColorDarkMagenta)

		chainDef := b2.DefaultChainDef()
		chainDef.Points = points
		chainDef.IsLoop = true
		chainDef.Materials = materials[:count]

		b2.CreateChain(groundId, &chainDef)

		s.friction = 0.6
		s.rollingResistance = 0.3
	}

	return s
}

func (s *TangentSpeed) dropBall() b2.BodyId {
	circle := b2.Circle{Radius: b2.QHalf()}

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.Position = b2.V2(110, -30)
	bodyId := b2.CreateBody(s.WorldId, &bodyDef)

	shapeDef := b2.DefaultShapeDef()
	shapeDef.Material.Friction = FromFloat64(s.friction)
	shapeDef.Material.RollingResistance = FromFloat64(s.rollingResistance)
	b2.CreateCircleShape(bodyId, &shapeDef, &circle)
	return bodyId
}

func (s *TangentSpeed) reset() {
	for _, bodyId := range s.bodyIds {
		b2.DestroyBody(bodyId)
	}

	s.bodyIds = s.bodyIds[:0]
}

func (s *TangentSpeed) UpdateGui() {
	height := 80
	gui := s.Context.Gui
	gui.Begin("Ball Parameters", 10, s.Context.Camera.Height-height-50, 260, height)

	if gui.SliderFloat("Friction", &s.friction, 0, 2) {
		s.reset()
	}

	if gui.SliderFloat("Rolling Resistance", &s.rollingResistance, 0, 1) {
		s.reset()
	}

	gui.End()
}

func (s *TangentSpeed) Step() {
	if s.StepCount%25 == 0 && len(s.bodyIds) < tangentSpeedTotalCount && !s.Context.Settings.Pause {
		id := s.dropBall()
		s.bodyIds = append(s.bodyIds, id)
	}

	s.Base.Step()
}

// ModifyGeometry shows how to modify the geometry on an existing shape.
// This is only supported on dynamic and kinematic shapes because static
// shapes don't look for new collisions.
type ModifyGeometry struct {
	Base

	shapeId   b2.ShapeId
	shapeType b2.ShapeType
	scale     float64
}

func NewModifyGeometry(ctx *SampleContext) Sample {
	s := &ModifyGeometry{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Zoom = 25 * 0.25
		ctx.Camera.Center = Vec2f{X: 0, Y: 5}
	}

	{
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(s.WorldId, &bodyDef)
		shapeDef := b2.DefaultShapeDef()
		box := b2.MakeOffsetBox(b2.F(10), b2.QOne(), b2.V2(0, -1), b2.RotIdentity())
		b2.CreatePolygonShape(groundId, &shapeDef, &box)
	}

	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.V2(0, 4)
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)
		shapeDef := b2.DefaultShapeDef()
		box := b2.MakeBox(b2.QOne(), b2.QOne())
		b2.CreatePolygonShape(bodyId, &shapeDef, &box)
	}

	{
		s.shapeType = b2.CircleShape
		s.scale = 1
		circle := b2.Circle{Radius: b2.QHalf()}
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.KinematicBody
		bodyDef.Position = b2.V2(0, 1)
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)
		shapeDef := b2.DefaultShapeDef()
		s.shapeId = b2.CreateCircleShape(bodyId, &shapeDef, &circle)
	}

	return s
}

func (s *ModifyGeometry) updateShape() {
	scale := FromFloat64(s.scale)
	half := b2.QHalf().Mul(scale)

	switch s.shapeType {
	case b2.CircleShape:
		circle := b2.Circle{Radius: half}
		s.shapeId.SetCircle(&circle)

	case b2.CapsuleShape:
		capsule := b2.Capsule{Center1: b2.Vec2{X: half.Neg()}, Center2: b2.Vec2{Y: half}, Radius: half}
		s.shapeId.SetCapsule(&capsule)

	case b2.SegmentShape:
		segment := b2.Segment{Point1: b2.Vec2{X: half.Neg()}, Point2: b2.Vec2{X: b2.F(0.75).Mul(scale)}}
		s.shapeId.SetSegment(&segment)

	case b2.PolygonShape:
		polygon := b2.MakeBox(half, b2.F(0.75).Mul(scale))
		s.shapeId.SetPolygon(&polygon)

	default:
		panic("samples: modify geometry has an unknown shape type")
	}

	bodyId := s.shapeId.GetBody()
	bodyId.ApplyMassFromShapes()
}

func (s *ModifyGeometry) UpdateGui() {
	height := 230
	gui := s.Context.Gui
	gui.Begin("Modify Geometry", 10, s.Context.Camera.Height-height-50, 200, height)

	if gui.RadioButton("Circle", s.shapeType == b2.CircleShape) {
		s.shapeType = b2.CircleShape
		s.updateShape()
	}

	if gui.RadioButton("Capsule", s.shapeType == b2.CapsuleShape) {
		s.shapeType = b2.CapsuleShape
		s.updateShape()
	}

	if gui.RadioButton("Segment", s.shapeType == b2.SegmentShape) {
		s.shapeType = b2.SegmentShape
		s.updateShape()
	}

	if gui.RadioButton("Polygon", s.shapeType == b2.PolygonShape) {
		s.shapeType = b2.PolygonShape
		s.updateShape()
	}

	if gui.SliderFloat("Scale", &s.scale, 0.1, 10) {
		s.updateShape()
	}

	bodyId := s.shapeId.GetBody()
	bodyType := bodyId.GetType()

	if gui.RadioButton("Static", bodyType == b2.StaticBody) {
		bodyId.SetType(b2.StaticBody)
	}

	if gui.RadioButton("Kinematic", bodyType == b2.KinematicBody) {
		bodyId.SetType(b2.KinematicBody)
	}

	if gui.RadioButton("Dynamic", bodyType == b2.DynamicBody) {
		bodyId.SetType(b2.DynamicBody)
	}

	gui.End()
}

// ChainLink shows how to link two chain shapes together. This is a useful
// technique for building large game levels with smooth collision.
type ChainLink struct {
	Base
}

func NewChainLink(ctx *SampleContext) Sample {
	s := &ChainLink{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 5}
		ctx.Camera.Zoom = 25 * 0.5
	}

	points1 := qvs(40, 1, 0, 0, -40, 0, -40, -1, 0, -1, 40, -1)
	points2 := qvs(-40, -1, 0, -1, 40, -1, 40, 0, 0, 0, -40, 0)

	bodyDef := b2.DefaultBodyDef()
	groundId := b2.CreateBody(s.WorldId, &bodyDef)

	{
		chainDef := b2.DefaultChainDef()
		chainDef.Points = points1
		chainDef.IsLoop = false
		b2.CreateChain(groundId, &chainDef)
	}

	{
		chainDef := b2.DefaultChainDef()
		chainDef.Points = points2
		chainDef.IsLoop = false
		b2.CreateChain(groundId, &chainDef)
	}

	bodyDef.Type = b2.DynamicBody
	shapeDef := b2.DefaultShapeDef()

	{
		bodyDef.Position = b2.V2(-5, 2)
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)
		circle := b2.Circle{Radius: b2.QHalf()}
		b2.CreateCircleShape(bodyId, &shapeDef, &circle)
	}

	{
		bodyDef.Position = b2.V2(0, 2)
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)
		capsule := b2.Capsule{Center1: b2.V2(-0.5, 0.0), Center2: b2.V2(0.5, 0.0), Radius: b2.F(0.25)}
		b2.CreateCapsuleShape(bodyId, &shapeDef, &capsule)
	}

	{
		bodyDef.Position = b2.V2(5, 2)
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)
		h := b2.QHalf()
		box := b2.MakeBox(h, h)
		b2.CreatePolygonShape(bodyId, &shapeDef, &box)
	}

	return s
}

func (s *ChainLink) Step() {
	s.Base.Step()

	s.DrawTextLine("This shows how to link together two chain shapes")
}

// createShapesBin is the ground and the two walls shared by the Rounded and
// Ellipse samples.
func createShapesBin(worldId b2.WorldId) {
	bodyDef := b2.DefaultBodyDef()
	groundId := b2.CreateBody(worldId, &bodyDef)

	shapeDef := b2.DefaultShapeDef()
	box := b2.MakeOffsetBox(b2.F(20), b2.QOne(), b2.V2(0, -1), b2.RotIdentity())
	b2.CreatePolygonShape(groundId, &shapeDef, &box)

	box = b2.MakeOffsetBox(b2.QOne(), b2.F(5), b2.V2(19, 5), b2.RotIdentity())
	b2.CreatePolygonShape(groundId, &shapeDef, &box)

	box = b2.MakeOffsetBox(b2.QOne(), b2.F(5), b2.V2(-19, 5), b2.RotIdentity())
	b2.CreatePolygonShape(groundId, &shapeDef, &box)
}

// RoundedShapes piles random rounded polygons in a bin.
type RoundedShapes struct {
	Base
}

func NewRoundedShapes(ctx *SampleContext) Sample {
	s := &RoundedShapes{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Zoom = 25 * 0.55
		ctx.Camera.Center = Vec2f{X: 2, Y: 8}
	}

	createShapesBin(s.WorldId)

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	shapeDef := b2.DefaultShapeDef()
	shapeDef.Material.RollingResistance = b2.F(0.3)

	y := b2.F(2)
	xcount, ycount := 10, 10

	for range ycount {
		x := b2.F(-5)
		for range xcount {
			bodyDef.Position = b2.Vec2{X: x, Y: y}
			bodyId := b2.CreateBody(s.WorldId, &bodyDef)

			poly := shared.RandomPolygon(b2.QHalf())
			poly.Radius = shared.RandomFloatRange(b2.F(0.05), b2.F(0.25))
			b2.CreatePolygonShape(bodyId, &shapeDef, &poly)

			x = x.Add(b2.QOne())
		}

		y = y.Add(b2.QOne())
	}

	return s
}

// EllipseShape piles rounded diamonds, which roll like ellipses, in a bin.
type EllipseShape struct {
	Base
}

func NewEllipseShape(ctx *SampleContext) Sample {
	s := &EllipseShape{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Zoom = 25 * 0.55
		ctx.Camera.Center = Vec2f{X: 2, Y: 8}
	}

	createShapesBin(s.WorldId)

	points := qvs(0, -0.25, 0, 0.25, 0.05, 0.075, -0.05, 0.075, 0.05, -0.075, -0.05, -0.075)
	diamondHull := b2.ComputeHull(points)
	poly := b2.MakePolygon(&diamondHull, b2.F(0.2))

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	shapeDef := b2.DefaultShapeDef()
	shapeDef.Material.RollingResistance = b2.F(0.2)

	y := b2.F(2)
	xCount, yCount := 10, 10

	for range yCount {
		x := b2.F(-5)
		for range xCount {
			bodyDef.Position = b2.Vec2{X: x, Y: y}
			bodyId := b2.CreateBody(s.WorldId, &bodyDef)
			b2.CreatePolygonShape(bodyId, &shapeDef, &poly)

			x = x.Add(b2.QOne())
		}

		y = y.Add(b2.QOne())
	}

	return s
}

// OffsetShapes has shapes placed far from their body origins.
type OffsetShapes struct {
	Base
}

func NewOffsetShapes(ctx *SampleContext) Sample {
	s := &OffsetShapes{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Zoom = 25 * 0.55
		ctx.Camera.Center = Vec2f{X: 2, Y: 8}
	}

	// Turns; the reference rotation is 0.5 pi.
	quarter := b2.MakeRot(b2.QFromRatio(1, 4))

	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Position = b2.V2(-1, 1)
		groundId := b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		box := b2.MakeOffsetBox(b2.QOne(), b2.QOne(), b2.V2(10, -2), quarter)
		b2.CreatePolygonShape(groundId, &shapeDef, &box)
	}

	{
		capsule := b2.Capsule{Center1: b2.V2(-5, 1), Center2: b2.V2(-4, 1), Radius: b2.F(0.25)}
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Position = b2.V2(13.5, -0.75)
		bodyDef.Type = b2.DynamicBody
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)
		shapeDef := b2.DefaultShapeDef()
		b2.CreateCapsuleShape(bodyId, &shapeDef, &capsule)
	}

	{
		box := b2.MakeOffsetBox(b2.F(0.75), b2.QHalf(), b2.V2(9, 2), quarter)
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Position = b2.Vec2{}
		bodyDef.Type = b2.DynamicBody
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)
		shapeDef := b2.DefaultShapeDef()
		b2.CreatePolygonShape(bodyId, &shapeDef, &box)
	}

	return s
}

func (s *OffsetShapes) Step() {
	s.Base.Step()

	s.Context.Draw.DrawTransform(b2.TransformIdentity())
}

// Explosion shows how to use explosions and demonstrates the projected
// perimeter.
type Explosion struct {
	Base

	jointIds       []b2.JointId
	radius         float64
	falloff        float64
	impulse        float64
	referenceAngle b2.Q
}

func NewExplosion(ctx *SampleContext) Sample {
	s := &Explosion{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 0}
		ctx.Camera.Zoom = 14
	}

	bodyDef := b2.DefaultBodyDef()
	groundId := b2.CreateBody(s.WorldId, &bodyDef)

	bodyDef.Type = b2.DynamicBody
	bodyDef.GravityScale = b2.QZero()
	shapeDef := b2.DefaultShapeDef()

	s.referenceAngle = b2.QZero()

	weldDef := b2.DefaultWeldJointDef()
	weldDef.ReferenceAngle = s.referenceAngle
	weldDef.AngularHertz = b2.QHalf()
	weldDef.AngularDampingRatio = b2.F(0.7)
	weldDef.LinearHertz = b2.QHalf()
	weldDef.LinearDampingRatio = b2.F(0.7)
	weldDef.BodyIdA = groundId
	weldDef.LocalAnchorB = b2.Vec2{}

	r := b2.F(8)
	for angle := 0; angle < 360; angle += 30 {
		cosSin := b2.MakeRot(b2.QFromRatio(angle, 360))
		bodyDef.Position = b2.Vec2{X: r.Mul(cosSin.Cos), Y: r.Mul(cosSin.Sin)}
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)

		box := b2.MakeBox(b2.QOne(), b2.F(0.1))
		b2.CreatePolygonShape(bodyId, &shapeDef, &box)

		weldDef.LocalAnchorA = bodyDef.Position
		weldDef.BodyIdB = bodyId
		jointId := b2.CreateWeldJoint(s.WorldId, &weldDef)
		s.jointIds = append(s.jointIds, jointId)
	}

	s.radius = 7
	s.falloff = 3
	s.impulse = 10
	return s
}

func (s *Explosion) UpdateGui() {
	height := 160
	gui := s.Context.Gui
	gui.Begin("Explosion", 10, s.Context.Camera.Height-height-50, 240, height)

	if gui.Button("Explode") {
		def := b2.DefaultExplosionDef()
		def.Position = b2.Vec2{}
		def.Radius = FromFloat64(s.radius)
		def.Falloff = FromFloat64(s.falloff)
		def.ImpulsePerLength = FromFloat64(s.impulse)
		s.WorldId.Explode(&def)
	}

	gui.SliderFloat("radius", &s.radius, 0, 20)
	gui.SliderFloat("falloff", &s.falloff, 0, 20)
	gui.SliderFloat("impulse", &s.impulse, -20, 20)

	gui.End()
}

func (s *Explosion) Step() {
	settings := s.Context.Settings
	if !settings.Pause || settings.SingleStep {
		// Turns; the reference turns 60 degrees per second.
		if settings.Hertz > 0 {
			s.referenceAngle = s.referenceAngle.Add(b2.QOne().Div(FromFloat64(6 * settings.Hertz)))
		}
		s.referenceAngle = b2.UnwindAngle(s.referenceAngle)

		for _, jointId := range s.jointIds {
			jointId.SetReferenceAngle(s.referenceAngle)
		}
	}

	s.Base.Step()

	s.DrawTextLine("reference angle = %g", 2*math.Pi*ToFloat64(s.referenceAngle))

	s.Context.Draw.DrawCircle(b2.Vec2{}, FromFloat64(s.radius+s.falloff), b2.ColorBox2DBlue)
	s.Context.Draw.DrawCircle(b2.Vec2{}, FromFloat64(s.radius), b2.ColorBox2DYellow)
}

// RecreateStatic tests a static shape being recreated every step.
type RecreateStatic struct {
	Base

	groundId b2.BodyId
}

func NewRecreateStatic(ctx *SampleContext) Sample {
	s := &RecreateStatic{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 2.5}
		ctx.Camera.Zoom = 3.5
	}

	bodyDef := b2.DefaultBodyDef()
	shapeDef := b2.DefaultShapeDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.Position = b2.V2(0, 1)
	bodyId := b2.CreateBody(s.WorldId, &bodyDef)

	box := b2.MakeBox(b2.QOne(), b2.QOne())
	b2.CreatePolygonShape(bodyId, &shapeDef, &box)

	s.groundId = b2.BodyId{}
	return s
}

func (s *RecreateStatic) Step() {
	if !s.groundId.IsNull() {
		b2.DestroyBody(s.groundId)
		s.groundId = b2.BodyId{}
	}

	bodyDef := b2.DefaultBodyDef()
	s.groundId = b2.CreateBody(s.WorldId, &bodyDef)

	shapeDef := b2.DefaultShapeDef()

	// Invoke contact creation so that contact points are created immediately
	// on a static body.
	shapeDef.InvokeContactCreation = true

	segment := b2.Segment{Point1: b2.V2(-10, 0), Point2: b2.V2(10, 0)}
	b2.CreateSegmentShape(s.groundId, &shapeDef, &segment)

	s.Base.Step()
}
