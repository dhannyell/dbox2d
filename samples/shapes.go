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

// qvs builds a point list from string pairs.
func qvs(coords ...string) []dbox2d.Vec2 {
	points := make([]dbox2d.Vec2, len(coords)/2)
	for i := range points {
		points[i] = qv(coords[2*i], coords[2*i+1])
	}
	return points
}

// ChainShape rolls a shape along a closed chain.
type ChainShape struct {
	Base

	groundId    dbox2d.BodyId
	bodyId      dbox2d.BodyId
	chainId     dbox2d.ChainId
	shapeType   int
	shapeId     dbox2d.ShapeId
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
		dbox2d.DestroyBody(s.groundId)
	}

	// https://betravis.github.io/shape-tools/path-to-polygon/
	points := qvs(
		"-56.885498", "12.8985004", "-56.885498", "16.2057495", "56.885498", "16.2057495", "56.885498", "-16.2057514",
		"51.5935059", "-16.2057514", "43.6559982", "-10.9139996", "35.7184982", "-10.9139996", "27.7809982", "-10.9139996",
		"21.1664963", "-14.2212505", "11.9059982", "-16.2057514", "0", "-16.2057514", "-10.5835037", "-14.8827496",
		"-17.1980019", "-13.5597477", "-21.1665001", "-12.2370014", "-25.1355019", "-9.5909977", "-31.75", "-3.63799858",
		"-38.3644981", "6.2840004", "-42.3334999", "9.59125137", "-47.625", "11.5755005", "-56.885498", "12.8985004",
	)

	material := dbox2d.SurfaceMaterial{}
	material.Friction = qs("0.2")
	material.CustomColor = uint32(dbox2d.ColorSteelBlue)
	material.UserMaterialId = 42

	chainDef := dbox2d.DefaultChainDef()
	chainDef.Points = points
	chainDef.Materials = []dbox2d.SurfaceMaterial{material}
	chainDef.IsLoop = true

	bodyDef := dbox2d.DefaultBodyDef()
	s.groundId = dbox2d.CreateBody(s.WorldId, &bodyDef)

	s.chainId = dbox2d.CreateChain(s.groundId, &chainDef)
}

func (s *ChainShape) launch() {
	if !s.bodyId.IsNull() {
		dbox2d.DestroyBody(s.bodyId)
	}

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	bodyDef.Position = qv("-55", "13.5")
	s.bodyId = dbox2d.CreateBody(s.WorldId, &bodyDef)

	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Density = dbox2d.QOne()
	shapeDef.Material.Friction = FromFloat64(s.friction)
	shapeDef.Material.Restitution = FromFloat64(s.restitution)

	switch s.shapeType {
	case chainCircleShape:
		circle := dbox2d.Circle{Radius: dbox2d.QHalf()}
		s.shapeId = dbox2d.CreateCircleShape(s.bodyId, &shapeDef, &circle)

	case chainCapsuleShape:
		capsule := dbox2d.Capsule{Center1: qv("-0.5", "0"), Center2: qv("0.5", "0"), Radius: qs("0.25")}
		s.shapeId = dbox2d.CreateCapsuleShape(s.bodyId, &shapeDef, &capsule)

	default:
		h := dbox2d.QHalf()
		box := dbox2d.MakeBox(h, h)
		s.shapeId = dbox2d.CreatePolygonShape(s.bodyId, &shapeDef, &box)
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

	s.Context.Draw.DrawSegment(dbox2d.Vec2{}, qv("0.5", "0"), dbox2d.ColorRed)
	s.Context.Draw.DrawSegment(dbox2d.Vec2{}, qv("0", "0.5"), dbox2d.ColorGreen)
}

// CompoundShapes shows how careful creation of compound shapes leads to
// better simulation and avoids objects getting stuck. It also shows how to
// get the combined AABB for the body.
type CompoundShapes struct {
	Base

	table1Id      dbox2d.BodyId
	table2Id      dbox2d.BodyId
	ship1Id       dbox2d.BodyId
	ship2Id       dbox2d.BodyId
	drawBodyAABBs bool
	aabbVertices  [4]dbox2d.Vec2
}

func NewCompoundShapes(ctx *SampleContext) Sample {
	s := &CompoundShapes{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 6}
		ctx.Camera.Zoom = 25 * 0.5
	}

	{
		bodyDef := dbox2d.DefaultBodyDef()
		groundId := dbox2d.CreateBody(s.WorldId, &bodyDef)
		shapeDef := dbox2d.DefaultShapeDef()
		segment := dbox2d.Segment{Point1: qv("50", "0"), Point2: qv("-50", "0")}
		dbox2d.CreateSegmentShape(groundId, &shapeDef, &segment)
	}

	// Table 1
	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = qv("-15", "1")
		s.table1Id = dbox2d.CreateBody(s.WorldId, &bodyDef)

		shapeDef := dbox2d.DefaultShapeDef()
		top := dbox2d.MakeOffsetBox(dbox2d.QFromInt(3), dbox2d.QHalf(), qv("0", "3.5"), dbox2d.RotIdentity())
		leftLeg := dbox2d.MakeOffsetBox(dbox2d.QHalf(), qs("1.5"), qv("-2.5", "1.5"), dbox2d.RotIdentity())
		rightLeg := dbox2d.MakeOffsetBox(dbox2d.QHalf(), qs("1.5"), qv("2.5", "1.5"), dbox2d.RotIdentity())

		dbox2d.CreatePolygonShape(s.table1Id, &shapeDef, &top)
		dbox2d.CreatePolygonShape(s.table1Id, &shapeDef, &leftLeg)
		dbox2d.CreatePolygonShape(s.table1Id, &shapeDef, &rightLeg)
	}

	// Table 2
	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = qv("-5", "1")
		s.table2Id = dbox2d.CreateBody(s.WorldId, &bodyDef)

		shapeDef := dbox2d.DefaultShapeDef()
		top := dbox2d.MakeOffsetBox(dbox2d.QFromInt(3), dbox2d.QHalf(), qv("0", "3.5"), dbox2d.RotIdentity())
		leftLeg := dbox2d.MakeOffsetBox(dbox2d.QHalf(), dbox2d.QFromInt(2), qv("-2.5", "2"), dbox2d.RotIdentity())
		rightLeg := dbox2d.MakeOffsetBox(dbox2d.QHalf(), dbox2d.QFromInt(2), qv("2.5", "2"), dbox2d.RotIdentity())

		dbox2d.CreatePolygonShape(s.table2Id, &shapeDef, &top)
		dbox2d.CreatePolygonShape(s.table2Id, &shapeDef, &leftLeg)
		dbox2d.CreatePolygonShape(s.table2Id, &shapeDef, &rightLeg)
	}

	// Spaceship 1
	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = qv("5", "1")
		s.ship1Id = dbox2d.CreateBody(s.WorldId, &bodyDef)

		shapeDef := dbox2d.DefaultShapeDef()
		var vertices [3]dbox2d.Vec2

		vertices[0] = qv("-2", "0")
		vertices[1] = dbox2d.Vec2{Y: dbox2d.QFromRatio(4, 3)}
		vertices[2] = qv("0", "4")
		hull := dbox2d.ComputeHull(vertices[:])
		left := dbox2d.MakePolygon(&hull, dbox2d.QZero())

		vertices[0] = qv("2", "0")
		vertices[1] = dbox2d.Vec2{Y: dbox2d.QFromRatio(4, 3)}
		vertices[2] = qv("0", "4")
		hull = dbox2d.ComputeHull(vertices[:])
		right := dbox2d.MakePolygon(&hull, dbox2d.QZero())

		dbox2d.CreatePolygonShape(s.ship1Id, &shapeDef, &left)
		dbox2d.CreatePolygonShape(s.ship1Id, &shapeDef, &right)
	}

	// Spaceship 2
	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = qv("15", "1")
		s.ship2Id = dbox2d.CreateBody(s.WorldId, &bodyDef)

		shapeDef := dbox2d.DefaultShapeDef()
		var vertices [3]dbox2d.Vec2

		vertices[0] = qv("-2", "0")
		vertices[1] = qv("1", "2")
		vertices[2] = qv("0", "4")
		hull := dbox2d.ComputeHull(vertices[:])
		left := dbox2d.MakePolygon(&hull, dbox2d.QZero())

		vertices[0] = qv("2", "0")
		vertices[1] = qv("-1", "2")
		vertices[2] = qv("0", "4")
		hull = dbox2d.ComputeHull(vertices[:])
		right := dbox2d.MakePolygon(&hull, dbox2d.QZero())

		dbox2d.CreatePolygonShape(s.ship2Id, &shapeDef, &left)
		dbox2d.CreatePolygonShape(s.ship2Id, &shapeDef, &right)
	}

	s.drawBodyAABBs = false
	return s
}

func (s *CompoundShapes) spawn() {
	// Table 1 obstruction
	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = s.table1Id.GetPosition()
		bodyDef.Rotation = s.table1Id.GetRotation()
		bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		shapeDef := dbox2d.DefaultShapeDef()
		box := dbox2d.MakeOffsetBox(dbox2d.QFromInt(4), qs("0.1"), qv("0", "3"), dbox2d.RotIdentity())
		dbox2d.CreatePolygonShape(bodyId, &shapeDef, &box)
	}

	// Table 2 obstruction
	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = s.table2Id.GetPosition()
		bodyDef.Rotation = s.table2Id.GetRotation()
		bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		shapeDef := dbox2d.DefaultShapeDef()
		box := dbox2d.MakeOffsetBox(dbox2d.QFromInt(4), qs("0.1"), qv("0", "3"), dbox2d.RotIdentity())
		dbox2d.CreatePolygonShape(bodyId, &shapeDef, &box)
	}

	// Ship 1 obstruction
	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = s.ship1Id.GetPosition()
		bodyDef.Rotation = s.ship1Id.GetRotation()
		bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		shapeDef := dbox2d.DefaultShapeDef()
		circle := dbox2d.Circle{Center: qv("0", "2"), Radius: dbox2d.QHalf()}
		dbox2d.CreateCircleShape(bodyId, &shapeDef, &circle)
	}

	// Ship 2 obstruction
	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = s.ship2Id.GetPosition()
		bodyDef.Rotation = s.ship2Id.GetRotation()
		bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		shapeDef := dbox2d.DefaultShapeDef()
		circle := dbox2d.Circle{Center: qv("0", "2"), Radius: dbox2d.QHalf()}
		dbox2d.CreateCircleShape(bodyId, &shapeDef, &circle)
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
func (s *CompoundShapes) drawAABB(box dbox2d.AABB, color dbox2d.HexColor) {
	v := &s.aabbVertices
	v[0] = box.LowerBound
	v[1] = dbox2d.Vec2{X: box.UpperBound.X, Y: box.LowerBound.Y}
	v[2] = box.UpperBound
	v[3] = dbox2d.Vec2{X: box.LowerBound.X, Y: box.UpperBound.Y}
	s.Context.Draw.DrawPolygon(v[:], color)
}

func (s *CompoundShapes) Step() {
	s.Base.Step()

	if s.drawBodyAABBs {
		s.drawAABB(s.table1Id.ComputeAABB(), dbox2d.ColorYellow)
		s.drawAABB(s.table2Id.ComputeAABB(), dbox2d.ColorYellow)
		s.drawAABB(s.ship1Id.ComputeAABB(), dbox2d.ColorYellow)
		s.drawAABB(s.ship2Id.ComputeAABB(), dbox2d.ColorYellow)
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

	player1Id dbox2d.BodyId
	player2Id dbox2d.BodyId
	player3Id dbox2d.BodyId

	shape1Id dbox2d.ShapeId
	shape2Id dbox2d.ShapeId
	shape3Id dbox2d.ShapeId

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
		bodyDef := dbox2d.DefaultBodyDef()
		groundId := dbox2d.CreateBody(s.WorldId, &bodyDef)
		segment := dbox2d.Segment{Point1: qv("-20", "0"), Point2: qv("20", "0")}

		shapeDef := dbox2d.DefaultShapeDef()
		shapeDef.Filter.CategoryBits = filterGround
		shapeDef.Filter.MaskBits = filterAllBits

		dbox2d.CreateSegmentShape(groundId, &shapeDef, &segment)
	}

	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody

		bodyDef.Position = qv("0", "2")
		s.player1Id = dbox2d.CreateBody(s.WorldId, &bodyDef)

		bodyDef.Position = qv("0", "5")
		s.player2Id = dbox2d.CreateBody(s.WorldId, &bodyDef)

		bodyDef.Position = qv("0", "8")
		s.player3Id = dbox2d.CreateBody(s.WorldId, &bodyDef)

		box := dbox2d.MakeBox(dbox2d.QFromInt(2), dbox2d.QOne())

		shapeDef := dbox2d.DefaultShapeDef()

		shapeDef.Filter.CategoryBits = filterTeam1
		shapeDef.Filter.MaskBits = filterGround | filterTeam2 | filterTeam3
		s.shape1Id = dbox2d.CreatePolygonShape(s.player1Id, &shapeDef, &box)

		shapeDef.Filter.CategoryBits = filterTeam2
		shapeDef.Filter.MaskBits = filterGround | filterTeam1 | filterTeam3
		s.shape2Id = dbox2d.CreatePolygonShape(s.player2Id, &shapeDef, &box)

		shapeDef.Filter.CategoryBits = filterTeam3
		shapeDef.Filter.MaskBits = filterGround | filterTeam1 | filterTeam2
		s.shape3Id = dbox2d.CreatePolygonShape(s.player3Id, &shapeDef, &box)
	}

	return s
}

// teamCheckbox shows one mask bit of a shape filter as a checkbox and
// writes the filter back when it is toggled.
func teamCheckbox(gui Gui, label string, state *bool, shapeId dbox2d.ShapeId, team uint64) {
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
	s.Context.Draw.DrawString(dbox2d.Vec2{X: p1.X.Sub(dbox2d.QHalf()), Y: p1.Y}, "player 1", dbox2d.ColorWhite)

	p2 := s.player2Id.GetPosition()
	s.Context.Draw.DrawString(dbox2d.Vec2{X: p2.X.Sub(dbox2d.QHalf()), Y: p2.Y}, "player 2", dbox2d.ColorWhite)

	p3 := s.player3Id.GetPosition()
	s.Context.Draw.DrawString(dbox2d.Vec2{X: p3.X.Sub(dbox2d.QHalf()), Y: p3.Y}, "player 3", dbox2d.ColorWhite)
}

const customFilterCount = 10

// CustomFilter shows how to use custom filtering.
type CustomFilter struct {
	Base

	bodyIds  [customFilterCount]dbox2d.BodyId
	shapeIds [customFilterCount]dbox2d.ShapeId
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
		bodyDef := dbox2d.DefaultBodyDef()
		groundId := dbox2d.CreateBody(s.WorldId, &bodyDef)
		segment := dbox2d.Segment{Point1: qv("-40", "0"), Point2: qv("40", "0")}

		shapeDef := dbox2d.DefaultShapeDef()

		dbox2d.CreateSegmentShape(groundId, &shapeDef, &segment)
	}

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	shapeDef := dbox2d.DefaultShapeDef()
	box := dbox2d.MakeSquare(dbox2d.QOne())
	x := dbox2d.QFromInt(-customFilterCount)

	for i := range customFilterCount {
		bodyDef.Position = dbox2d.Vec2{X: x, Y: dbox2d.QFromInt(5)}
		s.bodyIds[i] = dbox2d.CreateBody(s.WorldId, &bodyDef)

		shapeDef.UserData = i + 1
		s.shapeIds[i] = dbox2d.CreatePolygonShape(s.bodyIds[i], &shapeDef, &box)
		x = x.Add(dbox2d.QFromInt(2))
	}

	return s
}

func (s *CustomFilter) Step() {
	s.DrawTextLine("Custom filter disables collision between odd and even shapes")

	s.Base.Step()

	for i := range customFilterCount {
		p := s.bodyIds[i].GetPosition()
		s.Context.Draw.DrawString(p, fmt.Sprintf("%d", i), dbox2d.ColorWhite)
	}
}

func (s *CustomFilter) shouldCollide(shapeIdA, shapeIdB dbox2d.ShapeId) bool {
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

	bodyIds   [restitutionCount]dbox2d.BodyId
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
		bodyDef := dbox2d.DefaultBodyDef()
		groundId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		h := dbox2d.QFromInt(restitutionCount)
		segment := dbox2d.Segment{Point1: dbox2d.Vec2{X: h.Neg()}, Point2: dbox2d.Vec2{X: h}}
		shapeDef := dbox2d.DefaultShapeDef()
		dbox2d.CreateSegmentShape(groundId, &shapeDef, &segment)
	}

	s.shapeType = restitutionCircleShape

	s.createBodies()
	return s
}

func (s *Restitution) createBodies() {
	for i := range restitutionCount {
		if !s.bodyIds[i].IsNull() {
			dbox2d.DestroyBody(s.bodyIds[i])
			s.bodyIds[i] = dbox2d.BodyId{}
		}
	}

	circle := dbox2d.Circle{Radius: dbox2d.QHalf()}

	box := dbox2d.MakeBox(dbox2d.QHalf(), dbox2d.QHalf())

	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Density = dbox2d.QOne()
	shapeDef.Material.Restitution = dbox2d.QZero()

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody

	dr := dbox2d.QFromRatio(1, max(restitutionCount-1, 1))
	x := dbox2d.QFromInt(-(restitutionCount - 1))
	dx := dbox2d.QFromInt(2)

	for i := range restitutionCount {
		bodyDef.Position = dbox2d.Vec2{X: x, Y: dbox2d.QFromInt(40)}
		bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		s.bodyIds[i] = bodyId

		if s.shapeType == restitutionCircleShape {
			dbox2d.CreateCircleShape(bodyId, &shapeDef, &circle)
		} else {
			dbox2d.CreatePolygonShape(bodyId, &shapeDef, &box)
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
		bodyDef := dbox2d.DefaultBodyDef()
		groundId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		shapeDef := dbox2d.DefaultShapeDef()
		shapeDef.Material.Friction = qs("0.2")

		segment := dbox2d.Segment{Point1: qv("-40", "0"), Point2: qv("40", "0")}
		dbox2d.CreateSegmentShape(groundId, &shapeDef, &segment)

		// The ramp angles are 0.25 radians.
		box := dbox2d.MakeOffsetBox(dbox2d.QFromInt(13), qs("0.25"), qv("-4", "22"), dbox2d.MakeRot(radiansToTurns(-0.25)))
		dbox2d.CreatePolygonShape(groundId, &shapeDef, &box)

		box = dbox2d.MakeOffsetBox(qs("0.25"), dbox2d.QOne(), qv("10.5", "19"), dbox2d.RotIdentity())
		dbox2d.CreatePolygonShape(groundId, &shapeDef, &box)

		box = dbox2d.MakeOffsetBox(dbox2d.QFromInt(13), qs("0.25"), qv("4", "14"), dbox2d.MakeRot(radiansToTurns(0.25)))
		dbox2d.CreatePolygonShape(groundId, &shapeDef, &box)

		box = dbox2d.MakeOffsetBox(qs("0.25"), dbox2d.QOne(), qv("-10.5", "11"), dbox2d.RotIdentity())
		dbox2d.CreatePolygonShape(groundId, &shapeDef, &box)

		box = dbox2d.MakeOffsetBox(dbox2d.QFromInt(13), qs("0.25"), qv("-4", "6"), dbox2d.MakeRot(radiansToTurns(-0.25)))
		dbox2d.CreatePolygonShape(groundId, &shapeDef, &box)
	}

	{
		box := dbox2d.MakeBox(dbox2d.QHalf(), dbox2d.QHalf())

		shapeDef := dbox2d.DefaultShapeDef()
		shapeDef.Density = dbox2d.QFromInt(25)

		friction := [5]dbox2d.Q{qs("0.75"), qs("0.5"), qs("0.35"), qs("0.1"), qs("0")}

		for i := range 5 {
			bodyDef := dbox2d.DefaultBodyDef()
			bodyDef.Type = dbox2d.DynamicBody
			bodyDef.Position = dbox2d.Vec2{X: dbox2d.QFromInt(-15 + 4*i), Y: dbox2d.QFromInt(28)}
			bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)

			shapeDef.Material.Friction = friction[i]
			dbox2d.CreatePolygonShape(bodyId, &shapeDef, &box)
		}
	}

	return s
}

// RollingResistance rolls balls of increasing rolling resistance.
type RollingResistance struct {
	Base

	resistScale dbox2d.Q
	lift        dbox2d.Q
}

func NewRollingResistance(ctx *SampleContext) Sample {
	s := &RollingResistance{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 5, Y: 20}
		ctx.Camera.Zoom = 27.5
	}

	s.lift = dbox2d.QZero()
	s.resistScale = qs("0.02")
	s.createScene()
	return s
}

func (s *RollingResistance) createScene() {
	circle := dbox2d.Circle{Radius: dbox2d.QHalf()}

	shapeDef := dbox2d.DefaultShapeDef()

	for i := range 20 {
		bodyDef := dbox2d.DefaultBodyDef()
		groundId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		y := dbox2d.QFromInt(2 * i)
		segment := dbox2d.Segment{
			Point1: dbox2d.Vec2{X: dbox2d.QFromInt(-40), Y: y},
			Point2: dbox2d.Vec2{X: dbox2d.QFromInt(40), Y: y.Add(s.lift)},
		}
		dbox2d.CreateSegmentShape(groundId, &shapeDef, &segment)

		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = dbox2d.Vec2{X: qs("-39.5"), Y: y.Add(qs("0.75"))}
		bodyDef.AngularVelocity = radiansToTurns(-10)
		bodyDef.LinearVelocity = qv("5", "0")

		bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)
		shapeDef.Material.RollingResistance = s.resistScale.Mul(dbox2d.QFromInt(i))
		dbox2d.CreateCircleShape(bodyId, &shapeDef, &circle)
	}
}

func (s *RollingResistance) Keyboard(key Key) {
	switch key {
	case Key1:
		s.lift = dbox2d.QZero()
		s.CreateWorld()
		s.createScene()

	case Key2:
		s.lift = dbox2d.QFromInt(5)
		s.CreateWorld()
		s.createScene()

	case Key3:
		s.lift = dbox2d.QFromInt(-5)
		s.CreateWorld()
		s.createScene()

	default:
		s.Base.Keyboard(key)
	}
}

func (s *RollingResistance) Step() {
	s.Base.Step()

	for i := range 20 {
		p := dbox2d.Vec2{X: qs("-41.5"), Y: dbox2d.QFromInt(2*i + 1)}
		text := fmt.Sprintf("%.2f", ToFloat64(s.resistScale.Mul(dbox2d.QFromInt(i))))
		s.Context.Draw.DrawString(p, text, dbox2d.ColorWhite)
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
		bodyDef := dbox2d.DefaultBodyDef()
		groundId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		shapeDef := dbox2d.DefaultShapeDef()
		segment := dbox2d.Segment{Point1: qv("-20", "0"), Point2: qv("20", "0")}
		dbox2d.CreateSegmentShape(groundId, &shapeDef, &segment)
	}

	// Platform
	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Position = qv("-5", "5")
		bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		box := dbox2d.MakeRoundedBox(dbox2d.QFromInt(10), qs("0.25"), qs("0.25"))

		shapeDef := dbox2d.DefaultShapeDef()
		shapeDef.Material.Friction = qs("0.8")
		shapeDef.Material.TangentSpeed = dbox2d.QFromInt(2)

		dbox2d.CreatePolygonShape(bodyId, &shapeDef, &box)
	}

	// Boxes
	shapeDef := dbox2d.DefaultShapeDef()
	cube := dbox2d.MakeSquare(dbox2d.QHalf())
	for i := range 5 {
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = dbox2d.Vec2{X: dbox2d.QFromInt(-10 + 2*i), Y: dbox2d.QFromInt(7)}
		bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		dbox2d.CreatePolygonShape(bodyId, &shapeDef, &cube)
	}

	return s
}

const tangentSpeedTotalCount = 200

// TangentSpeed drops balls onto a chain whose segments have increasing
// tangent speeds.
type TangentSpeed struct {
	Base

	bodyIds           []dbox2d.BodyId
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
		bodyDef := dbox2d.DefaultBodyDef()
		groundId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		path := "m 613.8334,185.20833 -42.33338,0 h -37.04166 l -34.39581,0 -29.10417,-2.64583 -26.45834,-7.9375 " +
			"-26.45833,-13.22917 -23.81251,-21.16666 h -13.22916 v 44.97916 H 68.791712 V 0 h -21.16671 v " +
			"206.375 l 566.208398,-1e-5 z"

		offset := qv("-47.375002", "0.25")

		scale := qs("0.2")
		points := parsePath(path, offset, 20, scale)
		count := len(points)

		var materials [20]dbox2d.SurfaceMaterial
		for i := range materials {
			materials[i].Friction = qs("0.6")
		}

		materials[0].TangentSpeed = dbox2d.QFromInt(-10)
		materials[0].CustomColor = uint32(dbox2d.ColorDarkBlue)
		materials[1].TangentSpeed = dbox2d.QFromInt(-20)
		materials[1].CustomColor = uint32(dbox2d.ColorDarkCyan)
		materials[2].TangentSpeed = dbox2d.QFromInt(-30)
		materials[2].CustomColor = uint32(dbox2d.ColorDarkGoldenRod)
		materials[3].TangentSpeed = dbox2d.QFromInt(-40)
		materials[3].CustomColor = uint32(dbox2d.ColorDarkGray)
		materials[4].TangentSpeed = dbox2d.QFromInt(-50)
		materials[4].CustomColor = uint32(dbox2d.ColorDarkGreen)
		materials[5].TangentSpeed = dbox2d.QFromInt(-60)
		materials[5].CustomColor = uint32(dbox2d.ColorDarkKhaki)
		materials[6].TangentSpeed = dbox2d.QFromInt(-70)
		materials[6].CustomColor = uint32(dbox2d.ColorDarkMagenta)

		chainDef := dbox2d.DefaultChainDef()
		chainDef.Points = points
		chainDef.IsLoop = true
		chainDef.Materials = materials[:count]

		dbox2d.CreateChain(groundId, &chainDef)

		s.friction = 0.6
		s.rollingResistance = 0.3
	}

	return s
}

func (s *TangentSpeed) dropBall() dbox2d.BodyId {
	circle := dbox2d.Circle{Radius: dbox2d.QHalf()}

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	bodyDef.Position = qv("110", "-30")
	bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)

	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Material.Friction = FromFloat64(s.friction)
	shapeDef.Material.RollingResistance = FromFloat64(s.rollingResistance)
	dbox2d.CreateCircleShape(bodyId, &shapeDef, &circle)
	return bodyId
}

func (s *TangentSpeed) reset() {
	for _, bodyId := range s.bodyIds {
		dbox2d.DestroyBody(bodyId)
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

	shapeId   dbox2d.ShapeId
	shapeType dbox2d.ShapeType
	scale     float64
}

func NewModifyGeometry(ctx *SampleContext) Sample {
	s := &ModifyGeometry{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Zoom = 25 * 0.25
		ctx.Camera.Center = Vec2f{X: 0, Y: 5}
	}

	{
		bodyDef := dbox2d.DefaultBodyDef()
		groundId := dbox2d.CreateBody(s.WorldId, &bodyDef)
		shapeDef := dbox2d.DefaultShapeDef()
		box := dbox2d.MakeOffsetBox(dbox2d.QFromInt(10), dbox2d.QOne(), qv("0", "-1"), dbox2d.RotIdentity())
		dbox2d.CreatePolygonShape(groundId, &shapeDef, &box)
	}

	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = qv("0", "4")
		bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)
		shapeDef := dbox2d.DefaultShapeDef()
		box := dbox2d.MakeBox(dbox2d.QOne(), dbox2d.QOne())
		dbox2d.CreatePolygonShape(bodyId, &shapeDef, &box)
	}

	{
		s.shapeType = dbox2d.CircleShape
		s.scale = 1
		circle := dbox2d.Circle{Radius: dbox2d.QHalf()}
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.KinematicBody
		bodyDef.Position = qv("0", "1")
		bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)
		shapeDef := dbox2d.DefaultShapeDef()
		s.shapeId = dbox2d.CreateCircleShape(bodyId, &shapeDef, &circle)
	}

	return s
}

func (s *ModifyGeometry) updateShape() {
	scale := FromFloat64(s.scale)
	half := dbox2d.QHalf().Mul(scale)

	switch s.shapeType {
	case dbox2d.CircleShape:
		circle := dbox2d.Circle{Radius: half}
		s.shapeId.SetCircle(&circle)

	case dbox2d.CapsuleShape:
		capsule := dbox2d.Capsule{Center1: dbox2d.Vec2{X: half.Neg()}, Center2: dbox2d.Vec2{Y: half}, Radius: half}
		s.shapeId.SetCapsule(&capsule)

	case dbox2d.SegmentShape:
		segment := dbox2d.Segment{Point1: dbox2d.Vec2{X: half.Neg()}, Point2: dbox2d.Vec2{X: qs("0.75").Mul(scale)}}
		s.shapeId.SetSegment(&segment)

	case dbox2d.PolygonShape:
		polygon := dbox2d.MakeBox(half, qs("0.75").Mul(scale))
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

	if gui.RadioButton("Circle", s.shapeType == dbox2d.CircleShape) {
		s.shapeType = dbox2d.CircleShape
		s.updateShape()
	}

	if gui.RadioButton("Capsule", s.shapeType == dbox2d.CapsuleShape) {
		s.shapeType = dbox2d.CapsuleShape
		s.updateShape()
	}

	if gui.RadioButton("Segment", s.shapeType == dbox2d.SegmentShape) {
		s.shapeType = dbox2d.SegmentShape
		s.updateShape()
	}

	if gui.RadioButton("Polygon", s.shapeType == dbox2d.PolygonShape) {
		s.shapeType = dbox2d.PolygonShape
		s.updateShape()
	}

	if gui.SliderFloat("Scale", &s.scale, 0.1, 10) {
		s.updateShape()
	}

	bodyId := s.shapeId.GetBody()
	bodyType := bodyId.GetType()

	if gui.RadioButton("Static", bodyType == dbox2d.StaticBody) {
		bodyId.SetType(dbox2d.StaticBody)
	}

	if gui.RadioButton("Kinematic", bodyType == dbox2d.KinematicBody) {
		bodyId.SetType(dbox2d.KinematicBody)
	}

	if gui.RadioButton("Dynamic", bodyType == dbox2d.DynamicBody) {
		bodyId.SetType(dbox2d.DynamicBody)
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

	points1 := qvs("40", "1", "0", "0", "-40", "0", "-40", "-1", "0", "-1", "40", "-1")
	points2 := qvs("-40", "-1", "0", "-1", "40", "-1", "40", "0", "0", "0", "-40", "0")

	bodyDef := dbox2d.DefaultBodyDef()
	groundId := dbox2d.CreateBody(s.WorldId, &bodyDef)

	{
		chainDef := dbox2d.DefaultChainDef()
		chainDef.Points = points1
		chainDef.IsLoop = false
		dbox2d.CreateChain(groundId, &chainDef)
	}

	{
		chainDef := dbox2d.DefaultChainDef()
		chainDef.Points = points2
		chainDef.IsLoop = false
		dbox2d.CreateChain(groundId, &chainDef)
	}

	bodyDef.Type = dbox2d.DynamicBody
	shapeDef := dbox2d.DefaultShapeDef()

	{
		bodyDef.Position = qv("-5", "2")
		bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)
		circle := dbox2d.Circle{Radius: dbox2d.QHalf()}
		dbox2d.CreateCircleShape(bodyId, &shapeDef, &circle)
	}

	{
		bodyDef.Position = qv("0", "2")
		bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)
		capsule := dbox2d.Capsule{Center1: qv("-0.5", "0"), Center2: qv("0.5", "0"), Radius: qs("0.25")}
		dbox2d.CreateCapsuleShape(bodyId, &shapeDef, &capsule)
	}

	{
		bodyDef.Position = qv("5", "2")
		bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)
		h := dbox2d.QHalf()
		box := dbox2d.MakeBox(h, h)
		dbox2d.CreatePolygonShape(bodyId, &shapeDef, &box)
	}

	return s
}

func (s *ChainLink) Step() {
	s.Base.Step()

	s.DrawTextLine("This shows how to link together two chain shapes")
}

// createShapesBin is the ground and the two walls shared by the Rounded and
// Ellipse samples.
func createShapesBin(worldId dbox2d.WorldId) {
	bodyDef := dbox2d.DefaultBodyDef()
	groundId := dbox2d.CreateBody(worldId, &bodyDef)

	shapeDef := dbox2d.DefaultShapeDef()
	box := dbox2d.MakeOffsetBox(dbox2d.QFromInt(20), dbox2d.QOne(), qv("0", "-1"), dbox2d.RotIdentity())
	dbox2d.CreatePolygonShape(groundId, &shapeDef, &box)

	box = dbox2d.MakeOffsetBox(dbox2d.QOne(), dbox2d.QFromInt(5), qv("19", "5"), dbox2d.RotIdentity())
	dbox2d.CreatePolygonShape(groundId, &shapeDef, &box)

	box = dbox2d.MakeOffsetBox(dbox2d.QOne(), dbox2d.QFromInt(5), qv("-19", "5"), dbox2d.RotIdentity())
	dbox2d.CreatePolygonShape(groundId, &shapeDef, &box)
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

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Material.RollingResistance = qs("0.3")

	y := dbox2d.QFromInt(2)
	xcount, ycount := 10, 10

	for range ycount {
		x := dbox2d.QFromInt(-5)
		for range xcount {
			bodyDef.Position = dbox2d.Vec2{X: x, Y: y}
			bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)

			poly := shared.RandomPolygon(dbox2d.QHalf())
			poly.Radius = shared.RandomFloatRange(qs("0.05"), qs("0.25"))
			dbox2d.CreatePolygonShape(bodyId, &shapeDef, &poly)

			x = x.Add(dbox2d.QOne())
		}

		y = y.Add(dbox2d.QOne())
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

	points := qvs("0", "-0.25", "0", "0.25", "0.05", "0.075", "-0.05", "0.075", "0.05", "-0.075", "-0.05", "-0.075")
	diamondHull := dbox2d.ComputeHull(points)
	poly := dbox2d.MakePolygon(&diamondHull, qs("0.2"))

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Material.RollingResistance = qs("0.2")

	y := dbox2d.QFromInt(2)
	xCount, yCount := 10, 10

	for range yCount {
		x := dbox2d.QFromInt(-5)
		for range xCount {
			bodyDef.Position = dbox2d.Vec2{X: x, Y: y}
			bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)
			dbox2d.CreatePolygonShape(bodyId, &shapeDef, &poly)

			x = x.Add(dbox2d.QOne())
		}

		y = y.Add(dbox2d.QOne())
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
	quarter := dbox2d.MakeRot(dbox2d.QFromRatio(1, 4))

	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Position = qv("-1", "1")
		groundId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		shapeDef := dbox2d.DefaultShapeDef()
		box := dbox2d.MakeOffsetBox(dbox2d.QOne(), dbox2d.QOne(), qv("10", "-2"), quarter)
		dbox2d.CreatePolygonShape(groundId, &shapeDef, &box)
	}

	{
		capsule := dbox2d.Capsule{Center1: qv("-5", "1"), Center2: qv("-4", "1"), Radius: qs("0.25")}
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Position = qv("13.5", "-0.75")
		bodyDef.Type = dbox2d.DynamicBody
		bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)
		shapeDef := dbox2d.DefaultShapeDef()
		dbox2d.CreateCapsuleShape(bodyId, &shapeDef, &capsule)
	}

	{
		box := dbox2d.MakeOffsetBox(qs("0.75"), dbox2d.QHalf(), qv("9", "2"), quarter)
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Position = dbox2d.Vec2{}
		bodyDef.Type = dbox2d.DynamicBody
		bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)
		shapeDef := dbox2d.DefaultShapeDef()
		dbox2d.CreatePolygonShape(bodyId, &shapeDef, &box)
	}

	return s
}

func (s *OffsetShapes) Step() {
	s.Base.Step()

	s.Context.Draw.DrawTransform(dbox2d.TransformIdentity())
}

// Explosion shows how to use explosions and demonstrates the projected
// perimeter.
type Explosion struct {
	Base

	jointIds       []dbox2d.JointId
	radius         float64
	falloff        float64
	impulse        float64
	referenceAngle dbox2d.Q
}

func NewExplosion(ctx *SampleContext) Sample {
	s := &Explosion{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 0}
		ctx.Camera.Zoom = 14
	}

	bodyDef := dbox2d.DefaultBodyDef()
	groundId := dbox2d.CreateBody(s.WorldId, &bodyDef)

	bodyDef.Type = dbox2d.DynamicBody
	bodyDef.GravityScale = dbox2d.QZero()
	shapeDef := dbox2d.DefaultShapeDef()

	s.referenceAngle = dbox2d.QZero()

	weldDef := dbox2d.DefaultWeldJointDef()
	weldDef.ReferenceAngle = s.referenceAngle
	weldDef.AngularHertz = dbox2d.QHalf()
	weldDef.AngularDampingRatio = qs("0.7")
	weldDef.LinearHertz = dbox2d.QHalf()
	weldDef.LinearDampingRatio = qs("0.7")
	weldDef.BodyIdA = groundId
	weldDef.LocalAnchorB = dbox2d.Vec2{}

	r := dbox2d.QFromInt(8)
	for angle := 0; angle < 360; angle += 30 {
		cosSin := dbox2d.MakeRot(dbox2d.QFromRatio(angle, 360))
		bodyDef.Position = dbox2d.Vec2{X: r.Mul(cosSin.Cos), Y: r.Mul(cosSin.Sin)}
		bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		box := dbox2d.MakeBox(dbox2d.QOne(), qs("0.1"))
		dbox2d.CreatePolygonShape(bodyId, &shapeDef, &box)

		weldDef.LocalAnchorA = bodyDef.Position
		weldDef.BodyIdB = bodyId
		jointId := dbox2d.CreateWeldJoint(s.WorldId, &weldDef)
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
		def := dbox2d.DefaultExplosionDef()
		def.Position = dbox2d.Vec2{}
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
			s.referenceAngle = s.referenceAngle.Add(dbox2d.QOne().Div(FromFloat64(6 * settings.Hertz)))
		}
		s.referenceAngle = dbox2d.UnwindAngle(s.referenceAngle)

		for _, jointId := range s.jointIds {
			jointId.SetReferenceAngle(s.referenceAngle)
		}
	}

	s.Base.Step()

	s.DrawTextLine("reference angle = %g", 2*math.Pi*ToFloat64(s.referenceAngle))

	s.Context.Draw.DrawCircle(dbox2d.Vec2{}, FromFloat64(s.radius+s.falloff), dbox2d.ColorBox2DBlue)
	s.Context.Draw.DrawCircle(dbox2d.Vec2{}, FromFloat64(s.radius), dbox2d.ColorBox2DYellow)
}

// RecreateStatic tests a static shape being recreated every step.
type RecreateStatic struct {
	Base

	groundId dbox2d.BodyId
}

func NewRecreateStatic(ctx *SampleContext) Sample {
	s := &RecreateStatic{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 2.5}
		ctx.Camera.Zoom = 3.5
	}

	bodyDef := dbox2d.DefaultBodyDef()
	shapeDef := dbox2d.DefaultShapeDef()
	bodyDef.Type = dbox2d.DynamicBody
	bodyDef.Position = qv("0", "1")
	bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)

	box := dbox2d.MakeBox(dbox2d.QOne(), dbox2d.QOne())
	dbox2d.CreatePolygonShape(bodyId, &shapeDef, &box)

	s.groundId = dbox2d.BodyId{}
	return s
}

func (s *RecreateStatic) Step() {
	if !s.groundId.IsNull() {
		dbox2d.DestroyBody(s.groundId)
		s.groundId = dbox2d.BodyId{}
	}

	bodyDef := dbox2d.DefaultBodyDef()
	s.groundId = dbox2d.CreateBody(s.WorldId, &bodyDef)

	shapeDef := dbox2d.DefaultShapeDef()

	// Invoke contact creation so that contact points are created immediately
	// on a static body.
	shapeDef.InvokeContactCreation = true

	segment := dbox2d.Segment{Point1: qv("-10", "0"), Point2: qv("10", "0")}
	dbox2d.CreateSegmentShape(s.groundId, &shapeDef, &segment)

	s.Base.Step()
}
