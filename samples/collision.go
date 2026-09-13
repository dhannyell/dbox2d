// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/sample_collision.cpp of Box2D v3.1.1

package samples

import (
	"fmt"
	"math"
	"time"

	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/dbox2d/internal/shared"
)

func init() {
	RegisterSample("Collision", "Shape Distance", NewCollisionShapeDistance)
	RegisterSample("Collision", "Dynamic Tree", NewDynamicTree)
	RegisterSample("Collision", "Ray Cast", NewRayCast)
	RegisterSample("Collision", "Cast World", NewCastWorld)
	RegisterSample("Collision", "Overlap World", NewOverlapWorld)
	RegisterSample("Collision", "Manifold", NewManifold)
	RegisterSample("Collision", "Smooth Manifold", NewSmoothManifold)
	RegisterSample("Collision", "Shape Cast", NewShapeCast)
	RegisterSample("Collision", "Time of Impact", NewTimeOfImpact)
}

// rotFromRadians builds a rotation from a GUI angle, which stays in
// radians like the reference slider.
func rotFromRadians(angle float64) b2.Rot { return b2.MakeRot(radiansToTurns(angle)) }

func clampFloat(a, lower, upper float64) float64 { return math.Max(lower, math.Min(a, upper)) }

// drawSolidCircleAt draws a solid circle whose center is local to the
// transform, like the reference Draw::DrawSolidCircle.
func drawSolidCircleAt(draw *b2.DebugDraw, transform b2.Transform, center b2.Vec2, radius b2.Q, color b2.HexColor) {
	draw.DrawSolidCircle(b2.Transform{P: b2.TransformPoint(transform, center), Q: transform.Q}, radius, color)
}

// The proxy shapes of Shape Distance and Shape Cast.
const (
	proxyPoint = iota
	proxySegment
	proxyTriangle
	proxyBox
)

var proxyShapeNames = []string{"point", "segment", "triangle", "box"}

type proxyShapes struct {
	point    b2.Vec2
	segment  b2.Segment
	triangle b2.Polygon
	box      b2.Polygon
}

func (ps *proxyShapes) makeProxy(shapeType int, radius b2.Q) b2.ShapeProxy {
	proxy := b2.ShapeProxy{Radius: radius}

	switch shapeType {
	case proxyPoint:
		proxy.Points[0] = b2.Vec2{}
		proxy.Count = 1

	case proxySegment:
		proxy.Points[0] = ps.segment.Point1
		proxy.Points[1] = ps.segment.Point2
		proxy.Count = 2

	case proxyTriangle:
		copy(proxy.Points[:], ps.triangle.Vertices[:ps.triangle.Count])
		proxy.Count = ps.triangle.Count

	case proxyBox:
		copy(proxy.Points[:4], ps.box.Vertices[:4])
		proxy.Count = 4
	}

	return proxy
}

func (ps *proxyShapes) drawShape(draw *b2.DebugDraw, shapeType int, transform b2.Transform, radius b2.Q, color b2.HexColor) {
	switch shapeType {
	case proxyPoint:
		p := b2.TransformPoint(transform, ps.point)
		if radius.Greater(b2.QZero()) {
			drawSolidCircleAt(draw, transform, ps.point, radius, color)
		} else {
			draw.DrawPoint(p, b2.F(5), color)
		}

	case proxySegment:
		p1 := b2.TransformPoint(transform, ps.segment.Point1)
		p2 := b2.TransformPoint(transform, ps.segment.Point2)

		if radius.Greater(b2.QZero()) {
			draw.DrawSolidCapsule(p1, p2, radius, color)
		} else {
			draw.DrawSegment(p1, p2, color)
		}

	case proxyTriangle:
		draw.DrawSolidPolygon(transform, ps.triangle.Vertices[:ps.triangle.Count], radius, color)

	case proxyBox:
		draw.DrawSolidPolygon(transform, ps.box.Vertices[:ps.box.Count], radius, color)
	}
}

// sliderQ edits a scalar with a float GUI slider.
func sliderQ(gui Gui, label string, v *b2.Q, lo, hi float64) bool {
	f := ToFloat64(*v)
	if gui.SliderFloat(label, &f, lo, hi) {
		*v = FromFloat64(f)
		return true
	}
	return false
}

const shapeDistanceSimplexCapacity = 20

// CollisionShapeDistance shows the GJK distance between two proxies. It is
// the reference's ShapeDistance; the name ShapeDistance belongs to the
// benchmark.
type CollisionShapeDistance struct {
	Base

	shapes proxyShapes

	typeA, typeB     int
	radiusA, radiusB float64
	proxyA, proxyB   b2.ShapeProxy

	cache        b2.SimplexCache
	simplexes    [shapeDistanceSimplexCapacity]b2.Simplex
	simplexCount int
	simplexIndex int

	transform b2.Transform
	angle     float64

	basePosition b2.Vec2
	startPoint   b2.Vec2
	baseAngle    float64

	dragging    bool
	rotating    bool
	showIndices bool
	useCache    bool
	drawSimplex bool
}

func NewCollisionShapeDistance(ctx *SampleContext) Sample {
	s := &CollisionShapeDistance{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 0}
		ctx.Camera.Zoom = 3
	}

	s.shapes.segment = b2.Segment{Point1: b2.V2(-0.5, 0.0), Point2: b2.V2(0.5, 0.0)}

	{
		hull := b2.ComputeHull([]b2.Vec2{b2.V2(-0.5, 0.0), b2.V2(0.5, 0.0), b2.V2(0, 1)})
		s.shapes.triangle = b2.MakePolygon(&hull, b2.QZero())
	}

	s.shapes.box = b2.MakeSquare(b2.QHalf())

	s.transform = b2.TransformIdentity()

	s.typeA = proxyBox
	s.typeB = proxyBox

	s.proxyA = s.shapes.makeProxy(s.typeA, FromFloat64(s.radiusA))
	s.proxyB = s.shapes.makeProxy(s.typeB, FromFloat64(s.radiusB))
	return s
}

func (s *CollisionShapeDistance) UpdateGui() {
	gui := s.Context.Gui
	height := 310
	gui.Begin("Shape Distance", 10, s.Context.Camera.Height-height-50, 240, height)

	if gui.Combo("shape A", &s.typeA, proxyShapeNames) {
		s.proxyA = s.shapes.makeProxy(s.typeA, FromFloat64(s.radiusA))
	}

	if gui.SliderFloat("radius A", &s.radiusA, 0, 0.5) {
		s.proxyA.Radius = FromFloat64(s.radiusA)
	}

	if gui.Combo("shape B", &s.typeB, proxyShapeNames) {
		s.proxyB = s.shapes.makeProxy(s.typeB, FromFloat64(s.radiusB))
	}

	if gui.SliderFloat("radius B", &s.radiusB, 0, 0.5) {
		s.proxyB.Radius = FromFloat64(s.radiusB)
	}

	sliderQ(gui, "x offset", &s.transform.P.X, -2, 2)
	sliderQ(gui, "y offset", &s.transform.P.Y, -2, 2)

	if gui.SliderFloat("angle", &s.angle, -math.Pi, math.Pi) {
		s.transform.Q = rotFromRadians(s.angle)
	}

	gui.Checkbox("show indices", &s.showIndices)
	gui.Checkbox("use cache", &s.useCache)

	if gui.Checkbox("draw simplex", &s.drawSimplex) {
		s.simplexIndex = 0
	}

	if s.drawSimplex {
		gui.SliderInt("index", &s.simplexIndex, 0, s.simplexCount-1)
		s.simplexIndex = b2.ClampInt(s.simplexIndex, 0, s.simplexCount-1)
	}

	gui.End()
}

func (s *CollisionShapeDistance) MouseDown(p b2.Vec2, button MouseButton, mod Modifier) {
	if button == MouseButtonLeft {
		if mod == 0 && !s.rotating {
			s.dragging = true
			s.startPoint = p
			s.basePosition = s.transform.P
		} else if mod == ModShift && !s.dragging {
			s.rotating = true
			s.startPoint = p
			s.baseAngle = s.angle
		}
	}
}

func (s *CollisionShapeDistance) MouseUp(p b2.Vec2, button MouseButton) {
	if button == MouseButtonLeft {
		s.dragging = false
		s.rotating = false
	}
}

func (s *CollisionShapeDistance) MouseMove(p b2.Vec2) {
	if s.dragging {
		s.transform.P = s.basePosition.Add(p.Sub(s.startPoint).Mul(b2.QHalf()))
	} else if s.rotating {
		dx := ToFloat64(p.X.Sub(s.startPoint.X))
		s.angle = clampFloat(s.baseAngle+dx, -math.Pi, math.Pi)
		s.transform.Q = rotFromRadians(s.angle)
	}
}

func weight2(a1 b2.Q, w1 b2.Vec2, a2 b2.Q, w2 b2.Vec2) b2.Vec2 {
	return b2.Vec2{X: a1.Mul(w1.X).Add(a2.Mul(w2.X)), Y: a1.Mul(w1.Y).Add(a2.Mul(w2.Y))}
}

func weight3(a1 b2.Q, w1 b2.Vec2, a2 b2.Q, w2 b2.Vec2, a3 b2.Q, w3 b2.Vec2) b2.Vec2 {
	return b2.Vec2{
		X: a1.Mul(w1.X).Add(a2.Mul(w2.X)).Add(a3.Mul(w3.X)),
		Y: a1.Mul(w1.Y).Add(a2.Mul(w2.Y)).Add(a3.Mul(w3.Y)),
	}
}

func computeSimplexWitnessPoints(s *b2.Simplex) (a, b b2.Vec2) {
	switch s.Count {
	case 1:
		a = s.V1.WA
		b = s.V1.WB

	case 2:
		a = weight2(s.V1.A, s.V1.WA, s.V2.A, s.V2.WA)
		b = weight2(s.V1.A, s.V1.WB, s.V2.A, s.V2.WB)

	case 3:
		a = weight3(s.V1.A, s.V1.WA, s.V2.A, s.V2.WA, s.V3.A, s.V3.WA)
		b = a
	}
	return a, b
}

func (s *CollisionShapeDistance) Step() {
	input := b2.DistanceInput{
		ProxyA:     s.proxyA,
		ProxyB:     s.proxyB,
		TransformA: b2.TransformIdentity(),
		TransformB: s.transform,
		UseRadii:   true,
	}

	if !s.useCache {
		s.cache.Count = 0
	}

	output := b2.ShapeDistance(&input, &s.cache, s.simplexes[:])

	s.simplexCount = output.SimplexCount

	draw := &s.Context.Draw
	s.shapes.drawShape(draw, s.typeA, b2.TransformIdentity(), FromFloat64(s.radiusA), b2.ColorCyan)
	s.shapes.drawShape(draw, s.typeB, s.transform, FromFloat64(s.radiusB), b2.ColorBisque)

	ten := b2.F(10)

	if s.drawSimplex && s.simplexIndex >= 0 && s.simplexIndex < s.simplexCount {
		simplex := &s.simplexes[s.simplexIndex]
		vertices := [3]*b2.SimplexVertex{&simplex.V1, &simplex.V2, &simplex.V3}

		if s.simplexIndex > 0 {
			// The first recorded simplex does not have valid barycentric coordinates
			pointA, pointB := computeSimplexWitnessPoints(simplex)

			draw.DrawSegment(pointA, pointB, b2.ColorWhite)
			draw.DrawPoint(pointA, ten, b2.ColorWhite)
			draw.DrawPoint(pointB, ten, b2.ColorWhite)
		}

		colors := [3]b2.HexColor{b2.ColorRed, b2.ColorGreen, b2.ColorBlue}

		for i := 0; i < simplex.Count; i++ {
			vertex := vertices[i]
			draw.DrawPoint(vertex.WA, ten, colors[i])
			draw.DrawPoint(vertex.WB, ten, colors[i])
		}
	} else {
		draw.DrawSegment(output.PointA, output.PointB, b2.ColorDimGray)
		draw.DrawPoint(output.PointA, ten, b2.ColorWhite)
		draw.DrawPoint(output.PointB, ten, b2.ColorWhite)

		draw.DrawSegment(output.PointA, b2.MulAdd(output.PointA, b2.QHalf(), output.Normal), b2.ColorYellow)
	}

	if s.showIndices {
		for i := 0; i < s.proxyA.Count; i++ {
			p := s.proxyA.Points[i]
			draw.DrawString(p, fmt.Sprintf(" %d", i), b2.ColorWhite)
		}

		for i := 0; i < s.proxyB.Count; i++ {
			p := b2.TransformPoint(s.transform, s.proxyB.Points[i])
			draw.DrawString(p, fmt.Sprintf(" %d", i), b2.ColorWhite)
		}
	}

	s.DrawTextLine("mouse button 1: drag")
	s.DrawTextLine("mouse button 1 + shift: rotate")
	s.DrawTextLine("distance = %.2f, iterations = %d", ToFloat64(output.Distance), output.Iterations)

	c := &s.cache
	switch c.Count {
	case 1:
		s.DrawTextLine("cache = {%d}, {%d}", c.IndexA[0], c.IndexB[0])
	case 2:
		s.DrawTextLine("cache = {%d, %d}, {%d, %d}", c.IndexA[0], c.IndexA[1], c.IndexB[0], c.IndexB[1])
	case 3:
		s.DrawTextLine("cache = {%d, %d, %d}, {%d, %d, %d}", c.IndexA[0], c.IndexA[1], c.IndexA[2],
			c.IndexB[0], c.IndexB[1], c.IndexB[2])
	}
}

// The update modes of the Dynamic Tree sample.
const (
	updateIncremental = iota
	updateFullRebuild
	updatePartialRebuild
)

type treeProxy struct {
	box        b2.AABB
	fatBox     b2.AABB
	position   b2.Vec2
	width      b2.Vec2
	proxyId    int
	rayStamp   int
	queryStamp int
	moved      bool
}

// DynamicTree tests the Box2D bounding volume hierarchy (BVH). The dynamic
// tree can be used independently as a spatial data structure.
type DynamicTree struct {
	Base

	tree        *b2.DynamicTree
	rowCount    int
	columnCount int
	proxies     []treeProxy
	timeStamp   int
	updateType  int

	fill         float64
	moveFraction float64
	moveDelta    float64
	ratio        float64
	grid         float64

	startPoint b2.Vec2
	endPoint   b2.Vec2

	rayDrag   bool
	queryDrag bool
	validate  bool

	// boxVertices is the scratch of drawAABB.
	boxVertices [4]b2.Vec2
}

func NewDynamicTree(ctx *SampleContext) Sample {
	s := &DynamicTree{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 500, Y: 500}
		ctx.Camera.Zoom = 25 * 21
	}

	s.fill = 0.25
	s.moveFraction = 0.05
	s.moveDelta = 0.1
	s.ratio = 5
	s.grid = 1

	// The reference's release counts.
	s.rowCount = 1000
	s.columnCount = 1000
	s.buildTree()
	s.timeStamp = 0
	s.updateType = updateIncremental

	s.validate = true
	return s
}

func (s *DynamicTree) Destroy() {
	if s.tree != nil {
		s.tree.Destroy()
		s.tree = nil
	}
	s.Base.Destroy()
}

func (s *DynamicTree) buildTree() {
	if s.tree != nil {
		s.tree.Destroy()
	}

	s.proxies = make([]treeProxy, 0, s.rowCount*s.columnCount)

	y := b2.F(-4)

	s.tree = b2.NewDynamicTree()

	aabbMargin := b2.Vec2{X: b2.QFromRatio(1, 10), Y: b2.QFromRatio(1, 10)}

	fill := FromFloat64(s.fill)
	grid := FromFloat64(s.grid)
	maxRatio := FromFloat64(s.ratio)

	for i := 0; i < s.rowCount; i++ {
		x := b2.F(-40)

		for j := 0; j < s.columnCount; j++ {
			fillTest := shared.RandomFloatRange(b2.QZero(), b2.QOne())
			if !fillTest.Greater(fill) {
				var p treeProxy
				p.position = b2.Vec2{X: x, Y: y}

				ratio := shared.RandomFloatRange(b2.QOne(), maxRatio)
				width := shared.RandomFloatRange(b2.QFromRatio(1, 10), b2.QHalf())
				if shared.RandomFloat().Greater(b2.QZero()) {
					p.width.X = ratio.Mul(width)
					p.width.Y = width
				} else {
					p.width.X = width
					p.width.Y = ratio.Mul(width)
				}

				p.box.LowerBound = b2.Vec2{X: x, Y: y}
				p.box.UpperBound = b2.Vec2{X: x.Add(p.width.X), Y: y.Add(p.width.Y)}
				p.fatBox.LowerBound = p.box.LowerBound.Sub(aabbMargin)
				p.fatBox.UpperBound = p.box.UpperBound.Add(aabbMargin)

				p.proxyId = s.tree.CreateProxy(p.fatBox, b2.DefaultCategoryBits, uint64(len(s.proxies)))
				p.rayStamp = -1
				p.queryStamp = -1
				p.moved = false
				s.proxies = append(s.proxies, p)
			}

			x = x.Add(grid)
		}

		y = y.Add(grid)
	}
}

func (s *DynamicTree) UpdateGui() {
	gui := s.Context.Gui
	height := 320
	gui.Begin("Dynamic Tree", 10, s.Context.Camera.Height-height-50, 200, height)

	changed := false
	if gui.SliderInt("rows", &s.rowCount, 0, 1000) {
		changed = true
	}

	if gui.SliderInt("columns", &s.columnCount, 0, 1000) {
		changed = true
	}

	if gui.SliderFloat("fill", &s.fill, 0, 1) {
		changed = true
	}

	if gui.SliderFloat("grid", &s.grid, 0.5, 2) {
		changed = true
	}

	if gui.SliderFloat("ratio", &s.ratio, 1, 10) {
		changed = true
	}

	gui.SliderFloat("move", &s.moveFraction, 0, 1)
	gui.SliderFloat("delta", &s.moveDelta, 0, 1)

	if gui.RadioButton("Incremental", s.updateType == updateIncremental) {
		s.updateType = updateIncremental
		changed = true
	}

	if gui.RadioButton("Full Rebuild", s.updateType == updateFullRebuild) {
		s.updateType = updateFullRebuild
		changed = true
	}

	if gui.RadioButton("Partial Rebuild", s.updateType == updatePartialRebuild) {
		s.updateType = updatePartialRebuild
		changed = true
	}

	gui.Text("mouse button 1: ray cast")
	gui.Text("mouse button 1 + shift: query")

	gui.End()

	if changed {
		s.buildTree()
	}
}

func (s *DynamicTree) MouseDown(p b2.Vec2, button MouseButton, mod Modifier) {
	if button == MouseButtonLeft {
		if mod == 0 && !s.queryDrag {
			s.rayDrag = true
			s.startPoint = p
			s.endPoint = p
		} else if mod == ModShift && !s.rayDrag {
			s.queryDrag = true
			s.startPoint = p
			s.endPoint = p
		}
	}
}

func (s *DynamicTree) MouseUp(p b2.Vec2, button MouseButton) {
	if button == MouseButtonLeft {
		s.queryDrag = false
		s.rayDrag = false
	}
}

func (s *DynamicTree) MouseMove(p b2.Vec2) {
	s.endPoint = p
}

// drawAABB draws a box outline, like the reference Draw::DrawAABB.
func (s *DynamicTree) drawAABB(box b2.AABB, color b2.HexColor) {
	v := &s.boxVertices
	v[0] = box.LowerBound
	v[1] = b2.Vec2{X: box.UpperBound.X, Y: box.LowerBound.Y}
	v[2] = box.UpperBound
	v[3] = b2.Vec2{X: box.LowerBound.X, Y: box.UpperBound.Y}
	s.Context.Draw.DrawPolygon(v[:], color)
}

func elapsedMilliseconds(start time.Time) float64 {
	return float64(time.Since(start).Nanoseconds()) / 1e6
}

func (s *DynamicTree) Step() {
	draw := &s.Context.Draw

	if s.queryDrag {
		box := b2.AABB{LowerBound: b2.Min(s.startPoint, s.endPoint), UpperBound: b2.Max(s.startPoint, s.endPoint)}
		s.tree.Query(box, b2.DefaultMaskBits, func(proxyId int, userData uint64) bool {
			proxy := &s.proxies[userData]
			proxy.queryStamp = s.timeStamp
			return true
		})

		s.drawAABB(box, b2.ColorWhite)
	}

	if s.rayDrag {
		input := b2.RayCastInput{Origin: s.startPoint, Translation: s.endPoint.Sub(s.startPoint), MaxFraction: b2.QOne()}
		result := s.tree.RayCast(&input, b2.DefaultMaskBits, func(input *b2.RayCastInput, proxyId int, userData uint64) b2.Q {
			proxy := &s.proxies[userData]
			proxy.rayStamp = s.timeStamp
			return input.MaxFraction
		})

		draw.DrawSegment(s.startPoint, s.endPoint, b2.ColorWhite)
		draw.DrawPoint(s.startPoint, b2.F(5), b2.ColorGreen)
		draw.DrawPoint(s.endPoint, b2.F(5), b2.ColorRed)

		s.DrawTextLine("node visits = %d, leaf visits = %d", result.NodeVisits, result.LeafVisits)
	}

	c := b2.ColorBlue
	qc := b2.ColorGreen

	aabbMargin := b2.Vec2{X: b2.QFromRatio(1, 10), Y: b2.QFromRatio(1, 10)}
	moveFraction := FromFloat64(s.moveFraction)
	moveDelta := FromFloat64(s.moveDelta)

	for i := range s.proxies {
		p := &s.proxies[i]

		if p.queryStamp == s.timeStamp || p.rayStamp == s.timeStamp {
			s.drawAABB(p.box, qc)
		} else {
			s.drawAABB(p.box, c)
		}

		moveTest := shared.RandomFloatRange(b2.QZero(), b2.QOne())
		if moveFraction.Greater(moveTest) {
			dx := moveDelta.Mul(shared.RandomFloat())
			dy := moveDelta.Mul(shared.RandomFloat())

			p.position.X = p.position.X.Add(dx)
			p.position.Y = p.position.Y.Add(dy)

			p.box.LowerBound.X = p.position.X.Add(dx)
			p.box.LowerBound.Y = p.position.Y.Add(dy)
			p.box.UpperBound.X = p.position.X.Add(dx).Add(p.width.X)
			p.box.UpperBound.Y = p.position.Y.Add(dy).Add(p.width.Y)

			if !b2.AABBContains(p.fatBox, p.box) {
				p.fatBox.LowerBound = p.box.LowerBound.Sub(aabbMargin)
				p.fatBox.UpperBound = p.box.UpperBound.Add(aabbMargin)
				p.moved = true
			} else {
				p.moved = false
			}
		} else {
			p.moved = false
		}
	}

	switch s.updateType {
	case updateIncremental:
		start := time.Now()
		for i := range s.proxies {
			p := &s.proxies[i]
			if p.moved {
				s.tree.MoveProxy(p.proxyId, p.fatBox)
			}
		}
		s.DrawTextLine("incremental : %.3f ms", elapsedMilliseconds(start))

	case updateFullRebuild:
		for i := range s.proxies {
			p := &s.proxies[i]
			if p.moved {
				s.tree.EnlargeProxy(p.proxyId, p.fatBox)
			}
		}

		start := time.Now()
		boxCount := s.tree.Rebuild(true)
		s.DrawTextLine("full build %d : %.3f ms", boxCount, elapsedMilliseconds(start))

	case updatePartialRebuild:
		for i := range s.proxies {
			p := &s.proxies[i]
			if p.moved {
				s.tree.EnlargeProxy(p.proxyId, p.fatBox)
			}
		}

		start := time.Now()
		boxCount := s.tree.Rebuild(false)
		s.DrawTextLine("partial rebuild %d : %.3f ms", boxCount, elapsedMilliseconds(start))
	}

	height := s.tree.GetHeight()
	areaRatio := s.tree.GetAreaRatio()

	proxyCount := len(s.proxies)
	hmin := 0
	if proxyCount > 0 {
		hmin = int(math.Ceil(math.Log(float64(proxyCount))/math.Log(2) - 1))
	}
	s.DrawTextLine("proxies = %d, height = %d, hmin = %d, area ratio = %.1f", proxyCount, height, hmin, ToFloat64(areaRatio))

	s.tree.Validate()

	s.timeStamp += 1
}

// RayCast casts a ray against each shape type in its local frame.
type RayCast struct {
	Base

	box      b2.Polygon
	triangle b2.Polygon
	circle   b2.Circle
	capsule  b2.Capsule
	segment  b2.Segment

	transform b2.Transform
	angle     float64

	rayStart b2.Vec2
	rayEnd   b2.Vec2

	basePosition b2.Vec2
	baseAngle    float64

	startPosition b2.Vec2

	rayDrag      bool
	translating  bool
	rotating     bool
	showFraction bool
}

func NewRayCast(ctx *SampleContext) Sample {
	s := &RayCast{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 20}
		ctx.Camera.Zoom = 17.5
	}

	s.circle = b2.Circle{Radius: b2.F(2)}
	s.capsule = b2.Capsule{Center1: b2.V2(-1, 1), Center2: b2.V2(1, -1), Radius: b2.F(1.5)}
	s.box = b2.MakeBox(b2.F(2), b2.F(2))

	hull := b2.ComputeHull([]b2.Vec2{b2.V2(-2, 0), b2.V2(2, 0), b2.V2(2, 3)})
	s.triangle = b2.MakePolygon(&hull, b2.QZero())

	s.segment = b2.Segment{Point1: b2.V2(-3, 0), Point2: b2.V2(3, 0)}

	s.transform = b2.TransformIdentity()

	s.rayStart = b2.V2(0, 30)
	s.rayEnd = b2.V2(0, 0)
	return s
}

func (s *RayCast) UpdateGui() {
	gui := s.Context.Gui
	height := 230
	gui.Begin("Ray-cast", 10, s.Context.Camera.Height-height-50, 200, height)

	sliderQ(gui, "x offset", &s.transform.P.X, -2, 2)
	sliderQ(gui, "y offset", &s.transform.P.Y, -2, 2)

	if gui.SliderFloat("angle", &s.angle, -math.Pi, math.Pi) {
		s.transform.Q = rotFromRadians(s.angle)
	}

	gui.Checkbox("show fraction", &s.showFraction)

	if gui.Button("Reset") {
		s.transform = b2.TransformIdentity()
		s.angle = 0
	}

	gui.Text("mouse btn 1: ray cast")
	gui.Text("mouse btn 1 + shft: translate")
	gui.Text("mouse btn 1 + ctrl: rotate")

	gui.End()
}

func (s *RayCast) MouseDown(p b2.Vec2, button MouseButton, mod Modifier) {
	if button == MouseButtonLeft {
		s.startPosition = p

		if mod == 0 {
			s.rayStart = p
			s.rayDrag = true
		} else if mod == ModShift {
			s.translating = true
			s.basePosition = s.transform.P
		} else if mod == ModControl {
			s.rotating = true
			s.baseAngle = s.angle
		}
	}
}

func (s *RayCast) MouseUp(p b2.Vec2, button MouseButton) {
	if button == MouseButtonLeft {
		s.rayDrag = false
		s.rotating = false
		s.translating = false
	}
}

func (s *RayCast) MouseMove(p b2.Vec2) {
	if s.rayDrag {
		s.rayEnd = p
	} else if s.translating {
		s.transform.P = s.basePosition.Add(p.Sub(s.startPosition).Mul(b2.QHalf()))
	} else if s.rotating {
		dx := ToFloat64(p.X.Sub(s.startPosition.X))
		s.angle = clampFloat(s.baseAngle+0.5*dx, -math.Pi, math.Pi)
		s.transform.Q = rotFromRadians(s.angle)
	}
}

func (s *RayCast) drawRay(output *b2.CastOutput) {
	draw := &s.Context.Draw
	five := b2.F(5)

	p1 := s.rayStart
	p2 := s.rayEnd
	d := p2.Sub(p1)

	if output.Hit {
		var p b2.Vec2

		if output.Fraction.Eq(b2.QZero()) {
			p = output.Point
			draw.DrawPoint(output.Point, five, b2.ColorPeru)
		} else {
			p = b2.MulAdd(p1, output.Fraction, d)
			draw.DrawSegment(p1, p, b2.ColorWhite)
			draw.DrawPoint(p1, five, b2.ColorGreen)
			draw.DrawPoint(output.Point, five, b2.ColorWhite)

			n := b2.MulAdd(p, b2.QOne(), output.Normal)
			draw.DrawSegment(p, n, b2.ColorViolet)
		}

		if s.showFraction {
			ps := b2.Vec2{X: p.X.Add(b2.F(0.05)), Y: p.Y.Sub(b2.F(0.02))}
			draw.DrawString(ps, fmt.Sprintf("%.2f", ToFloat64(output.Fraction)), b2.ColorWhite)
		}
	} else {
		draw.DrawSegment(p1, p2, b2.ColorWhite)
		draw.DrawPoint(p1, five, b2.ColorGreen)
		draw.DrawPoint(p2, five, b2.ColorRed)
	}
}

func (s *RayCast) Step() {
	draw := &s.Context.Draw

	offset := b2.V2(-20, 20)
	increment := b2.V2(10, 0)

	color1 := b2.ColorYellow

	output := b2.CastOutput{}
	maxFraction := b2.QOne()

	// cast runs a ray cast in the local frame of the transform and keeps
	// the closest hit in world space.
	cast := func(transform b2.Transform, rayCast func(input *b2.RayCastInput) b2.CastOutput) {
		start := b2.InvTransformPoint(transform, s.rayStart)
		translation := b2.InvRotateVector(transform.Q, s.rayEnd.Sub(s.rayStart))
		input := b2.RayCastInput{Origin: start, Translation: translation, MaxFraction: maxFraction}

		localOutput := rayCast(&input)
		if localOutput.Hit {
			output = localOutput
			output.Point = b2.TransformPoint(transform, localOutput.Point)
			output.Normal = b2.RotateVector(transform.Q, localOutput.Normal)
			maxFraction = localOutput.Fraction
		}
	}

	// circle
	{
		transform := b2.Transform{P: s.transform.P.Add(offset), Q: s.transform.Q}
		drawSolidCircleAt(draw, transform, s.circle.Center, s.circle.Radius, color1)

		cast(transform, func(input *b2.RayCastInput) b2.CastOutput {
			return b2.RayCastCircle(input, &s.circle)
		})

		offset = offset.Add(increment)
	}

	// capsule
	{
		transform := b2.Transform{P: s.transform.P.Add(offset), Q: s.transform.Q}
		v1 := b2.TransformPoint(transform, s.capsule.Center1)
		v2 := b2.TransformPoint(transform, s.capsule.Center2)
		draw.DrawSolidCapsule(v1, v2, s.capsule.Radius, color1)

		cast(transform, func(input *b2.RayCastInput) b2.CastOutput {
			return b2.RayCastCapsule(input, &s.capsule)
		})

		offset = offset.Add(increment)
	}

	// box
	{
		transform := b2.Transform{P: s.transform.P.Add(offset), Q: s.transform.Q}
		draw.DrawSolidPolygon(transform, s.box.Vertices[:s.box.Count], b2.QZero(), color1)

		cast(transform, func(input *b2.RayCastInput) b2.CastOutput {
			return b2.RayCastPolygon(input, &s.box)
		})

		offset = offset.Add(increment)
	}

	// triangle
	{
		transform := b2.Transform{P: s.transform.P.Add(offset), Q: s.transform.Q}
		draw.DrawSolidPolygon(transform, s.triangle.Vertices[:s.triangle.Count], b2.QZero(), color1)

		cast(transform, func(input *b2.RayCastInput) b2.CastOutput {
			return b2.RayCastPolygon(input, &s.triangle)
		})

		offset = offset.Add(increment)
	}

	// segment
	{
		transform := b2.Transform{P: s.transform.P.Add(offset), Q: s.transform.Q}

		p1 := b2.TransformPoint(transform, s.segment.Point1)
		p2 := b2.TransformPoint(transform, s.segment.Point2)
		draw.DrawSegment(p1, p2, color1)

		cast(transform, func(input *b2.RayCastInput) b2.CastOutput {
			return b2.RayCastSegment(input, &s.segment, false)
		})

		offset = offset.Add(increment)
	}

	s.drawRay(&output)
}

// castShapeUserData shows how to filter a specific shape using user data.
type castShapeUserData struct {
	index  int
	ignore bool
}

// castContext collects the ray cast hits. Do what you want with this.
type castContext struct {
	points    [3]b2.Vec2
	normals   [3]b2.Vec2
	fractions [3]b2.Q
	count     int
}

// castIgnored skips a specific shape. It also skips the initial overlap.
func castIgnored(shapeId b2.ShapeId, fraction b2.Q) bool {
	userData, _ := shapeId.GetUserData().(*castShapeUserData)
	return (userData != nil && userData.ignore) || fraction.Eq(b2.QZero())
}

// rayCastClosest finds the closest hit. This is the most common callback
// used in games.
func (c *castContext) rayCastClosest(shapeId b2.ShapeId, point, normal b2.Vec2, fraction b2.Q) b2.Q {
	if castIgnored(shapeId, fraction) {
		// By returning -1, we instruct the calling code to ignore this shape and
		// continue the ray-cast to the next shape.
		return b2.QOne().Neg()
	}

	c.points[0] = point
	c.normals[0] = normal
	c.fractions[0] = fraction
	c.count = 1

	// By returning the current fraction, we instruct the calling code to clip the ray and
	// continue the ray-cast to the next shape. WARNING: do not assume that shapes
	// are reported in order. However, by clipping, we can always get the closest shape.
	return fraction
}

// rayCastAny finds any hit. For this type of query we are usually just
// checking for obstruction, so the hit data is not relevant.
// NOTE: shape hits are not ordered, so this may not return the closest hit
func (c *castContext) rayCastAny(shapeId b2.ShapeId, point, normal b2.Vec2, fraction b2.Q) b2.Q {
	if castIgnored(shapeId, fraction) {
		return b2.QOne().Neg()
	}

	c.points[0] = point
	c.normals[0] = normal
	c.fractions[0] = fraction
	c.count = 1

	// At this point we have a hit, so we know the ray is obstructed.
	// By returning 0, we instruct the calling code to terminate the ray-cast.
	return b2.QZero()
}

// rayCastMultiple collects multiple hits along the ray. The shapes are not
// necessary reported in order, so we might not capture the closest shape.
// NOTE: shape hits are not ordered, so this may return hits in any order. This means that
// if you limit the number of results, you may discard the closest hit. You can see this
// behavior in the sample.
func (c *castContext) rayCastMultiple(shapeId b2.ShapeId, point, normal b2.Vec2, fraction b2.Q) b2.Q {
	if castIgnored(shapeId, fraction) {
		return b2.QOne().Neg()
	}

	count := c.count

	c.points[count] = point
	c.normals[count] = normal
	c.fractions[count] = fraction
	c.count = count + 1

	if c.count == 3 {
		// At this point the buffer is full.
		// By returning 0, we instruct the calling code to terminate the ray-cast.
		return b2.QZero()
	}

	// By returning 1, we instruct the caller to continue without clipping the ray.
	return b2.QOne()
}

// rayCastSorted collects multiple hits along the ray and sorts them.
func (c *castContext) rayCastSorted(shapeId b2.ShapeId, point, normal b2.Vec2, fraction b2.Q) b2.Q {
	if castIgnored(shapeId, fraction) {
		return b2.QOne().Neg()
	}

	count := c.count

	index := 3
	for fraction.Less(c.fractions[index-1]) {
		index -= 1

		if index == 0 {
			break
		}
	}

	if index == 3 {
		// not closer, continue but tell the caller not to consider fractions further than the largest fraction acquired
		// this only happens once the buffer is full
		return c.fractions[2]
	}

	for j := 2; j > index; j-- {
		c.points[j] = c.points[j-1]
		c.normals[j] = c.normals[j-1]
		c.fractions[j] = c.fractions[j-1]
	}

	c.points[index] = point
	c.normals[index] = normal
	c.fractions[index] = fraction
	if count < 3 {
		c.count = count + 1
	} else {
		c.count = 3
	}

	if c.count == 3 {
		return c.fractions[2]
	}

	// By returning 1, we instruct the caller to continue without clipping the ray.
	return b2.QOne()
}

// The query modes and cast shapes of the Cast World sample.
const (
	castModeAny = iota
	castModeClosest
	castModeMultiple
	castModeSorted
)

const (
	castTypeRay = iota
	castTypeCircle
	castTypeCapsule
	castTypePolygon
)

const castWorldMaxCount = 64

// castWorldPolygons returns the four polygons that Cast World and Overlap
// World drop. Only Cast World rounds the second one.
func castWorldPolygons(roundSecond bool) [4]b2.Polygon {
	var polygons [4]b2.Polygon

	{
		hull := b2.ComputeHull([]b2.Vec2{b2.V2(-0.5, 0.0), b2.V2(0.5, 0.0), b2.V2(0.0, 1.5)})
		polygons[0] = b2.MakePolygon(&hull, b2.QZero())
	}

	{
		hull := b2.ComputeHull([]b2.Vec2{b2.V2(-0.1, 0.0), b2.V2(0.1, 0.0), b2.V2(0.0, 1.5)})
		polygons[1] = b2.MakePolygon(&hull, b2.QZero())
		if roundSecond {
			polygons[1].Radius = b2.QHalf()
		}
	}

	{
		w := b2.QOne()
		sqrt2 := b2.F(2).Sqrt()
		b := w.Div(b2.F(2).Add(sqrt2))
		s := sqrt2.Mul(b)
		half := b2.QHalf()

		vertices := []b2.Vec2{
			{X: half.Mul(s), Y: b2.QZero()},
			{X: half.Mul(w), Y: b},
			{X: half.Mul(w), Y: b.Add(s)},
			{X: half.Mul(s), Y: w},
			{X: half.Mul(s).Neg(), Y: w},
			{X: half.Mul(w).Neg(), Y: b.Add(s)},
			{X: half.Mul(w).Neg(), Y: b},
			{X: half.Mul(s).Neg(), Y: b2.QZero()},
		}

		hull := b2.ComputeHull(vertices)
		polygons[2] = b2.MakePolygon(&hull, b2.QZero())
	}

	polygons[3] = b2.MakeBox(b2.QHalf(), b2.QHalf())
	return polygons
}

// CastWorld shows how to use the ray and shape cast functions on a world.
// This sample is configured to ignore initial overlap.
type CastWorld struct {
	Base

	bodyIndex int
	bodyIds   [castWorldMaxCount]b2.BodyId
	userData  [castWorldMaxCount]castShapeUserData
	polygons  [4]b2.Polygon
	capsule   b2.Capsule
	circle    b2.Circle
	segment   b2.Segment

	simple bool

	mode        int
	ignoreIndex int

	castType   int
	castRadius float64

	angleAnchor b2.Vec2
	baseAngle   float64
	angle       float64
	rotating    bool

	rayStart b2.Vec2
	rayEnd   b2.Vec2
	dragging bool
}

func NewCastWorld(ctx *SampleContext) Sample {
	s := &CastWorld{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 2, Y: 14}
		ctx.Camera.Zoom = 25 * 0.75
	}

	// Ground body
	{
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		segment := b2.Segment{Point1: b2.V2(-40, 0), Point2: b2.V2(40, 0)}
		b2.CreateSegmentShape(groundId, &shapeDef, &segment)
	}

	s.polygons = castWorldPolygons(true)
	s.capsule = b2.Capsule{Center1: b2.V2(-0.5, 0.0), Center2: b2.V2(0.5, 0.0), Radius: b2.F(0.25)}
	s.circle = b2.Circle{Radius: b2.QHalf()}
	s.segment = b2.Segment{Point1: b2.V2(-1, 0), Point2: b2.V2(1, 0)}

	s.mode = castModeClosest
	s.ignoreIndex = 7

	s.castType = castTypeRay
	s.castRadius = 0.5

	s.rayStart = b2.V2(-20, 10)
	s.rayEnd = b2.V2(20, 10)
	return s
}

func (s *CastWorld) create(index int) {
	if !s.bodyIds[s.bodyIndex].IsNull() {
		b2.DestroyBody(s.bodyIds[s.bodyIndex])
		s.bodyIds[s.bodyIndex] = b2.BodyId{}
	}

	x := shared.RandomFloatRange(b2.F(-20), b2.F(20))
	y := shared.RandomFloatRange(b2.QZero(), b2.F(20))

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Position = b2.Vec2{X: x, Y: y}
	bodyDef.Rotation = b2.MakeRot(shared.RandomFloatRange(b2.QHalf().Neg(), b2.QHalf()))

	mod := s.bodyIndex % 3
	if mod == 0 {
		bodyDef.Type = b2.StaticBody
	} else if mod == 1 {
		bodyDef.Type = b2.KinematicBody
	} else if mod == 2 {
		bodyDef.Type = b2.DynamicBody
		bodyDef.GravityScale = b2.QZero()
	}

	s.bodyIds[s.bodyIndex] = b2.CreateBody(s.WorldId, &bodyDef)

	shapeDef := b2.DefaultShapeDef()
	shapeDef.UserData = &s.userData[s.bodyIndex]
	s.userData[s.bodyIndex].ignore = false
	if s.bodyIndex == s.ignoreIndex {
		s.userData[s.bodyIndex].ignore = true
	}

	if index < 4 {
		b2.CreatePolygonShape(s.bodyIds[s.bodyIndex], &shapeDef, &s.polygons[index])
	} else if index == 4 {
		b2.CreateCircleShape(s.bodyIds[s.bodyIndex], &shapeDef, &s.circle)
	} else if index == 5 {
		b2.CreateCapsuleShape(s.bodyIds[s.bodyIndex], &shapeDef, &s.capsule)
	} else {
		b2.CreateSegmentShape(s.bodyIds[s.bodyIndex], &shapeDef, &s.segment)
	}

	s.bodyIndex = (s.bodyIndex + 1) % castWorldMaxCount
}

func (s *CastWorld) createN(index, count int) {
	for range count {
		s.create(index)
	}
}

func (s *CastWorld) destroyBody() {
	for i := range s.bodyIds {
		if !s.bodyIds[i].IsNull() {
			b2.DestroyBody(s.bodyIds[i])
			s.bodyIds[i] = b2.BodyId{}
			return
		}
	}
}

func (s *CastWorld) MouseDown(p b2.Vec2, button MouseButton, mod Modifier) {
	if button == MouseButtonLeft {
		if mod == 0 && !s.rotating {
			s.rayStart = p
			s.rayEnd = p
			s.dragging = true
		} else if mod == ModShift && !s.dragging {
			s.rotating = true
			s.angleAnchor = p
			s.baseAngle = s.angle
		}
	}
}

func (s *CastWorld) MouseUp(p b2.Vec2, button MouseButton) {
	if button == MouseButtonLeft {
		s.dragging = false
		s.rotating = false
	}
}

func (s *CastWorld) MouseMove(p b2.Vec2) {
	if s.dragging {
		s.rayEnd = p
	} else if s.rotating {
		dx := ToFloat64(p.X.Sub(s.angleAnchor.X))
		s.angle = s.baseAngle + 1.0*dx
	}
}

// dropButtons draws the create buttons that Cast World and Overlap World
// share. ImGui puts each "10x" beside its button; this GUI stacks them.
func dropButtons(gui Gui, create func(index int), createN func(index, count int)) {
	names := [...]string{"Polygon 1", "Polygon 2", "Polygon 3", "Box", "Circle", "Capsule", "Segment"}
	for i, name := range names {
		if gui.Button(name) {
			create(i)
		}
		if gui.Button("10x " + name) {
			createN(i, 10)
		}
	}
}

func (s *CastWorld) UpdateGui() {
	gui := s.Context.Gui
	height := 320
	gui.Begin("Ray-cast World", 10, s.Context.Camera.Height-height-50, 200, height)

	gui.Checkbox("Simple", &s.simple)

	if !s.simple {
		gui.Combo("Type", &s.castType, []string{"Ray", "Circle", "Capsule", "Polygon"})

		if s.castType != castTypeRay {
			gui.SliderFloat("Radius", &s.castRadius, 0, 2)
		}

		gui.Combo("Mode", &s.mode, []string{"Any", "Closest", "Multiple", "Sorted"})
	}

	dropButtons(gui, s.create, s.createN)

	if gui.Button("Destroy Shape") {
		s.destroyBody()
	}

	gui.End()
}

func (s *CastWorld) Step() {
	s.Base.Step()

	s.DrawTextLine("Click left mouse button and drag to modify ray cast")
	s.DrawTextLine("Shape 7 is intentionally ignored by the ray")

	draw := &s.Context.Draw
	five := b2.F(5)

	color1 := b2.ColorGreen
	color2 := b2.ColorLightGray
	color3 := b2.ColorMagenta

	rayTranslation := s.rayEnd.Sub(s.rayStart)

	if s.simple {
		s.DrawTextLine("Simple closest point ray cast")

		// This version doesn't have a callback, but it doesn't skip the ignored shape
		result := s.WorldId.CastRayClosest(s.rayStart, rayTranslation, b2.DefaultQueryFilter())

		if result.Hit && result.Fraction.Greater(b2.QZero()) {
			c := b2.MulAdd(s.rayStart, result.Fraction, rayTranslation)
			draw.DrawPoint(result.Point, five, color1)
			draw.DrawSegment(s.rayStart, c, color2)
			head := b2.MulAdd(result.Point, b2.QHalf(), result.Normal)
			draw.DrawSegment(result.Point, head, color3)
		} else {
			draw.DrawSegment(s.rayStart, s.rayEnd, color2)
		}
	} else {
		switch s.mode {
		case castModeAny:
			s.DrawTextLine("Cast mode: any - check for obstruction - unsorted")
		case castModeClosest:
			s.DrawTextLine("Cast mode: closest - find closest shape along the cast")
		case castModeMultiple:
			s.DrawTextLine("Cast mode: multiple - gather up to 3 shapes - unsorted")
		case castModeSorted:
			s.DrawTextLine("Cast mode: sorted - gather up to 3 shapes sorted by closeness")
		}

		context := &castContext{}

		// Must initialize fractions for sorting
		context.fractions[0] = b2.Huge
		context.fractions[1] = b2.Huge
		context.fractions[2] = b2.Huge

		functions := [...]b2.CastResultFcn{
			context.rayCastAny,
			context.rayCastClosest,
			context.rayCastMultiple,
			context.rayCastSorted,
		}
		modeFcn := functions[s.mode]

		castRadius := FromFloat64(s.castRadius)
		transform := b2.Transform{P: s.rayStart, Q: rotFromRadians(s.angle)}
		circle := b2.Circle{Center: s.rayStart, Radius: castRadius}
		capsule := b2.Capsule{
			Center1: b2.TransformPoint(transform, b2.V2(-0.25, 0.0)),
			Center2: b2.TransformPoint(transform, b2.V2(0.25, 0.0)),
			Radius:  castRadius,
		}
		box := b2.MakeOffsetRoundedBox(b2.F(0.25), b2.QHalf(), transform.P, transform.Q, castRadius)
		var proxy b2.ShapeProxy

		if s.castType == castTypeRay {
			s.WorldId.CastRay(s.rayStart, rayTranslation, b2.DefaultQueryFilter(), modeFcn)
		} else {
			if s.castType == castTypeCircle {
				proxy = b2.MakeProxy([]b2.Vec2{circle.Center}, circle.Radius)
			} else if s.castType == castTypeCapsule {
				proxy = b2.MakeProxy([]b2.Vec2{capsule.Center1, capsule.Center2}, capsule.Radius)
			} else {
				proxy = b2.MakeProxy(box.Vertices[:box.Count], box.Radius)
			}

			s.WorldId.CastShape(&proxy, rayTranslation, b2.DefaultQueryFilter(), modeFcn)
		}

		if context.count > 0 {
			colors := [3]b2.HexColor{b2.ColorRed, b2.ColorGreen, b2.ColorBlue}
			for i := 0; i < context.count; i++ {
				c := b2.MulAdd(s.rayStart, context.fractions[i], rayTranslation)
				p := context.points[i]
				n := context.normals[i]
				draw.DrawPoint(p, five, colors[i])
				draw.DrawSegment(s.rayStart, c, color2)
				head := b2.MulAdd(p, b2.QOne(), n)
				draw.DrawSegment(p, head, color3)

				t := rayTranslation.Mul(context.fractions[i])
				shiftedTransform := b2.Transform{P: t, Q: b2.RotIdentity()}

				if s.castType == castTypeCircle {
					drawSolidCircleAt(draw, shiftedTransform, circle.Center, castRadius, b2.ColorYellow)
				} else if s.castType == castTypeCapsule {
					p1 := capsule.Center1.Add(t)
					p2 := capsule.Center2.Add(t)
					draw.DrawSolidCapsule(p1, p2, castRadius, b2.ColorYellow)
				} else if s.castType == castTypePolygon {
					draw.DrawSolidPolygon(shiftedTransform, box.Vertices[:box.Count], box.Radius, b2.ColorYellow)
				}
			}
		} else {
			shiftedTransform := b2.Transform{P: transform.P.Add(rayTranslation), Q: transform.Q}
			draw.DrawSegment(s.rayStart, s.rayEnd, color2)

			if s.castType == castTypeCircle {
				drawSolidCircleAt(draw, shiftedTransform, b2.Vec2{}, castRadius, b2.ColorGray)
			} else if s.castType == castTypeCapsule {
				p1 := b2.TransformPoint(transform, capsule.Center1).Add(rayTranslation)
				p2 := b2.TransformPoint(transform, capsule.Center2).Add(rayTranslation)
				draw.DrawSolidCapsule(p1, p2, castRadius, b2.ColorYellow)
			} else if s.castType == castTypePolygon {
				draw.DrawSolidPolygon(shiftedTransform, box.Vertices[:box.Count], box.Radius, b2.ColorYellow)
			}
		}
	}

	draw.DrawPoint(s.rayStart, five, b2.ColorGreen)

	if !s.bodyIds[s.ignoreIndex].IsNull() {
		p := s.bodyIds[s.ignoreIndex].GetPosition()
		p.X = p.X.Sub(b2.F(0.2))
		draw.DrawString(p, "ign", b2.ColorWhite)
	}
}

// The query shapes of the Overlap World sample.
const (
	overlapCircle = iota
	overlapCapsule
	overlapBox
)

const overlapWorldMaxDoomed = 16

// OverlapWorld destroys the bodies that overlap the query shape.
type OverlapWorld struct {
	Base

	bodyIndex   int
	bodyIds     [castWorldMaxCount]b2.BodyId
	userData    [castWorldMaxCount]castShapeUserData
	polygons    [4]b2.Polygon
	capsule     b2.Capsule
	circle      b2.Circle
	segment     b2.Segment
	ignoreIndex int

	doomIds   [overlapWorldMaxDoomed]b2.ShapeId
	doomCount int

	shapeType int

	startPosition b2.Vec2

	position  b2.Vec2
	angle     float64
	baseAngle float64

	dragging bool
	rotating bool
}

func NewOverlapWorld(ctx *SampleContext) Sample {
	s := &OverlapWorld{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 10}
		ctx.Camera.Zoom = 25 * 0.7
	}

	s.polygons = castWorldPolygons(false)
	s.capsule = b2.Capsule{Center1: b2.V2(-0.5, 0.0), Center2: b2.V2(0.5, 0.0), Radius: b2.F(0.25)}
	s.circle = b2.Circle{Radius: b2.QHalf()}
	s.segment = b2.Segment{Point1: b2.V2(-1, 0), Point2: b2.V2(1, 0)}

	s.ignoreIndex = 7

	s.shapeType = overlapCircle

	s.position = b2.V2(0, 10)

	s.createN(0, 10)
	return s
}

func (s *OverlapWorld) create(index int) {
	if !s.bodyIds[s.bodyIndex].IsNull() {
		b2.DestroyBody(s.bodyIds[s.bodyIndex])
		s.bodyIds[s.bodyIndex] = b2.BodyId{}
	}

	x := shared.RandomFloatRange(b2.F(-20), b2.F(20))
	y := shared.RandomFloatRange(b2.QZero(), b2.F(20))

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Position = b2.Vec2{X: x, Y: y}
	bodyDef.Rotation = b2.MakeRot(shared.RandomFloatRange(b2.QHalf().Neg(), b2.QHalf()))

	s.bodyIds[s.bodyIndex] = b2.CreateBody(s.WorldId, &bodyDef)

	shapeDef := b2.DefaultShapeDef()
	shapeDef.UserData = &s.userData[s.bodyIndex]
	s.userData[s.bodyIndex].index = s.bodyIndex
	s.userData[s.bodyIndex].ignore = false
	if s.bodyIndex == s.ignoreIndex {
		s.userData[s.bodyIndex].ignore = true
	}

	if index < 4 {
		b2.CreatePolygonShape(s.bodyIds[s.bodyIndex], &shapeDef, &s.polygons[index])
	} else if index == 4 {
		b2.CreateCircleShape(s.bodyIds[s.bodyIndex], &shapeDef, &s.circle)
	} else if index == 5 {
		b2.CreateCapsuleShape(s.bodyIds[s.bodyIndex], &shapeDef, &s.capsule)
	} else {
		b2.CreateSegmentShape(s.bodyIds[s.bodyIndex], &shapeDef, &s.segment)
	}

	s.bodyIndex = (s.bodyIndex + 1) % castWorldMaxCount
}

func (s *OverlapWorld) createN(index, count int) {
	for range count {
		s.create(index)
	}
}

func (s *OverlapWorld) destroyBody() {
	for i := range s.bodyIds {
		if !s.bodyIds[i].IsNull() {
			b2.DestroyBody(s.bodyIds[i])
			s.bodyIds[i] = b2.BodyId{}
			return
		}
	}
}

func (s *OverlapWorld) MouseDown(p b2.Vec2, button MouseButton, mod Modifier) {
	if button == MouseButtonLeft {
		if mod == 0 && !s.rotating {
			s.dragging = true
			s.position = p
		} else if mod == ModShift && !s.dragging {
			s.rotating = true
			s.startPosition = p
			s.baseAngle = s.angle
		}
	}
}

func (s *OverlapWorld) MouseUp(p b2.Vec2, button MouseButton) {
	if button == MouseButtonLeft {
		s.dragging = false
		s.rotating = false
	}
}

func (s *OverlapWorld) MouseMove(p b2.Vec2) {
	if s.dragging {
		s.position = p
	} else if s.rotating {
		dx := ToFloat64(p.X.Sub(s.startPosition.X))
		s.angle = s.baseAngle + 1.0*dx
	}
}

func (s *OverlapWorld) UpdateGui() {
	gui := s.Context.Gui
	height := 330
	gui.Begin("Overlap World", 10, s.Context.Camera.Height-height-50, 140, height)

	dropButtons(gui, s.create, s.createN)

	if gui.Button("Destroy Shape") {
		s.destroyBody()
	}

	gui.Text("Overlap Shape")
	if gui.RadioButton("Overlap Circle", s.shapeType == overlapCircle) {
		s.shapeType = overlapCircle
	}
	if gui.RadioButton("Overlap Capsule", s.shapeType == overlapCapsule) {
		s.shapeType = overlapCapsule
	}
	if gui.RadioButton("Overlap Box", s.shapeType == overlapBox) {
		s.shapeType = overlapBox
	}

	gui.End()
}

func (s *OverlapWorld) Step() {
	s.Base.Step()

	s.DrawTextLine("left mouse button: drag query shape")
	s.DrawTextLine("left mouse button + shift: rotate query shape")

	draw := &s.Context.Draw

	s.doomCount = 0

	transform := b2.Transform{P: s.position, Q: rotFromRadians(s.angle)}
	var proxy b2.ShapeProxy

	if s.shapeType == overlapCircle {
		circle := b2.Circle{Center: transform.P, Radius: b2.QOne()}
		proxy = b2.MakeProxy([]b2.Vec2{circle.Center}, circle.Radius)
		drawSolidCircleAt(draw, b2.TransformIdentity(), circle.Center, circle.Radius, b2.ColorWhite)
	} else if s.shapeType == overlapCapsule {
		capsule := b2.Capsule{
			Center1: b2.TransformPoint(transform, b2.V2(-1, 0)),
			Center2: b2.TransformPoint(transform, b2.V2(1, 0)),
			Radius:  b2.QHalf(),
		}
		proxy = b2.MakeProxy([]b2.Vec2{capsule.Center1, capsule.Center2}, capsule.Radius)
		draw.DrawSolidCapsule(capsule.Center1, capsule.Center2, capsule.Radius, b2.ColorWhite)
	} else if s.shapeType == overlapBox {
		box := b2.MakeOffsetBox(b2.F(2), b2.QHalf(), transform.P, transform.Q)
		proxy = b2.MakeProxy(box.Vertices[:box.Count], box.Radius)
		draw.DrawPolygon(box.Vertices[:box.Count], b2.ColorWhite)
	}

	s.WorldId.OverlapShape(&proxy, b2.DefaultQueryFilter(), func(shapeId b2.ShapeId) bool {
		userData, _ := shapeId.GetUserData().(*castShapeUserData)
		if userData != nil && userData.ignore {
			// continue the query
			return true
		}

		if s.doomCount < overlapWorldMaxDoomed {
			index := s.doomCount
			s.doomIds[index] = shapeId
			s.doomCount += 1
		}

		// continue the query
		return true
	})

	if !s.bodyIds[s.ignoreIndex].IsNull() {
		p := s.bodyIds[s.ignoreIndex].GetPosition()
		p.X = p.X.Sub(b2.F(0.2))
		draw.DrawString(p, "skip", b2.ColorWhite)
	}

	for i := 0; i < s.doomCount; i++ {
		shapeId := s.doomIds[i]
		userData, _ := shapeId.GetUserData().(*castShapeUserData)
		if userData == nil {
			continue
		}

		index := userData.index
		b2.DestroyBody(s.bodyIds[index])
		s.bodyIds[index] = b2.BodyId{}
	}
}

// Manifold tests manifolds and contact points.
type Manifold struct {
	Base

	smgroxCache1 b2.SimplexCache
	smgroxCache2 b2.SimplexCache
	smgcapCache1 b2.SimplexCache
	smgcapCache2 b2.SimplexCache

	wedge b2.Hull

	transform b2.Transform
	angle     float64
	round     float64

	basePosition b2.Vec2
	startPoint   b2.Vec2
	baseAngle    float64

	dragging       bool
	rotating       bool
	showCount      bool
	showIds        bool
	showAnchors    bool
	showSeparation bool
	enableCaching  bool
}

func NewManifold(ctx *SampleContext) Sample {
	s := &Manifold{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 1.8, Y: 0}
		ctx.Camera.Zoom = 25 * 0.45
	}

	s.transform = b2.TransformIdentity()
	s.transform.P.X = b2.F(0.17)
	s.transform.P.Y = b2.F(1.12)
	s.round = 0.1

	s.enableCaching = true

	s.wedge = b2.ComputeHull([]b2.Vec2{b2.V2(-0.1, -0.5), b2.V2(0.1, -0.5), b2.V2(0.0, 0.5)})
	return s
}

func (s *Manifold) UpdateGui() {
	gui := s.Context.Gui
	height := 320
	gui.Begin("Manifold", 10, s.Context.Camera.Height-height-50, 340, height)

	sliderQ(gui, "x offset", &s.transform.P.X, -2, 2)
	sliderQ(gui, "y offset", &s.transform.P.Y, -2, 2)

	if gui.SliderFloat("angle", &s.angle, -math.Pi, math.Pi) {
		s.transform.Q = rotFromRadians(s.angle)
	}

	gui.SliderFloat("round", &s.round, 0, 0.4)

	gui.Checkbox("show count", &s.showCount)
	gui.Checkbox("show ids", &s.showIds)
	gui.Checkbox("show separation", &s.showSeparation)
	gui.Checkbox("show anchors", &s.showAnchors)
	gui.Checkbox("enable caching", &s.enableCaching)

	if gui.Button("Reset") {
		s.transform = b2.TransformIdentity()
		s.angle = 0
	}

	gui.Text("mouse button 1: drag")
	gui.Text("mouse button 1 + shift: rotate")

	gui.End()
}

func (s *Manifold) MouseDown(p b2.Vec2, button MouseButton, mod Modifier) {
	if button == MouseButtonLeft {
		if mod == 0 && !s.rotating {
			s.dragging = true
			s.startPoint = p
			s.basePosition = s.transform.P
		} else if mod == ModShift && !s.dragging {
			s.rotating = true
			s.startPoint = p
			s.baseAngle = s.angle
		}
	}
}

func (s *Manifold) MouseUp(p b2.Vec2, button MouseButton) {
	if button == MouseButtonLeft {
		s.dragging = false
		s.rotating = false
	}
}

func (s *Manifold) MouseMove(p b2.Vec2) {
	if s.dragging {
		s.transform.P = s.basePosition.Add(p.Sub(s.startPoint).Mul(b2.QHalf()))
	} else if s.rotating {
		dx := ToFloat64(p.X.Sub(s.startPoint.X))
		s.angle = clampFloat(s.baseAngle+1.0*dx, -math.Pi, math.Pi)
		s.transform.Q = rotFromRadians(s.angle)
	}
}

func (s *Manifold) drawManifold(manifold *b2.Manifold, origin1, origin2 b2.Vec2) {
	draw := &s.Context.Draw

	if s.showCount {
		p := origin1.Add(origin2).Mul(b2.QHalf())
		draw.DrawString(p, fmt.Sprintf("%d", manifold.PointCount), b2.ColorWhite)
	}

	for i := 0; i < manifold.PointCount; i++ {
		mp := &manifold.Points[i]

		p1 := mp.Point
		p2 := b2.MulAdd(p1, b2.QHalf(), manifold.Normal)
		draw.DrawSegment(p1, p2, b2.ColorViolet)

		if s.showAnchors {
			draw.DrawPoint(origin1.Add(mp.AnchorA), b2.F(5), b2.ColorRed)
			draw.DrawPoint(origin2.Add(mp.AnchorB), b2.F(5), b2.ColorGreen)
		} else {
			draw.DrawPoint(p1, b2.F(10), b2.ColorBlue)
		}

		if s.showIds {
			p := b2.Vec2{X: p1.X.Add(b2.F(0.05)), Y: p1.Y.Sub(b2.F(0.02))}
			draw.DrawString(p, fmt.Sprintf("0x%04x", mp.Id), b2.ColorWhite)
		}

		if s.showSeparation {
			p := b2.Vec2{X: p1.X.Add(b2.F(0.05)), Y: p1.Y.Add(b2.F(0.03))}
			draw.DrawString(p, fmt.Sprintf("%.3f", ToFloat64(mp.Separation)), b2.ColorWhite)
		}
	}
}

func (s *Manifold) Step() {
	draw := &s.Context.Draw

	offset := b2.V2(-10, -5)
	increment := b2.V2(4, 0)

	color1 := b2.ColorAquamarine
	color2 := b2.ColorPaleGoldenRod

	if !s.enableCaching {
		s.smgroxCache1 = b2.SimplexCache{}
		s.smgroxCache2 = b2.SimplexCache{}
		s.smgcapCache1 = b2.SimplexCache{}
		s.smgcapCache2 = b2.SimplexCache{}
	}

	round := FromFloat64(s.round)
	h := b2.QHalf().Sub(round)

	// transforms returns the fixed frame and the dragged frame at the
	// current offset.
	transforms := func() (b2.Transform, b2.Transform) {
		return b2.Transform{P: offset, Q: b2.RotIdentity()},
			b2.Transform{P: s.transform.P.Add(offset), Q: s.transform.Q}
	}

	// circle-circle
	{
		circle1 := b2.Circle{Radius: b2.QHalf()}
		circle2 := b2.Circle{Radius: b2.QOne()}

		transform1, transform2 := transforms()

		m := b2.CollideCircles(&circle1, transform1, &circle2, transform2)

		drawSolidCircleAt(draw, transform1, circle1.Center, circle1.Radius, color1)
		drawSolidCircleAt(draw, transform2, circle2.Center, circle2.Radius, color2)

		s.drawManifold(&m, transform1.P, transform2.P)

		offset = offset.Add(increment)
	}

	// capsule-circle
	{
		capsule := b2.Capsule{Center1: b2.V2(-0.5, 0.0), Center2: b2.V2(0.5, 0.0), Radius: b2.F(0.25)}
		circle := b2.Circle{Radius: b2.QHalf()}

		transform1, transform2 := transforms()

		m := b2.CollideCapsuleAndCircle(&capsule, transform1, &circle, transform2)

		v1 := b2.TransformPoint(transform1, capsule.Center1)
		v2 := b2.TransformPoint(transform1, capsule.Center2)
		draw.DrawSolidCapsule(v1, v2, capsule.Radius, color1)

		drawSolidCircleAt(draw, transform2, circle.Center, circle.Radius, color2)

		s.drawManifold(&m, transform1.P, transform2.P)

		offset = offset.Add(increment)
	}

	// segment-circle
	{
		segment := b2.Segment{Point1: b2.V2(-1, 0), Point2: b2.V2(1, 0)}
		circle := b2.Circle{Radius: b2.QHalf()}

		transform1, transform2 := transforms()

		m := b2.CollideSegmentAndCircle(&segment, transform1, &circle, transform2)

		p1 := b2.TransformPoint(transform1, segment.Point1)
		p2 := b2.TransformPoint(transform1, segment.Point2)
		draw.DrawSegment(p1, p2, color1)

		drawSolidCircleAt(draw, transform2, circle.Center, circle.Radius, color2)

		s.drawManifold(&m, transform1.P, transform2.P)

		offset = offset.Add(increment)
	}

	// box-circle
	{
		circle := b2.Circle{Radius: b2.QHalf()}
		box := b2.MakeSquare(b2.QHalf())
		box.Radius = round

		transform1, transform2 := transforms()

		m := b2.CollidePolygonAndCircle(&box, transform1, &circle, transform2)

		draw.DrawSolidPolygon(transform1, box.Vertices[:box.Count], round, color1)
		drawSolidCircleAt(draw, transform2, circle.Center, circle.Radius, color2)

		s.drawManifold(&m, transform1.P, transform2.P)

		offset = offset.Add(increment)
	}

	// capsule-capsule
	{
		capsule1 := b2.Capsule{Center1: b2.V2(-0.5, 0.0), Center2: b2.V2(0.5, 0.0), Radius: b2.F(0.25)}
		capsule2 := b2.Capsule{Center1: b2.V2(0.25, 0.0), Center2: b2.V2(1, 0), Radius: b2.F(0.1)}

		transform1, transform2 := transforms()

		m := b2.CollideCapsules(&capsule1, transform1, &capsule2, transform2)

		v1 := b2.TransformPoint(transform1, capsule1.Center1)
		v2 := b2.TransformPoint(transform1, capsule1.Center2)
		draw.DrawSolidCapsule(v1, v2, capsule1.Radius, color1)

		v1 = b2.TransformPoint(transform2, capsule2.Center1)
		v2 = b2.TransformPoint(transform2, capsule2.Center2)
		draw.DrawSolidCapsule(v1, v2, capsule2.Radius, color2)

		s.drawManifold(&m, transform1.P, transform2.P)

		offset = offset.Add(increment)
	}

	// box-capsule
	{
		capsule := b2.Capsule{Center1: b2.V2(-0.4, 0.0), Center2: b2.V2(-0.1, 0.0), Radius: b2.F(0.1)}
		box := b2.MakeOffsetBox(b2.F(0.25), b2.QOne(), b2.V2(1, -1), b2.MakeRot(b2.QFromRatio(1, 8)))

		transform1, transform2 := transforms()

		m := b2.CollidePolygonAndCapsule(&box, transform1, &capsule, transform2)

		draw.DrawSolidPolygon(transform1, box.Vertices[:box.Count], box.Radius, color1)

		v1 := b2.TransformPoint(transform2, capsule.Center1)
		v2 := b2.TransformPoint(transform2, capsule.Center2)
		draw.DrawSolidCapsule(v1, v2, capsule.Radius, color2)

		s.drawManifold(&m, transform1.P, transform2.P)

		offset = offset.Add(increment)
	}

	// segment-capsule
	{
		segment := b2.Segment{Point1: b2.V2(-1, 0), Point2: b2.V2(1, 0)}
		capsule := b2.Capsule{Center1: b2.V2(-0.5, 0.0), Center2: b2.V2(0.5, 0.0), Radius: b2.F(0.25)}

		transform1, transform2 := transforms()

		m := b2.CollideSegmentAndCapsule(&segment, transform1, &capsule, transform2)

		p1 := b2.TransformPoint(transform1, segment.Point1)
		p2 := b2.TransformPoint(transform1, segment.Point2)
		draw.DrawSegment(p1, p2, color1)

		p1 = b2.TransformPoint(transform2, capsule.Center1)
		p2 = b2.TransformPoint(transform2, capsule.Center2)
		draw.DrawSolidCapsule(p1, p2, capsule.Radius, color2)

		s.drawManifold(&m, transform1.P, transform2.P)

		offset = offset.Add(increment)
	}

	offset = b2.V2(-10, 0)

	// square-square
	{
		box1 := b2.MakeSquare(b2.QHalf())
		box := b2.MakeSquare(b2.QHalf())

		transform1, transform2 := transforms()

		m := b2.CollidePolygons(&box1, transform1, &box, transform2)

		draw.DrawSolidPolygon(transform1, box1.Vertices[:box1.Count], box1.Radius, color1)
		draw.DrawSolidPolygon(transform2, box.Vertices[:box.Count], box.Radius, color2)

		s.drawManifold(&m, transform1.P, transform2.P)

		offset = offset.Add(increment)
	}

	// box-box
	{
		box1 := b2.MakeBox(b2.F(2), b2.F(0.1))
		box := b2.MakeSquare(b2.F(0.25))

		transform1, transform2 := transforms()

		m := b2.CollidePolygons(&box1, transform1, &box, transform2)

		draw.DrawSolidPolygon(transform1, box1.Vertices[:box1.Count], box1.Radius, color1)
		draw.DrawSolidPolygon(transform2, box.Vertices[:box.Count], box.Radius, color2)

		s.drawManifold(&m, transform1.P, transform2.P)

		offset = offset.Add(increment)
	}

	// box-rox
	{
		box := b2.MakeSquare(b2.QHalf())
		rox := b2.MakeRoundedBox(h, h, round)

		transform1, transform2 := transforms()

		m := b2.CollidePolygons(&box, transform1, &rox, transform2)

		draw.DrawSolidPolygon(transform1, box.Vertices[:box.Count], box.Radius, color1)
		draw.DrawSolidPolygon(transform2, rox.Vertices[:rox.Count], rox.Radius, color2)

		s.drawManifold(&m, transform1.P, transform2.P)

		offset = offset.Add(increment)
	}

	// rox-rox
	{
		rox := b2.MakeRoundedBox(h, h, round)

		transform1, transform2 := transforms()

		m := b2.CollidePolygons(&rox, transform1, &rox, transform2)

		draw.DrawSolidPolygon(transform1, rox.Vertices[:rox.Count], rox.Radius, color1)
		draw.DrawSolidPolygon(transform2, rox.Vertices[:rox.Count], rox.Radius, color2)

		s.drawManifold(&m, transform1.P, transform2.P)

		offset = offset.Add(increment)
	}

	// segment-rox
	{
		segment := b2.Segment{Point1: b2.V2(-1, 0), Point2: b2.V2(1, 0)}
		rox := b2.MakeRoundedBox(h, h, round)

		transform1, transform2 := transforms()

		m := b2.CollideSegmentAndPolygon(&segment, transform1, &rox, transform2)

		p1 := b2.TransformPoint(transform1, segment.Point1)
		p2 := b2.TransformPoint(transform1, segment.Point2)
		draw.DrawSegment(p1, p2, color1)
		draw.DrawSolidPolygon(transform2, rox.Vertices[:rox.Count], rox.Radius, color2)

		s.drawManifold(&m, transform1.P, transform2.P)

		offset = offset.Add(increment)
	}

	// wox-wox
	{
		wox := b2.MakePolygon(&s.wedge, round)

		transform1, transform2 := transforms()

		m := b2.CollidePolygons(&wox, transform1, &wox, transform2)

		draw.DrawSolidPolygon(transform1, wox.Vertices[:wox.Count], wox.Radius, color1)
		draw.DrawSolidPolygon(transform1, wox.Vertices[:wox.Count], b2.QZero(), color1)
		draw.DrawSolidPolygon(transform2, wox.Vertices[:wox.Count], wox.Radius, color2)
		draw.DrawSolidPolygon(transform2, wox.Vertices[:wox.Count], b2.QZero(), color2)

		s.drawManifold(&m, transform1.P, transform2.P)

		offset = offset.Add(increment)
	}

	// wox-wox
	{
		p1s := []b2.Vec2{b2.V2(0.175740838, 0.224936664), b2.V2(-0.301293969, 0.194021404), b2.V2(-0.105151534, -0.432157338)}
		p2s := []b2.Vec2{b2.V2(-0.427884758, -0.225028217), b2.V2(0.0566576123, -0.128772855), b2.V2(0.176625848, 0.338923335)}

		h1 := b2.ComputeHull(p1s)
		h2 := b2.ComputeHull(p2s)
		w1 := b2.MakePolygon(&h1, b2.F(0.158798501))
		w2 := b2.MakePolygon(&h2, b2.F(0.205900759))

		transform1, transform2 := transforms()

		m := b2.CollidePolygons(&w1, transform1, &w2, transform2)

		draw.DrawSolidPolygon(transform1, w1.Vertices[:w1.Count], w1.Radius, color1)
		draw.DrawSolidPolygon(transform1, w1.Vertices[:w1.Count], b2.QZero(), color1)
		draw.DrawSolidPolygon(transform2, w2.Vertices[:w2.Count], w2.Radius, color2)
		draw.DrawSolidPolygon(transform2, w2.Vertices[:w2.Count], b2.QZero(), color2)

		s.drawManifold(&m, transform1.P, transform2.P)

		offset = offset.Add(increment)
	}

	offset = b2.V2(-10, 5)

	// box-triangle
	{
		box := b2.MakeBox(b2.QOne(), b2.QOne())
		hull := b2.ComputeHull([]b2.Vec2{b2.V2(-0.05, 0.0), b2.V2(0.05, 0.0), b2.V2(0.0, 0.1)})
		tri := b2.MakePolygon(&hull, b2.QZero())

		transform1, transform2 := transforms()

		m := b2.CollidePolygons(&box, transform1, &tri, transform2)

		draw.DrawSolidPolygon(transform1, box.Vertices[:box.Count], b2.QZero(), color1)
		draw.DrawSolidPolygon(transform2, tri.Vertices[:tri.Count], b2.QZero(), color2)

		s.drawManifold(&m, transform1.P, transform2.P)

		offset = offset.Add(increment)
	}

	two := b2.F(2)
	four := b2.F(4)
	five := b2.F(5)

	segment1 := b2.ChainSegment{
		Ghost1:  b2.V2(2, 1),
		Segment: b2.Segment{Point1: b2.V2(1, 1), Point2: b2.V2(-1, 0)},
		Ghost2:  b2.V2(-2, 0),
		ChainId: -1,
	}
	segment2 := b2.ChainSegment{
		Ghost1:  b2.V2(3, 1),
		Segment: b2.Segment{Point1: b2.V2(2, 1), Point2: b2.V2(1, 1)},
		Ghost2:  b2.V2(-1, 0),
		ChainId: -1,
	}

	// drawChainPair draws segment1 with its head ghost and segment2 with
	// its tail ghost.
	drawChainPair := func(transform1 b2.Transform) {
		{
			g2 := b2.TransformPoint(transform1, segment1.Ghost2)
			p1 := b2.TransformPoint(transform1, segment1.Segment.Point1)
			p2 := b2.TransformPoint(transform1, segment1.Segment.Point2)
			draw.DrawSegment(p1, p2, color1)
			draw.DrawPoint(p1, four, color1)
			draw.DrawPoint(p2, four, color1)
			draw.DrawSegment(p2, g2, b2.ColorLightGray)
		}

		{
			g1 := b2.TransformPoint(transform1, segment2.Ghost1)
			p1 := b2.TransformPoint(transform1, segment2.Segment.Point1)
			p2 := b2.TransformPoint(transform1, segment2.Segment.Point2)
			draw.DrawSegment(g1, p1, b2.ColorLightGray)
			draw.DrawSegment(p1, p2, color1)
			draw.DrawPoint(p1, four, color1)
			draw.DrawPoint(p2, four, color1)
		}
	}

	// chain-segment vs circle
	{
		segment := segment1
		circle := b2.Circle{Radius: b2.QHalf()}

		transform1, transform2 := transforms()

		m := b2.CollideChainSegmentAndCircle(&segment, transform1, &circle, transform2)

		g1 := b2.TransformPoint(transform1, segment.Ghost1)
		g2 := b2.TransformPoint(transform1, segment.Ghost2)
		p1 := b2.TransformPoint(transform1, segment.Segment.Point1)
		p2 := b2.TransformPoint(transform1, segment.Segment.Point2)
		draw.DrawSegment(g1, p1, b2.ColorLightGray)
		draw.DrawSegment(p1, p2, color1)
		draw.DrawSegment(p2, g2, b2.ColorLightGray)
		drawSolidCircleAt(draw, transform2, circle.Center, circle.Radius, color2)

		s.drawManifold(&m, transform1.P, transform2.P)

		offset.X = offset.X.Add(two.Mul(increment.X))
	}

	// chain-segment vs rounded polygon
	{
		rox := b2.MakeRoundedBox(h, h, round)

		transform1, transform2 := transforms()

		m1 := b2.CollideChainSegmentAndPolygon(&segment1, transform1, &rox, transform2, &s.smgroxCache1)
		m2 := b2.CollideChainSegmentAndPolygon(&segment2, transform1, &rox, transform2, &s.smgroxCache2)

		drawChainPair(transform1)

		draw.DrawSolidPolygon(transform2, rox.Vertices[:rox.Count], rox.Radius, color2)
		draw.DrawPoint(b2.TransformPoint(transform2, rox.Centroid), five, b2.ColorGainsboro)

		s.drawManifold(&m1, transform1.P, transform2.P)
		s.drawManifold(&m2, transform1.P, transform2.P)

		offset.X = offset.X.Add(two.Mul(increment.X))
	}

	// chain-segment vs capsule
	{
		capsule := b2.Capsule{Center1: b2.V2(-0.5, 0.0), Center2: b2.V2(0.5, 0.0), Radius: b2.F(0.25)}

		transform1, transform2 := transforms()

		m1 := b2.CollideChainSegmentAndCapsule(&segment1, transform1, &capsule, transform2, &s.smgcapCache1)
		m2 := b2.CollideChainSegmentAndCapsule(&segment2, transform1, &capsule, transform2, &s.smgcapCache2)

		drawChainPair(transform1)

		p1 := b2.TransformPoint(transform2, capsule.Center1)
		p2 := b2.TransformPoint(transform2, capsule.Center2)
		draw.DrawSolidCapsule(p1, p2, capsule.Radius, color2)

		draw.DrawPoint(b2.Lerp(p1, p2, b2.QHalf()), five, b2.ColorGainsboro)

		s.drawManifold(&m1, transform1.P, transform2.P)
		s.drawManifold(&m2, transform1.P, transform2.P)

		offset.X = offset.X.Add(two.Mul(increment.X))
	}
}

// The shapes of the Smooth Manifold sample.
const (
	smoothCircle = iota
	smoothBox
)

// smoothManifoldPoints is the closed path of the Smooth Manifold sample,
// from https://betravis.github.io/shape-tools/path-to-polygon/.
var smoothManifoldPoints = [...][2]float64{
	{-20.58325, 14.54175},
	{-21.90625, 15.8645},
	{-24.552, 17.1875},
	{-27.198, 11.89575},
	{-29.84375, 15.8645},
	{-29.84375, 21.15625},
	{-25.875, 23.802},
	{-20.58325, 25.125},
	{-25.875, 29.09375},
	{-20.58325, 31.7395},
	{-11.0089998, 23.2290001},
	{-8.67700005, 21.15625},
	{-6.03125, 21.15625},
	{-7.35424995, 29.09375},
	{-3.38549995, 29.09375},
	{1.90625, 30.41675},
	{5.875, 17.1875},
	{11.16675, 25.125},
	{9.84375, 29.09375},
	{13.8125, 31.7395},
	{21.75, 30.41675},
	{28.3644981, 26.448},
	{25.71875, 18.5105},
	{24.3957481, 13.21875},
	{17.78125, 11.89575},
	{15.1355, 7.92700005},
	{5.875, 9.25},
	{1.90625, 11.89575},
	{-3.25, 11.89575},
	{-3.25, 9.9375},
	{-4.70825005, 9.25},
	{-8.67700005, 9.25},
	{-11.323, 11.89575},
	{-13.96875, 11.89575},
	{-15.29175, 14.54175},
	{-19.2605, 14.54175},
}

// SmoothManifold collides a shape against a closed loop of chain segments.
type SmoothManifold struct {
	Base

	shapeType int

	segments []b2.ChainSegment

	transform b2.Transform
	angle     float64
	round     float64

	basePosition b2.Vec2
	startPoint   b2.Vec2
	baseAngle    float64

	dragging       bool
	rotating       bool
	showIds        bool
	showAnchors    bool
	showSeparation bool
}

func NewSmoothManifold(ctx *SampleContext) Sample {
	s := &SmoothManifold{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 2, Y: 20}
		ctx.Camera.Zoom = 21
	}

	s.shapeType = smoothBox
	s.transform = b2.Transform{P: b2.V2(0, 20), Q: b2.RotIdentity()}

	count := len(smoothManifoldPoints)
	points := make([]b2.Vec2, count)
	for i, p := range smoothManifoldPoints {
		points[i] = b2.V2(p[0], p[1])
	}

	s.segments = make([]b2.ChainSegment, count)

	for i := 0; i < count; i++ {
		i0 := count - 1
		if i > 0 {
			i0 = i - 1
		}
		i1 := i
		i2 := 0
		if i1 < count-1 {
			i2 = i1 + 1
		}
		i3 := 0
		if i2 < count-1 {
			i3 = i2 + 1
		}

		g1 := points[i0]
		p1 := points[i1]
		p2 := points[i2]
		g2 := points[i3]

		s.segments[i] = b2.ChainSegment{Ghost1: g1, Segment: b2.Segment{Point1: p1, Point2: p2}, Ghost2: g2, ChainId: -1}
	}
	return s
}

func (s *SmoothManifold) UpdateGui() {
	gui := s.Context.Gui
	height := 290
	gui.Begin("Smooth Manifold", 10, s.Context.Camera.Height-height-50, 180, height)

	gui.Combo("Shape", &s.shapeType, []string{"Circle", "Box"})

	sliderQ(gui, "x Offset", &s.transform.P.X, -2, 2)
	sliderQ(gui, "y Offset", &s.transform.P.Y, -2, 2)

	if gui.SliderFloat("Angle", &s.angle, -math.Pi, math.Pi) {
		s.transform.Q = rotFromRadians(s.angle)
	}

	gui.SliderFloat("Round", &s.round, 0, 0.4)
	gui.Checkbox("Show Ids", &s.showIds)
	gui.Checkbox("Show Separation", &s.showSeparation)
	gui.Checkbox("Show Anchors", &s.showAnchors)

	if gui.Button("Reset") {
		s.transform = b2.TransformIdentity()
		s.angle = 0
	}

	gui.Text("mouse button 1: drag")
	gui.Text("mouse button 1 + shift: rotate")

	gui.End()
}

func (s *SmoothManifold) MouseDown(p b2.Vec2, button MouseButton, mod Modifier) {
	if button == MouseButtonLeft {
		if mod == 0 && !s.rotating {
			s.dragging = true
			s.startPoint = p
			s.basePosition = s.transform.P
		} else if mod == ModShift && !s.dragging {
			s.rotating = true
			s.startPoint = p
			s.baseAngle = s.angle
		}
	}
}

func (s *SmoothManifold) MouseUp(p b2.Vec2, button MouseButton) {
	if button == MouseButtonLeft {
		s.dragging = false
		s.rotating = false
	}
}

func (s *SmoothManifold) MouseMove(p b2.Vec2) {
	if s.dragging {
		s.transform.P = s.basePosition.Add(p.Sub(s.startPoint))
	} else if s.rotating {
		dx := ToFloat64(p.X.Sub(s.startPoint.X))
		s.angle = clampFloat(s.baseAngle+1.0*dx, -math.Pi, math.Pi)
		s.transform.Q = rotFromRadians(s.angle)
	}
}

func (s *SmoothManifold) drawManifold(manifold *b2.Manifold) {
	draw := &s.Context.Draw

	for i := 0; i < manifold.PointCount; i++ {
		mp := &manifold.Points[i]

		p1 := mp.Point
		p2 := b2.MulAdd(p1, b2.QHalf(), manifold.Normal)
		draw.DrawSegment(p1, p2, b2.ColorWhite)

		// The reference draws the same point with or without anchors.
		draw.DrawPoint(p1, b2.F(5), b2.ColorGreen)

		if s.showIds {
			p := b2.Vec2{X: p1.X.Add(b2.F(0.05)), Y: p1.Y.Sub(b2.F(0.02))}
			draw.DrawString(p, fmt.Sprintf("0x%04x", mp.Id), b2.ColorWhite)
		}

		if s.showSeparation {
			p := b2.Vec2{X: p1.X.Add(b2.F(0.05)), Y: p1.Y.Add(b2.F(0.03))}
			draw.DrawString(p, fmt.Sprintf("%.3f", ToFloat64(mp.Separation)), b2.ColorWhite)
		}
	}
}

func (s *SmoothManifold) Step() {
	draw := &s.Context.Draw

	color1 := b2.ColorYellow
	color2 := b2.ColorMagenta

	transform1 := b2.TransformIdentity()
	transform2 := s.transform

	for i := range s.segments {
		segment := &s.segments[i]
		p1 := b2.TransformPoint(transform1, segment.Segment.Point1)
		p2 := b2.TransformPoint(transform1, segment.Segment.Point2)
		draw.DrawSegment(p1, p2, color1)
		draw.DrawPoint(p1, b2.F(4), color1)
	}

	// chain-segment vs circle
	if s.shapeType == smoothCircle {
		circle := b2.Circle{Radius: b2.QHalf()}
		drawSolidCircleAt(draw, transform2, circle.Center, circle.Radius, color2)

		for i := range s.segments {
			m := b2.CollideChainSegmentAndCircle(&s.segments[i], transform1, &circle, transform2)
			s.drawManifold(&m)
		}
	} else if s.shapeType == smoothBox {
		round := FromFloat64(s.round)
		h := b2.QHalf().Sub(round)
		rox := b2.MakeRoundedBox(h, h, round)
		draw.DrawSolidPolygon(transform2, rox.Vertices[:rox.Count], rox.Radius, color2)

		for i := range s.segments {
			cache := b2.SimplexCache{}
			m := b2.CollideChainSegmentAndPolygon(&s.segments[i], transform1, &rox, transform2, &cache)
			s.drawManifold(&m)
		}
	}
}

// ShapeCast sweeps proxy B against proxy A.
type ShapeCast struct {
	Base

	shapes proxyShapes

	typeA, typeB     int
	radiusA, radiusB float64
	proxyA, proxyB   b2.ShapeProxy

	transform   b2.Transform
	angle       float64
	translation b2.Vec2

	basePosition b2.Vec2
	startPoint   b2.Vec2
	baseAngle    float64

	dragging    bool
	sweeping    bool
	rotating    bool
	showIndices bool
	encroach    bool
}

func NewShapeCast(ctx *SampleContext) Sample {
	s := &ShapeCast{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 0.25}
		ctx.Camera.Zoom = 3
	}

	s.shapes.segment = b2.Segment{Point1: b2.V2(0, 0), Point2: b2.V2(0.5, 0.0)}

	{
		hull := b2.ComputeHull([]b2.Vec2{b2.V2(-0.5, 0.0), b2.V2(0.5, 0.0), b2.V2(0, 1)})
		s.shapes.triangle = b2.MakePolygon(&hull, b2.QZero())
	}

	s.shapes.box = b2.MakeOffsetBox(b2.QHalf(), b2.QHalf(), b2.Vec2{}, b2.RotIdentity())

	s.transform = b2.Transform{P: b2.V2(-0.6, 0.0), Q: b2.RotIdentity()}
	s.translation = b2.V2(2, 0)

	s.typeA = proxyBox
	s.typeB = proxyPoint
	s.radiusA = 0
	s.radiusB = 0.2

	s.proxyA = s.shapes.makeProxy(s.typeA, FromFloat64(s.radiusA))
	s.proxyB = s.shapes.makeProxy(s.typeB, FromFloat64(s.radiusB))
	return s
}

func (s *ShapeCast) MouseDown(p b2.Vec2, button MouseButton, mod Modifier) {
	if button == MouseButtonLeft {
		if mod == 0 {
			s.dragging = true
			s.sweeping = false
			s.rotating = false
			s.startPoint = p
			s.basePosition = s.transform.P
		} else if mod == ModShift {
			s.dragging = false
			s.sweeping = false
			s.rotating = true
			s.startPoint = p
			s.baseAngle = s.angle
		} else if mod == ModControl {
			s.dragging = false
			s.sweeping = true
			s.rotating = false
			s.startPoint = p
			s.basePosition = b2.Vec2{}
		}
	}
}

func (s *ShapeCast) MouseUp(p b2.Vec2, button MouseButton) {
	if button == MouseButtonLeft {
		s.dragging = false
		s.sweeping = false
		s.rotating = false
	}
}

func (s *ShapeCast) MouseMove(p b2.Vec2) {
	if s.dragging {
		s.transform.P = s.basePosition.Add(p.Sub(s.startPoint).Mul(b2.QHalf()))
	} else if s.rotating {
		dx := ToFloat64(p.X.Sub(s.startPoint.X))
		s.angle = clampFloat(s.baseAngle+1.0*dx, -math.Pi, math.Pi)
		s.transform.Q = rotFromRadians(s.angle)
	} else if s.sweeping {
		s.translation = p.Sub(s.startPoint)
	}
}

func (s *ShapeCast) UpdateGui() {
	gui := s.Context.Gui
	height := 300
	// The reference titles this window "Shape Distance" as well.
	gui.Begin("Shape Distance", 10, s.Context.Camera.Height-height-50, 240, height)

	if gui.Combo("shape A", &s.typeA, proxyShapeNames) {
		s.proxyA = s.shapes.makeProxy(s.typeA, FromFloat64(s.radiusA))
	}

	if gui.SliderFloat("radius A", &s.radiusA, 0, 0.5) {
		s.proxyA.Radius = FromFloat64(s.radiusA)
	}

	if gui.Combo("shape B", &s.typeB, proxyShapeNames) {
		s.proxyB = s.shapes.makeProxy(s.typeB, FromFloat64(s.radiusB))
	}

	if gui.SliderFloat("radius B", &s.radiusB, 0, 0.5) {
		s.proxyB.Radius = FromFloat64(s.radiusB)
	}

	sliderQ(gui, "x offset", &s.transform.P.X, -2, 2)
	sliderQ(gui, "y offset", &s.transform.P.Y, -2, 2)

	if gui.SliderFloat("angle", &s.angle, -math.Pi, math.Pi) {
		s.transform.Q = rotFromRadians(s.angle)
	}

	gui.Checkbox("show indices", &s.showIndices)
	gui.Checkbox("encroach", &s.encroach)

	gui.End()
}

func (s *ShapeCast) Step() {
	s.Base.Step()

	input := b2.ShapeCastPairInput{
		ProxyA:       s.proxyA,
		ProxyB:       s.proxyB,
		TransformA:   b2.TransformIdentity(),
		TransformB:   s.transform,
		TranslationB: s.translation,
		MaxFraction:  b2.QOne(),
		CanEncroach:  s.encroach,
	}

	output := b2.ShapeCast(&input)

	transform := b2.Transform{
		P: b2.MulAdd(s.transform.P, output.Fraction, input.TranslationB),
		Q: s.transform.Q,
	}

	distanceInput := b2.DistanceInput{
		ProxyA:     s.proxyA,
		ProxyB:     s.proxyB,
		TransformA: b2.TransformIdentity(),
		TransformB: transform,
		UseRadii:   false,
	}
	distanceCache := b2.SimplexCache{}
	distanceOutput := b2.ShapeDistance(&distanceInput, &distanceCache, nil)

	s.DrawTextLine("hit = %t, iterations = %d, fraction = %g, distance = %g", output.Hit, output.Iterations,
		ToFloat64(output.Fraction), ToFloat64(distanceOutput.Distance))

	draw := &s.Context.Draw

	s.shapes.drawShape(draw, s.typeA, b2.TransformIdentity(), FromFloat64(s.radiusA), b2.ColorCyan)
	s.shapes.drawShape(draw, s.typeB, s.transform, FromFloat64(s.radiusB), b2.ColorLightGreen)
	transform2 := b2.Transform{P: s.transform.P.Add(s.translation), Q: s.transform.Q}
	s.shapes.drawShape(draw, s.typeB, transform2, FromFloat64(s.radiusB), b2.ColorIndianRed)

	if output.Hit {
		s.shapes.drawShape(draw, s.typeB, transform, FromFloat64(s.radiusB), b2.ColorPlum)

		if output.Fraction.Greater(b2.QZero()) {
			draw.DrawPoint(output.Point, b2.F(5), b2.ColorWhite)
			draw.DrawSegment(output.Point, b2.MulAdd(output.Point, b2.QHalf(), output.Normal), b2.ColorYellow)
		} else {
			draw.DrawPoint(output.Point, b2.F(5), b2.ColorPeru)
		}
	}

	if s.showIndices {
		for i := 0; i < s.proxyA.Count; i++ {
			p := s.proxyA.Points[i]
			draw.DrawString(p, fmt.Sprintf(" %d", i), b2.ColorWhite)
		}

		for i := 0; i < s.proxyB.Count; i++ {
			p := b2.TransformPoint(s.transform, s.proxyB.Points[i])
			draw.DrawString(p, fmt.Sprintf(" %d", i), b2.ColorWhite)
		}
	}

	s.DrawTextLine("mouse button 1: drag")
	s.DrawTextLine("mouse button 1 + shift: rotate")
	s.DrawTextLine("mouse button 1 + control: sweep")
	s.DrawTextLine("distance = %.2f, iterations = %d", ToFloat64(distanceOutput.Distance), output.Iterations)
}

// TimeOfImpact replays a recorded time of impact between a box and a thin
// capsule.
type TimeOfImpact struct {
	Base

	verticesA [4]b2.Vec2
	verticesB [2]b2.Vec2

	radiusA b2.Q
	radiusB b2.Q
}

func NewTimeOfImpact(ctx *SampleContext) Sample {
	s := &TimeOfImpact{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: -16, Y: 45}
		ctx.Camera.Zoom = 5
	}

	s.verticesA = [4]b2.Vec2{b2.V2(-16.25, 44.75), b2.V2(-15.75, 44.75), b2.V2(-15.75, 45.25), b2.V2(-16.25, 45.25)}
	s.verticesB = [2]b2.Vec2{b2.V2(0.0, -0.125), b2.V2(0.0, 0.125)}

	s.radiusA = b2.QZero()
	s.radiusB = b2.F(0.0299999993)
	return s
}

func (s *TimeOfImpact) Step() {
	s.Base.Step()

	sweepA := b2.Sweep{
		Q1: b2.RotIdentity(),
		Q2: b2.RotIdentity(),
	}
	sweepB := b2.Sweep{
		C1: b2.V2(-15.8332710, 45.3520279),
		C2: b2.V2(-15.8324337, 45.3413048),
		Q1: b2.Rot{Cos: b2.F(-0.540891349), Sin: b2.F(0.841092527)},
		Q2: b2.Rot{Cos: b2.F(-0.457797021), Sin: b2.F(0.889056742)},
	}

	input := b2.TOIInput{
		ProxyA:      b2.MakeProxy(s.verticesA[:], s.radiusA),
		ProxyB:      b2.MakeProxy(s.verticesB[:], s.radiusB),
		SweepA:      sweepA,
		SweepB:      sweepB,
		MaxFraction: b2.QOne(),
	}

	output := b2.TimeOfImpact(&input)

	s.DrawTextLine("toi = %g", ToFloat64(output.Fraction))

	draw := &s.Context.Draw
	countA := len(s.verticesA)
	countB := len(s.verticesB)

	var vertices [b2.MaxPolygonVertices]b2.Vec2

	// Draw A
	transformA := b2.GetSweepTransform(&sweepA, b2.QZero())
	for i := 0; i < countA; i++ {
		vertices[i] = b2.TransformPoint(transformA, s.verticesA[i])
	}
	draw.DrawPolygon(vertices[:countA], b2.ColorGray)

	// Draw B at t = 0
	transformB := b2.GetSweepTransform(&sweepB, b2.QZero())
	for i := 0; i < countB; i++ {
		vertices[i] = b2.TransformPoint(transformB, s.verticesB[i])
	}
	draw.DrawSolidCapsule(vertices[0], vertices[1], s.radiusB, b2.ColorGreen)

	// Draw B at t = hit_time
	transformB = b2.GetSweepTransform(&sweepB, output.Fraction)
	for i := 0; i < countB; i++ {
		vertices[i] = b2.TransformPoint(transformB, s.verticesB[i])
	}
	draw.DrawPolygon(vertices[:countB], b2.ColorOrange)

	// Draw B at t = 1
	transformB = b2.GetSweepTransform(&sweepB, b2.QOne())
	for i := 0; i < countB; i++ {
		vertices[i] = b2.TransformPoint(transformB, s.verticesB[i])
	}
	draw.DrawSolidCapsule(vertices[0], vertices[1], s.radiusB, b2.ColorRed)

	if output.State == b2.TOIStateHit {
		distanceInput := b2.DistanceInput{
			ProxyA:     input.ProxyA,
			ProxyB:     input.ProxyB,
			TransformA: b2.GetSweepTransform(&sweepA, output.Fraction),
			TransformB: b2.GetSweepTransform(&sweepB, output.Fraction),
			UseRadii:   false,
		}
		cache := b2.SimplexCache{}
		distanceOutput := b2.ShapeDistance(&distanceInput, &cache, nil)
		s.DrawTextLine("distance = %g", ToFloat64(distanceOutput.Distance))
	}
}
