// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/sample_collision.cpp of Box2D v3.1.1

package samples

import (
	"fmt"
	"math"
	"time"

	"github.com/dhannyell/dbox2d"
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

// qs parses a decimal constant of the reference.
func qs(s string) dbox2d.Q { return dbox2d.QMustParse(s) }

// qv parses a vector constant of the reference.
func qv(x, y string) dbox2d.Vec2 { return dbox2d.Vec2{X: qs(x), Y: qs(y)} }

// rotFromRadians builds a rotation from a GUI angle, which stays in
// radians like the reference slider.
func rotFromRadians(angle float64) dbox2d.Rot { return dbox2d.MakeRot(radiansToTurns(angle)) }

func clampFloat(a, lower, upper float64) float64 { return math.Max(lower, math.Min(a, upper)) }

// drawSolidCircleAt draws a solid circle whose center is local to the
// transform, like the reference Draw::DrawSolidCircle.
func drawSolidCircleAt(draw *dbox2d.DebugDraw, transform dbox2d.Transform, center dbox2d.Vec2, radius dbox2d.Q, color dbox2d.HexColor) {
	draw.DrawSolidCircle(dbox2d.Transform{P: dbox2d.TransformPoint(transform, center), Q: transform.Q}, radius, color)
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
	point    dbox2d.Vec2
	segment  dbox2d.Segment
	triangle dbox2d.Polygon
	box      dbox2d.Polygon
}

func (ps *proxyShapes) makeProxy(shapeType int, radius dbox2d.Q) dbox2d.ShapeProxy {
	proxy := dbox2d.ShapeProxy{Radius: radius}

	switch shapeType {
	case proxyPoint:
		proxy.Points[0] = dbox2d.Vec2{}
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

func (ps *proxyShapes) drawShape(draw *dbox2d.DebugDraw, shapeType int, transform dbox2d.Transform, radius dbox2d.Q, color dbox2d.HexColor) {
	switch shapeType {
	case proxyPoint:
		p := dbox2d.TransformPoint(transform, ps.point)
		if radius.Greater(dbox2d.QZero()) {
			drawSolidCircleAt(draw, transform, ps.point, radius, color)
		} else {
			draw.DrawPoint(p, dbox2d.QFromInt(5), color)
		}

	case proxySegment:
		p1 := dbox2d.TransformPoint(transform, ps.segment.Point1)
		p2 := dbox2d.TransformPoint(transform, ps.segment.Point2)

		if radius.Greater(dbox2d.QZero()) {
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
func sliderQ(gui Gui, label string, v *dbox2d.Q, lo, hi float64) bool {
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
	proxyA, proxyB   dbox2d.ShapeProxy

	cache        dbox2d.SimplexCache
	simplexes    [shapeDistanceSimplexCapacity]dbox2d.Simplex
	simplexCount int
	simplexIndex int

	transform dbox2d.Transform
	angle     float64

	basePosition dbox2d.Vec2
	startPoint   dbox2d.Vec2
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

	s.shapes.segment = dbox2d.Segment{Point1: qv("-0.5", "0"), Point2: qv("0.5", "0")}

	{
		hull := dbox2d.ComputeHull([]dbox2d.Vec2{qv("-0.5", "0"), qv("0.5", "0"), qv("0", "1")})
		s.shapes.triangle = dbox2d.MakePolygon(&hull, dbox2d.QZero())
	}

	s.shapes.box = dbox2d.MakeSquare(dbox2d.QHalf())

	s.transform = dbox2d.TransformIdentity()

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
		s.simplexIndex = dbox2d.ClampInt(s.simplexIndex, 0, s.simplexCount-1)
	}

	gui.End()
}

func (s *CollisionShapeDistance) MouseDown(p dbox2d.Vec2, button MouseButton, mod Modifier) {
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

func (s *CollisionShapeDistance) MouseUp(p dbox2d.Vec2, button MouseButton) {
	if button == MouseButtonLeft {
		s.dragging = false
		s.rotating = false
	}
}

func (s *CollisionShapeDistance) MouseMove(p dbox2d.Vec2) {
	if s.dragging {
		s.transform.P = s.basePosition.Add(p.Sub(s.startPoint).Mul(dbox2d.QHalf()))
	} else if s.rotating {
		dx := ToFloat64(p.X.Sub(s.startPoint.X))
		s.angle = clampFloat(s.baseAngle+dx, -math.Pi, math.Pi)
		s.transform.Q = rotFromRadians(s.angle)
	}
}

func weight2(a1 dbox2d.Q, w1 dbox2d.Vec2, a2 dbox2d.Q, w2 dbox2d.Vec2) dbox2d.Vec2 {
	return dbox2d.Vec2{X: a1.Mul(w1.X).Add(a2.Mul(w2.X)), Y: a1.Mul(w1.Y).Add(a2.Mul(w2.Y))}
}

func weight3(a1 dbox2d.Q, w1 dbox2d.Vec2, a2 dbox2d.Q, w2 dbox2d.Vec2, a3 dbox2d.Q, w3 dbox2d.Vec2) dbox2d.Vec2 {
	return dbox2d.Vec2{
		X: a1.Mul(w1.X).Add(a2.Mul(w2.X)).Add(a3.Mul(w3.X)),
		Y: a1.Mul(w1.Y).Add(a2.Mul(w2.Y)).Add(a3.Mul(w3.Y)),
	}
}

func computeSimplexWitnessPoints(s *dbox2d.Simplex) (a, b dbox2d.Vec2) {
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
	input := dbox2d.DistanceInput{
		ProxyA:     s.proxyA,
		ProxyB:     s.proxyB,
		TransformA: dbox2d.TransformIdentity(),
		TransformB: s.transform,
		UseRadii:   true,
	}

	if !s.useCache {
		s.cache.Count = 0
	}

	output := dbox2d.ShapeDistance(&input, &s.cache, s.simplexes[:])

	s.simplexCount = output.SimplexCount

	draw := &s.Context.Draw
	s.shapes.drawShape(draw, s.typeA, dbox2d.TransformIdentity(), FromFloat64(s.radiusA), dbox2d.ColorCyan)
	s.shapes.drawShape(draw, s.typeB, s.transform, FromFloat64(s.radiusB), dbox2d.ColorBisque)

	ten := dbox2d.QFromInt(10)

	if s.drawSimplex && s.simplexIndex >= 0 && s.simplexIndex < s.simplexCount {
		simplex := &s.simplexes[s.simplexIndex]
		vertices := [3]*dbox2d.SimplexVertex{&simplex.V1, &simplex.V2, &simplex.V3}

		if s.simplexIndex > 0 {
			// The first recorded simplex does not have valid barycentric coordinates
			pointA, pointB := computeSimplexWitnessPoints(simplex)

			draw.DrawSegment(pointA, pointB, dbox2d.ColorWhite)
			draw.DrawPoint(pointA, ten, dbox2d.ColorWhite)
			draw.DrawPoint(pointB, ten, dbox2d.ColorWhite)
		}

		colors := [3]dbox2d.HexColor{dbox2d.ColorRed, dbox2d.ColorGreen, dbox2d.ColorBlue}

		for i := 0; i < simplex.Count; i++ {
			vertex := vertices[i]
			draw.DrawPoint(vertex.WA, ten, colors[i])
			draw.DrawPoint(vertex.WB, ten, colors[i])
		}
	} else {
		draw.DrawSegment(output.PointA, output.PointB, dbox2d.ColorDimGray)
		draw.DrawPoint(output.PointA, ten, dbox2d.ColorWhite)
		draw.DrawPoint(output.PointB, ten, dbox2d.ColorWhite)

		draw.DrawSegment(output.PointA, dbox2d.MulAdd(output.PointA, dbox2d.QHalf(), output.Normal), dbox2d.ColorYellow)
	}

	if s.showIndices {
		for i := 0; i < s.proxyA.Count; i++ {
			p := s.proxyA.Points[i]
			draw.DrawString(p, fmt.Sprintf(" %d", i), dbox2d.ColorWhite)
		}

		for i := 0; i < s.proxyB.Count; i++ {
			p := dbox2d.TransformPoint(s.transform, s.proxyB.Points[i])
			draw.DrawString(p, fmt.Sprintf(" %d", i), dbox2d.ColorWhite)
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
	box        dbox2d.AABB
	fatBox     dbox2d.AABB
	position   dbox2d.Vec2
	width      dbox2d.Vec2
	proxyId    int
	rayStamp   int
	queryStamp int
	moved      bool
}

// DynamicTree tests the Box2D bounding volume hierarchy (BVH). The dynamic
// tree can be used independently as a spatial data structure.
type DynamicTree struct {
	Base

	tree        *dbox2d.DynamicTree
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

	startPoint dbox2d.Vec2
	endPoint   dbox2d.Vec2

	rayDrag   bool
	queryDrag bool
	validate  bool

	// boxVertices is the scratch of drawAABB.
	boxVertices [4]dbox2d.Vec2
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

	y := dbox2d.QFromInt(-4)

	s.tree = dbox2d.NewDynamicTree()

	aabbMargin := dbox2d.Vec2{X: dbox2d.QFromRatio(1, 10), Y: dbox2d.QFromRatio(1, 10)}

	fill := FromFloat64(s.fill)
	grid := FromFloat64(s.grid)
	maxRatio := FromFloat64(s.ratio)

	for i := 0; i < s.rowCount; i++ {
		x := dbox2d.QFromInt(-40)

		for j := 0; j < s.columnCount; j++ {
			fillTest := randomFloatRange(dbox2d.QZero(), dbox2d.QOne())
			if !fillTest.Greater(fill) {
				var p treeProxy
				p.position = dbox2d.Vec2{X: x, Y: y}

				ratio := randomFloatRange(dbox2d.QOne(), maxRatio)
				width := randomFloatRange(dbox2d.QFromRatio(1, 10), dbox2d.QHalf())
				if randomFloat().Greater(dbox2d.QZero()) {
					p.width.X = ratio.Mul(width)
					p.width.Y = width
				} else {
					p.width.X = width
					p.width.Y = ratio.Mul(width)
				}

				p.box.LowerBound = dbox2d.Vec2{X: x, Y: y}
				p.box.UpperBound = dbox2d.Vec2{X: x.Add(p.width.X), Y: y.Add(p.width.Y)}
				p.fatBox.LowerBound = p.box.LowerBound.Sub(aabbMargin)
				p.fatBox.UpperBound = p.box.UpperBound.Add(aabbMargin)

				p.proxyId = s.tree.CreateProxy(p.fatBox, dbox2d.DefaultCategoryBits, uint64(len(s.proxies)))
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

func (s *DynamicTree) MouseDown(p dbox2d.Vec2, button MouseButton, mod Modifier) {
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

func (s *DynamicTree) MouseUp(p dbox2d.Vec2, button MouseButton) {
	if button == MouseButtonLeft {
		s.queryDrag = false
		s.rayDrag = false
	}
}

func (s *DynamicTree) MouseMove(p dbox2d.Vec2) {
	s.endPoint = p
}

// drawAABB draws a box outline, like the reference Draw::DrawAABB.
func (s *DynamicTree) drawAABB(box dbox2d.AABB, color dbox2d.HexColor) {
	v := &s.boxVertices
	v[0] = box.LowerBound
	v[1] = dbox2d.Vec2{X: box.UpperBound.X, Y: box.LowerBound.Y}
	v[2] = box.UpperBound
	v[3] = dbox2d.Vec2{X: box.LowerBound.X, Y: box.UpperBound.Y}
	s.Context.Draw.DrawPolygon(v[:], color)
}

func elapsedMilliseconds(start time.Time) float64 {
	return float64(time.Since(start).Nanoseconds()) / 1e6
}

func (s *DynamicTree) Step() {
	draw := &s.Context.Draw

	if s.queryDrag {
		box := dbox2d.AABB{LowerBound: dbox2d.Min(s.startPoint, s.endPoint), UpperBound: dbox2d.Max(s.startPoint, s.endPoint)}
		s.tree.Query(box, dbox2d.DefaultMaskBits, func(proxyId int, userData uint64) bool {
			proxy := &s.proxies[userData]
			proxy.queryStamp = s.timeStamp
			return true
		})

		s.drawAABB(box, dbox2d.ColorWhite)
	}

	if s.rayDrag {
		input := dbox2d.RayCastInput{Origin: s.startPoint, Translation: s.endPoint.Sub(s.startPoint), MaxFraction: dbox2d.QOne()}
		result := s.tree.RayCast(&input, dbox2d.DefaultMaskBits, func(input *dbox2d.RayCastInput, proxyId int, userData uint64) dbox2d.Q {
			proxy := &s.proxies[userData]
			proxy.rayStamp = s.timeStamp
			return input.MaxFraction
		})

		draw.DrawSegment(s.startPoint, s.endPoint, dbox2d.ColorWhite)
		draw.DrawPoint(s.startPoint, dbox2d.QFromInt(5), dbox2d.ColorGreen)
		draw.DrawPoint(s.endPoint, dbox2d.QFromInt(5), dbox2d.ColorRed)

		s.DrawTextLine("node visits = %d, leaf visits = %d", result.NodeVisits, result.LeafVisits)
	}

	c := dbox2d.ColorBlue
	qc := dbox2d.ColorGreen

	aabbMargin := dbox2d.Vec2{X: dbox2d.QFromRatio(1, 10), Y: dbox2d.QFromRatio(1, 10)}
	moveFraction := FromFloat64(s.moveFraction)
	moveDelta := FromFloat64(s.moveDelta)

	for i := range s.proxies {
		p := &s.proxies[i]

		if p.queryStamp == s.timeStamp || p.rayStamp == s.timeStamp {
			s.drawAABB(p.box, qc)
		} else {
			s.drawAABB(p.box, c)
		}

		moveTest := randomFloatRange(dbox2d.QZero(), dbox2d.QOne())
		if moveFraction.Greater(moveTest) {
			dx := moveDelta.Mul(randomFloat())
			dy := moveDelta.Mul(randomFloat())

			p.position.X = p.position.X.Add(dx)
			p.position.Y = p.position.Y.Add(dy)

			p.box.LowerBound.X = p.position.X.Add(dx)
			p.box.LowerBound.Y = p.position.Y.Add(dy)
			p.box.UpperBound.X = p.position.X.Add(dx).Add(p.width.X)
			p.box.UpperBound.Y = p.position.Y.Add(dy).Add(p.width.Y)

			if !dbox2d.AABBContains(p.fatBox, p.box) {
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

	box      dbox2d.Polygon
	triangle dbox2d.Polygon
	circle   dbox2d.Circle
	capsule  dbox2d.Capsule
	segment  dbox2d.Segment

	transform dbox2d.Transform
	angle     float64

	rayStart dbox2d.Vec2
	rayEnd   dbox2d.Vec2

	basePosition dbox2d.Vec2
	baseAngle    float64

	startPosition dbox2d.Vec2

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

	s.circle = dbox2d.Circle{Radius: dbox2d.QFromInt(2)}
	s.capsule = dbox2d.Capsule{Center1: qv("-1", "1"), Center2: qv("1", "-1"), Radius: qs("1.5")}
	s.box = dbox2d.MakeBox(dbox2d.QFromInt(2), dbox2d.QFromInt(2))

	hull := dbox2d.ComputeHull([]dbox2d.Vec2{qv("-2", "0"), qv("2", "0"), qv("2", "3")})
	s.triangle = dbox2d.MakePolygon(&hull, dbox2d.QZero())

	s.segment = dbox2d.Segment{Point1: qv("-3", "0"), Point2: qv("3", "0")}

	s.transform = dbox2d.TransformIdentity()

	s.rayStart = qv("0", "30")
	s.rayEnd = qv("0", "0")
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
		s.transform = dbox2d.TransformIdentity()
		s.angle = 0
	}

	gui.Text("mouse btn 1: ray cast")
	gui.Text("mouse btn 1 + shft: translate")
	gui.Text("mouse btn 1 + ctrl: rotate")

	gui.End()
}

func (s *RayCast) MouseDown(p dbox2d.Vec2, button MouseButton, mod Modifier) {
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

func (s *RayCast) MouseUp(p dbox2d.Vec2, button MouseButton) {
	if button == MouseButtonLeft {
		s.rayDrag = false
		s.rotating = false
		s.translating = false
	}
}

func (s *RayCast) MouseMove(p dbox2d.Vec2) {
	if s.rayDrag {
		s.rayEnd = p
	} else if s.translating {
		s.transform.P = s.basePosition.Add(p.Sub(s.startPosition).Mul(dbox2d.QHalf()))
	} else if s.rotating {
		dx := ToFloat64(p.X.Sub(s.startPosition.X))
		s.angle = clampFloat(s.baseAngle+0.5*dx, -math.Pi, math.Pi)
		s.transform.Q = rotFromRadians(s.angle)
	}
}

func (s *RayCast) drawRay(output *dbox2d.CastOutput) {
	draw := &s.Context.Draw
	five := dbox2d.QFromInt(5)

	p1 := s.rayStart
	p2 := s.rayEnd
	d := p2.Sub(p1)

	if output.Hit {
		var p dbox2d.Vec2

		if output.Fraction.Eq(dbox2d.QZero()) {
			p = output.Point
			draw.DrawPoint(output.Point, five, dbox2d.ColorPeru)
		} else {
			p = dbox2d.MulAdd(p1, output.Fraction, d)
			draw.DrawSegment(p1, p, dbox2d.ColorWhite)
			draw.DrawPoint(p1, five, dbox2d.ColorGreen)
			draw.DrawPoint(output.Point, five, dbox2d.ColorWhite)

			n := dbox2d.MulAdd(p, dbox2d.QOne(), output.Normal)
			draw.DrawSegment(p, n, dbox2d.ColorViolet)
		}

		if s.showFraction {
			ps := dbox2d.Vec2{X: p.X.Add(qs("0.05")), Y: p.Y.Sub(qs("0.02"))}
			draw.DrawString(ps, fmt.Sprintf("%.2f", ToFloat64(output.Fraction)), dbox2d.ColorWhite)
		}
	} else {
		draw.DrawSegment(p1, p2, dbox2d.ColorWhite)
		draw.DrawPoint(p1, five, dbox2d.ColorGreen)
		draw.DrawPoint(p2, five, dbox2d.ColorRed)
	}
}

func (s *RayCast) Step() {
	draw := &s.Context.Draw

	offset := qv("-20", "20")
	increment := qv("10", "0")

	color1 := dbox2d.ColorYellow

	output := dbox2d.CastOutput{}
	maxFraction := dbox2d.QOne()

	// cast runs a ray cast in the local frame of the transform and keeps
	// the closest hit in world space.
	cast := func(transform dbox2d.Transform, rayCast func(input *dbox2d.RayCastInput) dbox2d.CastOutput) {
		start := dbox2d.InvTransformPoint(transform, s.rayStart)
		translation := dbox2d.InvRotateVector(transform.Q, s.rayEnd.Sub(s.rayStart))
		input := dbox2d.RayCastInput{Origin: start, Translation: translation, MaxFraction: maxFraction}

		localOutput := rayCast(&input)
		if localOutput.Hit {
			output = localOutput
			output.Point = dbox2d.TransformPoint(transform, localOutput.Point)
			output.Normal = dbox2d.RotateVector(transform.Q, localOutput.Normal)
			maxFraction = localOutput.Fraction
		}
	}

	// circle
	{
		transform := dbox2d.Transform{P: s.transform.P.Add(offset), Q: s.transform.Q}
		drawSolidCircleAt(draw, transform, s.circle.Center, s.circle.Radius, color1)

		cast(transform, func(input *dbox2d.RayCastInput) dbox2d.CastOutput {
			return dbox2d.RayCastCircle(input, &s.circle)
		})

		offset = offset.Add(increment)
	}

	// capsule
	{
		transform := dbox2d.Transform{P: s.transform.P.Add(offset), Q: s.transform.Q}
		v1 := dbox2d.TransformPoint(transform, s.capsule.Center1)
		v2 := dbox2d.TransformPoint(transform, s.capsule.Center2)
		draw.DrawSolidCapsule(v1, v2, s.capsule.Radius, color1)

		cast(transform, func(input *dbox2d.RayCastInput) dbox2d.CastOutput {
			return dbox2d.RayCastCapsule(input, &s.capsule)
		})

		offset = offset.Add(increment)
	}

	// box
	{
		transform := dbox2d.Transform{P: s.transform.P.Add(offset), Q: s.transform.Q}
		draw.DrawSolidPolygon(transform, s.box.Vertices[:s.box.Count], dbox2d.QZero(), color1)

		cast(transform, func(input *dbox2d.RayCastInput) dbox2d.CastOutput {
			return dbox2d.RayCastPolygon(input, &s.box)
		})

		offset = offset.Add(increment)
	}

	// triangle
	{
		transform := dbox2d.Transform{P: s.transform.P.Add(offset), Q: s.transform.Q}
		draw.DrawSolidPolygon(transform, s.triangle.Vertices[:s.triangle.Count], dbox2d.QZero(), color1)

		cast(transform, func(input *dbox2d.RayCastInput) dbox2d.CastOutput {
			return dbox2d.RayCastPolygon(input, &s.triangle)
		})

		offset = offset.Add(increment)
	}

	// segment
	{
		transform := dbox2d.Transform{P: s.transform.P.Add(offset), Q: s.transform.Q}

		p1 := dbox2d.TransformPoint(transform, s.segment.Point1)
		p2 := dbox2d.TransformPoint(transform, s.segment.Point2)
		draw.DrawSegment(p1, p2, color1)

		cast(transform, func(input *dbox2d.RayCastInput) dbox2d.CastOutput {
			return dbox2d.RayCastSegment(input, &s.segment, false)
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
	points    [3]dbox2d.Vec2
	normals   [3]dbox2d.Vec2
	fractions [3]dbox2d.Q
	count     int
}

// castIgnored skips a specific shape. It also skips the initial overlap.
func castIgnored(shapeId dbox2d.ShapeId, fraction dbox2d.Q) bool {
	userData, _ := shapeId.GetUserData().(*castShapeUserData)
	return (userData != nil && userData.ignore) || fraction.Eq(dbox2d.QZero())
}

// rayCastClosest finds the closest hit. This is the most common callback
// used in games.
func (c *castContext) rayCastClosest(shapeId dbox2d.ShapeId, point, normal dbox2d.Vec2, fraction dbox2d.Q) dbox2d.Q {
	if castIgnored(shapeId, fraction) {
		// By returning -1, we instruct the calling code to ignore this shape and
		// continue the ray-cast to the next shape.
		return dbox2d.QOne().Neg()
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
func (c *castContext) rayCastAny(shapeId dbox2d.ShapeId, point, normal dbox2d.Vec2, fraction dbox2d.Q) dbox2d.Q {
	if castIgnored(shapeId, fraction) {
		return dbox2d.QOne().Neg()
	}

	c.points[0] = point
	c.normals[0] = normal
	c.fractions[0] = fraction
	c.count = 1

	// At this point we have a hit, so we know the ray is obstructed.
	// By returning 0, we instruct the calling code to terminate the ray-cast.
	return dbox2d.QZero()
}

// rayCastMultiple collects multiple hits along the ray. The shapes are not
// necessary reported in order, so we might not capture the closest shape.
// NOTE: shape hits are not ordered, so this may return hits in any order. This means that
// if you limit the number of results, you may discard the closest hit. You can see this
// behavior in the sample.
func (c *castContext) rayCastMultiple(shapeId dbox2d.ShapeId, point, normal dbox2d.Vec2, fraction dbox2d.Q) dbox2d.Q {
	if castIgnored(shapeId, fraction) {
		return dbox2d.QOne().Neg()
	}

	count := c.count

	c.points[count] = point
	c.normals[count] = normal
	c.fractions[count] = fraction
	c.count = count + 1

	if c.count == 3 {
		// At this point the buffer is full.
		// By returning 0, we instruct the calling code to terminate the ray-cast.
		return dbox2d.QZero()
	}

	// By returning 1, we instruct the caller to continue without clipping the ray.
	return dbox2d.QOne()
}

// rayCastSorted collects multiple hits along the ray and sorts them.
func (c *castContext) rayCastSorted(shapeId dbox2d.ShapeId, point, normal dbox2d.Vec2, fraction dbox2d.Q) dbox2d.Q {
	if castIgnored(shapeId, fraction) {
		return dbox2d.QOne().Neg()
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
	return dbox2d.QOne()
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
func castWorldPolygons(roundSecond bool) [4]dbox2d.Polygon {
	var polygons [4]dbox2d.Polygon

	{
		hull := dbox2d.ComputeHull([]dbox2d.Vec2{qv("-0.5", "0"), qv("0.5", "0"), qv("0", "1.5")})
		polygons[0] = dbox2d.MakePolygon(&hull, dbox2d.QZero())
	}

	{
		hull := dbox2d.ComputeHull([]dbox2d.Vec2{qv("-0.1", "0"), qv("0.1", "0"), qv("0", "1.5")})
		polygons[1] = dbox2d.MakePolygon(&hull, dbox2d.QZero())
		if roundSecond {
			polygons[1].Radius = dbox2d.QHalf()
		}
	}

	{
		w := dbox2d.QOne()
		sqrt2 := dbox2d.QFromInt(2).Sqrt()
		b := w.Div(dbox2d.QFromInt(2).Add(sqrt2))
		s := sqrt2.Mul(b)
		half := dbox2d.QHalf()

		vertices := []dbox2d.Vec2{
			{X: half.Mul(s), Y: dbox2d.QZero()},
			{X: half.Mul(w), Y: b},
			{X: half.Mul(w), Y: b.Add(s)},
			{X: half.Mul(s), Y: w},
			{X: half.Mul(s).Neg(), Y: w},
			{X: half.Mul(w).Neg(), Y: b.Add(s)},
			{X: half.Mul(w).Neg(), Y: b},
			{X: half.Mul(s).Neg(), Y: dbox2d.QZero()},
		}

		hull := dbox2d.ComputeHull(vertices)
		polygons[2] = dbox2d.MakePolygon(&hull, dbox2d.QZero())
	}

	polygons[3] = dbox2d.MakeBox(dbox2d.QHalf(), dbox2d.QHalf())
	return polygons
}

// CastWorld shows how to use the ray and shape cast functions on a world.
// This sample is configured to ignore initial overlap.
type CastWorld struct {
	Base

	bodyIndex int
	bodyIds   [castWorldMaxCount]dbox2d.BodyId
	userData  [castWorldMaxCount]castShapeUserData
	polygons  [4]dbox2d.Polygon
	capsule   dbox2d.Capsule
	circle    dbox2d.Circle
	segment   dbox2d.Segment

	simple bool

	mode        int
	ignoreIndex int

	castType   int
	castRadius float64

	angleAnchor dbox2d.Vec2
	baseAngle   float64
	angle       float64
	rotating    bool

	rayStart dbox2d.Vec2
	rayEnd   dbox2d.Vec2
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
		bodyDef := dbox2d.DefaultBodyDef()
		groundId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		shapeDef := dbox2d.DefaultShapeDef()
		segment := dbox2d.Segment{Point1: qv("-40", "0"), Point2: qv("40", "0")}
		dbox2d.CreateSegmentShape(groundId, &shapeDef, &segment)
	}

	s.polygons = castWorldPolygons(true)
	s.capsule = dbox2d.Capsule{Center1: qv("-0.5", "0"), Center2: qv("0.5", "0"), Radius: qs("0.25")}
	s.circle = dbox2d.Circle{Radius: dbox2d.QHalf()}
	s.segment = dbox2d.Segment{Point1: qv("-1", "0"), Point2: qv("1", "0")}

	s.mode = castModeClosest
	s.ignoreIndex = 7

	s.castType = castTypeRay
	s.castRadius = 0.5

	s.rayStart = qv("-20", "10")
	s.rayEnd = qv("20", "10")
	return s
}

func (s *CastWorld) create(index int) {
	if !s.bodyIds[s.bodyIndex].IsNull() {
		dbox2d.DestroyBody(s.bodyIds[s.bodyIndex])
		s.bodyIds[s.bodyIndex] = dbox2d.BodyId{}
	}

	x := randomFloatRange(dbox2d.QFromInt(-20), dbox2d.QFromInt(20))
	y := randomFloatRange(dbox2d.QZero(), dbox2d.QFromInt(20))

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Position = dbox2d.Vec2{X: x, Y: y}
	bodyDef.Rotation = dbox2d.MakeRot(randomFloatRange(dbox2d.QHalf().Neg(), dbox2d.QHalf()))

	mod := s.bodyIndex % 3
	if mod == 0 {
		bodyDef.Type = dbox2d.StaticBody
	} else if mod == 1 {
		bodyDef.Type = dbox2d.KinematicBody
	} else if mod == 2 {
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.GravityScale = dbox2d.QZero()
	}

	s.bodyIds[s.bodyIndex] = dbox2d.CreateBody(s.WorldId, &bodyDef)

	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.UserData = &s.userData[s.bodyIndex]
	s.userData[s.bodyIndex].ignore = false
	if s.bodyIndex == s.ignoreIndex {
		s.userData[s.bodyIndex].ignore = true
	}

	if index < 4 {
		dbox2d.CreatePolygonShape(s.bodyIds[s.bodyIndex], &shapeDef, &s.polygons[index])
	} else if index == 4 {
		dbox2d.CreateCircleShape(s.bodyIds[s.bodyIndex], &shapeDef, &s.circle)
	} else if index == 5 {
		dbox2d.CreateCapsuleShape(s.bodyIds[s.bodyIndex], &shapeDef, &s.capsule)
	} else {
		dbox2d.CreateSegmentShape(s.bodyIds[s.bodyIndex], &shapeDef, &s.segment)
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
			dbox2d.DestroyBody(s.bodyIds[i])
			s.bodyIds[i] = dbox2d.BodyId{}
			return
		}
	}
}

func (s *CastWorld) MouseDown(p dbox2d.Vec2, button MouseButton, mod Modifier) {
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

func (s *CastWorld) MouseUp(p dbox2d.Vec2, button MouseButton) {
	if button == MouseButtonLeft {
		s.dragging = false
		s.rotating = false
	}
}

func (s *CastWorld) MouseMove(p dbox2d.Vec2) {
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
	five := dbox2d.QFromInt(5)

	color1 := dbox2d.ColorGreen
	color2 := dbox2d.ColorLightGray
	color3 := dbox2d.ColorMagenta

	rayTranslation := s.rayEnd.Sub(s.rayStart)

	if s.simple {
		s.DrawTextLine("Simple closest point ray cast")

		// This version doesn't have a callback, but it doesn't skip the ignored shape
		result := s.WorldId.CastRayClosest(s.rayStart, rayTranslation, dbox2d.DefaultQueryFilter())

		if result.Hit && result.Fraction.Greater(dbox2d.QZero()) {
			c := dbox2d.MulAdd(s.rayStart, result.Fraction, rayTranslation)
			draw.DrawPoint(result.Point, five, color1)
			draw.DrawSegment(s.rayStart, c, color2)
			head := dbox2d.MulAdd(result.Point, dbox2d.QHalf(), result.Normal)
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
		context.fractions[0] = dbox2d.Huge
		context.fractions[1] = dbox2d.Huge
		context.fractions[2] = dbox2d.Huge

		functions := [...]dbox2d.CastResultFcn{
			context.rayCastAny,
			context.rayCastClosest,
			context.rayCastMultiple,
			context.rayCastSorted,
		}
		modeFcn := functions[s.mode]

		castRadius := FromFloat64(s.castRadius)
		transform := dbox2d.Transform{P: s.rayStart, Q: rotFromRadians(s.angle)}
		circle := dbox2d.Circle{Center: s.rayStart, Radius: castRadius}
		capsule := dbox2d.Capsule{
			Center1: dbox2d.TransformPoint(transform, qv("-0.25", "0")),
			Center2: dbox2d.TransformPoint(transform, qv("0.25", "0")),
			Radius:  castRadius,
		}
		box := dbox2d.MakeOffsetRoundedBox(qs("0.25"), dbox2d.QHalf(), transform.P, transform.Q, castRadius)
		var proxy dbox2d.ShapeProxy

		if s.castType == castTypeRay {
			s.WorldId.CastRay(s.rayStart, rayTranslation, dbox2d.DefaultQueryFilter(), modeFcn)
		} else {
			if s.castType == castTypeCircle {
				proxy = dbox2d.MakeProxy([]dbox2d.Vec2{circle.Center}, circle.Radius)
			} else if s.castType == castTypeCapsule {
				proxy = dbox2d.MakeProxy([]dbox2d.Vec2{capsule.Center1, capsule.Center2}, capsule.Radius)
			} else {
				proxy = dbox2d.MakeProxy(box.Vertices[:box.Count], box.Radius)
			}

			s.WorldId.CastShape(&proxy, rayTranslation, dbox2d.DefaultQueryFilter(), modeFcn)
		}

		if context.count > 0 {
			colors := [3]dbox2d.HexColor{dbox2d.ColorRed, dbox2d.ColorGreen, dbox2d.ColorBlue}
			for i := 0; i < context.count; i++ {
				c := dbox2d.MulAdd(s.rayStart, context.fractions[i], rayTranslation)
				p := context.points[i]
				n := context.normals[i]
				draw.DrawPoint(p, five, colors[i])
				draw.DrawSegment(s.rayStart, c, color2)
				head := dbox2d.MulAdd(p, dbox2d.QOne(), n)
				draw.DrawSegment(p, head, color3)

				t := rayTranslation.Mul(context.fractions[i])
				shiftedTransform := dbox2d.Transform{P: t, Q: dbox2d.RotIdentity()}

				if s.castType == castTypeCircle {
					drawSolidCircleAt(draw, shiftedTransform, circle.Center, castRadius, dbox2d.ColorYellow)
				} else if s.castType == castTypeCapsule {
					p1 := capsule.Center1.Add(t)
					p2 := capsule.Center2.Add(t)
					draw.DrawSolidCapsule(p1, p2, castRadius, dbox2d.ColorYellow)
				} else if s.castType == castTypePolygon {
					draw.DrawSolidPolygon(shiftedTransform, box.Vertices[:box.Count], box.Radius, dbox2d.ColorYellow)
				}
			}
		} else {
			shiftedTransform := dbox2d.Transform{P: transform.P.Add(rayTranslation), Q: transform.Q}
			draw.DrawSegment(s.rayStart, s.rayEnd, color2)

			if s.castType == castTypeCircle {
				drawSolidCircleAt(draw, shiftedTransform, dbox2d.Vec2{}, castRadius, dbox2d.ColorGray)
			} else if s.castType == castTypeCapsule {
				p1 := dbox2d.TransformPoint(transform, capsule.Center1).Add(rayTranslation)
				p2 := dbox2d.TransformPoint(transform, capsule.Center2).Add(rayTranslation)
				draw.DrawSolidCapsule(p1, p2, castRadius, dbox2d.ColorYellow)
			} else if s.castType == castTypePolygon {
				draw.DrawSolidPolygon(shiftedTransform, box.Vertices[:box.Count], box.Radius, dbox2d.ColorYellow)
			}
		}
	}

	draw.DrawPoint(s.rayStart, five, dbox2d.ColorGreen)

	if !s.bodyIds[s.ignoreIndex].IsNull() {
		p := s.bodyIds[s.ignoreIndex].GetPosition()
		p.X = p.X.Sub(qs("0.2"))
		draw.DrawString(p, "ign", dbox2d.ColorWhite)
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
	bodyIds     [castWorldMaxCount]dbox2d.BodyId
	userData    [castWorldMaxCount]castShapeUserData
	polygons    [4]dbox2d.Polygon
	capsule     dbox2d.Capsule
	circle      dbox2d.Circle
	segment     dbox2d.Segment
	ignoreIndex int

	doomIds   [overlapWorldMaxDoomed]dbox2d.ShapeId
	doomCount int

	shapeType int

	startPosition dbox2d.Vec2

	position  dbox2d.Vec2
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
	s.capsule = dbox2d.Capsule{Center1: qv("-0.5", "0"), Center2: qv("0.5", "0"), Radius: qs("0.25")}
	s.circle = dbox2d.Circle{Radius: dbox2d.QHalf()}
	s.segment = dbox2d.Segment{Point1: qv("-1", "0"), Point2: qv("1", "0")}

	s.ignoreIndex = 7

	s.shapeType = overlapCircle

	s.position = qv("0", "10")

	s.createN(0, 10)
	return s
}

func (s *OverlapWorld) create(index int) {
	if !s.bodyIds[s.bodyIndex].IsNull() {
		dbox2d.DestroyBody(s.bodyIds[s.bodyIndex])
		s.bodyIds[s.bodyIndex] = dbox2d.BodyId{}
	}

	x := randomFloatRange(dbox2d.QFromInt(-20), dbox2d.QFromInt(20))
	y := randomFloatRange(dbox2d.QZero(), dbox2d.QFromInt(20))

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Position = dbox2d.Vec2{X: x, Y: y}
	bodyDef.Rotation = dbox2d.MakeRot(randomFloatRange(dbox2d.QHalf().Neg(), dbox2d.QHalf()))

	s.bodyIds[s.bodyIndex] = dbox2d.CreateBody(s.WorldId, &bodyDef)

	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.UserData = &s.userData[s.bodyIndex]
	s.userData[s.bodyIndex].index = s.bodyIndex
	s.userData[s.bodyIndex].ignore = false
	if s.bodyIndex == s.ignoreIndex {
		s.userData[s.bodyIndex].ignore = true
	}

	if index < 4 {
		dbox2d.CreatePolygonShape(s.bodyIds[s.bodyIndex], &shapeDef, &s.polygons[index])
	} else if index == 4 {
		dbox2d.CreateCircleShape(s.bodyIds[s.bodyIndex], &shapeDef, &s.circle)
	} else if index == 5 {
		dbox2d.CreateCapsuleShape(s.bodyIds[s.bodyIndex], &shapeDef, &s.capsule)
	} else {
		dbox2d.CreateSegmentShape(s.bodyIds[s.bodyIndex], &shapeDef, &s.segment)
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
			dbox2d.DestroyBody(s.bodyIds[i])
			s.bodyIds[i] = dbox2d.BodyId{}
			return
		}
	}
}

func (s *OverlapWorld) MouseDown(p dbox2d.Vec2, button MouseButton, mod Modifier) {
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

func (s *OverlapWorld) MouseUp(p dbox2d.Vec2, button MouseButton) {
	if button == MouseButtonLeft {
		s.dragging = false
		s.rotating = false
	}
}

func (s *OverlapWorld) MouseMove(p dbox2d.Vec2) {
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

	transform := dbox2d.Transform{P: s.position, Q: rotFromRadians(s.angle)}
	var proxy dbox2d.ShapeProxy

	if s.shapeType == overlapCircle {
		circle := dbox2d.Circle{Center: transform.P, Radius: dbox2d.QOne()}
		proxy = dbox2d.MakeProxy([]dbox2d.Vec2{circle.Center}, circle.Radius)
		drawSolidCircleAt(draw, dbox2d.TransformIdentity(), circle.Center, circle.Radius, dbox2d.ColorWhite)
	} else if s.shapeType == overlapCapsule {
		capsule := dbox2d.Capsule{
			Center1: dbox2d.TransformPoint(transform, qv("-1", "0")),
			Center2: dbox2d.TransformPoint(transform, qv("1", "0")),
			Radius:  dbox2d.QHalf(),
		}
		proxy = dbox2d.MakeProxy([]dbox2d.Vec2{capsule.Center1, capsule.Center2}, capsule.Radius)
		draw.DrawSolidCapsule(capsule.Center1, capsule.Center2, capsule.Radius, dbox2d.ColorWhite)
	} else if s.shapeType == overlapBox {
		box := dbox2d.MakeOffsetBox(dbox2d.QFromInt(2), dbox2d.QHalf(), transform.P, transform.Q)
		proxy = dbox2d.MakeProxy(box.Vertices[:box.Count], box.Radius)
		draw.DrawPolygon(box.Vertices[:box.Count], dbox2d.ColorWhite)
	}

	s.WorldId.OverlapShape(&proxy, dbox2d.DefaultQueryFilter(), func(shapeId dbox2d.ShapeId) bool {
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
		p.X = p.X.Sub(qs("0.2"))
		draw.DrawString(p, "skip", dbox2d.ColorWhite)
	}

	for i := 0; i < s.doomCount; i++ {
		shapeId := s.doomIds[i]
		userData, _ := shapeId.GetUserData().(*castShapeUserData)
		if userData == nil {
			continue
		}

		index := userData.index
		dbox2d.DestroyBody(s.bodyIds[index])
		s.bodyIds[index] = dbox2d.BodyId{}
	}
}

// Manifold tests manifolds and contact points.
type Manifold struct {
	Base

	smgroxCache1 dbox2d.SimplexCache
	smgroxCache2 dbox2d.SimplexCache
	smgcapCache1 dbox2d.SimplexCache
	smgcapCache2 dbox2d.SimplexCache

	wedge dbox2d.Hull

	transform dbox2d.Transform
	angle     float64
	round     float64

	basePosition dbox2d.Vec2
	startPoint   dbox2d.Vec2
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

	s.transform = dbox2d.TransformIdentity()
	s.transform.P.X = qs("0.17")
	s.transform.P.Y = qs("1.12")
	s.round = 0.1

	s.enableCaching = true

	s.wedge = dbox2d.ComputeHull([]dbox2d.Vec2{qv("-0.1", "-0.5"), qv("0.1", "-0.5"), qv("0", "0.5")})
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
		s.transform = dbox2d.TransformIdentity()
		s.angle = 0
	}

	gui.Text("mouse button 1: drag")
	gui.Text("mouse button 1 + shift: rotate")

	gui.End()
}

func (s *Manifold) MouseDown(p dbox2d.Vec2, button MouseButton, mod Modifier) {
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

func (s *Manifold) MouseUp(p dbox2d.Vec2, button MouseButton) {
	if button == MouseButtonLeft {
		s.dragging = false
		s.rotating = false
	}
}

func (s *Manifold) MouseMove(p dbox2d.Vec2) {
	if s.dragging {
		s.transform.P = s.basePosition.Add(p.Sub(s.startPoint).Mul(dbox2d.QHalf()))
	} else if s.rotating {
		dx := ToFloat64(p.X.Sub(s.startPoint.X))
		s.angle = clampFloat(s.baseAngle+1.0*dx, -math.Pi, math.Pi)
		s.transform.Q = rotFromRadians(s.angle)
	}
}

func (s *Manifold) drawManifold(manifold *dbox2d.Manifold, origin1, origin2 dbox2d.Vec2) {
	draw := &s.Context.Draw

	if s.showCount {
		p := origin1.Add(origin2).Mul(dbox2d.QHalf())
		draw.DrawString(p, fmt.Sprintf("%d", manifold.PointCount), dbox2d.ColorWhite)
	}

	for i := 0; i < manifold.PointCount; i++ {
		mp := &manifold.Points[i]

		p1 := mp.Point
		p2 := dbox2d.MulAdd(p1, dbox2d.QHalf(), manifold.Normal)
		draw.DrawSegment(p1, p2, dbox2d.ColorViolet)

		if s.showAnchors {
			draw.DrawPoint(origin1.Add(mp.AnchorA), dbox2d.QFromInt(5), dbox2d.ColorRed)
			draw.DrawPoint(origin2.Add(mp.AnchorB), dbox2d.QFromInt(5), dbox2d.ColorGreen)
		} else {
			draw.DrawPoint(p1, dbox2d.QFromInt(10), dbox2d.ColorBlue)
		}

		if s.showIds {
			p := dbox2d.Vec2{X: p1.X.Add(qs("0.05")), Y: p1.Y.Sub(qs("0.02"))}
			draw.DrawString(p, fmt.Sprintf("0x%04x", mp.Id), dbox2d.ColorWhite)
		}

		if s.showSeparation {
			p := dbox2d.Vec2{X: p1.X.Add(qs("0.05")), Y: p1.Y.Add(qs("0.03"))}
			draw.DrawString(p, fmt.Sprintf("%.3f", ToFloat64(mp.Separation)), dbox2d.ColorWhite)
		}
	}
}

func (s *Manifold) Step() {
	draw := &s.Context.Draw

	offset := qv("-10", "-5")
	increment := qv("4", "0")

	color1 := dbox2d.ColorAquamarine
	color2 := dbox2d.ColorPaleGoldenRod

	if !s.enableCaching {
		s.smgroxCache1 = dbox2d.SimplexCache{}
		s.smgroxCache2 = dbox2d.SimplexCache{}
		s.smgcapCache1 = dbox2d.SimplexCache{}
		s.smgcapCache2 = dbox2d.SimplexCache{}
	}

	round := FromFloat64(s.round)
	h := dbox2d.QHalf().Sub(round)

	// transforms returns the fixed frame and the dragged frame at the
	// current offset.
	transforms := func() (dbox2d.Transform, dbox2d.Transform) {
		return dbox2d.Transform{P: offset, Q: dbox2d.RotIdentity()},
			dbox2d.Transform{P: s.transform.P.Add(offset), Q: s.transform.Q}
	}

	// circle-circle
	{
		circle1 := dbox2d.Circle{Radius: dbox2d.QHalf()}
		circle2 := dbox2d.Circle{Radius: dbox2d.QOne()}

		transform1, transform2 := transforms()

		m := dbox2d.CollideCircles(&circle1, transform1, &circle2, transform2)

		drawSolidCircleAt(draw, transform1, circle1.Center, circle1.Radius, color1)
		drawSolidCircleAt(draw, transform2, circle2.Center, circle2.Radius, color2)

		s.drawManifold(&m, transform1.P, transform2.P)

		offset = offset.Add(increment)
	}

	// capsule-circle
	{
		capsule := dbox2d.Capsule{Center1: qv("-0.5", "0"), Center2: qv("0.5", "0"), Radius: qs("0.25")}
		circle := dbox2d.Circle{Radius: dbox2d.QHalf()}

		transform1, transform2 := transforms()

		m := dbox2d.CollideCapsuleAndCircle(&capsule, transform1, &circle, transform2)

		v1 := dbox2d.TransformPoint(transform1, capsule.Center1)
		v2 := dbox2d.TransformPoint(transform1, capsule.Center2)
		draw.DrawSolidCapsule(v1, v2, capsule.Radius, color1)

		drawSolidCircleAt(draw, transform2, circle.Center, circle.Radius, color2)

		s.drawManifold(&m, transform1.P, transform2.P)

		offset = offset.Add(increment)
	}

	// segment-circle
	{
		segment := dbox2d.Segment{Point1: qv("-1", "0"), Point2: qv("1", "0")}
		circle := dbox2d.Circle{Radius: dbox2d.QHalf()}

		transform1, transform2 := transforms()

		m := dbox2d.CollideSegmentAndCircle(&segment, transform1, &circle, transform2)

		p1 := dbox2d.TransformPoint(transform1, segment.Point1)
		p2 := dbox2d.TransformPoint(transform1, segment.Point2)
		draw.DrawSegment(p1, p2, color1)

		drawSolidCircleAt(draw, transform2, circle.Center, circle.Radius, color2)

		s.drawManifold(&m, transform1.P, transform2.P)

		offset = offset.Add(increment)
	}

	// box-circle
	{
		circle := dbox2d.Circle{Radius: dbox2d.QHalf()}
		box := dbox2d.MakeSquare(dbox2d.QHalf())
		box.Radius = round

		transform1, transform2 := transforms()

		m := dbox2d.CollidePolygonAndCircle(&box, transform1, &circle, transform2)

		draw.DrawSolidPolygon(transform1, box.Vertices[:box.Count], round, color1)
		drawSolidCircleAt(draw, transform2, circle.Center, circle.Radius, color2)

		s.drawManifold(&m, transform1.P, transform2.P)

		offset = offset.Add(increment)
	}

	// capsule-capsule
	{
		capsule1 := dbox2d.Capsule{Center1: qv("-0.5", "0"), Center2: qv("0.5", "0"), Radius: qs("0.25")}
		capsule2 := dbox2d.Capsule{Center1: qv("0.25", "0"), Center2: qv("1", "0"), Radius: qs("0.1")}

		transform1, transform2 := transforms()

		m := dbox2d.CollideCapsules(&capsule1, transform1, &capsule2, transform2)

		v1 := dbox2d.TransformPoint(transform1, capsule1.Center1)
		v2 := dbox2d.TransformPoint(transform1, capsule1.Center2)
		draw.DrawSolidCapsule(v1, v2, capsule1.Radius, color1)

		v1 = dbox2d.TransformPoint(transform2, capsule2.Center1)
		v2 = dbox2d.TransformPoint(transform2, capsule2.Center2)
		draw.DrawSolidCapsule(v1, v2, capsule2.Radius, color2)

		s.drawManifold(&m, transform1.P, transform2.P)

		offset = offset.Add(increment)
	}

	// box-capsule
	{
		capsule := dbox2d.Capsule{Center1: qv("-0.4", "0"), Center2: qv("-0.1", "0"), Radius: qs("0.1")}
		box := dbox2d.MakeOffsetBox(qs("0.25"), dbox2d.QOne(), qv("1", "-1"), dbox2d.MakeRot(dbox2d.QFromRatio(1, 8)))

		transform1, transform2 := transforms()

		m := dbox2d.CollidePolygonAndCapsule(&box, transform1, &capsule, transform2)

		draw.DrawSolidPolygon(transform1, box.Vertices[:box.Count], box.Radius, color1)

		v1 := dbox2d.TransformPoint(transform2, capsule.Center1)
		v2 := dbox2d.TransformPoint(transform2, capsule.Center2)
		draw.DrawSolidCapsule(v1, v2, capsule.Radius, color2)

		s.drawManifold(&m, transform1.P, transform2.P)

		offset = offset.Add(increment)
	}

	// segment-capsule
	{
		segment := dbox2d.Segment{Point1: qv("-1", "0"), Point2: qv("1", "0")}
		capsule := dbox2d.Capsule{Center1: qv("-0.5", "0"), Center2: qv("0.5", "0"), Radius: qs("0.25")}

		transform1, transform2 := transforms()

		m := dbox2d.CollideSegmentAndCapsule(&segment, transform1, &capsule, transform2)

		p1 := dbox2d.TransformPoint(transform1, segment.Point1)
		p2 := dbox2d.TransformPoint(transform1, segment.Point2)
		draw.DrawSegment(p1, p2, color1)

		p1 = dbox2d.TransformPoint(transform2, capsule.Center1)
		p2 = dbox2d.TransformPoint(transform2, capsule.Center2)
		draw.DrawSolidCapsule(p1, p2, capsule.Radius, color2)

		s.drawManifold(&m, transform1.P, transform2.P)

		offset = offset.Add(increment)
	}

	offset = qv("-10", "0")

	// square-square
	{
		box1 := dbox2d.MakeSquare(dbox2d.QHalf())
		box := dbox2d.MakeSquare(dbox2d.QHalf())

		transform1, transform2 := transforms()

		m := dbox2d.CollidePolygons(&box1, transform1, &box, transform2)

		draw.DrawSolidPolygon(transform1, box1.Vertices[:box1.Count], box1.Radius, color1)
		draw.DrawSolidPolygon(transform2, box.Vertices[:box.Count], box.Radius, color2)

		s.drawManifold(&m, transform1.P, transform2.P)

		offset = offset.Add(increment)
	}

	// box-box
	{
		box1 := dbox2d.MakeBox(dbox2d.QFromInt(2), qs("0.1"))
		box := dbox2d.MakeSquare(qs("0.25"))

		transform1, transform2 := transforms()

		m := dbox2d.CollidePolygons(&box1, transform1, &box, transform2)

		draw.DrawSolidPolygon(transform1, box1.Vertices[:box1.Count], box1.Radius, color1)
		draw.DrawSolidPolygon(transform2, box.Vertices[:box.Count], box.Radius, color2)

		s.drawManifold(&m, transform1.P, transform2.P)

		offset = offset.Add(increment)
	}

	// box-rox
	{
		box := dbox2d.MakeSquare(dbox2d.QHalf())
		rox := dbox2d.MakeRoundedBox(h, h, round)

		transform1, transform2 := transforms()

		m := dbox2d.CollidePolygons(&box, transform1, &rox, transform2)

		draw.DrawSolidPolygon(transform1, box.Vertices[:box.Count], box.Radius, color1)
		draw.DrawSolidPolygon(transform2, rox.Vertices[:rox.Count], rox.Radius, color2)

		s.drawManifold(&m, transform1.P, transform2.P)

		offset = offset.Add(increment)
	}

	// rox-rox
	{
		rox := dbox2d.MakeRoundedBox(h, h, round)

		transform1, transform2 := transforms()

		m := dbox2d.CollidePolygons(&rox, transform1, &rox, transform2)

		draw.DrawSolidPolygon(transform1, rox.Vertices[:rox.Count], rox.Radius, color1)
		draw.DrawSolidPolygon(transform2, rox.Vertices[:rox.Count], rox.Radius, color2)

		s.drawManifold(&m, transform1.P, transform2.P)

		offset = offset.Add(increment)
	}

	// segment-rox
	{
		segment := dbox2d.Segment{Point1: qv("-1", "0"), Point2: qv("1", "0")}
		rox := dbox2d.MakeRoundedBox(h, h, round)

		transform1, transform2 := transforms()

		m := dbox2d.CollideSegmentAndPolygon(&segment, transform1, &rox, transform2)

		p1 := dbox2d.TransformPoint(transform1, segment.Point1)
		p2 := dbox2d.TransformPoint(transform1, segment.Point2)
		draw.DrawSegment(p1, p2, color1)
		draw.DrawSolidPolygon(transform2, rox.Vertices[:rox.Count], rox.Radius, color2)

		s.drawManifold(&m, transform1.P, transform2.P)

		offset = offset.Add(increment)
	}

	// wox-wox
	{
		wox := dbox2d.MakePolygon(&s.wedge, round)

		transform1, transform2 := transforms()

		m := dbox2d.CollidePolygons(&wox, transform1, &wox, transform2)

		draw.DrawSolidPolygon(transform1, wox.Vertices[:wox.Count], wox.Radius, color1)
		draw.DrawSolidPolygon(transform1, wox.Vertices[:wox.Count], dbox2d.QZero(), color1)
		draw.DrawSolidPolygon(transform2, wox.Vertices[:wox.Count], wox.Radius, color2)
		draw.DrawSolidPolygon(transform2, wox.Vertices[:wox.Count], dbox2d.QZero(), color2)

		s.drawManifold(&m, transform1.P, transform2.P)

		offset = offset.Add(increment)
	}

	// wox-wox
	{
		p1s := []dbox2d.Vec2{qv("0.175740838", "0.224936664"), qv("-0.301293969", "0.194021404"), qv("-0.105151534", "-0.432157338")}
		p2s := []dbox2d.Vec2{qv("-0.427884758", "-0.225028217"), qv("0.0566576123", "-0.128772855"), qv("0.176625848", "0.338923335")}

		h1 := dbox2d.ComputeHull(p1s)
		h2 := dbox2d.ComputeHull(p2s)
		w1 := dbox2d.MakePolygon(&h1, qs("0.158798501"))
		w2 := dbox2d.MakePolygon(&h2, qs("0.205900759"))

		transform1, transform2 := transforms()

		m := dbox2d.CollidePolygons(&w1, transform1, &w2, transform2)

		draw.DrawSolidPolygon(transform1, w1.Vertices[:w1.Count], w1.Radius, color1)
		draw.DrawSolidPolygon(transform1, w1.Vertices[:w1.Count], dbox2d.QZero(), color1)
		draw.DrawSolidPolygon(transform2, w2.Vertices[:w2.Count], w2.Radius, color2)
		draw.DrawSolidPolygon(transform2, w2.Vertices[:w2.Count], dbox2d.QZero(), color2)

		s.drawManifold(&m, transform1.P, transform2.P)

		offset = offset.Add(increment)
	}

	offset = qv("-10", "5")

	// box-triangle
	{
		box := dbox2d.MakeBox(dbox2d.QOne(), dbox2d.QOne())
		hull := dbox2d.ComputeHull([]dbox2d.Vec2{qv("-0.05", "0"), qv("0.05", "0"), qv("0", "0.1")})
		tri := dbox2d.MakePolygon(&hull, dbox2d.QZero())

		transform1, transform2 := transforms()

		m := dbox2d.CollidePolygons(&box, transform1, &tri, transform2)

		draw.DrawSolidPolygon(transform1, box.Vertices[:box.Count], dbox2d.QZero(), color1)
		draw.DrawSolidPolygon(transform2, tri.Vertices[:tri.Count], dbox2d.QZero(), color2)

		s.drawManifold(&m, transform1.P, transform2.P)

		offset = offset.Add(increment)
	}

	two := dbox2d.QFromInt(2)
	four := dbox2d.QFromInt(4)
	five := dbox2d.QFromInt(5)

	segment1 := dbox2d.ChainSegment{
		Ghost1:  qv("2", "1"),
		Segment: dbox2d.Segment{Point1: qv("1", "1"), Point2: qv("-1", "0")},
		Ghost2:  qv("-2", "0"),
		ChainId: -1,
	}
	segment2 := dbox2d.ChainSegment{
		Ghost1:  qv("3", "1"),
		Segment: dbox2d.Segment{Point1: qv("2", "1"), Point2: qv("1", "1")},
		Ghost2:  qv("-1", "0"),
		ChainId: -1,
	}

	// drawChainPair draws segment1 with its head ghost and segment2 with
	// its tail ghost.
	drawChainPair := func(transform1 dbox2d.Transform) {
		{
			g2 := dbox2d.TransformPoint(transform1, segment1.Ghost2)
			p1 := dbox2d.TransformPoint(transform1, segment1.Segment.Point1)
			p2 := dbox2d.TransformPoint(transform1, segment1.Segment.Point2)
			draw.DrawSegment(p1, p2, color1)
			draw.DrawPoint(p1, four, color1)
			draw.DrawPoint(p2, four, color1)
			draw.DrawSegment(p2, g2, dbox2d.ColorLightGray)
		}

		{
			g1 := dbox2d.TransformPoint(transform1, segment2.Ghost1)
			p1 := dbox2d.TransformPoint(transform1, segment2.Segment.Point1)
			p2 := dbox2d.TransformPoint(transform1, segment2.Segment.Point2)
			draw.DrawSegment(g1, p1, dbox2d.ColorLightGray)
			draw.DrawSegment(p1, p2, color1)
			draw.DrawPoint(p1, four, color1)
			draw.DrawPoint(p2, four, color1)
		}
	}

	// chain-segment vs circle
	{
		segment := segment1
		circle := dbox2d.Circle{Radius: dbox2d.QHalf()}

		transform1, transform2 := transforms()

		m := dbox2d.CollideChainSegmentAndCircle(&segment, transform1, &circle, transform2)

		g1 := dbox2d.TransformPoint(transform1, segment.Ghost1)
		g2 := dbox2d.TransformPoint(transform1, segment.Ghost2)
		p1 := dbox2d.TransformPoint(transform1, segment.Segment.Point1)
		p2 := dbox2d.TransformPoint(transform1, segment.Segment.Point2)
		draw.DrawSegment(g1, p1, dbox2d.ColorLightGray)
		draw.DrawSegment(p1, p2, color1)
		draw.DrawSegment(p2, g2, dbox2d.ColorLightGray)
		drawSolidCircleAt(draw, transform2, circle.Center, circle.Radius, color2)

		s.drawManifold(&m, transform1.P, transform2.P)

		offset.X = offset.X.Add(two.Mul(increment.X))
	}

	// chain-segment vs rounded polygon
	{
		rox := dbox2d.MakeRoundedBox(h, h, round)

		transform1, transform2 := transforms()

		m1 := dbox2d.CollideChainSegmentAndPolygon(&segment1, transform1, &rox, transform2, &s.smgroxCache1)
		m2 := dbox2d.CollideChainSegmentAndPolygon(&segment2, transform1, &rox, transform2, &s.smgroxCache2)

		drawChainPair(transform1)

		draw.DrawSolidPolygon(transform2, rox.Vertices[:rox.Count], rox.Radius, color2)
		draw.DrawPoint(dbox2d.TransformPoint(transform2, rox.Centroid), five, dbox2d.ColorGainsboro)

		s.drawManifold(&m1, transform1.P, transform2.P)
		s.drawManifold(&m2, transform1.P, transform2.P)

		offset.X = offset.X.Add(two.Mul(increment.X))
	}

	// chain-segment vs capsule
	{
		capsule := dbox2d.Capsule{Center1: qv("-0.5", "0"), Center2: qv("0.5", "0"), Radius: qs("0.25")}

		transform1, transform2 := transforms()

		m1 := dbox2d.CollideChainSegmentAndCapsule(&segment1, transform1, &capsule, transform2, &s.smgcapCache1)
		m2 := dbox2d.CollideChainSegmentAndCapsule(&segment2, transform1, &capsule, transform2, &s.smgcapCache2)

		drawChainPair(transform1)

		p1 := dbox2d.TransformPoint(transform2, capsule.Center1)
		p2 := dbox2d.TransformPoint(transform2, capsule.Center2)
		draw.DrawSolidCapsule(p1, p2, capsule.Radius, color2)

		draw.DrawPoint(dbox2d.Lerp(p1, p2, dbox2d.QHalf()), five, dbox2d.ColorGainsboro)

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
var smoothManifoldPoints = [...][2]string{
	{"-20.58325", "14.54175"},
	{"-21.90625", "15.8645"},
	{"-24.552", "17.1875"},
	{"-27.198", "11.89575"},
	{"-29.84375", "15.8645"},
	{"-29.84375", "21.15625"},
	{"-25.875", "23.802"},
	{"-20.58325", "25.125"},
	{"-25.875", "29.09375"},
	{"-20.58325", "31.7395"},
	{"-11.0089998", "23.2290001"},
	{"-8.67700005", "21.15625"},
	{"-6.03125", "21.15625"},
	{"-7.35424995", "29.09375"},
	{"-3.38549995", "29.09375"},
	{"1.90625", "30.41675"},
	{"5.875", "17.1875"},
	{"11.16675", "25.125"},
	{"9.84375", "29.09375"},
	{"13.8125", "31.7395"},
	{"21.75", "30.41675"},
	{"28.3644981", "26.448"},
	{"25.71875", "18.5105"},
	{"24.3957481", "13.21875"},
	{"17.78125", "11.89575"},
	{"15.1355", "7.92700005"},
	{"5.875", "9.25"},
	{"1.90625", "11.89575"},
	{"-3.25", "11.89575"},
	{"-3.25", "9.9375"},
	{"-4.70825005", "9.25"},
	{"-8.67700005", "9.25"},
	{"-11.323", "11.89575"},
	{"-13.96875", "11.89575"},
	{"-15.29175", "14.54175"},
	{"-19.2605", "14.54175"},
}

// SmoothManifold collides a shape against a closed loop of chain segments.
type SmoothManifold struct {
	Base

	shapeType int

	segments []dbox2d.ChainSegment

	transform dbox2d.Transform
	angle     float64
	round     float64

	basePosition dbox2d.Vec2
	startPoint   dbox2d.Vec2
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
	s.transform = dbox2d.Transform{P: qv("0", "20"), Q: dbox2d.RotIdentity()}

	count := len(smoothManifoldPoints)
	points := make([]dbox2d.Vec2, count)
	for i, p := range smoothManifoldPoints {
		points[i] = qv(p[0], p[1])
	}

	s.segments = make([]dbox2d.ChainSegment, count)

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

		s.segments[i] = dbox2d.ChainSegment{Ghost1: g1, Segment: dbox2d.Segment{Point1: p1, Point2: p2}, Ghost2: g2, ChainId: -1}
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
		s.transform = dbox2d.TransformIdentity()
		s.angle = 0
	}

	gui.Text("mouse button 1: drag")
	gui.Text("mouse button 1 + shift: rotate")

	gui.End()
}

func (s *SmoothManifold) MouseDown(p dbox2d.Vec2, button MouseButton, mod Modifier) {
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

func (s *SmoothManifold) MouseUp(p dbox2d.Vec2, button MouseButton) {
	if button == MouseButtonLeft {
		s.dragging = false
		s.rotating = false
	}
}

func (s *SmoothManifold) MouseMove(p dbox2d.Vec2) {
	if s.dragging {
		s.transform.P = s.basePosition.Add(p.Sub(s.startPoint))
	} else if s.rotating {
		dx := ToFloat64(p.X.Sub(s.startPoint.X))
		s.angle = clampFloat(s.baseAngle+1.0*dx, -math.Pi, math.Pi)
		s.transform.Q = rotFromRadians(s.angle)
	}
}

func (s *SmoothManifold) drawManifold(manifold *dbox2d.Manifold) {
	draw := &s.Context.Draw

	for i := 0; i < manifold.PointCount; i++ {
		mp := &manifold.Points[i]

		p1 := mp.Point
		p2 := dbox2d.MulAdd(p1, dbox2d.QHalf(), manifold.Normal)
		draw.DrawSegment(p1, p2, dbox2d.ColorWhite)

		// The reference draws the same point with or without anchors.
		draw.DrawPoint(p1, dbox2d.QFromInt(5), dbox2d.ColorGreen)

		if s.showIds {
			p := dbox2d.Vec2{X: p1.X.Add(qs("0.05")), Y: p1.Y.Sub(qs("0.02"))}
			draw.DrawString(p, fmt.Sprintf("0x%04x", mp.Id), dbox2d.ColorWhite)
		}

		if s.showSeparation {
			p := dbox2d.Vec2{X: p1.X.Add(qs("0.05")), Y: p1.Y.Add(qs("0.03"))}
			draw.DrawString(p, fmt.Sprintf("%.3f", ToFloat64(mp.Separation)), dbox2d.ColorWhite)
		}
	}
}

func (s *SmoothManifold) Step() {
	draw := &s.Context.Draw

	color1 := dbox2d.ColorYellow
	color2 := dbox2d.ColorMagenta

	transform1 := dbox2d.TransformIdentity()
	transform2 := s.transform

	for i := range s.segments {
		segment := &s.segments[i]
		p1 := dbox2d.TransformPoint(transform1, segment.Segment.Point1)
		p2 := dbox2d.TransformPoint(transform1, segment.Segment.Point2)
		draw.DrawSegment(p1, p2, color1)
		draw.DrawPoint(p1, dbox2d.QFromInt(4), color1)
	}

	// chain-segment vs circle
	if s.shapeType == smoothCircle {
		circle := dbox2d.Circle{Radius: dbox2d.QHalf()}
		drawSolidCircleAt(draw, transform2, circle.Center, circle.Radius, color2)

		for i := range s.segments {
			m := dbox2d.CollideChainSegmentAndCircle(&s.segments[i], transform1, &circle, transform2)
			s.drawManifold(&m)
		}
	} else if s.shapeType == smoothBox {
		round := FromFloat64(s.round)
		h := dbox2d.QHalf().Sub(round)
		rox := dbox2d.MakeRoundedBox(h, h, round)
		draw.DrawSolidPolygon(transform2, rox.Vertices[:rox.Count], rox.Radius, color2)

		for i := range s.segments {
			cache := dbox2d.SimplexCache{}
			m := dbox2d.CollideChainSegmentAndPolygon(&s.segments[i], transform1, &rox, transform2, &cache)
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
	proxyA, proxyB   dbox2d.ShapeProxy

	transform   dbox2d.Transform
	angle       float64
	translation dbox2d.Vec2

	basePosition dbox2d.Vec2
	startPoint   dbox2d.Vec2
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

	s.shapes.segment = dbox2d.Segment{Point1: qv("0", "0"), Point2: qv("0.5", "0")}

	{
		hull := dbox2d.ComputeHull([]dbox2d.Vec2{qv("-0.5", "0"), qv("0.5", "0"), qv("0", "1")})
		s.shapes.triangle = dbox2d.MakePolygon(&hull, dbox2d.QZero())
	}

	s.shapes.box = dbox2d.MakeOffsetBox(dbox2d.QHalf(), dbox2d.QHalf(), dbox2d.Vec2{}, dbox2d.RotIdentity())

	s.transform = dbox2d.Transform{P: qv("-0.6", "0"), Q: dbox2d.RotIdentity()}
	s.translation = qv("2", "0")

	s.typeA = proxyBox
	s.typeB = proxyPoint
	s.radiusA = 0
	s.radiusB = 0.2

	s.proxyA = s.shapes.makeProxy(s.typeA, FromFloat64(s.radiusA))
	s.proxyB = s.shapes.makeProxy(s.typeB, FromFloat64(s.radiusB))
	return s
}

func (s *ShapeCast) MouseDown(p dbox2d.Vec2, button MouseButton, mod Modifier) {
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
			s.basePosition = dbox2d.Vec2{}
		}
	}
}

func (s *ShapeCast) MouseUp(p dbox2d.Vec2, button MouseButton) {
	if button == MouseButtonLeft {
		s.dragging = false
		s.sweeping = false
		s.rotating = false
	}
}

func (s *ShapeCast) MouseMove(p dbox2d.Vec2) {
	if s.dragging {
		s.transform.P = s.basePosition.Add(p.Sub(s.startPoint).Mul(dbox2d.QHalf()))
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

	input := dbox2d.ShapeCastPairInput{
		ProxyA:       s.proxyA,
		ProxyB:       s.proxyB,
		TransformA:   dbox2d.TransformIdentity(),
		TransformB:   s.transform,
		TranslationB: s.translation,
		MaxFraction:  dbox2d.QOne(),
		CanEncroach:  s.encroach,
	}

	output := dbox2d.ShapeCast(&input)

	transform := dbox2d.Transform{
		P: dbox2d.MulAdd(s.transform.P, output.Fraction, input.TranslationB),
		Q: s.transform.Q,
	}

	distanceInput := dbox2d.DistanceInput{
		ProxyA:     s.proxyA,
		ProxyB:     s.proxyB,
		TransformA: dbox2d.TransformIdentity(),
		TransformB: transform,
		UseRadii:   false,
	}
	distanceCache := dbox2d.SimplexCache{}
	distanceOutput := dbox2d.ShapeDistance(&distanceInput, &distanceCache, nil)

	s.DrawTextLine("hit = %t, iterations = %d, fraction = %g, distance = %g", output.Hit, output.Iterations,
		ToFloat64(output.Fraction), ToFloat64(distanceOutput.Distance))

	draw := &s.Context.Draw

	s.shapes.drawShape(draw, s.typeA, dbox2d.TransformIdentity(), FromFloat64(s.radiusA), dbox2d.ColorCyan)
	s.shapes.drawShape(draw, s.typeB, s.transform, FromFloat64(s.radiusB), dbox2d.ColorLightGreen)
	transform2 := dbox2d.Transform{P: s.transform.P.Add(s.translation), Q: s.transform.Q}
	s.shapes.drawShape(draw, s.typeB, transform2, FromFloat64(s.radiusB), dbox2d.ColorIndianRed)

	if output.Hit {
		s.shapes.drawShape(draw, s.typeB, transform, FromFloat64(s.radiusB), dbox2d.ColorPlum)

		if output.Fraction.Greater(dbox2d.QZero()) {
			draw.DrawPoint(output.Point, dbox2d.QFromInt(5), dbox2d.ColorWhite)
			draw.DrawSegment(output.Point, dbox2d.MulAdd(output.Point, dbox2d.QHalf(), output.Normal), dbox2d.ColorYellow)
		} else {
			draw.DrawPoint(output.Point, dbox2d.QFromInt(5), dbox2d.ColorPeru)
		}
	}

	if s.showIndices {
		for i := 0; i < s.proxyA.Count; i++ {
			p := s.proxyA.Points[i]
			draw.DrawString(p, fmt.Sprintf(" %d", i), dbox2d.ColorWhite)
		}

		for i := 0; i < s.proxyB.Count; i++ {
			p := dbox2d.TransformPoint(s.transform, s.proxyB.Points[i])
			draw.DrawString(p, fmt.Sprintf(" %d", i), dbox2d.ColorWhite)
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

	verticesA [4]dbox2d.Vec2
	verticesB [2]dbox2d.Vec2

	radiusA dbox2d.Q
	radiusB dbox2d.Q
}

func NewTimeOfImpact(ctx *SampleContext) Sample {
	s := &TimeOfImpact{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: -16, Y: 45}
		ctx.Camera.Zoom = 5
	}

	s.verticesA = [4]dbox2d.Vec2{qv("-16.25", "44.75"), qv("-15.75", "44.75"), qv("-15.75", "45.25"), qv("-16.25", "45.25")}
	s.verticesB = [2]dbox2d.Vec2{qv("0", "-0.125"), qv("0", "0.125")}

	s.radiusA = dbox2d.QZero()
	s.radiusB = qs("0.0299999993")
	return s
}

func (s *TimeOfImpact) Step() {
	s.Base.Step()

	sweepA := dbox2d.Sweep{
		Q1: dbox2d.RotIdentity(),
		Q2: dbox2d.RotIdentity(),
	}
	sweepB := dbox2d.Sweep{
		C1: qv("-15.8332710", "45.3520279"),
		C2: qv("-15.8324337", "45.3413048"),
		Q1: dbox2d.Rot{Cos: qs("-0.540891349"), Sin: qs("0.841092527")},
		Q2: dbox2d.Rot{Cos: qs("-0.457797021"), Sin: qs("0.889056742")},
	}

	input := dbox2d.TOIInput{
		ProxyA:      dbox2d.MakeProxy(s.verticesA[:], s.radiusA),
		ProxyB:      dbox2d.MakeProxy(s.verticesB[:], s.radiusB),
		SweepA:      sweepA,
		SweepB:      sweepB,
		MaxFraction: dbox2d.QOne(),
	}

	output := dbox2d.TimeOfImpact(&input)

	s.DrawTextLine("toi = %g", ToFloat64(output.Fraction))

	draw := &s.Context.Draw
	countA := len(s.verticesA)
	countB := len(s.verticesB)

	var vertices [dbox2d.MaxPolygonVertices]dbox2d.Vec2

	// Draw A
	transformA := dbox2d.GetSweepTransform(&sweepA, dbox2d.QZero())
	for i := 0; i < countA; i++ {
		vertices[i] = dbox2d.TransformPoint(transformA, s.verticesA[i])
	}
	draw.DrawPolygon(vertices[:countA], dbox2d.ColorGray)

	// Draw B at t = 0
	transformB := dbox2d.GetSweepTransform(&sweepB, dbox2d.QZero())
	for i := 0; i < countB; i++ {
		vertices[i] = dbox2d.TransformPoint(transformB, s.verticesB[i])
	}
	draw.DrawSolidCapsule(vertices[0], vertices[1], s.radiusB, dbox2d.ColorGreen)

	// Draw B at t = hit_time
	transformB = dbox2d.GetSweepTransform(&sweepB, output.Fraction)
	for i := 0; i < countB; i++ {
		vertices[i] = dbox2d.TransformPoint(transformB, s.verticesB[i])
	}
	draw.DrawPolygon(vertices[:countB], dbox2d.ColorOrange)

	// Draw B at t = 1
	transformB = dbox2d.GetSweepTransform(&sweepB, dbox2d.QOne())
	for i := 0; i < countB; i++ {
		vertices[i] = dbox2d.TransformPoint(transformB, s.verticesB[i])
	}
	draw.DrawSolidCapsule(vertices[0], vertices[1], s.radiusB, dbox2d.ColorRed)

	if output.State == dbox2d.TOIStateHit {
		distanceInput := dbox2d.DistanceInput{
			ProxyA:     input.ProxyA,
			ProxyB:     input.ProxyB,
			TransformA: dbox2d.GetSweepTransform(&sweepA, output.Fraction),
			TransformB: dbox2d.GetSweepTransform(&sweepB, output.Fraction),
			UseRadii:   false,
		}
		cache := dbox2d.SimplexCache{}
		distanceOutput := dbox2d.ShapeDistance(&distanceInput, &cache, nil)
		s.DrawTextLine("distance = %g", ToFloat64(distanceOutput.Distance))
	}
}
