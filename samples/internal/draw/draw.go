// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/draw.h and samples/draw.cpp of Box2D v3.1.1

package draw

import (
	"math"

	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/dbox2d/samples"
)

// minCapsuleLength is the reference's early-out in GLSolidCapsules::AddCapsule.
const minCapsuleLength = 0.001

// f32 converts a simulation scalar to the presentation float32.
func f32(q dbox2d.Q) float32 { return float32(samples.ToFloat64(q)) }

// v2f converts a simulation vector to the presentation float32 pair.
func v2f(v dbox2d.Vec2) Vec2f { return Vec2f{X: f32(v.X), Y: f32(v.Y)} }

// Draw fills the batches. It is the Go form of the C++ Draw class without
// the GL objects.
type Draw struct {
	Camera  *samples.Camera
	Batches Batches
}

// New returns a Draw that reads the camera for text placement.
func New(camera *samples.Camera) *Draw {
	return &Draw{Camera: camera}
}

// DrawPolygon appends the polygon's edges as lines, closing the loop from
// the last vertex (draw.cpp Draw::DrawPolygon).
func (d *Draw) DrawPolygon(vertices []dbox2d.Vec2, color dbox2d.HexColor) {
	p1 := vertices[len(vertices)-1]
	for _, p2 := range vertices {
		d.DrawSegment(p1, p2, color)
		p1 = p2
	}
}

// DrawSolidPolygon appends one solid-polygon instance (draw.cpp
// Draw::DrawSolidPolygon / GLSolidPolygons::AddPolygon).
func (d *Draw) DrawSolidPolygon(transform dbox2d.Transform, vertices []dbox2d.Vec2, radius dbox2d.Q, color dbox2d.HexColor) {
	data := PolygonData{
		Transform: Transformf{P: v2f(transform.P), C: f32(transform.Q.Cos), S: f32(transform.Q.Sin)},
		Radius:    f32(radius),
		Color:     MakeRGBA8(color, 1.0),
	}
	n := min(len(vertices), 8)
	for i := range n {
		data.Points[i] = v2f(vertices[i])
	}
	data.Count = int32(n)
	d.Batches.SolidPolygons = append(d.Batches.SolidPolygons, data)
}

// DrawCircle appends one outline-circle instance.
func (d *Draw) DrawCircle(center dbox2d.Vec2, radius dbox2d.Q, color dbox2d.HexColor) {
	d.Batches.Circles = append(d.Batches.Circles, CircleData{
		Position: v2f(center),
		Radius:   f32(radius),
		Color:    MakeRGBA8(color, 1.0),
	})
}

// DrawSolidCircle appends one solid-circle instance. The centre is folded
// into the transform's position first, matching Draw::DrawSolidCircle.
func (d *Draw) DrawSolidCircle(transform dbox2d.Transform, center dbox2d.Vec2, radius dbox2d.Q, color dbox2d.HexColor) {
	transform.P = dbox2d.TransformPoint(transform, center)
	d.Batches.SolidCircles = append(d.Batches.SolidCircles, SolidCircleData{
		Transform: Transformf{P: v2f(transform.P), C: f32(transform.Q.Cos), S: f32(transform.Q.Sin)},
		Radius:    f32(radius),
		Color:     MakeRGBA8(color, 1.0),
	})
}

// DrawSolidCapsule appends one solid-capsule instance. The transform is
// derived from the two endpoints in float32, matching
// GLSolidCapsules::AddCapsule.
func (d *Draw) DrawSolidCapsule(p1, p2 dbox2d.Vec2, radius dbox2d.Q, color dbox2d.HexColor) {
	a, b := v2f(p1), v2f(p2)
	dx, dy := b.X-a.X, b.Y-a.Y
	l := hypot32(dx, dy)
	if l < minCapsuleLength {
		return
	}
	axisX, axisY := dx/l, dy/l
	mid := Vec2f{X: 0.5 * (a.X + b.X), Y: 0.5 * (a.Y + b.Y)}

	d.Batches.SolidCapsules = append(d.Batches.SolidCapsules, CapsuleData{
		Transform: Transformf{P: mid, C: axisX, S: axisY},
		Radius:    f32(radius),
		Length:    l,
		Color:     MakeRGBA8(color, 1.0),
	})
}

// hypot32 returns the float32 length of (x, y), matching b2Length.
func hypot32(x, y float32) float32 {
	return float32(math.Sqrt(float64(x)*float64(x) + float64(y)*float64(y)))
}

// DrawSegment appends one line segment.
func (d *Draw) DrawSegment(p1, p2 dbox2d.Vec2, color dbox2d.HexColor) {
	rgba := MakeRGBA8(color, 1.0)
	d.Batches.Lines = append(d.Batches.Lines,
		VertexData{Position: v2f(p1), Color: rgba},
		VertexData{Position: v2f(p2), Color: rgba},
	)
}

// axisScale is the length Draw::DrawTransform gives each axis line.
const axisScale = 0.2

// DrawTransform appends the red X axis and the green Y axis of transform.
func (d *Draw) DrawTransform(transform dbox2d.Transform) {
	p1 := transform.P
	x := dbox2d.RotGetXAxis(transform.Q)
	p2 := dbox2d.MulAdd(p1, samples.FromFloat64(axisScale), x)
	d.DrawSegment(p1, p2, dbox2d.ColorRed)

	y := dbox2d.RotGetYAxis(transform.Q)
	p2 = dbox2d.MulAdd(p1, samples.FromFloat64(axisScale), y)
	d.DrawSegment(p1, p2, dbox2d.ColorGreen)
}

// DrawPoint appends one point sprite. Unlike the reference, this port never
// doubles size for Apple hosts; a host does that itself if it needs to.
func (d *Draw) DrawPoint(p dbox2d.Vec2, size dbox2d.Q, color dbox2d.HexColor) {
	d.Batches.Points = append(d.Batches.Points, PointData{
		Position: v2f(p),
		Size:     f32(size),
		Color:    MakeRGBA8(color, 1.0),
	})
}

// drawStringColor and drawStringAtColor match the fixed colours the
// reference passes to ImGui::TextColoredV for the two DrawString overloads.
var (
	drawStringColor   = RGBA8{R: 230, G: 153, B: 153, A: 255}
	drawStringAtColor = RGBA8{R: 230, G: 230, B: 230, A: 255}
)

// DrawString appends one overlay line at a fixed screen position, in pixels.
func (d *Draw) DrawString(x, y int, s string) {
	d.Batches.Text = append(d.Batches.Text, TextItem{X: float32(x), Y: float32(y), Text: s, Color: drawStringColor})
}

// DrawStringAt appends one overlay line at a world position, converted to
// screen space through the camera.
func (d *Draw) DrawStringAt(p dbox2d.Vec2, s string) {
	ps := d.Camera.ConvertWorldToScreen(samples.Vec2f{X: samples.ToFloat64(p.X), Y: samples.ToFloat64(p.Y)})
	d.Batches.Text = append(d.Batches.Text, TextItem{X: float32(ps.X), Y: float32(ps.Y), Text: s, Color: drawStringAtColor})
}

// DrawAABB appends the box's four edges as lines.
func (d *Draw) DrawAABB(aabb dbox2d.AABB, color dbox2d.HexColor) {
	p1 := aabb.LowerBound
	p2 := dbox2d.Vec2{X: aabb.UpperBound.X, Y: aabb.LowerBound.Y}
	p3 := aabb.UpperBound
	p4 := dbox2d.Vec2{X: aabb.LowerBound.X, Y: aabb.UpperBound.Y}

	d.DrawSegment(p1, p2, color)
	d.DrawSegment(p2, p3, color)
	d.DrawSegment(p3, p4, color)
	d.DrawSegment(p4, p1, color)
}

// DebugDraw returns callbacks that write into d, with the flags
// DefaultDebugDraw starts with.
func (d *Draw) DebugDraw() dbox2d.DebugDraw {
	dd := dbox2d.DefaultDebugDraw()
	dd.DrawPolygon = d.DrawPolygon
	dd.DrawSolidPolygon = d.DrawSolidPolygon
	dd.DrawCircle = d.DrawCircle
	dd.DrawSolidCircle = func(transform dbox2d.Transform, radius dbox2d.Q, color dbox2d.HexColor) {
		d.DrawSolidCircle(transform, dbox2d.Vec2{}, radius, color)
	}
	dd.DrawSolidCapsule = d.DrawSolidCapsule
	dd.DrawSegment = d.DrawSegment
	dd.DrawTransform = d.DrawTransform
	dd.DrawPoint = d.DrawPoint
	dd.DrawString = func(p dbox2d.Vec2, s string, _ dbox2d.HexColor) { d.DrawStringAt(p, s) }
	return dd
}
