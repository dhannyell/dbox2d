// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

package draw_test

import (
	"strings"
	"testing"
	"unsafe"

	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/dbox2d/samples"
	"github.com/dhannyell/dbox2d/samples/internal/draw"
	"github.com/dhannyell/fixed"
)

// buildScene creates a small world with a box, a circle, a capsule and a
// revolute joint, close enough together that Draw sees every batch kind.
func buildScene(t *testing.T) dbox2d.WorldId {
	t.Helper()
	def := dbox2d.DefaultWorldDef()
	w := dbox2d.CreateWorld(&def)
	t.Cleanup(func() { dbox2d.DestroyWorld(w) })

	sd := dbox2d.DefaultShapeDef()
	bd := dbox2d.DefaultBodyDef()
	ground := dbox2d.CreateBody(w, &bd)
	segment := dbox2d.Segment{
		Point1: dbox2d.Vec2{X: fixed.Q32FromInt(-20)},
		Point2: dbox2d.Vec2{X: fixed.Q32FromInt(20)},
	}
	dbox2d.CreateSegmentShape(ground, &sd, &segment)

	bd = dbox2d.DefaultBodyDef()
	bd.Type = dbox2d.DynamicBody
	bd.Position = dbox2d.Vec2{X: fixed.Q32FromInt(-6), Y: fixed.Q32FromInt(3)}
	box := dbox2d.MakeSquare(fixed.Q32One())
	dbox2d.CreatePolygonShape(dbox2d.CreateBody(w, &bd), &sd, &box)

	bd = dbox2d.DefaultBodyDef()
	bd.Type = dbox2d.DynamicBody
	bd.Position = dbox2d.Vec2{X: fixed.Q32Zero(), Y: fixed.Q32FromInt(3)}
	circle := dbox2d.Circle{Radius: fixed.Q32Half()}
	dbox2d.CreateCircleShape(dbox2d.CreateBody(w, &bd), &sd, &circle)

	bd = dbox2d.DefaultBodyDef()
	bd.Type = dbox2d.DynamicBody
	bd.Position = dbox2d.Vec2{X: fixed.Q32FromInt(6), Y: fixed.Q32FromInt(3)}
	capsule := dbox2d.Capsule{
		Center1: dbox2d.Vec2{X: fixed.Q32Half().Neg()},
		Center2: dbox2d.Vec2{X: fixed.Q32Half()},
		Radius:  fixed.Q32MustParse("0.25"),
	}
	capsuleBody := dbox2d.CreateBody(w, &bd)
	dbox2d.CreateCapsuleShape(capsuleBody, &sd, &capsule)

	bd = dbox2d.DefaultBodyDef()
	bd.Type = dbox2d.DynamicBody
	bd.Position = dbox2d.Vec2{X: fixed.Q32FromInt(12), Y: fixed.Q32FromInt(3)}
	hinge := dbox2d.CreateBody(w, &bd)
	dbox2d.CreatePolygonShape(hinge, &sd, &box)
	jd := dbox2d.DefaultRevoluteJointDef()
	jd.BodyIdA, jd.BodyIdB = ground, hinge
	jd.LocalAnchorA = dbox2d.Vec2{X: fixed.Q32FromInt(12), Y: fixed.Q32FromInt(3)}
	dbox2d.CreateRevoluteJoint(w, &jd)

	return w
}

// TestBatchesMatchTheCallbackOrder pins that DebugDraw fans world callbacks
// into the batches, and that Draw's own methods append the same way.
func TestBatchesMatchTheCallbackOrder(t *testing.T) {
	w := buildScene(t)
	camera := samples.NewCamera()
	d := draw.New(&camera)
	dd := d.DebugDraw()
	dd.DrawShapes = true
	dd.DrawJoints = true

	w.Step(fixed.Q32FromRatio(1, 60), 4)
	w.Draw(&dd)

	if len(d.Batches.SolidPolygons) < 1 {
		t.Error("no solid polygons drawn")
	}
	if len(d.Batches.SolidCircles) < 1 {
		t.Error("no solid circles drawn")
	}
	if len(d.Batches.SolidCapsules) < 1 {
		t.Error("no solid capsules drawn")
	}
	if len(d.Batches.Lines) < 1 {
		t.Error("no lines drawn (joint anchors and axes draw as lines)")
	}

	// DrawPolygon closes the loop starting from the last vertex.
	before := len(d.Batches.Lines)
	triangle := []dbox2d.Vec2{
		{X: fixed.Q32Zero(), Y: fixed.Q32Zero()},
		{X: fixed.Q32One(), Y: fixed.Q32Zero()},
		{X: fixed.Q32Zero(), Y: fixed.Q32One()},
	}
	d.DrawPolygon(triangle, dbox2d.ColorWhite)
	added := d.Batches.Lines[before:]
	if len(added) != 3*2 {
		t.Fatalf("DrawPolygon appended %d line vertices, want 6", len(added))
	}
	// The first segment closes from the last vertex back to the first.
	if added[0].Position != (draw.Vec2f{X: 0, Y: 1}) || added[1].Position != (draw.Vec2f{}) {
		t.Errorf("first segment = %+v -> %+v, want the closing edge", added[0].Position, added[1].Position)
	}

	// DrawAABB appends exactly four lines (eight vertices).
	before = len(d.Batches.Lines)
	d.DrawAABB(dbox2d.AABB{UpperBound: dbox2d.Vec2{X: fixed.Q32One(), Y: fixed.Q32One()}}, dbox2d.ColorWhite)
	if got := len(d.Batches.Lines) - before; got != 4*2 {
		t.Fatalf("DrawAABB appended %d line vertices, want 8", got)
	}

	d.Batches.Reset()
	if len(d.Batches.SolidPolygons) != 0 || len(d.Batches.Lines) != 0 || len(d.Batches.SolidCircles) != 0 || len(d.Batches.SolidCapsules) != 0 {
		t.Error("Reset left a non-empty slice")
	}
	if cap(d.Batches.Lines) == 0 {
		t.Error("Reset dropped the backing array's capacity")
	}
}

// TestVertexLayoutsMatchTheReference pins the GPU-visible memory layout of
// every batch element against the C structs in draw.cpp.
func TestVertexLayoutsMatchTheReference(t *testing.T) {
	cases := []struct {
		name string
		size uintptr
	}{
		{"PointData", unsafe.Sizeof(draw.PointData{})},
		{"VertexData", unsafe.Sizeof(draw.VertexData{})},
		{"CircleData", unsafe.Sizeof(draw.CircleData{})},
		{"SolidCircleData", unsafe.Sizeof(draw.SolidCircleData{})},
		{"CapsuleData", unsafe.Sizeof(draw.CapsuleData{})},
		{"PolygonData", unsafe.Sizeof(draw.PolygonData{})},
		{"TextVertex", unsafe.Sizeof(draw.TextVertex{})},
	}
	want := map[string]uintptr{
		"PointData": 16, "VertexData": 12, "CircleData": 16,
		"SolidCircleData": 24, "CapsuleData": 28, "PolygonData": 92, "TextVertex": 20,
	}
	for _, c := range cases {
		if c.size != want[c.name] {
			t.Errorf("sizeof(%s) = %d, want %d", c.name, c.size, want[c.name])
		}
	}

	var p draw.PolygonData
	if got := unsafe.Offsetof(p.Count); got != 80 {
		t.Errorf("offsetof(PolygonData.Count) = %d, want 80", got)
	}
	if got := unsafe.Offsetof(p.Radius); got != 84 {
		t.Errorf("offsetof(PolygonData.Radius) = %d, want 84", got)
	}
	if got := unsafe.Offsetof(p.Color); got != 88 {
		t.Errorf("offsetof(PolygonData.Color) = %d, want 88", got)
	}
}

// TestShadersEmbedBothStages pins that every WGSL module compiles both
// pipeline stages under the names a host expects.
func TestShadersEmbedBothStages(t *testing.T) {
	shaders := map[string]string{
		"BackgroundWGSL":   draw.BackgroundWGSL,
		"CircleWGSL":       draw.CircleWGSL,
		"SolidCircleWGSL":  draw.SolidCircleWGSL,
		"SolidCapsuleWGSL": draw.SolidCapsuleWGSL,
		"SolidPolygonWGSL": draw.SolidPolygonWGSL,
		"LinesWGSL":        draw.LinesWGSL,
		"PointsWGSL":       draw.PointsWGSL,
		"TextWGSL":         draw.TextWGSL,
	}
	for name, src := range shaders {
		if !strings.Contains(src, "@vertex") || !strings.Contains(src, "vs_main") {
			t.Errorf("%s: missing a vertex stage", name)
		}
		if !strings.Contains(src, "@fragment") || !strings.Contains(src, "fs_main") {
			t.Errorf("%s: missing a fragment stage", name)
		}
	}
}

// TestAtlasMeasuresText pins the glyph metrics a text layer depends on.
func TestAtlasMeasuresText(t *testing.T) {
	atlas, err := draw.NewAtlas(14)
	if err != nil {
		t.Fatalf("NewAtlas: %v", err)
	}
	if got := atlas.TextWidth(""); got != 0 {
		t.Errorf("TextWidth(\"\") = %d, want 0", got)
	}
	a, b, ab := atlas.TextWidth("a"), atlas.TextWidth("b"), atlas.TextWidth("ab")
	if ab != a+b {
		t.Errorf("TextWidth(ab) = %d, want %d (a) + %d (b) = %d", ab, a, b, a+b)
	}
	if atlas.TextHeight() <= 0 {
		t.Error("TextHeight() <= 0")
	}

	var buf []draw.TextVertex
	buf = atlas.AppendQuads(buf, draw.TextItem{Text: "abc", Color: draw.RGBA8{A: 255}})
	if len(buf) != 3*6 {
		t.Fatalf("AppendQuads(3 chars) = %d vertices, want 18", len(buf))
	}
	for _, v := range buf {
		if v.U < 0 || v.U > 1 || v.V < 0 || v.V > 1 {
			t.Errorf("vertex uv out of range: %+v", v)
		}
	}
}
