// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/draw.cpp of Box2D v3.1.1

package draw

import "github.com/dhannyell/dbox2d"

// RGBA8 is one colour as the GPU reads it: four unorm bytes.
type RGBA8 struct{ R, G, B, A uint8 }

// MakeRGBA8 unpacks a HexColor into the GPU's four-byte form (draw.cpp:39).
func MakeRGBA8(c dbox2d.HexColor, alpha float32) RGBA8 {
	return RGBA8{
		R: uint8((c >> 16) & 0xFF),
		G: uint8((c >> 8) & 0xFF),
		B: uint8(c & 0xFF),
		A: uint8(0xFF * alpha),
	}
}

// Vec2f is a float32 pair with the C layout of a GPU vertex attribute.
type Vec2f struct{ X, Y float32 }

// Transformf mirrors b2Transform{p, q{c,s}} in float32.
type Transformf struct {
	P    Vec2f
	C, S float32
}

// PointData is one instance of GLPoints::PointData (16 bytes).
type PointData struct {
	Position Vec2f
	Size     float32
	Color    RGBA8
}

// VertexData is one vertex of GLLines::VertexData (12 bytes).
type VertexData struct {
	Position Vec2f
	Color    RGBA8
}

// CircleData is one instance of GLCircles::CircleData (16 bytes).
type CircleData struct {
	Position Vec2f
	Radius   float32
	Color    RGBA8
}

// SolidCircleData is one instance of GLSolidCircles::SolidCircleData (24 bytes).
type SolidCircleData struct {
	Transform Transformf
	Radius    float32
	Color     RGBA8
}

// CapsuleData is one instance of GLSolidCapsules::CapsuleData (28 bytes).
type CapsuleData struct {
	Transform Transformf
	Radius    float32
	Length    float32
	Color     RGBA8
}

// PolygonData is one instance of GLSolidPolygons::PolygonData (92 bytes).
type PolygonData struct {
	Transform Transformf
	Points    [8]Vec2f
	Count     int32
	Radius    float32
	Color     RGBA8
}

// TextItem is one string to draw at a screen position, in pixels.
type TextItem struct {
	X, Y  float32
	Text  string
	Color RGBA8
}

// QuadVertices is the unit quad every instanced batch shares as vertex
// buffer 0; the 1.1 extent leaves room for the border (draw.cpp, a = 1.1f).
var QuadVertices = [6]Vec2f{
	{X: -1.1, Y: -1.1}, {X: 1.1, Y: -1.1}, {X: -1.1, Y: 1.1},
	{X: 1.1, Y: -1.1}, {X: 1.1, Y: 1.1}, {X: -1.1, Y: 1.1},
}

// Batch sizes mirror e_batchSize per batch in draw.cpp.
const (
	PointBatchSize        = 2048
	LineBatchSize         = 2 * 2048
	CircleBatchSize       = 2048
	SolidCircleBatchSize  = 2048
	SolidCapsuleBatchSize = 2048
	SolidPolygonBatchSize = 512
)

// Batches holds everything one frame draws, in the order the callbacks
// arrived.
type Batches struct {
	Points        []PointData
	Lines         []VertexData
	Circles       []CircleData
	SolidCircles  []SolidCircleData
	SolidCapsules []CapsuleData
	SolidPolygons []PolygonData
	Text          []TextItem
}

// Reset empties every slice but keeps its capacity, so a host reuses the
// same backing arrays frame after frame.
func (b *Batches) Reset() {
	b.Points = b.Points[:0]
	b.Lines = b.Lines[:0]
	b.Circles = b.Circles[:0]
	b.SolidCircles = b.SolidCircles[:0]
	b.SolidCapsules = b.SolidCapsules[:0]
	b.SolidPolygons = b.SolidPolygons[:0]
	b.Text = b.Text[:0]
}

// Kind names one of the batches Flush draws.
type Kind int

// The seven batch kinds, text last because it is not part of Draw::Flush.
const (
	KindSolidCircles Kind = iota
	KindSolidCapsules
	KindSolidPolygons
	KindCircles
	KindLines
	KindPoints
	KindText
)

// FlushOrder is the order the reference draws the batches in: solid
// circles, solid capsules, solid polygons, circles, lines, points
// (draw.cpp Draw::Flush). Text is drawn last, after Flush, by the host.
var FlushOrder = [...]Kind{
	KindSolidCircles,
	KindSolidCapsules,
	KindSolidPolygons,
	KindCircles,
	KindLines,
	KindPoints,
	KindText,
}

// ZBias is the depth bias Flush uses for the batch's projection matrix:
// points 0, lines 0.1, everything else 0.2.
func (k Kind) ZBias() float32 {
	switch k {
	case KindPoints:
		return 0.0
	case KindLines:
		return 0.1
	default:
		return 0.2
	}
}
