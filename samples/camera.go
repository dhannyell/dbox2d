// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/draw.h and samples/draw.cpp (Camera only) of Box2D v3.1.1

package samples

import (
	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/fixed"
)

// q32FractionBits is the fractional width of dbox2d.Q (Q32.32); it converts
// between the simulation's fixed point and the camera's float64 math.
const q32FractionBits = 32

// FromFloat64 and ToFloat64 are presentation helpers; the simulation never
// calls them.

// FromFloat64 converts a float64 to the simulation's fixed-point type.
func FromFloat64(f float64) dbox2d.Q {
	return fixed.Q32FromRaw(int64(f * (1 << q32FractionBits)))
}

// ToFloat64 converts the simulation's fixed-point type to a float64.
func ToFloat64(q dbox2d.Q) float64 {
	return float64(q.Raw()) / (1 << q32FractionBits)
}

// Vec2f is a screen- or camera-space vector. Presentation math stays in
// float64; the simulation never reads it.
type Vec2f struct {
	X, Y float64
}

// Camera converts between screen pixels and world coordinates. It carries no
// simulation state.
type Camera struct {
	Center Vec2f
	Zoom   float64
	Width  int
	Height int
}

// NewCamera returns a camera with the reference's startup view.
func NewCamera() Camera {
	c := Camera{Width: 1920, Height: 1080}
	c.ResetView()
	return c
}

// ResetView restores the default center and zoom.
func (c *Camera) ResetView() {
	c.Center = Vec2f{X: 0, Y: 20}
	c.Zoom = 1
}

// ConvertScreenToWorld maps a screen pixel to a world point.
func (c *Camera) ConvertScreenToWorld(ps Vec2f) Vec2f {
	w := float64(c.Width)
	h := float64(c.Height)
	u := ps.X / w
	v := (h - ps.Y) / h

	ratio := w / h
	ex, ey := c.Zoom*ratio, c.Zoom

	lowerX, lowerY := c.Center.X-ex, c.Center.Y-ey
	upperX, upperY := c.Center.X+ex, c.Center.Y+ey

	return Vec2f{
		X: (1-u)*lowerX + u*upperX,
		Y: (1-v)*lowerY + v*upperY,
	}
}

// ConvertWorldToScreen maps a world point to a screen pixel.
func (c *Camera) ConvertWorldToScreen(pw Vec2f) Vec2f {
	w := float64(c.Width)
	h := float64(c.Height)
	ratio := w / h
	ex, ey := c.Zoom*ratio, c.Zoom

	lowerX, lowerY := c.Center.X-ex, c.Center.Y-ey
	upperX, upperY := c.Center.X+ex, c.Center.Y+ey

	u := (pw.X - lowerX) / (upperX - lowerX)
	v := (pw.Y - lowerY) / (upperY - lowerY)

	return Vec2f{X: u * w, Y: (1 - v) * h}
}

// GetViewBounds returns the world-space box the camera currently frames. It
// only gates what Draw draws; it carries no simulation meaning.
func (c *Camera) GetViewBounds() dbox2d.AABB {
	lower := c.ConvertScreenToWorld(Vec2f{X: 0, Y: float64(c.Height)})
	upper := c.ConvertScreenToWorld(Vec2f{X: float64(c.Width), Y: 0})
	return dbox2d.AABB{
		LowerBound: dbox2d.Vec2{X: FromFloat64(lower.X), Y: FromFloat64(lower.Y)},
		UpperBound: dbox2d.Vec2{X: FromFloat64(upper.X), Y: FromFloat64(upper.Y)},
	}
}

// BuildProjectionMatrix returns the reference's column-major orthographic
// projection for the current view (draw.cpp Camera::BuildProjectionMatrix).
// zBias becomes clip z as is; the hosts render without a depth attachment
// and rely on the flush order, like the reference.
func (c *Camera) BuildProjectionMatrix(zBias float32) [16]float32 {
	ratio := float32(c.Width) / float32(c.Height)
	zoom := float32(c.Zoom)
	ex, ey := zoom*ratio, zoom
	cx, cy := float32(c.Center.X), float32(c.Center.Y)

	lowerX, lowerY := cx-ex, cy-ey
	upperX, upperY := cx+ex, cy+ey
	w := upperX - lowerX
	h := upperY - lowerY

	var m [16]float32
	m[0] = 2.0 / w
	m[5] = 2.0 / h
	m[10] = -1.0
	m[12] = -2.0 * cx / w
	m[13] = -2.0 * cy / h
	m[14] = zBias
	m[15] = 1.0
	return m
}

// PixelScale is the pixelScale uniform of the reference shaders:
// height / zoom, which is twice the pixels per world unit.
func (c *Camera) PixelScale() float32 {
	return float32(c.Height) / float32(c.Zoom)
}
