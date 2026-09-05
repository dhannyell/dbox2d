// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/sample_stacking.cpp of Box2D v3.1.1

package samples

import (
	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/fixed"
)

func init() {
	RegisterSample("Stacking", "Single Box", NewSingleBox)
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

	extent := fixed.Q32One()

	bodyDef := dbox2d.DefaultBodyDef()
	groundId := dbox2d.CreateBody(s.WorldId, &bodyDef)

	groundWidth := fixed.Q32FromInt(66).Mul(extent)
	shapeDef := dbox2d.DefaultShapeDef()

	segment := dbox2d.Segment{
		Point1: dbox2d.Vec2{X: groundWidth.Neg(), Y: fixed.Q32Zero()},
		Point2: dbox2d.Vec2{X: groundWidth, Y: fixed.Q32Zero()},
	}
	dbox2d.CreateSegmentShape(groundId, &shapeDef, &segment)

	bodyDef.Type = dbox2d.DynamicBody
	box := dbox2d.MakeBox(extent, extent)
	bodyDef.Position = dbox2d.Vec2{X: fixed.Q32Zero(), Y: fixed.Q32One()}
	bodyDef.LinearVelocity = dbox2d.Vec2{X: fixed.Q32FromInt(5), Y: fixed.Q32Zero()}
	s.bodyId = dbox2d.CreateBody(s.WorldId, &bodyDef)
	dbox2d.CreatePolygonShape(s.bodyId, &shapeDef, &box)

	return s
}

// Step advances the scene and reports the box position.
func (s *SingleBox) Step() {
	s.Base.Step()

	position := s.bodyId.GetPosition()
	s.DrawTextLine("(x, y) = (%.2g, %.2g)", floatFromQ(position.X), floatFromQ(position.Y))
}
