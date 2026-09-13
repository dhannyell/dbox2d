// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/sample_geometry.cpp of Box2D v3.1.1

package samples

import (
	"fmt"

	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/dbox2d/internal/shared"
)

func init() {
	RegisterSample("Geometry", "Convex Hull", NewConvexHull)
}

const convexHullCount = b2.MaxPolygonVertices

// ConvexHull computes the hull of random points clamped onto a square,
// which creates collinearities that stress the hull algorithm.
type ConvexHull struct {
	Base

	points     [convexHullCount]b2.Vec2
	count      int
	generation int
	auto       bool
	bulk       bool
}

func NewConvexHull(ctx *SampleContext) Sample {
	s := &ConvexHull{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0.5, Y: 0}
		ctx.Camera.Zoom = 25 * 0.3
	}

	s.generation = 0
	s.auto = false
	s.bulk = false
	s.generate()
	return s
}

// generate is the active branch of Generate; the reference keeps two fixed
// point sets behind #if 0.
func (s *ConvexHull) generate() {
	// Turns; the reference angle is pi times a random float.
	angle := b2.QHalf().Mul(shared.RandomFloat())
	r := b2.MakeRot(angle)

	lowerBound := b2.V2(-4, -4)
	upperBound := b2.V2(4, 4)

	for i := range convexHullCount {
		x := b2.F(10).Mul(shared.RandomFloat())
		y := b2.F(10).Mul(shared.RandomFloat())

		// Clamp onto a square to help create collinearities.
		// This will stress the convex hull algorithm.
		v := b2.Clamp(b2.Vec2{X: x, Y: y}, lowerBound, upperBound)
		s.points[i] = b2.RotateVector(r, v)
	}

	s.count = convexHullCount

	s.generation += 1
}

func (s *ConvexHull) Keyboard(key Key) {
	switch key {
	case KeyA:
		s.auto = !s.auto

	case KeyB:
		s.bulk = !s.bulk

	case KeyG:
		s.generate()
	}
}

func (s *ConvexHull) Step() {
	s.Base.Step()

	s.DrawTextLine("Options: generate(g), auto(a), bulk(b)")

	var hull b2.Hull
	valid := false

	if s.bulk {
		// defect hunting; the reference keeps a timing loop behind #else
		for range 10000 {
			s.generate()
			hull = b2.ComputeHull(s.points[:s.count])
			if hull.Count == 0 {
				continue
			}

			valid = b2.ValidateHull(&hull)
			if !valid || !s.bulk {
				s.bulk = false
				break
			}
		}
	} else {
		if s.auto {
			s.generate()
		}

		hull = b2.ComputeHull(s.points[:s.count])
		if hull.Count > 0 {
			valid = b2.ValidateHull(&hull)
			if !valid {
				s.auto = false
			}
		}
	}

	if !valid {
		s.DrawTextLine("generation = %d, FAILED", s.generation)
	} else {
		s.DrawTextLine("generation = %d, count = %d", s.generation, hull.Count)
	}

	draw := &s.Context.Draw
	draw.DrawPolygon(hull.Points[:hull.Count], b2.ColorGray)

	for i := range s.count {
		draw.DrawPoint(s.points[i], b2.F(5), b2.ColorBlue)
		draw.DrawString(s.points[i].Add(b2.V2(0.1, 0.1)), fmt.Sprintf("%d", i), b2.ColorWhite)
	}

	for i := range hull.Count {
		draw.DrawPoint(hull.Points[i], b2.F(6), b2.ColorGreen)
	}
}
