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

const convexHullCount = dbox2d.MaxPolygonVertices

// ConvexHull computes the hull of random points clamped onto a square,
// which creates collinearities that stress the hull algorithm.
type ConvexHull struct {
	Base

	points     [convexHullCount]dbox2d.Vec2
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
	angle := dbox2d.QHalf().Mul(shared.RandomFloat())
	r := dbox2d.MakeRot(angle)

	lowerBound := qv("-4", "-4")
	upperBound := qv("4", "4")

	for i := range convexHullCount {
		x := dbox2d.QFromInt(10).Mul(shared.RandomFloat())
		y := dbox2d.QFromInt(10).Mul(shared.RandomFloat())

		// Clamp onto a square to help create collinearities.
		// This will stress the convex hull algorithm.
		v := dbox2d.Clamp(dbox2d.Vec2{X: x, Y: y}, lowerBound, upperBound)
		s.points[i] = dbox2d.RotateVector(r, v)
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

	var hull dbox2d.Hull
	valid := false

	if s.bulk {
		// defect hunting; the reference keeps a timing loop behind #else
		for range 10000 {
			s.generate()
			hull = dbox2d.ComputeHull(s.points[:s.count])
			if hull.Count == 0 {
				continue
			}

			valid = dbox2d.ValidateHull(&hull)
			if !valid || !s.bulk {
				s.bulk = false
				break
			}
		}
	} else {
		if s.auto {
			s.generate()
		}

		hull = dbox2d.ComputeHull(s.points[:s.count])
		if hull.Count > 0 {
			valid = dbox2d.ValidateHull(&hull)
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
	draw.DrawPolygon(hull.Points[:hull.Count], dbox2d.ColorGray)

	for i := range s.count {
		draw.DrawPoint(s.points[i], dbox2d.QFromInt(5), dbox2d.ColorBlue)
		draw.DrawString(s.points[i].Add(qv("0.1", "0.1")), fmt.Sprintf("%d", i), dbox2d.ColorWhite)
	}

	for i := range hull.Count {
		draw.DrawPoint(hull.Points[i], dbox2d.QFromInt(6), dbox2d.ColorGreen)
	}
}
