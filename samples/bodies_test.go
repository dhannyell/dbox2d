// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

package samples

import (
	"math"
	"testing"

	"github.com/dhannyell/dbox2d"
)

// TestPivotLowerEndStartsAtRest pins the radians-to-turns conversion of the
// initial angular velocity: the lower end of the lever must not move.
func TestPivotLowerEndStartsAtRest(t *testing.T) {
	s := NewPivot(NewSampleContext()).(*Pivot)
	defer s.Destroy()

	v := s.bodyId.GetLinearVelocity()
	omega := ToFloat64(s.bodyId.GetAngularVelocity()) * 2 * math.Pi
	r := s.bodyId.GetWorldVector(dbox2d.Vec2{Y: s.lever.Neg()})

	vx := ToFloat64(v.X) - omega*ToFloat64(r.Y)
	vy := ToFloat64(v.Y) + omega*ToFloat64(r.X)
	if math.Hypot(vx, vy) > 1e-3 {
		t.Fatalf("pivot velocity = (%g, %g), want (0, 0)", vx, vy)
	}
}

func TestBodyTypeSwitchesPlatform(t *testing.T) {
	s := NewBodyType(NewSampleContext()).(*BodyType)
	defer s.Destroy()

	s.setType(dbox2d.KinematicBody)
	if got := s.platformId.GetType(); got != dbox2d.KinematicBody {
		t.Fatalf("platform type = %v, want kinematic", got)
	}
	if got := ToFloat64(s.platformId.GetLinearVelocity().X); got != -3 {
		t.Fatalf("platform velocity x = %g, want -3", got)
	}
	for range 10 {
		s.Step()
	}

	s.setType(dbox2d.StaticBody)
	for _, id := range []dbox2d.BodyId{s.platformId, s.secondAttachmentId, s.secondPayloadId, s.touchingBodyId, s.floatingBodyId} {
		if got := id.GetType(); got != dbox2d.StaticBody {
			t.Fatalf("body type = %v, want static", got)
		}
	}
	s.Step()
}

// TestKinematicBodyReachesTarget checks that the body lands on the target
// set in the last step.
func TestKinematicBodyReachesTarget(t *testing.T) {
	s := NewKinematicBody(NewSampleContext()).(*KinematicBody)
	defer s.Destroy()

	var target float64
	for range 60 {
		target = s.time
		s.Step()
	}

	p := s.bodyId.GetPosition()
	wantX := 2 * s.amplitude * math.Cos(target)
	wantY := s.amplitude * math.Sin(2*target)
	if math.Abs(ToFloat64(p.X)-wantX) > 1e-2 || math.Abs(ToFloat64(p.Y)-wantY) > 1e-2 {
		t.Fatalf("position = (%g, %g), want (%g, %g)", ToFloat64(p.X), ToFloat64(p.Y), wantX, wantY)
	}
}
