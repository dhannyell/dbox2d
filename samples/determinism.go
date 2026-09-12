// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/sample_determinism.cpp of Box2D v3.1.1. The scene itself
// is shared/determinism.c, in internal/shared.

package samples

import (
	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/dbox2d/internal/shared"
)

func init() {
	RegisterSample("Determinism", "Falling Hinges", NewFallingHinges)
}

// hashInit and hashBytes are B2_HASH_INIT and b2Hash, the djb2 hash of the
// reference determinism test.
const hashInit uint32 = 5381

func hashBytes(hash uint32, data []byte) uint32 {
	result := hash
	for _, b := range data {
		result = (result << 5) + result + uint32(b)
	}
	return result
}

// FallingHinges is a visual form of the cross platform determinism test.
// The scenario is designed to produce a chaotic result engaging:
// - continuous collision
// - joint limits (approximate atan2)
// - MakeRot (approximate sin/cos)
// Once all the bodies go to sleep the step counter and transform hash is
// emitted. The hash reads the scalar bits, so it differs between the fixed
// and the float build.
type FallingHinges struct {
	Base

	bodyIds   []dbox2d.BodyId
	stepCount int
	sleepStep int
	hash      uint32
	done      bool
}

func NewFallingHinges(ctx *SampleContext) Sample {
	s := &FallingHinges{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 7.5}
		ctx.Camera.Zoom = 10
	}

	// b2MakeRot through the public API: the angle is the float32 radian value
	// of the reference, rounded after each operation, then converted to turns.
	rotation := func(radians float32) dbox2d.Rot { return rotFromRadians(float64(radians)) }
	s.bodyIds = shared.CreateFallingHinges(s.WorldId, shared.Options{Rotation: rotation})
	s.stepCount = 0
	s.sleepStep = -1
	s.hash = 0
	s.done = false
	return s
}

// updateFallingHinges is UpdateFallingHinges. It reports true once every
// body sleeps and the hash is taken.
func (s *FallingHinges) updateFallingHinges() bool {
	if s.hash == 0 {
		bodyEvents := s.WorldId.GetBodyEvents()

		if len(bodyEvents.MoveEvents) == 0 {
			if s.WorldId.GetAwakeBodyCount() != 0 {
				panic("samples: falling hinges has awake bodies without move events")
			}

			s.hash = hashInit
			var buf []byte
			for _, bodyId := range s.bodyIds {
				xf := bodyId.GetTransform()
				buf = appendScalarBytes(buf[:0], xf.P.X)
				buf = appendScalarBytes(buf, xf.P.Y)
				buf = appendScalarBytes(buf, xf.Q.Cos)
				buf = appendScalarBytes(buf, xf.Q.Sin)
				s.hash = hashBytes(s.hash, buf)
			}

			s.sleepStep = s.stepCount
		}
	}

	s.stepCount += 1

	return s.hash != 0
}

func (s *FallingHinges) Step() {
	s.Base.Step()

	if !s.Context.Settings.Pause && !s.done {
		s.done = s.updateFallingHinges()
	} else {
		s.DrawTextLine("sleep step = %d, hash = 0x%08x", s.sleepStep, s.hash)
	}
}
