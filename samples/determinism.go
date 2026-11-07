// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/sample_determinism.cpp of Box2D v3.2.0. The scene itself
// is shared/determinism.c, in internal/shared.

package samples

import (
	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/dbox2d/internal/shared"
)

func init() {
	RegisterSample("Determinism", "Falling Hinges", NewFallingHinges)
	RegisterSample("Determinism", "Rollback", NewRollback)
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

	bodyIds   []b2.BodyId
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
	rotation := func(radians float32) b2.Rot { return rotFromRadians(float64(radians)) }
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
				buf = appendScalarBytes(buf[:0], b2.ToVec2(xf.P).X)
				buf = appendScalarBytes(buf, b2.ToVec2(xf.P).Y)
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

		// Exercise the queries, as the reference does, so they take part in
		// the determinism check.
		filter := b2.DefaultQueryFilter()
		scanBox := b2.AABB{LowerBound: b2.V2(5, 1), UpperBound: b2.V2(7, 2.5)}
		s.WorldId.OverlapAABB(b2.ToPos(b2.Vec2{}), scanBox, filter, func(b2.ShapeId) bool { return true })

		origin := b2.V2(0, 12)
		translation := b2.V2(0, -14)

		if s.Base.StepCount == 30 {
			s.WorldId.CastRayClosest(b2.ToPos(origin), translation, filter)
		}

		if s.Base.StepCount < 5 || 100 < s.Base.StepCount {
			origin = b2.V2(-10, 2)
			translation = b2.V2(20, 0)
			s.WorldId.CastRay(b2.ToPos(origin), translation, filter, func(b2.ShapeId, b2.Pos, b2.Vec2, b2.Q) b2.Q { return b2.QOne() })
		}
	} else {
		s.DrawTextLine("sleep step = %d, hash = 0x%08x", s.sleepStep, s.hash)
	}
}

// rollbackSnapshot is one captured world image and the step it was taken at.
type rollbackSnapshot struct {
	image []byte
	step  int
}

type rollbackResult int

const (
	rollbackRunning rollbackResult = iota
	rollbackMatch
	rollbackMismatch
)

// rollbackInterval is the step count between snapshots.
const rollbackInterval = 20

// Rollback runs the falling hinges scene once, snapshotting the world every
// rollbackInterval steps, and records the sleep step and hash of that
// reference run. Roll Back restores a snapshot and steps again: the new run
// must reach the same sleep step and hash. It corresponds to the Rollback
// sample of Box2D v3.2.0.
type Rollback struct {
	Base

	bodyIds []b2.BodyId

	snapshots []rollbackSnapshot
	selected  int

	// The reference run, completed once and kept.
	referenceSleepStep int
	referenceHash      uint32

	stepCount int
	sleepStep int
	hash      uint32
	result    rollbackResult
	rolled    bool
	rolledTo  int
}

func NewRollback(ctx *SampleContext) Sample {
	s := &Rollback{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 7.5}
		ctx.Camera.Zoom = 10
	}

	rotation := func(radians float32) b2.Rot { return rotFromRadians(float64(radians)) }
	s.bodyIds = shared.CreateFallingHinges(s.WorldId, shared.Options{Rotation: rotation})
	s.sleepStep = -1
	s.referenceSleepStep = -1
	s.capture()
	return s
}

func (s *Rollback) capture() {
	image := make([]byte, s.WorldId.GetSnapshot(nil))
	s.WorldId.GetSnapshot(image)
	s.snapshots = append(s.snapshots, rollbackSnapshot{image: image, step: s.stepCount})
}

// updateHash takes the transform hash once nothing moves.
func (s *Rollback) updateHash() {
	if s.hash != 0 {
		return
	}
	if len(s.WorldId.GetBodyEvents().MoveEvents) != 0 {
		return
	}
	s.hash = hashInit
	var buf []byte
	for _, bodyId := range s.bodyIds {
		xf := bodyId.GetTransform()
		buf = appendScalarBytes(buf[:0], b2.ToVec2(xf.P).X)
		buf = appendScalarBytes(buf, b2.ToVec2(xf.P).Y)
		buf = appendScalarBytes(buf, xf.Q.Cos)
		buf = appendScalarBytes(buf, xf.Q.Sin)
		s.hash = hashBytes(s.hash, buf)
	}
	s.sleepStep = s.stepCount
}

func (s *Rollback) rollBack(index int) {
	snap := s.snapshots[index]
	if !s.WorldId.RestoreFromSnapshot(snap.image) {
		return
	}
	s.stepCount = snap.step
	s.sleepStep = -1
	s.hash = 0
	s.result = rollbackRunning
	s.rolled = true
	s.rolledTo = snap.step
}

func (s *Rollback) UpdateGui() {
	gui := s.Context.Gui
	if !gui.Begin("Rollback", 10, 300, 240, 120) {
		gui.End()
		return
	}
	if len(s.snapshots) > 0 {
		s.selected = min(s.selected, len(s.snapshots)-1)
		gui.SliderInt("Snapshot", &s.selected, 0, len(s.snapshots)-1)
	}
	if s.referenceHash != 0 && len(s.snapshots) > 0 && gui.Button("Roll Back") {
		s.rollBack(s.selected)
	}
	gui.End()
}

func (s *Rollback) Step() {
	settings := &s.Context.Settings
	running := !settings.Pause || settings.SingleStep

	// Snapshots are only taken while the reference run is in progress.
	if running && s.referenceHash == 0 && s.stepCount > 0 && s.stepCount%rollbackInterval == 0 &&
		s.snapshots[len(s.snapshots)-1].step != s.stepCount {
		s.capture()
	}

	s.Base.Step()

	if running {
		s.updateHash()
		s.stepCount++

		if s.hash != 0 {
			if s.referenceHash == 0 && !s.rolled {
				s.referenceSleepStep = s.sleepStep
				s.referenceHash = s.hash
			} else if s.rolled && s.result == rollbackRunning {
				if s.hash == s.referenceHash && s.sleepStep == s.referenceSleepStep {
					s.result = rollbackMatch
				} else {
					s.result = rollbackMismatch
				}
			}
		}
	}

	switch {
	case s.rolled:
		s.DrawTextLine("rolled back to step %d", s.rolledTo)
	default:
		s.DrawTextLine("reference run")
	}
	suffix := ""
	switch s.result {
	case rollbackMatch:
		suffix = " (match)"
	case rollbackMismatch:
		suffix = " (MISMATCH)"
	}
	s.DrawTextLine("sleep step = %d, hash = 0x%08X%s", s.sleepStep, s.hash, suffix)
}
