// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/sample_determinism.cpp and shared/determinism.c of Box2D v3.1.1

package samples

import (
	"github.com/dhannyell/dbox2d"
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

	s.createFallingHinges()
	s.done = false
	return s
}

func (s *FallingHinges) createFallingHinges() {
	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Position = qv("0", "-1")
		groundId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		box := dbox2d.MakeBox(dbox2d.QFromInt(20), dbox2d.QOne())
		shapeDef := dbox2d.DefaultShapeDef()
		dbox2d.CreatePolygonShape(groundId, &shapeDef, &box)
	}

	columnCount := 4
	rowCount := 30
	bodyCount := rowCount * columnCount

	s.bodyIds = make([]dbox2d.BodyId, bodyCount)

	h := qs("0.25")
	r := qs("0.1").Mul(h)
	box := dbox2d.MakeRoundedBox(h.Sub(r), h.Sub(r), r)

	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Material.Friction = qs("0.3")

	offset := qs("0.4").Mul(h)
	dx := dbox2d.QFromInt(10).Mul(h)
	xroot := dbox2d.QHalf().Neg().Mul(dx).Mul(dbox2d.QFromInt(columnCount - 1))

	jointDef := dbox2d.DefaultRevoluteJointDef()
	jointDef.EnableLimit = true
	// Turns; the reference limits are -0.1 pi and 0.2 pi.
	jointDef.LowerAngle = dbox2d.QFromRatio(-1, 20)
	jointDef.UpperAngle = dbox2d.QFromRatio(1, 10)
	jointDef.EnableSpring = true
	jointDef.Hertz = dbox2d.QHalf()
	jointDef.DampingRatio = dbox2d.QHalf()
	jointDef.LocalAnchorA = dbox2d.Vec2{X: h, Y: h}
	jointDef.LocalAnchorB = dbox2d.Vec2{X: offset, Y: h.Neg()}
	jointDef.DrawSize = qs("0.1")

	bodyIndex := 0

	for j := range columnCount {
		x := xroot.Add(dbox2d.QFromInt(j).Mul(dx))

		prevBodyId := dbox2d.BodyId{}

		for i := range rowCount {
			bodyDef := dbox2d.DefaultBodyDef()
			bodyDef.Type = dbox2d.DynamicBody

			bodyDef.Position.X = x.Add(offset.Mul(dbox2d.QFromInt(i)))
			bodyDef.Position.Y = h.Add(dbox2d.QFromInt(2).Mul(h).Mul(dbox2d.QFromInt(i)))

			// this tests the deterministic cosine and sine functions
			// The angle is the float32 radians of the reference, rounded
			// after each operation, then converted to turns.
			radians := float32(float32(0.1)*float32(i)) - 1
			bodyDef.Rotation = dbox2d.MakeRot(radiansToTurns(float64(radians)))

			bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)

			if i&1 == 0 {
				prevBodyId = bodyId
			} else {
				jointDef.BodyIdA = prevBodyId
				jointDef.BodyIdB = bodyId
				dbox2d.CreateRevoluteJoint(s.WorldId, &jointDef)
				prevBodyId = dbox2d.BodyId{}
			}

			dbox2d.CreatePolygonShape(bodyId, &shapeDef, &box)

			s.bodyIds[bodyIndex] = bodyId

			bodyIndex += 1
		}
	}

	s.stepCount = 0
	s.sleepStep = -1
	s.hash = 0
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
