// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/sample_benchmark.cpp (BenchmarkTumbler) and
// shared/benchmarks.c (CreateTumbler) of Box2D v3.1.1

package samples

import (
	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/fixed"
)

func init() {
	RegisterSample("Benchmark", "Tumbler", NewTumbler)
}

// tumblerGridCount is the reference's non-debug gridCount.
const tumblerGridCount = 45

// Tumbler spins a hollow box full of small boxes with a motorized revolute
// joint, a stress scene for the broad and narrow phase alike.
type Tumbler struct {
	Base
}

// NewTumbler builds the scene, matching CreateTumbler.
func NewTumbler(ctx *SampleContext) Sample {
	s := &Tumbler{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 1.5, Y: 10}
		ctx.Camera.Zoom = 25 * 0.6
	}

	bodyDef := dbox2d.DefaultBodyDef()
	groundId := dbox2d.CreateBody(s.WorldId, &bodyDef)

	bodyDef.Type = dbox2d.DynamicBody
	bodyDef.Position = dbox2d.Vec2{X: fixed.Q32Zero(), Y: fixed.Q32FromInt(10)}
	bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)

	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Density = fixed.Q32FromInt(50)

	ten, half := fixed.Q32FromInt(10), fixed.Q32Half()
	walls := []struct{ hw, hh, cx, cy fixed.Q32 }{
		{half, ten, ten, fixed.Q32Zero()},
		{half, ten, ten.Neg(), fixed.Q32Zero()},
		{ten, half, fixed.Q32Zero(), ten},
		{ten, half, fixed.Q32Zero(), ten.Neg()},
	}
	for _, w := range walls {
		box := dbox2d.MakeOffsetBox(w.hw, w.hh, dbox2d.Vec2{X: w.cx, Y: w.cy}, dbox2d.RotIdentity())
		dbox2d.CreatePolygonShape(bodyId, &shapeDef, &box)
	}

	// The reference measures angular speed in radians per second; this
	// port's joint fields are in turns, so (pi/180)*25 rad/s becomes
	// 25/360 turns/s.
	motorSpeed := fixed.Q32FromRatio(25, 360)

	jd := dbox2d.DefaultRevoluteJointDef()
	jd.BodyIdA = groundId
	jd.BodyIdB = bodyId
	jd.LocalAnchorA = dbox2d.Vec2{X: fixed.Q32Zero(), Y: fixed.Q32FromInt(10)}
	jd.MotorSpeed = motorSpeed
	jd.MaxMotorTorque = fixed.Q32FromInt(100_000_000)
	jd.EnableMotor = true
	dbox2d.CreateRevoluteJoint(s.WorldId, &jd)

	gridBox := dbox2d.MakeBox(fixed.Q32MustParse("0.125"), fixed.Q32MustParse("0.125"))
	step := fixed.Q32FromRatio(4, 10)
	start := fixed.Q32FromRatio(-2*tumblerGridCount, 10) // -0.2 * gridCount

	gridBodyDef := dbox2d.DefaultBodyDef()
	gridBodyDef.Type = dbox2d.DynamicBody
	gridShapeDef := dbox2d.DefaultShapeDef()

	y := start.Add(fixed.Q32FromInt(10))
	for range tumblerGridCount {
		x := start
		for range tumblerGridCount {
			gridBodyDef.Position = dbox2d.Vec2{X: x, Y: y}
			gridBodyId := dbox2d.CreateBody(s.WorldId, &gridBodyDef)
			dbox2d.CreatePolygonShape(gridBodyId, &gridShapeDef, &gridBox)
			x = x.Add(step)
		}
		y = y.Add(step)
	}

	return s
}
