// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/sample_robustness.cpp of Box2D v3.1.1

package samples

import (
	"github.com/dhannyell/dbox2d"
)

func init() {
	RegisterSample("Robustness", "HighMassRatio1", NewHighMassRatio1)
	RegisterSample("Robustness", "HighMassRatio2", NewHighMassRatio2)
	RegisterSample("Robustness", "HighMassRatio3", NewHighMassRatio3)
	RegisterSample("Robustness", "Overlap Recovery", NewOverlapRecovery)
	RegisterSample("Robustness", "Tiny Pyramid", NewTinyPyramid)
	RegisterSample("Robustness", "Cart", NewCart)
}

// createRobustnessGround is the wide ground box shared by the high mass
// ratio samples.
func createRobustnessGround(worldId b2.WorldId) {
	bodyDef := b2.DefaultBodyDef()
	groundId := b2.CreateBody(worldId, &bodyDef)
	shapeDef := b2.DefaultShapeDef()
	box := b2.MakeOffsetBox(b2.F(50), b2.QOne(), b2.V2(0, -1), b2.RotIdentity())
	b2.CreatePolygonShape(groundId, &shapeDef, &box)
}

// HighMassRatio1 is a pyramid with a heavy box on top.
type HighMassRatio1 struct {
	Base
}

func NewHighMassRatio1(ctx *SampleContext) Sample {
	s := &HighMassRatio1{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 3, Y: 14}
		ctx.Camera.Zoom = 25
	}

	extent := b2.QOne()

	createRobustnessGround(s.WorldId)

	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		box := b2.MakeBox(extent, extent)
		shapeDef := b2.DefaultShapeDef()

		for j := range 3 {
			count := 10
			// -20 + 2 * (count + 1) * j, in units of extent
			offset := b2.QFromInt(-20 + 2*(count+1)*j).Mul(extent)
			y := extent
			for count > 0 {
				for i := range count {
					// 2 * (i - count / 2), in units of extent
					coeff := b2.F(2*i - count)

					yy := y
					if count == 1 {
						yy = y.Add(b2.F(2))
					}
					bodyDef.Position = b2.Vec2{X: coeff.Mul(extent).Add(offset), Y: yy}
					bodyId := b2.CreateBody(s.WorldId, &bodyDef)

					shapeDef.Density = b2.QOne()
					if count == 1 {
						shapeDef.Density = b2.QFromInt((j + 1) * 100)
					}
					b2.CreatePolygonShape(bodyId, &shapeDef, &box)
				}

				count--
				y = y.Add(b2.F(2).Mul(extent))
			}
		}
	}

	return s
}

// HighMassRatio2 is a big box on small boxes.
type HighMassRatio2 struct {
	Base
}

func NewHighMassRatio2(ctx *SampleContext) Sample {
	s := &HighMassRatio2{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 16.5}
		ctx.Camera.Zoom = 25
	}

	createRobustnessGround(s.WorldId)

	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		shapeDef := b2.DefaultShapeDef()

		extent := b2.QOne()
		smallBox := b2.MakeBox(b2.QHalf().Mul(extent), b2.QHalf().Mul(extent))
		bigBox := b2.MakeBox(b2.F(10).Mul(extent), b2.F(10).Mul(extent))

		{
			bodyDef.Position = b2.Vec2{X: b2.F(-9).Mul(extent), Y: b2.QHalf().Mul(extent)}
			bodyId := b2.CreateBody(s.WorldId, &bodyDef)
			b2.CreatePolygonShape(bodyId, &shapeDef, &smallBox)
		}

		{
			bodyDef.Position = b2.Vec2{X: b2.F(9).Mul(extent), Y: b2.QHalf().Mul(extent)}
			bodyId := b2.CreateBody(s.WorldId, &bodyDef)
			b2.CreatePolygonShape(bodyId, &shapeDef, &smallBox)
		}

		{
			bodyDef.Position = b2.Vec2{Y: b2.F(10 + 16).Mul(extent)}
			bodyId := b2.CreateBody(s.WorldId, &bodyDef)
			b2.CreatePolygonShape(bodyId, &shapeDef, &bigBox)
		}
	}

	return s
}

// HighMassRatio3 is a big box on small triangles.
type HighMassRatio3 struct {
	Base
}

func NewHighMassRatio3(ctx *SampleContext) Sample {
	s := &HighMassRatio3{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 16.5}
		ctx.Camera.Zoom = 25
	}

	createRobustnessGround(s.WorldId)

	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		shapeDef := b2.DefaultShapeDef()

		extent := b2.QOne()
		half := b2.QHalf().Mul(extent)
		points := []b2.Vec2{{X: half.Neg()}, {X: half}, {Y: extent}}
		hull := b2.ComputeHull(points)
		smallTriangle := b2.MakePolygon(&hull, b2.QZero())
		bigBox := b2.MakeBox(b2.F(10).Mul(extent), b2.F(10).Mul(extent))

		{
			bodyDef.Position = b2.Vec2{X: b2.F(-9).Mul(extent), Y: half}
			bodyId := b2.CreateBody(s.WorldId, &bodyDef)
			b2.CreatePolygonShape(bodyId, &shapeDef, &smallTriangle)
		}

		{
			bodyDef.Position = b2.Vec2{X: b2.F(9).Mul(extent), Y: half}
			bodyId := b2.CreateBody(s.WorldId, &bodyDef)
			b2.CreatePolygonShape(bodyId, &shapeDef, &smallTriangle)
		}

		{
			bodyDef.Position = b2.Vec2{Y: b2.F(10 + 4).Mul(extent)}
			bodyId := b2.CreateBody(s.WorldId, &bodyDef)
			b2.CreatePolygonShape(bodyId, &shapeDef, &bigBox)
		}
	}

	return s
}

// OverlapRecovery starts a pyramid of overlapping boxes and lets the
// contact push-out separate them.
type OverlapRecovery struct {
	Base

	bodyIds      []b2.BodyId
	baseCount    int
	overlap      float64
	extent       float64
	pushOut      float64
	hertz        float64
	dampingRatio float64
}

func NewOverlapRecovery(ctx *SampleContext) Sample {
	s := &OverlapRecovery{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 2.5}
		ctx.Camera.Zoom = 3.75
	}

	s.baseCount = 4
	s.overlap = 0.25
	s.extent = 0.5
	s.pushOut = 3
	s.hertz = 30
	s.dampingRatio = 10

	bodyDef := b2.DefaultBodyDef()
	groundId := b2.CreateBody(s.WorldId, &bodyDef)

	groundWidth := b2.F(40)
	shapeDef := b2.DefaultShapeDef()
	shapeDef.Density = b2.QOne()

	segment := b2.Segment{Point1: b2.Vec2{X: groundWidth.Neg()}, Point2: b2.Vec2{X: groundWidth}}
	b2.CreateSegmentShape(groundId, &shapeDef, &segment)

	s.createScene()
	return s
}

func (s *OverlapRecovery) createScene() {
	for _, bodyId := range s.bodyIds {
		b2.DestroyBody(bodyId)
	}

	s.WorldId.SetContactTuning(FromFloat64(s.hertz), FromFloat64(s.dampingRatio), FromFloat64(s.pushOut))

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody

	extent := FromFloat64(s.extent)
	box := b2.MakeBox(extent, extent)
	shapeDef := b2.DefaultShapeDef()
	shapeDef.Density = b2.QOne()

	bodyCount := s.baseCount * (s.baseCount + 1) / 2
	s.bodyIds = make([]b2.BodyId, 0, bodyCount)

	fraction := b2.QOne().Sub(FromFloat64(s.overlap))
	step := b2.F(2).Mul(fraction).Mul(extent)
	y := extent
	for i := range s.baseCount {
		x := fraction.Mul(extent).Mul(b2.F(i - s.baseCount))
		for j := i; j < s.baseCount; j++ {
			bodyDef.Position = b2.Vec2{X: x, Y: y}
			bodyId := b2.CreateBody(s.WorldId, &bodyDef)

			b2.CreatePolygonShape(bodyId, &shapeDef, &box)

			s.bodyIds = append(s.bodyIds, bodyId)

			x = x.Add(step)
		}

		y = y.Add(step)
	}

	sampleAssert(len(s.bodyIds) == bodyCount, "overlap recovery body count")
}

func (s *OverlapRecovery) UpdateGui() {
	height := 210
	gui := s.Context.Gui
	gui.Begin("Overlap Recovery", 10, s.Context.Camera.Height-height-50, 220, height)

	changed := false
	changed = changed || gui.SliderFloat("Extent", &s.extent, 0.1, 1)
	changed = changed || gui.SliderInt("Base Count", &s.baseCount, 1, 10)
	changed = changed || gui.SliderFloat("Overlap", &s.overlap, 0, 1)
	changed = changed || gui.SliderFloat("Speed", &s.pushOut, 0, 10)
	changed = changed || gui.SliderFloat("Hertz", &s.hertz, 0, 240)
	changed = changed || gui.SliderFloat("Damping Ratio", &s.dampingRatio, 0, 20)
	changed = changed || gui.Button("Reset Scene")

	if changed {
		s.createScene()
	}

	gui.End()
}

// TinyPyramid is a pyramid of 5cm boxes.
type TinyPyramid struct {
	Base

	extent b2.Q
}

func NewTinyPyramid(ctx *SampleContext) Sample {
	s := &TinyPyramid{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 0.8}
		ctx.Camera.Zoom = 1
	}

	{
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(s.WorldId, &bodyDef)
		shapeDef := b2.DefaultShapeDef()
		box := b2.MakeOffsetBox(b2.F(5), b2.QOne(), b2.V2(0, -1), b2.RotIdentity())
		b2.CreatePolygonShape(groundId, &shapeDef, &box)
	}

	{
		s.extent = b2.F(0.025)
		baseCount := 30

		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody

		shapeDef := b2.DefaultShapeDef()

		box := b2.MakeSquare(s.extent)

		for i := range baseCount {
			y := b2.F(2*i + 1).Mul(s.extent)

			for j := i; j < baseCount; j++ {
				x := b2.F(i + 1).Mul(s.extent).
					Add(b2.QFromInt(2 * (j - i)).Mul(s.extent)).
					Sub(b2.F(baseCount).Mul(s.extent))
				bodyDef.Position = b2.Vec2{X: x, Y: y}

				bodyId := b2.CreateBody(s.WorldId, &bodyDef)
				b2.CreatePolygonShape(bodyId, &shapeDef, &box)
			}
		}
	}

	return s
}

func (s *TinyPyramid) Step() {
	s.DrawTextLine("%.1fcm squares", 200*ToFloat64(s.extent))
	s.Base.Step()
}

// Cart has high gravity and a high mass ratio between the chassis and the
// wheels.
type Cart struct {
	Base

	chassisId b2.BodyId
	wheelId1  b2.BodyId
	wheelId2  b2.BodyId
	jointId1  b2.JointId
	jointId2  b2.JointId

	contactHertz        float64
	contactDampingRatio float64
	contactSpeed        float64
	jointHertz          float64
	jointDampingRatio   float64
}

func NewCart(ctx *SampleContext) Sample {
	s := &Cart{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 1}
		ctx.Camera.Zoom = 1.5
	}

	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Position = b2.V2(0, -1)
		groundId := b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		groundBox := b2.MakeBox(b2.F(20), b2.QOne())
		b2.CreatePolygonShape(groundId, &shapeDef, &groundBox)
	}

	s.WorldId.SetGravity(b2.V2(0, -22))

	s.contactHertz = 30
	s.contactDampingRatio = 10
	s.contactSpeed = 3
	s.setContactTuning()

	s.jointHertz = 60
	s.jointDampingRatio = 1

	s.createScene()
	return s
}

func (s *Cart) setContactTuning() {
	s.WorldId.SetContactTuning(FromFloat64(s.contactHertz), FromFloat64(s.contactDampingRatio), FromFloat64(s.contactSpeed))
}

func (s *Cart) createScene() {
	if !s.chassisId.IsNull() {
		b2.DestroyBody(s.chassisId)
	}

	if !s.wheelId1.IsNull() {
		b2.DestroyBody(s.wheelId1)
	}

	if !s.wheelId2.IsNull() {
		b2.DestroyBody(s.wheelId2)
	}

	yBase := b2.F(2)

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.Position = b2.Vec2{Y: yBase}
	s.chassisId = b2.CreateBody(s.WorldId, &bodyDef)

	shapeDef := b2.DefaultShapeDef()
	shapeDef.Density = b2.F(100)

	box := b2.MakeOffsetBox(b2.QHalf(), b2.F(0.25), b2.V2(0.0, 0.25), b2.RotIdentity())
	b2.CreatePolygonShape(s.chassisId, &shapeDef, &box)

	shapeDef = b2.DefaultShapeDef()
	shapeDef.Material.RollingResistance = b2.F(0.02)
	shapeDef.Density = b2.F(10)

	circle := b2.Circle{Radius: b2.F(0.1)}
	bodyDef.Position = b2.Vec2{X: b2.F(-0.4), Y: yBase.Sub(b2.F(0.15))}
	s.wheelId1 = b2.CreateBody(s.WorldId, &bodyDef)
	b2.CreateCircleShape(s.wheelId1, &shapeDef, &circle)

	bodyDef.Position = b2.Vec2{X: b2.F(0.4), Y: yBase.Sub(b2.F(0.15))}
	s.wheelId2 = b2.CreateBody(s.WorldId, &bodyDef)
	b2.CreateCircleShape(s.wheelId2, &shapeDef, &circle)

	jointHertz := FromFloat64(s.jointHertz)
	jointDampingRatio := FromFloat64(s.jointDampingRatio)

	jointDef := b2.DefaultRevoluteJointDef()
	jointDef.BodyIdA = s.chassisId
	jointDef.BodyIdB = s.wheelId1
	jointDef.LocalAnchorA = b2.V2(-0.4, -0.15)
	jointDef.LocalAnchorB = b2.Vec2{}

	s.jointId1 = b2.CreateRevoluteJoint(s.WorldId, &jointDef)
	s.jointId1.SetConstraintTuning(jointHertz, jointDampingRatio)

	jointDef.BodyIdA = s.chassisId
	jointDef.BodyIdB = s.wheelId2
	jointDef.LocalAnchorA = b2.V2(0.4, -0.15)
	jointDef.LocalAnchorB = b2.Vec2{}

	s.jointId2 = b2.CreateRevoluteJoint(s.WorldId, &jointDef)
	s.jointId2.SetConstraintTuning(jointHertz, jointDampingRatio)
}

func (s *Cart) UpdateGui() {
	height := 240
	gui := s.Context.Gui
	gui.Begin("Cart", 10, s.Context.Camera.Height-height-50, 320, height)

	changed := false
	gui.Text("Contact")
	changed = changed || gui.SliderFloat("Hertz", &s.contactHertz, 0, 240)
	changed = changed || gui.SliderFloat("Damping Ratio", &s.contactDampingRatio, 0, 1000)
	changed = changed || gui.SliderFloat("Speed", &s.contactSpeed, 0, 5)

	if changed {
		s.setContactTuning()
		s.createScene()
	}

	changed = false
	gui.Text("Joint")
	changed = changed || gui.SliderFloat("Hertz", &s.jointHertz, 0, 240)
	changed = changed || gui.SliderFloat("Damping Ratio", &s.jointDampingRatio, 0, 1000)

	changed = changed || gui.Button("Reset Scene")

	if changed {
		s.jointId1.SetConstraintTuning(FromFloat64(s.jointHertz), FromFloat64(s.jointDampingRatio))
		s.jointId2.SetConstraintTuning(FromFloat64(s.jointHertz), FromFloat64(s.jointDampingRatio))
		s.createScene()
	}

	gui.End()
}
