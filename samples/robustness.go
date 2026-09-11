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
func createRobustnessGround(worldId dbox2d.WorldId) {
	bodyDef := dbox2d.DefaultBodyDef()
	groundId := dbox2d.CreateBody(worldId, &bodyDef)
	shapeDef := dbox2d.DefaultShapeDef()
	box := dbox2d.MakeOffsetBox(dbox2d.QFromInt(50), dbox2d.QOne(), qv("0", "-1"), dbox2d.RotIdentity())
	dbox2d.CreatePolygonShape(groundId, &shapeDef, &box)
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

	extent := dbox2d.QOne()

	createRobustnessGround(s.WorldId)

	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		box := dbox2d.MakeBox(extent, extent)
		shapeDef := dbox2d.DefaultShapeDef()

		for j := range 3 {
			count := 10
			// -20 + 2 * (count + 1) * j, in units of extent
			offset := dbox2d.QFromInt(-20 + 2*(count+1)*j).Mul(extent)
			y := extent
			for count > 0 {
				for i := range count {
					// 2 * (i - count / 2), in units of extent
					coeff := dbox2d.QFromInt(2*i - count)

					yy := y
					if count == 1 {
						yy = y.Add(dbox2d.QFromInt(2))
					}
					bodyDef.Position = dbox2d.Vec2{X: coeff.Mul(extent).Add(offset), Y: yy}
					bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)

					shapeDef.Density = dbox2d.QOne()
					if count == 1 {
						shapeDef.Density = dbox2d.QFromInt((j + 1) * 100)
					}
					dbox2d.CreatePolygonShape(bodyId, &shapeDef, &box)
				}

				count--
				y = y.Add(dbox2d.QFromInt(2).Mul(extent))
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
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		shapeDef := dbox2d.DefaultShapeDef()

		extent := dbox2d.QOne()
		smallBox := dbox2d.MakeBox(dbox2d.QHalf().Mul(extent), dbox2d.QHalf().Mul(extent))
		bigBox := dbox2d.MakeBox(dbox2d.QFromInt(10).Mul(extent), dbox2d.QFromInt(10).Mul(extent))

		{
			bodyDef.Position = dbox2d.Vec2{X: dbox2d.QFromInt(-9).Mul(extent), Y: dbox2d.QHalf().Mul(extent)}
			bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)
			dbox2d.CreatePolygonShape(bodyId, &shapeDef, &smallBox)
		}

		{
			bodyDef.Position = dbox2d.Vec2{X: dbox2d.QFromInt(9).Mul(extent), Y: dbox2d.QHalf().Mul(extent)}
			bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)
			dbox2d.CreatePolygonShape(bodyId, &shapeDef, &smallBox)
		}

		{
			bodyDef.Position = dbox2d.Vec2{Y: dbox2d.QFromInt(10 + 16).Mul(extent)}
			bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)
			dbox2d.CreatePolygonShape(bodyId, &shapeDef, &bigBox)
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
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		shapeDef := dbox2d.DefaultShapeDef()

		extent := dbox2d.QOne()
		half := dbox2d.QHalf().Mul(extent)
		points := []dbox2d.Vec2{{X: half.Neg()}, {X: half}, {Y: extent}}
		hull := dbox2d.ComputeHull(points)
		smallTriangle := dbox2d.MakePolygon(&hull, dbox2d.QZero())
		bigBox := dbox2d.MakeBox(dbox2d.QFromInt(10).Mul(extent), dbox2d.QFromInt(10).Mul(extent))

		{
			bodyDef.Position = dbox2d.Vec2{X: dbox2d.QFromInt(-9).Mul(extent), Y: half}
			bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)
			dbox2d.CreatePolygonShape(bodyId, &shapeDef, &smallTriangle)
		}

		{
			bodyDef.Position = dbox2d.Vec2{X: dbox2d.QFromInt(9).Mul(extent), Y: half}
			bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)
			dbox2d.CreatePolygonShape(bodyId, &shapeDef, &smallTriangle)
		}

		{
			bodyDef.Position = dbox2d.Vec2{Y: dbox2d.QFromInt(10 + 4).Mul(extent)}
			bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)
			dbox2d.CreatePolygonShape(bodyId, &shapeDef, &bigBox)
		}
	}

	return s
}

// OverlapRecovery starts a pyramid of overlapping boxes and lets the
// contact push-out separate them.
type OverlapRecovery struct {
	Base

	bodyIds      []dbox2d.BodyId
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

	bodyDef := dbox2d.DefaultBodyDef()
	groundId := dbox2d.CreateBody(s.WorldId, &bodyDef)

	groundWidth := dbox2d.QFromInt(40)
	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Density = dbox2d.QOne()

	segment := dbox2d.Segment{Point1: dbox2d.Vec2{X: groundWidth.Neg()}, Point2: dbox2d.Vec2{X: groundWidth}}
	dbox2d.CreateSegmentShape(groundId, &shapeDef, &segment)

	s.createScene()
	return s
}

func (s *OverlapRecovery) createScene() {
	for _, bodyId := range s.bodyIds {
		dbox2d.DestroyBody(bodyId)
	}

	s.WorldId.SetContactTuning(FromFloat64(s.hertz), FromFloat64(s.dampingRatio), FromFloat64(s.pushOut))

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody

	extent := FromFloat64(s.extent)
	box := dbox2d.MakeBox(extent, extent)
	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Density = dbox2d.QOne()

	bodyCount := s.baseCount * (s.baseCount + 1) / 2
	s.bodyIds = make([]dbox2d.BodyId, 0, bodyCount)

	fraction := dbox2d.QOne().Sub(FromFloat64(s.overlap))
	step := dbox2d.QFromInt(2).Mul(fraction).Mul(extent)
	y := extent
	for i := range s.baseCount {
		x := fraction.Mul(extent).Mul(dbox2d.QFromInt(i - s.baseCount))
		for j := i; j < s.baseCount; j++ {
			bodyDef.Position = dbox2d.Vec2{X: x, Y: y}
			bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)

			dbox2d.CreatePolygonShape(bodyId, &shapeDef, &box)

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

	extent dbox2d.Q
}

func NewTinyPyramid(ctx *SampleContext) Sample {
	s := &TinyPyramid{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 0.8}
		ctx.Camera.Zoom = 1
	}

	{
		bodyDef := dbox2d.DefaultBodyDef()
		groundId := dbox2d.CreateBody(s.WorldId, &bodyDef)
		shapeDef := dbox2d.DefaultShapeDef()
		box := dbox2d.MakeOffsetBox(dbox2d.QFromInt(5), dbox2d.QOne(), qv("0", "-1"), dbox2d.RotIdentity())
		dbox2d.CreatePolygonShape(groundId, &shapeDef, &box)
	}

	{
		s.extent = qs("0.025")
		baseCount := 30

		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody

		shapeDef := dbox2d.DefaultShapeDef()

		box := dbox2d.MakeSquare(s.extent)

		for i := range baseCount {
			y := dbox2d.QFromInt(2*i + 1).Mul(s.extent)

			for j := i; j < baseCount; j++ {
				x := dbox2d.QFromInt(i + 1).Mul(s.extent).
					Add(dbox2d.QFromInt(2 * (j - i)).Mul(s.extent)).
					Sub(dbox2d.QFromInt(baseCount).Mul(s.extent))
				bodyDef.Position = dbox2d.Vec2{X: x, Y: y}

				bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)
				dbox2d.CreatePolygonShape(bodyId, &shapeDef, &box)
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

	chassisId dbox2d.BodyId
	wheelId1  dbox2d.BodyId
	wheelId2  dbox2d.BodyId
	jointId1  dbox2d.JointId
	jointId2  dbox2d.JointId

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
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Position = qv("0", "-1")
		groundId := dbox2d.CreateBody(s.WorldId, &bodyDef)

		shapeDef := dbox2d.DefaultShapeDef()
		groundBox := dbox2d.MakeBox(dbox2d.QFromInt(20), dbox2d.QOne())
		dbox2d.CreatePolygonShape(groundId, &shapeDef, &groundBox)
	}

	s.WorldId.SetGravity(qv("0", "-22"))

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
		dbox2d.DestroyBody(s.chassisId)
	}

	if !s.wheelId1.IsNull() {
		dbox2d.DestroyBody(s.wheelId1)
	}

	if !s.wheelId2.IsNull() {
		dbox2d.DestroyBody(s.wheelId2)
	}

	yBase := dbox2d.QFromInt(2)

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	bodyDef.Position = dbox2d.Vec2{Y: yBase}
	s.chassisId = dbox2d.CreateBody(s.WorldId, &bodyDef)

	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Density = dbox2d.QFromInt(100)

	box := dbox2d.MakeOffsetBox(dbox2d.QHalf(), qs("0.25"), qv("0", "0.25"), dbox2d.RotIdentity())
	dbox2d.CreatePolygonShape(s.chassisId, &shapeDef, &box)

	shapeDef = dbox2d.DefaultShapeDef()
	shapeDef.Material.RollingResistance = qs("0.02")
	shapeDef.Density = dbox2d.QFromInt(10)

	circle := dbox2d.Circle{Radius: qs("0.1")}
	bodyDef.Position = dbox2d.Vec2{X: qs("-0.4"), Y: yBase.Sub(qs("0.15"))}
	s.wheelId1 = dbox2d.CreateBody(s.WorldId, &bodyDef)
	dbox2d.CreateCircleShape(s.wheelId1, &shapeDef, &circle)

	bodyDef.Position = dbox2d.Vec2{X: qs("0.4"), Y: yBase.Sub(qs("0.15"))}
	s.wheelId2 = dbox2d.CreateBody(s.WorldId, &bodyDef)
	dbox2d.CreateCircleShape(s.wheelId2, &shapeDef, &circle)

	jointHertz := FromFloat64(s.jointHertz)
	jointDampingRatio := FromFloat64(s.jointDampingRatio)

	jointDef := dbox2d.DefaultRevoluteJointDef()
	jointDef.BodyIdA = s.chassisId
	jointDef.BodyIdB = s.wheelId1
	jointDef.LocalAnchorA = qv("-0.4", "-0.15")
	jointDef.LocalAnchorB = dbox2d.Vec2{}

	s.jointId1 = dbox2d.CreateRevoluteJoint(s.WorldId, &jointDef)
	s.jointId1.SetConstraintTuning(jointHertz, jointDampingRatio)

	jointDef.BodyIdA = s.chassisId
	jointDef.BodyIdB = s.wheelId2
	jointDef.LocalAnchorA = qv("0.4", "-0.15")
	jointDef.LocalAnchorB = dbox2d.Vec2{}

	s.jointId2 = dbox2d.CreateRevoluteJoint(s.WorldId, &jointDef)
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
