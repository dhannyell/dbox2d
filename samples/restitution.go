// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/sample_restitution.cpp of Box2D v3.2.0

package samples

import (
	"fmt"
	"math"

	"github.com/dhannyell/dbox2d"
)

func init() {
	RegisterSample("Restitution", "Varying", NewVaryingRestitution)
	RegisterSample("Restitution", "Single Box Restitution", NewSingleBoxRestitution)
	RegisterSample("Restitution", "Single Circle Restitution", NewSingleCircleRestitution)
	RegisterSample("Restitution", "Circle Stack Restitution", NewCircleStackRestitution)
	RegisterSample("Restitution", "Restitution Propagation", NewRestitutionPropagation)
	RegisterSample("Restitution", "Box Restitution", NewBoxRestitution)
	RegisterSample("Restitution", "Rotated Box Restitution", NewRotatedBoxRestitution)
	RegisterSample("Restitution", "Two Box Restitution", NewTwoBoxRestitution)
	RegisterSample("Restitution", "Circle Restitution", NewCircleRestitution)
}

// restitutionComputeEnergy is ComputeEnergy: the linear and angular kinetic
// energy and the gravitational potential energy of the bodies, in float64
// since it only feeds the overlay. Angular velocity is in turns per second in
// this port, so it is converted to radians for the energy.
func restitutionComputeEnergy(worldId b2.WorldId, bodyIds []b2.BodyId) (linear, angular, potential float64) {
	gravity := worldId.GetGravity()
	gx, gy := ToFloat64(gravity.X), ToFloat64(gravity.Y)

	for _, bodyId := range bodyIds {
		massData := bodyId.GetMassData()
		v := bodyId.GetLinearVelocity()
		vx, vy := ToFloat64(v.X), ToFloat64(v.Y)
		w := 2 * math.Pi * ToFloat64(bodyId.GetAngularVelocity())
		mass := ToFloat64(massData.Mass)
		center := bodyId.GetWorldCenter()

		linear += 0.5 * mass * (vx*vx + vy*vy)
		angular += 0.5 * ToFloat64(massData.RotationalInertia) * w * w
		potential -= mass * (gx*ToFloat64(center.X) + gy*ToFloat64(center.Y))
	}
	return
}

// restitutionDrawEnergy prints the three energy lines the samples share and
// returns the new peak.
func (b *Base) restitutionDrawEnergy(linear, angular, potential, start, peak float64) float64 {
	total := linear + angular + potential
	peak = math.Max(peak, total)

	scale := 0.0
	if start != 0 {
		scale = 100 / start
	}
	b.DrawTextLine("kinetic   = %.3f J linear + %.3f J angular", linear, angular)
	b.DrawTextLine("potential = %.3f J", potential)
	b.DrawTextLine("total     = %.3f J (%.2f%% of start, peak %.2f%%)", total, scale*total, scale*peak)
	return peak
}

func restitutionHLine(draw *b2.DebugDraw, x1, x2, y float64, color b2.HexColor) {
	draw.DrawLineFcn(b2.V2(x1, y), b2.V2(x2, y), color)
}

const varyingRestitutionCount = 40

const (
	restitutionCircleShape = iota
	restitutionBoxShape
)

// VaryingRestitution drops a row of shapes with increasing restitution. It
// mirrors the VaryingRestitution sample; restitution is approximate since
// Box2D uses speculative collision.
type VaryingRestitution struct {
	Base

	bodyIds   [varyingRestitutionCount]b2.BodyId
	shapeType int
}

// NewVaryingRestitution builds the varying restitution scene.
func NewVaryingRestitution(ctx *SampleContext) Sample {
	s := &VaryingRestitution{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 4, Y: 17}
		ctx.Camera.Zoom = 27.5
	}

	{
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(s.WorldId, &bodyDef)

		h := b2.F(varyingRestitutionCount)
		segment := b2.Segment{Point1: b2.Vec2{X: h.Neg()}, Point2: b2.Vec2{X: h}}
		shapeDef := b2.DefaultShapeDef()
		b2.CreateSegmentShape(groundId, &shapeDef, &segment)
	}

	s.shapeType = restitutionCircleShape
	s.createBodies()
	return s
}

func (s *VaryingRestitution) createBodies() {
	for i := range varyingRestitutionCount {
		if !s.bodyIds[i].IsNull() {
			b2.DestroyBody(s.bodyIds[i])
			s.bodyIds[i] = b2.BodyId{}
		}
	}

	circle := b2.Circle{Radius: b2.QHalf()}
	box := b2.MakeBox(b2.QHalf(), b2.QHalf())

	shapeDef := b2.DefaultShapeDef()
	shapeDef.Density = b2.QOne()
	shapeDef.Material.Restitution = b2.QZero()
	shapeDef.Material.Friction = b2.QZero()

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody

	dr := b2.QFromRatio(1, max(varyingRestitutionCount-1, 1))
	x := b2.QFromInt(-(varyingRestitutionCount - 1))
	dx := b2.F(2)

	for i := range varyingRestitutionCount {
		bodyDef.Name = fmt.Sprintf("%.2f", ToFloat64(shapeDef.Material.Restitution))
		bodyDef.Position = b2.Vec2{X: x, Y: b2.F(40)}
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)

		s.bodyIds[i] = bodyId

		if s.shapeType == restitutionCircleShape {
			b2.CreateCircleShape(bodyId, &shapeDef, &circle)
		} else {
			b2.CreatePolygonShape(bodyId, &shapeDef, &box)
		}

		shapeDef.Material.Restitution = shapeDef.Material.Restitution.Add(dr)
		x = x.Add(dx)
	}
}

func (s *VaryingRestitution) UpdateGui() {
	height := 100
	gui := s.Context.Gui
	gui.Begin("Restitution", 10, s.Context.Camera.Height-height-50, 240, height)

	changed := false
	shapeTypes := []string{"Circle", "Box"}

	changed = gui.Combo("Shape", &s.shapeType, shapeTypes) || changed
	changed = gui.Button("Reset") || changed

	if changed {
		s.createBodies()
	}

	gui.End()
}

func (s *VaryingRestitution) Step() {
	s.Base.Step()

	h := float64(varyingRestitutionCount)
	restitutionHLine(&s.Context.Draw, -h, h, 40.5, b2.ColorRed)
}

// SingleBoxRestitution drops one bouncy box and tracks its energy. It mirrors
// the SingleBoxRestitution sample.
type SingleBoxRestitution struct {
	Base

	bodyId      b2.BodyId
	maxY        float64
	startEnergy float64
	peakEnergy  float64
	height      float64
}

// NewSingleBoxRestitution builds the single box scene.
func NewSingleBoxRestitution(ctx *SampleContext) Sample {
	s := &SingleBoxRestitution{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 5}
		ctx.Camera.Zoom = 8
	}

	{
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(s.WorldId, &bodyDef)

		segment := b2.Segment{Point1: b2.V2(-4, 0), Point2: b2.V2(4, 0)}
		shapeDef := b2.DefaultShapeDef()
		b2.CreateSegmentShape(groundId, &shapeDef, &segment)
	}

	s.height = 5
	s.maxY = 5

	box := b2.MakeBox(b2.QHalf(), b2.QHalf())

	shapeDef := b2.DefaultShapeDef()
	shapeDef.Density = b2.QOne()
	shapeDef.Material.Restitution = b2.QOne()
	shapeDef.Material.Friction = b2.QZero()
	shapeDef.EnableHitEvents = true

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.Position = b2.V2(0, s.height)
	bodyDef.SafetyFactor = b2.QFromRatio(1, 100)
	s.bodyId = b2.CreateBody(s.WorldId, &bodyDef)
	b2.CreatePolygonShape(s.bodyId, &shapeDef, &box)

	linear, angular, potential := restitutionComputeEnergy(s.WorldId, []b2.BodyId{s.bodyId})
	s.startEnergy = linear + angular + potential
	s.peakEnergy = s.startEnergy
	return s
}

func (s *SingleBoxRestitution) Step() {
	s.Base.Step()

	restitutionHLine(&s.Context.Draw, -4, 4, s.height+0.5, b2.ColorRed)

	events := s.WorldId.GetContactEvents()

	p := s.bodyId.GetPosition()
	if len(events.HitEvents) == 1 {
		s.maxY = ToFloat64(p.Y)
	} else {
		s.maxY = math.Max(s.maxY, ToFloat64(p.Y))
	}

	s.DrawTextLine("maxY = %.2f", s.maxY)

	linear, angular, potential := restitutionComputeEnergy(s.WorldId, []b2.BodyId{s.bodyId})
	s.peakEnergy = s.restitutionDrawEnergy(linear, angular, potential, s.startEnergy, s.peakEnergy)
}

// SingleCircleRestitution drops one bouncy circle and tracks its energy. It
// mirrors the SingleCircleRestitution sample.
type SingleCircleRestitution struct {
	Base

	bodyId      b2.BodyId
	maxY        float64
	startEnergy float64
	peakEnergy  float64
}

// NewSingleCircleRestitution builds the single circle scene.
func NewSingleCircleRestitution(ctx *SampleContext) Sample {
	s := &SingleCircleRestitution{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 5}
		ctx.Camera.Zoom = 8
	}

	{
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(s.WorldId, &bodyDef)

		segment := b2.Segment{Point1: b2.V2(-4, 0), Point2: b2.V2(4, 0)}
		shapeDef := b2.DefaultShapeDef()
		b2.CreateSegmentShape(groundId, &shapeDef, &segment)
	}

	circle := b2.Circle{Radius: b2.QHalf()}

	shapeDef := b2.DefaultShapeDef()
	shapeDef.Density = b2.QOne()
	shapeDef.Material.Restitution = b2.QOne()
	shapeDef.Material.Friction = b2.QZero()
	shapeDef.EnableHitEvents = true

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.Position = b2.V2(0, 10)
	bodyDef.SafetyFactor = b2.QFromRatio(1, 100)
	s.bodyId = b2.CreateBody(s.WorldId, &bodyDef)
	b2.CreateCircleShape(s.bodyId, &shapeDef, &circle)

	s.maxY = 10

	linear, angular, potential := restitutionComputeEnergy(s.WorldId, []b2.BodyId{s.bodyId})
	s.startEnergy = linear + angular + potential
	s.peakEnergy = s.startEnergy
	return s
}

func (s *SingleCircleRestitution) Step() {
	s.Base.Step()

	events := s.WorldId.GetContactEvents()

	p := s.bodyId.GetPosition()
	if len(events.HitEvents) == 1 {
		s.maxY = ToFloat64(p.Y)
	} else {
		s.maxY = math.Max(s.maxY, ToFloat64(p.Y))
	}

	restitutionHLine(&s.Context.Draw, -4, 4, 10.5, b2.ColorRed)

	s.DrawTextLine("maxY = %.2f", s.maxY)

	linear, angular, potential := restitutionComputeEnergy(s.WorldId, []b2.BodyId{s.bodyId})
	s.peakEnergy = s.restitutionDrawEnergy(linear, angular, potential, s.startEnergy, s.peakEnergy)
}

const (
	circleStackImpactSpeed   = 5.0
	circleStackMaxStackCount = 3
)

// CircleStackRestitution drops a bouncy circle onto a stack of circles. It
// mirrors the CircleStackRestitution sample, which is similar to the
// MeasureSupportedBounce unit test.
type CircleStackRestitution struct {
	Base

	bodyIds     [circleStackMaxStackCount + 1]b2.BodyId
	impactorId  b2.BodyId
	bodyCount   int
	stackCount  int
	restitution float64
	startHeight float64
	coefficient float64
	startEnergy float64
	peakEnergy  float64
	gravity     bool
	latched     bool
	hit         bool
}

// NewCircleStackRestitution builds the circle stack restitution scene.
func NewCircleStackRestitution(ctx *SampleContext) Sample {
	s := &CircleStackRestitution{Base: NewBase(ctx), stackCount: 2, restitution: 1}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 2.5}
		ctx.Camera.Zoom = 6
	}

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Position = b2.V2(0, -1)
	groundId := b2.CreateBody(s.WorldId, &bodyDef)

	shapeDef := b2.DefaultShapeDef()
	shapeDef.Material.Friction = b2.QZero()
	shapeDef.Material.Restitution = b2.QZero()
	box := b2.MakeBox(b2.F(40), b2.QOne())
	b2.CreatePolygonShape(groundId, &shapeDef, &box)

	s.createScene()
	return s
}

func (s *CircleStackRestitution) createBall(y, velocityY, restitution float64, hitEvents bool) b2.BodyId {
	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.Position = b2.V2(0, y)
	bodyDef.LinearVelocity = b2.V2(0, velocityY)
	bodyDef.EnableSleep = false
	bodyId := b2.CreateBody(s.WorldId, &bodyDef)

	shapeDef := b2.DefaultShapeDef()
	shapeDef.Material.Friction = b2.QZero()
	shapeDef.Material.Restitution = FromFloat64(restitution)
	shapeDef.EnableHitEvents = hitEvents
	circle := b2.Circle{Radius: b2.QHalf()}
	b2.CreateCircleShape(bodyId, &shapeDef, &circle)

	return bodyId
}

func (s *CircleStackRestitution) createScene() {
	for i := range s.bodyCount {
		b2.DestroyBody(s.bodyIds[i])
	}
	s.bodyCount = 0

	if s.gravity {
		s.WorldId.SetGravity(b2.V2(0, -10))
	} else {
		s.WorldId.SetGravity(b2.Vec2{})
	}

	for i := range s.stackCount {
		s.bodyIds[s.bodyCount] = s.createBall(0.5+float64(i), 0, 0, false)
		s.bodyCount++
	}

	// Start half a step of travel above contact so the impact lands mid step, matching the test
	s.startHeight = 0.5 + float64(s.stackCount) + 0.5*circleStackImpactSpeed*(1.0/60.0)

	s.impactorId = s.createBall(s.startHeight, -circleStackImpactSpeed, s.restitution, true)
	s.bodyIds[s.bodyCount] = s.impactorId
	s.bodyCount++

	linear, angular, potential := restitutionComputeEnergy(s.WorldId, s.bodyIds[:s.bodyCount])
	s.startEnergy = linear + angular + potential
	s.peakEnergy = s.startEnergy

	s.coefficient = 0
	s.latched = false
	s.hit = false
}

func (s *CircleStackRestitution) UpdateGui() {
	height := 130
	gui := s.Context.Gui
	gui.Begin("Circle Stack Restitution", 10, s.Context.Camera.Height-height-50, 240, height)

	rebuild := false
	if gui.SliderFloat("Restitution", &s.restitution, 0, 1) {
		rebuild = true
	}
	if gui.SliderInt("Stack", &s.stackCount, 0, circleStackMaxStackCount) {
		rebuild = true
	}
	if gui.Checkbox("Gravity", &s.gravity) {
		rebuild = true
	}
	if gui.Button("Reset") {
		rebuild = true
	}
	if rebuild {
		s.createScene()
	}

	gui.End()
}

func (s *CircleStackRestitution) Step() {
	s.Base.Step()

	events := s.WorldId.GetContactEvents()
	if len(events.HitEvents) > 0 {
		s.hit = true
	}

	vy := ToFloat64(s.impactorId.GetLinearVelocity().Y)

	// The rebound only means something once the impact has happened and the impactor is leaving
	if !s.latched && s.hit && vy > 0 {
		s.coefficient = vy / circleStackImpactSpeed
		s.latched = true
	}

	supportSpeed := 0.0
	for i := range s.stackCount {
		supportSpeed = math.Max(supportSpeed, ToFloat64(s.bodyIds[i].GetLinearVelocity().Len()))
	}

	restitutionHLine(&s.Context.Draw, -2, 2, s.startHeight, b2.ColorRed)

	s.DrawTextLine("impactor vy = %.3f m/s", vy)
	s.DrawTextLine("coefficient = %.4f live (target %.2f)", vy/circleStackImpactSpeed, s.restitution)
	if s.latched {
		s.DrawTextLine("coefficient = %.4f at first rebound", s.coefficient)
	}
	s.DrawTextLine("support speed = %.4f m/s", supportSpeed)

	linear, angular, potential := restitutionComputeEnergy(s.WorldId, s.bodyIds[:s.bodyCount])
	s.peakEnergy = s.restitutionDrawEnergy(linear, angular, potential, s.startEnergy, s.peakEnergy)
}

const (
	propagationColumnCount = 3
	propagationDropHeight  = 5.0
)

// RestitutionPropagation drops bouncy balls on bare ground, one crate and two
// crates. It mirrors the RestitutionPropagation sample: without restitution
// propagation the rebound is shared with the crate, with it the ground
// contact under the crate reacts in the restitution solver.
type RestitutionPropagation struct {
	Base

	bodyIds    [2 * propagationColumnCount]b2.BodyId
	ballIds    [propagationColumnCount]b2.BodyId
	crateIds   [propagationColumnCount]b2.BodyId
	restHeight [propagationColumnCount]float64
	apex       [propagationColumnCount]float64
	rebounding [propagationColumnCount]bool
	done       [propagationColumnCount]bool
	bodyCount  int
	crateCount int

	restitution float64
	kick        float64
}

// NewRestitutionPropagation builds the restitution propagation scene.
func NewRestitutionPropagation(ctx *SampleContext) Sample {
	s := &RestitutionPropagation{Base: NewBase(ctx), restitution: 0.9}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 4}
		ctx.Camera.Zoom = 10
	}

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Position = b2.V2(0, -1)
	groundId := b2.CreateBody(s.WorldId, &bodyDef)

	shapeDef := b2.DefaultShapeDef()
	shapeDef.Material.Friction = FromFloat64(0.6)
	box := b2.MakeBox(b2.F(40), b2.QOne())
	b2.CreatePolygonShape(groundId, &shapeDef, &box)

	s.createScene()
	return s
}

func (s *RestitutionPropagation) createScene() {
	for i := range s.bodyCount {
		b2.DestroyBody(s.bodyIds[i])
	}
	s.bodyCount = 0
	s.crateCount = 0
	s.kick = 0

	box := b2.MakeBox(b2.QHalf(), b2.QHalf())
	circle := b2.Circle{Radius: b2.QHalf()}

	for column := range propagationColumnCount {
		x := 4.0 * float64(column-1)

		for k := range column {
			bodyDef := b2.DefaultBodyDef()
			bodyDef.Type = b2.DynamicBody
			bodyDef.Position = b2.V2(x, 0.5+float64(k))
			bodyDef.EnableSleep = false
			crateId := b2.CreateBody(s.WorldId, &bodyDef)

			shapeDef := b2.DefaultShapeDef()
			b2.CreatePolygonShape(crateId, &shapeDef, &box)

			s.bodyIds[s.bodyCount] = crateId
			s.bodyCount++
			s.crateIds[s.crateCount] = crateId
			s.crateCount++
		}

		s.restHeight[column] = 0.5 + float64(column)

		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.V2(x, s.restHeight[column]+propagationDropHeight)
		bodyDef.EnableSleep = false
		ballId := b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		shapeDef.Material.Restitution = FromFloat64(s.restitution)
		b2.CreateCircleShape(ballId, &shapeDef, &circle)

		s.bodyIds[s.bodyCount] = ballId
		s.bodyCount++
		s.ballIds[column] = ballId
		s.apex[column] = 0
		s.rebounding[column] = false
		s.done[column] = false
	}
}

func (s *RestitutionPropagation) UpdateGui() {
	height := 80
	gui := s.Context.Gui
	gui.Begin("Restitution Propagation", 10, s.Context.Camera.Height-height-50, 240, height)

	rebuild := gui.SliderFloat("Restitution", &s.restitution, 0, 1)
	if gui.Button("Reset") {
		rebuild = true
	}
	if rebuild {
		s.createScene()
	}

	gui.End()
}

func (s *RestitutionPropagation) Step() {
	s.Base.Step()

	labels := [propagationColumnCount]string{"ground", "one crate", "two crates"}
	draw := &s.Context.Draw

	for column := range propagationColumnCount {
		vy := ToFloat64(s.ballIds[column].GetLinearVelocity().Y)
		height := ToFloat64(s.ballIds[column].GetPosition().Y) - s.restHeight[column]

		// The first rebound apex: the ball starts at rest and falls, so upward velocity means it bounced
		if !s.rebounding[column] && vy > 0 {
			s.rebounding[column] = true
		}

		if s.rebounding[column] && !s.done[column] {
			s.apex[column] = math.Max(s.apex[column], height)
			if vy < 0 {
				s.done[column] = true
			}
		}

		x := 4.0 * float64(column-1)
		restitutionHLine(draw, x-1, x+1, s.restHeight[column]+propagationDropHeight, b2.ColorRed)
		if s.apex[column] > 0 {
			restitutionHLine(draw, x-1, x+1, s.restHeight[column]+s.apex[column], b2.ColorGreen)
		}

		s.DrawTextLine("%s: apex %.2f m (%.0f%% of drop)", labels[column], s.apex[column], 100*s.apex[column]/propagationDropHeight)
	}

	for i := range s.crateCount {
		s.kick = math.Max(s.kick, ToFloat64(s.crateIds[i].GetLinearVelocity().Len()))
	}

	s.DrawTextLine("peak crate speed %.2f m/s", s.kick)
}

const boxRestitutionImpactSpeed = 5.0

// BoxRestitution drops a spinning box on the ground. It mirrors the
// BoxRestitution sample, which is similar to the MeasureFlatBounce and
// SpinTest unit tests.
type BoxRestitution struct {
	Base

	boxId       b2.BodyId
	restitution float64
	spin        float64
	coefficient float64
	startEnergy float64
	peakEnergy  float64
	latched     bool
	hit         bool
}

// NewBoxRestitution builds the box restitution scene.
func NewBoxRestitution(ctx *SampleContext) Sample {
	s := &BoxRestitution{Base: NewBase(ctx), restitution: 0.9}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 8}
		ctx.Camera.Zoom = 10
	}

	// Gravity would bias the measured coefficient
	s.WorldId.SetGravity(b2.Vec2{})

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Position = b2.V2(0, -1)
	groundId := b2.CreateBody(s.WorldId, &bodyDef)

	shapeDef := b2.DefaultShapeDef()
	shapeDef.Material.Friction = b2.QZero()
	shapeDef.Material.Restitution = b2.QZero()
	box := b2.MakeBox(b2.F(40), b2.QOne())
	b2.CreatePolygonShape(groundId, &shapeDef, &box)

	s.createScene()
	return s
}

func (s *BoxRestitution) createScene() {
	if !s.boxId.IsNull() {
		b2.DestroyBody(s.boxId)
	}

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.Position = b2.V2(0, 0.25+0.5*boxRestitutionImpactSpeed*(1.0/60.0))
	bodyDef.LinearVelocity = b2.V2(0, -boxRestitutionImpactSpeed)
	bodyDef.AngularVelocity = radiansToTurns(s.spin)
	bodyDef.EnableSleep = false
	s.boxId = b2.CreateBody(s.WorldId, &bodyDef)

	shapeDef := b2.DefaultShapeDef()
	shapeDef.Density = b2.QOne()
	shapeDef.Material.Friction = b2.QZero()
	shapeDef.Material.Restitution = FromFloat64(s.restitution)
	shapeDef.EnableHitEvents = true
	box := b2.MakeBox(b2.QOne(), b2.QFromRatio(1, 4))
	b2.CreatePolygonShape(s.boxId, &shapeDef, &box)

	linear, angular, potential := restitutionComputeEnergy(s.WorldId, []b2.BodyId{s.boxId})
	s.startEnergy = linear + angular + potential
	s.peakEnergy = s.startEnergy

	s.coefficient = 0
	s.latched = false
	s.hit = false
}

func (s *BoxRestitution) UpdateGui() {
	height := 110
	gui := s.Context.Gui
	gui.Begin("Box Restitution", 10, s.Context.Camera.Height-height-50, 240, height)

	rebuild := false
	if gui.SliderFloat("Restitution", &s.restitution, 0, 1) {
		rebuild = true
	}
	if gui.SliderFloat("Spin", &s.spin, 0, 2) {
		rebuild = true
	}
	if gui.Button("Reset") {
		rebuild = true
	}
	if rebuild {
		s.createScene()
	}

	gui.End()
}

func (s *BoxRestitution) Step() {
	s.Base.Step()

	events := s.WorldId.GetContactEvents()
	if len(events.HitEvents) > 0 {
		s.hit = true
	}

	vy := ToFloat64(s.boxId.GetLinearVelocity().Y)
	w := 2 * math.Pi * ToFloat64(s.boxId.GetAngularVelocity())

	if !s.latched && s.hit && vy > 0 {
		s.coefficient = vy / boxRestitutionImpactSpeed
		s.latched = true
	}

	draw := &s.Context.Draw
	draw.DrawPointFcn(s.boxId.GetWorldPoint(b2.V2(-1, -0.25)), b2.F(8), b2.ColorYellow)
	draw.DrawPointFcn(s.boxId.GetWorldPoint(b2.V2(1, -0.25)), b2.F(8), b2.ColorYellow)

	s.DrawTextLine("vy = %.3f m/s", vy)
	s.DrawTextLine("coefficient = %.4f live (target %.2f)", vy/boxRestitutionImpactSpeed, s.restitution)
	if s.latched {
		s.DrawTextLine("coefficient = %.4f at first rebound", s.coefficient)
	}
	s.DrawTextLine("spin in = %.2f, spin out = %.4f (ideal %.2f at e = 1)", s.spin, w, -s.spin)

	linear, angular, potential := restitutionComputeEnergy(s.WorldId, []b2.BodyId{s.boxId})
	s.peakEnergy = s.restitutionDrawEnergy(linear, angular, potential, s.startEnergy, s.peakEnergy)
}

const (
	rotatedBoxCount      = 12
	rotatedBoxDropHeight = 10.0
)

// RotatedBoxRestitution drops boxes at a range of rotations. It mirrors the
// RotatedBoxRestitution sample.
type RotatedBoxRestitution struct {
	Base

	bodyIds      [rotatedBoxCount]b2.BodyId
	startEnergy  [rotatedBoxCount]float64
	restitution  float64
	friction     float64
	safetyFactor float64
}

// NewRotatedBoxRestitution builds the rotated box restitution scene.
func NewRotatedBoxRestitution(ctx *SampleContext) Sample {
	s := &RotatedBoxRestitution{Base: NewBase(ctx), restitution: 0.9, safetyFactor: 0.5}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 4}
		ctx.Camera.Zoom = 14
	}

	bodyDef := b2.DefaultBodyDef()
	groundId := b2.CreateBody(s.WorldId, &bodyDef)

	shapeDef := b2.DefaultShapeDef()
	h := b2.F(2*rotatedBoxCount + 1)
	segment := b2.Segment{Point1: b2.Vec2{X: h.Neg()}, Point2: b2.Vec2{X: h}}
	b2.CreateSegmentShape(groundId, &shapeDef, &segment)

	s.createScene()
	return s
}

func (s *RotatedBoxRestitution) createScene() {
	for i := range rotatedBoxCount {
		if !s.bodyIds[i].IsNull() {
			b2.DestroyBody(s.bodyIds[i])
			s.bodyIds[i] = b2.BodyId{}
		}
	}

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.SafetyFactor = FromFloat64(s.safetyFactor)

	shapeDef := b2.DefaultShapeDef()
	shapeDef.Material.Friction = FromFloat64(s.friction)
	shapeDef.Material.Restitution = FromFloat64(s.restitution)

	// Boxes in a negative group never collide with each other.
	shapeDef.Filter.GroupIndex = -1

	box := b2.MakeBox(b2.QHalf(), b2.QHalf())

	x := b2.QFromInt(-(rotatedBoxCount - 1))

	for i := range rotatedBoxCount {
		bodyDef.Position = b2.Vec2{X: x, Y: b2.F(rotatedBoxDropHeight)}
		// A square repeats every quarter turn: i * 0.5 * pi / count radians.
		bodyDef.Rotation = b2.MakeRot(b2.QFromRatio(i, 4*rotatedBoxCount))
		s.bodyIds[i] = b2.CreateBody(s.WorldId, &bodyDef)
		b2.CreatePolygonShape(s.bodyIds[i], &shapeDef, &box)

		linear, angular, potential := restitutionComputeEnergy(s.WorldId, s.bodyIds[i:i+1])
		s.startEnergy[i] = linear + angular + potential

		x = x.Add(b2.F(2))
	}
}

func (s *RotatedBoxRestitution) UpdateGui() {
	height := 130
	gui := s.Context.Gui
	gui.Begin("Rotated Box Restitution", 10, s.Context.Camera.Height-height-50, 240, height)

	rebuild := false
	if gui.SliderFloat("Restitution", &s.restitution, 0, 1) {
		rebuild = true
	}
	if gui.SliderFloat("Friction", &s.friction, 0, 1) {
		rebuild = true
	}
	if gui.SliderFloat("Safety Factor", &s.safetyFactor, 0.01, 0.5) {
		rebuild = true
	}
	if gui.Button("Reset") {
		rebuild = true
	}
	if rebuild {
		s.createScene()
	}

	gui.End()
}

func (s *RotatedBoxRestitution) Step() {
	s.Base.Step()

	h := float64(rotatedBoxCount)
	restitutionHLine(&s.Context.Draw, -h, h, rotatedBoxDropHeight+0.5, b2.ColorRed)

	minPercent := 0.0
	maxPercent := 0.0
	x := -float64(rotatedBoxCount - 1)

	for i := range rotatedBoxCount {
		linear, angular, potential := restitutionComputeEnergy(s.WorldId, s.bodyIds[i:i+1])
		total := linear + angular + potential

		percent := 0.0
		if s.startEnergy[i] != 0 {
			percent = 100 * total / s.startEnergy[i]
		}
		if i == 0 {
			minPercent, maxPercent = percent, percent
		} else {
			minPercent = math.Min(minPercent, percent)
			maxPercent = math.Max(maxPercent, percent)
		}

		s.Context.Draw.DrawStringFcn(b2.V2(x-0.5, rotatedBoxDropHeight+1.5), fmt.Sprintf("%.1f%%", percent), b2.ColorWhite)
		x += 2
	}

	s.DrawTextLine("energy: min %.2f%%, max %.2f%% of start", minPercent, maxPercent)
}

// TwoBoxRestitution drops one box onto another. It mirrors the
// TwoBoxRestitution sample, based on
// https://github.com/erincatto/box2d/discussions/957.
type TwoBoxRestitution struct {
	Base

	bodyIds     [2]b2.BodyId
	startEnergy float64
	peakEnergy  float64
}

// NewTwoBoxRestitution builds the two box restitution scene.
func NewTwoBoxRestitution(ctx *SampleContext) Sample {
	s := &TwoBoxRestitution{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 3}
		ctx.Camera.Zoom = 5
	}

	{
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(s.WorldId, &bodyDef)

		segment := b2.Segment{Point1: b2.V2(-10, 0), Point2: b2.V2(10, 0)}
		shapeDef := b2.DefaultShapeDef()
		b2.CreateSegmentShape(groundId, &shapeDef, &segment)
	}

	box := b2.MakeSquare(b2.QHalf())

	shapeDef := b2.DefaultShapeDef()
	shapeDef.Material.Restitution = FromFloat64(0.9)

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody

	bodyDef.Position.Y = b2.QHalf()
	s.bodyIds[0] = b2.CreateBody(s.WorldId, &bodyDef)
	b2.CreatePolygonShape(s.bodyIds[0], &shapeDef, &box)

	bodyDef.Position.X = bodyDef.Position.X.Add(b2.QFromRatio(1, 10))
	bodyDef.Position.Y = b2.F(6)
	s.bodyIds[1] = b2.CreateBody(s.WorldId, &bodyDef)
	b2.CreatePolygonShape(s.bodyIds[1], &shapeDef, &box)

	linear, angular, potential := restitutionComputeEnergy(s.WorldId, s.bodyIds[:])
	s.startEnergy = linear + angular + potential
	s.peakEnergy = s.startEnergy
	return s
}

func (s *TwoBoxRestitution) Step() {
	s.Base.Step()

	linear, angular, potential := restitutionComputeEnergy(s.WorldId, s.bodyIds[:])
	s.peakEnergy = s.restitutionDrawEnergy(linear, angular, potential, s.startEnergy, s.peakEnergy)
}

const circleRestitutionCount = 10

// CircleRestitution drops two rows of circles with increasing restitution. It
// mirrors the CircleRestitution sample.
type CircleRestitution struct {
	Base
}

// NewCircleRestitution builds the circle restitution scene.
func NewCircleRestitution(ctx *SampleContext) Sample {
	s := &CircleRestitution{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 5}
		ctx.Camera.Zoom = 10
	}

	{
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(s.WorldId, &bodyDef)

		h := b2.F(2 * circleRestitutionCount)
		segment := b2.Segment{Point1: b2.Vec2{X: h.Neg()}, Point2: b2.Vec2{X: h}}
		shapeDef := b2.DefaultShapeDef()
		b2.CreateSegmentShape(groundId, &shapeDef, &segment)
	}

	circle := b2.Circle{Radius: b2.QHalf()}

	shapeDef := b2.DefaultShapeDef()
	shapeDef.Density = b2.QOne()
	shapeDef.Material.Restitution = b2.QZero()

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody

	dr := b2.QFromRatio(1, max(circleRestitutionCount-1, 1))
	x := b2.QFromInt(-(circleRestitutionCount - 1))
	dx := b2.F(2)

	for range circleRestitutionCount {
		name := fmt.Sprintf("%.2f", ToFloat64(shapeDef.Material.Restitution))

		bodyDef.Position = b2.Vec2{X: x, Y: b2.QOne()}
		bodyDef.Name = name
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)
		b2.CreateCircleShape(bodyId, &shapeDef, &circle)

		bodyDef.Position = b2.Vec2{X: x, Y: b2.F(4)}
		bodyDef.Name = name
		bodyId = b2.CreateBody(s.WorldId, &bodyDef)
		b2.CreateCircleShape(bodyId, &shapeDef, &circle)

		shapeDef.Material.Restitution = shapeDef.Material.Restitution.Add(dr)
		x = x.Add(dx)
	}

	return s
}
