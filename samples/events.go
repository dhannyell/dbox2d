// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/sample_events.cpp of Box2D v3.1.1

package samples

import (
	"strings"

	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/dbox2d/internal/shared"
)

func init() {
	RegisterSample("Events", "Sensor Funnel", NewSensorFunnel)
	RegisterSample("Events", "Sensor Bookend", NewSensorBookend)
	RegisterSample("Events", "Foot Sensor", NewFootSensor)
	RegisterSample("Events", "Contact", NewContactEvent)
	RegisterSample("Events", "Platformer", NewPlatformer)
	RegisterSample("Events", "Body Move", NewBodyMove)
	RegisterSample("Events", "Sensor Types", NewSensorTypes)
}

// sampleAssert stands in for the assert of the reference samples.
func sampleAssert(condition bool, what string) {
	if !condition {
		panic("samples: " + what)
	}
}

// stepTime is one step of the sample clock, 1/hertz, the amount the
// reference subtracts from its countdowns.
func stepTime(settings *Settings) b2.Q {
	return b2.QOne().Div(b2.QFromInt(int(settings.Hertz)))
}

const (
	funnelDonut = 1
	funnelHuman = 2
	funnelCount = 32
)

// SensorFunnel drops donuts or humans through a funnel of spinning paddles
// and destroys them when they reach the sensor at the bottom.
type SensorFunnel struct {
	Base

	humans      [funnelCount]shared.Human
	donuts      [funnelCount]donut
	isSpawned   [funnelCount]bool
	elementType int
	wait        b2.Q
	side        b2.Q
}

func NewSensorFunnel(ctx *SampleContext) Sample {
	s := &SensorFunnel{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 0}
		ctx.Camera.Zoom = 25 * 1.333
	}

	ctx.Settings.DrawJoints = false

	{
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(s.WorldId, &bodyDef)

		points := []b2.Vec2{
			b2.V2(-16.8672504, 31.088623), b2.V2(16.8672485, 31.088623), b2.V2(16.8672485, 17.1978741),
			b2.V2(8.26824951, 11.906374), b2.V2(16.8672485, 11.906374), b2.V2(16.8672485, -0.661376953),
			b2.V2(8.26824951, -5.953125), b2.V2(16.8672485, -5.953125), b2.V2(16.8672485, -13.229126),
			b2.V2(3.63799858, -23.151123), b2.V2(3.63799858, -31.088623), b2.V2(-3.63800049, -31.088623),
			b2.V2(-3.63800049, -23.151123), b2.V2(-16.8672504, -13.229126), b2.V2(-16.8672504, -5.953125),
			b2.V2(-8.26825142, -5.953125), b2.V2(-16.8672504, -0.661376953), b2.V2(-16.8672504, 11.906374),
			b2.V2(-8.26825142, 11.906374), b2.V2(-16.8672504, 17.1978741),
		}

		material := b2.SurfaceMaterial{}
		material.Friction = b2.F(0.2)

		chainDef := b2.DefaultChainDef()
		chainDef.Points = points
		chainDef.IsLoop = true
		chainDef.Materials = []b2.SurfaceMaterial{material}
		b2.CreateChain(groundId, &chainDef)

		sign := b2.QOne()
		y := b2.F(14)
		for range 3 {
			bodyDef.Position = b2.Vec2{Y: y}
			bodyDef.Type = b2.DynamicBody

			bodyId := b2.CreateBody(s.WorldId, &bodyDef)

			box := b2.MakeBox(b2.F(6), b2.QHalf())
			shapeDef := b2.DefaultShapeDef()
			shapeDef.Material.Friction = b2.F(0.1)
			shapeDef.Material.Restitution = b2.QOne()
			shapeDef.Density = b2.QOne()

			b2.CreatePolygonShape(bodyId, &shapeDef, &box)

			revoluteDef := b2.DefaultRevoluteJointDef()
			revoluteDef.BodyIdA = groundId
			revoluteDef.BodyIdB = bodyId
			revoluteDef.LocalAnchorA = bodyDef.Position
			revoluteDef.LocalAnchorB = b2.Vec2{}
			revoluteDef.MaxMotorTorque = b2.F(200)
			revoluteDef.MotorSpeed = radiansQToTurns(b2.F(2).Mul(sign))
			revoluteDef.EnableMotor = true

			b2.CreateRevoluteJoint(s.WorldId, &revoluteDef)

			y = y.Sub(b2.F(14))
			sign = sign.Neg()
		}

		{
			box := b2.MakeOffsetBox(b2.F(4), b2.QOne(), b2.V2(0.0, -30.5), b2.RotIdentity())
			shapeDef := b2.DefaultShapeDef()
			shapeDef.IsSensor = true
			shapeDef.EnableSensorEvents = true

			b2.CreatePolygonShape(groundId, &shapeDef, &box)
		}
	}

	s.wait = b2.QHalf()
	s.side = b2.F(-15)
	s.elementType = funnelHuman

	s.createElement()
	return s
}

func (s *SensorFunnel) createElement() {
	index := -1
	for i := range funnelCount {
		if !s.isSpawned[i] {
			index = i
			break
		}
	}

	if index == -1 {
		return
	}

	center := b2.Vec2{X: s.side, Y: b2.F(29.5)}

	if s.elementType == funnelDonut {
		d := &s.donuts[index]
		d.create(s.WorldId, center, b2.QOne(), 0, true, d)
	} else {
		h := &s.humans[index]
		scale := b2.F(2)
		jointFriction := b2.F(0.05)
		jointHertz := b2.F(6)
		jointDamping := b2.QHalf()
		colorize := true
		*h = shared.CreateHuman(s.WorldId, center, scale, jointFriction, jointHertz, jointDamping, index+1, h, colorize)
		h.EnableSensorEvents(true)
	}

	s.isSpawned[index] = true
	s.side = s.side.Neg()
}

func (s *SensorFunnel) destroyElement(index int) {
	if s.elementType == funnelDonut {
		s.donuts[index].destroy()
	} else {
		s.humans[index].Destroy()
	}

	s.isSpawned[index] = false
}

func (s *SensorFunnel) clear() {
	for i := range funnelCount {
		if s.isSpawned[i] {
			if s.elementType == funnelDonut {
				s.donuts[i].destroy()
			} else {
				s.humans[i].Destroy()
			}

			s.isSpawned[i] = false
		}
	}
}

func (s *SensorFunnel) UpdateGui() {
	height := 90
	gui := s.Context.Gui
	gui.Begin("Sensor Event", 10, s.Context.Camera.Height-height-50, 140, height)

	if gui.RadioButton("donut", s.elementType == funnelDonut) {
		s.clear()
		s.elementType = funnelDonut
	}

	if gui.RadioButton("human", s.elementType == funnelHuman) {
		s.clear()
		s.elementType = funnelHuman
	}

	gui.End()
}

func (s *SensorFunnel) Step() {
	s.Base.Step()

	// Discover rings that touch the bottom sensor
	var deferredDestruction [funnelCount]bool
	sensorEvents := s.WorldId.GetSensorEvents()
	for _, event := range sensorEvents.BeginEvents {
		visitorId := event.VisitorShapeId
		bodyId := visitorId.GetBody()

		index := -1
		if s.elementType == funnelDonut {
			if d, ok := bodyId.GetUserData().(*donut); ok && d != nil {
				for i := range s.donuts {
					if d == &s.donuts[i] {
						index = i
						break
					}
				}
				sampleAssert(0 <= index && index < funnelCount, "sensor funnel donut is not in the pool")
			}
		} else {
			if h, ok := bodyId.GetUserData().(*shared.Human); ok && h != nil {
				for i := range s.humans {
					if h == &s.humans[i] {
						index = i
						break
					}
				}
				sampleAssert(0 <= index && index < funnelCount, "sensor funnel human is not in the pool")
			}
		}

		if index != -1 {
			// Defer destruction to avoid double destruction and event invalidation (orphaned shape ids)
			deferredDestruction[index] = true
		}
	}

	// todo destroy mouse joint if necessary

	// Safely destroy rings that hit the bottom sensor
	for i := range funnelCount {
		if deferredDestruction[i] {
			s.destroyElement(i)
		}
	}

	if s.Context.Settings.Hertz > 0 && !s.Context.Settings.Pause {
		s.wait = s.wait.Sub(stepTime(&s.Context.Settings))
		if s.wait.Less(b2.QZero()) {
			s.createElement()
			s.wait = s.wait.Add(b2.QHalf())
		}
	}
}

// SensorBookend creates and destroys sensors and visitors and checks that
// every begin touch event has a matching end touch event.
type SensorBookend struct {
	Base

	sensorBodyId1  b2.BodyId
	sensorShapeId1 b2.ShapeId

	sensorBodyId2  b2.BodyId
	sensorShapeId2 b2.ShapeId

	visitorBodyId  b2.BodyId
	visitorShapeId b2.ShapeId

	isVisiting1         bool
	isVisiting2         bool
	sensorsOverlapCount int
}

func NewSensorBookend(ctx *SampleContext) Sample {
	s := &SensorBookend{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 6}
		ctx.Camera.Zoom = 7.5
	}

	{
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(s.WorldId, &bodyDef)
		shapeDef := b2.DefaultShapeDef()

		groundSegment := b2.Segment{Point1: b2.V2(-10, 0), Point2: b2.V2(10, 0)}
		b2.CreateSegmentShape(groundId, &shapeDef, &groundSegment)

		groundSegment = b2.Segment{Point1: b2.V2(-10, 0), Point2: b2.V2(-10, 10)}
		b2.CreateSegmentShape(groundId, &shapeDef, &groundSegment)

		groundSegment = b2.Segment{Point1: b2.V2(10, 0), Point2: b2.V2(10, 10)}
		b2.CreateSegmentShape(groundId, &shapeDef, &groundSegment)

		s.isVisiting1 = false
		s.isVisiting2 = false
		s.sensorsOverlapCount = 0
	}

	s.createSensor1()
	s.createSensor2()
	s.createVisitor()
	return s
}

func (s *SensorBookend) createSensor1() {
	bodyDef := b2.DefaultBodyDef()

	bodyDef.Position = b2.V2(-2, 1)
	s.sensorBodyId1 = b2.CreateBody(s.WorldId, &bodyDef)

	shapeDef := b2.DefaultShapeDef()
	shapeDef.IsSensor = true
	shapeDef.EnableSensorEvents = true

	box := b2.MakeSquare(b2.QOne())
	s.sensorShapeId1 = b2.CreatePolygonShape(s.sensorBodyId1, &shapeDef, &box)
}

func (s *SensorBookend) createSensor2() {
	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.Position = b2.V2(2, 1)
	s.sensorBodyId2 = b2.CreateBody(s.WorldId, &bodyDef)

	shapeDef := b2.DefaultShapeDef()
	shapeDef.IsSensor = true
	shapeDef.EnableSensorEvents = true

	box := b2.MakeRoundedBox(b2.QHalf(), b2.QHalf(), b2.QHalf())
	s.sensorShapeId2 = b2.CreatePolygonShape(s.sensorBodyId2, &shapeDef, &box)

	// Solid middle
	shapeDef.IsSensor = false
	shapeDef.EnableSensorEvents = false
	box = b2.MakeSquare(b2.QHalf())
	b2.CreatePolygonShape(s.sensorBodyId2, &shapeDef, &box)
}

func (s *SensorBookend) createVisitor() {
	bodyDef := b2.DefaultBodyDef()
	bodyDef.Position = b2.V2(-4, 1)
	bodyDef.Type = b2.DynamicBody

	s.visitorBodyId = b2.CreateBody(s.WorldId, &bodyDef)

	shapeDef := b2.DefaultShapeDef()
	shapeDef.EnableSensorEvents = true

	circle := b2.Circle{Radius: b2.QHalf()}
	s.visitorShapeId = b2.CreateCircleShape(s.visitorBodyId, &shapeDef, &circle)
}

// bookendControls draws the create, destroy and enable controls of one
// body. The destroy branch keeps the shape id for the end events.
func bookendControls(gui Gui, bodyId *b2.BodyId, shapeId b2.ShapeId, create func(), createLabel, destroyLabel, eventsLabel, enableLabel string) {
	if bodyId.IsNull() {
		if gui.Button(createLabel) {
			create()
		}
		return
	}

	if gui.Button(destroyLabel) {
		b2.DestroyBody(*bodyId)
		*bodyId = b2.BodyId{}
		// Retain the shape id for end events.
		return
	}

	enabledEvents := shapeId.AreSensorEventsEnabled()
	if gui.Checkbox(eventsLabel, &enabledEvents) {
		shapeId.EnableSensorEvents(enabledEvents)
	}

	enabledBody := bodyId.IsEnabled()
	if gui.Checkbox(enableLabel, &enabledBody) {
		if enabledBody {
			bodyId.Enable()
		} else {
			bodyId.Disable()
		}
	}
}

func (s *SensorBookend) UpdateGui() {
	height := 260
	gui := s.Context.Gui
	gui.Begin("Sensor Bookend", 10, s.Context.Camera.Height-height-50, 200, height)

	bookendControls(gui, &s.visitorBodyId, s.visitorShapeId, s.createVisitor,
		"create visitor", "destroy visitor", "visitor events", "enable visitor body")

	bookendControls(gui, &s.sensorBodyId1, s.sensorShapeId1, s.createSensor1,
		"create sensor1", "destroy sensor1", "sensor 1 events", "enable sensor1 body")

	bookendControls(gui, &s.sensorBodyId2, s.sensorShapeId2, s.createSensor2,
		"create sensor2", "destroy sensor2", "sensor2 events", "enable sensor2 body")

	gui.End()
}

func (s *SensorBookend) Step() {
	s.Base.Step()

	sensorEvents := s.WorldId.GetSensorEvents()
	for _, event := range sensorEvents.BeginEvents {
		if event.SensorShapeId == s.sensorShapeId1 {
			if event.VisitorShapeId == s.visitorShapeId {
				sampleAssert(!s.isVisiting1, "sensor bookend visitor begins sensor 1 twice")
				s.isVisiting1 = true
			} else {
				sampleAssert(event.VisitorShapeId == s.sensorShapeId2, "sensor bookend unknown visitor of sensor 1")
				s.sensorsOverlapCount += 1
			}
		} else {
			sampleAssert(event.SensorShapeId == s.sensorShapeId2, "sensor bookend unknown sensor")

			if event.VisitorShapeId == s.visitorShapeId {
				sampleAssert(!s.isVisiting2, "sensor bookend visitor begins sensor 2 twice")
				s.isVisiting2 = true
			} else {
				sampleAssert(event.VisitorShapeId == s.sensorShapeId1, "sensor bookend unknown visitor of sensor 2")
				s.sensorsOverlapCount += 1
			}
		}
	}

	sampleAssert(s.sensorsOverlapCount == 0 || s.sensorsOverlapCount == 2, "sensor bookend overlap count")

	for _, event := range sensorEvents.EndEvents {
		if event.SensorShapeId == s.sensorShapeId1 {
			if event.VisitorShapeId == s.visitorShapeId {
				sampleAssert(s.isVisiting1, "sensor bookend visitor ends sensor 1 without a begin")
				s.isVisiting1 = false
			} else {
				sampleAssert(event.VisitorShapeId == s.sensorShapeId2, "sensor bookend unknown visitor of sensor 1")
				s.sensorsOverlapCount -= 1
			}
		} else {
			sampleAssert(event.SensorShapeId == s.sensorShapeId2, "sensor bookend unknown sensor")

			if event.VisitorShapeId == s.visitorShapeId {
				sampleAssert(s.isVisiting2, "sensor bookend visitor ends sensor 2 without a begin")
				s.isVisiting2 = false
			} else {
				sampleAssert(event.VisitorShapeId == s.sensorShapeId1, "sensor bookend unknown visitor of sensor 2")
				s.sensorsOverlapCount -= 1
			}
		}
	}

	sampleAssert(s.sensorsOverlapCount == 0 || s.sensorsOverlapCount == 2, "sensor bookend overlap count")

	// Nullify invalid shape ids after end events are processed.
	if !s.visitorShapeId.IsValid() {
		s.visitorShapeId = b2.ShapeId{}
	}

	if !s.sensorShapeId1.IsValid() {
		s.sensorShapeId1 = b2.ShapeId{}
	}

	if !s.sensorShapeId2.IsValid() {
		s.sensorShapeId2 = b2.ShapeId{}
	}

	s.DrawTextLine("visiting 1 == %t", s.isVisiting1)
	s.DrawTextLine("visiting 2 == %t", s.isVisiting2)
	s.DrawTextLine("sensors overlap count == %d", s.sensorsOverlapCount)
}

const (
	footGround uint64 = 0x00000001
	footPlayer uint64 = 0x00000002
	footFoot   uint64 = 0x00000004
)

// FootSensor counts the ground segments under the foot sensor of a player.
type FootSensor struct {
	Base

	playerId     b2.BodyId
	sensorId     b2.ShapeId
	overlaps     []b2.ShapeId
	overlapCount int
}

func NewFootSensor(ctx *SampleContext) Sample {
	s := &FootSensor{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 6}
		ctx.Camera.Zoom = 7.5
	}

	{
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(s.WorldId, &bodyDef)

		points := make([]b2.Vec2, 20)
		x := b2.F(10)
		for i := range 20 {
			points[i] = b2.Vec2{X: x}
			x = x.Sub(b2.QOne())
		}

		chainDef := b2.DefaultChainDef()
		chainDef.Points = points
		chainDef.Filter.CategoryBits = footGround
		chainDef.Filter.MaskBits = footFoot | footPlayer
		chainDef.IsLoop = false
		chainDef.EnableSensorEvents = true

		b2.CreateChain(groundId, &chainDef)
	}

	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.FixedRotation = true
		bodyDef.Position = b2.V2(0, 1)
		s.playerId = b2.CreateBody(s.WorldId, &bodyDef)
		shapeDef := b2.DefaultShapeDef()
		shapeDef.Filter.CategoryBits = footPlayer
		shapeDef.Filter.MaskBits = footGround
		shapeDef.Material.Friction = b2.F(0.3)
		capsule := b2.Capsule{Center1: b2.V2(0.0, -0.5), Center2: b2.V2(0.0, 0.5), Radius: b2.QHalf()}
		b2.CreateCapsuleShape(s.playerId, &shapeDef, &capsule)

		box := b2.MakeOffsetBox(b2.QHalf(), b2.F(0.25), b2.V2(0, -1), b2.RotIdentity())
		shapeDef.Filter.CategoryBits = footFoot
		shapeDef.Filter.MaskBits = footGround
		shapeDef.IsSensor = true
		shapeDef.EnableSensorEvents = true
		s.sensorId = b2.CreatePolygonShape(s.playerId, &shapeDef, &box)
	}

	s.overlapCount = 0
	return s
}

func (s *FootSensor) Step() {
	if s.keyDown(KeyA) {
		s.playerId.ApplyForceToCenter(b2.V2(-50, 0), true)
	}

	if s.keyDown(KeyD) {
		s.playerId.ApplyForceToCenter(b2.V2(50, 0), true)
	}

	s.Base.Step()

	sensorEvents := s.WorldId.GetSensorEvents()
	for _, event := range sensorEvents.BeginEvents {
		sampleAssert(event.VisitorShapeId != s.sensorId, "foot sensor visits itself")

		if event.SensorShapeId == s.sensorId {
			s.overlapCount += 1
		}
	}

	for _, event := range sensorEvents.EndEvents {
		sampleAssert(event.VisitorShapeId != s.sensorId, "foot sensor visits itself")

		if event.SensorShapeId == s.sensorId {
			s.overlapCount -= 1
		}
	}

	s.DrawTextLine("count == %d", s.overlapCount)

	capacity := s.sensorId.GetSensorCapacity()
	s.overlaps = append(s.overlaps[:0], make([]b2.ShapeId, capacity)...)
	count := s.sensorId.GetSensorOverlaps(s.overlaps)
	for i := range count {
		shapeId := s.overlaps[i]
		aabb := shapeId.GetAABB()
		point := b2.AABBCenter(aabb)
		s.Context.Draw.DrawPoint(point, b2.F(10), b2.ColorWhite)
	}
}

// contactUserData is the BodyUserData of the reference Contact sample.
type contactUserData struct {
	index int
}

const contactCount = 20

// ContactEvent lets a player collect debris. Debris that touches the player
// becomes part of it; a collected piece that touches the wall is lost.
type ContactEvent struct {
	Base

	playerId     b2.BodyId
	coreShapeId  b2.ShapeId
	debrisIds    [contactCount]b2.BodyId
	bodyUserData [contactCount]contactUserData
	force        float64
	wait         b2.Q
}

func NewContactEvent(ctx *SampleContext) Sample {
	s := &ContactEvent{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 0}
		ctx.Camera.Zoom = 25 * 1.75
	}

	{
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(s.WorldId, &bodyDef)

		points := []b2.Vec2{b2.V2(40, -40), b2.V2(-40, -40), b2.V2(-40, 40), b2.V2(40, 40)}

		chainDef := b2.DefaultChainDef()
		chainDef.Points = points
		chainDef.IsLoop = true

		b2.CreateChain(groundId, &chainDef)
	}

	// Player
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.GravityScale = b2.QZero()
		bodyDef.LinearDamping = b2.QHalf()
		bodyDef.AngularDamping = b2.QHalf()
		bodyDef.IsBullet = true
		s.playerId = b2.CreateBody(s.WorldId, &bodyDef)

		circle := b2.Circle{Radius: b2.QOne()}
		shapeDef := b2.DefaultShapeDef()

		// Enable contact events for the player shape
		shapeDef.EnableContactEvents = true

		s.coreShapeId = b2.CreateCircleShape(s.playerId, &shapeDef, &circle)
	}

	for i := range contactCount {
		s.debrisIds[i] = b2.BodyId{}
		s.bodyUserData[i].index = i
	}

	s.wait = b2.QHalf()
	s.force = 200
	return s
}

func (s *ContactEvent) spawnDebris() {
	index := -1
	for i := range contactCount {
		if s.debrisIds[i].IsNull() {
			index = i
			break
		}
	}

	if index == -1 {
		return
	}

	// Debris
	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.Position = b2.Vec2{
		X: shared.RandomFloatRange(b2.F(-38), b2.F(38)),
		Y: shared.RandomFloatRange(b2.F(-38), b2.F(38)),
	}
	// Turns; the reference range is -pi to pi radians.
	bodyDef.Rotation = b2.MakeRot(shared.RandomFloatRange(b2.QHalf().Neg(), b2.QHalf()))
	bodyDef.LinearVelocity = b2.Vec2{
		X: shared.RandomFloatRange(b2.F(-5), b2.F(5)),
		Y: shared.RandomFloatRange(b2.F(-5), b2.F(5)),
	}
	bodyDef.AngularVelocity = radiansQToTurns(shared.RandomFloatRange(b2.F(-1), b2.F(1)))
	bodyDef.GravityScale = b2.QZero()
	bodyDef.UserData = &s.bodyUserData[index]
	s.debrisIds[index] = b2.CreateBody(s.WorldId, &bodyDef)

	shapeDef := b2.DefaultShapeDef()
	shapeDef.Material.Restitution = b2.F(0.8)

	// No events when debris hits debris
	shapeDef.EnableContactEvents = false

	if (index+1)%3 == 0 {
		circle := b2.Circle{Radius: b2.QHalf()}
		b2.CreateCircleShape(s.debrisIds[index], &shapeDef, &circle)
	} else if (index+1)%2 == 0 {
		capsule := b2.Capsule{Center1: b2.V2(0.0, -0.25), Center2: b2.V2(0.0, 0.25), Radius: b2.F(0.25)}
		b2.CreateCapsuleShape(s.debrisIds[index], &shapeDef, &capsule)
	} else {
		box := b2.MakeBox(b2.F(0.4), b2.F(0.6))
		b2.CreatePolygonShape(s.debrisIds[index], &shapeDef, &box)
	}
}

func (s *ContactEvent) UpdateGui() {
	height := 60
	gui := s.Context.Gui
	gui.Begin("Contact Event", 10, s.Context.Camera.Height-height-50, 240, height)

	gui.SliderFloat("force", &s.force, 100, 500)

	gui.End()
}

// drawContactImpulses draws the final impulses of the contact between the
// two shapes of a begin event, read from the contact data of shapeId.
func (s *ContactEvent) drawContactImpulses(shapeId, otherId, pairId b2.ShapeId, color b2.HexColor) {
	contactData := make([]b2.ContactData, shapeId.GetContactCapacity())

	// The count may be less than the capacity
	count := shapeId.GetContactData(contactData)
	sampleAssert(count >= 1, "contact begin event without contact data")

	for j := range count {
		idA := contactData[j].ShapeIdA
		idB := contactData[j].ShapeIdB
		if idA == otherId || idB == otherId {
			sampleAssert(idA == pairId || idB == pairId, "contact data does not hold the event pair")

			manifold := contactData[j].Manifold
			normal := manifold.Normal

			for k := range manifold.PointCount {
				point := manifold.Points[k]
				s.Context.Draw.DrawSegment(point.Point, b2.MulAdd(point.Point, point.TotalNormalImpulse, normal), color)
				s.Context.Draw.DrawPoint(point.Point, b2.F(10), b2.ColorWhite)
			}
		}
	}
}

func (s *ContactEvent) Step() {
	s.DrawTextLine("move using WASD")

	position := s.playerId.GetPosition()
	force := FromFloat64(s.force)

	if s.keyDown(KeyA) {
		s.playerId.ApplyForce(b2.Vec2{X: force.Neg()}, position, true)
	}

	if s.keyDown(KeyD) {
		s.playerId.ApplyForce(b2.Vec2{X: force}, position, true)
	}

	if s.keyDown(KeyW) {
		s.playerId.ApplyForce(b2.Vec2{Y: force}, position, true)
	}

	if s.keyDown(KeyS) {
		s.playerId.ApplyForce(b2.Vec2{Y: force.Neg()}, position, true)
	}

	s.Base.Step()

	// Discover rings that touch the bottom sensor
	var debrisToAttach [contactCount]int
	var shapesToDestroy [contactCount]b2.ShapeId
	attachCount := 0
	destroyCount := 0

	// Process contact begin touch events.
	contactEvents := s.WorldId.GetContactEvents()
	for _, event := range contactEvents.BeginEvents {
		bodyIdA := event.ShapeIdA.GetBody()
		bodyIdB := event.ShapeIdB.GetBody()

		// The begin touch events have the contact manifolds, but the impulses are zero. This is because the manifolds
		// are gathered before the contact solver is run.

		// We can get the final contact data from the shapes. The manifold is shared by the two shapes, so we just need the
		// contact data from one of the shapes. Choose the one with the smallest number of contacts.

		capacityA := event.ShapeIdA.GetContactCapacity()
		capacityB := event.ShapeIdB.GetContactCapacity()

		if capacityA < capacityB {
			s.drawContactImpulses(event.ShapeIdA, event.ShapeIdB, event.ShapeIdA, b2.ColorBlueViolet)
		} else {
			s.drawContactImpulses(event.ShapeIdB, event.ShapeIdA, event.ShapeIdB, b2.ColorYellowGreen)
		}

		// The player shape and the other body of the event.
		playerShapeId, otherBodyId := event.ShapeIdA, bodyIdB
		if bodyIdA != s.playerId {
			// Only expect events for the player
			sampleAssert(bodyIdB == s.playerId, "contact event without the player")
			playerShapeId, otherBodyId = event.ShapeIdB, bodyIdA
		}

		userData, _ := otherBodyId.GetUserData().(*contactUserData)
		if userData == nil {
			if playerShapeId != s.coreShapeId && destroyCount < contactCount {
				// player non-core shape hit the wall

				found := false
				for j := range destroyCount {
					if playerShapeId == shapesToDestroy[j] {
						found = true
						break
					}
				}

				// avoid double deletion
				if !found {
					shapesToDestroy[destroyCount] = playerShapeId
					destroyCount += 1
				}
			}
		} else if attachCount < contactCount {
			debrisToAttach[attachCount] = userData.index
			attachCount += 1
		}
	}

	// Attach debris to player body
	for i := range attachCount {
		index := debrisToAttach[i]
		debrisId := s.debrisIds[index]
		if debrisId.IsNull() {
			continue
		}

		playerTransform := s.playerId.GetTransform()
		debrisTransform := debrisId.GetTransform()
		relativeTransform := b2.InvMulTransforms(playerTransform, debrisTransform)

		shapeCount := debrisId.GetShapeCount()
		if shapeCount == 0 {
			continue
		}

		var shapeIds [1]b2.ShapeId
		debrisId.GetShapes(shapeIds[:])
		shapeId := shapeIds[0]

		shapeDef := b2.DefaultShapeDef()
		shapeDef.EnableContactEvents = true

		switch shapeId.GetType() {
		case b2.CircleShape:
			circle := shapeId.GetCircle()
			circle.Center = b2.TransformPoint(relativeTransform, circle.Center)

			b2.CreateCircleShape(s.playerId, &shapeDef, &circle)

		case b2.CapsuleShape:
			capsule := shapeId.GetCapsule()
			capsule.Center1 = b2.TransformPoint(relativeTransform, capsule.Center1)
			capsule.Center2 = b2.TransformPoint(relativeTransform, capsule.Center2)

			b2.CreateCapsuleShape(s.playerId, &shapeDef, &capsule)

		case b2.PolygonShape:
			originalPolygon := shapeId.GetPolygon()
			polygon := b2.TransformPolygon(relativeTransform, &originalPolygon)

			b2.CreatePolygonShape(s.playerId, &shapeDef, &polygon)

		default:
			sampleAssert(false, "contact debris has an unexpected shape type")
		}

		b2.DestroyBody(debrisId)
		s.debrisIds[index] = b2.BodyId{}
	}

	for i := range destroyCount {
		updateMass := false
		b2.DestroyShape(shapesToDestroy[i], updateMass)
	}

	if destroyCount > 0 {
		// Update mass just once
		s.playerId.ApplyMassFromShapes()
	}

	if s.Context.Settings.Hertz > 0 && !s.Context.Settings.Pause {
		s.wait = s.wait.Sub(stepTime(&s.Context.Settings))
		if s.wait.Less(b2.QZero()) {
			s.spawnDebris()
			s.wait = s.wait.Add(b2.QHalf())
		}
	}
}

// Platformer shows how to make a rigid body character mover and use the
// pre-solve callback. In this case the platform should get the pre-solve
// event, not the player.
type Platformer struct {
	Base

	jumping          bool
	radius           b2.Q
	force            float64
	impulse          float64
	jumpDelay        b2.Q
	playerId         b2.BodyId
	playerShapeId    b2.ShapeId
	movingPlatformId b2.BodyId
}

func NewPlatformer(ctx *SampleContext) Sample {
	s := &Platformer{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0.5, Y: 7.5}
		ctx.Camera.Zoom = 25 * 0.4
	}

	s.WorldId.SetPreSolveCallback(s.preSolve)

	// Ground
	{
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(s.WorldId, &bodyDef)
		shapeDef := b2.DefaultShapeDef()
		segment := b2.Segment{Point1: b2.V2(-20, 0), Point2: b2.V2(20, 0)}
		b2.CreateSegmentShape(groundId, &shapeDef, &segment)
	}

	// Static Platform
	// This tests pre-solve with continuous collision
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.StaticBody
		bodyDef.Position = b2.V2(-6, 6)
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()

		// Need to turn this on to get the callback
		shapeDef.EnablePreSolveEvents = true

		box := b2.MakeBox(b2.F(2), b2.QHalf())
		b2.CreatePolygonShape(bodyId, &shapeDef, &box)
	}

	// Moving Platform
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.KinematicBody
		bodyDef.Position = b2.V2(0, 6)
		bodyDef.LinearVelocity = b2.V2(2, 0)
		s.movingPlatformId = b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()

		// Need to turn this on to get the callback
		shapeDef.EnablePreSolveEvents = true

		box := b2.MakeBox(b2.F(3), b2.QHalf())
		b2.CreatePolygonShape(s.movingPlatformId, &shapeDef, &box)
	}

	// Player
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.FixedRotation = true
		bodyDef.LinearDamping = b2.QHalf()
		bodyDef.Position = b2.V2(0, 1)
		s.playerId = b2.CreateBody(s.WorldId, &bodyDef)

		s.radius = b2.QHalf()
		capsule := b2.Capsule{Center1: b2.V2(0, 0), Center2: b2.V2(0, 1), Radius: s.radius}
		shapeDef := b2.DefaultShapeDef()
		shapeDef.Material.Friction = b2.F(0.1)

		s.playerShapeId = b2.CreateCapsuleShape(s.playerId, &shapeDef, &capsule)
	}

	s.force = 25
	s.impulse = 25
	s.jumpDelay = b2.F(0.25)
	s.jumping = false
	return s
}

// preSolve must be thread-safe. It may be called multiple times
// simultaneously. It reads only fields that stay fixed after construction
// and does not try to access any values in the world that may be changing,
// such as contact data.
func (s *Platformer) preSolve(shapeIdA, shapeIdB b2.ShapeId, manifold *b2.Manifold) bool {
	sampleAssert(shapeIdA.IsValid(), "platformer pre-solve shape A is invalid")
	sampleAssert(shapeIdB.IsValid(), "platformer pre-solve shape B is invalid")

	var sign b2.Q
	if shapeIdA == s.playerShapeId {
		sign = b2.QOne().Neg()
	} else if shapeIdB == s.playerShapeId {
		sign = b2.QOne()
	} else {
		// not colliding with the player, enable contact
		return true
	}

	normal := manifold.Normal
	if sign.Mul(normal.Y).Greater(b2.F(0.95)) {
		return true
	}

	separation := b2.QZero()
	for i := range manifold.PointCount {
		sep := manifold.Points[i].Separation
		if sep.Less(separation) {
			separation = sep
		}
	}

	if separation.Greater(b2.F(0.1).Mul(s.radius)) {
		// shallow overlap
		return true
	}

	// normal points down, disable contact
	return false
}

func (s *Platformer) UpdateGui() {
	height := 100
	gui := s.Context.Gui
	gui.Begin("One-Sided Platform", 10, s.Context.Camera.Height-height-50, 240, height)

	gui.SliderFloat("force", &s.force, 0, 50)
	gui.SliderFloat("impulse", &s.impulse, 0, 50)

	gui.End()
}

func (s *Platformer) Step() {
	canJump := false
	velocity := s.playerId.GetLinearVelocity()
	if s.jumpDelay.Eq(b2.QZero()) && !s.jumping && velocity.Y.Less(b2.F(0.01)) {
		capacity := min(s.playerId.GetContactCapacity(), 4)
		var contactData [4]b2.ContactData
		count := s.playerId.GetContactData(contactData[:capacity])
		for i := range count {
			bodyIdA := contactData[i].ShapeIdA.GetBody()
			var sign b2.Q
			if bodyIdA == s.playerId {
				// normal points from A to B
				sign = b2.QOne().Neg()
			} else {
				sign = b2.QOne()
			}

			if sign.Mul(contactData[i].Manifold.Normal.Y).Greater(b2.F(0.9)) {
				canJump = true
				break
			}
		}
	}

	// A kinematic body is moved by setting its velocity. This
	// ensure friction works correctly.
	platformPosition := s.movingPlatformId.GetPosition()
	if platformPosition.X.Less(b2.F(-15)) {
		s.movingPlatformId.SetLinearVelocity(b2.V2(2, 0))
	} else if platformPosition.X.Greater(b2.F(15)) {
		s.movingPlatformId.SetLinearVelocity(b2.V2(-2, 0))
	}

	if s.keyDown(KeyA) {
		s.playerId.ApplyForceToCenter(b2.Vec2{X: FromFloat64(-s.force)}, true)
	}

	if s.keyDown(KeyD) {
		s.playerId.ApplyForceToCenter(b2.Vec2{X: FromFloat64(s.force)}, true)
	}

	if s.keyDown(KeySpace) {
		if canJump {
			s.playerId.ApplyLinearImpulseToCenter(b2.Vec2{Y: FromFloat64(s.impulse)}, true)
			s.jumpDelay = b2.QHalf()
			s.jumping = true
		}
	} else {
		s.jumping = false
	}

	s.Base.Step()

	var contactData [1]b2.ContactData
	contactCount := s.movingPlatformId.GetContactData(contactData[:])
	s.DrawTextLine("Platform contact count = %d, point count = %d", contactCount, contactData[0].Manifold.PointCount)
	s.DrawTextLine("Movement: A/D/Space")
	s.DrawTextLine("Can jump = %t", canJump)

	if s.Context.Settings.Hertz > 0 {
		s.jumpDelay = s.jumpDelay.Sub(stepTime(&s.Context.Settings))
		if s.jumpDelay.Less(b2.QZero()) {
			s.jumpDelay = b2.QZero()
		}
	}
}

const bodyMoveCount = 50

// BodyMove shows how to process body events.
type BodyMove struct {
	Base

	bodyIds            [bodyMoveCount]b2.BodyId
	sleeping           [bodyMoveCount]bool
	count              int
	sleepCount         int
	explosionPosition  b2.Vec2
	explosionRadius    b2.Q
	explosionMagnitude float64
}

func NewBodyMove(ctx *SampleContext) Sample {
	s := &BodyMove{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 2, Y: 8}
		ctx.Camera.Zoom = 25 * 0.55
	}

	{
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		shapeDef.Material.Friction = b2.F(0.1)

		// Turns; the reference rotations are -0.15 pi and 0.15 pi.
		box := b2.MakeOffsetBox(b2.F(12), b2.F(0.1), b2.V2(-10.0, -0.1), b2.MakeRot(b2.F(-0.075)))
		b2.CreatePolygonShape(groundId, &shapeDef, &box)

		box = b2.MakeOffsetBox(b2.F(12), b2.F(0.1), b2.V2(10.0, -0.1), b2.MakeRot(b2.F(0.075)))
		b2.CreatePolygonShape(groundId, &shapeDef, &box)

		shapeDef.Material.Restitution = b2.F(0.8)

		box = b2.MakeOffsetBox(b2.F(0.1), b2.F(10), b2.V2(19.9, 10.0), b2.RotIdentity())
		b2.CreatePolygonShape(groundId, &shapeDef, &box)

		box = b2.MakeOffsetBox(b2.F(0.1), b2.F(10), b2.V2(-19.9, 10.0), b2.RotIdentity())
		b2.CreatePolygonShape(groundId, &shapeDef, &box)

		box = b2.MakeOffsetBox(b2.F(20), b2.F(0.1), b2.V2(0.0, 20.1), b2.RotIdentity())
		b2.CreatePolygonShape(groundId, &shapeDef, &box)
	}

	s.sleepCount = 0
	s.count = 0

	s.explosionPosition = b2.V2(0, -5)
	s.explosionRadius = b2.F(10)
	s.explosionMagnitude = 10
	return s
}

func (s *BodyMove) createBodies() {
	capsule := b2.Capsule{Center1: b2.V2(-0.25, 0.0), Center2: b2.V2(0.25, 0.0), Radius: b2.F(0.25)}
	circle := b2.Circle{Radius: b2.F(0.35)}
	square := b2.MakeSquare(b2.F(0.35))

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	shapeDef := b2.DefaultShapeDef()

	x, y := b2.F(-5), b2.F(10)
	for i := 0; i < 10 && s.count < bodyMoveCount; i++ {
		bodyDef.Position = b2.Vec2{X: x, Y: y}
		bodyDef.IsBullet = s.count%12 == 0
		// The reference points the user data at the body id slot; the
		// index is the Go form of that pointer.
		bodyDef.UserData = s.count
		s.bodyIds[s.count] = b2.CreateBody(s.WorldId, &bodyDef)
		s.sleeping[s.count] = false

		remainder := s.count % 4
		if remainder == 0 {
			b2.CreateCapsuleShape(s.bodyIds[s.count], &shapeDef, &capsule)
		} else if remainder == 1 {
			b2.CreateCircleShape(s.bodyIds[s.count], &shapeDef, &circle)
		} else if remainder == 2 {
			b2.CreatePolygonShape(s.bodyIds[s.count], &shapeDef, &square)
		} else {
			poly := shared.RandomPolygon(b2.F(0.75))
			poly.Radius = b2.F(0.1)
			b2.CreatePolygonShape(s.bodyIds[s.count], &shapeDef, &poly)
		}

		s.count += 1
		x = x.Add(b2.QOne())
	}
}

func (s *BodyMove) UpdateGui() {
	height := 100
	gui := s.Context.Gui
	gui.Begin("Body Move", 10, s.Context.Camera.Height-height-50, 240, height)

	if gui.Button("Explode") {
		def := b2.DefaultExplosionDef()
		def.Position = s.explosionPosition
		def.Radius = s.explosionRadius
		def.Falloff = b2.F(0.1)
		def.ImpulsePerLength = FromFloat64(s.explosionMagnitude)
		s.WorldId.Explode(&def)
	}

	gui.SliderFloat("Magnitude", &s.explosionMagnitude, -20, 20)

	gui.End()
}

func (s *BodyMove) Step() {
	if !s.Context.Settings.Pause && s.StepCount&15 == 15 && s.count < bodyMoveCount {
		s.createBodies()
	}

	s.Base.Step()

	// Process body events
	events := s.WorldId.GetBodyEvents()
	for i := range events.MoveEvents {
		// draw the transform of every body that moved (not sleeping)
		event := &events.MoveEvents[i]
		s.Context.Draw.DrawTransform(event.Transform)

		transform := event.BodyId.GetTransform()
		sampleAssert(transform == event.Transform, "body move event transform differs from the body")

		// this shows a somewhat contrived way to track body sleeping
		index := event.UserData.(int)
		sleeping := &s.sleeping[index]

		if event.FellAsleep {
			*sleeping = true
			s.sleepCount += 1
		} else {
			if *sleeping {
				*sleeping = false
				s.sleepCount -= 1
			}
		}
	}

	s.Context.Draw.DrawCircle(s.explosionPosition, s.explosionRadius, b2.ColorAzure)

	s.DrawTextLine("sleep count: %d", s.sleepCount)
}

const (
	sensorTypesGround  uint64 = 0x00000001
	sensorTypesSensor  uint64 = 0x00000002
	sensorTypesDefault uint64 = 0x00000004
)

// SensorTypes shows the overlaps of a static, a kinematic and a dynamic
// sensor.
type SensorTypes struct {
	Base

	staticSensorId    b2.ShapeId
	kinematicSensorId b2.ShapeId
	dynamicSensorId   b2.ShapeId

	kinematicBodyId b2.BodyId

	overlaps []b2.ShapeId
}

func NewSensorTypes(ctx *SampleContext) Sample {
	s := &SensorTypes{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 3}
		ctx.Camera.Zoom = 4.5
	}

	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Name = "ground"

		groundId := b2.CreateBody(s.WorldId, &bodyDef)
		shapeDef := b2.DefaultShapeDef()

		// Enable sensor events, but filter them out as a test
		shapeDef.Filter.CategoryBits = sensorTypesGround
		shapeDef.Filter.MaskBits = sensorTypesDefault
		shapeDef.EnableSensorEvents = true

		groundSegment := b2.Segment{Point1: b2.V2(-6, 0), Point2: b2.V2(6, 0)}
		b2.CreateSegmentShape(groundId, &shapeDef, &groundSegment)

		groundSegment = b2.Segment{Point1: b2.V2(-6, 0), Point2: b2.V2(-6, 4)}
		b2.CreateSegmentShape(groundId, &shapeDef, &groundSegment)

		groundSegment = b2.Segment{Point1: b2.V2(6, 0), Point2: b2.V2(6, 4)}
		b2.CreateSegmentShape(groundId, &shapeDef, &groundSegment)
	}

	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Name = "static sensor"
		bodyDef.Type = b2.StaticBody
		bodyDef.Position = b2.V2(-3.0, 0.8)
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		shapeDef.Filter.CategoryBits = sensorTypesSensor
		shapeDef.IsSensor = true
		shapeDef.EnableSensorEvents = true
		box := b2.MakeSquare(b2.QOne())
		s.staticSensorId = b2.CreatePolygonShape(bodyId, &shapeDef, &box)
	}

	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Name = "kinematic sensor"
		bodyDef.Type = b2.KinematicBody
		bodyDef.Position = b2.V2(0, 0)
		bodyDef.LinearVelocity = b2.V2(0, 1)
		s.kinematicBodyId = b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		shapeDef.Filter.CategoryBits = sensorTypesSensor
		shapeDef.IsSensor = true
		shapeDef.EnableSensorEvents = true
		box := b2.MakeSquare(b2.QOne())
		s.kinematicSensorId = b2.CreatePolygonShape(s.kinematicBodyId, &shapeDef, &box)
	}

	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Name = "dynamic sensor"
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.V2(3, 1)
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		shapeDef.Filter.CategoryBits = sensorTypesSensor
		shapeDef.IsSensor = true
		shapeDef.EnableSensorEvents = true
		box := b2.MakeSquare(b2.QOne())
		s.dynamicSensorId = b2.CreatePolygonShape(bodyId, &shapeDef, &box)

		// Add some real collision so the dynamic body is valid
		shapeDef.Filter.CategoryBits = sensorTypesDefault
		shapeDef.IsSensor = false
		shapeDef.EnableSensorEvents = false
		box = b2.MakeSquare(b2.F(0.8))
		b2.CreatePolygonShape(bodyId, &shapeDef, &box)
	}

	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Name = "ball_01"
		bodyDef.Position = b2.V2(-5, 1)
		bodyDef.Type = b2.DynamicBody

		bodyId := b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		shapeDef.Filter.CategoryBits = sensorTypesDefault
		shapeDef.Filter.MaskBits = sensorTypesGround | sensorTypesDefault | sensorTypesSensor
		shapeDef.EnableSensorEvents = true

		circle := b2.Circle{Radius: b2.QHalf()}
		b2.CreateCircleShape(bodyId, &shapeDef, &circle)
	}
	return s
}

func (s *SensorTypes) printOverlaps(sensorShapeId b2.ShapeId, prefix string) {
	// Determine the necessary capacity
	capacity := sensorShapeId.GetSensorCapacity()
	s.overlaps = append(s.overlaps[:0], make([]b2.ShapeId, capacity)...)

	// Get all overlaps and record the actual count
	count := sensorShapeId.GetSensorOverlaps(s.overlaps)
	s.overlaps = s.overlaps[:count]

	var buffer strings.Builder
	buffer.WriteString(prefix)
	buffer.WriteString(": ")
	for _, visitorId := range s.overlaps {
		if !visitorId.IsValid() {
			continue
		}

		bodyId := visitorId.GetBody()
		buffer.WriteString(bodyId.GetName())
		buffer.WriteString(", ")
	}

	s.DrawTextLine("%s", buffer.String())
}

func (s *SensorTypes) Step() {
	position := s.kinematicBodyId.GetPosition()
	if position.Y.Less(b2.QZero()) {
		s.kinematicBodyId.SetLinearVelocity(b2.V2(0, 1))
	} else if position.Y.Greater(b2.F(3)) {
		s.kinematicBodyId.SetLinearVelocity(b2.V2(0, -1))
	}

	s.Base.Step()

	s.printOverlaps(s.staticSensorId, "static")
	s.printOverlaps(s.kinematicSensorId, "kinematic")
	s.printOverlaps(s.dynamicSensorId, "dynamic")

	origin := b2.V2(5, 1)
	translation := b2.V2(-10, 0)
	result := s.WorldId.CastRayClosest(origin, translation, b2.DefaultQueryFilter())
	s.Context.Draw.DrawSegment(origin, origin.Add(translation), b2.ColorDimGray)

	if result.Hit {
		s.Context.Draw.DrawPoint(result.Point, b2.F(10), b2.ColorCyan)
	}
}
