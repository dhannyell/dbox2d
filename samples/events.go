// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/sample_events.cpp of Box2D v3.2.0

package samples

import (
	"fmt"
	"strings"

	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/dbox2d/internal/shared"
)

func init() {
	RegisterSample("Events", "Sensor Funnel", NewSensorFunnel)
	RegisterSample("Events", "Sensor Bookend", NewSensorBookend)
	RegisterSample("Events", "Foot Sensor", NewFootSensor)
	RegisterSample("Events", "Contact", NewContactEvent)
	RegisterSample("Events", "Body Move", NewBodyMove)
	RegisterSample("Events", "Sensor Types", NewSensorTypes)
	RegisterSample("Events", "Joint", NewJointEvent)
	RegisterSample("Events", "Persistent Contact", NewPersistentContact)
	RegisterSample("Events", "Sensor Hits", NewSensorHits)
	RegisterSample("Events", "Projectile Event", NewProjectileEvent)
	RegisterSample("Events", "Circle Impulse", NewCircleImpulse)
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
			revoluteDef.Base.BodyIdA = groundId
			revoluteDef.Base.BodyIdB = bodyId
			revoluteDef.Base.LocalFrameA.P = bodyDef.Position
			revoluteDef.Base.LocalFrameB.P = b2.Vec2{}
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
		chainDef.Points = points[1:19]
		chainDef.Ghost1, chainDef.Ghost2 = points[0], points[19]
		chainDef.Filter.CategoryBits = footGround
		chainDef.Filter.MaskBits = footFoot | footPlayer
		chainDef.IsLoop = false
		chainDef.EnableSensorEvents = true

		b2.CreateChain(groundId, &chainDef)
	}

	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.MotionLocks.AngularZ = true
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
	count := s.sensorId.GetSensorData(s.overlaps)
	for i := range count {
		shapeId := s.overlaps[i]
		aabb := shapeId.GetBounds()
		point := b2.AABBCenter(aabb)
		s.Context.Draw.DrawPointFcn(point, b2.F(10), b2.ColorWhite)
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

			centerA := idA.GetBody().GetWorldCenter()
			for k := range manifold.PointCount {
				point := manifold.Points[k]
				p1 := centerA.Add(point.AnchorA)
				s.Context.Draw.DrawLineFcn(p1, b2.MulAdd(p1, point.TotalNormalImpulse, normal), color)
				s.Context.Draw.DrawPointFcn(p1, b2.F(10), b2.ColorWhite)
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
		s.playerId.UpdateMassFromShapes()
	}

	if s.Context.Settings.Hertz > 0 && !s.Context.Settings.Pause {
		s.wait = s.wait.Sub(stepTime(&s.Context.Settings))
		if s.wait.Less(b2.QZero()) {
			s.spawnDebris()
			s.wait = s.wait.Add(b2.QHalf())
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
		if event.UserData == nil {
			// The mouse body has no user data
			continue
		}

		s.Context.Draw.DrawTransformFcn(event.Transform)

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

	s.Context.Draw.DrawCircleFcn(s.explosionPosition, s.explosionRadius, b2.ColorAzure)

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
	count := sensorShapeId.GetSensorData(s.overlaps)
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
	s.Context.Draw.DrawLineFcn(origin, origin.Add(translation), b2.ColorDimGray)

	if result.Hit {
		s.Context.Draw.DrawPointFcn(result.Point, b2.F(10), b2.ColorCyan)
	}
}

const jointEventCount = 6

// JointEvent mirrors the Joint sample of sample_events.cpp. It breaks joints
// when the internal reaction force becomes large, using joint events instead
// of polling.
type JointEvent struct {
	Base

	jointIds [jointEventCount]b2.JointId
}

// NewJointEvent builds the joint event scene.
func NewJointEvent(ctx *SampleContext) Sample {
	s := &JointEvent{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 8}
		ctx.Camera.Zoom = 25 * 0.7
	}

	bodyDef := b2.DefaultBodyDef()
	groundId := b2.CreateBody(s.WorldId, &bodyDef)

	shapeDef := b2.DefaultShapeDef()
	segment := b2.Segment{Point1: b2.V2(-40, 0), Point2: b2.V2(40, 0)}
	b2.CreateSegmentShape(groundId, &shapeDef, &segment)

	position := b2.V2(-12.5, 10)
	bodyDef.Type = b2.DynamicBody
	bodyDef.EnableSleep = false

	box := b2.MakeBox(b2.QOne(), b2.QOne())

	index := 0
	forceThreshold := b2.F(20000)
	torqueThreshold := b2.F(10000)

	// Each joint gets a fresh body and the shared thresholds.
	newBody := func() b2.BodyId {
		bodyDef.Position = position
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)
		b2.CreatePolygonShape(bodyId, &shapeDef, &box)
		return bodyId
	}
	setup := func(base *b2.JointDef, bodyId b2.BodyId) {
		base.BodyIdA = groundId
		base.BodyIdB = bodyId
		base.ForceThreshold = forceThreshold
		base.TorqueThreshold = torqueThreshold
		base.CollideConnected = true
		base.UserData = index
	}
	pivotFrames := func(base *b2.JointDef, bodyId b2.BodyId) {
		pivot := b2.Vec2{X: position.X.Sub(b2.QOne()), Y: position.Y}
		base.LocalFrameA.P = groundId.GetLocalPoint(pivot)
		base.LocalFrameB.P = bodyId.GetLocalPoint(pivot)
	}

	// distance joint
	{
		bodyId := newBody()
		length := b2.F(2)
		pivot1 := b2.Vec2{X: position.X, Y: position.Y.Add(b2.QOne()).Add(length)}
		pivot2 := b2.Vec2{X: position.X, Y: position.Y.Add(b2.QOne())}
		jointDef := b2.DefaultDistanceJointDef()
		setup(&jointDef.Base, bodyId)
		jointDef.Base.LocalFrameA.P = groundId.GetLocalPoint(pivot1)
		jointDef.Base.LocalFrameB.P = bodyId.GetLocalPoint(pivot2)
		jointDef.Length = length
		s.jointIds[index] = b2.CreateDistanceJoint(s.WorldId, &jointDef)
	}
	position.X = position.X.Add(b2.F(5))
	index++

	// motor joint
	{
		bodyId := newBody()
		jointDef := b2.DefaultMotorJointDef()
		setup(&jointDef.Base, bodyId)
		jointDef.Base.LocalFrameA.P = position
		jointDef.MaxVelocityForce = b2.F(1000)
		jointDef.MaxVelocityTorque = b2.F(20)
		s.jointIds[index] = b2.CreateMotorJoint(s.WorldId, &jointDef)
	}
	position.X = position.X.Add(b2.F(5))
	index++

	// prismatic joint
	{
		bodyId := newBody()
		jointDef := b2.DefaultPrismaticJointDef()
		setup(&jointDef.Base, bodyId)
		pivotFrames(&jointDef.Base, bodyId)
		s.jointIds[index] = b2.CreatePrismaticJoint(s.WorldId, &jointDef)
	}
	position.X = position.X.Add(b2.F(5))
	index++

	// revolute joint
	{
		bodyId := newBody()
		jointDef := b2.DefaultRevoluteJointDef()
		setup(&jointDef.Base, bodyId)
		pivotFrames(&jointDef.Base, bodyId)
		s.jointIds[index] = b2.CreateRevoluteJoint(s.WorldId, &jointDef)
	}
	position.X = position.X.Add(b2.F(5))
	index++

	// weld joint
	{
		bodyId := newBody()
		jointDef := b2.DefaultWeldJointDef()
		setup(&jointDef.Base, bodyId)
		pivotFrames(&jointDef.Base, bodyId)
		jointDef.AngularHertz = b2.F(2)
		jointDef.AngularDampingRatio = b2.F(0.5)
		s.jointIds[index] = b2.CreateWeldJoint(s.WorldId, &jointDef)
	}
	position.X = position.X.Add(b2.F(5))
	index++

	// wheel joint
	{
		bodyId := newBody()
		jointDef := b2.DefaultWheelJointDef()
		setup(&jointDef.Base, bodyId)
		pivotFrames(&jointDef.Base, bodyId)
		jointDef.Hertz = b2.QOne()
		jointDef.DampingRatio = b2.F(0.7)
		jointDef.LowerTranslation = b2.F(-1)
		jointDef.UpperTranslation = b2.QOne()
		jointDef.EnableLimit = true
		jointDef.EnableMotor = true
		jointDef.MaxMotorTorque = b2.F(10)
		jointDef.MotorSpeed = b2.QOne()
		s.jointIds[index] = b2.CreateWheelJoint(s.WorldId, &jointDef)
	}
	return s
}

func (s *JointEvent) Step() {
	s.Base.Step()

	// Process joint events
	events := s.WorldId.GetJointEvents()
	for i := range events {
		// Destroy the joint if it is still valid
		event := &events[i]
		if event.JointId.IsValid() {
			index := event.UserData.(int)
			sampleAssert(0 <= index && index < jointEventCount, "joint event index")
			b2.DestroyJoint(event.JointId)
			s.jointIds[index] = b2.JointId{}
		}
	}
}

// PersistentContact mirrors the Persistent Contact sample of
// sample_events.cpp. It tracks a contact across time steps, as one would for
// rolling or sliding sounds.
type PersistentContact struct {
	Base

	contactId b2.ContactId
}

// NewPersistentContact builds the persistent contact scene.
func NewPersistentContact(ctx *SampleContext) Sample {
	s := &PersistentContact{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 6}
		ctx.Camera.Zoom = 7.5
	}

	{
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(s.WorldId, &bodyDef)

		points := make([]b2.Vec2, 22)
		x := b2.F(10)
		for i := range 20 {
			points[i] = b2.Vec2{X: x}
			x = x.Sub(b2.QOne())
		}
		points[20] = b2.V2(-9, 10)
		points[21] = b2.V2(10, 10)

		chainDef := b2.DefaultChainDef()
		chainDef.Points = points
		chainDef.IsLoop = true
		b2.CreateChain(groundId, &chainDef)
	}

	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.V2(-8, 1)
		bodyDef.LinearVelocity = b2.V2(2, 0)
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		shapeDef.EnableContactEvents = true
		circle := b2.Circle{Radius: b2.QHalf()}
		b2.CreateCircleShape(bodyId, &shapeDef, &circle)
	}
	return s
}

func (s *PersistentContact) Step() {
	s.Base.Step()

	events := s.WorldId.GetContactEvents()
	if len(events.BeginEvents) > 0 {
		s.contactId = events.BeginEvents[0].ContactId
	}

	for i := range events.EndEvents {
		if s.contactId == events.EndEvents[i].ContactId {
			s.contactId = b2.ContactId{}
			break
		}
	}

	// Contacts are created and destroyed by Box2D, so it is important to make
	// sure a contact id is valid before accessing contact data.
	if !s.contactId.IsNull() && s.contactId.IsValid() {
		data := s.contactId.GetData()
		draw := &s.Context.Draw

		for i := range data.Manifold.PointCount {
			manifoldPoint := &data.Manifold.Points[i]
			bodyIdA := data.ShapeIdA.GetBody()
			p1 := bodyIdA.GetWorldCenter().Add(manifoldPoint.AnchorA)
			p2 := b2.MulAdd(p1, manifoldPoint.TotalNormalImpulse, data.Manifold.Normal)
			draw.DrawLineFcn(p1, p2, b2.ColorCrimson)
			draw.DrawPointFcn(p1, b2.F(6), b2.ColorCrimson)
			draw.DrawStringFcn(p1, fmt.Sprintf("%.2f", ToFloat64(manifoldPoint.TotalNormalImpulse)), b2.ColorWhite)
		}
	} else {
		s.contactId = b2.ContactId{}
	}
}

const sensorHitsTransformCapacity = 20

// SensorHits mirrors the Sensor Hits sample of sample_events.cpp. A fast
// projectile crosses static, kinematic and dynamic sensors.
type SensorHits struct {
	Base

	staticSensorId    b2.ShapeId
	kinematicSensorId b2.ShapeId
	dynamicSensorId   b2.ShapeId

	kinematicBodyId b2.BodyId
	dynamicBodyId   b2.BodyId
	jointId         b2.JointId

	bodyId  b2.BodyId
	shapeId b2.ShapeId

	transformCount int
	transforms     [sensorHitsTransformCapacity]b2.Transform

	isBullet   bool
	beginCount int
	endCount   int
}

// NewSensorHits builds the sensor hits scene.
func NewSensorHits(ctx *SampleContext) Sample {
	s := &SensorHits{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 5}
		ctx.Camera.Zoom = 7.5
	}

	var groundId b2.BodyId
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Name = "ground"
		groundId = b2.CreateBody(s.WorldId, &bodyDef)
		shapeDef := b2.DefaultShapeDef()

		groundSegment := b2.Segment{Point1: b2.V2(-10, 0), Point2: b2.V2(10, 0)}
		b2.CreateSegmentShape(groundId, &shapeDef, &groundSegment)

		groundSegment = b2.Segment{Point1: b2.V2(10, 0), Point2: b2.V2(10, 10)}
		b2.CreateSegmentShape(groundId, &shapeDef, &groundSegment)
	}

	// Static sensor
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Name = "static sensor"
		bodyDef.Position = b2.V2(-4, 1)
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)
		shapeDef := b2.DefaultShapeDef()
		shapeDef.IsSensor = true
		shapeDef.EnableSensorEvents = true

		segment := b2.Segment{Point2: b2.V2(0, 10)}
		s.staticSensorId = b2.CreateSegmentShape(bodyId, &shapeDef, &segment)
	}

	// Kinematic sensor
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Name = "kinematic sensor"
		bodyDef.Type = b2.KinematicBody
		bodyDef.Position = b2.V2(0, 1)
		bodyDef.LinearVelocity = b2.V2(0.5, 0)
		s.kinematicBodyId = b2.CreateBody(s.WorldId, &bodyDef)
		shapeDef := b2.DefaultShapeDef()
		shapeDef.IsSensor = true
		shapeDef.EnableSensorEvents = true

		segment := b2.Segment{Point2: b2.V2(0, 10)}
		s.kinematicSensorId = b2.CreateSegmentShape(s.kinematicBodyId, &shapeDef, &segment)
	}

	// Dynamic sensor
	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Name = "dynamic sensor"
		bodyDef.Type = b2.DynamicBody
		bodyDef.Position = b2.V2(4, 1)
		s.dynamicBodyId = b2.CreateBody(s.WorldId, &bodyDef)
		shapeDef := b2.DefaultShapeDef()
		shapeDef.IsSensor = true
		shapeDef.EnableSensorEvents = true

		capsule := b2.Capsule{Center1: b2.V2(0, 1), Center2: b2.V2(0, 9), Radius: b2.F(0.1)}
		s.dynamicSensorId = b2.CreateCapsuleShape(s.dynamicBodyId, &shapeDef, &capsule)

		pivot := bodyDef.Position.Add(b2.V2(0, 6))
		axis := b2.MakeRotFromUnitVector(b2.V2(1, 0))
		jointDef := b2.DefaultPrismaticJointDef()
		jointDef.Base.BodyIdA = groundId
		jointDef.Base.BodyIdB = s.dynamicBodyId
		jointDef.Base.LocalFrameA.Q = axis
		jointDef.Base.LocalFrameA.P = groundId.GetLocalPoint(pivot)
		jointDef.Base.LocalFrameB.Q = axis
		jointDef.Base.LocalFrameB.P = s.dynamicBodyId.GetLocalPoint(pivot)
		jointDef.EnableMotor = true
		jointDef.MaxMotorForce = b2.F(1000)
		jointDef.MotorSpeed = b2.QHalf()
		s.jointId = b2.CreatePrismaticJoint(s.WorldId, &jointDef)
	}

	s.isBullet = true
	s.launch()
	return s
}

func (s *SensorHits) launch() {
	if !s.bodyId.IsNull() {
		b2.DestroyBody(s.bodyId)
	}

	s.transformCount = 0
	s.beginCount = 0
	s.endCount = 0

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.Position = b2.V2(-26.7, 6)
	speed := shared.RandomFloatRange(b2.F(200), b2.F(300))
	bodyDef.LinearVelocity = b2.Vec2{X: speed}
	bodyDef.IsBullet = s.isBullet
	s.bodyId = b2.CreateBody(s.WorldId, &bodyDef)

	shapeDef := b2.DefaultShapeDef()
	shapeDef.EnableSensorEvents = true
	shapeDef.Material.Friction = b2.F(0.8)
	shapeDef.Material.RollingResistance = b2.F(0.01)
	circle := b2.Circle{Radius: b2.F(0.25)}
	s.shapeId = b2.CreateCircleShape(s.bodyId, &shapeDef, &circle)
}

func (s *SensorHits) UpdateGui() {
	gui := s.Context.Gui
	gui.Begin("Sensor Hits", 10, s.Context.Camera.Height-100-50, 200, 100)

	gui.Checkbox("Bullet", &s.isBullet)
	if gui.Button("Launch") || s.keyDown(KeyB) {
		s.launch()
	}

	gui.End()
}

func (s *SensorHits) collectTransforms(sensorShapeId b2.ShapeId) {
	const capacity = 5
	var visitorIds [capacity]b2.ShapeId
	count := sensorShapeId.GetSensorData(visitorIds[:])

	for i := 0; i < count && s.transformCount < sensorHitsTransformCapacity; i++ {
		sensorBodyId := sensorShapeId.GetBody()
		s.transforms[s.transformCount] = sensorBodyId.GetTransform()
		s.transformCount++
	}
}

func (s *SensorHits) Step() {
	p := s.kinematicBodyId.GetPosition()
	if p.X.Greater(b2.QOne()) {
		s.kinematicBodyId.SetLinearVelocity(b2.V2(-0.5, 0))
	} else if p.X.Less(b2.F(-1)) {
		s.kinematicBodyId.SetLinearVelocity(b2.V2(0.5, 0))
	}

	x := s.jointId.GetCurrentTranslation()
	if x.Greater(b2.QOne()) {
		s.jointId.SetTargetMotorSpeed(b2.F(-0.5))
	} else if x.Less(b2.F(-1)) {
		s.jointId.SetTargetMotorSpeed(b2.QHalf())
	}

	s.Base.Step()

	for i := range s.transformCount {
		s.Context.Draw.DrawTransformFcn(s.transforms[i])
	}

	sensorEvents := s.WorldId.GetSensorEvents()
	s.beginCount += len(sensorEvents.BeginEvents)
	s.endCount += len(sensorEvents.EndEvents)

	for i := range sensorEvents.BeginEvents {
		event := &sensorEvents.BeginEvents[i]
		if event.SensorShapeId.IsValid() {
			s.collectTransforms(event.SensorShapeId)
		}
	}

	s.DrawTextLine("begin touch count = %d", s.beginCount)
	s.DrawTextLine("end touch count = %d", s.endCount)
}

// ProjectileEvent mirrors the Projectile Event sample of sample_events.cpp.
// It shows how to create a projectile that explodes on impact.
type ProjectileEvent struct {
	Base

	projectileId      b2.BodyId
	projectileShapeId b2.ShapeId
	point1            b2.Vec2
	point2            b2.Vec2
	dragging          bool
}

// NewProjectileEvent builds the projectile event scene.
func NewProjectileEvent(ctx *SampleContext) Sample {
	s := &ProjectileEvent{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: -7, Y: 9}
		ctx.Camera.Zoom = 14
	}

	{
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		shapeDef.EnableSensorEvents = true

		segment := b2.Segment{Point1: b2.V2(10, 0), Point2: b2.V2(10, 20)}
		b2.CreateSegmentShape(groundId, &shapeDef, &segment)

		segment = b2.Segment{Point1: b2.V2(-30, 0), Point2: b2.V2(30, 0)}
		b2.CreateSegmentShape(groundId, &shapeDef, &segment)
	}

	box := b2.MakeRoundedBox(b2.F(0.45), b2.F(0.45), b2.F(0.05))

	shapeDef := b2.DefaultShapeDef()
	shapeDef.EnableSensorEvents = true

	offset := b2.F(0.01)
	x := b2.F(8)
	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody

	for i := range 8 {
		shift := offset
		if i%2 == 0 {
			shift = offset.Neg()
		}
		bodyDef.Position = b2.Vec2{X: x.Add(shift), Y: b2.F(0.5).Add(b2.QFromInt(i))}
		bodyId := b2.CreateBody(s.WorldId, &bodyDef)
		b2.CreatePolygonShape(bodyId, &shapeDef, &box)
	}
	return s
}

func (s *ProjectileEvent) fireProjectile() {
	if !s.projectileId.IsNull() {
		b2.DestroyBody(s.projectileId)
	}

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.Position = s.point1
	bodyDef.LinearVelocity = s.point2.Sub(s.point1).Mul(b2.F(4))
	bodyDef.IsBullet = true
	s.projectileId = b2.CreateBody(s.WorldId, &bodyDef)

	circle := b2.Circle{Radius: b2.F(0.25)}
	shapeDef := b2.DefaultShapeDef()
	shapeDef.EnableContactEvents = true
	s.projectileShapeId = b2.CreateCircleShape(s.projectileId, &shapeDef, &circle)
}

func (s *ProjectileEvent) MouseDown(p b2.Vec2, button MouseButton, mod Modifier) {
	if button == MouseButtonLeft && mod == ModControl {
		s.dragging = true
		s.point1 = p
	}
}

func (s *ProjectileEvent) MouseUp(_ b2.Vec2, button MouseButton) {
	if button == MouseButtonLeft && s.dragging {
		s.dragging = false
		s.fireProjectile()
	}
}

func (s *ProjectileEvent) MouseMove(p b2.Vec2) {
	if s.dragging {
		s.point2 = p
	}
}

func (s *ProjectileEvent) Step() {
	s.DrawTextLine("Use Ctrl + Left Mouse to drag and shoot a projectile")

	s.Base.Step()

	draw := &s.Context.Draw
	if s.dragging {
		draw.DrawLineFcn(s.point1, s.point2, b2.ColorWhite)
		draw.DrawPointFcn(s.point1, b2.F(5), b2.ColorGreen)
		draw.DrawPointFcn(s.point2, b2.F(5), b2.ColorRed)
	}

	contactEvents := s.WorldId.GetContactEvents()
	for i := range contactEvents.BeginEvents {
		event := &contactEvents.BeginEvents[i]

		if event.ShapeIdA == s.projectileShapeId || event.ShapeIdB == s.projectileShapeId {
			if event.ContactId.IsValid() {
				data := event.ContactId.GetData()

				if data.Manifold.PointCount > 0 {
					explosionDef := b2.DefaultExplosionDef()
					explosionDef.Position = data.ShapeIdA.GetBody().GetWorldCenter().Add(data.Manifold.Points[0].AnchorA)
					explosionDef.Radius = b2.QOne()
					explosionDef.ImpulsePerLength = b2.F(20)
					s.WorldId.Explode(&explosionDef)

					b2.DestroyBody(s.projectileId)
					s.projectileId = b2.BodyId{}
				}
			}

			break
		}
	}
}

type circleImpulseEvent struct {
	impulse      b2.Q
	totalImpulse b2.Q
	speed        b2.Q
}

// CircleImpulse mirrors the Circle Impulse sample of sample_events.cpp. It
// reports the hit speed and the impulses of a circle that hits the ground.
type CircleImpulse struct {
	Base

	mass           b2.Q
	events         []circleImpulseEvent
	bodyId         b2.BodyId
	restitution    b2.Q
	useGravity     bool
	useRestitution bool
}

// NewCircleImpulse builds the circle impulse scene.
func NewCircleImpulse(ctx *SampleContext) Sample {
	s := &CircleImpulse{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 2.7}
		ctx.Camera.Zoom = 3.4
	}

	{
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		segment := b2.Segment{Point1: b2.V2(-10, 0), Point2: b2.V2(10, 0)}
		b2.CreateSegmentShape(groundId, &shapeDef, &segment)
	}

	s.restitution = b2.F(0.25)
	s.mass = b2.QOne()
	s.spawn()
	return s
}

func (s *CircleImpulse) spawn() {
	if !s.bodyId.IsNull() {
		b2.DestroyBody(s.bodyId)
		s.bodyId = b2.BodyId{}
	}

	s.events = s.events[:0]

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	if !s.useGravity {
		bodyDef.GravityScale = b2.QZero()
	}
	bodyDef.LinearVelocity.Y = b2.F(-25)
	bodyDef.Position.Y = b2.F(5.5)

	circle := b2.Circle{Radius: b2.F(0.25)}

	shapeDef := b2.DefaultShapeDef()
	shapeDef.EnableHitEvents = true
	shapeDef.Material.Friction = b2.QZero()
	if s.useRestitution {
		shapeDef.Material.Restitution = s.restitution
	}

	s.bodyId = b2.CreateBody(s.WorldId, &bodyDef)
	b2.CreateCircleShape(s.bodyId, &shapeDef, &circle)

	// Override mass
	massData := s.bodyId.GetMassData()
	ratio := s.mass.Div(massData.Mass)
	massData.Mass = s.mass
	massData.RotationalInertia = massData.RotationalInertia.Mul(ratio)
	s.bodyId.SetMassData(massData)
}

func (s *CircleImpulse) UpdateGui() {
	gui := s.Context.Gui
	gui.Begin("Circle Impulse", 10, s.Context.Camera.Height-100-50, 200, 100)

	if gui.Checkbox("gravity", &s.useGravity) {
		s.spawn()
	}
	if gui.Checkbox("restitution", &s.useRestitution) {
		s.spawn()
	}

	gui.End()
}

func (s *CircleImpulse) Step() {
	s.Base.Step()

	events := s.WorldId.GetContactEvents()
	for i := range events.HitEvents {
		event := &events.HitEvents[i]

		s.Context.Draw.DrawPointFcn(event.Point, b2.F(10), b2.ColorWhite)

		data := event.ContactId.GetData()

		e := circleImpulseEvent{speed: event.ApproachSpeed}
		if data.Manifold.PointCount > 0 {
			e.impulse = data.Manifold.Points[0].NormalImpulse
			e.totalImpulse = data.Manifold.Points[0].TotalNormalImpulse
		}
		s.events = append(s.events, e)
	}

	gravity := 0.0
	if s.useGravity {
		gravity = 10
	}
	restitution := 0.0
	if s.useRestitution {
		restitution = ToFloat64(s.restitution)
	}
	s.DrawTextLine("mass = %g, gravity = %g, restitution = %g", ToFloat64(s.mass), gravity, restitution)

	for _, e := range s.events {
		s.DrawTextLine("hit speed = %g, hit momentum = %g, final impulse = %g, total impulse = %g",
			ToFloat64(e.speed), ToFloat64(s.mass.Mul(e.speed)), ToFloat64(e.impulse), ToFloat64(e.totalImpulse))
	}
}
