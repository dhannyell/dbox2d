package dbox2d

import (
	"testing"

	"github.com/dhannyell/fixed"
)

func addSensorBox(t *testing.T, worldId WorldId, position Vec2, halfWidth, halfHeight Q) (BodyId, ShapeId) {
	t.Helper()
	bodyDef := DefaultBodyDef()
	bodyDef.Position = position
	bodyId := CreateBody(worldId, &bodyDef)
	shapeDef := DefaultShapeDef()
	shapeDef.IsSensor = true
	shapeDef.EnableSensorEvents = true
	box := MakeBox(halfWidth, halfHeight)
	return bodyId, CreatePolygonShape(bodyId, &shapeDef, &box)
}

func addSensorVisitorBox(t *testing.T, worldId WorldId, position Vec2, halfWidth, halfHeight Q, enableEvents bool) (BodyId, ShapeId) {
	t.Helper()
	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = position
	bodyId := CreateBody(worldId, &bodyDef)
	shapeDef := DefaultShapeDef()
	shapeDef.EnableSensorEvents = enableEvents
	box := MakeBox(halfWidth, halfHeight)
	return bodyId, CreatePolygonShape(bodyId, &shapeDef, &box)
}

// TestSensorBeginAndEndAroundAFallingBox pins one begin step followed
// immediately by one end step as a box falls through a thin sensor.
func TestSensorBeginAndEndAroundAFallingBox(t *testing.T) {
	worldId := createTestWorld(t)
	thin := fixed.Q32MustParse("0.0005")
	_, sensorShapeId := addSensorBox(t, worldId, Vec2Zero(), fixed.Q32One(), thin)
	_, visitorShapeId := addSensorVisitorBox(t, worldId, Vec2{Y: fixed.Q32MustParse("0.0012")}, fixed.Q32Half(), thin, true)

	beginStep, endStep := -1, -1
	for step := range 120 {
		worldId.Step(stepDt(), 4)
		events := worldId.GetSensorEvents()
		if len(events.BeginEvents) > 1 {
			t.Fatalf("step %d reports %d sensor begin events, want at most 1", step, len(events.BeginEvents))
		}
		if len(events.BeginEvents) == 1 {
			if beginStep != -1 {
				t.Fatalf("sensor begin event repeated on steps %d and %d", beginStep, step)
			}
			begin := events.BeginEvents[0]
			if begin.SensorShapeId != sensorShapeId || begin.VisitorShapeId != visitorShapeId {
				t.Fatalf("step %d reports begin event %+v, want sensor %v and visitor %v", step, begin, sensorShapeId, visitorShapeId)
			}
			beginStep = step
		}
		if len(events.EndEvents) > 1 {
			t.Fatalf("step %d reports %d sensor end events, want at most 1", step, len(events.EndEvents))
		}
		if len(events.EndEvents) == 1 {
			if endStep != -1 {
				t.Fatalf("sensor end event repeated on steps %d and %d", endStep, step)
			}
			end := events.EndEvents[0]
			if end.SensorShapeId != sensorShapeId || end.VisitorShapeId != visitorShapeId {
				t.Fatalf("step %d reports end event %+v, want sensor %v and visitor %v", step, end, sensorShapeId, visitorShapeId)
			}
			endStep = step
		}
	}

	if beginStep == -1 {
		t.Fatal("the falling box produced no sensor begin event")
	}
	if endStep == -1 {
		t.Fatal("the falling box produced no sensor end event")
	}
	if endStep != beginStep+1 {
		t.Fatalf("sensor begin and end events occurred on steps %d and %d, want consecutive steps", beginStep, endStep)
	}
}

// TestSensorEventsAreOrderedByShapeId pins begin-event ordering when two
// visitors enter the same sensor on one step.
func TestSensorEventsAreOrderedByShapeId(t *testing.T) {
	worldId := createTestWorld(t)
	_, sensorShapeId := addSensorBox(t, worldId, Vec2Zero(), fixed.Q32FromInt(4), fixed.Q32FromInt(2))
	_, visitorA := addSensorVisitorBox(t, worldId, v2(2, 0), fixed.Q32Half(), fixed.Q32Half(), true)
	_, visitorB := addSensorVisitorBox(t, worldId, v2(-2, 0), fixed.Q32Half(), fixed.Q32Half(), true)

	worldId.Step(stepDt(), 4)
	events := worldId.GetSensorEvents()
	if len(events.BeginEvents) != 2 {
		t.Fatalf("the first step reports %d sensor begin events, want 2", len(events.BeginEvents))
	}
	smaller, larger := visitorA, visitorB
	if larger.index1 < smaller.index1 {
		smaller, larger = larger, smaller
	}
	if events.BeginEvents[0].SensorShapeId != sensorShapeId || events.BeginEvents[0].VisitorShapeId != smaller {
		t.Errorf("the first begin event is %+v, want visitor %v", events.BeginEvents[0], smaller)
	}
	if events.BeginEvents[1].SensorShapeId != sensorShapeId || events.BeginEvents[1].VisitorShapeId != larger {
		t.Errorf("the second begin event is %+v, want visitor %v", events.BeginEvents[1], larger)
	}
}

// TestSensorIgnoresVisitorWithoutSensorEvents pins the opt-in gate on
// visitor shapes while they remain inside a sensor.
func TestSensorIgnoresVisitorWithoutSensorEvents(t *testing.T) {
	worldId := createTestWorld(t)
	addSensorBox(t, worldId, Vec2Zero(), fixed.Q32FromInt(4), fixed.Q32FromInt(2))
	addSensorVisitorBox(t, worldId, Vec2Zero(), fixed.Q32Half(), fixed.Q32Half(), false)

	for step := range 8 {
		worldId.Step(stepDt(), 4)
		if events := worldId.GetSensorEvents(); len(events.BeginEvents) != 0 {
			t.Fatalf("step %d reports %d sensor begin events for an opted-out visitor", step, len(events.BeginEvents))
		}
	}
}

// TestDestroyVisitorEmitsEnd pins the end event produced after an
// overlapping visitor shape is destroyed.
func TestDestroyVisitorEmitsEnd(t *testing.T) {
	worldId := createTestWorld(t)
	_, sensorShapeId := addSensorBox(t, worldId, Vec2Zero(), fixed.Q32FromInt(4), fixed.Q32FromInt(2))
	_, visitorShapeId := addSensorVisitorBox(t, worldId, Vec2Zero(), fixed.Q32Half(), fixed.Q32Half(), true)

	worldId.Step(stepDt(), 4)
	events := worldId.GetSensorEvents()
	if len(events.BeginEvents) != 1 || events.BeginEvents[0].SensorShapeId != sensorShapeId || events.BeginEvents[0].VisitorShapeId != visitorShapeId {
		t.Fatalf("the first step reports begin events %+v, want the sensor and visitor pair", events.BeginEvents)
	}

	DestroyShape(visitorShapeId, false)
	worldId.Step(stepDt(), 4)
	events = worldId.GetSensorEvents()
	if len(events.EndEvents) != 1 || events.EndEvents[0].SensorShapeId != sensorShapeId || events.EndEvents[0].VisitorShapeId != visitorShapeId {
		t.Fatalf("the step after visitor destruction reports end events %+v, want the sensor and visitor pair", events.EndEvents)
	}
}

// TestDestroySensorEmitsEnds pins one end event for every visitor when an
// overlapping sensor body is destroyed.
func TestDestroySensorEmitsEnds(t *testing.T) {
	worldId := createTestWorld(t)
	sensorBodyId, sensorShapeId := addSensorBox(t, worldId, Vec2Zero(), fixed.Q32FromInt(4), fixed.Q32FromInt(2))
	_, visitorA := addSensorVisitorBox(t, worldId, v2(2, 0), fixed.Q32Half(), fixed.Q32Half(), true)
	_, visitorB := addSensorVisitorBox(t, worldId, v2(-2, 0), fixed.Q32Half(), fixed.Q32Half(), true)

	worldId.Step(stepDt(), 4)
	beginEvents := worldId.GetSensorEvents().BeginEvents
	if len(beginEvents) != 2 {
		t.Fatalf("the first step reports %d sensor begin events, want 2", len(beginEvents))
	}
	wantBegins := map[ShapeId]bool{visitorA: true, visitorB: true}
	for _, event := range beginEvents {
		if event.SensorShapeId != sensorShapeId || !wantBegins[event.VisitorShapeId] {
			t.Fatalf("unexpected sensor begin event %+v", event)
		}
		delete(wantBegins, event.VisitorShapeId)
	}
	if len(wantBegins) != 0 {
		t.Fatalf("sensor begin events missed %d visitors", len(wantBegins))
	}

	DestroyBody(sensorBodyId)
	worldId.Step(stepDt(), 4)
	endEvents := worldId.GetSensorEvents().EndEvents
	if len(endEvents) != 2 {
		t.Fatalf("the step after sensor destruction reports %d sensor end events, want 2", len(endEvents))
	}
	wantVisitors := map[ShapeId]bool{visitorA: true, visitorB: true}
	for _, event := range endEvents {
		if event.SensorShapeId != sensorShapeId || !wantVisitors[event.VisitorShapeId] {
			t.Errorf("unexpected sensor end event %+v", event)
			continue
		}
		delete(wantVisitors, event.VisitorShapeId)
	}
	if len(wantVisitors) != 0 {
		t.Errorf("sensor end events missed %d visitors", len(wantVisitors))
	}
}

// TestSensorEndEventsUseTheDoubleBuffer pins that destruction end events
// become visible only after the following step flips the event buffers.
func TestSensorEndEventsUseTheDoubleBuffer(t *testing.T) {
	worldId := createTestWorld(t)
	_, sensorShapeId := addSensorBox(t, worldId, Vec2Zero(), fixed.Q32FromInt(4), fixed.Q32FromInt(2))
	_, visitorShapeId := addSensorVisitorBox(t, worldId, Vec2Zero(), fixed.Q32Half(), fixed.Q32Half(), true)

	worldId.Step(stepDt(), 4)
	beginEvents := worldId.GetSensorEvents().BeginEvents
	if len(beginEvents) != 1 || beginEvents[0].SensorShapeId != sensorShapeId || beginEvents[0].VisitorShapeId != visitorShapeId {
		t.Fatalf("the first step reports begin events %+v, want the sensor and visitor pair", beginEvents)
	}

	DestroyShape(visitorShapeId, false)
	if endEvents := worldId.GetSensorEvents().EndEvents; len(endEvents) != 0 {
		t.Fatalf("visitor destruction exposes %d sensor end events before the next step", len(endEvents))
	}

	worldId.Step(stepDt(), 4)
	endEvents := worldId.GetSensorEvents().EndEvents
	if len(endEvents) != 1 || endEvents[0].SensorShapeId != sensorShapeId || endEvents[0].VisitorShapeId != visitorShapeId {
		t.Fatalf("the step after visitor destruction reports end events %+v, want the sensor and visitor pair", endEvents)
	}
}

// TestSensorTouchingEdgeIsNotOverlap pins exact-zero overlap detection for
// boxes separated by one linear slop.
func TestSensorTouchingEdgeIsNotOverlap(t *testing.T) {
	worldId := createTestWorld(t)
	addSensorBox(t, worldId, Vec2Zero(), fixed.Q32One(), fixed.Q32One())
	visitorX := fixed.Q32FromInt(2).Add(linearSlop)
	addSensorVisitorBox(t, worldId, Vec2{X: visitorX}, fixed.Q32One(), fixed.Q32One(), true)

	worldId.Step(stepDt(), 4)
	if events := worldId.GetSensorEvents(); len(events.BeginEvents) != 0 {
		t.Fatalf("boxes separated by one linear slop report %d sensor begin events", len(events.BeginEvents))
	}
}
