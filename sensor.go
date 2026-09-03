package dbox2d

import (
	"math/bits"
	"sort"

	"github.com/dhannyell/fixed"
)

// shapeRef identifies one sensor overlap. It corresponds to b2ShapeRef in
// src/sensor.h.
type shapeRef struct {
	shapeId    int
	generation uint16
}

// sensor stores the double-buffered overlaps of one sensor shape. It
// corresponds to b2Sensor in src/sensor.h.
type sensor struct {
	overlaps1 []shapeRef
	overlaps2 []shapeRef
	shapeId   int
}

// sensorTaskContext stores the changed sensors of one serial task. It
// corresponds to b2SensorTaskContext in src/sensor.h.
type sensorTaskContext struct {
	eventBits bitSet
}

// sensorQueryContext carries one sensor through its tree queries. It
// corresponds to b2SensorQueryContext in src/sensor.c.
type sensorQueryContext struct {
	world       *world
	sensor      *sensor
	sensorShape *shape
	transform   Transform
}

// sensorQueryCallback records one exact sensor overlap. It corresponds to
// b2SensorQueryCallback in src/sensor.c.
func sensorQueryCallback(_ int, userData uint64, context *sensorQueryContext) bool {
	shapeId := int(userData)
	sensorShape := context.sensorShape
	if shapeId == sensorShape.id {
		return true
	}

	w := context.world
	otherShape := &w.shapes[shapeId]
	if !otherShape.enableSensorEvents {
		return true
	}
	if otherShape.bodyId == sensorShape.bodyId {
		return true
	}
	if !shouldShapesCollide(sensorShape.filter, otherShape.filter) {
		return true
	}

	otherTransform := getBodyTransformQuick(w, &w.bodies[otherShape.bodyId])
	input := DistanceInput{
		ProxyA:     makeShapeDistanceProxy(sensorShape),
		ProxyB:     makeShapeDistanceProxy(otherShape),
		TransformA: context.transform,
		TransformB: otherTransform,
		UseRadii:   true,
	}
	var cache SimplexCache
	output := ShapeDistance(&input, &cache, nil)

	// D-012: sensor overlap requires an exact zero distance.
	if !output.Distance.Eq(fixed.Q32Zero()) {
		return true
	}

	context.sensor.overlaps2 = append(context.sensor.overlaps2, shapeRef{
		shapeId:    shapeId,
		generation: otherShape.generation,
	})
	return true
}

// compareShapeRefs orders references by shape id and then generation. It
// corresponds to b2CompareShapeRefs in src/sensor.c.
func compareShapeRefs(a, b shapeRef) bool {
	return a.shapeId < b.shapeId || a.shapeId == b.shapeId && a.generation < b.generation
}

// sensorTask queries every sensor and marks changed overlap sets. It
// corresponds to b2SensorTask in src/sensor.c; the port runs serially.
func sensorTask(w *world) {
	sensorCount := len(w.sensors)
	if sensorCount == 0 {
		return
	}

	taskContext := &w.sensorTaskContexts[0]
	setBitCountAndClear(&taskContext.eventBits, sensorCount)

	for sensorIndex := range w.sensors {
		s := &w.sensors[sensorIndex]
		s.overlaps2 = s.overlaps2[:0]

		sensorShape := &w.shapes[s.shapeId]
		if sensorShape.sensorIndex != sensorIndex {
			panic("dbox2d: a sensor shape has the wrong sensor index")
		}

		b := &w.bodies[sensorShape.bodyId]
		if b.setIndex != disabledSet && sensorShape.enableSensorEvents {
			queryContext := sensorQueryContext{
				world:       w,
				sensor:      s,
				sensorShape: sensorShape,
				transform:   getBodyTransformQuick(w, b),
			}
			callback := func(proxyId int, userData uint64) bool {
				return sensorQueryCallback(proxyId, userData, &queryContext)
			}

			maskBits := sensorShape.filter.MaskBits
			w.broadPhase.trees[StaticBody].query(sensorShape.fatAABB, maskBits, callback)
			w.broadPhase.trees[KinematicBody].query(sensorShape.fatAABB, maskBits, callback)
			w.broadPhase.trees[DynamicBody].query(sensorShape.fatAABB, maskBits, callback)
		}

		sort.Slice(s.overlaps2, func(i, j int) bool {
			return compareShapeRefs(s.overlaps2[i], s.overlaps2[j])
		})

		index1, index2 := 0, 0
		changed := false
		for index1 < len(s.overlaps1) && index2 < len(s.overlaps2) {
			ref1 := s.overlaps1[index1]
			ref2 := s.overlaps2[index2]
			if compareShapeRefs(ref1, ref2) || compareShapeRefs(ref2, ref1) {
				changed = true
				break
			}
			index1++
			index2++
		}
		if index1 < len(s.overlaps1) || index2 < len(s.overlaps2) {
			changed = true
		}
		if changed {
			taskContext.eventBits.setBit(sensorIndex)
		}
	}
}

// overlapSensors publishes deterministic begin and end events. It
// corresponds to b2OverlapSensors in src/sensor.c.
func overlapSensors(w *world) {
	sensorTask(w)
	if len(w.sensors) == 0 {
		return
	}

	eventBits := &w.sensorTaskContexts[0].eventBits
	for blockIndex := range eventBits.bits {
		bitsWord := eventBits.bits[blockIndex]
		for bitsWord != 0 {
			sensorIndex := 64*blockIndex + bits.TrailingZeros64(bitsWord)
			s := &w.sensors[sensorIndex]
			sensorShape := &w.shapes[s.shapeId]
			sensorShapeId := shapeIdOf(w, sensorShape)

			index1, index2 := 0, 0
			for index1 < len(s.overlaps1) && index2 < len(s.overlaps2) {
				ref1 := s.overlaps1[index1]
				ref2 := s.overlaps2[index2]

				switch {
				case compareShapeRefs(ref1, ref2):
					visitorShapeId := ShapeId{index1: int32(ref1.shapeId) + 1, world0: w.worldId, generation: ref1.generation}
					event := SensorEndTouchEvent{SensorShapeId: sensorShapeId, VisitorShapeId: visitorShapeId}
					w.sensorEndEvents[w.endEventArrayIndex] = append(w.sensorEndEvents[w.endEventArrayIndex], event)
					index1++
				case compareShapeRefs(ref2, ref1):
					visitorShapeId := ShapeId{index1: int32(ref2.shapeId) + 1, world0: w.worldId, generation: ref2.generation}
					event := SensorBeginTouchEvent{SensorShapeId: sensorShapeId, VisitorShapeId: visitorShapeId}
					w.sensorBeginEvents = append(w.sensorBeginEvents, event)
					index2++
				default:
					index1++
					index2++
				}
			}

			for index1 < len(s.overlaps1) {
				ref := s.overlaps1[index1]
				visitorShapeId := ShapeId{index1: int32(ref.shapeId) + 1, world0: w.worldId, generation: ref.generation}
				event := SensorEndTouchEvent{SensorShapeId: sensorShapeId, VisitorShapeId: visitorShapeId}
				w.sensorEndEvents[w.endEventArrayIndex] = append(w.sensorEndEvents[w.endEventArrayIndex], event)
				index1++
			}

			for index2 < len(s.overlaps2) {
				ref := s.overlaps2[index2]
				visitorShapeId := ShapeId{index1: int32(ref.shapeId) + 1, world0: w.worldId, generation: ref.generation}
				event := SensorBeginTouchEvent{SensorShapeId: sensorShapeId, VisitorShapeId: visitorShapeId}
				w.sensorBeginEvents = append(w.sensorBeginEvents, event)
				index2++
			}

			bitsWord &= bitsWord - 1
		}
	}

	for sensorIndex := range w.sensors {
		s := &w.sensors[sensorIndex]
		s.overlaps1, s.overlaps2 = s.overlaps2, s.overlaps1
	}
}

// destroySensor publishes end events and removes a sensor by swap. It
// corresponds to b2DestroySensor in src/sensor.c.
func destroySensor(w *world, sensorShape *shape) {
	sensorIndex := sensorShape.sensorIndex
	if sensorIndex < 0 || len(w.sensors) <= sensorIndex {
		panic("dbox2d: the sensor index is out of range")
	}

	s := &w.sensors[sensorIndex]
	sensorShapeId := shapeIdOf(w, sensorShape)
	for _, ref := range s.overlaps1 {
		visitorShapeId := ShapeId{index1: int32(ref.shapeId) + 1, world0: w.worldId, generation: ref.generation}
		event := SensorEndTouchEvent{SensorShapeId: sensorShapeId, VisitorShapeId: visitorShapeId}
		w.sensorEndEvents[w.endEventArrayIndex] = append(w.sensorEndEvents[w.endEventArrayIndex], event)
	}

	var movedIndex int
	w.sensors, movedIndex = removeSwap(w.sensors, sensorIndex)
	if movedIndex != nullIndex {
		movedSensor := &w.sensors[sensorIndex]
		movedSensorShape := &w.shapes[movedSensor.shapeId]
		movedSensorShape.sensorIndex = sensorIndex
	}
}
