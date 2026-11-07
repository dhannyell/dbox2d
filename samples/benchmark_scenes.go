// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from shared/benchmarks.c of Box2D v3.2.0: the Washer, Junkyard,
// Sleep, Compounds and Queries scenes that internal/shared does not carry.
// Debug-only sizes use the release values. The b2Capacity hints of the
// reference (GetXxxCapacity) are not ported: the Go world grows on demand.

package samples

import (
	"math"

	"github.com/dhannyell/dbox2d"
)

// benchGroundSquares adds the three rows of static squares that frame the
// Compounds scene: a floor of 81 squares and two walls of 100.
func benchGroundSquares(groundId b2.BodyId, shapeDef *b2.ShapeDef) {
	gridSize := b2.QOne()
	wide := b2.F(0.55).Mul(gridSize)
	half := b2.QHalf().Mul(gridSize)

	y := b2.QZero()
	x := b2.F(-40).Mul(gridSize)
	for range 81 {
		box := b2.MakeOffsetBox(wide, half, b2.Vec2{X: x, Y: y}, b2.RotIdentity())
		b2.CreatePolygonShape(groundId, shapeDef, &box)
		x = x.Add(gridSize)
	}

	y = gridSize
	x = b2.F(-40).Mul(gridSize)
	for range 100 {
		box := b2.MakeOffsetBox(half, wide, b2.Vec2{X: x, Y: y}, b2.RotIdentity())
		b2.CreatePolygonShape(groundId, shapeDef, &box)
		y = y.Add(gridSize)
	}

	y = gridSize
	x = b2.F(40).Mul(gridSize)
	for range 100 {
		box := b2.MakeOffsetBox(half, wide, b2.Vec2{X: x, Y: y}, b2.RotIdentity())
		b2.CreatePolygonShape(groundId, shapeDef, &box)
		y = y.Add(gridSize)
	}
}

// benchCreateCompounds is CreateCompounds: each dynamic body is a compound
// of two triangular polygon shapes.
func benchCreateCompounds(worldId b2.WorldId) {
	{
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(worldId, &bodyDef)
		shapeDef := b2.DefaultShapeDef()
		benchGroundSquares(groundId, &shapeDef)

		segment := b2.Segment{Point1: b2.V2(-800, -80), Point2: b2.V2(800, -80)}
		b2.CreateSegmentShape(groundId, &shapeDef, &segment)
	}

	columnCount := 20
	rowCount := 150

	leftPoints := []b2.Vec2{b2.V2(-1, 0), b2.V2(0.5, 1), b2.V2(0, 2)}
	leftHull := b2.ComputeHull(leftPoints)
	left := b2.MakePolygon(&leftHull, b2.QZero())

	rightPoints := []b2.Vec2{b2.V2(1, 0), b2.V2(-0.5, 1), b2.V2(0, 2)}
	rightHull := b2.ComputeHull(rightPoints)
	right := b2.MakePolygon(&rightHull, b2.QZero())

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody

	shapeDef := b2.DefaultShapeDef()
	shapeDef.Density = b2.QOne()
	shapeDef.Material.Friction = b2.F(0.5)

	// Match the sample exactly: centery is computed before shift is reset
	// for the compound branch.
	shift := b2.F(2)
	extraY := b2.F(0.25)
	side := b2.F(0.25)
	centerX := shift.Mul(b2.F(columnCount)).Div(b2.F(2)).Sub(b2.QOne())
	centerY := b2.F(1.15).Div(b2.F(2))
	yStart := b2.F(5)

	for i := range columnCount {
		x := b2.F(i).Mul(shift).Sub(centerX)

		for j := range rowCount {
			y := b2.F(j).Mul(shift.Add(extraY)).Add(centerY).Add(yStart)

			bodyDef.Position = b2.ToPos(b2.Vec2{X: x.Add(side), Y: y})
			side = side.Neg()

			bodyId := b2.CreateBody(worldId, &bodyDef)
			b2.CreatePolygonShape(bodyId, &shapeDef, &left)
			b2.CreatePolygonShape(bodyId, &shapeDef, &right)
		}
	}
}

// benchCreateWasher is CreateWasher: a kinematic drum of walls filled with a
// grid of small boxes that report hit events.
func benchCreateWasher(worldId b2.WorldId) {
	{
		// The reference degrees-per-second motor speed, in turns per second.
		motorSpeed := b2.F(25).Div(b2.F(360))

		bodyDef := b2.DefaultBodyDef()
		bodyDef.Position = b2.ToPos(b2.V2(0, 10))
		bodyDef.Type = b2.KinematicBody
		bodyDef.AngularVelocity = motorSpeed
		bodyDef.LinearVelocity = b2.Vec2{X: b2.F(0.001), Y: b2.F(-0.002)}

		bodyId := b2.CreateBody(worldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()

		r0 := b2.F(14)
		r1 := b2.F(16)
		r2 := b2.F(18)

		angle := math.Pi / 18.0
		q := b2.Rot{Cos: b2.F(math.Cos(angle)), Sin: b2.F(math.Sin(angle))}
		qo := b2.Rot{Cos: b2.F(math.Cos(0.1 * angle)), Sin: b2.F(math.Sin(0.1 * angle))}
		u1 := b2.V2(1, 0)
		for i := range 36 {
			var u2 b2.Vec2
			if i == 35 {
				u2 = b2.V2(1, 0)
			} else {
				u2 = b2.RotateVector(q, u1)
			}

			{
				a1 := b2.InvRotateVector(qo, u1)
				a2 := b2.RotateVector(qo, u2)

				points := []b2.Vec2{a1.Mul(r1), a1.Mul(r2), a2.Mul(r1), a2.Mul(r2)}
				hull := b2.ComputeHull(points)
				polygon := b2.MakePolygon(&hull, b2.QZero())
				b2.CreatePolygonShape(bodyId, &shapeDef, &polygon)
			}

			if i%9 == 0 {
				points := []b2.Vec2{u1.Mul(r0), u1.Mul(r1), u2.Mul(r0), u2.Mul(r1)}
				hull := b2.ComputeHull(points)
				polygon := b2.MakePolygon(&hull, b2.QZero())
				b2.CreatePolygonShape(bodyId, &shapeDef, &polygon)
			}

			u1 = u2
		}
	}

	gridCount := 90
	a := b2.F(0.1)

	polygon := b2.MakeSquare(a)
	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	shapeDef := b2.DefaultShapeDef()
	shapeDef.EnableHitEvents = true

	y := b2.F(-1.1).Mul(a).Mul(b2.F(gridCount)).Add(b2.F(10))
	for range gridCount {
		x := b2.F(-1.1).Mul(a).Mul(b2.F(gridCount))

		for range gridCount {
			bodyDef.Position = b2.ToPos(b2.Vec2{X: x, Y: y})
			bodyId := b2.CreateBody(worldId, &bodyDef)

			b2.CreatePolygonShape(bodyId, &shapeDef, &polygon)

			x = x.Add(b2.F(2.1).Mul(a))
		}

		y = y.Add(b2.F(2.1).Mul(a))
	}
}

// benchJunkyard is the state of the Junkyard scene.
type benchJunkyard struct {
	pusherId b2.BodyId
}

// create is CreateJunkyard: a large pile of small pentagons and a kinematic
// pusher that sweeps through it.
func (j *benchJunkyard) create(worldId b2.WorldId) {
	{
		gridSize := b2.QOne()
		wide := b2.F(0.55).Mul(gridSize)
		half := b2.QHalf().Mul(gridSize)

		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(worldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()

		y := b2.QZero()
		x := b2.F(-80).Mul(gridSize)
		for range 161 {
			box := b2.MakeOffsetBox(wide, half, b2.Vec2{X: x, Y: y}, b2.RotIdentity())
			b2.CreatePolygonShape(groundId, &shapeDef, &box)
			x = x.Add(gridSize)
		}

		y = gridSize
		x = b2.F(-80).Mul(gridSize)
		for range 50 {
			box := b2.MakeOffsetBox(half, wide, b2.Vec2{X: x, Y: y}, b2.RotIdentity())
			b2.CreatePolygonShape(groundId, &shapeDef, &box)
			y = y.Add(gridSize)
		}

		y = gridSize
		x = b2.F(80).Mul(gridSize)
		for range 50 {
			box := b2.MakeOffsetBox(half, wide, b2.Vec2{X: x, Y: y}, b2.RotIdentity())
			b2.CreatePolygonShape(groundId, &shapeDef, &box)
			y = y.Add(gridSize)
		}
	}

	columnCount := 200
	rowCount := 40

	radius := b2.F(0.25)
	var polygon b2.Polygon
	{
		// Fibonacci sphere algorithm
		phi := math.Pi * (math.Sqrt(5.0) - 1.0)
		points := make([]b2.Vec2, 5)

		for i := range points {
			cs := rotFromRadians(phi * float64(i))
			points[i] = b2.Vec2{X: radius.Mul(cs.Cos), Y: radius.Mul(cs.Sin)}
		}

		hull := b2.ComputeHull(points)
		polygon = b2.MakePolygon(&hull, b2.QZero())
	}

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	shapeDef := b2.DefaultShapeDef()

	side := b2.F(-0.1)
	yStart := b2.F(15)

	for i := range columnCount {
		x := b2.F(1.5).Mul(b2.F(2*i - columnCount)).Mul(radius)

		for k := range rowCount {
			y := b2.F(4 * k).Mul(radius).Add(yStart)

			bodyDef.Position = b2.ToPos(b2.Vec2{X: x.Add(side), Y: y})
			side = side.Neg()

			bodyId := b2.CreateBody(worldId, &bodyDef)
			b2.CreatePolygonShape(bodyId, &shapeDef, &polygon)
		}
	}

	bodyDef.Type = b2.KinematicBody
	bodyDef.Position = b2.ToPos(b2.Vec2{})
	j.pusherId = b2.CreateBody(worldId, &bodyDef)
	box := b2.MakeOffsetBox(b2.F(2), b2.F(4), b2.V2(0, 4), b2.RotIdentity())
	b2.CreatePolygonShape(j.pusherId, &shapeDef, &box)
}

// step is StepJunkyard: it drives the pusher along a sine path.
func (j *benchJunkyard) step(stepCount int) {
	timeStep := b2.QFromRatio(1, 60)
	t := float64(stepCount) / 60.0
	cs := rotFromRadians(0.2 * t)
	target := b2.Transform{
		P: b2.Vec2{X: b2.F(60).Mul(cs.Sin)},
		Q: b2.RotIdentity(),
	}
	j.pusherId.SetTargetTransform(b2.MakeWorldTransform(target), timeStep, true)
}

const benchSleepPyramidCount = 10

// benchSleep is the state of the Sleep scene.
type benchSleep struct {
	bodyIdA [benchSleepPyramidCount]b2.BodyId
	bodyIdB [benchSleepPyramidCount]b2.BodyId
}

// createPyramid reports two bodies that are certain to share a touching
// contact, so the filter joint of step always lands inside one island.
func (d *benchSleep) createPyramid(worldId b2.WorldId, baseCount int, extent, centerX b2.Q) (b2.BodyId, b2.BodyId) {
	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.SleepThreshold = b2.QOne()

	shapeDef := b2.DefaultShapeDef()
	box := b2.MakeSquare(extent)

	var idA, idB b2.BodyId
	two := b2.F(2)
	for i := range baseCount {
		y := two.Mul(b2.F(i)).Add(b2.QOne()).Mul(extent)

		for j := i; j < baseCount; j++ {
			x := b2.F(i + 1).Mul(extent).Add(two.Mul(b2.F(j - i)).Mul(extent)).Add(centerX)
			bodyDef.Position = b2.ToPos(b2.Vec2{X: x, Y: y})

			bodyId := b2.CreateBody(worldId, &bodyDef)
			b2.CreatePolygonShape(bodyId, &shapeDef, &box)

			if i == 0 && j == 0 {
				idA = bodyId
			} else if i == 1 && j == 1 {
				// Rests on the base body above, so the pair always has a touching contact.
				idB = bodyId
			}
		}
	}
	return idA, idB
}

// create is CreateSleep: ten pyramids that stress waking and sleeping.
func (d *benchSleep) create(worldId b2.WorldId) {
	*d = benchSleep{}

	baseCount := 60
	extent := b2.QHalf()

	baseWidth := b2.F(2).Mul(extent).Mul(b2.F(baseCount))
	pitch := baseWidth.Add(b2.F(8).Mul(extent))
	span := pitch.Mul(b2.F(benchSleepPyramidCount - 1))

	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Position = b2.ToPos(b2.V2(0, -1))
		groundId := b2.CreateBody(worldId, &bodyDef)

		box := b2.MakeBox(b2.QHalf().Mul(span).Add(baseWidth), b2.QOne())
		shapeDef := b2.DefaultShapeDef()
		b2.CreatePolygonShape(groundId, &shapeDef, &box)
	}

	for i := range benchSleepPyramidCount {
		centerX := b2.QHalf().Neg().Mul(span).Add(b2.F(i).Mul(pitch)).Sub(b2.QHalf().Mul(baseWidth))
		d.bodyIdA[i], d.bodyIdB[i] = d.createPyramid(worldId, baseCount, extent, centerX)
	}
}

// step is StepSleep: it wakes one sleeping pyramid per step.
func (d *benchSleep) step(worldId b2.WorldId) {
	for i := range benchSleepPyramidCount {
		if !d.bodyIdA[i].IsAwake() {
			// Creating and destroying a joint engages the island splitter.
			jointDef := b2.DefaultFilterJointDef()
			jointDef.Base.BodyIdA = d.bodyIdA[i]
			jointDef.Base.BodyIdB = d.bodyIdB[i]
			jointId := b2.CreateFilterJoint(worldId, &jointDef)

			// This wakes the island.
			b2.DestroyJoint(jointId)

			// Only one per step.
			break
		}
	}
}

const benchQueryCount = 1000

// benchQueries is the state of the Queries scene. The queries are
// precomputed so only the tree work lands in the step time.
type benchQueries struct {
	origins      [benchQueryCount]b2.Vec2
	translations [benchQueryCount]b2.Vec2
	paddleId     b2.BodyId
	stats        b2.TreeStats
	extent       float32
	paddleMinX   b2.Q
	paddleMaxX   b2.Q
	randomState  uint32
}

// random is QueryRandom: a repeatable generator in single precision.
func (d *benchQueries) random(lower, upper float32) b2.Q {
	d.randomState = 1664525*d.randomState + 1013904223
	unit := float32(d.randomState>>8) * (1.0 / 16777216.0)
	return b2.F(float64(lower + (upper-lower)*unit))
}

// create is CreateQueries: a sparse grid of static boxes, a pile of dynamic
// boxes settling on a ground inside it with a kinematic paddle, and a few
// static sensors in the pile.
func (d *benchQueries) create(worldId b2.WorldId) {
	d.randomState = 1234

	d.extent = 500
	cellCount := 500
	fill := b2.F(0.1)

	// The pile keeps this region of the grid clear.
	pileX := b2.F(0.4).Mul(b2.F(500))
	pileY := b2.F(0.3).Mul(b2.F(500))
	clearLowerX := pileX.Sub(b2.F(10))
	clearLowerY := pileY.Sub(b2.F(10))
	clearUpperX := pileX.Add(b2.F(50))
	clearUpperY := pileY.Add(b2.F(60))

	bodyDef := b2.DefaultBodyDef()
	shapeDef := b2.DefaultShapeDef()

	for i := range cellCount {
		y := b2.F(i)
		for j := range cellCount {
			x := b2.F(j)

			fillTest := d.random(0, 1)
			ratio := d.random(1, 5)
			halfWidth := d.random(0.05, 0.25)
			orientation := d.random(0, 1)
			category := int(ToFloat64(d.random(0, 2.999)))

			if fillTest.Greater(fill) {
				continue
			}

			if !x.Less(clearLowerX) && !x.Greater(clearUpperX) && !y.Less(clearLowerY) && !y.Greater(clearUpperY) {
				continue
			}

			bodyDef.Position = b2.ToPos(b2.Vec2{X: x, Y: y})
			bodyId := b2.CreateBody(worldId, &bodyDef)

			var box b2.Polygon
			if orientation.Greater(b2.QHalf()) {
				box = b2.MakeBox(ratio.Mul(halfWidth), halfWidth)
			} else {
				box = b2.MakeBox(halfWidth, ratio.Mul(halfWidth))
			}
			shapeDef.Filter.CategoryBits = uint64(1) << category
			b2.CreatePolygonShape(bodyId, &shapeDef, &box)
		}
	}

	shapeDef.Filter.CategoryBits = 1

	// Ground for the pile.
	{
		bodyDef.Position = b2.ToPos(b2.Vec2{})
		groundId := b2.CreateBody(worldId, &bodyDef)
		segment := b2.Segment{
			Point1: b2.Vec2{X: pileX.Sub(b2.F(10)), Y: pileY},
			Point2: b2.Vec2{X: pileX.Add(b2.F(50)), Y: pileY},
		}
		b2.CreateSegmentShape(groundId, &shapeDef, &segment)
	}

	// Static sensors sitting in the pile.
	{
		sensorDef := b2.DefaultShapeDef()
		sensorDef.IsSensor = true
		sensorBox := b2.MakeSquare(b2.F(2))
		for i := range 8 {
			bodyDef.Position = b2.ToPos(b2.Vec2{X: pileX.Add(b2.F(4 * i)), Y: pileY.Add(b2.F(2))})
			sensorId := b2.CreateBody(worldId, &bodyDef)
			b2.CreatePolygonShape(sensorId, &sensorDef, &sensorBox)
		}
	}

	// The pile, dropped from just above the ground so it settles quickly.
	{
		columnCount := 40
		rowCount := 50
		spacing := b2.F(0.55)
		box := b2.MakeSquare(b2.F(0.25))

		bodyDef.Type = b2.DynamicBody
		for i := range rowCount {
			for j := range columnCount {
				bodyDef.Position = b2.ToPos(b2.Vec2{
					X: pileX.Add(spacing.Mul(b2.F(j))),
					Y: pileY.Add(b2.QOne()).Add(spacing.Mul(b2.F(i))),
				})
				bodyId := b2.CreateBody(worldId, &bodyDef)
				b2.CreatePolygonShape(bodyId, &shapeDef, &box)
			}
		}
	}

	// Kinematic paddle sweeping through the bottom of the pile.
	{
		bodyDef.Type = b2.KinematicBody
		bodyDef.Position = b2.ToPos(b2.Vec2{X: pileX.Sub(b2.F(5)), Y: pileY.Add(b2.F(1.2))})
		bodyDef.LinearVelocity = b2.V2(5, 0)
		d.paddleId = b2.CreateBody(worldId, &bodyDef)
		paddle := b2.MakeBox(b2.QOne(), b2.QOne())
		b2.CreatePolygonShape(d.paddleId, &shapeDef, &paddle)
		d.paddleMinX = pileX.Sub(b2.F(5))
		d.paddleMaxX = pileX.Add(b2.F(30))
	}

	// Rays and casts cross the whole world, overlaps are centered on the origins.
	for i := range benchQueryCount {
		x1 := d.random(0, d.extent)
		y1 := d.random(0, d.extent)
		x2 := d.random(0, d.extent)
		y2 := d.random(0, d.extent)
		d.origins[i] = b2.Vec2{X: x1, Y: y1}
		d.translations[i] = b2.Vec2{X: x2.Sub(x1), Y: y2.Sub(y1)}
	}

	d.stats = b2.TreeStats{}
}

// step is StepQueries: every step runs a fixed set of closest rays, circle
// casts and box overlaps, once with the default filter and once with a mask
// that hits one category in three. It returns the hit count.
func (d *benchQueries) step(worldId b2.WorldId) int {
	// Reverse the paddle at the ends of its sweep.
	paddlePosition := d.paddleId.GetPosition()
	if b2.ToVec2(paddlePosition).X.Greater(d.paddleMaxX) {
		d.paddleId.SetLinearVelocity(b2.V2(-5, 0))
	} else if b2.ToVec2(paddlePosition).X.Less(d.paddleMinX) {
		d.paddleId.SetLinearVelocity(b2.V2(5, 0))
	}

	filters := [2]b2.QueryFilter{b2.DefaultQueryFilter(), b2.DefaultQueryFilter()}
	filters[1].MaskBits = 1

	circle := b2.MakeProxy([]b2.Vec2{{}}, b2.F(0.1))

	hitCount := 0
	var stats b2.TreeStats

	for _, filter := range filters {
		for i := range benchQueryCount {
			result := worldId.CastRayClosest(b2.ToPos(d.origins[i]), d.translations[i], filter)
			stats.NodeVisits += result.NodeVisits
			stats.LeafVisits += result.LeafVisits
			if result.Hit {
				hitCount++
			}
		}

		for i := range benchQueryCount {
			fraction := b2.QOne()
			castStats := worldId.CastShape(b2.ToPos(d.origins[i]), &circle, d.translations[i], filter,
				func(_ b2.ShapeId, _ b2.Pos, _ b2.Vec2, f b2.Q) b2.Q {
					fraction = f
					return f
				})
			stats.NodeVisits += castStats.NodeVisits
			stats.LeafVisits += castStats.LeafVisits
			if fraction.Less(b2.QOne()) {
				hitCount++
			}
		}

		for i := range benchQueryCount {
			overlapCount := 0
			aabb := b2.AABB{LowerBound: b2.V2(-5, -5), UpperBound: b2.V2(5, 5)}
			overlapStats := worldId.OverlapAABB(b2.ToPos(d.origins[i]), aabb, filter, func(b2.ShapeId) bool {
				overlapCount++
				return true
			})
			stats.NodeVisits += overlapStats.NodeVisits
			stats.LeafVisits += overlapStats.LeafVisits
			hitCount += overlapCount
		}
	}

	d.stats.NodeVisits += stats.NodeVisits
	d.stats.LeafVisits += stats.LeafVisits
	return hitCount
}
