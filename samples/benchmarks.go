// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from shared/benchmarks.c of Box2D v3.1.1. Debug-only sizes use
// the release values.

package samples

import (
	"github.com/dhannyell/dbox2d"
)

func createJointGrid(worldId dbox2d.WorldId) {
	worldId.EnableSleeping(false)

	n := 100
	bodies := make([]dbox2d.BodyId, n*n)
	index := 0

	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Density = dbox2d.QOne()
	shapeDef.Filter.CategoryBits = 2
	shapeDef.Filter.MaskBits = dbox2d.DefaultMaskBits &^ uint64(2)

	circle := dbox2d.Circle{
		Center: dbox2d.Vec2{X: dbox2d.QZero(), Y: dbox2d.QZero()},
		Radius: dbox2d.QMustParse("0.4"),
	}

	jd := dbox2d.DefaultRevoluteJointDef()
	bodyDef := dbox2d.DefaultBodyDef()

	for k := range n {
		for i := range n {
			fk := dbox2d.QFromInt(k)
			fi := dbox2d.QFromInt(i)

			if k >= n/2-3 && k <= n/2+3 && i == 0 {
				bodyDef.Type = dbox2d.StaticBody
			} else {
				bodyDef.Type = dbox2d.DynamicBody
			}

			bodyDef.Position = dbox2d.Vec2{X: fk, Y: fi.Neg()}

			body := dbox2d.CreateBody(worldId, &bodyDef)

			dbox2d.CreateCircleShape(body, &shapeDef, &circle)

			if i > 0 {
				jd.BodyIdA = bodies[index-1]
				jd.BodyIdB = body
				jd.LocalAnchorA = dbox2d.Vec2{X: dbox2d.QZero(), Y: dbox2d.QHalf().Neg()}
				jd.LocalAnchorB = dbox2d.Vec2{X: dbox2d.QZero(), Y: dbox2d.QHalf()}
				dbox2d.CreateRevoluteJoint(worldId, &jd)
			}

			if k > 0 {
				jd.BodyIdA = bodies[index-n]
				jd.BodyIdB = body
				jd.LocalAnchorA = dbox2d.Vec2{X: dbox2d.QHalf(), Y: dbox2d.QZero()}
				jd.LocalAnchorB = dbox2d.Vec2{X: dbox2d.QHalf().Neg(), Y: dbox2d.QZero()}
				dbox2d.CreateRevoluteJoint(worldId, &jd)
			}

			bodies[index] = body
			index++
		}
	}
}

func createLargePyramid(worldId dbox2d.WorldId) {
	worldId.EnableSleeping(false)

	baseCount := 100

	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Position = dbox2d.Vec2{X: dbox2d.QZero(), Y: dbox2d.QFromInt(-1)}
		groundId := dbox2d.CreateBody(worldId, &bodyDef)

		box := dbox2d.MakeBox(dbox2d.QFromInt(100), dbox2d.QOne())
		shapeDef := dbox2d.DefaultShapeDef()
		dbox2d.CreatePolygonShape(groundId, &shapeDef, &box)
	}

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody

	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Density = dbox2d.QOne()

	h := dbox2d.QHalf()
	box := dbox2d.MakeSquare(h)

	shift := dbox2d.QOne().Mul(h)

	for i := range baseCount {
		y := dbox2d.QFromInt(2).Mul(dbox2d.QFromInt(i)).Add(dbox2d.QOne()).Mul(shift)

		for j := i; j < baseCount; j++ {
			x := dbox2d.QFromInt(i + 1).Mul(shift).
				Add(dbox2d.QFromInt(2).Mul(dbox2d.QFromInt(j - i)).Mul(shift)).
				Sub(h.Mul(dbox2d.QFromInt(baseCount)))

			bodyDef.Position = dbox2d.Vec2{X: x, Y: y}

			bodyId := dbox2d.CreateBody(worldId, &bodyDef)
			dbox2d.CreatePolygonShape(bodyId, &shapeDef, &box)
		}
	}
}

func createSmallPyramid(worldId dbox2d.WorldId, baseCount int, extent, centerX, baseY dbox2d.Q) {
	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody

	shapeDef := dbox2d.DefaultShapeDef()

	box := dbox2d.MakeSquare(extent)

	for i := range baseCount {
		y := dbox2d.QFromInt(2).Mul(dbox2d.QFromInt(i)).Add(dbox2d.QOne()).Mul(extent).Add(baseY)

		for j := i; j < baseCount; j++ {
			x := dbox2d.QFromInt(i + 1).Mul(extent).
				Add(dbox2d.QFromInt(2).Mul(dbox2d.QFromInt(j - i)).Mul(extent)).
				Add(centerX).Sub(dbox2d.QHalf())
			bodyDef.Position = dbox2d.Vec2{X: x, Y: y}

			bodyId := dbox2d.CreateBody(worldId, &bodyDef)
			dbox2d.CreatePolygonShape(bodyId, &shapeDef, &box)
		}
	}
}

func createManyPyramids(worldId dbox2d.WorldId) {
	worldId.EnableSleeping(false)

	baseCount := 10
	extent := dbox2d.QHalf()
	rowCount := 20
	columnCount := 20

	bodyDef := dbox2d.DefaultBodyDef()
	groundId := dbox2d.CreateBody(worldId, &bodyDef)

	groundDeltaY := dbox2d.QFromInt(2).Mul(extent).Mul(dbox2d.QFromInt(baseCount).Add(dbox2d.QOne()))
	groundWidth := dbox2d.QFromInt(2).Mul(extent).Mul(dbox2d.QFromInt(columnCount)).Mul(dbox2d.QFromInt(baseCount).Add(dbox2d.QOne()))
	shapeDef := dbox2d.DefaultShapeDef()

	groundY := dbox2d.QZero()

	for range rowCount {
		segment := dbox2d.Segment{
			Point1: dbox2d.Vec2{X: dbox2d.QHalf().Neg().Mul(dbox2d.QFromInt(2)).Mul(groundWidth), Y: groundY},
			Point2: dbox2d.Vec2{X: dbox2d.QHalf().Mul(dbox2d.QFromInt(2)).Mul(groundWidth), Y: groundY},
		}
		dbox2d.CreateSegmentShape(groundId, &shapeDef, &segment)
		groundY = groundY.Add(groundDeltaY)
	}

	baseWidth := dbox2d.QFromInt(2).Mul(extent).Mul(dbox2d.QFromInt(baseCount))
	baseY := dbox2d.QZero()

	for range rowCount {
		for j := range columnCount {
			centerX := dbox2d.QHalf().Neg().Mul(groundWidth).
				Add(dbox2d.QFromInt(j).Mul(baseWidth.Add(dbox2d.QFromInt(2).Mul(extent)))).
				Add(extent)
			createSmallPyramid(worldId, baseCount, extent, centerX, baseY)
		}

		baseY = baseY.Add(groundDeltaY)
	}
}

const spinnerPointCount = 360

func createSpinner(worldId dbox2d.WorldId) {
	var groundId dbox2d.BodyId
	{
		bodyDef := dbox2d.DefaultBodyDef()
		groundId = dbox2d.CreateBody(worldId, &bodyDef)

		points := make([]dbox2d.Vec2, spinnerPointCount)
		q := dbox2d.MakeRot(dbox2d.QFromRatio(-1, spinnerPointCount))
		p := dbox2d.Vec2{X: dbox2d.QFromInt(40), Y: dbox2d.QZero()}
		for i := range spinnerPointCount {
			points[i] = dbox2d.Vec2{X: p.X, Y: p.Y.Add(dbox2d.QFromInt(32))}
			p = dbox2d.RotateVector(q, p)
		}

		material := dbox2d.SurfaceMaterial{Friction: dbox2d.QMustParse("0.1")}
		chainDef := dbox2d.DefaultChainDef()
		chainDef.Points = points
		chainDef.IsLoop = true
		chainDef.Materials = []dbox2d.SurfaceMaterial{material}
		dbox2d.CreateChain(groundId, &chainDef)
	}

	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = dbox2d.Vec2{X: dbox2d.QZero(), Y: dbox2d.QFromInt(12)}
		bodyDef.EnableSleep = false

		spinnerId := dbox2d.CreateBody(worldId, &bodyDef)

		box := dbox2d.MakeRoundedBox(dbox2d.QMustParse("0.4"), dbox2d.QFromInt(20), dbox2d.QMustParse("0.2"))
		shapeDef := dbox2d.DefaultShapeDef()
		shapeDef.Material.Friction = dbox2d.QZero()
		dbox2d.CreatePolygonShape(spinnerId, &shapeDef, &box)

		motorSpeed := dbox2d.QMustParse("0.7957747155") // 5 rad/s / (2*pi) in turns/s.
		maxMotorTorque := dbox2d.QFromInt(40000)
		jointDef := dbox2d.DefaultRevoluteJointDef()
		jointDef.BodyIdA = groundId
		jointDef.BodyIdB = spinnerId
		jointDef.LocalAnchorA = bodyDef.Position
		jointDef.EnableMotor = true
		jointDef.MotorSpeed = motorSpeed
		jointDef.MaxMotorTorque = maxMotorTorque

		dbox2d.CreateRevoluteJoint(worldId, &jointDef)
	}

	capsule := dbox2d.Capsule{
		Center1: dbox2d.Vec2{X: dbox2d.QMustParse("-0.25"), Y: dbox2d.QZero()},
		Center2: dbox2d.Vec2{X: dbox2d.QMustParse("0.25"), Y: dbox2d.QZero()},
		Radius:  dbox2d.QMustParse("0.25"),
	}
	circle := dbox2d.Circle{
		Center: dbox2d.Vec2{X: dbox2d.QZero(), Y: dbox2d.QZero()},
		Radius: dbox2d.QMustParse("0.35"),
	}
	square := dbox2d.MakeSquare(dbox2d.QMustParse("0.35"))

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Material.Friction = dbox2d.QMustParse("0.1")
	shapeDef.Material.Restitution = dbox2d.QMustParse("0.1")
	shapeDef.Density = dbox2d.QMustParse("0.25")

	bodyCount := 3038

	x, y := dbox2d.QFromInt(-24), dbox2d.QFromInt(2)
	for i := range bodyCount {
		bodyDef.Position = dbox2d.Vec2{X: x, Y: y}
		bodyId := dbox2d.CreateBody(worldId, &bodyDef)

		switch i % 3 {
		case 0:
			dbox2d.CreateCapsuleShape(bodyId, &shapeDef, &capsule)
		case 1:
			dbox2d.CreateCircleShape(bodyId, &shapeDef, &circle)
		case 2:
			dbox2d.CreatePolygonShape(bodyId, &shapeDef, &square)
		}

		x = x.Add(dbox2d.QOne())
		if x.Greater(dbox2d.QFromInt(24)) {
			x = dbox2d.QFromInt(-24)
			y = y.Add(dbox2d.QOne())
		}
	}
}

func createSmash(worldId dbox2d.WorldId) {
	worldId.SetGravity(dbox2d.Vec2Zero())

	{
		box := dbox2d.MakeBox(dbox2d.QFromInt(4), dbox2d.QFromInt(4))

		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = dbox2d.Vec2{X: dbox2d.QFromInt(-20), Y: dbox2d.QZero()}
		bodyDef.LinearVelocity = dbox2d.Vec2{X: dbox2d.QFromInt(40), Y: dbox2d.QZero()}
		bodyId := dbox2d.CreateBody(worldId, &bodyDef)

		shapeDef := dbox2d.DefaultShapeDef()
		shapeDef.Density = dbox2d.QFromInt(8)
		dbox2d.CreatePolygonShape(bodyId, &shapeDef, &box)
	}

	d := dbox2d.QMustParse("0.4")
	box := dbox2d.MakeSquare(dbox2d.QHalf().Mul(d))

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	bodyDef.IsAwake = false

	shapeDef := dbox2d.DefaultShapeDef()

	columns := 120
	rows := 80

	for i := range columns {
		for j := range rows {
			bodyDef.Position = dbox2d.Vec2{
				X: dbox2d.QFromInt(i).Mul(d).Add(dbox2d.QFromInt(30)),
				Y: dbox2d.QFromInt(j).Sub(dbox2d.QFromRatio(rows, 2)).Mul(d),
			}
			bodyId := dbox2d.CreateBody(worldId, &bodyDef)
			dbox2d.CreatePolygonShape(bodyId, &shapeDef, &box)
		}
	}
}

// Release values are 5, 40, and 5; BENCHMARK_DEBUG values were 3, 10, and 2.
const (
	rainRowCount    = 5
	rainColumnCount = 40
	rainGroupSize   = 5
)

type rainGroup struct {
	humans [rainGroupSize]human
}

type rainDataT struct {
	groups      [rainRowCount * rainColumnCount]rainGroup
	gridSize    dbox2d.Q
	gridCount   int
	columnCount int
	columnIndex int
}

var rainData rainDataT

func createRain(worldId dbox2d.WorldId) {
	rainData = rainDataT{}
	rainData.gridSize = dbox2d.QHalf()
	rainData.gridCount = 500

	{
		bodyDef := dbox2d.DefaultBodyDef()
		groundId := dbox2d.CreateBody(worldId, &bodyDef)

		shapeDef := dbox2d.DefaultShapeDef()
		y := dbox2d.QZero()
		width := rainData.gridSize
		height := rainData.gridSize

		for range rainRowCount {
			x := dbox2d.QHalf().Neg().Mul(dbox2d.QFromInt(rainData.gridCount)).Mul(rainData.gridSize)
			for j := 0; j <= rainData.gridCount; j++ {
				box := dbox2d.MakeOffsetBox(
					dbox2d.QHalf().Mul(width),
					dbox2d.QHalf().Mul(height),
					dbox2d.Vec2{X: x, Y: y},
					dbox2d.RotIdentity(),
				)
				dbox2d.CreatePolygonShape(groundId, &shapeDef, &box)
				x = x.Add(rainData.gridSize)
			}

			y = y.Add(dbox2d.QFromInt(45))
		}
	}

	rainData.columnCount = 0
	rainData.columnIndex = 0
}

func createGroup(worldId dbox2d.WorldId, rowIndex, columnIndex int) {
	groupIndex := rowIndex*rainColumnCount + columnIndex

	span := dbox2d.QFromInt(rainData.gridCount).Mul(rainData.gridSize)
	groupDistance := span.Div(dbox2d.QFromInt(rainColumnCount))
	position := dbox2d.Vec2{
		X: dbox2d.QHalf().Neg().Mul(span).Add(
			groupDistance.Mul(dbox2d.QFromInt(columnIndex).Add(dbox2d.QHalf())),
		),
		Y: dbox2d.QFromInt(40).Add(dbox2d.QFromInt(45).Mul(dbox2d.QFromInt(rowIndex))),
	}

	scale := dbox2d.QOne()
	jointFriction := dbox2d.QMustParse("0.05")
	jointHertz := dbox2d.QFromInt(5)
	jointDamping := dbox2d.QHalf()

	for i := range rainGroupSize {
		rainData.groups[groupIndex].humans[i] = createHuman(
			worldId,
			position,
			scale,
			jointFriction,
			jointHertz,
			jointDamping,
			i+1,
			nil,
			false,
		)
		position.X = position.X.Add(dbox2d.QHalf())
	}
}

func destroyGroup(rowIndex, columnIndex int) {
	groupIndex := rowIndex*rainColumnCount + columnIndex

	for i := range rainGroupSize {
		rainData.groups[groupIndex].humans[i].destroy()
	}
}

func stepRain(worldId dbox2d.WorldId, stepCount int) {
	delay := 0x7

	if stepCount&delay == 0 {
		if rainData.columnCount < rainColumnCount {
			for i := range rainRowCount {
				createGroup(worldId, i, rainData.columnCount)
			}
			rainData.columnCount++
		} else {
			for i := range rainRowCount {
				destroyGroup(i, rainData.columnIndex)
				createGroup(worldId, i, rainData.columnIndex)
			}
			rainData.columnIndex = (rainData.columnIndex + 1) % rainColumnCount
		}
	}
}
