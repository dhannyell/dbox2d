// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from shared/benchmarks.c of Box2D v3.1.1. Debug-only sizes use
// the release values.

package samples

import (
	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/fixed"
)

func createJointGrid(worldId dbox2d.WorldId) {
	worldId.EnableSleeping(false)

	n := 100
	bodies := make([]dbox2d.BodyId, n*n)
	index := 0

	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Density = fixed.Q32One()
	shapeDef.Filter.CategoryBits = 2
	shapeDef.Filter.MaskBits = dbox2d.DefaultMaskBits &^ uint64(2)

	circle := dbox2d.Circle{
		Center: dbox2d.Vec2{X: fixed.Q32Zero(), Y: fixed.Q32Zero()},
		Radius: fixed.Q32MustParse("0.4"),
	}

	jd := dbox2d.DefaultRevoluteJointDef()
	bodyDef := dbox2d.DefaultBodyDef()

	for k := range n {
		for i := range n {
			fk := fixed.Q32FromInt(k)
			fi := fixed.Q32FromInt(i)

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
				jd.LocalAnchorA = dbox2d.Vec2{X: fixed.Q32Zero(), Y: fixed.Q32Half().Neg()}
				jd.LocalAnchorB = dbox2d.Vec2{X: fixed.Q32Zero(), Y: fixed.Q32Half()}
				dbox2d.CreateRevoluteJoint(worldId, &jd)
			}

			if k > 0 {
				jd.BodyIdA = bodies[index-n]
				jd.BodyIdB = body
				jd.LocalAnchorA = dbox2d.Vec2{X: fixed.Q32Half(), Y: fixed.Q32Zero()}
				jd.LocalAnchorB = dbox2d.Vec2{X: fixed.Q32Half().Neg(), Y: fixed.Q32Zero()}
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
		bodyDef.Position = dbox2d.Vec2{X: fixed.Q32Zero(), Y: fixed.Q32FromInt(-1)}
		groundId := dbox2d.CreateBody(worldId, &bodyDef)

		box := dbox2d.MakeBox(fixed.Q32FromInt(100), fixed.Q32One())
		shapeDef := dbox2d.DefaultShapeDef()
		dbox2d.CreatePolygonShape(groundId, &shapeDef, &box)
	}

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody

	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Density = fixed.Q32One()

	h := fixed.Q32Half()
	box := dbox2d.MakeSquare(h)

	shift := fixed.Q32One().Mul(h)

	for i := range baseCount {
		y := fixed.Q32FromInt(2).Mul(fixed.Q32FromInt(i)).Add(fixed.Q32One()).Mul(shift)

		for j := i; j < baseCount; j++ {
			x := fixed.Q32FromInt(i + 1).Mul(shift).
				Add(fixed.Q32FromInt(2).Mul(fixed.Q32FromInt(j - i)).Mul(shift)).
				Sub(h.Mul(fixed.Q32FromInt(baseCount)))

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
		y := fixed.Q32FromInt(2).Mul(fixed.Q32FromInt(i)).Add(fixed.Q32One()).Mul(extent).Add(baseY)

		for j := i; j < baseCount; j++ {
			x := fixed.Q32FromInt(i + 1).Mul(extent).
				Add(fixed.Q32FromInt(2).Mul(fixed.Q32FromInt(j - i)).Mul(extent)).
				Add(centerX).Sub(fixed.Q32Half())
			bodyDef.Position = dbox2d.Vec2{X: x, Y: y}

			bodyId := dbox2d.CreateBody(worldId, &bodyDef)
			dbox2d.CreatePolygonShape(bodyId, &shapeDef, &box)
		}
	}
}

func createManyPyramids(worldId dbox2d.WorldId) {
	worldId.EnableSleeping(false)

	baseCount := 10
	extent := fixed.Q32Half()
	rowCount := 20
	columnCount := 20

	bodyDef := dbox2d.DefaultBodyDef()
	groundId := dbox2d.CreateBody(worldId, &bodyDef)

	groundDeltaY := fixed.Q32FromInt(2).Mul(extent).Mul(fixed.Q32FromInt(baseCount).Add(fixed.Q32One()))
	groundWidth := fixed.Q32FromInt(2).Mul(extent).Mul(fixed.Q32FromInt(columnCount)).Mul(fixed.Q32FromInt(baseCount).Add(fixed.Q32One()))
	shapeDef := dbox2d.DefaultShapeDef()

	groundY := fixed.Q32Zero()

	for range rowCount {
		segment := dbox2d.Segment{
			Point1: dbox2d.Vec2{X: fixed.Q32Half().Neg().Mul(fixed.Q32FromInt(2)).Mul(groundWidth), Y: groundY},
			Point2: dbox2d.Vec2{X: fixed.Q32Half().Mul(fixed.Q32FromInt(2)).Mul(groundWidth), Y: groundY},
		}
		dbox2d.CreateSegmentShape(groundId, &shapeDef, &segment)
		groundY = groundY.Add(groundDeltaY)
	}

	baseWidth := fixed.Q32FromInt(2).Mul(extent).Mul(fixed.Q32FromInt(baseCount))
	baseY := fixed.Q32Zero()

	for range rowCount {
		for j := range columnCount {
			centerX := fixed.Q32Half().Neg().Mul(groundWidth).
				Add(fixed.Q32FromInt(j).Mul(baseWidth.Add(fixed.Q32FromInt(2).Mul(extent)))).
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
		q := dbox2d.MakeRot(fixed.Q32FromRatio(-1, spinnerPointCount))
		p := dbox2d.Vec2{X: fixed.Q32FromInt(40), Y: fixed.Q32Zero()}
		for i := range spinnerPointCount {
			points[i] = dbox2d.Vec2{X: p.X, Y: p.Y.Add(fixed.Q32FromInt(32))}
			p = dbox2d.RotateVector(q, p)
		}

		material := dbox2d.SurfaceMaterial{Friction: fixed.Q32MustParse("0.1")}
		chainDef := dbox2d.DefaultChainDef()
		chainDef.Points = points
		chainDef.IsLoop = true
		chainDef.Materials = []dbox2d.SurfaceMaterial{material}
		dbox2d.CreateChain(groundId, &chainDef)
	}

	{
		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = dbox2d.Vec2{X: fixed.Q32Zero(), Y: fixed.Q32FromInt(12)}
		bodyDef.EnableSleep = false

		spinnerId := dbox2d.CreateBody(worldId, &bodyDef)

		box := dbox2d.MakeRoundedBox(fixed.Q32MustParse("0.4"), fixed.Q32FromInt(20), fixed.Q32MustParse("0.2"))
		shapeDef := dbox2d.DefaultShapeDef()
		shapeDef.Material.Friction = fixed.Q32Zero()
		dbox2d.CreatePolygonShape(spinnerId, &shapeDef, &box)

		motorSpeed := fixed.Q32MustParse("0.7957747155") // 5 rad/s / (2*pi) in turns/s.
		maxMotorTorque := fixed.Q32FromInt(40000)
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
		Center1: dbox2d.Vec2{X: fixed.Q32MustParse("-0.25"), Y: fixed.Q32Zero()},
		Center2: dbox2d.Vec2{X: fixed.Q32MustParse("0.25"), Y: fixed.Q32Zero()},
		Radius:  fixed.Q32MustParse("0.25"),
	}
	circle := dbox2d.Circle{
		Center: dbox2d.Vec2{X: fixed.Q32Zero(), Y: fixed.Q32Zero()},
		Radius: fixed.Q32MustParse("0.35"),
	}
	square := dbox2d.MakeSquare(fixed.Q32MustParse("0.35"))

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Material.Friction = fixed.Q32MustParse("0.1")
	shapeDef.Material.Restitution = fixed.Q32MustParse("0.1")
	shapeDef.Density = fixed.Q32MustParse("0.25")

	bodyCount := 3038

	x, y := fixed.Q32FromInt(-24), fixed.Q32FromInt(2)
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

		x = x.Add(fixed.Q32One())
		if x.Greater(fixed.Q32FromInt(24)) {
			x = fixed.Q32FromInt(-24)
			y = y.Add(fixed.Q32One())
		}
	}
}

func createSmash(worldId dbox2d.WorldId) {
	worldId.SetGravity(dbox2d.Vec2Zero())

	{
		box := dbox2d.MakeBox(fixed.Q32FromInt(4), fixed.Q32FromInt(4))

		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		bodyDef.Position = dbox2d.Vec2{X: fixed.Q32FromInt(-20), Y: fixed.Q32Zero()}
		bodyDef.LinearVelocity = dbox2d.Vec2{X: fixed.Q32FromInt(40), Y: fixed.Q32Zero()}
		bodyId := dbox2d.CreateBody(worldId, &bodyDef)

		shapeDef := dbox2d.DefaultShapeDef()
		shapeDef.Density = fixed.Q32FromInt(8)
		dbox2d.CreatePolygonShape(bodyId, &shapeDef, &box)
	}

	d := fixed.Q32MustParse("0.4")
	box := dbox2d.MakeSquare(fixed.Q32Half().Mul(d))

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	bodyDef.IsAwake = false

	shapeDef := dbox2d.DefaultShapeDef()

	columns := 120
	rows := 80

	for i := range columns {
		for j := range rows {
			bodyDef.Position = dbox2d.Vec2{
				X: fixed.Q32FromInt(i).Mul(d).Add(fixed.Q32FromInt(30)),
				Y: fixed.Q32FromInt(j).Sub(fixed.Q32FromRatio(rows, 2)).Mul(d),
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
	rainData.gridSize = fixed.Q32Half()
	rainData.gridCount = 500

	{
		bodyDef := dbox2d.DefaultBodyDef()
		groundId := dbox2d.CreateBody(worldId, &bodyDef)

		shapeDef := dbox2d.DefaultShapeDef()
		y := fixed.Q32Zero()
		width := rainData.gridSize
		height := rainData.gridSize

		for range rainRowCount {
			x := fixed.Q32Half().Neg().Mul(fixed.Q32FromInt(rainData.gridCount)).Mul(rainData.gridSize)
			for j := 0; j <= rainData.gridCount; j++ {
				box := dbox2d.MakeOffsetBox(
					fixed.Q32Half().Mul(width),
					fixed.Q32Half().Mul(height),
					dbox2d.Vec2{X: x, Y: y},
					dbox2d.RotIdentity(),
				)
				dbox2d.CreatePolygonShape(groundId, &shapeDef, &box)
				x = x.Add(rainData.gridSize)
			}

			y = y.Add(fixed.Q32FromInt(45))
		}
	}

	rainData.columnCount = 0
	rainData.columnIndex = 0
}

func createGroup(worldId dbox2d.WorldId, rowIndex, columnIndex int) {
	groupIndex := rowIndex*rainColumnCount + columnIndex

	span := fixed.Q32FromInt(rainData.gridCount).Mul(rainData.gridSize)
	groupDistance := span.Div(fixed.Q32FromInt(rainColumnCount))
	position := dbox2d.Vec2{
		X: fixed.Q32Half().Neg().Mul(span).Add(
			groupDistance.Mul(fixed.Q32FromInt(columnIndex).Add(fixed.Q32Half())),
		),
		Y: fixed.Q32FromInt(40).Add(fixed.Q32FromInt(45).Mul(fixed.Q32FromInt(rowIndex))),
	}

	scale := fixed.Q32One()
	jointFriction := fixed.Q32MustParse("0.05")
	jointHertz := fixed.Q32FromInt(5)
	jointDamping := fixed.Q32Half()

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
		position.X = position.X.Add(fixed.Q32Half())
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
