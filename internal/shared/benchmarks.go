// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from shared/benchmarks.c of Box2D v3.1.1. Debug-only sizes use the
// release values.

// Package shared is the shared/ directory of the reference: the benchmark
// scenes of benchmarks.c, the ragdoll of human.c and the deterministic
// generator of random.h. Upstream links all three into both benchmark/main.c
// and its sample app; here they serve the conformance suite, the scene
// benchmark and the samples module. Keeping one source is what keeps a
// benchmark honest: a second copy that drifts still reports a number, and no
// test catches it. The builders use only the public API. Options carries the
// one reference value that public API cannot express.
package shared

import . "github.com/dhannyell/dbox2d"

// Options carries values that scenes cannot express through the public API.
type Options struct {
	// MotorJoint receives the tumbler's motor joint. Float conformance uses it
	// to set the reference radian speed directly; the public turn-based API
	// cannot reproduce the same float32 value, so samples leave it nil.
	MotorJoint func(JointId)

	// Rotation builds a body rotation from a radian angle, as b2MakeRot does.
	// CreateFallingHinges requires it; no other scene reads it. The public API
	// of this port takes turns (D-004), so a radian angle has no single
	// spelling here and the consumer has to name the one it means: conformance
	// passes the radian constructor of the library, which is what the frozen
	// traces record, and samples convert to turns.
	Rotation func(radians float32) Rot
}

// StepFn is the per-step hook of a scene, run before the world step with
// the index of that step. Only rain and spinner have one.
type StepFn func(step int)

// Spec is one scene: how to build it and how many steps benchmark/main.c
// runs it for.
type Spec struct {
	Steps int
	Build func(WorldId, Options) StepFn
}

// Specs is the scene table of benchmark/main.c, keyed by its scene name.
// falling_hinges is absent there and so absent here.
var Specs = map[string]Spec{
	"joint_grid":    {Steps: 500, Build: BuildJointGrid},
	"large_pyramid": {Steps: 500, Build: BuildLargePyramid},
	"many_pyramids": {Steps: 200, Build: BuildManyPyramids},
	"rain":          {Steps: 1000, Build: BuildRain},
	"smash":         {Steps: 300, Build: BuildSmash},
	"spinner":       {Steps: 1400, Build: BuildSpinner},
	"tumbler":       {Steps: 750, Build: BuildTumbler},
}

// BuildJointGrid is CreateJointGrid: a 100 by 100 grid of circles tied to
// their neighbours by revolute joints, a stress scene for the solver graph.
func BuildJointGrid(worldId WorldId, opts Options) StepFn {
	worldId.EnableSleeping(false)
	const n = 100
	bodies := make([]BodyId, n*n)
	index := 0
	shapeDef := DefaultShapeDef()
	shapeDef.Density = QOne()
	shapeDef.Filter.CategoryBits = 2
	shapeDef.Filter.MaskBits = DefaultMaskBits &^ uint64(2)
	circle := Circle{Radius: QMustParse("0.4")}
	jointDef := DefaultRevoluteJointDef()
	bodyDef := DefaultBodyDef()
	for k := range n {
		for i := range n {
			if k >= n/2-3 && k <= n/2+3 && i == 0 {
				bodyDef.Type = StaticBody
			} else {
				bodyDef.Type = DynamicBody
			}
			bodyDef.Position = Vec2{X: QFromInt(k), Y: QFromInt(i).Neg()}
			bodyId := CreateBody(worldId, &bodyDef)
			CreateCircleShape(bodyId, &shapeDef, &circle)
			if i > 0 {
				jointDef.BodyIdA = bodies[index-1]
				jointDef.BodyIdB = bodyId
				jointDef.LocalAnchorA = Vec2{Y: QHalf().Neg()}
				jointDef.LocalAnchorB = Vec2{Y: QHalf()}
				CreateRevoluteJoint(worldId, &jointDef)
			}
			if k > 0 {
				jointDef.BodyIdA = bodies[index-n]
				jointDef.BodyIdB = bodyId
				jointDef.LocalAnchorA = Vec2{X: QHalf()}
				jointDef.LocalAnchorB = Vec2{X: QHalf().Neg()}
				CreateRevoluteJoint(worldId, &jointDef)
			}
			bodies[index] = bodyId
			index++
		}
	}
	return nil
}

// BuildLargePyramid is CreateLargePyramid: one pyramid 100 boxes wide.
func BuildLargePyramid(worldId WorldId, opts Options) StepFn {
	worldId.EnableSleeping(false)
	const baseCount = 100
	groundDef := DefaultBodyDef()
	groundDef.Position = Vec2{Y: QFromInt(-1)}
	groundId := CreateBody(worldId, &groundDef)
	ground := MakeBox(QFromInt(100), QOne())
	shapeDef := DefaultShapeDef()
	CreatePolygonShape(groundId, &shapeDef, &ground)

	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	shapeDef.Density = QOne()
	half := QHalf()
	box := MakeSquare(half)
	for i := range baseCount {
		y := QFromInt(2).Mul(QFromInt(i)).Add(QOne()).Mul(half)
		for offset := range baseCount - i {
			j := i + offset
			x := QFromInt(i + 1).Mul(half).
				Add(QFromInt(2).Mul(QFromInt(j - i)).Mul(half)).
				Sub(half.Mul(QFromInt(baseCount)))
			bodyDef.Position = Vec2{X: x, Y: y}
			bodyId := CreateBody(worldId, &bodyDef)
			CreatePolygonShape(bodyId, &shapeDef, &box)
		}
	}
	return nil
}

func buildSmallPyramid(worldId WorldId, baseCount int, extent, centerX, baseY Q) {
	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	shapeDef := DefaultShapeDef()
	box := MakeSquare(extent)
	for i := range baseCount {
		y := QFromInt(2).Mul(QFromInt(i)).Add(QOne()).Mul(extent).Add(baseY)
		for offset := range baseCount - i {
			j := i + offset
			x := QFromInt(i + 1).Mul(extent).
				Add(QFromInt(2).Mul(QFromInt(j - i)).Mul(extent)).
				Add(centerX).Sub(QHalf())
			bodyDef.Position = Vec2{X: x, Y: y}
			bodyId := CreateBody(worldId, &bodyDef)
			CreatePolygonShape(bodyId, &shapeDef, &box)
		}
	}
}

// BuildManyPyramids is CreateManyPyramids: a 20 by 20 field of small
// pyramids, which spreads the load over many islands.
func BuildManyPyramids(worldId WorldId, opts Options) StepFn {
	worldId.EnableSleeping(false)
	const baseCount = 10
	const rowCount = 20
	const columnCount = 20
	extent := QHalf()
	groundDef := DefaultBodyDef()
	groundId := CreateBody(worldId, &groundDef)
	groundDeltaY := QFromInt(2).Mul(extent).Mul(QFromInt(baseCount).Add(QOne()))
	groundWidth := QFromInt(2).Mul(extent).Mul(QFromInt(columnCount)).Mul(QFromInt(baseCount).Add(QOne()))
	shapeDef := DefaultShapeDef()
	groundY := QZero()
	for range rowCount {
		segment := Segment{
			Point1: Vec2{X: groundWidth.Neg(), Y: groundY},
			Point2: Vec2{X: groundWidth, Y: groundY},
		}
		CreateSegmentShape(groundId, &shapeDef, &segment)
		groundY = groundY.Add(groundDeltaY)
	}
	baseWidth := QFromInt(2).Mul(extent).Mul(QFromInt(baseCount))
	baseY := QZero()
	for range rowCount {
		for j := range columnCount {
			centerX := QHalf().Neg().Mul(groundWidth).
				Add(QFromInt(j).Mul(baseWidth.Add(QFromInt(2).Mul(extent)))).
				Add(extent)
			buildSmallPyramid(worldId, baseCount, extent, centerX, baseY)
		}
		baseY = baseY.Add(groundDeltaY)
	}
	return nil
}

// BuildSmash is CreateSmash: a heavy box driven into a wall of light ones.
func BuildSmash(worldId WorldId, opts Options) StepFn {
	worldId.SetGravity(Vec2Zero())
	impact := MakeBox(QFromInt(4), QFromInt(4))
	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = Vec2{X: QFromInt(-20)}
	bodyDef.LinearVelocity = Vec2{X: QFromInt(40)}
	bodyId := CreateBody(worldId, &bodyDef)
	shapeDef := DefaultShapeDef()
	shapeDef.Density = QFromInt(8)
	CreatePolygonShape(bodyId, &shapeDef, &impact)

	d := QMustParse("0.4")
	box := MakeSquare(QHalf().Mul(d))
	bodyDef = DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.IsAwake = false
	shapeDef = DefaultShapeDef()
	const columns = 120
	const rows = 80
	for i := range columns {
		for j := range rows {
			bodyDef.Position = Vec2{
				X: QFromInt(i).Mul(d).Add(QFromInt(30)),
				Y: QFromInt(j).Sub(QFromRatio(rows, 2)).Mul(d),
			}
			bodyId = CreateBody(worldId, &bodyDef)
			CreatePolygonShape(bodyId, &shapeDef, &box)
		}
	}
	return nil
}

// BuildSpinner is CreateSpinner: a motorised bar sweeping capsules inside a
// chain loop, the scene with the most contact churn.
func BuildSpinner(worldId WorldId, opts Options) StepFn {
	groundDef := DefaultBodyDef()
	groundId := CreateBody(worldId, &groundDef)
	const pointCount = 360
	points := make([]Vec2, pointCount)
	rotation := MakeRot(QFromRatio(-1, pointCount))
	point := Vec2{X: QFromInt(40)}
	for i := range points {
		points[i] = Vec2{X: point.X, Y: point.Y.Add(QFromInt(32))}
		point = RotateVector(rotation, point)
	}
	chainDef := DefaultChainDef()
	chainDef.Points = points
	chainDef.IsLoop = true
	chainDef.Materials = []SurfaceMaterial{{Friction: QMustParse("0.1")}}
	CreateChain(groundId, &chainDef)

	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = Vec2{Y: QFromInt(12)}
	bodyDef.EnableSleep = false
	spinnerId := CreateBody(worldId, &bodyDef)
	spinner := MakeRoundedBox(QMustParse("0.4"), QFromInt(20), QMustParse("0.2"))
	shapeDef := DefaultShapeDef()
	shapeDef.Material.Friction = QZero()
	CreatePolygonShape(spinnerId, &shapeDef, &spinner)
	jointDef := DefaultRevoluteJointDef()
	jointDef.BodyIdA = groundId
	jointDef.BodyIdB = spinnerId
	jointDef.LocalAnchorA = bodyDef.Position
	jointDef.EnableMotor = true
	jointDef.MotorSpeed = QMustParse("0.7957747155")
	jointDef.MaxMotorTorque = QFromInt(40000)
	spinnerJoint := CreateRevoluteJoint(worldId, &jointDef)

	capsule := Capsule{Center1: Vec2{X: QMustParse("-0.25")}, Center2: Vec2{X: QMustParse("0.25")}, Radius: QMustParse("0.25")}
	circle := Circle{Radius: QMustParse("0.35")}
	square := MakeSquare(QMustParse("0.35"))
	bodyDef = DefaultBodyDef()
	bodyDef.Type = DynamicBody
	shapeDef = DefaultShapeDef()
	shapeDef.Material.Friction = QMustParse("0.1")
	shapeDef.Material.Restitution = QMustParse("0.1")
	shapeDef.Density = QMustParse("0.25")
	x, y := QFromInt(-24), QFromInt(2)
	for i := range 3038 {
		bodyDef.Position = Vec2{X: x, Y: y}
		bodyId := CreateBody(worldId, &bodyDef)
		switch i % 3 {
		case 0:
			CreateCapsuleShape(bodyId, &shapeDef, &capsule)
		case 1:
			CreateCircleShape(bodyId, &shapeDef, &circle)
		case 2:
			CreatePolygonShape(bodyId, &shapeDef, &square)
		}
		x = x.Add(QOne())
		if x.Greater(QFromInt(24)) {
			x = QFromInt(-24)
			y = y.Add(QOne())
		}
	}
	return func(int) { _ = spinnerJoint.GetAngle() }
}

// BuildTumbler is CreateTumbler: a motorised hollow box full of small ones.
// It is the only scene that reads Options.
func BuildTumbler(worldId WorldId, opts Options) StepFn {
	groundDef := DefaultBodyDef()
	groundId := CreateBody(worldId, &groundDef)
	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = Vec2{Y: QFromInt(10)}
	tumblerId := CreateBody(worldId, &bodyDef)
	shapeDef := DefaultShapeDef()
	shapeDef.Density = QFromInt(50)
	walls := []struct {
		halfWidth, halfHeight Q
		center                Vec2
	}{
		{QHalf(), QFromInt(10), Vec2{X: QFromInt(10)}},
		{QHalf(), QFromInt(10), Vec2{X: QFromInt(-10)}},
		{QFromInt(10), QHalf(), Vec2{Y: QFromInt(10)}},
		{QFromInt(10), QHalf(), Vec2{Y: QFromInt(-10)}},
	}
	for _, wall := range walls {
		polygon := MakeOffsetBox(wall.halfWidth, wall.halfHeight, wall.center, RotIdentity())
		CreatePolygonShape(tumblerId, &shapeDef, &polygon)
	}
	jointDef := DefaultRevoluteJointDef()
	jointDef.BodyIdA = groundId
	jointDef.BodyIdB = tumblerId
	jointDef.LocalAnchorA = Vec2{Y: QFromInt(10)}
	jointDef.MotorSpeed = QFromRatio(25, 360)
	jointDef.MaxMotorTorque = QFromInt(100000000)
	jointDef.EnableMotor = true
	jointId := CreateRevoluteJoint(worldId, &jointDef)
	if opts.MotorJoint != nil {
		opts.MotorJoint(jointId)
	}

	const gridCount = 45
	box := MakeBox(QMustParse("0.125"), QMustParse("0.125"))
	bodyDef = DefaultBodyDef()
	bodyDef.Type = DynamicBody
	shapeDef = DefaultShapeDef()
	y := QMustParse("-0.2").Mul(QFromInt(gridCount)).Add(QFromInt(10))
	for range gridCount {
		x := QMustParse("-0.2").Mul(QFromInt(gridCount))
		for range gridCount {
			bodyDef.Position = Vec2{X: x, Y: y}
			bodyId := CreateBody(worldId, &bodyDef)
			CreatePolygonShape(bodyId, &shapeDef, &box)
			x = x.Add(QMustParse("0.4"))
		}
		y = y.Add(QMustParse("0.4"))
	}
	return nil
}

const (
	rainRowCount    = 5
	rainColumnCount = 40
	rainGroupSize   = 5
)

type rainGroup struct {
	humans [rainGroupSize]Human
}

type rainData struct {
	groups      [rainRowCount * rainColumnCount]rainGroup
	gridSize    Q
	gridCount   int
	columnCount int
	columnIndex int
}

func (data *rainData) createGroup(worldId WorldId, rowIndex, columnIndex int) {
	groupIndex := rowIndex*rainColumnCount + columnIndex
	span := QFromInt(data.gridCount).Mul(data.gridSize)
	groupDistance := span.Div(QFromInt(rainColumnCount))
	position := Vec2{
		X: QHalf().Neg().Mul(span).Add(groupDistance.Mul(QFromInt(columnIndex).Add(QHalf()))),
		Y: QFromInt(40).Add(QFromInt(45).Mul(QFromInt(rowIndex))),
	}
	for i := range rainGroupSize {
		data.groups[groupIndex].humans[i] = CreateHuman(worldId, position, QOne(), QMustParse("0.05"), QFromInt(5), QHalf(), i+1, nil, false)
		position.X = position.X.Add(QHalf())
	}
}

func (data *rainData) destroyGroup(rowIndex, columnIndex int) {
	groupIndex := rowIndex*rainColumnCount + columnIndex
	for i := range rainGroupSize {
		data.groups[groupIndex].humans[i].Destroy()
	}
}

// BuildRain is CreateRain with StepRain: a long ground strip onto which the
// step hook keeps spawning and destroying groups of ragdolls. It is the only
// scene whose hook does real work, so its hook time is a body creation and
// destruction measurement rather than a solver one.
func BuildRain(worldId WorldId, opts Options) StepFn {
	data := &rainData{gridSize: QHalf(), gridCount: 500}
	groundDef := DefaultBodyDef()
	groundId := CreateBody(worldId, &groundDef)
	shapeDef := DefaultShapeDef()
	y := QZero()
	for range rainRowCount {
		x := QHalf().Neg().Mul(QFromInt(data.gridCount)).Mul(data.gridSize)
		for range data.gridCount + 1 {
			box := MakeOffsetBox(QHalf().Mul(data.gridSize), QHalf().Mul(data.gridSize), Vec2{X: x, Y: y}, RotIdentity())
			CreatePolygonShape(groundId, &shapeDef, &box)
			x = x.Add(data.gridSize)
		}
		y = y.Add(QFromInt(45))
	}
	return func(step int) {
		if step&0x7 != 0 {
			return
		}
		if data.columnCount < rainColumnCount {
			for row := range rainRowCount {
				data.createGroup(worldId, row, data.columnCount)
			}
			data.columnCount++
			return
		}
		for row := range rainRowCount {
			data.destroyGroup(row, data.columnIndex)
			data.createGroup(worldId, row, data.columnIndex)
		}
		data.columnIndex = (data.columnIndex + 1) % rainColumnCount
	}
}
