// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from shared/benchmarks.c of Box2D v3.1.1. Debug-only sizes use the
// release values.

// Package scenes builds the benchmark scenes of the reference. It mirrors
// shared/benchmarks.c, which Box2D links into both benchmark/main.c and its
// sample app, and it serves two consumers here: the conformance suite, which
// replays a scene against its frozen trace, and the scene benchmark, which
// times it against tools/cbench. The samples module still carries its own
// copy.
//
// One source matters more here than it does upstream. A scene is only a
// valid benchmark against the C side, and only a valid conformance subject,
// if it is the same scene; a second copy that drifts produces a wrong number
// with no test to catch it.
//
// The package is written against the public API alone, dot-imported so the
// builders read as they do in the reference. The one value the public API
// cannot express is in Options.
package scenes

import . "github.com/dhannyell/dbox2d"

// Options carries what a scene cannot build for itself.
type Options struct {
	// MotorJoint receives the motor joint of a scene that has exactly one,
	// which today is the tumbler alone.
	//
	// It exists for one value. The reference sets the tumbler motor to
	// (B2_PI / 180.0f) * 25.0f radians per second, and in float mode a joint
	// keeps its speed in radians (D-004). No binary32 turn rate times
	// floatTau rounds back to that number, so a caller that needs the bits
	// of the reference has to store the radians directly, and only a caller
	// inside the library package can. The conformance suite and the scene
	// benchmark both do; the samples module leaves this nil and gets
	// QFromRatio(25, 360), which is the same motor to the eye and not to the
	// last bit.
	MotorJoint func(JointId)
}

// floatMode reports whether this build is the float scalar mode.
func floatMode() bool { return ScalarMode == "float" }

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
	boneHip = iota
	boneTorso
	boneHead
	boneUpperLeftLeg
	boneLowerLeftLeg
	boneUpperRightLeg
	boneLowerRightLeg
	boneUpperLeftArm
	boneLowerLeftArm
	boneUpperRightArm
	boneLowerRightArm
	boneCount
)

// Bone is one limb of a ragdoll: its body and the joint to its parent.
type Bone struct {
	bodyId  BodyId
	jointId JointId
}

// Human is the ragdoll of shared/human.c, which rain spawns by the group.
type Human struct {
	bones [boneCount]Bone
}

func (human *Human) destroy() {
	for i := range human.bones {
		if !human.bones[i].jointId.IsNull() {
			DestroyJoint(human.bones[i].jointId)
			human.bones[i].jointId = JointId{}
		}
	}
	for i := range human.bones {
		if !human.bones[i].bodyId.IsNull() {
			DestroyBody(human.bones[i].bodyId)
			human.bones[i].bodyId = BodyId{}
		}
	}
}

// CreateHuman is CreateHuman of shared/human.c, building the eleven bones
// and the revolute joints between them.
func CreateHuman(worldId WorldId, position Vec2, scale, frictionTorque, hertz, dampingRatio Q, groupIndex int) Human {
	human := Human{}
	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.SleepThreshold = QMustParse("0.1")
	shapeDef := DefaultShapeDef()
	shapeDef.Material.Friction = QMustParse("0.2")
	shapeDef.Filter.GroupIndex = -groupIndex
	shapeDef.Filter.CategoryBits = 2
	shapeDef.Filter.MaskBits = 1 | 2
	footShapeDef := shapeDef
	footShapeDef.Material.Friction = QMustParse("0.05")
	footShapeDef.Filter.MaskBits = 1

	makeBone := func(y, center1, center2, radius, damping Q) BodyId {
		bodyDef.Position = Vec2{Y: y.Mul(scale)}.Add(position)
		bodyDef.LinearDamping = damping
		bodyId := CreateBody(worldId, &bodyDef)
		capsule := Capsule{
			Center1: Vec2{Y: center1.Mul(scale)},
			Center2: Vec2{Y: center2.Mul(scale)},
			Radius:  radius.Mul(scale),
		}
		CreateCapsuleShape(bodyId, &shapeDef, &capsule)
		return bodyId
	}
	maxTorque := frictionTorque.Mul(scale)
	addJoint := func(parent, child BodyId, pivotY, lower, upper, reference, frictionScale Q) JointId {
		pivot := Vec2{Y: pivotY.Mul(scale)}.Add(position)
		jointDef := DefaultRevoluteJointDef()
		jointDef.BodyIdA = parent
		jointDef.BodyIdB = child
		jointDef.LocalAnchorA = parent.GetLocalPoint(pivot)
		jointDef.LocalAnchorB = child.GetLocalPoint(pivot)
		jointDef.ReferenceAngle = reference
		jointDef.EnableLimit = true
		jointDef.LowerAngle = lower
		jointDef.UpperAngle = upper
		jointDef.EnableMotor = true
		jointDef.MaxMotorTorque = frictionScale.Mul(maxTorque)
		jointDef.EnableSpring = hertz.Greater(QZero())
		jointDef.Hertz = hertz
		jointDef.DampingRatio = dampingRatio
		jointDef.DrawSize = QMustParse("0.05")
		return CreateRevoluteJoint(worldId, &jointDef)
	}

	human.bones[boneHip].bodyId = makeBone(QMustParse("0.95"), QMustParse("-0.02"), QMustParse("0.02"), QMustParse("0.095"), QZero())
	human.bones[boneTorso].bodyId = makeBone(QMustParse("1.2"), QMustParse("-0.135"), QMustParse("0.135"), QMustParse("0.09"), QZero())
	human.bones[boneTorso].jointId = addJoint(human.bones[boneHip].bodyId, human.bones[boneTorso].bodyId, QOne(), QFromRatio(-1, 8), QZero(), QZero(), QHalf())
	human.bones[boneHead].bodyId = makeBone(QMustParse("1.475"), QMustParse("-0.038"), QMustParse("0.039"), QMustParse("0.075"), QMustParse("0.1"))
	human.bones[boneHead].jointId = addJoint(human.bones[boneTorso].bodyId, human.bones[boneHead].bodyId, QMustParse("1.4"), QFromRatio(-3, 20), QFromRatio(1, 20), QZero(), QFromRatio(1, 4))
	human.bones[boneUpperLeftLeg].bodyId = makeBone(QMustParse("0.775"), QMustParse("-0.125"), QMustParse("0.125"), QMustParse("0.06"), QZero())
	human.bones[boneUpperLeftLeg].jointId = addJoint(human.bones[boneHip].bodyId, human.bones[boneUpperLeftLeg].bodyId, QMustParse("0.9"), QFromRatio(-1, 40), QFromRatio(1, 5), QZero(), QOne())

	footHull := ComputeHull([]Vec2{
		{X: QMustParse("-0.03").Mul(scale), Y: QMustParse("-0.185").Mul(scale)},
		{X: QMustParse("0.11").Mul(scale), Y: QMustParse("-0.185").Mul(scale)},
		{X: QMustParse("0.11").Mul(scale), Y: QMustParse("-0.16").Mul(scale)},
		{X: QMustParse("-0.03").Mul(scale), Y: QMustParse("-0.14").Mul(scale)},
	})
	footPolygon := MakePolygon(&footHull, QMustParse("0.015").Mul(scale))
	human.bones[boneLowerLeftLeg].bodyId = makeBone(QMustParse("0.475"), QMustParse("-0.155"), QMustParse("0.125"), QMustParse("0.045"), QZero())
	CreatePolygonShape(human.bones[boneLowerLeftLeg].bodyId, &footShapeDef, &footPolygon)
	human.bones[boneLowerLeftLeg].jointId = addJoint(human.bones[boneUpperLeftLeg].bodyId, human.bones[boneLowerLeftLeg].bodyId, QMustParse("0.625"), QFromRatio(-1, 4), QFromRatio(-1, 100), QZero(), QHalf())
	human.bones[boneUpperRightLeg].bodyId = makeBone(QMustParse("0.775"), QMustParse("-0.125"), QMustParse("0.125"), QMustParse("0.06"), QZero())
	human.bones[boneUpperRightLeg].jointId = addJoint(human.bones[boneHip].bodyId, human.bones[boneUpperRightLeg].bodyId, QMustParse("0.9"), QFromRatio(-1, 40), QFromRatio(1, 5), QZero(), QOne())
	human.bones[boneLowerRightLeg].bodyId = makeBone(QMustParse("0.475"), QMustParse("-0.155"), QMustParse("0.125"), QMustParse("0.045"), QZero())
	CreatePolygonShape(human.bones[boneLowerRightLeg].bodyId, &footShapeDef, &footPolygon)
	human.bones[boneLowerRightLeg].jointId = addJoint(human.bones[boneUpperRightLeg].bodyId, human.bones[boneLowerRightLeg].bodyId, QMustParse("0.625"), QFromRatio(-1, 4), QFromRatio(-1, 100), QZero(), QHalf())
	human.bones[boneUpperLeftArm].bodyId = makeBone(QMustParse("1.225"), QMustParse("-0.125"), QMustParse("0.125"), QMustParse("0.035"), QZero())
	human.bones[boneUpperLeftArm].jointId = addJoint(human.bones[boneTorso].bodyId, human.bones[boneUpperLeftArm].bodyId, QMustParse("1.35"), QFromRatio(-1, 20), QFromRatio(2, 5), QZero(), QHalf())
	human.bones[boneLowerLeftArm].bodyId = makeBone(QMustParse("0.975"), QMustParse("-0.125"), QMustParse("0.125"), QMustParse("0.03"), QMustParse("0.1"))
	human.bones[boneLowerLeftArm].jointId = addJoint(human.bones[boneUpperLeftArm].bodyId, human.bones[boneLowerLeftArm].bodyId, QMustParse("1.1"), QFromRatio(-1, 10), QFromRatio(3, 20), QFromRatio(1, 8), QMustParse("0.1"))
	human.bones[boneUpperRightArm].bodyId = makeBone(QMustParse("1.225"), QMustParse("-0.125"), QMustParse("0.125"), QMustParse("0.035"), QZero())
	human.bones[boneUpperRightArm].jointId = addJoint(human.bones[boneTorso].bodyId, human.bones[boneUpperRightArm].bodyId, QMustParse("1.35"), QFromRatio(-1, 20), QFromRatio(2, 5), QZero(), QHalf())
	human.bones[boneLowerRightArm].bodyId = makeBone(QMustParse("0.975"), QMustParse("-0.125"), QMustParse("0.125"), QMustParse("0.03"), QMustParse("0.1"))
	human.bones[boneLowerRightArm].jointId = addJoint(human.bones[boneUpperRightArm].bodyId, human.bones[boneLowerRightArm].bodyId, QMustParse("1.1"), QFromRatio(-1, 10), QFromRatio(3, 20), QFromRatio(1, 8), QMustParse("0.1"))
	return human
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
		data.groups[groupIndex].humans[i] = CreateHuman(worldId, position, QOne(), QMustParse("0.05"), QFromInt(5), QHalf(), i+1)
		position.X = position.X.Add(QHalf())
	}
}

func (data *rainData) destroyGroup(rowIndex, columnIndex int) {
	groupIndex := rowIndex*rainColumnCount + columnIndex
	for i := range rainGroupSize {
		data.groups[groupIndex].humans[i].destroy()
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
