package dbox2d

// The seven benchmark scenes of the reference, ported from
// shared/benchmarks.c, plus falling_hinges from its sample of the same name.
// They are shared by two consumers: the conformance suite, which replays
// them against the frozen traces, and TestSceneBench, which times them
// against tools/cbench. A third copy of the same scenes lives in
// samples/benchmarks.go, because the samples module is a separate module
// that this module's tests cannot import.
//
// They sit in a test file because that is where their only consumers are.
// The cost is that a program under tools/ cannot reach them, so the scene
// benchmark has to be a test too.

type conformanceStepFn func(step int)

type conformanceSceneSpec struct {
	steps int
	build func(WorldId) conformanceStepFn
}

var conformanceSceneSpecs = map[string]conformanceSceneSpec{
	"joint_grid.txt":     {steps: 500, build: buildConformanceJointGrid},
	"large_pyramid.txt":  {steps: 500, build: buildConformanceLargePyramid},
	"many_pyramids.txt":  {steps: 200, build: buildConformanceManyPyramids},
	"rain.txt":           {steps: 1000, build: buildConformanceRain},
	"smash.txt":          {steps: 300, build: buildConformanceSmash},
	"spinner.txt":        {steps: 1400, build: buildConformanceSpinner},
	"tumbler.txt":        {steps: 750, build: buildConformanceTumbler},
	"falling_hinges.txt": {build: buildConformanceFallingHinges},
}

func buildConformanceJointGrid(worldId WorldId) conformanceStepFn {
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

func buildConformanceLargePyramid(worldId WorldId) conformanceStepFn {
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

func buildConformanceSmallPyramid(worldId WorldId, baseCount int, extent, centerX, baseY Q) {
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

func buildConformanceManyPyramids(worldId WorldId) conformanceStepFn {
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
			buildConformanceSmallPyramid(worldId, baseCount, extent, centerX, baseY)
		}
		baseY = baseY.Add(groundDeltaY)
	}
	return nil
}

func buildConformanceSmash(worldId WorldId) conformanceStepFn {
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

func buildConformanceSpinner(worldId WorldId) conformanceStepFn {
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

func buildConformanceTumbler(worldId WorldId) conformanceStepFn {
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
	if conformanceFloatMode() {
		// The reference sets (B2_PI / 180.0f) * 25.0f radians per second. No
		// binary32 turn rate times floatTau rounds to that value, so float
		// mode stores the radians of the reference.
		pi := float32(3.14159265359)
		speed := pi / 180
		speed *= 25
		joint := getJointSimCheckType(getWorldFromId(worldId), jointId, RevoluteJoint)
		joint.revoluteJoint.motorSpeed = QFromFloat64(float64(speed))
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

func buildConformanceFallingHinges(worldId WorldId) conformanceStepFn {
	groundDef := DefaultBodyDef()
	groundDef.Position = Vec2{Y: QFromInt(-1)}
	groundId := CreateBody(worldId, &groundDef)
	ground := MakeBox(QFromInt(20), QOne())
	shapeDef := DefaultShapeDef()
	CreatePolygonShape(groundId, &shapeDef, &ground)

	const columnCount = 4
	const rowCount = 30
	half := QMustParse("0.25")
	radius := QMustParse("0.025")
	box := MakeRoundedBox(half.Sub(radius), half.Sub(radius), radius)
	shapeDef = DefaultShapeDef()
	shapeDef.Material.Friction = QMustParse("0.3")
	offset := QMustParse("0.1")
	dx := QFromInt(10).Mul(half)
	xroot := QHalf().Neg().Mul(dx).Mul(QFromInt(columnCount - 1))
	jointDef := DefaultRevoluteJointDef()
	jointDef.EnableLimit = true
	jointDef.LowerAngle = QFromRatio(-1, 20)
	jointDef.UpperAngle = QFromRatio(1, 10)
	jointDef.EnableSpring = true
	jointDef.Hertz = QHalf()
	jointDef.DampingRatio = QHalf()
	jointDef.LocalAnchorA = Vec2{X: half, Y: half}
	jointDef.LocalAnchorB = Vec2{X: offset, Y: half.Neg()}
	jointDef.DrawSize = QMustParse("0.1")

	for j := range columnCount {
		x := xroot.Add(QFromInt(j).Mul(dx))
		var previous BodyId
		for i := range rowCount {
			bodyDef := DefaultBodyDef()
			bodyDef.Type = DynamicBody
			bodyDef.Position = Vec2{X: x.Add(offset.Mul(QFromInt(i))), Y: half.Add(QFromInt(2).Mul(half).Mul(QFromInt(i)))}
			radians := float32(0.1*float32(i) - 1)
			bodyDef.Rotation = conformanceRotFromRadians(radians)
			bodyId := CreateBody(worldId, &bodyDef)
			if i&1 == 0 {
				previous = bodyId
			} else {
				jointDef.BodyIdA = previous
				jointDef.BodyIdB = bodyId
				CreateRevoluteJoint(worldId, &jointDef)
				previous = BodyId{}
			}
			CreatePolygonShape(bodyId, &shapeDef, &box)
		}
	}
	return nil
}

const (
	conformanceHip = iota
	conformanceTorso
	conformanceHead
	conformanceUpperLeftLeg
	conformanceLowerLeftLeg
	conformanceUpperRightLeg
	conformanceLowerRightLeg
	conformanceUpperLeftArm
	conformanceLowerLeftArm
	conformanceUpperRightArm
	conformanceLowerRightArm
	conformanceBoneCount
)

type conformanceBone struct {
	bodyId  BodyId
	jointId JointId
}

type conformanceHuman struct {
	bones [conformanceBoneCount]conformanceBone
}

func (human *conformanceHuman) destroy() {
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

func createConformanceHuman(worldId WorldId, position Vec2, scale, frictionTorque, hertz, dampingRatio Q, groupIndex int) conformanceHuman {
	human := conformanceHuman{}
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

	human.bones[conformanceHip].bodyId = makeBone(QMustParse("0.95"), QMustParse("-0.02"), QMustParse("0.02"), QMustParse("0.095"), QZero())
	human.bones[conformanceTorso].bodyId = makeBone(QMustParse("1.2"), QMustParse("-0.135"), QMustParse("0.135"), QMustParse("0.09"), QZero())
	human.bones[conformanceTorso].jointId = addJoint(human.bones[conformanceHip].bodyId, human.bones[conformanceTorso].bodyId, QOne(), QFromRatio(-1, 8), QZero(), QZero(), QHalf())
	human.bones[conformanceHead].bodyId = makeBone(QMustParse("1.475"), QMustParse("-0.038"), QMustParse("0.039"), QMustParse("0.075"), QMustParse("0.1"))
	human.bones[conformanceHead].jointId = addJoint(human.bones[conformanceTorso].bodyId, human.bones[conformanceHead].bodyId, QMustParse("1.4"), QFromRatio(-3, 20), QFromRatio(1, 20), QZero(), QFromRatio(1, 4))
	human.bones[conformanceUpperLeftLeg].bodyId = makeBone(QMustParse("0.775"), QMustParse("-0.125"), QMustParse("0.125"), QMustParse("0.06"), QZero())
	human.bones[conformanceUpperLeftLeg].jointId = addJoint(human.bones[conformanceHip].bodyId, human.bones[conformanceUpperLeftLeg].bodyId, QMustParse("0.9"), QFromRatio(-1, 40), QFromRatio(1, 5), QZero(), QOne())

	footHull := ComputeHull([]Vec2{
		{X: QMustParse("-0.03").Mul(scale), Y: QMustParse("-0.185").Mul(scale)},
		{X: QMustParse("0.11").Mul(scale), Y: QMustParse("-0.185").Mul(scale)},
		{X: QMustParse("0.11").Mul(scale), Y: QMustParse("-0.16").Mul(scale)},
		{X: QMustParse("-0.03").Mul(scale), Y: QMustParse("-0.14").Mul(scale)},
	})
	footPolygon := MakePolygon(&footHull, QMustParse("0.015").Mul(scale))
	human.bones[conformanceLowerLeftLeg].bodyId = makeBone(QMustParse("0.475"), QMustParse("-0.155"), QMustParse("0.125"), QMustParse("0.045"), QZero())
	CreatePolygonShape(human.bones[conformanceLowerLeftLeg].bodyId, &footShapeDef, &footPolygon)
	human.bones[conformanceLowerLeftLeg].jointId = addJoint(human.bones[conformanceUpperLeftLeg].bodyId, human.bones[conformanceLowerLeftLeg].bodyId, QMustParse("0.625"), QFromRatio(-1, 4), QFromRatio(-1, 100), QZero(), QHalf())
	human.bones[conformanceUpperRightLeg].bodyId = makeBone(QMustParse("0.775"), QMustParse("-0.125"), QMustParse("0.125"), QMustParse("0.06"), QZero())
	human.bones[conformanceUpperRightLeg].jointId = addJoint(human.bones[conformanceHip].bodyId, human.bones[conformanceUpperRightLeg].bodyId, QMustParse("0.9"), QFromRatio(-1, 40), QFromRatio(1, 5), QZero(), QOne())
	human.bones[conformanceLowerRightLeg].bodyId = makeBone(QMustParse("0.475"), QMustParse("-0.155"), QMustParse("0.125"), QMustParse("0.045"), QZero())
	CreatePolygonShape(human.bones[conformanceLowerRightLeg].bodyId, &footShapeDef, &footPolygon)
	human.bones[conformanceLowerRightLeg].jointId = addJoint(human.bones[conformanceUpperRightLeg].bodyId, human.bones[conformanceLowerRightLeg].bodyId, QMustParse("0.625"), QFromRatio(-1, 4), QFromRatio(-1, 100), QZero(), QHalf())
	human.bones[conformanceUpperLeftArm].bodyId = makeBone(QMustParse("1.225"), QMustParse("-0.125"), QMustParse("0.125"), QMustParse("0.035"), QZero())
	human.bones[conformanceUpperLeftArm].jointId = addJoint(human.bones[conformanceTorso].bodyId, human.bones[conformanceUpperLeftArm].bodyId, QMustParse("1.35"), QFromRatio(-1, 20), QFromRatio(2, 5), QZero(), QHalf())
	human.bones[conformanceLowerLeftArm].bodyId = makeBone(QMustParse("0.975"), QMustParse("-0.125"), QMustParse("0.125"), QMustParse("0.03"), QMustParse("0.1"))
	human.bones[conformanceLowerLeftArm].jointId = addJoint(human.bones[conformanceUpperLeftArm].bodyId, human.bones[conformanceLowerLeftArm].bodyId, QMustParse("1.1"), QFromRatio(-1, 10), QFromRatio(3, 20), QFromRatio(1, 8), QMustParse("0.1"))
	human.bones[conformanceUpperRightArm].bodyId = makeBone(QMustParse("1.225"), QMustParse("-0.125"), QMustParse("0.125"), QMustParse("0.035"), QZero())
	human.bones[conformanceUpperRightArm].jointId = addJoint(human.bones[conformanceTorso].bodyId, human.bones[conformanceUpperRightArm].bodyId, QMustParse("1.35"), QFromRatio(-1, 20), QFromRatio(2, 5), QZero(), QHalf())
	human.bones[conformanceLowerRightArm].bodyId = makeBone(QMustParse("0.975"), QMustParse("-0.125"), QMustParse("0.125"), QMustParse("0.03"), QMustParse("0.1"))
	human.bones[conformanceLowerRightArm].jointId = addJoint(human.bones[conformanceUpperRightArm].bodyId, human.bones[conformanceLowerRightArm].bodyId, QMustParse("1.1"), QFromRatio(-1, 10), QFromRatio(3, 20), QFromRatio(1, 8), QMustParse("0.1"))
	return human
}

const (
	conformanceRainRowCount    = 5
	conformanceRainColumnCount = 40
	conformanceRainGroupSize   = 5
)

type conformanceRainGroup struct {
	humans [conformanceRainGroupSize]conformanceHuman
}

type conformanceRainData struct {
	groups      [conformanceRainRowCount * conformanceRainColumnCount]conformanceRainGroup
	gridSize    Q
	gridCount   int
	columnCount int
	columnIndex int
}

func (data *conformanceRainData) createGroup(worldId WorldId, rowIndex, columnIndex int) {
	groupIndex := rowIndex*conformanceRainColumnCount + columnIndex
	span := QFromInt(data.gridCount).Mul(data.gridSize)
	groupDistance := span.Div(QFromInt(conformanceRainColumnCount))
	position := Vec2{
		X: QHalf().Neg().Mul(span).Add(groupDistance.Mul(QFromInt(columnIndex).Add(QHalf()))),
		Y: QFromInt(40).Add(QFromInt(45).Mul(QFromInt(rowIndex))),
	}
	for i := range conformanceRainGroupSize {
		data.groups[groupIndex].humans[i] = createConformanceHuman(worldId, position, QOne(), QMustParse("0.05"), QFromInt(5), QHalf(), i+1)
		position.X = position.X.Add(QHalf())
	}
}

func (data *conformanceRainData) destroyGroup(rowIndex, columnIndex int) {
	groupIndex := rowIndex*conformanceRainColumnCount + columnIndex
	for i := range conformanceRainGroupSize {
		data.groups[groupIndex].humans[i].destroy()
	}
}

func buildConformanceRain(worldId WorldId) conformanceStepFn {
	data := &conformanceRainData{gridSize: QHalf(), gridCount: 500}
	groundDef := DefaultBodyDef()
	groundId := CreateBody(worldId, &groundDef)
	shapeDef := DefaultShapeDef()
	y := QZero()
	for range conformanceRainRowCount {
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
		if data.columnCount < conformanceRainColumnCount {
			for row := range conformanceRainRowCount {
				data.createGroup(worldId, row, data.columnCount)
			}
			data.columnCount++
			return
		}
		for row := range conformanceRainRowCount {
			data.destroyGroup(row, data.columnIndex)
			data.createGroup(worldId, row, data.columnIndex)
		}
		data.columnIndex = (data.columnIndex + 1) % conformanceRainColumnCount
	}
}
