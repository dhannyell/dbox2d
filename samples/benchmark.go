// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/sample_benchmark.cpp of Box2D v3.1.1, plus
// CreateTumbler from shared/benchmarks.c. Debug-only sizes use the
// release values.

package samples

import (
	"math"
	"sort"
	"time"

	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/fixed"
)

func init() {
	RegisterSample("Benchmark", "Barrel", NewBarrel)
	RegisterSample("Benchmark", "Tumbler", NewTumbler)
	RegisterSample("Benchmark", "Many Tumblers", NewManyTumblers)
	RegisterSample("Benchmark", "Large Pyramid", NewLargePyramid)
	RegisterSample("Benchmark", "Many Pyramids", NewManyPyramids)
	RegisterSample("Benchmark", "CreateDestroy", NewCreateDestroy)
	RegisterSample("Benchmark", "Sleep", NewSleep)
	RegisterSample("Benchmark", "Joint Grid", NewJointGrid)
	RegisterSample("Benchmark", "Smash", NewSmash)
	RegisterSample("Benchmark", "Compound", NewCompound)
	RegisterSample("Benchmark", "Kinematic", NewKinematic)
	RegisterSample("Benchmark", "Cast", NewCast)
	RegisterSample("Benchmark", "Spinner", NewSpinner)
	RegisterSample("Benchmark", "Rain", NewRain)
	RegisterSample("Benchmark", "Shape Distance", NewShapeDistance)
	RegisterSample("Benchmark", "Sensor", NewSensor)
}

// tumblerGridCount is the reference's non-debug gridCount.
const tumblerGridCount = 45

// Tumbler spins a hollow box full of small boxes with a motorized revolute
// joint, a stress scene for the broad and narrow phase alike.
type Tumbler struct {
	Base
}

// NewTumbler builds the scene, matching CreateTumbler.
func NewTumbler(ctx *SampleContext) Sample {
	s := &Tumbler{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 1.5, Y: 10}
		ctx.Camera.Zoom = 25 * 0.6
	}

	bodyDef := dbox2d.DefaultBodyDef()
	groundId := dbox2d.CreateBody(s.WorldId, &bodyDef)

	bodyDef.Type = dbox2d.DynamicBody
	bodyDef.Position = dbox2d.Vec2{X: fixed.Q32Zero(), Y: fixed.Q32FromInt(10)}
	bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)

	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Density = fixed.Q32FromInt(50)

	ten, half := fixed.Q32FromInt(10), fixed.Q32Half()
	walls := []struct{ hw, hh, cx, cy fixed.Q32 }{
		{half, ten, ten, fixed.Q32Zero()},
		{half, ten, ten.Neg(), fixed.Q32Zero()},
		{ten, half, fixed.Q32Zero(), ten},
		{ten, half, fixed.Q32Zero(), ten.Neg()},
	}
	for _, w := range walls {
		box := dbox2d.MakeOffsetBox(w.hw, w.hh, dbox2d.Vec2{X: w.cx, Y: w.cy}, dbox2d.RotIdentity())
		dbox2d.CreatePolygonShape(bodyId, &shapeDef, &box)
	}

	// (pi/180)*25 rad/s is 25/360 turns/s.
	motorSpeed := fixed.Q32FromRatio(25, 360)

	jd := dbox2d.DefaultRevoluteJointDef()
	jd.BodyIdA = groundId
	jd.BodyIdB = bodyId
	jd.LocalAnchorA = dbox2d.Vec2{X: fixed.Q32Zero(), Y: fixed.Q32FromInt(10)}
	jd.MotorSpeed = motorSpeed
	jd.MaxMotorTorque = fixed.Q32FromInt(100_000_000)
	jd.EnableMotor = true
	dbox2d.CreateRevoluteJoint(s.WorldId, &jd)

	gridBox := dbox2d.MakeBox(fixed.Q32MustParse("0.125"), fixed.Q32MustParse("0.125"))
	step := fixed.Q32FromRatio(4, 10)
	start := fixed.Q32FromRatio(-2*tumblerGridCount, 10) // -0.2 * gridCount

	gridBodyDef := dbox2d.DefaultBodyDef()
	gridBodyDef.Type = dbox2d.DynamicBody
	gridShapeDef := dbox2d.DefaultShapeDef()

	y := start.Add(fixed.Q32FromInt(10))
	for range tumblerGridCount {
		x := start
		for range tumblerGridCount {
			gridBodyDef.Position = dbox2d.Vec2{X: x, Y: y}
			gridBodyId := dbox2d.CreateBody(s.WorldId, &gridBodyDef)
			dbox2d.CreatePolygonShape(gridBodyId, &gridShapeDef, &gridBox)
			x = x.Add(step)
		}
		y = y.Add(step)
	}

	return s
}

type barrelShapeType int

const (
	barrelCircleShape barrelShapeType = iota
	barrelCapsuleShape
	barrelMixShape
	barrelCompoundShape
	barrelHumanShape
	barrelMaxColumns = 26
	barrelMaxRows    = 150
)

// Barrel is the mixed-shape barrel benchmark scene.
type Barrel struct {
	Base

	bodies      [barrelMaxRows * barrelMaxColumns]dbox2d.BodyId
	humans      [barrelMaxRows * barrelMaxColumns]human
	columnCount int
	rowCount    int
	shapeType   barrelShapeType
}

// NewBarrel builds the barrel benchmark scene.
func NewBarrel(ctx *SampleContext) Sample {
	s := &Barrel{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 8, Y: 53}
		ctx.Camera.Zoom = 25 * 2.35
	}
	ctx.Settings.DrawJoints = false

	{
		gridSize := fixed.Q32One()

		bodyDef := dbox2d.DefaultBodyDef()
		groundID := dbox2d.CreateBody(s.WorldId, &bodyDef)

		shapeDef := dbox2d.DefaultShapeDef()

		y := fixed.Q32Zero()
		x := fixed.Q32FromInt(-40).Mul(gridSize)
		for range 81 {
			box := dbox2d.MakeOffsetBox(
				fixed.Q32Half().Mul(gridSize),
				fixed.Q32Half().Mul(gridSize),
				dbox2d.Vec2{X: x, Y: y},
				dbox2d.RotIdentity(),
			)
			dbox2d.CreatePolygonShape(groundID, &shapeDef, &box)
			x = x.Add(gridSize)
		}

		y = gridSize
		x = fixed.Q32FromInt(-40).Mul(gridSize)
		for range 100 {
			box := dbox2d.MakeOffsetBox(
				fixed.Q32Half().Mul(gridSize),
				fixed.Q32Half().Mul(gridSize),
				dbox2d.Vec2{X: x, Y: y},
				dbox2d.RotIdentity(),
			)
			dbox2d.CreatePolygonShape(groundID, &shapeDef, &box)
			y = y.Add(gridSize)
		}

		y = gridSize
		x = fixed.Q32FromInt(40).Mul(gridSize)
		for range 100 {
			box := dbox2d.MakeOffsetBox(
				fixed.Q32Half().Mul(gridSize),
				fixed.Q32Half().Mul(gridSize),
				dbox2d.Vec2{X: x, Y: y},
				dbox2d.RotIdentity(),
			)
			dbox2d.CreatePolygonShape(groundID, &shapeDef, &box)
			y = y.Add(gridSize)
		}

		segment := dbox2d.Segment{
			Point1: dbox2d.Vec2{X: fixed.Q32FromInt(-800), Y: fixed.Q32FromInt(-80)},
			Point2: dbox2d.Vec2{X: fixed.Q32FromInt(800), Y: fixed.Q32FromInt(-80)},
		}
		dbox2d.CreateSegmentShape(groundID, &shapeDef, &segment)
	}

	s.shapeType = barrelCompoundShape
	s.createScene()

	return s
}

func (s *Barrel) createScene() {
	randomSeed = 42

	for i := range s.bodies {
		if !s.bodies[i].IsNull() {
			dbox2d.DestroyBody(s.bodies[i])
			s.bodies[i] = dbox2d.BodyId{}
		}

		if s.humans[i].isSpawned {
			s.humans[i].destroy()
		}
	}

	s.columnCount = barrelMaxColumns
	s.rowCount = barrelMaxRows
	switch s.shapeType {
	case barrelCompoundShape:
		s.columnCount = 20
	case barrelHumanShape:
		s.rowCount = 30
		s.columnCount = 26
	}

	rad := fixed.Q32Half()
	shift := fixed.Q32MustParse("1.15")
	centerX := shift.Mul(fixed.Q32FromInt(s.columnCount)).Div(fixed.Q32FromInt(2))
	centerY := shift.Div(fixed.Q32FromInt(2))

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	if s.shapeType == barrelMixShape {
		bodyDef.AngularDamping = fixed.Q32MustParse("0.3")
	}

	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Density = fixed.Q32One()
	shapeDef.Material.Friction = fixed.Q32MustParse("0.5")

	capsule := dbox2d.Capsule{
		Center1: dbox2d.Vec2{Y: fixed.Q32MustParse("-0.25")},
		Center2: dbox2d.Vec2{Y: fixed.Q32MustParse("0.25")},
		Radius:  rad,
	}
	circle := dbox2d.Circle{Radius: rad}

	wedgePoints := []dbox2d.Vec2{
		{X: fixed.Q32MustParse("-0.1"), Y: fixed.Q32MustParse("-0.5")},
		{X: fixed.Q32MustParse("0.1"), Y: fixed.Q32MustParse("-0.5")},
		{Y: fixed.Q32MustParse("0.5")},
	}
	wedgeHull := dbox2d.ComputeHull(wedgePoints)
	wedge := dbox2d.MakePolygon(&wedgeHull, fixed.Q32Zero())

	vertices := []dbox2d.Vec2{
		{X: fixed.Q32FromInt(-1)},
		{X: fixed.Q32Half(), Y: fixed.Q32One()},
		{Y: fixed.Q32FromInt(2)},
	}
	hull := dbox2d.ComputeHull(vertices)
	left := dbox2d.MakePolygon(&hull, fixed.Q32Zero())

	vertices[0] = dbox2d.Vec2{X: fixed.Q32One()}
	vertices[1] = dbox2d.Vec2{X: fixed.Q32Half().Neg(), Y: fixed.Q32One()}
	hull = dbox2d.ComputeHull(vertices)
	right := dbox2d.MakePolygon(&hull, fixed.Q32Zero())

	side := fixed.Q32MustParse("-0.1")
	extraY := fixed.Q32Half()
	switch s.shapeType {
	case barrelCompoundShape:
		extraY = fixed.Q32MustParse("0.25")
		side = fixed.Q32MustParse("0.25")
		shift = fixed.Q32FromInt(2)
		centerX = shift.Mul(fixed.Q32FromInt(s.columnCount)).Div(fixed.Q32FromInt(2)).Sub(fixed.Q32One())
	case barrelHumanShape:
		extraY = fixed.Q32Half()
		side = fixed.Q32MustParse("0.55")
		shift = fixed.Q32MustParse("2.5")
		centerX = shift.Mul(fixed.Q32FromInt(s.columnCount)).Div(fixed.Q32FromInt(2))
	}

	index := 0
	yStart := fixed.Q32FromInt(100)
	if s.shapeType == barrelHumanShape {
		yStart = fixed.Q32FromInt(2)
	}

	for i := range s.columnCount {
		x := fixed.Q32FromInt(i).Mul(shift).Sub(centerX)

		for j := range s.rowCount {
			y := fixed.Q32FromInt(j).Mul(shift.Add(extraY)).Add(centerY).Add(yStart)
			bodyDef.Position = dbox2d.Vec2{X: x.Add(side), Y: y}
			side = side.Neg()

			switch s.shapeType {
			case barrelCircleShape:
				s.bodies[index] = dbox2d.CreateBody(s.WorldId, &bodyDef)
				circle.Radius = randomFloatRange(fixed.Q32MustParse("0.25"), fixed.Q32MustParse("0.75"))
				shapeDef.Material.RollingResistance = fixed.Q32MustParse("0.2")
				dbox2d.CreateCircleShape(s.bodies[index], &shapeDef, &circle)
			case barrelCapsuleShape:
				s.bodies[index] = dbox2d.CreateBody(s.WorldId, &bodyDef)
				capsule.Radius = randomFloatRange(fixed.Q32MustParse("0.25"), fixed.Q32MustParse("0.5"))
				length := randomFloatRange(fixed.Q32MustParse("0.25"), fixed.Q32One())
				capsule.Center1 = dbox2d.Vec2{Y: fixed.Q32Half().Mul(length).Neg()}
				capsule.Center2 = dbox2d.Vec2{Y: fixed.Q32Half().Mul(length)}
				shapeDef.Material.RollingResistance = fixed.Q32MustParse("0.2")
				dbox2d.CreateCapsuleShape(s.bodies[index], &shapeDef, &capsule)
			case barrelMixShape:
				s.bodies[index] = dbox2d.CreateBody(s.WorldId, &bodyDef)
				switch index % 3 {
				case 0:
					circle.Radius = randomFloatRange(fixed.Q32MustParse("0.25"), fixed.Q32MustParse("0.75"))
					dbox2d.CreateCircleShape(s.bodies[index], &shapeDef, &circle)
				case 1:
					capsule.Radius = randomFloatRange(fixed.Q32MustParse("0.25"), fixed.Q32MustParse("0.5"))
					length := randomFloatRange(fixed.Q32MustParse("0.25"), fixed.Q32One())
					capsule.Center1 = dbox2d.Vec2{Y: fixed.Q32Half().Mul(length).Neg()}
					capsule.Center2 = dbox2d.Vec2{Y: fixed.Q32Half().Mul(length)}
					dbox2d.CreateCapsuleShape(s.bodies[index], &shapeDef, &capsule)
				case 2:
					width := randomFloatRange(fixed.Q32MustParse("0.1"), fixed.Q32MustParse("0.5"))
					height := randomFloatRange(fixed.Q32MustParse("0.5"), fixed.Q32MustParse("0.75"))
					box := dbox2d.MakeBox(width, height)
					value := randomFloatRange(fixed.Q32FromInt(-1), fixed.Q32One())
					box.Radius = fixed.Q32FromRatio(1, 4).Mul(value.Max(fixed.Q32Zero()))
					dbox2d.CreatePolygonShape(s.bodies[index], &shapeDef, &box)
				default:
					wedge.Radius = randomFloatRange(fixed.Q32MustParse("0.1"), fixed.Q32MustParse("0.25"))
					dbox2d.CreatePolygonShape(s.bodies[index], &shapeDef, &wedge)
				}
			case barrelCompoundShape:
				s.bodies[index] = dbox2d.CreateBody(s.WorldId, &bodyDef)
				dbox2d.CreatePolygonShape(s.bodies[index], &shapeDef, &left)
				dbox2d.CreatePolygonShape(s.bodies[index], &shapeDef, &right)
			case barrelHumanShape:
				scale := fixed.Q32MustParse("3.5")
				jointFriction := fixed.Q32MustParse("0.05")
				jointHertz := fixed.Q32FromInt(5)
				jointDamping := fixed.Q32Half()
				s.humans[index] = createHuman(
					s.WorldId, bodyDef.Position, scale, jointFriction, jointHertz, jointDamping,
					index+1, nil, false,
				)
			}

			index++
		}
	}
}

// UpdateGui exposes the barrel shape and reset controls.
func (s *Barrel) UpdateGui() {
	height := 80
	gui := s.Context.Gui
	gui.Begin("Benchmark: Barrel", 10, s.Context.Camera.Height-height-50, 220, height)

	changed := false
	shapeTypes := []string{"Circle", "Capsule", "Mix", "Compound", "Human"}
	shapeType := int(s.shapeType)
	changed = changed || gui.Combo("Shape", &shapeType, shapeTypes)
	s.shapeType = barrelShapeType(shapeType)
	changed = changed || gui.Button("Reset Scene")
	if changed {
		s.createScene()
	}

	gui.End()
}

// ManyTumblers is a grid of independently rotating tumblers.
type ManyTumblers struct {
	Base

	groundID     dbox2d.BodyId
	rowCount     int
	columnCount  int
	tumblerIDs   []dbox2d.BodyId
	positions    []dbox2d.Vec2
	tumblerCount int
	bodyIDs      []dbox2d.BodyId
	bodyCount    int
	bodyIndex    int
	angularSpeed float64
}

// NewManyTumblers builds the many-tumblers benchmark scene.
// angularVelocity converts the degrees-per-second slider to turns per second.
func (s *ManyTumblers) angularVelocity() dbox2d.Q {
	return FromFloat64(s.angularSpeed).Div(fixed.Q32FromInt(360))
}

func NewManyTumblers(ctx *SampleContext) Sample {
	s := &ManyTumblers{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 1, Y: -5.5}
		ctx.Camera.Zoom = 25 * 3.4
		ctx.Settings.DrawJoints = false
	}

	bodyDef := dbox2d.DefaultBodyDef()
	s.groundID = dbox2d.CreateBody(s.WorldId, &bodyDef)

	s.rowCount = 19
	s.columnCount = 19
	s.angularSpeed = 25
	s.createScene()

	return s
}

func (s *ManyTumblers) createTumbler(position dbox2d.Vec2, index int) {
	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.KinematicBody
	bodyDef.Position = position
	bodyDef.AngularVelocity = s.angularVelocity()
	bodyID := dbox2d.CreateBody(s.WorldId, &bodyDef)
	s.tumblerIDs[index] = bodyID

	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Density = fixed.Q32FromInt(50)

	walls := []struct {
		halfWidth, halfHeight fixed.Q32
		center                dbox2d.Vec2
	}{
		{fixed.Q32MustParse("0.25"), fixed.Q32FromInt(2), dbox2d.Vec2{X: fixed.Q32FromInt(2)}},
		{fixed.Q32MustParse("0.25"), fixed.Q32FromInt(2), dbox2d.Vec2{X: fixed.Q32FromInt(-2)}},
		{fixed.Q32FromInt(2), fixed.Q32MustParse("0.25"), dbox2d.Vec2{Y: fixed.Q32FromInt(2)}},
		{fixed.Q32FromInt(2), fixed.Q32MustParse("0.25"), dbox2d.Vec2{Y: fixed.Q32FromInt(-2)}},
	}
	for _, wall := range walls {
		polygon := dbox2d.MakeOffsetBox(wall.halfWidth, wall.halfHeight, wall.center, dbox2d.RotIdentity())
		dbox2d.CreatePolygonShape(bodyID, &shapeDef, &polygon)
	}
}

func (s *ManyTumblers) createScene() {
	for i := range s.bodyCount {
		if !s.bodyIDs[i].IsNull() {
			dbox2d.DestroyBody(s.bodyIDs[i])
		}
	}

	for i := range s.tumblerCount {
		dbox2d.DestroyBody(s.tumblerIDs[i])
	}

	s.tumblerCount = s.rowCount * s.columnCount
	s.tumblerIDs = make([]dbox2d.BodyId, s.tumblerCount)
	s.positions = make([]dbox2d.Vec2, s.tumblerCount)

	index := 0
	x := fixed.Q32FromInt(-4 * s.rowCount)
	for range s.rowCount {
		y := fixed.Q32FromInt(-4 * s.columnCount)
		for range s.columnCount {
			s.positions[index] = dbox2d.Vec2{X: x, Y: y}
			s.createTumbler(s.positions[index], index)
			index++
			y = y.Add(fixed.Q32FromInt(8))
		}
		x = x.Add(fixed.Q32FromInt(8))
	}

	bodiesPerTumbler := 50
	s.bodyCount = bodiesPerTumbler * s.tumblerCount
	s.bodyIDs = make([]dbox2d.BodyId, s.bodyCount)
	s.bodyIndex = 0
}

// UpdateGui exposes the grid dimensions and angular speed controls.
func (s *ManyTumblers) UpdateGui() {
	height := 110
	gui := s.Context.Gui
	gui.Begin("Benchmark: Many Tumblers", 10, s.Context.Camera.Height-height-50, 200, height)

	changed := false
	changed = changed || gui.SliderInt("Row Count", &s.rowCount, 1, 32)
	changed = changed || gui.SliderInt("Column Count", &s.columnCount, 1, 32)
	if changed {
		s.createScene()
	}

	if gui.SliderFloat("Speed", &s.angularSpeed, 0, 100) {
		angularVelocity := s.angularVelocity()
		for i := range s.tumblerCount {
			s.tumblerIDs[i].SetAngularVelocity(angularVelocity)
			s.tumblerIDs[i].SetAwake(true)
		}
	}

	gui.End()
}

// Step advances the world and gradually fills each tumbler with capsules.
func (s *ManyTumblers) Step() {
	s.Base.Step()

	if s.bodyIndex < s.bodyCount && s.StepCount&0x7 == 0 {
		shapeDef := dbox2d.DefaultShapeDef()
		capsule := dbox2d.Capsule{
			Center1: dbox2d.Vec2{X: fixed.Q32MustParse("-0.1")},
			Center2: dbox2d.Vec2{X: fixed.Q32MustParse("0.1")},
			Radius:  fixed.Q32MustParse("0.075"),
		}

		for i := range s.tumblerCount {
			bodyDef := dbox2d.DefaultBodyDef()
			bodyDef.Type = dbox2d.DynamicBody
			bodyDef.Position = s.positions[i]
			s.bodyIDs[s.bodyIndex] = dbox2d.CreateBody(s.WorldId, &bodyDef)
			dbox2d.CreateCapsuleShape(s.bodyIDs[s.bodyIndex], &shapeDef, &capsule)
			s.bodyIndex++
		}
	}
}

// LargePyramid creates the release-sized pyramid benchmark scene.
type LargePyramid struct {
	Base
}

// NewLargePyramid builds the large pyramid benchmark scene.
func NewLargePyramid(ctx *SampleContext) Sample {
	s := &LargePyramid{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 50}
		ctx.Camera.Zoom = 25 * 2.2
		ctx.Settings.EnableSleep = false
	}

	createLargePyramid(s.WorldId)
	return s
}

// ManyPyramids creates the release-sized many-pyramids benchmark scene.
type ManyPyramids struct {
	Base
}

// NewManyPyramids builds the many pyramids benchmark scene.
func NewManyPyramids(ctx *SampleContext) Sample {
	s := &ManyPyramids{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 16, Y: 110}
		ctx.Camera.Zoom = 25 * 5.0
		ctx.Settings.EnableSleep = false
	}

	createManyPyramids(s.WorldId)
	return s
}

const maxBaseCount = 100

// CreateDestroy measures rebuilding a pyramid of dynamic bodies.
type CreateDestroy struct {
	Base

	bodies      []dbox2d.BodyId
	bodyCount   int
	baseCount   int
	iterations  int
	createTime  float64
	destroyTime float64
}

// NewCreateDestroy builds the create/destroy benchmark ground.
func NewCreateDestroy(ctx *SampleContext) Sample {
	s := &CreateDestroy{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 50}
		ctx.Camera.Zoom = 25 * 2.2
	}

	bodyDef := dbox2d.DefaultBodyDef()
	groundID := dbox2d.CreateBody(s.WorldId, &bodyDef)
	ground := dbox2d.MakeBox(fixed.Q32FromInt(100), fixed.Q32One())
	shapeDef := dbox2d.DefaultShapeDef()
	dbox2d.CreatePolygonShape(groundID, &shapeDef, &ground)

	maxBodyCount := maxBaseCount * (maxBaseCount + 1) / 2
	s.bodies = make([]dbox2d.BodyId, maxBodyCount)
	s.createTime = 0
	s.destroyTime = 0
	s.baseCount = maxBaseCount
	s.iterations = 10
	s.bodyCount = 0

	return s
}

// CreateScene destroys the previous pyramid, creates the next one, and steps it once.
func (s *CreateDestroy) CreateScene() {
	timer := time.Now()
	for i := range s.bodies {
		if !s.bodies[i].IsNull() {
			dbox2d.DestroyBody(s.bodies[i])
			s.bodies[i] = dbox2d.BodyId{}
		}
	}
	s.destroyTime += time.Since(timer).Seconds() * 1000

	count := s.baseCount
	rad := fixed.Q32Half()
	shift := rad.Mul(fixed.Q32FromInt(2))
	centerX := shift.Mul(fixed.Q32FromInt(count)).Div(fixed.Q32FromInt(2))
	centerY := shift.Div(fixed.Q32FromInt(2)).Add(fixed.Q32One())

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody

	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Density = fixed.Q32One()
	shapeDef.Material.Friction = fixed.Q32MustParse("0.5")

	box := dbox2d.MakeRoundedBox(fixed.Q32Half(), fixed.Q32Half(), fixed.Q32Zero())
	index := 0

	timer = time.Now()
	for i := range count {
		y := fixed.Q32FromInt(i).Mul(shift).Add(centerY)

		for j := i; j < count; j++ {
			x := fixed.Q32FromRatio(i, 2).Mul(shift).
				Add(fixed.Q32FromInt(j - i).Mul(shift)).
				Sub(centerX)
			bodyDef.Position = dbox2d.Vec2{X: x, Y: y}

			s.bodies[index] = dbox2d.CreateBody(s.WorldId, &bodyDef)
			dbox2d.CreatePolygonShape(s.bodies[index], &shapeDef, &box)
			index++
		}
	}
	s.createTime += time.Since(timer).Seconds() * 1000

	s.bodyCount = index
	s.WorldId.Step(fixed.Q32FromRatio(1, 60), 4)
}

// Step rebuilds the pyramid repeatedly and reports per-body timings.
func (s *CreateDestroy) Step() {
	s.createTime = 0
	s.destroyTime = 0

	for range s.iterations {
		s.CreateScene()
	}

	s.DrawTextLine("total: create = %g ms, destroy = %g ms", s.createTime, s.destroyTime)
	createPerBody := 1000 * s.createTime / float64(s.iterations) / float64(s.bodyCount)
	destroyPerBody := 1000 * s.destroyTime / float64(s.iterations) / float64(s.bodyCount)
	s.DrawTextLine("body: create = %g us, destroy = %g us", createPerBody, destroyPerBody)

	s.Base.Step()
}

// Sleep measures repeatedly waking and sleeping the first body in a pyramid.
type Sleep struct {
	Base

	bodies     []dbox2d.BodyId
	bodyCount  int
	baseCount  int
	iterations int
	awake      bool
	wakeTotal  float64
	sleepTotal float64
	wakeCount  int
	sleepCount int
}

// NewSleep builds the sleep benchmark pyramid.
func NewSleep(ctx *SampleContext) Sample {
	s := &Sleep{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 50}
		ctx.Camera.Zoom = 25 * 2.2
	}

	bodyDef := dbox2d.DefaultBodyDef()
	groundID := dbox2d.CreateBody(s.WorldId, &bodyDef)
	ground := dbox2d.MakeBox(fixed.Q32FromInt(100), fixed.Q32One())
	shapeDef := dbox2d.DefaultShapeDef()
	dbox2d.CreatePolygonShape(groundID, &shapeDef, &ground)

	maxBodyCount := maxBaseCount * (maxBaseCount + 1) / 2
	s.bodies = make([]dbox2d.BodyId, maxBodyCount)
	s.baseCount = maxBaseCount
	s.iterations = 41
	s.bodyCount = 0
	s.awake = false
	s.wakeTotal = 0
	s.wakeCount = 0
	s.sleepTotal = 0
	s.sleepCount = 0

	s.CreateScene()
	return s
}

// CreateScene rebuilds the sleep benchmark pyramid.
func (s *Sleep) CreateScene() {
	for i := range s.bodies {
		if !s.bodies[i].IsNull() {
			dbox2d.DestroyBody(s.bodies[i])
			s.bodies[i] = dbox2d.BodyId{}
		}
	}

	count := s.baseCount
	rad := fixed.Q32Half()
	shift := rad.Mul(fixed.Q32FromInt(2))
	centerX := shift.Mul(fixed.Q32FromInt(count)).Div(fixed.Q32FromInt(2))
	centerY := shift.Div(fixed.Q32FromInt(2)).Add(fixed.Q32One())

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody

	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Density = fixed.Q32One()
	shapeDef.Material.Friction = fixed.Q32MustParse("0.5")

	box := dbox2d.MakeRoundedBox(fixed.Q32Half(), fixed.Q32Half(), fixed.Q32Zero())
	index := 0

	for i := range count {
		y := fixed.Q32FromInt(i).Mul(shift).Add(centerY)

		for j := i; j < count; j++ {
			x := fixed.Q32FromRatio(i, 2).Mul(shift).
				Add(fixed.Q32FromInt(j - i).Mul(shift)).
				Sub(centerX)
			bodyDef.Position = dbox2d.Vec2{X: x, Y: y}

			s.bodies[index] = dbox2d.CreateBody(s.WorldId, &bodyDef)
			dbox2d.CreatePolygonShape(s.bodies[index], &shapeDef, &box)
			index++
		}
	}

	s.bodyCount = index
}

// Step toggles the first pyramid body and reports wake and sleep timings.
func (s *Sleep) Step() {
	for range s.iterations {
		timer := time.Now()
		s.bodies[0].SetAwake(s.awake)
		elapsed := time.Since(timer).Seconds() * 1000

		if s.awake {
			s.wakeTotal += elapsed
			s.wakeCount++
		} else {
			s.sleepTotal += elapsed
			s.sleepCount++
		}
		s.awake = !s.awake
	}

	if s.wakeCount > 0 {
		s.DrawTextLine("wake ave = %g ms", s.wakeTotal/float64(s.wakeCount))
	}
	if s.sleepCount > 0 {
		s.DrawTextLine("sleep ave = %g ms", s.sleepTotal/float64(s.sleepCount))
	}

	s.Base.Step()
}

// JointGrid creates the release-sized joint grid benchmark.
type JointGrid struct {
	Base
}

// NewJointGrid builds the joint grid benchmark scene.
func NewJointGrid(ctx *SampleContext) Sample {
	s := &JointGrid{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 60, Y: -57}
		ctx.Camera.Zoom = 25 * 2.5
		ctx.Settings.EnableSleep = false
	}

	createJointGrid(s.WorldId)
	return s
}

// Smash is the release-sized high-speed impact benchmark scene.
type Smash struct {
	Base
}

// NewSmash builds the smash benchmark scene.
func NewSmash(ctx *SampleContext) Sample {
	s := &Smash{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 60, Y: 6}
		ctx.Camera.Zoom = 25 * 1.6
	}

	createSmash(s.WorldId)
	return s
}

// Compound builds the compound-shape stress benchmark scene.
type Compound struct {
	Base
}

// NewCompound builds the compound benchmark scene.
func NewCompound(ctx *SampleContext) Sample {
	s := &Compound{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 18, Y: 115}
		ctx.Camera.Zoom = 25 * 5.5
	}

	grid := fixed.Q32One()
	height := 200
	width := 200

	{
		bodyDef := dbox2d.DefaultBodyDef()
		groundID := dbox2d.CreateBody(s.WorldId, &bodyDef)
		shapeDef := dbox2d.DefaultShapeDef()

		for i := range height {
			y := fixed.Q32FromInt(i).Mul(grid)
			for j := i; j < width; j++ {
				x := fixed.Q32FromInt(j).Mul(grid)
				square := dbox2d.MakeOffsetBox(
					fixed.Q32Half().Mul(grid),
					fixed.Q32Half().Mul(grid),
					dbox2d.Vec2{X: x, Y: y},
					dbox2d.RotIdentity(),
				)
				dbox2d.CreatePolygonShape(groundID, &shapeDef, &square)
			}
		}

		for i := range height {
			y := fixed.Q32FromInt(i).Mul(grid)
			for j := i; j < width; j++ {
				x := fixed.Q32FromInt(-j).Mul(grid)
				square := dbox2d.MakeOffsetBox(
					fixed.Q32Half().Mul(grid),
					fixed.Q32Half().Mul(grid),
					dbox2d.Vec2{X: x, Y: y},
					dbox2d.RotIdentity(),
				)
				dbox2d.CreatePolygonShape(groundID, &shapeDef, &square)
			}
		}
	}

	{
		span := 20
		count := 5

		bodyDef := dbox2d.DefaultBodyDef()
		bodyDef.Type = dbox2d.DynamicBody
		// Defer mass properties to avoid n-squared mass computations.
		shapeDef := dbox2d.DefaultShapeDef()
		shapeDef.UpdateBodyMass = false

		for m := range count {
			yBody := fixed.Q32FromInt(100 + m*span).Mul(grid)

			for n := range count {
				xBody := fixed.Q32Half().Neg().Mul(grid).
					Mul(fixed.Q32FromInt(count)).
					Mul(fixed.Q32FromInt(span)).
					Add(fixed.Q32FromInt(n * span).Mul(grid))
				bodyDef.Position = dbox2d.Vec2{X: xBody, Y: yBody}
				bodyID := dbox2d.CreateBody(s.WorldId, &bodyDef)

				for i := range span {
					y := fixed.Q32FromInt(i).Mul(grid)
					for j := range span {
						x := fixed.Q32FromInt(j).Mul(grid)
						square := dbox2d.MakeOffsetBox(
							fixed.Q32Half().Mul(grid),
							fixed.Q32Half().Mul(grid),
							dbox2d.Vec2{X: x, Y: y},
							dbox2d.RotIdentity(),
						)
						dbox2d.CreatePolygonShape(bodyID, &shapeDef, &square)
					}
				}

				bodyID.ApplyMassFromShapes()
			}
		}
	}

	return s
}

// Kinematic builds the release-sized rotating compound benchmark scene.
type Kinematic struct {
	Base
}

// NewKinematic builds the kinematic benchmark scene.
func NewKinematic(ctx *SampleContext) Sample {
	s := &Kinematic{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 0}
		ctx.Camera.Zoom = 150
	}

	grid := fixed.Q32One()
	span := 100

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.KinematicBody
	// The reference uses 1 rad/s; body angular velocity is stored in turns/s.
	bodyDef.AngularVelocity = fixed.Q32MustParse("0.1591549431")

	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Filter.CategoryBits = 1
	shapeDef.Filter.MaskBits = 2
	// Defer mass properties to avoid n-squared mass computations.
	shapeDef.UpdateBodyMass = false

	bodyID := dbox2d.CreateBody(s.WorldId, &bodyDef)

	for i := -span; i < span; i++ {
		y := fixed.Q32FromInt(i).Mul(grid)
		for j := -span; j < span; j++ {
			x := fixed.Q32FromInt(j).Mul(grid)
			square := dbox2d.MakeOffsetBox(
				fixed.Q32Half().Mul(grid),
				fixed.Q32Half().Mul(grid),
				dbox2d.Vec2{X: x, Y: y},
				dbox2d.RotIdentity(),
			)
			dbox2d.CreatePolygonShape(bodyID, &shapeDef, &square)
		}
	}

	bodyID.ApplyMassFromShapes()
	return s
}

type castQueryType int

const (
	castRay castQueryType = iota
	castCircle
	castOverlap
)

type castResult struct {
	point    dbox2d.Vec2
	fraction dbox2d.Q
	hit      bool
}

type overlapResult struct {
	points [32]dbox2d.Vec2
	count  int
}

// Cast benchmarks ray, circle, and AABB queries against a large static scene.
type Cast struct {
	Base

	queryType    castQueryType
	origins      []dbox2d.Vec2
	translations []dbox2d.Vec2
	minTime      float64
	buildTime    float64
	rowCount     int
	columnCount  int
	drawIndex    int
	radius       dbox2d.Q
	fill         dbox2d.Q
	ratio        dbox2d.Q
	grid         dbox2d.Q
	topDown      bool
}

// NewCast builds the release-sized cast benchmark scene.
func NewCast(ctx *SampleContext) Sample {
	s := &Cast{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 500, Y: 500}
		ctx.Camera.Zoom = 25 * 21
	}

	s.queryType = castCircle
	s.ratio = fixed.Q32FromInt(5)
	s.grid = fixed.Q32One()
	s.fill = fixed.Q32MustParse("0.1")
	s.rowCount = 1000    // release value; BENCHMARK_DEBUG value was 100
	s.columnCount = 1000 // release value; BENCHMARK_DEBUG value was 100
	s.minTime = 1e6
	s.drawIndex = 0
	s.topDown = false
	s.buildTime = 0
	s.radius = fixed.Q32MustParse("0.1")

	randomSeed = 1234
	sampleCount := 10000 // release value; BENCHMARK_DEBUG value was 100
	s.origins = make([]dbox2d.Vec2, sampleCount)
	s.translations = make([]dbox2d.Vec2, sampleCount)
	extent := fixed.Q32FromInt(s.rowCount).Mul(s.grid)

	// Precompute rays so each step measures queries instead of randomization.
	for i := range sampleCount {
		rayStart := randomVec2(fixed.Q32Zero(), extent)
		rayEnd := randomVec2(fixed.Q32Zero(), extent)
		s.origins[i] = rayStart
		s.translations[i] = rayEnd.Sub(rayStart)
	}

	s.buildScene()
	return s
}

func (s *Cast) buildScene() {
	randomSeed = 1234
	started := time.Now()
	s.CreateWorld()

	bodyDef := dbox2d.DefaultBodyDef()
	shapeDef := dbox2d.DefaultShapeDef()

	for i := range s.rowCount {
		y := fixed.Q32FromInt(i).Mul(s.grid)
		for j := range s.columnCount {
			x := fixed.Q32FromInt(j).Mul(s.grid)
			fillTest := randomFloatRange(fixed.Q32Zero(), fixed.Q32One())
			if !s.fill.Less(fillTest) {
				bodyDef.Position = dbox2d.Vec2{X: x, Y: y}
				bodyID := dbox2d.CreateBody(s.WorldId, &bodyDef)

				ratio := randomFloatRange(fixed.Q32One(), s.ratio)
				halfWidth := randomFloatRange(fixed.Q32MustParse("0.05"), fixed.Q32MustParse("0.25"))
				var box dbox2d.Polygon
				if randomFloat().Greater(fixed.Q32Zero()) {
					box = dbox2d.MakeBox(ratio.Mul(halfWidth), halfWidth)
				} else {
					box = dbox2d.MakeBox(halfWidth, ratio.Mul(halfWidth))
				}

				category := randomIntRange(0, 2)
				shapeDef.Filter.CategoryBits = uint64(1 << category)
				switch category {
				case 0:
					shapeDef.Material.CustomColor = uint32(dbox2d.ColorBox2DBlue)
				case 1:
					shapeDef.Material.CustomColor = uint32(dbox2d.ColorBox2DYellow)
				default:
					shapeDef.Material.CustomColor = uint32(dbox2d.ColorBox2DGreen)
				}

				dbox2d.CreatePolygonShape(bodyID, &shapeDef, &box)
			}

		}
	}

	if s.topDown {
		s.WorldId.RebuildStaticTree()
	}

	s.buildTime = time.Since(started).Seconds() * 1000
	s.minTime = 1e6
}

// UpdateGui exposes the query and static-scene controls in reference order.
func (s *Cast) UpdateGui() {
	height := 240
	gui := s.Context.Gui
	gui.Begin("Cast", 10, s.Context.Camera.Height-height-50, 200, height)

	changed := false
	queryTypes := []string{"Ray", "Circle", "Overlap"}
	queryType := int(s.queryType)
	// Every widget draws each frame, so no short-circuit here.
	if gui.Combo("Query", &queryType, queryTypes) {
		s.queryType = castQueryType(queryType)
		if s.queryType == castOverlap {
			s.radius = fixed.Q32FromInt(5)
		} else {
			s.radius = fixed.Q32MustParse("0.1")
		}
		changed = true
	}

	if gui.SliderInt("rows", &s.rowCount, 0, 1000) {
		changed = true
	}
	if gui.SliderInt("columns", &s.columnCount, 0, 1000) {
		changed = true
	}

	fill := ToFloat64(s.fill)
	if gui.SliderFloat("fill", &fill, 0, 1) {
		s.fill = FromFloat64(fill)
		changed = true
	}

	grid := ToFloat64(s.grid)
	if gui.SliderFloat("grid", &grid, 0.5, 2) {
		s.grid = FromFloat64(grid)
		changed = true
	}

	ratio := ToFloat64(s.ratio)
	if gui.SliderFloat("ratio", &ratio, 1, 10) {
		s.ratio = FromFloat64(ratio)
		changed = true
	}

	if gui.Checkbox("top down", &s.topDown) {
		changed = true
	}

	if gui.Button("Draw Next") {
		s.drawIndex = (s.drawIndex + 1) % len(s.origins)
	}

	gui.End()

	if changed {
		s.buildScene()
	}
}

func (s *Cast) Step() {
	s.Base.Step()

	filter := dbox2d.DefaultQueryFilter()
	filter.MaskBits = 1
	hitCount := 0
	nodeVisits := 0
	leafVisits := 0
	ms := 0.0
	sampleCount := len(s.origins)

	switch s.queryType {
	case castRay:
		started := time.Now()
		var drawResult dbox2d.RayResult

		for i := range sampleCount {
			result := s.WorldId.CastRayClosest(s.origins[i], s.translations[i], filter)
			if i == s.drawIndex {
				drawResult = result
			}
			nodeVisits += result.NodeVisits
			leafVisits += result.LeafVisits
			if result.Hit {
				hitCount++
			}
		}

		ms = time.Since(started).Seconds() * 1000
		if ms < s.minTime {
			s.minTime = ms
		}

		p1 := s.origins[s.drawIndex]
		p2 := p1.Add(s.translations[s.drawIndex])
		s.Context.Draw.DrawSegment(p1, p2, dbox2d.ColorWhite)
		s.Context.Draw.DrawPoint(p1, fixed.Q32FromInt(5), dbox2d.ColorGreen)
		s.Context.Draw.DrawPoint(p2, fixed.Q32FromInt(5), dbox2d.ColorRed)
		if drawResult.Hit {
			s.Context.Draw.DrawPoint(drawResult.Point, fixed.Q32FromInt(5), dbox2d.ColorWhite)
		}

	case castCircle:
		started := time.Now()
		var drawResult, result castResult

		for i := range sampleCount {
			proxy := dbox2d.MakeProxy([]dbox2d.Vec2{s.origins[i]}, s.radius)
			result.hit = false
			traversalResult := s.WorldId.CastShape(&proxy, s.translations[i], filter, func(_ dbox2d.ShapeId, point, _ dbox2d.Vec2, fraction dbox2d.Q) dbox2d.Q {
				result.point = point
				result.fraction = fraction
				result.hit = true
				return fraction
			})

			if i == s.drawIndex {
				drawResult = result
			}
			nodeVisits += traversalResult.NodeVisits
			leafVisits += traversalResult.LeafVisits
			if result.hit {
				hitCount++
			}
		}

		ms = time.Since(started).Seconds() * 1000
		if ms < s.minTime {
			s.minTime = ms
		}

		p1 := s.origins[s.drawIndex]
		p2 := p1.Add(s.translations[s.drawIndex])
		s.Context.Draw.DrawSegment(p1, p2, dbox2d.ColorWhite)
		s.Context.Draw.DrawPoint(p1, fixed.Q32FromInt(5), dbox2d.ColorGreen)
		s.Context.Draw.DrawPoint(p2, fixed.Q32FromInt(5), dbox2d.ColorRed)
		if drawResult.hit {
			t := dbox2d.Lerp(p1, p2, drawResult.fraction)
			s.Context.Draw.DrawCircle(t, s.radius, dbox2d.ColorWhite)
			s.Context.Draw.DrawPoint(drawResult.point, fixed.Q32FromInt(5), dbox2d.ColorWhite)
		}

	case castOverlap:
		started := time.Now()
		var drawResult, result overlapResult
		extent := dbox2d.Vec2{X: s.radius, Y: s.radius}

		for i := range sampleCount {
			origin := s.origins[i]
			aabb := dbox2d.AABB{LowerBound: origin.Sub(extent), UpperBound: origin.Add(extent)}
			result.count = 0
			traversalResult := s.WorldId.OverlapAABB(aabb, filter, func(shapeID dbox2d.ShapeId) bool {
				if result.count < len(result.points) {
					result.points[result.count] = dbox2d.AABBCenter(shapeID.GetAABB())
					result.count++
				}
				return true
			})

			if i == s.drawIndex {
				drawResult = result
			}
			nodeVisits += traversalResult.NodeVisits
			leafVisits += traversalResult.LeafVisits
			hitCount += result.count
		}

		ms = time.Since(started).Seconds() * 1000
		if ms < s.minTime {
			s.minTime = ms
		}

		origin := s.origins[s.drawIndex]
		aabb := dbox2d.AABB{LowerBound: origin.Sub(extent), UpperBound: origin.Add(extent)}
		s.Context.Draw.DrawPolygon([]dbox2d.Vec2{
			aabb.LowerBound,
			dbox2d.Vec2{X: aabb.UpperBound.X, Y: aabb.LowerBound.Y},
			aabb.UpperBound,
			dbox2d.Vec2{X: aabb.LowerBound.X, Y: aabb.UpperBound.Y},
		}, dbox2d.ColorWhite)
		for i := range drawResult.count {
			s.Context.Draw.DrawPoint(drawResult.points[i], fixed.Q32FromInt(5), dbox2d.ColorHotPink)
		}
	}

	s.DrawTextLine("build time ms = %g", s.buildTime)
	s.DrawTextLine("hit count = %d, node visits = %d, leaf visits = %d", hitCount, nodeVisits, leafVisits)
	s.DrawTextLine("total ms = %.3f", ms)
	s.DrawTextLine("min total ms = %.3f", s.minTime)

	averageMicroseconds := 1000 * s.minTime / float64(sampleCount)
	s.DrawTextLine("average us = %.2f", averageMicroseconds)
}

// Spinner is the release-sized rotating benchmark scene.
type Spinner struct {
	Base
}

// NewSpinner builds the spinner benchmark scene.
func NewSpinner(ctx *SampleContext) Sample {
	s := &Spinner{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 32}
		ctx.Camera.Zoom = 42
	}

	createSpinner(s.WorldId)
	return s
}

// Rain is the release-sized falling-humans benchmark scene.
type Rain struct {
	Base
}

// NewRain builds the rain benchmark scene.
func NewRain(ctx *SampleContext) Sample {
	s := &Rain{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 110}
		ctx.Camera.Zoom = 125
		ctx.Settings.EnableSleep = true
	}
	ctx.Settings.DrawJoints = false

	createRain(s.WorldId)
	return s
}

func (s *Rain) Step() {
	if !s.Context.Settings.Pause || s.Context.Settings.SingleStep {
		stepRain(s.WorldId, s.StepCount)
	}

	// The reference's m_stepCount % 1000 == 0 branch only added zero.
	s.Base.Step()
}

const shapeDistanceCount = 10000

// ShapeDistance benchmarks repeated distance queries between two polygons.
type ShapeDistance struct {
	Base

	polygonA        dbox2d.Polygon
	polygonB        dbox2d.Polygon
	transformAs     []dbox2d.Transform
	transformBs     []dbox2d.Transform
	outputs         []dbox2d.DistanceOutput
	minMilliseconds float64
	drawIndex       int
}

// NewShapeDistance builds the shape-distance benchmark scene.
func NewShapeDistance(ctx *SampleContext) Sample {
	s := &ShapeDistance{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 0}
		ctx.Camera.Zoom = 3
	}

	{
		points := make([]dbox2d.Vec2, 8)
		q := dbox2d.MakeRot(fixed.Q32FromRatio(1, 8))
		points[0] = dbox2d.Vec2{X: fixed.Q32Half()}
		for i := 1; i < len(points); i++ {
			points[i] = dbox2d.RotateVector(q, points[i-1])
		}
		hull := dbox2d.ComputeHull(points)
		s.polygonA = dbox2d.MakePolygon(&hull, fixed.Q32Zero())
	}

	{
		points := make([]dbox2d.Vec2, 8)
		q := dbox2d.MakeRot(fixed.Q32FromRatio(1, 8))
		points[0] = dbox2d.Vec2{X: fixed.Q32Half()}
		for i := 1; i < len(points); i++ {
			points[i] = dbox2d.RotateVector(q, points[i-1])
		}
		hull := dbox2d.ComputeHull(points)
		s.polygonB = dbox2d.MakePolygon(&hull, fixed.Q32MustParse("0.1"))
	}

	s.transformAs = make([]dbox2d.Transform, shapeDistanceCount)
	s.transformBs = make([]dbox2d.Transform, shapeDistanceCount)
	s.outputs = make([]dbox2d.DistanceOutput, shapeDistanceCount)

	randomSeed = 42
	for i := range shapeDistanceCount {
		s.transformAs[i] = dbox2d.Transform{
			P: randomVec2(fixed.Q32MustParse("-0.1"), fixed.Q32MustParse("0.1")),
			Q: randomRot(),
		}
		s.transformBs[i] = dbox2d.Transform{
			P: randomVec2(fixed.Q32MustParse("0.25"), fixed.Q32FromInt(2)),
			Q: randomRot(),
		}
	}

	s.drawIndex = 0
	s.minMilliseconds = math.MaxFloat64
	return s
}

// UpdateGui exposes the distance benchmark draw-index control.
func (s *ShapeDistance) UpdateGui() {
	height := 80
	gui := s.Context.Gui
	gui.Begin("Benchmark: Shape Distance", 10, s.Context.Camera.Height-height-50, 220, height)
	gui.SliderInt("draw index", &s.drawIndex, 0, shapeDistanceCount-1)
	gui.End()
}

// Step runs the distance benchmark and draws the selected query.
func (s *ShapeDistance) Step() {
	if !s.Context.Settings.Pause || s.Context.Settings.SingleStep {
		input := dbox2d.DistanceInput{
			ProxyA:   dbox2d.MakeProxy(s.polygonA.Vertices[:s.polygonA.Count], s.polygonA.Radius),
			ProxyB:   dbox2d.MakeProxy(s.polygonB.Vertices[:s.polygonB.Count], s.polygonB.Radius),
			UseRadii: true,
		}
		totalIterations := 0
		start := time.Now()
		for i := range shapeDistanceCount {
			cache := dbox2d.SimplexCache{}
			input.TransformA = s.transformAs[i]
			input.TransformB = s.transformBs[i]
			s.outputs[i] = dbox2d.ShapeDistance(&input, &cache, nil)
			totalIterations += s.outputs[i].Iterations
		}
		ms := time.Since(start).Seconds() * 1000
		s.minMilliseconds = min(s.minMilliseconds, ms)

		s.DrawTextLine("count = %d", shapeDistanceCount)
		// Cycle counters are omitted; Go measures elapsed time directly.
		s.DrawTextLine("min ms = %g, ave us = %g", s.minMilliseconds, 1000*s.minMilliseconds/float64(shapeDistanceCount))
		s.DrawTextLine("average iterations = %g", float64(totalIterations)/float64(shapeDistanceCount))
	}

	xfA := s.transformAs[s.drawIndex]
	xfB := s.transformBs[s.drawIndex]
	output := s.outputs[s.drawIndex]
	s.Context.Draw.DrawSolidPolygon(xfA, s.polygonA.Vertices[:s.polygonA.Count], s.polygonA.Radius, dbox2d.ColorBox2DGreen)
	s.Context.Draw.DrawSolidPolygon(xfB, s.polygonB.Vertices[:s.polygonB.Count], s.polygonB.Radius, dbox2d.ColorBox2DBlue)
	s.Context.Draw.DrawSegment(output.PointA, output.PointB, dbox2d.ColorDimGray)
	s.Context.Draw.DrawPoint(output.PointA, fixed.Q32FromInt(10), dbox2d.ColorWhite)
	s.Context.Draw.DrawPoint(output.PointB, fixed.Q32FromInt(10), dbox2d.ColorWhite)
	s.Context.Draw.DrawSegment(output.PointA, dbox2d.MulAdd(output.PointA, fixed.Q32Half(), output.Normal), dbox2d.ColorYellow)
	s.DrawTextLine("distance = %s", output.Distance)

	s.Base.Step()
}

const (
	sensorColumnCount = 40
	sensorRowCount    = 40
)

type sensorUserData struct {
	shouldDestroyVisitors bool
}

// Sensor benchmarks sensor begin/end event processing and visitor creation.
type Sensor struct {
	Base

	groundId      dbox2d.BodyId
	passiveSensor *sensorUserData
	activeSensor  *sensorUserData
	maxBeginCount int
	maxEndCount   int
	lastStepCount int
}

// NewSensor builds the sensor benchmark scene.
func NewSensor(ctx *SampleContext) Sample {
	s := &Sensor{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 105}
		ctx.Camera.Zoom = 125
	}

	s.passiveSensor = &sensorUserData{shouldDestroyVisitors: false}
	s.activeSensor = &sensorUserData{shouldDestroyVisitors: true}

	bodyDef := dbox2d.DefaultBodyDef()
	s.groundId = dbox2d.CreateBody(s.WorldId, &bodyDef)

	{
		gridSize := fixed.Q32FromInt(3)
		shapeDef := dbox2d.DefaultShapeDef()
		shapeDef.IsSensor = true
		shapeDef.EnableSensorEvents = true
		shapeDef.UserData = s.activeSensor

		x := fixed.Q32FromInt(-40).Mul(gridSize)
		for range 81 {
			box := dbox2d.MakeOffsetBox(
				fixed.Q32Half().Mul(gridSize),
				fixed.Q32Half().Mul(gridSize),
				dbox2d.Vec2{X: x},
				dbox2d.RotIdentity(),
			)
			dbox2d.CreatePolygonShape(s.groundId, &shapeDef, &box)
			x = x.Add(gridSize)
		}
	}

	randomSeed = 42
	shift := fixed.Q32FromInt(5)
	xCenter := fixed.Q32Half().Mul(shift).Mul(fixed.Q32FromInt(sensorColumnCount))
	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.IsSensor = true
	shapeDef.EnableSensorEvents = true
	shapeDef.UserData = s.passiveSensor
	yStart := fixed.Q32FromInt(10)

	for j := range sensorRowCount {
		y := fixed.Q32FromInt(j).Mul(shift).Add(yStart)
		for i := range sensorColumnCount {
			x := fixed.Q32FromInt(i).Mul(shift).Sub(xCenter)
			yOffset := randomFloatRange(fixed.Q32FromInt(-1), fixed.Q32One())
			box := dbox2d.MakeOffsetRoundedBox(
				fixed.Q32Half(),
				fixed.Q32Half(),
				dbox2d.Vec2{X: x, Y: y.Add(yOffset)},
				randomRot(),
				fixed.Q32MustParse("0.1"),
			)
			dbox2d.CreatePolygonShape(s.groundId, &shapeDef, &box)
		}
	}

	s.maxBeginCount = 0
	s.maxEndCount = 0
	s.lastStepCount = 0
	return s
}

func (s *Sensor) createRow(y dbox2d.Q) {
	shift := fixed.Q32FromInt(5)
	xCenter := fixed.Q32Half().Mul(shift).Mul(fixed.Q32FromInt(sensorColumnCount))

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	bodyDef.GravityScale = fixed.Q32Zero()
	bodyDef.LinearVelocity = dbox2d.Vec2{Y: fixed.Q32FromInt(-5)}

	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.EnableSensorEvents = true
	circle := dbox2d.Circle{Radius: fixed.Q32Half()}
	for i := range sensorColumnCount {
		bodyDef.Position = dbox2d.Vec2{
			X: fixed.Q32FromInt(i).Mul(shift).Sub(xCenter),
			Y: y,
		}
		bodyID := dbox2d.CreateBody(s.WorldId, &bodyDef)
		dbox2d.CreateCircleShape(bodyID, &shapeDef, &circle)
	}
}

// Step processes sensor events after advancing the sample world.
func (s *Sensor) Step() {
	s.Base.Step()

	if s.StepCount == s.lastStepCount {
		return
	}

	zombies := make(map[dbox2d.BodyId]struct{})
	events := s.WorldId.GetSensorEvents()
	for _, event := range events.BeginEvents {
		userData := event.SensorShapeId.GetUserData().(*sensorUserData)
		if userData.shouldDestroyVisitors {
			zombies[event.VisitorShapeId.GetBody()] = struct{}{}
		} else {
			material := event.VisitorShapeId.GetSurfaceMaterial()
			material.CustomColor = uint32(dbox2d.ColorLime)
			event.VisitorShapeId.SetSurfaceMaterial(material)
		}
	}

	for _, event := range events.EndEvents {
		if !event.VisitorShapeId.IsValid() {
			continue
		}
		material := event.VisitorShapeId.GetSurfaceMaterial()
		material.CustomColor = 0
		event.VisitorShapeId.SetSurfaceMaterial(material)
	}

	zombieIDs := make([]dbox2d.BodyId, 0, len(zombies))
	for bodyID := range zombies {
		zombieIDs = append(zombieIDs, bodyID)
	}
	sort.Slice(zombieIDs, func(i, j int) bool {
		return dbox2d.StoreBodyId(zombieIDs[i]) < dbox2d.StoreBodyId(zombieIDs[j])
	})
	for _, bodyID := range zombieIDs {
		dbox2d.DestroyBody(bodyID)
	}

	delay := 0x1F
	if s.StepCount&delay == 0 {
		s.createRow(fixed.Q32FromInt(10 + sensorRowCount*5))
	}

	s.lastStepCount = s.StepCount
	s.maxBeginCount = max(s.maxBeginCount, len(events.BeginEvents))
	s.maxEndCount = max(s.maxEndCount, len(events.EndEvents))
	s.DrawTextLine("max begin touch events = %d", s.maxBeginCount)
	s.DrawTextLine("max end touch events = %d", s.maxEndCount)
}
