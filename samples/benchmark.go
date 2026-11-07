// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/sample_benchmark.cpp of Box2D v3.2.0, plus the
// scenes of shared/benchmarks.c (internal/shared and benchmark_scenes.go).
// Debug-only sizes use the release values. The b2Capacity hints of
// RegisterSampleWithCapacity are not ported: the Go world has no
// preallocation to size.

package samples

import (
	"math"
	"sort"
	"time"

	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/dbox2d/internal/shared"
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
	RegisterSample("Benchmark", "Large Compounds", NewLargeCompounds)
	RegisterSample("Benchmark", "Compounds", NewCompounds)
	RegisterSample("Benchmark", "Barrel 2.4", NewBarrel24)
	RegisterSample("Benchmark", "Washer", NewWasher)
	RegisterSample("Benchmark", "Junkyard", NewJunkyard)
	RegisterSample("Benchmark", "Capacity", NewCapacity)
	RegisterSample("Benchmark", "Queries", NewQueries)
	RegisterSample("Benchmark", "Kinematic", NewKinematic)
	RegisterSample("Benchmark", "Cast", NewCast)
	RegisterSample("Benchmark", "Spinner", NewSpinner)
	RegisterSample("Benchmark", "Rain", NewRain)
	RegisterSample("Benchmark", "Shape Distance", NewShapeDistance)
	RegisterSample("Benchmark", "Sensor", NewSensor)
}

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
		ctx.Camera.Zoom = 15
	}

	shared.BuildTumbler(s.WorldId, shared.Options{})
	return s
}

type barrelShapeType int

const (
	barrelCircleShape barrelShapeType = iota
	barrelCapsuleShape
	barrelMixShape
	barrelCompoundShape
	barrelHumanShape
	barrelGopherShape
	barrelMaxColumns = 26
	barrelMaxRows    = 150
)

// Barrel is the mixed-shape barrel benchmark scene.
type Barrel struct {
	Base

	bodies      [barrelMaxRows * barrelMaxColumns]b2.BodyId
	humans      [barrelMaxRows * barrelMaxColumns]shared.Human
	gophers     [barrelMaxRows * barrelMaxColumns]gopher
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
		gridSize := b2.QOne()

		bodyDef := b2.DefaultBodyDef()
		groundID := b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()

		y := b2.QZero()
		x := b2.F(-40).Mul(gridSize)
		for range 81 {
			box := b2.MakeOffsetBox(
				b2.F(0.55).Mul(gridSize),
				b2.QHalf().Mul(gridSize),
				b2.Vec2{X: x, Y: y},
				b2.RotIdentity(),
			)
			b2.CreatePolygonShape(groundID, &shapeDef, &box)
			x = x.Add(gridSize)
		}

		y = gridSize
		x = b2.F(-40).Mul(gridSize)
		for range 100 {
			box := b2.MakeOffsetBox(
				b2.QHalf().Mul(gridSize),
				b2.F(0.55).Mul(gridSize),
				b2.Vec2{X: x, Y: y},
				b2.RotIdentity(),
			)
			b2.CreatePolygonShape(groundID, &shapeDef, &box)
			y = y.Add(gridSize)
		}

		y = gridSize
		x = b2.F(40).Mul(gridSize)
		for range 100 {
			box := b2.MakeOffsetBox(
				b2.QHalf().Mul(gridSize),
				b2.F(0.55).Mul(gridSize),
				b2.Vec2{X: x, Y: y},
				b2.RotIdentity(),
			)
			b2.CreatePolygonShape(groundID, &shapeDef, &box)
			y = y.Add(gridSize)
		}

		segment := b2.Segment{
			Point1: b2.V2(-800, -80),
			Point2: b2.V2(800, -80),
		}
		b2.CreateSegmentShape(groundID, &shapeDef, &segment)
	}

	s.shapeType = barrelGopherShape
	s.createScene()

	return s
}

func (s *Barrel) createScene() {
	shared.RandomSeed = 42

	for i := range s.bodies {
		if !s.bodies[i].IsNull() {
			b2.DestroyBody(s.bodies[i])
			s.bodies[i] = b2.BodyId{}
		}

		if s.humans[i].IsSpawned {
			s.humans[i].Destroy()
		}

		if s.gophers[i].isSpawned {
			s.gophers[i].destroy()
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
	case barrelGopherShape:
		s.rowCount = 20
		s.columnCount = 14
	}

	rad := b2.QHalf()
	shift := b2.F(1.15)
	centerX := shift.Mul(b2.F(s.columnCount)).Div(b2.F(2))
	centerY := shift.Div(b2.F(2))

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	if s.shapeType == barrelMixShape {
		bodyDef.AngularDamping = b2.F(0.3)
	}

	shapeDef := b2.DefaultShapeDef()
	shapeDef.Density = b2.QOne()
	shapeDef.Material.Friction = b2.F(0.5)

	capsule := b2.Capsule{
		Center1: b2.V2(0.0, -0.25),
		Center2: b2.V2(0.0, 0.25),
		Radius:  rad,
	}
	circle := b2.Circle{Radius: rad}

	wedgePoints := []b2.Vec2{
		{X: b2.F(-0.1), Y: b2.F(-0.5)},
		{X: b2.F(0.1), Y: b2.F(-0.5)},
		{Y: b2.F(0.5)},
	}
	wedgeHull := b2.ComputeHull(wedgePoints)
	wedge := b2.MakePolygon(&wedgeHull, b2.QZero())

	vertices := []b2.Vec2{
		{X: b2.F(-1)},
		{X: b2.QHalf(), Y: b2.QOne()},
		{Y: b2.F(2)},
	}
	hull := b2.ComputeHull(vertices)
	left := b2.MakePolygon(&hull, b2.QZero())

	vertices[0] = b2.Vec2{X: b2.QOne()}
	vertices[1] = b2.Vec2{X: b2.QHalf().Neg(), Y: b2.QOne()}
	hull = b2.ComputeHull(vertices)
	right := b2.MakePolygon(&hull, b2.QZero())

	side := b2.F(-0.1)
	extraY := b2.QHalf()
	switch s.shapeType {
	case barrelCompoundShape:
		extraY = b2.F(0.25)
		side = b2.F(0.25)
		shift = b2.F(2)
		centerX = shift.Mul(b2.F(s.columnCount)).Div(b2.F(2)).Sub(b2.QOne())
	case barrelHumanShape:
		extraY = b2.QHalf()
		side = b2.F(0.55)
		shift = b2.F(2.5)
		centerX = shift.Mul(b2.F(s.columnCount)).Div(b2.F(2))
	case barrelGopherShape:
		extraY = b2.F(2.5)
		side = b2.QOne()
		shift = b2.F(4.5)
		centerX = shift.Mul(b2.F(s.columnCount)).Div(b2.F(2))
	}

	index := 0
	yStart := b2.F(100)
	switch s.shapeType {
	case barrelHumanShape:
		yStart = b2.F(2)
	case barrelGopherShape:
		yStart = b2.F(110)
	}

	for i := range s.columnCount {
		x := b2.F(i).Mul(shift).Sub(centerX)

		for j := range s.rowCount {
			y := b2.F(j).Mul(shift.Add(extraY)).Add(centerY).Add(yStart)
			bodyDef.Position = b2.Vec2{X: x.Add(side), Y: y}
			side = side.Neg()

			switch s.shapeType {
			case barrelCircleShape:
				s.bodies[index] = b2.CreateBody(s.WorldId, &bodyDef)
				circle.Radius = shared.RandomFloatRange(b2.F(0.25), b2.F(0.75))
				shapeDef.Material.RollingResistance = b2.F(0.2)
				b2.CreateCircleShape(s.bodies[index], &shapeDef, &circle)
			case barrelCapsuleShape:
				s.bodies[index] = b2.CreateBody(s.WorldId, &bodyDef)
				capsule.Radius = shared.RandomFloatRange(b2.F(0.25), b2.F(0.5))
				length := shared.RandomFloatRange(b2.F(0.25), b2.QOne())
				capsule.Center1 = b2.Vec2{Y: b2.QHalf().Mul(length).Neg()}
				capsule.Center2 = b2.Vec2{Y: b2.QHalf().Mul(length)}
				shapeDef.Material.RollingResistance = b2.F(0.2)
				b2.CreateCapsuleShape(s.bodies[index], &shapeDef, &capsule)
			case barrelMixShape:
				s.bodies[index] = b2.CreateBody(s.WorldId, &bodyDef)
				switch index % 3 {
				case 0:
					circle.Radius = shared.RandomFloatRange(b2.F(0.25), b2.F(0.75))
					b2.CreateCircleShape(s.bodies[index], &shapeDef, &circle)
				case 1:
					capsule.Radius = shared.RandomFloatRange(b2.F(0.25), b2.F(0.5))
					length := shared.RandomFloatRange(b2.F(0.25), b2.QOne())
					capsule.Center1 = b2.Vec2{Y: b2.QHalf().Mul(length).Neg()}
					capsule.Center2 = b2.Vec2{Y: b2.QHalf().Mul(length)}
					b2.CreateCapsuleShape(s.bodies[index], &shapeDef, &capsule)
				case 2:
					width := shared.RandomFloatRange(b2.F(0.1), b2.F(0.5))
					height := shared.RandomFloatRange(b2.F(0.5), b2.F(0.75))
					box := b2.MakeBox(width, height)
					value := shared.RandomFloatRange(b2.F(-1), b2.QOne())
					box.Radius = b2.QFromRatio(1, 4).Mul(value.Max(b2.QZero()))
					b2.CreatePolygonShape(s.bodies[index], &shapeDef, &box)
				default:
					wedge.Radius = shared.RandomFloatRange(b2.F(0.1), b2.F(0.25))
					b2.CreatePolygonShape(s.bodies[index], &shapeDef, &wedge)
				}
			case barrelCompoundShape:
				s.bodies[index] = b2.CreateBody(s.WorldId, &bodyDef)
				b2.CreatePolygonShape(s.bodies[index], &shapeDef, &left)
				b2.CreatePolygonShape(s.bodies[index], &shapeDef, &right)
			case barrelHumanShape:
				scale := b2.F(3.5)
				jointFriction := b2.F(0.05)
				jointHertz := b2.F(5)
				jointDamping := b2.QHalf()
				s.humans[index] = shared.CreateHuman(
					s.WorldId, bodyDef.Position, scale, jointFriction, jointHertz, jointDamping,
					index+1, nil, false,
				)
			case barrelGopherShape:
				s.gophers[index] = createGopher(s.WorldId, bodyDef.Position, 3, index+1)
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
	shapeTypes := []string{"Circle", "Capsule", "Mix", "Compound", "Human", "Gopher"}
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

	groundID     b2.BodyId
	rowCount     int
	columnCount  int
	tumblerIDs   []b2.BodyId
	positions    []b2.Vec2
	tumblerCount int
	bodyIDs      []b2.BodyId
	bodyCount    int
	bodyIndex    int
	angularSpeed float64
}

// NewManyTumblers builds the many-tumblers benchmark scene.
// angularVelocity converts the degrees-per-second slider to turns per second.
func (s *ManyTumblers) angularVelocity() b2.Q {
	return FromFloat64(s.angularSpeed).Div(b2.F(360))
}

func NewManyTumblers(ctx *SampleContext) Sample {
	s := &ManyTumblers{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 1, Y: -5.5}
		ctx.Camera.Zoom = 25 * 3.4
		ctx.Settings.DrawJoints = false
	}

	bodyDef := b2.DefaultBodyDef()
	s.groundID = b2.CreateBody(s.WorldId, &bodyDef)

	s.rowCount = 19
	s.columnCount = 19
	s.angularSpeed = 25
	s.createScene()

	return s
}

func (s *ManyTumblers) createTumbler(position b2.Vec2, index int) {
	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.KinematicBody
	bodyDef.Position = position
	bodyDef.AngularVelocity = s.angularVelocity()
	bodyID := b2.CreateBody(s.WorldId, &bodyDef)
	s.tumblerIDs[index] = bodyID

	shapeDef := b2.DefaultShapeDef()
	shapeDef.Density = b2.F(50)

	walls := []struct {
		halfWidth, halfHeight b2.Q
		center                b2.Vec2
	}{
		{b2.F(0.25), b2.F(2), b2.V2(2, 0)},
		{b2.F(0.25), b2.F(2), b2.V2(-2, 0)},
		{b2.F(2), b2.F(0.25), b2.V2(0, 2)},
		{b2.F(2), b2.F(0.25), b2.V2(0, -2)},
	}
	for _, wall := range walls {
		polygon := b2.MakeOffsetBox(wall.halfWidth, wall.halfHeight, wall.center, b2.RotIdentity())
		b2.CreatePolygonShape(bodyID, &shapeDef, &polygon)
	}
}

func (s *ManyTumblers) createScene() {
	for i := range s.bodyCount {
		if !s.bodyIDs[i].IsNull() {
			b2.DestroyBody(s.bodyIDs[i])
		}
	}

	for i := range s.tumblerCount {
		b2.DestroyBody(s.tumblerIDs[i])
	}

	s.tumblerCount = s.rowCount * s.columnCount
	s.tumblerIDs = make([]b2.BodyId, s.tumblerCount)
	s.positions = make([]b2.Vec2, s.tumblerCount)

	index := 0
	x := b2.F(-4 * s.rowCount)
	for range s.rowCount {
		y := b2.F(-4 * s.columnCount)
		for range s.columnCount {
			s.positions[index] = b2.Vec2{X: x, Y: y}
			s.createTumbler(s.positions[index], index)
			index++
			y = y.Add(b2.F(8))
		}
		x = x.Add(b2.F(8))
	}

	bodiesPerTumbler := 50
	s.bodyCount = bodiesPerTumbler * s.tumblerCount
	s.bodyIDs = make([]b2.BodyId, s.bodyCount)
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
		shapeDef := b2.DefaultShapeDef()
		capsule := b2.Capsule{
			Center1: b2.V2(-0.1, 0.0),
			Center2: b2.V2(0.1, 0.0),
			Radius:  b2.F(0.075),
		}

		for i := range s.tumblerCount {
			bodyDef := b2.DefaultBodyDef()
			bodyDef.Type = b2.DynamicBody
			bodyDef.Position = s.positions[i]
			s.bodyIDs[s.bodyIndex] = b2.CreateBody(s.WorldId, &bodyDef)
			b2.CreateCapsuleShape(s.bodyIDs[s.bodyIndex], &shapeDef, &capsule)
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

	shared.BuildLargePyramid(s.WorldId, shared.Options{})
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
		ctx.Camera.Center = Vec2f{X: 23, Y: 72.5}
		ctx.Camera.Zoom = 165
		ctx.Settings.EnableSleep = false
	}

	shared.BuildManyPyramids(s.WorldId, shared.Options{})
	return s
}

const maxBaseCount = 100

// CreateDestroy measures rebuilding a pyramid of dynamic bodies.
type CreateDestroy struct {
	Base

	bodies      []b2.BodyId
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

	bodyDef := b2.DefaultBodyDef()
	groundID := b2.CreateBody(s.WorldId, &bodyDef)
	ground := b2.MakeBox(b2.F(100), b2.QOne())
	shapeDef := b2.DefaultShapeDef()
	b2.CreatePolygonShape(groundID, &shapeDef, &ground)

	maxBodyCount := maxBaseCount * (maxBaseCount + 1) / 2
	s.bodies = make([]b2.BodyId, maxBodyCount)
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
			b2.DestroyBody(s.bodies[i])
			s.bodies[i] = b2.BodyId{}
		}
	}
	s.destroyTime += time.Since(timer).Seconds() * 1000

	count := s.baseCount
	rad := b2.QHalf()
	shift := rad.Mul(b2.F(2))
	centerX := shift.Mul(b2.F(count)).Div(b2.F(2))
	centerY := shift.Div(b2.F(2)).Add(b2.QOne())

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody

	shapeDef := b2.DefaultShapeDef()
	shapeDef.Density = b2.QOne()
	shapeDef.Material.Friction = b2.F(0.5)

	box := b2.MakeRoundedBox(b2.QHalf(), b2.QHalf(), b2.QZero())
	index := 0

	timer = time.Now()
	for i := range count {
		y := b2.F(i).Mul(shift).Add(centerY)

		for j := i; j < count; j++ {
			x := b2.QFromRatio(i, 2).Mul(shift).
				Add(b2.F(j - i).Mul(shift)).
				Sub(centerX)
			bodyDef.Position = b2.Vec2{X: x, Y: y}

			s.bodies[index] = b2.CreateBody(s.WorldId, &bodyDef)
			b2.CreatePolygonShape(s.bodies[index], &shapeDef, &box)
			index++
		}
	}
	s.createTime += time.Since(timer).Seconds() * 1000

	s.bodyCount = index
	s.WorldId.Step(b2.QFromRatio(1, 60), 4)
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

// Sleep stresses waking and sleeping: ten pyramids whose island is woken
// again each time it falls asleep. It mirrors BenchmarkSleep; the scene is
// CreateSleep and StepSleep of shared/benchmarks.c.
type Sleep struct {
	Base

	data benchSleep
}

// NewSleep builds the sleep benchmark scene.
func NewSleep(ctx *SampleContext) Sample {
	s := &Sleep{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 20}
		ctx.Camera.Zoom = 25 * 6
	}

	s.data.create(s.WorldId)
	return s
}

// Step wakes a sleeping pyramid and advances the world.
func (s *Sleep) Step() {
	if !s.Context.Settings.Pause || s.Context.Settings.SingleStep {
		s.data.step(s.WorldId)
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

	shared.BuildJointGrid(s.WorldId, shared.Options{})
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

	shared.BuildSmash(s.WorldId, shared.Options{})
	return s
}

// LargeCompounds builds the compound-shape stress benchmark scene. It
// mirrors BenchmarkLargeCompounds, the former Compound sample.
type LargeCompounds struct {
	Base
}

// NewLargeCompounds builds the large compounds benchmark scene.
func NewLargeCompounds(ctx *SampleContext) Sample {
	s := &LargeCompounds{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 18, Y: 115}
		ctx.Camera.Zoom = 25 * 5.5
	}

	grid := b2.QOne()
	height := 200
	width := 200

	{
		bodyDef := b2.DefaultBodyDef()
		groundID := b2.CreateBody(s.WorldId, &bodyDef)
		shapeDef := b2.DefaultShapeDef()

		for i := range height {
			y := b2.F(i).Mul(grid)
			for j := i; j < width; j++ {
				x := b2.F(j).Mul(grid)
				square := b2.MakeOffsetBox(
					b2.QHalf().Mul(grid),
					b2.QHalf().Mul(grid),
					b2.Vec2{X: x, Y: y},
					b2.RotIdentity(),
				)
				b2.CreatePolygonShape(groundID, &shapeDef, &square)
			}
		}

		for i := range height {
			y := b2.F(i).Mul(grid)
			for j := i; j < width; j++ {
				x := b2.F(-j).Mul(grid)
				square := b2.MakeOffsetBox(
					b2.QHalf().Mul(grid),
					b2.QHalf().Mul(grid),
					b2.Vec2{X: x, Y: y},
					b2.RotIdentity(),
				)
				b2.CreatePolygonShape(groundID, &shapeDef, &square)
			}
		}
	}

	{
		span := 20
		count := 5

		bodyDef := b2.DefaultBodyDef()
		bodyDef.Type = b2.DynamicBody
		// Defer mass properties to avoid n-squared mass computations.
		shapeDef := b2.DefaultShapeDef()
		shapeDef.UpdateBodyMass = false

		for m := range count {
			yBody := b2.F(100 + m*span).Mul(grid)

			for n := range count {
				xBody := b2.QHalf().Neg().Mul(grid).
					Mul(b2.F(count)).
					Mul(b2.F(span)).
					Add(b2.F(n * span).Mul(grid))
				bodyDef.Position = b2.Vec2{X: xBody, Y: yBody}
				bodyID := b2.CreateBody(s.WorldId, &bodyDef)

				for i := range span {
					y := b2.F(i).Mul(grid)
					for j := range span {
						x := b2.F(j).Mul(grid)
						square := b2.MakeOffsetBox(
							b2.QHalf().Mul(grid),
							b2.QHalf().Mul(grid),
							b2.Vec2{X: x, Y: y},
							b2.RotIdentity(),
						)
						b2.CreatePolygonShape(bodyID, &shapeDef, &square)
					}
				}

				bodyID.UpdateMassFromShapes()
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

	grid := b2.QOne()
	span := 100

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.KinematicBody
	// The reference uses 1 rad/s; body angular velocity is stored in turns/s.
	bodyDef.AngularVelocity = b2.F(0.1591549431)

	shapeDef := b2.DefaultShapeDef()
	shapeDef.Filter.CategoryBits = 1
	shapeDef.Filter.MaskBits = 2
	// Defer mass properties to avoid n-squared mass computations.
	shapeDef.UpdateBodyMass = false

	bodyID := b2.CreateBody(s.WorldId, &bodyDef)

	for i := -span; i < span; i++ {
		y := b2.F(i).Mul(grid)
		for j := -span; j < span; j++ {
			x := b2.F(j).Mul(grid)
			square := b2.MakeOffsetBox(
				b2.QHalf().Mul(grid),
				b2.QHalf().Mul(grid),
				b2.Vec2{X: x, Y: y},
				b2.RotIdentity(),
			)
			b2.CreatePolygonShape(bodyID, &shapeDef, &square)
		}
	}

	bodyID.UpdateMassFromShapes()
	return s
}

type castQueryType int

const (
	castRay castQueryType = iota
	castCircle
	castOverlap
)

type castResult struct {
	point    b2.Vec2
	fraction b2.Q
	hit      bool
}

type overlapResult struct {
	points [32]b2.Vec2
	count  int
}

// Cast benchmarks ray, circle, and AABB queries against a large static scene.
type Cast struct {
	Base

	queryType    castQueryType
	origins      []b2.Vec2
	translations []b2.Vec2
	minTime      float64
	buildTime    float64
	rowCount     int
	columnCount  int
	drawIndex    int
	radius       b2.Q
	fill         b2.Q
	ratio        b2.Q
	grid         b2.Q
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
	s.ratio = b2.F(5)
	s.grid = b2.QOne()
	s.fill = b2.F(0.1)
	s.rowCount = 1000    // release value; BENCHMARK_DEBUG value was 100
	s.columnCount = 1000 // release value; BENCHMARK_DEBUG value was 100
	s.minTime = 1e6
	s.drawIndex = 0
	s.topDown = false
	s.buildTime = 0
	s.radius = b2.F(0.1)

	shared.RandomSeed = 1234
	sampleCount := 10000 // release value; BENCHMARK_DEBUG value was 100
	s.origins = make([]b2.Vec2, sampleCount)
	s.translations = make([]b2.Vec2, sampleCount)
	extent := b2.F(s.rowCount).Mul(s.grid)

	// Precompute rays so each step measures queries instead of randomization.
	for i := range sampleCount {
		rayStart := shared.RandomVec2(b2.QZero(), extent)
		rayEnd := shared.RandomVec2(b2.QZero(), extent)
		s.origins[i] = rayStart
		s.translations[i] = rayEnd.Sub(rayStart)
	}

	s.buildScene()
	return s
}

func (s *Cast) buildScene() {
	shared.RandomSeed = 1234
	started := time.Now()
	s.CreateWorld()

	bodyDef := b2.DefaultBodyDef()
	shapeDef := b2.DefaultShapeDef()

	for i := range s.rowCount {
		y := b2.F(i).Mul(s.grid)
		for j := range s.columnCount {
			x := b2.F(j).Mul(s.grid)
			fillTest := shared.RandomFloatRange(b2.QZero(), b2.QOne())
			if !s.fill.Less(fillTest) {
				bodyDef.Position = b2.Vec2{X: x, Y: y}
				bodyID := b2.CreateBody(s.WorldId, &bodyDef)

				ratio := shared.RandomFloatRange(b2.QOne(), s.ratio)
				halfWidth := shared.RandomFloatRange(b2.F(0.05), b2.F(0.25))
				var box b2.Polygon
				if shared.RandomFloat().Greater(b2.QZero()) {
					box = b2.MakeBox(ratio.Mul(halfWidth), halfWidth)
				} else {
					box = b2.MakeBox(halfWidth, ratio.Mul(halfWidth))
				}

				category := shared.RandomIntRange(0, 2)
				shapeDef.Filter.CategoryBits = uint64(1 << category)
				switch category {
				case 0:
					shapeDef.Material.CustomColor = uint32(b2.ColorBox2DBlue)
				case 1:
					shapeDef.Material.CustomColor = uint32(b2.ColorBox2DYellow)
				default:
					shapeDef.Material.CustomColor = uint32(b2.ColorBox2DGreen)
				}

				b2.CreatePolygonShape(bodyID, &shapeDef, &box)
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
			s.radius = b2.F(5)
		} else {
			s.radius = b2.F(0.1)
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

	filter := b2.DefaultQueryFilter()
	filter.MaskBits = 1
	hitCount := 0
	nodeVisits := 0
	leafVisits := 0
	ms := 0.0
	sampleCount := len(s.origins)

	switch s.queryType {
	case castRay:
		started := time.Now()
		var drawResult b2.RayResult

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
		s.Context.Draw.DrawLineFcn(p1, p2, b2.ColorWhite)
		s.Context.Draw.DrawPointFcn(p1, b2.F(5), b2.ColorGreen)
		s.Context.Draw.DrawPointFcn(p2, b2.F(5), b2.ColorRed)
		if drawResult.Hit {
			s.Context.Draw.DrawPointFcn(drawResult.Point, b2.F(5), b2.ColorWhite)
		}

	case castCircle:
		started := time.Now()
		var drawResult, result castResult

		for i := range sampleCount {
			proxy := b2.MakeProxy([]b2.Vec2{{}}, s.radius)
			result.hit = false
			traversalResult := s.WorldId.CastShape(s.origins[i], &proxy, s.translations[i], filter, func(_ b2.ShapeId, point, _ b2.Vec2, fraction b2.Q) b2.Q {
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
		s.Context.Draw.DrawLineFcn(p1, p2, b2.ColorWhite)
		s.Context.Draw.DrawPointFcn(p1, b2.F(5), b2.ColorGreen)
		s.Context.Draw.DrawPointFcn(p2, b2.F(5), b2.ColorRed)
		if drawResult.hit {
			t := p1.Add(s.translations[s.drawIndex].Mul(drawResult.fraction))
			s.Context.Draw.DrawCircleFcn(t, s.radius, b2.ColorWhite)
			s.Context.Draw.DrawPointFcn(drawResult.point, b2.F(5), b2.ColorWhite)
		}

	case castOverlap:
		started := time.Now()
		var drawResult, result overlapResult
		extent := b2.Vec2{X: s.radius, Y: s.radius}

		for i := range sampleCount {
			aabb := b2.AABB{LowerBound: b2.Vec2{X: s.radius.Neg(), Y: s.radius.Neg()}, UpperBound: extent}
			result.count = 0
			traversalResult := s.WorldId.OverlapAABB(s.origins[i], aabb, filter, func(shapeID b2.ShapeId) bool {
				if result.count < len(result.points) {
					result.points[result.count] = b2.AABBCenter(shapeID.GetBounds())
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
		aabb := b2.AABB{LowerBound: origin.Sub(extent), UpperBound: origin.Add(extent)}
		s.Context.Draw.DrawBoundsFcn(aabb, b2.ColorWhite)
		for i := range drawResult.count {
			s.Context.Draw.DrawPointFcn(drawResult.points[i], b2.F(5), b2.ColorHotPink)
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

	shared.BuildSpinner(s.WorldId, shared.Options{})
	return s
}

// Rain is the release-sized falling-humans benchmark scene.
type Rain struct {
	Base

	step shared.StepFn
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

	s.step = shared.BuildRain(s.WorldId, shared.Options{})
	return s
}

func (s *Rain) Step() {
	if !s.Context.Settings.Pause || s.Context.Settings.SingleStep {
		s.step(s.StepCount)
	}

	// The reference's m_stepCount % 1000 == 0 branch only added zero.
	s.Base.Step()
}

const shapeDistanceCount = 10000

// ShapeDistance benchmarks repeated distance queries between two polygons.
type ShapeDistance struct {
	Base

	polygonA        b2.Polygon
	polygonB        b2.Polygon
	transformAs     []b2.Transform
	transformBs     []b2.Transform
	outputs         []b2.DistanceOutput
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
		points := make([]b2.Vec2, 8)
		q := b2.MakeRot(b2.QFromRatio(1, 8))
		points[0] = b2.Vec2{X: b2.QHalf()}
		for i := 1; i < len(points); i++ {
			points[i] = b2.RotateVector(q, points[i-1])
		}
		hull := b2.ComputeHull(points)
		s.polygonA = b2.MakePolygon(&hull, b2.QZero())
	}

	{
		points := make([]b2.Vec2, 8)
		q := b2.MakeRot(b2.QFromRatio(1, 8))
		points[0] = b2.Vec2{X: b2.QHalf()}
		for i := 1; i < len(points); i++ {
			points[i] = b2.RotateVector(q, points[i-1])
		}
		hull := b2.ComputeHull(points)
		s.polygonB = b2.MakePolygon(&hull, b2.F(0.1))
	}

	s.transformAs = make([]b2.Transform, shapeDistanceCount)
	s.transformBs = make([]b2.Transform, shapeDistanceCount)
	s.outputs = make([]b2.DistanceOutput, shapeDistanceCount)

	shared.RandomSeed = 42
	for i := range shapeDistanceCount {
		s.transformAs[i] = b2.Transform{
			P: shared.RandomVec2(b2.F(-0.1), b2.F(0.1)),
			Q: shared.RandomRot(),
		}
		s.transformBs[i] = b2.Transform{
			P: shared.RandomVec2(b2.F(0.25), b2.F(2)),
			Q: shared.RandomRot(),
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
		input := b2.DistanceInput{
			ProxyA:   b2.MakeProxy(s.polygonA.Vertices[:s.polygonA.Count], s.polygonA.Radius),
			ProxyB:   b2.MakeProxy(s.polygonB.Vertices[:s.polygonB.Count], s.polygonB.Radius),
			UseRadii: true,
		}
		totalIterations := 0
		start := time.Now()
		for i := range shapeDistanceCount {
			cache := b2.SimplexCache{}
			input.Transform = b2.InvMulTransforms(s.transformAs[i], s.transformBs[i])
			s.outputs[i] = b2.ComputeShapeDistance(&input, &cache, nil)
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
	s.Context.Draw.DrawSolidPolygonFcn(xfA, s.polygonA.Vertices[:s.polygonA.Count], s.polygonA.Radius, b2.ColorBox2DGreen)
	s.Context.Draw.DrawSolidPolygonFcn(xfB, s.polygonB.Vertices[:s.polygonB.Count], s.polygonB.Radius, b2.ColorBox2DBlue)
	// The query returns frame A results; project to world for drawing.
	worldPointA := b2.TransformPoint(xfA, output.PointA)
	worldPointB := b2.TransformPoint(xfA, output.PointB)
	worldNormal := b2.RotateVector(xfA.Q, output.Normal)
	s.Context.Draw.DrawLineFcn(worldPointA, worldPointB, b2.ColorDimGray)
	s.Context.Draw.DrawPointFcn(worldPointA, b2.F(10), b2.ColorWhite)
	s.Context.Draw.DrawPointFcn(worldPointB, b2.F(10), b2.ColorWhite)
	s.Context.Draw.DrawLineFcn(worldPointA, b2.MulAdd(worldPointA, b2.QHalf(), worldNormal), b2.ColorYellow)
	s.DrawTextLine("distance = %s", output.Distance)

	s.Base.Step()
}

const (
	sensorColumnCount = 40
	sensorRowCount    = 40
)

// sensorUserData is ShapeUserData: the row of a sensor and whether it
// destroys its visitors.
type sensorUserData struct {
	row    int
	active bool
}

// Sensor benchmarks sensor begin/end event processing and visitor creation.
type Sensor struct {
	Base

	groundId       b2.BodyId
	passiveSensors [sensorRowCount]sensorUserData
	activeSensor   *sensorUserData
	filterRow      int
	maxBeginCount  int
	maxEndCount    int
	lastStepCount  int
}

// NewSensor builds the sensor benchmark scene.
func NewSensor(ctx *SampleContext) Sample {
	s := &Sensor{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 105}
		ctx.Camera.Zoom = 125
	}

	s.WorldId.SetCustomFilterCallback(s.filter)

	s.activeSensor = &sensorUserData{row: 0, active: true}

	bodyDef := b2.DefaultBodyDef()
	s.groundId = b2.CreateBody(s.WorldId, &bodyDef)

	{
		gridSize := b2.F(3)
		shapeDef := b2.DefaultShapeDef()
		shapeDef.IsSensor = true
		shapeDef.EnableSensorEvents = true
		shapeDef.UserData = s.activeSensor

		x := b2.F(-40).Mul(gridSize)
		for range 81 {
			box := b2.MakeOffsetBox(
				b2.F(0.45).Mul(gridSize),
				b2.F(0.45).Mul(gridSize),
				b2.Vec2{X: x},
				b2.RotIdentity(),
			)
			b2.CreatePolygonShape(s.groundId, &shapeDef, &box)
			x = x.Add(gridSize)
		}
	}

	shared.RandomSeed = 42
	shift := b2.F(5)
	xCenter := b2.QHalf().Mul(shift).Mul(b2.F(sensorColumnCount))
	shapeDef := b2.DefaultShapeDef()
	shapeDef.IsSensor = true
	shapeDef.EnableSensorEvents = true
	yStart := b2.F(10)
	s.filterRow = sensorRowCount >> 1

	for j := range sensorRowCount {
		s.passiveSensors[j] = sensorUserData{row: j, active: false}
		shapeDef.UserData = &s.passiveSensors[j]

		if j == s.filterRow {
			shapeDef.EnableCustomFiltering = true
			shapeDef.Material.CustomColor = uint32(b2.ColorFuchsia)
		} else {
			shapeDef.EnableCustomFiltering = false
			shapeDef.Material.CustomColor = 0
		}

		y := b2.F(j).Mul(shift).Add(yStart)
		for i := range sensorColumnCount {
			x := b2.F(i).Mul(shift).Sub(xCenter)
			box := b2.MakeOffsetRoundedBox(
				b2.QHalf(),
				b2.QHalf(),
				b2.Vec2{X: x, Y: y},
				b2.RotIdentity(),
				b2.F(0.1),
			)
			b2.CreatePolygonShape(s.groundId, &shapeDef, &box)
		}
	}

	s.maxBeginCount = 0
	s.maxEndCount = 0
	s.lastStepCount = 0
	return s
}

func (s *Sensor) createRow(y b2.Q) {
	shift := b2.F(5)
	xCenter := b2.QHalf().Mul(shift).Mul(b2.F(sensorColumnCount))

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.GravityScale = b2.QZero()
	bodyDef.LinearVelocity = b2.V2(0, -5)

	shapeDef := b2.DefaultShapeDef()
	shapeDef.EnableSensorEvents = true
	circle := b2.Circle{Radius: b2.QHalf()}
	for i := range sensorColumnCount {
		// Stagger bodies to avoid bunching up events into a single update.
		yOffset := shared.RandomFloatRange(b2.F(-1), b2.QOne())
		bodyDef.Position = b2.Vec2{
			X: b2.F(i).Mul(shift).Sub(xCenter),
			Y: y.Add(yOffset),
		}
		bodyID := b2.CreateBody(s.WorldId, &bodyDef)
		b2.CreateCircleShape(bodyID, &shapeDef, &circle)
	}
}

// Step processes sensor events after advancing the sample world.
func (s *Sensor) Step() {
	s.Base.Step()

	if s.StepCount == s.lastStepCount {
		return
	}

	zombies := make(map[b2.BodyId]struct{})
	events := s.WorldId.GetSensorEvents()
	for _, event := range events.BeginEvents {
		userData := event.SensorShapeId.GetUserData().(*sensorUserData)
		if userData.active {
			zombies[event.VisitorShapeId.GetBody()] = struct{}{}
		} else {
			// Check custom filter correctness.
			if userData.row == s.filterRow {
				panic("sensor custom filter let the filtered row through")
			}

			material := event.VisitorShapeId.GetSurfaceMaterial()
			material.CustomColor = uint32(b2.ColorLime)
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

	zombieIDs := make([]b2.BodyId, 0, len(zombies))
	for bodyID := range zombies {
		zombieIDs = append(zombieIDs, bodyID)
	}
	sort.Slice(zombieIDs, func(i, j int) bool {
		return b2.StoreBodyId(zombieIDs[i]) < b2.StoreBodyId(zombieIDs[j])
	})
	for _, bodyID := range zombieIDs {
		b2.DestroyBody(bodyID)
	}

	delay := 0x1F
	if s.StepCount&delay == 0 {
		s.createRow(b2.F(10 + sensorRowCount*5))
	}

	s.lastStepCount = s.StepCount
	s.maxBeginCount = max(s.maxBeginCount, len(events.BeginEvents))
	s.maxEndCount = max(s.maxEndCount, len(events.EndEvents))
	s.DrawTextLine("max begin touch events = %d", s.maxBeginCount)
	s.DrawTextLine("max end touch events = %d", s.maxEndCount)
}

// filter is BenchmarkSensor::Filter: the sensors of the filter row only
// detect shapes while they are the active sensor.
func (s *Sensor) filter(idA, idB b2.ShapeId) bool {
	var userData *sensorUserData
	if idA.IsSensor() {
		userData, _ = idA.GetUserData().(*sensorUserData)
	} else if idB.IsSensor() {
		userData, _ = idB.GetUserData().(*sensorUserData)
	}

	if userData != nil {
		return userData.active || userData.row != s.filterRow
	}

	return true
}

// Barrel24 is the Box2D v2.4 barrel, used to compare performance with that
// version. It mirrors BenchmarkBarrel24.
type Barrel24 struct {
	Base
}

// NewBarrel24 builds the v2.4 barrel scene.
func NewBarrel24(ctx *SampleContext) Sample {
	s := &Barrel24{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 8, Y: 53}
		ctx.Camera.Zoom = 25 * 2.35
	}

	groundSize := b2.F(25)

	{
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(s.WorldId, &bodyDef)

		box := b2.MakeBox(groundSize, b2.F(1.2))
		shapeDef := b2.DefaultShapeDef()
		b2.CreatePolygonShape(groundId, &shapeDef, &box)

		// A quarter turn is the reference's 0.5 * pi.
		bodyDef.Rotation = b2.MakeRot(b2.QFromRatio(1, 4))
		bodyDef.Position = b2.Vec2{X: groundSize, Y: b2.F(2).Mul(groundSize)}
		groundId = b2.CreateBody(s.WorldId, &bodyDef)

		box = b2.MakeBox(b2.F(2).Mul(groundSize), b2.F(1.2))
		b2.CreatePolygonShape(groundId, &shapeDef, &box)

		bodyDef.Position = b2.Vec2{X: groundSize.Neg(), Y: b2.F(2).Mul(groundSize)}
		groundId = b2.CreateBody(s.WorldId, &bodyDef)
		b2.CreatePolygonShape(groundId, &shapeDef, &box)
	}

	num := 26
	rad := b2.QHalf()

	shift := rad.Mul(b2.F(2))
	centerX := shift.Mul(b2.F(num)).Div(b2.F(2))
	centerY := shift.Div(b2.F(2))

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody

	shapeDef := b2.DefaultShapeDef()
	shapeDef.Density = b2.QOne()
	shapeDef.Material.Friction = b2.F(0.5)

	cuboid := b2.MakeSquare(b2.QHalf())

	numj := 5 * num
	for i := range num {
		x := b2.F(i).Mul(shift).Sub(centerX)

		for j := range numj {
			y := b2.F(j).Mul(shift).Add(centerY).Add(b2.F(2))

			bodyDef.Position = b2.Vec2{X: x, Y: y}

			bodyId := b2.CreateBody(s.WorldId, &bodyDef)
			b2.CreatePolygonShape(bodyId, &shapeDef, &cuboid)
		}
	}

	return s
}

// Compounds is a pile of two-triangle compound bodies. It mirrors
// BenchmarkCompounds; the scene is CreateCompounds of shared/benchmarks.c.
type Compounds struct {
	Base
}

// NewCompounds builds the compounds benchmark scene.
func NewCompounds(ctx *SampleContext) Sample {
	s := &Compounds{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 50}
		ctx.Camera.Zoom = 25 * 2.2
		ctx.Settings.EnableSleep = false
	}

	benchCreateCompounds(s.WorldId)
	return s
}

// Washer stresses most aspects of simulation:
//   - broad-phase because pairs constantly change
//   - narrow-phase because contact recycling does not always apply
//   - constraint solver because there are many contacts
//   - hit events are enabled so there is extra velocity computation and reporting
//
// It mirrors BenchmarkWasher; the scene is CreateWasher of shared/benchmarks.c.
type Washer struct {
	Base
}

// NewWasher builds the washer benchmark scene.
func NewWasher(ctx *SampleContext) Sample {
	s := &Washer{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 1.5, Y: 10}
		ctx.Camera.Zoom = 20
	}

	benchCreateWasher(s.WorldId)
	return s
}

// Step advances the world and reports the hit event count.
func (s *Washer) Step() {
	s.Base.Step()

	events := s.WorldId.GetContactEvents()
	s.DrawTextLine("hits = %d", len(events.HitEvents))
}

// Junkyard is a large pile of small pentagons pushed by a kinematic body.
// It mirrors BenchmarkJunkyard; the scene is CreateJunkyard and StepJunkyard
// of shared/benchmarks.c.
type Junkyard struct {
	Base

	data benchJunkyard
}

// NewJunkyard builds the junkyard benchmark scene.
func NewJunkyard(ctx *SampleContext) Sample {
	s := &Junkyard{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 8, Y: 25}
		ctx.Camera.Zoom = 60
	}

	s.data.create(s.WorldId)
	return s
}

// Step moves the pusher and advances the world.
func (s *Junkyard) Step() {
	if !s.Context.Settings.Pause || s.Context.Settings.SingleStep {
		s.data.step(s.StepCount)
	}

	s.Base.Step()
}

// Capacity pushes the engine to the limit for a large pile. It terminates
// once simulation is deemed to be slow. The higher the body count achieved,
// the better. This benchmark stresses the sleep system more than any other;
// better results are achieved if sleeping is disabled. It mirrors
// BenchmarkCapacity.
type Capacity struct {
	Base

	square     b2.Polygon
	reachCount int
	done       bool
}

// NewCapacity builds the capacity benchmark scene.
func NewCapacity(ctx *SampleContext) Sample {
	s := &Capacity{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 150}
		ctx.Camera.Zoom = 200
	}

	{
		bodyDef := b2.DefaultBodyDef()
		bodyDef.Position.Y = b2.F(-5)
		groundId := b2.CreateBody(s.WorldId, &bodyDef)

		box := b2.MakeBox(b2.F(800), b2.F(5))
		shapeDef := b2.DefaultShapeDef()
		b2.CreatePolygonShape(groundId, &shapeDef, &box)
	}

	s.square = b2.MakeSquare(b2.QHalf())
	s.done = false
	s.reachCount = 0
	return s
}

// Step advances the world and drops another row until a step takes too long.
func (s *Capacity) Step() {
	s.Base.Step()

	millisecondLimit := 20.0

	profile := s.WorldId.GetProfile()
	if profile.Step > millisecondLimit {
		s.reachCount++
		if s.reachCount > 60 {
			// Hit the millisecond limit 60 times in a row.
			s.done = true
		}
	} else {
		s.reachCount = 0
	}

	if s.done {
		return
	}

	if s.StepCount&0x1F != 0x1F {
		return
	}

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.Position.Y = b2.F(200)

	shapeDef := b2.DefaultShapeDef()

	count := 200
	x := b2.F(-count)
	for range count {
		bodyDef.Position.X = x
		bodyDef.Position.Y = bodyDef.Position.Y.Add(b2.QHalf())

		bodyId := b2.CreateBody(s.WorldId, &bodyDef)
		b2.CreatePolygonShape(bodyId, &shapeDef, &s.square)

		x = x.Add(b2.F(2))
	}
}

// Queries is the queries benchmark scene with one of its queries drawn and
// the cost of the whole set. It mirrors BenchmarkQueries; the scene is
// CreateQueries and StepQueries of shared/benchmarks.c.
type Queries struct {
	Base

	data       benchQueries
	drawIndex  int
	minTime    float64
	lastTime   float64
	hitCount   int
	nodeVisits int
	leafVisits int
}

// NewQueries builds the queries benchmark scene.
func NewQueries(ctx *SampleContext) Sample {
	s := &Queries{Base: NewBase(ctx)}

	s.data.create(s.WorldId)

	if !ctx.Settings.Restart {
		extent := float64(s.data.extent)
		ctx.Camera.Center = Vec2f{X: 0.5 * extent, Y: 0.5 * extent}
		ctx.Camera.Zoom = 0.55 * extent
	}

	s.drawIndex = 0
	s.minTime = 1e6
	return s
}

// UpdateGui exposes the draw index control.
func (s *Queries) UpdateGui() {
	height := 80
	gui := s.Context.Gui
	gui.Begin("Benchmark: Queries", 10, s.Context.Camera.Height-height-50, 220, height)
	gui.SliderInt("draw", &s.drawIndex, 0, benchQueryCount-1)
	gui.End()
}

// Step runs the whole query set, advances the world and draws one query.
func (s *Queries) Step() {
	if !s.Context.Settings.Pause || s.Context.Settings.SingleStep {
		beforeNodes, beforeLeaves := s.data.stats.NodeVisits, s.data.stats.LeafVisits
		started := time.Now()
		s.hitCount = s.data.step(s.WorldId)
		s.lastTime = time.Since(started).Seconds() * 1000
		s.minTime = min(s.minTime, s.lastTime)
		s.nodeVisits = s.data.stats.NodeVisits - beforeNodes
		s.leafVisits = s.data.stats.LeafVisits - beforeLeaves
	}

	s.Base.Step()

	origin := s.data.origins[s.drawIndex]
	end := origin.Add(s.data.translations[s.drawIndex])
	s.Context.Draw.DrawLineFcn(origin, end, b2.ColorWhite)
	s.Context.Draw.DrawPointFcn(origin, b2.F(5), b2.ColorGreen)
	s.Context.Draw.DrawPointFcn(end, b2.F(5), b2.ColorRed)

	// The overlap box the same query runs.
	extent := b2.F(5)
	box := b2.AABB{
		LowerBound: b2.Vec2{X: origin.X.Sub(extent), Y: origin.Y.Sub(extent)},
		UpperBound: b2.Vec2{X: origin.X.Add(extent), Y: origin.Y.Add(extent)},
	}
	s.Context.Draw.DrawBoundsFcn(box, b2.ColorWhite)

	s.DrawTextLine("%d queries of each kind and filter, %d hits", benchQueryCount, s.hitCount)
	s.DrawTextLine("node visits = %d, leaf visits = %d", s.nodeVisits, s.leafVisits)
	s.DrawTextLine("queries ms = %.3f, min = %.3f", s.lastTime, s.minTime)
}
