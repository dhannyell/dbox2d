// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/sample_world.cpp of Box2D v3.2.0

package samples

import (
	"fmt"
	"math"

	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/dbox2d/internal/shared"
)

func init() {
	RegisterSample("World", "Tiles", NewTileWorld)
	RegisterSample("World", "Far Pyramid", NewFarPyramid)
	RegisterSample("World", "Far Ragdolls", NewFarRagdolls)
	RegisterSample("World", "Far Gate", NewFarGate)
}

// The reference uses 10 cycles in debug builds and 600 otherwise; the port
// has no debug build and keeps the release size.
const (
	tileWorldPeriod     = 40
	tileWorldCycleCount = 600
	tileWorldGridSize   = 1
	tileWorldGridCount  = tileWorldCycleCount * tileWorldPeriod / tileWorldGridSize
)

// TileWorld is a 24 kilometer world with bodies spread along its length.
type TileWorld struct {
	Base

	car               car
	viewPosition      Vec2f
	cycleIndex        int
	speed             float64
	explosionPosition b2.Vec2
	explode           bool
	followCar         bool
}

func NewTileWorld(ctx *SampleContext) Sample {
	s := &TileWorld{Base: NewBase(ctx)}

	omega := float32(2.0 * math.Pi / tileWorldPeriod)

	xStart := -tileWorldCycleCount * tileWorldPeriod / 2

	s.viewPosition = Vec2f{X: float64(xStart), Y: 15}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = s.viewPosition
		ctx.Camera.Zoom = 25 * 1
		ctx.Settings.DrawJoints = false
	}

	{
		bodyDef := b2.DefaultBodyDef()
		shapeDef := b2.DefaultShapeDef()

		// Setting this to false significantly reduces the cost of creating
		// static bodies and shapes.
		shapeDef.InvokeContactCreation = false

		var height float32 = 4
		xBody := xStart
		xShape := xStart

		gridSize := b2.F(tileWorldGridSize)
		halfExtent := b2.F(0.4).Mul(gridSize)

		var groundId b2.BodyId

		for i := range tileWorldGridCount {
			// Create a new body regularly so that shapes are not too far from the body origin.
			// Most algorithms in Box2D work in local coordinates, but contact points are computed
			// relative to the body origin.
			// This makes a noticeable improvement in stability far from the origin.
			if i%10 == 0 {
				setPosX(&bodyDef.Position, b2.F(xBody))
				groundId = b2.CreateBody(s.WorldId, &bodyDef)
				xShape = 0
			}

			y := b2.QZero()

			// The ground profile is the float32 cosine of the reference.
			cosine := float32(math.Cos(float64(omega * float32(xBody))))
			ycount := int(math.Round(float64(height*cosine))) + 12

			for range ycount {
				square := b2.MakeOffsetBox(halfExtent, halfExtent, b2.Vec2{X: b2.F(xShape), Y: y}, b2.RotIdentity())
				square.Radius = b2.F(0.1)
				b2.CreatePolygonShape(groundId, &shapeDef, &square)

				y = y.Add(gridSize)
			}

			xBody += tileWorldGridSize
			xShape += tileWorldGridSize
		}
	}

	humanIndex := 0
	for cycleIndex := range tileWorldCycleCount {
		// (0.5 + cycleIndex) * period + xStart
		xbase := b2.F(tileWorldPeriod/2 + cycleIndex*tileWorldPeriod + xStart)

		remainder := cycleIndex % 3
		if remainder == 0 {
			bodyDef := b2.DefaultBodyDef()
			bodyDef.Type = b2.DynamicBody
			bodyDef.Position = b2.ToPos(b2.Vec2{X: xbase.Sub(b2.F(3)), Y: b2.F(10)})

			shapeDef := b2.DefaultShapeDef()
			box := b2.MakeBox(b2.F(0.3), b2.F(0.2))

			for range 10 {
				setPosY(&bodyDef.Position, b2.F(10))
				for range 5 {
					bodyId := b2.CreateBody(s.WorldId, &bodyDef)
					b2.CreatePolygonShape(bodyId, &shapeDef, &box)
					setPosY(&bodyDef.Position, b2.ToVec2(bodyDef.Position).Y.Add(b2.QHalf()))
				}
				setPosX(&bodyDef.Position, b2.ToVec2(bodyDef.Position).X.Add(b2.F(0.6)))
			}
		} else if remainder == 1 {
			position := b2.Vec2{X: xbase.Sub(b2.F(2)), Y: b2.F(10)}
			for range 5 {
				shared.CreateHuman(s.WorldId, b2.ToPos(position), b2.F(1.5), b2.F(0.05), b2.QZero(), b2.QZero(), humanIndex+1, nil, false)
				humanIndex += 1
				position.X = position.X.Add(b2.QOne())
			}
		} else {
			position := b2.Vec2{X: xbase.Sub(b2.F(4)), Y: b2.F(12)}

			for range 5 {
				var d donut
				d.create(s.WorldId, position, b2.F(0.75), 0, false, nil)
				position.X = position.X.Add(b2.F(2))
			}
		}
	}

	s.car.spawn(s.WorldId, b2.Vec2{X: b2.F(xStart + 20), Y: b2.F(40)},
		b2.F(10), b2.F(2), b2.F(0.7), b2.F(2000), nil)

	s.cycleIndex = 0
	s.speed = 0
	s.explosionPosition = b2.Vec2{
		X: b2.F(tileWorldPeriod/2 + s.cycleIndex*tileWorldPeriod + xStart),
		Y: b2.F(7),
	}
	s.explode = true
	s.followCar = false
	return s
}

func (s *TileWorld) UpdateGui() {
	height := 160
	gui := s.Context.Gui
	gui.Begin("Tiles", 10, s.Context.Camera.Height-height-50, 240, height)

	gui.SliderFloat("speed", &s.speed, -400, 400)
	if gui.Button("stop") {
		s.speed = 0
	}

	gui.Checkbox("explode", &s.explode)
	gui.Checkbox("follow car", &s.followCar)

	gui.Text(fmt.Sprintf("world size = %g kilometers", float64(tileWorldGridSize*tileWorldGridCount)/1000))
	gui.End()
}

func (s *TileWorld) Step() {
	span := tileWorldPeriod * tileWorldCycleCount / 2
	timeStep := 0.0
	if s.Context.Settings.Hertz > 0 {
		timeStep = 1 / s.Context.Settings.Hertz
	}

	if s.Context.Settings.Pause {
		timeStep = 0
	}

	s.viewPosition.X += timeStep * s.speed
	s.viewPosition.X = min(max(s.viewPosition.X, -float64(span)), float64(span))

	if s.speed != 0 {
		s.Context.Camera.Center = s.viewPosition
	}

	if s.followCar {
		s.Context.Camera.Center.X = ToFloat64(b2.ToVec2(s.car.chassisId.GetPosition()).X)
	}

	radius := b2.F(2)
	if s.StepCount&0x1 == 0x1 && s.explode {
		s.explosionPosition.X = b2.F(tileWorldPeriod/2 + s.cycleIndex*tileWorldPeriod - span)

		def := b2.DefaultExplosionDef()
		def.Position = b2.ToPos(s.explosionPosition)
		def.Radius = radius
		def.Falloff = b2.F(0.1)
		def.ImpulsePerLength = b2.QOne()
		s.WorldId.Explode(&def)

		s.cycleIndex = (s.cycleIndex + 1) % tileWorldCycleCount
	}

	if s.explode {
		s.Context.Draw.DrawCircleFcn(b2.ToPos(s.explosionPosition), radius, b2.ColorAzure)
	}

	if s.keyDown(KeyA) {
		s.car.setSpeed(b2.F(20))
	}

	if s.keyDown(KeyS) {
		s.car.setSpeed(b2.QZero())
	}

	if s.keyDown(KeyD) {
		s.car.setSpeed(b2.F(-5))
	}

	s.Base.Step()
}

// scalarModeLabel names the scalar of this build for the overlay. The
// reference prints single or double precision of its position type.
func scalarModeLabel() string {
	if b2.ScalarMode == "fixed" {
		return "Fixed-point Q32.32"
	}
	if b2.IsDoublePrecision() {
		return "Float32 with double"
	}
	return "Float32"
}

// gridStepAt returns the spacing of representable positions at x. Under the
// double tag positions are float64; every other quantity stays float32.
func gridStepAt(x float64) float64 {
	if b2.ScalarMode == "fixed" {
		return math.Ldexp(1, -32)
	}
	if b2.IsDoublePrecision() {
		return math.Nextafter(x, math.Inf(1)) - x
	}
	fx := float32(x)
	return float64(math.Nextafter32(fx, float32(math.Inf(1))) - fx)
}

// FarPyramid is a pyramid built far from the origin to exercise large world
// positions.
type FarPyramid struct {
	Base
}

// NewFarPyramid builds the far pyramid scene.
func NewFarPyramid(ctx *SampleContext) Sample {
	s := &FarPyramid{Base: NewBase(ctx)}

	// 1e7 is exactly representable in float, so integer box offsets stay exact.
	origin := b2.ToPos(b2.Vec2{X: b2.F(10.0e6), Y: b2.QZero()})

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 10.0e6, Y: 12}
		ctx.Camera.Zoom = 17
	}

	h := b2.F(0.25)

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Position = origin
	groundId := b2.CreateBody(s.WorldId, &bodyDef)

	shapeDef := b2.DefaultShapeDef()
	segment := b2.Segment{Point1: b2.V2(-40, 0), Point2: b2.V2(40, 0)}
	b2.CreateSegmentShape(groundId, &shapeDef, &segment)

	box := b2.MakeBox(h, h)
	bodyDef.Type = b2.DynamicBody

	const baseCount = 50
	for i := range baseCount {
		y := b2.F(2*i + 1).Mul(h)
		for j := i; j < baseCount; j++ {
			x := b2.F(i + 1).Mul(h).Add(b2.F(2 * (j - i)).Mul(h)).Sub(h.Mul(b2.F(baseCount)))
			bodyDef.Position = b2.OffsetPos(origin, b2.Vec2{X: x, Y: y})
			bodyId := b2.CreateBody(s.WorldId, &bodyDef)
			b2.CreatePolygonShape(bodyId, &shapeDef, &box)
		}
	}
	return s
}

func (s *FarPyramid) Step() {
	s.Base.Step()

	s.DrawTextLine("%s positions", scalarModeLabel())
	s.DrawTextLine("view center %.0f km from world origin", 0.001*s.Context.Camera.Center.X)
}

// FarRagdolls is a pile of ragdolls far from the origin.
type FarRagdolls struct {
	Base

	humans     []shared.Human
	humanCount int
}

const (
	farRagdollColumns = 5
	farRagdollRows    = 5
)

// NewFarRagdolls builds the far ragdolls scene.
func NewFarRagdolls(ctx *SampleContext) Sample {
	s := &FarRagdolls{Base: NewBase(ctx)}

	origin := b2.ToPos(b2.Vec2{X: b2.F(10.0e6), Y: b2.QZero()})

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 10.0e6, Y: 6}
		ctx.Camera.Zoom = 10
	}

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Position = origin
	groundId := b2.CreateBody(s.WorldId, &bodyDef)

	shapeDef := b2.DefaultShapeDef()
	w := b2.F(6)
	h := b2.F(12)
	floor := b2.Segment{Point1: b2.Vec2{X: w.Neg(), Y: b2.QZero()}, Point2: b2.Vec2{X: w, Y: b2.QZero()}}
	leftWall := b2.Segment{Point1: b2.Vec2{X: w.Neg(), Y: b2.QZero()}, Point2: b2.Vec2{X: w.Neg(), Y: h}}
	rightWall := b2.Segment{Point1: b2.Vec2{X: w, Y: b2.QZero()}, Point2: b2.Vec2{X: w, Y: h}}
	b2.CreateSegmentShape(groundId, &shapeDef, &floor)
	b2.CreateSegmentShape(groundId, &shapeDef, &leftWall)
	b2.CreateSegmentShape(groundId, &shapeDef, &rightWall)

	scale := b2.QOne()
	index := 0
	s.humans = make([]shared.Human, farRagdollColumns*farRagdollRows)
	for i := range farRagdollRows {
		for j := range farRagdollColumns {
			// 2.4 * scale * (j - 0.5 * (columns - 1)) + random(-0.3, 0.3)
			x := b2.F(2.4).Mul(scale).Mul(b2.F(j).Sub(b2.F(0.5).Mul(b2.F(farRagdollColumns - 1)))).
				Add(shared.RandomFloatRange(b2.F(-0.3), b2.F(0.3)))
			y := b2.F(2).Add(b2.F(2.2).Mul(scale).Mul(b2.F(i)))
			p := b2.OffsetPos(origin, b2.Vec2{X: x, Y: y})
			s.humans[index] = shared.CreateHuman(s.WorldId, p, scale, b2.F(0.05), b2.QOne(), b2.F(0.5), index+1, nil, false)
			index++
		}
	}
	s.humanCount = index
	return s
}

func (s *FarRagdolls) Step() {
	s.Base.Step()

	s.DrawTextLine("%s positions", scalarModeLabel())
	s.DrawTextLine("%d ragdolls piled %.0f km from the world origin", s.humanCount, 0.001*s.Context.Camera.Center.X)
}

// FarGate is the Gear Lift mechanism rebuilt 1000 km from the origin.
type FarGate struct {
	Base

	origin      b2.Pos
	driverId    b2.JointId
	motorTorque float64
	motorSpeed  float64
	enableMotor bool
}

// NewFarGate builds the far gate scene.
func NewFarGate(ctx *SampleContext) Sample {
	g := &FarGate{Base: NewBase(ctx)}

	// 1e6 is exactly representable in float so the placement adds no error of its own.
	origin := b2.ToPos(b2.Vec2{X: b2.F(1.0e6), Y: b2.QZero()})
	g.origin = origin

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 1.0e6, Y: 6}
		ctx.Camera.Zoom = 7
		ctx.Settings.DrawJoints = false
	}

	bodyDef := b2.DefaultBodyDef()
	bodyDef.Position = origin
	groundId := b2.CreateBody(g.WorldId, &bodyDef)

	path := "m 63.500002,201.08333 103.187498,0 1e-5,-37.04166 h -2.64584 l 0,34.39583 h -42.33333 v -2.64583 l " +
		"-2.64584,-1e-5 v -2.64583 h -2.64583 v -2.64584 h -2.64584 v -2.64583 H 111.125 v -2.64583 h -2.64583 v " +
		"-2.64583 h -2.64583 v -2.64584 l -2.64584,1e-5 v -2.64583 l -2.64583,-1e-5 V 174.625 h -2.645834 v -2.64584 l " +
		"-2.645833,1e-5 v -2.64584 H 92.60417 v -2.64583 h -2.645834 v -2.64583 l -26.458334,0 0,37.04166"
	material := b2.DefaultSurfaceMaterial()
	material.CustomColor = uint32(b2.ColorDarkSeaGreen)
	chainDef := b2.DefaultChainDef()
	chainDef.Points = parsePath(path, b2.V2(-120, -200), 64, b2.F(0.2))
	chainDef.IsLoop = true
	chainDef.Materials = []b2.SurfaceMaterial{material}
	b2.CreateChain(groundId, &chainDef)

	gearRadius := b2.F(1)
	toothHalfWidth := b2.F(0.09)
	toothHalfHeight := b2.F(0.06)
	toothRadius := b2.F(0.03)
	linkHalfLength := b2.F(0.07)
	linkRadius := b2.F(0.05)
	doorHalfHeight := b2.F(1.5)

	gearPosition1 := b2.OffsetPos(origin, b2.V2(-4.25, 9.75))
	gearPosition2 := b2.OffsetPos(gearPosition1, b2.V2(2, 1))
	linkAttachPosition := b2.OffsetPos(gearPosition2, b2.Vec2{X: gearRadius.Add(toothHalfWidth.Mul(b2.F(2))).Add(toothRadius)})
	doorPosition := b2.OffsetPos(linkAttachPosition, b2.Vec2{Y: linkHalfLength.Mul(b2.F(2)).Mul(b2.F(40)).Add(doorHalfHeight).Neg()})

	// driver gear
	bodyDef = b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.Position = gearPosition1
	driverBodyId := b2.CreateBody(g.WorldId, &bodyDef)

	shapeDef := b2.DefaultShapeDef()
	shapeDef.Material.Friction = b2.F(0.1)
	shapeDef.Material.CustomColor = uint32(b2.ColorSaddleBrown)
	circle := b2.Circle{Center: b2.Vec2{}, Radius: gearRadius}
	b2.CreateCircleShape(driverBodyId, &shapeDef, &circle)

	dq := b2.MakeRot(b2.QFromRatio(1, 16))
	center := b2.Vec2{X: gearRadius.Add(toothHalfHeight)}
	rotation := b2.RotIdentity()
	for range 16 {
		tooth := b2.MakeOffsetRoundedBox(toothHalfWidth, toothHalfHeight, center, rotation, toothRadius)
		shapeDef.Material.CustomColor = uint32(b2.ColorGray)
		b2.CreatePolygonShape(driverBodyId, &shapeDef, &tooth)
		rotation = b2.MulRot(dq, rotation)
		center = b2.RotateVector(rotation, b2.Vec2{X: gearRadius.Add(toothHalfHeight)})
	}

	revoluteDef := b2.DefaultRevoluteJointDef()
	g.motorTorque = 80
	g.motorSpeed = -0.3
	g.enableMotor = true
	revoluteDef.Base.BodyIdA = groundId
	revoluteDef.Base.BodyIdB = driverBodyId
	revoluteDef.Base.LocalFrameA.P = groundId.GetLocalPoint(gearPosition1)
	revoluteDef.Base.LocalFrameB.P = b2.Vec2{}
	revoluteDef.EnableMotor = g.enableMotor
	revoluteDef.MaxMotorTorque = FromFloat64(g.motorTorque)
	revoluteDef.MotorSpeed = radiansToTurns(g.motorSpeed)
	g.driverId = b2.CreateRevoluteJoint(g.WorldId, &revoluteDef)

	// follower gear
	bodyDef.Position = gearPosition2
	followerId := b2.CreateBody(g.WorldId, &bodyDef)
	shapeDef = b2.DefaultShapeDef()
	shapeDef.Material.Friction = b2.F(0.1)
	shapeDef.Material.CustomColor = uint32(b2.ColorSaddleBrown)
	b2.CreateCircleShape(followerId, &shapeDef, &circle)
	center = b2.Vec2{X: gearRadius.Add(toothHalfWidth)}
	rotation = b2.RotIdentity()
	for range 16 {
		tooth := b2.MakeOffsetRoundedBox(toothHalfWidth, toothHalfHeight, center, rotation, toothRadius)
		shapeDef.Material.CustomColor = uint32(b2.ColorGray)
		b2.CreatePolygonShape(followerId, &shapeDef, &tooth)
		rotation = b2.MulRot(dq, rotation)
		center = b2.RotateVector(rotation, b2.Vec2{X: gearRadius.Add(toothHalfWidth)})
	}
	revoluteDef = b2.DefaultRevoluteJointDef()
	revoluteDef.Base.BodyIdA = groundId
	revoluteDef.Base.BodyIdB = followerId
	revoluteDef.Base.LocalFrameA.P = groundId.GetLocalPoint(gearPosition2)
	revoluteDef.Base.LocalFrameA.Q = b2.MakeRot(b2.QFromRatio(1, 8))
	revoluteDef.Base.LocalFrameB.P = b2.Vec2{}
	revoluteDef.EnableMotor = true
	revoluteDef.MaxMotorTorque = b2.F(0.5)
	revoluteDef.LowerAngle = b2.QFromRatio(-15, 100)
	revoluteDef.UpperAngle = b2.QFromRatio(4, 10)
	revoluteDef.EnableLimit = true
	b2.CreateRevoluteJoint(g.WorldId, &revoluteDef)

	// chain of links
	capsule := b2.Capsule{Center1: b2.Vec2{Y: linkHalfLength.Neg()}, Center2: b2.Vec2{Y: linkHalfLength}, Radius: linkRadius}
	shapeDef = b2.DefaultShapeDef()
	shapeDef.Density = b2.F(2)
	shapeDef.Material.CustomColor = uint32(b2.ColorLightSteelBlue)
	jointDef := b2.DefaultRevoluteJointDef()
	jointDef.MaxMotorTorque = b2.F(0.05)
	jointDef.EnableMotor = true
	bodyDef = b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	position := b2.OffsetPos(linkAttachPosition, b2.Vec2{Y: linkHalfLength.Neg()})
	prevBodyId := followerId
	var lastLinkId b2.BodyId
	for range 40 {
		bodyDef.Position = position
		bodyId := b2.CreateBody(g.WorldId, &bodyDef)
		b2.CreateCapsuleShape(bodyId, &shapeDef, &capsule)

		pivot := b2.ToPos(b2.Vec2{X: b2.ToVec2(position).X, Y: b2.ToVec2(position).Y.Add(linkHalfLength)})
		jointDef.Base.BodyIdA = prevBodyId
		jointDef.Base.BodyIdB = bodyId
		jointDef.Base.LocalFrameA.P = prevBodyId.GetLocalPoint(pivot)
		jointDef.Base.LocalFrameB.P = bodyId.GetLocalPoint(pivot)
		jointDef.Base.DrawScale = b2.F(0.2)
		b2.CreateRevoluteJoint(g.WorldId, &jointDef)

		setPosY(&position, b2.ToVec2(position).Y.Sub(linkHalfLength.Mul(b2.F(2))))
		prevBodyId = bodyId
		lastLinkId = bodyId
	}

	// door
	bodyDef = b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	bodyDef.Position = doorPosition
	doorBodyId := b2.CreateBody(g.WorldId, &bodyDef)
	box := b2.MakeBox(b2.F(0.15), doorHalfHeight)
	shapeDef = b2.DefaultShapeDef()
	shapeDef.Material.Friction = b2.F(0.1)
	shapeDef.Material.CustomColor = uint32(b2.ColorDarkCyan)
	b2.CreatePolygonShape(doorBodyId, &shapeDef, &box)

	pivot := b2.OffsetPos(doorPosition, b2.Vec2{Y: doorHalfHeight})
	revoluteDef = b2.DefaultRevoluteJointDef()
	revoluteDef.Base.BodyIdA = lastLinkId
	revoluteDef.Base.BodyIdB = doorBodyId
	revoluteDef.Base.LocalFrameA.P = lastLinkId.GetLocalPoint(pivot)
	revoluteDef.Base.LocalFrameB.P = b2.Vec2{Y: doorHalfHeight}
	revoluteDef.EnableMotor = true
	revoluteDef.MaxMotorTorque = b2.F(0.05)
	b2.CreateRevoluteJoint(g.WorldId, &revoluteDef)

	prismaticDef := b2.DefaultPrismaticJointDef()
	prismaticDef.Base.BodyIdA = groundId
	prismaticDef.Base.BodyIdB = doorBodyId
	prismaticDef.Base.LocalFrameA.P = groundId.GetLocalPoint(doorPosition)
	prismaticDef.Base.LocalFrameA.Q = b2.MakeRotFromUnitVector(b2.Vec2{Y: b2.QOne()})
	prismaticDef.Base.LocalFrameB.P = b2.Vec2{}
	prismaticDef.Base.LocalFrameB.Q = prismaticDef.Base.LocalFrameA.Q
	prismaticDef.MaxMotorForce = b2.F(0.2)
	prismaticDef.EnableMotor = true
	prismaticDef.Base.CollideConnected = true
	b2.CreatePrismaticJoint(g.WorldId, &prismaticDef)

	// rubble
	bodyDef = b2.DefaultBodyDef()
	bodyDef.Type = b2.DynamicBody
	shapeDef = b2.DefaultShapeDef()
	shapeDef.Material.RollingResistance = b2.F(0.3)
	colors := [5]uint32{uint32(b2.ColorGray), uint32(b2.ColorGainsboro), uint32(b2.ColorLightGray), uint32(b2.ColorLightSlateGray), uint32(b2.ColorDarkGray)}
	y := b2.F(4.25)
	for range 20 {
		x := b2.F(-3.15)
		for range 10 {
			bodyDef.Position = b2.OffsetPos(origin, b2.Vec2{X: x, Y: y})
			bodyId := b2.CreateBody(g.WorldId, &bodyDef)
			poly := shared.RandomPolygon(b2.F(0.1))
			poly.Radius = shared.RandomFloatRange(b2.F(0.01), b2.F(0.02))
			shapeDef.Material.CustomColor = colors[shared.RandomIntRange(0, 4)]
			b2.CreatePolygonShape(bodyId, &shapeDef, &poly)
			x = x.Add(b2.F(0.2))
		}
		y = y.Add(b2.F(0.2))
	}

	return g
}

func (g *FarGate) UpdateGui() {
	const height = 120
	gui := g.Context.Gui
	gui.Begin("Far Gate", 10, g.Context.Camera.Height-height-25, 240, height)
	if gui.Checkbox("Motor", &g.enableMotor) {
		g.driverId.EnableMotor(g.enableMotor)
		g.driverId.WakeBodies()
	}
	if gui.SliderFloat("Max Torque", &g.motorTorque, 0, 100) {
		g.driverId.SetMaxMotorTorque(FromFloat64(g.motorTorque))
		g.driverId.WakeBodies()
	}
	if gui.SliderFloat("Speed", &g.motorSpeed, -0.3, 0.3) {
		g.driverId.SetTargetMotorSpeed(radiansToTurns(g.motorSpeed))
		g.driverId.WakeBodies()
	}
	gui.End()
}

func (g *FarGate) Step() {
	if g.keyDown(KeyA) {
		g.motorSpeed = max(-0.3, g.motorSpeed-0.01)
		g.driverId.SetTargetMotorSpeed(radiansToTurns(g.motorSpeed))
		g.driverId.WakeBodies()
	}
	if g.keyDown(KeyD) {
		g.motorSpeed = min(0.3, g.motorSpeed+0.01)
		g.driverId.SetTargetMotorSpeed(radiansToTurns(g.motorSpeed))
		g.driverId.WakeBodies()
	}

	g.Base.Step()

	g.DrawTextLine("%s positions", scalarModeLabel())
	g.DrawTextLine("%.0f km from the origin", ToFloat64(b2.ToVec2(g.origin).X)/1000)
	g.DrawTextLine("scalar grid step here = %g m, gear teeth are about 0.12 m", gridStepAt(ToFloat64(b2.ToVec2(g.origin).X)))
}
