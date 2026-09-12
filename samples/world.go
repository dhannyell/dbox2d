// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/sample_world.cpp of Box2D v3.1.1

package samples

import (
	"fmt"
	"math"

	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/dbox2d/internal/shared"
)

func init() {
	RegisterSample("World", "Large World", NewLargeWorld)
}

// The reference uses 10 cycles in debug builds and 600 otherwise; the port
// has no debug build and keeps the release size.
const (
	largeWorldPeriod     = 40
	largeWorldCycleCount = 600
	largeWorldGridSize   = 1
	largeWorldGridCount  = largeWorldCycleCount * largeWorldPeriod / largeWorldGridSize
)

// LargeWorld is a 24 kilometer world with bodies spread along its length.
type LargeWorld struct {
	Base

	car               car
	viewPosition      Vec2f
	cycleIndex        int
	speed             float64
	explosionPosition dbox2d.Vec2
	explode           bool
	followCar         bool
}

func NewLargeWorld(ctx *SampleContext) Sample {
	s := &LargeWorld{Base: NewBase(ctx)}

	omega := float32(2.0 * math.Pi / largeWorldPeriod)

	xStart := -largeWorldCycleCount * largeWorldPeriod / 2

	s.viewPosition = Vec2f{X: float64(xStart), Y: 15}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = s.viewPosition
		ctx.Camera.Zoom = 25 * 1
		ctx.Settings.DrawJoints = false
		ctx.Settings.UseCameraBounds = true
	}

	{
		bodyDef := dbox2d.DefaultBodyDef()
		shapeDef := dbox2d.DefaultShapeDef()

		// Setting this to false significantly reduces the cost of creating
		// static bodies and shapes.
		shapeDef.InvokeContactCreation = false

		var height float32 = 4
		xBody := xStart
		xShape := xStart

		gridSize := dbox2d.QFromInt(largeWorldGridSize)
		halfExtent := qs("0.4").Mul(gridSize)

		var groundId dbox2d.BodyId

		for i := range largeWorldGridCount {
			// Create a new body regularly so that shapes are not too far from the body origin.
			// Most algorithms in Box2D work in local coordinates, but contact points are computed
			// relative to the body origin.
			// This makes a noticeable improvement in stability far from the origin.
			if i%10 == 0 {
				bodyDef.Position.X = dbox2d.QFromInt(xBody)
				groundId = dbox2d.CreateBody(s.WorldId, &bodyDef)
				xShape = 0
			}

			y := dbox2d.QZero()

			// The ground profile is the float32 cosine of the reference.
			cosine := float32(math.Cos(float64(omega * float32(xBody))))
			ycount := int(math.Round(float64(height*cosine))) + 12

			for range ycount {
				square := dbox2d.MakeOffsetBox(halfExtent, halfExtent, dbox2d.Vec2{X: dbox2d.QFromInt(xShape), Y: y}, dbox2d.RotIdentity())
				square.Radius = qs("0.1")
				dbox2d.CreatePolygonShape(groundId, &shapeDef, &square)

				y = y.Add(gridSize)
			}

			xBody += largeWorldGridSize
			xShape += largeWorldGridSize
		}
	}

	humanIndex := 0
	for cycleIndex := range largeWorldCycleCount {
		// (0.5 + cycleIndex) * period + xStart
		xbase := dbox2d.QFromInt(largeWorldPeriod/2 + cycleIndex*largeWorldPeriod + xStart)

		remainder := cycleIndex % 3
		if remainder == 0 {
			bodyDef := dbox2d.DefaultBodyDef()
			bodyDef.Type = dbox2d.DynamicBody
			bodyDef.Position = dbox2d.Vec2{X: xbase.Sub(dbox2d.QFromInt(3)), Y: dbox2d.QFromInt(10)}

			shapeDef := dbox2d.DefaultShapeDef()
			box := dbox2d.MakeBox(qs("0.3"), qs("0.2"))

			for range 10 {
				bodyDef.Position.Y = dbox2d.QFromInt(10)
				for range 5 {
					bodyId := dbox2d.CreateBody(s.WorldId, &bodyDef)
					dbox2d.CreatePolygonShape(bodyId, &shapeDef, &box)
					bodyDef.Position.Y = bodyDef.Position.Y.Add(dbox2d.QHalf())
				}
				bodyDef.Position.X = bodyDef.Position.X.Add(qs("0.6"))
			}
		} else if remainder == 1 {
			position := dbox2d.Vec2{X: xbase.Sub(dbox2d.QFromInt(2)), Y: dbox2d.QFromInt(10)}
			for range 5 {
				shared.CreateHuman(s.WorldId, position, qs("1.5"), qs("0.05"), dbox2d.QZero(), dbox2d.QZero(), humanIndex+1, nil, false)
				humanIndex += 1
				position.X = position.X.Add(dbox2d.QOne())
			}
		} else {
			position := dbox2d.Vec2{X: xbase.Sub(dbox2d.QFromInt(4)), Y: dbox2d.QFromInt(12)}

			for range 5 {
				var d donut
				d.create(s.WorldId, position, qs("0.75"), 0, false, nil)
				position.X = position.X.Add(dbox2d.QFromInt(2))
			}
		}
	}

	s.car.spawn(s.WorldId, dbox2d.Vec2{X: dbox2d.QFromInt(xStart + 20), Y: dbox2d.QFromInt(40)},
		dbox2d.QFromInt(10), dbox2d.QFromInt(2), qs("0.7"), dbox2d.QFromInt(2000), nil)

	s.cycleIndex = 0
	s.speed = 0
	s.explosionPosition = dbox2d.Vec2{
		X: dbox2d.QFromInt(largeWorldPeriod/2 + s.cycleIndex*largeWorldPeriod + xStart),
		Y: dbox2d.QFromInt(7),
	}
	s.explode = true
	s.followCar = false
	return s
}

func (s *LargeWorld) UpdateGui() {
	height := 160
	gui := s.Context.Gui
	gui.Begin("Large World", 10, s.Context.Camera.Height-height-50, 240, height)

	gui.SliderFloat("speed", &s.speed, -400, 400)
	if gui.Button("stop") {
		s.speed = 0
	}

	gui.Checkbox("explode", &s.explode)
	gui.Checkbox("follow car", &s.followCar)

	gui.Text(fmt.Sprintf("world size = %g kilometers", float64(largeWorldGridSize*largeWorldGridCount)/1000))
	gui.End()
}

func (s *LargeWorld) Step() {
	span := largeWorldPeriod * largeWorldCycleCount / 2
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
		s.Context.Camera.Center.X = ToFloat64(s.car.chassisId.GetPosition().X)
	}

	radius := dbox2d.QFromInt(2)
	if s.StepCount&0x1 == 0x1 && s.explode {
		s.explosionPosition.X = dbox2d.QFromInt(largeWorldPeriod/2 + s.cycleIndex*largeWorldPeriod - span)

		def := dbox2d.DefaultExplosionDef()
		def.Position = s.explosionPosition
		def.Radius = radius
		def.Falloff = qs("0.1")
		def.ImpulsePerLength = dbox2d.QOne()
		s.WorldId.Explode(&def)

		s.cycleIndex = (s.cycleIndex + 1) % largeWorldCycleCount
	}

	if s.explode {
		s.Context.Draw.DrawCircle(s.explosionPosition, radius, dbox2d.ColorAzure)
	}

	if s.keyDown(KeyA) {
		s.car.setSpeed(dbox2d.QFromInt(20))
	}

	if s.keyDown(KeyS) {
		s.car.setSpeed(dbox2d.QZero())
	}

	if s.keyDown(KeyD) {
		s.car.setSpeed(dbox2d.QFromInt(-5))
	}

	s.Base.Step()
}
