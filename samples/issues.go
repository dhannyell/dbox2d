// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/sample_issues.cpp of Box2D v3.2.0

package samples

import (
	"github.com/dhannyell/dbox2d"
)

func init() {
	RegisterSample("Issues", "Unstable Prismatic Joints", NewUnstablePrismaticJoints)
	RegisterSample("Issues", "Unstable Windmill", NewUnstableWindmill)
}

// UnstablePrismaticJoints stresses the solver by putting a light mass between
// two bodies on a prismatic joint with a stiff spring. It can be made stable
// by increasing the size of the middle circle and/or the number of sub-steps.
// It mirrors the UnstablePrismaticJoints sample of sample_issues.cpp.
type UnstablePrismaticJoints struct {
	Base
}

// NewUnstablePrismaticJoints builds the unstable prismatic joint scene.
func NewUnstablePrismaticJoints(ctx *SampleContext) Sample {
	s := &UnstablePrismaticJoints{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 1.75}
		ctx.Camera.Zoom = 32
	}

	{
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		segment := b2.Segment{Point1: b2.V2(-100, 0), Point2: b2.V2(100, 0)}
		b2.CreateSegmentShape(groundId, &shapeDef, &segment)
	}

	var centerId b2.BodyId
	{
		bd := b2.DefaultBodyDef()
		bd.Type = b2.DynamicBody
		bd.Position = b2.V2(0, 3)
		centerId = b2.CreateBody(s.WorldId, &bd)

		sd := b2.DefaultShapeDef()

		// A radius of 0.1 diverges to inf/nan.
		circle := b2.Circle{Radius: b2.QHalf()}
		b2.CreateCircleShape(centerId, &sd, &circle)
	}

	jd := b2.DefaultPrismaticJointDef()
	jd.EnableSpring = true
	jd.Hertz = b2.F(10)
	jd.DampingRatio = b2.F(2)

	side := func(x float64, target float64) {
		bd := b2.DefaultBodyDef()
		bd.Type = b2.DynamicBody
		bd.Position = b2.V2(x, 3)
		bodyId := b2.CreateBody(s.WorldId, &bd)

		sd := b2.DefaultShapeDef()
		circle := b2.Circle{Radius: b2.F(2)}
		b2.CreateCircleShape(bodyId, &sd, &circle)

		jd.Base.BodyIdA = centerId
		jd.Base.BodyIdB = bodyId
		jd.TargetTranslation = FromFloat64(target)
		b2.CreatePrismaticJoint(s.WorldId, &jd)
	}

	side(-3.5, -3)
	side(3.5, 3)

	return s
}

// UnstableWindmill is a windmill of welded boxes around a heavy circle that
// has stability problems. It mirrors the UnstableWindmill sample of
// sample_issues.cpp.
type UnstableWindmill struct {
	Base
}

// NewUnstableWindmill builds the unstable windmill scene.
func NewUnstableWindmill(ctx *SampleContext) Sample {
	s := &UnstableWindmill{Base: NewBase(ctx)}

	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 1.75}
		ctx.Camera.Zoom = 32
	}

	{
		bodyDef := b2.DefaultBodyDef()
		groundId := b2.CreateBody(s.WorldId, &bodyDef)

		shapeDef := b2.DefaultShapeDef()
		segment := b2.Segment{Point1: b2.V2(-100, -10), Point2: b2.V2(100, -10)}
		b2.CreateSegmentShape(groundId, &shapeDef, &segment)
	}

	bdef := b2.DefaultBodyDef()
	bdef.GravityScale = b2.QZero()
	bdef.Type = b2.DynamicBody
	sdef := b2.DefaultShapeDef()
	sdef.Material = b2.DefaultSurfaceMaterial()
	sdef.Material.Friction = b2.F(0.1)

	// center
	bdef.Position = b2.V2(10, 10)
	center := b2.CreateBody(s.WorldId, &bdef)
	circle := b2.Circle{Radius: b2.F(5)}
	b2.CreateCircleShape(center, &sdef, &circle)

	// rotors
	wjdef := b2.DefaultWeldJointDef()

	// This simulation can be stabilized by using a lower constraint stiffness:
	// wjdef.Base.ConstraintHertz = 30
	wjdef.Base.BodyIdA = center

	rotor := func(position b2.Vec2, halfWidth, halfHeight float64, anchorA, anchorB b2.Vec2) {
		bdef.Position = position
		body := b2.CreateBody(s.WorldId, &bdef)
		polygon := b2.MakeBox(b2.F(halfWidth), b2.F(halfHeight))
		b2.CreatePolygonShape(body, &sdef, &polygon)
		wjdef.Base.LocalFrameA = b2.Transform{P: anchorA, Q: b2.RotIdentity()}
		wjdef.Base.BodyIdB = body
		wjdef.Base.LocalFrameB = b2.Transform{P: anchorB, Q: b2.RotIdentity()}
		b2.CreateWeldJoint(s.WorldId, &wjdef)
	}

	rotor(b2.V2(10, 0), 4, 5, b2.V2(0, -5), b2.V2(0, 5))
	rotor(b2.V2(20, 10), 5, 4, b2.V2(5, 0), b2.V2(-5, 0))
	rotor(b2.V2(10, 20), 4, 5, b2.V2(0, 5), b2.V2(0, -5))
	rotor(b2.V2(0, 10), 5, 4, b2.V2(-5, 0), b2.V2(5, 0))

	return s
}
