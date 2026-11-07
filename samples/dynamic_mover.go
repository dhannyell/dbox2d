// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/dynamic_mover.h and samples/dynamic_mover.cpp of Box2D v3.2.0

package samples

import (
	"github.com/dhannyell/dbox2d"
)

// dynamicMoverDef configures a dynamicMover. It mirrors DynamicMoverDef.
// The tuning values are float64 because the sample sliders edit them.
type dynamicMoverDef struct {
	position             b2.Vec2
	capsule              b2.Capsule
	density              float64
	filter               b2.Filter
	jumpSpeed            float64
	maxSpeed             float64
	minSpeed             float64
	stopSpeed            float64
	accelerate           float64
	airSteer             float64
	friction             float64
	gravityScale         float64
	maxGroundForce       float64
	maxAirForce          float64
	pogoRestLength       float64
	pogoHertz            float64
	pogoDampingRatio     float64
	pogoCompressionScale float64
	pogoTensionScale     float64
	enablePreSolveEvents bool
}

// defaultDynamicMoverDef returns the default values of DynamicMoverDef.
func defaultDynamicMoverDef() dynamicMoverDef {
	return dynamicMoverDef{
		capsule:              b2.Capsule{Center1: b2.V2(0, -0.5), Center2: b2.V2(0, 0.5), Radius: b2.F(0.3)},
		density:              1.0,
		filter:               b2.DefaultFilter(),
		jumpSpeed:            7.0,
		maxSpeed:             6.0,
		minSpeed:             0.1,
		stopSpeed:            3.0,
		accelerate:           20.0,
		airSteer:             0.5,
		friction:             8.0,
		gravityScale:         1.5,
		maxGroundForce:       70.0,
		maxAirForce:          20.0,
		pogoRestLength:       0.9,
		pogoHertz:            5.0,
		pogoDampingRatio:     0.8,
		pogoCompressionScale: 100.0,
		pogoTensionScale:     100.0,
	}
}

// dynamicMoverCastResult is the ground probe hit of the dynamic mover.
type dynamicMoverCastResult struct {
	point    b2.Vec2
	normal   b2.Vec2
	bodyId   b2.BodyId
	fraction b2.Q
	hit      bool
}

// dynamicMover is the dynamic character mover. The capsule is a rigid body
// with a locked rotation, so it collides and is pushed like everything else
// in the world. A mover joint drives it to the target velocity and a pogo
// joint holds it above the ground so it can climb steps without touching the
// capsule. It mirrors the DynamicMover class.
type dynamicMover struct {
	jumpSpeed  float64
	maxSpeed   float64
	minSpeed   float64
	stopSpeed  float64
	accelerate float64

	// airSteer is the fraction of the ground acceleration available in the air.
	airSteer float64

	// friction is the ground friction in units of 1/time.
	friction float64

	// gravityScale: a character usually wants to fall faster than the rest of
	// the world.
	gravityScale float64

	// maxGroundForce and maxAirForce are the force budget the mover joint has
	// for reaching the target velocity.
	maxGroundForce float64
	maxAirForce    float64

	pogoRestLength   float64
	pogoHertz        float64
	pogoDampingRatio float64

	// Pogo force limits as a multiple of the mover weight.
	pogoCompressionScale float64
	pogoTensionScale     float64

	// minGroundNormalY: surfaces steeper than this are not ground.
	minGroundNormalY float64

	capsule b2.Capsule
	filter  b2.Filter

	worldId      b2.WorldId
	groundId     b2.BodyId
	moverId      b2.BodyId
	moverJointId b2.JointId
	pogoJointId  b2.JointId

	castResult dynamicMoverCastResult

	velocity  b2.Vec2
	onGround  bool
	walkable  bool
	jumping   bool
	jumpTicks int

	// Pogo joint state, carried across the joint being recreated each solve.
	pogoImpulse  b2.Q
	pogoVelocity b2.Q
	pogoLength   b2.Q

	// Ground probe from the last solve, kept for debug drawing.
	pogoOrigin      b2.Vec2
	pogoTranslation b2.Vec2
	pogoFraction    b2.Q
	pogoHit         bool
}

// newDynamicMover returns a mover with the DynamicMover constructor values.
func newDynamicMover() *dynamicMover {
	return &dynamicMover{
		jumpSpeed:            5.0,
		maxSpeed:             6.0,
		minSpeed:             0.1,
		stopSpeed:            3.0,
		accelerate:           20.0,
		airSteer:             0.2,
		friction:             8.0,
		gravityScale:         1.5,
		maxGroundForce:       70.0,
		maxAirForce:          20.0,
		pogoRestLength:       0.9,
		pogoHertz:            5.0,
		pogoDampingRatio:     0.8,
		pogoCompressionScale: 100.0,
		pogoTensionScale:     100.0,
		minGroundNormalY:     0.7,
		capsule:              b2.Capsule{Center1: b2.V2(0, -0.5), Center2: b2.V2(0, 0.5), Radius: b2.F(0.3)},
		filter:               b2.DefaultFilter(),
		pogoFraction:         b2.QOne(),
	}
}

// create places the mover in the world.
func (m *dynamicMover) create(worldId b2.WorldId, def *dynamicMoverDef) {
	m.capsule = def.capsule
	m.filter = def.filter
	m.jumpSpeed = def.jumpSpeed
	m.maxSpeed = def.maxSpeed
	m.minSpeed = def.minSpeed
	m.stopSpeed = def.stopSpeed
	m.accelerate = def.accelerate
	m.airSteer = def.airSteer
	m.friction = def.friction
	m.gravityScale = def.gravityScale
	m.maxGroundForce = def.maxGroundForce
	m.maxAirForce = def.maxAirForce
	m.pogoRestLength = def.pogoRestLength
	m.pogoHertz = def.pogoHertz
	m.pogoDampingRatio = def.pogoDampingRatio
	m.pogoCompressionScale = def.pogoCompressionScale
	m.pogoTensionScale = def.pogoTensionScale
	m.minGroundNormalY = 0.7

	m.worldId = worldId
	m.velocity = b2.Vec2{}
	m.onGround = false
	m.walkable = false
	m.jumping = false
	m.pogoImpulse = b2.QZero()
	m.pogoVelocity = b2.QZero()
	m.pogoLength = b2.QZero()
	m.pogoJointId = b2.JointId{}

	bodyDef := b2.DefaultBodyDef()
	m.groundId = b2.CreateBody(worldId, &bodyDef)

	// Position is the center of the capsule.
	bodyDef.Type = b2.DynamicBody
	bodyDef.Position = def.position
	bodyDef.GravityScale = FromFloat64(m.gravityScale)
	bodyDef.MotionLocks.AngularZ = true
	bodyDef.EnableSleep = false
	bodyDef.Name = "mover"

	m.moverId = b2.CreateBody(worldId, &bodyDef)

	shapeDef := b2.DefaultShapeDef()
	shapeDef.Density = FromFloat64(def.density)

	// The mover joint and the pogo joint handle all the movement, surface
	// friction would fight them.
	shapeDef.Material.Friction = b2.QZero()
	shapeDef.Filter = def.filter
	shapeDef.EnablePreSolveEvents = def.enablePreSolveEvents

	b2.CreateCapsuleShape(m.moverId, &shapeDef, &m.capsule)

	moverDef := b2.DefaultMoverJointDef()
	moverDef.LinearVelocity = m.velocity
	moverDef.MaxVelocityForce = b2.Vec2{X: FromFloat64(m.maxGroundForce)}
	moverDef.Base.BodyIdA = m.groundId
	moverDef.Base.BodyIdB = m.moverId
	moverDef.Base.CollideConnected = true

	m.moverJointId = b2.CreateMoverJoint(worldId, &moverDef)
}

// destroy removes the mover bodies; the joints go with them.
func (m *dynamicMover) destroy() {
	if !m.moverId.IsNull() {
		b2.DestroyBody(m.moverId)
	}

	if !m.groundId.IsNull() {
		b2.DestroyBody(m.groundId)
	}

	m.groundId = b2.BodyId{}
	m.moverId = b2.BodyId{}
	m.moverJointId = b2.JointId{}
	m.pogoJointId = b2.JointId{}
}

// setGravityScale changes the gravity scale of the mover body.
func (m *dynamicMover) setGravityScale(gravityScale float64) {
	m.gravityScale = gravityScale

	if !m.moverId.IsNull() {
		m.moverId.SetGravityScale(FromFloat64(gravityScale))
	}
}

// jump jumps if grounded. It returns false when airborne.
func (m *dynamicMover) jump() bool {
	if !m.onGround || !m.walkable {
		return false
	}

	surfaceVelocity := b2.QZero()
	if m.castResult.bodyId.IsValid() {
		v := m.castResult.bodyId.GetWorldPointVelocity(m.castResult.point)
		surfaceVelocity = v.Y
	}

	mass := m.moverId.GetMass()
	vy := m.moverId.GetLinearVelocity().Y

	// Remove the pogo constraint velocity but add the surface velocity.
	dv := b2.QZero().Max(FromFloat64(m.jumpSpeed).Sub(vy)).Add(surfaceVelocity)
	m.moverId.ApplyLinearImpulseToCenter(b2.Vec2{Y: mass.Mul(dv)}, true)

	// This removes the pogo step down on the next update.
	m.onGround = false
	m.walkable = false
	m.jumping = true
	m.jumpTicks = 0
	return true
}

// update advances the mover. Throttle is the horizontal input on [-1, 1].
//
// Movement follows the Quake ground and air model:
// https://github.com/id-Software/Quake/blob/master/QW/client/pmove.c#L390
//
// onGround controls friction, mover max force, and pogo step-down extension.
// walkable controls air steer. jumping blocks the pogo from pulling down.
func (m *dynamicMover) update(timeStep, throttle b2.Q) {
	zero := b2.QZero()
	one := b2.QOne()
	restLength := FromFloat64(m.pogoRestLength)

	// Reach further while grounded so the spring can find the ground over a step
	stepDownLength := restLength
	rayLength := restLength
	if m.onGround {
		rayLength = restLength.Add(stepDownLength)
	}
	translation := b2.Vec2{Y: rayLength.Neg()}

	position := m.moverId.GetPosition()
	origin := position.Add(m.capsule.Center1)

	queryFilter := b2.QueryFilter{CategoryBits: m.filter.CategoryBits, MaskBits: m.filter.MaskBits}

	m.castResult = dynamicMoverCastResult{}
	m.worldId.CastRay(origin, translation, queryFilter, func(shapeId b2.ShapeId, point, normal b2.Vec2, fraction b2.Q) b2.Q {
		m.castResult.point = point
		m.castResult.normal = normal
		m.castResult.bodyId = shapeId.GetBody()
		m.castResult.fraction = fraction
		m.castResult.hit = true
		return fraction
	})

	m.pogoOrigin = origin
	m.pogoTranslation = translation
	m.pogoFraction = one
	if m.castResult.hit {
		m.pogoFraction = m.castResult.fraction
	}
	m.pogoHit = m.castResult.hit

	// Should jumping end?
	if m.jumping {
		m.jumpTicks++

		// Jump ticks allow time for the jump to leave the ground. Otherwise
		// jumping ends when the character approaches the current ground.
		if m.jumpTicks > 2 && m.castResult.hit && !m.velocity.Dot(m.castResult.normal).Greater(zero) {
			m.jumping = false
		}
	}

	if m.castResult.hit {
		// The cast can still hit while jumping.
		m.onGround = !m.jumping

		// Is the ground too steep to walk?
		m.walkable = !m.castResult.normal.Y.Less(FromFloat64(m.minGroundNormalY))
	} else {
		m.onGround = false
		m.walkable = false
	}

	m.velocity = m.moverId.GetLinearVelocity()

	// Friction
	if m.onGround {
		speed, _ := b2.GetLengthAndNormalize(m.velocity)
		if speed.Less(FromFloat64(m.minSpeed)) {
			m.velocity.X = zero
		} else {
			// Linear damping above stopSpeed and fixed reduction below stopSpeed
			control := speed
			if speed.Less(FromFloat64(m.stopSpeed)) {
				control = FromFloat64(m.stopSpeed)
			}

			// friction has units of 1/time
			drop := control.Mul(FromFloat64(m.friction)).Mul(timeStep)
			newSpeed := zero.Max(speed.Sub(drop))
			m.velocity.X = m.velocity.X.Mul(newSpeed.Div(speed))
		}
	}

	// Mover force
	if m.onGround {
		maxForce := zero
		if m.walkable {
			maxForce = FromFloat64(m.maxGroundForce)
		}
		m.moverJointId.SetMoverMaxVelocityForce(b2.Vec2{X: maxForce})
	} else {
		m.moverJointId.SetMoverMaxVelocityForce(b2.Vec2{X: FromFloat64(m.maxAirForce)})
	}

	maxSpeed := FromFloat64(m.maxSpeed)
	desiredVelocity := b2.Vec2{X: maxSpeed.Mul(throttle)}
	desiredSpeed, desiredDirection := b2.GetLengthAndNormalize(desiredVelocity)

	if desiredSpeed.Greater(maxSpeed) {
		desiredSpeed = maxSpeed
	}

	// Accelerate
	currentSpeed := m.velocity.Dot(desiredDirection)
	addSpeed := desiredSpeed.Sub(currentSpeed)
	if addSpeed.Greater(zero) {
		steer := FromFloat64(m.airSteer)
		if m.walkable {
			steer = one
		}
		accelSpeed := steer.Mul(FromFloat64(m.accelerate)).Mul(maxSpeed).Mul(timeStep)
		if accelSpeed.Greater(addSpeed) {
			accelSpeed = addSpeed
		}

		m.velocity.X = m.velocity.X.Add(accelSpeed.Mul(desiredDirection.X))
	}

	// The pogo joint is rebuilt every solve because it may land on a different body
	if m.pogoJointId.IsValid() {
		m.pogoLength = m.pogoJointId.GetCurrentLength()
		m.pogoImpulse = m.pogoJointId.GetImpulse()
		m.pogoVelocity = m.pogoJointId.GetCurrentSpeed()

		b2.DestroyJoint(m.pogoJointId)
		m.pogoJointId = b2.JointId{}
	}

	if m.castResult.hit {
		moverMass := m.moverId.GetMass()
		moverWeight := FromFloat64(m.gravityScale).Mul(m.worldId.GetGravity().Len()).Mul(moverMass)

		pogoDef := b2.DefaultPogoJointDef()
		pogoDef.Base.LocalFrameA.P = m.castResult.bodyId.GetLocalPoint(m.castResult.point)
		pogoDef.Base.LocalFrameB.P = m.capsule.Center1
		pogoDef.Normal = m.castResult.normal
		pogoDef.Base.BodyIdA = m.castResult.bodyId
		pogoDef.Base.BodyIdB = m.moverId
		pogoDef.Base.CollideConnected = true
		pogoDef.RestLength = restLength
		pogoDef.Hertz = FromFloat64(m.pogoHertz)
		pogoDef.DampingRatio = FromFloat64(m.pogoDampingRatio)
		pogoDef.MaxCompressionForce = FromFloat64(m.pogoCompressionScale).Mul(moverWeight)

		// Warm start from the joint that was just destroyed
		pogoDef.Impulse = m.pogoImpulse
		pogoDef.Velocity = m.pogoVelocity

		if m.jumping {
			// Don't allow the pogo to pull down
			pogoDef.MaxTensionForce = zero
		} else {
			// The pogo can pull down at a multiple of the gravity force.
			pogoDef.MaxTensionForce = FromFloat64(m.pogoTensionScale).Mul(moverWeight)
		}

		m.pogoJointId = b2.CreatePogoJoint(m.worldId, &pogoDef)
	} else {
		m.pogoImpulse = zero
		m.pogoVelocity = zero
	}

	m.moverJointId.SetLinearVelocity(m.velocity)
}
