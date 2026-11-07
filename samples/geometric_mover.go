// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/geometric_mover.h and samples/geometric_mover.cpp of Box2D v3.2.0

package samples

import (
	"github.com/dhannyell/dbox2d"
)

// moverPlaneCapacity is GeometricMover::m_planeCapacity.
const moverPlaneCapacity = 8

// geometricMoverDef configures a geometricMover. It mirrors GeometricMoverDef.
type geometricMoverDef struct {
	position b2.Vec2
	capsule  b2.Capsule
	filter   b2.QueryFilter

	jumpSpeed            float64
	maxSpeed             float64
	minSpeed             float64
	stopSpeed            float64
	accelerate           float64
	airSteer             float64
	friction             float64
	gravity              float64
	pogoRestLength       float64
	pogoHertz            float64
	pogoDampingRatio     float64
	pogoCompressionScale float64
	pogoTensionScale     float64
}

// defaultGeometricMoverDef returns the default values of GeometricMoverDef.
func defaultGeometricMoverDef() geometricMoverDef {
	return geometricMoverDef{
		capsule:              b2.Capsule{Center1: b2.V2(0, -0.5), Center2: b2.V2(0, 0.5), Radius: b2.F(0.3)},
		filter:               b2.DefaultQueryFilter(),
		jumpSpeed:            10.0,
		maxSpeed:             6.0,
		minSpeed:             0.1,
		stopSpeed:            3.0,
		accelerate:           20.0,
		airSteer:             0.2,
		friction:             8.0,
		gravity:              30.0,
		pogoRestLength:       0.9,
		pogoHertz:            5.0,
		pogoDampingRatio:     0.8,
		pogoCompressionScale: 100.0,
		pogoTensionScale:     100.0,
	}
}

// moverShapeUserData is the optional shape user data read by the geometric
// mover when it gathers collision planes (MoverShapeUserData). Shapes without
// user data are treated as rigid and fully clipping.
type moverShapeUserData struct {
	// maxPush is the distance limit on the push this surface may apply to the
	// mover. Huge is rigid. Use a small value for surfaces that must not shove
	// the mover, such as an elevator that would otherwise push it through the
	// floor.
	maxPush b2.Q

	// clipVelocity false keeps the velocity along this surface, which is what
	// soft collision needs.
	clipVelocity bool
}

// moverPlaneExtra is the data kept alongside each collision plane so the
// mover can push rigid bodies (PlaneExtra).
type moverPlaneExtra struct {
	point   b2.Vec2
	shapeId b2.ShapeId
}

// geometricMover is the geometric character mover. The capsule is moved by
// casting and by solving collision planes, so the game keeps full control of
// the motion. A pogo spring holds the capsule above the ground so it can move
// up steps. It mirrors the GeometricMover class.
type geometricMover struct {
	jumpSpeed  float64
	maxSpeed   float64
	minSpeed   float64
	stopSpeed  float64
	accelerate float64

	// airSteer is the fraction of the ground acceleration available in the air.
	airSteer float64

	// friction is the ground friction in units of 1/time.
	friction float64

	// gravity: the mover is not simulated so it needs its own gravity.
	gravity float64

	pogoRestLength   float64
	pogoHertz        float64
	pogoDampingRatio float64

	// pogoGroundForce is the downward force applied to whatever the pogo stick
	// is resting on.
	pogoGroundForce float64

	// minGroundNormalY: surfaces steeper than this are not ground.
	minGroundNormalY float64

	// pushBodies pushes touched rigid bodies out of the way by solving a
	// normal constraint against each collision plane. Without this the mover
	// slides along a crate instead of shoving it.
	pushBodies bool

	// capsule is relative to the mover position.
	capsule b2.Capsule

	filter b2.QueryFilter

	worldId b2.WorldId

	// position is the center of the capsule.
	position b2.Vec2

	velocity     b2.Vec2
	pogoVelocity b2.Q
	onGround     bool

	// Planes from the last solve, kept for debug drawing.
	planes          [moverPlaneCapacity]b2.CollisionPlane
	planeExtras     [moverPlaneCapacity]moverPlaneExtra
	planeCount      int
	totalIterations int

	pogoOrigin      b2.Vec2
	pogoTranslation b2.Vec2
	pogoFraction    b2.Q
	pogoHit         bool
}

// newGeometricMover returns a mover with the GeometricMover constructor values.
func newGeometricMover() *geometricMover {
	return &geometricMover{
		jumpSpeed:        10.0,
		maxSpeed:         6.0,
		minSpeed:         0.1,
		stopSpeed:        3.0,
		accelerate:       20.0,
		airSteer:         0.2,
		friction:         8.0,
		gravity:          30.0,
		pogoRestLength:   0.9,
		pogoHertz:        5.0,
		pogoDampingRatio: 0.8,
		pogoGroundForce:  50.0,
		minGroundNormalY: 0.7,
		pushBodies:       true,
		capsule:          b2.Capsule{Center1: b2.V2(0, -0.5), Center2: b2.V2(0, 0.5), Radius: b2.F(0.3)},
		filter:           b2.DefaultQueryFilter(),
		pogoFraction:     b2.QOne(),
	}
}

// create places the mover. The capsule and tuning may be changed at any time.
func (m *geometricMover) create(worldId b2.WorldId, def *geometricMoverDef) {
	m.capsule = def.capsule
	m.filter = def.filter
	m.jumpSpeed = def.jumpSpeed
	m.maxSpeed = def.maxSpeed
	m.minSpeed = def.minSpeed
	m.stopSpeed = def.stopSpeed
	m.accelerate = def.accelerate
	m.airSteer = def.airSteer
	m.friction = def.friction
	m.gravity = def.gravity
	m.pogoRestLength = def.pogoRestLength
	m.pogoHertz = def.pogoHertz
	m.pogoDampingRatio = def.pogoDampingRatio
	m.minGroundNormalY = 0.7

	m.worldId = worldId
	m.position = def.position
	m.velocity = b2.Vec2{}
	m.pogoVelocity = b2.QZero()
	m.onGround = false
	m.planeCount = 0
	m.totalIterations = 0
	m.pogoFraction = b2.QOne()
	m.pogoHit = false
}

// jump jumps if grounded. It returns false when airborne.
func (m *geometricMover) jump() bool {
	if !m.onGround {
		return false
	}

	m.velocity.Y = FromFloat64(m.jumpSpeed)
	m.onGround = false
	return true
}

// planeResult collects the collision planes of the mover, with the push
// tuning of the shape it hits.
func (m *geometricMover) planeResult(shapeId b2.ShapeId, planeResult *b2.PlaneResult) bool {
	maxPush := b2.Huge
	clipVelocity := true
	if userData, ok := shapeId.GetUserData().(*moverShapeUserData); ok && userData != nil {
		maxPush = userData.maxPush
		clipVelocity = userData.clipVelocity
	}

	if m.planeCount < moverPlaneCapacity {
		m.planes[m.planeCount] = b2.CollisionPlane{
			Plane:        planeResult.Plane,
			PushLimit:    maxPush,
			Push:         b2.QZero(),
			ClipVelocity: clipVelocity,
		}

		// The plane point is relative to the origin passed to the collide call.
		m.planeExtras[m.planeCount] = moverPlaneExtra{
			point:   b2.ToVec2(b2.OffsetPos(b2.ToPos(m.position), planeResult.Point)),
			shapeId: shapeId,
		}
		m.planeCount++
	}

	return true
}

// pushBodiesAlongPlanes solves a normal constraint between the mover and each
// touched rigid body. This is the same velocity constraint the solver uses for
// a contact, except the mover is treated as having infinite mass, so all of
// the impulse lands on the body. It corresponds to GeometricMover::PushBodies.
func (m *geometricMover) pushBodiesAlongPlanes() {
	zero := b2.QZero()
	one := b2.QOne()
	for i := range m.planeCount {
		bodyId := m.planeExtras[i].shapeId.GetBody()
		if bodyId.GetType() != b2.DynamicBody {
			continue
		}

		// Plane normals point at the mover
		point := m.planeExtras[i].point
		normal := b2.Neg(m.planes[i].Plane.Normal)

		invMassA := zero

		massB := bodyId.GetMass()
		invMassB := zero
		if massB.Greater(zero) {
			invMassB = one.Div(massB)
		}

		inertiaB := bodyId.GetRotationalInertia()
		invIB := zero
		if inertiaB.Greater(zero) {
			invIB = one.Div(inertiaB)
		}

		centerB := bodyId.GetWorldCenter()
		rB := b2.SubPos(b2.ToPos(point), centerB)

		rnB := b2.Cross(rB, normal)
		kNormal := invMassA.Add(invMassB).Add(invIB.Mul(rnB).Mul(rnB))
		normalMass := zero
		if kNormal.Greater(zero) {
			normalMass = one.Div(kNormal)
		}

		vB := bodyId.GetLinearVelocity()
		omegaB := bodyId.GetAngularVelocity()
		vrB := vB.Add(b2.CrossSV(omegaB, rB))
		vn := vrB.Sub(m.velocity).Dot(normal)

		// Push only, a separating body is left alone
		impulse := zero.Max(normalMass.Neg().Mul(vn))

		P := normal.Mul(impulse)
		m.velocity = b2.MulSub(m.velocity, invMassA, P)

		bodyId.ApplyLinearImpulse(P, b2.ToPos(point), true)
	}
}

// update advances the mover. Throttle is the horizontal input on [-1, 1].
//
// Movement follows the Quake ground and air model:
// https://github.com/id-Software/Quake/blob/master/QW/client/pmove.c#L390
func (m *geometricMover) update(timeStep, throttle b2.Q) {
	zero := b2.QZero()
	one := b2.QOne()
	maxSpeed := FromFloat64(m.maxSpeed)

	// Friction
	speed, _ := b2.GetLengthAndNormalize(m.velocity)
	if speed.Less(FromFloat64(m.minSpeed)) {
		m.velocity.X = zero
		m.velocity.Y = zero
	} else if m.onGround {
		// Linear damping above stopSpeed and fixed reduction below stopSpeed
		control := speed
		if speed.Less(FromFloat64(m.stopSpeed)) {
			control = FromFloat64(m.stopSpeed)
		}

		// friction has units of 1/time
		drop := control.Mul(FromFloat64(m.friction)).Mul(timeStep)
		newSpeed := zero.Max(speed.Sub(drop))
		m.velocity = m.velocity.Mul(newSpeed.Div(speed))
	}

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
		if m.onGround {
			steer = one
		}
		accelSpeed := steer.Mul(FromFloat64(m.accelerate)).Mul(maxSpeed).Mul(timeStep)
		if accelSpeed.Greater(addSpeed) {
			accelSpeed = addSpeed
		}

		m.velocity = m.velocity.Add(desiredDirection.Mul(accelSpeed))
	}

	// The probe reaches past the rest length so the spring can be stretched
	restLength := FromFloat64(m.pogoRestLength)
	rayLength := restLength
	if m.onGround {
		rayLength = b2.F(2).Mul(restLength)
	}
	translation := b2.Vec2{Y: rayLength.Neg()}
	origin := m.position.Add(m.capsule.Center1)

	var castResult moverCastResult
	m.worldId.CastRay(b2.ToPos(origin), translation, m.filter, func(shapeId b2.ShapeId, point b2.Pos, normal b2.Vec2, fraction b2.Q) b2.Q {
		castResult.point = b2.ToVec2(point)
		castResult.normal = normal
		castResult.bodyId = shapeId.GetBody()
		castResult.fraction = fraction
		castResult.hit = true
		return fraction
	})

	m.pogoOrigin = origin
	m.pogoTranslation = translation
	m.pogoFraction = one
	if castResult.hit {
		m.pogoFraction = castResult.fraction
	}
	m.pogoHit = castResult.hit

	// Avoid snapping to ground if still going up
	haveGround := castResult.hit && castResult.normal.Y.Greater(FromFloat64(m.minGroundNormalY))

	if !m.onGround {
		m.onGround = haveGround && !m.velocity.Y.Greater(b2.QFromRatio(1, 100))
	} else {
		m.onGround = haveGround
	}

	if !m.onGround {
		m.velocity.Y = m.velocity.Y.Sub(FromFloat64(m.gravity).Mul(timeStep))
		m.pogoVelocity = zero
	} else {
		m.velocity.Y = zero

		pogoCurrentLength := castResult.fraction.Mul(rayLength)

		offset := pogoCurrentLength.Sub(restLength)
		m.pogoVelocity = b2.SpringDamper(FromFloat64(m.pogoHertz), FromFloat64(m.pogoDampingRatio), offset, m.pogoVelocity, timeStep)

		castResult.bodyId.ApplyForce(b2.Vec2{Y: FromFloat64(-m.pogoGroundForce)}, b2.ToPos(castResult.point), true)
	}

	target := m.position.Add(m.velocity.Mul(timeStep)).Add(b2.Vec2{Y: timeStep.Mul(m.pogoVelocity)})

	m.totalIterations = 0
	tolerance := b2.QFromRatio(1, 100)

	// Don't cast against other movers.
	castFilter := b2.QueryFilter{
		CategoryBits: m.filter.CategoryBits,
		MaskBits:     m.filter.MaskBits &^ m.filter.CategoryBits,
	}

	for range 5 {
		m.planeCount = 0

		m.worldId.CollideMover(b2.ToPos(m.position), &m.capsule, m.filter, m.planeResult)
		result := b2.SolvePlanes(target.Sub(m.position), m.planes[:m.planeCount])

		m.totalIterations += result.IterationCount

		fraction := m.worldId.CastMover(b2.ToPos(m.position), &m.capsule, result.Translation, castFilter)

		delta := result.Translation.Mul(fraction)
		m.position = m.position.Add(delta)

		if delta.LenSq().Less(tolerance.Mul(tolerance)) {
			break
		}
	}

	if m.pushBodies {
		m.pushBodiesAlongPlanes()
	}

	// Stop pushing into surfaces so speed can't accumulate against a wall
	m.velocity = b2.ClipVector(m.velocity, m.planes[:m.planeCount])
}
