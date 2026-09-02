package dbox2d

import "github.com/dhannyell/fixed"

// softness holds the soft constraint coefficients of one sub-step. It
// corresponds to b2Softness in src/solver.h.
type softness struct {
	biasRate     Q
	massScale    Q
	impulseScale Q
}

// makeSoft derives the coefficients from the frequency (hertz), the damping
// ratio (zeta) and the sub-step time (h). A zero frequency means a rigid
// constraint. It corresponds to b2MakeSoft in src/solver.h.
func makeSoft(hertz, zeta, h Q) softness {
	if hertz.Eq(fixed.Q32Zero()) {
		return softness{}
	}

	// D-004: the reference multiplies by two pi; one turn is the same value.
	omega := tau.Mul(hertz)
	a1 := zeta.Add(zeta).Add(h.Mul(omega))
	a2 := h.Mul(omega).Mul(a1)

	// bias = w / (2 * z + hw)
	// massScale = hw * (2 * z + hw) / (1 + hw * (2 * z + hw))
	// impulseScale = 1 / (1 + hw * (2 * z + hw))
	// In all cases: massScale + impulseScale == 1
	// D-006: the reference multiplies by the reciprocal of (1 + a2). Each
	// coefficient divides instead.
	one := fixed.Q32One()
	return softness{
		biasRate:     omega.Div(a1),
		massScale:    a2.Div(one.Add(a2)),
		impulseScale: one.Div(one.Add(a2)),
	}
}

// integrateVelocitiesTask applies forces, gravity and damping to the awake
// bodies. It corresponds to b2IntegrateVelocitiesTask in src/solver.c.
func integrateVelocitiesTask(startIndex, endIndex int, context *stepContext) {
	states := context.states
	sims := context.sims

	gravity := context.world.gravity
	h := context.h
	maxLinearSpeed := context.maxLinearVelocity
	maxAngularSpeed := maxRotation.Mul(context.invDt)
	maxLinearSpeedSquared := maxLinearSpeed.Mul(maxLinearSpeed)
	maxAngularSpeedSquared := maxAngularSpeed.Mul(maxAngularSpeed)

	zero := fixed.Q32Zero()
	one := fixed.Q32One()
	for i := startIndex; i < endIndex; i++ {
		sim := &sims[i]
		state := &states[i]

		v := state.linearVelocity
		omega := state.angularVelocity

		// Apply damping.
		// Differential equation: dv/dt + c * v = 0
		// Solution: v(t) = v0 * exp(-c * t)
		// Pade approximation:
		// v2 = v1 * 1 / (1 + c * dt)
		// D-006: the reference multiplies by the factor. Each use divides by
		// the denominator instead.
		linearDamping := one.Add(h.Mul(sim.linearDamping))
		angularDamping := one.Add(h.Mul(sim.angularDamping))

		// Gravity scale will be zero for kinematic bodies
		gravityScale := zero
		if zero.Less(sim.invMass) {
			gravityScale = sim.gravityScale
		}

		// lvd = h * im * f + h * g
		linearVelocityDelta := sim.force.Mul(h.Mul(sim.invMass)).Add(gravity.Mul(h.Mul(gravityScale)))
		// The state stores turns per second and the torque produces radians
		// per second, so the delta divides by one turn.
		angularVelocityDelta := h.Mul(sim.invInertia).Mul(sim.torque).Div(tau)

		v = Vec2{
			X: linearVelocityDelta.X.Add(v.X.Div(linearDamping)),
			Y: linearVelocityDelta.Y.Add(v.Y.Div(linearDamping)),
		}
		omega = angularVelocityDelta.Add(omega.Div(angularDamping))

		// Clamp to max linear speed
		if maxLinearSpeedSquared.Less(v.Dot(v)) {
			ratio := maxLinearSpeed.Div(v.Len())
			v = v.Mul(ratio)
			sim.isSpeedCapped = true
		}

		// Clamp to max angular speed
		if maxAngularSpeedSquared.Less(omega.Mul(omega)) && !sim.allowFastRotation {
			ratio := maxAngularSpeed.Div(omega.Abs())
			omega = omega.Mul(ratio)
			sim.isSpeedCapped = true
		}

		state.linearVelocity = v
		state.angularVelocity = omega
	}
}

// integratePositionsTask advances the position deltas of the awake bodies by
// the sub-step time. It corresponds to b2IntegratePositionsTask in
// src/solver.c.
func integratePositionsTask(startIndex, endIndex int, context *stepContext) {
	states := context.states
	h := context.h

	if endIndex < startIndex {
		panic("dbox2d: the task range is inverted")
	}

	for i := startIndex; i < endIndex; i++ {
		state := &states[i]
		state.deltaRotation = IntegrateRotation(state.deltaRotation, h.Mul(state.angularVelocity))
		state.deltaPosition = MulAdd(state.deltaPosition, h, state.linearVelocity)
	}
}

// finalizeBodiesTask writes the advanced deltas into the transforms, tracks
// sleep time and refreshes the shape bounds. It corresponds to
// b2FinalizeBodiesTask in src/solver.c.
func finalizeBodiesTask(startIndex, endIndex int, context *stepContext) {
	w := context.world
	enableSleep := w.enableSleep
	states := context.states
	sims := context.sims
	timeStep := context.dt
	invTimeStep := context.invDt

	// Deferred: the move events, the enlarged and awake island bit sets and
	// the task contexts of the reference.

	if endIndex < startIndex {
		panic("dbox2d: the task range is inverted")
	}

	zero := fixed.Q32Zero()
	half := fixed.Q32Half()
	for simIndex := startIndex; simIndex < endIndex; simIndex++ {
		state := &states[simIndex]
		sim := &sims[simIndex]

		v := state.linearVelocity
		omega := state.angularVelocity

		if !IsValidVec2(v) {
			panic("dbox2d: the linear velocity is not valid")
		}
		if !IsValidQ(omega) {
			panic("dbox2d: the angular velocity is not valid")
		}

		sim.center = sim.center.Add(state.deltaPosition)
		sim.transform.Q = NormalizeRot(MulRot(state.deltaRotation, sim.transform.Q))

		// Use the velocity of the farthest point on the body to account for
		// rotation. The angular velocity is in turns per second and the arc
		// speed needs radians per second, so it scales by one turn.
		maxVelocity := v.Len().Add(tau.Mul(omega.Abs()).Mul(sim.maxExtent))

		// Sleep needs to observe position correction as well as true velocity.
		maxDeltaPosition := state.deltaPosition.Len().Add(state.deltaRotation.Sin.Abs().Mul(sim.maxExtent))

		// Position correction is not as important for sleep as true velocity.
		positionSleepFactor := half

		sleepVelocity := maxVelocity.Max(positionSleepFactor.Mul(invTimeStep).Mul(maxDeltaPosition))

		// reset state deltas
		state.deltaPosition = Vec2Zero()
		state.deltaRotation = fixed.RotIdentity()

		sim.transform.P = sim.center.Sub(RotateVector(sim.transform.Q, sim.localCenter))

		b := &w.bodies[sim.bodyId]
		b.bodyMoveIndex = simIndex
		// Deferred: the move event of the reference fills here.

		// reset applied force and torque
		sim.force = Vec2Zero()
		sim.torque = zero

		b.isSpeedCapped = sim.isSpeedCapped
		sim.isSpeedCapped = false

		sim.isFast = false

		if !enableSleep || !b.enableSleep || b.sleepThreshold.Less(sleepVelocity) {
			// Body is not sleepy
			b.sleepTime = zero

			// Continuous collision and its fast-body test are deferred. Until
			// they land, finalize every body discretely so its previous
			// transform and bounds do not remain stale.
			sim.center0 = sim.center
			sim.rotation0 = sim.transform.Q
		} else {
			// Body is safe to advance and is falling asleep
			sim.center0 = sim.center
			sim.rotation0 = sim.transform.Q
			b.sleepTime = b.sleepTime.Add(timeStep)
		}

		// Deferred: the island sleep bookkeeping of the reference runs here.

		// Update shapes AABBs
		transform := sim.transform
		shapeId := b.headShapeId
		for shapeId != nullIndex {
			s := &w.shapes[shapeId]

			aabb := computeShapeAABB(s, transform)
			aabb.LowerBound.X = aabb.LowerBound.X.Sub(speculativeDistance)
			aabb.LowerBound.Y = aabb.LowerBound.Y.Sub(speculativeDistance)
			aabb.UpperBound.X = aabb.UpperBound.X.Add(speculativeDistance)
			aabb.UpperBound.Y = aabb.UpperBound.Y.Add(speculativeDistance)
			s.aabb = aabb

			if !AABBContains(s.fatAABB, aabb) {
				fatAABB := AABB{
					LowerBound: Vec2{X: aabb.LowerBound.X.Sub(aabbMargin), Y: aabb.LowerBound.Y.Sub(aabbMargin)},
					UpperBound: Vec2{X: aabb.UpperBound.X.Add(aabbMargin), Y: aabb.UpperBound.Y.Add(aabbMargin)},
				}
				s.fatAABB = fatAABB
				// Deferred: the enlargedAABB flag and the enlarged bit set feed
				// the broad-phase.
			}

			shapeId = s.nextShapeId
		}
	}
}

// solve runs the sub-step loop over the awake set, then finalizes the
// bodies. The reference splits the same order into parallel stages; the port
// runs them on one worker. It corresponds to b2Solve and b2SolverTask in
// src/solver.c.
func solve(w *world, context *stepContext) {
	w.stepIndex += 1

	// Deferred: the island merge of the reference runs here.

	set := &w.solverSets[awakeSet]
	awakeBodyCount := len(set.bodySims)
	if awakeBodyCount == 0 {
		return
	}

	context.sims = set.bodySims
	context.states = set.bodyStates

	// Deferred: the constraint graph, the joint and contact preparation and
	// the stage blocks of the reference.

	for range context.subStepCount {
		integrateVelocitiesTask(0, awakeBodyCount, context)
		// Deferred: the warm start and the constraint solve run here.
		integratePositionsTask(0, awakeBodyCount, context)
		// Deferred: the relax iteration runs here.
	}

	// Deferred: the restitution pass and the impulse store run here.

	finalizeBodiesTask(0, awakeBodyCount, context)

	// Deferred: the continuous collision stage and the island sleep of the
	// reference run here.
}
