package dbox2d

import (
	"math/bits"

	"github.com/dhannyell/fixed"
)

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

	if endIndex > len(w.bodyMoveEvents) {
		panic("dbox2d: the move events are not sized for the awake set")
	}
	moveEvents := w.bodyMoveEvents

	if endIndex < startIndex {
		panic("dbox2d: the task range is inverted")
	}

	taskContext := &w.taskContext
	enlargedSimBitSet := &taskContext.enlargedSimBitSet
	awakeIslandBitSet := &taskContext.awakeIslandBitSet

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

		// cache miss here, however I need the shape list below
		b := &w.bodies[sim.bodyId]
		b.bodyMoveIndex = simIndex
		moveEvents[simIndex].Transform = sim.transform
		moveEvents[simIndex].BodyId = BodyId{index1: int32(sim.bodyId) + 1, world0: w.worldId, generation: b.generation}
		moveEvents[simIndex].UserData = b.userData
		moveEvents[simIndex].FellAsleep = false

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

		// Any single body in an island can keep it awake
		isl := &w.islands[b.islandId]
		if b.sleepTime.Less(timeToSleep) {
			// keep island awake
			islandIndex := isl.localIndex
			awakeIslandBitSet.setBit(islandIndex)
		} else if isl.constraintRemoveCount > 0 {
			// body wants to sleep but its island needs splitting first
			if taskContext.splitSleepTime.Less(b.sleepTime) {
				// pick the sleepiest candidate
				taskContext.splitIslandId = b.islandId
				taskContext.splitSleepTime = b.sleepTime
			}
		}

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

			if s.enlargedAABB {
				panic("dbox2d: the shape is still marked enlarged")
			}

			if !AABBContains(s.fatAABB, aabb) {
				fatAABB := AABB{
					LowerBound: Vec2{X: aabb.LowerBound.X.Sub(aabbMargin), Y: aabb.LowerBound.Y.Sub(aabbMargin)},
					UpperBound: Vec2{X: aabb.UpperBound.X.Add(aabbMargin), Y: aabb.UpperBound.Y.Add(aabbMargin)},
				}
				s.fatAABB = fatAABB

				s.enlargedAABB = true

				// Bit-set to keep the move array sorted
				enlargedSimBitSet.setBit(simIndex)
			}

			shapeId = s.nextShapeId
		}
	}
}

// solve merges the islands, runs the constraint stages over the awake set,
// finalizes the bodies and puts the sleepy islands to sleep. The reference
// splits the same order into parallel stages over the graph colors; the
// port runs them on one worker, so only the overflow color solves. It
// corresponds to b2Solve and b2SolverTask in src/solver.c.
func solve(w *world, context *stepContext) {
	w.stepIndex += 1

	// Merge islands
	mergeAwakeIslands(w)

	// Are there any awake bodies? This scenario should not be important for profiling.
	awake := &w.solverSets[awakeSet]
	awakeBodyCount := len(awake.bodySims)
	if awakeBodyCount == 0 {
		// Nothing to simulate. The tree rebuild already ran in collide.
		return
	}

	// Solve constraints using graph coloring
	{
		graph := context.graph
		colors := &graph.colors

		context.sims = awake.bodySims
		context.states = awake.bodyStates

		w.bodyMoveEvents = resizeMoveEvents(w.bodyMoveEvents, awakeBodyCount)

		// Deferred: the contact pointers, the joints and the stage blocks
		// serve the parallel executor.

		// One contiguous scratch serves every color, as the SIMD scratch of
		// the reference does.
		contactCount := 0
		for i := range graphColorCount {
			contactCount += len(colors[i].contactSims)
		}
		contactConstraints, constraintMem := arenaSlice[contactConstraint](&w.arena, contactCount, "contact constraint")
		contactBase := 0
		for i := range graphColorCount {
			count := len(colors[i].contactSims)
			colors[i].contactConstraints = contactConstraints[contactBase : contactBase+count : contactBase+count]
			contactBase += count
		}

		// The reference runs the overflow color first in each stage, then
		// the active colors in order. Joint stages run beside each contact
		// stage and wait for the joints.
		prepareContacts(context, overflowIndex)
		for i := range overflowIndex {
			prepareContacts(context, i)
		}

		for range context.subStepCount {
			integrateVelocitiesTask(0, awakeBodyCount, context)

			warmStartContacts(context, overflowIndex)
			for i := range overflowIndex {
				warmStartContacts(context, i)
			}

			useBias := true
			solveContacts(context, overflowIndex, useBias)
			for i := range overflowIndex {
				solveContacts(context, i, useBias)
			}

			integratePositionsTask(0, awakeBodyCount, context)

			useBias = false
			solveContacts(context, overflowIndex, useBias)
			for i := range overflowIndex {
				solveContacts(context, i, useBias)
			}
		}

		applyRestitution(context, overflowIndex)
		for i := range overflowIndex {
			applyRestitution(context, i)
		}

		storeImpulses(context, overflowIndex)
		for i := range overflowIndex {
			storeImpulses(context, i)
		}

		// Split an awake island. This modifies:
		// - stack allocator
		// - world island array and solver set
		// - island indices on bodies, contacts, and joints
		// The reference runs the split beside the constraint solve. The
		// split cannot run beside the body finalize.
		if w.splitIslandId != nullIndex {
			splitIsland(w, w.splitIslandId)
		}
		w.splitIslandId = nullIndex

		// Prepare the enlarged body and island bit sets used in body finalization.
		awakeIslandCount := len(awake.islandSims)
		taskContext := &w.taskContext
		setBitCountAndClear(&taskContext.enlargedSimBitSet, awakeBodyCount)
		setBitCountAndClear(&taskContext.awakeIslandBitSet, awakeIslandCount)
		taskContext.splitIslandId = nullIndex
		taskContext.splitSleepTime = fixed.Q32Zero()

		// Finalize bodies. Must happen after the constraint solver and after island splitting.
		finalizeBodiesTask(0, awakeBodyCount, context)

		for i := range graphColorCount {
			colors[i].contactConstraints = nil
		}
		w.arena.freeItem(constraintMem)
	}

	// Report hit events
	{
		if len(w.contactHitEvents) != 0 {
			panic("dbox2d: the hit events are not clear")
		}

		threshold := w.hitEventThreshold
		colors := &w.constraintGraph.colors
		zero := fixed.Q32Zero()
		for i := range graphColorCount {
			color := &colors[i]
			contactSims := color.contactSims
			for j := range contactSims {
				cs := &contactSims[j]
				if cs.simFlags&simEnableHitEvent == 0 {
					continue
				}

				event := ContactHitEvent{}
				event.ApproachSpeed = threshold

				hit := false
				pointCount := cs.manifold.PointCount
				for k := range pointCount {
					mp := &cs.manifold.Points[k]
					approachSpeed := mp.NormalVelocity.Neg()

					// Need to check total impulse because the point may be speculative and not colliding
					if event.ApproachSpeed.Less(approachSpeed) && zero.Less(mp.TotalNormalImpulse) {
						event.ApproachSpeed = approachSpeed
						event.Point = mp.Point
						hit = true
					}
				}

				if hit {
					event.Normal = cs.manifold.Normal
					event.ShapeIdA = shapeIdOf(w, &w.shapes[cs.shapeIdA])
					event.ShapeIdB = shapeIdOf(w, &w.shapes[cs.shapeIdB])
					w.contactHitEvents = append(w.contactHitEvents, event)
				}
			}
		}
	}

	// Refit the broad-phase. The tree rebuild already ran in collide.
	{
		enlargedBodyBitSet := &w.taskContext.enlargedSimBitSet

		// Enlarge broad-phase proxies and build move array
		// Apply shape AABB changes to broad-phase. This also create the move array which must be
		// in deterministic order. Sim bodies are tracked because the number of shape ids can be huge.
		broadPhase := &w.broadPhase
		bodySimArray := awake.bodySims

		for k := range enlargedBodyBitSet.bits {
			word := enlargedBodyBitSet.bits[k]
			for word != 0 {
				ctz := bits.TrailingZeros64(word)
				bodySimIndex := 64*k + ctz

				bodySim := &bodySimArray[bodySimIndex]

				b := &w.bodies[bodySim.bodyId]

				// Deferred: a fast bullet body buffers its shapes here and
				// enlarges them in the continuous stage.

				shapeId := b.headShapeId
				for shapeId != nullIndex {
					s := &w.shapes[shapeId]

					// The AABB may not have been enlarged, despite the body being flagged as enlarged.
					// For example, a body with multiple shapes may have not have all shapes enlarged.
					if s.enlargedAABB {
						broadPhase.enlargeProxy(s.proxyKey, s.fatAABB)
						s.enlargedAABB = false
					}

					shapeId = s.nextShapeId
				}

				// Clear the smallest set bit
				word = word & (word - 1)
			}
		}
	}

	// Deferred: the continuous collision stage of the reference runs here.

	// Island sleeping
	// This must be done last because putting islands to sleep invalidates the enlarged body bits.
	if w.enableSleep {
		// Collect split island candidate for the next time step. No need to split if sleeping is disabled.
		if w.splitIslandId != nullIndex {
			panic("dbox2d: the split candidate is not clear")
		}
		taskContext := &w.taskContext
		if taskContext.splitIslandId != nullIndex {
			if !fixed.Q32Zero().Less(taskContext.splitSleepTime) {
				panic("dbox2d: the split candidate has no sleep time")
			}
			w.splitIslandId = taskContext.splitIslandId
		}

		awakeIslandBitSet := &taskContext.awakeIslandBitSet

		// Need to process in reverse because this moves islands to sleeping solver sets.
		islands := awake.islandSims
		for islandIndex := len(islands) - 1; islandIndex >= 0; islandIndex-- {
			if awakeIslandBitSet.getBit(islandIndex) {
				// this island is still awake
				continue
			}

			islandId := islands[islandIndex].islandId
			trySleepIsland(w, islandId)
		}
	}
}

// resizeMoveEvents sets the length of the move event array to the awake
// body count and keeps its capacity, so a step allocates only on growth.
func resizeMoveEvents(events []BodyMoveEvent, count int) []BodyMoveEvent {
	if count <= cap(events) {
		return events[:count]
	}
	grown := make([]BodyMoveEvent, count, max(count, 2*cap(events)))
	return grown
}
