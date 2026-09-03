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
// continuousContext carries the fast shape and its sweep through the tree
// queries of the continuous stage. It corresponds to b2ContinuousContext
// in src/solver.c.
type continuousContext struct {
	world       *world
	fastBodySim *bodySim
	fastShape   *shape
	centroid1   Vec2
	centroid2   Vec2
	sweep       Sweep
	fraction    Q
}

// queryCallback runs for each proxy the swept box of the fast shape
// touches and shortens the fraction on a time of impact. The method value
// replaces the context pointer of b2ContinuousQueryCallback in
// src/solver.c. See D-014.
func (ctx *continuousContext) queryCallback(_ int, userData uint64) bool {
	zero := fixed.Q32Zero()

	shapeId := int(userData)

	fastShape := ctx.fastShape
	fastBodySim := ctx.fastBodySim

	// Skip same shape
	if shapeId == fastShape.id {
		return true
	}

	w := ctx.world

	s := &w.shapes[shapeId]

	// Skip same body
	if s.bodyId == fastShape.bodyId {
		return true
	}

	// Skip sensors
	if s.sensorIndex != nullIndex {
		return true
	}

	// Skip filtered shapes
	canCollide := shouldShapesCollide(fastShape.filter, s.filter)
	if !canCollide {
		return true
	}

	b := &w.bodies[s.bodyId]

	sim := getBodySim(w, b)
	if b.bodyType != StaticBody && !fastBodySim.isBullet {
		panic("dbox2d: a fast body swept a moving body")
	}

	// Skip bullets
	if sim.isBullet {
		return true
	}

	// Skip filtered bodies
	fastBody := &w.bodies[fastBodySim.bodyId]
	canCollide = shouldBodiesCollide(w, fastBody, b)
	if !canCollide {
		return true
	}

	// Deferred: the custom filter callback of the reference runs here.

	// Prevent pausing on chain segment junctions
	if s.shapeType == ChainSegmentShape {
		transform := sim.transform
		p1 := TransformPoint(transform, s.chainSegment.Segment.Point1)
		p2 := TransformPoint(transform, s.chainSegment.Segment.Point2)
		e := p2.Sub(p1)
		var length Q
		length, e = GetLengthAndNormalize(e)
		if linearSlop.Less(length) {
			c1 := ctx.centroid1
			offset1 := Cross(c1.Sub(p1), e)
			c2 := ctx.centroid2
			offset2 := Cross(c2.Sub(p1), e)

			// todo this should use the min extent of the fast shape, not the body
			allowedFraction := fixed.Q32FromRatio(1, 4)
			if offset1.Less(zero) || offset1.Sub(offset2).Less(allowedFraction.Mul(fastBodySim.minExtent)) {
				// Minimal clipping
				return true
			}
		}
	}

	var input TOIInput
	input.ProxyA = makeShapeDistanceProxy(s)
	input.ProxyB = makeShapeDistanceProxy(fastShape)
	input.SweepA = makeSweep(sim)
	input.SweepB = ctx.sweep
	input.MaxFraction = ctx.fraction

	hitFraction := ctx.fraction

	didHit := false
	output := TimeOfImpact(&input)
	if zero.Less(output.Fraction) && output.Fraction.Less(ctx.fraction) {
		hitFraction = output.Fraction
		didHit = true
	} else if output.Fraction.Eq(zero) {
		// fallback to TOI of a small circle around the fast shape centroid
		centroid := getShapeCentroid(fastShape)
		extent := computeShapeExtent(fastShape, centroid)
		radius := fixed.Q32FromRatio(1, 4).Mul(extent.minExtent)
		centroidPoint := [1]Vec2{centroid}
		input.ProxyB = MakeProxy(centroidPoint[:], radius)
		output = TimeOfImpact(&input)
		if zero.Less(output.Fraction) && output.Fraction.Less(ctx.fraction) {
			hitFraction = output.Fraction
			didHit = true
		}
	}

	// Deferred: the pre-solve callback of the reference runs here on a
	// temporary manifold.

	if didHit {
		ctx.fraction = hitFraction
	}

	return true
}

// solveContinuous sweeps one fast body against the static tree, or
// against every tree for a bullet, and moves the body back to its first
// time of impact. It corresponds to b2SolveContinuous in src/solver.c.
func solveContinuous(w *world, bodySimIndex int) {
	awake := &w.solverSets[awakeSet]
	fastBodySim := &awake.bodySims[bodySimIndex]
	if !fastBodySim.isFast {
		panic("dbox2d: the continuous stage got a slow body")
	}

	sweep := makeSweep(fastBodySim)

	var xf1 Transform
	xf1.Q = sweep.Q1
	xf1.P = sweep.C1.Sub(RotateVector(sweep.Q1, sweep.LocalCenter))

	var xf2 Transform
	xf2.Q = sweep.Q2
	xf2.P = sweep.C2.Sub(RotateVector(sweep.Q2, sweep.LocalCenter))

	staticTree := &w.broadPhase.trees[StaticBody]
	kinematicTree := &w.broadPhase.trees[KinematicBody]
	dynamicTree := &w.broadPhase.trees[DynamicBody]
	fastBody := &w.bodies[fastBodySim.bodyId]

	var ctx continuousContext
	ctx.world = w
	ctx.sweep = sweep
	ctx.fastBodySim = fastBodySim
	ctx.fraction = fixed.Q32One()

	isBullet := fastBodySim.isBullet

	shapeId := fastBody.headShapeId
	for shapeId != nullIndex {
		fastShape := &w.shapes[shapeId]
		shapeId = fastShape.nextShapeId

		ctx.fastShape = fastShape
		ctx.centroid1 = TransformPoint(xf1, fastShape.localCentroid)
		ctx.centroid2 = TransformPoint(xf2, fastShape.localCentroid)

		box1 := fastShape.aabb
		box2 := computeShapeAABB(fastShape, xf2)
		box := AABBUnion(box1, box2)

		// Store this to avoid double computation in the case there is no impact event
		fastShape.aabb = box2

		// No continuous collision for sensors (but still need the updated bounds)
		if fastShape.sensorIndex != nullIndex {
			continue
		}

		staticTree.query(box, DefaultMaskBits, ctx.queryCallback)

		if isBullet {
			kinematicTree.query(box, DefaultMaskBits, ctx.queryCallback)
			dynamicTree.query(box, DefaultMaskBits, ctx.queryCallback)
		}
	}

	if ctx.fraction.Less(fixed.Q32One()) {
		// Handle time of impact event
		q := NLerp(sweep.Q1, sweep.Q2, ctx.fraction)
		c := Lerp(sweep.C1, sweep.C2, ctx.fraction)
		origin := c.Sub(RotateVector(q, sweep.LocalCenter))

		// Advance body
		transform := Transform{P: origin, Q: q}
		fastBodySim.transform = transform
		fastBodySim.center = c
		fastBodySim.rotation0 = q
		fastBodySim.center0 = c

		// Update body move event
		w.bodyMoveEvents[bodySimIndex].Transform = transform

		// Prepare AABBs for broad-phase.
		// Even though a body is fast, it may not move much. So the
		// AABB may not need enlargement.

		shapeId = fastBody.headShapeId
		for shapeId != nullIndex {
			s := &w.shapes[shapeId]

			// Must recompute aabb at the interpolated transform
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

				s.enlargedAABB = true
				fastBodySim.enlargeAABB = true
			}

			shapeId = s.nextShapeId
		}
	} else {
		// No time of impact event

		// Advance body
		fastBodySim.rotation0 = fastBodySim.transform.Q
		fastBodySim.center0 = fastBodySim.center

		// Prepare AABBs for broad-phase
		shapeId = fastBody.headShapeId
		for shapeId != nullIndex {
			s := &w.shapes[shapeId]

			// shape->aabb is still valid from above

			if !AABBContains(s.fatAABB, s.aabb) {
				fatAABB := AABB{
					LowerBound: Vec2{X: s.aabb.LowerBound.X.Sub(aabbMargin), Y: s.aabb.LowerBound.Y.Sub(aabbMargin)},
					UpperBound: Vec2{X: s.aabb.UpperBound.X.Add(aabbMargin), Y: s.aabb.UpperBound.Y.Add(aabbMargin)},
				}
				s.fatAABB = fatAABB

				s.enlargedAABB = true
				fastBodySim.enlargeAABB = true
			}

			shapeId = s.nextShapeId
		}
	}
}

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

	enableContinuous := w.enableContinuous

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

			if b.bodyType == DynamicBody && enableContinuous && half.Mul(sim.minExtent).Less(maxVelocity.Mul(timeStep)) {
				// This flag is only retained for debug draw
				sim.isFast = true

				// Store in fast array for the continuous collision stage
				// This is deterministic because the order of TOI sweeps doesn't matter
				if sim.isBullet {
					context.bulletBodies[context.bulletBodyCount] = simIndex
					context.bulletBodyCount++
				} else {
					solveContinuous(w, simIndex)
				}
			} else {
				// Body is safe to advance
				sim.center0 = sim.center
				sim.rotation0 = sim.transform.Q
			}
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
		isFast := sim.isFast
		shapeId := b.headShapeId
		for shapeId != nullIndex {
			s := &w.shapes[shapeId]

			if isFast {
				// For fast non-bullet bodies the AABB has already been updated in solveContinuous
				// For fast bullet bodies the AABB will be updated at a later stage

				// Add to enlarged shapes regardless of AABB changes.
				// Bit-set to keep the move array sorted
				enlargedSimBitSet.setBit(simIndex)
			} else {
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
		// Prepare buffers for bullets
		context.bulletBodyCount = 0
		context.bulletBodies, context.bulletBodyMem = arenaSlice[int](&w.arena, awakeBodyCount, "bullet bodies")

		graph := context.graph
		colors := &graph.colors

		context.sims = awake.bodySims
		context.states = awake.bodyStates

		w.bodyMoveEvents = resizeMoveEvents(w.bodyMoveEvents, awakeBodyCount)

		// Deferred: the contact pointers, the joint pointers and the stage
		// blocks serve the parallel executor.

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
		// the active colors in order. In each stage the joints go before
		// the contacts, as the joint stage of the reference finishes
		// before the contact stage starts.
		prepareJoints(context, overflowIndex)
		for i := range overflowIndex {
			prepareJoints(context, i)
		}
		prepareContacts(context, overflowIndex)
		for i := range overflowIndex {
			prepareContacts(context, i)
		}

		for range context.subStepCount {
			integrateVelocitiesTask(0, awakeBodyCount, context)

			warmStartJoints(context, overflowIndex)
			warmStartContacts(context, overflowIndex)
			for i := range overflowIndex {
				warmStartJoints(context, i)
				warmStartContacts(context, i)
			}

			useBias := true
			solveJoints(context, overflowIndex, useBias)
			solveContacts(context, overflowIndex, useBias)
			for i := range overflowIndex {
				solveJoints(context, i, useBias)
				solveContacts(context, i, useBias)
			}

			integratePositionsTask(0, awakeBodyCount, context)

			useBias = false
			solveJoints(context, overflowIndex, useBias)
			solveContacts(context, overflowIndex, useBias)
			for i := range overflowIndex {
				solveJoints(context, i, useBias)
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

				shapeId := b.headShapeId
				if bodySim.isBullet && bodySim.isFast {
					// Fast bullet bodies don't have their final AABB yet
					for shapeId != nullIndex {
						s := &w.shapes[shapeId]

						// Shape is fast. It's aabb will be enlarged in continuous collision.
						// Update the move array here for determinism because bullets are processed
						// below in non-deterministic order.
						broadPhase.bufferMove(s.proxyKey)

						shapeId = s.nextShapeId
					}
				} else {
					for shapeId != nullIndex {
						s := &w.shapes[shapeId]

						// The AABB may not have been enlarged, despite the body being flagged as enlarged.
						// For example, a body with multiple shapes may have not have all shapes enlarged.
						// A fast body may have been flagged as enlarged despite having no shapes enlarged.
						if s.enlargedAABB {
							broadPhase.enlargeProxy(s.proxyKey, s.fatAABB)
							s.enlargedAABB = false
						}

						shapeId = s.nextShapeId
					}
				}

				// Clear the smallest set bit
				word = word & (word - 1)
			}
		}
	}

	// Continuous collision of the bullet bodies. The reference sweeps them
	// in parallel and enlarges their proxies serially; the port runs both
	// on one worker, in the buffer order.
	if context.bulletBodyCount > 0 {
		// Fast bullet bodies
		// Note: a bullet body may be moving slow
		for i := range context.bulletBodyCount {
			solveContinuous(w, context.bulletBodies[i])
		}

		// Serially enlarge broad-phase proxies for bullet shapes
		broadPhase := &w.broadPhase
		dynamicTree := &broadPhase.trees[DynamicBody]

		bodySimArray := awake.bodySims
		bulletBodySimIndices := context.bulletBodies[:context.bulletBodyCount]

		for _, simIndex := range bulletBodySimIndices {
			bulletBodySim := &bodySimArray[simIndex]
			if !bulletBodySim.enlargeAABB {
				continue
			}

			// clear flag
			bulletBodySim.enlargeAABB = false

			bulletBody := &w.bodies[bulletBodySim.bodyId]

			shapeId := bulletBody.headShapeId
			for shapeId != nullIndex {
				s := &w.shapes[shapeId]
				if !s.enlargedAABB {
					shapeId = s.nextShapeId
					continue
				}

				// clear flag
				s.enlargedAABB = false

				proxyKey := s.proxyKey
				proxyId := proxyIdOf(proxyKey)
				if proxyTypeOf(proxyKey) != DynamicBody {
					panic("dbox2d: a bullet shape is not in the dynamic tree")
				}

				// all fast bullet shapes should already be in the move buffer
				if !broadPhase.moveSet.containsKey(uint64(proxyKey) + 1) {
					panic("dbox2d: a bullet shape is not in the move buffer")
				}

				dynamicTree.enlargeProxy(proxyId, s.fatAABB)

				shapeId = s.nextShapeId
			}
		}
	}

	// Need to free this even if no bullets got processed.
	w.arena.freeItem(context.bulletBodyMem)
	context.bulletBodies = nil
	context.bulletBodyMem = nil
	context.bulletBodyCount = 0

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
