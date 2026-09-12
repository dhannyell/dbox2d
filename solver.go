package dbox2d

import (
	"math"
	"math/bits"
	"slices"
	"sync/atomic"
	"time"
)

type solverStageType int

const (
	stagePrepareJoints solverStageType = iota
	stagePrepareContacts
	stageIntegrateVelocities
	stageWarmStart
	stageSolve
	stageIntegratePositions
	stageRelax
	stageRestitution
	stageStoreImpulses
)

type solverBlockType int

const (
	bodyBlock solverBlockType = iota
	jointBlock
	contactBlock
	graphJointBlock
	graphContactBlock
)

type solverBlock struct {
	startIndex int
	count      int
	blockType  int16
	syncIndex  atomic.Int32
}

type solverStage struct {
	stageType       solverStageType
	blocks          []solverBlock
	colorIndex      int
	completionCount atomic.Int32
}

type workerContext struct {
	context     *stepContext
	workerIndex int
}

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
	if hertz.Eq(QZero()) {
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
	// D-006: a3 is the reciprocal of (1 + a2).
	one := QOne()
	a3 := makeRecip(one.Add(a2))
	return softness{
		biasRate:     omega.Div(a1),
		massScale:    a3.scale(a2),
		impulseScale: a3.scale(one),
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
	// D-004: MaxRotation is in turns; one turn scales it to radians exactly.
	maxAngularSpeed := maxRotation.Mul(tau).Mul(context.invDt)
	maxLinearSpeedSquared := maxLinearSpeed.Mul(maxLinearSpeed)
	maxAngularSpeedSquared := maxAngularSpeed.Mul(maxAngularSpeed)

	zero := QZero()
	one := QOne()
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
		// D-006: each factor is the reciprocal of its denominator.
		linearDamping := makeRecip(one.Add(h.Mul(sim.linearDamping)))
		angularDamping := makeRecip(one.Add(h.Mul(sim.angularDamping)))

		// Gravity scale will be zero for kinematic bodies
		gravityScale := zero
		if zero.Less(sim.invMass) {
			gravityScale = sim.gravityScale
		}

		// lvd = h * im * f + h * g
		linearVelocityDelta := sim.force.Mul(h.Mul(sim.invMass)).Add(gravity.Mul(h.Mul(gravityScale)))
		angularVelocityDelta := h.Mul(sim.invInertia).Mul(sim.torque)

		v = Vec2{
			X: linearVelocityDelta.X.Add(linearDamping.scale(v.X)),
			Y: linearVelocityDelta.Y.Add(linearDamping.scale(v.Y)),
		}
		omega = angularVelocityDelta.Add(angularDamping.scale(omega))

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
		state.deltaRotation = integrateRotation(state.deltaRotation, h.Mul(state.angularVelocity))
		state.deltaPosition = MulAdd(state.deltaPosition, h, state.linearVelocity)
	}
}

func executeBlock(stage *solverStage, context *stepContext, block *solverBlock) {
	startIndex := block.startIndex
	endIndex := startIndex + block.count
	blockType := solverBlockType(block.blockType)

	switch stage.stageType {
	case stagePrepareJoints:
		prepareJointsTask(startIndex, endIndex, context)
	case stagePrepareContacts:
		runContactStageBlock(stage, context, startIndex, endIndex)
	case stageIntegrateVelocities:
		integrateVelocitiesTask(startIndex, endIndex, context)
	case stageWarmStart:
		switch blockType {
		case graphContactBlock:
			runGraphContactBlock(stage, context, startIndex, endIndex)
		case graphJointBlock:
			warmStartJointsTask(startIndex, endIndex, context, stage.colorIndex)
		}
	case stageSolve:
		switch blockType {
		case graphContactBlock:
			runGraphContactBlock(stage, context, startIndex, endIndex)
		case graphJointBlock:
			solveJointsTask(startIndex, endIndex, context, stage.colorIndex, true)
		}
	case stageIntegratePositions:
		integratePositionsTask(startIndex, endIndex, context)
	case stageRelax:
		switch blockType {
		case graphContactBlock:
			runGraphContactBlock(stage, context, startIndex, endIndex)
		case graphJointBlock:
			solveJointsTask(startIndex, endIndex, context, stage.colorIndex, false)
		}
	case stageRestitution:
		if blockType == graphContactBlock {
			runGraphContactBlock(stage, context, startIndex, endIndex)
		}
	case stageStoreImpulses:
		runContactStageBlock(stage, context, startIndex, endIndex)
	}
}

func getWorkerStartIndex(workerIndex, blockCount, workerCount int) int {
	if blockCount <= workerCount {
		if workerIndex < blockCount {
			return workerIndex
		}
		return nullIndex
	}

	blocksPerWorker := blockCount / workerCount
	remainder := blockCount - blocksPerWorker*workerCount
	return blocksPerWorker*workerIndex + min(remainder, workerIndex)
}

func executeStage(stage *solverStage, context *stepContext, previousSyncIndex, syncIndex, workerIndex int) {
	completedCount := 0
	blocks := stage.blocks
	blockCount := len(blocks)
	startIndex := getWorkerStartIndex(workerIndex, blockCount, context.workerCount)
	if startIndex == nullIndex {
		return
	}

	blockIndex := startIndex
	for blocks[blockIndex].syncIndex.CompareAndSwap(int32(previousSyncIndex), int32(syncIndex)) {
		executeBlock(stage, context, &blocks[blockIndex])
		completedCount += 1
		blockIndex += 1
		if blockIndex >= blockCount {
			blockIndex = 0
		}
	}

	blockIndex = startIndex - 1
	for {
		if blockIndex < 0 {
			blockIndex = blockCount - 1
		}
		if !blocks[blockIndex].syncIndex.CompareAndSwap(int32(previousSyncIndex), int32(syncIndex)) {
			break
		}

		executeBlock(stage, context, &blocks[blockIndex])
		completedCount += 1
		blockIndex -= 1
	}

	stage.completionCount.Add(int32(completedCount))
}

func executeMainStage(stage *solverStage, context *stepContext, syncBits uint32) {
	if context.workerCount == 1 {
		for i := range stage.blocks {
			executeBlock(stage, context, &stage.blocks[i])
		}
		return
	}

	blockCount := len(stage.blocks)
	if blockCount == 0 {
		return
	}
	if blockCount == 1 {
		executeBlock(stage, context, &stage.blocks[0])
		return
	}

	context.atomicSyncBits.Store(syncBits)
	syncIndex := int(syncBits >> 16)
	previousSyncIndex := syncIndex - 1
	executeStage(stage, context, previousSyncIndex, syncIndex, 0)

	var s spinner
	for stage.completionCount.Load() != int32(blockCount) {
		s.spin()
	}
	stage.completionCount.Store(0)
}

func solverTask(workerIndex int, context *stepContext) {
	worker := workerContext{context: context, workerIndex: workerIndex}
	if worker.workerIndex == 0 {
		solverMainTask(worker.context)
		return
	}
	solverWorkerTask(worker)
}

// splitIslandSide is the side task of the solver script. It corresponds
// to b2SplitIslandTask in src/solver.c.
func splitIslandSide(context *stepContext) {
	w := context.world
	splitStart := time.Now()
	splitIsland(w, w.splitIslandId)
	w.profile.SplitIslands += millisecondsSince(splitStart)
}

func solverMainTask(context *stepContext) {
	activeColorCount := context.activeColorCount
	stages := context.stages

	ticks := time.Now()
	bodySyncIndex := 1
	stageIndex := 0

	jointSyncIndex := 1
	syncBits := uint32(jointSyncIndex)<<16 | uint32(stageIndex)
	executeMainStage(&stages[stageIndex], context, syncBits)
	stageIndex += 1

	contactSyncIndex := 1
	syncBits = uint32(contactSyncIndex)<<16 | uint32(stageIndex)
	executeMainStage(&stages[stageIndex], context, syncBits)
	stageIndex += 1
	contactSyncIndex += 1

	graphSyncIndex := 1
	prepareOverflowJoints(context)
	prepareOverflowContacts(context)
	prepareContacts32(context)
	context.world.profile.PrepareConstraints += millisecondsAndReset(&ticks)

	for range context.subStepCount {
		iterStageIndex := stageIndex

		syncBits = uint32(bodySyncIndex)<<16 | uint32(iterStageIndex)
		executeMainStage(&stages[iterStageIndex], context, syncBits)
		iterStageIndex += 1
		bodySyncIndex += 1
		context.world.profile.IntegrateVelocities += millisecondsAndReset(&ticks)

		warmStartOverflowJoints(context)
		warmStartOverflowContacts(context)
		for range activeColorCount {
			syncBits = uint32(graphSyncIndex)<<16 | uint32(iterStageIndex)
			executeMainStage(&stages[iterStageIndex], context, syncBits)
			iterStageIndex += 1
		}
		graphSyncIndex += 1
		context.world.profile.WarmStart += millisecondsAndReset(&ticks)

		useBias := true
		solveOverflowJoints(context, useBias)
		solveOverflowContacts(context, useBias)
		for range activeColorCount {
			syncBits = uint32(graphSyncIndex)<<16 | uint32(iterStageIndex)
			executeMainStage(&stages[iterStageIndex], context, syncBits)
			iterStageIndex += 1
		}
		graphSyncIndex += 1
		context.world.profile.SolveImpulses += millisecondsAndReset(&ticks)

		syncBits = uint32(bodySyncIndex)<<16 | uint32(iterStageIndex)
		executeMainStage(&stages[iterStageIndex], context, syncBits)
		iterStageIndex += 1
		bodySyncIndex += 1
		context.world.profile.IntegratePositions += millisecondsAndReset(&ticks)

		useBias = false
		solveOverflowJoints(context, useBias)
		solveOverflowContacts(context, useBias)
		for range activeColorCount {
			syncBits = uint32(graphSyncIndex)<<16 | uint32(iterStageIndex)
			executeMainStage(&stages[iterStageIndex], context, syncBits)
			iterStageIndex += 1
		}
		graphSyncIndex += 1
		context.world.profile.RelaxImpulses += millisecondsAndReset(&ticks)
	}

	stageIndex += 2 + 3*activeColorCount
	applyOverflowRestitution(context)
	iterStageIndex := stageIndex
	for range activeColorCount {
		syncBits = uint32(graphSyncIndex)<<16 | uint32(iterStageIndex)
		executeMainStage(&stages[iterStageIndex], context, syncBits)
		iterStageIndex += 1
	}
	stageIndex += activeColorCount
	context.world.profile.ApplyRestitution += millisecondsAndReset(&ticks)

	storeOverflowImpulses(context)
	storeImpulses32(context)
	syncBits = uint32(contactSyncIndex)<<16 | uint32(stageIndex)
	executeMainStage(&stages[stageIndex], context, syncBits)
	context.world.profile.StoreImpulses += millisecondsAndReset(&ticks)

	if context.workerCount > 1 {
		context.atomicSyncBits.Store(math.MaxUint32)
	}
	if stageIndex+1 != len(stages) {
		panic("dbox2d: the solver stage script is incomplete")
	}
}

func solverWorkerTask(worker workerContext) {
	context := worker.context
	workerIndex := worker.workerIndex
	stages := context.stages
	lastSyncBits := uint32(0)
	for {
		syncBits := context.atomicSyncBits.Load()
		var s spinner
		for syncBits == lastSyncBits {
			s.spin()
			syncBits = context.atomicSyncBits.Load()
		}

		if syncBits == math.MaxUint32 {
			break
		}

		stageIndex := int(syncBits & 0xFFFF)
		syncIndex := int(syncBits >> 16)
		executeStage(&stages[stageIndex], context, syncIndex-1, syncIndex, workerIndex)
		lastSyncBits = syncBits
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
	zero := QZero()

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

	// Custom user filtering
	if w.customFilterFcn != nil {
		idA := ShapeId{index1: int32(s.id) + 1, world0: w.worldId, generation: s.generation}
		idB := ShapeId{index1: int32(fastShape.id) + 1, world0: w.worldId, generation: fastShape.generation}
		if !w.customFilterFcn(idA, idB) {
			return true
		}
	}

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
			allowedFraction := oneQuarter
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
		radius := oneQuarter.Mul(extent.minExtent)
		centroidPoint := [1]Vec2{centroid}
		input.ProxyB = MakeProxy(centroidPoint[:], radius)
		output = TimeOfImpact(&input)
		if zero.Less(output.Fraction) && output.Fraction.Less(ctx.fraction) {
			hitFraction = output.Fraction
			didHit = true
		}
	}

	if didHit && (s.enablePreSolveEvents || fastShape.enablePreSolveEvents) && w.preSolveFcn != nil {
		// Pre-solve is expensive: it needs a temporary manifold. The user
		// may edit it, but only the real manifold of the discrete solver
		// matters, so the edit here has no effect.
		transformA := GetSweepTransform(&input.SweepA, hitFraction)
		transformB := GetSweepTransform(&input.SweepB, hitFraction)
		manifold := computeManifold(s, transformA, fastShape, transformB)
		idA := ShapeId{index1: int32(s.id) + 1, world0: w.worldId, generation: s.generation}
		idB := ShapeId{index1: int32(fastShape.id) + 1, world0: w.worldId, generation: fastShape.generation}
		didHit = w.preSolveFcn(idA, idB, &manifold)
	}

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
	ctx.fraction = QOne()

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

	if ctx.fraction.Less(QOne()) {
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

func finalizeBodiesTask(startIndex, endIndex, workerIndex int, context *stepContext) {
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

	taskContext := &w.taskContexts[workerIndex]
	enlargedSimBitSet := &taskContext.enlargedSimBitSet
	awakeIslandBitSet := &taskContext.awakeIslandBitSet
	taskContext.bulletBodies = context.bulletBodies[startIndex:endIndex:endIndex]
	taskContext.bulletBodyCount = 0

	enableContinuous := w.enableContinuous

	zero := QZero()
	half := QHalf()
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
		// rotation.
		maxVelocity := v.Len().Add(omega.Abs().Mul(sim.maxExtent))

		// Sleep needs to observe position correction as well as true velocity.
		maxDeltaPosition := state.deltaPosition.Len().Add(state.deltaRotation.Sin.Abs().Mul(sim.maxExtent))

		// Position correction is not as important for sleep as true velocity.
		positionSleepFactor := half

		sleepVelocity := maxVelocity.Max(positionSleepFactor.Mul(invTimeStep).Mul(maxDeltaPosition))

		// reset state deltas
		state.deltaPosition = Vec2Zero()
		state.deltaRotation = RotIdentity()

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
					taskContext.bulletBodies[taskContext.bulletBodyCount] = simIndex
					taskContext.bulletBodyCount++
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

// bulletBodyTask sweeps a range of bullet bodies. It corresponds to
// b2BulletBodyTask in src/solver.c.
func bulletBodyTask(startIndex, endIndex, _ int, context *stepContext) {
	w := context.world
	for i := startIndex; i < endIndex; i++ {
		solveContinuous(w, context.bulletBodies[i])
	}
}

func setSolverStage(stage *solverStage, stageType solverStageType, blocks []solverBlock, colorIndex int) {
	stage.stageType = stageType
	stage.blocks = blocks
	stage.colorIndex = colorIndex
	stage.completionCount.Store(0)
}

// partitionContacts splits each color into the contacts the lane grid fits
// and the contacts it solves in Q32. The pointer list keeps the active
// fitting contacts first, then the overflow fitting contacts, then the Q32
// contacts of every color. It returns the active fitting count.
func partitionContacts(w *world, context *stepContext, colors *[graphColorCount]graphColor) int {
	var fitCounts [graphColorCount]int
	fitCount := 0
	count32 := 0
	for i := range graphColorCount {
		for j := range colors[i].contactSims {
			if contactFitsLane(&colors[i].contactSims[j]) {
				fitCounts[i]++
			}
		}
		fitCount += fitCounts[i]
		count32 += len(colors[i].contactSims) - fitCounts[i]
	}

	total := fitCount + count32
	w.contactPointers = slices.Grow(w.contactPointers[:0], total)[:total]
	pointers := w.contactPointers
	fitBase := 0
	base32 := fitCount
	for i := range graphColorCount {
		color := &colors[i]
		fitStart, start32 := fitBase, base32
		for j := range color.contactSims {
			cs := &color.contactSims[j]
			if contactFitsLane(cs) {
				pointers[fitBase] = cs
				fitBase++
			} else {
				pointers[base32] = cs
				base32++
			}
		}
		if fitBase != fitStart+fitCounts[i] {
			panic("dbox2d: the contact partition is inconsistent")
		}
		color.contacts = pointers[fitStart:fitBase:fitBase]
		color.contacts32 = pointers[start32:base32:base32]
	}

	activeFitCount := fitCount - fitCounts[overflowIndex]
	context.contacts = pointers[:activeFitCount:activeFitCount]
	w.contactCount32 = count32
	return activeFitCount
}

func buildSolverStages(w *world, context *stepContext, awakeBodyCount int) {
	const blocksPerWorker = 4
	maxBlockCount := blocksPerWorker * context.workerCount

	bodyBlockSize := 32
	bodyBlockCount := 0
	if awakeBodyCount > bodyBlockSize*maxBlockCount {
		bodyBlockSize = awakeBodyCount / maxBlockCount
		bodyBlockCount = maxBlockCount
	} else {
		bodyBlockCount = ((awakeBodyCount - 1) >> 5) + 1
	}

	var colorContactCounts [graphColorCount]int
	var colorContactBlockSizes [graphColorCount]int
	var colorContactBlockCounts [graphColorCount]int
	var colorJointCounts [graphColorCount]int
	var colorJointBlockSizes [graphColorCount]int
	var colorJointBlockCounts [graphColorCount]int
	graphBlockCount := 0

	for c := range context.activeColorCount {
		colorIndex := context.activeColorIndices[c]
		color := &context.graph.colors[colorIndex]
		colorContactCount := colorContactUnits(color)
		colorJointCount := len(color.jointSims)

		colorContactCounts[c] = colorContactCount
		if colorContactCount > blocksPerWorker*maxBlockCount {
			colorContactBlockSizes[c] = colorContactCount / maxBlockCount
			colorContactBlockCounts[c] = maxBlockCount
		} else if colorContactCount > 0 {
			colorContactBlockSizes[c] = blocksPerWorker
			colorContactBlockCounts[c] = ((colorContactCount - 1) >> 2) + 1
		}

		colorJointCounts[c] = colorJointCount
		if colorJointCount > blocksPerWorker*maxBlockCount {
			colorJointBlockSizes[c] = colorJointCount / maxBlockCount
			colorJointBlockCounts[c] = maxBlockCount
		} else if colorJointCount > 0 {
			colorJointBlockSizes[c] = blocksPerWorker
			colorJointBlockCounts[c] = ((colorJointCount - 1) >> 2) + 1
		}

		graphBlockCount += colorJointBlockCounts[c] + colorContactBlockCounts[c]
	}

	contactCount := contactStageCount(context)
	contactBlockSize := blocksPerWorker
	contactBlockCount := 0
	if contactCount > 0 {
		contactBlockCount = ((contactCount - 1) >> 2) + 1
	}
	if contactCount > contactBlockSize*maxBlockCount {
		contactBlockSize = contactCount / maxBlockCount
		contactBlockCount = maxBlockCount
	}

	jointCount := len(context.joints)
	jointBlockSize := blocksPerWorker
	jointBlockCount := 0
	if jointCount > 0 {
		jointBlockCount = ((jointCount - 1) >> 2) + 1
	}
	if jointCount > jointBlockSize*maxBlockCount {
		jointBlockSize = jointCount / maxBlockCount
		jointBlockCount = maxBlockCount
	}

	stageCount := 5 + 4*context.activeColorCount
	w.solverStages = slices.Grow(w.solverStages[:0], stageCount)[:stageCount]
	w.bodyBlocks = slices.Grow(w.bodyBlocks[:0], bodyBlockCount)[:bodyBlockCount]
	w.jointBlocks = slices.Grow(w.jointBlocks[:0], jointBlockCount)[:jointBlockCount]
	w.contactBlocks = slices.Grow(w.contactBlocks[:0], contactBlockCount)[:contactBlockCount]
	w.graphBlocks = slices.Grow(w.graphBlocks[:0], graphBlockCount)[:graphBlockCount]

	for i := range bodyBlockCount {
		block := &w.bodyBlocks[i]
		block.startIndex = i * bodyBlockSize
		block.count = bodyBlockSize
		block.blockType = int16(bodyBlock)
		block.syncIndex.Store(0)
	}
	w.bodyBlocks[bodyBlockCount-1].count = awakeBodyCount - (bodyBlockCount-1)*bodyBlockSize

	for i := range jointBlockCount {
		block := &w.jointBlocks[i]
		block.startIndex = i * jointBlockSize
		block.count = jointBlockSize
		block.blockType = int16(jointBlock)
		block.syncIndex.Store(0)
	}
	if jointBlockCount > 0 {
		w.jointBlocks[jointBlockCount-1].count = jointCount - (jointBlockCount-1)*jointBlockSize
	}

	for i := range contactBlockCount {
		block := &w.contactBlocks[i]
		block.startIndex = i * contactBlockSize
		block.count = contactBlockSize
		block.blockType = int16(contactBlock)
		block.syncIndex.Store(0)
	}
	if contactBlockCount > 0 {
		w.contactBlocks[contactBlockCount-1].count = contactCount - (contactBlockCount-1)*contactBlockSize
	}

	var graphColorBlocks [graphColorCount][]solverBlock
	graphBlockBase := 0
	for c := range context.activeColorCount {
		colorBlockBase := graphBlockBase
		colorJointBlockCount := colorJointBlockCounts[c]
		colorJointBlockSize := colorJointBlockSizes[c]
		for j := range colorJointBlockCount {
			block := &w.graphBlocks[graphBlockBase+j]
			block.startIndex = j * colorJointBlockSize
			block.count = colorJointBlockSize
			block.blockType = int16(graphJointBlock)
			block.syncIndex.Store(0)
		}
		if colorJointBlockCount > 0 {
			lastBlock := &w.graphBlocks[graphBlockBase+colorJointBlockCount-1]
			lastBlock.count = colorJointCounts[c] - (colorJointBlockCount-1)*colorJointBlockSize
			graphBlockBase += colorJointBlockCount
		}

		colorContactBlockCount := colorContactBlockCounts[c]
		colorContactBlockSize := colorContactBlockSizes[c]
		for j := range colorContactBlockCount {
			block := &w.graphBlocks[graphBlockBase+j]
			block.startIndex = j * colorContactBlockSize
			block.count = colorContactBlockSize
			block.blockType = int16(graphContactBlock)
			block.syncIndex.Store(0)
		}
		if colorContactBlockCount > 0 {
			lastBlock := &w.graphBlocks[graphBlockBase+colorContactBlockCount-1]
			lastBlock.count = colorContactCounts[c] - (colorContactBlockCount-1)*colorContactBlockSize
			graphBlockBase += colorContactBlockCount
		}

		graphColorBlocks[c] = w.graphBlocks[colorBlockBase:graphBlockBase]
	}
	if graphBlockBase != graphBlockCount {
		panic("dbox2d: the graph block table is incomplete")
	}

	stageIndex := 0
	setSolverStage(&w.solverStages[stageIndex], stagePrepareJoints, w.jointBlocks, nullIndex)
	stageIndex += 1
	setSolverStage(&w.solverStages[stageIndex], stagePrepareContacts, w.contactBlocks, nullIndex)
	stageIndex += 1
	setSolverStage(&w.solverStages[stageIndex], stageIntegrateVelocities, w.bodyBlocks, nullIndex)
	stageIndex += 1
	for c := range context.activeColorCount {
		setSolverStage(&w.solverStages[stageIndex], stageWarmStart, graphColorBlocks[c], context.activeColorIndices[c])
		stageIndex += 1
	}
	for c := range context.activeColorCount {
		setSolverStage(&w.solverStages[stageIndex], stageSolve, graphColorBlocks[c], context.activeColorIndices[c])
		stageIndex += 1
	}
	setSolverStage(&w.solverStages[stageIndex], stageIntegratePositions, w.bodyBlocks, nullIndex)
	stageIndex += 1
	for c := range context.activeColorCount {
		setSolverStage(&w.solverStages[stageIndex], stageRelax, graphColorBlocks[c], context.activeColorIndices[c])
		stageIndex += 1
	}
	for c := range context.activeColorCount {
		setSolverStage(&w.solverStages[stageIndex], stageRestitution, graphColorBlocks[c], context.activeColorIndices[c])
		stageIndex += 1
	}
	setSolverStage(&w.solverStages[stageIndex], stageStoreImpulses, w.contactBlocks, nullIndex)
	stageIndex += 1
	if stageIndex != stageCount {
		panic("dbox2d: the solver stage table is incomplete")
	}

	context.stages = w.solverStages
	if context.workerCount > 1 {
		context.atomicSyncBits.Store(0)
	}
}

// solve merges the islands, runs the constraint stages over the awake set,
// finalizes the bodies and puts the sleepy islands to sleep. It corresponds
// to b2Solve and b2SolverTask in src/solver.c.
func solve(w *world, context *stepContext) {
	w.stepIndex += 1

	// Merge islands
	mergeIslandsStart := time.Now()
	mergeAwakeIslands(w)
	w.profile.MergeIslands = millisecondsSince(mergeIslandsStart)

	// Are there any awake bodies? This scenario should not be important for profiling.
	awake := &w.solverSets[awakeSet]
	awakeBodyCount := len(awake.bodySims)
	if awakeBodyCount == 0 {
		// Nothing to simulate, however the tree rebuild must be finished.
		w.treeTask.wait()
		return
	}

	// Solve constraints using graph coloring
	{
		prepareStagesStart := time.Now()

		// Prepare buffers for bullets
		context.bulletBodyCount = 0
		context.bulletBodies, context.bulletBodyMem = arenaSlice[int](&w.arena, awakeBodyCount, "bullet bodies")

		graph := context.graph
		colors := &graph.colors

		context.sims = awake.bodySims
		context.states = awake.bodyStates

		w.bodyMoveEvents = resizeMoveEvents(w.bodyMoveEvents, awakeBodyCount)

		// One contiguous scratch serves every color, as the SIMD scratch of
		// the reference does.
		activeJointCount := 0
		for i := range overflowIndex {
			colorContactCount := len(colors[i].contactSims)
			colorJointCount := len(colors[i].jointSims)
			activeJointCount += colorJointCount
			if colorContactCount+colorJointCount > 0 {
				context.activeColorIndices[context.activeColorCount] = i
				context.activeColorCount += 1
			}
		}

		w.jointPointers = slices.Grow(w.jointPointers[:0], activeJointCount)[:activeJointCount]
		context.joints = w.jointPointers
		context.workerCount = w.executor.activeWorkerCount()

		jointBase := 0
		for i := range overflowIndex {
			color := &colors[i]
			colorJointCount := len(color.jointSims)
			for j := range colorJointCount {
				context.joints[jointBase+j] = &color.jointSims[j]
			}
			jointBase += colorJointCount
		}

		activeContactCount := partitionContacts(w, context, colors)
		w.wide.allocateContactConstraints(w, context, colors, overflowIndex, activeContactCount)
		allocateContactConstraints32(w, context, colors)
		buildSolverStages(w, context, awakeBodyCount)

		w.profile.PrepareStages = millisecondsSince(prepareStagesStart)

		// The constraint stages run as one persistent-pool job per worker.
		solveConstraintsStart := time.Now()
		var sideFn func(*stepContext)
		if w.splitIslandId != nullIndex {
			sideFn = splitIslandSide
			w.taskCount++
		}
		// The split writes the islands and their links, which the script
		// never reads; the body finalize must wait for it.
		w.taskCount++
		w.executor.runContextWithSide(solverTask, sideFn, context)
		w.splitIslandId = nullIndex

		w.profile.SolveConstraints = millisecondsSince(solveConstraintsStart)

		transformsStart := time.Now()

		// Prepare the enlarged body and island bit sets used in body finalization.
		awakeIslandCount := len(awake.islandSims)
		for i := range w.workerCount {
			taskContext := &w.taskContexts[i]
			setBitCountAndClear(&taskContext.enlargedSimBitSet, awakeBodyCount)
			setBitCountAndClear(&taskContext.awakeIslandBitSet, awakeIslandCount)
			taskContext.splitIslandId = nullIndex
			taskContext.splitSleepTime = QZero()
			taskContext.bulletBodyCount = 0
		}

		// Finalize bodies. Must happen after the constraint solver and after island splitting.
		w.taskCount++
		w.executor.parallelFor(awakeBodyCount, 64, finalizeBodiesTask, context)

		context.bulletBodyCount = w.taskContexts[0].bulletBodyCount
		for i := 1; i < w.workerCount; i++ {
			taskContext := &w.taskContexts[i]
			copy(context.bulletBodies[context.bulletBodyCount:], taskContext.bulletBodies[:taskContext.bulletBodyCount])
			context.bulletBodyCount += taskContext.bulletBodyCount
		}

		for i := 1; i < w.workerCount; i++ {
			inPlaceUnion(&w.taskContexts[0].enlargedSimBitSet, &w.taskContexts[i].enlargedSimBitSet)
		}
		for i := range graphColorCount {
			colors[i].contacts = nil
			colors[i].contacts32 = nil
			colors[i].contactConstraints = nil
			colors[i].contactConstraintsWide = nil
			colors[i].contactConstraints32 = nil
		}
		context.contactConstraints = nil
		context.contactConstraintsWide = nil
		context.joints = nil
		context.contacts = nil
		context.stages = nil
		if context.contactConstraintMem32 != nil {
			w.arena.freeItem(context.contactConstraintMem32)
			context.contactConstraintMem32 = nil
		}
		if context.contactConstraintMemWide != nil {
			w.arena.freeItem(context.contactConstraintMem)
			w.arena.freeItem(context.contactConstraintMemWide)
			context.contactConstraintMemWide = nil
		} else {
			w.arena.freeItem(context.contactConstraintMem)
		}
		context.contactConstraintMem = nil

		w.profile.Transforms = millisecondsSince(transformsStart)
	}

	// Report hit events
	{
		hitEventsStart := time.Now()

		if len(w.contactHitEvents) != 0 {
			panic("dbox2d: the hit events are not clear")
		}

		threshold := w.hitEventThreshold
		colors := &w.constraintGraph.colors
		zero := QZero()
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

		w.profile.HitEvents = millisecondsSince(hitEventsStart)
	}

	// Refit the broad-phase.
	{
		refitStart := time.Now()

		// Finish the tree rebuild that started in collide. It must be
		// complete before the broad-phase is touched.
		w.treeTask.wait()

		enlargedBodyBitSet := &w.taskContexts[0].enlargedSimBitSet

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

		w.profile.Refit = millisecondsSince(refitStart)
	}

	// Continuous collision of the bullet bodies. The proxy refit remains serial.
	if context.bulletBodyCount > 0 {
		bulletsStart := time.Now()

		// Fast bullet bodies
		// Note: a bullet body may be moving slow
		w.taskCount++
		w.executor.parallelFor(context.bulletBodyCount, 8, bulletBodyTask, context)

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

		w.profile.Bullets = millisecondsSince(bulletsStart)
	}

	// Need to free this even if no bullets got processed.
	w.arena.freeItem(context.bulletBodyMem)
	context.bulletBodies = nil
	context.bulletBodyMem = nil
	context.bulletBodyCount = 0

	// Island sleeping
	// This must be done last because putting islands to sleep invalidates the enlarged body bits.
	if w.enableSleep {
		sleepIslandsStart := time.Now()

		// Collect split island candidate for the next time step. No need to split if sleeping is disabled.
		if w.splitIslandId != nullIndex {
			panic("dbox2d: the split candidate is not clear")
		}
		// The ranges are fixed and ascending, so the first worker wins a
		// tie, as the first body wins inside a worker. The reference breaks
		// ties by island id because of work stealing.
		splitSleepTimer := QZero()
		for i := range w.workerCount {
			taskContext := &w.taskContexts[i]
			if taskContext.splitIslandId != nullIndex && splitSleepTimer.Less(taskContext.splitSleepTime) {
				w.splitIslandId = taskContext.splitIslandId
				splitSleepTimer = taskContext.splitSleepTime
			}
		}

		awakeIslandBitSet := &w.taskContexts[0].awakeIslandBitSet
		for i := 1; i < w.workerCount; i++ {
			inPlaceUnion(awakeIslandBitSet, &w.taskContexts[i].awakeIslandBitSet)
		}

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

		w.profile.SleepIslands = millisecondsSince(sleepIslandsStart)
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

// runContactStageBlockScalar keeps flat prepare/store work behind the family hook.
func runContactStageBlockScalar(stage *solverStage, context *stepContext, startIndex, endIndex int) {
	switch stage.stageType {
	case stagePrepareContacts:
		prepareContactsTask(startIndex, endIndex, context)
	case stageStoreImpulses:
		storeImpulsesTask(startIndex, endIndex, context)
	}
}

// runGraphContactBlockScalar keeps the existing scalar graph-contact stages
// behind one hook so the wide family can replace them as a unit.
func runGraphContactBlockScalar(stage *solverStage, context *stepContext, startIndex, endIndex int) {
	switch stage.stageType {
	case stageWarmStart:
		warmStartContactsTask(startIndex, endIndex, context, stage.colorIndex)
	case stageSolve:
		solveContactsTask(startIndex, endIndex, context, stage.colorIndex, true)
	case stageRelax:
		solveContactsTask(startIndex, endIndex, context, stage.colorIndex, false)
	case stageRestitution:
		applyRestitutionTask(startIndex, endIndex, context, stage.colorIndex)
	}
}

// colorContactUnits counts the solver units of a color: the family units of
// the fitting contacts, then one unit per Q32 contact.
func colorContactUnits(color *graphColor) int {
	return colorContactConstraintCount(len(color.contacts)) + len(color.contacts32)
}

// runGraphContactBlock runs a block of color units: the family units it
// covers, then its Q32 contacts.
func runGraphContactBlock(stage *solverStage, context *stepContext, startIndex, endIndex int) {
	color := &context.graph.colors[stage.colorIndex]
	familyUnits := colorContactConstraintCount(len(color.contacts))
	if startIndex < familyUnits {
		runGraphContactFamilyBlock(stage, context, startIndex, min(endIndex, familyUnits))
	}
	if endIndex <= familyUnits {
		return
	}
	start32 := max(startIndex, familyUnits) - familyUnits
	end32 := endIndex - familyUnits
	switch stage.stageType {
	case stageWarmStart:
		warmStartContacts32(start32, end32, context, stage.colorIndex)
	case stageSolve:
		solveContacts32(start32, end32, context, stage.colorIndex, true)
	case stageRelax:
		solveContacts32(start32, end32, context, stage.colorIndex, false)
	case stageRestitution:
		applyRestitution32(start32, end32, context, stage.colorIndex)
	}
}

// allocateContactConstraintsScalar preserves the scalar arena layout while
// allowing the wide hook to own its separate scratch allocation.
func allocateContactConstraintsScalar(w *world, context *stepContext, colors *[graphColorCount]graphColor, overflowIndex, activeContactCount int) {
	contactCount := activeContactCount + len(colors[overflowIndex].contacts)
	contactConstraints, constraintMem := arenaSlice[contactConstraint](&w.arena, contactCount, "contact constraint")
	context.contactConstraints = contactConstraints[:activeContactCount:activeContactCount]
	context.contactConstraintMem = constraintMem
	contactBase := 0
	for i := range graphColorCount {
		color := &colors[i]
		colorContactCount := len(color.contacts)
		color.contactConstraints = contactConstraints[contactBase : contactBase+colorContactCount : contactBase+colorContactCount]
		contactBase += colorContactCount
	}
}
