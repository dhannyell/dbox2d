package dbox2d

import (
	"math/bits"
	"slices"
	"sync/atomic"
	"time"
)

// millisecondsSince returns the elapsed wall-clock time since start, in
// milliseconds. It never feeds the simulation.
func millisecondsSince(start time.Time) float64 {
	return float64(time.Since(start)) / 1e6
}

// millisecondsAndReset returns the elapsed wall-clock time since *t and
// resets *t to now, mirroring b2GetMillisecondsAndReset.
func millisecondsAndReset(t *time.Time) float64 {
	now := time.Now()
	elapsed := float64(now.Sub(*t)) / 1e6
	*t = now
	return elapsed
}

// stepContext carries the per-step data that the solver stages share. It
// corresponds to b2StepContext in src/solver.h.
type stepContext struct {
	world *world

	// dt is the time step of the full step.
	dt Q

	// invDt is the inverse of dt, or zero when dt is zero.
	invDt Q

	// h is the sub-step time: dt divided by the sub-step count.
	h Q

	// invH is the inverse of h, or zero when h is zero.
	invH Q

	subStepCount int

	// Stiffer for static contacts to avoid bodies getting pushed through
	// the ground.
	contactSoftness softness
	staticSoftness  softness

	restitutionThreshold Q
	maxLinearVelocity    Q

	graph *constraintGraph

	// Flat constraint arrays cover colors 0 through 10 in color order.
	contacts                 []*contactSim
	joints                   []*jointSim
	contactConstraints       []contactConstraint
	contactConstraintsWide   []contactConstraintWide
	contactConstraintMem     []byte
	contactConstraintMemWide []byte
	contactConstraintMem32   []byte
	stages                   []solverStage
	activeColorCount         int
	activeColorIndices       [graphColorCount]int
	workerCount              int

	enableWarmStarting bool

	// The body arrays of the awake set.
	sims   []bodySim
	states []bodyState

	// bulletBodies buffers the awake sim indices of the fast bullet
	// bodies for the continuous stage. The slice lives in the arena.
	bulletBodies    []int
	bulletBodyMem   []byte
	bulletBodyCount int

	pad0 [64]byte //nolint:unused // Keeps atomicSyncBits on its own cache line.

	atomicSyncBits atomic.Uint32
}

// Step advances the simulation by timeStep, split into subStepCount
// sub-steps. The reference recommends a fixed time step and 4 sub-steps.
// It corresponds to b2World_Step in src/world.c.
func (worldId WorldId) Step(timeStep Q, subStepCount int) {
	if !IsValidQ(timeStep) {
		panic("dbox2d: the time step is not valid")
	}
	if subStepCount <= 0 {
		panic("dbox2d: the sub-step count is not positive")
	}

	w := getWorldFromId(worldId)
	if w.locked {
		panic("dbox2d: the world is locked")
	}
	w.taskCount = 0
	if w.workerCount > 1 {
		w.executor.start(w.workerCount)
	}
	w.executor.serial = len(w.solverSets[awakeSet].bodySims) < serialBodyThreshold

	// Prepare to capture events
	// Ensure user does not access stale data if there is an early return
	w.bodyMoveEvents = w.bodyMoveEvents[:0]
	w.contactBeginEvents = w.contactBeginEvents[:0]
	w.contactHitEvents = w.contactHitEvents[:0]
	w.sensorBeginEvents = w.sensorBeginEvents[:0]

	w.profile = Profile{}

	zero := QZero()
	if timeStep.Eq(zero) {
		// Swap end event array buffers
		w.endEventArrayIndex = 1 - w.endEventArrayIndex
		w.sensorEndEvents[w.endEventArrayIndex] = w.sensorEndEvents[w.endEventArrayIndex][:0]
		w.contactEndEvents[w.endEventArrayIndex] = w.contactEndEvents[w.endEventArrayIndex][:0]
		return
	}

	w.locked = true

	stepStart := time.Now()

	context := &w.solverContext
	context.world = w

	// Update collision pairs and create contacts
	pairsStart := time.Now()
	updateBroadPhasePairs(w, context)
	w.profile.Pairs = millisecondsSince(pairsStart)

	context.dt = timeStep
	context.invDt = zero
	context.h = zero
	context.invH = zero
	context.subStepCount = max(1, subStepCount)
	context.activeColorCount = 0

	if zero.Less(timeStep) {
		context.invDt = QOne().Div(timeStep)
		context.h = timeStep.Div(QFromInt(context.subStepCount))
		context.invH = QFromInt(context.subStepCount).Mul(context.invDt)
	}

	w.invH = context.invH

	// Hertz values get reduced for large time steps
	contactHertz := w.contactHertz.Min(QFromRatio(1, 8).Mul(context.invH))
	context.contactSoftness = makeSoft(contactHertz, w.contactDampingRatio, context.h)
	context.staticSoftness = makeSoft(contactHertz.Add(contactHertz), w.contactDampingRatio, context.h)

	// D-006: a zero contact frequency gives a zero mass scale. The
	// reference divides and gets infinity; the port keeps a zero speed.
	if context.staticSoftness.massScale.Eq(zero) {
		w.contactSpeed = zero
	} else {
		w.contactSpeed = w.maxContactPushSpeed.Div(context.staticSoftness.massScale)
	}

	context.restitutionThreshold = w.restitutionThreshold
	context.maxLinearVelocity = w.maxLinearSpeed
	context.graph = &w.constraintGraph
	context.enableWarmStarting = w.enableWarmStarting

	// Update contacts
	collideStart := time.Now()
	collide(context)
	w.profile.Collide = millisecondsSince(collideStart)

	// Integrate velocities, solve velocity constraints, and integrate positions.
	if zero.Less(context.dt) {
		solveStart := time.Now()
		solve(w, context)
		w.profile.Solve = millisecondsSince(solveStart)
	}

	// D-016: a zero time step skips the solve, so the refit never joined
	// the tree rebuild. The reference leaks the task handle here and lets
	// the sensor queries read a tree that is still being rebuilt; the port
	// joins instead. Every other path has already joined, so this is free.
	w.treeTask.wait()

	sensorsStart := time.Now()
	overlapSensors(context)
	w.profile.Sensors = millisecondsSince(sensorsStart)

	w.profile.Step = millisecondsSince(stepStart)

	if getArenaAllocation(&w.arena) != 0 {
		panic("dbox2d: the arena is not empty after the step")
	}

	// Ensure stack is large enough
	w.arena.grow()

	// Swap end event array buffers
	w.endEventArrayIndex = 1 - w.endEventArrayIndex
	w.sensorEndEvents[w.endEventArrayIndex] = w.sensorEndEvents[w.endEventArrayIndex][:0]
	w.contactEndEvents[w.endEventArrayIndex] = w.contactEndEvents[w.endEventArrayIndex][:0]
	w.locked = false
}

// collideTask updates the manifolds of a run of contact sims and marks the
// contacts whose touch state changed. It corresponds to b2CollideTask in
// src/world.c.
func collideTask(startIndex, endIndex, workerIndex int, context *stepContext) {
	w := context.world
	taskContext := &w.taskContexts[workerIndex]
	shapes := w.shapes
	bodies := w.bodies

	for contactIndex := startIndex; contactIndex < endIndex; contactIndex++ {
		cs := context.contacts[contactIndex]

		contactId := cs.contactId

		shapeA := &shapes[cs.shapeIdA]
		shapeB := &shapes[cs.shapeIdB]

		// Do proxies still overlap?
		overlap := AABBOverlaps(shapeA.fatAABB, shapeB.fatAABB)
		if !overlap {
			cs.simFlags |= simDisjoint
			cs.simFlags &^= simTouchingFlag
			taskContext.contactStateBitSet.setBit(contactId)
		} else {
			wasTouching := cs.simFlags&simTouchingFlag != 0

			// Update contact respecting shape/body order (A,B)
			bodyA := &bodies[shapeA.bodyId]
			bodyB := &bodies[shapeB.bodyId]
			bodySimA := getBodySim(w, bodyA)
			bodySimB := getBodySim(w, bodyB)

			// avoid cache misses in b2PrepareContactsTask
			cs.bodySimIndexA = nullIndex
			if bodyA.setIndex == awakeSet {
				cs.bodySimIndexA = bodyA.localIndex
			}
			cs.invMassA = bodySimA.invMass
			cs.invIA = bodySimA.invInertia

			cs.bodySimIndexB = nullIndex
			if bodyB.setIndex == awakeSet {
				cs.bodySimIndexB = bodyB.localIndex
			}
			cs.invMassB = bodySimB.invMass
			cs.invIB = bodySimB.invInertia

			transformA := bodySimA.transform
			transformB := bodySimB.transform

			centerOffsetA := RotateVector(transformA.Q, bodySimA.localCenter)
			centerOffsetB := RotateVector(transformB.Q, bodySimB.localCenter)

			// This updates solid contacts
			touching := updateContact(w, cs, shapeA, transformA, centerOffsetA, shapeB, transformB, centerOffsetB)

			// State changes that affect island connectivity. Also affects contact events.
			if touching && !wasTouching {
				cs.simFlags |= simStartedTouching
				taskContext.contactStateBitSet.setBit(contactId)
			} else if !touching && wasTouching {
				cs.simFlags |= simStoppedTouching
				taskContext.contactStateBitSet.setBit(contactId)
			}
		}
	}
}

// treeTask rebuilds the dynamic and kinematic broad-phase trees in parallel
// with the rest of the step. This matches userTreeTask in src/world.c, where
// the rebuild starts during narrow phase and finishes at the refit.
//
// It uses a dedicated goroutine because the worker pool is fully occupied by
// the parallel loops that run in between.
//
// Nothing reads these trees before wait. Proxy updates, the move buffer, and
// bullet queries all happen after the refit.
//
// With one worker, start only records the world and wait runs the rebuild
// synchronously, matching the reference behavior.
type treeTask struct {
	start   chan *world
	done    chan struct{}
	pending *world
	running bool
}

// begin hands the rebuild to the task goroutine, or holds it for wait when
// the world runs on a single worker.
func (t *treeTask) begin(w *world) {
	if w.executor.activeWorkerCount() == 1 {
		t.pending = w
		return
	}
	if t.start == nil {
		t.start = make(chan *world, 1)
		t.done = make(chan struct{}, 1)
		go rebuildTreesLoop(t.start, t.done)
	}
	t.running = true
	t.start <- w
}

// wait joins the rebuild. It is a no-op when none is outstanding, so every
// path out of a step can call it.
func (t *treeTask) wait() {
	if t.pending != nil {
		w := t.pending
		t.pending = nil
		w.broadPhase.rebuildTrees()
		return
	}
	if !t.running {
		return
	}
	<-t.done
	t.running = false
}

// stop joins the rebuild and releases the goroutine.
func (t *treeTask) stop() {
	t.wait()
	if t.start != nil {
		close(t.start)
		t.start = nil
		t.done = nil
	}
}

func rebuildTreesLoop(start chan *world, done chan struct{}) {
	for w := range start {
		w.broadPhase.rebuildTrees()
		done <- struct{}{}
	}
}

// addNonTouchingContact copies a sim that stopped touching into the awake
// set. It corresponds to b2AddNonTouchingContact in src/world.c.
func addNonTouchingContact(w *world, c *contact, cs *contactSim) {
	if c.setIndex != awakeSet {
		panic("dbox2d: a non-touching contact must be awake")
	}
	set := &w.solverSets[awakeSet]
	c.colorIndex = nullIndex
	c.localIndex = len(set.contactSims)
	set.contactSims = append(set.contactSims, *cs)
}

// removeNonTouchingContact removes a sim from a set by swap and fixes the
// moved contact. It corresponds to b2RemoveNonTouchingContact in
// src/world.c.
func removeNonTouchingContact(w *world, setIndex, localIndex int) {
	set := &w.solverSets[setIndex]
	var movedIndex int
	set.contactSims, movedIndex = removeSwapNoClear(set.contactSims, localIndex)
	if movedIndex != nullIndex {
		movedContactSim := &set.contactSims[localIndex]
		movedContact := &w.contacts[movedContactSim.contactId]
		if movedContact.setIndex != setIndex || movedContact.localIndex != movedIndex || movedContact.colorIndex != nullIndex {
			panic("dbox2d: the moved contact does not point back at its sim")
		}
		movedContact.localIndex = localIndex
	}
}

// collide runs the narrow-phase over every awake contact, then applies the
// touch state changes in contact id order: a disjoint contact dies, a
// contact that started to touch links its island and enters the graph, a
// contact that stopped leaves both. It corresponds to b2Collide in
// src/world.c.
func collide(context *stepContext) {
	w := context.world

	// Start the rebuild before the contact count is known, matching the reference.
	// Even worlds with no contacts still need their trees rebuilt, and starting
	// here lets the work overlap the narrow phase and solve.
	w.taskCount++
	w.treeTask.begin(w)

	graphColors := &w.constraintGraph.colors
	contactCount := 0
	for i := range graphColorCount {
		contactCount += len(graphColors[i].contactSims)
	}

	nonTouchingCount := len(w.solverSets[awakeSet].contactSims)
	contactCount += nonTouchingCount

	if contactCount == 0 {
		return
	}

	// Contact bit set on ids because contact pointers are unstable as they move between touching and not touching.
	contactIdCapacity := w.contactIdPool.idCapacity()
	for i := range w.workerCount {
		setBitCountAndClear(&w.taskContexts[i].contactStateBitSet, contactIdCapacity)
	}

	// One pointer array over the colors and the awake set, in that order,
	// so the ranges split evenly.
	w.contactPointers = slices.Grow(w.contactPointers[:0], contactCount)[:contactCount]
	contactIndex := 0
	for i := range graphColorCount {
		for j := range graphColors[i].contactSims {
			w.contactPointers[contactIndex] = &graphColors[i].contactSims[j]
			contactIndex++
		}
	}
	for i := range w.solverSets[awakeSet].contactSims {
		w.contactPointers[contactIndex] = &w.solverSets[awakeSet].contactSims[i]
		contactIndex++
	}
	context.contacts = w.contactPointers
	w.taskCount++
	w.executor.parallelFor(contactCount, 64, collideTask, context)
	context.contacts = nil

	// Serially update contact state
	bitSet := &w.taskContexts[0].contactStateBitSet
	for i := 1; i < w.workerCount; i++ {
		inPlaceUnion(bitSet, &w.taskContexts[i].contactStateBitSet)
	}

	awake := &w.solverSets[awakeSet]

	endEventArrayIndex := w.endEventArrayIndex

	// Process contact state changes. Iterate over set bits
	for k := range bitSet.bits {
		bitsWord := bitSet.bits[k]
		for bitsWord != 0 {
			ctz := bits.TrailingZeros64(bitsWord)
			contactId := 64*k + ctz

			c := &w.contacts[contactId]
			if c.setIndex != awakeSet {
				panic("dbox2d: a changed contact is not awake")
			}

			colorIndex := c.colorIndex
			localIndex := c.localIndex

			var cs *contactSim
			if colorIndex != nullIndex {
				// contact lives in constraint graph
				if colorIndex < 0 || colorIndex >= graphColorCount {
					panic("dbox2d: the color index is out of range")
				}
				color := &graphColors[colorIndex]
				cs = &color.contactSims[localIndex]
			} else {
				cs = &awake.contactSims[localIndex]
			}

			shapeIdA := shapeIdOf(w, &w.shapes[c.shapeIdA])
			shapeIdB := shapeIdOf(w, &w.shapes[c.shapeIdB])
			flags := c.flags
			simFlags := cs.simFlags

			switch {
			case simFlags&simDisjoint != 0:
				// Bounding boxes no longer overlap
				destroyContact(w, c, false)
			case simFlags&simStartedTouching != 0:
				if c.islandId != nullIndex {
					panic("dbox2d: a contact that starts to touch is in an island")
				}

				if flags&contactEnableContactEvents != 0 {
					event := ContactBeginTouchEvent{ShapeIdA: shapeIdA, ShapeIdB: shapeIdB, Manifold: cs.manifold}
					w.contactBeginEvents = append(w.contactBeginEvents, event)
				}

				if cs.manifold.PointCount <= 0 {
					panic("dbox2d: a contact that starts to touch has no points")
				}

				// Link first because this wakes colliding bodies and ensures the body sims
				// are in the correct place.
				c.flags |= contactTouchingFlag
				linkContact(w, c)

				// Make sure these didn't change
				if c.colorIndex != nullIndex || c.localIndex != localIndex {
					panic("dbox2d: the link moved the contact")
				}

				// Contact sim pointer may have become orphaned due to awake set growth,
				// so I just need to refresh it.
				cs = &awake.contactSims[localIndex]

				cs.simFlags &^= simStartedTouching

				addContactToGraph(w, cs, c)
				removeNonTouchingContact(w, awakeSet, localIndex)
			case simFlags&simStoppedTouching != 0:
				cs.simFlags &^= simStoppedTouching
				c.flags &^= contactTouchingFlag

				if c.flags&contactEnableContactEvents != 0 {
					event := ContactEndTouchEvent{ShapeIdA: shapeIdA, ShapeIdB: shapeIdB}
					w.contactEndEvents[endEventArrayIndex] = append(w.contactEndEvents[endEventArrayIndex], event)
				}

				if cs.manifold.PointCount != 0 {
					panic("dbox2d: a contact that stops touching keeps points")
				}

				unlinkContact(w, c)
				bodyIdA := c.edges[0].bodyId
				bodyIdB := c.edges[1].bodyId

				addNonTouchingContact(w, c, cs)
				removeContactFromGraph(w, bodyIdA, bodyIdB, colorIndex, localIndex)
			}

			// Clear the smallest set bit
			bitsWord &= bitsWord - 1
		}
	}
}
