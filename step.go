package dbox2d

import (
	"math/bits"

	"github.com/dhannyell/fixed"
)

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

	// Deferred: the contact pointer array and the worker fields of the
	// reference serve the parallel executor.

	enableWarmStarting bool

	// The body arrays of the awake set.
	sims   []bodySim
	states []bodyState

	// bulletBodies buffers the awake sim indices of the fast bullet
	// bodies for the continuous stage. The slice lives in the arena.
	bulletBodies    []int
	bulletBodyMem   []byte
	bulletBodyCount int
}

// Step advances the simulation by timeStep, split into subStepCount
// sub-steps. The reference recommends a fixed time step and 4 sub-steps.
// It corresponds to b2World_Step in src/world.c.
func Step(worldId WorldId, timeStep Q, subStepCount int) {
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

	// Prepare to capture events
	// Ensure user does not access stale data if there is an early return
	w.bodyMoveEvents = w.bodyMoveEvents[:0]
	w.contactBeginEvents = w.contactBeginEvents[:0]
	w.contactHitEvents = w.contactHitEvents[:0]
	// Deferred: the sensor events of the reference.

	zero := fixed.Q32Zero()
	if timeStep.Eq(zero) {
		// Swap end event array buffers
		w.endEventArrayIndex = 1 - w.endEventArrayIndex
		w.contactEndEvents[w.endEventArrayIndex] = w.contactEndEvents[w.endEventArrayIndex][:0]
		return
	}

	w.locked = true

	// Update collision pairs and create contacts
	updateBroadPhasePairs(w)

	context := stepContext{}
	context.world = w
	context.dt = timeStep
	context.subStepCount = max(1, subStepCount)

	if zero.Less(timeStep) {
		context.invDt = fixed.Q32One().Div(timeStep)
		context.h = timeStep.Div(fixed.Q32FromInt(context.subStepCount))
		context.invH = fixed.Q32FromInt(context.subStepCount).Mul(context.invDt)
	}

	w.invH = context.invH

	// Hertz values get reduced for large time steps
	contactHertz := w.contactHertz.Min(fixed.Q32FromRatio(1, 8).Mul(context.invH))
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
	collide(&context)

	// Integrate velocities, solve velocity constraints, and integrate positions.
	if zero.Less(context.dt) {
		solve(w, &context)
	}

	// Deferred: the sensor overlap update of the reference runs here.

	if getArenaAllocation(&w.arena) != 0 {
		panic("dbox2d: the arena is not empty after the step")
	}

	// Ensure stack is large enough
	w.arena.grow()

	// Swap end event array buffers
	w.endEventArrayIndex = 1 - w.endEventArrayIndex
	w.contactEndEvents[w.endEventArrayIndex] = w.contactEndEvents[w.endEventArrayIndex][:0]
	w.locked = false
}

// collideTask updates the manifolds of a run of contact sims and marks the
// contacts whose touch state changed. It corresponds to b2CollideTask in
// src/world.c; the port walks each array in place instead of a pointer
// array.
func collideTask(contactSims []contactSim, context *stepContext) {
	w := context.world
	taskContext := &w.taskContext
	shapes := w.shapes
	bodies := w.bodies

	for contactIndex := range contactSims {
		cs := &contactSims[contactIndex]

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
	set.contactSims, movedIndex = removeSwap(set.contactSims, localIndex)
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

	// The reference rebuilds the trees on a task beside the collide pass
	// and finishes it before the refit. One worker rebuilds them first.
	w.broadPhase.rebuildTrees()

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
	taskContext := &w.taskContext
	setBitCountAndClear(&taskContext.contactStateBitSet, contactIdCapacity)

	// The reference gathers the sims into one pointer array for the
	// parallel-for. The port walks the colors and the awake set in the
	// same order.
	for i := range graphColorCount {
		collideTask(graphColors[i].contactSims, context)
	}
	collideTask(w.solverSets[awakeSet].contactSims, context)

	// Serially update contact state
	bitSet := &taskContext.contactStateBitSet

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
