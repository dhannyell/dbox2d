package dbox2d

// overflowIndex is the color that holds the constraints that do not fit
// the color limit. It corresponds to B2_OVERFLOW_INDEX in
// src/constraint_graph.h.
const overflowIndex = graphColorCount - 1

// forceOverflow sends every constraint to the overflow color. It is a
// debug switch of the reference; it corresponds to B2_FORCE_OVERFLOW in
// src/constraint_graph.c.
const forceOverflow = false

// graphColor holds the constraints that share no dynamic body. It
// corresponds to b2GraphColor in src/constraint_graph.h.
type graphColor struct {
	// bodySet is indexed by body id, so it is over-sized to encompass
	// static bodies. The overflow color does not use it.
	bodySet bitSet

	// contactSims of the touching contacts in this color.
	contactSims []contactSim

	// contactConstraints is the solver scratch of the color. The solver
	// fills it on each step from the arena.
	contactConstraints []contactConstraint

	// jointSims of the joints in this color.
	jointSims []jointSim
}

// constraintGraph colors the awake constraints so that each color can
// solve in parallel. The overflow color comes last. It corresponds to
// b2ConstraintGraph in src/constraint_graph.h.
type constraintGraph struct {
	colors [graphColorCount]graphColor
}

// createGraph resets a graph and sizes the body sets. The overflow color
// gets no set. It corresponds to b2CreateGraph in src/constraint_graph.c.
func createGraph(graph *constraintGraph, bodyCapacity int) {
	*graph = constraintGraph{}

	bodyCapacity = max(bodyCapacity, 8)

	// Initialize graph color bit set.
	// No bitset for overflow color.
	for i := range overflowIndex {
		color := &graph.colors[i]
		color.bodySet = createBitSet(bodyCapacity)
		setBitCountAndClear(&color.bodySet, bodyCapacity)
	}
}

// destroyGraph releases the colors. It corresponds to b2DestroyGraph in
// src/constraint_graph.c.
func destroyGraph(graph *constraintGraph) {
	for i := range graphColorCount {
		color := &graph.colors[i]

		// The bit set should never be used on the overflow color
		if i == overflowIndex && color.bodySet.bits != nil {
			panic("dbox2d: the overflow color has a body set")
		}

		*color = graphColor{}
	}
}

// addContactToGraph clones a touching contact sim into the first color
// that has neither body. A contact with a static body skips color 0. It
// corresponds to b2AddContactToGraph in src/constraint_graph.c.
func addContactToGraph(w *world, cs *contactSim, c *contact) {
	if cs.manifold.PointCount <= 0 {
		panic("dbox2d: a contact without points cannot enter the graph")
	}
	if cs.simFlags&simTouchingFlag == 0 || c.flags&contactTouchingFlag == 0 {
		panic("dbox2d: a non-touching contact cannot enter the graph")
	}

	graph := &w.constraintGraph
	colorIndex := overflowIndex

	bodyIdA := c.edges[0].bodyId
	bodyIdB := c.edges[1].bodyId
	bodyA := &w.bodies[bodyIdA]
	bodyB := &w.bodies[bodyIdB]
	staticA := bodyA.setIndex == staticSet
	staticB := bodyB.setIndex == staticSet
	if staticA && staticB {
		panic("dbox2d: two static bodies cannot touch")
	}

	if !forceOverflow {
		switch {
		case !staticA && !staticB:
			for i := range overflowIndex {
				color := &graph.colors[i]
				if color.bodySet.getBit(bodyIdA) || color.bodySet.getBit(bodyIdB) {
					continue
				}

				color.bodySet.setBitGrow(bodyIdA)
				color.bodySet.setBitGrow(bodyIdB)
				colorIndex = i
				break
			}
		case !staticA:
			// No static contacts in color 0
			for i := 1; i < overflowIndex; i++ {
				color := &graph.colors[i]
				if color.bodySet.getBit(bodyIdA) {
					continue
				}

				color.bodySet.setBitGrow(bodyIdA)
				colorIndex = i
				break
			}
		case !staticB:
			// No static contacts in color 0
			for i := 1; i < overflowIndex; i++ {
				color := &graph.colors[i]
				if color.bodySet.getBit(bodyIdB) {
					continue
				}

				color.bodySet.setBitGrow(bodyIdB)
				colorIndex = i
				break
			}
		}
	}

	color := &graph.colors[colorIndex]
	c.colorIndex = colorIndex
	c.localIndex = len(color.contactSims)

	color.contactSims = append(color.contactSims, *cs)
	newContact := &color.contactSims[c.localIndex]

	if staticA {
		newContact.bodySimIndexA = nullIndex
		newContact.invMassA = Q{}
		newContact.invIA = Q{}
	} else {
		if bodyA.setIndex != awakeSet {
			panic("dbox2d: a graph contact needs an awake body")
		}
		awake := &w.solverSets[awakeSet]

		localIndex := bodyA.localIndex
		newContact.bodySimIndexA = localIndex

		bodySimA := &awake.bodySims[localIndex]
		newContact.invMassA = bodySimA.invMass
		newContact.invIA = bodySimA.invInertia
	}

	if staticB {
		newContact.bodySimIndexB = nullIndex
		newContact.invMassB = Q{}
		newContact.invIB = Q{}
	} else {
		if bodyB.setIndex != awakeSet {
			panic("dbox2d: a graph contact needs an awake body")
		}
		awake := &w.solverSets[awakeSet]

		localIndex := bodyB.localIndex
		newContact.bodySimIndexB = localIndex

		bodySimB := &awake.bodySims[localIndex]
		newContact.invMassB = bodySimB.invMass
		newContact.invIB = bodySimB.invInertia
	}
}

// removeContactFromGraph frees the color bits of both bodies and removes
// the sim by swap. It corresponds to b2RemoveContactFromGraph in
// src/constraint_graph.c.
func removeContactFromGraph(w *world, bodyIdA, bodyIdB, colorIndex, localIndex int) {
	graph := &w.constraintGraph

	if colorIndex < 0 || colorIndex >= graphColorCount {
		panic("dbox2d: the color index is out of range")
	}
	color := &graph.colors[colorIndex]

	if colorIndex != overflowIndex {
		// might clear a bit for a static body, but this has no effect
		color.bodySet.clearBit(bodyIdA)
		color.bodySet.clearBit(bodyIdB)
	}

	var movedIndex int
	color.contactSims, movedIndex = removeSwap(color.contactSims, localIndex)
	if movedIndex != nullIndex {
		// Fix index on swapped contact
		movedContactSim := &color.contactSims[localIndex]

		// Fix moved contact
		movedContact := &w.contacts[movedContactSim.contactId]
		if movedContact.setIndex != awakeSet || movedContact.colorIndex != colorIndex || movedContact.localIndex != movedIndex {
			panic("dbox2d: the moved contact does not point back at its sim")
		}
		movedContact.localIndex = localIndex
	}
}

// assignJointColor picks the first color where neither dynamic body has
// a constraint. Unlike a contact, a joint with a static body may take
// color zero. It corresponds to b2AssignJointColor in
// src/constraint_graph.c.
func assignJointColor(graph *constraintGraph, bodyIdA, bodyIdB int, staticA, staticB bool) int {
	if staticA && staticB {
		panic("dbox2d: two static bodies cannot share a joint color")
	}

	if !forceOverflow {
		switch {
		case !staticA && !staticB:
			for i := range overflowIndex {
				color := &graph.colors[i]
				if color.bodySet.getBit(bodyIdA) || color.bodySet.getBit(bodyIdB) {
					continue
				}

				color.bodySet.setBitGrow(bodyIdA)
				color.bodySet.setBitGrow(bodyIdB)
				return i
			}
		case !staticA:
			for i := range overflowIndex {
				color := &graph.colors[i]
				if color.bodySet.getBit(bodyIdA) {
					continue
				}

				color.bodySet.setBitGrow(bodyIdA)
				return i
			}
		case !staticB:
			for i := range overflowIndex {
				color := &graph.colors[i]
				if color.bodySet.getBit(bodyIdB) {
					continue
				}

				color.bodySet.setBitGrow(bodyIdB)
				return i
			}
		}
	}

	return overflowIndex
}

// createJointInGraph appends a zeroed joint sim to a color and points the
// joint at it. It corresponds to b2CreateJointInGraph in
// src/constraint_graph.c.
func createJointInGraph(w *world, j *joint) *jointSim {
	graph := &w.constraintGraph

	bodyIdA := j.edges[0].bodyId
	bodyIdB := j.edges[1].bodyId
	bodyA := &w.bodies[bodyIdA]
	bodyB := &w.bodies[bodyIdB]
	staticA := bodyA.setIndex == staticSet
	staticB := bodyB.setIndex == staticSet

	colorIndex := assignJointColor(graph, bodyIdA, bodyIdB, staticA, staticB)

	color := &graph.colors[colorIndex]
	color.jointSims = append(color.jointSims, jointSim{})

	j.colorIndex = colorIndex
	j.localIndex = len(color.jointSims) - 1
	return &color.jointSims[j.localIndex]
}

// addJointToGraph copies a joint sim into the graph. It corresponds to
// b2AddJointToGraph in src/constraint_graph.c.
//
//nolint:unused // The set transfer reads this in a later stage.
func addJointToGraph(w *world, js *jointSim, j *joint) {
	jointDst := createJointInGraph(w, j)
	*jointDst = *js
}

// removeJointFromGraph takes a joint sim out of its color and fixes the
// joint that moved into its slot. It corresponds to
// b2RemoveJointFromGraph in src/constraint_graph.c.
func removeJointFromGraph(w *world, bodyIdA, bodyIdB, colorIndex, localIndex int) {
	graph := &w.constraintGraph

	if colorIndex < 0 || colorIndex >= graphColorCount {
		panic("dbox2d: the color index is out of range")
	}
	color := &graph.colors[colorIndex]

	if colorIndex != overflowIndex {
		// May clear static bodies, no effect
		color.bodySet.clearBit(bodyIdA)
		color.bodySet.clearBit(bodyIdB)
	}

	var movedIndex int
	color.jointSims, movedIndex = removeSwap(color.jointSims, localIndex)
	if movedIndex != nullIndex {
		// Fix moved joint
		movedJointSim := &color.jointSims[localIndex]
		movedId := movedJointSim.jointId
		movedJoint := &w.joints[movedId]
		if movedJoint.setIndex != awakeSet || movedJoint.colorIndex != colorIndex || movedJoint.localIndex != movedIndex {
			panic("dbox2d: the moved joint does not point back at its sim")
		}
		movedJoint.localIndex = localIndex
	}
}
