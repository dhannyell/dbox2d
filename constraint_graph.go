package b2

// overflowIndex is the color that holds the constraints that do not fit
// the color limit. It corresponds to B2_OVERFLOW_INDEX in
// src/constraint_graph.h.
const overflowIndex = graphColorCount - 1

// dynamicColorCount keeps the constraints between two dynamic bodies at a
// lower solver priority than the constraints with a static or kinematic
// body, which reduces push through. It corresponds to
// B2_DYNAMIC_COLOR_COUNT in src/constraint_graph.h.
const dynamicColorCount = graphColorCount - 4

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

	// The step splits contactSims, in order, into the contacts the lane grid
	// fits and the contacts32 it does not. In float mode contacts32 is empty.
	contacts   []*contactSim
	contacts32 []*contactSim

	// contactConstraints is the solver scratch of the color. The solver
	// fills it on each step from the arena.
	contactConstraints []contactConstraint
	// contactConstraintsWide is the wide-family solver scratch of the color.
	// The solver fills it on each step from the arena when dbox2d_simd is set.
	contactConstraintsWide []contactConstraintWide
	// contactConstraints32 is the scalar-grid scratch of contacts32.
	contactConstraints32 []contactConstraint32

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

// addContactToGraph clones a touching contact sim into its color (see
// assignConstraintColor). A contact with one dynamic body skips color 0. It
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
	typeA := bodyA.bodyType
	typeB := bodyB.bodyType
	if typeA != DynamicBody && typeB != DynamicBody {
		panic("dbox2d: a graph contact needs a dynamic body")
	}

	if !forceOverflow {
		colorIndex = assignConstraintColor(graph, bodyIdA, bodyIdB, typeA, typeB, 1)
	}

	color := &graph.colors[colorIndex]
	c.colorIndex = colorIndex
	c.localIndex = len(color.contactSims)

	color.contactSims = append(color.contactSims, *cs)
	newContact := &color.contactSims[c.localIndex]

	if typeA == StaticBody {
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

	if typeB == StaticBody {
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
	color.contactSims, movedIndex = removeSwapNoClear(color.contactSims, localIndex)
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

// assignConstraintColor picks the color of a constraint. A constraint
// between two dynamic bodies takes the first dynamic color where neither
// body has a constraint. A constraint with one dynamic body takes the last
// color, down to firstStaticColor, where that body has none: the static
// colors build from the end, so they solve after the dynamic ones and keep
// the higher priority. Kinematic and static bodies get no color bit. It
// returns overflowIndex when no color fits. It corresponds to the color
// loops of b2AddContactToGraph and b2AssignJointColor in
// src/constraint_graph.c.
func assignConstraintColor(graph *constraintGraph, bodyIdA, bodyIdB int, typeA, typeB BodyType, firstStaticColor int) int {
	switch {
	case typeA == DynamicBody && typeB == DynamicBody:
		// Dynamic constraint colors cannot encroach on colors reserved for static constraints
		for i := range dynamicColorCount {
			color := &graph.colors[i]
			if color.bodySet.getBit(bodyIdA) || color.bodySet.getBit(bodyIdB) {
				continue
			}

			color.bodySet.setBitGrow(bodyIdA)
			color.bodySet.setBitGrow(bodyIdB)
			return i
		}
	case typeA == DynamicBody:
		// Static constraint colors build from the end to get higher priority than dyn-dyn constraints
		for i := overflowIndex - 1; i >= firstStaticColor; i-- {
			color := &graph.colors[i]
			if color.bodySet.getBit(bodyIdA) {
				continue
			}

			color.bodySet.setBitGrow(bodyIdA)
			return i
		}
	case typeB == DynamicBody:
		// Static constraint colors build from the end to get higher priority than dyn-dyn constraints
		for i := overflowIndex - 1; i >= firstStaticColor; i-- {
			color := &graph.colors[i]
			if color.bodySet.getBit(bodyIdB) {
				continue
			}

			color.bodySet.setBitGrow(bodyIdB)
			return i
		}
	}

	return overflowIndex
}

// assignJointColor picks the color of a joint. It corresponds to
// b2AssignJointColor in src/constraint_graph.c.
func assignJointColor(graph *constraintGraph, bodyIdA, bodyIdB int, typeA, typeB BodyType) int {
	if typeA != DynamicBody && typeB != DynamicBody {
		panic("dbox2d: a graph joint needs a dynamic body")
	}

	if forceOverflow {
		return overflowIndex
	}
	return assignConstraintColor(graph, bodyIdA, bodyIdB, typeA, typeB, 1)
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
	colorIndex := assignJointColor(graph, bodyIdA, bodyIdB, bodyA.bodyType, bodyB.bodyType)

	color := &graph.colors[colorIndex]
	color.jointSims = append(color.jointSims, jointSim{})

	j.colorIndex = colorIndex
	j.localIndex = len(color.jointSims) - 1
	return &color.jointSims[j.localIndex]
}

// addJointToGraph copies a joint sim into the graph. It corresponds to
// b2AddJointToGraph in src/constraint_graph.c.
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
	color.jointSims, movedIndex = removeSwapNoClear(color.jointSims, localIndex)
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
