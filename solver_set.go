package dbox2d

// solverSet groups the sim data of bodies that the solver treats together:
// one static set, one disabled set, one awake set and one set per sleeping
// island. The grouping gives the solver high memory locality.
type solverSet struct {
	// bodySims of the bodies in this set. Empty for an unused set.
	bodySims []bodySim

	// bodyStates of the bodies. Only the awake set has them.
	bodyStates []bodyState

	// contactSims of the non-touching contacts in this set. A touching
	// contact of the awake set lives in the constraint graph instead.
	contactSims []contactSim

	// jointSims of the joints in this set. A joint of the awake set lives
	// in the constraint graph instead.
	jointSims []jointSim

	// islandSims of the islands in this set. The static and disabled sets
	// have none.
	islandSims []islandSim

	// setIndex aligns with the solverSetIdPool of the world. It gives a
	// stable id to the set, or nullIndex for an unused slot.
	setIndex int
}

// destroySolverSet releases a set and returns its id to the pool.
func destroySolverSet(w *world, setIndex int) {
	set := &w.solverSets[setIndex]
	w.solverSetIdPool.freeId(setIndex)
	*set = solverSet{setIndex: nullIndex}
}

// wakeSolverSet moves a sleeping set into the awake set. It does not merge
// islands. Non-touching contacts of the disabled set move to the awake
// set; touching contacts of the sleeping set enter the constraint graph.
// It corresponds to b2WakeSolverSet in src/solver_set.c.
func wakeSolverSet(w *world, setIndex int) {
	if setIndex < firstSleepingSet {
		panic("dbox2d: only a sleeping set wakes")
	}
	set := &w.solverSets[setIndex]
	awake := &w.solverSets[awakeSet]
	disabled := &w.solverSets[disabledSet]

	bodies := w.bodies

	bodyCount := len(set.bodySims)
	for i := range bodyCount {
		simSrc := &set.bodySims[i]

		b := &bodies[simSrc.bodyId]
		if b.setIndex != setIndex {
			panic("dbox2d: a body does not point at its sleeping set")
		}
		b.setIndex = awakeSet
		b.localIndex = len(awake.bodySims)

		// Reset sleep timer
		b.sleepTime = Q{}

		awake.bodySims = append(awake.bodySims, *simSrc)
		awake.bodyStates = append(awake.bodyStates, identityBodyState())

		// move non-touching contacts from disabled set to awake set
		contactKey := b.headContactKey
		for contactKey != nullIndex {
			edgeIndex := contactKey & 1
			contactId := contactKey >> 1

			c := &w.contacts[contactId]

			contactKey = c.edges[edgeIndex].nextKey

			if c.setIndex != disabledSet {
				if c.setIndex != awakeSet && c.setIndex != setIndex {
					panic("dbox2d: a contact of a sleeping body is in a foreign set")
				}
				continue
			}

			localIndex := c.localIndex
			cs := &disabled.contactSims[localIndex]

			if c.flags&contactTouchingFlag != 0 || cs.manifold.PointCount != 0 {
				panic("dbox2d: a touching contact is in the disabled set")
			}

			c.setIndex = awakeSet
			c.localIndex = len(awake.contactSims)
			awake.contactSims = append(awake.contactSims, *cs)

			var movedLocalIndex int
			disabled.contactSims, movedLocalIndex = removeSwap(disabled.contactSims, localIndex)
			if movedLocalIndex != nullIndex {
				// fix moved element
				movedContactSim := &disabled.contactSims[localIndex]
				movedContact := &w.contacts[movedContactSim.contactId]
				if movedContact.localIndex != movedLocalIndex {
					panic("dbox2d: the moved contact index does not match")
				}
				movedContact.localIndex = localIndex
			}
		}
	}

	// transfer touching contacts from sleeping set to contact graph
	{
		contactCount := len(set.contactSims)
		for i := range contactCount {
			cs := &set.contactSims[i]
			c := &w.contacts[cs.contactId]
			if c.flags&contactTouchingFlag == 0 || cs.simFlags&simTouchingFlag == 0 || cs.manifold.PointCount <= 0 {
				panic("dbox2d: a sleeping set holds a non-touching contact")
			}
			if c.setIndex != setIndex {
				panic("dbox2d: a contact does not point at its sleeping set")
			}
			addContactToGraph(w, cs, c)
			c.setIndex = awakeSet
		}
	}

	// transfer joints from sleeping set to awake set
	{
		jointCount := len(set.jointSims)
		for i := range jointCount {
			js := &set.jointSims[i]
			j := &w.joints[js.jointId]
			if j.setIndex != setIndex {
				panic("dbox2d: a joint does not point at its sleeping set")
			}
			addJointToGraph(w, js, j)
			j.setIndex = awakeSet
		}
	}

	// transfer island from sleeping set to awake set
	// Usually a sleeping set has only one island, but it is possible
	// that joints are created between sleeping islands and they
	// are moved to the same sleeping set.
	{
		islandCount := len(set.islandSims)
		for i := range islandCount {
			islandSrc := set.islandSims[i]
			isl := &w.islands[islandSrc.islandId]
			isl.setIndex = awakeSet
			isl.localIndex = len(awake.islandSims)
			awake.islandSims = append(awake.islandSims, islandSrc)
		}
	}

	// destroy the sleeping set
	destroySolverSet(w, setIndex)
}

// trySleepIsland moves an awake island into a new sleeping set. An island
// with a pending split stays awake. Non-touching contacts of the island
// move to the disabled set when their other body sleeps too. It
// corresponds to b2TrySleepIsland in src/solver_set.c.
func trySleepIsland(w *world, islandId int) {
	isl := &w.islands[islandId]
	if isl.setIndex != awakeSet {
		panic("dbox2d: only an awake island sleeps")
	}

	// cannot put an island to sleep while it has a pending split
	if isl.constraintRemoveCount > 0 {
		return
	}

	// island is sleeping
	// - create new sleeping solver set
	// - move island to sleeping solver set
	// - identify non-touching contacts that should move to sleeping solver set or disabled set
	// - remove old island
	// - fix island
	sleepSetId := w.solverSetIdPool.allocId()
	if sleepSetId == len(w.solverSets) {
		w.solverSets = append(w.solverSets, solverSet{setIndex: nullIndex})
	}

	sleepSet := &w.solverSets[sleepSetId]
	*sleepSet = solverSet{}

	// grab awake set after creating the sleep set because the solver set
	// array may have been resized
	awake := &w.solverSets[awakeSet]
	if isl.localIndex < 0 || isl.localIndex >= len(awake.islandSims) {
		panic("dbox2d: the island local index is out of range")
	}

	sleepSet.setIndex = sleepSetId
	sleepSet.bodySims = make([]bodySim, 0, isl.bodyCount)
	sleepSet.contactSims = make([]contactSim, 0, isl.contactCount)
	sleepSet.jointSims = make([]jointSim, 0, isl.jointCount)

	// move awake bodies to sleeping set
	// this shuffles around bodies in the awake set
	{
		disabled := &w.solverSets[disabledSet]
		bodyId := isl.headBody
		for bodyId != nullIndex {
			b := &w.bodies[bodyId]
			if b.setIndex != awakeSet || b.islandId != islandId {
				panic("dbox2d: a body of the island is not awake in it")
			}

			// Update the body move event to indicate this body fell asleep
			// It could happen the body is forced asleep before it ever moves.
			if b.bodyMoveIndex != nullIndex {
				moveEvent := &w.bodyMoveEvents[b.bodyMoveIndex]
				if int(moveEvent.BodyId.index1)-1 != bodyId || moveEvent.BodyId.generation != b.generation {
					panic("dbox2d: the move event does not belong to the body")
				}
				moveEvent.FellAsleep = true
			}

			awakeBodyIndex := b.localIndex
			awakeSim := &awake.bodySims[awakeBodyIndex]

			// move body sim to sleep set
			sleepBodyIndex := len(sleepSet.bodySims)
			sleepSet.bodySims = append(sleepSet.bodySims, *awakeSim)

			var movedIndex int
			awake.bodySims, movedIndex = removeSwap(awake.bodySims, awakeBodyIndex)
			if movedIndex != nullIndex {
				// fix local index on moved element
				movedSim := &awake.bodySims[awakeBodyIndex]
				movedBody := &w.bodies[movedSim.bodyId]
				if movedBody.localIndex != movedIndex {
					panic("dbox2d: the moved body index does not match")
				}
				movedBody.localIndex = awakeBodyIndex
			}

			// destroy state, no need to clone
			awake.bodyStates, _ = removeSwap(awake.bodyStates, awakeBodyIndex)

			b.setIndex = sleepSetId
			b.localIndex = sleepBodyIndex

			// Move non-touching contacts to the disabled set.
			// Non-touching contacts may exist between sleeping islands and
			// there is no clear ownership.
			contactKey := b.headContactKey
			for contactKey != nullIndex {
				contactId := contactKey >> 1
				edgeIndex := contactKey & 1

				c := &w.contacts[contactId]

				if c.setIndex != awakeSet && c.setIndex != disabledSet {
					panic("dbox2d: a contact of an awake body is in a sleeping set")
				}
				contactKey = c.edges[edgeIndex].nextKey

				if c.setIndex == disabledSet {
					// already moved to disabled set by another body in the island
					continue
				}

				if c.colorIndex != nullIndex {
					// contact is touching and will be moved separately
					if c.flags&contactTouchingFlag == 0 {
						panic("dbox2d: a graph contact is not touching")
					}
					continue
				}

				// the other body may still be awake, it still may go to
				// sleep and then it will be responsible for moving this
				// contact to the disabled set.
				otherEdgeIndex := edgeIndex ^ 1
				otherBodyId := c.edges[otherEdgeIndex].bodyId
				otherBody := &w.bodies[otherBodyId]
				if otherBody.setIndex == awakeSet {
					continue
				}

				localIndex := c.localIndex
				cs := &awake.contactSims[localIndex]

				if cs.manifold.PointCount != 0 || c.flags&contactTouchingFlag != 0 {
					panic("dbox2d: a touching contact is outside the graph")
				}

				// move the non-touching contact to the disabled set
				c.setIndex = disabledSet
				c.localIndex = len(disabled.contactSims)
				disabled.contactSims = append(disabled.contactSims, *cs)

				var movedLocalIndex int
				awake.contactSims, movedLocalIndex = removeSwap(awake.contactSims, localIndex)
				if movedLocalIndex != nullIndex {
					// fix moved element
					movedContactSim := &awake.contactSims[localIndex]
					movedContact := &w.contacts[movedContactSim.contactId]
					if movedContact.localIndex != movedLocalIndex {
						panic("dbox2d: the moved contact index does not match")
					}
					movedContact.localIndex = localIndex
				}
			}

			bodyId = b.islandNext
		}
	}

	// move touching contacts
	// this shuffles contacts in the awake set
	{
		contactId := isl.headContact
		for contactId != nullIndex {
			c := &w.contacts[contactId]
			if c.setIndex != awakeSet || c.islandId != islandId {
				panic("dbox2d: a contact of the island is not awake in it")
			}
			colorIndex := c.colorIndex
			if colorIndex < 0 || colorIndex >= graphColorCount {
				panic("dbox2d: the color index is out of range")
			}

			color := &w.constraintGraph.colors[colorIndex]

			// Remove bodies from graph coloring associated with this constraint
			if colorIndex != overflowIndex {
				// might clear a bit for a static body, but this has no effect
				color.bodySet.clearBit(c.edges[0].bodyId)
				color.bodySet.clearBit(c.edges[1].bodyId)
			}

			localIndex := c.localIndex
			awakeContactSim := &color.contactSims[localIndex]

			sleepContactIndex := len(sleepSet.contactSims)
			sleepSet.contactSims = append(sleepSet.contactSims, *awakeContactSim)

			var movedLocalIndex int
			color.contactSims, movedLocalIndex = removeSwap(color.contactSims, localIndex)
			if movedLocalIndex != nullIndex {
				// fix moved element
				movedContactSim := &color.contactSims[localIndex]
				movedContact := &w.contacts[movedContactSim.contactId]
				if movedContact.localIndex != movedLocalIndex {
					panic("dbox2d: the moved contact index does not match")
				}
				movedContact.localIndex = localIndex
			}

			c.setIndex = sleepSetId
			c.colorIndex = nullIndex
			c.localIndex = sleepContactIndex

			contactId = c.islandNext
		}
	}

	// move joints
	// this shuffles joints in the awake set
	{
		jointId := isl.headJoint
		for jointId != nullIndex {
			j := &w.joints[jointId]
			if j.setIndex != awakeSet || j.islandId != islandId {
				panic("dbox2d: a joint of the island is not awake in it")
			}
			colorIndex := j.colorIndex
			localIndex := j.localIndex

			if colorIndex < 0 || colorIndex >= graphColorCount {
				panic("dbox2d: the color index is out of range")
			}

			color := &w.constraintGraph.colors[colorIndex]

			awakeJointSim := &color.jointSims[localIndex]

			if colorIndex != overflowIndex {
				// might clear a bit for a static body, but this has no effect
				color.bodySet.clearBit(j.edges[0].bodyId)
				color.bodySet.clearBit(j.edges[1].bodyId)
			}

			sleepJointIndex := len(sleepSet.jointSims)
			sleepSet.jointSims = append(sleepSet.jointSims, *awakeJointSim)

			var movedIndex int
			color.jointSims, movedIndex = removeSwap(color.jointSims, localIndex)
			if movedIndex != nullIndex {
				// fix moved element
				movedJointSim := &color.jointSims[localIndex]
				movedId := movedJointSim.jointId
				movedJoint := &w.joints[movedId]
				if movedJoint.localIndex != movedIndex {
					panic("dbox2d: the moved joint has the wrong local index")
				}
				movedJoint.localIndex = localIndex
			}

			j.setIndex = sleepSetId
			j.colorIndex = nullIndex
			j.localIndex = sleepJointIndex

			jointId = j.islandNext
		}
	}

	// move island struct
	{
		if isl.setIndex != awakeSet {
			panic("dbox2d: the island left the awake set")
		}

		islandIndex := isl.localIndex
		sleepSet.islandSims = append(sleepSet.islandSims, islandSim{islandId: islandId})

		var movedIslandIndex int
		awake.islandSims, movedIslandIndex = removeSwap(awake.islandSims, islandIndex)
		if movedIslandIndex != nullIndex {
			// fix index on moved element
			movedIslandSim := &awake.islandSims[islandIndex]
			movedIsland := &w.islands[movedIslandSim.islandId]
			if movedIsland.localIndex != movedIslandIndex {
				panic("dbox2d: the moved island index does not match")
			}
			movedIsland.localIndex = islandIndex
		}

		isl.setIndex = sleepSetId
		isl.localIndex = 0
	}
}

// mergeSolverSets merges two sleeping sets. It moves the smaller set into
// the larger one. Islands merge when the set wakes. It corresponds to
// b2MergeSolverSets in src/solver_set.c.
func mergeSolverSets(w *world, setId1, setId2 int) {
	if setId1 < firstSleepingSet || setId2 < firstSleepingSet {
		panic("dbox2d: only sleeping sets merge")
	}
	set1 := &w.solverSets[setId1]
	set2 := &w.solverSets[setId2]

	// Move the fewest number of bodies
	if len(set1.bodySims) < len(set2.bodySims) {
		set1, set2 = set2, set1
		setId1, setId2 = setId2, setId1
	}

	// transfer bodies
	{
		bodies := w.bodies
		bodyCount := len(set2.bodySims)
		for i := range bodyCount {
			simSrc := &set2.bodySims[i]

			b := &bodies[simSrc.bodyId]
			if b.setIndex != setId2 {
				panic("dbox2d: a body does not point at its set")
			}
			b.setIndex = setId1
			b.localIndex = len(set1.bodySims)

			set1.bodySims = append(set1.bodySims, *simSrc)
		}
	}

	// transfer contacts
	{
		contactCount := len(set2.contactSims)
		for i := range contactCount {
			contactSrc := &set2.contactSims[i]

			c := &w.contacts[contactSrc.contactId]
			if c.setIndex != setId2 {
				panic("dbox2d: a contact does not point at its set")
			}
			c.setIndex = setId1
			c.localIndex = len(set1.contactSims)

			set1.contactSims = append(set1.contactSims, *contactSrc)
		}
	}

	// transfer joints
	{
		jointCount := len(set2.jointSims)
		for i := range jointCount {
			jointSrc := &set2.jointSims[i]

			j := &w.joints[jointSrc.jointId]
			if j.setIndex != setId2 {
				panic("dbox2d: a joint does not point at its set")
			}
			j.setIndex = setId1
			j.localIndex = len(set1.jointSims)

			set1.jointSims = append(set1.jointSims, *jointSrc)
		}
	}

	// transfer islands
	{
		islandCount := len(set2.islandSims)
		for i := range islandCount {
			islandSrc := set2.islandSims[i]

			isl := &w.islands[islandSrc.islandId]
			isl.setIndex = setId1
			isl.localIndex = len(set1.islandSims)

			set1.islandSims = append(set1.islandSims, islandSrc)
		}
	}

	// destroy the merged set
	destroySolverSet(w, setId2)
}

// transferBody moves the sim data of a body from one set to another. The
// state of a body that leaves the awake set is discarded; a body that enters
// it starts from the identity state.
func transferBody(w *world, targetSet, sourceSet *solverSet, b *body) {
	if targetSet == sourceSet {
		panic("dbox2d: transfer of a body inside one set")
	}

	sourceIndex := b.localIndex
	sourceSim := sourceSet.bodySims[sourceIndex]

	targetIndex := len(targetSet.bodySims)
	targetSet.bodySims = append(targetSet.bodySims, sourceSim)

	// Remove body sim from solver set that owns it
	var movedIndex int
	sourceSet.bodySims, movedIndex = removeSwap(sourceSet.bodySims, sourceIndex)
	if movedIndex != nullIndex {
		// Fix moved body index
		movedSim := &sourceSet.bodySims[sourceIndex]
		movedBody := &w.bodies[movedSim.bodyId]
		if movedBody.localIndex != movedIndex {
			panic("dbox2d: the moved body index does not match")
		}
		movedBody.localIndex = sourceIndex
	}

	if sourceSet.setIndex == awakeSet {
		sourceSet.bodyStates, _ = removeSwap(sourceSet.bodyStates, sourceIndex)
	} else if targetSet.setIndex == awakeSet {
		targetSet.bodyStates = append(targetSet.bodyStates, identityBodyState())
	}

	b.setIndex = targetSet.setIndex
	b.localIndex = targetIndex
}

// transferJoint moves a joint sim between two sets. The awake set keeps
// its sims in the graph colors. It corresponds to b2TransferJoint in
// src/solver_set.c.
func transferJoint(w *world, targetSet, sourceSet *solverSet, j *joint) {
	if targetSet == sourceSet {
		panic("dbox2d: the sets are the same")
	}

	localIndex := j.localIndex
	colorIndex := j.colorIndex

	// Retrieve source.
	var sourceSim *jointSim
	if sourceSet.setIndex == awakeSet {
		if colorIndex < 0 || colorIndex >= graphColorCount {
			panic("dbox2d: the color index is out of range")
		}
		color := &w.constraintGraph.colors[colorIndex]

		sourceSim = &color.jointSims[localIndex]
	} else {
		if colorIndex != nullIndex {
			panic("dbox2d: a joint outside the awake set has a color")
		}
		sourceSim = &sourceSet.jointSims[localIndex]
	}

	// Create target and copy. Fix joint.
	if targetSet.setIndex == awakeSet {
		addJointToGraph(w, sourceSim, j)
		j.setIndex = awakeSet
	} else {
		j.setIndex = targetSet.setIndex
		j.localIndex = len(targetSet.jointSims)
		j.colorIndex = nullIndex

		targetSet.jointSims = append(targetSet.jointSims, *sourceSim)
	}

	// Destroy source.
	if sourceSet.setIndex == awakeSet {
		removeJointFromGraph(w, j.edges[0].bodyId, j.edges[1].bodyId, colorIndex, localIndex)
	} else {
		var movedIndex int
		sourceSet.jointSims, movedIndex = removeSwap(sourceSet.jointSims, localIndex)
		if movedIndex != nullIndex {
			// fix swapped element
			movedJointSim := &sourceSet.jointSims[localIndex]
			movedId := movedJointSim.jointId
			movedJoint := &w.joints[movedId]
			movedJoint.localIndex = localIndex
		}
	}
}
