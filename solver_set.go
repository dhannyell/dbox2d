package dbox2d

// solverSet groups the sim data of bodies that the solver treats together:
// one static set, one disabled set, one awake set and one set per sleeping
// island. The grouping gives the solver high memory locality.
type solverSet struct {
	// bodySims of the bodies in this set. Empty for an unused set.
	bodySims []bodySim

	// bodyStates of the bodies. Only the awake set has them.
	bodyStates []bodyState

	// Deferred: the joint, contact and island sim arrays of the reference.

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
