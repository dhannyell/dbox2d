package dbox2d

// island is a persistent connected component of awake bodies, contacts and
// joints. It corresponds to b2Island in src/island.h.
type island struct {
	// setIndex is the solver set that owns the island, or nullIndex.
	setIndex int

	// localIndex is the position inside the islandSims of the set, or
	// nullIndex.
	localIndex int

	islandId int

	headBody  int
	tailBody  int
	bodyCount int

	headContact  int
	tailContact  int
	contactCount int

	// Deferred: the joint list of the reference.

	// parentIsland links the island to its union-find parent.
	parentIsland int

	// constraintRemoveCount counts the constraints removed from the
	// island. A positive count marks a split candidate.
	constraintRemoveCount int
}

// islandSim moves islands across solver sets. It corresponds to b2IslandSim
// in src/island.h.
type islandSim struct {
	islandId int
}

// createIsland allocates an empty island in a set. It corresponds to
// b2CreateIsland in src/island.c.
func createIsland(w *world, setIndex int) *island {
	if setIndex != awakeSet && setIndex < firstSleepingSet {
		panic("dbox2d: an island lives in the awake set or in a sleeping set")
	}

	islandId := w.islandIdPool.allocId()

	if islandId == len(w.islands) {
		w.islands = append(w.islands, island{})
	} else if w.islands[islandId].setIndex != nullIndex {
		panic("dbox2d: the island slot is in use")
	}

	set := &w.solverSets[setIndex]

	isl := &w.islands[islandId]
	isl.setIndex = setIndex
	isl.localIndex = len(set.islandSims)
	isl.islandId = islandId
	isl.headBody = nullIndex
	isl.tailBody = nullIndex
	isl.bodyCount = 0
	isl.headContact = nullIndex
	isl.tailContact = nullIndex
	isl.contactCount = 0
	isl.parentIsland = nullIndex
	isl.constraintRemoveCount = 0

	set.islandSims = append(set.islandSims, islandSim{islandId: islandId})

	return isl
}

// destroyIsland frees an empty island and its id. It corresponds to
// b2DestroyIsland in src/island.c.
func destroyIsland(w *world, islandId int) {
	if w.splitIslandId == islandId {
		w.splitIslandId = nullIndex
	}

	// assume island is empty
	isl := &w.islands[islandId]
	set := &w.solverSets[isl.setIndex]
	var movedIndex int
	set.islandSims, movedIndex = removeSwap(set.islandSims, isl.localIndex)
	if movedIndex != nullIndex {
		// Fix index on moved element
		movedElement := &set.islandSims[isl.localIndex]
		movedIsland := &w.islands[movedElement.islandId]
		if movedIsland.localIndex != movedIndex {
			panic("dbox2d: the moved island index does not match")
		}
		movedIsland.localIndex = isl.localIndex
	}

	// Free island and id (preserve island revision)
	isl.islandId = nullIndex
	isl.setIndex = nullIndex
	isl.localIndex = nullIndex
	w.islandIdPool.freeId(islandId)
}

// addContactToIsland pushes a contact at the head of the island list. It
// corresponds to b2AddContactToIsland in src/island.c.
func addContactToIsland(w *world, islandId int, c *contact) {
	if c.islandId != nullIndex || c.islandPrev != nullIndex || c.islandNext != nullIndex {
		panic("dbox2d: the contact is already in an island")
	}

	isl := &w.islands[islandId]

	if isl.headContact != nullIndex {
		c.islandNext = isl.headContact
		headContact := &w.contacts[isl.headContact]
		headContact.islandPrev = c.contactId
	}

	isl.headContact = c.contactId
	if isl.tailContact == nullIndex {
		isl.tailContact = isl.headContact
	}

	isl.contactCount += 1
	c.islandId = islandId
}

// findRootIsland walks to the union-find root of an island and compresses
// the path. It returns the root and its id. It mirrors the inline loops of
// b2LinkContact in src/island.c.
func findRootIsland(w *world, islandId int) (*island, int) {
	isl := &w.islands[islandId]
	parentId := isl.parentIsland
	for parentId != nullIndex {
		parent := &w.islands[parentId]
		if parent.parentIsland != nullIndex {
			// path compression
			isl.parentIsland = parent.parentIsland
		}

		isl = parent
		islandId = parentId
		parentId = isl.parentIsland
	}
	return isl, islandId
}

// linkContact links a touching contact into the island graph. It performs
// union-find with path compression to join islands. It corresponds to
// b2LinkContact in src/island.c.
func linkContact(w *world, c *contact) {
	if c.flags&contactTouchingFlag == 0 {
		panic("dbox2d: only a touching contact links islands")
	}

	bodyIdA := c.edges[0].bodyId
	bodyIdB := c.edges[1].bodyId

	bodyA := &w.bodies[bodyIdA]
	bodyB := &w.bodies[bodyIdB]

	if bodyA.setIndex == disabledSet || bodyB.setIndex == disabledSet {
		panic("dbox2d: a disabled body cannot link a contact")
	}
	if bodyA.setIndex == staticSet && bodyB.setIndex == staticSet {
		panic("dbox2d: two static bodies cannot link a contact")
	}

	// Wake bodyB if bodyA is awake and bodyB is sleeping
	if bodyA.setIndex == awakeSet && bodyB.setIndex >= firstSleepingSet {
		wakeSolverSet(w, bodyB.setIndex)
	}

	// Wake bodyA if bodyB is awake and bodyA is sleeping
	if bodyB.setIndex == awakeSet && bodyA.setIndex >= firstSleepingSet {
		wakeSolverSet(w, bodyA.setIndex)
	}

	islandIdA := bodyA.islandId
	islandIdB := bodyB.islandId

	// Static bodies have null island indices.
	if bodyA.setIndex == staticSet && islandIdA != nullIndex {
		panic("dbox2d: a static body has no island")
	}
	if bodyB.setIndex == staticSet && islandIdB != nullIndex {
		panic("dbox2d: a static body has no island")
	}
	if islandIdA == nullIndex && islandIdB == nullIndex {
		panic("dbox2d: a contact links at least one island")
	}

	if islandIdA == islandIdB {
		// Contact in same island
		addContactToIsland(w, islandIdA, c)
		return
	}

	// Union-find root of islandA
	var islandA *island
	if islandIdA != nullIndex {
		islandA, islandIdA = findRootIsland(w, islandIdA)
	}

	// Union-find root of islandB
	var islandB *island
	if islandIdB != nullIndex {
		islandB, islandIdB = findRootIsland(w, islandIdB)
	}

	// Union-Find link island roots
	if islandA != islandB && islandA != nil && islandB != nil {
		if islandB.parentIsland != nullIndex {
			panic("dbox2d: the root island has a parent")
		}
		islandB.parentIsland = islandIdA
	}

	if islandA != nil {
		addContactToIsland(w, islandIdA, c)
	} else {
		addContactToIsland(w, islandIdB, c)
	}
}

// unlinkContact removes a contact from its island when it stops touching
// or when it is destroyed. It corresponds to b2UnlinkContact in
// src/island.c.
func unlinkContact(w *world, c *contact) {
	if c.islandId == nullIndex {
		panic("dbox2d: the contact is not in an island")
	}

	// remove from island
	islandId := c.islandId
	isl := &w.islands[islandId]

	if c.islandPrev != nullIndex {
		prevContact := &w.contacts[c.islandPrev]
		if prevContact.islandNext != c.contactId {
			panic("dbox2d: the island contact list is not doubly linked")
		}
		prevContact.islandNext = c.islandNext
	}

	if c.islandNext != nullIndex {
		nextContact := &w.contacts[c.islandNext]
		if nextContact.islandPrev != c.contactId {
			panic("dbox2d: the island contact list is not doubly linked")
		}
		nextContact.islandPrev = c.islandPrev
	}

	if isl.headContact == c.contactId {
		isl.headContact = c.islandNext
	}

	if isl.tailContact == c.contactId {
		isl.tailContact = c.islandPrev
	}

	if isl.contactCount <= 0 {
		panic("dbox2d: the island contact count underflows")
	}
	isl.contactCount -= 1
	isl.constraintRemoveCount += 1

	c.islandId = nullIndex
	c.islandPrev = nullIndex
	c.islandNext = nullIndex
}

// mergeIsland merges an island into its root island. The bodies and the
// contacts of the child append at the tail of the root lists. It
// corresponds to b2MergeIsland in src/island.c.
func mergeIsland(w *world, isl *island) {
	if isl.parentIsland == nullIndex {
		panic("dbox2d: a root island does not merge")
	}

	rootId := isl.parentIsland
	rootIsland := &w.islands[rootId]
	if rootIsland.parentIsland != nullIndex {
		panic("dbox2d: the parent island is not a root")
	}

	// remap island indices
	bodyId := isl.headBody
	for bodyId != nullIndex {
		b := &w.bodies[bodyId]
		b.islandId = rootId
		bodyId = b.islandNext
	}

	contactId := isl.headContact
	for contactId != nullIndex {
		c := &w.contacts[contactId]
		c.islandId = rootId
		contactId = c.islandNext
	}

	// Deferred: the joints remap here.

	// connect body lists
	if rootIsland.tailBody == nullIndex {
		panic("dbox2d: the root island has no bodies")
	}
	tailBody := &w.bodies[rootIsland.tailBody]
	if tailBody.islandNext != nullIndex {
		panic("dbox2d: the root tail body has a next body")
	}
	tailBody.islandNext = isl.headBody

	if isl.headBody == nullIndex {
		panic("dbox2d: the child island has no bodies")
	}
	headBody := &w.bodies[isl.headBody]
	if headBody.islandPrev != nullIndex {
		panic("dbox2d: the child head body has a previous body")
	}
	headBody.islandPrev = rootIsland.tailBody

	rootIsland.tailBody = isl.tailBody
	rootIsland.bodyCount += isl.bodyCount

	// connect contact lists
	if rootIsland.headContact == nullIndex {
		// Root island has no contacts
		if rootIsland.tailContact != nullIndex || rootIsland.contactCount != 0 {
			panic("dbox2d: the root island contact list is inconsistent")
		}
		rootIsland.headContact = isl.headContact
		rootIsland.tailContact = isl.tailContact
		rootIsland.contactCount = isl.contactCount
	} else if isl.headContact != nullIndex {
		// Both islands have contacts
		if isl.tailContact == nullIndex || isl.contactCount <= 0 {
			panic("dbox2d: the child island contact list is inconsistent")
		}
		if rootIsland.tailContact == nullIndex || rootIsland.contactCount <= 0 {
			panic("dbox2d: the root island contact list is inconsistent")
		}

		tailContact := &w.contacts[rootIsland.tailContact]
		if tailContact.islandNext != nullIndex {
			panic("dbox2d: the root tail contact has a next contact")
		}
		tailContact.islandNext = isl.headContact

		headContact := &w.contacts[isl.headContact]
		if headContact.islandPrev != nullIndex {
			panic("dbox2d: the child head contact has a previous contact")
		}
		headContact.islandPrev = rootIsland.tailContact

		rootIsland.tailContact = isl.tailContact
		rootIsland.contactCount += isl.contactCount
	}

	// Deferred: the joint lists connect here.

	// Track removed constraints
	rootIsland.constraintRemoveCount += isl.constraintRemoveCount
}

// mergeAwakeIslands merges every awake island into its root. A merged
// island leaves the awake islandSims and returns to the pool. It
// corresponds to b2MergeAwakeIslands in src/island.c.
func mergeAwakeIslands(w *world) {
	awake := &w.solverSets[awakeSet]
	islandSims := awake.islandSims
	awakeIslandCount := len(islandSims)

	// Step 1: Ensure every child island points to its root island. This
	// avoids merging a child island with a parent island that has already
	// been merged with a grand-parent island.
	for i := range awakeIslandCount {
		islandId := islandSims[i].islandId

		isl := &w.islands[islandId]

		// find the root island
		rootId := islandId
		rootIsland := isl
		for rootIsland.parentIsland != nullIndex {
			parent := &w.islands[rootIsland.parentIsland]
			if parent.parentIsland != nullIndex {
				// path compression
				rootIsland.parentIsland = parent.parentIsland
			}

			rootId = rootIsland.parentIsland
			rootIsland = parent
		}

		if rootIsland != isl {
			isl.parentIsland = rootId
		}
	}

	// Step 2: merge every awake island into its parent (which must be a
	// root island). Reverse to support removal from awake array.
	for i := awakeIslandCount - 1; i >= 0; i-- {
		islandId := islandSims[i].islandId
		isl := &w.islands[islandId]

		if isl.parentIsland == nullIndex {
			continue
		}

		mergeIsland(w, isl)

		// this call does a remove swap from the end of the island sim array
		destroyIsland(w, islandId)
	}
}

// splitIsland rebuilds the islands of a base island by a depth first
// search over the touching contacts. Only an awake island with removed
// constraints splits. It corresponds to b2SplitIsland in src/island.c.
func splitIsland(w *world, baseId int) {
	baseIsland := &w.islands[baseId]
	setIndex := baseIsland.setIndex

	if setIndex != awakeSet {
		// can only split awake island
		return
	}

	if baseIsland.constraintRemoveCount == 0 {
		// this island doesn't need to be split
		return
	}

	bodyCount := baseIsland.bodyCount

	bodies := w.bodies
	alloc := &w.arena

	stack, stackMem := arenaSlice[int](alloc, bodyCount, "island stack")
	bodyIds, bodyIdsMem := arenaSlice[int](alloc, bodyCount, "body ids")

	// Build array containing all body indices from base island. These
	// serve as seed bodies for the depth first search (DFS).
	index := 0
	nextBody := baseIsland.headBody
	for nextBody != nullIndex {
		bodyIds[index] = nextBody
		index++
		b := &bodies[nextBody]

		// Clear visitation mark
		b.isMarked = false

		nextBody = b.islandNext
	}
	if index != bodyCount {
		panic("dbox2d: the island body list and the body count disagree")
	}

	// Clear contact island flags. Only need to consider contacts already
	// in the base island.
	nextContactId := baseIsland.headContact
	for nextContactId != nullIndex {
		c := &w.contacts[nextContactId]
		c.isMarked = false
		nextContactId = c.islandNext
	}

	// Deferred: the joint marks clear here.

	// Done with the base split island.
	destroyIsland(w, baseId)

	// Each island is found as a depth first search starting from a seed body
	for i := range bodyCount {
		seedIndex := bodyIds[i]
		seed := &bodies[seedIndex]
		if seed.setIndex != setIndex {
			panic("dbox2d: the seed body left the set of the island")
		}

		if seed.isMarked {
			// The body has already been visited
			continue
		}

		stackCount := 0
		stack[stackCount] = seedIndex
		stackCount++
		seed.isMarked = true

		// Create new island
		isl := createIsland(w, setIndex)

		islandId := isl.islandId

		// Perform a depth first search (DFS) on the constraint graph.
		for stackCount > 0 {
			// Grab the next body off the stack and add it to the island.
			stackCount--
			bodyId := stack[stackCount]
			b := &bodies[bodyId]
			if b.setIndex != awakeSet {
				panic("dbox2d: a split visits an awake body only")
			}
			if !b.isMarked {
				panic("dbox2d: a body on the stack is marked")
			}

			// Add body to island
			b.islandId = islandId
			if isl.tailBody != nullIndex {
				bodies[isl.tailBody].islandNext = bodyId
			}
			b.islandPrev = isl.tailBody
			b.islandNext = nullIndex
			isl.tailBody = bodyId

			if isl.headBody == nullIndex {
				isl.headBody = bodyId
			}

			isl.bodyCount += 1

			// Search all contacts connected to this body.
			contactKey := b.headContactKey
			for contactKey != nullIndex {
				contactId := contactKey >> 1
				edgeIndex := contactKey & 1

				c := &w.contacts[contactId]
				if c.contactId != contactId {
					panic("dbox2d: the contact id does not match its slot")
				}

				// Next key
				contactKey = c.edges[edgeIndex].nextKey

				// Has this contact already been added to this island?
				if c.isMarked {
					continue
				}

				// Is this contact enabled and touching?
				if c.flags&contactTouchingFlag == 0 {
					continue
				}

				c.isMarked = true

				otherEdgeIndex := edgeIndex ^ 1
				otherBodyId := c.edges[otherEdgeIndex].bodyId
				otherBody := &bodies[otherBodyId]

				// Maybe add other body to stack
				if !otherBody.isMarked && otherBody.setIndex != staticSet {
					if stackCount >= bodyCount {
						panic("dbox2d: the island stack overflows")
					}
					stack[stackCount] = otherBodyId
					stackCount++
					otherBody.isMarked = true
				}

				// Add contact to island
				c.islandId = islandId
				if isl.tailContact != nullIndex {
					tailContact := &w.contacts[isl.tailContact]
					tailContact.islandNext = contactId
				}
				c.islandPrev = isl.tailContact
				c.islandNext = nullIndex
				isl.tailContact = contactId

				if isl.headContact == nullIndex {
					isl.headContact = contactId
				}

				isl.contactCount += 1
			}

			// Deferred: the joints connected to this body join the search
			// here.
		}
	}

	alloc.freeItem(bodyIdsMem)
	alloc.freeItem(stackMem)
}

// validateIsland checks the lists and the counts of one island. The
// reference compiles it only into validation builds; here only the tests
// call it.
func validateIsland(w *world, islandId int) {
	isl := &w.islands[islandId]
	if isl.islandId != islandId {
		panic("dbox2d: the island id does not match its slot")
	}
	if isl.setIndex == nullIndex {
		panic("dbox2d: the island has no set")
	}
	if isl.headBody == nullIndex {
		panic("dbox2d: the island has no bodies")
	}

	{
		if isl.tailBody == nullIndex || isl.bodyCount <= 0 {
			panic("dbox2d: the island body list is inconsistent")
		}
		if isl.bodyCount > 1 && isl.tailBody == isl.headBody {
			panic("dbox2d: the island body list is inconsistent")
		}
		if isl.bodyCount > w.bodyIdPool.idCount() {
			panic("dbox2d: the island holds more bodies than the pool")
		}

		count := 0
		bodyId := isl.headBody
		for bodyId != nullIndex {
			b := &w.bodies[bodyId]
			if b.islandId != islandId {
				panic("dbox2d: a body does not point back at its island")
			}
			if b.setIndex != isl.setIndex {
				panic("dbox2d: a body and its island differ in set")
			}
			count += 1

			if count == isl.bodyCount && bodyId != isl.tailBody {
				panic("dbox2d: the island tail body is wrong")
			}

			bodyId = b.islandNext
		}
		if count != isl.bodyCount {
			panic("dbox2d: the island body count is wrong")
		}
	}

	if isl.headContact != nullIndex {
		if isl.tailContact == nullIndex || isl.contactCount <= 0 {
			panic("dbox2d: the island contact list is inconsistent")
		}
		if isl.contactCount > 1 && isl.tailContact == isl.headContact {
			panic("dbox2d: the island contact list is inconsistent")
		}
		if isl.contactCount > w.contactIdPool.idCount() {
			panic("dbox2d: the island holds more contacts than the pool")
		}

		count := 0
		contactId := isl.headContact
		for contactId != nullIndex {
			c := &w.contacts[contactId]
			if c.setIndex != isl.setIndex {
				panic("dbox2d: a contact and its island differ in set")
			}
			if c.islandId != islandId {
				panic("dbox2d: a contact does not point back at its island")
			}
			count += 1

			if count == isl.contactCount && contactId != isl.tailContact {
				panic("dbox2d: the island tail contact is wrong")
			}

			contactId = c.islandNext
		}
		if count != isl.contactCount {
			panic("dbox2d: the island contact count is wrong")
		}
	} else if isl.tailContact != nullIndex || isl.contactCount != 0 {
		panic("dbox2d: an island without contacts has a tail or a count")
	}

	// Deferred: the joint list check.
}
