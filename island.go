package b2

// contactLink is a contact of an island with its two body ids, so the split
// pass never needs to touch the contact. It corresponds to b2ContactLink in
// src/island.h.
type contactLink struct {
	contactId int
	bodyIdA   int
	bodyIdB   int
}

// jointLink is a joint of an island with its two body ids. It corresponds
// to b2JointLink in src/island.h.
type jointLink struct {
	jointId int
	bodyIdA int
	bodyIdB int
}

// island is a persistent connected component of awake bodies, contacts and
// joints. It corresponds to b2Island in src/island.h.
type island struct {
	// setIndex is the solver set that owns the island, or nullIndex.
	setIndex int

	// localIndex is the position inside the islandSims of the set, or
	// nullIndex.
	localIndex int

	islandId int

	// constraintRemoveCount counts the constraints removed from the
	// island. A positive count marks a split candidate.
	constraintRemoveCount int

	bodies []int

	// contacts and joints belong to this island. They may connect to
	// static bodies not in the island.
	contacts []contactLink
	joints   []jointLink
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
	isl.bodies = nil
	isl.contacts = nil
	isl.joints = nil
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
	{
		localIndex := isl.localIndex
		lastIndex := len(set.islandSims) - 1
		if localIndex < 0 || localIndex > lastIndex {
			panic("dbox2d: the island local index is out of range")
		}
		moveIslandId := set.islandSims[lastIndex].islandId
		set.islandSims[localIndex] = set.islandSims[lastIndex]
		w.islands[moveIslandId].localIndex = localIndex
		set.islandSims = set.islandSims[:lastIndex]
	}

	// Free island and id (preserve island revision)
	isl.bodies = nil
	isl.contacts = nil
	isl.joints = nil
	isl.constraintRemoveCount = 0
	isl.localIndex = nullIndex
	isl.islandId = nullIndex
	isl.setIndex = nullIndex

	w.islandIdPool.freeId(islandId)
}

// mergeIslands moves the bodies and the constraints of the smaller island
// into the bigger one and destroys the smaller. It returns the surviving
// island id. It corresponds to b2MergeIslands in src/island.c.
func mergeIslands(w *world, islandIdA, islandIdB int) int {
	if islandIdA == islandIdB {
		return islandIdA
	}

	if islandIdA == nullIndex {
		return islandIdB
	}

	if islandIdB == nullIndex {
		return islandIdA
	}

	var smallIsland, bigIsland *island
	{
		islandA := &w.islands[islandIdA]
		islandB := &w.islands[islandIdB]

		// Keep the biggest island to reduce cache misses
		if len(islandA.bodies) >= len(islandB.bodies) {
			bigIsland = islandA
			smallIsland = islandB
		} else {
			bigIsland = islandB
			smallIsland = islandA
		}
	}

	bigIslandId := bigIsland.islandId

	// Move bodies from smaller island to larger island
	for _, bodyId := range smallIsland.bodies {
		b := &w.bodies[bodyId]
		b.islandId = bigIslandId
		b.islandIndex = len(bigIsland.bodies)
		bigIsland.bodies = append(bigIsland.bodies, bodyId)
	}

	// Migrate contacts from smaller island to larger island
	for _, link := range smallIsland.contacts {
		c := &w.contacts[link.contactId]
		c.islandId = bigIslandId
		c.islandIndex = len(bigIsland.contacts)
		bigIsland.contacts = append(bigIsland.contacts, link)
	}

	// Migrate joints from smaller island to larger island
	for _, link := range smallIsland.joints {
		j := &w.joints[link.jointId]
		j.islandId = bigIslandId
		j.islandIndex = len(bigIsland.joints)
		bigIsland.joints = append(bigIsland.joints, link)
	}

	// Track removed constraints
	bigIsland.constraintRemoveCount += smallIsland.constraintRemoveCount

	destroyIsland(w, smallIsland.islandId)

	return bigIslandId
}

// addContactToIsland appends a contact to an island. It corresponds to
// b2AddContactToIsland in src/island.c.
func addContactToIsland(w *world, islandId int, c *contact) {
	if c.islandId != nullIndex || c.islandIndex != nullIndex {
		panic("dbox2d: the contact is already in an island")
	}

	isl := &w.islands[islandId]

	c.islandId = islandId
	c.islandIndex = len(isl.contacts)

	isl.contacts = append(isl.contacts, contactLink{
		contactId: c.contactId,
		bodyIdA:   c.edges[0].bodyId,
		bodyIdB:   c.edges[1].bodyId,
	})
}

// linkContact links a touching contact into an island, merging the islands
// of its bodies. It corresponds to b2LinkContact in src/island.c.
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

	// Merge islands. This will destroy one of the islands.
	finalIslandId := mergeIslands(w, islandIdA, islandIdB)

	// Add contact to the island that survived
	addContactToIsland(w, finalIslandId, c)
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

	removeIndex := c.islandIndex
	if removeIndex < 0 || removeIndex >= len(isl.contacts) || isl.contacts[removeIndex].contactId != c.contactId {
		panic("dbox2d: the contact island index is stale")
	}

	var movedIndex int
	isl.contacts, movedIndex = removeSwapNoClear(isl.contacts, removeIndex)
	if movedIndex != nullIndex {
		// Fix islandIndex on the contact that was swapped into removeIndex
		movedContact := &w.contacts[isl.contacts[removeIndex].contactId]
		if movedContact.islandIndex != movedIndex {
			panic("dbox2d: the moved contact island index does not match")
		}
		movedContact.islandIndex = removeIndex
	}

	c.islandId = nullIndex
	c.islandIndex = nullIndex
	isl.constraintRemoveCount += 1
}

// addJointToIsland appends a joint to an island. It corresponds to
// b2AddJointToIsland in src/island.c.
func addJointToIsland(w *world, islandId int, j *joint) {
	if j.islandId != nullIndex || j.islandIndex != nullIndex {
		panic("dbox2d: the joint is already in an island")
	}

	isl := &w.islands[islandId]

	j.islandId = islandId
	j.islandIndex = len(isl.joints)

	isl.joints = append(isl.joints, jointLink{
		jointId: j.jointId,
		bodyIdA: j.edges[0].bodyId,
		bodyIdB: j.edges[1].bodyId,
	})
}

// linkJoint links a joint into an island, merging the islands of its
// bodies. It corresponds to b2LinkJoint in src/island.c.
func linkJoint(w *world, j *joint) {
	bodyA := &w.bodies[j.edges[0].bodyId]
	bodyB := &w.bodies[j.edges[1].bodyId]

	if bodyA.bodyType != DynamicBody && bodyB.bodyType != DynamicBody {
		panic("dbox2d: a linked joint has a dynamic body")
	}

	if bodyA.setIndex == awakeSet && bodyB.setIndex >= firstSleepingSet {
		wakeSolverSet(w, bodyB.setIndex)
	} else if bodyB.setIndex == awakeSet && bodyA.setIndex >= firstSleepingSet {
		wakeSolverSet(w, bodyA.setIndex)
	}

	islandIdA := bodyA.islandId
	islandIdB := bodyB.islandId

	if islandIdA == nullIndex && islandIdB == nullIndex {
		panic("dbox2d: a joint links at least one island")
	}

	// Merge islands. This will destroy one of the islands.
	finalIslandId := mergeIslands(w, islandIdA, islandIdB)

	// Add joint the island that survived
	addJointToIsland(w, finalIslandId, j)
}

// unlinkJoint removes a joint from its island, if it has one. It
// corresponds to b2UnlinkJoint in src/island.c.
func unlinkJoint(w *world, j *joint) {
	if j.islandId == nullIndex {
		return
	}

	// remove from island
	islandId := j.islandId
	isl := &w.islands[islandId]

	removeIndex := j.islandIndex
	if removeIndex < 0 || removeIndex >= len(isl.joints) || isl.joints[removeIndex].jointId != j.jointId {
		panic("dbox2d: the joint island index is stale")
	}

	var movedIndex int
	isl.joints, movedIndex = removeSwapNoClear(isl.joints, removeIndex)
	if movedIndex != nullIndex {
		// Fix islandIndex on the joint that was swapped into removeIndex
		movedJoint := &w.joints[isl.joints[removeIndex].jointId]
		if movedJoint.islandIndex != movedIndex {
			panic("dbox2d: the moved joint island index does not match")
		}
		movedJoint.islandIndex = removeIndex
	}

	j.islandId = nullIndex
	j.islandIndex = nullIndex
	isl.constraintRemoveCount += 1
}

// islandFindParent finds the root of a node, halving the path on the way.
// It corresponds to b2IslandFindParent in src/island.c.
func islandFindParent(parents []int, node int) int {
	// Walk the chain of parents to find the node that is its own parent (the root)
	for parents[node] != node {
		grandParent := parents[parents[node]]
		parents[node] = grandParent
		node = grandParent
	}

	return node
}

// islandUnion connects the components of two nodes by rank and carries the
// per-component contact and joint counts. It corresponds to b2IslandUnion
// in src/island.c.
func islandUnion(parents, ranks []int, node1, node2 int, contactCounts, jointCounts []int) {
	root1 := islandFindParent(parents, node1)
	root2 := islandFindParent(parents, node2)
	if root1 != root2 {
		if ranks[root1] < ranks[root2] {
			parents[root1] = root2
			contactCounts[root2] += contactCounts[root1]
			jointCounts[root2] += jointCounts[root1]
		} else if ranks[root1] > ranks[root2] {
			parents[root2] = root1
			contactCounts[root1] += contactCounts[root2]
			jointCounts[root1] += jointCounts[root2]
		} else {
			parents[root2] = root1
			ranks[root1] += 1
			contactCounts[root1] += contactCounts[root2]
			jointCounts[root1] += jointCounts[root2]
		}
	}
}

// splitIsland rebuilds an awake island with removed constraints into its
// connected components using union-find. It corresponds to b2SplitIsland in
// src/island.c.
func splitIsland(w *world, baseId int) {
	baseIsland := &w.islands[baseId]
	if baseIsland.constraintRemoveCount <= 0 {
		panic("dbox2d: a split island has removed constraints")
	}
	if baseIsland.setIndex != awakeSet {
		panic("dbox2d: only an awake island splits")
	}

	// Cache base island fields before createIsland, which may reallocate
	// w.islands and invalidate the baseIsland pointer.
	baseBodyIds := baseIsland.bodies
	baseBodyCount := len(baseBodyIds)
	baseContacts := baseIsland.contacts
	baseJoints := baseIsland.joints

	alloc := &w.arena

	// No lock is needed because the allocator is not used while this task
	// is active. Allocate contactCounts and jointCounts before ranks so
	// ranks can be freed first.
	parents, parentsMem := arenaSlice[int](alloc, baseBodyCount, "parents")
	contactCounts, contactCountsMem := arenaSlice[int](alloc, baseBodyCount, "contact counts")
	jointCounts, jointCountsMem := arenaSlice[int](alloc, baseBodyCount, "joint counts")
	ranks, ranksMem := arenaSlice[int](alloc, baseBodyCount, "ranks")
	for i := range baseBodyCount {
		parents[i] = i
		ranks[i] = 0
		contactCounts[i] = 0
		jointCounts[i] = 0
	}

	bodies := w.bodies

	// Union over contacts, tracking per-component contact counts
	for i := range baseContacts {
		islandIndexA := bodies[baseContacts[i].bodyIdA].islandIndex
		islandIndexB := bodies[baseContacts[i].bodyIdB].islandIndex

		// Only connect non-static bodies
		if islandIndexA != nullIndex && islandIndexB != nullIndex {
			islandUnion(parents, ranks, islandIndexA, islandIndexB, contactCounts, jointCounts)
			root := islandFindParent(parents, islandIndexA)
			contactCounts[root] += 1
		} else {
			islandIndex := islandIndexA
			if islandIndex == nullIndex {
				islandIndex = islandIndexB
			}
			root := islandFindParent(parents, islandIndex)
			contactCounts[root] += 1
		}
	}

	// Union over joints, tracking per-component joint counts
	for i := range baseJoints {
		islandIndexA := bodies[baseJoints[i].bodyIdA].islandIndex
		islandIndexB := bodies[baseJoints[i].bodyIdB].islandIndex

		// Only connect non-static bodies
		if islandIndexA != nullIndex && islandIndexB != nullIndex {
			islandUnion(parents, ranks, islandIndexA, islandIndexB, contactCounts, jointCounts)
			root := islandFindParent(parents, islandIndexA)
			jointCounts[root] += 1
		} else {
			islandIndex := islandIndexA
			if islandIndex == nullIndex {
				islandIndex = islandIndexB
			}
			root := islandFindParent(parents, islandIndex)
			jointCounts[root] += 1
		}
	}

	// Done with ranks
	alloc.freeItem(ranksMem)

	// Flatten all parent indices and count connected components.
	componentCount := 0
	for i := range baseBodyCount {
		parents[i] = islandFindParent(parents, i)
		if parents[i] == i {
			componentCount += 1
		}
	}

	// Early return, the island is still fully connected.
	if componentCount == 1 {
		baseIsland.constraintRemoveCount = 0
		alloc.freeItem(jointCountsMem)
		alloc.freeItem(contactCountsMem)
		alloc.freeItem(parentsMem)
		return
	}

	// Detach the arrays from the base island so destroyIsland leaves them.
	baseIsland.bodies = nil
	baseIsland.contacts = nil
	baseIsland.joints = nil

	// Nil so code below doesn't accidentally use this.
	baseIsland = nil

	// Map from body index to new island index. Only set for root bodies.
	rootMap, rootMapMem := arenaSlice[int](alloc, baseBodyCount, "root map")
	for i := range baseBodyCount {
		rootMap[i] = nullIndex
	}

	componentBodyCounts, componentBodyCountsMem := arenaSlice[int](alloc, componentCount, "component body counts")
	componentContactCounts, componentContactCountsMem := arenaSlice[int](alloc, componentCount, "component contact counts")
	componentJointCounts, componentJointCountsMem := arenaSlice[int](alloc, componentCount, "component joint counts")
	islandCount := 0

	// Find the root body for each body and create islands as needed.
	// Extract per-component counts from the root nodes' accumulated counts.
	for i := range baseBodyCount {
		rootIndex := parents[i]
		if rootMap[rootIndex] == nullIndex {
			rootMap[rootIndex] = islandCount
			componentBodyCounts[islandCount] = 0
			componentContactCounts[islandCount] = contactCounts[rootIndex]
			componentJointCounts[islandCount] = jointCounts[rootIndex]
			islandCount += 1
		}

		componentBodyCounts[rootMap[rootIndex]] += 1
	}

	if islandCount != componentCount {
		panic("dbox2d: the split island count does not match the components")
	}

	// Map from new island index to island id
	islandIds, islandIdsMem := arenaSlice[int](alloc, islandCount, "island ids")

	// Create new islands and reserve body/contact/joint arrays
	for i := range islandCount {
		newIsland := createIsland(w, awakeSet)
		islandIds[i] = newIsland.islandId

		// Reserve arrays to avoid wasteful growth and copies.
		newIsland.bodies = make([]int, 0, componentBodyCounts[i])
		newIsland.contacts = make([]contactLink, 0, componentContactCounts[i])
		newIsland.joints = make([]jointLink, 0, componentJointCounts[i])
	}

	// Assign bodies to new islands
	for i := range baseBodyCount {
		bodyId := baseBodyIds[i]
		root := islandFindParent(parents, i)
		newIslandId := islandIds[rootMap[root]]

		b := &w.bodies[bodyId]
		newIsland := &w.islands[newIslandId]

		b.islandId = newIslandId
		b.islandIndex = len(newIsland.bodies)
		newIsland.bodies = append(newIsland.bodies, bodyId)
	}

	// Assign contacts to the island of their bodies
	for _, link := range baseContacts {
		c := &w.contacts[link.contactId]

		// Static bodies don't have an island id.
		targetIslandId := w.bodies[link.bodyIdA].islandId
		if targetIslandId == nullIndex {
			targetIslandId = w.bodies[link.bodyIdB].islandId
		}

		targetIsland := &w.islands[targetIslandId]
		c.islandId = targetIslandId
		c.islandIndex = len(targetIsland.contacts)
		targetIsland.contacts = append(targetIsland.contacts, link)
	}

	// Assign joints to the island of their bodies
	for _, link := range baseJoints {
		j := &w.joints[link.jointId]

		// Static bodies don't have an island id.
		targetIslandId := w.bodies[link.bodyIdA].islandId
		if targetIslandId == nullIndex {
			targetIslandId = w.bodies[link.bodyIdB].islandId
		}

		targetIsland := &w.islands[targetIslandId]
		j.islandId = targetIslandId
		j.islandIndex = len(targetIsland.joints)
		targetIsland.joints = append(targetIsland.joints, link)
	}

	// Destroy the base island
	destroyIsland(w, baseId)

	// Free arena items in LIFO order
	alloc.freeItem(islandIdsMem)
	alloc.freeItem(componentJointCountsMem)
	alloc.freeItem(componentContactCountsMem)
	alloc.freeItem(componentBodyCountsMem)
	alloc.freeItem(rootMapMem)
	alloc.freeItem(jointCountsMem)
	alloc.freeItem(contactCountsMem)
	alloc.freeItem(parentsMem)
}

// validateIsland checks the arrays and the back indices of one island. The
// reference compiles it only into validation builds; here only the tests
// call it.
func validateIsland(w *world, islandId int) {
	if islandId == nullIndex {
		return
	}

	isl := &w.islands[islandId]
	if isl.islandId != islandId {
		panic("dbox2d: the island id does not match its slot")
	}
	if isl.setIndex == nullIndex {
		panic("dbox2d: the island has no set")
	}

	if len(isl.bodies) == 0 {
		panic("dbox2d: the island has no bodies")
	}
	if len(isl.bodies) > w.bodyIdPool.idCount() {
		panic("dbox2d: the island holds more bodies than the pool")
	}
	for i, bodyId := range isl.bodies {
		b := &w.bodies[bodyId]
		if b.islandId != islandId {
			panic("dbox2d: a body does not point back at its island")
		}
		if b.islandIndex != i {
			panic("dbox2d: a body island index is stale")
		}
		if b.setIndex != isl.setIndex {
			panic("dbox2d: a body and its island differ in set")
		}
	}

	if len(isl.contacts) > w.contactIdPool.idCount() {
		panic("dbox2d: the island holds more contacts than the pool")
	}
	for i := range isl.contacts {
		c := &w.contacts[isl.contacts[i].contactId]
		if c.setIndex != isl.setIndex {
			panic("dbox2d: a contact and its island differ in set")
		}
		if c.islandId != islandId {
			panic("dbox2d: a contact does not point back at its island")
		}
		if c.islandIndex != i {
			panic("dbox2d: a contact island index is stale")
		}
	}

	if len(isl.joints) > w.jointIdPool.idCount() {
		panic("dbox2d: the island joint count exceeds the pool")
	}
	for i := range isl.joints {
		j := &w.joints[isl.joints[i].jointId]
		if j.setIndex != isl.setIndex {
			panic("dbox2d: a joint and its island differ in set")
		}
		if j.islandId != islandId {
			panic("dbox2d: a joint does not point back at its island")
		}
		if j.islandIndex != i {
			panic("dbox2d: a joint island index is stale")
		}
	}
}
