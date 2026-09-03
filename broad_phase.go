package dbox2d

// The proxy key packs the body type in the low two bits and the tree
// proxy id above them. It corresponds to B2_PROXY_KEY and its companions
// in src/broad_phase.h.
func proxyKeyOf(id int, proxyType BodyType) int {
	return (id << 2) | int(proxyType)
}

func proxyTypeOf(key int) BodyType {
	return BodyType(key & 3)
}

func proxyIdOf(key int) int {
	return key >> 2
}

// movePair is one candidate pair of a moved proxy. The pairs of one moved
// proxy form a list through next. The reference links pairs by pointer;
// the port links them by index into movePairs.
type movePair struct {
	shapeIndexA int
	shapeIndexB int
	next        int
}

// moveResult is the head of the pair list of one moved proxy.
type moveResult struct {
	pairList int
}

// broadPhase computes the candidate pairs and serves the volume queries.
// It does not persist pairs: it reports the new pairs of the proxies that
// moved and the world consumes them. It corresponds to b2BroadPhase in
// src/broad_phase.h.
type broadPhase struct {
	// trees holds one tree per body type.
	trees [BodyTypeCount]dynamicTree

	// The move set and array track the proxies that moved enough to need a
	// pair query. The array keeps a deterministic order. The set holds the
	// key plus one, because zero is the sentinel of the set.
	moveSet   hashSet
	moveArray []int

	// The pair query fills moveResults and movePairs from the arena, and
	// the world creates the contacts from them in deterministic order.
	moveResults   []moveResult
	movePairs     []movePair
	moveResultMem []byte
	movePairMem   []byte

	// pairSet tracks the shape pairs that have a contact.
	pairSet hashSet
}

// createBroadPhase initializes the trees and the sets. It corresponds to
// b2CreateBroadPhase in src/broad_phase.c.
func createBroadPhase(bp *broadPhase) {
	bp.moveSet = createSet(16)
	bp.moveArray = make([]int, 0, 16)
	bp.moveResults = nil
	bp.movePairs = nil
	bp.pairSet = createSet(32)

	for i := range BodyTypeCount {
		bp.trees[i] = createTree()
	}
}

// destroyBroadPhase releases the trees and the sets.
func destroyBroadPhase(bp *broadPhase) {
	for i := range BodyTypeCount {
		destroyTree(&bp.trees[i])
	}

	destroySet(&bp.moveSet)
	bp.moveArray = nil
	destroySet(&bp.pairSet)

	*bp = broadPhase{}
}

// bufferMove records a proxy for the next pair query. This is what
// triggers new contact pairs. The caller must call it in deterministic
// order. It corresponds to b2BufferMove in src/broad_phase.h.
func (bp *broadPhase) bufferMove(queryProxy int) {
	// Adding 1 because 0 is the sentinel
	alreadyAdded := bp.moveSet.addKey(uint64(queryProxy + 1))
	if !alreadyAdded {
		bp.moveArray = append(bp.moveArray, queryProxy)
	}
}

// unBufferMove drops a proxy from the pending pair query.
func (bp *broadPhase) unBufferMove(proxyKey int) {
	found := bp.moveSet.removeKey(uint64(proxyKey + 1))

	if found {
		// Purge from move buffer. Linear search.
		count := len(bp.moveArray)
		for i := range count {
			if bp.moveArray[i] == proxyKey {
				bp.moveArray, _ = removeSwap(bp.moveArray, i)
				break
			}
		}
	}
}

// createProxy inserts a shape box in the tree of its body type and returns
// the proxy key. A non-static proxy, or a forced one, joins the move
// buffer. It corresponds to b2BroadPhase_CreateProxy in
// src/broad_phase.c.
func (bp *broadPhase) createProxy(proxyType BodyType, aabb AABB, categoryBits uint64, shapeIndex int, forcePairCreation bool) int {
	if proxyType < 0 || proxyType >= BodyTypeCount {
		panic("dbox2d: the proxy type is out of range")
	}
	proxyId := bp.trees[proxyType].createProxy(aabb, categoryBits, uint64(shapeIndex))
	proxyKey := proxyKeyOf(proxyId, proxyType)
	if proxyType != StaticBody || forcePairCreation {
		bp.bufferMove(proxyKey)
	}
	return proxyKey
}

// destroyProxy removes a proxy from its tree and from the move buffer.
func (bp *broadPhase) destroyProxy(proxyKey int) {
	if len(bp.moveArray) != bp.moveSet.count {
		panic("dbox2d: the move array and the move set disagree")
	}
	bp.unBufferMove(proxyKey)

	proxyType := proxyTypeOf(proxyKey)
	proxyId := proxyIdOf(proxyKey)

	if proxyType < 0 || proxyType >= BodyTypeCount {
		panic("dbox2d: the proxy type is out of range")
	}
	bp.trees[proxyType].destroyProxy(proxyId)
}

// moveProxy reinserts a proxy with a new box and buffers it.
func (bp *broadPhase) moveProxy(proxyKey int, aabb AABB) {
	proxyType := proxyTypeOf(proxyKey)
	proxyId := proxyIdOf(proxyKey)

	bp.trees[proxyType].moveProxy(proxyId, aabb)
	bp.bufferMove(proxyKey)
}

// enlargeProxy grows a non-static proxy in place and buffers it.
func (bp *broadPhase) enlargeProxy(proxyKey int, aabb AABB) {
	if proxyKey == nullIndex {
		panic("dbox2d: enlargeProxy on the null key")
	}
	typeIndex := proxyTypeOf(proxyKey)
	proxyId := proxyIdOf(proxyKey)

	if typeIndex == StaticBody {
		panic("dbox2d: a static proxy cannot enlarge")
	}

	bp.trees[typeIndex].enlargeProxy(proxyId, aabb)
	bp.bufferMove(proxyKey)
}

// queryPairContext carries the state of one moved proxy through the tree
// queries of findPairs. It corresponds to b2QueryPairContext in
// src/broad_phase.c.
type queryPairContext struct {
	w               *world
	moveResult      *moveResult
	queryTreeType   BodyType
	queryProxyKey   int
	queryShapeIndex int
}

// pairQueryCallback receives each proxy that overlaps a moved proxy and
// records the pair when no rule rejects it. It corresponds to
// b2PairQueryCallback in src/broad_phase.c.
func (ctx *queryPairContext) pairQueryCallback(proxyId int, userData uint64) bool {
	shapeId := int(userData)

	w := ctx.w
	bp := &w.broadPhase

	proxyKey := proxyKeyOf(proxyId, ctx.queryTreeType)
	queryProxyKey := ctx.queryProxyKey

	// A proxy cannot form a pair with itself.
	if proxyKey == queryProxyKey {
		return true
	}

	treeType := ctx.queryTreeType
	queryProxyType := proxyTypeOf(queryProxyKey)

	// De-duplication. Both proxies of a pair may sit in the move set, so
	// the pair must come from only one of them. A static proxy can sit in
	// the move set too, so the static tree gets no shortcut.

	// Is this proxy also moving?
	if queryProxyType == DynamicBody {
		if treeType == DynamicBody && proxyKey < queryProxyKey {
			moved := bp.moveSet.containsKey(uint64(proxyKey + 1))
			if moved {
				// Both proxies are moving. Avoid duplicate pairs.
				return true
			}
		}
	} else {
		if treeType != DynamicBody {
			panic("dbox2d: a non-dynamic proxy queried a non-dynamic tree")
		}
		moved := bp.moveSet.containsKey(uint64(proxyKey + 1))
		if moved {
			// Both proxies are moving. Avoid duplicate pairs.
			return true
		}
	}

	pairKey := shapePairKey(shapeId, ctx.queryShapeIndex)
	if bp.pairSet.containsKey(pairKey) {
		// contact exists
		return true
	}

	var shapeIdA, shapeIdB int
	if proxyKey < queryProxyKey {
		shapeIdA = shapeId
		shapeIdB = ctx.queryShapeIndex
	} else {
		shapeIdA = ctx.queryShapeIndex
		shapeIdB = shapeId
	}

	shapeA := &w.shapes[shapeIdA]
	shapeB := &w.shapes[shapeIdB]

	bodyIdA := shapeA.bodyId
	bodyIdB := shapeB.bodyId

	// Are the shapes on the same body?
	if bodyIdA == bodyIdB {
		return true
	}

	// Sensors are handled elsewhere
	if shapeA.sensorIndex != nullIndex || shapeB.sensorIndex != nullIndex {
		return true
	}

	if !shouldShapesCollide(shapeA.filter, shapeB.filter) {
		return true
	}

	// Does a joint override collision?
	bodyA := &w.bodies[bodyIdA]
	bodyB := &w.bodies[bodyIdB]
	if !shouldBodiesCollide(w, bodyA, bodyB) {
		return true
	}

	// Deferred: the custom filter callback of the reference.

	// D-010: the arena slice grows by append when the sixteen pairs per
	// moved proxy run out; the reference takes those from the heap.
	pairIndex := len(bp.movePairs)
	bp.movePairs = append(bp.movePairs, movePair{shapeIndexA: shapeIdA, shapeIndexB: shapeIdB, next: nullIndex})

	// D-013: the reference prepends, so the list follows the tree walk.
	// The port keeps the list sorted by (shapeIdA, shapeIdB), so any tree
	// with the same leaves creates the contacts in the same order.
	prev := nullIndex
	cur := ctx.moveResult.pairList
	for cur != nullIndex {
		p := &bp.movePairs[cur]
		if shapeIdA < p.shapeIndexA || (shapeIdA == p.shapeIndexA && shapeIdB < p.shapeIndexB) {
			break
		}
		prev = cur
		cur = p.next
	}
	bp.movePairs[pairIndex].next = cur
	if prev == nullIndex {
		ctx.moveResult.pairList = pairIndex
	} else {
		bp.movePairs[prev].next = pairIndex
	}

	// continue the query
	return true
}

// findPairs queries the trees for every moved proxy in the range and
// fills its move result. It corresponds to b2FindPairsTask in
// src/broad_phase.c for one worker.
func findPairs(w *world, startIndex, endIndex int) {
	bp := &w.broadPhase

	ctx := queryPairContext{w: w}

	for i := startIndex; i < endIndex; i++ {
		// Initialize move result for this moved proxy
		ctx.moveResult = &bp.moveResults[i]
		ctx.moveResult.pairList = nullIndex

		proxyKey := bp.moveArray[i]
		if proxyKey == nullIndex {
			// proxy was destroyed after it moved
			continue
		}

		proxyType := proxyTypeOf(proxyKey)

		proxyId := proxyIdOf(proxyKey)
		ctx.queryProxyKey = proxyKey

		baseTree := &bp.trees[proxyType]

		// We have to query the tree with the fat AABB so that
		// we don't fail to create a contact that may touch later.
		fatAABB := baseTree.getAABB(proxyId)
		ctx.queryShapeIndex = int(baseTree.getUserData(proxyId))

		// Query trees. Only dynamic proxies collide with kinematic and static proxies.
		// Using DefaultMaskBits so that Filter.GroupIndex works.
		if proxyType == DynamicBody {
			ctx.queryTreeType = KinematicBody
			bp.trees[KinematicBody].query(fatAABB, DefaultMaskBits, ctx.pairQueryCallback)

			ctx.queryTreeType = StaticBody
			bp.trees[StaticBody].query(fatAABB, DefaultMaskBits, ctx.pairQueryCallback)
		}

		// All proxies collide with dynamic proxies
		// Using DefaultMaskBits so that Filter.GroupIndex works.
		ctx.queryTreeType = DynamicBody
		bp.trees[DynamicBody].query(fatAABB, DefaultMaskBits, ctx.pairQueryCallback)
	}
}

// updateBroadPhasePairs finds the new pairs of the moved proxies and
// creates their contacts in the order of the move array. It corresponds
// to b2UpdateBroadPhasePairs in src/broad_phase.c.
func updateBroadPhasePairs(w *world) {
	bp := &w.broadPhase

	moveCount := len(bp.moveArray)
	if moveCount != bp.moveSet.count {
		panic("dbox2d: the move array and the move set disagree")
	}

	if moveCount == 0 {
		return
	}

	alloc := &w.arena

	bp.moveResults, bp.moveResultMem = arenaSlice[moveResult](alloc, moveCount, "move results")
	movePairCapacity := 16 * moveCount
	bp.movePairs, bp.movePairMem = arenaSlice[movePair](alloc, movePairCapacity, "move pairs")
	bp.movePairs = bp.movePairs[:0]

	findPairs(w, 0, moveCount)

	// Single-threaded work
	// - Clear move flags
	// - Create contacts in deterministic order
	for i := range moveCount {
		result := &bp.moveResults[i]
		pair := result.pairList
		for pair != nullIndex {
			shapeIdA := bp.movePairs[pair].shapeIndexA
			shapeIdB := bp.movePairs[pair].shapeIndexB

			shapeA := &w.shapes[shapeIdA]
			shapeB := &w.shapes[shapeIdB]

			createContact(w, shapeA, shapeB)

			pair = bp.movePairs[pair].next
		}
	}

	// Reset move buffer
	bp.moveArray = bp.moveArray[:0]
	clearSet(&bp.moveSet)

	alloc.freeItem(bp.movePairMem)
	bp.movePairs = nil
	bp.movePairMem = nil
	alloc.freeItem(bp.moveResultMem)
	bp.moveResults = nil
	bp.moveResultMem = nil
}

// testOverlap reports whether the boxes of two proxies overlap.
func (bp *broadPhase) testOverlap(proxyKeyA, proxyKeyB int) bool {
	typeIndexA := proxyTypeOf(proxyKeyA)
	proxyIdA := proxyIdOf(proxyKeyA)
	typeIndexB := proxyTypeOf(proxyKeyB)
	proxyIdB := proxyIdOf(proxyKeyB)

	aabbA := bp.trees[typeIndexA].getAABB(proxyIdA)
	aabbB := bp.trees[typeIndexB].getAABB(proxyIdB)
	return AABBOverlaps(aabbA, aabbB)
}

// rebuildTrees rebuilds the enlarged parts of the dynamic and the
// kinematic trees. It corresponds to b2BroadPhase_RebuildTrees in
// src/broad_phase.c.
func (bp *broadPhase) rebuildTrees() {
	bp.trees[DynamicBody].rebuild(false)
	bp.trees[KinematicBody].rebuild(false)
}

// getShapeIndex returns the shape id behind a proxy key.
func (bp *broadPhase) getShapeIndex(proxyKey int) int {
	typeIndex := proxyTypeOf(proxyKey)
	proxyId := proxyIdOf(proxyKey)

	return int(bp.trees[typeIndex].getUserData(proxyId))
}

// validate checks the dynamic and the kinematic trees. Only the tests
// call it. It corresponds to b2ValidateBroadphase in src/broad_phase.c.
func (bp *broadPhase) validate() {
	bp.trees[DynamicBody].validate()
	bp.trees[KinematicBody].validate()
}

// validateNoEnlarged checks that no tree keeps an enlarged node. Only the
// tests call it. It corresponds to b2ValidateNoEnlarged in
// src/broad_phase.c.
func (bp *broadPhase) validateNoEnlarged() {
	for j := range BodyTypeCount {
		bp.trees[j].validateNoEnlarged()
	}
}
