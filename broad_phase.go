package b2

import "math/bits"

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

// crossSeedCount bounds the breadth-first seeds of one cross-tree search.
// It corresponds to B2_CROSS_SEED_COUNT in src/broad_phase.c.
const crossSeedCount = 64

// candidateBatch is the candidate pair batch of one pair search. It
// corresponds to B2_CANDIDATE_BATCH in src/broad_phase.c.
const candidateBatch = 32

// nodePair is a pair of node copies from two trees. It corresponds to
// b2NodePair in src/broad_phase.c.
type nodePair struct {
	a treeNode
	b treeNode
}

// indexPair is a pair of children pairs, one in each array. It
// corresponds to b2IndexPair in src/broad_phase.c.
type indexPair struct {
	a, b int
}

// candidatePair corresponds to b2CandidatePair in src/broad_phase.c.
type candidatePair struct {
	shapeIdA int
	shapeIdB int
}

// broadPhase computes the candidate pairs and serves the volume queries.
// It does not persist pairs: it reports the new pairs of the proxies that
// moved and the world consumes them. It corresponds to b2BroadPhase in
// src/broad_phase.h.
type broadPhase struct {
	// trees holds one tree per body type.
	trees [BodyTypeCount]dynamicTree

	// movedSiblings holds the moved sibling pairs gathered from the
	// dynamic tree. The reference takes it from the stack allocator; the
	// port keeps it across steps.
	movedSiblings []int

	// The cross-tree seeds of one pair update: the static seeds first, the
	// kinematic seeds after them.
	crossSeeds      [2 * crossSeedCount]nodePair
	staticSeedCount int

	// pairKeys and tempKeys gather and sort the new pairs of one update.
	pairKeys []uint64
	tempKeys []uint64

	// pairSet tracks the shape pairs that have a contact.
	pairSet hashSet
}

// createBroadPhase initializes the trees and the pair set. It corresponds
// to b2CreateBroadPhase in src/broad_phase.c.
func createBroadPhase(bp *broadPhase) {
	bp.pairSet = createSet(32)

	bp.trees[StaticBody] = createTree(16)
	bp.trees[KinematicBody] = createTree(16)
	bp.trees[DynamicBody] = createTree(16)

	bp.movedSiblings = nil
}

// destroyBroadPhase releases the trees and the sets.
func destroyBroadPhase(bp *broadPhase) {
	for i := range BodyTypeCount {
		destroyTree(&bp.trees[i])
	}

	destroySet(&bp.pairSet)

	*bp = broadPhase{}
}

// createProxy inserts a shape box in the tree of its body type and returns
// the proxy key. A non-static proxy, or a forced one, is marked moved so
// the next update finds its pairs. It corresponds to
// b2BroadPhase_CreateProxy in src/broad_phase.c.
func (bp *broadPhase) createProxy(proxyType BodyType, aabb AABB, categoryBits uint64, shapeIndex int, forcePairCreation bool) int {
	if proxyType < 0 || proxyType >= BodyTypeCount {
		panic("dbox2d: the proxy type is out of range")
	}

	mark := proxyType != StaticBody || forcePairCreation

	proxyId := bp.trees[proxyType].createProxyInternal(aabb, categoryBits, uint64(shapeIndex), mark)
	return proxyKeyOf(proxyId, proxyType)
}

// destroyProxy removes a proxy from its tree. It corresponds to
// b2BroadPhase_DestroyProxy in src/broad_phase.c.
func (bp *broadPhase) destroyProxy(proxyKey int) {
	proxyType := proxyTypeOf(proxyKey)
	proxyId := proxyIdOf(proxyKey)

	if proxyType < 0 || proxyType >= BodyTypeCount {
		panic("dbox2d: the proxy type is out of range")
	}
	bp.trees[proxyType].destroyProxy(proxyId)
}

// moveProxy reinserts a proxy with a new box and marks it moved. It
// corresponds to b2BroadPhase_MoveProxy in src/broad_phase.c.
func (bp *broadPhase) moveProxy(proxyKey int, aabb AABB) {
	proxyType := proxyTypeOf(proxyKey)
	proxyId := proxyIdOf(proxyKey)

	bp.trees[proxyType].moveProxyInternal(proxyId, aabb, true)
}

// markProxyMovedSerial marks a proxy for the next pair update without
// changing its box. It corresponds to b2BroadPhase_MarkProxyMovedSerial in
// src/broad_phase.h.
func (bp *broadPhase) markProxyMovedSerial(proxyKey int) {
	bp.trees[proxyTypeOf(proxyKey)].markProxyMovedSerial(proxyIdOf(proxyKey))
}

// markProxyMoved sets a proxy box and marks it moved. Workers may call it
// for different proxies at once. It corresponds to
// b2BroadPhase_MarkProxyMoved in src/broad_phase.h.
func (bp *broadPhase) markProxyMoved(proxyKey int, aabb AABB) {
	bp.trees[proxyTypeOf(proxyKey)].markProxyMoved(proxyIdOf(proxyKey), aabb)
}

// gatherMovedSiblings appends the sibling pairs with a moved node,
// skipping the root. It corresponds to b2GatherMovedSiblings in
// src/broad_phase.c.
func gatherMovedSiblings(tree *dynamicTree, pairIndices []int) []int {
	nodes := tree.nodes
	for pair := 2; pair < tree.nodeEnd; pair += 2 {
		// Push when either sibling moved.
		if (nodes[pair].flagIndex|nodes[pair+1].flagIndex)&movedNode != 0 {
			pairIndices = append(pairIndices, pair)
		}
	}
	return pairIndices
}

// pairContext collects the candidate pairs of one worker. It corresponds
// to b2PairContext in src/broad_phase.c.
type pairContext struct {
	w          *world
	pairKeys   *[]uint64
	batch      [candidateBatch]candidatePair
	batchCount int
}

// flushCandidatePairs culls the batched candidates that already have a
// contact, filters the rest, and keeps the pair keys of the survivors. It
// corresponds to b2FlushCandidatePairs in src/broad_phase.c.
func (ctx *pairContext) flushCandidatePairs() {
	w := ctx.w
	bp := &w.broadPhase

	count := ctx.batchCount
	ctx.batchCount = 0

	for i := range count {
		shapeIdA := ctx.batch[i].shapeIdA
		shapeIdB := ctx.batch[i].shapeIdB

		// Cull existing pairs.
		pairKey := shapePairKey(shapeIdA, shapeIdB)
		if bp.pairSet.containsKey(pairKey) {
			continue
		}

		shapeA := &w.shapes[shapeIdA]
		shapeB := &w.shapes[shapeIdB]

		bodyIdA := shapeA.bodyId
		bodyIdB := shapeB.bodyId

		// Are the shapes on the same body?
		if bodyIdA == bodyIdB {
			continue
		}

		// Sensors are handled elsewhere
		if shapeA.sensorIndex != nullIndex || shapeB.sensorIndex != nullIndex {
			continue
		}

		if !shouldShapesCollide(shapeA.filter, shapeB.filter) {
			continue
		}

		if contactRegisters[shapeA.shapeType][shapeB.shapeType].fcn == nil {
			// For example, no segment vs segment collision
			continue
		}

		// Does a joint override collision?
		bodyA := &w.bodies[bodyIdA]
		bodyB := &w.bodies[bodyIdB]
		if !shouldBodiesCollide(w, bodyA, bodyB) {
			continue
		}

		// Custom user filter
		if (shapeA.enableCustomFiltering || shapeB.enableCustomFiltering) && w.customFilterFcn != nil {
			idA := ShapeId{index1: int32(shapeIdA) + 1, world0: w.worldId, generation: shapeA.generation}
			idB := ShapeId{index1: int32(shapeIdB) + 1, world0: w.worldId, generation: shapeB.generation}
			if !w.customFilterFcn(idA, idB) {
				continue
			}
		}

		// The pair passed the gauntlet. A new contact will be created.
		*ctx.pairKeys = append(*ctx.pairKeys, pairKey)
	}
}

// addCandidatePair follows shape index order. It corresponds to
// b2AddCandidatePair in src/broad_phase.c.
func (ctx *pairContext) addCandidatePair(shapeIdA, shapeIdB int) {
	candidate := &ctx.batch[ctx.batchCount]
	candidate.shapeIdA = min(shapeIdA, shapeIdB)
	candidate.shapeIdB = max(shapeIdA, shapeIdB)
	ctx.batchCount++
	if ctx.batchCount == candidateBatch {
		ctx.flushCandidatePairs()
	}
}

// testPair reports whether either node moved and the two overlap. It
// corresponds to b2TestPair in src/broad_phase.c.
func testPair(a, b *treeNode) bool {
	if (a.flagIndex|b.flagIndex)&movedNode == 0 {
		return false
	}
	return AABBOverlaps(a.aabb, b.aabb)
}

// collideProxyAndSubtree collides one leaf against the subtree under a
// children pair. An unmoved leaf only pairs with moved nodes. It
// corresponds to b2CollideProxyAndSubtree in src/broad_phase.c.
func (ctx *pairContext) collideProxyAndSubtree(proxy *treeNode, nodes []treeNode, pair int) {
	proxyMoved := proxy.isMoved()
	box := proxy.aabb
	shapeId := int(proxy.aux)

	var stack [treeStackSize]int
	stackCount := 0
	stack[stackCount] = pair
	stackCount++

	for stackCount > 0 {
		stackCount--
		pair = stack[stackCount]

		for i := range 2 {
			node := &nodes[pair+i]
			if !proxyMoved && !node.isMoved() {
				continue
			}
			if !AABBOverlaps(box, node.aabb) {
				continue
			}

			if node.isLeaf() {
				ctx.addCandidatePair(shapeId, int(node.aux))
			} else {
				if stackCount >= treeStackSize {
					panic("dbox2d: the pair search stack is full")
				}
				stack[stackCount] = node.leftChild()
				stackCount++
			}
		}
	}
}

// expandPair corresponds to b2ExpandPair in src/broad_phase.c.
func (ctx *pairContext) expandPair(arrayA, arrayB []treeNode, nodeA, nodeB *treeNode, stack *[treeStackSize]indexPair, stackCount *int) {
	leafA := nodeA.isLeaf()
	leafB := nodeB.isLeaf()
	if leafA && leafB {
		ctx.addCandidatePair(int(nodeA.aux), int(nodeB.aux))
	} else if leafA {
		ctx.collideProxyAndSubtree(nodeA, arrayB, nodeB.leftChild())
	} else if leafB {
		ctx.collideProxyAndSubtree(nodeB, arrayA, nodeA.leftChild())
	} else {
		if *stackCount >= treeStackSize {
			panic("dbox2d: the pair search stack is full")
		}
		stack[*stackCount] = indexPair{a: nodeA.leftChild(), b: nodeB.leftChild()}
		*stackCount++
	}
}

// collideCrossPairs collides two subtrees against each other. They can
// live in the same tree. It only generates pairs across the subtrees,
// never within one, so self collision has no duplicates: with the full
// binary tree A (B (D E) C (F G)), colliding B and C gives (D,F) (D,G)
// (E,F) and (E,G), then B gives (D,E) and C gives (F,G). Moved flags
// propagate to the root, so self collision takes the moved internal nodes
// and collides their subtrees together (Real-time collision detection
// 6.3.2). It scales linearly, against linear times log for querying every
// moved proxy. It corresponds to b2CollideCrossPairs in
// src/broad_phase.c.
func (ctx *pairContext) collideCrossPairs(arrayA, arrayB []treeNode, subtreeA, subtreeB *treeNode) {
	var stack [treeStackSize]indexPair
	stackCount := 0

	// Seed the stack.
	if testPair(subtreeA, subtreeB) {
		ctx.expandPair(arrayA, arrayB, subtreeA, subtreeB, &stack, &stackCount)
	}

	for stackCount > 0 {
		stackCount--
		pair := stack[stackCount]

		// The four lanes are (a0,b0) (a0,b1) (a1,b0) (a1,b1); a lane needs
		// a moved side and an overlap.
		pairA := arrayA[pair.a : pair.a+2]
		pairB := arrayB[pair.b : pair.b+2]

		mask := 0
		if pairA[0].isMoved() {
			mask |= 0x3
		}
		if pairA[1].isMoved() {
			mask |= 0xC
		}
		if pairB[0].isMoved() {
			mask |= 0x5
		}
		if pairB[1].isMoved() {
			mask |= 0xA
		}

		for mask != 0 {
			lane := bits.TrailingZeros32(uint32(mask))
			mask &= mask - 1

			nodeA := &pairA[lane>>1]
			nodeB := &pairB[lane&1]
			if AABBOverlaps(nodeA.aabb, nodeB.aabb) {
				ctx.expandPair(arrayA, arrayB, nodeA, nodeB, &stack, &stackCount)
			}
		}
	}
}

// selfPairsTask collides the subtrees of the moved sibling pairs of the
// dynamic tree. It corresponds to b2SelfPairsTask in src/broad_phase.c.
func selfPairsTask(startIndex, endIndex, workerIndex int, context *stepContext) {
	w := context.world
	bp := &w.broadPhase
	nodes := bp.trees[DynamicBody].nodes
	siblingIndices := bp.movedSiblings

	ctx := pairContext{w: w, pairKeys: &w.taskContexts[workerIndex].pairKeys}

	for i := startIndex; i < endIndex; i++ {
		nodeIndex := siblingIndices[i]
		ctx.collideCrossPairs(nodes, nodes, &nodes[nodeIndex], &nodes[nodeIndex+1])
	}

	ctx.flushCandidatePairs()
}

// gatherCrossSeeds runs a serial cross-tree breadth-first search until
// the queue is full, then returns the queued pairs as seeds for a
// parallel search. It corresponds to b2GatherCrossSeeds in
// src/broad_phase.c.
func gatherCrossSeeds(treeA, treeB *dynamicTree, seeds []nodePair) int {
	nodesA := treeA.nodes
	nodesB := treeB.nodes

	// Breadth-first search from the two roots as one pair. An empty tree
	// has a sentinel root that survives nothing.
	var queue [2 * crossSeedCount]nodePair
	mask := 2*crossSeedCount - 1
	head := 0
	tail := 0

	rootA := &nodesA[rootNode]
	rootB := &nodesB[rootNode]
	if testPair(rootA, rootB) {
		queue[tail&mask] = nodePair{a: *rootA, b: *rootB}
		tail++
	}

	seedCount := 0
	for head < tail && seedCount+(tail-head)+3 < crossSeedCount {
		pair := queue[head&mask]
		head++

		if pair.a.isLeaf() || pair.b.isLeaf() {
			seeds[seedCount] = pair
			seedCount++
			continue
		}

		a := nodesA[pair.a.leftChild():]
		b := nodesB[pair.b.leftChild():]

		// Nodes have two children each, so four combinations.
		for i := range 2 {
			for j := range 2 {
				if testPair(&a[i], &b[j]) {
					queue[tail&mask] = nodePair{a: a[i], b: b[j]}
					tail++
				}
			}
		}
	}

	for head < tail {
		seeds[seedCount] = queue[head&mask]
		seedCount++
		head++
	}

	return seedCount
}

// crossPairsTask collides the dynamic tree against the static and the
// kinematic trees from the gathered seeds. It corresponds to
// b2CrossPairsTask in src/broad_phase.c.
func crossPairsTask(startIndex, endIndex, workerIndex int, context *stepContext) {
	w := context.world
	bp := &w.broadPhase
	staticNodes := bp.trees[StaticBody].nodes
	kinematicNodes := bp.trees[KinematicBody].nodes
	dynamicNodes := bp.trees[DynamicBody].nodes

	ctx := pairContext{w: w, pairKeys: &w.taskContexts[workerIndex].pairKeys}

	for i := startIndex; i < endIndex; i++ {
		nodesB := kinematicNodes
		if i < bp.staticSeedCount {
			nodesB = staticNodes
		}
		seed := &bp.crossSeeds[i]
		ctx.collideCrossPairs(dynamicNodes, nodesB, &seed.a, &seed.b)
	}

	ctx.flushCandidatePairs()
}

// radixSortKeys sorts the keys with an LSD radix sort, using tempKeys of
// the same length as scratch. It skips a digit that every key shares,
// which is common for shape pair keys. It corresponds to b2RadixSortKeys
// in src/broad_phase.c.
func radixSortKeys(keys, tempKeys []uint64) {
	count := len(keys)
	if count == 0 {
		return
	}

	var digitCounts [8][256]int
	for _, key := range keys {
		for digit := range 8 {
			digitCounts[digit][(key>>(8*digit))&0xFF]++
		}
	}

	source := keys
	target := tempKeys[:count]

	for digit := range 8 {
		shift := 8 * digit
		offsets := &digitCounts[digit]

		if offsets[(source[0]>>shift)&0xFF] == count {
			// All keys have the same value for this digit.
			continue
		}

		sum := 0
		for i := range 256 {
			digitCount := offsets[i]
			offsets[i] = sum
			sum += digitCount
		}

		for _, key := range source {
			d := (key >> shift) & 0xFF
			target[offsets[d]] = key
			offsets[d]++
		}

		// Swap for the next pass
		source, target = target, source
	}

	if &source[0] != &keys[0] {
		copy(keys, source)
	}
}

// updateBroadPhasePairs finds the new pairs of the moved proxies, starts
// the tree rebuild, and creates the contacts in pair key order. It
// corresponds to b2UpdateBroadPhasePairs in src/broad_phase.c.
func updateBroadPhasePairs(w *world, context *stepContext) {
	bp := &w.broadPhase
	context.world = w

	needUpdate := bp.trees[StaticBody].hasMoved()
	needUpdate = needUpdate || bp.trees[KinematicBody].needsRebuild()
	needUpdate = needUpdate || bp.trees[DynamicBody].needsRebuild()

	if !needUpdate {
		return
	}

	for i := range w.workerCount {
		w.taskContexts[i].pairKeys = w.taskContexts[i].pairKeys[:0]
	}

	// Generate pairs by querying the dynamic body tree against itself and
	// against the kinematic and static trees.
	{
		// Get the sibling pairs of the dynamic body tree that have moved.
		dynamicTree := &bp.trees[DynamicBody]
		bp.movedSiblings = gatherMovedSiblings(dynamicTree, bp.movedSiblings[:0])
		dynamicMoveCount := len(bp.movedSiblings)

		// Get seeds for colliding against the static and kinematic trees.
		staticSeedCount := gatherCrossSeeds(dynamicTree, &bp.trees[StaticBody], bp.crossSeeds[:])
		kinematicSeedCount := gatherCrossSeeds(dynamicTree, &bp.trees[KinematicBody], bp.crossSeeds[staticSeedCount:])
		bp.staticSeedCount = staticSeedCount
		crossMoveCount := staticSeedCount + kinematicSeedCount

		// Collide the dynamic body tree against the static and kinematic
		// trees.
		if crossMoveCount > 0 {
			w.taskCount++
			w.executor.parallelFor(crossMoveCount, 1, crossPairsTask, context)
		}

		// Collide the dynamic body tree against itself.
		if dynamicMoveCount > 0 {
			w.taskCount++
			w.executor.parallelFor(dynamicMoveCount, 64, selfPairsTask, context)
		}
	}

	bp.trees[StaticBody].clearMoved()

	// Update stale trees, in parallel with the rest of the step.
	w.taskCount++
	w.treeTask.begin(w)

	// Pairs arrive in deterministic order but scrambled relative to body
	// and shape order; sorting them here improves solver performance.
	pairKeys := bp.pairKeys[:0]
	for i := range w.workerCount {
		pairKeys = append(pairKeys, w.taskContexts[i].pairKeys...)
	}
	bp.pairKeys = pairKeys

	pairCount := len(pairKeys)
	if pairCount > 1 {
		if cap(bp.tempKeys) < pairCount {
			bp.tempKeys = make([]uint64, pairCount)
		}
		radixSortKeys(pairKeys, bp.tempKeys[:pairCount])
	}

	for _, key := range pairKeys {
		shapeIdA := int(key >> 32)
		shapeIdB := int(key & 0xFFFFFFFF)
		createContact(w, &w.shapes[shapeIdA], &w.shapes[shapeIdB])
	}

	validateSolverSetsDebug(w)
}

// rebuildTrees rebuilds the stale parts of the dynamic and the kinematic
// trees. It corresponds to b2UpdateTreesTask in src/broad_phase.c.
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

// validate checks the dynamic and the kinematic trees. It corresponds to
// b2ValidateBroadphase in src/broad_phase.c.
func (bp *broadPhase) validate() {
	bp.trees[DynamicBody].validate()
	bp.trees[KinematicBody].validate()
}

// validateNoMoved checks that no tree keeps a moved node. It corresponds
// to b2ValidateNoEnlarged in src/broad_phase.c.
func (bp *broadPhase) validateNoMoved() {
	for j := range BodyTypeCount {
		bp.trees[j].validateNoMoved()
	}
}
