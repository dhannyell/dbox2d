package b2

import (
	"sync/atomic"
	"unsafe"
)

// treeStackSize is the traversal stack of every tree walk. It corresponds
// to B2_TREE_STACK_SIZE in src/dynamic_tree.h.
const treeStackSize = 512

// The flagIndex bits of src/dynamic_tree.h. A moved node needs pairs and
// may need a rebuild. The mark comes from solver movement, a joint that
// disables collision, proxy creation, and setting a body transform.
const (
	movedNode     uint32 = 1 << 30
	leafNode      uint32 = 1 << 31
	nodeIndexMask uint32 = 0xFFFFFFFF &^ (movedNode | leafNode)

	// emptyNode marks a free node and the node beside a leaf root. It
	// passes as a leaf so that internal node processing skips it.
	emptyNode = nodeIndexMask | leafNode

	rootNode = 0
)

// treeNode is a node of the dynamic tree. Siblings sit together at an even
// index. The root is at index zero and index one is always empty. It
// corresponds to b2TreeNode in src/dynamic_tree.h.
type treeNode struct {
	// aabb is the node bounding box.
	aabb AABB

	// flagIndex holds the leaf flag in bit 31, the moved flag in bit 30,
	// and in the low bits the index of the children pair or, for a leaf,
	// the proxy id.
	flagIndex uint32

	// aux is the height of an internal node or the shape index of a leaf,
	// truncated from the proxy user data. The reference overlays the two
	// in a union.
	aux int32
}

func (n *treeNode) isLeaf() bool       { return n.flagIndex&leafNode == leafNode }
func (n *treeNode) isMoved() bool      { return n.flagIndex&movedNode == movedNode }
func (n *treeNode) isEmpty() bool      { return n.flagIndex == emptyNode }
func (n *treeNode) leftChild() int     { return int(n.flagIndex & nodeIndexMask) }
func (n *treeNode) proxyId() int       { return int(n.flagIndex & nodeIndexMask) }
func (n *treeNode) setLeftChild(p int) { n.flagIndex = n.flagIndex&^nodeIndexMask | uint32(p) }

// height returns zero for a leaf. It corresponds to b2GetNodeHeight.
func (n *treeNode) height() int {
	if n.isLeaf() {
		return 0
	}
	return int(n.aux)
}

// makeEmptyNode returns the inverted box that overlaps nothing. It
// corresponds to b2MakeEmptyNode.
func makeEmptyNode() treeNode {
	inf := qInfinity()
	return treeNode{
		aabb: AABB{
			LowerBound: Vec2{X: inf, Y: inf},
			UpperBound: Vec2{X: inf.Neg(), Y: inf.Neg()},
		},
		flagIndex: emptyNode,
	}
}

// makeInternalNode returns the node above a children pair. It corresponds
// to b2MakeInternalNode.
func makeInternalNode(nodes []treeNode, pair int) treeNode {
	c1 := &nodes[pair]
	c2 := &nodes[pair+1]
	return treeNode{
		aabb:      AABBUnion(c1.aabb, c2.aabb),
		flagIndex: uint32(pair) | (c1.flagIndex|c2.flagIndex)&movedNode,
		aux:       int32(1 + max(c1.height(), c2.height())),
	}
}

// makeLeafNode corresponds to b2MakeLeafNode.
func makeLeafNode(aabb AABB, proxyId int, userData uint64, moved bool) treeNode {
	flags := uint32(proxyId) | leafNode
	if moved {
		flags |= movedNode
	}
	return treeNode{aabb: aabb, flagIndex: flags, aux: int32(userData)}
}

// rootPair returns the pair a walk starts from: the root pair when the
// root is a leaf, else the children of the root. It corresponds to
// b2GetRootPair.
func rootPair(nodes []treeNode) int {
	root := &nodes[rootNode]
	if root.isLeaf() {
		return rootNode
	}
	return root.leftChild()
}

// treeProxy is the cold leaf data, split from the node array. It
// corresponds to b2TreeProxy in src/dynamic_tree.h.
type treeProxy struct {
	// userData is an index; the world uses it as the shape index.
	userData uint64

	// categoryBits serve collision filtering.
	categoryBits uint64

	// node is the leaf node, or nullIndex for a free proxy.
	node int32

	// next is the free list link.
	next int32
}

// dynamicTree is a binary AABB tree over proxies. Nodes are relocatable,
// so the tree holds indices instead of pointers. It corresponds to
// b2DynamicTree in src/dynamic_tree.h.
type dynamicTree struct {
	// nodes is the node array. Its length is the node capacity. Free
	// pairs leave holes below nodeEnd.
	nodes []treeNode

	// parents holds the parent of each node. The free pair list is
	// chained through the parent of the first node of each free pair.
	parents []int32

	// proxies is the proxy pool. Its length is the proxy capacity.
	proxies []treeProxy

	// nodeEnd bounds every allocated node.
	nodeEnd int

	// pairFreeList is the head of the free pairs below nodeEnd.
	pairFreeList int

	proxyCount    int
	proxyFreeList int

	// swapNodes is the rebuild target, as long as nodes once allocated.
	swapNodes []treeNode

	// The rebuild scratch. Its capacity is len(leafIndices). The leaves
	// may be proxies or retained subtrees. The centers serve the median
	// split; the boxes and the bins serve the surface area heuristic.
	leafIndices []int
	leafNodes   []treeNode
	leafBoxes   []AABB
	leafCenters []Vec2
	binIndices  []int

	// dfsOrdered holds when every child sits above its parent. The
	// rebuild restores it; insertion can break it. The sweep refit needs
	// it.
	dfsOrdered bool
}

// createTree initializes the tree. It corresponds to
// b2CreateDynamicTreeInternal in src/dynamic_tree.c.
func createTree(proxyCapacity int) dynamicTree {
	capacity := max(proxyCapacity, 16)

	tree := dynamicTree{}

	// A tree of n proxies has 2n - 1 nodes plus the empty node beside the
	// root.
	nodeCapacity := 2 * capacity
	tree.nodes = make([]treeNode, nodeCapacity)
	tree.parents = make([]int32, nodeCapacity)
	tree.pairFreeList = nullIndex

	// The root and the empty node always exist.
	tree.nodes[rootNode] = makeEmptyNode()
	tree.nodes[rootNode+1] = makeEmptyNode()
	tree.parents[rootNode] = nullIndex
	tree.parents[rootNode+1] = nullIndex

	// This is the bump index.
	tree.nodeEnd = 2
	tree.dfsOrdered = true

	tree.proxies = make([]treeProxy, capacity)
	linkFreeProxies(tree.proxies, 0)
	tree.proxyFreeList = 0

	return tree
}

// linkFreeProxies chains the proxies from start on into a free list.
func linkFreeProxies(proxies []treeProxy, start int) {
	n := len(proxies)
	for i := start; i < n-1; i++ {
		proxies[i].node = nullIndex
		proxies[i].next = int32(i + 1)
	}
	proxies[n-1].node = nullIndex
	proxies[n-1].next = nullIndex
}

// destroyTree releases the tree.
func destroyTree(tree *dynamicTree) {
	*tree = dynamicTree{}
}

// byteCount corresponds to b2DynamicTree_GetByteCount in
// src/dynamic_tree.c.
func (tree *dynamicTree) byteCount() int {
	var t dynamicTree
	var n treeNode
	var p treeProxy
	var i int
	var box AABB
	var v Vec2
	nodeSize := int(unsafe.Sizeof(n))
	size := int(unsafe.Sizeof(t))
	size += len(tree.nodes) * (nodeSize + 4)
	size += len(tree.proxies) * int(unsafe.Sizeof(p))
	if tree.swapNodes != nil {
		size += len(tree.nodes) * nodeSize
	}
	rebuildCapacity := len(tree.leafIndices)
	size += rebuildCapacity * (2*int(unsafe.Sizeof(i)) + nodeSize + int(unsafe.Sizeof(box)) + int(unsafe.Sizeof(v)))
	return size
}

// allocateSiblingPair takes a pair from the free list, else from the bump
// pointer. A rebuild empties the free list. It corresponds to
// b2AllocateSiblingPair in src/dynamic_tree.c.
func allocateSiblingPair(tree *dynamicTree) int {
	if tree.pairFreeList != nullIndex {
		pair := tree.pairFreeList
		tree.pairFreeList = int(tree.parents[pair])
		return pair
	}

	// Extend the bump allocator if needed.
	if tree.nodeEnd+2 > len(tree.nodes) {
		oldCapacity := len(tree.nodes)
		newCapacity := oldCapacity + oldCapacity>>1
		newCapacity += newCapacity & 1

		nodes := make([]treeNode, newCapacity)
		copy(nodes, tree.nodes)
		tree.nodes = nodes

		parents := make([]int32, newCapacity)
		copy(parents, tree.parents)
		tree.parents = parents

		// The spare has to match; the rebuild allocates it again.
		tree.swapNodes = nil
	}

	pair := tree.nodeEnd
	tree.nodeEnd += 2
	return pair
}

// freePair corresponds to b2FreePair in src/dynamic_tree.c.
func freePair(tree *dynamicTree, pair int) {
	if pair&1 != 0 || pair < 2 || pair >= tree.nodeEnd {
		panic("dbox2d: the freed tree pair is not allocated")
	}
	tree.nodes[pair] = makeEmptyNode()
	tree.nodes[pair+1] = makeEmptyNode()
	tree.parents[pair] = int32(tree.pairFreeList)
	tree.parents[pair+1] = nullIndex
	tree.pairFreeList = pair
}

// allocateProxy takes a proxy from the pool and grows the pool when the
// free list is empty. It corresponds to b2AllocateProxy in
// src/dynamic_tree.c.
func allocateProxy(tree *dynamicTree) int {
	if tree.proxyFreeList == nullIndex {
		if tree.proxyCount != len(tree.proxies) {
			panic("dbox2d: the tree proxy free list is corrupt")
		}

		// A restored pool can have capacity one, so guarantee strict
		// growth.
		oldCapacity := len(tree.proxies)
		proxies := make([]treeProxy, oldCapacity+max(oldCapacity>>1, 1))
		copy(proxies, tree.proxies)
		tree.proxies = proxies
		linkFreeProxies(tree.proxies, oldCapacity)
		tree.proxyFreeList = oldCapacity
	}

	proxyIndex := tree.proxyFreeList
	tree.proxyFreeList = int(tree.proxies[proxyIndex].next)
	tree.proxies[proxyIndex] = treeProxy{node: nullIndex, next: nullIndex}
	tree.proxyCount++
	return proxyIndex
}

// freeProxy returns a proxy to the pool. It corresponds to b2FreeProxy in
// src/dynamic_tree.c.
func freeProxy(tree *dynamicTree, proxyId int) {
	checkProxyId(tree, proxyId)
	if tree.proxyCount <= 0 {
		panic("dbox2d: the tree has no proxy to free")
	}
	tree.proxies[proxyId].node = nullIndex
	tree.proxies[proxyId].next = int32(tree.proxyFreeList)
	tree.proxyFreeList = proxyId
	tree.proxyCount--
}

// checkProxyId panics on an id outside the proxy pool.
func checkProxyId(tree *dynamicTree, proxyId int) {
	if proxyId < 0 || proxyId >= len(tree.proxies) {
		panic("dbox2d: the proxy id is out of range")
	}
}

// linkChildren tells what hangs below a node that landed at a new index.
// It corresponds to b2LinkChildren in src/dynamic_tree.c.
func linkChildren(tree *dynamicTree, nodeIndex int) {
	node := &tree.nodes[nodeIndex]
	if node.isLeaf() {
		tree.proxies[node.proxyId()].node = int32(nodeIndex)
	} else {
		pair := node.leftChild()
		tree.parents[pair] = int32(nodeIndex)
		tree.parents[pair+1] = int32(nodeIndex)
	}
}

// isNodeOrdered holds when the children of the node sit above it, which
// the sweep refit needs. It corresponds to b2IsNodeOrdered.
func isNodeOrdered(nodes []treeNode, nodeIndex int) bool {
	node := &nodes[nodeIndex]
	return node.isLeaf() || nodeIndex < node.leftChild()
}

// findBestSibling picks the sibling of a new leaf with a greedy descent
// under the surface area heuristic. It corresponds to b2FindBestSibling in
// src/dynamic_tree.c.
//
// We have three nodes A-(B,C) and want to add a leaf D, there are three
// choices.
// 1: make a new parent for A and D : E-(A-(B,C), D)
// 2: associate D with B
//
//	a: B is a leaf : A-(E-(B,D), C)
//	b: B is an internal node: A-(B{D},C)
//
// 3: associate D with C
//
//	a: C is a leaf : A-(B, E-(C,D))
//	b: C is an internal node: A-(B, C{D})
//
// All of these have a clear cost except when B or C is an internal node.
// Hence we need to be greedy.
func findBestSibling(tree *dynamicTree, boxD AABB) int {
	nodes := tree.nodes
	centerD := AABBCenter(boxD)
	areaD := perimeter(boxD)

	nodeIndex := rootNode
	if nodes[nodeIndex].isLeaf() {
		return nodeIndex
	}

	rootBox := nodes[nodeIndex].aabb

	// Area of current node
	areaBase := perimeter(rootBox)

	// Area of inflated node
	directCost := perimeter(AABBUnion(rootBox, boxD))
	inheritedCost := QZero()

	bestSibling := nodeIndex
	bestCost := directCost

	zero := QZero()

	// D-009: the seed is the largest representable value, which no
	// perimeter reaches.
	maxCost := QMaxValue()

	// Descend the tree, following a single greedy path.
	for {
		child1 := nodes[nodeIndex].leftChild()
		child2 := child1 + 1

		// Cost of creating a new parent for this node and the new leaf
		cost := directCost.Add(inheritedCost)

		// Sometimes there are multiple identical costs within tolerance.
		// This breaks the ties using the centroid distance.
		if cost.Less(bestCost) {
			bestSibling = nodeIndex
			bestCost = cost
		}

		// Inheritance cost seen by children
		inheritedCost = inheritedCost.Add(directCost.Sub(areaBase))

		leaf1 := nodes[child1].isLeaf()
		leaf2 := nodes[child2].isLeaf()

		// Cost of descending into child 1
		lowerCost1 := maxCost
		box1 := nodes[child1].aabb
		directCost1 := perimeter(AABBUnion(box1, boxD))
		area1 := zero
		if leaf1 {
			// Child 1 is a leaf
			// Cost of creating new node and increasing area of node P
			cost1 := directCost1.Add(inheritedCost)
			if cost1.Less(bestCost) {
				bestSibling = child1
				bestCost = cost1
			}
		} else {
			// Child 1 is an internal node
			area1 = perimeter(box1)

			// Lower bound cost of inserting under child 1. The minimum accounts for two possibilities:
			// 1. Child1 could be the sibling with cost1 = inheritedCost + directCost1
			// 2. A descendant of child1 could be the sibling with the lower bound cost of
			//       cost1 = inheritedCost + (directCost1 - area1) + areaD
			// This minimum here leads to the minimum of these two costs.
			lowerCost1 = inheritedCost.Add(directCost1).Add(areaD.Sub(area1).Min(zero))
		}

		// Cost of descending into child 2
		lowerCost2 := maxCost
		box2 := nodes[child2].aabb
		directCost2 := perimeter(AABBUnion(box2, boxD))
		area2 := zero
		if leaf2 {
			cost2 := directCost2.Add(inheritedCost)
			if cost2.Less(bestCost) {
				bestSibling = child2
				bestCost = cost2
			}
		} else {
			area2 = perimeter(box2)
			lowerCost2 = inheritedCost.Add(directCost2).Add(areaD.Sub(area2).Min(zero))
		}

		if leaf1 && leaf2 {
			break
		}

		// Can the cost possibly be decreased?
		if !lowerCost1.Less(bestCost) && !lowerCost2.Less(bestCost) {
			break
		}

		if lowerCost1.Eq(lowerCost2) && !leaf1 {
			if !lowerCost1.Less(maxCost) || !lowerCost2.Less(maxCost) {
				panic("dbox2d: the sibling search compares two unbounded costs")
			}

			// No clear choice based on lower bound surface area. This can happen when both
			// children fully contain D. Fall back to node distance.
			d1 := AABBCenter(box1).Sub(centerD)
			d2 := AABBCenter(box2).Sub(centerD)
			lowerCost1 = d1.LenSq()
			lowerCost2 = d2.LenSq()
		}

		// Descend
		if lowerCost1.Less(lowerCost2) && !leaf1 {
			nodeIndex = child1
			areaBase = area1
			directCost = directCost1
		} else {
			nodeIndex = child2
			areaBase = area2
			directCost = directCost2
		}
	}

	return bestSibling
}

// swapTreeNodes swaps a child of A with a grandchild under the other
// child. Swapping B with G: (A (B (D E) C (F G)) -> (A (G) C (F B (D E)).
// It corresponds to b2SwapNodes in src/dynamic_tree.c.
func swapTreeNodes(tree *dynamicTree, iDown, iUp int) {
	nodes := tree.nodes
	nodes[iDown], nodes[iUp] = nodes[iUp], nodes[iDown]
	linkChildren(tree, iDown)
	linkChildren(tree, iUp)

	if !isNodeOrdered(nodes, iDown) || !isNodeOrdered(nodes, iUp) {
		tree.dfsOrdered = false
	}

	// The sibling of the node that went down holds a different subtree
	// now.
	iC := iDown ^ 1
	nodes[iC] = makeInternalNode(nodes, nodes[iC].leftChild())
}

// rotateNodes performs a left or right rotation if node A is imbalanced.
// Tree: (A (B (D E) C (F G)). It corresponds to b2RotateNodes in
// src/dynamic_tree.c.
func rotateNodes(tree *dynamicTree, iA int) {
	nodes := tree.nodes
	A := &nodes[iA]
	if A.isLeaf() {
		panic("dbox2d: the tree rotates a leaf")
	}

	iB := A.leftChild()
	iC := iB + 1
	B := &nodes[iB]
	C := &nodes[iC]

	leafB := B.isLeaf()
	leafC := C.isLeaf()
	if leafB && leafC {
		return
	}

	bestDown := nullIndex
	bestUp := nullIndex
	bestDelta := QZero()

	if !leafC {
		// Swap B with F or G
		iF := C.leftChild()
		iG := iF + 1
		areaC := perimeter(C.aabb)

		// B <-> F then C (B G)
		deltaBF := perimeter(AABBUnion(B.aabb, nodes[iG].aabb)).Sub(areaC)
		if deltaBF.Less(bestDelta) {
			bestDown = iB
			bestUp = iF
			bestDelta = deltaBF
		}

		// B <-> G then C (F B)
		deltaBG := perimeter(AABBUnion(B.aabb, nodes[iF].aabb)).Sub(areaC)
		if deltaBG.Less(bestDelta) {
			bestDown = iB
			bestUp = iG
			bestDelta = deltaBG
		}
	}

	if !leafB {
		// Swap C with D or E
		iD := B.leftChild()
		iE := iD + 1
		areaB := perimeter(B.aabb)

		// C <-> D then B (C E)
		deltaCD := perimeter(AABBUnion(C.aabb, nodes[iE].aabb)).Sub(areaB)
		if deltaCD.Less(bestDelta) {
			bestDown = iC
			bestUp = iD
			bestDelta = deltaCD
		}

		// C <-> E then B (D C)
		deltaCE := perimeter(AABBUnion(C.aabb, nodes[iD].aabb)).Sub(areaB)
		if deltaCE.Less(bestDelta) {
			bestDown = iC
			bestUp = iE
		}
	}

	if bestDown != nullIndex {
		swapTreeNodes(tree, bestDown, bestUp)
	}
}

// insertLeaf corresponds to b2InsertLeaf in src/dynamic_tree.c.
func insertLeaf(tree *dynamicTree, aabb AABB, proxyId int, moved bool, shouldRotate bool) {
	proxy := &tree.proxies[proxyId]
	leaf := makeLeafNode(aabb, proxyId, proxy.userData, moved)

	if tree.nodes[rootNode].isEmpty() {
		tree.nodes[rootNode] = leaf
		proxy.node = rootNode
		return
	}

	sibling := findBestSibling(tree, aabb)

	// The sibling's position becomes the new parent. The sibling moves
	// down into a new pair beside the leaf, which puts it below its own
	// children when it is internal.
	pair := allocateSiblingPair(tree)
	nodes := tree.nodes
	parents := tree.parents

	nodes[pair] = nodes[sibling]
	nodes[pair+1] = leaf
	parents[pair] = int32(sibling)
	parents[pair+1] = int32(sibling)
	linkChildren(tree, pair)
	tree.proxies[proxyId].node = int32(pair + 1)

	nodes[sibling] = makeInternalNode(nodes, pair)

	if !isNodeOrdered(nodes, sibling) || !isNodeOrdered(nodes, pair) {
		tree.dfsOrdered = false
	}

	// Walk back up the tree refitting ancestors, the root included.
	index := sibling
	for index != nullIndex {
		if shouldRotate {
			rotateNodes(tree, index)
		}

		nodes[index] = makeInternalNode(nodes, nodes[index].leftChild())
		index = int(parents[index])
	}
}

// removeLeaf corresponds to b2RemoveLeaf in src/dynamic_tree.c.
func removeLeaf(tree *dynamicTree, proxyId int) {
	nodes := tree.nodes
	parents := tree.parents

	leaf := int(tree.proxies[proxyId].node)
	if leaf < 0 || leaf >= tree.nodeEnd || !nodes[leaf].isLeaf() || nodes[leaf].proxyId() != proxyId {
		panic("dbox2d: the proxy is not a leaf of the tree")
	}

	if leaf == rootNode {
		nodes[rootNode] = makeEmptyNode()
		return
	}

	// The sibling takes the parent's position and the pair is freed. This
	// keeps the order, the sibling only moves down.
	parent := int(parents[leaf])
	nodes[parent] = nodes[leaf^1]
	linkChildren(tree, parent)
	freePair(tree, leaf&^1)

	// Update ancestors.
	index := int(parents[parent])
	for index != nullIndex {
		nodes[index] = makeInternalNode(nodes, nodes[index].leftChild())
		index = int(parents[index])
	}
}

// checkTreeExtent panics on an invalid box or on a box wider than huge.
func checkTreeExtent(aabb AABB) {
	if !IsValidAABB(aabb) {
		panic("dbox2d: the proxy box is not valid")
	}
	if !aabb.UpperBound.X.Sub(aabb.LowerBound.X).Less(Huge) || !aabb.UpperBound.Y.Sub(aabb.LowerBound.Y).Less(Huge) {
		panic("dbox2d: the proxy box is wider than the huge range")
	}
}

// createProxyInternal inserts a leaf and returns its proxy id. It
// corresponds to b2CreateTreeProxyInternal in src/dynamic_tree.c.
func (tree *dynamicTree) createProxyInternal(aabb AABB, categoryBits uint64, userData uint64, markMoved bool) int {
	if !IsValidAABB(aabb) {
		panic("dbox2d: the proxy box is not valid")
	}

	proxyId := allocateProxy(tree)

	proxy := &tree.proxies[proxyId]
	proxy.categoryBits = categoryBits
	proxy.userData = userData

	shouldRotate := true
	insertLeaf(tree, aabb, proxyId, markMoved, shouldRotate)

	return proxyId
}

// createProxy corresponds to b2CreateTreeProxy in src/dynamic_tree.c.
func (tree *dynamicTree) createProxy(aabb AABB, categoryBits uint64, userData uint64) int {
	return tree.createProxyInternal(aabb, categoryBits, userData, false)
}

// destroyProxy corresponds to b2DestroyTreeProxy in src/dynamic_tree.c.
func (tree *dynamicTree) destroyProxy(proxyId int) {
	checkProxyId(tree, proxyId)
	removeLeaf(tree, proxyId)
	freeProxy(tree, proxyId)
}

func (tree *dynamicTree) getProxyCount() int {
	return tree.proxyCount
}

// moveProxyInternal reinserts a proxy with a new box. It corresponds to
// b2DynamicTree_MoveProxyInternal in src/dynamic_tree.c.
func (tree *dynamicTree) moveProxyInternal(proxyId int, aabb AABB, markMoved bool) {
	checkTreeExtent(aabb)
	checkProxyId(tree, proxyId)

	removeLeaf(tree, proxyId)

	shouldRotate := false
	insertLeaf(tree, aabb, proxyId, markMoved, shouldRotate)
}

// moveProxy corresponds to b2DynamicTree_MoveProxy in src/dynamic_tree.c.
func (tree *dynamicTree) moveProxy(proxyId int, aabb AABB) {
	tree.moveProxyInternal(proxyId, aabb, false)
}

// enlargeProxy grows a leaf in place and its ancestors as needed, marking
// the path to the root as moved. The solver uses it for bullets. The new
// box must not fit in the old one. It corresponds to
// b2DynamicTree_EnlargeProxy in src/dynamic_tree.c.
func (tree *dynamicTree) enlargeProxy(proxyId int, aabb AABB) {
	checkTreeExtent(aabb)
	checkProxyId(tree, proxyId)

	nodes := tree.nodes
	parents := tree.parents

	index := int(tree.proxies[proxyId].node)
	node := &nodes[index]
	if !node.isLeaf() {
		panic("dbox2d: the enlarged proxy is not a leaf")
	}

	// Caller must ensure this
	if AABBContains(node.aabb, aabb) {
		panic("dbox2d: the enlarged box fits in the proxy box")
	}

	node.aabb = aabb
	node.flagIndex |= movedNode

	index = int(parents[index])
	for index != nullIndex {
		node = &nodes[index]
		changed := enlargeAABB(&node.aabb, aabb)

		// This is marked to ensure the root is marked in this loop or the
		// one below.
		node.flagIndex |= movedNode

		index = int(parents[index])

		if !changed {
			break
		}
	}

	// Mark all the way up to the root.
	for index != nullIndex {
		node = &nodes[index]
		if node.flagIndex&movedNode != 0 {
			// Early out because this ancestor was previously ascended and
			// marked as moved.
			break
		}

		node.flagIndex |= movedNode
		index = int(parents[index])
	}
}

func (tree *dynamicTree) setCategoryBits(proxyId int, categoryBits uint64) {
	checkProxyId(tree, proxyId)
	tree.proxies[proxyId].categoryBits = categoryBits
}

func (tree *dynamicTree) getCategoryBits(proxyId int) uint64 {
	checkProxyId(tree, proxyId)
	return tree.proxies[proxyId].categoryBits
}

func (tree *dynamicTree) getHeight() int {
	return tree.nodes[rootNode].height()
}

// getAreaRatio returns the summed perimeter of the non-root internal nodes
// over the perimeter of the root. It measures the quality of the tree. It
// corresponds to b2DynamicTree_GetAreaRatio in src/dynamic_tree.c.
func (tree *dynamicTree) getAreaRatio() Q {
	if tree.proxyCount == 0 {
		return QZero()
	}

	nodes := tree.nodes
	rootArea := perimeter(nodes[rootNode].aabb)
	if !QZero().Less(rootArea) {
		return QZero()
	}

	// Free nodes and the empty node are leaf tagged.
	internalArea := QZero()
	for i := 2; i < tree.nodeEnd; i++ {
		if !nodes[i].isLeaf() {
			internalArea = internalArea.Add(perimeter(nodes[i].aabb))
		}
	}

	return internalArea.Div(rootArea)
}

func (tree *dynamicTree) getRootBounds() AABB {
	if tree.proxyCount == 0 {
		return AABB{}
	}
	return tree.nodes[rootNode].aabb
}

func (tree *dynamicTree) getUserData(proxyId int) uint64 {
	checkProxyId(tree, proxyId)
	return tree.proxies[proxyId].userData
}

// getProxyBounds corresponds to b2DynamicTree_GetProxyBounds in
// src/dynamic_tree.c.
func (tree *dynamicTree) getProxyBounds(proxyId int) AABB {
	checkProxyId(tree, proxyId)
	nodeIndex := int(tree.proxies[proxyId].node)
	if nodeIndex < 0 || nodeIndex >= tree.nodeEnd {
		panic("dbox2d: the proxy is free")
	}
	return tree.nodes[nodeIndex].aabb
}

// hasMoved corresponds to b2HasTreeMoved in src/dynamic_tree.h.
func (tree *dynamicTree) hasMoved() bool {
	return tree.nodes[rootNode].isMoved()
}

// needsRebuild corresponds to b2NeedsRebuild in src/dynamic_tree.h.
func (tree *dynamicTree) needsRebuild() bool {
	return tree.nodes[rootNode].isMoved() || !tree.dfsOrdered
}

// validateSubtree returns the height of the subtree and counts its leaves.
// It corresponds to b2ValidateSubtree in src/dynamic_tree.c.
func (tree *dynamicTree) validateSubtree(nodeIndex int, leafCount *int) int {
	check := func(ok bool, what string) {
		if !ok {
			panic("dbox2d: tree validation: " + what)
		}
	}

	check(0 <= nodeIndex && nodeIndex < tree.nodeEnd, "node index out of range")
	node := &tree.nodes[nodeIndex]
	check(!node.isEmpty(), "empty node in the tree")

	if node.isLeaf() {
		proxyId := node.proxyId()
		check(0 <= proxyId && proxyId < len(tree.proxies), "proxy id out of range")
		check(int(tree.proxies[proxyId].node) == nodeIndex, "proxy does not point at its leaf")
		check(int32(tree.proxies[proxyId].userData) == node.aux, "leaf shape index differs from the proxy")
		*leafCount += 1
		return 0
	}

	pair := node.leftChild()
	check(pair&1 == 0 && 2 <= pair && pair < tree.nodeEnd, "bad children pair")
	check(int(tree.parents[pair]) == nodeIndex, "bad parent of the left child")
	check(int(tree.parents[pair+1]) == nodeIndex, "bad parent of the right child")
	check(!tree.dfsOrdered || nodeIndex < pair, "children below the parent in a DFS ordered tree")

	c1 := &tree.nodes[pair]
	c2 := &tree.nodes[pair+1]
	check(AABBContains(node.aabb, c1.aabb), "parent box misses the left child")
	check(AABBContains(node.aabb, c2.aabb), "parent box misses the right child")
	check(node.isMoved() == (c1.isMoved() || c2.isMoved()), "moved flag differs from the children")

	height1 := tree.validateSubtree(pair, leafCount)
	height2 := tree.validateSubtree(pair+1, leafCount)
	height := 1 + max(height1, height2)
	check(int(node.aux) == height, "bad height")
	return height
}

// validate checks the whole structure. It corresponds to
// b2DynamicTree_Validate in src/dynamic_tree.c.
func (tree *dynamicTree) validate() {
	check := func(ok bool, what string) {
		if !ok {
			panic("dbox2d: tree validation: " + what)
		}
	}

	check(2 <= tree.nodeEnd && tree.nodeEnd <= len(tree.nodes), "node end out of range")
	check(tree.nodeEnd&1 == 0, "odd node end")
	check(tree.parents[rootNode] == nullIndex, "root has a parent")
	check(tree.nodes[rootNode+1].isEmpty(), "node one is not empty")

	freePairCount := 0
	pair := tree.pairFreeList
	for pair != nullIndex {
		check(pair&1 == 0 && 2 <= pair && pair < tree.nodeEnd, "bad free pair")
		check(tree.nodes[pair].isEmpty() && tree.nodes[pair+1].isEmpty(), "free pair not empty")
		pair = int(tree.parents[pair])
		freePairCount++
		check(2*freePairCount < tree.nodeEnd, "free pair list cycles")
	}

	freeProxyCount := 0
	freeIndex := tree.proxyFreeList
	for freeIndex != nullIndex {
		check(0 <= freeIndex && freeIndex < len(tree.proxies), "bad free proxy")
		check(tree.proxies[freeIndex].node == nullIndex, "free proxy has a node")
		freeIndex = int(tree.proxies[freeIndex].next)
		freeProxyCount++
	}
	check(tree.proxyCount+freeProxyCount == len(tree.proxies), "proxy count mismatch")

	check(tree.nodeEnd == 2*max(tree.proxyCount, 1)+2*freePairCount, "node count mismatch")

	if tree.proxyCount == 0 {
		check(tree.nodes[rootNode].isEmpty(), "empty tree with a root")
		return
	}

	leafCount := 0
	tree.validateSubtree(rootNode, &leafCount)
	check(leafCount == tree.proxyCount, "leaf count mismatch")
}

// validateNoMoved corresponds to b2DynamicTree_ValidateNoMoved in
// src/dynamic_tree.c.
func (tree *dynamicTree) validateNoMoved() {
	for i := range tree.nodeEnd {
		if tree.nodes[i].isMoved() {
			panic("dbox2d: tree validation: a node is still marked moved")
		}
	}
}

// treeStats reports the work of one tree walk. It corresponds to
// b2TreeStats in include/box2d/collision.h.
type treeStats struct {
	nodeVisits int
	leafVisits int
}

// treeQueryCallback receives each leaf that overlaps the query box. It
// returns false to stop the query. D-014: the reference passes a context
// pointer; the port uses a closure.
type treeQueryCallback func(proxyId int, userData uint64) bool

// treeRayCastCallback receives each leaf whose box the ray reaches. It
// returns zero to stop the cast, a fraction in (0, maxFraction] to clip
// the ray, or any other value to continue unchanged. D-014 applies.
type treeRayCastCallback func(input *RayCastInput, proxyId int, userData uint64) Q

// BoxCastInput is the input of a swept box cast through a tree. It
// corresponds to b2BoxCastInput in include/box2d/collision.h.
type BoxCastInput struct {
	// Box is the box to cast, in the tree's frame.
	Box AABB

	// Translation is the sweep translation.
	Translation Vec2

	// MaxFraction limits the translation to consider. It is usually one.
	MaxFraction Q
}

// treeBoxCastCallback receives each leaf the swept box may reach. It
// returns zero to stop the cast, a fraction below maxFraction to clip the
// sweep, or maxFraction to continue unchanged. D-014 applies.
type treeBoxCastCallback func(input *BoxCastInput, proxyId int, userData uint64) Q

// overlapAABB calls back every leaf that overlaps the box and passes the
// mask. It corresponds to b2DynamicTree_OverlapAABB in src/dynamic_tree.c.
func (tree *dynamicTree) overlapAABB(aabb AABB, maskBits uint64, callback treeQueryCallback) treeStats {
	var stack [treeStackSize]int
	return tree.overlapAABBStack(&stack, aabb, maskBits, false, callback)
}

// overlapAABBAll ignores the category bits. It serves debug draw. It
// corresponds to b2DynamicTree_OverlapAABBAll in src/dynamic_tree.c.
func (tree *dynamicTree) overlapAABBAll(aabb AABB, callback treeQueryCallback) treeStats {
	var stack [treeStackSize]int
	return tree.overlapAABBStack(&stack, aabb, 0, true, callback)
}

// overlapAABBStack is overlapAABB on a caller stack, which lets a worker
// reuse one stack across queries.
func (tree *dynamicTree) overlapAABBStack(stack *[treeStackSize]int, aabb AABB, maskBits uint64, all bool, callback treeQueryCallback) treeStats {
	result := treeStats{}

	if tree.proxyCount == 0 {
		return result
	}

	nodes := tree.nodes
	proxies := tree.proxies

	stackCount := 0
	stack[stackCount] = rootPair(nodes)
	stackCount++

	for stackCount > 0 {
		stackCount--
		pair := stack[stackCount]
		result.nodeVisits += 1

		for i := range 2 {
			node := &nodes[pair+i]
			if !AABBOverlaps(node.aabb, aabb) {
				continue
			}

			if node.isLeaf() {
				// callback to user code with proxy id
				proxyId := node.proxyId()
				proxy := &proxies[proxyId]
				if all || proxy.categoryBits&maskBits != 0 {
					proceed := callback(proxyId, proxy.userData)
					result.leafVisits += 1

					if !proceed {
						return result
					}
				}
			} else {
				if stackCount >= treeStackSize-1 {
					panic("dbox2d: the tree query stack is full")
				}
				stack[stackCount] = node.leftChild()
				stackCount++
			}
		}
	}

	return result
}

// castRay calls back every leaf whose box the ray reaches, nearer pairs
// first. It corresponds to b2DynamicTree_CastRay in src/dynamic_tree.c.
func (tree *dynamicTree) castRay(input *RayCastInput, maskBits uint64, callback treeRayCastCallback) treeStats {
	result := treeStats{}

	if tree.proxyCount == 0 {
		return result
	}

	p1 := input.Origin
	d := input.Translation

	r := d.Normalize()

	// v is perpendicular to the segment.
	v := CrossSV(QOne(), r)
	absV := Abs(v)

	// Separating axis for segment (Gino, p80).
	// |dot(v, p1 - c)| > dot(|v|, h)
	maxFraction := input.MaxFraction

	p2 := MulAdd(p1, maxFraction, d)

	// Build a bounding box for the segment.
	segmentAABB := AABB{LowerBound: Min(p1, p2), UpperBound: Max(p1, p2)}

	nodes := tree.nodes
	proxies := tree.proxies

	var stack [treeStackSize]int
	stackCount := 0
	stack[stackCount] = rootPair(nodes)
	stackCount++

	subInput := *input

	zero := QZero()

	for stackCount > 0 {
		stackCount--
		pair := stack[stackCount]
		result.nodeVisits += 1

		var hit [2]int
		var isLeaf [2]bool
		hitCount := 0
		for i := range 2 {
			node := &nodes[pair+i]
			if !AABBOverlaps(node.aabb, segmentAABB) {
				continue
			}

			// Separating axis for segment (Gino, p80).
			// |dot(v, p1 - c)| > dot(|v|, h)
			c := AABBCenter(node.aabb)
			h := AABBExtents(node.aabb)
			term1 := v.Dot(p1.Sub(c)).Abs()
			term2 := absV.Dot(h)
			if term2.Less(term1) {
				continue
			}

			isLeaf[hitCount] = node.isLeaf()
			hit[hitCount] = pair + i
			hitCount++
		}

		if hitCount == 2 && !isLeaf[0] && !isLeaf[1] {
			d1 := AABBCenter(nodes[hit[0]].aabb).DistanceSq(p1)
			d2 := AABBCenter(nodes[hit[1]].aabb).DistanceSq(p1)

			// Want to push the closest one last. Both have the same
			// isLeaf, so they don't swap.
			if d1.Less(d2) {
				hit[0], hit[1] = hit[1], hit[0]
			}
		}

		for i := range hitCount {
			node := &nodes[hit[i]]
			if isLeaf[i] {
				proxyId := node.proxyId()
				proxy := &proxies[proxyId]

				if proxy.categoryBits&maskBits == 0 {
					continue
				}

				subInput.MaxFraction = maxFraction

				value := callback(&subInput, proxyId, proxy.userData)
				result.leafVisits += 1

				// The user may return -1 to indicate this shape should be skipped

				if value.Eq(zero) {
					// The client has terminated the ray cast.
					return result
				}

				if zero.Less(value) && !maxFraction.Less(value) {
					// Update segment bounding box.
					maxFraction = value
					p2 = MulAdd(p1, maxFraction, d)
					segmentAABB.LowerBound = Min(p1, p2)
					segmentAABB.UpperBound = Max(p1, p2)
				}
			} else {
				if stackCount >= treeStackSize-1 {
					panic("dbox2d: the tree ray cast stack is full")
				}
				stack[stackCount] = node.leftChild()
				stackCount++
			}
		}
	}

	return result
}

// castBox calls back every leaf the swept box may reach, nearer pairs
// first. It corresponds to b2DynamicTree_CastBox in src/dynamic_tree.c.
func (tree *dynamicTree) castBox(input *BoxCastInput, maskBits uint64, callback treeBoxCastCallback) treeStats {
	result := treeStats{}

	if tree.proxyCount == 0 {
		return result
	}

	originAABB := input.Box

	p1 := AABBCenter(originAABB)
	extension := AABBExtents(originAABB)

	// v is perpendicular to the segment.
	r := input.Translation
	v := CrossSV(QOne(), r)
	absV := Abs(v)

	// Separating axis for segment (Gino, p80).
	// |dot(v, p1 - c)| > dot(|v|, h)

	maxFraction := input.MaxFraction

	// Build total box for the cast
	t := input.Translation.Mul(maxFraction)
	totalAABB := AABB{
		LowerBound: Min(originAABB.LowerBound, originAABB.LowerBound.Add(t)),
		UpperBound: Max(originAABB.UpperBound, originAABB.UpperBound.Add(t)),
	}

	nodes := tree.nodes
	proxies := tree.proxies

	var stack [treeStackSize]int
	stackCount := 0
	stack[stackCount] = rootPair(nodes)
	stackCount++

	subInput := *input

	zero := QZero()

	for stackCount > 0 {
		stackCount--
		pair := stack[stackCount]
		result.nodeVisits += 1

		var hit [2]int
		var isLeaf [2]bool
		hitCount := 0
		for i := range 2 {
			node := &nodes[pair+i]
			if !AABBOverlaps(node.aabb, totalAABB) {
				continue
			}

			// Separating axis for segment (Gino, p80).
			// |dot(v, p1 - c)| > dot(|v|, h)
			// radius extension is added to the node in this case
			c := AABBCenter(node.aabb)
			h := AABBExtents(node.aabb).Add(extension)
			term1 := v.Dot(p1.Sub(c)).Abs()
			term2 := absV.Dot(h)
			if term2.Less(term1) {
				continue
			}

			isLeaf[hitCount] = node.isLeaf()
			hit[hitCount] = pair + i
			hitCount++
		}

		if hitCount == 2 && !isLeaf[0] && !isLeaf[1] {
			d1 := AABBCenter(nodes[hit[0]].aabb).DistanceSq(p1)
			d2 := AABBCenter(nodes[hit[1]].aabb).DistanceSq(p1)

			// Want to push the closest one last. Both have the same
			// isLeaf, so they don't swap.
			if d1.Less(d2) {
				hit[0], hit[1] = hit[1], hit[0]
			}
		}

		for i := range hitCount {
			node := &nodes[hit[i]]
			if isLeaf[i] {
				proxyId := node.proxyId()
				proxy := &proxies[proxyId]

				if proxy.categoryBits&maskBits == 0 {
					continue
				}

				subInput.MaxFraction = maxFraction

				value := callback(&subInput, proxyId, proxy.userData)
				result.leafVisits += 1

				// The user may return -1 to indicate this shape should be skipped

				if value.Eq(zero) {
					// The client has terminated the cast.
					return result
				}

				if zero.Less(value) && value.Less(maxFraction) {
					// Update cast bounding box.
					maxFraction = value
					t = input.Translation.Mul(maxFraction)
					totalAABB.LowerBound = Min(originAABB.LowerBound, originAABB.LowerBound.Add(t))
					totalAABB.UpperBound = Max(originAABB.UpperBound, originAABB.UpperBound.Add(t))
				}
			} else {
				if stackCount >= treeStackSize-1 {
					panic("dbox2d: the tree box cast stack is full")
				}
				stack[stackCount] = node.leftChild()
				stackCount++
			}
		}
	}

	return result
}

// treeHeuristic selects the rebuild partition, as B2_TREE_HEURISTIC in
// src/dynamic_tree.c: zero is the median split, one is the surface area
// heuristic.
const treeHeuristic = 0

// partitionMid splits the centers on the longest axis at the middle of
// their bounds, with the Hoare scheme. It returns the left count. It
// corresponds to b2PartitionMid in src/dynamic_tree.c.
func partitionMid(indices []int, centers []Vec2, count int) int {
	// Handle trivial case
	if count <= 2 {
		return count / 2
	}

	lowerBound := centers[0]
	upperBound := centers[0]

	for i := 1; i < count; i++ {
		lowerBound = Min(lowerBound, centers[i])
		upperBound = Max(upperBound, centers[i])
	}

	d := upperBound.Sub(lowerBound)
	half := QHalf()
	c := Vec2{X: half.Mul(lowerBound.X.Add(upperBound.X)), Y: half.Mul(lowerBound.Y.Add(upperBound.Y))}

	// Partition longest axis using the Hoare partition scheme
	// https://en.wikipedia.org/wiki/Quicksort
	// https://nicholasvadivelu.com/2021/01/11/array-partition/
	i1, i2 := 0, count
	if d.Y.Less(d.X) {
		pivot := c.X

		for i1 < i2 {
			for i1 < i2 && centers[i1].X.Less(pivot) {
				i1 += 1
			}

			for i1 < i2 && !centers[i2-1].X.Less(pivot) {
				i2 -= 1
			}

			if i1 < i2 {
				indices[i1], indices[i2-1] = indices[i2-1], indices[i1]
				centers[i1], centers[i2-1] = centers[i2-1], centers[i1]

				i1 += 1
				i2 -= 1
			}
		}
	} else {
		pivot := c.Y

		for i1 < i2 {
			for i1 < i2 && centers[i1].Y.Less(pivot) {
				i1 += 1
			}

			for i1 < i2 && !centers[i2-1].Y.Less(pivot) {
				i2 -= 1
			}

			if i1 < i2 {
				indices[i1], indices[i2-1] = indices[i2-1], indices[i1]
				centers[i1], centers[i2-1] = centers[i2-1], centers[i1]

				i1 += 1
				i2 -= 1
			}
		}
	}
	if i1 != i2 {
		panic("dbox2d: the median partition did not meet")
	}

	if i1 > 0 && i1 < count {
		return i1
	}

	return count / 2
}

// treeBinCount is the bin count of the surface area partition.
const treeBinCount = 8

type treeBin struct {
	aabb  AABB
	count int
}

type treePlane struct {
	leftAABB   AABB
	rightAABB  AABB
	leftCount  int
	rightCount int
}

// partitionSAH splits the boxes on the plane of least surface area cost,
// "On Fast Construction of SAH-based Bounding Volume Hierarchies" by
// Ingo Wald. It returns the left count. It corresponds to b2PartitionSAH
// in src/dynamic_tree.c and runs only when treeHeuristic is one.
func partitionSAH(indices []int, binIndices []int, boxes []AABB, count int) int {
	if count <= 0 {
		panic("dbox2d: the surface area partition needs a box")
	}

	var bins [treeBinCount]treeBin
	var planes [treeBinCount - 1]treePlane

	center := AABBCenter(boxes[0])
	centroidAABB := AABB{LowerBound: center, UpperBound: center}

	for i := 1; i < count; i++ {
		center = AABBCenter(boxes[i])
		centroidAABB.LowerBound = Min(centroidAABB.LowerBound, center)
		centroidAABB.UpperBound = Max(centroidAABB.UpperBound, center)
	}

	d := centroidAABB.UpperBound.Sub(centroidAABB.LowerBound)

	// Find longest axis
	var axisIndex int
	var invD Q
	if d.Y.Less(d.X) {
		axisIndex = 0
		invD = d.X
	} else {
		axisIndex = 1
		invD = d.Y
	}

	zero := QZero()
	if zero.Less(invD) {
		invD = QOne().Div(invD)
	} else {
		invD = zero
	}

	// D-009: the seed bounds are the largest values, which no box reaches.
	maxValue := QMaxValue()

	// Initialize bin bounds and count
	for i := range treeBinCount {
		bins[i].aabb.LowerBound = Vec2{X: maxValue, Y: maxValue}
		bins[i].aabb.UpperBound = Vec2{X: maxValue.Neg(), Y: maxValue.Neg()}
		bins[i].count = 0
	}

	// Assign boxes to bins and compute bin boxes
	binCount := QFromInt(treeBinCount)
	lowerBoundArray := [2]Q{centroidAABB.LowerBound.X, centroidAABB.LowerBound.Y}
	minC := lowerBoundArray[axisIndex]
	for i := range count {
		c := AABBCenter(boxes[i])
		cArray := [2]Q{c.X, c.Y}
		binIndex := binCount.Mul(cArray[axisIndex].Sub(minC)).Mul(invD).Int()
		binIndex = ClampInt(binIndex, 0, treeBinCount-1)
		binIndices[i] = binIndex
		bins[binIndex].count += 1
		bins[binIndex].aabb = AABBUnion(bins[binIndex].aabb, boxes[i])
	}

	planeCount := treeBinCount - 1

	// Prepare all the left planes, candidates for left child
	planes[0].leftCount = bins[0].count
	planes[0].leftAABB = bins[0].aabb
	for i := 1; i < planeCount; i++ {
		planes[i].leftCount = planes[i-1].leftCount + bins[i].count
		planes[i].leftAABB = AABBUnion(planes[i-1].leftAABB, bins[i].aabb)
	}

	// Prepare all the right planes, candidates for right child
	planes[planeCount-1].rightCount = bins[planeCount].count
	planes[planeCount-1].rightAABB = bins[planeCount].aabb
	for i := planeCount - 2; i >= 0; i-- {
		planes[i].rightCount = planes[i+1].rightCount + bins[i+1].count
		planes[i].rightAABB = AABBUnion(planes[i+1].rightAABB, bins[i+1].aabb)
	}

	// Find best split to minimize SAH
	minCost := maxValue
	bestPlane := 0
	for i := range planeCount {
		leftArea := perimeter(planes[i].leftAABB)
		rightArea := perimeter(planes[i].rightAABB)
		leftCount := planes[i].leftCount
		rightCount := planes[i].rightCount

		cost := QFromInt(leftCount).Mul(leftArea).Add(QFromInt(rightCount).Mul(rightArea))
		if cost.Less(minCost) {
			bestPlane = i
			minCost = cost
		}
	}

	// Partition node indices and boxes using the Hoare partition scheme
	// https://en.wikipedia.org/wiki/Quicksort
	// https://nicholasvadivelu.com/2021/01/11/array-partition/
	i1, i2 := 0, count
	for i1 < i2 {
		for i1 < i2 && binIndices[i1] < bestPlane {
			i1 += 1
		}

		for i1 < i2 && binIndices[i2-1] >= bestPlane {
			i2 -= 1
		}

		if i1 < i2 {
			indices[i1], indices[i2-1] = indices[i2-1], indices[i1]
			boxes[i1], boxes[i2-1] = boxes[i2-1], boxes[i1]

			i1 += 1
			i2 -= 1
		}
	}
	if i1 != i2 {
		panic("dbox2d: the surface area partition did not meet")
	}

	if i1 > 0 && i1 < count {
		return i1
	}

	return count / 2
}

// partition splits the leaves of one rebuild item with the selected
// heuristic.
func partition(tree *dynamicTree, startIndex, count int) int {
	if treeHeuristic == 0 {
		return partitionMid(tree.leafIndices[startIndex:], tree.leafCenters[startIndex:], count)
	}
	return partitionSAH(tree.leafIndices[startIndex:], tree.binIndices[startIndex:], tree.leafBoxes[startIndex:], count)
}

// rebuildItem tracks the rebuild of one tree node. It corresponds to
// b2RebuildItem in src/dynamic_tree.c.
type rebuildItem struct {
	// Where this node is written and the pair its children go in.
	nodeIndex  int
	pair       int
	childCount int

	// Leaf indices
	startIndex int
	splitIndex int
	endIndex   int
}

// copyItem corresponds to b2CopyItem in src/dynamic_tree.c.
type copyItem struct {
	oldPair  int
	newIndex int
}

// bumpPair bump allocates a pair of sibling nodes and returns the index of
// the first. It corresponds to b2BumpPair in src/dynamic_tree.c.
func bumpPair(tree *dynamicTree, parent int) int {
	pair := tree.nodeEnd
	if pair+2 > len(tree.nodes) {
		panic("dbox2d: the tree rebuild overflows the node array")
	}
	tree.nodeEnd += 2
	tree.parents[pair] = int32(parent)
	tree.parents[pair+1] = int32(parent)
	return pair
}

// copySubtree copies a retained subtree from the old array into the
// rebuilt DFS array with the bump allocator. It corresponds to
// b2CopySubtree in src/dynamic_tree.c.
func copySubtree(tree *dynamicTree, node treeNode, newIndex int, stack *[treeStackSize]copyItem) {
	oldNodes := tree.nodes
	newNodes := tree.swapNodes
	proxies := tree.proxies

	if node.isLeaf() {
		newNodes[newIndex] = node
		proxies[node.proxyId()].node = int32(newIndex)
		return
	}

	stackCount := 0

	oldPair := node.leftChild()
	newPair := bumpPair(tree, newIndex)
	node.setLeftChild(newPair)

	// Copy the subtree root. This can be a left or right sibling (even or
	// odd).
	newNodes[newIndex] = node

	for {
		// Copy the siblings.
		newNodes[newPair] = oldNodes[oldPair]
		newNodes[newPair+1] = oldNodes[oldPair+1]
		left := &newNodes[newPair]
		right := &newNodes[newPair+1]

		if right.isLeaf() {
			// Hook up proxy on right sibling.
			proxies[right.proxyId()].node = int32(newPair + 1)
		} else {
			// Push the left child of the right sibling.
			if stackCount >= treeStackSize {
				panic("dbox2d: the tree copy stack is full")
			}
			stack[stackCount] = copyItem{oldPair: right.leftChild(), newIndex: newPair + 1}
			stackCount++
		}

		if !left.isLeaf() {
			// Descend into left subtree.
			leftIndex := newPair
			oldPair = left.leftChild()
			newPair = bumpPair(tree, leftIndex)
			newNodes[leftIndex].setLeftChild(newPair)
			continue
		}

		// Hook up proxy on left sibling.
		proxies[left.proxyId()].node = int32(newPair)

		if stackCount == 0 {
			break
		}

		// Descend a right subtree that was previously pushed.
		stackCount--
		item := stack[stackCount]
		oldPair = item.oldPair
		newPair = bumpPair(tree, item.newIndex)
		newNodes[item.newIndex].setLeftChild(newPair)
	}
}

// placeLeaf writes a build leaf, a proxy or a retained subtree. It
// corresponds to b2PlaceLeaf in src/dynamic_tree.c.
func placeLeaf(tree *dynamicTree, leaf treeNode, newIndex int, stack *[treeStackSize]copyItem) {
	if leaf.isLeaf() {
		tree.swapNodes[newIndex] = leaf
		tree.proxies[leaf.proxyId()].node = int32(newIndex)
	} else {
		copySubtree(tree, leaf, newIndex, stack)
	}
}

// buildTree builds the swap nodes top down over the gathered build
// leaves. It corresponds to b2BuildTree in src/dynamic_tree.c.
func buildTree(tree *dynamicTree, leafCount int) {
	// One stack for all subtree copies. A local array in copySubtree is zeroed per call.
	var copyStack [treeStackSize]copyItem

	nodes := tree.swapNodes
	leaves := tree.leafNodes
	leafIndices := tree.leafIndices

	// Bump allocation into the swap nodes.
	tree.nodeEnd = 2
	nodes[rootNode+1] = makeEmptyNode()
	tree.parents[rootNode] = nullIndex
	tree.parents[rootNode+1] = nullIndex

	if leafCount == 1 {
		placeLeaf(tree, leaves[leafIndices[0]], rootNode, &copyStack)
		return
	}

	var stack [treeStackSize]rebuildItem
	top := 0

	stack[0].nodeIndex = rootNode
	stack[0].pair = bumpPair(tree, rootNode)
	stack[0].childCount = -1
	stack[0].startIndex = 0
	stack[0].endIndex = leafCount
	stack[0].splitIndex = partition(tree, 0, leafCount)

	for {
		item := &stack[top]
		item.childCount += 1

		if item.childCount == 2 {
			// Both children written, so the parent node can be finalized.
			nodes[item.nodeIndex] = makeInternalNode(nodes, item.pair)
			if top == 0 {
				break
			}

			top -= 1
			continue
		}

		slot := item.childCount
		startIndex := item.startIndex
		endIndex := item.splitIndex
		if slot != 0 {
			startIndex = item.splitIndex
			endIndex = item.endIndex
		}
		count := endIndex - startIndex
		if count <= 0 {
			panic("dbox2d: the tree rebuild split is empty")
		}

		nodeIndex := item.pair + slot
		if count == 1 {
			placeLeaf(tree, leaves[leafIndices[startIndex]], nodeIndex, &copyStack)
			continue
		}

		if top >= treeStackSize-1 {
			panic("dbox2d: the tree rebuild stack is full")
		}

		top += 1
		newItem := &stack[top]
		newItem.nodeIndex = nodeIndex
		newItem.pair = bumpPair(tree, nodeIndex)
		newItem.childCount = -1
		newItem.startIndex = startIndex
		newItem.endIndex = endIndex
		newItem.splitIndex = startIndex + partition(tree, startIndex, count)
	}
}

// rebuild rebuilds the stale parts of the tree and puts the whole tree in
// DFS order, which makes refitting much faster. It returns the number of
// boxes sorted. Not safe to access the tree during this operation. It
// corresponds to b2DynamicTree_Rebuild in src/dynamic_tree.c.
func (tree *dynamicTree) rebuild(fullBuild bool) int {
	proxyCount := tree.proxyCount
	if proxyCount == 0 {
		return 0
	}

	// An unordered tree is rebuilt even when nothing moved, the sweep
	// refit needs the order.
	nodes := tree.nodes
	root := nodes[rootNode]
	if !fullBuild && !root.isMoved() && tree.dfsOrdered {
		return 0
	}

	if tree.swapNodes == nil {
		tree.swapNodes = make([]treeNode, len(tree.nodes))
	}

	if proxyCount > len(tree.leafIndices) {
		newCapacity := proxyCount + proxyCount/2
		tree.leafIndices = make([]int, newCapacity)
		tree.leafNodes = make([]treeNode, newCapacity)
		if treeHeuristic == 0 {
			tree.leafCenters = make([]Vec2, newCapacity)
		} else {
			tree.leafBoxes = make([]AABB, newCapacity)
			tree.binIndices = make([]int, newCapacity)
		}
	}

	leafIndices := tree.leafIndices
	leaves := tree.leafNodes

	addLeaf := func(leafCount int, node treeNode) {
		node.flagIndex &^= movedNode
		leafIndices[leafCount] = leafCount
		leaves[leafCount] = node
		if treeHeuristic == 0 {
			tree.leafCenters[leafCount] = AABBCenter(node.aabb)
		} else {
			tree.leafBoxes[leafCount] = node.aabb
		}
	}

	// Gather build leaves. A marked internal node is descended and
	// abandoned in the old array. Everything else is a build leaf, kept
	// subtrees included. A leaf root, or an unmarked root on an unordered
	// array, is the one build leaf.
	leafCount := 0
	var stack [treeStackSize]int
	stackCount := 0
	if !root.isLeaf() && (fullBuild || root.isMoved()) {
		stack[stackCount] = root.leftChild()
		stackCount++
	} else {
		addLeaf(0, root)
		leafCount = 1
	}

	for stackCount > 0 {
		stackCount--
		pair := stack[stackCount]
		for i := range 2 {
			node := nodes[pair+i]
			if !node.isLeaf() && (fullBuild || node.isMoved()) {
				if stackCount >= treeStackSize {
					panic("dbox2d: the tree rebuild stack is full")
				}
				stack[stackCount] = node.leftChild()
				stackCount++
				continue
			}

			addLeaf(leafCount, node)
			leafCount++
		}
	}

	if leafCount <= 0 || leafCount > proxyCount {
		panic("dbox2d: the tree rebuild gathered a bad leaf count")
	}

	buildTree(tree, leafCount)

	// The spare is now the tree and the old array is the spare. The build
	// is dense, so the stale tail of the old array sits above the end and
	// is never read.
	tree.nodes, tree.swapNodes = tree.swapNodes, tree.nodes
	tree.pairFreeList = nullIndex
	tree.dfsOrdered = true

	validateTreeDebug(tree)
	validateTreeNoMovedDebug(tree)

	return leafCount
}

// markProxyMovedSerial sets the moved flag on the proxy and its
// ancestors. It corresponds to b2DynamicTree_MarkProxyMovedSerial in
// src/dynamic_tree.c.
func (tree *dynamicTree) markProxyMovedSerial(proxyId int) {
	checkProxyId(tree, proxyId)

	nodes := tree.nodes
	parents := tree.parents

	index := int(tree.proxies[proxyId].node)
	if index < 0 || index >= tree.nodeEnd || !nodes[index].isLeaf() {
		panic("dbox2d: the marked proxy is not a leaf")
	}

	for index != nullIndex {
		nodes[index].flagIndex |= movedNode
		index = int(parents[index])
	}
}

// markProxyMoved updates a proxy box and flags the ancestors as moved. It
// is safe across workers that mark different proxies: the leaf is owned by
// the caller and the ancestors take atomic flags. It corresponds to
// b2DynamicTree_MarkProxyMoved in src/dynamic_tree.c.
func (tree *dynamicTree) markProxyMoved(proxyId int, aabb AABB) {
	nodes := tree.nodes
	parents := tree.parents

	index := int(tree.proxies[proxyId].node)
	node := &nodes[index]

	// This is not raced since it is the leaf.
	node.aabb = aabb
	node.flagIndex |= movedNode

	index = int(parents[index])
	for index != nullIndex {
		flags := &nodes[index].flagIndex

		// Read first to avoid the fetch-or if possible.
		if atomic.LoadUint32(flags)&movedNode != 0 {
			break
		}

		previousFlags := atomic.OrUint32(flags, movedNode)
		if previousFlags&movedNode != 0 {
			// Ancestor already visited.
			break
		}

		index = int(parents[index])
	}
}

// clearMoved clears the moved flags from the entire tree. It corresponds
// to b2DynamicTree_ClearMoved in src/dynamic_tree.c.
func (tree *dynamicTree) clearMoved() {
	nodes := tree.nodes

	root := &nodes[rootNode]
	if !root.isMoved() {
		return
	}

	root.flagIndex &^= movedNode
	if root.isLeaf() {
		return
	}

	var stack [treeStackSize]int
	stackCount := 0
	stack[stackCount] = root.leftChild()
	stackCount++

	for stackCount > 0 {
		stackCount--
		pair := stack[stackCount]
		for i := range 2 {
			node := &nodes[pair+i]
			if node.flagIndex&movedNode != 0 {
				node.flagIndex &^= movedNode
				if !node.isLeaf() {
					// Bad stuff will happen if the moved flags don't get
					// cleared.
					if stackCount >= treeStackSize {
						panic("dbox2d: the tree clear stack is full")
					}
					stack[stackCount] = node.leftChild()
					stackCount++
				}
			}
		}
	}
}

// gatherMovedProxies appends the moved proxies to proxyIds and returns
// it. It corresponds to b2DynamicTree_GatherMovedProxies in
// src/dynamic_tree.c.
func (tree *dynamicTree) gatherMovedProxies(proxyIds []int) []int {
	nodes := tree.nodes
	root := &nodes[rootNode]
	if !root.isMoved() {
		return proxyIds
	}

	if root.isLeaf() {
		return append(proxyIds, root.proxyId())
	}

	var stack [treeStackSize]int
	stackCount := 0
	stack[stackCount] = root.leftChild()
	stackCount++

	for stackCount > 0 {
		stackCount--
		pair := stack[stackCount]
		for i := range 2 {
			node := &nodes[pair+i]
			if !node.isMoved() {
				continue
			}

			if node.isLeaf() {
				proxyIds = append(proxyIds, node.proxyId())
			} else {
				if stackCount >= treeStackSize {
					panic("dbox2d: the tree gather stack is full")
				}
				stack[stackCount] = node.leftChild()
				stackCount++
			}
		}
	}

	return proxyIds
}

// refitSubtree is the slow refit for an unordered tree, which only unit
// tests build. Negative entries are deferred unions: a parent is pushed
// before its children, so the LIFO order finishes both children before
// the union runs. It corresponds to b2RefitSubtree in src/dynamic_tree.c.
func refitSubtree(nodes []treeNode, rootIndex int) {
	root := &nodes[rootIndex]
	if root.isLeaf() || !root.isMoved() {
		return
	}

	var stack [treeStackSize]int
	stackCount := 0
	push := func(item int) {
		stack[stackCount] = item
		stackCount++
	}

	// ^0 == -1
	push(^rootIndex)
	push(root.leftChild())
	push(root.leftChild() + 1)

	for stackCount > 0 {
		stackCount--
		item := stack[stackCount]
		if item < 0 {
			node := &nodes[^item]
			pair := node.leftChild()
			node.aabb = AABBUnion(nodes[pair].aabb, nodes[pair+1].aabb)
			continue
		}

		node := &nodes[item]
		if node.isLeaf() || !node.isMoved() {
			continue
		}

		if stackCount+3 > treeStackSize {
			panic("dbox2d: the tree refit stack is full")
		}
		pair := node.leftChild()
		push(^item)
		push(pair)
		push(pair + 1)
	}
}

// refit recomputes the boxes of the moved internal nodes. On a DFS
// ordered tree it sweeps the pairs from the end down, so every child is
// done before its parent. It corresponds to b2DynamicTree_Refit in
// src/dynamic_tree.c.
func (tree *dynamicTree) refit() {
	if !tree.hasMoved() {
		return
	}

	nodes := tree.nodes

	if !tree.dfsOrdered {
		// This only happens in unit tests.
		refitSubtree(nodes, rootNode)
		return
	}

	for pair := tree.nodeEnd - 2; pair >= 0; pair -= 2 {
		flags1 := nodes[pair].flagIndex
		flags2 := nodes[pair+1].flagIndex

		// Did either move?
		if (flags1|flags2)&movedNode == 0 {
			continue
		}

		// Is the node internal and moved?
		if flags1&(leafNode|movedNode) == movedNode {
			children := int(flags1 & nodeIndexMask)
			nodes[pair].aabb = AABBUnion(nodes[children].aabb, nodes[children+1].aabb)
		}
		if flags2&(leafNode|movedNode) == movedNode {
			children := int(flags2 & nodeIndexMask)
			nodes[pair+1].aabb = AABBUnion(nodes[children].aabb, nodes[children+1].aabb)
		}
	}
}

// DynamicTree is a standalone bounding volume hierarchy over proxies. The
// world keeps its own trees; this type serves user code that wants the
// tree as a spatial structure. It corresponds to b2DynamicTree and the
// b2DynamicTree_* functions of include/box2d/collision.h.
type DynamicTree struct {
	tree dynamicTree
}

// TreeQueryCallbackFcn receives each proxy that overlaps the query box.
// It returns false to stop the query. It corresponds to
// b2TreeQueryCallbackFcn in include/box2d/collision.h. D-014: a closure
// replaces the context pointer.
type TreeQueryCallbackFcn func(proxyId int, userData uint64) bool

// TreeRayCastCallbackFcn receives each proxy whose box the ray reaches.
// It returns zero to stop the cast, a fraction in (0, maxFraction] to clip
// the ray, or any other value to continue unchanged. It corresponds to
// b2TreeRayCastCallbackFcn in include/box2d/collision.h. D-014 applies.
type TreeRayCastCallbackFcn func(input *RayCastInput, proxyId int, userData uint64) Q

// TreeBoxCastCallbackFcn receives each proxy the swept box may reach. It
// returns zero to stop the cast, a value below input.MaxFraction to clip
// the cast, or input.MaxFraction to continue without clipping. It
// corresponds to b2TreeBoxCastCallbackFcn in include/box2d/collision.h.
// D-014 applies.
type TreeBoxCastCallbackFcn func(input *BoxCastInput, proxyId int, userData uint64) Q

// NewDynamicTree returns an empty tree with room for proxyCapacity
// proxies. It corresponds to b2CreateDynamicTree.
func NewDynamicTree(proxyCapacity int) *DynamicTree {
	return &DynamicTree{tree: createTree(proxyCapacity)}
}

// Destroy releases the tree. It corresponds to b2DestroyDynamicTree.
func (t *DynamicTree) Destroy() {
	destroyTree(&t.tree)
}

// CreateProxy inserts a proxy with a fat box and returns its id. It
// corresponds to b2CreateTreeProxy.
func (t *DynamicTree) CreateProxy(aabb AABB, categoryBits uint64, userData uint64) int {
	return t.tree.createProxy(aabb, categoryBits, userData)
}

// DestroyProxy removes a proxy. It corresponds to b2DestroyTreeProxy.
func (t *DynamicTree) DestroyProxy(proxyId int) {
	t.tree.destroyProxy(proxyId)
}

// MoveProxy reinserts a proxy with a new box. It corresponds to
// b2DynamicTree_MoveProxy.
func (t *DynamicTree) MoveProxy(proxyId int, aabb AABB) {
	t.tree.moveProxy(proxyId, aabb)
}

// SetCategoryBits changes the category bits of a proxy. It corresponds to
// b2DynamicTree_SetCategoryBits.
func (t *DynamicTree) SetCategoryBits(proxyId int, categoryBits uint64) {
	t.tree.setCategoryBits(proxyId, categoryBits)
}

// GetCategoryBits returns the category bits of a proxy. It corresponds to
// b2DynamicTree_GetCategoryBits.
func (t *DynamicTree) GetCategoryBits(proxyId int) uint64 {
	return t.tree.getCategoryBits(proxyId)
}

// OverlapAABB calls back every proxy that overlaps the box and passes the
// mask. It corresponds to b2DynamicTree_OverlapAABB.
func (t *DynamicTree) OverlapAABB(aabb AABB, maskBits uint64, fcn TreeQueryCallbackFcn) TreeStats {
	stats := t.tree.overlapAABB(aabb, maskBits, treeQueryCallback(fcn))
	return TreeStats{NodeVisits: stats.nodeVisits, LeafVisits: stats.leafVisits}
}

// OverlapAABBAll calls back every proxy that overlaps the box, ignoring
// the category bits. It corresponds to b2DynamicTree_OverlapAABBAll.
func (t *DynamicTree) OverlapAABBAll(aabb AABB, fcn TreeQueryCallbackFcn) TreeStats {
	stats := t.tree.overlapAABBAll(aabb, treeQueryCallback(fcn))
	return TreeStats{NodeVisits: stats.nodeVisits, LeafVisits: stats.leafVisits}
}

// CastRay calls back every proxy whose box the ray reaches. It
// corresponds to b2DynamicTree_CastRay.
func (t *DynamicTree) CastRay(input *RayCastInput, maskBits uint64, fcn TreeRayCastCallbackFcn) TreeStats {
	stats := t.tree.castRay(input, maskBits, treeRayCastCallback(fcn))
	return TreeStats{NodeVisits: stats.nodeVisits, LeafVisits: stats.leafVisits}
}

// CastBox calls back every proxy the swept box may reach. The box sweeps
// from its origin to origin + MaxFraction * Translation. It corresponds to
// b2DynamicTree_CastBox.
func (t *DynamicTree) CastBox(input *BoxCastInput, maskBits uint64, fcn TreeBoxCastCallbackFcn) TreeStats {
	stats := t.tree.castBox(input, maskBits, treeBoxCastCallback(fcn))
	return TreeStats{NodeVisits: stats.nodeVisits, LeafVisits: stats.leafVisits}
}

// GetProxyCount returns the number of proxies. It corresponds to
// b2DynamicTree_GetProxyCount.
func (t *DynamicTree) GetProxyCount() int {
	return t.tree.getProxyCount()
}

// GetHeight returns the height of the tree. It corresponds to
// b2DynamicTree_GetHeight.
func (t *DynamicTree) GetHeight() int {
	return t.tree.getHeight()
}

// GetAreaRatio returns the summed perimeter of the internal nodes over
// the perimeter of the root. It corresponds to b2DynamicTree_GetAreaRatio.
func (t *DynamicTree) GetAreaRatio() Q {
	return t.tree.getAreaRatio()
}

// GetRootBounds returns the box of the root, or the empty box for an empty
// tree. It corresponds to b2DynamicTree_GetRootBounds.
func (t *DynamicTree) GetRootBounds() AABB {
	return t.tree.getRootBounds()
}

// GetByteCount returns the memory the tree holds. It corresponds to
// b2DynamicTree_GetByteCount.
func (t *DynamicTree) GetByteCount() int {
	return t.tree.byteCount()
}

// GetUserData returns the user data of a proxy. It corresponds to
// b2DynamicTree_GetUserData.
func (t *DynamicTree) GetUserData(proxyId int) uint64 {
	return t.tree.getUserData(proxyId)
}

// GetProxyBounds returns the box of a proxy. It corresponds to
// b2DynamicTree_GetProxyBounds.
func (t *DynamicTree) GetProxyBounds(proxyId int) AABB {
	return t.tree.getProxyBounds(proxyId)
}

// Validate checks the tree. Like the reference, it does nothing unless
// the build enables validation (-tags dbox2d_validate). It corresponds to
// b2DynamicTree_Validate.
func (t *DynamicTree) Validate() {
	validateTreeDebug(&t.tree)
}
