package dbox2d

import "github.com/dhannyell/fixed"

// The tree node flags of src/constants.h.
const (
	allocatedNode uint16 = 0x0001
	enlargedNode  uint16 = 0x0002
	leafNode      uint16 = 0x0004
)

// treeNode is a node of the dynamic tree. The reference overlays the
// children and the user data in one union, and the parent and the free
// list link in another; Go has no union, so each pair is two fields.
type treeNode struct {
	// aabb is the node bounding box.
	aabb AABB

	// categoryBits serve collision filtering.
	categoryBits uint64

	// The children of an internal node.
	child1 int32
	child2 int32

	// userData of a leaf node.
	userData uint64

	// parent of an allocated node.
	parent int32

	// next is the free list link of a free node.
	next int32

	height uint16
	flags  uint16
}

// defaultTreeNode is the state of a freshly allocated node.
var defaultTreeNode = treeNode{
	categoryBits: DefaultCategoryBits,
	child1:       nullIndex,
	child2:       nullIndex,
	parent:       nullIndex,
	next:         nullIndex,
	height:       0,
	flags:        allocatedNode,
}

func (n *treeNode) isLeaf() bool {
	return n.flags&leafNode != 0
}

func (n *treeNode) isAllocated() bool {
	return n.flags&allocatedNode != 0
}

// dynamicTree is a binary AABB tree over proxies. Nodes are pooled and
// relocatable, so the tree holds indices instead of pointers. It
// corresponds to b2DynamicTree in include/box2d/collision.h.
type dynamicTree struct {
	// nodes is the node pool. Its length is the node capacity.
	nodes []treeNode

	root int

	// nodeCount is the number of allocated nodes.
	nodeCount int

	// freeList is the head of the free node list.
	freeList int

	// proxyCount is the number of leaves.
	proxyCount int
}

// createTree initializes the node pool. It corresponds to
// b2DynamicTree_Create in src/dynamic_tree.c.
func createTree() dynamicTree {
	tree := dynamicTree{}
	tree.root = nullIndex

	nodeCapacity := 16
	tree.nodeCount = 0
	tree.nodes = make([]treeNode, nodeCapacity)

	// Build a linked list for the free list.
	for i := range nodeCapacity - 1 {
		tree.nodes[i].next = int32(i + 1)
	}

	tree.nodes[nodeCapacity-1].next = nullIndex
	tree.freeList = 0

	tree.proxyCount = 0

	return tree
}

// destroyTree releases the node pool.
func destroyTree(tree *dynamicTree) {
	*tree = dynamicTree{}
}

// allocateNode takes a node from the pool. It grows the pool when the free
// list is empty.
func allocateNode(tree *dynamicTree) int {
	// Expand the node pool as needed.
	if tree.freeList == nullIndex {
		oldCapacity := len(tree.nodes)
		if tree.nodeCount != oldCapacity {
			panic("dbox2d: the tree free list is empty before the pool is full")
		}

		// The free list is empty. Rebuild a bigger pool.
		nodeCapacity := oldCapacity + oldCapacity>>1
		nodes := make([]treeNode, nodeCapacity)
		copy(nodes, tree.nodes[:tree.nodeCount])
		tree.nodes = nodes

		// Build a linked list for the free list. The parent pointer becomes the "next" pointer.
		for i := tree.nodeCount; i < nodeCapacity-1; i++ {
			tree.nodes[i].next = int32(i + 1)
		}

		tree.nodes[nodeCapacity-1].next = nullIndex
		tree.freeList = tree.nodeCount
	}

	// Peel a node off the free list.
	nodeIndex := tree.freeList
	node := &tree.nodes[nodeIndex]
	tree.freeList = int(node.next)
	*node = defaultTreeNode
	tree.nodeCount++
	return nodeIndex
}

// freeNode returns a node to the pool.
func freeNode(tree *dynamicTree, nodeId int) {
	if nodeId < 0 || nodeId >= len(tree.nodes) {
		panic("dbox2d: the tree node id is out of range")
	}
	if tree.nodeCount <= 0 {
		panic("dbox2d: the tree has no node to free")
	}
	tree.nodes[nodeId].next = int32(tree.freeList)
	tree.nodes[nodeId].flags = 0
	tree.freeList = nodeId
	tree.nodeCount--
}

// findBestSibling walks one greedy path from the root and returns the
// sibling whose surface area heuristic cost is the lowest. It corresponds
// to b2FindBestSibling in src/dynamic_tree.c.
//
// With three nodes A-(B,C) and a new leaf D there are three choices:
// a new parent E-(A-(B,C), D); D under B; D under C. When B or C is an
// internal node the cost is a lower bound, so the search is greedy.
//
// cost of sibling H = area(union(H, D)) + increased area of ancestors
func findBestSibling(tree *dynamicTree, boxD AABB) int {
	centerD := AABBCenter(boxD)
	areaD := perimeter(boxD)

	nodes := tree.nodes
	rootIndex := tree.root

	rootBox := nodes[rootIndex].aabb

	// Area of current node
	areaBase := perimeter(rootBox)

	// Area of inflated node
	directCost := perimeter(AABBUnion(rootBox, boxD))
	inheritedCost := fixed.Q32Zero()

	bestSibling := rootIndex
	bestCost := directCost

	zero := fixed.Q32Zero()

	// D-009: the seed is the largest representable value, which no
	// perimeter reaches.
	maxCost := fixed.Q32MaxValue()

	// Descend the tree from root, following a single greedy path.
	index := rootIndex
	for nodes[index].height > 0 {
		child1 := int(nodes[index].child1)
		child2 := int(nodes[index].child2)

		// Cost of creating a new parent for this node and the new leaf
		cost := directCost.Add(inheritedCost)

		// Sometimes there are multiple identical costs within tolerance.
		// This breaks the ties using the centroid distance.
		if cost.Less(bestCost) {
			bestSibling = index
			bestCost = cost
		}

		// Inheritance cost seen by children
		inheritedCost = inheritedCost.Add(directCost.Sub(areaBase))

		leaf1 := nodes[child1].height == 0
		leaf2 := nodes[child2].height == 0

		// Cost of descending into child 1
		lowerCost1 := maxCost
		box1 := nodes[child1].aabb
		directCost1 := perimeter(AABBUnion(box1, boxD))
		area1 := zero
		if leaf1 {
			// Child 1 is a leaf
			// Cost of creating new node and increasing area of node P
			cost1 := directCost1.Add(inheritedCost)

			// Need this here due to while condition above
			if cost1.Less(bestCost) {
				bestSibling = child1
				bestCost = cost1
			}
		} else {
			// Child 1 is an internal node
			area1 = perimeter(box1)

			// Lower bound cost of inserting under child 1. The minimum accounts for two possibilities:
			// 1. Child1 could be the sibling with cost1 = inheritedCost + directCost1
			// 2. A descendent of child1 could be the sibling with the lower bound cost of
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
			index = child1
			areaBase = area1
			directCost = directCost1
		} else {
			index = child2
			areaBase = area2
			directCost = directCost2
		}

		if nodes[index].height <= 0 {
			panic("dbox2d: the sibling search descended into a leaf")
		}
	}

	return bestSibling
}

// The rotation choices of rotateNodes.
const (
	rotateNone = iota
	rotateBF
	rotateBG
	rotateCD
	rotateCE
)

// rotateNodes performs a left or right rotation if node A is imbalanced.
// It corresponds to b2RotateNodes in src/dynamic_tree.c.
func rotateNodes(tree *dynamicTree, iA int) {
	if iA == nullIndex {
		panic("dbox2d: rotateNodes on the null node")
	}

	nodes := tree.nodes

	A := &nodes[iA]
	if A.height < 2 {
		return
	}

	iB := int(A.child1)
	iC := int(A.child2)
	checkNodeIndex(tree, iB)
	checkNodeIndex(tree, iC)

	B := &nodes[iB]
	C := &nodes[iC]

	switch {
	case B.height == 0:
		// B is a leaf and C is internal
		if C.height <= 0 {
			panic("dbox2d: rotateNodes found two leaves under an internal node")
		}

		iF := int(C.child1)
		iG := int(C.child2)
		F := &nodes[iF]
		G := &nodes[iG]
		checkNodeIndex(tree, iF)
		checkNodeIndex(tree, iG)

		// Base cost
		costBase := perimeter(C.aabb)

		// Cost of swapping B and F
		aabbBG := AABBUnion(B.aabb, G.aabb)
		costBF := perimeter(aabbBG)

		// Cost of swapping B and G
		aabbBF := AABBUnion(B.aabb, F.aabb)
		costBG := perimeter(aabbBF)

		if costBase.Less(costBF) && costBase.Less(costBG) {
			// Rotation does not improve cost
			return
		}

		if costBF.Less(costBG) {
			// Swap B and F
			A.child1 = int32(iF)
			C.child1 = int32(iB)

			B.parent = int32(iC)
			F.parent = int32(iA)

			C.aabb = aabbBG

			C.height = 1 + max(B.height, G.height)
			A.height = 1 + max(C.height, F.height)
			C.categoryBits = B.categoryBits | G.categoryBits
			A.categoryBits = C.categoryBits | F.categoryBits
			C.flags |= (B.flags | G.flags) & enlargedNode
			A.flags |= (C.flags | F.flags) & enlargedNode
		} else {
			// Swap B and G
			A.child1 = int32(iG)
			C.child2 = int32(iB)

			B.parent = int32(iC)
			G.parent = int32(iA)

			C.aabb = aabbBF

			C.height = 1 + max(B.height, F.height)
			A.height = 1 + max(C.height, G.height)
			C.categoryBits = B.categoryBits | F.categoryBits
			A.categoryBits = C.categoryBits | G.categoryBits
			C.flags |= (B.flags | F.flags) & enlargedNode
			A.flags |= (C.flags | G.flags) & enlargedNode
		}

	case C.height == 0:
		// C is a leaf and B is internal
		if B.height <= 0 {
			panic("dbox2d: rotateNodes found two leaves under an internal node")
		}

		iD := int(B.child1)
		iE := int(B.child2)
		D := &nodes[iD]
		E := &nodes[iE]
		checkNodeIndex(tree, iD)
		checkNodeIndex(tree, iE)

		// Base cost
		costBase := perimeter(B.aabb)

		// Cost of swapping C and D
		aabbCE := AABBUnion(C.aabb, E.aabb)
		costCD := perimeter(aabbCE)

		// Cost of swapping C and E
		aabbCD := AABBUnion(C.aabb, D.aabb)
		costCE := perimeter(aabbCD)

		if costBase.Less(costCD) && costBase.Less(costCE) {
			// Rotation does not improve cost
			return
		}

		if costCD.Less(costCE) {
			// Swap C and D
			A.child2 = int32(iD)
			B.child1 = int32(iC)

			C.parent = int32(iB)
			D.parent = int32(iA)

			B.aabb = aabbCE

			B.height = 1 + max(C.height, E.height)
			A.height = 1 + max(B.height, D.height)
			B.categoryBits = C.categoryBits | E.categoryBits
			A.categoryBits = B.categoryBits | D.categoryBits
			B.flags |= (C.flags | E.flags) & enlargedNode
			A.flags |= (B.flags | D.flags) & enlargedNode
		} else {
			// Swap C and E
			A.child2 = int32(iE)
			B.child2 = int32(iC)

			C.parent = int32(iB)
			E.parent = int32(iA)

			B.aabb = aabbCD
			B.height = 1 + max(C.height, D.height)
			A.height = 1 + max(B.height, E.height)
			B.categoryBits = C.categoryBits | D.categoryBits
			A.categoryBits = B.categoryBits | E.categoryBits
			B.flags |= (C.flags | D.flags) & enlargedNode
			A.flags |= (B.flags | E.flags) & enlargedNode
		}

	default:
		iD := int(B.child1)
		iE := int(B.child2)
		iF := int(C.child1)
		iG := int(C.child2)

		D := &nodes[iD]
		E := &nodes[iE]
		F := &nodes[iF]
		G := &nodes[iG]

		checkNodeIndex(tree, iD)
		checkNodeIndex(tree, iE)
		checkNodeIndex(tree, iF)
		checkNodeIndex(tree, iG)

		// Base cost
		areaB := perimeter(B.aabb)
		areaC := perimeter(C.aabb)
		costBase := areaB.Add(areaC)
		bestRotation := rotateNone
		bestCost := costBase

		// Cost of swapping B and F
		aabbBG := AABBUnion(B.aabb, G.aabb)
		costBF := areaB.Add(perimeter(aabbBG))
		if costBF.Less(bestCost) {
			bestRotation = rotateBF
			bestCost = costBF
		}

		// Cost of swapping B and G
		aabbBF := AABBUnion(B.aabb, F.aabb)
		costBG := areaB.Add(perimeter(aabbBF))
		if costBG.Less(bestCost) {
			bestRotation = rotateBG
			bestCost = costBG
		}

		// Cost of swapping C and D
		aabbCE := AABBUnion(C.aabb, E.aabb)
		costCD := areaC.Add(perimeter(aabbCE))
		if costCD.Less(bestCost) {
			bestRotation = rotateCD
			bestCost = costCD
		}

		// Cost of swapping C and E
		aabbCD := AABBUnion(C.aabb, D.aabb)
		costCE := areaC.Add(perimeter(aabbCD))
		if costCE.Less(bestCost) {
			bestRotation = rotateCE
			// bestCost = costCE
		}

		switch bestRotation {
		case rotateNone:

		case rotateBF:
			A.child1 = int32(iF)
			C.child1 = int32(iB)

			B.parent = int32(iC)
			F.parent = int32(iA)

			C.aabb = aabbBG
			C.height = 1 + max(B.height, G.height)
			A.height = 1 + max(C.height, F.height)
			C.categoryBits = B.categoryBits | G.categoryBits
			A.categoryBits = C.categoryBits | F.categoryBits
			C.flags |= (B.flags | G.flags) & enlargedNode
			A.flags |= (C.flags | F.flags) & enlargedNode

		case rotateBG:
			A.child1 = int32(iG)
			C.child2 = int32(iB)

			B.parent = int32(iC)
			G.parent = int32(iA)

			C.aabb = aabbBF
			C.height = 1 + max(B.height, F.height)
			A.height = 1 + max(C.height, G.height)
			C.categoryBits = B.categoryBits | F.categoryBits
			A.categoryBits = C.categoryBits | G.categoryBits
			C.flags |= (B.flags | F.flags) & enlargedNode
			A.flags |= (C.flags | G.flags) & enlargedNode

		case rotateCD:
			A.child2 = int32(iD)
			B.child1 = int32(iC)

			C.parent = int32(iB)
			D.parent = int32(iA)

			B.aabb = aabbCE
			B.height = 1 + max(C.height, E.height)
			A.height = 1 + max(B.height, D.height)
			B.categoryBits = C.categoryBits | E.categoryBits
			A.categoryBits = B.categoryBits | D.categoryBits
			B.flags |= (C.flags | E.flags) & enlargedNode
			A.flags |= (B.flags | D.flags) & enlargedNode

		case rotateCE:
			A.child2 = int32(iE)
			B.child2 = int32(iC)

			C.parent = int32(iB)
			E.parent = int32(iA)

			B.aabb = aabbCD
			B.height = 1 + max(C.height, D.height)
			A.height = 1 + max(B.height, E.height)
			B.categoryBits = C.categoryBits | D.categoryBits
			A.categoryBits = B.categoryBits | E.categoryBits
			B.flags |= (C.flags | D.flags) & enlargedNode
			A.flags |= (B.flags | E.flags) & enlargedNode

		default:
			panic("dbox2d: unknown tree rotation")
		}
	}
}

// checkNodeIndex panics when an index points outside the node pool.
func checkNodeIndex(tree *dynamicTree, index int) {
	if index < 0 || index >= len(tree.nodes) {
		panic("dbox2d: the tree node index is out of range")
	}
}

// insertLeaf places a leaf beside its best sibling and repairs the
// ancestors. It corresponds to b2InsertLeaf in src/dynamic_tree.c.
func insertLeaf(tree *dynamicTree, leaf int, shouldRotate bool) {
	if tree.root == nullIndex {
		tree.root = leaf
		tree.nodes[tree.root].parent = nullIndex
		return
	}

	// Stage 1: find the best sibling for this node
	leafAABB := tree.nodes[leaf].aabb
	sibling := findBestSibling(tree, leafAABB)

	// Stage 2: create a new parent for the leaf and sibling
	oldParent := int(tree.nodes[sibling].parent)
	newParent := allocateNode(tree)

	// warning: node pointer can change after allocation
	nodes := tree.nodes
	nodes[newParent].parent = int32(oldParent)
	nodes[newParent].userData = ^uint64(0)
	nodes[newParent].aabb = AABBUnion(leafAABB, nodes[sibling].aabb)
	nodes[newParent].categoryBits = nodes[leaf].categoryBits | nodes[sibling].categoryBits
	nodes[newParent].height = nodes[sibling].height + 1

	if oldParent != nullIndex {
		// The sibling was not the root.
		if int(nodes[oldParent].child1) == sibling {
			nodes[oldParent].child1 = int32(newParent)
		} else {
			nodes[oldParent].child2 = int32(newParent)
		}

		nodes[newParent].child1 = int32(sibling)
		nodes[newParent].child2 = int32(leaf)
		nodes[sibling].parent = int32(newParent)
		nodes[leaf].parent = int32(newParent)
	} else {
		// The sibling was the root.
		nodes[newParent].child1 = int32(sibling)
		nodes[newParent].child2 = int32(leaf)
		nodes[sibling].parent = int32(newParent)
		nodes[leaf].parent = int32(newParent)
		tree.root = newParent
	}

	// Stage 3: walk back up the tree fixing heights and AABBs
	index := int(nodes[leaf].parent)
	for index != nullIndex {
		child1 := int(nodes[index].child1)
		child2 := int(nodes[index].child2)

		if child1 == nullIndex || child2 == nullIndex {
			panic("dbox2d: an internal tree node lost a child")
		}

		nodes[index].aabb = AABBUnion(nodes[child1].aabb, nodes[child2].aabb)
		nodes[index].categoryBits = nodes[child1].categoryBits | nodes[child2].categoryBits
		nodes[index].height = 1 + max(nodes[child1].height, nodes[child2].height)
		nodes[index].flags |= (nodes[child1].flags | nodes[child2].flags) & enlargedNode

		if shouldRotate {
			rotateNodes(tree, index)
		}

		index = int(nodes[index].parent)
	}
}

// removeLeaf detaches a leaf, frees its parent and repairs the ancestors.
// It corresponds to b2RemoveLeaf in src/dynamic_tree.c.
func removeLeaf(tree *dynamicTree, leaf int) {
	if leaf == tree.root {
		tree.root = nullIndex
		return
	}

	nodes := tree.nodes

	parent := int(nodes[leaf].parent)
	grandParent := int(nodes[parent].parent)
	var sibling int
	if int(nodes[parent].child1) == leaf {
		sibling = int(nodes[parent].child2)
	} else {
		sibling = int(nodes[parent].child1)
	}

	if grandParent != nullIndex {
		// Destroy parent and connect sibling to grandParent.
		if int(nodes[grandParent].child1) == parent {
			nodes[grandParent].child1 = int32(sibling)
		} else {
			nodes[grandParent].child2 = int32(sibling)
		}
		nodes[sibling].parent = int32(grandParent)
		freeNode(tree, parent)

		// Adjust ancestor bounds.
		index := grandParent
		for index != nullIndex {
			node := &nodes[index]
			child1 := &nodes[node.child1]
			child2 := &nodes[node.child2]

			node.aabb = AABBUnion(child1.aabb, child2.aabb)
			node.categoryBits = child1.categoryBits | child2.categoryBits
			node.height = 1 + max(child1.height, child2.height)

			index = int(node.parent)
		}
	} else {
		tree.root = sibling
		tree.nodes[sibling].parent = nullIndex
		freeNode(tree, parent)
	}
}

// checkTreeBox panics when a coordinate of the box leaves the huge range.
// D-003 and D-005: the reference asserts against B2_HUGE.
func checkTreeBox(aabb AABB) {
	negHuge := huge.Neg()
	ok := negHuge.Less(aabb.LowerBound.X) && aabb.LowerBound.X.Less(huge)
	ok = ok && negHuge.Less(aabb.LowerBound.Y) && aabb.LowerBound.Y.Less(huge)
	ok = ok && negHuge.Less(aabb.UpperBound.X) && aabb.UpperBound.X.Less(huge)
	ok = ok && negHuge.Less(aabb.UpperBound.Y) && aabb.UpperBound.Y.Less(huge)
	if !ok {
		panic("dbox2d: the proxy box is out of the huge range")
	}
}

// checkTreeExtent panics on an inverted box or on a box wider than huge.
func checkTreeExtent(aabb AABB) {
	if !IsValidAABB(aabb) {
		panic("dbox2d: the proxy box is not valid")
	}
	if !aabb.UpperBound.X.Sub(aabb.LowerBound.X).Less(huge) || !aabb.UpperBound.Y.Sub(aabb.LowerBound.Y).Less(huge) {
		panic("dbox2d: the proxy box is wider than the huge range")
	}
}

// checkLeaf panics when the id is not a leaf of the tree.
func checkLeaf(tree *dynamicTree, proxyId int) {
	checkNodeIndex(tree, proxyId)
	if !tree.nodes[proxyId].isLeaf() {
		panic("dbox2d: the proxy id is not a leaf")
	}
}

// createProxy inserts a leaf and returns its node index. The index stays
// valid while the pool grows. It corresponds to b2DynamicTree_CreateProxy
// in src/dynamic_tree.c.
func (tree *dynamicTree) createProxy(aabb AABB, categoryBits uint64, userData uint64) int {
	checkTreeBox(aabb)

	proxyId := allocateNode(tree)
	node := &tree.nodes[proxyId]

	node.aabb = aabb
	node.userData = userData
	node.categoryBits = categoryBits
	node.height = 0
	node.flags = allocatedNode | leafNode

	shouldRotate := true
	insertLeaf(tree, proxyId, shouldRotate)

	tree.proxyCount += 1

	return proxyId
}

// destroyProxy removes a leaf. It corresponds to
// b2DynamicTree_DestroyProxy in src/dynamic_tree.c.
func (tree *dynamicTree) destroyProxy(proxyId int) {
	checkLeaf(tree, proxyId)

	removeLeaf(tree, proxyId)
	freeNode(tree, proxyId)

	if tree.proxyCount <= 0 {
		panic("dbox2d: the tree has no proxy to destroy")
	}
	tree.proxyCount -= 1
}

// getProxyCount returns the number of leaves.
func (tree *dynamicTree) getProxyCount() int {
	return tree.proxyCount
}

// moveProxy reinserts a leaf with a new box, without rotations. It
// corresponds to b2DynamicTree_MoveProxy in src/dynamic_tree.c.
func (tree *dynamicTree) moveProxy(proxyId int, aabb AABB) {
	checkTreeExtent(aabb)
	checkLeaf(tree, proxyId)

	removeLeaf(tree, proxyId)

	tree.nodes[proxyId].aabb = aabb

	shouldRotate := false
	insertLeaf(tree, proxyId, shouldRotate)
}

// enlargeProxy grows a leaf in place and grows the ancestors that no
// longer contain it, marking them enlarged. It corresponds to
// b2DynamicTree_EnlargeProxy in src/dynamic_tree.c.
func (tree *dynamicTree) enlargeProxy(proxyId int, aabb AABB) {
	nodes := tree.nodes

	checkTreeExtent(aabb)
	checkLeaf(tree, proxyId)

	// Caller must ensure this
	if AABBContains(nodes[proxyId].aabb, aabb) {
		panic("dbox2d: enlargeProxy with a box the leaf already contains")
	}

	nodes[proxyId].aabb = aabb

	parentIndex := int(nodes[proxyId].parent)
	for parentIndex != nullIndex {
		changed := enlargeAABB(&nodes[parentIndex].aabb, aabb)
		nodes[parentIndex].flags |= enlargedNode
		parentIndex = int(nodes[parentIndex].parent)

		if !changed {
			break
		}
	}

	for parentIndex != nullIndex {
		if nodes[parentIndex].flags&enlargedNode != 0 {
			// early out because this ancestor was previously ascended and marked as enlarged
			break
		}

		nodes[parentIndex].flags |= enlargedNode
		parentIndex = int(nodes[parentIndex].parent)
	}
}

// setCategoryBits changes the bits of a leaf and repairs the ancestors.
// It corresponds to b2DynamicTree_SetCategoryBits in src/dynamic_tree.c.
func (tree *dynamicTree) setCategoryBits(proxyId int, categoryBits uint64) {
	nodes := tree.nodes

	if nodes[proxyId].child1 != nullIndex || nodes[proxyId].child2 != nullIndex || !nodes[proxyId].isLeaf() {
		panic("dbox2d: setCategoryBits on an internal node")
	}

	nodes[proxyId].categoryBits = categoryBits

	// Fix up category bits in ancestor internal nodes
	nodeIndex := int(nodes[proxyId].parent)
	for nodeIndex != nullIndex {
		node := &nodes[nodeIndex]
		child1 := node.child1
		child2 := node.child2
		if child1 == nullIndex || child2 == nullIndex {
			panic("dbox2d: an internal tree node lost a child")
		}
		node.categoryBits = nodes[child1].categoryBits | nodes[child2].categoryBits

		nodeIndex = int(node.parent)
	}
}

// getCategoryBits returns the bits of a leaf.
func (tree *dynamicTree) getCategoryBits(proxyId int) uint64 {
	checkNodeIndex(tree, proxyId)
	return tree.nodes[proxyId].categoryBits
}

// getHeight returns the height of the root, or zero for an empty tree.
func (tree *dynamicTree) getHeight() int {
	if tree.root == nullIndex {
		return 0
	}

	return int(tree.nodes[tree.root].height)
}

// getRootBounds returns the box of the root, or the empty box.
func (tree *dynamicTree) getRootBounds() AABB {
	if tree.root != nullIndex {
		return tree.nodes[tree.root].aabb
	}

	return AABB{}
}

// getUserData returns the user data of a leaf.
func (tree *dynamicTree) getUserData(proxyId int) uint64 {
	checkNodeIndex(tree, proxyId)
	return tree.nodes[proxyId].userData
}

// getAABB returns the box of a leaf.
func (tree *dynamicTree) getAABB(proxyId int) AABB {
	checkNodeIndex(tree, proxyId)
	return tree.nodes[proxyId].aabb
}

// computeHeight returns the height of a sub-tree by recursion. Only the
// validation uses it.
func computeHeight(tree *dynamicTree, nodeId int) int {
	checkNodeIndex(tree, nodeId)
	node := &tree.nodes[nodeId]

	if node.isLeaf() {
		return 0
	}

	height1 := computeHeight(tree, int(node.child1))
	height2 := computeHeight(tree, int(node.child2))
	return 1 + max(height1, height2)
}

// validateStructure checks the parent links and the enlarged flags of a
// sub-tree.
func validateStructure(tree *dynamicTree, index int) {
	if index == nullIndex {
		return
	}

	if index == tree.root {
		if tree.nodes[index].parent != nullIndex {
			panic("dbox2d: the tree root has a parent")
		}
	}

	node := &tree.nodes[index]

	if node.flags != 0 && !node.isAllocated() {
		panic("dbox2d: a tree node has flags but is not allocated")
	}

	if node.isLeaf() {
		if node.height != 0 {
			panic("dbox2d: a tree leaf has a height")
		}
		return
	}

	child1 := int(node.child1)
	child2 := int(node.child2)

	checkNodeIndex(tree, child1)
	checkNodeIndex(tree, child2)

	if int(tree.nodes[child1].parent) != index || int(tree.nodes[child2].parent) != index {
		panic("dbox2d: a tree child does not point back at its parent")
	}

	if (tree.nodes[child1].flags|tree.nodes[child2].flags)&enlargedNode != 0 {
		if node.flags&enlargedNode == 0 {
			panic("dbox2d: an enlarged child has a parent that is not enlarged")
		}
	}

	validateStructure(tree, child1)
	validateStructure(tree, child2)
}

// validateMetrics checks the heights, the bounds and the category bits of
// a sub-tree.
func validateMetrics(tree *dynamicTree, index int) {
	if index == nullIndex {
		return
	}

	node := &tree.nodes[index]

	if node.isLeaf() {
		if node.height != 0 {
			panic("dbox2d: a tree leaf has a height")
		}
		return
	}

	child1 := int(node.child1)
	child2 := int(node.child2)

	checkNodeIndex(tree, child1)
	checkNodeIndex(tree, child2)

	height1 := int(tree.nodes[child1].height)
	height2 := int(tree.nodes[child2].height)
	height := 1 + max(height1, height2)
	if int(node.height) != height {
		panic("dbox2d: a tree node height does not match its children")
	}

	if !AABBContains(node.aabb, tree.nodes[child1].aabb) || !AABBContains(node.aabb, tree.nodes[child2].aabb) {
		panic("dbox2d: a tree node does not contain its children")
	}

	categoryBits := tree.nodes[child1].categoryBits | tree.nodes[child2].categoryBits
	if node.categoryBits != categoryBits {
		panic("dbox2d: a tree node does not carry the bits of its children")
	}

	validateMetrics(tree, child1)
	validateMetrics(tree, child2)
}

// validate checks the whole tree: structure, metrics, the free list and
// the height. The reference compiles it only into validation builds; here
// only the tests call it. It corresponds to b2DynamicTree_Validate in
// src/dynamic_tree.c.
func (tree *dynamicTree) validate() {
	if tree.root == nullIndex {
		return
	}

	validateStructure(tree, tree.root)
	validateMetrics(tree, tree.root)

	freeCount := 0
	freeIndex := tree.freeList
	for freeIndex != nullIndex {
		checkNodeIndex(tree, freeIndex)
		freeIndex = int(tree.nodes[freeIndex].next)
		freeCount++
	}

	height := tree.getHeight()
	computedHeight := computeHeight(tree, tree.root)
	if height != computedHeight {
		panic("dbox2d: the tree height does not match the computed height")
	}

	if tree.nodeCount+freeCount != len(tree.nodes) {
		panic("dbox2d: the tree node count and the free list disagree")
	}
}

// validateNoEnlarged checks that no allocated node carries the enlarged
// flag. Only the tests call it. It corresponds to
// b2DynamicTree_ValidateNoEnlarged in src/dynamic_tree.c.
func (tree *dynamicTree) validateNoEnlarged() {
	for i := range tree.nodes {
		node := &tree.nodes[i]
		if node.flags&allocatedNode != 0 {
			if node.flags&enlargedNode != 0 {
				panic("dbox2d: a tree node is still enlarged")
			}
		}
	}
}
