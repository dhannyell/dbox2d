package dbox2d

import (
	"testing"

	"github.com/dhannyell/fixed"
)

// TestTreeInsertKeepsTheInvariants pins the growth path: a hundred proxies
// pass validate and the pool grew past its first sixteen nodes.
func TestTreeInsertKeepsTheInvariants(t *testing.T) {
	tree := createTree()
	ids := make([]int, 0, 100)
	for i := range 100 {
		x := (i % 10) * 3
		y := (i / 10) * 3
		id := tree.createProxy(box(x, y, x+2, y+2), DefaultCategoryBits, uint64(i))
		ids = append(ids, id)
	}
	tree.validate()

	if tree.getProxyCount() != 100 || tree.nodeCount != 199 {
		t.Fatalf("100 proxies gave %d proxies and %d nodes, want 100 and 199", tree.getProxyCount(), tree.nodeCount)
	}
	if len(tree.nodes) <= 16 {
		t.Errorf("the pool did not grow past 16 nodes")
	}
	if tree.getUserData(ids[42]) != 42 {
		t.Errorf("proxy 42 lost its user data")
	}

	for _, id := range ids {
		tree.destroyProxy(id)
	}
	tree.validate()
	if tree.getProxyCount() != 0 || tree.nodeCount != 0 || tree.root != nullIndex {
		t.Errorf("the tree is not empty after every destroy")
	}
}

// TestTreeRemoveLeafRestoresTheHeight pins removeLeaf: with three leaves the
// height is two, and removing the lone leaf under the root brings it to one.
func TestTreeRemoveLeafRestoresTheHeight(t *testing.T) {
	tree := createTree()
	a := tree.createProxy(box(0, 0, 1, 1), DefaultCategoryBits, 0)
	b := tree.createProxy(box(10, 10, 11, 11), DefaultCategoryBits, 1)
	c := tree.createProxy(box(1, 0, 2, 1), DefaultCategoryBits, 2)
	if tree.getHeight() != 2 {
		t.Fatalf("three leaves give height %d, want 2", tree.getHeight())
	}

	tree.destroyProxy(b)
	tree.validate()
	if tree.getHeight() != 1 {
		t.Errorf("after the far leaf leaves the height is %d, want 1", tree.getHeight())
	}
	want := box(0, 0, 2, 1)
	if got := tree.getRootBounds(); got != want {
		t.Errorf("the root bounds are %v, want %v", got, want)
	}

	tree.destroyProxy(a)
	tree.destroyProxy(c)
	if tree.getHeight() != 0 {
		t.Errorf("the empty tree has height %d", tree.getHeight())
	}
}

// TestTreeFindBestSiblingPicksTheCheapest pins the three leaf case computed
// by hand: the root costs 44, the near leaf 6, the far leaf 42.
func TestTreeFindBestSiblingPicksTheCheapest(t *testing.T) {
	tree := createTree()
	a := tree.createProxy(box(0, 0, 1, 1), DefaultCategoryBits, 0)
	tree.createProxy(box(10, 10, 11, 11), DefaultCategoryBits, 1)

	if got := findBestSibling(&tree, box(1, 0, 2, 1)); got != a {
		t.Errorf("the best sibling is node %d, want the near leaf %d", got, a)
	}
}

// TestTreeRotateNodesLowersTheCost pins the leaf/internal case of the
// reference: A(B, C(F, G)) with B beside F swaps B and G under C.
func TestTreeRotateNodesLowersTheCost(t *testing.T) {
	tree := createTree()
	iA := allocateNode(&tree)
	iB := allocateNode(&tree)
	iC := allocateNode(&tree)
	iF := allocateNode(&tree)
	iG := allocateNode(&tree)

	leaf := func(i int, aabb AABB) {
		tree.nodes[i].aabb = aabb
		tree.nodes[i].flags = allocatedNode | leafNode
	}
	leaf(iB, box(0, 0, 2, 2))
	leaf(iF, box(1, 1, 3, 3))
	leaf(iG, box(20, 20, 22, 22))

	tree.nodes[iC].child1, tree.nodes[iC].child2 = int32(iF), int32(iG)
	tree.nodes[iC].aabb = box(1, 1, 22, 22)
	tree.nodes[iC].height = 1
	tree.nodes[iF].parent, tree.nodes[iG].parent = int32(iC), int32(iC)

	tree.nodes[iA].child1, tree.nodes[iA].child2 = int32(iB), int32(iC)
	tree.nodes[iA].aabb = box(0, 0, 22, 22)
	tree.nodes[iA].height = 2
	tree.nodes[iB].parent, tree.nodes[iC].parent = int32(iA), int32(iA)
	tree.root = iA
	tree.proxyCount = 3
	tree.validate()

	costBefore := perimeter(tree.nodes[iC].aabb)
	rotateNodes(&tree, iA)
	tree.validate()

	if int(tree.nodes[iA].child1) != iG || int(tree.nodes[iC].child2) != iB {
		t.Fatalf("the rotation did not swap B and G")
	}
	costAfter := perimeter(tree.nodes[iC].aabb)
	if !costAfter.Less(costBefore) {
		t.Errorf("the rotation raised the cost of C from %v to %v", costBefore, costAfter)
	}
	if want := box(0, 0, 3, 3); tree.nodes[iC].aabb != want {
		t.Errorf("C covers %v, want %v", tree.nodes[iC].aabb, want)
	}
}

// TestTreeEnlargeProxyStopsAtTheContainingAncestor pins enlargeProxy: the
// leaf and its parent grow, the root already contains the new box and keeps
// its bounds, and every ancestor carries the enlarged flag.
func TestTreeEnlargeProxyStopsAtTheContainingAncestor(t *testing.T) {
	tree := createTree()
	a := tree.createProxy(box(0, 0, 1, 1), DefaultCategoryBits, 0)
	tree.createProxy(box(1, 0, 2, 1), DefaultCategoryBits, 1)
	tree.createProxy(box(10, 10, 11, 11), DefaultCategoryBits, 2)
	tree.validateNoEnlarged()

	rootBefore := tree.getRootBounds()
	parent := int(tree.nodes[a].parent)

	tree.enlargeProxy(a, box(0, 0, 1, 3))
	tree.validate()

	if got := tree.getAABB(a); got != box(0, 0, 1, 3) {
		t.Errorf("the leaf covers %v, want the new box", got)
	}
	if !AABBContains(tree.nodes[parent].aabb, box(0, 0, 1, 3)) {
		t.Errorf("the parent did not grow")
	}
	if tree.getRootBounds() != rootBefore {
		t.Errorf("the root grew although it contained the new box")
	}
	for i := parent; i != nullIndex; i = int(tree.nodes[i].parent) {
		if tree.nodes[i].flags&enlargedNode == 0 {
			t.Errorf("ancestor %d is not flagged as enlarged", i)
		}
	}
	requirePanic(t, func() { tree.validateNoEnlarged() })
}

// TestTreeSeedNeverWins extends D-009 to the sibling search: when both
// internal children contain the new box, the seed cost is unbounded and the
// centroid distance decides, so the search descends toward the near cluster.
func TestTreeSeedNeverWins(t *testing.T) {
	if !perimeter(box(-huge.Int(), -huge.Int(), huge.Int(), huge.Int())).Less(fixed.Q32MaxValue()) {
		t.Fatalf("the widest proxy box reaches the seed")
	}

	tree := createTree()
	near := []int{
		tree.createProxy(box(0, 0, 4, 4), DefaultCategoryBits, 0),
		tree.createProxy(box(0, 0, 4, 4), DefaultCategoryBits, 1),
	}
	tree.createProxy(box(0, 40, 4, 44), DefaultCategoryBits, 2)
	tree.createProxy(box(0, 40, 4, 44), DefaultCategoryBits, 3)
	tree.validate()
	if tree.getHeight() != 2 {
		t.Fatalf("two clusters give height %d, want 2", tree.getHeight())
	}

	got := findBestSibling(&tree, box(1, 1, 2, 2))
	nearParent := int(tree.nodes[near[0]].parent)
	if got != nearParent && got != near[0] && got != near[1] {
		t.Errorf("the best sibling is node %d, outside the near cluster", got)
	}
}

// TestTreeMoveProxyAndCategoryBits pins the two leaf updates: moveProxy
// reinserts the leaf under the new box, and setCategoryBits refreshes the
// bits of every ancestor.
func TestTreeMoveProxyAndCategoryBits(t *testing.T) {
	tree := createTree()
	a := tree.createProxy(box(0, 0, 1, 1), 1, 0)
	tree.createProxy(box(1, 0, 2, 1), 2, 1)
	tree.createProxy(box(10, 10, 11, 11), 4, 2)

	tree.moveProxy(a, box(11, 10, 12, 11))
	tree.validate()
	if got := tree.getAABB(a); got != box(11, 10, 12, 11) {
		t.Errorf("the moved leaf covers %v", got)
	}
	if want := box(1, 0, 12, 11); tree.getRootBounds() != want {
		t.Errorf("the root covers %v, want %v", tree.getRootBounds(), want)
	}

	tree.setCategoryBits(a, 8)
	tree.validate()
	if tree.getCategoryBits(a) != 8 {
		t.Errorf("the leaf bits are %d, want 8", tree.getCategoryBits(a))
	}
	if bits := tree.nodes[tree.root].categoryBits; bits != 2|4|8 {
		t.Errorf("the root bits are %d, want 14", bits)
	}

	destroyTree(&tree)
	if tree.nodes != nil || tree.nodeCount != 0 {
		t.Errorf("destroyTree left nodes behind")
	}
}
