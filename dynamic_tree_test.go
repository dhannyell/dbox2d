package dbox2d

import (
	"math/rand"
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
	if !perimeter(box(-Huge.Int(), -Huge.Int(), Huge.Int(), Huge.Int())).Less(fixed.Q32MaxValue()) {
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

// TestTreeQueryReportsTheOverlaps pins query: the callback sees exactly the
// leaves that overlap the box and pass the mask, and a false return stops
// the walk after one leaf.
func TestTreeQueryReportsTheOverlaps(t *testing.T) {
	tree := createTree()
	for i := range 10 {
		bits := uint64(1)
		if i%2 == 1 {
			bits = 2
		}
		tree.createProxy(box(i*3, 0, i*3+2, 2), bits, uint64(i))
	}

	seen := map[uint64]bool{}
	stats := tree.query(box(4, 1, 13, 1), DefaultMaskBits, func(proxyId int, userData uint64) bool {
		if tree.getUserData(proxyId) != userData {
			t.Errorf("proxy %d reports user data %d", proxyId, userData)
		}
		seen[userData] = true
		return true
	})
	for _, want := range []uint64{1, 2, 3, 4} {
		if !seen[want] {
			t.Errorf("the query missed leaf %d", want)
		}
	}
	if len(seen) != 4 || stats.leafVisits != 4 {
		t.Errorf("the query saw %d leaves with %d leaf visits, want 4 and 4", len(seen), stats.leafVisits)
	}

	odd := 0
	tree.query(box(0, 0, 30, 2), 2, func(int, uint64) bool { odd++; return true })
	if odd != 5 {
		t.Errorf("the mask let %d leaves through, want 5", odd)
	}

	stats = tree.query(box(0, 0, 30, 2), DefaultMaskBits, func(int, uint64) bool { return false })
	if stats.leafVisits != 1 {
		t.Errorf("a false return let the query visit %d leaves", stats.leafVisits)
	}
}

// TestTreeRayCastClipsTheRay pins rayCast: leaves come nearest first, a
// clipped fraction hides the farther leaves, and a zero return stops the
// cast.
func TestTreeRayCastClipsTheRay(t *testing.T) {
	tree := createTree()
	for i := range 5 {
		tree.createProxy(box(i*10, 0, i*10+2, 2), DefaultCategoryBits, uint64(i))
	}

	input := RayCastInput{Origin: v2(-5, 1), Translation: v2(100, 0), MaxFraction: fixed.Q32One()}

	var order []uint64
	tree.rayCast(&input, DefaultMaskBits, func(sub *RayCastInput, proxyId int, userData uint64) Q {
		order = append(order, userData)
		return sub.MaxFraction
	})
	if len(order) != 5 || order[0] != 0 || order[4] != 4 {
		t.Errorf("the cast visited %v, want the leaves nearest first", order)
	}

	order = order[:0]
	stats := tree.rayCast(&input, DefaultMaskBits, func(sub *RayCastInput, proxyId int, userData uint64) Q {
		order = append(order, userData)
		p2 := MulAdd(sub.Origin, sub.MaxFraction, sub.Translation)
		out := aabbRayCast(tree.getAABB(proxyId), sub.Origin, p2)
		if !out.Hit {
			t.Fatalf("the ray reached leaf %d without a hit", userData)
		}
		return out.Fraction.Mul(sub.MaxFraction)
	})
	if len(order) != 1 || order[0] != 0 {
		t.Errorf("the clipped cast visited %v, want only the first leaf", order)
	}
	if stats.leafVisits != 1 {
		t.Errorf("the clipped cast counts %d leaf visits", stats.leafVisits)
	}

	visits := 0
	tree.rayCast(&input, DefaultMaskBits, func(*RayCastInput, int, uint64) Q { visits++; return fixed.Q32Zero() })
	if visits != 1 {
		t.Errorf("a zero return let the cast visit %d leaves", visits)
	}

	visits = 0
	tree.rayCast(&input, 2, func(*RayCastInput, int, uint64) Q { visits++; return fixed.Q32One() })
	if visits != 0 {
		t.Errorf("the mask let %d leaves through", visits)
	}
}

// TestTreePartitionMidSplitsTheLongestAxis pins the median split: the
// centers left of the middle of the x range come first, and a degenerate
// pair splits in half.
func TestTreePartitionMidSplitsTheLongestAxis(t *testing.T) {
	indices := []int{0, 1, 2, 3}
	centers := []Vec2{v2(0, 0), v2(10, 1), v2(1, 0), v2(11, 1)}

	split := partitionMid(indices, centers, 4)
	if split != 2 {
		t.Fatalf("the split is %d, want 2", split)
	}
	for i := range split {
		if !centers[i].X.Less(fixed.Q32FromInt(5)) {
			t.Errorf("center %d is on the right of the pivot", indices[i])
		}
	}
	for i := split; i < 4; i++ {
		if centers[i].X.Less(fixed.Q32FromInt(5)) {
			t.Errorf("center %d is on the left of the pivot", indices[i])
		}
	}

	same := []Vec2{v2(1, 1), v2(1, 1), v2(1, 1)}
	if got := partitionMid([]int{0, 1, 2}, same, 3); got != 1 {
		t.Errorf("identical centers split at %d, want 1", got)
	}
}

// TestTreePartitionSAHPicksTheCheapestPlane pins the surface area split,
// which the reference keeps under its heuristic switch: eight boxes in a
// row fill one bin each, plane three costs 176 against 188 for its
// neighbours, and the three boxes before it come first. Identical boxes
// split in half without a division.
func TestTreePartitionSAHPicksTheCheapestPlane(t *testing.T) {
	boxes := make([]AABB, 8)
	indices := make([]int, 8)
	for i := range 8 {
		// Insert out of order so the partition has work to do.
		j := (i * 5) % 8
		boxes[i] = box(3*j, 0, 3*j+1, 1)
		indices[i] = j
	}
	bins := make([]int, 8)

	split := partitionSAH(indices, bins, boxes, 8)
	if split != 3 {
		t.Fatalf("the split is %d, want 3", split)
	}
	for i := range split {
		if indices[i] >= 3 || !boxes[i].UpperBound.X.Less(fixed.Q32FromInt(9)) {
			t.Errorf("box %d sits right of plane three", indices[i])
		}
	}

	same := []AABB{box(1, 1, 2, 2), box(1, 1, 2, 2), box(1, 1, 2, 2), box(1, 1, 2, 2)}
	if got := partitionSAH([]int{0, 1, 2, 3}, make([]int, 4), same, 4); got != 2 {
		t.Errorf("identical boxes split at %d, want 2", got)
	}
}

// TestTreeRebuildKeepsEveryLeaf pins rebuild: a partial build after some
// enlargements and a full build both keep the proxies, clear the enlarged
// flags, pass validate and find the same leaves.
func TestTreeRebuildKeepsEveryLeaf(t *testing.T) {
	tree := createTree()
	ids := make([]int, 0, 100)
	for i := range 100 {
		x := (i % 10) * 3
		y := (i / 10) * 3
		ids = append(ids, tree.createProxy(box(x, y, x+2, y+2), DefaultCategoryBits, uint64(i)))
	}

	for i := 0; i < 100; i += 7 {
		aabb := tree.getAABB(ids[i])
		aabb.UpperBound = aabb.UpperBound.Add(v2(2, 2))
		tree.enlargeProxy(ids[i], aabb)
	}

	count := func() int {
		n := 0
		tree.query(box(-1, -1, 40, 40), DefaultMaskBits, func(int, uint64) bool { n++; return true })
		return n
	}

	leaves := tree.rebuild(false)
	tree.validate()
	tree.validateNoEnlarged()
	if leaves <= 0 || leaves > 100 {
		t.Errorf("the partial rebuild took %d leaves", leaves)
	}
	if tree.getProxyCount() != 100 || tree.nodeCount != 199 || count() != 100 {
		t.Errorf("the partial rebuild lost proxies: %d proxies, %d nodes, %d found", tree.getProxyCount(), tree.nodeCount, count())
	}

	heightBefore := tree.getHeight()
	if leaves = tree.rebuild(true); leaves != 100 {
		t.Errorf("the full rebuild took %d leaves, want 100", leaves)
	}
	tree.validate()
	if tree.getHeight() > 8 || tree.getHeight() > heightBefore {
		t.Errorf("the full rebuild has height %d, before %d", tree.getHeight(), heightBefore)
	}
	if count() != 100 || tree.getUserData(ids[42]) != 42 {
		t.Errorf("the full rebuild lost proxies")
	}

	// The scratch is sized once and the second build reuses it.
	if cap := len(tree.leafIndices); cap != 150 {
		t.Errorf("the rebuild scratch holds %d entries, want 150", cap)
	}

	empty := createTree()
	if empty.rebuild(true) != 0 {
		t.Errorf("an empty tree rebuilt leaves")
	}
}

// TestTreeQueryMatchesBruteForce compares query with the enumeration of
// a hundred random boxes: the same leaves for every box and mask.
func TestTreeQueryMatchesBruteForce(t *testing.T) {
	rng := rand.New(rand.NewSource(1))
	const count = 100
	tree := createTree()
	boxes := make([]AABB, count)
	categories := make([]uint64, count)
	for i := range count {
		x, y := rng.Intn(60), rng.Intn(60)
		boxes[i] = box(x, y, x+1+rng.Intn(6), y+1+rng.Intn(6))
		categories[i] = 1 << uint(rng.Intn(3))
		tree.createProxy(boxes[i], categories[i], uint64(i))
	}
	tree.validate()

	for range 50 {
		x, y := rng.Intn(60), rng.Intn(60)
		query := box(x, y, x+1+rng.Intn(20), y+1+rng.Intn(20))
		mask := uint64(rng.Intn(8))

		want := map[uint64]bool{}
		for i := range count {
			if AABBOverlaps(boxes[i], query) && categories[i]&mask != 0 {
				want[uint64(i)] = true
			}
		}
		got := map[uint64]bool{}
		tree.query(query, mask, func(_ int, userData uint64) bool {
			if got[userData] {
				t.Fatalf("query reported leaf %d twice", userData)
			}
			got[userData] = true
			return true
		})
		if len(got) != len(want) {
			t.Fatalf("query of %v with mask %d found %d leaves, brute force %d", query, mask, len(got), len(want))
		}
		for id := range want {
			if !got[id] {
				t.Fatalf("query of %v with mask %d missed leaf %d", query, mask, id)
			}
		}
	}
	destroyTree(&tree)
}

// TestTreeShapeCastMatchesBruteForce sweeps a small box through a hundred
// random boxes. The tree prunes by the swept bounds and by the separating
// axis of the sweep, so it may report a leaf the sweep only nears, but it
// must report every leaf the swept box touches, and every reported leaf
// must overlap the swept bounds and pass the mask. The oracle grows each
// leaf by the box and casts the center through it.
func TestTreeShapeCastMatchesBruteForce(t *testing.T) {
	rng := rand.New(rand.NewSource(1))
	const count = 100
	tree := createTree()
	boxes := make([]AABB, count)
	categories := make([]uint64, count)
	for i := range count {
		x, y := rng.Intn(60), rng.Intn(60)
		boxes[i] = box(x, y, x+1+rng.Intn(6), y+1+rng.Intn(6))
		categories[i] = 1 << uint(rng.Intn(3))
		tree.createProxy(boxes[i], categories[i], uint64(i))
	}
	tree.validate()

	for range 50 {
		x, y := rng.Intn(60), rng.Intn(60)
		unit := MakeBox(fixed.Q32One(), fixed.Q32One())
		proxy := MakeOffsetProxy(unit.Vertices[:unit.Count], fixed.Q32Zero(), v2(x, y), RotIdentity())
		translation := v2(rng.Intn(41)-20, rng.Intn(41)-20)
		mask := uint64(rng.Intn(8))
		input := ShapeCastInput{Proxy: proxy, Translation: translation, MaxFraction: fixed.Q32One()}

		start := box(x-1, y-1, x+1, y+1)
		swept := AABBUnion(start, AABB{LowerBound: start.LowerBound.Add(translation), UpperBound: start.UpperBound.Add(translation)})

		want := map[uint64]bool{}
		for i := range count {
			grown := AABB{LowerBound: boxes[i].LowerBound.Sub(v2(1, 1)), UpperBound: boxes[i].UpperBound.Add(v2(1, 1))}
			touches := AABBOverlaps(start, boxes[i]) || aabbRayCast(grown, v2(x, y), v2(x, y).Add(translation)).Hit
			if touches && categories[i]&mask != 0 {
				want[uint64(i)] = true
			}
		}
		got := map[uint64]bool{}
		tree.shapeCast(&input, mask, func(sub *ShapeCastInput, _ int, userData uint64) Q {
			if got[userData] {
				t.Fatalf("the cast reported leaf %d twice", userData)
			}
			got[userData] = true
			if !AABBOverlaps(boxes[userData], swept) || categories[userData]&mask == 0 {
				t.Fatalf("the cast reported leaf %d outside the swept bounds or the mask", userData)
			}
			return sub.MaxFraction
		})
		for id := range want {
			if !got[id] {
				t.Fatalf("the cast from %v by %v with mask %d missed leaf %d", v2(x, y), translation, mask, id)
			}
		}
	}
	destroyTree(&tree)
}

// TestTreeShapeCastClipsTheSweep checks that a callback that returns the
// hit fraction shortens the sweep so later leaves stay unvisited.
func TestTreeShapeCastClipsTheSweep(t *testing.T) {
	tree := createTree()
	for i := range 5 {
		tree.createProxy(box(i*10, 0, i*10+2, 2), DefaultCategoryBits, uint64(i))
	}

	unit := MakeBox(fixed.Q32Half(), fixed.Q32Half())
	proxy := MakeOffsetProxy(unit.Vertices[:unit.Count], fixed.Q32Zero(), v2(-5, 1), RotIdentity())
	input := ShapeCastInput{Proxy: proxy, Translation: v2(100, 0), MaxFraction: fixed.Q32One()}

	var order []uint64
	stats := tree.shapeCast(&input, DefaultMaskBits, func(sub *ShapeCastInput, proxyId int, userData uint64) Q {
		order = append(order, userData)
		leaf := MakeOffsetBox(fixed.Q32One(), fixed.Q32One(), AABBCenter(tree.getAABB(proxyId)), RotIdentity())
		out := ShapeCastPolygon(&ShapeCastInput{
			Proxy:       sub.Proxy,
			Translation: sub.Translation,
			MaxFraction: sub.MaxFraction,
		}, &leaf)
		if !out.Hit {
			t.Fatalf("the sweep reached leaf %d without a hit", userData)
		}
		return out.Fraction
	})
	if len(order) != 1 || order[0] != 0 || stats.leafVisits != 1 {
		t.Errorf("the clipped sweep visited %v, want only the first leaf", order)
	}
	destroyTree(&tree)
}
