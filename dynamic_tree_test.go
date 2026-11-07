package b2

import (
	"math/rand"
	"slices"
	"testing"
)

// gridTree fills a tree with a ten by ten grid of two unit boxes and
// returns the proxy ids.
func gridTree(tree *dynamicTree) []int {
	ids := make([]int, 0, 100)
	for i := range 100 {
		x := (i % 10) * 3
		y := (i / 10) * 3
		ids = append(ids, tree.createProxy(box(x, y, x+2, y+2), DefaultCategoryBits, uint64(i)))
	}
	return ids
}

// countLeaves counts the leaves that overlap the box.
func countLeaves(tree *dynamicTree, aabb AABB) int {
	n := 0
	tree.overlapAABB(aabb, DefaultMaskBits, func(int, uint64) bool { n++; return true })
	return n
}

// TestTreeInsertKeepsTheInvariants pins the growth path: a hundred proxies
// pass validate, the nodes are dense pairs, and the proxy pool grew past
// its first sixteen entries.
func TestTreeInsertKeepsTheInvariants(t *testing.T) {
	tree := createTree(16)
	ids := gridTree(&tree)
	tree.validate()

	// n leaves give 2n - 1 nodes plus the empty node beside the root.
	if tree.getProxyCount() != 100 || tree.nodeEnd != 200 {
		t.Fatalf("100 proxies gave %d proxies and node end %d, want 100 and 200", tree.getProxyCount(), tree.nodeEnd)
	}
	if len(tree.proxies) <= 16 {
		t.Errorf("the proxy pool did not grow past 16 entries")
	}
	if tree.getUserData(ids[42]) != 42 {
		t.Errorf("proxy 42 lost its user data")
	}
	if !tree.nodes[rootNode+1].isEmpty() {
		t.Errorf("the node beside the root is not empty")
	}

	for _, id := range ids {
		tree.destroyProxy(id)
	}
	tree.validate()
	if tree.getProxyCount() != 0 || !tree.nodes[rootNode].isEmpty() {
		t.Errorf("the tree is not empty after every destroy")
	}

	// The freed pairs are reused before the bump pointer moves.
	end := tree.nodeEnd
	gridTree(&tree)
	tree.validate()
	if tree.nodeEnd != end {
		t.Errorf("the reinsert moved the node end from %d to %d", end, tree.nodeEnd)
	}
}

// TestTreeRemoveLeafRestoresTheHeight pins removeLeaf: with three leaves the
// height is two, and removing the lone leaf under the root brings it to one.
func TestTreeRemoveLeafRestoresTheHeight(t *testing.T) {
	tree := createTree(16)
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
	tree := createTree(16)
	a := tree.createProxy(box(0, 0, 1, 1), DefaultCategoryBits, 0)
	tree.createProxy(box(10, 10, 11, 11), DefaultCategoryBits, 1)

	near := int(tree.proxies[a].node)
	if got := findBestSibling(&tree, box(1, 0, 2, 1)); got != near {
		t.Errorf("the best sibling is node %d, want the near leaf %d", got, near)
	}
}

// TestTreeSeedNeverWins extends D-009 to the sibling search: when both
// internal children contain the new box, the seed cost is unbounded and the
// centroid distance decides, so the search descends toward the near cluster.
func TestTreeSeedNeverWins(t *testing.T) {
	if !perimeter(box(-Huge.Int(), -Huge.Int(), Huge.Int(), Huge.Int())).Less(QMaxValue()) {
		t.Fatalf("the widest proxy box reaches the seed")
	}

	tree := createTree(16)
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
	n0, n1 := int(tree.proxies[near[0]].node), int(tree.proxies[near[1]].node)
	nearParent := int(tree.parents[n0])
	if got != nearParent && got != n0 && got != n1 {
		t.Errorf("the best sibling is node %d, outside the near cluster", got)
	}
}

// TestTreeEnlargeProxyMarksThePath pins enlargeProxy: the leaf and its
// parent grow, the root already contains the new box and keeps its bounds,
// and every ancestor carries the moved flag until clearMoved.
func TestTreeEnlargeProxyMarksThePath(t *testing.T) {
	tree := createTree(16)
	a := tree.createProxy(box(0, 0, 1, 1), DefaultCategoryBits, 0)
	tree.createProxy(box(1, 0, 2, 1), DefaultCategoryBits, 1)
	tree.createProxy(box(10, 10, 11, 11), DefaultCategoryBits, 2)
	tree.validateNoMoved()

	rootBefore := tree.getRootBounds()
	leaf := int(tree.proxies[a].node)
	parent := int(tree.parents[leaf])

	tree.enlargeProxy(a, box(0, 0, 1, 3))
	tree.validate()

	if got := tree.getProxyBounds(a); got != box(0, 0, 1, 3) {
		t.Errorf("the leaf covers %v, want the new box", got)
	}
	if !AABBContains(tree.nodes[parent].aabb, box(0, 0, 1, 3)) {
		t.Errorf("the parent did not grow")
	}
	if tree.getRootBounds() != rootBefore {
		t.Errorf("the root grew although it contained the new box")
	}
	for i := leaf; i != nullIndex; i = int(tree.parents[i]) {
		if !tree.nodes[i].isMoved() {
			t.Errorf("node %d on the path is not flagged as moved", i)
		}
	}
	if got := tree.gatherMovedProxies(nil); !slices.Equal(got, []int{a}) {
		t.Errorf("the moved proxies are %v, want [%d]", got, a)
	}
	requirePanic(t, func() { tree.validateNoMoved() })
	requirePanic(t, func() { tree.enlargeProxy(a, box(0, 0, 1, 2)) })

	tree.clearMoved()
	tree.validateNoMoved()
}

// TestTreeMarkProxyMovedRefits pins the moved path the solver takes:
// markProxyMoved writes the leaf box and flags the ancestors, refit grows
// the internal boxes, and gather finds exactly the marked proxies. The
// sweep refit runs on a rebuilt tree and the subtree refit on an
// unordered one.
func TestTreeMarkProxyMovedRefits(t *testing.T) {
	for _, ordered := range []bool{true, false} {
		tree := createTree(16)
		ids := gridTree(&tree)
		tree.rebuild(true)
		if !tree.dfsOrdered {
			t.Fatalf("the rebuild left the tree unordered")
		}
		tree.dfsOrdered = ordered

		moved := []int{ids[3], ids[57], ids[99]}
		for k, id := range moved {
			aabb := tree.getProxyBounds(id)
			aabb.UpperBound = aabb.UpperBound.Add(v2(40+k, 40))
			tree.markProxyMoved(id, aabb)
		}
		if !tree.needsRebuild() {
			t.Errorf("ordered %v: the marked tree does not need a rebuild", ordered)
		}

		tree.refit()
		tree.validate()

		if want := box(0, 0, 71, 69); tree.getRootBounds() != want {
			t.Errorf("ordered %v: the root covers %v, want %v", ordered, tree.getRootBounds(), want)
		}
		got := tree.gatherMovedProxies(nil)
		slices.Sort(got)
		want := slices.Clone(moved)
		slices.Sort(want)
		if !slices.Equal(got, want) {
			t.Errorf("ordered %v: the moved proxies are %v, want %v", ordered, got, want)
		}
		if n := countLeaves(&tree, box(45, 40, 46, 41)); n != 3 {
			t.Errorf("ordered %v: the grown corner finds %d leaves, want 3", ordered, n)
		}

		tree.clearMoved()
		tree.validateNoMoved()
		if tree.hasMoved() || len(tree.gatherMovedProxies(nil)) != 0 {
			t.Errorf("ordered %v: clearMoved left flags behind", ordered)
		}
	}
}

// TestTreeMoveProxyAndCategoryBits pins the two leaf updates: moveProxy
// reinserts the leaf under the new box, and setCategoryBits writes the
// proxy, which the query mask reads.
func TestTreeMoveProxyAndCategoryBits(t *testing.T) {
	tree := createTree(16)
	a := tree.createProxy(box(0, 0, 1, 1), 1, 0)
	tree.createProxy(box(1, 0, 2, 1), 2, 1)
	tree.createProxy(box(10, 10, 11, 11), 4, 2)

	tree.moveProxy(a, box(11, 10, 12, 11))
	tree.validate()
	tree.validateNoMoved()
	if got := tree.getProxyBounds(a); got != box(11, 10, 12, 11) {
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
	n := 0
	tree.overlapAABB(box(0, 0, 20, 20), 8, func(int, uint64) bool { n++; return true })
	if n != 1 {
		t.Errorf("the mask 8 found %d leaves, want 1", n)
	}

	destroyTree(&tree)
	if tree.nodes != nil || tree.proxyCount != 0 {
		t.Errorf("destroyTree left nodes behind")
	}
}

// TestTreeOverlapReportsTheOverlaps pins overlapAABB: the callback sees
// exactly the leaves that overlap the box and pass the mask, and a false
// return stops the walk after one leaf. overlapAABBAll skips the mask.
func TestTreeOverlapReportsTheOverlaps(t *testing.T) {
	tree := createTree(16)
	for i := range 10 {
		bits := uint64(1)
		if i%2 == 1 {
			bits = 2
		}
		tree.createProxy(box(i*3, 0, i*3+2, 2), bits, uint64(i))
	}

	seen := map[uint64]bool{}
	stats := tree.overlapAABB(box(4, 1, 13, 1), DefaultMaskBits, func(proxyId int, userData uint64) bool {
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
	tree.overlapAABB(box(0, 0, 30, 2), 2, func(int, uint64) bool { odd++; return true })
	if odd != 5 {
		t.Errorf("the mask let %d leaves through, want 5", odd)
	}

	all := 0
	tree.overlapAABBAll(box(0, 0, 30, 2), func(int, uint64) bool { all++; return true })
	if all != 10 {
		t.Errorf("overlapAABBAll found %d leaves, want 10", all)
	}

	stats = tree.overlapAABB(box(0, 0, 30, 2), DefaultMaskBits, func(int, uint64) bool { return false })
	if stats.leafVisits != 1 {
		t.Errorf("a false return let the query visit %d leaves", stats.leafVisits)
	}
}

// TestTreeCastRayClipsTheRay pins castRay: every leaf on the ray is
// reached, a clipped fraction hides the leaves outside the pair of the
// hit, and a zero return stops the cast. Both leaves of a pair are
// reported before the clip applies, as in the reference.
func TestTreeCastRayClipsTheRay(t *testing.T) {
	tree := createTree(16)
	for i := range 5 {
		tree.createProxy(box(i*10, 0, i*10+2, 2), DefaultCategoryBits, uint64(i))
	}

	input := RayCastInput{Origin: v2(-5, 1), Translation: v2(100, 0), MaxFraction: QOne()}

	var order []uint64
	tree.castRay(&input, DefaultMaskBits, func(sub *RayCastInput, proxyId int, userData uint64) Q {
		order = append(order, userData)
		return sub.MaxFraction
	})
	sorted := slices.Clone(order)
	slices.Sort(sorted)
	if !slices.Equal(sorted, []uint64{0, 1, 2, 3, 4}) {
		t.Errorf("the cast visited %v, want every leaf once", order)
	}

	order = order[:0]
	stats := tree.castRay(&input, DefaultMaskBits, func(sub *RayCastInput, proxyId int, userData uint64) Q {
		order = append(order, userData)
		p2 := MulAdd(sub.Origin, sub.MaxFraction, sub.Translation)
		out := aabbRayCast(tree.getProxyBounds(proxyId), sub.Origin, p2)
		if !out.Hit {
			return QFromInt(-1)
		}
		return out.Fraction.Mul(sub.MaxFraction)
	})
	if len(order) == 0 || len(order) > 2 || !slices.Contains(order, 0) {
		t.Errorf("the clipped cast visited %v, want the first leaf and at most its sibling", order)
	}
	if stats.leafVisits != len(order) {
		t.Errorf("the clipped cast counts %d leaf visits", stats.leafVisits)
	}

	visits := 0
	tree.castRay(&input, DefaultMaskBits, func(*RayCastInput, int, uint64) Q { visits++; return QZero() })
	if visits != 1 {
		t.Errorf("a zero return let the cast visit %d leaves", visits)
	}

	visits = 0
	tree.castRay(&input, 2, func(*RayCastInput, int, uint64) Q { visits++; return QOne() })
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
		if !centers[i].X.Less(QFromInt(5)) {
			t.Errorf("center %d is on the right of the pivot", indices[i])
		}
	}
	for i := split; i < 4; i++ {
		if centers[i].X.Less(QFromInt(5)) {
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
		if indices[i] >= 3 || !boxes[i].UpperBound.X.Less(QFromInt(9)) {
			t.Errorf("box %d sits right of plane three", indices[i])
		}
	}

	same := []AABB{box(1, 1, 2, 2), box(1, 1, 2, 2), box(1, 1, 2, 2), box(1, 1, 2, 2)}
	if got := partitionSAH([]int{0, 1, 2, 3}, make([]int, 4), same, 4); got != 2 {
		t.Errorf("identical boxes split at %d, want 2", got)
	}
}

// TestTreeRebuildKeepsEveryLeaf pins rebuild: a partial build after some
// moves and a full build both keep the proxies, clear the moved flags,
// restore the order, pass validate and find the same leaves.
func TestTreeRebuildKeepsEveryLeaf(t *testing.T) {
	tree := createTree(16)
	ids := gridTree(&tree)

	for i := 0; i < 100; i += 7 {
		aabb := tree.getProxyBounds(ids[i])
		aabb.UpperBound = aabb.UpperBound.Add(v2(2, 2))
		tree.markProxyMoved(ids[i], aabb)
	}
	tree.refit()

	all := box(-1, -1, 40, 40)
	leaves := tree.rebuild(false)
	tree.validate()
	tree.validateNoMoved()
	if leaves <= 0 || leaves > 100 {
		t.Errorf("the partial rebuild took %d leaves", leaves)
	}
	if tree.getProxyCount() != 100 || tree.nodeEnd != 200 || countLeaves(&tree, all) != 100 {
		t.Errorf("the partial rebuild lost proxies: %d proxies, node end %d, %d found", tree.getProxyCount(), tree.nodeEnd, countLeaves(&tree, all))
	}
	if !tree.dfsOrdered {
		t.Errorf("the partial rebuild left the tree unordered")
	}
	if tree.rebuild(false) != 0 {
		t.Errorf("a clean ordered tree rebuilt again")
	}

	heightBefore := tree.getHeight()
	if leaves = tree.rebuild(true); leaves != 100 {
		t.Errorf("the full rebuild took %d leaves, want 100", leaves)
	}
	tree.validate()
	if tree.getHeight() > 8 || tree.getHeight() > heightBefore {
		t.Errorf("the full rebuild has height %d, before %d", tree.getHeight(), heightBefore)
	}
	if countLeaves(&tree, all) != 100 || tree.getUserData(ids[42]) != 42 {
		t.Errorf("the full rebuild lost proxies")
	}
	for _, id := range ids {
		node := int(tree.proxies[id].node)
		if !tree.nodes[node].isLeaf() || tree.nodes[node].proxyId() != id {
			t.Fatalf("proxy %d does not point at its leaf", id)
		}
	}

	// The scratch is sized once and the second build reuses it.
	if n := len(tree.leafIndices); n != 150 {
		t.Errorf("the rebuild scratch holds %d entries, want 150", n)
	}

	empty := createTree(16)
	if empty.rebuild(true) != 0 {
		t.Errorf("an empty tree rebuilt leaves")
	}
}

// TestTreeOverlapMatchesBruteForce compares overlapAABB with the
// enumeration of a hundred random boxes: the same leaves for every box and
// mask.
func TestTreeOverlapMatchesBruteForce(t *testing.T) {
	rng := rand.New(rand.NewSource(1))
	const count = 100
	tree := createTree(16)
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
		tree.overlapAABB(query, mask, func(_ int, userData uint64) bool {
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

// TestTreeCastBoxMatchesBruteForce sweeps a small box through a hundred
// random boxes. The tree prunes by the swept bounds and by the separating
// axis of the sweep, so it may report a leaf the sweep only nears, but it
// must report every leaf the swept box touches, and every reported leaf
// must overlap the swept bounds and pass the mask. The oracle grows each
// leaf by the box and casts the center through it.
func TestTreeCastBoxMatchesBruteForce(t *testing.T) {
	rng := rand.New(rand.NewSource(1))
	const count = 100
	tree := createTree(16)
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
		translation := v2(rng.Intn(41)-20, rng.Intn(41)-20)
		mask := uint64(rng.Intn(8))
		start := box(x-1, y-1, x+1, y+1)
		input := BoxCastInput{Box: start, Translation: translation, MaxFraction: QOne()}

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
		tree.castBox(&input, mask, func(sub *BoxCastInput, _ int, userData uint64) Q {
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

// TestTreeCastBoxClipsTheSweep checks that a callback that returns the
// hit fraction shortens the sweep so the leaves outside the pair of the
// hit stay unvisited.
func TestTreeCastBoxClipsTheSweep(t *testing.T) {
	tree := createTree(16)
	for i := range 5 {
		tree.createProxy(box(i*10, 0, i*10+2, 2), DefaultCategoryBits, uint64(i))
	}

	start := box(-6, 0, -4, 2)
	input := BoxCastInput{Box: start, Translation: v2(100, 0), MaxFraction: QOne()}
	center := AABBCenter(start)

	var order []uint64
	stats := tree.castBox(&input, DefaultMaskBits, func(sub *BoxCastInput, proxyId int, userData uint64) Q {
		order = append(order, userData)
		leaf := tree.getProxyBounds(proxyId)
		grown := AABB{LowerBound: leaf.LowerBound.Sub(v2(1, 1)), UpperBound: leaf.UpperBound.Add(v2(1, 1))}
		p2 := MulAdd(center, sub.MaxFraction, sub.Translation)
		out := aabbRayCast(grown, center, p2)
		if !out.Hit {
			return QFromInt(-1)
		}
		return out.Fraction.Mul(sub.MaxFraction)
	})
	if len(order) == 0 || len(order) > 2 || !slices.Contains(order, 0) || stats.leafVisits != len(order) {
		t.Errorf("the clipped sweep visited %v, want the first leaf and at most its sibling", order)
	}
	destroyTree(&tree)
}
