package dbox2d

import "testing"

// TestShapePairKeyIsOrderFree checks the key contract: one key per pair,
// in either order, and no clash between distinct pairs.
func TestShapePairKeyIsOrderFree(t *testing.T) {
	if shapePairKey(3, 7) != shapePairKey(7, 3) {
		t.Fatal("the pair key depends on the argument order")
	}
	if shapePairKey(3, 7) == shapePairKey(3, 8) {
		t.Fatal("distinct pairs share a key")
	}
}

// TestHashSetAddsAndRemovesKeys covers the membership contract, including
// the duplicate report and the miss report.
func TestHashSetAddsAndRemovesKeys(t *testing.T) {
	set := createSet(16)

	if set.addKey(shapePairKey(1, 2)) {
		t.Fatal("a new key reported as already present")
	}
	if !set.addKey(shapePairKey(1, 2)) {
		t.Fatal("a duplicate key reported as new")
	}
	if !set.containsKey(shapePairKey(2, 1)) {
		t.Fatal("the reversed pair is missing")
	}
	if set.count != 1 {
		t.Fatalf("count %d, want 1", set.count)
	}

	if !set.removeKey(shapePairKey(1, 2)) {
		t.Fatal("a present key was not removed")
	}
	if set.removeKey(shapePairKey(1, 2)) {
		t.Fatal("an absent key reported as removed")
	}
	if set.containsKey(shapePairKey(1, 2)) || set.count != 0 {
		t.Fatal("the set is not empty after the removal")
	}
}

// TestHashSetGrowsAndKeepsEveryKey pushes past the growth threshold and
// then removes half the keys, so the probe chain repair runs under
// collisions.
func TestHashSetGrowsAndKeepsEveryKey(t *testing.T) {
	set := createSet(16)

	const n = 200
	for i := range n {
		if set.addKey(shapePairKey(i, i+1)) {
			t.Fatalf("key %d reported as already present", i)
		}
	}
	if set.count != n {
		t.Fatalf("count %d, want %d", set.count, n)
	}
	if len(set.items)&(len(set.items)-1) != 0 {
		t.Fatalf("capacity %d is not a power of two", len(set.items))
	}

	// Remove the even pairs; every odd pair must survive the chain repair.
	for i := 0; i < n; i += 2 {
		if !set.removeKey(shapePairKey(i, i+1)) {
			t.Fatalf("key %d was not removed", i)
		}
	}
	for i := range n {
		want := i%2 == 1
		if set.containsKey(shapePairKey(i, i+1)) != want {
			t.Fatalf("key %d membership is %v, want %v", i, !want, want)
		}
	}
	if set.count != n/2 {
		t.Fatalf("count %d, want %d", set.count, n/2)
	}
}

// TestHashSetClearAndDestroy checks the two ownership operations: clear keeps
// the allocation reusable, while destroy releases it.
func TestHashSetClearAndDestroy(t *testing.T) {
	set := createSet(32)
	capacity := len(set.items)
	bytes := getHashSetBytes(&set)
	set.addKey(shapePairKey(1, 2))
	set.addKey(shapePairKey(3, 4))

	clearSet(&set)
	if set.count != 0 || len(set.items) != capacity || getHashSetBytes(&set) != bytes {
		t.Fatal("clear changed the set allocation")
	}
	for _, item := range set.items {
		if item != (setItem{}) {
			t.Fatal("clear left an occupied slot")
		}
	}
	if set.addKey(shapePairKey(5, 6)) {
		t.Fatal("a key added after clear reported as present")
	}

	destroySet(&set)
	if set.items != nil || set.count != 0 || getHashSetBytes(&set) != 0 {
		t.Fatal("destroy left set storage or accounting behind")
	}
}
