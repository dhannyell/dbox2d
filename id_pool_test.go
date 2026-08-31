package dbox2d

import "testing"

// TestIdPoolReusesTheLastFreedIndex pins the allocation order. The order is
// part of the result: two runs that free the same indices must hand out the
// same ones again.
func TestIdPoolReusesTheLastFreedIndex(t *testing.T) {
	pool := createIdPool()

	for want := range 4 {
		if got := pool.allocId(); got != want {
			t.Fatalf("allocId = %d, want %d", got, want)
		}
	}

	pool.freeId(1)
	pool.freeId(3)

	if got := pool.allocId(); got != 3 {
		t.Errorf("allocId after free = %d, want the last freed index 3", got)
	}
	if got := pool.allocId(); got != 1 {
		t.Errorf("allocId after free = %d, want 1", got)
	}
	if got := pool.allocId(); got != 4 {
		t.Errorf("allocId with an empty free list = %d, want 4", got)
	}
}

// TestIdPoolCounts checks the two numbers that the world reports.
func TestIdPoolCounts(t *testing.T) {
	pool := createIdPool()
	for range 3 {
		pool.allocId()
	}
	pool.freeId(0)

	if got := pool.idCount(); got != 2 {
		t.Errorf("idCount = %d, want 2", got)
	}
	if got := pool.idCapacity(); got != 3 {
		t.Errorf("idCapacity = %d, want 3", got)
	}
	if !pool.isFreeId(0) || pool.isUsedId(0) {
		t.Errorf("index 0 is not reported as free")
	}
	if !pool.isUsedId(2) {
		t.Errorf("index 2 is not reported as used")
	}

	pool.destroy()
	if pool.idCount() != 0 || pool.idCapacity() != 0 {
		t.Errorf("the pool still reports indices after destroy")
	}
}

// TestIdPoolRejectsAnUnknownIndex checks the guard that replaced an
// assertion of the reference. Freeing an index the pool never handed out
// would corrupt every later allocation.
func TestIdPoolRejectsAnUnknownIndex(t *testing.T) {
	pool := createIdPool()
	pool.allocId()

	defer func() {
		if recover() == nil {
			t.Errorf("freeing an unknown index did not panic")
		}
	}()
	pool.freeId(7)
}
