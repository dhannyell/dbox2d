package dbox2d

import "testing"

// TestArenaNestsAllocations checks the LIFO discipline and the 32-byte
// size rounding of the accounting.
func TestArenaNestsAllocations(t *testing.T) {
	arena := createArenaAllocator(128)

	first := arena.allocateItem(1, "first")
	if len(first) != 32 || arena.allocation != 32 || arena.index != 32 {
		t.Fatalf("allocate(1): len=%d allocation=%d index=%d, want 32 for all",
			len(first), arena.allocation, arena.index)
	}

	second := arena.allocateItem(33, "second")
	if len(second) != 64 || arena.index != 96 {
		t.Fatalf("allocate(33): len=%d index=%d, want 64 and 96", len(second), arena.index)
	}

	// Free out of order must panic: allocate and free pairs nest.
	requirePanic(t, func() { arena.freeItem(first) })

	arena.freeItem(second)
	arena.freeItem(first)
	if arena.index != 0 || arena.allocation != 0 || arena.maxAllocation != 96 {
		t.Fatalf("after frees: index=%d allocation=%d max=%d, want 0, 0, 96",
			arena.index, arena.allocation, arena.maxAllocation)
	}
}

// TestArenaFallsBackToHeap checks the overflow path: the entry lives on
// the heap and the arena index does not move.
func TestArenaFallsBackToHeap(t *testing.T) {
	arena := createArenaAllocator(32)

	inArena := arena.allocateItem(32, "fits")
	onHeap := arena.allocateItem(1, "overflow")
	if arena.index != 32 {
		t.Fatalf("heap fallback moved the index to %d, want 32", arena.index)
	}
	if arena.allocation != 64 || arena.maxAllocation != 64 {
		t.Fatalf("allocation=%d max=%d, want 64 and 64", arena.allocation, arena.maxAllocation)
	}

	arena.freeItem(onHeap)
	arena.freeItem(inArena)
	if arena.index != 0 || arena.allocation != 0 {
		t.Fatalf("after frees: index=%d allocation=%d, want 0 and 0", arena.index, arena.allocation)
	}
}

// TestArenaGrowsToPeakUsage checks the between-steps growth policy:
// capacity becomes the peak plus half, and later steps stay in the arena.
func TestArenaGrowsToPeakUsage(t *testing.T) {
	arena := createArenaAllocator(32)

	item := arena.allocateItem(96, "peak")
	requirePanic(t, func() { arena.grow() })
	arena.freeItem(item)

	arena.grow()
	if len(arena.data) != 96+96/2 {
		t.Fatalf("capacity after grow is %d, want %d", len(arena.data), 96+96/2)
	}

	again := arena.allocateItem(96, "reuse")
	if arena.index != 96 {
		t.Fatalf("reuse still fell back to the heap, index=%d", arena.index)
	}
	arena.freeItem(again)
}
