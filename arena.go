package dbox2d

import "unsafe"

// arenaEntry records one live allocation of the arena. It corresponds to
// b2ArenaEntry in src/arena_allocator.h.
type arenaEntry struct {
	data     []byte
	name     string
	size     int
	usedHeap bool
}

// arenaAllocator is the stack-like scratch allocator for one step.
// Allocate and free calls must nest. It corresponds to b2ArenaAllocator
// in src/arena_allocator.h; the capacity is len(data).
type arenaAllocator struct {
	data          []byte
	index         int
	allocation    int
	maxAllocation int
	entries       []arenaEntry
}

// arenaInitialEntryCapacity mirrors the initial entry array size of the
// reference.
const arenaInitialEntryCapacity = 32

// createArenaAllocator returns an arena with the given byte capacity.
func createArenaAllocator(capacity int) arenaAllocator {
	if capacity < 0 {
		panic("dbox2d: negative arena capacity")
	}
	return arenaAllocator{
		data:    make([]byte, capacity),
		entries: make([]arenaEntry, 0, arenaInitialEntryCapacity),
	}
}

// destroyArenaAllocator releases the arena storage. Go would reclaim the
// slices eventually; clearing them here preserves the explicit lifetime of
// b2DestroyArenaAllocator in src/arena_allocator.c.
func destroyArenaAllocator(a *arenaAllocator) {
	*a = arenaAllocator{}
}

// getArenaCapacity returns the number of bytes reserved by the arena.
func getArenaCapacity(a *arenaAllocator) int {
	return len(a.data)
}

// getArenaAllocation returns the number of bytes in live allocations.
func getArenaAllocation(a *arenaAllocator) int {
	return a.allocation
}

// getMaxArenaAllocation returns the largest live allocation total observed.
func getMaxArenaAllocation(a *arenaAllocator) int {
	return a.maxAllocation
}

// allocateItem returns scratch memory for the current step. The reference
// rounds every size up to a multiple of 32 bytes for SIMD alignment; the
// port keeps the rounding so the accounting numbers match.
func (a *arenaAllocator) allocateItem(size int, name string) []byte {
	if size < 0 {
		panic("dbox2d: negative arena allocation")
	}
	size32 := ((size - 1) | 0x1F) + 1

	entry := arenaEntry{name: name, size: size32}

	if a.index+size32 > len(a.data) {
		// Fall back to the heap (undesirable).
		entry.data = make([]byte, size32)
		entry.usedHeap = true
	} else {
		// The three-index slice caps the capacity so an append by the
		// caller cannot grow over the next arena entry.
		entry.data = a.data[a.index : a.index+size32 : a.index+size32]
		entry.usedHeap = false
		a.index += size32
	}

	a.allocation += size32
	if a.allocation > a.maxAllocation {
		a.maxAllocation = a.allocation
	}

	a.entries = append(a.entries, entry)
	return entry.data
}

// freeItem returns the most recent allocation. It panics when mem is not
// the top entry, because allocate and free must nest.
func (a *arenaAllocator) freeItem(mem []byte) {
	entryCount := len(a.entries)
	if entryCount == 0 {
		panic("dbox2d: free on an empty arena")
	}
	entry := &a.entries[entryCount-1]
	if len(mem) != entry.size || (entry.size > 0 && &mem[0] != &entry.data[0]) {
		panic("dbox2d: arena free out of order")
	}
	if !entry.usedHeap {
		a.index -= entry.size
	}
	a.allocation -= entry.size
	a.entries = a.entries[:entryCount-1]
}

// grow resizes the arena to fit the peak usage plus headroom. The arena
// must be empty; the reference grows between steps only.
func (a *arenaAllocator) grow() {
	if a.allocation != 0 {
		panic("dbox2d: grow on an arena in use")
	}
	if a.maxAllocation > len(a.data) {
		a.data = make([]byte, a.maxAllocation+a.maxAllocation/2)
	}
}

// arenaSlice returns a typed scratch slice of count elements over one
// arena item, plus the item for freeItem. The reference casts the bytes
// in place; the port uses unsafe.Slice so the arena accounting matches.
func arenaSlice[T any](a *arenaAllocator, count int, name string) ([]T, []byte) {
	var zero T
	mem := a.allocateItem(count*int(unsafe.Sizeof(zero)), name)
	if count == 0 {
		return nil, mem
	}
	return unsafe.Slice((*T)(unsafe.Pointer(&mem[0])), count), mem
}
