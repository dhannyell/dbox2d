package dbox2d

import (
	"math/bits"
	"unsafe"
)

// shapePairKey builds the ordered 64-bit key of a shape pair, so the pair
// (a, b) and the pair (b, a) share one key. It corresponds to
// B2_SHAPE_PAIR_KEY in src/table.h.
func shapePairKey(k1, k2 int) uint64 {
	if k1 < k2 {
		return uint64(k1)<<32 | uint64(k2)
	}
	return uint64(k2)<<32 | uint64(k1)
}

// setItem is one slot of the hash set. It corresponds to b2SetItem in
// src/table.h.
type setItem struct {
	key  uint64
	hash uint32
}

// hashSet is an open-addressing set of 64-bit keys. A key of zero is the
// empty sentinel and is not valid. The set only answers membership, so its
// internal slot order never enters a simulation result. It corresponds to
// b2HashSet in src/table.h.
type hashSet struct {
	items []setItem
	count int
}

// createSet makes a set with the capacity rounded up to a power of two.
// It corresponds to b2CreateSet in src/table.c.
func createSet(capacity int) hashSet {
	n := 16
	if capacity > 16 {
		n = 1 << bits.Len(uint(capacity-1))
	}
	return hashSet{items: make([]setItem, n)}
}

// destroySet releases the set storage. It corresponds to b2DestroySet in
// src/table.c.
func destroySet(s *hashSet) {
	s.items = nil
	s.count = 0
}

// clearSet removes every key without releasing the reserved storage. It
// corresponds to b2ClearSet in src/table.c.
func clearSet(s *hashSet) {
	clear(s.items)
	s.count = 0
}

// byteCount returns the storage occupied by the item array. It corresponds
// to b2GetHashSetBytes in src/table.c.
func (s *hashSet) byteCount() int {
	return len(s.items) * int(unsafe.Sizeof(setItem{}))
}

// keyHash mixes the key with the 64-bit Murmur finalizer. The keys come
// from pairs of small increasing integers, so a weak hash would collide.
// It corresponds to b2KeyHash in src/table.c.
func keyHash(key uint64) uint32 {
	h := key
	h ^= h >> 33
	h *= 0xff51afd7ed558ccd
	h ^= h >> 33
	h *= 0xc4ceb9fe1a85ec53
	h ^= h >> 33
	return uint32(h)
}

// findSlot probes linearly from the home slot of the hash. It corresponds
// to b2FindSlot in src/table.c.
func (s *hashSet) findSlot(key uint64, hash uint32) int {
	mask := uint32(len(s.items) - 1)
	index := hash & mask
	for s.items[index].hash != 0 && s.items[index].key != key {
		index = (index + 1) & mask
	}
	return int(index)
}

// addKeyHaveCapacity fills the slot of a key that is not in the set. It
// corresponds to b2AddKeyHaveCapacity in src/table.c.
func (s *hashSet) addKeyHaveCapacity(key uint64, hash uint32) {
	index := s.findSlot(key, hash)
	if s.items[index].hash != 0 {
		panic("dbox2d: hash set slot occupied")
	}

	s.items[index] = setItem{key: key, hash: hash}
	s.count++
}

// grow doubles the capacity and rehashes every item. It corresponds to
// b2GrowTable in src/table.c.
func (s *hashSet) grow() {
	oldItems := s.items
	oldCount := s.count

	s.count = 0
	s.items = make([]setItem, 2*len(oldItems))

	for _, item := range oldItems {
		if item.hash == 0 {
			// This slot was empty.
			continue
		}

		s.addKeyHaveCapacity(item.key, item.hash)
	}

	if s.count != oldCount {
		panic("dbox2d: hash set lost items in growth")
	}
}

// containsKey reports whether the key is in the set. It corresponds to
// b2ContainsKey in src/table.c.
func (s *hashSet) containsKey(key uint64) bool {
	if key == 0 {
		panic("dbox2d: zero hash set key")
	}

	hash := keyHash(key)
	index := s.findSlot(key, hash)
	return s.items[index].key == key
}

// addKey inserts the key and reports whether it was already in the set.
// It corresponds to b2AddKey in src/table.c.
func (s *hashSet) addKey(key uint64) bool {
	if key == 0 {
		panic("dbox2d: zero hash set key")
	}

	hash := keyHash(key)
	// A zero hash would look like an empty slot.
	if hash == 0 {
		panic("dbox2d: zero hash")
	}

	index := s.findSlot(key, hash)
	if s.items[index].hash != 0 {
		// Already in the set.
		return true
	}

	if 2*s.count >= len(s.items) {
		s.grow()
	}

	s.addKeyHaveCapacity(key, hash)
	return false
}

// removeKey removes the key and repairs the probe chain behind it. It
// reports whether the key was found. It corresponds to b2RemoveKey in
// src/table.c.
func (s *hashSet) removeKey(key uint64) bool {
	hash := keyHash(key)
	i := s.findSlot(key, hash)
	items := s.items
	if items[i].hash == 0 {
		// Not in the set.
		return false
	}

	// Mark slot i as unoccupied.
	items[i] = setItem{}

	if s.count == 0 {
		panic("dbox2d: hash set underflow")
	}
	s.count--

	// Attempt to refill slot i from the chain that follows it.
	mask := len(items) - 1
	j := i
	for {
		j = (j + 1) & mask
		if items[j].hash == 0 {
			break
		}

		// k is the home slot for the item at j.
		k := int(items[j].hash) & mask

		// Keep the item at j when its home k lies cyclically in (i, j].
		if i <= j {
			if i < k && k <= j {
				continue
			}
		} else {
			if i < k || k <= j {
				continue
			}
		}

		// Move j into i and continue behind j.
		items[i] = items[j]
		items[j] = setItem{}
		i = j
	}

	return true
}
