package dbox2d

// bitSet stores one bit per integer id in 64-bit blocks. The length of bits
// is the block count and the capacity is the block capacity. It corresponds
// to b2BitSet in src/bitset.h.
type bitSet struct {
	bits []uint64
}

// bitSetBlocks returns the block count that holds bitCount bits.
func bitSetBlocks(bitCount int) int {
	return (bitCount + 63) / 64
}

// createBitSet reserves room for bitCapacity bits with a block count of
// zero. It corresponds to b2CreateBitSet in src/bitset.c.
func createBitSet(bitCapacity int) bitSet {
	return bitSet{bits: make([]uint64, 0, bitSetBlocks(bitCapacity))}
}

// setBitCountAndClear sets the block count for bitCount bits and clears
// them. A short capacity grows by half. It corresponds to
// b2SetBitCountAndClear in src/bitset.c.
func setBitCountAndClear(s *bitSet, bitCount int) {
	blockCount := bitSetBlocks(bitCount)
	if cap(s.bits) < blockCount {
		newBitCapacity := bitCount + (bitCount >> 1)
		*s = createBitSet(newBitCapacity)
	}

	s.bits = s.bits[:blockCount]
	clear(s.bits)
}

// growBitSet raises the block count. It corresponds to b2GrowBitSet in
// src/bitset.c.
func growBitSet(s *bitSet, blockCount int) {
	if blockCount <= len(s.bits) {
		panic("dbox2d: the bit set does not grow")
	}
	if blockCount > cap(s.bits) {
		newBits := make([]uint64, blockCount, blockCount+blockCount/2)
		copy(newBits, s.bits)
		s.bits = newBits
		return
	}

	s.bits = s.bits[:blockCount]
}

// inPlaceUnion ors setB into setA. Both sets must have the same block
// count. It corresponds to b2InPlaceUnion in src/bitset.c.
func inPlaceUnion(setA *bitSet, setB *bitSet) {
	if len(setA.bits) != len(setB.bits) {
		panic("dbox2d: the bit sets differ in block count")
	}
	for i := range setA.bits {
		setA.bits[i] |= setB.bits[i]
	}
}

// setBit sets one bit. The bit must be inside the block count.
func (s *bitSet) setBit(bitIndex int) {
	blockIndex := bitIndex / 64
	if blockIndex >= len(s.bits) {
		panic("dbox2d: the bit is outside the bit set")
	}
	s.bits[blockIndex] |= uint64(1) << (bitIndex % 64)
}

// setBitGrow sets one bit and grows the block count when needed.
func (s *bitSet) setBitGrow(bitIndex int) {
	blockIndex := bitIndex / 64
	if blockIndex >= len(s.bits) {
		growBitSet(s, blockIndex+1)
	}
	s.bits[blockIndex] |= uint64(1) << (bitIndex % 64)
}

// clearBit clears one bit. A bit outside the block count is already clear.
func (s *bitSet) clearBit(bitIndex int) {
	blockIndex := bitIndex / 64
	if blockIndex >= len(s.bits) {
		return
	}
	s.bits[blockIndex] &^= uint64(1) << (bitIndex % 64)
}

// getBit reports one bit. A bit outside the block count is clear.
func (s *bitSet) getBit(bitIndex int) bool {
	blockIndex := bitIndex / 64
	if blockIndex >= len(s.bits) {
		return false
	}
	return s.bits[blockIndex]&(uint64(1)<<(bitIndex%64)) != 0
}
