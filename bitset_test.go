package dbox2d

import "testing"

// TestBitSetGrowsAndClears pins the block accounting: the count follows the
// requested bits, the capacity grows by half, and a clear wipes the bits.
func TestBitSetGrowsAndClears(t *testing.T) {
	s := createBitSet(8)
	if len(s.bits) != 0 || cap(s.bits) != 1 {
		t.Fatalf("createBitSet(8) has %d blocks and capacity %d, want 0 and 1", len(s.bits), cap(s.bits))
	}

	setBitCountAndClear(&s, 200)
	if len(s.bits) != 4 {
		t.Fatalf("200 bits need %d blocks, want 4", len(s.bits))
	}
	if cap(s.bits) != bitSetBlocks(300) {
		t.Errorf("the capacity is %d blocks, want %d", cap(s.bits), bitSetBlocks(300))
	}

	s.setBit(199)
	if !s.getBit(199) || s.getBit(198) {
		t.Errorf("setBit(199) did not set only bit 199")
	}

	setBitCountAndClear(&s, 64)
	if len(s.bits) != 1 || s.bits[0] != 0 {
		t.Errorf("the clear left %d blocks with bits %x", len(s.bits), s.bits[0])
	}
	if s.getBit(199) {
		t.Errorf("a bit outside the count reads as set")
	}
}

// TestBitSetGrowOnSet pins setBitGrow: a bit past the count raises the
// count and keeps the earlier bits.
func TestBitSetGrowOnSet(t *testing.T) {
	s := createBitSet(64)
	setBitCountAndClear(&s, 64)
	s.setBit(3)

	s.setBitGrow(130)
	if len(s.bits) != 3 {
		t.Fatalf("setBitGrow(130) left %d blocks, want 3", len(s.bits))
	}
	if !s.getBit(3) || !s.getBit(130) {
		t.Errorf("the grow lost a bit")
	}

	s.clearBit(130)
	if s.getBit(130) {
		t.Errorf("clearBit(130) left the bit set")
	}
}

// TestBitSetUnion pins inPlaceUnion on equal block counts and its refusal
// of mismatched sets.
func TestBitSetUnion(t *testing.T) {
	a := createBitSet(128)
	b := createBitSet(128)
	setBitCountAndClear(&a, 128)
	setBitCountAndClear(&b, 128)
	a.setBit(1)
	b.setBit(100)

	inPlaceUnion(&a, &b)
	if !a.getBit(1) || !a.getBit(100) {
		t.Errorf("the union lost a bit")
	}
	if b.getBit(1) {
		t.Errorf("the union changed the second set")
	}

	c := createBitSet(64)
	setBitCountAndClear(&c, 64)
	requirePanic(t, func() { inPlaceUnion(&a, &c) })
}
