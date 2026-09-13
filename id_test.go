package b2_test

import (
	"testing"

	"github.com/dhannyell/dbox2d"
)

// TestIdRoundTrip checks the packed form of each id. An application stores
// the integer and gets the same handle back.
func TestIdRoundTrip(t *testing.T) {
	world := b2.LoadWorldId(0x1234_5678)
	if got := b2.StoreWorldId(world); got != 0x1234_5678 {
		t.Errorf("world id = %#x, want %#x", got, 0x1234_5678)
	}

	const packed = 0x0000_002A_0003_0007
	body := b2.LoadBodyId(packed)
	if got := b2.StoreBodyId(body); got != packed {
		t.Errorf("body id = %#x, want %#x", got, uint64(packed))
	}
	shape, again := b2.LoadShapeId(packed), b2.LoadShapeId(packed)
	if shape != again {
		t.Errorf("two ids from the same integer differ")
	}
	if got := b2.StoreJointId(b2.LoadJointId(packed)); got != packed {
		t.Errorf("joint id = %#x, want %#x", got, uint64(packed))
	}
	if got := b2.StoreChainId(b2.LoadChainId(packed)); got != packed {
		t.Errorf("chain id = %#x, want %#x", got, uint64(packed))
	}
}

// TestZeroIdIsNull checks that the zero value references nothing.
func TestZeroIdIsNull(t *testing.T) {
	if !(b2.WorldId{}).IsNull() || !(b2.BodyId{}).IsNull() {
		t.Errorf("a zero id is not null")
	}
	if (b2.LoadBodyId(0x0000_0001_0000_0000)).IsNull() {
		t.Errorf("an id with index one is null")
	}
}
