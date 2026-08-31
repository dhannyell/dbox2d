package dbox2d_test

import (
	"testing"

	"github.com/dhannyell/dbox2d"
)

// TestIdRoundTrip checks the packed form of each id. An application stores
// the integer and gets the same handle back.
func TestIdRoundTrip(t *testing.T) {
	world := dbox2d.LoadWorldId(0x1234_5678)
	if got := dbox2d.StoreWorldId(world); got != 0x1234_5678 {
		t.Errorf("world id = %#x, want %#x", got, 0x1234_5678)
	}

	const packed = 0x0000_002A_0003_0007
	body := dbox2d.LoadBodyId(packed)
	if got := dbox2d.StoreBodyId(body); got != packed {
		t.Errorf("body id = %#x, want %#x", got, uint64(packed))
	}
	shape, again := dbox2d.LoadShapeId(packed), dbox2d.LoadShapeId(packed)
	if shape != again {
		t.Errorf("two ids from the same integer differ")
	}
	if got := dbox2d.StoreJointId(dbox2d.LoadJointId(packed)); got != packed {
		t.Errorf("joint id = %#x, want %#x", got, uint64(packed))
	}
	if got := dbox2d.StoreChainId(dbox2d.LoadChainId(packed)); got != packed {
		t.Errorf("chain id = %#x, want %#x", got, uint64(packed))
	}
}

// TestZeroIdIsNull fixes the rule that the zero value references nothing.
func TestZeroIdIsNull(t *testing.T) {
	if !(dbox2d.WorldId{}).IsNull() || !(dbox2d.BodyId{}).IsNull() {
		t.Errorf("a zero id is not null")
	}
	if (dbox2d.LoadBodyId(0x0000_0001_0000_0000)).IsNull() {
		t.Errorf("an id with index one is null")
	}
}
