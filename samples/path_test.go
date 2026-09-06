package samples

import (
	"testing"

	"github.com/dhannyell/dbox2d"
)

// TestParsePathFlipsYAndStopsAtZ pins the reference conventions: the y axis
// flips, the offset applies before the scale, and z ends the path.
func TestParsePathFlipsYAndStopsAtZ(t *testing.T) {
	offset := dbox2d.Vec2{X: dbox2d.QFromInt(-120), Y: dbox2d.QFromInt(-200)}
	points := parsePath("m 63.5,201 h 10 v -5 z l 1,1", offset, 64, dbox2d.QMustParse("0.2"))
	if len(points) != 3 {
		t.Fatalf("got %d points, want 3", len(points))
	}
	want := []dbox2d.Vec2{
		{X: dbox2d.QMustParse("-11.3"), Y: dbox2d.QMustParse("-0.2")},
		{X: dbox2d.QMustParse("-9.3"), Y: dbox2d.QMustParse("-0.2")},
		{X: dbox2d.QMustParse("-9.3"), Y: dbox2d.QMustParse("0.8")},
	}
	for i, p := range points {
		if p.Sub(want[i]).Len().Greater(dbox2d.QMustParse("0.0001")) {
			t.Errorf("point %d is %s, want %s", i, p, want[i])
		}
	}
}
