package samples

import (
	"testing"

	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/fixed"
)

// TestParsePathFlipsYAndStopsAtZ pins the reference conventions: the y axis
// flips, the offset applies before the scale, and z ends the path.
func TestParsePathFlipsYAndStopsAtZ(t *testing.T) {
	offset := dbox2d.Vec2{X: fixed.Q32FromInt(-120), Y: fixed.Q32FromInt(-200)}
	points := parsePath("m 63.5,201 h 10 v -5 z l 1,1", offset, 64, fixed.Q32MustParse("0.2"))
	if len(points) != 3 {
		t.Fatalf("got %d points, want 3", len(points))
	}
	want := []dbox2d.Vec2{
		{X: fixed.Q32MustParse("-11.3"), Y: fixed.Q32MustParse("-0.2")},
		{X: fixed.Q32MustParse("-9.3"), Y: fixed.Q32MustParse("-0.2")},
		{X: fixed.Q32MustParse("-9.3"), Y: fixed.Q32MustParse("0.8")},
	}
	for i, p := range points {
		if p.Sub(want[i]).Len().Greater(fixed.Q32MustParse("0.0001")) {
			t.Errorf("point %d is %s, want %s", i, p, want[i])
		}
	}
}
