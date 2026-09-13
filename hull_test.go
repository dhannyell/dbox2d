package b2_test

import (
	"testing"

	"github.com/dhannyell/dbox2d"
)

// pt returns the point with the given decimal coordinates.
func pt(x, y string) b2.Vec2 {
	return b2.Vec2{X: b2.QMustParse(x), Y: b2.QMustParse(y)}
}

// TestComputeHullDropsAnInteriorPoint checks the quickhull on the smallest
// case that exercises both recursions: a square with a point inside it.
func TestComputeHullDropsAnInteriorPoint(t *testing.T) {
	points := []b2.Vec2{
		pt("-1", "-1"), pt("1", "-1"), pt("1", "1"), pt("-1", "1"), pt("0", "0"),
	}

	hull := b2.ComputeHull(points)

	if hull.Count != 4 {
		t.Fatalf("hull has %d points, want 4", hull.Count)
	}
	if !b2.ValidateHull(&hull) {
		t.Errorf("ValidateHull rejects the hull of a square")
	}

	// The exact order pins the reference algorithm: the first hull point is
	// the input point farthest from the AABB center, ties keep the earliest,
	// and the stitch walks counterclockwise from there.
	want := []b2.Vec2{pt("-1", "-1"), pt("1", "-1"), pt("1", "1"), pt("-1", "1")}
	for i, w := range want {
		got := hull.Points[i]
		if !got.X.Eq(w.X) || !got.Y.Eq(w.Y) {
			t.Errorf("point %d = %v, want %v", i, got, w)
		}
	}
}

// TestComputeHullRejectsDegenerateInput pins the four documented failures.
// Each one returns an empty hull instead of a broken polygon.
func TestComputeHullRejectsDegenerateInput(t *testing.T) {
	cases := []struct {
		name   string
		points []b2.Vec2
	}{
		{"too few points", []b2.Vec2{pt("0", "0"), pt("1", "0")}},
		{"too many points", make([]b2.Vec2, b2.MaxPolygonVertices+1)},
		{"collinear points", []b2.Vec2{pt("0", "0"), pt("1", "0"), pt("2", "0")}},
		// Two points weld together, which leaves fewer than three.
		{"welded points", []b2.Vec2{pt("0", "0"), pt("0.001", "0"), pt("5", "3")}},
	}

	for _, c := range cases {
		if hull := b2.ComputeHull(c.points); hull.Count != 0 {
			t.Errorf("%s: hull has %d points, want 0", c.name, hull.Count)
		}
	}
}

// TestValidateHullRejectsAReflexVertex guards the convexity test. A hull is
// only trustworthy when it comes from ComputeHull, so the check must catch
// hand-written data.
func TestValidateHullRejectsAReflexVertex(t *testing.T) {
	hull := b2.Hull{Count: 4}
	hull.Points[0] = pt("0", "0")
	hull.Points[1] = pt("2", "0")
	hull.Points[2] = pt("1", "1")
	hull.Points[3] = pt("2", "2")

	if b2.ValidateHull(&hull) {
		t.Errorf("ValidateHull accepts a reflex vertex")
	}
}
