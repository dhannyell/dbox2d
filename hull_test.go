package dbox2d_test

import (
	"testing"

	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/fixed"
)

// pt returns the point with the given decimal coordinates.
func pt(x, y string) dbox2d.Vec2 {
	return dbox2d.Vec2{X: fixed.MustParse(x), Y: fixed.MustParse(y)}
}

// TestComputeHullDropsAnInteriorPoint checks the quickhull on the smallest
// case that exercises both recursions: a square with a point inside it.
func TestComputeHullDropsAnInteriorPoint(t *testing.T) {
	points := []dbox2d.Vec2{
		pt("-1", "-1"), pt("1", "-1"), pt("1", "1"), pt("-1", "1"), pt("0", "0"),
	}

	hull := dbox2d.ComputeHull(points)

	if hull.Count != 4 {
		t.Fatalf("hull has %d points, want 4", hull.Count)
	}
	if !dbox2d.ValidateHull(&hull) {
		t.Errorf("ValidateHull rejects the hull of a square")
	}

	// The exact order pins the reference algorithm: the first hull point is
	// the input point farthest from the AABB center, ties keep the earliest,
	// and the stitch walks counterclockwise from there.
	want := []dbox2d.Vec2{pt("-1", "-1"), pt("1", "-1"), pt("1", "1"), pt("-1", "1")}
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
		points []dbox2d.Vec2
	}{
		{"too few points", []dbox2d.Vec2{pt("0", "0"), pt("1", "0")}},
		{"too many points", make([]dbox2d.Vec2, dbox2d.MaxPolygonVertices+1)},
		{"collinear points", []dbox2d.Vec2{pt("0", "0"), pt("1", "0"), pt("2", "0")}},
		// Two points weld together, which leaves fewer than three.
		{"welded points", []dbox2d.Vec2{pt("0", "0"), pt("0.001", "0"), pt("5", "3")}},
	}

	for _, c := range cases {
		if hull := dbox2d.ComputeHull(c.points); hull.Count != 0 {
			t.Errorf("%s: hull has %d points, want 0", c.name, hull.Count)
		}
	}
}

// TestValidateHullRejectsAReflexVertex guards the convexity test. A hull is
// only trustworthy when it comes from ComputeHull, so the check must catch
// hand-written data.
func TestValidateHullRejectsAReflexVertex(t *testing.T) {
	hull := dbox2d.Hull{Count: 4}
	hull.Points[0] = pt("0", "0")
	hull.Points[1] = pt("2", "0")
	hull.Points[2] = pt("1", "1")
	hull.Points[3] = pt("2", "2")

	if dbox2d.ValidateHull(&hull) {
		t.Errorf("ValidateHull accepts a reflex vertex")
	}
}
