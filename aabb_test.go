package dbox2d

import (
	"testing"

	"github.com/dhannyell/fixed"
)

// box returns the box with the given integer corners.
func box(lx, ly, ux, uy int) AABB {
	return AABB{
		LowerBound: Vec2{X: fixed.FromInt(lx), Y: fixed.FromInt(ly)},
		UpperBound: Vec2{X: fixed.FromInt(ux), Y: fixed.FromInt(uy)},
	}
}

// TestAABBOverlapsAtTheTouchingEdge pins the boundary case. Two boxes that
// share an edge overlap, because the broadphase must report the pair before
// the shapes interpenetrate.
func TestAABBOverlapsAtTheTouchingEdge(t *testing.T) {
	a := box(0, 0, 2, 2)
	touching := box(2, 0, 4, 2)
	apart := box(3, 0, 5, 2)

	if !AABBOverlaps(a, touching) {
		t.Errorf("touching boxes do not overlap")
	}
	if AABBOverlaps(a, apart) {
		t.Errorf("separated boxes overlap")
	}
}

// TestAABBContainsIsNotStrict checks that a box contains itself, which the
// dynamic tree relies on to skip an update.
func TestAABBContainsIsNotStrict(t *testing.T) {
	a := box(0, 0, 4, 4)

	if !AABBContains(a, a) {
		t.Errorf("a box does not contain itself")
	}
	if !AABBContains(a, box(1, 1, 3, 3)) {
		t.Errorf("a box does not contain an inner box")
	}
	if AABBContains(a, box(1, 1, 5, 3)) {
		t.Errorf("a box contains a box that leaves it")
	}
}

// TestEnlargeAABBReportsGrowth checks the flag that drives the tree update.
func TestEnlargeAABBReportsGrowth(t *testing.T) {
	a := box(0, 0, 4, 4)

	if enlargeAABB(&a, box(1, 1, 2, 2)) {
		t.Errorf("an inner box reported growth")
	}
	if !enlargeAABB(&a, box(-1, 0, 4, 6)) {
		t.Errorf("an outer box reported no growth")
	}
	if want := box(-1, 0, 4, 6); a != AABBUnion(box(0, 0, 4, 4), want) {
		t.Errorf("enlarged box = %v, want the union", a)
	}
}

// TestAABBCenterAndExtentsRebuildTheBox checks the pair that the narrowphase
// uses to describe a box.
func TestAABBCenterAndExtentsRebuildTheBox(t *testing.T) {
	a := box(-2, 1, 6, 9)
	center := AABBCenter(a)
	extents := AABBExtents(a)

	rebuilt := AABB{
		LowerBound: center.Sub(extents),
		UpperBound: center.Add(extents),
	}
	if rebuilt != a {
		t.Errorf("rebuilt box = %v, want %v", rebuilt, a)
	}
	if got, want := perimeter(a), fixed.FromInt(32); !got.Eq(want) {
		t.Errorf("perimeter = %v, want %v", got, want)
	}
}

// TestMakeAABBAddsTheRadius covers the bound of a set of circles, which is
// how a capsule and a rounded polygon report their bounds.
func TestMakeAABBAddsTheRadius(t *testing.T) {
	points := []Vec2{
		{X: fixed.FromInt(1), Y: fixed.FromInt(2)},
		{X: fixed.FromInt(-3), Y: fixed.FromInt(5)},
	}

	got := MakeAABB(points, fixed.One())
	if want := box(-4, 1, 2, 6); got != want {
		t.Errorf("MakeAABB = %v, want %v", got, want)
	}
	if !IsValidAABB(got) {
		t.Errorf("MakeAABB returned an invalid box")
	}
}

// TestMakeAABBRejectsEmptyPoints covers the precondition that prevents the
// constructor from reading a missing first point.
func TestMakeAABBRejectsEmptyPoints(t *testing.T) {
	defer func() {
		if recover() == nil {
			t.Errorf("an empty point set did not panic")
		}
	}()
	MakeAABB(nil, fixed.Zero())
}

// TestIsValidAABBRejectsAnInvertedBox guards the check that catches a bad
// shape before it reaches the tree.
func TestIsValidAABBRejectsAnInvertedBox(t *testing.T) {
	if IsValidAABB(box(2, 0, 1, 3)) {
		t.Errorf("an inverted box passed validation")
	}
	if !IsValidAABB(box(1, 1, 1, 1)) {
		t.Errorf("a degenerate but ordered box failed validation")
	}
}

// TestAABBRayCastHitsTheNearFace pins the slab test. The chosen numbers are
// exact in Q32.32, so the comparison needs no tolerance.
func TestAABBRayCastHitsTheNearFace(t *testing.T) {
	a := box(0, 0, 2, 2)

	p1 := Vec2{X: fixed.FromInt(-1), Y: fixed.One()}
	p2 := Vec2{X: fixed.FromInt(3), Y: fixed.One()}

	output := AABBRayCast(a, p1, p2)

	if !output.Hit {
		t.Fatalf("the ray misses the box")
	}
	if want := fixed.MustParse("0.25"); !output.Fraction.Eq(want) {
		t.Errorf("fraction = %v, want %v", output.Fraction, want)
	}
	if !output.Point.X.Eq(fixed.Zero()) || !output.Point.Y.Eq(fixed.One()) {
		t.Errorf("point = %v, want (0, 1)", output.Point)
	}
	if !output.Normal.X.Eq(fixed.FromInt(-1)) || !output.Normal.Y.Eq(fixed.Zero()) {
		t.Errorf("normal = %v, want (-1, 0)", output.Normal)
	}

	// A ray parallel to a slab and outside it misses.
	above1 := Vec2{X: fixed.FromInt(-1), Y: fixed.FromInt(3)}
	above2 := Vec2{X: fixed.FromInt(3), Y: fixed.FromInt(3)}
	if AABBRayCast(a, above1, above2).Hit {
		t.Errorf("a ray that passes above the box reports a hit")
	}
}
