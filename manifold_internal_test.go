package dbox2d

import (
	"testing"

	"github.com/dhannyell/fixed"
)

// clipSegments has no production caller until the chain-segment colliders
// land, so these tests pin its contract directly.

func qv(x, y string) Vec2 {
	return Vec2{X: fixed.Q32MustParse(x), Y: fixed.Q32MustParse(y)}
}

// TestClipSegmentsLerpsBothEnds makes the incident segment overhang both
// reference ends, so both guarded lerps run. See D-012.
func TestClipSegmentsLerpsBothEnds(t *testing.T) {
	a1 := qv("0", "0")
	a2 := qv("-2", "0")
	b1 := qv("-6", "0.5")
	b2 := qv("2", "0.5")
	normal := qv("0", "1")
	zero := fixed.Q32Zero()

	manifold := clipSegments(a1, a2, b1, b2, normal, zero, zero, makeId(0, 1), makeId(1, 0))
	if manifold.PointCount != 2 {
		t.Fatalf("pointCount %d, want 2", manifold.PointCount)
	}

	sep := fixed.Q32Half()
	p0 := manifold.Points[0]
	if p0.AnchorA != qv("0", "0.25") || !p0.Separation.Eq(sep) {
		t.Fatalf("point 0 %v separation %v, want (0, 0.25) and 0.5", p0.AnchorA, p0.Separation)
	}
	p1 := manifold.Points[1]
	if p1.AnchorA != qv("-2", "0.25") || !p1.Separation.Eq(sep) {
		t.Fatalf("point 1 %v separation %v, want (-2, 0.25) and 0.5", p1.AnchorA, p1.Separation)
	}
	if p0.Id != makeId(0, 1) || p1.Id != makeId(1, 0) {
		t.Fatalf("ids %d, %d, want %d, %d", p0.Id, p1.Id, makeId(0, 1), makeId(1, 0))
	}
}

// TestClipSegmentsKeepsADegeneratePoint clips against a zero-length incident
// segment inside the reference range: the span is zero, no lerp runs and no
// division happens. See D-012.
func TestClipSegmentsKeepsADegeneratePoint(t *testing.T) {
	a1 := qv("0", "0")
	a2 := qv("-2", "0")
	point := qv("-1", "0.25")
	normal := qv("0", "1")
	zero := fixed.Q32Zero()
	quarter := fixed.Q32MustParse("0.25")

	manifold := clipSegments(a1, a2, point, point, normal, zero, quarter, makeId(0, 0), makeId(1, 1))
	if manifold.PointCount != 2 {
		t.Fatalf("pointCount %d, want 2", manifold.PointCount)
	}
	for i := range manifold.PointCount {
		p := manifold.Points[i]
		if p.AnchorA != qv("-1", "0") || !p.Separation.Eq(zero) {
			t.Fatalf("point %d %v separation %v, want (-1, 0) and 0", i, p.AnchorA, p.Separation)
		}
	}
}

// TestClipSegmentsRejectsDisjointSegments pins the early return. It bounds
// the lerp spans, so the guarded false branch stays unreachable in Q.
func TestClipSegmentsRejectsDisjointSegments(t *testing.T) {
	a1 := qv("0", "0")
	a2 := qv("-2", "0")
	point := qv("1", "0.25")
	normal := qv("0", "1")
	zero := fixed.Q32Zero()

	manifold := clipSegments(a1, a2, point, point, normal, zero, zero, makeId(0, 0), makeId(1, 1))
	if manifold.PointCount != 0 {
		t.Fatalf("disjoint segments produced %d points", manifold.PointCount)
	}
}
