package dbox2d_test

import (
	"testing"

	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/fixed"
)

func vec(x, y int) dbox2d.Vec2 {
	return dbox2d.Vec2{X: fixed.FromInt(x), Y: fixed.FromInt(y)}
}

// segmentDistanceCase carries exact expected values from the reference
// algorithm on integer inputs, where Q arithmetic has no rounding.
type segmentDistanceCase struct {
	name           string
	p1, q1, p2, q2 dbox2d.Vec2
	f1, f2         dbox2d.Q
	distSq         dbox2d.Q
}

// TestSegmentDistanceMatchesTheReference walks the branches of the
// closed-form algorithm: intersection, the do-over clamps of segment 2
// and parallel segments.
func TestSegmentDistanceMatchesTheReference(t *testing.T) {
	half := fixed.MustParse("0.5")
	one := fixed.One()
	zero := fixed.Zero()

	cases := []segmentDistanceCase{
		{
			// Segments cross: both fractions are interior, distance zero.
			name: "intersecting",
			p1:   vec(0, 0), q1: vec(2, 0), p2: vec(1, -1), q2: vec(1, 1),
			f1: half, f2: half, distSq: zero,
		},
		{
			// f2 starts negative: segment 2 clamps to its start and
			// segment 1 gets the do over.
			name: "do over after f2 clamps low",
			p1:   vec(0, 0), q1: vec(2, 0), p2: vec(1, 1), q2: vec(1, 3),
			f1: half, f2: zero, distSq: one,
		},
		{
			// f2 overshoots one: segment 2 clamps to its end and
			// segment 1 gets the do over.
			name: "do over after f2 clamps high",
			p1:   vec(0, 0), q1: vec(4, 0), p2: vec(5, -3), q2: vec(5, -1),
			f1: one, f2: one, distSq: fixed.FromInt(2),
		},
		{
			// Parallel segments: the denominator is zero and f1 stays
			// at the start.
			name: "parallel",
			p1:   vec(0, 0), q1: vec(2, 0), p2: vec(0, 1), q2: vec(2, 1),
			f1: zero, f2: zero, distSq: one,
		},
	}

	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			result := dbox2d.SegmentDistance(c.p1, c.q1, c.p2, c.q2)
			if !result.Fraction1.Eq(c.f1) || !result.Fraction2.Eq(c.f2) {
				t.Fatalf("fractions (%v, %v), want (%v, %v)",
					result.Fraction1, result.Fraction2, c.f1, c.f2)
			}
			if !result.DistanceSquared.Eq(c.distSq) {
				t.Fatalf("distanceSquared %v, want %v", result.DistanceSquared, c.distSq)
			}
		})
	}
}

// TestSegmentDistanceHandlesDegenerateSegments exercises the branch that
// the FLT_EPSILON guard of the reference protected. In Q an exactly zero
// squared length selects it. See D-012.
func TestSegmentDistanceHandlesDegenerateSegments(t *testing.T) {
	zero := fixed.Zero()
	one := fixed.One()

	// Segment 2 is a point past the end of segment 1: f1 clamps to one.
	point := vec(3, 4)
	result := dbox2d.SegmentDistance(vec(0, 0), vec(2, 0), point, point)
	if !result.Fraction1.Eq(one) || !result.Fraction2.Eq(zero) {
		t.Fatalf("fractions (%v, %v), want (1, 0)", result.Fraction1, result.Fraction2)
	}
	if !result.DistanceSquared.Eq(fixed.FromInt(17)) {
		t.Fatalf("distanceSquared %v, want 17", result.DistanceSquared)
	}

	// Both segments are points: the distance is between the points.
	result = dbox2d.SegmentDistance(vec(1, 2), vec(1, 2), vec(4, 6), vec(4, 6))
	if !result.Fraction1.Eq(zero) || !result.Fraction2.Eq(zero) {
		t.Fatalf("fractions (%v, %v), want (0, 0)", result.Fraction1, result.Fraction2)
	}
	if !result.DistanceSquared.Eq(fixed.FromInt(25)) {
		t.Fatalf("distanceSquared %v, want 25", result.DistanceSquared)
	}
}

// TestMakeProxyTruncatesToThePolygonLimit checks the only decision the
// constructor makes: extra points do not enter.
func TestMakeProxyTruncatesToThePolygonLimit(t *testing.T) {
	points := make([]dbox2d.Vec2, dbox2d.MaxPolygonVertices+2)
	for i := range points {
		points[i] = vec(i, -i)
	}

	radius := fixed.MustParse("0.25")
	proxy := dbox2d.MakeProxy(points, radius)
	if proxy.Count != dbox2d.MaxPolygonVertices {
		t.Fatalf("count %d, want %d", proxy.Count, dbox2d.MaxPolygonVertices)
	}
	if !proxy.Radius.Eq(radius) {
		t.Fatalf("radius %v, want %v", proxy.Radius, radius)
	}
	last := proxy.Points[dbox2d.MaxPolygonVertices-1]
	if !last.X.Eq(fixed.FromInt(dbox2d.MaxPolygonVertices - 1)) {
		t.Fatalf("last point %v, want x=%d", last, dbox2d.MaxPolygonVertices-1)
	}
}
