package dbox2d_test

import (
	"testing"

	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/fixed"
)

// sqrtTol is the tolerance for values that pass through a square root.
// Pure additive paths assert exact values instead.
func sqrtTol() dbox2d.Q {
	return tol(1, 10_000_000)
}

func vecQ(x, y string) dbox2d.Vec2 {
	return dbox2d.Vec2{X: fixed.MustParse(x), Y: fixed.MustParse(y)}
}

func nearVec(a, b dbox2d.Vec2, limit dbox2d.Q) bool {
	return near(a.X, b.X, limit) && near(a.Y, b.Y, limit)
}

// TestCollideCirclesMatchesTheReference uses binary-exact inputs, so every
// output value is exact: no square root rounding enters.
func TestCollideCirclesMatchesTheReference(t *testing.T) {
	circleA := dbox2d.Circle{Center: vec(0, 0), Radius: fixed.One()}
	circleB := dbox2d.Circle{Center: vec(0, 0), Radius: fixed.One()}
	xfA := dbox2d.TransformIdentity()
	xfB := dbox2d.Transform{P: vecQ("1.5", "0"), Q: dbox2d.RotIdentity()}

	manifold := dbox2d.CollideCircles(&circleA, xfA, &circleB, xfB)
	if manifold.PointCount != 1 {
		t.Fatalf("pointCount %d, want 1", manifold.PointCount)
	}
	if !manifold.Normal.X.Eq(fixed.One()) || !manifold.Normal.Y.Eq(fixed.Zero()) {
		t.Fatalf("normal %v, want (1, 0)", manifold.Normal)
	}
	mp := manifold.Points[0]
	if !mp.Separation.Eq(fixed.MustParse("-0.5")) {
		t.Fatalf("separation %v, want -0.5", mp.Separation)
	}
	if !mp.Point.X.Eq(fixed.MustParse("0.75")) || !mp.Point.Y.Eq(fixed.Zero()) {
		t.Fatalf("point %v, want (0.75, 0)", mp.Point)
	}
	if mp.Id != 0 {
		t.Fatalf("id %d, want 0", mp.Id)
	}

	// A separated pair beyond the speculative distance has no manifold.
	xfB.P = vec(3, 0)
	manifold = dbox2d.CollideCircles(&circleA, xfA, &circleB, xfB)
	if manifold.PointCount != 0 {
		t.Fatalf("separated circles produced %d points", manifold.PointCount)
	}
}

// TestCollideCapsuleAndCircleRegions walks the three closest-point regions
// of the capsule axis.
func TestCollideCapsuleAndCircleRegions(t *testing.T) {
	quarter := fixed.MustParse("0.25")
	capsuleA := dbox2d.Capsule{Center1: vec(-1, 0), Center2: vec(1, 0), Radius: quarter}
	xf := dbox2d.TransformIdentity()

	// Interior region: the closest point is the projection onto the axis.
	circleB := dbox2d.Circle{Center: vecQ("0", "0.4"), Radius: quarter}
	manifold := dbox2d.CollideCapsuleAndCircle(&capsuleA, xf, &circleB, xf)
	if manifold.PointCount != 1 {
		t.Fatalf("interior: pointCount %d, want 1", manifold.PointCount)
	}
	if !nearVec(manifold.Normal, vec(0, 1), sqrtTol()) {
		t.Fatalf("interior: normal %v, want (0, 1)", manifold.Normal)
	}
	want := fixed.MustParse("0.4").Sub(fixed.Half())
	if !near(manifold.Points[0].Separation, want, sqrtTol()) {
		t.Fatalf("interior: separation %v, want %v", manifold.Points[0].Separation, want)
	}

	// The p1 region: the closest point clamps to the first center.
	circleB.Center = vecQ("-1.4", "0")
	manifold = dbox2d.CollideCapsuleAndCircle(&capsuleA, xf, &circleB, xf)
	if manifold.PointCount != 1 {
		t.Fatalf("p1 region: pointCount %d, want 1", manifold.PointCount)
	}
	if !nearVec(manifold.Normal, vec(-1, 0), sqrtTol()) {
		t.Fatalf("p1 region: normal %v, want (-1, 0)", manifold.Normal)
	}

	// The p2 region, past the speculative distance: no manifold.
	circleB.Center = vec(2, 0)
	manifold = dbox2d.CollideCapsuleAndCircle(&capsuleA, xf, &circleB, xf)
	if manifold.PointCount != 0 {
		t.Fatalf("far p2 region produced %d points", manifold.PointCount)
	}
}

// TestCollidePolygonAndCircleRegions covers the face region and the vertex
// region. The vertex branch runs only for an exactly positive separation,
// the Q form of the FLT_EPSILON guard. See D-012.
func TestCollidePolygonAndCircleRegions(t *testing.T) {
	square := dbox2d.MakeSquare(fixed.One())
	xf := dbox2d.TransformIdentity()

	// Face region: the circle floats over the top edge.
	circleB := dbox2d.Circle{Center: vecQ("0.2", "1.2"), Radius: fixed.Half()}
	manifold := dbox2d.CollidePolygonAndCircle(&square, xf, &circleB, xf)
	if manifold.PointCount != 1 {
		t.Fatalf("face: pointCount %d, want 1", manifold.PointCount)
	}
	if !manifold.Normal.X.Eq(fixed.Zero()) || !manifold.Normal.Y.Eq(fixed.One()) {
		t.Fatalf("face: normal %v, want (0, 1)", manifold.Normal)
	}
	if !manifold.Points[0].Separation.Eq(fixed.MustParse("1.2").Sub(fixed.One()).Sub(fixed.Half())) {
		t.Fatalf("face: separation %v, want -0.3", manifold.Points[0].Separation)
	}
	if !nearVec(manifold.Points[0].Point, vecQ("0.2", "0.85"), sqrtTol()) {
		t.Fatalf("face: point %v, want (0.2, 0.85)", manifold.Points[0].Point)
	}

	// Vertex region: the center sits diagonally past the (1, 1) corner and
	// the separation is positive, so the guard admits the vertex normal.
	circleB.Center = vecQ("1.3", "1.3")
	manifold = dbox2d.CollidePolygonAndCircle(&square, xf, &circleB, xf)
	if manifold.PointCount != 1 {
		t.Fatalf("vertex: pointCount %d, want 1", manifold.PointCount)
	}
	invSqrt2 := fixed.One().Div(fixed.FromInt(2).Sqrt())
	if !nearVec(manifold.Normal, dbox2d.Vec2{X: invSqrt2, Y: invSqrt2}, sqrtTol()) {
		t.Fatalf("vertex: normal %v, want (%v, %v)", manifold.Normal, invSqrt2, invSqrt2)
	}
	// separation = 0.3 * sqrt(2) - 0.5
	wantSep := fixed.MustParse("0.3").Mul(fixed.FromInt(2).Sqrt()).Sub(fixed.Half())
	if !near(manifold.Points[0].Separation, wantSep, sqrtTol()) {
		t.Fatalf("vertex: separation %v, want %v", manifold.Points[0].Separation, wantSep)
	}

	// A center exactly on the corner has zero separation: the guard sends
	// it down the face branch, the same side the reference takes for zero.
	circleB.Center = vec(1, 1)
	manifold = dbox2d.CollidePolygonAndCircle(&square, xf, &circleB, xf)
	if manifold.PointCount != 1 {
		t.Fatalf("corner: pointCount %d, want 1", manifold.PointCount)
	}
	if !manifold.Normal.X.Eq(fixed.One()) || !manifold.Normal.Y.Eq(fixed.Zero()) {
		t.Fatalf("corner: normal %v, want the face normal (1, 0)", manifold.Normal)
	}
	if !manifold.Points[0].Separation.Eq(fixed.Half().Neg()) {
		t.Fatalf("corner: separation %v, want -0.5", manifold.Points[0].Separation)
	}
}

// TestCollideCapsulesClipsTwoPoints checks the parallel clip path: two
// points, the ids of the reference and the world-space conversion.
func TestCollideCapsulesClipsTwoPoints(t *testing.T) {
	quarter := fixed.MustParse("0.25")
	capsuleA := dbox2d.Capsule{Center1: vec(-1, 0), Center2: vec(1, 0), Radius: quarter}
	capsuleB := dbox2d.Capsule{Center1: vecQ("-0.5", "0.3"), Center2: vecQ("1.5", "0.3"), Radius: quarter}
	xf := dbox2d.TransformIdentity()

	manifold := dbox2d.CollideCapsules(&capsuleA, xf, &capsuleB, xf)
	if manifold.PointCount != 2 {
		t.Fatalf("pointCount %d, want 2", manifold.PointCount)
	}
	if !manifold.Normal.X.Eq(fixed.Zero()) || !manifold.Normal.Y.Eq(fixed.One()) {
		t.Fatalf("normal %v, want (0, 1)", manifold.Normal)
	}

	wantSep := fixed.MustParse("0.3").Sub(fixed.Half())
	for i := range 2 {
		if !near(manifold.Points[i].Separation, wantSep, sqrtTol()) {
			t.Fatalf("point %d: separation %v, want %v", i, manifold.Points[i].Separation, wantSep)
		}
	}
	if manifold.Points[0].Id != 0 || manifold.Points[1].Id != 1 {
		t.Fatalf("ids (%d, %d), want (0, 1)", manifold.Points[0].Id, manifold.Points[1].Id)
	}
	if !nearVec(manifold.Points[0].Point, vecQ("-0.5", "0.15"), sqrtTol()) {
		t.Fatalf("point 0 at %v, want (-0.5, 0.15)", manifold.Points[0].Point)
	}
	if !nearVec(manifold.Points[1].Point, vecQ("1", "0.15"), sqrtTol()) {
		t.Fatalf("point 1 at %v, want (1, 0.15)", manifold.Points[1].Point)
	}
}

// TestCollideCapsulesKeepsStableIds moves the pair slightly between frames.
// The ids must not change: they are the warm-starting contract.
func TestCollideCapsulesKeepsStableIds(t *testing.T) {
	quarter := fixed.MustParse("0.25")
	capsuleA := dbox2d.Capsule{Center1: vec(-1, 0), Center2: vec(1, 0), Radius: quarter}
	capsuleB := dbox2d.Capsule{Center1: vecQ("-0.5", "0.3"), Center2: vecQ("1.5", "0.3"), Radius: quarter}
	xf := dbox2d.TransformIdentity()

	first := dbox2d.CollideCapsules(&capsuleA, xf, &capsuleB, xf)

	moved := dbox2d.Transform{P: vecQ("0.125", "-0.03125"), Q: dbox2d.RotIdentity()}
	second := dbox2d.CollideCapsules(&capsuleA, xf, &capsuleB, moved)

	if first.PointCount != 2 || second.PointCount != 2 {
		t.Fatalf("pointCounts (%d, %d), want (2, 2)", first.PointCount, second.PointCount)
	}
	for i := range 2 {
		if first.Points[i].Id != second.Points[i].Id {
			t.Fatalf("point %d changed id from %d to %d",
				i, first.Points[i].Id, second.Points[i].Id)
		}
	}
}

// TestCollideCapsulesFallsBackOnCoincidentClosestPoints exercises the G4b
// guard: collinear end-to-end capsules make the closest points coincide,
// the difference cannot be normalized and the axis perpendicular takes
// over. See D-012.
func TestCollideCapsulesFallsBackOnCoincidentClosestPoints(t *testing.T) {
	quarter := fixed.MustParse("0.25")
	capsuleA := dbox2d.Capsule{Center1: vec(-1, 0), Center2: vec(1, 0), Radius: quarter}
	capsuleB := dbox2d.Capsule{Center1: vec(1, 0), Center2: vec(3, 0), Radius: quarter}
	xf := dbox2d.TransformIdentity()

	manifold := dbox2d.CollideCapsules(&capsuleA, xf, &capsuleB, xf)
	if manifold.PointCount != 1 {
		t.Fatalf("pointCount %d, want 1", manifold.PointCount)
	}
	// LeftPerp of the axis (1, 0) is (0, 1); everything is binary exact.
	if !manifold.Normal.X.Eq(fixed.Zero()) || !manifold.Normal.Y.Eq(fixed.One()) {
		t.Fatalf("normal %v, want the perpendicular (0, 1)", manifold.Normal)
	}
	mp := manifold.Points[0]
	if !mp.Separation.Eq(fixed.Half().Neg()) {
		t.Fatalf("separation %v, want -0.5", mp.Separation)
	}
	// The contact point sits on the touching junction of the two capsules.
	if !mp.Point.X.Eq(fixed.One()) || !mp.Point.Y.Eq(fixed.Zero()) {
		t.Fatalf("point %v, want (1, 0)", mp.Point)
	}
	if mp.Id != makeIdWant(1, 0) {
		t.Fatalf("id %d, want %d: f1 clamped to the end, f2 to the start", mp.Id, makeIdWant(1, 0))
	}
}

// TestCollideCapsulesRejectsADegenerateCapsule checks the G4 guard: the
// length assert of the reference becomes a panic on an exactly zero axis.
// See D-003 and D-012.
func TestCollideCapsulesRejectsADegenerateCapsule(t *testing.T) {
	defer func() {
		if recover() == nil {
			t.Fatal("a zero-length capsule did not panic")
		}
	}()

	point := dbox2d.Capsule{Center1: vec(1, 1), Center2: vec(1, 1), Radius: fixed.Half()}
	sane := dbox2d.Capsule{Center1: vec(-1, 0), Center2: vec(1, 0), Radius: fixed.Half()}
	xf := dbox2d.TransformIdentity()
	dbox2d.CollideCapsules(&point, xf, &sane, xf)
}

// makeIdWant mirrors B2_MAKE_ID of the reference for the test expectations.
func makeIdWant(a, b int) uint16 {
	return uint16(a)<<8 | uint16(b)
}

// TestSegmentCollidersMatchTheirCapsuleForm checks the wrapper contract: a
// segment is a zero-radius capsule.
func TestSegmentCollidersMatchTheirCapsuleForm(t *testing.T) {
	segment := dbox2d.Segment{Point1: vec(-1, 0), Point2: vec(1, 0)}
	capsule := dbox2d.Capsule{Center1: vec(-1, 0), Center2: vec(1, 0)}
	xf := dbox2d.TransformIdentity()

	circle := dbox2d.Circle{Center: vecQ("0", "0.2"), Radius: quarterQ()}
	got := dbox2d.CollideSegmentAndCircle(&segment, xf, &circle, xf)
	want := dbox2d.CollideCapsuleAndCircle(&capsule, xf, &circle, xf)
	if got != want {
		t.Fatalf("segment vs circle differs from its capsule form")
	}

	other := dbox2d.Capsule{Center1: vecQ("-0.5", "0.25"), Center2: vecQ("1.5", "0.25"), Radius: quarterQ()}
	gotC := dbox2d.CollideSegmentAndCapsule(&segment, xf, &other, xf)
	wantC := dbox2d.CollideCapsules(&capsule, xf, &other, xf)
	if gotC != wantC {
		t.Fatalf("segment vs capsule differs from its capsule form")
	}
}

func quarterQ() dbox2d.Q {
	return fixed.MustParse("0.25")
}

// TestCollideChainSegmentAndCircleIsOneSided covers the one-sided cull, the
// Voronoi hand-off to the neighbor edge and the two accepting regions.
func TestCollideChainSegmentAndCircleIsOneSided(t *testing.T) {
	segment := dbox2d.ChainSegment{
		Ghost1:  vec(-2, 0),
		Segment: dbox2d.Segment{Point1: vec(-1, 0), Point2: vec(1, 0)},
		Ghost2:  vec(2, 0),
	}
	xf := dbox2d.TransformIdentity()

	// The normal points to the right of p1->p2, which is downward. A circle
	// above the segment does not collide.
	circle := dbox2d.Circle{Center: vecQ("0", "0.5"), Radius: fixed.MustParse("0.4")}
	manifold := dbox2d.CollideChainSegmentAndCircle(&segment, xf, &circle, xf)
	if manifold.PointCount != 0 {
		t.Fatalf("left side produced %d points", manifold.PointCount)
	}

	// Interior region below the segment.
	circle.Center = vecQ("0", "-0.35")
	manifold = dbox2d.CollideChainSegmentAndCircle(&segment, xf, &circle, xf)
	if manifold.PointCount != 1 {
		t.Fatalf("interior: pointCount %d, want 1", manifold.PointCount)
	}
	if !nearVec(manifold.Normal, vec(0, -1), sqrtTol()) {
		t.Fatalf("interior: normal %v, want (0, -1)", manifold.Normal)
	}
	wantSep := fixed.MustParse("0.35").Sub(fixed.MustParse("0.4"))
	if !near(manifold.Points[0].Separation, wantSep, sqrtTol()) {
		t.Fatalf("interior: separation %v, want %v", manifold.Points[0].Separation, wantSep)
	}

	// Behind point1 with a collinear previous edge: the previous segment
	// owns the region and this one yields.
	circle.Center = vecQ("-1.2", "-0.1")
	manifold = dbox2d.CollideChainSegmentAndCircle(&segment, xf, &circle, xf)
	if manifold.PointCount != 0 {
		t.Fatalf("previous-edge region produced %d points", manifold.PointCount)
	}

	// The same circle with a receding ghost vertex: the corner belongs to
	// this segment and p1 is the closest point.
	bent := segment
	bent.Ghost1 = vec(-2, 1)
	circle.Center = vecQ("-1", "-0.3")
	manifold = dbox2d.CollideChainSegmentAndCircle(&bent, xf, &circle, xf)
	if manifold.PointCount != 1 {
		t.Fatalf("p1 region: pointCount %d, want 1", manifold.PointCount)
	}
	if !nearVec(manifold.Normal, vec(0, -1), sqrtTol()) {
		t.Fatalf("p1 region: normal %v, want (0, -1)", manifold.Normal)
	}
}
