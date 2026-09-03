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
	return dbox2d.Vec2{X: fixed.Q32MustParse(x), Y: fixed.Q32MustParse(y)}
}

func nearVec(a, b dbox2d.Vec2, limit dbox2d.Q) bool {
	return near(a.X, b.X, limit) && near(a.Y, b.Y, limit)
}

// TestCollideCirclesMatchesTheReference uses binary-exact inputs, so every
// output value is exact: no square root rounding enters.
func TestCollideCirclesMatchesTheReference(t *testing.T) {
	circleA := dbox2d.Circle{Center: vec(0, 0), Radius: fixed.Q32One()}
	circleB := dbox2d.Circle{Center: vec(0, 0), Radius: fixed.Q32One()}
	xfA := dbox2d.TransformIdentity()
	xfB := dbox2d.Transform{P: vecQ("1.5", "0"), Q: dbox2d.RotIdentity()}

	manifold := dbox2d.CollideCircles(&circleA, xfA, &circleB, xfB)
	if manifold.PointCount != 1 {
		t.Fatalf("pointCount %d, want 1", manifold.PointCount)
	}
	if !manifold.Normal.X.Eq(fixed.Q32One()) || !manifold.Normal.Y.Eq(fixed.Q32Zero()) {
		t.Fatalf("normal %v, want (1, 0)", manifold.Normal)
	}
	mp := manifold.Points[0]
	if !mp.Separation.Eq(fixed.Q32MustParse("-0.5")) {
		t.Fatalf("separation %v, want -0.5", mp.Separation)
	}
	if !mp.Point.X.Eq(fixed.Q32MustParse("0.75")) || !mp.Point.Y.Eq(fixed.Q32Zero()) {
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
	quarter := fixed.Q32MustParse("0.25")
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
	want := fixed.Q32MustParse("0.4").Sub(fixed.Q32Half())
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
	square := dbox2d.MakeSquare(fixed.Q32One())
	xf := dbox2d.TransformIdentity()

	// Face region: the circle floats over the top edge.
	circleB := dbox2d.Circle{Center: vecQ("0.2", "1.2"), Radius: fixed.Q32Half()}
	manifold := dbox2d.CollidePolygonAndCircle(&square, xf, &circleB, xf)
	if manifold.PointCount != 1 {
		t.Fatalf("face: pointCount %d, want 1", manifold.PointCount)
	}
	if !manifold.Normal.X.Eq(fixed.Q32Zero()) || !manifold.Normal.Y.Eq(fixed.Q32One()) {
		t.Fatalf("face: normal %v, want (0, 1)", manifold.Normal)
	}
	if !manifold.Points[0].Separation.Eq(fixed.Q32MustParse("1.2").Sub(fixed.Q32One()).Sub(fixed.Q32Half())) {
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
	invSqrt2 := fixed.Q32One().Div(fixed.Q32FromInt(2).Sqrt())
	if !nearVec(manifold.Normal, dbox2d.Vec2{X: invSqrt2, Y: invSqrt2}, sqrtTol()) {
		t.Fatalf("vertex: normal %v, want (%v, %v)", manifold.Normal, invSqrt2, invSqrt2)
	}
	// separation = 0.3 * sqrt(2) - 0.5
	wantSep := fixed.Q32MustParse("0.3").Mul(fixed.Q32FromInt(2).Sqrt()).Sub(fixed.Q32Half())
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
	if !manifold.Normal.X.Eq(fixed.Q32One()) || !manifold.Normal.Y.Eq(fixed.Q32Zero()) {
		t.Fatalf("corner: normal %v, want the face normal (1, 0)", manifold.Normal)
	}
	if !manifold.Points[0].Separation.Eq(fixed.Q32Half().Neg()) {
		t.Fatalf("corner: separation %v, want -0.5", manifold.Points[0].Separation)
	}
}

// TestCollideCapsulesClipsTwoPoints checks the parallel clip path: two
// points, the ids of the reference and the world-space conversion.
func TestCollideCapsulesClipsTwoPoints(t *testing.T) {
	quarter := fixed.Q32MustParse("0.25")
	capsuleA := dbox2d.Capsule{Center1: vec(-1, 0), Center2: vec(1, 0), Radius: quarter}
	capsuleB := dbox2d.Capsule{Center1: vecQ("-0.5", "0.3"), Center2: vecQ("1.5", "0.3"), Radius: quarter}
	xf := dbox2d.TransformIdentity()

	manifold := dbox2d.CollideCapsules(&capsuleA, xf, &capsuleB, xf)
	if manifold.PointCount != 2 {
		t.Fatalf("pointCount %d, want 2", manifold.PointCount)
	}
	if !manifold.Normal.X.Eq(fixed.Q32Zero()) || !manifold.Normal.Y.Eq(fixed.Q32One()) {
		t.Fatalf("normal %v, want (0, 1)", manifold.Normal)
	}

	wantSep := fixed.Q32MustParse("0.3").Sub(fixed.Q32Half())
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
	quarter := fixed.Q32MustParse("0.25")
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
	quarter := fixed.Q32MustParse("0.25")
	capsuleA := dbox2d.Capsule{Center1: vec(-1, 0), Center2: vec(1, 0), Radius: quarter}
	capsuleB := dbox2d.Capsule{Center1: vec(1, 0), Center2: vec(3, 0), Radius: quarter}
	xf := dbox2d.TransformIdentity()

	manifold := dbox2d.CollideCapsules(&capsuleA, xf, &capsuleB, xf)
	if manifold.PointCount != 1 {
		t.Fatalf("pointCount %d, want 1", manifold.PointCount)
	}
	// LeftPerp of the axis (1, 0) is (0, 1); everything is binary exact.
	if !manifold.Normal.X.Eq(fixed.Q32Zero()) || !manifold.Normal.Y.Eq(fixed.Q32One()) {
		t.Fatalf("normal %v, want the perpendicular (0, 1)", manifold.Normal)
	}
	mp := manifold.Points[0]
	if !mp.Separation.Eq(fixed.Q32Half().Neg()) {
		t.Fatalf("separation %v, want -0.5", mp.Separation)
	}
	// The contact point sits on the touching junction of the two capsules.
	if !mp.Point.X.Eq(fixed.Q32One()) || !mp.Point.Y.Eq(fixed.Q32Zero()) {
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

	point := dbox2d.Capsule{Center1: vec(1, 1), Center2: vec(1, 1), Radius: fixed.Q32Half()}
	sane := dbox2d.Capsule{Center1: vec(-1, 0), Center2: vec(1, 0), Radius: fixed.Q32Half()}
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
	return fixed.Q32MustParse("0.25")
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
	circle := dbox2d.Circle{Center: vecQ("0", "0.5"), Radius: fixed.Q32MustParse("0.4")}
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
	wantSep := fixed.Q32MustParse("0.35").Sub(fixed.Q32MustParse("0.4"))
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

// TestCollidePolygonsClipsOverlappingBoxes checks the parallel-face overlap
// branch of the clipper: two points, exact values, stable ids.
func TestCollidePolygonsClipsOverlappingBoxes(t *testing.T) {
	boxA := dbox2d.MakeBox(fixed.Q32One(), fixed.Q32One())
	boxB := dbox2d.MakeBox(fixed.Q32One(), fixed.Q32One())
	xfA := dbox2d.TransformIdentity()
	xfB := dbox2d.Transform{P: vecQ("0", "1.5"), Q: dbox2d.RotIdentity()}

	manifold := dbox2d.CollidePolygons(&boxA, xfA, &boxB, xfB)
	if manifold.PointCount != 2 {
		t.Fatalf("pointCount %d, want 2", manifold.PointCount)
	}
	if !manifold.Normal.X.Eq(fixed.Q32Zero()) || !manifold.Normal.Y.Eq(fixed.Q32One()) {
		t.Fatalf("normal %v, want (0, 1)", manifold.Normal)
	}

	negHalf := fixed.Q32MustParse("-0.5")
	p0 := manifold.Points[0]
	if p0.Point != vecQ("1", "0.75") || !p0.Separation.Eq(negHalf) {
		t.Fatalf("point 0 %v separation %v, want (1, 0.75) and -0.5", p0.Point, p0.Separation)
	}
	if p0.Id != makeIdWant(2, 1) {
		t.Fatalf("point 0 id %d, want %d", p0.Id, makeIdWant(2, 1))
	}

	p1 := manifold.Points[1]
	if p1.Point != vecQ("-1", "0.75") || !p1.Separation.Eq(negHalf) {
		t.Fatalf("point 1 %v separation %v, want (-1, 0.75) and -0.5", p1.Point, p1.Separation)
	}
	if p1.Id != makeIdWant(3, 0) {
		t.Fatalf("point 1 id %d, want %d", p1.Id, makeIdWant(3, 0))
	}
}

// TestCollidePolygonsClipsThePartialOverlap puts a small box on the corner
// side of the top face, so the lower clip point comes from the guarded lerp.
func TestCollidePolygonsClipsThePartialOverlap(t *testing.T) {
	boxA := dbox2d.MakeBox(fixed.Q32One(), fixed.Q32One())
	boxB := dbox2d.MakeBox(fixed.Q32Half(), fixed.Q32Half())
	xfA := dbox2d.TransformIdentity()
	xfB := dbox2d.Transform{P: vecQ("0.75", "1.25"), Q: dbox2d.RotIdentity()}

	manifold := dbox2d.CollidePolygons(&boxA, xfA, &boxB, xfB)
	if manifold.PointCount != 2 {
		t.Fatalf("pointCount %d, want 2", manifold.PointCount)
	}
	if !manifold.Normal.X.Eq(fixed.Q32Zero()) || !manifold.Normal.Y.Eq(fixed.Q32One()) {
		t.Fatalf("normal %v, want (0, 1)", manifold.Normal)
	}

	sep := fixed.Q32MustParse("-0.25")
	p0 := manifold.Points[0]
	if p0.Point != vecQ("1", "0.875") || !p0.Separation.Eq(sep) {
		t.Fatalf("point 0 %v separation %v, want (1, 0.875) and -0.25", p0.Point, p0.Separation)
	}
	p1 := manifold.Points[1]
	if p1.Point != vecQ("0.25", "0.875") || !p1.Separation.Eq(sep) {
		t.Fatalf("point 1 %v separation %v, want (0.25, 0.875) and -0.25", p1.Point, p1.Separation)
	}
	if p0.Id != makeIdWant(2, 1) || p1.Id != makeIdWant(3, 0) {
		t.Fatalf("ids %d, %d, want %d, %d", p0.Id, p1.Id, makeIdWant(2, 1), makeIdWant(3, 0))
	}
}

// TestCollidePolygonsFindsTheVertexVertexContact offsets a box past the
// corner, so the clipper finds disjoint edges and the closest vertices win.
func TestCollidePolygonsFindsTheVertexVertexContact(t *testing.T) {
	boxA := dbox2d.MakeBox(fixed.Q32One(), fixed.Q32One())
	boxB := dbox2d.MakeBox(fixed.Q32One(), fixed.Q32One())
	xfA := dbox2d.TransformIdentity()

	// The offset 1/128 keeps the corner gap inside the speculative distance.
	offset := fixed.Q32FromInt(2).Add(fixed.Q32MustParse("0.0078125"))
	xfB := dbox2d.Transform{P: dbox2d.Vec2{X: offset, Y: offset}, Q: dbox2d.RotIdentity()}

	manifold := dbox2d.CollidePolygons(&boxA, xfA, &boxB, xfB)
	if manifold.PointCount != 1 {
		t.Fatalf("pointCount %d, want 1", manifold.PointCount)
	}

	// The normal is the unit diagonal; the square root brings rounding.
	invSqrt2 := fixed.Q32MustParse("0.7071067811")
	if !nearVec(manifold.Normal, dbox2d.Vec2{X: invSqrt2, Y: invSqrt2}, sqrtTol()) {
		t.Fatalf("normal %v, want the unit diagonal", manifold.Normal)
	}

	// The separation is the corner distance sqrt(2)/128.
	wantSep := fixed.Q32MustParse("0.0110485434")
	if !near(manifold.Points[0].Separation, wantSep, sqrtTol()) {
		t.Fatalf("separation %v, want %v", manifold.Points[0].Separation, wantSep)
	}

	// The contact point is the exact midpoint of the two corners.
	if manifold.Points[0].Point != vecQ("1.00390625", "1.00390625") {
		t.Fatalf("point %v, want the corner midpoint", manifold.Points[0].Point)
	}
	if manifold.Points[0].Id != makeIdWant(2, 0) {
		t.Fatalf("id %d, want %d", manifold.Points[0].Id, makeIdWant(2, 0))
	}
}

// TestCollidePolygonsRejectsTheSpeculativeGap separates the boxes beyond the
// speculative distance.
func TestCollidePolygonsRejectsTheSpeculativeGap(t *testing.T) {
	boxA := dbox2d.MakeBox(fixed.Q32One(), fixed.Q32One())
	boxB := dbox2d.MakeBox(fixed.Q32One(), fixed.Q32One())
	xfA := dbox2d.TransformIdentity()
	xfB := dbox2d.Transform{P: vecQ("0", "2.125"), Q: dbox2d.RotIdentity()}

	manifold := dbox2d.CollidePolygons(&boxA, xfA, &boxB, xfB)
	if manifold.PointCount != 0 {
		t.Fatalf("separated boxes produced %d points", manifold.PointCount)
	}
}

// TestCollidePolygonAndCapsuleClipsTheEndCap stands a capsule on a box top
// face: the degenerate incident edge keeps both clip endpoints, one real and
// one speculative.
func TestCollidePolygonAndCapsuleClipsTheEndCap(t *testing.T) {
	boxA := dbox2d.MakeBox(fixed.Q32One(), fixed.Q32One())
	quarter := fixed.Q32MustParse("0.25")
	capsuleB := dbox2d.Capsule{Center1: vecQ("0", "-0.5"), Center2: vecQ("0", "0.5"), Radius: quarter}
	xfA := dbox2d.TransformIdentity()
	xfB := dbox2d.Transform{P: vecQ("0", "1.625"), Q: dbox2d.RotIdentity()}

	manifold := dbox2d.CollidePolygonAndCapsule(&boxA, xfA, &capsuleB, xfB)
	if manifold.PointCount != 2 {
		t.Fatalf("pointCount %d, want 2", manifold.PointCount)
	}
	if !manifold.Normal.X.Eq(fixed.Q32Zero()) || !manifold.Normal.Y.Eq(fixed.Q32One()) {
		t.Fatalf("normal %v, want (0, 1)", manifold.Normal)
	}

	p0 := manifold.Points[0]
	if p0.Point != vecQ("0", "1.4375") || !p0.Separation.Eq(fixed.Q32MustParse("0.875")) {
		t.Fatalf("point 0 %v separation %v, want (0, 1.4375) and 0.875", p0.Point, p0.Separation)
	}
	p1 := manifold.Points[1]
	if p1.Point != vecQ("0", "0.9375") || !p1.Separation.Eq(fixed.Q32MustParse("-0.125")) {
		t.Fatalf("point 1 %v separation %v, want (0, 0.9375) and -0.125", p1.Point, p1.Separation)
	}
	if p0.Id != makeIdWant(2, 1) || p1.Id != makeIdWant(3, 0) {
		t.Fatalf("ids %d, %d, want %d, %d", p0.Id, p1.Id, makeIdWant(2, 1), makeIdWant(3, 0))
	}
}

// TestCollideSegmentAndPolygonMatchesTheGround rests a box on a segment.
// The reference edge index wraps, so the second id uses vertex zero.
func TestCollideSegmentAndPolygonMatchesTheGround(t *testing.T) {
	segmentA := dbox2d.Segment{Point1: vec(-2, 0), Point2: vec(2, 0)}
	boxB := dbox2d.MakeBox(fixed.Q32One(), fixed.Q32One())
	xfA := dbox2d.TransformIdentity()
	xfB := dbox2d.Transform{P: vecQ("0", "0.75"), Q: dbox2d.RotIdentity()}

	manifold := dbox2d.CollideSegmentAndPolygon(&segmentA, xfA, &boxB, xfB)
	if manifold.PointCount != 2 {
		t.Fatalf("pointCount %d, want 2", manifold.PointCount)
	}
	if !manifold.Normal.X.Eq(fixed.Q32Zero()) || !manifold.Normal.Y.Eq(fixed.Q32One()) {
		t.Fatalf("normal %v, want (0, 1)", manifold.Normal)
	}

	sep := fixed.Q32MustParse("-0.25")
	p0 := manifold.Points[0]
	if p0.Point != vecQ("1", "-0.125") || !p0.Separation.Eq(sep) {
		t.Fatalf("point 0 %v separation %v, want (1, -0.125) and -0.25", p0.Point, p0.Separation)
	}
	p1 := manifold.Points[1]
	if p1.Point != vecQ("-1", "-0.125") || !p1.Separation.Eq(sep) {
		t.Fatalf("point 1 %v separation %v, want (-1, -0.125) and -0.25", p1.Point, p1.Separation)
	}
	if p0.Id != makeIdWant(1, 1) || p1.Id != makeIdWant(0, 0) {
		t.Fatalf("ids %d, %d, want %d, %d", p0.Id, p1.Id, makeIdWant(1, 1), makeIdWant(0, 0))
	}
}

// TestCollideSegmentAndPolygonRejectsADegenerateSegment checks the length
// assert of the capsule builder: a zero axis becomes a panic. See D-003 and
// D-012.
func TestCollideSegmentAndPolygonRejectsADegenerateSegment(t *testing.T) {
	defer func() {
		if recover() == nil {
			t.Fatal("a zero-length segment did not panic")
		}
	}()

	segmentA := dbox2d.Segment{Point1: vec(1, 1), Point2: vec(1, 1)}
	boxB := dbox2d.MakeBox(fixed.Q32One(), fixed.Q32One())
	xf := dbox2d.TransformIdentity()
	dbox2d.CollideSegmentAndPolygon(&segmentA, xf, &boxB, xf)
}

// TestCollideChainSegmentAndPolygonMatchesTheReference covers the face
// contact through the segment normal, the one-sided cull on the ghost
// side, the capsule wrapper and the vertex-vertex speculative point that
// the Gauss map admits at a convex corner.
func TestCollideChainSegmentAndPolygonMatchesTheReference(t *testing.T) {
	// The collidable side is to the right of p1->p2, which is below.
	segment := dbox2d.ChainSegment{
		Ghost1:  vec(-2, 0),
		Segment: dbox2d.Segment{Point1: vec(-1, 0), Point2: vec(1, 0)},
		Ghost2:  vec(2, 0),
	}
	xf := dbox2d.TransformIdentity()
	half := fixed.Q32Half()

	t.Run("face contact", func(t *testing.T) {
		var cache dbox2d.SimplexCache
		box := dbox2d.MakeBox(half, half)
		xfB := dbox2d.Transform{P: vecQ("0", "-0.4"), Q: dbox2d.RotIdentity()}
		manifold := dbox2d.CollideChainSegmentAndPolygon(&segment, xf, &box, xfB, &cache)
		if manifold.PointCount != 2 {
			t.Fatalf("pointCount %d, want 2", manifold.PointCount)
		}
		if manifold.Normal != vec(0, -1) {
			t.Fatalf("normal %v, want (0, -1)", manifold.Normal)
		}
		wantSep := fixed.Q32MustParse("-0.1")
		for i := range 2 {
			if !manifold.Points[i].Separation.Eq(wantSep) {
				t.Fatalf("point %d separation %v, want %v", i, manifold.Points[i].Separation, wantSep)
			}
		}
	})

	t.Run("ghost side", func(t *testing.T) {
		var cache dbox2d.SimplexCache
		box := dbox2d.MakeBox(half, half)
		xfB := dbox2d.Transform{P: vecQ("0", "0.4"), Q: dbox2d.RotIdentity()}
		manifold := dbox2d.CollideChainSegmentAndPolygon(&segment, xf, &box, xfB, &cache)
		if manifold.PointCount != 0 {
			t.Fatalf("ghost side produced %d points", manifold.PointCount)
		}
	})

	t.Run("capsule", func(t *testing.T) {
		var cache dbox2d.SimplexCache
		capsule := dbox2d.Capsule{Center1: vecQ("-0.5", "0"), Center2: vecQ("0.5", "0"), Radius: fixed.Q32MustParse("0.25")}
		xfB := dbox2d.Transform{P: vecQ("0", "-0.2"), Q: dbox2d.RotIdentity()}
		manifold := dbox2d.CollideChainSegmentAndCapsule(&segment, xf, &capsule, xfB, &cache)
		if manifold.PointCount != 2 {
			t.Fatalf("pointCount %d, want 2", manifold.PointCount)
		}
		wantSep := fixed.Q32MustParse("-0.05")
		if !near(manifold.Points[0].Separation, wantSep, sqrtTol()) {
			t.Fatalf("separation %v, want %v", manifold.Points[0].Separation, wantSep)
		}
	})

	t.Run("convex corner admits a vertex", func(t *testing.T) {
		// The next edge turns up, so p2 is a convex corner. A rounded
		// diamond hangs below and to the right of p2 with its top vertex
		// just outside the radius: the normal lies inside the Gauss map
		// and one speculative point comes back.
		bent := segment
		bent.Ghost2 = vec(2, 1)
		var cache dbox2d.SimplexCache
		diamond := dbox2d.MakeOffsetBox(half, half, vec(0, 0), dbox2d.MakeRot(fixed.Q32FromRatio(1, 8)))
		diamond.Radius = fixed.Q32MustParse("0.3")
		xfB := dbox2d.Transform{P: vecQ("1.118", "-0.994"), Q: dbox2d.RotIdentity()}
		manifold := dbox2d.CollideChainSegmentAndPolygon(&bent, xf, &diamond, xfB, &cache)
		if manifold.PointCount != 1 {
			t.Fatalf("pointCount %d, want 1", manifold.PointCount)
		}
		sep := manifold.Points[0].Separation
		if !fixed.Q32Zero().Less(sep) || !sep.Less(dbox2d.SpeculativeDistance()) {
			t.Fatalf("separation %v, want a speculative gap", sep)
		}
		if !fixed.Q32Zero().Less(manifold.Normal.X) || !manifold.Normal.Y.Less(fixed.Q32Zero()) {
			t.Fatalf("normal %v, want down and to the right", manifold.Normal)
		}
	})
}
