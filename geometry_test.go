package b2_test

import (
	"testing"

	"github.com/dhannyell/dbox2d"
)

// ray returns a ray cast input with a maximum fraction of one.
func ray(origin, translation b2.Vec2) b2.RayCastInput {
	return b2.RayCastInput{
		Origin:      origin,
		Translation: translation,
		MaxFraction: b2.QOne(),
	}
}

// TestMakeBoxMassMatchesTheReference pins the mass integral on the shape
// whose exact answer is known: a unit square of unit density has a mass of
// one and a rotational inertia of one twelfth per unit of squared side.
func TestMakeBoxMassMatchesTheReference(t *testing.T) {
	box := b2.MakeBox(b2.QHalf(), b2.QHalf())
	data := b2.ComputePolygonMass(&box, b2.QOne())

	limit := tol(1, 10000)
	if !near(data.Mass, b2.QOne(), limit) {
		t.Errorf("mass = %v, want 1", data.Mass)
	}
	if !near(data.Center.X, b2.QZero(), limit) || !near(data.Center.Y, b2.QZero(), limit) {
		t.Errorf("center = %v, want the origin", data.Center)
	}
	if want := b2.QFromRatio(1, 6); !near(data.RotationalInertia, want, limit) {
		t.Errorf("rotational inertia = %v, want %v", data.RotationalInertia, want)
	}
}

// TestComputeCircleMassMatchesTheReference pins the circle integral to the
// upstream constant. A unit circle has a mass of pi and half that as inertia.
func TestComputeCircleMassMatchesTheReference(t *testing.T) {
	circle := b2.Circle{Radius: b2.QOne()}
	data := b2.ComputeCircleMass(&circle, b2.QOne())

	wantMass := b2.QMustParse("3.14159265359")
	wantInertia := b2.QMustParse("1.570796326795")
	if !data.Mass.Eq(wantMass) {
		t.Errorf("mass = %v, want %v", data.Mass, wantMass)
	}
	if !data.RotationalInertia.Eq(wantInertia) {
		t.Errorf("rotational inertia = %v, want %v", data.RotationalInertia, wantInertia)
	}
}

// TestZeroLengthCapsuleMassEqualsACircle checks the limit case of the
// capsule integral. The rectangle vanishes and the two half circles rejoin.
func TestZeroLengthCapsuleMassEqualsACircle(t *testing.T) {
	center := pt("2", "-1")
	capsule := b2.Capsule{Center1: center, Center2: center, Radius: b2.QHalf()}
	circle := b2.Circle{Center: center, Radius: b2.QHalf()}

	got := b2.ComputeCapsuleMass(&capsule, b2.QFromInt(3))
	want := b2.ComputeCircleMass(&circle, b2.QFromInt(3))

	limit := tol(1, 10000)
	if !near(got.Mass, want.Mass, limit) {
		t.Errorf("mass = %v, want %v", got.Mass, want.Mass)
	}
	if !near(got.Center.X, want.Center.X, limit) || !near(got.Center.Y, want.Center.Y, limit) {
		t.Errorf("center = %v, want %v", got.Center, want.Center)
	}
	if !near(got.RotationalInertia, want.RotationalInertia, limit) {
		t.Errorf("rotational inertia = %v, want %v", got.RotationalInertia, want.RotationalInertia)
	}
}

// TestPolygonCentroidOfATriangle pins the area divisions. The centroid of a
// triangle is exactly the mean of its vertices.
func TestPolygonCentroidOfATriangle(t *testing.T) {
	hull := b2.ComputeHull([]b2.Vec2{pt("0", "0"), pt("3", "0"), pt("0", "3")})
	polygon := b2.MakePolygon(&hull, b2.QZero())

	if polygon.Centroid != pt("1", "1") {
		t.Errorf("centroid = %v, want (1, 1)", polygon.Centroid)
	}
}

// TestTriangleMassMatchesTheReference pins the polygon mass divisions. A
// right triangle with legs of length three has an exact integral in Q32.32.
func TestTriangleMassMatchesTheReference(t *testing.T) {
	hull := b2.ComputeHull([]b2.Vec2{pt("0", "0"), pt("3", "0"), pt("0", "3")})
	triangle := b2.MakePolygon(&hull, b2.QZero())
	data := b2.ComputePolygonMass(&triangle, b2.QOne())

	if want := b2.QMustParse("4.5"); !data.Mass.Eq(want) {
		t.Errorf("mass = %v, want %v", data.Mass, want)
	}
	if data.Center != pt("1", "1") {
		t.Errorf("center = %v, want (1, 1)", data.Center)
	}
	if want := b2.QMustParse("13.5"); !data.RotationalInertia.Eq(want) {
		t.Errorf("rotational inertia = %v, want %v", data.RotationalInertia, want)
	}
}

// TestComputePolygonAABBContainsTheVertices guards the box that the
// broadphase stores. A rotated box still needs every vertex inside.
func TestComputePolygonAABBContainsTheVertices(t *testing.T) {
	box := b2.MakeBox(b2.QOne(), b2.QHalf())
	xf := b2.Transform{
		P: pt("2", "3"),
		Q: b2.MakeRot(b2.QMustParse("0.125")),
	}

	aabb := b2.ComputePolygonAABB(&box, xf)

	for i := range box.Count {
		v := b2.TransformPoint(xf, box.Vertices[i])
		if v.X.Less(aabb.LowerBound.X) || aabb.UpperBound.X.Less(v.X) ||
			v.Y.Less(aabb.LowerBound.Y) || aabb.UpperBound.Y.Less(v.Y) {
			t.Errorf("vertex %d at %v falls outside %v", i, v, aabb)
		}
	}
}

// TestTransformedBoxesMatchAQuarterTurn checks both ways to place a box
// against coordinates calculated independently from their shared helpers.
func TestTransformedBoxesMatchAQuarterTurn(t *testing.T) {
	center := pt("2", "-3")
	rotation := b2.Rot{Sin: b2.QOne(), Cos: b2.QZero()}

	box := b2.MakeBox(b2.QOne(), b2.QHalf())
	cases := []struct {
		name string
		got  b2.Polygon
	}{
		{"TransformPolygon", b2.TransformPolygon(b2.Transform{P: center, Q: rotation}, &box)},
		{"MakeOffsetBox", b2.MakeOffsetBox(b2.QOne(), b2.QHalf(), center, rotation)},
	}
	wantVertices := []b2.Vec2{pt("2.5", "-4"), pt("2.5", "-2"), pt("1.5", "-2"), pt("1.5", "-4")}
	wantNormals := []b2.Vec2{pt("1", "0"), pt("0", "1"), pt("-1", "0"), pt("0", "-1")}

	for _, c := range cases {
		if c.got.Count != 4 || c.got.Centroid != center || !c.got.Radius.Eq(b2.QZero()) {
			t.Errorf("%s metadata = %+v, want count 4, center %v and radius 0", c.name, c.got, center)
			continue
		}
		for i := range c.got.Count {
			if c.got.Vertices[i] != wantVertices[i] {
				t.Errorf("%s vertex %d = %v, want %v", c.name, i, c.got.Vertices[i], wantVertices[i])
			}
			if c.got.Normals[i] != wantNormals[i] {
				t.Errorf("%s normal %d = %v, want %v", c.name, i, c.got.Normals[i], wantNormals[i])
			}
		}
	}
}

// TestPointTestsIncludeTheBoundary pins the closed comparison. A point on
// the surface counts as inside, for the circle and for the capsule.
func TestPointTestsIncludeTheBoundary(t *testing.T) {
	circle := b2.Circle{Radius: b2.QFromInt(2)}
	if !b2.PointInCircle(pt("2", "0"), &circle) {
		t.Errorf("a point on the circle surface is outside")
	}
	if b2.PointInCircle(pt("2.001", "0"), &circle) {
		t.Errorf("a point beyond the circle surface is inside")
	}

	capsule := b2.Capsule{Center1: pt("-1", "0"), Center2: pt("1", "0"), Radius: b2.QHalf()}
	if !b2.PointInCapsule(pt("0", "0.5"), &capsule) {
		t.Errorf("a point on the capsule side is outside")
	}
	if !b2.PointInCapsule(pt("1.5", "0"), &capsule) {
		t.Errorf("a point on the capsule cap is outside")
	}
	if b2.PointInCapsule(pt("1.501", "0"), &capsule) {
		t.Errorf("a point beyond the capsule cap is inside")
	}
}

// TestRayCastCircleHitsTheNearSurface pins the fraction, the point and the
// normal against values computed by hand.
func TestRayCastCircleHitsTheNearSurface(t *testing.T) {
	circle := b2.Circle{Radius: b2.QOne()}
	input := ray(pt("-3", "0"), pt("6", "0"))

	output := b2.RayCastCircle(&input, &circle)

	if !output.Hit {
		t.Fatalf("the ray misses the circle")
	}
	limit := tol(1, 1000)
	if want := b2.QFromRatio(1, 3); !near(output.Fraction, want, limit) {
		t.Errorf("fraction = %v, want %v", output.Fraction, want)
	}
	if !near(output.Point.X, b2.QFromInt(-1), limit) || !near(output.Point.Y, b2.QZero(), limit) {
		t.Errorf("point = %v, want (-1, 0)", output.Point)
	}
	if !near(output.Normal.X, b2.QFromInt(-1), limit) || !near(output.Normal.Y, b2.QZero(), limit) {
		t.Errorf("normal = %v, want (-1, 0)", output.Normal)
	}

	// A ray that passes beside the circle reports no hit.
	miss := ray(pt("-3", "2"), pt("6", "0"))
	if b2.RayCastCircle(&miss, &circle).Hit {
		t.Errorf("a ray that passes beside the circle reports a hit")
	}
}

// TestRayCastSegmentSkipsTheLeftSide pins the one-sided rule, which the
// chain shapes depend on.
func TestRayCastSegmentSkipsTheLeftSide(t *testing.T) {
	segment := b2.Segment{Point1: pt("-1", "0"), Point2: pt("1", "0")}

	fromAbove := ray(pt("0", "1"), pt("0", "-2"))
	if b2.RayCastSegment(&fromAbove, &segment, true).Hit {
		t.Errorf("the one-sided segment accepts a ray from the left side")
	}
	if !b2.RayCastSegment(&fromAbove, &segment, false).Hit {
		t.Errorf("the two-sided segment rejects a ray from the left side")
	}

	fromBelow := ray(pt("0", "-1"), pt("0", "2"))
	output := b2.RayCastSegment(&fromBelow, &segment, true)
	if !output.Hit {
		t.Fatalf("the one-sided segment rejects a ray from the right side")
	}
	limit := tol(1, 1000)
	if !near(output.Fraction, b2.QHalf(), limit) {
		t.Errorf("fraction = %v, want 0.5", output.Fraction)
	}
	if !near(output.Normal.Y, b2.QFromInt(-1), limit) {
		t.Errorf("normal = %v, want (0, -1)", output.Normal)
	}
}

// TestRayCastPolygonHitsABoxFace pins the half-space clip. The reference
// avoids a division in the comparison, so the port must keep that form.
func TestRayCastPolygonHitsABoxFace(t *testing.T) {
	box := b2.MakeBox(b2.QOne(), b2.QOne())
	input := ray(pt("-3", "0"), pt("6", "0"))

	output := b2.RayCastPolygon(&input, &box)

	if !output.Hit {
		t.Fatalf("the ray misses the box")
	}
	limit := tol(1, 1000)
	if want := b2.QFromRatio(1, 3); !near(output.Fraction, want, limit) {
		t.Errorf("fraction = %v, want %v", output.Fraction, want)
	}
	if !near(output.Point.X, b2.QFromInt(-1), limit) || !near(output.Point.Y, b2.QZero(), limit) {
		t.Errorf("point = %v, want (-1, 0)", output.Point)
	}
	if !near(output.Normal.X, b2.QFromInt(-1), limit) || !near(output.Normal.Y, b2.QZero(), limit) {
		t.Errorf("normal = %v, want (-1, 0)", output.Normal)
	}

	// A ray that starts inside reports the origin and a zero fraction.
	inside := ray(pt("0", "0"), pt("6", "0"))
	if output := b2.RayCastPolygon(&inside, &box); !output.Hit || !output.Fraction.Eq(b2.QZero()) {
		t.Errorf("initial overlap = %+v, want a hit with a zero fraction", output)
	}
}

// TestIsValidRayBoundsTheFraction guards the input check that the circle,
// capsule and polygon casts run first. The segment cast skips it, as the
// reference does.
func TestIsValidRayBoundsTheFraction(t *testing.T) {
	good := ray(pt("0", "0"), pt("1", "0"))
	if !b2.IsValidRay(&good) {
		t.Errorf("IsValidRay rejects a usable ray")
	}

	negative := good
	negative.MaxFraction = b2.QOne().Neg()
	if b2.IsValidRay(&negative) {
		t.Errorf("IsValidRay accepts a negative fraction")
	}
}

// TestRayCastCapsuleDegenerateCases pins both exact guards: a capsule of zero
// length is a circle, and a parallel ray outside the surface misses. The
// upstream uses epsilon guards for both cases.
func TestRayCastCapsuleDegenerateCases(t *testing.T) {
	radius := b2.QMustParse("0.5")
	point := b2.Capsule{Center1: pt("0", "0"), Center2: pt("0", "0"), Radius: radius}
	input := ray(pt("-2", "0"), pt("4", "0"))

	output := b2.RayCastCapsule(&input, &point)

	// The delegation target gives the oracle, and the literal values pin it.
	circle := b2.Circle{Center: pt("0", "0"), Radius: radius}
	viaCircle := b2.RayCastCircle(&input, &circle)
	if output != viaCircle {
		t.Errorf("capsule cast = %+v, circle cast = %+v", output, viaCircle)
	}
	if !output.Hit {
		t.Fatalf("the ray misses the zero-length capsule")
	}
	if want := b2.QMustParse("0.375"); !output.Fraction.Eq(want) {
		t.Errorf("fraction = %v, want %v", output.Fraction, want)
	}
	if want := pt("-0.5", "0"); output.Point != want {
		t.Errorf("point = %v, want %v", output.Point, want)
	}
	if want := pt("-1", "0"); output.Normal != want {
		t.Errorf("normal = %v, want %v", output.Normal, want)
	}

	// A ray parallel to the axis and outside the surface misses: the
	// determinant is exactly zero.
	capsule := b2.Capsule{Center1: pt("-1", "0"), Center2: pt("1", "0"), Radius: b2.QMustParse("0.25")}
	beside := ray(pt("-2", "1"), pt("4", "0"))
	if b2.RayCastCapsule(&beside, &capsule).Hit {
		t.Errorf("a parallel ray outside the capsule reports a hit")
	}
}

// TestPolygonConstructorsRejectInvalidHull checks the common validation
// boundary before either constructor computes normals or a centroid.
func TestPolygonConstructorsRejectInvalidHull(t *testing.T) {
	hull := b2.Hull{Count: 3}
	hull.Points[0] = pt("0", "0")
	hull.Points[1] = pt("1", "0")
	hull.Points[2] = pt("1", "0")

	cases := []struct {
		name  string
		build func()
	}{
		{"MakePolygon", func() { b2.MakePolygon(&hull, b2.QZero()) }},
		{"MakeOffsetRoundedPolygon", func() {
			b2.MakeOffsetRoundedPolygon(&hull, pt("0", "0"), b2.RotIdentity(), b2.QHalf())
		}},
	}

	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			defer func() {
				if recover() == nil {
					t.Errorf("%s accepts a hull with a zero-length edge", c.name)
				}
			}()
			c.build()
		})
	}
}

// TestComputePolygonMassRejectsZeroArea checks the exact area guard on a
// hand-written polygon, which does not pass through a hull constructor.
func TestComputePolygonMassRejectsZeroArea(t *testing.T) {
	polygon := b2.Polygon{Count: 3}
	polygon.Vertices[0] = pt("0", "0")
	polygon.Vertices[1] = pt("1", "0")
	polygon.Vertices[2] = pt("2", "0")

	defer func() {
		if recover() == nil {
			t.Errorf("ComputePolygonMass accepts a polygon with zero area")
		}
	}()
	b2.ComputePolygonMass(&polygon, b2.QOne())
}

// TestPolygonConstructorsMatchTheReference pins the literal layout of each
// constructor on dyadic inputs. Vertices, normals and radius are exact
// copies, so they compare exactly; the centroid integral may floor.
func TestPolygonConstructorsMatchTheReference(t *testing.T) {
	unitSquare := b2.ComputeHull([]b2.Vec2{
		pt("-1", "-1"), pt("1", "-1"), pt("1", "1"), pt("-1", "1"),
	})
	quarter := b2.QMustParse("0.25")

	boxNormals := []b2.Vec2{pt("0", "-1"), pt("1", "0"), pt("0", "1"), pt("-1", "0")}

	cases := []struct {
		name     string
		got      b2.Polygon
		vertices []b2.Vec2
		normals  []b2.Vec2
		centroid b2.Vec2
		radius   b2.Q
	}{
		{
			"MakeSquare",
			b2.MakeSquare(b2.QHalf()),
			[]b2.Vec2{pt("-0.5", "-0.5"), pt("0.5", "-0.5"), pt("0.5", "0.5"), pt("-0.5", "0.5")},
			boxNormals, pt("0", "0"), b2.QZero(),
		},
		{
			"MakeRoundedBox",
			b2.MakeRoundedBox(b2.QOne(), b2.QFromInt(2), quarter),
			[]b2.Vec2{pt("-1", "-2"), pt("1", "-2"), pt("1", "2"), pt("-1", "2")},
			boxNormals, pt("0", "0"), quarter,
		},
		{
			"MakeOffsetPolygon",
			b2.MakeOffsetPolygon(&unitSquare, pt("2", "3"), b2.RotIdentity()),
			[]b2.Vec2{pt("1", "2"), pt("3", "2"), pt("3", "4"), pt("1", "4")},
			boxNormals, pt("2", "3"), b2.QZero(),
		},
		{
			"MakeOffsetRoundedPolygon",
			b2.MakeOffsetRoundedPolygon(&unitSquare, pt("2", "3"), b2.RotIdentity(), quarter),
			[]b2.Vec2{pt("1", "2"), pt("3", "2"), pt("3", "4"), pt("1", "4")},
			boxNormals, pt("2", "3"), quarter,
		},
	}

	limit := tol(1, 10000)
	for _, c := range cases {
		if c.got.Count != len(c.vertices) {
			t.Errorf("%s: count = %d, want %d", c.name, c.got.Count, len(c.vertices))
			continue
		}
		for i := range c.vertices {
			if c.got.Vertices[i] != c.vertices[i] {
				t.Errorf("%s: vertex %d = %v, want %v", c.name, i, c.got.Vertices[i], c.vertices[i])
			}
			if c.got.Normals[i] != c.normals[i] {
				t.Errorf("%s: normal %d = %v, want %v", c.name, i, c.got.Normals[i], c.normals[i])
			}
		}
		if !c.got.Radius.Eq(c.radius) {
			t.Errorf("%s: radius = %v, want %v", c.name, c.got.Radius, c.radius)
		}
		if !near(c.got.Centroid.X, c.centroid.X, limit) || !near(c.got.Centroid.Y, c.centroid.Y, limit) {
			t.Errorf("%s: centroid = %v, want %v", c.name, c.got.Centroid, c.centroid)
		}
	}
}

// TestShapeAABBsMatchTheReference pins the box of each shape under one
// translation. The inputs are dyadic, so every bound compares exactly.
func TestShapeAABBsMatchTheReference(t *testing.T) {
	xf := b2.Transform{P: pt("1", "2"), Q: b2.RotIdentity()}
	quarter := b2.QMustParse("0.25")

	circle := b2.Circle{Center: pt("0.5", "0"), Radius: quarter}
	capsule := b2.Capsule{Center1: pt("-0.5", "0"), Center2: pt("0.5", "0"), Radius: quarter}
	segment := b2.Segment{Point1: pt("0", "-1"), Point2: pt("2", "1")}

	cases := []struct {
		name         string
		got          b2.AABB
		lower, upper b2.Vec2
	}{
		{"circle", b2.ComputeCircleAABB(&circle, xf), pt("1.25", "1.75"), pt("1.75", "2.25")},
		{"capsule", b2.ComputeCapsuleAABB(&capsule, xf), pt("0.25", "1.75"), pt("1.75", "2.25")},
		{"segment", b2.ComputeSegmentAABB(&segment, xf), pt("1", "1"), pt("3", "3")},
	}

	for _, c := range cases {
		if c.got.LowerBound != c.lower || c.got.UpperBound != c.upper {
			t.Errorf("%s: aabb = %+v, want [%v, %v]", c.name, c.got, c.lower, c.upper)
		}
	}
}

// TestShapeCastsReachTheNearSurface sweeps a point proxy from the left
// against each shape and expects the fraction within a slop of the exact
// value: the advancement stops a slop short of a sharp surface and a slop
// inside a rounded one. The rounded polygon ray cast goes through the
// same sweep.
func TestShapeCastsReachTheNearSurface(t *testing.T) {
	one := b2.QOne()
	point := b2.MakeProxy([]b2.Vec2{pt("-3", "0")}, b2.QZero())
	input := b2.ShapeCastInput{Proxy: point, Translation: pt("10", "0"), MaxFraction: one}

	circle := b2.Circle{Radius: one}
	capsule := b2.Capsule{Center1: pt("-1", "-1"), Center2: pt("-1", "1"), Radius: b2.QHalf()}
	segment := b2.Segment{Point1: pt("-1", "1"), Point2: pt("-1", "-1")}
	box := b2.MakeBox(one, one)
	rounded := b2.MakeBox(b2.QHalf(), b2.QHalf())
	rounded.Radius = b2.QHalf()

	cases := []struct {
		name  string
		got   b2.CastOutput
		exact string
	}{
		{"circle", b2.ShapeCastCircle(&input, &circle), "0.2"},
		{"capsule", b2.ShapeCastCapsule(&input, &capsule), "0.15"},
		{"segment", b2.ShapeCastSegment(&input, &segment), "0.2"},
		{"polygon", b2.ShapeCastPolygon(&input, &box), "0.2"},
		{"rounded polygon ray", b2.RayCastPolygon(&b2.RayCastInput{Origin: pt("-3", "0"), Translation: pt("10", "0"), MaxFraction: one}, &rounded), "0.2"},
	}

	for _, c := range cases {
		exact := b2.QMustParse(c.exact)
		if !c.got.Hit {
			t.Fatalf("%s: the sweep missed", c.name)
		}
		if !near(c.got.Fraction, exact, b2.QMustParse("0.001")) {
			t.Fatalf("%s: fraction %v, want near %s", c.name, c.got.Fraction, c.exact)
		}
		if !nearVec(c.got.Normal, pt("-1", "0"), sqrtTol()) {
			t.Fatalf("%s: normal %v, want (-1, 0)", c.name, c.got.Normal)
		}
	}
}
