package dbox2d_test

import (
	"testing"

	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/fixed"
)

// ray returns a ray cast input with a maximum fraction of one.
func ray(origin, translation dbox2d.Vec2) dbox2d.RayCastInput {
	return dbox2d.RayCastInput{
		Origin:      origin,
		Translation: translation,
		MaxFraction: fixed.One(),
	}
}

// TestMakeBoxMassMatchesTheReference pins the mass integral on the shape
// whose exact answer is known: a unit square of unit density has a mass of
// one and a rotational inertia of one twelfth per unit of squared side.
func TestMakeBoxMassMatchesTheReference(t *testing.T) {
	box := dbox2d.MakeBox(fixed.Half(), fixed.Half())
	data := dbox2d.ComputePolygonMass(&box, fixed.One())

	limit := tol(1, 10000)
	if !near(data.Mass, fixed.One(), limit) {
		t.Errorf("mass = %v, want 1", data.Mass)
	}
	if !near(data.Center.X, fixed.Zero(), limit) || !near(data.Center.Y, fixed.Zero(), limit) {
		t.Errorf("center = %v, want the origin", data.Center)
	}
	if want := fixed.FromRatio(1, 6); !near(data.RotationalInertia, want, limit) {
		t.Errorf("rotational inertia = %v, want %v", data.RotationalInertia, want)
	}
}

// TestComputeCircleMassMatchesTheReference pins the circle integral to the
// upstream constant. A unit circle has a mass of pi and half that as inertia.
func TestComputeCircleMassMatchesTheReference(t *testing.T) {
	circle := dbox2d.Circle{Radius: fixed.One()}
	data := dbox2d.ComputeCircleMass(&circle, fixed.One())

	wantMass := fixed.MustParse("3.14159265359")
	wantInertia := fixed.MustParse("1.570796326795")
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
	capsule := dbox2d.Capsule{Center1: center, Center2: center, Radius: fixed.Half()}
	circle := dbox2d.Circle{Center: center, Radius: fixed.Half()}

	got := dbox2d.ComputeCapsuleMass(&capsule, fixed.FromInt(3))
	want := dbox2d.ComputeCircleMass(&circle, fixed.FromInt(3))

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
	hull := dbox2d.ComputeHull([]dbox2d.Vec2{pt("0", "0"), pt("3", "0"), pt("0", "3")})
	polygon := dbox2d.MakePolygon(&hull, fixed.Zero())

	if polygon.Centroid != pt("1", "1") {
		t.Errorf("centroid = %v, want (1, 1)", polygon.Centroid)
	}
}

// TestTriangleMassMatchesTheReference pins the polygon mass divisions. A
// right triangle with legs of length three has an exact integral in Q32.32.
func TestTriangleMassMatchesTheReference(t *testing.T) {
	hull := dbox2d.ComputeHull([]dbox2d.Vec2{pt("0", "0"), pt("3", "0"), pt("0", "3")})
	triangle := dbox2d.MakePolygon(&hull, fixed.Zero())
	data := dbox2d.ComputePolygonMass(&triangle, fixed.One())

	if want := fixed.MustParse("4.5"); !data.Mass.Eq(want) {
		t.Errorf("mass = %v, want %v", data.Mass, want)
	}
	if data.Center != pt("1", "1") {
		t.Errorf("center = %v, want (1, 1)", data.Center)
	}
	if want := fixed.MustParse("13.5"); !data.RotationalInertia.Eq(want) {
		t.Errorf("rotational inertia = %v, want %v", data.RotationalInertia, want)
	}
}

// TestComputePolygonAABBContainsTheVertices guards the box that the
// broadphase stores. A rotated box still needs every vertex inside.
func TestComputePolygonAABBContainsTheVertices(t *testing.T) {
	box := dbox2d.MakeBox(fixed.One(), fixed.Half())
	xf := dbox2d.Transform{
		P: pt("2", "3"),
		Q: dbox2d.MakeRot(fixed.MustParse("0.125")),
	}

	aabb := dbox2d.ComputePolygonAABB(&box, xf)

	for i := range box.Count {
		v := dbox2d.TransformPoint(xf, box.Vertices[i])
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
	rotation := dbox2d.Rot{Sin: fixed.One(), Cos: fixed.Zero()}

	box := dbox2d.MakeBox(fixed.One(), fixed.Half())
	cases := []struct {
		name string
		got  dbox2d.Polygon
	}{
		{"TransformPolygon", dbox2d.TransformPolygon(dbox2d.Transform{P: center, Q: rotation}, &box)},
		{"MakeOffsetBox", dbox2d.MakeOffsetBox(fixed.One(), fixed.Half(), center, rotation)},
	}
	wantVertices := []dbox2d.Vec2{pt("2.5", "-4"), pt("2.5", "-2"), pt("1.5", "-2"), pt("1.5", "-4")}
	wantNormals := []dbox2d.Vec2{pt("1", "0"), pt("0", "1"), pt("-1", "0"), pt("0", "-1")}

	for _, c := range cases {
		if c.got.Count != 4 || c.got.Centroid != center || !c.got.Radius.Eq(fixed.Zero()) {
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
	circle := dbox2d.Circle{Radius: fixed.FromInt(2)}
	if !dbox2d.PointInCircle(pt("2", "0"), &circle) {
		t.Errorf("a point on the circle surface is outside")
	}
	if dbox2d.PointInCircle(pt("2.001", "0"), &circle) {
		t.Errorf("a point beyond the circle surface is inside")
	}

	capsule := dbox2d.Capsule{Center1: pt("-1", "0"), Center2: pt("1", "0"), Radius: fixed.Half()}
	if !dbox2d.PointInCapsule(pt("0", "0.5"), &capsule) {
		t.Errorf("a point on the capsule side is outside")
	}
	if !dbox2d.PointInCapsule(pt("1.5", "0"), &capsule) {
		t.Errorf("a point on the capsule cap is outside")
	}
	if dbox2d.PointInCapsule(pt("1.501", "0"), &capsule) {
		t.Errorf("a point beyond the capsule cap is inside")
	}
}

// TestRayCastCircleHitsTheNearSurface pins the fraction, the point and the
// normal against values computed by hand.
func TestRayCastCircleHitsTheNearSurface(t *testing.T) {
	circle := dbox2d.Circle{Radius: fixed.One()}
	input := ray(pt("-3", "0"), pt("6", "0"))

	output := dbox2d.RayCastCircle(&input, &circle)

	if !output.Hit {
		t.Fatalf("the ray misses the circle")
	}
	limit := tol(1, 1000)
	if want := fixed.FromRatio(1, 3); !near(output.Fraction, want, limit) {
		t.Errorf("fraction = %v, want %v", output.Fraction, want)
	}
	if !near(output.Point.X, fixed.FromInt(-1), limit) || !near(output.Point.Y, fixed.Zero(), limit) {
		t.Errorf("point = %v, want (-1, 0)", output.Point)
	}
	if !near(output.Normal.X, fixed.FromInt(-1), limit) || !near(output.Normal.Y, fixed.Zero(), limit) {
		t.Errorf("normal = %v, want (-1, 0)", output.Normal)
	}

	// A ray that passes beside the circle reports no hit.
	miss := ray(pt("-3", "2"), pt("6", "0"))
	if dbox2d.RayCastCircle(&miss, &circle).Hit {
		t.Errorf("a ray that passes beside the circle reports a hit")
	}
}

// TestRayCastSegmentSkipsTheLeftSide pins the one-sided rule, which the
// chain shapes depend on.
func TestRayCastSegmentSkipsTheLeftSide(t *testing.T) {
	segment := dbox2d.Segment{Point1: pt("-1", "0"), Point2: pt("1", "0")}

	fromAbove := ray(pt("0", "1"), pt("0", "-2"))
	if dbox2d.RayCastSegment(&fromAbove, &segment, true).Hit {
		t.Errorf("the one-sided segment accepts a ray from the left side")
	}
	if !dbox2d.RayCastSegment(&fromAbove, &segment, false).Hit {
		t.Errorf("the two-sided segment rejects a ray from the left side")
	}

	fromBelow := ray(pt("0", "-1"), pt("0", "2"))
	output := dbox2d.RayCastSegment(&fromBelow, &segment, true)
	if !output.Hit {
		t.Fatalf("the one-sided segment rejects a ray from the right side")
	}
	limit := tol(1, 1000)
	if !near(output.Fraction, fixed.Half(), limit) {
		t.Errorf("fraction = %v, want 0.5", output.Fraction)
	}
	if !near(output.Normal.Y, fixed.FromInt(-1), limit) {
		t.Errorf("normal = %v, want (0, -1)", output.Normal)
	}
}

// TestRayCastPolygonHitsABoxFace pins the half-space clip. The reference
// avoids a division in the comparison, so the port must keep that form.
func TestRayCastPolygonHitsABoxFace(t *testing.T) {
	box := dbox2d.MakeBox(fixed.One(), fixed.One())
	input := ray(pt("-3", "0"), pt("6", "0"))

	output := dbox2d.RayCastPolygon(&input, &box)

	if !output.Hit {
		t.Fatalf("the ray misses the box")
	}
	limit := tol(1, 1000)
	if want := fixed.FromRatio(1, 3); !near(output.Fraction, want, limit) {
		t.Errorf("fraction = %v, want %v", output.Fraction, want)
	}
	if !near(output.Point.X, fixed.FromInt(-1), limit) || !near(output.Point.Y, fixed.Zero(), limit) {
		t.Errorf("point = %v, want (-1, 0)", output.Point)
	}
	if !near(output.Normal.X, fixed.FromInt(-1), limit) || !near(output.Normal.Y, fixed.Zero(), limit) {
		t.Errorf("normal = %v, want (-1, 0)", output.Normal)
	}

	// A ray that starts inside reports the origin and a zero fraction.
	inside := ray(pt("0", "0"), pt("6", "0"))
	if output := dbox2d.RayCastPolygon(&inside, &box); !output.Hit || !output.Fraction.Eq(fixed.Zero()) {
		t.Errorf("initial overlap = %+v, want a hit with a zero fraction", output)
	}
}

// TestRayCastCapsuleHitsTheSide pins the Cramer solve with an oblique ray.
// Its determinant is not reciprocal-exact in Q32.32.
func TestRayCastCapsuleHitsTheSide(t *testing.T) {
	capsule := dbox2d.Capsule{Center1: pt("-1", "0"), Center2: pt("1", "0"), Radius: fixed.Half()}
	translation := pt("-9.5", "-9.5")
	wantFraction := fixed.FromRatio(3, 5)
	target := pt("0", "0.5")
	origin := target.Sub(translation.Mul(wantFraction))
	input := ray(origin, translation)

	output := dbox2d.RayCastCapsule(&input, &capsule)

	if !output.Hit {
		t.Fatalf("the ray misses the capsule")
	}
	if !output.Fraction.Eq(wantFraction) {
		t.Errorf("fraction = %v, want %v", output.Fraction, wantFraction)
	}
	// The reference order of operations floors the x coordinate two raw
	// units below zero.
	wantPoint := dbox2d.Vec2{X: fixed.FromRaw(-2), Y: fixed.Half()}
	if output.Point != wantPoint {
		t.Errorf("point = %v, want %v", output.Point, wantPoint)
	}
	if output.Normal != pt("0", "1") {
		t.Errorf("normal = %v, want (0, 1)", output.Normal)
	}
}

// TestIsValidRayBoundsTheFraction guards the input check that the circle,
// capsule and polygon casts run first. The segment cast skips it, as the
// reference does.
func TestIsValidRayBoundsTheFraction(t *testing.T) {
	good := ray(pt("0", "0"), pt("1", "0"))
	if !dbox2d.IsValidRay(&good) {
		t.Errorf("IsValidRay rejects a usable ray")
	}

	negative := good
	negative.MaxFraction = fixed.One().Neg()
	if dbox2d.IsValidRay(&negative) {
		t.Errorf("IsValidRay accepts a negative fraction")
	}

	saturated := good
	saturated.Origin = dbox2d.Vec2{X: fixed.MaxValue(), Y: fixed.Zero()}
	if dbox2d.IsValidRay(&saturated) {
		t.Errorf("IsValidRay accepts a saturated origin")
	}
}

// TestRayCastCapsuleDegenerateCases pins both exact guards: a capsule of zero
// length is a circle, and a parallel ray outside the surface misses. The
// upstream uses epsilon guards for both cases.
func TestRayCastCapsuleDegenerateCases(t *testing.T) {
	radius := fixed.MustParse("0.5")
	point := dbox2d.Capsule{Center1: pt("0", "0"), Center2: pt("0", "0"), Radius: radius}
	input := ray(pt("-2", "0"), pt("4", "0"))

	output := dbox2d.RayCastCapsule(&input, &point)

	// The delegation target gives the oracle, and the literal values pin it.
	circle := dbox2d.Circle{Center: pt("0", "0"), Radius: radius}
	viaCircle := dbox2d.RayCastCircle(&input, &circle)
	if output != viaCircle {
		t.Errorf("capsule cast = %+v, circle cast = %+v", output, viaCircle)
	}
	if !output.Hit {
		t.Fatalf("the ray misses the zero-length capsule")
	}
	if want := fixed.MustParse("0.375"); !output.Fraction.Eq(want) {
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
	capsule := dbox2d.Capsule{Center1: pt("-1", "0"), Center2: pt("1", "0"), Radius: fixed.MustParse("0.25")}
	beside := ray(pt("-2", "1"), pt("4", "0"))
	if dbox2d.RayCastCapsule(&beside, &capsule).Hit {
		t.Errorf("a parallel ray outside the capsule reports a hit")
	}
}

// TestPolygonConstructorsRejectInvalidHull checks the common validation
// boundary before either constructor computes normals or a centroid.
func TestPolygonConstructorsRejectInvalidHull(t *testing.T) {
	hull := dbox2d.Hull{Count: 3}
	hull.Points[0] = pt("0", "0")
	hull.Points[1] = pt("1", "0")
	hull.Points[2] = pt("1", "0")

	cases := []struct {
		name  string
		build func()
	}{
		{"MakePolygon", func() { dbox2d.MakePolygon(&hull, fixed.Zero()) }},
		{"MakeOffsetRoundedPolygon", func() {
			dbox2d.MakeOffsetRoundedPolygon(&hull, pt("0", "0"), dbox2d.RotIdentity(), fixed.Half())
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
	polygon := dbox2d.Polygon{Count: 3}
	polygon.Vertices[0] = pt("0", "0")
	polygon.Vertices[1] = pt("1", "0")
	polygon.Vertices[2] = pt("2", "0")

	defer func() {
		if recover() == nil {
			t.Errorf("ComputePolygonMass accepts a polygon with zero area")
		}
	}()
	dbox2d.ComputePolygonMass(&polygon, fixed.One())
}

// TestPolygonConstructorsMatchTheReference pins the literal layout of each
// constructor on dyadic inputs. Vertices, normals and radius are exact
// copies, so they compare exactly; the centroid integral may floor.
func TestPolygonConstructorsMatchTheReference(t *testing.T) {
	unitSquare := dbox2d.ComputeHull([]dbox2d.Vec2{
		pt("-1", "-1"), pt("1", "-1"), pt("1", "1"), pt("-1", "1"),
	})
	quarter := fixed.MustParse("0.25")

	boxNormals := []dbox2d.Vec2{pt("0", "-1"), pt("1", "0"), pt("0", "1"), pt("-1", "0")}

	cases := []struct {
		name     string
		got      dbox2d.Polygon
		vertices []dbox2d.Vec2
		normals  []dbox2d.Vec2
		centroid dbox2d.Vec2
		radius   dbox2d.Q
	}{
		{
			"MakeSquare",
			dbox2d.MakeSquare(fixed.Half()),
			[]dbox2d.Vec2{pt("-0.5", "-0.5"), pt("0.5", "-0.5"), pt("0.5", "0.5"), pt("-0.5", "0.5")},
			boxNormals, pt("0", "0"), fixed.Zero(),
		},
		{
			"MakeRoundedBox",
			dbox2d.MakeRoundedBox(fixed.One(), fixed.FromInt(2), quarter),
			[]dbox2d.Vec2{pt("-1", "-2"), pt("1", "-2"), pt("1", "2"), pt("-1", "2")},
			boxNormals, pt("0", "0"), quarter,
		},
		{
			"MakeOffsetPolygon",
			dbox2d.MakeOffsetPolygon(&unitSquare, pt("2", "3"), dbox2d.RotIdentity()),
			[]dbox2d.Vec2{pt("1", "2"), pt("3", "2"), pt("3", "4"), pt("1", "4")},
			boxNormals, pt("2", "3"), fixed.Zero(),
		},
		{
			"MakeOffsetRoundedPolygon",
			dbox2d.MakeOffsetRoundedPolygon(&unitSquare, pt("2", "3"), dbox2d.RotIdentity(), quarter),
			[]dbox2d.Vec2{pt("1", "2"), pt("3", "2"), pt("3", "4"), pt("1", "4")},
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
	xf := dbox2d.Transform{P: pt("1", "2"), Q: dbox2d.RotIdentity()}
	quarter := fixed.MustParse("0.25")

	circle := dbox2d.Circle{Center: pt("0.5", "0"), Radius: quarter}
	capsule := dbox2d.Capsule{Center1: pt("-0.5", "0"), Center2: pt("0.5", "0"), Radius: quarter}
	segment := dbox2d.Segment{Point1: pt("0", "-1"), Point2: pt("2", "1")}

	cases := []struct {
		name         string
		got          dbox2d.AABB
		lower, upper dbox2d.Vec2
	}{
		{"circle", dbox2d.ComputeCircleAABB(&circle, xf), pt("1.25", "1.75"), pt("1.75", "2.25")},
		{"capsule", dbox2d.ComputeCapsuleAABB(&capsule, xf), pt("0.25", "1.75"), pt("1.75", "2.25")},
		{"segment", dbox2d.ComputeSegmentAABB(&segment, xf), pt("1", "1"), pt("3", "3")},
	}

	for _, c := range cases {
		if c.got.LowerBound != c.lower || c.got.UpperBound != c.upper {
			t.Errorf("%s: aabb = %+v, want [%v, %v]", c.name, c.got, c.lower, c.upper)
		}
	}
}
