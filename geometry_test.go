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

// TestComputeCircleMassMatchesTheReference pins the circle integral. A unit
// circle of unit density has a mass of pi and half that as inertia.
func TestComputeCircleMassMatchesTheReference(t *testing.T) {
	circle := dbox2d.Circle{Radius: fixed.One()}
	data := dbox2d.ComputeCircleMass(&circle, fixed.One())

	limit := tol(1, 10000)
	if !near(data.Mass, dbox2d.Pi(), limit) {
		t.Errorf("mass = %v, want pi", data.Mass)
	}
	if want := dbox2d.Pi().Mul(fixed.Half()); !near(data.RotationalInertia, want, limit) {
		t.Errorf("rotational inertia = %v, want %v", data.RotationalInertia, want)
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

// TestPolygonCentroidOfATriangle pins the area centroid, which is the mean
// of the three vertices.
func TestPolygonCentroidOfATriangle(t *testing.T) {
	hull := dbox2d.ComputeHull([]dbox2d.Vec2{pt("0", "0"), pt("3", "0"), pt("0", "3")})
	polygon := dbox2d.MakePolygon(&hull, fixed.Zero())

	limit := tol(1, 1000)
	if !near(polygon.Centroid.X, fixed.One(), limit) || !near(polygon.Centroid.Y, fixed.One(), limit) {
		t.Errorf("centroid = %v, want (1, 1)", polygon.Centroid)
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

// TestTransformPolygonAgreesWithMakeOffsetBox checks that moving a shape
// gives the same polygon as building it in place.
func TestTransformPolygonAgreesWithMakeOffsetBox(t *testing.T) {
	center := pt("2", "-3")
	rotation := dbox2d.MakeRot(fixed.MustParse("0.2"))

	box := dbox2d.MakeBox(fixed.One(), fixed.Half())
	got := dbox2d.TransformPolygon(dbox2d.Transform{P: center, Q: rotation}, &box)
	want := dbox2d.MakeOffsetBox(fixed.One(), fixed.Half(), center, rotation)

	if got != want {
		t.Errorf("TransformPolygon = %v, want %v", got, want)
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

// TestRayCastCapsuleHitsTheSide pins the Cramer solve of the capsule cast.
func TestRayCastCapsuleHitsTheSide(t *testing.T) {
	capsule := dbox2d.Capsule{Center1: pt("-1", "0"), Center2: pt("1", "0"), Radius: fixed.Half()}
	input := ray(pt("0", "3"), pt("0", "-6"))

	output := dbox2d.RayCastCapsule(&input, &capsule)

	if !output.Hit {
		t.Fatalf("the ray misses the capsule")
	}
	limit := tol(1, 1000)
	if want := fixed.FromRatio(5, 12); !near(output.Fraction, want, limit) {
		t.Errorf("fraction = %v, want %v", output.Fraction, want)
	}
	if !near(output.Point.X, fixed.Zero(), limit) || !near(output.Point.Y, fixed.Half(), limit) {
		t.Errorf("point = %v, want (0, 0.5)", output.Point)
	}
	if !near(output.Normal.Y, fixed.One(), limit) {
		t.Errorf("normal = %v, want (0, 1)", output.Normal)
	}
}

// TestIsValidRayBoundsTheFraction guards the input check that every cast
// runs first.
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
