package dbox2d_test

import (
	"encoding/binary"
	"hash/fnv"
	"math/rand"
	"testing"

	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/fixed"
)

func vec(x, y int) dbox2d.Vec2 {
	return dbox2d.Vec2{X: fixed.Q32FromInt(x), Y: fixed.Q32FromInt(y)}
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
	half := fixed.Q32MustParse("0.5")
	one := fixed.Q32One()
	zero := fixed.Q32Zero()

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
			f1: one, f2: one, distSq: fixed.Q32FromInt(2),
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
	zero := fixed.Q32Zero()
	one := fixed.Q32One()

	// Segment 2 is a point past the end of segment 1: f1 clamps to one.
	point := vec(3, 4)
	result := dbox2d.SegmentDistance(vec(0, 0), vec(2, 0), point, point)
	if !result.Fraction1.Eq(one) || !result.Fraction2.Eq(zero) {
		t.Fatalf("fractions (%v, %v), want (1, 0)", result.Fraction1, result.Fraction2)
	}
	if !result.DistanceSquared.Eq(fixed.Q32FromInt(17)) {
		t.Fatalf("distanceSquared %v, want 17", result.DistanceSquared)
	}

	// Both segments are points: the distance is between the points.
	result = dbox2d.SegmentDistance(vec(1, 2), vec(1, 2), vec(4, 6), vec(4, 6))
	if !result.Fraction1.Eq(zero) || !result.Fraction2.Eq(zero) {
		t.Fatalf("fractions (%v, %v), want (0, 0)", result.Fraction1, result.Fraction2)
	}
	if !result.DistanceSquared.Eq(fixed.Q32FromInt(25)) {
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

	radius := fixed.Q32MustParse("0.25")
	proxy := dbox2d.MakeProxy(points, radius)
	if proxy.Count != dbox2d.MaxPolygonVertices {
		t.Fatalf("count %d, want %d", proxy.Count, dbox2d.MaxPolygonVertices)
	}
	if !proxy.Radius.Eq(radius) {
		t.Fatalf("radius %v, want %v", proxy.Radius, radius)
	}
	last := proxy.Points[dbox2d.MaxPolygonVertices-1]
	if !last.X.Eq(fixed.Q32FromInt(dbox2d.MaxPolygonVertices - 1)) {
		t.Fatalf("last point %v, want x=%d", last, dbox2d.MaxPolygonVertices-1)
	}
}

func boxProxy(halfWidth, halfHeight int, center dbox2d.Vec2) dbox2d.ShapeProxy {
	box := dbox2d.MakeBox(fixed.Q32FromInt(halfWidth), fixed.Q32FromInt(halfHeight))
	return dbox2d.MakeOffsetProxy(box.Vertices[:box.Count], fixed.Q32Zero(), center, dbox2d.RotIdentity())
}

func identityDistanceInput(a, b dbox2d.ShapeProxy, useRadii bool) dbox2d.DistanceInput {
	return dbox2d.DistanceInput{
		ProxyA:     a,
		ProxyB:     b,
		TransformA: dbox2d.TransformIdentity(),
		TransformB: dbox2d.TransformIdentity(),
		UseRadii:   useRadii,
	}
}

// TestShapeDistanceMatchesHandCases pins the GJK output on inputs whose
// answer is exact: two boxes a unit apart, two circles as points with
// radii, and a point against a segment.
func TestShapeDistanceMatchesHandCases(t *testing.T) {
	var cache dbox2d.SimplexCache

	t.Run("boxes", func(t *testing.T) {
		input := identityDistanceInput(boxProxy(1, 1, vec(0, 0)), boxProxy(1, 1, vec(3, 0)), false)
		out := dbox2d.ShapeDistance(&input, &cache, nil)
		if !out.Distance.Eq(fixed.Q32One()) || !out.Normal.X.Eq(fixed.Q32One()) || !out.Normal.Y.Eq(fixed.Q32Zero()) {
			t.Fatalf("distance %v normal %v, want 1 and (1, 0)", out.Distance, out.Normal)
		}
		if !out.PointA.X.Eq(fixed.Q32One()) || !out.PointB.X.Eq(fixed.Q32FromInt(2)) {
			t.Fatalf("points %v %v, want x = 1 and x = 2", out.PointA, out.PointB)
		}
		if out.Iterations == 0 || out.Iterations > 20 {
			t.Fatalf("iterations %d", out.Iterations)
		}
	})

	t.Run("circles with radii", func(t *testing.T) {
		half := fixed.Q32Half()
		a := dbox2d.MakeProxy([]dbox2d.Vec2{vec(0, 0)}, half)
		b := dbox2d.MakeProxy([]dbox2d.Vec2{vec(3, 0)}, half)
		input := identityDistanceInput(a, b, true)
		out := dbox2d.ShapeDistance(&input, &cache, nil)
		if !out.Distance.Eq(fixed.Q32FromInt(2)) {
			t.Fatalf("distance %v, want 2", out.Distance)
		}
		if !out.PointA.X.Eq(half) || !out.PointB.X.Eq(fixed.Q32FromRatio(5, 2)) {
			t.Fatalf("points %v %v, want x = 0.5 and x = 2.5", out.PointA, out.PointB)
		}
	})

	t.Run("point and segment", func(t *testing.T) {
		// The closest point of the segment from (1, -1) to (1, 1) to the
		// origin is (1, 0), which the two-simplex reaches with a1 = a2 = 1/2.
		a := dbox2d.MakeProxy([]dbox2d.Vec2{vec(0, 0)}, fixed.Q32Zero())
		b := dbox2d.MakeProxy([]dbox2d.Vec2{vec(1, -1), vec(1, 1)}, fixed.Q32Zero())
		input := identityDistanceInput(a, b, false)
		var simplexes [4]dbox2d.Simplex
		out := dbox2d.ShapeDistance(&input, &cache, simplexes[:])
		if !out.Distance.Eq(fixed.Q32One()) || !out.PointB.X.Eq(fixed.Q32One()) || !out.PointB.Y.Eq(fixed.Q32Zero()) {
			t.Fatalf("distance %v point B %v, want 1 and (1, 0)", out.Distance, out.PointB)
		}
		if out.SimplexCount < 2 || simplexes[out.SimplexCount-1].Count != 2 {
			t.Fatalf("simplex count %d, last simplex %+v", out.SimplexCount, simplexes[out.SimplexCount-1])
		}
		if cache.Count != 2 {
			t.Fatalf("cache count %d, want 2", cache.Count)
		}
	})
}

// TestShapeDistanceReportsOverlap covers the two overlap exits: the
// origin inside the triangle and the exact zero search direction that
// replaces the FLT_EPSILON test of the reference (D-012).
func TestShapeDistanceReportsOverlap(t *testing.T) {
	t.Run("triangle contains the origin", func(t *testing.T) {
		var cache dbox2d.SimplexCache
		input := identityDistanceInput(boxProxy(1, 1, vec(0, 0)), boxProxy(1, 1, vecQ("0.5", "0.25")), false)
		out := dbox2d.ShapeDistance(&input, &cache, nil)
		if !out.Distance.Eq(fixed.Q32Zero()) {
			t.Fatalf("distance %v, want 0", out.Distance)
		}
	})

	t.Run("origin on the segment", func(t *testing.T) {
		// The point sits on the segment, so the two-simplex holds the
		// origin and its search direction is the exact zero vector.
		var cache dbox2d.SimplexCache
		a := dbox2d.MakeProxy([]dbox2d.Vec2{vec(0, 0)}, fixed.Q32Zero())
		b := dbox2d.MakeProxy([]dbox2d.Vec2{vec(-1, 0), vec(1, 0)}, fixed.Q32Zero())
		input := identityDistanceInput(a, b, false)
		out := dbox2d.ShapeDistance(&input, &cache, nil)
		if !out.Distance.Eq(fixed.Q32Zero()) || !out.PointA.X.Eq(fixed.Q32Zero()) {
			t.Fatalf("distance %v point A %v, want 0 and the origin", out.Distance, out.PointA)
		}
	})
}

// TestShapeDistanceWarmStartsFromTheCache reruns a query with the cache
// of the first run and expects the same answer in no more iterations.
func TestShapeDistanceWarmStartsFromTheCache(t *testing.T) {
	var cache dbox2d.SimplexCache
	input := identityDistanceInput(boxProxy(1, 2, vec(0, 0)), boxProxy(2, 1, vec(4, 3)), false)
	input.TransformB.Q = dbox2d.MakeRot(fixed.Q32FromRatio(1, 8))

	cold := dbox2d.ShapeDistance(&input, &cache, nil)
	warm := dbox2d.ShapeDistance(&input, &cache, nil)

	if !cold.Distance.Eq(warm.Distance) || cold.PointA != warm.PointA {
		t.Fatalf("cold %+v, warm %+v", cold, warm)
	}
	if warm.Iterations > cold.Iterations {
		t.Fatalf("warm start took %d iterations, cold %d", warm.Iterations, cold.Iterations)
	}
}

// randomDistanceInput draws a pair of boxes, capsules or circles on a
// millimetre grid inside a ten metre box.
func randomDistanceInput(rng *rand.Rand) dbox2d.DistanceInput {
	milli := func(lo, hi int) dbox2d.Q { return fixed.Q32FromRatio(lo+rng.Intn(hi-lo+1), 1000) }
	proxy := func() dbox2d.ShapeProxy {
		switch rng.Intn(3) {
		case 0:
			box := dbox2d.MakeBox(milli(100, 2000), milli(100, 2000))
			return dbox2d.MakeProxy(box.Vertices[:box.Count], fixed.Q32Zero())
		case 1:
			points := []dbox2d.Vec2{{X: milli(-1000, 1000), Y: milli(-1000, 1000)}, {X: milli(-1000, 1000), Y: milli(-1000, 1000)}}
			return dbox2d.MakeProxy(points, milli(50, 500))
		default:
			return dbox2d.MakeProxy([]dbox2d.Vec2{{X: milli(-500, 500), Y: milli(-500, 500)}}, milli(50, 1000))
		}
	}
	transform := func() dbox2d.Transform {
		return dbox2d.Transform{
			P: dbox2d.Vec2{X: milli(-5000, 5000), Y: milli(-5000, 5000)},
			Q: dbox2d.MakeRot(milli(0, 999)),
		}
	}
	return dbox2d.DistanceInput{
		ProxyA:     proxy(),
		ProxyB:     proxy(),
		TransformA: transform(),
		TransformB: transform(),
		UseRadii:   true,
	}
}

// TestShapeDistanceConvergesOnRandomPairs runs a thousand random pairs
// and checks the iteration bound, the unit normal, a zero saturation
// count and a fixed witness of the result bits.
func TestShapeDistanceConvergesOnRandomPairs(t *testing.T) {
	const witness uint64 = 13937014052321988253

	rng := rand.New(rand.NewSource(1))
	fixed.ResetSaturationCount()
	h := fnv.New64a()
	var buf [8]byte

	for range 1000 {
		input := randomDistanceInput(rng)
		var cache dbox2d.SimplexCache
		out := dbox2d.ShapeDistance(&input, &cache, nil)

		if out.Iterations > 20 {
			t.Fatalf("iterations %d on %+v", out.Iterations, input)
		}
		if out.Distance.Less(fixed.Q32Zero()) {
			t.Fatalf("negative distance %v", out.Distance)
		}
		if fixed.Q32Zero().Less(out.Distance) && !dbox2d.IsNormalized(out.Normal) {
			t.Fatalf("normal %v is not unit for distance %v", out.Normal, out.Distance)
		}

		for _, q := range []dbox2d.Q{out.Distance, out.PointA.X, out.PointA.Y, out.PointB.X, out.PointB.Y} {
			binary.LittleEndian.PutUint64(buf[:], uint64(q.Raw()))
			h.Write(buf[:])
		}
	}

	if n := fixed.SaturationCount(); n != 0 {
		t.Fatalf("saturation count %d", n)
	}
	if got := h.Sum64(); got != witness {
		t.Fatalf("witness %d, want %d", got, witness)
	}
}

// TestGetSweepTransformInterpolatesTheCenter checks the two ends and the
// midpoint of a sweep with an offset center of mass.
func TestGetSweepTransformInterpolatesTheCenter(t *testing.T) {
	sweep := dbox2d.Sweep{
		LocalCenter: vec(1, 0),
		C1:          vec(1, 0),
		C2:          vec(5, 0),
		Q1:          dbox2d.RotIdentity(),
		Q2:          dbox2d.RotIdentity(),
	}

	start := dbox2d.GetSweepTransform(&sweep, fixed.Q32Zero())
	mid := dbox2d.GetSweepTransform(&sweep, fixed.Q32Half())
	end := dbox2d.GetSweepTransform(&sweep, fixed.Q32One())

	if start.P != vec(0, 0) || mid.P != vec(2, 0) || end.P != vec(4, 0) {
		t.Fatalf("origins %v %v %v, want x = 0, 2, 4", start.P, mid.P, end.P)
	}
}

// TestShapeCastMatchesHandCases pins the conservative advancement on
// boxes whose hit fraction is known: the sweep stops once the gap falls
// under the target, so the fraction lands just short of the exact value.
func TestShapeCastMatchesHandCases(t *testing.T) {
	one := fixed.Q32One()
	a := boxProxy(1, 1, vec(0, 0))
	b := boxProxy(1, 1, vec(0, 0))
	pair := func(positionB, translation dbox2d.Vec2) dbox2d.ShapeCastPairInput {
		return dbox2d.ShapeCastPairInput{
			ProxyA:       a,
			ProxyB:       b,
			TransformA:   dbox2d.TransformIdentity(),
			TransformB:   dbox2d.Transform{P: positionB, Q: dbox2d.RotIdentity()},
			TranslationB: translation,
			MaxFraction:  one,
		}
	}

	t.Run("hit", func(t *testing.T) {
		input := pair(vec(5, 0), vec(-10, 0))
		out := dbox2d.ShapeCast(&input)
		if !out.Hit {
			t.Fatal("the sweep missed")
		}
		// The faces meet at a fraction of 0.3; the sweep stops a target
		// short, within a quarter of a slop.
		exact := fixed.Q32MustParse("0.3")
		if !out.Fraction.Less(exact) || exact.Sub(out.Fraction).Less(fixed.Q32Zero()) || !exact.Sub(out.Fraction).Less(fixed.Q32MustParse("0.001")) {
			t.Fatalf("fraction %v, want just under 0.3", out.Fraction)
		}
		if out.Normal != vec(1, 0) || !out.Point.X.Eq(one) {
			t.Fatalf("normal %v point %v, want (1, 0) on the face x = 1", out.Normal, out.Point)
		}
		if out.Iterations == 0 || out.Iterations > 20 {
			t.Fatalf("iterations %d", out.Iterations)
		}
	})

	t.Run("miss", func(t *testing.T) {
		input := pair(vec(5, 0), vec(0, 10))
		if out := dbox2d.ShapeCast(&input); out.Hit {
			t.Fatalf("a parallel sweep hit at %v", out.Fraction)
		}
	})

	t.Run("out of range", func(t *testing.T) {
		input := pair(vec(5, 0), vec(-1, 0))
		if out := dbox2d.ShapeCast(&input); out.Hit {
			t.Fatalf("a short sweep hit at %v", out.Fraction)
		}
	})

	t.Run("initial overlap", func(t *testing.T) {
		input := pair(vec(1, 0), vec(-10, 0))
		out := dbox2d.ShapeCast(&input)
		if !out.Hit || !out.Fraction.Eq(fixed.Q32Zero()) || out.Normal != vec(0, 0) {
			t.Fatalf("overlap reported %+v, want a hit at zero with no normal", out)
		}
	})

	t.Run("encroach", func(t *testing.T) {
		// Two circles whose surfaces overlap by more than two slops may
		// move a little closer: the sweep hits, but not at zero.
		half := fixed.Q32Half()
		input := dbox2d.ShapeCastPairInput{
			ProxyA:       dbox2d.MakeProxy([]dbox2d.Vec2{vec(0, 0)}, half),
			ProxyB:       dbox2d.MakeProxy([]dbox2d.Vec2{vec(0, 0)}, half),
			TransformA:   dbox2d.TransformIdentity(),
			TransformB:   dbox2d.Transform{P: vecQ("0.9", "0"), Q: dbox2d.RotIdentity()},
			TranslationB: vec(-1, 0),
			MaxFraction:  one,
			CanEncroach:  true,
		}
		out := dbox2d.ShapeCast(&input)
		if !out.Hit || !fixed.Q32Zero().Less(out.Fraction) || !out.Fraction.Less(fixed.Q32MustParse("0.01")) {
			t.Fatalf("encroach reported %+v, want a small positive fraction", out)
		}
	})
}

// toiPair builds a time of impact input for two proxies. A rests at the
// origin; B translates from c1 to c2 and turns from q1 to q2.
func toiPair(a, b dbox2d.ShapeProxy, c1, c2 dbox2d.Vec2, q1, q2 dbox2d.Rot) dbox2d.TOIInput {
	return dbox2d.TOIInput{
		ProxyA:      a,
		ProxyB:      b,
		SweepA:      dbox2d.Sweep{Q1: dbox2d.RotIdentity(), Q2: dbox2d.RotIdentity()},
		SweepB:      dbox2d.Sweep{C1: c1, C2: c2, Q1: q1, Q2: q2},
		MaxFraction: fixed.Q32One(),
	}
}

// toiGap returns the core distance of the proxies at the fraction, without
// the radii, which is the quantity the solver drives to its target.
func toiGap(input *dbox2d.TOIInput, fraction dbox2d.Q) dbox2d.Q {
	distanceInput := dbox2d.DistanceInput{
		ProxyA:     input.ProxyA,
		ProxyB:     input.ProxyB,
		TransformA: dbox2d.GetSweepTransform(&input.SweepA, fraction),
		TransformB: dbox2d.GetSweepTransform(&input.SweepB, fraction),
	}
	var cache dbox2d.SimplexCache
	return dbox2d.ShapeDistance(&distanceInput, &cache, nil).Distance
}

// toiTarget returns the separation the solver seeks and the band around it.
func toiTarget(input *dbox2d.TOIInput) (target, tolerance dbox2d.Q) {
	slop := dbox2d.LinearSlop()
	totalRadius := input.ProxyA.Radius.Add(input.ProxyB.Radius)
	return slop.Max(totalRadius.Sub(slop)), slop.Div(fixed.Q32FromInt(4))
}

// TestTimeOfImpactMatchesHandCases pins the solver on sweeps whose answer
// is known. The hit fraction lands where the gap equals one slop, within a
// quarter of a slop.
func TestTimeOfImpactMatchesHandCases(t *testing.T) {
	identity := dbox2d.RotIdentity()
	a := boxProxy(1, 1, vec(0, 0))
	b := boxProxy(1, 1, vec(0, 0))

	t.Run("hit", func(t *testing.T) {
		input := toiPair(a, b, vec(5, 0), vec(-5, 0), identity, identity)
		out := dbox2d.TimeOfImpact(&input)
		if out.State != dbox2d.TOIStateHit {
			t.Fatalf("state %v, want a hit", out.State)
		}
		// The faces close a gap of 3 at a speed of 10 and stop one slop
		// short: (3 - 0.005) / 10.
		if !near(out.Fraction, fixed.Q32MustParse("0.2995"), fixed.Q32MustParse("0.0002")) {
			t.Fatalf("fraction %v, want about 0.2995", out.Fraction)
		}
	})

	t.Run("separated", func(t *testing.T) {
		input := toiPair(a, b, vec(5, 0), vec(10, 0), identity, identity)
		out := dbox2d.TimeOfImpact(&input)
		if out.State != dbox2d.TOIStateSeparated || !out.Fraction.Eq(fixed.Q32One()) {
			t.Fatalf("state %v fraction %v, want separated at 1", out.State, out.Fraction)
		}
	})

	t.Run("out of range", func(t *testing.T) {
		input := toiPair(a, b, vec(5, 0), vec(-5, 0), identity, identity)
		input.MaxFraction = fixed.Q32FromRatio(1, 4)
		out := dbox2d.TimeOfImpact(&input)
		if out.State != dbox2d.TOIStateSeparated || !out.Fraction.Eq(input.MaxFraction) {
			t.Fatalf("state %v fraction %v, want separated at the max fraction", out.State, out.Fraction)
		}
	})

	t.Run("overlapped", func(t *testing.T) {
		input := toiPair(a, b, vec(0, 0), vec(5, 0), identity, identity)
		out := dbox2d.TimeOfImpact(&input)
		if out.State != dbox2d.TOIStateOverlapped || !out.Fraction.Eq(fixed.Q32Zero()) {
			t.Fatalf("state %v fraction %v, want overlapped at 0", out.State, out.Fraction)
		}
	})

	t.Run("touching", func(t *testing.T) {
		// The gap starts at half a slop, inside the target band.
		input := toiPair(a, b, vecQ("2.0025", "0"), vec(-5, 0), identity, identity)
		out := dbox2d.TimeOfImpact(&input)
		if out.State != dbox2d.TOIStateHit || !out.Fraction.Eq(fixed.Q32Zero()) {
			t.Fatalf("state %v fraction %v, want a hit at 0", out.State, out.Fraction)
		}
	})

	t.Run("rotation", func(t *testing.T) {
		// A rod turns a quarter turn about its center and sweeps a small
		// box that sits above it. The translation alone never touches.
		rod := boxProxy(3, 1, vec(0, 0))
		rod.Points[0].Y = fixed.Q32MustParse("-0.1")
		rod.Points[1].Y = fixed.Q32MustParse("-0.1")
		rod.Points[2].Y = fixed.Q32MustParse("0.1")
		rod.Points[3].Y = fixed.Q32MustParse("0.1")
		input := dbox2d.TOIInput{
			ProxyA:      boxProxy(1, 1, vec(0, 0)),
			ProxyB:      rod,
			SweepA:      dbox2d.Sweep{C1: vec(2, 2), C2: vec(2, 2), Q1: identity, Q2: identity},
			SweepB:      dbox2d.Sweep{Q1: identity, Q2: dbox2d.MakeRot(fixed.Q32FromRatio(1, 4))},
			MaxFraction: fixed.Q32One(),
		}
		out := dbox2d.TimeOfImpact(&input)
		if out.State != dbox2d.TOIStateHit {
			t.Fatalf("state %v, want a hit", out.State)
		}
		if !fixed.Q32Zero().Less(out.Fraction) || !out.Fraction.Less(fixed.Q32One()) {
			t.Fatalf("fraction %v, want inside (0, 1)", out.Fraction)
		}
		target, tolerance := toiTarget(&input)
		gap := toiGap(&input, out.Fraction)
		if !near(gap, target, tolerance) {
			t.Fatalf("gap %v at the fraction, want %v within %v", gap, target, tolerance)
		}
	})
}

// randomTOIInput builds a sweep pair on a millimetre grid. The rotations
// turn at most a quarter turn, so the interpolated rotation never
// collapses to zero.
func randomTOIInput(rng *rand.Rand) dbox2d.TOIInput {
	milli := func(lo, hi int) dbox2d.Q { return fixed.Q32FromRatio(lo+rng.Intn(hi-lo+1), 1000) }
	sweep := func() dbox2d.Sweep {
		turn := milli(0, 999)
		return dbox2d.Sweep{
			LocalCenter: dbox2d.Vec2{X: milli(-500, 500), Y: milli(-500, 500)},
			C1:          dbox2d.Vec2{X: milli(-5000, 5000), Y: milli(-5000, 5000)},
			C2:          dbox2d.Vec2{X: milli(-5000, 5000), Y: milli(-5000, 5000)},
			Q1:          dbox2d.MakeRot(turn),
			Q2:          dbox2d.MakeRot(turn.Add(milli(-250, 250))),
		}
	}
	distance := randomDistanceInput(rng)
	return dbox2d.TOIInput{
		ProxyA:      distance.ProxyA,
		ProxyB:      distance.ProxyB,
		SweepA:      sweep(),
		SweepB:      sweep(),
		MaxFraction: fixed.Q32One(),
	}
}

// TestTimeOfImpactConvergesOnRandomSweeps checks every result state on
// random sweeps against the gap at the reported fraction, pins the bits
// with a witness and confirms that no operation saturated.
func TestTimeOfImpactConvergesOnRandomSweeps(t *testing.T) {
	const witness uint64 = 3402533517278441094

	rng := rand.New(rand.NewSource(11))
	hash := fnv.New64a()
	var buf [8]byte
	fixed.ResetSaturationCount()
	hits, failed := 0, 0

	for range 1000 {
		input := randomTOIInput(rng)
		out := dbox2d.TimeOfImpact(&input)
		target, tolerance := toiTarget(&input)

		switch out.State {
		case dbox2d.TOIStateHit:
			hits++
			gap := toiGap(&input, out.Fraction)
			if fixed.Q32Zero().Less(out.Fraction) && !near(gap, target, tolerance) {
				t.Fatalf("hit at %v with a gap of %v, want %v within %v", out.Fraction, gap, target, tolerance)
			}
			if !fixed.Q32Zero().Less(gap) || target.Add(tolerance).Less(gap) {
				t.Fatalf("hit at %v with a gap of %v, want inside (0, %v]", out.Fraction, gap, target.Add(tolerance))
			}
		case dbox2d.TOIStateSeparated:
			if !out.Fraction.Eq(fixed.Q32One()) {
				t.Fatalf("separated at %v, want 1", out.Fraction)
			}
			if gap := toiGap(&input, out.Fraction); !target.Less(gap) {
				t.Fatalf("separated with a gap of %v, want over %v", gap, target)
			}
		case dbox2d.TOIStateOverlapped:
			if !out.Fraction.Eq(fixed.Q32Zero()) {
				t.Fatalf("overlapped at %v, want 0", out.Fraction)
			}
		case dbox2d.TOIStateFailed:
			failed++
		default:
			t.Fatalf("state %v", out.State)
		}

		binary.LittleEndian.PutUint64(buf[:], uint64(out.State))
		hash.Write(buf[:])
		binary.LittleEndian.PutUint64(buf[:], uint64(out.Fraction.Raw()))
		hash.Write(buf[:])
	}

	if hits < 100 {
		t.Fatalf("only %d hits", hits)
	}
	if failed > 10 {
		t.Fatalf("%d sweeps failed", failed)
	}
	if n := fixed.SaturationCount(); n != 0 {
		t.Fatalf("%d operations saturated", n)
	}
	if got := hash.Sum64(); got != witness {
		t.Fatalf("witness %d, want %d", got, witness)
	}
}
