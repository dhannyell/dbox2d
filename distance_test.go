package b2_test

import (
	"encoding/binary"
	"hash/fnv"
	"math/rand"
	"testing"

	"github.com/dhannyell/dbox2d"
)

func vec(x, y int) b2.Vec2 {
	return b2.Vec2{X: b2.QFromInt(x), Y: b2.QFromInt(y)}
}

// segmentDistanceCase carries exact expected values from the reference
// algorithm on integer inputs, where Q arithmetic has no rounding.
type segmentDistanceCase struct {
	name           string
	p1, q1, p2, q2 b2.Vec2
	f1, f2         b2.Q
	distSq         b2.Q
}

// TestSegmentDistanceMatchesTheReference walks the branches of the
// closed-form algorithm: intersection, the do-over clamps of segment 2
// and parallel segments.
func TestSegmentDistanceMatchesTheReference(t *testing.T) {
	half := b2.QMustParse("0.5")
	one := b2.QOne()
	zero := b2.QZero()

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
			f1: one, f2: one, distSq: b2.QFromInt(2),
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
			result := b2.SegmentDistance(c.p1, c.q1, c.p2, c.q2)
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
	zero := b2.QZero()
	one := b2.QOne()

	// Segment 2 is a point past the end of segment 1: f1 clamps to one.
	point := vec(3, 4)
	result := b2.SegmentDistance(vec(0, 0), vec(2, 0), point, point)
	if !result.Fraction1.Eq(one) || !result.Fraction2.Eq(zero) {
		t.Fatalf("fractions (%v, %v), want (1, 0)", result.Fraction1, result.Fraction2)
	}
	if !result.DistanceSquared.Eq(b2.QFromInt(17)) {
		t.Fatalf("distanceSquared %v, want 17", result.DistanceSquared)
	}

	// Both segments are points: the distance is between the points.
	result = b2.SegmentDistance(vec(1, 2), vec(1, 2), vec(4, 6), vec(4, 6))
	if !result.Fraction1.Eq(zero) || !result.Fraction2.Eq(zero) {
		t.Fatalf("fractions (%v, %v), want (0, 0)", result.Fraction1, result.Fraction2)
	}
	if !result.DistanceSquared.Eq(b2.QFromInt(25)) {
		t.Fatalf("distanceSquared %v, want 25", result.DistanceSquared)
	}
}

// TestMakeProxyTruncatesToThePolygonLimit checks the only decision the
// constructor makes: extra points do not enter.
func TestMakeProxyTruncatesToThePolygonLimit(t *testing.T) {
	points := make([]b2.Vec2, b2.MaxPolygonVertices+2)
	for i := range points {
		points[i] = vec(i, -i)
	}

	radius := b2.QMustParse("0.25")
	proxy := b2.MakeProxy(points, radius)
	if proxy.Count != b2.MaxPolygonVertices {
		t.Fatalf("count %d, want %d", proxy.Count, b2.MaxPolygonVertices)
	}
	if !proxy.Radius.Eq(radius) {
		t.Fatalf("radius %v, want %v", proxy.Radius, radius)
	}
	last := proxy.Points[b2.MaxPolygonVertices-1]
	if !last.X.Eq(b2.QFromInt(b2.MaxPolygonVertices - 1)) {
		t.Fatalf("last point %v, want x=%d", last, b2.MaxPolygonVertices-1)
	}
}

func boxProxy(halfWidth, halfHeight int, center b2.Vec2) b2.ShapeProxy {
	box := b2.MakeBox(b2.QFromInt(halfWidth), b2.QFromInt(halfHeight))
	return b2.MakeOffsetProxy(box.Vertices[:box.Count], b2.QZero(), center, b2.RotIdentity())
}

func identityDistanceInput(a, b b2.ShapeProxy, useRadii bool) b2.DistanceInput {
	return b2.DistanceInput{
		ProxyA:     a,
		ProxyB:     b,
		TransformA: b2.TransformIdentity(),
		TransformB: b2.TransformIdentity(),
		UseRadii:   useRadii,
	}
}

// TestShapeDistanceMatchesHandCases pins the GJK output on inputs whose
// answer is exact: two boxes a unit apart, two circles as points with
// radii, and a point against a segment.
func TestShapeDistanceMatchesHandCases(t *testing.T) {
	var cache b2.SimplexCache

	t.Run("boxes", func(t *testing.T) {
		input := identityDistanceInput(boxProxy(1, 1, vec(0, 0)), boxProxy(1, 1, vec(3, 0)), false)
		out := b2.ShapeDistance(&input, &cache, nil)
		if !out.Distance.Eq(b2.QOne()) || !out.Normal.X.Eq(b2.QOne()) || !out.Normal.Y.Eq(b2.QZero()) {
			t.Fatalf("distance %v normal %v, want 1 and (1, 0)", out.Distance, out.Normal)
		}
		if !out.PointA.X.Eq(b2.QOne()) || !out.PointB.X.Eq(b2.QFromInt(2)) {
			t.Fatalf("points %v %v, want x = 1 and x = 2", out.PointA, out.PointB)
		}
		if out.Iterations == 0 || out.Iterations > 20 {
			t.Fatalf("iterations %d", out.Iterations)
		}
	})

	t.Run("circles with radii", func(t *testing.T) {
		half := b2.QHalf()
		a := b2.MakeProxy([]b2.Vec2{vec(0, 0)}, half)
		b := b2.MakeProxy([]b2.Vec2{vec(3, 0)}, half)
		input := identityDistanceInput(a, b, true)
		out := b2.ShapeDistance(&input, &cache, nil)
		if !out.Distance.Eq(b2.QFromInt(2)) {
			t.Fatalf("distance %v, want 2", out.Distance)
		}
		if !out.PointA.X.Eq(half) || !out.PointB.X.Eq(b2.QFromRatio(5, 2)) {
			t.Fatalf("points %v %v, want x = 0.5 and x = 2.5", out.PointA, out.PointB)
		}
	})

	t.Run("point and segment", func(t *testing.T) {
		// The closest point of the segment from (1, -1) to (1, 1) to the
		// origin is (1, 0), which the two-simplex reaches with a1 = a2 = 1/2.
		a := b2.MakeProxy([]b2.Vec2{vec(0, 0)}, b2.QZero())
		b := b2.MakeProxy([]b2.Vec2{vec(1, -1), vec(1, 1)}, b2.QZero())
		input := identityDistanceInput(a, b, false)
		var simplexes [4]b2.Simplex
		out := b2.ShapeDistance(&input, &cache, simplexes[:])
		if !out.Distance.Eq(b2.QOne()) || !out.PointB.X.Eq(b2.QOne()) || !out.PointB.Y.Eq(b2.QZero()) {
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
		var cache b2.SimplexCache
		input := identityDistanceInput(boxProxy(1, 1, vec(0, 0)), boxProxy(1, 1, vecQ("0.5", "0.25")), false)
		out := b2.ShapeDistance(&input, &cache, nil)
		if !out.Distance.Eq(b2.QZero()) {
			t.Fatalf("distance %v, want 0", out.Distance)
		}
	})

	t.Run("origin on the segment", func(t *testing.T) {
		// The point sits on the segment, so the two-simplex holds the
		// origin and its search direction is the exact zero vector.
		var cache b2.SimplexCache
		a := b2.MakeProxy([]b2.Vec2{vec(0, 0)}, b2.QZero())
		b := b2.MakeProxy([]b2.Vec2{vec(-1, 0), vec(1, 0)}, b2.QZero())
		input := identityDistanceInput(a, b, false)
		out := b2.ShapeDistance(&input, &cache, nil)
		if !out.Distance.Eq(b2.QZero()) || !out.PointA.X.Eq(b2.QZero()) {
			t.Fatalf("distance %v point A %v, want 0 and the origin", out.Distance, out.PointA)
		}
	})
}

// TestShapeDistanceWarmStartsFromTheCache reruns a query with the cache
// of the first run and expects the same answer in no more iterations.
func TestShapeDistanceWarmStartsFromTheCache(t *testing.T) {
	var cache b2.SimplexCache
	input := identityDistanceInput(boxProxy(1, 2, vec(0, 0)), boxProxy(2, 1, vec(4, 3)), false)
	input.TransformB.Q = b2.MakeRot(b2.QFromRatio(1, 8))

	cold := b2.ShapeDistance(&input, &cache, nil)
	warm := b2.ShapeDistance(&input, &cache, nil)

	if !cold.Distance.Eq(warm.Distance) || cold.PointA != warm.PointA {
		t.Fatalf("cold %+v, warm %+v", cold, warm)
	}
	if warm.Iterations > cold.Iterations {
		t.Fatalf("warm start took %d iterations, cold %d", warm.Iterations, cold.Iterations)
	}
}

// randomDistanceInput draws a pair of boxes, capsules or circles on a
// millimetre grid inside a ten metre box.
func randomDistanceInput(rng *rand.Rand) b2.DistanceInput {
	milli := func(lo, hi int) b2.Q { return b2.QFromRatio(lo+rng.Intn(hi-lo+1), 1000) }
	proxy := func() b2.ShapeProxy {
		switch rng.Intn(3) {
		case 0:
			box := b2.MakeBox(milli(100, 2000), milli(100, 2000))
			return b2.MakeProxy(box.Vertices[:box.Count], b2.QZero())
		case 1:
			points := []b2.Vec2{{X: milli(-1000, 1000), Y: milli(-1000, 1000)}, {X: milli(-1000, 1000), Y: milli(-1000, 1000)}}
			return b2.MakeProxy(points, milli(50, 500))
		default:
			return b2.MakeProxy([]b2.Vec2{{X: milli(-500, 500), Y: milli(-500, 500)}}, milli(50, 1000))
		}
	}
	transform := func() b2.Transform {
		return b2.Transform{
			P: b2.Vec2{X: milli(-5000, 5000), Y: milli(-5000, 5000)},
			Q: b2.MakeRot(milli(0, 999)),
		}
	}
	return b2.DistanceInput{
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
	rng := rand.New(rand.NewSource(1))
	resetSaturationCount()
	h := fnv.New64a()
	var buf [8]byte

	for range 1000 {
		input := randomDistanceInput(rng)
		var cache b2.SimplexCache
		out := b2.ShapeDistance(&input, &cache, nil)

		if out.Iterations > 20 {
			t.Fatalf("iterations %d on %+v", out.Iterations, input)
		}
		if out.Distance.Less(b2.QZero()) {
			t.Fatalf("negative distance %v", out.Distance)
		}
		if b2.QZero().Less(out.Distance) && !b2.IsNormalized(out.Normal) {
			t.Fatalf("normal %v is not unit for distance %v", out.Normal, out.Distance)
		}

		for _, q := range []b2.Q{out.Distance, out.PointA.X, out.PointA.Y, out.PointB.X, out.PointB.Y} {
			binary.LittleEndian.PutUint64(buf[:], qBits(q))
			h.Write(buf[:])
		}
	}

	if n := saturationCount(); n != 0 {
		t.Fatalf("saturation count %d", n)
	}
	if got := h.Sum64(); got != shapeDistanceWitness {
		t.Fatalf("witness %d, want %d", got, shapeDistanceWitness)
	}
}

// TestGetSweepTransformInterpolatesTheCenter checks the two ends and the
// midpoint of a sweep with an offset center of mass.
func TestGetSweepTransformInterpolatesTheCenter(t *testing.T) {
	sweep := b2.Sweep{
		LocalCenter: vec(1, 0),
		C1:          vec(1, 0),
		C2:          vec(5, 0),
		Q1:          b2.RotIdentity(),
		Q2:          b2.RotIdentity(),
	}

	start := b2.GetSweepTransform(&sweep, b2.QZero())
	mid := b2.GetSweepTransform(&sweep, b2.QHalf())
	end := b2.GetSweepTransform(&sweep, b2.QOne())

	if start.P != vec(0, 0) || mid.P != vec(2, 0) || end.P != vec(4, 0) {
		t.Fatalf("origins %v %v %v, want x = 0, 2, 4", start.P, mid.P, end.P)
	}
}

// TestShapeCastMatchesHandCases pins the conservative advancement on
// boxes whose hit fraction is known: the sweep stops once the gap falls
// under the target, so the fraction lands just short of the exact value.
func TestShapeCastMatchesHandCases(t *testing.T) {
	one := b2.QOne()
	a := boxProxy(1, 1, vec(0, 0))
	b := boxProxy(1, 1, vec(0, 0))
	pair := func(positionB, translation b2.Vec2) b2.ShapeCastPairInput {
		return b2.ShapeCastPairInput{
			ProxyA:       a,
			ProxyB:       b,
			TransformA:   b2.TransformIdentity(),
			TransformB:   b2.Transform{P: positionB, Q: b2.RotIdentity()},
			TranslationB: translation,
			MaxFraction:  one,
		}
	}

	t.Run("hit", func(t *testing.T) {
		input := pair(vec(5, 0), vec(-10, 0))
		out := b2.ShapeCast(&input)
		if !out.Hit {
			t.Fatal("the sweep missed")
		}
		// The faces meet at a fraction of 0.3; the sweep stops a target
		// short, within a quarter of a slop.
		exact := b2.QMustParse("0.3")
		if !out.Fraction.Less(exact) || exact.Sub(out.Fraction).Less(b2.QZero()) || !exact.Sub(out.Fraction).Less(b2.QMustParse("0.001")) {
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
		if out := b2.ShapeCast(&input); out.Hit {
			t.Fatalf("a parallel sweep hit at %v", out.Fraction)
		}
	})

	t.Run("out of range", func(t *testing.T) {
		input := pair(vec(5, 0), vec(-1, 0))
		if out := b2.ShapeCast(&input); out.Hit {
			t.Fatalf("a short sweep hit at %v", out.Fraction)
		}
	})

	t.Run("initial overlap", func(t *testing.T) {
		input := pair(vec(1, 0), vec(-10, 0))
		out := b2.ShapeCast(&input)
		if !out.Hit || !out.Fraction.Eq(b2.QZero()) || out.Normal != vec(0, 0) {
			t.Fatalf("overlap reported %+v, want a hit at zero with no normal", out)
		}
	})

	t.Run("encroach", func(t *testing.T) {
		// Two circles whose surfaces overlap by more than two slops may
		// move a little closer: the sweep hits, but not at zero.
		half := b2.QHalf()
		input := b2.ShapeCastPairInput{
			ProxyA:       b2.MakeProxy([]b2.Vec2{vec(0, 0)}, half),
			ProxyB:       b2.MakeProxy([]b2.Vec2{vec(0, 0)}, half),
			TransformA:   b2.TransformIdentity(),
			TransformB:   b2.Transform{P: vecQ("0.9", "0"), Q: b2.RotIdentity()},
			TranslationB: vec(-1, 0),
			MaxFraction:  one,
			CanEncroach:  true,
		}
		out := b2.ShapeCast(&input)
		if !out.Hit || !b2.QZero().Less(out.Fraction) || !out.Fraction.Less(b2.QMustParse("0.01")) {
			t.Fatalf("encroach reported %+v, want a small positive fraction", out)
		}
	})
}

// toiPair builds a time of impact input for two proxies. A rests at the
// origin; B translates from c1 to c2 and turns from q1 to q2.
func toiPair(a, b b2.ShapeProxy, c1, c2 b2.Vec2, q1, q2 b2.Rot) b2.TOIInput {
	return b2.TOIInput{
		ProxyA:      a,
		ProxyB:      b,
		SweepA:      b2.Sweep{Q1: b2.RotIdentity(), Q2: b2.RotIdentity()},
		SweepB:      b2.Sweep{C1: c1, C2: c2, Q1: q1, Q2: q2},
		MaxFraction: b2.QOne(),
	}
}

// toiGap returns the core distance of the proxies at the fraction, without
// the radii, which is the quantity the solver drives to its target.
func toiGap(input *b2.TOIInput, fraction b2.Q) b2.Q {
	distanceInput := b2.DistanceInput{
		ProxyA:     input.ProxyA,
		ProxyB:     input.ProxyB,
		TransformA: b2.GetSweepTransform(&input.SweepA, fraction),
		TransformB: b2.GetSweepTransform(&input.SweepB, fraction),
	}
	var cache b2.SimplexCache
	return b2.ShapeDistance(&distanceInput, &cache, nil).Distance
}

// toiTarget returns the separation the solver seeks and the band around it.
func toiTarget(input *b2.TOIInput) (target, tolerance b2.Q) {
	slop := b2.LinearSlop()
	totalRadius := input.ProxyA.Radius.Add(input.ProxyB.Radius)
	return slop.Max(totalRadius.Sub(slop)), slop.Div(b2.QFromInt(4))
}

// TestTimeOfImpactMatchesHandCases pins the solver on sweeps whose answer
// is known. The hit fraction lands where the gap equals one slop, within a
// quarter of a slop.
func TestTimeOfImpactMatchesHandCases(t *testing.T) {
	identity := b2.RotIdentity()
	a := boxProxy(1, 1, vec(0, 0))
	b := boxProxy(1, 1, vec(0, 0))

	t.Run("hit", func(t *testing.T) {
		input := toiPair(a, b, vec(5, 0), vec(-5, 0), identity, identity)
		out := b2.TimeOfImpact(&input)
		if out.State != b2.TOIStateHit {
			t.Fatalf("state %v, want a hit", out.State)
		}
		// The faces close a gap of 3 at a speed of 10 and stop one slop
		// short: (3 - 0.005) / 10.
		if !near(out.Fraction, b2.QMustParse("0.2995"), b2.QMustParse("0.0002")) {
			t.Fatalf("fraction %v, want about 0.2995", out.Fraction)
		}
	})

	t.Run("separated", func(t *testing.T) {
		input := toiPair(a, b, vec(5, 0), vec(10, 0), identity, identity)
		out := b2.TimeOfImpact(&input)
		if out.State != b2.TOIStateSeparated || !out.Fraction.Eq(b2.QOne()) {
			t.Fatalf("state %v fraction %v, want separated at 1", out.State, out.Fraction)
		}
	})

	t.Run("out of range", func(t *testing.T) {
		input := toiPair(a, b, vec(5, 0), vec(-5, 0), identity, identity)
		input.MaxFraction = b2.QFromRatio(1, 4)
		out := b2.TimeOfImpact(&input)
		if out.State != b2.TOIStateSeparated || !out.Fraction.Eq(input.MaxFraction) {
			t.Fatalf("state %v fraction %v, want separated at the max fraction", out.State, out.Fraction)
		}
	})

	t.Run("overlapped", func(t *testing.T) {
		input := toiPair(a, b, vec(0, 0), vec(5, 0), identity, identity)
		out := b2.TimeOfImpact(&input)
		if out.State != b2.TOIStateOverlapped || !out.Fraction.Eq(b2.QZero()) {
			t.Fatalf("state %v fraction %v, want overlapped at 0", out.State, out.Fraction)
		}
	})

	t.Run("touching", func(t *testing.T) {
		// The gap starts at half a slop, inside the target band.
		input := toiPair(a, b, vecQ("2.0025", "0"), vec(-5, 0), identity, identity)
		out := b2.TimeOfImpact(&input)
		if out.State != b2.TOIStateHit || !out.Fraction.Eq(b2.QZero()) {
			t.Fatalf("state %v fraction %v, want a hit at 0", out.State, out.Fraction)
		}
	})

	t.Run("rotation", func(t *testing.T) {
		// A rod turns a quarter turn about its center and sweeps a small
		// box that sits above it. The translation alone never touches.
		rod := boxProxy(3, 1, vec(0, 0))
		rod.Points[0].Y = b2.QMustParse("-0.1")
		rod.Points[1].Y = b2.QMustParse("-0.1")
		rod.Points[2].Y = b2.QMustParse("0.1")
		rod.Points[3].Y = b2.QMustParse("0.1")
		input := b2.TOIInput{
			ProxyA:      boxProxy(1, 1, vec(0, 0)),
			ProxyB:      rod,
			SweepA:      b2.Sweep{C1: vec(2, 2), C2: vec(2, 2), Q1: identity, Q2: identity},
			SweepB:      b2.Sweep{Q1: identity, Q2: b2.MakeRot(b2.QFromRatio(1, 4))},
			MaxFraction: b2.QOne(),
		}
		out := b2.TimeOfImpact(&input)
		if out.State != b2.TOIStateHit {
			t.Fatalf("state %v, want a hit", out.State)
		}
		if !b2.QZero().Less(out.Fraction) || !out.Fraction.Less(b2.QOne()) {
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
func randomTOIInput(rng *rand.Rand) b2.TOIInput {
	milli := func(lo, hi int) b2.Q { return b2.QFromRatio(lo+rng.Intn(hi-lo+1), 1000) }
	sweep := func() b2.Sweep {
		turn := milli(0, 999)
		return b2.Sweep{
			LocalCenter: b2.Vec2{X: milli(-500, 500), Y: milli(-500, 500)},
			C1:          b2.Vec2{X: milli(-5000, 5000), Y: milli(-5000, 5000)},
			C2:          b2.Vec2{X: milli(-5000, 5000), Y: milli(-5000, 5000)},
			Q1:          b2.MakeRot(turn),
			Q2:          b2.MakeRot(turn.Add(milli(-250, 250))),
		}
	}
	distance := randomDistanceInput(rng)
	return b2.TOIInput{
		ProxyA:      distance.ProxyA,
		ProxyB:      distance.ProxyB,
		SweepA:      sweep(),
		SweepB:      sweep(),
		MaxFraction: b2.QOne(),
	}
}

// TestTimeOfImpactConvergesOnRandomSweeps checks every result state on
// random sweeps against the gap at the reported fraction, pins the bits
// with a witness and confirms that no operation saturated.
func TestTimeOfImpactConvergesOnRandomSweeps(t *testing.T) {
	rng := rand.New(rand.NewSource(11))
	hash := fnv.New64a()
	var buf [8]byte
	resetSaturationCount()
	hits, failed := 0, 0

	for range 1000 {
		input := randomTOIInput(rng)
		out := b2.TimeOfImpact(&input)
		target, tolerance := toiTarget(&input)

		switch out.State {
		case b2.TOIStateHit:
			hits++
			gap := toiGap(&input, out.Fraction)
			if b2.QZero().Less(out.Fraction) && !near(gap, target, tolerance) {
				t.Fatalf("hit at %v with a gap of %v, want %v within %v", out.Fraction, gap, target, tolerance)
			}
			if !b2.QZero().Less(gap) || target.Add(tolerance).Less(gap) {
				t.Fatalf("hit at %v with a gap of %v, want inside (0, %v]", out.Fraction, gap, target.Add(tolerance))
			}
		case b2.TOIStateSeparated:
			if !out.Fraction.Eq(b2.QOne()) {
				t.Fatalf("separated at %v, want 1", out.Fraction)
			}
			if gap := toiGap(&input, out.Fraction); !target.Less(gap) {
				t.Fatalf("separated with a gap of %v, want over %v", gap, target)
			}
		case b2.TOIStateOverlapped:
			if !out.Fraction.Eq(b2.QZero()) {
				t.Fatalf("overlapped at %v, want 0", out.Fraction)
			}
		case b2.TOIStateFailed:
			failed++
		default:
			t.Fatalf("state %v", out.State)
		}

		binary.LittleEndian.PutUint64(buf[:], uint64(out.State))
		hash.Write(buf[:])
		binary.LittleEndian.PutUint64(buf[:], qBits(out.Fraction))
		hash.Write(buf[:])
	}

	if hits < 100 {
		t.Fatalf("only %d hits", hits)
	}
	if failed > 10 {
		t.Fatalf("%d sweeps failed", failed)
	}
	if n := saturationCount(); n != 0 {
		t.Fatalf("%d operations saturated", n)
	}
	if got := hash.Sum64(); got != timeOfImpactWitness {
		t.Fatalf("witness %d, want %d", got, timeOfImpactWitness)
	}
}

// TestIterativeGeometryRejectsInvalidInput covers the assertions of the
// distance solver and the time of impact retained as panics per D-003.
func TestIterativeGeometryRejectsInvalidInput(t *testing.T) {
	expectPanic := func(t *testing.T, f func()) {
		t.Helper()
		defer func() {
			if recover() == nil {
				t.Fatal("no panic")
			}
		}()
		f()
	}

	t.Run("empty proxy", func(t *testing.T) {
		input := identityDistanceInput(boxProxy(1, 1, vec(0, 0)), b2.ShapeProxy{}, false)
		var cache b2.SimplexCache
		expectPanic(t, func() { b2.ShapeDistance(&input, &cache, nil) })
	})

	t.Run("non-unit rotation", func(t *testing.T) {
		input := toiPair(boxProxy(1, 1, vec(0, 0)), boxProxy(1, 1, vec(0, 0)), vec(5, 0), vec(-5, 0), b2.RotIdentity(), b2.Rot{})
		expectPanic(t, func() { b2.TimeOfImpact(&input) })
	})
}

// TestShapeCastNeverPanicsOnAGrid sweeps a rounded box against a fixed
// box from a grid of starts in four directions. The cast must return a
// fraction in [0, 1] and must not panic on any start.
func TestShapeCastNeverPanicsOnAGrid(t *testing.T) {
	one := b2.QOne()
	rounded := b2.MakeRoundedBox(one, one, b2.QFromRatio(1, 10))
	fixedBox := b2.MakeBox(one, one)
	proxyA := b2.MakeProxy(fixedBox.Vertices[:fixedBox.Count], fixedBox.Radius)
	proxyB := b2.MakeProxy(rounded.Vertices[:rounded.Count], rounded.Radius)

	directions := []b2.Vec2{vec(10, 0), vec(-10, 0), vec(0, 10), vec(0, -10)}
	step := b2.QFromRatio(3, 10)
	for i := range 21 {
		for j := range 21 {
			start := b2.Vec2{
				X: b2.QFromInt(-3).Add(step.Mul(b2.QFromInt(i))),
				Y: b2.QFromInt(-3).Add(step.Mul(b2.QFromInt(j))),
			}
			for _, direction := range directions {
				input := b2.ShapeCastPairInput{
					ProxyA:       proxyA,
					ProxyB:       proxyB,
					TransformA:   b2.TransformIdentity(),
					TransformB:   b2.Transform{P: start, Q: b2.RotIdentity()},
					TranslationB: direction,
					MaxFraction:  one,
				}
				out := b2.ShapeCast(&input)
				if out.Fraction.Less(b2.QZero()) || one.Less(out.Fraction) {
					t.Fatalf("start %v direction %v: fraction %v", start, direction, out.Fraction)
				}
			}
		}
	}
}
