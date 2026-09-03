package dbox2d

import "github.com/dhannyell/fixed"

// SegmentDistance computes the closest points of two line segments,
// clamping at the end points when needed. It follows Ericson 5.1.9 and
// corresponds to b2SegmentDistance in src/distance.c.
func SegmentDistance(p1, q1, p2, q2 Vec2) SegmentDistanceResult {
	var result SegmentDistanceResult

	zero := fixed.Q32Zero()
	one := fixed.Q32One()

	d1 := q1.Sub(p1)
	d2 := q2.Sub(p2)
	r := p1.Sub(p2)
	dd1 := d1.Dot(d1)
	dd2 := d2.Dot(d2)
	rd1 := r.Dot(d1)
	rd2 := r.Dot(d2)

	// The reference compares dd against FLT_EPSILON squared. Q has no
	// rounding noise, so an exact zero selects the branch. See D-012.
	if dd1.Eq(zero) || dd2.Eq(zero) {
		// Handle all degeneracies.
		if !dd1.Eq(zero) {
			// Segment 2 is degenerate.
			result.Fraction1 = rd1.Neg().Div(dd1).Clamp(zero, one)
			result.Fraction2 = zero
		} else if !dd2.Eq(zero) {
			// Segment 1 is degenerate.
			result.Fraction1 = zero
			result.Fraction2 = rd2.Div(dd2).Clamp(zero, one)
		} else {
			result.Fraction1 = zero
			result.Fraction2 = zero
		}
	} else {
		// Non-degenerate segments.
		d12 := d1.Dot(d2)

		denominator := dd1.Mul(dd2).Sub(d12.Mul(d12))

		// Fraction on segment 1.
		f1 := zero
		if !denominator.Eq(zero) {
			// Not parallel.
			f1 = d12.Mul(rd2).Sub(rd1.Mul(dd2)).Div(denominator).Clamp(zero, one)
		}

		// Compute the point on segment 2 closest to p1 + f1 * d1.
		f2 := d12.Mul(f1).Add(rd2).Div(dd2)

		// Clamping of segment 2 requires a do over on segment 1.
		if f2.Less(zero) {
			f2 = zero
			f1 = rd1.Neg().Div(dd1).Clamp(zero, one)
		} else if one.Less(f2) {
			f2 = one
			f1 = d12.Sub(rd1).Div(dd1).Clamp(zero, one)
		}

		result.Fraction1 = f1
		result.Fraction2 = f2
	}

	result.Closest1 = MulAdd(p1, result.Fraction1, d1)
	result.Closest2 = MulAdd(p2, result.Fraction2, d2)
	result.DistanceSquared = result.Closest1.DistanceSq(result.Closest2)
	return result
}

// MakeProxy makes a proxy for the overlap and distance routines. The
// slice replaces the pointer and count pair of b2MakeProxy in
// src/distance.c; extra points beyond the polygon limit do not enter.
func MakeProxy(points []Vec2, radius Q) ShapeProxy {
	count := min(len(points), MaxPolygonVertices)
	proxy := ShapeProxy{Count: count, Radius: radius}
	copy(proxy.Points[:], points[:count])
	return proxy
}

// GetSweepTransform evaluates the sweep at a time in [0, 1]. It
// corresponds to b2GetSweepTransform in src/distance.c.
func GetSweepTransform(sweep *Sweep, time Q) Transform {
	// https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/
	omt := fixed.Q32One().Sub(time)

	var xf Transform
	xf.P = sweep.C1.Mul(omt).Add(sweep.C2.Mul(time))

	q := Rot{
		Cos: omt.Mul(sweep.Q1.Cos).Add(time.Mul(sweep.Q2.Cos)),
		Sin: omt.Mul(sweep.Q1.Sin).Add(time.Mul(sweep.Q2.Sin)),
	}

	xf.Q = NormalizeRot(q)

	// Shift to origin
	xf.P = xf.P.Sub(RotateVector(xf.Q, sweep.LocalCenter))
	return xf
}

// MakeOffsetProxy makes a proxy with the points transformed by a position
// and a rotation. The slice replaces the pointer and count pair of
// b2MakeOffsetProxy in src/distance.c.
func MakeOffsetProxy(points []Vec2, radius Q, position Vec2, rotation Rot) ShapeProxy {
	count := min(len(points), MaxPolygonVertices)
	transform := Transform{P: position, Q: rotation}
	proxy := ShapeProxy{Count: count, Radius: radius}
	for i := range count {
		proxy.Points[i] = TransformPoint(transform, points[i])
	}
	return proxy
}

// weight2 returns a1 * w1 + a2 * w2. It corresponds to b2Weight2 in
// src/distance.c.
func weight2(a1 Q, w1 Vec2, a2 Q, w2 Vec2) Vec2 {
	return Vec2{
		X: a1.Mul(w1.X).Add(a2.Mul(w2.X)),
		Y: a1.Mul(w1.Y).Add(a2.Mul(w2.Y)),
	}
}

// weight3 returns a1 * w1 + a2 * w2 + a3 * w3. It corresponds to
// b2Weight3 in src/distance.c.
func weight3(a1 Q, w1 Vec2, a2 Q, w2 Vec2, a3 Q, w3 Vec2) Vec2 {
	return Vec2{
		X: a1.Mul(w1.X).Add(a2.Mul(w2.X)).Add(a3.Mul(w3.X)),
		Y: a1.Mul(w1.Y).Add(a2.Mul(w2.Y)).Add(a3.Mul(w3.Y)),
	}
}

// findSupport returns the index of the proxy point farthest along the
// direction. It corresponds to b2FindSupport in src/distance.c.
func findSupport(proxy *ShapeProxy, direction Vec2) int {
	bestIndex := 0
	bestValue := proxy.Points[0].Dot(direction)
	for i := 1; i < proxy.Count; i++ {
		value := proxy.Points[i].Dot(direction)
		if bestValue.Less(value) {
			bestIndex = i
			bestValue = value
		}
	}

	return bestIndex
}

// makeSimplexFromCache rebuilds a simplex from the cached indices. It
// corresponds to b2MakeSimplexFromCache in src/distance.c.
func makeSimplexFromCache(cache *SimplexCache, proxyA, proxyB *ShapeProxy) Simplex {
	if cache.Count > 3 {
		panic("dbox2d: the simplex cache holds more than three vertices")
	}

	var s Simplex

	// Copy data from cache.
	s.Count = int(cache.Count)

	vertices := [3]*SimplexVertex{&s.V1, &s.V2, &s.V3}
	for i := range s.Count {
		v := vertices[i]
		v.IndexA = int(cache.IndexA[i])
		v.IndexB = int(cache.IndexB[i])
		v.WA = proxyA.Points[v.IndexA]
		v.WB = proxyB.Points[v.IndexB]
		v.W = v.WA.Sub(v.WB)

		// invalid
		v.A = fixed.Q32One().Neg()
	}

	// If the cache is empty or invalid ...
	if s.Count == 0 {
		v := vertices[0]
		v.IndexA = 0
		v.IndexB = 0
		v.WA = proxyA.Points[0]
		v.WB = proxyB.Points[0]
		v.W = v.WA.Sub(v.WB)
		v.A = fixed.Q32One()
		s.Count = 1
	}

	return s
}

// makeSimplexCache stores the simplex indices for the next call. It
// corresponds to b2MakeSimplexCache in src/distance.c.
func makeSimplexCache(cache *SimplexCache, simplex *Simplex) {
	cache.Count = uint16(simplex.Count)
	vertices := [3]*SimplexVertex{&simplex.V1, &simplex.V2, &simplex.V3}
	for i := range simplex.Count {
		cache.IndexA[i] = uint8(vertices[i].IndexA)
		cache.IndexB[i] = uint8(vertices[i].IndexB)
	}
}

// computeSimplexWitnessPoints returns the closest points on A and on B for
// the simplex. It corresponds to b2ComputeSimplexWitnessPoints in
// src/distance.c.
func computeSimplexWitnessPoints(s *Simplex) (a, b Vec2) {
	switch s.Count {
	case 1:
		a = s.V1.WA
		b = s.V1.WB

	case 2:
		a = weight2(s.V1.A, s.V1.WA, s.V2.A, s.V2.WA)
		b = weight2(s.V1.A, s.V1.WB, s.V2.A, s.V2.WB)

	case 3:
		a = weight3(s.V1.A, s.V1.WA, s.V2.A, s.V2.WA, s.V3.A, s.V3.WA)
		// todo why are these not equal?
		// b = weight3(s.V1.A, s.V1.WB, s.V2.A, s.V2.WB, s.V3.A, s.V3.WB)
		b = a

	default:
		panic("dbox2d: the simplex count is out of range")
	}

	return a, b
}

// solveSimplex2 solves a line segment with barycentric coordinates.
//
// p = a1 * w1 + a2 * w2
// a1 + a2 = 1
//
// The vector from the origin to the closest point on the line is
// perpendicular to the line.
// e12 = w2 - w1
// dot(p, e) = 0
// a1 * dot(w1, e) + a2 * dot(w2, e) = 0
//
// 2-by-2 linear system
// [1      1     ][a1] = [1]
// [w1.e12 w2.e12][a2] = [0]
//
// Define
// d12_1 =  dot(w2, e12)
// d12_2 = -dot(w1, e12)
// d12 = d12_1 + d12_2
//
// Solution
// a1 = d12_1 / d12
// a2 = d12_2 / d12
//
// It returns a vector that points towards the origin. It corresponds to
// b2SolveSimplex2 in src/distance.c; the reciprocal of d12 becomes two
// divisions (D-006).
func solveSimplex2(s *Simplex) Vec2 {
	zero := fixed.Q32Zero()
	one := fixed.Q32One()

	w1 := s.V1.W
	w2 := s.V2.W
	e12 := w2.Sub(w1)

	// w1 region
	d12_2 := w1.Dot(e12).Neg()
	if !zero.Less(d12_2) {
		// a2 <= 0, so we clamp it to 0
		s.V1.A = one
		s.Count = 1
		return Neg(w1)
	}

	// w2 region
	d12_1 := w2.Dot(e12)
	if !zero.Less(d12_1) {
		// a1 <= 0, so we clamp it to 0
		s.V2.A = one
		s.Count = 1
		s.V1 = s.V2
		return Neg(w2)
	}

	// Must be in e12 region.
	d12 := d12_1.Add(d12_2)
	s.V1.A = d12_1.Div(d12)
	s.V2.A = d12_2.Div(d12)
	s.Count = 2
	return CrossSV(Cross(w1.Add(w2), e12), e12)
}

// solveSimplex3 solves a triangle with barycentric coordinates. It
// corresponds to b2SolveSimplex3 in src/distance.c; each reciprocal
// becomes divisions (D-006).
func solveSimplex3(s *Simplex) Vec2 {
	zero := fixed.Q32Zero()
	one := fixed.Q32One()

	w1 := s.V1.W
	w2 := s.V2.W
	w3 := s.V3.W

	// Edge12
	// [1      1     ][a1] = [1]
	// [w1.e12 w2.e12][a2] = [0]
	// a3 = 0
	e12 := w2.Sub(w1)
	w1e12 := w1.Dot(e12)
	w2e12 := w2.Dot(e12)
	d12_1 := w2e12
	d12_2 := w1e12.Neg()

	// Edge13
	// [1      1     ][a1] = [1]
	// [w1.e13 w3.e13][a3] = [0]
	// a2 = 0
	e13 := w3.Sub(w1)
	w1e13 := w1.Dot(e13)
	w3e13 := w3.Dot(e13)
	d13_1 := w3e13
	d13_2 := w1e13.Neg()

	// Edge23
	// [1      1     ][a2] = [1]
	// [w2.e23 w3.e23][a3] = [0]
	// a1 = 0
	e23 := w3.Sub(w2)
	w2e23 := w2.Dot(e23)
	w3e23 := w3.Dot(e23)
	d23_1 := w3e23
	d23_2 := w2e23.Neg()

	// Triangle123
	n123 := Cross(e12, e13)

	d123_1 := n123.Mul(Cross(w2, w3))
	d123_2 := n123.Mul(Cross(w3, w1))
	d123_3 := n123.Mul(Cross(w1, w2))

	// w1 region
	if !zero.Less(d12_2) && !zero.Less(d13_2) {
		s.V1.A = one
		s.Count = 1
		return Neg(w1)
	}

	// e12
	if zero.Less(d12_1) && zero.Less(d12_2) && !zero.Less(d123_3) {
		d12 := d12_1.Add(d12_2)
		s.V1.A = d12_1.Div(d12)
		s.V2.A = d12_2.Div(d12)
		s.Count = 2
		return CrossSV(Cross(w1.Add(w2), e12), e12)
	}

	// e13
	if zero.Less(d13_1) && zero.Less(d13_2) && !zero.Less(d123_2) {
		d13 := d13_1.Add(d13_2)
		s.V1.A = d13_1.Div(d13)
		s.V3.A = d13_2.Div(d13)
		s.Count = 2
		s.V2 = s.V3
		return CrossSV(Cross(w1.Add(w3), e13), e13)
	}

	// w2 region
	if !zero.Less(d12_1) && !zero.Less(d23_2) {
		s.V2.A = one
		s.Count = 1
		s.V1 = s.V2
		return Neg(w2)
	}

	// w3 region
	if !zero.Less(d13_1) && !zero.Less(d23_1) {
		s.V3.A = one
		s.Count = 1
		s.V1 = s.V3
		return Neg(w3)
	}

	// e23
	if zero.Less(d23_1) && zero.Less(d23_2) && !zero.Less(d123_1) {
		d23 := d23_1.Add(d23_2)
		s.V2.A = d23_1.Div(d23)
		s.V3.A = d23_2.Div(d23)
		s.Count = 2
		s.V1 = s.V3
		return CrossSV(Cross(w2.Add(w3), e23), e23)
	}

	// Must be in triangle123
	d123 := d123_1.Add(d123_2).Add(d123_3)
	s.V1.A = d123_1.Div(d123)
	s.V2.A = d123_2.Div(d123)
	s.V3.A = d123_3.Div(d123)
	s.Count = 3

	// No search direction
	return Vec2Zero()
}

// ShapeDistance computes the closest points between two convex proxies
// with GJK. The cache is input and output; start with the zero value.
// The simplexes slice receives the simplex of each iteration for
// debugging; pass nil to skip it. It corresponds to b2ShapeDistance in
// src/distance.c, https://box2d.org/files/ErinCatto_GJK_GDC2010.pdf.
//
// The reference stops on a search direction shorter than FLT_EPSILON;
// the port stops on a squared length of exactly zero (D-012).
func ShapeDistance(input *DistanceInput, cache *SimplexCache, simplexes []Simplex) DistanceOutput {
	zero := fixed.Q32Zero()

	if input.ProxyA.Count <= 0 || input.ProxyB.Count <= 0 {
		panic("dbox2d: ShapeDistance needs at least one point per proxy")
	}
	if input.ProxyA.Radius.Less(zero) || input.ProxyB.Radius.Less(zero) {
		panic("dbox2d: ShapeDistance needs non-negative radii")
	}

	var output DistanceOutput

	proxyA := &input.ProxyA

	// Get proxyB in frame A to avoid further transforms in the main loop.
	// This is still a performance gain at 8 points.
	var localProxyB ShapeProxy
	{
		transform := InvMulTransforms(input.TransformA, input.TransformB)
		localProxyB.Count = input.ProxyB.Count
		localProxyB.Radius = input.ProxyB.Radius
		for i := range localProxyB.Count {
			localProxyB.Points[i] = TransformPoint(transform, input.ProxyB.Points[i])
		}
	}

	// Initialize the simplex.
	simplex := makeSimplexFromCache(cache, proxyA, &localProxyB)

	simplexIndex := 0
	if simplexIndex < len(simplexes) {
		simplexes[simplexIndex] = simplex
		simplexIndex++
	}

	// Get simplex vertices as an array.
	vertices := [3]*SimplexVertex{&simplex.V1, &simplex.V2, &simplex.V3}

	nonUnitNormal := Vec2Zero()

	// These store the vertices of the last simplex so that we can check for duplicates and prevent cycling.
	var saveA, saveB [3]int

	// Main iteration loop. All computations are done in frame A.
	const maxIterations = 20
	iteration := 0
	for iteration < maxIterations {
		// Copy simplex so we can identify duplicates.
		saveCount := simplex.Count
		for i := range saveCount {
			saveA[i] = vertices[i].IndexA
			saveB[i] = vertices[i].IndexB
		}

		var d Vec2
		switch simplex.Count {
		case 1:
			d = Neg(simplex.V1.W)

		case 2:
			d = solveSimplex2(&simplex)

		case 3:
			d = solveSimplex3(&simplex)

		default:
			panic("dbox2d: the simplex count is out of range")
		}

		// If we have 3 points, then the origin is in the corresponding triangle.
		if simplex.Count == 3 {
			// Overlap
			localPointA, localPointB := computeSimplexWitnessPoints(&simplex)
			output.PointA = TransformPoint(input.TransformA, localPointA)
			output.PointB = TransformPoint(input.TransformA, localPointB)
			return output
		}

		if simplexIndex < len(simplexes) {
			simplexes[simplexIndex] = simplex
			simplexIndex++
		}

		// Ensure the search direction is numerically fit.
		if d.Dot(d).Eq(zero) {
			// This is unlikely but could lead to bad cycling.
			// The branch predictor seems to make this check have low cost.

			// The origin is probably contained by a line segment
			// or triangle. Thus the shapes are overlapped.

			// Must return overlap due to invalid normal.
			localPointA, localPointB := computeSimplexWitnessPoints(&simplex)
			output.PointA = TransformPoint(input.TransformA, localPointA)
			output.PointB = TransformPoint(input.TransformA, localPointB)
			return output
		}

		// Save the normal
		nonUnitNormal = d

		// Compute a tentative new simplex vertex using support points.
		// support = support(a, d) - support(b, -d)
		vertex := vertices[simplex.Count]
		vertex.IndexA = findSupport(proxyA, d)
		vertex.WA = proxyA.Points[vertex.IndexA]
		vertex.IndexB = findSupport(&localProxyB, Neg(d))
		vertex.WB = localProxyB.Points[vertex.IndexB]
		vertex.W = vertex.WA.Sub(vertex.WB)

		// Iteration count is equated to the number of support point calls.
		iteration++

		// Check for duplicate support points. This is the main termination criteria.
		duplicate := false
		for i := range saveCount {
			if vertex.IndexA == saveA[i] && vertex.IndexB == saveB[i] {
				duplicate = true
				break
			}
		}

		// If we found a duplicate support point we must exit to avoid cycling.
		if duplicate {
			break
		}

		// New vertex is valid and needed.
		simplex.Count++
	}

	if simplexIndex < len(simplexes) {
		simplexes[simplexIndex] = simplex
		simplexIndex++
	}

	// Prepare output
	normal := nonUnitNormal.Normalize()
	if !IsNormalized(normal) {
		panic("dbox2d: the GJK normal did not normalize")
	}
	normal = RotateVector(input.TransformA.Q, normal)

	localPointA, localPointB := computeSimplexWitnessPoints(&simplex)
	output.Normal = normal
	output.Distance = localPointA.Distance(localPointB)
	output.PointA = TransformPoint(input.TransformA, localPointA)
	output.PointB = TransformPoint(input.TransformA, localPointB)
	output.Iterations = iteration
	output.SimplexCount = simplexIndex

	// Cache the simplex
	makeSimplexCache(cache, &simplex)

	// Apply radii if requested
	if input.UseRadii && linearSlop.Div(fixed.Q32FromInt(10)).Less(output.Distance) {
		radiusA := input.ProxyA.Radius
		radiusB := input.ProxyB.Radius
		output.Distance = zero.Max(output.Distance.Sub(radiusA).Sub(radiusB))

		// Keep closest points on perimeter even if overlapped, this way the points move smoothly.
		output.PointA = MulAdd(output.PointA, radiusA, normal)
		output.PointB = MulSub(output.PointB, radiusB, normal)
	}

	return output
}

// ShapeCast casts shape B against a fixed shape A by conservative
// advancement and reports the hit point, the normal and the fraction of
// the translation. Shapes that start in contact report a hit at a zero
// fraction unless the input allows an encroach. It corresponds to
// b2ShapeCast in src/distance.c.
func ShapeCast(input *ShapeCastPairInput) CastOutput {
	zero := fixed.Q32Zero()

	// Compute tolerance
	totalRadius := input.ProxyA.Radius.Add(input.ProxyB.Radius)
	target := linearSlop.Max(totalRadius.Sub(linearSlop))
	tolerance := linearSlop.Div(fixed.Q32FromInt(4))

	if !tolerance.Less(target) {
		panic("dbox2d: the shape cast target is inside the tolerance")
	}

	// Prepare input for distance query
	var cache SimplexCache

	fraction := zero

	var distanceInput DistanceInput
	distanceInput.ProxyA = input.ProxyA
	distanceInput.ProxyB = input.ProxyB
	distanceInput.TransformA = input.TransformA
	distanceInput.TransformB = input.TransformB
	distanceInput.UseRadii = false

	delta2 := input.TranslationB
	var output CastOutput

	const maxIterations = 20
	for iteration := 0; iteration < maxIterations; iteration++ {
		output.Iterations++

		distanceOutput := ShapeDistance(&distanceInput, &cache, nil)

		if distanceOutput.Distance.Less(target.Add(tolerance)) {
			if iteration == 0 {
				if input.CanEncroach && linearSlop.Mul(fixed.Q32FromInt(2)).Less(distanceOutput.Distance) {
					target = distanceOutput.Distance.Sub(linearSlop)
				} else {
					// Initial overlap
					output.Hit = true

					// Compute a common point
					c1 := MulAdd(distanceOutput.PointA, input.ProxyA.Radius, distanceOutput.Normal)
					c2 := MulAdd(distanceOutput.PointB, input.ProxyB.Radius.Neg(), distanceOutput.Normal)
					output.Point = Lerp(c1, c2, fixed.Q32Half())
					return output
				}
			} else {
				// Regular hit
				if !zero.Less(distanceOutput.Distance) || !IsNormalized(distanceOutput.Normal) {
					panic("dbox2d: the shape cast hit has no valid normal")
				}
				output.Fraction = fraction
				output.Point = MulAdd(distanceOutput.PointA, input.ProxyA.Radius, distanceOutput.Normal)
				output.Normal = distanceOutput.Normal
				output.Hit = true
				return output
			}
		}

		if !zero.Less(distanceOutput.Distance) {
			panic("dbox2d: the shape cast advanced into an overlap")
		}
		if !IsNormalized(distanceOutput.Normal) {
			panic("dbox2d: the shape cast normal is not unit")
		}

		// Check if shapes are approaching each other
		denominator := delta2.Dot(distanceOutput.Normal)
		if !denominator.Less(zero) {
			// Miss
			return output
		}

		// Advance sweep
		fraction = fraction.Add(target.Sub(distanceOutput.Distance).Div(denominator))
		if !fraction.Less(input.MaxFraction) {
			// Miss
			return output
		}

		distanceInput.TransformB.P = MulAdd(input.TransformB.P, fraction, delta2)
	}

	// Failure!
	return output
}

// separationType selects the axis a separationFunction tracks. It
// corresponds to b2SeparationType in src/distance.c.
type separationType int

const (
	pointsType separationType = iota
	faceAType
	faceBType
)

// separationFunction evaluates the separation of two proxies along one
// axis over their sweeps. It corresponds to b2SeparationFunction in
// src/distance.c.
type separationFunction struct {
	proxyA     *ShapeProxy
	proxyB     *ShapeProxy
	sweepA     Sweep
	sweepB     Sweep
	localPoint Vec2
	axis       Vec2
	kind       separationType
}

// makeSeparationFunction builds the separating axis from the simplex cache
// at the time t1. It corresponds to b2MakeSeparationFunction in
// src/distance.c.
func makeSeparationFunction(cache *SimplexCache, proxyA *ShapeProxy, sweepA *Sweep, proxyB *ShapeProxy, sweepB *Sweep, t1 Q) separationFunction {
	var f separationFunction

	f.proxyA = proxyA
	f.proxyB = proxyB
	count := int(cache.Count)
	if count <= 0 || count >= 3 {
		panic("dbox2d: the separation function needs one or two cached vertices")
	}

	f.sweepA = *sweepA
	f.sweepB = *sweepB

	xfA := GetSweepTransform(sweepA, t1)
	xfB := GetSweepTransform(sweepB, t1)

	if count == 1 {
		f.kind = pointsType
		localPointA := proxyA.Points[cache.IndexA[0]]
		localPointB := proxyB.Points[cache.IndexB[0]]
		pointA := TransformPoint(xfA, localPointA)
		pointB := TransformPoint(xfB, localPointB)
		f.axis = pointB.Sub(pointA).Normalize()
		f.localPoint = Vec2Zero()
		return f
	}

	if cache.IndexA[0] == cache.IndexA[1] {
		// Two points on B and one on A.
		f.kind = faceBType
		localPointB1 := proxyB.Points[cache.IndexB[0]]
		localPointB2 := proxyB.Points[cache.IndexB[1]]

		f.axis = CrossVS(localPointB2.Sub(localPointB1), fixed.Q32One())
		f.axis = f.axis.Normalize()
		normal := RotateVector(xfB.Q, f.axis)

		f.localPoint = localPointB1.Add(localPointB2).Mul(fixed.Q32Half())
		pointB := TransformPoint(xfB, f.localPoint)

		localPointA := proxyA.Points[cache.IndexA[0]]
		pointA := TransformPoint(xfA, localPointA)

		s := pointA.Sub(pointB).Dot(normal)
		if s.Less(fixed.Q32Zero()) {
			f.axis = Neg(f.axis)
		}
		return f
	}

	// Two points on A and one or two points on B.
	f.kind = faceAType
	localPointA1 := proxyA.Points[cache.IndexA[0]]
	localPointA2 := proxyA.Points[cache.IndexA[1]]

	f.axis = CrossVS(localPointA2.Sub(localPointA1), fixed.Q32One())
	f.axis = f.axis.Normalize()
	normal := RotateVector(xfA.Q, f.axis)

	f.localPoint = localPointA1.Add(localPointA2).Mul(fixed.Q32Half())
	pointA := TransformPoint(xfA, f.localPoint)

	localPointB := proxyB.Points[cache.IndexB[0]]
	pointB := TransformPoint(xfB, localPointB)

	s := pointB.Sub(pointA).Dot(normal)
	if s.Less(fixed.Q32Zero()) {
		f.axis = Neg(f.axis)
	}
	return f
}

// findMinSeparation returns the deepest separation along the axis at the
// time t and the witness indices. The return values replace the out
// parameters of b2FindMinSeparation in src/distance.c.
func findMinSeparation(f *separationFunction, t Q) (separation Q, indexA, indexB int) {
	xfA := GetSweepTransform(&f.sweepA, t)
	xfB := GetSweepTransform(&f.sweepB, t)

	switch f.kind {
	case pointsType:
		axisA := InvRotateVector(xfA.Q, f.axis)
		axisB := InvRotateVector(xfB.Q, Neg(f.axis))

		indexA = findSupport(f.proxyA, axisA)
		indexB = findSupport(f.proxyB, axisB)

		localPointA := f.proxyA.Points[indexA]
		localPointB := f.proxyB.Points[indexB]

		pointA := TransformPoint(xfA, localPointA)
		pointB := TransformPoint(xfB, localPointB)

		separation = pointB.Sub(pointA).Dot(f.axis)
		return separation, indexA, indexB

	case faceAType:
		normal := RotateVector(xfA.Q, f.axis)
		pointA := TransformPoint(xfA, f.localPoint)

		axisB := InvRotateVector(xfB.Q, Neg(normal))

		indexA = -1
		indexB = findSupport(f.proxyB, axisB)

		localPointB := f.proxyB.Points[indexB]
		pointB := TransformPoint(xfB, localPointB)

		separation = pointB.Sub(pointA).Dot(normal)
		return separation, indexA, indexB

	case faceBType:
		normal := RotateVector(xfB.Q, f.axis)
		pointB := TransformPoint(xfB, f.localPoint)

		axisA := InvRotateVector(xfA.Q, Neg(normal))

		indexB = -1
		indexA = findSupport(f.proxyA, axisA)

		localPointA := f.proxyA.Points[indexA]
		pointA := TransformPoint(xfA, localPointA)

		separation = pointA.Sub(pointB).Dot(normal)
		return separation, indexA, indexB

	default:
		panic("dbox2d: the separation type is out of range")
	}
}

// evaluateSeparation returns the separation of the witness points along
// the axis at the time t. It corresponds to b2EvaluateSeparation in
// src/distance.c.
func evaluateSeparation(f *separationFunction, indexA, indexB int, t Q) Q {
	xfA := GetSweepTransform(&f.sweepA, t)
	xfB := GetSweepTransform(&f.sweepB, t)

	switch f.kind {
	case pointsType:
		localPointA := f.proxyA.Points[indexA]
		localPointB := f.proxyB.Points[indexB]

		pointA := TransformPoint(xfA, localPointA)
		pointB := TransformPoint(xfB, localPointB)

		return pointB.Sub(pointA).Dot(f.axis)

	case faceAType:
		normal := RotateVector(xfA.Q, f.axis)
		pointA := TransformPoint(xfA, f.localPoint)

		localPointB := f.proxyB.Points[indexB]
		pointB := TransformPoint(xfB, localPointB)

		return pointB.Sub(pointA).Dot(normal)

	case faceBType:
		normal := RotateVector(xfB.Q, f.axis)
		pointB := TransformPoint(xfB, f.localPoint)

		localPointA := f.proxyA.Points[indexA]
		pointA := TransformPoint(xfA, localPointA)

		return pointA.Sub(pointB).Dot(normal)

	default:
		panic("dbox2d: the separation type is out of range")
	}
}

// TimeOfImpact computes the largest fraction of the sweeps at which the
// two proxies stay separated by the target. It uses the local separating
// axis method, so it may miss some intermediate, non-tunneling
// collisions. The reference counters under B2_SNOOP_TOI_COUNTERS are not
// ported. It corresponds to b2TimeOfImpact in src/distance.c.
func TimeOfImpact(input *TOIInput) TOIOutput {
	zero := fixed.Q32Zero()

	var output TOIOutput
	output.State = TOIStateUnknown
	output.Fraction = input.MaxFraction

	sweepA := input.SweepA
	sweepB := input.SweepB
	if !IsNormalizedRot(sweepA.Q1) || !IsNormalizedRot(sweepA.Q2) || !IsNormalizedRot(sweepB.Q1) || !IsNormalizedRot(sweepB.Q2) {
		panic("dbox2d: TimeOfImpact needs unit rotations")
	}

	proxyA := &input.ProxyA
	proxyB := &input.ProxyB

	tMax := input.MaxFraction

	totalRadius := proxyA.Radius.Add(proxyB.Radius)
	target := linearSlop.Max(totalRadius.Sub(linearSlop))
	tolerance := linearSlop.Div(fixed.Q32FromInt(4))
	if !tolerance.Less(target) {
		panic("dbox2d: the time of impact target is inside the tolerance")
	}

	t1 := zero
	const maxIterations = 20
	distanceIterations := 0

	// Prepare input for distance query.
	var cache SimplexCache
	var distanceInput DistanceInput
	distanceInput.ProxyA = input.ProxyA
	distanceInput.ProxyB = input.ProxyB
	distanceInput.UseRadii = false

	// The outer loop progressively attempts to compute new separating axes.
	// This loop terminates when an axis is repeated (no progress is made).
	for {
		xfA := GetSweepTransform(&sweepA, t1)
		xfB := GetSweepTransform(&sweepB, t1)

		// Get the distance between shapes. We can also use the results
		// to get a separating axis.
		distanceInput.TransformA = xfA
		distanceInput.TransformB = xfB
		distanceOutput := ShapeDistance(&distanceInput, &cache, nil)

		distanceIterations++

		// If the shapes are overlapped, we give up on continuous collision.
		if !zero.Less(distanceOutput.Distance) {
			// Failure!
			output.State = TOIStateOverlapped
			output.Fraction = zero
			break
		}

		if !target.Add(tolerance).Less(distanceOutput.Distance) {
			// Victory!
			output.State = TOIStateHit
			output.Fraction = t1
			break
		}

		// Initialize the separating axis.
		fcn := makeSeparationFunction(&cache, proxyA, &sweepA, proxyB, &sweepB, t1)

		// Compute the TOI on the separating axis. We do this by successively
		// resolving the deepest point. This loop is bounded by the number of vertices.
		done := false
		t2 := tMax
		pushBackIterations := 0
		for {
			// Find the deepest point at t2. Store the witness point indices.
			s2, indexA, indexB := findMinSeparation(&fcn, t2)

			// Is the final configuration separated?
			if target.Add(tolerance).Less(s2) {
				// Victory!
				output.State = TOIStateSeparated
				output.Fraction = tMax
				done = true
				break
			}

			// Has the separation reached tolerance?
			if target.Sub(tolerance).Less(s2) {
				// Advance the sweeps
				t1 = t2
				break
			}

			// Compute the initial separation of the witness points.
			s1 := evaluateSeparation(&fcn, indexA, indexB, t1)

			// Check for initial overlap. This might happen if the root finder
			// runs out of iterations.
			if s1.Less(target.Sub(tolerance)) {
				output.State = TOIStateFailed
				output.Fraction = t1
				done = true
				break
			}

			// Check for touching
			if !target.Add(tolerance).Less(s1) {
				// Victory! t1 should hold the TOI (could be 0.0).
				output.State = TOIStateHit
				output.Fraction = t1
				done = true
				break
			}

			// Compute 1D root of: f(x) - target = 0
			rootIterationCount := 0
			a1, a2 := t1, t2
			for {
				// Use a mix of the secant rule and bisection.
				var t Q
				if rootIterationCount&1 == 1 {
					// Secant rule to improve convergence.
					t = a1.Add(target.Sub(s1).Mul(a2.Sub(a1)).Div(s2.Sub(s1)))
				} else {
					// Bisection to guarantee progress.
					t = fixed.Q32Half().Mul(a1.Add(a2))
				}

				rootIterationCount++

				s := evaluateSeparation(&fcn, indexA, indexB, t)

				if s.Sub(target).Abs().Less(tolerance) {
					// t2 holds a tentative value for t1
					t2 = t
					break
				}

				// Ensure we continue to bracket the root.
				if target.Less(s) {
					a1 = t
					s1 = s
				} else {
					a2 = t
					s2 = s
				}

				if rootIterationCount == 50 {
					break
				}
			}

			pushBackIterations++

			if pushBackIterations == MaxPolygonVertices {
				break
			}
		}

		if done {
			break
		}

		if distanceIterations == maxIterations {
			// Root finder got stuck. Semi-victory.
			output.State = TOIStateFailed
			output.Fraction = t1
			break
		}
	}

	return output
}
