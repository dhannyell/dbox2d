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
