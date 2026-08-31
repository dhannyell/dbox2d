package dbox2d

import "github.com/dhannyell/fixed"

// SegmentDistance computes the closest points of two line segments,
// clamping at the end points when needed. It follows Ericson 5.1.9 and
// corresponds to b2SegmentDistance in src/distance.c.
func SegmentDistance(p1, q1, p2, q2 Vec2) SegmentDistanceResult {
	var result SegmentDistanceResult

	zero := fixed.Zero()
	one := fixed.One()

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
