package dbox2d

import "github.com/dhannyell/fixed"

// IsValidRay reports whether a ray cast input is usable.
func IsValidRay(input *RayCastInput) bool {
	isValid := IsValidVec2(input.Origin) && IsValidVec2(input.Translation) &&
		IsValidQ(input.MaxFraction) && !input.MaxFraction.Less(fixed.Zero()) && input.MaxFraction.Less(huge)
	return isValid
}

// computePolygonCentroid returns the area centroid of a convex polygon. It
// panics on a polygon with no area.
func computePolygonCentroid(vertices []Vec2) Vec2 {
	center := Vec2Zero()
	area := fixed.Zero()

	// Get a reference point for forming triangles.
	// Use the first vertex to reduce round-off errors.
	origin := vertices[0]

	half := fixed.Half()
	three := fixed.FromInt(3)

	for i := 1; i < len(vertices)-1; i++ {
		// Triangle edges
		e1 := vertices[i].Sub(origin)
		e2 := vertices[i+1].Sub(origin)
		a := half.Mul(Cross(e1, e2))

		// Area weighted centroid
		center = MulAdd(center, a.Div(three), e1.Add(e2))
		area = area.Add(a)
	}

	if !fixed.Zero().Less(area) {
		panic("dbox2d: a polygon centroid needs a positive area")
	}
	center = center.Div(area)

	// Restore offset
	center = origin.Add(center)

	return center
}

// MakePolygon returns a convex polygon built from a hull. The radius rounds
// the corners. It panics on a hull that ValidateHull rejects.
func MakePolygon(hull *Hull, radius Q) Polygon {
	if !ValidateHull(hull) {
		panic("dbox2d: MakePolygon needs a hull from ComputeHull")
	}

	shape := Polygon{}
	shape.Count = hull.Count
	shape.Radius = radius

	// Copy vertices
	for i := range shape.Count {
		shape.Vertices[i] = hull.Points[i]
	}

	// Compute normals. Ensure the edges have non-zero length.
	for i := range shape.Count {
		i1 := i
		i2 := 0
		if i+1 < shape.Count {
			i2 = i + 1
		}
		edge := shape.Vertices[i2].Sub(shape.Vertices[i1])
		if edge.Dot(edge).Eq(fixed.Zero()) {
			panic("dbox2d: a polygon edge has zero length")
		}
		shape.Normals[i] = CrossVS(edge, fixed.One()).Normalize()
	}

	shape.Centroid = computePolygonCentroid(shape.Vertices[:shape.Count])

	return shape
}

// MakeOffsetPolygon returns a convex polygon built from a hull and placed at
// a position and a rotation.
func MakeOffsetPolygon(hull *Hull, position Vec2, rotation Rot) Polygon {
	return MakeOffsetRoundedPolygon(hull, position, rotation, fixed.Zero())
}

// MakeOffsetRoundedPolygon returns a convex polygon built from a hull, placed
// at a position and a rotation, with rounded corners of the given radius.
func MakeOffsetRoundedPolygon(hull *Hull, position Vec2, rotation Rot, radius Q) Polygon {
	if !ValidateHull(hull) {
		panic("dbox2d: MakeOffsetRoundedPolygon needs a hull from ComputeHull")
	}

	transform := Transform{P: position, Q: rotation}

	shape := Polygon{}
	shape.Count = hull.Count
	shape.Radius = radius

	// Copy vertices
	for i := range shape.Count {
		shape.Vertices[i] = TransformPoint(transform, hull.Points[i])
	}

	// Compute normals. Ensure the edges have non-zero length.
	for i := range shape.Count {
		i1 := i
		i2 := 0
		if i+1 < shape.Count {
			i2 = i + 1
		}
		edge := shape.Vertices[i2].Sub(shape.Vertices[i1])
		if edge.Dot(edge).Eq(fixed.Zero()) {
			panic("dbox2d: a polygon edge has zero length")
		}
		shape.Normals[i] = CrossVS(edge, fixed.One()).Normalize()
	}

	shape.Centroid = computePolygonCentroid(shape.Vertices[:shape.Count])

	return shape
}

// MakeSquare returns a square polygon of the given half-width, without a
// hull.
func MakeSquare(halfWidth Q) Polygon {
	return MakeBox(halfWidth, halfWidth)
}

// MakeBox returns a rectangle polygon of the given half-extents, without a
// hull. It panics on a half-extent that is not positive.
func MakeBox(halfWidth, halfHeight Q) Polygon {
	zero := fixed.Zero()
	one := fixed.One()

	if !IsValidQ(halfWidth) || !zero.Less(halfWidth) {
		panic("dbox2d: MakeBox needs a positive half-width")
	}
	if !IsValidQ(halfHeight) || !zero.Less(halfHeight) {
		panic("dbox2d: MakeBox needs a positive half-height")
	}

	shape := Polygon{}
	shape.Count = 4
	shape.Vertices[0] = Vec2{X: halfWidth.Neg(), Y: halfHeight.Neg()}
	shape.Vertices[1] = Vec2{X: halfWidth, Y: halfHeight.Neg()}
	shape.Vertices[2] = Vec2{X: halfWidth, Y: halfHeight}
	shape.Vertices[3] = Vec2{X: halfWidth.Neg(), Y: halfHeight}
	shape.Normals[0] = Vec2{X: zero, Y: one.Neg()}
	shape.Normals[1] = Vec2{X: one, Y: zero}
	shape.Normals[2] = Vec2{X: zero, Y: one}
	shape.Normals[3] = Vec2{X: one.Neg(), Y: zero}
	shape.Radius = zero
	shape.Centroid = Vec2Zero()
	return shape
}

// MakeRoundedBox returns a rectangle polygon with rounded corners, without a
// hull.
func MakeRoundedBox(halfWidth, halfHeight, radius Q) Polygon {
	if !IsValidQ(radius) || radius.Less(fixed.Zero()) {
		panic("dbox2d: MakeRoundedBox needs a radius that is not negative")
	}
	shape := MakeBox(halfWidth, halfHeight)
	shape.Radius = radius
	return shape
}

// MakeOffsetBox returns a rectangle polygon placed at a center and a
// rotation, without a hull.
func MakeOffsetBox(halfWidth, halfHeight Q, center Vec2, rotation Rot) Polygon {
	return MakeOffsetRoundedBox(halfWidth, halfHeight, center, rotation, fixed.Zero())
}

// MakeOffsetRoundedBox returns a rectangle polygon with rounded corners,
// placed at a center and a rotation, without a hull.
func MakeOffsetRoundedBox(halfWidth, halfHeight Q, center Vec2, rotation Rot, radius Q) Polygon {
	zero := fixed.Zero()
	one := fixed.One()

	if !IsValidQ(radius) || radius.Less(zero) {
		panic("dbox2d: MakeOffsetRoundedBox needs a radius that is not negative")
	}

	xf := Transform{P: center, Q: rotation}

	shape := Polygon{}
	shape.Count = 4
	shape.Vertices[0] = TransformPoint(xf, Vec2{X: halfWidth.Neg(), Y: halfHeight.Neg()})
	shape.Vertices[1] = TransformPoint(xf, Vec2{X: halfWidth, Y: halfHeight.Neg()})
	shape.Vertices[2] = TransformPoint(xf, Vec2{X: halfWidth, Y: halfHeight})
	shape.Vertices[3] = TransformPoint(xf, Vec2{X: halfWidth.Neg(), Y: halfHeight})
	shape.Normals[0] = RotateVector(xf.Q, Vec2{X: zero, Y: one.Neg()})
	shape.Normals[1] = RotateVector(xf.Q, Vec2{X: one, Y: zero})
	shape.Normals[2] = RotateVector(xf.Q, Vec2{X: zero, Y: one})
	shape.Normals[3] = RotateVector(xf.Q, Vec2{X: one.Neg(), Y: zero})
	shape.Radius = radius
	shape.Centroid = xf.P
	return shape
}

// TransformPolygon returns the polygon moved by a transform. It moves a shape
// from one body to another.
func TransformPolygon(transform Transform, polygon *Polygon) Polygon {
	p := *polygon

	for i := range p.Count {
		p.Vertices[i] = TransformPoint(transform, p.Vertices[i])
		p.Normals[i] = RotateVector(transform.Q, p.Normals[i])
	}

	p.Centroid = TransformPoint(transform, p.Centroid)

	return p
}

// ComputeCircleMass returns the mass properties of a circle.
func ComputeCircleMass(shape *Circle, density Q) MassData {
	rr := shape.Radius.Mul(shape.Radius)

	massData := MassData{}
	massData.Mass = density.Mul(pi).Mul(rr)
	massData.Center = shape.Center

	// inertia about the local origin
	massData.RotationalInertia = massData.Mass.Mul(fixed.Half().Mul(rr).Add(shape.Center.Dot(shape.Center)))

	return massData
}

// ComputeCapsuleMass returns the mass properties of a capsule.
func ComputeCapsuleMass(shape *Capsule, density Q) MassData {
	half := fixed.Half()
	two := fixed.FromInt(2)
	three := fixed.FromInt(3)
	four := fixed.FromInt(4)
	twelve := fixed.FromInt(12)

	radius := shape.Radius
	rr := radius.Mul(radius)
	p1 := shape.Center1
	p2 := shape.Center2
	length := p2.Sub(p1).Len()
	ll := length.Mul(length)

	circleMass := density.Mul(pi.Mul(radius).Mul(radius))
	boxMass := density.Mul(two.Mul(radius).Mul(length))

	massData := MassData{}
	massData.Mass = circleMass.Add(boxMass)
	massData.Center.X = half.Mul(p1.X.Add(p2.X))
	massData.Center.Y = half.Mul(p1.Y.Add(p2.Y))

	// Two offset half circles. Both halves add up to a full circle and each
	// half is offset by half the length. The parallel-axis theorem applies
	// twice: first it shifts the semicircle centroid to the origin, then it
	// shifts the semicircle to the end of the box.

	// half circle centroid, upstream 4 r / (3 pi)
	lc := four.Mul(radius).Div(three.Mul(pi))

	// half length of rectangular portion of capsule
	h := half.Mul(length)

	circleInertia := circleMass.Mul(half.Mul(rr).Add(h.Mul(h)).Add(two.Mul(h).Mul(lc)))
	boxInertia := boxMass.Mul(four.Mul(rr).Add(ll)).Div(twelve)
	massData.RotationalInertia = circleInertia.Add(boxInertia)

	// inertia about the local origin
	massData.RotationalInertia = massData.RotationalInertia.Add(
		massData.Mass.Mul(massData.Center.Dot(massData.Center)))

	return massData
}

// ComputePolygonMass returns the mass properties of a convex polygon. It
// approximates a rounded polygon by pushing the vertices outward.
func ComputePolygonMass(shape *Polygon, density Q) MassData {
	// The mass, the centroid and the inertia come from one integral per
	// triangle of the polygon fan. See the reference for the derivation.

	if shape.Count <= 0 {
		panic("dbox2d: ComputePolygonMass needs at least one vertex")
	}

	if shape.Count == 1 {
		circle := Circle{Center: shape.Vertices[0], Radius: shape.Radius}
		return ComputeCircleMass(&circle, density)
	}

	if shape.Count == 2 {
		capsule := Capsule{Center1: shape.Vertices[0], Center2: shape.Vertices[1], Radius: shape.Radius}
		return ComputeCapsuleMass(&capsule, density)
	}

	zero := fixed.Zero()
	half := fixed.Half()
	quarter := fixed.MustParse("0.25")
	three := fixed.FromInt(3)

	var vertices [MaxPolygonVertices]Vec2
	count := shape.Count
	radius := shape.Radius

	if zero.Less(radius) {
		// Approximate mass of rounded polygons by pushing out the vertices.
		sqrt2 := fixed.MustParse("1.412")
		for i := range count {
			j := i - 1
			if i == 0 {
				j = count - 1
			}
			n1 := shape.Normals[j]
			n2 := shape.Normals[i]

			mid := n1.Add(n2).Normalize()
			vertices[i] = MulAdd(shape.Vertices[i], sqrt2.Mul(radius), mid)
		}
	} else {
		for i := range count {
			vertices[i] = shape.Vertices[i]
		}
	}

	center := Vec2Zero()
	area := zero
	rotationalInertia := zero

	// Get a reference point for forming triangles.
	// Use the first vertex to reduce round-off errors.
	r := vertices[0]

	for i := 1; i < count-1; i++ {
		// Triangle edges
		e1 := vertices[i].Sub(r)
		e2 := vertices[i+1].Sub(r)

		d := Cross(e1, e2)

		triangleArea := half.Mul(d)
		area = area.Add(triangleArea)

		// Area weighted centroid, r at origin
		center = MulAdd(center, triangleArea.Div(three), e1.Add(e2))

		ex1, ey1 := e1.X, e1.Y
		ex2, ey2 := e2.X, e2.Y

		intx2 := ex1.Mul(ex1).Add(ex2.Mul(ex1)).Add(ex2.Mul(ex2))
		inty2 := ey1.Mul(ey1).Add(ey2.Mul(ey1)).Add(ey2.Mul(ey2))

		rotationalInertia = rotationalInertia.Add(quarter.Mul(d).Div(three).Mul(intx2.Add(inty2)))
	}

	massData := MassData{}

	// Total mass
	massData.Mass = density.Mul(area)

	// Center of mass, shift back from origin at r
	if !zero.Less(area) {
		panic("dbox2d: ComputePolygonMass needs a positive area")
	}
	center = center.Div(area)
	massData.Center = r.Add(center)

	// Inertia tensor relative to the local origin.
	massData.RotationalInertia = density.Mul(rotationalInertia)

	// Shift to center of mass then to original body origin.
	massData.RotationalInertia = massData.RotationalInertia.Add(
		massData.Mass.Mul(massData.Center.Dot(massData.Center).Sub(center.Dot(center))))

	return massData
}

// ComputeCircleAABB returns the bounding box of a transformed circle.
func ComputeCircleAABB(shape *Circle, xf Transform) AABB {
	p := TransformPoint(xf, shape.Center)
	r := shape.Radius

	aabb := AABB{
		LowerBound: Vec2{X: p.X.Sub(r), Y: p.Y.Sub(r)},
		UpperBound: Vec2{X: p.X.Add(r), Y: p.Y.Add(r)},
	}
	return aabb
}

// ComputeCapsuleAABB returns the bounding box of a transformed capsule.
func ComputeCapsuleAABB(shape *Capsule, xf Transform) AABB {
	v1 := TransformPoint(xf, shape.Center1)
	v2 := TransformPoint(xf, shape.Center2)

	r := Vec2{X: shape.Radius, Y: shape.Radius}
	lower := Min(v1, v2).Sub(r)
	upper := Max(v1, v2).Add(r)

	aabb := AABB{LowerBound: lower, UpperBound: upper}
	return aabb
}

// ComputePolygonAABB returns the bounding box of a transformed polygon.
func ComputePolygonAABB(shape *Polygon, xf Transform) AABB {
	if shape.Count <= 0 {
		panic("dbox2d: ComputePolygonAABB needs at least one vertex")
	}
	lower := TransformPoint(xf, shape.Vertices[0])
	upper := lower

	for i := 1; i < shape.Count; i++ {
		v := TransformPoint(xf, shape.Vertices[i])
		lower = Min(lower, v)
		upper = Max(upper, v)
	}

	r := Vec2{X: shape.Radius, Y: shape.Radius}
	lower = lower.Sub(r)
	upper = upper.Add(r)

	aabb := AABB{LowerBound: lower, UpperBound: upper}
	return aabb
}

// ComputeSegmentAABB returns the bounding box of a transformed segment.
func ComputeSegmentAABB(shape *Segment, xf Transform) AABB {
	v1 := TransformPoint(xf, shape.Point1)
	v2 := TransformPoint(xf, shape.Point2)

	lower := Min(v1, v2)
	upper := Max(v1, v2)

	aabb := AABB{LowerBound: lower, UpperBound: upper}
	return aabb
}

// PointInCircle reports whether a point overlaps a circle in local space.
func PointInCircle(point Vec2, shape *Circle) bool {
	center := shape.Center
	return !shape.Radius.Mul(shape.Radius).Less(point.DistanceSq(center))
}

// PointInCapsule reports whether a point overlaps a capsule in local space.
func PointInCapsule(point Vec2, shape *Capsule) bool {
	rr := shape.Radius.Mul(shape.Radius)
	p1 := shape.Center1
	p2 := shape.Center2

	d := p2.Sub(p1)
	dd := d.Dot(d)
	if dd.Eq(fixed.Zero()) {
		// Capsule is really a circle
		return !rr.Less(point.DistanceSq(p1))
	}

	// Get closest point on capsule segment
	// c = p1 + t * d
	// dot(point - c, d) = 0
	// t = dot(point - p1, d) / dot(d, d)
	t := point.Sub(p1).Dot(d).Div(dd)
	t = t.Clamp(fixed.Zero(), fixed.One())
	c := MulAdd(p1, t, d)

	// Is query point within radius around closest point?
	return !rr.Less(point.DistanceSq(c))
}

// RayCastCircle casts a ray against a circle in local space. An initial
// overlap reports a hit at the ray origin with a zero fraction.
func RayCastCircle(input *RayCastInput, shape *Circle) CastOutput {
	if !IsValidRay(input) {
		panic("dbox2d: RayCastCircle needs a valid ray")
	}

	zero := fixed.Zero()
	p := shape.Center

	output := CastOutput{}

	// Shift ray so circle center is the origin
	s := input.Origin.Sub(p)

	r := shape.Radius
	rr := r.Mul(r)

	length, d := GetLengthAndNormalize(input.Translation)
	if length.Eq(zero) {
		// zero length ray

		if s.LenSq().Less(rr) {
			// initial overlap
			output.Point = input.Origin
			output.Hit = true
		}

		return output
	}

	// Find closest point on ray to origin
	// solve: dot(s + t * d, d) = 0
	t := s.Dot(d).Neg()

	// c is the closest point on the line to the origin
	c := MulAdd(s, t, d)

	cc := c.Dot(c)

	if rr.Less(cc) {
		// closest point is outside the circle
		return output
	}

	// Pythagoras
	h := rr.Sub(cc).Sqrt()

	fraction := t.Sub(h)

	if fraction.Less(zero) || input.MaxFraction.Mul(length).Less(fraction) {
		// intersection is point outside the range of the ray segment

		if s.LenSq().Less(rr) {
			// initial overlap
			output.Point = input.Origin
			output.Hit = true
		}

		return output
	}

	// hit point relative to center
	hitPoint := MulAdd(s, fraction, d)

	output.Fraction = fraction.Div(length)
	output.Normal = hitPoint.Normalize()
	output.Point = MulAdd(p, shape.Radius, output.Normal)
	output.Hit = true

	return output
}

// RayCastCapsule casts a ray against a capsule in local space. An initial
// overlap reports a hit at the ray origin with a zero fraction.
func RayCastCapsule(input *RayCastInput, shape *Capsule) CastOutput {
	if !IsValidRay(input) {
		panic("dbox2d: RayCastCapsule needs a valid ray")
	}

	zero := fixed.Zero()
	output := CastOutput{}

	v1 := shape.Center1
	v2 := shape.Center2

	e := v2.Sub(v1)

	capsuleLength, a := GetLengthAndNormalize(e)

	if capsuleLength.Eq(zero) {
		// Capsule is really a circle
		circle := Circle{Center: v1, Radius: shape.Radius}
		return RayCastCircle(input, &circle)
	}

	p1 := input.Origin
	d := input.Translation

	// Ray from capsule start to ray start
	q := p1.Sub(v1)
	qa := q.Dot(a)

	// Vector to ray start that is perpendicular to capsule axis
	qp := MulAdd(q, qa.Neg(), a)

	radius := shape.Radius

	// Does the ray start within the infinite length capsule?
	if qp.Dot(qp).Less(radius.Mul(radius)) {
		if qa.Less(zero) {
			// start point behind capsule segment
			circle := Circle{Center: v1, Radius: shape.Radius}
			return RayCastCircle(input, &circle)
		}

		if capsuleLength.Less(qa) {
			// start point ahead of capsule segment
			circle := Circle{Center: v2, Radius: shape.Radius}
			return RayCastCircle(input, &circle)
		}

		// ray starts inside capsule -> no hit
		output.Point = input.Origin
		output.Hit = true
		return output
	}

	// Perpendicular to capsule axis, pointing right
	n := Vec2{X: a.Y, Y: a.X.Neg()}

	rayLength, u := GetLengthAndNormalize(d)

	// Intersect ray with infinite length capsule
	// v1 + radius * n + s1 * a = p1 + s2 * u
	// v1 - radius * n + s1 * a = p1 + s2 * u
	//
	// s1 * a - s2 * u = b, with b = q - radius * n or b = q + radius * n

	// Cramer's rule [a -u]
	den := a.X.Neg().Mul(u.Y).Add(u.X.Mul(a.Y))
	if den.Eq(zero) {
		// Ray is parallel to capsule and outside infinite length capsule
		return output
	}

	b1 := MulSub(q, radius, n)
	b2 := MulAdd(q, radius, n)

	// Cramer's rule [a b1]
	s21 := a.X.Mul(b1.Y).Sub(b1.X.Mul(a.Y)).Div(den)

	// Cramer's rule [a b2]
	s22 := a.X.Mul(b2.Y).Sub(b2.X.Mul(a.Y)).Div(den)

	var s2 Q
	var b Vec2
	if s21.Less(s22) {
		s2 = s21
		b = b1
	} else {
		s2 = s22
		b = b2
		n = Neg(n)
	}

	if s2.Less(zero) || input.MaxFraction.Mul(rayLength).Less(s2) {
		return output
	}

	// Cramer's rule [b -u]
	s1 := b.X.Neg().Mul(u.Y).Add(u.X.Mul(b.Y)).Div(den)

	if s1.Less(zero) {
		// ray passes behind capsule segment
		circle := Circle{Center: v1, Radius: shape.Radius}
		return RayCastCircle(input, &circle)
	} else if capsuleLength.Less(s1) {
		// ray passes ahead of capsule segment
		circle := Circle{Center: v2, Radius: shape.Radius}
		return RayCastCircle(input, &circle)
	}

	// ray hits capsule side
	output.Fraction = s2.Div(rayLength)
	output.Point = Lerp(v1, v2, s1.Div(capsuleLength)).Add(n.Mul(shape.Radius))
	output.Normal = n
	output.Hit = true
	return output
}

// RayCastSegment casts a ray against a segment in local space. A one-sided
// segment reports a miss for a ray that arrives from the left.
func RayCastSegment(input *RayCastInput, shape *Segment, oneSided bool) CastOutput {
	zero := fixed.Zero()

	if oneSided {
		// Skip left-side collision
		offset := Cross(input.Origin.Sub(shape.Point1), shape.Point2.Sub(shape.Point1))
		if offset.Less(zero) {
			output := CastOutput{}
			return output
		}
	}

	// Put the ray into the frame of reference of the edge.
	p1 := input.Origin
	d := input.Translation

	v1 := shape.Point1
	v2 := shape.Point2
	e := v2.Sub(v1)

	output := CastOutput{}

	length, eUnit := GetLengthAndNormalize(e)
	if length.Eq(zero) {
		return output
	}

	// Normal points to the right, looking from v1 towards v2
	normal := RightPerp(eUnit)

	// Intersect ray with infinite segment using normal
	// p = p1 + t * d
	// dot(normal, p - v1) = 0
	numerator := normal.Dot(v1.Sub(p1))
	denominator := normal.Dot(d)

	if denominator.Eq(zero) {
		// parallel
		return output
	}

	t := numerator.Div(denominator)
	if t.Less(zero) || input.MaxFraction.Less(t) {
		// out of ray range
		return output
	}

	// Intersection point on infinite segment
	p := MulAdd(p1, t, d)

	// Compute position of p along segment
	// p = v1 + s * e
	s := p.Sub(v1).Dot(eUnit)
	if s.Less(zero) || length.Less(s) {
		// out of segment range
		return output
	}

	if zero.Less(numerator) {
		normal = Neg(normal)
	}

	output.Fraction = t
	output.Point = p
	output.Normal = normal
	output.Hit = true

	return output
}

// RayCastPolygon casts a ray against a convex polygon in local space. An
// initial overlap reports a hit at the ray origin with a zero fraction.
//
// A rounded polygon needs the shape cast, which arrives with the distance
// stage, so this function accepts only a polygon with a zero radius.
func RayCastPolygon(input *RayCastInput, shape *Polygon) CastOutput {
	if !IsValidRay(input) {
		panic("dbox2d: RayCastPolygon needs a valid ray")
	}

	zero := fixed.Zero()

	if !shape.Radius.Eq(zero) {
		panic("dbox2d: RayCastPolygon does not yet accept a rounded polygon")
	}

	// Shift all math to first vertex since the polygon may be far
	// from the origin.
	base := shape.Vertices[0]

	p1 := input.Origin.Sub(base)
	d := input.Translation

	lower, upper := zero, input.MaxFraction

	index := -1

	output := CastOutput{}

	for i := range shape.Count {
		// p = p1 + a * d
		// dot(normal, p - v) = 0
		vertex := shape.Vertices[i].Sub(base)
		numerator := shape.Normals[i].Dot(vertex.Sub(p1))
		denominator := shape.Normals[i].Dot(d)

		if denominator.Eq(zero) {
			if numerator.Less(zero) {
				return output
			}
		} else {
			// The predicate avoids a division: for a negative denominator,
			// lower < numerator / denominator flips to
			// denominator * lower > numerator.
			if denominator.Less(zero) && numerator.Less(lower.Mul(denominator)) {
				// Increase lower.
				// The segment enters this half-space.
				lower = numerator.Div(denominator)
				index = i
			} else if zero.Less(denominator) && numerator.Less(upper.Mul(denominator)) {
				// Decrease upper.
				// The segment exits this half-space.
				upper = numerator.Div(denominator)
			}
		}

		if upper.Less(lower) {
			// Ray misses
			return output
		}
	}

	if lower.Less(zero) || input.MaxFraction.Less(lower) {
		panic("dbox2d: the polygon ray cast left the ray range")
	}

	if index >= 0 {
		output.Fraction = lower
		output.Normal = shape.Normals[index]
		output.Point = MulAdd(input.Origin, lower, d)
		output.Hit = true
	} else {
		// initial overlap
		output.Point = input.Origin
		output.Hit = true
	}

	return output
}
