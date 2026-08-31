package dbox2d

import "github.com/dhannyell/fixed"

// makeId packs two feature indices into a contact point id. It corresponds
// to B2_MAKE_ID in src/manifold.c.
func makeId(a, b int) uint16 {
	return uint16(uint8(a))<<8 | uint16(uint8(b))
}

// CollideCircles computes the contact manifold of two circles. It
// corresponds to b2CollideCircles in src/manifold.c.
func CollideCircles(circleA *Circle, xfA Transform, circleB *Circle, xfB Transform) Manifold {
	var manifold Manifold

	xf := InvMulTransforms(xfA, xfB)

	pointA := circleA.Center
	pointB := TransformPoint(xf, circleB.Center)

	distance, normal := GetLengthAndNormalize(pointB.Sub(pointA))

	radiusA := circleA.Radius
	radiusB := circleB.Radius

	separation := distance.Sub(radiusA).Sub(radiusB)
	if SpeculativeDistance().Less(separation) {
		return manifold
	}

	cA := MulAdd(pointA, radiusA, normal)
	cB := MulAdd(pointB, radiusB.Neg(), normal)
	contactPointA := Lerp(cA, cB, fixed.Half())

	manifold.Normal = RotateVector(xfA.Q, normal)
	mp := &manifold.Points[0]
	mp.AnchorA = RotateVector(xfA.Q, contactPointA)
	mp.AnchorB = mp.AnchorA.Add(xfA.P.Sub(xfB.P))
	mp.Point = mp.AnchorA.Add(xfA.P)
	mp.Separation = separation
	mp.Id = 0
	manifold.PointCount = 1
	return manifold
}

// CollideCapsuleAndCircle computes the contact manifold of a capsule and a
// circle. It corresponds to b2CollideCapsuleAndCircle in src/manifold.c.
func CollideCapsuleAndCircle(capsuleA *Capsule, xfA Transform, circleB *Circle, xfB Transform) Manifold {
	var manifold Manifold

	xf := InvMulTransforms(xfA, xfB)

	// Compute the circle position in the frame of the capsule.
	pB := TransformPoint(xf, circleB.Center)

	// Compute the closest point.
	p1 := capsuleA.Center1
	p2 := capsuleA.Center2

	e := p2.Sub(p1)

	var pA Vec2
	s1 := pB.Sub(p1).Dot(e)
	s2 := p2.Sub(pB).Dot(e)
	zero := fixed.Zero()
	if s1.Less(zero) {
		// The p1 region.
		pA = p1
	} else if s2.Less(zero) {
		// The p2 region.
		pA = p2
	} else {
		// The circle collides with the segment interior.
		s := s1.Div(e.Dot(e))
		pA = MulAdd(p1, s, e)
	}

	distance, normal := GetLengthAndNormalize(pB.Sub(pA))

	radiusA := capsuleA.Radius
	radiusB := circleB.Radius
	separation := distance.Sub(radiusA).Sub(radiusB)
	if SpeculativeDistance().Less(separation) {
		return manifold
	}

	cA := MulAdd(pA, radiusA, normal)
	cB := MulAdd(pB, radiusB.Neg(), normal)
	contactPointA := Lerp(cA, cB, fixed.Half())

	manifold.Normal = RotateVector(xfA.Q, normal)
	mp := &manifold.Points[0]
	mp.AnchorA = RotateVector(xfA.Q, contactPointA)
	mp.AnchorB = mp.AnchorA.Add(xfA.P.Sub(xfB.P))
	mp.Point = xfA.P.Add(mp.AnchorA)
	mp.Separation = separation
	mp.Id = 0
	manifold.PointCount = 1
	return manifold
}

// CollidePolygonAndCircle computes the contact manifold of a polygon and a
// circle. It corresponds to b2CollidePolygonAndCircle in src/manifold.c.
func CollidePolygonAndCircle(polygonA *Polygon, xfA Transform, circleB *Circle, xfB Transform) Manifold {
	var manifold Manifold
	speculative := SpeculativeDistance()
	zero := fixed.Zero()
	half := fixed.Half()

	xf := InvMulTransforms(xfA, xfB)

	// Compute the circle position in the frame of the polygon.
	center := TransformPoint(xf, circleB.Center)
	radiusA := polygonA.Radius
	radiusB := circleB.Radius
	radius := radiusA.Add(radiusB)

	// Find the minimum separating edge. The seed follows D-009.
	normalIndex := 0
	separation := fixed.MinValue()
	vertexCount := polygonA.Count
	for i := range vertexCount {
		s := polygonA.Normals[i].Dot(center.Sub(polygonA.Vertices[i]))
		if separation.Less(s) {
			separation = s
			normalIndex = i
		}
	}

	if radius.Add(speculative).Less(separation) {
		return manifold
	}

	// Vertices of the reference edge.
	vertIndex1 := normalIndex
	vertIndex2 := 0
	if vertIndex1+1 < vertexCount {
		vertIndex2 = vertIndex1 + 1
	}
	v1 := polygonA.Vertices[vertIndex1]
	v2 := polygonA.Vertices[vertIndex2]

	// Compute barycentric coordinates.
	u1 := center.Sub(v1).Dot(v2.Sub(v1))
	u2 := center.Sub(v2).Dot(v1.Sub(v2))

	// The reference guards the vertex regions with FLT_EPSILON. In Q any
	// exactly positive separation is safe to normalize. See D-012.
	if u1.Less(zero) && zero.Less(separation) {
		// The circle center is closest to v1 and safely outside the polygon.
		normal := center.Sub(v1).Normalize()
		separation = center.Sub(v1).Dot(normal)
		if radius.Add(speculative).Less(separation) {
			return manifold
		}

		cA := MulAdd(v1, radiusA, normal)
		cB := MulSub(center, radiusB, normal)
		contactPointA := Lerp(cA, cB, half)

		manifold.Normal = RotateVector(xfA.Q, normal)
		mp := &manifold.Points[0]
		mp.AnchorA = RotateVector(xfA.Q, contactPointA)
		mp.AnchorB = mp.AnchorA.Add(xfA.P.Sub(xfB.P))
		mp.Point = xfA.P.Add(mp.AnchorA)
		mp.Separation = cB.Sub(cA).Dot(normal)
		mp.Id = 0
		manifold.PointCount = 1
	} else if u2.Less(zero) && zero.Less(separation) {
		// The circle center is closest to v2 and safely outside the polygon.
		normal := center.Sub(v2).Normalize()
		separation = center.Sub(v2).Dot(normal)
		if radius.Add(speculative).Less(separation) {
			return manifold
		}

		cA := MulAdd(v2, radiusA, normal)
		cB := MulSub(center, radiusB, normal)
		contactPointA := Lerp(cA, cB, half)

		manifold.Normal = RotateVector(xfA.Q, normal)
		mp := &manifold.Points[0]
		mp.AnchorA = RotateVector(xfA.Q, contactPointA)
		mp.AnchorB = mp.AnchorA.Add(xfA.P.Sub(xfB.P))
		mp.Point = xfA.P.Add(mp.AnchorA)
		mp.Separation = cB.Sub(cA).Dot(normal)
		mp.Id = 0
		manifold.PointCount = 1
	} else {
		// The circle center is between v1 and v2. It may be inside the
		// polygon.
		normal := polygonA.Normals[normalIndex]
		manifold.Normal = RotateVector(xfA.Q, normal)

		// cA is the projection of the circle center onto the reference edge.
		cA := MulAdd(center, radiusA.Sub(center.Sub(v1).Dot(normal)), normal)

		// cB is the deepest point of the circle on the reference edge.
		cB := MulSub(center, radiusB, normal)

		contactPointA := Lerp(cA, cB, half)

		mp := &manifold.Points[0]
		mp.AnchorA = RotateVector(xfA.Q, contactPointA)
		mp.AnchorB = mp.AnchorA.Add(xfA.P.Sub(xfB.P))
		mp.Point = xfA.P.Add(mp.AnchorA)
		mp.Separation = separation.Sub(radius)
		mp.Id = 0
		manifold.PointCount = 1
	}

	return manifold
}

// CollideCapsules computes the contact manifold of two capsules. It follows
// Ericson 5.1.9 with clipping for a second contact point and corresponds to
// b2CollideCapsules in src/manifold.c.
func CollideCapsules(capsuleA *Capsule, xfA Transform, capsuleB *Capsule, xfB Transform) Manifold {
	origin := capsuleA.Center1

	// Shift capsule A to the origin.
	sfA := Transform{P: xfA.P.Add(RotateVector(xfA.Q, origin)), Q: xfA.Q}
	xf := InvMulTransforms(sfA, xfB)

	p1 := Vec2{}
	q1 := capsuleA.Center2.Sub(origin)

	p2 := TransformPoint(xf, capsuleB.Center1)
	q2 := TransformPoint(xf, capsuleB.Center2)

	d1 := q1.Sub(p1)
	d2 := q2.Sub(p2)

	dd1 := d1.Dot(d1)
	dd2 := d2.Dot(d2)

	zero := fixed.Zero()
	one := fixed.One()
	half := fixed.Half()
	linearSlop := LinearSlop()

	// The reference asserts both lengths against FLT_EPSILON squared. The
	// exact zero is the Q form. See D-003 and D-012.
	if dd1.Eq(zero) || dd2.Eq(zero) {
		panic("dbox2d: degenerate capsule")
	}

	r := p1.Sub(p2)
	rd1 := r.Dot(d1)
	rd2 := r.Dot(d2)

	d12 := d1.Dot(d2)

	denom := dd1.Mul(dd2).Sub(d12.Mul(d12))

	// Fraction on segment 1.
	f1 := zero
	if !denom.Eq(zero) {
		// Not parallel.
		f1 = d12.Mul(rd2).Sub(rd1.Mul(dd2)).Div(denom).Clamp(zero, one)
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

	closest1 := MulAdd(p1, f1, d1)
	closest2 := MulAdd(p2, f2, d2)
	distanceSquared := closest1.DistanceSq(closest2)

	var manifold Manifold
	radiusA := capsuleA.Radius
	radiusB := capsuleB.Radius
	radius := radiusA.Add(radiusB)
	maxDistance := radius.Add(SpeculativeDistance())

	if maxDistance.Mul(maxDistance).Less(distanceSquared) {
		return manifold
	}

	distance := distanceSquared.Sqrt()

	length1, u1 := GetLengthAndNormalize(d1)
	length2, u2 := GetLengthAndNormalize(d2)

	// Does segment B project outside segment A?
	fp2 := p2.Sub(p1).Dot(u1)
	fq2 := q2.Sub(p1).Dot(u1)
	outsideA := (fp2.Cmp(zero) <= 0 && fq2.Cmp(zero) <= 0) ||
		(fp2.Cmp(length1) >= 0 && fq2.Cmp(length1) >= 0)

	// Does segment A project outside segment B?
	fp1 := p1.Sub(p2).Dot(u2)
	fq1 := q1.Sub(p2).Dot(u2)
	outsideB := (fp1.Cmp(zero) <= 0 && fq1.Cmp(zero) <= 0) ||
		(fp1.Cmp(length2) >= 0 && fq1.Cmp(length2) >= 0)

	if !outsideA && !outsideB {
		// Attempt to clip. This may yield contact points with excessive
		// separation; then the algorithm falls back to a single point.

		// Find the reference edge using SAT.
		normalA := LeftPerp(u1)
		var separationA Q
		{
			ss1 := p2.Sub(p1).Dot(normalA)
			ss2 := q2.Sub(p1).Dot(normalA)
			s1p := ss1.Min(ss2)
			s1n := ss1.Neg().Min(ss2.Neg())

			if s1n.Less(s1p) {
				separationA = s1p
			} else {
				separationA = s1n
				normalA = Neg(normalA)
			}
		}

		normalB := LeftPerp(u2)
		var separationB Q
		{
			ss1 := p1.Sub(p2).Dot(normalB)
			ss2 := q1.Sub(p2).Dot(normalB)
			s1p := ss1.Min(ss2)
			s1n := ss1.Neg().Min(ss2.Neg())

			if s1n.Less(s1p) {
				separationB = s1p
			} else {
				separationB = s1n
				normalB = Neg(normalB)
			}
		}

		// Biased to avoid feature flip-flop; upstream 0.1f * B2_LINEAR_SLOP.
		slopBias := linearSlop.Div(fixed.FromInt(10))
		if !separationA.Add(slopBias).Less(separationB) {
			manifold.Normal = normalA

			cp := p2
			cq := q2

			// Clip to p1.
			if fp2.Less(zero) && zero.Less(fq2) {
				cp = Lerp(p2, q2, zero.Sub(fp2).Div(fq2.Sub(fp2)))
			} else if fq2.Less(zero) && zero.Less(fp2) {
				cq = Lerp(q2, p2, zero.Sub(fq2).Div(fp2.Sub(fq2)))
			}

			// Clip to q1.
			if length1.Less(fp2) && fq2.Less(length1) {
				cp = Lerp(p2, q2, fp2.Sub(length1).Div(fp2.Sub(fq2)))
			} else if length1.Less(fq2) && fp2.Less(length1) {
				cq = Lerp(q2, p2, fq2.Sub(length1).Div(fq2.Sub(fp2)))
			}

			sp := cp.Sub(p1).Dot(normalA)
			sq := cq.Sub(p1).Dot(normalA)

			if sp.Cmp(distance.Add(linearSlop)) <= 0 || sq.Cmp(distance.Add(linearSlop)) <= 0 {
				mp := &manifold.Points[0]
				mp.AnchorA = MulAdd(cp, half.Mul(radiusA.Sub(radiusB).Sub(sp)), normalA)
				mp.Separation = sp.Sub(radius)
				mp.Id = makeId(0, 0)

				mp = &manifold.Points[1]
				mp.AnchorA = MulAdd(cq, half.Mul(radiusA.Sub(radiusB).Sub(sq)), normalA)
				mp.Separation = sq.Sub(radius)
				mp.Id = makeId(0, 1)
				manifold.PointCount = 2
			}
		} else {
			// The normal always points from A to B.
			manifold.Normal = Neg(normalB)

			cp := p1
			cq := q1

			// Clip to p2.
			if fp1.Less(zero) && zero.Less(fq1) {
				cp = Lerp(p1, q1, zero.Sub(fp1).Div(fq1.Sub(fp1)))
			} else if fq1.Less(zero) && zero.Less(fp1) {
				cq = Lerp(q1, p1, zero.Sub(fq1).Div(fp1.Sub(fq1)))
			}

			// Clip to q2.
			if length2.Less(fp1) && fq1.Less(length2) {
				cp = Lerp(p1, q1, fp1.Sub(length2).Div(fp1.Sub(fq1)))
			} else if length2.Less(fq1) && fp1.Less(length2) {
				cq = Lerp(q1, p1, fq1.Sub(length2).Div(fq1.Sub(fp1)))
			}

			sp := cp.Sub(p2).Dot(normalB)
			sq := cq.Sub(p2).Dot(normalB)

			if sp.Cmp(distance.Add(linearSlop)) <= 0 || sq.Cmp(distance.Add(linearSlop)) <= 0 {
				mp := &manifold.Points[0]
				mp.AnchorA = MulAdd(cp, half.Mul(radiusB.Sub(radiusA).Sub(sp)), normalB)
				mp.Separation = sp.Sub(radius)
				mp.Id = makeId(0, 0)

				mp = &manifold.Points[1]
				mp.AnchorA = MulAdd(cq, half.Mul(radiusB.Sub(radiusA).Sub(sq)), normalB)
				mp.Separation = sq.Sub(radius)
				mp.Id = makeId(1, 0)
				manifold.PointCount = 2
			}
		}
	}

	if manifold.PointCount == 0 {
		// Single point collision. The reference guards the normalization
		// with FLT_EPSILON squared; the exact zero is the Q form (G4b).
		// See D-012.
		normal := closest2.Sub(closest1)
		if zero.Less(normal.Dot(normal)) {
			normal = normal.Normalize()
		} else {
			normal = LeftPerp(u1)
		}

		c1 := MulAdd(closest1, radiusA, normal)
		c2 := MulAdd(closest2, radiusB.Neg(), normal)

		i1, i2 := 1, 1
		if f1.Eq(zero) {
			i1 = 0
		}
		if f2.Eq(zero) {
			i2 = 0
		}

		manifold.Normal = normal
		manifold.Points[0].AnchorA = Lerp(c1, c2, half)
		manifold.Points[0].Separation = distanceSquared.Sqrt().Sub(radius)
		manifold.Points[0].Id = makeId(i1, i2)
		manifold.PointCount = 1
	}

	// Convert the manifold to world space.
	manifold.Normal = RotateVector(xfA.Q, manifold.Normal)
	for i := range manifold.PointCount {
		mp := &manifold.Points[i]

		// Anchor points relative to the shape origin in world space.
		mp.AnchorA = RotateVector(xfA.Q, mp.AnchorA.Add(origin))
		mp.AnchorB = mp.AnchorA.Add(xfA.P.Sub(xfB.P))
		mp.Point = xfA.P.Add(mp.AnchorA)
	}

	return manifold
}

// CollideSegmentAndCapsule computes the contact manifold of a segment and a
// capsule. It corresponds to b2CollideSegmentAndCapsule in src/manifold.c.
func CollideSegmentAndCapsule(segmentA *Segment, xfA Transform, capsuleB *Capsule, xfB Transform) Manifold {
	capsuleA := Capsule{Center1: segmentA.Point1, Center2: segmentA.Point2}
	return CollideCapsules(&capsuleA, xfA, capsuleB, xfB)
}

// CollideSegmentAndCircle computes the contact manifold of a segment and a
// circle. It corresponds to b2CollideSegmentAndCircle in src/manifold.c.
func CollideSegmentAndCircle(segmentA *Segment, xfA Transform, circleB *Circle, xfB Transform) Manifold {
	capsuleA := Capsule{Center1: segmentA.Point1, Center2: segmentA.Point2}
	return CollideCapsuleAndCircle(&capsuleA, xfA, circleB, xfB)
}

// CollideChainSegmentAndCircle computes the contact manifold of a one-sided
// chain segment and a circle. It corresponds to
// b2CollideChainSegmentAndCircle in src/manifold.c.
func CollideChainSegmentAndCircle(segmentA *ChainSegment, xfA Transform, circleB *Circle, xfB Transform) Manifold {
	var manifold Manifold
	zero := fixed.Zero()

	xf := InvMulTransforms(xfA, xfB)

	// Compute the circle in the frame of the segment.
	pB := TransformPoint(xf, circleB.Center)

	p1 := segmentA.Segment.Point1
	p2 := segmentA.Segment.Point2
	e := p2.Sub(p1)

	// The normal points to the right; the collision is one-sided.
	offset := RightPerp(e).Dot(pB.Sub(p1))
	if offset.Less(zero) {
		return manifold
	}

	// Barycentric coordinates.
	u := e.Dot(p2.Sub(pB))
	v := e.Dot(pB.Sub(p1))

	var pA Vec2

	if v.Cmp(zero) <= 0 {
		// Behind point1. Is pB in the Voronoi region of the previous edge?
		prevEdge := p1.Sub(segmentA.Ghost1)
		uPrev := prevEdge.Dot(pB.Sub(p1))
		if uPrev.Cmp(zero) <= 0 {
			return manifold
		}

		pA = p1
	} else if u.Cmp(zero) <= 0 {
		// Ahead of point2. Is pB in the Voronoi region of the next edge?
		nextEdge := segmentA.Ghost2.Sub(p2)
		vNext := nextEdge.Dot(pB.Sub(p2))
		if zero.Less(vNext) {
			return manifold
		}

		pA = p2
	} else {
		// The reference multiplies by the reciprocal of e dot e; the port
		// divides per D-006.
		ee := e.Dot(e)
		if zero.Less(ee) {
			pA = p1.Mul(u).Add(p2.Mul(v)).Div(ee)
		} else {
			pA = p1
		}
	}

	distance, normal := GetLengthAndNormalize(pB.Sub(pA))

	radius := circleB.Radius
	separation := distance.Sub(radius)
	if SpeculativeDistance().Less(separation) {
		return manifold
	}

	cA := pA
	cB := MulAdd(pB, radius.Neg(), normal)
	contactPointA := Lerp(cA, cB, fixed.Half())

	manifold.Normal = RotateVector(xfA.Q, normal)

	mp := &manifold.Points[0]
	mp.AnchorA = RotateVector(xfA.Q, contactPointA)
	mp.AnchorB = mp.AnchorA.Add(xfA.P.Sub(xfB.P))
	mp.Point = xfA.P.Add(mp.AnchorA)
	mp.Separation = separation
	mp.Id = 0
	manifold.PointCount = 1
	return manifold
}
