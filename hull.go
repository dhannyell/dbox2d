package dbox2d

import "github.com/dhannyell/fixed"

// recurseHull is the quickhull recursion. It returns the hull of the points
// that lie to the right of the edge p1-p2, excluding p1 and p2.
func recurseHull(p1, p2 Vec2, ps []Vec2) Hull {
	hull := Hull{}

	count := len(ps)
	if count == 0 {
		return hull
	}

	// create an edge vector pointing from p1 to p2
	e := p2.Sub(p1).Normalize()

	// discard points left of e and find point furthest to the right of e
	var rightPoints [MaxPolygonVertices]Vec2
	rightCount := 0

	zero := fixed.Zero()

	bestIndex := 0
	bestDistance := Cross(ps[bestIndex].Sub(p1), e)
	if zero.Less(bestDistance) {
		rightPoints[rightCount] = ps[bestIndex]
		rightCount++
	}

	for i := 1; i < count; i++ {
		distance := Cross(ps[i].Sub(p1), e)
		if bestDistance.Less(distance) {
			bestIndex = i
			bestDistance = distance
		}

		if zero.Less(distance) {
			rightPoints[rightCount] = ps[i]
			rightCount++
		}
	}

	if bestDistance.Less(fixed.FromInt(2).Mul(linearSlop)) {
		return hull
	}

	bestPoint := ps[bestIndex]

	// compute hull to the right of p1-bestPoint
	hull1 := recurseHull(p1, bestPoint, rightPoints[:rightCount])

	// compute hull to the right of bestPoint-p2
	hull2 := recurseHull(bestPoint, p2, rightPoints[:rightCount])

	// stitch together hulls
	for i := range hull1.Count {
		hull.Points[hull.Count] = hull1.Points[i]
		hull.Count++
	}

	hull.Points[hull.Count] = bestPoint
	hull.Count++

	for i := range hull2.Count {
		hull.Points[hull.Count] = hull2.Points[i]
		hull.Count++
	}

	if hull.Count >= MaxPolygonVertices {
		panic("dbox2d: the quickhull recursion overflowed the vertex limit")
	}

	return hull
}

// ComputeHull returns the convex hull of a set of points. It welds points
// that are closer than four linear slops and it drops collinear points. It
// returns an empty hull on failure: fewer than three points, more than
// MaxPolygonVertices points, points all very close, or points all on a line.
func ComputeHull(points []Vec2) Hull {
	hull := Hull{}

	count := len(points)
	if count < 3 || count > MaxPolygonVertices {
		// check your data
		return hull
	}

	count = min(count, MaxPolygonVertices)

	aabb := AABB{
		LowerBound: Vec2{X: fixed.MaxValue(), Y: fixed.MaxValue()},
		UpperBound: Vec2{X: fixed.MinValue(), Y: fixed.MinValue()},
	}

	// Perform aggressive point welding. First point always remains.
	// Also compute the bounding box for later.
	var ps [MaxPolygonVertices]Vec2
	n := 0
	tolSqr := fixed.FromInt(16).Mul(linearSlop).Mul(linearSlop)
	for i := range count {
		aabb.LowerBound = Min(aabb.LowerBound, points[i])
		aabb.UpperBound = Max(aabb.UpperBound, points[i])

		vi := points[i]

		unique := true
		for j := range i {
			vj := points[j]

			distSqr := vi.DistanceSq(vj)
			if distSqr.Less(tolSqr) {
				unique = false
				break
			}
		}

		if unique {
			ps[n] = vi
			n++
		}
	}

	if n < 3 {
		// all points very close together, check your data and check your scale
		return hull
	}

	// Find an extreme point as the first point on the hull
	c := AABBCenter(aabb)
	f1 := 0
	dsq1 := c.DistanceSq(ps[f1])
	for i := 1; i < n; i++ {
		dsq := c.DistanceSq(ps[i])
		if dsq1.Less(dsq) {
			f1 = i
			dsq1 = dsq
		}
	}

	// remove p1 from working set
	p1 := ps[f1]
	ps[f1] = ps[n-1]
	n = n - 1

	f2 := 0
	dsq2 := p1.DistanceSq(ps[f2])
	for i := 1; i < n; i++ {
		dsq := p1.DistanceSq(ps[i])
		if dsq2.Less(dsq) {
			f2 = i
			dsq2 = dsq
		}
	}

	// remove p2 from working set
	p2 := ps[f2]
	ps[f2] = ps[n-1]
	n = n - 1

	// split the points into points that are left and right of the line p1-p2.
	var rightPoints [MaxPolygonVertices - 2]Vec2
	rightCount := 0

	var leftPoints [MaxPolygonVertices - 2]Vec2
	leftCount := 0

	e := p2.Sub(p1).Normalize()

	twoSlops := fixed.FromInt(2).Mul(linearSlop)

	for i := range n {
		d := Cross(ps[i].Sub(p1), e)

		// slop used here to skip points that are very close to the line p1-p2
		if !d.Less(twoSlops) {
			rightPoints[rightCount] = ps[i]
			rightCount++
		} else if !twoSlops.Neg().Less(d) {
			leftPoints[leftCount] = ps[i]
			leftCount++
		}
	}

	// compute hulls on right and left
	hull1 := recurseHull(p1, p2, rightPoints[:rightCount])
	hull2 := recurseHull(p2, p1, leftPoints[:leftCount])

	if hull1.Count == 0 && hull2.Count == 0 {
		// all points collinear
		return hull
	}

	// stitch hulls together, preserving CCW winding order
	hull.Points[hull.Count] = p1
	hull.Count++

	for i := range hull1.Count {
		hull.Points[hull.Count] = hull1.Points[i]
		hull.Count++
	}

	hull.Points[hull.Count] = p2
	hull.Count++

	for i := range hull2.Count {
		hull.Points[hull.Count] = hull2.Points[i]
		hull.Count++
	}

	if hull.Count > MaxPolygonVertices {
		panic("dbox2d: the hull overflowed the vertex limit")
	}

	// merge collinear
	searching := true
	for searching && hull.Count > 2 {
		searching = false

		for i := range hull.Count {
			i1 := i
			i2 := (i + 1) % hull.Count
			i3 := (i + 2) % hull.Count

			s1 := hull.Points[i1]
			s2 := hull.Points[i2]
			s3 := hull.Points[i3]

			// unit edge vector for s1-s3
			r := s3.Sub(s1).Normalize()

			distance := Cross(s2.Sub(s1), r)
			if !twoSlops.Less(distance) {
				// remove midpoint from hull
				for j := i2; j < hull.Count-1; j++ {
					hull.Points[j] = hull.Points[j+1]
				}
				hull.Count -= 1

				// continue searching for collinear points
				searching = true

				break
			}
		}
	}

	if hull.Count < 3 {
		// all points collinear, which the check above should have caught
		hull.Count = 0
	}

	return hull
}

// ValidateHull reports whether a hull is convex and free of collinear
// points. It is expensive, so it does not belong in a running simulation.
func ValidateHull(hull *Hull) bool {
	if hull.Count < 3 || MaxPolygonVertices < hull.Count {
		return false
	}

	zero := fixed.Zero()

	// test that every point is behind every edge
	for i := range hull.Count {
		// create an edge vector
		i1 := i
		i2 := 0
		if i < hull.Count-1 {
			i2 = i1 + 1
		}
		p := hull.Points[i1]
		e := hull.Points[i2].Sub(p).Normalize()

		for j := range hull.Count {
			// skip points that subtend the current edge
			if j == i1 || j == i2 {
				continue
			}

			distance := Cross(hull.Points[j].Sub(p), e)
			if !distance.Less(zero) {
				return false
			}
		}
	}

	// test for collinear points
	for i := range hull.Count {
		i1 := i
		i2 := (i + 1) % hull.Count
		i3 := (i + 2) % hull.Count

		p1 := hull.Points[i1]
		p2 := hull.Points[i2]
		p3 := hull.Points[i3]

		e := p3.Sub(p1).Normalize()

		distance := Cross(p2.Sub(p1), e)
		if !linearSlop.Less(distance) {
			// p1-p2-p3 are collinear
			return false
		}
	}

	return true
}
