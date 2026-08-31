package dbox2d

import "github.com/dhannyell/fixed"

// MaxPolygonVertices is the vertex limit of a convex polygon. Raising it
// costs performance even for shapes that use fewer vertices.
const MaxPolygonVertices = 8

// RayCastInput is the input of a ray cast in the local frame of a shape.
type RayCastInput struct {
	// Origin is the start point of the ray.
	Origin Vec2

	// Translation is the displacement of the ray.
	Translation Vec2

	// MaxFraction limits the translation to consider. It is usually one.
	MaxFraction Q
}

// ShapeProxy is a point cloud with an external radius. It stands for any
// shape in the distance and cast routines.
type ShapeProxy struct {
	// Points holds the cloud. Only the first Count entries are valid.
	Points [MaxPolygonVertices]Vec2

	// Count is the number of points. It is greater than zero.
	Count int

	// Radius is the external radius of the cloud. It may be zero.
	Radius Q
}

// ShapeCastInput is the input of a shape cast in generic form.
type ShapeCastInput struct {
	Proxy       ShapeProxy
	Translation Vec2

	// MaxFraction limits the translation to consider. It is usually one.
	MaxFraction Q

	// CanEncroach lets a cast that starts in contact advance. It applies
	// only when the radius is greater than zero.
	CanEncroach bool
}

// CastOutput is the output of a ray cast or of a shape cast. An initial
// overlap returns a zero fraction and a zero normal.
type CastOutput struct {
	// Normal is the surface normal at the hit point.
	Normal Vec2

	// Point is the surface hit point.
	Point Vec2

	// Fraction is the part of the input translation at the collision.
	Fraction Q

	// Iterations is the number of iterations the cast used.
	Iterations int

	// Hit reports whether the cast hit.
	Hit bool
}

// MassData holds the mass properties of a shape.
type MassData struct {
	// Mass is the mass of the shape, usually in kilograms.
	Mass Q

	// Center is the centroid of the shape, relative to the shape origin.
	Center Vec2

	// RotationalInertia is measured about the local origin.
	RotationalInertia Q
}

// Circle is a solid circle.
type Circle struct {
	Center Vec2
	Radius Q
}

// Capsule is a solid capsule: two semicircles joined by a rectangle.
type Capsule struct {
	// Center1 is the local center of the first semicircle.
	Center1 Vec2

	// Center2 is the local center of the second semicircle.
	Center2 Vec2

	// Radius is the radius of both semicircles.
	Radius Q
}

// Polygon is a solid convex polygon. The interior lies to the left of every
// edge. Build one with MakePolygon or MakeBox; never fill the fields by hand.
type Polygon struct {
	// Vertices holds the corners. Only the first Count entries are valid.
	Vertices [MaxPolygonVertices]Vec2

	// Normals holds the outward normal of each side.
	Normals [MaxPolygonVertices]Vec2

	// Centroid is the area centroid of the polygon.
	Centroid Vec2

	// Radius is the external radius of a rounded polygon.
	Radius Q

	// Count is the number of vertices.
	Count int
}

// Segment is a line segment with two-sided collision.
type Segment struct {
	Point1 Vec2
	Point2 Vec2
}

// ChainSegment is a line segment with one-sided collision. It collides only
// on the right side. A chain shape generates several of them, in the order
// ghost1, point1, point2, ghost2.
type ChainSegment struct {
	// Ghost1 is the tail ghost vertex.
	Ghost1 Vec2

	// Segment is the colliding part.
	Segment Segment

	// Ghost2 is the head ghost vertex.
	Ghost2 Vec2

	// ChainId is the owning chain shape. It is internal bookkeeping.
	ChainId int
}

// Hull is a convex hull. It feeds the polygon constructors. Build one with
// ComputeHull and do not modify it afterwards.
type Hull struct {
	// Points holds the hull corners. Only the first Count entries are valid.
	Points [MaxPolygonVertices]Vec2

	// Count is the number of points.
	Count int
}

// ManifoldPoint is one contact point of a contact manifold. The solver uses
// speculative collision, so a point may still be separated.
type ManifoldPoint struct {
	// Point is the contact point in world space. Use it for debugging only,
	// because it loses precision far from the origin.
	Point Vec2

	// AnchorA is the contact point relative to the origin of shape A, in
	// world space. Inside the solver it is relative to the center of mass.
	AnchorA Vec2

	// AnchorB is the contact point relative to the origin of shape B, in
	// world space. Inside the solver it is relative to the center of mass.
	AnchorB Vec2

	// Separation is negative when the shapes penetrate.
	Separation Q

	// NormalImpulse is the impulse along the manifold normal.
	NormalImpulse Q

	// TangentImpulse is the friction impulse.
	TangentImpulse Q

	// TotalNormalImpulse accumulates over the substeps and the restitution
	// pass. It tells a speculative point that acted from one that did not.
	TotalNormalImpulse Q

	// NormalVelocity is the relative normal velocity before the solve. It
	// feeds the hit events. A negative value means the shapes approach.
	NormalVelocity Q

	// Id identifies the contact point between the two shapes.
	Id uint16

	// Persisted reports whether the point existed in the previous step.
	Persisted bool
}

// Manifold describes the contact points between two colliding shapes. The
// solver uses speculative collision, so a point may still be separated.
type Manifold struct {
	// Normal is the unit normal in world space. It points from A to B.
	Normal Vec2

	// RollingImpulse is the angular impulse of the rolling resistance.
	RollingImpulse Q

	// Points holds the contact points. Two are possible in 2D.
	Points [2]ManifoldPoint

	// PointCount is zero, one or two.
	PointCount int
}

// AABBRayCast casts the segment p1-p2 against the box a. It ignores any
// radius. From Real-time Collision Detection, page 179.
func AABBRayCast(a AABB, p1, p2 Vec2) CastOutput {
	output := CastOutput{}

	zero := fixed.Zero()
	one := fixed.One()

	tmin := fixed.MinValue()
	tmax := fixed.MaxValue()

	p := p1
	d := p2.Sub(p1)
	absD := Abs(d)

	normal := Vec2Zero()

	// x-coordinate
	if absD.X.Eq(zero) {
		// parallel
		if p.X.Less(a.LowerBound.X) || a.UpperBound.X.Less(p.X) {
			return output
		}
	} else {
		t1 := a.LowerBound.X.Sub(p.X).Div(d.X)
		t2 := a.UpperBound.X.Sub(p.X).Div(d.X)

		// Sign of the normal vector.
		s := one.Neg()

		if t2.Less(t1) {
			t1, t2 = t2, t1
			s = one
		}

		// Push the min up
		if tmin.Less(t1) {
			normal.Y = zero
			normal.X = s
			tmin = t1
		}

		// Pull the max down
		tmax = tmax.Min(t2)

		if tmax.Less(tmin) {
			return output
		}
	}

	// y-coordinate
	if absD.Y.Eq(zero) {
		// parallel
		if p.Y.Less(a.LowerBound.Y) || a.UpperBound.Y.Less(p.Y) {
			return output
		}
	} else {
		t1 := a.LowerBound.Y.Sub(p.Y).Div(d.Y)
		t2 := a.UpperBound.Y.Sub(p.Y).Div(d.Y)

		// Sign of the normal vector.
		s := one.Neg()

		if t2.Less(t1) {
			t1, t2 = t2, t1
			s = one
		}

		// Push the min up
		if tmin.Less(t1) {
			normal.X = zero
			normal.Y = s
			tmin = t1
		}

		// Pull the max down
		tmax = tmax.Min(t2)

		if tmax.Less(tmin) {
			return output
		}
	}

	// Does the ray start inside the box?
	// Does the ray intersect beyond the max fraction?
	if tmin.Less(zero) || one.Less(tmin) {
		return output
	}

	output.Fraction = tmin
	output.Normal = normal
	output.Point = Lerp(p1, p2, tmin)
	output.Hit = true
	return output
}
