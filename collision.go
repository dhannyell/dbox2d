package dbox2d

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

// SegmentDistanceResult is the output of SegmentDistance. It corresponds
// to b2SegmentDistanceResult in include/box2d/collision.h.
type SegmentDistanceResult struct {
	// Closest1 is the closest point on the first segment.
	Closest1 Vec2

	// Closest2 is the closest point on the second segment.
	Closest2 Vec2

	// Fraction1 is the barycentric coordinate on the first segment.
	Fraction1 Q

	// Fraction2 is the barycentric coordinate on the second segment.
	Fraction2 Q

	// DistanceSquared is the squared distance between the closest points.
	DistanceSquared Q
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

// SimplexCache warm starts the distance solver between two calls. The zero
// value is an empty cache; ShapeDistance fills it. It corresponds to
// b2SimplexCache in include/box2d/collision.h.
type SimplexCache struct {
	// Count is the number of stored simplex vertices.
	Count uint16

	// IndexA holds the vertices of shape A on the simplex.
	IndexA [3]uint8

	// IndexB holds the vertices of shape B on the simplex.
	IndexB [3]uint8
}

// DistanceInput is the input of ShapeDistance. It corresponds to
// b2DistanceInput in include/box2d/collision.h.
type DistanceInput struct {
	// ProxyA is the proxy of shape A.
	ProxyA ShapeProxy

	// ProxyB is the proxy of shape B.
	ProxyB ShapeProxy

	// TransformA is the world transform of shape A.
	TransformA Transform

	// TransformB is the world transform of shape B.
	TransformB Transform

	// UseRadii asks the solver to subtract the proxy radii.
	UseRadii bool
}

// DistanceOutput is the output of ShapeDistance. It corresponds to
// b2DistanceOutput in include/box2d/collision.h.
type DistanceOutput struct {
	// PointA is the closest point on shape A.
	PointA Vec2

	// PointB is the closest point on shape B.
	PointB Vec2

	// Normal points from A to B. It is invalid when the distance is zero.
	Normal Vec2

	// Distance is the final distance, zero when the shapes overlap.
	Distance Q

	// Iterations counts the GJK iterations used.
	Iterations int

	// SimplexCount is the number of simplexes stored in the debug slice.
	SimplexCount int
}

// SimplexVertex is a vertex of the GJK simplex, kept for debugging. It
// corresponds to b2SimplexVertex in include/box2d/collision.h.
type SimplexVertex struct {
	// WA is the support point in proxy A.
	WA Vec2

	// WB is the support point in proxy B.
	WB Vec2

	// W is wA - wB.
	W Vec2

	// A is the barycentric coordinate of the closest point.
	A Q

	// IndexA is the index of WA.
	IndexA int

	// IndexB is the index of WB.
	IndexB int
}

// Simplex is the simplex of the GJK algorithm. It corresponds to
// b2Simplex in include/box2d/collision.h.
type Simplex struct {
	// V1, V2 and V3 are the vertices.
	V1, V2, V3 SimplexVertex

	// Count is the number of valid vertices.
	Count int
}

// ShapeCastPairInput is the input of ShapeCast. It corresponds to
// b2ShapeCastPairInput in include/box2d/collision.h.
type ShapeCastPairInput struct {
	// ProxyA is the proxy of shape A.
	ProxyA ShapeProxy

	// ProxyB is the proxy of shape B.
	ProxyB ShapeProxy

	// TransformA is the world transform of shape A.
	TransformA Transform

	// TransformB is the world transform of shape B.
	TransformB Transform

	// TranslationB is the translation of shape B.
	TranslationB Vec2

	// MaxFraction is the fraction of the translation to consider. It is
	// usually one.
	MaxFraction Q

	// CanEncroach lets shapes with a radius move a little closer when
	// they already touch.
	CanEncroach bool
}

// Sweep describes the motion of a body for the time of impact. Shapes are
// defined about the body origin, which may differ from the center of mass,
// so the sweep interpolates the center of mass. It corresponds to b2Sweep
// in include/box2d/collision.h.
type Sweep struct {
	// LocalCenter is the local center of mass.
	LocalCenter Vec2

	// C1 is the starting world center of mass.
	C1 Vec2

	// C2 is the ending world center of mass.
	C2 Vec2

	// Q1 is the starting world rotation.
	Q1 Rot

	// Q2 is the ending world rotation.
	Q2 Rot
}

// TOIInput is the input of TimeOfImpact. It corresponds to b2TOIInput in
// include/box2d/collision.h.
type TOIInput struct {
	// ProxyA is the proxy for shape A.
	ProxyA ShapeProxy

	// ProxyB is the proxy for shape B.
	ProxyB ShapeProxy

	// SweepA is the movement of shape A.
	SweepA Sweep

	// SweepB is the movement of shape B.
	SweepB Sweep

	// MaxFraction defines the sweep interval [0, MaxFraction].
	MaxFraction Q
}

// TOIState describes the result of TimeOfImpact. It corresponds to
// b2TOIState in include/box2d/collision.h.
type TOIState int

const (
	// TOIStateUnknown means the solver did not reach a conclusion.
	TOIStateUnknown TOIState = iota

	// TOIStateFailed means the root finder ran out of iterations.
	TOIStateFailed

	// TOIStateOverlapped means the shapes overlap at the start.
	TOIStateOverlapped

	// TOIStateHit means the shapes touch at the fraction.
	TOIStateHit

	// TOIStateSeparated means the shapes stay apart over the whole interval.
	TOIStateSeparated
)

// TOIOutput is the output of TimeOfImpact. It corresponds to b2TOIOutput
// in include/box2d/collision.h.
type TOIOutput struct {
	// State is the type of result.
	State TOIState

	// Fraction is the sweep time of the collision.
	Fraction Q
}

// TreeStats counts the nodes a world query visited. It corresponds to
// b2TreeStats in include/box2d/collision.h.
type TreeStats struct {
	// NodeVisits counts every node the walk touched.
	NodeVisits int

	// LeafVisits counts the leaves the walk tested.
	LeafVisits int
}
