package dbox2d

import "github.com/dhannyell/fixed"

// shape is the internal record of a shape on a body.
type shape struct {
	id          int
	bodyId      int
	prevShapeId int
	nextShapeId int
	sensorIndex int
	shapeType   ShapeType

	density           Q
	friction          Q
	restitution       Q
	rollingResistance Q
	tangentSpeed      Q
	userMaterialId    int

	aabb          AABB
	fatAABB       AABB
	localCentroid Vec2
	proxyKey      int

	filter      Filter
	userData    any
	customColor uint32

	// The reference overlays the five geometries in a union. Go has no
	// union, so the shape carries all five; shapeType selects the live one.
	capsule      Capsule
	circle       Circle
	polygon      Polygon
	segment      Segment
	chainSegment ChainSegment

	// generation advances on each allocation of this slot, so a stale
	// ShapeId fails validation.
	generation uint16

	enableSensorEvents   bool
	enableContactEvents  bool
	enableHitEvents      bool
	enablePreSolveEvents bool
	enlargedAABB         bool
}

// shapeExtent holds the distances from a reference point to the nearest and
// farthest surface of a shape.
type shapeExtent struct {
	minExtent Q
	maxExtent Q
}

// getShape returns a validated shape from an id. It panics on a stale id.
func getShape(w *world, shapeId ShapeId) *shape {
	id := int(shapeId.index1) - 1
	s := &w.shapes[id]
	if s.id != id || s.generation != shapeId.generation {
		panic("dbox2d: invalid ShapeId")
	}
	return s
}

// updateShapeAABBs refreshes the tight and the fat bounds of a shape.
func updateShapeAABBs(s *shape, transform Transform, proxyType BodyType) {
	// Compute a bounding box with a speculative margin
	aabb := computeShapeAABB(s, transform)
	aabb.LowerBound.X = aabb.LowerBound.X.Sub(speculativeDistance)
	aabb.LowerBound.Y = aabb.LowerBound.Y.Sub(speculativeDistance)
	aabb.UpperBound.X = aabb.UpperBound.X.Add(speculativeDistance)
	aabb.UpperBound.Y = aabb.UpperBound.Y.Add(speculativeDistance)
	s.aabb = aabb

	// Smaller margin for static bodies. Cannot be zero due to TOI tolerance.
	margin := aabbMargin
	if proxyType == StaticBody {
		margin = speculativeDistance
	}
	s.fatAABB = AABB{
		LowerBound: Vec2{X: aabb.LowerBound.X.Sub(margin), Y: aabb.LowerBound.Y.Sub(margin)},
		UpperBound: Vec2{X: aabb.UpperBound.X.Add(margin), Y: aabb.UpperBound.Y.Add(margin)},
	}
}

// createShapeInternal allocates the shape record and links it to the body.
func createShapeInternal(w *world, b *body, transform Transform, def *ShapeDef, geometry any, shapeType ShapeType) *shape {
	shapeId := w.shapeIdPool.allocId()

	if shapeId == len(w.shapes) {
		w.shapes = append(w.shapes, shape{})
	} else if w.shapes[shapeId].id != nullIndex {
		panic("dbox2d: the shape slot is still in use")
	}

	s := &w.shapes[shapeId]

	switch shapeType {
	case CapsuleShape:
		s.capsule = *geometry.(*Capsule)
	case CircleShape:
		s.circle = *geometry.(*Circle)
	case PolygonShape:
		s.polygon = *geometry.(*Polygon)
	case SegmentShape:
		s.segment = *geometry.(*Segment)
	case ChainSegmentShape:
		s.chainSegment = *geometry.(*ChainSegment)
	default:
		panic("dbox2d: unknown shape type")
	}

	s.id = shapeId
	s.bodyId = b.id
	s.shapeType = shapeType
	s.density = def.Density
	s.friction = def.Material.Friction
	s.restitution = def.Material.Restitution
	s.rollingResistance = def.Material.RollingResistance
	s.tangentSpeed = def.Material.TangentSpeed
	s.userMaterialId = def.Material.UserMaterialId
	s.filter = def.Filter
	s.userData = def.UserData
	s.customColor = def.Material.CustomColor
	s.enlargedAABB = false
	s.enableSensorEvents = def.EnableSensorEvents
	s.enableContactEvents = def.EnableContactEvents
	s.enableHitEvents = def.EnableHitEvents
	s.enablePreSolveEvents = def.EnablePreSolveEvents
	s.proxyKey = nullIndex
	s.localCentroid = getShapeCentroid(s)
	s.aabb = AABB{}
	s.fatAABB = AABB{}
	s.generation += 1

	if b.setIndex != disabledSet {
		proxyType := b.bodyType
		createShapeProxy(s, &w.broadPhase, proxyType, transform, def.InvokeContactCreation || def.IsSensor)
	}

	// Add to shape doubly linked list
	if b.headShapeId != nullIndex {
		headShape := &w.shapes[b.headShapeId]
		headShape.prevShapeId = shapeId
	}

	s.prevShapeId = nullIndex
	s.nextShapeId = b.headShapeId
	b.headShapeId = shapeId
	b.shapeCount += 1

	// Deferred: the sensor record of the reference sits here.
	s.sensorIndex = nullIndex

	return s
}

// createShape validates the definition and dispatches on the geometry.
func createShape(bodyId BodyId, def *ShapeDef, geometry any, shapeType ShapeType) ShapeId {
	checkDef(def.internalValue)
	zero := fixed.Q32Zero()
	if !IsValidQ(def.Density) || def.Density.Less(zero) {
		panic("dbox2d: ShapeDef.Density is not valid")
	}
	if !IsValidQ(def.Material.Friction) || def.Material.Friction.Less(zero) {
		panic("dbox2d: ShapeDef.Material.Friction is not valid")
	}
	if !IsValidQ(def.Material.Restitution) || def.Material.Restitution.Less(zero) {
		panic("dbox2d: ShapeDef.Material.Restitution is not valid")
	}
	if !IsValidQ(def.Material.RollingResistance) || def.Material.RollingResistance.Less(zero) {
		panic("dbox2d: ShapeDef.Material.RollingResistance is not valid")
	}
	if !IsValidQ(def.Material.TangentSpeed) {
		panic("dbox2d: ShapeDef.Material.TangentSpeed is not valid")
	}

	// Deferred: the sensor module. The rejection comes before the first
	// mutation, so a recovered panic leaves no orphan shape.
	if def.IsSensor {
		panic("dbox2d: a sensor shape is not supported yet")
	}

	w := getWorldLocked(bodyId.world0)

	b := getBodyFullId(w, bodyId)
	transform := getBodyTransformQuick(w, b)

	s := createShapeInternal(w, b, transform, def, geometry, shapeType)

	if def.UpdateBodyMass {
		updateBodyMassData(w, b)
	}

	return ShapeId{index1: int32(s.id) + 1, world0: bodyId.world0, generation: s.generation}
}

// CreateCircleShape creates a circle shape on a body. The definition must
// come from DefaultShapeDef and is not retained.
func CreateCircleShape(bodyId BodyId, def *ShapeDef, circle *Circle) ShapeId {
	return createShape(bodyId, def, circle, CircleShape)
}

// CreateCapsuleShape creates a capsule shape on a body. A capsule shorter
// than the linear slop becomes a circle at its midpoint.
func CreateCapsuleShape(bodyId BodyId, def *ShapeDef, capsule *Capsule) ShapeId {
	lengthSqr := capsule.Center1.DistanceSq(capsule.Center2)
	if !linearSlop.Mul(linearSlop).Less(lengthSqr) {
		circle := Circle{Center: Lerp(capsule.Center1, capsule.Center2, fixed.Q32Half()), Radius: capsule.Radius}
		return createShape(bodyId, def, &circle, CircleShape)
	}

	return createShape(bodyId, def, capsule, CapsuleShape)
}

// CreatePolygonShape creates a polygon shape on a body. Build the polygon
// with MakePolygon or MakeBox.
func CreatePolygonShape(bodyId BodyId, def *ShapeDef, polygon *Polygon) ShapeId {
	if !IsValidQ(polygon.Radius) || polygon.Radius.Less(fixed.Q32Zero()) {
		panic("dbox2d: Polygon.Radius is not valid")
	}
	return createShape(bodyId, def, polygon, PolygonShape)
}

// CreateSegmentShape creates a segment shape on a body. It panics on a
// segment shorter than the linear slop.
func CreateSegmentShape(bodyId BodyId, def *ShapeDef, segment *Segment) ShapeId {
	lengthSqr := segment.Point1.DistanceSq(segment.Point2)
	if !linearSlop.Mul(linearSlop).Less(lengthSqr) {
		panic("dbox2d: the segment is degenerate")
	}

	return createShape(bodyId, def, segment, SegmentShape)
}

// destroyShapeInternal unlinks the shape, destroys its contacts and frees its
// id. DestroyBody removes all contacts before walking the shape list itself.
func destroyShapeInternal(w *world, s *shape, b *body) {
	shapeId := s.id

	// Remove the shape from the doubly linked list of the body.
	if s.prevShapeId != nullIndex {
		prevShape := &w.shapes[s.prevShapeId]
		prevShape.nextShapeId = s.nextShapeId
	}

	if s.nextShapeId != nullIndex {
		nextShape := &w.shapes[s.nextShapeId]
		nextShape.prevShapeId = s.prevShapeId
	}

	if shapeId == b.headShapeId {
		b.headShapeId = s.nextShapeId
	}

	b.shapeCount -= 1

	// Remove from broad-phase
	destroyShapeProxy(s, &w.broadPhase)

	// Deferred: the sensor record of the reference goes away here.

	// Destroy contacts before releasing the shape id. The next key is read
	// first because destroyContact unlinks the current edge from this body.
	contactKey := b.headContactKey
	for contactKey != nullIndex {
		contactId := contactKey >> 1
		edgeIndex := contactKey & 1
		c := &w.contacts[contactId]
		contactKey = c.edges[edgeIndex].nextKey

		if c.shapeIdA == shapeId || c.shapeIdB == shapeId {
			destroyContact(w, c, true)
		}
	}

	// Return shape to free list.
	w.shapeIdPool.freeId(shapeId)
	s.id = nullIndex
}

// DestroyShape destroys a shape. Pass updateBodyMass true to recompute the
// mass of the body from the remaining shapes.
func DestroyShape(shapeId ShapeId, updateBodyMass bool) {
	w := getWorldLocked(shapeId.world0)

	s := getShape(w, shapeId)
	b := &w.bodies[s.bodyId]

	destroyShapeInternal(w, s, b)

	if updateBodyMass {
		updateBodyMassData(w, b)
	}
}

// createShapeProxy refreshes the bounds of a shape and inserts it in the
// broadphase. It corresponds to b2CreateShapeProxy in src/shape.c.
func createShapeProxy(s *shape, bp *broadPhase, proxyType BodyType, transform Transform, forcePairCreation bool) {
	if s.proxyKey != nullIndex {
		panic("dbox2d: the shape already has a proxy")
	}

	updateShapeAABBs(s, transform, proxyType)

	// Create proxies in the broad-phase.
	s.proxyKey = bp.createProxy(proxyType, s.fatAABB, s.filter.CategoryBits, s.id, forcePairCreation)
	if proxyTypeOf(s.proxyKey) >= BodyTypeCount {
		panic("dbox2d: the proxy key carries a bad type")
	}
}

// destroyShapeProxy removes a shape from the broadphase, if it has a
// proxy. It corresponds to b2DestroyShapeProxy in src/shape.c.
func destroyShapeProxy(s *shape, bp *broadPhase) {
	if s.proxyKey != nullIndex {
		bp.destroyProxy(s.proxyKey)
		s.proxyKey = nullIndex
	}
}

// computeShapeAABB returns the bounds of a shape under a transform.
func computeShapeAABB(s *shape, xf Transform) AABB {
	switch s.shapeType {
	case CapsuleShape:
		return ComputeCapsuleAABB(&s.capsule, xf)
	case CircleShape:
		return ComputeCircleAABB(&s.circle, xf)
	case PolygonShape:
		return ComputePolygonAABB(&s.polygon, xf)
	case SegmentShape:
		return ComputeSegmentAABB(&s.segment, xf)
	case ChainSegmentShape:
		return ComputeSegmentAABB(&s.chainSegment.Segment, xf)
	default:
		panic("dbox2d: unknown shape type")
	}
}

// getShapeCentroid returns the centroid of a shape in local coordinates.
func getShapeCentroid(s *shape) Vec2 {
	switch s.shapeType {
	case CapsuleShape:
		return Lerp(s.capsule.Center1, s.capsule.Center2, fixed.Q32Half())
	case CircleShape:
		return s.circle.Center
	case PolygonShape:
		return s.polygon.Centroid
	case SegmentShape:
		return Lerp(s.segment.Point1, s.segment.Point2, fixed.Q32Half())
	case ChainSegmentShape:
		return Lerp(s.chainSegment.Segment.Point1, s.chainSegment.Segment.Point2, fixed.Q32Half())
	default:
		return Vec2Zero()
	}
}

// getShapeRadius returns the round radius of a shape. Segments have no
// radius. It corresponds to b2GetShapeRadius in src/shape.h.
func getShapeRadius(s *shape) Q {
	switch s.shapeType {
	case CapsuleShape:
		return s.capsule.Radius
	case CircleShape:
		return s.circle.Radius
	case PolygonShape:
		return s.polygon.Radius
	default:
		return fixed.Q32Zero()
	}
}

// computeShapeMass returns the mass data of a shape. Segments have no area,
// so they return zero mass.
func computeShapeMass(s *shape) MassData {
	switch s.shapeType {
	case CapsuleShape:
		return ComputeCapsuleMass(&s.capsule, s.density)
	case CircleShape:
		return ComputeCircleMass(&s.circle, s.density)
	case PolygonShape:
		return ComputePolygonMass(&s.polygon, s.density)
	default:
		return MassData{}
	}
}

// computeShapeExtent returns the extents of a shape about a local reference
// point.
func computeShapeExtent(s *shape, localCenter Vec2) shapeExtent {
	extent := shapeExtent{}

	switch s.shapeType {
	case CapsuleShape:
		radius := s.capsule.Radius
		extent.minExtent = radius
		c1 := s.capsule.Center1.Sub(localCenter)
		c2 := s.capsule.Center2.Sub(localCenter)
		extent.maxExtent = c1.LenSq().Max(c2.LenSq()).Sqrt().Add(radius)

	case CircleShape:
		radius := s.circle.Radius
		extent.minExtent = radius
		extent.maxExtent = s.circle.Center.Sub(localCenter).Len().Add(radius)

	case PolygonShape:
		poly := &s.polygon
		minExtent := huge
		maxExtentSqr := fixed.Q32Zero()
		for i := range poly.Count {
			v := poly.Vertices[i]
			planeOffset := poly.Normals[i].Dot(v.Sub(poly.Centroid))
			minExtent = minExtent.Min(planeOffset)

			distanceSqr := v.Sub(localCenter).LenSq()
			maxExtentSqr = maxExtentSqr.Max(distanceSqr)
		}

		extent.minExtent = minExtent.Add(poly.Radius)
		extent.maxExtent = maxExtentSqr.Sqrt().Add(poly.Radius)

	case SegmentShape:
		extent.minExtent = fixed.Q32Zero()
		c1 := s.segment.Point1.Sub(localCenter)
		c2 := s.segment.Point2.Sub(localCenter)
		extent.maxExtent = c1.LenSq().Max(c2.LenSq()).Sqrt()

	case ChainSegmentShape:
		extent.minExtent = fixed.Q32Zero()
		c1 := s.chainSegment.Segment.Point1.Sub(localCenter)
		c2 := s.chainSegment.Segment.Point2.Sub(localCenter)
		extent.maxExtent = c1.LenSq().Max(c2.LenSq()).Sqrt()
	}

	return extent
}

// shapeIdOf builds the public id of a shape.
func shapeIdOf(w *world, s *shape) ShapeId {
	return ShapeId{index1: int32(s.id) + 1, world0: w.worldId, generation: s.generation}
}

// shouldShapesCollide applies the filter rule of the reference: a shared
// nonzero group decides by its sign, else the masks decide. It corresponds
// to b2ShouldShapesCollide in src/shape.h.
func shouldShapesCollide(filterA, filterB Filter) bool {
	if filterA.GroupIndex == filterB.GroupIndex && filterA.GroupIndex != 0 {
		return filterA.GroupIndex > 0
	}

	return filterA.MaskBits&filterB.CategoryBits != 0 && filterA.CategoryBits&filterB.MaskBits != 0
}

// shouldQueryCollide applies the query filter to a shape filter. It
// corresponds to b2ShouldQueryCollide in src/shape.h.
func shouldQueryCollide(shapeFilter Filter, queryFilter QueryFilter) bool {
	return shapeFilter.CategoryBits&queryFilter.MaskBits != 0 && shapeFilter.MaskBits&queryFilter.CategoryBits != 0
}

// rayCastShape casts a world-space ray against a shape. It corresponds
// to b2RayCastShape in src/shape.c.
func rayCastShape(input *RayCastInput, s *shape, transform Transform) CastOutput {
	localInput := *input
	localInput.Origin = InvTransformPoint(transform, input.Origin)
	localInput.Translation = InvRotateVector(transform.Q, input.Translation)

	var output CastOutput
	switch s.shapeType {
	case CapsuleShape:
		output = RayCastCapsule(&localInput, &s.capsule)
	case CircleShape:
		output = RayCastCircle(&localInput, &s.circle)
	case PolygonShape:
		output = RayCastPolygon(&localInput, &s.polygon)
	case SegmentShape:
		output = RayCastSegment(&localInput, &s.segment, false)
	case ChainSegmentShape:
		output = RayCastSegment(&localInput, &s.chainSegment.Segment, true)
	default:
		return output
	}

	output.Point = TransformPoint(transform, output.Point)
	output.Normal = RotateVector(transform.Q, output.Normal)
	return output
}

// shapeCastShape casts a proxy in world space against a shape at a
// transform. It corresponds to b2ShapeCastShape in src/shape.c.
func shapeCastShape(input *ShapeCastInput, s *shape, transform Transform) CastOutput {
	localInput := *input

	for i := range localInput.Proxy.Count {
		localInput.Proxy.Points[i] = InvTransformPoint(transform, input.Proxy.Points[i])
	}

	localInput.Translation = InvRotateVector(transform.Q, input.Translation)

	var output CastOutput
	switch s.shapeType {
	case CapsuleShape:
		output = ShapeCastCapsule(&localInput, &s.capsule)
	case CircleShape:
		output = ShapeCastCircle(&localInput, &s.circle)
	case PolygonShape:
		output = ShapeCastPolygon(&localInput, &s.polygon)
	case SegmentShape:
		output = ShapeCastSegment(&localInput, &s.segment)
	case ChainSegmentShape:
		output = ShapeCastSegment(&localInput, &s.chainSegment.Segment)
	default:
		return output
	}

	output.Point = TransformPoint(transform, output.Point)
	output.Normal = RotateVector(transform.Q, output.Normal)
	return output
}

// makeShapeDistanceProxy builds the distance proxy of a shape in its body
// frame. It corresponds to b2MakeShapeDistanceProxy in src/shape.c.
func makeShapeDistanceProxy(s *shape) ShapeProxy {
	switch s.shapeType {
	case CapsuleShape:
		return MakeProxy([]Vec2{s.capsule.Center1, s.capsule.Center2}, s.capsule.Radius)
	case CircleShape:
		return MakeProxy([]Vec2{s.circle.Center}, s.circle.Radius)
	case PolygonShape:
		return MakeProxy(s.polygon.Vertices[:s.polygon.Count], s.polygon.Radius)
	case SegmentShape:
		return MakeProxy([]Vec2{s.segment.Point1, s.segment.Point2}, fixed.Q32Zero())
	case ChainSegmentShape:
		return MakeProxy([]Vec2{s.chainSegment.Segment.Point1, s.chainSegment.Segment.Point2}, fixed.Q32Zero())
	default:
		panic("dbox2d: unknown shape type")
	}
}
