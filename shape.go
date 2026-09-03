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

// GetBody returns the body that owns the shape. It corresponds to
// b2Shape_GetBody.
func (shapeId ShapeId) GetBody() BodyId {
	w := getWorld(shapeId.world0)
	s := getShape(w, shapeId)
	return makeBodyId(w, s.bodyId)
}

// GetWorld returns the world containing the shape. It corresponds to
// b2Shape_GetWorld.
func (shapeId ShapeId) GetWorld() WorldId {
	w := getWorld(shapeId.world0)
	return WorldId{index1: shapeId.world0 + 1, generation: w.generation}
}

// SetUserData attaches application data to the shape. It corresponds to
// b2Shape_SetUserData.
func (shapeId ShapeId) SetUserData(userData any) {
	w := getWorld(shapeId.world0)
	getShape(w, shapeId).userData = userData
}

// GetUserData returns the data attached to the shape. It corresponds to
// b2Shape_GetUserData.
func (shapeId ShapeId) GetUserData() any {
	w := getWorld(shapeId.world0)
	return getShape(w, shapeId).userData
}

// IsSensor reports whether the shape is a sensor. It corresponds to
// b2Shape_IsSensor.
func (shapeId ShapeId) IsSensor() bool {
	w := getWorld(shapeId.world0)
	return getShape(w, shapeId).sensorIndex != nullIndex
}

// TestPoint reports whether a world point lies within the shape. It
// corresponds to b2Shape_TestPoint.
func (shapeId ShapeId) TestPoint(point Vec2) bool {
	w := getWorld(shapeId.world0)
	s := getShape(w, shapeId)
	transform := getBodyTransform(w, s.bodyId)
	localPoint := InvTransformPoint(transform, point)

	switch s.shapeType {
	case CapsuleShape:
		return PointInCapsule(localPoint, &s.capsule)
	case CircleShape:
		return PointInCircle(localPoint, &s.circle)
	case PolygonShape:
		zero := fixed.Q32Zero()
		input := DistanceInput{
			ProxyA:     MakeProxy(s.polygon.Vertices[:s.polygon.Count], zero),
			ProxyB:     MakeProxy([]Vec2{localPoint}, zero),
			TransformA: TransformIdentity(),
			TransformB: TransformIdentity(),
			UseRadii:   false,
		}
		cache := SimplexCache{}
		output := ShapeDistance(&input, &cache, nil)
		return !s.polygon.Radius.Less(output.Distance)
	default:
		return false
	}
}

// RayCast casts a world-space ray against the shape. It corresponds to
// b2Shape_RayCast.
func (shapeId ShapeId) RayCast(input *RayCastInput) CastOutput {
	w := getWorld(shapeId.world0)
	s := getShape(w, shapeId)
	transform := getBodyTransform(w, s.bodyId)
	return rayCastShape(input, s, transform)
}

// SetDensity changes the shape density and optionally updates body mass. It
// corresponds to b2Shape_SetDensity.
func (shapeId ShapeId) SetDensity(density Q, updateBodyMass bool) {
	zero := fixed.Q32Zero()
	if !IsValidQ(density) || density.Less(zero) {
		panic("dbox2d: SetDensity density is not valid")
	}

	w := getWorldLocked(shapeId.world0)
	s := getShape(w, shapeId)
	if density.Eq(s.density) {
		return
	}

	s.density = density
	if updateBodyMass {
		updateBodyMassData(w, &w.bodies[s.bodyId])
	}
}

// GetDensity returns the shape density. It corresponds to
// b2Shape_GetDensity.
func (shapeId ShapeId) GetDensity() Q {
	w := getWorld(shapeId.world0)
	return getShape(w, shapeId).density
}

// SetFriction changes the shape friction. It corresponds to
// b2Shape_SetFriction.
func (shapeId ShapeId) SetFriction(friction Q) {
	zero := fixed.Q32Zero()
	if !IsValidQ(friction) || friction.Less(zero) {
		panic("dbox2d: SetFriction friction is not valid")
	}

	w := getWorldLocked(shapeId.world0)
	getShape(w, shapeId).friction = friction
}

// GetFriction returns the shape friction. It corresponds to
// b2Shape_GetFriction.
func (shapeId ShapeId) GetFriction() Q {
	w := getWorld(shapeId.world0)
	return getShape(w, shapeId).friction
}

// SetRestitution changes the shape restitution. It corresponds to
// b2Shape_SetRestitution.
func (shapeId ShapeId) SetRestitution(restitution Q) {
	zero := fixed.Q32Zero()
	if !IsValidQ(restitution) || restitution.Less(zero) {
		panic("dbox2d: SetRestitution restitution is not valid")
	}

	w := getWorldLocked(shapeId.world0)
	getShape(w, shapeId).restitution = restitution
}

// GetRestitution returns the shape restitution. It corresponds to
// b2Shape_GetRestitution.
func (shapeId ShapeId) GetRestitution() Q {
	w := getWorld(shapeId.world0)
	return getShape(w, shapeId).restitution
}

// SetMaterial sets the shape user material id. It corresponds to
// b2Shape_SetMaterial.
func (shapeId ShapeId) SetMaterial(material int) {
	w := getWorldLocked(shapeId.world0)
	getShape(w, shapeId).userMaterialId = material
}

// GetMaterial returns the shape user material id. It corresponds to
// b2Shape_GetMaterial.
func (shapeId ShapeId) GetMaterial() int {
	w := getWorld(shapeId.world0)
	return getShape(w, shapeId).userMaterialId
}

// GetSurfaceMaterial returns the shape surface material. It corresponds to
// b2Shape_GetSurfaceMaterial.
func (shapeId ShapeId) GetSurfaceMaterial() SurfaceMaterial {
	w := getWorld(shapeId.world0)
	s := getShape(w, shapeId)
	return SurfaceMaterial{
		Friction:          s.friction,
		Restitution:       s.restitution,
		RollingResistance: s.rollingResistance,
		TangentSpeed:      s.tangentSpeed,
		UserMaterialId:    s.userMaterialId,
		CustomColor:       s.customColor,
	}
}

// SetSurfaceMaterial changes the shape surface material. It corresponds to
// b2Shape_SetSurfaceMaterial.
func (shapeId ShapeId) SetSurfaceMaterial(material SurfaceMaterial) {
	w := getWorld(shapeId.world0)
	s := getShape(w, shapeId)
	s.friction = material.Friction
	s.restitution = material.Restitution
	s.rollingResistance = material.RollingResistance
	s.tangentSpeed = material.TangentSpeed
	s.userMaterialId = material.UserMaterialId
	s.customColor = material.CustomColor
}

// GetFilter returns the shape collision filter. It corresponds to
// b2Shape_GetFilter.
func (shapeId ShapeId) GetFilter() Filter {
	w := getWorld(shapeId.world0)
	return getShape(w, shapeId).filter
}

// resetProxy destroys contacts and refreshes a shape proxy after a shape
// change. It corresponds to b2ResetProxy.
func resetProxy(w *world, s *shape, wakeBodies, destroyProxy bool) {
	b := &w.bodies[s.bodyId]

	contactKey := b.headContactKey
	for contactKey != nullIndex {
		contactId := contactKey >> 1
		edgeIndex := contactKey & 1
		c := &w.contacts[contactId]
		contactKey = c.edges[edgeIndex].nextKey

		if c.shapeIdA == s.id || c.shapeIdB == s.id {
			destroyContact(w, c, wakeBodies)
		}
	}

	transform := getBodyTransformQuick(w, b)
	if s.proxyKey != nullIndex {
		proxyType := proxyTypeOf(s.proxyKey)
		updateShapeAABBs(s, transform, proxyType)

		if destroyProxy {
			w.broadPhase.destroyProxy(s.proxyKey)
			s.proxyKey = w.broadPhase.createProxy(proxyType, s.fatAABB, s.filter.CategoryBits, s.id, true)
		} else {
			w.broadPhase.moveProxy(s.proxyKey, s.fatAABB)
		}
	} else {
		updateShapeAABBs(s, transform, b.bodyType)
	}

	validateSolverSets(w)
}

// SetFilter changes the shape collision filter. It corresponds to
// b2Shape_SetFilter.
func (shapeId ShapeId) SetFilter(filter Filter) {
	w := getWorldLocked(shapeId.world0)
	s := getShape(w, shapeId)
	if filter.MaskBits == s.filter.MaskBits &&
		filter.CategoryBits == s.filter.CategoryBits &&
		filter.GroupIndex == s.filter.GroupIndex {
		return
	}

	destroyProxy := filter.CategoryBits != s.filter.CategoryBits
	s.filter = filter
	resetProxy(w, s, true, destroyProxy)
}

// EnableSensorEvents enables sensor events on the shape. It corresponds to
// b2Shape_EnableSensorEvents.
func (shapeId ShapeId) EnableSensorEvents(flag bool) {
	w := getWorldLocked(shapeId.world0)
	getShape(w, shapeId).enableSensorEvents = flag
}

// AreSensorEventsEnabled reports whether sensor events are enabled. It
// corresponds to b2Shape_AreSensorEventsEnabled.
func (shapeId ShapeId) AreSensorEventsEnabled() bool {
	w := getWorld(shapeId.world0)
	return getShape(w, shapeId).enableSensorEvents
}

// EnableContactEvents enables contact events on the shape. It corresponds to
// b2Shape_EnableContactEvents.
func (shapeId ShapeId) EnableContactEvents(flag bool) {
	w := getWorldLocked(shapeId.world0)
	getShape(w, shapeId).enableContactEvents = flag
}

// AreContactEventsEnabled reports whether contact events are enabled. It
// corresponds to b2Shape_AreContactEventsEnabled.
func (shapeId ShapeId) AreContactEventsEnabled() bool {
	w := getWorld(shapeId.world0)
	return getShape(w, shapeId).enableContactEvents
}

// EnablePreSolveEvents enables pre-solve events on the shape. It corresponds
// to b2Shape_EnablePreSolveEvents.
func (shapeId ShapeId) EnablePreSolveEvents(flag bool) {
	w := getWorldLocked(shapeId.world0)
	getShape(w, shapeId).enablePreSolveEvents = flag
}

// ArePreSolveEventsEnabled reports whether pre-solve events are enabled. It
// corresponds to b2Shape_ArePreSolveEventsEnabled.
func (shapeId ShapeId) ArePreSolveEventsEnabled() bool {
	w := getWorld(shapeId.world0)
	return getShape(w, shapeId).enablePreSolveEvents
}

// EnableHitEvents enables hit events on the shape. It corresponds to
// b2Shape_EnableHitEvents.
func (shapeId ShapeId) EnableHitEvents(flag bool) {
	w := getWorldLocked(shapeId.world0)
	getShape(w, shapeId).enableHitEvents = flag
}

// AreHitEventsEnabled reports whether hit events are enabled. It corresponds
// to b2Shape_AreHitEventsEnabled.
func (shapeId ShapeId) AreHitEventsEnabled() bool {
	w := getWorld(shapeId.world0)
	return getShape(w, shapeId).enableHitEvents
}

// GetType returns the shape geometry type. It corresponds to
// b2Shape_GetType.
func (shapeId ShapeId) GetType() ShapeType {
	w := getWorld(shapeId.world0)
	return getShape(w, shapeId).shapeType
}

// GetCircle returns the circle geometry. It corresponds to
// b2Shape_GetCircle.
func (shapeId ShapeId) GetCircle() Circle {
	w := getWorld(shapeId.world0)
	s := getShape(w, shapeId)
	if s.shapeType != CircleShape {
		panic("dbox2d: shape is not a circle")
	}
	return s.circle
}

// GetSegment returns the segment geometry. It corresponds to
// b2Shape_GetSegment.
func (shapeId ShapeId) GetSegment() Segment {
	w := getWorld(shapeId.world0)
	s := getShape(w, shapeId)
	if s.shapeType != SegmentShape {
		panic("dbox2d: shape is not a segment")
	}
	return s.segment
}

// GetChainSegment returns the chain segment geometry. It corresponds to
// b2Shape_GetChainSegment.
func (shapeId ShapeId) GetChainSegment() ChainSegment {
	w := getWorld(shapeId.world0)
	s := getShape(w, shapeId)
	if s.shapeType != ChainSegmentShape {
		panic("dbox2d: shape is not a chain segment")
	}
	return s.chainSegment
}

// GetCapsule returns the capsule geometry. It corresponds to
// b2Shape_GetCapsule.
func (shapeId ShapeId) GetCapsule() Capsule {
	w := getWorld(shapeId.world0)
	s := getShape(w, shapeId)
	if s.shapeType != CapsuleShape {
		panic("dbox2d: shape is not a capsule")
	}
	return s.capsule
}

// GetPolygon returns the polygon geometry. It corresponds to
// b2Shape_GetPolygon.
func (shapeId ShapeId) GetPolygon() Polygon {
	w := getWorld(shapeId.world0)
	s := getShape(w, shapeId)
	if s.shapeType != PolygonShape {
		panic("dbox2d: shape is not a polygon")
	}
	return s.polygon
}

// SetCircle replaces the shape geometry with a circle. It corresponds to
// b2Shape_SetCircle.
func (shapeId ShapeId) SetCircle(circle *Circle) {
	w := getWorldLocked(shapeId.world0)
	s := getShape(w, shapeId)
	s.circle = *circle
	s.shapeType = CircleShape
	resetProxy(w, s, true, true)
}

// SetCapsule replaces the shape geometry with a capsule. It corresponds to
// b2Shape_SetCapsule.
func (shapeId ShapeId) SetCapsule(capsule *Capsule) {
	w := getWorldLocked(shapeId.world0)
	s := getShape(w, shapeId)
	s.capsule = *capsule
	s.shapeType = CapsuleShape
	resetProxy(w, s, true, true)
}

// SetSegment replaces the shape geometry with a segment. It corresponds to
// b2Shape_SetSegment.
func (shapeId ShapeId) SetSegment(segment *Segment) {
	w := getWorldLocked(shapeId.world0)
	s := getShape(w, shapeId)
	s.segment = *segment
	s.shapeType = SegmentShape
	resetProxy(w, s, true, true)
}

// SetPolygon replaces the shape geometry with a polygon. It corresponds to
// b2Shape_SetPolygon.
func (shapeId ShapeId) SetPolygon(polygon *Polygon) {
	w := getWorldLocked(shapeId.world0)
	s := getShape(w, shapeId)
	s.polygon = *polygon
	s.shapeType = PolygonShape
	resetProxy(w, s, true, true)
}

// GetContactCapacity returns the shape's conservative contact capacity. It
// corresponds to b2Shape_GetContactCapacity.
func (shapeId ShapeId) GetContactCapacity() int {
	w := getWorldLocked(shapeId.world0)
	s := getShape(w, shapeId)
	if s.sensorIndex != nullIndex {
		return 0
	}
	return w.bodies[s.bodyId].contactCount
}

// GetContactData fills touching contact data in body-list order. It
// corresponds to b2Shape_GetContactData.
func (shapeId ShapeId) GetContactData(data []ContactData) int {
	w := getWorldLocked(shapeId.world0)
	s := getShape(w, shapeId)
	if s.sensorIndex != nullIndex {
		return 0
	}

	b := &w.bodies[s.bodyId]
	contactKey := b.headContactKey
	count := 0
	for contactKey != nullIndex && count < len(data) {
		contactId := contactKey >> 1
		edgeIndex := contactKey & 1
		c := &w.contacts[contactId]

		if (c.shapeIdA == s.id || c.shapeIdB == s.id) && c.flags&contactTouchingFlag != 0 {
			sA := &w.shapes[c.shapeIdA]
			sB := &w.shapes[c.shapeIdB]
			data[count].ShapeIdA = ShapeId{index1: int32(sA.id) + 1, world0: shapeId.world0, generation: sA.generation}
			data[count].ShapeIdB = ShapeId{index1: int32(sB.id) + 1, world0: shapeId.world0, generation: sB.generation}
			data[count].Manifold = getContactSim(w, c).manifold
			count++
		}

		contactKey = c.edges[edgeIndex].nextKey
	}
	return count
}

// GetAABB returns the shape's current axis-aligned bounds. It corresponds to
// b2Shape_GetAABB.
func (shapeId ShapeId) GetAABB() AABB {
	w := getWorld(shapeId.world0)
	return getShape(w, shapeId).aabb
}

// GetMassData returns the shape mass, centroid and rotational inertia. It
// corresponds to b2Shape_GetMassData.
func (shapeId ShapeId) GetMassData() MassData {
	w := getWorld(shapeId.world0)
	return computeShapeMass(getShape(w, shapeId))
}

// GetClosestPoint returns the closest point on the shape to a world target.
// It corresponds to b2Shape_GetClosestPoint.
func (shapeId ShapeId) GetClosestPoint(target Vec2) Vec2 {
	w := getWorld(shapeId.world0)
	s := getShape(w, shapeId)
	b := &w.bodies[s.bodyId]
	transform := getBodyTransformQuick(w, b)

	zero := fixed.Q32Zero()
	input := DistanceInput{
		ProxyA:     makeShapeDistanceProxy(s),
		ProxyB:     MakeProxy([]Vec2{target}, zero),
		TransformA: transform,
		TransformB: TransformIdentity(),
		UseRadii:   true,
	}
	cache := SimplexCache{}
	return ShapeDistance(&input, &cache, nil).PointA
}
