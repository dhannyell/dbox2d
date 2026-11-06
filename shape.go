package b2

import "unsafe"

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
	userMaterialId    uint64

	aabb          AABB
	aabbMargin    Q
	localCentroid Vec2
	proxyKey      int

	filter      Filter
	userData    any
	customColor uint32

	// The reference overlays the five geometries in a union; shapeType
	// selects the live one. The geometries share this storage through the
	// typed accessors below, so a shape does not carry the four dead ones.
	geometry shapeGeometry

	// generation advances on each allocation of this slot, so a stale
	// ShapeId fails validation.
	generation uint16

	enableSensorEvents    bool
	enableContactEvents   bool
	enableHitEvents       bool
	enablePreSolveEvents  bool
	enableCustomFiltering bool
}

// shapeGeometry is the storage of the geometry union: the largest of the
// five geometries, in 8-byte words so every field is aligned. None of the
// geometries holds a pointer, so the garbage collector never looks inside.
type shapeGeometry [(max(
	unsafe.Sizeof(Capsule{}),
	unsafe.Sizeof(Circle{}),
	unsafe.Sizeof(Polygon{}),
	unsafe.Sizeof(Segment{}),
	unsafe.Sizeof(ChainSegment{}),
) + 7) / 8]uint64

func (s *shape) capsule() *Capsule           { return (*Capsule)(unsafe.Pointer(&s.geometry)) }
func (s *shape) circle() *Circle             { return (*Circle)(unsafe.Pointer(&s.geometry)) }
func (s *shape) polygon() *Polygon           { return (*Polygon)(unsafe.Pointer(&s.geometry)) }
func (s *shape) segment() *Segment           { return (*Segment)(unsafe.Pointer(&s.geometry)) }
func (s *shape) chainSegment() *ChainSegment { return (*ChainSegment)(unsafe.Pointer(&s.geometry)) }

type chainShape struct {
	id           int
	bodyId       int
	nextChainId  int
	shapeIndices []int
	generation   uint16
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

// computeShapeMargin returns the fat AABB margin of a shape: a fraction of
// its extent, capped at maxAABBMargin. Small shapes get small margins. It
// corresponds to b2ComputeShapeMargin.
func computeShapeMargin(s *shape) Q {
	var margin Q
	switch s.shapeType {
	case CapsuleShape:
		c := s.capsule()
		margin = QHalf().Mul(c.Center2.Distance(c.Center1)).Add(c.Radius)
	case CircleShape:
		margin = s.circle().Radius
	case PolygonShape:
		poly := s.polygon()
		maxExtentSqr := QZero()
		for i := 0; i < poly.Count; i++ {
			distanceSqr := poly.Vertices[i].DistanceSq(poly.Centroid)
			maxExtentSqr = maxExtentSqr.Max(distanceSqr)
		}
		margin = maxExtentSqr.Sqrt()
	case SegmentShape:
		seg := s.segment()
		margin = QHalf().Mul(seg.Point1.Distance(seg.Point2))
	case ChainSegmentShape:
		seg := &s.chainSegment().Segment
		margin = QHalf().Mul(seg.Point1.Distance(seg.Point2))
	default:
		panic("dbox2d: unknown shape type")
	}

	return maxAABBMargin.Min(aabbMarginFraction.Mul(margin))
}

// fatMargin returns the margin of a fat AABB: the margin of the shape, or
// the speculative distance for a static proxy.
func fatMargin(s *shape, proxyType BodyType) Q {
	// Smaller margin for static bodies. Cannot be zero due to TOI tolerance.
	if proxyType == StaticBody {
		return speculativeDistance
	}
	return s.aabbMargin
}

// fattenAABB grows a box by a margin on every side.
func fattenAABB(aabb AABB, margin Q) AABB {
	return AABB{
		LowerBound: Vec2{X: aabb.LowerBound.X.Sub(margin), Y: aabb.LowerBound.Y.Sub(margin)},
		UpperBound: Vec2{X: aabb.UpperBound.X.Add(margin), Y: aabb.UpperBound.Y.Add(margin)},
	}
}

// updateShapeAABBs refreshes the tight bounds of a shape and its fat bounds.
// It corresponds to b2UpdateShapeAABBs.
func updateShapeAABBs(s *shape, fatAABB *AABB, transform Transform, proxyType BodyType) {
	// Compute a bounding box with a speculative margin
	aabb := computeFatShapeAABB(s, transform, speculativeDistance)
	s.aabb = aabb
	*fatAABB = fattenAABB(aabb, fatMargin(s, proxyType))
}

// createShapeInternal allocates the shape record and links it to the body.
func createShapeInternal(w *world, b *body, transform Transform, def *ShapeDef, geometry any, shapeType ShapeType) *shape {
	shapeId := w.shapeIdPool.allocId()

	if shapeId == len(w.shapes) {
		w.shapes = append(w.shapes, shape{})
		w.fatAABBs = append(w.fatAABBs, AABB{})
	} else if w.shapes[shapeId].id != nullIndex {
		panic("dbox2d: the shape slot is still in use")
	}

	s := &w.shapes[shapeId]

	switch shapeType {
	case CapsuleShape:
		*s.capsule() = *geometry.(*Capsule)
	case CircleShape:
		*s.circle() = *geometry.(*Circle)
	case PolygonShape:
		*s.polygon() = *geometry.(*Polygon)
	case SegmentShape:
		*s.segment() = *geometry.(*Segment)
	case ChainSegmentShape:
		*s.chainSegment() = *geometry.(*ChainSegment)
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
	s.enableSensorEvents = def.EnableSensorEvents
	s.enableContactEvents = def.EnableContactEvents
	s.enableHitEvents = def.EnableHitEvents
	s.enablePreSolveEvents = def.EnablePreSolveEvents
	s.enableCustomFiltering = def.EnableCustomFiltering
	s.proxyKey = nullIndex
	s.localCentroid = getShapeCentroid(s)
	s.aabbMargin = computeShapeMargin(s)
	s.aabb = AABB{}
	w.fatAABBs[shapeId] = AABB{}
	s.generation += 1

	if b.setIndex != disabledSet {
		proxyType := b.bodyType
		createShapeProxy(w, s, proxyType, transform, def.InvokeContactCreation || def.IsSensor)
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

	s.sensorIndex = nullIndex
	if def.IsSensor {
		s.sensorIndex = len(w.sensors)
		w.sensors = append(w.sensors, sensor{shapeId: s.id})
	}

	return s
}

// createShape validates the definition and dispatches on the geometry.
func createShape(bodyId BodyId, def *ShapeDef, geometry any, shapeType ShapeType) ShapeId {
	checkDef(def.internalValue)
	zero := QZero()
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
// than the linear slop is not created and the null ShapeId is returned.
func CreateCapsuleShape(bodyId BodyId, def *ShapeDef, capsule *Capsule) ShapeId {
	lengthSqr := capsule.Center1.DistanceSq(capsule.Center2)
	if !linearSlop.Mul(linearSlop).Less(lengthSqr) {
		return ShapeId{}
	}

	return createShape(bodyId, def, capsule, CapsuleShape)
}

// CreatePolygonShape creates a polygon shape on a body. Build the polygon
// with MakePolygon or MakeBox.
func CreatePolygonShape(bodyId BodyId, def *ShapeDef, polygon *Polygon) ShapeId {
	if !IsValidQ(polygon.Radius) || polygon.Radius.Less(QZero()) {
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

// validateMaterial panics on a material with an invalid friction,
// restitution, rolling resistance or tangent speed. It corresponds to
// b2IsValidMaterial.
func validateMaterial(material *SurfaceMaterial, what string) {
	zero := QZero()
	if !IsValidQ(material.Friction) || material.Friction.Less(zero) {
		panic("dbox2d: " + what + " has an invalid friction")
	}
	if !IsValidQ(material.Restitution) || material.Restitution.Less(zero) {
		panic("dbox2d: " + what + " has an invalid restitution")
	}
	if !IsValidQ(material.RollingResistance) || material.RollingResistance.Less(zero) {
		panic("dbox2d: " + what + " has an invalid rolling resistance")
	}
	if !IsValidQ(material.TangentSpeed) {
		panic("dbox2d: " + what + " has an invalid tangent speed")
	}
}

// isValidChainSegment reports whether every point of the chain segment is
// valid. It corresponds to b2IsValidChainSegment.
func isValidChainSegment(chainSegment *ChainSegment) bool {
	return IsValidVec2(chainSegment.Ghost1) && IsValidVec2(chainSegment.Segment.Point1) &&
		IsValidVec2(chainSegment.Segment.Point2) && IsValidVec2(chainSegment.Ghost2)
}

// isValidGhost reports whether a chain ghost point was set: it is valid and
// is not the infinity sentinel of DefaultChainDef. Fixed point has no
// infinity, so the sentinel is the largest value in both components.
func isValidGhost(ghost Vec2) bool {
	return IsValidVec2(ghost) && ghost != Vec2{X: qInfinity(), Y: qInfinity()}
}

// CreateChainSegmentShape creates a chain segment that has no parent chain.
// It panics on a degenerate segment. It corresponds to
// b2CreateChainSegmentShape.
func CreateChainSegmentShape(bodyId BodyId, def *ShapeDef, chainSegment *ChainSegment) ShapeId {
	if !isValidChainSegment(chainSegment) {
		panic("dbox2d: the chain segment is not valid")
	}

	lengthSqr := chainSegment.Segment.Point1.DistanceSq(chainSegment.Segment.Point2)
	if !linearSlop.Mul(linearSlop).Less(lengthSqr) {
		panic("dbox2d: the chain segment is degenerate")
	}

	// No parent chain shape
	local := *chainSegment
	local.ChainId = nullIndex

	return createShape(bodyId, def, &local, ChainSegmentShape)
}

// CreateChain creates a chain of segment shapes on a body. An open chain
// needs at least two points and both ghost points; a loop needs at least
// three points. It corresponds to b2CreateChain.
func CreateChain(bodyId BodyId, def *ChainDef) ChainId {
	checkDef(def.internalValue)

	pointCount := len(def.Points)
	if def.IsLoop && pointCount < 3 {
		panic("dbox2d: a loop chain needs at least 3 points")
	}
	if !def.IsLoop && pointCount < 2 {
		panic("dbox2d: an open chain needs at least 2 points")
	}

	segmentCount := pointCount - 1
	if def.IsLoop {
		segmentCount = pointCount
	}

	materialCount := len(def.Materials)
	if materialCount != 1 && materialCount != segmentCount {
		panic("dbox2d: ChainDef.Materials must contain one material or one per segment")
	}

	for i := range def.Points {
		if !IsValidVec2(def.Points[i]) {
			panic("dbox2d: ChainDef.Points contains an invalid point")
		}
	}

	for i := range def.Materials {
		validateMaterial(&def.Materials[i], "ChainDef.Materials")
	}

	if !def.IsLoop {
		// These are set to infinity in the default def as a sentinel to
		// ensure they are set.
		if !isValidGhost(def.Ghost1) || !isValidGhost(def.Ghost2) {
			panic("dbox2d: an open chain needs ChainDef.Ghost1 and ChainDef.Ghost2")
		}
	}

	w := getWorldLocked(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	transform := getBodyTransformQuick(w, b)

	chainID := w.chainIdPool.allocId()
	if chainID == len(w.chainShapes) {
		w.chainShapes = append(w.chainShapes, chainShape{})
	} else if w.chainShapes[chainID].id != nullIndex {
		panic("dbox2d: the chain slot is still in use")
	}

	chain := &w.chainShapes[chainID]
	chain.id = chainID
	chain.bodyId = b.id
	chain.nextChainId = b.headChainId
	chain.generation += 1
	b.headChainId = chainID

	shapeDef := DefaultShapeDef()
	shapeDef.UserData = def.UserData
	shapeDef.Filter = def.Filter
	shapeDef.EnableSensorEvents = def.EnableSensorEvents
	shapeDef.EnableContactEvents = false
	shapeDef.EnableHitEvents = false

	points := def.Points
	n := segmentCount
	chain.shapeIndices = make([]int, n)

	var chainSegment ChainSegment
	if def.IsLoop {
		prevIndex := n - 1
		for i := 0; i < n; i++ {
			chainSegment.Ghost1 = points[prevIndex]
			chainSegment.Segment.Point1 = points[i]
			chainSegment.Segment.Point2 = points[(i+1)%n]
			chainSegment.Ghost2 = points[(i+2)%n]
			chainSegment.ChainId = chainID
			prevIndex = i

			materialIndex := 0
			if materialCount != 1 {
				materialIndex = i
			}
			shapeDef.Material = def.Materials[materialIndex]

			s := createShapeInternal(w, b, transform, &shapeDef, &chainSegment, ChainSegmentShape)
			chain.shapeIndices[i] = s.id
		}
	} else {
		for i := 0; i < n; i++ {
			if i == 0 {
				chainSegment.Ghost1 = def.Ghost1
			} else {
				chainSegment.Ghost1 = points[i-1]
			}
			chainSegment.Segment.Point1 = points[i]
			chainSegment.Segment.Point2 = points[i+1]
			if i == n-1 {
				chainSegment.Ghost2 = def.Ghost2
			} else {
				chainSegment.Ghost2 = points[i+2]
			}
			chainSegment.ChainId = chainID

			materialIndex := 0
			if materialCount != 1 {
				materialIndex = i
			}
			shapeDef.Material = def.Materials[materialIndex]

			s := createShapeInternal(w, b, transform, &shapeDef, &chainSegment, ChainSegmentShape)
			chain.shapeIndices[i] = s.id
		}
	}

	return ChainId{index1: int32(chainID) + 1, world0: w.worldId, generation: chain.generation}
}

// DestroyChain ports b2DestroyChain.
func DestroyChain(chainId ChainId) {
	w := getWorldLocked(chainId.world0)
	id := int(chainId.index1) - 1
	chain := &w.chainShapes[id]
	if chain.id != id || chain.generation != chainId.generation {
		panic("dbox2d: invalid ChainId")
	}
	b := &w.bodies[chain.bodyId]

	chainIDPtr := &b.headChainId
	found := false
	for *chainIDPtr != nullIndex {
		if *chainIDPtr == chain.id {
			*chainIDPtr = chain.nextChainId
			found = true
			break
		}
		chainIDPtr = &w.chainShapes[*chainIDPtr].nextChainId
	}
	if !found {
		panic("dbox2d: chain is not attached to its body")
	}

	for _, shapeID := range chain.shapeIndices {
		s := &w.shapes[shapeID]
		destroyShapeInternal(w, s, b)
	}

	freeChainData(chain)
	w.chainIdPool.freeId(chain.id)
	chain.id = nullIndex
}

func getChain(w *world, chainId ChainId) *chainShape {
	id := int(chainId.index1) - 1
	chain := &w.chainShapes[id]
	if chain.id != id || chain.generation != chainId.generation {
		panic("dbox2d: invalid ChainId")
	}
	return chain
}

// freeChainData releases the storage owned by a chain. It corresponds to
// b2FreeChainData.
func freeChainData(chain *chainShape) {
	chain.shapeIndices = nil
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

	if s.sensorIndex != nullIndex {
		destroySensor(w, s)
	}

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

	// Cannot destroy a chain segment that has a parent chain shape
	if s.shapeType == ChainSegmentShape && s.chainSegment().ChainId != nullIndex {
		panic("dbox2d: DestroyShape cannot destroy a segment owned by a chain")
	}

	b := &w.bodies[s.bodyId]

	destroyShapeInternal(w, s, b)

	if updateBodyMass {
		updateBodyMassData(w, b)
	}
}

// createShapeProxy refreshes the bounds of a shape and inserts it in the
// broadphase. It corresponds to b2CreateShapeProxy in src/shape.c.
func createShapeProxy(w *world, s *shape, proxyType BodyType, transform Transform, forcePairCreation bool) {
	if s.proxyKey != nullIndex {
		panic("dbox2d: the shape already has a proxy")
	}

	fatAABB := &w.fatAABBs[s.id]
	updateShapeAABBs(s, fatAABB, transform, proxyType)

	// Create proxies in the broad-phase.
	s.proxyKey = w.broadPhase.createProxy(proxyType, *fatAABB, s.filter.CategoryBits, s.id, forcePairCreation)
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
		return ComputeCapsuleBounds(s.capsule(), xf)
	case CircleShape:
		return ComputeCircleBounds(s.circle(), xf)
	case PolygonShape:
		return ComputePolygonBounds(s.polygon(), xf)
	case SegmentShape:
		return ComputeSegmentBounds(s.segment(), xf)
	case ChainSegmentShape:
		return ComputeSegmentBounds(&s.chainSegment().Segment, xf)
	default:
		panic("dbox2d: unknown shape type")
	}
}

// computeFatShapeAABB returns the bounds of a shape grown by extra on every
// side. Like upstream, the radius and extra are summed before they meet the
// position, so each bound rounds once. It corresponds to
// b2ComputeFatShapeAABB.
func computeFatShapeAABB(s *shape, xf Transform, extra Q) AABB {
	var lower, upper Vec2
	radius := QZero()
	switch s.shapeType {
	case CapsuleShape:
		c := s.capsule()
		v1 := TransformPoint(xf, c.Center1)
		v2 := TransformPoint(xf, c.Center2)
		lower, upper, radius = Min(v1, v2), Max(v1, v2), c.Radius
	case CircleShape:
		c := s.circle()
		lower = TransformPoint(xf, c.Center)
		upper, radius = lower, c.Radius
	case PolygonShape:
		poly := s.polygon()
		lower = TransformPoint(xf, poly.Vertices[0])
		upper = lower
		for i := 1; i < poly.Count; i++ {
			v := TransformPoint(xf, poly.Vertices[i])
			lower = Min(lower, v)
			upper = Max(upper, v)
		}
		radius = poly.Radius
	case SegmentShape:
		seg := s.segment()
		v1 := TransformPoint(xf, seg.Point1)
		v2 := TransformPoint(xf, seg.Point2)
		lower, upper = Min(v1, v2), Max(v1, v2)
	case ChainSegmentShape:
		seg := &s.chainSegment().Segment
		v1 := TransformPoint(xf, seg.Point1)
		v2 := TransformPoint(xf, seg.Point2)
		lower, upper = Min(v1, v2), Max(v1, v2)
	default:
		panic("dbox2d: unknown shape type")
	}

	return AABB{
		LowerBound: Vec2{X: qSubSum(lower.X, radius, extra), Y: qSubSum(lower.Y, radius, extra)},
		UpperBound: Vec2{X: qAddSum(upper.X, radius, extra), Y: qAddSum(upper.Y, radius, extra)},
	}
}

// getShapeCentroid returns the centroid of a shape in local coordinates.
func getShapeCentroid(s *shape) Vec2 {
	switch s.shapeType {
	case CapsuleShape:
		return Lerp(s.capsule().Center1, s.capsule().Center2, QHalf())
	case CircleShape:
		return s.circle().Center
	case PolygonShape:
		return s.polygon().Centroid
	case SegmentShape:
		return Lerp(s.segment().Point1, s.segment().Point2, QHalf())
	case ChainSegmentShape:
		return Lerp(s.chainSegment().Segment.Point1, s.chainSegment().Segment.Point2, QHalf())
	default:
		return Vec2Zero()
	}
}

// getShapeRadius returns the round radius of a shape. Segments have no
// radius. It corresponds to b2GetShapeRadius in src/shape.h.
func getShapeRadius(s *shape) Q {
	switch s.shapeType {
	case CapsuleShape:
		return s.capsule().Radius
	case CircleShape:
		return s.circle().Radius
	case PolygonShape:
		return s.polygon().Radius
	default:
		return QZero()
	}
}

// computeShapeMass returns the mass data of a shape. Segments have no area,
// so they return zero mass.
func computeShapeMass(s *shape) MassData {
	switch s.shapeType {
	case CapsuleShape:
		return ComputeCapsuleMass(s.capsule(), s.density)
	case CircleShape:
		return ComputeCircleMass(s.circle(), s.density)
	case PolygonShape:
		return ComputePolygonMass(s.polygon(), s.density)
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
		radius := s.capsule().Radius
		extent.minExtent = radius
		c1 := s.capsule().Center1.Sub(localCenter)
		c2 := s.capsule().Center2.Sub(localCenter)
		extent.maxExtent = c1.LenSq().Max(c2.LenSq()).Sqrt().Add(radius)

	case CircleShape:
		radius := s.circle().Radius
		extent.minExtent = radius
		extent.maxExtent = s.circle().Center.Sub(localCenter).Len().Add(radius)

	case PolygonShape:
		poly := s.polygon()
		minExtent := Huge
		maxExtentSqr := QZero()
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
		extent.minExtent = QZero()
		c1 := s.segment().Point1.Sub(localCenter)
		c2 := s.segment().Point2.Sub(localCenter)
		extent.maxExtent = c1.LenSq().Max(c2.LenSq()).Sqrt()

	case ChainSegmentShape:
		extent.minExtent = QZero()
		c1 := s.chainSegment().Segment.Point1.Sub(localCenter)
		c2 := s.chainSegment().Segment.Point2.Sub(localCenter)
		extent.maxExtent = c1.LenSq().Max(c2.LenSq()).Sqrt()
	}

	return extent
}

// getShapeProjectedPerimeter projects the shape perimeter onto an infinite
// line. It corresponds to b2GetShapeProjectedPerimeter in src/shape.c.
func getShapeProjectedPerimeter(s *shape, line Vec2) Q {
	switch s.shapeType {
	case CapsuleShape:
		axis := s.capsule().Center2.Sub(s.capsule().Center1)
		projectedLength := axis.Dot(line).Abs()
		return projectedLength.Add(s.capsule().Radius.Add(s.capsule().Radius))

	case CircleShape:
		return s.circle().Radius.Add(s.circle().Radius)

	case PolygonShape:
		poly := s.polygon()
		value := poly.Vertices[0].Dot(line)
		lower := value
		upper := value
		for i := 1; i < poly.Count; i++ {
			value = poly.Vertices[i].Dot(line)
			lower = lower.Min(value)
			upper = upper.Max(value)
		}
		return upper.Sub(lower).Add(poly.Radius.Add(poly.Radius))

	case SegmentShape:
		value1 := s.segment().Point1.Dot(line)
		value2 := s.segment().Point2.Dot(line)
		return value2.Sub(value1).Abs()

	case ChainSegmentShape:
		value1 := s.chainSegment().Segment.Point1.Dot(line)
		value2 := s.chainSegment().Segment.Point2.Dot(line)
		return value2.Sub(value1).Abs()

	default:
		return QZero()
	}
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
		output = CastRayCapsule(s.capsule(), &localInput)
	case CircleShape:
		output = CastRayCircle(s.circle(), &localInput)
	case PolygonShape:
		output = CastRayPolygon(s.polygon(), &localInput)
	case SegmentShape:
		output = CastRaySegment(s.segment(), &localInput, false)
	case ChainSegmentShape:
		output = CastRaySegment(&s.chainSegment().Segment, &localInput, true)
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
	var output CastOutput
	if input.Proxy.Count == 0 {
		return output
	}

	localInput := *input

	for i := range localInput.Proxy.Count {
		localInput.Proxy.Points[i] = InvTransformPoint(transform, input.Proxy.Points[i])
	}

	localInput.Translation = InvRotateVector(transform.Q, input.Translation)

	switch s.shapeType {
	case CapsuleShape:
		output = CastShapeCapsule(s.capsule(), &localInput)
	case CircleShape:
		output = CastShapeCircle(s.circle(), &localInput)
	case PolygonShape:
		output = CastShapePolygon(s.polygon(), &localInput)
	case SegmentShape:
		output = CastShapeSegment(s.segment(), &localInput)
	case ChainSegmentShape:
		// A cast that starts behind a chain segment misses it (v3.2 fcc60b7).
		approximateCentroid := localInput.Proxy.Points[0]
		for i := 1; i < localInput.Proxy.Count; i++ {
			approximateCentroid = approximateCentroid.Add(localInput.Proxy.Points[i])
		}
		approximateCentroid = approximateCentroid.Mul(QOne().Div(QFromInt(localInput.Proxy.Count)))

		segment := &s.chainSegment().Segment
		edge := segment.Point2.Sub(segment.Point1)
		r := approximateCentroid.Sub(segment.Point1)
		if Cross(r, edge).Less(QZero()) {
			return output
		}

		output = CastShapeSegment(segment, &localInput)
	default:
		return output
	}

	output.Point = TransformPoint(transform, output.Point)
	output.Normal = RotateVector(transform.Q, output.Normal)
	return output
}

// collideMover finds the plane where a mover capsule pushes against a
// shape at a transform. It corresponds to b2CollideMover in
// src/shape.c.
func collideMover(mover *Capsule, s *shape, transform Transform) PlaneResult {
	localMover := Capsule{
		Center1: InvTransformPoint(transform, mover.Center1),
		Center2: InvTransformPoint(transform, mover.Center2),
		Radius:  mover.Radius,
	}

	var result PlaneResult
	switch s.shapeType {
	case CapsuleShape:
		result = CollideMoverAndCapsule(&localMover, s.capsule())
	case CircleShape:
		result = CollideMoverAndCircle(&localMover, s.circle())
	case PolygonShape:
		result = CollideMoverAndPolygon(&localMover, s.polygon())
	case SegmentShape:
		result = CollideMoverAndSegment(&localMover, s.segment())
	case ChainSegmentShape:
		result = CollideMoverAndSegment(&localMover, &s.chainSegment().Segment)
	default:
		return result
	}

	if !result.Hit {
		return result
	}

	result.Plane.Normal = RotateVector(transform.Q, result.Plane.Normal)
	result.Point = TransformPoint(transform, result.Point)
	return result
}

// makeShapeDistanceProxy builds the distance proxy of a shape in its body
// frame. It corresponds to b2MakeShapeDistanceProxy in src/shape.c.
func makeShapeDistanceProxy(s *shape) ShapeProxy {
	switch s.shapeType {
	case CapsuleShape:
		return MakeProxy([]Vec2{s.capsule().Center1, s.capsule().Center2}, s.capsule().Radius)
	case CircleShape:
		return MakeProxy([]Vec2{s.circle().Center}, s.circle().Radius)
	case PolygonShape:
		return MakeProxy(s.polygon().Vertices[:s.polygon().Count], s.polygon().Radius)
	case SegmentShape:
		return MakeProxy([]Vec2{s.segment().Point1, s.segment().Point2}, QZero())
	case ChainSegmentShape:
		return MakeProxy([]Vec2{s.chainSegment().Segment.Point1, s.chainSegment().Segment.Point2}, QZero())
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

// GetParentChain returns the chain that owns the shape. It corresponds to
// b2Shape_GetParentChain.
func (shapeId ShapeId) GetParentChain() ChainId {
	w := getWorld(shapeId.world0)
	s := getShape(w, shapeId)
	if s.shapeType != ChainSegmentShape {
		return ChainId{}
	}

	chainIndex := s.chainSegment().ChainId
	if chainIndex == nullIndex {
		return ChainId{}
	}

	chain := &w.chainShapes[chainIndex]
	return ChainId{index1: int32(chainIndex) + 1, world0: shapeId.world0, generation: chain.generation}
}

// IsValid reports whether the id references a live chain.
func (chainId ChainId) IsValid() bool {
	if maxWorlds <= chainId.world0 {
		return false
	}

	w := &worlds[chainId.world0]
	if w.worldId != chainId.world0 {
		return false
	}

	chainIndex := int(chainId.index1) - 1
	if chainIndex < 0 || len(w.chainShapes) <= chainIndex {
		return false
	}

	chain := &w.chainShapes[chainIndex]
	if chain.id == nullIndex {
		return false
	}
	if chain.id != chainIndex {
		panic("dbox2d: the chain id does not match its slot")
	}

	return chain.generation == chainId.generation
}

// GetWorld returns the world containing the chain. It corresponds to
// b2Chain_GetWorld.
func (chainId ChainId) GetWorld() WorldId {
	w := getWorld(chainId.world0)
	return WorldId{index1: chainId.world0 + 1, generation: w.generation}
}

// GetSegmentCount returns the number of segments in the chain. It corresponds
// to b2Chain_GetSegmentCount.
func (chainId ChainId) GetSegmentCount() int {
	w := getWorld(chainId.world0)
	return len(getChain(w, chainId).shapeIndices)
}

// GetSegments fills segments with the chain's segment shapes. It corresponds
// to b2Chain_GetSegments.
func (chainId ChainId) GetSegments(segments []ShapeId) int {
	w := getWorld(chainId.world0)
	chain := getChain(w, chainId)
	count := len(chain.shapeIndices)
	if len(segments) < count {
		count = len(segments)
	}

	for i := 0; i < count; i++ {
		s := &w.shapes[chain.shapeIndices[i]]
		segments[i] = shapeIdOf(w, s)
	}
	return count
}

// SetSurfaceMaterial changes the material of one chain segment. It
// corresponds to b2Chain_SetSurfaceMaterial.
func (chainId ChainId) SetSurfaceMaterial(material SurfaceMaterial, segmentIndex int) {
	validateMaterial(&material, "SetSurfaceMaterial material")

	w := getWorldLocked(chainId.world0)
	chain := getChain(w, chainId)
	if segmentIndex < 0 || len(chain.shapeIndices) <= segmentIndex {
		panic("dbox2d: the chain segment index is out of range")
	}
	setShapeMaterial(&w.shapes[chain.shapeIndices[segmentIndex]], material)
}

// SetAllSurfaceMaterials changes the material of every chain segment. It
// corresponds to b2Chain_SetAllSurfaceMaterials.
func (chainId ChainId) SetAllSurfaceMaterials(material SurfaceMaterial) {
	validateMaterial(&material, "SetAllSurfaceMaterials material")

	w := getWorldLocked(chainId.world0)
	chain := getChain(w, chainId)
	for _, shapeIndex := range chain.shapeIndices {
		setShapeMaterial(&w.shapes[shapeIndex], material)
	}
}

// GetSurfaceMaterial returns the material of one chain segment. It
// corresponds to b2Chain_GetSurfaceMaterial.
func (chainId ChainId) GetSurfaceMaterial(segmentIndex int) SurfaceMaterial {
	w := getWorld(chainId.world0)
	chain := getChain(w, chainId)
	if segmentIndex < 0 || len(chain.shapeIndices) <= segmentIndex {
		panic("dbox2d: the chain segment index is out of range")
	}
	return shapeMaterial(&w.shapes[chain.shapeIndices[segmentIndex]])
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

// GetSensorCapacity returns the number of current sensor overlaps. It
// corresponds to b2Shape_GetSensorCapacity in src/shape.c.
func (shapeId ShapeId) GetSensorCapacity() int {
	if !shapeId.IsValid() {
		return 0
	}

	w := getWorldLocked(shapeId.world0)
	s := getShape(w, shapeId)
	if s.sensorIndex == nullIndex {
		return 0
	}

	return len(w.sensors[s.sensorIndex].overlaps1)
}

// GetSensorData fills overlaps with the current sensor overlaps. It
// corresponds to b2Shape_GetSensorData in src/shape.c.
func (shapeId ShapeId) GetSensorData(overlaps []ShapeId) int {
	if !shapeId.IsValid() {
		return 0
	}

	w := getWorldLocked(shapeId.world0)
	s := getShape(w, shapeId)
	if s.sensorIndex == nullIndex {
		return 0
	}

	count := 0
	for _, ref := range w.sensors[s.sensorIndex].overlaps1 {
		if count == len(overlaps) {
			break
		}

		visitorShape := &w.shapes[ref.shapeId]
		if visitorShape.id != ref.shapeId || visitorShape.generation != ref.generation {
			continue
		}

		overlaps[count] = ShapeId{index1: int32(ref.shapeId) + 1, world0: w.worldId, generation: ref.generation}
		count++
	}

	return count
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
		return PointInCapsule(s.capsule(), localPoint)
	case CircleShape:
		return PointInCircle(s.circle(), localPoint)
	case PolygonShape:
		zero := QZero()
		input := DistanceInput{
			ProxyA:    MakeProxy(s.polygon().Vertices[:s.polygon().Count], zero),
			ProxyB:    MakeProxy([]Vec2{localPoint}, zero),
			Transform: TransformIdentity(),
			UseRadii:  false,
		}
		cache := SimplexCache{}
		output := ComputeShapeDistance(&input, &cache, nil)
		return !s.polygon().Radius.Less(output.Distance)
	default:
		return false
	}
}

// CastRay casts a world-space ray against the shape. It corresponds to
// b2Shape_CastRay.
func (shapeId ShapeId) CastRay(input *RayCastInput) CastOutput {
	w := getWorld(shapeId.world0)
	s := getShape(w, shapeId)
	transform := getBodyTransform(w, s.bodyId)
	return rayCastShape(input, s, transform)
}

// SetDensity changes the shape density and optionally updates body mass. It
// corresponds to b2Shape_SetDensity.
func (shapeId ShapeId) SetDensity(density Q, updateBodyMass bool) {
	zero := QZero()
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
	zero := QZero()
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
	zero := QZero()
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

// GetSurfaceMaterial returns the shape surface material. It corresponds to
// b2Shape_GetSurfaceMaterial.
func (shapeId ShapeId) GetSurfaceMaterial() SurfaceMaterial {
	w := getWorld(shapeId.world0)
	return shapeMaterial(getShape(w, shapeId))
}

// shapeMaterial gathers the surface material of a shape.
func shapeMaterial(s *shape) SurfaceMaterial {
	return SurfaceMaterial{
		Friction:          s.friction,
		Restitution:       s.restitution,
		RollingResistance: s.rollingResistance,
		TangentSpeed:      s.tangentSpeed,
		UserMaterialId:    s.userMaterialId,
		CustomColor:       s.customColor,
	}
}

// setShapeMaterial scatters a surface material into a shape.
func setShapeMaterial(s *shape, material SurfaceMaterial) {
	s.friction = material.Friction
	s.restitution = material.Restitution
	s.rollingResistance = material.RollingResistance
	s.tangentSpeed = material.TangentSpeed
	s.userMaterialId = material.UserMaterialId
	s.customColor = material.CustomColor
}

// SetUserMaterial sets the shape user material id. It corresponds to
// b2Shape_SetUserMaterial.
func (shapeId ShapeId) SetUserMaterial(material uint64) {
	w := getWorldLocked(shapeId.world0)
	getShape(w, shapeId).userMaterialId = material
}

// GetUserMaterial returns the shape user material id. It corresponds to
// b2Shape_GetUserMaterial.
func (shapeId ShapeId) GetUserMaterial() uint64 {
	w := getWorld(shapeId.world0)
	return getShape(w, shapeId).userMaterialId
}

// SetSurfaceMaterial changes the shape surface material. It corresponds to
// b2Shape_SetSurfaceMaterial.
func (shapeId ShapeId) SetSurfaceMaterial(material SurfaceMaterial) {
	validateMaterial(&material, "SetSurfaceMaterial material")

	w := getWorldLocked(shapeId.world0)
	setShapeMaterial(getShape(w, shapeId), material)
}

// GetFilter returns the shape collision filter. It corresponds to
// b2Shape_GetFilter.
func (shapeId ShapeId) GetFilter() Filter {
	w := getWorld(shapeId.world0)
	return getShape(w, shapeId).filter
}

// airDensity is the density of air in kg/m^3. In 2D the shape has unit
// depth.
var airDensity = QMustParse("1.225")

// ApplyWind applies a wind force to the body of the shape, using the density
// of air. It considers the projected area of the shape in the wind direction
// and the relative velocity of the shape. drag scales the force that opposes
// the relative velocity, lift the force perpendicular to it. Only circles,
// capsules and polygons on dynamic bodies feel the wind; a sleeping body
// feels it only when wake is set. It corresponds to b2Shape_ApplyWind.
func (shapeId ShapeId) ApplyWind(wind Vec2, drag, lift Q, wake bool) {
	if !IsValidVec2(wind) || !IsValidQ(drag) || !IsValidQ(lift) {
		panic("dbox2d: ApplyWind input is not valid")
	}

	w := getWorld(shapeId.world0)
	s := getShape(w, shapeId)

	shapeType := s.shapeType
	if shapeType != CircleShape && shapeType != CapsuleShape && shapeType != PolygonShape {
		return
	}

	b := &w.bodies[s.bodyId]
	if b.bodyType != DynamicBody {
		return
	}

	if b.setIndex >= firstSleepingSet && !wake {
		return
	}

	if b.setIndex != awakeSet {
		// Must wake for state to exist
		wakeBody(w, b)
	}

	if b.setIndex != awakeSet {
		panic("dbox2d: ApplyWind could not wake the body")
	}

	sim := getBodySim(w, b)
	state := getBodyState(w, b)
	transform := sim.transform
	half := QHalf()

	force := Vec2Zero()
	torque := QZero()

	centroid := s.localCentroid
	lever := RotateVector(transform.Q, centroid.Sub(sim.localCenter))
	shapeVelocity := state.linearVelocity.Add(CrossSV(state.angularVelocity, lever))
	relativeVelocity := MulSub(wind, drag, shapeVelocity)
	speed, direction := GetLengthAndNormalize(relativeVelocity)

	switch shapeType {
	case CircleShape:
		radius := s.circle().Radius
		projectedArea := QFromInt(2).Mul(radius)
		force = direction.Mul(half.Mul(airDensity).Mul(projectedArea).Mul(speed).Mul(speed))
		torque = Cross(lever, force)

	case CapsuleShape:
		c := s.capsule()
		d := RotateVector(transform.Q, c.Center2.Sub(c.Center1))

		radius := c.Radius
		projectedArea := QFromInt(2).Mul(radius).Add(Cross(d, direction).Abs())

		// Normal that opposes the wind
		normal := LeftPerp(d.Normalize())
		if QZero().Less(normal.Dot(direction)) {
			normal = Neg(normal)
		}

		// portion of wind that is perpendicular to surface
		liftDirection := CrossSV(Cross(normal, direction), direction)

		forceMagnitude := half.Mul(airDensity).Mul(projectedArea).Mul(speed).Mul(speed)
		force = MulAdd(direction, lift, liftDirection).Mul(forceMagnitude)

		edgeLever := MulAdd(lever, radius, normal)
		torque = Cross(edgeLever, force)

	case PolygonShape:
		// polygon radius is ignored for simplicity
		poly := s.polygon()
		count := poly.Count
		v1 := poly.Vertices[count-1]
		for i := 0; i < count; i++ {
			v2 := poly.Vertices[i]
			d := v2.Sub(v1)
			edgeCenter := Lerp(v1, v2, half)
			v1 = v2

			d = RotateVector(transform.Q, d)

			projectedArea := Cross(d, direction)
			if !QZero().Less(projectedArea) {
				// back facing
				continue
			}

			normal := RightPerp(d.Normalize())

			// portion of wind that is perpendicular to surface
			liftDirection := CrossSV(Cross(normal, direction), direction)

			forceMagnitude := half.Mul(airDensity).Mul(projectedArea).Mul(speed).Mul(speed)
			f := MulAdd(direction, lift, liftDirection).Mul(forceMagnitude)

			edgeLever := RotateVector(transform.Q, edgeCenter.Sub(sim.localCenter))

			force = force.Add(f)
			torque = torque.Add(Cross(edgeLever, f))
		}
	}

	sim.force = sim.force.Add(force)
	sim.torque = sim.torque.Add(torque)
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
		updateShapeAABBs(s, &w.fatAABBs[s.id], transform, proxyType)

		if destroyProxy {
			w.broadPhase.destroyProxy(s.proxyKey)
			s.proxyKey = w.broadPhase.createProxy(proxyType, w.fatAABBs[s.id], s.filter.CategoryBits, s.id, true)
		} else {
			w.broadPhase.moveProxy(s.proxyKey, w.fatAABBs[s.id])
		}
	} else {
		updateShapeAABBs(s, &w.fatAABBs[s.id], transform, b.bodyType)
	}

	validateSolverSetsDebug(w)
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
	return *s.circle()
}

// GetSegment returns the segment geometry. It corresponds to
// b2Shape_GetSegment.
func (shapeId ShapeId) GetSegment() Segment {
	w := getWorld(shapeId.world0)
	s := getShape(w, shapeId)
	if s.shapeType != SegmentShape {
		panic("dbox2d: shape is not a segment")
	}
	return *s.segment()
}

// GetChainSegment returns the chain segment geometry. It corresponds to
// b2Shape_GetChainSegment.
func (shapeId ShapeId) GetChainSegment() ChainSegment {
	w := getWorld(shapeId.world0)
	s := getShape(w, shapeId)
	if s.shapeType != ChainSegmentShape {
		panic("dbox2d: shape is not a chain segment")
	}
	return *s.chainSegment()
}

// GetCapsule returns the capsule geometry. It corresponds to
// b2Shape_GetCapsule.
func (shapeId ShapeId) GetCapsule() Capsule {
	w := getWorld(shapeId.world0)
	s := getShape(w, shapeId)
	if s.shapeType != CapsuleShape {
		panic("dbox2d: shape is not a capsule")
	}
	return *s.capsule()
}

// GetPolygon returns the polygon geometry. It corresponds to
// b2Shape_GetPolygon.
func (shapeId ShapeId) GetPolygon() Polygon {
	w := getWorld(shapeId.world0)
	s := getShape(w, shapeId)
	if s.shapeType != PolygonShape {
		panic("dbox2d: shape is not a polygon")
	}
	return *s.polygon()
}

// SetCircle replaces the shape geometry with a circle. It corresponds to
// b2Shape_SetCircle.
func (shapeId ShapeId) SetCircle(circle *Circle) {
	w := getWorldLocked(shapeId.world0)
	s := getShape(w, shapeId)
	*s.circle() = *circle
	s.shapeType = CircleShape
	s.aabbMargin = computeShapeMargin(s)
	resetProxy(w, s, true, true)
}

// SetCapsule replaces the shape geometry with a capsule. It corresponds to
// b2Shape_SetCapsule.
func (shapeId ShapeId) SetCapsule(capsule *Capsule) {
	w := getWorldLocked(shapeId.world0)
	s := getShape(w, shapeId)
	*s.capsule() = *capsule
	s.shapeType = CapsuleShape
	s.aabbMargin = computeShapeMargin(s)
	resetProxy(w, s, true, true)
}

// SetSegment replaces the shape geometry with a segment. It corresponds to
// b2Shape_SetSegment.
func (shapeId ShapeId) SetSegment(segment *Segment) {
	w := getWorldLocked(shapeId.world0)
	s := getShape(w, shapeId)
	*s.segment() = *segment
	s.shapeType = SegmentShape
	s.aabbMargin = computeShapeMargin(s)
	resetProxy(w, s, true, true)
}

// SetChainSegment replaces the shape geometry with a chain segment that has
// no parent chain. A segment owned by a chain cannot change, and a
// degenerate segment is ignored. It corresponds to b2Shape_SetChainSegment.
func (shapeId ShapeId) SetChainSegment(chainSegment *ChainSegment) {
	if !isValidChainSegment(chainSegment) {
		panic("dbox2d: the chain segment is not valid")
	}

	w := getWorldLocked(shapeId.world0)
	s := getShape(w, shapeId)

	// Cannot modify a chain segment that has a parent chain shape
	if s.shapeType == ChainSegmentShape && s.chainSegment().ChainId != nullIndex {
		panic("dbox2d: SetChainSegment cannot change a segment owned by a chain")
	}

	lengthSqr := chainSegment.Segment.Point1.DistanceSq(chainSegment.Segment.Point2)
	if !linearSlop.Mul(linearSlop).Less(lengthSqr) {
		return
	}

	*s.chainSegment() = *chainSegment
	s.chainSegment().ChainId = nullIndex
	s.shapeType = ChainSegmentShape
	s.aabbMargin = computeShapeMargin(s)
	resetProxy(w, s, true, true)
}

// SetPolygon replaces the shape geometry with a polygon. It corresponds to
// b2Shape_SetPolygon.
func (shapeId ShapeId) SetPolygon(polygon *Polygon) {
	w := getWorldLocked(shapeId.world0)
	s := getShape(w, shapeId)
	*s.polygon() = *polygon
	s.shapeType = PolygonShape
	s.aabbMargin = computeShapeMargin(s)
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
			data[count].ContactId = contactIdOf(w, c)
			data[count].ShapeIdA = ShapeId{index1: int32(sA.id) + 1, world0: shapeId.world0, generation: sA.generation}
			data[count].ShapeIdB = ShapeId{index1: int32(sB.id) + 1, world0: shapeId.world0, generation: sB.generation}
			data[count].Manifold = getContactSim(w, c).manifold
			count++
		}

		contactKey = c.edges[edgeIndex].nextKey
	}
	return count
}

// GetBounds returns the shape's current axis-aligned bounds. It corresponds to
// b2Shape_GetBounds.
func (shapeId ShapeId) GetBounds() AABB {
	w := getWorld(shapeId.world0)
	return getShape(w, shapeId).aabb
}

// ComputeMassData returns the shape mass, centroid and rotational inertia. It
// corresponds to b2Shape_ComputeMassData.
func (shapeId ShapeId) ComputeMassData() MassData {
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

	// The target rides in as the frame of proxy B, so the relative pose is
	// differenced before the query.
	zero := QZero()
	targetTransform := Transform{P: target, Q: RotIdentity()}
	input := DistanceInput{
		ProxyA:    makeShapeDistanceProxy(s),
		ProxyB:    MakeProxy([]Vec2{Vec2Zero()}, zero),
		Transform: InvMulTransforms(transform, targetTransform),
		UseRadii:  true,
	}
	cache := SimplexCache{}
	output := ComputeShapeDistance(&input, &cache, nil)
	return TransformPoint(transform, output.PointA)
}
