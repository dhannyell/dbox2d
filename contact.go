package b2

// Contact flags of the cold data. They correspond to b2ContactFlags in
// src/contact.h. The touching flag arrives with the narrowphase update of
// the step; the hit event flag arrives with the solver events.
const (
	// contactTouchingFlag is set when the shapes touch.
	contactTouchingFlag uint32 = 0x0001

	// contactEnableContactEvents marks a contact that wants contact events.
	contactEnableContactEvents uint32 = 0x0004

	// contactRecycleFlag marks a contact whose bodies both enable contact
	// recycling. The contact sim inherits it.
	contactRecycleFlag uint32 = 0x0008
)

// Contact sim flags, shifted to be distinct from the contact flags. They
// correspond to b2ContactSimFlags in src/contact.h.
const (
	// simTouchingFlag is set when the shapes touch.
	simTouchingFlag uint32 = 0x00010000

	// simDisjoint is set when the bounding boxes stop overlapping.
	simDisjoint uint32 = 0x00020000

	// simStartedTouching is set when the shapes began to touch this step.
	simStartedTouching uint32 = 0x00040000

	// simStoppedTouching is set when the shapes stopped touching this
	// step.
	simStoppedTouching uint32 = 0x00080000

	// simEnableHitEvent is set when a touching contact wants hit events.
	simEnableHitEvent uint32 = 0x00100000

	// simEnablePreSolveEvents marks a contact that wants pre-solve events.
	simEnablePreSolveEvents uint32 = 0x00200000

	// simRelativeTransformValid is set when the contact cached the relative
	// transform of its bodies for recycling.
	simRelativeTransformValid uint32 = 0x00400000
)

// contactEdge connects a body to a contact inside the contact graph. Each
// contact has two edges, one per body, on a doubly linked list keyed by
// contactId<<1|edgeIndex. It corresponds to b2ContactEdge in src/contact.h.
type contactEdge struct {
	bodyId  int
	prevKey int
	nextKey int
}

// contact is the cold contact data: a persistent handle and the island
// connectivity. It corresponds to b2Contact in src/contact.h.
type contact struct {
	// setIndex is the solver set in the world, or nullIndex for a free
	// slot.
	setIndex int

	// colorIndex is the constraint graph color. It stays nullIndex until
	// the graph lands.
	colorIndex int

	// localIndex is the position of the sim data inside the set.
	localIndex int

	edges    [2]contactEdge
	shapeIdA int
	shapeIdB int

	// A contact belongs to an island only when touching.
	islandId    int
	islandIndex int

	contactId int

	flags uint32

	// generation advances each time a contact takes this slot. It lets a
	// ContactId detect a destroyed contact.
	generation uint32
}

// contactIdOf builds the public id of a live contact.
func contactIdOf(w *world, c *contact) ContactId {
	return ContactId{index1: int32(c.contactId) + 1, world0: w.worldId, generation: c.generation}
}

// getContactFullId returns the contact of a valid id. It corresponds to
// b2GetContactFullId in src/contact.c.
func getContactFullId(w *world, contactId ContactId) *contact {
	id := int(contactId.index1) - 1
	if id < 0 || len(w.contacts) <= id {
		panic("dbox2d: the contact id is out of range")
	}
	c := &w.contacts[id]
	if c.contactId != id || c.generation != contactId.generation {
		panic("dbox2d: the contact id is stale")
	}
	return c
}

// IsValid reports whether the id references a live contact. It provides
// validation for up to 2^32 allocations per slot. It corresponds to
// b2Contact_IsValid.
func (contactId ContactId) IsValid() bool {
	if maxWorlds <= contactId.world0 {
		return false
	}

	w := &worlds[contactId.world0]
	if w.worldId != contactId.world0 {
		// world is free
		return false
	}

	id := int(contactId.index1) - 1
	if id < 0 || len(w.contacts) <= id {
		return false
	}

	c := &w.contacts[id]
	if c.contactId == nullIndex {
		// contact is free
		return false
	}

	if c.contactId != id {
		panic("dbox2d: a live contact has the wrong id")
	}

	return contactId.generation == c.generation
}

// GetData returns the shapes and the manifold of a contact. The manifold
// may have no points if the contact is not touching. It corresponds to
// b2Contact_GetData.
func (contactId ContactId) GetData() ContactData {
	w := getWorld(contactId.world0)
	c := getContactFullId(w, contactId)
	return ContactData{
		ContactId: contactId,
		ShapeIdA:  shapeIdOf(w, &w.shapes[c.shapeIdA]),
		ShapeIdB:  shapeIdOf(w, &w.shapes[c.shapeIdB]),
		Manifold:  getContactSim(w, c).manifold,
	}
}

// contactSim is the warm contact data that the narrowphase and the solver
// consume. It corresponds to b2ContactSim in src/contact.h.
type contactSim struct {
	contactId int

	bodySimIndexA int
	bodySimIndexB int

	shapeIdA int
	shapeIdB int

	// Inverse mass and inertia copies, filled when the contact enters the
	// constraint graph.
	invMassA Q
	invIA    Q
	invMassB Q
	invIB    Q

	manifold Manifold

	// Mixed friction and restitution.
	friction    Q
	restitution Q

	rollingResistance Q
	tangentSpeed      Q

	simFlags uint32

	cache SimplexCache

	// The body rotations and the relative transform at the last manifold
	// update, for contact recycling.
	cachedRotationA    Rot
	cachedRotationB    Rot
	cachedRelativePose Transform
}

// manifoldFcn computes the manifold of one ordered shape type pair. It
// corresponds to b2ManifoldFcn in src/contact.c.
//
// The result is in the frame of shape A and returns by value: an out
// pointer through this indirect call would move the caller's local to the
// heap.
type manifoldFcn func(shapeA *shape, shapeB *shape, xf Transform, cache *SimplexCache) LocalManifold

// contactRegister pairs the collide function with the order of its
// arguments. It corresponds to b2ContactRegister in src/contact.c.
type contactRegister struct {
	fcn     manifoldFcn
	primary bool
}

// contactRegisters is the dispatch table by shape type pair. A nil entry
// means the pair does not collide. It corresponds to s_registers in
// src/contact.c.
var contactRegisters [ShapeTypeCount][ShapeTypeCount]contactRegister

func circleManifold(shapeA *shape, shapeB *shape, xf Transform, _ *SimplexCache) (manifold LocalManifold) {
	return CollideCircles(shapeA.circle(), shapeB.circle(), xf)
}

func capsuleAndCircleManifold(shapeA *shape, shapeB *shape, xf Transform, _ *SimplexCache) (manifold LocalManifold) {
	return CollideCapsuleAndCircle(shapeA.capsule(), shapeB.circle(), xf)
}

func capsuleManifold(shapeA *shape, shapeB *shape, xf Transform, _ *SimplexCache) (manifold LocalManifold) {
	return CollideCapsules(shapeA.capsule(), shapeB.capsule(), xf)
}

func polygonAndCircleManifold(shapeA *shape, shapeB *shape, xf Transform, _ *SimplexCache) (manifold LocalManifold) {
	return CollidePolygonAndCircle(shapeA.polygon(), shapeB.circle(), xf)
}

func polygonAndCapsuleManifold(shapeA *shape, shapeB *shape, xf Transform, _ *SimplexCache) (manifold LocalManifold) {
	polyB := makeCapsule(shapeB.capsule().Center1, shapeB.capsule().Center2, shapeB.capsule().Radius)
	collidePolygonsInto(&manifold, shapeA.polygon(), &polyB, xf)
	return manifold
}

func polygonManifold(shapeA *shape, shapeB *shape, xf Transform, _ *SimplexCache) (manifold LocalManifold) {
	collidePolygonsInto(&manifold, shapeA.polygon(), shapeB.polygon(), xf)
	return manifold
}

func segmentAndCircleManifold(shapeA *shape, shapeB *shape, xf Transform, _ *SimplexCache) (manifold LocalManifold) {
	return CollideSegmentAndCircle(shapeA.segment(), shapeB.circle(), xf)
}

func segmentAndCapsuleManifold(shapeA *shape, shapeB *shape, xf Transform, _ *SimplexCache) (manifold LocalManifold) {
	return CollideSegmentAndCapsule(shapeA.segment(), shapeB.capsule(), xf)
}

func segmentAndPolygonManifold(shapeA *shape, shapeB *shape, xf Transform, _ *SimplexCache) (manifold LocalManifold) {
	polygonA := makeCapsule(shapeA.segment().Point1, shapeA.segment().Point2, QZero())
	collidePolygonsInto(&manifold, &polygonA, shapeB.polygon(), xf)
	return manifold
}

func chainSegmentAndCircleManifold(shapeA *shape, shapeB *shape, xf Transform, _ *SimplexCache) (manifold LocalManifold) {
	return CollideChainSegmentAndCircle(shapeA.chainSegment(), shapeB.circle(), xf)
}

func chainSegmentAndCapsuleManifold(shapeA *shape, shapeB *shape, xf Transform, cache *SimplexCache) (manifold LocalManifold) {
	return CollideChainSegmentAndCapsule(shapeA.chainSegment(), shapeB.capsule(), xf, cache)
}

func chainSegmentAndPolygonManifold(shapeA *shape, shapeB *shape, xf Transform, cache *SimplexCache) (manifold LocalManifold) {
	return CollideChainSegmentAndPolygon(shapeA.chainSegment(), shapeB.polygon(), xf, cache)
}

// addType registers a collide function for a type pair, in both orders.
// It corresponds to b2AddType in src/contact.c.
func addType(fcn manifoldFcn, type1, type2 ShapeType) {
	contactRegisters[type1][type2] = contactRegister{fcn: fcn, primary: true}
	if type1 != type2 {
		contactRegisters[type2][type1] = contactRegister{fcn: fcn, primary: false}
	}
}

// The lazy flag of b2InitializeContactRegisters becomes a package init,
// which runs once and in a deterministic order. Pairs without an entry,
// such as segment against segment, make createContact skip the pair.
func init() {
	addType(circleManifold, CircleShape, CircleShape)
	addType(capsuleAndCircleManifold, CapsuleShape, CircleShape)
	addType(capsuleManifold, CapsuleShape, CapsuleShape)
	addType(polygonAndCircleManifold, PolygonShape, CircleShape)
	addType(polygonAndCapsuleManifold, PolygonShape, CapsuleShape)
	addType(polygonManifold, PolygonShape, PolygonShape)
	addType(segmentAndCircleManifold, SegmentShape, CircleShape)
	addType(segmentAndCapsuleManifold, SegmentShape, CapsuleShape)
	addType(segmentAndPolygonManifold, SegmentShape, PolygonShape)
	addType(chainSegmentAndCircleManifold, ChainSegmentShape, CircleShape)
	addType(chainSegmentAndCapsuleManifold, ChainSegmentShape, CapsuleShape)
	addType(chainSegmentAndPolygonManifold, ChainSegmentShape, PolygonShape)
}

// createContact makes a non-touching contact between two shapes and links
// it to both bodies. It corresponds to b2CreateContact in src/contact.c.
func createContact(w *world, shapeA, shapeB *shape) {
	type1 := shapeA.shapeType
	type2 := shapeB.shapeType

	if contactRegisters[type1][type2].fcn == nil {
		// For example, no segment versus segment collision.
		return
	}

	if !contactRegisters[type1][type2].primary {
		// Flip the order.
		createContact(w, shapeB, shapeA)
		return
	}

	bodyA := &w.bodies[shapeA.bodyId]
	bodyB := &w.bodies[shapeB.bodyId]

	if bodyA.setIndex == disabledSet || bodyB.setIndex == disabledSet {
		panic("dbox2d: a disabled body cannot receive a contact")
	}
	if bodyA.setIndex == staticSet && bodyB.setIndex == staticSet {
		panic("dbox2d: two static bodies cannot touch")
	}

	setIndex := awakeSet
	if bodyA.setIndex != awakeSet && bodyB.setIndex != awakeSet {
		// Sleeping and non-touching contacts live in the disabled set.
		// A touching contact later links the sleeping islands.
		setIndex = disabledSet
	}

	set := &w.solverSets[setIndex]

	// Create the contact key and the contact.
	contactId := w.contactIdPool.allocId()
	if contactId == len(w.contacts) {
		w.contacts = append(w.contacts, contact{})
	}

	shapeIdA := shapeA.id
	shapeIdB := shapeB.id

	c := &w.contacts[contactId]
	c.contactId = contactId
	c.generation++
	c.setIndex = setIndex
	c.colorIndex = nullIndex
	c.localIndex = len(set.contactSims)
	c.islandId = nullIndex
	c.islandIndex = nullIndex
	c.shapeIdA = shapeIdA
	c.shapeIdB = shapeIdB
	c.flags = 0

	// Both bodies must enable recycling
	if bodyA.flags&bodyEnableContactRecycling != 0 && bodyB.flags&bodyEnableContactRecycling != 0 {
		c.flags |= contactRecycleFlag
	}

	if shapeA.sensorIndex != nullIndex || shapeB.sensorIndex != nullIndex {
		panic("dbox2d: a sensor shape cannot receive a contact")
	}

	if shapeA.enableContactEvents || shapeB.enableContactEvents {
		c.flags |= contactEnableContactEvents
	}

	// Connect to body A.
	{
		c.edges[0].bodyId = shapeA.bodyId
		c.edges[0].prevKey = nullIndex
		c.edges[0].nextKey = bodyA.headContactKey

		keyA := contactId << 1
		headContactKey := bodyA.headContactKey
		if headContactKey != nullIndex {
			headContact := &w.contacts[headContactKey>>1]
			headContact.edges[headContactKey&1].prevKey = keyA
		}
		bodyA.headContactKey = keyA
		bodyA.contactCount++
	}

	// Connect to body B.
	{
		c.edges[1].bodyId = shapeB.bodyId
		c.edges[1].prevKey = nullIndex
		c.edges[1].nextKey = bodyB.headContactKey

		keyB := contactId<<1 | 1
		headContactKey := bodyB.headContactKey
		if headContactKey != nullIndex {
			headContact := &w.contacts[headContactKey>>1]
			headContact.edges[headContactKey&1].prevKey = keyB
		}
		bodyB.headContactKey = keyB
		bodyB.contactCount++
	}

	// Add to the pair set for fast lookup. The reference hosts the set on
	// the broadphase; it moves there when the broadphase lands.
	w.broadPhase.pairSet.addKey(shapePairKey(shapeIdA, shapeIdB))

	// Contacts start non-touching. A touching contact later links islands
	// and moves into the constraint graph.
	set.contactSims = append(set.contactSims, contactSim{})
	cs := &set.contactSims[len(set.contactSims)-1]
	cs.contactId = contactId
	cs.bodySimIndexA = nullIndex
	cs.bodySimIndexB = nullIndex
	cs.shapeIdA = shapeIdA
	cs.shapeIdB = shapeIdB

	// The narrowphase keeps these updated as well.
	cs.friction = w.frictionCallback(shapeA.friction, shapeA.userMaterialId, shapeB.friction, shapeB.userMaterialId)
	cs.restitution = w.restitutionCallback(shapeA.restitution, shapeA.userMaterialId, shapeB.restitution, shapeB.userMaterialId)

	// The sim inherits the contact flags, the recycle flag among them.
	cs.simFlags = c.flags

	if shapeA.enablePreSolveEvents || shapeB.enablePreSolveEvents {
		cs.simFlags |= simEnablePreSolveEvents
	}
}

// destroyContact unlinks a contact from both bodies and frees it.
// wakeBodies wakes the bodies of a touching contact. The end touch event
// waits for the events. It corresponds to b2DestroyContact in
// src/contact.c.
func destroyContact(w *world, c *contact, wakeBodies bool) {
	// Remove the pair from the set.
	w.broadPhase.pairSet.removeKey(shapePairKey(c.shapeIdA, c.shapeIdB))

	edgeA := &c.edges[0]
	edgeB := &c.edges[1]

	bodyA := &w.bodies[edgeA.bodyId]
	bodyB := &w.bodies[edgeB.bodyId]

	flags := c.flags
	touching := flags&contactTouchingFlag != 0

	// End touch event
	if touching && flags&contactEnableContactEvents != 0 {
		shapeA := &w.shapes[c.shapeIdA]
		shapeB := &w.shapes[c.shapeIdB]
		event := ContactEndTouchEvent{ShapeIdA: shapeIdOf(w, shapeA), ShapeIdB: shapeIdOf(w, shapeB), ContactId: contactIdOf(w, c)}
		w.contactEndEvents[w.endEventArrayIndex] = append(w.contactEndEvents[w.endEventArrayIndex], event)
	}

	// Remove from body A.
	if edgeA.prevKey != nullIndex {
		prevContact := &w.contacts[edgeA.prevKey>>1]
		prevContact.edges[edgeA.prevKey&1].nextKey = edgeA.nextKey
	}
	if edgeA.nextKey != nullIndex {
		nextContact := &w.contacts[edgeA.nextKey>>1]
		nextContact.edges[edgeA.nextKey&1].prevKey = edgeA.prevKey
	}

	contactId := c.contactId

	if bodyA.headContactKey == contactId<<1 {
		bodyA.headContactKey = edgeA.nextKey
	}
	bodyA.contactCount--

	// Remove from body B.
	if edgeB.prevKey != nullIndex {
		prevContact := &w.contacts[edgeB.prevKey>>1]
		prevContact.edges[edgeB.prevKey&1].nextKey = edgeB.nextKey
	}
	if edgeB.nextKey != nullIndex {
		nextContact := &w.contacts[edgeB.nextKey>>1]
		nextContact.edges[edgeB.nextKey&1].prevKey = edgeB.prevKey
	}

	if bodyB.headContactKey == contactId<<1|1 {
		bodyB.headContactKey = edgeB.nextKey
	}
	bodyB.contactCount--

	// Remove contact from the array that owns it
	if c.islandId != nullIndex {
		unlinkContact(w, c)
	}

	if c.colorIndex != nullIndex {
		// contact is an active constraint
		if c.setIndex != awakeSet {
			panic("dbox2d: a graph contact lives in the awake set")
		}
		removeContactFromGraph(w, edgeA.bodyId, edgeB.bodyId, c.colorIndex, c.localIndex)
	} else {
		// contact is non-touching or is sleeping
		if c.setIndex == awakeSet && touching {
			panic("dbox2d: a touching awake contact is outside the graph")
		}
		set := &w.solverSets[c.setIndex]
		var movedIndex int
		set.contactSims, movedIndex = removeSwapNoClear(set.contactSims, c.localIndex)
		if movedIndex != nullIndex {
			movedContactSim := &set.contactSims[c.localIndex]
			movedContact := &w.contacts[movedContactSim.contactId]
			movedContact.localIndex = c.localIndex
		}
	}

	c.contactId = nullIndex
	c.setIndex = nullIndex
	c.colorIndex = nullIndex
	c.localIndex = nullIndex

	w.contactIdPool.freeId(contactId)

	if wakeBodies && touching {
		wakeBody(w, bodyA)
		wakeBody(w, bodyB)
	}
}

// getContactSim returns the warm data of a contact. It corresponds to
// b2GetContactSim in src/contact.c.
func getContactSim(w *world, c *contact) *contactSim {
	if c.setIndex == awakeSet && c.colorIndex != nullIndex {
		// contact lives in constraint graph
		if c.colorIndex < 0 || c.colorIndex >= graphColorCount {
			panic("dbox2d: the color index is out of range")
		}
		color := &w.constraintGraph.colors[c.colorIndex]
		return &color.contactSims[c.localIndex]
	}

	set := &w.solverSets[c.setIndex]
	return &set.contactSims[c.localIndex]
}

// updateContact recomputes the manifold and the touching status, and
// matches the point ids of the old manifold to carry the stored impulses
// into the warm start. The AABBs of the shapes may not overlap. It
// corresponds to b2UpdateContact in src/contact.c.
func updateContact(w *world, cs *contactSim, shapeA *shape, transformA Transform, centerOffsetA Vec2,
	shapeB *shape, transformB Transform, centerOffsetB Vec2) bool {
	// Save the old manifold.
	oldManifold := cs.manifold

	// Compute the manifold in frame A, then convert it to world anchors
	// relative to each shape origin. The relative transform subtracts the
	// two world positions before the narrow phase, so precision holds far
	// from the origin. anchorB = rot(qA, localPoint) + pA - pB.
	relativeTransform := InvMulTransforms(transformA, transformB)
	fcn := contactRegisters[shapeA.shapeType][shapeB.shapeType].fcn
	local := fcn(shapeA, shapeB, relativeTransform, &cs.cache)

	cs.manifold = Manifold{}
	cs.manifold.Normal = RotateVector(transformA.Q, local.Normal)
	cs.manifold.PointCount = local.PointCount

	originDelta := transformA.P.Sub(transformB.P)
	for i := range local.PointCount {
		mp := &cs.manifold.Points[i]
		mp.AnchorA = RotateVector(transformA.Q, local.Points[i].Point)
		mp.AnchorB = mp.AnchorA.Add(originDelta)
		mp.Separation = local.Points[i].Separation
		mp.Id = local.Points[i].Id
	}

	// Keep these updated in case the shape values changed.
	cs.friction = w.frictionCallback(shapeA.friction, shapeA.userMaterialId, shapeB.friction, shapeB.userMaterialId)
	cs.restitution = w.restitutionCallback(shapeA.restitution, shapeA.userMaterialId, shapeB.restitution, shapeB.userMaterialId)

	zero := QZero()
	if zero.Less(shapeA.rollingResistance) || zero.Less(shapeB.rollingResistance) {
		radiusA := getShapeRadius(shapeA)
		radiusB := getShapeRadius(shapeB)
		maxRadius := radiusA.Max(radiusB)
		cs.rollingResistance = shapeA.rollingResistance.Max(shapeB.rollingResistance).Mul(maxRadius)
	} else {
		cs.rollingResistance = zero
	}

	cs.tangentSpeed = shapeA.tangentSpeed.Add(shapeB.tangentSpeed)

	pointCount := cs.manifold.PointCount
	touching := pointCount > 0

	if touching && w.preSolveFcn != nil && cs.simFlags&simEnablePreSolveEvents != 0 {
		idA := ShapeId{index1: int32(shapeA.id) + 1, world0: w.worldId, generation: shapeA.generation}
		idB := ShapeId{index1: int32(shapeB.id) + 1, world0: w.worldId, generation: shapeB.generation}

		// This call assumes thread safety.
		w.preSolveFcn(idA, idB, &cs.manifold)

		// Keep these current.
		pointCount = cs.manifold.PointCount
		touching = pointCount > 0

		// Mark recycling dirty.
		if !touching {
			cs.simFlags &^= simRelativeTransformValid
		}
	}

	// This flag exists for testing. The reference tests point zero in both
	// branches, so only the first branch can fire; the port keeps the
	// behaviour without the dead branch.
	if !w.enableSpeculative && pointCount == 2 {
		slop := LinearSlop()
		if slop.Add(slop.Div(QFromInt(2))).Less(cs.manifold.Points[0].Separation) {
			cs.manifold.Points[0] = cs.manifold.Points[1]
			cs.manifold.PointCount = 1
		}

		pointCount = cs.manifold.PointCount
	}

	if touching && (shapeA.enableHitEvents || shapeB.enableHitEvents) {
		cs.simFlags |= simEnableHitEvent
	} else {
		cs.simFlags &^= simEnableHitEvent
	}

	if pointCount > 0 {
		cs.manifold.RollingImpulse = oldManifold.RollingImpulse
	}

	// Match the old contact ids to the new contact ids and copy the stored
	// impulses to warm start the solver.
	for i := range pointCount {
		mp2 := &cs.manifold.Points[i]

		// Shift the anchors to be relative to the centers of mass.
		mp2.AnchorA = mp2.AnchorA.Sub(centerOffsetA)
		mp2.AnchorB = mp2.AnchorB.Sub(centerOffsetB)

		mp2.NormalImpulse = zero
		mp2.TangentImpulse = zero
		mp2.TotalNormalImpulse = zero
		mp2.NormalVelocity = zero
		mp2.Persisted = false

		for j := range oldManifold.PointCount {
			mp1 := &oldManifold.Points[j]

			if mp1.Id == mp2.Id {
				mp2.NormalImpulse = mp1.NormalImpulse
				mp2.TangentImpulse = mp1.TangentImpulse
				mp2.Persisted = true
				break
			}
		}
	}

	if touching {
		cs.simFlags |= simTouchingFlag
	} else {
		cs.simFlags &^= simTouchingFlag
	}

	return touching
}

// computeManifold computes a temporary manifold through the contact dispatch
// table. It corresponds to b2ComputeManifold in src/contact.c and starts with
// an empty simplex cache because the result is not a persistent contact.
func computeManifold(shapeA *shape, transformA Transform, shapeB *shape, transformB Transform) Manifold {
	fcn := contactRegisters[shapeA.shapeType][shapeB.shapeType].fcn
	cache := SimplexCache{}
	local := fcn(shapeA, shapeB, InvMulTransforms(transformA, transformB), &cache)

	var manifold Manifold
	manifold.Normal = RotateVector(transformA.Q, local.Normal)
	manifold.PointCount = local.PointCount
	originDelta := transformA.P.Sub(transformB.P)
	for i := range local.PointCount {
		mp := &manifold.Points[i]
		mp.AnchorA = RotateVector(transformA.Q, local.Points[i].Point)
		mp.AnchorB = mp.AnchorA.Add(originDelta)
		mp.Separation = local.Points[i].Separation
		mp.Id = local.Points[i].Id
	}
	return manifold
}
