package dbox2d

import "github.com/dhannyell/fixed"

// Contact flags of the cold data. They correspond to b2ContactFlags in
// src/contact.h. The touching flag arrives with the narrowphase update of
// the step; the hit event flag arrives with the solver events.
const (
	// contactTouchingFlag is set when the shapes touch.
	contactTouchingFlag uint32 = 0x0001

	// contactEnableContactEvents marks a contact that wants contact events.
	contactEnableContactEvents uint32 = 0x0004
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

	// A contact belongs to an island only when touching. The island fields
	// stay nullIndex until the island stage lands.
	islandPrev int
	islandNext int
	islandId   int

	contactId int

	flags uint32

	isMarked bool
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
}

// manifoldFcn computes the manifold of one ordered shape type pair. It
// corresponds to b2ManifoldFcn in src/contact.c.
type manifoldFcn func(shapeA *shape, xfA Transform, shapeB *shape, xfB Transform, cache *SimplexCache) Manifold

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

func circleManifold(shapeA *shape, xfA Transform, shapeB *shape, xfB Transform, _ *SimplexCache) Manifold {
	return CollideCircles(&shapeA.circle, xfA, &shapeB.circle, xfB)
}

func capsuleAndCircleManifold(shapeA *shape, xfA Transform, shapeB *shape, xfB Transform, _ *SimplexCache) Manifold {
	return CollideCapsuleAndCircle(&shapeA.capsule, xfA, &shapeB.circle, xfB)
}

func capsuleManifold(shapeA *shape, xfA Transform, shapeB *shape, xfB Transform, _ *SimplexCache) Manifold {
	return CollideCapsules(&shapeA.capsule, xfA, &shapeB.capsule, xfB)
}

func polygonAndCircleManifold(shapeA *shape, xfA Transform, shapeB *shape, xfB Transform, _ *SimplexCache) Manifold {
	return CollidePolygonAndCircle(&shapeA.polygon, xfA, &shapeB.circle, xfB)
}

func polygonAndCapsuleManifold(shapeA *shape, xfA Transform, shapeB *shape, xfB Transform, _ *SimplexCache) Manifold {
	return CollidePolygonAndCapsule(&shapeA.polygon, xfA, &shapeB.capsule, xfB)
}

func polygonManifold(shapeA *shape, xfA Transform, shapeB *shape, xfB Transform, _ *SimplexCache) Manifold {
	return CollidePolygons(&shapeA.polygon, xfA, &shapeB.polygon, xfB)
}

func segmentAndCircleManifold(shapeA *shape, xfA Transform, shapeB *shape, xfB Transform, _ *SimplexCache) Manifold {
	return CollideSegmentAndCircle(&shapeA.segment, xfA, &shapeB.circle, xfB)
}

func segmentAndCapsuleManifold(shapeA *shape, xfA Transform, shapeB *shape, xfB Transform, _ *SimplexCache) Manifold {
	return CollideSegmentAndCapsule(&shapeA.segment, xfA, &shapeB.capsule, xfB)
}

func segmentAndPolygonManifold(shapeA *shape, xfA Transform, shapeB *shape, xfB Transform, _ *SimplexCache) Manifold {
	return CollideSegmentAndPolygon(&shapeA.segment, xfA, &shapeB.polygon, xfB)
}

func chainSegmentAndCircleManifold(shapeA *shape, xfA Transform, shapeB *shape, xfB Transform, _ *SimplexCache) Manifold {
	return CollideChainSegmentAndCircle(&shapeA.chainSegment, xfA, &shapeB.circle, xfB)
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
// which runs once and in a deterministic order. The chain segment pairs
// against capsule and polygon wait for the iterative distance solver; their
// nil entries make createContact skip the pair.
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
	c.setIndex = setIndex
	c.colorIndex = nullIndex
	c.localIndex = len(set.contactSims)
	c.islandId = nullIndex
	c.islandPrev = nullIndex
	c.islandNext = nullIndex
	c.shapeIdA = shapeIdA
	c.shapeIdB = shapeIdB
	c.isMarked = false
	c.flags = 0

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
		event := ContactEndTouchEvent{ShapeIdA: shapeIdOf(w, shapeA), ShapeIdB: shapeIdOf(w, shapeB)}
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
		set.contactSims, movedIndex = removeSwap(set.contactSims, c.localIndex)
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

	// Compute the new manifold.
	fcn := contactRegisters[shapeA.shapeType][shapeB.shapeType].fcn
	cs.manifold = fcn(shapeA, transformA, shapeB, transformB, &cs.cache)

	// Keep these updated in case the shape values changed.
	cs.friction = w.frictionCallback(shapeA.friction, shapeA.userMaterialId, shapeB.friction, shapeB.userMaterialId)
	cs.restitution = w.restitutionCallback(shapeA.restitution, shapeA.userMaterialId, shapeB.restitution, shapeB.userMaterialId)

	zero := fixed.Q32Zero()
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

	// The pre-solve callback waits for the event stage.

	// This flag exists for testing. The reference tests point zero in both
	// branches, so only the first branch can fire; the port keeps the
	// behaviour without the dead branch.
	if !w.enableSpeculative && pointCount == 2 {
		slop := LinearSlop()
		if slop.Add(slop.Div(fixed.Q32FromInt(2))).Less(cs.manifold.Points[0].Separation) {
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

				// Clear the old impulse, so a duplicate id cannot reuse it.
				mp1.NormalImpulse = zero
				mp1.TangentImpulse = zero
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
	return fcn(shapeA, transformA, shapeB, transformB, &cache)
}
