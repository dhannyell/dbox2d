package dbox2d

import "github.com/dhannyell/fixed"

// The solver set positions of src/world.h. The first sets have fixed
// indices; every later index is a sleeping island set.
const (
	staticSet        = 0
	disabledSet      = 1
	awakeSet         = 2
	firstSleepingSet = 3
)

// world manages all physics entities and the dynamic simulation.
type world struct {
	// Deferred: the broad-phase, the joint, chain and sensor storage, the
	// events, the callbacks and the task system of the reference.

	// constraintGraph colors the awake touching contacts.
	constraintGraph constraintGraph

	// arena is the scratch allocator of one step.
	arena arenaAllocator

	// bodyIdPool allocates and recycles body ids. An id gives the
	// application a stable identifier; the sim data moves between sets.
	bodyIdPool idPool

	// bodies maps a body id to the body data. It is sparse: the sims live
	// in the solver sets.
	bodies []body

	// solverSetIdPool provides a free list for solver sets.
	solverSetIdPool idPool

	// solverSets stores the sims in contiguous arrays. See solverSet.
	solverSets []solverSet

	shapeIdPool idPool

	// shapes maps a shape id to the shape data.
	shapes []shape

	contactIdPool idPool

	// contacts maps a contact id to the cold contact data. The sims live in
	// the solver sets.
	contacts []contact

	islandIdPool idPool

	// islands maps an island id to the island data. The sims live in the
	// solver sets.
	islands []island

	// splitIslandId is the island chosen for a split on the next step, or
	// nullIndex.
	splitIslandId int

	// taskContext is the per-worker scratch of the reference. The port has
	// one worker.
	taskContext taskContext

	// The event arrays of the last step. The end events use two buffers,
	// because a contact destroyed between steps reports into the buffer
	// that the next step returns.
	bodyMoveEvents     []BodyMoveEvent
	contactBeginEvents []ContactBeginTouchEvent
	contactEndEvents   [2][]ContactEndTouchEvent
	contactHitEvents   []ContactHitEvent
	endEventArrayIndex int

	// Deferred: the sensor events of the reference.

	// broadPhase holds the trees, the move buffer and the pair set.
	broadPhase broadPhase

	// stepIndex advances once per step.
	stepIndex uint64

	gravity              Vec2
	hitEventThreshold    Q
	restitutionThreshold Q
	maxLinearSpeed       Q
	maxContactPushSpeed  Q

	// contactSpeed is derived from the contact tuning on each step.
	contactSpeed        Q
	contactHertz        Q
	contactDampingRatio Q

	// generation advances on each destruction of this slot, so a stale
	// WorldId fails validation.
	generation uint16

	userData any

	// inv_h is the inverse time step of the last step, for force reports.
	invH Q

	// The mixing callbacks combine the material values of two shapes into
	// the effective contact values.
	frictionCallback    mixingCallback
	restitutionCallback mixingCallback

	worldId uint16

	enableSleep        bool
	locked             bool
	enableWarmStarting bool
	enableContinuous   bool
	enableSpeculative  bool
	inUse              bool
}

// mixingCallback combines the material values of two shapes. It corresponds
// to b2FrictionCallback and b2RestitutionCallback in include/box2d/types.h.
type mixingCallback func(valueA Q, userMaterialIdA int, valueB Q, userMaterialIdB int) Q

// defaultFrictionCallback is the geometric mean. It corresponds to
// b2DefaultFrictionCallback in src/world.c.
func defaultFrictionCallback(frictionA Q, _ int, frictionB Q, _ int) Q {
	return frictionA.Mul(frictionB).Sqrt()
}

// defaultRestitutionCallback is the maximum. It corresponds to
// b2DefaultRestitutionCallback in src/world.c.
func defaultRestitutionCallback(restitutionA Q, _ int, restitutionB Q, _ int) Q {
	return restitutionA.Max(restitutionB)
}

// worlds is the global registry. A WorldId indexes into it, so world zero
// with generation zero stays distinguishable from the null id by index1.
var worlds [maxWorlds]world

// getWorldFromId returns the world of an id. It panics on a stale id.
func getWorldFromId(id WorldId) *world {
	if id.index1 < 1 || maxWorlds < id.index1 {
		panic("dbox2d: the WorldId index is out of range")
	}
	w := &worlds[id.index1-1]
	if id.index1 != w.worldId+1 {
		panic("dbox2d: the world is not allocated")
	}
	if id.generation != w.generation {
		panic("dbox2d: the WorldId is stale")
	}
	return w
}

// getWorld returns the world at a raw index. It panics on a freed slot.
func getWorld(index uint16) *world {
	if index >= maxWorlds {
		panic("dbox2d: the world index is out of range")
	}
	w := &worlds[index]
	if w.worldId != index {
		panic("dbox2d: the world is not allocated")
	}
	return w
}

// getWorldLocked returns the world at a raw index. It panics on a freed slot
// and on a locked world, because a callback must not mutate the world.
func getWorldLocked(index uint16) *world {
	w := getWorld(index)
	if w.locked {
		panic("dbox2d: the world is locked")
	}
	return w
}

// CreateWorld creates a world from a definition. The definition must come
// from DefaultWorldDef and is not retained. It returns the null id when
// every slot of the registry is in use.
func CreateWorld(def *WorldDef) WorldId {
	checkDef(def.internalValue)

	worldId := nullIndex
	for i := range maxWorlds {
		if !worlds[i].inUse {
			worldId = i
			break
		}
	}

	if worldId == nullIndex {
		return WorldId{}
	}

	w := &worlds[worldId]
	generation := w.generation

	*w = world{}

	w.worldId = uint16(worldId)
	w.generation = generation
	w.inUse = true

	// pools
	w.bodyIdPool = createIdPool()
	w.bodies = make([]body, 0, 16)
	w.solverSets = make([]solverSet, 0, 8)

	// add empty static, disabled and awake body sets
	w.solverSetIdPool = createIdPool()

	// static set
	w.solverSets = append(w.solverSets, solverSet{setIndex: w.solverSetIdPool.allocId()})
	if w.solverSets[staticSet].setIndex != staticSet {
		panic("dbox2d: the static set index is wrong")
	}

	// disabled set
	w.solverSets = append(w.solverSets, solverSet{setIndex: w.solverSetIdPool.allocId()})
	if w.solverSets[disabledSet].setIndex != disabledSet {
		panic("dbox2d: the disabled set index is wrong")
	}

	// awake set
	w.solverSets = append(w.solverSets, solverSet{setIndex: w.solverSetIdPool.allocId()})
	if w.solverSets[awakeSet].setIndex != awakeSet {
		panic("dbox2d: the awake set index is wrong")
	}

	w.shapeIdPool = createIdPool()
	w.shapes = make([]shape, 0, 16)

	w.contactIdPool = createIdPool()
	w.contacts = make([]contact, 0, 16)
	createBroadPhase(&w.broadPhase)

	w.islandIdPool = createIdPool()
	w.islands = make([]island, 0, 8)

	w.arena = createArenaAllocator(2048)

	createGraph(&w.constraintGraph, 16)

	w.splitIslandId = nullIndex

	w.taskContext.contactStateBitSet = createBitSet(1024)
	w.taskContext.enlargedSimBitSet = createBitSet(256)
	w.taskContext.awakeIslandBitSet = createBitSet(256)
	w.taskContext.splitIslandId = nullIndex

	w.bodyMoveEvents = make([]BodyMoveEvent, 0, 16)
	w.contactBeginEvents = make([]ContactBeginTouchEvent, 0, 16)
	w.contactEndEvents[0] = make([]ContactEndTouchEvent, 0, 16)
	w.contactEndEvents[1] = make([]ContactEndTouchEvent, 0, 16)
	w.contactHitEvents = make([]ContactHitEvent, 0, 16)
	w.endEventArrayIndex = 0

	w.frictionCallback = defaultFrictionCallback
	w.restitutionCallback = defaultRestitutionCallback

	w.stepIndex = 0
	w.gravity = def.Gravity
	w.hitEventThreshold = def.HitEventThreshold
	w.restitutionThreshold = def.RestitutionThreshold
	w.maxLinearSpeed = def.MaximumLinearSpeed
	w.maxContactPushSpeed = def.MaxContactPushSpeed
	w.contactHertz = def.ContactHertz
	w.contactDampingRatio = def.ContactDampingRatio

	w.enableSleep = def.EnableSleep
	w.locked = false
	w.enableWarmStarting = true
	w.enableContinuous = def.EnableContinuous
	w.enableSpeculative = true
	w.userData = def.UserData

	// add one to the world id so that 0 represents a null WorldId
	return WorldId{index1: uint16(worldId) + 1, generation: w.generation}
}

// DestroyWorld destroys a world and every body and shape in it.
func DestroyWorld(worldId WorldId) {
	w := getWorldFromId(worldId)

	destroyGraph(&w.constraintGraph)

	// Destroy solver sets
	for i := range w.solverSets {
		if w.solverSets[i].setIndex != nullIndex {
			destroySolverSet(w, i)
		}
	}

	w.bodyIdPool.destroy()
	w.shapeIdPool.destroy()
	w.contactIdPool.destroy()
	w.islandIdPool.destroy()
	w.solverSetIdPool.destroy()

	destroyBroadPhase(&w.broadPhase)
	destroyArenaAllocator(&w.arena)

	// Wipe world but preserve generation
	generation := w.generation
	*w = world{}
	w.worldId = 0
	w.generation = generation + 1
}

// IsValid reports whether the id references a live world. It lets an
// application hold ids safely across destruction.
func (id WorldId) IsValid() bool {
	if id.index1 < 1 || maxWorlds < id.index1 {
		return false
	}

	w := &worlds[id.index1-1]

	if w.worldId != id.index1-1 {
		// world is not allocated
		return false
	}

	return id.generation == w.generation
}

// IsValid reports whether the id references a live body.
func (id BodyId) IsValid() bool {
	if maxWorlds <= id.world0 {
		// invalid world
		return false
	}

	w := &worlds[id.world0]
	if w.worldId != id.world0 {
		// world is free
		return false
	}

	if id.index1 < 1 || len(w.bodies) < int(id.index1) {
		// invalid index
		return false
	}

	b := &w.bodies[id.index1-1]
	if b.setIndex == nullIndex {
		// this was freed
		return false
	}

	if b.localIndex == nullIndex {
		panic("dbox2d: a live body has no local index")
	}

	if b.generation != id.generation {
		// this id is orphaned
		return false
	}

	return true
}

// IsValid reports whether the id references a live shape.
func (id ShapeId) IsValid() bool {
	if maxWorlds <= id.world0 {
		return false
	}

	w := &worlds[id.world0]
	if w.worldId != id.world0 {
		// world is free
		return false
	}

	shapeId := int(id.index1) - 1
	if shapeId < 0 || len(w.shapes) <= shapeId {
		return false
	}

	s := &w.shapes[shapeId]
	if s.id == nullIndex {
		// shape is free
		return false
	}

	if s.id != shapeId {
		panic("dbox2d: the shape id does not match its slot")
	}

	return id.generation == s.generation
}

// validateSolverSets checks the bijection between the sparse arrays and the
// solver sets. The reference compiles it only into validation builds; here
// only the tests call it.
func validateSolverSets(w *world) {
	if w.bodyIdPool.idCapacity() != len(w.bodies) {
		panic("dbox2d: the body pool and the body array disagree")
	}
	if w.solverSetIdPool.idCapacity() != len(w.solverSets) {
		panic("dbox2d: the set pool and the set array disagree")
	}
	if w.islandIdPool.idCapacity() != len(w.islands) {
		panic("dbox2d: the island pool and the island array disagree")
	}

	if w.contactIdPool.idCapacity() != len(w.contacts) {
		panic("dbox2d: the contact pool and the contact array disagree")
	}

	activeSetCount := 0
	totalBodyCount := 0
	totalContactCount := 0
	totalIslandCount := 0

	// Validate all solver sets
	for setIndex := range w.solverSets {
		set := &w.solverSets[setIndex]
		if set.setIndex != nullIndex {
			activeSetCount += 1

			switch setIndex {
			case staticSet:
				if len(set.contactSims) != 0 || len(set.islandSims) != 0 || len(set.bodyStates) != 0 {
					panic("dbox2d: the static set holds contacts, islands or states")
				}
			case awakeSet:
				if len(set.bodySims) != len(set.bodyStates) {
					panic("dbox2d: the awake sims and states differ in length")
				}
			case disabledSet:
				if len(set.islandSims) != 0 || len(set.bodyStates) != 0 {
					panic("dbox2d: the disabled set holds islands or states")
				}
			default:
				if len(set.bodyStates) != 0 {
					panic("dbox2d: only the awake set holds body states")
				}
			}

			// Validate bodies
			totalBodyCount += len(set.bodySims)
			for i := range set.bodySims {
				sim := &set.bodySims[i]

				bodyId := sim.bodyId
				if bodyId < 0 || bodyId >= len(w.bodies) {
					panic("dbox2d: a sim points outside the body array")
				}
				b := &w.bodies[bodyId]
				if b.setIndex != setIndex || b.localIndex != i {
					panic("dbox2d: a body does not point back at its sim")
				}

				if setIndex == disabledSet && b.headContactKey != nullIndex {
					panic("dbox2d: a disabled body has contacts")
				}

				// Validate body shapes
				prevShapeId := nullIndex
				shapeId := b.headShapeId
				for shapeId != nullIndex {
					s := &w.shapes[shapeId]
					if s.id != shapeId {
						panic("dbox2d: the shape id does not match its slot")
					}
					if s.prevShapeId != prevShapeId {
						panic("dbox2d: the shape list is not doubly linked")
					}

					prevShapeId = shapeId
					shapeId = s.nextShapeId
				}

				// Validate body contacts
				contactKey := b.headContactKey
				for contactKey != nullIndex {
					contactId := contactKey >> 1
					edgeIndex := contactKey & 1

					c := &w.contacts[contactId]
					if c.setIndex == staticSet {
						panic("dbox2d: a contact is in the static set")
					}
					if c.edges[0].bodyId != bodyId && c.edges[1].bodyId != bodyId {
						panic("dbox2d: a contact on the body list does not touch the body")
					}
					contactKey = c.edges[edgeIndex].nextKey
				}
			}

			// Validate contacts
			totalContactCount += len(set.contactSims)
			for i := range set.contactSims {
				cs := &set.contactSims[i]
				c := &w.contacts[cs.contactId]
				if setIndex == awakeSet {
					// contact should be non-touching if awake
					// or it could be this contact hasn't been transferred yet
					if cs.manifold.PointCount != 0 && cs.simFlags&simStartedTouching == 0 {
						panic("dbox2d: a touching contact is outside the graph")
					}
				}
				if c.setIndex != setIndex || c.colorIndex != nullIndex || c.localIndex != i {
					panic("dbox2d: a contact does not point back at its sim")
				}
			}

			// Validate islands
			totalIslandCount += len(set.islandSims)
			for i := range set.islandSims {
				islandId := set.islandSims[i].islandId
				if islandId < 0 || islandId >= len(w.islands) {
					panic("dbox2d: an island sim points outside the island array")
				}
				isl := &w.islands[islandId]
				if isl.setIndex != setIndex || isl.localIndex != i {
					panic("dbox2d: an island does not point back at its sim")
				}
			}
		} else {
			if len(set.bodySims) != 0 || len(set.contactSims) != 0 || len(set.islandSims) != 0 || len(set.bodyStates) != 0 {
				panic("dbox2d: an unused set is not empty")
			}
		}
	}

	if activeSetCount != w.solverSetIdPool.idCount() {
		panic("dbox2d: the live set count and the set pool disagree")
	}

	if totalBodyCount != w.bodyIdPool.idCount() {
		panic("dbox2d: the body count and the body pool disagree")
	}

	if totalIslandCount != w.islandIdPool.idCount() {
		panic("dbox2d: the island count and the island pool disagree")
	}

	// Validate constraint graph
	for colorIndex := range graphColorCount {
		color := &w.constraintGraph.colors[colorIndex]
		totalContactCount += len(color.contactSims)
		for i := range color.contactSims {
			cs := &color.contactSims[i]
			c := &w.contacts[cs.contactId]
			// contact should be touching in the constraint graph or awaiting transfer to non-touching
			if cs.manifold.PointCount <= 0 && cs.simFlags&(simStoppedTouching|simDisjoint) == 0 {
				panic("dbox2d: a non-touching contact is in the graph")
			}
			if c.setIndex != awakeSet || c.colorIndex != colorIndex || c.localIndex != i {
				panic("dbox2d: a graph contact does not point back at its sim")
			}

			bodyIdA := c.edges[0].bodyId
			bodyIdB := c.edges[1].bodyId

			if colorIndex < overflowIndex {
				bodyA := &w.bodies[bodyIdA]
				bodyB := &w.bodies[bodyIdB]
				if color.bodySet.getBit(bodyIdA) != (bodyA.bodyType != StaticBody) {
					panic("dbox2d: the color bit of body A is wrong")
				}
				if color.bodySet.getBit(bodyIdB) != (bodyB.bodyType != StaticBody) {
					panic("dbox2d: the color bit of body B is wrong")
				}
			}
		}
	}

	if totalContactCount != w.contactIdPool.idCount() {
		panic("dbox2d: the contact count and the contact pool disagree")
	}
	if totalContactCount != w.broadPhase.pairSet.count {
		panic("dbox2d: the contact count and the pair set disagree")
	}
}

// Gravity returns the gravity vector of the world.
func (id WorldId) Gravity() Vec2 {
	w := getWorldFromId(id)
	return w.gravity
}

// taskContext is the scratch that the body finalize fills for the island
// sleep. It corresponds to b2TaskContext in src/world.h.
type taskContext struct {
	// contactStateBitSet marks the contacts whose touch state changed in
	// the collide pass, by contact id, because the sims move between the
	// touching and the non-touching arrays.
	contactStateBitSet bitSet

	// enlargedSimBitSet marks the awake body sims whose shapes grew their
	// fat bounds in the finalize, so the refit walks them in order.
	enlargedSimBitSet bitSet

	// awakeIslandBitSet marks the awake islands by local index.
	awakeIslandBitSet bitSet

	// splitIslandId is the sleepiest island with a pending split.
	splitIslandId  int
	splitSleepTime Q

	// Deferred: the enlarged body bit set of the reference serves the
	// broad-phase.
}

// GetBodyEvents returns the move events of the last step. It corresponds
// to b2World_GetBodyEvents in src/world.c.
func GetBodyEvents(worldId WorldId) BodyEvents {
	w := getWorldFromId(worldId)
	if w.locked {
		panic("dbox2d: the world is locked")
	}
	return BodyEvents{MoveEvents: w.bodyMoveEvents}
}

// GetContactEvents returns the contact events of the last step. It
// corresponds to b2World_GetContactEvents in src/world.c.
func GetContactEvents(worldId WorldId) ContactEvents {
	w := getWorldFromId(worldId)
	if w.locked {
		panic("dbox2d: the world is locked")
	}

	// Careful to use previous buffer
	endEventArrayIndex := 1 - w.endEventArrayIndex

	return ContactEvents{
		BeginEvents: w.contactBeginEvents,
		EndEvents:   w.contactEndEvents[endEventArrayIndex],
		HitEvents:   w.contactHitEvents,
	}
}

// validateConnectivity checks that every touching contact of a body sits
// in the root island of that body, and that a non-touching contact has no
// island. Only the tests call it. It corresponds to b2ValidateConnectivity
// in src/world.c.
func validateConnectivity(w *world) {
	for bodyIndex := range w.bodies {
		b := &w.bodies[bodyIndex]
		if b.id == nullIndex {
			if !w.bodyIdPool.isFreeId(bodyIndex) {
				panic("dbox2d: a free body slot holds a used id")
			}
			continue
		}
		if !w.bodyIdPool.isUsedId(bodyIndex) {
			panic("dbox2d: a live body holds a free id")
		}
		if bodyIndex != b.id {
			panic("dbox2d: the body id does not match its slot")
		}

		// Islands merge on the next step, so compare the roots.
		bodyIslandId := nullIndex
		if b.islandId != nullIndex {
			_, bodyIslandId = findRootIsland(w, b.islandId)
		}
		bodySetIndex := b.setIndex

		contactKey := b.headContactKey
		for contactKey != nullIndex {
			contactId := contactKey >> 1
			edgeIndex := contactKey & 1
			c := &w.contacts[contactId]

			touching := c.flags&contactTouchingFlag != 0
			if touching {
				if bodySetIndex != staticSet {
					_, contactIslandId := findRootIsland(w, c.islandId)
					if contactIslandId != bodyIslandId {
						panic("dbox2d: a touching contact and its body are in different islands")
					}
				}
			} else if c.islandId != nullIndex {
				panic("dbox2d: a non-touching contact has an island")
			}

			contactKey = c.edges[edgeIndex].nextKey
		}

		// Deferred: the joint edges of the reference.
	}
}

// validateContacts checks the set, color and sim of every live contact.
// Only the tests call it. It corresponds to b2ValidateContacts in
// src/world.c.
func validateContacts(w *world) {
	if len(w.contacts) != w.contactIdPool.idCapacity() {
		panic("dbox2d: the contact array and the id pool differ in capacity")
	}
	allocatedContactCount := 0

	for contactIndex := range w.contacts {
		c := &w.contacts[contactIndex]
		if c.contactId == nullIndex {
			continue
		}
		if c.contactId != contactIndex {
			panic("dbox2d: the contact id does not match its slot")
		}
		allocatedContactCount += 1

		touching := c.flags&contactTouchingFlag != 0
		setId := c.setIndex

		switch {
		case setId == awakeSet:
			if touching {
				if c.colorIndex < 0 || c.colorIndex >= graphColorCount {
					panic("dbox2d: an awake touching contact has no color")
				}
			} else if c.colorIndex != nullIndex {
				panic("dbox2d: an awake non-touching contact has a color")
			}
		case setId >= firstSleepingSet:
			// Only touching contacts sleep.
			if !touching {
				panic("dbox2d: a sleeping set holds a non-touching contact")
			}
		default:
			// Sleeping and non-touching contacts belong in the disabled set.
			if touching || setId != disabledSet {
				panic("dbox2d: a non-touching contact is outside the disabled set")
			}
		}

		cs := getContactSim(w, c)
		if cs.contactId != contactIndex {
			panic("dbox2d: the contact sim id does not match")
		}
		if cs.shapeIdA != c.shapeIdA || cs.shapeIdB != c.shapeIdB {
			panic("dbox2d: the contact sim shapes do not match")
		}

		simTouching := cs.simFlags&simTouchingFlag != 0
		if touching != simTouching {
			panic("dbox2d: the contact and its sim disagree on touching")
		}
		if cs.manifold.PointCount < 0 || cs.manifold.PointCount > 2 {
			panic("dbox2d: the manifold point count is out of range")
		}
	}

	if allocatedContactCount != w.contactIdPool.idCount() {
		panic("dbox2d: the live contact count and the id pool differ")
	}
}

// OverlapAABB reports every shape whose fat bounds overlap the box and
// whose filter accepts the query. A locked world panics. It corresponds
// to b2World_OverlapAABB in src/world.c.
func OverlapAABB(worldId WorldId, aabb AABB, filter QueryFilter, fcn OverlapResultFcn) TreeStats {
	w := getWorldFromId(worldId)
	if w.locked {
		panic("dbox2d: the world is locked")
	}
	if !IsValidAABB(aabb) {
		panic("dbox2d: OverlapAABB needs a valid box")
	}

	callback := func(_ int, userData uint64) bool {
		s := &w.shapes[int(userData)]
		if !shouldQueryCollide(s.filter, filter) {
			return true
		}
		return fcn(shapeIdOf(w, s))
	}

	var stats TreeStats
	for i := range w.broadPhase.trees {
		result := w.broadPhase.trees[i].query(aabb, filter.MaskBits, callback)
		stats.NodeVisits += result.nodeVisits
		stats.LeafVisits += result.leafVisits
	}
	return stats
}

// CastRay reports the shapes a ray hits, tree by tree. The callback
// clips the ray for the next tree; a zero return stops the cast. A
// locked world panics. It corresponds to b2World_CastRay in src/world.c.
func CastRay(worldId WorldId, origin, translation Vec2, filter QueryFilter, fcn CastResultFcn) TreeStats {
	w := getWorldFromId(worldId)
	if w.locked {
		panic("dbox2d: the world is locked")
	}
	if !IsValidVec2(origin) || !IsValidVec2(translation) {
		panic("dbox2d: CastRay needs a valid ray")
	}

	zero := fixed.Q32Zero()
	one := fixed.Q32One()
	input := RayCastInput{Origin: origin, Translation: translation, MaxFraction: one}
	fraction := one

	callback := func(input *RayCastInput, _ int, userData uint64) Q {
		s := &w.shapes[int(userData)]
		if !shouldQueryCollide(s.filter, filter) {
			return input.MaxFraction
		}

		transform := getBodyTransformQuick(w, &w.bodies[s.bodyId])
		output := rayCastShape(input, s, transform)
		if !output.Hit {
			return input.MaxFraction
		}

		value := fcn(shapeIdOf(w, s), output.Point, output.Normal, output.Fraction)

		// A negative value skips the shape and keeps the clip.
		if !value.Less(zero) && !one.Less(value) {
			fraction = value
		}
		return value
	}

	var stats TreeStats
	for i := range w.broadPhase.trees {
		result := w.broadPhase.trees[i].rayCast(&input, filter.MaskBits, callback)
		stats.NodeVisits += result.nodeVisits
		stats.LeafVisits += result.leafVisits

		if fraction.Eq(zero) {
			return stats
		}
		input.MaxFraction = fraction
	}
	return stats
}
