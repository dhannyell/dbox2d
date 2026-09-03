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
	// Deferred: the task system of the reference.
	sensors            []sensor
	sensorTaskContexts [1]sensorTaskContext

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

	chainIdPool idPool

	// chainShapes maps a chain id to the chain data.
	chainShapes []chainShape

	contactIdPool idPool

	// contacts maps a contact id to the cold contact data. The sims live in
	// the solver sets.
	contacts []contact

	jointIdPool idPool

	// joints maps a joint id to the cold joint data. The sims live in the
	// solver sets and in the graph colors.
	joints []joint

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
	sensorBeginEvents  []SensorBeginTouchEvent
	sensorEndEvents    [2][]SensorEndTouchEvent

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
	frictionCallback    FrictionCallback
	restitutionCallback RestitutionCallback

	worldId uint16

	enableSleep        bool
	locked             bool
	enableWarmStarting bool
	enableContinuous   bool
	enableSpeculative  bool
	inUse              bool
}

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
	w.chainIdPool = createIdPool()
	w.chainShapes = make([]chainShape, 0, 16)

	w.contactIdPool = createIdPool()
	w.contacts = make([]contact, 0, 16)
	w.jointIdPool = createIdPool()
	w.joints = make([]joint, 0, 16)
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
	w.sensorBeginEvents = make([]SensorBeginTouchEvent, 0, 16)
	w.sensorEndEvents[0] = make([]SensorEndTouchEvent, 0, 16)
	w.sensorEndEvents[1] = make([]SensorEndTouchEvent, 0, 16)
	w.endEventArrayIndex = 0

	w.frictionCallback = def.FrictionCallback
	if w.frictionCallback == nil {
		w.frictionCallback = defaultFrictionCallback
	}
	w.restitutionCallback = def.RestitutionCallback
	if w.restitutionCallback == nil {
		w.restitutionCallback = defaultRestitutionCallback
	}

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
	w.jointIdPool.destroy()
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
	if w.jointIdPool.idCapacity() != len(w.joints) {
		panic("dbox2d: the joint pool and the joint array disagree")
	}

	activeSetCount := 0
	totalBodyCount := 0
	totalContactCount := 0
	totalIslandCount := 0
	totalJointCount := 0

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
				if len(set.jointSims) != 0 {
					panic("dbox2d: the awake set holds joints outside the graph")
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

				// Validate body joints
				jointKey := b.headJointKey
				for jointKey != nullIndex {
					jointId := jointKey >> 1
					edgeIndex := jointKey & 1

					j := &w.joints[jointId]

					otherEdgeIndex := edgeIndex ^ 1

					otherBody := &w.bodies[j.edges[otherEdgeIndex].bodyId]

					switch {
					case setIndex == disabledSet || otherBody.setIndex == disabledSet:
						if j.setIndex != disabledSet {
							panic("dbox2d: a joint of a disabled body is not disabled")
						}
					case setIndex == staticSet && otherBody.setIndex == staticSet:
						if j.setIndex != staticSet {
							panic("dbox2d: a joint of two static bodies is not static")
						}
					case setIndex == awakeSet:
						if j.setIndex != awakeSet {
							panic("dbox2d: a joint of an awake body is not awake")
						}
					case setIndex >= firstSleepingSet:
						if j.setIndex != setIndex {
							panic("dbox2d: a joint of a sleeping body is in another set")
						}
					}

					js := getJointSim(w, j)
					if js.jointId != jointId {
						panic("dbox2d: a joint sim does not point back at its joint")
					}
					if js.bodyIdA != j.edges[0].bodyId || js.bodyIdB != j.edges[1].bodyId {
						panic("dbox2d: a joint sim and its joint differ in bodies")
					}

					jointKey = j.edges[edgeIndex].nextKey
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

			// Validate joints
			totalJointCount += len(set.jointSims)
			for i := range set.jointSims {
				js := &set.jointSims[i]
				j := &w.joints[js.jointId]
				if j.setIndex != setIndex || j.colorIndex != nullIndex || j.localIndex != i {
					panic("dbox2d: a joint does not point back at its sim")
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
			if len(set.bodySims) != 0 || len(set.contactSims) != 0 || len(set.islandSims) != 0 || len(set.bodyStates) != 0 || len(set.jointSims) != 0 {
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

		totalJointCount += len(color.jointSims)
		for i := range color.jointSims {
			js := &color.jointSims[i]
			j := &w.joints[js.jointId]
			if j.setIndex != awakeSet || j.colorIndex != colorIndex || j.localIndex != i {
				panic("dbox2d: a graph joint does not point back at its sim")
			}

			bodyIdA := j.edges[0].bodyId
			bodyIdB := j.edges[1].bodyId

			if colorIndex < overflowIndex {
				bodyA := &w.bodies[bodyIdA]
				bodyB := &w.bodies[bodyIdB]
				if color.bodySet.getBit(bodyIdA) != (bodyA.bodyType != StaticBody) {
					panic("dbox2d: the color bit of joint body A is wrong")
				}
				if color.bodySet.getBit(bodyIdB) != (bodyB.bodyType != StaticBody) {
					panic("dbox2d: the color bit of joint body B is wrong")
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

	if totalJointCount != w.jointIdPool.idCount() {
		panic("dbox2d: the joint count and the joint pool disagree")
	}
}

// GetGravity returns the gravity vector. It corresponds to b2World_GetGravity.
func (worldId WorldId) GetGravity() Vec2 {
	w := getWorldFromId(worldId)
	return w.gravity
}

// EnableSleeping changes sleeping and wakes sleeping sets when disabled.
// It corresponds to b2World_EnableSleeping in src/world.c.
func (worldId WorldId) EnableSleeping(flag bool) {
	w := getWorldFromId(worldId)
	if w.locked || flag == w.enableSleep {
		return
	}
	w.enableSleep = flag
	if !flag {
		for setIndex := firstSleepingSet; setIndex < len(w.solverSets); setIndex++ {
			if w.solverSets[setIndex].setIndex != nullIndex && len(w.solverSets[setIndex].bodySims) > 0 {
				wakeSolverSet(w, setIndex)
			}
		}
	}
}

// IsSleepingEnabled reports whether sleeping is enabled.
// It corresponds to b2World_IsSleepingEnabled in src/world.c.
func (worldId WorldId) IsSleepingEnabled() bool { return getWorldFromId(worldId).enableSleep }

// EnableContinuous changes continuous collision detection.
// It corresponds to b2World_EnableContinuous in src/world.c.
func (worldId WorldId) EnableContinuous(flag bool) {
	w := getWorldFromId(worldId)
	if w.locked {
		return
	}
	w.enableContinuous = flag
}

// IsContinuousEnabled reports whether continuous collision is enabled.
// It corresponds to b2World_IsContinuousEnabled in src/world.c.
func (worldId WorldId) IsContinuousEnabled() bool { return getWorldFromId(worldId).enableContinuous }

// SetRestitutionThreshold limits the speed threshold to the valid Q range.
// It corresponds to b2World_SetRestitutionThreshold in src/world.c.
func (worldId WorldId) SetRestitutionThreshold(value Q) {
	w := getWorldFromId(worldId)
	if w.locked {
		return
	}
	w.restitutionThreshold = value.Max(fixed.Q32Zero()).Min(fixed.Q32MaxValue())
}

// GetRestitutionThreshold returns the restitution speed threshold.
// It corresponds to b2World_GetRestitutionThreshold in src/world.c.
func (worldId WorldId) GetRestitutionThreshold() Q {
	return getWorldFromId(worldId).restitutionThreshold
}

// SetHitEventThreshold limits the hit-event threshold to the valid Q range.
// It corresponds to b2World_SetHitEventThreshold in src/world.c.
func (worldId WorldId) SetHitEventThreshold(value Q) {
	w := getWorldFromId(worldId)
	if w.locked {
		return
	}
	w.hitEventThreshold = value.Max(fixed.Q32Zero()).Min(fixed.Q32MaxValue())
}

// GetHitEventThreshold returns the hit-event threshold.
// It corresponds to b2World_GetHitEventThreshold in src/world.c.
func (worldId WorldId) GetHitEventThreshold() Q { return getWorldFromId(worldId).hitEventThreshold }

// SetGravity changes the world gravity vector.
// It corresponds to b2World_SetGravity in src/world.c.
func (worldId WorldId) SetGravity(gravity Vec2) { getWorldFromId(worldId).gravity = gravity }

// SetContactTuning changes contact softness and push speed.
// It corresponds to b2World_SetContactTuning in src/world.c.
func (worldId WorldId) SetContactTuning(hertz, dampingRatio, pushSpeed Q) {
	w := getWorldFromId(worldId)
	if w.locked {
		return
	}
	zero, maxValue := fixed.Q32Zero(), fixed.Q32MaxValue()
	w.contactHertz = hertz.Max(zero).Min(maxValue)
	w.contactDampingRatio = dampingRatio.Max(zero).Min(maxValue)
	w.maxContactPushSpeed = pushSpeed.Max(zero).Min(maxValue)
}

// SetMaximumLinearSpeed changes the velocity cap.
// It corresponds to b2World_SetMaximumLinearSpeed in src/world.c.
func (worldId WorldId) SetMaximumLinearSpeed(maximumLinearSpeed Q) {
	w := getWorldFromId(worldId)
	if w.locked {
		return
	}
	w.maxLinearSpeed = maximumLinearSpeed
}

// GetMaximumLinearSpeed returns the velocity cap.
// It corresponds to b2World_GetMaximumLinearSpeed in src/world.c.
func (worldId WorldId) GetMaximumLinearSpeed() Q { return getWorldFromId(worldId).maxLinearSpeed }

// EnableWarmStarting changes impulse warm starting.
// It corresponds to b2World_EnableWarmStarting in src/world.c.
func (worldId WorldId) EnableWarmStarting(flag bool) {
	w := getWorldFromId(worldId)
	if w.locked {
		return
	}
	w.enableWarmStarting = flag
}

// IsWarmStartingEnabled reports whether warm starting is enabled.
// It corresponds to b2World_IsWarmStartingEnabled in src/world.c.
func (worldId WorldId) IsWarmStartingEnabled() bool {
	return getWorldFromId(worldId).enableWarmStarting
}

// GetAwakeBodyCount returns the number of bodies in the awake set.
// It corresponds to b2World_GetAwakeBodyCount in src/world.c.
func (worldId WorldId) GetAwakeBodyCount() int {
	return len(getWorldFromId(worldId).solverSets[awakeSet].bodySims)
}

// GetCounters reports entity, tree, arena and graph counts.
// It corresponds to b2World_GetCounters in src/world.c.
func (worldId WorldId) GetCounters() Counters {
	w := getWorldFromId(worldId)
	result := Counters{
		BodyCount: w.bodyIdPool.idCount(), ShapeCount: w.shapeIdPool.idCount(), ContactCount: w.contactIdPool.idCount(),
		JointCount: w.jointIdPool.idCount(), IslandCount: w.islandIdPool.idCount(), StackUsed: getMaxArenaAllocation(&w.arena),
		StaticTreeHeight: w.broadPhase.trees[StaticBody].getHeight(),
	}
	result.TreeHeight = max(w.broadPhase.trees[DynamicBody].getHeight(), w.broadPhase.trees[KinematicBody].getHeight())
	for i := range graphColorCount {
		color := &w.constraintGraph.colors[i]
		result.ColorCounts[i] = len(color.contactSims) + len(color.jointSims)
	}
	return result
}

// SetUserData attaches application data to the world.
// It corresponds to b2World_SetUserData in src/world.c.
func (worldId WorldId) SetUserData(userData any) { getWorldFromId(worldId).userData = userData }

// GetUserData returns the data attached to the world.
// It corresponds to b2World_GetUserData in src/world.c.
func (worldId WorldId) GetUserData() any { return getWorldFromId(worldId).userData }

// SetFrictionCallback changes friction mixing for future contacts.
// It corresponds to b2World_SetFrictionCallback in src/world.c.
func (worldId WorldId) SetFrictionCallback(callback FrictionCallback) {
	w := getWorldFromId(worldId)
	if w.locked {
		return
	}
	if callback == nil {
		callback = defaultFrictionCallback
	}
	w.frictionCallback = callback
}

// SetRestitutionCallback changes restitution mixing for future contacts.
// It corresponds to b2World_SetRestitutionCallback in src/world.c.
func (worldId WorldId) SetRestitutionCallback(callback RestitutionCallback) {
	w := getWorldFromId(worldId)
	if w.locked {
		return
	}
	if callback == nil {
		callback = defaultRestitutionCallback
	}
	w.restitutionCallback = callback
}

// RebuildStaticTree rebuilds the static broad-phase tree.
// It corresponds to b2World_RebuildStaticTree in src/world.c.
func (worldId WorldId) RebuildStaticTree() {
	w := getWorldFromId(worldId)
	if w.locked {
		return
	}
	w.broadPhase.trees[StaticBody].rebuild(true)
}

// EnableSpeculative changes speculative collision handling.
// It corresponds to b2World_EnableSpeculative in src/world.c.
func (worldId WorldId) EnableSpeculative(flag bool) { getWorldFromId(worldId).enableSpeculative = flag }

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
func (worldId WorldId) GetBodyEvents() BodyEvents {
	w := getWorldFromId(worldId)
	if w.locked {
		panic("dbox2d: the world is locked")
	}
	return BodyEvents{MoveEvents: w.bodyMoveEvents}
}

// GetContactEvents returns the contact events of the last step. It
// corresponds to b2World_GetContactEvents in src/world.c.
func (worldId WorldId) GetContactEvents() ContactEvents {
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

// GetSensorEvents returns the sensor events of the last step. It
// corresponds to b2World_GetSensorEvents in src/world.c.
func (worldId WorldId) GetSensorEvents() SensorEvents {
	w := getWorldFromId(worldId)
	if w.locked {
		panic("dbox2d: the world is locked")
	}

	// Careful to use previous buffer
	endEventArrayIndex := 1 - w.endEventArrayIndex

	return SensorEvents{
		BeginEvents: w.sensorBeginEvents,
		EndEvents:   w.sensorEndEvents[endEventArrayIndex],
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

		jointKey := b.headJointKey
		for jointKey != nullIndex {
			jointId := jointKey >> 1
			edgeIndex := jointKey & 1

			j := &w.joints[jointId]

			otherEdgeIndex := edgeIndex ^ 1

			otherBody := &w.bodies[j.edges[otherEdgeIndex].bodyId]

			switch {
			case bodySetIndex == disabledSet || otherBody.setIndex == disabledSet:
				if j.islandId != nullIndex {
					panic("dbox2d: a joint of a disabled body has an island")
				}
			case bodySetIndex == staticSet:
				if otherBody.setIndex == staticSet && j.islandId != nullIndex {
					panic("dbox2d: a joint of two static bodies has an island")
				}
			default:
				_, jointIslandId := findRootIsland(w, j.islandId)
				if jointIslandId != bodyIslandId {
					panic("dbox2d: a joint and its body are in different islands")
				}
			}

			jointKey = j.edges[edgeIndex].nextKey
		}
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
// whose filter accepts the query. It walks the static, the kinematic and
// the dynamic tree in that order; a false result ends only the current
// tree, as in the reference. A locked world panics. It corresponds to
// b2World_OverlapAABB in src/world.c.
func (worldId WorldId) OverlapAABB(aabb AABB, filter QueryFilter, fcn OverlapResultFcn) TreeStats {
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
func (worldId WorldId) CastRay(origin, translation Vec2, filter QueryFilter, fcn CastResultFcn) TreeStats {
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

// OverlapShape reports every shape that overlaps the proxy and whose
// filter accepts the query, tree by tree; a false result ends only the
// current tree. The distance solver decides the overlap, so the report is
// exact, not a bounds test. A locked world panics. It corresponds to
// b2World_OverlapShape in src/world.c.
func (worldId WorldId) OverlapShape(proxy *ShapeProxy, filter QueryFilter, fcn OverlapResultFcn) TreeStats {
	w := getWorldFromId(worldId)
	if w.locked {
		panic("dbox2d: the world is locked")
	}

	aabb := MakeAABB(proxy.Points[:proxy.Count], proxy.Radius)
	tolerance := linearSlop.Div(fixed.Q32FromInt(10))

	callback := func(_ int, userData uint64) bool {
		s := &w.shapes[int(userData)]
		if !shouldQueryCollide(s.filter, filter) {
			return true
		}

		transform := getBodyTransformQuick(w, &w.bodies[s.bodyId])

		var input DistanceInput
		input.ProxyA = *proxy
		input.ProxyB = makeShapeDistanceProxy(s)
		input.TransformA = TransformIdentity()
		input.TransformB = transform
		input.UseRadii = true

		var cache SimplexCache
		output := ShapeDistance(&input, &cache, nil)

		if tolerance.Less(output.Distance) {
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

// CastRayClosest reports the closest hit of a ray, the most common cast
// in games. An initial overlap does not count as a hit. A locked world
// panics. It corresponds to b2World_CastRayClosest in src/world.c; the
// closure replaces b2RayCastClosestFcn.
func (worldId WorldId) CastRayClosest(origin, translation Vec2, filter QueryFilter) RayResult {
	var result RayResult

	w := getWorldFromId(worldId)
	if w.locked {
		panic("dbox2d: the world is locked")
	}
	if !IsValidVec2(origin) || !IsValidVec2(translation) {
		panic("dbox2d: CastRayClosest needs a valid ray")
	}

	zero := fixed.Q32Zero()
	one := fixed.Q32One()
	input := RayCastInput{Origin: origin, Translation: translation, MaxFraction: one}
	fraction := one

	closest := func(shapeId ShapeId, point, normal Vec2, hitFraction Q) Q {
		// Ignore initial overlap
		if hitFraction.Eq(zero) {
			return one.Neg()
		}

		result.ShapeId = shapeId
		result.Point = point
		result.Normal = normal
		result.Fraction = hitFraction
		result.Hit = true
		return hitFraction
	}

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

		value := closest(shapeIdOf(w, s), output.Point, output.Normal, output.Fraction)

		// The user may return -1 to skip this shape
		if !value.Less(zero) && !one.Less(value) {
			fraction = value
		}
		return value
	}

	for i := range w.broadPhase.trees {
		treeResult := w.broadPhase.trees[i].rayCast(&input, filter.MaskBits, callback)
		result.NodeVisits += treeResult.nodeVisits
		result.LeafVisits += treeResult.leafVisits

		if fraction.Eq(zero) {
			return result
		}
		input.MaxFraction = fraction
	}

	return result
}

// CastShape sweeps a proxy through the world and reports each hit, tree
// by tree. The callback returns as for CastRay. A locked world panics. It
// corresponds to b2World_CastShape in src/world.c.
func (worldId WorldId) CastShape(proxy *ShapeProxy, translation Vec2, filter QueryFilter, fcn CastResultFcn) TreeStats {
	w := getWorldFromId(worldId)
	if w.locked {
		panic("dbox2d: the world is locked")
	}
	if !IsValidVec2(translation) {
		panic("dbox2d: CastShape needs a valid translation")
	}

	zero := fixed.Q32Zero()
	one := fixed.Q32One()

	var input ShapeCastInput
	input.Proxy = *proxy
	input.Translation = translation
	input.MaxFraction = one

	fraction := one

	callback := func(input *ShapeCastInput, _ int, userData uint64) Q {
		s := &w.shapes[int(userData)]
		if !shouldQueryCollide(s.filter, filter) {
			return input.MaxFraction
		}

		transform := getBodyTransformQuick(w, &w.bodies[s.bodyId])

		output := shapeCastShape(input, s, transform)
		if !output.Hit {
			return input.MaxFraction
		}

		value := fcn(shapeIdOf(w, s), output.Point, output.Normal, output.Fraction)

		// The user may return -1 to skip this shape
		if !value.Less(zero) && !one.Less(value) {
			fraction = value
		}
		return value
	}

	var stats TreeStats
	for i := range w.broadPhase.trees {
		result := w.broadPhase.trees[i].shapeCast(&input, filter.MaskBits, callback)
		stats.NodeVisits += result.nodeVisits
		stats.LeafVisits += result.leafVisits

		if fraction.Eq(zero) {
			return stats
		}
		input.MaxFraction = fraction
	}
	return stats
}

// CastMover sweeps a capsule through the world and returns the fraction
// of the translation it may travel. Shapes it already overlaps do not
// stop it; a capsule that already touches may move a little closer. A
// locked world panics. It corresponds to b2World_CastMover in
// src/world.c.
func (worldId WorldId) CastMover(mover *Capsule, translation Vec2, filter QueryFilter) Q {
	if !IsValidVec2(translation) {
		panic("dbox2d: CastMover needs a valid translation")
	}
	if !linearSlop.Mul(fixed.Q32FromInt(2)).Less(mover.Radius) {
		panic("dbox2d: the mover radius must exceed two slops")
	}

	w := getWorldFromId(worldId)
	if w.locked {
		panic("dbox2d: the world is locked")
	}

	zero := fixed.Q32Zero()
	one := fixed.Q32One()

	var input ShapeCastInput
	input.Proxy.Points[0] = mover.Center1
	input.Proxy.Points[1] = mover.Center2
	input.Proxy.Count = 2
	input.Proxy.Radius = mover.Radius
	input.Translation = translation
	input.MaxFraction = one
	input.CanEncroach = true

	fraction := one

	callback := func(input *ShapeCastInput, _ int, userData uint64) Q {
		s := &w.shapes[int(userData)]
		if !shouldQueryCollide(s.filter, filter) {
			return fraction
		}

		transform := getBodyTransformQuick(w, &w.bodies[s.bodyId])

		output := shapeCastShape(input, s, transform)
		if output.Fraction.Eq(zero) {
			// Ignore overlapping shapes
			return fraction
		}

		fraction = output.Fraction
		return output.Fraction
	}

	for i := range w.broadPhase.trees {
		w.broadPhase.trees[i].shapeCast(&input, filter.MaskBits, callback)

		if fraction.Eq(zero) {
			return zero
		}
		input.MaxFraction = fraction
	}

	return fraction
}
