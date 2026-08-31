package dbox2d

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
	// Deferred: the arena, the broad-phase, the constraint graph, the
	// joint, contact, island, chain and sensor storage, the events, the
	// callbacks and the task system of the reference.

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

	worldId uint16

	enableSleep        bool
	locked             bool
	enableWarmStarting bool
	enableContinuous   bool
	enableSpeculative  bool
	inUse              bool
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

	// Destroy solver sets
	for i := range w.solverSets {
		if w.solverSets[i].setIndex != nullIndex {
			destroySolverSet(w, i)
		}
	}

	w.bodyIdPool.destroy()
	w.shapeIdPool.destroy()
	w.solverSetIdPool.destroy()

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

	activeSetCount := 0
	totalBodyCount := 0

	// Validate all solver sets
	for setIndex := range w.solverSets {
		set := &w.solverSets[setIndex]
		if set.setIndex != nullIndex {
			activeSetCount += 1

			if setIndex == awakeSet {
				if len(set.bodySims) != len(set.bodyStates) {
					panic("dbox2d: the awake sims and states differ in length")
				}
			} else if len(set.bodyStates) != 0 {
				panic("dbox2d: only the awake set holds body states")
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
			}
		} else {
			if len(set.bodySims) != 0 || len(set.bodyStates) != 0 {
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
}

// Gravity returns the gravity vector of the world.
func (id WorldId) Gravity() Vec2 {
	w := getWorldFromId(id)
	return w.gravity
}
