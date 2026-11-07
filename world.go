package b2

import (
	"fmt"
	"io"
	"math/bits"
	"strconv"
	"strings"
	"unsafe"
)

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
	sensors            []sensor
	sensorTaskContexts []sensorTaskContext
	workerCount        int
	executor           executor
	treeTask           treeTask
	solverContext      stepContext
	solverStages       []solverStage
	bodyBlocks         []solverBlock
	jointBlocks        []solverBlock
	contactBlocks      []solverBlock
	graphBlocks        []solverBlock
	contactPointers    []*contactSim
	jointPointers      []*jointSim
	// wide is the scratch of the SIMD family; empty in scalar builds.
	wide wideScratch

	// contactCount32 counts the contacts of the step outside the lane window.
	contactCount32 int

	// spareSet holds the sim storage of the last destroyed sleeping set, for
	// the next island that falls asleep. Only its slices are used. See D-010.
	spareSet solverSet

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

	// fatAABBs holds the fat bounds of each shape, indexed by shape id. They
	// live apart from the shapes for cache efficiency in the pair update.
	fatAABBs []AABB

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

	// profile times the last Step. The clock never feeds the simulation.
	profile Profile

	// taskCount counts the tasks of the last Step, inline or not.
	taskCount int

	// taskContexts holds per-worker scratch for the step stages.
	taskContexts []taskContext

	// The event arrays of the last step. The end events use two buffers,
	// because a contact destroyed between steps reports into the buffer
	// that the next step returns.
	bodyMoveEvents     []BodyMoveEvent
	contactBeginEvents []ContactBeginTouchEvent
	contactEndEvents   [2][]ContactEndTouchEvent
	contactHitEvents   []ContactHitEvent
	jointEvents        []JointEvent
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

	// contactSpeed caps the overlap resolution speed.
	contactSpeed Q

	// restitutionIterations is the number of restitution passes, and
	// enableRestitutionPropagation lets them solve every contact.
	restitutionIterations        int
	enableRestitutionPropagation bool

	// contactRecycleDistance is the farthest a body may move relative to
	// the other body of a contact for the contact to keep its manifold.
	contactRecycleDistance Q
	contactHertz           Q
	contactDampingRatio    Q

	// generation advances on each destruction of this slot, so a stale
	// WorldId fails validation.
	generation uint16

	userData any

	// inv_h is the inverse time step of the last step, for force reports.
	invH Q

	// invDt is the inverse full time step of the last step, for the drawn
	// contact forces.
	invDt Q

	// The debug sets mark what Draw has visited, by id. They live in the
	// world, as in the reference, so drawing does not allocate once they
	// have grown.
	debugBodySet    bitSet
	debugJointSet   bitSet
	debugContactSet bitSet
	debugIslandSet  bitSet

	// The mixing callbacks combine the material values of two shapes into
	// the effective contact values.
	frictionCallback    FrictionCallback
	restitutionCallback RestitutionCallback

	// customFilterFcn and preSolveFcn are nil unless the caller sets them,
	// matching the reference's NULL default (no context: D-014).
	customFilterFcn  CustomFilterFcn
	preSolveFcn      PreSolveFcn
	preContinuousFcn PreContinuousFcn

	worldId uint16

	enableSleep        bool
	locked             bool
	enableWarmStarting bool
	enableContinuous   bool
	inUse              bool

	// recording is non-nil while the world records; see recording.go.
	recording *Recording

	// maxCapacity tracks the high water marks that a snapshot carries.
	maxCapacity capacity
}

// defaultFrictionCallback is the geometric mean. It corresponds to
// b2DefaultFrictionCallback in src/world.c.
func defaultFrictionCallback(frictionA Q, _ uint64, frictionB Q, _ uint64) Q {
	return frictionA.Mul(frictionB).Sqrt()
}

// defaultRestitutionCallback is the maximum. It corresponds to
// b2DefaultRestitutionCallback in src/world.c.
func defaultRestitutionCallback(restitutionA Q, _ uint64, restitutionB Q, _ uint64) Q {
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
	w.workerCount = effectiveWorkerCount(def.WorkerCount)

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

	w.taskContexts = make([]taskContext, w.workerCount)
	for i := range w.taskContexts {
		w.taskContexts[i].contactStateBitSet = createBitSet(1024)
		w.taskContexts[i].awakeIslandBitSet = createBitSet(256)
		w.taskContexts[i].jointStateBitSet = createBitSet(1024)
		w.taskContexts[i].hitEventBitSet = createBitSet(1024)
		w.taskContexts[i].splitIslandId = nullIndex
	}
	w.sensorTaskContexts = make([]sensorTaskContext, w.workerCount)

	w.bodyMoveEvents = make([]BodyMoveEvent, 0, 16)
	w.contactBeginEvents = make([]ContactBeginTouchEvent, 0, 16)
	w.contactEndEvents[0] = make([]ContactEndTouchEvent, 0, 16)
	w.contactEndEvents[1] = make([]ContactEndTouchEvent, 0, 16)
	w.contactHitEvents = make([]ContactHitEvent, 0, 16)
	w.jointEvents = make([]JointEvent, 0, 4)
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
	w.contactSpeed = def.MaxContactPushSpeed.Max(QZero()).Min(QMaxValue())
	w.restitutionIterations = min(max(def.RestitutionIterations, 0), maxRestitutionIterations)
	w.enableRestitutionPropagation = def.EnableRestitutionPropagation
	w.contactRecycleDistance = contactRecycleDistance
	w.contactHertz = def.ContactHertz
	w.contactDampingRatio = def.ContactDampingRatio

	w.enableSleep = def.EnableSleep
	w.locked = false
	w.enableWarmStarting = true
	w.enableContinuous = def.EnableContinuous
	w.userData = def.UserData

	// add one to the world id so that 0 represents a null WorldId
	return WorldId{index1: uint16(worldId) + 1, generation: w.generation}
}

// DestroyWorld destroys a world and every body and shape in it.
func DestroyWorld(worldId WorldId) {
	w := getWorldFromId(worldId)

	// Detach any recording before teardown. The caller owns the recording buffer.
	stopRecordingInternal(w)

	w.treeTask.stop()
	w.executor.stop()

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

// validateSolverSets validates all solver sets. The reference only enables
// this check when B2_VALIDATE is set.
//
// Tests call it directly. Runtime mutation paths use validateSolverSetsDebug,
// which is gated by dbox2d_validate because the validation walks every body,
// joint, and contact and can be expensive when called repeatedly.
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
					case b.bodyType != DynamicBody && otherBody.bodyType != DynamicBody:
						if j.setIndex != staticSet {
							panic("dbox2d: a joint of two non-dynamic bodies is not static")
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
				if color.bodySet.getBit(bodyIdA) != (bodyA.bodyType == DynamicBody) {
					panic("dbox2d: the color bit of body A is wrong")
				}
				if color.bodySet.getBit(bodyIdB) != (bodyB.bodyType == DynamicBody) {
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
				if color.bodySet.getBit(bodyIdA) != (bodyA.bodyType == DynamicBody) {
					panic("dbox2d: the color bit of joint body A is wrong")
				}
				if color.bodySet.getBit(bodyIdB) != (bodyB.bodyType == DynamicBody) {
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
	if w.locked {
		return
	}
	if w.recording != nil {
		w.recording.rec(opWorldEnableSleeping, worldId, flag)
	}
	if flag == w.enableSleep {
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
		panic("dbox2d: the world is locked")
	}
	if w.recording != nil {
		w.recording.rec(opWorldEnableContinuous, worldId, flag)
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
		panic("dbox2d: the world is locked")
	}
	if w.recording != nil {
		w.recording.rec(opWorldSetRestitutionThreshold, worldId, value)
	}
	w.restitutionThreshold = value.Max(QZero()).Min(QMaxValue())
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
		panic("dbox2d: the world is locked")
	}
	if w.recording != nil {
		w.recording.rec(opWorldSetHitEventThreshold, worldId, value)
	}
	w.hitEventThreshold = value.Max(QZero()).Min(QMaxValue())
}

// GetHitEventThreshold returns the hit-event threshold.
// It corresponds to b2World_GetHitEventThreshold in src/world.c.
func (worldId WorldId) GetHitEventThreshold() Q { return getWorldFromId(worldId).hitEventThreshold }

// SetGravity changes the world gravity vector.
// It corresponds to b2World_SetGravity in src/world.c.
func (worldId WorldId) SetGravity(gravity Vec2) {
	w := getWorldFromId(worldId)
	if w.recording != nil {
		w.recording.rec(opWorldSetGravity, worldId, gravity)
	}
	w.gravity = gravity
}

// SetContactTuning changes contact softness and push speed.
// It corresponds to b2World_SetContactTuning in src/world.c.
func (worldId WorldId) SetContactTuning(hertz, dampingRatio, pushSpeed Q) {
	w := getWorldFromId(worldId)
	if w.locked {
		panic("dbox2d: the world is locked")
	}
	if w.recording != nil {
		w.recording.rec(opWorldSetContactTuning, worldId, hertz, dampingRatio, pushSpeed)
	}
	zero, maxValue := QZero(), QMaxValue()
	w.contactHertz = hertz.Max(zero).Min(maxValue)
	w.contactDampingRatio = dampingRatio.Max(zero).Min(maxValue)
	w.contactSpeed = pushSpeed.Max(zero).Min(maxValue)
}

// GetContactTuning returns the contact softness and push speed. It
// corresponds to b2World_GetContactTuning in src/physics_world.c.
func (worldId WorldId) GetContactTuning() (hertz, dampingRatio, pushSpeed Q) {
	w := getWorldFromId(worldId)
	return w.contactHertz, w.contactDampingRatio, w.contactSpeed
}

// SetRestitutionIterations changes the number of restitution passes,
// clamped to [0, 63]. It corresponds to b2World_SetRestitutionIterations in
// src/physics_world.c.
func (worldId WorldId) SetRestitutionIterations(iterations int) {
	w := getWorldFromId(worldId)
	if w.locked {
		panic("dbox2d: the world is locked")
	}
	if w.recording != nil {
		w.recording.rec(opWorldSetRestitutionIterations, worldId, iterations)
	}
	w.restitutionIterations = min(max(iterations, 0), maxRestitutionIterations)
}

// GetRestitutionIterations returns the number of restitution passes. It
// corresponds to b2World_GetRestitutionIterations in src/physics_world.c.
func (worldId WorldId) GetRestitutionIterations() int {
	return getWorldFromId(worldId).restitutionIterations
}

// EnableRestitutionPropagation lets the restitution passes solve every
// contact. It corresponds to b2World_EnableRestitutionPropagation in
// src/physics_world.c.
func (worldId WorldId) EnableRestitutionPropagation(flag bool) {
	w := getWorldFromId(worldId)
	if w.locked {
		panic("dbox2d: the world is locked")
	}
	if w.recording != nil {
		w.recording.rec(opWorldEnableRestitutionPropagation, worldId, flag)
	}
	w.enableRestitutionPropagation = flag
}

// IsRestitutionPropagationEnabled reports whether the restitution passes
// solve every contact. It corresponds to
// b2World_IsRestitutionPropagationEnabled in src/physics_world.c.
func (worldId WorldId) IsRestitutionPropagationEnabled() bool {
	return getWorldFromId(worldId).enableRestitutionPropagation
}

// SetContactRecycleDistance changes the farthest relative movement that
// keeps a contact manifold. Zero turns contact recycling off. It
// corresponds to b2World_SetContactRecycleDistance in src/physics_world.c.
func (worldId WorldId) SetContactRecycleDistance(recycleDistance Q) {
	w := getWorldFromId(worldId)
	if w.locked {
		panic("dbox2d: the world is locked")
	}
	if w.recording != nil {
		w.recording.rec(opWorldSetContactRecycleDistance, worldId, recycleDistance)
	}
	w.contactRecycleDistance = recycleDistance.Max(QZero()).Min(QMaxValue())
}

// GetContactRecycleDistance returns the contact recycle distance. It
// corresponds to b2World_GetContactRecycleDistance in src/physics_world.c.
func (worldId WorldId) GetContactRecycleDistance() Q {
	return getWorldFromId(worldId).contactRecycleDistance
}

// SetMaxLinearSpeed changes the velocity cap, which must be positive. It
// corresponds to b2World_SetMaxLinearSpeed in src/physics_world.c.
func (worldId WorldId) SetMaxLinearSpeed(maximumLinearSpeed Q) {
	if !IsValidQ(maximumLinearSpeed) || !QZero().Less(maximumLinearSpeed) {
		panic("dbox2d: SetMaxLinearSpeed needs a positive speed")
	}
	w := getWorldFromId(worldId)
	if w.locked {
		panic("dbox2d: the world is locked")
	}
	if w.recording != nil {
		w.recording.rec(opWorldSetMaxLinearSpeed, worldId, maximumLinearSpeed)
	}
	w.maxLinearSpeed = maximumLinearSpeed
}

// GetMaxLinearSpeed returns the velocity cap. It corresponds to
// b2World_GetMaxLinearSpeed in src/physics_world.c.
func (worldId WorldId) GetMaxLinearSpeed() Q { return getWorldFromId(worldId).maxLinearSpeed }

// GetBounds returns the union of the root bounds of the static, kinematic
// and dynamic trees, skipping empty trees; an empty world reports a zero
// box. It corresponds to b2World_GetBounds in src/physics_world.c.
func (worldId WorldId) GetBounds() AABB {
	w := getWorldFromId(worldId)
	if w.locked {
		panic("dbox2d: the world is locked")
	}
	bounds, _ := computeWorldBounds(w)
	return bounds
}

// computeWorldBounds unions the root bounds of the non-empty trees and
// reports whether any tree had a proxy. It corresponds to
// b2ComputeWorldBounds in src/physics_world.c.
func computeWorldBounds(w *world) (AABB, bool) {
	var bounds AABB
	haveBounds := false
	for i := range w.broadPhase.trees {
		tree := &w.broadPhase.trees[i]
		if tree.getProxyCount() == 0 {
			continue
		}
		treeBounds := tree.getRootBounds()
		if haveBounds {
			bounds = AABBUnion(bounds, treeBounds)
		} else {
			bounds = treeBounds
		}
		haveBounds = true
	}
	return bounds, haveBounds
}

// EnableWarmStarting changes impulse warm starting.
// It corresponds to b2World_EnableWarmStarting in src/world.c.
func (worldId WorldId) EnableWarmStarting(flag bool) {
	w := getWorldFromId(worldId)
	if w.locked {
		panic("dbox2d: the world is locked")
	}
	if w.recording != nil {
		w.recording.rec(opWorldEnableWarmStarting, worldId, flag)
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
		TaskCount:        w.taskCount,
	}
	result.TreeHeight = max(w.broadPhase.trees[DynamicBody].getHeight(), w.broadPhase.trees[KinematicBody].getHeight())
	for i := range w.workerCount {
		result.RecycledContactCount += w.taskContexts[i].recycledContactCount
	}
	for i := range graphColorCount {
		color := &w.constraintGraph.colors[i]
		result.ColorCounts[i] = len(color.contactSims) + len(color.jointSims)
		result.AwakeContactCount += len(color.contactSims)
	}
	result.AwakeContactCount += len(w.solverSets[awakeSet].contactSims)
	return result
}

// GetProfile returns the timing of the last Step.
// It corresponds to b2World_GetProfile in src/world.c.
func (id WorldId) GetProfile() Profile { return getWorldFromId(id).profile }

// DumpMemoryStats writes the memory use of the world to w, one section
// per structure. It corresponds to b2World_DumpMemoryStats in src/world.c.
func (id WorldId) DumpMemoryStats(w io.Writer) {
	wd := getWorldFromId(id)

	_, _ = fmt.Fprintf(w, "id pools\n")
	_, _ = fmt.Fprintf(w, "body ids: %d\n", wd.bodyIdPool.byteCount())
	_, _ = fmt.Fprintf(w, "solver set ids: %d\n", wd.solverSetIdPool.byteCount())
	_, _ = fmt.Fprintf(w, "joint ids: %d\n", wd.jointIdPool.byteCount())
	_, _ = fmt.Fprintf(w, "contact ids: %d\n", wd.contactIdPool.byteCount())
	_, _ = fmt.Fprintf(w, "island ids: %d\n", wd.islandIdPool.byteCount())
	_, _ = fmt.Fprintf(w, "shape ids: %d\n", wd.shapeIdPool.byteCount())
	_, _ = fmt.Fprintf(w, "chain ids: %d\n", wd.chainIdPool.byteCount())
	_, _ = fmt.Fprintf(w, "\n")

	_, _ = fmt.Fprintf(w, "world arrays\n")
	_, _ = fmt.Fprintf(w, "bodies: %d\n", sliceBytes(wd.bodies))
	_, _ = fmt.Fprintf(w, "solver sets: %d\n", sliceBytes(wd.solverSets))
	_, _ = fmt.Fprintf(w, "joints: %d\n", sliceBytes(wd.joints))
	_, _ = fmt.Fprintf(w, "contacts: %d\n", sliceBytes(wd.contacts))
	_, _ = fmt.Fprintf(w, "islands: %d\n", sliceBytes(wd.islands))
	_, _ = fmt.Fprintf(w, "shapes: %d\n", sliceBytes(wd.shapes))
	_, _ = fmt.Fprintf(w, "chains: %d\n", sliceBytes(wd.chainShapes))
	_, _ = fmt.Fprintf(w, "\n")

	_, _ = fmt.Fprintf(w, "broad-phase\n")
	_, _ = fmt.Fprintf(w, "static tree: %d\n", wd.broadPhase.trees[StaticBody].byteCount())
	_, _ = fmt.Fprintf(w, "kinematic tree: %d\n", wd.broadPhase.trees[KinematicBody].byteCount())
	_, _ = fmt.Fprintf(w, "dynamic tree: %d\n", wd.broadPhase.trees[DynamicBody].byteCount())
	pairSet := &wd.broadPhase.pairSet
	_, _ = fmt.Fprintf(w, "pairSet: %d (%d, %d)\n", pairSet.byteCount(), pairSet.count, len(pairSet.items))
	_, _ = fmt.Fprintf(w, "\n")

	var bodySimCapacity, bodyStateCapacity, jointSimCapacity, contactSimCapacity, islandSimCapacity int
	for i := range wd.solverSets {
		set := &wd.solverSets[i]
		if set.setIndex == nullIndex {
			continue
		}
		bodySimCapacity += cap(set.bodySims)
		bodyStateCapacity += cap(set.bodyStates)
		jointSimCapacity += cap(set.jointSims)
		contactSimCapacity += cap(set.contactSims)
		islandSimCapacity += cap(set.islandSims)
	}

	_, _ = fmt.Fprintf(w, "solver sets\n")
	_, _ = fmt.Fprintf(w, "body sim: %d\n", bodySimCapacity*int(unsafe.Sizeof(bodySim{})))
	_, _ = fmt.Fprintf(w, "body state: %d\n", bodyStateCapacity*int(unsafe.Sizeof(bodyState{})))
	_, _ = fmt.Fprintf(w, "joint sim: %d\n", jointSimCapacity*int(unsafe.Sizeof(jointSim{})))
	_, _ = fmt.Fprintf(w, "contact sim: %d\n", contactSimCapacity*int(unsafe.Sizeof(contactSim{})))
	// The reference multiplies by sizeof(int) here, a bug; the port uses the
	// actual island sim size. See DIVERGENCES.md D-015.
	_, _ = fmt.Fprintf(w, "island sim: %d\n", islandSimCapacity*int(unsafe.Sizeof(islandSim{})))
	_, _ = fmt.Fprintf(w, "\n")

	var bodyBitSetBytes, graphContactSimCapacity, graphJointSimCapacity int
	for i := range graphColorCount {
		color := &wd.constraintGraph.colors[i]
		bodyBitSetBytes += color.bodySet.byteCount()
		graphContactSimCapacity += cap(color.contactSims)
		graphJointSimCapacity += cap(color.jointSims)
	}

	_, _ = fmt.Fprintf(w, "constraint graph\n")
	_, _ = fmt.Fprintf(w, "body bit sets: %d\n", bodyBitSetBytes)
	_, _ = fmt.Fprintf(w, "joint sim: %d\n", graphJointSimCapacity*int(unsafe.Sizeof(jointSim{})))
	_, _ = fmt.Fprintf(w, "contact sim: %d\n", graphContactSimCapacity*int(unsafe.Sizeof(contactSim{})))
	_, _ = fmt.Fprintf(w, "\n")

	_, _ = fmt.Fprintf(w, "stack allocator: %d\n\n", getArenaCapacity(&wd.arena))
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
		panic("dbox2d: the world is locked")
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
		panic("dbox2d: the world is locked")
	}
	if callback == nil {
		callback = defaultRestitutionCallback
	}
	w.restitutionCallback = callback
}

// SetCustomFilterCallback sets the pair filter run during collision.
// It corresponds to b2World_SetCustomFilterCallback in src/world.c; the
// reference also stores a void* context, dropped here per D-014.
func (worldId WorldId) SetCustomFilterCallback(fcn CustomFilterFcn) {
	w := getWorldFromId(worldId)
	if fcn != nil && w.recording != nil {
		panic("dbox2d: a custom filter callback is not supported while recording")
	}
	w.customFilterFcn = fcn
}

// SetPreSolveCallback sets the callback run after a contact's manifold is
// updated, before the solver sees it, and the callback run on a time of
// impact event of the continuous stage. Either may be nil. It corresponds
// to b2World_SetPreSolveCallback in src/physics_world.c.
func (worldId WorldId) SetPreSolveCallback(preSolveFcn PreSolveFcn, preContinuousFcn PreContinuousFcn) {
	w := getWorldFromId(worldId)
	if (preSolveFcn != nil || preContinuousFcn != nil) && w.recording != nil {
		panic("dbox2d: a pre-solve callback is not supported while recording")
	}
	w.preSolveFcn = preSolveFcn
	w.preContinuousFcn = preContinuousFcn
}

// RebuildStaticTree rebuilds the static broad-phase tree.
// It corresponds to b2World_RebuildStaticTree in src/world.c.
func (worldId WorldId) RebuildStaticTree() {
	w := getWorldFromId(worldId)
	if w.locked {
		panic("dbox2d: the world is locked")
	}
	if w.recording != nil {
		w.recording.rec(opWorldRebuildStaticTree, worldId)
	}

	staticTree := &w.broadPhase.trees[StaticBody]

	// Keep the moved marks, the next pair update still needs them.
	var movedProxies []int
	if staticTree.getProxyCount() > 0 {
		movedProxies = staticTree.gatherMovedProxies(nil)
	}

	staticTree.rebuild(true)

	for _, proxyId := range movedProxies {
		staticTree.markProxyMovedSerial(proxyId)
	}
}

// taskContext is the scratch that the body finalize fills for the island
// sleep. It corresponds to b2TaskContext in src/world.h.
type taskContext struct {
	// contactStateBitSet marks the contacts whose touch state changed in
	// the collide pass, by contact id, because the sims move between the
	// touching and the non-touching arrays.
	contactStateBitSet bitSet

	// awakeIslandBitSet marks the awake islands by local index.
	awakeIslandBitSet bitSet

	// recycledContactCount counts the contacts this worker recycled in the
	// last step.
	recycledContactCount int

	// jointStateBitSet marks the joints whose reaction crossed a threshold,
	// by joint id.
	jointStateBitSet bitSet

	// hitEventBitSet marks the contacts with a hit event candidate, by
	// contact id; hasHitEvents is set with the first bit.
	hitEventBitSet bitSet
	hasHitEvents   bool

	// bulletBodies is the slice of the step's bullet buffer that covers
	// this worker's body range; the ranges never overlap.
	bulletBodies    []int
	bulletBodyCount int

	// pairKeys holds the new shape pair keys this worker found. It keeps
	// its capacity between steps.
	pairKeys []uint64

	// sensorHits holds the sensor crossings this worker's continuous sweeps
	// found. It keeps its capacity between steps.
	sensorHits []sensorHit

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

// GetJointEvents returns the joint events of the last step: the awake
// joints whose force or torque crossed their threshold. It corresponds to
// b2World_GetJointEvents in src/physics_world.c.
func (worldId WorldId) GetJointEvents() []JointEvent {
	w := getWorldFromId(worldId)
	if w.locked {
		panic("dbox2d: the world is locked")
	}
	return w.jointEvents
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

		bodyIslandId := b.islandId
		bodySetIndex := b.setIndex

		contactKey := b.headContactKey
		for contactKey != nullIndex {
			contactId := contactKey >> 1
			edgeIndex := contactKey & 1
			c := &w.contacts[contactId]

			touching := c.flags&contactTouchingFlag != 0
			if touching {
				if bodySetIndex != staticSet {
					if c.islandId != bodyIslandId {
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
			case b.bodyType != DynamicBody && otherBody.bodyType != DynamicBody:
				if j.islandId != nullIndex {
					panic("dbox2d: a joint of two non-dynamic bodies has an island")
				}
			default:
				if j.islandId != bodyIslandId {
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

// OverlapAABB reports every shape whose fat bounds overlap the box, given
// relative to origin, and whose filter accepts the query. It walks the static, the kinematic and
// the dynamic tree in that order; a false result ends only the current
// tree, as in the reference. A locked world panics. It corresponds to
// b2World_OverlapAABB in src/world.c.
func (worldId WorldId) OverlapAABB(origin Pos, aabb AABB, filter QueryFilter, fcn OverlapResultFcn) TreeStats {
	w := getWorldFromId(worldId)
	if w.recording == nil || w.locked {
		return worldId.overlapAABBCore(origin, aabb, filter, fcn)
	}
	qr := beginQuery(w.recording, worldId, wirePos{origin}, aabb, filter)
	stats := worldId.overlapAABBCore(origin, aabb, filter, qr.overlap(fcn))
	qr.commit(opQueryOverlapAABB, &stats)
	return stats
}

func (worldId WorldId) overlapAABBCore(origin Pos, aabb AABB, filter QueryFilter, fcn OverlapResultFcn) TreeStats {
	w := getWorldFromId(worldId)
	if w.locked {
		panic("dbox2d: the world is locked")
	}
	if !IsValidPos(origin) || !IsValidAABB(aabb) {
		panic("dbox2d: OverlapAABB needs a valid box")
	}
	worldBox := offsetAABB(aabb, origin)

	callback := func(_ int, userData uint64) bool {
		s := &w.shapes[int(userData)]
		if !shouldQueryCollide(s.filter, filter) {
			return true
		}
		return fcn(shapeIdOf(w, s))
	}

	var stats TreeStats
	for i := range w.broadPhase.trees {
		result := w.broadPhase.trees[i].overlapAABB(worldBox, filter.MaskBits, callback)
		stats.NodeVisits += result.nodeVisits
		stats.LeafVisits += result.leafVisits
	}
	return stats
}

// CastRay reports the shapes a ray hits, tree by tree. The callback
// clips the ray for the next tree; a zero return stops the cast. A
// locked world panics. It corresponds to b2World_CastRay in src/world.c.
func (worldId WorldId) CastRay(origin Pos, translation Vec2, filter QueryFilter, fcn CastResultFcn) TreeStats {
	w := getWorldFromId(worldId)
	if w.recording == nil || w.locked {
		return worldId.castRayCore(origin, translation, filter, fcn)
	}
	qr := beginQuery(w.recording, worldId, wirePos{origin}, translation, filter)
	stats := worldId.castRayCore(origin, translation, filter, qr.cast(fcn))
	qr.commit(opQueryCastRay, &stats)
	return stats
}

func (worldId WorldId) castRayCore(origin Pos, translation Vec2, filter QueryFilter, fcn CastResultFcn) TreeStats {
	w := getWorldFromId(worldId)
	if w.locked {
		panic("dbox2d: the world is locked")
	}
	if !IsValidPos(origin) || !IsValidVec2(translation) {
		panic("dbox2d: CastRay needs a valid ray")
	}

	zero := QZero()
	one := QOne()
	input := RayCastInput{Origin: ToVec2(origin), Translation: translation, MaxFraction: one}
	fraction := one

	callback := func(input *RayCastInput, _ int, userData uint64) Q {
		s := &w.shapes[int(userData)]
		if !shouldQueryCollide(s.filter, filter) {
			return input.MaxFraction
		}

		// Cast from the origin frame, as the reference does for large worlds.
		transform := ToRelativeTransform(getBodyTransformQuick(w, &w.bodies[s.bodyId]), origin)
		localInput := *input
		localInput.Origin = Vec2{}
		output := rayCastShape(&localInput, s, transform)
		if !output.Hit {
			return input.MaxFraction
		}

		value := fcn(shapeIdOf(w, s), OffsetPos(origin, output.Point), output.Normal, output.Fraction)

		// A negative value skips the shape and keeps the clip.
		if !value.Less(zero) && !one.Less(value) {
			fraction = value
		}
		return value
	}

	var stats TreeStats
	for i := range w.broadPhase.trees {
		result := w.broadPhase.trees[i].castRay(&input, filter.MaskBits, callback)
		stats.NodeVisits += result.nodeVisits
		stats.LeafVisits += result.leafVisits

		if fraction.Eq(zero) {
			return stats
		}
		input.MaxFraction = fraction
	}
	return stats
}

// OverlapShape reports every shape that overlaps the proxy, given relative
// to origin, and whose
// filter accepts the query, tree by tree; a false result ends only the
// current tree. The distance solver decides the overlap, so the report is
// exact, not a bounds test. A locked world panics. It corresponds to
// b2World_OverlapShape in src/world.c.
func (worldId WorldId) OverlapShape(origin Pos, proxy *ShapeProxy, filter QueryFilter, fcn OverlapResultFcn) TreeStats {
	w := getWorldFromId(worldId)
	if w.recording == nil || w.locked {
		return worldId.overlapShapeCore(origin, proxy, filter, fcn)
	}
	qr := beginQuery(w.recording, worldId, wirePos{origin}, proxy, filter)
	stats := worldId.overlapShapeCore(origin, proxy, filter, qr.overlap(fcn))
	qr.commit(opQueryOverlapShape, &stats)
	return stats
}

func (worldId WorldId) overlapShapeCore(origin Pos, proxy *ShapeProxy, filter QueryFilter, fcn OverlapResultFcn) TreeStats {
	w := getWorldFromId(worldId)
	if w.locked {
		panic("dbox2d: the world is locked")
	}
	if !IsValidPos(origin) {
		panic("dbox2d: OverlapShape needs a valid origin")
	}

	aabb := offsetAABB(MakeAABB(proxy.Points[:proxy.Count], proxy.Radius), origin)
	tolerance := linearSlopTenth

	callback := func(_ int, userData uint64) bool {
		s := &w.shapes[int(userData)]
		if !shouldQueryCollide(s.filter, filter) {
			return true
		}

		// Re-center on the query origin.
		transform := ToRelativeTransform(getBodyTransformQuick(w, &w.bodies[s.bodyId]), origin)

		var input DistanceInput
		input.ProxyA = *proxy
		input.ProxyB = makeShapeDistanceProxy(s)
		input.Transform = transform
		input.UseRadii = true

		var cache SimplexCache
		output := ComputeShapeDistance(&input, &cache, nil)

		if tolerance.Less(output.Distance) {
			return true
		}

		return fcn(shapeIdOf(w, s))
	}

	var stats TreeStats
	for i := range w.broadPhase.trees {
		result := w.broadPhase.trees[i].overlapAABB(aabb, filter.MaskBits, callback)
		stats.NodeVisits += result.nodeVisits
		stats.LeafVisits += result.leafVisits
	}
	return stats
}

// CastRayClosest reports the closest hit of a ray, the most common cast
// in games. An initial overlap does not count as a hit. A locked world
// panics. It corresponds to b2World_CastRayClosest in src/world.c; the
// closure replaces b2RayCastClosestFcn.
func (worldId WorldId) CastRayClosest(origin Pos, translation Vec2, filter QueryFilter) RayResult {
	result := worldId.castRayClosestCore(origin, translation, filter)
	if w := getWorldFromId(worldId); w.recording != nil {
		recordOneShot(w.recording, opQueryCastRayClosest, worldId, wirePos{origin}, translation, filter,
			result.ShapeId, wirePos{result.Point}, result.Normal, result.Fraction, result.NodeVisits, result.LeafVisits, result.Hit)
	}
	return result
}

func (worldId WorldId) castRayClosestCore(origin Pos, translation Vec2, filter QueryFilter) RayResult {
	var result RayResult

	w := getWorldFromId(worldId)
	if w.locked {
		panic("dbox2d: the world is locked")
	}
	if !IsValidPos(origin) || !IsValidVec2(translation) {
		panic("dbox2d: CastRayClosest needs a valid ray")
	}

	zero := QZero()
	one := QOne()
	input := RayCastInput{Origin: ToVec2(origin), Translation: translation, MaxFraction: one}
	fraction := one

	closest := func(shapeId ShapeId, point Pos, normal Vec2, hitFraction Q) Q {
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

		transform := ToRelativeTransform(getBodyTransformQuick(w, &w.bodies[s.bodyId]), origin)
		localInput := *input
		localInput.Origin = Vec2{}
		output := rayCastShape(&localInput, s, transform)
		if !output.Hit {
			return input.MaxFraction
		}

		value := closest(shapeIdOf(w, s), OffsetPos(origin, output.Point), output.Normal, output.Fraction)

		// The user may return -1 to skip this shape
		if !value.Less(zero) && !one.Less(value) {
			fraction = value
		}
		return value
	}

	for i := range w.broadPhase.trees {
		treeResult := w.broadPhase.trees[i].castRay(&input, filter.MaskBits, callback)
		result.NodeVisits += treeResult.nodeVisits
		result.LeafVisits += treeResult.leafVisits

		if fraction.Eq(zero) {
			return result
		}
		input.MaxFraction = fraction
	}

	return result
}

// CastShape sweeps a proxy, given relative to origin, through the world and
// reports each hit, tree
// by tree. The callback returns as for CastRay. A locked world panics. It
// corresponds to b2World_CastShape in src/world.c.
func (worldId WorldId) CastShape(origin Pos, proxy *ShapeProxy, translation Vec2, filter QueryFilter, fcn CastResultFcn) TreeStats {
	w := getWorldFromId(worldId)
	if w.recording == nil || w.locked {
		return worldId.castShapeCore(origin, proxy, translation, filter, fcn)
	}
	qr := beginQuery(w.recording, worldId, wirePos{origin}, proxy, translation, filter)
	stats := worldId.castShapeCore(origin, proxy, translation, filter, qr.cast(fcn))
	qr.commit(opQueryCastShape, &stats)
	return stats
}

func (worldId WorldId) castShapeCore(origin Pos, proxy *ShapeProxy, translation Vec2, filter QueryFilter, fcn CastResultFcn) TreeStats {
	w := getWorldFromId(worldId)
	if w.locked {
		panic("dbox2d: the world is locked")
	}
	if !IsValidPos(origin) || !IsValidVec2(translation) {
		panic("dbox2d: CastShape needs a valid translation")
	}

	zero := QZero()
	one := QOne()

	var input ShapeCastInput
	input.Proxy = *proxy
	input.Translation = translation
	input.MaxFraction = one

	fraction := one

	callback := func(treeInput *BoxCastInput, _ int, userData uint64) Q {
		s := &w.shapes[int(userData)]
		if !shouldQueryCollide(s.filter, filter) {
			return treeInput.MaxFraction
		}

		// Rebuild the shape input, taking only the advancing fraction from
		// the tree.
		localInput := input
		localInput.MaxFraction = treeInput.MaxFraction

		transform := ToRelativeTransform(getBodyTransformQuick(w, &w.bodies[s.bodyId]), origin)

		output := shapeCastShape(&localInput, s, transform)
		if !output.Hit {
			return treeInput.MaxFraction
		}

		value := fcn(shapeIdOf(w, s), OffsetPos(origin, output.Point), output.Normal, output.Fraction)

		// The user may return -1 to skip this shape
		if !value.Less(zero) && !one.Less(value) {
			fraction = value
		}
		return value
	}

	// Create a conservative world box from the proxy and origin.
	treeInput := BoxCastInput{
		Box:         offsetAABB(MakeAABB(proxy.Points[:proxy.Count], proxy.Radius), origin),
		Translation: translation,
		MaxFraction: one,
	}

	var stats TreeStats
	for i := range w.broadPhase.trees {
		result := w.broadPhase.trees[i].castBox(&treeInput, filter.MaskBits, callback)
		stats.NodeVisits += result.nodeVisits
		stats.LeafVisits += result.leafVisits

		if fraction.Eq(zero) {
			return stats
		}
		treeInput.MaxFraction = fraction
	}
	return stats
}

// CastMover sweeps a capsule, given relative to origin, through the world and returns the fraction
// of the translation it may travel. Shapes it already overlaps do not
// stop it; a capsule that already touches may move a little closer. A
// locked world panics. It corresponds to b2World_CastMover in
// src/world.c.
func (worldId WorldId) CastMover(origin Pos, mover *Capsule, translation Vec2, filter QueryFilter) Q {
	fraction := worldId.castMoverCore(origin, mover, translation, filter)
	if w := getWorldFromId(worldId); w.recording != nil {
		recordOneShot(w.recording, opQueryCastMover, worldId, wirePos{origin}, *mover, translation, filter, fraction)
	}
	return fraction
}

func (worldId WorldId) castMoverCore(origin Pos, mover *Capsule, translation Vec2, filter QueryFilter) Q {
	if !IsValidPos(origin) || !IsValidVec2(translation) {
		panic("dbox2d: CastMover needs a valid translation")
	}
	if !linearSlop.Mul(QFromInt(2)).Less(mover.Radius) {
		panic("dbox2d: the mover radius must exceed two slops")
	}

	w := getWorldFromId(worldId)
	if w.locked {
		panic("dbox2d: the world is locked")
	}

	zero := QZero()
	one := QOne()

	var input ShapeCastInput
	input.Proxy.Points[0] = mover.Center1
	input.Proxy.Points[1] = mover.Center2
	input.Proxy.Count = 2
	input.Proxy.Radius = mover.Radius
	input.Translation = translation
	input.MaxFraction = one
	input.CanEncroach = true

	fraction := one

	callback := func(treeInput *BoxCastInput, _ int, userData uint64) Q {
		s := &w.shapes[int(userData)]
		if !shouldQueryCollide(s.filter, filter) {
			return fraction
		}

		// Rebuild the shape input, taking only the advancing fraction from
		// the tree.
		localInput := input
		localInput.MaxFraction = treeInput.MaxFraction

		transform := ToRelativeTransform(getBodyTransformQuick(w, &w.bodies[s.bodyId]), origin)

		output := shapeCastShape(&localInput, s, transform)
		if output.Fraction.Eq(zero) {
			// Ignore overlapping shapes
			return fraction
		}

		fraction = output.Fraction
		return output.Fraction
	}

	// Bound the capsule.
	centers := [2]Vec2{mover.Center1, mover.Center2}
	treeInput := BoxCastInput{
		Box:         offsetAABB(MakeAABB(centers[:], mover.Radius), origin),
		Translation: translation,
		MaxFraction: one,
	}

	for i := range w.broadPhase.trees {
		w.broadPhase.trees[i].castBox(&treeInput, filter.MaskBits, callback)

		if fraction.Eq(zero) {
			return zero
		}
		treeInput.MaxFraction = fraction
	}

	return fraction
}

// CollideMover reports the collision plane of every shape the mover
// capsule, given relative to origin, touches, tree by tree; the planes are
// relative to origin too. The callback returns false to stop the
// walk. A locked world panics. It corresponds to b2World_CollideMover in
// src/world.c.
func (worldId WorldId) CollideMover(origin Pos, mover *Capsule, filter QueryFilter, fcn PlaneResultFcn) {
	w := getWorldFromId(worldId)
	if w.recording == nil || w.locked {
		worldId.collideMoverCore(origin, mover, filter, fcn)
		return
	}
	qr := beginQuery(w.recording, worldId, wirePos{origin}, *mover, filter)
	worldId.collideMoverCore(origin, mover, filter, qr.plane(fcn))
	qr.commit(opQueryCollideMover, nil)
}

func (worldId WorldId) collideMoverCore(origin Pos, mover *Capsule, filter QueryFilter, fcn PlaneResultFcn) {
	w := getWorldFromId(worldId)
	if w.locked {
		panic("dbox2d: the world is locked")
	}
	if !IsValidPos(origin) {
		panic("dbox2d: CollideMover needs a valid origin")
	}

	lower := Min(mover.Center1, mover.Center2)
	upper := Max(mover.Center1, mover.Center2)
	aabb := offsetAABB(AABB{
		LowerBound: Vec2{X: lower.X.Sub(mover.Radius), Y: lower.Y.Sub(mover.Radius)},
		UpperBound: Vec2{X: upper.X.Add(mover.Radius), Y: upper.Y.Add(mover.Radius)},
	}, origin)

	callback := func(_ int, userData uint64) bool {
		s := &w.shapes[int(userData)]
		if !shouldQueryCollide(s.filter, filter) {
			return true
		}

		transform := ToRelativeTransform(getBodyTransformQuick(w, &w.bodies[s.bodyId]), origin)
		result := collideMover(mover, s, transform)

		if result.Hit && IsNormalized(result.Plane.Normal) {
			return fcn(shapeIdOf(w, s), &result)
		}

		return true
	}

	for i := range w.broadPhase.trees {
		w.broadPhase.trees[i].overlapAABB(aabb, filter.MaskBits, callback)
	}
}

// explosionContext carries the explosion parameters to explosionCallback. It
// corresponds to the reference's ExplosionContext in src/world.c.
type explosionContext struct {
	w                *world
	position         Pos
	radius           Q
	falloff          Q
	impulsePerLength Q
}

// explosionCallback applies the impulse of one explosion to one shape's
// body. It corresponds to ExplosionCallback in src/world.c.
func (ctx *explosionContext) explosionCallback(_ int, userData uint64) bool {
	w := ctx.w
	s := &w.shapes[int(userData)]
	b := &w.bodies[s.bodyId]

	transform := getBodyTransformQuick(w, b)

	zero := QZero()
	one := QOne()

	// Re-center the explosion into the shape local frame so distance and
	// direction stay precise far from the origin.
	localPosition := InvTransformWorldPoint(transform, ctx.position)

	input := DistanceInput{
		ProxyA:    makeShapeDistanceProxy(s),
		ProxyB:    MakeProxy([]Vec2{localPosition}, zero),
		Transform: TransformIdentity(),
		UseRadii:  true,
	}
	cache := SimplexCache{}
	output := ComputeShapeDistance(&input, &cache, nil)

	if output.Distance.Greater(ctx.radius.Add(ctx.falloff)) {
		return true
	}

	wakeBody(w, b)
	if b.setIndex != awakeSet {
		return true
	}

	// The distance runs in the frame of the shape.
	closestPoint := output.PointA
	// D-012: the reference's exact zero-distance check.
	if output.Distance.Eq(zero) {
		closestPoint = getShapeCentroid(s)
	}

	direction := closestPoint.Sub(localPosition)
	// D-012: the guard reads the scalar mode; see DIVERGENCES.md.
	if QFromInt(100).Mul(scalarEpsilonSq).Less(direction.LenSq()) {
		direction = direction.Normalize()
	} else {
		direction = Vec2{X: one}
	}

	localLine := LeftPerp(direction)
	perimeter := getShapeProjectedPerimeter(s, localLine)

	scale := one
	if output.Distance.Greater(ctx.radius) && ctx.falloff.Greater(zero) {
		// D-006: the reference's division stays a division.
		scale = ctx.radius.Add(ctx.falloff).Sub(output.Distance).Div(ctx.falloff).Clamp(zero, one)
	}

	magnitude := ctx.impulsePerLength.Mul(perimeter).Mul(scale)
	impulse := RotateVector(transform.Q, direction).Mul(magnitude)

	state := getBodyState(w, b)
	sim := getBodySim(w, b)
	state.linearVelocity = MulAdd(state.linearVelocity, sim.invMass, impulse)
	// Lever arm from the center of mass to the closest point, rotated to world.
	r := RotateVector(transform.Q, closestPoint.Sub(sim.localCenter))
	state.angularVelocity = state.angularVelocity.Add(sim.invInertia.Mul(Cross(r, impulse)))

	return true
}

// Explode applies an impulse to every dynamic body near an explosion
// center, scaled by distance and by the shape perimeter facing the
// center. A locked world is a no-op. It corresponds to b2World_Explode in
// src/world.c.
func (worldId WorldId) Explode(def *ExplosionDef) {
	w := getWorldFromId(worldId)
	if w.locked {
		panic("dbox2d: the world is locked")
	}
	if w.recording != nil {
		w.recording.rec(opWorldExplode, worldId, *def)
	}

	ctx := explosionContext{
		w:                w,
		position:         def.Position,
		radius:           def.Radius,
		falloff:          def.Falloff,
		impulsePerLength: def.ImpulsePerLength,
	}

	reach := def.Radius.Add(def.Falloff)
	// The broad-phase tree is float, so translate a local query box out to
	// world with outward rounding.
	localBox := AABB{
		LowerBound: Vec2{X: reach.Neg(), Y: reach.Neg()},
		UpperBound: Vec2{X: reach, Y: reach},
	}
	aabb := offsetAABB(localBox, def.Position)

	w.broadPhase.trees[DynamicBody].overlapAABB(aabb, def.MaskBits, ctx.explosionCallback)
}

// drawShape forwards one shape's geometry to the drawer. It corresponds to
// b2DrawShape in src/physics_world.c.
func drawShape(draw *DebugDraw, shape *shape, transform WorldTransform, color HexColor, drawChainNormals bool) {
	switch shape.shapeType {
	case CapsuleShape:
		capsule := shape.capsule()
		p1 := TransformWorldPoint(transform, capsule.Center1)
		p2 := TransformWorldPoint(transform, capsule.Center2)
		draw.DrawSolidCapsuleFcn(p1, p2, capsule.Radius, color)
	case CircleShape:
		circle := shape.circle()
		draw.DrawSolidCircleFcn(transform, circle.Center, circle.Radius, color)
	case PolygonShape:
		poly := shape.polygon()
		draw.DrawSolidPolygonFcn(transform, poly.Vertices[:poly.Count], poly.Radius, color)
	case SegmentShape:
		segment := shape.segment()
		p1 := TransformWorldPoint(transform, segment.Point1)
		p2 := TransformWorldPoint(transform, segment.Point2)
		draw.DrawLineFcn(p1, p2, color)
	case ChainSegmentShape:
		segment := shape.chainSegment().Segment
		p1 := TransformWorldPoint(transform, segment.Point1)
		p2 := TransformWorldPoint(transform, segment.Point2)
		draw.DrawLineFcn(p1, p2, color)
		draw.DrawPointFcn(p2, QFromInt(4), color)

		if drawChainNormals {
			c := LerpPos(p1, p2, QHalf())
			e := SubPos(p2, p1).Normalize()
			n := RightPerp(e)
			length := QMustParse("0.2")
			draw.DrawLineFcn(c, OffsetPos(c, n.Mul(length)), ColorPaleGreen)
		}
	}
}

// drawShapeColor picks the debug color of a shape from its body state. It
// is the color chain of DrawQueryCallback in src/physics_world.c.
func drawShapeColor(body *body, sim *bodySim, shape *shape) HexColor {
	switch {
	case shape.customColor != 0:
		return HexColor(shape.customColor)
	case body.bodyType == DynamicBody && body.mass == (Q{}):
		// Bad body
		return ColorRed
	case body.setIndex == disabledSet:
		return ColorSlateGray
	case shape.sensorIndex != nullIndex:
		return ColorWheat
	case sim.flags&bodyHadTimeOfImpact != 0:
		return ColorLime
	case sim.flags&bodyIsBullet != 0 && body.setIndex == awakeSet:
		return ColorTurquoise
	case body.flags&bodyIsSpeedCapped != 0:
		return ColorYellow
	case sim.flags&bodyIsFast != 0:
		return ColorSalmon
	case body.bodyType == StaticBody:
		return ColorPaleGreen
	case body.bodyType == KinematicBody:
		return ColorRoyalBlue
	case body.setIndex == awakeSet:
		return ColorPink
	}
	return ColorGray
}

func bytesIndex(name []byte) int {
	for i, b := range name {
		if b == 0 {
			return i
		}
	}
	return len(name)
}

var graphColors = [graphColorCount]HexColor{
	ColorRed, ColorOrange, ColorYellow, ColorLimeGreen, ColorSpringGreen,
	ColorAqua, ColorDodgerBlue, ColorBlueViolet, ColorMagenta, ColorDeepPink,
	ColorCrimson, ColorCoral, ColorGold, ColorGreenYellow, ColorMediumSeaGreen,
	ColorTurquoise, ColorDeepSkyBlue, ColorCornflowerBlue, ColorMediumSlateBlue, ColorMediumOrchid,
	ColorHotPink, ColorTomato, ColorKhaki, ColorSilver,
}

// drawNumber formats a label as printf's "%.Nf" does, without converting Q
// to float.
func drawNumber(value Q, places int) string {
	text := value.String()
	if strings.ContainsAny(text, "eE") {
		// Float mode prints small and large values with an exponent; the
		// rounding below needs plain decimal digits.
		f, _ := strconv.ParseFloat(text, 64)
		text = strconv.FormatFloat(f, 'f', -1, 32)
	}
	negative := strings.HasPrefix(text, "-")
	text = strings.TrimPrefix(text, "-")
	whole, fraction, _ := strings.Cut(text, ".")
	fraction += strings.Repeat("0", places+1)
	digits, _ := strconv.ParseUint(whole+fraction[:places], 10, 64)
	// Round ties to even, as the reference's default printf rounding does.
	if fraction[places] > '5' || fraction[places] == '5' &&
		(digits%2 != 0 || strings.Trim(fraction[places+1:], "0") != "") {
		digits++
	}
	text = strconv.FormatUint(digits, 10)
	if len(text) <= places {
		text = strings.Repeat("0", places+1-len(text)) + text
	}
	if places > 0 {
		text = text[:len(text)-places] + "." + text[len(text)-places:]
	}
	if negative {
		text = "-" + text
	}
	return text
}

// drawGeneral formats a label as printf's "%g" does.
func drawGeneral(value Q) string {
	return strconv.FormatFloat(QToFloat64(value), 'g', 6, 64)
}

// drawContext carries the world and the drawer into the tree query. The
// visited sets live in the world, as in the reference, so drawing does not
// allocate once they have grown.
type drawContext struct {
	world *world
	draw  *DebugDraw
}

// drawQueryCallback draws one shape inside the drawing bounds and marks its
// body. It corresponds to DrawQueryCallback in src/physics_world.c.
func (ctx *drawContext) drawQueryCallback(_ int, userData uint64) bool {
	w, draw := ctx.world, ctx.draw
	shapeId := int(userData)
	shape := &w.shapes[shapeId]
	body := &w.bodies[shape.bodyId]
	sim := getBodySim(w, body)

	w.debugBodySet.setBit(shape.bodyId)

	if draw.DrawShapes {
		drawShape(draw, shape, sim.transform, drawShapeColor(body, sim, shape), draw.DrawChainNormals)
	}

	if draw.DrawBounds {
		draw.DrawBoundsFcn(w.fatAABBs[shapeId], ColorGold)
	}

	return true
}

// drawContactPoints draws the points of one contact. It is the contact
// block of b2World_Draw in src/physics_world.c.
func drawContactPoints(draw *DebugDraw, w *world, contact *contact) {
	cs := getContactSim(w, contact)
	simA := getBodySim(w, &w.bodies[contact.edges[0].bodyId])
	simB := getBodySim(w, &w.bodies[contact.edges[1].bodyId])
	drawManifoldPoints(draw, &cs.manifold, simA.center, simB.center, contact.colorIndex, w.invDt, w.invH)
}

// drawManifoldPoints draws the points of a manifold whose anchors are
// relative to centerA and centerB, the centers of mass of the two bodies.
func drawManifoldPoints(draw *DebugDraw, manifold *Manifold, centerA, centerB Pos, colorIndex int, invDt, invH Q) {
	axisScale := QMustParse("0.3")
	half := QHalf()
	normal := manifold.Normal

	for j := range manifold.PointCount {
		mp := &manifold.Points[j]

		var p Pos
		if draw.DrawAnchorA {
			p = OffsetPos(centerA, mp.AnchorA)
		} else {
			p = OffsetPos(centerB, mp.AnchorB)
		}

		switch {
		case draw.DrawGraphColors && colorIndex != nullIndex:
			// graph color
			pointSize := QFromInt(5)
			if colorIndex == overflowIndex {
				pointSize = QMustParse("7.5")
			}
			draw.DrawPointFcn(p, pointSize, graphColors[colorIndex])
		case linearSlop.Less(mp.Separation):
			// Speculative
			draw.DrawPointFcn(p, QFromInt(5), ColorGainsboro)
		case !mp.Persisted:
			// Add
			draw.DrawPointFcn(p, QFromInt(10), ColorGreen)
		default:
			// Persist
			draw.DrawPointFcn(p, QFromInt(5), ColorBlue)
		}

		if draw.DrawContactNormals {
			p2 := OffsetPos(p, normal.Mul(axisScale))
			draw.DrawLineFcn(p, p2, ColorDimGray)
			draw.DrawStringFcn(p, " "+drawNumber(mp.Separation, 2), ColorWhite)
		} else if draw.DrawContactForces {
			// multiply by one-half due to relax iteration
			force := half.Mul(mp.TotalNormalImpulse).Mul(invDt)
			p2 := OffsetPos(p, normal.Mul(draw.ForceScale.Mul(force)))
			draw.DrawLineFcn(p, p2, ColorMagenta)
			draw.DrawStringFcn(p, drawNumber(force, 1), ColorWhite)
		}

		if draw.DrawContactFeatures {
			draw.DrawStringFcn(p, strconv.Itoa(int(mp.Id)), ColorOrange)
		}

		if draw.DrawFrictionForces {
			force := half.Mul(mp.TangentImpulse).Mul(invH)
			tangent := RightPerp(normal)
			p2 := OffsetPos(p, tangent.Mul(draw.ForceScale.Mul(force)))
			draw.DrawLineFcn(p, p2, ColorYellow)
			draw.DrawStringFcn(p, drawNumber(force, 1), ColorWhite)
		}
	}
}

// drawIsland draws the union of the fat bounds of an island's shapes. It is
// the island block of b2World_Draw in src/physics_world.c.
func drawIsland(draw *DebugDraw, w *world, island *island) {
	maxValue := QMaxValue()
	shapeCount := 0
	aabb := AABB{
		LowerBound: Vec2{X: maxValue, Y: maxValue},
		UpperBound: Vec2{X: maxValue.Neg(), Y: maxValue.Neg()},
	}

	for _, islandBodyId := range island.bodies {
		for shapeId := w.bodies[islandBodyId].headShapeId; shapeId != nullIndex; shapeId = w.shapes[shapeId].nextShapeId {
			aabb = AABBUnion(aabb, w.fatAABBs[shapeId])
			shapeCount++
		}
	}

	if shapeCount > 0 {
		draw.DrawBoundsFcn(aabb, ColorOrangeRed)
	}
}

// Draw draws the shapes inside draw.DrawingBounds, then the names, mass,
// joints, contacts and islands of their bodies, as the flags select. Start
// from DefaultDebugDraw. A locked world panics. It corresponds to
// b2World_Draw in src/physics_world.c.
func (worldId WorldId) Draw(draw *DebugDraw) {
	w := getWorldFromId(worldId)
	if w.locked {
		panic("dbox2d: the world is locked")
	}
	if draw == nil {
		panic("dbox2d: DebugDraw is nil")
	}
	if !IsValidAABB(draw.DrawingBounds) {
		panic("dbox2d: invalid drawing bounds")
	}

	setBitCountAndClear(&w.debugBodySet, w.bodyIdPool.idCapacity())
	setBitCountAndClear(&w.debugJointSet, w.jointIdPool.idCapacity())
	setBitCountAndClear(&w.debugContactSet, w.contactIdPool.idCapacity())
	setBitCountAndClear(&w.debugIslandSet, w.islandIdPool.idCapacity())

	ctx := drawContext{world: w, draw: draw}
	for i := range w.broadPhase.trees {
		w.broadPhase.trees[i].overlapAABBAll(draw.DrawingBounds, ctx.drawQueryCallback)
	}

	offset := Vec2{X: QMustParse("0.1"), Y: QMustParse("0.1")}
	for k, word := range w.debugBodySet.bits {
		for word != 0 {
			bodyId := 64*k + bits.TrailingZeros64(word)
			body := &w.bodies[bodyId]

			if draw.DrawBodyNames && body.name[0] != 0 {
				sim := getBodySim(w, body)
				transform := WorldTransform{P: sim.center, Q: sim.transform.Q}
				p := TransformWorldPoint(transform, offset)
				draw.DrawStringFcn(p, string(body.name[:bytesIndex(body.name[:])]), ColorWhiteSmoke)
			}

			if draw.DrawMass && body.bodyType == DynamicBody {
				sim := getBodySim(w, body)
				transform := WorldTransform{P: sim.center, Q: sim.transform.Q}
				draw.DrawLineFcn(sim.center0, sim.center, ColorWhiteSmoke)
				draw.DrawTransformFcn(transform)

				p := TransformWorldPoint(transform, offset)
				draw.DrawStringFcn(p, "  "+drawNumber(body.mass, 2), ColorWhite)
			}

			if draw.DrawJoints {
				for jointKey := body.headJointKey; jointKey != nullIndex; {
					jointId, edgeIndex := jointKey>>1, jointKey&1
					joint := &w.joints[jointId]

					// avoid double draw
					if !w.debugJointSet.getBit(jointId) {
						drawJoint(draw, w, joint)
						w.debugJointSet.setBit(jointId)
					}

					jointKey = joint.edges[edgeIndex].nextKey
				}
			}

			if draw.DrawContacts && body.bodyType == DynamicBody {
				for contactKey := body.headContactKey; contactKey != nullIndex; {
					contactId, edgeIndex := contactKey>>1, contactKey&1
					contact := &w.contacts[contactId]
					contactKey = contact.edges[edgeIndex].nextKey

					// avoid double draw
					if !w.debugContactSet.getBit(contactId) {
						drawContactPoints(draw, w, contact)
						w.debugContactSet.setBit(contactId)
					}
				}
			}

			if draw.DrawIslands {
				islandId := body.islandId
				if islandId != nullIndex && !w.debugIslandSet.getBit(islandId) {
					// The reference skips an island without a set by a
					// continue that would revisit the body; a live body's
					// island always has a set, so the port only guards it.
					island := &w.islands[islandId]
					if island.setIndex != nullIndex {
						drawIsland(draw, w, island)
					}
					w.debugIslandSet.setBit(islandId)
				}
			}

			// Clear the smallest set bit
			word &= word - 1
		}
	}
}
