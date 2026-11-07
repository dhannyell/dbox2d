package b2

// body holds the organizational details that the solver does not use.
type body struct {
	name [nameLength + 1]byte

	userData any

	// setIndex is the solver set in the world, or nullIndex.
	setIndex int

	// localIndex is the position of the sim data inside the set, or
	// nullIndex.
	localIndex int

	// headContactKey packs [31: contactId | 1: edgeIndex].
	headContactKey int
	contactCount   int

	headShapeId int
	shapeCount  int

	headChainId int

	// headJointKey packs [31: jointId | 1: edgeIndex].
	headJointKey int
	jointCount   int

	// Every enabled dynamic and kinematic body is in an island.
	islandId int

	// islandIndex is the position of the body in its island.
	islandIndex int

	mass Q

	// inertia is the rotational inertia about the center of mass.
	inertia Q

	sleepThreshold Q
	safetyFactor   Q
	sleepTime      Q

	// bodyMoveIndex adjusts the fellAsleep flag in the body move array.
	bodyMoveIndex int

	id int

	// bodyType is the upstream field named type, which Go reserves.
	bodyType BodyType

	// generation advances on each allocation of this slot, so a stale
	// BodyId fails validation.
	generation uint16

	// flags holds the bodyFlags. The body copy is the master; the sim and
	// the state copies follow through syncBodyFlags.
	flags uint32
}

// bodyState is the solver view of a body. Only awake dynamic and kinematic
// bodies have one. The angular velocity is in radians per second, the unit
// of the reference; the accessors convert it to turns (D-004).
type bodyState struct {
	linearVelocity  Vec2
	angularVelocity Q

	// flags holds the bodyFlags the solver reads: the locks and the speed
	// cap.
	flags uint32

	// deltaPosition reduces round-off error far from the origin.
	deltaPosition Vec2

	// deltaRotation stays identity for a static body, which has no state
	// that the solver can write.
	deltaRotation Rot
}

// identityBodyState returns the state at rest. The delta rotation is the
// identity, (1, 0).
func identityBodyState() bodyState {
	return bodyState{deltaRotation: RotIdentity()}
}

// bodySim is the data that integrates position and velocity. The transform
// serves collision and solver preparation.
type bodySim struct {
	// transform of the body origin
	transform WorldTransform

	// center of mass in world space
	center Pos

	// rotation0 and center0 are the previous rotation and center of mass,
	// for time of impact.
	rotation0 Rot
	center0   Pos

	// localCenter is the center of mass relative to the body origin.
	localCenter Vec2

	force  Vec2
	torque Q

	// The solver reads the inverse mass and inertia in its inner loop, so
	// the layout of the reference stays. A division computes each one once.
	invMass    Q
	invInertia Q

	minExtent      Q
	maxExtent      Q
	linearDamping  Q
	angularDamping Q
	gravityScale   Q

	// bodyId is stable; the sim data moves between sets.
	bodyId int

	// flags holds the bodyFlags.
	flags uint32
}

// The body flags. They correspond to b2BodyFlags in src/body.h.
const (
	// bodyLockLinearX fixes the translation along the x-axis.
	bodyLockLinearX uint32 = 0x00000001

	// bodyLockLinearY fixes the translation along the y-axis.
	bodyLockLinearY uint32 = 0x00000002

	// bodyLockAngularZ fixes the rotation.
	bodyLockAngularZ uint32 = 0x00000004

	// bodyIsFast serves the debug draw. Only on the body sim.
	bodyIsFast uint32 = 0x00000008

	// bodyIsBullet makes a dynamic body do a final continuous pass against
	// every body type, but not other bullets.
	bodyIsBullet uint32 = 0x00000010

	// bodyIsSpeedCapped marks a body speed capped in the current step.
	bodyIsSpeedCapped uint32 = 0x00000020

	// bodyHadTimeOfImpact marks a body the continuous stage moved back to
	// its time of impact in the current step.
	bodyHadTimeOfImpact uint32 = 0x00000040

	// bodyAllowFastRotation lifts the limit on angular velocity.
	bodyAllowFastRotation uint32 = 0x00000080

	// bodyEnlargeBulletBounds marks a bullet whose box must grow. Bullets
	// do not follow the standard broad phase update.
	bodyEnlargeBulletBounds uint32 = 0x00000100

	// bodyDynamicFlag marks a dynamic body.
	bodyDynamicFlag uint32 = 0x00000200

	// bodyEnableSleep lets the body sleep.
	bodyEnableSleep uint32 = 0x00000800

	// bodyEnableContactRecycling lets the contacts of the body keep their
	// manifold while the bodies barely move.
	bodyEnableContactRecycling uint32 = 0x00001000

	bodyAllLocks = bodyLockAngularZ | bodyLockLinearX | bodyLockLinearY

	// bodyFixedRotation is set when the body has fixed rotation.
	bodyFixedRotation = bodyLockAngularZ

	// bodyTransientFlags are transient per time step. They may differ
	// across the body, the sim and the state.
	bodyTransientFlags = bodyIsSpeedCapped | bodyHadTimeOfImpact
)

// syncBodyFlags copies the body flags to the sim and the state when
// something external changes them. It corresponds to b2SyncBodyFlags in
// src/body.c.
func syncBodyFlags(w *world, b *body) {
	sim := getBodySim(w, b)

	// Preserve the sim only flags: fast for the debug draw.
	sim.flags = sim.flags&bodyIsFast | b.flags&^bodyTransientFlags

	if state := getBodyState(w, b); state != nil {
		state.flags = b.flags &^ bodyTransientFlags
	}
}

// motionLockFlags packs the motion locks into body flags.
func motionLockFlags(locks MotionLocks) uint32 {
	var flags uint32
	if locks.LinearX {
		flags |= bodyLockLinearX
	}
	if locks.LinearY {
		flags |= bodyLockLinearY
	}
	if locks.AngularZ {
		flags |= bodyLockAngularZ
	}
	return flags
}

// setDynamicFlag keeps the dynamic flag in step with the body type.
func setDynamicFlag(b *body) {
	if b.bodyType == DynamicBody {
		b.flags |= bodyDynamicFlag
	} else {
		b.flags &^= bodyDynamicFlag
	}
}

// getBodyFullId returns a validated body from an id. It panics on an invalid
// id.
func getBodyFullId(w *world, bodyId BodyId) *body {
	if !bodyId.IsValid() {
		panic("dbox2d: invalid BodyId")
	}

	// the id index starts at one, so zero can represent null
	return &w.bodies[bodyId.index1-1]
}

// getBodyTransformQuick returns the transform of a body already in hand.
func getBodyTransformQuick(w *world, b *body) WorldTransform {
	set := &w.solverSets[b.setIndex]
	return set.bodySims[b.localIndex].transform
}

// getBodyTransform returns the transform of a body by raw id. It corresponds
// to b2GetBodyTransform in src/body.c.
func getBodyTransform(w *world, bodyId int) WorldTransform {
	b := &w.bodies[bodyId]
	return getBodyTransformQuick(w, b)
}

// makeBodyId builds a BodyId from a raw id.
func makeBodyId(w *world, bodyId int) BodyId {
	b := &w.bodies[bodyId]
	return BodyId{index1: int32(bodyId) + 1, world0: w.worldId, generation: b.generation}
}

// getBodySim returns the sim data of a body.
func getBodySim(w *world, b *body) *bodySim {
	set := &w.solverSets[b.setIndex]
	return &set.bodySims[b.localIndex]
}

// getBodyState returns the solver state of a body, or nil for a body outside
// the awake set.
func getBodyState(w *world, b *body) *bodyState {
	if b.setIndex == awakeSet {
		set := &w.solverSets[awakeSet]
		return &set.bodyStates[b.localIndex]
	}

	return nil
}

// limitVelocity caps the linear velocity as b2LimitVelocity does in
// src/body.c.
func limitVelocity(state *bodyState, maxLinearSpeed Q) {
	v2 := state.linearVelocity.Dot(state.linearVelocity)
	if maxLinearSpeed.Mul(maxLinearSpeed).Less(v2) {
		state.linearVelocity = state.linearVelocity.Mul(maxLinearSpeed.Div(v2.Sqrt()))
	}
}

// CreateBody creates a rigid body from a definition. The definition must come
// from DefaultBodyDef and is not retained.
func CreateBody(worldId WorldId, def *BodyDef) BodyId {
	checkDef(def.internalValue)
	if !IsValidPos(def.Position) {
		panic("dbox2d: BodyDef.Position is not valid")
	}
	if !IsValidRot(def.Rotation) {
		panic("dbox2d: BodyDef.Rotation is not valid")
	}
	if !IsValidVec2(def.LinearVelocity) {
		panic("dbox2d: BodyDef.LinearVelocity is not valid")
	}
	if !IsValidQ(def.AngularVelocity) {
		panic("dbox2d: BodyDef.AngularVelocity is not valid")
	}
	zero := QZero()
	if !IsValidQ(def.LinearDamping) || def.LinearDamping.Less(zero) {
		panic("dbox2d: BodyDef.LinearDamping is not valid")
	}
	if !IsValidQ(def.AngularDamping) || def.AngularDamping.Less(zero) {
		panic("dbox2d: BodyDef.AngularDamping is not valid")
	}
	if !IsValidQ(def.SleepThreshold) || def.SleepThreshold.Less(zero) {
		panic("dbox2d: BodyDef.SleepThreshold is not valid")
	}
	if !IsValidQ(def.SafetyFactor) || def.SafetyFactor.Less(zero) {
		panic("dbox2d: BodyDef.SafetyFactor is not valid")
	}
	if !IsValidQ(def.GravityScale) {
		panic("dbox2d: BodyDef.GravityScale is not valid")
	}

	w := getWorldFromId(worldId)
	if w.locked {
		panic("dbox2d: the world is locked")
	}

	isAwake := (def.IsAwake || !def.EnableSleep) && def.IsEnabled

	// determine the solver set
	var setId int
	switch {
	case !def.IsEnabled:
		// any body type can be disabled
		setId = disabledSet
	case def.Type == StaticBody:
		setId = staticSet
	case isAwake:
		setId = awakeSet
	default:
		// new set for a sleeping body in its own island
		setId = w.solverSetIdPool.allocId()
		if setId == len(w.solverSets) {
			w.solverSets = append(w.solverSets, solverSet{setIndex: nullIndex})
		} else if w.solverSets[setId].setIndex != nullIndex {
			panic("dbox2d: the solver set slot is still in use")
		}

		w.solverSets[setId].setIndex = setId
	}

	if setId < 0 || setId >= len(w.solverSets) {
		panic("dbox2d: the solver set index is out of range")
	}

	bodyId := w.bodyIdPool.allocId()

	flags := motionLockFlags(def.MotionLocks)
	if def.IsBullet {
		flags |= bodyIsBullet
	}
	if def.AllowFastRotation {
		flags |= bodyAllowFastRotation
	}
	if def.Type == DynamicBody {
		flags |= bodyDynamicFlag
	}
	if def.EnableSleep {
		flags |= bodyEnableSleep
	}
	if def.EnableContactRecycling {
		flags |= bodyEnableContactRecycling
	}

	set := &w.solverSets[setId]
	set.bodySims = append(set.bodySims, bodySim{
		transform:      WorldTransform{P: def.Position, Q: def.Rotation},
		center:         def.Position,
		rotation0:      def.Rotation,
		center0:        def.Position,
		minExtent:      Huge,
		maxExtent:      QZero(),
		linearDamping:  def.LinearDamping,
		angularDamping: def.AngularDamping,
		gravityScale:   def.GravityScale,
		bodyId:         bodyId,
		flags:          flags,
	})

	if setId == awakeSet {
		state := identityBodyState()
		state.linearVelocity = def.LinearVelocity
		state.angularVelocity = def.AngularVelocity.Mul(tau)
		state.flags = flags
		set.bodyStates = append(set.bodyStates, state)
	}

	if bodyId == len(w.bodies) {
		w.bodies = append(w.bodies, body{})
	} else if w.bodies[bodyId].id != nullIndex {
		panic("dbox2d: the body slot is still in use")
	}

	b := &w.bodies[bodyId]

	// the name keeps at most nameLength bytes, as the reference does
	b.name = [nameLength + 1]byte{}
	copy(b.name[:nameLength], def.Name)

	b.userData = def.UserData
	b.setIndex = setId
	b.localIndex = len(set.bodySims) - 1
	b.generation += 1
	b.headShapeId = nullIndex
	b.shapeCount = 0
	b.headChainId = nullIndex
	b.headContactKey = nullIndex
	b.contactCount = 0
	b.headJointKey = nullIndex
	b.jointCount = 0
	b.islandId = nullIndex
	b.islandIndex = nullIndex
	b.bodyMoveIndex = nullIndex
	b.id = bodyId
	b.mass = QZero()
	b.inertia = QZero()
	b.sleepThreshold = def.SleepThreshold
	b.safetyFactor = def.SafetyFactor
	b.sleepTime = QZero()
	b.bodyType = def.Type
	b.flags = flags

	// dynamic and kinematic bodies that are enabled need a island
	if setId >= awakeSet {
		createIslandForBody(w, setId, b)
	}

	id := BodyId{index1: int32(bodyId) + 1, world0: w.worldId, generation: b.generation}
	if w.recording != nil {
		w.recording.recRet(opCreateBody, id, worldId, *def)
	}
	return id
}

// DestroyBody destroys a body. Every shape attached to it goes away too.
func DestroyBody(bodyId BodyId) {
	w := getWorldLocked(bodyId.world0)
	if w.recording != nil {
		w.recording.rec(opDestroyBody, bodyId)
	}

	b := getBodyFullId(w, bodyId)

	// Wake bodies attached to this body, even if this body is static.
	wakeBodies := true

	// Destroy the attached joints
	edgeKey := b.headJointKey
	for edgeKey != nullIndex {
		jointId := edgeKey >> 1
		edgeIndex := edgeKey & 1

		j := &w.joints[jointId]
		edgeKey = j.edges[edgeIndex].nextKey

		// Careful because this modifies the list being traversed
		destroyJointInternal(w, j)
	}

	// Destroy all contacts attached to this body.
	destroyBodyContacts(w, b, wakeBodies)

	// Destroy the attached shapes and their broad-phase proxies.
	shapeId := b.headShapeId
	for shapeId != nullIndex {
		s := &w.shapes[shapeId]

		if s.sensorIndex != nullIndex {
			destroySensor(w, s)
		}

		destroyShapeProxy(s, &w.broadPhase)

		// Return shape to free list.
		w.shapeIdPool.freeId(shapeId)
		s.id = nullIndex

		shapeId = s.nextShapeId
	}

	// Destroy the attached chains. The associated shapes have already been destroyed above.
	for chainID := b.headChainId; chainID != nullIndex; {
		chain := &w.chainShapes[chainID]
		nextChainID := chain.nextChainId

		freeChainData(chain)

		// Return chain to free list.
		w.chainIdPool.freeId(chainID)
		chain.id = nullIndex

		chainID = nextChainID
	}

	removeBodyFromIsland(w, b)

	// Remove body sim from solver set that owns it
	set := &w.solverSets[b.setIndex]
	var movedIndex int
	set.bodySims, movedIndex = removeSwapNoClear(set.bodySims, b.localIndex)
	if movedIndex != nullIndex {
		// Fix moved body index
		movedSim := &set.bodySims[b.localIndex]
		movedBody := &w.bodies[movedSim.bodyId]
		if movedBody.localIndex != movedIndex {
			panic("dbox2d: the moved body index does not match")
		}
		movedBody.localIndex = b.localIndex
	}

	// Remove body state from awake set
	if b.setIndex == awakeSet {
		var result int
		set.bodyStates, result = removeSwapNoClear(set.bodyStates, b.localIndex)
		if result != movedIndex {
			panic("dbox2d: the sim and state arrays moved different indices")
		}
	} else if set.setIndex >= firstSleepingSet && len(set.bodySims) == 0 {
		// Remove solver set if it is now an orphan.
		destroySolverSet(w, set.setIndex)
	}

	// Free body and id (preserve body generation)
	w.bodyIdPool.freeId(b.id)

	b.setIndex = nullIndex
	b.localIndex = nullIndex
	b.id = nullIndex
}

// updateBodyMassData recomputes the mass, the center of mass and the extents
// of a body from its shapes.
func updateBodyMassData(w *world, b *body) {
	sim := getBodySim(w, b)

	// Compute mass data from shapes. Each shape has its own density.
	zero := QZero()
	b.mass = zero
	b.inertia = zero

	sim.invMass = zero
	sim.invInertia = zero
	sim.localCenter = Vec2Zero()
	sim.minExtent = Huge
	sim.maxExtent = zero

	// Static and kinematic sims have zero mass.
	if b.bodyType != DynamicBody {
		sim.center = sim.transform.P

		// Need extents for kinematic bodies for sleeping to work correctly.
		if b.bodyType == KinematicBody {
			shapeId := b.headShapeId
			for shapeId != nullIndex {
				s := &w.shapes[shapeId]

				extent := computeShapeExtent(s, Vec2Zero())
				sim.minExtent = sim.minExtent.Min(extent.minExtent)
				sim.maxExtent = sim.maxExtent.Max(extent.maxExtent)

				shapeId = s.nextShapeId
			}
		}

		return
	}

	// Accumulate mass over all shapes.
	localCenter := Vec2Zero()
	shapeId := b.headShapeId
	for shapeId != nullIndex {
		s := &w.shapes[shapeId]
		shapeId = s.nextShapeId

		if s.density.Eq(zero) {
			continue
		}

		massData := computeShapeMass(s)
		b.mass = b.mass.Add(massData.Mass)
		localCenter = MulAdd(localCenter, massData.Mass, massData.Center)
	}

	// Compute center of mass.
	if zero.Less(b.mass) {
		sim.invMass = QOne().Div(b.mass)
		localCenter = localCenter.Mul(sim.invMass)
	}

	// Second loop to accumulate the rotational inertia about the center of
	// mass. The reference keeps the mass data of the first loop in a stack
	// buffer; the port computes it again, which gives the same values.
	shapeId = b.headShapeId
	for shapeId != nullIndex {
		s := &w.shapes[shapeId]
		shapeId = s.nextShapeId

		if s.density.Eq(zero) {
			continue
		}

		massData := computeShapeMass(s)
		if massData.Mass.Eq(zero) {
			continue
		}

		// Shift to center of mass. This is safe because it can only increase.
		offset := localCenter.Sub(massData.Center)
		inertia := massData.RotationalInertia.Add(massData.Mass.Mul(offset.Dot(offset)))
		b.inertia = b.inertia.Add(inertia)
	}

	if b.inertia.Less(zero) {
		panic("dbox2d: the body inertia is negative")
	}

	if zero.Less(b.inertia) && b.flags&bodyFixedRotation == 0 {
		sim.invInertia = QOne().Div(b.inertia)
	} else {
		b.inertia = zero
		sim.invInertia = zero
	}

	// Move center of mass.
	oldCenter := sim.center
	sim.localCenter = localCenter
	sim.center = TransformWorldPoint(sim.transform, sim.localCenter)
	sim.center0 = sim.center

	// Update center of mass velocity.
	state := getBodyState(w, b)
	if state != nil {
		deltaLinear := CrossSV(state.angularVelocity, SubPos(sim.center, oldCenter))
		state.linearVelocity = state.linearVelocity.Add(deltaLinear)
	}

	// Compute body extents relative to center of mass
	shapeId = b.headShapeId
	for shapeId != nullIndex {
		s := &w.shapes[shapeId]

		extent := computeShapeExtent(s, localCenter)
		sim.minExtent = sim.minExtent.Min(extent.minExtent)
		sim.maxExtent = sim.maxExtent.Max(extent.maxExtent)

		shapeId = s.nextShapeId
	}

	// When the center of mass changes, any cached contact manifold becomes invalid.
	invalidateContactRecycling(w, b)
}

// invalidateContactRecycling clears the cached relative transform of every
// contact of a body, so the next collide recomputes the manifolds.
func invalidateContactRecycling(w *world, b *body) {
	edgeKey := b.headContactKey
	for edgeKey != nullIndex {
		contactId := edgeKey >> 1
		edgeIndex := edgeKey & 1

		c := &w.contacts[contactId]
		cs := getContactSim(w, c)
		cs.simFlags &^= simRelativeTransformValid

		edgeKey = c.edges[edgeIndex].nextKey
	}
}

// GetType returns the body type. It corresponds to b2Body_GetType.
func (bodyId BodyId) GetType() BodyType {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return b.bodyType
}

// SetType changes the body type. It corresponds to b2Body_SetType.
func (bodyId BodyId) SetType(bodyType BodyType) {
	w := getWorld(bodyId.world0)
	if w.locked {
		panic("dbox2d: the world is locked")
	}
	if w.recording != nil {
		w.recording.rec(opBodySetType, bodyId, int(bodyType))
	}
	b := getBodyFullId(w, bodyId)

	originalType := b.bodyType
	if originalType == bodyType {
		return
	}

	if b.setIndex == disabledSet {
		b.bodyType = bodyType
		setDynamicFlag(b)
		syncBodyFlags(w, b)
		updateBodyMassData(w, b)
		return
	}

	// Stage 2: destroy all contacts
	destroyBodyContacts(w, b, false)

	// Stage 3: wake this body (does nothing if body is static), otherwise it
	// will also wake all bodies in the same sleeping solver set.
	wakeBody(w, b)

	// Stage 4: move joints to temporary storage
	staticSolverSet := &w.solverSets[staticSet]

	for jointKey := b.headJointKey; jointKey != nullIndex; {
		jointId := jointKey >> 1
		edgeIndex := jointKey & 1
		j := &w.joints[jointId]
		jointKey = j.edges[edgeIndex].nextKey

		// Joint may be disabled by other body
		if j.setIndex == disabledSet {
			continue
		}

		// Wake attached bodies. The wakeBody call above does not wake bodies
		// attached to a static body. But it is necessary because the body may
		// have no joints.
		wakeBody(w, &w.bodies[j.edges[0].bodyId])
		wakeBody(w, &w.bodies[j.edges[1].bodyId])

		// Remove joint from island
		if j.islandId != nullIndex {
			unlinkJoint(w, j)
		}

		// It is necessary to transfer all joints to the static set so they can
		// be added to the constraint graph below and acquire consistent colors.
		if j.setIndex != staticSet {
			transferJoint(w, staticSolverSet, &w.solverSets[j.setIndex], j)
		}
	}

	// Stage 5: change the body type and transfer body
	b.bodyType = bodyType
	setDynamicFlag(b)

	awakeSolverSet := &w.solverSets[awakeSet]
	sourceSet := &w.solverSets[b.setIndex]
	targetSet := awakeSolverSet
	if bodyType == StaticBody {
		targetSet = staticSolverSet
	}

	// Transfer body
	if targetSet != sourceSet {
		transferBody(w, targetSet, sourceSet, b)
	}

	// Stage 6: update island participation for the body
	if originalType == StaticBody {
		// Create island for body
		createIslandForBody(w, awakeSet, b)
	} else if bodyType == StaticBody {
		// Remove body from island.
		removeBodyFromIsland(w, b)
		sim := &staticSolverSet.bodySims[b.localIndex]
		sim.flags &^= bodyIsFast | bodyTransientFlags
	}

	// Stage 7: Transfer joints to the target set
	for jointKey := b.headJointKey; jointKey != nullIndex; {
		jointId := jointKey >> 1
		edgeIndex := jointKey & 1
		j := &w.joints[jointId]
		jointKey = j.edges[edgeIndex].nextKey

		// Joint may be disabled by other body
		if j.setIndex == disabledSet {
			continue
		}

		// All joints were transferred to the static set in an earlier stage
		if j.setIndex != staticSet {
			panic("dbox2d: a joint was not moved to the static set")
		}

		bodyA := &w.bodies[j.edges[0].bodyId]
		bodyB := &w.bodies[j.edges[1].bodyId]
		if bodyA.bodyType == DynamicBody || bodyB.bodyType == DynamicBody {
			transferJoint(w, awakeSolverSet, staticSolverSet, j)
		}
	}

	// Recreate shape proxies in broadphase
	transform := getBodyTransformQuick(w, b)
	for shapeId := b.headShapeId; shapeId != nullIndex; {
		s := &w.shapes[shapeId]
		shapeId = s.nextShapeId
		destroyShapeProxy(s, &w.broadPhase)
		createShapeProxy(w, s, bodyType, transform, true)
	}

	// Relink all joints
	for jointKey := b.headJointKey; jointKey != nullIndex; {
		jointId := jointKey >> 1
		edgeIndex := jointKey & 1
		j := &w.joints[jointId]
		jointKey = j.edges[edgeIndex].nextKey

		otherBody := &w.bodies[j.edges[edgeIndex^1].bodyId]
		if otherBody.setIndex == disabledSet {
			continue
		}
		if b.bodyType != DynamicBody && otherBody.bodyType != DynamicBody {
			continue
		}
		linkJoint(w, j)
	}

	syncBodyFlags(w, b)

	updateBodyMassData(w, b)
}

// GetPosition returns the world position of the body origin. It corresponds
// to b2Body_GetPosition.
func (bodyId BodyId) GetPosition() Pos {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return getBodyTransformQuick(w, b).P
}

// GetRotation returns the world rotation of the body. It corresponds to
// b2Body_GetRotation.
func (bodyId BodyId) GetRotation() Rot {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return getBodyTransformQuick(w, b).Q
}

// GetTransform returns the world transform of the body. It corresponds to
// b2Body_GetTransform.
func (bodyId BodyId) GetTransform() WorldTransform {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return getBodyTransformQuick(w, b)
}

// SetTransform changes the body's world transform. It corresponds to
// b2Body_SetTransform.
func (bodyId BodyId) SetTransform(position Pos, rotation Rot) {
	if !IsValidPos(position) {
		panic("dbox2d: SetTransform position is not valid")
	}
	if !IsValidRot(rotation) {
		panic("dbox2d: SetTransform rotation is not valid")
	}
	w := getWorld(bodyId.world0)
	if w.locked {
		panic("dbox2d: the world is locked")
	}
	if w.recording != nil {
		w.recording.rec(opBodySetTransform, bodyId, wirePos{position}, rotation)
	}
	b := getBodyFullId(w, bodyId)
	sim := getBodySim(w, b)

	sim.transform = WorldTransform{P: position, Q: rotation}
	sim.center = TransformWorldPoint(sim.transform, sim.localCenter)
	sim.rotation0 = sim.transform.Q
	sim.center0 = sim.center

	for shapeId := b.headShapeId; shapeId != nullIndex; shapeId = w.shapes[shapeId].nextShapeId {
		s := &w.shapes[shapeId]
		aabb := computeFatShapeAABB(s, sim.transform, speculativeDistance)
		s.aabb = aabb

		if !AABBContains(w.fatAABBs[shapeId], aabb) {
			fatAABB := fattenAABB(aabb, s.aabbMargin)
			w.fatAABBs[shapeId] = fatAABB

			// The body could be disabled
			if s.proxyKey != nullIndex {
				w.broadPhase.moveProxy(s.proxyKey, fatAABB)
			}
		}
	}
}

// SetTargetTransform sets the velocity needed to reach a target transform
// in timeStep. A sleeping body is left alone unless wake is true, and it
// stays asleep when the motion would be below its sleep threshold. It
// corresponds to b2Body_SetTargetTransform.
func (bodyId BodyId) SetTargetTransform(target WorldTransform, timeStep Q, wake bool) {
	if !IsValidWorldTransform(target) || !IsValidQ(timeStep) {
		panic("dbox2d: SetTargetTransform needs a valid target and time step")
	}
	w := getWorld(bodyId.world0)
	if w.recording != nil {
		w.recording.rec(opBodySetTargetTransform, bodyId, wireWorldXf{target}, timeStep, wake)
	}
	b := getBodyFullId(w, bodyId)
	if b.setIndex == disabledSet {
		return
	}

	zero := QZero()
	if b.bodyType == StaticBody || !zero.Less(timeStep) {
		return
	}

	if b.setIndex != awakeSet && !wake {
		return
	}

	sim := getBodySim(w, b)

	// D-006: the reciprocal is an exact fixed-point division.
	invTimeStep := QOne().Div(timeStep)
	delta := SubPos(TransformWorldPoint(target, sim.localCenter), sim.center)
	linearVelocity := delta.Mul(invTimeStep)

	// D-004: the state stores radians.
	deltaAngle := relativeAngle(sim.transform.Q, target.Q)
	angularVelocity := angleRadians(deltaAngle).Mul(invTimeStep)

	// Early out if the body is asleep already and the desired movement is
	// small.
	if b.setIndex != awakeSet {
		maxVelocity := linearVelocity.Len().Add(angularVelocity.Abs().Mul(sim.maxExtent))

		// Return if velocity would be sleepy
		if maxVelocity.Less(b.sleepThreshold) {
			return
		}

		// Must wake for state to exist
		wakeBody(w, b)
	}

	state := getBodyState(w, b)
	state.linearVelocity = linearVelocity
	state.angularVelocity = angularVelocity
}

// GetLocalPoint converts a world point to the body's local frame. It
// corresponds to b2Body_GetLocalPoint.
func (bodyId BodyId) GetLocalPoint(worldPoint Pos) Vec2 {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return InvTransformWorldPoint(getBodyTransformQuick(w, b), worldPoint)
}

// GetWorldPoint converts a local point to the world frame. It corresponds to
// b2Body_GetWorldPoint.
func (bodyId BodyId) GetWorldPoint(localPoint Vec2) Pos {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return TransformWorldPoint(getBodyTransformQuick(w, b), localPoint)
}

// GetLocalVector converts a world vector to the body's local frame. It
// corresponds to b2Body_GetLocalVector.
func (bodyId BodyId) GetLocalVector(worldVector Vec2) Vec2 {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return InvRotateVector(getBodyTransformQuick(w, b).Q, worldVector)
}

// GetWorldVector converts a local vector to the world frame. It corresponds
// to b2Body_GetWorldVector.
func (bodyId BodyId) GetWorldVector(localVector Vec2) Vec2 {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return RotateVector(getBodyTransformQuick(w, b).Q, localVector)
}

// GetLinearVelocity returns the body's linear velocity. It corresponds to
// b2Body_GetLinearVelocity.
func (bodyId BodyId) GetLinearVelocity() Vec2 {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	state := getBodyState(w, b)
	if state != nil {
		return state.linearVelocity
	}
	return Vec2Zero()
}

// GetAngularVelocity returns the body's angular velocity in turns per
// second. It corresponds to b2Body_GetAngularVelocity.
func (bodyId BodyId) GetAngularVelocity() Q {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	state := getBodyState(w, b)
	if state != nil {
		// D-004: the state stores radians per second and the API reports turns.
		return state.angularVelocity.Div(tau)
	}
	return QZero()
}

// SetLinearVelocity changes the body's linear velocity. It corresponds to
// b2Body_SetLinearVelocity.
func (bodyId BodyId) SetLinearVelocity(linearVelocity Vec2) {
	w := getWorld(bodyId.world0)
	if w.recording != nil {
		w.recording.rec(opBodySetLinearVelocity, bodyId, linearVelocity)
	}
	b := getBodyFullId(w, bodyId)
	if b.bodyType == StaticBody {
		return
	}
	if QZero().Less(linearVelocity.Dot(linearVelocity)) {
		wakeBody(w, b)
	}
	state := getBodyState(w, b)
	if state == nil {
		return
	}
	state.linearVelocity = linearVelocity
}

// SetAngularVelocity changes the body's angular velocity in turns per
// second. It corresponds to b2Body_SetAngularVelocity.
func (bodyId BodyId) SetAngularVelocity(angularVelocity Q) {
	// D-004: the API takes turns per second and the state stores radians.
	bodyId.setAngularVelocityRadians(angularVelocity.Mul(tau))
}

// setAngularVelocityRadians is SetAngularVelocity in the unit of the body
// state. The recording stores this value and the replay passes it back, so
// neither direction converts (D-025).
func (bodyId BodyId) setAngularVelocityRadians(angularVelocity Q) {
	w := getWorld(bodyId.world0)
	if w.recording != nil {
		w.recording.rec(opBodySetAngularVelocity, bodyId, angularVelocity)
	}
	b := getBodyFullId(w, bodyId)
	if b.bodyType == StaticBody || b.flags&bodyLockAngularZ != 0 {
		return
	}
	if !angularVelocity.Eq(QZero()) {
		wakeBody(w, b)
	}
	state := getBodyState(w, b)
	if state == nil {
		return
	}
	state.angularVelocity = angularVelocity
}

// GetLocalPointVelocity returns the velocity at a local point. It corresponds
// to b2Body_GetLocalPointVelocity.
func (bodyId BodyId) GetLocalPointVelocity(localPoint Vec2) Vec2 {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	state := getBodyState(w, b)
	if state == nil {
		return Vec2Zero()
	}
	sim := getBodySim(w, b)
	r := RotateVector(sim.transform.Q, localPoint.Sub(sim.localCenter))
	return state.linearVelocity.Add(CrossSV(state.angularVelocity, r))
}

// GetWorldPointVelocity returns the velocity at a world point. It corresponds
// to b2Body_GetWorldPointVelocity.
func (bodyId BodyId) GetWorldPointVelocity(worldPoint Pos) Vec2 {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	state := getBodyState(w, b)
	if state == nil {
		return Vec2Zero()
	}
	sim := getBodySim(w, b)
	return state.linearVelocity.Add(CrossSV(state.angularVelocity, SubPos(worldPoint, sim.center)))
}

// ApplyForce applies a force at a world point. It corresponds to
// b2Body_ApplyForce.
func (bodyId BodyId) ApplyForce(force Vec2, point Pos, wake bool) {
	w := getWorld(bodyId.world0)
	if w.recording != nil {
		w.recording.rec(opBodyApplyForce, bodyId, force, wirePos{point}, wake)
	}
	b := getBodyFullId(w, bodyId)
	if wake && b.setIndex >= firstSleepingSet {
		wakeBody(w, b)
	}
	if b.setIndex == awakeSet {
		sim := getBodySim(w, b)
		sim.force = sim.force.Add(force)
		sim.torque = sim.torque.Add(Cross(SubPos(point, sim.center), force))
	}
}

// ApplyForceToCenter applies a force at the center of mass. It corresponds
// to b2Body_ApplyForceToCenter.
func (bodyId BodyId) ApplyForceToCenter(force Vec2, wake bool) {
	w := getWorld(bodyId.world0)
	if w.recording != nil {
		w.recording.rec(opBodyApplyForceToCenter, bodyId, force, wake)
	}
	b := getBodyFullId(w, bodyId)
	if wake && b.setIndex >= firstSleepingSet {
		wakeBody(w, b)
	}
	if b.setIndex == awakeSet {
		sim := getBodySim(w, b)
		sim.force = sim.force.Add(force)
	}
}

// ApplyTorque applies a torque. It corresponds to b2Body_ApplyTorque.
func (bodyId BodyId) ApplyTorque(torque Q, wake bool) {
	w := getWorld(bodyId.world0)
	if w.recording != nil {
		w.recording.rec(opBodyApplyTorque, bodyId, torque, wake)
	}
	b := getBodyFullId(w, bodyId)
	if wake && b.setIndex >= firstSleepingSet {
		wakeBody(w, b)
	}
	if b.setIndex == awakeSet {
		getBodySim(w, b).torque = getBodySim(w, b).torque.Add(torque)
	}
}

// ApplyLinearImpulse applies a linear impulse at a world point. It
// corresponds to b2Body_ApplyLinearImpulse.
func (bodyId BodyId) ApplyLinearImpulse(impulse Vec2, point Pos, wake bool) {
	w := getWorld(bodyId.world0)
	if w.recording != nil {
		w.recording.rec(opBodyApplyLinearImpulse, bodyId, impulse, wirePos{point}, wake)
	}
	b := getBodyFullId(w, bodyId)
	if wake && b.setIndex >= firstSleepingSet {
		wakeBody(w, b)
	}
	if b.setIndex == awakeSet {
		state := getBodyState(w, b)
		sim := getBodySim(w, b)
		state.linearVelocity = MulAdd(state.linearVelocity, sim.invMass, impulse)
		state.angularVelocity = state.angularVelocity.Add(sim.invInertia.Mul(Cross(SubPos(point, sim.center), impulse)))
		limitVelocity(state, w.maxLinearSpeed)
	}
}

// ApplyLinearImpulseToCenter applies a linear impulse at the center of mass.
// It corresponds to b2Body_ApplyLinearImpulseToCenter.
func (bodyId BodyId) ApplyLinearImpulseToCenter(impulse Vec2, wake bool) {
	w := getWorld(bodyId.world0)
	if w.recording != nil {
		w.recording.rec(opBodyApplyLinearImpulseToCenter, bodyId, impulse, wake)
	}
	b := getBodyFullId(w, bodyId)
	if wake && b.setIndex >= firstSleepingSet {
		wakeBody(w, b)
	}
	if b.setIndex == awakeSet {
		state := getBodyState(w, b)
		sim := getBodySim(w, b)
		state.linearVelocity = MulAdd(state.linearVelocity, sim.invMass, impulse)
		limitVelocity(state, w.maxLinearSpeed)
	}
}

// ApplyAngularImpulse applies an angular impulse and receives the impulse in
// the reference's angular units. It corresponds to b2Body_ApplyAngularImpulse.
func (bodyId BodyId) ApplyAngularImpulse(impulse Q, wake bool) {
	w := getWorld(bodyId.world0)
	if w.recording != nil {
		w.recording.rec(opBodyApplyAngularImpulse, bodyId, impulse, wake)
	}
	b := getBodyFullId(w, bodyId)
	if wake && b.setIndex >= firstSleepingSet {
		wakeBody(w, b)
	}
	if b.setIndex == awakeSet {
		state := getBodyState(w, b)
		sim := getBodySim(w, b)
		state.angularVelocity = state.angularVelocity.Add(sim.invInertia.Mul(impulse))
	}
}

// GetMass returns the mass of the body, usually in kilograms. It corresponds
// to b2Body_GetMass.
func (bodyId BodyId) GetMass() Q {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return b.mass
}

// GetRotationalInertia returns the inertia about the center of mass. It
// corresponds to b2Body_GetRotationalInertia.
func (bodyId BodyId) GetRotationalInertia() Q {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return b.inertia
}

// GetLocalCenter returns the center of mass in local coordinates. It
// corresponds to b2Body_GetLocalCenter.
func (bodyId BodyId) GetLocalCenter() Vec2 {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return getBodySim(w, b).localCenter
}

// GetWorldCenter returns the center of mass in world coordinates. It
// corresponds to b2Body_GetWorldCenter.
func (bodyId BodyId) GetWorldCenter() Pos {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return getBodySim(w, b).center
}

// GetMinExtent returns the minimum distance from any point on the body
// shapes to the center of mass. It corresponds to b2Body_GetMinExtent.
func (bodyId BodyId) GetMinExtent() Q {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return getBodySim(w, b).minExtent
}

// GetMaxExtent returns the maximum distance from any point on the body
// shapes to the center of mass. It corresponds to b2Body_GetMaxExtent.
func (bodyId BodyId) GetMaxExtent() Q {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return getBodySim(w, b).maxExtent
}

// GetMaxExtentFromOrigin returns a conservative maximum distance from any
// point on the body shapes to the body origin. It corresponds to
// b2Body_GetMaxExtentFromOrigin.
func (bodyId BodyId) GetMaxExtentFromOrigin() Q {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	sim := getBodySim(w, b)
	return sim.maxExtent.Add(sim.localCenter.Len())
}

// ClearForces clears the force and torque accumulated on the body. The
// world step clears them anyway; call this only to drop forces applied
// before the next step. It corresponds to b2Body_ClearForces.
func (bodyId BodyId) ClearForces() {
	w := getWorld(bodyId.world0)
	if w.recording != nil {
		w.recording.rec(opBodyClearForces, bodyId)
	}
	b := getBodyFullId(w, bodyId)
	sim := getBodySim(w, b)
	sim.force = Vec2Zero()
	sim.torque = QZero()
}

// GetMassData returns the body's mass, local center and rotational inertia.
// It corresponds to b2Body_GetMassData.
func (bodyId BodyId) GetMassData() MassData {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	sim := getBodySim(w, b)
	return MassData{Mass: b.mass, Center: sim.localCenter, RotationalInertia: b.inertia}
}

// SetMassData overrides the body's mass, center and rotational inertia. It
// corresponds to b2Body_SetMassData.
func (bodyId BodyId) SetMassData(massData MassData) {
	w := getWorldLocked(bodyId.world0)
	if w.recording != nil {
		w.recording.rec(opBodySetMassData, bodyId, massData)
	}
	zero := QZero()
	if !IsValidQ(massData.Mass) || massData.Mass.Less(zero) {
		panic("dbox2d: mass data has an invalid mass")
	}
	if !IsValidQ(massData.RotationalInertia) || massData.RotationalInertia.Less(zero) {
		panic("dbox2d: mass data has an invalid rotational inertia")
	}
	if !IsValidVec2(massData.Center) {
		panic("dbox2d: mass data has an invalid center")
	}

	b := getBodyFullId(w, bodyId)
	sim := getBodySim(w, b)
	b.mass = massData.Mass
	b.inertia = massData.RotationalInertia
	sim.localCenter = massData.Center
	sim.center = TransformWorldPoint(sim.transform, massData.Center)
	sim.center0 = sim.center

	if zero.Less(b.mass) {
		// D-006: the reciprocal is an exact fixed-point division.
		sim.invMass = QOne().Div(b.mass)
	} else {
		sim.invMass = zero
	}
	if zero.Less(b.inertia) {
		// D-006: the reciprocal is an exact fixed-point division.
		sim.invInertia = QOne().Div(b.inertia)
	} else {
		sim.invInertia = zero
	}

	// When the center of mass changes, any cached contact manifold becomes invalid.
	invalidateContactRecycling(w, b)

	// Motion locks take priority over mass data.
	if b.flags&bodyFixedRotation != 0 {
		b.inertia = zero
		sim.invInertia = zero
	}
}

// UpdateMassFromShapes recomputes mass data from the body's shapes. It
// corresponds to b2Body_UpdateMassFromShapes.
func (bodyId BodyId) UpdateMassFromShapes() {
	w := getWorldLocked(bodyId.world0)
	if w.recording != nil {
		w.recording.rec(opBodyUpdateMassFromShapes, bodyId)
	}
	b := getBodyFullId(w, bodyId)
	updateBodyMassData(w, b)
}

// SetLinearDamping changes linear damping. It corresponds to
// b2Body_SetLinearDamping.
func (bodyId BodyId) SetLinearDamping(linearDamping Q) {
	w := getWorldLocked(bodyId.world0)
	if w.recording != nil {
		w.recording.rec(opBodySetLinearDamping, bodyId, linearDamping)
	}
	b := getBodyFullId(w, bodyId)
	getBodySim(w, b).linearDamping = linearDamping
}

// GetLinearDamping returns linear damping. It corresponds to
// b2Body_GetLinearDamping.
func (bodyId BodyId) GetLinearDamping() Q {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return getBodySim(w, b).linearDamping
}

// SetAngularDamping changes angular damping. It corresponds to
// b2Body_SetAngularDamping.
func (bodyId BodyId) SetAngularDamping(angularDamping Q) {
	w := getWorldLocked(bodyId.world0)
	if w.recording != nil {
		w.recording.rec(opBodySetAngularDamping, bodyId, angularDamping)
	}
	b := getBodyFullId(w, bodyId)
	getBodySim(w, b).angularDamping = angularDamping
}

// GetAngularDamping returns angular damping. It corresponds to
// b2Body_GetAngularDamping.
func (bodyId BodyId) GetAngularDamping() Q {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return getBodySim(w, b).angularDamping
}

// SetGravityScale changes the body's gravity scale. It corresponds to
// b2Body_SetGravityScale.
func (bodyId BodyId) SetGravityScale(gravityScale Q) {
	w := getWorldLocked(bodyId.world0)
	if w.recording != nil {
		w.recording.rec(opBodySetGravityScale, bodyId, gravityScale)
	}
	b := getBodyFullId(w, bodyId)
	getBodySim(w, b).gravityScale = gravityScale
}

// GetGravityScale returns the body's gravity scale. It corresponds to
// b2Body_GetGravityScale.
func (bodyId BodyId) GetGravityScale() Q {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return getBodySim(w, b).gravityScale
}

// IsAwake reports whether the body is in the awake set. It corresponds to
// b2Body_IsAwake.
func (bodyId BodyId) IsAwake() bool {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return b.setIndex == awakeSet
}

// SetAwake wakes or sleeps the body's island. It corresponds to
// b2Body_SetAwake.
func (bodyId BodyId) SetAwake(awake bool) {
	w := getWorldLocked(bodyId.world0)
	if w.recording != nil {
		w.recording.rec(opBodySetAwake, bodyId, awake)
	}
	b := getBodyFullId(w, bodyId)
	if awake && b.setIndex >= firstSleepingSet {
		wakeBody(w, b)
	} else if !awake && b.setIndex == awakeSet {
		isl := &w.islands[b.islandId]
		if isl.constraintRemoveCount > 0 {
			splitIsland(w, b.islandId)
		}
		trySleepIsland(w, b.islandId)
	}
}

// EnableSleeping enables or disables sleeping. It corresponds to
// b2Body_EnableSleeping.
func (bodyId BodyId) EnableSleeping(enableSleep bool) {
	w := getWorldLocked(bodyId.world0)
	if w.recording != nil {
		w.recording.rec(opBodyEnableSleeping, bodyId, enableSleep)
	}
	b := getBodyFullId(w, bodyId)
	if enableSleep == (b.flags&bodyEnableSleep != 0) {
		return
	}
	if enableSleep {
		b.flags |= bodyEnableSleep
	} else {
		b.flags &^= bodyEnableSleep
	}
	syncBodyFlags(w, b)
	if !enableSleep {
		wakeBody(w, b)
	}
}

// EnableContactRecycling lets the contacts of the body keep their manifold
// while the bodies barely move. A contact recycles only when both bodies
// enable it, and only contacts created after this call see the change. It
// corresponds to b2Body_EnableContactRecycling.
func (bodyId BodyId) EnableContactRecycling(flag bool) {
	w := getWorldLocked(bodyId.world0)
	if w.recording != nil {
		w.recording.rec(opBodyEnableContactRecycling, bodyId, flag)
	}
	b := getBodyFullId(w, bodyId)
	if flag == (b.flags&bodyEnableContactRecycling != 0) {
		return
	}
	b.flags &^= bodyEnableContactRecycling
	if flag {
		b.flags |= bodyEnableContactRecycling
	}
	syncBodyFlags(w, b)
}

// IsContactRecyclingEnabled reports whether the contacts of the body may
// recycle. It corresponds to b2Body_IsContactRecyclingEnabled.
func (bodyId BodyId) IsContactRecyclingEnabled() bool {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return b.flags&bodyEnableContactRecycling != 0
}

// IsSleepingEnabled reports whether the body may sleep. It corresponds to
// b2Body_IsSleepingEnabled.
func (bodyId BodyId) IsSleepingEnabled() bool {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return b.flags&bodyEnableSleep != 0
}

// SetSafetyFactor changes the continuous collision safety factor. Smaller is
// safer but can lead to hitching. It corresponds to b2Body_SetSafetyFactor.
func (bodyId BodyId) SetSafetyFactor(safetyFactor Q) {
	if !IsValidQ(safetyFactor) || safetyFactor.Less(QZero()) {
		panic("dbox2d: SetSafetyFactor safetyFactor is not valid")
	}
	w := getWorld(bodyId.world0)
	if w.recording != nil {
		w.recording.rec(opBodySetSafetyFactor, bodyId, safetyFactor)
	}
	getBodyFullId(w, bodyId).safetyFactor = safetyFactor
}

// GetSafetyFactor returns the continuous collision safety factor. It
// corresponds to b2Body_GetSafetyFactor.
func (bodyId BodyId) GetSafetyFactor() Q {
	w := getWorld(bodyId.world0)
	return getBodyFullId(w, bodyId).safetyFactor
}

// WakeTouching wakes the bodies touching this body. It works for static
// bodies. It corresponds to b2Body_WakeTouching.
func (bodyId BodyId) WakeTouching() {
	w := getWorldLocked(bodyId.world0)
	if w.recording != nil {
		w.recording.rec(opBodyWakeTouching, bodyId)
	}
	b := getBodyFullId(w, bodyId)

	contactKey := b.headContactKey
	for contactKey != nullIndex {
		contactId := contactKey >> 1
		edgeIndex := contactKey & 1

		c := &w.contacts[contactId]
		shapeA := &w.shapes[c.shapeIdA]
		shapeB := &w.shapes[c.shapeIdB]

		if shapeA.bodyId == b.id {
			wakeBody(w, &w.bodies[shapeB.bodyId])
		} else {
			wakeBody(w, &w.bodies[shapeA.bodyId])
		}

		contactKey = c.edges[edgeIndex].nextKey
	}
}

// SetSleepThreshold changes the body's sleep threshold. It corresponds to
// b2Body_SetSleepThreshold.
func (bodyId BodyId) SetSleepThreshold(sleepThreshold Q) {
	w := getWorld(bodyId.world0)
	if w.recording != nil {
		w.recording.rec(opBodySetSleepThreshold, bodyId, sleepThreshold)
	}
	b := getBodyFullId(w, bodyId)
	b.sleepThreshold = sleepThreshold
}

// GetSleepThreshold returns the body's sleep threshold. It corresponds to
// b2Body_GetSleepThreshold.
func (bodyId BodyId) GetSleepThreshold() Q {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return b.sleepThreshold
}

// IsEnabled reports whether the body is enabled. It corresponds to
// b2Body_IsEnabled.
func (bodyId BodyId) IsEnabled() bool {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return b.setIndex != disabledSet
}

// Disable removes the body from simulation. It corresponds to b2Body_Disable.
func (bodyId BodyId) Disable() {
	w := getWorld(bodyId.world0)
	if w.locked {
		panic("dbox2d: the world is locked")
	}
	if w.recording != nil {
		w.recording.rec(opBodyDisable, bodyId)
	}
	b := getBodyFullId(w, bodyId)
	if b.setIndex == disabledSet {
		return
	}

	// Destroy contacts and wake bodies touching this body. This avoid floating bodies.
	// This is necessary even for static bodies.
	destroyBodyContacts(w, b, true)
	removeBodyFromIsland(w, b)

	for shapeId := b.headShapeId; shapeId != nullIndex; {
		s := &w.shapes[shapeId]
		shapeId = s.nextShapeId
		destroyShapeProxy(s, &w.broadPhase)
	}

	set := &w.solverSets[b.setIndex]
	disabledSolverSet := &w.solverSets[disabledSet]
	transferBody(w, disabledSolverSet, set, b)

	for jointKey := b.headJointKey; jointKey != nullIndex; {
		jointId := jointKey >> 1
		edgeIndex := jointKey & 1
		j := &w.joints[jointId]
		jointKey = j.edges[edgeIndex].nextKey

		if j.setIndex == disabledSet {
			continue
		}
		if j.setIndex != set.setIndex && set.setIndex != staticSet {
			panic("dbox2d: a joint of the body is in an unexpected set")
		}
		if j.islandId != nullIndex {
			unlinkJoint(w, j)
		}

		jointSet := &w.solverSets[j.setIndex]
		transferJoint(w, disabledSolverSet, jointSet, j)
	}
}

// Enable returns the body to simulation. It corresponds to b2Body_Enable.
func (bodyId BodyId) Enable() {
	w := getWorld(bodyId.world0)
	if w.locked {
		panic("dbox2d: the world is locked")
	}
	if w.recording != nil {
		w.recording.rec(opBodyEnable, bodyId)
	}
	b := getBodyFullId(w, bodyId)
	if b.setIndex != disabledSet {
		return
	}

	disabledSolverSet := &w.solverSets[disabledSet]
	setId := awakeSet
	if b.bodyType == StaticBody {
		setId = staticSet
	}
	targetSet := &w.solverSets[setId]
	transferBody(w, targetSet, disabledSolverSet, b)

	transform := getBodyTransformQuick(w, b)
	proxyType := b.bodyType
	forcePairCreation := true
	for shapeId := b.headShapeId; shapeId != nullIndex; {
		s := &w.shapes[shapeId]
		shapeId = s.nextShapeId
		createShapeProxy(w, s, proxyType, transform, forcePairCreation)
	}

	if setId != staticSet {
		createIslandForBody(w, setId, b)
	}

	for jointKey := b.headJointKey; jointKey != nullIndex; {
		jointId := jointKey >> 1
		edgeIndex := jointKey & 1
		j := &w.joints[jointId]
		if j.setIndex != disabledSet {
			panic("dbox2d: a joint of a disabled body is not disabled")
		}
		if j.islandId != nullIndex {
			panic("dbox2d: a disabled joint belongs to an island")
		}

		jointKey = j.edges[edgeIndex].nextKey
		bodyA := &w.bodies[j.edges[0].bodyId]
		bodyB := &w.bodies[j.edges[1].bodyId]
		if bodyA.setIndex == disabledSet || bodyB.setIndex == disabledSet {
			continue
		}

		var jointSetId int
		if bodyA.bodyType != DynamicBody && bodyB.bodyType != DynamicBody {
			jointSetId = staticSet
		} else if bodyA.setIndex == staticSet {
			jointSetId = bodyB.setIndex
		} else {
			jointSetId = bodyA.setIndex
		}

		jointSet := &w.solverSets[jointSetId]
		transferJoint(w, jointSet, disabledSolverSet, j)
		if jointSetId != staticSet {
			linkJoint(w, j)
		}
	}
}

// SetMotionLocks restricts the body movement. A lock zeroes the matching
// velocity at the end of each sub-step; the angular lock also zeroes the
// rotational inertia. It corresponds to b2Body_SetMotionLocks.
func (bodyId BodyId) SetMotionLocks(locks MotionLocks) {
	w := getWorldLocked(bodyId.world0)
	if w.recording != nil {
		w.recording.rec(opBodySetMotionLocks, bodyId, locks)
	}
	newFlags := motionLockFlags(locks)

	b := getBodyFullId(w, bodyId)
	if b.flags&bodyAllLocks != newFlags {
		b.flags &^= bodyAllLocks
		b.flags |= newFlags

		syncBodyFlags(w, b)

		if state := getBodyState(w, b); state != nil {
			if locks.LinearX {
				state.linearVelocity.X = QZero()
			}
			if locks.LinearY {
				state.linearVelocity.Y = QZero()
			}
			if locks.AngularZ {
				state.angularVelocity = QZero()
			}
		}

		// Motion locks can affect mass properties.
		updateBodyMassData(w, b)
	}
}

// GetMotionLocks returns the motion locks. It corresponds to
// b2Body_GetMotionLocks.
func (bodyId BodyId) GetMotionLocks() MotionLocks {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return MotionLocks{
		LinearX:  b.flags&bodyLockLinearX != 0,
		LinearY:  b.flags&bodyLockLinearY != 0,
		AngularZ: b.flags&bodyLockAngularZ != 0,
	}
}

// SetBullet changes whether the body is treated as a bullet. It corresponds
// to b2Body_SetBullet.
func (bodyId BodyId) SetBullet(flag bool) {
	w := getWorldLocked(bodyId.world0)
	if w.recording != nil {
		w.recording.rec(opBodySetBullet, bodyId, flag)
	}
	b := getBodyFullId(w, bodyId)
	var newFlag uint32
	if flag {
		newFlag = bodyIsBullet
	}
	if b.flags&bodyIsBullet == newFlag {
		return
	}
	b.flags &^= bodyIsBullet
	b.flags |= newFlag
	syncBodyFlags(w, b)
}

// IsBullet reports whether the body is treated as a bullet. It corresponds
// to b2Body_IsBullet.
func (bodyId BodyId) IsBullet() bool {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return getBodySim(w, b).flags&bodyIsBullet != 0
}

// EnableContactEvents enables contact events on every body shape. It
// corresponds to b2Body_EnableContactEvents.
func (bodyId BodyId) EnableContactEvents(flag bool) {
	w := getWorld(bodyId.world0)
	if w.recording != nil {
		w.recording.rec(opBodyEnableContactEvents, bodyId, flag)
	}
	b := getBodyFullId(w, bodyId)
	for shapeId := b.headShapeId; shapeId != nullIndex; shapeId = w.shapes[shapeId].nextShapeId {
		w.shapes[shapeId].enableContactEvents = flag
	}
}

// EnableHitEvents enables hit events on every body shape. It corresponds to
// b2Body_EnableHitEvents.
func (bodyId BodyId) EnableHitEvents(flag bool) {
	w := getWorld(bodyId.world0)
	if w.recording != nil {
		w.recording.rec(opBodyEnableHitEvents, bodyId, flag)
	}
	b := getBodyFullId(w, bodyId)
	for shapeId := b.headShapeId; shapeId != nullIndex; shapeId = w.shapes[shapeId].nextShapeId {
		w.shapes[shapeId].enableHitEvents = flag
	}
}

// GetWorld returns the world containing the body. It corresponds to
// b2Body_GetWorld.
func (bodyId BodyId) GetWorld() WorldId {
	w := getWorld(bodyId.world0)
	return WorldId{index1: bodyId.world0 + 1, generation: w.generation}
}

// GetShapeCount returns the number of shapes on the body. It corresponds to
// b2Body_GetShapeCount.
func (bodyId BodyId) GetShapeCount() int {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return b.shapeCount
}

// GetShapes fills shapes in body-list order. It corresponds to
// b2Body_GetShapes.
func (bodyId BodyId) GetShapes(shapes []ShapeId) int {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	shapeId := b.headShapeId
	count := 0
	for shapeId != nullIndex && count < len(shapes) {
		s := &w.shapes[shapeId]
		shapes[count] = ShapeId{index1: int32(s.id) + 1, world0: bodyId.world0, generation: s.generation}
		count++
		shapeId = s.nextShapeId
	}
	return count
}

// GetJointCount returns the number of joints on the body. It corresponds to
// b2Body_GetJointCount.
func (bodyId BodyId) GetJointCount() int {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return b.jointCount
}

// GetJoints fills joints in body-list order. It corresponds to
// b2Body_GetJoints.
func (bodyId BodyId) GetJoints(joints []JointId) int {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	jointKey := b.headJointKey
	count := 0
	for jointKey != nullIndex && count < len(joints) {
		jointId := jointKey >> 1
		edgeIndex := jointKey & 1
		j := &w.joints[jointId]
		joints[count] = JointId{index1: int32(jointId) + 1, world0: bodyId.world0, generation: j.generation}
		count++
		jointKey = j.edges[edgeIndex].nextKey
	}
	return count
}

// GetContactCapacity returns the body's conservative contact capacity. It
// corresponds to b2Body_GetContactCapacity.
func (bodyId BodyId) GetContactCapacity() int {
	w := getWorld(bodyId.world0)
	if w.locked {
		return 0
	}
	b := getBodyFullId(w, bodyId)
	return b.contactCount
}

// GetContactData fills touching contact data in body-list order. It
// corresponds to b2Body_GetContactData.
func (bodyId BodyId) GetContactData(data []ContactData) int {
	w := getWorld(bodyId.world0)
	if w.locked {
		return 0
	}
	b := getBodyFullId(w, bodyId)
	contactKey := b.headContactKey
	count := 0
	for contactKey != nullIndex && count < len(data) {
		contactId := contactKey >> 1
		edgeIndex := contactKey & 1
		c := &w.contacts[contactId]
		if c.flags&contactTouchingFlag != 0 {
			sA := &w.shapes[c.shapeIdA]
			sB := &w.shapes[c.shapeIdB]
			data[count].ContactId = contactIdOf(w, c)
			data[count].ShapeIdA = ShapeId{index1: int32(sA.id) + 1, world0: bodyId.world0, generation: sA.generation}
			data[count].ShapeIdB = ShapeId{index1: int32(sB.id) + 1, world0: bodyId.world0, generation: sB.generation}
			data[count].Manifold = getContactSim(w, c).manifold
			count++
		}
		contactKey = c.edges[edgeIndex].nextKey
	}
	return count
}

// ComputeBounds returns the union of the body's shape bounds. It corresponds
// to b2Body_ComputeBounds.
func (bodyId BodyId) ComputeBounds() AABB {
	w := getWorld(bodyId.world0)
	if w.locked {
		return AABB{}
	}
	b := getBodyFullId(w, bodyId)
	if b.headShapeId == nullIndex {
		transform := getBodyTransform(w, b.id)
		// No shapes: bracket the body origin so the box still contains the
		// true position far away.
		return posBoxPoint(transform.P)
	}
	aabb := w.shapes[b.headShapeId].aabb
	for shapeId := w.shapes[b.headShapeId].nextShapeId; shapeId != nullIndex; shapeId = w.shapes[shapeId].nextShapeId {
		aabb = AABBUnion(aabb, w.shapes[shapeId].aabb)
	}
	return aabb
}

// SetUserData attaches application data to the body. It corresponds to
// b2Body_SetUserData.
func (bodyId BodyId) SetUserData(userData any) {
	w := getWorld(bodyId.world0)
	getBodyFullId(w, bodyId).userData = userData
}

// GetUserData returns the data attached to the body. It corresponds to
// b2Body_GetUserData.
func (bodyId BodyId) GetUserData() any {
	w := getWorld(bodyId.world0)
	return getBodyFullId(w, bodyId).userData
}

// SetName stores up to 10 bytes of the body name. It corresponds to
// b2Body_SetName.
func (bodyId BodyId) SetName(name string) {
	w := getWorld(bodyId.world0)
	if w.recording != nil {
		w.recording.rec(opBodySetName, bodyId, name)
	}
	b := getBodyFullId(w, bodyId)
	b.name = [nameLength + 1]byte{}
	copy(b.name[:nameLength], name)
}

// GetName returns the stored body name. It corresponds to b2Body_GetName.
func (bodyId BodyId) GetName() string {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	for i, value := range b.name {
		if value == 0 {
			return string(b.name[:i])
		}
	}
	return string(b.name[:])
}

// wakeBody wakes the sleeping set of a body. It reports whether a set
// woke. It corresponds to b2WakeBody in src/body.c.
func wakeBody(w *world, b *body) bool {
	if b.setIndex >= firstSleepingSet {
		wakeSolverSet(w, b.setIndex)
		return true
	}

	return false
}

// destroyBodyContacts destroys every contact of a body. It corresponds to
// b2DestroyBodyContacts in src/body.c.
func destroyBodyContacts(w *world, b *body, wakeBodies bool) {
	// Destroy the attached contacts
	edgeKey := b.headContactKey
	for edgeKey != nullIndex {
		contactId := edgeKey >> 1
		edgeIndex := edgeKey & 1

		c := &w.contacts[contactId]
		edgeKey = c.edges[edgeIndex].nextKey
		destroyContact(w, c, wakeBodies)
	}
}

// createIslandForBody gives an enabled body its own island. It corresponds
// to b2CreateIslandForBody in src/body.c.
func createIslandForBody(w *world, setIndex int, b *body) {
	if b.islandId != nullIndex {
		panic("dbox2d: the body is already in an island")
	}
	if setIndex == disabledSet {
		panic("dbox2d: a disabled body has no island")
	}

	isl := createIsland(w, setIndex)
	isl.bodies = append(isl.bodies, b.id)
	b.islandId = isl.islandId
	b.islandIndex = 0
}

// removeBodyFromIsland removes a body from its island and destroys the
// island when it becomes empty. It corresponds to b2RemoveBodyFromIsland in
// src/body.c.
func removeBodyFromIsland(w *world, b *body) {
	if b.islandId == nullIndex {
		if b.islandIndex != nullIndex {
			panic("dbox2d: a body without an island has an island index")
		}
		return
	}

	islandId := b.islandId
	isl := &w.islands[islandId]
	{
		localIndex := b.islandIndex
		last := len(isl.bodies) - 1
		movedBodyId := isl.bodies[last]
		isl.bodies[localIndex] = movedBodyId
		if w.bodies[movedBodyId].islandIndex != last {
			panic("dbox2d: the moved body island index does not match")
		}
		w.bodies[movedBodyId].islandIndex = localIndex
		isl.bodies = isl.bodies[:last]
	}

	if len(isl.bodies) == 0 {
		// Destroy empty island
		if len(isl.contacts) != 0 || len(isl.joints) != 0 {
			panic("dbox2d: the emptied island still lists constraints")
		}

		// Free the island
		destroyIsland(w, isl.islandId)
	}

	b.islandId = nullIndex
	b.islandIndex = nullIndex
}

// shouldBodiesCollide rejects a pair of non-dynamic bodies. It corresponds
// to b2ShouldBodiesCollide in src/body.c.
func shouldBodiesCollide(w *world, bodyA, bodyB *body) bool {
	if bodyA.bodyType != DynamicBody && bodyB.bodyType != DynamicBody {
		return false
	}

	var jointKey int
	var otherBodyId int
	if bodyA.jointCount < bodyB.jointCount {
		jointKey = bodyA.headJointKey
		otherBodyId = bodyB.id
	} else {
		jointKey = bodyB.headJointKey
		otherBodyId = bodyA.id
	}

	for jointKey != nullIndex {
		jointId := jointKey >> 1
		edgeIndex := jointKey & 1
		otherEdgeIndex := edgeIndex ^ 1

		j := &w.joints[jointId]
		if !j.collideConnected && j.edges[otherEdgeIndex].bodyId == otherBodyId {
			return false
		}

		jointKey = j.edges[edgeIndex].nextKey
	}

	return true
}

// makeRelativeSweep builds the sweep of a body sim relative to a base
// position, so the time of impact stays near the origin. It corresponds to
// b2MakeRelativeSweep in src/body.h.
func makeRelativeSweep(sim *bodySim, base Pos) Sweep {
	return Sweep{
		LocalCenter: sim.localCenter,
		C1:          SubPos(sim.center0, base),
		C2:          SubPos(sim.center, base),
		Q1:          sim.rotation0,
		Q2:          sim.transform.Q,
	}
}
