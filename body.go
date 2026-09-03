package dbox2d

import "github.com/dhannyell/fixed"

// body holds the organizational details that the solver does not use.
type body struct {
	name [32]byte

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
	islandId   int
	islandPrev int
	islandNext int

	mass Q

	// inertia is the rotational inertia about the center of mass.
	inertia Q

	sleepThreshold Q
	sleepTime      Q

	// bodyMoveIndex adjusts the fellAsleep flag in the body move array.
	bodyMoveIndex int

	id int

	// bodyType is the upstream field named type, which Go reserves.
	bodyType BodyType

	// generation advances on each allocation of this slot, so a stale
	// BodyId fails validation.
	generation uint16

	enableSleep   bool
	fixedRotation bool
	isSpeedCapped bool
	isMarked      bool
}

// bodyState is the solver view of a body. Only awake dynamic and kinematic
// bodies have one. The angular velocity is in turns per second.
type bodyState struct {
	linearVelocity  Vec2
	angularVelocity Q
	flags           int

	// deltaPosition reduces round-off error far from the origin.
	deltaPosition Vec2

	// deltaRotation stays identity for a static body, which has no state
	// that the solver can write.
	deltaRotation Rot
}

// identityBodyState returns the state at rest. The delta rotation is the
// identity, (1, 0).
func identityBodyState() bodyState {
	return bodyState{deltaRotation: fixed.RotIdentity()}
}

// bodySim is the data that integrates position and velocity. The transform
// serves collision and solver preparation.
type bodySim struct {
	// transform of the body origin
	transform Transform

	// center of mass in world space
	center Vec2

	// rotation0 and center0 are the previous rotation and center of mass,
	// for time of impact.
	rotation0 Rot
	center0   Vec2

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

	// isFast serves the debug draw.
	isFast bool

	isBullet          bool
	isSpeedCapped     bool
	allowFastRotation bool
	enlargeAABB       bool
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
func getBodyTransformQuick(w *world, b *body) Transform {
	set := &w.solverSets[b.setIndex]
	return set.bodySims[b.localIndex].transform
}

// getBodyTransform returns the transform of a body by raw id. It corresponds
// to b2GetBodyTransform in src/body.c.
func getBodyTransform(w *world, bodyId int) Transform {
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
	if !IsValidVec2(def.Position) {
		panic("dbox2d: BodyDef.Position is not valid")
	}
	if !IsValidRotation(def.Rotation) {
		panic("dbox2d: BodyDef.Rotation is not valid")
	}
	if !IsValidVec2(def.LinearVelocity) {
		panic("dbox2d: BodyDef.LinearVelocity is not valid")
	}
	if !IsValidQ(def.AngularVelocity) {
		panic("dbox2d: BodyDef.AngularVelocity is not valid")
	}
	zero := fixed.Q32Zero()
	if !IsValidQ(def.LinearDamping) || def.LinearDamping.Less(zero) {
		panic("dbox2d: BodyDef.LinearDamping is not valid")
	}
	if !IsValidQ(def.AngularDamping) || def.AngularDamping.Less(zero) {
		panic("dbox2d: BodyDef.AngularDamping is not valid")
	}
	if !IsValidQ(def.SleepThreshold) || def.SleepThreshold.Less(zero) {
		panic("dbox2d: BodyDef.SleepThreshold is not valid")
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

	set := &w.solverSets[setId]
	set.bodySims = append(set.bodySims, bodySim{
		transform:         Transform{P: def.Position, Q: def.Rotation},
		center:            def.Position,
		rotation0:         def.Rotation,
		center0:           def.Position,
		minExtent:         huge,
		maxExtent:         fixed.Q32Zero(),
		linearDamping:     def.LinearDamping,
		angularDamping:    def.AngularDamping,
		gravityScale:      def.GravityScale,
		bodyId:            bodyId,
		isBullet:          def.IsBullet,
		allowFastRotation: def.AllowFastRotation,
	})

	if setId == awakeSet {
		state := identityBodyState()
		state.linearVelocity = def.LinearVelocity
		state.angularVelocity = def.AngularVelocity
		set.bodyStates = append(set.bodyStates, state)
	}

	if bodyId == len(w.bodies) {
		w.bodies = append(w.bodies, body{})
	} else if w.bodies[bodyId].id != nullIndex {
		panic("dbox2d: the body slot is still in use")
	}

	b := &w.bodies[bodyId]

	// the name keeps at most 31 bytes, as the reference does
	b.name = [32]byte{}
	copy(b.name[:31], def.Name)

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
	b.islandPrev = nullIndex
	b.islandNext = nullIndex
	b.bodyMoveIndex = nullIndex
	b.id = bodyId
	b.mass = fixed.Q32Zero()
	b.inertia = fixed.Q32Zero()
	b.sleepThreshold = def.SleepThreshold
	b.sleepTime = fixed.Q32Zero()
	b.bodyType = def.Type
	b.enableSleep = def.EnableSleep
	b.fixedRotation = def.FixedRotation
	b.isSpeedCapped = false
	b.isMarked = false

	// dynamic and kinematic bodies that are enabled need a island
	if setId >= awakeSet {
		createIslandForBody(w, setId, b)
	}

	return BodyId{index1: int32(bodyId) + 1, world0: w.worldId, generation: b.generation}
}

// DestroyBody destroys a body. Every shape attached to it goes away too.
func DestroyBody(bodyId BodyId) {
	w := getWorldLocked(bodyId.world0)

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
		destroyJointInternal(w, j, wakeBodies)
	}

	// Destroy all contacts attached to this body.
	destroyBodyContacts(w, b, wakeBodies)

	// Destroy the attached shapes. Chain segments are destroyed with their
	// owning chain below.
	shapeId := b.headShapeId
	for shapeId != nullIndex {
		s := &w.shapes[shapeId]

		if s.chainId == nullIndex {
			destroyShapeProxy(s, &w.broadPhase)

			// Return shape to free list.
			w.shapeIdPool.freeId(shapeId)
			s.id = nullIndex
		}

		shapeId = s.nextShapeId
	}

	for chainID := b.headChainId; chainID != nullIndex; {
		chain := &w.chainShapes[chainID]
		nextChainID := chain.nextChainId
		DestroyChain(ChainId{index1: int32(chainID) + 1, world0: w.worldId, generation: chain.generation})
		chainID = nextChainID
	}

	removeBodyFromIsland(w, b)

	// Remove body sim from solver set that owns it
	set := &w.solverSets[b.setIndex]
	var movedIndex int
	set.bodySims, movedIndex = removeSwap(set.bodySims, b.localIndex)
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
		set.bodyStates, result = removeSwap(set.bodyStates, b.localIndex)
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
	zero := fixed.Q32Zero()
	b.mass = zero
	b.inertia = zero

	sim.invMass = zero
	sim.invInertia = zero
	sim.localCenter = Vec2Zero()
	sim.minExtent = huge
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
		b.inertia = b.inertia.Add(massData.RotationalInertia)
	}

	// Compute center of mass.
	if zero.Less(b.mass) {
		sim.invMass = fixed.Q32One().Div(b.mass)
		localCenter = localCenter.Mul(sim.invMass)
	}

	if zero.Less(b.inertia) && !b.fixedRotation {
		// Center the inertia about the center of mass.
		b.inertia = b.inertia.Sub(b.mass.Mul(localCenter.Dot(localCenter)))
		if !zero.Less(b.inertia) {
			panic("dbox2d: the centered inertia is not positive")
		}
		sim.invInertia = fixed.Q32One().Div(b.inertia)
	} else {
		b.inertia = zero
		sim.invInertia = zero
	}

	// Move center of mass.
	oldCenter := sim.center
	sim.localCenter = localCenter
	sim.center = TransformPoint(sim.transform, sim.localCenter)
	sim.center0 = sim.center

	// Update center of mass velocity. The state stores turns per second and
	// the cross product needs radians per second, so the velocity scales by
	// one turn.
	state := getBodyState(w, b)
	if state != nil {
		deltaLinear := CrossSV(tau.Mul(state.angularVelocity), sim.center.Sub(oldCenter))
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
	b := getBodyFullId(w, bodyId)

	originalType := b.bodyType
	if originalType == bodyType {
		return
	}

	if b.setIndex == disabledSet {
		b.bodyType = bodyType
		updateBodyMassData(w, b)
		return
	}

	destroyBodyContacts(w, b, false)
	wakeBody(w, b)

	for jointKey := b.headJointKey; jointKey != nullIndex; {
		jointId := jointKey >> 1
		edgeIndex := jointKey & 1
		j := &w.joints[jointId]
		if j.islandId != nullIndex {
			unlinkJoint(w, j)
		}

		bodyA := &w.bodies[j.edges[0].bodyId]
		bodyB := &w.bodies[j.edges[1].bodyId]
		wakeBody(w, bodyA)
		wakeBody(w, bodyB)

		jointKey = j.edges[edgeIndex].nextKey
	}

	b.bodyType = bodyType

	if originalType == StaticBody {
		if b.setIndex != staticSet {
			panic("dbox2d: a static body is not in the static set")
		}

		staticSolverSet := &w.solverSets[staticSet]
		awakeSolverSet := &w.solverSets[awakeSet]
		transferBody(w, awakeSolverSet, staticSolverSet, b)
		createIslandForBody(w, awakeSet, b)

		for jointKey := b.headJointKey; jointKey != nullIndex; {
			jointId := jointKey >> 1
			edgeIndex := jointKey & 1
			j := &w.joints[jointId]

			if j.setIndex == staticSet {
				transferJoint(w, awakeSolverSet, staticSolverSet, j)
			} else if j.setIndex == awakeSet {
				transferJoint(w, staticSolverSet, awakeSolverSet, j)
				transferJoint(w, awakeSolverSet, staticSolverSet, j)
			} else if j.setIndex != disabledSet {
				panic("dbox2d: a joint of a static body is in an unexpected set")
			}

			jointKey = j.edges[edgeIndex].nextKey
		}

		transform := getBodyTransformQuick(w, b)
		for shapeId := b.headShapeId; shapeId != nullIndex; {
			s := &w.shapes[shapeId]
			shapeId = s.nextShapeId
			destroyShapeProxy(s, &w.broadPhase)
			createShapeProxy(s, &w.broadPhase, bodyType, transform, true)
		}
	} else if bodyType == StaticBody {
		if b.setIndex != awakeSet {
			panic("dbox2d: a moving body is not in the awake set")
		}

		staticSolverSet := &w.solverSets[staticSet]
		awakeSolverSet := &w.solverSets[awakeSet]
		transferBody(w, staticSolverSet, awakeSolverSet, b)
		removeBodyFromIsland(w, b)

		sim := &staticSolverSet.bodySims[b.localIndex]
		sim.isFast = false

		for jointKey := b.headJointKey; jointKey != nullIndex; {
			jointId := jointKey >> 1
			edgeIndex := jointKey & 1
			j := &w.joints[jointId]
			jointKey = j.edges[edgeIndex].nextKey

			otherBody := &w.bodies[j.edges[edgeIndex^1].bodyId]
			if j.setIndex == disabledSet {
				if otherBody.setIndex != disabledSet {
					panic("dbox2d: a disabled joint is not connected to a disabled body")
				}
				continue
			}
			if j.setIndex != awakeSet {
				panic("dbox2d: a joint of a moving body is not awake")
			}

			if otherBody.setIndex == staticSet {
				transferJoint(w, staticSolverSet, awakeSolverSet, j)
			} else {
				if otherBody.setIndex != awakeSet {
					panic("dbox2d: the other joint body is not awake")
				}
				if j.colorIndex < 0 || j.colorIndex >= graphColorCount {
					panic("dbox2d: the joint color index is out of range")
				}
				transferJoint(w, staticSolverSet, awakeSolverSet, j)
				transferJoint(w, awakeSolverSet, staticSolverSet, j)
			}
		}

		transform := getBodyTransformQuick(w, b)
		for shapeId := b.headShapeId; shapeId != nullIndex; {
			s := &w.shapes[shapeId]
			shapeId = s.nextShapeId
			destroyShapeProxy(s, &w.broadPhase)
			createShapeProxy(s, &w.broadPhase, StaticBody, transform, true)
		}
	} else {
		if originalType != DynamicBody && originalType != KinematicBody {
			panic("dbox2d: the original body type is not movable")
		}
		if bodyType != DynamicBody && bodyType != KinematicBody {
			panic("dbox2d: the new body type is not movable")
		}

		transform := getBodyTransformQuick(w, b)
		for shapeId := b.headShapeId; shapeId != nullIndex; {
			s := &w.shapes[shapeId]
			shapeId = s.nextShapeId
			destroyShapeProxy(s, &w.broadPhase)
			createShapeProxy(s, &w.broadPhase, bodyType, transform, true)
		}
	}

	for jointKey := b.headJointKey; jointKey != nullIndex; {
		jointId := jointKey >> 1
		edgeIndex := jointKey & 1
		j := &w.joints[jointId]
		jointKey = j.edges[edgeIndex].nextKey

		otherBody := &w.bodies[j.edges[edgeIndex^1].bodyId]
		if otherBody.setIndex == disabledSet {
			continue
		}
		if b.bodyType == StaticBody && otherBody.bodyType == StaticBody {
			continue
		}
		linkJoint(w, j, false)
	}
	mergeAwakeIslands(w)

	updateBodyMassData(w, b)
}

// GetPosition returns the world position of the body origin. It corresponds
// to b2Body_GetPosition.
func (bodyId BodyId) GetPosition() Vec2 {
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
func (bodyId BodyId) GetTransform() Transform {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return getBodyTransformQuick(w, b)
}

// SetTransform changes the body's world transform. It corresponds to
// b2Body_SetTransform.
func (bodyId BodyId) SetTransform(position Vec2, rotation Rot) {
	if !IsValidVec2(position) {
		panic("dbox2d: SetTransform position is not valid")
	}
	if !IsValidRotation(rotation) {
		panic("dbox2d: SetTransform rotation is not valid")
	}
	w := getWorld(bodyId.world0)
	if w.locked {
		panic("dbox2d: the world is locked")
	}
	b := getBodyFullId(w, bodyId)
	sim := getBodySim(w, b)

	sim.transform = Transform{P: position, Q: rotation}
	sim.center = TransformPoint(sim.transform, sim.localCenter)
	sim.rotation0 = sim.transform.Q
	sim.center0 = sim.center

	for shapeId := b.headShapeId; shapeId != nullIndex; shapeId = w.shapes[shapeId].nextShapeId {
		s := &w.shapes[shapeId]
		fatAABB := s.fatAABB
		updateShapeAABBs(s, sim.transform, b.bodyType)
		if AABBContains(fatAABB, s.aabb) {
			s.fatAABB = fatAABB
			continue
		}
		if s.proxyKey != nullIndex {
			w.broadPhase.moveProxy(s.proxyKey, s.fatAABB)
		}
	}
}

// SetTargetTransform sets the velocity needed to reach a target transform.
// It corresponds to b2Body_SetTargetTransform.
func (bodyId BodyId) SetTargetTransform(target Transform, timeStep Q) {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	zero := fixed.Q32Zero()
	if b.bodyType == StaticBody || !zero.Less(timeStep) {
		return
	}

	sim := getBodySim(w, b)
	// D-006: the reciprocal is an exact fixed-point division.
	invTimeStep := fixed.Q32One().Div(timeStep)
	center1 := sim.center
	center2 := TransformPoint(target, sim.localCenter)
	linearVelocity := center2.Sub(center1).Mul(invTimeStep)

	angularVelocity := zero
	if !b.fixedRotation {
		deltaAngle := RelativeAngle(target.Q, sim.transform.Q)
		// D-004: RelativeAngle already returns turns.
		angularVelocity = deltaAngle.Mul(invTimeStep)
	}

	maxVelocity := linearVelocity.Len().Add(tau.Mul(angularVelocity.Abs()).Mul(sim.maxExtent))
	if maxVelocity.Less(b.sleepThreshold) {
		return
	}

	wakeBody(w, b)
	state := getBodyState(w, b)
	if state == nil {
		return
	}
	state.linearVelocity = linearVelocity
	state.angularVelocity = angularVelocity
	limitVelocity(state, w.maxLinearSpeed)
}

// GetLocalPoint converts a world point to the body's local frame. It
// corresponds to b2Body_GetLocalPoint.
func (bodyId BodyId) GetLocalPoint(worldPoint Vec2) Vec2 {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return InvTransformPoint(getBodyTransformQuick(w, b), worldPoint)
}

// GetWorldPoint converts a local point to the world frame. It corresponds to
// b2Body_GetWorldPoint.
func (bodyId BodyId) GetWorldPoint(localPoint Vec2) Vec2 {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return TransformPoint(getBodyTransformQuick(w, b), localPoint)
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
		// D-004: body angular velocity is stored in turns per second.
		return state.angularVelocity
	}
	return fixed.Q32Zero()
}

// SetLinearVelocity changes the body's linear velocity. It corresponds to
// b2Body_SetLinearVelocity.
func (bodyId BodyId) SetLinearVelocity(linearVelocity Vec2) {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	if b.bodyType == StaticBody {
		return
	}
	if fixed.Q32Zero().Less(linearVelocity.Dot(linearVelocity)) {
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
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	if b.bodyType == StaticBody || b.fixedRotation {
		return
	}
	if !angularVelocity.Eq(fixed.Q32Zero()) {
		wakeBody(w, b)
	}
	state := getBodyState(w, b)
	if state == nil {
		return
	}
	// D-004: body angular velocity is stored in turns per second.
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
	// D-004: convert stored turns per second to radians for the cross product.
	return state.linearVelocity.Add(CrossSV(tau.Mul(state.angularVelocity), r))
}

// GetWorldPointVelocity returns the velocity at a world point. It corresponds
// to b2Body_GetWorldPointVelocity.
func (bodyId BodyId) GetWorldPointVelocity(worldPoint Vec2) Vec2 {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	state := getBodyState(w, b)
	if state == nil {
		return Vec2Zero()
	}
	sim := getBodySim(w, b)
	// D-004: convert stored turns per second to radians for the cross product.
	return state.linearVelocity.Add(CrossSV(tau.Mul(state.angularVelocity), worldPoint.Sub(sim.center)))
}

// ApplyForce applies a force at a world point. It corresponds to
// b2Body_ApplyForce.
func (bodyId BodyId) ApplyForce(force, point Vec2, wake bool) {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	if wake && b.setIndex >= firstSleepingSet {
		wakeBody(w, b)
	}
	if b.setIndex == awakeSet {
		sim := getBodySim(w, b)
		sim.force = sim.force.Add(force)
		sim.torque = sim.torque.Add(Cross(point.Sub(sim.center), force))
	}
}

// ApplyForceToCenter applies a force at the center of mass. It corresponds
// to b2Body_ApplyForceToCenter.
func (bodyId BodyId) ApplyForceToCenter(force Vec2, wake bool) {
	w := getWorld(bodyId.world0)
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
	b := getBodyFullId(w, bodyId)
	if wake && b.setIndex >= firstSleepingSet {
		wakeBody(w, b)
	}
	if b.setIndex == awakeSet {
		// D-004: the solver converts the accumulated torque to turns.
		getBodySim(w, b).torque = getBodySim(w, b).torque.Add(torque)
	}
}

// ApplyLinearImpulse applies a linear impulse at a world point. It
// corresponds to b2Body_ApplyLinearImpulse.
func (bodyId BodyId) ApplyLinearImpulse(impulse, point Vec2, wake bool) {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	if wake && b.setIndex >= firstSleepingSet {
		wakeBody(w, b)
	}
	if b.setIndex == awakeSet {
		state := getBodyState(w, b)
		sim := getBodySim(w, b)
		state.linearVelocity = MulAdd(state.linearVelocity, sim.invMass, impulse)
		// D-004: convert the reference's radian angular impulse to turns.
		state.angularVelocity = state.angularVelocity.Add(sim.invInertia.Mul(Cross(point.Sub(sim.center), impulse)).Div(tau))
		limitVelocity(state, w.maxLinearSpeed)
	}
}

// ApplyLinearImpulseToCenter applies a linear impulse at the center of mass.
// It corresponds to b2Body_ApplyLinearImpulseToCenter.
func (bodyId BodyId) ApplyLinearImpulseToCenter(impulse Vec2, wake bool) {
	w := getWorld(bodyId.world0)
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
	b := getBodyFullId(w, bodyId)
	if wake && b.setIndex >= firstSleepingSet {
		wakeBody(w, b)
	}
	if b.setIndex == awakeSet {
		state := getBodyState(w, b)
		sim := getBodySim(w, b)
		// D-004: convert the reference's radian result to turns per second.
		state.angularVelocity = state.angularVelocity.Add(sim.invInertia.Mul(impulse).Div(tau))
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

// GetLocalCenterOfMass returns the center of mass in local coordinates. It
// corresponds to b2Body_GetLocalCenterOfMass.
func (bodyId BodyId) GetLocalCenterOfMass() Vec2 {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return getBodySim(w, b).localCenter
}

// GetWorldCenterOfMass returns the center of mass in world coordinates. It
// corresponds to b2Body_GetWorldCenterOfMass.
func (bodyId BodyId) GetWorldCenterOfMass() Vec2 {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return getBodySim(w, b).center
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
	w := getWorld(bodyId.world0)
	if w.locked {
		return
	}
	zero := fixed.Q32Zero()
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
	sim.center = TransformPoint(sim.transform, massData.Center)
	sim.center0 = sim.center

	if zero.Less(b.mass) {
		// D-006: the reciprocal is an exact fixed-point division.
		sim.invMass = fixed.Q32One().Div(b.mass)
	} else {
		sim.invMass = zero
	}
	if zero.Less(b.inertia) {
		// D-006: the reciprocal is an exact fixed-point division.
		sim.invInertia = fixed.Q32One().Div(b.inertia)
	} else {
		sim.invInertia = zero
	}
}

// ApplyMassFromShapes recomputes mass data from the body's shapes. It
// corresponds to b2Body_ApplyMassFromShapes.
func (bodyId BodyId) ApplyMassFromShapes() {
	w := getWorld(bodyId.world0)
	if w.locked {
		return
	}
	b := getBodyFullId(w, bodyId)
	updateBodyMassData(w, b)
}

// SetLinearDamping changes linear damping. It corresponds to
// b2Body_SetLinearDamping.
func (bodyId BodyId) SetLinearDamping(linearDamping Q) {
	w := getWorld(bodyId.world0)
	if w.locked {
		return
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
	w := getWorld(bodyId.world0)
	if w.locked {
		return
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
	w := getWorld(bodyId.world0)
	if w.locked {
		return
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
	w := getWorld(bodyId.world0)
	if w.locked {
		return
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

// EnableSleep enables or disables sleeping. It corresponds to
// b2Body_EnableSleep.
func (bodyId BodyId) EnableSleep(enableSleep bool) {
	w := getWorld(bodyId.world0)
	if w.locked {
		return
	}
	b := getBodyFullId(w, bodyId)
	b.enableSleep = enableSleep
	if !enableSleep {
		wakeBody(w, b)
	}
}

// IsSleepEnabled reports whether the body may sleep. It corresponds to
// b2Body_IsSleepEnabled.
func (bodyId BodyId) IsSleepEnabled() bool {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return b.enableSleep
}

// SetSleepThreshold changes the body's sleep threshold. It corresponds to
// b2Body_SetSleepThreshold.
func (bodyId BodyId) SetSleepThreshold(sleepThreshold Q) {
	w := getWorld(bodyId.world0)
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
	b := getBodyFullId(w, bodyId)
	if b.setIndex == disabledSet {
		return
	}

	wakeBody(w, b)
	destroyBodyContacts(w, b, false)
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
		createShapeProxy(s, &w.broadPhase, proxyType, transform, forcePairCreation)
	}

	if setId != staticSet {
		createIslandForBody(w, setId, b)
	}

	mergeIslands := false
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
		if bodyA.setIndex == staticSet && bodyB.setIndex == staticSet {
			jointSetId = staticSet
		} else if bodyA.setIndex == staticSet {
			jointSetId = bodyB.setIndex
		} else {
			jointSetId = bodyA.setIndex
		}

		jointSet := &w.solverSets[jointSetId]
		transferJoint(w, jointSet, disabledSolverSet, j)
		if jointSetId != staticSet {
			linkJoint(w, j, mergeIslands)
		}
	}

	mergeAwakeIslands(w)
}

// SetFixedRotation enables or disables body rotation. It corresponds to
// b2Body_SetFixedRotation.
func (bodyId BodyId) SetFixedRotation(flag bool) {
	w := getWorld(bodyId.world0)
	if w.locked {
		return
	}
	b := getBodyFullId(w, bodyId)
	if b.fixedRotation != flag {
		b.fixedRotation = flag
		if state := getBodyState(w, b); state != nil {
			state.angularVelocity = fixed.Q32Zero()
		}
		updateBodyMassData(w, b)
	}
}

// IsFixedRotation reports whether body rotation is fixed. It corresponds to
// b2Body_IsFixedRotation.
func (bodyId BodyId) IsFixedRotation() bool {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return b.fixedRotation
}

// SetBullet changes whether the body is treated as a bullet. It corresponds
// to b2Body_SetBullet.
func (bodyId BodyId) SetBullet(flag bool) {
	w := getWorld(bodyId.world0)
	if w.locked {
		return
	}
	b := getBodyFullId(w, bodyId)
	getBodySim(w, b).isBullet = flag
}

// IsBullet reports whether the body is treated as a bullet. It corresponds
// to b2Body_IsBullet.
func (bodyId BodyId) IsBullet() bool {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return getBodySim(w, b).isBullet
}

// EnableContactEvents enables contact events on every body shape. It
// corresponds to b2Body_EnableContactEvents.
func (bodyId BodyId) EnableContactEvents(flag bool) {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	for shapeId := b.headShapeId; shapeId != nullIndex; shapeId = w.shapes[shapeId].nextShapeId {
		w.shapes[shapeId].enableContactEvents = flag
	}
}

// EnableHitEvents enables hit events on every body shape. It corresponds to
// b2Body_EnableHitEvents.
func (bodyId BodyId) EnableHitEvents(flag bool) {
	w := getWorld(bodyId.world0)
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
			data[count].ShapeIdA = ShapeId{index1: int32(sA.id) + 1, world0: bodyId.world0, generation: sA.generation}
			data[count].ShapeIdB = ShapeId{index1: int32(sB.id) + 1, world0: bodyId.world0, generation: sB.generation}
			data[count].Manifold = getContactSim(w, c).manifold
			count++
		}
		contactKey = c.edges[edgeIndex].nextKey
	}
	return count
}

// ComputeAABB returns the union of the body's shape bounds. It corresponds
// to b2Body_ComputeAABB.
func (bodyId BodyId) ComputeAABB() AABB {
	w := getWorld(bodyId.world0)
	if w.locked {
		return AABB{}
	}
	b := getBodyFullId(w, bodyId)
	if b.headShapeId == nullIndex {
		transform := getBodyTransform(w, b.id)
		return AABB{LowerBound: transform.P, UpperBound: transform.P}
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

// SetName stores up to 31 bytes of the body name. It corresponds to
// b2Body_SetName.
func (bodyId BodyId) SetName(name string) {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	b.name = [32]byte{}
	copy(b.name[:31], name)
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
	if b.islandId != nullIndex || b.islandPrev != nullIndex || b.islandNext != nullIndex {
		panic("dbox2d: the body is already in an island")
	}
	if setIndex == disabledSet {
		panic("dbox2d: a disabled body has no island")
	}

	isl := createIsland(w, setIndex)

	b.islandId = isl.islandId
	isl.headBody = b.id
	isl.tailBody = b.id
	isl.bodyCount = 1
}

// removeBodyFromIsland unlinks a body from its island and destroys the
// island when it becomes empty. It corresponds to b2RemoveBodyFromIsland in
// src/body.c.
func removeBodyFromIsland(w *world, b *body) {
	if b.islandId == nullIndex {
		if b.islandPrev != nullIndex || b.islandNext != nullIndex {
			panic("dbox2d: a body without an island has island links")
		}
		return
	}

	islandId := b.islandId
	isl := &w.islands[islandId]

	// Fix the island's linked list of sims
	if b.islandPrev != nullIndex {
		prevBody := &w.bodies[b.islandPrev]
		prevBody.islandNext = b.islandNext
	}

	if b.islandNext != nullIndex {
		nextBody := &w.bodies[b.islandNext]
		nextBody.islandPrev = b.islandPrev
	}

	if isl.bodyCount <= 0 {
		panic("dbox2d: the island body count underflows")
	}
	isl.bodyCount -= 1

	if isl.headBody == b.id {
		isl.headBody = b.islandNext

		if isl.headBody == nullIndex {
			// Destroy empty island
			if isl.tailBody != b.id || isl.bodyCount != 0 {
				panic("dbox2d: the emptied island still lists bodies")
			}
			if isl.contactCount != 0 {
				panic("dbox2d: the emptied island still lists contacts")
			}

			// Free the island
			destroyIsland(w, isl.islandId)
		}
	} else if isl.tailBody == b.id {
		isl.tailBody = b.islandPrev
	}

	b.islandId = nullIndex
	b.islandPrev = nullIndex
	b.islandNext = nullIndex
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

// makeSweep builds the sweep of a body sim from its previous and current
// center and rotation. It corresponds to b2MakeSweep in src/body.c.
func makeSweep(sim *bodySim) Sweep {
	return Sweep{
		LocalCenter: sim.localCenter,
		C1:          sim.center0,
		C2:          sim.center,
		Q1:          sim.rotation0,
		Q2:          sim.transform.Q,
	}
}
