package dbox2d

import "github.com/dhannyell/fixed"

// The island management, the joint and contact edges, the body events and the
// sweep live with the solver and the broad-phase. PORTING.md lists the
// deferrals.

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

// getBodyTransform returns the transform of the body with the raw id.
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
	zero := fixed.Zero()
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
		maxExtent:         fixed.Zero(),
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
	b.mass = fixed.Zero()
	b.inertia = fixed.Zero()
	b.sleepThreshold = def.SleepThreshold
	b.sleepTime = fixed.Zero()
	b.bodyType = def.Type
	b.enableSleep = def.EnableSleep
	b.fixedRotation = def.FixedRotation
	b.isSpeedCapped = false
	b.isMarked = false

	// Deferred: an enabled dynamic or kinematic body joins an island here.
	// The island module arrives with the solver.

	return BodyId{index1: int32(bodyId) + 1, world0: w.worldId, generation: b.generation}
}

// DestroyBody destroys a body. Every shape attached to it goes away too.
func DestroyBody(bodyId BodyId) {
	w := getWorldLocked(bodyId.world0)

	b := getBodyFullId(w, bodyId)

	// Deferred: the joints and the contacts attached to the body go away
	// here. Both modules arrive with the solver.

	// Destroy the attached shapes. Deferred with the broad-phase: the shape
	// proxies and the sensors.
	shapeId := b.headShapeId
	for shapeId != nullIndex {
		s := &w.shapes[shapeId]

		// Return shape to free list.
		w.shapeIdPool.freeId(shapeId)
		s.id = nullIndex

		shapeId = s.nextShapeId
	}

	// Deferred: the attached chains go away here, after their shapes.

	// Deferred: the body leaves its island here.

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
	zero := fixed.Zero()
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
		sim.invMass = fixed.One().Div(b.mass)
		localCenter = localCenter.Mul(sim.invMass)
	}

	if zero.Less(b.inertia) && !b.fixedRotation {
		// Center the inertia about the center of mass.
		b.inertia = b.inertia.Sub(b.mass.Mul(localCenter.Dot(localCenter)))
		if !zero.Less(b.inertia) {
			panic("dbox2d: the centered inertia is not positive")
		}
		sim.invInertia = fixed.One().Div(b.inertia)
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

// Position returns the world position of the body origin.
func (bodyId BodyId) Position() Vec2 {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return getBodyTransformQuick(w, b).P
}

// Rotation returns the world rotation of the body.
func (bodyId BodyId) Rotation() Rot {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return getBodyTransformQuick(w, b).Q
}

// Mass returns the mass of the body, usually in kilograms.
func (bodyId BodyId) Mass() Q {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return b.mass
}

// ShapeCount returns the number of shapes on the body.
func (bodyId BodyId) ShapeCount() int {
	w := getWorld(bodyId.world0)
	b := getBodyFullId(w, bodyId)
	return b.shapeCount
}
