package dbox2d

import "github.com/dhannyell/fixed"

// This file corresponds to src/joint.h and src/joint.c of the reference.

// jointEdge links a joint into the joint list of one body. It corresponds
// to b2JointEdge in src/joint.h.
type jointEdge struct {
	bodyId  int
	prevKey int
	nextKey int
}

// joint is the cold joint data, stable in the world joint array. The sim
// data moves between the solver sets and the graph colors. It corresponds
// to b2Joint in src/joint.h.
type joint struct {
	userData any

	// setIndex is the solver set in the world, or nullIndex for a free
	// slot.
	setIndex int

	// colorIndex is the constraint graph color, or nullIndex outside the
	// awake set.
	colorIndex int

	// localIndex is the position of the sim data inside the set or the
	// color.
	localIndex int

	edges [2]jointEdge

	jointId    int
	islandId   int
	islandPrev int
	islandNext int

	jointType JointType

	generation uint16

	isMarked         bool
	collideConnected bool
}

// distanceJoint is the sim data of a distance joint. It corresponds to
// b2DistanceJoint in src/joint.h.
type distanceJoint struct {
	length        Q
	hertz         Q
	dampingRatio  Q
	minLength     Q
	maxLength     Q
	maxMotorForce Q
	motorSpeed    Q

	impulse      Q
	lowerImpulse Q
	upperImpulse Q
	motorImpulse Q

	indexA           int
	indexB           int
	anchorA          Vec2
	anchorB          Vec2
	deltaCenter      Vec2
	distanceSoftness softness
	axialMass        Q

	enableSpring bool
	enableLimit  bool
	enableMotor  bool
}

// motorJoint is the sim data of a motor joint. It corresponds to
// b2MotorJoint in src/joint.h.
type motorJoint struct {
	linearOffset     Vec2
	angularOffset    Q
	linearImpulse    Vec2
	angularImpulse   Q
	maxForce         Q
	maxTorque        Q
	correctionFactor Q

	indexA      int
	indexB      int
	anchorA     Vec2
	anchorB     Vec2
	deltaCenter Vec2
	deltaAngle  Q
	linearMass  Mat22
	angularMass Q
}

// mouseJoint is the sim data of a mouse joint. It corresponds to
// b2MouseJoint in src/joint.h.
type mouseJoint struct {
	targetA      Vec2
	hertz        Q
	dampingRatio Q
	maxForce     Q

	linearImpulse   Vec2
	angularImpulse  Q
	linearSoftness  softness
	angularSoftness softness

	indexB      int
	anchorB     Vec2
	deltaCenter Vec2
	linearMass  Mat22
}

// prismaticJoint is the sim data of a prismatic joint. It corresponds to
// b2PrismaticJoint in src/joint.h.
type prismaticJoint struct {
	localAxisA        Vec2
	impulse           Vec2
	springImpulse     Q
	motorImpulse      Q
	lowerImpulse      Q
	upperImpulse      Q
	hertz             Q
	dampingRatio      Q
	targetTranslation Q
	maxMotorForce     Q
	motorSpeed        Q
	referenceAngle    Q
	lowerTranslation  Q
	upperTranslation  Q

	indexA         int
	indexB         int
	anchorA        Vec2
	anchorB        Vec2
	axisA          Vec2
	deltaCenter    Vec2
	deltaAngle     Q
	axialMass      Q
	springSoftness softness

	enableSpring bool
	enableLimit  bool
	enableMotor  bool
}

// revoluteJoint is the sim data of a revolute joint. It corresponds to
// b2RevoluteJoint in src/joint.h.
type revoluteJoint struct {
	linearImpulse  Vec2
	springImpulse  Q
	motorImpulse   Q
	lowerImpulse   Q
	upperImpulse   Q
	hertz          Q
	dampingRatio   Q
	targetAngle    Q
	maxMotorTorque Q
	motorSpeed     Q
	referenceAngle Q
	lowerAngle     Q
	upperAngle     Q

	indexA         int
	indexB         int
	anchorA        Vec2
	anchorB        Vec2
	deltaCenter    Vec2
	deltaAngle     Q
	axialMass      Q
	springSoftness softness

	enableSpring bool
	enableMotor  bool
	enableLimit  bool
}

// weldJoint is the sim data of a weld joint. It corresponds to
// b2WeldJoint in src/joint.h.
type weldJoint struct {
	referenceAngle      Q
	linearHertz         Q
	linearDampingRatio  Q
	angularHertz        Q
	angularDampingRatio Q
	linearSoftness      softness
	angularSoftness     softness
	linearImpulse       Vec2
	angularImpulse      Q

	indexA      int
	indexB      int
	anchorA     Vec2
	anchorB     Vec2
	deltaCenter Vec2
	deltaAngle  Q
	axialMass   Q
}

// wheelJoint is the sim data of a wheel joint. It corresponds to
// b2WheelJoint in src/joint.h.
type wheelJoint struct {
	localAxisA       Vec2
	perpImpulse      Q
	motorImpulse     Q
	springImpulse    Q
	lowerImpulse     Q
	upperImpulse     Q
	maxMotorTorque   Q
	motorSpeed       Q
	lowerTranslation Q
	upperTranslation Q
	hertz            Q
	dampingRatio     Q

	indexA         int
	indexB         int
	anchorA        Vec2
	anchorB        Vec2
	axisA          Vec2
	deltaCenter    Vec2
	perpMass       Q
	motorMass      Q
	axialMass      Q
	springSoftness softness

	enableSpring bool
	enableMotor  bool
	enableLimit  bool
}

// jointSim is the warm joint data that the solver consumes. It lives in
// a solver set or in a graph color. It corresponds to b2JointSim in
// src/joint.h.
type jointSim struct {
	jointId int

	bodyIdA int
	bodyIdB int

	jointType JointType

	// Anchors relative to body origin
	localOriginAnchorA Vec2
	localOriginAnchorB Vec2

	invMassA, invMassB Q
	invIA, invIB       Q

	constraintHertz        Q
	constraintDampingRatio Q

	constraintSoftness softness

	// The reference stores the joint data in a union, so the sim carries
	// all seven; jointType selects the live one.
	distanceJoint  distanceJoint
	motorJoint     motorJoint
	mouseJoint     mouseJoint
	revoluteJoint  revoluteJoint
	prismaticJoint prismaticJoint
	weldJoint      weldJoint
	wheelJoint     wheelJoint
}

// jointPair binds the cold and the warm data of one joint. It corresponds
// to b2JointPair in src/joint.h.
type jointPair struct {
	joint    *joint
	jointSim *jointSim
}

// getJointFullId returns a validated joint from an id. It panics on an
// invalid id (D-003). It corresponds to b2GetJointFullId in src/joint.c.
func getJointFullId(w *world, jointId JointId) *joint {
	id := int(jointId.index1) - 1
	if id < 0 || id >= len(w.joints) {
		panic("dbox2d: the joint id is out of range")
	}
	j := &w.joints[id]
	if j.jointId != id || j.generation != jointId.generation {
		panic("dbox2d: the joint id is stale")
	}
	return j
}

// getJointSim returns the sim data of a joint from its set or color. It
// corresponds to b2GetJointSim in src/joint.c.
func getJointSim(w *world, j *joint) *jointSim {
	if j.setIndex == awakeSet {
		if j.colorIndex < 0 || j.colorIndex >= graphColorCount {
			panic("dbox2d: the color index is out of range")
		}
		color := &w.constraintGraph.colors[j.colorIndex]
		return &color.jointSims[j.localIndex]
	}

	set := &w.solverSets[j.setIndex]
	return &set.jointSims[j.localIndex]
}

// getJointSimCheckType returns the sim data of a joint of the expected
// type. It corresponds to b2GetJointSimCheckType in src/joint.c.
func getJointSimCheckType(jointId JointId, jointType JointType) *jointSim {
	w := getWorldLocked(jointId.world0)

	j := getJointFullId(w, jointId)
	if j.jointType != jointType {
		panic("dbox2d: the joint has another type")
	}
	sim := getJointSim(w, j)
	if sim.jointType != jointType {
		panic("dbox2d: the joint sim has another type")
	}
	return sim
}

// createJoint allocates a joint between two bodies, links it into both
// body lists and picks its solver set. It corresponds to b2CreateJoint in
// src/joint.c.
func createJoint(w *world, bodyA, bodyB *body, userData any, jointType JointType, collideConnected bool) jointPair {
	bodyIdA := bodyA.id
	bodyIdB := bodyB.id
	maxSetIndex := max(bodyA.setIndex, bodyB.setIndex)

	// Create joint id and joint
	jointId := w.jointIdPool.allocId()
	if jointId == len(w.joints) {
		w.joints = append(w.joints, joint{})
	}

	j := &w.joints[jointId]
	j.jointId = jointId
	j.userData = userData
	j.generation += 1
	j.setIndex = nullIndex
	j.colorIndex = nullIndex
	j.localIndex = nullIndex
	j.islandId = nullIndex
	j.islandPrev = nullIndex
	j.islandNext = nullIndex
	j.jointType = jointType
	j.collideConnected = collideConnected
	j.isMarked = false

	// Doubly linked list on bodyA
	j.edges[0].bodyId = bodyIdA
	j.edges[0].prevKey = nullIndex
	j.edges[0].nextKey = bodyA.headJointKey

	keyA := jointId << 1
	if bodyA.headJointKey != nullIndex {
		jointA := &w.joints[bodyA.headJointKey>>1]
		edgeA := &jointA.edges[bodyA.headJointKey&1]
		edgeA.prevKey = keyA
	}
	bodyA.headJointKey = keyA
	bodyA.jointCount += 1

	// Doubly linked list on bodyB
	j.edges[1].bodyId = bodyIdB
	j.edges[1].prevKey = nullIndex
	j.edges[1].nextKey = bodyB.headJointKey

	keyB := jointId<<1 | 1
	if bodyB.headJointKey != nullIndex {
		jointB := &w.joints[bodyB.headJointKey>>1]
		edgeB := &jointB.edges[bodyB.headJointKey&1]
		edgeB.prevKey = keyB
	}
	bodyB.headJointKey = keyB
	bodyB.jointCount += 1

	var js *jointSim

	switch {
	case bodyA.setIndex == disabledSet || bodyB.setIndex == disabledSet:
		// if either body is disabled, create in disabled set
		set := &w.solverSets[disabledSet]
		j.setIndex = disabledSet
		j.localIndex = len(set.jointSims)

		set.jointSims = append(set.jointSims, jointSim{})
		js = &set.jointSims[j.localIndex]

		js.jointId = jointId
		js.bodyIdA = bodyIdA
		js.bodyIdB = bodyIdB

	case bodyA.setIndex == staticSet && bodyB.setIndex == staticSet:
		// joint is connecting static bodies
		set := &w.solverSets[staticSet]
		j.setIndex = staticSet
		j.localIndex = len(set.jointSims)

		set.jointSims = append(set.jointSims, jointSim{})
		js = &set.jointSims[j.localIndex]

		js.jointId = jointId
		js.bodyIdA = bodyIdA
		js.bodyIdB = bodyIdB

	case bodyA.setIndex == awakeSet || bodyB.setIndex == awakeSet:
		// if either body is sleeping, wake it
		if maxSetIndex >= firstSleepingSet {
			wakeSolverSet(w, maxSetIndex)
		}

		j.setIndex = awakeSet

		js = createJointInGraph(w, j)
		js.jointId = jointId
		js.bodyIdA = bodyIdA
		js.bodyIdB = bodyIdB

	default:
		// joint connected between sleeping and/or static bodies
		if bodyA.setIndex < firstSleepingSet && bodyB.setIndex < firstSleepingSet {
			panic("dbox2d: neither body sleeps")
		}
		if bodyA.setIndex == staticSet && bodyB.setIndex == staticSet {
			panic("dbox2d: both bodies are static")
		}

		// joint should go into the sleeping set (not static set)
		setIndex := maxSetIndex

		set := &w.solverSets[setIndex]
		j.setIndex = setIndex
		j.localIndex = len(set.jointSims)

		set.jointSims = append(set.jointSims, jointSim{})
		js = &set.jointSims[j.localIndex]

		js.jointId = jointId
		js.bodyIdA = bodyIdA
		js.bodyIdB = bodyIdB

		if bodyA.setIndex != bodyB.setIndex && bodyA.setIndex >= firstSleepingSet &&
			bodyB.setIndex >= firstSleepingSet {
			// merge sleeping sets
			mergeSolverSets(w, bodyA.setIndex, bodyB.setIndex)
			if bodyA.setIndex != bodyB.setIndex {
				panic("dbox2d: the merge left the bodies in two sets")
			}

			// fix potentially invalid set index
			setIndex = bodyA.setIndex

			mergedSet := &w.solverSets[setIndex]

			// Careful! The joint sim pointer was orphaned by the set merge.
			js = &mergedSet.jointSims[j.localIndex]
		}

		if j.setIndex != setIndex {
			panic("dbox2d: the joint set index is wrong")
		}
	}

	js.constraintHertz = jointConstraintHertz
	js.constraintDampingRatio = jointConstraintDampingRatio
	js.constraintSoftness = softness{
		biasRate:     fixed.Q32Zero(),
		massScale:    fixed.Q32One(),
		impulseScale: fixed.Q32Zero(),
	}

	if js.jointId != jointId || js.bodyIdA != bodyIdA || js.bodyIdB != bodyIdB {
		panic("dbox2d: the joint sim does not match the joint")
	}

	if j.setIndex > disabledSet {
		// Add edge to island graph
		mergeIslands := true
		linkJoint(w, j, mergeIslands)
	}

	validateSolverSets(w)

	return jointPair{j, js}
}

// destroyContactsBetweenBodies removes every contact between two bodies
// that a joint now keeps apart. It corresponds to
// b2DestroyContactsBetweenBodies in src/joint.c.
func destroyContactsBetweenBodies(w *world, bodyA, bodyB *body) {
	var contactKey int
	var otherBodyId int

	// use the smaller of the two contact lists
	if bodyA.contactCount < bodyB.contactCount {
		contactKey = bodyA.headContactKey
		otherBodyId = bodyB.id
	} else {
		contactKey = bodyB.headContactKey
		otherBodyId = bodyA.id
	}

	// no need to wake bodies when a joint removes collision between them
	wakeBodies := false

	// destroy the contacts
	for contactKey != nullIndex {
		contactId := contactKey >> 1
		edgeIndex := contactKey & 1

		c := &w.contacts[contactId]
		contactKey = c.edges[edgeIndex].nextKey

		otherEdgeIndex := edgeIndex ^ 1
		if c.edges[otherEdgeIndex].bodyId == otherBodyId {
			// Careful, this removes the contact from the current doubly linked list
			destroyContact(w, c, wakeBodies)
		}
	}

	validateSolverSets(w)
}

// getJointWorld returns the unlocked world of a joint definition and its
// two validated bodies.
func getJointWorld(worldId WorldId, bodyIdA, bodyIdB BodyId) (*world, *body, *body) {
	w := getWorldFromId(worldId)
	if w.locked {
		panic("dbox2d: the world is locked")
	}
	if !bodyIdA.IsValid() {
		panic("dbox2d: BodyIdA is not valid")
	}
	if !bodyIdB.IsValid() {
		panic("dbox2d: BodyIdB is not valid")
	}
	bodyA := getBodyFullId(w, bodyIdA)
	bodyB := getBodyFullId(w, bodyIdB)
	return w, bodyA, bodyB
}

// makeJointId builds the public handle of a new joint.
func makeJointId(w *world, pair jointPair) JointId {
	return JointId{index1: int32(pair.jointSim.jointId) + 1, world0: w.worldId, generation: pair.joint.generation}
}

// CreateFilterJoint creates a filter joint. The joint only disables
// collision between the two bodies. The definition must come from
// DefaultFilterJointDef.
func CreateFilterJoint(worldId WorldId, def *FilterJointDef) JointId {
	checkDef(def.internalValue)
	w, bodyA, bodyB := getJointWorld(worldId, def.BodyIdA, def.BodyIdB)

	collideConnected := false
	pair := createJoint(w, bodyA, bodyB, def.UserData, FilterJoint, collideConnected)

	js := pair.jointSim
	js.jointType = FilterJoint
	js.localOriginAnchorA = Vec2Zero()
	js.localOriginAnchorB = Vec2Zero()

	return makeJointId(w, pair)
}

// CreateDistanceJoint creates a distance joint. The definition must come
// from DefaultDistanceJointDef.
func CreateDistanceJoint(worldId WorldId, def *DistanceJointDef) JointId {
	checkDef(def.internalValue)
	zero := fixed.Q32Zero()
	if !IsValidQ(def.Length) || !zero.Less(def.Length) {
		panic("dbox2d: DistanceJointDef.Length must be positive")
	}
	w, bodyA, bodyB := getJointWorld(worldId, def.BodyIdA, def.BodyIdB)

	pair := createJoint(w, bodyA, bodyB, def.UserData, DistanceJoint, def.CollideConnected)

	js := pair.jointSim
	js.jointType = DistanceJoint
	js.localOriginAnchorA = def.LocalAnchorA
	js.localOriginAnchorB = def.LocalAnchorB

	js.distanceJoint = distanceJoint{}
	js.distanceJoint.length = def.Length.Max(linearSlop)
	js.distanceJoint.hertz = def.Hertz
	js.distanceJoint.dampingRatio = def.DampingRatio
	js.distanceJoint.minLength = def.MinLength.Max(linearSlop)
	js.distanceJoint.maxLength = def.MinLength.Max(def.MaxLength)
	js.distanceJoint.maxMotorForce = def.MaxMotorForce
	js.distanceJoint.motorSpeed = def.MotorSpeed
	js.distanceJoint.enableSpring = def.EnableSpring
	js.distanceJoint.enableLimit = def.EnableLimit
	js.distanceJoint.enableMotor = def.EnableMotor
	js.distanceJoint.impulse = zero
	js.distanceJoint.lowerImpulse = zero
	js.distanceJoint.upperImpulse = zero
	js.distanceJoint.motorImpulse = zero

	// If the joint prevents collisions, then destroy all contacts between attached bodies
	if !def.CollideConnected {
		destroyContactsBetweenBodies(w, bodyA, bodyB)
	}

	return makeJointId(w, pair)
}

// CreateMotorJoint creates a motor joint. The definition must come from
// DefaultMotorJointDef.
func CreateMotorJoint(worldId WorldId, def *MotorJointDef) JointId {
	checkDef(def.internalValue)
	w, bodyA, bodyB := getJointWorld(worldId, def.BodyIdA, def.BodyIdB)

	pair := createJoint(w, bodyA, bodyB, def.UserData, MotorJoint, def.CollideConnected)
	js := pair.jointSim

	js.jointType = MotorJoint
	js.localOriginAnchorA = Vec2Zero()
	js.localOriginAnchorB = Vec2Zero()
	js.motorJoint = motorJoint{}
	js.motorJoint.linearOffset = def.LinearOffset
	js.motorJoint.angularOffset = def.AngularOffset
	js.motorJoint.maxForce = def.MaxForce
	js.motorJoint.maxTorque = def.MaxTorque
	js.motorJoint.correctionFactor = def.CorrectionFactor.Clamp(fixed.Q32Zero(), fixed.Q32One())

	// If the joint prevents collisions, then destroy all contacts between attached bodies
	if !def.CollideConnected {
		destroyContactsBetweenBodies(w, bodyA, bodyB)
	}

	return makeJointId(w, pair)
}

// CreateMouseJoint creates a mouse joint. The definition must come from
// DefaultMouseJointDef.
func CreateMouseJoint(worldId WorldId, def *MouseJointDef) JointId {
	checkDef(def.internalValue)
	w, bodyA, bodyB := getJointWorld(worldId, def.BodyIdA, def.BodyIdB)

	transformA := getBodyTransformQuick(w, bodyA)
	transformB := getBodyTransformQuick(w, bodyB)

	pair := createJoint(w, bodyA, bodyB, def.UserData, MouseJoint, def.CollideConnected)

	js := pair.jointSim
	js.jointType = MouseJoint
	js.localOriginAnchorA = InvTransformPoint(transformA, def.Target)
	js.localOriginAnchorB = InvTransformPoint(transformB, def.Target)

	js.mouseJoint = mouseJoint{}
	js.mouseJoint.targetA = def.Target
	js.mouseJoint.hertz = def.Hertz
	js.mouseJoint.dampingRatio = def.DampingRatio
	js.mouseJoint.maxForce = def.MaxForce

	return makeJointId(w, pair)
}

// CreatePrismaticJoint creates a prismatic joint. The definition must come
// from DefaultPrismaticJointDef.
func CreatePrismaticJoint(worldId WorldId, def *PrismaticJointDef) JointId {
	checkDef(def.internalValue)
	if def.UpperTranslation.Less(def.LowerTranslation) {
		panic("dbox2d: PrismaticJointDef.LowerTranslation exceeds UpperTranslation")
	}
	w, bodyA, bodyB := getJointWorld(worldId, def.BodyIdA, def.BodyIdB)

	pair := createJoint(w, bodyA, bodyB, def.UserData, PrismaticJoint, def.CollideConnected)

	js := pair.jointSim
	js.jointType = PrismaticJoint
	js.localOriginAnchorA = def.LocalAnchorA
	js.localOriginAnchorB = def.LocalAnchorB

	js.prismaticJoint = prismaticJoint{}

	js.prismaticJoint.localAxisA = def.LocalAxisA.Normalize()
	js.prismaticJoint.referenceAngle = def.ReferenceAngle
	js.prismaticJoint.targetTranslation = def.TargetTranslation
	js.prismaticJoint.hertz = def.Hertz
	js.prismaticJoint.dampingRatio = def.DampingRatio
	js.prismaticJoint.lowerTranslation = def.LowerTranslation
	js.prismaticJoint.upperTranslation = def.UpperTranslation
	js.prismaticJoint.maxMotorForce = def.MaxMotorForce
	js.prismaticJoint.motorSpeed = def.MotorSpeed
	js.prismaticJoint.enableSpring = def.EnableSpring
	js.prismaticJoint.enableLimit = def.EnableLimit
	js.prismaticJoint.enableMotor = def.EnableMotor

	// If the joint prevents collisions, then destroy all contacts between attached bodies
	if !def.CollideConnected {
		destroyContactsBetweenBodies(w, bodyA, bodyB)
	}

	return makeJointId(w, pair)
}

// CreateRevoluteJoint creates a revolute joint. The definition must come
// from DefaultRevoluteJointDef. The angles are in turns.
func CreateRevoluteJoint(worldId WorldId, def *RevoluteJointDef) JointId {
	checkDef(def.internalValue)
	if def.UpperAngle.Less(def.LowerAngle) {
		panic("dbox2d: RevoluteJointDef.LowerAngle exceeds UpperAngle")
	}
	// The reference limits the range to 0.99 pi; in turns that is 0.495.
	limitAngle := fixed.Q32MustParse("0.495")
	if def.LowerAngle.Less(limitAngle.Neg()) {
		panic("dbox2d: RevoluteJointDef.LowerAngle is below -0.495 turns")
	}
	if limitAngle.Less(def.UpperAngle) {
		panic("dbox2d: RevoluteJointDef.UpperAngle is above 0.495 turns")
	}

	w, bodyA, bodyB := getJointWorld(worldId, def.BodyIdA, def.BodyIdB)

	pair := createJoint(w, bodyA, bodyB, def.UserData, RevoluteJoint, def.CollideConnected)

	js := pair.jointSim
	js.jointType = RevoluteJoint
	js.localOriginAnchorA = def.LocalAnchorA
	js.localOriginAnchorB = def.LocalAnchorB

	js.revoluteJoint = revoluteJoint{}

	halfTurn := fixed.Q32Half()
	js.revoluteJoint.referenceAngle = def.ReferenceAngle.Clamp(halfTurn.Neg(), halfTurn)
	js.revoluteJoint.targetAngle = def.TargetAngle.Clamp(halfTurn.Neg(), halfTurn)
	js.revoluteJoint.hertz = def.Hertz
	js.revoluteJoint.dampingRatio = def.DampingRatio
	js.revoluteJoint.lowerAngle = def.LowerAngle
	js.revoluteJoint.upperAngle = def.UpperAngle
	js.revoluteJoint.maxMotorTorque = def.MaxMotorTorque
	js.revoluteJoint.motorSpeed = def.MotorSpeed
	js.revoluteJoint.enableSpring = def.EnableSpring
	js.revoluteJoint.enableLimit = def.EnableLimit
	js.revoluteJoint.enableMotor = def.EnableMotor

	// If the joint prevents collisions, then destroy all contacts between attached bodies
	if !def.CollideConnected {
		destroyContactsBetweenBodies(w, bodyA, bodyB)
	}

	return makeJointId(w, pair)
}

// CreateWeldJoint creates a weld joint. The definition must come from
// DefaultWeldJointDef.
func CreateWeldJoint(worldId WorldId, def *WeldJointDef) JointId {
	checkDef(def.internalValue)
	w, bodyA, bodyB := getJointWorld(worldId, def.BodyIdA, def.BodyIdB)

	pair := createJoint(w, bodyA, bodyB, def.UserData, WeldJoint, def.CollideConnected)

	js := pair.jointSim
	js.jointType = WeldJoint
	js.localOriginAnchorA = def.LocalAnchorA
	js.localOriginAnchorB = def.LocalAnchorB

	js.weldJoint = weldJoint{}
	js.weldJoint.referenceAngle = def.ReferenceAngle
	js.weldJoint.linearHertz = def.LinearHertz
	js.weldJoint.linearDampingRatio = def.LinearDampingRatio
	js.weldJoint.angularHertz = def.AngularHertz
	js.weldJoint.angularDampingRatio = def.AngularDampingRatio
	js.weldJoint.linearImpulse = Vec2Zero()
	js.weldJoint.angularImpulse = fixed.Q32Zero()

	// If the joint prevents collisions, then destroy all contacts between attached bodies
	if !def.CollideConnected {
		destroyContactsBetweenBodies(w, bodyA, bodyB)
	}

	return makeJointId(w, pair)
}

// CreateWheelJoint creates a wheel joint. The definition must come from
// DefaultWheelJointDef.
func CreateWheelJoint(worldId WorldId, def *WheelJointDef) JointId {
	checkDef(def.internalValue)
	if def.UpperTranslation.Less(def.LowerTranslation) {
		panic("dbox2d: WheelJointDef.LowerTranslation exceeds UpperTranslation")
	}
	w, bodyA, bodyB := getJointWorld(worldId, def.BodyIdA, def.BodyIdB)

	pair := createJoint(w, bodyA, bodyB, def.UserData, WheelJoint, def.CollideConnected)

	js := pair.jointSim
	js.jointType = WheelJoint
	js.localOriginAnchorA = def.LocalAnchorA
	js.localOriginAnchorB = def.LocalAnchorB

	zero := fixed.Q32Zero()
	js.wheelJoint = wheelJoint{}
	js.wheelJoint.localAxisA = def.LocalAxisA.Normalize()
	js.wheelJoint.perpMass = zero
	js.wheelJoint.axialMass = zero
	js.wheelJoint.motorImpulse = zero
	js.wheelJoint.lowerImpulse = zero
	js.wheelJoint.upperImpulse = zero
	js.wheelJoint.lowerTranslation = def.LowerTranslation
	js.wheelJoint.upperTranslation = def.UpperTranslation
	js.wheelJoint.maxMotorTorque = def.MaxMotorTorque
	js.wheelJoint.motorSpeed = def.MotorSpeed
	js.wheelJoint.hertz = def.Hertz
	js.wheelJoint.dampingRatio = def.DampingRatio
	js.wheelJoint.enableSpring = def.EnableSpring
	js.wheelJoint.enableLimit = def.EnableLimit
	js.wheelJoint.enableMotor = def.EnableMotor

	// If the joint prevents collisions, then destroy all contacts between attached bodies
	if !def.CollideConnected {
		destroyContactsBetweenBodies(w, bodyA, bodyB)
	}

	return makeJointId(w, pair)
}

// destroyJointInternal unlinks a joint from its bodies, its island and its
// set, then frees the id. It corresponds to b2DestroyJointInternal in
// src/joint.c.
func destroyJointInternal(w *world, j *joint, wakeBodies bool) {
	jointId := j.jointId

	edgeA := &j.edges[0]
	edgeB := &j.edges[1]

	idA := edgeA.bodyId
	idB := edgeB.bodyId
	bodyA := &w.bodies[idA]
	bodyB := &w.bodies[idB]

	// Remove from body A
	if edgeA.prevKey != nullIndex {
		prevJoint := &w.joints[edgeA.prevKey>>1]
		prevEdge := &prevJoint.edges[edgeA.prevKey&1]
		prevEdge.nextKey = edgeA.nextKey
	}

	if edgeA.nextKey != nullIndex {
		nextJoint := &w.joints[edgeA.nextKey>>1]
		nextEdge := &nextJoint.edges[edgeA.nextKey&1]
		nextEdge.prevKey = edgeA.prevKey
	}

	edgeKeyA := jointId << 1
	if bodyA.headJointKey == edgeKeyA {
		bodyA.headJointKey = edgeA.nextKey
	}

	bodyA.jointCount -= 1

	// Remove from body B
	if edgeB.prevKey != nullIndex {
		prevJoint := &w.joints[edgeB.prevKey>>1]
		prevEdge := &prevJoint.edges[edgeB.prevKey&1]
		prevEdge.nextKey = edgeB.nextKey
	}

	if edgeB.nextKey != nullIndex {
		nextJoint := &w.joints[edgeB.nextKey>>1]
		nextEdge := &nextJoint.edges[edgeB.nextKey&1]
		nextEdge.prevKey = edgeB.prevKey
	}

	edgeKeyB := jointId<<1 | 1
	if bodyB.headJointKey == edgeKeyB {
		bodyB.headJointKey = edgeB.nextKey
	}

	bodyB.jointCount -= 1

	if j.islandId != nullIndex {
		if j.setIndex <= disabledSet {
			panic("dbox2d: a joint in an island must be in a live set")
		}
		unlinkJoint(w, j)
	} else if j.setIndex > disabledSet {
		panic("dbox2d: a joint in a live set must be in an island")
	}

	// Remove joint from solver set that owns it
	setIndex := j.setIndex
	localIndex := j.localIndex

	if setIndex == awakeSet {
		removeJointFromGraph(w, j.edges[0].bodyId, j.edges[1].bodyId, j.colorIndex, localIndex)
	} else {
		set := &w.solverSets[setIndex]
		var movedIndex int
		set.jointSims, movedIndex = removeSwap(set.jointSims, localIndex)
		if movedIndex != nullIndex {
			// Fix moved joint
			movedJointSim := &set.jointSims[localIndex]
			movedId := movedJointSim.jointId
			movedJoint := &w.joints[movedId]
			if movedJoint.localIndex != movedIndex {
				panic("dbox2d: the moved joint has the wrong local index")
			}
			movedJoint.localIndex = localIndex
		}
	}

	// Free joint and id (preserve joint generation)
	j.setIndex = nullIndex
	j.localIndex = nullIndex
	j.colorIndex = nullIndex
	j.jointId = nullIndex
	w.jointIdPool.freeId(jointId)

	if wakeBodies {
		wakeBody(w, bodyA)
		wakeBody(w, bodyB)
	}

	validateSolverSets(w)
}

// DestroyJoint destroys a joint and wakes the two bodies.
func DestroyJoint(jointId JointId) {
	w := getWorldLocked(jointId.world0)

	j := getJointFullId(w, jointId)

	destroyJointInternal(w, j, true)
}

// jointStates returns the solver states of the two bodies of a joint. A
// body outside the awake set gets the dummy state.
func jointStates(states []bodyState, dummy *bodyState, indexA, indexB int) (stateA, stateB *bodyState) {
	stateA = dummy
	if indexA != nullIndex {
		stateA = &states[indexA]
	}
	stateB = dummy
	if indexB != nullIndex {
		stateB = &states[indexB]
	}
	return stateA, stateB
}

// prepareJoint corresponds to b2PrepareJoint in src/joint.c.
func prepareJoint(js *jointSim, context *stepContext) {
	// Clamp joint hertz based on the time step to reduce jitter.
	// D-006: the reference multiplies the inverse step by 0.25.
	hertz := js.constraintHertz.Min(context.invH.Div(fixed.Q32FromInt(4)))
	js.constraintSoftness = makeSoft(hertz, js.constraintDampingRatio, context.h)

	switch js.jointType {
	case FilterJoint:
	case DistanceJoint:
		prepareDistanceJoint(js, context)
	case PrismaticJoint:
		preparePrismaticJoint(js, context)
	case RevoluteJoint:
		prepareRevoluteJoint(js, context)
	default:
		panic("dbox2d: joint type not ported")
	}
}

// warmStartJoint corresponds to b2WarmStartJoint in src/joint.c.
func warmStartJoint(js *jointSim, context *stepContext) {
	switch js.jointType {
	case FilterJoint:
	case DistanceJoint:
		warmStartDistanceJoint(js, context)
	case PrismaticJoint:
		warmStartPrismaticJoint(js, context)
	case RevoluteJoint:
		warmStartRevoluteJoint(js, context)
	default:
		panic("dbox2d: joint type not ported")
	}
}

// solveJoint corresponds to b2SolveJoint in src/joint.c.
func solveJoint(js *jointSim, context *stepContext, useBias bool) {
	switch js.jointType {
	case FilterJoint:
	case DistanceJoint:
		solveDistanceJoint(js, context, useBias)
	case PrismaticJoint:
		solvePrismaticJoint(js, context, useBias)
	case RevoluteJoint:
		solveRevoluteJoint(js, context, useBias)
	default:
		panic("dbox2d: joint type not ported")
	}
}

// prepareJoints prepares the joints of one color. It corresponds to
// b2PrepareOverflowJoints in src/joint.c and to b2PrepareJointsTask in
// src/solver.c.
func prepareJoints(context *stepContext, colorIndex int) {
	joints := context.graph.colors[colorIndex].jointSims

	for i := range joints {
		prepareJoint(&joints[i], context)
	}
}

// warmStartJoints applies the stored impulses of the joints of one color.
// It corresponds to b2WarmStartOverflowJoints in src/joint.c and to
// b2WarmStartJointsTask in src/solver.c.
func warmStartJoints(context *stepContext, colorIndex int) {
	joints := context.graph.colors[colorIndex].jointSims

	for i := range joints {
		warmStartJoint(&joints[i], context)
	}
}

// solveJoints runs one iteration over the joints of one color. It
// corresponds to b2SolveOverflowJoints in src/joint.c and to
// b2SolveJointsTask in src/solver.c.
func solveJoints(context *stepContext, colorIndex int, useBias bool) {
	joints := context.graph.colors[colorIndex].jointSims

	for i := range joints {
		solveJoint(&joints[i], context, useBias)
	}
}
