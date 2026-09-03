package dbox2d

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
