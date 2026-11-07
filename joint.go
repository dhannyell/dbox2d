package b2

import "unsafe"

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

	jointId     int
	islandId    int
	islandIndex int

	jointType JointType
	drawScale Q

	generation uint16

	collideConnected bool
}

// distanceJoint is the sim data of a distance joint. It corresponds to
// b2DistanceJoint in src/joint.h.
type distanceJoint struct {
	length           Q
	hertz            Q
	dampingRatio     Q
	lowerSpringForce Q
	upperSpringForce Q
	minLength        Q
	maxLength        Q
	maxMotorForce    Q
	motorSpeed       Q

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
// b2MotorJoint in src/joint.h. The angular velocity is in the angle unit of
// the mode (D-004).
type motorJoint struct {
	linearVelocity      Vec2
	maxVelocityForce    Q
	angularVelocity     Q
	maxVelocityTorque   Q
	linearHertz         Q
	linearDampingRatio  Q
	maxSpringForce      Q
	angularHertz        Q
	angularDampingRatio Q
	maxSpringTorque     Q

	linearVelocityImpulse  Vec2
	angularVelocityImpulse Q
	linearSpringImpulse    Vec2
	angularSpringImpulse   Q

	linearSpring  softness
	angularSpring softness

	indexA      int
	indexB      int
	frameA      Transform
	frameB      Transform
	deltaCenter Vec2
	linearMass  Mat22
	angularMass Q
}

// moverJoint is the sim data of a mover joint. It corresponds to
// b2MoverJoint in src/joint.h.
type moverJoint struct {
	linearVelocity        Vec2
	maxVelocityForce      Vec2
	linearVelocityImpulse Vec2

	indexA     int
	indexB     int
	frameA     Transform
	frameB     Transform
	linearMass Q
}

// pogoJoint is the sim data of a pogo joint, a hybrid between a contact
// and a joint. It corresponds to b2PogoJoint in src/joint.h.
type pogoJoint struct {
	normal              Vec2
	restLength          Q
	hertz               Q
	dampingRatio        Q
	maxTensionForce     Q
	maxCompressionForce Q

	impulse     Q
	indexA      int
	indexB      int
	frameA      Transform
	frameB      Transform
	deltaCenter Vec2
	linearMass  Q
	velocity    Q
}

// prismaticJoint is the sim data of a prismatic joint. It corresponds to
// b2PrismaticJoint in src/joint.h.
type prismaticJoint struct {
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
	lowerTranslation  Q
	upperTranslation  Q

	indexA         int
	indexB         int
	frameA         Transform
	frameB         Transform
	deltaCenter    Vec2
	springSoftness softness

	enableSpring bool
	enableLimit  bool
	enableMotor  bool
}

// revoluteJoint is the sim data of a revolute joint. It corresponds to
// b2RevoluteJoint in src/joint.h. The angles are in the angle unit of the
// mode (D-004).
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
	lowerAngle     Q
	upperAngle     Q

	indexA         int
	indexB         int
	frameA         Transform
	frameB         Transform
	deltaCenter    Vec2
	axialMass      Q
	springSoftness softness

	enableSpring bool
	enableMotor  bool
	enableLimit  bool
}

// weldJoint is the sim data of a weld joint. It corresponds to
// b2WeldJoint in src/joint.h.
type weldJoint struct {
	linearHertz         Q
	linearDampingRatio  Q
	angularHertz        Q
	angularDampingRatio Q
	linearSpring        softness
	angularSpring       softness
	linearImpulse       Vec2
	angularImpulse      Q

	indexA      int
	indexB      int
	frameA      Transform
	frameB      Transform
	deltaCenter Vec2
	axialMass   Q
}

// wheelJoint is the sim data of a wheel joint. It corresponds to
// b2WheelJoint in src/joint.h.
type wheelJoint struct {
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
	frameA         Transform
	frameB         Transform
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

	// Joint frames relative to the body origins
	localFrameA Transform
	localFrameB Transform

	invMassA, invMassB Q
	invIA, invIB       Q

	constraintHertz        Q
	constraintDampingRatio Q

	constraintSoftness softness

	forceThreshold  Q
	torqueThreshold Q

	// The reference stores the joint data in a union; jointType selects
	// the live one. The eight payloads share this storage through the
	// typed accessors below, so a sim is a fraction of the size it would be
	// with all eight inline, which is what the solver and the set moves pay.
	payload jointPayload
}

// jointPayload is the storage of the joint union: the largest of the eight
// payloads, in 8-byte words so every field of every payload is aligned.
// None of the payloads holds a pointer, so the garbage collector never
// looks inside.
type jointPayload [(max(
	unsafe.Sizeof(distanceJoint{}),
	unsafe.Sizeof(motorJoint{}),
	unsafe.Sizeof(moverJoint{}),
	unsafe.Sizeof(pogoJoint{}),
	unsafe.Sizeof(revoluteJoint{}),
	unsafe.Sizeof(prismaticJoint{}),
	unsafe.Sizeof(weldJoint{}),
	unsafe.Sizeof(wheelJoint{}),
) + 7) / 8]uint64

func (js *jointSim) distance() *distanceJoint { return (*distanceJoint)(unsafe.Pointer(&js.payload)) }
func (js *jointSim) motor() *motorJoint       { return (*motorJoint)(unsafe.Pointer(&js.payload)) }
func (js *jointSim) mover() *moverJoint       { return (*moverJoint)(unsafe.Pointer(&js.payload)) }
func (js *jointSim) pogo() *pogoJoint         { return (*pogoJoint)(unsafe.Pointer(&js.payload)) }
func (js *jointSim) revolute() *revoluteJoint { return (*revoluteJoint)(unsafe.Pointer(&js.payload)) }
func (js *jointSim) prismatic() *prismaticJoint {
	return (*prismaticJoint)(unsafe.Pointer(&js.payload))
}
func (js *jointSim) weld() *weldJoint   { return (*weldJoint)(unsafe.Pointer(&js.payload)) }
func (js *jointSim) wheel() *wheelJoint { return (*wheelJoint)(unsafe.Pointer(&js.payload)) }

// jointPair binds the cold and the warm data of one joint. It corresponds
// to b2JointPair in src/joint.h.
type jointPair struct {
	joint    *joint
	jointSim *jointSim
}

// minJointDrawScale keeps the joint drawing visible. It corresponds to the
// 0.0001 floor in b2DrawJoint.
var minJointDrawScale = QMustParse("0.0001")

func drawJoint(draw *DebugDraw, w *world, joint *joint) {
	bodyA := &w.bodies[joint.edges[0].bodyId]
	bodyB := &w.bodies[joint.edges[1].bodyId]
	if bodyA.setIndex == disabledSet || bodyB.setIndex == disabledSet {
		return
	}
	base := getJointSim(w, joint)
	transformA := getBodyTransformQuick(w, bodyA)
	transformB := getBodyTransformQuick(w, bodyB)
	pA := TransformPoint(transformA, base.localFrameA.P)
	pB := TransformPoint(transformB, base.localFrameB.P)

	scale := minJointDrawScale.Max(draw.JointScale.Mul(joint.drawScale))
	pointSize := QFromInt(8)

	switch joint.jointType {
	case DistanceJoint:
		drawDistanceJoint(draw, base, transformA, transformB)
	case FilterJoint:
		draw.DrawPointFcn(pA, pointSize, ColorLightSkyBlue)
		draw.DrawPointFcn(pB, pointSize, ColorLightSkyBlue)
		if draw.DrawJointExtras {
			draw.DrawLineFcn(pA, pB, ColorGold)
		}
	case MotorJoint:
		draw.DrawPointFcn(pA, pointSize, ColorYellowGreen)
		draw.DrawPointFcn(pB, pointSize, ColorPlum)
		if draw.DrawJointExtras {
			draw.DrawLineFcn(pA, pB, ColorLightGray)
		}
	case MoverJoint:
		draw.DrawPointFcn(pB, pointSize, ColorPlum)
	case PogoJoint:
		draw.DrawLineFcn(pA, pB, ColorLightGray)
	case PrismaticJoint:
		drawPrismaticJoint(draw, base, transformA, transformB, scale)
	case RevoluteJoint:
		drawRevoluteJoint(draw, base, transformA, transformB, scale)
	case WeldJoint:
		drawWeldJoint(draw, base, transformA, transformB, scale)
	case WheelJoint:
		drawWheelJoint(draw, base, transformA, transformB, scale)
	}
	if draw.DrawGraphColors && joint.colorIndex != nullIndex {
		draw.DrawPointFcn(LerpPos(pA, pB, QHalf()), QFromInt(5), graphColors[joint.colorIndex])
	}
	if draw.DrawJointExtras {
		force := getJointConstraintForce(w, joint)
		torque := getJointConstraintTorque(w, joint)
		p := LerpPos(pA, pB, QHalf())
		draw.DrawLineFcn(p, OffsetPos(p, force.Mul(QMustParse("0.001"))), ColorAzure)
		text := "f = [" + drawGeneral(force.X) + ", " + drawGeneral(force.Y) + "], t = " + drawGeneral(torque)
		draw.DrawStringFcn(p, text, ColorAzure)
	}
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

// getJointSimCheckType returns joint simulation data after checking its type.
func getJointSimCheckType(w *world, jointId JointId, jointType JointType) *jointSim {
	j := getJointFullId(w, jointId)
	if j.jointType != jointType {
		panic("dbox2d: the joint type does not match")
	}

	js := getJointSim(w, j)
	if js.jointType != jointType {
		panic("dbox2d: the joint sim type does not match")
	}
	return js
}

// IsValid reports whether the id references a live joint (b2Joint_IsValid).
func (jointId JointId) IsValid() bool {
	if maxWorlds <= jointId.world0 {
		// invalid world
		return false
	}

	w := &worlds[jointId.world0]
	if w.worldId != jointId.world0 {
		// world is free
		return false
	}

	if jointId.index1 < 1 || len(w.joints) < int(jointId.index1) {
		// invalid index
		return false
	}

	j := &w.joints[jointId.index1-1]
	if j.setIndex == nullIndex {
		// this was freed
		return false
	}

	if j.localIndex == nullIndex {
		panic("dbox2d: a live joint has no local index")
	}

	if j.generation != jointId.generation {
		// this id is orphaned
		return false
	}

	return true
}

// GetType reports the joint type (b2Joint_GetType).
func (jointId JointId) GetType() JointType {
	w := getWorld(jointId.world0)
	return getJointFullId(w, jointId).jointType
}

// GetBodyA reports the first attached body (b2Joint_GetBodyA).
func (jointId JointId) GetBodyA() BodyId {
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	return makeBodyId(w, j.edges[0].bodyId)
}

// GetBodyB reports the second attached body (b2Joint_GetBodyB).
func (jointId JointId) GetBodyB() BodyId {
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	return makeBodyId(w, j.edges[1].bodyId)
}

// GetWorld reports the world containing the joint (b2Joint_GetWorld).
func (jointId JointId) GetWorld() WorldId {
	w := getWorld(jointId.world0)
	return WorldId{index1: jointId.world0 + 1, generation: w.generation}
}

// SetLocalFrameA changes the joint frame on body A (b2Joint_SetLocalFrameA).
func (jointId JointId) SetLocalFrameA(localFrame Transform) {
	if !IsValidTransform(localFrame) {
		panic("dbox2d: SetLocalFrameA needs a valid transform")
	}
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	getJointSim(w, j).localFrameA = localFrame
}

// GetLocalFrameA reports the joint frame on body A (b2Joint_GetLocalFrameA).
func (jointId JointId) GetLocalFrameA() Transform {
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	return getJointSim(w, j).localFrameA
}

// SetLocalFrameB changes the joint frame on body B (b2Joint_SetLocalFrameB).
func (jointId JointId) SetLocalFrameB(localFrame Transform) {
	if !IsValidTransform(localFrame) {
		panic("dbox2d: SetLocalFrameB needs a valid transform")
	}
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	getJointSim(w, j).localFrameB = localFrame
}

// GetLocalFrameB reports the joint frame on body B (b2Joint_GetLocalFrameB).
func (jointId JointId) GetLocalFrameB() Transform {
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	return getJointSim(w, j).localFrameB
}

// SetCollideConnected changes whether the attached bodies collide (b2Joint_SetCollideConnected).
func (jointId JointId) SetCollideConnected(shouldCollide bool) {
	w := getWorldLocked(jointId.world0)
	j := getJointFullId(w, jointId)
	if j.collideConnected == shouldCollide {
		return
	}

	j.collideConnected = shouldCollide
	bodyA := &w.bodies[j.edges[0].bodyId]
	bodyB := &w.bodies[j.edges[1].bodyId]

	if shouldCollide {
		typeA := bodyA.bodyType
		typeB := bodyB.bodyType

		shapeId := nullIndex
		if typeA == DynamicBody && typeB == DynamicBody {
			shapeId = bodyB.headShapeId
			if bodyA.shapeCount < bodyB.shapeCount {
				shapeId = bodyA.headShapeId
			}
		} else if typeA == DynamicBody {
			shapeId = bodyA.headShapeId
		} else if typeB == DynamicBody {
			shapeId = bodyB.headShapeId
		}

		for shapeId != nullIndex {
			s := &w.shapes[shapeId]
			if s.proxyKey != nullIndex {
				w.broadPhase.markProxyMovedSerial(s.proxyKey)
			}
			shapeId = s.nextShapeId
		}
		return
	}

	destroyContactsBetweenBodies(w, bodyA, bodyB)
}

// GetCollideConnected reports whether the attached bodies collide (b2Joint_GetCollideConnected).
func (jointId JointId) GetCollideConnected() bool {
	w := getWorld(jointId.world0)
	return getJointFullId(w, jointId).collideConnected
}

// SetUserData attaches application data to the joint (b2Joint_SetUserData).
func (jointId JointId) SetUserData(userData any) {
	w := getWorld(jointId.world0)
	getJointFullId(w, jointId).userData = userData
}

// GetUserData reports the data attached to the joint (b2Joint_GetUserData).
func (jointId JointId) GetUserData() any {
	w := getWorld(jointId.world0)
	return getJointFullId(w, jointId).userData
}

// WakeBodies wakes both bodies attached to the joint (b2Joint_WakeBodies).
func (jointId JointId) WakeBodies() {
	w := getWorldLocked(jointId.world0)
	j := getJointFullId(w, jointId)
	wakeBody(w, &w.bodies[j.edges[0].bodyId])
	wakeBody(w, &w.bodies[j.edges[1].bodyId])
}

// getJointReaction reports the force and the torque magnitudes of the last
// step. The joint events compare them with the thresholds. It corresponds
// to b2GetJointReaction in src/joint.c.
func getJointReaction(js *jointSim, invTimeStep Q) (force, torque Q) {
	linearImpulse := QZero()
	angularImpulse := QZero()

	switch js.jointType {
	case DistanceJoint:
		d := js.distance()
		linearImpulse = d.impulse.Add(d.lowerImpulse).Sub(d.upperImpulse).Add(d.motorImpulse).Abs()
	case MotorJoint:
		m := js.motor()
		linearImpulse = m.linearVelocityImpulse.Add(m.linearSpringImpulse).Len()
		angularImpulse = m.angularVelocityImpulse.Add(m.angularSpringImpulse).Abs()
	case MoverJoint:
		linearImpulse = js.mover().linearVelocityImpulse.Len()
	case PogoJoint:
		linearImpulse = js.pogo().impulse.Abs()
	case PrismaticJoint:
		p := js.prismatic()
		perpImpulse := p.impulse.X
		axialImpulse := p.motorImpulse.Add(p.lowerImpulse).Sub(p.upperImpulse)
		linearImpulse = perpImpulse.Mul(perpImpulse).Add(axialImpulse.Mul(axialImpulse)).Sqrt()
		angularImpulse = p.impulse.Y.Abs()
	case RevoluteJoint:
		r := js.revolute()
		linearImpulse = r.linearImpulse.Len()
		angularImpulse = r.motorImpulse.Add(r.lowerImpulse).Sub(r.upperImpulse).Abs()
	case WeldJoint:
		wj := js.weld()
		linearImpulse = wj.linearImpulse.Len()
		angularImpulse = wj.angularImpulse.Abs()
	case WheelJoint:
		wh := js.wheel()
		perpImpulse := wh.perpImpulse
		axialImpulse := wh.springImpulse.Add(wh.lowerImpulse).Sub(wh.upperImpulse)
		linearImpulse = perpImpulse.Mul(perpImpulse).Add(axialImpulse.Mul(axialImpulse)).Sqrt()
		angularImpulse = wh.motorImpulse.Abs()
	}

	return linearImpulse.Mul(invTimeStep), angularImpulse.Mul(invTimeStep)
}

// getJointConstraintForce corresponds to b2GetJointConstraintForce in
// src/joint.c.
func getJointConstraintForce(w *world, j *joint) Vec2 {
	base := getJointSim(w, j)

	switch j.jointType {
	case DistanceJoint:
		return getDistanceJointForce(w, base)
	case MotorJoint:
		return getMotorJointForce(w, base)
	case MoverJoint:
		return getMoverJointForce(w, base)
	case FilterJoint:
		return Vec2Zero()
	case PogoJoint:
		return getPogoJointForce(w, base)
	case PrismaticJoint:
		return getPrismaticJointForce(w, base)
	case RevoluteJoint:
		return getRevoluteJointForce(w, base)
	case WeldJoint:
		return getWeldJointForce(w, base)
	case WheelJoint:
		return getWheelJointForce(w, base)
	default:
		panic("dbox2d: unknown joint type")
	}
}

// getJointConstraintTorque corresponds to b2GetJointConstraintTorque in
// src/joint.c. The mover and the pogo joints apply no torque.
func getJointConstraintTorque(w *world, j *joint) Q {
	base := getJointSim(w, j)

	switch j.jointType {
	case DistanceJoint, FilterJoint, MoverJoint, PogoJoint:
		return QZero()
	case MotorJoint:
		return getMotorJointTorque(w, base)
	case PrismaticJoint:
		return getPrismaticJointTorque(w, base)
	case RevoluteJoint:
		return getRevoluteJointTorque(w, base)
	case WeldJoint:
		return getWeldJointTorque(w, base)
	case WheelJoint:
		return getWheelJointTorque(w, base)
	default:
		panic("dbox2d: unknown joint type")
	}
}

// GetConstraintForce reports the constraint force from the last step (b2Joint_GetConstraintForce).
func (jointId JointId) GetConstraintForce() Vec2 {
	w := getWorld(jointId.world0)
	return getJointConstraintForce(w, getJointFullId(w, jointId))
}

// GetConstraintTorque reports the constraint torque from the last step (b2Joint_GetConstraintTorque).
func (jointId JointId) GetConstraintTorque() Q {
	w := getWorld(jointId.world0)
	return getJointConstraintTorque(w, getJointFullId(w, jointId))
}

// GetLinearSeparation reports the joint's linear separation (b2Joint_GetLinearSeparation).
func (jointId JointId) GetLinearSeparation() Q {
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	base := getJointSim(w, j)

	xfA := getBodyTransform(w, j.edges[0].bodyId)
	xfB := getBodyTransform(w, j.edges[1].bodyId)
	pA := TransformPoint(xfA, base.localFrameA.P)
	pB := TransformPoint(xfB, base.localFrameB.P)
	dp := pB.Sub(pA)
	zero := QZero()

	switch j.jointType {
	case DistanceJoint:
		distance := base.distance()
		length := dp.Len()
		if distance.enableSpring {
			if distance.enableLimit {
				if length.Less(distance.minLength) {
					return distance.minLength.Sub(length)
				}
				if distance.maxLength.Less(length) {
					return length.Sub(distance.maxLength)
				}
			}
			return zero
		}
		return length.Sub(distance.length).Abs()
	case MotorJoint, MoverJoint, PogoJoint, FilterJoint:
		return zero
	case PrismaticJoint:
		prismatic := base.prismatic()
		// The reference takes the axis from the body rotation, not the frame.
		axisA := RotateVector(xfA.Q, Vec2{X: QOne(), Y: zero})
		perpendicularSeparation := LeftPerp(axisA).Dot(dp).Abs()
		limitSeparation := zero
		if prismatic.enableLimit {
			translation := axisA.Dot(dp)
			if translation.Less(prismatic.lowerTranslation) {
				limitSeparation = prismatic.lowerTranslation.Sub(translation)
			}
			if prismatic.upperTranslation.Less(translation) {
				limitSeparation = translation.Sub(prismatic.upperTranslation)
			}
		}
		return perpendicularSeparation.Mul(perpendicularSeparation).Add(limitSeparation.Mul(limitSeparation)).Sqrt()
	case RevoluteJoint:
		return dp.Len()
	case WeldJoint:
		if base.weld().linearHertz.Eq(zero) {
			return dp.Len()
		}
		return zero
	case WheelJoint:
		wheel := base.wheel()
		axisA := RotateVector(xfA.Q, Vec2{X: QOne(), Y: zero})
		perpendicularSeparation := LeftPerp(axisA).Dot(dp).Abs()
		limitSeparation := zero
		if wheel.enableLimit {
			translation := axisA.Dot(dp)
			if translation.Less(wheel.lowerTranslation) {
				limitSeparation = wheel.lowerTranslation.Sub(translation)
			}
			if wheel.upperTranslation.Less(translation) {
				limitSeparation = translation.Sub(wheel.upperTranslation)
			}
		}
		return perpendicularSeparation.Mul(perpendicularSeparation).Add(limitSeparation.Mul(limitSeparation)).Sqrt()
	default:
		panic("dbox2d: unknown joint type")
	}
}

// GetAngularSeparation reports angular separation in turns (b2Joint_GetAngularSeparation).
// As in the reference, the angle is the relative body rotation; the joint
// frames do not enter.
func (jointId JointId) GetAngularSeparation() Q {
	// D-004: the joint computes in its angle unit and reports turns.
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	base := getJointSim(w, j)

	xfA := getBodyTransform(w, j.edges[0].bodyId)
	xfB := getBodyTransform(w, j.edges[1].bodyId)
	relative := relativeAngle(xfA.Q, xfB.Q)
	zero := QZero()

	switch j.jointType {
	case DistanceJoint, MotorJoint, MoverJoint, PogoJoint, FilterJoint, WheelJoint:
		return zero
	case PrismaticJoint:
		return angleToTurns(relative)
	case RevoluteJoint:
		revolute := base.revolute()
		if revolute.enableLimit {
			angle := relative
			if angle.Less(revolute.lowerAngle) {
				return angleToTurns(revolute.lowerAngle.Sub(angle))
			}
			if revolute.upperAngle.Less(angle) {
				return angleToTurns(angle.Sub(revolute.upperAngle))
			}
		}
		return zero
	case WeldJoint:
		if base.weld().angularHertz.Eq(zero) {
			return angleToTurns(relative)
		}
		return zero
	default:
		panic("dbox2d: unknown joint type")
	}
}

// SetConstraintTuning changes the joint constraint tuning (b2Joint_SetConstraintTuning).
func (jointId JointId) SetConstraintTuning(hertz, dampingRatio Q) {
	zero := QZero()
	if !IsValidQ(hertz) || hertz.Less(zero) {
		panic("dbox2d: SetConstraintTuning hertz is invalid")
	}
	if !IsValidQ(dampingRatio) || dampingRatio.Less(zero) {
		panic("dbox2d: SetConstraintTuning damping ratio is invalid")
	}
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	base := getJointSim(w, j)
	base.constraintHertz = hertz
	base.constraintDampingRatio = dampingRatio
}

// GetConstraintTuning reports the joint constraint tuning (b2Joint_GetConstraintTuning).
func (jointId JointId) GetConstraintTuning() (hertz, dampingRatio Q) {
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	base := getJointSim(w, j)
	return base.constraintHertz, base.constraintDampingRatio
}

// SetForceThreshold changes the force that triggers a joint event (b2Joint_SetForceThreshold).
func (jointId JointId) SetForceThreshold(threshold Q) {
	if !isForceBound(threshold) || threshold.Less(QZero()) {
		panic("dbox2d: SetForceThreshold needs a valid non-negative threshold")
	}
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	getJointSim(w, j).forceThreshold = threshold
}

// GetForceThreshold reports the force that triggers a joint event (b2Joint_GetForceThreshold).
func (jointId JointId) GetForceThreshold() Q {
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	return getJointSim(w, j).forceThreshold
}

// SetTorqueThreshold changes the torque that triggers a joint event (b2Joint_SetTorqueThreshold).
func (jointId JointId) SetTorqueThreshold(threshold Q) {
	if !isForceBound(threshold) || threshold.Less(QZero()) {
		panic("dbox2d: SetTorqueThreshold needs a valid non-negative threshold")
	}
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	getJointSim(w, j).torqueThreshold = threshold
}

// GetTorqueThreshold reports the torque that triggers a joint event (b2Joint_GetTorqueThreshold).
func (jointId JointId) GetTorqueThreshold() Q {
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	return getJointSim(w, j).torqueThreshold
}

// validateJointDef panics on an invalid base definition (D-003). It
// corresponds to b2IsValidJointDef in src/joint.c.
func validateJointDef(def *JointDef) {
	zero := QZero()
	if !IsValidTransform(def.LocalFrameA) || !IsValidTransform(def.LocalFrameB) {
		panic("dbox2d: JointDef local frames must be valid transforms")
	}
	if !isForceBound(def.ForceThreshold) || def.ForceThreshold.Less(zero) {
		panic("dbox2d: JointDef.ForceThreshold must be non-negative")
	}
	if !isForceBound(def.TorqueThreshold) || def.TorqueThreshold.Less(zero) {
		panic("dbox2d: JointDef.TorqueThreshold must be non-negative")
	}
	if !IsValidQ(def.ConstraintHertz) || def.ConstraintHertz.Less(zero) {
		panic("dbox2d: JointDef.ConstraintHertz must be non-negative")
	}
	if !IsValidQ(def.ConstraintDampingRatio) || def.ConstraintDampingRatio.Less(zero) {
		panic("dbox2d: JointDef.ConstraintDampingRatio must be non-negative")
	}
	if !IsValidQ(def.DrawScale) {
		panic("dbox2d: JointDef.DrawScale must be valid")
	}
}

// validQ panics when a definition field is not a valid number (D-003).
func validQ(value Q, message string) {
	if !IsValidQ(value) {
		panic(message)
	}
}

// createJoint allocates a joint between the two bodies of a definition,
// links it into both body lists and picks its solver set. It corresponds
// to b2CreateJoint in src/joint.c.
func createJoint(worldId WorldId, def *JointDef, jointType JointType) (*world, jointPair) {
	validateJointDef(def)
	w := getWorldFromId(worldId)
	if w.locked {
		panic("dbox2d: the world is locked")
	}
	if !def.BodyIdA.IsValid() {
		panic("dbox2d: BodyIdA is not valid")
	}
	if !def.BodyIdB.IsValid() {
		panic("dbox2d: BodyIdB is not valid")
	}
	if def.BodyIdA.world0 != w.worldId || def.BodyIdB.world0 != w.worldId {
		panic("dbox2d: the joint bodies belong to another world")
	}
	if def.BodyIdA == def.BodyIdB {
		panic("dbox2d: a joint needs two different bodies")
	}
	bodyA := getBodyFullId(w, def.BodyIdA)
	bodyB := getBodyFullId(w, def.BodyIdB)

	// If the joint prevents collisions, then destroy all contacts between
	// attached bodies. Do this first so world arrays are not disturbed in
	// the following code.
	if !def.CollideConnected {
		destroyContactsBetweenBodies(w, bodyA, bodyB)
	}

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
	j.userData = def.UserData
	j.generation += 1
	j.setIndex = nullIndex
	j.colorIndex = nullIndex
	j.localIndex = nullIndex
	j.islandId = nullIndex
	j.islandIndex = nullIndex
	j.drawScale = def.DrawScale
	j.jointType = jointType
	j.collideConnected = def.CollideConnected

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

	case bodyA.bodyType != DynamicBody && bodyB.bodyType != DynamicBody:
		// joint is not attached to a dynamic body
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

		// These must be set to accommodate the merge below
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

	js.localFrameA = def.LocalFrameA
	js.localFrameB = def.LocalFrameB
	js.jointType = jointType
	js.constraintHertz = def.ConstraintHertz
	js.constraintDampingRatio = def.ConstraintDampingRatio
	js.constraintSoftness = softness{
		biasRate:     QZero(),
		massScale:    QOne(),
		impulseScale: QZero(),
	}
	js.forceThreshold = def.ForceThreshold
	js.torqueThreshold = def.TorqueThreshold

	if js.jointId != jointId || js.bodyIdA != bodyIdA || js.bodyIdB != bodyIdB {
		panic("dbox2d: the joint sim does not match the joint")
	}

	if j.setIndex > disabledSet {
		// Add edge to island graph
		linkJoint(w, j)
	}

	validateSolverSetsDebug(w)

	return w, jointPair{j, js}
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

	validateSolverSetsDebug(w)
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
	w, pair := createJoint(worldId, &def.Base, FilterJoint)
	return makeJointId(w, pair)
}

// CreateDistanceJoint creates a distance joint. The definition must come
// from DefaultDistanceJointDef.
func CreateDistanceJoint(worldId WorldId, def *DistanceJointDef) JointId {
	checkDef(def.internalValue)
	zero := QZero()
	if !IsValidQ(def.Length) || !zero.Less(def.Length) {
		panic("dbox2d: DistanceJointDef.Length must be positive")
	}
	validQ(def.Hertz, "dbox2d: DistanceJointDef.Hertz must be valid")
	validQ(def.DampingRatio, "dbox2d: DistanceJointDef.DampingRatio must be valid")
	validQ(def.MinLength, "dbox2d: DistanceJointDef.MinLength must be valid")
	validQ(def.MaxLength, "dbox2d: DistanceJointDef.MaxLength must be valid")
	validQ(def.MaxMotorForce, "dbox2d: DistanceJointDef.MaxMotorForce must be valid")
	validQ(def.MotorSpeed, "dbox2d: DistanceJointDef.MotorSpeed must be valid")
	if !isForceBound(def.LowerSpringForce) {
		panic("dbox2d: DistanceJointDef.LowerSpringForce must be valid")
	}
	if !isForceBound(def.UpperSpringForce) {
		panic("dbox2d: DistanceJointDef.UpperSpringForce must be valid")
	}
	if def.UpperSpringForce.Less(def.LowerSpringForce) {
		panic("dbox2d: DistanceJointDef.LowerSpringForce exceeds UpperSpringForce")
	}

	w, pair := createJoint(worldId, &def.Base, DistanceJoint)

	js := pair.jointSim
	*js.distance() = distanceJoint{}
	d := js.distance()
	d.length = def.Length.Max(linearSlop)
	d.hertz = def.Hertz
	d.dampingRatio = def.DampingRatio
	d.minLength = def.MinLength.Max(linearSlop)
	d.maxLength = def.MinLength.Max(def.MaxLength)
	d.maxMotorForce = def.MaxMotorForce
	d.motorSpeed = def.MotorSpeed
	d.enableSpring = def.EnableSpring
	d.lowerSpringForce = def.LowerSpringForce.Min(def.UpperSpringForce)
	d.upperSpringForce = def.LowerSpringForce.Max(def.UpperSpringForce)
	d.enableLimit = def.EnableLimit
	d.enableMotor = def.EnableMotor

	return makeJointId(w, pair)
}

// CreateMotorJoint creates a motor joint. The definition must come from
// DefaultMotorJointDef. The angular velocity is in turns per second.
func CreateMotorJoint(worldId WorldId, def *MotorJointDef) JointId {
	checkDef(def.internalValue)
	if !IsValidVec2(def.LinearVelocity) {
		panic("dbox2d: MotorJointDef.LinearVelocity must be valid")
	}
	validQ(def.MaxVelocityForce, "dbox2d: MotorJointDef.MaxVelocityForce must be valid")
	validQ(def.AngularVelocity, "dbox2d: MotorJointDef.AngularVelocity must be valid")
	validQ(def.MaxVelocityTorque, "dbox2d: MotorJointDef.MaxVelocityTorque must be valid")
	validQ(def.LinearHertz, "dbox2d: MotorJointDef.LinearHertz must be valid")
	validQ(def.LinearDampingRatio, "dbox2d: MotorJointDef.LinearDampingRatio must be valid")
	validQ(def.MaxSpringForce, "dbox2d: MotorJointDef.MaxSpringForce must be valid")
	validQ(def.AngularHertz, "dbox2d: MotorJointDef.AngularHertz must be valid")
	validQ(def.AngularDampingRatio, "dbox2d: MotorJointDef.AngularDampingRatio must be valid")
	validQ(def.MaxSpringTorque, "dbox2d: MotorJointDef.MaxSpringTorque must be valid")

	w, pair := createJoint(worldId, &def.Base, MotorJoint)

	js := pair.jointSim
	*js.motor() = motorJoint{}
	m := js.motor()
	m.linearVelocity = def.LinearVelocity
	m.maxVelocityForce = def.MaxVelocityForce
	m.angularVelocity = angleFromTurns(def.AngularVelocity)
	m.maxVelocityTorque = def.MaxVelocityTorque
	m.linearHertz = def.LinearHertz
	m.linearDampingRatio = def.LinearDampingRatio
	m.maxSpringForce = def.MaxSpringForce
	m.angularHertz = def.AngularHertz
	m.angularDampingRatio = def.AngularDampingRatio
	m.maxSpringTorque = def.MaxSpringTorque

	return makeJointId(w, pair)
}

// CreateMoverJoint creates a mover joint. The definition must come from
// DefaultMoverJointDef.
func CreateMoverJoint(worldId WorldId, def *MoverJointDef) JointId {
	checkDef(def.internalValue)
	if !IsValidVec2(def.LinearVelocity) {
		panic("dbox2d: MoverJointDef.LinearVelocity must be valid")
	}
	if !IsValidVec2(def.MaxVelocityForce) {
		panic("dbox2d: MoverJointDef.MaxVelocityForce must be valid")
	}

	w, pair := createJoint(worldId, &def.Base, MoverJoint)

	js := pair.jointSim
	*js.mover() = moverJoint{}
	js.mover().linearVelocity = def.LinearVelocity
	js.mover().maxVelocityForce = def.MaxVelocityForce

	return makeJointId(w, pair)
}

// CreatePogoJoint creates a pogo joint. The definition must come from
// DefaultPogoJointDef.
func CreatePogoJoint(worldId WorldId, def *PogoJointDef) JointId {
	checkDef(def.internalValue)
	zero := QZero()
	if !IsValidVec2(def.Normal) {
		panic("dbox2d: PogoJointDef.Normal must be valid")
	}
	for _, field := range []struct {
		value Q
		name  string
	}{
		{def.RestLength, "RestLength"},
		{def.Hertz, "Hertz"},
		{def.DampingRatio, "DampingRatio"},
		{def.MaxTensionForce, "MaxTensionForce"},
		{def.MaxCompressionForce, "MaxCompressionForce"},
	} {
		if !IsValidQ(field.value) || field.value.Less(zero) {
			panic("dbox2d: PogoJointDef." + field.name + " must be non-negative")
		}
	}
	validQ(def.Impulse, "dbox2d: PogoJointDef.Impulse must be valid")
	validQ(def.Velocity, "dbox2d: PogoJointDef.Velocity must be valid")

	w, pair := createJoint(worldId, &def.Base, PogoJoint)

	js := pair.jointSim
	*js.pogo() = pogoJoint{}
	p := js.pogo()
	p.restLength = def.RestLength
	p.hertz = def.Hertz
	p.dampingRatio = def.DampingRatio
	p.maxTensionForce = def.MaxTensionForce
	p.maxCompressionForce = def.MaxCompressionForce
	p.normal = def.Normal
	p.impulse = def.Impulse
	p.velocity = def.Velocity

	return makeJointId(w, pair)
}

// CreatePrismaticJoint creates a prismatic joint. The definition must come
// from DefaultPrismaticJointDef. The joint slides along the x-axis of
// LocalFrameA.
func CreatePrismaticJoint(worldId WorldId, def *PrismaticJointDef) JointId {
	checkDef(def.internalValue)
	validQ(def.Hertz, "dbox2d: PrismaticJointDef.Hertz must be valid")
	validQ(def.DampingRatio, "dbox2d: PrismaticJointDef.DampingRatio must be valid")
	validQ(def.TargetTranslation, "dbox2d: PrismaticJointDef.TargetTranslation must be valid")
	validQ(def.LowerTranslation, "dbox2d: PrismaticJointDef.LowerTranslation must be valid")
	validQ(def.UpperTranslation, "dbox2d: PrismaticJointDef.UpperTranslation must be valid")
	validQ(def.MaxMotorForce, "dbox2d: PrismaticJointDef.MaxMotorForce must be valid")
	validQ(def.MotorSpeed, "dbox2d: PrismaticJointDef.MotorSpeed must be valid")
	if def.UpperTranslation.Less(def.LowerTranslation) {
		panic("dbox2d: PrismaticJointDef.LowerTranslation exceeds UpperTranslation")
	}

	w, pair := createJoint(worldId, &def.Base, PrismaticJoint)

	js := pair.jointSim
	*js.prismatic() = prismaticJoint{}
	p := js.prismatic()
	p.hertz = def.Hertz
	p.dampingRatio = def.DampingRatio
	p.targetTranslation = def.TargetTranslation
	p.lowerTranslation = def.LowerTranslation.Min(def.UpperTranslation)
	p.upperTranslation = def.LowerTranslation.Max(def.UpperTranslation)
	p.maxMotorForce = def.MaxMotorForce
	p.motorSpeed = def.MotorSpeed
	p.enableSpring = def.EnableSpring
	p.enableLimit = def.EnableLimit
	p.enableMotor = def.EnableMotor

	return makeJointId(w, pair)
}

// CreateRevoluteJoint creates a revolute joint. The definition must come
// from DefaultRevoluteJointDef. The angles are in turns.
func CreateRevoluteJoint(worldId WorldId, def *RevoluteJointDef) JointId {
	checkDef(def.internalValue)
	validQ(def.TargetAngle, "dbox2d: RevoluteJointDef.TargetAngle must be valid")
	validQ(def.Hertz, "dbox2d: RevoluteJointDef.Hertz must be valid")
	validQ(def.DampingRatio, "dbox2d: RevoluteJointDef.DampingRatio must be valid")
	validQ(def.LowerAngle, "dbox2d: RevoluteJointDef.LowerAngle must be valid")
	validQ(def.UpperAngle, "dbox2d: RevoluteJointDef.UpperAngle must be valid")
	validQ(def.MaxMotorTorque, "dbox2d: RevoluteJointDef.MaxMotorTorque must be valid")
	validQ(def.MotorSpeed, "dbox2d: RevoluteJointDef.MotorSpeed must be valid")
	if def.UpperAngle.Less(def.LowerAngle) {
		panic("dbox2d: RevoluteJointDef.LowerAngle exceeds UpperAngle")
	}

	w, pair := createJoint(worldId, &def.Base, RevoluteJoint)

	js := pair.jointSim
	*js.revolute() = revoluteJoint{}
	r := js.revolute()

	// D-004: the reference bounds are +/- pi and +/- 0.99 pi; in turns they
	// are +/- 0.5 and +/- 0.495.
	halfTurn := QHalf()
	r.targetAngle = angleFromTurns(def.TargetAngle.Clamp(halfTurn.Neg(), halfTurn))
	r.hertz = def.Hertz
	r.dampingRatio = def.DampingRatio

	lowerAngle := def.LowerAngle.Min(def.UpperAngle)
	upperAngle := def.LowerAngle.Max(def.UpperAngle)
	r.lowerAngle = angleFromTurns(lowerAngle.Clamp(revoluteLimitAngle.Neg(), revoluteLimitAngle))
	r.upperAngle = angleFromTurns(upperAngle.Clamp(revoluteLimitAngle.Neg(), revoluteLimitAngle))

	r.maxMotorTorque = def.MaxMotorTorque
	r.motorSpeed = angleFromTurns(def.MotorSpeed)
	r.enableSpring = def.EnableSpring
	r.enableLimit = def.EnableLimit
	r.enableMotor = def.EnableMotor

	return makeJointId(w, pair)
}

// CreateWeldJoint creates a weld joint. The definition must come from
// DefaultWeldJointDef.
func CreateWeldJoint(worldId WorldId, def *WeldJointDef) JointId {
	checkDef(def.internalValue)
	validQ(def.LinearHertz, "dbox2d: WeldJointDef.LinearHertz must be valid")
	validQ(def.AngularHertz, "dbox2d: WeldJointDef.AngularHertz must be valid")
	validQ(def.LinearDampingRatio, "dbox2d: WeldJointDef.LinearDampingRatio must be valid")
	validQ(def.AngularDampingRatio, "dbox2d: WeldJointDef.AngularDampingRatio must be valid")

	w, pair := createJoint(worldId, &def.Base, WeldJoint)

	js := pair.jointSim
	*js.weld() = weldJoint{}
	wj := js.weld()
	wj.linearHertz = def.LinearHertz
	wj.linearDampingRatio = def.LinearDampingRatio
	wj.angularHertz = def.AngularHertz
	wj.angularDampingRatio = def.AngularDampingRatio

	return makeJointId(w, pair)
}

// CreateWheelJoint creates a wheel joint. The definition must come from
// DefaultWheelJointDef. The wheel axis is the x-axis of LocalFrameA.
func CreateWheelJoint(worldId WorldId, def *WheelJointDef) JointId {
	checkDef(def.internalValue)
	validQ(def.Hertz, "dbox2d: WheelJointDef.Hertz must be valid")
	validQ(def.DampingRatio, "dbox2d: WheelJointDef.DampingRatio must be valid")
	validQ(def.LowerTranslation, "dbox2d: WheelJointDef.LowerTranslation must be valid")
	validQ(def.UpperTranslation, "dbox2d: WheelJointDef.UpperTranslation must be valid")
	validQ(def.MaxMotorTorque, "dbox2d: WheelJointDef.MaxMotorTorque must be valid")
	validQ(def.MotorSpeed, "dbox2d: WheelJointDef.MotorSpeed must be valid")
	if def.UpperTranslation.Less(def.LowerTranslation) {
		panic("dbox2d: WheelJointDef.LowerTranslation exceeds UpperTranslation")
	}

	w, pair := createJoint(worldId, &def.Base, WheelJoint)

	js := pair.jointSim
	*js.wheel() = wheelJoint{}
	wh := js.wheel()
	wh.lowerTranslation = def.LowerTranslation.Min(def.UpperTranslation)
	wh.upperTranslation = def.LowerTranslation.Max(def.UpperTranslation)
	wh.maxMotorTorque = def.MaxMotorTorque
	wh.motorSpeed = angleFromTurns(def.MotorSpeed)
	wh.hertz = def.Hertz
	wh.dampingRatio = def.DampingRatio
	wh.enableSpring = def.EnableSpring
	wh.enableLimit = def.EnableLimit
	wh.enableMotor = def.EnableMotor

	return makeJointId(w, pair)
}

// destroyJointInternal unlinks a joint from its bodies, its island and its
// set, then frees the id. It corresponds to b2DestroyJointInternal in
// src/joint.c.
func destroyJointInternal(w *world, j *joint) {
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
		set.jointSims, movedIndex = removeSwapNoClear(set.jointSims, localIndex)
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

	wakeBody(w, bodyA)
	wakeBody(w, bodyB)

	validateSolverSetsDebug(w)
}

// DestroyJoint destroys a joint and wakes the two bodies.
func DestroyJoint(jointId JointId) {
	w := getWorldLocked(jointId.world0)

	j := getJointFullId(w, jointId)

	destroyJointInternal(w, j)
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
	hertz := js.constraintHertz.Min(context.invH.Div(QFromInt(4)))
	js.constraintSoftness = makeSoft(hertz, js.constraintDampingRatio, context.h)

	switch js.jointType {
	case FilterJoint:
	case DistanceJoint:
		prepareDistanceJoint(js, context)
	case PrismaticJoint:
		preparePrismaticJoint(js, context)
	case WheelJoint:
		prepareWheelJoint(js, context)
	case WeldJoint:
		prepareWeldJoint(js, context)
	case MotorJoint:
		prepareMotorJoint(js, context)
	case MoverJoint:
		prepareMoverJoint(js, context)
	case PogoJoint:
		preparePogoJoint(js, context)
	case RevoluteJoint:
		prepareRevoluteJoint(js, context)
	default:
		panic("dbox2d: unknown joint type")
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
	case WheelJoint:
		warmStartWheelJoint(js, context)
	case WeldJoint:
		warmStartWeldJoint(js, context)
	case MotorJoint:
		warmStartMotorJoint(js, context)
	case MoverJoint:
		warmStartMoverJoint(js, context)
	case PogoJoint:
		warmStartPogoJoint(js, context)
	case RevoluteJoint:
		warmStartRevoluteJoint(js, context)
	default:
		panic("dbox2d: unknown joint type")
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
	case WheelJoint:
		solveWheelJoint(js, context, useBias)
	case WeldJoint:
		solveWeldJoint(js, context, useBias)
	case MotorJoint:
		solveMotorJoint(js, context)
	case MoverJoint:
		solveMoverJoint(js, context)
	case PogoJoint:
		solvePogoJoint(js, context, useBias)
	case RevoluteJoint:
		solveRevoluteJoint(js, context, useBias)
	default:
		panic("dbox2d: unknown joint type")
	}
}

// prepareJointsTask prepares a range of the flat joint array. It
// corresponds to b2PrepareJointsTask in src/solver.c.
func prepareJointsTask(startIndex, endIndex int, context *stepContext) {
	prepareJointRange(startIndex, endIndex, context, context.joints, nil)
}

// prepareOverflowJoints prepares the overflow joints. It corresponds to
// b2PrepareOverflowJoints in src/joint.c.
func prepareOverflowJoints(context *stepContext) {
	joints := context.graph.colors[overflowIndex].jointSims
	prepareJointRange(0, len(joints), context, nil, joints)
}

func prepareJointRange(startIndex, endIndex int, context *stepContext, jointPointers []*jointSim, joints []jointSim) {
	if jointPointers != nil {
		jointPointers = jointPointers[startIndex:endIndex]
	} else {
		joints = joints[startIndex:endIndex]
	}

	for i := range endIndex - startIndex {
		if jointPointers != nil {
			prepareJoint(jointPointers[i], context)
		} else {
			prepareJoint(&joints[i], context)
		}
	}
}

// warmStartJointsTask applies stored impulses to a joint range. It
// corresponds to b2WarmStartJointsTask in src/solver.c.
func warmStartJointsTask(startIndex, endIndex int, context *stepContext, colorIndex int) {
	joints := context.graph.colors[colorIndex].jointSims
	warmStartJointRange(startIndex, endIndex, context, joints)
}

// warmStartOverflowJoints applies stored impulses to the overflow joints.
// It corresponds to b2WarmStartOverflowJoints in src/joint.c.
func warmStartOverflowJoints(context *stepContext) {
	joints := context.graph.colors[overflowIndex].jointSims
	warmStartJointRange(0, len(joints), context, joints)
}

func warmStartJointRange(startIndex, endIndex int, context *stepContext, joints []jointSim) {
	joints = joints[startIndex:endIndex]
	for i := range endIndex - startIndex {
		warmStartJoint(&joints[i], context)
	}
}

// solveJointsTask solves a joint range and flags the joints whose reaction
// crosses a threshold. It corresponds to b2SolveJointsTask in
// src/joint.c.
func solveJointsTask(startIndex, endIndex int, context *stepContext, colorIndex int, useBias bool, workerIndex int) {
	joints := context.graph.colors[colorIndex].jointSims
	solveJointRange(startIndex, endIndex, context, joints, useBias)
	if !useBias {
		return
	}

	jointStateBitSet := &context.world.taskContexts[workerIndex].jointStateBitSet
	maxValue := QMaxValue()
	for i := startIndex; i < endIndex; i++ {
		js := &joints[i]
		if !js.forceThreshold.Less(maxValue) && !js.torqueThreshold.Less(maxValue) {
			continue
		}
		if jointStateBitSet.getBit(js.jointId) {
			continue
		}

		force, torque := getJointReaction(js, context.invH)

		// Check thresholds. A zero threshold means all awake joints get
		// reported.
		if !force.Less(js.forceThreshold) || !torque.Less(js.torqueThreshold) {
			// Flag this joint for processing.
			jointStateBitSet.setBit(js.jointId)
		}
	}
}

// solveOverflowJoints solves the overflow joints. It corresponds to
// b2SolveOverflowJoints in src/joint.c.
func solveOverflowJoints(context *stepContext, useBias bool) {
	joints := context.graph.colors[overflowIndex].jointSims
	solveJointRange(0, len(joints), context, joints, useBias)
}

func solveJointRange(startIndex, endIndex int, context *stepContext, joints []jointSim, useBias bool) {
	joints = joints[startIndex:endIndex]
	for i := range endIndex - startIndex {
		solveJoint(&joints[i], context, useBias)
	}
}

// DefaultExplosionDef returns an explosion definition with every mask bit
// set and every other field zero. It corresponds to b2DefaultExplosionDef.
func DefaultExplosionDef() ExplosionDef {
	return ExplosionDef{MaskBits: DefaultMaskBits}
}

// prepareJointBodies loads the inverse masses of the two bodies into the
// joint and returns their sims and their awake state indices. A body
// outside the awake set gets the null index. It is the preamble every
// b2Prepare*Joint of the reference repeats.
func prepareJointBodies(base *jointSim, context *stepContext) (bodySimA, bodySimB *bodySim, indexA, indexB int) {
	w := context.world
	bodyA := &w.bodies[base.bodyIdA]
	bodyB := &w.bodies[base.bodyIdB]
	if bodyA.setIndex != awakeSet && bodyB.setIndex != awakeSet {
		panic("dbox2d: neither body of the joint is awake")
	}
	bodySimA = &w.solverSets[bodyA.setIndex].bodySims[bodyA.localIndex]
	bodySimB = &w.solverSets[bodyB.setIndex].bodySims[bodyB.localIndex]

	base.invMassA = bodySimA.invMass
	base.invMassB = bodySimB.invMass
	base.invIA = bodySimA.invInertia
	base.invIB = bodySimB.invInertia

	indexA = nullIndex
	if bodyA.setIndex == awakeSet {
		indexA = bodyA.localIndex
	}
	indexB = nullIndex
	if bodyB.setIndex == awakeSet {
		indexB = bodyB.localIndex
	}
	return bodySimA, bodySimB, indexA, indexB
}

// jointFrame computes a joint frame with the world rotation, relative to
// the center of mass: rot(xf.q, f.p - lc) avoids the round-off of
// (xf.p - c) + rot(xf.q, f.p).
func jointFrame(sim *bodySim, localFrame Transform) Transform {
	return Transform{
		P: RotateVector(sim.transform.Q, localFrame.P.Sub(sim.localCenter)),
		Q: MulRot(sim.transform.Q, localFrame.Q),
	}
}

// storeJointVelocity writes a solved velocity back to a body state. Only a
// dynamic body takes it: a kinematic body keeps its velocity and the dummy
// state of a static body stays at rest.
func storeJointVelocity(state *bodyState, v Vec2, w Q) {
	if state.flags&bodyDynamicFlag != 0 {
		state.linearVelocity = v
		state.angularVelocity = w
	}
}
