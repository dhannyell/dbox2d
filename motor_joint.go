package dbox2d

import "github.com/dhannyell/fixed"

// This file corresponds to src/motor_joint.c of the reference. The angles
// are turns (D-004); the angular separation enters the bias in radians.

// getMotorJointForce reports the constraint force of the last step. It
// corresponds to b2GetMotorJointForce in src/motor_joint.c.
func getMotorJointForce(w *world, base *jointSim) Vec2 {
	force := base.motorJoint.linearImpulse.Mul(w.invH)
	return force
}

// getMotorJointTorque reports the constraint torque of the last step. It
// corresponds to b2GetMotorJointTorque in src/motor_joint.c.
func getMotorJointTorque(w *world, base *jointSim) Q {
	return w.invH.Mul(base.motorJoint.angularImpulse)
}

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Angle constraint
// C = angle2 - angle1 - referenceAngle
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

// prepareMotorJoint corresponds to b2PrepareMotorJoint in
// src/motor_joint.c.
func prepareMotorJoint(base *jointSim, context *stepContext) {
	if base.jointType != MotorJoint {
		panic("dbox2d: the joint is not a motor joint")
	}

	// chase body id to the solver set where the body lives
	idA := base.bodyIdA
	idB := base.bodyIdB

	w := context.world

	bodyA := &w.bodies[idA]
	bodyB := &w.bodies[idB]

	if bodyA.setIndex != awakeSet && bodyB.setIndex != awakeSet {
		panic("dbox2d: neither body of the joint is awake")
	}

	setA := &w.solverSets[bodyA.setIndex]
	setB := &w.solverSets[bodyB.setIndex]

	localIndexA := bodyA.localIndex
	localIndexB := bodyB.localIndex

	bodySimA := &setA.bodySims[localIndexA]
	bodySimB := &setB.bodySims[localIndexB]

	mA := bodySimA.invMass
	iA := bodySimA.invInertia
	mB := bodySimB.invMass
	iB := bodySimB.invInertia

	base.invMassA = mA
	base.invMassB = mB
	base.invIA = iA
	base.invIB = iB

	joint := &base.motorJoint
	joint.indexA = nullIndex
	if bodyA.setIndex == awakeSet {
		joint.indexA = localIndexA
	}
	joint.indexB = nullIndex
	if bodyB.setIndex == awakeSet {
		joint.indexB = localIndexB
	}

	joint.anchorA = RotateVector(bodySimA.transform.Q, base.localOriginAnchorA.Sub(bodySimA.localCenter))
	joint.anchorB = RotateVector(bodySimB.transform.Q, base.localOriginAnchorB.Sub(bodySimB.localCenter))
	joint.deltaCenter = bodySimB.center.Sub(bodySimA.center).Sub(joint.linearOffset)
	joint.deltaAngle = RelativeAngle(bodySimB.transform.Q, bodySimA.transform.Q).Sub(joint.angularOffset)

	rA := joint.anchorA
	rB := joint.anchorB

	var K Mat22
	K.Cx.X = mA.Add(mB).Add(rA.Y.Mul(rA.Y).Mul(iA)).Add(rB.Y.Mul(rB.Y).Mul(iB))
	K.Cx.Y = rA.Y.Neg().Mul(rA.X).Mul(iA).Sub(rB.Y.Mul(rB.X).Mul(iB))
	K.Cy.X = K.Cx.Y
	K.Cy.Y = mA.Add(mB).Add(rA.X.Mul(rA.X).Mul(iA)).Add(rB.X.Mul(rB.X).Mul(iB))
	joint.linearMass = GetInverse22(K)

	zero := fixed.Q32Zero()
	ka := iA.Add(iB)
	// D-006: the reference multiplies by the reciprocal of ka.
	joint.angularMass = zero
	if zero.Less(ka) {
		joint.angularMass = fixed.Q32One().Div(ka)
	}

	if !context.enableWarmStarting {
		joint.linearImpulse = Vec2Zero()
		joint.angularImpulse = zero
	}
}

// warmStartMotorJoint corresponds to b2WarmStartMotorJoint in
// src/motor_joint.c.
func warmStartMotorJoint(base *jointSim, context *stepContext) {
	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	joint := &base.motorJoint

	// dummy state for static bodies
	dummyState := identityBodyState()

	bodyA, bodyB := jointStates(context.states, &dummyState, joint.indexA, joint.indexB)

	rA := RotateVector(bodyA.deltaRotation, joint.anchorA)
	rB := RotateVector(bodyB.deltaRotation, joint.anchorB)

	// D-004: the angular velocity of the state is turns per second.
	bodyA.linearVelocity = MulSub(bodyA.linearVelocity, mA, joint.linearImpulse)
	wA := bodyA.angularVelocity.Mul(tau)
	wA = wA.Sub(iA.Mul(Cross(rA, joint.linearImpulse).Add(joint.angularImpulse)))
	bodyA.angularVelocity = wA.Div(tau)

	bodyB.linearVelocity = MulAdd(bodyB.linearVelocity, mB, joint.linearImpulse)
	wB := bodyB.angularVelocity.Mul(tau)
	wB = wB.Add(iB.Mul(Cross(rB, joint.linearImpulse).Add(joint.angularImpulse)))
	bodyB.angularVelocity = wB.Div(tau)
}

// solveMotorJoint corresponds to b2SolveMotorJoint in src/motor_joint.c.
// The reference ignores useBias.
func solveMotorJoint(base *jointSim, context *stepContext, _ bool) {
	if base.jointType != MotorJoint {
		panic("dbox2d: the joint is not a motor joint")
	}

	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState()

	joint := &base.motorJoint
	bodyA, bodyB := jointStates(context.states, &dummyState, joint.indexA, joint.indexB)

	vA := bodyA.linearVelocity
	wA := bodyA.angularVelocity.Mul(tau)
	vB := bodyB.linearVelocity
	wB := bodyB.angularVelocity.Mul(tau)

	// angular constraint
	{
		angularSeparation := RelativeAngle(bodyB.deltaRotation, bodyA.deltaRotation).Add(joint.deltaAngle)
		angularSeparation = UnwindAngle(angularSeparation)

		// D-004: the separation enters the bias in radians.
		angularBias := context.invH.Mul(joint.correctionFactor).Mul(angularSeparation.Mul(tau))

		Cdot := wB.Sub(wA)
		impulse := joint.angularMass.Neg().Mul(Cdot.Add(angularBias))

		oldImpulse := joint.angularImpulse
		maxImpulse := context.h.Mul(joint.maxTorque)
		joint.angularImpulse = joint.angularImpulse.Add(impulse).Clamp(maxImpulse.Neg(), maxImpulse)
		impulse = joint.angularImpulse.Sub(oldImpulse)

		wA = wA.Sub(iA.Mul(impulse))
		wB = wB.Add(iB.Mul(impulse))
	}

	// linear constraint
	{
		rA := RotateVector(bodyA.deltaRotation, joint.anchorA)
		rB := RotateVector(bodyB.deltaRotation, joint.anchorB)

		ds := bodyB.deltaPosition.Sub(bodyA.deltaPosition).Add(rB.Sub(rA))
		linearSeparation := joint.deltaCenter.Add(ds)
		linearBias := linearSeparation.Mul(context.invH.Mul(joint.correctionFactor))

		Cdot := vB.Add(CrossSV(wB, rB)).Sub(vA.Add(CrossSV(wA, rA)))
		b := MulMV(joint.linearMass, Cdot.Add(linearBias))
		impulse := Vec2{X: b.X.Neg(), Y: b.Y.Neg()}

		oldImpulse := joint.linearImpulse
		maxImpulse := context.h.Mul(joint.maxForce)
		joint.linearImpulse = joint.linearImpulse.Add(impulse)

		if maxImpulse.Mul(maxImpulse).Less(joint.linearImpulse.LenSq()) {
			joint.linearImpulse = joint.linearImpulse.Normalize()
			joint.linearImpulse.X = joint.linearImpulse.X.Mul(maxImpulse)
			joint.linearImpulse.Y = joint.linearImpulse.Y.Mul(maxImpulse)
		}

		impulse = joint.linearImpulse.Sub(oldImpulse)

		vA = MulSub(vA, mA, impulse)
		wA = wA.Sub(iA.Mul(Cross(rA, impulse)))
		vB = MulAdd(vB, mB, impulse)
		wB = wB.Add(iB.Mul(Cross(rB, impulse)))
	}

	bodyA.linearVelocity = vA
	bodyA.angularVelocity = wA.Div(tau)
	bodyB.linearVelocity = vB
	bodyB.angularVelocity = wB.Div(tau)
}

// SetLinearOffset changes the motor joint's linear offset (b2MotorJoint_SetLinearOffset).
func (jointId JointId) SetLinearOffset(linearOffset Vec2) {
	if !IsValidVec2(linearOffset) {
		panic("dbox2d: SetLinearOffset needs a valid vector")
	}
	w := getWorld(jointId.world0)
	js := getJointSimCheckType(w, jointId, MotorJoint)
	js.motorJoint.linearOffset = linearOffset
}

// GetLinearOffset reports the motor joint's linear offset (b2MotorJoint_GetLinearOffset).
func (jointId JointId) GetLinearOffset() Vec2 {
	w := getWorld(jointId.world0)
	js := getJointSimCheckType(w, jointId, MotorJoint)
	return js.motorJoint.linearOffset
}

// SetAngularOffset changes the motor joint's angular offset in turns (b2MotorJoint_SetAngularOffset).
func (jointId JointId) SetAngularOffset(angularOffset Q) {
	if !IsValidQ(angularOffset) {
		panic("dbox2d: SetAngularOffset needs a valid angle")
	}
	w := getWorld(jointId.world0)
	js := getJointSimCheckType(w, jointId, MotorJoint)
	halfTurn := fixed.Q32Half()
	// D-004: the port bounds turns to a half turn; the reference leaves radians unbounded.
	angularOffset = angularOffset.Clamp(halfTurn.Neg(), halfTurn) // D-004
	js.motorJoint.angularOffset = angularOffset
}

// GetAngularOffset reports the motor joint's angular offset in turns (b2MotorJoint_GetAngularOffset).
func (jointId JointId) GetAngularOffset() Q {
	w := getWorld(jointId.world0)
	js := getJointSimCheckType(w, jointId, MotorJoint)
	return js.motorJoint.angularOffset
}

// SetMaxForce changes the maximum force of a motor or mouse joint (b2MotorJoint_SetMaxForce, b2MouseJoint_SetMaxForce).
func (jointId JointId) SetMaxForce(maxForce Q) {
	if !IsValidQ(maxForce) {
		panic("dbox2d: SetMaxForce needs a valid value")
	}
	zero := fixed.Q32Zero()
	if maxForce.Less(zero) {
		maxForce = zero
	}
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	js := getJointSim(w, j)
	switch j.jointType {
	case MotorJoint:
		js.motorJoint.maxForce = maxForce
	case MouseJoint:
		js.mouseJoint.maxForce = maxForce
	default:
		panic("dbox2d: SetMaxForce needs a motor or mouse joint")
	}
}

// GetMaxForce reports the maximum force of a motor or mouse joint (b2MotorJoint_GetMaxForce, b2MouseJoint_GetMaxForce).
func (jointId JointId) GetMaxForce() Q {
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	js := getJointSim(w, j)
	switch j.jointType {
	case MotorJoint:
		return js.motorJoint.maxForce
	case MouseJoint:
		return js.mouseJoint.maxForce
	default:
		panic("dbox2d: GetMaxForce needs a motor or mouse joint")
	}
}

// SetMaxTorque changes the motor joint's maximum torque (b2MotorJoint_SetMaxTorque).
func (jointId JointId) SetMaxTorque(maxTorque Q) {
	if !IsValidQ(maxTorque) {
		panic("dbox2d: SetMaxTorque needs a valid value")
	}
	zero := fixed.Q32Zero()
	if maxTorque.Less(zero) {
		maxTorque = zero
	}
	w := getWorld(jointId.world0)
	js := getJointSimCheckType(w, jointId, MotorJoint)
	js.motorJoint.maxTorque = maxTorque
}

// GetMaxTorque reports the motor joint's maximum torque (b2MotorJoint_GetMaxTorque).
func (jointId JointId) GetMaxTorque() Q {
	w := getWorld(jointId.world0)
	js := getJointSimCheckType(w, jointId, MotorJoint)
	return js.motorJoint.maxTorque
}

// SetCorrectionFactor changes the motor joint's correction factor (b2MotorJoint_SetCorrectionFactor).
func (jointId JointId) SetCorrectionFactor(correctionFactor Q) {
	if !IsValidQ(correctionFactor) {
		panic("dbox2d: SetCorrectionFactor needs a valid value")
	}
	w := getWorld(jointId.world0)
	js := getJointSimCheckType(w, jointId, MotorJoint)
	js.motorJoint.correctionFactor = correctionFactor.Clamp(fixed.Q32Zero(), fixed.Q32One())
}

// GetCorrectionFactor reports the motor joint's correction factor (b2MotorJoint_GetCorrectionFactor).
func (jointId JointId) GetCorrectionFactor() Q {
	w := getWorld(jointId.world0)
	js := getJointSimCheckType(w, jointId, MotorJoint)
	return js.motorJoint.correctionFactor
}
