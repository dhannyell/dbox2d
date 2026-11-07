package b2

// This file corresponds to src/motor_joint.c of the reference. The motor
// joint drives body B relative to body A with a velocity constraint and a
// spring, each with its own force and torque limit. The API takes the
// angular velocity in turns per second (D-004); the joint keeps it in the
// angle unit of the mode.

// getMotorJointForce reports the constraint force of the last step. It
// corresponds to b2GetMotorJointForce in src/motor_joint.c.
func getMotorJointForce(w *world, base *jointSim) Vec2 {
	joint := base.motor()
	return joint.linearVelocityImpulse.Add(joint.linearSpringImpulse).Mul(w.invH)
}

// getMotorJointTorque reports the constraint torque of the last step. It
// corresponds to b2GetMotorJointTorque in src/motor_joint.c.
func getMotorJointTorque(w *world, base *jointSim) Q {
	joint := base.motor()
	return w.invH.Mul(joint.angularVelocityImpulse.Add(joint.angularSpringImpulse))
}

// motorLinearMass returns the inverse of the point constraint mass.
func motorLinearMass(mA, mB, iA, iB Q, rA, rB Vec2) Mat22 {
	var kl Mat22
	kl.Cx.X = mA.Add(mB).Add(rA.Y.Mul(rA.Y).Mul(iA)).Add(rB.Y.Mul(rB.Y).Mul(iB))
	kl.Cx.Y = rA.Y.Neg().Mul(rA.X).Mul(iA).Sub(rB.Y.Mul(rB.X).Mul(iB))
	kl.Cy.X = kl.Cx.Y
	kl.Cy.Y = mA.Add(mB).Add(rA.X.Mul(rA.X).Mul(iA)).Add(rB.X.Mul(rB.X).Mul(iB))
	return GetInverse22(kl)
}

// clampImpulseLength scales an accumulated impulse down to maxImpulse.
func clampImpulseLength(impulse Vec2, maxImpulse Q) Vec2 {
	if maxImpulse.Mul(maxImpulse).Less(impulse.LenSq()) {
		impulse = impulse.Normalize()
		impulse.X = impulse.X.Mul(maxImpulse)
		impulse.Y = impulse.Y.Mul(maxImpulse)
	}
	return impulse
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
	bodySimA, bodySimB, indexA, indexB := prepareJointBodies(base, context)
	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	joint := base.motor()
	joint.indexA = indexA
	joint.indexB = indexB

	joint.frameA = jointFrame(bodySimA, base.localFrameA)
	joint.frameB = jointFrame(bodySimB, base.localFrameB)

	// Compute the initial center delta. Incremental position updates are
	// relative to this.
	joint.deltaCenter = SubPos(bodySimB.center, bodySimA.center)

	rA := joint.frameA.P
	rB := joint.frameB.P

	joint.linearSpring = makeSoft(joint.linearHertz, joint.linearDampingRatio, context.h)
	joint.angularSpring = makeSoft(joint.angularHertz, joint.angularDampingRatio, context.h)

	joint.linearMass = motorLinearMass(mA, mB, iA, iB, rA, rB)

	zero := QZero()
	ka := iA.Add(iB)
	// D-006: the reference multiplies by the reciprocal of ka.
	joint.angularMass = zero
	if zero.Less(ka) {
		joint.angularMass = QOne().Div(ka)
	}

	if !context.enableWarmStarting {
		joint.linearVelocityImpulse = Vec2Zero()
		joint.angularVelocityImpulse = zero
		joint.linearSpringImpulse = Vec2Zero()
		joint.angularSpringImpulse = zero
	}
}

// warmStartMotorJoint corresponds to b2WarmStartMotorJoint in
// src/motor_joint.c.
func warmStartMotorJoint(base *jointSim, context *stepContext) {
	if base.jointType != MotorJoint {
		panic("dbox2d: the joint is not a motor joint")
	}

	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	joint := base.motor()

	// dummy state for static bodies
	dummyState := identityBodyState()
	stateA, stateB := jointStates(context.states, &dummyState, joint.indexA, joint.indexB)

	rA := RotateVector(stateA.deltaRotation, joint.frameA.P)
	rB := RotateVector(stateB.deltaRotation, joint.frameB.P)

	linearImpulse := joint.linearVelocityImpulse.Add(joint.linearSpringImpulse)
	angularImpulse := joint.angularVelocityImpulse.Add(joint.angularSpringImpulse)

	if stateA.flags&bodyDynamicFlag != 0 {
		stateA.linearVelocity = MulSub(stateA.linearVelocity, mA, linearImpulse)
		stateA.angularVelocity = stateA.angularVelocity.Sub(iA.Mul(Cross(rA, linearImpulse).Add(angularImpulse)))
	}
	if stateB.flags&bodyDynamicFlag != 0 {
		stateB.linearVelocity = MulAdd(stateB.linearVelocity, mB, linearImpulse)
		stateB.angularVelocity = stateB.angularVelocity.Add(iB.Mul(Cross(rB, linearImpulse).Add(angularImpulse)))
	}
}

// solveMotorJoint corresponds to b2SolveMotorJoint in src/motor_joint.c.
// The springs act during relax as well, so the solve takes no useBias.
func solveMotorJoint(base *jointSim, context *stepContext) {
	if base.jointType != MotorJoint {
		panic("dbox2d: the joint is not a motor joint")
	}

	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState()

	joint := base.motor()
	stateA, stateB := jointStates(context.states, &dummyState, joint.indexA, joint.indexB)

	vA := stateA.linearVelocity
	wA := stateA.angularVelocity
	vB := stateB.linearVelocity
	wB := stateB.angularVelocity

	zero := QZero()

	// angular spring
	if zero.Less(joint.maxSpringTorque) && zero.Less(joint.angularHertz) {
		qA := MulRot(stateA.deltaRotation, joint.frameA.Q)
		qB := MulRot(stateB.deltaRotation, joint.frameB.Q)
		// relativeAngle(qA, qB) is b2Rot_GetAngle(b2InvMulRot(qA, qB)).
		c := angleRadians(relativeAngle(qA, qB))

		bias := joint.angularSpring.biasRate.Mul(c)
		massScale := joint.angularSpring.massScale
		impulseScale := joint.angularSpring.impulseScale

		cdot := wB.Sub(wA)

		maxImpulse := context.h.Mul(joint.maxSpringTorque)
		oldImpulse := joint.angularSpringImpulse
		impulse := massScale.Neg().Mul(joint.angularMass).Mul(cdot.Add(bias)).Sub(impulseScale.Mul(oldImpulse))
		joint.angularSpringImpulse = oldImpulse.Add(impulse).Clamp(maxImpulse.Neg(), maxImpulse)
		impulse = joint.angularSpringImpulse.Sub(oldImpulse)

		wA = wA.Sub(iA.Mul(impulse))
		wB = wB.Add(iB.Mul(impulse))
	}

	// angular velocity
	if zero.Less(joint.maxVelocityTorque) {
		cdot := wB.Sub(wA).Sub(angleRadians(joint.angularVelocity))
		impulse := joint.angularMass.Neg().Mul(cdot)

		maxImpulse := context.h.Mul(joint.maxVelocityTorque)
		oldImpulse := joint.angularVelocityImpulse
		joint.angularVelocityImpulse = oldImpulse.Add(impulse).Clamp(maxImpulse.Neg(), maxImpulse)
		impulse = joint.angularVelocityImpulse.Sub(oldImpulse)

		wA = wA.Sub(iA.Mul(impulse))
		wB = wB.Add(iB.Mul(impulse))
	}

	rA := RotateVector(stateA.deltaRotation, joint.frameA.P)
	rB := RotateVector(stateB.deltaRotation, joint.frameB.P)

	// linear spring
	if zero.Less(joint.maxSpringForce) && zero.Less(joint.linearHertz) {
		dcA := stateA.deltaPosition
		dcB := stateB.deltaPosition
		c := dcB.Sub(dcA).Add(rB.Sub(rA)).Add(joint.deltaCenter)

		bias := c.Mul(joint.linearSpring.biasRate)
		massScale := joint.linearSpring.massScale
		impulseScale := joint.linearSpring.impulseScale

		cdot := vB.Add(CrossSV(wB, rB)).Sub(vA.Add(CrossSV(wA, rA)))
		cdot = cdot.Add(bias)

		// Updating the effective mass here may be overkill
		joint.linearMass = motorLinearMass(mA, mB, iA, iB, rA, rB)
		b := MulMV(joint.linearMass, cdot)

		oldImpulse := joint.linearSpringImpulse
		impulse := Vec2{
			X: massScale.Neg().Mul(b.X).Sub(impulseScale.Mul(oldImpulse.X)),
			Y: massScale.Neg().Mul(b.Y).Sub(impulseScale.Mul(oldImpulse.Y)),
		}

		maxImpulse := context.h.Mul(joint.maxSpringForce)
		joint.linearSpringImpulse = clampImpulseLength(joint.linearSpringImpulse.Add(impulse), maxImpulse)

		impulse = joint.linearSpringImpulse.Sub(oldImpulse)

		vA = MulSub(vA, mA, impulse)
		wA = wA.Sub(iA.Mul(Cross(rA, impulse)))
		vB = MulAdd(vB, mB, impulse)
		wB = wB.Add(iB.Mul(Cross(rB, impulse)))
	}

	// linear velocity
	if zero.Less(joint.maxVelocityForce) {
		cdot := vB.Add(CrossSV(wB, rB)).Sub(vA.Add(CrossSV(wA, rA)))
		cdot = cdot.Sub(joint.linearVelocity)
		b := MulMV(joint.linearMass, cdot)
		impulse := Vec2{X: b.X.Neg(), Y: b.Y.Neg()}

		oldImpulse := joint.linearVelocityImpulse
		maxImpulse := context.h.Mul(joint.maxVelocityForce)
		joint.linearVelocityImpulse = clampImpulseLength(joint.linearVelocityImpulse.Add(impulse), maxImpulse)

		impulse = joint.linearVelocityImpulse.Sub(oldImpulse)

		vA = MulSub(vA, mA, impulse)
		wA = wA.Sub(iA.Mul(Cross(rA, impulse)))
		vB = MulAdd(vB, mB, impulse)
		wB = wB.Add(iB.Mul(Cross(rB, impulse)))
	}

	storeJointVelocity(stateA, vA, wA)
	storeJointVelocity(stateB, vB, wB)
}
