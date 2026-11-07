package b2

// This file corresponds to src/revolute_joint.c of the reference. The API
// takes angles in turns (D-004). The joint keeps them in the angle unit of
// the mode, and angleRadians converts an angle where it enters an error C.

// drawRevoluteJoint corresponds to b2DrawRevoluteJoint.
func drawRevoluteJoint(draw *DebugDraw, base *jointSim, transformA, transformB Transform, drawScale Q) {
	joint := base.revolute()
	frameA := MulTransforms(transformA, base.localFrameA)
	frameB := MulTransforms(transformB, base.localFrameB)

	radius := QMustParse("0.25").Mul(drawScale)
	draw.DrawCircleFcn(frameB.P, radius, ColorGray)

	rx := Vec2{X: radius, Y: QZero()}
	r := RotateVector(frameA.Q, rx)
	draw.DrawLineFcn(frameA.P, frameA.P.Add(r), ColorGray)

	r = RotateVector(frameB.Q, rx)
	draw.DrawLineFcn(frameB.P, frameB.P.Add(r), ColorBlue)

	if draw.DrawJointExtras {
		degrees := RelativeAngle(frameA.Q, frameB.Q).Mul(QFromInt(360))
		draw.DrawStringFcn(frameA.P.Add(r), " "+drawNumber(degrees, 1)+" deg", ColorWhite)
	}

	if joint.enableLimit {
		rotLo := MulRot(frameA.Q, MakeRot(angleToTurns(joint.lowerAngle)))
		rlo := RotateVector(rotLo, rx)
		rotHi := MulRot(frameA.Q, MakeRot(angleToTurns(joint.upperAngle)))
		rhi := RotateVector(rotHi, rx)
		draw.DrawLineFcn(frameB.P, frameB.P.Add(rlo), ColorGreen)
		draw.DrawLineFcn(frameB.P, frameB.P.Add(rhi), ColorRed)
	}

	if joint.enableSpring {
		q := MulRot(frameA.Q, MakeRot(angleToTurns(joint.targetAngle)))
		v := RotateVector(q, rx)
		draw.DrawLineFcn(frameB.P, frameB.P.Add(v), ColorViolet)
	}

	draw.DrawLineFcn(frameA.P, frameB.P, ColorGold)
	if draw.DrawJointExtras {
		draw.DrawLineFcn(transformA.P, frameA.P, ColorGold)
		draw.DrawLineFcn(transformB.P, frameB.P, ColorGold)
	}
}

// getRevoluteJointForce reports the constraint force of the last step. It
// corresponds to b2GetRevoluteJointForce in src/revolute_joint.c.
func getRevoluteJointForce(w *world, base *jointSim) Vec2 {
	return base.revolute().linearImpulse.Mul(w.invH)
}

// getRevoluteJointTorque reports the constraint torque of the last step.
// It corresponds to b2GetRevoluteJointTorque in src/revolute_joint.c.
func getRevoluteJointTorque(w *world, base *jointSim) Q {
	revolute := base.revolute()
	return w.invH.Mul(revolute.motorImpulse.Add(revolute.lowerImpulse).Sub(revolute.upperImpulse))
}

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Motor constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

// prepareRevoluteJoint corresponds to b2PrepareRevoluteJoint in
// src/revolute_joint.c.
func prepareRevoluteJoint(base *jointSim, context *stepContext) {
	if base.jointType != RevoluteJoint {
		panic("dbox2d: the joint is not a revolute joint")
	}
	bodySimA, bodySimB, indexA, indexB := prepareJointBodies(base, context)
	joint := base.revolute()
	joint.indexA = indexA
	joint.indexB = indexB

	joint.frameA = jointFrame(bodySimA, base.localFrameA)
	joint.frameB = jointFrame(bodySimB, base.localFrameB)

	// Compute the initial center delta. Incremental position updates are
	// relative to this.
	joint.deltaCenter = bodySimB.center.Sub(bodySimA.center)

	zero := QZero()
	k := base.invIA.Add(base.invIB)
	// D-006: the reference multiplies by the reciprocal of k.
	joint.axialMass = zero
	if zero.Less(k) {
		joint.axialMass = QOne().Div(k)
	}

	joint.springSoftness = makeSoft(joint.hertz, joint.dampingRatio, context.h)

	if !context.enableWarmStarting {
		joint.linearImpulse = Vec2Zero()
		joint.springImpulse = zero
		joint.motorImpulse = zero
		joint.lowerImpulse = zero
		joint.upperImpulse = zero
	}
}

// warmStartRevoluteJoint corresponds to b2WarmStartRevoluteJoint in
// src/revolute_joint.c.
func warmStartRevoluteJoint(base *jointSim, context *stepContext) {
	if base.jointType != RevoluteJoint {
		panic("dbox2d: the joint is not a revolute joint")
	}

	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState()

	joint := base.revolute()
	stateA, stateB := jointStates(context.states, &dummyState, joint.indexA, joint.indexB)

	rA := RotateVector(stateA.deltaRotation, joint.frameA.P)
	rB := RotateVector(stateB.deltaRotation, joint.frameB.P)

	axialImpulse := joint.springImpulse.Add(joint.motorImpulse).Add(joint.lowerImpulse).Sub(joint.upperImpulse)

	if stateA.flags&bodyDynamicFlag != 0 {
		stateA.linearVelocity = MulSub(stateA.linearVelocity, mA, joint.linearImpulse)
		stateA.angularVelocity = stateA.angularVelocity.Sub(iA.Mul(Cross(rA, joint.linearImpulse).Add(axialImpulse)))
	}
	if stateB.flags&bodyDynamicFlag != 0 {
		stateB.linearVelocity = MulAdd(stateB.linearVelocity, mB, joint.linearImpulse)
		stateB.angularVelocity = stateB.angularVelocity.Add(iB.Mul(Cross(rB, joint.linearImpulse).Add(axialImpulse)))
	}
}

// solveRevoluteJoint corresponds to b2SolveRevoluteJoint in
// src/revolute_joint.c. The axial constraints feed their impulses forward
// through angularCdot and apply them once, with the point constraint.
func solveRevoluteJoint(base *jointSim, context *stepContext, useBias bool) {
	if base.jointType != RevoluteJoint {
		panic("dbox2d: the joint is not a revolute joint")
	}

	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState()

	joint := base.revolute()

	stateA, stateB := jointStates(context.states, &dummyState, joint.indexA, joint.indexB)

	vA := stateA.linearVelocity
	wA := stateA.angularVelocity
	vB := stateB.linearVelocity
	wB := stateB.angularVelocity

	qA := MulRot(stateA.deltaRotation, joint.frameA.Q)
	qB := MulRot(stateB.deltaRotation, joint.frameB.Q)

	// current anchors
	rA := RotateVector(stateA.deltaRotation, joint.frameA.P)
	rB := RotateVector(stateB.deltaRotation, joint.frameB.P)

	linearCdot := vB.Add(CrossSV(wB, rB)).Sub(vA.Add(CrossSV(wA, rA)))
	angularCdot := wB.Sub(wA)
	zero := QZero()
	one := QOne()
	axialImpulse := zero

	k := iA.Add(iB)
	fixedRotation := k.Eq(zero)

	// Solve spring.
	if joint.enableSpring && !fixedRotation {
		// relativeAngle(qA, qB) is b2Rot_GetAngle(b2InvMulRot(qA, qB)).
		jointAngle := relativeAngle(qA, qB)
		jointAngleDelta := unwindAngle(jointAngle.Sub(joint.targetAngle))

		C := angleRadians(jointAngleDelta)
		bias := joint.springSoftness.biasRate.Mul(C)
		massScale := joint.springSoftness.massScale
		impulseScale := joint.springSoftness.impulseScale

		Cdot := angularCdot
		impulse := massScale.Neg().Mul(joint.axialMass).Mul(Cdot.Add(bias)).Sub(impulseScale.Mul(joint.springImpulse))
		joint.springImpulse = joint.springImpulse.Add(impulse)

		angularCdot = angularCdot.Add(k.Mul(impulse))
		axialImpulse = axialImpulse.Add(impulse)
	}

	// Solve motor constraint.
	if joint.enableMotor && !fixedRotation {
		Cdot := angularCdot.Sub(angleRadians(joint.motorSpeed))
		oldImpulse := joint.motorImpulse
		maxImpulse := context.h.Mul(joint.maxMotorTorque)
		impulse := joint.axialMass.Neg().Mul(Cdot).Clamp(maxImpulse.Neg().Sub(oldImpulse), maxImpulse.Sub(oldImpulse))
		joint.motorImpulse = oldImpulse.Add(impulse)

		angularCdot = angularCdot.Add(k.Mul(impulse))
		axialImpulse = axialImpulse.Add(impulse)
	}

	if joint.enableLimit && !fixedRotation {
		jointAngle := relativeAngle(qA, qB)

		// Lower limit
		{
			C := angleRadians(jointAngle.Sub(joint.lowerAngle))
			bias := zero
			massScale := one
			impulseScale := zero
			if zero.Less(C) {
				// speculation
				bias = C.Mul(context.invH)
			} else if useBias {
				bias = base.constraintSoftness.biasRate.Mul(C)
				massScale = base.constraintSoftness.massScale
				impulseScale = base.constraintSoftness.impulseScale
			}

			Cdot := angularCdot
			oldImpulse := joint.lowerImpulse
			impulse := massScale.Neg().Mul(joint.axialMass).Mul(Cdot.Add(bias)).Sub(impulseScale.Mul(oldImpulse)).Max(oldImpulse.Neg())
			joint.lowerImpulse = oldImpulse.Add(impulse)

			angularCdot = angularCdot.Add(k.Mul(impulse))
			axialImpulse = axialImpulse.Add(impulse)
		}

		// Upper limit
		// Note: signs are flipped to keep C positive when the constraint is satisfied.
		// This also keeps the impulse positive when the limit is active.
		{
			C := angleRadians(joint.upperAngle.Sub(jointAngle))
			bias := zero
			massScale := one
			impulseScale := zero
			if zero.Less(C) {
				// speculation
				bias = C.Mul(context.invH)
			} else if useBias {
				bias = base.constraintSoftness.biasRate.Mul(C)
				massScale = base.constraintSoftness.massScale
				impulseScale = base.constraintSoftness.impulseScale
			}

			// sign flipped on Cdot
			Cdot := angularCdot.Neg()
			oldImpulse := joint.upperImpulse
			impulse := massScale.Neg().Mul(joint.axialMass).Mul(Cdot.Add(bias)).Sub(impulseScale.Mul(oldImpulse)).Max(oldImpulse.Neg())
			joint.upperImpulse = oldImpulse.Add(impulse)

			// sign flipped on applied impulse
			axialImpulse = axialImpulse.Sub(impulse)
		}
	}

	// Solve point-to-point constraint
	{
		// J = [-I -r1_skew I r2_skew]
		// r_skew = [-ry; rx]
		// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x]
		//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB]

		Cdot := MulAdd(linearCdot, axialImpulse, CrossSV(iA, rA).Add(CrossSV(iB, rB)))

		bias := Vec2Zero()
		massScale := one
		impulseScale := zero
		if useBias {
			dcA := stateA.deltaPosition
			dcB := stateB.deltaPosition

			separation := dcB.Sub(dcA).Add(rB.Sub(rA)).Add(joint.deltaCenter)
			bias = separation.Mul(base.constraintSoftness.biasRate)
			massScale = base.constraintSoftness.massScale
			impulseScale = base.constraintSoftness.impulseScale
		}

		var K Mat22
		K.Cx.X = mA.Add(mB).Add(rA.Y.Mul(rA.Y).Mul(iA)).Add(rB.Y.Mul(rB.Y).Mul(iB))
		K.Cy.X = rA.Y.Neg().Mul(rA.X).Mul(iA).Sub(rB.Y.Mul(rB.X).Mul(iB))
		K.Cx.Y = K.Cy.X
		K.Cy.Y = mA.Add(mB).Add(rA.X.Mul(rA.X).Mul(iA)).Add(rB.X.Mul(rB.X).Mul(iB))
		b := Solve22(K, Cdot.Add(bias))

		var impulse Vec2
		impulse.X = massScale.Neg().Mul(b.X).Sub(impulseScale.Mul(joint.linearImpulse.X))
		impulse.Y = massScale.Neg().Mul(b.Y).Sub(impulseScale.Mul(joint.linearImpulse.Y))
		joint.linearImpulse.X = joint.linearImpulse.X.Add(impulse.X)
		joint.linearImpulse.Y = joint.linearImpulse.Y.Add(impulse.Y)

		vA = MulSub(vA, mA, impulse)
		wA = wA.Sub(iA.Mul(Cross(rA, impulse).Add(axialImpulse)))
		vB = MulAdd(vB, mB, impulse)
		wB = wB.Add(iB.Mul(Cross(rB, impulse).Add(axialImpulse)))
	}

	storeJointVelocity(stateA, vA, wA)
	storeJointVelocity(stateB, vB, wB)
}
