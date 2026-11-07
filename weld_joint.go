package b2

// This file corresponds to src/weld_joint.c of the reference. The joint
// keeps frame B on frame A; the angular error enters C in radians. The
// reference also carries a block solver behind B2_WELD_BLOCK_SOLVE, off by
// default; the port keeps the default split solver only.

// getWeldJointForce reports the constraint force of the last step. It
// corresponds to b2GetWeldJointForce in src/weld_joint.c.
func getWeldJointForce(w *world, base *jointSim) Vec2 {
	return base.weld().linearImpulse.Mul(w.invH)
}

// getWeldJointTorque reports the constraint torque of the last step. It
// corresponds to b2GetWeldJointTorque in src/weld_joint.c.
func getWeldJointTorque(w *world, base *jointSim) Q {
	return w.invH.Mul(base.weld().angularImpulse)
}

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-E -r1_skew E r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Angle constraint
// C = angle2 - angle1 - referenceAngle
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

// prepareWeldJoint corresponds to b2PrepareWeldJoint in src/weld_joint.c.
func prepareWeldJoint(base *jointSim, context *stepContext) {
	if base.jointType != WeldJoint {
		panic("dbox2d: the joint is not a weld joint")
	}
	bodySimA, bodySimB, indexA, indexB := prepareJointBodies(base, context)
	joint := base.weld()
	joint.indexA = indexA
	joint.indexB = indexB

	joint.frameA = jointFrame(bodySimA, base.localFrameA)
	joint.frameB = jointFrame(bodySimB, base.localFrameB)

	// Compute the initial center delta. Incremental position updates are
	// relative to this.
	joint.deltaCenter = bodySimB.center.Sub(bodySimA.center)

	zero := QZero()
	ka := base.invIA.Add(base.invIB)
	// D-006: the reference multiplies by the reciprocal of ka.
	joint.axialMass = zero
	if zero.Less(ka) {
		joint.axialMass = QOne().Div(ka)
	}

	if joint.linearHertz.Eq(zero) {
		joint.linearSpring = base.constraintSoftness
	} else {
		joint.linearSpring = makeSoft(joint.linearHertz, joint.linearDampingRatio, context.h)
	}

	if joint.angularHertz.Eq(zero) {
		joint.angularSpring = base.constraintSoftness
	} else {
		joint.angularSpring = makeSoft(joint.angularHertz, joint.angularDampingRatio, context.h)
	}

	if !context.enableWarmStarting {
		joint.linearImpulse = Vec2Zero()
		joint.angularImpulse = zero
	}
}

// warmStartWeldJoint corresponds to b2WarmStartWeldJoint in
// src/weld_joint.c.
func warmStartWeldJoint(base *jointSim, context *stepContext) {
	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState()

	joint := base.weld()
	stateA, stateB := jointStates(context.states, &dummyState, joint.indexA, joint.indexB)

	rA := RotateVector(stateA.deltaRotation, joint.frameA.P)
	rB := RotateVector(stateB.deltaRotation, joint.frameB.P)

	if stateA.flags&bodyDynamicFlag != 0 {
		stateA.linearVelocity = MulSub(stateA.linearVelocity, mA, joint.linearImpulse)
		stateA.angularVelocity = stateA.angularVelocity.Sub(iA.Mul(Cross(rA, joint.linearImpulse).Add(joint.angularImpulse)))
	}
	if stateB.flags&bodyDynamicFlag != 0 {
		stateB.linearVelocity = MulAdd(stateB.linearVelocity, mB, joint.linearImpulse)
		stateB.angularVelocity = stateB.angularVelocity.Add(iB.Mul(Cross(rB, joint.linearImpulse).Add(joint.angularImpulse)))
	}
}

// solveWeldJoint corresponds to b2SolveWeldJoint in src/weld_joint.c.
func solveWeldJoint(base *jointSim, context *stepContext, useBias bool) {
	if base.jointType != WeldJoint {
		panic("dbox2d: the joint is not a weld joint")
	}

	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState()

	joint := base.weld()
	stateA, stateB := jointStates(context.states, &dummyState, joint.indexA, joint.indexB)

	vA := stateA.linearVelocity
	wA := stateA.angularVelocity
	vB := stateB.linearVelocity
	wB := stateB.angularVelocity

	zero := QZero()
	one := QOne()

	// angular constraint
	{
		qA := MulRot(stateA.deltaRotation, joint.frameA.Q)
		qB := MulRot(stateB.deltaRotation, joint.frameB.Q)
		// relativeAngle(qA, qB) is b2Rot_GetAngle(b2InvMulRot(qA, qB)).
		jointAngle := relativeAngle(qA, qB)

		bias := zero
		massScale := one
		impulseScale := zero
		if useBias || zero.Less(joint.angularHertz) {
			C := angleRadians(jointAngle)
			bias = joint.angularSpring.biasRate.Mul(C)
			massScale = joint.angularSpring.massScale
			impulseScale = joint.angularSpring.impulseScale
		}

		Cdot := wB.Sub(wA)
		impulse := massScale.Neg().Mul(joint.axialMass).Mul(Cdot.Add(bias)).Sub(impulseScale.Mul(joint.angularImpulse))
		joint.angularImpulse = joint.angularImpulse.Add(impulse)

		wA = wA.Sub(iA.Mul(impulse))
		wB = wB.Add(iB.Mul(impulse))
	}

	// linear constraint
	{
		rA := RotateVector(stateA.deltaRotation, joint.frameA.P)
		rB := RotateVector(stateB.deltaRotation, joint.frameB.P)

		bias := Vec2Zero()
		massScale := one
		impulseScale := zero
		if useBias || zero.Less(joint.linearHertz) {
			dcA := stateA.deltaPosition
			dcB := stateB.deltaPosition
			C := dcB.Sub(dcA).Add(rB.Sub(rA)).Add(joint.deltaCenter)

			bias = C.Mul(joint.linearSpring.biasRate)
			massScale = joint.linearSpring.massScale
			impulseScale = joint.linearSpring.impulseScale
		}

		Cdot := vB.Add(CrossSV(wB, rB)).Sub(vA.Add(CrossSV(wA, rA)))

		var K Mat22
		K.Cx.X = mA.Add(mB).Add(rA.Y.Mul(rA.Y).Mul(iA)).Add(rB.Y.Mul(rB.Y).Mul(iB))
		K.Cy.X = rA.Y.Neg().Mul(rA.X).Mul(iA).Sub(rB.Y.Mul(rB.X).Mul(iB))
		K.Cx.Y = K.Cy.X
		K.Cy.Y = mA.Add(mB).Add(rA.X.Mul(rA.X).Mul(iA)).Add(rB.X.Mul(rB.X).Mul(iB))
		b := Solve22(K, Cdot.Add(bias))

		impulse := Vec2{
			X: massScale.Neg().Mul(b.X).Sub(impulseScale.Mul(joint.linearImpulse.X)),
			Y: massScale.Neg().Mul(b.Y).Sub(impulseScale.Mul(joint.linearImpulse.Y)),
		}

		joint.linearImpulse = joint.linearImpulse.Add(impulse)

		vA = MulSub(vA, mA, impulse)
		wA = wA.Sub(iA.Mul(Cross(rA, impulse)))
		vB = MulAdd(vB, mB, impulse)
		wB = wB.Add(iB.Mul(Cross(rB, impulse)))
	}

	storeJointVelocity(stateA, vA, wA)
	storeJointVelocity(stateB, vB, wB)
}

// drawWeldJoint corresponds to b2DrawWeldJoint: a box on each frame.
func drawWeldJoint(draw *DebugDraw, base *jointSim, transformA, transformB Transform, drawScale Q) {
	frameA := MulTransforms(transformA, base.localFrameA)
	frameB := MulTransforms(transformB, base.localFrameB)

	box := MakeBox(QMustParse("0.25").Mul(drawScale), QMustParse("0.125").Mul(drawScale))
	draw.DrawPolygonFcn(frameA, box.Vertices[:4], ColorDarkOrange)
	draw.DrawPolygonFcn(frameB, box.Vertices[:4], ColorDarkCyan)
}
