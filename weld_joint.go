package dbox2d

import "github.com/dhannyell/fixed"

// This file corresponds to src/weld_joint.c of the reference. The angles
// are turns (D-004); the angular error enters C in radians.

// getWeldJointForce reports the constraint force of the last step. It
// corresponds to b2GetWeldJointForce in src/weld_joint.c.
func getWeldJointForce(w *world, base *jointSim) Vec2 {
	force := base.weldJoint.linearImpulse.Mul(w.invH)
	return force
}

// getWeldJointTorque reports the constraint torque of the last step. It
// corresponds to b2GetWeldJointTorque in src/weld_joint.c.
func getWeldJointTorque(w *world, base *jointSim) Q {
	return w.invH.Mul(base.weldJoint.angularImpulse)
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

// prepareWeldJoint corresponds to b2PrepareWeldJoint in src/weld_joint.c.
func prepareWeldJoint(base *jointSim, context *stepContext) {
	if base.jointType != WeldJoint {
		panic("dbox2d: the joint is not a weld joint")
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

	joint := &base.weldJoint
	joint.indexA = nullIndex
	if bodyA.setIndex == awakeSet {
		joint.indexA = localIndexA
	}
	joint.indexB = nullIndex
	if bodyB.setIndex == awakeSet {
		joint.indexB = localIndexB
	}

	qA := bodySimA.transform.Q
	qB := bodySimB.transform.Q

	joint.anchorA = RotateVector(qA, base.localOriginAnchorA.Sub(bodySimA.localCenter))
	joint.anchorB = RotateVector(qB, base.localOriginAnchorB.Sub(bodySimB.localCenter))
	joint.deltaCenter = bodySimB.center.Sub(bodySimA.center)
	joint.deltaAngle = RelativeAngle(qB, qA).Sub(joint.referenceAngle)
	joint.deltaAngle = UnwindAngle(joint.deltaAngle)

	zero := fixed.Q32Zero()
	ka := iA.Add(iB)
	// D-006: the reference multiplies by the reciprocal of ka.
	joint.axialMass = zero
	if zero.Less(ka) {
		joint.axialMass = fixed.Q32One().Div(ka)
	}

	if joint.linearHertz.Eq(zero) {
		joint.linearSoftness = base.constraintSoftness
	} else {
		joint.linearSoftness = makeSoft(joint.linearHertz, joint.linearDampingRatio, context.h)
	}

	if joint.angularHertz.Eq(zero) {
		joint.angularSoftness = base.constraintSoftness
	} else {
		joint.angularSoftness = makeSoft(joint.angularHertz, joint.angularDampingRatio, context.h)
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

	joint := &base.weldJoint

	stateA, stateB := jointStates(context.states, &dummyState, joint.indexA, joint.indexB)

	rA := RotateVector(stateA.deltaRotation, joint.anchorA)
	rB := RotateVector(stateB.deltaRotation, joint.anchorB)

	// D-004: the angular velocity of the state is turns per second.
	stateA.linearVelocity = MulSub(stateA.linearVelocity, mA, joint.linearImpulse)
	wA := stateA.angularVelocity.Mul(tau)
	wA = wA.Sub(iA.Mul(Cross(rA, joint.linearImpulse).Add(joint.angularImpulse)))
	stateA.angularVelocity = wA.Div(tau)

	stateB.linearVelocity = MulAdd(stateB.linearVelocity, mB, joint.linearImpulse)
	wB := stateB.angularVelocity.Mul(tau)
	wB = wB.Add(iB.Mul(Cross(rB, joint.linearImpulse).Add(joint.angularImpulse)))
	stateB.angularVelocity = wB.Div(tau)
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

	joint := &base.weldJoint

	stateA, stateB := jointStates(context.states, &dummyState, joint.indexA, joint.indexB)

	vA := stateA.linearVelocity
	wA := stateA.angularVelocity.Mul(tau)
	vB := stateB.linearVelocity
	wB := stateB.angularVelocity.Mul(tau)

	zero := fixed.Q32Zero()
	one := fixed.Q32One()

	// angular constraint
	{
		bias := zero
		massScale := one
		impulseScale := zero
		if useBias || zero.Less(joint.angularHertz) {
			// D-004: the angle enters the error in radians.
			C := RelativeAngle(stateB.deltaRotation, stateA.deltaRotation).Add(joint.deltaAngle).Mul(tau)
			bias = joint.angularSoftness.biasRate.Mul(C)
			massScale = joint.angularSoftness.massScale
			impulseScale = joint.angularSoftness.impulseScale
		}

		Cdot := wB.Sub(wA)
		impulse := massScale.Neg().Mul(joint.axialMass).Mul(Cdot.Add(bias)).Sub(impulseScale.Mul(joint.angularImpulse))
		joint.angularImpulse = joint.angularImpulse.Add(impulse)

		wA = wA.Sub(iA.Mul(impulse))
		wB = wB.Add(iB.Mul(impulse))
	}

	// linear constraint
	{
		rA := RotateVector(stateA.deltaRotation, joint.anchorA)
		rB := RotateVector(stateB.deltaRotation, joint.anchorB)

		bias := Vec2Zero()
		massScale := one
		impulseScale := zero
		if useBias || zero.Less(joint.linearHertz) {
			dcA := stateA.deltaPosition
			dcB := stateB.deltaPosition
			C := dcB.Sub(dcA).Add(rB.Sub(rA)).Add(joint.deltaCenter)

			bias = C.Mul(joint.linearSoftness.biasRate)
			massScale = joint.linearSoftness.massScale
			impulseScale = joint.linearSoftness.impulseScale
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

	stateA.linearVelocity = vA
	stateA.angularVelocity = wA.Div(tau)
	stateB.linearVelocity = vB
	stateB.angularVelocity = wB.Div(tau)
}
