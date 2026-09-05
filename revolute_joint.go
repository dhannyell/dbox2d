package dbox2d

import "github.com/dhannyell/fixed"

// This file corresponds to src/revolute_joint.c of the reference. The
// angles are turns (D-004); an angle turns into radians by tau at the
// single point where it enters an error C.

func drawRevoluteJoint(draw *DebugDraw, base *jointSim, transformA, transformB Transform, drawSize Q) {
	joint := &base.revoluteJoint
	pA := TransformPoint(transformA, base.localOriginAnchorA)
	pB := TransformPoint(transformB, base.localOriginAnchorB)
	angle := RelativeAngle(transformB.Q, transformA.Q)
	rot := MakeRot(angle)
	pC := pB.Add(Vec2{X: drawSize.Mul(rot.Cos), Y: drawSize.Mul(rot.Sin)})
	draw.DrawCircle(pB, drawSize, ColorGray)
	draw.DrawSegment(pB, pC, ColorGray)
	if draw.DrawJointExtras {
		degrees := UnwindAngle(angle.Sub(joint.referenceAngle)).Mul(fixed.Q32FromInt(360))
		draw.DrawString(pC, " "+drawNumber(degrees, 1)+" deg", ColorWhite)
	}
	if joint.enableLimit {
		for _, limit := range []struct {
			angle Q
			color HexColor
		}{
			{joint.lowerAngle.Add(joint.referenceAngle), ColorGreen},
			{joint.upperAngle.Add(joint.referenceAngle), ColorRed},
			{joint.referenceAngle, ColorBlue},
		} {
			r := MakeRot(limit.angle)
			end := pB.Add(Vec2{X: drawSize.Mul(r.Cos), Y: drawSize.Mul(r.Sin)})
			draw.DrawSegment(pB, end, limit.color)
		}
	}
	draw.DrawSegment(transformA.P, pA, ColorGold)
	draw.DrawSegment(pA, pB, ColorGold)
	draw.DrawSegment(transformB.P, pB, ColorGold)
}

// getRevoluteJointForce reports the constraint force of the last step. It
// corresponds to b2GetRevoluteJointForce in src/revolute_joint.c.
func getRevoluteJointForce(w *world, base *jointSim) Vec2 {
	force := base.revoluteJoint.linearImpulse.Mul(w.invH)
	return force
}

// getRevoluteJointTorque reports the constraint torque of the last step.
// It corresponds to b2GetRevoluteJointTorque in src/revolute_joint.c.
func getRevoluteJointTorque(w *world, base *jointSim) Q {
	revolute := &base.revoluteJoint
	torque := w.invH.Mul(revolute.motorImpulse.Add(revolute.lowerImpulse).Sub(revolute.upperImpulse))
	return torque
}

// SetTargetAngle changes the revolute spring target angle in turns.
func (jointId JointId) SetTargetAngle(angle Q) {
	w := getWorld(jointId.world0)
	joint := getJointSimCheckType(w, jointId, RevoluteJoint)
	// D-004: the target angle is stored in turns and is bounded to half a turn.
	halfTurn := fixed.Q32Half()
	joint.revoluteJoint.targetAngle = angle.Clamp(halfTurn.Neg(), halfTurn)
}

// GetTargetAngle reports the revolute spring target angle in turns.
func (jointId JointId) GetTargetAngle() Q {
	w := getWorld(jointId.world0)
	joint := getJointSimCheckType(w, jointId, RevoluteJoint)
	// D-004: the target angle is stored in turns.
	return joint.revoluteJoint.targetAngle
}

// GetAngle reports the revolute joint angle relative to its reference angle,
// in turns.
func (jointId JointId) GetAngle() Q {
	w := getWorld(jointId.world0)
	joint := getJointSimCheckType(w, jointId, RevoluteJoint)
	transformA := getBodyTransform(w, joint.bodyIdA)
	transformB := getBodyTransform(w, joint.bodyIdB)
	// D-004: RelativeAngle and the stored reference angle are in turns.
	angle := RelativeAngle(transformB.Q, transformA.Q).Sub(joint.revoluteJoint.referenceAngle)
	return UnwindAngle(angle)
}

// GetLowerLimit reports the lower angular or linear joint limit.
func (jointId JointId) GetLowerLimit() Q {
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	joint := getJointSim(w, j)
	switch j.jointType {
	case RevoluteJoint:
		// D-004: revolute limits are stored in turns.
		return joint.revoluteJoint.lowerAngle
	case PrismaticJoint:
		return joint.prismaticJoint.lowerTranslation
	case WheelJoint:
		return joint.wheelJoint.lowerTranslation
	default:
		panic("dbox2d: joint type does not support lower limits")
	}
}

// GetUpperLimit reports the upper angular or linear joint limit.
func (jointId JointId) GetUpperLimit() Q {
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	joint := getJointSim(w, j)
	switch j.jointType {
	case RevoluteJoint:
		// D-004: revolute limits are stored in turns.
		return joint.revoluteJoint.upperAngle
	case PrismaticJoint:
		return joint.prismaticJoint.upperTranslation
	case WheelJoint:
		return joint.wheelJoint.upperTranslation
	default:
		panic("dbox2d: joint type does not support upper limits")
	}
}

// SetLimits changes the angular or linear joint limits.
func (jointId JointId) SetLimits(lower, upper Q) {
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	joint := getJointSim(w, j)
	switch j.jointType {
	case RevoluteJoint:
		// D-004: convert the reference +/- pi bound to +/- half a turn.
		halfTurn := fixed.Q32Half()
		lower = lower.Clamp(halfTurn.Neg(), halfTurn)
		upper = upper.Clamp(halfTurn.Neg(), halfTurn)
		lowerAngle := lower.Min(upper)
		upperAngle := lower.Max(upper)
		if !lowerAngle.Eq(joint.revoluteJoint.lowerAngle) || !upperAngle.Eq(joint.revoluteJoint.upperAngle) {
			joint.revoluteJoint.lowerAngle = lowerAngle
			joint.revoluteJoint.upperAngle = upperAngle
			joint.revoluteJoint.lowerImpulse = fixed.Q32Zero()
			joint.revoluteJoint.upperImpulse = fixed.Q32Zero()
		}
	case PrismaticJoint:
		if !lower.Eq(joint.prismaticJoint.lowerTranslation) || !upper.Eq(joint.prismaticJoint.upperTranslation) {
			joint.prismaticJoint.lowerTranslation = lower.Min(upper)
			joint.prismaticJoint.upperTranslation = lower.Max(upper)
			joint.prismaticJoint.lowerImpulse = fixed.Q32Zero()
			joint.prismaticJoint.upperImpulse = fixed.Q32Zero()
		}
	case WheelJoint:
		if !lower.Eq(joint.wheelJoint.lowerTranslation) || !upper.Eq(joint.wheelJoint.upperTranslation) {
			joint.wheelJoint.lowerTranslation = lower.Min(upper)
			joint.wheelJoint.upperTranslation = lower.Max(upper)
			joint.wheelJoint.lowerImpulse = fixed.Q32Zero()
			joint.wheelJoint.upperImpulse = fixed.Q32Zero()
		}
	default:
		panic("dbox2d: joint type does not support limits")
	}
}

// GetMotorTorque reports the last revolute or wheel motor torque.
func (jointId JointId) GetMotorTorque() Q {
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	joint := getJointSim(w, j)
	switch j.jointType {
	case RevoluteJoint:
		return w.invH.Mul(joint.revoluteJoint.motorImpulse)
	case WheelJoint:
		return w.invH.Mul(joint.wheelJoint.motorImpulse)
	default:
		panic("dbox2d: joint type does not support motor torque")
	}
}

// SetMaxMotorTorque changes the maximum revolute or wheel motor torque.
func (jointId JointId) SetMaxMotorTorque(torque Q) {
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	joint := getJointSim(w, j)
	switch j.jointType {
	case RevoluteJoint:
		joint.revoluteJoint.maxMotorTorque = torque
	case WheelJoint:
		joint.wheelJoint.maxMotorTorque = torque
	default:
		panic("dbox2d: joint type does not support motor torque")
	}
}

// GetMaxMotorTorque reports the maximum revolute or wheel motor torque.
func (jointId JointId) GetMaxMotorTorque() Q {
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	joint := getJointSim(w, j)
	switch j.jointType {
	case RevoluteJoint:
		return joint.revoluteJoint.maxMotorTorque
	case WheelJoint:
		return joint.wheelJoint.maxMotorTorque
	default:
		panic("dbox2d: joint type does not support motor torque")
	}
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

	joint := &base.revoluteJoint

	joint.indexA = nullIndex
	if bodyA.setIndex == awakeSet {
		joint.indexA = localIndexA
	}
	joint.indexB = nullIndex
	if bodyB.setIndex == awakeSet {
		joint.indexB = localIndexB
	}

	// initial anchors in world space
	joint.anchorA = RotateVector(bodySimA.transform.Q, base.localOriginAnchorA.Sub(bodySimA.localCenter))
	joint.anchorB = RotateVector(bodySimB.transform.Q, base.localOriginAnchorB.Sub(bodySimB.localCenter))
	joint.deltaCenter = bodySimB.center.Sub(bodySimA.center)
	joint.deltaAngle = RelativeAngle(bodySimB.transform.Q, bodySimA.transform.Q)

	zero := fixed.Q32Zero()
	k := iA.Add(iB)
	// D-006: the reference multiplies by the reciprocal of k.
	joint.axialMass = zero
	if zero.Less(k) {
		joint.axialMass = fixed.Q32One().Div(k)
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

	joint := &base.revoluteJoint
	stateA, stateB := jointStates(context.states, &dummyState, joint.indexA, joint.indexB)

	rA := RotateVector(stateA.deltaRotation, joint.anchorA)
	rB := RotateVector(stateB.deltaRotation, joint.anchorB)

	axialImpulse := joint.springImpulse.Add(joint.motorImpulse).Add(joint.lowerImpulse).Sub(joint.upperImpulse)

	// D-004: the angular velocity of the state is turns per second.
	stateA.linearVelocity = MulSub(stateA.linearVelocity, mA, joint.linearImpulse)
	wA := stateA.angularVelocity.Mul(tau)
	wA = wA.Sub(iA.Mul(Cross(rA, joint.linearImpulse).Add(axialImpulse)))
	stateA.angularVelocity = wA.Div(tau)

	stateB.linearVelocity = MulAdd(stateB.linearVelocity, mB, joint.linearImpulse)
	wB := stateB.angularVelocity.Mul(tau)
	wB = wB.Add(iB.Mul(Cross(rB, joint.linearImpulse).Add(axialImpulse)))
	stateB.angularVelocity = wB.Div(tau)
}

// solveRevoluteJoint corresponds to b2SolveRevoluteJoint in
// src/revolute_joint.c.
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

	joint := &base.revoluteJoint

	stateA, stateB := jointStates(context.states, &dummyState, joint.indexA, joint.indexB)

	vA := stateA.linearVelocity
	wA := stateA.angularVelocity.Mul(tau)
	vB := stateB.linearVelocity
	wB := stateB.angularVelocity.Mul(tau)

	dqA := stateA.deltaRotation
	dqB := stateB.deltaRotation

	zero := fixed.Q32Zero()
	one := fixed.Q32One()
	fixedRotation := iA.Add(iB).Eq(zero)

	// Solve spring.
	if joint.enableSpring && !fixedRotation {
		jointAngle := RelativeAngle(stateB.deltaRotation, stateA.deltaRotation).Add(joint.deltaAngle)
		jointAngleDelta := UnwindAngle(jointAngle.Sub(joint.targetAngle))

		// D-004: the angle enters the error in radians.
		C := jointAngleDelta.Mul(tau)
		bias := joint.springSoftness.biasRate.Mul(C)
		massScale := joint.springSoftness.massScale
		impulseScale := joint.springSoftness.impulseScale

		Cdot := wB.Sub(wA)
		impulse := massScale.Neg().Mul(joint.axialMass).Mul(Cdot.Add(bias)).Sub(impulseScale.Mul(joint.springImpulse))
		joint.springImpulse = joint.springImpulse.Add(impulse)

		wA = wA.Sub(iA.Mul(impulse))
		wB = wB.Add(iB.Mul(impulse))
	}

	// Solve motor constraint.
	if joint.enableMotor && !fixedRotation {
		// D-004: the motor speed is turns per second.
		Cdot := wB.Sub(wA).Sub(joint.motorSpeed.Mul(tau))
		impulse := joint.axialMass.Neg().Mul(Cdot)
		oldImpulse := joint.motorImpulse
		maxImpulse := context.h.Mul(joint.maxMotorTorque)
		joint.motorImpulse = joint.motorImpulse.Add(impulse).Clamp(maxImpulse.Neg(), maxImpulse)
		impulse = joint.motorImpulse.Sub(oldImpulse)

		wA = wA.Sub(iA.Mul(impulse))
		wB = wB.Add(iB.Mul(impulse))
	}

	if joint.enableLimit && !fixedRotation {
		jointAngle := RelativeAngle(dqB, dqA).Add(joint.deltaAngle).Sub(joint.referenceAngle)
		jointAngle = UnwindAngle(jointAngle)

		// Lower limit
		{
			// D-004: the angle enters the error in radians.
			C := jointAngle.Sub(joint.lowerAngle).Mul(tau)
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

			Cdot := wB.Sub(wA)
			oldImpulse := joint.lowerImpulse
			impulse := massScale.Neg().Mul(joint.axialMass).Mul(Cdot.Add(bias)).Sub(impulseScale.Mul(oldImpulse))
			joint.lowerImpulse = oldImpulse.Add(impulse).Max(zero)
			impulse = joint.lowerImpulse.Sub(oldImpulse)

			wA = wA.Sub(iA.Mul(impulse))
			wB = wB.Add(iB.Mul(impulse))
		}

		// Upper limit
		// Note: signs are flipped to keep C positive when the constraint is satisfied.
		// This also keeps the impulse positive when the limit is active.
		{
			C := joint.upperAngle.Sub(jointAngle).Mul(tau)
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
			Cdot := wA.Sub(wB)
			oldImpulse := joint.upperImpulse
			impulse := massScale.Neg().Mul(joint.axialMass).Mul(Cdot.Add(bias)).Sub(impulseScale.Mul(oldImpulse))
			joint.upperImpulse = oldImpulse.Add(impulse).Max(zero)
			impulse = joint.upperImpulse.Sub(oldImpulse)

			// sign flipped on applied impulse
			wA = wA.Add(iA.Mul(impulse))
			wB = wB.Sub(iB.Mul(impulse))
		}
	}

	// Solve point-to-point constraint
	{
		// J = [-I -r1_skew I r2_skew]
		// r_skew = [-ry; rx]
		// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x]
		//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB]

		// current anchors
		rA := RotateVector(stateA.deltaRotation, joint.anchorA)
		rB := RotateVector(stateB.deltaRotation, joint.anchorB)

		Cdot := vB.Add(CrossSV(wB, rB)).Sub(vA.Add(CrossSV(wA, rA)))

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
		wA = wA.Sub(iA.Mul(Cross(rA, impulse)))
		vB = MulAdd(vB, mB, impulse)
		wB = wB.Add(iB.Mul(Cross(rB, impulse)))
	}

	stateA.linearVelocity = vA
	stateA.angularVelocity = wA.Div(tau)
	stateB.linearVelocity = vB
	stateB.angularVelocity = wB.Div(tau)
}
