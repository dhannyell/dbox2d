package dbox2d

// This file corresponds to src/revolute_joint.c of the reference. The API
// takes angles in turns (D-004). The joint keeps them in the angle unit of
// the mode, and angleRadians converts an angle where it enters an error C.

func drawRevoluteJoint(draw *DebugDraw, base *jointSim, transformA, transformB Transform, drawSize Q) {
	joint := base.revolute()
	pA := TransformPoint(transformA, base.localOriginAnchorA)
	pB := TransformPoint(transformB, base.localOriginAnchorB)
	angle := RelativeAngle(transformB.Q, transformA.Q)
	rot := MakeRot(angle)
	pC := pB.Add(Vec2{X: drawSize.Mul(rot.Cos), Y: drawSize.Mul(rot.Sin)})
	draw.DrawCircle(pB, drawSize, ColorGray)
	draw.DrawSegment(pB, pC, ColorGray)
	if draw.DrawJointExtras {
		degrees := UnwindAngle(angle.Sub(angleToTurns(joint.referenceAngle))).Mul(QFromInt(360))
		draw.DrawString(pC, " "+drawNumber(degrees, 1)+" deg", ColorWhite)
	}
	if joint.enableLimit {
		for _, limit := range []struct {
			angle Q
			color HexColor
		}{
			{angleToTurns(joint.lowerAngle.Add(joint.referenceAngle)), ColorGreen},
			{angleToTurns(joint.upperAngle.Add(joint.referenceAngle)), ColorRed},
			{angleToTurns(joint.referenceAngle), ColorBlue},
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
	force := base.revolute().linearImpulse.Mul(w.invH)
	return force
}

// getRevoluteJointTorque reports the constraint torque of the last step.
// It corresponds to b2GetRevoluteJointTorque in src/revolute_joint.c.
func getRevoluteJointTorque(w *world, base *jointSim) Q {
	revolute := base.revolute()
	torque := w.invH.Mul(revolute.motorImpulse.Add(revolute.lowerImpulse).Sub(revolute.upperImpulse))
	return torque
}

// SetTargetAngle changes the revolute spring target angle in turns.
func (jointId JointId) SetTargetAngle(angle Q) {
	w := getWorld(jointId.world0)
	joint := getJointSimCheckType(w, jointId, RevoluteJoint)
	// D-004: the target angle is bounded to half a turn.
	halfTurn := QHalf()
	joint.revolute().targetAngle = angleFromTurns(angle.Clamp(halfTurn.Neg(), halfTurn))
}

// GetTargetAngle reports the revolute spring target angle in turns.
func (jointId JointId) GetTargetAngle() Q {
	w := getWorld(jointId.world0)
	joint := getJointSimCheckType(w, jointId, RevoluteJoint)
	return angleToTurns(joint.revolute().targetAngle)
}

// GetAngle reports the revolute joint angle relative to its reference angle,
// in turns.
func (jointId JointId) GetAngle() Q {
	w := getWorld(jointId.world0)
	joint := getJointSimCheckType(w, jointId, RevoluteJoint)
	transformA := getBodyTransform(w, joint.bodyIdA)
	transformB := getBodyTransform(w, joint.bodyIdB)
	angle := relativeAngle(transformB.Q, transformA.Q).Sub(joint.revolute().referenceAngle)
	return angleToTurns(unwindAngle(angle))
}

// GetLowerLimit reports the lower angular or linear joint limit.
func (jointId JointId) GetLowerLimit() Q {
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	joint := getJointSim(w, j)
	switch j.jointType {
	case RevoluteJoint:
		return angleToTurns(joint.revolute().lowerAngle)
	case PrismaticJoint:
		return joint.prismatic().lowerTranslation
	case WheelJoint:
		return joint.wheel().lowerTranslation
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
		return angleToTurns(joint.revolute().upperAngle)
	case PrismaticJoint:
		return joint.prismatic().upperTranslation
	case WheelJoint:
		return joint.wheel().upperTranslation
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
		halfTurn := QHalf()
		lower = lower.Clamp(halfTurn.Neg(), halfTurn)
		upper = upper.Clamp(halfTurn.Neg(), halfTurn)
		lowerAngle := angleFromTurns(lower.Min(upper))
		upperAngle := angleFromTurns(lower.Max(upper))
		if !lowerAngle.Eq(joint.revolute().lowerAngle) || !upperAngle.Eq(joint.revolute().upperAngle) {
			joint.revolute().lowerAngle = lowerAngle
			joint.revolute().upperAngle = upperAngle
			joint.revolute().lowerImpulse = QZero()
			joint.revolute().upperImpulse = QZero()
		}
	case PrismaticJoint:
		if !lower.Eq(joint.prismatic().lowerTranslation) || !upper.Eq(joint.prismatic().upperTranslation) {
			joint.prismatic().lowerTranslation = lower.Min(upper)
			joint.prismatic().upperTranslation = lower.Max(upper)
			joint.prismatic().lowerImpulse = QZero()
			joint.prismatic().upperImpulse = QZero()
		}
	case WheelJoint:
		if !lower.Eq(joint.wheel().lowerTranslation) || !upper.Eq(joint.wheel().upperTranslation) {
			joint.wheel().lowerTranslation = lower.Min(upper)
			joint.wheel().upperTranslation = lower.Max(upper)
			joint.wheel().lowerImpulse = QZero()
			joint.wheel().upperImpulse = QZero()
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
		return w.invH.Mul(joint.revolute().motorImpulse)
	case WheelJoint:
		return w.invH.Mul(joint.wheel().motorImpulse)
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
		joint.revolute().maxMotorTorque = torque
	case WheelJoint:
		joint.wheel().maxMotorTorque = torque
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
		return joint.revolute().maxMotorTorque
	case WheelJoint:
		return joint.wheel().maxMotorTorque
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

	joint := base.revolute()

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
	joint.deltaAngle = relativeAngle(bodySimB.transform.Q, bodySimA.transform.Q)

	zero := QZero()
	k := iA.Add(iB)
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

	rA := RotateVector(stateA.deltaRotation, joint.anchorA)
	rB := RotateVector(stateB.deltaRotation, joint.anchorB)

	axialImpulse := joint.springImpulse.Add(joint.motorImpulse).Add(joint.lowerImpulse).Sub(joint.upperImpulse)

	stateA.linearVelocity = MulSub(stateA.linearVelocity, mA, joint.linearImpulse)
	wA := stateA.angularVelocity
	wA = wA.Sub(iA.Mul(Cross(rA, joint.linearImpulse).Add(axialImpulse)))
	stateA.angularVelocity = wA

	stateB.linearVelocity = MulAdd(stateB.linearVelocity, mB, joint.linearImpulse)
	wB := stateB.angularVelocity
	wB = wB.Add(iB.Mul(Cross(rB, joint.linearImpulse).Add(axialImpulse)))
	stateB.angularVelocity = wB
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

	joint := base.revolute()

	stateA, stateB := jointStates(context.states, &dummyState, joint.indexA, joint.indexB)

	vA := stateA.linearVelocity
	wA := stateA.angularVelocity
	vB := stateB.linearVelocity
	wB := stateB.angularVelocity

	dqA := stateA.deltaRotation
	dqB := stateB.deltaRotation

	zero := QZero()
	one := QOne()
	fixedRotation := iA.Add(iB).Eq(zero)

	// Solve spring.
	if joint.enableSpring && !fixedRotation {
		jointAngle := relativeAngle(stateB.deltaRotation, stateA.deltaRotation).Add(joint.deltaAngle)
		jointAngleDelta := unwindAngle(jointAngle.Sub(joint.targetAngle))

		C := angleRadians(jointAngleDelta)
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
		Cdot := wB.Sub(wA).Sub(angleRadians(joint.motorSpeed))
		impulse := joint.axialMass.Neg().Mul(Cdot)
		oldImpulse := joint.motorImpulse
		maxImpulse := context.h.Mul(joint.maxMotorTorque)
		joint.motorImpulse = joint.motorImpulse.Add(impulse).Clamp(maxImpulse.Neg(), maxImpulse)
		impulse = joint.motorImpulse.Sub(oldImpulse)

		wA = wA.Sub(iA.Mul(impulse))
		wB = wB.Add(iB.Mul(impulse))
	}

	if joint.enableLimit && !fixedRotation {
		jointAngle := relativeAngle(dqB, dqA).Add(joint.deltaAngle).Sub(joint.referenceAngle)
		jointAngle = unwindAngle(jointAngle)

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
	stateA.angularVelocity = wA
	stateB.linearVelocity = vB
	stateB.angularVelocity = wB
}
