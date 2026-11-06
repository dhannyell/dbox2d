package b2

// This file corresponds to src/wheel_joint.c of the reference. The wheel
// slides along the x-axis of frame A. The API takes the motor speed in
// turns per second (D-004); the joint keeps it in the angle unit of the
// mode and angleRadians converts it where it meets an angular velocity.

// getWheelJointForce reports the constraint force of the last step. It
// corresponds to b2GetWheelJointForce in src/wheel_joint.c.
func getWheelJointForce(w *world, base *jointSim) Vec2 {
	qA := getBodyTransform(w, base.bodyIdA).Q

	localAxisA := prismaticAxis(base.localFrameA.Q)
	axisA := RotateVector(qA, localAxisA)
	perpA := LeftPerp(axisA)

	joint := base.wheel()

	perpForce := w.invH.Mul(joint.perpImpulse)
	axialForce := w.invH.Mul(joint.springImpulse.Add(joint.lowerImpulse).Sub(joint.upperImpulse))

	return perpA.Mul(perpForce).Add(axisA.Mul(axialForce))
}

// getWheelJointTorque reports the constraint torque of the last step. It
// corresponds to b2GetWheelJointTorque in src/wheel_joint.c.
func getWheelJointTorque(w *world, base *jointSim) Q {
	return w.invH.Mul(base.wheel().motorImpulse)
}

// Linear constraint (point-to-line)
// d = pB - pA = xB + rB - xA - rA
// C = dot(ay, d)
// Cdot = dot(d, cross(wA, ay)) + dot(ay, vB + cross(wB, rB) - vA - cross(wA, rA))
//      = -dot(ay, vA) - dot(cross(d + rA, ay), wA) + dot(ay, vB) + dot(cross(rB, ay), vB)
// J = [-ay, -cross(d + rA, ay), ay, cross(rB, ay)]

// Spring linear constraint
// C = dot(ax, d)
// Cdot = = -dot(ax, vA) - dot(cross(d + rA, ax), wA) + dot(ax, vB) + dot(cross(rB, ax), vB)
// J = [-ax -cross(d+rA, ax) ax cross(rB, ax)]

// Motor rotational constraint
// Cdot = wB - wA
// J = [0 0 -1 0 0 1]

// prepareWheelJoint corresponds to b2PrepareWheelJoint in
// src/wheel_joint.c.
func prepareWheelJoint(base *jointSim, context *stepContext) {
	if base.jointType != WheelJoint {
		panic("dbox2d: the joint is not a wheel joint")
	}
	bodySimA, bodySimB, indexA, indexB := prepareJointBodies(base, context)
	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	joint := base.wheel()
	joint.indexA = indexA
	joint.indexB = indexB

	joint.frameA = jointFrame(bodySimA, base.localFrameA)
	joint.frameB = jointFrame(bodySimB, base.localFrameB)

	// Compute the initial center delta. Incremental position updates are
	// relative to this.
	joint.deltaCenter = bodySimB.center.Sub(bodySimA.center)

	rA := joint.frameA.P
	rB := joint.frameB.P

	d := joint.deltaCenter.Add(rB.Sub(rA))
	axisA := prismaticAxis(joint.frameA.Q)
	perpA := LeftPerp(axisA)

	zero := QZero()
	one := QOne()

	// perpendicular constraint (keep wheel on line)
	s1 := Cross(d.Add(rA), perpA)
	s2 := Cross(rB, perpA)

	// D-006: the reference multiplies by the reciprocals of the masses.
	kp := mA.Add(mB).Add(iA.Mul(s1).Mul(s1)).Add(iB.Mul(s2).Mul(s2))
	joint.perpMass = zero
	if zero.Less(kp) {
		joint.perpMass = one.Div(kp)
	}

	// spring constraint
	a1 := Cross(d.Add(rA), axisA)
	a2 := Cross(rB, axisA)

	ka := mA.Add(mB).Add(iA.Mul(a1).Mul(a1)).Add(iB.Mul(a2).Mul(a2))
	joint.axialMass = zero
	if zero.Less(ka) {
		joint.axialMass = one.Div(ka)
	}

	joint.springSoftness = makeSoft(joint.hertz, joint.dampingRatio, context.h)

	km := iA.Add(iB)
	joint.motorMass = zero
	if zero.Less(km) {
		joint.motorMass = one.Div(km)
	}

	if !context.enableWarmStarting {
		joint.perpImpulse = zero
		joint.springImpulse = zero
		joint.motorImpulse = zero
		joint.lowerImpulse = zero
		joint.upperImpulse = zero
	}
}

// warmStartWheelJoint corresponds to b2WarmStartWheelJoint in
// src/wheel_joint.c.
func warmStartWheelJoint(base *jointSim, context *stepContext) {
	if base.jointType != WheelJoint {
		panic("dbox2d: the joint is not a wheel joint")
	}

	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState()

	joint := base.wheel()
	stateA, stateB := jointStates(context.states, &dummyState, joint.indexA, joint.indexB)

	rA := RotateVector(stateA.deltaRotation, joint.frameA.P)
	rB := RotateVector(stateB.deltaRotation, joint.frameB.P)

	d := stateB.deltaPosition.Sub(stateA.deltaPosition).Add(joint.deltaCenter).Add(rB.Sub(rA))
	axisA := prismaticAxis(joint.frameA.Q)
	axisA = RotateVector(stateA.deltaRotation, axisA)
	perpA := LeftPerp(axisA)

	a1 := Cross(d.Add(rA), axisA)
	a2 := Cross(rB, axisA)
	s1 := Cross(d.Add(rA), perpA)
	s2 := Cross(rB, perpA)

	axialImpulse := joint.springImpulse.Add(joint.lowerImpulse).Sub(joint.upperImpulse)

	P := axisA.Mul(axialImpulse).Add(perpA.Mul(joint.perpImpulse))
	LA := axialImpulse.Mul(a1).Add(joint.perpImpulse.Mul(s1)).Add(joint.motorImpulse)
	LB := axialImpulse.Mul(a2).Add(joint.perpImpulse.Mul(s2)).Add(joint.motorImpulse)

	if stateA.flags&bodyDynamicFlag != 0 {
		stateA.linearVelocity = MulSub(stateA.linearVelocity, mA, P)
		stateA.angularVelocity = stateA.angularVelocity.Sub(iA.Mul(LA))
	}
	if stateB.flags&bodyDynamicFlag != 0 {
		stateB.linearVelocity = MulAdd(stateB.linearVelocity, mB, P)
		stateB.angularVelocity = stateB.angularVelocity.Add(iB.Mul(LB))
	}
}

// solveWheelJoint corresponds to b2SolveWheelJoint in src/wheel_joint.c.
func solveWheelJoint(base *jointSim, context *stepContext, useBias bool) {
	if base.jointType != WheelJoint {
		panic("dbox2d: the joint is not a wheel joint")
	}

	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState()

	joint := base.wheel()
	stateA, stateB := jointStates(context.states, &dummyState, joint.indexA, joint.indexB)

	vA := stateA.linearVelocity
	wA := stateA.angularVelocity
	vB := stateB.linearVelocity
	wB := stateB.angularVelocity

	zero := QZero()
	one := QOne()
	fixedRotation := iA.Add(iB).Eq(zero)

	// current anchors
	rA := RotateVector(stateA.deltaRotation, joint.frameA.P)
	rB := RotateVector(stateB.deltaRotation, joint.frameB.P)

	d := stateB.deltaPosition.Sub(stateA.deltaPosition).Add(joint.deltaCenter).Add(rB.Sub(rA))
	axisA := prismaticAxis(joint.frameA.Q)
	axisA = RotateVector(stateA.deltaRotation, axisA)
	translation := axisA.Dot(d)

	a1 := Cross(d.Add(rA), axisA)
	a2 := Cross(rB, axisA)

	// motor constraint
	if joint.enableMotor && !fixedRotation {
		Cdot := wB.Sub(wA).Sub(angleRadians(joint.motorSpeed))
		impulse := joint.motorMass.Neg().Mul(Cdot)
		oldImpulse := joint.motorImpulse
		maxImpulse := context.h.Mul(joint.maxMotorTorque)
		joint.motorImpulse = joint.motorImpulse.Add(impulse).Clamp(maxImpulse.Neg(), maxImpulse)
		impulse = joint.motorImpulse.Sub(oldImpulse)

		wA = wA.Sub(iA.Mul(impulse))
		wB = wB.Add(iB.Mul(impulse))
	}

	// spring constraint
	if joint.enableSpring {
		// This is a real spring and should be applied even during relax
		C := translation
		bias := joint.springSoftness.biasRate.Mul(C)
		massScale := joint.springSoftness.massScale
		impulseScale := joint.springSoftness.impulseScale

		Cdot := axisA.Dot(vB.Sub(vA)).Add(a2.Mul(wB)).Sub(a1.Mul(wA))
		impulse := massScale.Neg().Mul(joint.axialMass).Mul(Cdot.Add(bias)).Sub(impulseScale.Mul(joint.springImpulse))
		joint.springImpulse = joint.springImpulse.Add(impulse)

		P := axisA.Mul(impulse)
		LA := impulse.Mul(a1)
		LB := impulse.Mul(a2)

		vA = MulSub(vA, mA, P)
		wA = wA.Sub(iA.Mul(LA))
		vB = MulAdd(vB, mB, P)
		wB = wB.Add(iB.Mul(LB))
	}

	if joint.enableLimit {
		// Lower limit
		{
			C := translation.Sub(joint.lowerTranslation)
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

			Cdot := axisA.Dot(vB.Sub(vA)).Add(a2.Mul(wB)).Sub(a1.Mul(wA))
			impulse := massScale.Neg().Mul(joint.axialMass).Mul(Cdot.Add(bias)).Sub(impulseScale.Mul(joint.lowerImpulse))
			oldImpulse := joint.lowerImpulse
			joint.lowerImpulse = oldImpulse.Add(impulse).Max(zero)
			impulse = joint.lowerImpulse.Sub(oldImpulse)

			P := axisA.Mul(impulse)
			LA := impulse.Mul(a1)
			LB := impulse.Mul(a2)

			vA = MulSub(vA, mA, P)
			wA = wA.Sub(iA.Mul(LA))
			vB = MulAdd(vB, mB, P)
			wB = wB.Add(iB.Mul(LB))
		}

		// Upper limit
		// Note: signs are flipped to keep C positive when the constraint is satisfied.
		// This also keeps the impulse positive when the limit is active.
		{
			// sign flipped
			C := joint.upperTranslation.Sub(translation)
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
			Cdot := axisA.Dot(vA.Sub(vB)).Add(a1.Mul(wA)).Sub(a2.Mul(wB))
			impulse := massScale.Neg().Mul(joint.axialMass).Mul(Cdot.Add(bias)).Sub(impulseScale.Mul(joint.upperImpulse))
			oldImpulse := joint.upperImpulse
			joint.upperImpulse = oldImpulse.Add(impulse).Max(zero)
			impulse = joint.upperImpulse.Sub(oldImpulse)

			P := axisA.Mul(impulse)
			LA := impulse.Mul(a1)
			LB := impulse.Mul(a2)

			// sign flipped on applied impulse
			vA = MulAdd(vA, mA, P)
			wA = wA.Add(iA.Mul(LA))
			vB = MulSub(vB, mB, P)
			wB = wB.Sub(iB.Mul(LB))
		}
	}

	// point to line constraint
	{
		perpA := LeftPerp(axisA)

		bias := zero
		massScale := one
		impulseScale := zero
		if useBias {
			C := perpA.Dot(d)
			bias = base.constraintSoftness.biasRate.Mul(C)
			massScale = base.constraintSoftness.massScale
			impulseScale = base.constraintSoftness.impulseScale
		}

		s1 := Cross(d.Add(rA), perpA)
		s2 := Cross(rB, perpA)
		Cdot := perpA.Dot(vB.Sub(vA)).Add(s2.Mul(wB)).Sub(s1.Mul(wA))

		impulse := massScale.Neg().Mul(joint.perpMass).Mul(Cdot.Add(bias)).Sub(impulseScale.Mul(joint.perpImpulse))
		joint.perpImpulse = joint.perpImpulse.Add(impulse)

		P := perpA.Mul(impulse)
		LA := impulse.Mul(s1)
		LB := impulse.Mul(s2)

		vA = MulSub(vA, mA, P)
		wA = wA.Sub(iA.Mul(LA))
		vB = MulAdd(vB, mB, P)
		wB = wB.Add(iB.Mul(LB))
	}

	storeJointVelocity(stateA, vA, wA)
	storeJointVelocity(stateB, vB, wB)
}

// drawWheelJoint corresponds to b2DrawWheelJoint.
func drawWheelJoint(draw *DebugDraw, base *jointSim, transformA, transformB Transform, drawScale Q) {
	joint := base.wheel()

	frameA := MulTransforms(transformA, base.localFrameA)
	frameB := MulTransforms(transformB, base.localFrameB)
	axisA := prismaticAxis(frameA.Q)

	c1 := ColorGray
	c2 := ColorGreen
	c3 := ColorRed
	c4 := ColorDimGray
	c5 := ColorBlue

	draw.DrawSegment(frameA.P, frameB.P, c5)

	if joint.enableLimit {
		lower := MulAdd(frameA.P, joint.lowerTranslation, axisA)
		upper := MulAdd(frameA.P, joint.upperTranslation, axisA)
		perp := LeftPerp(axisA)
		b := QMustParse("0.1").Mul(drawScale)
		draw.DrawSegment(lower, upper, c1)
		draw.DrawSegment(MulSub(lower, b, perp), MulAdd(lower, b, perp), c2)
		draw.DrawSegment(MulSub(upper, b, perp), MulAdd(upper, b, perp), c3)
	} else {
		draw.DrawSegment(frameA.P.Sub(axisA), frameA.P.Add(axisA), c1)
	}

	draw.DrawPoint(frameA.P, QFromInt(5), c1)
	draw.DrawPoint(frameB.P, QFromInt(5), c4)
}
