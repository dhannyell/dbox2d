package b2

// This file corresponds to src/prismatic_joint.c of the reference. The
// joint slides along the x-axis of frame A; the angular error enters C in
// radians.

// prismaticAxis returns the x-axis of a frame rotation.
func prismaticAxis(q Rot) Vec2 {
	return RotateVector(q, Vec2{X: QOne(), Y: QZero()})
}

// getPrismaticJointForce reports the constraint force of the last step. It
// corresponds to b2GetPrismaticJointForce in src/prismatic_joint.c.
func getPrismaticJointForce(w *world, base *jointSim) Vec2 {
	qA := getBodyTransform(w, base.bodyIdA).Q

	joint := base.prismatic()

	localAxisA := prismaticAxis(base.localFrameA.Q)
	axisA := RotateVector(qA, localAxisA)
	perpA := LeftPerp(axisA)

	invH := w.invH
	perpForce := invH.Mul(joint.impulse.X)
	axialForce := invH.Mul(joint.motorImpulse.Add(joint.lowerImpulse).Sub(joint.upperImpulse))

	return perpA.Mul(perpForce).Add(axisA.Mul(axialForce))
}

// getPrismaticJointTorque reports the constraint torque of the last step.
// It corresponds to b2GetPrismaticJointTorque in src/prismatic_joint.c.
func getPrismaticJointTorque(w *world, base *jointSim) Q {
	return w.invH.Mul(base.prismatic().impulse.Y)
}

// getPrismaticJointSpeed reports the translation speed along the joint
// axis. It corresponds to b2PrismaticJoint_GetCurrentSpeed. A body outside
// the awake set has no state and counts as at rest.
func getPrismaticJointSpeed(w *world, base *jointSim) Q {
	bodyA := &w.bodies[base.bodyIdA]
	bodyB := &w.bodies[base.bodyIdB]
	bodySimA := getBodySim(w, bodyA)
	bodySimB := getBodySim(w, bodyB)
	bodyStateA := getBodyState(w, bodyA)
	bodyStateB := getBodyState(w, bodyB)

	qA := bodySimA.transform.Q
	qB := bodySimB.transform.Q

	localAxisA := prismaticAxis(base.localFrameA.Q)
	axisA := RotateVector(qA, localAxisA)
	rA := RotateVector(qA, base.localFrameA.P.Sub(bodySimA.localCenter))
	rB := RotateVector(qB, base.localFrameB.P.Sub(bodySimB.localCenter))

	dc := bodySimB.center.Sub(bodySimA.center)
	d := dc.Add(rB.Sub(rA))

	vA, vB := Vec2Zero(), Vec2Zero()
	wA, wB := QZero(), QZero()
	if bodyStateA != nil {
		vA = bodyStateA.linearVelocity
		wA = bodyStateA.angularVelocity
	}
	if bodyStateB != nil {
		vB = bodyStateB.linearVelocity
		wB = bodyStateB.angularVelocity
	}

	vRel := vB.Add(CrossSV(wB, rB)).Sub(vA.Add(CrossSV(wA, rA)))
	return d.Dot(CrossSV(wA, axisA)).Add(axisA.Dot(vRel))
}

// Linear constraint (point-to-line)
// d = pB - pA = xB + rB - xA - rA
// C = dot(perp, d)
// Cdot = dot(d, cross(wA, perp)) + dot(perp, vB + cross(wB, rB) - vA - cross(wA, rA))
//      = -dot(perp, vA) - dot(cross(rA + d, perp), wA) + dot(perp, vB) + dot(cross(rB, perp), vB)
// J = [-perp, -cross(rA + d, perp), perp, cross(rB, perp)]
//
// Angular constraint
// C = aB - aA + a_initial
// Cdot = wB - wA
// J = [0 0 -1 0 0 1]
//
// K = J * invM * JT
//
// J = [-a -sA a sB]
//     [0  -1  0  1]
// a = perp
// sA = cross(rA + d, a) = cross(pB - xA, a)
// sB = cross(rB, a) = cross(pB - xB, a)

// Motor/Limit linear constraint
// C = dot(axA, d)
// Cdot = -dot(axA, vA) - dot(cross(rA + d, axA), wA) + dot(axA, vB) + dot(cross(rB, axA), vB)
// J = [-axA -cross(rA + d, axA) axA cross(rB, ax1)]

// Predictive limit is applied even when the limit is not active.
// Prevents a constraint speed that can lead to a constraint error in one time step.
// Want C2 = C1 + h * Cdot >= 0
// Or:
// Cdot + C1/h >= 0
// I do not apply a negative constraint error because that is handled in position correction.
// So:
// Cdot + max(C1, 0)/h >= 0

// Block Solver
// We develop a block solver that includes the angular and linear constraints. This makes the limit stiffer.
//
// The Jacobian has 2 rows:
// J = [-uT -s1 uT s2] // linear
//     [0   -1   0  1] // angular
//
// u = perp
// s1 = cross(d + r1, u), s2 = cross(r2, u)
// a1 = cross(d + r1, v), a2 = cross(r2, v)

// preparePrismaticJoint corresponds to b2PreparePrismaticJoint in
// src/prismatic_joint.c.
func preparePrismaticJoint(base *jointSim, context *stepContext) {
	if base.jointType != PrismaticJoint {
		panic("dbox2d: the joint is not a prismatic joint")
	}
	bodySimA, bodySimB, indexA, indexB := prepareJointBodies(base, context)
	joint := base.prismatic()
	joint.indexA = indexA
	joint.indexB = indexB

	joint.frameA = jointFrame(bodySimA, base.localFrameA)
	joint.frameB = jointFrame(bodySimB, base.localFrameB)

	// Compute the initial center delta. Incremental position updates are
	// relative to this.
	joint.deltaCenter = bodySimB.center.Sub(bodySimA.center)

	joint.springSoftness = makeSoft(joint.hertz, joint.dampingRatio, context.h)

	if !context.enableWarmStarting {
		zero := QZero()
		joint.impulse = Vec2Zero()
		joint.springImpulse = zero
		joint.motorImpulse = zero
		joint.lowerImpulse = zero
		joint.upperImpulse = zero
	}
}

// warmStartPrismaticJoint corresponds to b2WarmStartPrismaticJoint in
// src/prismatic_joint.c.
func warmStartPrismaticJoint(base *jointSim, context *stepContext) {
	if base.jointType != PrismaticJoint {
		panic("dbox2d: the joint is not a prismatic joint")
	}

	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState()

	joint := base.prismatic()
	stateA, stateB := jointStates(context.states, &dummyState, joint.indexA, joint.indexB)

	rA := RotateVector(stateA.deltaRotation, joint.frameA.P)
	rB := RotateVector(stateB.deltaRotation, joint.frameB.P)

	d := stateB.deltaPosition.Sub(stateA.deltaPosition).Add(joint.deltaCenter).Add(rB.Sub(rA))

	axisA := prismaticAxis(joint.frameA.Q)
	axisA = RotateVector(stateA.deltaRotation, axisA)

	// impulse is applied at anchor point on body B
	a1 := Cross(rA.Add(d), axisA)
	a2 := Cross(rB, axisA)
	axialImpulse := joint.springImpulse.Add(joint.motorImpulse).Add(joint.lowerImpulse).Sub(joint.upperImpulse)

	// perpendicular constraint
	perpA := LeftPerp(axisA)
	s1 := Cross(rA.Add(d), perpA)
	s2 := Cross(rB, perpA)
	perpImpulse := joint.impulse.X
	angleImpulse := joint.impulse.Y

	P := axisA.Mul(axialImpulse).Add(perpA.Mul(perpImpulse))
	LA := axialImpulse.Mul(a1).Add(perpImpulse.Mul(s1)).Add(angleImpulse)
	LB := axialImpulse.Mul(a2).Add(perpImpulse.Mul(s2)).Add(angleImpulse)

	if stateA.flags&bodyDynamicFlag != 0 {
		stateA.linearVelocity = MulSub(stateA.linearVelocity, mA, P)
		stateA.angularVelocity = stateA.angularVelocity.Sub(iA.Mul(LA))
	}
	if stateB.flags&bodyDynamicFlag != 0 {
		stateB.linearVelocity = MulAdd(stateB.linearVelocity, mB, P)
		stateB.angularVelocity = stateB.angularVelocity.Add(iB.Mul(LB))
	}
}

// solvePrismaticJoint corresponds to b2SolvePrismaticJoint in
// src/prismatic_joint.c.
func solvePrismaticJoint(base *jointSim, context *stepContext, useBias bool) {
	if base.jointType != PrismaticJoint {
		panic("dbox2d: the joint is not a prismatic joint")
	}

	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState()

	joint := base.prismatic()
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

	d := stateB.deltaPosition.Sub(stateA.deltaPosition).Add(joint.deltaCenter).Add(rB.Sub(rA))

	axisA := prismaticAxis(joint.frameA.Q)
	axisA = RotateVector(stateA.deltaRotation, axisA)
	translation := axisA.Dot(d)

	// These scalars are for torques generated by axial forces
	a1 := Cross(rA.Add(d), axisA)
	a2 := Cross(rB, axisA)

	zero := QZero()
	one := QOne()
	k := mA.Add(mB).Add(iA.Mul(a1).Mul(a1)).Add(iB.Mul(a2).Mul(a2))
	// D-006: the reference multiplies by the reciprocal of k.
	axialMass := zero
	if zero.Less(k) {
		axialMass = one.Div(k)
	}

	softness := base.constraintSoftness

	// spring constraint
	if joint.enableSpring {
		// This is a real spring and should be applied even during relax
		C := translation.Sub(joint.targetTranslation)
		bias := joint.springSoftness.biasRate.Mul(C)
		massScale := joint.springSoftness.massScale
		impulseScale := joint.springSoftness.impulseScale

		Cdot := axisA.Dot(vB.Sub(vA)).Add(a2.Mul(wB)).Sub(a1.Mul(wA))
		deltaImpulse := massScale.Neg().Mul(axialMass).Mul(Cdot.Add(bias)).Sub(impulseScale.Mul(joint.springImpulse))
		joint.springImpulse = joint.springImpulse.Add(deltaImpulse)

		P := axisA.Mul(deltaImpulse)
		LA := deltaImpulse.Mul(a1)
		LB := deltaImpulse.Mul(a2)

		vA = MulSub(vA, mA, P)
		wA = wA.Sub(iA.Mul(LA))
		vB = MulAdd(vB, mB, P)
		wB = wB.Add(iB.Mul(LB))
	}

	// Solve motor constraint
	if joint.enableMotor {
		Cdot := axisA.Dot(vB.Sub(vA)).Add(a2.Mul(wB)).Sub(a1.Mul(wA))
		impulse := axialMass.Mul(joint.motorSpeed.Sub(Cdot))
		oldImpulse := joint.motorImpulse
		maxImpulse := context.h.Mul(joint.maxMotorForce)
		joint.motorImpulse = joint.motorImpulse.Add(impulse).Clamp(maxImpulse.Neg(), maxImpulse)
		impulse = joint.motorImpulse.Sub(oldImpulse)

		P := axisA.Mul(impulse)
		LA := impulse.Mul(a1)
		LB := impulse.Mul(a2)

		vA = MulSub(vA, mA, P)
		wA = wA.Sub(iA.Mul(LA))
		vB = MulAdd(vB, mB, P)
		wB = wB.Add(iB.Mul(LB))
	}

	if joint.enableLimit {
		// Clamp the speculative distance to a reasonable value
		speculativeDistance := QMustParse("0.25").Mul(joint.upperTranslation.Sub(joint.lowerTranslation))

		// The reference bounds the speculation to one length unit per meter.
		safe := one

		// Lower limit
		{
			C := translation.Sub(joint.lowerTranslation)

			if C.Less(speculativeDistance) {
				bias := zero
				massScale := one
				impulseScale := zero

				if zero.Less(C) {
					// speculation
					bias = C.Min(safe).Mul(context.invH)
				} else if useBias {
					bias = softness.biasRate.Mul(C)
					massScale = softness.massScale
					impulseScale = softness.impulseScale
				}

				oldImpulse := joint.lowerImpulse
				Cdot := axisA.Dot(vB.Sub(vA)).Add(a2.Mul(wB)).Sub(a1.Mul(wA))
				deltaImpulse := axialMass.Neg().Mul(massScale).Mul(Cdot.Add(bias)).Sub(impulseScale.Mul(oldImpulse))
				joint.lowerImpulse = oldImpulse.Add(deltaImpulse).Max(zero)
				deltaImpulse = joint.lowerImpulse.Sub(oldImpulse)

				P := axisA.Mul(deltaImpulse)
				LA := deltaImpulse.Mul(a1)
				LB := deltaImpulse.Mul(a2)

				vA = MulSub(vA, mA, P)
				wA = wA.Sub(iA.Mul(LA))
				vB = MulAdd(vB, mB, P)
				wB = wB.Add(iB.Mul(LB))
			} else {
				joint.lowerImpulse = zero
			}
		}

		// Upper limit
		// Note: signs are flipped to keep C positive when the constraint is satisfied.
		// This also keeps the impulse positive when the limit is active.
		{
			// sign flipped
			C := joint.upperTranslation.Sub(translation)

			if C.Less(speculativeDistance) {
				bias := zero
				massScale := one
				impulseScale := zero

				if zero.Less(C) {
					// speculation
					bias = C.Min(safe).Mul(context.invH)
				} else if useBias {
					bias = softness.biasRate.Mul(C)
					massScale = softness.massScale
					impulseScale = softness.impulseScale
				}

				oldImpulse := joint.upperImpulse

				// sign flipped
				Cdot := axisA.Dot(vA.Sub(vB)).Add(a1.Mul(wA)).Sub(a2.Mul(wB))
				deltaImpulse := axialMass.Neg().Mul(massScale).Mul(Cdot.Add(bias)).Sub(impulseScale.Mul(oldImpulse))
				joint.upperImpulse = oldImpulse.Add(deltaImpulse).Max(zero)
				deltaImpulse = joint.upperImpulse.Sub(oldImpulse)

				P := axisA.Mul(deltaImpulse)
				LA := deltaImpulse.Mul(a1)
				LB := deltaImpulse.Mul(a2)

				// sign flipped
				vA = MulAdd(vA, mA, P)
				wA = wA.Add(iA.Mul(LA))
				vB = MulSub(vB, mB, P)
				wB = wB.Sub(iB.Mul(LB))
			} else {
				joint.upperImpulse = zero
			}
		}
	}

	// Solve the prismatic constraint in block form
	{
		perpA := LeftPerp(axisA)

		// These scalars are for torques generated by the perpendicular constraint force
		s1 := Cross(d.Add(rA), perpA)
		s2 := Cross(rB, perpA)

		var Cdot Vec2
		Cdot.X = perpA.Dot(vB.Sub(vA)).Add(s2.Mul(wB)).Sub(s1.Mul(wA))
		Cdot.Y = wB.Sub(wA)

		bias := Vec2Zero()
		massScale := one
		impulseScale := zero
		if useBias {
			var C Vec2
			C.X = perpA.Dot(d)
			// relativeAngle(qA, qB) is b2Rot_GetAngle(b2InvMulRot(qA, qB)).
			C.Y = angleRadians(relativeAngle(qA, qB))

			bias = C.Mul(softness.biasRate)
			massScale = softness.massScale
			impulseScale = softness.impulseScale
		}

		k11 := mA.Add(mB).Add(iA.Mul(s1).Mul(s1)).Add(iB.Mul(s2).Mul(s2))
		k12 := iA.Mul(s1).Add(iB.Mul(s2))
		k22 := iA.Add(iB)
		if k22.Eq(zero) {
			// For bodies with fixed rotation.
			k22 = one
		}

		K := Mat22{Cx: Vec2{X: k11, Y: k12}, Cy: Vec2{X: k12, Y: k22}}

		b := Solve22(K, Cdot.Add(bias))
		var deltaImpulse Vec2
		deltaImpulse.X = massScale.Neg().Mul(b.X).Sub(impulseScale.Mul(joint.impulse.X))
		deltaImpulse.Y = massScale.Neg().Mul(b.Y).Sub(impulseScale.Mul(joint.impulse.Y))

		joint.impulse.X = joint.impulse.X.Add(deltaImpulse.X)
		joint.impulse.Y = joint.impulse.Y.Add(deltaImpulse.Y)

		P := perpA.Mul(deltaImpulse.X)
		LA := deltaImpulse.X.Mul(s1).Add(deltaImpulse.Y)
		LB := deltaImpulse.X.Mul(s2).Add(deltaImpulse.Y)

		vA = MulSub(vA, mA, P)
		wA = wA.Sub(iA.Mul(LA))
		vB = MulAdd(vB, mB, P)
		wB = wB.Add(iB.Mul(LB))
	}

	storeJointVelocity(stateA, vA, wA)
	storeJointVelocity(stateB, vB, wB)
}

// drawPrismaticJoint corresponds to b2DrawPrismaticJoint.
func drawPrismaticJoint(draw *DebugDraw, base *jointSim, transformA, transformB Transform, drawScale Q) {
	joint := base.prismatic()

	frameA := MulTransforms(transformA, base.localFrameA)
	frameB := MulTransforms(transformB, base.localFrameB)
	axisA := prismaticAxis(frameA.Q)

	draw.DrawLineFcn(frameA.P, frameB.P, ColorDimGray)

	if joint.enableLimit {
		b := QMustParse("0.25").Mul(drawScale)
		lower := MulAdd(frameA.P, joint.lowerTranslation, axisA)
		upper := MulAdd(frameA.P, joint.upperTranslation, axisA)
		perp := LeftPerp(axisA)
		draw.DrawLineFcn(lower, upper, ColorGray)
		draw.DrawLineFcn(MulSub(lower, b, perp), MulAdd(lower, b, perp), ColorGreen)
		draw.DrawLineFcn(MulSub(upper, b, perp), MulAdd(upper, b, perp), ColorRed)
	} else {
		draw.DrawLineFcn(frameA.P.Sub(axisA), frameA.P.Add(axisA), ColorGray)
	}

	if joint.enableSpring {
		p := MulAdd(frameA.P, joint.targetTranslation, axisA)
		draw.DrawPointFcn(p, QFromInt(8), ColorViolet)
	}

	draw.DrawPointFcn(frameA.P, QFromInt(5), ColorGray)
	draw.DrawPointFcn(frameB.P, QFromInt(5), ColorBlue)
}
