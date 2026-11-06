package b2

// This file corresponds to src/distance_joint.c of the reference.

// drawDistanceJoint corresponds to b2DrawDistanceJoint.
func drawDistanceJoint(draw *DebugDraw, base *jointSim, transformA, transformB Transform) {
	joint := base.distance()
	pA := TransformPoint(transformA, base.localFrameA.P)
	pB := TransformPoint(transformB, base.localFrameB.P)
	axis := pB.Sub(pA).Normalize()
	if joint.enableLimit && joint.minLength.Less(joint.maxLength) {
		pMin := MulAdd(pA, joint.minLength, axis)
		pMax := MulAdd(pA, joint.maxLength, axis)
		offset := RightPerp(axis).Mul(QMustParse("0.05"))
		if linearSlop.Less(joint.minLength) {
			draw.DrawSegment(pMin.Sub(offset), pMin.Add(offset), ColorLightGreen)
		}
		if joint.maxLength.Less(Huge) {
			draw.DrawSegment(pMax.Sub(offset), pMax.Add(offset), ColorRed)
		}
		if linearSlop.Less(joint.minLength) && joint.maxLength.Less(Huge) {
			draw.DrawSegment(pMin, pMax, ColorGray)
		}
	}
	draw.DrawSegment(pA, pB, ColorWhite)
	draw.DrawPoint(pA, QFromInt(4), ColorWhite)
	draw.DrawPoint(pB, QFromInt(4), ColorWhite)
	if joint.enableSpring && (Q{}).Less(joint.hertz) {
		draw.DrawPoint(MulAdd(pA, joint.length, axis), QFromInt(4), ColorBlue)
	}
}

// getDistanceJointForce reports the constraint force of the last step. It
// corresponds to b2GetDistanceJointForce in src/distance_joint.c.
func getDistanceJointForce(w *world, base *jointSim) Vec2 {
	joint := base.distance()

	transformA := getBodyTransform(w, base.bodyIdA)
	transformB := getBodyTransform(w, base.bodyIdB)

	pA := TransformPoint(transformA, base.localFrameA.P)
	pB := TransformPoint(transformB, base.localFrameB.P)
	d := pB.Sub(pA)
	axis := d.Normalize()
	force := joint.impulse.Add(joint.lowerImpulse).Sub(joint.upperImpulse).Add(joint.motorImpulse).Mul(w.invH)
	return axis.Mul(force)
}

// 1-D constrained system
// m (v2 - v1) = lambda
// v2 + (beta/h) * x1 + gamma * lambda = 0, gamma has units of inverse mass.
// x2 = x1 + h * v2

// 1-D mass-damper-spring system
// m (v2 - v1) + h * d * v2 + h * k *

// C = norm(p2 - p1) - L
// u = (p2 - p1) / norm(p2 - p1)
// Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
// J = [-u -cross(r1, u) u cross(r2, u)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

// prepareDistanceJoint corresponds to b2PrepareDistanceJoint in
// src/distance_joint.c.
func prepareDistanceJoint(base *jointSim, context *stepContext) {
	if base.jointType != DistanceJoint {
		panic("dbox2d: the joint is not a distance joint")
	}

	bodySimA, bodySimB, indexA, indexB := prepareJointBodies(base, context)
	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	joint := base.distance()
	joint.indexA = indexA
	joint.indexB = indexB

	// initial anchors in world space
	joint.anchorA = RotateVector(bodySimA.transform.Q, base.localFrameA.P.Sub(bodySimA.localCenter))
	joint.anchorB = RotateVector(bodySimB.transform.Q, base.localFrameB.P.Sub(bodySimB.localCenter))
	joint.deltaCenter = bodySimB.center.Sub(bodySimA.center)

	rA := joint.anchorA
	rB := joint.anchorB
	separation := rB.Sub(rA).Add(joint.deltaCenter)
	axis := separation.Normalize()

	// compute effective mass
	crA := Cross(rA, axis)
	crB := Cross(rB, axis)
	k := mA.Add(mB).Add(iA.Mul(crA).Mul(crA)).Add(iB.Mul(crB).Mul(crB))
	// D-006: the reference multiplies by the reciprocal of k.
	zero := QZero()
	joint.axialMass = zero
	if zero.Less(k) {
		joint.axialMass = QOne().Div(k)
	}

	joint.distanceSoftness = makeSoft(joint.hertz, joint.dampingRatio, context.h)

	if !context.enableWarmStarting {
		joint.impulse = zero
		joint.lowerImpulse = zero
		joint.upperImpulse = zero
		joint.motorImpulse = zero
	}
}

// warmStartDistanceJoint corresponds to b2WarmStartDistanceJoint in
// src/distance_joint.c.
func warmStartDistanceJoint(base *jointSim, context *stepContext) {
	if base.jointType != DistanceJoint {
		panic("dbox2d: the joint is not a distance joint")
	}

	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState()

	joint := base.distance()
	stateA, stateB := jointStates(context.states, &dummyState, joint.indexA, joint.indexB)

	rA := RotateVector(stateA.deltaRotation, joint.anchorA)
	rB := RotateVector(stateB.deltaRotation, joint.anchorB)

	ds := stateB.deltaPosition.Sub(stateA.deltaPosition).Add(rB.Sub(rA))
	separation := joint.deltaCenter.Add(ds)
	axis := separation.Normalize()

	axialImpulse := joint.impulse.Add(joint.lowerImpulse).Sub(joint.upperImpulse).Add(joint.motorImpulse)
	P := axis.Mul(axialImpulse)

	if stateA.flags&bodyDynamicFlag != 0 {
		stateA.linearVelocity = MulSub(stateA.linearVelocity, mA, P)
		stateA.angularVelocity = stateA.angularVelocity.Sub(iA.Mul(Cross(rA, P)))
	}
	if stateB.flags&bodyDynamicFlag != 0 {
		stateB.linearVelocity = MulAdd(stateB.linearVelocity, mB, P)
		stateB.angularVelocity = stateB.angularVelocity.Add(iB.Mul(Cross(rB, P)))
	}
}

// solveDistanceJoint corresponds to b2SolveDistanceJoint in
// src/distance_joint.c.
func solveDistanceJoint(base *jointSim, context *stepContext, useBias bool) {
	if base.jointType != DistanceJoint {
		panic("dbox2d: the joint is not a distance joint")
	}

	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState()

	joint := base.distance()
	stateA, stateB := jointStates(context.states, &dummyState, joint.indexA, joint.indexB)

	vA := stateA.linearVelocity
	wA := stateA.angularVelocity
	vB := stateB.linearVelocity
	wB := stateB.angularVelocity

	// current anchors
	rA := RotateVector(stateA.deltaRotation, joint.anchorA)
	rB := RotateVector(stateB.deltaRotation, joint.anchorB)

	// current separation
	ds := stateB.deltaPosition.Sub(stateA.deltaPosition).Add(rB.Sub(rA))
	separation := joint.deltaCenter.Add(ds)

	length := separation.Len()
	axis := separation.Normalize()

	zero := QZero()
	one := QOne()

	// joint is soft if
	// - spring is enabled
	// - and (joint limit is disabled or limits are not equal)
	if joint.enableSpring && (joint.minLength.Less(joint.maxLength) || !joint.enableLimit) {
		// spring
		if zero.Less(joint.hertz) {
			// Cdot = dot(u, v + cross(w, r))
			vr := vB.Sub(vA).Add(CrossSV(wB, rB).Sub(CrossSV(wA, rA)))
			Cdot := axis.Dot(vr)
			C := length.Sub(joint.length)
			bias := joint.distanceSoftness.biasRate.Mul(C)

			m := joint.distanceSoftness.massScale.Mul(joint.axialMass)
			oldImpulse := joint.impulse
			impulse := m.Neg().Mul(Cdot.Add(bias)).Sub(joint.distanceSoftness.impulseScale.Mul(oldImpulse))

			h := context.h
			joint.impulse = joint.impulse.Add(impulse).Clamp(joint.lowerSpringForce.Mul(h), joint.upperSpringForce.Mul(h))
			impulse = joint.impulse.Sub(oldImpulse)

			P := axis.Mul(impulse)
			vA = MulSub(vA, mA, P)
			wA = wA.Sub(iA.Mul(Cross(rA, P)))
			vB = MulAdd(vB, mB, P)
			wB = wB.Add(iB.Mul(Cross(rB, P)))
		}

		if joint.enableMotor {
			vr := vB.Sub(vA).Add(CrossSV(wB, rB).Sub(CrossSV(wA, rA)))
			Cdot := axis.Dot(vr)
			impulse := joint.axialMass.Mul(joint.motorSpeed.Sub(Cdot))
			oldImpulse := joint.motorImpulse
			maxImpulse := context.h.Mul(joint.maxMotorForce)
			joint.motorImpulse = joint.motorImpulse.Add(impulse).Clamp(maxImpulse.Neg(), maxImpulse)
			impulse = joint.motorImpulse.Sub(oldImpulse)

			P := axis.Mul(impulse)
			vA = MulSub(vA, mA, P)
			wA = wA.Sub(iA.Mul(Cross(rA, P)))
			vB = MulAdd(vB, mB, P)
			wB = wB.Add(iB.Mul(Cross(rB, P)))
		}

		if joint.enableLimit {
			// lower limit
			{
				vr := vB.Sub(vA).Add(CrossSV(wB, rB).Sub(CrossSV(wA, rA)))
				Cdot := axis.Dot(vr)

				C := length.Sub(joint.minLength)

				bias := zero
				massScale := one
				impulseScale := zero
				if zero.Less(C) {
					// speculative
					bias = C.Mul(context.invH)
				} else if useBias {
					bias = base.constraintSoftness.biasRate.Mul(C)
					massScale = base.constraintSoftness.massScale
					impulseScale = base.constraintSoftness.impulseScale
				}

				impulse := massScale.Neg().Mul(joint.axialMass).Mul(Cdot.Add(bias)).Sub(impulseScale.Mul(joint.lowerImpulse))
				newImpulse := zero.Max(joint.lowerImpulse.Add(impulse))
				impulse = newImpulse.Sub(joint.lowerImpulse)
				joint.lowerImpulse = newImpulse

				P := axis.Mul(impulse)
				vA = MulSub(vA, mA, P)
				wA = wA.Sub(iA.Mul(Cross(rA, P)))
				vB = MulAdd(vB, mB, P)
				wB = wB.Add(iB.Mul(Cross(rB, P)))
			}

			// upper
			{
				vr := vA.Sub(vB).Add(CrossSV(wA, rA).Sub(CrossSV(wB, rB)))
				Cdot := axis.Dot(vr)

				C := joint.maxLength.Sub(length)

				bias := zero
				massScale := one
				impulseScale := zero
				if zero.Less(C) {
					// speculative
					bias = C.Mul(context.invH)
				} else if useBias {
					bias = base.constraintSoftness.biasRate.Mul(C)
					massScale = base.constraintSoftness.massScale
					impulseScale = base.constraintSoftness.impulseScale
				}

				impulse := massScale.Neg().Mul(joint.axialMass).Mul(Cdot.Add(bias)).Sub(impulseScale.Mul(joint.upperImpulse))
				newImpulse := zero.Max(joint.upperImpulse.Add(impulse))
				impulse = newImpulse.Sub(joint.upperImpulse)
				joint.upperImpulse = newImpulse

				P := axis.Mul(impulse.Neg())
				vA = MulSub(vA, mA, P)
				wA = wA.Sub(iA.Mul(Cross(rA, P)))
				vB = MulAdd(vB, mB, P)
				wB = wB.Add(iB.Mul(Cross(rB, P)))
			}
		}
	} else {
		// rigid constraint
		vr := vB.Sub(vA).Add(CrossSV(wB, rB).Sub(CrossSV(wA, rA)))
		Cdot := axis.Dot(vr)

		C := length.Sub(joint.length)

		bias := zero
		massScale := one
		impulseScale := zero
		if useBias {
			bias = base.constraintSoftness.biasRate.Mul(C)
			massScale = base.constraintSoftness.massScale
			impulseScale = base.constraintSoftness.impulseScale
		}

		impulse := massScale.Neg().Mul(joint.axialMass).Mul(Cdot.Add(bias)).Sub(impulseScale.Mul(joint.impulse))
		joint.impulse = joint.impulse.Add(impulse)

		P := axis.Mul(impulse)
		vA = MulSub(vA, mA, P)
		wA = wA.Sub(iA.Mul(Cross(rA, P)))
		vB = MulAdd(vB, mB, P)
		wB = wB.Add(iB.Mul(Cross(rB, P)))
	}

	storeJointVelocity(stateA, vA, wA)
	storeJointVelocity(stateB, vB, wB)
}
