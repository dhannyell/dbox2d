package dbox2d

import "github.com/dhannyell/fixed"

func drawPrismaticJoint(draw *DebugDraw, base *jointSim, transformA, transformB Transform) {
	joint := &base.prismaticJoint
	pA := TransformPoint(transformA, base.localOriginAnchorA)
	pB := TransformPoint(transformB, base.localOriginAnchorB)
	axis := RotateVector(transformA.Q, joint.localAxisA)
	draw.DrawSegment(pA, pB, ColorDimGray)
	if joint.enableLimit {
		lower := MulAdd(pA, joint.lowerTranslation, axis)
		upper := MulAdd(pA, joint.upperTranslation, axis)
		perp := LeftPerp(axis).Mul(fixed.Q32MustParse("0.1"))
		draw.DrawSegment(lower, upper, ColorGray)
		draw.DrawSegment(lower.Sub(perp), lower.Add(perp), ColorGreen)
		draw.DrawSegment(upper.Sub(perp), upper.Add(perp), ColorRed)
	} else {
		draw.DrawSegment(pA.Sub(axis), pA.Add(axis), ColorGray)
	}
	draw.DrawPoint(pA, fixed.Q32FromInt(5), ColorGray)
	draw.DrawPoint(pB, fixed.Q32FromInt(5), ColorBlue)
}

// This file corresponds to src/prismatic_joint.c of the reference. The
// angles are turns (D-004); the angular error enters C in radians.

// getPrismaticJointForce reports the constraint force of the last step. It
// corresponds to b2GetPrismaticJointForce in src/prismatic_joint.c.
func getPrismaticJointForce(w *world, base *jointSim) Vec2 {
	idA := base.bodyIdA
	transformA := getBodyTransform(w, idA)

	joint := &base.prismaticJoint

	axisA := RotateVector(transformA.Q, joint.localAxisA)
	perpA := LeftPerp(axisA)

	invH := w.invH
	perpForce := invH.Mul(joint.impulse.X)
	axialForce := invH.Mul(joint.motorImpulse.Add(joint.lowerImpulse).Sub(joint.upperImpulse))

	force := perpA.Mul(perpForce).Add(axisA.Mul(axialForce))
	return force
}

// getPrismaticJointTorque reports the constraint torque of the last step.
// It corresponds to b2GetPrismaticJointTorque in src/prismatic_joint.c.
func getPrismaticJointTorque(w *world, base *jointSim) Q {
	return w.invH.Mul(base.prismaticJoint.impulse.Y)
}

// SetTargetTranslation changes the prismatic spring target translation.
func (jointId JointId) SetTargetTranslation(translation Q) {
	w := getWorld(jointId.world0)
	joint := getJointSimCheckType(w, jointId, PrismaticJoint)
	joint.prismaticJoint.targetTranslation = translation
}

// GetTargetTranslation reports the prismatic spring target translation.
func (jointId JointId) GetTargetTranslation() Q {
	w := getWorld(jointId.world0)
	joint := getJointSimCheckType(w, jointId, PrismaticJoint)
	return joint.prismaticJoint.targetTranslation
}

// GetTranslation reports the current translation along the prismatic axis.
func (jointId JointId) GetTranslation() Q {
	w := getWorld(jointId.world0)
	joint := getJointSimCheckType(w, jointId, PrismaticJoint)
	transformA := getBodyTransform(w, joint.bodyIdA)
	transformB := getBodyTransform(w, joint.bodyIdB)
	axisA := RotateVector(transformA.Q, joint.prismaticJoint.localAxisA)
	pA := TransformPoint(transformA, joint.localOriginAnchorA)
	pB := TransformPoint(transformB, joint.localOriginAnchorB)
	return axisA.Dot(pB.Sub(pA))
}

// GetSpeed reports the current prismatic translation speed.
func (jointId JointId) GetSpeed() Q {
	w := getWorld(jointId.world0)
	joint := getJointSimCheckType(w, jointId, PrismaticJoint)
	bodyA := &w.bodies[joint.bodyIdA]
	bodyB := &w.bodies[joint.bodyIdB]
	bodySimA := getBodySim(w, bodyA)
	bodySimB := getBodySim(w, bodyB)
	bodyStateA := getBodyState(w, bodyA)
	bodyStateB := getBodyState(w, bodyB)

	transformA := bodySimA.transform
	transformB := bodySimB.transform
	prismatic := &joint.prismaticJoint
	axisA := RotateVector(transformA.Q, prismatic.localAxisA)
	rA := RotateVector(transformA.Q, joint.localOriginAnchorA.Sub(bodySimA.localCenter))
	rB := RotateVector(transformB.Q, joint.localOriginAnchorB.Sub(bodySimB.localCenter))
	d := bodySimB.center.Sub(bodySimA.center).Add(rB.Sub(rA))

	zero := fixed.Q32Zero()
	vA := Vec2Zero()
	vB := Vec2Zero()
	wA := zero
	wB := zero
	if bodyStateA != nil {
		vA = bodyStateA.linearVelocity
		wA = bodyStateA.angularVelocity
	}
	if bodyStateB != nil {
		vB = bodyStateB.linearVelocity
		wB = bodyStateB.angularVelocity
	}

	// D-004: solver angular velocities are turns per second; CrossSV uses radians.
	vRel := vB.Add(CrossSV(wB.Mul(tau), rB)).Sub(vA.Add(CrossSV(wA.Mul(tau), rA)))
	return d.Dot(CrossSV(wA.Mul(tau), axisA)).Add(axisA.Dot(vRel))
}

// Linear constraint (point-to-line)
// d = p2 - p1 = x2 + r2 - x1 - r1
// C = dot(perp, d)
// Cdot = dot(d, cross(w1, perp)) + dot(perp, v2 + cross(w2, r2) - v1 - cross(w1, r1))
//      = -dot(perp, v1) - dot(cross(d + r1, perp), w1) + dot(perp, v2) + dot(cross(r2, perp), v2)
// J = [-perp, -cross(d + r1, perp), perp, cross(r2,perp)]
//
// Angular constraint
// C = a2 - a1 + a_initial
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
//
// K = J * invM * JT
//
// J = [-a -s1 a s2]
//     [0  -1  0  1]
// a = perp
// s1 = cross(d + r1, a) = cross(p2 - x1, a)
// s2 = cross(r2, a) = cross(p2 - x2, a)

// Motor/Limit linear constraint
// C = dot(ax1, d)
// Cdot = -dot(ax1, v1) - dot(cross(d + r1, ax1), w1) + dot(ax1, v2) + dot(cross(r2, ax1), v2)
// J = [-ax1 -cross(d+r1,ax1) ax1 cross(r2,ax1)]

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

	joint := &base.prismaticJoint
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
	joint.axisA = RotateVector(qA, joint.localAxisA)
	joint.deltaCenter = bodySimB.center.Sub(bodySimA.center)
	joint.deltaAngle = RelativeAngle(qB, qA).Sub(joint.referenceAngle)
	joint.deltaAngle = UnwindAngle(joint.deltaAngle)

	rA := joint.anchorA
	rB := joint.anchorB

	d := joint.deltaCenter.Add(rB.Sub(rA))
	a1 := Cross(d.Add(rA), joint.axisA)
	a2 := Cross(rB, joint.axisA)

	// effective masses
	k := mA.Add(mB).Add(iA.Mul(a1).Mul(a1)).Add(iB.Mul(a2).Mul(a2))
	// D-006: the reference multiplies by the reciprocal of k.
	zero := fixed.Q32Zero()
	joint.axialMass = zero
	if zero.Less(k) {
		joint.axialMass = fixed.Q32One().Div(k)
	}

	joint.springSoftness = makeSoft(joint.hertz, joint.dampingRatio, context.h)

	if !context.enableWarmStarting {
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

	joint := &base.prismaticJoint

	stateA, stateB := jointStates(context.states, &dummyState, joint.indexA, joint.indexB)

	rA := RotateVector(stateA.deltaRotation, joint.anchorA)
	rB := RotateVector(stateB.deltaRotation, joint.anchorB)

	d := stateB.deltaPosition.Sub(stateA.deltaPosition).Add(joint.deltaCenter).Add(rB.Sub(rA))
	axisA := RotateVector(stateA.deltaRotation, joint.axisA)

	// impulse is applied at anchor point on body B
	a1 := Cross(d.Add(rA), axisA)
	a2 := Cross(rB, axisA)
	axialImpulse := joint.springImpulse.Add(joint.motorImpulse).Add(joint.lowerImpulse).Sub(joint.upperImpulse)

	// perpendicular constraint
	perpA := LeftPerp(axisA)
	s1 := Cross(d.Add(rA), perpA)
	s2 := Cross(rB, perpA)
	perpImpulse := joint.impulse.X
	angleImpulse := joint.impulse.Y

	P := axisA.Mul(axialImpulse).Add(perpA.Mul(perpImpulse))
	LA := axialImpulse.Mul(a1).Add(perpImpulse.Mul(s1)).Add(angleImpulse)
	LB := axialImpulse.Mul(a2).Add(perpImpulse.Mul(s2)).Add(angleImpulse)

	// D-004: the angular velocity of the state is turns per second.
	stateA.linearVelocity = MulSub(stateA.linearVelocity, mA, P)
	stateA.angularVelocity = stateA.angularVelocity.Mul(tau).Sub(iA.Mul(LA)).Div(tau)
	stateB.linearVelocity = MulAdd(stateB.linearVelocity, mB, P)
	stateB.angularVelocity = stateB.angularVelocity.Mul(tau).Add(iB.Mul(LB)).Div(tau)
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

	joint := &base.prismaticJoint

	stateA, stateB := jointStates(context.states, &dummyState, joint.indexA, joint.indexB)

	vA := stateA.linearVelocity
	wA := stateA.angularVelocity.Mul(tau)
	vB := stateB.linearVelocity
	wB := stateB.angularVelocity.Mul(tau)

	// current anchors
	rA := RotateVector(stateA.deltaRotation, joint.anchorA)
	rB := RotateVector(stateB.deltaRotation, joint.anchorB)

	d := stateB.deltaPosition.Sub(stateA.deltaPosition).Add(joint.deltaCenter).Add(rB.Sub(rA))
	axisA := RotateVector(stateA.deltaRotation, joint.axisA)
	translation := axisA.Dot(d)

	// These scalars are for torques generated by axial forces
	a1 := Cross(d.Add(rA), axisA)
	a2 := Cross(rB, axisA)

	zero := fixed.Q32Zero()
	one := fixed.Q32One()

	// spring constraint
	if joint.enableSpring {
		// This is a real spring and should be applied even during relax
		C := translation.Sub(joint.targetTranslation)
		bias := joint.springSoftness.biasRate.Mul(C)
		massScale := joint.springSoftness.massScale
		impulseScale := joint.springSoftness.impulseScale

		Cdot := axisA.Dot(vB.Sub(vA)).Add(a2.Mul(wB)).Sub(a1.Mul(wA))
		deltaImpulse := massScale.Neg().Mul(joint.axialMass).Mul(Cdot.Add(bias)).Sub(impulseScale.Mul(joint.springImpulse))
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
		impulse := joint.axialMass.Mul(joint.motorSpeed.Sub(Cdot))
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

			oldImpulse := joint.lowerImpulse
			Cdot := axisA.Dot(vB.Sub(vA)).Add(a2.Mul(wB)).Sub(a1.Mul(wA))
			impulse := joint.axialMass.Neg().Mul(massScale).Mul(Cdot.Add(bias)).Sub(impulseScale.Mul(oldImpulse))
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

			oldImpulse := joint.upperImpulse
			// sign flipped
			Cdot := axisA.Dot(vA.Sub(vB)).Add(a1.Mul(wA)).Sub(a2.Mul(wB))
			impulse := joint.axialMass.Neg().Mul(massScale).Mul(Cdot.Add(bias)).Sub(impulseScale.Mul(oldImpulse))
			joint.upperImpulse = oldImpulse.Add(impulse).Max(zero)
			impulse = joint.upperImpulse.Sub(oldImpulse)

			P := axisA.Mul(impulse)
			LA := impulse.Mul(a1)
			LB := impulse.Mul(a2)

			// sign flipped
			vA = MulAdd(vA, mA, P)
			wA = wA.Add(iA.Mul(LA))
			vB = MulSub(vB, mB, P)
			wB = wB.Sub(iB.Mul(LB))
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
			// D-004: the angle enters the error in radians.
			C.Y = RelativeAngle(stateB.deltaRotation, stateA.deltaRotation).Add(joint.deltaAngle).Mul(tau)

			bias = C.Mul(base.constraintSoftness.biasRate)
			massScale = base.constraintSoftness.massScale
			impulseScale = base.constraintSoftness.impulseScale
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
		var impulse Vec2
		impulse.X = massScale.Neg().Mul(b.X).Sub(impulseScale.Mul(joint.impulse.X))
		impulse.Y = massScale.Neg().Mul(b.Y).Sub(impulseScale.Mul(joint.impulse.Y))

		joint.impulse.X = joint.impulse.X.Add(impulse.X)
		joint.impulse.Y = joint.impulse.Y.Add(impulse.Y)

		P := perpA.Mul(impulse.X)
		LA := impulse.X.Mul(s1).Add(impulse.Y)
		LB := impulse.X.Mul(s2).Add(impulse.Y)

		vA = MulSub(vA, mA, P)
		wA = wA.Sub(iA.Mul(LA))
		vB = MulAdd(vB, mB, P)
		wB = wB.Add(iB.Mul(LB))
	}

	stateA.linearVelocity = vA
	stateA.angularVelocity = wA.Div(tau)
	stateB.linearVelocity = vB
	stateB.angularVelocity = wB.Div(tau)
}
