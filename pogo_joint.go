package b2

// This file corresponds to src/pogo_joint.c of the reference. The pogo
// joint is a hybrid of a contact and a spring: it pushes body B along a
// fixed normal, with a spring-damper on the pogo axis of frame B and
// separate tension and compression limits.

// pogoAxis is b2_pogoAxis: the y-axis of frame B.
func pogoAxis() Vec2 { return Vec2{X: QZero(), Y: QOne()} }

// getPogoJointForce reports the constraint force of the last step. It
// corresponds to b2GetPogoJointForce in src/pogo_joint.c.
func getPogoJointForce(w *world, base *jointSim) Vec2 {
	joint := base.pogo()
	return joint.normal.Mul(w.invH.Mul(joint.impulse))
}

// preparePogoJoint corresponds to b2PreparePogoJoint in src/pogo_joint.c.
func preparePogoJoint(base *jointSim, context *stepContext) {
	if base.jointType != PogoJoint {
		panic("dbox2d: the joint is not a pogo joint")
	}
	bodySimA, bodySimB, indexA, indexB := prepareJointBodies(base, context)
	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	joint := base.pogo()
	joint.indexA = indexA
	joint.indexB = indexB

	joint.frameA = jointFrame(bodySimA, base.localFrameA)
	joint.frameB = jointFrame(bodySimB, base.localFrameB)

	// Compute the initial center delta. Incremental position updates are
	// relative to this.
	joint.deltaCenter = bodySimB.center.Sub(bodySimA.center)

	rA := joint.frameA.P
	rB := joint.frameB.P

	// compute effective mass
	crA := Cross(rA, joint.normal)
	crB := Cross(rB, joint.normal)
	k := mA.Add(mB).Add(iA.Mul(crA).Mul(crA)).Add(iB.Mul(crB).Mul(crB))
	// D-006: the reference multiplies by the reciprocal of k.
	zero := QZero()
	joint.linearMass = zero
	if zero.Less(k) {
		joint.linearMass = QOne().Div(k)
	}

	if !context.enableWarmStarting {
		joint.impulse = zero
	}
}

// warmStartPogoJoint corresponds to b2WarmStartPogoJoint in
// src/pogo_joint.c. A pogo without a spring holds no impulse.
func warmStartPogoJoint(base *jointSim, context *stepContext) {
	if base.jointType != PogoJoint {
		panic("dbox2d: the joint is not a pogo joint")
	}

	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	joint := base.pogo()
	if joint.hertz.Eq(QZero()) {
		joint.impulse = QZero()
		return
	}

	// dummy state for static bodies
	dummyState := identityBodyState()
	stateA, stateB := jointStates(context.states, &dummyState, joint.indexA, joint.indexB)

	rA := RotateVector(stateA.deltaRotation, joint.frameA.P)
	rB := RotateVector(stateB.deltaRotation, joint.frameB.P)

	linearImpulse := joint.normal.Mul(joint.impulse)

	if stateA.flags&bodyDynamicFlag != 0 {
		stateA.linearVelocity = MulSub(stateA.linearVelocity, mA, linearImpulse)
		stateA.angularVelocity = stateA.angularVelocity.Sub(iA.Mul(Cross(rA, linearImpulse)))
	}
	if stateB.flags&bodyDynamicFlag != 0 {
		stateB.linearVelocity = MulAdd(stateB.linearVelocity, mB, linearImpulse)
		stateB.angularVelocity = stateB.angularVelocity.Add(iB.Mul(Cross(rB, linearImpulse)))
	}
}

// solvePogoJoint corresponds to b2SolvePogoJoint in src/pogo_joint.c.
func solvePogoJoint(base *jointSim, context *stepContext, useBias bool) {
	if base.jointType != PogoJoint {
		panic("dbox2d: the joint is not a pogo joint")
	}

	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState()

	joint := base.pogo()
	zero := QZero()
	if joint.hertz.Eq(zero) {
		joint.impulse = zero
		return
	}

	stateA, stateB := jointStates(context.states, &dummyState, joint.indexA, joint.indexB)

	vA := stateA.linearVelocity
	wA := stateA.angularVelocity
	vB := stateB.linearVelocity
	wB := stateB.angularVelocity

	rA := RotateVector(stateA.deltaRotation, joint.frameA.P)
	rB := RotateVector(stateB.deltaRotation, joint.frameB.P)

	bias := zero
	if useBias {
		// This is a custom pogo position correction designed to avoid feeding velocity into
		// the mover body. Otherwise the mover can make huge jumps going up stairs. This still
		// works with continuous collision.
		dcA := stateA.deltaPosition
		dcB := stateB.deltaPosition
		d := dcB.Sub(dcA).Add(rB.Sub(rA)).Add(joint.deltaCenter)

		// The pogo axis is usually fixed as the world y-axis. Different than the ground normal.
		axis := RotateVector(joint.frameB.Q, pogoAxis())
		c := axis.Dot(d).Sub(joint.restLength)

		// This could be divided by dot(joint->normal, pogoAxis) to account for the constraint
		// direction, but I'd rather diminish the pogo recovery rate than risk making it huge.
		joint.velocity = SpringDamper(joint.hertz, joint.dampingRatio, c, joint.velocity, context.h)
		bias = joint.velocity.Neg()
	}

	vr := vB.Add(CrossSV(wB, rB)).Sub(vA.Add(CrossSV(wA, rA)))
	cdot := joint.normal.Dot(vr)

	maxTensionImpulse := context.h.Mul(joint.maxTensionForce)
	maxCompressionImpulse := context.h.Mul(joint.maxCompressionForce)

	oldImpulse := joint.impulse
	impulse := joint.linearMass.Neg().Mul(cdot.Add(bias))
	joint.impulse = joint.impulse.Add(impulse).Clamp(maxTensionImpulse.Neg(), maxCompressionImpulse)
	impulse = joint.impulse.Sub(oldImpulse)

	P := joint.normal.Mul(impulse)
	vA = MulSub(vA, mA, P)
	wA = wA.Sub(iA.Mul(Cross(rA, P)))
	vB = MulAdd(vB, mB, P)
	wB = wB.Add(iB.Mul(Cross(rB, P)))

	storeJointVelocity(stateA, vA, wA)
	storeJointVelocity(stateB, vB, wB)
}
