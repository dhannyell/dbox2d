package b2

// This file corresponds to src/mover_joint.c of the reference. The mover
// joint drives the linear velocity of body B relative to body A, with a
// separate force limit per axis. It ignores rotation. It is unrelated to
// the character mover of mover.go.

// getMoverJointForce reports the constraint force of the last step. It
// corresponds to b2GetMoverJointForce in src/mover_joint.c.
func getMoverJointForce(w *world, base *jointSim) Vec2 {
	return base.mover().linearVelocityImpulse.Mul(w.invH)
}

// prepareMoverJoint corresponds to b2PrepareMoverJoint in
// src/mover_joint.c. The joint applies no torque, so it zeroes the
// inverse inertias.
func prepareMoverJoint(base *jointSim, context *stepContext) {
	if base.jointType != MoverJoint {
		panic("dbox2d: the joint is not a mover joint")
	}
	_, _, indexA, indexB := prepareJointBodies(base, context)
	zero := QZero()
	base.invIA = zero
	base.invIB = zero

	joint := base.mover()
	joint.indexA = indexA
	joint.indexB = indexB

	k := base.invMassA.Add(base.invMassB)
	// D-006: the reference multiplies by the reciprocal of k.
	joint.linearMass = zero
	if zero.Less(k) {
		joint.linearMass = QOne().Div(k)
	}

	if !context.enableWarmStarting {
		joint.linearVelocityImpulse = Vec2Zero()
	}
}

// warmStartMoverJoint corresponds to b2WarmStartMoverJoint in
// src/mover_joint.c.
func warmStartMoverJoint(base *jointSim, context *stepContext) {
	if base.jointType != MoverJoint {
		panic("dbox2d: the joint is not a mover joint")
	}

	mA := base.invMassA
	mB := base.invMassB

	joint := base.mover()

	// dummy state for static bodies
	dummyState := identityBodyState()
	stateA, stateB := jointStates(context.states, &dummyState, joint.indexA, joint.indexB)

	if stateA.flags&bodyDynamicFlag != 0 {
		stateA.linearVelocity = MulSub(stateA.linearVelocity, mA, joint.linearVelocityImpulse)
	}
	if stateB.flags&bodyDynamicFlag != 0 {
		stateB.linearVelocity = MulAdd(stateB.linearVelocity, mB, joint.linearVelocityImpulse)
	}
}

// solveMoverJoint corresponds to b2SolveMoverJoint in src/mover_joint.c.
func solveMoverJoint(base *jointSim, context *stepContext) {
	if base.jointType != MoverJoint {
		panic("dbox2d: the joint is not a mover joint")
	}

	mA := base.invMassA
	mB := base.invMassB

	// dummy state for static bodies
	dummyState := identityBodyState()

	joint := base.mover()
	stateA, stateB := jointStates(context.states, &dummyState, joint.indexA, joint.indexB)

	vA := stateA.linearVelocity
	vB := stateB.linearVelocity

	zero := QZero()

	// linear velocity
	if zero.Less(joint.maxVelocityForce.X) || zero.Less(joint.maxVelocityForce.Y) {
		cdot := vB.Sub(vA)
		cdot = cdot.Sub(joint.linearVelocity)
		b := cdot.Mul(joint.linearMass)
		impulse := Vec2{X: b.X.Neg(), Y: b.Y.Neg()}

		oldImpulse := joint.linearVelocityImpulse
		joint.linearVelocityImpulse = joint.linearVelocityImpulse.Add(impulse)

		maxImpulse := joint.maxVelocityForce.Mul(context.h)

		// clamp x and y separately
		joint.linearVelocityImpulse = Clamp(joint.linearVelocityImpulse, Neg(maxImpulse), maxImpulse)
		impulse = joint.linearVelocityImpulse.Sub(oldImpulse)

		vA = MulSub(vA, mA, impulse)
		vB = MulAdd(vB, mB, impulse)
	} else {
		joint.linearVelocityImpulse = Vec2Zero()
	}

	if stateA.flags&bodyDynamicFlag != 0 {
		stateA.linearVelocity = vA
	}
	if stateB.flags&bodyDynamicFlag != 0 {
		stateB.linearVelocity = vB
	}
}
