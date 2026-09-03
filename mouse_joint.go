package dbox2d

import "github.com/dhannyell/fixed"

// SetTarget changes the mouse joint target (b2MouseJoint_SetTarget).
func (jointId JointId) SetTarget(target Vec2) {
	if !IsValidVec2(target) {
		panic("dbox2d: SetTarget needs a valid vector")
	}
	w := getWorld(jointId.world0)
	js := getJointSimCheckType(w, jointId, MouseJoint)
	js.mouseJoint.targetA = target
}

// GetTarget reports the mouse joint target (b2MouseJoint_GetTarget).
func (jointId JointId) GetTarget() Vec2 {
	w := getWorld(jointId.world0)
	js := getJointSimCheckType(w, jointId, MouseJoint)
	return js.mouseJoint.targetA
}

// This file corresponds to src/mouse_joint.c of the reference. The joint
// acts on body B alone; body A only carries the target frame.

// getMouseJointForce reports the constraint force of the last step. It
// corresponds to b2GetMouseJointForce in src/mouse_joint.c.
func getMouseJointForce(w *world, base *jointSim) Vec2 {
	force := base.mouseJoint.linearImpulse.Mul(w.invH)
	return force
}

// getMouseJointTorque reports the constraint torque of the last step. It
// corresponds to b2GetMouseJointTorque in src/mouse_joint.c.
func getMouseJointTorque(w *world, base *jointSim) Q {
	return w.invH.Mul(base.mouseJoint.angularImpulse)
}

// prepareMouseJoint corresponds to b2PrepareMouseJoint in
// src/mouse_joint.c.
func prepareMouseJoint(base *jointSim, context *stepContext) {
	if base.jointType != MouseJoint {
		panic("dbox2d: the joint is not a mouse joint")
	}

	// chase body id to the solver set where the body lives
	idB := base.bodyIdB

	w := context.world

	bodyB := &w.bodies[idB]

	if bodyB.setIndex != awakeSet {
		panic("dbox2d: the body of the mouse joint is not awake")
	}
	setB := &w.solverSets[bodyB.setIndex]

	localIndexB := bodyB.localIndex
	bodySimB := &setB.bodySims[localIndexB]

	base.invMassB = bodySimB.invMass
	base.invIB = bodySimB.invInertia

	joint := &base.mouseJoint
	joint.indexB = nullIndex
	if bodyB.setIndex == awakeSet {
		joint.indexB = localIndexB
	}
	joint.anchorB = RotateVector(bodySimB.transform.Q, base.localOriginAnchorB.Sub(bodySimB.localCenter))

	joint.linearSoftness = makeSoft(joint.hertz, joint.dampingRatio, context.h)

	angularHertz := fixed.Q32Half()
	angularDampingRatio := fixed.Q32MustParse("0.1")
	joint.angularSoftness = makeSoft(angularHertz, angularDampingRatio, context.h)

	rB := joint.anchorB
	mB := bodySimB.invMass
	iB := bodySimB.invInertia

	// K = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
	//   = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
	//     [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
	var K Mat22
	K.Cx.X = mB.Add(iB.Mul(rB.Y).Mul(rB.Y))
	K.Cx.Y = iB.Neg().Mul(rB.X).Mul(rB.Y)
	K.Cy.X = K.Cx.Y
	K.Cy.Y = mB.Add(iB.Mul(rB.X).Mul(rB.X))

	joint.linearMass = GetInverse22(K)
	joint.deltaCenter = bodySimB.center.Sub(joint.targetA)

	if !context.enableWarmStarting {
		joint.linearImpulse = Vec2Zero()
		joint.angularImpulse = fixed.Q32Zero()
	}
}

// warmStartMouseJoint corresponds to b2WarmStartMouseJoint in
// src/mouse_joint.c.
func warmStartMouseJoint(base *jointSim, context *stepContext) {
	if base.jointType != MouseJoint {
		panic("dbox2d: the joint is not a mouse joint")
	}

	mB := base.invMassB
	iB := base.invIB

	joint := &base.mouseJoint

	stateB := &context.states[joint.indexB]
	vB := stateB.linearVelocity
	// D-004: the angular velocity of the state is turns per second.
	wB := stateB.angularVelocity.Mul(tau)

	dqB := stateB.deltaRotation
	rB := RotateVector(dqB, joint.anchorB)

	vB = MulAdd(vB, mB, joint.linearImpulse)
	wB = wB.Add(iB.Mul(Cross(rB, joint.linearImpulse).Add(joint.angularImpulse)))

	stateB.linearVelocity = vB
	stateB.angularVelocity = wB.Div(tau)
}

// solveMouseJoint corresponds to b2SolveMouseJoint in src/mouse_joint.c.
// The reference has no useBias parameter here.
func solveMouseJoint(base *jointSim, context *stepContext) {
	mB := base.invMassB
	iB := base.invIB

	joint := &base.mouseJoint
	stateB := &context.states[joint.indexB]

	vB := stateB.linearVelocity
	wB := stateB.angularVelocity.Mul(tau)

	zero := fixed.Q32Zero()

	// Softness with no bias to reduce rotation speed
	{
		massScale := joint.angularSoftness.massScale
		impulseScale := joint.angularSoftness.impulseScale

		impulse := zero
		if zero.Less(iB) {
			impulse = wB.Neg().Div(iB)
		}
		impulse = massScale.Mul(impulse).Sub(impulseScale.Mul(joint.angularImpulse))
		joint.angularImpulse = joint.angularImpulse.Add(impulse)

		wB = wB.Add(iB.Mul(impulse))
	}

	maxImpulse := joint.maxForce.Mul(context.h)

	{
		dqB := stateB.deltaRotation
		rB := RotateVector(dqB, joint.anchorB)
		Cdot := vB.Add(CrossSV(wB, rB))

		separation := stateB.deltaPosition.Add(rB).Add(joint.deltaCenter)
		bias := separation.Mul(joint.linearSoftness.biasRate)

		massScale := joint.linearSoftness.massScale
		impulseScale := joint.linearSoftness.impulseScale

		b := MulMV(joint.linearMass, Cdot.Add(bias))

		var impulse Vec2
		impulse.X = massScale.Neg().Mul(b.X).Sub(impulseScale.Mul(joint.linearImpulse.X))
		impulse.Y = massScale.Neg().Mul(b.Y).Sub(impulseScale.Mul(joint.linearImpulse.Y))

		oldImpulse := joint.linearImpulse
		joint.linearImpulse.X = joint.linearImpulse.X.Add(impulse.X)
		joint.linearImpulse.Y = joint.linearImpulse.Y.Add(impulse.Y)

		mag := joint.linearImpulse.Len()
		if maxImpulse.Less(mag) {
			joint.linearImpulse = joint.linearImpulse.Normalize().Mul(maxImpulse)
		}

		impulse.X = joint.linearImpulse.X.Sub(oldImpulse.X)
		impulse.Y = joint.linearImpulse.Y.Sub(oldImpulse.Y)

		vB = MulAdd(vB, mB, impulse)
		wB = wB.Add(iB.Mul(Cross(rB, impulse)))
	}

	stateB.linearVelocity = vB
	stateB.angularVelocity = wB.Div(tau)
}
