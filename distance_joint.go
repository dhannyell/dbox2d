package dbox2d

// This file corresponds to src/distance_joint.c of the reference.

func drawDistanceJoint(draw *DebugDraw, base *jointSim, transformA, transformB Transform) {
	joint := base.distance()
	pA := TransformPoint(transformA, base.localOriginAnchorA)
	pB := TransformPoint(transformB, base.localOriginAnchorB)
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

	pA := TransformPoint(transformA, base.localOriginAnchorA)
	pB := TransformPoint(transformB, base.localOriginAnchorB)
	d := pB.Sub(pA)
	axis := d.Normalize()
	force := joint.impulse.Add(joint.lowerImpulse).Sub(joint.upperImpulse).Add(joint.motorImpulse).Mul(w.invH)
	return axis.Mul(force)
}

// SetLength changes the distance joint length.
func (jointId JointId) SetLength(length Q) {
	w := getWorld(jointId.world0)
	joint := getJointSimCheckType(w, jointId, DistanceJoint)
	length = length.Clamp(linearSlop, Huge)
	joint.distance().length = length
	joint.distance().impulse = QZero()
	joint.distance().lowerImpulse = QZero()
	joint.distance().upperImpulse = QZero()
}

// GetLength reports the distance joint length.
func (jointId JointId) GetLength() Q {
	w := getWorld(jointId.world0)
	joint := getJointSimCheckType(w, jointId, DistanceJoint)
	return joint.distance().length
}

// EnableSpring enables or disables the spring on a distance, revolute,
// prismatic, or wheel joint.
func (jointId JointId) EnableSpring(enableSpring bool) {
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	joint := getJointSim(w, j)
	switch j.jointType {
	case DistanceJoint:
		joint.distance().enableSpring = enableSpring
	case RevoluteJoint:
		if enableSpring != joint.revolute().enableSpring {
			joint.revolute().enableSpring = enableSpring
			joint.revolute().springImpulse = QZero()
		}
	case PrismaticJoint:
		if enableSpring != joint.prismatic().enableSpring {
			joint.prismatic().enableSpring = enableSpring
			joint.prismatic().springImpulse = QZero()
		}
	case WheelJoint:
		if enableSpring != joint.wheel().enableSpring {
			joint.wheel().enableSpring = enableSpring
			joint.wheel().springImpulse = QZero()
		}
	default:
		panic("dbox2d: joint type does not support spring")
	}
}

// IsSpringEnabled reports whether the spring is enabled.
func (jointId JointId) IsSpringEnabled() bool {
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	joint := getJointSim(w, j)
	switch j.jointType {
	case DistanceJoint:
		return joint.distance().enableSpring
	case RevoluteJoint:
		return joint.revolute().enableSpring
	case PrismaticJoint:
		return joint.prismatic().enableSpring
	case WheelJoint:
		return joint.wheel().enableSpring
	default:
		panic("dbox2d: joint type does not support spring")
	}
}

// SetSpringHertz changes the spring frequency.
func (jointId JointId) SetSpringHertz(hertz Q) {
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	joint := getJointSim(w, j)
	switch j.jointType {
	case DistanceJoint:
		joint.distance().hertz = hertz
	case RevoluteJoint:
		joint.revolute().hertz = hertz
	case PrismaticJoint:
		joint.prismatic().hertz = hertz
	case WheelJoint:
		joint.wheel().hertz = hertz
	case MouseJoint:
		joint.mouse().hertz = hertz
	default:
		panic("dbox2d: joint type does not support spring hertz")
	}
}

// GetSpringHertz reports the spring frequency.
func (jointId JointId) GetSpringHertz() Q {
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	joint := getJointSim(w, j)
	switch j.jointType {
	case DistanceJoint:
		return joint.distance().hertz
	case RevoluteJoint:
		return joint.revolute().hertz
	case PrismaticJoint:
		return joint.prismatic().hertz
	case WheelJoint:
		return joint.wheel().hertz
	case MouseJoint:
		return joint.mouse().hertz
	default:
		panic("dbox2d: joint type does not support spring hertz")
	}
}

// SetSpringDampingRatio changes the spring damping ratio.
func (jointId JointId) SetSpringDampingRatio(dampingRatio Q) {
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	joint := getJointSim(w, j)
	switch j.jointType {
	case DistanceJoint:
		joint.distance().dampingRatio = dampingRatio
	case RevoluteJoint:
		joint.revolute().dampingRatio = dampingRatio
	case PrismaticJoint:
		joint.prismatic().dampingRatio = dampingRatio
	case WheelJoint:
		joint.wheel().dampingRatio = dampingRatio
	case MouseJoint:
		joint.mouse().dampingRatio = dampingRatio
	default:
		panic("dbox2d: joint type does not support spring damping ratio")
	}
}

// GetSpringDampingRatio reports the spring damping ratio.
func (jointId JointId) GetSpringDampingRatio() Q {
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	joint := getJointSim(w, j)
	switch j.jointType {
	case DistanceJoint:
		return joint.distance().dampingRatio
	case RevoluteJoint:
		return joint.revolute().dampingRatio
	case PrismaticJoint:
		return joint.prismatic().dampingRatio
	case WheelJoint:
		return joint.wheel().dampingRatio
	case MouseJoint:
		return joint.mouse().dampingRatio
	default:
		panic("dbox2d: joint type does not support spring damping ratio")
	}
}

// EnableLimit enables or disables the joint limit.
func (jointId JointId) EnableLimit(enableLimit bool) {
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	joint := getJointSim(w, j)
	switch j.jointType {
	case DistanceJoint:
		joint.distance().enableLimit = enableLimit
	case RevoluteJoint:
		if enableLimit != joint.revolute().enableLimit {
			joint.revolute().enableLimit = enableLimit
			joint.revolute().lowerImpulse = QZero()
			joint.revolute().upperImpulse = QZero()
		}
	case PrismaticJoint:
		if enableLimit != joint.prismatic().enableLimit {
			joint.prismatic().enableLimit = enableLimit
			joint.prismatic().lowerImpulse = QZero()
			joint.prismatic().upperImpulse = QZero()
		}
	case WheelJoint:
		if enableLimit != joint.wheel().enableLimit {
			joint.wheel().enableLimit = enableLimit
			joint.wheel().lowerImpulse = QZero()
			joint.wheel().upperImpulse = QZero()
		}
	default:
		panic("dbox2d: joint type does not support limits")
	}
}

// IsLimitEnabled reports whether the joint limit is enabled.
func (jointId JointId) IsLimitEnabled() bool {
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	joint := getJointSim(w, j)
	switch j.jointType {
	case DistanceJoint:
		return joint.distance().enableLimit
	case RevoluteJoint:
		return joint.revolute().enableLimit
	case PrismaticJoint:
		return joint.prismatic().enableLimit
	case WheelJoint:
		return joint.wheel().enableLimit
	default:
		panic("dbox2d: joint type does not support limits")
	}
}

// SetLengthRange changes the distance joint limits.
func (jointId JointId) SetLengthRange(minLength, maxLength Q) {
	w := getWorld(jointId.world0)
	joint := getJointSimCheckType(w, jointId, DistanceJoint)
	minLength = minLength.Clamp(linearSlop, Huge)
	maxLength = maxLength.Clamp(linearSlop, Huge)
	joint.distance().minLength = minLength.Min(maxLength)
	joint.distance().maxLength = minLength.Max(maxLength)
	joint.distance().impulse = QZero()
	joint.distance().lowerImpulse = QZero()
	joint.distance().upperImpulse = QZero()
}

// GetMinLength reports the lower distance limit.
func (jointId JointId) GetMinLength() Q {
	w := getWorld(jointId.world0)
	joint := getJointSimCheckType(w, jointId, DistanceJoint)
	return joint.distance().minLength
}

// GetMaxLength reports the upper distance limit.
func (jointId JointId) GetMaxLength() Q {
	w := getWorld(jointId.world0)
	joint := getJointSimCheckType(w, jointId, DistanceJoint)
	return joint.distance().maxLength
}

// GetCurrentLength reports the distance between the current world anchors.
func (jointId JointId) GetCurrentLength() Q {
	w := getWorld(jointId.world0)
	joint := getJointSimCheckType(w, jointId, DistanceJoint)
	transformA := getBodyTransform(w, joint.bodyIdA)
	transformB := getBodyTransform(w, joint.bodyIdB)
	pA := TransformPoint(transformA, joint.localOriginAnchorA)
	pB := TransformPoint(transformB, joint.localOriginAnchorB)
	return pB.Sub(pA).Len()
}

// EnableMotor enables or disables the joint motor.
func (jointId JointId) EnableMotor(enableMotor bool) {
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	joint := getJointSim(w, j)
	switch j.jointType {
	case DistanceJoint:
		if enableMotor != joint.distance().enableMotor {
			joint.distance().enableMotor = enableMotor
			joint.distance().motorImpulse = QZero()
		}
	case RevoluteJoint:
		if enableMotor != joint.revolute().enableMotor {
			joint.revolute().enableMotor = enableMotor
			joint.revolute().motorImpulse = QZero()
		}
	case PrismaticJoint:
		if enableMotor != joint.prismatic().enableMotor {
			joint.prismatic().enableMotor = enableMotor
			joint.prismatic().motorImpulse = QZero()
		}
	case WheelJoint:
		if enableMotor != joint.wheel().enableMotor {
			joint.wheel().enableMotor = enableMotor
			joint.wheel().motorImpulse = QZero()
		}
	default:
		panic("dbox2d: joint type does not support motors")
	}
}

// IsMotorEnabled reports whether the joint motor is enabled.
func (jointId JointId) IsMotorEnabled() bool {
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	joint := getJointSim(w, j)
	switch j.jointType {
	case DistanceJoint:
		return joint.distance().enableMotor
	case RevoluteJoint:
		return joint.revolute().enableMotor
	case PrismaticJoint:
		return joint.prismatic().enableMotor
	case WheelJoint:
		return joint.wheel().enableMotor
	default:
		panic("dbox2d: joint type does not support motors")
	}
}

// SetMotorSpeed changes the joint motor speed. Revolute and wheel values are
// turns per second (D-004); distance and prismatic values are metres/second.
func (jointId JointId) SetMotorSpeed(motorSpeed Q) {
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	joint := getJointSim(w, j)
	switch j.jointType {
	case DistanceJoint:
		joint.distance().motorSpeed = motorSpeed
	case RevoluteJoint:
		// D-004: revolute motor speed is stored in turns per second.
		joint.revolute().motorSpeed = angleFromTurns(motorSpeed)
	case PrismaticJoint:
		joint.prismatic().motorSpeed = motorSpeed
	case WheelJoint:
		// D-004: wheel motor speed is stored in turns per second.
		joint.wheel().motorSpeed = angleFromTurns(motorSpeed)
	default:
		panic("dbox2d: joint type does not support motor speed")
	}
}

// GetMotorSpeed reports the joint motor speed.
func (jointId JointId) GetMotorSpeed() Q {
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	joint := getJointSim(w, j)
	switch j.jointType {
	case DistanceJoint:
		return joint.distance().motorSpeed
	case RevoluteJoint:
		// D-004: revolute motor speed is stored in turns per second.
		return angleToTurns(joint.revolute().motorSpeed)
	case PrismaticJoint:
		return joint.prismatic().motorSpeed
	case WheelJoint:
		// D-004: wheel motor speed is stored in turns per second.
		return angleToTurns(joint.wheel().motorSpeed)
	default:
		panic("dbox2d: joint type does not support motor speed")
	}
}

// SetMaxMotorForce changes the maximum distance or prismatic motor force.
func (jointId JointId) SetMaxMotorForce(force Q) {
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	joint := getJointSim(w, j)
	switch j.jointType {
	case DistanceJoint:
		joint.distance().maxMotorForce = force
	case PrismaticJoint:
		joint.prismatic().maxMotorForce = force
	default:
		panic("dbox2d: joint type does not support motor force")
	}
}

// GetMaxMotorForce reports the maximum distance or prismatic motor force.
func (jointId JointId) GetMaxMotorForce() Q {
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	joint := getJointSim(w, j)
	switch j.jointType {
	case DistanceJoint:
		return joint.distance().maxMotorForce
	case PrismaticJoint:
		return joint.prismatic().maxMotorForce
	default:
		panic("dbox2d: joint type does not support motor force")
	}
}

// GetMotorForce reports the last distance or prismatic motor force.
func (jointId JointId) GetMotorForce() Q {
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	joint := getJointSim(w, j)
	switch j.jointType {
	case DistanceJoint:
		return w.invH.Mul(joint.distance().motorImpulse)
	case PrismaticJoint:
		return w.invH.Mul(joint.prismatic().motorImpulse)
	default:
		panic("dbox2d: joint type does not support motor force")
	}
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

	joint := base.distance()

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

	stateA.linearVelocity = MulSub(stateA.linearVelocity, mA, P)
	stateA.angularVelocity = stateA.angularVelocity.Sub(iA.Mul(Cross(rA, P)))
	stateB.linearVelocity = MulAdd(stateB.linearVelocity, mB, P)
	stateB.angularVelocity = stateB.angularVelocity.Add(iB.Mul(Cross(rB, P)))
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
			impulse := m.Neg().Mul(Cdot.Add(bias)).Sub(joint.distanceSoftness.impulseScale.Mul(joint.impulse))
			joint.impulse = joint.impulse.Add(impulse)

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
				massCoeff := one
				impulseCoeff := zero
				if zero.Less(C) {
					// speculative
					bias = C.Mul(context.invH)
				} else if useBias {
					bias = base.constraintSoftness.biasRate.Mul(C)
					massCoeff = base.constraintSoftness.massScale
					impulseCoeff = base.constraintSoftness.impulseScale
				}

				impulse := massCoeff.Neg().Mul(joint.axialMass).Mul(Cdot.Add(bias)).Sub(impulseCoeff.Mul(joint.lowerImpulse))
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

	stateA.linearVelocity = vA
	stateA.angularVelocity = wA
	stateB.linearVelocity = vB
	stateB.angularVelocity = wB
}
