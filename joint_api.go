package b2

// This file holds the typed joint accessors of include/box2d/box2d.h. The
// reference names them per joint type, as b2RevoluteJoint_EnableSpring and
// b2WheelJoint_EnableSpring; the port folds an accessor that several types
// share into one JointId method that dispatches on the joint type and
// panics on a type without it (D-003). Angles and angular speeds are in
// turns (D-004).

// unsupported panics for a joint type that lacks an accessor.
func unsupported(feature string) {
	panic("dbox2d: joint type does not support " + feature)
}

// checkValidQ panics on an invalid accessor input (D-003).
func checkValidQ(value Q, name string) {
	if !IsValidQ(value) {
		panic("dbox2d: " + name + " needs a valid value")
	}
}

// isForceBound reports whether a force bound is valid. The largest
// magnitude counts as unbounded: it is the default of the distance spring
// range and of the event thresholds, and in fixed mode IsValidQ reserves
// it as the saturation value.
func isForceBound(value Q) bool {
	return IsValidQ(value) || value.Eq(QMaxValue()) || value.Eq(QMaxValue().Neg())
}

// jointSimOf returns the cold and the warm data of a joint id.
func jointSimOf(jointId JointId) (*world, *joint, *jointSim) {
	w := getWorld(jointId.world0)
	j := getJointFullId(w, jointId)
	return w, j, getJointSim(w, j)
}

// currentAngularSpeed reports wB - wA in turns per second. A body outside
// the awake set has no state and counts as at rest.
func currentAngularSpeed(w *world, js *jointSim) Q {
	wA := QZero()
	if state := getBodyState(w, &w.bodies[js.bodyIdA]); state != nil {
		wA = state.angularVelocity
	}
	wB := QZero()
	if state := getBodyState(w, &w.bodies[js.bodyIdB]); state != nil {
		wB = state.angularVelocity
	}
	// D-004: the state stores radians per second and the API reports turns.
	return wB.Sub(wA).Div(tau)
}

// EnableSpring enables or disables the spring on a distance, prismatic,
// revolute or wheel joint.
func (jointId JointId) EnableSpring(enableSpring bool) {
	_, j, js := jointSimOf(jointId)
	switch j.jointType {
	case DistanceJoint:
		js.distance().enableSpring = enableSpring
	case PrismaticJoint:
		if enableSpring != js.prismatic().enableSpring {
			js.prismatic().enableSpring = enableSpring
			js.prismatic().springImpulse = QZero()
		}
	case RevoluteJoint:
		if enableSpring != js.revolute().enableSpring {
			js.revolute().enableSpring = enableSpring
			js.revolute().springImpulse = QZero()
		}
	case WheelJoint:
		if enableSpring != js.wheel().enableSpring {
			js.wheel().enableSpring = enableSpring
			js.wheel().springImpulse = QZero()
		}
	default:
		unsupported("a spring")
	}
}

// IsSpringEnabled reports whether the spring is enabled.
func (jointId JointId) IsSpringEnabled() bool {
	_, j, js := jointSimOf(jointId)
	switch j.jointType {
	case DistanceJoint:
		return js.distance().enableSpring
	case PrismaticJoint:
		return js.prismatic().enableSpring
	case RevoluteJoint:
		return js.revolute().enableSpring
	case WheelJoint:
		return js.wheel().enableSpring
	default:
		unsupported("a spring")
		return false
	}
}

// SetSpringHertz changes the spring stiffness in hertz on a distance,
// pogo, prismatic, revolute or wheel joint.
func (jointId JointId) SetSpringHertz(hertz Q) {
	checkValidQ(hertz, "SetSpringHertz")
	_, j, js := jointSimOf(jointId)
	switch j.jointType {
	case DistanceJoint:
		js.distance().hertz = hertz
	case PogoJoint:
		js.pogo().hertz = hertz
	case PrismaticJoint:
		js.prismatic().hertz = hertz
	case RevoluteJoint:
		js.revolute().hertz = hertz
	case WheelJoint:
		js.wheel().hertz = hertz
	default:
		unsupported("a spring")
	}
}

// GetSpringHertz reports the spring stiffness in hertz.
func (jointId JointId) GetSpringHertz() Q {
	_, j, js := jointSimOf(jointId)
	switch j.jointType {
	case DistanceJoint:
		return js.distance().hertz
	case PogoJoint:
		return js.pogo().hertz
	case PrismaticJoint:
		return js.prismatic().hertz
	case RevoluteJoint:
		return js.revolute().hertz
	case WheelJoint:
		return js.wheel().hertz
	default:
		unsupported("a spring")
		return QZero()
	}
}

// SetSpringDampingRatio changes the spring damping ratio on a distance,
// pogo, prismatic, revolute or wheel joint.
func (jointId JointId) SetSpringDampingRatio(dampingRatio Q) {
	checkValidQ(dampingRatio, "SetSpringDampingRatio")
	_, j, js := jointSimOf(jointId)
	switch j.jointType {
	case DistanceJoint:
		js.distance().dampingRatio = dampingRatio
	case PogoJoint:
		js.pogo().dampingRatio = dampingRatio
	case PrismaticJoint:
		js.prismatic().dampingRatio = dampingRatio
	case RevoluteJoint:
		js.revolute().dampingRatio = dampingRatio
	case WheelJoint:
		js.wheel().dampingRatio = dampingRatio
	default:
		unsupported("a spring")
	}
}

// GetSpringDampingRatio reports the spring damping ratio.
func (jointId JointId) GetSpringDampingRatio() Q {
	_, j, js := jointSimOf(jointId)
	switch j.jointType {
	case DistanceJoint:
		return js.distance().dampingRatio
	case PogoJoint:
		return js.pogo().dampingRatio
	case PrismaticJoint:
		return js.prismatic().dampingRatio
	case RevoluteJoint:
		return js.revolute().dampingRatio
	case WheelJoint:
		return js.wheel().dampingRatio
	default:
		unsupported("a spring")
		return QZero()
	}
}

// EnableLimit enables or disables the limit on a distance, prismatic,
// revolute or wheel joint.
func (jointId JointId) EnableLimit(enableLimit bool) {
	_, j, js := jointSimOf(jointId)
	switch j.jointType {
	case DistanceJoint:
		js.distance().enableLimit = enableLimit
	case PrismaticJoint:
		if enableLimit != js.prismatic().enableLimit {
			js.prismatic().enableLimit = enableLimit
			js.prismatic().lowerImpulse = QZero()
			js.prismatic().upperImpulse = QZero()
		}
	case RevoluteJoint:
		if enableLimit != js.revolute().enableLimit {
			js.revolute().enableLimit = enableLimit
			js.revolute().lowerImpulse = QZero()
			js.revolute().upperImpulse = QZero()
		}
	case WheelJoint:
		if enableLimit != js.wheel().enableLimit {
			js.wheel().lowerImpulse = QZero()
			js.wheel().upperImpulse = QZero()
			js.wheel().enableLimit = enableLimit
		}
	default:
		unsupported("limits")
	}
}

// IsLimitEnabled reports whether the limit is enabled.
func (jointId JointId) IsLimitEnabled() bool {
	_, j, js := jointSimOf(jointId)
	switch j.jointType {
	case DistanceJoint:
		return js.distance().enableLimit
	case PrismaticJoint:
		return js.prismatic().enableLimit
	case RevoluteJoint:
		return js.revolute().enableLimit
	case WheelJoint:
		return js.wheel().enableLimit
	default:
		unsupported("limits")
		return false
	}
}

// SetLimits changes the limits of a distance joint (the length range), a
// prismatic or wheel joint (the translation range) or a revolute joint (the
// angle range in turns, bounded to 0.495 turns).
func (jointId JointId) SetLimits(lower, upper Q) {
	checkValidQ(lower, "SetLimits")
	checkValidQ(upper, "SetLimits")
	_, j, js := jointSimOf(jointId)
	switch j.jointType {
	case DistanceJoint:
		lower = lower.Clamp(linearSlop, Huge)
		upper = upper.Clamp(linearSlop, Huge)
		js.distance().minLength = lower.Min(upper)
		js.distance().maxLength = lower.Max(upper)
	case PrismaticJoint:
		if upper.Less(lower) {
			panic("dbox2d: SetLimits lower exceeds upper")
		}
		js.prismatic().lowerTranslation = lower.Min(upper)
		js.prismatic().upperTranslation = lower.Max(upper)
	case RevoluteJoint:
		if upper.Less(lower) {
			panic("dbox2d: SetLimits lower exceeds upper")
		}
		// D-004: the reference bound of 0.99 pi is 0.495 turns.
		lowerAngle := lower.Min(upper).Clamp(revoluteLimitAngle.Neg(), revoluteLimitAngle)
		upperAngle := lower.Max(upper).Clamp(revoluteLimitAngle.Neg(), revoluteLimitAngle)
		js.revolute().lowerAngle = angleFromTurns(lowerAngle)
		js.revolute().upperAngle = angleFromTurns(upperAngle)
	case WheelJoint:
		if upper.Less(lower) {
			panic("dbox2d: SetLimits lower exceeds upper")
		}
		js.wheel().lowerTranslation = lower.Min(upper)
		js.wheel().upperTranslation = lower.Max(upper)
	default:
		unsupported("limits")
	}
}

// GetLowerLimit reports the lower limit: a length, a translation or an
// angle in turns.
func (jointId JointId) GetLowerLimit() Q {
	_, j, js := jointSimOf(jointId)
	switch j.jointType {
	case DistanceJoint:
		return js.distance().minLength
	case PrismaticJoint:
		return js.prismatic().lowerTranslation
	case RevoluteJoint:
		return angleToTurns(js.revolute().lowerAngle)
	case WheelJoint:
		return js.wheel().lowerTranslation
	default:
		unsupported("limits")
		return QZero()
	}
}

// GetUpperLimit reports the upper limit: a length, a translation or an
// angle in turns.
func (jointId JointId) GetUpperLimit() Q {
	_, j, js := jointSimOf(jointId)
	switch j.jointType {
	case DistanceJoint:
		return js.distance().maxLength
	case PrismaticJoint:
		return js.prismatic().upperTranslation
	case RevoluteJoint:
		return angleToTurns(js.revolute().upperAngle)
	case WheelJoint:
		return js.wheel().upperTranslation
	default:
		unsupported("limits")
		return QZero()
	}
}

// EnableMotor enables or disables the motor on a distance, prismatic,
// revolute or wheel joint.
func (jointId JointId) EnableMotor(enableMotor bool) {
	_, j, js := jointSimOf(jointId)
	switch j.jointType {
	case DistanceJoint:
		if enableMotor != js.distance().enableMotor {
			js.distance().enableMotor = enableMotor
			js.distance().motorImpulse = QZero()
		}
	case PrismaticJoint:
		if enableMotor != js.prismatic().enableMotor {
			js.prismatic().enableMotor = enableMotor
			js.prismatic().motorImpulse = QZero()
		}
	case RevoluteJoint:
		if enableMotor != js.revolute().enableMotor {
			js.revolute().enableMotor = enableMotor
			js.revolute().motorImpulse = QZero()
		}
	case WheelJoint:
		if enableMotor != js.wheel().enableMotor {
			js.wheel().motorImpulse = QZero()
			js.wheel().enableMotor = enableMotor
		}
	default:
		unsupported("a motor")
	}
}

// IsMotorEnabled reports whether the motor is enabled.
func (jointId JointId) IsMotorEnabled() bool {
	_, j, js := jointSimOf(jointId)
	switch j.jointType {
	case DistanceJoint:
		return js.distance().enableMotor
	case PrismaticJoint:
		return js.prismatic().enableMotor
	case RevoluteJoint:
		return js.revolute().enableMotor
	case WheelJoint:
		return js.wheel().enableMotor
	default:
		unsupported("a motor")
		return false
	}
}

// SetTargetMotorSpeed changes the motor speed: meters per second on a
// distance or prismatic joint, turns per second on a revolute or wheel
// joint.
func (jointId JointId) SetTargetMotorSpeed(motorSpeed Q) {
	checkValidQ(motorSpeed, "SetTargetMotorSpeed")
	_, j, js := jointSimOf(jointId)
	switch j.jointType {
	case DistanceJoint:
		js.distance().motorSpeed = motorSpeed
	case PrismaticJoint:
		js.prismatic().motorSpeed = motorSpeed
	case RevoluteJoint:
		js.revolute().motorSpeed = angleFromTurns(motorSpeed)
	case WheelJoint:
		js.wheel().motorSpeed = angleFromTurns(motorSpeed)
	default:
		unsupported("a motor")
	}
}

// GetTargetMotorSpeed reports the motor speed.
func (jointId JointId) GetTargetMotorSpeed() Q {
	_, j, js := jointSimOf(jointId)
	switch j.jointType {
	case DistanceJoint:
		return js.distance().motorSpeed
	case PrismaticJoint:
		return js.prismatic().motorSpeed
	case RevoluteJoint:
		return angleToTurns(js.revolute().motorSpeed)
	case WheelJoint:
		return angleToTurns(js.wheel().motorSpeed)
	default:
		unsupported("a motor")
		return QZero()
	}
}

// SetMaxMotorForce changes the maximum motor force of a distance or
// prismatic joint.
func (jointId JointId) SetMaxMotorForce(force Q) {
	checkValidQ(force, "SetMaxMotorForce")
	_, j, js := jointSimOf(jointId)
	switch j.jointType {
	case DistanceJoint:
		js.distance().maxMotorForce = force
	case PrismaticJoint:
		js.prismatic().maxMotorForce = force
	default:
		unsupported("a motor force")
	}
}

// GetMaxMotorForce reports the maximum motor force.
func (jointId JointId) GetMaxMotorForce() Q {
	_, j, js := jointSimOf(jointId)
	switch j.jointType {
	case DistanceJoint:
		return js.distance().maxMotorForce
	case PrismaticJoint:
		return js.prismatic().maxMotorForce
	default:
		unsupported("a motor force")
		return QZero()
	}
}

// GetCurrentMotorForce reports the motor force of the last step on a
// distance or prismatic joint.
func (jointId JointId) GetCurrentMotorForce() Q {
	w, j, js := jointSimOf(jointId)
	switch j.jointType {
	case DistanceJoint:
		return w.invH.Mul(js.distance().motorImpulse)
	case PrismaticJoint:
		return w.invH.Mul(js.prismatic().motorImpulse)
	default:
		unsupported("a motor force")
		return QZero()
	}
}

// SetMaxMotorTorque changes the maximum motor torque of a revolute or wheel
// joint.
func (jointId JointId) SetMaxMotorTorque(torque Q) {
	checkValidQ(torque, "SetMaxMotorTorque")
	_, j, js := jointSimOf(jointId)
	switch j.jointType {
	case RevoluteJoint:
		js.revolute().maxMotorTorque = torque
	case WheelJoint:
		js.wheel().maxMotorTorque = torque
	default:
		unsupported("a motor torque")
	}
}

// GetMaxMotorTorque reports the maximum motor torque.
func (jointId JointId) GetMaxMotorTorque() Q {
	_, j, js := jointSimOf(jointId)
	switch j.jointType {
	case RevoluteJoint:
		return js.revolute().maxMotorTorque
	case WheelJoint:
		return js.wheel().maxMotorTorque
	default:
		unsupported("a motor torque")
		return QZero()
	}
}

// GetCurrentMotorTorque reports the motor torque of the last step on a
// revolute or wheel joint.
func (jointId JointId) GetCurrentMotorTorque() Q {
	w, j, js := jointSimOf(jointId)
	switch j.jointType {
	case RevoluteJoint:
		return w.invH.Mul(js.revolute().motorImpulse)
	case WheelJoint:
		return w.invH.Mul(js.wheel().motorImpulse)
	default:
		unsupported("a motor torque")
		return QZero()
	}
}

// SetRestLength changes the rest length of a distance or pogo joint. The
// distance joint clamps it to [linearSlop, Huge] and restarts its impulses.
func (jointId JointId) SetRestLength(length Q) {
	checkValidQ(length, "SetRestLength")
	_, j, js := jointSimOf(jointId)
	switch j.jointType {
	case DistanceJoint:
		d := js.distance()
		d.length = length.Clamp(linearSlop, Huge)
		d.impulse = QZero()
		d.lowerImpulse = QZero()
		d.upperImpulse = QZero()
	case PogoJoint:
		js.pogo().restLength = length
	default:
		unsupported("a rest length")
	}
}

// GetRestLength reports the rest length of a distance or pogo joint.
func (jointId JointId) GetRestLength() Q {
	_, j, js := jointSimOf(jointId)
	switch j.jointType {
	case DistanceJoint:
		return js.distance().length
	case PogoJoint:
		return js.pogo().restLength
	default:
		unsupported("a rest length")
		return QZero()
	}
}

// GetCurrentLength reports the current length of a distance joint, or the
// length along the pogo axis of a pogo joint.
func (jointId JointId) GetCurrentLength() Q {
	w, j, js := jointSimOf(jointId)
	if w.locked {
		panic("dbox2d: the world is locked")
	}
	transformA := getBodyTransform(w, js.bodyIdA)
	transformB := getBodyTransform(w, js.bodyIdB)
	pA := TransformPoint(transformA, js.localFrameA.P)
	pB := TransformPoint(transformB, js.localFrameB.P)
	d := pB.Sub(pA)
	switch j.jointType {
	case DistanceJoint:
		return d.Len()
	case PogoJoint:
		axis := RotateVector(js.localFrameB.Q, pogoAxis())
		axis = RotateVector(transformB.Q, axis)
		return d.Dot(axis)
	default:
		unsupported("a current length")
		return QZero()
	}
}

// GetCurrentSpeed reports the joint speed: the angular speed in turns per
// second of a revolute joint, the translation speed of a prismatic joint
// or the spring velocity of a pogo joint.
func (jointId JointId) GetCurrentSpeed() Q {
	w, j, js := jointSimOf(jointId)
	switch j.jointType {
	case PogoJoint:
		return js.pogo().velocity
	case PrismaticJoint:
		return getPrismaticJointSpeed(w, js)
	case RevoluteJoint:
		return currentAngularSpeed(w, js)
	default:
		unsupported("a current speed")
		return QZero()
	}
}

// SetSpringForceRange changes the force range of a distance joint spring.
func (jointId JointId) SetSpringForceRange(lowerForce, upperForce Q) {
	if !isForceBound(lowerForce) || !isForceBound(upperForce) {
		panic("dbox2d: SetSpringForceRange needs a valid value")
	}
	if upperForce.Less(lowerForce) {
		panic("dbox2d: SetSpringForceRange lower exceeds upper")
	}
	w := getWorld(jointId.world0)
	js := getJointSimCheckType(w, jointId, DistanceJoint)
	js.distance().lowerSpringForce = lowerForce.Min(upperForce)
	js.distance().upperSpringForce = lowerForce.Max(upperForce)
}

// GetSpringForceRange reports the force range of a distance joint spring.
func (jointId JointId) GetSpringForceRange() (lowerForce, upperForce Q) {
	w := getWorld(jointId.world0)
	js := getJointSimCheckType(w, jointId, DistanceJoint)
	return js.distance().lowerSpringForce, js.distance().upperSpringForce
}

// GetSpringForce reports the spring force of a distance joint in the last
// step, or zero without a spring.
func (jointId JointId) GetSpringForce() Q {
	w := getWorld(jointId.world0)
	js := getJointSimCheckType(w, jointId, DistanceJoint)
	if js.distance().enableSpring {
		return js.distance().impulse.Mul(w.invH)
	}
	return QZero()
}

// SetTargetTranslation changes the spring target of a prismatic joint.
func (jointId JointId) SetTargetTranslation(translation Q) {
	checkValidQ(translation, "SetTargetTranslation")
	w := getWorld(jointId.world0)
	getJointSimCheckType(w, jointId, PrismaticJoint).prismatic().targetTranslation = translation
}

// GetTargetTranslation reports the spring target of a prismatic joint.
func (jointId JointId) GetTargetTranslation() Q {
	w := getWorld(jointId.world0)
	return getJointSimCheckType(w, jointId, PrismaticJoint).prismatic().targetTranslation
}

// GetCurrentTranslation reports the translation of a prismatic joint along
// the x-axis of its frame A.
func (jointId JointId) GetCurrentTranslation() Q {
	w := getWorld(jointId.world0)
	js := getJointSimCheckType(w, jointId, PrismaticJoint)
	transformA := getBodyTransform(w, js.bodyIdA)
	transformB := getBodyTransform(w, js.bodyIdB)
	localAxisA := RotGetXAxis(js.localFrameA.Q)
	axisA := RotateVector(transformA.Q, localAxisA)
	pA := TransformPoint(transformA, js.localFrameA.P)
	pB := TransformPoint(transformB, js.localFrameB.P)
	return pB.Sub(pA).Dot(axisA)
}

// SetTargetAngle changes the spring target of a revolute joint in turns.
// The create clamps it to half a turn; this setter does not, as in the
// reference.
func (jointId JointId) SetTargetAngle(angle Q) {
	checkValidQ(angle, "SetTargetAngle")
	w := getWorld(jointId.world0)
	getJointSimCheckType(w, jointId, RevoluteJoint).revolute().targetAngle = angleFromTurns(angle)
}

// GetTargetAngle reports the spring target of a revolute joint in turns.
func (jointId JointId) GetTargetAngle() Q {
	w := getWorld(jointId.world0)
	return angleToTurns(getJointSimCheckType(w, jointId, RevoluteJoint).revolute().targetAngle)
}

// GetCurrentAngle reports the angle of a revolute joint in turns: frame B
// relative to frame A.
func (jointId JointId) GetCurrentAngle() Q {
	w := getWorld(jointId.world0)
	js := getJointSimCheckType(w, jointId, RevoluteJoint)
	qA := MulRot(getBodyTransform(w, js.bodyIdA).Q, js.localFrameA.Q)
	qB := MulRot(getBodyTransform(w, js.bodyIdB).Q, js.localFrameB.Q)
	return RelativeAngle(qA, qB)
}

// SetLinearHertz changes the linear spring stiffness of a motor or weld
// joint.
func (jointId JointId) SetLinearHertz(hertz Q) {
	checkValidQ(hertz, "SetLinearHertz")
	_, j, js := jointSimOf(jointId)
	switch j.jointType {
	case MotorJoint:
		js.motor().linearHertz = hertz
	case WeldJoint:
		if hertz.Less(QZero()) {
			panic("dbox2d: SetLinearHertz needs a non-negative value")
		}
		js.weld().linearHertz = hertz
	default:
		unsupported("a linear spring")
	}
}

// GetLinearHertz reports the linear spring stiffness.
func (jointId JointId) GetLinearHertz() Q {
	_, j, js := jointSimOf(jointId)
	switch j.jointType {
	case MotorJoint:
		return js.motor().linearHertz
	case WeldJoint:
		return js.weld().linearHertz
	default:
		unsupported("a linear spring")
		return QZero()
	}
}

// SetLinearDampingRatio changes the linear spring damping ratio of a motor
// or weld joint.
func (jointId JointId) SetLinearDampingRatio(dampingRatio Q) {
	checkValidQ(dampingRatio, "SetLinearDampingRatio")
	_, j, js := jointSimOf(jointId)
	switch j.jointType {
	case MotorJoint:
		js.motor().linearDampingRatio = dampingRatio
	case WeldJoint:
		if dampingRatio.Less(QZero()) {
			panic("dbox2d: SetLinearDampingRatio needs a non-negative value")
		}
		js.weld().linearDampingRatio = dampingRatio
	default:
		unsupported("a linear spring")
	}
}

// GetLinearDampingRatio reports the linear spring damping ratio.
func (jointId JointId) GetLinearDampingRatio() Q {
	_, j, js := jointSimOf(jointId)
	switch j.jointType {
	case MotorJoint:
		return js.motor().linearDampingRatio
	case WeldJoint:
		return js.weld().linearDampingRatio
	default:
		unsupported("a linear spring")
		return QZero()
	}
}

// SetAngularHertz changes the angular spring stiffness of a motor or weld
// joint.
func (jointId JointId) SetAngularHertz(hertz Q) {
	checkValidQ(hertz, "SetAngularHertz")
	_, j, js := jointSimOf(jointId)
	switch j.jointType {
	case MotorJoint:
		js.motor().angularHertz = hertz
	case WeldJoint:
		if hertz.Less(QZero()) {
			panic("dbox2d: SetAngularHertz needs a non-negative value")
		}
		js.weld().angularHertz = hertz
	default:
		unsupported("an angular spring")
	}
}

// GetAngularHertz reports the angular spring stiffness.
func (jointId JointId) GetAngularHertz() Q {
	_, j, js := jointSimOf(jointId)
	switch j.jointType {
	case MotorJoint:
		return js.motor().angularHertz
	case WeldJoint:
		return js.weld().angularHertz
	default:
		unsupported("an angular spring")
		return QZero()
	}
}

// SetAngularDampingRatio changes the angular spring damping ratio of a
// motor or weld joint.
func (jointId JointId) SetAngularDampingRatio(dampingRatio Q) {
	checkValidQ(dampingRatio, "SetAngularDampingRatio")
	_, j, js := jointSimOf(jointId)
	switch j.jointType {
	case MotorJoint:
		js.motor().angularDampingRatio = dampingRatio
	case WeldJoint:
		if dampingRatio.Less(QZero()) {
			panic("dbox2d: SetAngularDampingRatio needs a non-negative value")
		}
		js.weld().angularDampingRatio = dampingRatio
	default:
		unsupported("an angular spring")
	}
}

// GetAngularDampingRatio reports the angular spring damping ratio.
func (jointId JointId) GetAngularDampingRatio() Q {
	_, j, js := jointSimOf(jointId)
	switch j.jointType {
	case MotorJoint:
		return js.motor().angularDampingRatio
	case WeldJoint:
		return js.weld().angularDampingRatio
	default:
		unsupported("an angular spring")
		return QZero()
	}
}

// SetLinearVelocity changes the target linear velocity of a motor or mover
// joint.
func (jointId JointId) SetLinearVelocity(velocity Vec2) {
	if !IsValidVec2(velocity) {
		panic("dbox2d: SetLinearVelocity needs a valid vector")
	}
	_, j, js := jointSimOf(jointId)
	switch j.jointType {
	case MotorJoint:
		js.motor().linearVelocity = velocity
	case MoverJoint:
		js.mover().linearVelocity = velocity
	default:
		unsupported("a linear velocity")
	}
}

// GetLinearVelocity reports the target linear velocity of a motor or mover
// joint.
func (jointId JointId) GetLinearVelocity() Vec2 {
	_, j, js := jointSimOf(jointId)
	switch j.jointType {
	case MotorJoint:
		return js.motor().linearVelocity
	case MoverJoint:
		return js.mover().linearVelocity
	default:
		unsupported("a linear velocity")
		return Vec2Zero()
	}
}

// SetAngularVelocity changes the target angular velocity of a motor joint
// in turns per second.
func (jointId JointId) SetAngularVelocity(velocity Q) {
	checkValidQ(velocity, "SetAngularVelocity")
	w := getWorld(jointId.world0)
	getJointSimCheckType(w, jointId, MotorJoint).motor().angularVelocity = angleFromTurns(velocity)
}

// GetAngularVelocity reports the target angular velocity of a motor joint
// in turns per second.
func (jointId JointId) GetAngularVelocity() Q {
	w := getWorld(jointId.world0)
	return angleToTurns(getJointSimCheckType(w, jointId, MotorJoint).motor().angularVelocity)
}

// SetMaxVelocityForce changes the maximum force of the motor joint velocity
// constraint. The mover joint takes a per-axis vector: see
// SetMoverMaxVelocityForce.
func (jointId JointId) SetMaxVelocityForce(maxForce Q) {
	checkValidQ(maxForce, "SetMaxVelocityForce")
	w := getWorld(jointId.world0)
	getJointSimCheckType(w, jointId, MotorJoint).motor().maxVelocityForce = maxForce
}

// GetMaxVelocityForce reports the maximum force of the motor joint velocity
// constraint.
func (jointId JointId) GetMaxVelocityForce() Q {
	w := getWorld(jointId.world0)
	return getJointSimCheckType(w, jointId, MotorJoint).motor().maxVelocityForce
}

// SetMoverMaxVelocityForce changes the per-axis maximum force of a mover
// joint (b2MoverJoint_SetMaxVelocityForce).
func (jointId JointId) SetMoverMaxVelocityForce(maxForce Vec2) {
	if !IsValidVec2(maxForce) {
		panic("dbox2d: SetMoverMaxVelocityForce needs a valid vector")
	}
	w := getWorld(jointId.world0)
	getJointSimCheckType(w, jointId, MoverJoint).mover().maxVelocityForce = maxForce
}

// GetMoverMaxVelocityForce reports the per-axis maximum force of a mover
// joint (b2MoverJoint_GetMaxVelocityForce).
func (jointId JointId) GetMoverMaxVelocityForce() Vec2 {
	w := getWorld(jointId.world0)
	return getJointSimCheckType(w, jointId, MoverJoint).mover().maxVelocityForce
}

// SetMaxVelocityTorque changes the maximum torque of the motor joint
// velocity constraint.
func (jointId JointId) SetMaxVelocityTorque(maxTorque Q) {
	checkValidQ(maxTorque, "SetMaxVelocityTorque")
	w := getWorld(jointId.world0)
	getJointSimCheckType(w, jointId, MotorJoint).motor().maxVelocityTorque = maxTorque
}

// GetMaxVelocityTorque reports the maximum torque of the motor joint
// velocity constraint.
func (jointId JointId) GetMaxVelocityTorque() Q {
	w := getWorld(jointId.world0)
	return getJointSimCheckType(w, jointId, MotorJoint).motor().maxVelocityTorque
}

// SetMaxSpringForce changes the maximum force of the motor joint linear
// spring. A negative value counts as zero.
func (jointId JointId) SetMaxSpringForce(maxForce Q) {
	checkValidQ(maxForce, "SetMaxSpringForce")
	w := getWorld(jointId.world0)
	getJointSimCheckType(w, jointId, MotorJoint).motor().maxSpringForce = maxForce.Max(QZero())
}

// GetMaxSpringForce reports the maximum force of the motor joint linear
// spring.
func (jointId JointId) GetMaxSpringForce() Q {
	w := getWorld(jointId.world0)
	return getJointSimCheckType(w, jointId, MotorJoint).motor().maxSpringForce
}

// SetMaxSpringTorque changes the maximum torque of the motor joint angular
// spring. A negative value counts as zero.
func (jointId JointId) SetMaxSpringTorque(maxTorque Q) {
	checkValidQ(maxTorque, "SetMaxSpringTorque")
	w := getWorld(jointId.world0)
	getJointSimCheckType(w, jointId, MotorJoint).motor().maxSpringTorque = maxTorque.Max(QZero())
}

// GetMaxSpringTorque reports the maximum torque of the motor joint angular
// spring.
func (jointId JointId) GetMaxSpringTorque() Q {
	w := getWorld(jointId.world0)
	return getJointSimCheckType(w, jointId, MotorJoint).motor().maxSpringTorque
}

// GetImpulse reports the spring impulse of a pogo joint in the last step.
func (jointId JointId) GetImpulse() Q {
	w := getWorld(jointId.world0)
	return getJointSimCheckType(w, jointId, PogoJoint).pogo().impulse
}
