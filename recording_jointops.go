package b2

// Derived from the op table in recording_ops.go by a one-off script and
// maintained by hand since.

// jointOpKey names a per-type joint op by the joint type and the accessor.
type jointOpKey struct {
	jointType JointType
	kind      string
}

// jointOps maps the typed joint accessors to their op codes. The port folds
// accessors shared by several joint types into one JointId method (D-003),
// so the recording hook picks the op by type.
var jointOps = map[jointOpKey]byte{
	{DistanceJoint, "SetRestLength"}:          opDistanceJointSetRestLength,
	{DistanceJoint, "EnableSpring"}:           opDistanceJointEnableSpring,
	{DistanceJoint, "SetSpringForceRange"}:    opDistanceJointSetSpringForceRange,
	{DistanceJoint, "SetSpringHertz"}:         opDistanceJointSetSpringHertz,
	{DistanceJoint, "SetSpringDampingRatio"}:  opDistanceJointSetSpringDampingRatio,
	{DistanceJoint, "EnableLimit"}:            opDistanceJointEnableLimit,
	{DistanceJoint, "SetLimits"}:              opDistanceJointSetLimits,
	{DistanceJoint, "EnableMotor"}:            opDistanceJointEnableMotor,
	{DistanceJoint, "SetTargetMotorSpeed"}:    opDistanceJointSetTargetMotorSpeed,
	{DistanceJoint, "SetMaxMotorForce"}:       opDistanceJointSetMaxMotorForce,
	{MotorJoint, "SetLinearVelocity"}:         opMotorJointSetLinearVelocity,
	{MotorJoint, "SetAngularVelocity"}:        opMotorJointSetAngularVelocity,
	{MotorJoint, "SetMaxVelocityForce"}:       opMotorJointSetMaxVelocityForce,
	{MotorJoint, "SetMaxVelocityTorque"}:      opMotorJointSetMaxVelocityTorque,
	{MotorJoint, "SetLinearHertz"}:            opMotorJointSetLinearHertz,
	{MotorJoint, "SetLinearDampingRatio"}:     opMotorJointSetLinearDampingRatio,
	{MotorJoint, "SetAngularHertz"}:           opMotorJointSetAngularHertz,
	{MotorJoint, "SetAngularDampingRatio"}:    opMotorJointSetAngularDampingRatio,
	{MotorJoint, "SetMaxSpringForce"}:         opMotorJointSetMaxSpringForce,
	{MotorJoint, "SetMaxSpringTorque"}:        opMotorJointSetMaxSpringTorque,
	{MoverJoint, "SetLinearVelocity"}:         opMoverJointSetLinearVelocity,
	{MoverJoint, "SetMaxVelocityForce"}:       opMoverJointSetMaxVelocityForce,
	{PogoJoint, "SetRestLength"}:              opPogoJointSetRestLength,
	{PogoJoint, "SetSpringHertz"}:             opPogoJointSetSpringHertz,
	{PogoJoint, "SetSpringDampingRatio"}:      opPogoJointSetSpringDampingRatio,
	{PrismaticJoint, "EnableSpring"}:          opPrismaticJointEnableSpring,
	{PrismaticJoint, "SetSpringHertz"}:        opPrismaticJointSetSpringHertz,
	{PrismaticJoint, "SetSpringDampingRatio"}: opPrismaticJointSetSpringDampingRatio,
	{PrismaticJoint, "SetTargetTranslation"}:  opPrismaticJointSetTargetTranslation,
	{PrismaticJoint, "EnableLimit"}:           opPrismaticJointEnableLimit,
	{PrismaticJoint, "SetLimits"}:             opPrismaticJointSetLimits,
	{PrismaticJoint, "EnableMotor"}:           opPrismaticJointEnableMotor,
	{PrismaticJoint, "SetTargetMotorSpeed"}:   opPrismaticJointSetTargetMotorSpeed,
	{PrismaticJoint, "SetMaxMotorForce"}:      opPrismaticJointSetMaxMotorForce,
	{RevoluteJoint, "EnableSpring"}:           opRevoluteJointEnableSpring,
	{RevoluteJoint, "SetSpringHertz"}:         opRevoluteJointSetSpringHertz,
	{RevoluteJoint, "SetSpringDampingRatio"}:  opRevoluteJointSetSpringDampingRatio,
	{RevoluteJoint, "SetTargetAngle"}:         opRevoluteJointSetTargetAngle,
	{RevoluteJoint, "EnableLimit"}:            opRevoluteJointEnableLimit,
	{RevoluteJoint, "SetLimits"}:              opRevoluteJointSetLimits,
	{RevoluteJoint, "EnableMotor"}:            opRevoluteJointEnableMotor,
	{RevoluteJoint, "SetTargetMotorSpeed"}:    opRevoluteJointSetTargetMotorSpeed,
	{RevoluteJoint, "SetMaxMotorTorque"}:      opRevoluteJointSetMaxMotorTorque,
	{WeldJoint, "SetLinearHertz"}:             opWeldJointSetLinearHertz,
	{WeldJoint, "SetLinearDampingRatio"}:      opWeldJointSetLinearDampingRatio,
	{WeldJoint, "SetAngularHertz"}:            opWeldJointSetAngularHertz,
	{WeldJoint, "SetAngularDampingRatio"}:     opWeldJointSetAngularDampingRatio,
	{WheelJoint, "EnableSpring"}:              opWheelJointEnableSpring,
	{WheelJoint, "SetSpringHertz"}:            opWheelJointSetSpringHertz,
	{WheelJoint, "SetSpringDampingRatio"}:     opWheelJointSetSpringDampingRatio,
	{WheelJoint, "EnableLimit"}:               opWheelJointEnableLimit,
	{WheelJoint, "SetLimits"}:                 opWheelJointSetLimits,
	{WheelJoint, "EnableMotor"}:               opWheelJointEnableMotor,
	{WheelJoint, "SetTargetMotorSpeed"}:       opWheelJointSetTargetMotorSpeed,
	{WheelJoint, "SetMaxMotorTorque"}:         opWheelJointSetMaxMotorTorque,
}
