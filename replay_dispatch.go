package b2

// Derived from src/recording_ops.inl of the reference by a one-off script
// and maintained by hand since.

// dispatch reads the arguments of one record and calls the public API. It
// corresponds to the switch of b2RecDispatchOne in src/replay.c. It reports
// false for an opcode it does not know.
func (rd *recReader) dispatch(op byte) bool {
	switch op {
	case 0x01: // DestroyWorld
		rd.worldIdRaw()
		if !rd.ok {
			return true
		}
		// The player owns the lifetime of the replay world.
	case 0x80: // Step
		rd.worldIdRaw()
		a1 := rd.q()
		a2 := rd.i32()
		if !rd.ok {
			return true
		}
		rd.worldId.Step(a1, int(a2))
	case 0x02: // WorldEnableSleeping
		rd.worldIdRaw()
		a1 := rd.boolean()
		if !rd.ok {
			return true
		}
		rd.worldId.EnableSleeping(a1)
	case 0x03: // WorldEnableContinuous
		rd.worldIdRaw()
		a1 := rd.boolean()
		if !rd.ok {
			return true
		}
		rd.worldId.EnableContinuous(a1)
	case 0x04: // WorldSetRestitutionThreshold
		rd.worldIdRaw()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		rd.worldId.SetRestitutionThreshold(a1)
	case 0x05: // WorldSetHitEventThreshold
		rd.worldIdRaw()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		rd.worldId.SetHitEventThreshold(a1)
	case 0x06: // WorldSetGravity
		rd.worldIdRaw()
		a1 := rd.vec2()
		if !rd.ok {
			return true
		}
		rd.worldId.SetGravity(a1)
	case 0x07: // WorldExplode
		rd.worldIdRaw()
		a1 := rd.explosionDef()
		if !rd.ok {
			return true
		}
		rd.worldId.Explode(&a1)
	case 0x08: // WorldSetContactTuning
		rd.worldIdRaw()
		a1 := rd.q()
		a2 := rd.q()
		a3 := rd.q()
		if !rd.ok {
			return true
		}
		rd.worldId.SetContactTuning(a1, a2, a3)
	case 0x09: // WorldSetContactRecycleDistance
		rd.worldIdRaw()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		rd.worldId.SetContactRecycleDistance(a1)
	case 0x0A: // WorldSetMaxLinearSpeed
		rd.worldIdRaw()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		rd.worldId.SetMaxLinearSpeed(a1)
	case 0x0B: // WorldEnableWarmStarting
		rd.worldIdRaw()
		a1 := rd.boolean()
		if !rd.ok {
			return true
		}
		rd.worldId.EnableWarmStarting(a1)
	case 0x0C: // WorldRebuildStaticTree
		rd.worldIdRaw()
		if !rd.ok {
			return true
		}
		rd.worldId.RebuildStaticTree()
	case 0x0D: // WorldSetRestitutionIterations
		rd.worldIdRaw()
		a1 := rd.i32()
		if !rd.ok {
			return true
		}
		rd.worldId.SetRestitutionIterations(int(a1))
	case 0x0E: // WorldEnableRestitutionPropagation
		rd.worldIdRaw()
		a1 := rd.boolean()
		if !rd.ok {
			return true
		}
		rd.worldId.EnableRestitutionPropagation(a1)
	case 0x10: // CreateBody
		rd.worldIdRaw()
		a1 := rd.bodyDef()
		if !rd.ok {
			return true
		}
		rec := rd.bodyIdRaw()
		got := CreateBody(rd.worldId, &a1)
		rd.checkId("body", int(got.index1), uint32(got.generation), int(rec.index1), uint32(rec.generation))
		if rd.owner != nil {
			rd.owner.trackBodyCreate(got)
		}
	case 0x11: // DestroyBody
		a0 := rd.bodyId()
		if !rd.ok {
			return true
		}
		if rd.owner != nil {
			rd.owner.trackBodyDestroy(a0)
		}
		DestroyBody(a0)
	case 0x20: // BodySetTransform
		a0 := rd.bodyId()
		a1 := rd.pos()
		a2 := rd.rot()
		if !rd.ok {
			return true
		}
		a0.SetTransform(a1, a2)
	case 0x21: // BodySetLinearVelocity
		a0 := rd.bodyId()
		a1 := rd.vec2()
		if !rd.ok {
			return true
		}
		a0.SetLinearVelocity(a1)
	case 0x22: // BodySetType
		a0 := rd.bodyId()
		a1 := rd.i32()
		if !rd.ok {
			return true
		}
		a0.SetType(BodyType(a1))
	case 0x23: // BodySetName
		a0 := rd.bodyId()
		a1 := rd.str()
		if !rd.ok {
			return true
		}
		a0.SetName(a1)
	case 0x24: // BodySetAngularVelocity
		a0 := rd.bodyId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.setAngularVelocityRadians(a1)
	case 0x25: // BodySetTargetTransform
		a0 := rd.bodyId()
		a1 := rd.worldXf()
		a2 := rd.q()
		a3 := rd.boolean()
		if !rd.ok {
			return true
		}
		a0.SetTargetTransform(a1, a2, a3)
	case 0x26: // BodyApplyForce
		a0 := rd.bodyId()
		a1 := rd.vec2()
		a2 := rd.pos()
		a3 := rd.boolean()
		if !rd.ok {
			return true
		}
		a0.ApplyForce(a1, a2, a3)
	case 0x27: // BodyApplyForceToCenter
		a0 := rd.bodyId()
		a1 := rd.vec2()
		a2 := rd.boolean()
		if !rd.ok {
			return true
		}
		a0.ApplyForceToCenter(a1, a2)
	case 0x28: // BodyApplyTorque
		a0 := rd.bodyId()
		a1 := rd.q()
		a2 := rd.boolean()
		if !rd.ok {
			return true
		}
		a0.ApplyTorque(a1, a2)
	case 0x29: // BodyClearForces
		a0 := rd.bodyId()
		if !rd.ok {
			return true
		}
		a0.ClearForces()
	case 0x2A: // BodyApplyLinearImpulse
		a0 := rd.bodyId()
		a1 := rd.vec2()
		a2 := rd.pos()
		a3 := rd.boolean()
		if !rd.ok {
			return true
		}
		a0.ApplyLinearImpulse(a1, a2, a3)
	case 0x2B: // BodyApplyLinearImpulseToCenter
		a0 := rd.bodyId()
		a1 := rd.vec2()
		a2 := rd.boolean()
		if !rd.ok {
			return true
		}
		a0.ApplyLinearImpulseToCenter(a1, a2)
	case 0x2C: // BodyApplyAngularImpulse
		a0 := rd.bodyId()
		a1 := rd.q()
		a2 := rd.boolean()
		if !rd.ok {
			return true
		}
		a0.ApplyAngularImpulse(a1, a2)
	case 0x2D: // BodySetMassData
		a0 := rd.bodyId()
		a1 := rd.massData()
		if !rd.ok {
			return true
		}
		a0.SetMassData(a1)
	case 0x2E: // BodyUpdateMassFromShapes
		a0 := rd.bodyId()
		if !rd.ok {
			return true
		}
		a0.UpdateMassFromShapes()
	case 0x2F: // BodySetLinearDamping
		a0 := rd.bodyId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetLinearDamping(a1)
	case 0x30: // BodySetAngularDamping
		a0 := rd.bodyId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetAngularDamping(a1)
	case 0x31: // BodySetGravityScale
		a0 := rd.bodyId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetGravityScale(a1)
	case 0x32: // BodySetAwake
		a0 := rd.bodyId()
		a1 := rd.boolean()
		if !rd.ok {
			return true
		}
		a0.SetAwake(a1)
	case 0x33: // BodyWakeTouching
		a0 := rd.bodyId()
		if !rd.ok {
			return true
		}
		a0.WakeTouching()
	case 0x34: // BodyEnableSleeping
		a0 := rd.bodyId()
		a1 := rd.boolean()
		if !rd.ok {
			return true
		}
		a0.EnableSleeping(a1)
	case 0x35: // BodySetSleepThreshold
		a0 := rd.bodyId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetSleepThreshold(a1)
	case 0x36: // BodyDisable
		a0 := rd.bodyId()
		if !rd.ok {
			return true
		}
		a0.Disable()
	case 0x37: // BodyEnable
		a0 := rd.bodyId()
		if !rd.ok {
			return true
		}
		a0.Enable()
	case 0x38: // BodySetMotionLocks
		a0 := rd.bodyId()
		a1 := rd.locks()
		if !rd.ok {
			return true
		}
		a0.SetMotionLocks(a1)
	case 0x39: // BodySetBullet
		a0 := rd.bodyId()
		a1 := rd.boolean()
		if !rd.ok {
			return true
		}
		a0.SetBullet(a1)
	case 0x3A: // BodyEnableContactRecycling
		a0 := rd.bodyId()
		a1 := rd.boolean()
		if !rd.ok {
			return true
		}
		a0.EnableContactRecycling(a1)
	case 0x3B: // BodyEnableContactEvents
		a0 := rd.bodyId()
		a1 := rd.boolean()
		if !rd.ok {
			return true
		}
		a0.EnableContactEvents(a1)
	case 0x3C: // BodyEnableHitEvents
		a0 := rd.bodyId()
		a1 := rd.boolean()
		if !rd.ok {
			return true
		}
		a0.EnableHitEvents(a1)
	case 0x3D: // BodySetSafetyFactor
		a0 := rd.bodyId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetSafetyFactor(a1)
	case 0x40: // CreateCircleShape
		a0 := rd.bodyId()
		a1 := rd.shapeDef()
		a2 := rd.circle()
		if !rd.ok {
			return true
		}
		rec := rd.shapeIdRaw()
		got := CreateCircleShape(a0, &a1, &a2)
		rd.checkId("shape", int(got.index1), uint32(got.generation), int(rec.index1), uint32(rec.generation))
	case 0x41: // CreateCapsuleShape
		a0 := rd.bodyId()
		a1 := rd.shapeDef()
		a2 := rd.capsule()
		if !rd.ok {
			return true
		}
		rec := rd.shapeIdRaw()
		got := CreateCapsuleShape(a0, &a1, &a2)
		rd.checkId("shape", int(got.index1), uint32(got.generation), int(rec.index1), uint32(rec.generation))
	case 0x42: // CreateSegmentShape
		a0 := rd.bodyId()
		a1 := rd.shapeDef()
		a2 := rd.segment()
		if !rd.ok {
			return true
		}
		rec := rd.shapeIdRaw()
		got := CreateSegmentShape(a0, &a1, &a2)
		rd.checkId("shape", int(got.index1), uint32(got.generation), int(rec.index1), uint32(rec.generation))
	case 0x43: // CreatePolygonShape
		a0 := rd.bodyId()
		a1 := rd.shapeDef()
		a2 := rd.polygon()
		if !rd.ok {
			return true
		}
		rec := rd.shapeIdRaw()
		got := CreatePolygonShape(a0, &a1, &a2)
		rd.checkId("shape", int(got.index1), uint32(got.generation), int(rec.index1), uint32(rec.generation))
	case 0x44: // CreateChainSegmentShape
		a0 := rd.bodyId()
		a1 := rd.shapeDef()
		a2 := rd.chainSeg()
		if !rd.ok {
			return true
		}
		rec := rd.shapeIdRaw()
		got := CreateChainSegmentShape(a0, &a1, &a2)
		rd.checkId("shape", int(got.index1), uint32(got.generation), int(rec.index1), uint32(rec.generation))
	case 0x45: // DestroyShape
		a0 := rd.shapeId()
		a1 := rd.boolean()
		if !rd.ok {
			return true
		}
		DestroyShape(a0, a1)
	case 0x50: // ShapeSetDensity
		a0 := rd.shapeId()
		a1 := rd.q()
		a2 := rd.boolean()
		if !rd.ok {
			return true
		}
		a0.SetDensity(a1, a2)
	case 0x51: // ShapeSetFriction
		a0 := rd.shapeId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetFriction(a1)
	case 0x52: // ShapeSetRestitution
		a0 := rd.shapeId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetRestitution(a1)
	case 0x53: // ShapeSetUserMaterial
		a0 := rd.shapeId()
		a1 := rd.u64()
		if !rd.ok {
			return true
		}
		a0.SetUserMaterial(a1)
	case 0x54: // ShapeSetSurfaceMaterial
		a0 := rd.shapeId()
		a1 := rd.material()
		if !rd.ok {
			return true
		}
		a0.SetSurfaceMaterial(a1)
	case 0x55: // ShapeSetFilter
		a0 := rd.shapeId()
		a1 := rd.filter()
		if !rd.ok {
			return true
		}
		a0.SetFilter(a1)
	case 0x56: // ShapeEnableSensorEvents
		a0 := rd.shapeId()
		a1 := rd.boolean()
		if !rd.ok {
			return true
		}
		a0.EnableSensorEvents(a1)
	case 0x57: // ShapeEnableContactEvents
		a0 := rd.shapeId()
		a1 := rd.boolean()
		if !rd.ok {
			return true
		}
		a0.EnableContactEvents(a1)
	case 0x58: // ShapeEnablePreSolveEvents
		a0 := rd.shapeId()
		a1 := rd.boolean()
		if !rd.ok {
			return true
		}
		a0.EnablePreSolveEvents(a1)
	case 0x59: // ShapeEnableHitEvents
		a0 := rd.shapeId()
		a1 := rd.boolean()
		if !rd.ok {
			return true
		}
		a0.EnableHitEvents(a1)
	case 0x5A: // ShapeSetCircle
		a0 := rd.shapeId()
		a1 := rd.circle()
		if !rd.ok {
			return true
		}
		a0.SetCircle(&a1)
	case 0x5B: // ShapeSetCapsule
		a0 := rd.shapeId()
		a1 := rd.capsule()
		if !rd.ok {
			return true
		}
		a0.SetCapsule(&a1)
	case 0x5C: // ShapeSetSegment
		a0 := rd.shapeId()
		a1 := rd.segment()
		if !rd.ok {
			return true
		}
		a0.SetSegment(&a1)
	case 0x5D: // ShapeSetPolygon
		a0 := rd.shapeId()
		a1 := rd.polygon()
		if !rd.ok {
			return true
		}
		a0.SetPolygon(&a1)
	case 0x5E: // ShapeSetChainSegment
		a0 := rd.shapeId()
		a1 := rd.chainSeg()
		if !rd.ok {
			return true
		}
		a0.SetChainSegment(&a1)
	case 0x5F: // ShapeApplyWind
		a0 := rd.shapeId()
		a1 := rd.vec2()
		a2 := rd.q()
		a3 := rd.q()
		a4 := rd.boolean()
		if !rd.ok {
			return true
		}
		a0.ApplyWind(a1, a2, a3, a4)
	case 0x70: // CreateChain
		a0 := rd.bodyId()
		a1 := rd.chainDef()
		if !rd.ok {
			return true
		}
		rec := rd.chainIdRaw()
		got := CreateChain(a0, &a1)
		rd.checkId("chain", int(got.index1), uint32(got.generation), int(rec.index1), uint32(rec.generation))
	case 0x71: // DestroyChain
		a0 := rd.chainId()
		if !rd.ok {
			return true
		}
		DestroyChain(a0)
	case 0x72: // ChainSetSurfaceMaterial
		a0 := rd.chainId()
		a1 := rd.material()
		a2 := rd.i32()
		if !rd.ok {
			return true
		}
		a0.SetSurfaceMaterial(a1, int(a2))
	case 0x73: // ChainSetAllSurfaceMaterials
		a0 := rd.chainId()
		a1 := rd.material()
		if !rd.ok {
			return true
		}
		a0.SetAllSurfaceMaterials(a1)
	case 0x90: // DestroyJoint
		a0 := rd.jointId()
		if !rd.ok {
			return true
		}
		DestroyJoint(a0)
	case 0x91: // JointSetLocalFrameA
		a0 := rd.jointId()
		a1 := rd.xf()
		if !rd.ok {
			return true
		}
		a0.SetLocalFrameA(a1)
	case 0x92: // JointSetLocalFrameB
		a0 := rd.jointId()
		a1 := rd.xf()
		if !rd.ok {
			return true
		}
		a0.SetLocalFrameB(a1)
	case 0x93: // JointSetCollideConnected
		a0 := rd.jointId()
		a1 := rd.boolean()
		if !rd.ok {
			return true
		}
		a0.SetCollideConnected(a1)
	case 0x94: // JointWakeBodies
		a0 := rd.jointId()
		if !rd.ok {
			return true
		}
		a0.WakeBodies()
	case 0x95: // JointSetConstraintTuning
		a0 := rd.jointId()
		a1 := rd.q()
		a2 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetConstraintTuning(a1, a2)
	case 0x96: // JointSetForceThreshold
		a0 := rd.jointId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetForceThreshold(a1)
	case 0x97: // JointSetTorqueThreshold
		a0 := rd.jointId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetTorqueThreshold(a1)
	case 0x98: // CreateDistanceJoint
		rd.worldIdRaw()
		a1 := rd.distanceJointDef()
		if !rd.ok {
			return true
		}
		rec := rd.jointIdRaw()
		a1.Base.BodyIdA = rd.makeBody(a1.Base.BodyIdA)
		a1.Base.BodyIdB = rd.makeBody(a1.Base.BodyIdB)
		got := CreateDistanceJoint(rd.worldId, &a1)
		rd.checkId("joint", int(got.index1), uint32(got.generation), int(rec.index1), uint32(rec.generation))
	case 0x99: // DistanceJointSetRestLength
		a0 := rd.jointId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetRestLength(a1)
	case 0x9A: // DistanceJointEnableSpring
		a0 := rd.jointId()
		a1 := rd.boolean()
		if !rd.ok {
			return true
		}
		a0.EnableSpring(a1)
	case 0x9B: // DistanceJointSetSpringForceRange
		a0 := rd.jointId()
		a1 := rd.q()
		a2 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetSpringForceRange(a1, a2)
	case 0x9C: // DistanceJointSetSpringHertz
		a0 := rd.jointId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetSpringHertz(a1)
	case 0x9D: // DistanceJointSetSpringDampingRatio
		a0 := rd.jointId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetSpringDampingRatio(a1)
	case 0x9E: // DistanceJointEnableLimit
		a0 := rd.jointId()
		a1 := rd.boolean()
		if !rd.ok {
			return true
		}
		a0.EnableLimit(a1)
	case 0x9F: // DistanceJointSetLimits
		a0 := rd.jointId()
		a1 := rd.q()
		a2 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetLimits(a1, a2)
	case 0xA0: // DistanceJointEnableMotor
		a0 := rd.jointId()
		a1 := rd.boolean()
		if !rd.ok {
			return true
		}
		a0.EnableMotor(a1)
	case 0xA1: // DistanceJointSetTargetMotorSpeed
		a0 := rd.jointId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetTargetMotorSpeed(a1)
	case 0xA2: // DistanceJointSetMaxMotorForce
		a0 := rd.jointId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetMaxMotorForce(a1)
	case 0xA3: // CreateFilterJoint
		rd.worldIdRaw()
		a1 := rd.filterJointDef()
		if !rd.ok {
			return true
		}
		rec := rd.jointIdRaw()
		a1.Base.BodyIdA = rd.makeBody(a1.Base.BodyIdA)
		a1.Base.BodyIdB = rd.makeBody(a1.Base.BodyIdB)
		got := CreateFilterJoint(rd.worldId, &a1)
		rd.checkId("joint", int(got.index1), uint32(got.generation), int(rec.index1), uint32(rec.generation))
	case 0xA4: // CreateMotorJoint
		rd.worldIdRaw()
		a1 := rd.motorJointDef()
		if !rd.ok {
			return true
		}
		rec := rd.jointIdRaw()
		a1.Base.BodyIdA = rd.makeBody(a1.Base.BodyIdA)
		a1.Base.BodyIdB = rd.makeBody(a1.Base.BodyIdB)
		got := CreateMotorJoint(rd.worldId, &a1)
		rd.checkId("joint", int(got.index1), uint32(got.generation), int(rec.index1), uint32(rec.generation))
	case 0xA5: // MotorJointSetLinearVelocity
		a0 := rd.jointId()
		a1 := rd.vec2()
		if !rd.ok {
			return true
		}
		a0.SetLinearVelocity(a1)
	case 0xA6: // MotorJointSetAngularVelocity
		a0 := rd.jointId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.setMotorAngularVelocityRaw(a1)
	case 0xA7: // MotorJointSetMaxVelocityForce
		a0 := rd.jointId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetMaxVelocityForce(a1)
	case 0xA8: // MotorJointSetMaxVelocityTorque
		a0 := rd.jointId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetMaxVelocityTorque(a1)
	case 0xA9: // MotorJointSetLinearHertz
		a0 := rd.jointId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetLinearHertz(a1)
	case 0xAA: // MotorJointSetLinearDampingRatio
		a0 := rd.jointId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetLinearDampingRatio(a1)
	case 0xAB: // MotorJointSetAngularHertz
		a0 := rd.jointId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetAngularHertz(a1)
	case 0xAC: // MotorJointSetAngularDampingRatio
		a0 := rd.jointId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetAngularDampingRatio(a1)
	case 0xAD: // MotorJointSetMaxSpringForce
		a0 := rd.jointId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetMaxSpringForce(a1)
	case 0xAE: // MotorJointSetMaxSpringTorque
		a0 := rd.jointId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetMaxSpringTorque(a1)
	case 0xAF: // CreateMoverJoint
		rd.worldIdRaw()
		a1 := rd.moverJointDef()
		if !rd.ok {
			return true
		}
		rec := rd.jointIdRaw()
		a1.Base.BodyIdA = rd.makeBody(a1.Base.BodyIdA)
		a1.Base.BodyIdB = rd.makeBody(a1.Base.BodyIdB)
		got := CreateMoverJoint(rd.worldId, &a1)
		rd.checkId("joint", int(got.index1), uint32(got.generation), int(rec.index1), uint32(rec.generation))
	case 0xB0: // MoverJointSetLinearVelocity
		a0 := rd.jointId()
		a1 := rd.vec2()
		if !rd.ok {
			return true
		}
		a0.SetLinearVelocity(a1)
	case 0xB1: // MoverJointSetMaxVelocityForce
		a0 := rd.jointId()
		a1 := rd.vec2()
		if !rd.ok {
			return true
		}
		a0.SetMoverMaxVelocityForce(a1)
	case 0xB2: // CreatePogoJoint
		rd.worldIdRaw()
		a1 := rd.pogoJointDef()
		if !rd.ok {
			return true
		}
		rec := rd.jointIdRaw()
		a1.Base.BodyIdA = rd.makeBody(a1.Base.BodyIdA)
		a1.Base.BodyIdB = rd.makeBody(a1.Base.BodyIdB)
		got := CreatePogoJoint(rd.worldId, &a1)
		rd.checkId("joint", int(got.index1), uint32(got.generation), int(rec.index1), uint32(rec.generation))
	case 0xB3: // PogoJointSetRestLength
		a0 := rd.jointId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetRestLength(a1)
	case 0xB4: // PogoJointSetSpringHertz
		a0 := rd.jointId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetSpringHertz(a1)
	case 0xB5: // PogoJointSetSpringDampingRatio
		a0 := rd.jointId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetSpringDampingRatio(a1)
	case 0xB6: // CreatePrismaticJoint
		rd.worldIdRaw()
		a1 := rd.prismaticJointDef()
		if !rd.ok {
			return true
		}
		rec := rd.jointIdRaw()
		a1.Base.BodyIdA = rd.makeBody(a1.Base.BodyIdA)
		a1.Base.BodyIdB = rd.makeBody(a1.Base.BodyIdB)
		got := CreatePrismaticJoint(rd.worldId, &a1)
		rd.checkId("joint", int(got.index1), uint32(got.generation), int(rec.index1), uint32(rec.generation))
	case 0xB7: // PrismaticJointEnableSpring
		a0 := rd.jointId()
		a1 := rd.boolean()
		if !rd.ok {
			return true
		}
		a0.EnableSpring(a1)
	case 0xB8: // PrismaticJointSetSpringHertz
		a0 := rd.jointId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetSpringHertz(a1)
	case 0xB9: // PrismaticJointSetSpringDampingRatio
		a0 := rd.jointId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetSpringDampingRatio(a1)
	case 0xBA: // PrismaticJointSetTargetTranslation
		a0 := rd.jointId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetTargetTranslation(a1)
	case 0xBB: // PrismaticJointEnableLimit
		a0 := rd.jointId()
		a1 := rd.boolean()
		if !rd.ok {
			return true
		}
		a0.EnableLimit(a1)
	case 0xBC: // PrismaticJointSetLimits
		a0 := rd.jointId()
		a1 := rd.q()
		a2 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetLimits(a1, a2)
	case 0xBD: // PrismaticJointEnableMotor
		a0 := rd.jointId()
		a1 := rd.boolean()
		if !rd.ok {
			return true
		}
		a0.EnableMotor(a1)
	case 0xBE: // PrismaticJointSetTargetMotorSpeed
		a0 := rd.jointId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetTargetMotorSpeed(a1)
	case 0xBF: // PrismaticJointSetMaxMotorForce
		a0 := rd.jointId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetMaxMotorForce(a1)
	case 0xC0: // CreateRevoluteJoint
		rd.worldIdRaw()
		a1 := rd.revoluteJointDef()
		if !rd.ok {
			return true
		}
		rec := rd.jointIdRaw()
		a1.Base.BodyIdA = rd.makeBody(a1.Base.BodyIdA)
		a1.Base.BodyIdB = rd.makeBody(a1.Base.BodyIdB)
		got := CreateRevoluteJoint(rd.worldId, &a1)
		rd.checkId("joint", int(got.index1), uint32(got.generation), int(rec.index1), uint32(rec.generation))
	case 0xC1: // RevoluteJointEnableSpring
		a0 := rd.jointId()
		a1 := rd.boolean()
		if !rd.ok {
			return true
		}
		a0.EnableSpring(a1)
	case 0xC2: // RevoluteJointSetSpringHertz
		a0 := rd.jointId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetSpringHertz(a1)
	case 0xC3: // RevoluteJointSetSpringDampingRatio
		a0 := rd.jointId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetSpringDampingRatio(a1)
	case 0xC4: // RevoluteJointSetTargetAngle
		a0 := rd.jointId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.setTargetAngleRaw(a1)
	case 0xC5: // RevoluteJointEnableLimit
		a0 := rd.jointId()
		a1 := rd.boolean()
		if !rd.ok {
			return true
		}
		a0.EnableLimit(a1)
	case 0xC6: // RevoluteJointSetLimits
		a0 := rd.jointId()
		a1 := rd.q()
		a2 := rd.q()
		if !rd.ok {
			return true
		}
		a0.setRevoluteLimits(a1, a2)
	case 0xC7: // RevoluteJointEnableMotor
		a0 := rd.jointId()
		a1 := rd.boolean()
		if !rd.ok {
			return true
		}
		a0.EnableMotor(a1)
	case 0xC8: // RevoluteJointSetTargetMotorSpeed
		a0 := rd.jointId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.setTargetMotorSpeedRaw(a1)
	case 0xC9: // RevoluteJointSetMaxMotorTorque
		a0 := rd.jointId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetMaxMotorTorque(a1)
	case 0xCA: // CreateWeldJoint
		rd.worldIdRaw()
		a1 := rd.weldJointDef()
		if !rd.ok {
			return true
		}
		rec := rd.jointIdRaw()
		a1.Base.BodyIdA = rd.makeBody(a1.Base.BodyIdA)
		a1.Base.BodyIdB = rd.makeBody(a1.Base.BodyIdB)
		got := CreateWeldJoint(rd.worldId, &a1)
		rd.checkId("joint", int(got.index1), uint32(got.generation), int(rec.index1), uint32(rec.generation))
	case 0xCB: // WeldJointSetLinearHertz
		a0 := rd.jointId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetLinearHertz(a1)
	case 0xCC: // WeldJointSetLinearDampingRatio
		a0 := rd.jointId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetLinearDampingRatio(a1)
	case 0xCD: // WeldJointSetAngularHertz
		a0 := rd.jointId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetAngularHertz(a1)
	case 0xCE: // WeldJointSetAngularDampingRatio
		a0 := rd.jointId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetAngularDampingRatio(a1)
	case 0xCF: // CreateWheelJoint
		rd.worldIdRaw()
		a1 := rd.wheelJointDef()
		if !rd.ok {
			return true
		}
		rec := rd.jointIdRaw()
		a1.Base.BodyIdA = rd.makeBody(a1.Base.BodyIdA)
		a1.Base.BodyIdB = rd.makeBody(a1.Base.BodyIdB)
		got := CreateWheelJoint(rd.worldId, &a1)
		rd.checkId("joint", int(got.index1), uint32(got.generation), int(rec.index1), uint32(rec.generation))
	case 0xD0: // WheelJointEnableSpring
		a0 := rd.jointId()
		a1 := rd.boolean()
		if !rd.ok {
			return true
		}
		a0.EnableSpring(a1)
	case 0xD1: // WheelJointSetSpringHertz
		a0 := rd.jointId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetSpringHertz(a1)
	case 0xD2: // WheelJointSetSpringDampingRatio
		a0 := rd.jointId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetSpringDampingRatio(a1)
	case 0xD3: // WheelJointEnableLimit
		a0 := rd.jointId()
		a1 := rd.boolean()
		if !rd.ok {
			return true
		}
		a0.EnableLimit(a1)
	case 0xD4: // WheelJointSetLimits
		a0 := rd.jointId()
		a1 := rd.q()
		a2 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetLimits(a1, a2)
	case 0xD5: // WheelJointEnableMotor
		a0 := rd.jointId()
		a1 := rd.boolean()
		if !rd.ok {
			return true
		}
		a0.EnableMotor(a1)
	case 0xD6: // WheelJointSetTargetMotorSpeed
		a0 := rd.jointId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.setTargetMotorSpeedRaw(a1)
	case 0xD7: // WheelJointSetMaxMotorTorque
		a0 := rd.jointId()
		a1 := rd.q()
		if !rd.ok {
			return true
		}
		a0.SetMaxMotorTorque(a1)
	case 0xE0: // QueryOverlapAABB
		rd.worldIdRaw()
		a1 := rd.pos()
		a2 := rd.aabb()
		a3 := rd.queryFilter()
		if !rd.ok {
			return true
		}
		rd.replayQueryOverlapAABB(a1, a2, a3)
	case 0xE1: // QueryOverlapShape
		rd.worldIdRaw()
		a1 := rd.pos()
		a2 := rd.shapeProxy()
		a3 := rd.queryFilter()
		if !rd.ok {
			return true
		}
		rd.replayQueryOverlapShape(a1, a2, a3)
	case 0xE2: // QueryCastRay
		rd.worldIdRaw()
		a1 := rd.pos()
		a2 := rd.vec2()
		a3 := rd.queryFilter()
		if !rd.ok {
			return true
		}
		rd.replayQueryCastRay(a1, a2, a3)
	case 0xE3: // QueryCastShape
		rd.worldIdRaw()
		a1 := rd.pos()
		a2 := rd.shapeProxy()
		a3 := rd.vec2()
		a4 := rd.queryFilter()
		if !rd.ok {
			return true
		}
		rd.replayQueryCastShape(a1, a2, a3, a4)
	case 0xE4: // QueryCollideMover
		rd.worldIdRaw()
		a1 := rd.pos()
		a2 := rd.capsule()
		a3 := rd.queryFilter()
		if !rd.ok {
			return true
		}
		rd.replayQueryCollideMover(a1, &a2, a3)
	case 0xE5: // QueryCastRayClosest
		rd.worldIdRaw()
		a1 := rd.pos()
		a2 := rd.vec2()
		a3 := rd.queryFilter()
		if !rd.ok {
			return true
		}
		rd.replayQueryCastRayClosest(a1, a2, a3)
	case 0xE6: // QueryCastMover
		rd.worldIdRaw()
		a1 := rd.pos()
		a2 := rd.capsule()
		a3 := rd.vec2()
		a4 := rd.queryFilter()
		if !rd.ok {
			return true
		}
		rd.replayQueryCastMover(a1, &a2, a3, a4)
	case 0xF1: // StateHash
		rd.worldIdRaw()
		a1 := rd.u64()
		if !rd.ok {
			return true
		}
		rd.stateHash(a1)
	case 0xF2: // RecordingBounds
		a0 := rd.aabb()
		if !rd.ok {
			return true
		}
		if rd.owner != nil {
			rd.owner.bounds = a0
		}
	default:
		return false
	}
	return true
}
