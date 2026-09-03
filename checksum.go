package dbox2d

// This file has no upstream counterpart. A fixed-point world promises the
// same state on every platform, and the checksum is the witness of that
// promise. See D-011.

// FNV-1a parameters for 64 bits.
const (
	fnvOffsetBasis uint64 = 14695981039346656037
	fnvPrime       uint64 = 1099511628211
)

// fnvFold folds one 64-bit word into an FNV-1a hash, byte by byte, from the
// least significant byte.
func fnvFold(h, word uint64) uint64 {
	for range 8 {
		h ^= word & 0xff
		h *= fnvPrime
		word >>= 8
	}
	return h
}

func checksumBool(h uint64, value bool) uint64 {
	if value {
		return fnvFold(h, 1)
	}
	return fnvFold(h, 0)
}

func checksumQ(h uint64, value Q) uint64 {
	return fnvFold(h, uint64(value.Raw()))
}

func checksumVec2(h uint64, value Vec2) uint64 {
	h = checksumQ(h, value.X)
	return checksumQ(h, value.Y)
}

func checksumRot(h uint64, value Rot) uint64 {
	h = checksumQ(h, value.Cos)
	return checksumQ(h, value.Sin)
}

func checksumAABB(h uint64, value AABB) uint64 {
	h = checksumVec2(h, value.LowerBound)
	return checksumVec2(h, value.UpperBound)
}

func checksumShapeGeometry(h uint64, s *shape) uint64 {
	switch s.shapeType {
	case CapsuleShape:
		h = checksumVec2(h, s.capsule.Center1)
		h = checksumVec2(h, s.capsule.Center2)
		return checksumQ(h, s.capsule.Radius)
	case CircleShape:
		h = checksumVec2(h, s.circle.Center)
		return checksumQ(h, s.circle.Radius)
	case PolygonShape:
		h = fnvFold(h, uint64(s.polygon.Count))
		for i := range s.polygon.Count {
			h = checksumVec2(h, s.polygon.Vertices[i])
			h = checksumVec2(h, s.polygon.Normals[i])
		}
		h = checksumVec2(h, s.polygon.Centroid)
		return checksumQ(h, s.polygon.Radius)
	case SegmentShape:
		h = checksumVec2(h, s.segment.Point1)
		return checksumVec2(h, s.segment.Point2)
	case ChainSegmentShape:
		h = checksumVec2(h, s.chainSegment.Ghost1)
		h = checksumVec2(h, s.chainSegment.Segment.Point1)
		h = checksumVec2(h, s.chainSegment.Segment.Point2)
		return checksumVec2(h, s.chainSegment.Ghost2)
	default:
		panic("dbox2d: unknown shape type")
	}
}

func checksumShape(s *shape) uint64 {
	h := fnvOffsetBasis
	h = fnvFold(h, uint64(s.shapeType))
	h = checksumQ(h, s.density)
	h = checksumQ(h, s.friction)
	h = checksumQ(h, s.restitution)
	h = checksumQ(h, s.rollingResistance)
	h = checksumQ(h, s.tangentSpeed)
	h = fnvFold(h, uint64(int64(s.userMaterialId)))
	h = fnvFold(h, s.filter.CategoryBits)
	h = fnvFold(h, s.filter.MaskBits)
	h = fnvFold(h, uint64(int64(s.filter.GroupIndex)))
	h = checksumAABB(h, s.aabb)
	h = checksumAABB(h, s.fatAABB)
	h = checksumVec2(h, s.localCentroid)
	h = checksumBool(h, s.enableSensorEvents)
	h = checksumBool(h, s.enableContactEvents)
	h = checksumBool(h, s.enableHitEvents)
	h = checksumBool(h, s.enablePreSolveEvents)
	h = checksumBool(h, s.enlargedAABB)
	return checksumShapeGeometry(h, s)
}

func checksumSetKind(setIndex int) uint64 {
	if setIndex >= firstSleepingSet {
		return uint64(firstSleepingSet)
	}
	return uint64(setIndex)
}

func checksumBody(w *world, b *body) uint64 {
	set := &w.solverSets[b.setIndex]
	sim := &set.bodySims[b.localIndex]

	h := fnvOffsetBasis
	h = fnvFold(h, checksumSetKind(b.setIndex))
	h = fnvFold(h, uint64(b.bodyType))
	h = checksumQ(h, b.mass)
	h = checksumQ(h, b.inertia)
	h = checksumQ(h, b.sleepThreshold)
	h = checksumQ(h, b.sleepTime)
	h = checksumBool(h, b.enableSleep)
	h = checksumBool(h, b.fixedRotation)
	h = checksumBool(h, b.isSpeedCapped)
	h = checksumBool(h, b.isMarked)

	// A pending island split changes the next step: it blocks sleep and
	// picks the island to split. Only the sign of the count matters.
	pendingSplit := b.islandId != nullIndex && w.islands[b.islandId].constraintRemoveCount > 0
	h = checksumBool(h, pendingSplit)

	h = checksumVec2(h, sim.transform.P)
	h = checksumRot(h, sim.transform.Q)
	h = checksumVec2(h, sim.center)
	h = checksumRot(h, sim.rotation0)
	h = checksumVec2(h, sim.center0)
	h = checksumVec2(h, sim.localCenter)
	h = checksumVec2(h, sim.force)
	h = checksumQ(h, sim.torque)
	h = checksumQ(h, sim.invMass)
	h = checksumQ(h, sim.invInertia)
	h = checksumQ(h, sim.minExtent)
	h = checksumQ(h, sim.maxExtent)
	h = checksumQ(h, sim.linearDamping)
	h = checksumQ(h, sim.angularDamping)
	h = checksumQ(h, sim.gravityScale)
	h = checksumBool(h, sim.isFast)
	h = checksumBool(h, sim.isBullet)
	h = checksumBool(h, sim.isSpeedCapped)
	h = checksumBool(h, sim.allowFastRotation)
	h = checksumBool(h, sim.enlargeAABB)

	if b.setIndex == awakeSet {
		state := &set.bodyStates[b.localIndex]
		h = checksumVec2(h, state.linearVelocity)
		h = checksumQ(h, state.angularVelocity)
		h = fnvFold(h, uint64(int64(state.flags)))
		h = checksumVec2(h, state.deltaPosition)
		h = checksumRot(h, state.deltaRotation)
	}

	var shapeSum uint64
	shapeCount := 0
	for shapeId := b.headShapeId; shapeId != nullIndex; shapeId = w.shapes[shapeId].nextShapeId {
		shapeSum += checksumShape(&w.shapes[shapeId])
		shapeCount++
	}
	h = fnvFold(h, uint64(shapeCount))
	return fnvFold(h, shapeSum)
}

func checksumManifold(h uint64, m *Manifold, swap bool) uint64 {
	normal := m.Normal
	rollingImpulse := m.RollingImpulse
	if swap {
		normal = Neg(normal)
		rollingImpulse = rollingImpulse.Neg()
	}

	h = checksumVec2(h, normal)
	h = checksumQ(h, rollingImpulse)

	var pointSum uint64
	for i := range m.PointCount {
		p := &m.Points[i]
		anchorA := p.AnchorA
		anchorB := p.AnchorB
		id := p.Id
		if swap {
			anchorA, anchorB = anchorB, anchorA
			id = id<<8 | id>>8
		}

		pointHash := fnvOffsetBasis
		pointHash = checksumVec2(pointHash, p.Point)
		pointHash = checksumVec2(pointHash, anchorA)
		pointHash = checksumVec2(pointHash, anchorB)
		pointHash = checksumQ(pointHash, p.Separation)
		pointHash = checksumQ(pointHash, p.NormalImpulse)
		pointHash = checksumQ(pointHash, p.TangentImpulse)
		pointHash = checksumQ(pointHash, p.TotalNormalImpulse)
		pointHash = checksumQ(pointHash, p.NormalVelocity)
		pointHash = fnvFold(pointHash, uint64(id))
		pointHash = checksumBool(pointHash, p.Persisted)
		pointSum += pointHash
	}
	h = fnvFold(h, uint64(m.PointCount))
	return fnvFold(h, pointSum)
}

func checksumContactEndpoint(w *world, shapeId int) uint64 {
	s := &w.shapes[shapeId]
	b := &w.bodies[s.bodyId]

	h := fnvOffsetBasis
	h = fnvFold(h, checksumBody(w, b))
	return fnvFold(h, checksumShape(s))
}

func checksumContactOrientation(c *contact, cs *contactSim, endpointA, endpointB uint64, swap bool) uint64 {
	if swap {
		endpointA, endpointB = endpointB, endpointA
	}

	h := fnvOffsetBasis
	h = fnvFold(h, checksumSetKind(c.setIndex))
	h = fnvFold(h, endpointA)
	h = fnvFold(h, endpointB)
	h = fnvFold(h, uint64(c.flags))
	h = checksumQ(h, cs.friction)
	h = checksumQ(h, cs.restitution)
	h = checksumQ(h, cs.rollingResistance)
	h = checksumQ(h, cs.tangentSpeed)
	h = fnvFold(h, uint64(cs.simFlags))
	return checksumManifold(h, &cs.manifold, swap)
}

func checksumContact(w *world, c *contact) uint64 {
	cs := getContactSim(w, c)
	endpointA := checksumContactEndpoint(w, c.shapeIdA)
	endpointB := checksumContactEndpoint(w, c.shapeIdB)

	forward := checksumContactOrientation(c, cs, endpointA, endpointB, false)
	reverse := checksumContactOrientation(c, cs, endpointA, endpointB, true)
	return min(forward, reverse)
}

func checksumSoftness(h uint64, s softness) uint64 {
	h = checksumQ(h, s.biasRate)
	h = checksumQ(h, s.massScale)
	return checksumQ(h, s.impulseScale)
}

// checksumJointData folds the configuration and the accumulated impulses
// of the live joint type. The scratch of the prepare stage stays out.
func checksumJointData(h uint64, js *jointSim) uint64 {
	switch js.jointType {
	case DistanceJoint:
		d := &js.distanceJoint
		h = checksumQ(h, d.length)
		h = checksumQ(h, d.hertz)
		h = checksumQ(h, d.dampingRatio)
		h = checksumQ(h, d.minLength)
		h = checksumQ(h, d.maxLength)
		h = checksumQ(h, d.maxMotorForce)
		h = checksumQ(h, d.motorSpeed)
		h = checksumQ(h, d.impulse)
		h = checksumQ(h, d.lowerImpulse)
		h = checksumQ(h, d.upperImpulse)
		h = checksumQ(h, d.motorImpulse)
		h = checksumBool(h, d.enableSpring)
		h = checksumBool(h, d.enableLimit)
		return checksumBool(h, d.enableMotor)
	case FilterJoint:
		return h
	case MotorJoint:
		m := &js.motorJoint
		h = checksumVec2(h, m.linearOffset)
		h = checksumQ(h, m.angularOffset)
		h = checksumVec2(h, m.linearImpulse)
		h = checksumQ(h, m.angularImpulse)
		h = checksumQ(h, m.maxForce)
		h = checksumQ(h, m.maxTorque)
		return checksumQ(h, m.correctionFactor)
	case MouseJoint:
		m := &js.mouseJoint
		h = checksumVec2(h, m.targetA)
		h = checksumQ(h, m.hertz)
		h = checksumQ(h, m.dampingRatio)
		h = checksumQ(h, m.maxForce)
		h = checksumVec2(h, m.linearImpulse)
		return checksumQ(h, m.angularImpulse)
	case PrismaticJoint:
		p := &js.prismaticJoint
		h = checksumVec2(h, p.localAxisA)
		h = checksumVec2(h, p.impulse)
		h = checksumQ(h, p.springImpulse)
		h = checksumQ(h, p.motorImpulse)
		h = checksumQ(h, p.lowerImpulse)
		h = checksumQ(h, p.upperImpulse)
		h = checksumQ(h, p.hertz)
		h = checksumQ(h, p.dampingRatio)
		h = checksumQ(h, p.targetTranslation)
		h = checksumQ(h, p.maxMotorForce)
		h = checksumQ(h, p.motorSpeed)
		h = checksumQ(h, p.referenceAngle)
		h = checksumQ(h, p.lowerTranslation)
		h = checksumQ(h, p.upperTranslation)
		h = checksumBool(h, p.enableSpring)
		h = checksumBool(h, p.enableLimit)
		return checksumBool(h, p.enableMotor)
	case RevoluteJoint:
		r := &js.revoluteJoint
		h = checksumVec2(h, r.linearImpulse)
		h = checksumQ(h, r.springImpulse)
		h = checksumQ(h, r.motorImpulse)
		h = checksumQ(h, r.lowerImpulse)
		h = checksumQ(h, r.upperImpulse)
		h = checksumQ(h, r.hertz)
		h = checksumQ(h, r.dampingRatio)
		h = checksumQ(h, r.targetAngle)
		h = checksumQ(h, r.maxMotorTorque)
		h = checksumQ(h, r.motorSpeed)
		h = checksumQ(h, r.referenceAngle)
		h = checksumQ(h, r.lowerAngle)
		h = checksumQ(h, r.upperAngle)
		h = checksumBool(h, r.enableSpring)
		h = checksumBool(h, r.enableMotor)
		return checksumBool(h, r.enableLimit)
	case WeldJoint:
		wj := &js.weldJoint
		h = checksumQ(h, wj.referenceAngle)
		h = checksumQ(h, wj.linearHertz)
		h = checksumQ(h, wj.linearDampingRatio)
		h = checksumQ(h, wj.angularHertz)
		h = checksumQ(h, wj.angularDampingRatio)
		h = checksumVec2(h, wj.linearImpulse)
		return checksumQ(h, wj.angularImpulse)
	case WheelJoint:
		wh := &js.wheelJoint
		h = checksumVec2(h, wh.localAxisA)
		h = checksumQ(h, wh.perpImpulse)
		h = checksumQ(h, wh.motorImpulse)
		h = checksumQ(h, wh.springImpulse)
		h = checksumQ(h, wh.lowerImpulse)
		h = checksumQ(h, wh.upperImpulse)
		h = checksumQ(h, wh.maxMotorTorque)
		h = checksumQ(h, wh.motorSpeed)
		h = checksumQ(h, wh.lowerTranslation)
		h = checksumQ(h, wh.upperTranslation)
		h = checksumQ(h, wh.hertz)
		h = checksumQ(h, wh.dampingRatio)
		h = checksumBool(h, wh.enableSpring)
		h = checksumBool(h, wh.enableMotor)
		return checksumBool(h, wh.enableLimit)
	default:
		panic("dbox2d: unknown joint type")
	}
}

// checksumJoint identifies each body by its canonical state, as a
// contact does. A joint is directed, so the two bodies keep their order.
func checksumJoint(w *world, j *joint) uint64 {
	js := getJointSim(w, j)
	bodyA := &w.bodies[j.edges[0].bodyId]
	bodyB := &w.bodies[j.edges[1].bodyId]

	h := fnvOffsetBasis
	h = fnvFold(h, checksumSetKind(j.setIndex))
	h = fnvFold(h, uint64(j.jointType))
	h = checksumBool(h, j.collideConnected)
	h = fnvFold(h, checksumBody(w, bodyA))
	h = fnvFold(h, checksumBody(w, bodyB))
	h = checksumVec2(h, js.localOriginAnchorA)
	h = checksumVec2(h, js.localOriginAnchorB)
	h = checksumQ(h, js.constraintHertz)
	h = checksumQ(h, js.constraintDampingRatio)
	h = checksumSoftness(h, js.constraintSoftness)
	return checksumJointData(h, js)
}

// Checksum folds the complete deterministic state of a world into one
// integer. Application data and internal ids are excluded because they do not
// affect simulation. Bodies, shapes, contacts and joints use commutative folds, so
// equivalent worlds do not depend on creation order.
func Checksum(worldId WorldId) uint64 {
	w := getWorldFromId(worldId)

	h := fnvOffsetBasis
	h = fnvFold(h, w.stepIndex)
	h = checksumVec2(h, w.gravity)
	h = checksumQ(h, w.hitEventThreshold)
	h = checksumQ(h, w.restitutionThreshold)
	h = checksumQ(h, w.maxLinearSpeed)
	h = checksumQ(h, w.maxContactPushSpeed)
	h = checksumQ(h, w.contactSpeed)
	h = checksumQ(h, w.contactHertz)
	h = checksumQ(h, w.contactDampingRatio)
	h = checksumQ(h, w.invH)
	h = checksumBool(h, w.enableSleep)
	h = checksumBool(h, w.enableWarmStarting)
	h = checksumBool(h, w.enableContinuous)
	h = checksumBool(h, w.enableSpeculative)

	var bodySum uint64
	bodyCount := 0
	for i := range w.bodies {
		b := &w.bodies[i]
		if b.id == nullIndex {
			continue
		}
		bodySum += checksumBody(w, b)
		bodyCount++
	}
	h = fnvFold(h, uint64(bodyCount))
	h = fnvFold(h, bodySum)

	var contactSum uint64
	contactCount := 0
	for i := range w.contacts {
		c := &w.contacts[i]
		if c.contactId == nullIndex {
			continue
		}
		contactSum += checksumContact(w, c)
		contactCount++
	}
	h = fnvFold(h, uint64(contactCount))
	h = fnvFold(h, contactSum)

	var jointSum uint64
	jointCount := 0
	for i := range w.joints {
		j := &w.joints[i]
		if j.jointId == nullIndex {
			continue
		}
		jointSum += checksumJoint(w, j)
		jointCount++
	}
	h = fnvFold(h, uint64(jointCount))
	return fnvFold(h, jointSum)
}
