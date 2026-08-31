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

func checksumManifold(h uint64, m *Manifold) uint64 {
	h = checksumVec2(h, m.Normal)
	h = checksumQ(h, m.RollingImpulse)
	h = fnvFold(h, uint64(m.PointCount))
	for i := range m.PointCount {
		p := &m.Points[i]
		h = checksumVec2(h, p.Point)
		h = checksumVec2(h, p.AnchorA)
		h = checksumVec2(h, p.AnchorB)
		h = checksumQ(h, p.Separation)
		h = checksumQ(h, p.NormalImpulse)
		h = checksumQ(h, p.TangentImpulse)
		h = checksumQ(h, p.TotalNormalImpulse)
		h = checksumQ(h, p.NormalVelocity)
		h = fnvFold(h, uint64(p.Id))
		h = checksumBool(h, p.Persisted)
	}
	return h
}

func checksumContact(w *world, c *contact) uint64 {
	cs := getContactSim(w, c)

	h := fnvOffsetBasis
	h = fnvFold(h, checksumSetKind(c.setIndex))
	h = fnvFold(h, uint64(int64(c.shapeIdA)))
	h = fnvFold(h, uint64(int64(c.shapeIdB)))
	h = fnvFold(h, uint64(c.flags))
	for i := range c.edges {
		h = fnvFold(h, uint64(int64(c.edges[i].bodyId)))
		h = fnvFold(h, uint64(int64(c.edges[i].prevKey)))
		h = fnvFold(h, uint64(int64(c.edges[i].nextKey)))
	}

	h = checksumQ(h, cs.friction)
	h = checksumQ(h, cs.restitution)
	h = checksumQ(h, cs.rollingResistance)
	h = checksumQ(h, cs.tangentSpeed)
	h = fnvFold(h, uint64(cs.simFlags))
	return checksumManifold(h, &cs.manifold)
}

// Checksum folds the complete deterministic state of a world into one
// integer. Application data and internal ids are excluded because they do not
// affect simulation. Bodies, shapes and contacts use commutative folds, so
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
	return fnvFold(h, contactSum)
}
