package b2

import (
	"unsafe"
)

// This file corresponds to the reader half of src/replay.c of Box2D v3.2.0:
// the b2RecR_ primitives and the def readers. A read past the end of the
// data latches ok to false and returns zero values, so a truncated file stops
// the replay and never reads garbage.

// recRecordedHit is one hit of a recorded query. It corresponds to
// b2RecRecordedHit.
type recRecordedHit struct {
	id          ShapeId
	point       Pos
	normal      Vec2
	fraction    Q
	plane       PlaneResult
	userReturnF Q
	userReturnB bool
}

// recReader is the cursor over the op stream and the state that the
// dispatchers share. It corresponds to b2RecReader.
type recReader struct {
	data   []byte
	cursor int

	// worldId is the world that the replay created.
	worldId WorldId

	// ok turns false on a read overrun or an id mismatch, a fatal stop.
	ok bool

	// diverged turns true when a state hash or a query result does not
	// reproduce. It is not fatal, so a viewer can keep playing.
	diverged bool

	// hits is the scratch for the hits of one query.
	hits []recRecordedHit

	owner *Replay
}

func (rd *recReader) check(n int) {
	if n < 0 || int64(rd.cursor)+int64(n) > int64(len(rd.data)) {
		rd.ok = false
	}
}

// take returns the next n bytes, or nil after latching the failure.
func (rd *recReader) take(n int) []byte {
	rd.check(n)
	if !rd.ok {
		return nil
	}
	p := rd.data[rd.cursor : rd.cursor+n]
	rd.cursor += n
	return p
}

func (rd *recReader) u8() uint8 {
	p := rd.take(1)
	if p == nil {
		return 0
	}
	return p[0]
}

func (rd *recReader) u16() uint16 {
	p := rd.take(2)
	if p == nil {
		return 0
	}
	return uint16(p[0]) | uint16(p[1])<<8
}

func (rd *recReader) u24() uint32 {
	p := rd.take(3)
	if p == nil {
		return 0
	}
	return uint32(p[0]) | uint32(p[1])<<8 | uint32(p[2])<<16
}

func (rd *recReader) u32() uint32 {
	p := rd.take(4)
	if p == nil {
		return 0
	}
	return recLoad32(p)
}

func (rd *recReader) u64() uint64 {
	p := rd.take(8)
	if p == nil {
		return 0
	}
	return recLoad64(p)
}

func (rd *recReader) i32() int32 { return int32(rd.u32()) }

func (rd *recReader) boolean() bool { return rd.u8() != 0 }

// mem fills the n bytes at p from the stream, or zeroes them.
func (rd *recReader) mem(p unsafe.Pointer, n uintptr) {
	dst := unsafe.Slice((*byte)(p), n)
	src := rd.take(int(n))
	if src == nil {
		clear(dst)
		return
	}
	copy(dst, src)
}

func (rd *recReader) q() Q {
	var v Q
	rd.mem(unsafe.Pointer(&v), qSize)
	return v
}

func (rd *recReader) vec2() Vec2 {
	x := rd.q()
	y := rd.q()
	return Vec2{X: x, Y: y}
}

func (rd *recReader) pos() Pos {
	var v Pos
	rd.mem(unsafe.Pointer(&v), unsafe.Sizeof(v))
	return v
}

func (rd *recReader) rot() Rot {
	c := rd.q()
	s := rd.q()
	return Rot{Cos: c, Sin: s}
}

func (rd *recReader) xf() Transform {
	p := rd.vec2()
	q := rd.rot()
	return Transform{P: p, Q: q}
}

func (rd *recReader) worldXf() WorldTransform {
	var t WorldTransform
	t.P = rd.pos()
	t.Q = rd.rot()
	return t
}

func (rd *recReader) aabb() AABB {
	lo := rd.vec2()
	hi := rd.vec2()
	return AABB{LowerBound: lo, UpperBound: hi}
}

func (rd *recReader) worldIdRaw() WorldId { return LoadWorldId(rd.u32()) }
func (rd *recReader) bodyIdRaw() BodyId   { return LoadBodyId(rd.u64()) }
func (rd *recReader) shapeIdRaw() ShapeId { return LoadShapeId(rd.u64()) }
func (rd *recReader) chainIdRaw() ChainId { return LoadChainId(rd.u64()) }
func (rd *recReader) jointIdRaw() JointId { return LoadJointId(rd.u64()) }
func (rd *recReader) replaySlot() uint16  { return rd.worldId.index1 - 1 }
func (rd *recReader) bodyId() BodyId      { return rd.makeBody(rd.bodyIdRaw()) }
func (rd *recReader) shapeId() ShapeId    { return rd.makeShape(rd.shapeIdRaw()) }
func (rd *recReader) chainId() ChainId    { return rd.makeChain(rd.chainIdRaw()) }
func (rd *recReader) jointId() JointId    { return rd.makeJoint(rd.jointIdRaw()) }
func (rd *recReader) makeBody(id BodyId) BodyId {
	id.world0 = rd.replaySlot()
	return id
}

// The ids of a recording carry the world slot of the recording world. The
// replay world has another slot, so every id is remapped before the call.
func (rd *recReader) makeShape(id ShapeId) ShapeId {
	id.world0 = rd.replaySlot()
	return id
}

func (rd *recReader) makeChain(id ChainId) ChainId {
	id.world0 = rd.replaySlot()
	return id
}

func (rd *recReader) makeJoint(id JointId) JointId {
	id.world0 = rd.replaySlot()
	return id
}

// blob reads the C image of a struct that the stream copies as memory.
func (rd *recReader) blob(l *cLayout, p unsafe.Pointer) {
	src := rd.take(l.size)
	if src == nil {
		return
	}
	l.get(src, p)
}

func (rd *recReader) circle() Circle {
	var v Circle
	rd.blob(&lCircle, unsafe.Pointer(&v))
	return v
}

func (rd *recReader) capsule() Capsule {
	var v Capsule
	rd.blob(&lCapsule, unsafe.Pointer(&v))
	return v
}

func (rd *recReader) segment() Segment {
	var v Segment
	rd.blob(&lSegment, unsafe.Pointer(&v))
	return v
}

func (rd *recReader) polygon() Polygon {
	var v Polygon
	rd.blob(&lPolygon, unsafe.Pointer(&v))
	return v
}

func (rd *recReader) chainSeg() ChainSegment {
	var v ChainSegment
	rd.blob(&lChainSegment, unsafe.Pointer(&v))
	return v
}

func (rd *recReader) filter() Filter {
	var f Filter
	f.CategoryBits = rd.u64()
	f.MaskBits = rd.u64()
	f.GroupIndex = int(rd.i32())
	return f
}

func (rd *recReader) material() SurfaceMaterial {
	m := DefaultSurfaceMaterial()
	m.Friction = rd.q()
	m.Restitution = rd.q()
	m.RollingResistance = rd.q()
	m.TangentSpeed = rd.q()
	m.UserMaterialId = rd.u64()
	m.CustomColor = rd.u32()
	return m
}

func (rd *recReader) massData() MassData {
	var m MassData
	m.Mass = rd.q()
	m.Center = rd.vec2()
	m.RotationalInertia = rd.q()
	return m
}

func (rd *recReader) locks() MotionLocks {
	var l MotionLocks
	l.LinearX = rd.boolean()
	l.LinearY = rd.boolean()
	l.AngularZ = rd.boolean()
	return l
}

// str reads a string. The reference writes a null name as 0xFFFF and clamps
// a long name, skipping its full length so the cursor stays aligned.
func (rd *recReader) str() string {
	n := int(rd.u16())
	if n == 0xFFFF {
		return ""
	}
	p := rd.take(n)
	if p == nil {
		return ""
	}
	return string(p[:min(n, nameLength)])
}

func (rd *recReader) bodyDef() BodyDef {
	def := DefaultBodyDef()
	def.Type = BodyType(rd.i32())
	def.Position = rd.pos()
	def.Rotation = rd.rot()
	def.LinearVelocity = rd.vec2()
	def.AngularVelocity = angleToTurns(rd.q())
	def.LinearDamping = rd.q()
	def.AngularDamping = rd.q()
	def.GravityScale = rd.q()
	def.SleepThreshold = rd.q()
	def.SafetyFactor = rd.q()
	def.Name = rd.str()
	rd.u64() // userData is not preserved
	def.MotionLocks = rd.locks()
	def.EnableSleep = rd.boolean()
	def.IsAwake = rd.boolean()
	def.IsBullet = rd.boolean()
	def.IsEnabled = rd.boolean()
	def.AllowFastRotation = rd.boolean()
	def.EnableContactRecycling = rd.boolean()
	def.UserData = nil
	return def
}

func (rd *recReader) shapeDef() ShapeDef {
	def := DefaultShapeDef()
	rd.u64() // userData is not preserved
	def.Material = rd.material()
	def.Density = rd.q()
	def.Filter = rd.filter()
	def.EnableCustomFiltering = rd.boolean()
	def.IsSensor = rd.boolean()
	def.EnableSensorEvents = rd.boolean()
	def.EnableContactEvents = rd.boolean()
	def.EnableHitEvents = rd.boolean()
	def.EnablePreSolveEvents = rd.boolean()
	def.InvokeContactCreation = rd.boolean()
	def.UpdateBodyMass = rd.boolean()
	def.UserData = nil
	return def
}

// count reads an element count taken from an untrusted file. Every element
// takes at least one byte, so a valid count never exceeds the bytes left.
func (rd *recReader) count() int {
	n := int(rd.i32())
	if n < 0 {
		return 0
	}
	if n > len(rd.data)-rd.cursor {
		rd.ok = false
		return 0
	}
	return n
}

func (rd *recReader) chainDef() ChainDef {
	def := DefaultChainDef()
	rd.u64() // userData is not preserved
	n := rd.count()
	def.Points = nil
	if n > 0 {
		def.Points = make([]Vec2, n)
		for i := range def.Points {
			def.Points[i] = rd.vec2()
		}
	}
	def.Ghost1 = rd.vec2()
	def.Ghost2 = rd.vec2()
	m := rd.count()
	def.Materials = nil
	if m > 0 {
		def.Materials = make([]SurfaceMaterial, m)
		for i := range def.Materials {
			def.Materials[i] = rd.material()
		}
	}
	def.Filter = rd.filter()
	def.IsLoop = rd.boolean()
	def.EnableSensorEvents = rd.boolean()
	def.UserData = nil
	return def
}

func (rd *recReader) explosionDef() ExplosionDef {
	def := DefaultExplosionDef()
	def.MaskBits = rd.u64()
	def.Position = rd.pos()
	def.Radius = rd.q()
	def.Falloff = rd.q()
	def.ImpulsePerLength = rd.q()
	return def
}

// jointBase reads the base of a joint def. The body ids keep the recorded
// world slot; the create dispatcher remaps them.
func (rd *recReader) jointBase(base *JointDef) {
	rd.u64() // userData is not preserved
	base.BodyIdA = rd.bodyIdRaw()
	base.BodyIdB = rd.bodyIdRaw()
	base.LocalFrameA = rd.xf()
	base.LocalFrameB = rd.xf()
	base.ForceThreshold = rd.q()
	base.TorqueThreshold = rd.q()
	base.ConstraintHertz = rd.q()
	base.ConstraintDampingRatio = rd.q()
	base.DrawScale = rd.q()
	base.CollideConnected = rd.boolean()
	base.UserData = nil
}

func (rd *recReader) distanceJointDef() DistanceJointDef {
	def := DefaultDistanceJointDef()
	rd.jointBase(&def.Base)
	def.Length = rd.q()
	def.EnableSpring = rd.boolean()
	def.LowerSpringForce = rd.q()
	def.UpperSpringForce = rd.q()
	def.Hertz = rd.q()
	def.DampingRatio = rd.q()
	def.EnableLimit = rd.boolean()
	def.MinLength = rd.q()
	def.MaxLength = rd.q()
	def.EnableMotor = rd.boolean()
	def.MaxMotorForce = rd.q()
	def.MotorSpeed = rd.q()
	return def
}

func (rd *recReader) filterJointDef() FilterJointDef {
	def := DefaultFilterJointDef()
	rd.jointBase(&def.Base)
	return def
}

func (rd *recReader) motorJointDef() MotorJointDef {
	def := DefaultMotorJointDef()
	rd.jointBase(&def.Base)
	def.LinearVelocity = rd.vec2()
	def.MaxVelocityForce = rd.q()
	def.AngularVelocity = angleToTurns(rd.q())
	def.MaxVelocityTorque = rd.q()
	def.LinearHertz = rd.q()
	def.LinearDampingRatio = rd.q()
	def.MaxSpringForce = rd.q()
	def.AngularHertz = rd.q()
	def.AngularDampingRatio = rd.q()
	def.MaxSpringTorque = rd.q()
	return def
}

func (rd *recReader) moverJointDef() MoverJointDef {
	def := DefaultMoverJointDef()
	rd.jointBase(&def.Base)
	def.LinearVelocity = rd.vec2()
	def.MaxVelocityForce = rd.vec2()
	return def
}

func (rd *recReader) pogoJointDef() PogoJointDef {
	def := DefaultPogoJointDef()
	rd.jointBase(&def.Base)
	def.Normal = rd.vec2()
	def.Hertz = rd.q()
	def.DampingRatio = rd.q()
	def.RestLength = rd.q()
	def.MaxTensionForce = rd.q()
	def.MaxCompressionForce = rd.q()
	def.Impulse = rd.q()
	def.Velocity = rd.q()
	return def
}

func (rd *recReader) prismaticJointDef() PrismaticJointDef {
	def := DefaultPrismaticJointDef()
	rd.jointBase(&def.Base)
	def.EnableSpring = rd.boolean()
	def.Hertz = rd.q()
	def.DampingRatio = rd.q()
	def.TargetTranslation = rd.q()
	def.EnableLimit = rd.boolean()
	def.LowerTranslation = rd.q()
	def.UpperTranslation = rd.q()
	def.EnableMotor = rd.boolean()
	def.MaxMotorForce = rd.q()
	def.MotorSpeed = rd.q()
	return def
}

func (rd *recReader) revoluteJointDef() RevoluteJointDef {
	def := DefaultRevoluteJointDef()
	rd.jointBase(&def.Base)
	def.TargetAngle = angleToTurns(rd.q())
	def.EnableSpring = rd.boolean()
	def.Hertz = rd.q()
	def.DampingRatio = rd.q()
	def.EnableLimit = rd.boolean()
	def.LowerAngle = angleToTurns(rd.q())
	def.UpperAngle = angleToTurns(rd.q())
	def.EnableMotor = rd.boolean()
	def.MaxMotorTorque = rd.q()
	def.MotorSpeed = angleToTurns(rd.q())
	return def
}

func (rd *recReader) weldJointDef() WeldJointDef {
	def := DefaultWeldJointDef()
	rd.jointBase(&def.Base)
	def.LinearHertz = rd.q()
	def.AngularHertz = rd.q()
	def.LinearDampingRatio = rd.q()
	def.AngularDampingRatio = rd.q()
	return def
}

func (rd *recReader) wheelJointDef() WheelJointDef {
	def := DefaultWheelJointDef()
	rd.jointBase(&def.Base)
	def.EnableSpring = rd.boolean()
	def.Hertz = rd.q()
	def.DampingRatio = rd.q()
	def.EnableLimit = rd.boolean()
	def.LowerTranslation = rd.q()
	def.UpperTranslation = rd.q()
	def.EnableMotor = rd.boolean()
	def.MaxMotorTorque = rd.q()
	def.MotorSpeed = angleToTurns(rd.q())
	return def
}

func (rd *recReader) queryFilter() QueryFilter {
	var f QueryFilter
	f.CategoryBits = rd.u64()
	f.MaskBits = rd.u64()
	return f
}

func (rd *recReader) shapeProxy() ShapeProxy {
	var p ShapeProxy
	count := int(rd.i32())
	count = min(max(count, 0), MaxPolygonVertices)
	p.Count = count
	for i := range count {
		p.Points[i] = rd.vec2()
	}
	p.Radius = rd.q()
	return p
}

func (rd *recReader) rayResult() RayResult {
	var v RayResult
	v.ShapeId = rd.shapeIdRaw()
	v.Point = rd.pos()
	v.Normal = rd.vec2()
	v.Fraction = rd.q()
	v.NodeVisits = int(rd.i32())
	v.LeafVisits = int(rd.i32())
	v.Hit = rd.boolean()
	return v
}

func (rd *recReader) planeResult() PlaneResult {
	var v PlaneResult
	v.Plane.Normal = rd.vec2()
	v.Plane.Offset = rd.q()
	v.Point = rd.vec2()
	v.Hit = rd.boolean()
	return v
}

func (rd *recReader) treeStats() TreeStats {
	var v TreeStats
	v.NodeVisits = int(rd.i32())
	v.LeafVisits = int(rd.i32())
	return v
}

// ensureHits sizes the hit scratch for n hits, refusing a count that the
// remaining bytes cannot hold.
func (rd *recReader) ensureHits(n int) {
	if n < 0 || n > len(rd.data)-rd.cursor {
		rd.ok = false
		return
	}
	if cap(rd.hits) < n {
		rd.hits = make([]recRecordedHit, n)
	}
	rd.hits = rd.hits[:n]
}
