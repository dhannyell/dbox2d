package b2

import (
	"fmt"
	"math"
	"os"
	"reflect"
	"sync"
	"unsafe"
)

// This file corresponds to src/recording.c and src/recording.h of Box2D
// v3.2.0. A recording is a header, a snapshot of the world and a stream of
// the calls that changed it. The byte format is the upstream format, so an
// upstream replay viewer opens a recording of the port and the reverse. The
// scalar mode decides the width of a scalar on the wire: float and
// dbox2d_double are the upstream formats; dbox2d_fixed is not, see D-022.

const (
	// recMagic is 'B2RC' in little-endian.
	recMagic = 0x43523242

	recVersionMajor = 3
	recVersionMinor = 13

	recHeaderSize = 32

	// recFixedMarker fills the reserved word of the header in the fixed
	// mode, where the scalar is 8 bytes and no upstream build can read the
	// stream. It is "QFX" followed by zero.
	recFixedMarker = 0x00584651
)

// qSize is the size of a scalar in memory and on the wire.
const qSize = unsafe.Sizeof(Q{})

// Recording is a growable byte buffer that holds a recording. It
// corresponds to b2Recording.
type Recording struct {
	mu          sync.Mutex
	buf         recBuffer
	recordStart int

	accumulatedBounds AABB
	haveBounds        bool
}

// recBuffer is a byte sink. In count-only mode it only counts.
type recBuffer struct {
	data      []byte
	size      int
	countOnly bool
}

func (b *recBuffer) append(p []byte) {
	b.size += len(p)
	if !b.countOnly {
		b.data = append(b.data, p...)
	}
}

func (b *recBuffer) u8(v uint8) {
	b.size++
	if !b.countOnly {
		b.data = append(b.data, v)
	}
}

func (b *recBuffer) u16(v uint16) {
	b.append([]byte{byte(v), byte(v >> 8)})
}

func (b *recBuffer) u32(v uint32) {
	b.append([]byte{byte(v), byte(v >> 8), byte(v >> 16), byte(v >> 24)})
}

func (b *recBuffer) u64(v uint64) {
	b.append([]byte{byte(v), byte(v >> 8), byte(v >> 16), byte(v >> 24),
		byte(v >> 32), byte(v >> 40), byte(v >> 48), byte(v >> 56)})
}

func (b *recBuffer) i32(v int32) { b.u32(uint32(v)) }

func (b *recBuffer) boolean(v bool) {
	if v {
		b.u8(1)
	} else {
		b.u8(0)
	}
}

// mem appends the memory image of n bytes at p. The supported targets are
// little-endian, as the format is.
func (b *recBuffer) mem(p unsafe.Pointer, n uintptr) {
	b.append(unsafe.Slice((*byte)(p), n))
}

func (b *recBuffer) q(v Q) { b.mem(unsafe.Pointer(&v), qSize) }

func (b *recBuffer) vec2(v Vec2) {
	b.q(v.X)
	b.q(v.Y)
}

// pos writes a position, two floats or two doubles.
func (b *recBuffer) pos(v Pos) { b.mem(unsafe.Pointer(&v), unsafe.Sizeof(v)) }

// rot writes the cosine and then the sine, as the C struct holds them.
func (b *recBuffer) rot(v Rot) {
	b.q(v.Cos)
	b.q(v.Sin)
}

func (b *recBuffer) xf(v Transform) {
	b.vec2(v.P)
	b.rot(v.Q)
}

func (b *recBuffer) worldXf(v WorldTransform) {
	b.pos(v.P)
	b.rot(v.Q)
}

func (b *recBuffer) aabb(v AABB) {
	b.vec2(v.LowerBound)
	b.vec2(v.UpperBound)
}

// str writes a string; the wire has no null string on the Go side.
func (b *recBuffer) str(s string) {
	// Go has no null string, so the empty string stands for the null name
	// that the reference writes as 0xFFFF.
	if len(s) == 0 {
		b.u16(0xFFFF)
		return
	}
	n := min(len(s), 65534)
	b.u16(uint16(n))
	if n > 0 {
		b.append([]byte(s[:n]))
	}
}

// patchU24 writes the 24-bit size of a record.
func (b *recBuffer) patchU24(offset int, v int) {
	if b.countOnly {
		return
	}
	b.data[offset] = byte(v)
	b.data[offset+1] = byte(v >> 8)
	b.data[offset+2] = byte(v >> 16)
}

func (b *recBuffer) patchU32(offset int, v uint32) {
	if b.countOnly {
		return
	}
	b.data[offset] = byte(v)
	b.data[offset+1] = byte(v >> 8)
	b.data[offset+2] = byte(v >> 16)
	b.data[offset+3] = byte(v >> 24)
}

// CreateRecording makes an empty recording buffer. The capacity is a hint.
// It corresponds to b2CreateRecording.
func CreateRecording(byteCapacity int) *Recording {
	capacity := 65536
	if byteCapacity > 0 {
		capacity = byteCapacity
	}
	r := &Recording{}
	r.buf.data = make([]byte, 0, capacity)
	return r
}

// Destroy releases the buffer. It corresponds to b2DestroyRecording.
func (r *Recording) Destroy() {
	if r == nil {
		return
	}
	r.buf = recBuffer{}
}

// Data returns the recorded bytes. They stay valid until the next call that
// records or destroys. It corresponds to b2Recording_GetData.
func (r *Recording) Data() []byte { return r.buf.data }

// Size returns the number of recorded bytes. It corresponds to
// b2Recording_GetSize.
func (r *Recording) Size() int { return r.buf.size }

// SaveToFile writes the recording to a file and reports success. It
// corresponds to b2Recording_SaveToFile.
func (r *Recording) SaveToFile(path string) bool {
	if r == nil || path == "" {
		return false
	}
	return os.WriteFile(path, r.buf.data, 0o644) == nil
}

// CreateRecordingFromFile loads a recording file, or returns nil. It
// corresponds to b2CreateRecordingFromFile.
func CreateRecordingFromFile(path string) *Recording {
	if path == "" {
		return nil
	}
	data, err := os.ReadFile(path)
	if err != nil || len(data) < recHeaderSize || len(data) > math.MaxInt32 {
		return nil
	}
	if recLoad32(data) != recMagic {
		return nil
	}
	r := &Recording{}
	r.buf.data = data
	r.buf.size = len(data)
	return r
}

func recLoad32(p []byte) uint32 {
	return uint32(p[0]) | uint32(p[1])<<8 | uint32(p[2])<<16 | uint32(p[3])<<24
}

func recLoad64(p []byte) uint64 {
	return uint64(recLoad32(p)) | uint64(recLoad32(p[4:]))<<32
}

// beginRecord opens a record: the opcode and a 24-bit size to patch.
func (r *Recording) beginRecord(op byte) {
	r.buf.u8(op)
	r.recordStart = r.buf.size
	r.buf.append([]byte{0, 0, 0})
}

func (r *Recording) endRecord() {
	r.buf.patchU24(r.recordStart, r.buf.size-r.recordStart-3)
}

// rec records one call: the opcode and the arguments in the order of
// src/recording_ops.inl.
func (r *Recording) rec(op byte, args ...any) {
	r.mu.Lock()
	r.beginRecord(op)
	for _, a := range args {
		r.buf.arg(a)
	}
	r.endRecord()
	r.mu.Unlock()
}

// recRet records a create call: the arguments and then the returned id.
func (r *Recording) recRet(op byte, id any, args ...any) {
	r.mu.Lock()
	r.beginRecord(op)
	for _, a := range args {
		r.buf.arg(a)
	}
	r.buf.arg(id)
	r.endRecord()
	r.mu.Unlock()
}

// commitRecord appends a finished record, as the query trampolines do.
func (r *Recording) commitRecord(op byte, payload []byte) {
	r.mu.Lock()
	r.buf.u8(op)
	n := len(payload)
	r.buf.append([]byte{byte(n), byte(n >> 8), byte(n >> 16)})
	r.buf.append(payload)
	r.mu.Unlock()
}

func (r *Recording) accumulateBounds(bounds AABB) {
	if r.haveBounds {
		r.accumulatedBounds = AABBUnion(r.accumulatedBounds, bounds)
	} else {
		r.accumulatedBounds = bounds
	}
	r.haveBounds = true
}

// arg writes one argument by its Go type.
func (b *recBuffer) arg(a any) {
	switch v := a.(type) {
	case bool:
		b.boolean(v)
	case int:
		b.i32(int32(v))
	case int32:
		b.i32(v)
	case uint8:
		b.u8(v)
	case uint16:
		b.u16(v)
	case uint32:
		b.u32(v)
	case uint64:
		b.u64(v)
	case Q:
		b.q(v)
	case Vec2:
		b.vec2(v)
	case Rot:
		b.rot(v)
	case Transform:
		b.xf(v)
	case string:
		b.str(v)
	case AABB:
		b.aabb(v)
	case WorldId:
		b.u32(StoreWorldId(v))
	case BodyId:
		b.u64(StoreBodyId(v))
	case ShapeId:
		b.u64(StoreShapeId(v))
	case ChainId:
		b.u64(StoreChainId(v))
	case JointId:
		b.u64(StoreJointId(v))
	case Circle:
		b.raw(&lCircle, unsafe.Pointer(&v))
	case Capsule:
		b.raw(&lCapsule, unsafe.Pointer(&v))
	case Segment:
		b.raw(&lSegment, unsafe.Pointer(&v))
	case Polygon:
		b.raw(&lPolygon, unsafe.Pointer(&v))
	case ChainSegment:
		b.raw(&lChainSegment, unsafe.Pointer(&v))
	case Filter:
		b.filter(v)
	case SurfaceMaterial:
		b.material(v)
	case MassData:
		b.q(v.Mass)
		b.vec2(v.Center)
		b.q(v.RotationalInertia)
	case MotionLocks:
		b.boolean(v.LinearX)
		b.boolean(v.LinearY)
		b.boolean(v.AngularZ)
	case QueryFilter:
		b.u64(v.CategoryBits)
		b.u64(v.MaskBits)
	case ShapeProxy:
		b.shapeProxy(&v)
	case *ShapeProxy:
		b.shapeProxy(v)
	case wirePos:
		b.pos(v.p)
	case wireWorldXf:
		b.worldXf(v.t)
	case ExplosionDef:
		b.explosionDef(&v)
	case BodyDef:
		b.bodyDef(&v)
	case ShapeDef:
		b.shapeDef(&v)
	case ChainDef:
		b.chainDef(&v)
	case DistanceJointDef:
		b.jointBase(&v.Base)
		b.q(v.Length)
		b.boolean(v.EnableSpring)
		b.q(v.LowerSpringForce)
		b.q(v.UpperSpringForce)
		b.q(v.Hertz)
		b.q(v.DampingRatio)
		b.boolean(v.EnableLimit)
		b.q(v.MinLength)
		b.q(v.MaxLength)
		b.boolean(v.EnableMotor)
		b.q(v.MaxMotorForce)
		b.q(v.MotorSpeed)
	case FilterJointDef:
		b.jointBase(&v.Base)
	case MotorJointDef:
		b.jointBase(&v.Base)
		b.vec2(v.LinearVelocity)
		b.q(v.MaxVelocityForce)
		b.q(angleFromTurns(v.AngularVelocity))
		b.q(v.MaxVelocityTorque)
		b.q(v.LinearHertz)
		b.q(v.LinearDampingRatio)
		b.q(v.MaxSpringForce)
		b.q(v.AngularHertz)
		b.q(v.AngularDampingRatio)
		b.q(v.MaxSpringTorque)
	case MoverJointDef:
		b.jointBase(&v.Base)
		b.vec2(v.LinearVelocity)
		b.vec2(v.MaxVelocityForce)
	case PogoJointDef:
		b.jointBase(&v.Base)
		b.vec2(v.Normal)
		b.q(v.Hertz)
		b.q(v.DampingRatio)
		b.q(v.RestLength)
		b.q(v.MaxTensionForce)
		b.q(v.MaxCompressionForce)
		b.q(v.Impulse)
		b.q(v.Velocity)
	case PrismaticJointDef:
		b.jointBase(&v.Base)
		b.boolean(v.EnableSpring)
		b.q(v.Hertz)
		b.q(v.DampingRatio)
		b.q(v.TargetTranslation)
		b.boolean(v.EnableLimit)
		b.q(v.LowerTranslation)
		b.q(v.UpperTranslation)
		b.boolean(v.EnableMotor)
		b.q(v.MaxMotorForce)
		b.q(v.MotorSpeed)
	case RevoluteJointDef:
		b.jointBase(&v.Base)
		b.q(angleFromTurns(v.TargetAngle))
		b.boolean(v.EnableSpring)
		b.q(v.Hertz)
		b.q(v.DampingRatio)
		b.boolean(v.EnableLimit)
		b.q(angleFromTurns(v.LowerAngle))
		b.q(angleFromTurns(v.UpperAngle))
		b.boolean(v.EnableMotor)
		b.q(v.MaxMotorTorque)
		b.q(angleFromTurns(v.MotorSpeed))
	case WeldJointDef:
		b.jointBase(&v.Base)
		b.q(v.LinearHertz)
		b.q(v.AngularHertz)
		b.q(v.LinearDampingRatio)
		b.q(v.AngularDampingRatio)
	case WheelJointDef:
		b.jointBase(&v.Base)
		b.boolean(v.EnableSpring)
		b.q(v.Hertz)
		b.q(v.DampingRatio)
		b.boolean(v.EnableLimit)
		b.q(v.LowerTranslation)
		b.q(v.UpperTranslation)
		b.boolean(v.EnableMotor)
		b.q(v.MaxMotorTorque)
		b.q(angleFromTurns(v.MotorSpeed))
	default:
		panic(fmt.Sprintf("dbox2d: the recording has no encoding for %T", a))
	}
}

// wirePos and wireWorldXf mark an argument that is a position or a world
// transform, so the type switch does not need the alias of Pos.
type wirePos struct{ p Pos }
type wireWorldXf struct{ t WorldTransform }

// raw writes the C image of a struct with no padding.
func (b *recBuffer) raw(l *cLayout, p unsafe.Pointer) {
	var tmp [160]byte
	img := tmp[:l.size]
	l.put(img, p)
	b.append(img)
}

func (b *recBuffer) filter(v Filter) {
	b.u64(v.CategoryBits)
	b.u64(v.MaskBits)
	b.i32(int32(v.GroupIndex))
}

func (b *recBuffer) material(v SurfaceMaterial) {
	b.q(v.Friction)
	b.q(v.Restitution)
	b.q(v.RollingResistance)
	b.q(v.TangentSpeed)
	b.u64(v.UserMaterialId)
	b.u32(v.CustomColor)
}

func (b *recBuffer) shapeProxy(v *ShapeProxy) {
	count := min(max(v.Count, 0), MaxPolygonVertices)
	b.i32(int32(count))
	for i := range count {
		b.vec2(v.Points[i])
	}
	b.q(v.Radius)
}

func (b *recBuffer) explosionDef(v *ExplosionDef) {
	b.u64(v.MaskBits)
	b.pos(v.Position)
	b.q(v.Radius)
	b.q(v.Falloff)
	b.q(v.ImpulsePerLength)
}

func (b *recBuffer) bodyDef(v *BodyDef) {
	b.i32(int32(v.Type))
	b.pos(v.Position)
	b.rot(v.Rotation)
	b.vec2(v.LinearVelocity)
	b.q(angleFromTurns(v.AngularVelocity))
	b.q(v.LinearDamping)
	b.q(v.AngularDamping)
	b.q(v.GravityScale)
	b.q(v.SleepThreshold)
	b.q(v.SafetyFactor)
	b.str(v.Name)
	b.u64(0)
	b.arg(v.MotionLocks)
	b.boolean(v.EnableSleep)
	b.boolean(v.IsAwake)
	b.boolean(v.IsBullet)
	b.boolean(v.IsEnabled)
	b.boolean(v.AllowFastRotation)
	b.boolean(v.EnableContactRecycling)
}

func (b *recBuffer) shapeDef(v *ShapeDef) {
	b.u64(0)
	b.material(v.Material)
	b.q(v.Density)
	b.filter(v.Filter)
	b.boolean(v.EnableCustomFiltering)
	b.boolean(v.IsSensor)
	b.boolean(v.EnableSensorEvents)
	b.boolean(v.EnableContactEvents)
	b.boolean(v.EnableHitEvents)
	b.boolean(v.EnablePreSolveEvents)
	b.boolean(v.InvokeContactCreation)
	b.boolean(v.UpdateBodyMass)
}

func (b *recBuffer) chainDef(v *ChainDef) {
	b.u64(0)
	b.i32(int32(len(v.Points)))
	for _, p := range v.Points {
		b.vec2(p)
	}
	b.vec2(v.Ghost1)
	b.vec2(v.Ghost2)
	b.i32(int32(len(v.Materials)))
	for _, m := range v.Materials {
		b.material(m)
	}
	b.filter(v.Filter)
	b.boolean(v.IsLoop)
	b.boolean(v.EnableSensorEvents)
}

func (b *recBuffer) jointBase(v *JointDef) {
	b.u64(0)
	b.u64(StoreBodyId(v.BodyIdA))
	b.u64(StoreBodyId(v.BodyIdB))
	b.xf(v.LocalFrameA)
	b.xf(v.LocalFrameB)
	b.q(v.ForceThreshold)
	b.q(v.TorqueThreshold)
	b.q(v.ConstraintHertz)
	b.q(v.ConstraintDampingRatio)
	b.q(v.DrawScale)
	b.boolean(v.CollideConnected)
}

// Raw layouts of the geometry the stream copies as memory.
var (
	lCircle       = layoutOfType(typeOf[Circle]())
	lCapsule      = layoutOfType(typeOf[Capsule]())
	lSegment      = layoutOfType(typeOf[Segment]())
	lPolygon      = layoutOfType(typeOf[Polygon]())
	lChainSegment = layoutOfType(typeOf[ChainSegment]())
)

// StartRecording starts recording the world into the buffer. The world must
// be at a step boundary, and a world with a pre-solve or custom filter
// callback cannot be recorded because replay cannot reproduce them. It
// corresponds to b2World_StartRecording.
func (worldId WorldId) StartRecording(recording *Recording) {
	w := getWorldFromId(worldId)
	if w.locked || recording == nil || w.recording != nil {
		return
	}
	if w.preSolveFcn != nil || w.preContinuousFcn != nil || w.customFilterFcn != nil {
		fmt.Println("b2World_StartRecording: preSolve and custom filtering are not supported when recording")
		return
	}
	startRecordingIntoBuffer(w, recording)
}

// StopRecording ends the recording of the world. The buffer then holds a
// self-contained recording. It corresponds to b2World_StopRecording.
func (worldId WorldId) StopRecording() {
	w := getWorldFromId(worldId)
	if w.locked {
		return
	}
	stopRecordingInternal(w)
}

func worldIdOf(w *world) WorldId {
	return WorldId{index1: w.worldId + 1, generation: w.generation}
}

func startRecordingIntoBuffer(w *world, rec *Recording) {
	rec.buf.data = rec.buf.data[:0]
	rec.buf.size = 0
	rec.recordStart = 0
	rec.haveBounds = false

	var blob recBuffer
	serializeWorld(w, &blob)

	// b2RecHeader, 32 bytes.
	rec.buf.u32(recMagic)
	rec.buf.u16(recVersionMajor)
	rec.buf.u16(recVersionMinor)
	reserved2 := uint32(0)
	if ScalarMode == "fixed" {
		reserved2 = recFixedMarker
	}
	rec.buf.u32(reserved2)
	rec.buf.u32(math.Float32bits(1)) // lengthScale; the port has no length unit, D-002
	rec.buf.u8(0)                    // reserved3
	rec.buf.u8(8)                    // pointerWidth
	rec.buf.u8(0)                    // bigEndian
	rec.buf.u8(0)                    // validationEnabled
	rec.buf.u32(0)                   // reserved1
	rec.buf.u64(uint64(blob.size))
	rec.buf.append(blob.data)

	w.recording = rec

	if seed, ok := computeWorldBounds(w); ok {
		rec.accumulateBounds(seed)
	}

	rec.rec(opStateHash, worldIdOf(w), hashWorldState(w))
}

func stopRecordingInternal(w *world) {
	rec := w.recording
	if rec == nil {
		return
	}
	w.recording = nil

	var bounds AABB
	if rec.haveBounds {
		bounds = rec.accumulatedBounds
	}
	rec.rec(opRecordingBounds, bounds)
	rec.rec(opDestroyWorld, worldIdOf(w))
}

// hashWorldState hashes the transforms and the velocities of the bodies. It
// corresponds to b2HashWorldState.
func hashWorldState(w *world) uint64 {
	hash := fnvOffsetBasis
	for i := range w.bodies {
		b := &w.bodies[i]
		if b.id != i {
			continue
		}
		sim := getBodySim(w, b)
		hash = fnvMixPos(hash, sim.transform.P)
		hash = fnvMixQ(hash, sim.transform.Q.Cos)
		hash = fnvMixQ(hash, sim.transform.Q.Sin)
		if state := getBodyState(w, b); state != nil {
			hash = fnvMixQ(hash, state.linearVelocity.X)
			hash = fnvMixQ(hash, state.linearVelocity.Y)
			hash = fnvMixQ(hash, state.angularVelocity)
		}
	}
	return hash
}

// The snapshot hash is FNV-1a over bytes, one multiplication per byte of the
// input word for the byte forms and one per word for the word forms, as the
// reference does.
func fnvMixWord(hash, word uint64) uint64 { return (hash ^ word) * fnvPrime }

func fnvMixQ(hash uint64, q Q) uint64 { return fnvMixWord(hash, qBits(q)) }

func fnvMixInt(hash uint64, v int) uint64 { return fnvMixWord(hash, uint64(uint32(int32(v)))) }

func typeOf[T any]() reflect.Type { return reflect.TypeOf((*T)(nil)).Elem() }

// posBits returns the raw words of a position: doubles under dbox2d_double,
// otherwise the scalar bits.
func posBits(p Pos) (uint64, uint64) {
	if IsDoublePrecision() {
		w := (*[2]uint64)(unsafe.Pointer(&p))
		return w[0], w[1]
	}
	v := (*Vec2)(unsafe.Pointer(&p))
	return qBits(v.X), qBits(v.Y)
}

func fnvMixBytes(hash uint64, p unsafe.Pointer, n uintptr) uint64 {
	for _, c := range unsafe.Slice((*byte)(p), n) {
		hash = (hash ^ uint64(c)) * fnvPrime
	}
	return hash
}

func fnvMixPos(hash uint64, p Pos) uint64 {
	x, y := posBits(p)
	hash = fnvMixWord(hash, x)
	return fnvMixWord(hash, y)
}

// recordStepEnd writes the per-step state hash while the world is still
// locked, then grows the recorded bounds so a replay can frame the motion.
func recordStepEnd(w *world) {
	rec := w.recording
	if rec == nil {
		return
	}
	rec.rec(opStateHash, worldIdOf(w), hashWorldState(w))
	if bounds, ok := computeWorldBounds(w); ok {
		rec.accumulateBounds(bounds)
	}
}

// updateMaxCapacity tracks the high water marks that the snapshot carries.
// It corresponds to the b2Capacity block of b2World_Step.
func updateMaxCapacity(w *world) {
	c := &w.maxCapacity
	c.staticShapeCount = max(c.staticShapeCount, w.broadPhase.trees[StaticBody].proxyCount)
	c.dynamicShapeCount = max(c.dynamicShapeCount, w.broadPhase.trees[DynamicBody].proxyCount)
	staticBodyCount := len(w.solverSets[staticSet].bodySims)
	c.staticBodyCount = max(c.staticBodyCount, staticBodyCount)
	c.dynamicBodyCount = max(c.dynamicBodyCount, w.bodyIdPool.idCount()-staticBodyCount)
	c.contactCount = max(c.contactCount, w.contactIdPool.idCount())
}

// recJoint records a typed joint accessor. The port folds the accessors that
// several joint types share into one method (D-003), so the op depends on the
// joint type. An accessor the type does not have is not recorded. Angles
// arrive in the unit of the joint sim and are written as they are (D-025).
func (r *Recording) recJoint(jointType JointType, kind string, jointId JointId, args ...any) {
	op, ok := jointOps[jointOpKey{jointType, kind}]
	if !ok {
		return
	}
	r.rec(op, append([]any{jointId}, args...)...)
}

// recordCreateShape records a shape creation with the returned id.
func recordCreateShape(w *world, shapeType ShapeType, id ShapeId, bodyId BodyId, def *ShapeDef, geometry any) {
	var op byte
	var value any
	switch g := geometry.(type) {
	case *Circle:
		op, value = opCreateCircleShape, *g
	case *Capsule:
		op, value = opCreateCapsuleShape, *g
	case *Segment:
		op, value = opCreateSegmentShape, *g
	case *Polygon:
		op, value = opCreatePolygonShape, *g
	case *ChainSegment:
		op, value = opCreateChainSegmentShape, *g
	default:
		return
	}
	w.recording.recRet(op, id, bodyId, *def, value)
}
