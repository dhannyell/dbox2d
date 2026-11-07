package b2

import (
	"fmt"
	"reflect"
	"slices"
	"unsafe"
)

// This file corresponds to src/snapshot.c of Box2D v3.2.0. A snapshot is the
// complete simulation state of a world in one byte image: restoring it and
// stepping continues bit for bit. The image is the upstream image. The Go
// structs differ from the C structs in field order, in the width of int and
// in unions, so each struct is written through a layout plan from
// clayout.go. See D-022.

const (
	// snapMagic is 'BNS2' in little-endian.
	snapMagic   uint32 = 0x32534E42
	snapVersion uint32 = 13

	snapFlagValidation uint32 = 0x1
	snapFlagDouble     uint32 = 0x2

	// snapFlagFixed marks an image of the fixed scalar mode. No upstream
	// build writes or reads it, see D-022.
	snapFlagFixed uint32 = 0x4

	snapHeaderSize = 16
)

// capacity corresponds to b2Capacity. The snapshot carries it.
type capacity struct {
	staticShapeCount  int
	dynamicShapeCount int
	staticBodyCount   int
	dynamicBodyCount  int
	contactCount      int
	arenaByteCount    int
}

// The layout plans, one per serialized struct. They are built once.
var (
	plBody       cLayout
	plBodySim    cLayout
	plBodyState  cLayout
	plShape      [ShapeTypeCount]cLayout
	plContact    cLayout
	plContactSim cLayout
	plJoint      cLayout
	plJointSim   [WheelJoint + 1]cLayout
	plIslandSim  cLayout
	plContactLnk cLayout
	plJointLnk   cLayout
	plShapeRef   cLayout
	plTreeNode   cLayout
	plTreeProxy  cLayout
	plAABB       cLayout
)

// unionOf returns the size and alignment of a C union. The float and double
// builds use the upstream constants. A build with a wider Q (fixed, D-023)
// has wider members, so the union is the largest member.
func unionOf(size, align int, members []reflect.Type) (int, int) {
	if qSize == 4 {
		return size, align
	}
	size, align = 0, 1
	for _, t := range members {
		l := layoutOfType(t)
		size, align = max(size, l.size), max(align, l.align)
	}
	return alignUp(size, align), align
}

func init() {
	plBody = orderedLayout(typeOf[body](),
		cPtr(), cF("setIndex"), cF("localIndex"), cF("headContactKey"), cF("contactCount"),
		cF("headShapeId"), cF("shapeCount"), cF("headChainId"), cF("headJointKey"), cF("jointCount"),
		cF("islandId"), cF("islandIndex"), cF("mass"), cF("inertia"), cF("sleepThreshold"),
		cF("sleepTime"), cF("safetyFactor"), cF("bodyMoveIndex"), cF("id"), cF("flags"),
		cF("bodyType"), cF("generation"), cF("name"))
	plBodySim = layoutOfType(typeOf[bodySim]())
	plBodyState = layoutOfType(typeOf[bodyState]())

	geometry := [ShapeTypeCount]reflect.Type{
		CircleShape:       typeOf[Circle](),
		CapsuleShape:      typeOf[Capsule](),
		SegmentShape:      typeOf[Segment](),
		PolygonShape:      typeOf[Polygon](),
		ChainSegmentShape: typeOf[ChainSegment](),
	}
	geomSize, geomAlign := unionOf(144, 4, geometry[:])
	for i := range plShape {
		plShape[i] = orderedLayout(typeOf[shape](),
			cF("id"), cF("bodyId"), cF("prevShapeId"), cF("nextShapeId"), cF("sensorIndex"), cF("shapeType"),
			cG(cF("friction"), cF("restitution"), cF("rollingResistance"), cF("tangentSpeed"),
				cF("userMaterialId"), cF("customColor")),
			cF("density"), cF("aabbMargin"), cF("aabb"), cF("localCentroid"), cF("proxyKey"),
			cF("filter"), cPtr(),
			cV("geometry", geometry[i], geomSize, geomAlign),
			cF("generation"), cF("enableSensorEvents"), cF("enableContactEvents"),
			cF("enableCustomFiltering"), cF("enableHitEvents"), cF("enablePreSolveEvents"))
	}

	plContact = orderedLayout(typeOf[contact](),
		cF("edges"), cF("islandId"), cF("islandIndex"), cF("setIndex"), cF("colorIndex"),
		cF("localIndex"), cF("shapeIdA"), cF("shapeIdB"), cF("contactId"), cF("flags"), cF("generation"))
	plContactSim = orderedLayout(typeOf[contactSim](),
		cF("contactId"), cF("cachedRotationA"), cF("cachedRotationB"), cF("cachedRelativePose"),
		cF("bodySimIndexA"), cF("bodySimIndexB"), cF("simFlags"), cF("shapeIdA"), cF("shapeIdB"),
		cF("invMassA"), cF("invIA"), cF("invMassB"), cF("invIB"), cF("manifold"),
		cF("friction"), cF("restitution"), cF("rollingResistance"), cF("tangentSpeed"), cF("cache"))
	plJoint = orderedLayout(typeOf[joint](),
		cPtr(), cF("setIndex"), cF("colorIndex"), cF("localIndex"), cF("edges"), cF("jointId"),
		cF("islandId"), cF("islandIndex"), cF("drawScale"), cF("jointType"), cF("generation"),
		cF("collideConnected"))

	payloads := [WheelJoint + 1]reflect.Type{
		DistanceJoint:  typeOf[distanceJoint](),
		FilterJoint:    typeOf[struct{}](),
		MotorJoint:     typeOf[motorJoint](),
		MoverJoint:     typeOf[moverJoint](),
		PogoJoint:      typeOf[pogoJoint](),
		PrismaticJoint: typeOf[prismaticJoint](),
		RevoluteJoint:  typeOf[revoluteJoint](),
		WeldJoint:      typeOf[weldJoint](),
		WheelJoint:     typeOf[wheelJoint](),
	}
	payloadSize, payloadAlign := unionOf(160, 4, payloads[:])
	for i := range plJointSim {
		plJointSim[i] = orderedLayout(typeOf[jointSim](),
			cF("jointId"), cF("bodyIdA"), cF("bodyIdB"), cF("jointType"), cF("localFrameA"),
			cF("localFrameB"), cF("invMassA"), cF("invMassB"), cF("invIA"), cF("invIB"),
			cF("constraintHertz"), cF("constraintDampingRatio"), cF("constraintSoftness"),
			cF("forceThreshold"), cF("torqueThreshold"),
			cV("payload", payloads[i], payloadSize, payloadAlign))
	}

	offContactSimBodyA = fieldOffset(&plContactSim, typeOf[contactSim](), "bodySimIndexA")
	offContactSimBodyB = fieldOffset(&plContactSim, typeOf[contactSim](), "bodySimIndexB")

	plIslandSim = layoutOfType(typeOf[islandSim]())
	plContactLnk = layoutOfType(typeOf[contactLink]())
	plJointLnk = layoutOfType(typeOf[jointLink]())
	plShapeRef = layoutOfType(typeOf[shapeRef]())
	plTreeNode = orderedLayout(typeOf[treeNode](), cF("aabb"), cPad(8), cF("flagIndex"), cF("aux"))
	plTreeProxy = layoutOfType(typeOf[treeProxy]())
	plAABB = layoutOfType(typeOf[AABB]())
}

// snapLayoutHash corresponds to b2ComputeLayoutHash. The sizes are those of
// the plans, which equal the C sizes in the float and double modes.
func snapLayoutHash() uint32 {
	h := uint32(2166136261)
	mix := func(x int) {
		h ^= uint32(x)
		h *= 16777619
	}
	mix(plBody.size)
	mix(plBodySim.size)
	mix(plBodyState.size)
	mix(plShape[CircleShape].size)
	mix(32) // b2ChainShape
	mix(plContact.size)
	mix(plContactSim.size)
	mix(plJoint.size)
	mix(plJointSim[DistanceJoint].size)
	mix(64) // b2Island
	mix(plIslandSim.size)
	mix(plContactLnk.size)
	mix(plJointLnk.size)
	mix(56) // b2Sensor
	mix(plShapeRef.size)
	mix(88)  // b2SolverSet
	mix(64)  // b2GraphColor
	mix(104) // b2DynamicTree
	mix(plTreeNode.size)
	mix(plTreeProxy.size)
	mix(8)  // b2SetItem
	mix(24) // b2IdPool
	mix(32) // b2SurfaceMaterial
	mix(graphColorCount)
	mix(int(BodyTypeCount))
	mix(8) // sizeof(void*)
	if ScalarMode == "fixed" {
		// The fixed scalar is eight bytes, so every size above is the
		// size of a layout that no upstream build has.
		mix(int(qSize))
	}
	return h
}

func snapFlags() uint32 {
	var f uint32
	if IsDoublePrecision() {
		f |= snapFlagDouble
	}
	if ScalarMode == "fixed" {
		f |= snapFlagFixed
	}
	return f
}

// extend appends n zero bytes and returns them.
func (b *recBuffer) extend(n int) []byte {
	old := len(b.data)
	b.data = slices.Grow(b.data, n)[:old+n]
	clear(b.data[old:])
	b.size += n
	return b.data[old:]
}

func serPlan[T any](b *recBuffer, s []T, l *cLayout) {
	b.i32(int32(len(s)))
	for i := range s {
		l.put(b.extend(l.size), unsafe.Pointer(&s[i]))
	}
}

func serInts(b *recBuffer, s []int) {
	b.i32(int32(len(s)))
	for _, v := range s {
		b.i32(int32(v))
	}
}

func serIdPool(b *recBuffer, p *idPool) {
	b.i32(int32(p.nextIndex))
	serInts(b, p.freeArray)
}

func serBitSet(b *recBuffer, bs *bitSet) {
	b.u32(uint32(len(bs.bits)))
	for _, w := range bs.bits {
		b.u64(w)
	}
}

func serTree(b *recBuffer, t *dynamicTree) {
	b.i32(int32(t.nodeEnd))
	b.i32(int32(len(t.nodes)))
	b.i32(int32(t.pairFreeList))
	b.i32(int32(t.proxyCount))
	b.i32(int32(len(t.proxies)))
	b.i32(int32(t.proxyFreeList))
	if t.dfsOrdered {
		b.i32(1)
	} else {
		b.i32(0)
	}
	for i := range t.nodeEnd {
		plTreeNode.put(b.extend(plTreeNode.size), unsafe.Pointer(&t.nodes[i]))
	}
	for i := range t.nodeEnd {
		b.i32(t.parents[i])
	}
	for i := range t.proxies {
		plTreeProxy.put(b.extend(plTreeProxy.size), unsafe.Pointer(&t.proxies[i]))
	}
}

// serializeWorld corresponds to b2SerializeWorld.
func serializeWorld(w *world, b *recBuffer) {
	b.u32(snapMagic)
	b.u32(snapVersion)
	b.u32(snapLayoutHash())
	b.u32(snapFlags())

	// World config
	b.vec2(w.gravity)
	b.q(w.hitEventThreshold)
	b.q(w.restitutionThreshold)
	b.i32(int32(w.restitutionIterations))
	b.q(w.maxLinearSpeed)
	b.q(w.contactSpeed)
	b.q(w.contactHertz)
	b.q(w.contactDampingRatio)
	b.q(w.contactRecycleDistance)
	b.u64(w.stepIndex)
	b.i32(int32(w.splitIslandId))
	b.q(w.invH)
	b.q(w.invDt)
	b.i32(int32(w.endEventArrayIndex))
	c := &w.maxCapacity
	for _, v := range [...]int{c.staticShapeCount, c.dynamicShapeCount, c.staticBodyCount,
		c.dynamicBodyCount, c.contactCount, c.arenaByteCount} {
		b.i32(int32(v))
	}
	var flags uint8
	if w.enableSleep {
		flags |= 0x01
	}
	if w.enableWarmStarting {
		flags |= 0x02
	}
	if w.enableContinuous {
		flags |= 0x08
	}
	if w.enableRestitutionPropagation {
		flags |= 0x10
	}
	b.u8(flags)

	serIdPool(b, &w.bodyIdPool)
	serIdPool(b, &w.shapeIdPool)
	serIdPool(b, &w.chainIdPool)
	serIdPool(b, &w.contactIdPool)
	serIdPool(b, &w.jointIdPool)
	serIdPool(b, &w.islandIdPool)
	serIdPool(b, &w.solverSetIdPool)

	b.i32(int32(len(w.solverSets)))
	for i := range w.solverSets {
		set := &w.solverSets[i]
		b.i32(int32(set.setIndex))
		serPlan(b, set.bodySims, &plBodySim)
		serPlan(b, set.bodyStates, &plBodyState)
		serJointSims(b, set.jointSims)
		serContactSims(b, w, set.contactSims)
		serPlan(b, set.islandSims, &plIslandSim)
	}

	serPlan(b, w.bodies, &plBody)
	b.i32(int32(len(w.shapes)))
	for i := range w.shapes {
		s := &w.shapes[i]
		plShape[s.shapeType].put(b.extend(plShape[s.shapeType].size), unsafe.Pointer(s))
	}
	serPlan(b, w.fatAABBs, &plAABB)
	serPlan(b, w.contacts, &plContact)
	serPlan(b, w.joints, &plJoint)

	b.i32(int32(len(w.chainShapes)))
	for i := range w.chainShapes {
		chain := &w.chainShapes[i]
		b.i32(int32(chain.id))
		b.i32(int32(chain.bodyId))
		b.i32(int32(chain.nextChainId))
		b.i32(int32(len(chain.shapeIndices)))
		b.u16(chain.generation)
		if chain.id != nullIndex {
			for _, v := range chain.shapeIndices {
				b.i32(int32(v))
			}
		}
	}

	b.i32(int32(len(w.sensors)))
	for i := range w.sensors {
		s := &w.sensors[i]
		b.i32(int32(s.shapeId))
		serPlan(b, s.hits, &plShapeRef)
		serPlan(b, s.overlaps1, &plShapeRef)
		serPlan(b, s.overlaps2, &plShapeRef)
	}

	b.i32(int32(len(w.islands)))
	for i := range w.islands {
		is := &w.islands[i]
		b.i32(int32(is.setIndex))
		b.i32(int32(is.localIndex))
		b.i32(int32(is.islandId))
		b.i32(int32(is.constraintRemoveCount))
		serInts(b, is.bodies)
		serPlan(b, is.contacts, &plContactLnk)
		serPlan(b, is.joints, &plJointLnk)
	}

	bp := &w.broadPhase
	for t := range bp.trees {
		serTree(b, &bp.trees[t])
	}
	b.u32(uint32(len(bp.pairSet.items)))
	b.u32(uint32(bp.pairSet.count))
	for i := range bp.pairSet.items {
		b.u64(bp.pairSet.items[i].key)
	}

	for c := range graphColorCount {
		color := &w.constraintGraph.colors[c]
		if c != overflowIndex {
			serBitSet(b, &color.bodySet)
		}
		serContactSims(b, w, color.contactSims)
		serJointSims(b, color.jointSims)
	}
}

func serJointSims(b *recBuffer, sims []jointSim) {
	b.i32(int32(len(sims)))
	for i := range sims {
		l := &plJointSim[sims[i].jointType]
		l.put(b.extend(l.size), unsafe.Pointer(&sims[i]))
	}
}

// serContactSims writes contact sims. The reference stores in the sim the
// encoded index of a body: the awake index, or -(index + 2) for a static
// body, or null. The port keeps the awake index or null, so the static code
// is derived from the body here.
func serContactSims(b *recBuffer, w *world, sims []contactSim) {
	b.i32(int32(len(sims)))
	for i := range sims {
		cs := &sims[i]
		img := b.extend(plContactSim.size)
		plContactSim.put(img, unsafe.Pointer(cs))
		putI32(img[offContactSimBodyA:], encodeBodySimIndex(w, cs.bodySimIndexA, cs.shapeIdA))
		putI32(img[offContactSimBodyB:], encodeBodySimIndex(w, cs.bodySimIndexB, cs.shapeIdB))
	}
}

// The offsets of encodedBodySimA and encodedBodySimB in the C b2ContactSim:
// contactId 4, two rotations 16, a transform 16 in the float and double
// builds (36 and 40). They come from the plan, so a wider Q moves them.
var offContactSimBodyA, offContactSimBodyB int

// fieldOffset returns the image offset of the Go field of a plan.
func fieldOffset(l *cLayout, t reflect.Type, name string) int {
	f, ok := t.FieldByName(name)
	if !ok {
		panic("dbox2d: no field " + name + " in " + t.String())
	}
	for _, op := range l.ops {
		if op.goOff == f.Offset {
			return int(op.cOff)
		}
	}
	panic("dbox2d: no image offset for " + name)
}

func putI32(p []byte, v int) {
	u := uint32(int32(v))
	p[0], p[1], p[2], p[3] = byte(u), byte(u>>8), byte(u>>16), byte(u>>24)
}

func getI32(p []byte) int {
	return int(int32(uint32(p[0]) | uint32(p[1])<<8 | uint32(p[2])<<16 | uint32(p[3])<<24))
}

func encodeBodySimIndex(w *world, index int, shapeId int) int {
	if index >= 0 {
		return index
	}
	if shapeId < 0 || shapeId >= len(w.shapes) {
		return nullIndex
	}
	bodyId := w.shapes[shapeId].bodyId
	if bodyId < 0 || bodyId >= len(w.bodies) {
		return nullIndex
	}
	b := &w.bodies[bodyId]
	if b.setIndex == staticSet {
		return -(b.localIndex + 2)
	}
	return nullIndex
}

// snapReader is a bounds checked read cursor.
type snapReader struct {
	data   []byte
	cursor int
	ok     bool
}

func (r *snapReader) take(n int) []byte {
	if !r.ok || n < 0 || r.cursor+n > len(r.data) {
		r.ok = false
		return make([]byte, max(n, 0)&0xFFFF)
	}
	p := r.data[r.cursor : r.cursor+n]
	r.cursor += n
	return p
}

func (r *snapReader) i32() int {
	p := r.take(4)
	if len(p) < 4 {
		return 0
	}
	return getI32(p)
}

func (r *snapReader) u32() uint32 { return uint32(r.i32()) }

func (r *snapReader) u64() uint64 {
	p := r.take(8)
	if len(p) < 8 {
		return 0
	}
	return recLoad64(p)
}

func (r *snapReader) u16() uint16 {
	p := r.take(2)
	if len(p) < 2 {
		return 0
	}
	return uint16(p[0]) | uint16(p[1])<<8
}

func (r *snapReader) q() Q {
	var v Q
	p := r.take(int(qSize))
	if len(p) == int(qSize) {
		copy(unsafe.Slice((*byte)(unsafe.Pointer(&v)), qSize), p)
	}
	return v
}

func (r *snapReader) vec2() Vec2 { return Vec2{X: r.q(), Y: r.q()} }

// count reads an element count and checks that the stream holds that many
// elements of at least minBytes.
func (r *snapReader) count(minBytes int) int {
	n := r.i32()
	if !r.ok {
		return 0
	}
	if n < 0 || (minBytes > 0 && n > (len(r.data)-r.cursor)/minBytes) {
		r.ok = false
		return 0
	}
	return n
}

func desPlan[T any](r *snapReader, l *cLayout) []T {
	n := r.count(l.size)
	if n == 0 {
		return nil
	}
	out := make([]T, n)
	for i := range out {
		l.get(r.take(l.size), unsafe.Pointer(&out[i]))
	}
	return out
}

func desInts(r *snapReader) []int {
	n := r.count(4)
	if n == 0 {
		return nil
	}
	out := make([]int, n)
	for i := range out {
		out[i] = r.i32()
	}
	return out
}

func desIdPool(r *snapReader, p *idPool) {
	p.nextIndex = r.i32()
	p.freeArray = desInts(r)
}

func desBitSet(r *snapReader) bitSet {
	n := int(r.u32())
	if !r.ok || n > (len(r.data)-r.cursor)/8 {
		r.ok = false
		return bitSet{}
	}
	bs := bitSet{bits: make([]uint64, max(n, 1))[:n]}
	for i := range bs.bits {
		bs.bits[i] = r.u64()
	}
	return bs
}

func desTree(r *snapReader) dynamicTree {
	nodeEnd := r.i32()
	nodeCapacity := r.i32()
	pairFreeList := r.i32()
	proxyCount := r.i32()
	proxyCapacity := r.i32()
	proxyFreeList := r.i32()
	ordered := r.i32()
	if r.ok && (nodeEnd < 2 || nodeEnd&1 != 0 || nodeCapacity < nodeEnd || proxyCount < 0 ||
		proxyCapacity < 1 || proxyCapacity < proxyCount ||
		nodeEnd > (len(r.data)-r.cursor)/(plTreeNode.size+4) ||
		proxyCapacity > (len(r.data)-r.cursor)/plTreeProxy.size) {
		r.ok = false
	}
	var t dynamicTree
	if !r.ok {
		return createTree(0)
	}
	nodeCapacity = min(nodeCapacity, 2*nodeEnd)
	t.nodeEnd = nodeEnd
	t.pairFreeList = pairFreeList
	t.proxyCount = proxyCount
	t.proxyFreeList = proxyFreeList
	t.dfsOrdered = ordered != 0
	t.nodes = make([]treeNode, nodeCapacity)
	for i := range nodeEnd {
		plTreeNode.get(r.take(plTreeNode.size), unsafe.Pointer(&t.nodes[i]))
	}
	t.parents = make([]int32, nodeCapacity)
	for i := range nodeEnd {
		t.parents[i] = int32(r.i32())
	}
	t.proxies = make([]treeProxy, proxyCapacity)
	for i := range proxyCapacity {
		plTreeProxy.get(r.take(plTreeProxy.size), unsafe.Pointer(&t.proxies[i]))
	}
	return t
}

// openSnapshot validates the header and returns a reader past it.
func openSnapshot(image []byte) (snapReader, bool) {
	var r snapReader
	if len(image) < snapHeaderSize {
		return r, false
	}
	if recLoad32(image) != snapMagic || recLoad32(image[4:]) != snapVersion {
		return r, false
	}
	flags := recLoad32(image[12:])
	if flags&snapFlagDouble != snapFlags()&snapFlagDouble || flags&snapFlagFixed != snapFlags()&snapFlagFixed {
		fmt.Println("snapshot precision mismatch")
		return r, false
	}
	if recLoad32(image[8:]) != snapLayoutHash() {
		fmt.Println("snapshot layout mismatch")
		return r, false
	}
	r.data = image
	r.cursor = snapHeaderSize
	r.ok = true
	return r, true
}

// deserializeIntoWorld overwrites the simulation state of a world from the
// reader. Host wiring (callbacks, user data, workers) is untouched. It
// corresponds to b2DeserializeIntoShell.
func deserializeIntoWorld(r *snapReader, w *world) bool {
	w.gravity = r.vec2()
	w.hitEventThreshold = r.q()
	w.restitutionThreshold = r.q()
	w.restitutionIterations = r.i32()
	w.maxLinearSpeed = r.q()
	w.contactSpeed = r.q()
	w.contactHertz = r.q()
	w.contactDampingRatio = r.q()
	w.contactRecycleDistance = r.q()
	w.stepIndex = r.u64()
	w.splitIslandId = r.i32()
	w.invH = r.q()
	w.invDt = r.q()
	w.endEventArrayIndex = r.i32()
	c := &w.maxCapacity
	c.staticShapeCount = r.i32()
	c.dynamicShapeCount = r.i32()
	c.staticBodyCount = r.i32()
	c.dynamicBodyCount = r.i32()
	c.contactCount = r.i32()
	c.arenaByteCount = r.i32()
	flags := r.take(1)[0]
	w.enableSleep = flags&0x01 != 0
	w.enableWarmStarting = flags&0x02 != 0
	w.enableContinuous = flags&0x08 != 0
	w.enableRestitutionPropagation = flags&0x10 != 0
	if w.endEventArrayIndex != 0 && w.endEventArrayIndex != 1 {
		r.ok = false
	}

	desIdPool(r, &w.bodyIdPool)
	desIdPool(r, &w.shapeIdPool)
	desIdPool(r, &w.chainIdPool)
	desIdPool(r, &w.contactIdPool)
	desIdPool(r, &w.jointIdPool)
	desIdPool(r, &w.islandIdPool)
	desIdPool(r, &w.solverSetIdPool)

	setCount := r.count(24)
	w.solverSets = make([]solverSet, setCount)
	for i := range w.solverSets {
		set := &w.solverSets[i]
		set.setIndex = r.i32()
		set.bodySims = desPlan[bodySim](r, &plBodySim)
		set.bodyStates = desPlan[bodyState](r, &plBodyState)
		set.jointSims = desJointSims(r)
		set.contactSims = desContactSims(r)
		set.islandSims = desPlan[islandSim](r, &plIslandSim)
	}

	w.bodies = desPlan[body](r, &plBody)
	n := r.count(plShape[0].size)
	w.shapes = make([]shape, n)
	for i := range w.shapes {
		img := r.take(plShape[0].size)
		t := getI32(img[20:])
		if t < 0 || t >= int(ShapeTypeCount) {
			r.ok = false
			break
		}
		plShape[t].get(img, unsafe.Pointer(&w.shapes[i]))
	}
	w.fatAABBs = desPlan[AABB](r, &plAABB)
	w.contacts = desPlan[contact](r, &plContact)
	w.joints = desPlan[joint](r, &plJoint)

	n = r.count(18)
	w.chainShapes = make([]chainShape, n)
	for i := 0; i < n && r.ok; i++ {
		chain := &w.chainShapes[i]
		chain.id = r.i32()
		chain.bodyId = r.i32()
		chain.nextChainId = r.i32()
		segmentCount := r.i32()
		chain.generation = r.u16()
		if r.ok && chain.id != nullIndex {
			if segmentCount < 0 || segmentCount > (len(r.data)-r.cursor)/4 {
				r.ok = false
				break
			}
			chain.shapeIndices = make([]int, segmentCount)
			for k := range chain.shapeIndices {
				chain.shapeIndices[k] = r.i32()
			}
		}
	}

	n = r.count(16)
	w.sensors = make([]sensor, n)
	for i := 0; i < n && r.ok; i++ {
		s := &w.sensors[i]
		s.shapeId = r.i32()
		s.hits = desPlan[shapeRef](r, &plShapeRef)
		s.overlaps1 = desPlan[shapeRef](r, &plShapeRef)
		s.overlaps2 = desPlan[shapeRef](r, &plShapeRef)
	}

	n = r.count(28)
	w.islands = make([]island, n)
	for i := 0; i < n && r.ok; i++ {
		is := &w.islands[i]
		is.setIndex = r.i32()
		is.localIndex = r.i32()
		is.islandId = r.i32()
		is.constraintRemoveCount = r.i32()
		is.bodies = desInts(r)
		is.contacts = desPlan[contactLink](r, &plContactLnk)
		is.joints = desPlan[jointLink](r, &plJointLnk)
	}

	bp := &w.broadPhase
	for t := range bp.trees {
		bp.trees[t] = desTree(r)
	}
	bp.movedSiblings = nil
	capacityItems := int(r.u32())
	count := int(r.u32())
	if r.ok && (capacityItems&(capacityItems-1) != 0 || count > capacityItems ||
		capacityItems > (len(r.data)-r.cursor)/8) {
		r.ok = false
	}
	bp.pairSet = hashSet{}
	if r.ok && capacityItems > 0 {
		bp.pairSet.items = make([]setItem, capacityItems)
		bp.pairSet.count = count
		for i := range bp.pairSet.items {
			key := r.u64()
			if key != 0 {
				bp.pairSet.items[i] = setItem{key: key, hash: keyHash(key)}
			}
		}
	}

	for c := range graphColorCount {
		color := &w.constraintGraph.colors[c]
		if c != overflowIndex {
			color.bodySet = desBitSet(r)
		}
		color.contactSims = desContactSims(r)
		color.jointSims = desJointSims(r)
	}

	if !r.ok {
		return false
	}

	// The event buffers are transient and start empty.
	w.bodyMoveEvents = w.bodyMoveEvents[:0]
	w.sensorBeginEvents = w.sensorBeginEvents[:0]
	w.sensorEndEvents[0] = w.sensorEndEvents[0][:0]
	w.sensorEndEvents[1] = w.sensorEndEvents[1][:0]
	w.contactBeginEvents = w.contactBeginEvents[:0]
	w.contactEndEvents[0] = w.contactEndEvents[0][:0]
	w.contactEndEvents[1] = w.contactEndEvents[1][:0]
	w.contactHitEvents = w.contactHitEvents[:0]
	w.jointEvents = w.jointEvents[:0]
	return true
}

func desJointSims(r *snapReader) []jointSim {
	n := r.count(plJointSim[0].size)
	if n == 0 {
		return nil
	}
	out := make([]jointSim, n)
	for i := range out {
		img := r.take(plJointSim[0].size)
		t := getI32(img[12:])
		if t < 0 || t >= len(plJointSim) {
			r.ok = false
			break
		}
		plJointSim[t].get(img, unsafe.Pointer(&out[i]))
	}
	return out
}

func desContactSims(r *snapReader) []contactSim {
	n := r.count(plContactSim.size)
	if n == 0 {
		return nil
	}
	out := make([]contactSim, n)
	for i := range out {
		img := r.take(plContactSim.size)
		plContactSim.get(img, unsafe.Pointer(&out[i]))
		out[i].bodySimIndexA = max(getI32(img[offContactSimBodyA:]), nullIndex)
		out[i].bodySimIndexB = max(getI32(img[offContactSimBodyB:]), nullIndex)
	}
	return out
}

// GetSnapshot writes the snapshot of the world into image and returns its
// size. When image is too small nothing is copied; the returned size tells
// the capacity to allocate. It corresponds to b2World_GetSnapshot.
func (worldId WorldId) GetSnapshot(image []byte) int {
	w := getWorldFromId(worldId)
	if w.locked {
		return 0
	}
	var buf recBuffer
	serializeWorld(w, &buf)
	if buf.size <= len(image) {
		copy(image, buf.data)
	}
	return buf.size
}

// RestoreFromSnapshot overwrites the simulation state of a live world with a
// snapshot. Callbacks, user data and the worker count are kept; the user data
// of bodies, shapes and joints is cleared, as it is not part of the image.
// A world that records or steps refuses. A corrupt image leaves the world
// unusable past the header check, as upstream. It corresponds to
// b2World_RestoreFromSnapshot.
func (worldId WorldId) RestoreFromSnapshot(image []byte) bool {
	r, ok := openSnapshot(image)
	if !ok {
		return false
	}
	w := getWorldFromId(worldId)
	if w.locked {
		return false
	}
	if w.recording != nil {
		fmt.Println("b2World_RestoreFromSnapshot: not supported while recording")
		return false
	}
	w.treeTask.wait()
	return deserializeIntoWorld(&r, w)
}

// CreateWorldFromSnapshot creates a new world from a snapshot, or returns the
// null id. It corresponds to b2CreateWorldFromSnapshot.
func CreateWorldFromSnapshot(image []byte, workerCount int) WorldId {
	r, ok := openSnapshot(image)
	if !ok {
		return WorldId{}
	}
	def := DefaultWorldDef()
	def.WorkerCount = workerCount
	id := CreateWorld(&def)
	if !id.IsValid() {
		return WorldId{}
	}
	w := getWorldFromId(id)
	if !deserializeIntoWorld(&r, w) {
		DestroyWorld(id)
		return WorldId{}
	}
	w.preSolveFcn = nil
	w.customFilterFcn = nil
	w.userData = nil
	return id
}

// GetStateHash hashes the transforms, velocities, contacts, joint impulses
// and the id pools. Two runs that agree on it agree on the observable
// simulation. It corresponds to b2World_GetStateHash.
func (worldId WorldId) GetStateHash() uint64 {
	return hashWorldStateDeep(getWorldFromId(worldId))
}

func hashWorldStateDeep(w *world) uint64 {
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
		hash = fnvMixInt(hash, b.setIndex)
		hash = fnvMixInt(hash, b.localIndex)
	}

	for i := range w.contacts {
		c := &w.contacts[i]
		if c.contactId != i {
			continue
		}
		hash = fnvMixInt(hash, c.setIndex)
		hash = fnvMixInt(hash, c.colorIndex)
		hash = fnvMixInt(hash, c.localIndex)
		if sim := getContactSim(w, c); sim != nil {
			m := &sim.manifold
			hash = fnvMixInt(hash, m.PointCount)
			for p := range m.PointCount {
				hash = fnvMixQ(hash, m.Points[p].NormalImpulse)
				hash = fnvMixQ(hash, m.Points[p].TangentImpulse)
				hash = fnvMixQ(hash, m.Points[p].TotalNormalImpulse)
			}
		}
	}

	vec := func(h uint64, v Vec2) uint64 { return fnvMixBytes(h, unsafe.Pointer(&v), unsafe.Sizeof(v)) }
	for i := range w.joints {
		j := &w.joints[i]
		if j.jointId != i {
			continue
		}
		hash = fnvMixInt(hash, j.setIndex)
		hash = fnvMixInt(hash, j.colorIndex)
		hash = fnvMixInt(hash, j.localIndex)
		sim := getJointSim(w, j)
		if sim == nil {
			continue
		}
		switch sim.jointType {
		case DistanceJoint:
			d := sim.distance()
			hash = fnvMixQ(hash, d.impulse)
			hash = fnvMixQ(hash, d.lowerImpulse)
			hash = fnvMixQ(hash, d.upperImpulse)
			hash = fnvMixQ(hash, d.motorImpulse)
		case MoverJoint:
			m := sim.mover()
			hash = fnvMixQ(hash, m.linearVelocityImpulse.X)
			hash = fnvMixQ(hash, m.linearVelocityImpulse.Y)
		case MotorJoint:
			m := sim.motor()
			hash = fnvMixQ(hash, m.linearVelocityImpulse.X)
			hash = fnvMixQ(hash, m.linearVelocityImpulse.Y)
			hash = fnvMixQ(hash, m.angularVelocityImpulse)
			hash = fnvMixQ(hash, m.linearSpringImpulse.X)
			hash = fnvMixQ(hash, m.linearSpringImpulse.Y)
			hash = fnvMixQ(hash, m.angularSpringImpulse)
		case PogoJoint:
			p := sim.pogo()
			hash = fnvMixQ(hash, p.impulse)
			hash = fnvMixQ(hash, p.velocity)
		case PrismaticJoint:
			p := sim.prismatic()
			hash = vec(hash, p.impulse)
			hash = fnvMixQ(hash, p.springImpulse)
			hash = fnvMixQ(hash, p.motorImpulse)
			hash = fnvMixQ(hash, p.lowerImpulse)
			hash = fnvMixQ(hash, p.upperImpulse)
		case RevoluteJoint:
			r := sim.revolute()
			hash = vec(hash, r.linearImpulse)
			hash = fnvMixQ(hash, r.springImpulse)
			hash = fnvMixQ(hash, r.motorImpulse)
			hash = fnvMixQ(hash, r.lowerImpulse)
			hash = fnvMixQ(hash, r.upperImpulse)
		case WeldJoint:
			wj := sim.weld()
			hash = vec(hash, wj.linearImpulse)
			hash = fnvMixQ(hash, wj.angularImpulse)
		case WheelJoint:
			wh := sim.wheel()
			hash = fnvMixQ(hash, wh.perpImpulse)
			hash = fnvMixQ(hash, wh.motorImpulse)
			hash = fnvMixQ(hash, wh.springImpulse)
			hash = fnvMixQ(hash, wh.lowerImpulse)
			hash = fnvMixQ(hash, wh.upperImpulse)
		}
	}

	for _, p := range [...]*idPool{&w.bodyIdPool, &w.shapeIdPool, &w.chainIdPool, &w.contactIdPool,
		&w.jointIdPool, &w.islandIdPool, &w.solverSetIdPool} {
		hash = fnvMixInt(hash, p.nextIndex)
		hash = fnvMixInt(hash, p.idCount())
	}
	return fnvMixInt(hash, len(w.solverSets))
}
