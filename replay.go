package b2

import (
	"fmt"
	"math"
	"unsafe"
)

// This file corresponds to src/replay.c of Box2D v3.2.0: the incremental
// player that re-runs a recording against a fresh world, checks the recorded
// ids and state hashes, and lets a viewer step, seek and inspect the queries
// of each frame. It reads the upstream byte format.

const (
	replayKeyframeIntervalDefault = 16
	replayKeyframeBudgetDefault   = 512 * 1024 * 1024
)

// ReplayInfo is the static metadata of a recording, resolved once when the
// player opens it. It corresponds to b2ReplayInfo.
type ReplayInfo struct {
	// FrameCount is the number of recorded steps.
	FrameCount int

	// WorkerCount is the worker count of the replay world.
	WorkerCount int

	// TimeStep is the time step of the first recorded step.
	TimeStep Q

	// SubStepCount is the sub-step count of the first recorded step.
	SubStepCount int

	// LengthScale is the length units per meter in effect when recorded. The
	// port has one length unit, so a replay ignores it, see D-002.
	LengthScale float32

	// Bounds is the accumulated world bounds over the recording, zero-extent
	// if unavailable.
	Bounds AABB
}

// ReplayQueryType is the kind of a recorded spatial query. It corresponds to
// b2ReplayQueryType.
type ReplayQueryType int

const (
	ReplayQueryOverlapAABB ReplayQueryType = iota
	ReplayQueryOverlapShape
	ReplayQueryCastRay
	ReplayQueryCastShape
	ReplayQueryCollideMover
	ReplayQueryCastRayClosest
	ReplayQueryCastMover
)

// ReplayQueryInfo describes a spatial query recorded during a replayed
// frame. It corresponds to b2ReplayQueryInfo.
type ReplayQueryInfo struct {
	Type        ReplayQueryType
	Filter      QueryFilter
	AABB        AABB
	Origin      Pos
	Translation Vec2
	HitCount    int
}

// ReplayQueryHit is one result of a recorded spatial query. It corresponds
// to b2ReplayQueryHit.
type ReplayQueryHit struct {
	ShapeId  ShapeId
	Point    Pos
	Normal   Vec2
	Fraction Q
}

// recDrawQuery is the per-frame record of one query call. It corresponds to
// b2RecDrawQuery.
type recDrawQuery struct {
	kind         ReplayQueryType
	filter       QueryFilter
	aabb         AABB
	proxy        ShapeProxy
	mover        Capsule
	origin       Pos
	translation  Vec2
	castFraction Q
	hitStart     int
	hitCount     int
}

// recKeyframe is a restore point captured during forward replay, so a
// backward seek re-simulates only the gap from the nearest keyframe. It
// corresponds to b2RecKeyframe.
type recKeyframe struct {
	image        []byte
	frame        int
	cursor       int
	bodyIds      []BodyId
	divergeFrame int
	diverged     bool
}

func (kf *recKeyframe) bytes() int {
	return cap(kf.image) + len(kf.bodyIds)*int(unsafe.Sizeof(BodyId{}))
}

// Replay is an incremental player. It owns a private copy of the recording
// and drives the replay one step at a time. It corresponds to b2Replay.
type Replay struct {
	data         []byte
	headerEnd    int
	lengthScale  float32
	frame        int
	frameCount   int
	workerCount  int
	recordedDt   Q
	recordedSubs int
	bounds       AABB
	divergeFrame int
	atEnd        bool
	rdr          recReader

	// The query store of the current frame, reset at the top of StepFrame.
	frameQueries []recDrawQuery
	frameHits    []recRecordedHit

	// The live bodies in creation order, tracked from the create and
	// destroy ops to drive a viewer outliner. A destroyed slot holds the
	// null id so the ordinals stay stable.
	bodyIds []BodyId

	// The frame 0 image restarts in place, so the world id stays stable.
	frame0Image   []byte
	frame0BodyIds []BodyId

	// The keyframe ring. The spacing doubles and the off-grid keyframes are
	// evicted once the memory budget is hit.
	keyframes           []recKeyframe
	keyframeBudget      int
	keyframeBytes       int
	keyframeMinInterval int
	keyframeInterval    int
	lastKeyframeFrame   int
}

// CreateReplay opens a recording and creates the replay world from its seed
// snapshot. A workerCount of zero or less lets the replay world use the
// default. The recording bytes are copied. It returns nil when the recording
// is malformed or comes from another scalar mode. It corresponds to
// b2CreateReplay.
func CreateReplay(data []byte, workerCount int) *Replay {
	if len(data) < recHeaderSize || len(data) > math.MaxInt32 {
		fmt.Println("b2CreateReplay: recording too small")
		return nil
	}
	if recLoad32(data) != recMagic {
		fmt.Printf("b2CreateReplay: bad magic (got 0x%08X)\n", recLoad32(data))
		return nil
	}
	major := uint16(data[4]) | uint16(data[5])<<8
	minor := uint16(data[6]) | uint16(data[7])<<8
	if major != recVersionMajor || minor != recVersionMinor {
		fmt.Printf("b2CreateReplay: version mismatch (file=%d.%d, runtime=%d.%d)\n", major, minor, recVersionMajor, recVersionMinor)
		return nil
	}
	reserved2 := recLoad32(data[8:])
	if (ScalarMode == "fixed") != (reserved2 == recFixedMarker) {
		fmt.Println("b2CreateReplay: the scalar mode of the recording differs from this build")
		return nil
	}
	lengthScale := math.Float32frombits(recLoad32(data[12:]))
	if data[17] != 8 {
		fmt.Printf("b2CreateReplay: pointer width mismatch (file=%d, runtime=8)\n", data[17])
		return nil
	}
	if data[18] != 0 {
		fmt.Println("b2CreateReplay: big-endian recording not supported")
		return nil
	}
	snapshotSize := recLoad64(data[24:])
	if snapshotSize == 0 || snapshotSize > uint64(len(data)-recHeaderSize) {
		fmt.Println("b2CreateReplay: missing or oversized snapshot")
		return nil
	}
	headerEnd := recHeaderSize + int(snapshotSize)

	// The player owns a copy so the caller can reuse its buffer.
	copied := make([]byte, len(data))
	copy(copied, data)

	p := &Replay{
		data:                copied,
		headerEnd:           headerEnd,
		lengthScale:         lengthScale,
		divergeFrame:        -1,
		keyframeBudget:      replayKeyframeBudgetDefault,
		keyframeMinInterval: replayKeyframeIntervalDefault,
		keyframeInterval:    replayKeyframeIntervalDefault,
	}
	p.rdr = recReader{data: copied, cursor: headerEnd, ok: true, owner: p}

	p.scan()

	p.rdr.worldId = CreateWorldFromSnapshot(copied[recHeaderSize:headerEnd], workerCount)
	if !p.rdr.worldId.IsValid() {
		fmt.Println("b2CreateReplay: snapshot deserialize failed")
		p.Destroy()
		return nil
	}
	p.workerCount = workerCount
	p.frame0Image = copied[recHeaderSize:headerEnd]

	// The seed snapshot holds the bodies present when recording began; only
	// the creates after it reach the tracker, so seed the list from the
	// restored world.
	p.seedBodyIds()
	p.frame0BodyIds = append([]BodyId(nil), p.bodyIds...)
	return p
}

// scan walks the records once without dispatching, to count the steps and
// read the tuning of the first step.
func (p *Replay) scan() {
	data := p.data
	cursor := p.headerEnd
	gotStep := false
	for cursor+4 <= len(data) {
		op := data[cursor]
		size := int(data[cursor+1]) | int(data[cursor+2])<<8 | int(data[cursor+3])<<16
		start := cursor + 4
		if start+size > len(data) {
			break
		}
		switch {
		case op == opStep:
			p.frameCount++
			if !gotStep && size >= 4+int(qSize)+4 {
				copy(unsafe.Slice((*byte)(unsafe.Pointer(&p.recordedDt)), qSize), data[start+4:start+4+int(qSize)])
				p.recordedSubs = int(int32(recLoad32(data[start+4+int(qSize):])))
				gotStep = true
			}
		case op == opRecordingBounds && size >= 4*int(qSize):
			copy(unsafe.Slice((*byte)(unsafe.Pointer(&p.bounds)), unsafe.Sizeof(p.bounds)), data[start:start+4*int(qSize)])
		}
		cursor = start + size
	}
}

func (p *Replay) trackBodyCreate(id BodyId) { p.bodyIds = append(p.bodyIds, id) }

// trackBodyDestroy leaves a hole so the later ordinals do not shift.
func (p *Replay) trackBodyDestroy(id BodyId) {
	for i := range p.bodyIds {
		if p.bodyIds[i] == id {
			p.bodyIds[i] = BodyId{}
			return
		}
	}
}

func (p *Replay) seedBodyIds() {
	w := getWorldFromId(p.rdr.worldId)
	p.bodyIds = p.bodyIds[:0]
	for i := range w.bodies {
		if w.bodies[i].id != i {
			continue // free slot
		}
		p.trackBodyCreate(makeBodyId(w, i))
	}
}

// dispatchOne reads and dispatches one record. It returns the opcode, or -1
// at the end of the data or on a fatal read error.
func (p *Replay) dispatchOne() int {
	rd := &p.rdr
	if rd.cursor >= len(rd.data) || !rd.ok {
		return -1
	}
	op := rd.u8()
	if !rd.ok {
		return -1
	}
	size := int(rd.u24())
	if !rd.ok {
		return -1
	}
	start := rd.cursor
	if !rd.dispatch(op) {
		fmt.Printf("b2ReplayFile: unknown opcode 0x%02X, skipping %d bytes\n", op, size)
		if size > len(rd.data)-start {
			rd.ok = false
		} else {
			rd.cursor = start + size
		}
	}
	return int(op)
}

// captureKeyframe stores a restore point for the frame just completed. The
// cursor already sits at the Step of the next frame.
func (p *Replay) captureKeyframe() {
	w := getWorldFromId(p.rdr.worldId)
	var buf recBuffer
	serializeWorld(w, &buf)
	newBytes := cap(buf.data) + len(p.bodyIds)*int(unsafe.Sizeof(BodyId{}))

	// Make room under the budget: doubling the spacing drops the off-grid
	// keyframes, roughly halving the bytes, until the new keyframe fits or
	// nothing more goes. The budget is soft when one snapshot exceeds it.
	for len(p.keyframes) > 0 && p.keyframeBytes+newBytes > p.keyframeBudget {
		p.keyframeInterval *= 2
		kept := p.keyframes[:0]
		keptBytes := 0
		for _, kf := range p.keyframes {
			if kf.frame%p.keyframeInterval == 0 {
				kept = append(kept, kf)
				keptBytes += kf.bytes()
			}
		}
		progress := len(kept) < len(p.keyframes)
		clear(p.keyframes[len(kept):])
		p.keyframes = kept
		p.keyframeBytes = keptBytes
		if !progress {
			break
		}
	}

	p.keyframes = append(p.keyframes, recKeyframe{
		image:        buf.data,
		frame:        p.frame,
		cursor:       p.rdr.cursor,
		bodyIds:      append([]BodyId(nil), p.bodyIds...),
		divergeFrame: p.divergeFrame,
		diverged:     p.rdr.diverged,
	})
	p.keyframeBytes += newBytes
	p.lastKeyframeFrame = p.frame
}

// restoreKeyframe restores the world and the player from a keyframe, so a
// backward seek resumes from it. The world is restored in place, so the
// world id stays stable.
func (p *Replay) restoreKeyframe(kf *recKeyframe) {
	if !p.rdr.worldId.RestoreFromSnapshot(kf.image) {
		p.rdr.ok = false
		return
	}
	p.rdr.cursor = kf.cursor
	p.rdr.ok = true
	p.rdr.diverged = kf.diverged
	p.frame = kf.frame
	p.divergeFrame = kf.divergeFrame
	p.atEnd = false
	p.bodyIds = append(p.bodyIds[:0], kf.bodyIds...)
}

// StepFrame advances the replay by one recorded step and the records that
// trail it, and reports whether a step ran. It corresponds to
// b2Replay_StepFrame.
func (p *Replay) StepFrame() bool {
	if p.atEnd {
		return false
	}

	// Reset the per-frame query store before dispatching new records.
	p.frameQueries = p.frameQueries[:0]
	p.frameHits = p.frameHits[:0]

	// Run the Step of this frame, then consume the records that trail it
	// (the state hash, the queries, any mutators between frames) up to the
	// next Step. They were recorded after the Step they follow, so grouping
	// them with it keeps them paired with the state they were computed on.
	stepped := false
	for {
		if p.rdr.cursor >= len(p.rdr.data) || !p.rdr.ok {
			p.atEnd = true
			return stepped
		}
		if stepped && p.rdr.data[p.rdr.cursor] == opStep {
			// Capture a keyframe at the interval. The guard skips frames
			// already covered, so re-stepping a gap during a backward seek
			// never captures again.
			if p.frame > p.lastKeyframeFrame && p.frame%p.keyframeInterval == 0 {
				p.captureKeyframe()
			}
			return true
		}
		op := p.dispatchOne()
		if op < 0 {
			p.atEnd = true
			return stepped
		}
		if op == int(opStep) {
			p.frame++
			stepped = true
		}
		// Latch the first frame that diverged for the timeline marker.
		if p.divergeFrame < 0 && p.rdr.diverged {
			p.divergeFrame = p.frame
		}
	}
}

// Restart rewinds the replay to frame 0 in place. The world id stays stable.
// It corresponds to b2Replay_Restart.
func (p *Replay) Restart() {
	if !p.rdr.worldId.RestoreFromSnapshot(p.frame0Image) {
		p.rdr.ok = false
		return
	}
	p.rdr.cursor = p.headerEnd
	p.rdr.ok = true
	p.rdr.diverged = false
	p.frame = 0
	p.divergeFrame = -1
	p.atEnd = false

	// Frame 0 is the pre-step snapshot and has no recorded queries.
	p.frameQueries = p.frameQueries[:0]
	p.frameHits = p.frameHits[:0]

	p.bodyIds = append(p.bodyIds[:0], p.frame0BodyIds...)
}

// GetWorldId returns the replay world. It corresponds to
// b2Replay_GetWorldId.
func (p *Replay) GetWorldId() WorldId {
	if p == nil {
		return WorldId{}
	}
	return p.rdr.worldId
}

// GetFrame returns the number of steps dispatched so far. It corresponds to
// b2Replay_GetFrame.
func (p *Replay) GetFrame() int {
	if p == nil {
		return 0
	}
	return p.frame
}

// SeekFrame replays to the target frame, restoring the nearest keyframe
// first when that beats stepping. It corresponds to b2Replay_SeekFrame.
func (p *Replay) SeekFrame(targetFrame int) {
	if p == nil {
		return
	}
	targetFrame = max(targetFrame, 0)

	// Resume from the nearest keyframe strictly below the target when it
	// beats the current cursor. A backward seek must restore since the
	// cursor cannot rewind. A forward seek restores only when a keyframe
	// sits ahead of the cursor, which caps a long forward jump at one
	// keyframe interval. Strictly below, so the step loop still runs the
	// target frame and rebuilds its query store, body list and divergence
	// latch as a plain forward replay would.
	var best *recKeyframe
	for i := range p.keyframes {
		kf := &p.keyframes[i]
		if kf.frame < targetFrame && (best == nil || kf.frame > best.frame) {
			best = kf
		}
	}
	if targetFrame < p.frame {
		if best != nil {
			p.restoreKeyframe(best)
		} else {
			p.Restart()
		}
	} else if best != nil && best.frame > p.frame {
		p.restoreKeyframe(best)
	}
	for p.frame < targetFrame && p.StepFrame() {
	}
}

// GetInfo returns the metadata of the recording. It corresponds to
// b2Replay_GetInfo.
func (p *Replay) GetInfo() ReplayInfo {
	if p == nil {
		return ReplayInfo{}
	}
	return ReplayInfo{
		FrameCount:   p.frameCount,
		WorkerCount:  p.workerCount,
		TimeStep:     p.recordedDt,
		SubStepCount: p.recordedSubs,
		LengthScale:  p.lengthScale,
		Bounds:       p.bounds,
	}
}

// IsAtEnd reports whether a StepFrame ran out of records. It corresponds to
// b2Replay_IsAtEnd.
func (p *Replay) IsAtEnd() bool { return p == nil || p.atEnd }

// HasDiverged reports whether a state hash or a query did not reproduce. It
// corresponds to b2Replay_HasDiverged.
func (p *Replay) HasDiverged() bool { return p != nil && p.rdr.diverged }

// GetDivergeFrame returns the first frame that diverged, or -1. It
// corresponds to b2Replay_GetDivergeFrame.
func (p *Replay) GetDivergeFrame() int {
	if p == nil {
		return -1
	}
	return p.divergeFrame
}

// SetKeyframePolicy sets the memory budget in bytes and the finest spacing
// in frames of the keyframes. A value of zero or less keeps the current
// setting. It drops the captured keyframes. It corresponds to
// b2Replay_SetKeyframePolicy.
func (p *Replay) SetKeyframePolicy(budgetBytes int, minIntervalFrames int) {
	if p == nil {
		return
	}
	if budgetBytes > 0 {
		p.keyframeBudget = budgetBytes
	}
	if minIntervalFrames > 0 {
		p.keyframeMinInterval = minIntervalFrames
	}
	clear(p.keyframes)
	p.keyframes = p.keyframes[:0]
	p.keyframeBytes = 0
	p.keyframeInterval = p.keyframeMinInterval
	p.lastKeyframeFrame = 0
}

// GetKeyframeBudget returns the keyframe memory budget in bytes. It
// corresponds to b2Replay_GetKeyframeBudget.
func (p *Replay) GetKeyframeBudget() int {
	if p == nil {
		return 0
	}
	return p.keyframeBudget
}

// GetKeyframeMinInterval returns the finest keyframe spacing in frames. It
// corresponds to b2Replay_GetKeyframeMinInterval.
func (p *Replay) GetKeyframeMinInterval() int {
	if p == nil {
		return 0
	}
	return p.keyframeMinInterval
}

// GetKeyframeInterval returns the current keyframe spacing, a power-of-two
// multiple of the minimum. It corresponds to b2Replay_GetKeyframeInterval.
func (p *Replay) GetKeyframeInterval() int {
	if p == nil {
		return 0
	}
	return p.keyframeInterval
}

// GetKeyframeBytes returns the bytes that the kept keyframes hold. It
// corresponds to b2Replay_GetKeyframeBytes.
func (p *Replay) GetKeyframeBytes() int {
	if p == nil {
		return 0
	}
	return p.keyframeBytes
}

// Destroy destroys the replay world and releases the player. It corresponds
// to b2DestroyReplay.
func (p *Replay) Destroy() {
	if p == nil {
		return
	}
	if p.rdr.worldId.IsValid() {
		DestroyWorld(p.rdr.worldId)
	}
	*p = Replay{}
}

// GetFrameQueryCount returns the number of queries of the current frame. It
// corresponds to b2Replay_GetFrameQueryCount.
func (p *Replay) GetFrameQueryCount() int {
	if p == nil {
		return 0
	}
	return len(p.frameQueries)
}

// GetFrameQuery returns the description of a query of the current frame. It
// corresponds to b2Replay_GetFrameQuery.
func (p *Replay) GetFrameQuery(index int) ReplayQueryInfo {
	if p == nil || index < 0 || index >= len(p.frameQueries) {
		return ReplayQueryInfo{}
	}
	q := &p.frameQueries[index]
	return ReplayQueryInfo{
		Type:        q.kind,
		Filter:      q.filter,
		AABB:        q.aabb,
		Origin:      q.origin,
		Translation: q.translation,
		HitCount:    q.hitCount,
	}
}

// GetFrameQueryHit returns one result of a query of the current frame. It
// corresponds to b2Replay_GetFrameQueryHit.
func (p *Replay) GetFrameQueryHit(queryIndex, hitIndex int) ReplayQueryHit {
	if p == nil || queryIndex < 0 || queryIndex >= len(p.frameQueries) {
		return ReplayQueryHit{}
	}
	q := &p.frameQueries[queryIndex]
	if hitIndex < 0 || hitIndex >= q.hitCount {
		return ReplayQueryHit{}
	}
	h := &p.frameHits[q.hitStart+hitIndex]
	return ReplayQueryHit{ShapeId: h.id, Point: h.point, Normal: h.normal, Fraction: h.fraction}
}

// GetBodyCount returns the number of body slots of the outliner list. A
// destroyed body leaves a null slot. It corresponds to b2Replay_GetBodyCount.
func (p *Replay) GetBodyCount() int {
	if p == nil {
		return 0
	}
	return len(p.bodyIds)
}

// GetBodyId returns a body of the outliner list, or the null id. It
// corresponds to b2Replay_GetBodyId.
func (p *Replay) GetBodyId(index int) BodyId {
	if p == nil || index < 0 || index >= len(p.bodyIds) {
		return BodyId{}
	}
	return p.bodyIds[index]
}

// ValidateRecording replays a whole recording and reports whether every
// state hash and query reproduced. It corresponds to b2ValidateRecording.
func ValidateRecording(data []byte, workerCount int) bool {
	p := CreateReplay(data, workerCount)
	if p == nil {
		return false
	}
	defer p.Destroy()
	for p.StepFrame() {
		if p.rdr.diverged {
			break
		}
	}
	return p.rdr.ok && !p.rdr.diverged
}
