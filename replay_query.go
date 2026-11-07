package b2

import "fmt"

// This file corresponds to the query dispatchers of src/replay.c: each
// re-issues a recorded query against the replay world, compares the results
// with the recorded ones bit for bit and stashes the query for the viewer.

// checkId compares the id that a create call returned with the recorded one.
// Only the index and the generation are compared, since the world slot
// differs between record and replay. A mismatch means structural drift.
func (rd *recReader) checkId(kind string, gotIndex int, gotGen uint32, recIndex int, recGen uint32) {
	if gotIndex != recIndex || gotGen != recGen {
		fmt.Printf("b2ReplayFile: %s id mismatch (rec index1=%d gen=%d, got index1=%d gen=%d)\n", kind, recIndex, recGen, gotIndex, gotGen)
		rd.ok = false
	}
}

// stateHash compares the recorded state hash with the replay world.
func (rd *recReader) stateHash(recorded uint64) {
	computed := hashWorldState(getWorldFromId(rd.worldId))
	if computed != recorded {
		fmt.Printf("b2ReplayFile: StateHash mismatch (recorded=0x%X, computed=0x%X)\n", recorded, computed)
		// Non-fatal: reading continues so a viewer can show where the
		// divergence begins.
		rd.diverged = true
	}
}

func qDiffers(a, b Q) bool { return qBits(a) != qBits(b) }

func vec2Differs(a, b Vec2) bool { return qDiffers(a.X, b.X) || qDiffers(a.Y, b.Y) }

// replayQueryCtx walks the recorded hits in order and flags any divergence
// from the re-issued query. It corresponds to b2RecReplayQueryCtx.
type replayQueryCtx struct {
	rd     *recReader
	hits   []recRecordedHit
	cursor int
}

func (rc *replayQueryCtx) next() *recRecordedHit {
	if rc.cursor >= len(rc.hits) {
		rc.rd.diverged = true
		return nil
	}
	h := &rc.hits[rc.cursor]
	rc.cursor++
	return h
}

func (rc *replayQueryCtx) overlap(id ShapeId) bool {
	h := rc.next()
	if h == nil {
		return false
	}
	if id.index1 != h.id.index1 || id.generation != h.id.generation {
		rc.rd.diverged = true
	}
	return h.userReturnB
}

func (rc *replayQueryCtx) cast(id ShapeId, point Pos, normal Vec2, fraction Q) Q {
	h := rc.next()
	if h == nil {
		return QZero()
	}
	// Compare positions through the full width delta, truncating both sides
	// would pass vacuously far from the origin.
	if id.index1 != h.id.index1 || id.generation != h.id.generation ||
		vec2Differs(SubPos(point, h.point), Vec2{}) || vec2Differs(normal, h.normal) ||
		qDiffers(fraction, h.fraction) {
		rc.rd.diverged = true
	}
	return h.userReturnF
}

func (rc *replayQueryCtx) plane(id ShapeId, plane *PlaneResult) bool {
	h := rc.next()
	if h == nil {
		return true
	}
	if id.index1 != h.id.index1 || id.generation != h.id.generation ||
		vec2Differs(plane.Plane.Normal, h.plane.Plane.Normal) ||
		qDiffers(plane.Plane.Offset, h.plane.Plane.Offset) ||
		vec2Differs(plane.Point, h.plane.Point) || plane.Hit != h.plane.Hit {
		rc.rd.diverged = true
	}
	return h.userReturnB
}

// readOverlapHits reads the hit list of an overlap query and the tree stats.
func (rd *recReader) readOverlapHits() bool {
	n := int(rd.u32())
	rd.ensureHits(n)
	if !rd.ok {
		return false
	}
	for i := range n {
		rd.hits[i] = recRecordedHit{}
		rd.hits[i].id = rd.shapeId()
		rd.hits[i].userReturnB = rd.boolean()
	}
	rd.treeStats()
	return rd.ok
}

func (rd *recReader) readCastHits() bool {
	n := int(rd.u32())
	rd.ensureHits(n)
	if !rd.ok {
		return false
	}
	for i := range n {
		rd.hits[i] = recRecordedHit{}
		rd.hits[i].id = rd.shapeId()
		rd.hits[i].point = rd.pos()
		rd.hits[i].normal = rd.vec2()
		rd.hits[i].fraction = rd.q()
		rd.hits[i].userReturnF = rd.q()
	}
	rd.treeStats()
	return rd.ok
}

// stash records a query for the viewer. It corresponds to
// b2RecStashQueryBegin.
func (rd *recReader) stash(kind ReplayQueryType, hits []recRecordedHit) *recDrawQuery {
	p := rd.owner
	if p == nil {
		return nil
	}
	p.frameQueries = append(p.frameQueries, recDrawQuery{kind: kind, hitStart: len(p.frameHits), hitCount: len(hits)})
	p.frameHits = append(p.frameHits, hits...)
	return &p.frameQueries[len(p.frameQueries)-1]
}

func (rd *recReader) replayQueryOverlapAABB(origin Pos, aabb AABB, filter QueryFilter) {
	if !rd.readOverlapHits() {
		return
	}
	rc := replayQueryCtx{rd: rd, hits: rd.hits}
	rd.worldId.OverlapAABB(origin, aabb, filter, rc.overlap)
	if rc.cursor != len(rd.hits) {
		rd.diverged = true
	}
	if q := rd.stash(ReplayQueryOverlapAABB, rd.hits); q != nil {
		q.filter, q.origin, q.aabb = filter, origin, aabb
	}
}

func (rd *recReader) replayQueryOverlapShape(origin Pos, proxy ShapeProxy, filter QueryFilter) {
	if !rd.readOverlapHits() {
		return
	}
	rc := replayQueryCtx{rd: rd, hits: rd.hits}
	rd.worldId.OverlapShape(origin, &proxy, filter, rc.overlap)
	if rc.cursor != len(rd.hits) {
		rd.diverged = true
	}
	if q := rd.stash(ReplayQueryOverlapShape, rd.hits); q != nil {
		q.filter, q.origin, q.proxy = filter, origin, proxy
	}
}

func (rd *recReader) replayQueryCastRay(origin Pos, translation Vec2, filter QueryFilter) {
	if !rd.readCastHits() {
		return
	}
	rc := replayQueryCtx{rd: rd, hits: rd.hits}
	rd.worldId.CastRay(origin, translation, filter, rc.cast)
	if rc.cursor != len(rd.hits) {
		rd.diverged = true
	}
	if q := rd.stash(ReplayQueryCastRay, rd.hits); q != nil {
		q.filter, q.origin, q.translation = filter, origin, translation
	}
}

func (rd *recReader) replayQueryCastShape(origin Pos, proxy ShapeProxy, translation Vec2, filter QueryFilter) {
	if !rd.readCastHits() {
		return
	}
	rc := replayQueryCtx{rd: rd, hits: rd.hits}
	rd.worldId.CastShape(origin, &proxy, translation, filter, rc.cast)
	if rc.cursor != len(rd.hits) {
		rd.diverged = true
	}
	if q := rd.stash(ReplayQueryCastShape, rd.hits); q != nil {
		q.filter, q.origin, q.proxy, q.translation = filter, origin, proxy, translation
	}
}

func (rd *recReader) replayQueryCollideMover(origin Pos, mover *Capsule, filter QueryFilter) {
	n := int(rd.u32())
	rd.ensureHits(n)
	if !rd.ok {
		return
	}
	for i := range n {
		rd.hits[i] = recRecordedHit{}
		rd.hits[i].id = rd.shapeId()
		rd.hits[i].plane = rd.planeResult()
		rd.hits[i].userReturnB = rd.boolean()
	}
	// CollideMover has no tree stats tail, it returns nothing.
	if !rd.ok {
		return
	}
	rc := replayQueryCtx{rd: rd, hits: rd.hits}
	rd.worldId.CollideMover(origin, mover, filter, rc.plane)
	if rc.cursor != len(rd.hits) {
		rd.diverged = true
	}
	if q := rd.stash(ReplayQueryCollideMover, rd.hits); q != nil {
		q.filter, q.origin, q.mover = filter, origin, *mover
	}
}

func (rd *recReader) replayQueryCastRayClosest(origin Pos, translation Vec2, filter QueryFilter) {
	rec := rd.rayResult()
	if !rd.ok {
		return
	}
	got := rd.worldId.CastRayClosest(origin, translation, filter)
	if got.Hit != rec.Hit ||
		(got.Hit && (got.ShapeId.index1 != rec.ShapeId.index1 || got.ShapeId.generation != rec.ShapeId.generation ||
			vec2Differs(SubPos(got.Point, rec.Point), Vec2{}) ||
			vec2Differs(got.Normal, rec.Normal) || qDiffers(got.Fraction, rec.Fraction))) {
		rd.diverged = true
	}
	if rd.owner != nil {
		// Stash the closest result as a single hit so the shared draw loop
		// renders its point.
		var hits []recRecordedHit
		if rec.Hit {
			hits = []recRecordedHit{{id: rd.makeShape(rec.ShapeId), point: rec.Point, normal: rec.Normal, fraction: rec.Fraction}}
		}
		if q := rd.stash(ReplayQueryCastRayClosest, hits); q != nil {
			q.filter, q.origin, q.translation = filter, origin, translation
		}
	}
}

func (rd *recReader) replayQueryCastMover(origin Pos, mover *Capsule, translation Vec2, filter QueryFilter) {
	rec := rd.q()
	if !rd.ok {
		return
	}
	got := rd.worldId.CastMover(origin, mover, translation, filter)
	if qDiffers(got, rec) {
		rd.diverged = true
	}
	if q := rd.stash(ReplayQueryCastMover, nil); q != nil {
		q.filter, q.origin, q.mover, q.translation, q.castFraction = filter, origin, *mover, translation, rec
	}
}
