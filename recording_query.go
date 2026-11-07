package b2

// queryRec buffers one spatial query record. The hits arrive while the query
// runs, so the record is committed after it finishes with the hit count
// patched in. It corresponds to b2RecQueryWriter.
type queryRec struct {
	r        *Recording
	buf      recBuffer
	countOff int
	hits     uint32
}

// beginQuery writes the inputs of a query and reserves the hit count.
func beginQuery(r *Recording, args ...any) *queryRec {
	q := &queryRec{r: r}
	for _, a := range args {
		q.buf.arg(a)
	}
	q.countOff = q.buf.size
	q.buf.u32(0)
	return q
}

// overlap wraps an overlap callback to record each reported shape.
func (q *queryRec) overlap(fcn OverlapResultFcn) OverlapResultFcn {
	return func(shapeId ShapeId) bool {
		ret := fcn(shapeId)
		q.buf.arg(shapeId)
		q.buf.boolean(ret)
		q.hits++
		return ret
	}
}

// cast wraps a cast callback to record each hit and the answer.
func (q *queryRec) cast(fcn CastResultFcn) CastResultFcn {
	return func(shapeId ShapeId, point Pos, normal Vec2, fraction Q) Q {
		ret := fcn(shapeId, point, normal, fraction)
		q.buf.arg(shapeId)
		q.buf.pos(point)
		q.buf.vec2(normal)
		q.buf.q(fraction)
		q.buf.q(ret)
		q.hits++
		return ret
	}
}

// plane wraps a mover plane callback to record each plane and the answer.
func (q *queryRec) plane(fcn PlaneResultFcn) PlaneResultFcn {
	return func(shapeId ShapeId, result *PlaneResult) bool {
		ret := fcn(shapeId, result)
		q.buf.arg(shapeId)
		q.buf.vec2(result.Plane.Normal)
		q.buf.q(result.Plane.Offset)
		q.buf.vec2(result.Point)
		q.buf.boolean(result.Hit)
		q.buf.boolean(ret)
		q.hits++
		return ret
	}
}

// commit patches the hit count, appends the tree statistics if the query
// has them, and writes the record.
func (q *queryRec) commit(op byte, stats *TreeStats) {
	q.buf.patchU32(q.countOff, q.hits)
	if stats != nil {
		q.buf.i32(int32(stats.NodeVisits))
		q.buf.i32(int32(stats.LeafVisits))
	}
	q.r.commitRecord(op, q.buf.data[:q.buf.size])
}

// recordOneShot writes a query that has no hit list.
func recordOneShot(r *Recording, op byte, args ...any) {
	var buf recBuffer
	for _, a := range args {
		buf.arg(a)
	}
	r.commitRecord(op, buf.data[:buf.size])
}
