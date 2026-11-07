package b2

// DrawFrameQueries draws the spatial queries of the current frame through a
// debug draw. A queryIndex below zero draws every query, otherwise only that
// one, which is what a viewer does for the selected query. It corresponds to
// b2Replay_DrawFrameQueries.
func (p *Replay) DrawFrameQueries(draw *DebugDraw, queryIndex int) {
	if p == nil || draw == nil {
		return
	}

	for qi := range p.frameQueries {
		if queryIndex >= 0 && qi != queryIndex {
			continue
		}
		q := &p.frameQueries[qi]
		hits := p.frameHits[q.hitStart : q.hitStart+q.hitCount]

		switch q.kind {
		case ReplayQueryCastRay, ReplayQueryCastRayClosest:
			// The ray from its origin to its end, then each hit point with a
			// short normal.
			if draw.DrawLineFcn != nil {
				draw.DrawLineFcn(q.origin, OffsetPos(q.origin, q.translation), ColorYellow)
			}
			p.drawHitNormals(draw, hits, ColorYellow, ColorLightYellow)

		case ReplayQueryCastShape:
			// Per hit points along the swept path.
			p.drawHitNormals(draw, hits, ColorSkyBlue, ColorLightSkyBlue)

		case ReplayQueryCastMover:
			// The mover capsule is relative to the query origin.
			if draw.DrawSolidCapsuleFcn != nil {
				c1 := OffsetPos(q.origin, q.mover.Center1)
				c2 := OffsetPos(q.origin, q.mover.Center2)
				draw.DrawSolidCapsuleFcn(c1, c2, q.mover.Radius, ColorLightSkyBlue)
			}

		case ReplayQueryOverlapAABB:
			// The query box is relative to the query origin.
			lower, upper := q.aabb.LowerBound, q.aabb.UpperBound
			if draw.DrawPolygonFcn != nil {
				vs := []Vec2{lower, {X: upper.X, Y: lower.Y}, upper, {X: lower.X, Y: upper.Y}}
				draw.DrawPolygonFcn(WorldTransform{P: q.origin, Q: RotIdentity()}, vs, ColorLimeGreen)
			}
			p.drawHitBounds(draw, hits)

		case ReplayQueryOverlapShape:
			// The proxy points are relative to the query origin.
			if q.proxy.Count == 1 {
				if draw.DrawCircleFcn != nil {
					draw.DrawCircleFcn(OffsetPos(q.origin, q.proxy.Points[0]), q.proxy.Radius, ColorLimeGreen)
				}
			} else if q.proxy.Count >= 2 && draw.DrawPolygonFcn != nil {
				draw.DrawPolygonFcn(WorldTransform{P: q.origin, Q: RotIdentity()}, q.proxy.Points[:q.proxy.Count], ColorLimeGreen)
			}
			p.drawHitBounds(draw, hits)

		case ReplayQueryCollideMover:
			// The mover capsule and the collision planes are relative to the
			// query origin.
			if draw.DrawSolidCapsuleFcn != nil {
				c1 := OffsetPos(q.origin, q.mover.Center1)
				c2 := OffsetPos(q.origin, q.mover.Center2)
				draw.DrawSolidCapsuleFcn(c1, c2, q.mover.Radius, ColorTan)
			}
			for i := range hits {
				h := &hits[i]
				if h.plane.Hit && draw.DrawLineFcn != nil {
					point := OffsetPos(q.origin, h.plane.Point)
					draw.DrawLineFcn(point, OffsetPos(point, h.plane.Plane.Normal.Mul(QFromRatio(1, 5))), ColorOrange)
				}
			}
		}
	}
}

// drawHitNormals draws each hit point with a short normal.
func (p *Replay) drawHitNormals(draw *DebugDraw, hits []recRecordedHit, pointColor, normalColor HexColor) {
	for i := range hits {
		h := &hits[i]
		if draw.DrawPointFcn != nil {
			draw.DrawPointFcn(h.point, QFromInt(4), pointColor)
		}
		if draw.DrawLineFcn != nil {
			draw.DrawLineFcn(h.point, OffsetPos(h.point, h.normal.Mul(QFromRatio(1, 5))), normalColor)
		}
	}
}

// drawHitBounds outlines the bounds of the shapes that an overlap hit.
func (p *Replay) drawHitBounds(draw *DebugDraw, hits []recRecordedHit) {
	if draw.DrawPolygonFcn == nil {
		return
	}
	for i := range hits {
		id := hits[i].id
		if !id.IsValid() {
			continue
		}
		b := id.GetBounds()
		vs := []Vec2{b.LowerBound, {X: b.UpperBound.X, Y: b.LowerBound.Y}, b.UpperBound, {X: b.LowerBound.X, Y: b.UpperBound.Y}}
		draw.DrawPolygonFcn(WorldTransform{Q: RotIdentity()}, vs, ColorMagenta)
	}
}
