package dbox2d

import "github.com/dhannyell/fixed"

// PlaneResult is the collision plane and contact point for a mover collision.
type PlaneResult struct {
	Plane Plane
	Point Vec2
	Hit   bool
}

// CollisionPlane is a plane constraint for the mover plane solver.
type CollisionPlane struct {
	Plane        Plane
	PushLimit    Q
	Push         Q
	ClipVelocity bool
}

// PlaneSolverResult is the translation and iteration count from SolvePlanes.
type PlaneSolverResult struct {
	Translation    Vec2
	IterationCount int
}

// SolvePlanes ports b2SolvePlanes. Pass huge as PushLimit for a rigid plane.
func SolvePlanes(targetDelta Vec2, planes []CollisionPlane) PlaneSolverResult {
	zero := fixed.Q32Zero()
	for i := range planes {
		planes[i].Push = zero
	}

	delta := targetDelta
	iteration := 0
	for ; iteration < 20; iteration++ {
		totalPush := zero
		for i := range planes {
			plane := &planes[i]

			separation := PlaneSeparation(plane.Plane, delta).Add(linearSlop)
			push := separation.Neg()

			accumulatedPush := plane.Push
			plane.Push = plane.Push.Add(push).Clamp(zero, plane.PushLimit)
			push = plane.Push.Sub(accumulatedPush)
			delta = MulAdd(delta, push, plane.Plane.Normal)

			totalPush = totalPush.Add(push.Abs())
		}

		if totalPush.Less(linearSlop) {
			break
		}
	}

	return PlaneSolverResult{
		Translation:    delta,
		IterationCount: iteration,
	}
}

// ClipVector ports b2ClipVector.
func ClipVector(vector Vec2, planes []CollisionPlane) Vec2 {
	zero := fixed.Q32Zero()
	v := vector

	for i := range planes {
		plane := &planes[i]
		if plane.Push.Eq(zero) || !plane.ClipVelocity {
			continue
		}

		v = MulSub(v, v.Dot(plane.Plane.Normal).Min(zero), plane.Plane.Normal)
	}

	return v
}
