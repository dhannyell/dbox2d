package dbox2d

import "github.com/dhannyell/fixed"

// AABB is an axis-aligned bounding box.
type AABB struct {
	LowerBound Vec2
	UpperBound Vec2
}

// IsValidAABB reports whether a is usable and not inverted.
func IsValidAABB(a AABB) bool {
	d := a.UpperBound.Sub(a.LowerBound)
	valid := !d.X.Less(fixed.Zero()) && !d.Y.Less(fixed.Zero())
	valid = valid && IsValidVec2(a.LowerBound) && IsValidVec2(a.UpperBound)
	return valid
}

func perimeter(a AABB) Q {
	wx := a.UpperBound.X.Sub(a.LowerBound.X)
	wy := a.UpperBound.Y.Sub(a.LowerBound.Y)
	return fixed.FromInt(2).Mul(wx.Add(wy))
}

// enlargeAABB grows a until it contains b. It reports whether a grew.
func enlargeAABB(a *AABB, b AABB) bool {
	changed := false
	if b.LowerBound.X.Less(a.LowerBound.X) {
		a.LowerBound.X = b.LowerBound.X
		changed = true
	}

	if b.LowerBound.Y.Less(a.LowerBound.Y) {
		a.LowerBound.Y = b.LowerBound.Y
		changed = true
	}

	if a.UpperBound.X.Less(b.UpperBound.X) {
		a.UpperBound.X = b.UpperBound.X
		changed = true
	}

	if a.UpperBound.Y.Less(b.UpperBound.Y) {
		a.UpperBound.Y = b.UpperBound.Y
		changed = true
	}

	return changed
}

// AABBContains reports whether a fully contains b.
func AABBContains(a, b AABB) bool {
	s := true
	s = s && !b.LowerBound.X.Less(a.LowerBound.X)
	s = s && !b.LowerBound.Y.Less(a.LowerBound.Y)
	s = s && !a.UpperBound.X.Less(b.UpperBound.X)
	s = s && !a.UpperBound.Y.Less(b.UpperBound.Y)
	return s
}

// AABBCenter returns the center of a.
func AABBCenter(a AABB) Vec2 {
	half := fixed.Half()
	return Vec2{
		X: half.Mul(a.LowerBound.X.Add(a.UpperBound.X)),
		Y: half.Mul(a.LowerBound.Y.Add(a.UpperBound.Y)),
	}
}

// AABBExtents returns the half-widths of a.
func AABBExtents(a AABB) Vec2 {
	half := fixed.Half()
	return Vec2{
		X: half.Mul(a.UpperBound.X.Sub(a.LowerBound.X)),
		Y: half.Mul(a.UpperBound.Y.Sub(a.LowerBound.Y)),
	}
}

// AABBUnion returns the smallest box that contains a and b.
func AABBUnion(a, b AABB) AABB {
	var c AABB
	c.LowerBound.X = a.LowerBound.X.Min(b.LowerBound.X)
	c.LowerBound.Y = a.LowerBound.Y.Min(b.LowerBound.Y)
	c.UpperBound.X = a.UpperBound.X.Max(b.UpperBound.X)
	c.UpperBound.Y = a.UpperBound.Y.Max(b.UpperBound.Y)
	return c
}

// AABBOverlaps reports whether a and b overlap.
func AABBOverlaps(a, b AABB) bool {
	return !(a.UpperBound.X.Less(b.LowerBound.X) || a.UpperBound.Y.Less(b.LowerBound.Y) ||
		b.UpperBound.X.Less(a.LowerBound.X) || b.UpperBound.Y.Less(a.LowerBound.Y))
}

// MakeAABB returns the box that bounds a set of circles of the given radius.
// It panics when points is empty.
func MakeAABB(points []Vec2, radius Q) AABB {
	if len(points) == 0 {
		panic("dbox2d: MakeAABB needs at least one point")
	}

	a := AABB{LowerBound: points[0], UpperBound: points[0]}
	for i := 1; i < len(points); i++ {
		a.LowerBound = Min(a.LowerBound, points[i])
		a.UpperBound = Max(a.UpperBound, points[i])
	}

	r := Vec2{X: radius, Y: radius}
	a.LowerBound = a.LowerBound.Sub(r)
	a.UpperBound = a.UpperBound.Add(r)

	return a
}
