//go:build !dbox2d_double || dbox2d_fixed

// With both tags this file still builds, so the only error is the one in
// pos_fixed_double.go.

package b2

// Pos is a world position. Box2D v3.2 keeps world positions in double
// precision under its large world option and in float otherwise; without the
// dbox2d_double tag a position is a Vec2 (decision M-06). It corresponds to
// b2Pos in include/box2d/math_types.h.
type Pos = Vec2

// WorldTransform places a body in the world. Like Pos it is the plain
// transform without the dbox2d_double tag. It corresponds to
// b2WorldTransform in include/box2d/math_types.h.
type WorldTransform = Transform

// hugeMeters is B2_HUGE in meters; see Huge.
const hugeMeters = 100 * 1000

// IsDoublePrecision reports whether the library was built with the
// dbox2d_double tag. It corresponds to b2IsDoublePrecision.
func IsDoublePrecision() bool { return false }

// PosZero returns the origin. It corresponds to b2Pos_zero.
func PosZero() Pos { return Pos{} }

// IsValidPos reports whether p is a usable position. It corresponds to
// b2IsValidPos.
func IsValidPos(p Pos) bool { return IsValidVec2(p) }

// IsValidWorldTransform reports whether t has a usable position and a valid
// rotation. It corresponds to b2IsValidWorldTransform.
func IsValidWorldTransform(t WorldTransform) bool { return IsValidTransform(t) }

// ToPos converts a vector to a world position. It corresponds to b2ToPos.
func ToPos(v Vec2) Pos { return v }

// ToVec2 converts a world position to a vector. It corresponds to b2ToVec2.
func ToVec2(p Pos) Vec2 { return p }

// SubPos returns a - b as a vector. It corresponds to b2SubPos.
func SubPos(a, b Pos) Vec2 { return a.Sub(b) }

// OffsetPos returns p + d. It corresponds to b2OffsetPos.
func OffsetPos(p Pos, d Vec2) Pos { return p.Add(d) }

// LerpPos interpolates two world positions. It corresponds to b2LerpPos.
func LerpPos(a, b Pos, t Q) Pos { return Lerp(a, b, t) }

// TransformWorldPoint moves a local point into the world. It corresponds to
// b2TransformWorldPoint.
func TransformWorldPoint(t WorldTransform, p Vec2) Pos { return TransformPoint(t, p) }

// InvTransformWorldPoint moves a world position into a local frame. It
// corresponds to b2InvTransformWorldPoint.
func InvTransformWorldPoint(t WorldTransform, p Pos) Vec2 { return InvTransformPoint(t, p) }

// InvMulWorldTransforms returns frame B relative to frame A. It corresponds
// to b2InvMulWorldTransforms.
func InvMulWorldTransforms(a, b WorldTransform) Transform { return InvMulTransforms(a, b) }

// MulWorldTransforms moves a local transform into the world. It corresponds
// to b2MulWorldTransforms.
func MulWorldTransforms(a WorldTransform, b Transform) WorldTransform { return MulTransforms(a, b) }

// MakeWorldTransform promotes a transform to a world transform. It
// corresponds to b2MakeWorldTransform.
func MakeWorldTransform(t Transform) WorldTransform { return t }

// ToRelativeTransform shifts a world transform into the frame of a base
// position. It corresponds to b2ToRelativeTransform.
func ToRelativeTransform(t WorldTransform, base Pos) Transform {
	return Transform{P: t.P.Sub(base), Q: t.Q}
}

// offsetAABB moves a box by an origin. In the large world build of the
// reference the sum rounds outward to float; here it is a plain sum. It
// corresponds to b2OffsetAABB in src/aabb.h.
func offsetAABB(box AABB, origin Pos) AABB {
	return AABB{LowerBound: origin.Add(box.LowerBound), UpperBound: origin.Add(box.UpperBound)}
}

// posMin returns the component-wise minimum of two positions.
func posMin(a, b Pos) Pos { return Min(a, b) }

// posMax returns the component-wise maximum of two positions.
func posMax(a, b Pos) Pos { return Max(a, b) }

// posBox returns the box [lower - (radius + extra), upper + (radius +
// extra)] around two positions. The reference builds the box in double and
// rounds outward; here the helpers of the scalar round the sum once. It
// corresponds to the b2Compute*FatAABB helpers in src/geometry.c.
func posBox(lower, upper Pos, radius, extra Q) AABB {
	return AABB{
		LowerBound: Vec2{X: qSubSum(lower.X, radius, extra), Y: qSubSum(lower.Y, radius, extra)},
		UpperBound: Vec2{X: qAddSum(upper.X, radius, extra), Y: qAddSum(upper.Y, radius, extra)},
	}
}

// posBoxPoint returns the box that brackets the single position p.
func posBoxPoint(p Pos) AABB { return AABB{LowerBound: p, UpperBound: p} }

// WorldCastOutput is the world-space result of a shape cast. Without the
// dbox2d_double tag it is the plain cast output. It corresponds to
// b2WorldCastOutput in include/box2d/collision.h.
type WorldCastOutput = CastOutput

// checksumPos folds a world position into the state checksum.
func checksumPos(h uint64, p Pos) uint64 { return checksumVec2(h, p) }

// offsetPosSub returns p - d. The reference writes it as p + (-d), which is
// the same sum.
func offsetPosSub(p Pos, d Vec2) Pos { return p.Sub(d) }
