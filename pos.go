package b2

// Pos is a world position. Box2D v3.2 keeps world positions in double
// precision under its large world option and in float otherwise; this build
// has one precision, so a position is a Vec2 (decision M-06). It
// corresponds to b2Pos in include/box2d/math_functions.h.
type Pos = Vec2

// WorldTransform places a body in the world. Like Pos it is the plain
// transform in this build. It corresponds to b2WorldTransform in
// include/box2d/math_functions.h.
type WorldTransform = Transform

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
