//go:build dbox2d_double && !dbox2d_fixed

package b2

import "math"

// Pos is a world position in double precision, so coordinates stay accurate
// far from the origin. It is a distinct type; the dbox2d_double tag mirrors
// BOX2D_DOUBLE_PRECISION. It corresponds to b2Pos in
// include/box2d/math_types.h.
type Pos struct {
	X, Y float64
}

// WorldTransform places a body in the world, with a double precision
// translation and a rotation that stays float: the rotation is frame local
// and never needs the extra range. It corresponds to b2WorldTransform in
// include/box2d/math_types.h.
type WorldTransform struct {
	P Pos
	Q Rot
}

// hugeMeters is B2_HUGE in meters under large world mode: 100,000 km.
const hugeMeters = 100000 * 1000

// IsDoublePrecision reports whether the library was built with the
// dbox2d_double tag. It corresponds to b2IsDoublePrecision.
func IsDoublePrecision() bool { return true }

// PosZero returns the origin. It corresponds to b2Pos_zero.
func PosZero() Pos { return Pos{} }

// IsValidPos reports whether p is a usable position. It corresponds to
// b2IsValidPos.
func IsValidPos(p Pos) bool {
	return !math.IsNaN(p.X) && !math.IsNaN(p.Y) && !math.IsInf(p.X, 0) && !math.IsInf(p.Y, 0)
}

// IsValidWorldTransform reports whether t has a usable position and a valid
// rotation. It corresponds to b2IsValidWorldTransform.
func IsValidWorldTransform(t WorldTransform) bool {
	return IsValidPos(t.P) && IsValidRot(t.Q)
}

// ToPos converts a vector to a world position. It corresponds to b2ToPos.
func ToPos(v Vec2) Pos {
	return Pos{X: QToFloat64(v.X), Y: QToFloat64(v.Y)}
}

// ToVec2 converts a world position to a vector, rounding to float. It
// corresponds to b2ToVec2.
func ToVec2(p Pos) Vec2 {
	return Vec2{X: QFromFloat64(p.X), Y: QFromFloat64(p.Y)}
}

// SubPos returns a - b as a vector. The subtraction is in double and the
// result narrows to float; it is the primary precision boundary. It
// corresponds to b2SubPos.
func SubPos(a, b Pos) Vec2 {
	return Vec2{X: QFromFloat64(a.X - b.X), Y: QFromFloat64(a.Y - b.Y)}
}

// OffsetPos returns p + d, summed in double. It corresponds to b2OffsetPos.
func OffsetPos(p Pos, d Vec2) Pos {
	return Pos{X: p.X + QToFloat64(d.X), Y: p.Y + QToFloat64(d.Y)}
}

// LerpPos interpolates two world positions in double. It corresponds to
// b2LerpPos. Each product is converted so no target fuses it with the sum.
func LerpPos(a, b Pos, t Q) Pos {
	omt := QToFloat64(QOne().Sub(t))
	tt := QToFloat64(t)
	return Pos{
		X: float64(omt*a.X) + float64(tt*b.X),
		Y: float64(omt*a.Y) + float64(tt*b.Y),
	}
}

// TransformWorldPoint moves a local point into the world. The rotation runs
// in float and the translation in double. It corresponds to
// b2TransformWorldPoint.
func TransformWorldPoint(t WorldTransform, p Vec2) Pos {
	rx := t.Q.Cos.Mul(p.X).Sub(t.Q.Sin.Mul(p.Y))
	ry := t.Q.Sin.Mul(p.X).Add(t.Q.Cos.Mul(p.Y))
	return Pos{X: t.P.X + QToFloat64(rx), Y: t.P.Y + QToFloat64(ry)}
}

// InvTransformWorldPoint moves a world position into a local frame: one
// double subtraction, then float. It corresponds to
// b2InvTransformWorldPoint.
func InvTransformWorldPoint(t WorldTransform, p Pos) Vec2 {
	vx := QFromFloat64(p.X - t.P.X)
	vy := QFromFloat64(p.Y - t.P.Y)
	return Vec2{
		X: t.Q.Cos.Mul(vx).Add(t.Q.Sin.Mul(vy)),
		Y: t.Q.Sin.Neg().Mul(vx).Add(t.Q.Cos.Mul(vy)),
	}
}

// InvMulWorldTransforms returns frame B relative to frame A. It corresponds
// to b2InvMulWorldTransforms.
func InvMulWorldTransforms(a, b WorldTransform) Transform {
	var c Transform
	c.Q = InvMulRot(a.Q, b.Q)
	c.P = InvRotateVector(a.Q, SubPos(b.P, a.P))
	return c
}

// MulWorldTransforms moves a local transform into the world. It corresponds
// to b2MulWorldTransforms.
func MulWorldTransforms(a WorldTransform, b Transform) WorldTransform {
	var c WorldTransform
	c.Q = MulRot(a.Q, b.Q)
	c.P = OffsetPos(a.P, RotateVector(a.Q, b.P))
	return c
}

// MakeWorldTransform promotes a transform to a world transform. It is
// lossless. It corresponds to b2MakeWorldTransform.
func MakeWorldTransform(t Transform) WorldTransform {
	return WorldTransform{P: ToPos(t.P), Q: t.Q}
}

// ToRelativeTransform shifts a world transform into the frame of a base
// position. It corresponds to b2ToRelativeTransform.
func ToRelativeTransform(t WorldTransform, base Pos) Transform {
	return Transform{P: SubPos(t.P, base), Q: t.Q}
}

// roundDownFloat narrows a world coordinate to float, rounding toward
// negative infinity. With roundUpFloat it builds a conservative float box
// that always contains double bounds. It corresponds to b2roundDownFloat.
func roundDownFloat(x float64) Q {
	f := float32(x)
	if float64(f) > x {
		f = math.Nextafter32(f, -math.MaxFloat32)
	}
	return QFromFloat64(float64(f))
}

// roundUpFloat narrows a world coordinate to float, rounding toward
// positive infinity. It corresponds to b2roundUpFloat.
func roundUpFloat(x float64) Q {
	f := float32(x)
	if float64(f) < x {
		f = math.Nextafter32(f, math.MaxFloat32)
	}
	return QFromFloat64(float64(f))
}

// offsetAABB translates a relative box into world space, rounding outward so
// the float box always contains the true box far from the origin. It
// corresponds to b2OffsetAABB in src/aabb.h.
func offsetAABB(box AABB, origin Pos) AABB {
	return AABB{
		LowerBound: Vec2{
			X: roundDownFloat(origin.X + QToFloat64(box.LowerBound.X)),
			Y: roundDownFloat(origin.Y + QToFloat64(box.LowerBound.Y)),
		},
		UpperBound: Vec2{
			X: roundUpFloat(origin.X + QToFloat64(box.UpperBound.X)),
			Y: roundUpFloat(origin.Y + QToFloat64(box.UpperBound.Y)),
		},
	}
}

// posMin returns the component-wise minimum of two positions.
func posMin(a, b Pos) Pos {
	r := b
	if a.X < b.X {
		r.X = a.X
	}
	if a.Y < b.Y {
		r.Y = a.Y
	}
	return r
}

// posMax returns the component-wise maximum of two positions.
func posMax(a, b Pos) Pos {
	r := b
	if a.X > b.X {
		r.X = a.X
	}
	if a.Y > b.Y {
		r.Y = a.Y
	}
	return r
}

// posBox returns the box [lower - (radius + extra), upper + (radius +
// extra)] around two positions. The bounds are built in double and narrow
// to float once, outward, and the margin folds into the double step so it
// does not vanish into a float ULP far from the origin. It corresponds to
// the b2Compute*FatAABB helpers in src/geometry.c.
func posBox(lower, upper Pos, radius, extra Q) AABB {
	r := QToFloat64(radius) + QToFloat64(extra)
	return AABB{
		LowerBound: Vec2{X: roundDownFloat(lower.X - r), Y: roundDownFloat(lower.Y - r)},
		UpperBound: Vec2{X: roundUpFloat(upper.X + r), Y: roundUpFloat(upper.Y + r)},
	}
}

// posBoxPoint returns the box that brackets the single position p, so the
// float box still contains the true position far away.
func posBoxPoint(p Pos) AABB {
	return AABB{
		LowerBound: Vec2{X: roundDownFloat(p.X), Y: roundDownFloat(p.Y)},
		UpperBound: Vec2{X: roundUpFloat(p.X), Y: roundUpFloat(p.Y)},
	}
}

// WorldCastOutput is the world-space result of a shape cast. The hit point
// is a world position; the normal stays a float direction. It corresponds to
// b2WorldCastOutput in include/box2d/collision.h.
type WorldCastOutput struct {
	// Normal is the surface normal at the hit point.
	Normal Vec2

	// Point is the surface hit point in world space.
	Point Pos

	// Fraction is the part of the input translation at the collision.
	Fraction Q

	// Iterations is the number of iterations the cast used.
	Iterations int

	// Hit reports whether the cast hit.
	Hit bool
}

// checksumPos folds a world position into the state checksum, by the bits of
// its two doubles. Under the dbox2d_double tag the checksum of a world is not
// comparable to the checksum of the same world without the tag.
func checksumPos(h uint64, p Pos) uint64 {
	h = fnvFold(h, math.Float64bits(p.X))
	return fnvFold(h, math.Float64bits(p.Y))
}

// offsetPosSub returns p - d, summed in double. The reference writes it as
// p + (-d), which is the same sum.
func offsetPosSub(p Pos, d Vec2) Pos {
	return Pos{X: p.X - QToFloat64(d.X), Y: p.Y - QToFloat64(d.Y)}
}
