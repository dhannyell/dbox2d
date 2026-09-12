//go:build !dbox2d_fixed

package dbox2d

import (
	"math"
	"strconv"
)

// The library has one scalar per build mode. This file is the float mode:
// float32. Every product and division sits inside an explicit float32
// conversion. The conversion is a no-op on the value, but the Go spec makes
// it a rounding point, so the compiler cannot fuse the product with a
// following add. No float64 detour: it costs three conversions per product
// for the same bits.

type (
	// Q is a float32 scalar. Only the constructors make one.
	Q struct{ v float32 }

	// Vec2 is a 2D vector. It represents a point or a free vector.
	Vec2 struct {
		X, Y Q
	}

	// Rot is a 2D rotation, stored as a sine and cosine pair.
	Rot struct {
		Sin, Cos Q
	}
)

var (
	// floatPi is the reference B2_PI rounded to float32.
	floatPi = QMustParse("3.14159265359")

	// floatTau is one full turn in radians.
	floatTau = floatPi.Add(floatPi)

	// floatThreePi bounds the inline case of unwindAngle. Three times a
	// float32 is exact in float32 only by luck, so the product is float64
	// and the comparison happens there.
	floatThreePi = float32(3 * float64(floatPi.v))

	// The float mode uses FLT_EPSILON as its rounding-noise threshold.
	scalarEpsilon = QMustParse("1.1920929e-7")

	// The float mode squares FLT_EPSILON for squared-length comparisons.
	scalarEpsilonSq = scalarEpsilon.Mul(scalarEpsilon)

	// The float mode uses 10 * FLT_EPSILON for sensor overlap tests.
	sensorOverlapDistance = QFromInt(10).Mul(scalarEpsilon)

	// The reference uses 100 * FLT_EPSILON for normalized vectors.
	normalizedTolerance = QFromInt(100).Mul(scalarEpsilon)
)

// ScalarMode names the scalar mode of this build: "float" or "fixed".
const ScalarMode = "float"

// QZero returns zero.
func QZero() Q { return Q{0} }

// QOne returns one.
func QOne() Q { return Q{1} }

// QHalf returns one half.
func QHalf() Q { return Q{0.5} }

// QFromInt returns the integer i as a scalar.
func QFromInt(i int) Q { return Q{float32(i)} }

// QFromRatio returns num/den, rounded to float32.
func QFromRatio(num, den int) Q {
	return Q{float32(float64(num) / float64(den))}
}

// QMustParse returns the decimal literal s as a scalar. It panics when s is
// not a decimal number.
func QMustParse(s string) Q { return Q{mustParseFloat32(s)} }

// mustParseFloat32 stays out of line so that QMustParse fits the inlining
// budget, like the constructors of the fixed mode.
//
//go:noinline
func mustParseFloat32(s string) float32 {
	f, err := strconv.ParseFloat(s, 32)
	if err != nil {
		panic(err)
	}
	return float32(f)
}

// QMaxValue returns the largest finite float32 scalar.
func QMaxValue() Q { return Q{math.MaxFloat32} }

// QMinValue returns the smallest finite float32 scalar.
func QMinValue() Q { return Q{-math.MaxFloat32} }

// RotIdentity returns the rotation by zero turns.
func RotIdentity() Rot { return Rot{Sin: QZero(), Cos: QOne()} }

// MakeRot returns the rotation by the angle t, in turns.
func MakeRot(t Q) Rot {
	x := t.Sub(t.Round())
	return makeRotRadians(x.Mul(floatTau))
}

// makeRotRadians is b2ComputeCosSin after its unwind: the rotation by the
// angle radians, in [-pi, pi].
func makeRotRadians(radians Q) Rot {
	pi2 := floatPi.Mul(floatPi)
	var c Q
	if radians.Less(QHalf().Neg().Mul(floatPi)) {
		y := radians.Add(floatPi)
		y2 := y.Mul(y)
		c = pi2.Sub(QFromInt(4).Mul(y2)).Neg().Div(pi2.Add(y2))
	} else if radians.Greater(QHalf().Mul(floatPi)) {
		y := radians.Sub(floatPi)
		y2 := y.Mul(y)
		c = pi2.Sub(QFromInt(4).Mul(y2)).Neg().Div(pi2.Add(y2))
	} else {
		y2 := radians.Mul(radians)
		c = pi2.Sub(QFromInt(4).Mul(y2)).Div(pi2.Add(y2))
	}

	var s Q
	if radians.Less(QZero()) {
		y := radians.Add(floatPi)
		p := floatPi.Sub(y)
		numerator := QFromInt(16).Neg().Mul(y).Mul(p)
		denominator := QFromInt(5).Mul(pi2).Sub(QFromInt(4).Mul(y).Mul(p))
		s = numerator.Div(denominator)
	} else {
		p := floatPi.Sub(radians)
		numerator := QFromInt(16).Mul(radians).Mul(p)
		denominator := QFromInt(5).Mul(pi2).Sub(QFromInt(4).Mul(radians).Mul(p))
		s = numerator.Div(denominator)
	}

	mag := s.Mul(s).Add(c.Mul(c)).Sqrt()
	invMag := QZero()
	if mag.Greater(QZero()) {
		invMag = QOne().Div(mag)
	}
	return Rot{Sin: s.Mul(invMag), Cos: c.Mul(invMag)}
}

// atan2Turns returns the angle of (x, y), in turns.
func atan2Turns(y, x Q) Q { return atan2Radians(y, x).Div(floatTau) }

// atan2Radians is b2Atan2: the angle of (x, y), in radians.
func atan2Radians(y, x Q) Q {
	if x.Eq(QZero()) && y.Eq(QZero()) {
		return QZero()
	}

	ax := x.Abs()
	ay := y.Abs()
	mx := ay.Max(ax)
	mn := ay.Min(ax)
	a := mn.Div(mx)

	s := a.Mul(a)
	c := s.Mul(a)
	q := s.Mul(s)
	r := Q{0.024840285}.Mul(q).Add(Q{0.18681418})
	t := Q{-0.094097948}.Mul(q).Sub(Q{0.33213072})
	r = r.Mul(s).Add(t)
	r = r.Mul(c).Add(a)

	if ay.Greater(ax) {
		r = Q{1.57079637}.Sub(r)
	}
	if x.Less(QZero()) {
		r = Q{3.14159274}.Sub(r)
	}
	if y.Less(QZero()) {
		r = r.Neg()
	}
	return r
}

// A joint keeps its angles, its limits, its angular offset and its angular
// motor speed in the angle unit of the mode (D-004). Float mode keeps
// radians, the unit of the reference, so each joint computes its errors as
// the reference does. The API converts from turns and back.
func angleFromTurns(t Q) Q { return t.Mul(floatTau) }

func angleToTurns(a Q) Q { return a.Div(floatTau) }

func angleRadians(a Q) Q { return a }

// relativeAngle is b2RelativeAngle: the angle of b relative to a, in
// radians.
func relativeAngle(b, a Rot) Q {
	s := b.Sin.Mul(a.Cos).Sub(b.Cos.Mul(a.Sin))
	c := b.Cos.Mul(a.Cos).Add(b.Sin.Mul(a.Sin))
	return atan2Radians(s, c)
}

// unwindAngle is b2UnwindAngle, remainderf by two pi. The remainder of two
// float32 values is exact, so the float64 remainder rounds to it unchanged.
//
// math.Remainder is a software loop and the C side is one libm call, so
// the two common cases are handled inline with the bits the remainder
// would produce. The nearest multiple of tau is zero up to and including
// pi, where ties round to even, so the angle returns unchanged. For an
// angle strictly between pi and three pi the multiple is one, and the
// difference with tau is exact by Sterbenz, because the angle lies within
// a factor of two of tau. Everything else, including NaN, takes the slow
// path. TestUnwindAngleMatchesRemainder pins the bits.
func unwindAngle(a Q) Q {
	if -floatPi.v <= a.v && a.v <= floatPi.v {
		return a
	}
	if floatPi.v < a.v && a.v < floatThreePi {
		return Q{a.v - floatTau.v}
	}
	if -floatThreePi < a.v && a.v < -floatPi.v {
		// Negated so that exactly minus tau yields the negative zero of
		// the remainder rather than the positive zero of the sum.
		return Q{-(-a.v - floatTau.v)}
	}
	return Q{float32(math.Remainder(float64(a.v), float64(floatTau.v)))}
}

// Add returns q+o.
func (q Q) Add(o Q) Q { return Q{q.v + o.v} }

// Sub returns q-o.
func (q Q) Sub(o Q) Q { return Q{q.v - o.v} }

// Mul returns q*o. The conversion blocks fusion with a following add.
func (q Q) Mul(o Q) Q { return Q{float32(q.v * o.v)} }

// Div returns q/o. The conversion blocks fusion with a following add.
func (q Q) Div(o Q) Q { return Q{float32(q.v / o.v)} }

// Sqrt returns the square root, rounded once to float32.
func (q Q) Sqrt() Q { return Q{float32(math.Sqrt(float64(q.v)))} }

// Neg returns -q.
func (q Q) Neg() Q { return Q{-q.v} }

// Abs returns q with its sign bit cleared.
func (q Q) Abs() Q {
	return Q{math.Float32frombits(math.Float32bits(q.v) &^ (1 << 31))}
}

// Eq reports whether q equals o.
func (q Q) Eq(o Q) bool { return q.v == o.v }

// Less reports whether q is less than o.
func (q Q) Less(o Q) bool { return q.v < o.v }

// Greater reports whether q is greater than o.
func (q Q) Greater(o Q) bool { return q.v > o.v }

// Cmp returns -1 when q < o, 0 when q == o, and 1 when q > o.
func (q Q) Cmp(o Q) int {
	if q.v < o.v {
		return -1
	}
	if q.v > o.v {
		return 1
	}
	return 0
}

// Min returns the smaller of q and o.
func (q Q) Min(o Q) Q {
	if q.v < o.v {
		return q
	}
	return o
}

// Max returns the larger of q and o.
func (q Q) Max(o Q) Q {
	if q.v > o.v {
		return q
	}
	return o
}

// Clamp returns q limited to [lo, hi].
func (q Q) Clamp(lo, hi Q) Q {
	if q.v < lo.v {
		return lo
	}
	if q.v > hi.v {
		return hi
	}
	return q
}

// Round returns the nearest integer, with halves away from zero.
func (q Q) Round() Q { return Q{float32(math.Round(float64(q.v)))} }

// Int returns q truncated toward zero.
func (q Q) Int() int { return int(q.v) }

// String returns q in the shortest float32 decimal form.
func (q Q) String() string {
	return strconv.FormatFloat(float64(q.v), 'g', -1, 32)
}

// Add returns the component-wise sum of two vectors.
func (v Vec2) Add(o Vec2) Vec2 {
	return Vec2{X: v.X.Add(o.X), Y: v.Y.Add(o.Y)}
}

// Sub returns the component-wise difference of two vectors.
func (v Vec2) Sub(o Vec2) Vec2 {
	return Vec2{X: v.X.Sub(o.X), Y: v.Y.Sub(o.Y)}
}

// Mul returns v scaled by s.
func (v Vec2) Mul(s Q) Vec2 {
	return Vec2{X: v.X.Mul(s), Y: v.Y.Mul(s)}
}

// Div returns v divided by s.
func (v Vec2) Div(s Q) Vec2 {
	return Vec2{X: v.X.Div(s), Y: v.Y.Div(s)}
}

// Dot returns the dot product of two vectors.
func (v Vec2) Dot(o Vec2) Q {
	return v.X.Mul(o.X).Add(v.Y.Mul(o.Y))
}

// LenSq returns the squared length of the vector.
func (v Vec2) LenSq() Q { return v.Dot(v) }

// Len returns the length of the vector.
func (v Vec2) Len() Q { return v.LenSq().Sqrt() }

// Normalize returns a unit vector with the same direction as v. The zero
// vector returns the zero vector.
func (v Vec2) Normalize() Vec2 {
	length := v.Len()
	if length.Less(scalarEpsilon) {
		return Vec2{}
	}
	invLength := QOne().Div(length)
	return Vec2{X: invLength.Mul(v.X), Y: invLength.Mul(v.Y)}
}

// Distance returns the distance between v and o.
func (v Vec2) Distance(o Vec2) Q { return v.Sub(o).Len() }

// DistanceSq returns the squared distance between v and o.
func (v Vec2) DistanceSq(o Vec2) Q { return v.Sub(o).LenSq() }

// Lerp linearly interpolates between v and target by t.
func (v Vec2) Lerp(target Vec2, t Q) Vec2 {
	return v.Add(target.Sub(v).Mul(t))
}

// Apply rotates the vector v by r.
func (r Rot) Apply(v Vec2) Vec2 {
	return Vec2{
		X: r.Cos.Mul(v.X).Sub(r.Sin.Mul(v.Y)),
		Y: r.Sin.Mul(v.X).Add(r.Cos.Mul(v.Y)),
	}
}

// Mul composes the rotations: the result rotates by r then by o.
func (r Rot) Mul(o Rot) Rot {
	return Rot{
		Sin: r.Sin.Mul(o.Cos).Add(r.Cos.Mul(o.Sin)),
		Cos: r.Cos.Mul(o.Cos).Sub(r.Sin.Mul(o.Sin)),
	}
}

// Normalize rescales r to unit length. A zero r returns a zero rotation.
func (r Rot) Normalize() Rot {
	mag := r.Sin.Mul(r.Sin).Add(r.Cos.Mul(r.Cos)).Sqrt()
	invMag := QZero()
	if mag.Greater(QZero()) {
		invMag = QOne().Div(mag)
	}
	return Rot{Sin: r.Sin.Mul(invMag), Cos: r.Cos.Mul(invMag)}
}

// recip scales by the reciprocal of a denominator. Float mode rounds the
// reciprocal once and multiplies by it, as the reference does (D-006).
type recip struct{ inv Q }

func makeRecip(d Q) recip { return recip{inv: QOne().Div(d)} }

func (r recip) scale(x Q) Q { return x.Mul(r.inv) }

func (r recip) scaleVec(v Vec2) Vec2 { return v.Mul(r.inv) }

// belowEpsilon reports whether a non-negative x is below the rounding noise
// of the float32 format.
func belowEpsilon(x Q) bool { return x.Less(scalarEpsilon) }

// belowEpsilonSq reports whether a squared length is below float32 rounding
// noise.
func belowEpsilonSq(x Q) bool { return x.Less(scalarEpsilonSq) }

// sensorOverlaps reports whether a sensor overlap distance is below the
// float32 sensor threshold.
func sensorOverlaps(distance Q) bool { return distance.Less(sensorOverlapDistance) }

// IsValidQ reports whether a is finite in the float32 format.
func IsValidQ(a Q) bool {
	return math.Float32bits(a.v)&0x7f800000 != 0x7f800000
}

// qBits returns the bits the checksum folds.
func qBits(q Q) uint64 { return uint64(math.Float32bits(q.v)) }

// QFromFloat64 returns f rounded to the nearest float32. The rounding is the
// same on every architecture, so a constant converts to the same scalar
// everywhere. A float64 computed at run time is only as portable as its
// computation: Go fuses a multiply and an add into one rounding on arm64,
// and on amd64 with GOAMD64=v3.
// A decimal literal rounds to float64 first, so a long literal can differ
// from QMustParse in the last bit.
func QFromFloat64(f float64) Q { return Q{float32(f)} }

// QToFloat64 returns q as a float64. The conversion is exact.
func QToFloat64(q Q) float64 { return float64(q.v) }

// The float mode solves contacts in its one scalar, so the contact types
// are aliases and the conversions do nothing.
type (
	qc    = Q
	qa    = Q
	vec2c = Vec2
	rotc  = Rot
)

func qcFrom(x Q) qc { return x }

func qaFrom(x Q) qa { return x }

func (q Q) toQ() Q { return q }

func (q Q) widen() Q { return q }

func (q Q) narrow() Q { return q }

// rollingBound returns the rolling resistance bound rr·total.
func rollingBound(rr qc, total qa) qc { return rr.Mul(total) }

// Float mode has one grid, so every contact fits the lane and the Q32 path
// is empty. These stubs keep the dispatch one source for both modes.

// contactConstraint32 keeps the color layout available in float mode.
type contactConstraint32 struct{}

// contactFitsLane reports that every contact fits the float lane.
func contactFitsLane(*contactSim) bool { return true }

func allocateContactConstraints32(*world, *stepContext, *[graphColorCount]graphColor) {}

func prepareContacts32(*stepContext) {}

func warmStartContacts32(int, int, *stepContext, int) {}

func solveContacts32(int, int, *stepContext, int, bool) {}

func applyRestitution32(int, int, *stepContext, int) {}

func storeImpulses32(*stepContext) {}
