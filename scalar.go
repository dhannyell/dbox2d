package dbox2d

// Scalar is what the short constructors accept: a Go integer or float, for
// a literal or a value the caller computed, and Q itself, so a value that is
// already on the scalar grid passes through unchanged.
type Scalar interface {
	int | int32 | int64 | float32 | float64 | Q
}

// F returns v as a scalar of the build mode. An integer converts exactly and
// a float rounds once, as QFromInt and QFromFloat64 do, so a constant gives
// the same bits on every architecture. A float computed at run time is only
// as portable as its computation; see QFromFloat64.
func F[T Scalar](v T) Q {
	switch x := any(v).(type) {
	case Q:
		return x
	case int:
		return QFromInt(x)
	case int32:
		return QFromInt(int(x))
	case int64:
		return QFromInt(int(x))
	case float32:
		return QFromFloat64(float64(x))
	case float64:
		return QFromFloat64(x)
	}
	panic("dbox2d: unreachable")
}

// V2 returns the vector (x, y); both convert as F does.
func V2[T Scalar](x, y T) Vec2 {
	return Vec2{X: F(x), Y: F(y)}
}

// Degrees returns an angle given in degrees as the turns the API uses. An
// integer converts as the exact ratio over 360; a float divides in float64
// first, which is one rounding for a constant.
func Degrees[T Scalar](v T) Q {
	switch x := any(v).(type) {
	case Q:
		return x.Div(QFromInt(360))
	case int:
		return QFromRatio(x, 360)
	case int32:
		return QFromRatio(int(x), 360)
	case int64:
		return QFromRatio(int(x), 360)
	case float32:
		return QFromFloat64(float64(x) / 360)
	case float64:
		return QFromFloat64(x / 360)
	}
	panic("dbox2d: unreachable")
}
