//go:build !dbox2d_double || dbox2d_fixed

package b2

// posFloats reads a position as two float64 values.
func posFloats(p Pos) []float64 { return []float64{QToFloat64(p.X), QToFloat64(p.Y)} }
