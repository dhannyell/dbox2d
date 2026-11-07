//go:build dbox2d_double && !dbox2d_fixed

package b2

// posFloats reads a position as two float64 values; no narrowing happens.
func posFloats(p Pos) []float64 { return []float64{p.X, p.Y} }
