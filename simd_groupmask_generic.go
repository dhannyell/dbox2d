//go:build dbox2d_simd && (dbox2d_fixed || !goexperiment.simd || !go1.27 || !amd64)

package b2

// laneGroupMask sets the lanes whose lane group holds a nonzero value of v,
// as laneGroupAny does for the scalar family, and reports whether any lane
// does.
func laneGroupMask(v laneW) (maskW, bool) {
	var values, flags [wideWidth]laneScalar
	v.store(&values)
	var zero laneScalar
	one := laneScalarFromQ(QOne())
	anyLane := false
	for start := 0; start < wideWidth; start += laneGroupWidth {
		group := false
		for j := start; j < start+laneGroupWidth; j++ {
			if values[j] != zero {
				group = true
			}
		}
		if group {
			anyLane = true
			for j := start; j < start+laneGroupWidth; j++ {
				flags[j] = one
			}
		}
	}
	return laneLoad(&flags).Greater(laneZero()), anyLane
}
