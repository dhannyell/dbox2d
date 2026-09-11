//go:build dbox2d_simd && !dbox2d_fixed && goexperiment.simd && go1.27 && arm64

package dbox2d

import "simd/archsimd"

const (
	// wideWidth is the number of float32 lanes in the SIMD vector.
	wideWidth = 4
	// wideShift is the base-two logarithm of wideWidth.
	wideShift = 2
)

// laneData is the arm64 float32 vector type.
type laneData = archsimd.Float32x4

// maskData is the arm64 comparison-mask type.
type maskData = archsimd.Mask32x4

// wideAvailable reports that the arm64 SIMD path is available.
func wideAvailable() bool { return true }

// widePath reports the selected arm64 SIMD path.
func widePath() string { return "neon" }

// laneBroadcast fills a vector with one value.
func laneBroadcast(s laneScalar) laneData { return archsimd.BroadcastFloat32x4(s) }

// laneLoadData loads one aligned-width lane-element array.
func laneLoadData(p *[wideWidth]laneScalar) laneData { return archsimd.LoadFloat32x4Array(p) }

// AllZero reports whether no mask lane is set.
func (m maskW) AllZero() bool { return m.v.ToInt32x4().ReduceMin() == 0 }
