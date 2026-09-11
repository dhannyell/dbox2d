//go:build dbox2d_simd && !dbox2d_fixed && goexperiment.simd && go1.27 && wasm

package dbox2d

import "simd/archsimd"

const (
	// wideWidth is the number of float32 lanes in the SIMD128 vector.
	wideWidth = 4
	// wideShift is the base-two logarithm of wideWidth.
	wideShift = 2
)

// laneData is the wasm SIMD128 float32 vector type.
type laneData = archsimd.Float32x4

// maskData is the wasm SIMD128 comparison-mask type.
type maskData = archsimd.Mask32x4

// wideAvailable reports that the wasm SIMD128 path is available.
func wideAvailable() bool { return true }

// widePath reports the selected wasm SIMD128 path.
func widePath() string { return "simd128" }

// laneBroadcast fills a vector with one value.
func laneBroadcast(s laneScalar) laneData { return archsimd.BroadcastFloat32x4(s) }

// laneLoadData loads one aligned-width lane-element array.
func laneLoadData(p *[wideWidth]laneScalar) laneData { return archsimd.LoadFloat32x4Array(p) }

// The wasm backend has no mask reduction; four lanes are cheap to test.
func (m maskW) AllZero() bool {
	var v [4]int32
	m.v.ToInt32x4().StoreArray(&v)
	return v[0]|v[1]|v[2]|v[3] == 0
}
