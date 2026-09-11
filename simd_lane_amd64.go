//go:build dbox2d_simd && !dbox2d_fixed && goexperiment.simd && go1.27 && amd64

package dbox2d

import "simd/archsimd"

const (
	// wideWidth is the number of float32 lanes in the SIMD vector.
	wideWidth = 8
	// wideShift is the base-two logarithm of wideWidth.
	wideShift = 3
)

// laneData is the amd64 float32 vector type.
type laneData = archsimd.Float32x8

// maskData is the amd64 comparison-mask type.
type maskData = archsimd.Mask32x8

// wideAvailable reports whether the required amd64 feature is available.
func wideAvailable() bool { return archsimd.X86.AVX2() }

// widePath reports the selected amd64 SIMD path.
func widePath() string { return "avx2" }

// laneBroadcast fills a vector with one value.
func laneBroadcast(s laneScalar) laneData { return archsimd.BroadcastFloat32x8(s) }

// laneLoadData loads one aligned-width lane-element array.
func laneLoadData(p *[wideWidth]laneScalar) laneData { return archsimd.LoadFloat32x8Array(p) }

// AllZero reports whether no mask lane is set.
func (m maskW) AllZero() bool { return m.v.ToBits() == 0 }
