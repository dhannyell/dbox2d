//go:build amd64 || arm64

package dbox2d

// cpuPause is one idle turn of a spin loop: PAUSE on amd64, YIELD on arm64,
// nothing elsewhere.
func cpuPause()
