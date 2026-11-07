//go:build !dbox2d_double || dbox2d_fixed

package b2_test

import (
	"testing"

	. "github.com/dhannyell/dbox2d"
)

// conformanceSceneRoot holds the scene traces this build compares against.
const conformanceSceneRoot = "testdata/conformance"

// conformanceTracePos is one traced position coordinate; without the large
// world tag a trace stores it as a float.
type conformanceTracePos struct {
	bits  uint64
	value float64
}

func parseConformancePos(t *testing.T, token string) conformanceTracePos {
	t.Helper()
	f := parseConformanceFloat(t, token)
	return conformanceTracePos{bits: uint64(f.bits), value: float64(f.value)}
}

// positionSample returns a position coordinate pair with its bit patterns.
func positionSample(p Pos) (x, y float64, xBits, yBits uint64) {
	return QToFloat64(p.X), QToFloat64(p.Y), qBits(p.X), qBits(p.Y)
}

// foldConformancePos folds a position into the scene hash like the oracle.
func foldConformancePos(hash uint64, p Pos) uint64 {
	hash = FNVFold(hash, qBits(p.X))
	return FNVFold(hash, qBits(p.Y))
}
