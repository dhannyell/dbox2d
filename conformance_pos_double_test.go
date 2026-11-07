//go:build dbox2d_double && !dbox2d_fixed

package b2_test

import (
	"math"
	"strconv"
	"testing"

	. "github.com/dhannyell/dbox2d"
)

// conformanceSceneRoot holds the scene traces this build compares against.
// Large-world scenes record positions as doubles; the function traces do not
// involve positions and are shared with the default corpus.
const conformanceSceneRoot = "testdata/conformance_double"

// conformanceTracePos is one traced position coordinate: the double bits.
type conformanceTracePos struct {
	bits  uint64
	value float64
}

func parseConformancePos(t *testing.T, token string) conformanceTracePos {
	t.Helper()
	if len(token) != 16 {
		t.Fatalf("conformance position %q is not 16 hexadecimal digits", token)
	}
	bits, err := strconv.ParseUint(token, 16, 64)
	if err != nil {
		t.Fatalf("invalid conformance position %q: %v", token, err)
	}
	return conformanceTracePos{bits: bits, value: math.Float64frombits(bits)}
}

// positionSample returns a position coordinate pair with its bit patterns.
func positionSample(p Pos) (x, y float64, xBits, yBits uint64) {
	return p.X, p.Y, math.Float64bits(p.X), math.Float64bits(p.Y)
}

// foldConformancePos folds a position into the scene hash like the oracle.
func foldConformancePos(hash uint64, p Pos) uint64 {
	hash = FNVFold(hash, math.Float64bits(p.X))
	return FNVFold(hash, math.Float64bits(p.Y))
}
