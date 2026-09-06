// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

package samples

import (
	"testing"

	"github.com/dhannyell/dbox2d"

	"github.com/dhannyell/fixed"
)

func TestRandomIntSequence(t *testing.T) {
	randomSeed = randSeed
	want := []int{29818, 2479, 4000, 16874, 10707}
	for i, value := range want {
		if got := randomInt(); got != value {
			t.Fatalf("draw %d: got %d, want %d", i, got, value)
		}
	}
}

func TestRandomFloatRangeBounds(t *testing.T) {
	randomSeed = randSeed
	lo := fixed.Q32MustParse("-2.5")
	hi := fixed.Q32MustParse("3.25")
	for range 1000 {
		got := randomFloatRange(lo, hi)
		if got.Less(lo) || got.Greater(hi) {
			t.Fatalf("got %s outside [%s, %s]", got, lo, hi)
		}
	}
}

// The derived helpers must honour their bounds so scenes stay inside the
// reference layouts.
func TestRandomHelpersStayInRange(t *testing.T) {
	randomSeed = randSeed
	extent := fixed.Q32FromInt(2)
	for range 200 {
		if v := randomIntRange(3, 8); v < 3 || v > 8 {
			t.Fatalf("randomIntRange: got %d outside [3, 8]", v)
		}
		if v := randomFloat(); v.Less(fixed.Q32One().Neg()) || v.Greater(fixed.Q32One()) {
			t.Fatalf("randomFloat: got %s outside [-1, 1]", v)
		}
		p := randomVec2(extent.Neg(), extent)
		if p.X.Less(extent.Neg()) || p.X.Greater(extent) || p.Y.Less(extent.Neg()) || p.Y.Greater(extent) {
			t.Fatalf("randomVec2: got %v outside the extent", p)
		}
		if q := randomRot(); !dbox2d.IsNormalizedRot(q) {
			t.Fatalf("randomRot: got a non-normalized rotation %v", q)
		}
		if poly := randomPolygon(extent); poly.Count < 3 {
			t.Fatalf("randomPolygon: got %d vertices, want at least 3", poly.Count)
		}
	}
}
