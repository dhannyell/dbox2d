//go:build dbox2d_float

package dbox2d

import (
	"math"
	"testing"
)

// TestStepRejectsANaNTimeStep keeps the float-only invalid scalar case.
func TestStepRejectsANaNTimeStep(t *testing.T) {
	worldId := createTestWorld(t)
	requirePanic(t, func() { worldId.Step(QFromFloat64(math.NaN()), 4) })
}
