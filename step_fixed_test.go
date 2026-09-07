//go:build !dbox2d_float

package dbox2d

import "testing"

// TestStepRejectsASaturatedTimeStep keeps the fixed-only saturation case.
func TestStepRejectsASaturatedTimeStep(t *testing.T) {
	worldId := createTestWorld(t)
	requirePanic(t, func() { worldId.Step(QMaxValue(), 4) })
}
