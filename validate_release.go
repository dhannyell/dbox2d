//go:build !dbox2d_validate

package dbox2d

// validateSolverSetsDebug is a no-op in this build, which mirrors
// b2ValidateSolverSets in src/world.c at line 3262: the reference compiles
// the empty body unless B2_VALIDATE is set, and B2_VALIDATE is off in a
// release build.
//
// The cost is not incidental. The sweep walks every body, joint and contact
// in the world, and createJoint calls it once per joint, so a scene that
// builds joints against a large world pays it quadratically. Leaving it on
// made the rain benchmark's scene hook 290 times slower than the same hook
// in the reference; the solver was unaffected, because the step path never
// calls it.
//
// The tests call validateSolverSets directly rather than through this
// wrapper, so they keep validating at either tag setting.
func validateSolverSetsDebug(*world) {}
