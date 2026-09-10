//go:build !dbox2d_validate

package dbox2d

// validateSolverSetsDebug is a no-op in release builds, matching
// b2ValidateSolverSets in the reference when B2_VALIDATE is disabled.
//
// The full validation walks every body, joint, and contact, so calling it after
// each joint creation can make large scenes unnecessarily expensive.
//
// Tests call validateSolverSets directly, so validation is still covered
// regardless of the build setting.
func validateSolverSetsDebug(*world) {}

// validateTreeDebug is a no-op in release builds, matching
// b2DynamicTree_Validate in the reference when B2_VALIDATE is disabled.
func validateTreeDebug(*dynamicTree) {}

// validateTreeNoEnlargedDebug is a no-op in release builds, matching
// b2DynamicTree_ValidateNoEnlarged when B2_VALIDATE is disabled.
func validateTreeNoEnlargedDebug(*dynamicTree) {}
