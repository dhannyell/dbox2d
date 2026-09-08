//go:build !dbox2d_wide

package dbox2d

// contactConstraintWide keeps the context layout available in scalar builds.
type contactConstraintWide struct{}

// colorContactConstraintCount keeps scalar stages one contact per solver unit.
func colorContactConstraintCount(contactCount int) int { return contactCount }

// runGraphContactBlock selects the scalar graph-contact family in this build.
func runGraphContactBlock(stage *solverStage, context *stepContext, startIndex, endIndex int) {
	runGraphContactBlockScalar(stage, context, startIndex, endIndex)
}

// allocateContactConstraints keeps scalar scratch allocation in scalar builds.
func allocateContactConstraints(w *world, context *stepContext, colors *[graphColorCount]graphColor, overflowIndex, activeContactCount int) {
	allocateContactConstraintsScalar(w, context, colors, overflowIndex, activeContactCount)
}
