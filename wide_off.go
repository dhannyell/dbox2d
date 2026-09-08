//go:build !dbox2d_simd || !dbox2d_float

package dbox2d

// contactConstraintWide keeps the context layout available in scalar builds.
type contactConstraintWide struct{}

// colorContactConstraintCount keeps scalar stages one contact per solver unit.
func colorContactConstraintCount(contactCount int) int { return contactCount }

// contactStageCount keeps scalar prepare/store work one unit per contact.
func contactStageCount(context *stepContext) int { return len(context.contacts) }

// runContactStageBlock selects scalar prepare/store work in this build.
func runContactStageBlock(stage *solverStage, context *stepContext, startIndex, endIndex int) {
	runContactStageBlockScalar(stage, context, startIndex, endIndex)
}

// runGraphContactBlock selects the scalar graph-contact family in this build.
func runGraphContactBlock(stage *solverStage, context *stepContext, startIndex, endIndex int) {
	runGraphContactBlockScalar(stage, context, startIndex, endIndex)
}

// allocateContactConstraints keeps scalar scratch allocation in scalar builds.
func allocateContactConstraints(w *world, context *stepContext, colors *[graphColorCount]graphColor, overflowIndex, activeContactCount int) {
	allocateContactConstraintsScalar(w, context, colors, overflowIndex, activeContactCount)
}
