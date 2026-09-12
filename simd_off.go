//go:build !dbox2d_simd

package dbox2d

// contactConstraintWide keeps the context layout available in scalar builds.
type contactConstraintWide struct{}

// wideScratch keeps the world layout available in scalar builds.
type wideScratch struct{}

// colorContactConstraintCount keeps scalar stages one contact per solver unit.
func colorContactConstraintCount(contactCount int) int { return contactCount }

// contactStageCount keeps scalar prepare/store work one unit per contact.
func contactStageCount(context *stepContext) int { return len(context.contacts) }

// runContactStageBlock selects scalar prepare/store work in this build.
func runContactStageBlock(stage *solverStage, context *stepContext, startIndex, endIndex int) {
	runContactStageBlockScalar(stage, context, startIndex, endIndex)
}

// runGraphContactFamilyBlock selects the scalar graph-contact family in this build.
func runGraphContactFamilyBlock(stage *solverStage, context *stepContext, startIndex, endIndex int) {
	runGraphContactBlockScalar(stage, context, startIndex, endIndex)
}

// allocateContactConstraints keeps scalar scratch allocation in scalar builds.
func (*wideScratch) allocateContactConstraints(w *world, context *stepContext, colors *[graphColorCount]graphColor, overflowIndex, activeContactCount int) {
	allocateContactConstraintsScalar(w, context, colors, overflowIndex, activeContactCount)
}

// LanePath reports the contact-solver lane path of this build. It is
// "scalar" whenever the wide family is absent, so a caller can record the
// path it actually got rather than the path its build tags asked for.
func LanePath() string { return "scalar" }

// LaneWidth reports the number of contacts one solver unit carries. The
// scalar family carries one.
func LaneWidth() int { return 1 }
