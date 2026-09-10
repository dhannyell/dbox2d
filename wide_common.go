package dbox2d

import "slices"

// runContactStageBlockScalar keeps flat prepare/store work behind the family hook.
func runContactStageBlockScalar(stage *solverStage, context *stepContext, startIndex, endIndex int) {
	switch stage.stageType {
	case stagePrepareContacts:
		prepareContactsTask(startIndex, endIndex, context)
	case stageStoreImpulses:
		storeImpulsesTask(startIndex, endIndex, context)
	}
}

// runGraphContactBlockScalar keeps the existing scalar graph-contact stages
// behind one hook so the wide family can replace them as a unit.
func runGraphContactBlockScalar(stage *solverStage, context *stepContext, startIndex, endIndex int) {
	switch stage.stageType {
	case stageWarmStart:
		warmStartContactsTask(startIndex, endIndex, context, stage.colorIndex)
	case stageSolve:
		solveContactsTask(startIndex, endIndex, context, stage.colorIndex, true)
	case stageRelax:
		solveContactsTask(startIndex, endIndex, context, stage.colorIndex, false)
	case stageRestitution:
		applyRestitutionTask(startIndex, endIndex, context, stage.colorIndex)
	}
}

// allocateContactConstraintsScalar preserves the scalar arena layout while
// allowing the wide hook to own its separate scratch allocation.
func allocateContactConstraintsScalar(w *world, context *stepContext, colors *[graphColorCount]graphColor, overflowIndex, activeContactCount int) {
	contactCount := activeContactCount + len(colors[overflowIndex].contactSims)
	contactConstraints, constraintMem := arenaSlice[contactConstraint](&w.arena, contactCount, "contact constraint")
	context.contactConstraints = contactConstraints[:activeContactCount:activeContactCount]
	context.contactConstraintMem = constraintMem

	//  Only the non overflow colors reach the parallel-for, so only they are gathered.
	w.contactPointers = slices.Grow(w.contactPointers[:0], activeContactCount)[:activeContactCount]
	context.contacts = w.contactPointers

	contactBase := 0
	for i := range graphColorCount {
		color := &colors[i]
		colorContactCount := len(color.contactSims)
		color.contactConstraints = contactConstraints[contactBase : contactBase+colorContactCount : contactBase+colorContactCount]
		if i < overflowIndex {
			for j := range colorContactCount {
				context.contacts[contactBase+j] = &color.contactSims[j]
			}
		}
		contactBase += colorContactCount
	}
}
