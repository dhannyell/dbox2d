package dbox2d

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

// colorContactUnits counts the solver units of a color: the family units of
// the fitting contacts plus one unit for the Q32 tail when it is not empty.
func colorContactUnits(color *graphColor) int {
	units := colorContactConstraintCount(len(color.contacts))
	if len(color.contacts32) > 0 {
		units++
	}
	return units
}

// runGraphContactBlock runs a block of color units. The last unit of a
// color with a Q32 tail solves that tail whole; the other units go to the
// selected family.
func runGraphContactBlock(stage *solverStage, context *stepContext, startIndex, endIndex int) {
	color := &context.graph.colors[stage.colorIndex]
	if len(color.contacts32) > 0 && endIndex == colorContactUnits(color) {
		endIndex--
		switch stage.stageType {
		case stageWarmStart:
			warmStartContacts32(context, stage.colorIndex)
		case stageSolve:
			solveContacts32(context, stage.colorIndex, true)
		case stageRelax:
			solveContacts32(context, stage.colorIndex, false)
		case stageRestitution:
			applyRestitution32(context, stage.colorIndex)
		}
	}
	if startIndex < endIndex {
		runGraphContactFamilyBlock(stage, context, startIndex, endIndex)
	}
}

// allocateContactConstraintsScalar preserves the scalar arena layout while
// allowing the wide hook to own its separate scratch allocation.
func allocateContactConstraintsScalar(w *world, context *stepContext, colors *[graphColorCount]graphColor, overflowIndex, activeContactCount int) {
	contactCount := activeContactCount + len(colors[overflowIndex].contacts)
	contactConstraints, constraintMem := arenaSlice[contactConstraint](&w.arena, contactCount, "contact constraint")
	context.contactConstraints = contactConstraints[:activeContactCount:activeContactCount]
	context.contactConstraintMem = constraintMem
	contactBase := 0
	for i := range graphColorCount {
		color := &colors[i]
		colorContactCount := len(color.contacts)
		color.contactConstraints = contactConstraints[contactBase : contactBase+colorContactCount : contactBase+colorContactCount]
		contactBase += colorContactCount
	}
}
