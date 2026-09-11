//go:build dbox2d_simd

package dbox2d

import (
	"slices"
	"sync/atomic"
)

// contactConstraintWide mirrors b2ContactConstraintSIMD in contact_solver.c
// at lines 1034-1064; it always carries two point slots, with the second zero for a one-point manifold.
type contactConstraintWide struct {
	indexA, indexB [wideWidth]int
	// hasRolling is false when no lane uses rolling resistance or holds a
	// rolling impulse. The solver then skips the rolling blocks, which would
	// only add zero.
	hasRolling                                bool
	invMassA, invMassB, invIA, invIB          laneW
	normal                                    vec2W
	friction, tangentSpeed, rollingResistance laneW
	rollingMass                               laneW
	// The warm-started impulses stay on the lane grid, since every stage uses
	// them in lane form. Only the totals need the accumulator range.
	rollingImpulse                             laneW
	biasRate, massScale, impulseScale          laneW
	anchorA1, anchorB1                         vec2W
	normalMass1, tangentMass1, baseSeparation1 laneW
	normalImpulse1, tangentImpulse1            laneW
	totalNormalImpulse1                        accW
	anchorA2, anchorB2                         vec2W
	baseSeparation2                            laneW
	normalImpulse2, tangentImpulse2            laneW
	totalNormalImpulse2                        accW
	normalMass2, tangentMass2                  laneW
	restitution                                laneW
	relativeVelocity1, relativeVelocity2       laneW
}

// bodyStateW keeps gathered state in lane form for the wide stages.
type bodyStateW struct {
	v struct{ x, y accW }
	w accW
	// flags travel with the row so the amd64 scatter can store whole rows.
	flags laneW
	dp    vec2W
	dq    rotW
}

// wideEnabled gates the whole wide contact family behind one switch.
var wideEnabled = wideAvailable()

// wideContactAllocations records steps that built non-empty wide scratch.
var wideContactAllocations atomic.Uint64

// wideConstraintCount rounds contacts up to the number of wide constraints.
func wideConstraintCount(contactCount int) int {
	if contactCount <= 0 {
		return 0
	}
	return ((contactCount - 1) >> wideShift) + 1
}

// colorContactConstraintCount counts scalar contacts or rounded wide units.
func colorContactConstraintCount(contactCount int) int {
	if wideEnabled {
		return wideConstraintCount(contactCount)
	}
	return contactCount
}

// contactStageCount returns the prepare/store work units of the selected family.
func contactStageCount(context *stepContext) int {
	if wideEnabled {
		return len(context.contactConstraintsWide)
	}
	return len(context.contacts)
}

// runContactStageBlock dispatches flat prepare/store work to the selected family.
func runContactStageBlock(stage *solverStage, context *stepContext, startIndex, endIndex int) {
	if !wideEnabled {
		runContactStageBlockScalar(stage, context, startIndex, endIndex)
		return
	}
	switch stage.stageType {
	case stagePrepareContacts:
		prepareContactsTaskWide(startIndex, endIndex, context)
	case stageStoreImpulses:
		storeImpulsesTaskWide(startIndex, endIndex, context)
	}
}

// runGraphContactFamilyBlock dispatches graph contacts to the selected family.
func runGraphContactFamilyBlock(stage *solverStage, context *stepContext, startIndex, endIndex int) {
	if !wideEnabled {
		runGraphContactBlockScalar(stage, context, startIndex, endIndex)
		return
	}
	switch stage.stageType {
	case stageWarmStart:
		warmStartContactsTaskWide(startIndex, endIndex, context, stage.colorIndex)
	case stageSolve:
		solveContactsTaskWide(startIndex, endIndex, context, stage.colorIndex, true)
	case stageRelax:
		solveContactsTaskWide(startIndex, endIndex, context, stage.colorIndex, false)
	case stageRestitution:
		applyRestitutionTaskWide(startIndex, endIndex, context, stage.colorIndex)
	}
}

// allocateContactConstraints reserves scalar or wide contact scratch.
func allocateContactConstraints(w *world, context *stepContext, colors *[graphColorCount]graphColor, overflowIndex, activeContactCount int) {
	if !wideEnabled {
		allocateContactConstraintsScalar(w, context, colors, overflowIndex, activeContactCount)
		return
	}

	wideCount := 0
	realContactCount := 0
	for i := range overflowIndex {
		contactCount := len(colors[i].contacts)
		wideCount += wideConstraintCount(contactCount)
		realContactCount += contactCount
	}
	if realContactCount != activeContactCount {
		panic("dbox2d: the active contact count is inconsistent")
	}

	// The padded list lives apart from w.contactPointers, which the color
	// slices still point into.
	paddedContactCount := wideWidth * wideCount
	w.contactPointersWide = slices.Grow(w.contactPointersWide[:0], paddedContactCount)[:paddedContactCount]
	context.contacts = w.contactPointersWide

	constraintsWide, memWide := arenaSlice[contactConstraintWide](&w.arena, wideCount, "wide contact constraint")
	context.contactConstraintsWide = constraintsWide
	context.contactConstraintMemWide = memWide

	wideBase := 0
	for i := range overflowIndex {
		color := &colors[i]
		colorContactCount := len(color.contacts)
		colorWideCount := wideConstraintCount(colorContactCount)
		color.contactConstraintsWide = constraintsWide[wideBase : wideBase+colorWideCount : wideBase+colorWideCount]

		paddedBase := wideWidth * wideBase
		paddedColorCount := wideWidth * colorWideCount
		copy(context.contacts[paddedBase:], color.contacts)
		for j := colorContactCount; j < paddedColorCount; j++ {
			context.contacts[paddedBase+j] = nil
		}

		wideBase += colorWideCount
	}
	if wideBase != wideCount {
		panic("dbox2d: the wide contact layout is incomplete")
	}
	if wideCount > 0 {
		wideContactAllocations.Add(1)
	}

	overflowColor := &colors[overflowIndex]
	overflowCount := len(overflowColor.contacts)
	overflowConstraints, mem := arenaSlice[contactConstraint](&w.arena, overflowCount, "contact constraint")
	overflowColor.contactConstraints = overflowConstraints
	context.contactConstraintMem = mem
}
