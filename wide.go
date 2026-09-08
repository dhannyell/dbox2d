//go:build dbox2d_wide && dbox2d_float

package dbox2d

import (
	"slices"
	"sync/atomic"
)

// contactConstraintWide mirrors b2ContactConstraintSIMD in contact_solver.c
// at lines 1034-1064; it always carries two point slots, with the second zero for a one-point manifold.
type contactConstraintWide struct {
	indexA, indexB                                       [wideWidth]int
	invMassA, invMassB, invIA, invIB                     laneW
	normal                                               vec2W
	friction, tangentSpeed, rollingResistance            laneW
	rollingMass                                          laneW
	rollingImpulse                                       accW
	biasRate, massScale, impulseScale                    laneW
	anchorA1, anchorB1                                   vec2W
	normalMass1, tangentMass1, baseSeparation1           laneW
	normalImpulse1, totalNormalImpulse1, tangentImpulse1 accW
	anchorA2, anchorB2                                   vec2W
	baseSeparation2                                      laneW
	normalImpulse2, totalNormalImpulse2, tangentImpulse2 accW
	normalMass2, tangentMass2                            laneW
	restitution                                          laneW
	relativeVelocity1, relativeVelocity2                 laneW
}

// bodyStateW keeps gathered state in lane form for the wide stages.
type bodyStateW struct {
	v     struct{ x, y accW }
	w     accW
	flags laneW
	dp    vec2W
	dq    rotW
}

// gatherBodies loads body states and substitutes identity for null bodies.
func gatherBodies(states []bodyState, indices *[wideWidth]int) bodyStateW {
	var vx, vy, w, flags, dpx, dpy, dqc, dqs [wideWidth]float32
	for j := range wideWidth {
		idx := indices[j]
		if idx == nullIndex {
			dqc[j] = 1
			continue
		}
		s := &states[idx]
		vx[j] = s.linearVelocity.X.v
		vy[j] = s.linearVelocity.Y.v
		w[j] = s.angularVelocity.Mul(tau).v
		flags[j] = float32(s.flags)
		dpx[j] = s.deltaPosition.X.v
		dpy[j] = s.deltaPosition.Y.v
		dqc[j] = s.deltaRotation.Cos.v
		dqs[j] = s.deltaRotation.Sin.v
	}
	var body bodyStateW
	body.v.x = laneLoad(&vx).toAcc()
	body.v.y = laneLoad(&vy).toAcc()
	body.w = laneLoad(&w).toAcc()
	body.flags = laneLoad(&flags)
	body.dp = vec2W{x: laneLoad(&dpx), y: laneLoad(&dpy)}
	body.dq = rotW{c: laneLoad(&dqc), s: laneLoad(&dqs)}
	return body
}

// scatterBodies writes only real-body velocities back in turns per second.
func scatterBodies(states []bodyState, indices *[wideWidth]int, b *bodyStateW) {
	var vx, vy, w [wideWidth]float32
	b.v.x.toLane().store(&vx)
	b.v.y.toLane().store(&vy)
	b.w.toLane().store(&w)
	for j := range wideWidth {
		idx := indices[j]
		if idx == nullIndex {
			continue
		}
		s := &states[idx]
		s.linearVelocity.X.v = vx[j]
		s.linearVelocity.Y.v = vy[j]
		s.angularVelocity = Q{v: w[j]}.Div(tau)
	}
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

// runGraphContactBlock dispatches graph contacts to the selected family.
func runGraphContactBlock(stage *solverStage, context *stepContext, startIndex, endIndex int) {
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
		contactCount := len(colors[i].contactSims)
		wideCount += wideConstraintCount(contactCount)
		realContactCount += contactCount
	}
	if realContactCount != activeContactCount {
		panic("dbox2d: the active contact count is inconsistent")
	}

	paddedContactCount := wideWidth * wideCount
	w.contactPointers = slices.Grow(w.contactPointers[:0], paddedContactCount)[:paddedContactCount]
	context.contacts = w.contactPointers

	constraintsWide, memWide := arenaSlice[contactConstraintWide](&w.arena, wideCount, "wide contact constraint")
	context.contactConstraintsWide = constraintsWide
	context.contactConstraintMemWide = memWide

	wideBase := 0
	for i := range overflowIndex {
		color := &colors[i]
		colorContactCount := len(color.contactSims)
		colorWideCount := wideConstraintCount(colorContactCount)
		color.contactConstraintsWide = constraintsWide[wideBase : wideBase+colorWideCount : wideBase+colorWideCount]

		paddedBase := wideWidth * wideBase
		paddedColorCount := wideWidth * colorWideCount
		for j := range colorContactCount {
			context.contacts[paddedBase+j] = &color.contactSims[j]
		}
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
	overflowCount := len(overflowColor.contactSims)
	overflowConstraints, mem := arenaSlice[contactConstraint](&w.arena, overflowCount, "contact constraint")
	overflowColor.contactConstraints = overflowConstraints
	context.contactConstraintMem = mem
}
