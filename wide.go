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
	v  struct{ x, y accW }
	w  accW
	dp vec2W
	dq rotW
}

// bodyScratchW carries the gathered scalars between the body states and the lanes.
type bodyScratchW struct{ vx, vy, w, dpx, dpy, dqc, dqs [wideWidth]float32 }

// gatherBodies fills every lane of the scratch: real bodies from the states, null lanes as the identity.
func gatherBodies(states []bodyState, indices *[wideWidth]int, s *bodyScratchW) {
	for j := range wideWidth {
		idx := indices[j]
		if idx == nullIndex {
			s.vx[j], s.vy[j], s.w[j], s.dpx[j], s.dpy[j], s.dqs[j] = 0, 0, 0, 0, 0, 0
			s.dqc[j] = 1
			continue
		}
		b := &states[idx]
		s.vx[j] = b.linearVelocity.X.v
		s.vy[j] = b.linearVelocity.Y.v
		s.w[j] = b.angularVelocity.v
		s.dpx[j] = b.deltaPosition.X.v
		s.dpy[j] = b.deltaPosition.Y.v
		s.dqc[j] = b.deltaRotation.Cos.v
		s.dqs[j] = b.deltaRotation.Sin.v
	}
}

// loadBodyVelocityW builds the velocity lanes; the angular velocity becomes radians per second.
func loadBodyVelocityW(s *bodyScratchW, tauW laneW, b *bodyStateW) {
	b.v.x = laneLoad(&s.vx).toAcc()
	b.v.y = laneLoad(&s.vy).toAcc()
	b.w = laneLoad(&s.w).Mul(tauW).toAcc()
}

// loadBodyDeltaW builds the delta position and delta rotation lanes.
func loadBodyDeltaW(s *bodyScratchW, b *bodyStateW) {
	b.dp = vec2W{x: laneLoad(&s.dpx), y: laneLoad(&s.dpy)}
	b.dq = rotW{c: laneLoad(&s.dqc), s: laneLoad(&s.dqs)}
}

// storeBodyW writes the velocities to the scratch, the angular velocity back in turns per second.
func storeBodyW(b *bodyStateW, tauW laneW, s *bodyScratchW) {
	b.v.x.toLane().store(&s.vx)
	b.v.y.toLane().store(&s.vy)
	b.w.toLane().Div(tauW).store(&s.w)
}

// scatterBodies writes the real-body velocities back from the scratch.
func scatterBodies(states []bodyState, indices *[wideWidth]int, s *bodyScratchW) {
	for j := range wideWidth {
		idx := indices[j]
		if idx == nullIndex {
			continue
		}
		b := &states[idx]
		b.linearVelocity.X.v = s.vx[j]
		b.linearVelocity.Y.v = s.vy[j]
		b.angularVelocity.v = s.w[j]
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
