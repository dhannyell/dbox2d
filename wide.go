//go:build dbox2d_wide && dbox2d_float

package dbox2d

// contactConstraintWide mirrors b2ContactConstraintSIMD in contact_solver.c
// at lines 1034-1064; it always carries two point slots, with the second zero for a one-point manifold.
type contactConstraintWide struct {
	indexA, indexB                                       [wideWidth]int
	invMassA, invMassB, invIA, invIB                     laneW //nolint:unused // Reserved for the later wide stages.
	normal                                               vec2W //nolint:unused // Reserved for the later wide stages.
	friction, tangentSpeed, rollingResistance            laneW //nolint:unused // Reserved for the later wide stages.
	rollingMass                                          laneW //nolint:unused // Reserved for the later wide stages.
	rollingImpulse                                       accW  //nolint:unused // Reserved for the later wide stages.
	biasRate, massScale, impulseScale                    laneW //nolint:unused // Reserved for the later wide stages.
	anchorA1, anchorB1                                   vec2W //nolint:unused // Reserved for the later wide stages.
	normalMass1, tangentMass1, baseSeparation1           laneW //nolint:unused // Reserved for the later wide stages.
	normalImpulse1, totalNormalImpulse1, tangentImpulse1 accW  //nolint:unused // Reserved for the later wide stages.
	anchorA2, anchorB2                                   vec2W //nolint:unused // Reserved for the later wide stages.
	baseSeparation2                                      laneW //nolint:unused // Reserved for the later wide stages.
	normalImpulse2, totalNormalImpulse2, tangentImpulse2 accW  //nolint:unused // Reserved for the later wide stages.
	normalMass2, tangentMass2                            laneW //nolint:unused // Reserved for the later wide stages.
	restitution                                          laneW //nolint:unused // Reserved for the later wide stages.
	relativeVelocity1, relativeVelocity2                 laneW //nolint:unused // Reserved for the later wide stages.
}

// bodyStateW keeps gathered state in lane form for the future wide stages.
type bodyStateW struct {
	v     vec2W
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
	return bodyStateW{
		v:     vec2W{x: laneLoad(&vx), y: laneLoad(&vy)},
		w:     laneLoad(&w),
		flags: laneLoad(&flags),
		dp:    vec2W{x: laneLoad(&dpx), y: laneLoad(&dpy)},
		dq:    rotW{c: laneLoad(&dqc), s: laneLoad(&dqs)},
	}
}

// scatterBodies writes only real-body velocities back in turns per second.
func scatterBodies(states []bodyState, indices *[wideWidth]int, b *bodyStateW) {
	var vx, vy, w [wideWidth]float32
	b.v.x.store(&vx)
	b.v.y.store(&vy)
	b.w.store(&w)
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

// wideStagesReady stays false until the wide solve stages are implemented.
const wideStagesReady = false

// wideEnabled gates the whole wide contact family behind one switch.
var wideEnabled = wideAvailable() && wideStagesReady

// wideConstraintCount rounds contacts up to the number of wide constraints.
func wideConstraintCount(contactCount int) int {
	if contactCount <= 0 {
		return 0
	}
	return ((contactCount - 1) >> wideShift) + 1
}

// packWideColor copies colored contact body indices into wide constraint lanes.
func packWideColor(constraints []contactConstraintWide, contacts []*contactSim) {
	for i := range constraints {
		c := &constraints[i]
		for j := range wideWidth {
			k := wideWidth*i + j
			if k < len(contacts) {
				c.indexA[j] = contacts[k].bodySimIndexA
				c.indexB[j] = contacts[k].bodySimIndexB
			} else {
				c.indexA[j] = nullIndex
				c.indexB[j] = nullIndex
			}
		}
	}
}

// colorContactConstraintCount counts scalar contacts or rounded wide units.
func colorContactConstraintCount(contactCount int) int {
	if wideEnabled {
		return wideConstraintCount(contactCount)
	}
	return contactCount
}

// runGraphContactBlock dispatches graph contacts to the selected family.
func runGraphContactBlock(stage *solverStage, context *stepContext, startIndex, endIndex int) {
	if wideEnabled {
		panic("dbox2d: wide contact stages are not available")
	}
	runGraphContactBlockScalar(stage, context, startIndex, endIndex)
}

// allocateContactConstraints reserves scalar or wide contact scratch.
func allocateContactConstraints(w *world, context *stepContext, colors *[graphColorCount]graphColor, overflowIndex, activeContactCount int) {
	if !wideEnabled {
		allocateContactConstraintsScalar(w, context, colors, overflowIndex, activeContactCount)
		return
	}

	wideCount := 0
	for i := range overflowIndex {
		wideCount += wideConstraintCount(len(colors[i].contactSims))
	}
	constraintsWide, memWide := arenaSlice[contactConstraintWide](&w.arena, wideCount, "wide contact constraint")
	context.contactConstraintsWide = constraintsWide
	context.contactConstraintMemWide = memWide

	contactBase := 0
	wideBase := 0
	for i := range overflowIndex {
		color := &colors[i]
		colorContactCount := len(color.contactSims)
		colorWideCount := wideConstraintCount(colorContactCount)
		color.contactConstraintsWide = constraintsWide[wideBase : wideBase+colorWideCount : wideBase+colorWideCount]
		packWideColor(color.contactConstraintsWide, context.contacts[contactBase:contactBase+colorContactCount])
		wideBase += colorWideCount
		contactBase += colorContactCount
	}

	overflowColor := &colors[overflowIndex]
	overflowCount := len(overflowColor.contactSims)
	overflowConstraints, mem := arenaSlice[contactConstraint](&w.arena, overflowCount, "contact constraint")
	overflowColor.contactConstraints = overflowConstraints
	context.contactConstraintMem = mem
}
