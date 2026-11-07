//go:build dbox2d_simd

package b2

import (
	"slices"
	"sync/atomic"
)

// The wide family runs the colored stages of src/contact_solver_wide.inl on
// lanes. Each lane rounds the values of one constraint in the order of the
// scalar colored stages of contact_solver.go, which are its oracle.

// dotW computes a lane-wise dot product in scalar Vec2.Dot order. It
// corresponds to b2DotW in src/contact_solver_wide.inl.
func dotW(a, b vec2W) laneW {
	return a.x.Mul(b.x).Add(a.y.Mul(b.y))
}

// crossW computes a lane-wise cross product in scalar Cross order. It
// corresponds to b2CrossW in src/contact_solver_wide.inl.
func crossW(a, b vec2W) laneW {
	return a.x.Mul(b.y).Sub(a.y.Mul(b.x))
}

// rotateVectorW rotates lane-wise vectors in scalar Rot.Apply order. It
// corresponds to b2RotateVectorW in src/contact_solver_wide.inl.
func rotateVectorW(q rotW, v vec2W) vec2W {
	return vec2W{
		x: q.c.Mul(v.x).Sub(q.s.Mul(v.y)),
		y: q.s.Mul(v.x).Add(q.c.Mul(v.y)),
	}
}

// laneClamp clamps each lane like the scalar Clamp: below lo gives lo,
// above hi gives hi.
func laneClamp(a, lo, hi laneW) laneW {
	return laneBlend(lo.Greater(a), lo, laneBlend(a.Greater(hi), hi, a))
}

// laneGroupMask sets the lanes whose lane group holds a nonzero value of v,
// as laneGroupAny does for the scalar family, and reports whether any lane
// does.
func laneGroupMask(v laneW) (maskW, bool) {
	var values, flags [wideWidth]laneScalar
	v.store(&values)
	var zero laneScalar
	one := laneScalarFromQ(QOne())
	anyLane := false
	for start := 0; start < wideWidth; start += laneGroupWidth {
		group := false
		for j := start; j < start+laneGroupWidth; j++ {
			if values[j] != zero {
				group = true
			}
		}
		if group {
			anyLane = true
			for j := start; j < start+laneGroupWidth; j++ {
				flags[j] = one
			}
		}
	}
	return laneLoad(&flags).Greater(laneZero()), anyLane
}

// maskIndices returns indices with nullIndex in the lanes the mask clears.
func maskIndices(indices *[wideWidth]int, m maskW) [wideWidth]int {
	var flags [wideWidth]laneScalar
	laneBlend(m, laneSplat(QOne()), laneZero()).store(&flags)
	var zero laneScalar
	out := *indices
	for j := range wideWidth {
		if flags[j] == zero {
			out[j] = nullIndex
		}
	}
	return out
}

// subImpulseW applies the impulse (px, py) with the angular impulse to the
// first bodies of a constraint, as contactBody.subImpulse.
func (b *bodyStateW) subImpulseW(invMass, invI, px, py, angular laneW) {
	b.v.x = b.v.x.SubBounded(invMass.Mul(px).toAcc())
	b.v.y = b.v.y.SubBounded(invMass.Mul(py).toAcc())
	b.w = b.w.SubBounded(invI.Mul(angular).toAcc())
}

// addImpulseW applies the impulse (px, py) with the angular impulse to the
// second bodies of a constraint, as contactBody.addImpulse.
func (b *bodyStateW) addImpulseW(invMass, invI, px, py, angular laneW) {
	b.v.x = b.v.x.AddBounded(invMass.Mul(px).toAcc())
	b.v.y = b.v.y.AddBounded(invMass.Mul(py).toAcc())
	b.w = b.w.AddBounded(invI.Mul(angular).toAcc())
}

// separationW returns the current separation of a point.
func separationW(bodyA, bodyB *bodyStateW, dp vec2W, normal vec2W, cp *contactPointWide) laneW {
	rsA := rotateVectorW(bodyA.dq, cp.anchorA)
	rsB := rotateVectorW(bodyB.dq, cp.anchorB)
	ds := vec2W{x: dp.x.Add(rsB.x.Sub(rsA.x)), y: dp.y.Add(rsB.y.Sub(rsA.y))}
	return dotW(normal, ds).Add(cp.baseSeparation)
}

// prepareContactsTaskWide prepares scalar lanes before loading the SoA. It
// corresponds to b2PrepareContacts_Wide in src/contact_solver_wide.inl.
func prepareContactsTaskWide(startIndex, endIndex int, context *stepContext) {
	w := context.world
	awakeStates := context.states
	contactSoftness := context.contactSoftness
	staticSoftness := context.staticSoftness

	zero := QZero()
	one := QOne()
	warmStartScale := zero
	if w.enableWarmStarting {
		warmStartScale = one
	}
	anyRestitution := false

	for i := startIndex; i < endIndex; i++ {
		var (
			invMassA, invMassB, invIA, invIB                       [wideWidth]laneScalar
			normalX, normalY                                       [wideWidth]laneScalar
			friction, tangentSpeed, restitution, rollingResistance [wideWidth]laneScalar
			rollingMass, rollingImpulse                            [wideWidth]laneScalar
			pushRate, massScale, impulseScale                      [wideWidth]laneScalar
			anchorAX, anchorAY, anchorBX, anchorBY                 [2][wideWidth]laneScalar
			normalMass, tangentMass, baseSeparation                [2][wideWidth]laneScalar
			normalImpulse, tangentImpulse, relativeVelocity        [2][wideWidth]laneScalar
		)

		constraint := &context.contactConstraintsWide[i]
		// Float lanes never skip: the skipped blocks would store +0 where
		// they write -0, and the checksum reads the sign bit.
		anyRolling := signedZeroSurvives
		for j := range wideWidth {
			constraint.indexA[j] = nullIndex
			constraint.indexB[j] = nullIndex
			constraint.storeA[j] = nullIndex
			constraint.storeB[j] = nullIndex

			contact := context.contacts[wideWidth*i+j]
			if contact == nil {
				continue
			}

			manifold := &contact.manifold
			pointCount := manifold.PointCount
			if pointCount <= 0 || pointCount > 2 {
				panic("dbox2d: the manifold point count is out of range")
			}

			indexA := contact.bodySimIndexA
			indexB := contact.bodySimIndexB
			constraint.indexA[j] = indexA
			constraint.indexB[j] = indexB
			// Only a dynamic body takes the velocity back, as
			// contactBody.store.
			if indexA != nullIndex && awakeStates[indexA].flags&bodyDynamicFlag != 0 {
				constraint.storeA[j] = indexA
			}
			if indexB != nullIndex && awakeStates[indexB].flags&bodyDynamicFlag != 0 {
				constraint.storeB[j] = indexB
			}

			// Only sample the normal velocity when restitution or a hit
			// event needs it.
			haveRestitution := zero.Less(contact.restitution)
			sampleVelocity := haveRestitution || contact.simFlags&simEnableHitEvent != 0
			anyRestitution = anyRestitution || haveRestitution

			vA := Vec2Zero()
			wA := zero
			mA := contact.invMassA
			iA := contact.invIA
			if sampleVelocity && indexA != nullIndex {
				stateA := &awakeStates[indexA]
				vA = stateA.linearVelocity
				wA = stateA.angularVelocity
			}

			vB := Vec2Zero()
			wB := zero
			mB := contact.invMassB
			iB := contact.invIB
			if sampleVelocity && indexB != nullIndex {
				stateB := &awakeStates[indexB]
				vB = stateB.linearVelocity
				wB = stateB.angularVelocity
			}

			soft := contactSoftness
			if indexA == nullIndex || indexB == nullIndex {
				soft = staticSoftness
			}

			invMassA[j] = laneScalarFromQ(mA)
			invMassB[j] = laneScalarFromQ(mB)
			invIA[j] = laneScalarFromQ(iA)
			invIB[j] = laneScalarFromQ(iB)

			k := iA.Add(iB)
			if zero.Less(k) {
				rollingMass[j] = laneScalarFromQ(one.Div(k))
			}

			normal := manifold.Normal
			tangent := RightPerp(normal)
			normalX[j] = laneScalarFromQ(normal.X)
			normalY[j] = laneScalarFromQ(normal.Y)
			friction[j] = laneScalarFromQ(contact.friction)
			tangentSpeed[j] = laneScalarFromQ(contact.tangentSpeed)
			restitution[j] = laneScalarFromQ(contact.restitution)
			rollingResistance[j] = laneScalarFromQ(contact.rollingResistance)
			rollingImpulse[j] = laneScalarFromQ(warmStartScale.Mul(manifold.RollingImpulse))
			// A stored impulse counts too: the warm start applies it after
			// the rolling resistance drops to zero.
			if zero.Less(contact.rollingResistance) || !manifold.RollingImpulse.Eq(zero) {
				anyRolling = true
			}
			pushRate[j] = laneScalarFromQ(soft.massScale.Mul(soft.biasRate))
			massScale[j] = laneScalarFromQ(soft.massScale)
			impulseScale[j] = laneScalarFromQ(soft.impulseScale)

			for pointIndex := range pointCount {
				mp := &manifold.Points[pointIndex]
				rA := mp.AnchorA
				rB := mp.AnchorB

				anchorAX[pointIndex][j] = laneScalarFromQ(rA.X)
				anchorAY[pointIndex][j] = laneScalarFromQ(rA.Y)
				anchorBX[pointIndex][j] = laneScalarFromQ(rB.X)
				anchorBY[pointIndex][j] = laneScalarFromQ(rB.Y)
				baseSeparation[pointIndex][j] = laneScalarFromQ(mp.Separation.Sub(rB.Sub(rA).Dot(normal)))
				normalImpulse[pointIndex][j] = laneScalarFromQ(warmStartScale.Mul(mp.NormalImpulse))
				tangentImpulse[pointIndex][j] = laneScalarFromQ(warmStartScale.Mul(mp.TangentImpulse))

				rnA := Cross(rA, normal)
				rnB := Cross(rB, normal)
				kNormal := mA.Add(mB).Add(iA.Mul(rnA.Mul(rnA))).Add(iB.Mul(rnB.Mul(rnB)))
				if zero.Less(kNormal) {
					normalMass[pointIndex][j] = laneScalarFromQ(one.Div(kNormal))
				}

				rtA := Cross(rA, tangent)
				rtB := Cross(rB, tangent)
				kTangent := mA.Add(mB).Add(iA.Mul(rtA.Mul(rtA))).Add(iB.Mul(rtB.Mul(rtB)))
				if zero.Less(kTangent) {
					tangentMass[pointIndex][j] = laneScalarFromQ(one.Div(kTangent))
				}

				// Relative velocity at contact
				mp.NormalVelocity = zero
				if sampleVelocity {
					dvx := vB.X.Sub(wB.Mul(rB.Y)).Sub(vA.X.Sub(wA.Mul(rA.Y)))
					dvy := vB.Y.Add(wB.Mul(rB.X)).Sub(vA.Y.Add(wA.Mul(rA.X)))
					vn := dvx.Mul(normal.X).Add(dvy.Mul(normal.Y))
					relativeVelocity[pointIndex][j] = laneScalarFromQ(vn)
					mp.NormalVelocity = vn
				}
			}
		}
		constraint.hasRolling = anyRolling

		constraint.invMassA = laneLoad(&invMassA)
		constraint.invMassB = laneLoad(&invMassB)
		constraint.invIA = laneLoad(&invIA)
		constraint.invIB = laneLoad(&invIB)
		constraint.normal = vec2W{x: laneLoad(&normalX), y: laneLoad(&normalY)}
		constraint.friction = laneLoad(&friction)
		constraint.tangentSpeed = laneLoad(&tangentSpeed)
		constraint.restitution = laneLoad(&restitution)
		constraint.rollingResistance = laneLoad(&rollingResistance)
		constraint.rollingMass = laneLoad(&rollingMass)
		constraint.rollingImpulse = laneLoad(&rollingImpulse)
		constraint.pushRate = laneLoad(&pushRate)
		constraint.massScale = laneLoad(&massScale)
		constraint.impulseScale = laneLoad(&impulseScale)

		for p := range 2 {
			cp := &constraint.points[p]
			cp.anchorA = vec2W{x: laneLoad(&anchorAX[p]), y: laneLoad(&anchorAY[p])}
			cp.anchorB = vec2W{x: laneLoad(&anchorBX[p]), y: laneLoad(&anchorBY[p])}
			cp.normalMass = laneLoad(&normalMass[p])
			cp.tangentMass = laneLoad(&tangentMass[p])
			cp.baseSeparation = laneLoad(&baseSeparation[p])
			cp.normalImpulse = laneLoad(&normalImpulse[p])
			cp.tangentImpulse = laneLoad(&tangentImpulse[p])
			cp.relativeVelocity = laneLoad(&relativeVelocity[p])
			cp.restitutionImpulse = laneZero()
			cp.totalNormalImpulse = laneZero().toAcc()
		}
	}

	if anyRestitution {
		context.anyRestitution.Store(true)
	}
}

// warmStartContactsTaskWide applies stored impulses lane-wise. It corresponds
// to b2WarmStartContacts_Wide in src/contact_solver_wide.inl.
func warmStartContactsTaskWide(startIndex, endIndex int, context *stepContext, colorIndex int) {
	states := context.states
	constraints := context.graph.colors[colorIndex].contactConstraintsWide
	for i := startIndex; i < endIndex; i++ {
		constraint := &constraints[i]
		var bodyA, bodyB bodyStateW
		gatherBodyW(states, &constraint.indexA, &bodyA)
		gatherBodyW(states, &constraint.indexB, &bodyB)

		mA, iA := constraint.invMassA, constraint.invIA
		mB, iB := constraint.invMassB, constraint.invIB
		normal := constraint.normal
		tangent := vec2W{x: normal.y, y: normal.x.Neg()}

		for j := range 2 {
			cp := &constraint.points[j]
			p := vec2W{
				x: cp.normalImpulse.Mul(normal.x).Add(cp.tangentImpulse.Mul(tangent.x)),
				y: cp.normalImpulse.Mul(normal.y).Add(cp.tangentImpulse.Mul(tangent.y)),
			}
			bodyA.subImpulseW(mA, iA, p.x, p.y, crossW(cp.anchorA, p))
			bodyB.addImpulseW(mB, iB, p.x, p.y, crossW(cp.anchorB, p))

			cp.totalNormalImpulse = cp.totalNormalImpulse.AddBounded(cp.normalImpulse.toAcc())
		}

		if constraint.hasRolling {
			bodyA.w = bodyA.w.SubBounded(iA.Mul(constraint.rollingImpulse).toAcc())
			bodyB.w = bodyB.w.AddBounded(iB.Mul(constraint.rollingImpulse).toAcc())
		}

		scatterBodyW(states, &constraint.storeA, &bodyA)
		scatterBodyW(states, &constraint.storeB, &bodyB)
	}
}

// pushContactsTaskWide solves the non-penetration lane-wise with the soft
// bias. It corresponds to b2PushContacts_Wide in
// src/contact_solver_wide.inl.
func pushContactsTaskWide(startIndex, endIndex int, context *stepContext, colorIndex int) {
	states := context.states
	constraints := context.graph.colors[colorIndex].contactConstraintsWide
	invH := laneSplat(context.invH)
	negContactSpeed := laneSplat(context.world.contactSpeed).Neg()
	zero := laneZero()
	one := laneSplat(QOne())
	for i := startIndex; i < endIndex; i++ {
		constraint := &constraints[i]
		var bodyA, bodyB bodyStateW
		gatherBodyW(states, &constraint.indexA, &bodyA)
		gatherBodyW(states, &constraint.indexB, &bodyB)

		dp := vec2W{x: bodyB.dp.x.Sub(bodyA.dp.x), y: bodyB.dp.y.Sub(bodyA.dp.y)}
		mA, iA := constraint.invMassA, constraint.invIA
		mB, iB := constraint.invMassB, constraint.invIB
		normal := constraint.normal

		wA := bodyA.w.toLane()
		wB := bodyB.w.toLane()
		dn := bodyB.v.x.Sub(bodyA.v.x).toLane().Mul(normal.x).Add(bodyB.v.y.Sub(bodyA.v.y).toLane().Mul(normal.y))

		var rnA, rnB, impulses [2]laneW
		for j := range 2 {
			cp := &constraint.points[j]
			s := separationW(&bodyA, &bodyB, dp, normal, cp)

			// Apply speculative bias if separation is greater than zero,
			// otherwise apply soft constraint bias. The contact speed limits
			// stiffness, it does not increase it.
			separated := s.Greater(zero)
			velocityBias := laneBlend(separated, s.Mul(invH), negContactSpeed.Max(constraint.pushRate.Mul(s)))
			massScale := laneBlend(separated, one, constraint.massScale)
			impulseScale := laneBlend(separated, zero, constraint.impulseScale)

			rnA[j] = crossW(cp.anchorA, normal)
			rnB[j] = crossW(cp.anchorB, normal)

			vn := dn.Add(wB.Mul(rnB[j]).Sub(wA.Mul(rnA[j])))
			if j == 1 {
				// Point 2 only depends on the impulse from point 1.
				k := mA.Add(mB).Add(iA.Mul(rnA[0]).Mul(rnA[1]).Add(iB.Mul(rnB[0]).Mul(rnB[1])))
				vn = vn.Add(k.Mul(impulses[0]))
			}

			negImpulse := cp.normalMass.Mul(massScale.Mul(vn).Add(velocityBias)).Add(impulseScale.Mul(cp.normalImpulse))
			impulse := cp.normalImpulse.Neg().Max(negImpulse.Neg())
			cp.normalImpulse = cp.normalImpulse.Add(impulse)
			cp.totalNormalImpulse = cp.totalNormalImpulse.AddBounded(impulse.toAcc())
			impulses[j] = impulse
		}

		impulse := impulses[0].Add(impulses[1])
		px := impulse.Mul(normal.x)
		py := impulse.Mul(normal.y)
		angularA := impulses[0].Mul(rnA[0]).Add(impulses[1].Mul(rnA[1]))
		angularB := impulses[0].Mul(rnB[0]).Add(impulses[1].Mul(rnB[1]))

		bodyA.subImpulseW(mA, iA, px, py, angularA)
		bodyB.addImpulseW(mB, iB, px, py, angularB)

		scatterBodyW(states, &constraint.storeA, &bodyA)
		scatterBodyW(states, &constraint.storeB, &bodyB)
	}
}

// solveContactsTaskWide relaxes the constraints lane-wise: the normal
// impulse with the speculative bias only, friction and rolling resistance.
// It corresponds to b2SolveContacts_Wide in src/contact_solver_wide.inl.
func solveContactsTaskWide(startIndex, endIndex int, context *stepContext, colorIndex int) {
	states := context.states
	constraints := context.graph.colors[colorIndex].contactConstraintsWide
	invH := laneSplat(context.invH)
	zero := laneZero()
	for i := startIndex; i < endIndex; i++ {
		constraint := &constraints[i]
		var bodyA, bodyB bodyStateW
		gatherBodyW(states, &constraint.indexA, &bodyA)
		gatherBodyW(states, &constraint.indexB, &bodyB)

		dp := vec2W{x: bodyB.dp.x.Sub(bodyA.dp.x), y: bodyB.dp.y.Sub(bodyA.dp.y)}
		mA, iA := constraint.invMassA, constraint.invIA
		mB, iB := constraint.invMassB, constraint.invIB
		normal := constraint.normal
		tangent := vec2W{x: normal.y, y: normal.x.Neg()}
		c1 := &constraint.points[0]
		c2 := &constraint.points[1]

		// Speculative bias, positive if separated and zero if overlapped
		velocityBias1 := separationW(&bodyA, &bodyB, dp, normal, c1).Mul(invH).Max(zero)
		velocityBias2 := separationW(&bodyA, &bodyB, dp, normal, c2).Mul(invH).Max(zero)

		rnA1 := crossW(c1.anchorA, normal)
		rnB1 := crossW(c1.anchorB, normal)
		rnA2 := crossW(c2.anchorA, normal)
		rnB2 := crossW(c2.anchorB, normal)
		rtA1 := crossW(c1.anchorA, tangent)
		rtB1 := crossW(c1.anchorB, tangent)
		rtA2 := crossW(c2.anchorA, tangent)
		rtB2 := crossW(c2.anchorB, tangent)

		wnA1 := iA.Mul(rnA1)
		wnB1 := iB.Mul(rnB1)
		wnA2 := iA.Mul(rnA2)
		wnB2 := iB.Mul(rnB2)
		wtA1 := iA.Mul(rtA1)
		wtB1 := iB.Mul(rtB1)

		invMass := mA.Add(mB)
		k21 := invMass.Add(wnA1.Mul(rnA2).Add(wnB1.Mul(rnB2)))
		k31 := wnA1.Mul(rtA1).Add(wnB1.Mul(rtB1))
		k32 := wnA2.Mul(rtA1).Add(wnB2.Mul(rtB1))
		k41 := wnA1.Mul(rtA2).Add(wnB1.Mul(rtB2))
		k42 := wnA2.Mul(rtA2).Add(wnB2.Mul(rtB2))
		k43 := invMass.Add(wtA1.Mul(rtA2).Add(wtB1.Mul(rtB2)))

		wA := bodyA.w.toLane()
		wB := bodyB.w.toLane()
		dvx := bodyB.v.x.Sub(bodyA.v.x).toLane()
		dvy := bodyB.v.y.Sub(bodyA.v.y).toLane()
		dn := dvx.Mul(normal.x).Add(dvy.Mul(normal.y))
		dt := dvx.Mul(tangent.x).Add(dvy.Mul(tangent.y)).Sub(constraint.tangentSpeed)

		vn1 := dn.Add(wB.Mul(rnB1).Sub(wA.Mul(rnA1)))
		vn2 := dn.Add(wB.Mul(rnB2).Sub(wA.Mul(rnA2)))
		vt1 := dt.Add(wB.Mul(rtB1).Sub(wA.Mul(rtA1)))
		vt2 := dt.Add(wB.Mul(rtB2).Sub(wA.Mul(rtA2)))

		normalImpulse1 := c1.normalImpulse.Neg().Max(c1.normalMass.Mul(vn1.Add(velocityBias1)).Neg())
		c1.normalImpulse = c1.normalImpulse.Add(normalImpulse1)
		c1.totalNormalImpulse = c1.totalNormalImpulse.AddBounded(normalImpulse1.toAcc())

		vn2 = vn2.Add(k21.Mul(normalImpulse1))

		normalImpulse2 := c2.normalImpulse.Neg().Max(c2.normalMass.Mul(vn2.Add(velocityBias2)).Neg())
		c2.normalImpulse = c2.normalImpulse.Add(normalImpulse2)
		c2.totalNormalImpulse = c2.totalNormalImpulse.AddBounded(normalImpulse2.toAcc())

		vt1 = vt1.Add(k31.Mul(normalImpulse1)).Add(k32.Mul(normalImpulse2))
		vt2 = vt2.Add(k41.Mul(normalImpulse1)).Add(k42.Mul(normalImpulse2))

		angularA := normalImpulse1.Mul(rnA1).Add(normalImpulse2.Mul(rnA2))
		angularB := normalImpulse1.Mul(rnB1).Add(normalImpulse2.Mul(rnB2))

		// Rolling resistance, for the lane groups with any.
		if active, anyLane := laneGroupMask(constraint.rollingResistance); anyLane {
			relativeW := wA.Sub(wB).Sub(wnA1.Add(wnB1).Mul(normalImpulse1).Add(wnA2.Add(wnB2).Mul(normalImpulse2)))
			deltaLambda := laneDivPositive(relativeW, iA.Add(iB))
			lambda := constraint.rollingImpulse
			maxLambda := constraint.rollingResistance.Mul(c1.normalImpulse.Add(c2.normalImpulse))
			rollingImpulse := laneClamp(lambda.Add(deltaLambda), maxLambda.Neg(), maxLambda)
			deltaLambda = rollingImpulse.Sub(lambda)

			wtA2 := iA.Mul(rtA2)
			wtB2 := iB.Mul(rtB2)
			constraint.rollingImpulse = laneBlend(active, rollingImpulse, lambda)
			vt1 = laneBlend(active, vt1.Add(wtA1.Add(wtB1).Mul(deltaLambda)), vt1)
			vt2 = laneBlend(active, vt2.Add(wtA2.Add(wtB2).Mul(deltaLambda)), vt2)
			angularA = laneBlend(active, angularA.Add(deltaLambda), angularA)
			angularB = laneBlend(active, angularB.Add(deltaLambda), angularB)
		}

		maxFriction1 := constraint.friction.Mul(c1.normalImpulse)
		tangentImpulse1 := c1.tangentMass.Mul(vt1).Neg()
		tangentImpulse1 = maxFriction1.Sub(c1.tangentImpulse).Min(tangentImpulse1).Max(maxFriction1.Neg().Sub(c1.tangentImpulse))
		c1.tangentImpulse = c1.tangentImpulse.Add(tangentImpulse1)

		vt2 = vt2.Add(k43.Mul(tangentImpulse1))

		maxFriction2 := constraint.friction.Mul(c2.normalImpulse)
		tangentImpulse2 := c2.tangentMass.Mul(vt2).Neg()
		tangentImpulse2 = maxFriction2.Sub(c2.tangentImpulse).Min(tangentImpulse2).Max(maxFriction2.Neg().Sub(c2.tangentImpulse))
		c2.tangentImpulse = c2.tangentImpulse.Add(tangentImpulse2)

		angularA = angularA.Add(tangentImpulse1.Mul(rtA1).Add(tangentImpulse2.Mul(rtA2)))
		angularB = angularB.Add(tangentImpulse1.Mul(rtB1).Add(tangentImpulse2.Mul(rtB2)))

		normalImpulse := normalImpulse1.Add(normalImpulse2)
		tangentImpulse := tangentImpulse1.Add(tangentImpulse2)
		px := normalImpulse.Mul(normal.x).Add(tangentImpulse.Mul(tangent.x))
		py := normalImpulse.Mul(normal.y).Add(tangentImpulse.Mul(tangent.y))

		bodyA.subImpulseW(mA, iA, px, py, angularA)
		bodyB.addImpulseW(mB, iB, px, py, angularB)

		scatterBodyW(states, &constraint.storeA, &bodyA)
		scatterBodyW(states, &constraint.storeB, &bodyB)
	}
}

// applyRestitutionTaskWide applies Poisson restitution lane-wise. It
// corresponds to b2ApplyRestitution_Wide in src/contact_solver_wide.inl.
// Without propagation a lane group without restitution keeps its values,
// as the scalar stage skips it.
func applyRestitutionTaskWide(startIndex, endIndex int, context *stepContext, colorIndex int) {
	w := context.world
	states := context.states
	constraints := context.graph.colors[colorIndex].contactConstraintsWide
	negThreshold := laneSplat(w.restitutionThreshold).Neg()
	invH := laneSplat(context.invH)
	propagate := w.enableRestitutionPropagation
	zero := laneZero()
	one := laneSplat(QOne())
	for i := startIndex; i < endIndex; i++ {
		constraint := &constraints[i]
		restitution := constraint.restitution

		storeA, storeB := constraint.storeA, constraint.storeB
		var active maskW
		if !propagate {
			var anyLane bool
			active, anyLane = laneGroupMask(restitution)
			if !anyLane {
				continue
			}
			storeA = maskIndices(&storeA, active)
			storeB = maskIndices(&storeB, active)
		}

		var bodyA, bodyB bodyStateW
		gatherBodyW(states, &constraint.indexA, &bodyA)
		gatherBodyW(states, &constraint.indexB, &bodyB)

		dp := vec2W{x: bodyB.dp.x.Sub(bodyA.dp.x), y: bodyB.dp.y.Sub(bodyA.dp.y)}
		mA, iA := constraint.invMassA, constraint.invIA
		mB, iB := constraint.invMassB, constraint.invIB
		normal := constraint.normal
		haveRestitution := restitution.Greater(zero)

		for j := range 2 {
			cp := &constraint.points[j]
			rA := cp.anchorA
			rB := cp.anchorB

			normalMass := cp.normalMass
			if !propagate {
				normalMass = laneBlend(haveRestitution, normalMass, zero)
			}

			// The total normal impulse is 0 for speculative points.
			compressionImpulse := cp.totalNormalImpulse.Sub(cp.restitutionImpulse.toAcc()).toLane()
			armedLane := laneBlend(haveRestitution,
				laneBlend(negThreshold.Greater(cp.relativeVelocity),
					laneBlend(compressionImpulse.Greater(zero), one, zero), zero), zero)
			armed := armedLane.Greater(zero)

			velocityBias := laneBlend(armed, restitution.Mul(cp.relativeVelocity),
				separationW(&bodyA, &bodyB, dp, normal, cp).Mul(invH).Max(zero))

			wA := bodyA.w.toLane()
			wB := bodyB.w.toLane()
			dvx := bodyB.v.x.toLane().Sub(wB.Mul(rB.y)).Sub(bodyA.v.x.toLane().Sub(wA.Mul(rA.y)))
			dvy := bodyB.v.y.toLane().Add(wB.Mul(rB.x)).Sub(bodyA.v.y.toLane().Add(wA.Mul(rA.x)))
			vn := dvx.Mul(normal.x).Add(dvy.Mul(normal.y))

			negImpulse := normalMass.Mul(vn.Add(velocityBias))
			newImpulse := zero.Max(cp.normalImpulse.Sub(negImpulse))
			impulse := newImpulse.Sub(cp.normalImpulse)

			approachImpulse := zero.Max(impulse).Min(zero.Max(normalMass.Mul(vn).Neg()))
			// Poisson kinetic restitution guarantees no energy gain.
			allowance := restitution.Mul(compressionImpulse.Add(approachImpulse)).Sub(cp.restitutionImpulse)
			impulse = laneBlend(armed, approachImpulse.Add(zero.Max(allowance)).Min(impulse), impulse)

			normalImpulse := cp.normalImpulse.Add(impulse)
			restitutionImpulse := cp.restitutionImpulse.Add(impulse.Sub(approachImpulse))
			totalNormalImpulse := cp.totalNormalImpulse.AddBounded(impulse.toAcc())
			if propagate {
				cp.normalImpulse = normalImpulse
				cp.restitutionImpulse = restitutionImpulse
				cp.totalNormalImpulse = totalNormalImpulse
			} else {
				cp.normalImpulse = laneBlend(active, normalImpulse, cp.normalImpulse)
				cp.restitutionImpulse = laneBlend(active, restitutionImpulse, cp.restitutionImpulse)
				cp.totalNormalImpulse = blendAccW(active, totalNormalImpulse, cp.totalNormalImpulse)
			}

			px := impulse.Mul(normal.x)
			py := impulse.Mul(normal.y)
			bodyA.subImpulseW(mA, iA, px, py, rA.x.Mul(py).Sub(rA.y.Mul(px)))
			bodyB.addImpulseW(mB, iB, px, py, rB.x.Mul(py).Sub(rB.y.Mul(px)))
		}

		scatterBodyW(states, &storeA, &bodyA)
		scatterBodyW(states, &storeB, &bodyB)
	}
}

// storeImpulsesTaskWide stores the lanes back into the manifolds and flags
// the contacts with a hit event candidate. It corresponds to
// b2StoreImpulses_Wide in src/contact_solver_wide.inl.
func storeImpulsesTaskWide(startIndex, endIndex int, context *stepContext, workerIndex int) {
	w := context.world
	taskContext := &w.taskContexts[workerIndex]
	negHitThreshold := w.hitEventThreshold.Neg()
	zero := QZero()
	for constraintIndex := startIndex; constraintIndex < endIndex; constraintIndex++ {
		constraint := &context.contactConstraintsWide[constraintIndex]
		var (
			rollingImpulse     [wideWidth]laneScalar
			normalImpulse      [2][wideWidth]laneScalar
			tangentImpulse     [2][wideWidth]laneScalar
			totalNormalImpulse [2][wideWidth]accScalar
		)
		constraint.rollingImpulse.store(&rollingImpulse)
		for p := range 2 {
			cp := &constraint.points[p]
			cp.normalImpulse.store(&normalImpulse[p])
			cp.tangentImpulse.store(&tangentImpulse[p])
			cp.totalNormalImpulse.store(&totalNormalImpulse[p])
		}

		baseIndex := wideWidth * constraintIndex
		for laneIndex := range wideWidth {
			cs := context.contacts[baseIndex+laneIndex]
			if cs == nil {
				continue
			}

			manifold := &cs.manifold
			pointCount := manifold.PointCount
			for p := range pointCount {
				mp := &manifold.Points[p]
				mp.NormalImpulse = laneScalarToQ(normalImpulse[p][laneIndex])
				mp.TangentImpulse = laneScalarToQ(tangentImpulse[p][laneIndex])
				mp.TotalNormalImpulse = accScalarToQ(totalNormalImpulse[p][laneIndex])
			}
			manifold.RollingImpulse = laneScalarToQ(rollingImpulse[laneIndex])

			if cs.simFlags&simEnableHitEvent != 0 {
				for p := range pointCount {
					mp := &manifold.Points[p]

					// Need to check total impulse because the point may be speculative and not colliding
					if mp.NormalVelocity.Less(negHitThreshold) && zero.Less(mp.TotalNormalImpulse) {
						taskContext.hitEventBitSet.setBit(cs.contactId)
						taskContext.hasHitEvents = true
						break
					}
				}
			}
		}
	}
}

// wideScratch holds the lane-padded copy of the active contacts, which lives
// apart from w.contactPointers.
type wideScratch struct {
	contactPointers []*contactSim
}

// contactPointWide is one manifold point of a wide constraint. A one point
// manifold leaves the second point zero.
type contactPointWide struct {
	anchorA, anchorB              vec2W
	baseSeparation                laneW
	relativeVelocity              laneW
	normalImpulse, tangentImpulse laneW
	restitutionImpulse            laneW
	totalNormalImpulse            accW
	normalMass, tangentMass       laneW
}

// contactConstraintWide mirrors b2ContactConstraintWide in
// src/contact_solver_wide.inl.
type contactConstraintWide struct {
	// indexA and indexB gather the bodies; storeA and storeB scatter the
	// velocities back, with nullIndex for a body that is not dynamic.
	indexA, indexB [wideWidth]int
	storeA, storeB [wideWidth]int
	// hasRolling is false when no lane uses rolling resistance or holds a
	// rolling impulse. The warm start then skips the rolling block, which
	// would only add zero.
	hasRolling                                bool
	invMassA, invMassB, invIA, invIB          laneW
	normal                                    vec2W
	friction, tangentSpeed, rollingResistance laneW
	rollingMass                               laneW
	rollingImpulse                            laneW
	pushRate, massScale, impulseScale         laneW
	restitution                               laneW
	points                                    [2]contactPointWide
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

// LanePath reports the contact-solver lane path of this build. The wide
// family falls back to the scalar family on a CPU without the required
// feature, so this reports the path the process actually runs, not the path
// its build tags asked for.
func LanePath() string {
	if !wideEnabled {
		return "scalar"
	}
	return widePath()
}

// LaneWidth reports the number of contacts one solver unit carries.
func LaneWidth() int {
	if !wideEnabled {
		return 1
	}
	return wideWidth
}

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
func runContactStageBlock(stage *solverStage, context *stepContext, startIndex, endIndex, workerIndex int) {
	if !wideEnabled {
		runContactStageBlockScalar(stage, context, startIndex, endIndex, workerIndex)
		return
	}
	switch stage.stageType {
	case stagePrepareContacts:
		prepareContactsTaskWide(startIndex, endIndex, context)
	case stageStoreImpulses:
		storeImpulsesTaskWide(startIndex, endIndex, context, workerIndex)
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
		pushContactsTaskWide(startIndex, endIndex, context, stage.colorIndex)
	case stageRelax:
		solveContactsTaskWide(startIndex, endIndex, context, stage.colorIndex)
	case stageRestitution:
		applyRestitutionTaskWide(startIndex, endIndex, context, stage.colorIndex)
	}
}

// allocateContactConstraints reserves scalar or wide contact scratch.
func (s *wideScratch) allocateContactConstraints(w *world, context *stepContext, colors *[graphColorCount]graphColor, overflowIndex, activeContactCount int) {
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
	s.contactPointers = slices.Grow(s.contactPointers[:0], paddedContactCount)[:paddedContactCount]
	context.contacts = s.contactPointers

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
