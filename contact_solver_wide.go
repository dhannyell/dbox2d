//go:build dbox2d_simd && dbox2d_float

package dbox2d

// dotW computes a lane-wise dot product in scalar Vec2.Dot order. It
// corresponds to b2DotW in src/contact_solver.c.
func dotW(a, b vec2W) laneW {
	return a.x.Mul(b.x).Add(a.y.Mul(b.y))
}

// crossW computes a lane-wise cross product in scalar Cross order. It
// corresponds to b2CrossW in src/contact_solver.c.
func crossW(a, b vec2W) laneW {
	return a.x.Mul(b.y).Sub(a.y.Mul(b.x))
}

// rotateVectorW rotates lane-wise vectors in scalar Rot.Apply order. It
// corresponds to b2RotateVectorW in src/contact_solver.c.
func rotateVectorW(q rotW, v vec2W) vec2W {
	return vec2W{
		x: q.c.Mul(v.x).Sub(q.s.Mul(v.y)),
		y: q.s.Mul(v.x).Add(q.c.Mul(v.y)),
	}
}

// prepareContactsTaskWide prepares scalar lanes before loading the SoA. It
// corresponds to b2PrepareContactsTask in src/contact_solver.c.
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

	for i := startIndex; i < endIndex; i++ {
		var (
			invMassA, invMassB, invIA, invIB                                    [wideWidth]float32
			normalX, normalY                                                    [wideWidth]float32
			friction, tangentSpeed, restitution, rollingResistance              [wideWidth]float32
			rollingMass, rollingImpulse                                         [wideWidth]float32
			biasRate, massScale, impulseScale                                   [wideWidth]float32
			anchorAX, anchorAY, anchorBX, anchorBY                              [2][wideWidth]float32
			normalMass, tangentMass, baseSeparation                             [2][wideWidth]float32
			normalImpulse, totalNormalImpulse, tangentImpulse, relativeVelocity [2][wideWidth]float32
		)

		constraint := &context.contactConstraintsWide[i]
		for j := range wideWidth {
			constraint.indexA[j] = nullIndex
			constraint.indexB[j] = nullIndex

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

			vA := Vec2Zero()
			wA := zero
			mA := contact.invMassA
			iA := contact.invIA
			if indexA != nullIndex {
				stateA := &awakeStates[indexA]
				vA = stateA.linearVelocity
				wA = stateA.angularVelocity.Mul(tau)
			}

			vB := Vec2Zero()
			wB := zero
			mB := contact.invMassB
			iB := contact.invIB
			if indexB != nullIndex {
				stateB := &awakeStates[indexB]
				vB = stateB.linearVelocity
				wB = stateB.angularVelocity.Mul(tau)
			}

			soft := contactSoftness
			if indexA == nullIndex || indexB == nullIndex {
				soft = staticSoftness
			}

			invMassA[j] = mA.v
			invMassB[j] = mB.v
			invIA[j] = iA.v
			invIB[j] = iB.v

			k := iA.Add(iB)
			if zero.Less(k) {
				rollingMass[j] = one.Div(k).v
			}

			normal := manifold.Normal
			tangent := RightPerp(normal)
			normalX[j] = normal.X.v
			normalY[j] = normal.Y.v
			friction[j] = contact.friction.v
			tangentSpeed[j] = contact.tangentSpeed.v
			restitution[j] = contact.restitution.v
			rollingResistance[j] = contact.rollingResistance.v
			rollingImpulse[j] = warmStartScale.Mul(manifold.RollingImpulse).v
			biasRate[j] = soft.biasRate.v
			massScale[j] = soft.massScale.v
			impulseScale[j] = soft.impulseScale.v

			for pointIndex := range pointCount {
				mp := &manifold.Points[pointIndex]
				rA := mp.AnchorA
				rB := mp.AnchorB

				anchorAX[pointIndex][j] = rA.X.v
				anchorAY[pointIndex][j] = rA.Y.v
				anchorBX[pointIndex][j] = rB.X.v
				anchorBY[pointIndex][j] = rB.Y.v
				baseSeparation[pointIndex][j] = mp.Separation.Sub(rB.Sub(rA).Dot(normal)).v
				normalImpulse[pointIndex][j] = warmStartScale.Mul(mp.NormalImpulse).v
				tangentImpulse[pointIndex][j] = warmStartScale.Mul(mp.TangentImpulse).v
				totalNormalImpulse[pointIndex][j] = zero.v

				rnA := Cross(rA, normal)
				rnB := Cross(rB, normal)
				kNormal := mA.Add(mB).Add(iA.Mul(rnA).Mul(rnA)).Add(iB.Mul(rnB).Mul(rnB))
				if zero.Less(kNormal) {
					normalMass[pointIndex][j] = one.Div(kNormal).v
				}

				rtA := Cross(rA, tangent)
				rtB := Cross(rB, tangent)
				kTangent := mA.Add(mB).Add(iA.Mul(rtA).Mul(rtA)).Add(iB.Mul(rtB).Mul(rtB))
				if zero.Less(kTangent) {
					tangentMass[pointIndex][j] = one.Div(kTangent).v
				}

				vrA := vA.Add(CrossSV(wA, rA))
				vrB := vB.Add(CrossSV(wB, rB))
				relativeVelocity[pointIndex][j] = normal.Dot(vrB.Sub(vrA)).v
			}
		}

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
		constraint.rollingImpulse = laneLoad(&rollingImpulse).toAcc()
		constraint.biasRate = laneLoad(&biasRate)
		constraint.massScale = laneLoad(&massScale)
		constraint.impulseScale = laneLoad(&impulseScale)

		constraint.anchorA1 = vec2W{x: laneLoad(&anchorAX[0]), y: laneLoad(&anchorAY[0])}
		constraint.anchorB1 = vec2W{x: laneLoad(&anchorBX[0]), y: laneLoad(&anchorBY[0])}
		constraint.normalMass1 = laneLoad(&normalMass[0])
		constraint.tangentMass1 = laneLoad(&tangentMass[0])
		constraint.baseSeparation1 = laneLoad(&baseSeparation[0])
		constraint.normalImpulse1 = laneLoad(&normalImpulse[0]).toAcc()
		constraint.totalNormalImpulse1 = laneLoad(&totalNormalImpulse[0]).toAcc()
		constraint.tangentImpulse1 = laneLoad(&tangentImpulse[0]).toAcc()
		constraint.relativeVelocity1 = laneLoad(&relativeVelocity[0])

		constraint.anchorA2 = vec2W{x: laneLoad(&anchorAX[1]), y: laneLoad(&anchorAY[1])}
		constraint.anchorB2 = vec2W{x: laneLoad(&anchorBX[1]), y: laneLoad(&anchorBY[1])}
		constraint.normalMass2 = laneLoad(&normalMass[1])
		constraint.tangentMass2 = laneLoad(&tangentMass[1])
		constraint.baseSeparation2 = laneLoad(&baseSeparation[1])
		constraint.normalImpulse2 = laneLoad(&normalImpulse[1]).toAcc()
		constraint.totalNormalImpulse2 = laneLoad(&totalNormalImpulse[1]).toAcc()
		constraint.tangentImpulse2 = laneLoad(&tangentImpulse[1]).toAcc()
		constraint.relativeVelocity2 = laneLoad(&relativeVelocity[1])
	}
}

// warmStartContactsTaskWide applies stored impulses lane-wise. It corresponds
// to b2WarmStartContactsTask in src/contact_solver.c.
func warmStartContactsTaskWide(startIndex, endIndex int, context *stepContext, colorIndex int) {
	states := context.states
	constraints := context.graph.colors[colorIndex].contactConstraintsWide
	negOne := laneSplat(Q{v: -1})
	tauW := laneSplat(tau)

	for i := startIndex; i < endIndex; i++ {
		constraint := &constraints[i]
		var bodyA, bodyB bodyStateW
		gatherBodyW(states, &constraint.indexA, tauW, &bodyA)
		gatherBodyW(states, &constraint.indexB, tauW, &bodyB)
		tangentX := constraint.normal.y
		tangentY := negOne.Mul(constraint.normal.x)

		{
			normalImpulse := constraint.normalImpulse1.toLane()
			tangentImpulse := constraint.tangentImpulse1.toLane()
			p := vec2W{
				x: constraint.normal.x.Mul(normalImpulse).Add(tangentX.Mul(tangentImpulse)),
				y: constraint.normal.y.Mul(normalImpulse).Add(tangentY.Mul(tangentImpulse)),
			}
			bodyA.w = bodyA.w.Sub(constraint.invIA.Mul(crossW(constraint.anchorA1, p)).toAcc())
			bodyA.v.x = bodyA.v.x.Sub(constraint.invMassA.Mul(p.x).toAcc())
			bodyA.v.y = bodyA.v.y.Sub(constraint.invMassA.Mul(p.y).toAcc())
			bodyB.w = bodyB.w.Add(constraint.invIB.Mul(crossW(constraint.anchorB1, p)).toAcc())
			bodyB.v.x = bodyB.v.x.Add(constraint.invMassB.Mul(p.x).toAcc())
			bodyB.v.y = bodyB.v.y.Add(constraint.invMassB.Mul(p.y).toAcc())
		}

		{
			normalImpulse := constraint.normalImpulse2.toLane()
			tangentImpulse := constraint.tangentImpulse2.toLane()
			p := vec2W{
				x: constraint.normal.x.Mul(normalImpulse).Add(tangentX.Mul(tangentImpulse)),
				y: constraint.normal.y.Mul(normalImpulse).Add(tangentY.Mul(tangentImpulse)),
			}
			bodyA.w = bodyA.w.Sub(constraint.invIA.Mul(crossW(constraint.anchorA2, p)).toAcc())
			bodyA.v.x = bodyA.v.x.Sub(constraint.invMassA.Mul(p.x).toAcc())
			bodyA.v.y = bodyA.v.y.Sub(constraint.invMassA.Mul(p.y).toAcc())
			bodyB.w = bodyB.w.Add(constraint.invIB.Mul(crossW(constraint.anchorB2, p)).toAcc())
			bodyB.v.x = bodyB.v.x.Add(constraint.invMassB.Mul(p.x).toAcc())
			bodyB.v.y = bodyB.v.y.Add(constraint.invMassB.Mul(p.y).toAcc())
		}

		bodyA.w = bodyA.w.Sub(constraint.invIA.Mul(constraint.rollingImpulse.toLane()).toAcc())
		bodyB.w = bodyB.w.Add(constraint.invIB.Mul(constraint.rollingImpulse.toLane()).toAcc())
		scatterBodyW(states, &constraint.indexA, tauW, &bodyA)
		scatterBodyW(states, &constraint.indexB, tauW, &bodyB)
	}
}

// solveContactsTaskWide solves colored constraints lane-wise. It corresponds
// to b2SolveContactsTask in src/contact_solver.c.
func solveContactsTaskWide(startIndex, endIndex int, context *stepContext, colorIndex int, useBias bool) {
	states := context.states
	constraints := context.graph.colors[colorIndex].contactConstraintsWide
	invH := laneSplat(context.invH)
	minBiasVelocity := laneSplat(context.world.contactSpeed.Neg())
	zero := laneZero()
	one := laneSplat(QOne())
	negOne := laneSplat(Q{v: -1})
	tauW := laneSplat(tau)

	for i := startIndex; i < endIndex; i++ {
		constraint := &constraints[i]
		var bodyA, bodyB bodyStateW
		gatherBodyW(states, &constraint.indexA, tauW, &bodyA)
		gatherBodyW(states, &constraint.indexB, tauW, &bodyB)

		biasRate := zero
		massScale := one
		impulseScale := zero
		if useBias {
			biasRate = constraint.biasRate
			massScale = constraint.massScale
			impulseScale = constraint.impulseScale
		}

		totalNormalImpulse := zero.toAcc()
		dp := vec2W{x: bodyB.dp.x.Sub(bodyA.dp.x), y: bodyB.dp.y.Sub(bodyA.dp.y)}
		normal := constraint.normal
		tangent := vec2W{x: normal.y, y: negOne.Mul(normal.x)}

		{
			rA := constraint.anchorA1
			rB := constraint.anchorB1
			rsA := rotateVectorW(bodyA.dq, rA)
			rsB := rotateVectorW(bodyB.dq, rB)
			ds := vec2W{x: dp.x.Add(rsB.x.Sub(rsA.x)), y: dp.y.Add(rsB.y.Sub(rsA.y))}
			s := constraint.baseSeparation1.Add(dotW(ds, normal))

			separated := s.Greater(zero)
			speculativeBias := s.Mul(invH)
			softBias := biasRate.Mul(s).Max(minBiasVelocity)
			velocityBias := laneBlend(separated, speculativeBias, softBias)
			pointMassScale := laneBlend(separated, one, massScale)
			pointImpulseScale := laneBlend(separated, zero, impulseScale)

			negWA := negOne.Mul(bodyA.w.toLane())
			negWB := negOne.Mul(bodyB.w.toLane())
			vrA := vec2W{
				x: bodyA.v.x.toLane().Add(negWA.Mul(rA.y)),
				y: bodyA.v.y.toLane().Add(bodyA.w.toLane().Mul(rA.x)),
			}
			vrB := vec2W{
				x: bodyB.v.x.toLane().Add(negWB.Mul(rB.y)),
				y: bodyB.v.y.toLane().Add(bodyB.w.toLane().Mul(rB.x)),
			}
			vn := dotW(vec2W{x: vrB.x.Sub(vrA.x), y: vrB.y.Sub(vrA.y)}, normal)

			impulse := negOne.Mul(constraint.normalMass1).Mul(pointMassScale).Mul(vn.Add(velocityBias)).Sub(pointImpulseScale.Mul(constraint.normalImpulse1.toLane()))
			newImpulse := constraint.normalImpulse1.toLane().Add(impulse).Max(zero)
			impulse = newImpulse.Sub(constraint.normalImpulse1.toLane())
			constraint.normalImpulse1 = newImpulse.toAcc()
			constraint.totalNormalImpulse1 = constraint.totalNormalImpulse1.Add(newImpulse.toAcc())
			totalNormalImpulse = totalNormalImpulse.Add(newImpulse.toAcc())

			p := vec2W{x: normal.x.Mul(impulse), y: normal.y.Mul(impulse)}
			bodyA.v.x = bodyA.v.x.Sub(constraint.invMassA.Mul(p.x).toAcc())
			bodyA.v.y = bodyA.v.y.Sub(constraint.invMassA.Mul(p.y).toAcc())
			bodyA.w = bodyA.w.Sub(constraint.invIA.Mul(crossW(rA, p)).toAcc())
			bodyB.v.x = bodyB.v.x.Add(constraint.invMassB.Mul(p.x).toAcc())
			bodyB.v.y = bodyB.v.y.Add(constraint.invMassB.Mul(p.y).toAcc())
			bodyB.w = bodyB.w.Add(constraint.invIB.Mul(crossW(rB, p)).toAcc())
		}

		{
			rA := constraint.anchorA2
			rB := constraint.anchorB2
			rsA := rotateVectorW(bodyA.dq, rA)
			rsB := rotateVectorW(bodyB.dq, rB)
			ds := vec2W{x: dp.x.Add(rsB.x.Sub(rsA.x)), y: dp.y.Add(rsB.y.Sub(rsA.y))}
			s := constraint.baseSeparation2.Add(dotW(ds, normal))

			separated := s.Greater(zero)
			speculativeBias := s.Mul(invH)
			softBias := biasRate.Mul(s).Max(minBiasVelocity)
			velocityBias := laneBlend(separated, speculativeBias, softBias)
			pointMassScale := laneBlend(separated, one, massScale)
			pointImpulseScale := laneBlend(separated, zero, impulseScale)

			negWA := negOne.Mul(bodyA.w.toLane())
			negWB := negOne.Mul(bodyB.w.toLane())
			vrA := vec2W{
				x: bodyA.v.x.toLane().Add(negWA.Mul(rA.y)),
				y: bodyA.v.y.toLane().Add(bodyA.w.toLane().Mul(rA.x)),
			}
			vrB := vec2W{
				x: bodyB.v.x.toLane().Add(negWB.Mul(rB.y)),
				y: bodyB.v.y.toLane().Add(bodyB.w.toLane().Mul(rB.x)),
			}
			vn := dotW(vec2W{x: vrB.x.Sub(vrA.x), y: vrB.y.Sub(vrA.y)}, normal)

			impulse := negOne.Mul(constraint.normalMass2).Mul(pointMassScale).Mul(vn.Add(velocityBias)).Sub(pointImpulseScale.Mul(constraint.normalImpulse2.toLane()))
			newImpulse := constraint.normalImpulse2.toLane().Add(impulse).Max(zero)
			impulse = newImpulse.Sub(constraint.normalImpulse2.toLane())
			constraint.normalImpulse2 = newImpulse.toAcc()
			constraint.totalNormalImpulse2 = constraint.totalNormalImpulse2.Add(newImpulse.toAcc())
			totalNormalImpulse = totalNormalImpulse.Add(newImpulse.toAcc())

			p := vec2W{x: normal.x.Mul(impulse), y: normal.y.Mul(impulse)}
			bodyA.v.x = bodyA.v.x.Sub(constraint.invMassA.Mul(p.x).toAcc())
			bodyA.v.y = bodyA.v.y.Sub(constraint.invMassA.Mul(p.y).toAcc())
			bodyA.w = bodyA.w.Sub(constraint.invIA.Mul(crossW(rA, p)).toAcc())
			bodyB.v.x = bodyB.v.x.Add(constraint.invMassB.Mul(p.x).toAcc())
			bodyB.v.y = bodyB.v.y.Add(constraint.invMassB.Mul(p.y).toAcc())
			bodyB.w = bodyB.w.Add(constraint.invIB.Mul(crossW(rB, p)).toAcc())
		}

		{
			rA := constraint.anchorA1
			rB := constraint.anchorB1
			negWA := negOne.Mul(bodyA.w.toLane())
			negWB := negOne.Mul(bodyB.w.toLane())
			vrB := vec2W{
				x: bodyB.v.x.toLane().Add(negWB.Mul(rB.y)),
				y: bodyB.v.y.toLane().Add(bodyB.w.toLane().Mul(rB.x)),
			}
			vrA := vec2W{
				x: bodyA.v.x.toLane().Add(negWA.Mul(rA.y)),
				y: bodyA.v.y.toLane().Add(bodyA.w.toLane().Mul(rA.x)),
			}
			vt := dotW(vec2W{x: vrB.x.Sub(vrA.x), y: vrB.y.Sub(vrA.y)}, tangent).Sub(constraint.tangentSpeed)
			impulse := constraint.tangentMass1.Mul(negOne.Mul(vt))

			maxFriction := constraint.friction.Mul(constraint.normalImpulse1.toLane())
			newImpulse := constraint.tangentImpulse1.toLane().Add(impulse)
			newImpulse = negOne.Mul(maxFriction).Max(maxFriction.Min(newImpulse))
			impulse = newImpulse.Sub(constraint.tangentImpulse1.toLane())
			constraint.tangentImpulse1 = newImpulse.toAcc()

			p := vec2W{x: tangent.x.Mul(impulse), y: tangent.y.Mul(impulse)}
			bodyA.v.x = bodyA.v.x.Sub(constraint.invMassA.Mul(p.x).toAcc())
			bodyA.v.y = bodyA.v.y.Sub(constraint.invMassA.Mul(p.y).toAcc())
			bodyA.w = bodyA.w.Sub(constraint.invIA.Mul(crossW(rA, p)).toAcc())
			bodyB.v.x = bodyB.v.x.Add(constraint.invMassB.Mul(p.x).toAcc())
			bodyB.v.y = bodyB.v.y.Add(constraint.invMassB.Mul(p.y).toAcc())
			bodyB.w = bodyB.w.Add(constraint.invIB.Mul(crossW(rB, p)).toAcc())
		}

		{
			rA := constraint.anchorA2
			rB := constraint.anchorB2
			negWA := negOne.Mul(bodyA.w.toLane())
			negWB := negOne.Mul(bodyB.w.toLane())
			vrB := vec2W{
				x: bodyB.v.x.toLane().Add(negWB.Mul(rB.y)),
				y: bodyB.v.y.toLane().Add(bodyB.w.toLane().Mul(rB.x)),
			}
			vrA := vec2W{
				x: bodyA.v.x.toLane().Add(negWA.Mul(rA.y)),
				y: bodyA.v.y.toLane().Add(bodyA.w.toLane().Mul(rA.x)),
			}
			vt := dotW(vec2W{x: vrB.x.Sub(vrA.x), y: vrB.y.Sub(vrA.y)}, tangent).Sub(constraint.tangentSpeed)
			impulse := constraint.tangentMass2.Mul(negOne.Mul(vt))

			maxFriction := constraint.friction.Mul(constraint.normalImpulse2.toLane())
			newImpulse := constraint.tangentImpulse2.toLane().Add(impulse)
			newImpulse = negOne.Mul(maxFriction).Max(maxFriction.Min(newImpulse))
			impulse = newImpulse.Sub(constraint.tangentImpulse2.toLane())
			constraint.tangentImpulse2 = newImpulse.toAcc()

			p := vec2W{x: tangent.x.Mul(impulse), y: tangent.y.Mul(impulse)}
			bodyA.v.x = bodyA.v.x.Sub(constraint.invMassA.Mul(p.x).toAcc())
			bodyA.v.y = bodyA.v.y.Sub(constraint.invMassA.Mul(p.y).toAcc())
			bodyA.w = bodyA.w.Sub(constraint.invIA.Mul(crossW(rA, p)).toAcc())
			bodyB.v.x = bodyB.v.x.Add(constraint.invMassB.Mul(p.x).toAcc())
			bodyB.v.y = bodyB.v.y.Add(constraint.invMassB.Mul(p.y).toAcc())
			bodyB.w = bodyB.w.Add(constraint.invIB.Mul(crossW(rB, p)).toAcc())
		}

		deltaLambda := negOne.Mul(constraint.rollingMass).Mul(bodyB.w.Sub(bodyA.w).toLane())
		lambda := constraint.rollingImpulse
		maxLambda := constraint.rollingResistance.Mul(totalNormalImpulse.toLane())
		newRollingImpulse := lambda.toLane().Add(deltaLambda)
		newRollingImpulse = negOne.Mul(maxLambda).Max(maxLambda.Min(newRollingImpulse))
		constraint.rollingImpulse = newRollingImpulse.toAcc()
		deltaLambdaAcc := constraint.rollingImpulse.Sub(lambda)
		bodyA.w = bodyA.w.Sub(constraint.invIA.Mul(deltaLambdaAcc.toLane()).toAcc())
		bodyB.w = bodyB.w.Add(constraint.invIB.Mul(deltaLambdaAcc.toLane()).toAcc())

		scatterBodyW(states, &constraint.indexA, tauW, &bodyA)
		scatterBodyW(states, &constraint.indexB, tauW, &bodyB)
	}
}

// applyRestitutionTaskWide applies restitution masks lane-wise. It corresponds
// to b2ApplyRestitutionTask in src/contact_solver.c.
func applyRestitutionTaskWide(startIndex, endIndex int, context *stepContext, colorIndex int) {
	states := context.states
	constraints := context.graph.colors[colorIndex].contactConstraintsWide
	threshold := laneSplat(context.world.restitutionThreshold)
	zero := laneZero()
	one := laneSplat(QOne())
	negOne := laneSplat(Q{v: -1})
	tauW := laneSplat(tau)

	for i := startIndex; i < endIndex; i++ {
		constraint := &constraints[i]
		restitutionMask := constraint.restitution.Equals(zero)
		nonZeroMask := laneBlend(restitutionMask, zero, one).Greater(zero)
		if nonZeroMask.AllZero() {
			continue
		}

		var bodyA, bodyB bodyStateW
		gatherBodyW(states, &constraint.indexA, tauW, &bodyA)
		gatherBodyW(states, &constraint.indexB, tauW, &bodyB)
		normal := constraint.normal

		{
			thresholdMask := constraint.relativeVelocity1.Add(threshold).Greater(zero)
			impulseMask := constraint.totalNormalImpulse1.toLane().Equals(zero)
			skipMask := thresholdMask.Or(impulseMask).Or(restitutionMask)
			mass := laneBlend(skipMask, zero, constraint.normalMass1)

			rA := constraint.anchorA1
			rB := constraint.anchorB1
			negWA := negOne.Mul(bodyA.w.toLane())
			negWB := negOne.Mul(bodyB.w.toLane())
			vrB := vec2W{
				x: bodyB.v.x.toLane().Add(negWB.Mul(rB.y)),
				y: bodyB.v.y.toLane().Add(bodyB.w.toLane().Mul(rB.x)),
			}
			vrA := vec2W{
				x: bodyA.v.x.toLane().Add(negWA.Mul(rA.y)),
				y: bodyA.v.y.toLane().Add(bodyA.w.toLane().Mul(rA.x)),
			}
			vn := dotW(vec2W{x: vrB.x.Sub(vrA.x), y: vrB.y.Sub(vrA.y)}, normal)
			impulse := negOne.Mul(mass).Mul(vn.Add(constraint.restitution.Mul(constraint.relativeVelocity1)))

			newImpulse := constraint.normalImpulse1.toLane().Add(impulse).Max(zero)
			impulse = newImpulse.Sub(constraint.normalImpulse1.toLane())
			constraint.normalImpulse1 = newImpulse.toAcc()
			constraint.totalNormalImpulse1 = constraint.totalNormalImpulse1.Add(impulse.toAcc())

			p := vec2W{x: normal.x.Mul(impulse), y: normal.y.Mul(impulse)}
			bodyA.v.x = bodyA.v.x.Sub(constraint.invMassA.Mul(p.x).toAcc())
			bodyA.v.y = bodyA.v.y.Sub(constraint.invMassA.Mul(p.y).toAcc())
			bodyA.w = bodyA.w.Sub(constraint.invIA.Mul(crossW(rA, p)).toAcc())
			bodyB.v.x = bodyB.v.x.Add(constraint.invMassB.Mul(p.x).toAcc())
			bodyB.v.y = bodyB.v.y.Add(constraint.invMassB.Mul(p.y).toAcc())
			bodyB.w = bodyB.w.Add(constraint.invIB.Mul(crossW(rB, p)).toAcc())
		}

		{
			thresholdMask := constraint.relativeVelocity2.Add(threshold).Greater(zero)
			impulseMask := constraint.totalNormalImpulse2.toLane().Equals(zero)
			skipMask := thresholdMask.Or(impulseMask).Or(restitutionMask)
			mass := laneBlend(skipMask, zero, constraint.normalMass2)

			rA := constraint.anchorA2
			rB := constraint.anchorB2
			negWA := negOne.Mul(bodyA.w.toLane())
			negWB := negOne.Mul(bodyB.w.toLane())
			vrB := vec2W{
				x: bodyB.v.x.toLane().Add(negWB.Mul(rB.y)),
				y: bodyB.v.y.toLane().Add(bodyB.w.toLane().Mul(rB.x)),
			}
			vrA := vec2W{
				x: bodyA.v.x.toLane().Add(negWA.Mul(rA.y)),
				y: bodyA.v.y.toLane().Add(bodyA.w.toLane().Mul(rA.x)),
			}
			vn := dotW(vec2W{x: vrB.x.Sub(vrA.x), y: vrB.y.Sub(vrA.y)}, normal)
			impulse := negOne.Mul(mass).Mul(vn.Add(constraint.restitution.Mul(constraint.relativeVelocity2)))

			newImpulse := constraint.normalImpulse2.toLane().Add(impulse).Max(zero)
			impulse = newImpulse.Sub(constraint.normalImpulse2.toLane())
			constraint.normalImpulse2 = newImpulse.toAcc()
			constraint.totalNormalImpulse2 = constraint.totalNormalImpulse2.Add(impulse.toAcc())

			p := vec2W{x: normal.x.Mul(impulse), y: normal.y.Mul(impulse)}
			bodyA.v.x = bodyA.v.x.Sub(constraint.invMassA.Mul(p.x).toAcc())
			bodyA.v.y = bodyA.v.y.Sub(constraint.invMassA.Mul(p.y).toAcc())
			bodyA.w = bodyA.w.Sub(constraint.invIA.Mul(crossW(rA, p)).toAcc())
			bodyB.v.x = bodyB.v.x.Add(constraint.invMassB.Mul(p.x).toAcc())
			bodyB.v.y = bodyB.v.y.Add(constraint.invMassB.Mul(p.y).toAcc())
			bodyB.w = bodyB.w.Add(constraint.invIB.Mul(crossW(rB, p)).toAcc())
		}

		scatterBodyW(states, &constraint.indexA, tauW, &bodyA)
		scatterBodyW(states, &constraint.indexB, tauW, &bodyB)
	}
}

// storeImpulsesTaskWide stores scalar lanes back into real manifolds. It
// corresponds to b2StoreImpulsesTask in src/contact_solver.c.
func storeImpulsesTaskWide(startIndex, endIndex int, context *stepContext) {
	for constraintIndex := startIndex; constraintIndex < endIndex; constraintIndex++ {
		constraint := &context.contactConstraintsWide[constraintIndex]
		var (
			rollingImpulse                           [wideWidth]float32
			normalImpulse1, normalImpulse2           [wideWidth]float32
			tangentImpulse1, tangentImpulse2         [wideWidth]float32
			totalNormalImpulse1, totalNormalImpulse2 [wideWidth]float32
			relativeVelocity1, relativeVelocity2     [wideWidth]float32
		)
		constraint.rollingImpulse.toLane().store(&rollingImpulse)
		constraint.normalImpulse1.toLane().store(&normalImpulse1)
		constraint.normalImpulse2.toLane().store(&normalImpulse2)
		constraint.tangentImpulse1.toLane().store(&tangentImpulse1)
		constraint.tangentImpulse2.toLane().store(&tangentImpulse2)
		constraint.totalNormalImpulse1.toLane().store(&totalNormalImpulse1)
		constraint.totalNormalImpulse2.toLane().store(&totalNormalImpulse2)
		constraint.relativeVelocity1.store(&relativeVelocity1)
		constraint.relativeVelocity2.store(&relativeVelocity2)

		baseIndex := wideWidth * constraintIndex
		for laneIndex := range wideWidth {
			contact := context.contacts[baseIndex+laneIndex]
			if contact == nil {
				continue
			}

			manifold := &contact.manifold
			for pointIndex := range manifold.PointCount {
				if pointIndex == 0 {
					manifold.Points[pointIndex].NormalImpulse = Q{v: normalImpulse1[laneIndex]}
					manifold.Points[pointIndex].TangentImpulse = Q{v: tangentImpulse1[laneIndex]}
					manifold.Points[pointIndex].TotalNormalImpulse = Q{v: totalNormalImpulse1[laneIndex]}
					manifold.Points[pointIndex].NormalVelocity = Q{v: relativeVelocity1[laneIndex]}
				} else {
					manifold.Points[pointIndex].NormalImpulse = Q{v: normalImpulse2[laneIndex]}
					manifold.Points[pointIndex].TangentImpulse = Q{v: tangentImpulse2[laneIndex]}
					manifold.Points[pointIndex].TotalNormalImpulse = Q{v: totalNormalImpulse2[laneIndex]}
					manifold.Points[pointIndex].NormalVelocity = Q{v: relativeVelocity2[laneIndex]}
				}
			}
			manifold.RollingImpulse = Q{v: rollingImpulse[laneIndex]}
		}
	}
}
