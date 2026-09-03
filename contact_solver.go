package dbox2d

import "github.com/dhannyell/fixed"

// contact separation for sub-stepping
// s = s0 + dot(cB + rB - cA - rA, normal)
// normal is held constant
// body positions c can translation and anchors r can rotate
// s(t) = s0 + dot(cB(t) + rB(t) - cA(t) - rA(t), normal)
// s(t) = s0 + dot(cB0 + dpB + rot(dqB, rB0) - cA0 - dpA - rot(dqA, rA0), normal)
// s(t) = s0 + dot(cB0 - cA0, normal) + dot(dpB - dpA + rot(dqB, rB0) - rot(dqA, rA0), normal)
// s_base = s0 + dot(cB0 - cA0, normal)

// contactConstraintPoint is the solver view of one manifold point. It
// corresponds to b2ContactConstraintPoint in src/contact_solver.h.
type contactConstraintPoint struct {
	anchorA, anchorB   Vec2
	baseSeparation     Q
	relativeVelocity   Q
	normalImpulse      Q
	tangentImpulse     Q
	totalNormalImpulse Q
	normalMass         Q
	tangentMass        Q
}

// contactConstraint is the solver view of one touching contact. It
// corresponds to b2ContactConstraint in src/contact_solver.h.
type contactConstraint struct {
	indexA             int
	indexB             int
	points             [2]contactConstraintPoint
	normal             Vec2
	invMassA, invMassB Q
	invIA, invIB       Q
	friction           Q
	restitution        Q
	tangentSpeed       Q
	rollingResistance  Q
	rollingMass        Q
	rollingImpulse     Q
	softness           softness
	pointCount         int
}

// D-004: the body state keeps the angular velocity in turns per second.
// The solver works in radians per second, so each stage scales the
// velocity by one turn on load and divides by one turn on store.

// The reference solves the overflow color with the scalar family below and
// the other colors with a SIMD family over eight contacts at a time. The
// port has one worker, so the scalar family serves every color. Contacts
// of one color share no dynamic body, so the order inside a color cannot
// change the result and the schedule of the reference holds.

// prepareContacts builds the constraints of one color from its contact
// sims. It corresponds to b2PrepareOverflowContacts in
// src/contact_solver.c.
func prepareContacts(context *stepContext, colorIndex int) {
	w := context.world
	graph := &w.constraintGraph
	color := &graph.colors[colorIndex]
	constraints := color.contactConstraints
	contacts := color.contactSims
	awakeStates := context.states

	// Stiffer for static contacts to avoid bodies getting pushed through the ground
	contactSoftness := context.contactSoftness
	staticSoftness := context.staticSoftness

	zero := fixed.Q32Zero()
	one := fixed.Q32One()
	warmStartScale := zero
	if w.enableWarmStarting {
		warmStartScale = one
	}

	for i := range contacts {
		cs := &contacts[i]

		manifold := &cs.manifold
		pointCount := manifold.PointCount

		if pointCount <= 0 || pointCount > 2 {
			panic("dbox2d: the manifold point count is out of range")
		}

		indexA := cs.bodySimIndexA
		indexB := cs.bodySimIndexB

		constraint := &constraints[i]
		constraint.indexA = indexA
		constraint.indexB = indexB
		constraint.normal = manifold.Normal
		constraint.friction = cs.friction
		constraint.restitution = cs.restitution
		constraint.rollingResistance = cs.rollingResistance
		constraint.rollingImpulse = warmStartScale.Mul(manifold.RollingImpulse)
		constraint.tangentSpeed = cs.tangentSpeed
		constraint.pointCount = pointCount

		vA := Vec2Zero()
		wA := zero
		mA := cs.invMassA
		iA := cs.invIA
		if indexA != nullIndex {
			stateA := &awakeStates[indexA]
			vA = stateA.linearVelocity
			wA = stateA.angularVelocity.Mul(tau)
		}

		vB := Vec2Zero()
		wB := zero
		mB := cs.invMassB
		iB := cs.invIB
		if indexB != nullIndex {
			stateB := &awakeStates[indexB]
			vB = stateB.linearVelocity
			wB = stateB.angularVelocity.Mul(tau)
		}

		if indexA == nullIndex || indexB == nullIndex {
			constraint.softness = staticSoftness
		} else {
			constraint.softness = contactSoftness
		}

		// copy mass into constraint to avoid cache misses during sub-stepping
		constraint.invMassA = mA
		constraint.invIA = iA
		constraint.invMassB = mB
		constraint.invIB = iB

		// D-006: the effective masses store the reciprocal once, as the
		// inverse mass of a body does.
		{
			k := iA.Add(iB)
			constraint.rollingMass = zero
			if zero.Less(k) {
				constraint.rollingMass = one.Div(k)
			}
		}

		normal := constraint.normal
		tangent := RightPerp(constraint.normal)

		for j := range pointCount {
			mp := &manifold.Points[j]
			cp := &constraint.points[j]

			cp.normalImpulse = warmStartScale.Mul(mp.NormalImpulse)
			cp.tangentImpulse = warmStartScale.Mul(mp.TangentImpulse)
			cp.totalNormalImpulse = zero

			rA := mp.AnchorA
			rB := mp.AnchorB

			cp.anchorA = rA
			cp.anchorB = rB
			cp.baseSeparation = mp.Separation.Sub(rB.Sub(rA).Dot(normal))

			rnA := Cross(rA, normal)
			rnB := Cross(rB, normal)
			kNormal := mA.Add(mB).Add(iA.Mul(rnA).Mul(rnA)).Add(iB.Mul(rnB).Mul(rnB))
			cp.normalMass = zero
			if zero.Less(kNormal) {
				cp.normalMass = one.Div(kNormal)
			}

			rtA := Cross(rA, tangent)
			rtB := Cross(rB, tangent)
			kTangent := mA.Add(mB).Add(iA.Mul(rtA).Mul(rtA)).Add(iB.Mul(rtB).Mul(rtB))
			cp.tangentMass = zero
			if zero.Less(kTangent) {
				cp.tangentMass = one.Div(kTangent)
			}

			// Save relative velocity for restitution
			vrA := vA.Add(CrossSV(wA, rA))
			vrB := vB.Add(CrossSV(wB, rB))
			cp.relativeVelocity = normal.Dot(vrB.Sub(vrA))
		}
	}
}

// constraintStates returns the two body states of a constraint. A static
// side gets the dummy state, because static bodies have no solver body.
func constraintStates(states []bodyState, dummy *bodyState, constraint *contactConstraint) (stateA, stateB *bodyState) {
	stateA = dummy
	if constraint.indexA != nullIndex {
		stateA = &states[constraint.indexA]
	}
	stateB = dummy
	if constraint.indexB != nullIndex {
		stateB = &states[constraint.indexB]
	}
	return stateA, stateB
}

// warmStartContacts applies the impulses of the previous step. It
// corresponds to b2WarmStartOverflowContacts in src/contact_solver.c.
func warmStartContacts(context *stepContext, colorIndex int) {
	w := context.world
	graph := &w.constraintGraph
	color := &graph.colors[colorIndex]
	constraints := color.contactConstraints
	contactCount := len(color.contactSims)
	awake := &w.solverSets[awakeSet]
	states := awake.bodyStates

	// This is a dummy state to represent a static body because static bodies don't have a solver body.
	dummyState := identityBodyState()

	for i := range contactCount {
		constraint := &constraints[i]

		stateA, stateB := constraintStates(states, &dummyState, constraint)

		vA := stateA.linearVelocity
		wA := stateA.angularVelocity.Mul(tau)
		vB := stateB.linearVelocity
		wB := stateB.angularVelocity.Mul(tau)

		mA := constraint.invMassA
		iA := constraint.invIA
		mB := constraint.invMassB
		iB := constraint.invIB

		// Stiffer for static contacts to avoid bodies getting pushed through the ground
		normal := constraint.normal
		tangent := RightPerp(constraint.normal)
		pointCount := constraint.pointCount

		for j := range pointCount {
			cp := &constraint.points[j]

			// fixed anchors
			rA := cp.anchorA
			rB := cp.anchorB

			P := normal.Mul(cp.normalImpulse).Add(tangent.Mul(cp.tangentImpulse))
			wA = wA.Sub(iA.Mul(Cross(rA, P)))
			vA = MulAdd(vA, mA.Neg(), P)
			wB = wB.Add(iB.Mul(Cross(rB, P)))
			vB = MulAdd(vB, mB, P)
		}

		wA = wA.Sub(iA.Mul(constraint.rollingImpulse))
		wB = wB.Add(iB.Mul(constraint.rollingImpulse))

		stateA.linearVelocity = vA
		stateA.angularVelocity = wA.Div(tau)
		stateB.linearVelocity = vB
		stateB.angularVelocity = wB.Div(tau)
	}
}

// solveContacts runs one iteration of non-penetration, friction and
// rolling resistance. With useBias the soft constraint pushes the bodies
// apart; without it the pass only relaxes the velocities. It corresponds
// to b2SolveOverflowContacts in src/contact_solver.c.
func solveContacts(context *stepContext, colorIndex int, useBias bool) {
	w := context.world
	graph := &w.constraintGraph
	color := &graph.colors[colorIndex]
	constraints := color.contactConstraints
	contactCount := len(color.contactSims)
	awake := &w.solverSets[awakeSet]
	states := awake.bodyStates

	invH := context.invH
	pushout := w.maxContactPushSpeed

	// This is a dummy body to represent a static body since static bodies don't have a solver body.
	dummyState := identityBodyState()

	zero := fixed.Q32Zero()
	one := fixed.Q32One()
	for i := range contactCount {
		constraint := &constraints[i]
		mA := constraint.invMassA
		iA := constraint.invIA
		mB := constraint.invMassB
		iB := constraint.invIB

		stateA, stateB := constraintStates(states, &dummyState, constraint)
		vA := stateA.linearVelocity
		wA := stateA.angularVelocity.Mul(tau)
		dqA := stateA.deltaRotation

		vB := stateB.linearVelocity
		wB := stateB.angularVelocity.Mul(tau)
		dqB := stateB.deltaRotation

		dp := stateB.deltaPosition.Sub(stateA.deltaPosition)

		normal := constraint.normal
		tangent := RightPerp(normal)
		friction := constraint.friction
		soft := constraint.softness

		pointCount := constraint.pointCount
		totalNormalImpulse := zero

		// Non-penetration
		for j := range pointCount {
			cp := &constraint.points[j]

			// fixed anchor points
			rA := cp.anchorA
			rB := cp.anchorB

			// compute current separation
			// this is subject to round-off error if the anchor is far from the body center of mass
			ds := dp.Add(RotateVector(dqB, rB).Sub(RotateVector(dqA, rA)))
			s := cp.baseSeparation.Add(ds.Dot(normal))

			velocityBias := zero
			massScale := one
			impulseScale := zero
			if zero.Less(s) {
				// speculative bias
				velocityBias = s.Mul(invH)
			} else if useBias {
				velocityBias = soft.biasRate.Mul(s).Max(pushout.Neg())
				massScale = soft.massScale
				impulseScale = soft.impulseScale
			}

			// relative normal velocity at contact
			vrA := vA.Add(CrossSV(wA, rA))
			vrB := vB.Add(CrossSV(wB, rB))
			vn := vrB.Sub(vrA).Dot(normal)

			// incremental normal impulse
			impulse := cp.normalMass.Neg().Mul(massScale).Mul(vn.Add(velocityBias)).Sub(impulseScale.Mul(cp.normalImpulse))

			// clamp the accumulated impulse
			newImpulse := cp.normalImpulse.Add(impulse).Max(zero)
			impulse = newImpulse.Sub(cp.normalImpulse)
			cp.normalImpulse = newImpulse
			cp.totalNormalImpulse = cp.totalNormalImpulse.Add(newImpulse)
			totalNormalImpulse = totalNormalImpulse.Add(newImpulse)

			// apply normal impulse
			P := normal.Mul(impulse)
			vA = MulSub(vA, mA, P)
			wA = wA.Sub(iA.Mul(Cross(rA, P)))

			vB = MulAdd(vB, mB, P)
			wB = wB.Add(iB.Mul(Cross(rB, P)))
		}

		// Friction
		for j := range pointCount {
			cp := &constraint.points[j]

			// fixed anchor points
			rA := cp.anchorA
			rB := cp.anchorB

			// relative tangent velocity at contact
			vrB := vB.Add(CrossSV(wB, rB))
			vrA := vA.Add(CrossSV(wA, rA))

			// vt = dot(vrB - sB * tangent - (vrA + sA * tangent), tangent)
			//    = dot(vrB - vrA, tangent) - (sA + sB)
			vt := vrB.Sub(vrA).Dot(tangent).Sub(constraint.tangentSpeed)

			// incremental tangent impulse
			impulse := cp.tangentMass.Mul(vt.Neg())

			// clamp the accumulated force
			maxFriction := friction.Mul(cp.normalImpulse)
			newImpulse := cp.tangentImpulse.Add(impulse).Clamp(maxFriction.Neg(), maxFriction)
			impulse = newImpulse.Sub(cp.tangentImpulse)
			cp.tangentImpulse = newImpulse

			// apply tangent impulse
			P := tangent.Mul(impulse)
			vA = MulSub(vA, mA, P)
			wA = wA.Sub(iA.Mul(Cross(rA, P)))
			vB = MulAdd(vB, mB, P)
			wB = wB.Add(iB.Mul(Cross(rB, P)))
		}

		// Rolling resistance
		{
			deltaLambda := constraint.rollingMass.Neg().Mul(wB.Sub(wA))
			lambda := constraint.rollingImpulse
			maxLambda := constraint.rollingResistance.Mul(totalNormalImpulse)
			constraint.rollingImpulse = lambda.Add(deltaLambda).Clamp(maxLambda.Neg(), maxLambda)
			deltaLambda = constraint.rollingImpulse.Sub(lambda)

			wA = wA.Sub(iA.Mul(deltaLambda))
			wB = wB.Add(iB.Mul(deltaLambda))
		}

		stateA.linearVelocity = vA
		stateA.angularVelocity = wA.Div(tau)
		stateB.linearVelocity = vB
		stateB.angularVelocity = wB.Div(tau)
	}
}

// applyRestitution adds the bounce after the sub-steps. Only a point that
// approached faster than the threshold and carried an impulse bounces. It
// corresponds to b2ApplyOverflowRestitution in src/contact_solver.c.
func applyRestitution(context *stepContext, colorIndex int) {
	w := context.world
	graph := &w.constraintGraph
	color := &graph.colors[colorIndex]
	constraints := color.contactConstraints
	contactCount := len(color.contactSims)
	awake := &w.solverSets[awakeSet]
	states := awake.bodyStates

	threshold := w.restitutionThreshold

	// dummy state to represent a static body
	dummyState := identityBodyState()

	zero := fixed.Q32Zero()
	for i := range contactCount {
		constraint := &constraints[i]

		restitution := constraint.restitution
		if restitution.Eq(zero) {
			continue
		}

		mA := constraint.invMassA
		iA := constraint.invIA
		mB := constraint.invMassB
		iB := constraint.invIB

		stateA, stateB := constraintStates(states, &dummyState, constraint)
		vA := stateA.linearVelocity
		wA := stateA.angularVelocity.Mul(tau)

		vB := stateB.linearVelocity
		wB := stateB.angularVelocity.Mul(tau)

		normal := constraint.normal
		pointCount := constraint.pointCount

		// it is possible to get more accurate restitution by iterating
		// this only makes a difference if there are two contact points
		// for (int iter = 0; iter < 10; ++iter)
		{
			for j := range pointCount {
				cp := &constraint.points[j]

				// if the normal impulse is zero then there was no collision
				// this skips speculative contact points that didn't generate an impulse
				// The max normal impulse is used in case there was a collision that moved away within the sub-step process
				if threshold.Neg().Less(cp.relativeVelocity) || cp.totalNormalImpulse.Eq(zero) {
					continue
				}

				// fixed anchor points
				rA := cp.anchorA
				rB := cp.anchorB

				// relative normal velocity at contact
				vrB := vB.Add(CrossSV(wB, rB))
				vrA := vA.Add(CrossSV(wA, rA))
				vn := vrB.Sub(vrA).Dot(normal)

				// compute normal impulse
				impulse := cp.normalMass.Neg().Mul(vn.Add(restitution.Mul(cp.relativeVelocity)))

				// clamp the accumulated impulse
				// todo should this be stored?
				newImpulse := cp.normalImpulse.Add(impulse).Max(zero)
				impulse = newImpulse.Sub(cp.normalImpulse)
				cp.normalImpulse = newImpulse

				// Add the incremental impulse rather than the full impulse because this is not a sub-step
				cp.totalNormalImpulse = cp.totalNormalImpulse.Add(impulse)

				// apply contact impulse
				P := normal.Mul(impulse)
				vA = MulSub(vA, mA, P)
				wA = wA.Sub(iA.Mul(Cross(rA, P)))
				vB = MulAdd(vB, mB, P)
				wB = wB.Add(iB.Mul(Cross(rB, P)))
			}
		}

		stateA.linearVelocity = vA
		stateA.angularVelocity = wA.Div(tau)
		stateB.linearVelocity = vB
		stateB.angularVelocity = wB.Div(tau)
	}
}

// storeImpulses writes the impulses back into the manifolds for the warm
// start of the next step. It corresponds to b2StoreOverflowImpulses in
// src/contact_solver.c.
func storeImpulses(context *stepContext, colorIndex int) {
	graph := &context.world.constraintGraph
	color := &graph.colors[colorIndex]
	constraints := color.contactConstraints
	contacts := color.contactSims

	for i := range contacts {
		constraint := &constraints[i]
		manifold := &contacts[i].manifold
		pointCount := manifold.PointCount

		for j := range pointCount {
			manifold.Points[j].NormalImpulse = constraint.points[j].normalImpulse
			manifold.Points[j].TangentImpulse = constraint.points[j].tangentImpulse
			manifold.Points[j].TotalNormalImpulse = constraint.points[j].totalNormalImpulse
			manifold.Points[j].NormalVelocity = constraint.points[j].relativeVelocity
		}

		manifold.RollingImpulse = constraint.rollingImpulse
	}
}

// Deferred: the Task family of the reference, which is the SIMD form of the
// same five stages. It arrives with the second executor.
