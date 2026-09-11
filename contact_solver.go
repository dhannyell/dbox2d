package dbox2d

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
	anchorA, anchorB   vec2c
	baseSeparation     qc
	relativeVelocity   qc
	normalImpulse      qc
	tangentImpulse     qc
	totalNormalImpulse qa
	normalMass         qc
	tangentMass        qc
}

// contactConstraint is the solver view of one touching contact. It
// corresponds to b2ContactConstraint in src/contact_solver.h.
type contactConstraint struct {
	indexA             int
	indexB             int
	points             [2]contactConstraintPoint
	normal             vec2c
	invMassA, invMassB qc
	invIA, invIB       qc
	friction           qc
	restitution        qc
	tangentSpeed       qc
	rollingResistance  qc
	rollingMass        qc
	rollingImpulse     qc
	softness           contactSoft
	pointCount         int
}

// contactSoft is a softness on the contact grid.
type contactSoft struct {
	biasRate, massScale, impulseScale qc
}

func contactSoftFrom(s softness) contactSoft {
	return contactSoft{biasRate: qcFrom(s.biasRate), massScale: qcFrom(s.massScale), impulseScale: qcFrom(s.impulseScale)}
}

// The prepare stage computes in Q and rounds each result to the contact
// grid. The other stages compute on the grid and accumulate the body
// velocities and the total normal impulse in qa: the same grid with more
// integer range, so a sum does not saturate.

// D-004: the body state keeps the angular velocity in turns per second.
// The solver works in radians per second, so each stage scales the
// velocity by one turn on load and divides by one turn on store.

// contactBody is the velocity of one body while a stage solves a contact.
type contactBody struct {
	vx, vy, w qa
}

func loadContactBody(state *bodyState) contactBody {
	return contactBody{
		vx: qaFrom(state.linearVelocity.X),
		vy: qaFrom(state.linearVelocity.Y),
		w:  qaFrom(state.angularVelocity.Mul(tau)),
	}
}

func (b *contactBody) store(state *bodyState) {
	state.linearVelocity = Vec2{X: b.vx.toQ(), Y: b.vy.toQ()}
	state.angularVelocity = b.w.toQ().Div(tau)
}

// velocityAt returns the velocity of the body at the anchor r.
func (b *contactBody) velocityAt(r vec2c) vec2c {
	w := b.w.narrow()
	return vec2c{X: b.vx.narrow().Add(w.Neg().Mul(r.Y)), Y: b.vy.narrow().Add(w.Mul(r.X))}
}

// applyContactImpulse applies the impulse p at the anchors rA and rB.
func applyContactImpulse(bodyA, bodyB *contactBody, constraint *contactConstraint, rA, rB, p vec2c) {
	bodyA.vx = bodyA.vx.Sub(constraint.invMassA.Mul(p.X).widen())
	bodyA.vy = bodyA.vy.Sub(constraint.invMassA.Mul(p.Y).widen())
	bodyA.w = bodyA.w.Sub(constraint.invIA.Mul(crossc(rA, p)).widen())
	bodyB.vx = bodyB.vx.Add(constraint.invMassB.Mul(p.X).widen())
	bodyB.vy = bodyB.vy.Add(constraint.invMassB.Mul(p.Y).widen())
	bodyB.w = bodyB.w.Add(constraint.invIB.Mul(crossc(rB, p)).widen())
}

func vec2cFrom(v Vec2) vec2c { return vec2c{X: qcFrom(v.X), Y: qcFrom(v.Y)} }

func rotcFrom(r Rot) rotc { return rotc{Sin: qcFrom(r.Sin), Cos: qcFrom(r.Cos)} }

func crossc(a, b vec2c) qc { return a.X.Mul(b.Y).Sub(a.Y.Mul(b.X)) }

// The scalar family serves every color. Contacts of one active color share
// no dynamic body, while overflow stages remain whole-color operations.

// prepareContactsTask builds constraints for a range of the flat contact
// array. It corresponds to b2PrepareContactsTask in src/contact_solver.c.
func prepareContactsTask(startIndex, endIndex int, context *stepContext) {
	prepareContactRange(startIndex, endIndex, context, context.contacts, nil, context.contactConstraints)
}

// prepareOverflowContacts builds the overflow contact constraints. It
// corresponds to b2PrepareOverflowContacts in src/contact_solver.c.
func prepareOverflowContacts(context *stepContext) {
	color := &context.graph.colors[overflowIndex]
	prepareContactRange(0, len(color.contactSims), context, nil, color.contactSims, color.contactConstraints)
}

func prepareContactRange(startIndex, endIndex int, context *stepContext, contacts []*contactSim, contactSims []contactSim, constraints []contactConstraint) {
	w := context.world
	awakeStates := context.states
	constraints = constraints[startIndex:endIndex]
	if contacts != nil {
		contacts = contacts[startIndex:endIndex]
	} else {
		contactSims = contactSims[startIndex:endIndex]
	}

	// Stiffer for static contacts to avoid bodies getting pushed through the ground
	contactSoftness := contactSoftFrom(context.contactSoftness)
	staticSoftness := contactSoftFrom(context.staticSoftness)

	zero := QZero()
	one := QOne()
	warmStartScale := zero
	if w.enableWarmStarting {
		warmStartScale = one
	}

	for i := range endIndex - startIndex {
		var cs *contactSim
		if contacts != nil {
			cs = contacts[i]
		} else {
			cs = &contactSims[i]
		}

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
		constraint.normal = vec2cFrom(manifold.Normal)
		constraint.friction = qcFrom(cs.friction)
		constraint.restitution = qcFrom(cs.restitution)
		constraint.rollingResistance = qcFrom(cs.rollingResistance)
		constraint.rollingImpulse = qcFrom(warmStartScale.Mul(manifold.RollingImpulse))
		constraint.tangentSpeed = qcFrom(cs.tangentSpeed)
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
		constraint.invMassA = qcFrom(mA)
		constraint.invIA = qcFrom(iA)
		constraint.invMassB = qcFrom(mB)
		constraint.invIB = qcFrom(iB)

		// D-006: the effective masses store the reciprocal once, as the
		// inverse mass of a body does.
		{
			k := iA.Add(iB)
			constraint.rollingMass = qc{}
			if zero.Less(k) {
				constraint.rollingMass = qcFrom(one.Div(k))
			}
		}

		normal := manifold.Normal
		tangent := RightPerp(normal)

		for j := range pointCount {
			mp := &manifold.Points[j]
			cp := &constraint.points[j]

			cp.normalImpulse = qcFrom(warmStartScale.Mul(mp.NormalImpulse))
			cp.tangentImpulse = qcFrom(warmStartScale.Mul(mp.TangentImpulse))
			cp.totalNormalImpulse = qa{}

			rA := mp.AnchorA
			rB := mp.AnchorB

			cp.anchorA = vec2cFrom(rA)
			cp.anchorB = vec2cFrom(rB)
			cp.baseSeparation = qcFrom(mp.Separation.Sub(rB.Sub(rA).Dot(normal)))

			rnA := Cross(rA, normal)
			rnB := Cross(rB, normal)
			kNormal := mA.Add(mB).Add(iA.Mul(rnA).Mul(rnA)).Add(iB.Mul(rnB).Mul(rnB))
			cp.normalMass = qc{}
			if zero.Less(kNormal) {
				cp.normalMass = qcFrom(one.Div(kNormal))
			}

			rtA := Cross(rA, tangent)
			rtB := Cross(rB, tangent)
			kTangent := mA.Add(mB).Add(iA.Mul(rtA).Mul(rtA)).Add(iB.Mul(rtB).Mul(rtB))
			cp.tangentMass = qc{}
			if zero.Less(kTangent) {
				cp.tangentMass = qcFrom(one.Div(kTangent))
			}

			// Save relative velocity for restitution
			vrA := vA.Add(CrossSV(wA, rA))
			vrB := vB.Add(CrossSV(wB, rB))
			cp.relativeVelocity = qcFrom(normal.Dot(vrB.Sub(vrA)))
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

// warmStartContactsTask applies stored impulses to a contact range. It
// corresponds to b2WarmStartContactsTask in src/contact_solver.c.
func warmStartContactsTask(startIndex, endIndex int, context *stepContext, colorIndex int) {
	constraints := context.graph.colors[colorIndex].contactConstraints
	warmStartContactRange(startIndex, endIndex, context, constraints)
}

// warmStartOverflowContacts applies stored impulses to the overflow color.
// It corresponds to b2WarmStartOverflowContacts in src/contact_solver.c.
func warmStartOverflowContacts(context *stepContext) {
	constraints := context.graph.colors[overflowIndex].contactConstraints
	warmStartContactRange(0, len(constraints), context, constraints)
}

func warmStartContactRange(startIndex, endIndex int, context *stepContext, constraints []contactConstraint) {
	w := context.world
	awake := &w.solverSets[awakeSet]
	states := awake.bodyStates
	constraints = constraints[startIndex:endIndex]

	// This is a dummy state to represent a static body because static bodies don't have a solver body.
	dummyState := identityBodyState()

	for i := range endIndex - startIndex {
		constraint := &constraints[i]

		stateA, stateB := constraintStates(states, &dummyState, constraint)
		bodyA := loadContactBody(stateA)
		bodyB := loadContactBody(stateB)

		// Stiffer for static contacts to avoid bodies getting pushed through the ground
		normal := constraint.normal
		tangent := vec2c{X: normal.Y, Y: normal.X.Neg()}
		pointCount := constraint.pointCount

		for j := range pointCount {
			cp := &constraint.points[j]

			// fixed anchors
			rA := cp.anchorA
			rB := cp.anchorB

			P := normal.Mul(cp.normalImpulse).Add(tangent.Mul(cp.tangentImpulse))
			applyContactImpulse(&bodyA, &bodyB, constraint, rA, rB, P)
		}

		bodyA.w = bodyA.w.Sub(constraint.invIA.Mul(constraint.rollingImpulse).widen())
		bodyB.w = bodyB.w.Add(constraint.invIB.Mul(constraint.rollingImpulse).widen())

		bodyA.store(stateA)
		bodyB.store(stateB)
	}
}

// solveContactsTask solves a contact range. It corresponds to
// b2SolveContactsTask in src/contact_solver.c.
func solveContactsTask(startIndex, endIndex int, context *stepContext, colorIndex int, useBias bool) {
	constraints := context.graph.colors[colorIndex].contactConstraints
	// Colored contacts clamp by the contact speed, per b2SolveContactsTask.
	solveContactRange(startIndex, endIndex, context, constraints, useBias, qcFrom(context.world.contactSpeed))
}

// solveOverflowContacts solves the overflow contacts. It corresponds to
// b2SolveOverflowContacts in src/contact_solver.c.
func solveOverflowContacts(context *stepContext, useBias bool) {
	constraints := context.graph.colors[overflowIndex].contactConstraints
	// Overflow contacts clamp by the push speed, per b2SolveOverflowContacts.
	solveContactRange(0, len(constraints), context, constraints, useBias, qcFrom(context.world.maxContactPushSpeed))
}

func solveContactRange(startIndex, endIndex int, context *stepContext, constraints []contactConstraint, useBias bool, pushout qc) {
	w := context.world
	awake := &w.solverSets[awakeSet]
	states := awake.bodyStates
	constraints = constraints[startIndex:endIndex]

	invH := qcFrom(context.invH)

	// This is a dummy body to represent a static body since static bodies don't have a solver body.
	dummyState := identityBodyState()

	zero := qc{}
	one := qcFrom(QOne())
	for i := range endIndex - startIndex {
		constraint := &constraints[i]

		stateA, stateB := constraintStates(states, &dummyState, constraint)
		bodyA := loadContactBody(stateA)
		dqA := rotcFrom(stateA.deltaRotation)

		bodyB := loadContactBody(stateB)
		dqB := rotcFrom(stateB.deltaRotation)

		dp := vec2cFrom(stateB.deltaPosition).Sub(vec2cFrom(stateA.deltaPosition))

		normal := constraint.normal
		tangent := vec2c{X: normal.Y, Y: normal.X.Neg()}
		friction := constraint.friction
		soft := constraint.softness

		pointCount := constraint.pointCount
		totalNormalImpulse := qa{}

		// Non-penetration
		for j := range pointCount {
			cp := &constraint.points[j]

			// fixed anchor points
			rA := cp.anchorA
			rB := cp.anchorB

			// compute current separation
			// this is subject to round-off error if the anchor is far from the body center of mass
			ds := dp.Add(dqB.Apply(rB).Sub(dqA.Apply(rA)))
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
			vn := bodyB.velocityAt(rB).Sub(bodyA.velocityAt(rA)).Dot(normal)

			// incremental normal impulse
			impulse := cp.normalMass.Neg().Mul(massScale).Mul(vn.Add(velocityBias)).Sub(impulseScale.Mul(cp.normalImpulse))

			// clamp the accumulated impulse
			newImpulse := cp.normalImpulse.Add(impulse).Max(zero)
			impulse = newImpulse.Sub(cp.normalImpulse)
			cp.normalImpulse = newImpulse
			cp.totalNormalImpulse = cp.totalNormalImpulse.Add(newImpulse.widen())
			totalNormalImpulse = totalNormalImpulse.Add(newImpulse.widen())

			// apply normal impulse
			applyContactImpulse(&bodyA, &bodyB, constraint, rA, rB, normal.Mul(impulse))
		}

		// Friction
		for j := range pointCount {
			cp := &constraint.points[j]

			// fixed anchor points
			rA := cp.anchorA
			rB := cp.anchorB

			// relative tangent velocity at contact
			// vt = dot(vrB - sB * tangent - (vrA + sA * tangent), tangent)
			//    = dot(vrB - vrA, tangent) - (sA + sB)
			vt := bodyB.velocityAt(rB).Sub(bodyA.velocityAt(rA)).Dot(tangent).Sub(constraint.tangentSpeed)

			// incremental tangent impulse
			impulse := cp.tangentMass.Mul(vt.Neg())

			// clamp the accumulated force
			maxFriction := friction.Mul(cp.normalImpulse)
			newImpulse := cp.tangentImpulse.Add(impulse).Clamp(maxFriction.Neg(), maxFriction)
			impulse = newImpulse.Sub(cp.tangentImpulse)
			cp.tangentImpulse = newImpulse

			// apply tangent impulse
			applyContactImpulse(&bodyA, &bodyB, constraint, rA, rB, tangent.Mul(impulse))
		}

		// Rolling resistance
		{
			deltaLambda := constraint.rollingMass.Neg().Mul(bodyB.w.Sub(bodyA.w).narrow())
			lambda := constraint.rollingImpulse
			maxLambda := rollingBound(constraint.rollingResistance, totalNormalImpulse)
			constraint.rollingImpulse = lambda.Add(deltaLambda).Clamp(maxLambda.Neg(), maxLambda)
			deltaLambda = constraint.rollingImpulse.Sub(lambda)

			bodyA.w = bodyA.w.Sub(constraint.invIA.Mul(deltaLambda).widen())
			bodyB.w = bodyB.w.Add(constraint.invIB.Mul(deltaLambda).widen())
		}

		bodyA.store(stateA)
		bodyB.store(stateB)
	}
}

// applyRestitutionTask applies restitution to a contact range. It
// corresponds to b2ApplyRestitutionTask in src/contact_solver.c.
func applyRestitutionTask(startIndex, endIndex int, context *stepContext, colorIndex int) {
	constraints := context.graph.colors[colorIndex].contactConstraints
	applyRestitutionRange(startIndex, endIndex, context, constraints)
}

// applyOverflowRestitution applies restitution to the overflow contacts.
// It corresponds to b2ApplyOverflowRestitution in src/contact_solver.c.
func applyOverflowRestitution(context *stepContext) {
	constraints := context.graph.colors[overflowIndex].contactConstraints
	applyRestitutionRange(0, len(constraints), context, constraints)
}

func applyRestitutionRange(startIndex, endIndex int, context *stepContext, constraints []contactConstraint) {
	w := context.world
	awake := &w.solverSets[awakeSet]
	states := awake.bodyStates
	constraints = constraints[startIndex:endIndex]

	threshold := qcFrom(w.restitutionThreshold)

	// dummy state to represent a static body
	dummyState := identityBodyState()

	zero := qc{}
	for i := range endIndex - startIndex {
		constraint := &constraints[i]

		restitution := constraint.restitution
		if restitution.Eq(zero) {
			continue
		}

		stateA, stateB := constraintStates(states, &dummyState, constraint)
		bodyA := loadContactBody(stateA)
		bodyB := loadContactBody(stateB)

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
				if threshold.Neg().Less(cp.relativeVelocity) || cp.totalNormalImpulse.Eq(qa{}) {
					continue
				}

				// fixed anchor points
				rA := cp.anchorA
				rB := cp.anchorB

				// relative normal velocity at contact
				vn := bodyB.velocityAt(rB).Sub(bodyA.velocityAt(rA)).Dot(normal)

				// compute normal impulse
				impulse := cp.normalMass.Neg().Mul(vn.Add(restitution.Mul(cp.relativeVelocity)))

				// clamp the accumulated impulse
				// todo should this be stored?
				newImpulse := cp.normalImpulse.Add(impulse).Max(zero)
				impulse = newImpulse.Sub(cp.normalImpulse)
				cp.normalImpulse = newImpulse

				// Add the incremental impulse rather than the full impulse because this is not a sub-step
				cp.totalNormalImpulse = cp.totalNormalImpulse.Add(impulse.widen())

				// apply contact impulse
				applyContactImpulse(&bodyA, &bodyB, constraint, rA, rB, normal.Mul(impulse))
			}
		}

		bodyA.store(stateA)
		bodyB.store(stateB)
	}
}

// storeImpulsesTask stores a range of flat contact impulses. It corresponds
// to b2StoreImpulsesTask in src/contact_solver.c.
func storeImpulsesTask(startIndex, endIndex int, context *stepContext) {
	storeImpulseRange(startIndex, endIndex, context.contacts, nil, context.contactConstraints)
}

// storeOverflowImpulses stores the overflow contact impulses. It
// corresponds to b2StoreOverflowImpulses in src/contact_solver.c.
func storeOverflowImpulses(context *stepContext) {
	color := &context.graph.colors[overflowIndex]
	storeImpulseRange(0, len(color.contactSims), nil, color.contactSims, color.contactConstraints)
}

func storeImpulseRange(startIndex, endIndex int, contacts []*contactSim, contactSims []contactSim, constraints []contactConstraint) {
	constraints = constraints[startIndex:endIndex]
	if contacts != nil {
		contacts = contacts[startIndex:endIndex]
	} else {
		contactSims = contactSims[startIndex:endIndex]
	}

	for i := range endIndex - startIndex {
		constraint := &constraints[i]
		var contact *contactSim
		if contacts != nil {
			contact = contacts[i]
		} else {
			contact = &contactSims[i]
		}
		manifold := &contact.manifold
		pointCount := manifold.PointCount

		for j := range pointCount {
			manifold.Points[j].NormalImpulse = constraint.points[j].normalImpulse.toQ()
			manifold.Points[j].TangentImpulse = constraint.points[j].tangentImpulse.toQ()
			manifold.Points[j].TotalNormalImpulse = constraint.points[j].totalNormalImpulse.toQ()
			manifold.Points[j].NormalVelocity = constraint.points[j].relativeVelocity.toQ()
		}

		manifold.RollingImpulse = constraint.rollingImpulse.toQ()
	}
}
