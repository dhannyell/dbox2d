package b2

// The fixed build also solves contacts on the Q32 grid, with the same code:
// contact_solver_q32.go is generated from this file.
//go:generate go run ./tools/q32gen

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
	restitutionImpulse qc
	totalNormalImpulse qa
	normalMass         qc
	tangentMass        qc
}

// contactConstraint is the solver view of one touching contact. It
// corresponds to b2ContactConstraint in src/contact_solver.h and to one
// lane of b2ContactConstraintWide in src/contact_solver_wide.inl.
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

// contactSoft is a softness on the contact grid. pushRate is the bias rate
// scaled by the mass scale, the product the push stage applies.
type contactSoft struct {
	pushRate, massScale, impulseScale qc
}

func contactSoftFrom(s softness) contactSoft {
	return contactSoft{pushRate: qcFrom(s.massScale.Mul(s.biasRate)), massScale: qcFrom(s.massScale), impulseScale: qcFrom(s.impulseScale)}
}

// The prepare stage computes in Q and rounds each result to the contact
// grid. The other stages compute on the grid and accumulate the body
// velocities and the total normal impulse in qa: the same grid with more
// integer range, so a sum does not saturate.

// D-004: the body state keeps the angular velocity in radians per second,
// the unit of the reference, so the stages load and store it unchanged.

// contactBody is the velocity of one body while a stage solves a contact.
type contactBody struct {
	vx, vy, w qa
}

func loadContactBody(state *bodyState) contactBody {
	return contactBody{
		vx: qaFrom(state.linearVelocity.X),
		vy: qaFrom(state.linearVelocity.Y),
		w:  qaFrom(state.angularVelocity),
	}
}

// store writes the velocity back to a dynamic body. A static or kinematic
// body keeps its velocity, so contacts may share it inside one color.
func (b *contactBody) store(state *bodyState) {
	if state.flags&bodyDynamicFlag == 0 {
		return
	}
	state.linearVelocity = Vec2{X: b.vx.toQ(), Y: b.vy.toQ()}
	state.angularVelocity = b.w.toQ()
}

// subImpulse applies the impulse (px, py) with the angular impulse to the
// first body of a contact.
func (b *contactBody) subImpulse(invMass, invI, px, py, angular qc) {
	b.vx = b.vx.Sub(invMass.Mul(px).widen())
	b.vy = b.vy.Sub(invMass.Mul(py).widen())
	b.w = b.w.Sub(invI.Mul(angular).widen())
}

// addImpulse applies the impulse (px, py) with the angular impulse to the
// second body of a contact.
func (b *contactBody) addImpulse(invMass, invI, px, py, angular qc) {
	b.vx = b.vx.Add(invMass.Mul(px).widen())
	b.vy = b.vy.Add(invMass.Mul(py).widen())
	b.w = b.w.Add(invI.Mul(angular).widen())
}

func vec2cFrom(v Vec2) vec2c { return vec2c{X: qcFrom(v.X), Y: qcFrom(v.Y)} }

func rotcFrom(r Rot) rotc { return rotc{Sin: qcFrom(r.Sin), Cos: qcFrom(r.Cos)} }

func crossc(a, b vec2c) qc { return a.X.Mul(b.Y).Sub(a.Y.Mul(b.X)) }

// The reference compares lanes with b2MaxW and b2MinW, which keep the first
// operand on a tie, and scalars with b2MaxFloat and b2MinFloat, which keep
// the second. Max and Min keep the argument on a tie, so maxW and minW swap
// the operands. Only the sign of a zero can tell the two apart.
func maxW(a, b qc) qc { return b.Max(a) }

func minW(a, b qc) qc { return b.Min(a) }

// laneGroupAny reports whether a constraint of the lane group of index has
// a nonzero value of field. A colored stage that skips work for a whole
// lane group tests the group, not the contact (laneGroupWidth).
func laneGroupAny(constraints []contactConstraint, index int, field func(*contactConstraint) qc) bool {
	start := index &^ (laneGroupWidth - 1)
	end := min(start+laneGroupWidth, len(constraints))
	zero := qc{}
	for i := start; i < end; i++ {
		if !field(&constraints[i]).Eq(zero) {
			return true
		}
	}
	return false
}

func constraintRestitution(c *contactConstraint) qc { return c.restitution }

func constraintRollingResistance(c *contactConstraint) qc { return c.rollingResistance }

// The scalar family serves every color. A colored stage runs one lane of
// the wide stages of src/contact_solver_wide.inl and an overflow stage runs
// the stage of src/contact_solver.c; each rounds the same values in its own
// order. A colored constraint always solves two points: the second point of
// a one point manifold is zero, as the manifold point the lane reads. The
// overflow stages also run the Q32 tail of their color; the active colors
// run theirs as the last unit of the color (runGraphContactBlock).

// prepareContactsTask builds constraints for a range of the flat contact
// array. It corresponds to b2PrepareContacts_Wide in
// src/contact_solver_wide.inl.
func prepareContactsTask(startIndex, endIndex int, context *stepContext) {
	prepareContactRange(startIndex, endIndex, context, context.contacts, context.contactConstraints, true)
}

// prepareOverflowContacts builds the overflow contact constraints. It
// corresponds to b2PrepareContacts_Overflow in src/contact_solver.c.
func prepareOverflowContacts(context *stepContext) {
	color := &context.graph.colors[overflowIndex]
	prepareContactRange(0, len(color.contacts), context, color.contacts, color.contactConstraints, false)
}

func prepareContactRange(startIndex, endIndex int, context *stepContext, contacts []*contactSim, constraints []contactConstraint, colored bool) {
	w := context.world
	awakeStates := context.states
	constraints = constraints[startIndex:endIndex]
	contacts = contacts[startIndex:endIndex]

	// Stiffer for static contacts to avoid bodies getting pushed through the ground
	contactSoftness := contactSoftFrom(context.contactSoftness)
	staticSoftness := contactSoftFrom(context.staticSoftness)

	zero := QZero()
	one := QOne()
	warmStartScale := zero
	if w.enableWarmStarting {
		warmStartScale = one
	}
	anyRestitution := false

	for i := range endIndex - startIndex {
		cs := contacts[i]
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

		// Only sample the normal velocity when restitution or a hit event
		// needs it.
		haveRestitution := zero.Less(cs.restitution)
		sampleVelocity := haveRestitution || cs.simFlags&simEnableHitEvent != 0
		anyRestitution = anyRestitution || haveRestitution

		vA := Vec2Zero()
		wA := zero
		mA := cs.invMassA
		iA := cs.invIA
		if sampleVelocity && indexA != nullIndex {
			stateA := &awakeStates[indexA]
			vA = stateA.linearVelocity
			wA = stateA.angularVelocity
		}

		vB := Vec2Zero()
		wB := zero
		mB := cs.invMassB
		iB := cs.invIB
		if sampleVelocity && indexB != nullIndex {
			stateB := &awakeStates[indexB]
			vB = stateB.linearVelocity
			wB = stateB.angularVelocity
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

		constraint.points[1] = contactConstraintPoint{}
		for j := range pointCount {
			mp := &manifold.Points[j]
			cp := &constraint.points[j]

			cp.normalImpulse = qcFrom(warmStartScale.Mul(mp.NormalImpulse))
			cp.tangentImpulse = qcFrom(warmStartScale.Mul(mp.TangentImpulse))
			cp.totalNormalImpulse = qa{}
			cp.restitutionImpulse = qc{}

			rA := mp.AnchorA
			rB := mp.AnchorB

			cp.anchorA = vec2cFrom(rA)
			cp.anchorB = vec2cFrom(rB)
			cp.baseSeparation = qcFrom(mp.Separation.Sub(rB.Sub(rA).Dot(normal)))

			rnA := Cross(rA, normal)
			rnB := Cross(rB, normal)
			rtA := Cross(rA, tangent)
			rtB := Cross(rB, tangent)
			var kNormal, kTangent Q
			if colored {
				kNormal = mA.Add(mB).Add(iA.Mul(rnA.Mul(rnA))).Add(iB.Mul(rnB.Mul(rnB)))
				kTangent = mA.Add(mB).Add(iA.Mul(rtA.Mul(rtA))).Add(iB.Mul(rtB.Mul(rtB)))
			} else {
				kNormal = mA.Add(mB).Add(iA.Mul(rnA).Mul(rnA)).Add(iB.Mul(rnB).Mul(rnB))
				kTangent = mA.Add(mB).Add(iA.Mul(rtA).Mul(rtA)).Add(iB.Mul(rtB).Mul(rtB))
			}
			cp.normalMass = qc{}
			if zero.Less(kNormal) {
				cp.normalMass = qcFrom(one.Div(kNormal))
			}
			cp.tangentMass = qc{}
			if zero.Less(kTangent) {
				cp.tangentMass = qcFrom(one.Div(kTangent))
			}

			// Relative velocity at contact
			cp.relativeVelocity = qc{}
			mp.NormalVelocity = zero
			if sampleVelocity {
				dvx := vB.X.Sub(wB.Mul(rB.Y)).Sub(vA.X.Sub(wA.Mul(rA.Y)))
				dvy := vB.Y.Add(wB.Mul(rB.X)).Sub(vA.Y.Add(wA.Mul(rA.X)))
				vn := dvx.Mul(normal.X).Add(dvy.Mul(normal.Y))
				cp.relativeVelocity = qcFrom(vn)
				mp.NormalVelocity = vn
			}
		}
	}

	if anyRestitution {
		context.anyRestitution.Store(true)
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

// solvedPointCount returns the points a stage solves: two in a colored
// stage, the manifold points in an overflow stage.
func solvedPointCount(constraint *contactConstraint, colored bool) int {
	if colored {
		return 2
	}
	return constraint.pointCount
}

// warmStartContactsTask applies stored impulses to a contact range. It
// corresponds to b2WarmStartContacts_Wide in src/contact_solver_wide.inl.
func warmStartContactsTask(startIndex, endIndex int, context *stepContext, colorIndex int) {
	constraints := context.graph.colors[colorIndex].contactConstraints
	warmStartContactRange(startIndex, endIndex, context, constraints, true)
}

// warmStartOverflowContacts applies stored impulses to the overflow color.
// It corresponds to b2WarmStartContacts_Overflow in src/contact_solver.c.
func warmStartOverflowContacts(context *stepContext) {
	overflow := &context.graph.colors[overflowIndex]
	warmStartContactRange(0, len(overflow.contactConstraints), context, overflow.contactConstraints, false)
	warmStartContacts32(0, len(overflow.contacts32), context, overflowIndex)
}

func warmStartContactRange(startIndex, endIndex int, context *stepContext, constraints []contactConstraint, colored bool) {
	states := context.states
	constraints = constraints[startIndex:endIndex]

	// This is a dummy state to represent a static body because static bodies don't have a solver body.
	dummyState := identityBodyState()

	for i := range endIndex - startIndex {
		constraint := &constraints[i]

		stateA, stateB := constraintStates(states, &dummyState, constraint)
		bodyA := loadContactBody(stateA)
		bodyB := loadContactBody(stateB)

		mA, iA := constraint.invMassA, constraint.invIA
		mB, iB := constraint.invMassB, constraint.invIB

		normal := constraint.normal
		tangent := vec2c{X: normal.Y, Y: normal.X.Neg()}
		pointCount := solvedPointCount(constraint, colored)

		for j := range pointCount {
			cp := &constraint.points[j]

			// fixed anchors
			rA := cp.anchorA
			rB := cp.anchorB

			P := vec2c{
				X: cp.normalImpulse.Mul(normal.X).Add(cp.tangentImpulse.Mul(tangent.X)),
				Y: cp.normalImpulse.Mul(normal.Y).Add(cp.tangentImpulse.Mul(tangent.Y)),
			}
			bodyA.subImpulse(mA, iA, P.X, P.Y, crossc(rA, P))
			bodyB.addImpulse(mB, iB, P.X, P.Y, crossc(rB, P))

			cp.totalNormalImpulse = cp.totalNormalImpulse.Add(cp.normalImpulse.widen())
		}

		bodyA.w = bodyA.w.Sub(iA.Mul(constraint.rollingImpulse).widen())
		bodyB.w = bodyB.w.Add(iB.Mul(constraint.rollingImpulse).widen())

		bodyA.store(stateA)
		bodyB.store(stateB)
	}
}

// pushContactsTask solves the non-penetration of a contact range with the
// soft bias. It corresponds to b2PushContacts_Wide in
// src/contact_solver_wide.inl.
func pushContactsTask(startIndex, endIndex int, context *stepContext, colorIndex int) {
	constraints := context.graph.colors[colorIndex].contactConstraints
	pushContactRange(startIndex, endIndex, context, constraints, true)
}

// pushOverflowContacts solves the non-penetration of the overflow contacts.
// It corresponds to b2PushContacts_Overflow in src/contact_solver.c.
func pushOverflowContacts(context *stepContext) {
	overflow := &context.graph.colors[overflowIndex]
	pushContactRange(0, len(overflow.contactConstraints), context, overflow.contactConstraints, false)
	pushContacts32(0, len(overflow.contacts32), context, overflowIndex)
}

// pushContactRange solves the non-penetration of one range, with no
// friction and no restitution. Point 2 sees the impulse of point 1 through
// the coupling of the two points.
func pushContactRange(startIndex, endIndex int, context *stepContext, constraints []contactConstraint, colored bool) {
	states := context.states
	constraints = constraints[startIndex:endIndex]

	invH := qcFrom(context.invH)
	contactSpeed := qcFrom(context.world.contactSpeed)

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

		mA, iA := constraint.invMassA, constraint.invIA
		mB, iB := constraint.invMassB, constraint.invIB
		normal := constraint.normal
		soft := constraint.softness

		wA := bodyA.w.narrow()
		wB := bodyB.w.narrow()
		dn := bodyB.vx.Sub(bodyA.vx).narrow().Mul(normal.X).Add(bodyB.vy.Sub(bodyA.vy).narrow().Mul(normal.Y))

		var rnA, rnB, impulses [2]qc
		pointCount := solvedPointCount(constraint, colored)
		for j := range pointCount {
			cp := &constraint.points[j]

			// compute current separation
			// this is subject to round-off error if the anchor is far from the body center of mass
			ds := dp.Add(dqB.Apply(cp.anchorB).Sub(dqA.Apply(cp.anchorA)))
			s := normal.Dot(ds).Add(cp.baseSeparation)

			// Apply speculative bias if separation is greater than zero,
			// otherwise apply soft constraint bias. The contact speed limits
			// stiffness, it does not increase it.
			velocityBias := s.Mul(invH)
			massScale := one
			impulseScale := zero
			if !zero.Less(s) {
				velocityBias = maxW(soft.pushRate.Mul(s), contactSpeed.Neg())
				massScale = soft.massScale
				impulseScale = soft.impulseScale
			}

			rnA[j] = crossc(cp.anchorA, normal)
			rnB[j] = crossc(cp.anchorB, normal)

			// Relative normal velocity at contact.
			var vn qc
			if colored {
				vn = dn.Add(wB.Mul(rnB[j]).Sub(wA.Mul(rnA[j])))
			} else {
				vn = dn.Add(wB.Mul(rnB[j])).Sub(wA.Mul(rnA[j]))
			}
			if j == 1 {
				// Point 2 only depends on the impulse from point 1.
				var k qc
				if colored {
					k = mA.Add(mB).Add(iA.Mul(rnA[0]).Mul(rnA[1]).Add(iB.Mul(rnB[0]).Mul(rnB[1])))
				} else {
					k = mA.Add(mB).Add(iA.Mul(rnA[0]).Mul(rnA[1])).Add(iB.Mul(rnB[0]).Mul(rnB[1]))
				}
				vn = vn.Add(k.Mul(impulses[0]))
			}

			negImpulse := cp.normalMass.Mul(massScale.Mul(vn).Add(velocityBias)).Add(impulseScale.Mul(cp.normalImpulse))
			impulse := maxW(negImpulse.Neg(), cp.normalImpulse.Neg())
			cp.normalImpulse = cp.normalImpulse.Add(impulse)
			cp.totalNormalImpulse = cp.totalNormalImpulse.Add(impulse.widen())
			impulses[j] = impulse
		}

		impulse := impulses[0].Add(impulses[1])
		px := impulse.Mul(normal.X)
		py := impulse.Mul(normal.Y)
		angularA := impulses[0].Mul(rnA[0]).Add(impulses[1].Mul(rnA[1]))
		angularB := impulses[0].Mul(rnB[0]).Add(impulses[1].Mul(rnB[1]))

		bodyA.subImpulse(mA, iA, px, py, angularA)
		bodyB.addImpulse(mB, iB, px, py, angularB)

		bodyA.store(stateA)
		bodyB.store(stateB)
	}
}

// solveContactsTask relaxes a contact range: the normal impulse with the
// speculative bias only, friction and rolling resistance. It corresponds to
// b2SolveContacts_Wide in src/contact_solver_wide.inl.
func solveContactsTask(startIndex, endIndex int, context *stepContext, colorIndex int) {
	constraints := context.graph.colors[colorIndex].contactConstraints
	solveContactRange(startIndex, endIndex, context, constraints)
}

// solveOverflowContacts relaxes the overflow contacts. It corresponds to
// b2SolveContacts_Overflow in src/contact_solver.c.
func solveOverflowContacts(context *stepContext) {
	overflow := &context.graph.colors[overflowIndex]
	solveContactRangeSerial(0, len(overflow.contactConstraints), context, overflow.contactConstraints)
	solveContacts32(0, len(overflow.contacts32), context, overflowIndex)
}

// solveContactRange relaxes the colored contacts of one range of a color.
// constraints is the whole color: the rolling resistance runs for a lane
// group with any rolling resistance.
func solveContactRange(startIndex, endIndex int, context *stepContext, constraints []contactConstraint) {
	states := context.states
	invH := qcFrom(context.invH)

	// This is a dummy body to represent a static body since static bodies don't have a solver body.
	dummyState := identityBodyState()

	zero := qc{}
	for i := startIndex; i < endIndex; i++ {
		constraint := &constraints[i]

		stateA, stateB := constraintStates(states, &dummyState, constraint)
		bodyA := loadContactBody(stateA)
		dqA := rotcFrom(stateA.deltaRotation)

		bodyB := loadContactBody(stateB)
		dqB := rotcFrom(stateB.deltaRotation)

		dp := vec2cFrom(stateB.deltaPosition).Sub(vec2cFrom(stateA.deltaPosition))

		mA, iA := constraint.invMassA, constraint.invIA
		mB, iB := constraint.invMassB, constraint.invIB
		normal := constraint.normal
		tangent := vec2c{X: normal.Y, Y: normal.X.Neg()}
		c1 := &constraint.points[0]
		c2 := &constraint.points[1]

		// Speculative bias, positive if separated and zero if overlapped
		s1 := normal.Dot(dp.Add(dqB.Apply(c1.anchorB).Sub(dqA.Apply(c1.anchorA)))).Add(c1.baseSeparation)
		velocityBias1 := maxW(zero, s1.Mul(invH))
		s2 := normal.Dot(dp.Add(dqB.Apply(c2.anchorB).Sub(dqA.Apply(c2.anchorA)))).Add(c2.baseSeparation)
		velocityBias2 := maxW(zero, s2.Mul(invH))

		rnA1 := crossc(c1.anchorA, normal)
		rnB1 := crossc(c1.anchorB, normal)
		rnA2 := crossc(c2.anchorA, normal)
		rnB2 := crossc(c2.anchorB, normal)
		rtA1 := crossc(c1.anchorA, tangent)
		rtB1 := crossc(c1.anchorB, tangent)
		rtA2 := crossc(c2.anchorA, tangent)
		rtB2 := crossc(c2.anchorB, tangent)

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

		wA := bodyA.w.narrow()
		wB := bodyB.w.narrow()
		dvx := bodyB.vx.Sub(bodyA.vx).narrow()
		dvy := bodyB.vy.Sub(bodyA.vy).narrow()
		dn := dvx.Mul(normal.X).Add(dvy.Mul(normal.Y))
		dt := dvx.Mul(tangent.X).Add(dvy.Mul(tangent.Y)).Sub(constraint.tangentSpeed)

		vn1 := dn.Add(wB.Mul(rnB1).Sub(wA.Mul(rnA1)))
		vn2 := dn.Add(wB.Mul(rnB2).Sub(wA.Mul(rnA2)))
		vt1 := dt.Add(wB.Mul(rtB1).Sub(wA.Mul(rtA1)))
		vt2 := dt.Add(wB.Mul(rtB2).Sub(wA.Mul(rtA2)))

		normalImpulse1 := maxW(c1.normalMass.Mul(vn1.Add(velocityBias1)).Neg(), c1.normalImpulse.Neg())
		c1.normalImpulse = c1.normalImpulse.Add(normalImpulse1)
		c1.totalNormalImpulse = c1.totalNormalImpulse.Add(normalImpulse1.widen())

		vn2 = vn2.Add(k21.Mul(normalImpulse1))

		normalImpulse2 := maxW(c2.normalMass.Mul(vn2.Add(velocityBias2)).Neg(), c2.normalImpulse.Neg())
		c2.normalImpulse = c2.normalImpulse.Add(normalImpulse2)
		c2.totalNormalImpulse = c2.totalNormalImpulse.Add(normalImpulse2.widen())

		vt1 = vt1.Add(k31.Mul(normalImpulse1)).Add(k32.Mul(normalImpulse2))
		vt2 = vt2.Add(k41.Mul(normalImpulse1)).Add(k42.Mul(normalImpulse2))

		angularA := normalImpulse1.Mul(rnA1).Add(normalImpulse2.Mul(rnA2))
		angularB := normalImpulse1.Mul(rnB1).Add(normalImpulse2.Mul(rnB2))

		// Rolling resistance
		if laneGroupAny(constraints, i, constraintRollingResistance) {
			relativeW := wA.Sub(wB).Sub(wnA1.Add(wnB1).Mul(normalImpulse1).Add(wnA2.Add(wnB2).Mul(normalImpulse2)))
			k := iA.Add(iB)
			deltaLambda := zero
			if zero.Less(k) {
				deltaLambda = relativeW.Div(k)
			}
			lambda := constraint.rollingImpulse
			maxLambda := constraint.rollingResistance.Mul(c1.normalImpulse.Add(c2.normalImpulse))
			constraint.rollingImpulse = lambda.Add(deltaLambda).Clamp(maxLambda.Neg(), maxLambda)
			deltaLambda = constraint.rollingImpulse.Sub(lambda)

			wtA2 := iA.Mul(rtA2)
			wtB2 := iB.Mul(rtB2)
			vt1 = vt1.Add(wtA1.Add(wtB1).Mul(deltaLambda))
			vt2 = vt2.Add(wtA2.Add(wtB2).Mul(deltaLambda))

			angularA = angularA.Add(deltaLambda)
			angularB = angularB.Add(deltaLambda)
		}

		maxFriction1 := constraint.friction.Mul(c1.normalImpulse)
		tangentImpulse1 := c1.tangentMass.Mul(vt1).Neg()
		tangentImpulse1 = maxW(maxFriction1.Neg().Sub(c1.tangentImpulse), minW(tangentImpulse1, maxFriction1.Sub(c1.tangentImpulse)))
		c1.tangentImpulse = c1.tangentImpulse.Add(tangentImpulse1)

		vt2 = vt2.Add(k43.Mul(tangentImpulse1))

		maxFriction2 := constraint.friction.Mul(c2.normalImpulse)
		tangentImpulse2 := c2.tangentMass.Mul(vt2).Neg()
		tangentImpulse2 = maxW(maxFriction2.Neg().Sub(c2.tangentImpulse), minW(tangentImpulse2, maxFriction2.Sub(c2.tangentImpulse)))
		c2.tangentImpulse = c2.tangentImpulse.Add(tangentImpulse2)

		angularA = angularA.Add(tangentImpulse1.Mul(rtA1).Add(tangentImpulse2.Mul(rtA2)))
		angularB = angularB.Add(tangentImpulse1.Mul(rtB1).Add(tangentImpulse2.Mul(rtB2)))

		normalImpulse := normalImpulse1.Add(normalImpulse2)
		tangentImpulse := tangentImpulse1.Add(tangentImpulse2)
		px := normalImpulse.Mul(normal.X).Add(tangentImpulse.Mul(tangent.X))
		py := normalImpulse.Mul(normal.Y).Add(tangentImpulse.Mul(tangent.Y))

		bodyA.subImpulse(mA, iA, px, py, angularA)
		bodyB.addImpulse(mB, iB, px, py, angularB)

		bodyA.store(stateA)
		bodyB.store(stateB)
	}
}

// solveContactRangeSerial relaxes the overflow contacts of one range, in
// the order of b2SolveContacts_Overflow.
func solveContactRangeSerial(startIndex, endIndex int, context *stepContext, constraints []contactConstraint) {
	states := context.states
	constraints = constraints[startIndex:endIndex]

	invH := qcFrom(context.invH)

	// This is a dummy body to represent a static body since static bodies don't have a solver body.
	dummyState := identityBodyState()

	zero := qc{}
	for i := range endIndex - startIndex {
		constraint := &constraints[i]

		stateA, stateB := constraintStates(states, &dummyState, constraint)
		bodyA := loadContactBody(stateA)
		dqA := rotcFrom(stateA.deltaRotation)

		bodyB := loadContactBody(stateB)
		dqB := rotcFrom(stateB.deltaRotation)

		dp := vec2cFrom(stateB.deltaPosition).Sub(vec2cFrom(stateA.deltaPosition))

		mA, iA := constraint.invMassA, constraint.invIA
		mB, iB := constraint.invMassB, constraint.invIB
		normal := constraint.normal
		tangent := vec2c{X: normal.Y, Y: normal.X.Neg()}
		friction := constraint.friction
		pointCount := constraint.pointCount

		wA := bodyA.w.narrow()
		wB := bodyB.w.narrow()
		dvx := bodyB.vx.Sub(bodyA.vx).narrow()
		dvy := bodyB.vy.Sub(bodyA.vy).narrow()
		vnc := dvx.Mul(normal.X).Add(dvy.Mul(normal.Y))

		// vt = dot(vrB - sB * tangent - (vrA + sA * tangent), tangent)
		//    = dot(vrB - vrA, tangent) - (sA + sB)
		vtc := dvx.Mul(tangent.X).Add(dvy.Mul(tangent.Y)).Sub(constraint.tangentSpeed)

		var normalImpulses, tangentImpulses [2]qc

		// Cache cross products
		var rnA, rnB, rtA, rtB [2]qc
		for j := range pointCount {
			cp := &constraint.points[j]
			rnA[j] = crossc(cp.anchorA, normal)
			rnB[j] = crossc(cp.anchorB, normal)
			rtA[j] = crossc(cp.anchorA, tangent)
			rtB[j] = crossc(cp.anchorB, tangent)
		}

		totalNormalImpulse := qa{}
		for j := range pointCount {
			cp := &constraint.points[j]

			// Compute current separation.
			// This is subject to round-off error if the anchor is far from the body center of mass.
			ds := dp.Add(dqB.Apply(cp.anchorB).Sub(dqA.Apply(cp.anchorA)))
			s := cp.baseSeparation.Add(ds.Dot(normal))

			// Speculative bias, zero when overlapped.
			velocityBias := zero
			if zero.Less(s) {
				velocityBias = s.Mul(invH)
			}

			// Relative normal velocity at contact.
			vn := vnc.Add(wB.Mul(rnB[j])).Sub(wA.Mul(rnA[j]))
			for k := range j {
				// This allows getting point 2 normal velocity with only one dependency.
				vn = vn.Add(mA.Add(mB).Add(iA.Mul(rnA[k]).Mul(rnA[j])).Add(iB.Mul(rnB[k]).Mul(rnB[j])).Mul(normalImpulses[k]))
			}

			impulse := cp.normalMass.Neg().Mul(vn.Add(velocityBias)).Max(cp.normalImpulse.Neg())
			cp.normalImpulse = cp.normalImpulse.Add(impulse)
			cp.totalNormalImpulse = cp.totalNormalImpulse.Add(impulse.widen())
			normalImpulses[j] = impulse

			// This is used for rolling resistance
			totalNormalImpulse = totalNormalImpulse.Add(cp.normalImpulse.widen())
		}

		// Rolling resistance
		var rollingImpulse qc
		{
			cdot := wB.Sub(wA)
			for j := range pointCount {
				cdot = cdot.Add(iA.Mul(rnA[j]).Add(iB.Mul(rnB[j])).Mul(normalImpulses[j]))
			}

			lambda := constraint.rollingImpulse
			maxLambda := rollingBound(constraint.rollingResistance, totalNormalImpulse)
			constraint.rollingImpulse = lambda.Sub(constraint.rollingMass.Mul(cdot)).Clamp(maxLambda.Neg(), maxLambda)
			rollingImpulse = constraint.rollingImpulse.Sub(lambda)
		}

		// Friction
		for j := range pointCount {
			cp := &constraint.points[j]

			// relative tangent velocity at contact
			vt := vtc.Add(wB.Mul(rtB[j])).Sub(wA.Mul(rtA[j])).Add(iA.Mul(rtA[j]).Add(iB.Mul(rtB[j])).Mul(rollingImpulse))
			for k := range pointCount {
				vt = vt.Add(iA.Mul(rtA[j]).Mul(rnA[k]).Add(iB.Mul(rtB[j]).Mul(rnB[k])).Mul(normalImpulses[k]))
			}
			for k := range j {
				vt = vt.Add(mA.Add(mB).Add(iA.Mul(rtA[k]).Mul(rtA[j])).Add(iB.Mul(rtB[k]).Mul(rtB[j])).Mul(tangentImpulses[k]))
			}

			// clamp the accumulated force
			maxFriction := friction.Mul(cp.normalImpulse)
			impulse := cp.tangentMass.Neg().Mul(vt).Clamp(maxFriction.Neg().Sub(cp.tangentImpulse), maxFriction.Sub(cp.tangentImpulse))
			cp.tangentImpulse = cp.tangentImpulse.Add(impulse)
			tangentImpulses[j] = impulse
		}

		normalImpulse := zero
		tangentImpulse := zero
		angularA := rollingImpulse
		angularB := rollingImpulse
		for j := range pointCount {
			normalImpulse = normalImpulse.Add(normalImpulses[j])
			tangentImpulse = tangentImpulse.Add(tangentImpulses[j])
			angularA = angularA.Add(normalImpulses[j].Mul(rnA[j]).Add(tangentImpulses[j].Mul(rtA[j])))
			angularB = angularB.Add(normalImpulses[j].Mul(rnB[j]).Add(tangentImpulses[j].Mul(rtB[j])))
		}

		px := normalImpulse.Mul(normal.X).Add(tangentImpulse.Mul(tangent.X))
		py := normalImpulse.Mul(normal.Y).Add(tangentImpulse.Mul(tangent.Y))
		bodyA.subImpulse(mA, iA, px, py, angularA)
		bodyB.addImpulse(mB, iB, px, py, angularB)

		bodyA.store(stateA)
		bodyB.store(stateB)
	}
}

// applyRestitutionTask applies restitution to a contact range. It
// corresponds to b2ApplyRestitution_Wide in src/contact_solver_wide.inl.
func applyRestitutionTask(startIndex, endIndex int, context *stepContext, colorIndex int) {
	constraints := context.graph.colors[colorIndex].contactConstraints
	applyRestitutionRange(startIndex, endIndex, context, constraints, true)
}

// applyOverflowRestitution applies restitution to the overflow contacts.
// It corresponds to b2ApplyRestitution_Overflow in src/contact_solver.c.
func applyOverflowRestitution(context *stepContext) {
	overflow := &context.graph.colors[overflowIndex]
	applyRestitutionRange(0, len(overflow.contactConstraints), context, overflow.contactConstraints, false)
	applyRestitution32(0, len(overflow.contacts32), context, overflowIndex)
}

// applyRestitutionRange applies Poisson restitution to one range: a point
// that approached faster than the threshold and was compressed gets back at
// most the restitution times its compression impulse, so no energy is
// gained. constraints is the whole color, because a colored stage skips a
// lane group only when no lane has restitution.
func applyRestitutionRange(startIndex, endIndex int, context *stepContext, constraints []contactConstraint, colored bool) {
	w := context.world
	states := context.states

	negThreshold := qcFrom(w.restitutionThreshold).Neg()
	invH := qcFrom(context.invH)
	propagate := w.enableRestitutionPropagation

	// dummy state to represent a static body
	dummyState := identityBodyState()

	zero := qc{}
	for i := startIndex; i < endIndex; i++ {
		constraint := &constraints[i]
		restitution := constraint.restitution
		if !propagate {
			if colored {
				if !laneGroupAny(constraints, i, constraintRestitution) {
					continue
				}
			} else if restitution.Eq(zero) {
				continue
			}
		}

		stateA, stateB := constraintStates(states, &dummyState, constraint)
		bodyA := loadContactBody(stateA)
		dqA := rotcFrom(stateA.deltaRotation)

		bodyB := loadContactBody(stateB)
		dqB := rotcFrom(stateB.deltaRotation)

		dp := vec2cFrom(stateB.deltaPosition).Sub(vec2cFrom(stateA.deltaPosition))

		mA, iA := constraint.invMassA, constraint.invIA
		mB, iB := constraint.invMassB, constraint.invIB
		normal := constraint.normal
		haveRestitution := zero.Less(restitution)
		pointCount := solvedPointCount(constraint, colored)

		for j := range pointCount {
			cp := &constraint.points[j]

			rA := cp.anchorA
			rB := cp.anchorB

			normalMass := cp.normalMass
			if !propagate && !haveRestitution {
				normalMass = zero
			}

			// The total normal impulse is 0 for speculative points.
			compressionImpulse := cp.totalNormalImpulse.Sub(cp.restitutionImpulse.widen()).narrow()
			armed := haveRestitution && cp.relativeVelocity.Less(negThreshold) && zero.Less(compressionImpulse)

			var velocityBias qc
			if armed {
				velocityBias = restitution.Mul(cp.relativeVelocity)
			} else {
				ds := dp.Add(dqB.Apply(rB).Sub(dqA.Apply(rA)))
				s := normal.Dot(ds).Add(cp.baseSeparation)
				velocityBias = maxW(zero, s.Mul(invH))
			}

			wA := bodyA.w.narrow()
			wB := bodyB.w.narrow()
			dvx := bodyB.vx.narrow().Sub(wB.Mul(rB.Y)).Sub(bodyA.vx.narrow().Sub(wA.Mul(rA.Y)))
			dvy := bodyB.vy.narrow().Add(wB.Mul(rB.X)).Sub(bodyA.vy.narrow().Add(wA.Mul(rA.X)))
			vn := dvx.Mul(normal.X).Add(dvy.Mul(normal.Y))

			negImpulse := normalMass.Mul(vn.Add(velocityBias))
			newImpulse := maxW(cp.normalImpulse.Sub(negImpulse), zero)
			impulse := newImpulse.Sub(cp.normalImpulse)

			approachImpulse := minW(maxW(normalMass.Mul(vn).Neg(), zero), maxW(impulse, zero))
			if armed {
				// Poisson kinetic restitution guarantees no energy gain.
				allowance := restitution.Mul(compressionImpulse.Add(approachImpulse)).Sub(cp.restitutionImpulse)
				impulse = minW(impulse, approachImpulse.Add(maxW(allowance, zero)))
			}

			cp.normalImpulse = cp.normalImpulse.Add(impulse)
			cp.restitutionImpulse = cp.restitutionImpulse.Add(impulse.Sub(approachImpulse))
			cp.totalNormalImpulse = cp.totalNormalImpulse.Add(impulse.widen())

			px := impulse.Mul(normal.X)
			py := impulse.Mul(normal.Y)
			bodyA.subImpulse(mA, iA, px, py, rA.X.Mul(py).Sub(rA.Y.Mul(px)))
			bodyB.addImpulse(mB, iB, px, py, rB.X.Mul(py).Sub(rB.Y.Mul(px)))
		}

		bodyA.store(stateA)
		bodyB.store(stateB)
	}
}

// storeImpulsesTask stores a range of flat contact impulses. It corresponds
// to b2StoreImpulses_Wide in src/contact_solver_wide.inl.
func storeImpulsesTask(startIndex, endIndex int, context *stepContext, workerIndex int) {
	storeImpulseRange(startIndex, endIndex, context, context.contacts, context.contactConstraints, workerIndex)
}

// storeOverflowImpulses stores the overflow contact impulses. It
// corresponds to b2StoreImpulses_Overflow in src/contact_solver.c.
func storeOverflowImpulses(context *stepContext) {
	color := &context.graph.colors[overflowIndex]
	storeImpulseRange(0, len(color.contacts), context, color.contacts, color.contactConstraints, 0)
}

// storeImpulseRange writes the impulses back to the manifolds and flags the
// contacts with a hit event candidate, so the serial report reads only
// those.
func storeImpulseRange(startIndex, endIndex int, context *stepContext, contacts []*contactSim, constraints []contactConstraint, workerIndex int) {
	w := context.world
	taskContext := &w.taskContexts[workerIndex]
	negHitThreshold := w.hitEventThreshold.Neg()
	zero := QZero()
	constraints = constraints[startIndex:endIndex]
	contacts = contacts[startIndex:endIndex]

	for i := range endIndex - startIndex {
		constraint := &constraints[i]
		cs := contacts[i]
		manifold := &cs.manifold
		pointCount := manifold.PointCount

		for j := range pointCount {
			mp := &manifold.Points[j]
			mp.NormalImpulse = constraint.points[j].normalImpulse.toQ()
			mp.TangentImpulse = constraint.points[j].tangentImpulse.toQ()
			mp.TotalNormalImpulse = constraint.points[j].totalNormalImpulse.toQ()
		}

		manifold.RollingImpulse = constraint.rollingImpulse.toQ()

		if cs.simFlags&simEnableHitEvent != 0 {
			for j := range pointCount {
				mp := &manifold.Points[j]

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
