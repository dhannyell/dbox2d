package dbox2d

import "testing"

// withinQ reports whether a and b differ by at most limit.
func withinQ(a, b, limit Q) bool {
	return !limit.Less(a.Sub(b).Abs())
}

// restingBox builds a unit box of mass one on a static ground and moves
// their contact into the overflow color, so the tests also cover the
// color that keeps no body set, with one hand-made manifold point under
// the center of the box. It returns the box body and the
// step context of one 4 sub-step frame at 60 Hz.
func restingBox(t *testing.T) (*world, *body, *stepContext) {
	t.Helper()
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)

	groundDef := DefaultBodyDef()
	groundDef.Position = Vec2{Y: QHalf().Neg()}
	groundId := CreateBody(worldId, &groundDef)
	shapeDef := DefaultShapeDef()
	ground := MakeBox(QFromInt(5), QHalf())
	CreatePolygonShape(groundId, &shapeDef, &ground)

	boxDef := DefaultBodyDef()
	boxDef.Type = DynamicBody
	boxDef.Position = Vec2{Y: QHalf()}
	boxId := CreateBody(worldId, &boxDef)
	unit := MakeBox(QHalf(), QHalf())
	CreatePolygonShape(boxId, &shapeDef, &unit)

	c := startTouching(t, w, groundId, boxId)

	// Move the sim from its color to the overflow color.
	color := &w.constraintGraph.colors[c.colorIndex]
	cs := color.contactSims[c.localIndex]
	removeContactFromGraph(w, c.edges[0].bodyId, c.edges[1].bodyId, c.colorIndex, c.localIndex)
	overflow := &w.constraintGraph.colors[overflowIndex]
	c.colorIndex = overflowIndex
	c.localIndex = len(overflow.contactSims)
	overflow.contactSims = append(overflow.contactSims, cs)
	overflow.contactConstraints = make([]contactConstraint, 1)

	// One point at the bottom center of the box, on the top of the ground.
	// The anchors are relative to the centers of mass.
	m := &overflow.contactSims[0].manifold
	m.Normal = Vec2{Y: QOne()}
	m.PointCount = 1
	m.Points[0] = ManifoldPoint{
		AnchorA: Vec2{Y: QHalf()},
		AnchorB: Vec2{Y: QHalf().Neg()},
	}

	context := &stepContext{world: w}
	context.dt = QFromRatio(1, 60)
	context.subStepCount = 4
	context.invDt = QFromInt(60)
	context.h = QFromRatio(1, 240)
	context.invH = QFromInt(240)
	context.maxLinearVelocity = w.maxLinearSpeed
	context.enableWarmStarting = w.enableWarmStarting
	context.graph = &w.constraintGraph
	context.contactSoftness = makeSoft(w.contactHertz, w.contactDampingRatio, context.h)
	context.staticSoftness = makeSoft(w.contactHertz.Add(w.contactHertz), w.contactDampingRatio, context.h)
	awake := &w.solverSets[awakeSet]
	context.sims = awake.bodySims
	context.states = awake.bodyStates
	context.bulletBodies = make([]int, len(awake.bodySims))

	return w, getBodyFullId(w, boxId), context
}

// TestMakeSoftSplitsTheUnit pins makeSoft: a zero frequency is rigid, and
// the mass and impulse scales of a soft constraint add up to one.
func TestMakeSoftSplitsTheUnit(t *testing.T) {
	if makeSoft(QZero(), QOne(), QHalf()) != (softness{}) {
		t.Fatalf("a zero frequency is not rigid")
	}

	h := QFromRatio(1, 240)
	soft := makeSoft(QFromInt(30), QFromInt(10), h)

	// omega = 60 pi; a1 = 20 + omega / 240; biasRate = omega / a1.
	omega := tau.Mul(QFromInt(30))
	a1 := QFromInt(20).Add(omega.Mul(h))
	if !soft.biasRate.Eq(omega.Div(a1)) {
		t.Errorf("biasRate is %v, want %v", soft.biasRate, omega.Div(a1))
	}
	one := QOne()
	tolerance := qUlps(4)
	if !withinQ(soft.massScale.Add(soft.impulseScale), one, tolerance) {
		t.Errorf("massScale %v + impulseScale %v is not one", soft.massScale, soft.impulseScale)
	}
	if !QHalf().Less(soft.massScale) {
		t.Errorf("massScale %v is not the larger share at 30 Hz", soft.massScale)
	}
}

// TestPrepareOverflowContactsBuildsTheMasses pins the prepare stage with
// hand values: mass one and inertia one sixth give a normal mass of one
// under the center and a tangent mass of two fifths.
func TestPrepareOverflowContactsBuildsTheMasses(t *testing.T) {
	w, box, context := restingBox(t)
	state := getBodyState(w, box)
	state.linearVelocity = Vec2{Y: QFromInt(-3)}

	prepareOverflowContacts(context)

	constraint := &w.constraintGraph.colors[overflowIndex].contactConstraints[0]
	if constraint.indexA != nullIndex || constraint.indexB != box.localIndex {
		t.Fatalf("the constraint points at bodies %d and %d", constraint.indexA, constraint.indexB)
	}
	if constraint.softness != contactSoftFrom(context.staticSoftness) {
		t.Errorf("a ground contact did not take the static softness")
	}
	tolerance := qUlps(64).Add(contactRounding())
	if constraint.invMassB.toQ() != QOne() || !withinQ(constraint.invIB.toQ(), QFromInt(6), tolerance) {
		t.Errorf("the box has inverse mass %v and inverse inertia %v, want 1 and 6", constraint.invMassB.toQ(), constraint.invIB.toQ())
	}

	cp := &constraint.points[0]
	if !withinQ(cp.normalMass.toQ(), QOne(), tolerance) {
		t.Errorf("normalMass is %v, want 1", cp.normalMass.toQ())
	}
	if !withinQ(cp.tangentMass.toQ(), QFromRatio(2, 5), tolerance) {
		t.Errorf("tangentMass is %v, want 0.4", cp.tangentMass.toQ())
	}
	// baseSeparation = 0 - dot(rB - rA, n) = -(-0.5 - 0.5) = 1
	if !cp.baseSeparation.toQ().Eq(QOne()) {
		t.Errorf("baseSeparation is %v, want 1", cp.baseSeparation.toQ())
	}
	if !cp.relativeVelocity.toQ().Eq(QFromInt(-3)) {
		t.Errorf("relativeVelocity is %v, want -3", cp.relativeVelocity.toQ())
	}
}

// TestWarmStartReappliesTheStoredImpulse pins the warm start: the stored
// normal impulse of the manifold moves the box, and the world switch
// turns it off.
func TestWarmStartReappliesTheStoredImpulse(t *testing.T) {
	w, box, context := restingBox(t)
	m := &w.constraintGraph.colors[overflowIndex].contactSims[0].manifold
	m.Points[0].NormalImpulse = QFromInt(2)

	prepareOverflowContacts(context)
	warmStartOverflowContacts(context)

	state := getBodyState(w, box)
	if !state.linearVelocity.Y.Eq(QFromInt(2)) || !state.angularVelocity.Eq(QZero()) {
		t.Errorf("the warm start gave velocity %v and spin %v, want (0, 2) and 0", state.linearVelocity, state.angularVelocity)
	}

	w.enableWarmStarting = false
	state.linearVelocity = Vec2Zero()
	prepareOverflowContacts(context)
	warmStartOverflowContacts(context)
	if !state.linearVelocity.Y.Eq(QZero()) {
		t.Errorf("the warm start moved the box with warm starting off")
	}
}

// TestRestingBoxHoldsItsGround runs the sub-step order by hand for one
// second: the box stays at separation zero and each sub-step carries an
// impulse equal to the weight times the sub-step time.
func TestRestingBoxHoldsItsGround(t *testing.T) {
	w, box, context := restingBox(t)
	overflow := &w.constraintGraph.colors[overflowIndex]
	sim := getBodySim(w, box)
	m := &overflow.contactSims[0].manifold

	for range 60 {
		// The collide pass refreshes the separation on each step. The box
		// does not rotate, so the anchors stay put.
		m.Points[0].Separation = sim.center.Y.Sub(QHalf())
		prepareOverflowContacts(context)
		for range context.subStepCount {
			integrateVelocitiesTask(0, 1, context)
			warmStartOverflowContacts(context)
			solveOverflowContacts(context, true)
			integratePositionsTask(0, 1, context)
			solveOverflowContacts(context, false)
		}
		applyOverflowRestitution(context)
		storeOverflowImpulses(context)
		setBitCountAndClear(&w.taskContexts[0].awakeIslandBitSet, len(w.solverSets[awakeSet].islandSims))
		w.bodyMoveEvents = resizeMoveEvents(w.bodyMoveEvents, 1)
		finalizeBodiesTask(0, 1, 0, context)
	}

	tolerance := QMustParse("0.001")
	if !withinQ(sim.center.Y, QHalf(), tolerance) {
		t.Errorf("the box rests at y %v, want 0.5", sim.center.Y)
	}
	state := getBodyState(w, box)
	if !withinQ(state.linearVelocity.Y, QZero(), tolerance) {
		t.Errorf("the box moves at %v", state.linearVelocity.Y)
	}

	// weight * h = 10 / 240
	weightImpulse := QFromRatio(10, 240)
	if !withinQ(m.Points[0].NormalImpulse, weightImpulse, tolerance) {
		t.Errorf("the stored normal impulse is %v, want %v", m.Points[0].NormalImpulse, weightImpulse)
	}
	if m.Points[0].TotalNormalImpulse.Less(weightImpulse) {
		t.Errorf("the total normal impulse %v is below one sub-step", m.Points[0].TotalNormalImpulse)
	}
}

// TestFrictionSaturatesAtTheNormalImpulse pins the friction clamp: a fast
// slide takes the full mu times N and no more.
func TestFrictionSaturatesAtTheNormalImpulse(t *testing.T) {
	w, box, context := restingBox(t)
	state := getBodyState(w, box)
	state.linearVelocity = Vec2{X: QFromInt(100), Y: QFromInt(-1)}

	prepareOverflowContacts(context)
	solveOverflowContacts(context, true)

	cp := &w.constraintGraph.colors[overflowIndex].contactConstraints[0].points[0]
	constraint := &w.constraintGraph.colors[overflowIndex].contactConstraints[0]
	if !QZero().Less(cp.normalImpulse.toQ()) {
		t.Fatalf("the normal impulse is %v, want positive", cp.normalImpulse.toQ())
	}
	want := constraint.friction.Mul(cp.normalImpulse).Neg()
	if !cp.tangentImpulse.Eq(want) {
		t.Errorf("the tangent impulse is %v, want %v", cp.tangentImpulse.toQ(), want.toQ())
	}
	// A slide to the right rolls the box clockwise.
	if !state.angularVelocity.Less(QZero()) {
		t.Errorf("the friction under the center did not roll the box")
	}
}

// TestRollingBoundKeepsATwoPointTotal pins the rolling bound when the total
// of two points passes the contact grid range.
func TestRollingBoundKeepsATwoPointTotal(t *testing.T) {
	rr := qcFrom(QFromRatio(1, 8))
	total := qaFrom(QFromInt(40000))
	if got, want := rollingBound(rr, total), qcFrom(QFromInt(5000)); !got.Eq(want) {
		t.Fatalf("the rolling bound is %v, want %v", got.toQ(), want.toQ())
	}
}

// TestRestitutionNeedsTheThreshold pins the restitution gate: a fall
// faster than the threshold bounces to restitution times the approach
// speed; a slower fall does not bounce.
func TestRestitutionNeedsTheThreshold(t *testing.T) {
	for _, tc := range []struct {
		name   string
		fall   string
		bounce bool
	}{
		{"fast", "-3", true},
		{"slow", "-0.5", false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			w, box, context := restingBox(t)
			w.constraintGraph.colors[overflowIndex].contactSims[0].restitution = QHalf()
			state := getBodyState(w, box)
			state.linearVelocity = Vec2{Y: QMustParse(tc.fall)}

			prepareOverflowContacts(context)
			solveOverflowContacts(context, true)
			before := state.linearVelocity.Y
			applyOverflowRestitution(context)

			if !tc.bounce {
				if !state.linearVelocity.Y.Eq(before) {
					t.Errorf("a slow fall bounced to %v", state.linearVelocity.Y)
				}
				return
			}
			// -restitution * relativeVelocity = 1.5
			want := QMustParse("1.5")
			if !withinQ(state.linearVelocity.Y, want, qUlps(16)) {
				t.Errorf("the bounce is %v, want %v", state.linearVelocity.Y, want)
			}
		})
	}
}

// TestStoreOverflowImpulsesFillsTheManifold pins the store: the manifold
// carries the impulses and the approach speed for the next warm start
// and the hit events.
func TestStoreOverflowImpulsesFillsTheManifold(t *testing.T) {
	w, box, context := restingBox(t)
	state := getBodyState(w, box)
	state.linearVelocity = Vec2{X: QFromInt(1), Y: QFromInt(-2)}

	prepareOverflowContacts(context)
	solveOverflowContacts(context, true)
	storeOverflowImpulses(context)

	cp := &w.constraintGraph.colors[overflowIndex].contactConstraints[0].points[0]
	mp := &w.constraintGraph.colors[overflowIndex].contactSims[0].manifold.Points[0]
	if mp.NormalImpulse != cp.normalImpulse.toQ() || mp.TangentImpulse != cp.tangentImpulse.toQ() || mp.TotalNormalImpulse != cp.totalNormalImpulse.toQ() {
		t.Errorf("the manifold holds %v, %v, %v, want %v, %v, %v", mp.NormalImpulse, mp.TangentImpulse, mp.TotalNormalImpulse, cp.normalImpulse.toQ(), cp.tangentImpulse.toQ(), cp.totalNormalImpulse.toQ())
	}
	if !mp.NormalVelocity.Eq(QFromInt(-2)) {
		t.Errorf("the normal velocity is %v, want -2", mp.NormalVelocity)
	}
}
