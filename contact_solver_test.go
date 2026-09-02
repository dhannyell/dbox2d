package dbox2d

import (
	"testing"

	"github.com/dhannyell/fixed"
)

// withinQ reports whether a and b differ by at most limit.
func withinQ(a, b, limit Q) bool {
	return !limit.Less(a.Sub(b).Abs())
}

// restingBox builds a unit box of mass one on a static ground and moves
// their contact into the overflow color with one hand-made manifold
// point under the center of the box. It returns the box body and the
// step context of one 4 sub-step frame at 60 Hz.
func restingBox(t *testing.T) (*world, *body, *stepContext) {
	t.Helper()
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)

	groundDef := DefaultBodyDef()
	groundDef.Position = Vec2{Y: fixed.Q32Half().Neg()}
	groundId := CreateBody(worldId, &groundDef)
	shapeDef := DefaultShapeDef()
	ground := MakeBox(fixed.Q32FromInt(5), fixed.Q32Half())
	CreatePolygonShape(groundId, &shapeDef, &ground)

	boxDef := DefaultBodyDef()
	boxDef.Type = DynamicBody
	boxDef.Position = Vec2{Y: fixed.Q32Half()}
	boxId := CreateBody(worldId, &boxDef)
	unit := MakeBox(fixed.Q32Half(), fixed.Q32Half())
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
	overflow.overflowConstraints = make([]contactConstraint, 1)

	// One point at the bottom center of the box, on the top of the ground.
	// The anchors are relative to the centers of mass.
	m := &overflow.contactSims[0].manifold
	m.Normal = Vec2{Y: fixed.Q32One()}
	m.PointCount = 1
	m.Points[0] = ManifoldPoint{
		AnchorA: Vec2{Y: fixed.Q32Half()},
		AnchorB: Vec2{Y: fixed.Q32Half().Neg()},
	}

	context := &stepContext{world: w}
	context.dt = fixed.Q32FromRatio(1, 60)
	context.subStepCount = 4
	context.invDt = fixed.Q32FromInt(60)
	context.h = fixed.Q32FromRatio(1, 240)
	context.invH = fixed.Q32FromInt(240)
	context.maxLinearVelocity = w.maxLinearSpeed
	context.enableWarmStarting = w.enableWarmStarting
	context.contactSoftness = makeSoft(w.contactHertz, w.contactDampingRatio, context.h)
	context.staticSoftness = makeSoft(w.contactHertz.Add(w.contactHertz), w.contactDampingRatio, context.h)
	awake := &w.solverSets[awakeSet]
	context.sims = awake.bodySims
	context.states = awake.bodyStates

	return w, getBodyFullId(w, boxId), context
}

// TestMakeSoftSplitsTheUnit pins makeSoft: a zero frequency is rigid, and
// the mass and impulse scales of a soft constraint add up to one.
func TestMakeSoftSplitsTheUnit(t *testing.T) {
	if makeSoft(fixed.Q32Zero(), fixed.Q32One(), fixed.Q32Half()) != (softness{}) {
		t.Fatalf("a zero frequency is not rigid")
	}

	h := fixed.Q32FromRatio(1, 240)
	soft := makeSoft(fixed.Q32FromInt(30), fixed.Q32FromInt(10), h)

	// omega = 60 pi; a1 = 20 + omega / 240; biasRate = omega / a1.
	omega := tau.Mul(fixed.Q32FromInt(30))
	a1 := fixed.Q32FromInt(20).Add(omega.Mul(h))
	if !soft.biasRate.Eq(omega.Div(a1)) {
		t.Errorf("biasRate is %v, want %v", soft.biasRate, omega.Div(a1))
	}
	one := fixed.Q32One()
	tolerance := fixed.Q32FromRaw(4)
	if !withinQ(soft.massScale.Add(soft.impulseScale), one, tolerance) {
		t.Errorf("massScale %v + impulseScale %v is not one", soft.massScale, soft.impulseScale)
	}
	if !fixed.Q32Half().Less(soft.massScale) {
		t.Errorf("massScale %v is not the larger share at 30 Hz", soft.massScale)
	}
}

// TestPrepareOverflowContactsBuildsTheMasses pins the prepare stage with
// hand values: mass one and inertia one sixth give a normal mass of one
// under the center and a tangent mass of two fifths.
func TestPrepareOverflowContactsBuildsTheMasses(t *testing.T) {
	w, box, context := restingBox(t)
	state := getBodyState(w, box)
	state.linearVelocity = Vec2{Y: fixed.Q32FromInt(-3)}

	prepareOverflowContacts(context)

	constraint := &w.constraintGraph.colors[overflowIndex].overflowConstraints[0]
	if constraint.indexA != nullIndex || constraint.indexB != box.localIndex {
		t.Fatalf("the constraint points at bodies %d and %d", constraint.indexA, constraint.indexB)
	}
	if constraint.softness != context.staticSoftness {
		t.Errorf("a ground contact did not take the static softness")
	}
	tolerance := fixed.Q32FromRaw(64)
	if constraint.invMassB != fixed.Q32One() || !withinQ(constraint.invIB, fixed.Q32FromInt(6), tolerance) {
		t.Errorf("the box has inverse mass %v and inverse inertia %v, want 1 and 6", constraint.invMassB, constraint.invIB)
	}

	cp := &constraint.points[0]
	if !withinQ(cp.normalMass, fixed.Q32One(), tolerance) {
		t.Errorf("normalMass is %v, want 1", cp.normalMass)
	}
	if !withinQ(cp.tangentMass, fixed.Q32FromRatio(2, 5), tolerance) {
		t.Errorf("tangentMass is %v, want 0.4", cp.tangentMass)
	}
	// baseSeparation = 0 - dot(rB - rA, n) = -(-0.5 - 0.5) = 1
	if !cp.baseSeparation.Eq(fixed.Q32One()) {
		t.Errorf("baseSeparation is %v, want 1", cp.baseSeparation)
	}
	if !cp.relativeVelocity.Eq(fixed.Q32FromInt(-3)) {
		t.Errorf("relativeVelocity is %v, want -3", cp.relativeVelocity)
	}
}

// TestWarmStartReappliesTheStoredImpulse pins the warm start: the stored
// normal impulse of the manifold moves the box, and the world switch
// turns it off.
func TestWarmStartReappliesTheStoredImpulse(t *testing.T) {
	w, box, context := restingBox(t)
	m := &w.constraintGraph.colors[overflowIndex].contactSims[0].manifold
	m.Points[0].NormalImpulse = fixed.Q32FromInt(2)

	prepareOverflowContacts(context)
	warmStartOverflowContacts(context)

	state := getBodyState(w, box)
	if !state.linearVelocity.Y.Eq(fixed.Q32FromInt(2)) || !state.angularVelocity.Eq(fixed.Q32Zero()) {
		t.Errorf("the warm start gave velocity %v and spin %v, want (0, 2) and 0", state.linearVelocity, state.angularVelocity)
	}

	w.enableWarmStarting = false
	state.linearVelocity = Vec2Zero()
	prepareOverflowContacts(context)
	warmStartOverflowContacts(context)
	if !state.linearVelocity.Y.Eq(fixed.Q32Zero()) {
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
		m.Points[0].Separation = sim.center.Y.Sub(fixed.Q32Half())
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
		setBitCountAndClear(&w.taskContext.awakeIslandBitSet, len(w.solverSets[awakeSet].islandSims))
		finalizeBodiesTask(0, 1, context)
	}

	tolerance := fixed.Q32MustParse("0.001")
	if !withinQ(sim.center.Y, fixed.Q32Half(), tolerance) {
		t.Errorf("the box rests at y %v, want 0.5", sim.center.Y)
	}
	state := getBodyState(w, box)
	if !withinQ(state.linearVelocity.Y, fixed.Q32Zero(), tolerance) {
		t.Errorf("the box moves at %v", state.linearVelocity.Y)
	}

	// weight * h = 10 / 240
	weightImpulse := fixed.Q32FromRatio(10, 240)
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
	state.linearVelocity = Vec2{X: fixed.Q32FromInt(100), Y: fixed.Q32FromInt(-1)}

	prepareOverflowContacts(context)
	solveOverflowContacts(context, true)

	cp := &w.constraintGraph.colors[overflowIndex].overflowConstraints[0].points[0]
	constraint := &w.constraintGraph.colors[overflowIndex].overflowConstraints[0]
	if !fixed.Q32Zero().Less(cp.normalImpulse) {
		t.Fatalf("the normal impulse is %v, want positive", cp.normalImpulse)
	}
	want := constraint.friction.Mul(cp.normalImpulse).Neg()
	if !cp.tangentImpulse.Eq(want) {
		t.Errorf("the tangent impulse is %v, want %v", cp.tangentImpulse, want)
	}
	// A slide to the right rolls the box clockwise.
	if !state.angularVelocity.Less(fixed.Q32Zero()) {
		t.Errorf("the friction under the center did not roll the box")
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
			w.constraintGraph.colors[overflowIndex].contactSims[0].restitution = fixed.Q32Half()
			state := getBodyState(w, box)
			state.linearVelocity = Vec2{Y: fixed.Q32MustParse(tc.fall)}

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
			want := fixed.Q32MustParse("1.5")
			if !withinQ(state.linearVelocity.Y, want, fixed.Q32FromRaw(16)) {
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
	state.linearVelocity = Vec2{X: fixed.Q32FromInt(1), Y: fixed.Q32FromInt(-2)}

	prepareOverflowContacts(context)
	solveOverflowContacts(context, true)
	storeOverflowImpulses(context)

	cp := &w.constraintGraph.colors[overflowIndex].overflowConstraints[0].points[0]
	mp := &w.constraintGraph.colors[overflowIndex].contactSims[0].manifold.Points[0]
	if mp.NormalImpulse != cp.normalImpulse || mp.TangentImpulse != cp.tangentImpulse || mp.TotalNormalImpulse != cp.totalNormalImpulse {
		t.Errorf("the manifold holds %v, %v, %v, want %v, %v, %v", mp.NormalImpulse, mp.TangentImpulse, mp.TotalNormalImpulse, cp.normalImpulse, cp.tangentImpulse, cp.totalNormalImpulse)
	}
	if !mp.NormalVelocity.Eq(fixed.Q32FromInt(-2)) {
		t.Errorf("the normal velocity is %v, want -2", mp.NormalVelocity)
	}
}
