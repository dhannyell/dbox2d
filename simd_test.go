//go:build dbox2d_simd

package dbox2d

import "testing"

// TestWidePath records the selected path and checks its width relation.
func TestWidePath(t *testing.T) {
	t.Logf("path: %s width %d", widePath(), wideWidth)
	if wideWidth != 1<<wideShift {
		t.Fatalf("width %d does not match shift %d", wideWidth, wideShift)
	}
	if widePath() == "generic" && !wideAvailable() {
		t.Fatal("generic path must be available")
	}
}

// TestRollingBoundWKeepsATwoPointTotal pins the lane rolling bound when the
// total of two points passes the lane range.
func TestRollingBoundWKeepsATwoPointTotal(t *testing.T) {
	for _, tc := range []struct{ rr, total, want Q }{
		{QFromRatio(1, 8), QFromInt(40000), QFromInt(5000)},
		// An exact product: one rounding, and no saturation in the split.
		{QHalf(), QFromInt(32768), QFromInt(16384)},
	} {
		rr := laneSplat(tc.rr)
		half := laneSplat(tc.total.Div(QFromInt(2))).toAcc()
		before := saturationCount()
		var got [wideWidth]laneScalar
		rollingBoundW(rr, half.Add(half)).store(&got)
		for j := range wideWidth {
			if !laneScalarToQ(got[j]).Eq(tc.want) {
				t.Fatalf("lane %d: the rolling bound of %v·%v is %v, want %v", j, tc.rr, tc.total, laneScalarToQ(got[j]), tc.want)
			}
		}
		if n := saturationCount() - before; n != 0 {
			t.Fatalf("the rolling bound of %v·%v saturated %d times", tc.rr, tc.total, n)
		}
	}
}

// TestWideConstraintCountRoundsUp keeps scalar contact grouping width-independent.
func TestWideConstraintCountRoundsUp(t *testing.T) {
	for _, n := range []int{0, 1, 4, 5, 8, 9, 17} {
		want := 0
		if n > 0 {
			want = (n + wideWidth - 1) / wideWidth
		}
		if got := wideConstraintCount(n); got != want {
			t.Fatalf("contact count %d: got %d want %d", n, got, want)
		}
	}
}

// TestWideContactLayoutPadsEachColor protects global wide bases and nil tails.
func TestWideContactLayoutPadsEachColor(t *testing.T) {
	if !wideAvailable() || !wideEnabled {
		t.Fatal("the wide path is unavailable on this host")
	}

	var colors [graphColorCount]graphColor
	counts := map[int]int{0: 3, 2: wideWidth + 1}
	activeContactCount := 0
	for colorIndex, count := range counts {
		colors[colorIndex].contactSims = make([]contactSim, count)
		colors[colorIndex].contacts = testContactPointers(colors[colorIndex].contactSims)
		activeContactCount += count
	}
	colors[overflowIndex].contactSims = make([]contactSim, 1)
	colors[overflowIndex].contacts = testContactPointers(colors[overflowIndex].contactSims)

	w := world{arena: createArenaAllocator(1 << 20)}
	context := stepContext{}
	allocateContactConstraints(&w, &context, &colors, overflowIndex, activeContactCount)
	defer func() {
		w.arena.freeItem(context.contactConstraintMem)
		w.arena.freeItem(context.contactConstraintMemWide)
	}()

	wantWideCount := 0
	for colorIndex := range overflowIndex {
		wantWideCount += wideConstraintCount(len(colors[colorIndex].contactSims))
	}
	if len(context.contactConstraintsWide) != wantWideCount || len(context.contacts) != wideWidth*wantWideCount {
		t.Fatalf("wide layout has %d constraints and %d contacts, want %d and %d", len(context.contactConstraintsWide), len(context.contacts), wantWideCount, wideWidth*wantWideCount)
	}

	wideBase := 0
	for colorIndex := range overflowIndex {
		color := &colors[colorIndex]
		wideCount := wideConstraintCount(len(color.contactSims))
		if len(color.contactConstraintsWide) != wideCount {
			t.Fatalf("color %d has %d wide constraints, want %d", colorIndex, len(color.contactConstraintsWide), wideCount)
		}
		base := wideWidth * wideBase
		for j := range wideWidth * wideCount {
			got := context.contacts[base+j]
			if j < len(color.contactSims) {
				want := &color.contactSims[j]
				if got != want {
					t.Fatalf("color %d contact %d is %p, want %p", colorIndex, j, got, want)
				}
			} else if got != nil {
				t.Fatalf("color %d tail lane %d is not nil", colorIndex, j)
			}
		}
		wideBase += wideCount
	}
	if len(colors[overflowIndex].contactConstraints) != 1 || len(colors[overflowIndex].contactConstraintsWide) != 0 {
		t.Fatal("the overflow color did not keep scalar constraints")
	}
}

// TestWideStagesRunWithColoredContacts proves Step selects non-empty wide scratch.
func TestWideStagesRunWithColoredContacts(t *testing.T) {
	if !wideAvailable() {
		t.Fatal("the wide path is unavailable on this host")
	}
	if wideEnabled != wideAvailable() {
		t.Fatalf("wideEnabled is %t, wideAvailable is %t", wideEnabled, wideAvailable())
	}

	worldId := createTestWorld(t)
	idA := addDynamicCircle(t, worldId, v2(0, 0))
	idB := addDynamicCircle(t, worldId, Vec2{X: QMustParse("0.75")})
	startTouching(t, getWorldFromId(worldId), idA, idB)

	before := wideContactAllocations.Load()
	worldId.Step(stepDt(), 4)
	if wideContactAllocations.Load() == before {
		t.Fatal("Step did not allocate a non-empty wide contact color")
	}
}

// TestWideMatchesScalarStepByStep pins the wide family to its scalar oracle.
func TestWideMatchesScalarStepByStep(t *testing.T) {
	if !wideAvailable() {
		t.Fatal("the wide path is unavailable on this host")
	}
	defer func() { wideEnabled = wideAvailable() }()

	for _, scene := range []struct {
		name  string
		build func(*testing.T, WorldId)
	}{
		{"witness", buildChecksumWitness},
		// Each landing impulse fits Q16, but its per-step totals do not.
		{"heavy landing", buildHeavyLanding},
		// One color holds a lane contact and a Q32 contact.
		{"mixed grid", buildMixedGrid},
		// A lane block mixes restitutions 0 and 0.5, and a Q32 contact of a
		// later color shares a body with a lane without restitution.
		{"mixed restitution", buildMixedRestitution},
	} {
		t.Run(scene.name, func(t *testing.T) {
			scalarWorld := createTestWorld(t)
			wideWorld := createTestWorld(t)
			scene.build(t, scalarWorld)
			scene.build(t, wideWorld)
			dt := stepDt()
			for step := range 120 {
				wideEnabled = false
				scalarWorld.Step(dt, 4)
				wideEnabled = true
				wideWorld.Step(dt, 4)
				if scalarChecksum, wideChecksum := Checksum(scalarWorld), Checksum(wideWorld); scalarChecksum != wideChecksum {
					scalarState := getWorldFromId(scalarWorld)
					wideState := getWorldFromId(wideWorld)
					for i := range scalarState.bodies {
						if scalarState.bodies[i].id == nullIndex {
							continue
						}
						if checksumBody(scalarState, &scalarState.bodies[i]) != checksumBody(wideState, &wideState.bodies[i]) {
							sb := &scalarState.bodies[i]
							wb := &wideState.bodies[i]
							// Only the awake set keeps body states.
							if sb.setIndex != awakeSet || wb.setIndex != awakeSet {
								t.Logf("body %d scalar set=%d wide set=%d", i, sb.setIndex, wb.setIndex)
								continue
							}
							t.Logf("body %d scalar sim=%#v state=%#v", i, scalarState.solverSets[sb.setIndex].bodySims[sb.localIndex], scalarState.solverSets[sb.setIndex].bodyStates[sb.localIndex])
							t.Logf("body %d wide   sim=%#v state=%#v", i, wideState.solverSets[wb.setIndex].bodySims[wb.localIndex], wideState.solverSets[wb.setIndex].bodyStates[wb.localIndex])
						}
					}
					for i := range scalarState.contacts {
						if scalarState.contacts[i].contactId == nullIndex {
							continue
						}
						if checksumContact(scalarState, &scalarState.contacts[i]) != checksumContact(wideState, &wideState.contacts[i]) {
							t.Logf("contact %d scalar=%#v", i, getContactSim(scalarState, &scalarState.contacts[i]).manifold)
							t.Logf("contact %d wide  =%#v", i, getContactSim(wideState, &wideState.contacts[i]).manifold)
						}
					}
					t.Fatalf("step %d: wide checksum %d, scalar checksum %d", step+1, wideChecksum, scalarChecksum)
				}
			}
		})
	}
}

// buildMixedGrid drops a unit box and a 200000 kg box onto the ground.
func buildMixedGrid(t *testing.T, worldId WorldId) {
	t.Helper()
	boxOnGround(t, worldId, QMustParse("0.05"))
	heavyBox(worldId, Vec2{X: QFromInt(2), Y: QHalf().Add(QMustParse("0.05"))}, Vec2{})
}

// buildMixedRestitution rests two unit boxes side by side and a bouncy one
// apart on the ground, with a 200000 kg box on top of the first two.
func buildMixedRestitution(t *testing.T, worldId WorldId) {
	t.Helper()
	groundDef := DefaultBodyDef()
	groundDef.Position = Vec2{Y: QHalf().Neg()}
	groundId := CreateBody(worldId, &groundDef)
	shapeDef := DefaultShapeDef()
	ground := MakeBox(QFromInt(10), QHalf())
	CreatePolygonShape(groundId, &shapeDef, &ground)

	boxWithRestitution(worldId, Vec2{Y: QHalf()}, QZero())
	boxWithRestitution(worldId, Vec2{X: QOne(), Y: QHalf()}, QZero())
	boxWithRestitution(worldId, Vec2{X: QFromInt(3), Y: QHalf()}, QHalf())
	heavyBox(worldId, Vec2{Y: QOne().Add(QHalf())}, Vec2{})
}

// boxWithRestitution adds a unit box of the default density at the position.
func boxWithRestitution(worldId WorldId, position Vec2, restitution Q) BodyId {
	boxDef := DefaultBodyDef()
	boxDef.Type = DynamicBody
	boxDef.Position = position
	boxId := CreateBody(worldId, &boxDef)
	shapeDef := DefaultShapeDef()
	shapeDef.Material.Restitution = restitution
	unit := MakeBox(QHalf(), QHalf())
	CreatePolygonShape(boxId, &shapeDef, &unit)
	return boxId
}

// buildHeavyLanding drops a 2000 kg box onto the ground at 20 m/s.
func buildHeavyLanding(t *testing.T, worldId WorldId) {
	t.Helper()
	groundDef := DefaultBodyDef()
	groundId := CreateBody(worldId, &groundDef)
	shapeDef := DefaultShapeDef()
	ground := MakeBox(QFromInt(10), QOne())
	CreatePolygonShape(groundId, &shapeDef, &ground)

	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = v2(0, 4)
	bodyDef.LinearVelocity = v2(0, -20)
	bodyId := CreateBody(worldId, &bodyDef)
	shapeDef.Density = QFromInt(500) // 2 m x 2 m, so 2000 kg
	box := MakeSquare(QOne())
	CreatePolygonShape(bodyId, &shapeDef, &box)
}
