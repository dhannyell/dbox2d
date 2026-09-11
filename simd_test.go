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
		activeContactCount += count
	}
	colors[overflowIndex].contactSims = make([]contactSim, 1)

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

	scalarWorld := createTestWorld(t)
	wideWorld := createTestWorld(t)
	buildChecksumWitness(t, scalarWorld)
	buildChecksumWitness(t, wideWorld)
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
}
