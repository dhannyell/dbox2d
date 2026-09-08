//go:build dbox2d_simd && dbox2d_float

package dbox2d

import (
	"math"
	"testing"
)

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

// TestGatherBodiesSubstitutesIdentityForNull checks state conversion per lane.
func TestGatherBodiesSubstitutesIdentityForNull(t *testing.T) {
	states := make([]bodyState, wideWidth)
	for i := range states {
		states[i] = bodyState{
			linearVelocity:  Vec2{X: Q{v: 1.25 + float32(i)}, Y: Q{v: -2.5 - float32(i)}},
			angularVelocity: Q{v: 0.125 * float32(i+1)},
			flags:           int32(3 + i),
			deltaPosition:   Vec2{X: Q{v: 0.5 + float32(i)}, Y: Q{v: -0.75 - float32(i)}},
			deltaRotation:   Rot{Sin: Q{v: 0.1 + 0.01*float32(i)}, Cos: Q{v: 0.9 - 0.02*float32(i)}},
		}
	}
	nullLane := wideWidth - 1
	indices := [wideWidth]int{}
	for i := range indices {
		indices[i] = i
	}
	indices[nullLane] = nullIndex

	var body bodyStateW
	gatherBodyW(states, &indices, laneSplat(tau), &body)
	var vx, vy, w, dpx, dpy, dqc, dqs [wideWidth]float32
	body.v.x.store(&vx)
	body.v.y.store(&vy)
	body.w.store(&w)
	body.dp.x.store(&dpx)
	body.dp.y.store(&dpy)
	body.dq.c.store(&dqc)
	body.dq.s.store(&dqs)

	if math.Float32bits(vx[nullLane]) != math.Float32bits(0) ||
		math.Float32bits(vy[nullLane]) != math.Float32bits(0) ||
		math.Float32bits(w[nullLane]) != math.Float32bits(0) ||
		math.Float32bits(dpx[nullLane]) != math.Float32bits(0) ||
		math.Float32bits(dpy[nullLane]) != math.Float32bits(0) ||
		math.Float32bits(dqc[nullLane]) != math.Float32bits(1) ||
		math.Float32bits(dqs[nullLane]) != math.Float32bits(0) {
		t.Fatalf("null lane: v=(%#08x,%#08x) w=%#08x dp=(%#08x,%#08x) dq=(%#08x,%#08x)", math.Float32bits(vx[nullLane]), math.Float32bits(vy[nullLane]), math.Float32bits(w[nullLane]), math.Float32bits(dpx[nullLane]), math.Float32bits(dpy[nullLane]), math.Float32bits(dqc[nullLane]), math.Float32bits(dqs[nullLane]))
	}
	for i := range nullLane {
		s := &states[indices[i]]
		values := [][2]float32{
			{vx[i], s.linearVelocity.X.v},
			{vy[i], s.linearVelocity.Y.v},
			{w[i], s.angularVelocity.Mul(tau).v},
			{dpx[i], s.deltaPosition.X.v},
			{dpy[i], s.deltaPosition.Y.v},
			{dqc[i], s.deltaRotation.Cos.v},
			{dqs[i], s.deltaRotation.Sin.v},
		}
		for field, value := range values {
			if math.Float32bits(value[0]) != math.Float32bits(value[1]) {
				t.Fatalf("real lane %d field %d: got %#08x want %#08x", i, field, math.Float32bits(value[0]), math.Float32bits(value[1]))
			}
		}
	}
}

// TestScatterBodiesRestoresConvertedVelocities protects null lanes and bits.
func TestScatterBodiesRestoresConvertedVelocities(t *testing.T) {
	states := make([]bodyState, wideWidth)
	for i := range states {
		states[i] = bodyState{
			linearVelocity:  Vec2{X: Q{v: 2.25 + float32(i)}, Y: Q{v: -3.5 - float32(i)}},
			angularVelocity: Q{v: 0.0625 * float32(i+1)},
			flags:           int32(7 + i),
			deltaPosition:   Vec2{X: Q{v: 1.5 + float32(i)}, Y: Q{v: -1.75 - float32(i)}},
			deltaRotation:   Rot{Sin: Q{v: 0.2 + 0.01*float32(i)}, Cos: Q{v: 0.8 - 0.02*float32(i)}},
		}
	}
	before := append([]bodyState(nil), states...)
	nullLane := wideWidth - 1
	indices := [wideWidth]int{}
	for i := range indices {
		indices[i] = i
	}
	indices[nullLane] = nullIndex

	tauW := laneSplat(tau)
	var got bodyStateW
	gatherBodyW(states, &indices, tauW, &got)
	got.v.x = got.v.x.Add(laneSplat(Q{v: 0.25}))
	got.v.y = got.v.y.Sub(laneSplat(Q{v: 0.5}))
	got.w = got.w.Add(laneSplat(Q{v: 1.25}))
	var wantVx, wantVy, wantW [wideWidth]float32
	got.v.x.store(&wantVx)
	got.v.y.store(&wantVy)
	got.w.store(&wantW)

	after := append([]bodyState(nil), before...)
	scatterBodyW(after, &indices, tauW, &got)
	if after[nullLane] != before[nullLane] {
		t.Fatalf("state reserved for null lane changed: got %#v want %#v", after[nullLane], before[nullLane])
	}
	for i := range nullLane {
		s := &after[indices[i]]
		wantAngular := Q{v: wantW[i]}.Div(tau).v
		if math.Float32bits(s.linearVelocity.X.v) != math.Float32bits(wantVx[i]) ||
			math.Float32bits(s.linearVelocity.Y.v) != math.Float32bits(wantVy[i]) ||
			math.Float32bits(s.angularVelocity.v) != math.Float32bits(wantAngular) {
			t.Fatalf("real lane %d: got v=(%#08x,%#08x) w=%#08x want v=(%#08x,%#08x) w=%#08x", i, math.Float32bits(s.linearVelocity.X.v), math.Float32bits(s.linearVelocity.Y.v), math.Float32bits(s.angularVelocity.v), math.Float32bits(wantVx[i]), math.Float32bits(wantVy[i]), math.Float32bits(wantAngular))
		}
	}
}
