//go:build dbox2d_wide && dbox2d_float

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

// TestPackWideColorPreservesContactIndices keeps real and tail lanes addressable.
func TestPackWideColorPreservesContactIndices(t *testing.T) {
	for _, contactCount := range []int{5, 17} {
		contacts := make([]*contactSim, contactCount)
		for i := range contacts {
			contacts[i] = &contactSim{bodySimIndexA: i, bodySimIndexB: i + 1000}
		}

		constraints := make([]contactConstraintWide, wideConstraintCount(contactCount))
		packWideColor(constraints, contacts)
		for k := 0; k < len(constraints)*wideWidth; k++ {
			constraintIndex := k / wideWidth
			laneIndex := k % wideWidth
			wantA := nullIndex
			wantB := nullIndex
			if k < contactCount {
				wantA = k
				wantB = k + 1000
			}
			if got := constraints[constraintIndex].indexA[laneIndex]; got != wantA {
				t.Fatalf("contacts %d lane %d indexA: got %d want %d", contactCount, k, got, wantA)
			}
			if got := constraints[constraintIndex].indexB[laneIndex]; got != wantB {
				t.Fatalf("contacts %d lane %d indexB: got %d want %d", contactCount, k, got, wantB)
			}
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
			flags:           3 + i,
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

	got := gatherBodies(states, &indices)
	var vx, vy, w, flags, dpx, dpy, dqc, dqs [wideWidth]float32
	got.v.x.store(&vx)
	got.v.y.store(&vy)
	got.w.store(&w)
	got.flags.store(&flags)
	got.dp.x.store(&dpx)
	got.dp.y.store(&dpy)
	got.dq.c.store(&dqc)
	got.dq.s.store(&dqs)

	if math.Float32bits(vx[nullLane]) != math.Float32bits(0) ||
		math.Float32bits(vy[nullLane]) != math.Float32bits(0) ||
		math.Float32bits(w[nullLane]) != math.Float32bits(0) ||
		math.Float32bits(flags[nullLane]) != math.Float32bits(0) ||
		math.Float32bits(dpx[nullLane]) != math.Float32bits(0) ||
		math.Float32bits(dpy[nullLane]) != math.Float32bits(0) ||
		math.Float32bits(dqc[nullLane]) != math.Float32bits(1) ||
		math.Float32bits(dqs[nullLane]) != math.Float32bits(0) {
		t.Fatalf("null lane: v=(%#08x,%#08x) w=%#08x flags=%#08x dp=(%#08x,%#08x) dq=(%#08x,%#08x)", math.Float32bits(vx[nullLane]), math.Float32bits(vy[nullLane]), math.Float32bits(w[nullLane]), math.Float32bits(flags[nullLane]), math.Float32bits(dpx[nullLane]), math.Float32bits(dpy[nullLane]), math.Float32bits(dqc[nullLane]), math.Float32bits(dqs[nullLane]))
	}
	for i := range nullLane {
		s := &states[indices[i]]
		wantW := s.angularVelocity.Mul(tau).v
		values := [][2]float32{
			{vx[i], s.linearVelocity.X.v},
			{vy[i], s.linearVelocity.Y.v},
			{w[i], wantW},
			{flags[i], float32(s.flags)},
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
			flags:           7 + i,
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

	got := gatherBodies(states, &indices)
	got.v.x = got.v.x.Add(laneSplat(Q{v: 0.25}))
	got.v.y = got.v.y.Sub(laneSplat(Q{v: 0.5}))
	got.w = got.w.Add(laneSplat(Q{v: 1.25}))
	var wantVx, wantVy, wantW [wideWidth]float32
	got.v.x.store(&wantVx)
	got.v.y.store(&wantVy)
	got.w.store(&wantW)

	after := append([]bodyState(nil), before...)
	scatterBodies(after, &indices, &got)
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
