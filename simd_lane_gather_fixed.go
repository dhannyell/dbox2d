//go:build dbox2d_simd && dbox2d_fixed

package dbox2d

import "github.com/dhannyell/fixed"

// bodyScratchW carries the gathered scalars between the body states and the
// lanes. The velocities travel in the accumulator grid, as in the scalar
// contact stages; only the deltas are narrowed to the lane grid.
type bodyScratchW struct {
	vx, vy, w          [wideWidth]accScalar
	dpx, dpy, dqc, dqs [wideWidth]laneScalar
}

// gatherBodies fills the scratch; null lanes get the identity. The tau scaling
// runs in Q32 before the rounding, as in the scalar contact stages.
func gatherBodies(states []bodyState, indices *[wideWidth]int, s *bodyScratchW) {
	zero := fixed.Q16Zero()
	for j := range wideWidth {
		idx := indices[j]
		if idx == nullIndex {
			s.vx[j], s.vy[j], s.w[j] = fixed.Q48Zero(), fixed.Q48Zero(), fixed.Q48Zero()
			s.dpx[j], s.dpy[j], s.dqs[j] = zero, zero, zero
			s.dqc[j] = fixed.Q16One()
			continue
		}
		b := &states[idx]
		s.vx[j] = accScalarFromQ(b.linearVelocity.X)
		s.vy[j] = accScalarFromQ(b.linearVelocity.Y)
		s.w[j] = accScalarFromQ(b.angularVelocity.Mul(tau))
		s.dpx[j] = laneScalarFromQ(b.deltaPosition.X)
		s.dpy[j] = laneScalarFromQ(b.deltaPosition.Y)
		s.dqc[j] = laneScalarFromQ(b.deltaRotation.Cos)
		s.dqs[j] = laneScalarFromQ(b.deltaRotation.Sin)
	}
}

// loadBodyVelocityW builds the velocity accumulators from the scratch.
func loadBodyVelocityW(s *bodyScratchW, b *bodyStateW) {
	b.v.x = accLoad(&s.vx)
	b.v.y = accLoad(&s.vy)
	b.w = accLoad(&s.w)
}

// loadBodyDeltaW builds the delta position and delta rotation lanes.
func loadBodyDeltaW(s *bodyScratchW, b *bodyStateW) {
	b.dp = vec2W{x: laneLoad(&s.dpx), y: laneLoad(&s.dpy)}
	b.dq = rotW{c: laneLoad(&s.dqc), s: laneLoad(&s.dqs)}
}

// storeBodyW writes the velocity accumulators to the scratch.
func storeBodyW(b *bodyStateW, s *bodyScratchW) {
	b.v.x.store(&s.vx)
	b.v.y.store(&s.vy)
	b.w.store(&s.w)
}

// scatterBodies writes the real-lane velocities back. The tau division runs
// in Q32, as in the scalar store.
func scatterBodies(states []bodyState, indices *[wideWidth]int, s *bodyScratchW) {
	for j := range wideWidth {
		idx := indices[j]
		if idx == nullIndex {
			continue
		}
		b := &states[idx]
		b.linearVelocity.X = accScalarToQ(s.vx[j])
		b.linearVelocity.Y = accScalarToQ(s.vy[j])
		b.angularVelocity = accScalarToQ(s.w[j]).Div(tau)
	}
}

// gatherBodyW loads one constraint's bodies. tauW is unused: gatherBodies
// already scaled by tau.
func gatherBodyW(states []bodyState, indices *[wideWidth]int, tauW laneW, b *bodyStateW) {
	var scratch bodyScratchW
	gatherBodies(states, indices, &scratch)
	loadBodyVelocityW(&scratch, b)
	loadBodyDeltaW(&scratch, b)
	b.flags = laneZero()
}

// scatterBodyW writes one constraint's velocities back. tauW is unused:
// scatterBodies divides by tau.
func scatterBodyW(states []bodyState, indices *[wideWidth]int, tauW laneW, b *bodyStateW) {
	var scratch bodyScratchW
	storeBodyW(b, &scratch)
	scatterBodies(states, indices, &scratch)
}
