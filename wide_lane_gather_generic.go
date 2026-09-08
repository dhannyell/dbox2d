//go:build dbox2d_wide && dbox2d_float && (!goexperiment.simd || !go1.27 || !amd64)

package dbox2d

// bodyScratchW carries the gathered scalars between the body states and the lanes.
type bodyScratchW struct{ vx, vy, w, dpx, dpy, dqc, dqs [wideWidth]float32 }

// gatherBodies fills every lane of the scratch: real bodies from the states, null lanes as the identity.
func gatherBodies(states []bodyState, indices *[wideWidth]int, s *bodyScratchW) {
	for j := range wideWidth {
		idx := indices[j]
		if idx == nullIndex {
			s.vx[j], s.vy[j], s.w[j], s.dpx[j], s.dpy[j], s.dqs[j] = 0, 0, 0, 0, 0, 0
			s.dqc[j] = 1
			continue
		}
		b := &states[idx]
		s.vx[j] = b.linearVelocity.X.v
		s.vy[j] = b.linearVelocity.Y.v
		s.w[j] = b.angularVelocity.v
		s.dpx[j] = b.deltaPosition.X.v
		s.dpy[j] = b.deltaPosition.Y.v
		s.dqc[j] = b.deltaRotation.Cos.v
		s.dqs[j] = b.deltaRotation.Sin.v
	}
}

// loadBodyVelocityW builds the velocity lanes; the angular velocity becomes radians per second.
func loadBodyVelocityW(s *bodyScratchW, tauW laneW, b *bodyStateW) {
	b.v.x = laneLoad(&s.vx).toAcc()
	b.v.y = laneLoad(&s.vy).toAcc()
	b.w = laneLoad(&s.w).Mul(tauW).toAcc()
}

// loadBodyDeltaW builds the delta position and delta rotation lanes.
func loadBodyDeltaW(s *bodyScratchW, b *bodyStateW) {
	b.dp = vec2W{x: laneLoad(&s.dpx), y: laneLoad(&s.dpy)}
	b.dq = rotW{c: laneLoad(&s.dqc), s: laneLoad(&s.dqs)}
}

// storeBodyW writes the velocities to the scratch, the angular velocity back in turns per second.
func storeBodyW(b *bodyStateW, tauW laneW, s *bodyScratchW) {
	b.v.x.toLane().store(&s.vx)
	b.v.y.toLane().store(&s.vy)
	b.w.toLane().Div(tauW).store(&s.w)
}

// scatterBodies writes the real-body velocities back from the scratch.
func scatterBodies(states []bodyState, indices *[wideWidth]int, s *bodyScratchW) {
	for j := range wideWidth {
		idx := indices[j]
		if idx == nullIndex {
			continue
		}
		b := &states[idx]
		b.linearVelocity.X.v = s.vx[j]
		b.linearVelocity.Y.v = s.vy[j]
		b.angularVelocity.v = s.w[j]
	}
}

// gatherBodyW loads one constraint's bodies; null lanes are identities.
// Angular velocity becomes radians per second.
func gatherBodyW(states []bodyState, indices *[wideWidth]int, tauW laneW, b *bodyStateW) {
	var scratch bodyScratchW
	gatherBodies(states, indices, &scratch)
	loadBodyVelocityW(&scratch, tauW, b)
	loadBodyDeltaW(&scratch, b)
	b.flags = laneZero()
}

// scatterBodyW writes real-lane velocities back with angular velocity in turns per second.
func scatterBodyW(states []bodyState, indices *[wideWidth]int, tauW laneW, b *bodyStateW) {
	var scratch bodyScratchW
	storeBodyW(b, tauW, &scratch)
	scatterBodies(states, indices, &scratch)
}
