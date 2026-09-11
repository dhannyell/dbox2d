//go:build dbox2d_simd && !dbox2d_fixed && (!goexperiment.simd || !go1.27 || !amd64)

package dbox2d

// bodyScratchW carries the gathered scalars between the body states and the lanes.
type bodyScratchW struct{ vx, vy, w, dpx, dpy, dqc, dqs [wideWidth]laneScalar }

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
		s.vx[j] = laneScalarFromQ(b.linearVelocity.X)
		s.vy[j] = laneScalarFromQ(b.linearVelocity.Y)
		s.w[j] = laneScalarFromQ(b.angularVelocity)
		s.dpx[j] = laneScalarFromQ(b.deltaPosition.X)
		s.dpy[j] = laneScalarFromQ(b.deltaPosition.Y)
		s.dqc[j] = laneScalarFromQ(b.deltaRotation.Cos)
		s.dqs[j] = laneScalarFromQ(b.deltaRotation.Sin)
	}
}

// loadBodyVelocityW builds the velocity lanes.
func loadBodyVelocityW(s *bodyScratchW, b *bodyStateW) {
	b.v.x = laneLoad(&s.vx).toAcc()
	b.v.y = laneLoad(&s.vy).toAcc()
	b.w = laneLoad(&s.w).toAcc()
}

// loadBodyDeltaW builds the delta position and delta rotation lanes.
func loadBodyDeltaW(s *bodyScratchW, b *bodyStateW) {
	b.dp = vec2W{x: laneLoad(&s.dpx), y: laneLoad(&s.dpy)}
	b.dq = rotW{c: laneLoad(&s.dqc), s: laneLoad(&s.dqs)}
}

// storeBodyW writes the velocities to the scratch.
func storeBodyW(b *bodyStateW, s *bodyScratchW) {
	b.v.x.toLane().store(&s.vx)
	b.v.y.toLane().store(&s.vy)
	b.w.toLane().store(&s.w)
}

// scatterBodies writes the real-body velocities back from the scratch.
func scatterBodies(states []bodyState, indices *[wideWidth]int, s *bodyScratchW) {
	for j := range wideWidth {
		idx := indices[j]
		if idx == nullIndex {
			continue
		}
		b := &states[idx]
		b.linearVelocity.X = laneScalarToQ(s.vx[j])
		b.linearVelocity.Y = laneScalarToQ(s.vy[j])
		b.angularVelocity = laneScalarToQ(s.w[j])
	}
}

// gatherBodyW loads one constraint's bodies; null lanes are identities.
func gatherBodyW(states []bodyState, indices *[wideWidth]int, b *bodyStateW) {
	var scratch bodyScratchW
	gatherBodies(states, indices, &scratch)
	loadBodyVelocityW(&scratch, b)
	loadBodyDeltaW(&scratch, b)
	b.flags = laneZero()
}

// scatterBodyW writes real-lane velocities back.
func scatterBodyW(states []bodyState, indices *[wideWidth]int, b *bodyStateW) {
	var scratch bodyScratchW
	storeBodyW(b, &scratch)
	scatterBodies(states, indices, &scratch)
}
