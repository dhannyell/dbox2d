package dbox2d

import (
	"math"
	"testing"

	"github.com/dhannyell/fixed"
)

// This file measures the scalar penalty of Q32.32 against float64, and the
// allocation contract of Step. The velocity pair is the micro number; the
// Step pair mirrors the whole pipeline. A micro number diagnoses; only a
// composite benchmark decides an optimization.

const benchBodyCount = 1024

// makeBenchContext builds the integration inputs directly, without a world,
// so the benchmark isolates the arithmetic.
func makeBenchContext() *stepContext {
	w := &world{gravity: Vec2{Y: fixed.FromInt(-10)}}

	dt := fixed.One().Div(fixed.FromInt(60))
	context := &stepContext{
		world:             w,
		dt:                dt,
		invDt:             fixed.FromInt(60),
		h:                 dt.Div(fixed.FromInt(4)),
		subStepCount:      4,
		maxLinearVelocity: fixed.FromInt(400),
	}

	damping := fixed.MustParse("0.1")
	context.sims = make([]bodySim, benchBodyCount)
	context.states = make([]bodyState, benchBodyCount)
	for i := range benchBodyCount {
		sim := &context.sims[i]
		sim.invMass = fixed.One()
		sim.invInertia = fixed.FromInt(6)
		sim.linearDamping = damping
		sim.angularDamping = damping
		sim.gravityScale = fixed.One()
		state := &context.states[i]
		state.linearVelocity = Vec2{X: fixed.FromInt(i % 7), Y: fixed.FromInt(i % 5)}
		state.angularVelocity = fixed.MustParse("0.25")
	}
	return context
}

func BenchmarkIntegrateQ(b *testing.B) {
	context := makeBenchContext()
	b.ResetTimer()
	for range b.N {
		integrateVelocitiesTask(0, benchBodyCount, context)
	}
}

// f64BodySim mirrors the bodySim fields that the pipeline reads. The sleep
// fields of the body flatten into the sim, so the float64 pipeline stays one
// flat pass like the Q pipeline over the awake set.
type f64BodySim struct {
	invMass        float64
	invInertia     float64
	linearDamping  float64
	angularDamping float64
	gravityScale   float64
	forceX, forceY float64
	torque         float64

	px, py   float64
	qc, qs   float64
	cx, cy   float64
	c0x, c0y float64
	r0c, r0s float64
	lcx, lcy float64

	maxExtent      float64
	sleepTime      float64
	sleepThreshold float64

	isSpeedCapped     bool
	allowFastRotation bool
}

// f64BodyState mirrors the bodyState fields that the pipeline writes.
type f64BodyState struct {
	vx, vy   float64
	w        float64
	dpx, dpy float64
	dqc, dqs float64
}

// f64Shape mirrors one polygon shape: its local vertices and both bounds as
// [lower x, lower y, upper x, upper y].
type f64Shape struct {
	verts [4][2]float64
	aabb  [4]float64
	fat   [4]float64
}

// integrateVelocitiesF64 mirrors integrateVelocitiesTask line by line over
// float64, with the same division-based damping and the same turn unit.
func integrateVelocitiesF64(sims []f64BodySim, states []f64BodyState, gravityX, gravityY, h, invDt, maxLinearSpeed float64) {
	const tauF64 = 2 * math.Pi
	maxAngularSpeed := 0.125 * invDt
	maxLinearSpeedSquared := maxLinearSpeed * maxLinearSpeed
	maxAngularSpeedSquared := maxAngularSpeed * maxAngularSpeed

	for i := range sims {
		sim := &sims[i]
		state := &states[i]

		vx, vy := state.vx, state.vy
		w := state.w

		linearDamping := 1 + h*sim.linearDamping
		angularDamping := 1 + h*sim.angularDamping

		gravityScale := 0.0
		if sim.invMass > 0 {
			gravityScale = sim.gravityScale
		}

		deltaX := sim.forceX*(h*sim.invMass) + gravityX*(h*gravityScale)
		deltaY := sim.forceY*(h*sim.invMass) + gravityY*(h*gravityScale)
		deltaW := h * sim.invInertia * sim.torque / tauF64

		vx = deltaX + vx/linearDamping
		vy = deltaY + vy/linearDamping
		w = deltaW + w/angularDamping

		if vx*vx+vy*vy > maxLinearSpeedSquared {
			ratio := maxLinearSpeed / math.Sqrt(vx*vx+vy*vy)
			vx *= ratio
			vy *= ratio
			sim.isSpeedCapped = true
		}

		if w*w > maxAngularSpeedSquared && !sim.allowFastRotation {
			ratio := maxAngularSpeed / math.Abs(w)
			w *= ratio
			sim.isSpeedCapped = true
		}

		state.vx, state.vy = vx, vy
		state.w = w
	}
}

func BenchmarkIntegrateF64(b *testing.B) {
	sims := make([]f64BodySim, benchBodyCount)
	states := make([]f64BodyState, benchBodyCount)
	for i := range benchBodyCount {
		sim := &sims[i]
		sim.invMass = 1
		sim.invInertia = 6
		sim.linearDamping = 0.1
		sim.angularDamping = 0.1
		sim.gravityScale = 1
		state := &states[i]
		state.vx = float64(i % 7)
		state.vy = float64(i % 5)
		state.w = 0.25
	}

	dt := 1.0 / 60.0
	h := dt / 4
	b.ResetTimer()
	for range b.N {
		integrateVelocitiesF64(sims, states, 0, -10, h, 60, 400)
	}
}

// integratePositionsF64 mirrors integratePositionsTask line by line: the
// rotation integration converts turns to radians and normalizes.
func integratePositionsF64(states []f64BodyState, h float64) {
	const tauF64 = 2 * math.Pi
	for i := range states {
		state := &states[i]

		deltaAngle := h * state.w * tauF64
		c := state.dqc - deltaAngle*state.dqs
		s := state.dqs + deltaAngle*state.dqc
		mag := math.Sqrt(c*c + s*s)
		invMag := 0.0
		if mag > 0 {
			invMag = 1 / mag
		}
		state.dqc, state.dqs = c*invMag, s*invMag

		state.dpx += h * state.vx
		state.dpy += h * state.vy
	}
}

// finalizeF64 mirrors finalizeBodiesTask line by line: the transform update,
// the sleep arithmetic and the bounds refresh.
func finalizeF64(sims []f64BodySim, states []f64BodyState, shapes []f64Shape, dt, invDt float64) {
	const tauF64 = 2 * math.Pi
	const speculative = 4 * 0.005
	const margin = 0.05

	for i := range sims {
		sim := &sims[i]
		state := &states[i]

		sim.cx += state.dpx
		sim.cy += state.dpy

		c := state.dqc*sim.qc - state.dqs*sim.qs
		s := state.dqs*sim.qc + state.dqc*sim.qs
		mag := math.Sqrt(c*c + s*s)
		invMag := 0.0
		if mag > 0 {
			invMag = 1 / mag
		}
		sim.qc, sim.qs = c*invMag, s*invMag

		maxVelocity := math.Sqrt(state.vx*state.vx+state.vy*state.vy) + tauF64*math.Abs(state.w)*sim.maxExtent
		maxDeltaPosition := math.Sqrt(state.dpx*state.dpx+state.dpy*state.dpy) + math.Abs(state.dqs)*sim.maxExtent
		sleepVelocity := math.Max(maxVelocity, 0.5*invDt*maxDeltaPosition)

		state.dpx, state.dpy = 0, 0
		state.dqc, state.dqs = 1, 0

		sim.px = sim.cx - (sim.qc*sim.lcx - sim.qs*sim.lcy)
		sim.py = sim.cy - (sim.qs*sim.lcx + sim.qc*sim.lcy)

		sim.forceX, sim.forceY = 0, 0
		sim.torque = 0
		sim.isSpeedCapped = false

		if sleepVelocity > sim.sleepThreshold {
			sim.sleepTime = 0
		} else {
			sim.sleepTime += dt
		}
		sim.c0x, sim.c0y = sim.cx, sim.cy
		sim.r0c, sim.r0s = sim.qc, sim.qs

		shape := &shapes[i]
		lx, ly := math.Inf(1), math.Inf(1)
		ux, uy := math.Inf(-1), math.Inf(-1)
		for _, v := range shape.verts {
			x := sim.px + sim.qc*v[0] - sim.qs*v[1]
			y := sim.py + sim.qs*v[0] + sim.qc*v[1]
			lx = math.Min(lx, x)
			ly = math.Min(ly, y)
			ux = math.Max(ux, x)
			uy = math.Max(uy, y)
		}
		shape.aabb = [4]float64{lx - speculative, ly - speculative, ux + speculative, uy + speculative}

		if shape.aabb[0] < shape.fat[0] || shape.aabb[1] < shape.fat[1] ||
			shape.fat[2] < shape.aabb[2] || shape.fat[3] < shape.aabb[3] {
			shape.fat = [4]float64{
				shape.aabb[0] - margin, shape.aabb[1] - margin,
				shape.aabb[2] + margin, shape.aabb[3] + margin,
			}
		}
	}
}

// stepF64 mirrors the solve pipeline of Step: the sub-step loop and one
// finalize pass.
func stepF64(sims []f64BodySim, states []f64BodyState, shapes []f64Shape, dt float64, subStepCount int) {
	h := dt / float64(subStepCount)
	invDt := 1 / dt
	for range subStepCount {
		integrateVelocitiesF64(sims, states, 0, -10, h, invDt, 400)
		integratePositionsF64(states, h)
	}
	finalizeF64(sims, states, shapes, dt, invDt)
}

func BenchmarkStep(b *testing.B) {
	def := DefaultWorldDef()
	worldId := CreateWorld(&def)
	defer DestroyWorld(worldId)

	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	shapeDef := DefaultShapeDef()
	box := MakeSquare(fixed.One())
	for i := range benchBodyCount {
		bodyDef.Position = Vec2{X: fixed.FromInt(i * 3), Y: fixed.FromInt(i % 16)}
		bodyId := CreateBody(worldId, &bodyDef)
		CreatePolygonShape(bodyId, &shapeDef, &box)
	}

	dt := fixed.One().Div(fixed.FromInt(60))
	b.ReportAllocs()
	b.ResetTimer()
	for range b.N {
		Step(worldId, dt, 4)
	}
}

// BenchmarkStepF64 runs the float64 mirror of the whole pipeline over the
// same world shape as BenchmarkStep: 1024 free unit boxes under gravity.
func BenchmarkStepF64(b *testing.B) {
	sims := make([]f64BodySim, benchBodyCount)
	states := make([]f64BodyState, benchBodyCount)
	shapes := make([]f64Shape, benchBodyCount)
	for i := range benchBodyCount {
		sim := &sims[i]
		// A 2 by 2 unit box with density 1: mass 4, inertia 8/3.
		sim.invMass = 0.25
		sim.invInertia = 0.375
		sim.gravityScale = 1
		sim.qc = 1
		sim.px = float64(i * 3)
		sim.py = float64(i % 16)
		sim.cx, sim.cy = sim.px, sim.py
		sim.maxExtent = math.Sqrt2
		sim.sleepThreshold = 0.05

		states[i].dqc = 1

		shapes[i].verts = [4][2]float64{{-1, -1}, {1, -1}, {1, 1}, {-1, 1}}
	}

	dt := 1.0 / 60.0
	b.ReportAllocs()
	b.ResetTimer()
	for range b.N {
		stepF64(sims, states, shapes, dt, 4)
	}
}

// f64Polygon mirrors the Polygon fields that the collider reads.
type f64Polygon struct {
	verts  [8][2]float64
	norms  [8][2]float64
	radius float64
	count  int
}

// f64Manifold mirrors the manifold fields that the collider writes.
type f64Manifold struct {
	nx, ny float64
	anchor [2][2]float64
	sep    [2]float64
	ids    [2]uint16
	count  int
}

func makeBoxF64(h float64) f64Polygon {
	return f64Polygon{
		verts: [8][2]float64{{-h, -h}, {h, -h}, {h, h}, {-h, h}},
		norms: [8][2]float64{{0, -1}, {1, 0}, {0, 1}, {-1, 0}},
		count: 4,
	}
}

// segmentDistanceF64 mirrors SegmentDistance line by line and returns the
// fractions, the closest points and the squared distance.
func segmentDistanceF64(p1x, p1y, q1x, q1y, p2x, p2y, q2x, q2y float64) (f1, f2, c1x, c1y, c2x, c2y, distSq float64) {
	d1x, d1y := q1x-p1x, q1y-p1y
	d2x, d2y := q2x-p2x, q2y-p2y
	rx, ry := p1x-p2x, p1y-p2y
	dd1 := d1x*d1x + d1y*d1y
	dd2 := d2x*d2x + d2y*d2y
	rd1 := rx*d1x + ry*d1y
	rd2 := rx*d2x + ry*d2y

	clamp := func(v float64) float64 {
		return math.Max(0, math.Min(1, v))
	}

	if dd1 == 0 || dd2 == 0 {
		if dd1 != 0 {
			f1 = clamp(-rd1 / dd1)
		} else if dd2 != 0 {
			f2 = clamp(rd2 / dd2)
		}
	} else {
		d12 := d1x*d2x + d1y*d2y
		denominator := dd1*dd2 - d12*d12

		if denominator != 0 {
			f1 = clamp((d12*rd2 - rd1*dd2) / denominator)
		}

		f2 = (d12*f1 + rd2) / dd2

		if f2 < 0 {
			f2 = 0
			f1 = clamp(-rd1 / dd1)
		} else if f2 > 1 {
			f2 = 1
			f1 = clamp((d12 - rd1) / dd1)
		}
	}

	c1x, c1y = p1x+f1*d1x, p1y+f1*d1y
	c2x, c2y = p2x+f2*d2x, p2y+f2*d2y
	dx, dy := c2x-c1x, c2y-c1y
	distSq = dx*dx + dy*dy
	return
}

// findMaxSeparationF64 mirrors findMaxSeparation line by line.
func findMaxSeparationF64(poly1, poly2 *f64Polygon) (float64, int) {
	bestIndex := 0
	maxSeparation := math.Inf(-1)
	for i := range poly1.count {
		nx, ny := poly1.norms[i][0], poly1.norms[i][1]
		v1x, v1y := poly1.verts[i][0], poly1.verts[i][1]

		si := math.Inf(1)
		for j := range poly2.count {
			sij := nx*(poly2.verts[j][0]-v1x) + ny*(poly2.verts[j][1]-v1y)
			if sij < si {
				si = sij
			}
		}

		if si > maxSeparation {
			maxSeparation = si
			bestIndex = i
		}
	}
	return maxSeparation, bestIndex
}

// clipPolygonsF64 mirrors clipPolygons line by line.
func clipPolygonsF64(polyA, polyB *f64Polygon, edgeA, edgeB int, flip bool) f64Manifold {
	var manifold f64Manifold

	var poly1, poly2 *f64Polygon
	var i11, i12, i21, i22 int

	if flip {
		poly1, poly2 = polyB, polyA
		i11 = edgeB
		if edgeB+1 < polyB.count {
			i12 = edgeB + 1
		}
		i21 = edgeA
		if edgeA+1 < polyA.count {
			i22 = edgeA + 1
		}
	} else {
		poly1, poly2 = polyA, polyB
		i11 = edgeA
		if edgeA+1 < polyA.count {
			i12 = edgeA + 1
		}
		i21 = edgeB
		if edgeB+1 < polyB.count {
			i22 = edgeB + 1
		}
	}

	nx, ny := poly1.norms[i11][0], poly1.norms[i11][1]

	v11x, v11y := poly1.verts[i11][0], poly1.verts[i11][1]
	v12x, v12y := poly1.verts[i12][0], poly1.verts[i12][1]
	v21x, v21y := poly2.verts[i21][0], poly2.verts[i21][1]
	v22x, v22y := poly2.verts[i22][0], poly2.verts[i22][1]

	tx, ty := -ny, nx

	lower1 := 0.0
	upper1 := (v12x-v11x)*tx + (v12y-v11y)*ty

	upper2 := (v21x-v11x)*tx + (v21y-v11y)*ty
	lower2 := (v22x-v11x)*tx + (v22y-v11y)*ty

	if upper2 < lower1 || upper1 < lower2 {
		return manifold
	}

	vLx, vLy := v22x, v22y
	if lower2 < lower1 && upper2-lower2 > 0 {
		f := (lower1 - lower2) / (upper2 - lower2)
		vLx, vLy = v22x+f*(v21x-v22x), v22y+f*(v21y-v22y)
	}

	vUx, vUy := v21x, v21y
	if upper2 > upper1 && upper2-lower2 > 0 {
		f := (upper1 - lower2) / (upper2 - lower2)
		vUx, vUy = v22x+f*(v21x-v22x), v22y+f*(v21y-v22y)
	}

	separationLower := (vLx-v11x)*nx + (vLy-v11y)*ny
	separationUpper := (vUx-v11x)*nx + (vUy-v11y)*ny

	r1 := poly1.radius
	r2 := poly2.radius

	vLx += 0.5 * (r1 - r2 - separationLower) * nx
	vLy += 0.5 * (r1 - r2 - separationLower) * ny
	vUx += 0.5 * (r1 - r2 - separationUpper) * nx
	vUy += 0.5 * (r1 - r2 - separationUpper) * ny

	radius := r1 + r2

	if !flip {
		manifold.nx, manifold.ny = nx, ny
		manifold.anchor[0] = [2]float64{vLx, vLy}
		manifold.sep[0] = separationLower - radius
		manifold.ids[0] = makeId(i11, i22)
		manifold.anchor[1] = [2]float64{vUx, vUy}
		manifold.sep[1] = separationUpper - radius
		manifold.ids[1] = makeId(i12, i21)
		manifold.count = 2
	} else {
		manifold.nx, manifold.ny = -nx, -ny
		manifold.anchor[0] = [2]float64{vUx, vUy}
		manifold.sep[0] = separationUpper - radius
		manifold.ids[0] = makeId(i21, i12)
		manifold.anchor[1] = [2]float64{vLx, vLy}
		manifold.sep[1] = separationLower - radius
		manifold.ids[1] = makeId(i22, i11)
		manifold.count = 2
	}

	return manifold
}

// collidePolygonsF64 mirrors CollidePolygons line by line, with the same
// origin shift and the same branch structure.
func collidePolygonsF64(polygonA *f64Polygon, aPx, aPy, aQc, aQs float64, polygonB *f64Polygon, bPx, bPy, bQc, bQs float64) f64Manifold {
	const linearSlopF64 = 0.005
	const speculativeF64 = 4 * linearSlopF64

	originX, originY := polygonA.verts[0][0], polygonA.verts[0][1]

	// Shift polygon A to the origin.
	sfPx := aPx + aQc*originX - aQs*originY
	sfPy := aPy + aQs*originX + aQc*originY

	// xf = InvMulTransforms(sfA, xfB)
	xfQc := aQc*bQc + aQs*bQs
	xfQs := aQc*bQs - aQs*bQc
	dxp, dyp := bPx-sfPx, bPy-sfPy
	xfPx := aQc*dxp + aQs*dyp
	xfPy := -aQs*dxp + aQc*dyp

	var localPolyA f64Polygon
	localPolyA.count = polygonA.count
	localPolyA.radius = polygonA.radius
	localPolyA.norms[0] = polygonA.norms[0]
	for i := 1; i < localPolyA.count; i++ {
		localPolyA.verts[i] = [2]float64{polygonA.verts[i][0] - originX, polygonA.verts[i][1] - originY}
		localPolyA.norms[i] = polygonA.norms[i]
	}

	var localPolyB f64Polygon
	localPolyB.count = polygonB.count
	localPolyB.radius = polygonB.radius
	for i := range localPolyB.count {
		vx, vy := polygonB.verts[i][0], polygonB.verts[i][1]
		localPolyB.verts[i] = [2]float64{xfPx + xfQc*vx - xfQs*vy, xfPy + xfQs*vx + xfQc*vy}
		nx, ny := polygonB.norms[i][0], polygonB.norms[i][1]
		localPolyB.norms[i] = [2]float64{xfQc*nx - xfQs*ny, xfQs*nx + xfQc*ny}
	}

	separationA, edgeA := findMaxSeparationF64(&localPolyA, &localPolyB)
	separationB, edgeB := findMaxSeparationF64(&localPolyB, &localPolyA)

	radius := localPolyA.radius + localPolyB.radius

	if separationA > speculativeF64+radius || separationB > speculativeF64+radius {
		return f64Manifold{}
	}

	var flip bool
	if separationA >= separationB {
		flip = false
		sdx, sdy := localPolyA.norms[edgeA][0], localPolyA.norms[edgeA][1]

		edgeB = 0
		minDot := math.Inf(1)
		for i := range localPolyB.count {
			dot := sdx*localPolyB.norms[i][0] + sdy*localPolyB.norms[i][1]
			if dot < minDot {
				minDot = dot
				edgeB = i
			}
		}
	} else {
		flip = true
		sdx, sdy := localPolyB.norms[edgeB][0], localPolyB.norms[edgeB][1]

		edgeA = 0
		minDot := math.Inf(1)
		for i := range localPolyA.count {
			dot := sdx*localPolyA.norms[i][0] + sdy*localPolyA.norms[i][1]
			if dot < minDot {
				minDot = dot
				edgeA = i
			}
		}
	}

	var manifold f64Manifold

	const slopBias = 0.1 * linearSlopF64
	if separationA > slopBias || separationB > slopBias {
		// The edges are disjoint.
		i11 := edgeA
		i12 := 0
		if edgeA+1 < localPolyA.count {
			i12 = edgeA + 1
		}
		i21 := edgeB
		i22 := 0
		if edgeB+1 < localPolyB.count {
			i22 = edgeB + 1
		}

		v11 := localPolyA.verts[i11]
		v12 := localPolyA.verts[i12]
		v21 := localPolyB.verts[i21]
		v22 := localPolyB.verts[i22]

		f1, f2, c1x, c1y, c2x, c2y, distSq := segmentDistanceF64(
			v11[0], v11[1], v12[0], v12[1], v21[0], v21[1], v22[0], v22[1])
		_, _, _, _ = c1x, c1y, c2x, c2y

		distance := math.Sqrt(distSq)
		separation := distance - radius

		if distance-radius > speculativeF64 {
			return manifold
		}

		manifold = clipPolygonsF64(&localPolyA, &localPolyB, edgeA, edgeB, flip)

		minSeparation := math.Inf(1)
		for i := range manifold.count {
			minSeparation = math.Min(minSeparation, manifold.sep[i])
		}

		if minSeparation > separation+slopBias {
			vertexVertex := func(ax, ay, bx, by float64, id uint16) {
				invDistance := 1 / distance
				nx := (bx - ax) * invDistance
				ny := (by - ay) * invDistance

				c1x := ax + localPolyA.radius*nx
				c1y := ay + localPolyA.radius*ny
				c2x := bx - localPolyB.radius*nx
				c2y := by - localPolyB.radius*ny

				manifold.nx, manifold.ny = nx, ny
				manifold.anchor[0] = [2]float64{0.5 * (c1x + c2x), 0.5 * (c1y + c2y)}
				manifold.sep[0] = distance - radius
				manifold.ids[0] = id
				manifold.count = 1
			}
			if f1 == 0 && f2 == 0 {
				vertexVertex(v11[0], v11[1], v21[0], v21[1], makeId(i11, i21))
			} else if f1 == 0 && f2 == 1 {
				vertexVertex(v11[0], v11[1], v22[0], v22[1], makeId(i11, i22))
			} else if f1 == 1 && f2 == 0 {
				vertexVertex(v12[0], v12[1], v21[0], v21[1], makeId(i12, i21))
			} else if f1 == 1 && f2 == 1 {
				vertexVertex(v12[0], v12[1], v22[0], v22[1], makeId(i12, i22))
			}
		}
	} else {
		// The polygons overlap.
		manifold = clipPolygonsF64(&localPolyA, &localPolyB, edgeA, edgeB, flip)
	}

	// Convert the manifold to world space.
	if manifold.count > 0 {
		nx, ny := manifold.nx, manifold.ny
		manifold.nx = aQc*nx - aQs*ny
		manifold.ny = aQs*nx + aQc*ny
		for i := range manifold.count {
			ax := manifold.anchor[i][0] + originX
			ay := manifold.anchor[i][1] + originY
			manifold.anchor[i] = [2]float64{aQc*ax - aQs*ay, aQs*ax + aQc*ay}
		}
	}

	return manifold
}

// BenchmarkCollidePolygonsQ measures the dominant contact pattern: two unit
// boxes on the overlap branch of the clip path.
func BenchmarkCollidePolygonsQ(b *testing.B) {
	boxA := MakeSquare(fixed.One())
	boxB := MakeSquare(fixed.One())
	xfA := TransformIdentity()
	xfB := Transform{P: Vec2{Y: fixed.MustParse("1.5")}, Q: RotIdentity()}

	var sink int
	b.ResetTimer()
	for range b.N {
		m := CollidePolygons(&boxA, xfA, &boxB, xfB)
		sink += m.PointCount
	}
	if sink == 0 {
		b.Fatal("the boxes did not collide")
	}
}

// BenchmarkCollidePolygonsF64 runs the float64 mirror over the same boxes.
func BenchmarkCollidePolygonsF64(b *testing.B) {
	boxA := makeBoxF64(1)
	boxB := makeBoxF64(1)

	var sink int
	b.ResetTimer()
	for range b.N {
		m := collidePolygonsF64(&boxA, 0, 0, 1, 0, &boxB, 0, 1.5, 1, 0)
		sink += m.count
	}
	if sink == 0 {
		b.Fatal("the boxes did not collide")
	}
}
