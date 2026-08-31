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
