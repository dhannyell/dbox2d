package dbox2d

import (
	"math"
	"testing"

	"github.com/dhannyell/fixed"
)

// This file measures the scalar penalty of Q32.32 against float64 on the
// velocity integration, and the allocation contract of Step. A micro number
// diagnoses; only a composite benchmark decides an optimization.

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

// f64BodySim mirrors the bodySim fields that the integration reads.
type f64BodySim struct {
	invMass        float64
	invInertia     float64
	linearDamping  float64
	angularDamping float64
	gravityScale   float64
	forceX, forceY float64
	torque         float64

	isSpeedCapped     bool
	allowFastRotation bool
}

// f64BodyState mirrors the bodyState fields that the integration writes.
type f64BodyState struct {
	vx, vy float64
	w      float64
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
