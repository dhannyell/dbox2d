package dbox2d

import "github.com/dhannyell/fixed"

// stepContext carries the per-step data that the solver stages share. It
// corresponds to b2StepContext in src/solver.h.
type stepContext struct {
	world *world

	// dt is the time step of the full step.
	dt Q

	// invDt is the inverse of dt, or zero when dt is zero.
	invDt Q

	// h is the sub-step time: dt divided by the sub-step count.
	h Q

	// invH is the inverse of h, or zero when h is zero.
	invH Q

	subStepCount int

	// Stiffer for static contacts to avoid bodies getting pushed through
	// the ground.
	contactSoftness softness
	staticSoftness  softness

	// Deferred: the constraint arrays and the worker fields of the
	// reference.

	maxLinearVelocity Q

	enableWarmStarting bool

	// The body arrays of the awake set.
	sims   []bodySim
	states []bodyState
}

// Step advances the simulation by timeStep, split into subStepCount
// sub-steps. The reference recommends a fixed time step and 4 sub-steps.
// It corresponds to b2World_Step in src/world.c.
func Step(worldId WorldId, timeStep Q, subStepCount int) {
	if !IsValidQ(timeStep) {
		panic("dbox2d: the time step is not valid")
	}
	if subStepCount <= 0 {
		panic("dbox2d: the sub-step count is not positive")
	}

	w := getWorldFromId(worldId)
	if w.locked {
		panic("dbox2d: the world is locked")
	}

	// Deferred: the event arrays of the reference clear here.

	zero := fixed.Q32Zero()
	if timeStep.Eq(zero) {
		// Deferred: the end event buffers of the reference swap here.
		return
	}

	w.locked = true

	// Deferred: the broad-phase pair update and the contact update of the
	// reference run here.

	context := stepContext{}
	context.world = w
	context.dt = timeStep
	context.subStepCount = max(1, subStepCount)

	if zero.Less(timeStep) {
		context.invDt = fixed.Q32One().Div(timeStep)
		context.h = timeStep.Div(fixed.Q32FromInt(context.subStepCount))
		context.invH = fixed.Q32FromInt(context.subStepCount).Mul(context.invDt)
	}

	w.invH = context.invH

	// Deferred: the contact softness, the static softness and the contact
	// speed of the reference need the soft constraint setup.

	context.maxLinearVelocity = w.maxLinearSpeed
	context.enableWarmStarting = w.enableWarmStarting

	if zero.Less(context.dt) {
		solve(w, &context)
	}

	// Deferred: the sensor overlap update and the end event buffer swap of
	// the reference run here.

	w.locked = false
}
