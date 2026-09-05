// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/sample.h and samples/sample.cpp of Box2D v3.1.1

package samples

import (
	"fmt"

	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/fixed"
)

// SampleContext is the shared state a host hands to every sample.
type SampleContext struct {
	Camera   Camera
	Settings Settings
	// Draw is what the world draws into; the host renders it.
	Draw dbox2d.DebugDraw
	// TextLine receives one line of overlay text per call, at the pixel
	// position the reference's DrawString(5, m_textLine, ...) would use. A
	// headless host leaves it nil.
	TextLine func(x, y int, line string)
}

// NewSampleContext returns a context with the reference's startup defaults.
func NewSampleContext() *SampleContext {
	return &SampleContext{
		Camera:   NewCamera(),
		Settings: DefaultSettings(),
		Draw:     dbox2d.DefaultDebugDraw(),
	}
}

// Sample is one scene. Base supplies the defaults; a scene embeds Base and
// overrides what it needs.
type Sample interface {
	Step()
	UpdateGui()
	Keyboard(key Key)
	MouseDown(p dbox2d.Vec2, button MouseButton, mod Modifier)
	MouseUp(p dbox2d.Vec2, button MouseButton)
	MouseMove(p dbox2d.Vec2)
	Destroy()
}

// Base holds the state every sample shares: the world, the mouse joint and
// the profile.
type Base struct {
	Context      *SampleContext
	WorldId      dbox2d.WorldId
	GroundBodyId dbox2d.BodyId
	MouseJointId dbox2d.JointId
	StepCount    int
	MaxProfile   dbox2d.Profile
	TotalProfile dbox2d.Profile

	textLine      int
	textIncrement int
}

// NewBase builds the world and returns the shared state for a new sample.
// The reference also seeds a task scheduler and a random generator; this
// port steps on one worker and leaves randomness to the scene.
func NewBase(ctx *SampleContext) Base {
	b := Base{
		Context:       ctx,
		textLine:      30,
		textIncrement: 22,
	}
	b.CreateWorld()
	return b
}

// CreateWorld destroys any live world and creates a fresh one from the
// current settings. It corresponds to Sample::CreateWorld.
func (b *Base) CreateWorld() {
	if !b.WorldId.IsNull() {
		dbox2d.DestroyWorld(b.WorldId)
		b.WorldId = dbox2d.WorldId{}
	}
	worldDef := dbox2d.DefaultWorldDef()
	worldDef.EnableSleep = b.Context.Settings.EnableSleep
	b.WorldId = dbox2d.CreateWorld(&worldDef)
}

// Destroy tears the world down. Destroying the world also destroys the
// bomb, the mouse joint and every body created in it.
func (b *Base) Destroy() {
	dbox2d.DestroyWorld(b.WorldId)
}

// DrawTitle draws the scene title on its own reserved line.
func (b *Base) DrawTitle(title string) {
	if b.Context.TextLine != nil {
		b.Context.TextLine(5, 5, title)
	}
	b.textLine = 26
}

// DrawTextLine formats one overlay line and advances the cursor.
func (b *Base) DrawTextLine(format string, args ...any) {
	if b.Context.TextLine != nil {
		b.Context.TextLine(5, b.textLine, fmt.Sprintf(format, args...))
	}
	b.textLine += b.textIncrement
}

// World returns the sample's world id. Sample hides it behind Step and the
// input methods; a host that needs it (memory stats, checksums) type-asserts
// for this method instead of widening the interface.
func (b *Base) World() dbox2d.WorldId { return b.WorldId }

// ResetProfile clears the accumulated profile and the step counter.
func (b *Base) ResetProfile() {
	b.TotalProfile = dbox2d.Profile{}
	b.MaxProfile = dbox2d.Profile{}
	b.StepCount = 0
}

// UpdateGui does nothing by default; a scene overrides it to add controls.
func (b *Base) UpdateGui() {}

// Keyboard does nothing by default; a scene overrides it to react to keys.
func (b *Base) Keyboard(Key) {}

type queryContext struct {
	point  dbox2d.Vec2
	bodyId dbox2d.BodyId
}

// MouseDown starts dragging the first dynamic body under the point, using a
// mouse joint anchored to a throwaway ground body.
func (b *Base) MouseDown(p dbox2d.Vec2, button MouseButton, mod Modifier) {
	if !b.MouseJointId.IsNull() {
		return
	}
	if button != MouseButtonLeft {
		return
	}

	d := dbox2d.Vec2{X: fixed.Q32FromRatio(1, 1000), Y: fixed.Q32FromRatio(1, 1000)}
	box := dbox2d.AABB{
		LowerBound: dbox2d.Vec2{X: p.X.Sub(d.X), Y: p.Y.Sub(d.Y)},
		UpperBound: dbox2d.Vec2{X: p.X.Add(d.X), Y: p.Y.Add(d.Y)},
	}

	qc := queryContext{point: p}
	b.WorldId.OverlapAABB(box, dbox2d.DefaultQueryFilter(), func(shapeId dbox2d.ShapeId) bool {
		bodyId := shapeId.GetBody()
		if bodyId.GetType() != dbox2d.DynamicBody {
			return true
		}
		if shapeId.TestPoint(qc.point) {
			qc.bodyId = bodyId
			return false
		}
		return true
	})

	if qc.bodyId.IsNull() {
		return
	}

	bodyDef := dbox2d.DefaultBodyDef()
	b.GroundBodyId = dbox2d.CreateBody(b.WorldId, &bodyDef)

	mouseDef := dbox2d.DefaultMouseJointDef()
	mouseDef.BodyIdA = b.GroundBodyId
	mouseDef.BodyIdB = qc.bodyId
	mouseDef.Target = p
	mouseDef.Hertz = fixed.Q32FromInt(10)
	mouseDef.DampingRatio = fixed.Q32FromRatio(7, 10)
	gravityLength, _ := dbox2d.GetLengthAndNormalize(b.WorldId.GetGravity())
	mouseDef.MaxForce = fixed.Q32FromInt(1000).Mul(qc.bodyId.GetMass()).Mul(gravityLength)
	b.MouseJointId = dbox2d.CreateMouseJoint(b.WorldId, &mouseDef)

	qc.bodyId.SetAwake(true)
}

// MouseUp releases the mouse joint started by MouseDown.
func (b *Base) MouseUp(p dbox2d.Vec2, button MouseButton) {
	if !b.MouseJointId.IsNull() && !b.MouseJointId.IsValid() {
		// The world or attached body was destroyed.
		b.MouseJointId = dbox2d.JointId{}
	}

	if !b.MouseJointId.IsNull() && button == MouseButtonLeft {
		dbox2d.DestroyJoint(b.MouseJointId)
		b.MouseJointId = dbox2d.JointId{}

		dbox2d.DestroyBody(b.GroundBodyId)
		b.GroundBodyId = dbox2d.BodyId{}
	}
}

// MouseMove drags the mouse joint target to the new point.
func (b *Base) MouseMove(p dbox2d.Vec2) {
	if !b.MouseJointId.IsNull() && !b.MouseJointId.IsValid() {
		// The world or attached body was destroyed.
		b.MouseJointId = dbox2d.JointId{}
	}

	if !b.MouseJointId.IsNull() {
		b.MouseJointId.SetTarget(p)
		bodyIdB := b.MouseJointId.GetBodyB()
		bodyIdB.SetAwake(true)
	}
}

// Step advances the world by one frame, then draws it and records the
// profile. It corresponds to Sample::Step.
func (b *Base) Step() {
	s := &b.Context.Settings
	var timeStep dbox2d.Q
	if s.Hertz > 0 {
		timeStep = fixed.Q32One().Div(fixed.Q32FromInt(int(s.Hertz)))
	}

	if s.Pause {
		if s.SingleStep {
			s.SingleStep = false
		} else {
			timeStep = fixed.Q32Zero()
		}
		b.DrawTextLine("****PAUSED****")
	}

	draw := &b.Context.Draw
	draw.DrawingBounds = b.Context.Camera.GetViewBounds()
	draw.UseDrawingBounds = s.UseCameraBounds

	draw.DrawShapes = s.DrawShapes
	draw.DrawJoints = s.DrawJoints
	draw.DrawJointExtras = s.DrawJointExtras
	draw.DrawBounds = s.DrawBounds
	draw.DrawMass = s.DrawMass
	draw.DrawBodyNames = s.DrawBodyNames
	draw.DrawContacts = s.DrawContactPoints
	draw.DrawGraphColors = s.DrawGraphColors
	draw.DrawContactNormals = s.DrawContactNormals
	draw.DrawContactImpulses = s.DrawContactImpulses
	draw.DrawContactFeatures = s.DrawContactFeatures
	draw.DrawFrictionImpulses = s.DrawFrictionImpulses
	draw.DrawIslands = s.DrawIslands

	b.WorldId.EnableSleeping(s.EnableSleep)
	b.WorldId.EnableWarmStarting(s.EnableWarmStarting)
	b.WorldId.EnableContinuous(s.EnableContinuous)

	b.WorldId.Step(timeStep, s.SubStepCount)
	b.WorldId.Draw(draw)

	if timeStep.Greater(fixed.Q32Zero()) {
		b.StepCount++
	}

	if s.DrawCounters {
		c := b.WorldId.GetCounters()
		b.DrawTextLine("bodies/shapes/contacts/joints = %d/%d/%d/%d", c.BodyCount, c.ShapeCount, c.ContactCount, c.JointCount)
		b.DrawTextLine("islands = %d", c.IslandCount)
		b.DrawTextLine("tree height static/movable = %d/%d", c.StaticTreeHeight, c.TreeHeight)

		total := 0
		colors := "colors: "
		for _, n := range c.ColorCounts {
			colors += fmt.Sprintf("%d/", n)
			total += n
		}
		b.DrawTextLine("%s[%d]", colors, total)
		b.DrawTextLine("stack allocator size = %d K", c.StackUsed/1024)
	}

	p := b.WorldId.GetProfile()
	b.MaxProfile = maxProfile(b.MaxProfile, p)
	b.TotalProfile = addProfile(b.TotalProfile, p)

	if s.DrawProfile {
		var ave dbox2d.Profile
		if b.StepCount > 0 {
			ave = scaleProfile(b.TotalProfile, 1/float64(b.StepCount))
		}
		m := b.MaxProfile
		b.DrawTextLine("step [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.Step, ave.Step, m.Step)
		b.DrawTextLine("pairs [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.Pairs, ave.Pairs, m.Pairs)
		b.DrawTextLine("collide [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.Collide, ave.Collide, m.Collide)
		b.DrawTextLine("solve [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.Solve, ave.Solve, m.Solve)
		b.DrawTextLine("> merge islands [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.MergeIslands, ave.MergeIslands, m.MergeIslands)
		b.DrawTextLine("> prepare tasks [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.PrepareStages, ave.PrepareStages, m.PrepareStages)
		b.DrawTextLine("> solve constraints [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.SolveConstraints, ave.SolveConstraints, m.SolveConstraints)
		b.DrawTextLine(">> prepare constraints [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.PrepareConstraints, ave.PrepareConstraints, m.PrepareConstraints)
		b.DrawTextLine(">> integrate velocities [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.IntegrateVelocities, ave.IntegrateVelocities, m.IntegrateVelocities)
		b.DrawTextLine(">> warm start [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.WarmStart, ave.WarmStart, m.WarmStart)
		b.DrawTextLine(">> solve impulses [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.SolveImpulses, ave.SolveImpulses, m.SolveImpulses)
		b.DrawTextLine(">> integrate positions [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.IntegratePositions, ave.IntegratePositions, m.IntegratePositions)
		b.DrawTextLine(">> relax impulses [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.RelaxImpulses, ave.RelaxImpulses, m.RelaxImpulses)
		b.DrawTextLine(">> apply restitution [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.ApplyRestitution, ave.ApplyRestitution, m.ApplyRestitution)
		b.DrawTextLine(">> store impulses [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.StoreImpulses, ave.StoreImpulses, m.StoreImpulses)
		b.DrawTextLine(">> split islands [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.SplitIslands, ave.SplitIslands, m.SplitIslands)
		b.DrawTextLine("> update transforms [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.Transforms, ave.Transforms, m.Transforms)
		b.DrawTextLine("> hit events [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.HitEvents, ave.HitEvents, m.HitEvents)
		b.DrawTextLine("> refit BVH [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.Refit, ave.Refit, m.Refit)
		b.DrawTextLine("> sleep islands [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.SleepIslands, ave.SleepIslands, m.SleepIslands)
		b.DrawTextLine("> bullets [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.Bullets, ave.Bullets, m.Bullets)
		b.DrawTextLine("sensors [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.Sensors, ave.Sensors, m.Sensors)
	}
}

func maxProfile(a, b dbox2d.Profile) dbox2d.Profile {
	return dbox2d.Profile{
		Step:                max(a.Step, b.Step),
		Pairs:               max(a.Pairs, b.Pairs),
		Collide:             max(a.Collide, b.Collide),
		Solve:               max(a.Solve, b.Solve),
		MergeIslands:        max(a.MergeIslands, b.MergeIslands),
		PrepareStages:       max(a.PrepareStages, b.PrepareStages),
		SolveConstraints:    max(a.SolveConstraints, b.SolveConstraints),
		PrepareConstraints:  max(a.PrepareConstraints, b.PrepareConstraints),
		IntegrateVelocities: max(a.IntegrateVelocities, b.IntegrateVelocities),
		WarmStart:           max(a.WarmStart, b.WarmStart),
		SolveImpulses:       max(a.SolveImpulses, b.SolveImpulses),
		IntegratePositions:  max(a.IntegratePositions, b.IntegratePositions),
		RelaxImpulses:       max(a.RelaxImpulses, b.RelaxImpulses),
		ApplyRestitution:    max(a.ApplyRestitution, b.ApplyRestitution),
		StoreImpulses:       max(a.StoreImpulses, b.StoreImpulses),
		SplitIslands:        max(a.SplitIslands, b.SplitIslands),
		Transforms:          max(a.Transforms, b.Transforms),
		HitEvents:           max(a.HitEvents, b.HitEvents),
		Refit:               max(a.Refit, b.Refit),
		Bullets:             max(a.Bullets, b.Bullets),
		SleepIslands:        max(a.SleepIslands, b.SleepIslands),
		Sensors:             max(a.Sensors, b.Sensors),
	}
}

func addProfile(a, b dbox2d.Profile) dbox2d.Profile {
	return dbox2d.Profile{
		Step:                a.Step + b.Step,
		Pairs:               a.Pairs + b.Pairs,
		Collide:             a.Collide + b.Collide,
		Solve:               a.Solve + b.Solve,
		MergeIslands:        a.MergeIslands + b.MergeIslands,
		PrepareStages:       a.PrepareStages + b.PrepareStages,
		SolveConstraints:    a.SolveConstraints + b.SolveConstraints,
		PrepareConstraints:  a.PrepareConstraints + b.PrepareConstraints,
		IntegrateVelocities: a.IntegrateVelocities + b.IntegrateVelocities,
		WarmStart:           a.WarmStart + b.WarmStart,
		SolveImpulses:       a.SolveImpulses + b.SolveImpulses,
		IntegratePositions:  a.IntegratePositions + b.IntegratePositions,
		RelaxImpulses:       a.RelaxImpulses + b.RelaxImpulses,
		ApplyRestitution:    a.ApplyRestitution + b.ApplyRestitution,
		StoreImpulses:       a.StoreImpulses + b.StoreImpulses,
		SplitIslands:        a.SplitIslands + b.SplitIslands,
		Transforms:          a.Transforms + b.Transforms,
		HitEvents:           a.HitEvents + b.HitEvents,
		Refit:               a.Refit + b.Refit,
		Bullets:             a.Bullets + b.Bullets,
		SleepIslands:        a.SleepIslands + b.SleepIslands,
		Sensors:             a.Sensors + b.Sensors,
	}
}

func scaleProfile(a dbox2d.Profile, scale float64) dbox2d.Profile {
	return dbox2d.Profile{
		Step:                scale * a.Step,
		Pairs:               scale * a.Pairs,
		Collide:             scale * a.Collide,
		Solve:               scale * a.Solve,
		MergeIslands:        scale * a.MergeIslands,
		PrepareStages:       scale * a.PrepareStages,
		SolveConstraints:    scale * a.SolveConstraints,
		PrepareConstraints:  scale * a.PrepareConstraints,
		IntegrateVelocities: scale * a.IntegrateVelocities,
		WarmStart:           scale * a.WarmStart,
		SolveImpulses:       scale * a.SolveImpulses,
		IntegratePositions:  scale * a.IntegratePositions,
		RelaxImpulses:       scale * a.RelaxImpulses,
		ApplyRestitution:    scale * a.ApplyRestitution,
		StoreImpulses:       scale * a.StoreImpulses,
		SplitIslands:        scale * a.SplitIslands,
		Transforms:          scale * a.Transforms,
		HitEvents:           scale * a.HitEvents,
		Refit:               scale * a.Refit,
		Bullets:             scale * a.Bullets,
		SleepIslands:        scale * a.SleepIslands,
		Sensors:             scale * a.Sensors,
	}
}
