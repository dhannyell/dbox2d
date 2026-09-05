// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/main.cpp (UpdateUI) of Box2D v3.1.1. microui has no
// tabs, so the reference's "Controls" and "Samples" tabs become two
// collapsible headers in the same window.

package app

import (
	"strings"

	"github.com/dhannyell/dbox2d/samples"
	"github.com/dhannyell/dbox2d/samples/internal/draw"
	"github.com/dhannyell/dbox2d/samples/internal/microui"
	"github.com/dhannyell/dbox2d/samples/internal/render"
)

// checkbox is one Settings bool the Controls header exposes, in the
// reference's order.
type checkbox struct {
	label string
	value *bool
}

func (a *App) checkboxes() []checkbox {
	s := &a.ctx.Settings
	return []checkbox{
		{"Sleep", &s.EnableSleep},
		{"Warm Starting", &s.EnableWarmStarting},
		{"Continuous", &s.EnableContinuous},
		{"Shapes", &s.DrawShapes},
		{"Joints", &s.DrawJoints},
		{"Joint Extras", &s.DrawJointExtras},
		{"Bounds", &s.DrawBounds},
		{"Contact Points", &s.DrawContactPoints},
		{"Contact Normals", &s.DrawContactNormals},
		{"Contact Impulses", &s.DrawContactImpulses},
		{"Contact Features", &s.DrawContactFeatures},
		{"Friction Impulses", &s.DrawFrictionImpulses},
		{"Mass", &s.DrawMass},
		{"Body Names", &s.DrawBodyNames},
		{"Graph Colors", &s.DrawGraphColors},
		{"Islands", &s.DrawIslands},
		{"Counters", &s.DrawCounters},
		{"Profile", &s.DrawProfile},
	}
}

func (a *App) buildToolsWindow(entries []samples.Entry) {
	rect := microui.NewRect(a.ctx.Camera.Width-toolsWidth-10, 10, toolsWidth, a.ctx.Camera.Height-20)
	if a.mu.BeginWindowEx("Tools", rect, microui.MU_OPT_NORESIZE|microui.MU_OPT_NOCLOSE) == 0 {
		return
	}
	defer a.mu.EndWindow()

	if a.mu.Header("Controls") {
		a.buildControls()
	}
	if a.mu.Header("Samples") {
		a.buildSampleTree(entries)
	}
}

func (a *App) buildControls() {
	s := &a.ctx.Settings

	subSteps := float32(s.SubStepCount)
	a.mu.LayoutRow(2, []int{80, -1}, 0)
	a.mu.Label("Sub-steps")
	a.mu.SliderEx(&subSteps, 1, 32, 1, "%.0f", microui.MU_OPT_ALIGNCENTER)
	s.SubStepCount = int(subSteps + 0.5)

	hertz := float32(s.Hertz)
	a.mu.LayoutRow(2, []int{80, -1}, 0)
	a.mu.Label("Hertz")
	a.mu.SliderEx(&hertz, 5, 240, 1, "%.0f hz", microui.MU_OPT_ALIGNCENTER)
	s.Hertz = float64(hertz)

	a.mu.LayoutRow(1, []int{-1}, 0)
	a.mu.Label("Workers: 1")

	for _, cb := range a.checkboxes() {
		a.mu.LayoutRow(1, []int{-1}, 0)
		a.mu.Checkbox(cb.label, cb.value)
	}

	a.mu.LayoutRow(1, []int{-1}, 0)
	if a.mu.Button("Pause (P)") {
		s.Pause = !s.Pause
	}
	a.mu.LayoutRow(1, []int{-1}, 0)
	if a.mu.Button("Single Step (O)") {
		s.SingleStep = !s.SingleStep
	}
	a.mu.LayoutRow(1, []int{-1}, 0)
	if a.mu.Button("Dump Mem Stats") {
		a.dumpMemStats()
	}
	a.mu.LayoutRow(1, []int{-1}, 0)
	if a.mu.Button("Reset Profile") {
		if rp, ok := a.sample.(interface{ ResetProfile() }); ok {
			rp.ResetProfile()
		}
	}
	a.mu.LayoutRow(1, []int{-1}, 0)
	if a.mu.Button("Restart (R)") {
		s.Restart = true
	}
}

func (a *App) dumpMemStats() {
	worldId, ok := worldOf(a.sample)
	if !ok {
		return
	}
	var b strings.Builder
	worldId.DumpMemoryStats(&b)
	a.memStatsText = b.String()
	a.showMemWindow = true
}

func (a *App) buildMemoryWindow() {
	if !a.showMemWindow {
		return
	}
	rect := microui.NewRect(a.ctx.Camera.Width/2-200, a.ctx.Camera.Height/2-150, 400, 300)
	if a.mu.BeginWindowEx("Memory", rect, 0) == 0 {
		return
	}
	defer a.mu.EndWindow()

	a.mu.LayoutRow(1, []int{-1}, 0)
	for line := range strings.SplitSeq(a.memStatsText, "\n") {
		a.mu.Text(line)
	}
	a.mu.LayoutRow(1, []int{-1}, 0)
	if a.mu.Button("Close") {
		a.showMemWindow = false
	}
}

func (a *App) buildSampleTree(entries []samples.Entry) {
	i := 0
	for i < len(entries) {
		category := entries[i].Category
		a.mu.LayoutRow(1, []int{-1}, 0)
		open := a.mu.BeginTreeNode(category)
		for i < len(entries) && entries[i].Category == category {
			if open {
				idx := i
				a.mu.LayoutRow(1, []int{-1}, 0)
				if a.mu.Button(entries[idx].Name) {
					a.selection = idx
				}
			}
			i++
		}
		if open {
			a.mu.EndTreeNode()
		}
	}
}

// collectUICommands drains the microui command list built this frame into
// the render package's host-agnostic form.
func (a *App) collectUICommands() []render.UICommand {
	cmds := a.uiCmds[:0]
	var cmd *microui.Command
	for a.mu.NextCommand(&cmd) {
		switch cmd.Type {
		case microui.MU_COMMAND_CLIP:
			r := cmd.Clip.Rect
			cmds = append(cmds, render.UICommand{Kind: render.UIClip, Rect: [4]int{r.X, r.Y, r.W, r.H}})
		case microui.MU_COMMAND_RECT:
			r := cmd.Rect.Rect
			cmds = append(cmds, render.UICommand{Kind: render.UIRect, Rect: [4]int{r.X, r.Y, r.W, r.H}, Color: muColor(cmd.Rect.Color)})
		case microui.MU_COMMAND_TEXT:
			cmds = append(cmds, render.UICommand{Kind: render.UIText, X: cmd.Text.Pos.X, Y: cmd.Text.Pos.Y, Text: cmd.Text.Str, Color: muColor(cmd.Text.Color)})
		case microui.MU_COMMAND_ICON:
			r := cmd.Icon.Rect
			cmds = append(cmds, render.UICommand{Kind: render.UIIcon, Rect: [4]int{r.X, r.Y, r.W, r.H}, Icon: cmd.Icon.Id, Color: muColor(cmd.Icon.Color)})
		}
	}
	a.uiCmds = cmds
	return cmds
}

func muColor(c microui.Color) draw.RGBA8 {
	return draw.RGBA8{R: c.R, G: c.G, B: c.B, A: c.A}
}
