// Package app is the loop of the reference's main.cpp without a window: it
// owns the sample context, the current sample, the microui state and the
// batches a host renders. A host drives it with dt and forwards input; app
// never reads wall-clock time or a GPU binding.
package app

import (
	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/dbox2d/samples"
	"github.com/dhannyell/dbox2d/samples/internal/draw"
	"github.com/dhannyell/dbox2d/samples/internal/microui"
	"github.com/dhannyell/dbox2d/samples/internal/render"
)

// TextMeasurer is what microui needs to lay out controls. draw.Atlas
// satisfies it.
type TextMeasurer interface {
	TextWidth(s string) int
	TextHeight() int
}

// toolsWidth is the reference's menuWidth (main.cpp UpdateUI).
const toolsWidth = 180

// App is the loop of main.cpp without a window: it owns the context, the
// sample, the UI state and the batches.
type App struct {
	ctx      *samples.SampleContext
	measurer TextMeasurer
	drawer   *draw.Draw
	mu       *microui.Context

	sample    samples.Sample
	selection int
	showUI    bool

	rightMouseDown bool
	rightMouseAt   samples.Vec2f

	showMemWindow bool
	memStatsText  string

	uiCmds []render.UICommand
}

// New builds an App on the sample named "Tumbler" in category "Benchmark"
// (falling back to the first registered entry), measuring UI text with
// measurer.
func New(measurer TextMeasurer) *App {
	ctx := samples.NewSampleContext()
	d := draw.New(&ctx.Camera)
	ctx.Draw = d.DebugDraw()
	ctx.TextLine = d.DrawString

	entries := samples.Entries()
	start := 0
	for i, e := range entries {
		if e.Category == "Benchmark" && e.Name == "Tumbler" {
			start = i
			break
		}
	}
	ctx.Settings.SampleIndex = start

	mu := microui.NewContext()
	mu.TextWidth = func(_ microui.Font, s string) int { return measurer.TextWidth(s) }
	mu.TextHeight = func(_ microui.Font) int { return measurer.TextHeight() }

	a := &App{ctx: ctx, measurer: measurer, drawer: d, mu: mu, selection: start, showUI: true}
	a.sample = entries[start].Create(ctx)
	return a
}

// Resize sets the camera's pixel size; a zero dimension is ignored.
func (a *App) Resize(width, height int) {
	if width > 0 {
		a.ctx.Camera.Width = width
	}
	if height > 0 {
		a.ctx.Camera.Height = height
	}
}

// Camera returns the camera a host uses to build its projection.
func (a *App) Camera() *samples.Camera { return &a.ctx.Camera }

// Sample returns the sample currently running.
func (a *App) Sample() samples.Sample { return a.sample }

// Settings returns the settings a host or the tools window mutate.
func (a *App) Settings() *samples.Settings { return &a.ctx.Settings }

// Frame runs one iteration: pending restart or selection, the UI, the
// sample's Step, then the batches and UI commands the frame drew.
func (a *App) Frame(dtSeconds float64) (*draw.Batches, []render.UICommand) {
	_ = dtSeconds // the port never reads wall-clock; Base.Step paces itself from Settings.Hertz

	s := &a.ctx.Settings
	entries := samples.Entries()

	switch {
	case a.selection != s.SampleIndex:
		a.ctx.Camera.ResetView()
		s.SampleIndex = a.selection
		s.SubStepCount = 4
		s.DrawJoints = true
		s.UseCameraBounds = false
		s.Restart = false
		a.sample.Destroy()
		a.sample = entries[s.SampleIndex].Create(a.ctx)
	case s.Restart:
		// Restart stays set while the sample is built so its constructor
		// keeps the camera, as RestartSample does in the reference.
		a.sample.Destroy()
		a.sample = entries[s.SampleIndex].Create(a.ctx)
		s.Restart = false
	}

	a.drawer.Batches.Reset()

	a.mu.Begin()
	if a.showUI {
		a.buildToolsWindow(entries)
		a.buildMemoryWindow()
	}
	a.sample.UpdateGui()
	a.mu.End()

	if a.showUI {
		entry := entries[s.SampleIndex]
		if titled, ok := a.sample.(interface{ DrawTitle(string) }); ok {
			titled.DrawTitle(entry.Category + " : " + entry.Name)
		}
	}

	a.sample.Step()

	return &a.drawer.Batches, a.collectUICommands()
}

// worldOf type-asserts for Base.World, since Sample hides the world id
// behind Step and the input methods.
func worldOf(s samples.Sample) (dbox2d.WorldId, bool) {
	w, ok := s.(interface{ World() dbox2d.WorldId })
	if !ok {
		return dbox2d.WorldId{}, false
	}
	return w.World(), true
}
