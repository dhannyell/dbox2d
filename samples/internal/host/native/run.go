//go:build cgo

// Package native hosts the samples module in a desktop window: it drives
// the same app, renderer and WGSL as the wasm host through GLFW and
// wgpu-native, over the shared wgpuhost seam implementation.
package native

import (
	"fmt"
	"os"

	"github.com/go-gl/glfw/v3.4/glfw"

	"github.com/dhannyell/dbox2d/samples/internal/app"
	"github.com/dhannyell/dbox2d/samples/internal/draw"
	"github.com/dhannyell/dbox2d/samples/internal/host/wgpuhost"
	"github.com/dhannyell/dbox2d/samples/internal/render"
)

// windowWidth and windowHeight are window coordinates, not framebuffer
// pixels; the surface is sized from the framebuffer instead.
const (
	windowWidth  = 1920
	windowHeight = 1080
)

// Run opens the window, builds the app and the renderer, and drives them
// until the window closes.
func Run() error {
	if err := glfw.Init(); err != nil {
		return fmt.Errorf("native: init glfw: %w", err)
	}
	defer glfw.Terminate()

	glfw.WindowHint(glfw.ClientAPI, glfw.NoAPI)
	glfw.WindowHint(glfw.Resizable, glfw.True)
	window, err := glfw.CreateWindow(windowWidth, windowHeight, "dbox2d samples", nil, nil)
	if err != nil {
		return fmt.Errorf("native: create window: %w", err)
	}
	defer window.Destroy()

	gpuSt, err := newGPUState(window)
	if err != nil {
		return err
	}
	defer gpuSt.destroy()

	fbW, fbH := window.GetFramebufferSize()
	if !gpuSt.configure(fbW, fbH) {
		return fmt.Errorf("native: window has a zero framebuffer size at startup")
	}

	dev, err := wgpuhost.NewDevice(gpuSt.device, gpuSt.surface, gpuSt.config.Format)
	if err != nil {
		return fmt.Errorf("native: %w", err)
	}
	dev.Resize(fbW, fbH)

	fbDirty := false
	dev.OnSurfaceOutdated = func() { fbDirty = true }

	// The atlas is rasterized once at a fixed pixel size; it is not scaled
	// by the window's content scale.
	atlas, err := draw.NewAtlas(14)
	if err != nil {
		return fmt.Errorf("native: build atlas: %w", err)
	}

	a := app.New(atlas)
	a.Resize(fbW, fbH)

	renderer, err := render.New(dev, atlas)
	if err != nil {
		return fmt.Errorf("native: build renderer: %w", err)
	}

	scale := func() (float64, float64) {
		winW, winH := window.GetSize()
		fbW, fbH := window.GetFramebufferSize()
		if winW == 0 || winH == 0 {
			return 1, 1
		}
		return float64(fbW) / float64(winW), float64(fbH) / float64(winH)
	}
	registerCallbacks(window, a, &fbDirty, scale)

	lastTime := 0.0
	for !window.ShouldClose() {
		glfw.PollEvents()

		// A minimized window has no surface to present, so the loop would
		// spin; block until the next event instead.
		if w, h := window.GetFramebufferSize(); w == 0 || h == 0 {
			glfw.WaitEvents()
			continue
		}

		if fbDirty {
			fbDirty = false
			fbW, fbH := window.GetFramebufferSize()
			if gpuSt.configure(fbW, fbH) {
				dev.Resize(fbW, fbH)
				a.Resize(fbW, fbH)
			}
		}

		now := glfw.GetTime()
		dt := 1.0 / 60
		if lastTime != 0 {
			if d := now - lastTime; d > 0 {
				dt = d
			}
		}
		lastTime = now

		batches, ui := a.Frame(dt)
		if err := renderer.Frame(a.Camera(), batches, ui, now); err != nil {
			fmt.Fprintln(os.Stderr, "native: render:", err)
		}
	}
	return nil
}
