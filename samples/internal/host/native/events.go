//go:build cgo

package native

import (
	"github.com/go-gl/glfw/v3.4/glfw"

	"github.com/dhannyell/dbox2d/samples"
	"github.com/dhannyell/dbox2d/samples/internal/app"
)

// wheelStep is one GLFW notch expressed in browser wheel pixels, the unit
// app.Wheel takes.
const wheelStep = 100

func modifierOf(mods glfw.ModifierKey) samples.Modifier {
	var mod samples.Modifier
	if mods&glfw.ModShift != 0 {
		mod |= samples.ModShift
	}
	if mods&glfw.ModControl != 0 {
		mod |= samples.ModControl
	}
	if mods&glfw.ModAlt != 0 {
		mod |= samples.ModAlt
	}
	return mod
}

// registerCallbacks wires window input to a and marks fbDirty on resize, so
// Run reconfigures the surface on its next iteration instead of inside the
// callback.
func registerCallbacks(window *glfw.Window, a *app.App, fbDirty *bool, scale func() (float64, float64)) {
	cursor := func() (float64, float64) {
		x, y := window.GetCursorPos()
		sx, sy := scale()
		return x * sx, y * sy
	}

	window.SetFramebufferSizeCallback(func(_ *glfw.Window, _, _ int) {
		*fbDirty = true
	})

	window.SetCursorPosCallback(func(_ *glfw.Window, x, y float64) {
		sx, sy := scale()
		a.MouseMove(x*sx, y*sy)
	})

	window.SetMouseButtonCallback(func(_ *glfw.Window, button glfw.MouseButton, action glfw.Action, mods glfw.ModifierKey) {
		var btn samples.MouseButton
		switch button {
		case glfw.MouseButtonLeft:
			btn = samples.MouseButtonLeft
		case glfw.MouseButtonRight:
			btn = samples.MouseButtonRight
		default:
			return
		}
		x, y := cursor()
		switch action {
		case glfw.Press:
			a.MouseDown(x, y, btn, modifierOf(mods))
		case glfw.Release:
			a.MouseUp(x, y, btn)
		}
	})

	window.SetScrollCallback(func(_ *glfw.Window, _, yoffset float64) {
		if yoffset == 0 {
			return
		}
		a.Wheel(-yoffset * wheelStep)
	})

	window.SetKeyCallback(func(w *glfw.Window, key glfw.Key, _ int, action glfw.Action, mods glfw.ModifierKey) {
		if action == glfw.Repeat || action == glfw.Release {
			return
		}
		if key == glfw.KeyEscape {
			w.SetShouldClose(true)
			return
		}
		if k, ok := keyOf(key); ok {
			a.KeyDown(k, modifierOf(mods))
		}
	})
}
