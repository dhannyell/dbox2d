// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/main.cpp (KeyCallback, MouseButtonCallback,
// MouseMotionCallback, ScrollCallback) of Box2D v3.1.1

package app

import (
	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/dbox2d/samples"
	"github.com/dhannyell/dbox2d/samples/internal/microui"
)

// uiCaptured reports whether the pointer is over, or a control holds focus
// in, the tools window; a captured pointer or key is not forwarded to the
// sample.
func (a *App) uiCaptured() bool {
	return a.mu.HoverRoot != nil || a.mu.Focus != 0
}

func muButton(b samples.MouseButton) int {
	switch b {
	case samples.MouseButtonLeft:
		return microui.MU_MOUSE_LEFT
	case samples.MouseButtonRight:
		return microui.MU_MOUSE_RIGHT
	case samples.MouseButtonMiddle:
		return microui.MU_MOUSE_MIDDLE
	default:
		return 0
	}
}

func (a *App) worldPoint(x, y float64) dbox2d.Vec2 {
	pw := a.ctx.Camera.ConvertScreenToWorld(samples.Vec2f{X: x, Y: y})
	return dbox2d.Vec2{X: samples.FromFloat64(pw.X), Y: samples.FromFloat64(pw.Y)}
}

// MouseMove forwards a pointer move to microui and to the sample, always:
// a drag that started in the world must keep its target while the pointer
// crosses a window, as the reference's MouseMotionCallback does.
func (a *App) MouseMove(x, y float64) {
	a.mu.InputMouseMove(int(x), int(y))

	pw := a.ctx.Camera.ConvertScreenToWorld(samples.Vec2f{X: x, Y: y})
	a.sample.MouseMove(dbox2d.Vec2{X: samples.FromFloat64(pw.X), Y: samples.FromFloat64(pw.Y)})

	if a.rightMouseDown {
		a.ctx.Camera.Center.X -= pw.X - a.rightMouseAt.X
		a.ctx.Camera.Center.Y -= pw.Y - a.rightMouseAt.Y
		a.rightMouseAt = a.ctx.Camera.ConvertScreenToWorld(samples.Vec2f{X: x, Y: y})
	}
}

// MouseDown starts a drag: the left button drags a body through the
// sample, the right button pans the camera.
func (a *App) MouseDown(x, y float64, button samples.MouseButton, mod samples.Modifier) {
	if btn := muButton(button); btn != 0 {
		a.mu.InputMouseDown(int(x), int(y), btn)
	}
	if a.uiCaptured() {
		return
	}

	switch button {
	case samples.MouseButtonLeft:
		a.sample.MouseDown(a.worldPoint(x, y), button, mod)
	case samples.MouseButtonRight:
		a.rightMouseDown = true
		a.rightMouseAt = a.ctx.Camera.ConvertScreenToWorld(samples.Vec2f{X: x, Y: y})
	}
}

// MouseUp ends a drag or a camera pan. It always reaches the sample and the
// camera, so a capture change mid-drag never leaves either stuck.
func (a *App) MouseUp(x, y float64, button samples.MouseButton) {
	if btn := muButton(button); btn != 0 {
		a.mu.InputMouseUp(int(x), int(y), btn)
	}

	switch button {
	case samples.MouseButtonLeft:
		a.sample.MouseUp(a.worldPoint(x, y), button)
	case samples.MouseButtonRight:
		a.rightMouseDown = false
	}
}

// Wheel zooms the camera. dy follows the browser convention (negative is
// wheel-up), the opposite of the reference's GLFW dy.
func (a *App) Wheel(dy float64) {
	if dy == 0 {
		return
	}
	a.mu.InputScroll(0, int(dy))
	if a.uiCaptured() {
		return
	}
	if dy < 0 {
		a.ctx.Camera.Zoom /= 1.1
	} else {
		a.ctx.Camera.Zoom *= 1.1
	}
}

// panStep is the reference's per-keypress camera pan distance.
const panStep = 0.5

// KeyDown handles one key press, matching KeyCallback's GLFW_PRESS switch
// (the port has no window to close on Escape, and ShiftOrigin's Ctrl
// variant is a no-op here as in the reference note).
func (a *App) KeyDown(key samples.Key, mod samples.Modifier) {
	a.held[key] = true
	s := &a.ctx.Settings
	entries := samples.Entries()

	switch key {
	case samples.KeyLeft:
		a.ctx.Camera.Center.X -= panStep
	case samples.KeyRight:
		a.ctx.Camera.Center.X += panStep
	case samples.KeyDown:
		a.ctx.Camera.Center.Y -= panStep
	case samples.KeyUp:
		a.ctx.Camera.Center.Y += panStep
	case samples.KeyHome:
		a.ctx.Camera.ResetView()
	case samples.KeyR:
		s.Restart = true
	case samples.KeyO:
		s.SingleStep = true
	case samples.KeyP:
		s.Pause = !s.Pause
	case samples.KeyLeftBracket:
		a.selection = (a.selection - 1 + len(entries)) % len(entries)
	case samples.KeyRightBracket:
		a.selection = (a.selection + 1) % len(entries)
	case samples.KeyTab:
		a.showUI = !a.showUI
	case samples.KeyEscape:
		// No window to close.
	default:
		a.sample.Keyboard(key)
	}
}

// KeyUp clears a key from the held-key state.
func (a *App) KeyUp(key samples.Key) {
	delete(a.held, key)
}

// ReleaseKeys clears every held key. Hosts call it on focus loss, where the
// platform sends no release events.
func (a *App) ReleaseKeys() {
	clear(a.held)
}
