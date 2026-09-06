//go:build cgo

package native

import (
	"github.com/go-gl/glfw/v3.4/glfw"

	"github.com/dhannyell/dbox2d/samples"
)

// keyOf maps a GLFW key to the samples.Key it equals numerically, keeping
// the set explicit so an unmapped key never leaks through as a wrong one.
func keyOf(key glfw.Key) (samples.Key, bool) {
	switch key {
	case glfw.KeySpace, glfw.KeyTab, glfw.KeyEnter,
		glfw.KeyLeft, glfw.KeyRight, glfw.KeyDown, glfw.KeyUp,
		glfw.KeyHome, glfw.KeyEnd, glfw.KeyPageUp, glfw.KeyPageDown,
		glfw.KeyLeftBracket, glfw.KeyRightBracket,
		glfw.Key0, glfw.Key1, glfw.Key2, glfw.Key3, glfw.Key4,
		glfw.Key5, glfw.Key6, glfw.Key7, glfw.Key8, glfw.Key9,
		glfw.KeyA, glfw.KeyB, glfw.KeyC, glfw.KeyD, glfw.KeyE,
		glfw.KeyF, glfw.KeyG, glfw.KeyH, glfw.KeyI, glfw.KeyJ,
		glfw.KeyK, glfw.KeyL, glfw.KeyM, glfw.KeyN, glfw.KeyO,
		glfw.KeyP, glfw.KeyQ, glfw.KeyR, glfw.KeyS, glfw.KeyT,
		glfw.KeyU, glfw.KeyV, glfw.KeyW, glfw.KeyX, glfw.KeyY, glfw.KeyZ:
		return samples.Key(key), true
	case glfw.KeyLeftShift, glfw.KeyRightShift:
		return samples.KeyLeftShift, true
	case glfw.KeyLeftControl, glfw.KeyRightControl:
		return samples.KeyLeftControl, true
	default:
		return 0, false
	}
}
