//go:build js && wasm

package wasm

import "github.com/dhannyell/dbox2d/samples"

// keyCodes maps a KeyboardEvent.code to the GLFW-numbered samples.Key the
// rest of the module expects. Unmapped codes are ignored.
var keyCodes = map[string]samples.Key{
	"Space": samples.KeySpace, "Tab": samples.KeyTab, "Escape": samples.KeyEscape, "Enter": samples.KeyEnter,
	"ArrowLeft": samples.KeyLeft, "ArrowRight": samples.KeyRight, "ArrowUp": samples.KeyUp, "ArrowDown": samples.KeyDown,
	"Home": samples.KeyHome, "End": samples.KeyEnd, "PageUp": samples.KeyPageUp, "PageDown": samples.KeyPageDown,
	"BracketLeft": samples.KeyLeftBracket, "BracketRight": samples.KeyRightBracket,
	"Digit0": samples.Key0, "Digit1": samples.Key1, "Digit2": samples.Key2, "Digit3": samples.Key3, "Digit4": samples.Key4,
	"Digit5": samples.Key5, "Digit6": samples.Key6, "Digit7": samples.Key7, "Digit8": samples.Key8, "Digit9": samples.Key9,
	"KeyA": samples.KeyA, "KeyB": samples.KeyB, "KeyC": samples.KeyC, "KeyD": samples.KeyD, "KeyE": samples.KeyE,
	"KeyF": samples.KeyF, "KeyG": samples.KeyG, "KeyH": samples.KeyH, "KeyI": samples.KeyI, "KeyJ": samples.KeyJ,
	"KeyK": samples.KeyK, "KeyL": samples.KeyL, "KeyM": samples.KeyM, "KeyN": samples.KeyN, "KeyO": samples.KeyO,
	"KeyP": samples.KeyP, "KeyQ": samples.KeyQ, "KeyR": samples.KeyR, "KeyS": samples.KeyS, "KeyT": samples.KeyT,
	"KeyU": samples.KeyU, "KeyV": samples.KeyV, "KeyW": samples.KeyW, "KeyX": samples.KeyX, "KeyY": samples.KeyY,
	"KeyZ": samples.KeyZ, "ShiftLeft": samples.KeyLeftShift, "ShiftRight": samples.KeyLeftShift,
	"ControlLeft": samples.KeyLeftControl, "ControlRight": samples.KeyLeftControl,
}
