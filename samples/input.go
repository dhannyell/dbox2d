// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/sample.h of Box2D v3.1.1

package samples

// Key is a keyboard key, numbered like GLFW so a host maps its own key
// events straight through.
type Key int

// Key constants, numbered like GLFW.
const (
	KeySpace  Key = 32
	Key0      Key = 48
	Key1      Key = 49
	Key2      Key = 50
	Key3      Key = 51
	Key4      Key = 52
	Key5      Key = 53
	Key6      Key = 54
	Key7      Key = 55
	Key8      Key = 56
	Key9      Key = 57
	KeyA      Key = 65
	KeyB      Key = 66
	KeyC      Key = 67
	KeyD      Key = 68
	KeyE      Key = 69
	KeyF      Key = 70
	KeyG      Key = 71
	KeyH      Key = 72
	KeyI      Key = 73
	KeyJ      Key = 74
	KeyK      Key = 75
	KeyL      Key = 76
	KeyM      Key = 77
	KeyN      Key = 78
	KeyO      Key = 79
	KeyP      Key = 80
	KeyQ      Key = 81
	KeyR      Key = 82
	KeyS      Key = 83
	KeyT      Key = 84
	KeyU      Key = 85
	KeyV      Key = 86
	KeyW      Key = 87
	KeyX      Key = 88
	KeyY      Key = 89
	KeyZ      Key = 90
	KeyEscape Key = 256
	KeyEnter  Key = 257
	KeyTab    Key = 258
	KeyRight  Key = 262
	KeyLeft   Key = 263
	KeyDown   Key = 264
	KeyUp     Key = 265
	KeyHome   Key = 268
	KeyEnd    Key = 269

	KeyPageUp      Key = 266
	KeyPageDown    Key = 267
	KeyLeftShift   Key = 340
	KeyLeftControl Key = 341

	KeyLeftBracket  Key = 91
	KeyRightBracket Key = 93
)

// MouseButton identifies a mouse button, numbered like GLFW.
type MouseButton int

// Mouse button constants, numbered like GLFW.
const (
	MouseButtonLeft   MouseButton = 0
	MouseButtonRight  MouseButton = 1
	MouseButtonMiddle MouseButton = 2
)

// Modifier is a bitset of keyboard modifiers held during an input event.
type Modifier int

// Modifier bit flags.
const (
	ModShift   Modifier = 1
	ModControl Modifier = 2
	ModAlt     Modifier = 4
)
