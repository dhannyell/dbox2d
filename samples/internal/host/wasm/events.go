//go:build js && wasm

package wasm

import (
	"syscall/js"

	"github.com/dhannyell/webgpu/wgpu"

	"github.com/dhannyell/dbox2d/samples"
	"github.com/dhannyell/dbox2d/samples/internal/app"
	"github.com/dhannyell/dbox2d/samples/internal/host/wgpuhost"
)

// canvasButton maps a PointerEvent.button to the sample's mouse button;
// button 1 (middle) has no reference behaviour and is dropped.
func canvasButton(jsButton int) (samples.MouseButton, bool) {
	switch jsButton {
	case 0:
		return samples.MouseButtonLeft, true
	case 2:
		return samples.MouseButtonRight, true
	default:
		return 0, false
	}
}

// bindEvents wires the canvas and the window to a. The funcs it creates
// with js.FuncOf are kept alive for the page's lifetime, matching Run's own
// requestAnimationFrame func.
func bindEvents(canvas js.Value, a *app.App, dev *wgpuhost.Device, surface *wgpu.Surface, configure func(w, h int), dpr float64) {
	pointerPos := func(e js.Value) (float64, float64) {
		return e.Get("offsetX").Float() * dpr, e.Get("offsetY").Float() * dpr
	}

	moveFn := js.FuncOf(func(_ js.Value, args []js.Value) any {
		e := args[0]
		x, y := pointerPos(e)
		a.MouseMove(x, y)
		return nil
	})
	canvas.Call("addEventListener", "pointermove", moveFn)

	downFn := js.FuncOf(func(_ js.Value, args []js.Value) any {
		e := args[0]
		e.Call("preventDefault")
		canvas.Call("setPointerCapture", e.Get("pointerId"))
		if btn, ok := canvasButton(e.Get("button").Int()); ok {
			x, y := pointerPos(e)
			a.MouseDown(x, y, btn, modifierOf(e))
		}
		return nil
	})
	canvas.Call("addEventListener", "pointerdown", downFn)

	upFn := js.FuncOf(func(_ js.Value, args []js.Value) any {
		e := args[0]
		e.Call("preventDefault")
		if btn, ok := canvasButton(e.Get("button").Int()); ok {
			x, y := pointerPos(e)
			a.MouseUp(x, y, btn)
		}
		return nil
	})
	canvas.Call("addEventListener", "pointerup", upFn)

	// A lost capture ends both drags, so a button never stays down.
	cancelFn := js.FuncOf(func(_ js.Value, args []js.Value) any {
		x, y := pointerPos(args[0])
		a.MouseUp(x, y, samples.MouseButtonLeft)
		a.MouseUp(x, y, samples.MouseButtonRight)
		return nil
	})
	canvas.Call("addEventListener", "pointercancel", cancelFn)

	wheelFn := js.FuncOf(func(_ js.Value, args []js.Value) any {
		e := args[0]
		e.Call("preventDefault")
		a.Wheel(e.Get("deltaY").Float())
		return nil
	})
	canvas.Call("addEventListener", "wheel", wheelFn)

	contextMenuFn := js.FuncOf(func(_ js.Value, args []js.Value) any {
		args[0].Call("preventDefault")
		return nil
	})
	canvas.Call("addEventListener", "contextmenu", contextMenuFn)

	keyDownFn := js.FuncOf(func(_ js.Value, args []js.Value) any {
		e := args[0]
		if e.Get("repeat").Bool() {
			return nil
		}
		code := e.Get("code").String()
		key, ok := keyCodes[code]
		if !ok {
			return nil
		}
		e.Call("preventDefault")
		a.KeyDown(key, modifierOf(e))
		return nil
	})
	js.Global().Call("addEventListener", "keydown", keyDownFn)

	keyUpFn := js.FuncOf(func(_ js.Value, args []js.Value) any {
		e := args[0]
		code := e.Get("code").String()
		key, ok := keyCodes[code]
		if !ok {
			return nil
		}
		e.Call("preventDefault")
		a.KeyUp(key)
		return nil
	})
	js.Global().Call("addEventListener", "keyup", keyUpFn)

	blurFn := js.FuncOf(func(_ js.Value, _ []js.Value) any {
		a.ReleaseKeys()
		return nil
	})
	js.Global().Call("addEventListener", "blur", blurFn)

	resizeFn := js.FuncOf(func(_ js.Value, _ []js.Value) any {
		width, height := canvasSize(dpr)
		canvas.Set("width", width)
		canvas.Set("height", height)
		configure(width, height)
		dev.Resize(width, height)
		a.Resize(width, height)
		return nil
	})
	js.Global().Call("addEventListener", "resize", resizeFn)
}

func modifierOf(e js.Value) samples.Modifier {
	var mod samples.Modifier
	if e.Get("shiftKey").Bool() {
		mod |= samples.ModShift
	}
	if e.Get("ctrlKey").Bool() {
		mod |= samples.ModControl
	}
	if e.Get("altKey").Bool() {
		mod |= samples.ModAlt
	}
	return mod
}
