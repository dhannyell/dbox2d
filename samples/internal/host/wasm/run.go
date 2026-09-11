//go:build js && wasm

package wasm

import (
	"syscall/js"

	"github.com/dhannyell/webgpu/wgpu"

	"github.com/dhannyell/dbox2d/samples/internal/app"
	"github.com/dhannyell/dbox2d/samples/internal/draw"
	"github.com/dhannyell/dbox2d/samples/internal/host/wgpuhost"
	"github.com/dhannyell/dbox2d/samples/internal/render"
)

// Run configures the canvas's WebGPU surface, builds the app and the
// renderer, and drives them from requestAnimationFrame until the tab
// closes. It never returns while the page is open.
func Run() {
	canvas := js.Global().Get("document").Call("getElementById", "canvas")
	if !canvas.Truthy() {
		println(`wasm: no <canvas id="canvas"> in the page`)
		return
	}

	dpr := js.Global().Get("devicePixelRatio").Float()
	if dpr <= 0 {
		dpr = 1
	}
	width, height := canvasSize(dpr)
	canvas.Set("width", width)
	canvas.Set("height", height)

	instance := wgpu.CreateInstance(nil)
	if instance == nil {
		println("wasm: this browser does not support WebGPU")
		return
	}
	surface := instance.CreateSurface(&wgpu.SurfaceDescriptor{Canvas: canvas})

	adapter, err := instance.RequestAdapter(&wgpu.RequestAdapterOptions{CompatibleSurface: surface})
	if err != nil {
		println("wasm: request adapter: " + err.Error())
		return
	}
	wgpuDevice, err := adapter.RequestDevice(nil)
	if err != nil {
		println("wasm: request device: " + err.Error())
		return
	}

	surfaceFormat := surface.GetCapabilities(adapter).Formats[0]
	configure := func(w, h int) {
		surface.Configure(wgpuDevice, &wgpu.SurfaceConfiguration{
			Usage:       wgpu.TextureUsageRenderAttachment,
			Format:      surfaceFormat,
			Width:       uint32(w),
			Height:      uint32(h),
			PresentMode: wgpu.PresentModeFifo,
			AlphaMode:   wgpu.CompositeAlphaModeOpaque,
		})
	}
	configure(width, height)

	dev, err := wgpuhost.NewDevice(wgpuDevice, surface, surfaceFormat)
	if err != nil {
		println("wasm: " + err.Error())
		return
	}
	dev.Resize(width, height)

	// The app works in CSS pixels, as the reference works in window
	// coordinates divided by s_windowScale; the atlas is rasterized at
	// devicePixelRatio so text stays sharp.
	atlas, err := draw.NewAtlasScaled(draw.RegularFontSize, dpr)
	if err != nil {
		println("wasm: build atlas: " + err.Error())
		return
	}

	a := app.New(atlas)
	a.Resize(logicalSize())

	renderer, err := render.New(dev, atlas)
	if err != nil {
		println("wasm: build renderer: " + err.Error())
		return
	}

	bindEvents(canvas, a, dev, surface, configure, dpr)

	var lastTimestamp float64
	var raf js.Func
	raf = js.FuncOf(func(_ js.Value, args []js.Value) any {
		ts := args[0].Float()
		dt := 1.0 / 60
		if lastTimestamp != 0 {
			if d := (ts - lastTimestamp) / 1000; d > 0 {
				dt = d
			}
		}
		lastTimestamp = ts

		batches, ui := a.Frame(dt)
		if err := renderer.Frame(a.Camera(), batches, ui, ts/1000); err != nil {
			println("wasm: render: " + err.Error())
		}
		js.Global().Call("requestAnimationFrame", raf)
		return nil
	})
	js.Global().Call("requestAnimationFrame", raf)

	select {}
}

// canvasSize is the drawing-buffer size for a canvas that fills the
// viewport: the CSS size (set in web/index.html) times the device pixel
// ratio, so the surface is not upscaled by the browser.
func canvasSize(dpr float64) (int, int) {
	width := int(js.Global().Get("innerWidth").Float() * dpr)
	height := int(js.Global().Get("innerHeight").Float() * dpr)
	return width, height
}

// logicalSize is the canvas's CSS size, the pixel unit of the app's camera
// and UI.
func logicalSize() (int, int) {
	return js.Global().Get("innerWidth").Int(), js.Global().Get("innerHeight").Int()
}
