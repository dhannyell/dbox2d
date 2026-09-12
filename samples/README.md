# samples

The `dbox2d` sample gallery: the Box2D v3.1.1 scenes running in a native
window or in the browser through WebGPU.

[![Open the live demo](web/tumbler.png)](https://dhannyell.github.io/dbox2d/)

[Live demo: float mode](https://dhannyell.github.io/dbox2d/) · [fixed mode](https://dhannyell.github.io/dbox2d/?mode=fixed)

The default browser build uses `float32`; `?mode=fixed` loads Q32.32 fixed
point. The browser host runs on one worker because of Go WebAssembly limits.
The native host can use multiple workers.

This directory is a separate Go module and uses a local `replace` to test the
parent module while developing it.

## Run locally

The native host needs cgo, a C toolchain, and a GPU driver with Vulkan, Metal,
or D3D12.

```bash
cd samples && go run ./cmd/native
cd samples && go run -tags dbox2d_fixed ./cmd/native
```

Use `-tags dbox2d_fixed` to run the fixed-point build:

```bash
cd samples && go test ./...
cd samples && go test -tags dbox2d_fixed ./...
```

## Browser host

The browser host needs a browser with WebGPU. Build the wasm binary into
`web/` and serve that directory. `cmd/serve` is a static file server that
sends `.wasm` with the right content type; any other static server works.

```bash
cd samples && CGO_ENABLED=0 GOOS=js GOARCH=wasm go build -o web/app.wasm ./cmd/web
cd samples && CGO_ENABLED=0 GOOS=js GOARCH=wasm go build -tags dbox2d_fixed -o web/app-fixed.wasm ./cmd/web
```

The page loads `app.wasm` by default and `app-fixed.wasm` with `?mode=fixed`.

```bash
cd samples && go run ./cmd/serve
```

Then open `http://localhost:8080`. The published page uses the same two
artifacts.

## Headless WebAssembly probe

`cmd/probe` measures only `World.Step` on the Tumbler scene. It does not import
WebGPU, so it can run under the standard Go compiler or TinyGo. It warms up
for 120 steps, measures 600 steps, and prints average nanoseconds per step
plus a final checksum.

From this directory, build the standard Go binaries:

```powershell
$env:CGO_ENABLED = "0"
$env:GOOS = "js"
$env:GOARCH = "wasm"
go build -trimpath -ldflags='-s -w -X main.compiler=go' -o web/probe-go-fixed.wasm ./cmd/probe
go build -trimpath -tags dbox2d_float -ldflags='-s -w -X main.compiler=go' -o web/probe-go-float.wasm ./cmd/probe
Copy-Item "$(go env GOROOT)\lib\wasm\wasm_exec.js" web\wasm_exec-go.js
```

Build the TinyGo binaries with a TinyGo version compatible with the installed
Go toolchain:

```powershell
$env:CGO_ENABLED = "0"
$env:GOOS = "js"
$env:GOARCH = "wasm"
tinygo build -target=wasm -scheduler=asyncify -opt=2 -no-debug -tags fixed_nosatcounter -ldflags='-X main.compiler=tinygo' -o web/probe-tinygo-fixed.wasm ./cmd/probe
tinygo build -target=wasm -scheduler=asyncify -opt=2 -no-debug -tags dbox2d_float -ldflags='-X main.compiler=tinygo' -o web/probe-tinygo-float.wasm ./cmd/probe
$tinyRoot = tinygo env TINYGOROOT
Copy-Item "$tinyRoot\targets\wasm_exec.js" web\wasm_exec-tinygo.js
```

Serve the directory and open one of these URLs. The result is printed in the
browser console:

```text
http://localhost:8080/probe.html?compiler=go&mode=fixed
http://localhost:8080/probe.html?compiler=go&mode=float
http://localhost:8080/probe.html?compiler=tinygo&mode=fixed
http://localhost:8080/probe.html?compiler=tinygo&mode=float
```

Compare `ns_per_step` only between equal modes. Fixed checksums should match
between Go and TinyGo; float checksums are diagnostic because compilers can
evaluate floating-point expressions differently.

## Keys

| Key | Action |
|---|---|
| `[` / `]` | previous / next scene |
| `R` | restart the scene |
| `P` | pause |
| `O` | single step |
| arrows | pan the camera |
| `Home` | reset the camera |
| `Tab` | hide or show the UI |
| left drag | grab a body with a mouse joint |
| right drag / wheel | pan / zoom |

Some scenes read their own keys; the scene draws a hint at the top left.

## Add a scene

A scene is a struct that embeds `Base` and satisfies `Sample`. The
constructor builds the world; `Step` runs the shared step first. Register
it in an `init` under its category and it appears in the menu.

```go
type Pendulum struct {
	Base
}

func NewPendulum(ctx *SampleContext) Sample {
	s := &Pendulum{Base: NewBase(ctx)}
	if !ctx.Settings.Restart {
		ctx.Camera.Center = Vec2f{X: 0, Y: 5}
		ctx.Camera.Zoom = 10
	}
	// Create the ground, the bodies and the joints here.
	return s
}

func (s *Pendulum) Step() {
	s.Base.Step()
}

func init() {
	RegisterSample("Joints", "Pendulum", NewPendulum)
}
```

The port keeps the reference conventions with three changes. Every
simulation value is a `dbox2d.Q`; write constants with
`dbox2d.QMustParse("0.35")` or `dbox2d.QFromRatio(1, 8)`. Every angle is a
turn, not a radian: `π/2` is `1/4`; a motor speed in rad/s goes through
`radiansToTurns`. GUI sliders hold a `float64` and convert with
`FromFloat64` when the value enters the world.

## Scenes

Every scene of the reference is ported, under the same category and name.
Barrel adds a Gopher shape that is not in the reference; it is the default
shape, and Barrel is the scene the app opens on.

The scenes produce the same bits with any worker count.

## Layout

| Directory | Role |
|---|---|
| `.` | the scenes, `Sample`, `Base`, the camera, the settings, the registry |
| `internal/app` | the app loop, the menu, the settings panel, the profile overlay, input routing |
| `internal/draw` | the draw batches and the WGSL shaders |
| `internal/gpu`, `internal/render` | the WebGPU pipelines |
| `internal/host/native` | GLFW window and surface |
| `internal/host/wasm` | canvas, `requestAnimationFrame` and DOM events |
| `web` | the page, `wasm_exec.js`, the screenshot and the built `app.wasm` and `app-fixed.wasm` (ignored) |
| `internal/microui` | vendored copy of `zeozeozeo/microui-go` v1.0.1 (Unlicense) |
| `cmd/native`, `cmd/web`, `cmd/serve` | the two hosts and the static server |
