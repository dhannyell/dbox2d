# dbox2d

[![CI](https://github.com/dhannyell/dbox2d/actions/workflows/test.yml/badge.svg)](https://github.com/dhannyell/dbox2d/actions/workflows/test.yml)
[![Go Reference](https://pkg.go.dev/badge/github.com/dhannyell/dbox2d.svg)](https://pkg.go.dev/github.com/dhannyell/dbox2d)
[![Latest tag](https://img.shields.io/github/v/tag/dhannyell/dbox2d?sort=semver)](https://github.com/dhannyell/dbox2d/tags)
[![License](https://img.shields.io/github/license/dhannyell/dbox2d)](LICENSE)
[![Go version](https://img.shields.io/github/go-mod/go-version/dhannyell/dbox2d)](go.mod)

Deterministic 2D physics for Go.

`dbox2d` is a Go port of [Box2D](https://box2d.org) v3.1.1. It offers two
scalar modes from the same solver: `float32` is the default, and
`-tags dbox2d_fixed` selects Q32.32 fixed-point arithmetic. Both modes are
deterministic; the difference in their reproducibility contracts is described
in [Scalar modes and determinism](#scalar-modes-and-determinism).

**[![Try the browser samples](samples/gopher.gif)](https://dhannyell.github.io/dbox2d/)**

[Open the live demo](https://dhannyell.github.io/dbox2d/) · [float mode](https://dhannyell.github.io/dbox2d/?mode=float) · [fixed mode](https://dhannyell.github.io/dbox2d/?mode=fixed)

The browser demo requires WebGPU and run on a single thread due to limitations in Go's WebAssembly implementation. To use multithreading, try the native version. see [the samples](samples/README.md).

## Highlights

- Box2D v3.1.1's public API, expressed in Go style.
- Deterministic results and a checksum over the complete world state.
- Bodies, shapes, sensors, chains, contacts, seven joint types, queries,
  casts, continuous collision, events, and debug drawing.
- Native and WebAssembly samples rendered with WebGPU.
- Optional SIMD contact solving with scalar-parity checks.

## Install

Requires Go 1.26.4 or newer.

```sh
go get github.com/dhannyell/dbox2d
```

The API follows Box2D closely. A reference function such as
`b2Body_GetPosition(bodyId)` becomes `bodyId.GetPosition()`; constructors,
defaults, and geometry helpers remain package functions.

## First world

```go
package main

import "github.com/dhannyell/dbox2d"

func main() {
	def := dbox2d.DefaultWorldDef()
	def.Gravity = dbox2d.Vec2{Y: dbox2d.QFromInt(-10)}
	world := dbox2d.CreateWorld(&def)
	defer dbox2d.DestroyWorld(world)

	world.Step(dbox2d.QFromRatio(1, 60), 4)
}
```

Use `QFromInt`, `QFromRatio`, `QFromFloat64`, or `QMustParse` to create values
that are rounded consistently by the selected scalar mode.

## Run the samples

The `samples` module includes the Box2D scenes in native and WebAssembly
hosts. Native runs need cgo, a C toolchain, and a Vulkan, Metal, or D3D12
driver. The browser host needs WebGPU.

```sh
cd samples
go run ./cmd/native
```

To build the browser host:

```sh
cd samples
CGO_ENABLED=0 GOOS=js GOARCH=wasm go build -o web/app.wasm ./cmd/web
GOTOOLCHAIN=go1.27.0 GOEXPERIMENT=simd CGO_ENABLED=0 GOOS=js GOARCH=wasm go build -tags dbox2d_simd -o web/app.wasm ./cmd/web
go run ./cmd/serve
```

Then open `http://localhost:8080`. The full list of controls, platform
requirements, and instructions for adding a scene are in
[samples/README.md](samples/README.md).

## Scalar modes and determinism

Both modes are deterministic, but they are different numeric systems and can
follow different trajectories. `ScalarMode` reports the active mode.

| Build | Arithmetic | Best for |
| --- | --- | --- |
| default | `float32` | General use, multiplayer, rollback, and authoritative simulation with a controlled build matrix |
| `-tags dbox2d_fixed` | Signed Q32.32 fixed point | A stronger cross-platform reproducibility contract |

Float is deterministic for a specific supported build and target. A silently
introduced fused multiply-add (FMA), compiler change, or other floating-point
variation can make two peers take different paths while each continues to
produce internally valid results. For float rollback, lockstep, or
authoritative simulation, pin the toolchain and flags, validate every supported
target, keep update order and random state stable, and compare checksums
regularly.

Fixed point provides a stronger and easier-to-audit contract across toolchains
and architectures, but does not make the complete application deterministic by
itself.

Use these constructors to create simulation values:

```go
dbox2d.QZero()
dbox2d.QOne()
dbox2d.QHalf()
dbox2d.QFromInt(3)
dbox2d.QFromRatio(1, 8)
dbox2d.QFromFloat64(0.35)
dbox2d.QMustParse("0.35")
```

`QFromFloat64` rounds to the nearest scalar of the mode, the same way on every
architecture, and `QToFloat64` converts back. A constant converts to the same
bits everywhere. A `float64` computed at run time is only as deterministic as
its computation: Go fuses a multiply and an add into one rounding on arm64,
and on amd64 with `GOAMD64=v3`, so the same expression can give different bits
in different builds.
`QMustParse` rounds a decimal once, while a literal passed to `QFromFloat64`
rounds to `float64` first, so a long literal can differ in the last bit.

Angles in the API are turns in both modes: one quarter turn is
`QFromRatio(1, 4)`, and an angular velocity is in turns per second. Inside,
the body state keeps radians per second, and a joint keeps its angles in
radians in float mode, as Box2D does, and in turns in fixed mode (D-004).

Build and test fixed mode with:

```sh
go build -tags dbox2d_fixed ./...
go test -tags dbox2d_fixed ./...
cd samples
go test -tags dbox2d_fixed ./...
go run -tags dbox2d_fixed ./cmd/native
```

The published sample page serves the float build by default. Add
`?mode=fixed` to load the fixed build instead.

The port is checked against traces of the reference compiled without SIMD and
without FMA; the collision functions match the reference bit for bit in float
mode except at three documented sites, and the fixed mode stays within measured
budgets. See DIVERGENCES.md D-018.

### SIMD

Use `dbox2d_simd` to enable the SIMD contact solver. It is optional and
produces the same result bits as the scalar solver.

| Target | Native path |
| --- | --- |
| amd64 | AVX2, with scalar fallback when AVX2 is unavailable |
| arm64 | NEON |
| wasm and other targets | generic path |

Native vector paths require Go 1.27.0 and `GOEXPERIMENT=simd`:

```sh
GOTOOLCHAIN=go1.27.0 GOEXPERIMENT=simd go build -tags dbox2d_simd ./...
GOTOOLCHAIN=go1.27.0 GOEXPERIMENT=simd go build -tags dbox2d_simd,dbox2d_fixed ./...
```

The generic path is useful for compatibility and conformance; benchmark it
before enabling it for production.

### Workers, callbacks, and drawing

`WorldDef.WorkerCount` controls native workers without changing result bits.
WebAssembly always steps on one worker. Go callbacks use closures rather than
the C API's `void*` context.

`DefaultDebugDraw` provides no-op callbacks. Implement the operations your
host needs and call `worldId.Draw(&draw)`; the library walks the world and the
host decides how to render it.

## Compatibility with Box2D

This is a port, not a new physics design. It preserves the upstream file
structure, names without the `b2` prefix, and order of operations so code can
be compared with its Box2D counterpart.

It does not promise the same output bits as Box2D. Both scalar modes share one
source, and the fixed-point mode forbids some float-oriented operations, so
those operations change in both modes. Each intentional change is documented with
its rationale and test coverage in [DIVERGENCES.md](DIVERGENCES.md).

`dbox2d` is not affiliated with, endorsed by, or supported by the Box2D
project. Please report `dbox2d` issues here rather than upstream.

## Performance

Performance depends on the workload and target. Native SIMD improves
contact-heavy scenes on supported CPUs, while the generic path may be slower
than scalar. Measure both scalar modes and the target that matters to you:

```sh
go test -run "^$" -bench . -benchmem
go test -tags dbox2d_fixed -run "^$" -bench . -benchmem
```

The repository's benchmark results are historical measurements, not a promise
for another machine or scene. The saturation counter used by some tests is
diagnostic only and never changes simulation results.

## Reference source

The original C source is available locally on the
`reference/box2d-v3.1.1` branch. It never merges into the Go branch and never
builds as part of this module. It is kept as a fixed reference for porting
work:

```sh
git show reference/box2d-v3.1.1:src/manifold.c
git blame reference/box2d-v3.1.1 -- src/manifold.c
```

Upstream tags are intentionally absent from that branch. Go derives module
versions from tags, and an upstream tag could otherwise make `go get` publish
a version containing the C reference tree. Module users download only the
tagged Go tree.

## License

MIT. This is a derivative work of Box2D, which is also MIT. See
[LICENSE](LICENSE).
