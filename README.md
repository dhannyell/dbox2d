# dbox2d

`dbox2d` is a deterministic 2D physics library for Go and a port of
[Box2D](https://box2d.org) v3.1.1. It has two scalar modes: the default build
uses `float32`, while `-tags dbox2d_fixed` selects Q32.32 fixed-point
arithmetic. In either mode, equal inputs produce the same result bits on every
supported architecture and on every run.

Fixed point is the safer choice for cross-platform rollback, replay, and
authoritative simulation. Float mode is also deterministic on the
architectures covered by this repository's CI, and is usually faster in the
measured workloads. A float-mode application must still control its complete
simulation, not only its physics.

The project is pre-v1. Its public API and import path may change before the
first stable release. It requires Go 1.26.4 or newer.

**[![Tumbler in the browser host](samples/tumbler.gif)](https://dhannyell.github.io/dbox2d/)**\
**Try the sample scenes in browser: [float mode](https://dhannyell.github.io/dbox2d/) · [fixed mode](https://dhannyell.github.io/dbox2d/?mode=fixed) (WebGPU required)**

## What is included

The current port includes worlds, bodies, shapes, mass properties, contact
manifolds, distance queries, shape casts, time of impact, sleeping islands,
the constraint graph, the soft-step contact solver, events, dynamic trees,
broadphase queries, chains, sensors, the character mover, and all seven Box2D
joints plus the filter joint.

`WorldId.Step` creates new contact pairs, updates contacts, solves joints and
contacts, handles fast bodies and bullets, and puts resting islands to sleep.
It also exposes step profiling and memory statistics.

[PORTING.md](PORTING.md) records the status of the Box2D port. Read
[DIVERGENCES.md](DIVERGENCES.md) when you need to understand a deliberate
difference from the C implementation.

## Install

```sh
go get github.com/dhannyell/dbox2d
```

The package follows the Box2D API closely. Reference functions named
`b2<Type>_<Name>` normally become methods on the matching ID type:

```go
b2Body_GetPosition(bodyId)              // bodyId.GetPosition()
b2RevoluteJoint_EnableLimit(jointId, x) // jointId.EnableLimit(true)
b2World_GetGravity(worldId)             // worldId.GetGravity()
```

Functions that do not operate on an existing handle remain package functions,
including `Create*`, `Destroy*`, `Default*Def`, `Make*`, and geometry helpers.

## Run the samples

The `samples` module contains 47 Box2D scenes in the Stacking, Benchmark, and
Joints categories. A native host and a browser host render the same scenes with
WebGPU.

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

## Determinism and scalar modes

The default build stores simulation values in `float32`. The optional
`dbox2d_fixed` build tag uses signed Q32.32 fixed point from
[`fixed`](https://github.com/dhannyell/fixed) instead. Both modes keep a
deterministic result for a given architecture-supported build, but they are
different numeric systems and can follow different trajectories. `ScalarMode`
reports the mode of a build: `"float"` or `"fixed"`.

| Build | Scalar type | Determinism promise | When to use it |
| --- | --- | --- | --- |
| default | `float32` | identical float32 bits on amd64, arm64, 386, and wasm | most applications |
| `-tags dbox2d_fixed` | Q32.32 fixed point | identical Q32.32 bits across supported architectures | authoritative simulation, replay, rollback, and checksums |

Use the package constructors to create simulation values:

```go
QZero()
QOne()
QHalf()
QFromInt(3)
QFromRatio(1, 8)
QMustParse("0.35")
```

`QFromFloat64` and `QToFloat64` are for values entering or leaving the
simulation. The solver itself does not use them. Angles are expressed in turns:
one quarter turn is `QFromRatio(1, 4)`.

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

### SIMD family

The `dbox2d_simd` build tag enables a SIMD contact solver alongside the scalar
solver in both modes. Colored contacts are solved in lanes; the overflow color
remains scalar.

| Mode | amd64 | arm64 | wasm | Other targets |
| --- | --- | --- | --- | --- |
| fixed | AVX2, width 8 | NEON, width 4 | generic | generic |
| float | AVX2, width 8 | NEON, width 4 | SIMD128, width 4 | generic |

In fixed mode, the `fixed` module selects the lane implementation and
`fixed.LanePath()` reports the active path. Native vector paths require
`GOEXPERIMENT=simd` with Go 1.27.0. Without the experiment, the same build tag
uses the generic path.

On amd64, AVX2 falls back to the scalar contact solver at runtime when the CPU
does not support AVX2.

```sh
GOTOOLCHAIN=go1.27.0 GOEXPERIMENT=simd go build -tags dbox2d_simd ./...
GOTOOLCHAIN=go1.27.0 GOEXPERIMENT=simd go build -tags dbox2d_simd,dbox2d_fixed ./...
```

The SIMD and scalar families produce identical result bits on every supported
path in both modes. Float SIMD never fuses multiply-add or multiply-subtract
operations (DIVERGENCES.md D-019). Fixed SIMD uses the same Q16.16 contact grid
as the scalar solver and matches it while no contact value saturates. A
contact whose inverse mass or inertia falls outside the lane window of that
grid solves in Q32.32 in both families (D-020).

Measured on an AMD Ryzen 7 5800X3D: milliseconds for 60 steps of a settled
scene (after 240 warmup steps), median of 5 interleaved runs.

| Scene | Mode | Workers | Scalar | SIMD AVX2 | Generic |
| --- | --- | ---: | ---: | ---: | ---: |
| Tumbler | float | 1 | 444 | 298 | 531 |
| Tumbler | float | 8 | 124 | 100 | 176 |
| Tumbler | fixed | 1 | 1312 | 815 | 3563 |
| Tumbler | fixed | 8 | 350 | 244 | 817 |
| LargePyramid | float | 1 | 734 | 377 | 993 |
| LargePyramid | float | 8 | 152 | 92 | 227 |
| LargePyramid | fixed | 1 | 3125 | 1746 | 11093 |
| LargePyramid | fixed | 8 | 636 | 416 | 1867 |
| Barrel | float | 1 | 363 | 288 | 412 |
| Barrel | float | 8 | 133 | 120 | 157 |
| Barrel | fixed | 1 | 876 | 581 | 2555 |
| Barrel | fixed | 8 | 258 | 203 | 578 |

Fixed SIMD remains 1.7 to 4.6 times slower than float SIMD in these scenes. On
AVX2, each Q16.16 lane multiplication must widen to 64 bits before narrowing
back to Q16.16.

The generic lane path exists for conformance and is slower than the scalar
solver: up to 1.5 times in float mode and up to 3.5 times in fixed mode. Do not
enable `dbox2d_simd` on targets without a native vector path.
For fixed mode, this currently includes wasm.

### Choosing a mode for deterministic simulation

Both modes support deterministic simulation. Q32.32 is the recommended choice
for cross-platform rollback and replay because its arithmetic has a simpler,
stronger reproducibility contract.

Float mode is suitable for rollback when an application verifies its whole
simulation on every supported target. Use a fixed simulation tick, serialize
all state that affects future frames, use a deterministic random-number
generator, keep update order stable, and avoid platform-dependent math or
application logic outside `dbox2d`. Compare checksums regularly in development
and detect desynchronization in production.

### Workers and callbacks

A world owns its workers through `WorldDef.WorkerCount`. Go callbacks keep
their own state; they do not share the `void* context` convention of the C API.
Changing the worker count does not change the result bits. The repository pins
that contract in `TestStepIsWorkerCountIndependent`.

### Debug drawing

`DefaultDebugDraw` provides no-op callbacks. Implement only the drawing
operations your host needs, then call `worldId.Draw(&draw)`. The library walks
the world; the host decides how to render it.

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

Fixed point trades speed for reproducibility. The numbers below are medians
from one machine: AMD Ryzen 7 5800X3D, Windows/amd64, `GOMAXPROCS=16`, six
runs, summarized with `benchstat`. They are useful for understanding the
trade-off, not as a promise for another machine or workload.

| Workload | Q32.32 | `float64` mirror | Allocation |
| --- | ---: | ---: | ---: |
| Pyramid step: 210 boxes, 590 contacts, 4 sub-steps | ~2.1 ms | ~0.64 ms | 0 |
| Free-fall step: 1,024 bodies, 4 sub-steps | ~0.36 ms | ~0.11 ms | 0 |
| Velocity integration: 1,024 bodies | ~31 us | ~4.4 us | 0 |
| One polygon collision | ~0.40 us | ~0.091 us | 0 |
| Broadphase update: moving pyramid proxies | ~75 us | ~79 us | 0 |
| Dynamic-tree query: 100 boxes, four hits | ~0.14 us | ~0.14 us | 0 |
| One shape-distance query | ~0.33 us | ~0.082 us | 0 |
| One time-of-impact query | ~2.5 us | ~0.46 us | 0 |
| Bullet step: 64 fast boxes, 4 sub-steps | ~59 us | not measured | 0 |

The comparison rows use a line-by-line `float64` mirror of the same code. It
is a controlled measurement of the cost of the fixed-point implementation; it
is not a claim that the mirror is the complete upstream Box2D pipeline.

In the contact-heavy pyramid, the fixed build is about 3.3 times slower than
that mirror and allocates nothing after contacts have been created. Broadphase
updates and tree queries are effectively at parity because they mostly compare
bounds rather than perform fixed-point division.

Run the benchmarks on the hardware and workload that matter to your project:

```sh
go test -run "^$" -bench . -benchmem
go test -tags dbox2d_fixed -run "^$" -bench . -benchmem
```

The saturation counter of `fixed` is off by default and never changes a
result. The tests that assert that no operation saturated read it, so they
check nothing unless the build sets `fixed_satcounter`. CI sets it.

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
