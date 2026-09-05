# dbox2d

`dbox2d` is a deterministic 2D rigid body solver for Go. It computes with signed
Q32.32 fixed-point arithmetic from
[`fixed`](https://github.com/dhannyell/fixed), so equal inputs produce the same
result bits on every supported architecture, on every run.

The module is pre-v1. Its import path and its API may change before the first
stable release. It requires Go 1.26.4 or newer.

The port covers its foundation, the narrowphase and the scalar contact
solver: worlds, bodies, shapes, mass computation, a determinism checksum,
contact manifolds for every shape pair, the distance solver, the shape
casts and the time of impact, the contact bookkeeping, the islands with
sleep and wake, the constraint graph, the soft-step contact solver, the
body and contact events, the dynamic trees, the broadphase and the AABB,
ray, shape and mover queries of the world, the seven joints and the filter
joint, the chains, the sensors and the character mover. `Step` finds the
new pairs, updates the contacts, solves the joints and the contacts,
sweeps the fast bodies and the bullets against the world so they stop at
their first time of impact, and puts resting islands to sleep.
[PORTING.md](PORTING.md) tracks what has landed.

### Naming

A reference function `b2<Type>_<Name>(id, ...)` becomes the method
`<Type>Id.<Name>(...)`: the underscore turns into the receiver dot and the
name stays whole, `Get` and `Set` included:

```go
b2Body_GetPosition(bodyId)              → bodyId.GetPosition()
b2RevoluteJoint_EnableLimit(jointId, x) → jointId.EnableLimit(true)
b2World_GetGravity(worldId)             → worldId.GetGravity()
```

`Create*`, `Destroy*`, `Default*Def`, `Make*` and the geometry functions
stay free functions: they carry no handle in the reference name.

### Not ported

A few pieces of the reference surface do not cross, by design:

- `b2World_GetProfile` and `b2Profile`: the port carries no timers to
  report.
- `b2World_DumpMemoryStats`: the port has no allocation hooks to walk.
- The `byteCount` and `taskCount` fields of `b2Counters`, and the task
  fields of `b2WorldDef`: they serve a task system this port does not
  have.
- The `void* context` parameter of every callback function type: a Go
  closure already carries its own state.

See [PORTING.md](PORTING.md) for the full map and
[DIVERGENCES.md](DIVERGENCES.md) for what changed shape to survive fixed
point.

`DefaultDebugDraw` supplies no-op callbacks, so a host can implement only the
operations it needs and call `worldId.Draw(&draw)`. The library walks the
world; rendering remains the host's responsibility.

## Fidelity contract

`dbox2d` is a port of [Box2D](https://box2d.org) v3.1.1, not a rewrite. It keeps
the upstream file decomposition, the upstream names without the `b2` prefix, and
the upstream order of operations. Years of edge-case work are the reason the
port exists, and a faithful port lets each function be compared against its
upstream counterpart.

The port preserves the upstream formulation, not its output bits. Box2D
computes in floating point; `dbox2d` computes in fixed point, so the two produce
different trajectories. Every place where fixed-point arithmetic forbids the
original line is a divergence, and every divergence carries a reason and a
test in [DIVERGENCES.md](DIVERGENCES.md). [PORTING.md](PORTING.md) maps each
upstream file to its Go counterpart and to how faithful that port is.

`dbox2d` is not affiliated with the Box2D project, and it is neither endorsed
nor supported by its author. Report defects here, never upstream.

## Install

```sh
go get github.com/dhannyell/dbox2d
```

## Performance

Fixed point buys determinism and pays in speed. The repository measures that
price with eight benchmark pairs and one bullet composite in
`bench_test.go`. Each pair runs the Q32.32
code against a line-by-line `float64` mirror of the same code, which stands
in for the floating-point formulation of the reference.

| Benchmark (amd64) | Median time | Allocations |
| --- | --- | --- |
| Pyramid `Step`, 210 boxes, 590 contacts, 4 sub-steps, Q32.32 | ~2.1 ms | 0 |
| Pyramid `Step`, 210 boxes, 590 contacts, 4 sub-steps, `float64` mirror | ~0.64 ms | 0 |
| Free-fall `Step`, 1024 bodies, 4 sub-steps, Q32.32 | ~0.36 ms | 0 |
| Free-fall `Step`, 1024 bodies, 4 sub-steps, `float64` mirror | ~0.11 ms | 0 |
| Velocity integration only, 1024 bodies, Q32.32 | ~31 µs | 0 |
| Velocity integration only, 1024 bodies, `float64` mirror | ~4.4 µs | 0 |
| One `CollidePolygons`, two boxes, Q32.32 | ~0.40 µs | 0 |
| One `CollidePolygons`, two boxes, `float64` mirror | ~0.091 µs | 0 |
| Broadphase pair update, pyramid, every proxy moved, Q32.32 | ~75 µs | 0 |
| Broadphase pair update, pyramid, every proxy moved, `float64` mirror | ~79 µs | 0 |
| One tree query, 100 boxes, four hits, Q32.32 | ~0.14 µs | 0 |
| One tree query, 100 boxes, four hits, `float64` mirror | ~0.14 µs | 0 |
| One `ShapeDistance`, two rotated boxes, cold cache, Q32.32 | ~0.33 µs | 0 |
| One `ShapeDistance`, two rotated boxes, cold cache, `float64` mirror | ~0.082 µs | 0 |
| One `TimeOfImpact`, a turning box that hits, Q32.32 | ~2.5 µs | 0 |
| One `TimeOfImpact`, a turning box that hits, `float64` mirror | ~0.46 µs | 0 |
| Bullet `Step`, 64 fast boxes bouncing between two walls, half bullets, 4 sub-steps, Q32.32 | ~59 µs | 0 |
| Pyramid step, solver probe on a Q16.16 grid with Q48.16 sums, test only | ~1.7 ms | 0 |
| Pyramid step, solver probe on a Q20.12 grid with 64-bit sums, test only | ~1.4 ms | 0 |
| Pyramid step, solver probe with the state in Q32.32 and each contact in Q16.16 lanes, test only | ~2.5 ms | 0 |
| Pyramid step, the same probe with the contacts colored and solved through the Q16 batch functions, scalar kernels, test only | ~1.9 ms | 0 |
| Pyramid step, the same probe on the AVX2 kernels (`GOEXPERIMENT=simd`, Go 1.27), test only | ~1.6 ms | 0 |

The composite numbers are the honest ones. The pyramid, a settled stack that
collides and solves every contact on every step, runs about **3.3×** slower
than its `float64` mirror and allocates nothing once its contacts are
active. The first step that activates contacts grows the graph colors, the
arena and the event buffers once, as the reference does; every step after
that allocates nothing. The
free-fall step, with no contacts, pays about 3.4×. The micro pairs explain
where the cost lives: the velocity integrator alone pays about 7×, because
each Q division is a 128-by-64-bit hardware divide and the damping factor
takes three of them per body, and the polygon collider pays about 4.4×. The
rest of the pipeline, with its bookkeeping, square roots and bounds work,
dilutes those hot spots. The pyramid mirror keeps one constraint list instead
of the graph colors and skips the island and event bookkeeping, so its ratio
is an upper bound. The broadphase pays nothing: the tree walks compare
bounds and sum perimeters, which cost the same in both formats, so the
pair update and the tree query run at parity. The pyramid mirror takes
its pairs from a `float64` mirror of the tree. The iterative geometry
pays about 4× on the distance solver and about 5× on the time of impact:
each iteration normalizes a direction and the root finder divides. The
bullet composite has no mirror; it pins the cost and the allocation
contract of the continuous stage, at less than a microsecond per fast
body.

The probe lines run the contact solver of the pyramid on narrower grids
inside `probe_q16_test.go` and `probe_wide_test.go`. The first two keep
the state on the narrow grid. The third keeps the state and the impulse
sums in Q32.32, reads them into Q16.16 lanes for each contact, and writes
back only the delta. The last two color the contacts so that no two in
one color share a dynamic body, and run each color as slices through the
`Batch*16` functions of `fixed`; the scalar and the AVX2 kernels give the
same bits. The probe measures the behaviour and the throughput of those
grids only; the library keeps Q32.32.

The `fixed` v0.3.0 release moved these numbers: its `Q32.Mul` now inlines,
and its `Normalize` skips two divisions when the length is already one. The
integrator dropped about a quarter and the collider about a third against
the previous snapshot, with the same result bits.

These numbers come from one machine (Ryzen 7 5800X3D) and one snapshot of the
code. Run `go test -run "^$" -bench . -benchmem` for your own.

## Reference source

The upstream C source sits in this repository on the branch
`reference/box2d-v3.1.1`, with the upstream history up to that tag. The branch
never merges into `main`, it never builds, and no Go file imports it. It exists
so that the porting work has one pinned, versioned reference that still answers
why a line looks the way it does:

```sh
git show reference/box2d-v3.1.1:src/manifold.c
git blame reference/box2d-v3.1.1 -- src/manifold.c
```

The upstream tags are deliberately absent. Go derives module versions from tags,
so an upstream tag here would publish a version of this module that points at C
source.

The Go module zip contains only the tagged tree, so `go get` never downloads the
reference source.

## License

MIT. This is a derivative work of Box2D, which is also MIT. See
[LICENSE](LICENSE).
