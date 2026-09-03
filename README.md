# dbox2d

`dbox2d` is a deterministic 2D rigid body solver for Go. It computes with signed
Q32.32 fixed-point arithmetic from
[`fixed`](https://github.com/dhannyell/fixed), so equal inputs produce the same
result bits on every supported architecture, on every run.

The module is pre-v1. Its import path and its API may change before the first
stable release. It requires Go 1.26.4 or newer.

The port covers its foundation, the closed-form part of the narrowphase and
the scalar contact solver: worlds, bodies, shapes, mass computation, a
determinism checksum, contact manifolds for the supported shape pairs, the
contact bookkeeping, the islands with sleep and wake, the constraint graph,
the soft-step contact solver, the body and contact events, the dynamic
trees and the broadphase. `Step` finds the new pairs, updates the contacts,
solves them and puts resting islands to sleep. Chain segments against
capsules or polygons still wait for the iterative distance solver; joints,
sensors and continuous collision wait as well.
[PORTING.md](PORTING.md) tracks what has landed.

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
price with four benchmark pairs in `bench_test.go`. Each pair runs the Q32.32
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
is an upper bound.

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
