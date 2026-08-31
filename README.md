# dbox2d

`dbox2d` is a deterministic 2D rigid body solver for Go. It computes with signed
Q32.32 fixed-point arithmetic from
[`fixed`](https://github.com/dhannyell/fixed), so equal inputs produce the same
result bits on every supported architecture, on every run.

The module is pre-v1. Its import path and its API may change before the first
stable release. It requires Go 1.26.4 or newer.

The port is in its foundation stage: worlds, bodies, shapes, mass computation,
a determinism checksum and an integration-only `Step`. Contact and joint
solving are not ported yet. [PORTING.md](PORTING.md) tracks what has landed.

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
price with a benchmark pair in `bench_test.go`: the velocity integrator in
Q32.32, and a line-by-line `float64` mirror of the same code, which stands in
for the floating-point formulation of the reference.

| Benchmark (1024 bodies, amd64) | Median time | Allocations |
| --- | --- | --- |
| Velocity integration, Q32.32 | ~41 µs | 0 |
| Velocity integration, `float64` mirror | ~4.8 µs | 0 |
| Full `Step`, 4 sub-steps, Q32.32 | ~0.5 ms | 0 |

The fixed-point integrator runs about 8.5× slower than its `float64` mirror.
The dominant cost is division: each Q division is a 128-by-64-bit hardware
divide, and the damping factor alone takes three of them per body. A full
`Step` costs about 0.5 µs per body and allocates nothing after the world is
built.

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
