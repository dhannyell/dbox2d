# dbox2d

`dbox2d` is a deterministic 2D rigid body solver for Go. It computes with signed
Q32.32 fixed-point arithmetic from
[`fixed`](https://github.com/dhannyell/fixed), so equal inputs produce the same
result bits on every supported architecture, on every run.

The module is pre-v1. Its import path and its API may change before the first
stable release. It requires Go 1.26.4 or newer.

## Fidelity contract

`dbox2d` is a port of [Box2D](https://box2d.org) v3.1.1, not a rewrite. It keeps
the upstream file decomposition, the upstream names without the `b2` prefix, and
the upstream order of operations. Years of edge-case work are the reason the
port exists, and a faithful port lets each function be compared against its
upstream counterpart.

The closeness is one of formulation, never one of bits. Box2D computes in
floating point; `dbox2d` computes in fixed point. The two produce different
trajectories. Every place where fixed-point arithmetic forbids the original line
is a divergence, and every divergence carries a recorded reason and a test.

`dbox2d` is not affiliated with the Box2D project, and it is neither endorsed
nor supported by its author. Report defects here, never upstream.

## Install

```sh
go get github.com/dhannyell/dbox2d
```

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
