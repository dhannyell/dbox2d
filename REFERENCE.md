# Reference source

This branch carries the Box2D source, unmodified, as the pinned reference for
the port on `main`.

- Upstream: <https://github.com/erincatto/box2d>
- Tag: `v3.1.1`
- Commit: `8c661469c9507d3ad6fbd2fea3f1aa71669c2fe3`, 2025-06-03
- License: MIT, Copyright (c) 2022 Erin Catto. See `LICENSE` in this branch.

The branch does not build, no Go file imports it, and it never merges into
`main`.

## Why the upstream history is here

The parent of the single commit on this branch is the upstream commit itself,
with its full ancestry. `git log`, `git blame`, `git bisect` and `git log -S`
therefore work on the reference. The port keeps the upstream structure because
of the edge cases that upstream fixed over the years, and blame is how a porter
finds which bug a strange-looking line answers.

That one commit adds this file and appends the `linguist-vendored` lines to
`.gitattributes`. Nothing else differs from upstream.

The upstream tags are deliberately absent from this repository. Go derives
module versions from tags, so an upstream tag here would publish a version of
this module that points at C source.

A newer upstream release gets its own branch, such as `reference/box2d-v3.2.0`.
This branch never changes.

## Usage

```sh
git show reference/box2d-v3.1.1:src/manifold.c
git log --oneline reference/box2d-v3.1.1 -- src/manifold.c
git blame reference/box2d-v3.1.1 -- src/manifold.c
```

The Go module zip contains only the tagged tree of `main`, so `go get` never
downloads this source.
