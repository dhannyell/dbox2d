# Conformance trace source, v3.2.0 staging

These are the v2-format function traces of the Box2D v3.2.0 reference. They
stage the migration described in `docs/v3.2-migration-plan.md`: phase 1 moves
the Go collision functions to the v3.2 signatures and reads these files, and
phase 5 replaces `testdata/conformance` with a full v3.2.0 corpus, scenes
included. No test reads this directory yet.

The reference source is commit `cd26c7e` on branch `reference/box2d-v3.2.0`,
the upstream tag `v3.2.0` (`178495b`) plus the provenance commit. The
generator, compiler and flags are the same as in
[`testdata/conformance/SOURCE.md`](../conformance/SOURCE.md): GCC 13.2.0,
`-O3 -DNDEBUG -std=gnu17 -ffp-contract=off -fno-fast-math`, the reference
library with `BOX2D_DISABLE_SIMD`, one worker.

With `workerCount = 1` and no task callbacks, v3.2.0 takes its serial
fallback and starts no thread. Two runs of the generator wrote identical
traces, scenes included.

Run these commands from the repository root, with the reference branch
checked out at `D:/Workspace/dbox2d-ref-v320`:

```sh
cmake -S tools/conformance -B build/conf320 -G Ninja -DCMAKE_BUILD_TYPE=Release -DBOX2D_SOURCE_DIR=D:/Workspace/dbox2d-ref-v320
cmake --build build/conf320
./build/conf320/conformance.exe --out build/out320
```

Then copy `build/out320/functions` here.

## Against v3.1.1

`atan2`, `make_rot` and `compute_hull` are equal to the v3.1.1 traces past the
header line. Every collision, distance and time-of-impact file differs,
because the v2 format records the relative transform and the frame-A
manifold.
