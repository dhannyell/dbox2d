# Conformance trace source

The reference source is commit `cd26c7e` on branch `reference/box2d-v3.2.0`, the upstream tag `v3.2.0` (`178495b`) plus the provenance commit. The generator is the source at the latest commit that touches `tools/conformance/main.c` (`git log -1 --format=%h -- tools/conformance/main.c`), built with `CONFORMANCE_V32`, which selects the v3.2 API and the v2 trace format.

The traces were generated with GCC 13.2.0. The strict floating-point flags were `-ffp-contract=off -fno-fast-math` on the entire build. The Release generator and shared-library compilation used `-O3 -DNDEBUG -std=gnu17 -ffp-contract=off -fno-fast-math`. The reference library used the same flags plus `-fvisibility=hidden` and the definition `BOX2D_DISABLE_SIMD`. With `workerCount = 1` and no task callbacks, v3.2.0 takes its serial fallback and starts no thread; two runs of the generator wrote identical traces.

Run these commands from the repository root, with the reference branch checked out at `D:/Workspace/dbox2d-ref-v320`:

```sh
cmake -S tools/conformance -B build/conf320 -G Ninja -DCMAKE_BUILD_TYPE=Release -DBOX2D_SOURCE_DIR=D:/Workspace/dbox2d-ref-v320
cmake --build build/conf320
./build/conf320/conformance.exe --out testdata/conformance
```

The v3.1.1 corpus is in the history up to the commit that replaced it.

## Large-world (double) scenes

`testdata/conformance_double/scenes/` holds the scene traces of the `dbox2d_double` build. They come from the same generator and the same reference source, built with `BOX2D_DOUBLE_PRECISION=ON`, where the generator writes each position as 16 hex digits (the bits of the double) and folds the 64 bits of each position coordinate into the hash. Rotations and velocities stay 8 hex digits. The function traces do not involve world positions and are identical in double mode, so the double corpus has no `functions` directory and the Go tests read `testdata/conformance/functions` in both modes.

```sh
cmake -S tools/conformance -B build/conf320d -G Ninja -DCMAKE_BUILD_TYPE=Release -DBOX2D_SOURCE_DIR=D:/Workspace/dbox2d-ref-v320 -DBOX2D_DOUBLE_PRECISION=ON
cmake --build build/conf320d
./build/conf320d/conformance.exe --out _scratch/out_double
cp -r _scratch/out_double/scenes testdata/conformance_double/
```

The Go side compares them with `go test -tags dbox2d_double -run Conformance ./`, with the same strictness as the float corpus.

## Function files

| File | Cases |
| --- | ---: |
| `atan2.txt` | 1024 |
| `collide_capsule_and_circle.txt` | 256 |
| `collide_capsules.txt` | 256 |
| `collide_chain_segment_and_capsule.txt` | 256 |
| `collide_chain_segment_and_circle.txt` | 256 |
| `collide_chain_segment_and_polygon.txt` | 256 |
| `collide_circles.txt` | 256 |
| `collide_polygon_and_capsule.txt` | 256 |
| `collide_polygon_and_circle.txt` | 256 |
| `collide_polygons.txt` | 256 |
| `collide_segment_and_capsule.txt` | 256 |
| `collide_segment_and_circle.txt` | 256 |
| `collide_segment_and_polygon.txt` | 256 |
| `compute_hull.txt` | 128 |
| `make_rot.txt` | 1024 |
| `shape_distance.txt` | 512 |
| `time_of_impact.txt` | 256 |

## Scene files

The body count is the `bodies` header value measured after creation and before step 1. Body lines count all full-dump rows in the file. Every scene has one hash per world step. The scenes follow the v3.2.0 shared sources: `falling_hinges` has 20 rows of square boxes, `spinner` twice the bodies at half spacing, and `many_pyramids` a narrower ground.

| File | Bodies | Steps | Dumped steps | Body lines |
| --- | ---: | ---: | --- | ---: |
| `falling_hinges.txt` | 81 | 300 | 1, 300 | 162 |
| `joint_grid.txt` | 10000 | 500 | 500 | 10000 |
| `large_pyramid.txt` | 5051 | 500 | 500 | 5051 |
| `many_pyramids.txt` | 22001 | 200 | none | 0 |
| `rain.txt` | 1 | 1000 | 1, 1000 | 11277 |
| `smash.txt` | 9601 | 300 | 300 | 9601 |
| `spinner.txt` | 6078 | 1400 | 1400 | 6078 |
| `tumbler.txt` | 2026 | 750 | 1, 750 | 4052 |
