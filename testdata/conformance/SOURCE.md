# Conformance trace source

The reference source is commit `0aa402e` on branch `reference/box2d-v3.1.1`. The generator is the source at the latest commit that touches `tools/conformance/main.c` (`git log -1 --format=%h -- tools/conformance/main.c`).

The traces were generated with GCC 13.2.0. The strict floating-point flags were `-ffp-contract=off -fno-fast-math` on the entire build. The Release generator and shared-library compilation used `-O3 -DNDEBUG -std=gnu17 -ffp-contract=off -fno-fast-math`. The reference library used the same flags plus `-fvisibility=hidden` and the definition `BOX2D_DISABLE_SIMD`.

Run these commands from `D:/Workspace/dbox2d`:

```sh
cmake -S tools/conformance -B build/conformance -G Ninja -DCMAKE_BUILD_TYPE=Release -DBOX2D_SOURCE_DIR=D:/Workspace/dbox2d-ref
cmake --build build/conformance
./build/conformance/conformance.exe --out testdata/conformance
```

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

The body count is the `bodies` header value measured after creation and before step 1. Body lines count all full-dump rows in the file.

| File | Bodies | Steps | Dumped steps | Body lines |
| --- | ---: | ---: | --- | ---: |
| `falling_hinges.txt` | 121 | 289 | 1, 289 | 242 |
| `joint_grid.txt` | 10000 | 500 | 500 | 10000 |
| `large_pyramid.txt` | 5051 | 500 | 500 | 5051 |
| `many_pyramids.txt` | 22001 | 200 | none | 0 |
| `rain.txt` | 1 | 1000 | 1, 1000 | 11277 |
| `smash.txt` | 9601 | 300 | 300 | 9601 |
| `spinner.txt` | 3040 | 1400 | 1, 1400 | 6080 |
| `tumbler.txt` | 2026 | 750 | 1, 750 | 4052 |

The initial corpus with all normal samples was 5,229,559 bytes. Removing every middle dump reduced it to 4,993,408 bytes. That was still over the limit. Omitting only the `many_pyramids` final dump reduced the checked-in corpus to 3,970,471 bytes. Every scene still has one hash per world step.
