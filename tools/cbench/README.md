# Scene benchmark: port against the reference

The port and the reference run the same seven benchmark scenes, for the same
step counts, under one protocol, and write one JSON schema. `benchcmp` reads
two of those files and prints the ratio per scene.

The two sides are:

| side | harness | build |
| --- | --- | --- |
| Go | `TestSceneBench` in `bench_scene_test.go` | `dbox2d_simd`, `GOEXPERIMENT=simd`, Go 1.27.0 |
| C | `tools/cbench` | Box2D v3.1.1 with `BOX2D_AVX2=ON` |

Both select the width-8 AVX2 lane path, so the comparison holds the
algorithm, the lane width and the FMA rule fixed and varies only the
language. Both are swept over one, two, four and eight workers, so the
comparison also says whether the port keeps pace as threads are added.

## Protocol

It is the protocol of `benchmark/main.c` in the reference tree:

- The world is built fresh for every repeat, with `workerCount` workers.
- The scene hook runs for step 0, then one world step runs **untimed**. The
  first step is expensive and skews the result.
- The remaining steps run as one timed block. Each step is also timed on its
  own, and each hook call is timed on its own.
- A scene reports the minimum block total over the repeats, and the
  element-wise minimum per step index over the repeats.

`total_ms` is the block; `step_sum_ms` is the summed per-step minima;
`hook_ms` is the scene hook. `rain` is the only scene whose hook does real
work, so `step_sum_ms` is the number that compares the two solvers and
`hook_ms` is the one that compares body creation and destruction.

## Reproduce

The reference tree is not vendored. Clone Box2D v3.1.1 anywhere and name it
with `BOX2D_SOURCE_DIR`; the default is a sibling checkout named `box2d`, so
a clone next to this repository needs no flag. Configure refuses to proceed
when the path holds no Box2D source.

```sh
git clone --branch v3.1.1 --depth 1 https://github.com/erincatto/box2d ../box2d
```

```sh
cmake -S tools/cbench -B build/cbench -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/cbench
```

A tree elsewhere is named on the command line:

```sh
cmake -S tools/cbench -B build/cbench -G Ninja -DCMAKE_BUILD_TYPE=Release -DBOX2D_SOURCE_DIR=/path/to/box2d
```

```sh
for w in 1 2 4 8; do
  ./build/cbench/cbench.exe --want-lane=avx2 --repeats=3 --workers=$w \
    --arm=c-avx2 --out=testdata/bench/c-avx2-w$w.json
done
```

```sh
for w in 1 2 4 8; do
  GOTOOLCHAIN=go1.27.0 GOEXPERIMENT=simd go test -tags dbox2d_simd \
    -run TestSceneBench -timeout 7200s . \
    -bench.arm=go-avx2 -bench.want-lane=avx2 -bench.repeats=3 -bench.workers=$w \
    -bench.out=testdata/bench/go-avx2-w$w.json
done
```

The PGO arm is the same sweep from a test binary built with a profile. Build
the binary once, because `go test -pgo=...` would reapply the profile per
invocation, and name `-pgo=off` for the control so the comparison is against
a build from the same sitting:

```sh
GOTOOLCHAIN=go1.27.0 GOEXPERIMENT=simd go test -tags dbox2d_simd -c -o bench-pgo.test -pgo=cpu.pprof .
GOTOOLCHAIN=go1.27.0 GOEXPERIMENT=simd go test -tags dbox2d_simd -c -o bench-off.test -pgo=off .
```

```sh
for w in 1 2 4 8; do
  ./bench-pgo.test -test.run TestSceneBench -test.timeout 7200s     -bench.arm=go-pgo -bench.want-lane=avx2 -bench.repeats=3 -bench.workers=$w     -bench.out=testdata/bench/go-pgo-w$w.json
done
```

Interleave the two PGO arms round by round rather than finishing one and then
the other. A first attempt ran them in sequence and measured the control
42 percent slower than the same build an hour earlier; the machine had
drifted, not the code.

`benchcmp` compares one worker count; `benchscale` reads the whole sweep:

```sh
go run ./tools/benchcmp testdata/bench/c-avx2-w1.json testdata/bench/go-avx2-w1.json
```

```sh
go run ./tools/benchscale -base=c-avx2 testdata/bench/c-avx2-w*.json testdata/bench/go-avx2-w*.json
```

Run the two arms one after the other, never at the same time: two benchmarks
competing for the same cores measure the contention, not the code.

`testdata/bench/` is not tracked: a result is one machine on one day, and
the numbers worth keeping are summarised here and in the root README.
Create the directory before the first run.

## Workers

`b2CreateWorld` keeps `workerCount` only when `enqueueTask` and `finishTask`
are both set (`src/world.c:224`). A harness that sets the count alone gets a
world silently reset to one worker, so `--workers=8` would have measured
single-threaded C against multi-threaded Go and called the difference a
language gap.

The reference benchmark supplies those callbacks through enkiTS, which this
tree does not vendor and which would otherwise have to be fetched from the
network at configure time. `tools/cbench/taskpool.h` implements the contract
box2d actually uses instead: split a range into chunks of at least
`minRange`, run them on `workerCount` threads with distinct worker indices,
and let the enqueueing thread take chunks while it waits, which is what
`enkiWaitForTaskSet` does and what makes a one-worker run create no threads.

The port needs no equivalent: it carries its own executor, so `WorkerCount`
alone is enough.

Because a silent fallback to one thread is the specific failure here, both
harnesses record `observed_workers` per scene: the C pool counts the worker
indices that ran a chunk, and the Go harness records the largest
`activeWorkerCount` any step ran on. `benchcmp` refuses to compare two scenes
whose observed counts differ. A count below the requested one is not an
error on its own: `step.go:102` routes a world with fewer awake bodies than
`serialBodyThreshold` to the caller, and box2d likewise runs a stage inline
when it is too small to split.

## The lane guard

`--want-lane` and `-bench.want-lane` exist because neither side announces a
wrong lane path on its own.

On the Go side the avx2 file carries `goexperiment.simd && go1.27 && amd64`.
Drop `GOEXPERIMENT=simd`, or build with the Go 1.26 toolchain, and the same
build tags quietly select the generic width-4 path. The build succeeds and
the tests pass; only the measurement is wrong.

On the C side `src/CMakeLists.txt` sets `BOX2D_AVX2` `PRIVATE` on the box2d
target, so a harness that includes `core.h` without it reports `sse2` while
linking an AVX2 library. `tools/cbench/CMakeLists.txt` passes the definition
to the harness for that reason.

Both harnesses abort when the runtime path is not the one asked for.

## The clock

`b2GetTicks` in `src/timer.c` reaches for `QueryPerformanceCounter` only
under `_MSC_VER`. Built with MinGW it falls through to a stub that returns
zero, so the reference cannot time itself in that configuration.

The Go monotonic clock on Windows quantises to the system tick, measured at
500 microseconds here: a 4.5 ms step reads with double-digit percent error
and a cheap step reads as zero.

Both harnesses therefore call `QueryPerformanceCounter` directly, at 100 ns
resolution, and record the clock name in the output. `benchcmp` refuses to
compare two files that did not use the same clock.

## Measured

AMD Ryzen 7 5800X3D, Windows/amd64, sixteen logical cores, GCC 13.2.0,
Go 1.27.0, three repeats. Solver ms is the sum of the per-step minima, which
is the number that compares the two solvers.

| workers | port over reference | with PGO | reference over its x1 | port over its x1 | reference efficiency | port efficiency |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 1.46x | 1.42x | 1.00x | 1.00x | 100% | 100% |
| 2 | 1.29x | 1.22x | 1.60x | 1.81x | 80% | 91% |
| 4 | 1.34x | 1.26x | 2.77x | 3.02x | 69% | 75% |
| 8 | 1.39x | 1.31x | 4.00x | 4.20x | 50% | 52% |

Every figure is a geometric mean over the seven scenes: ratios compose by
multiplication, so an arithmetic mean of them is biased upward. The PGO
column is the ratio composed with the profile's own gain, measured against a
`-pgo=off` build in the same sitting, because a profile and a no-profile
sweep taken hours apart compare the machine as much as the code.

The gap narrows with the thread count: the port keeps more of its
single-thread speed as threads are added, so parallel efficiency at eight
workers is 52 percent for the port against 50 percent for the reference. At
one worker the per-scene ratios run 1.34x to 1.62x; at eight they run 1.32x
to 1.53x, a tighter spread than the single-thread one.

Solver ms at eight workers:

| scene | bodies | c-avx2 ms | go-avx2 ms | ratio |
| --- | ---: | ---: | ---: | ---: |
| joint_grid | 10000 | 746 | 1072 | 1.44x |
| large_pyramid | 5051 | 392 | 552 | 1.41x |
| many_pyramids | 22001 | 470 | 720 | 1.53x |
| rain | 1 | 2437 | 3330 | 1.37x |
| smash | 9601 | 649 | 872 | 1.34x |
| spinner | 3040 | 1453 | 1922 | 1.32x |
| tumbler | 2026 | 625 | 843 | 1.35x |

`benchcmp` prints a repeat spread per scene and marks a scene noisy past five
percent. At three repeats on this machine `many_pyramids` runs 12 percent and
`large_pyramid` six; the other five sit under five. Treat a single scene
inside a noisy band as indicative and the geometric mean as the result.

An earlier sweep on a busier machine put the eight-worker ratio at 1.55x and
the port's efficiency below the reference's. Nothing in the port changed
between the two; the reference had been measured while the machine was under
load. A benchmark taken beside other work measures the other work.

Sixteen workers was measured and dropped: `many_pyramids` moved from 518 ms
to 531 ms, so the sweep stops at eight.

### What the scene hook measures

`hook_ms` is body creation and destruction rather than stepping, and only
`rain` does enough of it to matter: 102 ms in the port against 57 ms in the
reference at eight workers.

That column found a real defect. It first read 50671 ms, which is not a
language gap but an algorithm one, and the profile put 64 percent of the run
in `validateSolverSets`. `src/world.c:3262` compiles `b2ValidateSolverSets`
to an empty body unless `B2_VALIDATE` is set; the port was running the sweep
unconditionally from `createJoint` and three other mutation paths, once per
joint, over every body, joint and contact in the world. `rain` creates 250
joints per hook call against a world of 11001 bodies, so it paid the sweep
quadratically.

The port now gates those call sites behind the `dbox2d_validate` build tag,
matching the reference. The hook fell by a factor of 290 and the solver
figures did not move, because the step path never called it. The tables
above are from after the fix.

The lesson for reading this benchmark: a total that diverges from its solver
column is a scene-hook result, and it is worth chasing rather than
averaging in.


## Determinism across worker counts

The sweep records `final_hash` per scene, which makes one invariant testable
that a single-worker run cannot reach: whether a scene lands in the same
place when the worker count changes.

The port does, on all seven scenes, at one, two, four and eight workers. The
reference does on six; `rain` gives a different hash at each worker count,
reproducibly. That is the reference's own behaviour rather than a harness
artefact, and Box2D scopes its determinism guarantee to a fixed worker count.

The hashes are not comparable between the two implementations, by design.
They are also sensitive to the order the scenes run in, because
`shared/random.h` keeps one seed for the whole process, so a hash is
meaningful only against another run of the same scene list.

## The baseline instruction set

The reference compiles with `-mavx2` on the whole box2d target, so GCC may
vectorise code outside the wide contact solver. The Go side ran at
`GOAMD64=v1`, the baseline SSE2 target, which raised the question of whether
the gap is an artefact of that asymmetry. It is not. Rebuilding the port at
`GOAMD64=v3` moves the solver time by at most two percent:

| scene | v1 ms | v3 ms | v3/v1 | v1 over C | v3 over C |
| --- | ---: | ---: | ---: | ---: | ---: |
| tumbler | 3266 | 3202 | 0.98x | 1.52x | 1.49x |
| large_pyramid | 2952 | 2928 | 0.99x | 1.56x | 1.55x |
| many_pyramids | 4580 | 4566 | 1.00x | 1.63x | 1.63x |

The wide contact solver already runs on AVX2 through `simd/archsimd` at
either setting, so the baseline only reaches the scalar remainder, and that
remainder is not where the difference lives.
