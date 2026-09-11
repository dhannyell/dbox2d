# Porting and audit map

This is the contributor's map from Box2D v3.1.1 to `dbox2d`. It answers three
questions: where an upstream file lives in Go, how closely it was ported, and
which differences are intentional.

For a quick overview, start with [the root README](README.md). Use this file
when reviewing the implementation or adding a ported subsystem. The map is
append-only: a changed decision keeps its row and gains a note.

## How to read this file

- **T0** mirrors the upstream structure, names, and operation order.
- **T1** keeps the same algorithm while replacing the scalar with the active
  mode's `Q` type.
- **T2** is a deliberate language or arithmetic divergence and must have an
  entry in [DIVERGENCES.md](DIVERGENCES.md).
- **T3** is outside the current port scope.

The public solver stays in one Go package because its subsystems share
unexported state. The samples and conformance tools are separate modules.

## Provenance

- Upstream: <https://github.com/erincatto/box2d>, tag `v3.1.1`
- Commit: `8c661469c9507d3ad6fbd2fea3f1aa71669c2fe3`, 2025-06-03
- Local branch: `reference/box2d-v3.1.1`, with the upstream history

Read the reference with `git show`, `git log` and `git blame` against that
branch. Blame is part of the method: when an upstream line looks strange, the
commit that introduced it usually names the bug it answers.

## Layout

The reference keeps its 59 sources flat in `src/`. The port mirrors that: one
upstream file becomes one Go file at the module root, and the whole solver is
one package. Go binds a package to a directory, so a subdirectory would split
the solver into several packages and force its internals to become public. The
flat layout keeps the internals internal and keeps each file beside the
upstream file it answers to.

A name is exported when the reference exports it from `include/box2d/`, plus
the tolerances of `constants.h`, which content authoring needs. Everything
else that the reference keeps under `src/` stays unexported here.

The scalar has two owners: `scalar_float.go` is the default float32 mode, and
`scalar_fixed.go` is the `dbox2d_fixed` Q32.32 mode. Both declare `Q`, `Vec2`
and `Rot` and the constructors that build a scalar. Every other file calls those
constructors; only the fixed SIMD lanes and the tests that read the raw format
still import the fixed module. A second scalar mode is a second file under
another build tag, not a sweep of the solver.

A sizing constant lands with the file that reads it, not before. A constant
with no consumer is dead weight that the compiler cannot check.

The reference's `samples/` tree is a second module, `samples/`, with its
own `go.mod`. It ports `sample.h`, `sample.cpp`, `draw.cpp` and the scene
files, and replaces OpenGL and Dear ImGui with WebGPU and a vendored
microui. See [samples/README.md](samples/README.md). Its checksum tests
pin the scenes the way the witness pins the solver.

## The order of operations

Two rules follow from the arithmetic and apply to every file of the port.

**A negation goes before the product.** The reference writes `-s * x`, which
in C negates the operand. `Q.Mul` floors its result, and the floor of a
negative product is not the negative of the floor, so negating the product
instead shifts the result by one raw unit. Write `s.Neg().Mul(x)`.

**A reciprocal becomes a division.** See `D-006`. The reference computes
`1/d` once and multiplies by it; the port divides each term by `d`.

## Stages

| Stage | Content |
|---|---|
| `foundation` | Constants, transforms, geometry, mass, world state, integration only. |
| `manifolds` | Narrowphase and contact bookkeeping. |
| `solver` | Soft Step, warm starting, relax, restitution, islands, coloring. |
| `broadphase` | Dynamic tree, pair finding, and the executor. |
| `joints` | The seven joint solvers plus the filter joint, their solver stages, islands and colors. |
| `surface` | The rest of the public surface: accessors, chains, sensors, the mover, explosions and the closure-based callbacks. |

## The map

`Order` is the port sequence. A dash means the file waits for a later stage.

| Upstream | Go | Tier | Stage | Order | Notes |
|---|---|---|---|---|---|
| `src/constants.h` | `constants.go` | T1 | foundation | 1 | Each constant keeps the upstream value in a comment. `B2_LINEAR_SLOP` is 0.005 m; the speculative distance is four slops. |
| `include/box2d/base.h`, `src/core.h`, `src/core.c` | `core.go` | T1/T2 | foundation | 2 | Platform, SIMD and profiler macros do not cross. Allocation hooks become Go allocation. |
| `include/box2d/math_functions.h`, `src/math_functions.c` | `math.go` | T1/T2 | foundation | 3 | Vector and rotation use the per-mode scalar implementations: fixed-module functions in fixed mode, and the reference Bhaskara cosine/sine approximation plus atan polynomial in float mode. Only the shapes those implementations lack stay here: `Transform`, sweeps, validation. |
| `src/aabb.h` | `aabb.go` | T0 | foundation | 4 | Union, overlap and contains. |
| `include/box2d/id.h` | `id.go` | T0 | foundation | 5 | Index plus generation handles. |
| `src/id_pool.h`, `src/id_pool.c` | `id_pool.go` | T0 | foundation | 6 | Free list over a monotonic index. |
| `include/box2d/collision.h` | `collision.go` | T0 | foundation | 7 | Shape structs, manifold structs, cast input and output. |
| `src/aabb.c` | `aabb.go` | T0/T2 | foundation | 7 | AABB ray cast, unexported: `src/aabb.h` declares it, `include/box2d/` does not. `IsValidAABB` landed with order 4. See D-006, D-008 and D-009. |
| `src/hull.c` | `hull.go` | T0/T2 | foundation | 8 | Recursive quickhull. Its tolerances are multiples of the linear slop, so only the `FLT_MAX` seed diverged. See D-009. |
| `src/geometry.c` | `geometry.go` | T0/T1/T2 | foundation | 9 | Shape constructors, mass data, AABB per shape, point tests, ray casts. The shape casts landed with order 32; the mover collisions landed with order 34. |
| `include/box2d/types.h`, `src/types.c` | `types.go` | T1 | foundation | 10 | Definition structs and their defaults. |
| `src/body.h`, `src/body.c` | `body.go` | T0/T2 | foundation | 11 | `body`, `bodySim`, `bodyState`, unexported: `src/body.h` declares them. Layout preserved. The body state keeps the angular velocity in radians per second; see D-004. The island hooks landed with order 25; the body events landed with order 28; the proxy destroy and the body collision rule landed with order 30; `makeSweep` landed with order 32. |
| `src/shape.h`, `src/shape.c` | `shape.go` | T0/T2 | foundation | 12 | Shape storage and the mass, AABB, centroid and extent dispatchers. The proxies and the filter rules landed with order 30; the ray cast dispatcher landed with the public queries. The shape cast dispatcher and the distance proxy landed with order 32. The sensors and chains landed with order 34; see D-003, D-010 and D-012. |
| `src/solver_set.h`, `src/solver_set.c` | `solver_set.go` | T0 | foundation | 13 | Static, awake, disabled and sleeping sets; body transfer, wake, sleep and set merge. The joint arrays and transfers landed with order 33. |
| `src/world.h`, `src/world.c` | `world.go` | T0/T2 | foundation | 14 | Split across stages. The foundation takes the registry, creation, destruction, the validity checks and the trimmed set validation. `b2World_Step` landed with order 16 in `step.go`; the events landed with order 28; the broadphase and the enlarged body bit set landed with order 30; `OverlapAABB` and `CastRay` landed with the public queries. `OverlapShape`, `CastShape`, `CastMover` and `CastRayClosest` landed with order 32. `Explode`, `SetCustomFilterCallback`, `SetPreSolveCallback`, `SetRestitutionCallback`, `CollideMover`, debug draw, `GetProfile` and `DumpMemoryStats` landed. See D-004, D-006, D-012, D-014 and D-015. |
| `src/array.h`, `src/array.c` | `array.go` | T2 | foundation | 15 | The macro-generated array template becomes a Go slice; `removeSwap` keeps the swap-remove contract. Capacity follows the Go runtime and never enters a result. See D-010. |
| `src/world.c` (`b2World_Step`), `src/solver.h` (`b2StepContext`) | `step.go` | T1/T2 | foundation | 16 | The step surface: validation, the context, the sub-step split, the locked flag. Assertions become panics per D-003. The softness setup landed with order 24; the collide block and the events landed with order 28; the pair update and the tree rebuild landed with order 30; the bullet buffer landed with order 32. |
| — | `checksum.go` | T2 | foundation | 17 | Port-only determinism witness over the complete canonical world state, commutative over bodies and shapes. See D-011. |
| `src/arena_allocator.h`, `src/arena_allocator.c` | `arena.go` | T1 | foundation | 18 | Per-step scratch. It is how the step allocates nothing. |
| `src/distance.c` (segment distance, proxies) | `distance.go` | T0 | manifolds | 19 | Closed-form part. The iterative part is order 32. |
| `src/manifold.c` | `manifold.go` | T0/T1 | manifolds | 20 | Nine `FLT_EPSILON` sites become exact zero tests, one T2 entry each. The chain segment colliders landed with order 32. |
| `src/contact.h`, `src/contact.c` | `contact.go` | T0 | manifolds | 21 | Contact bookkeeping and the collide dispatch table. The island and graph branches landed with orders 25 and 26. |
| `src/table.h`, `src/table.c` | `table.go` | T0 | manifolds | 22 | Open-addressing set of contact pairs. |
| `src/solver.h`, `src/solver.c` | `solver.go` | T0/T1/T2 | solver | 23 | Nine ordered stages, from prepare joints to store impulses. `makeSoft` landed with order 24. The integration tasks and the body finalize landed with order 16; the stage script with the per-color contact stages, the island split and the sleep tail landed with order 23; the stage blocks and the worker pool landed with D-016; the enlarged body bits and the broadphase refit landed with order 30; the continuous stage landed with order 32; the joint stages landed with order 33; see D-004 and D-006. Under `dbox2d_simd`, the stage table packs each color's contacts into lanes and pads to the lane width; see D-019. |
| `src/contact_solver.h`, `src/contact_solver.c` | `contact_solver.go` | T0/T1/T2 | solver | 24 | The scalar stages landed and serve every color; see D-004 and D-006. The SIMD `Task` family landed under `dbox2d_simd` in both modes; see D-019 and D-020. |
| `src/island.h`, `src/island.c` | `island.go` | T0 | solver | 25 | Island linking, merging and splitting landed; the wake calls and the sleep path landed when order 13 completed. The joint lists landed with order 33. |
| `src/constraint_graph.h`, `src/constraint_graph.c` | `constraint_graph.go` | T0 | solver | 26 | Eleven colors plus the overflow color landed. The color schedule is the parallel contract. The joint functions landed with order 33. |
| `src/bitset.h`, `src/bitset.c` | `bitset.go` | T0 | broadphase | 27 | Set, clear, test, grow and union landed. Backs the constraint graph and the contact state of the step. |
| `src/ctz.h` | `math/bits` | T2 | broadphase | 28 | The standard library replaces the compiler intrinsics. Landed with the collide block in `step.go`. |
| `src/dynamic_tree.c` | `dynamic_tree.go` | T0/T2 | broadphase | 29 | Landed. Fattened AABBs, surface-area heuristic, rotation rebalance, box query, ray cast, shape cast, partial rebuild. See D-009 and D-014. |
| `src/broad_phase.h`, `src/broad_phase.c` | `broad_phase.go` | T0/T2 | broadphase | 30 | Landed. Three trees, the move buffer, the pair query and the pair set. The pair list of each moved proxy is sorted by shape id, so any equivalent tree gives the same world; the `dbox2d_upstream_pairs` tag keeps the reference order. See D-010 and D-013. |
| `src/atomic.h` | `sync/atomic` | T1 | broadphase | 31 | Landed with the executor: the sync bits and the block indices of the solver script, the completion counts and the pool's generation. The pair index of the broadphase is the length of each worker's pair slice; see D-016. |
| `include/box2d/box2d.h` | public API | T0/T2 | all stages | 34 | Landed. The whole 3.1.1 surface is ported; see the surface note above, D-014 and D-015. |
| `src/joint.h`, `src/joint.c` | `joint.go` | T0/T2 | joints | 33 | Landed. Types, definitions, storage, creation, destruction, the island and graph hooks, the set transfers and the prepare, warm start and solve dispatch. Accessors landed with order 34; debug draw and dump do not cross. See D-003, D-004 and D-006. |
| `src/distance_joint.c`, `src/motor_joint.c`, `src/mouse_joint.c`, `src/prismatic_joint.c`, `src/revolute_joint.c`, `src/weld_joint.c`, `src/wheel_joint.c` | one file each | T0/T2 | joints | 33 | Landed. Force and torque reports, prepare, warm start and solve of each type; the filter joint has no solver. Accessors landed with order 34; debug draw and dump do not cross. See D-004, D-006 and D-009. |
| `src/distance.c` (simplex solver, shape cast, time of impact), `src/solver.c` (continuous stage) | `distance.go`, `solver.go` | T0/T2 | manifolds | 32 | Landed. The only iterative geometry in the library. Each stopping criterion keeps its form; the tests pin the iteration bounds, a float64 mirror and a bit witness. |
| `src/sensor.h`, `src/sensor.c` | `sensor.go` | T0/T2 | surface | 34 | Landed. Double-buffered overlap sets, begin and end touch events, `WorldId.GetSensorEvents`. The overlap test is an exact zero distance; see D-012. |
| `src/mover.c` | `mover.go` | T0/T2 | surface | 34 | Landed. `SolvePlanes`, `ClipVector`, the four `CollideMoverAnd*` functions and `WorldId.CollideMover`. The rigid push limit is `Huge`, not `FLT_MAX`; see D-009 and D-014. |
| `src/timer.c` | `time` package (`step.go`) | T2 | — | — | The standard clock replaces the platform timers; the profile is its only consumer. Timing never enters a deterministic result. |
| `src/CMakeLists.txt`, `src/box2d.natvis` | none | — | — | — | Build system and debugger visualizers do not apply. |

### Port-only files

The reference selects its scalar type, its SIMD width and its task system
with preprocessor conditions inside the files above. Go selects them with
build tags, and a build tag needs a file of its own. These files have no
upstream counterpart; each one carries the tag that selects it.

| Go | Tag | Notes |
|---|---|---|
| `scalar_fixed.go`, `scalar_float.go` | `dbox2d_fixed`, its negation | The scalar layer of each mode: `Q`, `Vec2`, `Rot`, the constructors, the contact grid types and the Q32 contact hooks. See D-017 and D-020. |
| `contact_solver_q32.go` | `dbox2d_fixed` | The contact stages over Q32.32 for the contacts outside the lane window. `go generate` derives it from `contact_solver.go` through `internal/q32gen` and `tools/q32gen`. See D-020. |
| `contact_solver_simd.go` | `dbox2d_simd` | The SIMD `Task` family with its dispatch and its scratch layout. See D-019. |
| `simd_off.go` | `!dbox2d_simd` | The scalar family behind the same hooks. |
| `simd_lane_amd64.go`, `simd_lane_arm64.go`, `simd_lane_wasm.go`, `simd_lane_generic.go`, `simd_lane_float.go`, `simd_lane_float_archsimd.go`, `simd_lane_gather_generic.go` | `dbox2d_simd && !dbox2d_fixed` plus the target | The float lanes: one width and vector type per target, the shared lane algebra, and the body gather of each path. See D-019. |
| `simd_lane_fixed.go` | `dbox2d_simd && dbox2d_fixed` | The fixed lanes and their body gather, over the fixed module. See D-019. |
| `executor.go`, `executor_wasm.go`, `executor_pool.go`, `spin_asm.go`, `spin_stub.go` | target | The worker pool and its spin wait; the reference leaves the task system to the caller. See D-016. |

## Conformance

The conformance harness lives in `tools/conformance/` and its frozen traces
live in `testdata/conformance/`. The traces cover collision functions and
benchmark scenes in both scalar modes.

Regenerate the traces with the exact commands in `testdata/conformance/SOURCE.md`;
they build the reference with one worker, without SIMD and without FMA.

The test reads every trace in fixed and float mode. Function traces use ULP
budgets in float mode and absolute budgets in fixed mode. Scene step 1 uses a
per-scene absolute budget in each mode, while hashes and later dumps are logged.
See DIVERGENCES.md D-018 for the exact budgets and known differences.

Regenerate only when the reference checkout or the trace format changes. Keep
`-ffp-contract=off` and `BOX2D_DISABLE_SIMD=ON` in the reference build.

## Coverage

Every file under `src/` and `include/box2d/` of the reference has a row. The
59 entries of `src/` and the 6 headers of `include/box2d/` are accounted for.
