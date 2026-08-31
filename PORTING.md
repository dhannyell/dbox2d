# Porting map

This file maps every upstream source file to its Go counterpart and records how
faithful that port is. It is the plan of record for the port and it grows as the
work lands. It never shrinks: a file that turns out to need a different tier
keeps its row and gains a note.

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

A sizing constant lands with the file that reads it, not before. A constant
with no consumer is dead weight that the compiler cannot check.

## The order of operations

Two rules follow from the arithmetic and apply to every file of the port.

**A negation goes before the product.** The reference writes `-s * x`, which
in C negates the operand. `Q.Mul` floors its result, and the floor of a
negative product is not the negative of the floor, so negating the product
instead shifts the result by one raw unit. Write `s.Neg().Mul(x)`.

**A reciprocal becomes a division.** See `D-006`. The reference computes
`1/d` once and multiplies by it; the port divides each term by `d`.

## Tiers

| Tier | Rule |
|---|---|
| **T0** | Mirror. Same decomposition, same names without the `b2` prefix, same order of operations. |
| **T1** | Same lines, `float` replaced by `Q`. |
| **T2** | Divergence. Fixed-point arithmetic or the Go language forbids the original line. Requires an entry in [DIVERGENCES.md](DIVERGENCES.md). |
| **T3** | Out of the current scope. Ports later, without changing its shape. |

## Stages

| Stage | Content |
|---|---|
| `foundation` | Constants, transforms, geometry, mass, world state, integration only. |
| `manifolds` | Narrowphase and contact bookkeeping. |
| `solver` | Soft Step, warm starting, relax, restitution, islands, coloring. |
| `broadphase` | Dynamic tree, pair finding, and the second executor. |
| `later` | Joints, time of impact, sensors, character mover, worker tasks. |

## What the inventory found

**1. The narrowphase is closed form, not iterative.** `manifold.c` computes
every manifold with support-point separation (`FindMaxSeparation`), polygon
clipping (`ClipPolygons`, `ClipSegments`) and closed-form segment distance.
The iterative simplex solver in `distance.c` reaches the narrowphase in exactly
one place: chain segment against polygon. Everything else in the narrowphase
ports at T0 or T1.

**2. The float tolerances in the narrowphase are few and local.**
`manifold.c` compares against `FLT_EPSILON` in eight places, all of them
guards against a degenerate segment or a zero-length span. In Q32.32 those
comparisons become exact tests against zero, which is a T2 entry each and
nothing more. The port loses no algorithm to arithmetic.

**3. The hot path has no trigonometry.** Neither `solver.c` nor
`contact_solver.c` nor `manifold.c` calls a trigonometric function. The only
transcendental in the narrowphase is `sqrtf`, and the fixed-point module
already provides an exact-floor square root. Rotations travel as a
sine-cosine pair, never as an angle.

**4. The state layout is three arrays of structs, split by role.**

- `b2Body` holds organizational data that the solver never touches.
- `b2BodySim` holds transform, mass and damping, used for integration.
- `b2BodyState` is a 32-byte hot struct: linear velocity, angular velocity,
  flags, delta position, delta rotation. Only awake bodies own one.

The arrays live inside solver sets: one static set, one awake set, one disabled
set, and one set per sleeping island. The split is by role, not by field, so a
faithful port keeps arrays of structs. Delta position and delta rotation exist
to reduce round-off far from the origin, which fixed point also wants.

**5. The upstream ships its own scalar contact solver.** `contact_solver.c`
contains two mirrored families over the same five stages, prepare, warm start,
solve, restitution and store impulses. The `Overflow` family is scalar and
solves the constraints that no color accepted. The `Task` family is the SIMD
one. The scalar family is a faithful in-repository reference for the whole
solver, so the port never has to invent the scalar form of the vector code.

## Progress

**Orders 1 to 6 have landed**: `constants.go`, `core.go`, `math.go`,
`aabb.go`, `id.go` and `id_pool.go`. Seven divergences came with them, D-001
to D-007. The notes below record what moved and what did not cross.

- The `b2AABB` type and its inline helpers sit in
  `include/box2d/math_functions.h` upstream. The port keeps them with the rest
  of the box code, in `aabb.go`.
- `b2MinInt`, `b2MaxInt`, `b2AbsInt`, `b2MinFloat`, `b2MaxFloat`, `b2AbsFloat`
  and `b2ClampFloat` do not cross. Go and the fixed-point module already give
  them. `ClampInt` crosses, because Go has no three-argument clamp.
- `b2CosSin` and `b2ComputeCosSin` do not cross. The fixed-point module owns
  the sine and the cosine.
- `b2GetIdBytes` does not cross. The size of an int changes with the platform,
  and a public number that changes with the platform is a trap in a
  deterministic library.
- `b2GetVersion` becomes `ReferenceVersion`. It names the Box2D release that
  this package ports; the module carries its own version.
- The tree node flags of `constants.h` wait for `dynamic_tree.go`, order 27.
- `b2Lerp` keeps the upstream weighted form, which returns each end exactly.
  The fixed-point module interpolates by a scaled difference, so the two
  round differently and the port does not delegate.
- `b2Perimeter` and `b2EnlargeAABB` live in `src/aabb.h`, so they stay
  unexported. Their consumer is the dynamic tree, order 27.
- `B2_NULL_INDEX`, `B2_MAX_WORKERS`, `B2_GRAPH_COLOR_COUNT` and
  `B2_MAX_WORLDS` wait for the files that read them: the body storage, the
  worker pool, the constraint graph and the world registry.
- The module depends on the standard library and on the fixed-point module,
  and on nothing else. `math/bits` and `sync/atomic` arrive with the
  broadphase.

**Orders 7 to 9 have landed**: `collision.go`, `hull.go`, `geometry.go` and
the ray cast of `aabb.go`. Two divergences came with them, D-008 and D-009,
and D-003 and D-006 grew new entries.

- The distance, the time of impact and the character mover own their types.
  `SegmentDistanceResult`, `SimplexCache`, `DistanceInput`, `DistanceOutput`,
  `Simplex`, `ShapeCastPairInput`, `Sweep`, `TOIInput`, `TOIOutput`,
  `PlaneResult`, `CollisionPlane` and `PlaneSolverResult` wait for them.
- `b2ShapeCastCircle`, `b2ShapeCastCapsule`, `b2ShapeCastSegment`,
  `b2ShapeCastPolygon`, `b2PointInPolygon` and the four `b2CollideMoverAnd`
  functions do not cross yet. Each one calls `b2ShapeDistance` or
  `b2ShapeCast`, which are the iterative geometry of `distance.c`.
- `RayCastPolygon` ports the closed-form branch, which needs a zero radius.
  A rounded polygon panics until the shape cast lands.
- `B2_MAX_POLYGON_VERTICES` becomes `MaxPolygonVertices`. A shape keeps the
  upstream layout: a fixed array of that size beside a count.
- `B2_HUGE` lands in `constants.go` with order 9, because `IsValidRay` in
  `geometry.go` is its first reader. It bounds the ray fraction, so it
  stays unexported like the rest of `src/constants.h`.
- The point arrays that the reference passes as a pointer and a count become
  slices. The fixed arrays inside `Hull`, `Polygon` and `ShapeProxy` stay.

## The map

`Order` is the port sequence. A dash means the file waits for a later stage.

| Upstream | Go | Tier | Stage | Order | Notes |
|---|---|---|---|---|---|
| `src/constants.h` | `constants.go` | T1 | foundation | 1 | Each constant keeps the upstream value in a comment. `B2_LINEAR_SLOP` is 0.005 m; the speculative distance is four slops. |
| `include/box2d/base.h`, `src/core.h`, `src/core.c` | `core.go` | T1/T2 | foundation | 2 | Platform, SIMD and profiler macros do not cross. Allocation hooks become Go allocation. |
| `include/box2d/math_functions.h`, `src/math_functions.c` | `math.go` | T1/T2 | foundation | 3 | Vector and rotation come from the fixed-point module. Only the shapes that module lacks stay here: `Transform`, sweeps, validation. |
| `src/aabb.h` | `aabb.go` | T0 | foundation | 4 | Union, overlap and contains. |
| `include/box2d/id.h` | `id.go` | T0 | foundation | 5 | Index plus generation handles. |
| `src/id_pool.h`, `src/id_pool.c` | `id_pool.go` | T0 | foundation | 6 | Free list over a monotonic index. |
| `include/box2d/collision.h` | `collision.go` | T0 | foundation | 7 | Shape structs, manifold structs, cast input and output. |
| `src/aabb.c` | `aabb.go` | T0/T2 | foundation | 7 | AABB ray cast, unexported: `src/aabb.h` declares it, `include/box2d/` does not. `IsValidAABB` landed with order 4. See D-006, D-008 and D-009. |
| `src/hull.c` | `hull.go` | T0/T2 | foundation | 8 | Recursive quickhull. Its tolerances are multiples of the linear slop, so only the `FLT_MAX` seed diverged. See D-009. |
| `src/geometry.c` | `geometry.go` | T0/T1/T2 | foundation | 9 | Shape constructors, mass data, AABB per shape, point tests, ray casts. The shape casts and the mover collisions wait for order 17. |
| `include/box2d/types.h`, `src/types.c` | `types.go` | T1 | foundation | 10 | Definition structs and their defaults. |
| `src/body.h`, `src/body.c` | `body.go` | T0 | foundation | 11 | `Body`, `BodySim`, `BodyState`. Layout preserved. |
| `src/shape.h`, `src/shape.c` | `shape.go` | T0 | foundation | 12 | Shape storage, proxy creation, shape queries. |
| `src/solver_set.h`, `src/solver_set.c` | `solver_set.go` | T0 | foundation | 13 | Static, awake, disabled and sleeping sets; body transfer between them. |
| `src/world.h`, `src/world.c` | `world.go` | T0 | foundation | 14 | Split across stages. The foundation takes creation, storage and the step skeleton. |
| `src/array.h`, `src/array.c` | slices | T2 | foundation | 15 | The macro-generated array template becomes a Go slice. The upstream growth policy stays. |
| `src/arena_allocator.h`, `src/arena_allocator.c` | `arena.go` | T1 | foundation | 16 | Per-step scratch. It is how the step allocates nothing. |
| `src/distance.c` (segment distance, proxies) | `distance.go` | T0 | manifolds | 17 | Closed-form part only. |
| `src/manifold.c` | `manifold.go` | T0/T1 | manifolds | 18 | Eight `FLT_EPSILON` guards become exact zero tests, one T2 entry each. |
| `src/contact.h`, `src/contact.c` | `contact.go` | T0 | manifolds | 19 | Contact bookkeeping and the collide dispatch table. |
| `src/table.h`, `src/table.c` | `table.go` | T0 | manifolds | 20 | Open-addressing set of contact pairs. |
| `src/solver.h`, `src/solver.c` | `solver.go` | T0/T1 | solver | 21 | Nine ordered stages, from prepare joints to store impulses. `MakeSoft` is pure arithmetic. |
| `src/contact_solver.h`, `src/contact_solver.c` | `contact_solver.go` | T0/T1 | solver | 22 | Port the `Overflow` family. The `Task` family is T2 until the second executor exists. |
| `src/island.h`, `src/island.c` | `island.go` | T0 | solver | 23 | Island linking, merging, splitting, sleeping. |
| `src/constraint_graph.h`, `src/constraint_graph.c` | `constraint_graph.go` | T0 | solver | 24 | Twelve colors plus the overflow set. The color schedule is the parallel contract. |
| `src/bitset.h`, `src/bitset.c` | `bitset.go` | T0 | broadphase | 25 | Backs island splitting and pair finding. |
| `src/ctz.h` | `math/bits` | T2 | broadphase | 26 | The standard library replaces the compiler intrinsics. |
| `src/dynamic_tree.c` | `dynamic_tree.go` | T0 | broadphase | 27 | Fattened AABBs, surface-area heuristic, rotation rebalance. |
| `src/broad_phase.h`, `src/broad_phase.c` | `broad_phase.go` | T0 | broadphase | 28 | Pair output is sorted by integer id, so any equivalent tree gives the same world. |
| `src/atomic.h` | `sync/atomic` | T2 | broadphase | 29 | Needed only when a second executor exists. |
| `include/box2d/box2d.h` | public API | T0 | all stages | — | The public surface arrives file by file with its owner. |
| `src/joint.h`, `src/joint.c` | `joint.go` | T3 | later | — | Eight joint types follow it. |
| `src/distance_joint.c`, `src/motor_joint.c`, `src/mouse_joint.c`, `src/prismatic_joint.c`, `src/revolute_joint.c`, `src/weld_joint.c`, `src/wheel_joint.c` | one file each | T3 | later | — | Ports after the solver is proven. |
| `src/distance.c` (simplex solver, shape cast, time of impact) | `gjk.go`, `toi.go` | T3 | later | — | The only iterative geometry in the library. Its stopping criteria are the largest T2 risk of the whole port. |
| `src/sensor.h`, `src/sensor.c` | `sensor.go` | T3 | later | — | Sensor overlap events. |
| `src/mover.c` | `mover.go` | T3 | later | — | Character movement helpers, built on shape casts. |
| `src/timer.c` | dropped | T2 | — | — | Platform timers serve profiling. Timing never enters a deterministic result. |
| `src/CMakeLists.txt`, `src/box2d.natvis` | none | — | — | — | Build system and debugger visualizers do not apply. |

## Coverage

Every file under `src/` and `include/box2d/` of the reference has a row. The
59 entries of `src/` and the 6 headers of `include/box2d/` are accounted for.
