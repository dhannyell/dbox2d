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
`manifold.c` compares against `FLT_EPSILON` in nine places — eight literal
uses and one through the `epsSqr` variable — all of them guards against a
degenerate segment or a zero-length span. In Q32.32 those comparisons become
exact tests against zero, which is a T2 entry each and nothing more. The
port loses no algorithm to arithmetic.

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
- `B2_MAX_WORKERS` and `B2_GRAPH_COLOR_COUNT` wait for the files that read
  them: the worker pool and the constraint graph. `B2_NULL_INDEX` and
  `B2_MAX_WORLDS` landed with order 10.
- The module depends on the standard library and on the fixed-point module,
  and on nothing else. `math/bits` and `sync/atomic` arrive with the
  broadphase.

**Orders 7 to 9 have landed**: `collision.go`, `hull.go`, `geometry.go` and
the AABB ray cast in `aabb.go`. Two divergences came with them, D-008 and
D-009, and D-003 and D-006 grew new entries.

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

**Orders 10 to 15 have landed**: `types.go`, `body.go`, `shape.go`,
`solver_set.go`, `world.go` and `array.go`, plus `nullIndex`, `maxWorlds`,
`secretCookie` and `checkDef` in `core.go`. One language divergence came with
them, D-010, and D-004 grew a `body.go` entry. The state bookkeeping itself
adds no arithmetic divergence.

- A function on a handle becomes a method: `b2Body_GetPosition` becomes
  `Position` on `BodyId`. The receiver replaces the first parameter and the
  `Get` prefix goes away, as Go style asks.
- A definition struct keeps `internalValue` unexported, so only its
  `Default` function can satisfy `checkDef`.
- `b2WorldDef` drops the task-system fields and the mixing callbacks. The
  task system does not cross; the callbacks arrive with the contact solver.
  The debug draw and `b2RayResult` wait with their consumers.
- The union inside `b2Shape` becomes five plain fields, one per geometry,
  because Go has no union. The `type` field of `b2Body` becomes `bodyType`,
  because `type` is a Go keyword.
- The body name stays a 32-byte buffer with the 31-byte copy of the
  reference.
- An angular velocity is in turns per second, per D-004: `AngularVelocity`
  in `BodyDef` and the body state. `updateBodyMassData` scales by one turn
  before its cross product, because the cross needs radians per second.
- Where the reference creates a broad-phase proxy, the port calls
  `updateShapeAABBs` and leaves `proxyKey` null, so the stored bounds stay
  faithful. The proxy arrives with order 30.
- A sensor shape panics until the sensor module lands. The chains, joints,
  contacts, islands and events defer the same way: `b2CreateBody`,
  `b2DestroyBody` and `b2DestroyShapeInternal` cross with their deferred
  blocks marked in comments.
- `b2ValidateSolverSets` crosses trimmed to the bodies and the sets, and
  only the tests call it. The reference compiles it out of release builds.
- `b2WakeBody`, `b2ShouldBodiesCollide`, `b2MakeSweep`, the wake, sleep and
  merge of solver sets and the step, query and cast surface of `world.c`
  wait for their stages.

**Orders 16 and 17 have landed**: `step.go`, the integration skeleton in
`solver.go` and `checksum.go`. D-011 came with them, D-003 now covers the
assertions retained by the step, and D-004 and D-006 grew `solver.go` entries.

- `Step` mirrors `b2World_Step`. The event, broad-phase, collision, softness
  and sensor blocks are deferred with comments at their points. A zero time
  step returns early, as the reference does.
- The sub-step keeps the stage order of the reference: integrate velocities,
  the deferred constraint stages, integrate positions. The body finalize
  runs once, after the loop.
- The damping factor becomes a division per D-006. The torque delta and the
  arc speed of the sleep test scale by one turn per D-004.
- The finalize refreshes the fat bounds directly. While continuous collision
  is deferred, fast bodies also take this discrete path so their previous
  transforms and bounds remain coherent. The `enlargedAABB` flag and the bit
  sets stay with the broad-phase.
- `Checksum` covers the deterministic world configuration and all canonical
  body and shape state. Per-body and per-shape sums make equivalent worlds
  independent of internal ids and creation order.
- One worker replaces the task system. The stage order is the contract that
  a second executor must keep.

**Order 18 has landed**: `arena.go`.

- The arena keeps the 32-byte size rounding of the reference, so the
  accounting numbers match even though Go has no SIMD alignment need.
- The heap fallback becomes a Go allocation. Arena slices carry a capped
  capacity, so an append by the caller cannot grow over the next entry.
- Destroy clears the backing slices and accounting explicitly; the three
  getters expose the same capacity, current allocation and peak values as the
  reference. The world adopts the arena when a stage allocates through it.

**Order 19 has landed**: the closed-form part of `distance.go`, and
`SegmentDistanceResult` in `collision.go`. D-012 came with it.

- `SegmentDistance` keeps the branch structure of the reference: the
  degeneracy dispatch, the parallel case and the two do-over clamps.
  The epsilon guard becomes an exact zero test per D-012.
- `MakeProxy` takes a slice in place of the pointer and count pair.
- `b2MakeOffsetProxy`, `b2GetSweepTransform` and the whole simplex
  apparatus stay with their consumers: the shape casts, the sweeps and
  the iterative distance solver.

**Order 20 has landed**: `manifold.go` — the circle and capsule colliders,
then the polygon colliders: `makeCapsule`, `clipPolygons`,
`findMaxSeparation`, `CollidePolygons`, `CollidePolygonAndCapsule`,
`CollideSegmentAndPolygon` and `clipSegments`.

- The branch structure of the reference stays intact. The epsilon guards
  become exact zero tests per D-012; the length asserts become panics per
  D-003. In the two clippers the guarded lerp span is provably positive in
  Q, so the guard is structural only.
- `makeId` mirrors `B2_MAKE_ID`, so the point ids keep the warm-starting
  contract across frames.
- `CollidePolygons` ports the compiled `#if 1` branch of the reference; the
  dead `#else` block does not enter.
- `clipSegments` has no caller yet: the chain-segment colliders that use it
  also need GJK and wait with the distance solver.

**Order 22 has landed**: `table.go`, ahead of order 21 because the contact
bookkeeping needs the pair set.

- The set keeps the open-addressing scheme, the Murmur finalizer and the
  probe chain repair of the reference. The capacity stays a power of two;
  the allocation always matches the rounded capacity.
- The set only answers membership, so its internal slot order never enters
  a simulation result.
- Destroy releases the item slice, clear preserves it for reuse, and the byte
  getter reports its actual Go storage size.

**Order 21 has landed**: `contact.go` — the cold `contact`, the warm
`contactSim`, the collide dispatch table, `createContact`, `destroyContact`,
`getContactSim`, `updateContact` and `computeManifold`. The world gains the
contact storage and the mixing callbacks; the solver sets gain the contact
sim arrays; `DestroyBody` and `DestroyShape` destroy attached contacts before
releasing their owners.

- The lazy register flag becomes a package `init`, which runs once and in a
  deterministic order.
- The chain segment pairs against capsule and polygon stay out of the
  register until the iterative distance solver lands; their nil entries make
  `createContact` skip the pair.
- The pair set lives on the world for now. The reference hosts it on the
  broadphase; it moves there when the broadphase lands.
- The end touch event, the pre-solve callback, the wake on destroy, the
  island and constraint graph branches and the inverse mass copies of the sim
  wait for their stages. `islandId` and `colorIndex` stay `nullIndex` until
  then.
- The speculative two-point reduction of the reference tests point zero in
  both branches, so only the first branch can fire; the port keeps the live
  branch only.
- `b2SimplexCache` enters as a data-only struct. The distance solver that
  fills it comes later; until then every dispatch runs with an empty cache.

**Orders 25 and 27 have landed**: `island.go` and `bitset.go`. The world
gains the arena, the island storage, the island id pool and the split
candidate; the solver sets gain the island sim arrays; a body enters its own
island at creation and leaves it at destruction; a destroyed contact unlinks
from its island first.

- The union-find keeps the rule of the reference: the root of body A is
  always the parent. The awake merge walks the islands in reverse, so the
  body order inside a merged island follows that walk.
- The split takes its stack and its seed ids from the arena through a typed
  view over one arena item. The reference casts the bytes in place; the port
  uses `unsafe.Slice` so the arena accounting matches.
- `validateIsland` exists, but only the tests call it, as with the set
  validation.
- The joint lists of the island, the wake calls of `linkContact` and the
  sleep path wait for their stages. The `ctz.h` intrinsics of order 28 are
  not needed yet: the bit sets only set, clear, test and union in this
  stage.

**Order 26 has landed**: `constraint_graph.go`. The world gains the graph;
`getContactSim` and `destroyContact` gain their graph branches; the contact
sim gains the inverse mass copies that the graph fills.

- The color rule follows the reference: a dynamic pair scans colors 0 to
  10, a pair with a static body scans 1 to 10, and the rest go to the
  overflow color 11, which keeps no body set. The force-overflow switch of
  the reference stays a constant set to false.
- The joint color assignment and the joint graph functions wait for the
  joints.

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
| `src/geometry.c` | `geometry.go` | T0/T1/T2 | foundation | 9 | Shape constructors, mass data, AABB per shape, point tests, ray casts. The shape casts and the mover collisions wait for order 19. |
| `include/box2d/types.h`, `src/types.c` | `types.go` | T1 | foundation | 10 | Definition structs and their defaults. |
| `src/body.h`, `src/body.c` | `body.go` | T0/T2 | foundation | 11 | `body`, `bodySim`, `bodyState`, unexported: `src/body.h` declares them. Layout preserved. The mass update scales the angular velocity by one turn; see D-004. The island hooks landed with order 25; the body events wait for the solver. |
| `src/shape.h`, `src/shape.c` | `shape.go` | T0 | foundation | 12 | Shape storage and the mass, AABB, centroid and extent dispatchers. The proxies, sensors, chains and cast queries wait for their stages. |
| `src/solver_set.h`, `src/solver_set.c` | `solver_set.go` | T0 | foundation | 13 | Static, awake, disabled and sleeping sets; body transfer between them. The contact arrays landed with order 21 and the island arrays with order 25; the joint arrays and the wake and sleep paths wait. |
| `src/world.h`, `src/world.c` | `world.go` | T0 | foundation | 14 | Split across stages. The foundation takes the registry, creation, destruction, the validity checks and the trimmed set validation. `b2World_Step` landed with order 16 in `step.go`; the query and cast surface waits. |
| `src/array.h`, `src/array.c` | `array.go` | T2 | foundation | 15 | The macro-generated array template becomes a Go slice; `removeSwap` keeps the swap-remove contract. Capacity follows the Go runtime and never enters a result. See D-010. |
| `src/world.c` (`b2World_Step`), `src/solver.h` (`b2StepContext`) | `step.go` | T1/T2 | foundation | 16 | The step surface: validation, the context, the sub-step split, the locked flag. Assertions become panics per D-003. The softness setup, the events and the collision blocks wait for their stages. |
| — | `checksum.go` | T2 | foundation | 17 | Port-only determinism witness over the complete canonical world state, commutative over bodies and shapes. See D-011. |
| `src/arena_allocator.h`, `src/arena_allocator.c` | `arena.go` | T1 | foundation | 18 | Per-step scratch. It is how the step allocates nothing. |
| `src/distance.c` (segment distance, proxies) | `distance.go` | T0 | manifolds | 19 | Closed-form part only. |
| `src/manifold.c` | `manifold.go` | T0/T1 | manifolds | 20 | Nine `FLT_EPSILON` sites become exact zero tests, one T2 entry each. |
| `src/contact.h`, `src/contact.c` | `contact.go` | T0 | manifolds | 21 | Contact bookkeeping and the collide dispatch table. The island and graph branches landed with orders 25 and 26. |
| `src/table.h`, `src/table.c` | `table.go` | T0 | manifolds | 22 | Open-addressing set of contact pairs. |
| `src/solver.h`, `src/solver.c` | `solver.go` | T0/T1/T2 | solver | 23 | Nine ordered stages, from prepare joints to store impulses. `MakeSoft` is pure arithmetic. The integration tasks, the body finalize and the single-worker sub-step order landed with order 16; see D-004 and D-006. The constraint stages wait. |
| `src/contact_solver.h`, `src/contact_solver.c` | `contact_solver.go` | T0/T1 | solver | 24 | Port the `Overflow` family. The `Task` family is T2 until the second executor exists. |
| `src/island.h`, `src/island.c` | `island.go` | T0 | solver | 25 | Island linking, merging and splitting landed. The joint lists, the wake calls and the sleep path wait for their stages. |
| `src/constraint_graph.h`, `src/constraint_graph.c` | `constraint_graph.go` | T0 | solver | 26 | Eleven colors plus the overflow color landed. The color schedule is the parallel contract. The joint functions wait. |
| `src/bitset.h`, `src/bitset.c` | `bitset.go` | T0 | broadphase | 27 | Set, clear, test, grow and union landed. Backs the constraint graph and the contact state of the step. |
| `src/ctz.h` | `math/bits` | T2 | broadphase | 28 | The standard library replaces the compiler intrinsics. |
| `src/dynamic_tree.c` | `dynamic_tree.go` | T0 | broadphase | 29 | Fattened AABBs, surface-area heuristic, rotation rebalance. |
| `src/broad_phase.h`, `src/broad_phase.c` | `broad_phase.go` | T0 | broadphase | 30 | Pair output is sorted by integer id, so any equivalent tree gives the same world. |
| `src/atomic.h` | `sync/atomic` | T2 | broadphase | 31 | Needed only when a second executor exists. |
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
