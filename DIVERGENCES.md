# Known differences from Box2D

`dbox2d` follows the Box2D v3.1.1 formulation by default. This ledger lists
the places where Go, fixed-point arithmetic, or deterministic execution
requires a different implementation.

These are decisions, not undocumented surprises: every entry gives the
reason, the resulting behaviour, and the test that protects it. If the Go
code differs from the reference and there is no entry here, treat it as a bug.

An entry is required for every line marked T2 in [PORTING.md](PORTING.md).
Entries are append-only. A resolved divergence keeps its number and gains a
`Resolved` line, so the history remains readable.

The most visible API differences are fixed Q32.32 values, turns instead of
radians in public angle fields, Go closures instead of `void*` callback
contexts, and panics for invalid preconditions. See the individual entries
for the exact contract.

## Entry format

```
### D-000 short title

- File: manifold.go (upstream src/manifold.c:186)
- Tier: T2
- Reason: one or two sentences. What the arithmetic or the language forbids.
- Behaviour: what the port does instead.
- Test: TestName in manifold_test.go
- Resolved: optional, with the commit that removed the divergence
```

Numbering is sequential from `D-001` and never reused.

## Entries

### D-001 A tolerance is a function, not a constant

- File: constants.go (upstream src/constants.h)
- Tier: T2
- Reason: Go declares a constant only for a basic type, and `Q` is a struct.
- Behaviour: each tolerance is an unexported variable with an exported
  accessor. A caller reads the value and nothing writes it.
- Test: TestConstantsMatchTheReference in math_test.go

### D-002 A length is a meter

- File: constants.go (upstream src/core.c:35)
- Tier: T2
- Reason: `b2SetLengthUnitsPerMeter` rescales every tolerance from a mutable
  global. Two peers that set it differently compute different worlds from the
  same input, and nothing reports the mismatch.
- Behaviour: the setter and the getter do not cross. A length is a meter.
- Test: TestConstantsMatchTheReference in math_test.go

### D-003 An assertion becomes a panic

- Files: validation sites throughout the solver (upstream `B2_ASSERT`)
- Tier: T2
- Reason: the reference can compile assertions out; Go has no equivalent
  release switch, and continuing after a failed precondition risks corruption.
- Behaviour: invalid preconditions panic in every build. Validation queries still
  return booleans, while mutation during a locked step panics.
- Test: `TestIdPoolRejectsAnUnknownIndex`, `TestStepRejectsInvalidInput`, and
  `TestCreateChainPanicsBelowFourPoints`, plus the geometry and joint cases

### D-004 An angle is a turn

- Files: `math.go`, `scalar_*.go`, `body.go`, `world.go`,
  `solver.go`, `contact_solver.go`, and the joint solvers
- Tier: T2
- Reason: a turn wraps by exact subtraction. A radian requires a rounded
  value of pi, and that rounding enters every reduction.
- Behaviour: public angles and angular velocities use turns; a quarter turn is
  `QFromRatio(1, 4)`. `UnwindAngle` subtracts the nearest whole turn and
  `MaxRotation` is 0.125 turns. Fixed-mode joints keep their angles in turns;
  float-mode joints keep the reference's internal radians. The body state also
  keeps radians per second where the solver expects the reference unit. Torque
  and angular-impulse APIs retain the reference's radian contract; target
  transforms use the selected mode's angle unit.
- Test: `TestIntegrateRotationCompletesATurn`,
  `TestUnwindAngleReducesToHalfTurn`, `TestSetTargetTransformDerivesVelocity`,
  and `TestApplyAngularImpulseInTurns`

### D-005 Validity is a range check

- File: math.go (upstream src/math_functions.c `b2IsValidFloat`)
- Tier: T2
- Reason: Q32.32 has no NaN and no infinity. A computation that leaves the
  range saturates instead.
- Behaviour: `IsValidQ` rejects the two saturation values, and the vector,
  rotation and box checks build on it. In the float mode the reference guard
  applies; see D-017.
- Test: TestSaturationMarksAValueInvalid in math_test.go

### D-006 A reciprocal becomes a division

- Files: scalar, geometry, query, solver, and joint arithmetic sites
- Tier: T2
- Reason: a Q32.32 reciprocal loses the leading precision needed by a later
  product.
- Behaviour: fixed mode divides at each affected site. Float mode uses the
  reference's rounded reciprocal. Effective masses are the exception: both
  modes compute and store them once because several solver stages reuse them.
  Zero denominators are exact checks in fixed mode.
- Test: `TestSolve22SolvesTheSystem`,
  `TestStepAppliesDampingByTheReciprocal`, and
  `TestPrepareOverflowContactsBuildsTheMasses`

### D-007 The normalization tolerance is in raw units

- File: math.go (upstream include/box2d/math_functions.h `b2IsNormalized`)
- Tier: T2
- Reason: `100 * FLT_EPSILON` describes the spacing of the float grid, which
  Q32.32 does not have.
- Behaviour: `IsNormalized` compares against 2^16 raw units, about 1.5e-5,
  which is the magnitude of the upstream 1.2e-5. `IsNormalizedRot` keeps the
  literal 0.0006 of the reference, because that one is a plain number. In the
  float mode the reference guard applies; see D-017.
- Test: TestNormalizedChecksAcceptAUnitPair in math_test.go

### D-008 An epsilon guard becomes a test against zero

- Files: `aabb.go`, `geometry.go`
- Tier: T2
- Reason: Q32.32 has one fixed spacing, so a float epsilon guard has no
  equivalent near-zero interval.
- Behaviour: affected ray, capsule, hull, and polygon guards test exact zero.
  Degenerate polygons still panic under D-003. Float mode keeps the reference
  epsilon guards.
- Test: `TestAABBRayCastHitsTheNearFace`,
  `TestPolygonConstructorsRejectInvalidHull`, and
  `TestComputePolygonMassRejectsZeroArea`

### D-009 An infinite sentinel becomes the largest representable value

- Files: `aabb.go`, `hull.go`, `manifold.go`, `dynamic_tree.go`,
  `types.go`, `world.go`, and `mover.go`
- Tier: T2
- Reason: the reference seeds searches with `FLT_MAX`; Q32.32 has no
  infinity.
- Behaviour: fixed mode uses the largest and smallest representable values for
  search seeds and clamps. Public defaults that need an unreachable bound use
  `Huge`, including distance-joint maximum length and mover push limits.
- Test: `TestTreeSeedNeverWins` and
  `TestSolvePlanesSinglePlaneProjects`

### D-010 A generated array becomes a slice

- Files: `array.go`, `broad_phase.go`, `distance.go`, `step.go`,
  `shape.go`, and `body.go`
- Tier: T2
- Reason: C generates typed arrays with macros; Go slices already carry length
  and capacity.
- Behaviour: simulation arrays become slices and `removeSwap` preserves the
  reference's index-repair contract. Count-plus-buffer APIs become
  caller-owned slices. Capacity affects allocation only, never a result.
  Pair, arena, bullet, chain, and simplex storage keep the same observable
  ordering and count rules.
- Test: `TestStepAllocatesNothing`,
  `TestSleepReusesTheStorageOfTheLastWokenSet`,
  `TestShapeDistanceWarmStartsFromTheCache`, and
  `TestGetContactDataReturnsTouchingManifold`

### D-011 The determinism witness is port-only

- File: `checksum.go` (no upstream counterpart)
- Tier: T2
- Reason: the reference has no cross-platform state witness, while the port
  needs one for tests and synchronization.
- Behaviour: `Checksum` folds the canonical world configuration and the
  complete state of bodies, shapes, contacts, and joints. It uses raw scalar
  bits and order-independent wrapping sums; application data and storage IDs
  are excluded. Contact endpoints and manifold points are canonicalized, and
  pending island splits are included because they affect future steps.
- Test: `TestChecksumIsOrderIndependent`, `TestChecksumSeesFutureBehaviour`,
  `TestChecksumSeesAPendingSplit`, and
  `TestStepIsReproducibleBitForBit`

### D-012 An epsilon guard becomes an exact zero test

- Files: `distance.go`, `manifold.go`, `sensor.go`, `world.go`
- Tier: T2
- Reason: below Q32.32's resolution, the only representable boundary is zero.
- Behaviour: segment degeneracy, zero search direction, coincident closest
  points, clipping spans, sensor overlap, and explosion direction use exact
  zero tests in fixed mode. Nonzero values take the regular path; float mode
  keeps the reference epsilon checks.
- Test: `TestSegmentDistanceHandlesDegenerateSegments`,
  `TestSensorTouchingEdgeIsNotOverlap`, and
  `TestExplodeImpulseByDistance`

### D-013 The pairs of a moved proxy are sorted by shape id

- File: `broad_phase.go` (upstream `b2PairQueryCallback`)
- Tier: T2
- Reason: the reference's tree-walk order depends on tree topology and can
  change contact creation order.
- Behaviour: new pairs for one moved proxy are appended and sorted by
  `(shapeIdA, shapeIdB)`; moved proxies retain move-buffer order. The default
  therefore remains independent of equivalent tree topology. The
  `dbox2d_upstream_pairs` tag restores reference prepend order and gives up
  that independence.
- Test: `TestBroadPhasePairsAreSortedByShapeId`,
  `TestChecksumIgnoresTheTreeTopology`, and `TestConformance`

### D-014 A callback with a context becomes a closure

- Files: `dynamic_tree.go`, `world.go`, `solver.go`, and `types.go`
- Tier: T2
- Reason: Go closures capture state directly; a separate `void*` context adds
  no value.
- Behaviour: tree walks, world queries, continuous collision, debug drawing,
  mover results, and material/filter callbacks use typed closures. A
  non-escaping closure allocates nothing; the step and bullet allocation tests
  protect that property.
- Test: `TestTreeQueryReportsTheOverlaps`, `TestShapeQueriesMatchBruteForce`,
  `TestSetFrictionCallbackAffectsNextContact`, and
  `TestPreSolveFalseLetsTheBoxThrough`

### D-015 The memory dump writes to an io.Writer

- File: `world.go` (upstream `b2World_DumpMemoryStats`)
- Tier: T2
- Reason: Go callers should choose the output sink instead of the library
  opening a fixed file.
- Behaviour: `DumpMemoryStats` writes the six reference sections to an
  `io.Writer`. It reports Go slice and backing-storage capacities, fixes the
  reference's `island sim` size calculation, which multiplies by `sizeof(int)`
  because of an upstream accumulator-name bug, and omits the reference's
  unfinished chain-shapes section.
- Test: `TestDumpMemoryStatsListsEverySection` in `world_test.go`

### D-016 The port owns its workers

- Files: `executor*.go`, `solver.go`, `step.go`, `broad_phase.go`,
  `sensor.go`, and `types.go`
- Tier: T2
- Reason: the C API receives a task system from the caller. Go can own the
  worker pool, and deterministic simulation requires worker-count independence.
- Behaviour: `WorldDef.WorkerCount == 1` runs inline; `0` selects
  `min(GOMAXPROCS, 64)`; WebAssembly uses one worker. Native workers keep
  the reference stage order and merge per-worker results in a fixed order, so
  worker count does not change result bits. Small worlds may run on the
  caller. Callbacks run on worker goroutines and `Counters.TaskCount` reports
  the last step's tasks.

  Box2D v3.1.1 has a sleep-selection tie bug: the candidate chosen inside a
  worker depends on the order in which islands are visited, while the final
  reduction uses the island id. As a result, the upstream `rain` scene starts
  diverging at step 450 when the worker count changes, despite the upstream
  documentation promising identical results for every thread count. The port
  keeps the single-worker choice and applies it in a fixed order, so its result
  is independent of worker count. Upstream fixed the tie after v3.1.1 in commit
  `436365a246` (PR #1065); moving to that release will change the witnesses and
  traces.
- Test: `TestStepIsWorkerCountIndependent`,
  `TestSensorEventsAreWorkerCountIndependent`, `TestCountersReportTasks`,
  and the executor tests

### D-017 Two scalar modes

- Files: `scalar_fixed.go`, `scalar_float.go`, and their shared consumers
- Tier: T2
- Reason: the reference is float32-only, while the port also provides Q32.32.
- Behaviour: both modes share the solver and expose the same Go API. Fixed mode
  uses Q32.32; float mode uses explicit float32 rounding points and the
  reference's scalar math. Float remains deterministic for a fixed supported
  build; the CI FMA gate prevents silent fusion from changing that build's
  result. Fixed point provides a stronger contract across toolchains and
  architectures. Mode-specific epsilon,
  reciprocal, angle, validity, and checksum rules are defined in D-004,
  D-005, D-006, D-008, and D-011.
- Test: `TestChecksumMatchesDeterministicWitness`, the per-mode scalar tests,
  cross-architecture builds, and the CI FMA gate

### D-018 Conformance traces

- Files: `conformance_test.go`, `tools/conformance/main.c`, and
  `testdata/conformance/`
- Tier: T2
- Reason: the reference provides neither a trace format nor a bit-level oracle.
- Behaviour: frozen traces cover 17 collision/function files and 8 scene files.
  The generator uses the pinned reference without SIMD or FMA. Float function
  traces are exact except for the documented `make_rot` budget; fixed traces
  use absolute budgets. Float scene traces match bit for bit with
  `dbox2d_upstream_pairs`; the default sorted-pair build is checked with
  step-one budgets and diagnostic later hashes. Fixed scene traces use
  fixed-mode budgets and saturation gates. The traces also caught two port
  bugs: colored contacts used `maxContactPushSpeed` instead of `contactSpeed`,
  and grouped the normal-impulse expression differently from the reference.
  Both fixes are covered by the regenerated witnesses.
- Test: `TestConformance`; generation details and exact format are in
  [tools/conformance/README.md](tools/conformance/README.md)

### D-019 SIMD contact family

- Files: `contact_solver_simd.go`, `simd_*.go`, `scalar_*.go`, and
  `solver.go`
- Tier: T2
- Reason: the reference selects SIMD lanes at compile time; colors rarely
  contain a multiple of the lane width, and the port needs one scalar oracle.
- Behaviour: `dbox2d_simd` adds a padded lane path beside the scalar path.
  Native widths are 8 on amd64 AVX2 and 4 on arm64 NEON; WebAssembly and other
  targets use a generic path, with scalar fallback where required. Padding
  never writes back. Each lane uses separate rounded multiply/add operations,
  so the SIMD family matches the scalar family across ISAs. Fixed mode uses
  the fixed module's Q16 lanes and Q48 accumulators, subject to D-020.
- Test: SIMD lane, scalar-parity, layout, witness, sample, conformance, and
  architecture-matrix tests

### D-020 Contacts solve on a Q16 grid

- Files: `contact_solver.go`, `contact_solver_q32.go`, `scalar_*.go`,
  `internal/q32gen`, and `tools/q32gen`
- Tier: T2
- Reason: Q32.32 contacts cannot match Q16.16 SIMD lanes bit for bit.
- Behaviour: fixed-mode contact preparation and solver stages round to a
  Q16.16 grid with Q48.16 accumulators. Contacts whose inverse mass or inertia
  lies outside [2^-6, 2^15) use the generated Q32.32 path instead. The scalar
  and SIMD families share this partition and match while no contact value
  saturates. Float mode aliases the grid types to its scalar type.
- Test: `TestHeavyBoxRestsOnTheGround`,
  `TestStepPartitionsContactsByTheLaneWindow`, `TestQ32ContactSolverIsFresh`,
  and the conformance saturation gate
