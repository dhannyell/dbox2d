# Divergence ledger

The port keeps the upstream structure by default. Every place where it does not
is recorded here, with the reason and the test that covers it. A divergence
without an entry is a defect, not a decision.

An entry is required for every line marked T2 in [PORTING.md](PORTING.md).
Entries are append-only: a divergence that is later removed keeps its entry and
gains a `Resolved` line, so the history of the port stays readable.

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

- Files: id_pool.go, aabb.go, math.go, hull.go, geometry.go, step.go,
  solver.go (upstream `B2_ASSERT`)
- Tier: T2
- Reason: `B2_ASSERT` compiles out in a release build. Go has no such switch,
  and a silent corruption costs more than a stop.
- Behaviour: a failed precondition panics in every build. Functions whose
  upstream contract is a validation query still return a bool.
- Test: TestIdPoolRejectsAnUnknownIndex in id_pool_test.go,
  TestMakeAABBRejectsEmptyPoints in aabb_test.go,
  TestComputeRotationBetweenUnitVectors in math_test.go,
  TestPolygonConstructorsRejectInvalidHull and
  TestComputePolygonMassRejectsZeroArea in geometry_test.go, and
  TestStepRejectsInvalidInput in step_test.go

### D-004 An angle is a turn

- Files: math.go, body.go, solver.go (upstream
  include/box2d/math_functions.h; src/body.c `b2UpdateBodyMassData`;
  src/solver.c `b2IntegrateVelocitiesTask`, `b2FinalizeBodiesTask`)
- Tier: T2
- Reason: a turn reduces to its range by an exact subtraction. A radian needs
  a rounded pi, and the rounding enters every reduction.
- Behaviour: `MaxRotation` is 0.125 turns, which is the upstream
  `0.25 * B2_PI` radians. `IntegrateRotation` scales its displacement by one
  turn before the first order step, and `ComputeAngularVelocity` divides by
  one turn after it. `RotGetAngle`, `RelativeAngle` and `UnwindAngle` work in
  turns, and `UnwindAngle` subtracts the nearest whole turn instead of taking
  a remainder of two pi. `updateBodyMassData` stores the angular velocity in
  turns per second, so it scales the velocity by one turn before the cross
  product that corrects the linear velocity of a moved center of mass.
  `integrateVelocitiesTask` divides the torque delta by one turn, and
  `finalizeBodiesTask` scales the arc speed of the sleep test by one turn.
- Test: TestIntegrateRotationCompletesATurn,
  TestComputeAngularVelocityInvertsIntegration and
  TestUnwindAngleReducesToHalfTurn in math_test.go,
  TestBodyMassComesFromItsShapes in world_test.go, and
  TestStepConvertsTorqueAndArcSpeedToTurns in step_test.go

### D-005 Validity is a range check

- File: math.go (upstream src/math_functions.c `b2IsValidFloat`)
- Tier: T2
- Reason: Q32.32 has no NaN and no infinity. A computation that leaves the
  range saturates instead.
- Behaviour: `IsValidQ` rejects the two saturation values, and the vector,
  rotation and box checks build on it.
- Test: TestSaturationMarksAValueInvalid in math_test.go

### D-006 A reciprocal becomes a division

- Files: math.go, aabb.go, geometry.go, solver.go (upstream
  include/box2d/math_functions.h `b2GetInverse22`, `b2Solve22`,
  `b2Normalize`, `b2NormalizeRot`; src/aabb.c `b2AABB_RayCast` `inv_d`;
  src/geometry.c `b2ComputePolygonCentroid` and `b2ComputePolygonMass`
  `inv3` and `invArea`, `b2RayCastCapsule` `invDen`;
  src/solver.c `b2IntegrateVelocitiesTask` damping factors)
- Tier: T2
- Reason: a Q32.32 reciprocal keeps only the leading bits of a large value.
  Multiplying by it discards the precision that a division keeps.
- Behaviour: each site divides by its denominator. Normalization delegates
  to the fixed-point module, which scales the pair before it squares, so a
  short vector cannot underflow to zero. The guard against a zero length
  becomes an exact test against zero instead of a test against an epsilon.
  `NormalizeRot` still returns a zero rotation for a zero input, as the
  reference does, so an invalid rotation stays visible to `IsValidRotation`.
  The slab test divides each distance by the direction component. The
  centroid, the polygon mass and the capsule side hit divide by the area or
  by the determinant at each use. The velocity integration divides each
  damped velocity by the damping denominator `1 + h*c` instead of
  multiplying by the reciprocal factor.
- Test: TestSolve22SolvesTheSystem, TestNormalizeKeepsAShortVector and
  TestNormalizeRotKeepsAZeroRotation in math_test.go,
  TestAABBRayCastHitsTheNearFace in aabb_test.go,
  TestPolygonCentroidOfATriangle, TestTriangleMassMatchesTheReference and
  TestRayCastCapsuleHitsTheSide in geometry_test.go, and
  TestStepAppliesDampingByDivision in step_test.go

### D-007 The normalization tolerance is in raw units

- File: math.go (upstream include/box2d/math_functions.h `b2IsNormalized`)
- Tier: T2
- Reason: `100 * FLT_EPSILON` describes the spacing of the float grid, which
  Q32.32 does not have.
- Behaviour: `IsNormalized` compares against 2^16 raw units, about 1.5e-5,
  which is the magnitude of the upstream 1.2e-5. `IsNormalizedRot` keeps the
  literal 0.0006 of the reference, because that one is a plain number.
- Test: TestNormalizedChecksAcceptAUnitPair in math_test.go

### D-008 An epsilon guard becomes a test against zero

- Files: aabb.go, geometry.go (upstream src/aabb.c `b2AABB_RayCast`,
  src/geometry.c `b2RayCastCapsule`, `b2ComputePolygonCentroid`,
  `b2ComputePolygonMass`, `b2MakePolygon`, `b2MakeOffsetRoundedPolygon`)
- Tier: T2
- Reason: `FLT_EPSILON` describes the spacing of the float grid near one.
  Q32.32 has one spacing everywhere, so a value below the float epsilon is
  either exactly zero or exactly representable. The guard has no meaning.
- Behaviour: each guard compares against zero. A parallel slab, a capsule or
  polygon edge of zero length, a determinant of zero and an area of zero are
  exact cases now, not near cases. A degenerate area still panics, which
  follows D-003, because a polygon with no area has no centroid or mass.
  `ValidateHull` rejects a zero-length edge before either polygon constructor
  reaches its redundant edge guard.
- Test: TestAABBRayCastHitsTheNearFace in aabb_test.go and
  TestRayCastCapsuleDegenerateCases, TestPolygonConstructorsRejectInvalidHull
  and TestComputePolygonMassRejectsZeroArea in geometry_test.go

### D-009 An infinite sentinel becomes the largest representable value

- Files: aabb.go, hull.go (upstream src/aabb.c `b2AABB_RayCast`,
  src/hull.c `b2ComputeHull`)
- Tier: T2
- Reason: the reference seeds a search with `FLT_MAX`, which no coordinate
  reaches. Q32.32 has no infinity and it saturates instead.
- Behaviour: the seeds are the largest and the smallest representable values.
  Those values sit outside the valid input range: `IsValidQ` rejects a
  coordinate that equals either seed.
- Test: TestAABBRayCastHitsTheNearFace in aabb_test.go and
  TestComputeHullDropsAnInteriorPoint in hull_test.go

### D-010 A generated array becomes a slice

- Files: array.go and every file that stores a sim array (upstream
  src/array.h, src/array.c)
- Tier: T2
- Reason: the reference generates one array type per element type with
  macros. Go has no macros, and the slice already carries the length and the
  capacity that the generated struct tracks by hand.
- Behaviour: every upstream array is a Go slice. `removeSwap` keeps the
  swap-remove contract and returns the old index of the moved element, so
  the caller repairs the stored index as the reference does. The growth
  policy is the one of the Go runtime; the capacity never enters a
  simulation result.
- Test: TestCreateAndDestroyOrdersProduceTheSameWorld and
  TestSleepingBodyGetsItsOwnSolverSet in world_test.go

### D-011 The determinism witness is port-only

- Files: checksum.go (no upstream counterpart)
- Tier: T2
- Reason: the reference runs on floats and promises no cross-platform state
  equality. A fixed-point world does, and the promise needs a witness that
  the tests and a network peer can compare.
- Behaviour: `Checksum` folds the deterministic world configuration and the
  complete simulation state of every live body and shape. Body and shape
  hashes use wrapping sums, so internal ids and creation order do not affect
  the result. Application data stays out because it cannot change the
  simulation. Q values enter as raw bits; no float ever does.
- Test: TestChecksumIsOrderIndependent, TestChecksumSeesAStateChange,
  TestChecksumSeesFutureBehaviour and TestChecksumMatchesDeterministicWitness
  in checksum_test.go, TestStepIsReproducibleBitForBit in step_test.go

### D-012 An epsilon guard becomes an exact zero test

- File: distance.go (upstream src/distance.c:44)
- Tier: T2
- Reason: `FLT_EPSILON` guards absorb float rounding noise. Q32.32 has no
  such noise, and its smallest magnitude is one raw unit, which already
  exceeds the squared-epsilon threshold of the reference. The exact zero is
  the only dividing line Q can express below its own resolution.
- Behaviour: the degenerate branch of `SegmentDistance` runs only when a
  squared segment length is exactly zero. Every nonzero Q length takes the
  regular path, as it would in float.
- Test: TestSegmentDistanceHandlesDegenerateSegments in distance_test.go
