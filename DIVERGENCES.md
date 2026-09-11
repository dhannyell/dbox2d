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
  solver.go, manifold.go, table.go, contact.go, distance.go, world.go,
  joint.go, shape.go and the seven joint files (upstream `B2_ASSERT`)
- Tier: T2
- Reason: `B2_ASSERT` compiles out in a release build. Go has no such switch,
  and a silent corruption costs more than a stop.
- Behaviour: a failed precondition panics in every build. Functions whose
  upstream contract is a validation query still return a bool. Every setter
  that the reference guards with `b2GetWorldLocked` or the equivalent
  `B2_ASSERT(world->locked == false)` panics on a locked world instead of
  silently returning.
- Test: TestIdPoolRejectsAnUnknownIndex in id_pool_test.go,
  TestMakeAABBRejectsEmptyPoints in aabb_test.go,
  TestComputeRotationBetweenUnitVectors in math_test.go,
  TestPolygonConstructorsRejectInvalidHull and
  TestComputePolygonMassRejectsZeroArea in geometry_test.go,
  TestStepRejectsInvalidInput in step_test.go,
  TestCollideCapsulesRejectsADegenerateCapsule and
  TestCollideSegmentAndPolygonRejectsADegenerateSegment in manifold_test.go,
  TestIterativeGeometryRejectsInvalidInput in distance_test.go,
  TestRevoluteRejectsAFullTurnLimit in joint_test.go, and
  TestCreateChainPanicsBelowFourPoints in shape_test.go

### D-004 An angle is a turn

- Files: math.go, scalar_fixed.go, scalar_float.go, body.go, world.go,
  solver.go, contact_solver.go, joint.go, motor_joint.go and the other six
  joint files (upstream
  include/box2d/math_functions.h; src/body.c `b2UpdateBodyMassData`,
  `b2Body_ApplyTorque`, `b2Body_ApplyAngularImpulse`,
  `b2Body_SetTargetTransform`; src/world.c `b2World_Explode`;
  src/solver.c `b2IntegrateVelocitiesTask`, `b2FinalizeBodiesTask`;
  src/solver.h `b2MakeSoft`; src/contact_solver.c the `Overflow` family;
  src/joint.c `b2CreateRevoluteJoint` limit check and the angle accessors;
  the prepare, warm start and solve of src/revolute_joint.c,
  src/prismatic_joint.c, src/wheel_joint.c, src/weld_joint.c,
  src/motor_joint.c `b2MotorJoint_SetAngularOffset` and
  `b2MotorJoint_GetAngularOffset`, src/mouse_joint.c and
  src/distance_joint.c)
- Tier: T2
- Reason: a turn reduces to its range by an exact subtraction. A radian needs
  a rounded pi, and the rounding enters every reduction.
- Behaviour: `MaxRotation` is 0.125 turns, which is the upstream
  `0.25 * B2_PI` radians. `IntegrateRotation` scales its displacement by one
  turn before the first order step, and `ComputeAngularVelocity` divides by
  one turn after it. `RotGetAngle`, `RelativeAngle` and `UnwindAngle` work in
  turns, and `UnwindAngle` subtracts the nearest whole turn instead of taking
  a remainder of two pi. The body state keeps the angular velocity in
  radians per second, the unit of the reference, so the contact and joint
  stages, the velocity integration, the sleep test and `updateBodyMassData`
  use it as the reference does, and the solver advances the rotation with
  the radian form of `IntegrateRotation`. The turn per second is a unit of
  the API: `BodyDef.AngularVelocity` and `BodyId.SetAngularVelocity`
  multiply by one turn, and `BodyId.GetAngularVelocity` divides by it.
  `integrateVelocitiesTask` scales `MaxRotation` by one turn, an exact
  product. `makeSoft` multiplies the frequency by one turn where the
  reference multiplies by two pi. The API takes a reference angle, a limit,
  a target angle and an angular offset in turns, and an angular motor speed
  in turns per second. A joint keeps them in the angle unit of the mode,
  which the scalar files define (D-017). Fixed mode keeps turns, and each
  enters a constraint error, a bias or the motor row multiplied by one turn.
  Float mode keeps radians, and the joint computes each error with
  `b2RelativeAngle` and `b2UnwindAngle` as the reference does. A turn of the
  form a/2 converts to the `a * B2_PI` of the reference exactly, because
  halving a float32 is exact; the tumbler motor speed,
  `(B2_PI / 180) * 25`, has no such form and stays one ulp away. The
  revolute limit check bounds the
  angles at 0.495 turns, the `0.99 * pi` of the reference. Every joint angle
  accessor, including `GetAngularSeparation`, reports turns; the motor
  joint's `SetAngularOffset` and `GetAngularOffset` keep the same unit.
  `BodyId.ApplyTorque`, `BodyId.ApplyAngularImpulse` and `WorldId.Explode`
  apply the reference's radian units as given. `BodyId.SetTargetTransform`
  takes the relative angle in the angle unit of the mode and converts it to
  radians for its velocity. The debug draw converts the stored revolute
  limits to turns for `MakeRot`.
- Test: TestIntegrateRotationCompletesATurn,
  TestComputeAngularVelocityInvertsIntegration and
  TestUnwindAngleReducesToHalfTurn in math_test.go,
  TestBodyMassComesFromItsShapes in world_test.go,
  TestStepAppliesTorqueAndArcSpeedInRadians in step_test.go,
  TestFrictionSaturatesAtTheNormalImpulse in contact_solver_test.go,
  TestRevoluteRejectsAFullTurnLimit in joint_test.go,
  TestMotorTurnsTowardTheAngularOffset and
  TestMotorJointAngularOffsetClampsToHalfTurn in motor_joint_test.go,
  TestSolveRevoluteJointTracksTheFloat64Mirror in
  revolute_joint_internal_test.go,
  TestSetTargetTransformDerivesVelocity and
  TestApplyAngularImpulseInTurns in body_test.go, and
  TestExplodeImpulseByDistance in world_test.go

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

- Files: scalar_fixed.go, scalar_float.go, math.go, aabb.go, geometry.go,
  body.go, world.go, solver.go,
  manifold.go, contact_solver.go, joint.go and the seven joint files
  (upstream
  include/box2d/math_functions.h `b2GetInverse22`, `b2Solve22`,
  `b2Normalize`, `b2NormalizeRot`; src/aabb.c `b2AABB_RayCast` `inv_d`;
  src/geometry.c `b2ComputePolygonCentroid` and `b2ComputePolygonMass`
  `inv3` and `invArea`, `b2RayCastCapsule` `invDen`;
  src/body.c `b2Body_SetTargetTransform` `invH` and
  `b2Body_SetMassData` `invMass` and `invI`;
  src/world.c `b2World_Explode` falloff `scale`;
  src/solver.c `b2IntegrateVelocitiesTask` damping factors;
  src/manifold.c `b2CollideChainSegmentAndCircle` `1/ee` and
  `b2CollidePolygons` vertex-vertex `1.0f / distance`;
  src/solver.h `b2MakeSoft` `a3`;
  src/contact_solver.c `b2PrepareOverflowContacts` effective masses;
  src/distance.c `b2SolveSimplex2` `inv_d12` and `b2SolveSimplex3`
  `inv_d12`, `inv_d13`, `inv_d23`, `inv_d123`; src/distance.c
  `b2ShapeDistance` `0.1f * B2_LINEAR_SLOP`, `b2ShapeCast` and
  `b2TimeOfImpact` `0.25f * B2_LINEAR_SLOP`; src/world.c
  `b2World_OverlapShape` `0.1f * B2_LINEAR_SLOP`; src/manifold.c
  `b2CollideChainSegmentAndPolygon` `0.1f * B2_LINEAR_SLOP`; src/joint.c
  `b2PrepareJointsTask` `0.25f * context->inv_h`; the prepare of each
  joint file, the `axialMass`, `perpMass`, `motorMass` and `angularMass`
  reciprocals)
- Tier: T2
- Reason: a Q32.32 reciprocal keeps only the leading bits of a large value.
  Multiplying by it discards the precision that a division keeps.
- Behaviour: in fixed mode each site divides by its denominator. In float
  mode each site multiplies by one rounded reciprocal, as the reference
  does: the sites go through `makeRecip`, which the scalar files define per
  mode (D-017). The forms below are the fixed forms. Normalization delegates
  to the fixed-point module, which scales the pair before it squares, so a
  short vector cannot underflow to zero. The guard against a zero length
  becomes an exact test against zero instead of a test against an epsilon.
  `NormalizeRot` still returns a zero rotation for a zero input, as the
  reference does, so an invalid rotation stays visible to `IsValidRotation`.
  The slab test divides each distance by the direction component. The
  centroid, the polygon mass and the capsule side hit divide by the area or
  by the determinant at each use. The velocity integration divides each
  damped velocity by the damping denominator `1 + h*c` instead of
  multiplying by the reciprocal factor. `makeSoft` divides each scale by
  `1 + a2` instead of multiplying by its reciprocal. The effective masses
  of a contact point are the exception: they store the reciprocal once,
  as the body inverse mass does, because three stages read them on every
  sub-step; the guard against a zero denominator is an exact test. The
  contact speed of the step becomes zero when the static softness has a
  zero mass scale, because the reference divides by that scale and a Q
  division by zero panics. The simplex solvers divide each barycentric
  weight by its denominator; the denominator is exactly positive on the
  branch that reaches it. The slop fractions of the distance queries are
  divisions as well: `ShapeDistance` divides the slop by ten for its radius
  guard, `ShapeCast` and `TimeOfImpact` divide it by four for their
  tolerance, and `OverlapShape` and `CollideChainSegmentAndPolygon` divide
  it by ten. The joint prepare divides the sub-step rate by four for the
  frequency clamp. The effective masses of a joint follow the contact
  point: the prepare divides one by the denominator once, guarded by an
  exact test against zero, and the stages read the stored mass.
  `BodyId.SetTargetTransform` computes the reciprocal of the caller's time
  step with one division and multiplies by it twice, matching the
  reference. `BodyId.SetMassData` divides by the mass
  and by the rotational inertia to store their inverses, guarded by the
  same exact zero test as the shape-driven mass update. `WorldId.Explode`
  divides the falloff band by its length to build the fade-out scale.
- Test: TestSolve22SolvesTheSystem, TestNormalizeKeepsAShortVector and
  TestNormalizeRotKeepsAZeroRotation in math_test.go,
  TestAABBRayCastHitsTheNearFace in aabb_test.go,
  TestPolygonCentroidOfATriangle, TestTriangleMassMatchesTheReference and
  TestRayCastCapsuleHitsTheSide in geometry_test.go,
  TestStepAppliesDampingByTheReciprocal in step_test.go, and
  TestMakeSoftSplitsTheUnit and TestPrepareOverflowContactsBuildsTheMasses
  in contact_solver_test.go, TestStepKeepsAZeroContactFrequencyFinite
  in step_test.go, TestShapeDistanceMatchesHandCases in
  distance_test.go, TestRevoluteHoldsTheAnchor in revolute_joint_test.go,
  and TestWheelLineHoldsTheBox in wheel_joint_test.go

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
  reaches its redundant edge guard. In the float mode the reference guard
  applies; see D-017.
- Test: TestAABBRayCastHitsTheNearFace in aabb_test.go and
  TestRayCastCapsuleDegenerateCases, TestPolygonConstructorsRejectInvalidHull
  and TestComputePolygonMassRejectsZeroArea in geometry_test.go

### D-009 An infinite sentinel becomes the largest representable value

- Files: aabb.go, hull.go, manifold.go, dynamic_tree.go, types.go, world.go,
  mover.go (upstream
  src/aabb.c `b2AABB_RayCast`, src/hull.c `b2ComputeHull`, src/manifold.c
  `b2CollidePolygonAndCircle`, `b2FindMaxSeparation` and `b2CollidePolygons`
  search seeds, `b2CollideChainSegmentAndPolygon` SAT seeds
  `edgeSeparation`, `s0`, `s2` (`FLT_MAX` at lines 1525, 1540, 1563) and
  `polygonSeparation` (`-FLT_MAX` at line 1585), src/dynamic_tree.c `b2FindBestSibling` lower bounds and
  `b2PartitionSAH` bin bounds and cost seed; src/types.c
  `b2DefaultDistanceJointDef` `maxLength`; src/world.c
  `b2World_SetContactTuning` clamp upper bound; src/mover.c
  `b2SolvePlanes` rigid `pushLimit`)
- Tier: T2
- Reason: the reference seeds a search with `FLT_MAX`, which no coordinate
  reaches. Q32.32 has no infinity and it saturates instead.
- Behaviour: the seeds are the largest and the smallest representable values.
  Those values sit outside the valid input range: `IsValidQ` rejects a
  coordinate that equals either seed. The default `MaxLength` of a distance
  joint is `Huge`, the `B2_HUGE` of the reference: a length that no world
  reaches and that the range checks accept, so the rope stays slack until
  a definition lowers it. `SetContactTuning` clamps its hertz, damping
  ratio and push speed against `Q32MaxValue` rather than `FLT_MAX`, so a
  caller cannot saturate the tuning past what a Q32.32 value can hold.
  `SolvePlanes` takes its rigid push limit as a plain `PushLimit` field;
  the mover callers pass `Huge`, the same `B2_HUGE` sentinel, in place of
  the reference's `FLT_MAX`.
- Test: TestAABBRayCastHitsTheNearFace in aabb_test.go,
  TestComputeHullDropsAnInteriorPoint in hull_test.go,
  TestTreeSeedNeverWins in dynamic_tree_test.go, and
  TestSolvePlanesSinglePlaneProjects in mover_test.go

### D-010 A generated array becomes a slice

- Files: array.go and every file that stores a sim array (upstream
  src/array.h, src/array.c), broad_phase.go (upstream src/broad_phase.c
  `b2PairQueryCallback` heap pairs), distance.go (upstream src/distance.c
  `b2MakeOffsetProxy` points and `b2ShapeDistance` simplex output),
  step.go (upstream src/solver.h `bulletBodies` and `bulletBodyCount`),
  shape.go and body.go (upstream src/types.h `b2ChainDef.points` and
  `.materials`, src/shape.h `b2ChainShape.shapeIndices`, and the
  `count`-plus-buffer accessors `b2Body_GetShapes`, `b2Body_GetJoints`,
  `b2Body_GetContactData`, `b2Shape_GetContactData`,
  `b2Chain_GetSegments` and `b2Shape_GetSensorOverlaps`)
- Tier: T2
- Reason: the reference generates one array type per element type with
  macros. Go has no macros, and the slice already carries the length and the
  capacity that the generated struct tracks by hand.
- Behaviour: every upstream array is a Go slice. `removeSwap` keeps the
  swap-remove contract and returns the old index of the moved element, so
  the caller repairs the stored index as the reference does. The growth
  policy is the one of the Go runtime; the capacity never enters a
  simulation result. A step allocates only when a slice, the arena or a
  graph color grows past its capacity, which happens on the first step that
  activates a contact and then stays flat; the reference grows its arrays
  and its arena at the same moments. The pair nodes of the broadphase
  live in one slice per worker that grows by append and keeps its
  capacity between steps; the reference takes slots from a shared array
  by an atomic index and single pairs from the heap when it runs out. The
  offset proxy takes a point slice, and the distance solver writes its
  simplex trace into a slice whose length is the capacity of the
  reference. The bullet buffer of the step is a slice over one arena item,
  sized to the awake body count, with the count beside it. `ChainDef`
  takes `Points` and `Materials` as slices in place of a pointer and a
  count; a chain keeps its segment shape ids in a `shapeIndices` slice.
  Every reference function that fills a caller array and returns the
  written count keeps that shape: `BodyId.GetShapes`, `BodyId.GetJoints`,
  `BodyId.GetContactData`, `ShapeId.GetContactData`,
  `ChainId.GetSegments` and `ShapeId.GetSensorOverlaps` each take a
  caller-owned slice and return how many entries they wrote, stopping
  early when the slice is shorter than the available data. A destroyed
  solver set hands its sim storage to a spare slot of the world, and the
  next island that falls asleep takes it when it is large enough; the
  reference frees the arrays of a destroyed set and allocates fresh ones
  for the next sleeping set. Go zeroes every new slice and collects the old
  one, which in a world that wakes and sleeps a large island many times per
  step cost more than moving the island. Only the storage is shared: a set
  that takes it starts empty, and the spare never backs two sets at once.
- Test: TestCreateAndDestroyOrdersProduceTheSameWorld and
  TestSleepingBodyGetsItsOwnSolverSet in world_test.go,
  TestSleepReusesTheStorageOfTheLastWokenSet in solver_set_test.go,
  TestStepAllocatesNothing and TestStepBulletStopsAtADynamicPlate in
  step_test.go, TestShapeDistanceWarmStartsFromTheCache in
  distance_test.go, TestCreateChainOpenBuildsSegmentsWithGhosts and
  TestChainSetFrictionReachesEverySegment in shape_test.go, and
  TestGetContactDataReturnsTouchingManifold in body_test.go

### D-011 The determinism witness is port-only

- Files: checksum.go (no upstream counterpart)
- Tier: T2
- Reason: the reference runs on floats and promises no cross-platform state
  equality. A fixed-point world does, and the promise needs a witness that
  the tests and a network peer can compare.
- Behaviour: `Checksum` folds the deterministic world configuration and the
  complete simulation state of every live body, shape and contact. Body,
  shape and contact hashes use wrapping sums, so the iteration order of the
  storage does not affect the result. Application data stays out because it
  cannot change the simulation. Q values enter as raw bits; no float ever
  does. A contact identifies each endpoint by the canonical body and shape
  state instead of storage ids or linked-list keys. Its two orientations fold
  to one value; contact points use a wrapping sum as well. Equivalent worlds
  therefore keep one checksum even when bodies, shapes, contacts or manifold
  points were created in another order. Each body folds whether its island
  has a pending split, because that flag blocks sleep and picks the island
  to split on the next step while no body or contact field shows it; the
  island id itself stays out. The witness contains a real contact that the
  step itself detects and solves, and re-baselines in the same commit that
  grows the fold or changes the solved state. The joints entered the fold
  with their storage: the world folds the joint count and a wrapping sum
  over the joints, each joint by the canonical state of its two bodies,
  its type and its solver data. The witness rebased once in that commit
  because the count entered the hash; the joint solvers did not move it,
  since the witness world has no joint.
- Test: TestChecksumIsOrderIndependent,
  TestChecksumContactsIgnoreCreationOrder, TestChecksumSeesContactState,
  TestChecksumSeesAStateChange, TestChecksumSeesFutureBehaviour,
  TestChecksumSeesAPendingSplit and
  TestChecksumMatchesDeterministicWitness in checksum_test.go,
  TestStepIsReproducibleBitForBit in step_test.go

### D-012 An epsilon guard becomes an exact zero test

- Files: distance.go (upstream src/distance.c:44 segment distance and
  :520 GJK search direction), manifold.go (upstream
  src/manifold.c:24 capsule polygon length assert; 186, 209 vertex-region
  guards; 284 capsule length assert; 495 single-point normal fallback;
  602, 612 and 1206, 1216 clip lerp spans), sensor.go (upstream
  src/sensor.c `b2SensorQueryCallback` overlap distance), world.go
  (upstream src/world.c `b2World_Explode` direction-vector guard)
- Tier: T2
- Reason: `FLT_EPSILON` guards absorb float rounding noise. Q32.32 has no
  such noise, and its smallest magnitude is one raw unit, which already
  exceeds the squared-epsilon threshold of the reference. The exact zero is
  the only dividing line Q can express below its own resolution.
- Behaviour: the degenerate branch of `SegmentDistance` runs only when a
  squared segment length is exactly zero. Every nonzero Q length takes the
  regular path, as it would in float. In `CollidePolygonAndCircle` the
  vertex region needs an exactly positive separation. In `CollideCapsules`
  and the capsule polygon builder a zero axis length panics per D-003, and
  an exactly zero closest-point difference selects the perpendicular
  fallback normal. In `clipPolygons` and `clipSegments` the lerp runs on an
  exactly positive span; the preceding disjoint test bounds the span, so in
  Q the guarded false branch is unreachable and the guard stays only for
  structure. In `ShapeDistance` the overlap exit on a short search
  direction fires only when the direction is exactly zero; every nonzero
  direction adds a support point, so the duplicate test and the iteration
  bound end the loop, as they do in float. The sensor overlap test in
  `sensorQueryCallback` requires an exactly zero shape distance rather
  than an epsilon band, so two shapes that are merely close, but not
  touching, produce no overlap. `WorldId.Explode` guards the direction
  from an explosion center to a struck shape's closest point the same
  way: only an exactly zero-length vector falls back to `(1, 0)`. In the float
  mode the reference guard applies; see D-017.
- Test: TestSegmentDistanceHandlesDegenerateSegments and
  TestShapeDistanceReportsOverlap in distance_test.go,
  TestCollidePolygonAndCircleRegions,
  TestCollideCapsulesFallsBackOnCoincidentClosestPoints,
  TestCollidePolygonsClipsThePartialOverlap and
  TestCollideSegmentAndPolygonRejectsADegenerateSegment in manifold_test.go,
  the clipSegments tests in manifold_internal_test.go,
  TestSensorTouchingEdgeIsNotOverlap in sensor_test.go, and
  TestExplodeImpulseByDistance in world_test.go

### D-013 The pairs of a moved proxy are sorted by shape id

- File: broad_phase.go (upstream src/broad_phase.c `b2PairQueryCallback`)
- Tier: T2
- Reason: the reference prepends each new pair to the list of the moved
  proxy, so the contact creation order follows the walk of the tree. Two
  trees with the same leaves and a different topology, as a rebuild or a
  different insertion history produces, would create the contacts in a
  different order, and the contact ids, the graph colors and the solver
  order would follow. The port promises the same world for any equivalent
  tree.
- Behaviour: the callback appends the pairs of a moved proxy, which stay
  contiguous, and `linkMovePairs` sorts them once in ascending `(shapeIdA,
  shapeIdB)` order and links them into the list of that proxy. One sort
  costs O(n log n) in the new pairs of one proxy, where a sorted insertion
  would cost O(n^2). The order across moved proxies stays the order of the
  move array, as upstream. The shape ids come from the pair itself: the
  shape of the smaller proxy key is A, as upstream.
  When one moved proxy finds several new pairs in the same step, this
  ascending order can still differ from the reference's prepend order, so
  the contact ids and the graph colors that `WorldId.Draw` emits can diverge
  from the reference for that step. The golden draw scene avoids this case.
  The `dbox2d_upstream_pairs` build tag links the pairs newest first
  instead, which is the prepend order of the reference, and gives up the
  independence from the tree topology. The port's trees walk their leaves
  in the reference's order, so with this tag float mode matches every
  conformance scene of D-018 bit for bit.
- Test: TestBroadPhasePairsAreSortedByShapeId in broad_phase_test.go and
  TestChecksumIgnoresTheTreeTopology in checksum_test.go, which skip under
  the tag; TestConformance gates the tag in float mode (D-018)

### D-014 A callback with a context becomes a closure

- Files: dynamic_tree.go (upstream src/dynamic_tree.c `b2DynamicTree_Query`,
  `b2DynamicTree_RayCast` and `b2DynamicTree_ShapeCast`,
  include/box2d/collision.h `b2TreeQueryCallbackFcn`,
  `b2TreeRayCastCallbackFcn` and `b2TreeShapeCastCallbackFcn`), world.go
  (upstream src/world.c `WorldQueryContext`, `WorldRayCastContext`,
  `WorldMoverCastContext` and `b2RayCastClosestFcn`), solver.go (upstream
  src/solver.c `b2ContinuousContext` and `b2ContinuousQueryCallback`),
  types.go (upstream include/box2d/types.h `b2DebugDraw`, `b2FrictionCallback`,
  `b2RestitutionCallback`, `b2CustomFilterFcn`, `b2PreSolveFcn` and
  `b2PlaneResultFcn`)
- Tier: T2
- Reason: the reference passes a function pointer and a `void*` context.
  Go closes over the context instead, and a typed closure keeps the call
  site legible.
- Behaviour: the tree walks take a closure with the proxy id and the user
  data. The public `OverlapResultFcn` and `CastResultFcn` drop the
  `void*` context of the reference as well; the caller closes over its
  state. `DebugDraw` uses the same closure rule for its nine host callbacks.
  The shape queries of the world and the closest ray hit close over
  their result as well. The continuous stage passes a method value of its
  context to the tree walks. A closure that does not escape allocates
  nothing; the step and bullet benchmarks pin zero allocations.
  `FrictionCallback`, `RestitutionCallback`, `CustomFilterFcn` and
  `PreSolveFcn` drop the reference's `void* context` the same way: a
  caller that needs state closes over it. `WorldId.CollideMover` takes a
  `PlaneResultFcn` closure in place of a callback and context pair, so the
  mover's plane collection loop can build its `[]CollisionPlane` directly
  in the closure.
- Test: TestTreeQueryReportsTheOverlaps, TestTreeRayCastClipsTheRay and
  TestTreeShapeCastMatchesBruteForce in dynamic_tree_test.go;
  TestOverlapAABBReportsTheFatBounds, TestCastRayClipsAcrossTheTrees,
  TestShapeQueriesMatchBruteForce and TestCastMoverStopsAtTheWall in
  world_test.go; TestStepBulletStopsAtADynamicPlate,
  TestPreSolveFalseLetsTheBoxThrough and TestMoverStopsOnTheGround in
  step_test.go; TestSetFrictionCallbackAffectsNextContact and
  TestCustomFilterRejectsPair in world_test.go

### D-015 The memory dump writes to an io.Writer

- Files: world.go (upstream src/world.c `b2World_DumpMemoryStats`)
- Tier: T2
- Reason: Go has no reason to open a fixed file name; the caller chooses
  the sink.
- Behaviour: `WorldId.DumpMemoryStats` takes an `io.Writer` in place of
  opening `box2d_memory.txt`, and writes the same six sections with the
  same labels: id pools, world arrays, broad-phase, solver sets,
  constraint graph and the stack allocator. Byte counts come from slice
  capacity times element size, the hash set and bit set backing storage,
  and the dynamic tree's struct size plus its nodes and rebuild scratch.
  The reference's `island sim` line multiplies the summed capacity by
  `sizeof(int)`, a bug from reusing an accumulator variable name as the
  sizeof argument; the port multiplies by the actual island sim size
  instead. The reference's trailing chain shapes section is left as
  `todo` and stays absent here.
- Test: TestDumpMemoryStatsListsEverySection in world_test.go

### D-016 The port owns its workers

- Files: executor.go, executor_wasm.go, executor_pool.go, solver.go,
  step.go, broad_phase.go, sensor.go, types.go (upstream
  include/box2d/types.h `b2WorldDef.workerCount`, `enqueueTask`,
  `finishTask`, `userTaskContext`; src/world.c `b2DefaultAddTaskFcn`;
  src/solver.c `b2SolverStage`, `b2ExecuteMainStage`, `b2SolverTask`,
  `b2FinalizeBodiesTask`, `b2BulletBodyTask`; src/broad_phase.c
  `b2FindPairsTask`; src/sensor.c `b2SensorTask`)
- Tier: T2
- Reason: the reference lends its work to a task system the caller
  supplies through callbacks. A Go package has goroutines and needs no
  callback to run a loop in parallel; a fixed-point port also needs a
  guarantee the reference does not give, that the worker count never
  changes a bit.
- Behaviour: `WorldDef.WorkerCount` replaces the three task fields. 1
  steps on the calling goroutine with no goroutine and no atomic; 0 picks
  `min(GOMAXPROCS, 64)`; `js` and `wasip1` always step on one worker. The
  world owns a pool of `WorkerCount-1` goroutines that starts on the
  first `Step` and stops in `DestroyWorld`. The solver script is the one
  of the reference: stages of blocks, work stealing by compare-and-swap,
  sync bits per stage, the overflow constraints on worker 0. The parallel
  loops outside the script (collide, pair finding, sensors, body
  finalization, bullet sweeps) split into contiguous ascending ranges and
  range k always runs on worker k, so every per-worker result joins in
  worker order: the bit sets by union, the bullet list by concatenation,
  the split candidate by the first worker on a tie, as the first body
  wins inside one worker; the reference breaks that tie by island id
  because its work stealing has no order. That rule makes the reference
  depend on how its bodies split across workers: when two islands tie on
  sleep time, one worker keeps the first body it meets and two workers
  keep the larger island id. In the rain scene of D-018 the reference
  with 2, 4 or 8 workers leaves its own single-worker run at step 450, and
  the three counts also disagree with each other; the reference's own
  determinism test runs only falling_hinges, which has no such tie. The
  port's rule is the single-worker choice of the reference: a copy of the
  reference patched to prefer the smallest awake index gave the same
  trace at 1, 2, 4 and 8 workers. The upstream documentation (docs/FAQ.md,
  docs/simulation.md) promises the same result for any thread count, so
  this is a v3.1.1 bug, not a documented limit. Upstream fixed it without a
  note in commit 436365a246 (PR #1065, "Snapshot recording", after
  v3.1.1): a worker breaks a sleep-time tie by the larger island id, as the
  cross-worker reduction already did. When the port moves to a release
  with that commit, the candidate test in `finalizeBodiesTask` and the
  reduction in `solve` both take the larger island id on a tie. The
  choice becomes a maximum over (sleep time, island id) that no longer
  needs the ordered ranges, the worker-order clause above goes, and the
  witnesses and traces change with the release. The pair nodes of a moved proxy
  live in the slice of the worker that queried it, and the move result
  records that worker. The island split runs on the last worker beside
  the script. The broad-phase tree rebuild is the one task the pool
  cannot carry, because every parallel loop between its start and its
  join claims all of the workers: it gets a goroutine of its own, parked
  on a channel between steps, started at the top of the narrow phase and
  joined at the refit, so it overlaps the narrow phase and the whole
  solve as the reference task does. With one worker the island split runs
  inline first and the rebuild runs inline at the join. A zero time step
  skips the solve and so skips the refit; the port joins the rebuild
  before the sensor overlap, where the reference leaks the task handle
  and lets the sensor queries read a tree that is still being rebuilt.
  The workers spin between commands and
  park after a while; a parked worker may take one runtime object when
  it wakes, so the allocation gate holds per step, not per process. A
  panic inside a step with several workers leaves the pool waiting on the
  script: the world can no longer step or be destroyed, and a panic on a
  worker goroutine ends the process. A
  step with fewer awake bodies than
  `serialBodyThreshold` runs on the caller whatever `WorkerCount` says;
  the reference has no such threshold. The custom filter, the pre-solve
  and the mixing callbacks run on worker goroutines, as in the reference.
  `Counters.TaskCount` counts the tasks of the last step and does not
  depend on the worker count. The stage is the
  seam: a SIMD or a GPU family replaces the body of one stage and keeps
  the script.
- Test: TestStepIsWorkerCountIndependent in checksum_test.go,
  TestSensorEventsAreWorkerCountIndependent in sensor_test.go,
  TestCountersReportTasks in world_test.go, and the executor tests in
  executor_test.go

### D-017 Two scalar modes

- Files: scalar_fixed.go, scalar_float.go, aabb.go, distance.go, geometry.go,
  manifold.go, sensor.go, world.go and checksum.go
- Tier: T2
- Reason: the reference computes in float32. A float mode gives a line-by-line
  float comparison against the reference and isolates the cost of the fixed
  format.
- Behaviour: the two modes share every file except `scalar_fixed.go` and
  `scalar_float.go`; only those scalar files select the mode, apart from the
  executor platform files and tests. `Q` is a struct in both modes: a
  `fixed.Q32` alias in fixed mode and a struct with one float32 field, `v`, in
  float mode. Only the constructors make a scalar for callers, so the rest of
  the package compiles identically in both modes.

  Fixed mode uses Q32.32 from `github.com/dhannyell/fixed`. Float mode uses
  plain float32 for Add and Sub. Every product and quotient sits inside an
  explicit float32 conversion. The conversion is a rounding point in the Go
  spec and prevents fusion with a following add, at no cost in instructions. CI cross-compiles float mode to arm64
  with `-gcflags=-S` and requires zero `FMADD`, `FMSUB`, `FNMADD`, `FNMSUB`,
  `FMLA` and `FMLS` instructions. The pinned float witness is checked on
  amd64, arm64, 386 and wasm.

  An angle is a turn in both modes, per D-004. Fixed mode keeps the CORDIC-
  style functions of the fixed module. Float `MakeRot` unwinds turns, converts
  with one product to radians, and applies the reference Bhaskara
  `b2ComputeCosSin` approximation in float32. `atan2Turns` applies the
  reference `b2Atan2` polynomial and divides by two pi. It does not call
  `math.Sin`, `math.Cos` or `math.Atan2`.

  Epsilon guards use exact zero in fixed mode. Float mode uses the reference
  FLT_EPSILON forms through `scalarEpsilon`, `scalarEpsilonSq`,
  `belowEpsilon`, `belowEpsilonSq` and `sensorOverlaps`, all defined per mode.
  A reciprocal goes through `makeRecip`, also per mode: fixed mode keeps the
  denominator and divides, float mode multiplies by one rounded reciprocal
  as the reference does (D-006). The angle unit of a joint is per mode as
  well: `angleFromTurns`, `angleToTurns`, `angleRadians`, `relativeAngle`
  and `unwindAngle` keep turns in fixed mode and radians in float mode
  (D-004).
  Fixed `IsValidQ` rejects the two saturation values. Float `IsValidQ` rejects
  NaN and Inf as `b2IsValidFloat` does, and `QMaxValue()` is FLT_MAX and is
  valid. `normalizedTolerance` is 2^-16 in fixed mode and 100 times
  FLT_EPSILON in float mode. The checksum folds raw Q32.32 bits in fixed mode
  and `math.Float32bits` in float mode. `Round` follows roundf and `Int`
  follows the C cast in both modes: halves go away from zero and integers
  truncate toward zero. `Vec2.Normalize` returns the zero vector below
  FLT_EPSILON in float mode, as `b2Normalize` does, and only at exact zero in
  fixed mode.

  The solver stages, order of operations, iteration counts, worker rules,
  `Huge` (100000), every constant in `constants.go` and the checksum structure
  never change by mode. `QFromFloat64` and `QToFloat64` exist for camera and
  other presentation conversions; simulation code never calls them, and CI
  checks that boundary. Only fixed mode promises that a contact checksum is
  unchanged when shapes A and B are swapped; the mirrored manifold rounds
  identically only in exact arithmetic. That property has a fixed-only test.
- Test: `scalar_float_test.go`; `TestChecksumMatchesDeterministicWitness` with
  the witness for each mode; the matching `_fixed_test.go` and
  `_float_test.go` pairs; `qUlps(n)` for n raw fixed units or n float32 ulps at
  one; the per-mode `mirrorTolerance`; and the CI FMA gate.

### D-018 Conformance traces

- Files: conformance_test.go, tools/conformance/main.c, testdata/conformance/ (upstream
  shared/benchmarks.c, shared/determinism.c, the collision functions of src/manifold.c,
  src/distance.c, src/hull.c and src/math_functions.c)
- Tier: T2
- Reason: the reference has no trace format and no bit-level oracle. The witness of D-011 pins
  the port to itself. A conformance trace pins the port to the reference C compiled without SIMD
  and without FMA.
- Behaviour: The generator in tools/conformance/ links to the frozen reference and writes text
  traces: 17 function files (the twelve b2Collide* manifolds, b2ShapeDistance, b2TimeOfImpact,
  b2ComputeHull, b2MakeRot, b2Atan2) with inputs drawn by the generator, and 8 scene files (the
  seven benchmark scenes and falling_hinges) with one hash of all body transforms per step and
  full dumps at step 1 and the last step for scenes of at most 5 000 bodies.
  Floats are binary32 bit patterns. See tools/conformance/README.md for the format and
  testdata/conformance/SOURCE.md for the compiler and flags.

  TestConformance reads every trace in both modes. Function traces: float mode is gated in ulps
  and is exact (0 ulps) for 16 of 17 files. The one non-zero budget is make_rot 1328 ulps
  (D-004: the turn is converted to a radian before the reference approximation). Fixed mode is
  gated by an absolute budget of twice the measured residue: the twelve manifolds between 1e-6
  and 4e-5, shape_distance 8e-6, time_of_impact 1e-7, make_rot 4e-3 and atan2 6e-5 (D-017:
  CORDIC against the reference polynomials). compute_hull is exact after a cyclic alignment: the
  first hull vertex is the point farthest from the AABB center, and a near tie can start the hull
  at another vertex in Q32.32.

  Scene traces: fixed mode does not match the reference bit for bit. Float mode with the pair
  order of the reference (the `dbox2d_upstream_pairs` tag of D-013) matches every scene bit for
  bit, in the scalar and the SIMD families: the dumps are exact and the hash is equal at every
  step. A joint keeps its angles in radians in float mode (D-004), and the falling_hinges builder
  makes its body rotations from radians in float mode, as the reference does. The tumbler
  builder stores the motor speed of the reference, `(B2_PI / 180.0f) * 25.0f`, in float mode: no
  binary32 turn rate times 2π rounds to that value.

  The default build sorts the pairs (D-013), which changes which contacts get a graph color when
  one body finds several new pairs in one step. The pyramid scenes and the spinner diverge at
  step 1 for that reason; colored and overflow contacts clamp their bias by different speeds
  (contactSpeed and maxContactPushSpeed, per b2SolveContactsTask and b2SolveOverflowContacts),
  so the spinner bar diverges by 4e-3 at step 1 with an identical set of contacts. Later steps
  diverge chaotically. So the default gate is the step-1 dump, with a budget per scene and per
  mode: tumbler 2e-5 fixed and exact in float, rain 4e-5 and exact, falling_hinges 4e-3 and
  exact, spinner 1e-2 in both. The per-step hash and the later dumps are logged, not gated: the
  test reports the first divergent step and the largest residue of each sampled step. In float
  mode the test also gates the number of leading steps with an equal hash: falling_hinges 158,
  joint_grid 500, rain 171, smash 69 and tumbler 6. Under the tag it gates exact dumps and an
  equal hash at every step instead. Scenes over 5 000 bodies (joint_grid,
  large_pyramid, many_pyramids, smash) have no step-1 dump, so only the body count and the hash
  report apply. Scene traces are skipped under `go test -short`: they take minutes on 32-bit and
  wasm targets, where fixed mode is bit-identical by construction and the witness hashes already
  hold.

  The traces come from the reference with one worker. A reference run with 2, 4 or 8 workers,
  through the task pool of tools/cbench, differs from it only in rain, from step 450 on, because
  of its split-island tie (D-016). The port with 1, 4 or 8 workers, in the scalar and the SIMD
  families, matches the single-worker trace in every scene. The SIMD builds of the reference, SSE2
  and AVX2, write the same bits as the `BOX2D_DISABLE_SIMD` build at the same worker count, so
  the traces stand for them as well.

  The traces found one port bug: the colored contacts clamped by maxContactPushSpeed instead of
  contactSpeed; the fix changed both witnesses. They found a second: the colored contacts took
  the normal impulse of b2SolveOverflowContacts, `-m * s * (vn + b) - i * p` grouped from the
  left, where b2SolveContactsTask computes `p - (m * (s * (vn + b)) + i * p)`, with m the normal
  mass, s the mass scale, b the velocity bias, i the impulse scale and p the accumulated
  impulse. Each mode rounds the two groupings differently. The colored contacts now take the
  grouping of the task family, in the scalar, Q32 and SIMD families, and the overflow contacts
  keep theirs; the fix changed both witnesses.
- Test: TestConformance in conformance_test.go

### D-019 SIMD contact family

- Files: contact_solver_simd.go, simd_off.go, solver.go, simd_lane_float.go,
  simd_lane_float_archsimd.go, simd_lane_amd64.go, simd_lane_arm64.go,
  simd_lane_wasm.go, simd_lane_generic.go, simd_lane_gather_generic.go,
  simd_lane_fixed.go, scalar_fixed.go, scalar_float.go
- Tier: T2
- Reason: the reference selects a SIMD `Task` family with `B2_SIMD_WIDTH` lanes
  at compile time, including an `B2_SIMD_NONE` variant that keeps four scalar
  lanes with no vector instructions. The port has no scalar-lane variant: its
  oracle is the scalar family in contact_solver.go, which serves every color
  and the overflow color and computes the normal impulse of a colored contact
  with the grouping of the `Task` family (D-018). A SIMD lane also needs a fixed number of
  contacts per call, and a color rarely holds a multiple of the lane width.
- Behaviour: the `dbox2d_simd` tag adds a second contact-solving path beside
  the scalar family, in both modes. Contacts of
  each color are padded to a multiple of the lane width; the padding lanes
  hold a null contact index and an identity body state, and the store step
  never writes them back. The solver stage table sizes each color's SIMD
  constraint block by `⌈n/width⌉`, mirroring the reference
  `colorContactCountSIMD`. On avx2, the gather loads each `bodyState` as one
  32-byte row; its `flags` field is `int32` for that reason, and the reference
  asserts the same 32-byte size. It transposes eight rows into lanes with
  shuffles. The scatter transposes back and stores whole rows for the real
  lanes. The arm64 and generic paths gather through a scalar scratch. The body
  state holds the angular velocity in radians per second (D-004), so every
  path loads and stores it unchanged.

  Four lane implementations share one padding and dispatch layer:
  avx2 (amd64, width 8), neon (arm64, width 4), simd128 (wasm, width 4), and
  a generic path of four named float32 fields for every other target,
  including a build without `GOEXPERIMENT=simd`. avx2 falls back to the
  scalar family at runtime when the running CPU has no AVX2. The port never fuses a multiply with an add or
  a subtract: `MulAdd` and `MulSub` are two rounded operations on every lane
  path, where the reference `b2MulAddW` is unfused on SSE2 and AVX2 but fused
  through `vmlaq_f32` on NEON. `Min` and `Max` are compare-and-select, so a
  signed-zero tie follows the same `if a < b` rule as the scalar family. The
  result is that the SIMD family is bit-identical to the scalar family, and
  therefore bit-identical across ISAs, where the reference is not. There is no
  SSE2 path. A velocity component that is exactly -0 becomes +0 when it
  passes an empty second manifold point or a masked point of a contact with
  restitution, because the lane computes `v - (-0)`; this is unreachable from
  +0 states, and the reference behaves the same way. A contact without
  restitution does not write its bodies back, as in the scalar stage; in
  fixed mode the write would round a velocity left by a Q32 contact of a
  later color (D-020).

  In fixed mode the lanes come from the fixed module: Q16.16 lanes with
  Q48.16 accumulators, the grid of the scalar contact stages (D-020). The
  module selects avx2 (width 8), neon (width 4) or a generic path; wasm runs
  the generic path. Every fixed path gathers through a scalar scratch. The
  scratch carries the velocities in Q48.16 and the position deltas in Q16.16,
  as the scalar contact stages do. It converts the angular velocity to radians
  per second in Q32.32 and rounds the result to the grid, as the scalar family
  does, because the lane grid is too coarse for that product. The velocity
  accumulations skip the
  Q48 overflow check, because their budget stays far inside the Q48 range
  (`AddBounded`). A pack with no rolling resistance and no stored rolling
  impulse skips the rolling blocks, which would only add zero. Float mode
  keeps them, because there a skipped block can change the sign of a zero. The
  two families match while no contact value saturates (D-020).
- Test: TestWidePath, TestWideMatchesScalarStepByStep,
  TestWideStagesRunWithColoredContacts
  and TestWideContactLayoutPadsEachColor in simd_test.go, in both modes;
  simd_lane_test.go checks each float lane operation against the scalar
  family on every path, and the fixed module tests its own lanes; the
  witness, samples and conformance suites all run under the `dbox2d_simd` tag
  in both modes in CI, on amd64 and arm64 with `GOEXPERIMENT=simd` and on the
  generic path across the four-architecture matrix.

### D-020 Contacts solve on a Q16 grid

- Files: contact_solver.go, contact_solver_q32.go, scalar_fixed.go,
  scalar_float.go, internal/q32gen, tools/q32gen
- Tier: T2
- Reason: a fixed-point SIMD lane holds Q16.16 values, so a Q32.32 contact
  solver could never match it bit for bit. The scalar family is the oracle of
  the SIMD family (D-019), so both families solve contacts on the same grid.
- Behaviour: in fixed mode the contact stages use `qc`, a Q16.16 value whose
  products round to nearest, and `qa`, a Q48.16 accumulator with the same 16
  fraction bits. The prepare stage computes in Q32.32, as before, and rounds
  each result to the grid: the normal, the anchors, the separation, the
  masses, the coefficients and the warm-start impulses. The other stages load
  the body velocities into `qa` and the position deltas into `qc`, both
  rounded to nearest. They compute each product on the grid and accumulate
  each velocity change and the total normal impulse in `qa`. The store writes
  the values back to the Q32.32 body state and manifold without rounding. The
  order of operations does not change. The rolling resistance bound multiplies
  the two-point total in `qa` and narrows only the product, because that total
  can pass the Q16 range while each point fits.

  The grid has a lane window: every nonzero inverse mass and inverse inertia
  of a contact must lie in [2^-6, 2^15). Below the window a coefficient keeps
  fewer than ten bits; at the top it does not fit. Each step partitions the
  contacts of every color once, by that rule. The contacts inside the window
  solve on the Q16 grid, in the scalar or the SIMD family. The contacts
  outside it solve in Q32.32, one stage unit each after the family units of
  their color, so the stage blocks spread them over the workers; the overflow
  color runs them whole after its scalar contacts. The Q32 stages are
  contact_solver_q32.go, which `go generate` derives from contact_solver.go
  with the same operations over Q32.32 values, and a test keeps it fresh. A
  body of 200000 kg rests and slides on the ground with this path. The Q32
  tail admits a Q32 lane later, if a scene with a heavy majority makes it
  worth the cost. In float mode every contact fits the lane and the Q32 path
  is empty.

  In float mode `qc` and `qa` are aliases of `Q` and every conversion is the
  identity, so the float witness does not change. The Q16 range is ±32768. A
  contact value outside it saturates, so the scene traces of TestConformance
  require zero saturations in fixed mode when the build sets
  `fixed_satcounter`. This change moved the fixed witness and four samples
  checksums; the lane window moved the Tumbler checksum, which holds a
  heavy body. The conformance budgets did not move. One grid unit moves a
  bounded friction label of the draw golden by 0.015, so in fixed mode the
  golden accepts one unit in the last printed place of a decimal label.
- Test: TestChecksumMatchesDeterministicWitness; the contact tests in
  contact_solver_test.go, with `contactRounding()` for one rounding to the
  grid; TestHeavyBoxRestsOnTheGround in step_test.go and
  TestStepPartitionsContactsByTheLaneWindow in step_fixed_test.go;
  TestQ32ContactSolverIsFresh; the saturation gate of the scene traces in
  TestConformance.
