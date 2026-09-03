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
  upstream contract is a validation query still return a bool.
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

- Files: math.go, body.go, world.go, solver.go, contact_solver.go, joint.go,
  motor_joint.go and the other six joint files (upstream
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
  a remainder of two pi. `updateBodyMassData` stores the angular velocity in
  turns per second, so it scales the velocity by one turn before the cross
  product that corrects the linear velocity of a moved center of mass.
  `integrateVelocitiesTask` divides the torque delta by one turn, and
  `finalizeBodiesTask` scales the arc speed of the sleep test by one turn.
  `makeSoft` multiplies the frequency by one turn where the reference
  multiplies by two pi. Each stage of the contact solver scales the
  angular velocity by one turn on load and divides it by one turn on
  store, so the cross products with the anchors stay in radians. The joint
  stages do the same. A reference angle, a limit, a target angle and an
  angular offset are turns, and each enters a constraint error or a bias
  multiplied by one turn; a motor speed is turns per second and multiplies
  by one turn before the motor row. The revolute limit check bounds the
  angles at 0.495 turns, the `0.99 * pi` of the reference. Every joint angle
  accessor, including `GetAngularSeparation`, reports turns; the motor
  joint's `SetAngularOffset` and `GetAngularOffset` keep the same unit.
  `BodyId.ApplyTorque` accumulates the torque as given, because the turn
  conversion happens where the solver consumes it, but
  `BodyId.ApplyAngularImpulse` divides by `tau` on the spot to turn the
  reference's radian impulse into turns per second, and
  `BodyId.SetTargetTransform` scales its angular velocity target the same
  way through `RelativeAngle`, which already returns turns.
  `WorldId.Explode` divides by `tau` as well, converting the angular
  impulse of each struck body from the reference's radians to turns per
  second.
- Test: TestIntegrateRotationCompletesATurn,
  TestComputeAngularVelocityInvertsIntegration and
  TestUnwindAngleReducesToHalfTurn in math_test.go,
  TestBodyMassComesFromItsShapes in world_test.go,
  TestStepConvertsTorqueAndArcSpeedToTurns in step_test.go,
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
  rotation and box checks build on it.
- Test: TestSaturationMarksAValueInvalid in math_test.go

### D-006 A reciprocal becomes a division

- Files: math.go, aabb.go, geometry.go, body.go, world.go, solver.go,
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
  `BodyId.SetTargetTransform` divides by the caller's time step instead of
  multiplying by its reciprocal. `BodyId.SetMassData` divides by the mass
  and by the rotational inertia to store their inverses, guarded by the
  same exact zero test as the shape-driven mass update. `WorldId.Explode`
  divides the falloff band by its length to build the fade-out scale.
- Test: TestSolve22SolvesTheSystem, TestNormalizeKeepsAShortVector and
  TestNormalizeRotKeepsAZeroRotation in math_test.go,
  TestAABBRayCastHitsTheNearFace in aabb_test.go,
  TestPolygonCentroidOfATriangle, TestTriangleMassMatchesTheReference and
  TestRayCastCapsuleHitsTheSide in geometry_test.go,
  TestStepAppliesDampingByDivision in step_test.go, and
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
  joint is `huge`, the `B2_HUGE` of the reference: a length that no world
  reaches and that the range checks accept, so the rope stays slack until
  a definition lowers it. `SetContactTuning` clamps its hertz, damping
  ratio and push speed against `Q32MaxValue` rather than `FLT_MAX`, so a
  caller cannot saturate the tuning past what a Q32.32 value can hold.
  `SolvePlanes` takes its rigid push limit as a plain `PushLimit` field;
  the mover callers pass `huge`, the same `B2_HUGE` sentinel, in place of
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
  and its arena at the same moments. The pair slice of the broadphase
  grows by append when the sixteen pairs per moved proxy run out; the
  reference takes single pairs from the heap at the same moment. The
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
  early when the slice is shorter than the available data.
- Test: TestCreateAndDestroyOrdersProduceTheSameWorld and
  TestSleepingBodyGetsItsOwnSolverSet in world_test.go,
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
  way: only an exactly zero-length vector falls back to `(1, 0)`.
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
- Behaviour: the callback inserts each pair in ascending `(shapeIdA,
  shapeIdB)` order in the list of its moved proxy. The order across moved
  proxies stays the order of the move array, as upstream. The shape ids
  come from the pair itself: the shape of the smaller proxy key is A, as
  upstream.
- Test: TestBroadPhasePairsAreSortedByShapeId in broad_phase_test.go and
  TestChecksumIgnoresTheTreeTopology in checksum_test.go

### D-014 A callback with a context becomes a closure

- Files: dynamic_tree.go (upstream src/dynamic_tree.c `b2DynamicTree_Query`,
  `b2DynamicTree_RayCast` and `b2DynamicTree_ShapeCast`,
  include/box2d/collision.h `b2TreeQueryCallbackFcn`,
  `b2TreeRayCastCallbackFcn` and `b2TreeShapeCastCallbackFcn`), world.go
  (upstream src/world.c `WorldQueryContext`, `WorldRayCastContext`,
  `WorldMoverCastContext` and `b2RayCastClosestFcn`), solver.go (upstream
  src/solver.c `b2ContinuousContext` and `b2ContinuousQueryCallback`),
  types.go (upstream include/box2d/types.h `b2FrictionCallback`,
  `b2RestitutionCallback`, `b2CustomFilterFcn`, `b2PreSolveFcn` and
  `b2PlaneResultFcn`)
- Tier: T2
- Reason: the reference passes a function pointer and a `void*` context.
  Go closes over the context instead, and a typed closure keeps the call
  site legible.
- Behaviour: the tree walks take a closure with the proxy id and the user
  data. The public `OverlapResultFcn` and `CastResultFcn` drop the
  `void*` context of the reference as well; the caller closes over its
  state. The shape queries of the world and the closest ray hit close over
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
