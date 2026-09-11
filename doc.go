// Package dbox2d implements a deterministic 2D rigid body solver.
//
// Equal inputs produce the same result bits on every supported architecture,
// on every run. A world therefore survives a snapshot, a replay and a rollback
// without drift.
//
// The package builds in two scalar modes. The default is float32. The build
// tag dbox2d_fixed selects the Q32.32 fixed-point arithmetic of
// github.com/dhannyell/fixed, with the same determinism promise; see the
// README for the mode details and commands. ScalarMode reports the mode of a
// build.
//
// The optional dbox2d_simd tag opts in to a SIMD contact solver in either
// mode. It produces the same result bits as the scalar solver.
//
// The optional dbox2d_upstream_pairs tag keeps the broadphase pair order of
// Box2D, which follows the walk of the tree. In float mode it reproduces
// the output bits of the reference built without SIMD and FMA, at the cost
// of a world that depends on the tree topology.
//
// # Fidelity
//
// The package is a port of Box2D v3.1.1. It keeps the upstream file
// decomposition, the upstream names without the b2 prefix, and the upstream
// order of operations. The port preserves the upstream formulation, not its
// output bits. Box2D computes in floating point, so the two libraries produce
// different trajectories.
//
// Fixed-point arithmetic forbids some upstream lines, such as an epsilon
// tuned for float or a stopping criterion that depends on it. Each such place
// is a recorded divergence with a reason and a test.
//
// # Angles
//
// Every angle of the API is a turn, not a radian, in both modes: one quarter
// turn is QFromRatio(1, 4), and an angular velocity is in turns per second.
// Inside, the body state keeps radians per second, the unit of the reference.
// A joint keeps its angles in radians in float mode, as the reference does,
// and in turns in fixed mode, because a turn reduces without pi (D-004). The
// solver stores an orientation as a [Rot] sine and cosine pair, never as an
// angle.
//
// # IDs
//
// WorldId, BodyId, ShapeId, ChainId and JointId are opaque handles. Their zero
// values are null, and IDs of the same type can be compared with ==. Pass them
// by value and obtain them from package functions; constructing a packed value
// by hand may make it refer to whatever occupies that slot.
//
// # Accumulation
//
// Saturated addition is not associative near the range limits. The solver
// keeps a fixed accumulation order for that reason. The worker count never
// changes that order: TestStepIsWorkerCountIndependent pins the same checksum
// for one and for several workers.
package dbox2d
