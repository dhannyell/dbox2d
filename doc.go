// Package dbox2d implements a deterministic 2D rigid body solver.
//
// The package computes with the Q32.32 fixed-point arithmetic of
// github.com/dhannyell/fixed. Equal inputs produce the same result bits on
// every supported architecture, on every run. A world therefore survives a
// snapshot, a replay and a rollback without drift.
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
// An angle is a turn, not a radian, because a turn reduces without pi. The
// solver stores an orientation as a [github.com/dhannyell/fixed.Rot] sine and
// cosine pair, never as an angle.
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
// keeps a fixed accumulation order for that reason. An executor that changes
// the order changes the result bits, so it must prove bit equality against the
// scalar path.
package dbox2d
