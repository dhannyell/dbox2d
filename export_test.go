package dbox2d

// The test suite runs in two packages. The in-package one reaches the
// solver's internals directly; the external one, which holds the
// conformance suite and the scene benchmark, cannot, because it imports
// internal/scenes and internal/scenes imports this package. This file is
// the whole of the boundary: it is a test file, so none of it reaches the
// public API of the library.

// UpstreamPairOrder reports whether this build restores the broadphase pair
// order of the reference (D-013).
func UpstreamPairOrder() bool { return upstreamPairOrder }

// FNVOffsetBasis is the seed of the FNV-1a hash the checksum uses.
const FNVOffsetBasis = fnvOffsetBasis

// FNVFold folds one 64-bit word into an FNV-1a hash.
func FNVFold(h, word uint64) uint64 { return fnvFold(h, word) }

// Atan2Radians returns the arc tangent in radians in either scalar mode.
func Atan2Radians(y, x Q) Q { return conformanceAtan2Radians(y, x) }

// RotFromRadians builds the rotation of b2MakeRot from a radian angle, the
// way a reference scene does. Float mode reaches the internal radian
// constructor, which no public call can express: MakeRot takes a turn and
// multiplies it by tau, and no binary32 turn recovers an arbitrary radian
// angle.
func RotFromRadians(radians float32) Rot { return conformanceRotFromRadians(radians) }

// ActiveWorkerCount reports the largest worker index the executor has handed
// out, which the scene benchmark records per scene. A world with few awake
// bodies runs on the caller's goroutine however many workers it was given,
// so a benchmark that does not observe this cannot tell a parallel run from
// a serial one.
func ActiveWorkerCount(worldId WorldId) int {
	return getWorldFromId(worldId).executor.activeWorkerCount()
}

// SetRevoluteMotorSpeedRadians stores a motor speed in the unit a joint keeps
// internally in float mode, bypassing the turn of the API (D-004).
//
// It exists for the tumbler of the reference, whose motor is
// (B2_PI / 180.0f) * 25.0f radians per second. No binary32 turn rate times
// floatTau rounds back to that value, so a scene that has to match the
// reference bit for bit stores the radians. Nothing outside the conformance
// scenes should want this: a value written here does not round-trip through
// GetMotorSpeed, which answers in turns.
func SetRevoluteMotorSpeedRadians(worldId WorldId, jointId JointId, radians Q) {
	joint := getJointSimCheckType(getWorldFromId(worldId), jointId, RevoluteJoint)
	joint.revoluteJoint.motorSpeed = radians
}

// BodyIndex returns the id index a body was created with, which orders a
// world's bodies the same way on every run and so makes a scene hash
// reproducible. The public API has no order over BodyId on purpose.
func BodyIndex(bodyId BodyId) int { return int(bodyId.index1) }
