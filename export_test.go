package dbox2d

// Internal tests reach unexported state directly. The external conformance
// and benchmark tests use internal/shared, so these test-only bridges keep
// that package boundary acyclic.

// UpstreamPairOrder reports whether this build restores the broadphase pair
// order of the reference (D-013).
func UpstreamPairOrder() bool { return upstreamPairOrder }

const FNVOffsetBasis = fnvOffsetBasis

func FNVFold(h, word uint64) uint64 { return fnvFold(h, word) }

// Atan2Radians returns the arc tangent in radians in either scalar mode.
func Atan2Radians(y, x Q) Q { return conformanceAtan2Radians(y, x) }

// RotFromRadians builds a reference rotation from radians. The public API
// accepts turns, which cannot represent every reference float32 angle.
func RotFromRadians(radians float32) Rot { return conformanceRotFromRadians(radians) }

// ActiveWorkerCount reports the highest worker index used by a step. Small
// scenes may run on the caller, so the benchmark uses this to detect that.
func ActiveWorkerCount(worldId WorldId) int {
	return getWorldFromId(worldId).executor.activeWorkerCount()
}

// SetRevoluteMotorSpeedRadians sets the tumbler motor in the internal float
// unit. The reference radian value cannot be recovered exactly through the
// public turn-based API, so this test-only bridge bypasses it.
func SetRevoluteMotorSpeedRadians(worldId WorldId, jointId JointId, radians Q) {
	joint := getJointSimCheckType(getWorldFromId(worldId), jointId, RevoluteJoint)
	joint.revoluteJoint.motorSpeed = radians
}

// BodyIndex exposes creation order for deterministic scene hashes. BodyId
// intentionally has no public ordering.
func BodyIndex(bodyId BodyId) int { return int(bodyId.index1) }
