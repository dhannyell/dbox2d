package dbox2d

// This file has no upstream counterpart. A fixed-point world promises the
// same state on every platform, and the checksum is the witness of that
// promise. See D-011.

// FNV-1a parameters for 64 bits.
const (
	fnvOffsetBasis uint64 = 14695981039346656037
	fnvPrime       uint64 = 1099511628211
)

// fnvFold folds one 64-bit word into an FNV-1a hash, byte by byte, from the
// least significant byte.
func fnvFold(h, word uint64) uint64 {
	for range 8 {
		h ^= word & 0xff
		h *= fnvPrime
		word >>= 8
	}
	return h
}

// Checksum folds the motion state of every live body into one integer. Two
// runs that build the same world produce the same checksum, on every
// platform. The fold over the bodies is a wrapping sum, so the checksum does
// not depend on the creation order of the bodies.
func Checksum(worldId WorldId) uint64 {
	w := getWorldFromId(worldId)

	var sum uint64
	for i := range w.bodies {
		b := &w.bodies[i]
		if b.id == nullIndex {
			continue
		}

		set := &w.solverSets[b.setIndex]
		sim := &set.bodySims[b.localIndex]

		h := fnvOffsetBasis
		h = fnvFold(h, uint64(sim.transform.P.X.Raw()))
		h = fnvFold(h, uint64(sim.transform.P.Y.Raw()))
		h = fnvFold(h, uint64(sim.transform.Q.Cos.Raw()))
		h = fnvFold(h, uint64(sim.transform.Q.Sin.Raw()))
		h = fnvFold(h, uint64(sim.center.X.Raw()))
		h = fnvFold(h, uint64(sim.center.Y.Raw()))

		// Only the awake set stores velocities. A body outside it is at
		// rest, and its hash folds nothing more.
		if b.setIndex == awakeSet {
			state := &set.bodyStates[b.localIndex]
			h = fnvFold(h, uint64(state.linearVelocity.X.Raw()))
			h = fnvFold(h, uint64(state.linearVelocity.Y.Raw()))
			h = fnvFold(h, uint64(state.angularVelocity.Raw()))
		}

		sum += h
	}
	return sum
}
