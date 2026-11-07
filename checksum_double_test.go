//go:build dbox2d_double && !dbox2d_fixed

package b2

// The double witness. Under the dbox2d_double tag the checksum folds the bits
// of each world position as two float64 values (checksumPos) instead of two
// float32 values, so the same simulation hashes to a different integer than
// the float witness. The simulation of the witness scene is the same one the
// float build runs. The value is pinned on the build host for this tag.
const checksumWitness uint64 = 437848164999225205
