//go:build !dbox2d_fixed

package b2

// The float witness, proved on four architectures by CI.
// v3.2 phase 2: the shape no longer folds the retired enlargedAABB flag.
const checksumWitness uint64 = 3640157120542509367
