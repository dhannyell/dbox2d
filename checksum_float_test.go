//go:build !dbox2d_fixed

package b2

// The float witness, proved on four architectures by CI.
// v3.2 phase 2: the shape no longer folds the retired enlargedAABB flag.
// v3.2 phase 3: the body flags fold as one word.
// v3.2 phase 3: the shape folds its AABB margin and its custom filtering
// flag, the body its safety factor, and the fat AABB moved to the world; the
// simulation of the witness scene is unchanged.
const checksumWitness uint64 = 9684624931612710896
