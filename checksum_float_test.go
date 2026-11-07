//go:build !dbox2d_fixed

package b2

// The float witness, proved on four architectures by CI.
// v3.2 phase 2: the shape no longer folds the retired enlargedAABB flag.
// v3.2 phase 3: the body flags fold as one word.
// v3.2 phase 3: the shape folds its AABB margin and its custom filtering
// flag, the body its safety factor, and the fat AABB moved to the world; the
// simulation of the witness scene is unchanged.
// v3.2 phase 5: the solver, contacts and islands switch to v3.2; the body
// no longer folds the retired visit mark.
const checksumWitness uint64 = 4868602092532673147
