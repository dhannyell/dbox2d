//go:build dbox2d_validate

package dbox2d

// validateSolverSetsDebug runs the solver-set validation from the mutation
// paths that the reference validates. Build with -tags dbox2d_validate to
// turn it on; see validate_release.go for why it is off by default.
func validateSolverSetsDebug(w *world) { validateSolverSets(w) }
