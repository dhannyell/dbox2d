// Command q32gen writes contact_solver_q32.go from contact_solver.go. It runs
// from the module root through go generate.
package main

import (
	"fmt"
	"os"

	"github.com/dhannyell/dbox2d/internal/q32gen"
)

func main() {
	src, err := os.ReadFile("contact_solver.go")
	if err != nil {
		fail(err)
	}
	out, err := q32gen.Generate(src)
	if err != nil {
		fail(err)
	}
	if err := os.WriteFile("contact_solver_q32.go", out, 0o644); err != nil {
		fail(err)
	}
}

func fail(err error) {
	fmt.Fprintln(os.Stderr, "q32gen:", err)
	os.Exit(1)
}
