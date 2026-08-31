package dbox2d_test

import (
	"go/parser"
	"go/token"
	"os"
	"strings"
	"testing"
)

// TestArrowRule limits production imports to the approved modules. The solver
// depends on the fixed-point module and on nothing else, so a consumer never
// inherits a transitive dependency from it. A new entry is a deliberate choice.
func TestArrowRule(t *testing.T) {
	allow := map[string]bool{`"github.com/dhannyell/fixed"`: true}
	entries, err := os.ReadDir(".")
	if err != nil {
		t.Fatal(err)
	}
	for _, e := range entries {
		name := e.Name()
		if !strings.HasSuffix(name, ".go") || strings.HasSuffix(name, "_test.go") {
			continue
		}
		file, err := parser.ParseFile(token.NewFileSet(), name, nil, parser.ImportsOnly)
		if err != nil {
			t.Fatal(err)
		}
		for _, imp := range file.Imports {
			if !allow[imp.Path.Value] {
				t.Errorf("%s imports %s: outside the allowlist", name, imp.Path.Value)
			}
		}
	}
}
