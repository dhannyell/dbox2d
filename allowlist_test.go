package dbox2d_test

import (
	"go/parser"
	"go/token"
	"os"
	"strings"
	"testing"
)

// TestArrowRule limits production imports to the standard library and to the
// approved modules. The solver depends on the fixed-point module and on
// nothing else, so a consumer never inherits a transitive dependency from it.
// A new entry is a deliberate choice.
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
			if isStdlib(imp.Path.Value) || allow[imp.Path.Value] {
				continue
			}
			t.Errorf("%s imports %s: outside the allowlist", name, imp.Path.Value)
		}
	}
}

// isStdlib reports whether a quoted import path belongs to the standard
// library. A module path carries a domain, so its first element has a dot.
func isStdlib(quoted string) bool {
	first, _, _ := strings.Cut(strings.Trim(quoted, `"`), "/")
	return !strings.Contains(first, ".")
}
