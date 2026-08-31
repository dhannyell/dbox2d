package dbox2d

// Version reports a three-part release number.
type Version struct {
	// Major counts significant changes.
	Major int

	// Minor counts incremental changes.
	Minor int

	// Revision counts bug fixes.
	Revision int
}

// ReferenceVersion returns the Box2D release that this package ports. The
// module has its own version; this one identifies the reference source.
func ReferenceVersion() Version {
	return Version{Major: 3, Minor: 1, Revision: 1}
}

// nullIndex marks an absent reference in the storage graph.
// upstream B2_NULL_INDEX
const nullIndex = -1

// maxWorlds bounds the world registry. upstream B2_MAX_WORLDS
const maxWorlds = 128

// secretCookie marks a definition that a Default function initialized.
// upstream B2_SECRET_COOKIE
const secretCookie = 1152023

// checkDef panics on a definition that skipped its Default function. It
// corresponds to B2_CHECK_DEF in src/core.h.
func checkDef(internalValue int) {
	if internalValue != secretCookie {
		panic("dbox2d: initialize a definition with its Default function")
	}
}
