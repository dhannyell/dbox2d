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
