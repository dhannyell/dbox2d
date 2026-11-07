//go:build dbox2d_double && dbox2d_fixed

package b2

// The large world mode and the fixed point mode exclude each other: Q32.32
// already spans +-2^31 m, so a double position adds nothing, and the double
// mode rounds floats outward, which a fixed grid has no use for.
func init() {
	dbox2d_double_and_dbox2d_fixed_are_mutually_exclusive()
}
