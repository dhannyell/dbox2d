//go:build dbox2d_wide && !dbox2d_float

package dbox2d

// Wide lanes are defined only for the float scalar mode.
var _ = wideRequiresFloatMode
