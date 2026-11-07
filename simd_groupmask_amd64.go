//go:build dbox2d_simd && !dbox2d_fixed && goexperiment.simd && go1.27 && amd64

package b2

import "simd/archsimd"

// laneGroupMaskTable holds the lane mask of each set of active lane groups:
// bit 0 is lanes 0-3 and bit 1 is lanes 4-7.
var laneGroupMaskTable = [4][wideWidth]int32{
	{},
	{-1, -1, -1, -1, 0, 0, 0, 0},
	{0, 0, 0, 0, -1, -1, -1, -1},
	{-1, -1, -1, -1, -1, -1, -1, -1},
}

// laneGroupMask sets the lanes whose lane group holds a nonzero value of v,
// as laneGroupAny does for the scalar family, and reports whether any lane
// does. It stays in registers: the generic form stores the lanes and reloads
// a mask, which stalls on store forwarding in the hottest contact loop.
func laneGroupMask(v laneW) (maskW, bool) {
	bits := v.v.NotEqual(laneBroadcast(0)).ToBits()
	group := 0
	if bits&0x0f != 0 {
		group |= 1
	}
	if bits&0xf0 != 0 {
		group |= 2
	}
	return maskW{v: archsimd.LoadInt32x8Array(&laneGroupMaskTable[group]).ToMask()}, group != 0
}
