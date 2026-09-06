package dbox2d

import "unsafe"

// sliceBytes returns the storage reserved by a slice, capacity times
// element size. It corresponds to the PREFIX##Array_ByteCount macro in
// src/array.h.
func sliceBytes[T any](s []T) int {
	var zero T
	return cap(s) * int(unsafe.Sizeof(zero))
}

// removeSwap removes s[index] by moving the last element into its place. It
// returns the shrunk slice and the old index of the moved element, or
// nullIndex when nothing moved. The caller must repair the stored index of
// the moved element. It corresponds to b2Array_RemoveSwap in src/array.h.
func removeSwap[T any](s []T, index int) ([]T, int) {
	last := len(s) - 1
	movedIndex := nullIndex
	if index != last {
		s[index] = s[last]
		movedIndex = last
	}

	// The vacated slot drops its references, so the garbage collector can
	// reclaim what they point to.
	var zero T
	s[last] = zero
	return s[:last], movedIndex
}
