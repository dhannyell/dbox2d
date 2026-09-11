//go:build dbox2d_simd && !dbox2d_fixed && goexperiment.simd && go1.27 && amd64

package dbox2d

import (
	"simd/archsimd"
	"unsafe"
)

var identityBodyRowW = [wideWidth]laneScalar{0, 0, 0, 0, 0, 0, 0, 1}

// Whole-row SIMD loads and stores require bodyState to occupy exactly one vector.
var _ [32]struct{} = [unsafe.Sizeof(bodyState{})]struct{}{}

// bodyRowW is the 32-byte state of one body as a vector; a null index yields the identity row.
func bodyRowW(states []bodyState, idx int, identity archsimd.Float32x8) archsimd.Float32x8 {
	if idx == nullIndex {
		return identity
	}
	return archsimd.LoadFloat32x8Array((*[wideWidth]laneScalar)(unsafe.Pointer(&states[idx])))
}

// storeBodyRowW writes one row back; null lanes have no body.
func storeBodyRowW(states []bodyState, idx int, row archsimd.Float32x8) {
	if idx != nullIndex {
		row.StoreArray((*[wideWidth]laneScalar)(unsafe.Pointer(&states[idx])))
	}
}

// gatherBodyW loads one constraint's bodies; null lanes are identities.
// The transpose is written out twice: a helper with vector arrays copies through the stack.
func gatherBodyW(states []bodyState, indices *[wideWidth]int, tauW laneW, b *bodyStateW) {
	identity := archsimd.LoadFloat32x8Array(&identityBodyRowW)
	r0 := bodyRowW(states, indices[0], identity)
	r1 := bodyRowW(states, indices[1], identity)
	r2 := bodyRowW(states, indices[2], identity)
	r3 := bodyRowW(states, indices[3], identity)
	r4 := bodyRowW(states, indices[4], identity)
	r5 := bodyRowW(states, indices[5], identity)
	r6 := bodyRowW(states, indices[6], identity)
	r7 := bodyRowW(states, indices[7], identity)

	t0 := r0.ConcatPermuteScalarsGrouped(0, 1, 4, 5, r1)
	t1 := r0.ConcatPermuteScalarsGrouped(2, 3, 6, 7, r1)
	t2 := r2.ConcatPermuteScalarsGrouped(0, 1, 4, 5, r3)
	t3 := r2.ConcatPermuteScalarsGrouped(2, 3, 6, 7, r3)
	t4 := r4.ConcatPermuteScalarsGrouped(0, 1, 4, 5, r5)
	t5 := r4.ConcatPermuteScalarsGrouped(2, 3, 6, 7, r5)
	t6 := r6.ConcatPermuteScalarsGrouped(0, 1, 4, 5, r7)
	t7 := r6.ConcatPermuteScalarsGrouped(2, 3, 6, 7, r7)

	c0 := t0.ConcatPermuteScalarsGrouped(0, 2, 4, 6, t2)
	c1 := t0.ConcatPermuteScalarsGrouped(1, 3, 5, 7, t2)
	c2 := t1.ConcatPermuteScalarsGrouped(0, 2, 4, 6, t3)
	c3 := t1.ConcatPermuteScalarsGrouped(1, 3, 5, 7, t3)
	d0 := t4.ConcatPermuteScalarsGrouped(0, 2, 4, 6, t6)
	d1 := t4.ConcatPermuteScalarsGrouped(1, 3, 5, 7, t6)
	d2 := t5.ConcatPermuteScalarsGrouped(0, 2, 4, 6, t7)
	d3 := t5.ConcatPermuteScalarsGrouped(1, 3, 5, 7, t7)

	b.v.x = laneW{v: c0.ConcatPermute128Scalars(0, 2, d0)}
	b.v.y = laneW{v: c1.ConcatPermute128Scalars(0, 2, d1)}
	b.w = laneW{v: c2.ConcatPermute128Scalars(0, 2, d2)}.Mul(tauW)
	b.flags = laneW{v: c3.ConcatPermute128Scalars(0, 2, d3)}
	b.dp.x = laneW{v: c0.ConcatPermute128Scalars(1, 3, d0)}
	b.dp.y = laneW{v: c1.ConcatPermute128Scalars(1, 3, d1)}
	b.dq.s = laneW{v: c2.ConcatPermute128Scalars(1, 3, d2)}
	b.dq.c = laneW{v: c3.ConcatPermute128Scalars(1, 3, d3)}
}

// scatterBodyW writes real-lane velocities back with angular velocity in turns per second.
func scatterBodyW(states []bodyState, indices *[wideWidth]int, tauW laneW, b *bodyStateW) {
	r0 := b.v.x.v
	r1 := b.v.y.v
	r2 := b.w.toLane().Div(tauW).v
	r3 := b.flags.v
	r4 := b.dp.x.v
	r5 := b.dp.y.v
	r6 := b.dq.s.v
	r7 := b.dq.c.v

	t0 := r0.ConcatPermuteScalarsGrouped(0, 1, 4, 5, r1)
	t1 := r0.ConcatPermuteScalarsGrouped(2, 3, 6, 7, r1)
	t2 := r2.ConcatPermuteScalarsGrouped(0, 1, 4, 5, r3)
	t3 := r2.ConcatPermuteScalarsGrouped(2, 3, 6, 7, r3)
	t4 := r4.ConcatPermuteScalarsGrouped(0, 1, 4, 5, r5)
	t5 := r4.ConcatPermuteScalarsGrouped(2, 3, 6, 7, r5)
	t6 := r6.ConcatPermuteScalarsGrouped(0, 1, 4, 5, r7)
	t7 := r6.ConcatPermuteScalarsGrouped(2, 3, 6, 7, r7)

	c0 := t0.ConcatPermuteScalarsGrouped(0, 2, 4, 6, t2)
	c1 := t0.ConcatPermuteScalarsGrouped(1, 3, 5, 7, t2)
	c2 := t1.ConcatPermuteScalarsGrouped(0, 2, 4, 6, t3)
	c3 := t1.ConcatPermuteScalarsGrouped(1, 3, 5, 7, t3)
	d0 := t4.ConcatPermuteScalarsGrouped(0, 2, 4, 6, t6)
	d1 := t4.ConcatPermuteScalarsGrouped(1, 3, 5, 7, t6)
	d2 := t5.ConcatPermuteScalarsGrouped(0, 2, 4, 6, t7)
	d3 := t5.ConcatPermuteScalarsGrouped(1, 3, 5, 7, t7)

	storeBodyRowW(states, indices[0], c0.ConcatPermute128Scalars(0, 2, d0))
	storeBodyRowW(states, indices[1], c1.ConcatPermute128Scalars(0, 2, d1))
	storeBodyRowW(states, indices[2], c2.ConcatPermute128Scalars(0, 2, d2))
	storeBodyRowW(states, indices[3], c3.ConcatPermute128Scalars(0, 2, d3))
	storeBodyRowW(states, indices[4], c0.ConcatPermute128Scalars(1, 3, d0))
	storeBodyRowW(states, indices[5], c1.ConcatPermute128Scalars(1, 3, d1))
	storeBodyRowW(states, indices[6], c2.ConcatPermute128Scalars(1, 3, d2))
	storeBodyRowW(states, indices[7], c3.ConcatPermute128Scalars(1, 3, d3))
}
