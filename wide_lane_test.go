//go:build dbox2d_simd && dbox2d_float

package dbox2d

import (
	"math"
	"math/rand"
	"testing"
)

const (
	// wideRandomVectors fixes the random coverage for reproducible lane checks.
	wideRandomVectors = 10000
)

var (
	// wideTinyValues covers subnormal and smallest-normal float32 values.
	wideTinyValues = [...]float32{
		math.Float32frombits(0x00000001),
		math.Float32frombits(0x007fffff),
		math.Float32frombits(0x00800000),
		math.Float32frombits(0x80000001),
		math.Float32frombits(0x807fffff),
		math.Float32frombits(0x80800000),
	}
	// wideSpecialValues covers signed zeros, unit values, and infinities.
	wideSpecialValues = [...]float32{
		math.Float32frombits(0x00000000),
		math.Float32frombits(0x80000000),
		1,
		-1,
		float32(math.Inf(1)),
		float32(math.Inf(-1)),
	}
)

// wideRandomValue draws finite, subnormal, zero, unit, and infinite values.
func wideRandomValue(r *rand.Rand) float32 {
	switch r.Intn(4) {
	case 0:
		return float32(2*r.Float64() - 1)
	case 1:
		value := float32(float64(math.MaxFloat32) * (0.5 + 0.5*r.Float64()))
		if r.Intn(2) == 0 {
			return value
		}
		return -value
	case 2:
		return wideTinyValues[r.Intn(len(wideTinyValues))]
	default:
		return wideSpecialValues[r.Intn(len(wideSpecialValues))]
	}
}

// wideRandomArray builds one reproducible random lane-width input.
func wideRandomArray(r *rand.Rand) (out [wideWidth]float32) {
	for i := range out {
		out[i] = wideRandomValue(r)
	}
	return out
}

// wideRandomOperands builds the arrays used by the arithmetic checks.
func wideRandomOperands(r *rand.Rand) (a, b, c [wideWidth]float32) {
	return wideRandomArray(r), wideRandomArray(r), wideRandomArray(r)
}

// wideArithmeticInputsValid excludes only undefined NaN-producing operations.
func wideArithmeticInputsValid(a, b, c [wideWidth]float32) bool {
	for i := range a {
		var sum = a[i] + b[i]
		var difference = a[i] - b[i]
		product := float32(b[i] * c[i])
		productSum := a[i] + product
		productDifference := a[i] - product
		quotient := float32(a[i] / b[i])
		if math.IsNaN(float64(sum)) || math.IsNaN(float64(difference)) || math.IsNaN(float64(product)) || math.IsNaN(float64(productSum)) || math.IsNaN(float64(productDifference)) || math.IsNaN(float64(quotient)) {
			return false
		}
	}
	return true
}

// wideCheckBits reports the first lane whose result is not bit-for-bit equal.
func wideCheckBits(t *testing.T, name string, got, want [wideWidth]float32) {
	t.Helper()
	for i := range got {
		if math.Float32bits(got[i]) != math.Float32bits(want[i]) {
			t.Fatalf("%s lane %d: got %#08x want %#08x", name, i, math.Float32bits(got[i]), math.Float32bits(want[i]))
		}
	}
}

// TestBodyGatherScatterW checks every transposed field and preserves untouched state bits.
func TestBodyGatherScatterW(t *testing.T) {
	finiteBits := [...]uint32{
		0x00000001, 0x80000001, 0x00800000, 0x80800000,
		0x3f800001, 0xbf800001, 0x41234567, 0xc0abcdef,
		0x7f7fffff, 0xff7fffff, 0x3eaaaaab, 0xbe4ccccd,
		0x4b123456, 0xcb654321, 0x00012345, 0x80054321,
		0x3dcccccd, 0xbdcccccd, 0x40490fdb, 0xc0490fdb,
		0x4e6e6b28, 0xce6e6b28, 0x01020304, 0x81020304,
	}
	value := func(body, field int) Q {
		return Q{v: math.Float32frombits(finiteBits[(7*body+field)%len(finiteBits)])}
	}

	states := make([]bodyState, 16)
	for i := range states {
		states[i] = bodyState{
			linearVelocity:  Vec2{X: value(i, 0), Y: value(i, 1)},
			angularVelocity: value(i, 2),
			flags:           int32(i+1) * 0x01010101,
			deltaPosition:   Vec2{X: value(i, 3), Y: value(i, 4)},
			deltaRotation:   Rot{Cos: value(i, 5), Sin: value(i, 6)},
		}
	}
	before := append([]bodyState(nil), states...)
	indexPattern := [...]int{2, nullIndex, 11, 5, 0, 15, nullIndex, 9}
	var indices [wideWidth]int
	for i := range wideWidth {
		indices[i] = indexPattern[i]
	}

	tauW := laneSplat(tau)
	var body bodyStateW
	gatherBodyW(states, &indices, tauW, &body)

	var gotVX, gotVY, gotW, gotFlags, gotDPX, gotDPY, gotDQC, gotDQS [wideWidth]float32
	body.v.x.toLane().store(&gotVX)
	body.v.y.toLane().store(&gotVY)
	body.w.toLane().store(&gotW)
	body.flags.store(&gotFlags)
	body.dp.x.store(&gotDPX)
	body.dp.y.store(&gotDPY)
	body.dq.c.store(&gotDQC)
	body.dq.s.store(&gotDQS)

	var wantVX, wantVY, wantW, wantFlags, wantDPX, wantDPY, wantDQC, wantDQS [wideWidth]float32
	for i := range wideWidth {
		idx := indices[i]
		if idx == nullIndex {
			wantDQC[i] = 1
			continue
		}
		state := states[idx]
		wantVX[i] = state.linearVelocity.X.v
		wantVY[i] = state.linearVelocity.Y.v
		wantW[i] = state.angularVelocity.Mul(tau).v
		if widePath() == "avx2" {
			wantFlags[i] = math.Float32frombits(uint32(state.flags))
		}
		wantDPX[i] = state.deltaPosition.X.v
		wantDPY[i] = state.deltaPosition.Y.v
		wantDQC[i] = state.deltaRotation.Cos.v
		wantDQS[i] = state.deltaRotation.Sin.v
	}
	wideCheckBits(t, "body v.x", gotVX, wantVX)
	wideCheckBits(t, "body v.y", gotVY, wantVY)
	wideCheckBits(t, "body w", gotW, wantW)
	wideCheckBits(t, "body flags", gotFlags, wantFlags)
	wideCheckBits(t, "body dp.x", gotDPX, wantDPX)
	wideCheckBits(t, "body dp.y", gotDPY, wantDPY)
	wideCheckBits(t, "body dq.c", gotDQC, wantDQC)
	wideCheckBits(t, "body dq.s", gotDQS, wantDQS)

	var newVX, newVY, newW [wideWidth]float32
	for i := range wideWidth {
		newVX[i] = math.Float32frombits(finiteBits[(3*i+17)%len(finiteBits)])
		newVY[i] = math.Float32frombits(finiteBits[(5*i+9)%len(finiteBits)])
		newW[i] = math.Float32frombits(finiteBits[(7*i+4)%len(finiteBits)])
	}
	body.v.x = laneLoad(&newVX).toAcc()
	body.v.y = laneLoad(&newVY).toAcc()
	body.w = laneLoad(&newW).toAcc()
	scatterBodyW(states, &indices, tauW, &body)

	referenced := make([]bool, len(states))
	for i := range wideWidth {
		idx := indices[i]
		if idx == nullIndex {
			continue
		}
		referenced[idx] = true
		if math.Float32bits(states[idx].linearVelocity.X.v) != math.Float32bits(newVX[i]) ||
			math.Float32bits(states[idx].linearVelocity.Y.v) != math.Float32bits(newVY[i]) ||
			math.Float32bits(states[idx].angularVelocity.v) != math.Float32bits(Q{v: newW[i]}.Div(tau).v) {
			t.Fatalf("body %d velocity bits changed incorrectly", idx)
		}
		if !sameBodyNonVelocityBits(states[idx], before[idx]) {
			t.Fatalf("body %d non-velocity bits changed", idx)
		}
	}
	for i := range states {
		if !referenced[i] && !sameBodyStateBits(states[i], before[i]) {
			t.Fatalf("unreferenced body %d changed", i)
		}
	}
}

func sameBodyNonVelocityBits(a, b bodyState) bool {
	return a.flags == b.flags &&
		math.Float32bits(a.deltaPosition.X.v) == math.Float32bits(b.deltaPosition.X.v) &&
		math.Float32bits(a.deltaPosition.Y.v) == math.Float32bits(b.deltaPosition.Y.v) &&
		math.Float32bits(a.deltaRotation.Cos.v) == math.Float32bits(b.deltaRotation.Cos.v) &&
		math.Float32bits(a.deltaRotation.Sin.v) == math.Float32bits(b.deltaRotation.Sin.v)
}

func sameBodyStateBits(a, b bodyState) bool {
	return math.Float32bits(a.linearVelocity.X.v) == math.Float32bits(b.linearVelocity.X.v) &&
		math.Float32bits(a.linearVelocity.Y.v) == math.Float32bits(b.linearVelocity.Y.v) &&
		math.Float32bits(a.angularVelocity.v) == math.Float32bits(b.angularVelocity.v) &&
		sameBodyNonVelocityBits(a, b)
}

// wideCheckMaskLanes checks mask bits through the public lane blend contract.
func wideCheckMaskLanes(t *testing.T, name string, got maskW, want [wideWidth]bool) {
	t.Helper()
	var actual [wideWidth]float32
	laneBlend(got, laneSplat(QOne()), laneZero()).store(&actual)
	for i := range actual {
		wantValue := float32(0)
		if want[i] {
			wantValue = 1
		}
		if math.Float32bits(actual[i]) != math.Float32bits(wantValue) {
			t.Fatalf("%s lane %d: got %t want %t", name, i, actual[i] != 0, want[i])
		}
	}
}

// TestWidePath records the selected path and checks its width relation.
func TestWidePath(t *testing.T) {
	t.Logf("path: %s width %d", widePath(), wideWidth)
	if wideWidth != 1<<wideShift {
		t.Fatalf("width %d does not match shift %d", wideWidth, wideShift)
	}
	if widePath() == "generic" && !wideAvailable() {
		t.Fatal("generic path must be available")
	}
}

// TestLaneArithmeticRoundsTwice keeps every lane operation scalar-equivalent.
func TestLaneArithmeticRoundsTwice(t *testing.T) {
	r := rand.New(rand.NewSource(1))
	for vector := 0; vector < wideRandomVectors; vector++ {
		a, b, c := wideRandomOperands(r)
		if !wideArithmeticInputsValid(a, b, c) {
			vector--
			continue
		}

		var gotAdd, gotSub, gotMul, gotDiv, gotMulAdd, gotMulSub [wideWidth]float32
		var gotMin, gotMax [wideWidth]float32
		laneLoad(&a).Add(laneLoad(&b)).store(&gotAdd)
		laneLoad(&a).Sub(laneLoad(&b)).store(&gotSub)
		laneLoad(&a).Mul(laneLoad(&b)).store(&gotMul)
		laneLoad(&a).Div(laneLoad(&b)).store(&gotDiv)
		laneLoad(&a).MulAdd(laneLoad(&b), laneLoad(&c)).store(&gotMulAdd)
		laneLoad(&a).MulSub(laneLoad(&b), laneLoad(&c)).store(&gotMulSub)
		laneLoad(&a).Min(laneLoad(&b)).store(&gotMin)
		laneLoad(&a).Max(laneLoad(&b)).store(&gotMax)

		var wantAdd, wantSub, wantMul, wantDiv, wantMulAdd, wantMulSub [wideWidth]float32
		var wantMin, wantMax [wideWidth]float32
		for i := range a {
			wantAdd[i] = a[i] + b[i]
			wantSub[i] = a[i] - b[i]
			wantMul[i] = a[i] * b[i]
			wantDiv[i] = float32(a[i] / b[i])
			product := float32(b[i] * c[i])
			wantMulAdd[i] = a[i] + product
			wantMulSub[i] = a[i] - product
			if a[i] < b[i] {
				wantMin[i] = a[i]
			} else {
				wantMin[i] = b[i]
			}
			if a[i] > b[i] {
				wantMax[i] = a[i]
			} else {
				wantMax[i] = b[i]
			}
		}

		wideCheckBits(t, "add", gotAdd, wantAdd)
		wideCheckBits(t, "sub", gotSub, wantSub)
		wideCheckBits(t, "mul", gotMul, wantMul)
		wideCheckBits(t, "div", gotDiv, wantDiv)
		wideCheckBits(t, "mul-add", gotMulAdd, wantMulAdd)
		wideCheckBits(t, "mul-sub", gotMulSub, wantMulSub)
		wideCheckBits(t, "min", gotMin, wantMin)
		wideCheckBits(t, "max", gotMax, wantMax)

		var gotIdentity [wideWidth]float32
		laneLoad(&a).toAcc().toLane().store(&gotIdentity)
		wideCheckBits(t, "lane identity", gotIdentity, a)
	}
}

// TestLaneComparisonsAndBlend keeps masks and selection lane-aligned.
func TestLaneComparisonsAndBlend(t *testing.T) {
	r := rand.New(rand.NewSource(1))
	for vector := 0; vector < wideRandomVectors; vector++ {
		a := wideRandomArray(r)
		b := wideRandomArray(r)
		greater := laneLoad(&a).Greater(laneLoad(&b))
		equal := laneLoad(&a).Equals(laneLoad(&b))
		combined := greater.Or(equal)
		wantGreater := [wideWidth]bool{}
		wantEqual := [wideWidth]bool{}
		wantCombined := [wideWidth]bool{}
		for i := range a {
			wantGreater[i] = a[i] > b[i]
			wantEqual[i] = a[i] == b[i]
			wantCombined[i] = wantGreater[i] || wantEqual[i]
		}
		wideCheckMaskLanes(t, "greater", greater, wantGreater)
		wideCheckMaskLanes(t, "equal", equal, wantEqual)
		wideCheckMaskLanes(t, "or", combined, wantCombined)
		if greater.AllZero() != !containsTrue(wantGreater) {
			t.Fatalf("greater AllZero mismatch")
		}
		if equal.AllZero() != !containsTrue(wantEqual) {
			t.Fatalf("equal AllZero mismatch")
		}
		if combined.AllZero() != !containsTrue(wantCombined) {
			t.Fatalf("or AllZero mismatch")
		}

		var gotBlend [wideWidth]float32
		laneBlend(combined, laneLoad(&a), laneLoad(&b)).store(&gotBlend)
		var wantBlend [wideWidth]float32
		for i := range wantBlend {
			if wantCombined[i] {
				wantBlend[i] = a[i]
			} else {
				wantBlend[i] = b[i]
			}
		}
		wideCheckBits(t, "blend", gotBlend, wantBlend)
	}
}

// containsTrue reports whether a boolean array has a set lane.
func containsTrue(values [wideWidth]bool) bool {
	for _, value := range values {
		if value {
			return true
		}
	}
	return false
}

// TestLaneLoadStorePreservesBits protects the lane boundary from conversions.
func TestLaneLoadStorePreservesBits(t *testing.T) {
	r := rand.New(rand.NewSource(1))
	for vector := 0; vector < wideRandomVectors; vector++ {
		want := wideRandomArray(r)
		var got [wideWidth]float32
		laneLoad(&want).store(&got)
		wideCheckBits(t, "load-store", got, want)
	}
}

// TestLaneSplatAndZeroPreserveBits protects scalar broadcast and zero signs.
func TestLaneSplatAndZeroPreserveBits(t *testing.T) {
	r := rand.New(rand.NewSource(1))
	for vector := 0; vector < wideRandomVectors; vector++ {
		value := wideRandomValue(r)
		var gotSplat [wideWidth]float32
		laneSplat(Q{v: value}).store(&gotSplat)
		var wantSplat [wideWidth]float32
		for i := range wantSplat {
			wantSplat[i] = value
		}
		wideCheckBits(t, "splat", gotSplat, wantSplat)

		var gotZero [wideWidth]float32
		laneZero().store(&gotZero)
		var wantZero [wideWidth]float32
		wideCheckBits(t, "zero", gotZero, wantZero)
	}
}

// TestLaneMinMaxReturnSecondOnEqual protects signed-zero and tie semantics.
func TestLaneMinMaxReturnSecondOnEqual(t *testing.T) {
	var first, second [wideWidth]float32
	for i := range first {
		switch i % 4 {
		case 0:
			first[i] = math.Float32frombits(0x00000000)
			second[i] = math.Float32frombits(0x80000000)
		case 1:
			first[i] = math.Float32frombits(0x80000000)
			second[i] = math.Float32frombits(0x00000000)
		case 2:
			first[i], second[i] = 1, 1
		default:
			first[i] = math.Float32frombits(0x80000000)
			second[i] = math.Float32frombits(0x80000000)
		}
	}
	var gotMin, gotMax [wideWidth]float32
	laneLoad(&first).Min(laneLoad(&second)).store(&gotMin)
	laneLoad(&first).Max(laneLoad(&second)).store(&gotMax)
	wideCheckBits(t, "min equal", gotMin, second)
	wideCheckBits(t, "max equal", gotMax, second)
}
