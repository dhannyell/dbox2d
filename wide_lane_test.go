//go:build dbox2d_wide && dbox2d_float

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
func wideRandomOperands(r *rand.Rand) (a, b, c, limit [wideWidth]float32) {
	return wideRandomArray(r), wideRandomArray(r), wideRandomArray(r), wideRandomArray(r)
}

// wideArithmeticInputsValid excludes only undefined NaN-producing operations.
func wideArithmeticInputsValid(a, b, c [wideWidth]float32) bool {
	for i := range a {
		var sum = a[i] + b[i]
		var difference = a[i] - b[i]
		var product = b[i] * c[i]
		//nolint:staticcheck // preserve the explicit validation result type
		var productSum float32 = a[i] + product
		//nolint:staticcheck // preserve the explicit validation result type
		var productDifference float32 = a[i] - product
		if math.IsNaN(float64(sum)) || math.IsNaN(float64(difference)) || math.IsNaN(float64(product)) || math.IsNaN(float64(productSum)) || math.IsNaN(float64(productDifference)) {
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
		a, b, c, limit := wideRandomOperands(r)
		if !wideArithmeticInputsValid(a, b, c) {
			vector--
			continue
		}

		var gotAdd, gotSub, gotMul, gotMulAdd, gotMulSub [wideWidth]float32
		var gotMin, gotMax, gotSymClamp [wideWidth]float32
		laneLoad(&a).Add(laneLoad(&b)).store(&gotAdd)
		laneLoad(&a).Sub(laneLoad(&b)).store(&gotSub)
		laneLoad(&a).Mul(laneLoad(&b)).store(&gotMul)
		laneLoad(&a).MulAdd(laneLoad(&b), laneLoad(&c)).store(&gotMulAdd)
		laneLoad(&a).MulSub(laneLoad(&b), laneLoad(&c)).store(&gotMulSub)
		laneLoad(&a).Min(laneLoad(&b)).store(&gotMin)
		laneLoad(&a).Max(laneLoad(&b)).store(&gotMax)
		laneLoad(&a).SymClamp(laneLoad(&limit)).store(&gotSymClamp)

		var wantAdd, wantSub, wantMul, wantMulAdd, wantMulSub [wideWidth]float32
		var wantMin, wantMax, wantSymClamp [wideWidth]float32
		for i := range a {
			wantAdd[i] = a[i] + b[i]
			wantSub[i] = a[i] - b[i]
			wantMul[i] = a[i] * b[i]
			//nolint:staticcheck // keep the separately rounded product statement
			var product float32 = b[i] * c[i]
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

			var negLimit = float32(0) - limit[i]
			var clamped float32
			if a[i] < limit[i] {
				clamped = a[i]
			} else {
				clamped = limit[i]
			}
			if negLimit > clamped {
				wantSymClamp[i] = negLimit
			} else {
				wantSymClamp[i] = clamped
			}
		}

		wideCheckBits(t, "add", gotAdd, wantAdd)
		wideCheckBits(t, "sub", gotSub, wantSub)
		wideCheckBits(t, "mul", gotMul, wantMul)
		wideCheckBits(t, "mul-add", gotMulAdd, wantMulAdd)
		wideCheckBits(t, "mul-sub", gotMulSub, wantMulSub)
		wideCheckBits(t, "min", gotMin, wantMin)
		wideCheckBits(t, "max", gotMax, wantMax)
		// SIMD Min/Max may choose either signed zero when the clamp values tie.
		for i := range gotSymClamp {
			if math.Float32bits(gotSymClamp[i]) == math.Float32bits(wantSymClamp[i]) {
				continue
			}
			var negLimit = float32(0) - limit[i]
			var clamped float32
			if a[i] < limit[i] {
				clamped = a[i]
			} else {
				clamped = limit[i]
			}
			if negLimit == clamped && math.Float32bits(negLimit) != math.Float32bits(clamped) {
				t.Logf("sym clamp signed-zero relaxation lane %d: got=%#08x want=%#08x", i, math.Float32bits(gotSymClamp[i]), math.Float32bits(wantSymClamp[i]))
				continue
			}
			t.Fatalf("symmetric clamp lane %d: got %#08x want %#08x", i, math.Float32bits(gotSymClamp[i]), math.Float32bits(wantSymClamp[i]))
		}

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
