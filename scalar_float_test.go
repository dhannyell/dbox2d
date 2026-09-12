//go:build !dbox2d_fixed

package dbox2d

import (
	"math"
	"testing"
)

func testQ(v float64) Q { return Q{float32(v)} }

func testQEqual(got, want Q) bool {
	return math.Float32bits(got.v) == math.Float32bits(want.v)
}

// TestQArithmetic ensures scalar operations round exactly as float32 does.
func TestQArithmetic(t *testing.T) {
	type pair struct {
		name string
		a, b float64
	}
	pairs := []pair{
		{name: "negative", a: -1000.25, b: 0.00125},
		{name: "small", a: -0.001, b: 0.0009999},
		{name: "mixed", a: -0.5, b: 1.5},
		{name: "large", a: 999.75, b: -1000.25},
		{name: "zero", a: 0, b: 0.001},
	}

	for _, tc := range pairs {
		a, b := testQ(tc.a), testQ(tc.b)
		t.Run(tc.name+"/Add", func(t *testing.T) {
			want := testQ(float64(a.v) + float64(b.v))
			if got := a.Add(b); !testQEqual(got, want) {
				t.Fatalf("Add() = %v, want %v", got, want)
			}
		})
		t.Run(tc.name+"/Sub", func(t *testing.T) {
			want := testQ(float64(a.v) - float64(b.v))
			if got := a.Sub(b); !testQEqual(got, want) {
				t.Fatalf("Sub() = %v, want %v", got, want)
			}
		})
		t.Run(tc.name+"/Mul", func(t *testing.T) {
			want := testQ(float64(a.v) * float64(b.v))
			if got := a.Mul(b); !testQEqual(got, want) {
				t.Fatalf("Mul() = %v, want %v", got, want)
			}
		})
		if b.v != 0 {
			t.Run(tc.name+"/Div", func(t *testing.T) {
				want := testQ(float64(a.v) / float64(b.v))
				if got := a.Div(b); !testQEqual(got, want) {
					t.Fatalf("Div() = %v, want %v", got, want)
				}
			})
		}
		if a.v >= 0 {
			t.Run(tc.name+"/Sqrt", func(t *testing.T) {
				want := testQ(math.Sqrt(float64(a.v)))
				if got := a.Sqrt(); !testQEqual(got, want) {
					t.Fatalf("Sqrt() = %v, want %v", got, want)
				}
			})
		}
		t.Run(tc.name+"/Neg", func(t *testing.T) {
			want := testQ(-float64(a.v))
			if got := a.Neg(); !testQEqual(got, want) {
				t.Fatalf("Neg() = %v, want %v", got, want)
			}
		})
		t.Run(tc.name+"/Abs", func(t *testing.T) {
			want := math.Float32frombits(math.Float32bits(a.v) &^ (1 << 31))
			if got := a.Abs(); got.v != want {
				t.Fatalf("Abs() = %v, want %v", got, want)
			}
		})
		t.Run(tc.name+"/Min", func(t *testing.T) {
			want := b
			if a.v < b.v {
				want = a
			}
			if got := a.Min(b); !testQEqual(got, want) {
				t.Fatalf("Min() = %v, want %v", got, want)
			}
		})
		t.Run(tc.name+"/Max", func(t *testing.T) {
			want := b
			if a.v > b.v {
				want = a
			}
			if got := a.Max(b); !testQEqual(got, want) {
				t.Fatalf("Max() = %v, want %v", got, want)
			}
		})
	}

	clamps := []struct {
		name string
		q    Q
		lo   Q
		hi   Q
		want Q
	}{
		{name: "below", q: testQ(-2), lo: testQ(-1), hi: testQ(1), want: testQ(-1)},
		{name: "inside", q: testQ(0.25), lo: testQ(-1), hi: testQ(1), want: testQ(0.25)},
		{name: "above", q: testQ(2), lo: testQ(-1), hi: testQ(1), want: testQ(1)},
	}
	for _, tc := range clamps {
		t.Run("Clamp/"+tc.name, func(t *testing.T) {
			if got := tc.q.Clamp(tc.lo, tc.hi); !testQEqual(got, tc.want) {
				t.Fatalf("Clamp() = %v, want %v", got, tc.want)
			}
		})
	}
}

// TestQComparisons ensures ordered comparisons and three-way comparison agree.
func TestQComparisons(t *testing.T) {
	tests := []struct {
		name     string
		a, b     Q
		cmp      int
		eq, less bool
		greater  bool
	}{
		{name: "less", a: testQ(-1), b: testQ(0), cmp: -1, less: true},
		{name: "equal", a: testQ(1), b: testQ(1), eq: true},
		{name: "greater", a: testQ(2), b: testQ(1), cmp: 1, greater: true},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			if got := tc.a.Cmp(tc.b); got != tc.cmp {
				t.Errorf("Cmp() = %d, want %d", got, tc.cmp)
			}
			if got := tc.a.Eq(tc.b); got != tc.eq {
				t.Errorf("Eq() = %v, want %v", got, tc.eq)
			}
			if got := tc.a.Less(tc.b); got != tc.less {
				t.Errorf("Less() = %v, want %v", got, tc.less)
			}
			if got := tc.a.Greater(tc.b); got != tc.greater {
				t.Errorf("Greater() = %v, want %v", got, tc.greater)
			}
		})
	}
}

// TestQRoundAndInt ensures ties round away from zero and casts truncate.
func TestQRoundAndInt(t *testing.T) {
	tests := []struct {
		value float64
		round float64
		whole int
	}{
		{value: 0.5, round: 1, whole: 0},
		{value: -0.5, round: -1, whole: 0},
		{value: 1.5, round: 2, whole: 1},
		{value: -1.5, round: -2, whole: -1},
		{value: 2.5, round: 3, whole: 2},
		{value: -2.5, round: -3, whole: -2},
		{value: 1.7, round: 2, whole: 1},
		{value: -1.7, round: -2, whole: -1},
	}
	for _, tc := range tests {
		q := testQ(tc.value)
		if got := q.Round(); got.v != float32(tc.round) {
			t.Errorf("Round(%v) = %v, want %v", tc.value, got, tc.round)
		}
		if got := q.Int(); got != tc.whole {
			t.Errorf("Int(%v) = %d, want %d", tc.value, got, tc.whole)
		}
	}
}

// TestMakeRot ensures the deterministic trigonometric approximation stays accurate and normalized.
func TestMakeRot(t *testing.T) {
	for i := range 65 {
		turns := -2 + 4*float64(i)/64
		got := MakeRot(testQ(turns))
		wantSin := math.Sin(2 * math.Pi * turns)
		wantCos := math.Cos(2 * math.Pi * turns)
		if math.Abs(float64(got.Sin.v)-wantSin) > 2e-3 {
			t.Errorf("MakeRot(%v).Sin = %v, want %v", turns, got.Sin, wantSin)
		}
		if math.Abs(float64(got.Cos.v)-wantCos) > 2e-3 {
			t.Errorf("MakeRot(%v).Cos = %v, want %v", turns, got.Cos, wantCos)
		}
		norm := float64(got.Sin.v)*float64(got.Sin.v) + float64(got.Cos.v)*float64(got.Cos.v)
		if math.Abs(norm-1) > 1e-5 {
			t.Errorf("MakeRot(%v) norm = %v, want 1", turns, norm)
		}
	}
}

// TestAtan2Turns ensures the deterministic atan2 approximation has the right angle and quadrants.
func TestAtan2Turns(t *testing.T) {
	for i := range 64 {
		angle := 2 * math.Pi * float64(i) / 64
		y, x := math.Sin(angle), math.Cos(angle)
		got := atan2Turns(testQ(y), testQ(x))
		want := math.Atan2(y, x) / (2 * math.Pi)
		if math.Abs(float64(got.v)-want) > 1e-4 {
			t.Errorf("atan2Turns(%v, %v) = %v, want %v", y, x, got, want)
		}
	}

	for _, tc := range []struct {
		y, x float64
	}{
		{y: 1, x: 2},
		{y: 2, x: -3},
		{y: -1, x: -2},
		{y: -4, x: 1},
		{y: 2, x: 0},
		{y: 0, x: -2},
	} {
		got := atan2Turns(testQ(tc.y), testQ(tc.x))
		want := math.Atan2(tc.y, tc.x) / (2 * math.Pi)
		if math.Abs(float64(got.v)-want) > 1e-4 {
			t.Errorf("atan2Turns(%v, %v) = %v, want %v", tc.y, tc.x, got, want)
		}
	}
	if got := atan2Turns(QZero(), QZero()); !testQEqual(got, QZero()) {
		t.Fatalf("atan2Turns(0, 0) = %v, want 0", got)
	}
}

// TestVec2Normalize ensures zero stays zero and a regular vector becomes unit length.
func TestVec2Normalize(t *testing.T) {
	if got := (Vec2{}).Normalize(); got != (Vec2{}) {
		t.Fatalf("zero Normalize() = %#v, want zero", got)
	}
	got := (Vec2{X: testQ(3), Y: testQ(4)}).Normalize()
	if length := got.Len(); math.Abs(float64(length.v)-1) > 1e-6 {
		t.Fatalf("normalized length = %v, want 1", length)
	}
}

// TestIsValidQ ensures non-finite float32 values are rejected while finite extremes remain valid.
func TestIsValidQ(t *testing.T) {
	for _, tc := range []struct {
		name string
		q    Q
		want bool
	}{
		{name: "nan", q: Q{math.Float32frombits(0x7fc00000)}},
		{name: "positive infinity", q: Q{math.Float32frombits(0x7f800000)}},
		{name: "negative infinity", q: Q{math.Float32frombits(0xff800000)}},
		{name: "max", q: QMaxValue(), want: true},
		{name: "min", q: QMinValue(), want: true},
		{name: "zero", q: QZero(), want: true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			if got := IsValidQ(tc.q); got != tc.want {
				t.Errorf("IsValidQ(%v) = %v, want %v", tc.q, got, tc.want)
			}
		})
	}
}

// TestQParsingAndPresentation ensures decimal parsing, epsilon bits, and presentation conversion are stable.
func TestQParsingAndPresentation(t *testing.T) {
	if got := QMustParse("0.25").String(); got != "0.25" {
		t.Errorf("QMustParse(0.25).String() = %q, want %q", got, "0.25")
	}
	epsilon := QMustParse("1.1920929e-7")
	if got := math.Float32bits(epsilon.v); got != 0x34000000 {
		t.Errorf("epsilon bits = %#x, want %#x", got, uint32(0x34000000))
	}
	if got := QToFloat64(QFromFloat64(0.375)); got != 0.375 {
		t.Errorf("presentation round trip = %v, want 0.375", got)
	}
}

// TestUnwindAngleMatchesRemainder pins the inline cases of unwindAngle to
// the float64 remainder they replace, across the edges and a random sweep.
func TestUnwindAngleMatchesRemainder(t *testing.T) {
	want := func(v float32) float32 {
		return float32(math.Remainder(float64(v), float64(floatTau.v)))
	}
	edges := []float32{0, floatPi.v, -floatPi.v, floatTau.v, -floatTau.v, floatThreePi, -floatThreePi,
		math.Nextafter32(floatPi.v, 10), math.Nextafter32(floatPi.v, 0),
		math.Nextafter32(floatThreePi, 10), math.Nextafter32(floatThreePi, 0),
		math.Nextafter32(-floatPi.v, -10), math.Nextafter32(-floatPi.v, 0),
		float32(math.Inf(1)), float32(math.Inf(-1)), float32(math.NaN()), 1e6, -1e6, 1e-30}
	for _, v := range edges {
		got := unwindAngle(Q{v}).v
		if math.Float32bits(got) != math.Float32bits(want(v)) && !(got != got && want(v) != want(v)) {
			t.Errorf("unwindAngle(%v) = %v, want %v", v, got, want(v))
		}
	}
	seed := uint32(12345)
	for range 2_000_000 {
		seed = seed*1664525 + 1013904223
		v := (float32(seed>>8)/float32(1<<24) - 0.5) * 40
		got := unwindAngle(Q{v}).v
		if math.Float32bits(got) != math.Float32bits(want(v)) {
			t.Fatalf("unwindAngle(%v) = %v, want %v", v, got, want(v))
		}
	}
}
