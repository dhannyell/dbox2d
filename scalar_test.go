package b2

import "testing"

func TestShortConstructorsMatchTheLongOnes(t *testing.T) {
	if F(3) != QFromInt(3) || F(int32(-7)) != QFromInt(-7) || F(int64(9)) != QFromInt(9) {
		t.Fatal("F of an integer")
	}
	if F(0.35) != QFromFloat64(0.35) || F(float32(0.25)) != QFromFloat64(0.25) {
		t.Fatal("F of a float")
	}
	q := QFromRatio(1, 3)
	if F(q) != q {
		t.Fatal("F of a Q must pass through")
	}
	if V2(1, -2) != (Vec2{X: QFromInt(1), Y: QFromInt(-2)}) || V2(0.5, 0.25) != (Vec2{X: QHalf(), Y: QFromRatio(1, 4)}) {
		t.Fatal("V2")
	}
	if V2(q, q) != (Vec2{X: q, Y: q}) {
		t.Fatal("V2 of Q")
	}
	if Degrees(90) != QFromRatio(1, 4) || Degrees(-180) != QFromRatio(-1, 2) {
		t.Fatal("Degrees of an integer")
	}
	if Degrees(45.0) != QFromFloat64(45.0/360) || Degrees(QFromInt(360)) != QOne() {
		t.Fatal("Degrees of a float or a Q")
	}
}

func TestShortConstructorsDoNotAllocate(t *testing.T) {
	var sink Vec2
	allocs := testing.AllocsPerRun(1000, func() {
		sink = V2(1.5, -2)
		sink.X = F(sink.X).Add(Degrees(90))
	})
	if allocs != 0 {
		t.Fatalf("allocs = %v", allocs)
	}
	_ = sink
}
