package b2

import (
	"math"
	"os"
	"reflect"
	"regexp"
	"slices"
	"strconv"
	"strings"
	"testing"
)

func TestDefaultDebugDrawIsCallable(t *testing.T) {
	draw := DefaultDebugDraw()
	draw.DrawPolygonFcn(Transform{}, nil, ColorBlack)
	draw.DrawSolidPolygonFcn(Transform{}, nil, Q{}, ColorBlack)
	draw.DrawCircleFcn(Vec2{}, Q{}, ColorBlack)
	draw.DrawSolidCircleFcn(Transform{}, Vec2{}, Q{}, ColorBlack)
	draw.DrawSolidCapsuleFcn(Vec2{}, Vec2{}, Q{}, ColorBlack)
	draw.DrawLineFcn(Vec2{}, Vec2{}, ColorBlack)
	draw.DrawTransformFcn(Transform{})
	draw.DrawPointFcn(Vec2{}, Q{}, ColorBlack)
	draw.DrawStringFcn(Vec2{}, "", ColorBlack)
	draw.DrawBoundsFcn(AABB{}, ColorBlack)
}

// TestDefaultDebugDrawMatchesTheReference pins b2DefaultDebugDraw: open
// bounds, unit scales and shapes on.
func TestDefaultDebugDrawMatchesTheReference(t *testing.T) {
	draw := DefaultDebugDraw()
	maxValue := QMaxValue().Mul(QHalf())
	want := AABB{LowerBound: Vec2{X: maxValue.Neg(), Y: maxValue.Neg()}, UpperBound: Vec2{X: maxValue, Y: maxValue}}
	if draw.DrawingBounds != want {
		t.Errorf("DrawingBounds = %+v, want %+v", draw.DrawingBounds, want)
	}
	if !draw.ForceScale.Eq(QOne()) || !draw.JointScale.Eq(QOne()) {
		t.Errorf("ForceScale, JointScale = %v, %v, want 1, 1", draw.ForceScale, draw.JointScale)
	}
	if !draw.DrawShapes || draw.DrawJoints || draw.DrawContacts {
		t.Error("only DrawShapes should start on")
	}
}

func TestDefaultJointDefDrawScale(t *testing.T) {
	if got, want := DefaultRevoluteJointDef().Base.DrawScale.String(), "1"; got != want {
		t.Fatalf("DrawScale = %s, want %s", got, want)
	}
}

func TestDrawRevoluteJointLimitsUseTurns(t *testing.T) {
	var segments [][2]Vec2
	draw := DefaultDebugDraw()
	draw.DrawLineFcn = func(a, b Vec2, _ HexColor) { segments = append(segments, [2]Vec2{a, b}) }
	var base jointSim
	base.localFrameA, base.localFrameB = TransformIdentity(), TransformIdentity()
	*base.revolute() = revoluteJoint{
		enableLimit: true,
		lowerAngle:  angleFromTurns(QMustParse("-0.25")),
		upperAngle:  angleFromTurns(QMustParse("0.25")),
	}
	// The draw scale 4 makes the radius 0.25 * 4 = 1.
	drawRevoluteJoint(&draw, &base, TransformIdentity(), TransformIdentity(), QFromInt(4))
	if len(segments) < 4 {
		t.Fatalf("segments = %d, want limit segments", len(segments))
	}
	lowerWant := Vec2{Y: QOne().Neg()}
	if got := segments[2][1]; got != lowerWant {
		t.Fatalf("lower limit endpoint = %#v, want %#v", got, lowerWant)
	}
	upperWant := Vec2{Y: QOne()}
	if got := segments[3][1]; got != upperWant {
		t.Fatalf("upper limit endpoint = %#v, want %#v", got, upperWant)
	}
}

func TestDrawFlagsAreIndependent(t *testing.T) {
	w := createDrawScene(t)
	cases := []struct {
		name   string
		enable func(*DebugDraw)
		kinds  []string
	}{
		{"shapes", func(d *DebugDraw) { d.DrawShapes = true }, []string{"solidPolygon", "solidCircle", "capsule", "line", "point"}},
		{"joints", func(d *DebugDraw) { d.DrawJoints = true }, []string{"circle", "line", "point"}},
		{"bounds", func(d *DebugDraw) { d.DrawBounds = true }, []string{"bounds"}},
		{"mass", func(d *DebugDraw) { d.DrawMass = true }, []string{"line", "transform", "string"}},
		{"names", func(d *DebugDraw) { d.DrawBodyNames = true }, []string{"string"}},
		{"contacts", func(d *DebugDraw) { d.DrawContacts = true }, []string{"point"}},
		{"islands", func(d *DebugDraw) { d.DrawIslands = true }, []string{"bounds"}},
		{"joint extras alone", func(d *DebugDraw) { d.DrawJointExtras = true }, nil},
		{"graph colors alone", func(d *DebugDraw) { d.DrawGraphColors = true }, nil},
		{"anchor A alone", func(d *DebugDraw) { d.DrawAnchorA = true }, nil},
		{"chain normals alone", func(d *DebugDraw) { d.DrawChainNormals = true }, nil},
		{"normals alone", func(d *DebugDraw) { d.DrawContactNormals = true }, nil},
		{"forces alone", func(d *DebugDraw) { d.DrawContactForces = true }, nil},
		{"features alone", func(d *DebugDraw) { d.DrawContactFeatures = true }, nil},
		{"friction alone", func(d *DebugDraw) { d.DrawFrictionForces = true }, nil},
		{"chain normals", func(d *DebugDraw) { d.DrawShapes, d.DrawChainNormals = true, true }, []string{"solidPolygon", "solidCircle", "capsule", "line", "point"}},
		{"normals", func(d *DebugDraw) { d.DrawContacts, d.DrawContactNormals = true, true }, []string{"point", "line", "string"}},
		{"forces", func(d *DebugDraw) { d.DrawContacts, d.DrawContactForces = true, true }, []string{"point", "line", "string"}},
		{"features", func(d *DebugDraw) { d.DrawContacts, d.DrawContactFeatures = true, true }, []string{"point", "string"}},
		{"friction", func(d *DebugDraw) { d.DrawContacts, d.DrawFrictionForces = true, true }, []string{"point", "line", "string"}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			draw, calls := recordDraw()
			w.Draw(draw)
			if len(*calls) != 0 {
				t.Fatal("disabled flags emitted callbacks")
			}
			tc.enable(draw)
			w.Draw(draw)
			seen := map[string]bool{}
			for _, c := range *calls {
				if !slices.Contains(tc.kinds, c.kind) {
					t.Fatalf("unexpected callback: %+v", c)
				}
				seen[c.kind] = true
			}
			for _, kind := range tc.kinds {
				if !seen[kind] {
					t.Errorf("missing %s callback", kind)
				}
			}
		})
	}
}

type drawCall struct {
	kind   string
	color  HexColor
	values []float64
	text   string
}

// recordDraw retains every argument so the oracle detects geometry and
// order changes. It starts with every flag off, DrawShapes included.
func recordDraw() (*DebugDraw, *[]drawCall) {
	var calls []drawCall
	q := func(v Q) float64 { return QToFloat64(v) }
	vec := func(p Vec2) []float64 { return []float64{q(p.X), q(p.Y)} }
	xf := func(x Transform) []float64 { return []float64{q(x.P.X), q(x.P.Y), q(x.Q.Cos), q(x.Q.Sin)} }
	add := func(kind string, color HexColor, values []float64, text string) {
		calls = append(calls, drawCall{kind, color, values, text})
	}
	draw := DefaultDebugDraw()
	draw.DrawShapes = false
	draw.DrawPolygonFcn = func(x Transform, v []Vec2, c HexColor) {
		values := append(xf(x), float64(len(v)))
		for _, p := range v {
			values = append(values, vec(p)...)
		}
		add("polygon", c, values, "")
	}
	draw.DrawSolidPolygonFcn = func(x Transform, v []Vec2, r Q, c HexColor) {
		values := append(xf(x), q(r), float64(len(v)))
		for _, p := range v {
			values = append(values, vec(p)...)
		}
		add("solidPolygon", c, values, "")
	}
	draw.DrawCircleFcn = func(p Vec2, r Q, c HexColor) { add("circle", c, append(vec(p), q(r)), "") }
	draw.DrawSolidCircleFcn = func(x Transform, center Vec2, r Q, c HexColor) {
		add("solidCircle", c, append(append(xf(x), vec(center)...), q(r)), "")
	}
	draw.DrawSolidCapsuleFcn = func(a, b Vec2, r Q, c HexColor) { add("capsule", c, append(append(vec(a), vec(b)...), q(r)), "") }
	draw.DrawLineFcn = func(a, b Vec2, c HexColor) { add("line", c, append(vec(a), vec(b)...), "") }
	draw.DrawTransformFcn = func(x Transform) { add("transform", 0, xf(x), "") }
	draw.DrawPointFcn = func(p Vec2, size Q, c HexColor) { add("point", c, append(vec(p), q(size)), "") }
	draw.DrawStringFcn = func(p Vec2, s string, c HexColor) { add("string", c, vec(p), s) }
	draw.DrawBoundsFcn = func(b AABB, c HexColor) { add("bounds", c, append(vec(b.LowerBound), vec(b.UpperBound)...), "") }
	return &draw, &calls
}

// enableAllDrawFlags turns on the flags of the oracle's "all" pass; anchor
// A and the contact forces wait for the bounded pass.
func enableAllDrawFlags(draw *DebugDraw) {
	draw.DrawShapes, draw.DrawChainNormals, draw.DrawJoints, draw.DrawJointExtras = true, true, true, true
	draw.DrawBounds, draw.DrawMass, draw.DrawBodyNames = true, true, true
	draw.DrawContacts, draw.DrawGraphColors, draw.DrawContactNormals = true, true, true
	draw.DrawContactFeatures, draw.DrawFrictionForces, draw.DrawIslands = true, true, true
}

// createDrawScene spaces every dynamic body far apart so each one touches
// only the ground: a moved proxy then finds at most one new pair, which
// keeps the port's pair order and graph colors identical to the
// reference's. No gap equals the speculative distance (4*linearSlop).
func createDrawScene(t testing.TB) WorldId {
	t.Helper()
	w := createTestWorld(t)
	sd := DefaultShapeDef()
	bd := DefaultBodyDef()
	bd.Position.Y = QHalf().Neg()
	bd.Name = "ground"
	ground := CreateBody(w, &bd)
	floor := MakeBox(QFromInt(40), QHalf())
	CreatePolygonShape(ground, &sd, &floor)

	box := MakeSquare(QHalf())
	bd = DefaultBodyDef()
	bd.Type, bd.EnableSleep = DynamicBody, false
	bd.Position, bd.Name = Vec2{X: QFromInt(-32), Y: QFromInt(3)}, "box"
	bd.Rotation, bd.MotionLocks = MakeRot(QMustParse("0.1")), MotionLocks{AngularZ: true}
	CreatePolygonShape(CreateBody(w, &bd), &sd, &box)

	slideSd := DefaultShapeDef()
	// A lower friction than the default keeps the box sliding through every step.
	slideSd.Material.Friction = QMustParse("0.1")
	bd = DefaultBodyDef()
	bd.Type, bd.EnableSleep = DynamicBody, false
	bd.Position, bd.Name = Vec2{X: QFromInt(-24), Y: QHalf()}, "sliding"
	bd.LinearVelocity = Vec2{X: QFromInt(3)}
	CreatePolygonShape(CreateBody(w, &bd), &slideSd, &box)

	bd = DefaultBodyDef()
	bd.Type, bd.EnableSleep = DynamicBody, false
	bd.Position, bd.Name = Vec2{X: QFromInt(-16), Y: QFromInt(3)}, "circle"
	circle := Circle{Radius: QMustParse("0.25")}
	CreateCircleShape(CreateBody(w, &bd), &sd, &circle)

	bd = DefaultBodyDef()
	bd.Type, bd.EnableSleep = DynamicBody, false
	bd.Position, bd.Name = Vec2{X: QFromInt(-8), Y: QFromInt(3)}, "rounded"
	rounded := MakeRoundedBox(QMustParse("0.4"), QMustParse("0.4"), QMustParse("0.1"))
	CreatePolygonShape(CreateBody(w, &bd), &sd, &rounded)

	bd = DefaultBodyDef()
	bd.Type, bd.EnableSleep = DynamicBody, false
	bd.Position, bd.Name = Vec2{X: QZero(), Y: QFromInt(3)}, "hinge"
	hinge := CreateBody(w, &bd)
	CreatePolygonShape(hinge, &sd, &box)
	jd := DefaultRevoluteJointDef()
	jd.Base.BodyIdA, jd.Base.BodyIdB = ground, hinge
	jd.Base.LocalFrameA.P = Vec2{X: QZero(), Y: QMustParse("3.5")}
	jd.EnableLimit = true
	jd.LowerAngle, jd.UpperAngle = QMustParse("-0.25"), QMustParse("0.25")
	CreateRevoluteJoint(w, &jd)

	small := MakeSquare(QMustParse("0.3"))
	bd = DefaultBodyDef()
	bd.Type, bd.EnableSleep = DynamicBody, false
	bd.Position, bd.Name = Vec2{X: QFromInt(8), Y: QFromInt(3)}, "distance"
	distanceBody := CreateBody(w, &bd)
	CreatePolygonShape(distanceBody, &sd, &small)
	dd := DefaultDistanceJointDef()
	dd.Base.BodyIdA, dd.Base.BodyIdB = ground, distanceBody
	dd.Base.LocalFrameA.P = Vec2{X: QFromInt(8), Y: QMustParse("5.5")}
	dd.Length = QFromInt(2)
	dd.EnableLimit = true
	dd.MinLength, dd.MaxLength = QOne(), QFromInt(3)
	CreateDistanceJoint(w, &dd)

	bd = DefaultBodyDef()
	bd.Type, bd.EnableSleep = DynamicBody, false
	bd.Position, bd.Name = Vec2{X: QFromInt(16), Y: QFromInt(6)}, "prismatic"
	prismaticBody := CreateBody(w, &bd)
	CreatePolygonShape(prismaticBody, &sd, &box)
	pd := DefaultPrismaticJointDef()
	pd.Base.BodyIdA, pd.Base.BodyIdB = ground, prismaticBody
	pd.Base.LocalFrameA.P = Vec2{X: QFromInt(16), Y: QMustParse("3.5")}
	pd.Base.LocalFrameA.Q = MakeRotFromUnitVector(Vec2{Y: QOne()})
	pd.Base.LocalFrameB.Q = pd.Base.LocalFrameA.Q
	pd.EnableLimit = true
	pd.LowerTranslation, pd.UpperTranslation = QZero(), QFromInt(3)
	CreatePrismaticJoint(w, &pd)

	bd = DefaultBodyDef()
	bd.Type, bd.EnableSleep = DynamicBody, false
	bd.Position, bd.Name = Vec2{X: QFromInt(24), Y: QFromInt(3)}, "wheel"
	wheelBody := CreateBody(w, &bd)
	CreatePolygonShape(wheelBody, &sd, &small)
	wd := DefaultWheelJointDef()
	wd.Base.BodyIdA, wd.Base.BodyIdB = ground, wheelBody
	wd.Base.LocalFrameA.P = Vec2{X: QFromInt(24), Y: QMustParse("3.5")}
	wd.Base.LocalFrameA.Q = MakeRot(QMustParse("0.25"))
	wd.EnableMotor = true
	wd.MotorSpeed, wd.MaxMotorTorque = QOne(), QFromInt(10)
	CreateWheelJoint(w, &wd)

	bd = DefaultBodyDef()
	bd.Type, bd.IsAwake = DynamicBody, false
	bd.Position, bd.Name = Vec2{X: QFromInt(32), Y: QMustParse("0.25")}, "capsule"
	capsule := Capsule{Center1: Vec2{X: QHalf().Neg()}, Center2: Vec2{X: QHalf()}, Radius: QMustParse("0.25")}
	CreateCapsuleShape(CreateBody(w, &bd), &sd, &capsule)

	bd = DefaultBodyDef()
	bd.Position = Vec2{X: QFromInt(40), Y: QFromInt(5)}
	segment := Segment{Point1: Vec2{X: QOne().Neg()}, Point2: Vec2{X: QOne()}}
	CreateSegmentShape(CreateBody(w, &bd), &sd, &segment)

	bd = DefaultBodyDef()
	bd.Position, bd.Name = Vec2{X: QFromInt(48), Y: QFromInt(3)}, "sensor"
	sensorSd := DefaultShapeDef()
	sensorSd.IsSensor = true
	CreatePolygonShape(CreateBody(w, &bd), &sensorSd, &box)

	bd = DefaultBodyDef()
	bd.Position, bd.Name = Vec2{X: QFromInt(56), Y: QFromInt(5)}, "chain"
	cd := DefaultChainDef()
	cd.Points = []Vec2{v2(-1, -1), v2(1, -1), v2(1, 1), v2(-1, 1)}
	cd.IsLoop = true
	CreateChain(CreateBody(w, &bd), &cd)

	for range 90 {
		w.Step(stepDt(), 4)
	}
	return w
}

// TestDrawMatchesGolden replays tools/drawgolden: the "all" pass draws the
// whole world, the "bounded" pass a box around the left half with the
// contact forces at anchor A and a doubled joint scale.
func TestDrawMatchesGolden(t *testing.T) {
	w := createDrawScene(t)
	draw, calls := recordDraw()
	enableAllDrawFlags(draw)
	// A small force scale keeps fixed mode's force lines inside the budget.
	draw.ForceScale = QMustParse("0.015625")
	*calls = append(*calls, drawCall{kind: "all"})
	w.Draw(draw)
	draw.DrawContactNormals, draw.DrawContactForces, draw.DrawAnchorA = false, true, true
	draw.DrawGraphColors = false
	draw.JointScale = QFromInt(2)
	draw.DrawingBounds = AABB{LowerBound: Vec2{X: QFromInt(-36), Y: QFromInt(-1)}, UpperBound: Vec2{X: QFromInt(4), Y: QFromInt(5)}}
	*calls = append(*calls, drawCall{kind: "bounded"})
	w.Draw(draw)
	wantBytes, err := os.ReadFile("testdata/draw_golden.txt")
	if err != nil {
		t.Fatal(err)
	}
	lines := strings.Split(strings.ReplaceAll(strings.TrimSpace(string(wantBytes)), "\r\n", "\n"), "\n")
	if len(*calls) != len(lines) {
		t.Fatalf("draw calls = %d, want %d", len(*calls), len(lines))
	}
	for i, line := range lines {
		got := (*calls)[i]
		parts := strings.SplitN(line, "|", 4)
		if len(parts) == 1 {
			if got.kind != line {
				t.Fatalf("line %d: kind %s, want %s", i+1, got.kind, line)
			}
			continue
		}
		if len(parts) != 4 {
			t.Fatalf("line %d: malformed oracle", i+1)
		}
		color, err := strconv.ParseUint(parts[1], 16, 32)
		if err != nil {
			t.Fatal(err)
		}
		text, err := strconv.Unquote(parts[3])
		if err != nil {
			t.Fatal(err)
		}
		if got.kind != parts[0] || got.color != HexColor(color) || !drawTextMatches(got.text, text) {
			t.Fatalf("line %d: got %s|%06X|%q, want %s|%s|%q", i+1, got.kind, got.color, got.text, parts[0], parts[1], text)
		}
		values := strings.Fields(parts[2])
		if len(got.values) != len(values) {
			t.Fatalf("line %d: argument count %d, want %d", i+1, len(got.values), len(values))
		}
		for j, value := range values {
			want, err := strconv.ParseFloat(value, 64)
			if err != nil {
				t.Fatal(err)
			}
			if math.Abs(got.values[j]-want) > 1e-3 {
				t.Fatalf("line %d argument %d: %.9f, want %.9f (budget 1e-3)", i+1, j, got.values[j], want)
			}
		}
	}
}

// drawNumberPattern matches the numbers of a label, "%g" exponents included.
var drawNumberPattern = regexp.MustCompile(`-?[0-9]+(\.[0-9]+)?(e[-+][0-9]+)?`)

// Numeric labels have the same fixed-point budget as geometry; names stay
// exact. A label may hold several numbers, as the joint force label does.
func drawTextMatches(got, want string) bool {
	if got == want {
		return true
	}
	if drawNumberPattern.ReplaceAllString(got, "#") != drawNumberPattern.ReplaceAllString(want, "#") {
		return false
	}
	gotNumbers := drawNumberPattern.FindAllString(got, -1)
	wantNumbers := drawNumberPattern.FindAllString(want, -1)
	for i, w := range wantNumbers {
		a, errA := strconv.ParseFloat(gotNumbers[i], 64)
		b, errB := strconv.ParseFloat(w, 64)
		limit := 1e-3
		// In fixed mode, a label near a rounding boundary may print one unit
		// apart in its last place.
		if j := strings.IndexByte(w, '.'); j >= 0 && ScalarMode == "fixed" {
			limit = math.Max(limit, 1.0001*math.Pow(10, -float64(len(w)-j-1)))
		}
		if errA != nil || errB != nil || math.Abs(a-b) > limit {
			return false
		}
	}
	return true
}

func TestDrawShapeCallbacksPerType(t *testing.T) {
	transform := Transform{P: Vec2{X: QFromInt(3), Y: QFromInt(4)}, Q: MakeRot(QMustParse("0.25"))}
	segment := Segment{Point1: Vec2{X: QOne()}, Point2: Vec2{X: QFromInt(2)}}
	cases := []struct {
		name  string
		shape shape
		kinds []string
	}{
		{"circle", withGeometry(CircleShape, &Circle{Center: segment.Point1, Radius: QHalf()}), []string{"solidCircle"}},
		{"capsule", withGeometry(CapsuleShape, &Capsule{Center1: segment.Point1, Center2: segment.Point2, Radius: QHalf()}), []string{"capsule"}},
		{"polygon", withGeometry(PolygonShape, ptr(MakeSquare(QHalf()))), []string{"solidPolygon"}},
		{"segment", withGeometry(SegmentShape, &segment), []string{"line"}},
		{"chain", withGeometry(ChainSegmentShape, &ChainSegment{Segment: segment}), []string{"line", "point", "line"}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			draw, calls := recordDraw()
			drawShape(draw, &tc.shape, transform, ColorCoral, true)
			if len(*calls) != len(tc.kinds) {
				t.Fatalf("callbacks = %+v", *calls)
			}
			for i, kind := range tc.kinds {
				if (*calls)[i].kind != kind {
					t.Fatalf("callbacks = %+v", *calls)
				}
			}
			if (*calls)[0].color != ColorCoral {
				t.Fatalf("callbacks = %+v", *calls)
			}
			if tc.kinds[0] == "line" || tc.kinds[0] == "capsule" {
				if (*calls)[0].values[0] != 3 || (*calls)[0].values[1] != 5 {
					t.Fatalf("transformed first point = %v", (*calls)[0].values)
				}
			}
			if tc.kinds[0] == "solidCircle" {
				// The circle keeps its local center beside the body transform.
				if v := (*calls)[0].values; v[0] != 3 || v[1] != 4 || v[4] != 1 || v[5] != 0 {
					t.Fatalf("circle transform and center = %v", v)
				}
			}
		})
	}
}

func TestDrawRejectsLockedWorld(t *testing.T) {
	w := createTestWorld(t)
	state := getWorldFromId(w)
	state.locked = true
	defer func() { state.locked = false }()
	draw := DefaultDebugDraw()
	requirePanic(t, func() { w.Draw(&draw) })
}

// TestDrawRejectsLockedWorldDuringStep drives the panic through a real
// callback, since the world is only actually locked while Step runs.
func TestDrawRejectsLockedWorldDuringStep(t *testing.T) {
	w := createTestWorld(t)
	addFilteredBox(t, w, v2(0, 0))
	addDynamicBox(t, w, v2(1, 0))
	draw := DefaultDebugDraw()
	var message any
	w.SetCustomFilterCallback(func(ShapeId, ShapeId) bool {
		func() {
			defer func() { message = recover() }()
			w.Draw(&draw)
		}()
		return true
	})
	for range 5 {
		w.Step(stepDt(), 4)
	}
	text, ok := message.(string)
	if !ok || !strings.HasPrefix(text, "dbox2d: ") {
		t.Fatalf("panic message = %v, want a \"dbox2d: \" panic", message)
	}
}

func TestDrawBoundsExcludeAllCallbacksOutsideTheQuery(t *testing.T) {
	w := createDrawScene(t)
	draw, calls := recordDraw()
	enableAllDrawFlags(draw)
	draw.DrawingBounds = AABB{LowerBound: v2(100, 100), UpperBound: v2(101, 101)}
	w.Draw(draw)
	if len(*calls) != 0 {
		t.Fatalf("empty query emitted %d callbacks", len(*calls))
	}
	draw.DrawingBounds = AABB{LowerBound: v2(1, 1), UpperBound: v2(-1, -1)}
	requirePanic(t, func() { w.Draw(draw) })
}

func TestDrawBoundsVisitLinkedJointOnlyOnce(t *testing.T) {
	w := createTestWorld(t)
	a := addDynamicBox(t, w, v2(0, 0))
	b := addDynamicBox(t, w, v2(3, 0))
	jd := DefaultDistanceJointDef()
	jd.Base.BodyIdA, jd.Base.BodyIdB, jd.Length = a, b, QFromInt(3)
	CreateDistanceJoint(w, &jd)
	for _, upper := range []int{1, 4} {
		draw, calls := recordDraw()
		draw.DrawJoints = true
		draw.DrawingBounds = AABB{LowerBound: v2(-1, -1), UpperBound: v2(upper, 1)}
		w.Draw(draw)
		if len(*calls) != 3 || (*calls)[0].kind != "line" {
			t.Fatalf("query upper=%d: callbacks = %+v", upper, *calls)
		}
	}
}

func TestDrawPreservesSimulationAndCallbackOrder(t *testing.T) {
	w, control := createDrawScene(t), createDrawScene(t)
	before := Checksum(w)
	for _, bounded := range []bool{false, true} {
		draw, calls := recordDraw()
		enableAllDrawFlags(draw)
		if bounded {
			draw.DrawingBounds = AABB{LowerBound: v2(-10, -10), UpperBound: v2(10, 10)}
		}
		w.Draw(draw)
		first := append([]drawCall(nil), (*calls)...)
		*calls = nil
		w.Draw(draw)
		if !reflect.DeepEqual(first, *calls) {
			t.Fatal("drawing changed callback order or arguments")
		}
		if got := Checksum(w); got != before {
			t.Fatalf("drawing changed checksum: %d -> %d", before, got)
		}
	}
	for range 120 {
		w.Step(stepDt(), 4)
		control.Step(stepDt(), 4)
	}
	if Checksum(w) != Checksum(control) {
		t.Fatal("drawing changed future simulation")
	}
}

// TestDrawContactDiagnosticsMatchReferenceUnits pins the force labels of
// b2World_Draw: half the impulse times the inverse step, the normal force
// over the full step and the friction force over the substep.
func TestDrawContactDiagnosticsMatchReferenceUnits(t *testing.T) {
	m := Manifold{Normal: Vec2{Y: QOne()}, PointCount: 1}
	m.Points[0] = ManifoldPoint{
		AnchorA: v2(1, 0), AnchorB: v2(-1, 0),
		TotalNormalImpulse: QHalf(), TangentImpulse: QMustParse("-0.25"), Id: 123,
	}
	invDt, invH := QFromInt(60), QFromInt(240)
	for _, anchorA := range []bool{false, true} {
		draw, calls := recordDraw()
		draw.DrawGraphColors, draw.DrawContactForces, draw.DrawContactFeatures, draw.DrawFrictionForces = true, true, true, true
		draw.DrawAnchorA = anchorA
		draw.ForceScale = QMustParse("0.125")
		// Both anchors land on (2, 3).
		drawManifoldPoints(draw, &m, v2(1, 3), v2(3, 3), overflowIndex, invDt, invH)
		if len(*calls) != 6 {
			t.Fatalf("diagnostic callbacks = %+v", *calls)
		}
		if c := (*calls)[0]; c.kind != "point" || c.color != ColorSilver || c.values[0] != 2 || c.values[2] != 7.5 {
			t.Fatalf("overflow point = %+v", c)
		}
		// force = 0.5 * 0.5 * 60 = 15, drawn 15 * 0.125 = 1.875 long
		if c := (*calls)[1]; c.color != ColorMagenta || c.values[3] != 4.875 {
			t.Fatalf("normal force line = %+v", c)
		}
		// friction = 0.5 * -0.25 * 240 = -30, drawn -3.75 along (1, 0)
		if (*calls)[2].text != "15.0" || (*calls)[3].text != "123" || (*calls)[5].text != "-30.0" {
			t.Fatalf("labels = %+v", *calls)
		}
		if c := (*calls)[4]; c.color != ColorYellow || c.values[2] != -1.75 || c.values[3] != 3 {
			t.Fatalf("friction line = %+v", c)
		}
		*calls = nil
		draw.DrawContactNormals = true
		drawManifoldPoints(draw, &m, v2(1, 3), v2(3, 3), overflowIndex, invDt, invH)
		if len(*calls) != 6 || (*calls)[1].color != ColorDimGray || (*calls)[2].text != " 0.00" {
			t.Fatalf("normals must take precedence over forces: %+v", *calls)
		}
	}
}

func TestDrawJointDiagnosticsRespectFlags(t *testing.T) {
	w := createTestWorld(t)
	// Draw reaches a joint through the shapes of its bodies.
	bd := DefaultBodyDef()
	sd := DefaultShapeDef()
	circle := Circle{Radius: QHalf()}
	a := CreateBody(w, &bd)
	CreateCircleShape(a, &sd, &circle)
	bd.Type = DynamicBody
	b := CreateBody(w, &bd)
	CreateCircleShape(b, &sd, &circle)
	jd := DefaultRevoluteJointDef()
	jd.Base.BodyIdA, jd.Base.BodyIdB = a, b
	CreateRevoluteJoint(w, &jd)
	draw, calls := recordDraw()
	draw.DrawJoints = true
	w.Draw(draw)
	for _, c := range *calls {
		if c.kind == "point" || c.kind == "string" {
			t.Fatalf("disabled diagnostics: %+v", c)
		}
	}
	*calls = nil
	draw.DrawJointExtras, draw.DrawGraphColors = true, true
	w.Draw(draw)
	var points, labels int
	for _, c := range *calls {
		if c.kind == "point" {
			points++
			if c.color != graphColors[getWorldFromId(w).joints[0].colorIndex] {
				t.Fatal("wrong joint color")
			}
		}
		if c.kind == "string" {
			labels++
			if c.text != " 0.0 deg" && !strings.HasPrefix(c.text, "f = [") {
				t.Fatalf("joint label = %q", c.text)
			}
		}
	}
	// The labels are the joint angle and the constraint force and torque.
	if points != 1 || labels != 2 {
		t.Fatalf("diagnostics points=%d labels=%d", points, labels)
	}
}

func TestDrawDistanceLimitsPreserveEndpointOrder(t *testing.T) {
	draw := DefaultDebugDraw()
	var segments [][2]Vec2
	draw.DrawLineFcn = func(a, b Vec2, _ HexColor) { segments = append(segments, [2]Vec2{a, b}) }
	var base jointSim
	*base.distance() = distanceJoint{enableLimit: true, minLength: QOne(), maxLength: QFromInt(3)}
	drawDistanceJoint(&draw, &base, TransformIdentity(), Transform{P: v2(2, 0), Q: RotIdentity()})
	want := [2]Vec2{{X: QOne(), Y: QMustParse("0.05")}, {X: QOne(), Y: QMustParse("-0.05")}}
	if len(segments) != 4 || segments[0] != want {
		t.Fatalf("limit segments = %+v", segments)
	}
}

// withGeometry builds a bare shape of one type around its geometry.
func withGeometry(shapeType ShapeType, geometry any) shape {
	s := shape{shapeType: shapeType}
	switch g := geometry.(type) {
	case *Capsule:
		*s.capsule() = *g
	case *Circle:
		*s.circle() = *g
	case *Polygon:
		*s.polygon() = *g
	case *Segment:
		*s.segment() = *g
	case *ChainSegment:
		*s.chainSegment() = *g
	}
	return s
}

func ptr[T any](v T) *T { return &v }

// TestDrawNumberPrintsSmallValuesInPlainDecimal pins the label of a value
// that float mode prints with an exponent: -2^-20 is -0.00, not -9.54.
func TestDrawNumberPrintsSmallValuesInPlainDecimal(t *testing.T) {
	if got := drawNumber(QFromRatio(-1, 1<<20), 2); got != "-0.00" {
		t.Errorf("drawNumber(-2^-20, 2) = %q, want -0.00", got)
	}
	if got := drawNumber(QFromInt(1<<20), 1); got != "1048576.0" {
		t.Errorf("drawNumber(2^20, 1) = %q, want 1048576.0", got)
	}
}
