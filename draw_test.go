package dbox2d

import (
	"math"
	"os"
	"reflect"
	"slices"
	"strconv"
	"strings"
	"testing"

	"github.com/dhannyell/fixed"
)

func TestDefaultDebugDrawIsCallable(t *testing.T) {
	draw := DefaultDebugDraw()
	draw.DrawPolygon(nil, ColorBlack)
	draw.DrawSolidPolygon(Transform{}, nil, Q{}, ColorBlack)
	draw.DrawCircle(Vec2{}, Q{}, ColorBlack)
	draw.DrawSolidCircle(Transform{}, Q{}, ColorBlack)
	draw.DrawSolidCapsule(Vec2{}, Vec2{}, Q{}, ColorBlack)
	draw.DrawSegment(Vec2{}, Vec2{}, ColorBlack)
	draw.DrawTransform(Transform{})
	draw.DrawPoint(Vec2{}, Q{}, ColorBlack)
	draw.DrawString(Vec2{}, "", ColorBlack)
}

func TestDefaultRevoluteJointDefDrawSize(t *testing.T) {
	if got, want := DefaultRevoluteJointDef().DrawSize.String(), "0.25"; got != want {
		t.Fatalf("DrawSize = %s, want %s", got, want)
	}
}

func TestDrawRevoluteJointLimitsUseTurns(t *testing.T) {
	var segments [][2]Vec2
	draw := DefaultDebugDraw()
	draw.DrawSegment = func(a, b Vec2, _ HexColor) { segments = append(segments, [2]Vec2{a, b}) }
	base := jointSim{revoluteJoint: revoluteJoint{
		enableLimit: true,
		lowerAngle:  fixed.Q32MustParse("-0.25"),
		upperAngle:  fixed.Q32MustParse("0.25"),
	}}
	drawRevoluteJoint(&draw, &base, TransformIdentity(), TransformIdentity(), fixed.Q32One())
	if len(segments) < 4 {
		t.Fatalf("segments = %d, want limit segments", len(segments))
	}
	lowerWant := Vec2{Y: fixed.Q32One().Neg()}
	if got := segments[1][1]; got != lowerWant {
		t.Fatalf("lower limit endpoint = %#v, want %#v", got, lowerWant)
	}
	upperWant := Vec2{Y: fixed.Q32One()}
	if got := segments[2][1]; got != upperWant {
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
		{"shapes", func(d *DebugDraw) { d.DrawShapes = true }, []string{"solidPolygon", "solidCircle", "capsule", "segment"}},
		{"joints", func(d *DebugDraw) { d.DrawJoints = true }, []string{"circle", "segment", "point"}},
		{"bounds", func(d *DebugDraw) { d.DrawBounds = true }, []string{"polygon", "string"}},
		{"mass", func(d *DebugDraw) { d.DrawMass = true }, []string{"transform", "string"}},
		{"names", func(d *DebugDraw) { d.DrawBodyNames = true }, []string{"string"}},
		{"contacts", func(d *DebugDraw) { d.DrawContacts = true }, []string{"point"}},
		{"islands", func(d *DebugDraw) { d.DrawIslands = true }, []string{"polygon"}},
		{"joint extras alone", func(d *DebugDraw) { d.DrawJointExtras = true }, nil},
		{"graph colors alone", func(d *DebugDraw) { d.DrawGraphColors = true }, nil},
		{"normals alone", func(d *DebugDraw) { d.DrawContactNormals = true }, nil},
		{"impulses alone", func(d *DebugDraw) { d.DrawContactImpulses = true }, nil},
		{"features alone", func(d *DebugDraw) { d.DrawContactFeatures = true }, nil},
		{"friction alone", func(d *DebugDraw) { d.DrawFrictionImpulses = true }, nil},
		{"normals", func(d *DebugDraw) { d.DrawContacts, d.DrawContactNormals = true, true }, []string{"point", "segment"}},
		{"impulses", func(d *DebugDraw) { d.DrawContacts, d.DrawContactImpulses = true, true }, []string{"point", "segment", "string"}},
		{"features", func(d *DebugDraw) { d.DrawContacts, d.DrawContactFeatures = true, true }, []string{"point", "string"}},
		{"friction", func(d *DebugDraw) { d.DrawContacts, d.DrawFrictionImpulses = true, true }, []string{"point", "segment", "string"}},
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

// recordDraw retains every argument so the oracle detects geometry and order changes.
func recordDraw() (*DebugDraw, *[]drawCall) {
	var calls []drawCall
	q := func(v Q) float64 { return float64(v.Raw()) / (1 << 32) }
	vec := func(p Vec2) []float64 { return []float64{q(p.X), q(p.Y)} }
	xf := func(x Transform) []float64 { return []float64{q(x.P.X), q(x.P.Y), q(x.Q.Cos), q(x.Q.Sin)} }
	add := func(kind string, color HexColor, values []float64, text string) {
		calls = append(calls, drawCall{kind, color, values, text})
	}
	draw := DefaultDebugDraw()
	draw.DrawPolygon = func(v []Vec2, c HexColor) {
		values := []float64{float64(len(v))}
		for _, p := range v {
			values = append(values, vec(p)...)
		}
		add("polygon", c, values, "")
	}
	draw.DrawSolidPolygon = func(x Transform, v []Vec2, r Q, c HexColor) {
		values := append(xf(x), q(r), float64(len(v)))
		for _, p := range v {
			values = append(values, vec(p)...)
		}
		add("solidPolygon", c, values, "")
	}
	draw.DrawCircle = func(p Vec2, r Q, c HexColor) { add("circle", c, append(vec(p), q(r)), "") }
	draw.DrawSolidCircle = func(x Transform, r Q, c HexColor) { add("solidCircle", c, append(xf(x), q(r)), "") }
	draw.DrawSolidCapsule = func(a, b Vec2, r Q, c HexColor) { add("capsule", c, append(append(vec(a), vec(b)...), q(r)), "") }
	draw.DrawSegment = func(a, b Vec2, c HexColor) { add("segment", c, append(vec(a), vec(b)...), "") }
	draw.DrawTransform = func(x Transform) { add("transform", 0, xf(x), "") }
	draw.DrawPoint = func(p Vec2, size Q, c HexColor) { add("point", c, append(vec(p), q(size)), "") }
	draw.DrawString = func(p Vec2, s string, c HexColor) { add("string", c, vec(p), s) }
	return &draw, &calls
}

func enableAllDrawFlags(draw *DebugDraw) {
	draw.DrawShapes, draw.DrawJoints, draw.DrawJointExtras = true, true, true
	draw.DrawBounds, draw.DrawMass, draw.DrawBodyNames = true, true, true
	draw.DrawContacts, draw.DrawGraphColors, draw.DrawContactNormals = true, true, true
	draw.DrawContactImpulses, draw.DrawContactFeatures, draw.DrawFrictionImpulses, draw.DrawIslands = true, true, true, true
}

// createDrawScene spaces every dynamic body far apart so each one touches
// only the ground: a moved proxy then finds at most one new pair, which
// keeps the port's pair order and graph colors identical to the
// reference's (D-013). No gap equals the speculative distance (4*linearSlop).
func createDrawScene(t testing.TB) WorldId {
	t.Helper()
	w := createTestWorld(t)
	sd := DefaultShapeDef()
	bd := DefaultBodyDef()
	bd.Position.Y = fixed.Q32Half().Neg()
	bd.Name = "ground"
	ground := CreateBody(w, &bd)
	floor := MakeBox(fixed.Q32FromInt(40), fixed.Q32Half())
	CreatePolygonShape(ground, &sd, &floor)

	box := MakeSquare(fixed.Q32Half())
	bd = DefaultBodyDef()
	bd.Type, bd.EnableSleep = DynamicBody, false
	bd.Position, bd.Name = Vec2{X: fixed.Q32FromInt(-32), Y: fixed.Q32FromInt(3)}, "box"
	bd.Rotation, bd.FixedRotation = MakeRot(fixed.Q32MustParse("0.1")), true
	CreatePolygonShape(CreateBody(w, &bd), &sd, &box)

	slideSd := DefaultShapeDef()
	// A lower friction than the default keeps the box sliding through every step.
	slideSd.Material.Friction = fixed.Q32MustParse("0.1")
	bd = DefaultBodyDef()
	bd.Type, bd.EnableSleep = DynamicBody, false
	bd.Position, bd.Name = Vec2{X: fixed.Q32FromInt(-24), Y: fixed.Q32Half()}, "sliding"
	bd.LinearVelocity = Vec2{X: fixed.Q32FromInt(3)}
	CreatePolygonShape(CreateBody(w, &bd), &slideSd, &box)

	bd = DefaultBodyDef()
	bd.Type, bd.EnableSleep = DynamicBody, false
	bd.Position, bd.Name = Vec2{X: fixed.Q32FromInt(-16), Y: fixed.Q32FromInt(3)}, "circle"
	circle := Circle{Radius: fixed.Q32MustParse("0.25")}
	CreateCircleShape(CreateBody(w, &bd), &sd, &circle)

	bd = DefaultBodyDef()
	bd.Type, bd.EnableSleep = DynamicBody, false
	bd.Position, bd.Name = Vec2{X: fixed.Q32FromInt(-8), Y: fixed.Q32FromInt(3)}, "rounded"
	rounded := MakeRoundedBox(fixed.Q32MustParse("0.4"), fixed.Q32MustParse("0.4"), fixed.Q32MustParse("0.1"))
	CreatePolygonShape(CreateBody(w, &bd), &sd, &rounded)

	bd = DefaultBodyDef()
	bd.Type, bd.EnableSleep = DynamicBody, false
	bd.Position, bd.Name = Vec2{X: fixed.Q32Zero(), Y: fixed.Q32FromInt(3)}, "hinge"
	hinge := CreateBody(w, &bd)
	CreatePolygonShape(hinge, &sd, &box)
	jd := DefaultRevoluteJointDef()
	jd.BodyIdA, jd.BodyIdB = ground, hinge
	jd.LocalAnchorA = Vec2{X: fixed.Q32Zero(), Y: fixed.Q32MustParse("3.5")}
	jd.EnableLimit = true
	jd.LowerAngle, jd.UpperAngle = fixed.Q32MustParse("-0.25"), fixed.Q32MustParse("0.25")
	CreateRevoluteJoint(w, &jd)

	small := MakeSquare(fixed.Q32MustParse("0.3"))
	bd = DefaultBodyDef()
	bd.Type, bd.EnableSleep = DynamicBody, false
	bd.Position, bd.Name = Vec2{X: fixed.Q32FromInt(8), Y: fixed.Q32FromInt(3)}, "distance"
	distanceBody := CreateBody(w, &bd)
	CreatePolygonShape(distanceBody, &sd, &small)
	dd := DefaultDistanceJointDef()
	dd.BodyIdA, dd.BodyIdB = ground, distanceBody
	dd.LocalAnchorA = Vec2{X: fixed.Q32FromInt(8), Y: fixed.Q32MustParse("5.5")}
	dd.Length = fixed.Q32FromInt(2)
	dd.EnableLimit = true
	dd.MinLength, dd.MaxLength = fixed.Q32One(), fixed.Q32FromInt(3)
	CreateDistanceJoint(w, &dd)

	bd = DefaultBodyDef()
	bd.Type, bd.EnableSleep = DynamicBody, false
	bd.Position, bd.Name = Vec2{X: fixed.Q32FromInt(16), Y: fixed.Q32FromInt(6)}, "prismatic"
	prismaticBody := CreateBody(w, &bd)
	CreatePolygonShape(prismaticBody, &sd, &box)
	pd := DefaultPrismaticJointDef()
	pd.BodyIdA, pd.BodyIdB = ground, prismaticBody
	pd.LocalAnchorA = Vec2{X: fixed.Q32FromInt(16), Y: fixed.Q32MustParse("3.5")}
	pd.LocalAxisA = Vec2{Y: fixed.Q32One()}
	pd.EnableLimit = true
	pd.LowerTranslation, pd.UpperTranslation = fixed.Q32Zero(), fixed.Q32FromInt(3)
	CreatePrismaticJoint(w, &pd)

	bd = DefaultBodyDef()
	bd.Type, bd.EnableSleep = DynamicBody, false
	bd.Position, bd.Name = Vec2{X: fixed.Q32FromInt(24), Y: fixed.Q32FromInt(3)}, "wheel"
	wheelBody := CreateBody(w, &bd)
	CreatePolygonShape(wheelBody, &sd, &small)
	wd := DefaultWheelJointDef()
	wd.BodyIdA, wd.BodyIdB = ground, wheelBody
	wd.LocalAnchorA = Vec2{X: fixed.Q32FromInt(24), Y: fixed.Q32MustParse("3.5")}
	wd.EnableMotor = true
	wd.MotorSpeed, wd.MaxMotorTorque = fixed.Q32One(), fixed.Q32FromInt(10)
	CreateWheelJoint(w, &wd)

	bd = DefaultBodyDef()
	bd.Type, bd.IsAwake = DynamicBody, false
	bd.Position, bd.Name = Vec2{X: fixed.Q32FromInt(32), Y: fixed.Q32MustParse("0.25")}, "capsule"
	capsule := Capsule{Center1: Vec2{X: fixed.Q32Half().Neg()}, Center2: Vec2{X: fixed.Q32Half()}, Radius: fixed.Q32MustParse("0.25")}
	CreateCapsuleShape(CreateBody(w, &bd), &sd, &capsule)

	bd = DefaultBodyDef()
	bd.Position = Vec2{X: fixed.Q32FromInt(40), Y: fixed.Q32FromInt(5)}
	segment := Segment{Point1: Vec2{X: fixed.Q32One().Neg()}, Point2: Vec2{X: fixed.Q32One()}}
	CreateSegmentShape(CreateBody(w, &bd), &sd, &segment)

	bd = DefaultBodyDef()
	bd.Position, bd.Name = Vec2{X: fixed.Q32FromInt(48), Y: fixed.Q32FromInt(3)}, "sensor"
	sensorSd := DefaultShapeDef()
	sensorSd.IsSensor = true
	CreatePolygonShape(CreateBody(w, &bd), &sensorSd, &box)

	for range 90 {
		w.Step(stepDt(), 4)
	}
	return w
}

func TestDrawMatchesGolden(t *testing.T) {
	w := createDrawScene(t)
	draw, calls := recordDraw()
	enableAllDrawFlags(draw)
	*calls = append(*calls, drawCall{kind: "all"})
	w.Draw(draw)
	draw.UseDrawingBounds = true
	draw.DrawingBounds = AABB{LowerBound: Vec2{X: fixed.Q32FromInt(-36), Y: fixed.Q32FromInt(-1)}, UpperBound: Vec2{X: fixed.Q32FromInt(4), Y: fixed.Q32FromInt(5)}}
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

// Numeric labels have the same fixed-point budget as geometry; names stay exact.
func drawTextMatches(got, want string) bool {
	if got == want {
		return true
	}
	if strings.HasSuffix(got, " deg") != strings.HasSuffix(want, " deg") {
		return false
	}
	a, errA := strconv.ParseFloat(strings.TrimSpace(strings.TrimSuffix(got, " deg")), 64)
	b, errB := strconv.ParseFloat(strings.TrimSpace(strings.TrimSuffix(want, " deg")), 64)
	return errA == nil && errB == nil && math.Abs(a-b) <= 1e-3
}

func TestDrawShapeCallsOneCallbackPerType(t *testing.T) {
	transform := Transform{P: Vec2{X: fixed.Q32FromInt(3), Y: fixed.Q32FromInt(4)}, Q: MakeRot(fixed.Q32MustParse("0.25"))}
	segment := Segment{Point1: Vec2{X: fixed.Q32One()}, Point2: Vec2{X: fixed.Q32FromInt(2)}}
	cases := []struct {
		name  string
		shape shape
		kind  string
	}{
		{"circle", shape{shapeType: CircleShape, circle: Circle{Center: segment.Point1, Radius: fixed.Q32Half()}}, "solidCircle"},
		{"capsule", shape{shapeType: CapsuleShape, capsule: Capsule{Center1: segment.Point1, Center2: segment.Point2, Radius: fixed.Q32Half()}}, "capsule"},
		{"polygon", shape{shapeType: PolygonShape, polygon: MakeSquare(fixed.Q32Half())}, "solidPolygon"},
		{"segment", shape{shapeType: SegmentShape, segment: segment}, "segment"},
		{"chain", shape{shapeType: ChainSegmentShape, chainSegment: ChainSegment{Segment: segment}}, "segment"},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			draw, calls := recordDraw()
			drawShape(draw, &tc.shape, transform, ColorCoral)
			if len(*calls) != 1 || (*calls)[0].kind != tc.kind || (*calls)[0].color != ColorCoral {
				t.Fatalf("callbacks = %+v", *calls)
			}
			if tc.kind == "segment" || tc.kind == "capsule" || tc.kind == "solidCircle" {
				if (*calls)[0].values[0] != 3 || (*calls)[0].values[1] != 5 {
					t.Fatalf("transformed first point = %v", (*calls)[0].values)
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
	addDynamicBox(t, w, v2(0, 0))
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
	draw.UseDrawingBounds = true
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
	jd.BodyIdA, jd.BodyIdB, jd.Length = a, b, fixed.Q32FromInt(3)
	CreateDistanceJoint(w, &jd)
	for _, upper := range []int{1, 4} {
		draw, calls := recordDraw()
		draw.DrawJoints, draw.UseDrawingBounds = true, true
		draw.DrawingBounds = AABB{LowerBound: v2(-1, -1), UpperBound: v2(upper, 1)}
		w.Draw(draw)
		if len(*calls) != 3 || (*calls)[0].kind != "segment" {
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
		draw.UseDrawingBounds = bounded
		draw.DrawingBounds = AABB{LowerBound: v2(-10, -10), UpperBound: v2(10, 10)}
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

func TestDrawContactDiagnosticsMatchReferenceUnits(t *testing.T) {
	m := Manifold{Normal: Vec2{Y: fixed.Q32One()}, PointCount: 1}
	m.Points[0] = ManifoldPoint{Point: v2(2, 3), NormalImpulse: fixed.Q32MustParse("0.125"), TotalNormalImpulse: fixed.Q32Half(), TangentImpulse: fixed.Q32MustParse("-0.25"), Id: 123}
	for _, tc := range []struct {
		bounded          bool
		end              float64
		normal, friction string
	}{
		{false, 3.5, "500.00", "-0.25"},
		{true, 3.125, "125.0", "-250.0"},
	} {
		draw, calls := recordDraw()
		draw.DrawGraphColors, draw.DrawContactImpulses, draw.DrawContactFeatures, draw.DrawFrictionImpulses = true, true, true, true
		drawContactManifold(draw, &m, overflowIndex, tc.bounded)
		if len(*calls) != 6 {
			t.Fatalf("diagnostic callbacks = %+v", *calls)
		}
		if c := (*calls)[0]; c.kind != "point" || c.color != ColorBlack || c.values[2] != 7.5 {
			t.Fatalf("overflow point = %+v", c)
		}
		if c := (*calls)[1]; c.color != ColorMagenta || c.values[3] != tc.end {
			t.Fatalf("normal segment = %+v", c)
		}
		if (*calls)[2].text != tc.normal || (*calls)[3].text != "123" || (*calls)[5].text != tc.friction {
			t.Fatalf("labels = %+v", *calls)
		}
		if c := (*calls)[4]; c.color != ColorYellow || c.values[2] != 1.75 || c.values[3] != 3 {
			t.Fatalf("friction segment = %+v", c)
		}
		*calls = nil
		draw.DrawContactNormals = true
		drawContactManifold(draw, &m, overflowIndex, tc.bounded)
		if len(*calls) != 5 || (*calls)[1].color != ColorDimGray {
			t.Fatalf("normals must take precedence over impulses: %+v", *calls)
		}
	}
}

func TestDrawJointDiagnosticsRespectFlags(t *testing.T) {
	w := createTestWorld(t)
	bd := DefaultBodyDef()
	a := CreateBody(w, &bd)
	bd.Type = DynamicBody
	b := CreateBody(w, &bd)
	jd := DefaultRevoluteJointDef()
	jd.BodyIdA, jd.BodyIdB = a, b
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
			if c.text != " 0.0 deg" {
				t.Fatalf("angle label = %q", c.text)
			}
		}
	}
	if points != 1 || labels != 1 {
		t.Fatalf("diagnostics points=%d labels=%d", points, labels)
	}
}

func TestDrawDistanceLimitsPreserveEndpointOrder(t *testing.T) {
	draw := DefaultDebugDraw()
	var segments [][2]Vec2
	draw.DrawSegment = func(a, b Vec2, _ HexColor) { segments = append(segments, [2]Vec2{a, b}) }
	base := jointSim{distanceJoint: distanceJoint{enableLimit: true, minLength: fixed.Q32One(), maxLength: fixed.Q32FromInt(3)}}
	drawDistanceJoint(&draw, &base, TransformIdentity(), Transform{P: v2(2, 0), Q: RotIdentity()})
	want := [2]Vec2{{X: fixed.Q32One(), Y: fixed.Q32MustParse("0.05")}, {X: fixed.Q32One(), Y: fixed.Q32MustParse("-0.05")}}
	if len(segments) != 4 || segments[0] != want {
		t.Fatalf("limit segments = %+v", segments)
	}
}

func TestDrawNumberRoundsLabelsWithoutFloat(t *testing.T) {
	for _, tc := range []struct {
		value  string
		places int
		want   string
	}{
		{"1.125", 2, "1.12"}, {"1.375", 2, "1.38"}, {"-1.375", 2, "-1.38"}, {"0", 2, "0.00"}, {"-0.001", 2, "-0.00"}, {"9.999", 2, "10.00"}, {"2147483647.5", 2, "2147483647.50"},
	} {
		if got := drawNumber(fixed.Q32MustParse(tc.value), tc.places); got != tc.want {
			t.Errorf("%s: %s, want %s", tc.value, got, tc.want)
		}
	}
}
