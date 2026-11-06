package b2

import (
	"math"
	"testing"
)

func TestShapeAccessorsRoundTrip(t *testing.T) {
	worldId := createTestWorld(t)
	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyId := CreateBody(worldId, &bodyDef)
	shapeDef := DefaultShapeDef()
	box := MakeSquare(QHalf())
	shapeId := CreatePolygonShape(bodyId, &shapeDef, &box)

	checks := []struct {
		name string
		set  func()
		got  func() bool
	}{
		{
			name: "user data",
			set:  func() { shapeId.SetUserData("shape") },
			got:  func() bool { return shapeId.GetUserData() == "shape" },
		},
		{
			name: "friction",
			set:  func() { shapeId.SetFriction(QMustParse("0.25")) },
			got:  func() bool { return shapeId.GetFriction().Eq(QMustParse("0.25")) },
		},
		{
			name: "restitution",
			set:  func() { shapeId.SetRestitution(QMustParse("0.75")) },
			got:  func() bool { return shapeId.GetRestitution().Eq(QMustParse("0.75")) },
		},
		{
			name: "material",
			set:  func() { shapeId.SetUserMaterial(7) },
			got:  func() bool { return shapeId.GetUserMaterial() == 7 },
		},
		{
			name: "surface material",
			set: func() {
				shapeId.SetSurfaceMaterial(SurfaceMaterial{
					Friction:          QMustParse("0.4"),
					Restitution:       QMustParse("0.6"),
					RollingResistance: QMustParse("0.2"),
					TangentSpeed:      QMustParse("1.5"),
					UserMaterialId:    7,
					CustomColor:       0x12345678,
				})
			},
			got: func() bool {
				return shapeId.GetSurfaceMaterial() == (SurfaceMaterial{
					Friction:          QMustParse("0.4"),
					Restitution:       QMustParse("0.6"),
					RollingResistance: QMustParse("0.2"),
					TangentSpeed:      QMustParse("1.5"),
					UserMaterialId:    7,
					CustomColor:       0x12345678,
				})
			},
		},
		{
			name: "filter",
			set:  func() { shapeId.SetFilter(Filter{CategoryBits: 2, MaskBits: 4, GroupIndex: -1}) },
			got:  func() bool { return shapeId.GetFilter() == (Filter{CategoryBits: 2, MaskBits: 4, GroupIndex: -1}) },
		},
	}
	for _, check := range checks {
		check.set()
		if !check.got() {
			t.Errorf("%s did not round-trip", check.name)
		}
	}

	boolChecks := []struct {
		name string
		set  func(bool)
		got  func() bool
	}{
		{"sensor events", shapeId.EnableSensorEvents, shapeId.AreSensorEventsEnabled},
		{"contact events", shapeId.EnableContactEvents, shapeId.AreContactEventsEnabled},
		{"pre-solve events", shapeId.EnablePreSolveEvents, shapeId.ArePreSolveEventsEnabled},
		{"hit events", shapeId.EnableHitEvents, shapeId.AreHitEventsEnabled},
	}
	for _, check := range boolChecks {
		for _, want := range []bool{true, false} {
			check.set(want)
			if got := check.got(); got != want {
				t.Errorf("%s = %t, want %t", check.name, got, want)
			}
		}
	}
}

// TestSetFilterRecreatesPairs verifies that changing a filter rebuilds the
// broad-phase pair and wakes the overlapping bodies.
func TestSetFilterRecreatesPairs(t *testing.T) {
	worldId := createTestWorld(t)

	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyA := CreateBody(worldId, &bodyDef)
	bodyB := CreateBody(worldId, &bodyDef)

	shapeDef := DefaultShapeDef()
	shapeDef.EnableContactEvents = true
	box := MakeSquare(QHalf())
	shapeDef.Filter = Filter{CategoryBits: 1, MaskBits: 1}
	shapeA := CreatePolygonShape(bodyA, &shapeDef, &box)
	shapeDef.Filter = Filter{CategoryBits: 2, MaskBits: 2}
	shapeB := CreatePolygonShape(bodyB, &shapeDef, &box)

	worldId.Step(stepDt(), 4)
	events := worldId.GetContactEvents()
	if len(events.BeginEvents) != 0 || shapeA.GetContactCapacity() != 0 || shapeB.GetContactCapacity() != 0 {
		t.Fatalf("excluded shapes produced %d begin events and capacities %d/%d", len(events.BeginEvents), shapeA.GetContactCapacity(), shapeB.GetContactCapacity())
	}

	shapeA.SetFilter(Filter{CategoryBits: 2, MaskBits: 2})
	worldId.Step(stepDt(), 4)
	events = worldId.GetContactEvents()
	if len(events.BeginEvents) != 1 {
		t.Fatalf("allowed shapes produced %d begin events, want 1", len(events.BeginEvents))
	}
	begin := events.BeginEvents[0]
	if !((begin.ShapeIdA == shapeA && begin.ShapeIdB == shapeB) || (begin.ShapeIdA == shapeB && begin.ShapeIdB == shapeA)) {
		t.Fatalf("begin event shapes = %v/%v, want the two overlapping shapes", begin.ShapeIdA, begin.ShapeIdB)
	}
	if !bodyA.IsAwake() || !bodyB.IsAwake() {
		t.Fatalf("bodies awake = %t/%t, want true/true", bodyA.IsAwake(), bodyB.IsAwake())
	}
}

func TestSetDensityUpdatesBodyMass(t *testing.T) {
	worldId := createTestWorld(t)
	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyId := CreateBody(worldId, &bodyDef)
	shapeDef := DefaultShapeDef()
	density := QMustParse("0.5")
	shapeDef.Density = density
	box := MakeSquare(QOne())
	shapeId := CreatePolygonShape(bodyId, &shapeDef, &box)

	massBefore := bodyId.GetMass()
	shapeId.SetDensity(density.Mul(QFromInt(2)), true)
	massAfter := bodyId.GetMass()
	if !massAfter.Eq(massBefore.Mul(QFromInt(2))) {
		t.Fatalf("mass after density update = %v, want %v", massAfter, massBefore.Mul(QFromInt(2)))
	}

	shapeId.SetDensity(QFromInt(3), false)
	if got := bodyId.GetMass(); !got.Eq(massAfter) {
		t.Fatalf("mass after non-updating density change = %v, want %v", got, massAfter)
	}
}

func TestSetCircleRefitsAABB(t *testing.T) {
	worldId := createTestWorld(t)
	bodyDef := DefaultBodyDef()
	bodyId := CreateBody(worldId, &bodyDef)
	shapeDef := DefaultShapeDef()
	circle := Circle{Radius: QMustParse("0.25")}
	shapeId := CreateCircleShape(bodyId, &shapeDef, &circle)
	oldAABB := shapeId.GetBounds()

	shapeId.SetCircle(&Circle{Radius: QMustParse("0.75")})
	newAABB := shapeId.GetBounds()
	if !newAABB.LowerBound.X.Less(oldAABB.LowerBound.X) ||
		!newAABB.LowerBound.Y.Less(oldAABB.LowerBound.Y) ||
		!oldAABB.UpperBound.X.Less(newAABB.UpperBound.X) ||
		!oldAABB.UpperBound.Y.Less(newAABB.UpperBound.Y) {
		t.Fatalf("AABB changed from %+v to %+v without strictly growing", oldAABB, newAABB)
	}
}

func TestRayCastHitsShape(t *testing.T) {
	worldId := createTestWorld(t)
	bodyDef := DefaultBodyDef()
	bodyId := CreateBody(worldId, &bodyDef)
	shapeDef := DefaultShapeDef()
	box := MakeSquare(QHalf())
	shapeId := CreatePolygonShape(bodyId, &shapeDef, &box)

	output := shapeId.CastRay(&RayCastInput{
		Origin:      v2(-2, 0),
		Translation: v2(4, 0),
		MaxFraction: QOne(),
	})
	if !output.Hit {
		t.Fatal("ray cast did not hit the box")
	}
	if output.Normal != (Vec2{X: QOne().Neg()}) {
		t.Fatalf("hit normal = %v, want (-1, 0)", output.Normal)
	}
	wantPoint := Vec2{X: QHalf().Neg()}
	if !withinQ(output.Point.X, wantPoint.X, qUlps(16)) ||
		!withinQ(output.Point.Y, wantPoint.Y, qUlps(16)) {
		t.Fatalf("hit point = %v, want %v", output.Point, wantPoint)
	}
}

func TestGetClosestPointOutsidePolygon(t *testing.T) {
	worldId := createTestWorld(t)
	bodyDef := DefaultBodyDef()
	bodyId := CreateBody(worldId, &bodyDef)
	shapeDef := DefaultShapeDef()
	box := MakeSquare(QHalf())
	shapeId := CreatePolygonShape(bodyId, &shapeDef, &box)

	got := shapeId.GetClosestPoint(v2(3, 0))
	want := Vec2{X: QHalf()}
	if !withinQ(got.X, want.X, qUlps(16)) || !withinQ(got.Y, want.Y, qUlps(16)) {
		t.Fatalf("closest point = %v, want %v", got, want)
	}
}

func TestTestPoint(t *testing.T) {
	worldId := createTestWorld(t)
	bodyDef := DefaultBodyDef()
	bodyDef.Position = v2(2, 3)
	bodyId := CreateBody(worldId, &bodyDef)
	shapeDef := DefaultShapeDef()
	box := MakeSquare(QHalf())
	shapeId := CreatePolygonShape(bodyId, &shapeDef, &box)

	if !shapeId.TestPoint(v2(2, 3)) {
		t.Fatal("TestPoint rejected a point inside the box")
	}
	if shapeId.TestPoint(v2(3, 3)) {
		t.Fatal("TestPoint accepted a point outside the box")
	}
}

func TestCreateChainOpenBuildsSegmentsWithGhosts(t *testing.T) {
	worldId := createTestWorld(t)
	bodyDef := DefaultBodyDef()
	bodyId := CreateBody(worldId, &bodyDef)
	// The ghosts sit at x = 0 and x = 4, so every segment sees the same
	// neighbors as one long line.
	points := []Vec2{v2(0, 0), v2(1, 0), v2(2, 0), v2(3, 0), v2(4, 0)}
	chainDef := DefaultChainDef()
	chainDef.Points = points[1:4]
	chainDef.Ghost1, chainDef.Ghost2 = points[0], points[4]
	chainId := CreateChain(bodyId, &chainDef)

	if got := chainId.GetSegmentCount(); got != 2 {
		t.Fatalf("segment count = %d, want 2", got)
	}
	segments := make([]ShapeId, chainId.GetSegmentCount())
	if got := chainId.GetSegments(segments); got != len(segments) {
		t.Fatalf("segments written = %d, want %d", got, len(segments))
	}
	for i, segmentId := range segments {
		segment := segmentId.GetChainSegment()
		if segment.Ghost1 != points[i] || segment.Ghost2 != points[i+3] {
			t.Errorf("segment %d ghosts = %v/%v, want %v/%v", i, segment.Ghost1, segment.Ghost2, points[i], points[i+3])
		}
		if got := segmentId.GetParentChain(); got != chainId {
			t.Errorf("segment %d parent chain = %v, want %v", i, got, chainId)
		}
	}
}

func TestCreateChainLoopBuildsAllSegments(t *testing.T) {
	worldId := createTestWorld(t)
	bodyDef := DefaultBodyDef()
	bodyId := CreateBody(worldId, &bodyDef)
	chainDef := DefaultChainDef()
	chainDef.Points = []Vec2{v2(0, 0), v2(2, 0), v2(2, 2), v2(0, 2)}
	chainDef.IsLoop = true
	chainId := CreateChain(bodyId, &chainDef)

	if got := chainId.GetSegmentCount(); got != 4 {
		t.Fatalf("segment count = %d, want 4", got)
	}
}

// TestCreateChainRejectsBadDefs checks the v3.2 minimums: two points and
// both ghosts for an open chain, three points for a loop, and one material
// or one per segment.
func TestCreateChainRejectsBadDefs(t *testing.T) {
	worldId := createTestWorld(t)
	bodyDef := DefaultBodyDef()
	bodyId := CreateBody(worldId, &bodyDef)

	open := func() ChainDef {
		def := DefaultChainDef()
		def.Points = []Vec2{v2(1, 0), v2(2, 0)}
		def.Ghost1, def.Ghost2 = v2(0, 0), v2(3, 0)
		return def
	}

	def := open()
	if got := CreateChain(bodyId, &def).GetSegmentCount(); got != 1 {
		t.Fatalf("two points built %d segments, want 1", got)
	}

	def = open()
	def.Points = def.Points[:1]
	requirePanic(t, func() { CreateChain(bodyId, &def) })

	def = open()
	def.Ghost1 = DefaultChainDef().Ghost1
	requirePanic(t, func() { CreateChain(bodyId, &def) })

	def = open()
	def.Ghost2 = DefaultChainDef().Ghost2
	requirePanic(t, func() { CreateChain(bodyId, &def) })

	def = open()
	def.Materials = []SurfaceMaterial{DefaultSurfaceMaterial(), DefaultSurfaceMaterial()}
	requirePanic(t, func() { CreateChain(bodyId, &def) })

	def = DefaultChainDef()
	def.Points = []Vec2{v2(0, 0), v2(1, 0)}
	def.IsLoop = true
	requirePanic(t, func() { CreateChain(bodyId, &def) })

	// A loop ignores the ghosts, so the sentinels are fine.
	def.Points = append(def.Points, v2(0, 1))
	if got := CreateChain(bodyId, &def).GetSegmentCount(); got != 3 {
		t.Fatalf("a three point loop built %d segments, want 3", got)
	}
}

// TestChainSurfaceMaterials sets the material of one segment and then of
// every segment, and checks the per-segment materials of the definition.
func TestChainSurfaceMaterials(t *testing.T) {
	worldId := createTestWorld(t)
	bodyDef := DefaultBodyDef()
	bodyId := CreateBody(worldId, &bodyDef)
	chainDef := DefaultChainDef()
	chainDef.Points = []Vec2{v2(1, 0), v2(2, 0), v2(3, 0)}
	chainDef.Ghost1, chainDef.Ghost2 = v2(0, 0), v2(4, 0)
	chainDef.Materials = []SurfaceMaterial{
		{Friction: QMustParse("0.25"), UserMaterialId: 1},
		{Friction: QMustParse("0.5"), UserMaterialId: 2},
	}
	chainId := CreateChain(bodyId, &chainDef)

	for i, want := range chainDef.Materials {
		if got := chainId.GetSurfaceMaterial(i); got != want {
			t.Errorf("segment %d material = %+v, want %+v", i, got, want)
		}
	}

	one := SurfaceMaterial{Friction: QMustParse("0.375"), UserMaterialId: 7}
	chainId.SetSurfaceMaterial(one, 1)
	if got := chainId.GetSurfaceMaterial(1); got != one {
		t.Errorf("segment 1 material = %+v, want %+v", got, one)
	}
	if got := chainId.GetSurfaceMaterial(0); got != chainDef.Materials[0] {
		t.Errorf("segment 0 changed to %+v", got)
	}

	all := SurfaceMaterial{Friction: QMustParse("0.125"), Restitution: QHalf()}
	chainId.SetAllSurfaceMaterials(all)
	segments := make([]ShapeId, chainId.GetSegmentCount())
	chainId.GetSegments(segments)
	for i, segmentId := range segments {
		if got := segmentId.GetSurfaceMaterial(); got != all {
			t.Errorf("segment %d material = %+v, want %+v", i, got, all)
		}
	}

	requirePanic(t, func() { chainId.SetSurfaceMaterial(one, 2) })
	requirePanic(t, func() { chainId.SetAllSurfaceMaterials(SurfaceMaterial{Friction: QFromInt(-1)}) })
}

// TestLooseChainSegment creates a chain segment without a parent chain,
// changes it, and destroys it. A segment owned by a chain refuses both.
func TestLooseChainSegment(t *testing.T) {
	worldId := createTestWorld(t)
	bodyDef := DefaultBodyDef()
	bodyId := CreateBody(worldId, &bodyDef)
	shapeDef := DefaultShapeDef()

	loose := ChainSegment{
		Ghost1:  v2(-1, 0),
		Segment: Segment{Point1: v2(0, 0), Point2: v2(1, 0)},
		Ghost2:  v2(2, 0),
		ChainId: 3,
	}
	shapeId := CreateChainSegmentShape(bodyId, &shapeDef, &loose)
	if got := shapeId.GetType(); got != ChainSegmentShape {
		t.Fatalf("type = %v, want a chain segment", got)
	}
	if shapeId.GetParentChain().IsValid() {
		t.Error("a loose segment has a parent chain")
	}

	moved := loose
	moved.Segment.Point2 = v2(2, 0)
	moved.Ghost2 = v2(3, 0)
	shapeId.SetChainSegment(&moved)
	if got := shapeId.GetChainSegment(); got.Segment != moved.Segment || got.ChainId != nullIndex {
		t.Errorf("chain segment = %+v, want %+v without a chain", got, moved)
	}

	degenerate := loose
	degenerate.Segment.Point2 = degenerate.Segment.Point1
	requirePanic(t, func() { CreateChainSegmentShape(bodyId, &shapeDef, &degenerate) })
	shapeId.SetChainSegment(&degenerate)
	if got := shapeId.GetChainSegment(); got.Segment != moved.Segment {
		t.Errorf("a degenerate segment replaced the geometry: %+v", got)
	}

	DestroyShape(shapeId, false)
	if shapeId.IsValid() {
		t.Error("the loose segment survived DestroyShape")
	}

	chainDef := DefaultChainDef()
	chainDef.Points = []Vec2{v2(0, 0), v2(2, 0), v2(2, 2)}
	chainDef.IsLoop = true
	chainId := CreateChain(bodyId, &chainDef)
	segments := make([]ShapeId, chainId.GetSegmentCount())
	chainId.GetSegments(segments)
	requirePanic(t, func() { DestroyShape(segments[0], false) })
	requirePanic(t, func() { segments[0].SetChainSegment(&loose) })
}

func TestDestroyChainRemovesSegmentsFromBody(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	bodyDef := DefaultBodyDef()
	bodyId := CreateBody(worldId, &bodyDef)
	chainDef := DefaultChainDef()
	chainDef.Points = []Vec2{v2(1, 0), v2(2, 0)}
	chainDef.Ghost1, chainDef.Ghost2 = v2(0, 0), v2(3, 0)
	chainId := CreateChain(bodyId, &chainDef)

	DestroyChain(chainId)
	if got := bodyId.GetShapeCount(); got != 0 {
		t.Errorf("shape count after destroy = %d, want 0", got)
	}
	if chainId.IsValid() {
		t.Error("destroyed chain is still valid")
	}
	validateWorld(w)
}

// TestApplyWindPushesAlongTheWind blows on a circle and a box at rest. The
// force is half the air density times the projected area times the squared
// speed, along the wind, and a box only feels its windward face.
func TestApplyWindPushesAlongTheWind(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	wind := v2(10, 0)

	circleId := addDynamicCircle(t, worldId, v2(0, 0))
	boxId := addDynamicBox(t, worldId, v2(5, 0))
	for _, tc := range []struct {
		bodyId BodyId
		want   float64
	}{{circleId, 61.25}, {boxId, 122.5}} {
		s := firstShape(w, tc.bodyId)
		shapeIdOf(w, s).ApplyWind(wind, QOne(), QZero(), false)
		sim := getBodySim(w, getBodyFullId(w, tc.bodyId))
		if got := QToFloat64(sim.force.X); math.Abs(got-tc.want) > 1e-3 || !sim.force.Y.Eq(QZero()) {
			t.Errorf("wind force = %v, want (%v, 0)", sim.force, tc.want)
		}
		if got := QToFloat64(sim.torque); math.Abs(got) > 1e-6 {
			t.Errorf("wind torque = %v, want 0", got)
		}
	}
}

// TestApplyWindWakesOnlyWhenAsked leaves a sleeping body alone unless wake
// is set, and ignores static bodies.
func TestApplyWindWakesOnlyWhenAsked(t *testing.T) {
	worldId := createTestWorld(t)
	w := getWorldFromId(worldId)
	bodyId := addDynamicCircle(t, worldId, v2(0, 0))
	shapeId := shapeIdOf(w, firstShape(w, bodyId))
	bodyId.SetAwake(false)

	shapeId.ApplyWind(v2(10, 0), QOne(), QZero(), false)
	if bodyId.IsAwake() {
		t.Fatal("the wind woke the body without wake")
	}

	shapeId.ApplyWind(v2(10, 0), QOne(), QZero(), true)
	if !bodyId.IsAwake() {
		t.Fatal("the wind did not wake the body")
	}
	if sim := getBodySim(w, getBodyFullId(w, bodyId)); !QZero().Less(sim.force.X) {
		t.Errorf("wind force = %v, want a push along x", sim.force)
	}

	groundDef := DefaultBodyDef()
	groundId := CreateBody(worldId, &groundDef)
	shapeDef := DefaultShapeDef()
	circle := Circle{Radius: QOne()}
	groundShape := CreateCircleShape(groundId, &shapeDef, &circle)
	groundShape.ApplyWind(v2(10, 0), QOne(), QZero(), true)
	if sim := getBodySim(w, getBodyFullId(w, groundId)); sim.force != Vec2Zero() {
		t.Errorf("a static body felt the wind: %v", sim.force)
	}
}
