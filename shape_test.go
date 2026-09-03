package dbox2d

import (
	"testing"

	"github.com/dhannyell/fixed"
)

// TestShapeAccessorsRoundTrip verifies shape data and event flag accessors.
func TestShapeAccessorsRoundTrip(t *testing.T) {
	worldId := createTestWorld(t)
	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyId := CreateBody(worldId, &bodyDef)
	shapeDef := DefaultShapeDef()
	box := MakeSquare(fixed.Q32Half())
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
			set:  func() { shapeId.SetFriction(fixed.Q32MustParse("0.25")) },
			got:  func() bool { return shapeId.GetFriction().Eq(fixed.Q32MustParse("0.25")) },
		},
		{
			name: "restitution",
			set:  func() { shapeId.SetRestitution(fixed.Q32MustParse("0.75")) },
			got:  func() bool { return shapeId.GetRestitution().Eq(fixed.Q32MustParse("0.75")) },
		},
		{
			name: "material",
			set:  func() { shapeId.SetMaterial(7) },
			got:  func() bool { return shapeId.GetMaterial() == 7 },
		},
		{
			name: "surface material",
			set: func() {
				shapeId.SetSurfaceMaterial(SurfaceMaterial{
					Friction:          fixed.Q32MustParse("0.4"),
					Restitution:       fixed.Q32MustParse("0.6"),
					RollingResistance: fixed.Q32MustParse("0.2"),
					TangentSpeed:      fixed.Q32MustParse("1.5"),
					UserMaterialId:    7,
					CustomColor:       0x12345678,
				})
			},
			got: func() bool {
				return shapeId.GetSurfaceMaterial() == (SurfaceMaterial{
					Friction:          fixed.Q32MustParse("0.4"),
					Restitution:       fixed.Q32MustParse("0.6"),
					RollingResistance: fixed.Q32MustParse("0.2"),
					TangentSpeed:      fixed.Q32MustParse("1.5"),
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
	box := MakeSquare(fixed.Q32Half())
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
	density := fixed.Q32MustParse("0.5")
	shapeDef.Density = density
	box := MakeSquare(fixed.Q32One())
	shapeId := CreatePolygonShape(bodyId, &shapeDef, &box)

	massBefore := bodyId.GetMass()
	shapeId.SetDensity(density.Mul(fixed.Q32FromInt(2)), true)
	massAfter := bodyId.GetMass()
	if !massAfter.Eq(massBefore.Mul(fixed.Q32FromInt(2))) {
		t.Fatalf("mass after density update = %v, want %v", massAfter, massBefore.Mul(fixed.Q32FromInt(2)))
	}

	shapeId.SetDensity(fixed.Q32FromInt(3), false)
	if got := bodyId.GetMass(); !got.Eq(massAfter) {
		t.Fatalf("mass after non-updating density change = %v, want %v", got, massAfter)
	}
}

func TestSetCircleRefitsAABB(t *testing.T) {
	worldId := createTestWorld(t)
	bodyDef := DefaultBodyDef()
	bodyId := CreateBody(worldId, &bodyDef)
	shapeDef := DefaultShapeDef()
	circle := Circle{Radius: fixed.Q32MustParse("0.25")}
	shapeId := CreateCircleShape(bodyId, &shapeDef, &circle)
	oldAABB := shapeId.GetAABB()

	shapeId.SetCircle(&Circle{Radius: fixed.Q32MustParse("0.75")})
	newAABB := shapeId.GetAABB()
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
	box := MakeSquare(fixed.Q32Half())
	shapeId := CreatePolygonShape(bodyId, &shapeDef, &box)

	output := shapeId.RayCast(&RayCastInput{
		Origin:      v2(-2, 0),
		Translation: v2(4, 0),
		MaxFraction: fixed.Q32One(),
	})
	if !output.Hit {
		t.Fatal("ray cast did not hit the box")
	}
	if output.Normal != (Vec2{X: fixed.Q32One().Neg()}) {
		t.Fatalf("hit normal = %v, want (-1, 0)", output.Normal)
	}
	wantPoint := Vec2{X: fixed.Q32Half().Neg()}
	if !withinQ(output.Point.X, wantPoint.X, fixed.Q32FromRaw(16)) ||
		!withinQ(output.Point.Y, wantPoint.Y, fixed.Q32FromRaw(16)) {
		t.Fatalf("hit point = %v, want %v", output.Point, wantPoint)
	}
}

func TestGetClosestPointOutsidePolygon(t *testing.T) {
	worldId := createTestWorld(t)
	bodyDef := DefaultBodyDef()
	bodyId := CreateBody(worldId, &bodyDef)
	shapeDef := DefaultShapeDef()
	box := MakeSquare(fixed.Q32Half())
	shapeId := CreatePolygonShape(bodyId, &shapeDef, &box)

	got := shapeId.GetClosestPoint(v2(3, 0))
	want := Vec2{X: fixed.Q32Half()}
	if !withinQ(got.X, want.X, fixed.Q32FromRaw(16)) || !withinQ(got.Y, want.Y, fixed.Q32FromRaw(16)) {
		t.Fatalf("closest point = %v, want %v", got, want)
	}
}

func TestTestPoint(t *testing.T) {
	worldId := createTestWorld(t)
	bodyDef := DefaultBodyDef()
	bodyDef.Position = v2(2, 3)
	bodyId := CreateBody(worldId, &bodyDef)
	shapeDef := DefaultShapeDef()
	box := MakeSquare(fixed.Q32Half())
	shapeId := CreatePolygonShape(bodyId, &shapeDef, &box)

	if !shapeId.TestPoint(v2(2, 3)) {
		t.Fatal("TestPoint rejected a point inside the box")
	}
	if shapeId.TestPoint(v2(3, 3)) {
		t.Fatal("TestPoint accepted a point outside the box")
	}
}
