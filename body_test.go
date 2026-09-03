package dbox2d

import (
	"testing"

	"github.com/dhannyell/fixed"
)

// TestBodyAccessorsRoundTrip verifies scalar, data, and boolean accessors.
func TestBodyAccessorsRoundTrip(t *testing.T) {
	worldId := createTestWorld(t)
	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyId := CreateBody(worldId, &bodyDef)
	shapeDef := DefaultShapeDef()
	box := MakeSquare(fixed.Q32One())
	CreatePolygonShape(bodyId, &shapeDef, &box)

	checks := []struct {
		name string
		set  func()
		got  func() Q
		want Q
	}{
		{"linear velocity x", func() { bodyId.SetLinearVelocity(v2(3, -2)) }, func() Q { return bodyId.GetLinearVelocity().X }, fixed.Q32FromInt(3)},
		{"angular velocity", func() { bodyId.SetAngularVelocity(fixed.Q32Half()) }, bodyId.GetAngularVelocity, fixed.Q32Half()},
		{"linear damping", func() { bodyId.SetLinearDamping(fixed.Q32MustParse("0.25")) }, bodyId.GetLinearDamping, fixed.Q32MustParse("0.25")},
		{"angular damping", func() { bodyId.SetAngularDamping(fixed.Q32MustParse("0.5")) }, bodyId.GetAngularDamping, fixed.Q32MustParse("0.5")},
		{"gravity scale", func() { bodyId.SetGravityScale(fixed.Q32MustParse("1.5")) }, bodyId.GetGravityScale, fixed.Q32MustParse("1.5")},
		{"sleep threshold", func() { bodyId.SetSleepThreshold(fixed.Q32MustParse("0.125")) }, bodyId.GetSleepThreshold, fixed.Q32MustParse("0.125")},
	}
	for _, check := range checks {
		check.set()
		if got := check.got(); !got.Eq(check.want) {
			t.Errorf("%s = %v, want %v", check.name, got, check.want)
		}
	}

	bodyId.SetUserData("body")
	bodyId.SetName("test body")
	if bodyId.GetUserData() != "body" || bodyId.GetName() != "test body" {
		t.Errorf("user data/name = %v/%q", bodyId.GetUserData(), bodyId.GetName())
	}

	boolChecks := []struct {
		name string
		set  func(bool)
		got  func() bool
	}{
		{"awake", bodyId.SetAwake, bodyId.IsAwake},
		{"sleep enabled", bodyId.EnableSleep, bodyId.IsSleepEnabled},
		{"fixed rotation", bodyId.SetFixedRotation, bodyId.IsFixedRotation},
		{"bullet", bodyId.SetBullet, bodyId.IsBullet},
	}
	for _, check := range boolChecks {
		check.set(false)
		if check.got() {
			t.Errorf("%s stayed true after false", check.name)
		}
		check.set(true)
		if !check.got() {
			t.Errorf("%s stayed false after true", check.name)
		}
	}
}

// TestSetTransformMovesBodyAndProxy verifies that a moved proxy creates the
// contact found by the next broad-phase update.
func TestSetTransformMovesBodyAndProxy(t *testing.T) {
	worldId := createTestWorld(t)

	plateDef := DefaultBodyDef()
	plateDef.Position = v2(0, -1)
	plate := CreateBody(worldId, &plateDef)
	shapeDef := DefaultShapeDef()
	shapeDef.EnableContactEvents = true
	plateBox := MakeBox(fixed.Q32FromInt(5), fixed.Q32Half())
	plateShape := CreatePolygonShape(plate, &shapeDef, &plateBox)

	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = v2(10, 0)
	body := CreateBody(worldId, &bodyDef)
	bodyBox := MakeSquare(fixed.Q32Half())
	bodyShape := CreatePolygonShape(body, &shapeDef, &bodyBox)

	newPosition := v2(0, -1)
	body.SetTransform(newPosition, RotIdentity())
	if got := body.GetPosition(); got != newPosition {
		t.Fatalf("position = %v, want %v", got, newPosition)
	}

	worldId.Step(stepDt(), 4)
	events := worldId.GetContactEvents()
	if len(events.BeginEvents) != 1 {
		t.Fatalf("begin events = %d, want 1", len(events.BeginEvents))
	}
	begin := events.BeginEvents[0]
	if !((begin.ShapeIdA == plateShape && begin.ShapeIdB == bodyShape) || (begin.ShapeIdA == bodyShape && begin.ShapeIdB == plateShape)) {
		t.Fatalf("begin event shapes = %v/%v, want plate/body", begin.ShapeIdA, begin.ShapeIdB)
	}
}

// TestSetTargetTransformDerivesVelocity verifies target position and rotation
// become the corresponding linear and turn-rate velocities.
func TestSetTargetTransformDerivesVelocity(t *testing.T) {
	worldId := createTestWorld(t)
	body := addDynamicBox(t, worldId, Vec2Zero())
	body.SetTargetTransform(Transform{P: v2(1, 0), Q: MakeRot(fixed.Q32MustParse("0.25"))}, fixed.Q32FromRatio(1, 60))

	// (1 m - 0 m) / (1/60 s) = 60 m/s; (0.25 turn - 0 turn) / (1/60 s) = 15 turns/s.
	linearWant := v2(60, 0)
	angularWant := fixed.Q32MustParse("0.25").Mul(fixed.Q32FromInt(60))
	tolerance := fixed.Q32FromRaw(2048)
	if got := body.GetLinearVelocity(); !withinQ(got.X, linearWant.X, tolerance) || !withinQ(got.Y, linearWant.Y, tolerance) {
		t.Errorf("linear velocity = %v, want %v", got, linearWant)
	}
	if got := body.GetAngularVelocity(); !withinQ(got, angularWant, tolerance) {
		t.Errorf("angular velocity = %v, want %v turns/s", got, angularWant)
	}
}

// TestSetMassDataOverridesShapeMass verifies explicit mass properties and a
// changed local center.
func TestSetMassDataOverridesShapeMass(t *testing.T) {
	worldId := createTestWorld(t)
	body := addDynamicBox(t, worldId, Vec2Zero())
	before := body.GetLocalCenterOfMass()
	massData := MassData{
		Mass:              fixed.Q32FromInt(5),
		Center:            v2(1, 0),
		RotationalInertia: fixed.Q32FromInt(2),
	}
	body.SetMassData(massData)

	if got := body.GetMass(); !got.Eq(massData.Mass) {
		t.Errorf("mass = %v, want %v", got, massData.Mass)
	}
	if got := body.GetRotationalInertia(); !got.Eq(massData.RotationalInertia) {
		t.Errorf("rotational inertia = %v, want %v", got, massData.RotationalInertia)
	}
	if got := body.GetLocalCenterOfMass(); got == before || got != massData.Center {
		t.Errorf("local center = %v, want moved to %v from %v", got, massData.Center, before)
	}
}

// TestApplyForceOffCenterGivesTorque verifies the expected torque sign.
func TestApplyForceOffCenterGivesTorque(t *testing.T) {
	def := DefaultWorldDef()
	def.Gravity = Vec2Zero()
	worldId := CreateWorld(&def)
	t.Cleanup(func() { DestroyWorld(worldId) })
	bodyId := addDynamicBox(t, worldId, Vec2Zero())
	bodyId.ApplyForce(v2(0, 10), v2(1, 0), true)
	worldId.Step(stepDt(), 4)
	if !fixed.Q32Zero().Less(bodyId.GetAngularVelocity()) {
		t.Fatalf("angular velocity = %v, want positive", bodyId.GetAngularVelocity())
	}
}

// TestApplyAngularImpulseInTurns verifies I/I = 1 rad/s = 1/tau turns/s.
func TestApplyAngularImpulseInTurns(t *testing.T) {
	worldId := createTestWorld(t)
	bodyId := addDynamicBox(t, worldId, Vec2Zero())
	inertia := bodyId.GetRotationalInertia()
	bodyId.ApplyAngularImpulse(inertia, true)
	want := fixed.Q32One().Div(tau)
	if !withinQ(bodyId.GetAngularVelocity(), want, fixed.Q32FromRaw(4)) {
		t.Fatalf("angular velocity = %v, want %v turns/s", bodyId.GetAngularVelocity(), want)
	}
}

// TestGetLocalPointVelocityUsesTurns verifies the turns-to-radians conversion.
func TestGetLocalPointVelocityUsesTurns(t *testing.T) {
	worldId := createTestWorld(t)
	bodyId := addDynamicBox(t, worldId, Vec2Zero())
	v := v2(3, -2)
	w := fixed.Q32MustParse("0.25")
	r := v2(2, 1)
	bodyId.SetLinearVelocity(v)
	bodyId.SetAngularVelocity(w)
	want := v.Add(CrossSV(tau.Mul(w), r))
	got := bodyId.GetLocalPointVelocity(r)
	if !withinQ(got.X, want.X, fixed.Q32FromRaw(8)) || !withinQ(got.Y, want.Y, fixed.Q32FromRaw(8)) {
		t.Fatalf("local point velocity = %v, want %v", got, want)
	}
}

// TestSetFixedRotationZeroesInertia verifies the fixed-rotation state change.
func TestSetFixedRotationZeroesInertia(t *testing.T) {
	worldId := createTestWorld(t)
	bodyId := addDynamicBox(t, worldId, Vec2Zero())
	bodyId.SetAngularVelocity(fixed.Q32One())
	bodyId.SetFixedRotation(true)
	if !bodyId.IsFixedRotation() || !bodyId.GetRotationalInertia().Eq(fixed.Q32Zero()) || !bodyId.GetAngularVelocity().Eq(fixed.Q32Zero()) {
		t.Fatalf("fixed rotation state = %v, inertia = %v, angular velocity = %v", bodyId.IsFixedRotation(), bodyId.GetRotationalInertia(), bodyId.GetAngularVelocity())
	}
}

// TestGetContactDataReturnsTouchingManifold verifies a resting contact record.
func TestGetContactDataReturnsTouchingManifold(t *testing.T) {
	worldId := createTestWorld(t)
	groundDef := DefaultBodyDef()
	groundDef.Position = v2(0, -2)
	ground := CreateBody(worldId, &groundDef)
	shapeDef := DefaultShapeDef()
	groundBox := MakeBox(fixed.Q32FromInt(5), fixed.Q32Half())
	groundShape := CreatePolygonShape(ground, &shapeDef, &groundBox)
	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = v2(0, -1)
	body := CreateBody(worldId, &bodyDef)
	box := MakeSquare(fixed.Q32One())
	bodyShape := CreatePolygonShape(body, &shapeDef, &box)
	for range 5 {
		worldId.Step(stepDt(), 4)
	}
	if body.GetContactCapacity() < 1 {
		t.Fatalf("contact capacity = %d, want at least 1", body.GetContactCapacity())
	}
	data := make([]ContactData, body.GetContactCapacity())
	if count := body.GetContactData(data); count < 1 {
		t.Fatal("GetContactData returned no contacts")
	} else if data[0].Manifold.PointCount == 0 || !((data[0].ShapeIdA == groundShape && data[0].ShapeIdB == bodyShape) || (data[0].ShapeIdA == bodyShape && data[0].ShapeIdB == groundShape)) {
		t.Fatalf("contact data = %+v, want touching ground/body shapes", data[0])
	}
}

// TestGetShapesAndJointsKeepListOrder verifies the prepended newest-first lists.
func TestGetShapesAndJointsKeepListOrder(t *testing.T) {
	worldId := createTestWorld(t)
	body := addProjectile(t, worldId, false)
	initialShapes := make([]ShapeId, 1)
	body.GetShapes(initialShapes)
	shapeDef := DefaultShapeDef()
	shape := MakeSquare(fixed.Q32Half())
	firstShape := CreatePolygonShape(body, &shapeDef, &shape)
	secondShape := CreatePolygonShape(body, &shapeDef, &shape)
	otherA := addPlate(t, worldId, StaticBody, -3)
	otherB := addPlate(t, worldId, StaticBody, 3)
	jointDef := DefaultRevoluteJointDef()
	jointDef.BodyIdA, jointDef.BodyIdB = body, otherA
	firstJoint := CreateRevoluteJoint(worldId, &jointDef)
	jointDef.BodyIdB = otherB
	secondJoint := CreateRevoluteJoint(worldId, &jointDef)

	if body.GetShapeCount() != 3 || body.GetJointCount() != 2 {
		t.Fatalf("counts = %d shapes, %d joints", body.GetShapeCount(), body.GetJointCount())
	}
	shapes := make([]ShapeId, 3)
	joints := make([]JointId, 2)
	body.GetShapes(shapes)
	body.GetJoints(joints)
	if shapes[0] != secondShape || shapes[1] != firstShape || shapes[2] != initialShapes[0] {
		t.Errorf("shapes = %v, want [%v %v %v]", shapes, secondShape, firstShape, initialShapes[0])
	}
	if joints[0] != secondJoint || joints[1] != firstJoint {
		t.Errorf("joints = %v, want [%v %v]", joints, secondJoint, firstJoint)
	}
}
