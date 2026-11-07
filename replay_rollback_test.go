package b2_test

import (
	"testing"

	. "github.com/dhannyell/dbox2d"
)

func rq(num, den int) Q { return QFromRatio(num, den) }

// rollbackScene is a small scene with bodies, a joint and contacts that runs
// in every scalar mode.
func rollbackScene() WorldId {
	worldDef := DefaultWorldDef()
	worldId := CreateWorld(&worldDef)

	bodyDef := DefaultBodyDef()
	bodyDef.Position = ToPos(Vec2{X: QZero(), Y: rq(-1, 1)})
	ground := CreateBody(worldId, &bodyDef)
	shapeDef := DefaultShapeDef()
	groundBox := MakeBox(rq(20, 1), rq(1, 1))
	CreatePolygonShape(ground, &shapeDef, &groundBox)

	box := MakeBox(rq(1, 2), rq(1, 2))
	var last BodyId
	for i := range 5 {
		bodyDef.Type = DynamicBody
		bodyDef.Position = ToPos(Vec2{X: rq(i, 8), Y: rq(1, 2).Add(QFromInt(i))})
		last = CreateBody(worldId, &bodyDef)
		CreatePolygonShape(last, &shapeDef, &box)
	}

	bodyDef.Position = ToPos(Vec2{X: rq(3, 1), Y: rq(4, 1)})
	ball := CreateBody(worldId, &bodyDef)
	circle := Circle{Radius: rq(1, 2)}
	CreateCircleShape(ball, &shapeDef, &circle)

	weldDef := DefaultWeldJointDef()
	weldDef.Base.BodyIdA = last
	weldDef.Base.BodyIdB = ball
	weldDef.Base.LocalFrameA.P = Vec2{X: rq(1, 2), Y: QZero()}
	weldDef.Base.LocalFrameB.P = Vec2{X: rq(-1, 2), Y: QZero()}
	CreateWeldJoint(worldId, &weldDef)
	return worldId
}

func stepN(worldId WorldId, n int) {
	dt := QOne().Div(QFromInt(60))
	for range n {
		worldId.Step(dt, 4)
	}
}

// TestRollback takes a snapshot, steps, restores the snapshot and steps the
// same count again: the checksum must be identical.
func TestRollback(t *testing.T) {
	worldId := rollbackScene()
	defer DestroyWorld(worldId)
	stepN(worldId, 20)

	image := make([]byte, worldId.GetSnapshot(nil))
	worldId.GetSnapshot(image)

	stepN(worldId, 40)
	first := Checksum(worldId)

	if !worldId.RestoreFromSnapshot(image) {
		t.Fatal("RestoreFromSnapshot failed")
	}
	stepN(worldId, 40)
	if second := Checksum(worldId); second != first {
		t.Fatalf("rollback checksum %x, want %x", second, first)
	}

	// A fresh world made from the same bytes continues the same way.
	other := CreateWorldFromSnapshot(image, 1)
	if !other.IsValid() {
		t.Fatal("CreateWorldFromSnapshot failed")
	}
	defer DestroyWorld(other)
	stepN(other, 40)
	if third := Checksum(other); third != first {
		t.Fatalf("restored world checksum %x, want %x", third, first)
	}
}

// TestSnapshotRoundTripIsStable checks that a restored world snapshots to
// the same size and checksum.
func TestSnapshotRoundTripIsStable(t *testing.T) {
	worldId := rollbackScene()
	defer DestroyWorld(worldId)
	stepN(worldId, 15)

	image := make([]byte, worldId.GetSnapshot(nil))
	worldId.GetSnapshot(image)

	other := CreateWorldFromSnapshot(image, 1)
	defer DestroyWorld(other)
	again := make([]byte, other.GetSnapshot(nil))
	other.GetSnapshot(again)
	if len(again) != len(image) {
		t.Fatalf("snapshot size %d, want %d", len(again), len(image))
	}
	if Checksum(other) != Checksum(worldId) {
		t.Fatal("the restored world has another checksum")
	}
}

// TestRecordReplayRoundTrip records the scene with changes between steps and
// replays it: no divergence, same final checksum.
func TestRecordReplayRoundTrip(t *testing.T) {
	worldId := rollbackScene()
	defer DestroyWorld(worldId)
	stepN(worldId, 5)

	recording := CreateRecording(0)
	defer recording.Destroy()
	worldId.StartRecording(recording)

	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = ToPos(Vec2{X: rq(-2, 1), Y: rq(6, 1)})
	extra := CreateBody(worldId, &bodyDef)
	shapeDef := DefaultShapeDef()
	small := Circle{Radius: rq(1, 4)}
	CreateCircleShape(extra, &shapeDef, &small)
	stepN(worldId, 10)
	worldId.SetGravity(Vec2{X: rq(1, 1), Y: rq(-5, 1)})
	extra.ApplyLinearImpulseToCenter(Vec2{X: rq(1, 1), Y: QZero()}, true)
	filter := DefaultQueryFilter()
	worldId.CastRayClosest(ToPos(Vec2{X: rq(-10, 1), Y: rq(1, 1)}), Vec2{X: rq(20, 1), Y: QZero()}, filter)
	stepN(worldId, 20)
	DestroyBody(extra)
	stepN(worldId, 10)
	worldId.StopRecording()

	data := recording.Data()
	if !ValidateRecording(data, 1) {
		t.Fatal("the recording of this build does not validate")
	}

	p := CreateReplay(data, 1)
	if p == nil {
		t.Fatal("CreateReplay failed")
	}
	defer p.Destroy()
	for p.StepFrame() {
	}
	if p.HasDiverged() || p.GetFrame() != 40 {
		t.Fatalf("frame %d, diverged %v", p.GetFrame(), p.HasDiverged())
	}
	if got, want := Checksum(p.GetWorldId()), Checksum(worldId); got != want {
		t.Fatalf("replay checksum %x, want %x", got, want)
	}
}

// TestReplayDrawFrameQueries checks that the ray of a recorded query reaches
// the debug draw on its frame.
func TestReplayDrawFrameQueries(t *testing.T) {
	worldId := rollbackScene()
	defer DestroyWorld(worldId)
	recording := CreateRecording(0)
	defer recording.Destroy()
	worldId.StartRecording(recording)
	stepN(worldId, 2)
	worldId.CastRayClosest(ToPos(Vec2{X: rq(-10, 1), Y: rq(1, 1)}), Vec2{X: rq(20, 1), Y: QZero()}, DefaultQueryFilter())
	stepN(worldId, 2)
	worldId.StopRecording()

	p := CreateReplay(recording.Data(), 1)
	if p == nil {
		t.Fatal("CreateReplay failed")
	}
	defer p.Destroy()

	lines := 0
	draw := DebugDraw{DrawLineFcn: func(Pos, Pos, HexColor) { lines++ }, DrawPointFcn: func(Pos, Q, HexColor) {}}
	for p.StepFrame() {
		if p.GetFrameQueryCount() > 0 {
			p.DrawFrameQueries(&draw, -1)
		}
	}
	if lines == 0 {
		t.Fatal("the recorded ray was not drawn")
	}
}

// TestRecordReplayAngleOps replays every op that carries an angle. The
// recording stores the unit of the sims and the replay writes it back, so the
// replayed world matches the live one bit for bit (D-025).
func TestRecordReplayAngleOps(t *testing.T) {
	worldDef := DefaultWorldDef()
	worldId := CreateWorld(&worldDef)
	defer DestroyWorld(worldId)

	bodyDef := DefaultBodyDef()
	ground := CreateBody(worldId, &bodyDef)
	shapeDef := DefaultShapeDef()
	box := MakeBox(rq(1, 2), rq(1, 2))
	bodies := make([]BodyId, 4)
	for i := range bodies {
		bodyDef.Type = DynamicBody
		bodyDef.Position = ToPos(Vec2{X: QFromInt(3 * i), Y: rq(5, 1)})
		bodies[i] = CreateBody(worldId, &bodyDef)
		CreatePolygonShape(bodies[i], &shapeDef, &box)
	}

	recording := CreateRecording(0)
	defer recording.Destroy()
	worldId.StartRecording(recording)

	revoluteDef := DefaultRevoluteJointDef()
	revoluteDef.Base.BodyIdA = ground
	revoluteDef.Base.BodyIdB = bodies[0]
	revoluteDef.Base.LocalFrameA.P = Vec2{X: QZero(), Y: rq(5, 1)}
	revolute := CreateRevoluteJoint(worldId, &revoluteDef)
	wheelDef := DefaultWheelJointDef()
	wheelDef.Base.BodyIdA = ground
	wheelDef.Base.BodyIdB = bodies[1]
	wheelDef.Base.LocalFrameA.P = Vec2{X: rq(3, 1), Y: rq(5, 1)}
	wheel := CreateWheelJoint(worldId, &wheelDef)
	motorDef := DefaultMotorJointDef()
	motorDef.Base.BodyIdA = ground
	motorDef.Base.BodyIdB = bodies[2]
	motor := CreateMotorJoint(worldId, &motorDef)

	stepN(worldId, 3)
	bodies[3].SetAngularVelocity(rq(3, 7))
	revolute.SetLimits(rq(-1, 7), rq(1, 9))
	revolute.EnableLimit(true)
	revolute.SetTargetAngle(rq(1, 11))
	revolute.EnableMotor(true)
	revolute.SetMaxMotorTorque(rq(5, 1))
	revolute.SetTargetMotorSpeed(rq(2, 3))
	wheel.EnableMotor(true)
	wheel.SetMaxMotorTorque(rq(5, 1))
	wheel.SetTargetMotorSpeed(rq(-5, 13))
	motor.SetAngularVelocity(rq(1, 3))
	stepN(worldId, 30)
	worldId.StopRecording()

	p := CreateReplay(recording.Data(), 1)
	if p == nil {
		t.Fatal("CreateReplay failed")
	}
	defer p.Destroy()
	for p.StepFrame() {
	}
	if p.HasDiverged() {
		t.Fatalf("diverged at frame %d", p.GetDivergeFrame())
	}
	if got, want := Checksum(p.GetWorldId()), Checksum(worldId); got != want {
		t.Fatalf("replay checksum %x, want %x", got, want)
	}
}
