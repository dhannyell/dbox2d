//go:build !dbox2d_fixed

package b2_test

import (
	"bytes"
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"testing"

	. "github.com/dhannyell/dbox2d"
)

// The golden files come from tools/recordgolden, built against Box2D v3.2.0
// with the single or the double precision build. The scene below is the same
// scene, call for call.

const goldenSteps = 60

// goldenName returns the file prefix of the build.
func goldenName() string {
	if IsDoublePrecision() {
		return "golden_double"
	}
	return "golden"
}

func goldenPath(suffix string) string {
	return filepath.Join("testdata", "recording", goldenName()+suffix)
}

func readGolden(t *testing.T, suffix string) []byte {
	t.Helper()
	data, err := os.ReadFile(goldenPath(suffix))
	if err != nil {
		t.Fatal(err)
	}
	return data
}

func gq(num, den int) Q { return QFromRatio(num, den) }

func gv(x, y Q) Vec2 { return Vec2{X: x, Y: y} }

func goldenBody(worldId WorldId, bodyType BodyType, x, y Q) BodyId {
	def := DefaultBodyDef()
	def.Type = bodyType
	def.Position = ToPos(gv(x, y))
	return CreateBody(worldId, &def)
}

// goldenScene is the scene of tools/recordgolden/main.c.
type goldenScene struct {
	worldId                                  WorldId
	ground, box1, box2, box3                 BodyId
	circleBody, capsuleBody, pendulum, extra BodyId
	circleShape                              ShapeId
	distanceJoint, weldJoint                 JointId

	overlapCount, rayCount int
}

func newGoldenScene() *goldenScene {
	ResetWorldGenerations()
	s := &goldenScene{}
	worldDef := DefaultWorldDef()
	s.worldId = CreateWorld(&worldDef)
	worldId := s.worldId

	shapeDef := DefaultShapeDef()

	s.ground = goldenBody(worldId, StaticBody, QZero(), gq(-1, 1))
	groundBox := MakeBox(gq(20, 1), gq(1, 1))
	CreatePolygonShape(s.ground, &shapeDef, &groundBox)

	box := MakeBox(gq(1, 2), gq(1, 2))
	s.box1 = goldenBody(worldId, DynamicBody, QZero(), gq(1, 2))
	s.box2 = goldenBody(worldId, DynamicBody, gq(1, 4), gq(16, 10))
	s.box3 = goldenBody(worldId, DynamicBody, gq(-1, 4), gq(27, 10))
	CreatePolygonShape(s.box1, &shapeDef, &box)
	CreatePolygonShape(s.box2, &shapeDef, &box)
	CreatePolygonShape(s.box3, &shapeDef, &box)

	s.circleBody = goldenBody(worldId, DynamicBody, gq(2, 1), gq(4, 1))
	circleDef := DefaultShapeDef()
	circleDef.Material.Restitution = gq(1, 2)
	circle := Circle{Radius: gq(1, 2)}
	s.circleShape = CreateCircleShape(s.circleBody, &circleDef, &circle)

	s.capsuleBody = goldenBody(worldId, DynamicBody, gq(-3, 1), gq(3, 1))
	capsule := Capsule{Center1: gv(gq(-1, 2), QZero()), Center2: gv(gq(1, 2), QZero()), Radius: gq(1, 4)}
	CreateCapsuleShape(s.capsuleBody, &shapeDef, &capsule)

	s.pendulum = goldenBody(worldId, DynamicBody, gq(5, 1), gq(5, 1))
	small := MakeBox(gq(1, 4), gq(1, 4))
	CreatePolygonShape(s.pendulum, &shapeDef, &small)
	revoluteDef := DefaultRevoluteJointDef()
	revoluteDef.Base.BodyIdA = s.ground
	revoluteDef.Base.BodyIdB = s.pendulum
	revoluteDef.Base.LocalFrameA.P = gv(gq(5, 1), gq(7, 1))
	revoluteDef.Base.LocalFrameB.P = gv(QZero(), gq(1, 1))
	CreateRevoluteJoint(worldId, &revoluteDef)

	distanceDef := DefaultDistanceJointDef()
	distanceDef.Base.BodyIdA = s.ground
	distanceDef.Base.BodyIdB = s.circleBody
	distanceDef.Base.LocalFrameA.P = gv(gq(3, 1), gq(7, 1))
	distanceDef.Length = gq(2, 1)
	distanceDef.EnableSpring = true
	distanceDef.Hertz = gq(2, 1)
	distanceDef.DampingRatio = gq(1, 2)
	s.distanceJoint = CreateDistanceJoint(worldId, &distanceDef)

	weldDef := DefaultWeldJointDef()
	weldDef.Base.BodyIdA = s.box1
	weldDef.Base.BodyIdB = s.box2
	weldDef.Base.LocalFrameA.P = gv(gq(1, 4), gq(55, 100))
	weldDef.Base.LocalFrameB.P = gv(QZero(), gq(-55, 100))
	weldDef.LinearHertz = gq(5, 1)
	weldDef.LinearDampingRatio = gq(7, 10)
	s.weldJoint = CreateWeldJoint(worldId, &weldDef)
	return s
}

// run steps the scene with the API calls of the C tool between the steps.
func (s *goldenScene) run(steps int) {
	worldId := s.worldId
	shapeDef := DefaultShapeDef()
	dt := QOne().Div(QFromInt(60))
	for step := range steps {
		switch step {
		case 5:
			worldId.SetGravity(gv(QZero(), gq(-98, 10)))
			s.circleBody.SetLinearVelocity(gv(gq(1, 1), QZero()))
		case 10:
			s.extra = goldenBody(worldId, DynamicBody, gq(-1, 1), gq(5, 1))
			tiny := Circle{Radius: gq(1, 4)}
			CreateCircleShape(s.extra, &shapeDef, &tiny)
		case 12:
			s.extra.SetName("extra")
		case 15:
			s.box3.ApplyLinearImpulseToCenter(gv(gq(2, 1), QZero()), true)
			s.box1.ApplyForceToCenter(gv(gq(10, 1), QZero()), true)
		case 20:
			filter := DefaultQueryFilter()
			worldId.CastRayClosest(ToPos(gv(gq(-10, 1), gq(3, 1))), gv(gq(20, 1), QZero()), filter)
			aabb := AABB{LowerBound: gv(gq(-5, 1), QZero()), UpperBound: gv(gq(5, 1), gq(5, 1))}
			worldId.OverlapAABB(PosZero(), aabb, filter, func(ShapeId) bool {
				s.overlapCount++
				return true
			})
			worldId.CastRay(ToPos(gv(gq(-10, 1), gq(1, 1))), gv(gq(20, 1), QZero()), filter,
				func(_ ShapeId, _ Pos, _ Vec2, fraction Q) Q {
					s.rayCount++
					return fraction
				})
		case 25:
			s.circleShape.SetRestitution(gq(1, 4))
		case 30:
			s.distanceJoint.SetSpringHertz(gq(3, 1))
		case 35:
			worldId.EnableSleeping(false)
		case 40:
			DestroyJoint(s.weldJoint)
		case 45:
			DestroyBody(s.extra)
		}
		worldId.Step(dt, 4)
	}
}

func (s *goldenScene) snapshot() []byte {
	n := s.worldId.GetSnapshot(nil)
	image := make([]byte, n)
	s.worldId.GetSnapshot(image)
	return image
}

// goldenText prints the final state the way the C tool does.
func (s *goldenScene) text() string {
	var b strings.Builder
	bodies := []BodyId{s.ground, s.box1, s.box2, s.box3, s.circleBody, s.capsuleBody, s.pendulum}
	for i, id := range bodies {
		t := id.GetTransform()
		x, y := PosBits(t.P)
		fmt.Fprintf(&b, "body %d %s %s %08x %08x\n", i, hexBits(x), hexBits(y),
			qBits(t.Q.Cos), qBits(t.Q.Sin))
	}
	return b.String()
}

func hexBits(v uint64) string {
	if IsDoublePrecision() {
		return fmt.Sprintf("%016x", v)
	}
	return fmt.Sprintf("%08x", v)
}

// maskArena zeroes the arenaByteCount of the world config: it is the high
// water mark of the C stack allocator, which the port does not share (D-022).
func maskArena(image []byte) []byte { return maskArenaAt(image, 0) }

func maskArenaAt(image []byte, offset int) []byte {
	out := bytes.Clone(image)
	clear(out[offset+100 : offset+104])
	return out
}

func firstDifference(a, b []byte) int {
	for i := 0; i < min(len(a), len(b)); i++ {
		if a[i] != b[i] {
			return i
		}
	}
	if len(a) != len(b) {
		return min(len(a), len(b))
	}
	return -1
}

func TestGoldenSnapshotMatchesUpstream(t *testing.T) {
	s := newGoldenScene()
	defer DestroyWorld(s.worldId)
	s.run(goldenSteps)

	got := maskArena(s.snapshot())
	want := maskArena(readGolden(t, ".snapshot"))
	if os.Getenv("GOLDEN_DIFF") != "" {
		n := 0
		for i := range min(len(got), len(want)) {
			if got[i] != want[i] && n < 60 {
				t.Logf("byte %d: got %02x want %02x", i, got[i], want[i])
				n++
			}
		}
	}
	if i := firstDifference(got, want); i >= 0 {
		t.Fatalf("the snapshot differs from the upstream snapshot at byte %d (lengths %d and %d)", i, len(got), len(want))
	}
}

func TestGoldenRecordingMatchesUpstream(t *testing.T) {
	s := newGoldenScene()
	defer DestroyWorld(s.worldId)

	recording := CreateRecording(0)
	defer recording.Destroy()
	s.worldId.StartRecording(recording)
	s.run(goldenSteps)
	s.worldId.StopRecording()

	got := recording.Data()
	want := readGolden(t, ".b2rec")
	if os.Getenv("GOLDEN_DIFF") != "" {
		n := 0
		for i := range min(len(got), len(want)) {
			if got[i] != want[i] && n < 60 {
				t.Logf("byte %d: got %02x want %02x", i, got[i], want[i])
				n++
			}
		}
	}
	// The snapshot blob follows the 32 byte header.
	got, want = maskArenaAt(got, 32), maskArenaAt(want, 32)
	if i := firstDifference(got, want); i >= 0 {
		t.Fatalf("the recording differs from the upstream recording at byte %d (lengths %d and %d)", i, len(got), len(want))
	}
}
