package dbox2d_test

// The seven benchmark scenes of the reference live in internal/scenes, which
// this suite shares with the scene benchmark in bench_scene_test.go. Two
// things stay here instead.
//
// falling_hinges is the first: it is a conformance scene rather than a
// benchmark one, absent from benchmark/main.c and from the scene table of
// shared/benchmarks.c, and its body rotations come from a radian angle that
// only the library's internal constructor can express.
//
// The tumbler motor is the second, through scenes.Options.

import (
	"github.com/dhannyell/dbox2d/internal/scenes"

	. "github.com/dhannyell/dbox2d"
)

// conformanceSpecs is the benchmark scene table plus the conformance scenes
// that are not in it.
var conformanceSpecs = func() map[string]scenes.Spec {
	all := make(map[string]scenes.Spec, len(scenes.Specs)+1)
	for name, spec := range scenes.Specs {
		all[name] = spec
	}
	all["falling_hinges"] = scenes.Spec{Build: buildFallingHinges}
	return all
}()

// conformanceSceneOptions carries the one value a scene cannot build for
// itself. The reference sets the tumbler motor to (B2_PI / 180.0f) * 25.0f
// radians per second, and in float mode a joint keeps radians (D-004); no
// binary32 turn rate times floatTau rounds back to that number. Fixed mode
// keeps turns, where QFromRatio(25, 360) is already the value the reference
// means, so the hook does nothing there.
func conformanceSceneOptions(worldId WorldId) scenes.Options {
	if ScalarMode != "float" {
		return scenes.Options{}
	}
	return scenes.Options{MotorJoint: func(jointId JointId) {
		pi := float32(3.14159265359)
		speed := pi / 180
		speed *= 25
		SetRevoluteMotorSpeedRadians(worldId, jointId, QFromFloat64(float64(speed)))
	}}
}

func buildFallingHinges(worldId WorldId, _ scenes.Options) scenes.StepFn {
	groundDef := DefaultBodyDef()
	groundDef.Position = Vec2{Y: QFromInt(-1)}
	groundId := CreateBody(worldId, &groundDef)
	ground := MakeBox(QFromInt(20), QOne())
	shapeDef := DefaultShapeDef()
	CreatePolygonShape(groundId, &shapeDef, &ground)

	const columnCount = 4
	const rowCount = 30
	half := QMustParse("0.25")
	radius := QMustParse("0.025")
	box := MakeRoundedBox(half.Sub(radius), half.Sub(radius), radius)
	shapeDef = DefaultShapeDef()
	shapeDef.Material.Friction = QMustParse("0.3")
	offset := QMustParse("0.1")
	dx := QFromInt(10).Mul(half)
	xroot := QHalf().Neg().Mul(dx).Mul(QFromInt(columnCount - 1))
	jointDef := DefaultRevoluteJointDef()
	jointDef.EnableLimit = true
	jointDef.LowerAngle = QFromRatio(-1, 20)
	jointDef.UpperAngle = QFromRatio(1, 10)
	jointDef.EnableSpring = true
	jointDef.Hertz = QHalf()
	jointDef.DampingRatio = QHalf()
	jointDef.LocalAnchorA = Vec2{X: half, Y: half}
	jointDef.LocalAnchorB = Vec2{X: offset, Y: half.Neg()}
	jointDef.DrawSize = QMustParse("0.1")

	for j := range columnCount {
		x := xroot.Add(QFromInt(j).Mul(dx))
		var previous BodyId
		for i := range rowCount {
			bodyDef := DefaultBodyDef()
			bodyDef.Type = DynamicBody
			bodyDef.Position = Vec2{X: x.Add(offset.Mul(QFromInt(i))), Y: half.Add(QFromInt(2).Mul(half).Mul(QFromInt(i)))}
			radians := float32(0.1*float32(i) - 1)
			bodyDef.Rotation = RotFromRadians(radians)
			bodyId := CreateBody(worldId, &bodyDef)
			if i&1 == 0 {
				previous = bodyId
			} else {
				jointDef.BodyIdA = previous
				jointDef.BodyIdB = bodyId
				CreateRevoluteJoint(worldId, &jointDef)
				previous = BodyId{}
			}
			CreatePolygonShape(bodyId, &shapeDef, &box)
		}
	}
	return nil
}
