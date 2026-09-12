package dbox2d_test

// The seven benchmark scenes of the reference live in internal/shared, which
// this suite shares with the scene benchmark in bench_scene_test.go. Two
// things stay here instead.
//
// falling_hinges is the first: shared/determinism.c builds it, but it is a
// conformance scene rather than a benchmark one, absent from benchmark/main.c
// and from the scene table of shared/benchmarks.c, so it joins the table here.
//
// The two reference values the public API cannot express are the second, both
// through shared.Options: the tumbler motor speed and the radian rotations of
// falling_hinges.

import (
	"github.com/dhannyell/dbox2d/internal/shared"

	. "github.com/dhannyell/dbox2d"
)

// conformanceSpecs is the benchmark scene table plus the conformance scenes
// that are not in it.
var conformanceSpecs = func() map[string]shared.Spec {
	all := make(map[string]shared.Spec, len(shared.Specs)+1)
	for name, spec := range shared.Specs {
		all[name] = spec
	}
	all["falling_hinges"] = shared.Spec{Build: shared.BuildFallingHinges}
	return all
}()

// conformanceSceneOptions carries the reference values a scene cannot build
// for itself.
//
// Rotation is b2MakeRot on the radian angle, which is what the frozen traces
// of both modes record.
//
// The tumbler motor is (B2_PI / 180.0f) * 25.0f radians per second, and in
// float mode a joint keeps radians (D-004); no binary32 turn rate times
// floatTau rounds back to that number. Fixed mode keeps turns, where
// QFromRatio(25, 360) is already the value the reference means, so the hook
// would only add a rounding and stays nil.
func conformanceSceneOptions(worldId WorldId) shared.Options {
	opts := shared.Options{Rotation: RotFromRadians}
	if ScalarMode != "float" {
		return opts
	}
	opts.MotorJoint = func(jointId JointId) {
		pi := float32(3.14159265359)
		speed := pi / 180
		speed *= 25
		SetRevoluteMotorSpeedRadians(worldId, jointId, QFromFloat64(float64(speed)))
	}
	return opts
}
