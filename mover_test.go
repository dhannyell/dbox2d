package dbox2d_test

import (
	"testing"

	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/fixed"
)

// TestSolvePlanesSinglePlaneProjects checks the projection onto one rigid
// plane while preserving the tangential component.
func TestSolvePlanesSinglePlaneProjects(t *testing.T) {
	planes := []dbox2d.CollisionPlane{{
		Plane:     dbox2d.Plane{Normal: pt("0", "1")},
		PushLimit: fixed.Q32FromInt(100000),
	}}

	result := dbox2d.SolvePlanes(pt("1", "-1"), planes)
	if !nearVec(result.Translation, pt("1", "0"), tol(1, 100)) {
		t.Errorf("translation = %v, want (1, 0)", result.Translation)
	}
}

// TestSolvePlanesCornerConverges checks that repeated plane passes resolve a
// displacement aimed into two rigid constraints.
func TestSolvePlanesCornerConverges(t *testing.T) {
	planes := []dbox2d.CollisionPlane{
		{Plane: dbox2d.Plane{Normal: pt("1", "0")}, PushLimit: fixed.Q32FromInt(100000)},
		{Plane: dbox2d.Plane{Normal: pt("0", "1")}, PushLimit: fixed.Q32FromInt(100000)},
	}

	result := dbox2d.SolvePlanes(pt("-1", "-1"), planes)
	if result.IterationCount >= 20 {
		t.Errorf("iteration count = %d, want less than 20", result.IterationCount)
	}
	for i, plane := range planes {
		separation := dbox2d.PlaneSeparation(plane.Plane, result.Translation)
		if separation.Less(dbox2d.LinearSlop().Neg()) {
			t.Errorf("plane %d separation = %v, want at least %v", i, separation, dbox2d.LinearSlop().Neg())
		}
	}
}

// TestSolvePlanesLowPushLimitLetsThrough checks that a limited correction
// leaves most of a one-meter penetration unresolved.
func TestSolvePlanesLowPushLimitLetsThrough(t *testing.T) {
	planes := []dbox2d.CollisionPlane{{
		Plane:     dbox2d.Plane{Normal: pt("0", "1")},
		PushLimit: fixed.Q32MustParse("0.1"),
	}}

	result := dbox2d.SolvePlanes(pt("0", "-1"), planes)
	if !near(result.Translation.Y, fixed.Q32MustParse("-0.9"), tol(1, 10000)) {
		t.Errorf("translation y = %v, want -0.9", result.Translation.Y)
	}
}

// TestClipVectorRemovesNormalComponent checks that an active clipping plane
// removes velocity directed into its negative half-space.
func TestClipVectorRemovesNormalComponent(t *testing.T) {
	normal := pt("0", "1")
	planes := []dbox2d.CollisionPlane{{
		Plane:        dbox2d.Plane{Normal: normal},
		Push:         fixed.Q32One(),
		ClipVelocity: true,
	}}

	result := dbox2d.ClipVector(pt("1", "-2"), planes)
	if !near(result.Dot(normal), fixed.Q32Zero(), tol(1, 100000)) {
		t.Errorf("normal component = %v, want 0", result.Dot(normal))
	}
}

// TestCollideMoverAndPolygonReportsUpNormal checks the contact orientation
// for a capsule standing on a box.
func TestCollideMoverAndPolygonReportsUpNormal(t *testing.T) {
	box := dbox2d.MakeBox(fixed.Q32One(), fixed.Q32One())
	mover := dbox2d.Capsule{
		Center1: pt("0", "1.4"),
		Center2: pt("0", "2.4"),
		Radius:  fixed.Q32Half(),
	}

	result := dbox2d.CollideMoverAndPolygon(&mover, &box)
	if !result.Hit {
		t.Fatalf("mover and box do not collide")
	}
	if !nearVec(result.Plane.Normal, pt("0", "1"), sqrtTol()) {
		t.Errorf("normal = %v, want (0, 1)", result.Plane.Normal)
	}
}
