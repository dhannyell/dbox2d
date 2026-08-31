package dbox2d

import (
	"testing"

	"github.com/dhannyell/fixed"
)

// TestComputePolygonCentroidRejectsZeroArea checks the private guard before
// a malformed polygon can produce a meaningless centroid.
func TestComputePolygonCentroidRejectsZeroArea(t *testing.T) {
	vertices := []Vec2{
		{X: fixed.Zero(), Y: fixed.Zero()},
		{X: fixed.One(), Y: fixed.Zero()},
		{X: fixed.FromInt(2), Y: fixed.Zero()},
	}

	defer func() {
		if recover() == nil {
			t.Errorf("computePolygonCentroid accepts vertices with zero area")
		}
	}()
	computePolygonCentroid(vertices)
}
