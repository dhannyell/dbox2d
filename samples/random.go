// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from shared/random.h and shared/random.c of Box2D v3.1.1

package samples

import (
	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/fixed"
)

const randLimit = 32767
const randSeed uint32 = 12345

var randomSeed uint32 = randSeed

func randomInt() int {
	x := randomSeed
	x ^= x << 13
	x ^= x >> 17
	x ^= x << 5
	randomSeed = x
	return int(x % (randLimit + 1))
}

func randomIntRange(lo, hi int) int {
	return lo + randomInt()%(hi-lo+1)
}

func randomFloat() dbox2d.Q {
	r := fixed.Q32FromRatio(randomInt(), randLimit)
	return fixed.Q32FromInt(2).Mul(r).Sub(fixed.Q32One())
}

func randomFloatRange(lo, hi dbox2d.Q) dbox2d.Q {
	r := fixed.Q32FromRatio(randomInt(), randLimit)
	return hi.Sub(lo).Mul(r).Add(lo)
}

func randomVec2(lo, hi dbox2d.Q) dbox2d.Vec2 {
	return dbox2d.Vec2{X: randomFloatRange(lo, hi), Y: randomFloatRange(lo, hi)}
}

func randomRot() dbox2d.Rot {
	return dbox2d.MakeRot(randomFloatRange(fixed.Q32Half().Neg(), fixed.Q32Half()))
}

func randomPolygon(extent dbox2d.Q) dbox2d.Polygon {
	points := make([]dbox2d.Vec2, dbox2d.MaxPolygonVertices)
	count := 3 + randomInt()%6
	for i := range count {
		points[i] = randomVec2(extent.Neg(), extent)
	}

	hull := dbox2d.ComputeHull(points[:count])
	if hull.Count > 0 {
		return dbox2d.MakePolygon(&hull, fixed.Q32Zero())
	}
	return dbox2d.MakeSquare(extent)
}
