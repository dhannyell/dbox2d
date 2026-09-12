// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from shared/random.h and shared/random.c of Box2D v3.1.1

package shared

import . "github.com/dhannyell/dbox2d"

// RandLimit and RandSeed are RAND_LIMIT and RAND_SEED of shared/random.h.
const RandLimit = 32767
const RandSeed uint32 = 12345

// RandomSeed is g_randomSeed of shared/random.h: one global stream shared by
// every caller, which is what makes a scene replay the same way twice. A
// second copy of this variable would silently split the stream, so the
// generator lives here with the scenes rather than in each consumer.
var RandomSeed uint32 = RandSeed

// RandomInt is RandomInt of shared/random.h: XorShift32, not the platform
// rand(), so the sequence is the same on every target.
func RandomInt() int {
	x := RandomSeed
	x ^= x << 13
	x ^= x >> 17
	x ^= x << 5
	RandomSeed = x
	return int(x % (RandLimit + 1))
}

// RandomIntRange returns a random integer in [lo, hi].
func RandomIntRange(lo, hi int) int {
	return lo + RandomInt()%(hi-lo+1)
}

// RandomFloat returns a random scalar in [-1, 1].
func RandomFloat() Q {
	r := QFromRatio(RandomInt(), RandLimit)
	return QFromInt(2).Mul(r).Sub(QOne())
}

// RandomFloatRange returns a random scalar in [lo, hi].
func RandomFloatRange(lo, hi Q) Q {
	r := QFromRatio(RandomInt(), RandLimit)
	return hi.Sub(lo).Mul(r).Add(lo)
}

// RandomVec2 returns a vector with both components in [lo, hi].
func RandomVec2(lo, hi Q) Vec2 {
	return Vec2{X: RandomFloatRange(lo, hi), Y: RandomFloatRange(lo, hi)}
}

// RandomRot returns a random rotation.
func RandomRot() Rot {
	return MakeRot(RandomFloatRange(QHalf().Neg(), QHalf()))
}

// RandomPolygon returns a random convex polygon no larger than extent.
func RandomPolygon(extent Q) Polygon {
	points := make([]Vec2, MaxPolygonVertices)
	count := 3 + RandomInt()%6
	for i := range count {
		points[i] = RandomVec2(extent.Neg(), extent)
	}

	hull := ComputeHull(points[:count])
	if hull.Count > 0 {
		return MakePolygon(&hull, QZero())
	}
	return MakeSquare(extent)
}
