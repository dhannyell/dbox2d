// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from shared/determinism.c of Box2D v3.1.1.

package shared

import (
	. "github.com/dhannyell/dbox2d"
)

// CreateFallingHinges is CreateFallingHinges. The scene is the cross platform
// determinism test of the reference: a wall of hinged rounded boxes whose
// chaotic fall engages continuous collision, joint limits and the approximate
// sine and cosine of MakeRot. It returns the bodies in creation order, which
// is the order a transform hash has to read them in.
//
// falling_hinges is not a benchmark scene, so it is absent from Specs; the
// conformance suite and the Determinism sample build it directly. Both must
// supply Options.Rotation, the radian form of b2MakeRot.
func CreateFallingHinges(worldId WorldId, opts Options) []BodyId {
	if opts.Rotation == nil {
		panic("shared: falling_hinges needs Options.Rotation")
	}

	groundDef := DefaultBodyDef()
	groundDef.Position = Vec2{Y: QFromInt(-1)}
	groundId := CreateBody(worldId, &groundDef)
	ground := MakeBox(QFromInt(20), QOne())
	shapeDef := DefaultShapeDef()
	CreatePolygonShape(groundId, &shapeDef, &ground)

	const columnCount = 4
	const rowCount = 30
	half := QMustParse("0.25")
	// The reference derives these from half as 0.1f * h and 0.4f * h. Both
	// products are exact in binary32, and the literals are the nearer Q32.32
	// value; the frozen traces of both modes were taken this way.
	radius := QMustParse("0.025")
	offset := QMustParse("0.1")
	box := MakeRoundedBox(half.Sub(radius), half.Sub(radius), radius)
	shapeDef = DefaultShapeDef()
	shapeDef.Material.Friction = QMustParse("0.3")
	dx := QFromInt(10).Mul(half)
	xroot := QHalf().Neg().Mul(dx).Mul(QFromInt(columnCount - 1))
	jointDef := DefaultRevoluteJointDef()
	jointDef.EnableLimit = true
	// Turns; the reference limits are -0.1 pi and 0.2 pi.
	jointDef.LowerAngle = QFromRatio(-1, 20)
	jointDef.UpperAngle = QFromRatio(1, 10)
	jointDef.EnableSpring = true
	jointDef.Hertz = QHalf()
	jointDef.DampingRatio = QHalf()
	jointDef.LocalAnchorA = Vec2{X: half, Y: half}
	jointDef.LocalAnchorB = Vec2{X: offset, Y: half.Neg()}
	jointDef.DrawSize = QMustParse("0.1")

	bodyIds := make([]BodyId, 0, rowCount*columnCount)
	for j := range columnCount {
		x := xroot.Add(QFromInt(j).Mul(dx))
		var previous BodyId
		for i := range rowCount {
			bodyDef := DefaultBodyDef()
			bodyDef.Type = DynamicBody
			bodyDef.Position = Vec2{X: x.Add(offset.Mul(QFromInt(i))), Y: half.Add(QFromInt(2).Mul(half).Mul(QFromInt(i)))}
			// this tests the deterministic cosine and sine functions
			bodyDef.Rotation = opts.Rotation(float32(0.1*float32(i) - 1))
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
			bodyIds = append(bodyIds, bodyId)
		}
	}
	return bodyIds
}

// BuildFallingHinges is CreateFallingHinges in the shape of a Spec builder,
// for consumers that drive the scene by step count and never read the bodies.
func BuildFallingHinges(worldId WorldId, opts Options) StepFn {
	CreateFallingHinges(worldId, opts)
	return nil
}
