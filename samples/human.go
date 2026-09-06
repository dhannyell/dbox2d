// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from shared/human.h and shared/human.c of Box2D v3.1.1

package samples

import (
	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/fixed"
)

type boneId int

const (
	boneHip boneId = iota
	boneTorso
	boneHead
	boneUpperLeftLeg
	boneLowerLeftLeg
	boneUpperRightLeg
	boneLowerRightLeg
	boneUpperLeftArm
	boneLowerLeftArm
	boneUpperRightArm
	boneLowerRightArm
	boneCount
)

type bone struct {
	bodyId        dbox2d.BodyId
	jointId       dbox2d.JointId
	frictionScale dbox2d.Q
	parentIndex   int
}

type human struct {
	bones          [boneCount]bone
	frictionTorque dbox2d.Q
	originalScale  dbox2d.Q
	scale          dbox2d.Q
	isSpawned      bool
}

func createHuman(worldId dbox2d.WorldId, position dbox2d.Vec2, scale, frictionTorque, hertz, dampingRatio dbox2d.Q, groupIndex int, userData any, colorize bool) human {
	h := human{
		frictionTorque: frictionTorque,
		originalScale:  scale,
		scale:          scale,
	}

	for i := range h.bones {
		h.bones[i].bodyId = dbox2d.BodyId{}
		h.bones[i].jointId = dbox2d.JointId{}
		h.bones[i].frictionScale = fixed.Q32One()
		h.bones[i].parentIndex = -1
	}

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	bodyDef.SleepThreshold = fixed.Q32MustParse("0.1")
	bodyDef.UserData = userData

	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Material.Friction = fixed.Q32MustParse("0.2")
	shapeDef.Filter.GroupIndex = -groupIndex
	shapeDef.Filter.CategoryBits = 2
	shapeDef.Filter.MaskBits = 1 | 2

	footShapeDef := shapeDef
	footShapeDef.Material.Friction = fixed.Q32MustParse("0.05")
	footShapeDef.Filter.CategoryBits = 2
	footShapeDef.Filter.MaskBits = 1

	if colorize {
		footShapeDef.Material.CustomColor = uint32(dbox2d.ColorSaddleBrown)
	}

	s := scale
	maxTorque := frictionTorque.Mul(s)
	enableMotor := true
	enableLimit := true
	drawSize := fixed.Q32MustParse("0.05")

	shirtColor := dbox2d.ColorMediumTurquoise
	pantColor := dbox2d.ColorDodgerBlue
	skinColors := [4]dbox2d.HexColor{
		dbox2d.ColorNavajoWhite,
		dbox2d.ColorLightYellow,
		dbox2d.ColorPeru,
		dbox2d.ColorTan,
	}
	skinColor := skinColors[groupIndex%4]

	{
		bone := &h.bones[boneHip]
		bone.parentIndex = -1

		bodyDef.Position = dbox2d.Vec2{Y: fixed.Q32MustParse("0.95").Mul(s)}.Add(position)
		bodyDef.LinearDamping = fixed.Q32Zero()
		bodyDef.Name = "hip"

		bone.bodyId = dbox2d.CreateBody(worldId, &bodyDef)

		if colorize {
			shapeDef.Material.CustomColor = uint32(pantColor)
		}

		capsule := dbox2d.Capsule{
			Center1: dbox2d.Vec2{Y: fixed.Q32MustParse("-0.02").Mul(s)},
			Center2: dbox2d.Vec2{Y: fixed.Q32MustParse("0.02").Mul(s)},
			Radius:  fixed.Q32MustParse("0.095").Mul(s),
		}
		dbox2d.CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule)
	}

	{
		bone := &h.bones[boneTorso]
		bone.parentIndex = int(boneHip)

		bodyDef.Position = dbox2d.Vec2{Y: fixed.Q32MustParse("1.2").Mul(s)}.Add(position)
		bodyDef.LinearDamping = fixed.Q32Zero()
		bodyDef.Name = "torso"

		bone.bodyId = dbox2d.CreateBody(worldId, &bodyDef)
		bone.frictionScale = fixed.Q32Half()
		bodyDef.Type = dbox2d.DynamicBody

		if colorize {
			shapeDef.Material.CustomColor = uint32(shirtColor)
		}

		capsule := dbox2d.Capsule{
			Center1: dbox2d.Vec2{Y: fixed.Q32MustParse("-0.135").Mul(s)},
			Center2: dbox2d.Vec2{Y: fixed.Q32MustParse("0.135").Mul(s)},
			Radius:  fixed.Q32MustParse("0.09").Mul(s),
		}
		dbox2d.CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule)

		pivot := dbox2d.Vec2{Y: s}.Add(position)
		jointDef := dbox2d.DefaultRevoluteJointDef()
		jointDef.BodyIdA = h.bones[bone.parentIndex].bodyId
		jointDef.BodyIdB = bone.bodyId
		jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
		jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
		jointDef.EnableLimit = enableLimit
		// Joint angles use turns; the reference values are multiples of pi.
		jointDef.LowerAngle = fixed.Q32FromRatio(-1, 8)
		jointDef.UpperAngle = fixed.Q32Zero()
		jointDef.EnableMotor = enableMotor
		jointDef.MaxMotorTorque = bone.frictionScale.Mul(maxTorque)
		jointDef.EnableSpring = hertz.Greater(fixed.Q32Zero())
		jointDef.Hertz = hertz
		jointDef.DampingRatio = dampingRatio
		jointDef.DrawSize = drawSize

		bone.jointId = dbox2d.CreateRevoluteJoint(worldId, &jointDef)
	}

	{
		bone := &h.bones[boneHead]
		bone.parentIndex = int(boneTorso)

		bodyDef.Position = dbox2d.Vec2{Y: fixed.Q32MustParse("1.475").Mul(s)}.Add(position)
		bodyDef.LinearDamping = fixed.Q32MustParse("0.1")
		bodyDef.Name = "head"

		bone.bodyId = dbox2d.CreateBody(worldId, &bodyDef)
		bone.frictionScale = fixed.Q32FromRatio(1, 4)

		if colorize {
			shapeDef.Material.CustomColor = uint32(skinColor)
		}

		capsule := dbox2d.Capsule{
			Center1: dbox2d.Vec2{Y: fixed.Q32MustParse("-0.038").Mul(s)},
			Center2: dbox2d.Vec2{Y: fixed.Q32MustParse("0.039").Mul(s)},
			Radius:  fixed.Q32MustParse("0.075").Mul(s),
		}
		dbox2d.CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule)

		pivot := dbox2d.Vec2{Y: fixed.Q32MustParse("1.4").Mul(s)}.Add(position)
		jointDef := dbox2d.DefaultRevoluteJointDef()
		jointDef.BodyIdA = h.bones[bone.parentIndex].bodyId
		jointDef.BodyIdB = bone.bodyId
		jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
		jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
		jointDef.EnableLimit = enableLimit
		jointDef.LowerAngle = fixed.Q32FromRatio(-3, 20)
		jointDef.UpperAngle = fixed.Q32FromRatio(1, 20)
		jointDef.EnableMotor = enableMotor
		jointDef.MaxMotorTorque = bone.frictionScale.Mul(maxTorque)
		jointDef.EnableSpring = hertz.Greater(fixed.Q32Zero())
		jointDef.Hertz = hertz
		jointDef.DampingRatio = dampingRatio
		jointDef.DrawSize = drawSize

		bone.jointId = dbox2d.CreateRevoluteJoint(worldId, &jointDef)
	}

	{
		bone := &h.bones[boneUpperLeftLeg]
		bone.parentIndex = int(boneHip)

		bodyDef.Position = dbox2d.Vec2{Y: fixed.Q32MustParse("0.775").Mul(s)}.Add(position)
		bodyDef.LinearDamping = fixed.Q32Zero()
		bodyDef.Name = "upper_left_leg"

		bone.bodyId = dbox2d.CreateBody(worldId, &bodyDef)
		bone.frictionScale = fixed.Q32One()

		if colorize {
			shapeDef.Material.CustomColor = uint32(pantColor)
		}

		capsule := dbox2d.Capsule{
			Center1: dbox2d.Vec2{Y: fixed.Q32MustParse("-0.125").Mul(s)},
			Center2: dbox2d.Vec2{Y: fixed.Q32MustParse("0.125").Mul(s)},
			Radius:  fixed.Q32MustParse("0.06").Mul(s),
		}
		dbox2d.CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule)

		pivot := dbox2d.Vec2{Y: fixed.Q32MustParse("0.9").Mul(s)}.Add(position)
		jointDef := dbox2d.DefaultRevoluteJointDef()
		jointDef.BodyIdA = h.bones[bone.parentIndex].bodyId
		jointDef.BodyIdB = bone.bodyId
		jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
		jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
		jointDef.EnableLimit = enableLimit
		jointDef.LowerAngle = fixed.Q32FromRatio(-1, 40)
		jointDef.UpperAngle = fixed.Q32FromRatio(1, 5)
		jointDef.EnableMotor = enableMotor
		jointDef.MaxMotorTorque = bone.frictionScale.Mul(maxTorque)
		jointDef.EnableSpring = hertz.Greater(fixed.Q32Zero())
		jointDef.Hertz = hertz
		jointDef.DampingRatio = dampingRatio
		jointDef.DrawSize = drawSize

		bone.jointId = dbox2d.CreateRevoluteJoint(worldId, &jointDef)
	}

	points := []dbox2d.Vec2{
		{X: fixed.Q32MustParse("-0.03").Mul(s), Y: fixed.Q32MustParse("-0.185").Mul(s)},
		{X: fixed.Q32MustParse("0.11").Mul(s), Y: fixed.Q32MustParse("-0.185").Mul(s)},
		{X: fixed.Q32MustParse("0.11").Mul(s), Y: fixed.Q32MustParse("-0.16").Mul(s)},
		{X: fixed.Q32MustParse("-0.03").Mul(s), Y: fixed.Q32MustParse("-0.14").Mul(s)},
	}
	footHull := dbox2d.ComputeHull(points)
	footPolygon := dbox2d.MakePolygon(&footHull, fixed.Q32MustParse("0.015").Mul(s))

	{
		bone := &h.bones[boneLowerLeftLeg]
		bone.parentIndex = int(boneUpperLeftLeg)

		bodyDef.Position = dbox2d.Vec2{Y: fixed.Q32MustParse("0.475").Mul(s)}.Add(position)
		bodyDef.LinearDamping = fixed.Q32Zero()
		bodyDef.Name = "lower_left_leg"

		bone.bodyId = dbox2d.CreateBody(worldId, &bodyDef)
		bone.frictionScale = fixed.Q32Half()

		if colorize {
			shapeDef.Material.CustomColor = uint32(pantColor)
		}

		capsule := dbox2d.Capsule{
			Center1: dbox2d.Vec2{Y: fixed.Q32MustParse("-0.155").Mul(s)},
			Center2: dbox2d.Vec2{Y: fixed.Q32MustParse("0.125").Mul(s)},
			Radius:  fixed.Q32MustParse("0.045").Mul(s),
		}
		dbox2d.CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule)
		dbox2d.CreatePolygonShape(bone.bodyId, &footShapeDef, &footPolygon)

		pivot := dbox2d.Vec2{Y: fixed.Q32MustParse("0.625").Mul(s)}.Add(position)
		jointDef := dbox2d.DefaultRevoluteJointDef()
		jointDef.BodyIdA = h.bones[bone.parentIndex].bodyId
		jointDef.BodyIdB = bone.bodyId
		jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
		jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
		jointDef.EnableLimit = enableLimit
		jointDef.LowerAngle = fixed.Q32FromRatio(-1, 4)
		jointDef.UpperAngle = fixed.Q32FromRatio(-1, 100)
		jointDef.EnableMotor = enableMotor
		jointDef.MaxMotorTorque = bone.frictionScale.Mul(maxTorque)
		jointDef.EnableSpring = hertz.Greater(fixed.Q32Zero())
		jointDef.Hertz = hertz
		jointDef.DampingRatio = dampingRatio
		jointDef.DrawSize = drawSize

		bone.jointId = dbox2d.CreateRevoluteJoint(worldId, &jointDef)
	}

	{
		bone := &h.bones[boneUpperRightLeg]
		bone.parentIndex = int(boneHip)

		bodyDef.Position = dbox2d.Vec2{Y: fixed.Q32MustParse("0.775").Mul(s)}.Add(position)
		bodyDef.LinearDamping = fixed.Q32Zero()
		bodyDef.Name = "upper_right_leg"

		bone.bodyId = dbox2d.CreateBody(worldId, &bodyDef)
		bone.frictionScale = fixed.Q32One()

		if colorize {
			shapeDef.Material.CustomColor = uint32(pantColor)
		}

		capsule := dbox2d.Capsule{
			Center1: dbox2d.Vec2{Y: fixed.Q32MustParse("-0.125").Mul(s)},
			Center2: dbox2d.Vec2{Y: fixed.Q32MustParse("0.125").Mul(s)},
			Radius:  fixed.Q32MustParse("0.06").Mul(s),
		}
		dbox2d.CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule)

		pivot := dbox2d.Vec2{Y: fixed.Q32MustParse("0.9").Mul(s)}.Add(position)
		jointDef := dbox2d.DefaultRevoluteJointDef()
		jointDef.BodyIdA = h.bones[bone.parentIndex].bodyId
		jointDef.BodyIdB = bone.bodyId
		jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
		jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
		jointDef.EnableLimit = enableLimit
		jointDef.LowerAngle = fixed.Q32FromRatio(-1, 40)
		jointDef.UpperAngle = fixed.Q32FromRatio(1, 5)
		jointDef.EnableMotor = enableMotor
		jointDef.MaxMotorTorque = bone.frictionScale.Mul(maxTorque)
		jointDef.EnableSpring = hertz.Greater(fixed.Q32Zero())
		jointDef.Hertz = hertz
		jointDef.DampingRatio = dampingRatio
		jointDef.DrawSize = drawSize

		bone.jointId = dbox2d.CreateRevoluteJoint(worldId, &jointDef)
	}

	{
		bone := &h.bones[boneLowerRightLeg]
		bone.parentIndex = int(boneUpperRightLeg)

		bodyDef.Position = dbox2d.Vec2{Y: fixed.Q32MustParse("0.475").Mul(s)}.Add(position)
		bodyDef.LinearDamping = fixed.Q32Zero()
		bodyDef.Name = "lower_right_leg"

		bone.bodyId = dbox2d.CreateBody(worldId, &bodyDef)
		bone.frictionScale = fixed.Q32Half()

		if colorize {
			shapeDef.Material.CustomColor = uint32(pantColor)
		}

		capsule := dbox2d.Capsule{
			Center1: dbox2d.Vec2{Y: fixed.Q32MustParse("-0.155").Mul(s)},
			Center2: dbox2d.Vec2{Y: fixed.Q32MustParse("0.125").Mul(s)},
			Radius:  fixed.Q32MustParse("0.045").Mul(s),
		}
		dbox2d.CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule)
		dbox2d.CreatePolygonShape(bone.bodyId, &footShapeDef, &footPolygon)

		pivot := dbox2d.Vec2{Y: fixed.Q32MustParse("0.625").Mul(s)}.Add(position)
		jointDef := dbox2d.DefaultRevoluteJointDef()
		jointDef.BodyIdA = h.bones[bone.parentIndex].bodyId
		jointDef.BodyIdB = bone.bodyId
		jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
		jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
		jointDef.EnableLimit = enableLimit
		jointDef.LowerAngle = fixed.Q32FromRatio(-1, 4)
		jointDef.UpperAngle = fixed.Q32FromRatio(-1, 100)
		jointDef.EnableMotor = enableMotor
		jointDef.MaxMotorTorque = bone.frictionScale.Mul(maxTorque)
		jointDef.EnableSpring = hertz.Greater(fixed.Q32Zero())
		jointDef.Hertz = hertz
		jointDef.DampingRatio = dampingRatio
		jointDef.DrawSize = drawSize

		bone.jointId = dbox2d.CreateRevoluteJoint(worldId, &jointDef)
	}

	{
		bone := &h.bones[boneUpperLeftArm]
		bone.parentIndex = int(boneTorso)
		bone.frictionScale = fixed.Q32Half()

		bodyDef.Position = dbox2d.Vec2{Y: fixed.Q32MustParse("1.225").Mul(s)}.Add(position)
		bodyDef.LinearDamping = fixed.Q32Zero()
		bodyDef.Name = "upper_left_arm"

		bone.bodyId = dbox2d.CreateBody(worldId, &bodyDef)

		if colorize {
			shapeDef.Material.CustomColor = uint32(shirtColor)
		}

		capsule := dbox2d.Capsule{
			Center1: dbox2d.Vec2{Y: fixed.Q32MustParse("-0.125").Mul(s)},
			Center2: dbox2d.Vec2{Y: fixed.Q32MustParse("0.125").Mul(s)},
			Radius:  fixed.Q32MustParse("0.035").Mul(s),
		}
		dbox2d.CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule)

		pivot := dbox2d.Vec2{Y: fixed.Q32MustParse("1.35").Mul(s)}.Add(position)
		jointDef := dbox2d.DefaultRevoluteJointDef()
		jointDef.BodyIdA = h.bones[bone.parentIndex].bodyId
		jointDef.BodyIdB = bone.bodyId
		jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
		jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
		jointDef.EnableLimit = enableLimit
		jointDef.LowerAngle = fixed.Q32FromRatio(-1, 20)
		jointDef.UpperAngle = fixed.Q32FromRatio(2, 5)
		jointDef.EnableMotor = enableMotor
		jointDef.MaxMotorTorque = bone.frictionScale.Mul(maxTorque)
		jointDef.EnableSpring = hertz.Greater(fixed.Q32Zero())
		jointDef.Hertz = hertz
		jointDef.DampingRatio = dampingRatio
		jointDef.DrawSize = drawSize

		bone.jointId = dbox2d.CreateRevoluteJoint(worldId, &jointDef)
	}

	{
		bone := &h.bones[boneLowerLeftArm]
		bone.parentIndex = int(boneUpperLeftArm)

		bodyDef.Position = dbox2d.Vec2{Y: fixed.Q32MustParse("0.975").Mul(s)}.Add(position)
		bodyDef.LinearDamping = fixed.Q32MustParse("0.1")
		bodyDef.Name = "lower_left_arm"

		bone.bodyId = dbox2d.CreateBody(worldId, &bodyDef)
		bone.frictionScale = fixed.Q32MustParse("0.1")

		if colorize {
			shapeDef.Material.CustomColor = uint32(skinColor)
		}

		capsule := dbox2d.Capsule{
			Center1: dbox2d.Vec2{Y: fixed.Q32MustParse("-0.125").Mul(s)},
			Center2: dbox2d.Vec2{Y: fixed.Q32MustParse("0.125").Mul(s)},
			Radius:  fixed.Q32MustParse("0.03").Mul(s),
		}
		dbox2d.CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule)

		pivot := dbox2d.Vec2{Y: fixed.Q32MustParse("1.1").Mul(s)}.Add(position)
		jointDef := dbox2d.DefaultRevoluteJointDef()
		jointDef.BodyIdA = h.bones[bone.parentIndex].bodyId
		jointDef.BodyIdB = bone.bodyId
		jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
		jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
		jointDef.ReferenceAngle = fixed.Q32FromRatio(1, 8)
		jointDef.EnableLimit = enableLimit
		jointDef.LowerAngle = fixed.Q32FromRatio(-1, 10)
		jointDef.UpperAngle = fixed.Q32FromRatio(3, 20)
		jointDef.EnableMotor = enableMotor
		jointDef.MaxMotorTorque = bone.frictionScale.Mul(maxTorque)
		jointDef.EnableSpring = hertz.Greater(fixed.Q32Zero())
		jointDef.Hertz = hertz
		jointDef.DampingRatio = dampingRatio
		jointDef.DrawSize = drawSize

		bone.jointId = dbox2d.CreateRevoluteJoint(worldId, &jointDef)
	}

	{
		bone := &h.bones[boneUpperRightArm]
		bone.parentIndex = int(boneTorso)

		bodyDef.Position = dbox2d.Vec2{Y: fixed.Q32MustParse("1.225").Mul(s)}.Add(position)
		bodyDef.LinearDamping = fixed.Q32Zero()
		bodyDef.Name = "upper_right_arm"

		bone.bodyId = dbox2d.CreateBody(worldId, &bodyDef)
		bone.frictionScale = fixed.Q32Half()

		if colorize {
			shapeDef.Material.CustomColor = uint32(shirtColor)
		}

		capsule := dbox2d.Capsule{
			Center1: dbox2d.Vec2{Y: fixed.Q32MustParse("-0.125").Mul(s)},
			Center2: dbox2d.Vec2{Y: fixed.Q32MustParse("0.125").Mul(s)},
			Radius:  fixed.Q32MustParse("0.035").Mul(s),
		}
		dbox2d.CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule)

		pivot := dbox2d.Vec2{Y: fixed.Q32MustParse("1.35").Mul(s)}.Add(position)
		jointDef := dbox2d.DefaultRevoluteJointDef()
		jointDef.BodyIdA = h.bones[bone.parentIndex].bodyId
		jointDef.BodyIdB = bone.bodyId
		jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
		jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
		jointDef.EnableLimit = enableLimit
		jointDef.LowerAngle = fixed.Q32FromRatio(-1, 20)
		jointDef.UpperAngle = fixed.Q32FromRatio(2, 5)
		jointDef.EnableMotor = enableMotor
		jointDef.MaxMotorTorque = bone.frictionScale.Mul(maxTorque)
		jointDef.EnableSpring = hertz.Greater(fixed.Q32Zero())
		jointDef.Hertz = hertz
		jointDef.DampingRatio = dampingRatio
		jointDef.DrawSize = drawSize

		bone.jointId = dbox2d.CreateRevoluteJoint(worldId, &jointDef)
	}

	{
		bone := &h.bones[boneLowerRightArm]
		bone.parentIndex = int(boneUpperRightArm)

		bodyDef.Position = dbox2d.Vec2{Y: fixed.Q32MustParse("0.975").Mul(s)}.Add(position)
		bodyDef.LinearDamping = fixed.Q32MustParse("0.1")
		bodyDef.Name = "lower_right_arm"

		bone.bodyId = dbox2d.CreateBody(worldId, &bodyDef)
		bone.frictionScale = fixed.Q32MustParse("0.1")

		if colorize {
			shapeDef.Material.CustomColor = uint32(skinColor)
		}

		capsule := dbox2d.Capsule{
			Center1: dbox2d.Vec2{Y: fixed.Q32MustParse("-0.125").Mul(s)},
			Center2: dbox2d.Vec2{Y: fixed.Q32MustParse("0.125").Mul(s)},
			Radius:  fixed.Q32MustParse("0.03").Mul(s),
		}
		dbox2d.CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule)

		pivot := dbox2d.Vec2{Y: fixed.Q32MustParse("1.1").Mul(s)}.Add(position)
		jointDef := dbox2d.DefaultRevoluteJointDef()
		jointDef.BodyIdA = h.bones[bone.parentIndex].bodyId
		jointDef.BodyIdB = bone.bodyId
		jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
		jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
		jointDef.ReferenceAngle = fixed.Q32FromRatio(1, 8)
		jointDef.EnableLimit = enableLimit
		jointDef.LowerAngle = fixed.Q32FromRatio(-1, 10)
		jointDef.UpperAngle = fixed.Q32FromRatio(3, 20)
		jointDef.EnableMotor = enableMotor
		jointDef.MaxMotorTorque = bone.frictionScale.Mul(maxTorque)
		jointDef.EnableSpring = hertz.Greater(fixed.Q32Zero())
		jointDef.Hertz = hertz
		jointDef.DampingRatio = dampingRatio
		jointDef.DrawSize = drawSize

		bone.jointId = dbox2d.CreateRevoluteJoint(worldId, &jointDef)
	}

	h.isSpawned = true
	return h
}

func (h *human) destroy() {
	for i := range int(boneCount) {
		if h.bones[i].jointId.IsNull() {
			continue
		}

		dbox2d.DestroyJoint(h.bones[i].jointId)
		h.bones[i].jointId = dbox2d.JointId{}
	}

	for i := range int(boneCount) {
		if h.bones[i].bodyId.IsNull() {
			continue
		}

		dbox2d.DestroyBody(h.bones[i].bodyId)
		h.bones[i].bodyId = dbox2d.BodyId{}
	}

	h.isSpawned = false
}

func (h *human) setVelocity(velocity dbox2d.Vec2) {
	for i := range int(boneCount) {
		bodyId := h.bones[i].bodyId
		if bodyId.IsNull() {
			continue
		}

		bodyId.SetLinearVelocity(velocity)
	}
}

func (h *human) applyRandomAngularImpulse(magnitude dbox2d.Q) {
	impulse := randomFloatRange(magnitude.Neg(), magnitude)
	h.bones[boneTorso].bodyId.ApplyAngularImpulse(impulse, true)
}

func (h *human) setJointFrictionTorque(torque dbox2d.Q) {
	if torque == fixed.Q32Zero() {
		for i := 1; i < int(boneCount); i++ {
			h.bones[i].jointId.EnableMotor(false)
		}
		return
	}

	for i := 1; i < int(boneCount); i++ {
		h.bones[i].jointId.EnableMotor(true)
		scale := h.scale.Mul(h.bones[i].frictionScale)
		h.bones[i].jointId.SetMaxMotorTorque(scale.Mul(torque))
	}
}

func (h *human) setJointSpringHertz(hertz dbox2d.Q) {
	if hertz == fixed.Q32Zero() {
		for i := 1; i < int(boneCount); i++ {
			h.bones[i].jointId.EnableSpring(false)
		}
		return
	}

	for i := 1; i < int(boneCount); i++ {
		h.bones[i].jointId.EnableSpring(true)
		h.bones[i].jointId.SetSpringHertz(hertz)
	}
}

func (h *human) setJointDampingRatio(dampingRatio dbox2d.Q) {
	for i := 1; i < int(boneCount); i++ {
		h.bones[i].jointId.SetSpringDampingRatio(dampingRatio)
	}
}

func (h *human) enableSensorEvents(enable bool) {
	shapes := make([]dbox2d.ShapeId, 1)
	count := h.bones[boneTorso].bodyId.GetShapes(shapes)
	if count == 1 {
		shapes[0].EnableSensorEvents(enable)
	}
}

func (h *human) setScale(scale dbox2d.Q) {
	ratio := scale.Div(h.scale)
	originalRatio := scale.Div(h.originalScale)
	frictionTorque := originalRatio.Mul(originalRatio).Mul(originalRatio).Mul(h.frictionTorque)

	origin := h.bones[boneHip].bodyId.GetPosition()
	for boneIndex := range int(boneCount) {
		bone := &h.bones[boneIndex]
		if boneIndex > 0 {
			transform := bone.bodyId.GetTransform()
			transform.P = origin.Add(transform.P.Sub(origin).Mul(ratio))
			bone.bodyId.SetTransform(transform.P, transform.Q)

			localAnchorA := bone.jointId.GetLocalAnchorA().Mul(ratio)
			localAnchorB := bone.jointId.GetLocalAnchorB().Mul(ratio)
			bone.jointId.SetLocalAnchorA(localAnchorA)
			bone.jointId.SetLocalAnchorB(localAnchorB)

			if bone.jointId.GetType() == dbox2d.RevoluteJoint {
				bone.jointId.SetMaxMotorTorque(bone.frictionScale.Mul(frictionTorque))
			}
		}

		shapeIds := make([]dbox2d.ShapeId, 2)
		shapeCount := bone.bodyId.GetShapes(shapeIds)
		for shapeIndex := range shapeCount {
			shapeId := shapeIds[shapeIndex]
			switch shapeId.GetType() {
			case dbox2d.CapsuleShape:
				capsule := shapeId.GetCapsule()
				capsule.Center1 = capsule.Center1.Mul(ratio)
				capsule.Center2 = capsule.Center2.Mul(ratio)
				capsule.Radius = capsule.Radius.Mul(ratio)
				shapeId.SetCapsule(&capsule)
			case dbox2d.PolygonShape:
				polygon := shapeId.GetPolygon()
				for pointIndex := range polygon.Count {
					polygon.Vertices[pointIndex] = polygon.Vertices[pointIndex].Mul(ratio)
				}
				polygon.Centroid = polygon.Centroid.Mul(ratio)
				polygon.Radius = polygon.Radius.Mul(ratio)
				shapeId.SetPolygon(&polygon)
			}
		}

		bone.bodyId.ApplyMassFromShapes()
	}

	h.scale = scale
}
