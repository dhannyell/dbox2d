// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from shared/human.h and shared/human.c of Box2D v3.1.1

package shared

import . "github.com/dhannyell/dbox2d"

// boneId names the eleven limbs in the order shared/human.h declares them,
// which is the order they are created in and so the order a scene hashes in.
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

// bone is one limb: its body, the joint to its parent, and the share of the
// ragdoll friction torque that joint carries.
type bone struct {
	bodyId        BodyId
	jointId       JointId
	frictionScale Q
	parentIndex   int
}

// Human is the ragdoll of shared/human.c. Rain spawns them by the group; the
// samples app also uses them directly, which is why this lives here rather
// than in either consumer.
type Human struct {
	bones          [boneCount]bone
	frictionTorque Q
	originalScale  Q
	scale          Q
	IsSpawned      bool
}

// CreateHuman is CreateHuman of shared/human.c. userData is attached to every
// bone body, and colorize paints the bones the way the sample app draws them;
// a benchmark passes nil and false, as shared/benchmarks.c does.
func CreateHuman(worldId WorldId, position Vec2, scale, frictionTorque, hertz, dampingRatio Q, groupIndex int, userData any, colorize bool) Human {
	h := Human{
		frictionTorque: frictionTorque,
		originalScale:  scale,
		scale:          scale,
	}

	for i := range h.bones {
		h.bones[i].bodyId = BodyId{}
		h.bones[i].jointId = JointId{}
		h.bones[i].frictionScale = QOne()
		h.bones[i].parentIndex = -1
	}

	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.SleepThreshold = QMustParse("0.1")
	bodyDef.UserData = userData

	shapeDef := DefaultShapeDef()
	shapeDef.Material.Friction = QMustParse("0.2")
	shapeDef.Filter.GroupIndex = -groupIndex
	shapeDef.Filter.CategoryBits = 2
	shapeDef.Filter.MaskBits = 1 | 2

	footShapeDef := shapeDef
	footShapeDef.Material.Friction = QMustParse("0.05")
	footShapeDef.Filter.CategoryBits = 2
	footShapeDef.Filter.MaskBits = 1

	if colorize {
		footShapeDef.Material.CustomColor = uint32(ColorSaddleBrown)
	}

	s := scale
	maxTorque := frictionTorque.Mul(s)
	enableMotor := true
	enableLimit := true
	drawSize := QMustParse("0.05")

	shirtColor := ColorMediumTurquoise
	pantColor := ColorDodgerBlue
	skinColors := [4]HexColor{
		ColorNavajoWhite,
		ColorLightYellow,
		ColorPeru,
		ColorTan,
	}
	skinColor := skinColors[groupIndex%4]

	{
		bone := &h.bones[boneHip]
		bone.parentIndex = -1

		bodyDef.Position = Vec2{Y: QMustParse("0.95").Mul(s)}.Add(position)
		bodyDef.LinearDamping = QZero()
		bodyDef.Name = "hip"

		bone.bodyId = CreateBody(worldId, &bodyDef)

		if colorize {
			shapeDef.Material.CustomColor = uint32(pantColor)
		}

		capsule := Capsule{
			Center1: Vec2{Y: QMustParse("-0.02").Mul(s)},
			Center2: Vec2{Y: QMustParse("0.02").Mul(s)},
			Radius:  QMustParse("0.095").Mul(s),
		}
		CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule)
	}

	{
		bone := &h.bones[boneTorso]
		bone.parentIndex = int(boneHip)

		bodyDef.Position = Vec2{Y: QMustParse("1.2").Mul(s)}.Add(position)
		bodyDef.LinearDamping = QZero()
		bodyDef.Name = "torso"

		bone.bodyId = CreateBody(worldId, &bodyDef)
		bone.frictionScale = QHalf()
		bodyDef.Type = DynamicBody

		if colorize {
			shapeDef.Material.CustomColor = uint32(shirtColor)
		}

		capsule := Capsule{
			Center1: Vec2{Y: QMustParse("-0.135").Mul(s)},
			Center2: Vec2{Y: QMustParse("0.135").Mul(s)},
			Radius:  QMustParse("0.09").Mul(s),
		}
		CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule)

		pivot := Vec2{Y: s}.Add(position)
		jointDef := DefaultRevoluteJointDef()
		jointDef.BodyIdA = h.bones[bone.parentIndex].bodyId
		jointDef.BodyIdB = bone.bodyId
		jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
		jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
		jointDef.EnableLimit = enableLimit
		// Joint angles use turns; the reference values are multiples of pi.
		jointDef.LowerAngle = QFromRatio(-1, 8)
		jointDef.UpperAngle = QZero()
		jointDef.EnableMotor = enableMotor
		jointDef.MaxMotorTorque = bone.frictionScale.Mul(maxTorque)
		jointDef.EnableSpring = hertz.Greater(QZero())
		jointDef.Hertz = hertz
		jointDef.DampingRatio = dampingRatio
		jointDef.DrawSize = drawSize

		bone.jointId = CreateRevoluteJoint(worldId, &jointDef)
	}

	{
		bone := &h.bones[boneHead]
		bone.parentIndex = int(boneTorso)

		bodyDef.Position = Vec2{Y: QMustParse("1.475").Mul(s)}.Add(position)
		bodyDef.LinearDamping = QMustParse("0.1")
		bodyDef.Name = "head"

		bone.bodyId = CreateBody(worldId, &bodyDef)
		bone.frictionScale = QFromRatio(1, 4)

		if colorize {
			shapeDef.Material.CustomColor = uint32(skinColor)
		}

		capsule := Capsule{
			Center1: Vec2{Y: QMustParse("-0.038").Mul(s)},
			Center2: Vec2{Y: QMustParse("0.039").Mul(s)},
			Radius:  QMustParse("0.075").Mul(s),
		}
		CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule)

		pivot := Vec2{Y: QMustParse("1.4").Mul(s)}.Add(position)
		jointDef := DefaultRevoluteJointDef()
		jointDef.BodyIdA = h.bones[bone.parentIndex].bodyId
		jointDef.BodyIdB = bone.bodyId
		jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
		jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
		jointDef.EnableLimit = enableLimit
		jointDef.LowerAngle = QFromRatio(-3, 20)
		jointDef.UpperAngle = QFromRatio(1, 20)
		jointDef.EnableMotor = enableMotor
		jointDef.MaxMotorTorque = bone.frictionScale.Mul(maxTorque)
		jointDef.EnableSpring = hertz.Greater(QZero())
		jointDef.Hertz = hertz
		jointDef.DampingRatio = dampingRatio
		jointDef.DrawSize = drawSize

		bone.jointId = CreateRevoluteJoint(worldId, &jointDef)
	}

	{
		bone := &h.bones[boneUpperLeftLeg]
		bone.parentIndex = int(boneHip)

		bodyDef.Position = Vec2{Y: QMustParse("0.775").Mul(s)}.Add(position)
		bodyDef.LinearDamping = QZero()
		bodyDef.Name = "upper_left_leg"

		bone.bodyId = CreateBody(worldId, &bodyDef)
		bone.frictionScale = QOne()

		if colorize {
			shapeDef.Material.CustomColor = uint32(pantColor)
		}

		capsule := Capsule{
			Center1: Vec2{Y: QMustParse("-0.125").Mul(s)},
			Center2: Vec2{Y: QMustParse("0.125").Mul(s)},
			Radius:  QMustParse("0.06").Mul(s),
		}
		CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule)

		pivot := Vec2{Y: QMustParse("0.9").Mul(s)}.Add(position)
		jointDef := DefaultRevoluteJointDef()
		jointDef.BodyIdA = h.bones[bone.parentIndex].bodyId
		jointDef.BodyIdB = bone.bodyId
		jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
		jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
		jointDef.EnableLimit = enableLimit
		jointDef.LowerAngle = QFromRatio(-1, 40)
		jointDef.UpperAngle = QFromRatio(1, 5)
		jointDef.EnableMotor = enableMotor
		jointDef.MaxMotorTorque = bone.frictionScale.Mul(maxTorque)
		jointDef.EnableSpring = hertz.Greater(QZero())
		jointDef.Hertz = hertz
		jointDef.DampingRatio = dampingRatio
		jointDef.DrawSize = drawSize

		bone.jointId = CreateRevoluteJoint(worldId, &jointDef)
	}

	points := []Vec2{
		{X: QMustParse("-0.03").Mul(s), Y: QMustParse("-0.185").Mul(s)},
		{X: QMustParse("0.11").Mul(s), Y: QMustParse("-0.185").Mul(s)},
		{X: QMustParse("0.11").Mul(s), Y: QMustParse("-0.16").Mul(s)},
		{X: QMustParse("-0.03").Mul(s), Y: QMustParse("-0.14").Mul(s)},
	}
	footHull := ComputeHull(points)
	footPolygon := MakePolygon(&footHull, QMustParse("0.015").Mul(s))

	{
		bone := &h.bones[boneLowerLeftLeg]
		bone.parentIndex = int(boneUpperLeftLeg)

		bodyDef.Position = Vec2{Y: QMustParse("0.475").Mul(s)}.Add(position)
		bodyDef.LinearDamping = QZero()
		bodyDef.Name = "lower_left_leg"

		bone.bodyId = CreateBody(worldId, &bodyDef)
		bone.frictionScale = QHalf()

		if colorize {
			shapeDef.Material.CustomColor = uint32(pantColor)
		}

		capsule := Capsule{
			Center1: Vec2{Y: QMustParse("-0.155").Mul(s)},
			Center2: Vec2{Y: QMustParse("0.125").Mul(s)},
			Radius:  QMustParse("0.045").Mul(s),
		}
		CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule)
		CreatePolygonShape(bone.bodyId, &footShapeDef, &footPolygon)

		pivot := Vec2{Y: QMustParse("0.625").Mul(s)}.Add(position)
		jointDef := DefaultRevoluteJointDef()
		jointDef.BodyIdA = h.bones[bone.parentIndex].bodyId
		jointDef.BodyIdB = bone.bodyId
		jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
		jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
		jointDef.EnableLimit = enableLimit
		jointDef.LowerAngle = QFromRatio(-1, 4)
		jointDef.UpperAngle = QFromRatio(-1, 100)
		jointDef.EnableMotor = enableMotor
		jointDef.MaxMotorTorque = bone.frictionScale.Mul(maxTorque)
		jointDef.EnableSpring = hertz.Greater(QZero())
		jointDef.Hertz = hertz
		jointDef.DampingRatio = dampingRatio
		jointDef.DrawSize = drawSize

		bone.jointId = CreateRevoluteJoint(worldId, &jointDef)
	}

	{
		bone := &h.bones[boneUpperRightLeg]
		bone.parentIndex = int(boneHip)

		bodyDef.Position = Vec2{Y: QMustParse("0.775").Mul(s)}.Add(position)
		bodyDef.LinearDamping = QZero()
		bodyDef.Name = "upper_right_leg"

		bone.bodyId = CreateBody(worldId, &bodyDef)
		bone.frictionScale = QOne()

		if colorize {
			shapeDef.Material.CustomColor = uint32(pantColor)
		}

		capsule := Capsule{
			Center1: Vec2{Y: QMustParse("-0.125").Mul(s)},
			Center2: Vec2{Y: QMustParse("0.125").Mul(s)},
			Radius:  QMustParse("0.06").Mul(s),
		}
		CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule)

		pivot := Vec2{Y: QMustParse("0.9").Mul(s)}.Add(position)
		jointDef := DefaultRevoluteJointDef()
		jointDef.BodyIdA = h.bones[bone.parentIndex].bodyId
		jointDef.BodyIdB = bone.bodyId
		jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
		jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
		jointDef.EnableLimit = enableLimit
		jointDef.LowerAngle = QFromRatio(-1, 40)
		jointDef.UpperAngle = QFromRatio(1, 5)
		jointDef.EnableMotor = enableMotor
		jointDef.MaxMotorTorque = bone.frictionScale.Mul(maxTorque)
		jointDef.EnableSpring = hertz.Greater(QZero())
		jointDef.Hertz = hertz
		jointDef.DampingRatio = dampingRatio
		jointDef.DrawSize = drawSize

		bone.jointId = CreateRevoluteJoint(worldId, &jointDef)
	}

	{
		bone := &h.bones[boneLowerRightLeg]
		bone.parentIndex = int(boneUpperRightLeg)

		bodyDef.Position = Vec2{Y: QMustParse("0.475").Mul(s)}.Add(position)
		bodyDef.LinearDamping = QZero()
		bodyDef.Name = "lower_right_leg"

		bone.bodyId = CreateBody(worldId, &bodyDef)
		bone.frictionScale = QHalf()

		if colorize {
			shapeDef.Material.CustomColor = uint32(pantColor)
		}

		capsule := Capsule{
			Center1: Vec2{Y: QMustParse("-0.155").Mul(s)},
			Center2: Vec2{Y: QMustParse("0.125").Mul(s)},
			Radius:  QMustParse("0.045").Mul(s),
		}
		CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule)
		CreatePolygonShape(bone.bodyId, &footShapeDef, &footPolygon)

		pivot := Vec2{Y: QMustParse("0.625").Mul(s)}.Add(position)
		jointDef := DefaultRevoluteJointDef()
		jointDef.BodyIdA = h.bones[bone.parentIndex].bodyId
		jointDef.BodyIdB = bone.bodyId
		jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
		jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
		jointDef.EnableLimit = enableLimit
		jointDef.LowerAngle = QFromRatio(-1, 4)
		jointDef.UpperAngle = QFromRatio(-1, 100)
		jointDef.EnableMotor = enableMotor
		jointDef.MaxMotorTorque = bone.frictionScale.Mul(maxTorque)
		jointDef.EnableSpring = hertz.Greater(QZero())
		jointDef.Hertz = hertz
		jointDef.DampingRatio = dampingRatio
		jointDef.DrawSize = drawSize

		bone.jointId = CreateRevoluteJoint(worldId, &jointDef)
	}

	{
		bone := &h.bones[boneUpperLeftArm]
		bone.parentIndex = int(boneTorso)
		bone.frictionScale = QHalf()

		bodyDef.Position = Vec2{Y: QMustParse("1.225").Mul(s)}.Add(position)
		bodyDef.LinearDamping = QZero()
		bodyDef.Name = "upper_left_arm"

		bone.bodyId = CreateBody(worldId, &bodyDef)

		if colorize {
			shapeDef.Material.CustomColor = uint32(shirtColor)
		}

		capsule := Capsule{
			Center1: Vec2{Y: QMustParse("-0.125").Mul(s)},
			Center2: Vec2{Y: QMustParse("0.125").Mul(s)},
			Radius:  QMustParse("0.035").Mul(s),
		}
		CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule)

		pivot := Vec2{Y: QMustParse("1.35").Mul(s)}.Add(position)
		jointDef := DefaultRevoluteJointDef()
		jointDef.BodyIdA = h.bones[bone.parentIndex].bodyId
		jointDef.BodyIdB = bone.bodyId
		jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
		jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
		jointDef.EnableLimit = enableLimit
		jointDef.LowerAngle = QFromRatio(-1, 20)
		jointDef.UpperAngle = QFromRatio(2, 5)
		jointDef.EnableMotor = enableMotor
		jointDef.MaxMotorTorque = bone.frictionScale.Mul(maxTorque)
		jointDef.EnableSpring = hertz.Greater(QZero())
		jointDef.Hertz = hertz
		jointDef.DampingRatio = dampingRatio
		jointDef.DrawSize = drawSize

		bone.jointId = CreateRevoluteJoint(worldId, &jointDef)
	}

	{
		bone := &h.bones[boneLowerLeftArm]
		bone.parentIndex = int(boneUpperLeftArm)

		bodyDef.Position = Vec2{Y: QMustParse("0.975").Mul(s)}.Add(position)
		bodyDef.LinearDamping = QMustParse("0.1")
		bodyDef.Name = "lower_left_arm"

		bone.bodyId = CreateBody(worldId, &bodyDef)
		bone.frictionScale = QMustParse("0.1")

		if colorize {
			shapeDef.Material.CustomColor = uint32(skinColor)
		}

		capsule := Capsule{
			Center1: Vec2{Y: QMustParse("-0.125").Mul(s)},
			Center2: Vec2{Y: QMustParse("0.125").Mul(s)},
			Radius:  QMustParse("0.03").Mul(s),
		}
		CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule)

		pivot := Vec2{Y: QMustParse("1.1").Mul(s)}.Add(position)
		jointDef := DefaultRevoluteJointDef()
		jointDef.BodyIdA = h.bones[bone.parentIndex].bodyId
		jointDef.BodyIdB = bone.bodyId
		jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
		jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
		jointDef.ReferenceAngle = QFromRatio(1, 8)
		jointDef.EnableLimit = enableLimit
		jointDef.LowerAngle = QFromRatio(-1, 10)
		jointDef.UpperAngle = QFromRatio(3, 20)
		jointDef.EnableMotor = enableMotor
		jointDef.MaxMotorTorque = bone.frictionScale.Mul(maxTorque)
		jointDef.EnableSpring = hertz.Greater(QZero())
		jointDef.Hertz = hertz
		jointDef.DampingRatio = dampingRatio
		jointDef.DrawSize = drawSize

		bone.jointId = CreateRevoluteJoint(worldId, &jointDef)
	}

	{
		bone := &h.bones[boneUpperRightArm]
		bone.parentIndex = int(boneTorso)

		bodyDef.Position = Vec2{Y: QMustParse("1.225").Mul(s)}.Add(position)
		bodyDef.LinearDamping = QZero()
		bodyDef.Name = "upper_right_arm"

		bone.bodyId = CreateBody(worldId, &bodyDef)
		bone.frictionScale = QHalf()

		if colorize {
			shapeDef.Material.CustomColor = uint32(shirtColor)
		}

		capsule := Capsule{
			Center1: Vec2{Y: QMustParse("-0.125").Mul(s)},
			Center2: Vec2{Y: QMustParse("0.125").Mul(s)},
			Radius:  QMustParse("0.035").Mul(s),
		}
		CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule)

		pivot := Vec2{Y: QMustParse("1.35").Mul(s)}.Add(position)
		jointDef := DefaultRevoluteJointDef()
		jointDef.BodyIdA = h.bones[bone.parentIndex].bodyId
		jointDef.BodyIdB = bone.bodyId
		jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
		jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
		jointDef.EnableLimit = enableLimit
		jointDef.LowerAngle = QFromRatio(-1, 20)
		jointDef.UpperAngle = QFromRatio(2, 5)
		jointDef.EnableMotor = enableMotor
		jointDef.MaxMotorTorque = bone.frictionScale.Mul(maxTorque)
		jointDef.EnableSpring = hertz.Greater(QZero())
		jointDef.Hertz = hertz
		jointDef.DampingRatio = dampingRatio
		jointDef.DrawSize = drawSize

		bone.jointId = CreateRevoluteJoint(worldId, &jointDef)
	}

	{
		bone := &h.bones[boneLowerRightArm]
		bone.parentIndex = int(boneUpperRightArm)

		bodyDef.Position = Vec2{Y: QMustParse("0.975").Mul(s)}.Add(position)
		bodyDef.LinearDamping = QMustParse("0.1")
		bodyDef.Name = "lower_right_arm"

		bone.bodyId = CreateBody(worldId, &bodyDef)
		bone.frictionScale = QMustParse("0.1")

		if colorize {
			shapeDef.Material.CustomColor = uint32(skinColor)
		}

		capsule := Capsule{
			Center1: Vec2{Y: QMustParse("-0.125").Mul(s)},
			Center2: Vec2{Y: QMustParse("0.125").Mul(s)},
			Radius:  QMustParse("0.03").Mul(s),
		}
		CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule)

		pivot := Vec2{Y: QMustParse("1.1").Mul(s)}.Add(position)
		jointDef := DefaultRevoluteJointDef()
		jointDef.BodyIdA = h.bones[bone.parentIndex].bodyId
		jointDef.BodyIdB = bone.bodyId
		jointDef.LocalAnchorA = jointDef.BodyIdA.GetLocalPoint(pivot)
		jointDef.LocalAnchorB = jointDef.BodyIdB.GetLocalPoint(pivot)
		jointDef.ReferenceAngle = QFromRatio(1, 8)
		jointDef.EnableLimit = enableLimit
		jointDef.LowerAngle = QFromRatio(-1, 10)
		jointDef.UpperAngle = QFromRatio(3, 20)
		jointDef.EnableMotor = enableMotor
		jointDef.MaxMotorTorque = bone.frictionScale.Mul(maxTorque)
		jointDef.EnableSpring = hertz.Greater(QZero())
		jointDef.Hertz = hertz
		jointDef.DampingRatio = dampingRatio
		jointDef.DrawSize = drawSize

		bone.jointId = CreateRevoluteJoint(worldId, &jointDef)
	}

	h.IsSpawned = true
	return h
}

// Destroy is DestroyHuman: the joints first, then the bodies.
func (h *Human) Destroy() {
	for i := range int(boneCount) {
		if h.bones[i].jointId.IsNull() {
			continue
		}

		DestroyJoint(h.bones[i].jointId)
		h.bones[i].jointId = JointId{}
	}

	for i := range int(boneCount) {
		if h.bones[i].bodyId.IsNull() {
			continue
		}

		DestroyBody(h.bones[i].bodyId)
		h.bones[i].bodyId = BodyId{}
	}

	h.IsSpawned = false
}

// SetVelocity is Human_SetVelocity.
func (h *Human) SetVelocity(velocity Vec2) {
	for i := range int(boneCount) {
		bodyId := h.bones[i].bodyId
		if bodyId.IsNull() {
			continue
		}

		bodyId.SetLinearVelocity(velocity)
	}
}

// ApplyRandomAngularImpulse is Human_ApplyRandomAngularImpulse. It draws from
// RandomSeed, so a caller that wants a repeatable result reseeds first.
func (h *Human) ApplyRandomAngularImpulse(magnitude Q) {
	impulse := RandomFloatRange(magnitude.Neg(), magnitude)
	h.bones[boneTorso].bodyId.ApplyAngularImpulse(impulse, true)
}

// SetJointFrictionTorque is Human_SetJointFrictionTorque.
func (h *Human) SetJointFrictionTorque(torque Q) {
	if torque == QZero() {
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

// SetJointSpringHertz is Human_SetJointSpringHertz.
func (h *Human) SetJointSpringHertz(hertz Q) {
	if hertz == QZero() {
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

// SetJointDampingRatio is Human_SetJointDampingRatio.
func (h *Human) SetJointDampingRatio(dampingRatio Q) {
	for i := 1; i < int(boneCount); i++ {
		h.bones[i].jointId.SetSpringDampingRatio(dampingRatio)
	}
}

// EnableSensorEvents is Human_EnableSensorEvents.
func (h *Human) EnableSensorEvents(enable bool) {
	shapes := make([]ShapeId, 1)
	count := h.bones[boneTorso].bodyId.GetShapes(shapes)
	if count == 1 {
		shapes[0].EnableSensorEvents(enable)
	}
}

// SetScale is Human_SetScale: it rescales every bone about the hip, along
// with the joint anchors, the shapes and the motor torques.
func (h *Human) SetScale(scale Q) {
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

			if bone.jointId.GetType() == RevoluteJoint {
				bone.jointId.SetMaxMotorTorque(bone.frictionScale.Mul(frictionTorque))
			}
		}

		shapeIds := make([]ShapeId, 2)
		shapeCount := bone.bodyId.GetShapes(shapeIds)
		for shapeIndex := range shapeCount {
			shapeId := shapeIds[shapeIndex]
			switch shapeId.GetType() {
			case CapsuleShape:
				capsule := shapeId.GetCapsule()
				capsule.Center1 = capsule.Center1.Mul(ratio)
				capsule.Center2 = capsule.Center2.Mul(ratio)
				capsule.Radius = capsule.Radius.Mul(ratio)
				shapeId.SetCapsule(&capsule)
			case PolygonShape:
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
