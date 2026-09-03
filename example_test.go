package dbox2d_test

import (
	"fmt"

	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/fixed"
)

// exampleWorld builds a world with a static ground body and one dynamic
// box above it. The examples hang joints between the two.
func exampleWorld() (dbox2d.WorldId, dbox2d.BodyId, dbox2d.BodyId) {
	worldDef := dbox2d.DefaultWorldDef()
	worldId := dbox2d.CreateWorld(&worldDef)

	groundDef := dbox2d.DefaultBodyDef()
	groundId := dbox2d.CreateBody(worldId, &groundDef)

	boxDef := dbox2d.DefaultBodyDef()
	boxDef.Type = dbox2d.DynamicBody
	boxDef.Position = dbox2d.Vec2{Y: fixed.Q32FromInt(2)}
	boxId := dbox2d.CreateBody(worldId, &boxDef)
	shapeDef := dbox2d.DefaultShapeDef()
	box := dbox2d.MakeSquare(fixed.Q32Half())
	dbox2d.CreatePolygonShape(boxId, &shapeDef, &box)

	return worldId, groundId, boxId
}

func ExampleCreateDistanceJoint() {
	worldId, groundId, boxId := exampleWorld()
	defer dbox2d.DestroyWorld(worldId)

	// A rope of length two from the ground origin to the box center.
	def := dbox2d.DefaultDistanceJointDef()
	def.BodyIdA = groundId
	def.BodyIdB = boxId
	def.Length = fixed.Q32FromInt(2)
	jointId := dbox2d.CreateDistanceJoint(worldId, &def)

	fmt.Println(jointId.IsNull())
	// Output: false
}

func ExampleWorldId_SetFrictionCallback() {
	worldDef := dbox2d.DefaultWorldDef()
	worldId := dbox2d.CreateWorld(&worldDef)
	defer dbox2d.DestroyWorld(worldId)

	worldId.SetFrictionCallback(func(frictionA dbox2d.Q, _ int, frictionB dbox2d.Q, _ int) dbox2d.Q {
		return frictionA.Add(frictionB).Div(fixed.Q32FromInt(2))
	})

	fmt.Println(worldId.IsValid())
	// Output: true
}

func ExampleCreateFilterJoint() {
	worldId, groundId, boxId := exampleWorld()
	defer dbox2d.DestroyWorld(worldId)

	// The box falls through the ground; the filter joint disables the
	// collision between the two bodies.
	def := dbox2d.DefaultFilterJointDef()
	def.BodyIdA = groundId
	def.BodyIdB = boxId
	jointId := dbox2d.CreateFilterJoint(worldId, &def)

	fmt.Println(jointId.IsNull())
	// Output: false
}

func ExampleCreateMotorJoint() {
	worldId, groundId, boxId := exampleWorld()
	defer dbox2d.DestroyWorld(worldId)

	// The motor drives the box toward an offset of (1, 2) from the ground.
	def := dbox2d.DefaultMotorJointDef()
	def.BodyIdA = groundId
	def.BodyIdB = boxId
	def.LinearOffset = dbox2d.Vec2{X: fixed.Q32One(), Y: fixed.Q32FromInt(2)}
	def.MaxForce = fixed.Q32FromInt(100)
	jointId := dbox2d.CreateMotorJoint(worldId, &def)

	fmt.Println(jointId.IsNull())
	// Output: false
}

func ExampleCreateMouseJoint() {
	worldId, groundId, boxId := exampleWorld()
	defer dbox2d.DestroyWorld(worldId)

	// The target is a world point; the joint pulls the box toward it.
	def := dbox2d.DefaultMouseJointDef()
	def.BodyIdA = groundId
	def.BodyIdB = boxId
	def.Target = dbox2d.Vec2{X: fixed.Q32One(), Y: fixed.Q32FromInt(2)}
	def.MaxForce = fixed.Q32FromInt(500)
	jointId := dbox2d.CreateMouseJoint(worldId, &def)

	fmt.Println(jointId.IsNull())
	// Output: false
}

func ExampleCreatePrismaticJoint() {
	worldId, groundId, boxId := exampleWorld()
	defer dbox2d.DestroyWorld(worldId)

	// The box slides along the vertical axis of the ground, between
	// zero and three units.
	def := dbox2d.DefaultPrismaticJointDef()
	def.BodyIdA = groundId
	def.BodyIdB = boxId
	def.LocalAxisA = dbox2d.Vec2{Y: fixed.Q32One()}
	def.EnableLimit = true
	def.LowerTranslation = fixed.Q32Zero()
	def.UpperTranslation = fixed.Q32FromInt(3)
	jointId := dbox2d.CreatePrismaticJoint(worldId, &def)

	fmt.Println(jointId.IsNull())
	// Output: false
}

func ExampleCreateRevoluteJoint() {
	worldId, groundId, boxId := exampleWorld()
	defer dbox2d.DestroyWorld(worldId)

	// A pendulum: the pivot is the ground origin, two units below the
	// box center. The angles are in turns; a quarter turn is 0.25.
	def := dbox2d.DefaultRevoluteJointDef()
	def.BodyIdA = groundId
	def.BodyIdB = boxId
	def.LocalAnchorB = dbox2d.Vec2{Y: fixed.Q32FromInt(-2)}
	def.EnableLimit = true
	def.LowerAngle = fixed.Q32MustParse("-0.25")
	def.UpperAngle = fixed.Q32MustParse("0.25")
	jointId := dbox2d.CreateRevoluteJoint(worldId, &def)

	fmt.Println(jointId.IsNull())
	// Output: false
}

func ExampleCreateWeldJoint() {
	worldId, groundId, boxId := exampleWorld()
	defer dbox2d.DestroyWorld(worldId)

	// A weld with a soft angular spring holds the box in place.
	def := dbox2d.DefaultWeldJointDef()
	def.BodyIdA = groundId
	def.BodyIdB = boxId
	def.LocalAnchorB = dbox2d.Vec2{Y: fixed.Q32FromInt(-2)}
	def.AngularHertz = fixed.Q32FromInt(2)
	def.AngularDampingRatio = fixed.Q32Half()
	jointId := dbox2d.CreateWeldJoint(worldId, &def)

	fmt.Println(jointId.IsNull())
	// Output: false
}

func ExampleCreateWheelJoint() {
	worldId, groundId, boxId := exampleWorld()
	defer dbox2d.DestroyWorld(worldId)

	// A suspension: the box rides the vertical axis on a spring and its
	// motor spins it at one turn per second.
	def := dbox2d.DefaultWheelJointDef()
	def.BodyIdA = groundId
	def.BodyIdB = boxId
	def.EnableMotor = true
	def.MotorSpeed = fixed.Q32One()
	def.MaxMotorTorque = fixed.Q32FromInt(10)
	jointId := dbox2d.CreateWheelJoint(worldId, &def)

	fmt.Println(jointId.IsNull())
	// Output: false
}
