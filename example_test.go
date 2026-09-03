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

func ExampleBodyId_SetType() {
	worldId, _, boxId := exampleWorld()
	defer dbox2d.DestroyWorld(worldId)

	// A dynamic box turned static stops falling.
	boxId.SetType(dbox2d.StaticBody)

	fmt.Println(boxId.GetType() == dbox2d.StaticBody)
	// Output: true
}

func ExampleBodyId_Disable() {
	worldId, _, boxId := exampleWorld()
	defer dbox2d.DestroyWorld(worldId)

	boxId.Disable()
	fmt.Println(boxId.IsEnabled())

	boxId.Enable()
	fmt.Println(boxId.IsEnabled())
	// Output:
	// false
	// true
}

func ExampleBodyId_SetTransform() {
	worldId, _, boxId := exampleWorld()
	defer dbox2d.DestroyWorld(worldId)

	boxId.SetTransform(dbox2d.Vec2{X: fixed.Q32One(), Y: fixed.Q32FromInt(3)}, dbox2d.RotIdentity())

	fmt.Println(boxId.GetPosition())
	// Output: {1 3}
}

func ExampleCreateChain() {
	worldId, groundId, _ := exampleWorld()
	defer dbox2d.DestroyWorld(worldId)

	// An open chain of four points builds two segments; the two end
	// segments extend with ghost vertices for smooth rolling contact.
	def := dbox2d.DefaultChainDef()
	def.Points = []dbox2d.Vec2{
		{X: fixed.Q32FromInt(0)},
		{X: fixed.Q32FromInt(1)},
		{X: fixed.Q32FromInt(2)},
		{X: fixed.Q32FromInt(3)},
		{X: fixed.Q32FromInt(4)},
	}
	chainId := dbox2d.CreateChain(groundId, &def)

	fmt.Println(chainId.GetSegmentCount())
	// Output: 2
}

func ExampleWorldId_GetSensorEvents() {
	worldDef := dbox2d.DefaultWorldDef()
	worldId := dbox2d.CreateWorld(&worldDef)
	defer dbox2d.DestroyWorld(worldId)

	sensorDef := dbox2d.DefaultBodyDef()
	sensorBodyId := dbox2d.CreateBody(worldId, &sensorDef)
	sensorShapeDef := dbox2d.DefaultShapeDef()
	sensorShapeDef.IsSensor = true
	sensorShapeDef.EnableSensorEvents = true
	sensorBox := dbox2d.MakeBox(fixed.Q32FromInt(2), fixed.Q32FromInt(2))
	dbox2d.CreatePolygonShape(sensorBodyId, &sensorShapeDef, &sensorBox)

	visitorDef := dbox2d.DefaultBodyDef()
	visitorDef.Type = dbox2d.DynamicBody
	visitorBodyId := dbox2d.CreateBody(worldId, &visitorDef)
	visitorShapeDef := dbox2d.DefaultShapeDef()
	visitorShapeDef.EnableSensorEvents = true
	visitorBox := dbox2d.MakeSquare(fixed.Q32Half())
	visitorShapeId := dbox2d.CreatePolygonShape(visitorBodyId, &visitorShapeDef, &visitorBox)

	dt := fixed.Q32One().Div(fixed.Q32FromInt(60))
	worldId.Step(dt, 4)
	begin := worldId.GetSensorEvents().BeginEvents
	fmt.Println(len(begin) == 1 && begin[0].VisitorShapeId == visitorShapeId)

	dbox2d.DestroyShape(visitorShapeId, false)
	worldId.Step(dt, 4)
	end := worldId.GetSensorEvents().EndEvents
	fmt.Println(len(end) == 1 && end[0].VisitorShapeId == visitorShapeId)
	// Output:
	// true
	// true
}

func ExampleSolvePlanes() {
	// A push limit of huge (the engine's rigid-contact bound) makes the
	// plane act as an unyielding wall: the tangential component of the
	// move survives untouched, and the normal component is pushed back
	// to just outside the plane, leaving a linear-slop margin.
	huge := fixed.Q32FromInt(100000)
	planes := []dbox2d.CollisionPlane{{
		Plane:     dbox2d.Plane{Normal: dbox2d.Vec2{Y: fixed.Q32One()}},
		PushLimit: huge,
	}}

	result := dbox2d.SolvePlanes(dbox2d.Vec2{X: fixed.Q32One(), Y: fixed.Q32One().Neg()}, planes)

	fmt.Println(result.Translation.X)
	fmt.Println(result.Translation.Y.Less(dbox2d.LinearSlop().Neg().Sub(dbox2d.LinearSlop())))
	// Output:
	// 1
	// false
}

func ExampleWorldId_Explode() {
	worldId, _, boxId := exampleWorld()
	defer dbox2d.DestroyWorld(worldId)

	def := dbox2d.DefaultExplosionDef()
	def.Position = boxId.GetPosition()
	def.Radius = fixed.Q32FromInt(4)
	def.Falloff = fixed.Q32FromInt(2)
	def.ImpulsePerLength = fixed.Q32FromInt(8)
	worldId.Explode(&def)

	// The box center coincides with the explosion center, so the
	// direction is undefined; the engine falls back to (1, 0) rather
	// than leaving the impulse with a zero direction.
	fmt.Println(boxId.GetLinearVelocity())
	// Output: {8 0}
}

func ExampleWorldId_SetCustomFilterCallback() {
	worldId, groundId, boxId := exampleWorld()
	defer dbox2d.DestroyWorld(worldId)

	// Rejecting every pair keeps the box from ever touching the ground.
	worldId.SetCustomFilterCallback(func(dbox2d.ShapeId, dbox2d.ShapeId) bool {
		return false
	})

	dt := fixed.Q32One().Div(fixed.Q32FromInt(60))
	for range 60 {
		worldId.Step(dt, 4)
	}

	fmt.Println(len(worldId.GetContactEvents().BeginEvents))
	fmt.Println(groundId.IsValid() && boxId.IsValid())
	// Output:
	// 0
	// true
}

func ExampleWorldId_SetPreSolveCallback() {
	worldId, _, boxId := exampleWorld()
	defer dbox2d.DestroyWorld(worldId)

	// Returning false disables the contact for the step, so the box
	// keeps falling through the ground instead of resting on it.
	worldId.SetPreSolveCallback(func(dbox2d.ShapeId, dbox2d.ShapeId, *dbox2d.Manifold) bool {
		return false
	})

	startY := boxId.GetPosition().Y
	dt := fixed.Q32One().Div(fixed.Q32FromInt(60))
	for range 60 {
		worldId.Step(dt, 4)
	}

	fmt.Println(boxId.GetPosition().Y.Less(startY))
	// Output: true
}

func ExampleBodyId_GetContactData() {
	worldDef := dbox2d.DefaultWorldDef()
	worldId := dbox2d.CreateWorld(&worldDef)
	defer dbox2d.DestroyWorld(worldId)

	groundDef := dbox2d.DefaultBodyDef()
	groundId := dbox2d.CreateBody(worldId, &groundDef)
	groundShapeDef := dbox2d.DefaultShapeDef()
	groundBox := dbox2d.MakeBox(fixed.Q32FromInt(5), fixed.Q32Half())
	dbox2d.CreatePolygonShape(groundId, &groundShapeDef, &groundBox)

	boxDef := dbox2d.DefaultBodyDef()
	boxDef.Type = dbox2d.DynamicBody
	boxDef.Position = dbox2d.Vec2{Y: fixed.Q32One()}
	boxId := dbox2d.CreateBody(worldId, &boxDef)
	boxShapeDef := dbox2d.DefaultShapeDef()
	box := dbox2d.MakeSquare(fixed.Q32Half())
	dbox2d.CreatePolygonShape(boxId, &boxShapeDef, &box)

	dt := fixed.Q32One().Div(fixed.Q32FromInt(60))
	for range 60 {
		worldId.Step(dt, 4)
	}

	data := make([]dbox2d.ContactData, boxId.GetContactCapacity())
	count := boxId.GetContactData(data)

	fmt.Println(count)
	fmt.Println(data[0].ShapeIdA.GetBody() == groundId || data[0].ShapeIdB.GetBody() == groundId)
	// Output:
	// 1
	// true
}

func ExampleJointId_SetCollideConnected() {
	worldId, groundId, boxId := exampleWorld()
	defer dbox2d.DestroyWorld(worldId)

	def := dbox2d.DefaultDistanceJointDef()
	def.BodyIdA = groundId
	def.BodyIdB = boxId
	def.Length = fixed.Q32FromInt(2)
	jointId := dbox2d.CreateDistanceJoint(worldId, &def)

	jointId.SetCollideConnected(true)

	fmt.Println(jointId.GetCollideConnected())
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
