package b2_test

import (
	"fmt"

	"github.com/dhannyell/dbox2d"
)

// exampleWorld builds a world with a static ground body and one dynamic
// box above it. The examples hang joints between the two.
func exampleWorld() (b2.WorldId, b2.BodyId, b2.BodyId) {
	worldDef := b2.DefaultWorldDef()
	worldId := b2.CreateWorld(&worldDef)

	groundDef := b2.DefaultBodyDef()
	groundId := b2.CreateBody(worldId, &groundDef)

	boxDef := b2.DefaultBodyDef()
	boxDef.Type = b2.DynamicBody
	boxDef.Position = b2.Vec2{Y: b2.QFromInt(2)}
	boxId := b2.CreateBody(worldId, &boxDef)
	shapeDef := b2.DefaultShapeDef()
	box := b2.MakeSquare(b2.QHalf())
	b2.CreatePolygonShape(boxId, &shapeDef, &box)

	return worldId, groundId, boxId
}

func ExampleCreateDistanceJoint() {
	worldId, groundId, boxId := exampleWorld()
	defer b2.DestroyWorld(worldId)

	// A rope of length two from the ground origin to the box center.
	def := b2.DefaultDistanceJointDef()
	def.BodyIdA = groundId
	def.BodyIdB = boxId
	def.Length = b2.QFromInt(2)
	jointId := b2.CreateDistanceJoint(worldId, &def)

	fmt.Println(jointId.IsNull())
	// Output: false
}

func ExampleWorldId_SetFrictionCallback() {
	worldDef := b2.DefaultWorldDef()
	worldId := b2.CreateWorld(&worldDef)
	defer b2.DestroyWorld(worldId)

	worldId.SetFrictionCallback(func(frictionA b2.Q, _ int, frictionB b2.Q, _ int) b2.Q {
		return frictionA.Add(frictionB).Div(b2.QFromInt(2))
	})

	fmt.Println(worldId.IsValid())
	// Output: true
}

func ExampleBodyId_SetType() {
	worldId, _, boxId := exampleWorld()
	defer b2.DestroyWorld(worldId)

	// A dynamic box turned static stops falling.
	boxId.SetType(b2.StaticBody)

	fmt.Println(boxId.GetType() == b2.StaticBody)
	// Output: true
}

func ExampleBodyId_Disable() {
	worldId, _, boxId := exampleWorld()
	defer b2.DestroyWorld(worldId)

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
	defer b2.DestroyWorld(worldId)

	boxId.SetTransform(b2.Vec2{X: b2.QOne(), Y: b2.QFromInt(3)}, b2.RotIdentity())

	fmt.Println(boxId.GetPosition())
	// Output: {1 3}
}

func ExampleCreateChain() {
	worldId, groundId, _ := exampleWorld()
	defer b2.DestroyWorld(worldId)

	// An open chain of four points builds two segments; the two end
	// segments extend with ghost vertices for smooth rolling contact.
	def := b2.DefaultChainDef()
	def.Points = []b2.Vec2{
		{X: b2.QFromInt(0)},
		{X: b2.QFromInt(1)},
		{X: b2.QFromInt(2)},
		{X: b2.QFromInt(3)},
		{X: b2.QFromInt(4)},
	}
	chainId := b2.CreateChain(groundId, &def)

	fmt.Println(chainId.GetSegmentCount())
	// Output: 2
}

func ExampleWorldId_Draw() {
	worldId, _, _ := exampleWorld()
	defer b2.DestroyWorld(worldId)

	// Every callback defaults to a no-op; wire up only the shapes callbacks
	// used here and count how many shapes the world hands back.
	shapeCount := 0
	draw := b2.DefaultDebugDraw()
	draw.DrawShapes = true
	draw.DrawSolidPolygon = func(b2.Transform, []b2.Vec2, b2.Q, b2.HexColor) { shapeCount++ }

	worldId.Draw(&draw)

	fmt.Println(shapeCount)
	// Output: 1
}

func ExampleWorldId_GetSensorEvents() {
	worldDef := b2.DefaultWorldDef()
	worldId := b2.CreateWorld(&worldDef)
	defer b2.DestroyWorld(worldId)

	sensorDef := b2.DefaultBodyDef()
	sensorBodyId := b2.CreateBody(worldId, &sensorDef)
	sensorShapeDef := b2.DefaultShapeDef()
	sensorShapeDef.IsSensor = true
	sensorShapeDef.EnableSensorEvents = true
	sensorBox := b2.MakeBox(b2.QFromInt(2), b2.QFromInt(2))
	b2.CreatePolygonShape(sensorBodyId, &sensorShapeDef, &sensorBox)

	visitorDef := b2.DefaultBodyDef()
	visitorDef.Type = b2.DynamicBody
	visitorBodyId := b2.CreateBody(worldId, &visitorDef)
	visitorShapeDef := b2.DefaultShapeDef()
	visitorShapeDef.EnableSensorEvents = true
	visitorBox := b2.MakeSquare(b2.QHalf())
	visitorShapeId := b2.CreatePolygonShape(visitorBodyId, &visitorShapeDef, &visitorBox)

	dt := b2.QOne().Div(b2.QFromInt(60))
	worldId.Step(dt, 4)
	begin := worldId.GetSensorEvents().BeginEvents
	fmt.Println(len(begin) == 1 && begin[0].VisitorShapeId == visitorShapeId)

	b2.DestroyShape(visitorShapeId, false)
	worldId.Step(dt, 4)
	end := worldId.GetSensorEvents().EndEvents
	fmt.Println(len(end) == 1 && end[0].VisitorShapeId == visitorShapeId)
	// Output:
	// true
	// true
}

func ExampleSolvePlanes() {
	// A push limit of b2.Huge (the engine's rigid-contact bound) makes
	// the plane act as an unyielding wall: the tangential component of the
	// move survives untouched, and the normal component is pushed back
	// to just outside the plane, leaving a linear-slop margin.
	planes := []b2.CollisionPlane{{
		Plane:     b2.Plane{Normal: b2.Vec2{Y: b2.QOne()}},
		PushLimit: b2.Huge,
	}}

	result := b2.SolvePlanes(b2.Vec2{X: b2.QOne(), Y: b2.QOne().Neg()}, planes)

	fmt.Println(result.Translation.X)
	fmt.Println(result.Translation.Y.Less(b2.LinearSlop().Neg().Sub(b2.LinearSlop())))
	// Output:
	// 1
	// false
}

func ExampleWorldId_Explode() {
	worldId, _, boxId := exampleWorld()
	defer b2.DestroyWorld(worldId)

	def := b2.DefaultExplosionDef()
	def.Position = boxId.GetPosition()
	def.Radius = b2.QFromInt(4)
	def.Falloff = b2.QFromInt(2)
	def.ImpulsePerLength = b2.QFromInt(8)
	worldId.Explode(&def)

	// The box center coincides with the explosion center, so the
	// direction is undefined; the engine falls back to (1, 0) rather
	// than leaving the impulse with a zero direction.
	fmt.Println(boxId.GetLinearVelocity())
	// Output: {8 0}
}

func ExampleWorldId_SetCustomFilterCallback() {
	worldId, groundId, boxId := exampleWorld()
	defer b2.DestroyWorld(worldId)

	// Rejecting every pair keeps the box from ever touching the ground.
	worldId.SetCustomFilterCallback(func(b2.ShapeId, b2.ShapeId) bool {
		return false
	})

	dt := b2.QOne().Div(b2.QFromInt(60))
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
	defer b2.DestroyWorld(worldId)

	// Returning false disables the contact for the step, so the box
	// keeps falling through the ground instead of resting on it.
	worldId.SetPreSolveCallback(func(b2.ShapeId, b2.ShapeId, *b2.Manifold) bool {
		return false
	})

	startY := boxId.GetPosition().Y
	dt := b2.QOne().Div(b2.QFromInt(60))
	for range 60 {
		worldId.Step(dt, 4)
	}

	fmt.Println(boxId.GetPosition().Y.Less(startY))
	// Output: true
}

func ExampleBodyId_GetContactData() {
	worldDef := b2.DefaultWorldDef()
	worldId := b2.CreateWorld(&worldDef)
	defer b2.DestroyWorld(worldId)

	groundDef := b2.DefaultBodyDef()
	groundId := b2.CreateBody(worldId, &groundDef)
	groundShapeDef := b2.DefaultShapeDef()
	groundBox := b2.MakeBox(b2.QFromInt(5), b2.QHalf())
	b2.CreatePolygonShape(groundId, &groundShapeDef, &groundBox)

	boxDef := b2.DefaultBodyDef()
	boxDef.Type = b2.DynamicBody
	boxDef.Position = b2.Vec2{Y: b2.QOne()}
	boxId := b2.CreateBody(worldId, &boxDef)
	boxShapeDef := b2.DefaultShapeDef()
	box := b2.MakeSquare(b2.QHalf())
	b2.CreatePolygonShape(boxId, &boxShapeDef, &box)

	dt := b2.QOne().Div(b2.QFromInt(60))
	for range 60 {
		worldId.Step(dt, 4)
	}

	data := make([]b2.ContactData, boxId.GetContactCapacity())
	count := boxId.GetContactData(data)

	fmt.Println(count)
	fmt.Println(data[0].ShapeIdA.GetBody() == groundId || data[0].ShapeIdB.GetBody() == groundId)
	// Output:
	// 1
	// true
}

func ExampleJointId_SetCollideConnected() {
	worldId, groundId, boxId := exampleWorld()
	defer b2.DestroyWorld(worldId)

	def := b2.DefaultDistanceJointDef()
	def.BodyIdA = groundId
	def.BodyIdB = boxId
	def.Length = b2.QFromInt(2)
	jointId := b2.CreateDistanceJoint(worldId, &def)

	jointId.SetCollideConnected(true)

	fmt.Println(jointId.GetCollideConnected())
	// Output: true
}

func ExampleCreateFilterJoint() {
	worldId, groundId, boxId := exampleWorld()
	defer b2.DestroyWorld(worldId)

	// The box falls through the ground; the filter joint disables the
	// collision between the two bodies.
	def := b2.DefaultFilterJointDef()
	def.BodyIdA = groundId
	def.BodyIdB = boxId
	jointId := b2.CreateFilterJoint(worldId, &def)

	fmt.Println(jointId.IsNull())
	// Output: false
}

func ExampleCreateMotorJoint() {
	worldId, groundId, boxId := exampleWorld()
	defer b2.DestroyWorld(worldId)

	// The motor drives the box toward an offset of (1, 2) from the ground.
	def := b2.DefaultMotorJointDef()
	def.BodyIdA = groundId
	def.BodyIdB = boxId
	def.LinearOffset = b2.Vec2{X: b2.QOne(), Y: b2.QFromInt(2)}
	def.MaxForce = b2.QFromInt(100)
	jointId := b2.CreateMotorJoint(worldId, &def)

	fmt.Println(jointId.IsNull())
	// Output: false
}

func ExampleCreateMouseJoint() {
	worldId, groundId, boxId := exampleWorld()
	defer b2.DestroyWorld(worldId)

	// The target is a world point; the joint pulls the box toward it.
	def := b2.DefaultMouseJointDef()
	def.BodyIdA = groundId
	def.BodyIdB = boxId
	def.Target = b2.Vec2{X: b2.QOne(), Y: b2.QFromInt(2)}
	def.MaxForce = b2.QFromInt(500)
	jointId := b2.CreateMouseJoint(worldId, &def)

	fmt.Println(jointId.IsNull())
	// Output: false
}

func ExampleCreatePrismaticJoint() {
	worldId, groundId, boxId := exampleWorld()
	defer b2.DestroyWorld(worldId)

	// The box slides along the vertical axis of the ground, between
	// zero and three units.
	def := b2.DefaultPrismaticJointDef()
	def.BodyIdA = groundId
	def.BodyIdB = boxId
	def.LocalAxisA = b2.Vec2{Y: b2.QOne()}
	def.EnableLimit = true
	def.LowerTranslation = b2.QZero()
	def.UpperTranslation = b2.QFromInt(3)
	jointId := b2.CreatePrismaticJoint(worldId, &def)

	fmt.Println(jointId.IsNull())
	// Output: false
}

func ExampleCreateRevoluteJoint() {
	worldId, groundId, boxId := exampleWorld()
	defer b2.DestroyWorld(worldId)

	// A pendulum: the pivot is the ground origin, two units below the
	// box center. The angles are in turns; a quarter turn is 0.25.
	def := b2.DefaultRevoluteJointDef()
	def.BodyIdA = groundId
	def.BodyIdB = boxId
	def.LocalAnchorB = b2.Vec2{Y: b2.QFromInt(-2)}
	def.EnableLimit = true
	def.LowerAngle = b2.QMustParse("-0.25")
	def.UpperAngle = b2.QMustParse("0.25")
	jointId := b2.CreateRevoluteJoint(worldId, &def)

	fmt.Println(jointId.IsNull())
	// Output: false
}

func ExampleCreateWeldJoint() {
	worldId, groundId, boxId := exampleWorld()
	defer b2.DestroyWorld(worldId)

	// A weld with a soft angular spring holds the box in place.
	def := b2.DefaultWeldJointDef()
	def.BodyIdA = groundId
	def.BodyIdB = boxId
	def.LocalAnchorB = b2.Vec2{Y: b2.QFromInt(-2)}
	def.AngularHertz = b2.QFromInt(2)
	def.AngularDampingRatio = b2.QHalf()
	jointId := b2.CreateWeldJoint(worldId, &def)

	fmt.Println(jointId.IsNull())
	// Output: false
}

func ExampleCreateWheelJoint() {
	worldId, groundId, boxId := exampleWorld()
	defer b2.DestroyWorld(worldId)

	// A suspension: the box rides the vertical axis on a spring and its
	// motor spins it at one turn per second.
	def := b2.DefaultWheelJointDef()
	def.BodyIdA = groundId
	def.BodyIdB = boxId
	def.EnableMotor = true
	def.MotorSpeed = b2.QOne()
	def.MaxMotorTorque = b2.QFromInt(10)
	jointId := b2.CreateWheelJoint(worldId, &def)

	fmt.Println(jointId.IsNull())
	// Output: false
}
