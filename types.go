package dbox2d

import (
	"math"

	"github.com/dhannyell/fixed"
)

// Default filter bits. upstream B2_DEFAULT_CATEGORY_BITS, B2_DEFAULT_MASK_BITS
const (
	DefaultCategoryBits uint64 = 1
	DefaultMaskBits     uint64 = math.MaxUint64
)

// WorldDef configures the creation of a world.
// Initialize it with DefaultWorldDef.
type WorldDef struct {
	// Gravity vector. The world has no up vector.
	Gravity Vec2

	// Restitution speed threshold, usually in meters per second. Collisions
	// above this speed have restitution applied, so they bounce.
	RestitutionThreshold Q

	// Threshold speed for hit events, usually in meters per second.
	HitEventThreshold Q

	// Contact stiffness in cycles per second. Increasing this increases the
	// speed of overlap recovery, but can introduce jitter.
	ContactHertz Q

	// Contact bounciness, non-dimensional. Decreasing this speeds up overlap
	// recovery, with a more energetic resolution as the trade-off.
	ContactDampingRatio Q

	// A cap on the overlap resolution speed, usually in meters per second.
	// The hertz and the damping ratio set the speed itself.
	MaxContactPushSpeed Q

	// Maximum linear speed, usually in meters per second.
	MaximumLinearSpeed Q

	// FrictionCallback customizes friction mixing for new contacts.
	FrictionCallback FrictionCallback

	// RestitutionCallback customizes restitution mixing for new contacts.
	RestitutionCallback RestitutionCallback

	// Deferred: the task system fields of the reference.

	// Enable sleeping to improve performance.
	EnableSleep bool

	// Enable continuous collision.
	EnableContinuous bool

	// Application data attached to the world.
	UserData any

	// internalValue proves that DefaultWorldDef ran. upstream internalValue
	internalValue int
}

// Counters reports world sizes and solver storage usage for diagnostics.
type Counters struct {
	BodyCount, ShapeCount, ContactCount, JointCount, IslandCount, StackUsed, StaticTreeHeight, TreeHeight int
	ColorCounts                                                                                           [graphColorCount]int
}

// FrictionCallback mixes the friction values of two shapes.
type FrictionCallback func(frictionA Q, userMaterialIdA int, frictionB Q, userMaterialIdB int) Q

// RestitutionCallback mixes the restitution values of two shapes.
type RestitutionCallback func(restitutionA Q, userMaterialIdA int, restitutionB Q, userMaterialIdB int) Q

// CustomFilterFcn decides whether two shapes may collide. It corresponds to
// b2CustomFilterFcn; return false to reject the pair.
type CustomFilterFcn func(shapeIdA, shapeIdB ShapeId) bool

// PreSolveFcn inspects a contact manifold before the solver runs. It
// corresponds to b2PreSolveFcn; return false to disable the contact this step.
type PreSolveFcn func(shapeIdA, shapeIdB ShapeId, manifold *Manifold) bool

// DefaultWorldDef returns the default world definition.
func DefaultWorldDef() WorldDef {
	return WorldDef{
		Gravity:              Vec2{Y: fixed.Q32FromInt(-10)},
		RestitutionThreshold: fixed.Q32One(),
		HitEventThreshold:    fixed.Q32One(),
		ContactHertz:         fixed.Q32FromInt(30),
		ContactDampingRatio:  fixed.Q32FromInt(10),
		MaxContactPushSpeed:  fixed.Q32FromInt(3),
		// 400 meters per second, faster than the speed of sound
		MaximumLinearSpeed: fixed.Q32FromInt(400),
		EnableSleep:        true,
		EnableContinuous:   true,
		internalValue:      secretCookie,
	}
}

// BodyType selects how the solver treats a body.
type BodyType int

const (
	// StaticBody has zero mass and zero velocity. An application may move it
	// manually.
	StaticBody BodyType = 0

	// KinematicBody has zero mass. The application sets the velocity and the
	// solver moves it.
	KinematicBody BodyType = 1

	// DynamicBody has positive mass. Forces determine the velocity and the
	// solver moves it.
	DynamicBody BodyType = 2

	// BodyTypeCount is the number of body types.
	BodyTypeCount = 3
)

// BodyDef holds the data to construct a rigid body. It is a temporary bundle
// of creation parameters; shapes come after construction.
// Initialize it with DefaultBodyDef.
type BodyDef struct {
	// The body type: static, kinematic or dynamic.
	Type BodyType

	// The initial world position. Create a body at its final position:
	// creating it at the origin and moving it costs nearly twice as much.
	Position Vec2

	// The initial world rotation. Use MakeRot for an angle in turns.
	Rotation Rot

	// The initial linear velocity of the body origin, usually in meters per
	// second.
	LinearVelocity Vec2

	// The initial angular velocity, in turns per second. The reference uses
	// radians per second; an angle is a turn in this package.
	AngularVelocity Q

	// Linear damping reduces the linear velocity. A value larger than 1 is
	// valid but grows sensitive to the time step. Usually undesirable,
	// because it makes bodies move as if they float.
	LinearDamping Q

	// Angular damping reduces the angular velocity. A value larger than 1 is
	// valid but grows sensitive to the time step.
	AngularDamping Q

	// Scale of the gravity on this body, non-dimensional.
	GravityScale Q

	// Sleep speed threshold, default 0.05 meters per second.
	SleepThreshold Q

	// Optional body name for debugging, up to 31 bytes.
	Name string

	// Application data attached to the body.
	UserData any

	// Set this to false if the body must never fall asleep.
	EnableSleep bool

	// Whether the body starts awake.
	IsAwake bool

	// Prevent the body from rotating. Useful for characters.
	FixedRotation bool

	// Treat the body as a high speed object that performs continuous
	// collision against dynamic and kinematic bodies, but not against other
	// bullets. Use sparingly: bullets solve no general dynamic-versus-dynamic
	// case and may interfere with joints.
	IsBullet bool

	// A disabled body does not move and does not collide.
	IsEnabled bool

	// Let the body bypass the rotational speed limit. Only for circular
	// bodies, like wheels.
	AllowFastRotation bool

	// internalValue proves that DefaultBodyDef ran. upstream internalValue
	internalValue int
}

// DefaultBodyDef returns the default body definition.
func DefaultBodyDef() BodyDef {
	return BodyDef{
		Type:           StaticBody,
		Rotation:       fixed.RotIdentity(),
		SleepThreshold: fixed.Q32MustParse("0.05"),
		GravityScale:   fixed.Q32One(),
		EnableSleep:    true,
		IsAwake:        true,
		IsEnabled:      true,
		internalValue:  secretCookie,
	}
}

// Filter controls collision between shapes, and between queries and shapes.
type Filter struct {
	// The collision category bits of this shape. Normally one bit per
	// application object type.
	CategoryBits uint64

	// The categories that this shape accepts for collision.
	MaskBits uint64

	// Shapes in the same negative group never collide; in the same positive
	// group they always collide. Zero has no effect. A non-zero group wins
	// against the mask bits.
	GroupIndex int
}

// DefaultFilter returns the default filter: category 1, every mask bit set,
// no group.
func DefaultFilter() Filter {
	return Filter{CategoryBits: DefaultCategoryBits, MaskBits: DefaultMaskBits}
}

// QueryFilter controls collision between a query and the shapes it visits.
type QueryFilter struct {
	// The collision category bits of this query. Normally one bit.
	CategoryBits uint64

	// The shape categories that this query accepts for collision.
	MaskBits uint64
}

// DefaultQueryFilter returns the default query filter: category 1, every mask
// bit set.
func DefaultQueryFilter() QueryFilter {
	return QueryFilter{CategoryBits: DefaultCategoryBits, MaskBits: DefaultMaskBits}
}

// ExplosionDef configures an explosion. It corresponds to b2ExplosionDef in
// include/box2d/types.h.
type ExplosionDef struct {
	// MaskBits filters the shapes the explosion reaches.
	MaskBits uint64

	// Position is the center of the explosion in world space.
	Position Vec2

	// Radius is the full-impulse radius of the explosion.
	Radius Q

	// Falloff is the distance beyond Radius where the impulse fades to zero.
	Falloff Q

	// ImpulsePerLength scales the impulse by the perimeter facing the
	// explosion. It may be negative for an implosion.
	ImpulsePerLength Q
}

// ShapeType identifies the geometry of a shape.
type ShapeType int

const (
	// CircleShape is a circle with an offset.
	CircleShape ShapeType = iota

	// CapsuleShape is an extruded circle.
	CapsuleShape

	// SegmentShape is a line segment.
	SegmentShape

	// PolygonShape is a convex polygon.
	PolygonShape

	// ChainSegmentShape is a line segment owned by a chain shape.
	ChainSegmentShape

	// ShapeTypeCount is the number of shape types.
	ShapeTypeCount
)

// SurfaceMaterial holds the surface properties of a shape.
type SurfaceMaterial struct {
	// The Coulomb friction coefficient, usually in the range [0, 1].
	Friction Q

	// The coefficient of restitution, usually in the range [0, 1].
	Restitution Q

	// The rolling resistance, usually in the range [0, 1].
	RollingResistance Q

	// The tangent speed, for conveyor belts.
	TangentSpeed Q

	// Application material identifier, returned with query results. The
	// world does not use it.
	UserMaterialId int

	// Custom debug draw color.
	CustomColor uint32
}

// DefaultSurfaceMaterial returns the default material: friction 0.6.
func DefaultSurfaceMaterial() SurfaceMaterial {
	return SurfaceMaterial{Friction: fixed.Q32MustParse("0.6")}
}

// ShapeDef holds the data to create a shape. It is a temporary bundle of
// creation parameters; one definition may create many shapes.
// Initialize it with DefaultShapeDef.
type ShapeDef struct {
	// Application data attached to the shape.
	UserData any

	// The surface material of this shape.
	Material SurfaceMaterial

	// The density, usually in kg/m^2. It stays out of the material because
	// it describes the interior, which may be hollow.
	Density Q

	// Collision filtering data.
	Filter Filter

	// A sensor generates overlap events and never a collision response.
	IsSensor bool

	// Enable sensor events for this shape. False by default, even for
	// sensors.
	EnableSensorEvents bool

	// Enable contact events. Only for kinematic and dynamic bodies. Ignored
	// for sensors.
	EnableContactEvents bool

	// Enable hit events. Only for kinematic and dynamic bodies. Ignored for
	// sensors.
	EnableHitEvents bool

	// Enable pre-solve contact events. Only for dynamic bodies. Ignored for
	// sensors.
	EnablePreSolveEvents bool

	// A new shape scans the environment for collision on the next step. This
	// flag skips the scan on a static body, where the scan is the dominant
	// creation cost. Dynamic and kinematic shapes ignore the flag.
	InvokeContactCreation bool

	// Update the mass of the body when this shape is created. Default true.
	UpdateBodyMass bool

	// internalValue proves that DefaultShapeDef ran. upstream internalValue
	internalValue int
}

// DefaultShapeDef returns the default shape definition.
func DefaultShapeDef() ShapeDef {
	return ShapeDef{
		Material:              DefaultSurfaceMaterial(),
		Density:               fixed.Q32One(),
		Filter:                DefaultFilter(),
		InvokeContactCreation: true,
		UpdateBodyMass:        true,
		internalValue:         secretCookie,
	}
}

// ChainDef configures a chain shape. It corresponds to b2ChainDef.
type ChainDef struct {
	// Application data attached to the chain's segments.
	UserData any

	// An array of at least four points. The points are copied during creation.
	Points []Vec2

	// One material for every segment, or one material shared by all segments.
	Materials []SurfaceMaterial

	// Collision filtering data.
	Filter Filter

	// Enable sensors to detect this chain.
	EnableSensorEvents bool

	// Connect the last point back to the first point.
	IsLoop bool

	// internalValue proves that DefaultChainDef ran. upstream internalValue
	internalValue int
}

// DefaultChainDef returns the default chain definition. It corresponds to
// b2DefaultChainDef.
func DefaultChainDef() ChainDef {
	return ChainDef{
		Materials:     []SurfaceMaterial{DefaultSurfaceMaterial()},
		Filter:        DefaultFilter(),
		internalValue: secretCookie,
	}
}

// ContactData reports the manifold of a touching contact on a body. It
// corresponds to b2ContactData in include/box2d/types.h.
type ContactData struct {
	// ShapeIdA is the first shape in the contact.
	ShapeIdA ShapeId

	// ShapeIdB is the second shape in the contact.
	ShapeIdB ShapeId

	// Manifold is the current contact manifold.
	Manifold Manifold
}

// ContactBeginTouchEvent reports that two shapes started to touch.
type ContactBeginTouchEvent struct {
	// ShapeIdA is the first shape.
	ShapeIdA ShapeId

	// ShapeIdB is the second shape.
	ShapeIdB ShapeId

	// Manifold is the initial contact manifold. The world records it
	// before the solver runs, so every impulse is zero.
	Manifold Manifold
}

// ContactEndTouchEvent reports that two shapes stopped touching. Any
// action that destroys a contact between steps also reports one, such as
// a transform change, a body or shape destroy, or a body type change.
type ContactEndTouchEvent struct {
	// ShapeIdA is the first shape. It may be destroyed; see IsValid.
	ShapeIdA ShapeId

	// ShapeIdB is the second shape. It may be destroyed; see IsValid.
	ShapeIdB ShapeId
}

// ContactHitEvent reports that two shapes collided faster than the hit
// event threshold. A speculative contact with a confirmed impulse also
// reports one.
type ContactHitEvent struct {
	// ShapeIdA is the first shape.
	ShapeIdA ShapeId

	// ShapeIdB is the second shape.
	ShapeIdB ShapeId

	// Point is where the shapes hit at the start of the step. It is the
	// mid-point between the two surfaces.
	Point Vec2

	// Normal points from shape A to shape B.
	Normal Vec2

	// ApproachSpeed is the closing speed of the shapes. It is positive.
	ApproachSpeed Q
}

// ContactEvents holds the contact events of the last step. The slices
// stay valid until the next step; a body or shape destroy may make an
// id in them stale.
type ContactEvents struct {
	BeginEvents []ContactBeginTouchEvent
	EndEvents   []ContactEndTouchEvent
	HitEvents   []ContactHitEvent
}

// SensorBeginTouchEvent reports that a shape started to overlap a sensor.
// It corresponds to b2SensorBeginTouchEvent in include/box2d/types.h.
type SensorBeginTouchEvent struct {
	SensorShapeId, VisitorShapeId ShapeId
}

// SensorEndTouchEvent reports that a shape stopped overlapping a sensor.
// It corresponds to b2SensorEndTouchEvent in include/box2d/types.h.
type SensorEndTouchEvent struct {
	SensorShapeId, VisitorShapeId ShapeId
}

// SensorEvents holds the sensor events of the last step. The slices stay
// valid until the next step. It corresponds to b2SensorEvents in
// include/box2d/types.h.
type SensorEvents struct {
	BeginEvents []SensorBeginTouchEvent
	EndEvents   []SensorEndTouchEvent
}

// BodyMoveEvent reports a body that the simulation moved. A body that the
// user moves does not report one. FellAsleep tells the application that
// it can sleep the object of the body. With sleep disabled, every dynamic
// and kinematic body reports on every step.
type BodyMoveEvent struct {
	Transform  Transform
	BodyId     BodyId
	UserData   any
	FellAsleep bool
}

// BodyEvents holds the body events of the last step. The slice stays
// valid until the next step; a body destroy makes its entry stale.
type BodyEvents struct {
	MoveEvents []BodyMoveEvent
}

// OverlapResultFcn receives each shape that an overlap query finds.
// Return false to stop the walk of the current tree; the query goes on
// with the next body type, as the reference does. It corresponds to
// b2OverlapResultFcn in include/box2d/types.h; the closure carries the
// context.
type OverlapResultFcn func(shapeId ShapeId) bool

// CastResultFcn receives each hit of a ray cast. Return -1 to skip the
// shape, 0 to stop, the fraction to clip the ray for the closest hit, or
// 1 to continue. It corresponds to b2CastResultFcn in
// include/box2d/types.h; the closure carries the context.
type CastResultFcn func(shapeId ShapeId, point, normal Vec2, fraction Q) Q

// PlaneResultFcn receives each collision plane a mover query finds.
// Return true to keep gathering planes. It corresponds to
// b2PlaneResultFcn in include/box2d/types.h; the closure carries the
// context.
type PlaneResultFcn func(shapeId ShapeId, result *PlaneResult) bool

// RayResult is the answer of CastRayClosest. On an initial overlap the
// fraction and the normal are zero and the point is an arbitrary point of
// the overlap region. It corresponds to b2RayResult in
// include/box2d/types.h.
type RayResult struct {
	ShapeId    ShapeId
	Point      Vec2
	Normal     Vec2
	Fraction   Q
	NodeVisits int
	LeafVisits int
	Hit        bool
}

// JointType selects the joint of a joint definition.
type JointType int

// Joint types. upstream b2JointType
const (
	DistanceJoint JointType = iota
	FilterJoint
	MotorJoint
	MouseJoint
	PrismaticJoint
	RevoluteJoint
	WeldJoint
	WheelJoint
)

// DistanceJointDef configures a distance joint: a point on each body kept
// at a fixed length, or within a length range. Initialize it with
// DefaultDistanceJointDef.
type DistanceJointDef struct {
	// The first attached body.
	BodyIdA BodyId

	// The second attached body.
	BodyIdB BodyId

	// The local anchor point relative to the origin of body A.
	LocalAnchorA Vec2

	// The local anchor point relative to the origin of body B.
	LocalAnchorB Vec2

	// The rest length of the joint. Clamped to a stable minimum value.
	Length Q

	// Enable the distance constraint to behave like a spring. If false the
	// distance joint is rigid, otherwise the limit and the motor apply.
	EnableSpring bool

	// The spring linear stiffness in cycles per second.
	Hertz Q

	// The spring linear damping ratio, non-dimensional.
	DampingRatio Q

	// Enable and disable the joint limit.
	EnableLimit bool

	// The minimum length. Clamped to a stable minimum value.
	MinLength Q

	// The maximum length. Must be greater than or equal to the minimum
	// length.
	MaxLength Q

	// Enable and disable the joint motor.
	EnableMotor bool

	// The maximum motor force, usually in newtons.
	MaxMotorForce Q

	// The desired motor speed, usually in meters per second.
	MotorSpeed Q

	// Set this flag to true if the attached bodies should collide.
	CollideConnected bool

	// Application data attached to the joint.
	UserData any

	// internalValue proves that DefaultDistanceJointDef ran. upstream
	// internalValue
	internalValue int
}

// DefaultDistanceJointDef returns the default distance joint definition.
func DefaultDistanceJointDef() DistanceJointDef {
	return DistanceJointDef{
		Length:        fixed.Q32One(),
		MaxLength:     Huge,
		internalValue: secretCookie,
	}
}

// MotorJointDef configures a motor joint: it drives the relative
// transform of two bodies toward an offset. Initialize it with
// DefaultMotorJointDef.
type MotorJointDef struct {
	// The first attached body.
	BodyIdA BodyId

	// The second attached body.
	BodyIdB BodyId

	// Position of body B minus the position of body A, in the frame of
	// body A, in meters.
	LinearOffset Vec2

	// The angle of body B minus the angle of body A, in turns.
	AngularOffset Q

	// The maximum motor force in newtons.
	MaxForce Q

	// The maximum motor torque in newton-meters.
	MaxTorque Q

	// Position correction factor in the range [0, 1].
	CorrectionFactor Q

	// Set this flag to true if the attached bodies should collide.
	CollideConnected bool

	// Application data attached to the joint.
	UserData any

	// internalValue proves that DefaultMotorJointDef ran. upstream
	// internalValue
	internalValue int
}

// DefaultMotorJointDef returns the default motor joint definition.
func DefaultMotorJointDef() MotorJointDef {
	return MotorJointDef{
		MaxForce:         fixed.Q32One(),
		MaxTorque:        fixed.Q32One(),
		CorrectionFactor: fixed.Q32MustParse("0.3"),
		internalValue:    secretCookie,
	}
}

// MouseJointDef configures a mouse joint: it drags a point on body B
// toward a world target with a spring. Body A is a ground reference and
// gets no reaction. Initialize it with DefaultMouseJointDef.
type MouseJointDef struct {
	// The first attached body. It is unused here, but it plays a role in
	// the solver order.
	BodyIdA BodyId

	// The second attached body.
	BodyIdB BodyId

	// The initial target point in world space.
	Target Vec2

	// Stiffness in cycles per second.
	Hertz Q

	// Damping ratio, non-dimensional.
	DampingRatio Q

	// Maximum force, usually in newtons.
	MaxForce Q

	// Set this flag to true if the attached bodies should collide.
	CollideConnected bool

	// Application data attached to the joint.
	UserData any

	// internalValue proves that DefaultMouseJointDef ran. upstream
	// internalValue
	internalValue int
}

// DefaultMouseJointDef returns the default mouse joint definition.
func DefaultMouseJointDef() MouseJointDef {
	return MouseJointDef{
		Hertz:         fixed.Q32FromInt(4),
		DampingRatio:  fixed.Q32One(),
		MaxForce:      fixed.Q32One(),
		internalValue: secretCookie,
	}
}

// FilterJointDef configures a filter joint: it only disables collision
// between two bodies. Initialize it with DefaultFilterJointDef.
type FilterJointDef struct {
	// The first attached body.
	BodyIdA BodyId

	// The second attached body.
	BodyIdB BodyId

	// Application data attached to the joint.
	UserData any

	// internalValue proves that DefaultFilterJointDef ran. upstream
	// internalValue
	internalValue int
}

// DefaultFilterJointDef returns the default filter joint definition.
func DefaultFilterJointDef() FilterJointDef {
	return FilterJointDef{internalValue: secretCookie}
}

// PrismaticJointDef configures a prismatic joint: body B translates along
// an axis fixed in body A, with no relative rotation. Initialize it with
// DefaultPrismaticJointDef.
type PrismaticJointDef struct {
	// The first attached body.
	BodyIdA BodyId

	// The second attached body.
	BodyIdB BodyId

	// The local anchor point relative to the origin of body A.
	LocalAnchorA Vec2

	// The local anchor point relative to the origin of body B.
	LocalAnchorB Vec2

	// The local translation unit axis in body A.
	LocalAxisA Vec2

	// The constrained angle between the bodies: angle of B minus angle of
	// A, in turns.
	ReferenceAngle Q

	// The target translation for the joint in meters. The spring-damper
	// pulls toward this translation.
	TargetTranslation Q

	// Enable a linear spring along the prismatic joint axis.
	EnableSpring bool

	// The spring stiffness in cycles per second.
	Hertz Q

	// The spring damping ratio, non-dimensional.
	DampingRatio Q

	// Enable and disable the joint limit.
	EnableLimit bool

	// The lower translation limit.
	LowerTranslation Q

	// The upper translation limit.
	UpperTranslation Q

	// Enable and disable the joint motor.
	EnableMotor bool

	// The maximum motor force, usually in newtons.
	MaxMotorForce Q

	// The desired motor speed, usually in meters per second.
	MotorSpeed Q

	// Set this flag to true if the attached bodies should collide.
	CollideConnected bool

	// Application data attached to the joint.
	UserData any

	// internalValue proves that DefaultPrismaticJointDef ran. upstream
	// internalValue
	internalValue int
}

// DefaultPrismaticJointDef returns the default prismatic joint definition.
func DefaultPrismaticJointDef() PrismaticJointDef {
	return PrismaticJointDef{
		LocalAxisA:    Vec2{X: fixed.Q32One()},
		internalValue: secretCookie,
	}
}

// RevoluteJointDef configures a revolute joint: a shared anchor point
// with free relative rotation. Initialize it with DefaultRevoluteJointDef.
type RevoluteJointDef struct {
	// The first attached body.
	BodyIdA BodyId

	// The second attached body.
	BodyIdB BodyId

	// The local anchor point relative to the origin of body A.
	LocalAnchorA Vec2

	// The local anchor point relative to the origin of body B.
	LocalAnchorB Vec2

	// The angle of body B minus the angle of body A in the reference
	// state, in turns. It is clamped to [-0.5, 0.5] turns.
	ReferenceAngle Q

	// The target angle for the joint in turns. The spring-damper pulls
	// toward this angle.
	TargetAngle Q

	// Enable a rotational spring on the revolute hinge axis.
	EnableSpring bool

	// The spring stiffness in cycles per second.
	Hertz Q

	// The spring damping ratio, non-dimensional.
	DampingRatio Q

	// Enable and disable the joint limit.
	EnableLimit bool

	// The lower angle for the joint limit in turns. Must be greater than
	// -0.495 turns.
	LowerAngle Q

	// The upper angle for the joint limit in turns. Must be less than
	// 0.495 turns.
	UpperAngle Q

	// Enable and disable the joint motor.
	EnableMotor bool

	// The maximum motor torque, usually in newton-meters.
	MaxMotorTorque Q

	// The desired motor speed in turns per second.
	MotorSpeed Q

	// DrawSize scales the debug draw of the joint.
	DrawSize Q

	// Set this flag to true if the attached bodies should collide.
	CollideConnected bool

	// Application data attached to the joint.
	UserData any

	// internalValue proves that DefaultRevoluteJointDef ran. upstream
	// internalValue
	internalValue int
}

// DefaultRevoluteJointDef returns the default revolute joint definition.
func DefaultRevoluteJointDef() RevoluteJointDef {
	return RevoluteJointDef{
		DrawSize:      fixed.Q32MustParse("0.25"),
		internalValue: secretCookie,
	}
}

// WeldJointDef configures a weld joint: it holds the relative transform
// of two bodies, rigidly or with a spring. Initialize it with
// DefaultWeldJointDef.
type WeldJointDef struct {
	// The first attached body.
	BodyIdA BodyId

	// The second attached body.
	BodyIdB BodyId

	// The local anchor point relative to the origin of body A.
	LocalAnchorA Vec2

	// The local anchor point relative to the origin of body B.
	LocalAnchorB Vec2

	// The angle of body B minus the angle of body A in the reference
	// state, in turns.
	ReferenceAngle Q

	// Linear stiffness in cycles per second. Zero means rigid.
	LinearHertz Q

	// Angular stiffness in cycles per second. Zero means rigid.
	AngularHertz Q

	// Linear damping ratio, non-dimensional.
	LinearDampingRatio Q

	// Angular damping ratio, non-dimensional.
	AngularDampingRatio Q

	// Set this flag to true if the attached bodies should collide.
	CollideConnected bool

	// Application data attached to the joint.
	UserData any

	// internalValue proves that DefaultWeldJointDef ran. upstream
	// internalValue
	internalValue int
}

// DefaultWeldJointDef returns the default weld joint definition.
func DefaultWeldJointDef() WeldJointDef {
	return WeldJointDef{internalValue: secretCookie}
}

// WheelJointDef configures a wheel joint: body B rotates freely about a
// point on an axis fixed in body A, with a suspension spring along the
// axis. Initialize it with DefaultWheelJointDef.
type WheelJointDef struct {
	// The first attached body.
	BodyIdA BodyId

	// The second attached body.
	BodyIdB BodyId

	// The local anchor point relative to the origin of body A.
	LocalAnchorA Vec2

	// The local anchor point relative to the origin of body B.
	LocalAnchorB Vec2

	// The local translation unit axis in body A.
	LocalAxisA Vec2

	// Enable a linear spring along the local axis.
	EnableSpring bool

	// The spring stiffness in cycles per second.
	Hertz Q

	// The spring damping ratio, non-dimensional.
	DampingRatio Q

	// Enable and disable the joint limit.
	EnableLimit bool

	// The lower translation limit.
	LowerTranslation Q

	// The upper translation limit.
	UpperTranslation Q

	// Enable and disable the joint rotational motor.
	EnableMotor bool

	// The maximum motor torque, usually in newton-meters.
	MaxMotorTorque Q

	// The desired motor speed in turns per second.
	MotorSpeed Q

	// Set this flag to true if the attached bodies should collide.
	CollideConnected bool

	// Application data attached to the joint.
	UserData any

	// internalValue proves that DefaultWheelJointDef ran. upstream
	// internalValue
	internalValue int
}

// HexColor is an RGB color encoded as 0xRRGGBB.
type HexColor uint32

const (
	// ColorAliceBlue is RGB 0xF0F8FF.
	ColorAliceBlue HexColor = 0xF0F8FF
	// ColorAntiqueWhite is RGB 0xFAEBD7.
	ColorAntiqueWhite HexColor = 0xFAEBD7
	// ColorAqua is RGB 0x00FFFF.
	ColorAqua HexColor = 0x00FFFF
	// ColorAquamarine is RGB 0x7FFFD4.
	ColorAquamarine HexColor = 0x7FFFD4
	// ColorAzure is RGB 0xF0FFFF.
	ColorAzure HexColor = 0xF0FFFF
	// ColorBeige is RGB 0xF5F5DC.
	ColorBeige HexColor = 0xF5F5DC
	// ColorBisque is RGB 0xFFE4C4.
	ColorBisque HexColor = 0xFFE4C4
	// ColorBlack is RGB 0x000000.
	ColorBlack HexColor = 0x000000
	// ColorBlanchedAlmond is RGB 0xFFEBCD.
	ColorBlanchedAlmond HexColor = 0xFFEBCD
	// ColorBlue is RGB 0x0000FF.
	ColorBlue HexColor = 0x0000FF
	// ColorBlueViolet is RGB 0x8A2BE2.
	ColorBlueViolet HexColor = 0x8A2BE2
	// ColorBrown is RGB 0xA52A2A.
	ColorBrown HexColor = 0xA52A2A
	// ColorBurlywood is RGB 0xDEB887.
	ColorBurlywood HexColor = 0xDEB887
	// ColorCadetBlue is RGB 0x5F9EA0.
	ColorCadetBlue HexColor = 0x5F9EA0
	// ColorChartreuse is RGB 0x7FFF00.
	ColorChartreuse HexColor = 0x7FFF00
	// ColorChocolate is RGB 0xD2691E.
	ColorChocolate HexColor = 0xD2691E
	// ColorCoral is RGB 0xFF7F50.
	ColorCoral HexColor = 0xFF7F50
	// ColorCornflowerBlue is RGB 0x6495ED.
	ColorCornflowerBlue HexColor = 0x6495ED
	// ColorCornsilk is RGB 0xFFF8DC.
	ColorCornsilk HexColor = 0xFFF8DC
	// ColorCrimson is RGB 0xDC143C.
	ColorCrimson HexColor = 0xDC143C
	// ColorCyan is RGB 0x00FFFF.
	ColorCyan HexColor = 0x00FFFF
	// ColorDarkBlue is RGB 0x00008B.
	ColorDarkBlue HexColor = 0x00008B
	// ColorDarkCyan is RGB 0x008B8B.
	ColorDarkCyan HexColor = 0x008B8B
	// ColorDarkGoldenRod is RGB 0xB8860B.
	ColorDarkGoldenRod HexColor = 0xB8860B
	// ColorDarkGray is RGB 0xA9A9A9.
	ColorDarkGray HexColor = 0xA9A9A9
	// ColorDarkGreen is RGB 0x006400.
	ColorDarkGreen HexColor = 0x006400
	// ColorDarkKhaki is RGB 0xBDB76B.
	ColorDarkKhaki HexColor = 0xBDB76B
	// ColorDarkMagenta is RGB 0x8B008B.
	ColorDarkMagenta HexColor = 0x8B008B
	// ColorDarkOliveGreen is RGB 0x556B2F.
	ColorDarkOliveGreen HexColor = 0x556B2F
	// ColorDarkOrange is RGB 0xFF8C00.
	ColorDarkOrange HexColor = 0xFF8C00
	// ColorDarkOrchid is RGB 0x9932CC.
	ColorDarkOrchid HexColor = 0x9932CC
	// ColorDarkRed is RGB 0x8B0000.
	ColorDarkRed HexColor = 0x8B0000
	// ColorDarkSalmon is RGB 0xE9967A.
	ColorDarkSalmon HexColor = 0xE9967A
	// ColorDarkSeaGreen is RGB 0x8FBC8F.
	ColorDarkSeaGreen HexColor = 0x8FBC8F
	// ColorDarkSlateBlue is RGB 0x483D8B.
	ColorDarkSlateBlue HexColor = 0x483D8B
	// ColorDarkSlateGray is RGB 0x2F4F4F.
	ColorDarkSlateGray HexColor = 0x2F4F4F
	// ColorDarkTurquoise is RGB 0x00CED1.
	ColorDarkTurquoise HexColor = 0x00CED1
	// ColorDarkViolet is RGB 0x9400D3.
	ColorDarkViolet HexColor = 0x9400D3
	// ColorDeepPink is RGB 0xFF1493.
	ColorDeepPink HexColor = 0xFF1493
	// ColorDeepSkyBlue is RGB 0x00BFFF.
	ColorDeepSkyBlue HexColor = 0x00BFFF
	// ColorDimGray is RGB 0x696969.
	ColorDimGray HexColor = 0x696969
	// ColorDodgerBlue is RGB 0x1E90FF.
	ColorDodgerBlue HexColor = 0x1E90FF
	// ColorFireBrick is RGB 0xB22222.
	ColorFireBrick HexColor = 0xB22222
	// ColorFloralWhite is RGB 0xFFFAF0.
	ColorFloralWhite HexColor = 0xFFFAF0
	// ColorForestGreen is RGB 0x228B22.
	ColorForestGreen HexColor = 0x228B22
	// ColorFuchsia is RGB 0xFF00FF.
	ColorFuchsia HexColor = 0xFF00FF
	// ColorGainsboro is RGB 0xDCDCDC.
	ColorGainsboro HexColor = 0xDCDCDC
	// ColorGhostWhite is RGB 0xF8F8FF.
	ColorGhostWhite HexColor = 0xF8F8FF
	// ColorGold is RGB 0xFFD700.
	ColorGold HexColor = 0xFFD700
	// ColorGoldenRod is RGB 0xDAA520.
	ColorGoldenRod HexColor = 0xDAA520
	// ColorGray is RGB 0x808080.
	ColorGray HexColor = 0x808080
	// ColorGreen is RGB 0x008000.
	ColorGreen HexColor = 0x008000
	// ColorGreenYellow is RGB 0xADFF2F.
	ColorGreenYellow HexColor = 0xADFF2F
	// ColorHoneyDew is RGB 0xF0FFF0.
	ColorHoneyDew HexColor = 0xF0FFF0
	// ColorHotPink is RGB 0xFF69B4.
	ColorHotPink HexColor = 0xFF69B4
	// ColorIndianRed is RGB 0xCD5C5C.
	ColorIndianRed HexColor = 0xCD5C5C
	// ColorIndigo is RGB 0x4B0082.
	ColorIndigo HexColor = 0x4B0082
	// ColorIvory is RGB 0xFFFFF0.
	ColorIvory HexColor = 0xFFFFF0
	// ColorKhaki is RGB 0xF0E68C.
	ColorKhaki HexColor = 0xF0E68C
	// ColorLavender is RGB 0xE6E6FA.
	ColorLavender HexColor = 0xE6E6FA
	// ColorLavenderBlush is RGB 0xFFF0F5.
	ColorLavenderBlush HexColor = 0xFFF0F5
	// ColorLawnGreen is RGB 0x7CFC00.
	ColorLawnGreen HexColor = 0x7CFC00
	// ColorLemonChiffon is RGB 0xFFFACD.
	ColorLemonChiffon HexColor = 0xFFFACD
	// ColorLightBlue is RGB 0xADD8E6.
	ColorLightBlue HexColor = 0xADD8E6
	// ColorLightCoral is RGB 0xF08080.
	ColorLightCoral HexColor = 0xF08080
	// ColorLightCyan is RGB 0xE0FFFF.
	ColorLightCyan HexColor = 0xE0FFFF
	// ColorLightGoldenRodYellow is RGB 0xFAFAD2.
	ColorLightGoldenRodYellow HexColor = 0xFAFAD2
	// ColorLightGray is RGB 0xD3D3D3.
	ColorLightGray HexColor = 0xD3D3D3
	// ColorLightGreen is RGB 0x90EE90.
	ColorLightGreen HexColor = 0x90EE90
	// ColorLightPink is RGB 0xFFB6C1.
	ColorLightPink HexColor = 0xFFB6C1
	// ColorLightSalmon is RGB 0xFFA07A.
	ColorLightSalmon HexColor = 0xFFA07A
	// ColorLightSeaGreen is RGB 0x20B2AA.
	ColorLightSeaGreen HexColor = 0x20B2AA
	// ColorLightSkyBlue is RGB 0x87CEFA.
	ColorLightSkyBlue HexColor = 0x87CEFA
	// ColorLightSlateGray is RGB 0x778899.
	ColorLightSlateGray HexColor = 0x778899
	// ColorLightSteelBlue is RGB 0xB0C4DE.
	ColorLightSteelBlue HexColor = 0xB0C4DE
	// ColorLightYellow is RGB 0xFFFFE0.
	ColorLightYellow HexColor = 0xFFFFE0
	// ColorLime is RGB 0x00FF00.
	ColorLime HexColor = 0x00FF00
	// ColorLimeGreen is RGB 0x32CD32.
	ColorLimeGreen HexColor = 0x32CD32
	// ColorLinen is RGB 0xFAF0E6.
	ColorLinen HexColor = 0xFAF0E6
	// ColorMagenta is RGB 0xFF00FF.
	ColorMagenta HexColor = 0xFF00FF
	// ColorMaroon is RGB 0x800000.
	ColorMaroon HexColor = 0x800000
	// ColorMediumAquaMarine is RGB 0x66CDAA.
	ColorMediumAquaMarine HexColor = 0x66CDAA
	// ColorMediumBlue is RGB 0x0000CD.
	ColorMediumBlue HexColor = 0x0000CD
	// ColorMediumOrchid is RGB 0xBA55D3.
	ColorMediumOrchid HexColor = 0xBA55D3
	// ColorMediumPurple is RGB 0x9370DB.
	ColorMediumPurple HexColor = 0x9370DB
	// ColorMediumSeaGreen is RGB 0x3CB371.
	ColorMediumSeaGreen HexColor = 0x3CB371
	// ColorMediumSlateBlue is RGB 0x7B68EE.
	ColorMediumSlateBlue HexColor = 0x7B68EE
	// ColorMediumSpringGreen is RGB 0x00FA9A.
	ColorMediumSpringGreen HexColor = 0x00FA9A
	// ColorMediumTurquoise is RGB 0x48D1CC.
	ColorMediumTurquoise HexColor = 0x48D1CC
	// ColorMediumVioletRed is RGB 0xC71585.
	ColorMediumVioletRed HexColor = 0xC71585
	// ColorMidnightBlue is RGB 0x191970.
	ColorMidnightBlue HexColor = 0x191970
	// ColorMintCream is RGB 0xF5FFFA.
	ColorMintCream HexColor = 0xF5FFFA
	// ColorMistyRose is RGB 0xFFE4E1.
	ColorMistyRose HexColor = 0xFFE4E1
	// ColorMoccasin is RGB 0xFFE4B5.
	ColorMoccasin HexColor = 0xFFE4B5
	// ColorNavajoWhite is RGB 0xFFDEAD.
	ColorNavajoWhite HexColor = 0xFFDEAD
	// ColorNavy is RGB 0x000080.
	ColorNavy HexColor = 0x000080
	// ColorOldLace is RGB 0xFDF5E6.
	ColorOldLace HexColor = 0xFDF5E6
	// ColorOlive is RGB 0x808000.
	ColorOlive HexColor = 0x808000
	// ColorOliveDrab is RGB 0x6B8E23.
	ColorOliveDrab HexColor = 0x6B8E23
	// ColorOrange is RGB 0xFFA500.
	ColorOrange HexColor = 0xFFA500
	// ColorOrangeRed is RGB 0xFF4500.
	ColorOrangeRed HexColor = 0xFF4500
	// ColorOrchid is RGB 0xDA70D6.
	ColorOrchid HexColor = 0xDA70D6
	// ColorPaleGoldenRod is RGB 0xEEE8AA.
	ColorPaleGoldenRod HexColor = 0xEEE8AA
	// ColorPaleGreen is RGB 0x98FB98.
	ColorPaleGreen HexColor = 0x98FB98
	// ColorPaleTurquoise is RGB 0xAFEEEE.
	ColorPaleTurquoise HexColor = 0xAFEEEE
	// ColorPaleVioletRed is RGB 0xDB7093.
	ColorPaleVioletRed HexColor = 0xDB7093
	// ColorPapayaWhip is RGB 0xFFEFD5.
	ColorPapayaWhip HexColor = 0xFFEFD5
	// ColorPeachPuff is RGB 0xFFDAB9.
	ColorPeachPuff HexColor = 0xFFDAB9
	// ColorPeru is RGB 0xCD853F.
	ColorPeru HexColor = 0xCD853F
	// ColorPink is RGB 0xFFC0CB.
	ColorPink HexColor = 0xFFC0CB
	// ColorPlum is RGB 0xDDA0DD.
	ColorPlum HexColor = 0xDDA0DD
	// ColorPowderBlue is RGB 0xB0E0E6.
	ColorPowderBlue HexColor = 0xB0E0E6
	// ColorPurple is RGB 0x800080.
	ColorPurple HexColor = 0x800080
	// ColorRebeccaPurple is RGB 0x663399.
	ColorRebeccaPurple HexColor = 0x663399
	// ColorRed is RGB 0xFF0000.
	ColorRed HexColor = 0xFF0000
	// ColorRosyBrown is RGB 0xBC8F8F.
	ColorRosyBrown HexColor = 0xBC8F8F
	// ColorRoyalBlue is RGB 0x4169E1.
	ColorRoyalBlue HexColor = 0x4169E1
	// ColorSaddleBrown is RGB 0x8B4513.
	ColorSaddleBrown HexColor = 0x8B4513
	// ColorSalmon is RGB 0xFA8072.
	ColorSalmon HexColor = 0xFA8072
	// ColorSandyBrown is RGB 0xF4A460.
	ColorSandyBrown HexColor = 0xF4A460
	// ColorSeaGreen is RGB 0x2E8B57.
	ColorSeaGreen HexColor = 0x2E8B57
	// ColorSeaShell is RGB 0xFFF5EE.
	ColorSeaShell HexColor = 0xFFF5EE
	// ColorSienna is RGB 0xA0522D.
	ColorSienna HexColor = 0xA0522D
	// ColorSilver is RGB 0xC0C0C0.
	ColorSilver HexColor = 0xC0C0C0
	// ColorSkyBlue is RGB 0x87CEEB.
	ColorSkyBlue HexColor = 0x87CEEB
	// ColorSlateBlue is RGB 0x6A5ACD.
	ColorSlateBlue HexColor = 0x6A5ACD
	// ColorSlateGray is RGB 0x708090.
	ColorSlateGray HexColor = 0x708090
	// ColorSnow is RGB 0xFFFAFA.
	ColorSnow HexColor = 0xFFFAFA
	// ColorSpringGreen is RGB 0x00FF7F.
	ColorSpringGreen HexColor = 0x00FF7F
	// ColorSteelBlue is RGB 0x4682B4.
	ColorSteelBlue HexColor = 0x4682B4
	// ColorTan is RGB 0xD2B48C.
	ColorTan HexColor = 0xD2B48C
	// ColorTeal is RGB 0x008080.
	ColorTeal HexColor = 0x008080
	// ColorThistle is RGB 0xD8BFD8.
	ColorThistle HexColor = 0xD8BFD8
	// ColorTomato is RGB 0xFF6347.
	ColorTomato HexColor = 0xFF6347
	// ColorTurquoise is RGB 0x40E0D0.
	ColorTurquoise HexColor = 0x40E0D0
	// ColorViolet is RGB 0xEE82EE.
	ColorViolet HexColor = 0xEE82EE
	// ColorWheat is RGB 0xF5DEB3.
	ColorWheat HexColor = 0xF5DEB3
	// ColorWhite is RGB 0xFFFFFF.
	ColorWhite HexColor = 0xFFFFFF
	// ColorWhiteSmoke is RGB 0xF5F5F5.
	ColorWhiteSmoke HexColor = 0xF5F5F5
	// ColorYellow is RGB 0xFFFF00.
	ColorYellow HexColor = 0xFFFF00
	// ColorYellowGreen is RGB 0x9ACD32.
	ColorYellowGreen HexColor = 0x9ACD32

	// ColorBox2DRed is RGB 0xDC3132.
	ColorBox2DRed HexColor = 0xDC3132
	// ColorBox2DBlue is RGB 0x30AEBF.
	ColorBox2DBlue HexColor = 0x30AEBF
	// ColorBox2DGreen is RGB 0x8CC924.
	ColorBox2DGreen HexColor = 0x8CC924
	// ColorBox2DYellow is RGB 0xFFEE8C.
	ColorBox2DYellow HexColor = 0xFFEE8C
)

// DrawPolygonFcn draws a closed polygon in counter-clockwise order.
type DrawPolygonFcn func(vertices []Vec2, color HexColor)

// DrawSolidPolygonFcn draws a solid polygon with a rounded radius.
type DrawSolidPolygonFcn func(transform Transform, vertices []Vec2, radius Q, color HexColor)

// DrawCircleFcn draws a circle outline.
type DrawCircleFcn func(center Vec2, radius Q, color HexColor)

// DrawSolidCircleFcn draws a solid circle.
type DrawSolidCircleFcn func(transform Transform, radius Q, color HexColor)

// DrawSolidCapsuleFcn draws a solid capsule.
type DrawSolidCapsuleFcn func(p1, p2 Vec2, radius Q, color HexColor)

// DrawSegmentFcn draws a line segment.
type DrawSegmentFcn func(p1, p2 Vec2, color HexColor)

// DrawTransformFcn draws a transform with a host-selected scale.
type DrawTransformFcn func(transform Transform)

// DrawPointFcn draws a point of the requested size.
type DrawPointFcn func(p Vec2, size Q, color HexColor)

// DrawStringFcn draws a world-space string.
type DrawStringFcn func(p Vec2, s string, color HexColor)

// DebugDraw holds the callbacks and options used by WorldId.Draw.
// Set the option fields to choose what the callbacks receive.
type DebugDraw struct {
	// DrawPolygon draws a closed polygon provided in CCW order.
	DrawPolygon DrawPolygonFcn
	// DrawSolidPolygon draws a solid closed polygon provided in CCW order.
	DrawSolidPolygon DrawSolidPolygonFcn
	// DrawCircle draws a circle.
	DrawCircle DrawCircleFcn
	// DrawSolidCircle draws a solid circle.
	DrawSolidCircle DrawSolidCircleFcn
	// DrawSolidCapsule draws a solid capsule.
	DrawSolidCapsule DrawSolidCapsuleFcn
	// DrawSegment draws a line segment.
	DrawSegment DrawSegmentFcn
	// DrawTransform draws a transform at a host-chosen length scale.
	DrawTransform DrawTransformFcn
	// DrawPoint draws a point.
	DrawPoint DrawPointFcn
	// DrawString draws a string in world space.
	DrawString DrawStringFcn

	// DrawingBounds is the region used when UseDrawingBounds restricts drawing.
	DrawingBounds AABB
	// UseDrawingBounds restricts drawing to DrawingBounds; depth sorting may be unstable.
	UseDrawingBounds bool
	// DrawShapes draws shapes.
	DrawShapes bool
	// DrawJoints draws joints.
	DrawJoints bool
	// DrawJointExtras draws additional information for joints.
	DrawJointExtras bool
	// DrawBounds draws the bounding boxes for shapes.
	DrawBounds bool
	// DrawMass draws the mass and center of mass of dynamic bodies.
	DrawMass bool
	// DrawBodyNames draws body names.
	DrawBodyNames bool
	// DrawContacts draws contact points.
	DrawContacts bool
	// DrawGraphColors visualizes the graph coloring used for contacts and joints.
	DrawGraphColors bool
	// DrawContactNormals draws contact normals.
	DrawContactNormals bool
	// DrawContactImpulses draws contact normal impulses.
	DrawContactImpulses bool
	// DrawContactFeatures draws contact feature ids.
	DrawContactFeatures bool
	// DrawFrictionImpulses draws contact friction impulses.
	DrawFrictionImpulses bool
	// DrawIslands draws islands as bounding boxes.
	DrawIslands bool
}

// DefaultDebugDraw returns a debug drawer whose callbacks are safe no-ops.
func DefaultDebugDraw() DebugDraw {
	return DebugDraw{
		DrawPolygon:      func([]Vec2, HexColor) {},
		DrawSolidPolygon: func(Transform, []Vec2, Q, HexColor) {},
		DrawCircle:       func(Vec2, Q, HexColor) {},
		DrawSolidCircle:  func(Transform, Q, HexColor) {},
		DrawSolidCapsule: func(Vec2, Vec2, Q, HexColor) {},
		DrawSegment:      func(Vec2, Vec2, HexColor) {},
		DrawTransform:    func(Transform) {},
		DrawPoint:        func(Vec2, Q, HexColor) {},
		DrawString:       func(Vec2, string, HexColor) {},
	}
}

// DefaultWheelJointDef returns the default wheel joint definition.
func DefaultWheelJointDef() WheelJointDef {
	return WheelJointDef{
		LocalAxisA:    Vec2{Y: fixed.Q32One()},
		EnableSpring:  true,
		Hertz:         fixed.Q32One(),
		DampingRatio:  fixed.Q32MustParse("0.7"),
		internalValue: secretCookie,
	}
}
