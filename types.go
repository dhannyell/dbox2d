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
	// Warning: sensors are not ported yet. Creating a shape with IsSensor set
	// panics.
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
		MaxLength:     huge,
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
	return RevoluteJointDef{internalValue: secretCookie}
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
