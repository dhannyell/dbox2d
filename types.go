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

	// Deferred: the mixing callbacks and the task system fields of the
	// reference.

	// Enable sleeping to improve performance.
	EnableSleep bool

	// Enable continuous collision.
	EnableContinuous bool

	// Application data attached to the world.
	UserData any

	// internalValue proves that DefaultWorldDef ran. upstream internalValue
	internalValue int
}

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
