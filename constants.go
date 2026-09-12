package dbox2d

// graphColorCount is the number of constraint graph colors, including the
// overflow color at the end. It corresponds to B2_GRAPH_COLOR_COUNT in
// src/constants.h.
const graphColorCount = 12

// A length is a meter. The reference lets an application rescale its
// tolerances; this package does not, because a world that rescales them
// stops matching a world that does not. See DIVERGENCES.md.
var (
	// upstream 0.005f * b2_lengthUnitsPerMeter
	linearSlop = QMustParse("0.005")

	// upstream 0.1f * B2_LINEAR_SLOP, the tolerance of the distance queries
	linearSlopTenth = makeRecip(QFromInt(10)).scale(linearSlop)

	// Huge is the largest coordinate the world accepts and the rigid push
	// limit of a collision plane. It corresponds to B2_HUGE in
	// include/box2d/math_functions.h. It bounds a length that an
	// application supplies, so a wild input fails early.
	Huge = QFromInt(100000)

	// upstream 0.25f * B2_PI radians, which is 0.125 turns
	maxRotation = QMustParse("0.125")

	// upstream 4.0f * B2_LINEAR_SLOP
	speculativeDistance = linearSlop.Mul(QFromInt(4))

	// upstream 0.05f * b2_lengthUnitsPerMeter
	aabbMargin = QMustParse("0.05")

	// upstream 0.5f
	timeToSleep = QHalf()

	// upstream 60.0f
	jointConstraintHertz = QFromInt(60)

	// upstream 2.0f
	jointConstraintDampingRatio = QFromInt(2)

	// The constructors below are hoisted out of per-step and per-manifold
	// code: QMustParse scans a string and the fixed QFromRatio divides, and
	// neither belongs inside a prepare or collide call. The bits are the
	// ones the call sites produced.

	// upstream 0.1f, the angular damping ratio of the mouse joint
	mouseAngularDampingRatio = QMustParse("0.1")

	// upstream 0.01f, the Gauss map tolerances of a chain segment
	chainSegmentTolerance = QFromRatio(1, 100)

	// upstream 0.25f, the continuous clip fraction and the rounded polygon
	// mass push-out; exact in both modes
	oneQuarter = QFromRatio(1, 4)

	// upstream 1.412f, the rounded polygon mass push-out
	roundedMassSqrt2 = QMustParse("1.412")

	// upstream 0.99f * B2_PI, the revolute limit range, in turns
	revoluteLimitAngle = QMustParse("0.495")

	// The literals of the Default*Def constructors. A definition is built
	// per body, shape and joint, so the parse runs once here rather than
	// on every creation.
	defaultSleepThreshold    = QMustParse("0.05")
	defaultFriction          = QMustParse("0.6")
	defaultCorrectionFactor  = QMustParse("0.3")
	defaultDrawSize          = QMustParse("0.25")
	defaultWheelDampingRatio = QMustParse("0.7")
)

// LinearSlop is the collision and constraint tolerance in meters. It is
// numerically significant and visually insignificant.
//
// Changing it has a significant effect on stability.
func LinearSlop() Q { return linearSlop }

// MaxRotation is the largest rotation of a body in one time step, in turns.
// The limit prevents numerical problems.
//
// Raising it to 0.25 turns or more breaks continuous collision.
func MaxRotation() Q { return maxRotation }

// SpeculativeDistance is the range of limited speculative collision, in
// meters. It reduces jitter.
//
// Changing it has a significant effect on performance and stability.
func SpeculativeDistance() Q { return speculativeDistance }

// AABBMargin fattens the bounds in the dynamic tree, in meters. A proxy that
// moves less than the margin triggers no tree adjustment.
//
// Changing it has a significant effect on performance.
func AABBMargin() Q { return aabbMargin }

// TimeToSleep is how long a body must stay still before it sleeps, in
// seconds.
func TimeToSleep() Q { return timeToSleep }

// JointConstraintHertz is the default stiffness of a joint constraint.
func JointConstraintHertz() Q { return jointConstraintHertz }

// JointConstraintDampingRatio is the default damping of a joint constraint.
func JointConstraintDampingRatio() Q { return jointConstraintDampingRatio }
