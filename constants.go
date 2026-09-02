package dbox2d

import "github.com/dhannyell/fixed"

// A length is a meter. The reference lets an application rescale its
// tolerances; this package does not, because a world that rescales them
// stops matching a world that does not. See DIVERGENCES.md.
var (
	// upstream 0.005f * b2_lengthUnitsPerMeter
	linearSlop = fixed.Q32MustParse("0.005")

	// upstream 100000.0f * b2_lengthUnitsPerMeter. It bounds a length that
	// an application supplies, so a wild input fails early.
	huge = fixed.Q32FromInt(100000)

	// upstream 0.25f * B2_PI radians, which is 0.125 turns
	maxRotation = fixed.Q32MustParse("0.125")

	// upstream 4.0f * B2_LINEAR_SLOP
	speculativeDistance = linearSlop.Mul(fixed.Q32FromInt(4))

	// upstream 0.05f * b2_lengthUnitsPerMeter
	aabbMargin = fixed.Q32MustParse("0.05")

	// upstream 0.5f
	timeToSleep = fixed.Q32Half()

	// upstream 60.0f
	jointConstraintHertz = fixed.Q32FromInt(60)

	// upstream 2.0f
	jointConstraintDampingRatio = fixed.Q32FromInt(2)
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
