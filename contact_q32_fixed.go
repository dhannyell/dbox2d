//go:build dbox2d_fixed

package dbox2d

import "github.com/dhannyell/fixed"

// The lane window is the range of inverse masses the Q16.16 contact grid
// resolves: a coefficient below 2^-6 keeps fewer than ten bits, and one at
// 2^15 or above does not fit. A contact with a coefficient outside the
// window solves on the scalar grid, in contact_solver_q32.go.
var (
	laneWindowLow  = fixed.Q32FromRaw(1 << 26)
	laneWindowHigh = fixed.Q32FromInt(1 << 15)
)

// contactFitsLane reports whether every nonzero inverse mass and inertia of
// the contact is inside the lane window.
func contactFitsLane(cs *contactSim) bool {
	return fitsLaneWindow(cs.invMassA) && fitsLaneWindow(cs.invMassB) && fitsLaneWindow(cs.invIA) && fitsLaneWindow(cs.invIB)
}

func fitsLaneWindow(inv Q) bool {
	return inv.Eq(QZero()) || (!inv.Less(laneWindowLow) && inv.Less(laneWindowHigh))
}

// allocateContactConstraints32 reserves the Q32 scratch of every color, after
// the family scratch; the step frees it first.
func allocateContactConstraints32(w *world, context *stepContext, colors *[graphColorCount]graphColor) {
	count := 0
	for i := range graphColorCount {
		count += len(colors[i].contacts32)
	}
	constraints, mem := arenaSlice[contactConstraint32](&w.arena, count, "contact constraint q32")
	context.contactConstraintMem32 = mem
	base := 0
	for i := range graphColorCount {
		color := &colors[i]
		n := len(color.contacts32)
		color.contactConstraints32 = constraints[base : base+n : base+n]
		base += n
	}
}

// prepareContacts32 builds the Q32 constraints of every color.
func prepareContacts32(context *stepContext) {
	for i := range graphColorCount {
		color := &context.graph.colors[i]
		prepareContactRange32(0, len(color.contacts32), context, color.contacts32, color.contactConstraints32)
	}
}

// warmStartContacts32 applies the stored impulses of the Q32 contacts of one color.
func warmStartContacts32(context *stepContext, colorIndex int) {
	constraints := context.graph.colors[colorIndex].contactConstraints32
	warmStartContactRange32(0, len(constraints), context, constraints)
}

// solveContacts32 solves the Q32 contacts of one color, with the push-out
// speed of its family.
func solveContacts32(context *stepContext, colorIndex int, useBias bool) {
	constraints := context.graph.colors[colorIndex].contactConstraints32
	pushout := context.world.contactSpeed
	if colorIndex == overflowIndex {
		pushout = context.world.maxContactPushSpeed
	}
	solveContactRange32(0, len(constraints), context, constraints, useBias, qcwFrom(pushout))
}

// applyRestitution32 applies restitution to the Q32 contacts of one color.
func applyRestitution32(context *stepContext, colorIndex int) {
	constraints := context.graph.colors[colorIndex].contactConstraints32
	applyRestitutionRange32(0, len(constraints), context, constraints)
}

// storeImpulses32 stores the impulses of the Q32 contacts of every color.
func storeImpulses32(context *stepContext) {
	for i := range graphColorCount {
		color := &context.graph.colors[i]
		storeImpulseRange32(0, len(color.contacts32), color.contacts32, color.contactConstraints32)
	}
}
