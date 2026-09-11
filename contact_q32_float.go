//go:build !dbox2d_fixed

package dbox2d

// Float mode has one grid, so every contact fits the lane and the Q32 path
// is empty. These stubs keep the dispatch one source for both modes.

// contactConstraint32 keeps the color layout available in float mode.
type contactConstraint32 struct{}

// contactFitsLane reports that every contact fits the float lane.
func contactFitsLane(*contactSim) bool { return true }

func allocateContactConstraints32(*world, *stepContext, *[graphColorCount]graphColor) {}

func prepareContacts32(*stepContext) {}

func warmStartContacts32(*stepContext, int) {}

func solveContacts32(*stepContext, int, bool) {}

func applyRestitution32(*stepContext, int) {}

func storeImpulses32(*stepContext) {}
