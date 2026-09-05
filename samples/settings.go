// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/sample.h of Box2D v3.1.1

package samples

// Settings holds the state a host toggles between steps. It excludes the
// window, camera and draw fields of the reference SampleContext, which live
// on SampleContext instead.
//
// Ini persistence (Save/Load in the reference) is host business, not ported.
type Settings struct {
	Hertz        float64
	SubStepCount int
	// WorkerCount is fixed at 1; the port steps on one worker.
	WorkerCount int

	Restart         bool
	Pause           bool
	SingleStep      bool
	UseCameraBounds bool

	DrawJointExtras      bool
	DrawBounds           bool
	DrawMass             bool
	DrawBodyNames        bool
	DrawContactPoints    bool
	DrawContactNormals   bool
	DrawContactImpulses  bool
	DrawContactFeatures  bool
	DrawFrictionImpulses bool
	DrawIslands          bool
	DrawGraphColors      bool
	DrawCounters         bool
	DrawProfile          bool

	EnableWarmStarting bool
	EnableContinuous   bool
	EnableSleep        bool

	SampleIndex int
	DrawShapes  bool
	DrawJoints  bool
}

// DefaultSettings returns the settings the reference app starts with.
func DefaultSettings() Settings {
	return Settings{
		Hertz:              60,
		SubStepCount:       4,
		WorkerCount:        1,
		EnableWarmStarting: true,
		EnableContinuous:   true,
		EnableSleep:        true,
		DrawShapes:         true,
		DrawJoints:         true,
	}
}
