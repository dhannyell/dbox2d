// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/sample.h of Box2D v3.1.1

package samples

// Gui is the small widget set scenes use for their controls.
type Gui interface {
	Begin(title string, x, y, width, height int) bool
	End()
	SliderFloat(label string, v *float64, lo, hi float64) bool
	SliderInt(label string, v *int, lo, hi int) bool
	Checkbox(label string, v *bool) bool
	Button(label string) bool
	Combo(label string, current *int, items []string) bool
}

// NopGui is a headless GUI implementation.
type NopGui struct{}

func (NopGui) Begin(string, int, int, int, int) bool               { return false }
func (NopGui) End()                                                {}
func (NopGui) SliderFloat(string, *float64, float64, float64) bool { return false }
func (NopGui) SliderInt(string, *int, int, int) bool               { return false }
func (NopGui) Checkbox(string, *bool) bool                         { return false }
func (NopGui) Button(string) bool                                  { return false }
func (NopGui) Combo(string, *int, []string) bool                   { return false }
