// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/main.cpp (UpdateUI) of Box2D v3.1.1

package app

import (
	"github.com/dhannyell/dbox2d/samples"
	"github.com/dhannyell/dbox2d/samples/internal/microui"
)

type gui struct {
	mu *microui.Context
}

func (g gui) Begin(title string, x, y, width, height int) bool {
	return g.mu.BeginWindowEx(title, microui.NewRect(x, y, width, height), microui.MU_OPT_NORESIZE|microui.MU_OPT_NOCLOSE) != 0
}

func (g gui) End() {
	g.mu.EndWindow()
}

func (g gui) SliderFloat(label string, value *float64, lo, hi float64) bool {
	g.mu.LayoutRow(2, []int{80, -1}, 0)
	g.mu.Label(label)
	v := float32(*value)
	changed := g.mu.SliderEx(&v, float32(lo), float32(hi), 0, "%.2f", microui.MU_OPT_ALIGNCENTER) != 0
	*value = float64(v)
	return changed
}

func (g gui) SliderInt(label string, value *int, lo, hi int) bool {
	g.mu.LayoutRow(2, []int{80, -1}, 0)
	g.mu.Label(label)
	v := float32(*value)
	changed := g.mu.SliderEx(&v, float32(lo), float32(hi), 1, "%.0f", microui.MU_OPT_ALIGNCENTER) != 0
	*value = int(v + 0.5)
	return changed
}

func (g gui) Checkbox(label string, value *bool) bool {
	g.mu.LayoutRow(1, []int{-1}, 0)
	return g.mu.Checkbox(label, value) != 0
}

func (g gui) Button(label string) bool {
	g.mu.LayoutRow(1, []int{-1}, 0)
	return g.mu.Button(label)
}

func (g gui) Combo(label string, current *int, items []string) bool {
	g.mu.LayoutRow(1, []int{-1}, 0)
	g.mu.Label(label)
	widths := make([]int, len(items))
	for i := range widths {
		widths[i] = -1
	}
	g.mu.LayoutRow(len(items), widths, 0)
	changed := false
	for i, item := range items {
		if g.mu.Button(comboLabel(i, *current, item)) && i != *current {
			*current = i
			changed = true
		}
	}
	return changed
}

func comboLabel(index, current int, item string) string {
	if index == current {
		return "* " + item
	}
	return item
}

var _ samples.Gui = gui{}
