// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/main.cpp (UpdateUI) of Box2D v3.1.1

package app

import (
	"github.com/dhannyell/dbox2d/samples"
	"github.com/dhannyell/dbox2d/samples/internal/microui"
)

type gui struct {
	mu     *microui.Context
	camera *samples.Camera
}

const minVisibleWindow = 48

func (g gui) Begin(title string, x, y, width, height int) bool {
	rect := microui.NewRect(x, y, width, height)
	return beginClampedWindow(g.mu, g.camera, title, rect, microui.MU_OPT_NORESIZE|microui.MU_OPT_NOCLOSE)
}

// beginClampedWindow opens a microui window with its stored rect clamped
// before drawing. Since microui only uses the rect passed to BeginWindowEx
// on the first frame, clamping the stored rect keeps moved or resized
// windows from getting stuck outside the frame.
func beginClampedWindow(mu *microui.Context, camera *samples.Camera, title string, rect microui.Rect, opt int) bool {
	if cnt := mu.GetContainer(title); cnt != nil {
		if cnt.Rect.W != 0 {
			rect = cnt.Rect
		}
		rect = clampWindowRect(rect, camera.Width, camera.Height, mu.Style.TitleHeight)
		cnt.Rect = rect
	}
	return mu.BeginWindowEx(title, rect, opt) != 0
}

// clampWindowRect keeps the title bar inside the frame so the window
// can always be dragged. The window can go past the sides as long as
// minVisibleWindow pixels stay visible. Its size is never changed, so
// oversized windows are clipped instead of resized.
func clampWindowRect(rect microui.Rect, frameWidth, frameHeight, titleHeight int) microui.Rect {
	visible := min(rect.W, minVisibleWindow)
	rect.X = min(max(rect.X, visible-rect.W), max(frameWidth-visible, 0))
	rect.Y = min(max(rect.Y, 0), max(frameHeight-titleHeight, 0))
	return rect
}

func (g gui) End() {
	g.mu.EndWindow()
}

func (g gui) SliderFloat(label string, value *float64, lo, hi float64) bool {
	g.mu.LayoutRow(2, []int{labelWidth, -1}, 0)
	g.mu.Label(label)
	v := float32(*value)
	changed := g.mu.SliderEx(&v, float32(lo), float32(hi), 0, "%.2f", microui.MU_OPT_ALIGNCENTER) != 0
	*value = float64(v)
	return changed
}

func (g gui) SliderInt(label string, value *int, lo, hi int) bool {
	g.mu.LayoutRow(2, []int{labelWidth, -1}, 0)
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

// Combo is ImGui::Combo in one row: a button naming the current item opens
// a popup list under it, or above it when the list would leave the frame.
func (g gui) Combo(label string, current *int, items []string) bool {
	if len(items) == 0 {
		return false
	}
	*current = min(max(*current, 0), len(items)-1)

	g.mu.LayoutRow(2, []int{labelWidth, -1}, 0)
	g.mu.Label(label)
	popup := "!combo " + label
	opened := g.mu.Button(items[*current])
	button := g.mu.LastRect
	if opened {
		g.mu.OpenPopup(popup)
		style := g.mu.Style
		rowHeight := style.Size.Y + 2*style.Padding
		listHeight := len(items)*(rowHeight+style.Spacing) - style.Spacing + 2*style.Padding
		y := button.Y + button.H
		if y+listHeight > g.camera.Height {
			y = button.Y - listHeight
		}
		if cnt := g.mu.GetContainer(popup); cnt != nil {
			cnt.Rect = microui.NewRect(button.X, y, button.W, listHeight)
		}
	}

	changed := false
	if g.mu.BeginPopup(popup) != 0 {
		g.mu.LayoutRow(1, []int{button.W - 2*g.mu.Style.Padding}, 0)
		for i, item := range items {
			if g.mu.Button(comboLabel(i, *current, item)) {
				changed = i != *current
				*current = i
				g.mu.GetCurrentContainer().Open = false
			}
		}
		g.mu.EndPopup()
	}
	return changed
}

func (g gui) RadioButton(label string, active bool) bool {
	g.mu.LayoutRow(1, []int{-1}, 0)
	return g.mu.Button(radioLabel(label, active))
}

func radioLabel(label string, active bool) string {
	if active {
		return "* " + label
	}
	return label
}

func (g gui) Text(text string) {
	g.mu.LayoutRow(1, []int{-1}, 0)
	g.mu.Label(text)
}

func comboLabel(index, current int, item string) string {
	if index == current {
		return "* " + item
	}
	return item
}

var _ samples.Gui = gui{}
