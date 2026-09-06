// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from Sample::ParsePath in samples/sample.cpp of Box2D v3.1.1

package samples

import (
	"strconv"
	"strings"

	"github.com/dhannyell/dbox2d"
)

// parsePath reads an SVG path made only of straight lines (M, L, H, V and
// their relative forms) and returns at most capacity scaled points. The y
// axis flips, as in the reference. The reference ignores reverseOrder.
func parsePath(svgPath string, offset dbox2d.Vec2, capacity int, scale dbox2d.Q) []dbox2d.Vec2 {
	points := make([]dbox2d.Vec2, 0, capacity)
	var x, y float64
	command := byte(0)
	for _, token := range strings.Fields(svgPath) {
		if token == "z" {
			break
		}
		if len(token) == 1 && strings.Contains("MLHVmlhv", token) {
			command = token[0]
			continue
		}
		values := strings.Split(token, ",")
		first, _ := strconv.ParseFloat(values[0], 64)
		second := 0.0
		if len(values) > 1 {
			second, _ = strconv.ParseFloat(values[1], 64)
		}
		switch command {
		case 'M', 'L':
			x, y = first, second
		case 'H':
			x = first
		case 'V':
			y = first
		case 'm', 'l':
			x, y = x+first, y+second
		case 'h':
			x += first
		case 'v':
			y += first
		}
		p := dbox2d.Vec2{
			X: scale.Mul(FromFloat64(x).Add(offset.X)),
			Y: scale.Neg().Mul(FromFloat64(y).Add(offset.Y)),
		}
		points = append(points, p)
		if len(points) == capacity {
			break
		}
	}
	return points
}
