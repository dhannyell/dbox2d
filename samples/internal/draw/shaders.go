// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/data/{background,circle,solid_circle,solid_capsule,solid_polygon}.{vs,fs} of Box2D v3.1.1

package draw

import _ "embed"

// Each constant is one WGSL module with a @vertex vs_main and a @fragment
// fs_main; a host compiles it directly into a render pipeline.
var (
	//go:embed shaders/background.wgsl
	BackgroundWGSL string

	//go:embed shaders/circle.wgsl
	CircleWGSL string

	//go:embed shaders/solid_circle.wgsl
	SolidCircleWGSL string

	//go:embed shaders/solid_capsule.wgsl
	SolidCapsuleWGSL string

	//go:embed shaders/solid_polygon.wgsl
	SolidPolygonWGSL string

	//go:embed shaders/lines.wgsl
	LinesWGSL string

	//go:embed shaders/points.wgsl
	PointsWGSL string

	//go:embed shaders/text.wgsl
	TextWGSL string
)
