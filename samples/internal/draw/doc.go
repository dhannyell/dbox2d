// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/draw.h and samples/draw.cpp of Box2D v3.1.1

// Package draw turns debug-draw callbacks into GPU-ready batches. It holds
// no GPU handles: a rendering host uploads the batches and runs the shader
// pairs this package embeds.
package draw
