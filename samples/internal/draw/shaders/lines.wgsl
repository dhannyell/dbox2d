// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from the GLLines shaders inlined in samples/draw.cpp of Box2D v3.1.1
//
// Vertex buffer 0 (step_mode: vertex), matching VertexData, drawn as
// primitive topology line-list:
//   @location(0) position: vec2<f32>  offset 0
//   @location(1) color:    unorm8x4   offset 8

struct Uniforms {
    projection: mat4x4<f32>,
    _pad0: f32,
    _pad1: f32,
    _pad2: f32,
    _pad3: f32,
}

@group(0) @binding(0) var<uniform> uniforms: Uniforms;

struct VertexInput {
    @location(0) position: vec2<f32>,
    @location(1) color: vec4<f32>,
}

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) f_color: vec4<f32>,
}

@vertex
fn vs_main(in: VertexInput) -> VertexOutput {
    var out: VertexOutput;
    out.f_color = in.color;
    out.clip_position = uniforms.projection * vec4<f32>(in.position, 0.0, 1.0);
    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    return in.f_color;
}
