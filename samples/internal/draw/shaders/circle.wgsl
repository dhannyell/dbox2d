// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/data/circle.vs and samples/data/circle.fs of Box2D v3.1.1
//
// Vertex buffer 0 (step_mode: vertex): the unit quad, @location(0) vec2<f32>,
// six vertices from draw.QuadVertices (a = 1.1, same as the reference VBO).
// Vertex buffer 1 (step_mode: instance), matching CircleData:
//   @location(1) position: vec2<f32>  offset 0
//   @location(2) radius:   f32        offset 8
//   @location(3) color:    unorm8x4   offset 12
//
// The GLSL calls smoothstep with inverted edges; this port writes the
// identical 1.0 - smoothstep(low, high, x) so no backend sees that case.

struct Uniforms {
    projection: mat4x4<f32>,
    pixel_scale: f32,
    _pad0: f32,
    _pad1: f32,
    _pad2: f32,
}

@group(0) @binding(0) var<uniform> uniforms: Uniforms;

struct VertexInput {
    @location(0) local_position: vec2<f32>,
    @location(1) instance_position: vec2<f32>,
    @location(2) instance_radius: f32,
    @location(3) instance_color: vec4<f32>,
}

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) f_position: vec2<f32>,
    @location(1) f_color: vec4<f32>,
    @location(2) f_thickness: f32,
}

@vertex
fn vs_main(in: VertexInput) -> VertexOutput {
    var out: VertexOutput;
    out.f_position = in.local_position;
    out.f_color = in.instance_color;
    let radius = in.instance_radius;

    // resolution.y = pixel_scale * radius
    out.f_thickness = 3.0 / (uniforms.pixel_scale * radius);

    let p = vec2<f32>(radius * in.local_position.x, radius * in.local_position.y) + in.instance_position;
    out.clip_position = uniforms.projection * vec4<f32>(p, 0.0, 1.0);
    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    let radius = 1.0;
    let w = in.f_position;
    let dw = length(w);
    let d = abs(dw - radius);
    return vec4<f32>(in.f_color.rgb, 1.0 - smoothstep(0.0, in.f_thickness, d));
}
