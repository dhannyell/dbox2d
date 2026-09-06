// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from the GLPoints shaders inlined in samples/draw.cpp of Box2D v3.1.1
//
// Deliberate deviation: GL draws GL_POINTS sized by gl_PointSize, which
// WebGPU has no equivalent for. This port instead expands each point into
// an instanced quad in the vertex shader (size converted from pixels to
// world units through pixel_scale, the same ratio circle.wgsl uses) and
// cuts a round SDF out of it in the fragment shader, so the on-screen
// result still reads as a round sprite of the requested pixel size.
//
// Vertex buffer 0 (step_mode: vertex): the unit quad, @location(0), the
// six vertices of draw.QuadVertices; the 0.1 margin is the anti-alias fringe.
// Vertex buffer 1 (step_mode: instance), matching PointData:
//   @location(1) position: vec2<f32>  offset 0
//   @location(2) size:     f32        offset 8
//   @location(3) color:    unorm8x4   offset 12

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
    @location(2) instance_size: f32,
    @location(3) instance_color: vec4<f32>,
}

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) f_position: vec2<f32>,
    @location(1) f_color: vec4<f32>,
}

@vertex
fn vs_main(in: VertexInput) -> VertexOutput {
    var out: VertexOutput;
    out.f_position = in.local_position;
    out.f_color = in.instance_color;

    // pixel_scale is twice the pixels per world unit, so size pixels of
    // diameter is size / pixel_scale of half extent in world units.
    let half_extent = in.instance_size / uniforms.pixel_scale;
    let p = in.local_position * half_extent + in.instance_position;
    out.clip_position = uniforms.projection * vec4<f32>(p, 0.0, 1.0);
    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    // round cutout of the quad, radius 1 in local space
    let d = length(in.f_position);
    let aa = fwidth(d);
    let alpha = 1.0 - smoothstep(1.0 - aa, 1.0, d);
    return vec4<f32>(in.f_color.rgb, in.f_color.a * alpha);
}
