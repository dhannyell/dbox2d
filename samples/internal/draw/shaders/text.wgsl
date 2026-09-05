// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT
// Text overlay shader; the reference draws text through ImGui, not a GLSL
// pipeline, so there is no GLSL source to port line by line here.
//
// Vertex buffer 0 (step_mode: vertex), matching TextVertex (text.go):
//   @location(0) position: vec2<f32>  offset 0
//   @location(1) uv:       vec2<f32>  offset 8
//   @location(2) color:    unorm8x4   offset 16
// The atlas is sampled as alpha only (r8unorm) and multiplied into color,
// matching the GLSL colour-conversion and premultiplication idiom used by
// the other shaders in this package (blend_colors is not needed here
// because the quads never overlap).

struct Uniforms {
    projection: mat4x4<f32>,
    _pad0: f32,
    _pad1: f32,
    _pad2: f32,
    _pad3: f32,
}

@group(0) @binding(0) var<uniform> uniforms: Uniforms;
@group(0) @binding(1) var atlas_texture: texture_2d<f32>;
@group(0) @binding(2) var atlas_sampler: sampler;

struct VertexInput {
    @location(0) position: vec2<f32>,
    @location(1) uv: vec2<f32>,
    @location(2) color: vec4<f32>,
}

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) f_uv: vec2<f32>,
    @location(1) f_color: vec4<f32>,
}

@vertex
fn vs_main(in: VertexInput) -> VertexOutput {
    var out: VertexOutput;
    out.f_uv = in.uv;
    out.f_color = in.color;
    out.clip_position = uniforms.projection * vec4<f32>(in.position, 0.0, 1.0);
    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    let a = textureSample(atlas_texture, atlas_sampler, in.f_uv).r;
    return vec4<f32>(in.f_color.rgb, in.f_color.a * a);
}
