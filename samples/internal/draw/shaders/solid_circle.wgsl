// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/data/solid_circle.vs and samples/data/solid_circle.fs of Box2D v3.1.1
//
// Vertex buffer 0 (step_mode: vertex): the unit quad, @location(0), the
// six vertices of draw.QuadVertices.
// Vertex buffer 1 (step_mode: instance), matching SolidCircleData:
//   @location(1) transform: vec4<f32> (x, y, cos, sin) offset 0
//   @location(2) radius:    f32                          offset 16
//   @location(3) color:     unorm8x4                      offset 20
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
    @location(1) instance_transform: vec4<f32>,
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

    out.f_thickness = 3.0 / (uniforms.pixel_scale * radius);

    let x = in.instance_transform.x;
    let y = in.instance_transform.y;
    let c = in.instance_transform.z;
    let s = in.instance_transform.w;
    var p = vec2<f32>(radius * in.local_position.x, radius * in.local_position.y);
    p = vec2<f32>((c * p.x - s * p.y) + x, (s * p.x + c * p.y) + y);
    out.clip_position = uniforms.projection * vec4<f32>(p, 0.0, 1.0);
    return out;
}

// blend_colors: https://en.wikipedia.org/wiki/Alpha_compositing
fn blend_colors(front: vec4<f32>, back: vec4<f32>) -> vec4<f32> {
    let c_src = front.rgb;
    let alpha_src = front.a;
    let c_dst = back.rgb;
    let alpha_dst = back.a;

    var c_out = c_src * alpha_src + c_dst * alpha_dst * (1.0 - alpha_src);
    let alpha_out = alpha_src + alpha_dst * (1.0 - alpha_src);
    c_out = c_out / alpha_out;

    return vec4<f32>(c_out, alpha_out);
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    let radius = 1.0;

    let e = vec2<f32>(radius, 0.0);
    let w = in.f_position;
    let we = dot(w, e);
    let b = w - e * clamp(we / dot(e, e), 0.0, 1.0);
    let da = length(b);

    let dw = length(w);
    let dc = abs(dw - radius);

    let d = min(da, dc);

    let border_color = in.f_color;
    let fill_color = 0.6 * border_color;

    let back = vec4<f32>(fill_color.rgb, fill_color.a * (1.0 - smoothstep(radius, radius + in.f_thickness, dw)));
    let front = vec4<f32>(border_color.rgb, 1.0 - smoothstep(0.0, in.f_thickness, d));

    return blend_colors(front, back);
}
