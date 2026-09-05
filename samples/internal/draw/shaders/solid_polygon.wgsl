// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/data/solid_polygon.vs and samples/data/solid_polygon.fs of Box2D v3.1.1
//
// Vertex buffer 0 (step_mode: vertex): the unit quad, @location(0), the
// six vertices of draw.QuadVertices.
// Vertex buffer 1 (step_mode: instance), matching PolygonData:
//   @location(1) transform: vec4<f32> (x, y, cos, sin)   offset 0
//   @location(2) points12:  vec4<f32> (p1, p2)             offset 16
//   @location(3) points34:  vec4<f32> (p3, p4)             offset 32
//   @location(4) points56:  vec4<f32> (p5, p6)             offset 48
//   @location(5) points78:  vec4<f32> (p7, p8)             offset 64
//   @location(6) count:     sint32                          offset 80
//   @location(7) radius:    f32                             offset 84
//   @location(8) color:     unorm8x4                        offset 88
//
// WGSL forbids passing an array as a single varying, so the eight points
// cross the vertex/fragment boundary as four vec4<f32> locations (9..12),
// packed the same way the instance buffer packs them, and fs_main rebuilds
// the array before calling sdConvexPolygon.
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
    @location(2) instance_points12: vec4<f32>,
    @location(3) instance_points34: vec4<f32>,
    @location(4) instance_points56: vec4<f32>,
    @location(5) instance_points78: vec4<f32>,
    @location(6) instance_count: i32,
    @location(7) instance_radius: f32,
    @location(8) instance_color: vec4<f32>,
}

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) f_position: vec2<f32>,
    @location(1) f_color: vec4<f32>,
    @location(2) f_points12: vec4<f32>,
    @location(3) f_points34: vec4<f32>,
    @location(4) f_points56: vec4<f32>,
    @location(5) f_points78: vec4<f32>,
    @location(6) @interpolate(flat) f_count: i32,
    @location(7) f_radius: f32,
    @location(8) f_thickness: f32,
}

@vertex
fn vs_main(in: VertexInput) -> VertexOutput {
    var out: VertexOutput;
    out.f_position = in.local_position;
    out.f_color = in.instance_color;
    out.f_count = in.instance_count;

    var points = array<vec2<f32>, 8>(
        in.instance_points12.xy, in.instance_points12.zw,
        in.instance_points34.xy, in.instance_points34.zw,
        in.instance_points56.xy, in.instance_points56.zw,
        in.instance_points78.xy, in.instance_points78.zw,
    );

    // Compute polygon AABB
    var lower = points[0];
    var upper = points[0];
    for (var i = 1; i < in.instance_count; i = i + 1) {
        lower = min(lower, points[i]);
        upper = max(upper, points[i]);
    }

    let center = 0.5 * (lower + upper);
    let width = upper - lower;
    let max_width = max(width.x, width.y);

    var radius = in.instance_radius;
    let scale = radius + 0.5 * max_width;
    let inv_scale = 1.0 / scale;

    // Shift and scale polygon points so they fit in a 2x2 quad
    for (var i = 0; i < in.instance_count; i = i + 1) {
        points[i] = inv_scale * (points[i] - center);
    }

    radius = inv_scale * radius;
    out.f_radius = radius;
    out.f_thickness = 3.0 / (uniforms.pixel_scale * scale);

    out.f_points12 = vec4<f32>(points[0], points[1]);
    out.f_points34 = vec4<f32>(points[2], points[3]);
    out.f_points56 = vec4<f32>(points[4], points[5]);
    out.f_points78 = vec4<f32>(points[6], points[7]);

    let x = in.instance_transform.x;
    let y = in.instance_transform.y;
    let c = in.instance_transform.z;
    let s = in.instance_transform.w;
    var p = vec2<f32>(scale * in.local_position.x, scale * in.local_position.y) + center;
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

fn cross2d(v1: vec2<f32>, v2: vec2<f32>) -> f32 {
    return v1.x * v2.y - v1.y * v2.x;
}

// Signed distance function for a convex polygon.
fn sd_convex_polygon(p: vec2<f32>, v: array<vec2<f32>, 8>, count: i32) -> f32 {
    var d = dot(p - v[0], p - v[0]);

    var side = -1.0;
    var j = count - 1;
    for (var i = 0; i < count; i = i + 1) {
        let e = v[i] - v[j];
        let w = p - v[j];
        let we = dot(w, e);
        let b = w - e * clamp(we / dot(e, e), 0.0, 1.0);
        let bb = dot(b, b);

        if (bb < d) {
            d = bb;
        }

        let s = cross2d(w, e);
        if (s >= 0.0) {
            side = 1.0;
        }

        j = i;
    }

    return side * sqrt(d);
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    let points = array<vec2<f32>, 8>(
        in.f_points12.xy, in.f_points12.zw,
        in.f_points34.xy, in.f_points34.zw,
        in.f_points56.xy, in.f_points56.zw,
        in.f_points78.xy, in.f_points78.zw,
    );

    let border_color = in.f_color;
    let fill_color = 0.6 * border_color;

    let dw = sd_convex_polygon(in.f_position, points, in.f_count);
    let d = abs(dw - in.f_radius);

    let back = vec4<f32>(fill_color.rgb, fill_color.a * (1.0 - smoothstep(in.f_radius, in.f_radius + in.f_thickness, dw)));
    let front = vec4<f32>(border_color.rgb, 1.0 - smoothstep(0.0, in.f_thickness, d));

    return blend_colors(front, back);
}
