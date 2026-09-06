// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/data/background.vs and samples/data/background.fs of Box2D v3.1.1
//
// Vertex layout: none. The full-screen quad comes from @builtin(vertex_index),
// six vertices, same corners the reference keeps in a static VBO.
// WGSL's @builtin(position) has its origin at the top-left, unlike GL's
// gl_FragCoord (bottom-left); the noise pattern still covers the screen the
// same way, just mirrored vertically frame to frame.

struct Uniforms {
    time: f32,
    _pad0: f32,
    _pad1: f32,
    _pad2: f32,
    resolution: vec2<f32>,
    _pad3: f32,
    _pad4: f32,
    base_color: vec3<f32>,
    _pad5: f32,
}

@group(0) @binding(0) var<uniform> uniforms: Uniforms;

struct VertexOutput {
    @builtin(position) position: vec4<f32>,
}

@vertex
fn vs_main(@builtin(vertex_index) index: u32) -> VertexOutput {
    // GL_TRIANGLE_STRIP {-1,1},{-1,-1},{1,1},{1,-1} becomes two triangles here.
    var quad = array<vec2<f32>, 6>(
        vec2<f32>(-1.0, 1.0), vec2<f32>(-1.0, -1.0), vec2<f32>(1.0, 1.0),
        vec2<f32>(1.0, -1.0), vec2<f32>(1.0, 1.0), vec2<f32>(-1.0, -1.0),
    );
    var out: VertexOutput;
    out.position = vec4<f32>(quad[index], 0.0, 1.0);
    return out;
}

fn random(st: vec2<f32>) -> f32 {
    return fract(sin(dot(st.xy, vec2<f32>(12.9898, 78.233))) * 43758.5453123);
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    let uv = in.position.xy / uniforms.resolution.xy;
    let noise = random(uv + uniforms.time * 0.1);
    let grain_intensity = 0.01;
    let color = uniforms.base_color + vec3<f32>(noise * grain_intensity);
    return vec4<f32>(color, 1.0);
}
