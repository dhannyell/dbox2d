// Package gpu is the seam between the renderer and a WebGPU binding. A host
// (the wasm host, or a future native one) implements Device; everything
// else in the samples module talks only to these interfaces, so the
// renderer builds with CGO_ENABLED=0 and never imports a wgpu binding.
package gpu

// BufferUsage selects how a Buffer is bound. Every usage also implies
// copy-dst, since the renderer always uploads through WriteBuffer.
type BufferUsage int

// Buffer usages the renderer needs.
const (
	BufferUsageVertex BufferUsage = iota
	BufferUsageUniform
)

// TextureFormat names a pixel format. Only the formats the renderer and the
// hosts actually use are listed.
type TextureFormat int

// Texture formats the renderer needs.
const (
	TextureFormatR8Unorm TextureFormat = iota
	TextureFormatBGRA8Unorm
	TextureFormatRGBA8Unorm
)

// VertexFormat names one vertex attribute's wire format.
type VertexFormat int

// Vertex formats the shaders in samples/internal/draw use.
const (
	VertexFormatFloat32 VertexFormat = iota
	VertexFormatFloat32x2
	VertexFormatFloat32x4
	VertexFormatUnorm8x4
	VertexFormatSint32
)

// StepMode selects whether a vertex buffer advances per vertex or per
// instance.
type StepMode int

// Step modes.
const (
	StepModeVertex StepMode = iota
	StepModeInstance
)

// Topology selects the primitive a pipeline assembles.
type Topology int

// Topologies the renderer needs.
const (
	TopologyTriangleList Topology = iota
	TopologyLineList
)

// Blend selects a pipeline's fragment blend state.
type Blend int

// Blend modes. BlendAlpha is src-alpha / one-minus-src-alpha for both the
// colour and the alpha channel, matching the reference's GL_BLEND setup.
const (
	BlendNone Blend = iota
	BlendAlpha
)

// BindingKind names what a bind group entry attaches to.
type BindingKind int

// Binding kinds.
const (
	BindingUniform BindingKind = iota
	BindingTexture2D
	BindingSampler
)

// VertexAttribute is one attribute of a VertexBufferLayout.
type VertexAttribute struct {
	Format   VertexFormat
	Offset   int
	Location int
}

// VertexBufferLayout describes one vertex buffer slot of a pipeline.
type VertexBufferLayout struct {
	Stride     int
	Step       StepMode
	Attributes []VertexAttribute
}

// BindingLayout is one binding a pipeline's shader declares.
type BindingLayout struct {
	Binding int
	Kind    BindingKind
}

// PipelineDesc describes a render pipeline. Bind group layout is left to
// the host's auto layout (BindGroupDesc takes the Pipeline, not a layout),
// so Bindings only needs to tell the host what kinds of resources the
// shader declares, for hosts where that matters.
type PipelineDesc struct {
	Label    string
	Shader   Shader
	Buffers  []VertexBufferLayout
	Bindings []BindingLayout
	Topology Topology
	Blend    Blend
}

// BindGroupEntry is one binding of a BindGroupDesc. Exactly one of Buffer,
// Texture or Sampler is set, matching Binding's Kind in the pipeline that
// declared it.
type BindGroupEntry struct {
	Binding    int
	Buffer     Buffer
	BufferSize int
	Texture    Texture
	// Sampler, when true, binds the host's one linear-clamp sampler.
	Sampler bool
}

// BindGroupDesc describes a bind group. Pipeline supplies the layout
// (GetBindGroupLayout(0) on the host side), since every pipeline here uses
// group 0 and an auto layout.
type BindGroupDesc struct {
	Label    string
	Pipeline Pipeline
	Entries  []BindGroupEntry
}

// Releasable is embedded by every GPU resource handle.
type Releasable interface{ Release() }

// Buffer is a GPU buffer handle.
type Buffer interface{ Releasable }

// Texture is a GPU texture handle.
type Texture interface{ Releasable }

// Shader is a compiled shader module handle.
type Shader interface{ Releasable }

// Pipeline is a render pipeline handle.
type Pipeline interface{ Releasable }

// BindGroup is a bind group handle.
type BindGroup interface{ Releasable }

// Device is the slice of WebGPU the renderer uses. A host implements it.
type Device interface {
	CreateBuffer(label string, usage BufferUsage, size int) Buffer
	// CreateTexture makes a 2D, one-mip texture usable as TEXTURE_BINDING |
	// COPY_DST.
	CreateTexture(label string, width, height int, format TextureFormat) Texture
	CreateShader(label, wgsl string) Shader
	CreatePipeline(desc PipelineDesc) Pipeline
	CreateBindGroup(desc BindGroupDesc) BindGroup
	WriteBuffer(b Buffer, offset int, data []byte)
	WriteTexture(t Texture, width, height int, bytesPerRow int, data []byte)
	SurfaceFormat() TextureFormat
	// BeginFrame starts a frame. It returns false when the surface has no
	// texture to draw into this frame (e.g. a minimized or hidden canvas).
	BeginFrame() (Frame, bool)
}

// Frame is one surface frame: at most one pass, then End presents it.
type Frame interface {
	// Size is the render target size in pixels.
	Size() (width, height int)
	BeginPass(clear [4]float32) Pass
	// End submits the frame's commands and presents it.
	End()
}

// Pass is one render pass within a Frame.
type Pass interface {
	SetPipeline(p Pipeline)
	SetBindGroup(index int, g BindGroup)
	SetVertexBuffer(slot int, b Buffer, offset, size int)
	SetScissor(x, y, w, h int)
	Draw(vertexCount, instanceCount int)
	End()
}
