//go:build js && wasm

// Package wasm hosts the samples module in a browser tab: it implements
// the gpu seam over the js backend of github.com/dhannyell/webgpu/wgpu,
// drives App.Frame from requestAnimationFrame, and routes canvas and
// keyboard events into it.
package wasm

import (
	"fmt"

	"github.com/dhannyell/webgpu/wgpu"

	"github.com/dhannyell/dbox2d/samples/internal/gpu"
)

// device implements gpu.Device over one wgpu.Device and the canvas surface
// it was configured against.
type device struct {
	dev           *wgpu.Device
	queue         *wgpu.Queue
	surface       *wgpu.Surface
	wgpuFormat    wgpu.TextureFormat
	format        gpu.TextureFormat
	sampler       *wgpu.Sampler
	width, height int
}

func newDevice(dev *wgpu.Device, surface *wgpu.Surface, wgpuFormat wgpu.TextureFormat) (*device, error) {
	sampler, err := dev.TryCreateSampler(&wgpu.SamplerDescriptor{
		AddressModeU:  wgpu.AddressModeClampToEdge,
		AddressModeV:  wgpu.AddressModeClampToEdge,
		AddressModeW:  wgpu.AddressModeClampToEdge,
		MagFilter:     wgpu.FilterModeLinear,
		MinFilter:     wgpu.FilterModeLinear,
		LodMaxClamp:   32,
		MaxAnisotropy: 1, // the browser rejects the zero value
	})
	if err != nil {
		return nil, fmt.Errorf("wasm: create sampler: %w", err)
	}
	return &device{
		dev: dev, queue: dev.GetQueue(), surface: surface,
		wgpuFormat: wgpuFormat, format: fromWgpuFormat(wgpuFormat), sampler: sampler,
	}, nil
}

// resize updates the pixel size Frame.Size reports; the caller has already
// reconfigured the surface at this size.
func (d *device) resize(width, height int) { d.width, d.height = width, height }

type whBuffer struct{ buf *wgpu.Buffer }

func (b *whBuffer) Release() { b.buf.Release() }

type whTexture struct {
	tex  *wgpu.Texture
	view *wgpu.TextureView
}

func (t *whTexture) Release() {
	t.view.Release()
	t.tex.Release()
}

type whShader struct{ mod *wgpu.ShaderModule }

func (s *whShader) Release() { s.mod.Release() }

type whPipeline struct{ pipeline *wgpu.RenderPipeline }

func (p *whPipeline) Release() { p.pipeline.Release() }

type whBindGroup struct{ group *wgpu.BindGroup }

func (g *whBindGroup) Release() { g.group.Release() }

func (d *device) CreateBuffer(label string, usage gpu.BufferUsage, size int) gpu.Buffer {
	buf, err := d.dev.TryCreateBuffer(&wgpu.BufferDescriptor{
		Label: label, Usage: toBufferUsage(usage), Size: uint64(size),
	})
	if err != nil {
		panic(fmt.Errorf("wasm: create buffer %q: %w", label, err))
	}
	return &whBuffer{buf}
}

func (d *device) CreateTexture(label string, width, height int, format gpu.TextureFormat) gpu.Texture {
	tex, err := d.dev.TryCreateTexture(&wgpu.TextureDescriptor{
		Label:         label,
		Usage:         wgpu.TextureUsageTextureBinding | wgpu.TextureUsageCopyDst,
		Dimension:     wgpu.TextureDimension2D,
		Size:          wgpu.Extent3D{Width: uint32(width), Height: uint32(height), DepthOrArrayLayers: 1},
		Format:        toTextureFormat(format),
		MipLevelCount: 1,
		SampleCount:   1,
	})
	if err != nil {
		panic(fmt.Errorf("wasm: create texture %q: %w", label, err))
	}
	view, err := tex.TryCreateView(nil)
	if err != nil {
		panic(fmt.Errorf("wasm: create texture view %q: %w", label, err))
	}
	return &whTexture{tex: tex, view: view}
}

func (d *device) CreateShader(label, wgsl string) gpu.Shader {
	mod, err := d.dev.TryCreateShaderModule(&wgpu.ShaderModuleDescriptor{
		Label: label, WGSLSource: &wgpu.ShaderSourceWGSL{Code: wgsl},
	})
	if err != nil {
		panic(fmt.Errorf("wasm: compile shader %q: %w", label, err))
	}
	return &whShader{mod}
}

func (d *device) CreatePipeline(desc gpu.PipelineDesc) gpu.Pipeline {
	buffers := make([]wgpu.VertexBufferLayout, len(desc.Buffers))
	for i, b := range desc.Buffers {
		attrs := make([]wgpu.VertexAttribute, len(b.Attributes))
		for j, a := range b.Attributes {
			attrs[j] = wgpu.VertexAttribute{
				Format: toVertexFormat(a.Format), Offset: uint64(a.Offset), ShaderLocation: uint32(a.Location),
			}
		}
		buffers[i] = wgpu.VertexBufferLayout{ArrayStride: uint64(b.Stride), StepMode: toStepMode(b.Step), Attributes: attrs}
	}

	shader := desc.Shader.(*whShader).mod
	pipeline, err := d.dev.TryCreateRenderPipeline(&wgpu.RenderPipelineDescriptor{
		Label:  desc.Label,
		Vertex: wgpu.VertexState{Module: shader, EntryPoint: "vs_main", Buffers: buffers},
		Primitive: wgpu.PrimitiveState{
			Topology: toTopology(desc.Topology), StripIndexFormat: wgpu.IndexFormatUndefined,
			FrontFace: wgpu.FrontFaceCCW, CullMode: wgpu.CullModeNone,
		},
		Multisample: wgpu.MultisampleState{Count: 1, Mask: 0xFFFFFFFF},
		Fragment: &wgpu.FragmentState{
			Module: shader, EntryPoint: "fs_main",
			Targets: []wgpu.ColorTargetState{{Format: d.wgpuFormat, Blend: toBlendState(desc.Blend), WriteMask: wgpu.ColorWriteMaskAll}},
		},
	})
	if err != nil {
		panic(fmt.Errorf("wasm: create pipeline %q: %w", desc.Label, err))
	}
	return &whPipeline{pipeline}
}

func (d *device) CreateBindGroup(desc gpu.BindGroupDesc) gpu.BindGroup {
	layout := desc.Pipeline.(*whPipeline).pipeline.GetBindGroupLayout(0)
	entries := make([]wgpu.BindGroupEntry, len(desc.Entries))
	for i, e := range desc.Entries {
		entry := wgpu.BindGroupEntry{Binding: uint32(e.Binding)}
		switch {
		case e.Buffer != nil:
			entry.Buffer = e.Buffer.(*whBuffer).buf
			entry.Size = uint64(e.BufferSize)
		case e.Texture != nil:
			entry.TextureView = e.Texture.(*whTexture).view
		case e.Sampler:
			entry.Sampler = d.sampler
		}
		entries[i] = entry
	}
	group, err := d.dev.TryCreateBindGroup(&wgpu.BindGroupDescriptor{Label: desc.Label, Layout: layout, Entries: entries})
	if err != nil {
		panic(fmt.Errorf("wasm: create bind group %q: %w", desc.Label, err))
	}
	return &whBindGroup{group}
}

func (d *device) WriteBuffer(b gpu.Buffer, offset int, data []byte) {
	if err := d.queue.TryWriteBuffer(b.(*whBuffer).buf, uint64(offset), data); err != nil {
		panic(fmt.Errorf("wasm: write buffer: %w", err))
	}
}

func (d *device) WriteTexture(t gpu.Texture, width, height int, bytesPerRow int, data []byte) {
	tex := t.(*whTexture).tex
	err := d.queue.TryWriteTexture(
		&wgpu.TexelCopyTextureInfo{Texture: tex},
		data,
		&wgpu.TexelCopyBufferLayout{BytesPerRow: uint32(bytesPerRow), RowsPerImage: uint32(height)},
		&wgpu.Extent3D{Width: uint32(width), Height: uint32(height), DepthOrArrayLayers: 1},
	)
	if err != nil {
		panic(fmt.Errorf("wasm: write texture: %w", err))
	}
}

func (d *device) SurfaceFormat() gpu.TextureFormat { return d.format }

func (d *device) BeginFrame() (gpu.Frame, bool) {
	if d.width == 0 || d.height == 0 {
		return nil, false
	}
	surfaceTexture, err := d.surface.TryGetCurrentTexture()
	if err != nil {
		return nil, false
	}
	tex, ok := surfaceTexture.Get()
	if !ok {
		return nil, false
	}
	view, err := tex.TryCreateView(nil)
	if err != nil {
		tex.Release()
		return nil, false
	}
	encoder, err := d.dev.TryCreateCommandEncoder(&wgpu.CommandEncoderDescriptor{Label: "frame"})
	if err != nil {
		view.Release()
		tex.Release()
		return nil, false
	}
	return &frame{d: d, tex: tex, view: view, encoder: encoder}, true
}
