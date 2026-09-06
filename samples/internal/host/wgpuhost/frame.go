//go:build cgo || js

package wgpuhost

import (
	"github.com/dhannyell/webgpu/wgpu"

	"github.com/dhannyell/dbox2d/samples/internal/gpu"
)

// frame is one surface frame: at most one pass, then End submits and
// presents it.
type frame struct {
	d       *Device
	tex     *wgpu.Texture
	view    *wgpu.TextureView
	encoder *wgpu.CommandEncoder
}

func (f *frame) Size() (int, int) { return f.d.width, f.d.height }

func (f *frame) BeginPass(clear [4]float32) gpu.Pass {
	renderPass, err := f.encoder.TryBeginRenderPass(&wgpu.RenderPassDescriptor{
		Label: "frame",
		ColorAttachments: []wgpu.RenderPassColorAttachment{{
			View: f.view, LoadOp: wgpu.LoadOpClear, StoreOp: wgpu.StoreOpStore,
			ClearValue: wgpu.Color{R: float64(clear[0]), G: float64(clear[1]), B: float64(clear[2]), A: float64(clear[3])},
		}},
	})
	if err != nil {
		panic(err)
	}
	return &pass{encoder: renderPass}
}

func (f *frame) End() {
	cmdBuffer, err := f.encoder.TryFinish(&wgpu.CommandBufferDescriptor{Label: "frame"})
	if err != nil {
		panic(err)
	}
	f.d.queue.Submit(cmdBuffer)
	f.d.surface.Present()

	cmdBuffer.Release()
	f.encoder.Release()
	f.view.Release()
	f.tex.Release()
}

// pass is one render pass within a frame.
type pass struct {
	encoder *wgpu.RenderPassEncoder
}

func (p *pass) SetPipeline(pipeline gpu.Pipeline) {
	p.encoder.SetPipeline(pipeline.(*whPipeline).pipeline)
}

func (p *pass) SetBindGroup(index int, g gpu.BindGroup) {
	p.encoder.SetBindGroup(uint32(index), g.(*whBindGroup).group, nil)
}

func (p *pass) SetVertexBuffer(slot int, b gpu.Buffer, offset, size int) {
	p.encoder.SetVertexBuffer(uint32(slot), b.(*whBuffer).buf, uint64(offset), uint64(size))
}

func (p *pass) SetScissor(x, y, w, h int) {
	p.encoder.SetScissorRect(uint32(x), uint32(y), uint32(w), uint32(h))
}

func (p *pass) Draw(vertexCount, instanceCount int) {
	p.encoder.Draw(uint32(vertexCount), uint32(instanceCount), 0, 0)
}

func (p *pass) End() {
	p.encoder.End()
	p.encoder.Release()
}
