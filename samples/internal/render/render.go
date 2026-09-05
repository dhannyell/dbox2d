// Package render turns draw.Batches and microui commands into GPU work
// through the gpu seam. It never imports a WebGPU binding: a host supplies
// a gpu.Device, and this package only calls that interface.
package render

import (
	"fmt"
	"unsafe"

	"github.com/dhannyell/dbox2d/samples"
	"github.com/dhannyell/dbox2d/samples/internal/draw"
	"github.com/dhannyell/dbox2d/samples/internal/gpu"
)

// UIKind names one microui draw command.
type UIKind int

// UI command kinds a Renderer draws.
const (
	UIClip UIKind = iota
	UIRect
	UIText
	UIIcon
)

// UICommand is one microui draw command, translated out of package microui
// so render never imports it directly.
type UICommand struct {
	Kind  UIKind
	Rect  [4]int // x, y, w, h
	Color draw.RGBA8
	Text  string
	X, Y  int
	Icon  int
}

// uniforms80 is the 80-byte uniform every batch pipeline and the text
// pipeline share: a projection matrix plus one scale float, padded to a
// vec4 boundary. Lines and text leave pixelScale unused.
type uniforms80 struct {
	projection [16]float32
	pixelScale float32
	_pad       [3]float32 //nolint:unused // GPU-layout padding, never read from Go.
}

// backgroundUniforms mirrors shaders/background.wgsl's Uniforms struct.
type backgroundUniforms struct {
	time       float32
	_pad0      [3]float32 //nolint:unused // GPU-layout padding, never read from Go.
	resolution [2]float32
	_pad1      [2]float32 //nolint:unused // GPU-layout padding, never read from Go.
	baseColor  [3]float32
	_pad2      float32 //nolint:unused // GPU-layout padding, never read from Go.
}

// batchPipeline is one instanced-quad pipeline: circles, solid circles,
// solid capsules, solid polygons and points all share this shape.
type batchPipeline struct {
	pipeline  gpu.Pipeline
	uniform   gpu.Buffer
	bindGroup gpu.BindGroup
	instances gpu.Buffer
	capacity  int
}

// vertexPipeline is a pipeline with a single per-vertex buffer: lines and
// the text/UI pipeline.
type vertexPipeline struct {
	pipeline  gpu.Pipeline
	uniform   gpu.Buffer
	bindGroup gpu.BindGroup
	vertices  gpu.Buffer
	capacity  int
}

// Renderer owns the pipelines and buffers for the batches and the UI.
type Renderer struct {
	dev   gpu.Device
	atlas *draw.Atlas
	quad  gpu.Buffer

	background struct {
		pipeline  gpu.Pipeline
		uniform   gpu.Buffer
		bindGroup gpu.BindGroup
	}

	circles       batchPipeline
	solidCircles  batchPipeline
	solidCapsules batchPipeline
	solidPolygons batchPipeline
	points        batchPipeline
	lines         vertexPipeline
	text          vertexPipeline

	// uiVertices and segments are reused across frames to stage the
	// overlay-text and UI draws.
	uiVertices []draw.TextVertex
	segments   []textSegment
}

func asBytes[T any](s []T) []byte {
	if len(s) == 0 {
		return nil
	}
	return unsafe.Slice((*byte)(unsafe.Pointer(&s[0])), len(s)*int(unsafe.Sizeof(s[0])))
}

// New builds every pipeline this package needs on dev and uploads atlas as
// the text/UI texture.
func New(dev gpu.Device, atlas *draw.Atlas) (*Renderer, error) {
	r := &Renderer{dev: dev, atlas: atlas}

	r.quad = dev.CreateBuffer("quad", gpu.BufferUsageVertex, int(unsafe.Sizeof(draw.QuadVertices)))
	dev.WriteBuffer(r.quad, 0, asBytes(draw.QuadVertices[:]))

	quadLayout := gpu.VertexBufferLayout{
		Stride: 8, Step: gpu.StepModeVertex,
		Attributes: []gpu.VertexAttribute{{Format: gpu.VertexFormatFloat32x2, Offset: 0, Location: 0}},
	}

	if err := r.buildBackground(dev); err != nil {
		return nil, err
	}

	r.circles = r.buildBatchPipeline(dev, "circle", draw.CircleWGSL, quadLayout, gpu.VertexBufferLayout{
		Stride: 16, Step: gpu.StepModeInstance,
		Attributes: []gpu.VertexAttribute{
			{Format: gpu.VertexFormatFloat32x2, Offset: 0, Location: 1},
			{Format: gpu.VertexFormatFloat32, Offset: 8, Location: 2},
			{Format: gpu.VertexFormatUnorm8x4, Offset: 12, Location: 3},
		},
	})
	r.solidCircles = r.buildBatchPipeline(dev, "solid_circle", draw.SolidCircleWGSL, quadLayout, gpu.VertexBufferLayout{
		Stride: 24, Step: gpu.StepModeInstance,
		Attributes: []gpu.VertexAttribute{
			{Format: gpu.VertexFormatFloat32x4, Offset: 0, Location: 1},
			{Format: gpu.VertexFormatFloat32, Offset: 16, Location: 2},
			{Format: gpu.VertexFormatUnorm8x4, Offset: 20, Location: 3},
		},
	})
	r.solidCapsules = r.buildBatchPipeline(dev, "solid_capsule", draw.SolidCapsuleWGSL, quadLayout, gpu.VertexBufferLayout{
		Stride: 28, Step: gpu.StepModeInstance,
		Attributes: []gpu.VertexAttribute{
			{Format: gpu.VertexFormatFloat32x4, Offset: 0, Location: 1},
			{Format: gpu.VertexFormatFloat32, Offset: 16, Location: 2},
			{Format: gpu.VertexFormatFloat32, Offset: 20, Location: 3},
			{Format: gpu.VertexFormatUnorm8x4, Offset: 24, Location: 4},
		},
	})
	r.solidPolygons = r.buildBatchPipeline(dev, "solid_polygon", draw.SolidPolygonWGSL, quadLayout, gpu.VertexBufferLayout{
		Stride: 92, Step: gpu.StepModeInstance,
		Attributes: []gpu.VertexAttribute{
			{Format: gpu.VertexFormatFloat32x4, Offset: 0, Location: 1},
			{Format: gpu.VertexFormatFloat32x4, Offset: 16, Location: 2},
			{Format: gpu.VertexFormatFloat32x4, Offset: 32, Location: 3},
			{Format: gpu.VertexFormatFloat32x4, Offset: 48, Location: 4},
			{Format: gpu.VertexFormatFloat32x4, Offset: 64, Location: 5},
			{Format: gpu.VertexFormatSint32, Offset: 80, Location: 6},
			{Format: gpu.VertexFormatFloat32, Offset: 84, Location: 7},
			{Format: gpu.VertexFormatUnorm8x4, Offset: 88, Location: 8},
		},
	})
	r.points = r.buildBatchPipeline(dev, "points", draw.PointsWGSL, quadLayout, gpu.VertexBufferLayout{
		Stride: 16, Step: gpu.StepModeInstance,
		Attributes: []gpu.VertexAttribute{
			{Format: gpu.VertexFormatFloat32x2, Offset: 0, Location: 1},
			{Format: gpu.VertexFormatFloat32, Offset: 8, Location: 2},
			{Format: gpu.VertexFormatUnorm8x4, Offset: 12, Location: 3},
		},
	})

	r.lines = r.buildVertexPipeline(dev, "lines", draw.LinesWGSL, gpu.TopologyLineList, gpu.VertexBufferLayout{
		Stride: 12, Step: gpu.StepModeVertex,
		Attributes: []gpu.VertexAttribute{
			{Format: gpu.VertexFormatFloat32x2, Offset: 0, Location: 0},
			{Format: gpu.VertexFormatUnorm8x4, Offset: 8, Location: 1},
		},
	})

	if err := r.buildText(dev, atlas); err != nil {
		return nil, err
	}

	return r, nil
}

func (r *Renderer) buildBackground(dev gpu.Device) error {
	shader := dev.CreateShader("background", draw.BackgroundWGSL)
	pipeline := dev.CreatePipeline(gpu.PipelineDesc{
		Label:    "background",
		Shader:   shader,
		Bindings: []gpu.BindingLayout{{Binding: 0, Kind: gpu.BindingUniform}},
		Topology: gpu.TopologyTriangleList,
		Blend:    gpu.BlendNone,
	})
	uniform := dev.CreateBuffer("background-uniforms", gpu.BufferUsageUniform, int(unsafe.Sizeof(backgroundUniforms{})))
	bindGroup := dev.CreateBindGroup(gpu.BindGroupDesc{
		Label: "background", Pipeline: pipeline,
		Entries: []gpu.BindGroupEntry{{Binding: 0, Buffer: uniform, BufferSize: int(unsafe.Sizeof(backgroundUniforms{}))}},
	})
	r.background.pipeline, r.background.uniform, r.background.bindGroup = pipeline, uniform, bindGroup
	return nil
}

func (r *Renderer) buildBatchPipeline(dev gpu.Device, label, wgsl string, quadLayout, instanceLayout gpu.VertexBufferLayout) batchPipeline {
	shader := dev.CreateShader(label, wgsl)
	pipeline := dev.CreatePipeline(gpu.PipelineDesc{
		Label:    label,
		Shader:   shader,
		Buffers:  []gpu.VertexBufferLayout{quadLayout, instanceLayout},
		Bindings: []gpu.BindingLayout{{Binding: 0, Kind: gpu.BindingUniform}},
		Topology: gpu.TopologyTriangleList,
		Blend:    gpu.BlendAlpha,
	})
	uniform := dev.CreateBuffer(label+"-uniforms", gpu.BufferUsageUniform, int(unsafe.Sizeof(uniforms80{})))
	bindGroup := dev.CreateBindGroup(gpu.BindGroupDesc{
		Label: label, Pipeline: pipeline,
		Entries: []gpu.BindGroupEntry{{Binding: 0, Buffer: uniform, BufferSize: int(unsafe.Sizeof(uniforms80{}))}},
	})
	return batchPipeline{pipeline: pipeline, uniform: uniform, bindGroup: bindGroup}
}

func (r *Renderer) buildVertexPipeline(dev gpu.Device, label, wgsl string, topology gpu.Topology, layout gpu.VertexBufferLayout) vertexPipeline {
	shader := dev.CreateShader(label, wgsl)
	pipeline := dev.CreatePipeline(gpu.PipelineDesc{
		Label:    label,
		Shader:   shader,
		Buffers:  []gpu.VertexBufferLayout{layout},
		Bindings: []gpu.BindingLayout{{Binding: 0, Kind: gpu.BindingUniform}},
		Topology: topology,
		Blend:    gpu.BlendAlpha,
	})
	uniform := dev.CreateBuffer(label+"-uniforms", gpu.BufferUsageUniform, int(unsafe.Sizeof(uniforms80{})))
	bindGroup := dev.CreateBindGroup(gpu.BindGroupDesc{
		Label: label, Pipeline: pipeline,
		Entries: []gpu.BindGroupEntry{{Binding: 0, Buffer: uniform, BufferSize: int(unsafe.Sizeof(uniforms80{}))}},
	})
	return vertexPipeline{pipeline: pipeline, uniform: uniform, bindGroup: bindGroup}
}

// textBytesPerRowAlign pads the atlas rows to the alignment a buffer
// copy would need, so a host may upload through either path.
const textBytesPerRowAlign = 256

func (r *Renderer) buildText(dev gpu.Device, atlas *draw.Atlas) error {
	shader := dev.CreateShader("text", draw.TextWGSL)
	pipeline := dev.CreatePipeline(gpu.PipelineDesc{
		Label:  "text",
		Shader: shader,
		Buffers: []gpu.VertexBufferLayout{{
			Stride: 20, Step: gpu.StepModeVertex,
			Attributes: []gpu.VertexAttribute{
				{Format: gpu.VertexFormatFloat32x2, Offset: 0, Location: 0},
				{Format: gpu.VertexFormatFloat32x2, Offset: 8, Location: 1},
				{Format: gpu.VertexFormatUnorm8x4, Offset: 16, Location: 2},
			},
		}},
		Bindings: []gpu.BindingLayout{
			{Binding: 0, Kind: gpu.BindingUniform},
			{Binding: 1, Kind: gpu.BindingTexture2D},
			{Binding: 2, Kind: gpu.BindingSampler},
		},
		Topology: gpu.TopologyTriangleList,
		Blend:    gpu.BlendAlpha,
	})
	uniform := dev.CreateBuffer("text-uniforms", gpu.BufferUsageUniform, int(unsafe.Sizeof(uniforms80{})))
	texture := dev.CreateTexture("text-atlas", atlas.Width, atlas.Height, gpu.TextureFormatR8Unorm)

	rowBytes := atlas.Width
	padded := rowBytes
	if rem := padded % textBytesPerRowAlign; rem != 0 {
		padded += textBytesPerRowAlign - rem
	}
	if padded == rowBytes {
		dev.WriteTexture(texture, atlas.Width, atlas.Height, rowBytes, atlas.Pixels)
	} else {
		buf := make([]byte, padded*atlas.Height)
		for y := range atlas.Height {
			copy(buf[y*padded:y*padded+rowBytes], atlas.Pixels[y*rowBytes:(y+1)*rowBytes])
		}
		dev.WriteTexture(texture, atlas.Width, atlas.Height, padded, buf)
	}

	bindGroup := dev.CreateBindGroup(gpu.BindGroupDesc{
		Label: "text", Pipeline: pipeline,
		Entries: []gpu.BindGroupEntry{
			{Binding: 0, Buffer: uniform, BufferSize: int(unsafe.Sizeof(uniforms80{}))},
			{Binding: 1, Texture: texture},
			{Binding: 2, Sampler: true},
		},
	})
	r.text = vertexPipeline{pipeline: pipeline, uniform: uniform, bindGroup: bindGroup}
	return nil
}

// ensureCapacity (re)creates b if data does not fit in capacity, and writes
// data into it. It returns the buffer and its current byte capacity.
func ensureCapacity(dev gpu.Device, label string, usage gpu.BufferUsage, b gpu.Buffer, capacity int, data []byte) (gpu.Buffer, int) {
	if len(data) == 0 {
		return b, capacity
	}
	if len(data) > capacity {
		if b != nil {
			b.Release()
		}
		capacity = 2 * len(data) // headroom, so a growing scene reallocates rarely
		b = dev.CreateBuffer(label, usage, capacity)
	}
	dev.WriteBuffer(b, 0, data)
	return b, capacity
}

// projectionUniform builds the 80-byte uniform for a batch or line
// pipeline, from the camera's projection at kind's z bias.
func projectionUniform(camera *samples.Camera, zBias float32) []byte {
	u := uniforms80{projection: camera.BuildProjectionMatrix(zBias), pixelScale: camera.PixelScale()}
	return asBytes([]uniforms80{u})
}

// pixelProjection builds a pixel-space orthographic matrix for the text/UI
// pipeline: x right, y down, origin top-left, matching how UICommand and
// draw.TextItem coordinates are produced.
func pixelProjection(width, height int) [16]float32 {
	w, h := float32(width), float32(height)
	var m [16]float32
	m[0] = 2.0 / w
	m[5] = -2.0 / h
	m[10] = 1.0
	m[12] = -1.0
	m[13] = 1.0
	m[15] = 1.0
	return m
}

// Frame draws the background and the batches in draw.FlushOrder into one
// render pass; the UI commands go out with the text batch, last.
func (r *Renderer) Frame(camera *samples.Camera, batches *draw.Batches, ui []UICommand, timeSeconds float64) error {
	frame, ok := r.dev.BeginFrame()
	if !ok {
		return nil
	}
	width, height := frame.Size()
	if width == 0 || height == 0 {
		frame.End()
		return nil
	}

	pass := frame.BeginPass([4]float32{0.2, 0.2, 0.2, 1})

	bg := backgroundUniforms{
		time:       float32(timeSeconds),
		resolution: [2]float32{float32(width), float32(height)},
		baseColor:  [3]float32{0.2, 0.2, 0.2},
	}
	r.dev.WriteBuffer(r.background.uniform, 0, asBytes([]backgroundUniforms{bg}))
	pass.SetPipeline(r.background.pipeline)
	pass.SetBindGroup(0, r.background.bindGroup)
	pass.Draw(6, 1)

	for _, kind := range draw.FlushOrder {
		if kind == draw.KindText {
			r.drawText(pass, width, height, batches.Text, ui)
			continue
		}
		if err := r.drawKind(pass, camera, kind, batches); err != nil {
			pass.End()
			frame.End()
			return err
		}
	}

	pass.End()
	frame.End()
	return nil
}

func (r *Renderer) drawKind(pass gpu.Pass, camera *samples.Camera, kind draw.Kind, batches *draw.Batches) error {
	switch kind {
	case draw.KindSolidCircles:
		return r.drawInstanced(pass, &r.solidCircles, camera, kind, asBytes(batches.SolidCircles), 24, len(batches.SolidCircles))
	case draw.KindSolidCapsules:
		return r.drawInstanced(pass, &r.solidCapsules, camera, kind, asBytes(batches.SolidCapsules), 28, len(batches.SolidCapsules))
	case draw.KindSolidPolygons:
		return r.drawInstanced(pass, &r.solidPolygons, camera, kind, asBytes(batches.SolidPolygons), 92, len(batches.SolidPolygons))
	case draw.KindCircles:
		return r.drawInstanced(pass, &r.circles, camera, kind, asBytes(batches.Circles), 16, len(batches.Circles))
	case draw.KindPoints:
		return r.drawInstanced(pass, &r.points, camera, kind, asBytes(batches.Points), 16, len(batches.Points))
	case draw.KindLines:
		return r.drawLines(pass, camera, batches.Lines)
	default:
		return fmt.Errorf("render: unhandled batch kind %d", kind)
	}
}

func (r *Renderer) drawInstanced(pass gpu.Pass, bp *batchPipeline, camera *samples.Camera, kind draw.Kind, data []byte, stride, count int) error {
	if count == 0 {
		return nil
	}
	r.dev.WriteBuffer(bp.uniform, 0, projectionUniform(camera, kind.ZBias()))
	bp.instances, bp.capacity = ensureCapacity(r.dev, "instances", gpu.BufferUsageVertex, bp.instances, bp.capacity, data)

	pass.SetPipeline(bp.pipeline)
	pass.SetBindGroup(0, bp.bindGroup)
	pass.SetVertexBuffer(0, r.quad, 0, int(unsafe.Sizeof(draw.QuadVertices)))
	pass.SetVertexBuffer(1, bp.instances, 0, len(data))
	pass.Draw(6, count)
	return nil
}

func (r *Renderer) drawLines(pass gpu.Pass, camera *samples.Camera, lines []draw.VertexData) error {
	if len(lines) == 0 {
		return nil
	}
	data := asBytes(lines)
	r.dev.WriteBuffer(r.lines.uniform, 0, projectionUniform(camera, draw.KindLines.ZBias()))
	r.lines.vertices, r.lines.capacity = ensureCapacity(r.dev, "lines", gpu.BufferUsageVertex, r.lines.vertices, r.lines.capacity, data)

	pass.SetPipeline(r.lines.pipeline)
	pass.SetBindGroup(0, r.lines.bindGroup)
	pass.SetVertexBuffer(0, r.lines.vertices, 0, len(data))
	pass.Draw(len(lines), 1)
	return nil
}

// textSegment is one run of text vertices drawn under one scissor rect.
type textSegment struct {
	first, count int
	scissor      [4]int
}

// drawText draws the overlay text, then the microui commands, through the
// text pipeline. Every vertex of the frame goes into one staging slice
// that is written once: a queued write lands before any draw of the
// frame executes, so a buffer must not be rewritten between two draws.
func (r *Renderer) drawText(pass gpu.Pass, width, height int, items []draw.TextItem, commands []UICommand) {
	r.uiVertices = r.uiVertices[:0]
	r.segments = r.segments[:0]

	full := [4]int{0, 0, width, height}
	for _, item := range items {
		r.uiVertices = r.atlas.AppendQuads(r.uiVertices, item)
	}
	r.cutSegment(full)

	scissor := full
	for _, cmd := range commands {
		switch cmd.Kind {
		case UIClip:
			r.cutSegment(scissor)
			x, y, w, h := clampScissor(cmd.Rect, width, height)
			scissor = [4]int{x, y, w, h}
		case UIRect:
			r.uiVertices = r.atlas.AppendRectQuad(r.uiVertices, float32(cmd.Rect[0]), float32(cmd.Rect[1]), float32(cmd.Rect[2]), float32(cmd.Rect[3]), cmd.Color)
		case UIText:
			r.uiVertices = r.atlas.AppendQuads(r.uiVertices, draw.TextItem{X: float32(cmd.X), Y: float32(cmd.Y), Text: cmd.Text, Color: cmd.Color})
		case UIIcon:
			r.uiVertices = r.appendIcon(r.uiVertices, cmd)
		}
	}
	r.cutSegment(scissor)
	if len(r.uiVertices) == 0 {
		return
	}

	data := asBytes(r.uiVertices)
	uniform := uniforms80{projection: pixelProjection(width, height)}
	r.dev.WriteBuffer(r.text.uniform, 0, asBytes([]uniforms80{uniform}))
	r.text.vertices, r.text.capacity = ensureCapacity(r.dev, "text", gpu.BufferUsageVertex, r.text.vertices, r.text.capacity, data)

	const vertexSize = int(unsafe.Sizeof(draw.TextVertex{}))
	pass.SetPipeline(r.text.pipeline)
	pass.SetBindGroup(0, r.text.bindGroup)
	for _, seg := range r.segments {
		pass.SetScissor(seg.scissor[0], seg.scissor[1], seg.scissor[2], seg.scissor[3])
		pass.SetVertexBuffer(0, r.text.vertices, seg.first*vertexSize, seg.count*vertexSize)
		pass.Draw(seg.count, 1)
	}
	pass.SetScissor(0, 0, width, height)
}

// cutSegment closes the run of vertices appended since the last cut, to
// be drawn under scissor.
func (r *Renderer) cutSegment(scissor [4]int) {
	first := 0
	if n := len(r.segments); n > 0 {
		last := r.segments[n-1]
		first = last.first + last.count
	}
	if count := len(r.uiVertices) - first; count > 0 {
		r.segments = append(r.segments, textSegment{first: first, count: count, scissor: scissor})
	}
}

// iconGlyphs maps a microui icon id to the glyph this port draws instead.
var iconGlyphs = map[int]rune{
	1: 'x', // MU_ICON_CLOSE
	2: 'v', // MU_ICON_CHECK
	3: '+', // MU_ICON_COLLAPSED
	4: '-', // MU_ICON_EXPANDED
}

func (r *Renderer) appendIcon(out []draw.TextVertex, cmd UICommand) []draw.TextVertex {
	ch, ok := iconGlyphs[cmd.Icon]
	if !ok {
		return out
	}
	s := string(ch)
	x := cmd.Rect[0] + (cmd.Rect[2]-r.atlas.TextWidth(s))/2
	y := cmd.Rect[1] + (cmd.Rect[3]-r.atlas.TextHeight())/2
	return r.atlas.AppendQuads(out, draw.TextItem{X: float32(x), Y: float32(y), Text: s, Color: cmd.Color})
}

// clampScissor clamps a microui clip rect to the frame, since microui sends
// an unclipped rect at the end that can be negative or oversized.
func clampScissor(rect [4]int, width, height int) (x, y, w, h int) {
	x0, y0, x1, y1 := rect[0], rect[1], rect[0]+rect[2], rect[1]+rect[3]
	if x0 < 0 {
		x0 = 0
	}
	if y0 < 0 {
		y0 = 0
	}
	if x1 > width {
		x1 = width
	}
	if y1 > height {
		y1 = height
	}
	if x1 < x0 {
		x1 = x0
	}
	if y1 < y0 {
		y1 = y0
	}
	return x0, y0, x1 - x0, y1 - y0
}
