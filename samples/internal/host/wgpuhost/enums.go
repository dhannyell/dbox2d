//go:build cgo || js

package wgpuhost

import (
	"github.com/dhannyell/webgpu/wgpu"

	"github.com/dhannyell/dbox2d/samples/internal/gpu"
)

func toBufferUsage(u gpu.BufferUsage) wgpu.BufferUsage {
	switch u {
	case gpu.BufferUsageUniform:
		return wgpu.BufferUsageUniform | wgpu.BufferUsageCopyDst
	default:
		return wgpu.BufferUsageVertex | wgpu.BufferUsageCopyDst
	}
}

func toTextureFormat(f gpu.TextureFormat) wgpu.TextureFormat {
	switch f {
	case gpu.TextureFormatBGRA8Unorm:
		return wgpu.TextureFormatBGRA8Unorm
	case gpu.TextureFormatRGBA8Unorm:
		return wgpu.TextureFormatRGBA8Unorm
	default:
		return wgpu.TextureFormatR8Unorm
	}
}

func fromWgpuFormat(f wgpu.TextureFormat) gpu.TextureFormat {
	switch f {
	case wgpu.TextureFormatBGRA8Unorm:
		return gpu.TextureFormatBGRA8Unorm
	case wgpu.TextureFormatRGBA8Unorm:
		return gpu.TextureFormatRGBA8Unorm
	default:
		return gpu.TextureFormatR8Unorm
	}
}

func toVertexFormat(f gpu.VertexFormat) wgpu.VertexFormat {
	switch f {
	case gpu.VertexFormatFloat32x2:
		return wgpu.VertexFormatFloat32x2
	case gpu.VertexFormatFloat32x4:
		return wgpu.VertexFormatFloat32x4
	case gpu.VertexFormatUnorm8x4:
		return wgpu.VertexFormatUnorm8x4
	case gpu.VertexFormatSint32:
		return wgpu.VertexFormatSint32
	default:
		return wgpu.VertexFormatFloat32
	}
}

func toStepMode(s gpu.StepMode) wgpu.VertexStepMode {
	if s == gpu.StepModeInstance {
		return wgpu.VertexStepModeInstance
	}
	return wgpu.VertexStepModeVertex
}

func toTopology(t gpu.Topology) wgpu.PrimitiveTopology {
	if t == gpu.TopologyLineList {
		return wgpu.PrimitiveTopologyLineList
	}
	return wgpu.PrimitiveTopologyTriangleList
}

func toBlendState(b gpu.Blend) *wgpu.BlendState {
	if b != gpu.BlendAlpha {
		return nil
	}
	state := wgpu.BlendStateAlphaBlending
	return &state
}
