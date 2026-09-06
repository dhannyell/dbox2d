//go:build cgo

package native

import (
	"fmt"

	"github.com/go-gl/glfw/v3.4/glfw"

	"github.com/dhannyell/webgpu/wgpu"
	"github.com/dhannyell/webgpu/wgpuglfw"
)

// gpuState owns the instance, surface and device wgpuhost.Device wraps, so
// Run can reconfigure and release them without reaching into the seam.
type gpuState struct {
	instance *wgpu.Instance
	surface  *wgpu.Surface
	device   *wgpu.Device
	queue    *wgpu.Queue
	config   *wgpu.SurfaceConfiguration
}

func newGPUState(window *glfw.Window) (*gpuState, error) {
	g := &gpuState{}

	g.instance = wgpu.CreateInstance(nil)
	if !g.instance.IsValid() {
		return nil, fmt.Errorf("native: create WebGPU instance")
	}
	g.surface = g.instance.CreateSurface(wgpuglfw.GetSurfaceDescriptor(window))
	if !g.surface.IsValid() {
		g.instance.Release()
		return nil, fmt.Errorf("native: create WebGPU surface")
	}

	adapter, err := g.instance.RequestAdapter(&wgpu.RequestAdapterOptions{
		CompatibleSurface: g.surface,
		PowerPreference:   wgpu.PowerPreferenceHighPerformance,
	})
	if err != nil {
		g.surface.Release()
		g.instance.Release()
		return nil, fmt.Errorf("native: request adapter: %w", err)
	}

	g.device, err = adapter.RequestDevice(nil)
	if err != nil {
		adapter.Release()
		g.surface.Release()
		g.instance.Release()
		return nil, fmt.Errorf("native: request device: %w", err)
	}
	g.queue = g.device.GetQueue()

	caps := g.surface.GetCapabilities(adapter)
	format, err := chooseSurfaceFormat(caps.Formats)
	if err != nil {
		adapter.Release()
		g.destroy()
		return nil, err
	}
	alphaMode, err := chooseAlphaMode(caps.AlphaModes)
	if err != nil {
		adapter.Release()
		g.destroy()
		return nil, err
	}

	g.config = &wgpu.SurfaceConfiguration{
		Usage:       wgpu.TextureUsageRenderAttachment,
		Format:      format,
		PresentMode: wgpu.PresentModeFifo,
		AlphaMode:   alphaMode,
	}

	adapter.Release()
	return g, nil
}

// chooseSurfaceFormat prefers BGRA8Unorm, matching the reference renderer's
// assumption about the swapchain's byte order.
func chooseSurfaceFormat(formats []wgpu.TextureFormat) (wgpu.TextureFormat, error) {
	for _, f := range formats {
		if f == wgpu.TextureFormatBGRA8Unorm {
			return f, nil
		}
	}
	for _, f := range formats {
		if f == wgpu.TextureFormatRGBA8Unorm {
			return f, nil
		}
	}
	return 0, fmt.Errorf("native: surface requires BGRA8Unorm or RGBA8Unorm")
}

func chooseAlphaMode(modes []wgpu.CompositeAlphaMode) (wgpu.CompositeAlphaMode, error) {
	if len(modes) == 0 {
		return 0, fmt.Errorf("native: surface reports no alpha modes")
	}
	for _, m := range modes {
		if m == wgpu.CompositeAlphaModeOpaque {
			return m, nil
		}
	}
	return modes[0], nil
}

// configure reconfigures the surface at a framebuffer size. A minimized
// window reports a zero size, which must never reach wgpu-native.
func (g *gpuState) configure(fbW, fbH int) bool {
	if fbW <= 0 || fbH <= 0 {
		return false
	}
	g.config.Width = uint32(fbW)
	g.config.Height = uint32(fbH)
	g.surface.Configure(g.device, g.config)
	return true
}

// destroy releases every handle. Release is idempotent in the fork, so a
// partial init can call destroy safely.
func (g *gpuState) destroy() {
	if g.surface != nil {
		g.surface.Release()
		g.surface = nil
	}
	if g.queue != nil {
		g.queue.Release()
		g.queue = nil
	}
	if g.device != nil {
		g.device.Release()
		g.device = nil
	}
	if g.instance != nil {
		g.instance.Release()
		g.instance = nil
	}
}
