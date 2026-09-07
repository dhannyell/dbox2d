package native

import "github.com/dhannyell/webgpu/wgpu"

// preferredBackend is D3D12 on Windows: its swapchain honours the frame
// latency of the surface, so the loop blocks once per vsync. Vulkan on
// Windows blocks in acquire with several milliseconds of jitter.
const preferredBackend = wgpu.BackendTypeD3D12
