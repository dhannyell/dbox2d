//go:build !windows

package native

import "github.com/dhannyell/webgpu/wgpu"

// preferredBackend lets wgpu choose outside Windows.
const preferredBackend = wgpu.BackendTypeUndefined
