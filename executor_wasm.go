//go:build (js || wasip1) && !tinygo.wasm.threads

package b2

const (
	maxWorkers         = 64
	platformMaxWorkers = 1
)
