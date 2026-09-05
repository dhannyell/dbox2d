//go:build js && wasm

// Command web builds to app.wasm: a browser tab hosting the samples module
// over WebGPU.
package main

import "github.com/dhannyell/dbox2d/samples/internal/host/wasm"

func main() {
	wasm.Run()
}
