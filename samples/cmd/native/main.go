//go:build cgo

// Command native builds a desktop binary hosting the samples module over
// wgpu-native and GLFW.
package main

import (
	"fmt"
	"os"
	"runtime"

	"github.com/dhannyell/dbox2d/samples/internal/host/native"
)

func init() {
	runtime.LockOSThread()
}

func main() {
	if err := native.Run(); err != nil {
		fmt.Fprintln(os.Stderr, "native:", err)
		os.Exit(1)
	}
}
