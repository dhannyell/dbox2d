//go:build cgo

// Command native builds a desktop binary hosting the samples module over
// wgpu-native and GLFW.
package main

import (
	"flag"
	"fmt"
	"os"
	"runtime"

	"github.com/dhannyell/dbox2d/samples"
	"github.com/dhannyell/dbox2d/samples/internal/host/native"
)

func init() {
	runtime.LockOSThread()
}

func main() {
	// The Replay viewer opens this recording, see samples.ReplayFile.
	flag.StringVar(&samples.ReplayFile, "replay", "", "recording file for the Replay / Viewer sample")
	flag.Parse()

	if err := native.Run(); err != nil {
		fmt.Fprintln(os.Stderr, "native:", err)
		os.Exit(1)
	}
}
