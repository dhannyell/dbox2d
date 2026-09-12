//go:build windows

package dbox2d_test

// The Go monotonic clock on Windows quantises to the system tick, which
// measured 500 microseconds here: a step of a few milliseconds reads with
// double-digit percent error and a cheap step reads as zero. The reference
// harness times with QueryPerformanceCounter through b2GetTicks in
// src/timer.c, so the port times with the same counter. Both sides then
// share one measurement mechanism, not just one protocol.

import (
	"syscall"
	"unsafe"
)

var (
	benchKernel32   = syscall.NewLazyDLL("kernel32.dll")
	benchQueryCount = benchKernel32.NewProc("QueryPerformanceCounter")
	benchQueryFreq  = benchKernel32.NewProc("QueryPerformanceFrequency")
	benchTickHz     = benchQueryPerformanceFrequency()
)

func benchQueryPerformanceFrequency() int64 {
	var hz int64
	_, _, _ = benchQueryFreq.Call(uintptr(unsafe.Pointer(&hz)))
	if hz == 0 {
		return 1
	}
	return hz
}

// benchTicks reads the performance counter.
func benchTicks() int64 {
	var ticks int64
	_, _, _ = benchQueryCount.Call(uintptr(unsafe.Pointer(&ticks)))
	return ticks
}

// benchMilliseconds converts a tick span to milliseconds.
func benchMilliseconds(ticks int64) float64 {
	return float64(ticks) * 1e3 / float64(benchTickHz)
}

// benchClockName reports the timing source, recorded in the output.
func benchClockName() string { return "QueryPerformanceCounter" }
