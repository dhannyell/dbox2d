//go:build !windows

package dbox2d_test

// Off Windows the Go monotonic clock is already the high-resolution source
// that src/timer.c reaches for, so the harness uses it directly.

import "time"

var benchClockOrigin = time.Now()

// benchTicks reads the monotonic clock in nanoseconds.
func benchTicks() int64 { return int64(time.Since(benchClockOrigin)) }

// benchMilliseconds converts a nanosecond span to milliseconds.
func benchMilliseconds(ticks int64) float64 { return float64(ticks) / 1e6 }

// benchClockName reports the timing source, recorded in the output.
func benchClockName() string { return "time.Since" }
