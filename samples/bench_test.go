// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

package samples_test

import (
	"strconv"
	"testing"

	"github.com/dhannyell/dbox2d/samples"
)

// BenchmarkTumbler measures the Tumbler scene with different worker counts.
func BenchmarkTumbler(b *testing.B) {
	for _, workers := range []int{1, 8} {
		b.Run("workers="+strconv.Itoa(workers), func(b *testing.B) {
			b.ReportAllocs()
			ctx := samples.NewSampleContext()
			ctx.Settings.WorkerCount = workers
			sample := samples.NewTumbler(ctx)
			b.ResetTimer()
			for range b.N {
				for range 60 {
					sample.Step()
				}
			}
			b.StopTimer()
			sample.Destroy()
		})
	}
}

// BenchmarkLargePyramid measures the Large Pyramid scene with different worker counts.
func BenchmarkLargePyramid(b *testing.B) {
	for _, workers := range []int{1, 8} {
		b.Run("workers="+strconv.Itoa(workers), func(b *testing.B) {
			b.ReportAllocs()
			ctx := samples.NewSampleContext()
			ctx.Settings.WorkerCount = workers
			sample := samples.NewLargePyramid(ctx)
			b.ResetTimer()
			for range b.N {
				for range 60 {
					sample.Step()
				}
			}
			b.StopTimer()
			sample.Destroy()
		})
	}
}
