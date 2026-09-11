// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

package samples_test

import (
	"strconv"
	"testing"

	"github.com/dhannyell/dbox2d/samples"
)

// stepsPerOp is one simulated second at 60 Hz.
const stepsPerOp = 60

// steadyWarmupSteps skips the opening collapse of a scene, which costs
// differently from the settled scene.
const steadyWarmupSteps = 240

// benchPhase names the part of a scene a benchmark measures.
type benchPhase struct {
	name   string
	warmup int
}

var benchPhases = []benchPhase{
	{name: "start", warmup: 0},
	{name: "steady", warmup: steadyWarmupSteps},
}

// benchScene rebuilds the scene every iteration. Carrying one world forward
// would make a faster build measure a later, cheaper part of the scene.
func benchScene(b *testing.B, newSample func(*samples.SampleContext) samples.Sample, steps int) {
	b.Helper()
	for _, workers := range []int{1, 8} {
		b.Run("workers="+strconv.Itoa(workers), func(b *testing.B) {
			for _, phase := range benchPhases {
				b.Run("phase="+phase.name, func(b *testing.B) {
					b.ReportAllocs()
					ctx := samples.NewSampleContext()
					ctx.Settings.WorkerCount = workers
					b.ResetTimer()
					for range b.N {
						b.StopTimer()
						sample := newSample(ctx)
						for range phase.warmup {
							sample.Step()
						}
						b.StartTimer()
						for range steps {
							sample.Step()
						}
						b.StopTimer()
						sample.Destroy()
					}
				})
			}
		})
	}
}

// BenchmarkTumbler measures the Tumbler scene with different worker counts.
func BenchmarkTumbler(b *testing.B) {
	benchScene(b, samples.NewTumbler, stepsPerOp)
}

// BenchmarkLargePyramid measures the Large Pyramid scene with different worker counts.
func BenchmarkLargePyramid(b *testing.B) {
	benchScene(b, samples.NewLargePyramid, stepsPerOp)
}

// BenchmarkBarrel measures the Barrel scene with different worker counts.
func BenchmarkBarrel(b *testing.B) {
	benchScene(b, samples.NewBarrel, stepsPerOp)
}

// BenchmarkCreateDestroy measures the CreateDestroy scene with different worker counts.
func BenchmarkCreateDestroy(b *testing.B) {
	// Fewer steps than the other scene benchmarks: Step already runs 10
	// create/destroy/step cycles, so 60 calls would be roughly 10x the work.
	const createDestroyStepsPerOp = 6
	benchScene(b, samples.NewCreateDestroy, createDestroyStepsPerOp)
}
