// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

package samples_test

import (
	"fmt"
	"testing"

	"github.com/dhannyell/dbox2d"
	"github.com/dhannyell/dbox2d/samples"
)

// TestSamplesStepWithoutPanic pins that every registered scene runs headless.
func TestSamplesStepWithoutPanic(t *testing.T) {
	for _, entry := range samples.Entries() {
		ctx := samples.NewSampleContext()
		sample := entry.Create(ctx)
		for range 60 {
			sample.Step()
		}
		sample.Destroy()
	}
}

func TestSingleBoxChecksum(t *testing.T) {
	ctx := samples.NewSampleContext()
	sample := samples.NewSingleBox(ctx).(*samples.SingleBox)
	for range 60 {
		sample.Step()
	}
	got := dbox2d.Checksum(sample.WorldId)
	if got != singleBoxChecksum {
		t.Fatalf("got checksum 0x%x, want 0x%x", got, singleBoxChecksum)
	}
	sample.Destroy()
}

func TestVerticalStackChecksum(t *testing.T) {
	ctx := samples.NewSampleContext()
	sample := samples.NewVerticalStack(ctx).(*samples.VerticalStack)
	for range 60 {
		sample.Step()
	}
	got := dbox2d.Checksum(sample.WorldId)
	if got != verticalStackChecksum {
		t.Fatalf("got checksum 0x%x, want 0x%x", got, verticalStackChecksum)
	}
	sample.Destroy()
}

func TestTumblerChecksum(t *testing.T) {
	ctx := samples.NewSampleContext()
	sample := samples.NewTumbler(ctx).(*samples.Tumbler)
	for range 60 {
		sample.Step()
	}
	got := dbox2d.Checksum(sample.WorldId)
	if got != tumblerChecksum {
		t.Fatalf("got checksum 0x%x, want 0x%x", got, tumblerChecksum)
	}
	sample.Destroy()
}

// ExampleEntries prints the first registered sample.
func ExampleEntries() {
	e := samples.Entries()[0]
	fmt.Printf("%s / %s\n", e.Category, e.Name)
	// Output: Benchmark / Barrel
}

func TestLargePyramidChecksum(t *testing.T) {
	ctx := samples.NewSampleContext()
	sample := samples.NewLargePyramid(ctx).(*samples.LargePyramid)
	for range 60 {
		sample.Step()
	}
	got := dbox2d.Checksum(sample.WorldId)
	if got != largePyramidChecksum {
		t.Fatalf("got checksum 0x%x, want 0x%x", got, largePyramidChecksum)
	}
	sample.Destroy()
}

func TestBridgeChecksum(t *testing.T) {
	ctx := samples.NewSampleContext()
	sample := samples.NewBridge(ctx).(*samples.Bridge)
	for range 60 {
		sample.Step()
	}
	got := dbox2d.Checksum(sample.WorldId)
	if got != bridgeChecksum {
		t.Fatalf("got checksum 0x%x, want 0x%x", got, bridgeChecksum)
	}
	sample.Destroy()
}

func TestRagdollChecksum(t *testing.T) {
	ctx := samples.NewSampleContext()
	sample := samples.NewRagdoll(ctx).(*samples.Ragdoll)
	for range 60 {
		sample.Step()
	}
	got := dbox2d.Checksum(sample.WorldId)
	if got != ragdollChecksum {
		t.Fatalf("got checksum 0x%x, want 0x%x", got, ragdollChecksum)
	}
	sample.Destroy()
}

func TestPinnedChecksumsWithFourWorkers(t *testing.T) {
	tests := []struct {
		name  string
		entry func(*samples.SampleContext) samples.Sample
		want  uint64
	}{
		{"Single Box", func(ctx *samples.SampleContext) samples.Sample { return samples.NewSingleBox(ctx) }, singleBoxChecksum},
		{"Vertical Stack", func(ctx *samples.SampleContext) samples.Sample { return samples.NewVerticalStack(ctx) }, verticalStackChecksum},
		{"Tumbler", func(ctx *samples.SampleContext) samples.Sample { return samples.NewTumbler(ctx) }, tumblerChecksum},
		{"Large Pyramid", func(ctx *samples.SampleContext) samples.Sample { return samples.NewLargePyramid(ctx) }, largePyramidChecksum},
		{"Bridge", func(ctx *samples.SampleContext) samples.Sample { return samples.NewBridge(ctx) }, bridgeChecksum},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			ctx := samples.NewSampleContext()
			ctx.Settings.WorkerCount = 4
			sample := tt.entry(ctx)
			for range 60 {
				sample.Step()
			}
			world := sample.(interface{ World() dbox2d.WorldId }).World()
			got := dbox2d.Checksum(world)
			if got != tt.want {
				t.Fatalf("got checksum 0x%x, want 0x%x", got, tt.want)
			}
			sample.Destroy()
		})
	}
}
