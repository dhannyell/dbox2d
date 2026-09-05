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

// singleBoxChecksum pins the world state after 60 steps of Single Box; a
// change here means the physics changed.
const singleBoxChecksum uint64 = 0x59758871b1c2509e
const verticalStackChecksum uint64 = 0xa4e992a5474561dd

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

// tumblerChecksum pins the world state after 60 steps of Tumbler; a change
// here means the physics changed.
const tumblerChecksum uint64 = 0x19424e411cbc3122

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

// largePyramidChecksum pins the world state after 60 steps of Large Pyramid;
// a change here means the physics changed.
const largePyramidChecksum uint64 = 0x0e249d83448947f5

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
