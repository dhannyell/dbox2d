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

// ExampleEntries prints the first registered sample.
func ExampleEntries() {
	e := samples.Entries()[0]
	fmt.Printf("%s / %s\n", e.Category, e.Name)
	// Output: Stacking / Single Box
}
