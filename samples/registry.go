// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/sample.h and samples/main.cpp (CompareSamples) of Box2D v3.1.1

package samples

import "sort"

// CreateFcn builds a sample on the given context.
type CreateFcn func(ctx *SampleContext) Sample

// Entry is one registered sample.
type Entry struct {
	Category string
	Name     string
	Create   CreateFcn
}

var entries []Entry

// RegisterSample adds a sample. Call it from an init function. It returns
// the entry index.
//
// The reference caps registration at MAX_SAMPLES (256); a Go slice grows on
// demand, so no cap is enforced here.
func RegisterSample(category, name string, create CreateFcn) int {
	entries = append(entries, Entry{Category: category, Name: name, Create: create})
	return len(entries) - 1
}

// Entries returns the registered samples sorted the way the reference app
// lists them: by category, then by name, both byte-wise.
func Entries() []Entry {
	sorted := make([]Entry, len(entries))
	copy(sorted, entries)
	sort.Slice(sorted, func(i, j int) bool {
		if sorted[i].Category != sorted[j].Category {
			return sorted[i].Category < sorted[j].Category
		}
		return sorted[i].Name < sorted[j].Name
	})
	return sorted
}
