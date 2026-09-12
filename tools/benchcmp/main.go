// Command benchcmp compares two scene-benchmark result files and prints the
// ratio per scene. It refuses to compare runs that did not measure the same
// thing: a mismatched lane path, lane width, worker count, clock or scene
// set makes the ratio meaningless, and a silently mismatched lane path is
// the specific failure this whole comparison exists to avoid.
//
// Usage:
//
//	go run ./tools/benchcmp baseline.json candidate.json
package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"math"
	"os"
	"text/tabwriter"
)

type stepStats struct {
	MinUS    float64 `json:"min_us"`
	MedianUS float64 `json:"median_us"`
	P99US    float64 `json:"p99_us"`
	MaxUS    float64 `json:"max_us"`
}

type scene struct {
	Name            string    `json:"name"`
	Bodies          int       `json:"bodies"`
	Steps           int       `json:"steps"`
	TimedSteps      int       `json:"timed_steps"`
	TotalMS         float64   `json:"total_ms"`
	StepSumMS       float64   `json:"step_sum_ms"`
	HookMS          float64   `json:"hook_ms"`
	FPS             float64   `json:"fps"`
	Step            stepStats `json:"step_us"`
	RepeatMS        []float64 `json:"repeat_ms"`
	ObservedWorkers int       `json:"observed_workers"`
	FinalHash       string    `json:"final_hash"`
}

type result struct {
	Schema    int               `json:"schema"`
	Arm       string            `json:"arm"`
	LanePath  string            `json:"lane_path"`
	LaneWidth int               `json:"lane_width"`
	Clock     string            `json:"clock"`
	Workers   int               `json:"workers"`
	Repeats   int               `json:"repeats"`
	Env       map[string]string `json:"env"`
	Scenes    []scene           `json:"scenes"`
}

// spreadThreshold is the repeat spread above which a scene's number is too
// noisy to compare. The minimum of the repeats absorbs a slow run, but a
// wide spread means the machine was not quiet.
const spreadThreshold = 0.05

func main() {
	strict := flag.Bool("strict", true, "exit non-zero when the two runs are not comparable")
	flag.Parse()
	if flag.NArg() != 2 {
		fmt.Fprintln(os.Stderr, "usage: benchcmp [-strict=false] <baseline.json> <candidate.json>")
		os.Exit(2)
	}

	base := load(flag.Arg(0))
	cand := load(flag.Arg(1))

	var problems []string
	check := func(name string, a, b any) {
		if a != b {
			problems = append(problems, fmt.Sprintf("%s differs: %v vs %v", name, a, b))
		}
	}
	check("schema", base.Schema, cand.Schema)
	check("lane_path", base.LanePath, cand.LanePath)
	check("lane_width", base.LaneWidth, cand.LaneWidth)
	check("clock", base.Clock, cand.Clock)
	check("workers", base.Workers, cand.Workers)

	byName := make(map[string]scene, len(cand.Scenes))
	for _, s := range cand.Scenes {
		byName[s.Name] = s
	}

	fmt.Printf("%s (%s w%d) vs %s (%s w%d), %d workers, clock %s\n\n",
		base.Arm, base.LanePath, base.LaneWidth,
		cand.Arm, cand.LanePath, cand.LaneWidth,
		base.Workers, base.Clock)

	out := tabwriter.NewWriter(os.Stdout, 0, 0, 2, ' ', 0)
	_, _ = fmt.Fprintf(out, "scene\tbodies\t%s ms\t%s ms\ttotal\tsolver\thook %s\thook %s\tspread\n",
		base.Arm, cand.Arm, base.Arm, cand.Arm)

	// Ratios compose by multiplication, so they average geometrically. An
	// arithmetic mean of ratios is biased upward and lets one outlier
	// speak for the set.
	logRatioSum := 0.0
	logSolverSum := 0.0
	ratioCount := 0
	for _, b := range base.Scenes {
		c, ok := byName[b.Name]
		if !ok {
			problems = append(problems, fmt.Sprintf("scene %s is missing from %s", b.Name, cand.Arm))
			continue
		}
		if b.Steps != c.Steps {
			problems = append(problems, fmt.Sprintf("scene %s ran %d steps vs %d", b.Name, b.Steps, c.Steps))
			continue
		}
		if b.Bodies != c.Bodies {
			problems = append(problems, fmt.Sprintf("scene %s built %d bodies vs %d", b.Name, b.Bodies, c.Bodies))
			continue
		}
		// A world that accepted the worker count and then stepped on one
		// thread is the failure a multi-worker comparison must not hide.
		// Both sides route small stages to the caller, so the counts are
		// compared to each other rather than to the requested count.
		if b.ObservedWorkers != c.ObservedWorkers {
			problems = append(problems, fmt.Sprintf("scene %s ran on %d workers in %s but %d in %s",
				b.Name, b.ObservedWorkers, base.Arm, c.ObservedWorkers, cand.Arm))
		}

		ratio := c.TotalMS / b.TotalMS
		logRatioSum += math.Log(ratio)
		ratioCount++

		spread := maxSpread(b.RepeatMS, c.RepeatMS)
		note := ""
		if spread > spreadThreshold {
			note = " noisy"
		}
		// The block total also carries the scene hook, which builds and
		// destroys bodies between steps in rain. The solver ratio uses the
		// summed step minima and the hook is timed on its own, so a slow
		// hook cannot pass for a slow solver.
		solverRatio := 0.0
		if b.StepSumMS > 0 {
			solverRatio = c.StepSumMS / b.StepSumMS
			logSolverSum += math.Log(solverRatio)
		}
		_, _ = fmt.Fprintf(out, "%s\t%d\t%.1f\t%.1f\t%.2fx\t%.2fx\t%.1f\t%.1f\t%.1f%%%s\n",
			b.Name, b.Bodies, b.TotalMS, c.TotalMS, ratio, solverRatio,
			b.HookMS, c.HookMS, 100*spread, note)
	}
	_ = out.Flush()

	if ratioCount > 0 {
		n := float64(ratioCount)
		fmt.Printf("\ngeometric mean over %d scenes, %s relative to %s:\n", ratioCount, cand.Arm, base.Arm)
		fmt.Printf("  total  %.2fx\n", math.Exp(logRatioSum/n))
		fmt.Printf("  solver %.2fx  <- the comparable number; the total also carries the scene hook\n",
			math.Exp(logSolverSum/n))
	}

	// The two implementations do not agree bit for bit, by design, so a
	// differing hash is information rather than an error. An equal hash
	// between two builds of the same implementation is the real guard.
	fmt.Println("\nfinal hashes (equal only within one implementation):")
	for _, b := range base.Scenes {
		c, ok := byName[b.Name]
		if !ok {
			continue
		}
		mark := "differ"
		if b.FinalHash == c.FinalHash {
			mark = "equal"
		}
		fmt.Printf("  %-14s %s  %s  %s\n", b.Name, b.FinalHash, c.FinalHash, mark)
	}

	if len(problems) > 0 {
		fmt.Fprintln(os.Stderr, "\nthese runs are not comparable:")
		for _, p := range problems {
			fmt.Fprintf(os.Stderr, "  %s\n", p)
		}
		if *strict {
			os.Exit(1)
		}
	}
}

// maxSpread returns the larger relative spread of the two repeat sets. A
// repeat set with one entry has no spread to report.
func maxSpread(sets ...[]float64) float64 {
	worst := 0.0
	for _, set := range sets {
		if len(set) < 2 {
			continue
		}
		lo, hi := set[0], set[0]
		for _, v := range set {
			lo = min(lo, v)
			hi = max(hi, v)
		}
		if lo > 0 {
			worst = max(worst, (hi-lo)/lo)
		}
	}
	return worst
}

func load(path string) result {
	blob, err := os.ReadFile(path)
	if err != nil {
		fmt.Fprintf(os.Stderr, "benchcmp: %v\n", err)
		os.Exit(2)
	}
	var r result
	if err := json.Unmarshal(blob, &r); err != nil {
		fmt.Fprintf(os.Stderr, "benchcmp: %s: %v\n", path, err)
		os.Exit(2)
	}
	return r
}
