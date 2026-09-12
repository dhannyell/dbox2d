// Command benchscale reads a set of scene-benchmark result files taken at
// several worker counts and prints how each side scales.
//
// benchcmp answers "how much slower is the port at this worker count". This
// answers the two questions that only a sweep can: whether the port keeps
// pace with the reference as workers are added, and whether the gap between
// them widens or closes with the thread count.
//
// Usage:
//
//	go run ./tools/benchscale testdata/bench/*.json
package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"math"
	"os"
	"sort"
	"text/tabwriter"
)

type scene struct {
	Name            string    `json:"name"`
	Bodies          int       `json:"bodies"`
	Steps           int       `json:"steps"`
	TotalMS         float64   `json:"total_ms"`
	StepSumMS       float64   `json:"step_sum_ms"`
	RepeatMS        []float64 `json:"repeat_ms"`
	ObservedWorkers int       `json:"observed_workers"`
}

type result struct {
	Schema    int     `json:"schema"`
	Arm       string  `json:"arm"`
	LanePath  string  `json:"lane_path"`
	LaneWidth int     `json:"lane_width"`
	Clock     string  `json:"clock"`
	Workers   int     `json:"workers"`
	Repeats   int     `json:"repeats"`
	Scenes    []scene `json:"scenes"`
}

// key identifies one measurement: one arm at one worker count.
type key struct {
	arm     string
	workers int
}

func main() {
	baseArm := flag.String("base", "", "arm treated as the baseline in the ratio column (default: the first arm read)")
	flag.Parse()
	if flag.NArg() < 2 {
		fmt.Fprintln(os.Stderr, "usage: benchscale [-base=arm] <result.json>...")
		os.Exit(2)
	}

	runs := make(map[key]result)
	var armOrder []string
	workerSet := make(map[int]bool)
	sceneOrder := []string{}
	seenScene := make(map[string]bool)

	var problems []string
	var lane, clock string
	laneWidth := 0

	for _, path := range flag.Args() {
		r := load(path)
		if lane == "" {
			lane, clock, laneWidth = r.LanePath, r.Clock, r.LaneWidth
		}
		// A sweep that mixes lane paths or clocks measures those instead of
		// the thread count, which is the whole point of holding them fixed.
		if r.LanePath != lane || r.Clock != clock || r.LaneWidth != laneWidth {
			problems = append(problems, fmt.Sprintf("%s: lane %s/%d clock %s does not match %s/%d %s",
				path, r.LanePath, r.LaneWidth, r.Clock, lane, laneWidth, clock))
			continue
		}
		k := key{r.Arm, r.Workers}
		if _, dup := runs[k]; dup {
			problems = append(problems, fmt.Sprintf("%s: %s at %d workers was read twice", path, r.Arm, r.Workers))
			continue
		}
		runs[k] = r
		workerSet[r.Workers] = true
		if !contains(armOrder, r.Arm) {
			armOrder = append(armOrder, r.Arm)
		}
		for _, s := range r.Scenes {
			if !seenScene[s.Name] {
				seenScene[s.Name] = true
				sceneOrder = append(sceneOrder, s.Name)
			}
		}
	}

	if len(armOrder) != 2 {
		fmt.Fprintf(os.Stderr, "benchscale: need exactly two arms, got %v\n", armOrder)
		os.Exit(2)
	}
	base, cand := armOrder[0], armOrder[1]
	if *baseArm != "" {
		if *baseArm == armOrder[1] {
			base, cand = armOrder[1], armOrder[0]
		} else if *baseArm != armOrder[0] {
			fmt.Fprintf(os.Stderr, "benchscale: -base=%s is not one of %v\n", *baseArm, armOrder)
			os.Exit(2)
		}
	}

	workers := keysSorted(workerSet)

	fmt.Printf("%s vs %s, lane %s width %d, clock %s\n", base, cand, lane, laneWidth, clock)
	fmt.Printf("solver ms is the sum of the per-step minima; speedup is each arm against its own one-worker run\n\n")

	out := tabwriter.NewWriter(os.Stdout, 0, 0, 2, ' ', 0)
	_, _ = fmt.Fprintf(out, "scene\tworkers\t%s ms\t%s ms\t%s/%s\t%s x1\t%s x1\tobserved\n",
		base, cand, cand, base, base, cand)

	// logRatio accumulates the cross-language ratio per worker count so the
	// summary can say whether the gap moves with the thread count.
	logRatio := make(map[int]float64)
	ratioCount := make(map[int]int)
	logSpeedBase := make(map[int]float64)
	logSpeedCand := make(map[int]float64)

	for _, name := range sceneOrder {
		baseOne := sceneAt(runs, key{base, 1}, name)
		candOne := sceneAt(runs, key{cand, 1}, name)

		for _, w := range workers {
			b := sceneAt(runs, key{base, w}, name)
			c := sceneAt(runs, key{cand, w}, name)
			if b == nil || c == nil {
				continue
			}
			if b.Steps != c.Steps || b.Bodies != c.Bodies {
				problems = append(problems, fmt.Sprintf("scene %s at %d workers: %d steps/%d bodies vs %d/%d",
					name, w, b.Steps, b.Bodies, c.Steps, c.Bodies))
				continue
			}

			ratio := c.StepSumMS / b.StepSumMS
			logRatio[w] += math.Log(ratio)
			ratioCount[w]++

			baseSpeed, candSpeed := 0.0, 0.0
			if baseOne != nil && b.StepSumMS > 0 {
				baseSpeed = baseOne.StepSumMS / b.StepSumMS
				logSpeedBase[w] += math.Log(baseSpeed)
			}
			if candOne != nil && c.StepSumMS > 0 {
				candSpeed = candOne.StepSumMS / c.StepSumMS
				logSpeedCand[w] += math.Log(candSpeed)
			}

			// The observed counts come straight from the harnesses: a scene
			// too small to split runs on the caller in both, and that is
			// worth reading next to a flat speedup rather than guessing at
			// it.
			label := name
			if w != workers[0] {
				label = ""
			}
			_, _ = fmt.Fprintf(out, "%s\t%d\t%.1f\t%.1f\t%.2fx\t%.2fx\t%.2fx\t%d/%d\n",
				label, w, b.StepSumMS, c.StepSumMS, ratio, baseSpeed, candSpeed,
				b.ObservedWorkers, c.ObservedWorkers)
		}
	}
	_ = out.Flush()

	fmt.Printf("\ngeometric mean over the scenes:\n\n")
	sum := tabwriter.NewWriter(os.Stdout, 0, 0, 2, ' ', 0)
	_, _ = fmt.Fprintf(sum, "workers\t%s/%s\t%s x1\t%s x1\t%s efficiency\t%s efficiency\n",
		cand, base, base, cand, base, cand)
	for _, w := range workers {
		n := float64(ratioCount[w])
		if n == 0 {
			continue
		}
		bs := math.Exp(logSpeedBase[w] / n)
		cs := math.Exp(logSpeedCand[w] / n)
		_, _ = fmt.Fprintf(sum, "%d\t%.2fx\t%.2fx\t%.2fx\t%.0f%%\t%.0f%%\n",
			w, math.Exp(logRatio[w]/n), bs, cs,
			100*bs/float64(w), 100*cs/float64(w))
	}
	_ = sum.Flush()

	if len(problems) > 0 {
		fmt.Fprintln(os.Stderr, "\nthese runs are not comparable:")
		for _, p := range problems {
			fmt.Fprintf(os.Stderr, "  %s\n", p)
		}
		os.Exit(1)
	}
}

func sceneAt(runs map[key]result, k key, name string) *scene {
	r, ok := runs[k]
	if !ok {
		return nil
	}
	for i := range r.Scenes {
		if r.Scenes[i].Name == name {
			return &r.Scenes[i]
		}
	}
	return nil
}

func contains(list []string, want string) bool {
	for _, v := range list {
		if v == want {
			return true
		}
	}
	return false
}

func keysSorted(set map[int]bool) []int {
	out := make([]int, 0, len(set))
	for k := range set {
		out = append(out, k)
	}
	sort.Ints(out)
	return out
}

func load(path string) result {
	blob, err := os.ReadFile(path)
	if err != nil {
		fmt.Fprintf(os.Stderr, "benchscale: %v\n", err)
		os.Exit(2)
	}
	var r result
	if err := json.Unmarshal(blob, &r); err != nil {
		fmt.Fprintf(os.Stderr, "benchscale: %s: %v\n", path, err)
		os.Exit(2)
	}
	return r
}
