package dbox2d_test

// The scene benchmark mirrors the protocol of benchmark/main.c in the
// reference tree, so a Go number and a C number describe the same
// measurement: the first world step runs untimed because it is expensive
// and skews the result, the remaining steps are timed as one block, and a
// repeat contributes the minimum. The harness lives in a test file because
// the scene builders and the trace hash live there; build it standalone
// with `go test -c`. See tools/cbench for the C side.

import (
	"encoding/json"
	"flag"
	"fmt"
	"os"
	"runtime"
	"runtime/debug"
	"sort"
	"testing"

	"github.com/dhannyell/dbox2d/internal/scenes"

	. "github.com/dhannyell/dbox2d"
)

var (
	benchScenesFlag  = flag.String("bench.scenes", "", "comma-separated scenes to run; empty runs the whole benchmark set")
	benchWorkersFlag = flag.Int("bench.workers", 1, "worker count for the world")
	benchRepeatsFlag = flag.Int("bench.repeats", 5, "timed repeats per scene; the minimum wins")
	benchOutFlag     = flag.String("bench.out", "", "write the result JSON to this path")
	benchArmFlag     = flag.String("bench.arm", "", "label for this build, recorded in the output")
	benchLaneFlag    = flag.String("bench.want-lane", "", "abort unless the runtime lane path equals this value")
	benchHashFlag    = flag.Bool("bench.hash", true, "record the final scene hash, which guards arm-to-arm comparability")
)

// benchSceneOrder is the scene table of benchmark/main.c, in its order.
// falling_hinges is absent there, so it is absent here.
var benchSceneOrder = []string{
	"joint_grid",
	"large_pyramid",
	"many_pyramids",
	"rain",
	"smash",
	"spinner",
	"tumbler",
}

type benchStepStats struct {
	MinUS    float64 `json:"min_us"`
	MedianUS float64 `json:"median_us"`
	P99US    float64 `json:"p99_us"`
	MaxUS    float64 `json:"max_us"`
}

type benchSceneResult struct {
	Name       string         `json:"name"`
	Bodies     int            `json:"bodies"`
	Steps      int            `json:"steps"`
	TimedSteps int            `json:"timed_steps"`
	TotalMS    float64        `json:"total_ms"`
	StepSumMS  float64        `json:"step_sum_ms"`
	HookMS     float64        `json:"hook_ms"`
	FPS        float64        `json:"fps"`
	Step       benchStepStats `json:"step_us"`
	RepeatMS   []float64      `json:"repeat_ms"`
	// ObservedWorkers is the largest worker count any step of this scene
	// actually ran on. step.go routes a world with fewer awake bodies than
	// serialBodyThreshold to the caller, so a scene can ask for eight
	// workers and legitimately run on one; the output says which happened.
	BuildMS         float64 `json:"build_ms"`
	ObservedWorkers int     `json:"observed_workers"`
	FinalHash       string  `json:"final_hash,omitempty"`
}

type benchResult struct {
	Schema    int                `json:"schema"`
	Arm       string             `json:"arm"`
	LanePath  string             `json:"lane_path"`
	LaneWidth int                `json:"lane_width"`
	Clock     string             `json:"clock"`
	Workers   int                `json:"workers"`
	Repeats   int                `json:"repeats"`
	Env       map[string]string  `json:"env"`
	Scenes    []benchSceneResult `json:"scenes"`
}

// TestSceneBench runs the benchmark set. It is a test, not a testing.B, so
// that the sampling protocol is the one in benchmark/main.c rather than the
// one the testing package would impose.
func TestSceneBench(t *testing.T) {
	if *benchOutFlag == "" && *benchArmFlag == "" {
		t.Skip("scene benchmark: pass -bench.out or -bench.arm to run it")
	}
	lane := LanePath()
	if *benchLaneFlag != "" && lane != *benchLaneFlag {
		t.Fatalf("lane path is %q, want %q: the build tags, GOEXPERIMENT and toolchain do not select the path you asked for", lane, *benchLaneFlag)
	}
	scenes := benchSceneOrder
	if *benchScenesFlag != "" {
		scenes = splitBenchScenes(*benchScenesFlag)
	}

	result := benchResult{
		Schema:    1,
		Arm:       *benchArmFlag,
		LanePath:  lane,
		LaneWidth: LaneWidth(),
		Clock:     benchClockName(),
		Workers:   *benchWorkersFlag,
		Repeats:   *benchRepeatsFlag,
		Env:       benchEnv(),
	}
	for _, name := range scenes {
		result.Scenes = append(result.Scenes, runBenchScene(t, name))
	}

	blob, err := json.MarshalIndent(result, "", "  ")
	if err != nil {
		t.Fatalf("marshal result: %v", err)
	}
	if *benchOutFlag != "" {
		if err := os.WriteFile(*benchOutFlag, append(blob, '\n'), 0o644); err != nil {
			t.Fatalf("write %s: %v", *benchOutFlag, err)
		}
	}
	t.Logf("\n%s", blob)
}

func splitBenchScenes(list string) []string {
	var out []string
	start := 0
	for i := 0; i <= len(list); i++ {
		if i == len(list) || list[i] == ',' {
			if i > start {
				out = append(out, list[start:i])
			}
			start = i + 1
		}
	}
	return out
}

func benchEnv() map[string]string {
	env := map[string]string{
		"go_version": runtime.Version(),
		"goos":       runtime.GOOS,
		"goarch":     runtime.GOARCH,
		"gomaxprocs": fmt.Sprint(runtime.GOMAXPROCS(0)),
		"num_cpu":    fmt.Sprint(runtime.NumCPU()),
	}
	if info, ok := debug.ReadBuildInfo(); ok {
		for _, setting := range info.Settings {
			switch setting.Key {
			case "vcs.revision", "GOAMD64", "GOEXPERIMENT", "-tags":
				env[setting.Key] = setting.Value
			}
		}
	}
	return env
}

func runBenchScene(t *testing.T, name string) benchSceneResult {
	spec, ok := scenes.Specs[name]
	if !ok {
		t.Fatalf("scene %s is not in the benchmark scene table", name)
	}
	if spec.Steps == 0 {
		t.Fatalf("scene %s has no fixed step count", name)
	}

	out := benchSceneResult{Name: name, Steps: spec.Steps, TimedSteps: spec.Steps - 1}
	stepMin := make([]int64, spec.Steps-1)
	for i := range stepMin {
		stepMin[i] = 1 << 62
	}

	for repeat := range *benchRepeatsFlag {
		worldDef := DefaultWorldDef()
		worldDef.WorkerCount = *benchWorkersFlag
		worldId := CreateWorld(&worldDef)
		if worldId.IsNull() {
			t.Fatalf("scene %s: CreateWorld returned the null id", name)
		}
		// The scene build is timed too. It is body, shape and joint
		// creation rather than stepping, and the sample apps stall on it
		// long before the solver has run once.
		buildStart := benchTicks()
		stepFn := spec.Build(worldId, conformanceSceneOptions(worldId))
		if buildMS := benchMilliseconds(benchTicks() - buildStart); repeat == 0 || buildMS < out.BuildMS {
			out.BuildMS = buildMS
		}
		if repeat == 0 {
			out.Bodies = len(conformanceSceneBodies(worldId))
		}

		// The first step is untimed, per benchmark/main.c.
		if stepFn != nil {
			stepFn(0)
		}
		worldId.Step(QFromRatio(1, 60), 4)

		var hookTicks int64
		start := benchTicks()
		for stepIndex := 1; stepIndex < spec.Steps; stepIndex++ {
			if stepFn != nil {
				hookStart := benchTicks()
				stepFn(stepIndex)
				hookTicks += benchTicks() - hookStart
			}
			stepStart := benchTicks()
			worldId.Step(QFromRatio(1, 60), 4)
			if d := benchTicks() - stepStart; d < stepMin[stepIndex-1] {
				stepMin[stepIndex-1] = d
			}
			if active := ActiveWorkerCount(worldId); active > out.ObservedWorkers {
				out.ObservedWorkers = active
			}
		}
		elapsed := benchTicks() - start

		if *benchHashFlag && repeat == 0 {
			out.FinalHash = fmt.Sprintf("%016x", conformanceSceneHash(worldId))
		}

		ms := benchMilliseconds(elapsed)
		out.RepeatMS = append(out.RepeatMS, ms)
		if repeat == 0 || ms < out.TotalMS {
			out.TotalMS = ms
			out.HookMS = benchMilliseconds(hookTicks)
		}
		DestroyWorld(worldId)
	}

	out.FPS = float64(out.TimedSteps) / (out.TotalMS / 1000)
	out.Step = benchSummarize(stepMin)
	// The block total also carries the scene hook, which builds and
	// destroys bodies between steps in rain. Summing the per-step minima
	// separates solver time from that hook.
	var stepSum int64
	for _, d := range stepMin {
		stepSum += d
	}
	out.StepSumMS = benchMilliseconds(stepSum)
	return out
}

func benchSummarize(steps []int64) benchStepStats {
	us := make([]float64, len(steps))
	for i, d := range steps {
		us[i] = benchMilliseconds(d) * 1e3
	}
	sort.Float64s(us)
	if len(us) == 0 {
		return benchStepStats{}
	}
	pick := func(q float64) float64 {
		i := int(q * float64(len(us)-1))
		return us[i]
	}
	return benchStepStats{MinUS: us[0], MedianUS: pick(0.5), P99US: pick(0.99), MaxUS: us[len(us)-1]}
}
