package dbox2d

import (
	"flag"
	"runtime"
	"sync/atomic"
	"testing"
	"time"
)

var workersFlag = flag.Int("workers", 1, "worker count for the step benchmarks and the checksum tests")

type executorTestState struct {
	counts     []int32
	inUse      []int32
	bad        int32
	rangeCount int32
}

func recordExecutorRange(startIndex, endIndex, workerIndex int, context *stepContext) {
	state := context.world.userData.(*executorTestState)
	if workerIndex < 0 || workerIndex >= len(state.inUse) || startIndex >= endIndex {
		atomic.StoreInt32(&state.bad, 1)
		return
	}
	if atomic.AddInt32(&state.inUse[workerIndex], 1) != 1 {
		atomic.StoreInt32(&state.bad, 1)
	}
	runtime.Gosched()
	for i := startIndex; i < endIndex; i++ {
		atomic.AddInt32(&state.counts[i], 1)
	}
	if atomic.AddInt32(&state.inUse[workerIndex], -1) != 0 {
		atomic.StoreInt32(&state.bad, 1)
	}
	atomic.AddInt32(&state.rangeCount, 1)
}

func noopExecutorTask(startIndex, endIndex, workerIndex int, context *stepContext) {}

func noopExecutorWorker(workerIndex int) {}

func TestParallelForCoversTheRangeOnce(t *testing.T) {
	for _, workerCount := range []int{1, 2, 3, 4, 8} {
		e := &executor{workerCount: workerCount}
		e.start(workerCount)
		workerCount = e.workerCount
		for _, itemCount := range []int{0, 1, 63, 64, 65, 1000} {
			state := &executorTestState{
				counts: make([]int32, itemCount),
				inUse:  make([]int32, workerCount),
			}
			context := &stepContext{world: &world{userData: state}}
			e.parallelFor(itemCount, 64, recordExecutorRange, context)

			if atomic.LoadInt32(&state.bad) != 0 {
				t.Fatalf("workers=%d items=%d: invalid range or worker use", workerCount, itemCount)
			}
			wantRanges := 0
			if itemCount != 0 {
				wantRanges = min(workerCount, (itemCount+63)/64)
			}
			if got := int(atomic.LoadInt32(&state.rangeCount)); got != wantRanges {
				t.Fatalf("workers=%d items=%d: got %d ranges, want %d", workerCount, itemCount, got, wantRanges)
			}
			for i := range state.counts {
				if got := atomic.LoadInt32(&state.counts[i]); got != 1 {
					t.Fatalf("workers=%d items=%d index=%d: got %d visits, want 1", workerCount, itemCount, i, got)
				}
			}
		}

		itemCount := 1000
		state := &executorTestState{
			counts: make([]int32, itemCount),
			inUse:  make([]int32, workerCount),
		}
		context := &stepContext{world: &world{userData: state}}
		e.parallelFor(itemCount, 1, recordExecutorRange, context)
		if atomic.LoadInt32(&state.bad) != 0 || atomic.LoadInt32(&state.rangeCount) != int32(workerCount) {
			t.Fatalf("workers=%d minRange=1: invalid ranges", workerCount)
		}
		for i := range state.counts {
			if got := atomic.LoadInt32(&state.counts[i]); got != 1 {
				t.Fatalf("workers=%d minRange=1 index=%d: got %d visits, want 1", workerCount, i, got)
			}
		}
		e.stop()
	}
}

func TestRunVisitsEveryWorker(t *testing.T) {
	for _, workerCount := range []int{1, 2, 3, 4, 8} {
		e := &executor{workerCount: workerCount}
		e.start(workerCount)
		// The platform may cap the pool; wasm always has one worker.
		workerCount = e.workerCount
		visits := make([]int32, workerCount)
		e.run(func(workerIndex int) {
			if workerIndex < 0 || workerIndex >= len(visits) {
				t.Errorf("got worker index %d", workerIndex)
				return
			}
			atomic.AddInt32(&visits[workerIndex], 1)
		})
		for workerIndex, count := range visits {
			if got := atomic.LoadInt32(&count); got != 1 {
				t.Fatalf("workers=%d index=%d: got %d visits, want 1", workerCount, workerIndex, got)
			}
		}
		e.stop()
	}
}

func TestExecutorStopsWithoutLeak(t *testing.T) {
	before := runtime.NumGoroutine()
	def := DefaultWorldDef()
	def.WorkerCount = 4
	worldID := CreateWorld(&def)
	worldID.Step(QOne().Div(QFromInt(60)), 1)
	DestroyWorld(worldID)

	for range 100 {
		if runtime.NumGoroutine() == before {
			return
		}
		runtime.Gosched()
		time.Sleep(time.Millisecond)
	}
	t.Fatalf("goroutine count did not return to %d: got %d", before, runtime.NumGoroutine())
}

func TestEffectiveWorkerCount(t *testing.T) {
	maxProcs := min(runtime.GOMAXPROCS(0), maxWorkers)
	if got := effectiveWorkerCount(0); got != min(maxProcs, platformMaxWorkers) {
		t.Fatalf("effectiveWorkerCount(0) = %d", got)
	}
	if got := effectiveWorkerCount(1); got != 1 {
		t.Fatalf("effectiveWorkerCount(1) = %d", got)
	}
	if got := effectiveWorkerCount(100); got != platformMaxWorkers {
		t.Fatalf("effectiveWorkerCount(100) = %d, want %d", got, platformMaxWorkers)
	}
}

func TestParallelForDoesNotAllocate(t *testing.T) {
	e := &executor{workerCount: 4}
	e.start(4)
	defer e.stop()

	if got := testing.AllocsPerRun(100, func() {
		e.parallelFor(1000, 64, noopExecutorTask, nil)
	}); got != 0 {
		t.Fatalf("parallelFor allocated %f times per run", got)
	}
	if got := testing.AllocsPerRun(100, func() {
		e.run(noopExecutorWorker)
	}); got != 0 {
		t.Fatalf("run allocated %f times per run", got)
	}
}
