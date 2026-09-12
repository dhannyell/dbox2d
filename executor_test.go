package dbox2d

import (
	"flag"
	"runtime"
	"strings"
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

type executorSideTestState struct {
	workerVisits []int32
	sideCount    int32
	bad          int32
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

func noopExecutorWorker(workerIndex int, context *stepContext) {}

func noopExecutorSide(context *stepContext) {}

func recordExecutorSide(context *stepContext) {
	state := context.world.userData.(*executorSideTestState)
	atomic.AddInt32(&state.sideCount, 1)
}

func recordExecutorSideWorker(workerIndex int, context *stepContext) {
	state := context.world.userData.(*executorSideTestState)
	if workerIndex == len(state.workerVisits)-1 && atomic.LoadInt32(&state.sideCount) != 1 {
		atomic.StoreInt32(&state.bad, 1)
	}
	atomic.AddInt32(&state.workerVisits[workerIndex], 1)
}

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
				rangeSize := (itemCount + min(workerCount, (itemCount+63)/64) - 1) / min(workerCount, (itemCount+63)/64)
				wantRanges = (itemCount + rangeSize - 1) / rangeSize
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

func TestRunContextVisitsEveryWorker(t *testing.T) {
	for _, workerCount := range []int{1, 2, 3, 4, 8} {
		e := &executor{workerCount: workerCount}
		e.start(workerCount)
		// The platform may cap the pool; wasm always has one worker.
		workerCount = e.workerCount
		visits := make([]int32, workerCount)
		e.runContext(func(workerIndex int, _ *stepContext) {
			if workerIndex < 0 || workerIndex >= len(visits) {
				t.Errorf("got worker index %d", workerIndex)
				return
			}
			atomic.AddInt32(&visits[workerIndex], 1)
		}, nil)
		for workerIndex, count := range visits {
			if got := atomic.LoadInt32(&count); got != 1 {
				t.Fatalf("workers=%d index=%d: got %d visits, want 1", workerCount, workerIndex, got)
			}
		}
		e.stop()
	}
}

func TestSideTaskRunsOnceBeforeTheWorkersFinish(t *testing.T) {
	for _, requestedWorkerCount := range []int{1, 2, 4} {
		e := &executor{workerCount: requestedWorkerCount}
		e.start(requestedWorkerCount)
		workerCount := e.workerCount

		state := &executorSideTestState{
			workerVisits: make([]int32, workerCount),
		}
		context := &stepContext{world: &world{userData: state}}
		e.runContextWithSide(recordExecutorSideWorker, recordExecutorSide, context)
		if got := atomic.LoadInt32(&state.sideCount); got != 1 {
			t.Fatalf("runContextWithSide workers=%d: side ran %d times, want 1", workerCount, got)
		}
		if atomic.LoadInt32(&state.bad) != 0 {
			t.Fatalf("runContextWithSide workers=%d: last worker started before the side task", workerCount)
		}
		for workerIndex := range workerCount {
			if got := atomic.LoadInt32(&state.workerVisits[workerIndex]); got != 1 {
				t.Fatalf("runContextWithSide workers=%d index=%d: got %d visits, want 1", workerCount, workerIndex, got)
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

func TestGetWorkerStartIndexMatchesTheReference(t *testing.T) {
	tests := []struct {
		workerIndex int
		blockCount  int
		workerCount int
		want        int
	}{
		{workerIndex: 0, blockCount: 1, workerCount: 4, want: 0},
		{workerIndex: 1, blockCount: 1, workerCount: 4, want: nullIndex},
		{workerIndex: 0, blockCount: 10, workerCount: 4, want: 0},
		{workerIndex: 1, blockCount: 10, workerCount: 4, want: 3},
		{workerIndex: 2, blockCount: 10, workerCount: 4, want: 6},
		{workerIndex: 3, blockCount: 10, workerCount: 4, want: 8},
		{workerIndex: 3, blockCount: 3, workerCount: 4, want: nullIndex},
	}

	for _, test := range tests {
		if got := getWorkerStartIndex(test.workerIndex, test.blockCount, test.workerCount); got != test.want {
			t.Errorf("getWorkerStartIndex(%d, %d, %d) = %d, want %d", test.workerIndex, test.blockCount, test.workerCount, got, test.want)
		}
	}
}

func requireSolverBlockLayout(t *testing.T, blocks []solverBlock, itemCount, blockSize, blockCount int, blockType solverBlockType) {
	t.Helper()
	if len(blocks) != blockCount {
		t.Fatalf("got %d blocks, want %d", len(blocks), blockCount)
	}
	for i := range blocks {
		block := &blocks[i]
		wantCount := blockSize
		if i == blockCount-1 {
			wantCount = itemCount - i*blockSize
		}
		if block.startIndex != i*blockSize || int(block.count) != wantCount || solverBlockType(block.blockType) != blockType {
			t.Fatalf("block %d = {start:%d count:%d type:%d}, want {start:%d count:%d type:%d}", i, block.startIndex, block.count, block.blockType, i*blockSize, wantCount, blockType)
		}
	}
}

func TestStageTableMatchesTheReferenceSizing(t *testing.T) {
	const blocksPerWorker = 4
	workerCounts := []int{1, 4, 8}

	for _, workerCount := range workerCounts {
		maxBlockCount := blocksPerWorker * workerCount
		for _, awakeBodyCount := range []int{1, 32, 33, 1000, 100000} {
			blockSize := 32
			blockCount := ((awakeBodyCount - 1) >> 5) + 1
			if awakeBodyCount > blockSize*maxBlockCount {
				blockSize = awakeBodyCount / maxBlockCount
				blockCount = maxBlockCount
			}

			w := world{}
			context := stepContext{workerCount: workerCount}
			buildSolverStages(&w, &context, awakeBodyCount)
			requireSolverBlockLayout(t, w.bodyBlocks, awakeBodyCount, blockSize, blockCount, bodyBlock)
		}

		for _, contactCount := range []int{1, 4, 5, 17, 10000} {
			// The blocks count constraint units: contacts in the scalar family, wide constraints in the wide one.
			unitCount := colorContactConstraintCount(contactCount)
			blockSize := blocksPerWorker
			blockCount := ((unitCount - 1) >> 2) + 1
			if unitCount > blockSize*maxBlockCount {
				blockSize = unitCount / maxBlockCount
				blockCount = maxBlockCount
			}

			graph := constraintGraph{}
			graph.colors[0].contactSims = make([]contactSim, contactCount)
			graph.colors[0].contacts = testContactPointers(graph.colors[0].contactSims)
			w := world{}
			context := stepContext{graph: &graph, activeColorCount: 1, workerCount: workerCount}
			context.activeColorIndices[0] = 0
			buildSolverStages(&w, &context, 1)
			requireSolverBlockLayout(t, w.graphBlocks, unitCount, blockSize, blockCount, graphContactBlock)
		}
	}

	graph := constraintGraph{}
	graph.colors[0].jointSims = make([]jointSim, 2)
	graph.colors[0].contactSims = make([]contactSim, 5)
	graph.colors[0].contacts = testContactPointers(graph.colors[0].contactSims)
	w := world{}
	context := stepContext{graph: &graph, activeColorCount: 1, workerCount: 4}
	context.activeColorIndices[0] = 0
	buildSolverStages(&w, &context, 1)
	requireSolverBlockLayout(t, w.graphBlocks[:1], 2, 4, 1, graphJointBlock)
	contactUnits := colorContactConstraintCount(5)
	requireSolverBlockLayout(t, w.graphBlocks[1:], contactUnits, 4, ((contactUnits-1)>>2)+1, graphContactBlock)
}

// 97 items over 12 workers round to ranges of 9; the twelfth would start
// past the end.
func TestParallelForNeverStartsPastTheEnd(t *testing.T) {
	e := &executor{workerCount: 12}
	e.start(12)
	if e.workerCount < 12 {
		t.Skip("this platform caps the workers")
	}
	const itemCount = 97
	state := &executorTestState{counts: make([]int32, itemCount), inUse: make([]int32, 12)}
	context := &stepContext{world: &world{userData: state}}
	e.parallelFor(itemCount, 8, recordExecutorRange, context)
	e.stop()
	if atomic.LoadInt32(&state.bad) != 0 {
		t.Fatal("a range was empty or inverted")
	}
	for i := range state.counts {
		if got := atomic.LoadInt32(&state.counts[i]); got != 1 {
			t.Fatalf("index %d: got %d visits, want 1", i, got)
		}
	}
}

// A pause longer than the spin limit parks the workers; the next step
// must wake them and give the bits of one worker.
func TestParkedWorkersWakeForTheNextStep(t *testing.T) {
	stepInParallel(t)
	run := func(workerCount int) uint64 {
		def := DefaultWorldDef()
		def.WorkerCount = workerCount
		def.EnableSleep = false
		worldId := CreateWorld(&def)
		defer DestroyWorld(worldId)
		buildPyramid(worldId, 20)
		dt := QOne().Div(QFromInt(60))
		for i := range 6 {
			if i%2 == 1 {
				time.Sleep(30 * time.Millisecond)
			}
			worldId.Step(dt, 4)
		}
		return Checksum(worldId)
	}
	want := run(1)
	if got := run(4); got != want {
		t.Fatalf("workers=4: got 0x%x, want 0x%x", got, want)
	}
}

// stepInParallel lowers the serial threshold so a small scene reaches the pool.
func stepInParallel(tb testing.TB) {
	tb.Helper()
	old := serialBodyThreshold
	serialBodyThreshold = 0
	tb.Cleanup(func() { serialBodyThreshold = old })
}

// A world under the threshold never publishes a command; one over it does.
func TestSmallWorldsStepOnOneWorker(t *testing.T) {
	steps := func(rows int) uint32 {
		def := DefaultWorldDef()
		def.WorkerCount = 4
		def.EnableSleep = false
		worldId := CreateWorld(&def)
		defer DestroyWorld(worldId)
		buildPyramid(worldId, rows)
		w := getWorldFromId(worldId)
		dt := QOne().Div(QFromInt(60))
		for range 3 {
			worldId.Step(dt, 4)
		}
		return w.executor.generation.Load()
	}
	if w := effectiveWorkerCount(4); w == 1 {
		t.Skip("this platform steps on one worker")
	}
	if got := steps(5); got != 0 {
		t.Fatalf("15 bodies published %d commands, want 0", got)
	}
	if got := steps(20); got == 0 {
		t.Fatalf("210 bodies published no command")
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
		e.runContext(noopExecutorWorker, nil)
	}); got != 0 {
		t.Fatalf("runContext allocated %f times per run", got)
	}
	if got := testing.AllocsPerRun(100, func() {
		e.runContextWithSide(noopExecutorWorker, noopExecutorSide, nil)
	}); got != 0 {
		t.Fatalf("runContextWithSide allocated %f times per run", got)
	}
}

func TestWorkerPanicReachesTheCaller(t *testing.T) {
	stepInParallel(t)
	def := DefaultWorldDef()
	def.WorkerCount = 4
	def.EnableSleep = false
	worldId := CreateWorld(&def)
	defer DestroyWorld(worldId)
	buildPyramid(worldId, 20)

	w := getWorldFromId(worldId)
	for i := range w.shapes {
		w.shapes[i].enablePreSolveEvents = true
	}
	dt := QOne().Div(QFromInt(60))
	worldId.Step(dt, 4)

	var panicked atomic.Int32
	worldId.SetPreSolveCallback(func(shapeIdA, shapeIdB ShapeId, manifold *Manifold) bool {
		panicked.Add(1)
		panic("boom in PreSolve")
	})

	caught := func() (value any) {
		defer func() { value = recover() }()
		worldId.Step(dt, 4)
		return nil
	}()
	if panicked.Load() == 0 {
		t.Fatal("the callback never ran")
	}
	if caught == nil {
		t.Fatal("Step did not panic")
	}
	wp, ok := caught.(*WorkerPanic)
	if !ok {
		t.Fatalf("caught %T %v, want *WorkerPanic", caught, caught)
	}
	if wp.Value != "boom in PreSolve" || len(wp.Stack) == 0 {
		t.Fatalf("WorkerPanic = %d %v stack=%d", wp.WorkerIndex, wp.Value, len(wp.Stack))
	}
	if !strings.Contains(wp.Error(), "boom in PreSolve") {
		t.Fatalf("Error() = %q", wp.Error())
	}
	if pending := w.executor.pending.Load(); pending != 0 {
		t.Fatalf("pending = %d after the panic", pending)
	}

	next := func() (value any) {
		defer func() { value = recover() }()
		worldId.Step(dt, 4)
		return nil
	}()
	if s, _ := next.(string); s != "dbox2d: the world is locked" {
		t.Fatalf("second Step: got %v, want the locked panic", next)
	}
}
