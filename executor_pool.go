package dbox2d

import (
	"runtime"
	"sync"
	"sync/atomic"
)

// taskFunc is the shape of every parallel loop of the reference: a range
// [startIndex, endIndex) and the worker that runs it.
type taskFunc func(startIndex, endIndex, workerIndex int, context *stepContext)

// idleSpinLimit is the number of paused atomic loads a worker spends waiting
// for the next command before it parks: tens of microseconds.
// The spin must not yield to the scheduler: with several workers, every
// yield contends on the runtime lock and costs more than the wait.
const idleSpinLimit = 1 << 12

// spinner counts the idle loads of one wait. It yields to the scheduler
// only after idleSpinLimit of them: Gosched takes the global scheduler
// lock, and with several workers the contention costs more than the wait.
type spinner int

// spin is one idle turn of a wait loop.
func (s *spinner) spin() {
	if *s < idleSpinLimit {
		cpuPause()
		*s++
		return
	}
	runtime.Gosched()
	*s = 0
}

// executorCommand is what the caller asks every worker to do. It is
// reused, so a command never allocates.
type executorCommand struct {
	fn           taskFunc
	runContextFn func(workerIndex int, context *stepContext)
	// sideFn belongs to the runContext path only; the last worker runs it
	// before its own share of runContextFn.
	sideFn     func(context *stepContext)
	context    *stepContext
	rangeCount int
	rangeSize  int
	itemCount  int
}

// executor runs the parallel loops of one world on a pool of goroutines.
// Worker 0 is the goroutine that called Step; workers 1..N-1 spin on the
// generation counter and park between steps. With one worker nothing here
// touches a goroutine or an atomic.
// serialBodyThreshold is the awake body count under which a step runs on
// one worker whatever WorkerCount says: measured on a pyramid, four workers
// lose at 15 bodies and win from 66. Tests lower it to reach the pool.
var serialBodyThreshold = 64

type executor struct {
	workerCount int
	started     bool
	stopped     bool
	// serial routes the current step to the caller; the bits do not change.
	serial bool

	command executorCommand

	// generation counts the commands; a worker acts when it changes.
	// pending counts the workers that have not finished the command.
	generation atomic.Uint32
	pending    atomic.Int32

	wake [maxWorkers]chan struct{}

	workers sync.WaitGroup
}

// effectiveWorkerCount resolves the WorkerCount of a WorldDef: 0 is the
// machine, and the platform caps the result.
func effectiveWorkerCount(requested int) int {
	if requested <= 0 {
		requested = runtime.GOMAXPROCS(0)
	}
	return min(requested, platformMaxWorkers)
}

// start spawns the workers 1..n-1. It is idempotent.
func (e *executor) start(n int) {
	if e.started {
		return
	}
	n = max(1, min(n, platformMaxWorkers))
	e.workerCount = n
	e.started = true
	if n == 1 {
		return
	}

	// The workers learn the generation here: a pool that starts inside the
	// first command must not miss it.
	seen := e.generation.Load()
	for workerIndex := 1; workerIndex < n; workerIndex++ {
		e.wake[workerIndex] = make(chan struct{}, 1)
		e.workers.Add(1)
		go e.worker(workerIndex, seen)
	}
}

// activeWorkerCount is the worker count of the current step.
func (e *executor) activeWorkerCount() int {
	if e.serial || e.workerCount <= 1 {
		return 1
	}
	return e.workerCount
}

// stop releases the workers and waits for them to exit.
func (e *executor) stop() {
	if !e.started || e.workerCount == 1 || e.stopped {
		return
	}
	e.stopped = true
	e.command = executorCommand{}
	e.publish()
	e.workers.Wait()
}

// publish hands the current command to the workers. The wake token is
// buffered, so a worker that is spinning keeps it for its next park.
func (e *executor) publish() {
	e.pending.Store(int32(e.workerCount - 1))
	e.generation.Add(1)
	for workerIndex := 1; workerIndex < e.workerCount; workerIndex++ {
		select {
		case e.wake[workerIndex] <- struct{}{}:
		default:
		}
	}
}

// await spins until every worker finished the command, yielding only after
// a bounded spin so a late worker can get a core.
func (e *executor) await() {
	var s spinner
	for e.pending.Load() != 0 {
		s.spin()
	}
}

func (e *executor) worker(workerIndex int, seen uint32) {
	defer e.workers.Done()
	for {
		spins := 0
		for e.generation.Load() == seen {
			if spins == idleSpinLimit {
				<-e.wake[workerIndex]
				spins = 0
				continue
			}
			spins++
		}
		seen = e.generation.Load()

		command := &e.command
		switch {
		case command.fn != nil:
			if workerIndex < command.rangeCount {
				startIndex := workerIndex * command.rangeSize
				endIndex := min(startIndex+command.rangeSize, command.itemCount)
				command.fn(startIndex, endIndex, workerIndex, command.context)
			}
		case command.runContextFn != nil:
			if command.sideFn != nil && workerIndex == e.workerCount-1 {
				command.sideFn(command.context)
			}
			command.runContextFn(workerIndex, command.context)
		default:
			return
		}
		e.pending.Add(-1)
	}
}

// parallelFor splits [0, itemCount) into at most workerCount ranges of at
// least minRange items, unless itemCount is smaller. Worker 0 is the caller.
func (e *executor) parallelFor(itemCount, minRange int, fn taskFunc, context *stepContext) {
	if e.activeWorkerCount() == 1 || itemCount <= minRange {
		if itemCount > 0 {
			fn(0, itemCount, 0, context)
		}
		return
	}
	if !e.started {
		e.start(e.workerCount)
	}

	rangeCount := min(e.workerCount, (itemCount+minRange-1)/minRange)
	rangeSize := (itemCount + rangeCount - 1) / rangeCount
	// The rounding of rangeSize can leave the last workers past the end.
	rangeCount = (itemCount + rangeSize - 1) / rangeSize
	e.command = executorCommand{fn: fn, context: context, rangeCount: rangeCount, rangeSize: rangeSize, itemCount: itemCount}
	e.publish()

	fn(0, min(rangeSize, itemCount), 0, context)
	e.await()
}

// runContext calls fn once per worker with the step context. Worker 0 is
// the caller. The context travels in the command, so fn captures nothing.
func (e *executor) runContext(fn func(workerIndex int, context *stepContext), context *stepContext) {
	e.runContextWithSide(fn, nil, context)
}

// runContextWithSide also runs sideFn once, on the last worker before fn.
func (e *executor) runContextWithSide(fn func(workerIndex int, context *stepContext), sideFn func(*stepContext), context *stepContext) {
	if e.activeWorkerCount() == 1 {
		if sideFn != nil {
			sideFn(context)
		}
		fn(0, context)
		return
	}
	if !e.started {
		e.start(e.workerCount)
	}
	e.command = executorCommand{runContextFn: fn, sideFn: sideFn, context: context}
	e.publish()

	fn(0, context)
	e.await()
}
