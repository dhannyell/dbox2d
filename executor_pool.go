package dbox2d

import (
	"runtime"
	"sync"
)

// taskFunc is the shape of every parallel loop of the reference: a range
// [startIndex, endIndex) and the worker that runs it.
type taskFunc func(startIndex, endIndex, workerIndex int, context *stepContext)

// executorJob is the slot of one worker. It is reused, so a job never
// allocates.
type executorJob struct {
	fn          taskFunc
	runFn       func(workerIndex int)
	startIndex  int
	endIndex    int
	workerIndex int
	context     *stepContext
}

// executor runs the parallel loops of one world on a pool of goroutines.
// Worker 0 is the goroutine that called Step; workers 1..N-1 sleep on a
// channel between jobs. With one worker nothing here touches a goroutine.
type executor struct {
	workerCount int
	started     bool
	stopped     bool

	wake [maxWorkers]chan struct{}
	done [maxWorkers]chan struct{}
	jobs [maxWorkers]executorJob

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
	if n < 1 {
		n = 1
	}
	if n > platformMaxWorkers {
		n = platformMaxWorkers
	}
	e.workerCount = n
	e.started = true
	if n == 1 {
		return
	}

	for workerIndex := 1; workerIndex < n; workerIndex++ {
		e.wake[workerIndex] = make(chan struct{})
		e.done[workerIndex] = make(chan struct{}, 1)
		e.workers.Add(1)
		go e.worker(workerIndex)
	}
}

// stop releases the workers and waits for them to exit.
func (e *executor) stop() {
	if !e.started || e.workerCount == 1 || e.stopped {
		return
	}
	for workerIndex := 1; workerIndex < e.workerCount; workerIndex++ {
		close(e.wake[workerIndex])
	}
	e.workers.Wait()
	e.stopped = true
}

func (e *executor) worker(workerIndex int) {
	defer e.workers.Done()
	for range e.wake[workerIndex] {
		job := &e.jobs[workerIndex]
		if job.fn != nil {
			job.fn(job.startIndex, job.endIndex, job.workerIndex, job.context)
		} else {
			job.runFn(job.workerIndex)
		}
		e.done[workerIndex] <- struct{}{}
	}
}

// parallelFor splits [0, itemCount) into at most workerCount ranges of at
// least minRange items, unless itemCount is smaller. Worker 0 is the caller.
func (e *executor) parallelFor(itemCount, minRange int, fn taskFunc, context *stepContext) {
	if itemCount == 0 {
		return
	}
	if e.workerCount <= 1 || itemCount <= minRange {
		fn(0, itemCount, 0, context)
		return
	}
	if !e.started {
		e.start(e.workerCount)
	}

	rangeCount := min(e.workerCount, (itemCount+minRange-1)/minRange)
	rangeSize := (itemCount + rangeCount - 1) / rangeCount
	for rangeIndex := 1; rangeIndex < rangeCount; rangeIndex++ {
		startIndex := rangeIndex * rangeSize
		endIndex := min(startIndex+rangeSize, itemCount)
		job := &e.jobs[rangeIndex]
		job.fn = fn
		job.runFn = nil
		job.startIndex = startIndex
		job.endIndex = endIndex
		job.workerIndex = rangeIndex
		job.context = context
		e.wake[rangeIndex] <- struct{}{}
	}

	fn(0, min(rangeSize, itemCount), 0, context)
	for rangeIndex := 1; rangeIndex < rangeCount; rangeIndex++ {
		<-e.done[rangeIndex]
	}
}

// run calls fn once per worker, concurrently. Worker 0 is the caller.
func (e *executor) run(fn func(workerIndex int)) {
	if e.workerCount <= 1 {
		fn(0)
		return
	}
	if !e.started {
		e.start(e.workerCount)
	}
	for workerIndex := 1; workerIndex < e.workerCount; workerIndex++ {
		job := &e.jobs[workerIndex]
		job.fn = nil
		job.runFn = fn
		job.workerIndex = workerIndex
		e.wake[workerIndex] <- struct{}{}
	}

	fn(0)
	for workerIndex := 1; workerIndex < e.workerCount; workerIndex++ {
		<-e.done[workerIndex]
	}
}
