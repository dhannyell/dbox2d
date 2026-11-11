//go:build tinygo.wasm.threads

package b2

import (
	"runtime"
	"unsafe"

	"github.com/dhannyell/dbox2d/internal/wasmthreads"
)

const (
	maxWorkers         = 64
	platformMaxWorkers = maxWorkers
)

func hardwareWorkers() int { return wasmthreads.Hardware() }

func prepareWorkers(count int) { wasmthreads.Start(count) }

// clampWorkers keeps a world within the started threads, so a page that did
// not call PrepareWorkers steps on one worker.
func clampWorkers(count int) int { return max(1, min(count, wasmthreads.Count())) }

func (e *executor) generationAddr() *uint32 {
	return (*uint32)(unsafe.Pointer(&e.generation))
}

func (e *executor) spawnWorker(workerIndex int, seen uint32) {
	wasmthreads.Go(func() { e.worker(workerIndex, seen) })
}

// parkWorker returns at once if the generation already moved past seen.
func (e *executor) parkWorker(workerIndex int, seen uint32) {
	wasmthreads.Wait(e.generationAddr(), seen)
}

func (e *executor) wakeWorkers() { wasmthreads.Notify(e.generationAddr()) }

// stop sets pending to the worker count, and each exit counts down.
func (e *executor) workerExited() { e.pending.Add(-1) }

func (e *executor) joinWorkers() {
	for e.pending.Load() != 0 {
	}
}

// idleYield lets the GC stop a worker that waits; a thread has no scheduler.
func idleYield() { runtime.Gosched() }

// launchTree runs the rebuild on a spare thread; the main thread spins in joinTree.
func (t *treeTask) launchTree(w *world) {
	t.flag.Store(0)
	wasmthreads.Go(func() { w.broadPhase.rebuildTrees(); t.flag.Store(1) })
}

func (t *treeTask) joinTree() {
	for t.flag.Load() == 0 {
	}
}
