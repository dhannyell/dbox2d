//go:build !tinygo.wasm.threads

package b2

import "runtime"

func hardwareWorkers() int { return runtime.GOMAXPROCS(0) }

func prepareWorkers(int) {}

func clampWorkers(count int) int { return count }

func (e *executor) spawnWorker(workerIndex int, seen uint32) {
	e.wake[workerIndex] = make(chan struct{}, 1)
	e.workers.Add(1)
	go e.worker(workerIndex, seen)
}

func (e *executor) parkWorker(workerIndex int, _ uint32) {
	<-e.wake[workerIndex]
}

func (e *executor) wakeWorkers() {
	for workerIndex := 1; workerIndex < e.workerCount; workerIndex++ {
		select {
		case e.wake[workerIndex] <- struct{}{}:
		default:
		}
	}
}

func (e *executor) workerExited() { e.workers.Done() }

func (e *executor) joinWorkers() { e.workers.Wait() }

func idleYield() { runtime.Gosched() }

func (t *treeTask) launchTree(w *world) {
	if t.start == nil {
		t.start = make(chan *world, 1)
		t.done = make(chan struct{}, 1)
		go rebuildTreesLoop(t.start, t.done)
	}
	t.start <- w
}

func (t *treeTask) joinTree() { <-t.done }
