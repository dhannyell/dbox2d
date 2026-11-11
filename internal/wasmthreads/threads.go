//go:build tinygo.wasm.threads

// Package wasmthreads runs Go functions on browser Workers that share the wasm
// memory. A function on a thread must not enter the scheduler or call the JS host.
package wasmthreads

import (
	"runtime"
	"sync/atomic"
	"syscall/js"
	"time"
	"unsafe"
)

// maxThreads is the size of the per-thread allocation caches in the runtime.
const maxThreads = 16

const (
	stackSize = 1 << 20
	tlsBlock  = 64 // thread.js checks __tls_size against this value
	// idleSpins bounds the wait for a thread that just finished its function.
	idleSpins = 1 << 20
)

// slot is one thread. word is 1 while fn waits to run or runs.
type slot struct {
	word atomic.Uint32
	fn   func()
	_    [48]byte
}

var (
	slots  [maxThreads]slot
	stacks [][]byte
	count  int
	ready  atomic.Int32
	tls    [maxThreads][tlsBlock / 8]uint64
)

//go:wasmimport threads notify
func hostNotify(addr uintptr)

// Hardware is the number of threads that this page can use. It is 1 if the
// page has no thread host or is not cross-origin isolated.
func Hardware() int {
	global := js.Global()
	if global.Get("__threadsSpawn").IsUndefined() || !global.Get("crossOriginIsolated").Truthy() {
		return 1
	}
	concurrency := global.Get("navigator").Get("hardwareConcurrency")
	if concurrency.IsUndefined() {
		return 1
	}
	return max(1, min(concurrency.Int(), maxThreads))
}

// Count is the number of threads that Start created.
func Count() int { return count }

// Start creates n Workers and waits for them. A browser starts a Worker from
// its event loop, so do not call Start in a JS callback. Later calls do nothing.
func Start(n int) {
	if count != 0 || js.Global().Get("__threadsSpawn").IsUndefined() {
		return
	}
	n = min(n, maxThreads)
	for i := range n {
		stack := make([]byte, stackSize)
		stacks = append(stacks, stack)
		low := uintptr(unsafe.Pointer(&stack[0]))
		runtime.RegisterThreadStack(low, low+uintptr(stackSize))
		js.Global().Call("__threadsSpawn", i, low+uintptr(stackSize), uintptr(unsafe.Pointer(&tls[i])))
	}
	count = n
	for ready.Load() != int32(n) {
		time.Sleep(time.Millisecond)
	}
}

// Go gives fn to an idle thread. Only the main thread may call it.
func Go(fn func()) {
	for range idleSpins {
		for i := range count {
			s := &slots[i]
			if s.word.Load() == 0 {
				s.fn = fn
				s.word.Store(1)
				Notify((*uint32)(unsafe.Pointer(&s.word)))
				return
			}
		}
	}
	panic("wasmthreads: no idle thread")
}

// Wait sleeps until Notify if *addr is seen. The main thread must not call it.
// The GC can run while the thread sleeps.
func Wait(addr *uint32, seen uint32) { runtime.ThreadWait(addr, seen) }

// Notify wakes all threads that wait on addr.
func Notify(addr *uint32) { hostNotify(uintptr(unsafe.Pointer(addr))) }

//export threadsMain
func threadsMain(index int32) {
	runtime.ThreadEnter()
	s := &slots[index]
	ready.Add(1)
	for {
		for s.word.Load() == 0 {
			Wait((*uint32)(unsafe.Pointer(&s.word)), 0)
		}
		s.fn()
		s.fn = nil
		s.word.Store(0)
	}
}
