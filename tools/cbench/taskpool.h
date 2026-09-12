// SPDX-License-Identifier: MIT
// A task scheduler for the scene benchmark.
//
// b2CreateWorld keeps workerCount only when enqueueTask and finishTask are
// both set (src/world.c:224); without them it silently resets the world to
// one worker. A harness that sets workerCount alone would therefore report
// single-threaded numbers under any --workers value, which is the one
// failure a multi-worker comparison must not have.
//
// The reference benchmark supplies these callbacks through enkiTS, which
// this tree does not vendor. This pool covers the contract box2d actually
// uses: split a range into chunks of at least minRange, run them on
// workerCount threads with distinct worker indices in [0, workerCount), and
// let the enqueueing thread join the work while it waits.
#pragma once

#include "box2d/types.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define BENCH_POOL_MAX_TASKS 128
#define BENCH_POOL_MAX_WORKERS 64

#if defined( _WIN32 )

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN 1
#endif
#include <windows.h>

typedef CRITICAL_SECTION benchMutex;
typedef CONDITION_VARIABLE benchCond;
typedef HANDLE benchThread;

static void benchMutexInit( benchMutex* m ) { InitializeCriticalSection( m ); }
static void benchMutexDestroy( benchMutex* m ) { DeleteCriticalSection( m ); }
static void benchLock( benchMutex* m ) { EnterCriticalSection( m ); }
static void benchUnlock( benchMutex* m ) { LeaveCriticalSection( m ); }
static void benchCondInit( benchCond* c ) { InitializeConditionVariable( c ); }
static void benchCondDestroy( benchCond* c ) { (void)c; }
static void benchCondWait( benchCond* c, benchMutex* m ) { SleepConditionVariableCS( c, m, INFINITE ); }
static void benchCondBroadcast( benchCond* c ) { WakeAllConditionVariable( c ); }

#else

#include <pthread.h>

typedef pthread_mutex_t benchMutex;
typedef pthread_cond_t benchCond;
typedef pthread_t benchThread;

static void benchMutexInit( benchMutex* m ) { pthread_mutex_init( m, NULL ); }
static void benchMutexDestroy( benchMutex* m ) { pthread_mutex_destroy( m ); }
static void benchLock( benchMutex* m ) { pthread_mutex_lock( m ); }
static void benchUnlock( benchMutex* m ) { pthread_mutex_unlock( m ); }
static void benchCondInit( benchCond* c ) { pthread_cond_init( c, NULL ); }
static void benchCondDestroy( benchCond* c ) { pthread_cond_destroy( c ); }
static void benchCondWait( benchCond* c, benchMutex* m ) { pthread_cond_wait( c, m ); }
static void benchCondBroadcast( benchCond* c ) { pthread_cond_broadcast( c ); }

#endif

typedef struct benchPoolTask
{
	b2TaskCallback* fcn;
	void* context;
	int itemCount;
	int chunkCount;
	int nextChunk;
	int doneChunks;
	bool linked;
	struct benchPoolTask* queueNext;
	struct benchPoolTask* freeNext;
} benchPoolTask;

typedef struct benchPool
{
	benchMutex mutex;
	benchCond work;
	benchCond done;

	benchPoolTask tasks[BENCH_POOL_MAX_TASKS];
	benchPoolTask* freeList;
	benchPoolTask* queueHead;

	benchThread threads[BENCH_POOL_MAX_WORKERS];
	int workerIndices[BENCH_POOL_MAX_WORKERS];
	int threadCount;
	int workerCount;
	bool stop;
	bool overflowed;
	// workerSeen records which worker indices actually ran a chunk. A world
	// that fell back to one worker is otherwise indistinguishable from a
	// parallel one in the output, and that is the mistake being guarded.
	bool workerSeen[BENCH_POOL_MAX_WORKERS];
} benchPool;

static benchPool benchPoolInstance;

// benchChunkRange splits [0, itemCount) evenly over chunkCount, which keeps
// every worker's slice within one item of every other.
static void benchChunkRange( const benchPoolTask* task, int chunk, int* start, int* end )
{
	int64_t count = task->itemCount;
	int64_t n = task->chunkCount;
	*start = (int)( (int64_t)chunk * count / n );
	*end = (int)( (int64_t)( chunk + 1 ) * count / n );
}

static void benchUnlinkLocked( benchPool* pool, benchPoolTask* task )
{
	if ( task->linked == false )
	{
		return;
	}
	benchPoolTask** link = &pool->queueHead;
	while ( *link != NULL )
	{
		if ( *link == task )
		{
			*link = task->queueNext;
			task->queueNext = NULL;
			task->linked = false;
			return;
		}
		link = &( *link )->queueNext;
	}
	task->linked = false;
}

// benchClaimLocked hands out the next unclaimed chunk from the queue and
// drops tasks that have none left. Returning false with the lock held means
// there is no work at all, which is what the workers wait on.
static bool benchClaimLocked( benchPool* pool, benchPoolTask** outTask, int* outChunk )
{
	benchPoolTask* task = pool->queueHead;
	while ( task != NULL )
	{
		benchPoolTask* next = task->queueNext;
		if ( task->nextChunk < task->chunkCount )
		{
			*outTask = task;
			*outChunk = task->nextChunk;
			task->nextChunk += 1;
			if ( task->nextChunk == task->chunkCount )
			{
				benchUnlinkLocked( pool, task );
			}
			return true;
		}
		benchUnlinkLocked( pool, task );
		task = next;
	}
	return false;
}

static void benchRunChunk( benchPool* pool, benchPoolTask* task, int chunk, int workerIndex )
{
	int start = 0;
	int end = 0;
	benchChunkRange( task, chunk, &start, &end );
	task->fcn( start, end, (uint32_t)workerIndex, task->context );

	benchLock( &pool->mutex );
	pool->workerSeen[workerIndex] = true;
	task->doneChunks += 1;
	if ( task->doneChunks == task->chunkCount )
	{
		benchCondBroadcast( &pool->done );
	}
	benchUnlock( &pool->mutex );
}

static void benchWorkerLoop( benchPool* pool, int workerIndex )
{
	for ( ;; )
	{
		benchPoolTask* task = NULL;
		int chunk = 0;

		benchLock( &pool->mutex );
		while ( benchClaimLocked( pool, &task, &chunk ) == false )
		{
			if ( pool->stop )
			{
				benchUnlock( &pool->mutex );
				return;
			}
			benchCondWait( &pool->work, &pool->mutex );
		}
		benchUnlock( &pool->mutex );

		benchRunChunk( pool, task, chunk, workerIndex );
	}
}

#if defined( _WIN32 )
static DWORD WINAPI benchWorkerEntry( LPVOID arg )
{
	benchWorkerLoop( &benchPoolInstance, *(int*)arg );
	return 0;
}
#else
static void* benchWorkerEntry( void* arg )
{
	benchWorkerLoop( &benchPoolInstance, *(int*)arg );
	return NULL;
}
#endif

// benchPoolStart brings up workerCount - 1 threads. The enqueueing thread is
// worker 0 and joins the work in benchPoolFinish, so a worker count of one
// creates no threads and runs exactly as the single-threaded harness did.
static bool benchPoolStart( int workerCount )
{
	benchPool* pool = &benchPoolInstance;
	if ( workerCount < 1 || workerCount > BENCH_POOL_MAX_WORKERS )
	{
		return false;
	}

	benchMutexInit( &pool->mutex );
	benchCondInit( &pool->work );
	benchCondInit( &pool->done );

	pool->freeList = NULL;
	pool->queueHead = NULL;
	pool->stop = false;
	pool->overflowed = false;
	for ( int i = 0; i < BENCH_POOL_MAX_WORKERS; ++i )
	{
		pool->workerSeen[i] = false;
	}
	pool->workerCount = workerCount;
	for ( int i = BENCH_POOL_MAX_TASKS - 1; i >= 0; --i )
	{
		pool->tasks[i].freeNext = pool->freeList;
		pool->freeList = &pool->tasks[i];
	}

	pool->threadCount = workerCount - 1;
	for ( int i = 0; i < pool->threadCount; ++i )
	{
		pool->workerIndices[i] = i + 1;
#if defined( _WIN32 )
		pool->threads[i] = CreateThread( NULL, 0, benchWorkerEntry, &pool->workerIndices[i], 0, NULL );
		if ( pool->threads[i] == NULL )
		{
			return false;
		}
#else
		if ( pthread_create( &pool->threads[i], NULL, benchWorkerEntry, &pool->workerIndices[i] ) != 0 )
		{
			return false;
		}
#endif
	}
	return true;
}

static void benchPoolStop( void )
{
	benchPool* pool = &benchPoolInstance;

	benchLock( &pool->mutex );
	pool->stop = true;
	benchCondBroadcast( &pool->work );
	benchUnlock( &pool->mutex );

	for ( int i = 0; i < pool->threadCount; ++i )
	{
#if defined( _WIN32 )
		WaitForSingleObject( pool->threads[i], INFINITE );
		CloseHandle( pool->threads[i] );
#else
		pthread_join( pool->threads[i], NULL );
#endif
	}
	pool->threadCount = 0;

	benchCondDestroy( &pool->done );
	benchCondDestroy( &pool->work );
	benchMutexDestroy( &pool->mutex );
}

static void* benchEnqueueTask( b2TaskCallback* fcn, int itemCount, int minRange, void* context, void* userContext )
{
	(void)userContext;
	benchPool* pool = &benchPoolInstance;

	if ( itemCount <= 0 )
	{
		return NULL;
	}

	int chunkCount = minRange > 0 ? itemCount / minRange : itemCount;
	if ( chunkCount < 1 )
	{
		chunkCount = 1;
	}
	if ( chunkCount > pool->workerCount )
	{
		chunkCount = pool->workerCount;
	}

	benchLock( &pool->mutex );
	benchPoolTask* task = pool->freeList;
	if ( task == NULL )
	{
		// Running the range inline keeps the result correct; the flag makes
		// the harness say so rather than quietly report a serialised run.
		pool->overflowed = true;
		benchUnlock( &pool->mutex );
		fcn( 0, itemCount, 0, context );
		return NULL;
	}
	pool->freeList = task->freeNext;

	task->fcn = fcn;
	task->context = context;
	task->itemCount = itemCount;
	task->chunkCount = chunkCount;
	task->nextChunk = 0;
	task->doneChunks = 0;
	task->freeNext = NULL;
	task->queueNext = pool->queueHead;
	task->linked = true;
	pool->queueHead = task;

	benchCondBroadcast( &pool->work );
	benchUnlock( &pool->mutex );
	return task;
}

// benchFinishTask blocks until the task is done, taking chunks itself as
// worker 0 while it waits. That mirrors enkiWaitForTaskSet: with one worker
// the calling thread does all of the work and no thread ever sleeps.
static void benchFinishTask( void* userTask, void* userContext )
{
	(void)userContext;
	if ( userTask == NULL )
	{
		return;
	}

	benchPool* pool = &benchPoolInstance;
	benchPoolTask* task = userTask;

	for ( ;; )
	{
		benchLock( &pool->mutex );
		if ( task->doneChunks == task->chunkCount )
		{
			benchUnlinkLocked( pool, task );
			task->freeNext = pool->freeList;
			pool->freeList = task;
			benchUnlock( &pool->mutex );
			return;
		}
		// Taking any pending chunk rather than only this task's is what
		// keeps the solver from deadlocking. solver.c:1691 enqueues one
		// task per worker and only then waits on the first of them, and
		// those tasks synchronise with each other on an internal barrier.
		// A waiter that would only run its own task can leave one of the
		// peers unclaimed with no runner left to take it, and the ones
		// already inside the barrier then wait for it forever.
		benchPoolTask* claimed = NULL;
		int chunk = 0;
		if ( benchClaimLocked( pool, &claimed, &chunk ) )
		{
			benchUnlock( &pool->mutex );
			benchRunChunk( pool, claimed, chunk, 0 );
			continue;
		}
		benchCondWait( &pool->done, &pool->mutex );
		benchUnlock( &pool->mutex );
	}
}

static bool benchPoolOverflowed( void )
{
	return benchPoolInstance.overflowed;
}

// benchPoolResetObserved clears the record so the next scene reports its own
// worker use rather than the whole run's.
static void benchPoolResetObserved( void )
{
	benchLock( &benchPoolInstance.mutex );
	for ( int i = 0; i < BENCH_POOL_MAX_WORKERS; ++i )
	{
		benchPoolInstance.workerSeen[i] = false;
	}
	benchUnlock( &benchPoolInstance.mutex );
}

// benchPoolObservedWorkers counts the worker indices that ran at least one
// chunk since the last reset.
static int benchPoolObservedWorkers( void )
{
	int count = 0;
	for ( int i = 0; i < BENCH_POOL_MAX_WORKERS; ++i )
	{
		if ( benchPoolInstance.workerSeen[i] )
		{
			count += 1;
		}
	}
	return count;
}
