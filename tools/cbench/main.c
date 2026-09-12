// SPDX-License-Identifier: MIT
// Scene benchmark for the frozen Box2D v3.1.1 checkout. It is the C side of
// the port comparison; the Go side is TestSceneBench in scenebench_test.go.
// Both run the same scenes for the same step counts, leave the first step
// untimed, time the rest as one block, keep the minimum of the repeats, and
// emit the same JSON. See README.md.
#include "box2d/box2d.h"
#include "benchmarks.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "core.h"
#include "taskpool.h"

// src/timer.c reaches for QueryPerformanceCounter only under _MSC_VER; on
// MinGW it falls through to a stub that returns zero, so b2GetTicks cannot
// time this build. The harness carries its own counter instead, which also
// keeps the timing source identical to the Go side rather than dependent on
// which compiler built the reference.
#if defined( _WIN32 )

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN 1
#endif
#include <windows.h>

#define CLOCK_NAME "QueryPerformanceCounter"

static int64_t benchTicks( void )
{
	LARGE_INTEGER counter;
	QueryPerformanceCounter( &counter );
	return (int64_t)counter.QuadPart;
}

static double benchMilliseconds( int64_t ticks )
{
	static double invFrequency = 0.0;
	if ( invFrequency == 0.0 )
	{
		LARGE_INTEGER frequency;
		QueryPerformanceFrequency( &frequency );
		invFrequency = frequency.QuadPart > 0 ? 1000.0 / (double)frequency.QuadPart : 0.0;
	}
	return invFrequency * (double)ticks;
}

#else

#include <time.h>

#define CLOCK_NAME "clock_gettime(CLOCK_MONOTONIC)"

static int64_t benchTicks( void )
{
	struct timespec ts;
	clock_gettime( CLOCK_MONOTONIC, &ts );
	return (int64_t)ts.tv_sec * 1000000000LL + (int64_t)ts.tv_nsec;
}

static double benchMilliseconds( int64_t ticks )
{
	return (double)ticks / 1.0e6;
}

#endif

#define STRINGIFY_1( x ) #x
#define STRINGIFY( x ) STRINGIFY_1( x )

#if defined( __clang__ )
#define COMPILER_ID "clang-" STRINGIFY( __clang_major__ ) "." STRINGIFY( __clang_minor__ ) "." STRINGIFY( __clang_patchlevel__ )
#elif defined( _MSC_VER )
#define COMPILER_ID "msvc-" STRINGIFY( _MSC_FULL_VER )
#elif defined( __GNUC__ )
#define COMPILER_ID "gcc-" STRINGIFY( __GNUC__ ) "." STRINGIFY( __GNUC_MINOR__ ) "." STRINGIFY( __GNUC_PATCHLEVEL__ )
#else
#define COMPILER_ID "unknown"
#endif

#if defined( B2_SIMD_AVX2 )
#define LANE_PATH "avx2"
#elif defined( B2_SIMD_NEON )
#define LANE_PATH "neon"
#elif defined( B2_SIMD_SSE2 )
#define LANE_PATH "sse2"
#else
#define LANE_PATH "generic"
#endif

typedef void CreateFcn( b2WorldId worldId );
typedef float StepFcn( b2WorldId worldId, int stepCount );

typedef struct Benchmark
{
	const char* name;
	CreateFcn* create;
	StepFcn* step;
	int steps;
} Benchmark;

// The scene table of benchmark/main.c, in its order. It matches
// benchSceneOrder in scenebench_test.go.
static Benchmark benchmarks[] = {
	{ "joint_grid", CreateJointGrid, NULL, 500 },
	{ "large_pyramid", CreateLargePyramid, NULL, 500 },
	{ "many_pyramids", CreateManyPyramids, NULL, 200 },
	{ "rain", CreateRain, StepRain, 1000 },
	{ "smash", CreateSmash, NULL, 300 },
	{ "spinner", CreateSpinner, StepSpinner, 1400 },
	{ "tumbler", CreateTumbler, NULL, 750 },
};

#define BENCHMARK_COUNT ( (int)( sizeof( benchmarks ) / sizeof( benchmarks[0] ) ) )

static void fail( const char* message )
{
	fprintf( stderr, "cbench: %s\n", message );
	exit( 1 );
}

typedef struct BodyList
{
	b2BodyId* ids;
	int count;
	int capacity;
} BodyList;

static int compareBodyIds( const void* left, const void* right )
{
	const b2BodyId* bodyA = left;
	const b2BodyId* bodyB = right;
	return ( bodyA->index1 > bodyB->index1 ) - ( bodyA->index1 < bodyB->index1 );
}

static bool collectBody( b2ShapeId shapeId, void* context )
{
	BodyList* list = context;
	if ( list->count == list->capacity )
	{
		int capacity = list->capacity == 0 ? 256 : 2 * list->capacity;
		b2BodyId* ids = realloc( list->ids, (size_t)capacity * sizeof( b2BodyId ) );
		if ( ids == NULL )
		{
			fail( "cannot grow body list" );
		}
		list->ids = ids;
		list->capacity = capacity;
	}
	list->ids[list->count++] = b2Shape_GetBody( shapeId );
	return true;
}

// collectBodies reproduces the canonical order of the conformance traces:
// an overlap query over the whole world, deduplicated, sorted by index1.
static void collectBodies( b2WorldId worldId, BodyList* list )
{
	list->count = 0;
	b2AABB bounds = { { -1.0e6f, -1.0e6f }, { 1.0e6f, 1.0e6f } };
	b2World_OverlapAABB( worldId, bounds, b2DefaultQueryFilter(), collectBody, list );
	qsort( list->ids, (size_t)list->count, sizeof( b2BodyId ), compareBodyIds );

	int uniqueCount = 0;
	for ( int i = 0; i < list->count; ++i )
	{
		if ( uniqueCount == 0 || list->ids[i].index1 != list->ids[uniqueCount - 1].index1 )
		{
			list->ids[uniqueCount++] = list->ids[i];
		}
	}
	list->count = uniqueCount;
}

static uint64_t floatBits( float value )
{
	uint32_t bits;
	memcpy( &bits, &value, sizeof( bits ) );
	return bits;
}

static uint64_t foldFloat( uint64_t hash, float value )
{
	uint64_t word = floatBits( value );
	for ( int i = 0; i < 8; ++i )
	{
		hash ^= word & UINT64_C( 0xff );
		hash *= UINT64_C( 1099511628211 );
		word >>= 8;
	}
	return hash;
}

static uint64_t hashBodies( const BodyList* list )
{
	uint64_t hash = UINT64_C( 14695981039346656037 );
	for ( int i = 0; i < list->count; ++i )
	{
		b2Vec2 position = b2Body_GetPosition( list->ids[i] );
		b2Rot rotation = b2Body_GetRotation( list->ids[i] );
		hash = foldFloat( hash, position.x );
		hash = foldFloat( hash, position.y );
		hash = foldFloat( hash, rotation.c );
		hash = foldFloat( hash, rotation.s );
	}
	return hash;
}

static int compareDouble( const void* left, const void* right )
{
	double a = *(const double*)left;
	double b = *(const double*)right;
	return ( a > b ) - ( a < b );
}

static void usage( void )
{
	printf( "Usage\n"
			"--workers=<n>    worker count for the world (default 1)\n"
			"--repeats=<n>    timed repeats per scene, minimum wins (default 5)\n"
			"--scenes=<list>  comma-separated scene names (default all)\n"
			"--arm=<label>    label recorded in the output\n"
			"--want-lane=<s>  abort unless the compiled lane path matches\n"
			"--out=<path>     write the result JSON here\n" );
}

int main( int argc, char** argv )
{
	int workerCount = 1;
	int repeats = 5;
	const char* outputPath = NULL;
	const char* arm = "c-avx2";
	const char* wantLane = NULL;
	const char* sceneFilter = NULL;

	for ( int i = 1; i < argc; ++i )
	{
		const char* arg = argv[i];
		if ( strncmp( arg, "--workers=", 10 ) == 0 )
		{
			workerCount = atoi( arg + 10 );
		}
		else if ( strncmp( arg, "--repeats=", 10 ) == 0 )
		{
			repeats = atoi( arg + 10 );
		}
		else if ( strncmp( arg, "--out=", 6 ) == 0 )
		{
			outputPath = arg + 6;
		}
		else if ( strncmp( arg, "--arm=", 6 ) == 0 )
		{
			arm = arg + 6;
		}
		else if ( strncmp( arg, "--want-lane=", 12 ) == 0 )
		{
			wantLane = arg + 12;
		}
		else if ( strncmp( arg, "--scenes=", 9 ) == 0 )
		{
			sceneFilter = arg + 9;
		}
		else if ( strcmp( arg, "-h" ) == 0 || strcmp( arg, "--help" ) == 0 )
		{
			usage();
			return 0;
		}
		else
		{
			usage();
			return 1;
		}
	}

	if ( workerCount < 1 || repeats < 1 )
	{
		fail( "workers and repeats must be at least one" );
	}
	if ( wantLane != NULL && strcmp( wantLane, LANE_PATH ) != 0 )
	{
		fprintf( stderr, "cbench: lane path is %s, want %s: the CMake SIMD options do not select the path you asked for\n",
				 LANE_PATH, wantLane );
		return 1;
	}

	if ( benchPoolStart( workerCount ) == false )
	{
		fail( "cannot start the task pool" );
	}

	FILE* out = stdout;
	if ( outputPath != NULL )
	{
		out = fopen( outputPath, "w" );
		if ( out == NULL )
		{
			fail( "cannot open the output file" );
		}
	}

	b2Version version = b2GetVersion();

	fprintf( out, "{\n" );
	fprintf( out, "  \"schema\": 1,\n" );
	fprintf( out, "  \"arm\": \"%s\",\n", arm );
	fprintf( out, "  \"lane_path\": \"%s\",\n", LANE_PATH );
	fprintf( out, "  \"lane_width\": %d,\n", B2_SIMD_WIDTH );
	fprintf( out, "  \"clock\": \"%s\",\n", CLOCK_NAME );
	fprintf( out, "  \"workers\": %d,\n", workerCount );
	fprintf( out, "  \"repeats\": %d,\n", repeats );
	fprintf( out, "  \"env\": {\n" );
	fprintf( out, "    \"compiler\": \"%s\",\n", COMPILER_ID );
	fprintf( out, "    \"box2d_version\": \"%d.%d.%d\"\n", version.major, version.minor, version.revision );
	fprintf( out, "  },\n" );
	fprintf( out, "  \"scenes\": [\n" );

	bool firstScene = true;
	for ( int benchmarkIndex = 0; benchmarkIndex < BENCHMARK_COUNT; ++benchmarkIndex )
	{
		Benchmark* benchmark = benchmarks + benchmarkIndex;
		if ( sceneFilter != NULL && strstr( sceneFilter, benchmark->name ) == NULL )
		{
			continue;
		}

		int timedSteps = benchmark->steps - 1;
		double* stepMin = malloc( (size_t)timedSteps * sizeof( double ) );
		double* repeatMs = malloc( (size_t)repeats * sizeof( double ) );
		double* sorted = malloc( (size_t)timedSteps * sizeof( double ) );
		if ( stepMin == NULL || repeatMs == NULL || sorted == NULL )
		{
			fail( "cannot allocate the timing buffers" );
		}
		for ( int i = 0; i < timedSteps; ++i )
		{
			stepMin[i] = 1.0e30;
		}

		BodyList list = { 0 };
		int bodyCount = 0;
		double totalMs = 0.0;
		double hookMs = 0.0;
		double buildMs = 0.0;
		uint64_t finalHash = 0;

		benchPoolResetObserved();

		for ( int repeat = 0; repeat < repeats; ++repeat )
		{
			b2WorldDef worldDef = b2DefaultWorldDef();
			worldDef.workerCount = workerCount;
			worldDef.enqueueTask = benchEnqueueTask;
			worldDef.finishTask = benchFinishTask;
			b2WorldId worldId = b2CreateWorld( &worldDef );
			// The scene build is timed too. It is body, shape and joint
			// creation rather than stepping, and it is the one phase the
			// step columns cannot see.
			int64_t buildStart = benchTicks();
			benchmark->create( worldId );
			double buildOne = benchMilliseconds( benchTicks() - buildStart );
			if ( repeat == 0 || buildOne < buildMs )
			{
				buildMs = buildOne;
			}

			if ( repeat == 0 )
			{
				collectBodies( worldId, &list );
				bodyCount = list.count;
			}

			const float timeStep = 1.0f / 60.0f;
			const int subStepCount = 4;

			// The first step is expensive and skews the result, per
			// benchmark/main.c; it runs before the clock starts.
			if ( benchmark->step != NULL )
			{
				benchmark->step( worldId, 0 );
			}
			b2World_Step( worldId, timeStep, subStepCount );

			int64_t hookTicks = 0;
			int64_t start = benchTicks();
			for ( int stepIndex = 1; stepIndex < benchmark->steps; ++stepIndex )
			{
				if ( benchmark->step != NULL )
				{
					int64_t hookStart = benchTicks();
					benchmark->step( worldId, stepIndex );
					hookTicks += benchTicks() - hookStart;
				}
				int64_t stepStart = benchTicks();
				b2World_Step( worldId, timeStep, subStepCount );
				double ms = benchMilliseconds( benchTicks() - stepStart );
				if ( ms < stepMin[stepIndex - 1] )
				{
					stepMin[stepIndex - 1] = ms;
				}
			}
			double elapsed = benchMilliseconds( benchTicks() - start );

			if ( repeat == 0 )
			{
				collectBodies( worldId, &list );
				finalHash = hashBodies( &list );
			}

			repeatMs[repeat] = elapsed;
			if ( repeat == 0 || elapsed < totalMs )
			{
				totalMs = elapsed;
				hookMs = benchMilliseconds( hookTicks );
			}

			b2DestroyWorld( worldId );
		}

		// The block total also carries the scene hook, which builds and
		// destroys bodies between steps in rain. Summing the per-step
		// minima separates solver time from that hook.
		double stepSumMs = 0.0;
		for ( int i = 0; i < timedSteps; ++i )
		{
			stepSumMs += stepMin[i];
		}

		memcpy( sorted, stepMin, (size_t)timedSteps * sizeof( double ) );
		qsort( sorted, (size_t)timedSteps, sizeof( double ), compareDouble );

		fprintf( out, "%s    {\n", firstScene ? "" : ",\n" );
		firstScene = false;
		fprintf( out, "      \"name\": \"%s\",\n", benchmark->name );
		fprintf( out, "      \"bodies\": %d,\n", bodyCount );
		fprintf( out, "      \"steps\": %d,\n", benchmark->steps );
		fprintf( out, "      \"timed_steps\": %d,\n", timedSteps );
		fprintf( out, "      \"total_ms\": %.6f,\n", totalMs );
		fprintf( out, "      \"step_sum_ms\": %.6f,\n", stepSumMs );
		fprintf( out, "      \"hook_ms\": %.6f,\n", hookMs );
		fprintf( out, "      \"build_ms\": %.6f,\n", buildMs );
		fprintf( out, "      \"fps\": %.6f,\n", (double)timedSteps / ( totalMs / 1000.0 ) );
		fprintf( out, "      \"step_us\": {\n" );
		fprintf( out, "        \"min_us\": %.3f,\n", sorted[0] * 1000.0 );
		fprintf( out, "        \"median_us\": %.3f,\n", sorted[(int)( 0.5 * ( timedSteps - 1 ) )] * 1000.0 );
		fprintf( out, "        \"p99_us\": %.3f,\n", sorted[(int)( 0.99 * ( timedSteps - 1 ) )] * 1000.0 );
		fprintf( out, "        \"max_us\": %.3f\n", sorted[timedSteps - 1] * 1000.0 );
		fprintf( out, "      },\n" );
		fprintf( out, "      \"repeat_ms\": [" );
		for ( int repeat = 0; repeat < repeats; ++repeat )
		{
			fprintf( out, "%s%.6f", repeat == 0 ? "" : ", ", repeatMs[repeat] );
		}
		fprintf( out, "],\n" );
		// observed_workers is the guard against a world that accepted the
		// worker count and then stepped on one thread anyway. Box2D routes
		// small stages to the caller, so a value below the worker count is
		// information about the scene rather than an error.
		fprintf( out, "      \"observed_workers\": %d,\n", benchPoolObservedWorkers() );
		fprintf( out, "      \"final_hash\": \"%016" PRIx64 "\"\n", finalHash );
		fprintf( out, "    }" );
		fflush( out );

		fprintf( stderr, "cbench: %s %.3f ms (%.2f fps)\n", benchmark->name, totalMs,
				 (double)timedSteps / ( totalMs / 1000.0 ) );

		free( sorted );
		free( stepMin );
		free( repeatMs );
		free( list.ids );
	}

	fprintf( out, "\n  ]\n}\n" );
	if ( out != stdout )
	{
		fclose( out );
	}

	benchPoolStop();

	if ( benchPoolOverflowed() )
	{
		fprintf( stderr, "cbench: the task pool overflowed and ran ranges inline; the numbers understate parallelism\n" );
	}
	return 0;
}
