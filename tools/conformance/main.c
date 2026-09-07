// SPDX-License-Identifier: MIT
// Link against the frozen Box2D v3.1.1 checkout. See README.md.
#include "box2d/box2d.h"
#include "benchmarks.h"
#include "determinism.h"
#include "random.h"

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined( _WIN32 )
#include <direct.h>
#define makeDirectory( path ) _mkdir( path )
#else
#include <sys/stat.h>
#define makeDirectory( path ) mkdir( path, 0777 )
#endif

#define STRINGIFY_1( x ) #x
#define STRINGIFY( x ) STRINGIFY_1( x )

#if defined( __clang__ )
#define COMPILER_ID "clang-" STRINGIFY( __clang_major__ ) "." STRINGIFY( __clang_minor__ ) "." STRINGIFY( __clang_patchlevel__ )
#define TRACE_FLAGS "-O3,-DNDEBUG,-std=gnu17,-ffp-contract=off,-fno-fast-math"
#elif defined( _MSC_VER )
#define COMPILER_ID "msvc-" STRINGIFY( _MSC_FULL_VER )
#define TRACE_FLAGS "/O2,/Ob2,/DNDEBUG,/std:c17,/fp:precise,/fp:contract-"
#elif defined( __GNUC__ )
#define COMPILER_ID "gcc-" STRINGIFY( __GNUC__ ) "." STRINGIFY( __GNUC_MINOR__ ) "." STRINGIFY( __GNUC_PATCHLEVEL__ )
#define TRACE_FLAGS "-O3,-DNDEBUG,-std=gnu17,-ffp-contract=off,-fno-fast-math"
#else
#define COMPILER_ID "unknown"
#define TRACE_FLAGS "unknown"
#endif

enum
{
	caseCountCollide = 256,
	caseCountDistance = 512,
	caseCountTOI = 256,
	caseCountHull = 128,
	caseCountMath = 1024,
};

typedef enum CollisionKind
{
	collideCircles,
	collideCapsuleAndCircle,
	collidePolygonAndCircle,
	collideCapsules,
	collideSegmentAndCapsule,
	collidePolygonAndCapsule,
	collidePolygons,
	collideSegmentAndCircle,
	collideSegmentAndPolygon,
	collideChainSegmentAndCircle,
	collideChainSegmentAndCapsule,
	collideChainSegmentAndPolygon,
} CollisionKind;

typedef struct BodyList
{
	b2BodyId* ids;
	int count;
	int capacity;
} BodyList;

typedef void CreateSceneFcn( b2WorldId worldId );
typedef float StepSceneFcn( b2WorldId worldId, int stepIndex );

typedef struct Scene
{
	const char* name;
	CreateSceneFcn* create;
	StepSceneFcn* beforeStep;
	int steps;
	bool writeFinalBodies;
} Scene;

static void fail( const char* message )
{
	fprintf( stderr, "conformance: %s\n", message );
	exit( 1 );
}

static void ensureDirectory( const char* path )
{
	if ( makeDirectory( path ) != 0 && errno != EEXIST )
	{
		fail( "cannot create output directory" );
	}
}

static FILE* openTrace( const char* outputDirectory, const char* kind, const char* name )
{
	char path[1024];
	int length = snprintf( path, sizeof( path ), "%s/%s/%s.txt", outputDirectory, kind, name );
	if ( length < 0 || length >= (int)sizeof( path ) )
	{
		fail( "output path is too long" );
	}

	FILE* file = fopen( path, "wb" );
	if ( file == NULL )
	{
		fail( "cannot open trace file" );
	}

	return file;
}

static void writeHeader( FILE* file, const char* kind, const char* name )
{
	fprintf( file, "# dbox2d conformance trace v1 kind=%s name=%s ref=0aa402e cc=%s flags=%s\n", kind, name, COMPILER_ID,
			 TRACE_FLAGS );
}

static uint32_t floatBits( float value )
{
	uint32_t bits;
	memcpy( &bits, &value, sizeof( bits ) );
	return bits;
}

static void writeFloat( FILE* file, float value )
{
	fprintf( file, " %08" PRIx32, floatBits( value ) );
}

static void writeInt( FILE* file, int value )
{
	fprintf( file, " %d", value );
}

static void writeVec2( FILE* file, b2Vec2 value )
{
	writeFloat( file, value.x );
	writeFloat( file, value.y );
}

static void writeRot( FILE* file, b2Rot value )
{
	writeFloat( file, value.c );
	writeFloat( file, value.s );
}

static void writeTransform( FILE* file, b2Transform value )
{
	writeVec2( file, value.p );
	writeRot( file, value.q );
}

static void writeCircle( FILE* file, b2Circle value )
{
	writeVec2( file, value.center );
	writeFloat( file, value.radius );
}

static void writeCapsule( FILE* file, b2Capsule value )
{
	writeVec2( file, value.center1 );
	writeVec2( file, value.center2 );
	writeFloat( file, value.radius );
}

static void writeSegment( FILE* file, b2Segment value )
{
	writeVec2( file, value.point1 );
	writeVec2( file, value.point2 );
}

static void writeChainSegment( FILE* file, b2ChainSegment value )
{
	writeVec2( file, value.ghost1 );
	writeSegment( file, value.segment );
	writeVec2( file, value.ghost2 );
	writeInt( file, value.chainId );
}

static void writePolygon( FILE* file, b2Polygon value )
{
	writeInt( file, value.count );
	for ( int i = 0; i < B2_MAX_POLYGON_VERTICES; ++i )
	{
		writeVec2( file, value.vertices[i] );
	}
	for ( int i = 0; i < B2_MAX_POLYGON_VERTICES; ++i )
	{
		writeVec2( file, value.normals[i] );
	}
	writeVec2( file, value.centroid );
	writeFloat( file, value.radius );
}

static void writeProxy( FILE* file, b2ShapeProxy value )
{
	writeInt( file, value.count );
	for ( int i = 0; i < B2_MAX_POLYGON_VERTICES; ++i )
	{
		writeVec2( file, value.points[i] );
	}
	writeFloat( file, value.radius );
}

static void writeSweep( FILE* file, b2Sweep value )
{
	writeVec2( file, value.localCenter );
	writeVec2( file, value.c1 );
	writeVec2( file, value.c2 );
	writeRot( file, value.q1 );
	writeRot( file, value.q2 );
}

static void writeCache( FILE* file, b2SimplexCache value )
{
	writeInt( file, value.count );
	for ( int i = 0; i < 3; ++i )
	{
		writeInt( file, value.indexA[i] );
	}
	for ( int i = 0; i < 3; ++i )
	{
		writeInt( file, value.indexB[i] );
	}
}

static void writeManifold( FILE* file, b2Manifold value )
{
	writeVec2( file, value.normal );
	writeFloat( file, value.rollingImpulse );
	writeInt( file, value.pointCount );
	for ( int i = 0; i < value.pointCount; ++i )
	{
		b2ManifoldPoint point = value.points[i];
		writeVec2( file, point.point );
		writeVec2( file, point.anchorA );
		writeVec2( file, point.anchorB );
		writeFloat( file, point.separation );
		writeInt( file, point.id );
	}
}

static void writeDistanceOutput( FILE* file, b2DistanceOutput value )
{
	writeVec2( file, value.normal );
	writeVec2( file, value.pointA );
	writeVec2( file, value.pointB );
	writeFloat( file, value.distance );
	writeInt( file, value.iterations );
	writeInt( file, value.simplexCount );
}

static void writeHull( FILE* file, b2Hull value )
{
	writeInt( file, value.count );
	for ( int i = 0; i < B2_MAX_POLYGON_VERTICES; ++i )
	{
		writeVec2( file, i < value.count ? value.points[i] : b2Vec2_zero );
	}
}

static b2Polygon cleanPolygon( b2Polygon source )
{
	b2Polygon polygon;
	memset( &polygon, 0, sizeof( polygon ) );
	polygon.count = source.count;
	for ( int i = 0; i < source.count; ++i )
	{
		polygon.vertices[i] = source.vertices[i];
		polygon.normals[i] = source.normals[i];
	}
	polygon.centroid = source.centroid;
	polygon.radius = source.radius;
	return polygon;
}

static float shapeExtent( int caseIndex )
{
	return caseIndex % 4 == 1 ? 2.5f : 0.75f;
}

static b2Circle makeCircle( int caseIndex, float extent )
{
	b2Circle circle;
	circle.center = RandomVec2( -0.2f * extent, 0.2f * extent );
	circle.radius = caseIndex % 16 == 0 ? 0.0f : RandomFloatRange( 0.1f * extent, 0.6f * extent );
	return circle;
}

static b2Capsule makeCapsule( int caseIndex, float extent )
{
	b2Vec2 center = RandomVec2( -0.15f * extent, 0.15f * extent );
	b2Rot rotation = RandomRot();
	float halfLength = caseIndex % 16 == 1 ? 0.0005f : RandomFloatRange( 0.2f * extent, 0.8f * extent );
	b2Vec2 offset = b2RotateVector( rotation, (b2Vec2){ halfLength, 0.0f } );
	b2Capsule capsule = {
		.center1 = b2Sub( center, offset ),
		.center2 = b2Add( center, offset ),
		.radius = caseIndex % 16 == 0 ? 0.0f : RandomFloatRange( 0.05f * extent, 0.35f * extent ),
	};
	return capsule;
}

static b2Segment makeSegment( int caseIndex, float extent )
{
	b2Vec2 center = RandomVec2( -0.15f * extent, 0.15f * extent );
	b2Rot rotation = RandomRot();
	float halfLength = caseIndex % 16 == 1 ? 0.0005f : RandomFloatRange( 0.3f * extent, extent );
	b2Vec2 offset = b2RotateVector( rotation, (b2Vec2){ halfLength, 0.0f } );
	b2Segment segment = { b2Sub( center, offset ), b2Add( center, offset ) };
	return segment;
}

static b2ChainSegment makeChainSegment( int caseIndex, float extent )
{
	float halfLength = caseIndex % 16 == 1 ? 0.0005f : extent;
	b2ChainSegment segment = {
		.ghost1 = { -3.0f * halfLength, 0.0f },
		.segment = { { -halfLength, 0.0f }, { halfLength, 0.0f } },
		.ghost2 = { 3.0f * halfLength, 0.0f },
		.chainId = caseIndex + 1,
	};
	return segment;
}

static b2Polygon makePolygon( int caseIndex, float extent )
{
	b2Polygon source;
	int mode = caseIndex % 10;
	if ( mode == 0 )
	{
		source = b2MakeSquare( extent );
	}
	else if ( mode == 1 )
	{
		source = b2MakeBox( extent, 0.6f * extent );
	}
	else if ( mode == 2 )
	{
		source = b2MakeRoundedBox( 0.8f * extent, 0.55f * extent, 0.1f * extent );
	}
	else if ( mode == 3 )
	{
		source = RandomPolygon( extent );
	}
	else
	{
		b2Vec2 points[B2_MAX_POLYGON_VERTICES] = { 0 };
		int count = 3 + caseIndex % 6;
		b2Rot rotation = RandomRot();
		b2Rot delta = b2MakeRot( 2.0f * B2_PI / count );
		b2Vec2 point = b2RotateVector( rotation, (b2Vec2){ extent, 0.0f } );
		for ( int i = 0; i < count; ++i )
		{
			points[i] = point;
			point = b2RotateVector( delta, point );
		}
		b2Hull hull = b2ComputeHull( points, count );
		if ( hull.count == 0 )
		{
			fail( "regular polygon hull failed" );
		}
		source = b2MakePolygon( &hull, caseIndex % 3 == 0 ? 0.05f * extent : 0.0f );
	}

	return cleanPolygon( source );
}

static void makePairTransforms( int caseIndex, float extent, bool oneSided, b2Transform* transformA, b2Transform* transformB )
{
	float range = caseIndex % 4 == 0 ? 1.5f : ( caseIndex % 4 == 1 ? 10.0f : 3.0f );
	transformA->p = RandomVec2( -range, range );
	transformA->q = caseIndex % 4 == 2 ? b2Rot_identity : RandomRot();
	transformB->q = caseIndex % 4 == 2 ? b2Rot_identity : RandomRot();

	int placement = caseIndex / 4 % 4;
	float separation;
	if ( placement == 0 )
	{
		separation = 0.05f * extent;
	}
	else if ( placement == 1 )
	{
		separation = extent;
	}
	else if ( placement == 2 )
	{
		separation = 5.0f * extent + 1.0f;
	}
	else
	{
		separation = 0.3f * extent;
	}

	b2Vec2 localOffset;
	if ( oneSided )
	{
		localOffset = (b2Vec2){ RandomFloatRange( -0.4f * extent, 0.4f * extent ), -separation };
	}
	else
	{
		b2Vec2 direction = b2RotateVector( RandomRot(), (b2Vec2){ 1.0f, 0.0f } );
		localOffset = b2MulSV( separation, direction );
	}
	transformB->p = b2Add( transformA->p, b2RotateVector( transformA->q, localOffset ) );
}

static void writeCollisionTrace( const char* outputDirectory, const char* name, CollisionKind kind )
{
	FILE* file = openTrace( outputDirectory, "functions", name );
	writeHeader( file, "function", name );
	fprintf( file, "cases %d\n", caseCountCollide );
	g_randomSeed = RAND_SEED;

	int touchingCount = 0;
	for ( int caseIndex = 0; caseIndex < caseCountCollide; ++caseIndex )
	{
		float extent = shapeExtent( caseIndex );
		b2Transform transformA, transformB;
		bool oneSided = kind == collideChainSegmentAndCircle || kind == collideChainSegmentAndCapsule ||
						kind == collideChainSegmentAndPolygon;
		makePairTransforms( caseIndex, extent, oneSided, &transformA, &transformB );

		b2Manifold manifold = { 0 };
		b2SimplexCache cache = { 0 };
		fprintf( file, "case %d", caseIndex );

		switch ( kind )
		{
			case collideCircles:
			{
				b2Circle shapeA = makeCircle( caseIndex, extent );
				b2Circle shapeB = makeCircle( caseIndex + 7, extent );
				writeCircle( file, shapeA );
				writeTransform( file, transformA );
				writeCircle( file, shapeB );
				writeTransform( file, transformB );
				manifold = b2CollideCircles( &shapeA, transformA, &shapeB, transformB );
				break;
			}

			case collideCapsuleAndCircle:
			{
				b2Capsule shapeA = makeCapsule( caseIndex, extent );
				b2Circle shapeB = makeCircle( caseIndex + 7, extent );
				writeCapsule( file, shapeA );
				writeTransform( file, transformA );
				writeCircle( file, shapeB );
				writeTransform( file, transformB );
				manifold = b2CollideCapsuleAndCircle( &shapeA, transformA, &shapeB, transformB );
				break;
			}

			case collidePolygonAndCircle:
			{
				b2Polygon shapeA = makePolygon( caseIndex, extent );
				b2Circle shapeB = makeCircle( caseIndex + 7, extent );
				writePolygon( file, shapeA );
				writeTransform( file, transformA );
				writeCircle( file, shapeB );
				writeTransform( file, transformB );
				manifold = b2CollidePolygonAndCircle( &shapeA, transformA, &shapeB, transformB );
				break;
			}

			case collideCapsules:
			{
				b2Capsule shapeA = makeCapsule( caseIndex, extent );
				b2Capsule shapeB = makeCapsule( caseIndex + 7, extent );
				writeCapsule( file, shapeA );
				writeTransform( file, transformA );
				writeCapsule( file, shapeB );
				writeTransform( file, transformB );
				manifold = b2CollideCapsules( &shapeA, transformA, &shapeB, transformB );
				break;
			}

			case collideSegmentAndCapsule:
			{
				b2Segment shapeA = makeSegment( caseIndex, extent );
				b2Capsule shapeB = makeCapsule( caseIndex + 7, extent );
				writeSegment( file, shapeA );
				writeTransform( file, transformA );
				writeCapsule( file, shapeB );
				writeTransform( file, transformB );
				manifold = b2CollideSegmentAndCapsule( &shapeA, transformA, &shapeB, transformB );
				break;
			}

			case collidePolygonAndCapsule:
			{
				b2Polygon shapeA = makePolygon( caseIndex, extent );
				b2Capsule shapeB = makeCapsule( caseIndex + 7, extent );
				writePolygon( file, shapeA );
				writeTransform( file, transformA );
				writeCapsule( file, shapeB );
				writeTransform( file, transformB );
				manifold = b2CollidePolygonAndCapsule( &shapeA, transformA, &shapeB, transformB );
				break;
			}

			case collidePolygons:
			{
				b2Polygon shapeA = makePolygon( caseIndex, extent );
				b2Polygon shapeB = makePolygon( caseIndex + 7, extent );
				writePolygon( file, shapeA );
				writeTransform( file, transformA );
				writePolygon( file, shapeB );
				writeTransform( file, transformB );
				manifold = b2CollidePolygons( &shapeA, transformA, &shapeB, transformB );
				break;
			}

			case collideSegmentAndCircle:
			{
				b2Segment shapeA = makeSegment( caseIndex, extent );
				b2Circle shapeB = makeCircle( caseIndex + 7, extent );
				writeSegment( file, shapeA );
				writeTransform( file, transformA );
				writeCircle( file, shapeB );
				writeTransform( file, transformB );
				manifold = b2CollideSegmentAndCircle( &shapeA, transformA, &shapeB, transformB );
				break;
			}

			case collideSegmentAndPolygon:
			{
				b2Segment shapeA = makeSegment( caseIndex, extent );
				b2Polygon shapeB = makePolygon( caseIndex + 7, extent );
				writeSegment( file, shapeA );
				writeTransform( file, transformA );
				writePolygon( file, shapeB );
				writeTransform( file, transformB );
				manifold = b2CollideSegmentAndPolygon( &shapeA, transformA, &shapeB, transformB );
				break;
			}

			case collideChainSegmentAndCircle:
			{
				b2ChainSegment shapeA = makeChainSegment( caseIndex, extent );
				b2Circle shapeB = makeCircle( caseIndex + 7, extent );
				writeChainSegment( file, shapeA );
				writeTransform( file, transformA );
				writeCircle( file, shapeB );
				writeTransform( file, transformB );
				manifold = b2CollideChainSegmentAndCircle( &shapeA, transformA, &shapeB, transformB );
				break;
			}

			case collideChainSegmentAndCapsule:
			{
				b2ChainSegment shapeA = makeChainSegment( caseIndex, extent );
				b2Capsule shapeB = makeCapsule( caseIndex + 7, extent );
				writeChainSegment( file, shapeA );
				writeTransform( file, transformA );
				writeCapsule( file, shapeB );
				writeTransform( file, transformB );
				writeCache( file, cache );
				manifold = b2CollideChainSegmentAndCapsule( &shapeA, transformA, &shapeB, transformB, &cache );
				break;
			}

			case collideChainSegmentAndPolygon:
			{
				b2ChainSegment shapeA = makeChainSegment( caseIndex, extent );
				b2Polygon shapeB = makePolygon( caseIndex + 7, extent );
				writeChainSegment( file, shapeA );
				writeTransform( file, transformA );
				writePolygon( file, shapeB );
				writeTransform( file, transformB );
				writeCache( file, cache );
				manifold = b2CollideChainSegmentAndPolygon( &shapeA, transformA, &shapeB, transformB, &cache );
				break;
			}
		}

		fprintf( file, " |" );
		writeManifold( file, manifold );
		if ( kind == collideChainSegmentAndCapsule || kind == collideChainSegmentAndPolygon )
		{
			writeCache( file, cache );
		}
		fputc( '\n', file );
		touchingCount += manifold.pointCount > 0;
	}

	if ( fclose( file ) != 0 )
	{
		fail( "cannot close collision trace" );
	}
	printf( "function %s cases=%d manifolds=%d\n", name, caseCountCollide, touchingCount );
}

static b2ShapeProxy makeRandomProxy( int caseIndex, int salt, float extent )
{
	b2Vec2 points[B2_MAX_POLYGON_VERTICES] = { 0 };
	int count = 1 + ( caseIndex * 5 + salt * 3 ) % B2_MAX_POLYGON_VERTICES;
	if ( ( caseIndex + salt ) % 17 == 0 )
	{
		b2Vec2 center = RandomVec2( -0.2f * extent, 0.2f * extent );
		for ( int i = 0; i < count; ++i )
		{
			points[i] = b2Add( center, RandomVec2( -0.001f, 0.001f ) );
		}
	}
	else
	{
		for ( int i = 0; i < count; ++i )
		{
			points[i] = RandomVec2( -extent, extent );
		}
	}

	float radius = ( caseIndex + salt ) % 3 == 0 ? 0.0f : RandomFloatRange( 0.02f * extent, 0.3f * extent );
	b2ShapeProxy proxy = b2MakeProxy( points, count, radius );
	for ( int i = count; i < B2_MAX_POLYGON_VERTICES; ++i )
	{
		proxy.points[i] = b2Vec2_zero;
	}
	return proxy;
}

static void writeDistanceTrace( const char* outputDirectory )
{
	const char* name = "shape_distance";
	FILE* file = openTrace( outputDirectory, "functions", name );
	writeHeader( file, "function", name );
	fprintf( file, "cases %d\n", caseCountDistance );
	g_randomSeed = RAND_SEED;

	for ( int caseIndex = 0; caseIndex < caseCountDistance; ++caseIndex )
	{
		float extent = shapeExtent( caseIndex );
		b2DistanceInput input = { 0 };
		input.proxyA = makeRandomProxy( caseIndex, 1, extent );
		input.proxyB = makeRandomProxy( caseIndex, 2, extent );
		makePairTransforms( caseIndex, extent, false, &input.transformA, &input.transformB );
		input.useRadii = ( caseIndex & 1 ) != 0;
		b2SimplexCache cache = { 0 };
		b2DistanceOutput output = b2ShapeDistance( &input, &cache, NULL, 0 );

		fprintf( file, "case %d", caseIndex );
		writeProxy( file, input.proxyA );
		writeProxy( file, input.proxyB );
		writeTransform( file, input.transformA );
		writeTransform( file, input.transformB );
		writeInt( file, input.useRadii ? 1 : 0 );
		fprintf( file, " |" );
		writeDistanceOutput( file, output );
		fputc( '\n', file );
	}

	if ( fclose( file ) != 0 )
	{
		fail( "cannot close distance trace" );
	}
	printf( "function %s cases=%d\n", name, caseCountDistance );
}

static b2Sweep makeRandomSweep( void )
{
	b2Sweep sweep;
	sweep.localCenter = RandomVec2( -0.25f, 0.25f );
	sweep.c1 = RandomVec2( -10.0f, 10.0f );
	sweep.c2 = RandomVec2( -10.0f, 10.0f );
	sweep.q1 = RandomRot();
	sweep.q2 = RandomRot();
	return sweep;
}

static void writeTOITrace( const char* outputDirectory )
{
	const char* name = "time_of_impact";
	FILE* file = openTrace( outputDirectory, "functions", name );
	writeHeader( file, "function", name );
	fprintf( file, "cases %d\n", caseCountTOI );
	g_randomSeed = RAND_SEED;

	int hitCount = 0;
	for ( int caseIndex = 0; caseIndex < caseCountTOI; ++caseIndex )
	{
		float extent = caseIndex % 4 == 1 ? 1.5f : 0.5f;
		b2TOIInput input = { 0 };
		input.proxyA = makeRandomProxy( caseIndex, 3, extent );
		input.proxyB = makeRandomProxy( caseIndex, 4, extent );
		input.sweepA = makeRandomSweep();
		input.sweepB = makeRandomSweep();
		input.maxFraction = RandomFloatRange( 0.55f, 1.0f );
		if ( caseIndex % 4 == 2 )
		{
			input.sweepA.q1 = b2Rot_identity;
			input.sweepA.q2 = b2Rot_identity;
			input.sweepB.q1 = b2Rot_identity;
			input.sweepB.q2 = b2Rot_identity;
		}

		if ( ( caseIndex & 1 ) == 0 )
		{
			b2Vec2 center = RandomVec2( -2.0f, 2.0f );
			b2Vec2 axis = b2RotateVector( RandomRot(), (b2Vec2){ 1.0f, 0.0f } );
			float separation = 4.0f * extent + 2.0f;
			b2Vec2 offset = b2MulSV( separation, axis );
			input.sweepA.c1 = b2Sub( center, offset );
			input.sweepA.c2 = b2Add( center, offset );
			input.sweepB.c1 = b2Add( center, offset );
			input.sweepB.c2 = b2Sub( center, offset );
		}

		b2TOIOutput output = b2TimeOfImpact( &input );
		fprintf( file, "case %d", caseIndex );
		writeProxy( file, input.proxyA );
		writeProxy( file, input.proxyB );
		writeSweep( file, input.sweepA );
		writeSweep( file, input.sweepB );
		writeFloat( file, input.maxFraction );
		fprintf( file, " |" );
		writeInt( file, output.state );
		writeFloat( file, output.fraction );
		fputc( '\n', file );
		hitCount += output.state == b2_toiStateHit;
	}

	if ( fclose( file ) != 0 )
	{
		fail( "cannot close TOI trace" );
	}
	printf( "function %s cases=%d hits=%d state=%d\n", name, caseCountTOI, hitCount, b2_toiStateHit );
}

static void writeHullTrace( const char* outputDirectory )
{
	const char* name = "compute_hull";
	FILE* file = openTrace( outputDirectory, "functions", name );
	writeHeader( file, "function", name );
	fprintf( file, "cases %d\n", caseCountHull );
	g_randomSeed = RAND_SEED;

	for ( int caseIndex = 0; caseIndex < caseCountHull; ++caseIndex )
	{
		b2Vec2 points[B2_MAX_POLYGON_VERTICES] = { 0 };
		int count = 3 + caseIndex % 6;
		float extent = shapeExtent( caseIndex );
		if ( caseIndex % 8 == 0 && count >= 3 )
		{
			b2Vec2 center = RandomVec2( -extent, extent );
			b2Vec2 direction = b2RotateVector( RandomRot(), (b2Vec2){ 1.0f, 0.0f } );
			for ( int i = 0; i < count; ++i )
			{
				points[i] = b2MulAdd( center, 0.2f * extent * ( i - count / 2 ), direction );
			}
		}
		else
		{
			for ( int i = 0; i < count; ++i )
			{
				points[i] = RandomVec2( -extent, extent );
			}
			if ( caseIndex % 8 == 1 && count >= 2 )
			{
				points[count - 1] = points[0];
			}
		}

		b2Hull hull = b2ComputeHull( points, count );
		fprintf( file, "case %d", caseIndex );
		writeInt( file, count );
		for ( int i = 0; i < B2_MAX_POLYGON_VERTICES; ++i )
		{
			writeVec2( file, points[i] );
		}
		fprintf( file, " |" );
		writeHull( file, hull );
		fputc( '\n', file );
	}

	if ( fclose( file ) != 0 )
	{
		fail( "cannot close hull trace" );
	}
	printf( "function %s cases=%d\n", name, caseCountHull );
}

static void writeMakeRotTrace( const char* outputDirectory )
{
	const char* name = "make_rot";
	FILE* file = openTrace( outputDirectory, "functions", name );
	writeHeader( file, "function", name );
	fprintf( file, "cases %d\n", caseCountMath );
	g_randomSeed = RAND_SEED;

	for ( int caseIndex = 0; caseIndex < caseCountMath; ++caseIndex )
	{
		float radians = RandomFloatRange( -4.0f * B2_PI, 4.0f * B2_PI );
		float turns = radians / ( 2.0f * B2_PI );
		b2Rot rotation = b2MakeRot( radians );
		fprintf( file, "case %d", caseIndex );
		writeFloat( file, turns );
		fprintf( file, " |" );
		writeRot( file, rotation );
		fputc( '\n', file );
	}

	if ( fclose( file ) != 0 )
	{
		fail( "cannot close rotation trace" );
	}
	printf( "function %s cases=%d\n", name, caseCountMath );
}

static void writeAtan2Trace( const char* outputDirectory )
{
	const char* name = "atan2";
	FILE* file = openTrace( outputDirectory, "functions", name );
	writeHeader( file, "function", name );
	fprintf( file, "cases %d\n", caseCountMath );
	g_randomSeed = RAND_SEED;

	for ( int caseIndex = 0; caseIndex < caseCountMath; ++caseIndex )
	{
		float y, x;
		if ( caseIndex % 32 == 0 )
		{
			y = 0.0f;
			x = 0.0f;
		}
		else if ( caseIndex % 16 == 1 )
		{
			y = RandomFloatRange( -10.0f, 10.0f );
			x = 0.0f;
		}
		else if ( caseIndex % 16 == 2 )
		{
			y = 0.0f;
			x = RandomFloatRange( -10.0f, 10.0f );
		}
		else
		{
			b2Vec2 input = RandomVec2( -10.0f, 10.0f );
			y = input.y;
			x = input.x;
		}

		float radians = b2Atan2( y, x );
		fprintf( file, "case %d", caseIndex );
		writeFloat( file, y );
		writeFloat( file, x );
		fprintf( file, " |" );
		writeFloat( file, radians );
		fputc( '\n', file );
	}

	if ( fclose( file ) != 0 )
	{
		fail( "cannot close atan2 trace" );
	}
	printf( "function %s cases=%d\n", name, caseCountMath );
}

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
		b2BodyId* ids = realloc( list->ids, capacity * sizeof( b2BodyId ) );
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

static void collectBodies( b2WorldId worldId, BodyList* list )
{
	list->count = 0;
	b2AABB bounds = { { -1.0e6f, -1.0e6f }, { 1.0e6f, 1.0e6f } };
	b2World_OverlapAABB( worldId, bounds, b2DefaultQueryFilter(), collectBody, list );
	qsort( list->ids, list->count, sizeof( b2BodyId ), compareBodyIds );

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

static bool isSampledStep( int step, int totalSteps, int bodyCount, bool writeFinalBodies )
{
	if ( step == totalSteps )
	{
		return writeFinalBodies;
	}
	if ( bodyCount > 5000 )
	{
		return false;
	}
	if ( step == 1 )
	{
		return true;
	}
	return false;
}

static void writeStep( FILE* file, int step, int totalSteps, const BodyList* list, bool writeFinalBodies )
{
	fprintf( file, "step %d %016" PRIx64 "\n", step, hashBodies( list ) );
	if ( isSampledStep( step, totalSteps, list->count, writeFinalBodies ) )
	{
		for ( int i = 0; i < list->count; ++i )
		{
			b2Vec2 position = b2Body_GetPosition( list->ids[i] );
			b2Rot rotation = b2Body_GetRotation( list->ids[i] );
			fprintf( file, "body %d", i );
			writeVec2( file, position );
			writeRot( file, rotation );
			fputc( '\n', file );
		}
	}
}

static b2WorldId createTraceWorld( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.workerCount = 1;
	return b2CreateWorld( &worldDef );
}

static void writeSceneTrace( const char* outputDirectory, Scene scene )
{
	g_randomSeed = RAND_SEED;
	b2WorldId worldId = createTraceWorld();
	scene.create( worldId );

	BodyList bodies = { 0 };
	collectBodies( worldId, &bodies );
	int initialBodyCount = bodies.count;

	FILE* file = openTrace( outputDirectory, "scenes", scene.name );
	writeHeader( file, "scene", scene.name );
	fprintf( file, "bodies %d\n", initialBodyCount );
	fprintf( file, "steps %d\n", scene.steps );

	for ( int step = 1; step <= scene.steps; ++step )
	{
		if ( scene.beforeStep != NULL )
		{
			scene.beforeStep( worldId, step - 1 );
		}
		b2World_Step( worldId, 1.0f / 60.0f, 4 );
		collectBodies( worldId, &bodies );
		writeStep( file, step, scene.steps, &bodies, scene.writeFinalBodies );
	}

	if ( fclose( file ) != 0 )
	{
		fail( "cannot close scene trace" );
	}
	b2DestroyWorld( worldId );
	free( bodies.ids );
}

static int findFallingHingeStepCount( void )
{
	g_randomSeed = RAND_SEED;
	b2WorldId worldId = createTraceWorld();
	FallingHingeData data = CreateFallingHinges( worldId );
	bool done = false;
	int stepCount = 0;
	while ( done == false && stepCount < 2000 )
	{
		b2World_Step( worldId, 1.0f / 60.0f, 4 );
		done = UpdateFallingHinges( worldId, &data );
		stepCount += 1;
	}
	DestroyFallingHinges( &data );
	b2DestroyWorld( worldId );
	return stepCount;
}

static void writeFallingHingeTrace( const char* outputDirectory )
{
	const char* name = "falling_hinges";
	int totalSteps = findFallingHingeStepCount();

	g_randomSeed = RAND_SEED;
	b2WorldId worldId = createTraceWorld();
	FallingHingeData data = CreateFallingHinges( worldId );
	BodyList bodies = { 0 };
	collectBodies( worldId, &bodies );
	int initialBodyCount = bodies.count;

	FILE* file = openTrace( outputDirectory, "scenes", name );
	writeHeader( file, "scene", name );
	fprintf( file, "bodies %d\n", initialBodyCount );
	fprintf( file, "steps %d\n", totalSteps );

	for ( int step = 1; step <= totalSteps; ++step )
	{
		b2World_Step( worldId, 1.0f / 60.0f, 4 );
		UpdateFallingHinges( worldId, &data );
		collectBodies( worldId, &bodies );
		writeStep( file, step, totalSteps, &bodies, true );
	}

	if ( fclose( file ) != 0 )
	{
		fail( "cannot close falling hinge trace" );
	}
	DestroyFallingHinges( &data );
	b2DestroyWorld( worldId );
	free( bodies.ids );
}

int main( int argc, char** argv )
{
	if ( argc != 3 || strcmp( argv[1], "--out" ) != 0 )
	{
		fprintf( stderr, "usage: conformance --out <dir>\n" );
		return 2;
	}

	const char* outputDirectory = argv[2];
	char functionsDirectory[1024];
	char scenesDirectory[1024];
	if ( snprintf( functionsDirectory, sizeof( functionsDirectory ), "%s/functions", outputDirectory ) >=
			 (int)sizeof( functionsDirectory ) ||
		snprintf( scenesDirectory, sizeof( scenesDirectory ), "%s/scenes", outputDirectory ) >= (int)sizeof( scenesDirectory ) )
	{
		fail( "output path is too long" );
	}
	ensureDirectory( outputDirectory );
	ensureDirectory( functionsDirectory );
	ensureDirectory( scenesDirectory );

	struct
	{
		const char* name;
		CollisionKind kind;
	} collisions[] = {
		{ "collide_circles", collideCircles },
		{ "collide_capsule_and_circle", collideCapsuleAndCircle },
		{ "collide_polygon_and_circle", collidePolygonAndCircle },
		{ "collide_capsules", collideCapsules },
		{ "collide_segment_and_capsule", collideSegmentAndCapsule },
		{ "collide_polygon_and_capsule", collidePolygonAndCapsule },
		{ "collide_polygons", collidePolygons },
		{ "collide_segment_and_circle", collideSegmentAndCircle },
		{ "collide_segment_and_polygon", collideSegmentAndPolygon },
		{ "collide_chain_segment_and_circle", collideChainSegmentAndCircle },
		{ "collide_chain_segment_and_capsule", collideChainSegmentAndCapsule },
		{ "collide_chain_segment_and_polygon", collideChainSegmentAndPolygon },
	};
	for ( int i = 0; i < (int)( sizeof( collisions ) / sizeof( collisions[0] ) ); ++i )
	{
		writeCollisionTrace( outputDirectory, collisions[i].name, collisions[i].kind );
	}
	writeDistanceTrace( outputDirectory );
	writeTOITrace( outputDirectory );
	writeHullTrace( outputDirectory );
	writeMakeRotTrace( outputDirectory );
	writeAtan2Trace( outputDirectory );

	Scene scenes[] = {
		{ "joint_grid", CreateJointGrid, NULL, 500, true },
		{ "large_pyramid", CreateLargePyramid, NULL, 500, true },
		{ "many_pyramids", CreateManyPyramids, NULL, 200, false },
		{ "rain", CreateRain, StepRain, 1000, true },
		{ "smash", CreateSmash, NULL, 300, true },
		{ "spinner", CreateSpinner, StepSpinner, 1400, true },
		{ "tumbler", CreateTumbler, NULL, 750, true },
	};
	for ( int i = 0; i < (int)( sizeof( scenes ) / sizeof( scenes[0] ) ); ++i )
	{
		writeSceneTrace( outputDirectory, scenes[i] );
	}
	writeFallingHingeTrace( outputDirectory );

	return 0;
}
