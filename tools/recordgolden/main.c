// SPDX-License-Identifier: MIT
// Link against the frozen Box2D v3.2.0 checkout. See README.md.
//
// Builds a fixed scene, records 60 steps with API calls in between, then
// writes the recording and the final snapshot. TestRecording* in the Go
// module builds the same scene and compares the bytes.
#include "box2d/box2d.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SCENE_STEPS 60

static int s_overlapCount;
static int s_rayCount;

static b2Pos pos( float x, float y )
{
	b2Vec2 v = { x, y };
	return b2ToPos( v );
}

static bool overlapFcn( b2ShapeId shapeId, void* context )
{
	(void)shapeId;
	(void)context;
	s_overlapCount += 1;
	return true;
}

static float castFcn( b2ShapeId shapeId, b2Pos point, b2Vec2 normal, float fraction, void* context )
{
	(void)shapeId;
	(void)point;
	(void)normal;
	(void)context;
	s_rayCount += 1;
	return fraction;
}

static void printBits( const void* p, int size )
{
	const uint8_t* bytes = (const uint8_t*)p;
	for ( int i = size - 1; i >= 0; --i )
	{
		printf( "%02x", bytes[i] );
	}
}

// b2Filter is 20 bytes of data and 4 bytes of padding. A struct copy carries
// the padding of the default along, and it is stack garbage. The snapshot
// holds raw shapes, so zero it to keep the golden files reproducible.
static b2ShapeDef defaultShapeDef( void )
{
	b2ShapeDef def = b2DefaultShapeDef();
	memset( (uint8_t*)&def.filter + 20, 0, sizeof( b2Filter ) - 20 );
	return def;
}

static b2BodyId makeBody( b2WorldId worldId, b2BodyType type, float x, float y )
{
	b2BodyDef def = b2DefaultBodyDef();
	def.type = type;
	def.position = pos( x, y );
	return b2CreateBody( worldId, &def );
}

int main( int argc, char** argv )
{
	const char* prefix = argc > 1 ? argv[1] : "golden";

	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );

	b2ShapeDef shapeDef = defaultShapeDef();

	// Ground
	b2BodyId ground = makeBody( worldId, b2_staticBody, 0.0f, -1.0f );
	b2Polygon groundBox = b2MakeBox( 20.0f, 1.0f );
	b2CreatePolygonShape( ground, &shapeDef, &groundBox );

	// A stack of three boxes
	b2Polygon box = b2MakeBox( 0.5f, 0.5f );
	b2BodyId box1 = makeBody( worldId, b2_dynamicBody, 0.0f, 0.5f );
	b2BodyId box2 = makeBody( worldId, b2_dynamicBody, 0.25f, 1.6f );
	b2BodyId box3 = makeBody( worldId, b2_dynamicBody, -0.25f, 2.7f );
	b2CreatePolygonShape( box1, &shapeDef, &box );
	b2CreatePolygonShape( box2, &shapeDef, &box );
	b2CreatePolygonShape( box3, &shapeDef, &box );

	// A circle and a capsule
	b2BodyId circleBody = makeBody( worldId, b2_dynamicBody, 2.0f, 4.0f );
	b2ShapeDef circleDef = defaultShapeDef();
	circleDef.material.restitution = 0.5f;
	b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
	b2ShapeId circleShape = b2CreateCircleShape( circleBody, &circleDef, &circle );

	b2BodyId capsuleBody = makeBody( worldId, b2_dynamicBody, -3.0f, 3.0f );
	b2Capsule capsule = { { -0.5f, 0.0f }, { 0.5f, 0.0f }, 0.25f };
	b2CreateCapsuleShape( capsuleBody, &shapeDef, &capsule );

	// A pendulum on a revolute joint
	b2BodyId pendulum = makeBody( worldId, b2_dynamicBody, 5.0f, 5.0f );
	b2Polygon small = b2MakeBox( 0.25f, 0.25f );
	b2CreatePolygonShape( pendulum, &shapeDef, &small );
	b2RevoluteJointDef revoluteDef = b2DefaultRevoluteJointDef();
	revoluteDef.base.bodyIdA = ground;
	revoluteDef.base.bodyIdB = pendulum;
	revoluteDef.base.localFrameA.p = ( b2Vec2 ){ 5.0f, 7.0f };
	revoluteDef.base.localFrameB.p = ( b2Vec2 ){ 0.0f, 1.0f };
	b2CreateRevoluteJoint( worldId, &revoluteDef );

	// A spring on a distance joint
	b2DistanceJointDef distanceDef = b2DefaultDistanceJointDef();
	distanceDef.base.bodyIdA = ground;
	distanceDef.base.bodyIdB = circleBody;
	distanceDef.base.localFrameA.p = ( b2Vec2 ){ 3.0f, 7.0f };
	distanceDef.length = 2.0f;
	distanceDef.enableSpring = true;
	distanceDef.hertz = 2.0f;
	distanceDef.dampingRatio = 0.5f;
	b2JointId distanceJoint = b2CreateDistanceJoint( worldId, &distanceDef );

	// A weld between the first two boxes
	b2WeldJointDef weldDef = b2DefaultWeldJointDef();
	weldDef.base.bodyIdA = box1;
	weldDef.base.bodyIdB = box2;
	weldDef.base.localFrameA.p = ( b2Vec2 ){ 0.25f, 0.55f };
	weldDef.base.localFrameB.p = ( b2Vec2 ){ 0.0f, -0.55f };
	weldDef.linearHertz = 5.0f;
	weldDef.linearDampingRatio = 0.7f;
	b2JointId weldJoint = b2CreateWeldJoint( worldId, &weldDef );

	b2Recording* recording = b2CreateRecording( 0 );
	b2World_StartRecording( worldId, recording );

	b2BodyId extra = b2_nullBodyId;
	float dt = 1.0f / 60.0f;
	for ( int step = 0; step < SCENE_STEPS; ++step )
	{
		switch ( step )
		{
			case 5:
				b2World_SetGravity( worldId, ( b2Vec2 ){ 0.0f, -9.8f } );
				b2Body_SetLinearVelocity( circleBody, ( b2Vec2 ){ 1.0f, 0.0f } );
				break;

			case 10:
			{
				extra = makeBody( worldId, b2_dynamicBody, -1.0f, 5.0f );
				b2Circle tiny = { { 0.0f, 0.0f }, 0.25f };
				b2CreateCircleShape( extra, &shapeDef, &tiny );
			}
			break;

			case 12:
				b2Body_SetName( extra, "extra" );
				break;

			case 15:
				b2Body_ApplyLinearImpulseToCenter( box3, ( b2Vec2 ){ 2.0f, 0.0f }, true );
				b2Body_ApplyForceToCenter( box1, ( b2Vec2 ){ 10.0f, 0.0f }, true );
				break;

			case 20:
			{
				b2QueryFilter filter = b2DefaultQueryFilter();
				b2RayResult ray = b2World_CastRayClosest( worldId, pos( -10.0f, 3.0f ), ( b2Vec2 ){ 20.0f, 0.0f }, filter );
				(void)ray;
				b2AABB aabb = { { -5.0f, 0.0f }, { 5.0f, 5.0f } };
				b2World_OverlapAABB( worldId, pos( 0.0f, 0.0f ), aabb, filter, overlapFcn, NULL );
				b2World_CastRay( worldId, pos( -10.0f, 1.0f ), ( b2Vec2 ){ 20.0f, 0.0f }, filter, castFcn, NULL );
			}
			break;

			case 25:
				b2Shape_SetRestitution( circleShape, 0.25f );
				break;

			case 30:
				b2DistanceJoint_SetSpringHertz( distanceJoint, 3.0f );
				break;

			case 35:
				b2World_EnableSleeping( worldId, false );
				break;

			case 40:
				b2DestroyJoint( weldJoint );
				break;

			case 45:
				b2DestroyBody( extra );
				break;

			default:
				break;
		}

		b2World_Step( worldId, dt, 4 );
	}

	b2World_StopRecording( worldId );

	char path[512];
	snprintf( path, sizeof( path ), "%s.b2rec", prefix );
	if ( b2Recording_SaveToFile( recording, path ) == false )
	{
		fprintf( stderr, "cannot write %s\n", path );
		return 1;
	}

	int snapshotSize = b2World_GetSnapshot( worldId, NULL, 0 );
	uint8_t* snapshot = (uint8_t*)malloc( (size_t)snapshotSize );
	b2World_GetSnapshot( worldId, snapshot, snapshotSize );
	snprintf( path, sizeof( path ), "%s.snapshot", prefix );
	FILE* f = fopen( path, "wb" );
	if ( f == NULL || fwrite( snapshot, 1, (size_t)snapshotSize, f ) != (size_t)snapshotSize )
	{
		fprintf( stderr, "cannot write %s\n", path );
		return 1;
	}
	fclose( f );

	printf( "recordBytes %d\n", b2Recording_GetSize( recording ) );
	printf( "snapshotBytes %d\n", snapshotSize );
	printf( "overlapCount %d\nrayCount %d\n", s_overlapCount, s_rayCount );
	printf( "stateHash %016llx\n", (unsigned long long)b2World_GetStateHash( worldId ) );

	b2BodyId bodies[] = { ground, box1, box2, box3, circleBody, capsuleBody, pendulum };
	for ( int i = 0; i < 7; ++i )
	{
		b2WorldTransform t = b2Body_GetTransform( bodies[i] );
		printf( "body %d ", i );
		printBits( &t.p.x, (int)sizeof( t.p.x ) );
		printf( " " );
		printBits( &t.p.y, (int)sizeof( t.p.y ) );
		printf( " " );
		printBits( &t.q.c, 4 );
		printf( " " );
		printBits( &t.q.s, 4 );
		printf( "\n" );
	}

	// The reference replays its own recording to the end
	b2Replay* replay = b2CreateReplay( b2Recording_GetData( recording ), b2Recording_GetSize( recording ), 1 );
	if ( replay != NULL )
	{
		while ( b2Replay_StepFrame( replay ) )
		{
		}
		b2ReplayInfo info = b2Replay_GetInfo( replay );
		printf( "replayFrames %d\nreplayFrame %d\nreplayDiverged %d\n", info.frameCount, b2Replay_GetFrame( replay ),
				b2Replay_HasDiverged( replay ) ? 1 : 0 );
		b2DestroyReplay( replay );
	}
	else
	{
		printf( "replay failed\n" );
	}

	free( snapshot );
	b2DestroyRecording( recording );
	b2DestroyWorld( worldId );
	return 0;
}
