// SPDX-License-Identifier: MIT
// Link against the frozen Box2D v3.2.0 checkout. See README.md.
#include "box2d/box2d.h"
#include <stdio.h>

static void header( const char* kind, b2HexColor color ) { printf( "%s|%06X|", kind, color ); }
static void vector( b2Vec2 v ) { printf( "%.3f %.3f ", v.x, v.y ); }
static void position( b2Pos p ) { printf( "%.3f %.3f ", (double)p.x, (double)p.y ); }
static void transform( b2WorldTransform x )
{
	position( x.p );
	printf( "%.3f %.3f ", x.q.c, x.q.s );
}
static void end( void ) { puts( "|\"\"" ); }
static void polygon( b2WorldTransform x, const b2Vec2* v, int n, b2HexColor c, void* ctx )
{
	(void)ctx;
	header( "polygon", c );
	transform( x );
	printf( "%d ", n );
	for ( int i = 0; i < n; ++i )
		vector( v[i] );
	end();
}
static void solidPolygon( b2WorldTransform x, const b2Vec2* v, int n, float r, b2HexColor c, void* ctx )
{
	(void)ctx;
	header( "solidPolygon", c );
	transform( x );
	printf( "%.3f %d ", r, n );
	for ( int i = 0; i < n; ++i )
		vector( v[i] );
	end();
}
static void circle( b2Pos p, float r, b2HexColor c, void* ctx )
{
	(void)ctx;
	header( "circle", c );
	position( p );
	printf( "%.3f ", r );
	end();
}
static void solidCircle( b2WorldTransform x, b2Vec2 center, float r, b2HexColor c, void* ctx )
{
	(void)ctx;
	header( "solidCircle", c );
	transform( x );
	vector( center );
	printf( "%.3f ", r );
	end();
}
static void capsule( b2Pos a, b2Pos b, float r, b2HexColor c, void* ctx )
{
	(void)ctx;
	header( "capsule", c );
	position( a );
	position( b );
	printf( "%.3f ", r );
	end();
}
static void line( b2Pos a, b2Pos b, b2HexColor c, void* ctx )
{
	(void)ctx;
	header( "line", c );
	position( a );
	position( b );
	end();
}
static void drawTransform( b2WorldTransform x, void* ctx )
{
	(void)ctx;
	header( "transform", 0 );
	transform( x );
	end();
}
static void point( b2Pos p, float size, b2HexColor c, void* ctx )
{
	(void)ctx;
	header( "point", c );
	position( p );
	printf( "%.3f ", size );
	end();
}
static void string( b2Pos p, const char* s, b2HexColor c, void* ctx )
{
	(void)ctx;
	header( "string", c );
	position( p );
	printf( "|\"%s\"\n", s );
}
static void bounds( b2AABB box, b2HexColor c, void* ctx )
{
	(void)ctx;
	header( "bounds", c );
	vector( box.lowerBound );
	vector( box.upperBound );
	end();
}

// Every dynamic body sits far from the others and touches only the ground:
// a moved proxy then finds at most one new pair, keeping the port's pair
// order and graph colors identical to the reference's. No gap equals the
// speculative distance (4*B2_LINEAR_SLOP). Angles in the port are turns.
int main( void )
{
	const float turn = 2.0f * B2_PI;

	b2WorldDef wd = b2DefaultWorldDef();
	wd.gravity = (b2Vec2){ 0, -10 };
	b2WorldId w = b2CreateWorld( &wd );
	b2BodyDef bd = b2DefaultBodyDef();
	bd.position = (b2Pos){ 0, -0.5f };
	bd.name = "ground";
	b2BodyId ground = b2CreateBody( w, &bd );
	b2ShapeDef sd = b2DefaultShapeDef();
	b2Polygon floor = b2MakeBox( 40, 0.5f );
	b2CreatePolygonShape( ground, &sd, &floor );

	b2Polygon box = b2MakeBox( 0.5f, 0.5f );
	bd = b2DefaultBodyDef();
	bd.type = b2_dynamicBody;
	bd.enableSleep = false;
	bd.position = (b2Pos){ -32, 3 };
	bd.name = "box";
	bd.rotation = b2MakeRot( 0.1f * turn );
	bd.motionLocks.angularZ = true;
	b2CreatePolygonShape( b2CreateBody( w, &bd ), &sd, &box );

	b2ShapeDef slideSd = b2DefaultShapeDef();
	// A lower friction than the default keeps the box sliding through every step.
	slideSd.material.friction = 0.1f;
	bd = b2DefaultBodyDef();
	bd.type = b2_dynamicBody;
	bd.enableSleep = false;
	bd.position = (b2Pos){ -24, 0.5f };
	bd.name = "sliding";
	bd.linearVelocity = (b2Vec2){ 3, 0 };
	b2CreatePolygonShape( b2CreateBody( w, &bd ), &slideSd, &box );

	bd = b2DefaultBodyDef();
	bd.type = b2_dynamicBody;
	bd.enableSleep = false;
	bd.position = (b2Pos){ -16, 3 };
	bd.name = "circle";
	b2Circle circ = { { 0, 0 }, 0.25f };
	b2CreateCircleShape( b2CreateBody( w, &bd ), &sd, &circ );

	bd = b2DefaultBodyDef();
	bd.type = b2_dynamicBody;
	bd.enableSleep = false;
	bd.position = (b2Pos){ -8, 3 };
	bd.name = "rounded";
	b2Polygon rounded = b2MakeRoundedBox( 0.4f, 0.4f, 0.1f );
	b2CreatePolygonShape( b2CreateBody( w, &bd ), &sd, &rounded );

	bd = b2DefaultBodyDef();
	bd.type = b2_dynamicBody;
	bd.enableSleep = false;
	bd.position = (b2Pos){ 0, 3 };
	bd.name = "hinge";
	b2BodyId hinge = b2CreateBody( w, &bd );
	b2CreatePolygonShape( hinge, &sd, &box );
	b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
	jd.base.bodyIdA = ground;
	jd.base.bodyIdB = hinge;
	jd.base.localFrameA.p = (b2Vec2){ 0, 3.5f };
	jd.enableLimit = true;
	jd.lowerAngle = -0.25f * turn;
	jd.upperAngle = 0.25f * turn;
	b2CreateRevoluteJoint( w, &jd );

	b2Polygon small = b2MakeBox( 0.3f, 0.3f );
	bd = b2DefaultBodyDef();
	bd.type = b2_dynamicBody;
	bd.enableSleep = false;
	bd.position = (b2Pos){ 8, 3 };
	bd.name = "distance";
	b2BodyId distanceBody = b2CreateBody( w, &bd );
	b2CreatePolygonShape( distanceBody, &sd, &small );
	b2DistanceJointDef dd = b2DefaultDistanceJointDef();
	dd.base.bodyIdA = ground;
	dd.base.bodyIdB = distanceBody;
	dd.base.localFrameA.p = (b2Vec2){ 8, 5.5f };
	dd.length = 2;
	dd.enableLimit = true;
	dd.minLength = 1;
	dd.maxLength = 3;
	b2CreateDistanceJoint( w, &dd );

	bd = b2DefaultBodyDef();
	bd.type = b2_dynamicBody;
	bd.enableSleep = false;
	bd.position = (b2Pos){ 16, 6 };
	bd.name = "prismatic";
	b2BodyId prismaticBody = b2CreateBody( w, &bd );
	b2CreatePolygonShape( prismaticBody, &sd, &box );
	b2PrismaticJointDef pd = b2DefaultPrismaticJointDef();
	pd.base.bodyIdA = ground;
	pd.base.bodyIdB = prismaticBody;
	pd.base.localFrameA.p = (b2Vec2){ 16, 3.5f };
	pd.base.localFrameA.q = b2MakeRotFromUnitVector( (b2Vec2){ 0, 1 } );
	pd.base.localFrameB.q = pd.base.localFrameA.q;
	pd.enableLimit = true;
	pd.lowerTranslation = 0;
	pd.upperTranslation = 3;
	b2CreatePrismaticJoint( w, &pd );

	bd = b2DefaultBodyDef();
	bd.type = b2_dynamicBody;
	bd.enableSleep = false;
	bd.position = (b2Pos){ 24, 3 };
	bd.name = "wheel";
	b2BodyId wheelBody = b2CreateBody( w, &bd );
	b2CreatePolygonShape( wheelBody, &sd, &small );
	b2WheelJointDef wjd = b2DefaultWheelJointDef();
	wjd.base.bodyIdA = ground;
	wjd.base.bodyIdB = wheelBody;
	wjd.base.localFrameA.p = (b2Vec2){ 24, 3.5f };
	wjd.base.localFrameA.q = b2MakeRot( 0.25f * turn );
	wjd.enableMotor = true;
	wjd.motorSpeed = turn;
	wjd.maxMotorTorque = 10;
	b2CreateWheelJoint( w, &wjd );

	bd = b2DefaultBodyDef();
	bd.type = b2_dynamicBody;
	bd.isAwake = false;
	bd.position = (b2Pos){ 32, 0.25f };
	bd.name = "capsule";
	b2Capsule cap = { { -0.5f, 0 }, { 0.5f, 0 }, 0.25f };
	b2CreateCapsuleShape( b2CreateBody( w, &bd ), &sd, &cap );

	bd = b2DefaultBodyDef();
	bd.position = (b2Pos){ 40, 5 };
	b2Segment seg = { { -1, 0 }, { 1, 0 } };
	b2CreateSegmentShape( b2CreateBody( w, &bd ), &sd, &seg );

	bd = b2DefaultBodyDef();
	bd.position = (b2Pos){ 48, 3 };
	bd.name = "sensor";
	b2ShapeDef sensorSd = b2DefaultShapeDef();
	sensorSd.isSensor = true;
	b2CreatePolygonShape( b2CreateBody( w, &bd ), &sensorSd, &box );

	bd = b2DefaultBodyDef();
	bd.position = (b2Pos){ 56, 5 };
	bd.name = "chain";
	b2Vec2 chainPoints[4] = { { -1, -1 }, { 1, -1 }, { 1, 1 }, { -1, 1 } };
	b2ChainDef cd = b2DefaultChainDef();
	cd.points = chainPoints;
	cd.pointCount = 4;
	cd.isLoop = true;
	b2CreateChain( b2CreateBody( w, &bd ), &cd );

	for ( int i = 0; i < 90; ++i )
		b2World_Step( w, 1.0f / 60.0f, 4 );

	b2DebugDraw d = b2DefaultDebugDraw();
	d.DrawPolygonFcn = polygon;
	d.DrawSolidPolygonFcn = solidPolygon;
	d.DrawCircleFcn = circle;
	d.DrawSolidCircleFcn = solidCircle;
	d.DrawSolidCapsuleFcn = capsule;
	d.DrawLineFcn = line;
	d.DrawTransformFcn = drawTransform;
	d.DrawPointFcn = point;
	d.DrawStringFcn = string;
	d.DrawBoundsFcn = bounds;
	// A small force scale keeps fixed mode's force lines inside the budget.
	d.forceScale = 0.015625f;
	d.drawShapes = d.drawChainNormals = d.drawJoints = d.drawJointExtras = d.drawBounds = d.drawMass = true;
	d.drawBodyNames = d.drawContacts = d.drawGraphColors = d.drawContactNormals = true;
	d.drawContactFeatures = d.drawFrictionForces = d.drawIslands = true;
	puts( "all" );
	b2World_Draw( w, &d );
	d.drawContactNormals = false;
	d.drawContactForces = true;
	d.drawAnchorA = true;
	d.drawGraphColors = false;
	d.jointScale = 2.0f;
	d.drawingBounds = (b2AABB){ { -36, -1 }, { 4, 5 } };
	puts( "bounded" );
	b2World_Draw( w, &d );
	b2DestroyWorld( w );
	return 0;
}
