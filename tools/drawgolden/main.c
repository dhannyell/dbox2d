// SPDX-License-Identifier: MIT
// Link against the frozen Box2D v3.1.1 checkout. See README.md.
#include "box2d/box2d.h"
#include <stdio.h>

static void header(const char* kind, b2HexColor color) { printf("%s|%06X|", kind, color); }
static void vector(b2Vec2 v) { printf("%.3f %.3f ", v.x, v.y); }
static void transform(b2Transform x) { vector(x.p); printf("%.3f %.3f ", x.q.c, x.q.s); }
static void end(void) { puts("|\"\""); }
static void polygon(const b2Vec2* v, int n, b2HexColor c, void* ctx) {
    (void)ctx; header("polygon", c); printf("%d ", n); for (int i = 0; i < n; ++i) vector(v[i]); end();
}
static void solidPolygon(b2Transform x, const b2Vec2* v, int n, float r, b2HexColor c, void* ctx) {
    (void)ctx; header("solidPolygon", c); transform(x); printf("%.3f %d ", r, n);
    for (int i = 0; i < n; ++i) vector(v[i]); end();
}
static void circle(b2Vec2 p, float r, b2HexColor c, void* ctx) {
    (void)ctx; header("circle", c); vector(p); printf("%.3f ", r); end();
}
static void solidCircle(b2Transform x, float r, b2HexColor c, void* ctx) {
    (void)ctx; header("solidCircle", c); transform(x); printf("%.3f ", r); end();
}
static void capsule(b2Vec2 a, b2Vec2 b, float r, b2HexColor c, void* ctx) {
    (void)ctx; header("capsule", c); vector(a); vector(b); printf("%.3f ", r); end();
}
static void segment(b2Vec2 a, b2Vec2 b, b2HexColor c, void* ctx) {
    (void)ctx; header("segment", c); vector(a); vector(b); end();
}
static void drawTransform(b2Transform x, void* ctx) {
    (void)ctx; header("transform", 0); transform(x); end();
}
static void point(b2Vec2 p, float size, b2HexColor c, void* ctx) {
    (void)ctx; header("point", c); vector(p); printf("%.3f ", size); end();
}
static void string(b2Vec2 p, const char* s, b2HexColor c, void* ctx) {
    (void)ctx; header("string", c); vector(p); printf("|\"%s\"\n", s);
}

// Every dynamic body sits far from the others and touches only the ground:
// a moved proxy then finds at most one new pair, keeping the port's pair
// order and graph colors identical to the reference's (D-013). No gap
// equals the speculative distance (4*b2_linearSlop).
int main(void) {
    b2WorldDef wd = b2DefaultWorldDef();
    wd.gravity = (b2Vec2){0, -10};
    b2WorldId w = b2CreateWorld(&wd);
    b2BodyDef bd = b2DefaultBodyDef();
    bd.position = (b2Vec2){0, -0.5f}; bd.name = "ground";
    b2BodyId ground = b2CreateBody(w, &bd);
    b2ShapeDef sd = b2DefaultShapeDef();
    b2Polygon floor = b2MakeBox(40, 0.5f);
    b2CreatePolygonShape(ground, &sd, &floor);

    b2Polygon box = b2MakeBox(0.5f, 0.5f);
    bd = b2DefaultBodyDef(); bd.type = b2_dynamicBody; bd.enableSleep = false;
    bd.position = (b2Vec2){-32, 3}; bd.name = "box";
    bd.rotation = b2MakeRot(0.1f * 2.0f * B2_PI);
    bd.fixedRotation = true;
    b2CreatePolygonShape(b2CreateBody(w, &bd), &sd, &box);

    b2ShapeDef slideSd = b2DefaultShapeDef();
    // A lower friction than the default keeps the box sliding through every step.
    slideSd.material.friction = 0.1f;
    bd = b2DefaultBodyDef(); bd.type = b2_dynamicBody; bd.enableSleep = false;
    bd.position = (b2Vec2){-24, 0.5f}; bd.name = "sliding";
    bd.linearVelocity = (b2Vec2){3, 0};
    b2CreatePolygonShape(b2CreateBody(w, &bd), &slideSd, &box);

    bd = b2DefaultBodyDef(); bd.type = b2_dynamicBody; bd.enableSleep = false;
    bd.position = (b2Vec2){-16, 3}; bd.name = "circle";
    b2Circle circ = {{0, 0}, 0.25f};
    b2CreateCircleShape(b2CreateBody(w, &bd), &sd, &circ);

    bd = b2DefaultBodyDef(); bd.type = b2_dynamicBody; bd.enableSleep = false;
    bd.position = (b2Vec2){-8, 3}; bd.name = "rounded";
    b2Polygon rounded = b2MakeRoundedBox(0.4f, 0.4f, 0.1f);
    b2CreatePolygonShape(b2CreateBody(w, &bd), &sd, &rounded);

    bd = b2DefaultBodyDef(); bd.type = b2_dynamicBody; bd.enableSleep = false;
    bd.position = (b2Vec2){0, 3}; bd.name = "hinge";
    b2BodyId hinge = b2CreateBody(w, &bd);
    b2CreatePolygonShape(hinge, &sd, &box);
    b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
    jd.bodyIdA = ground; jd.bodyIdB = hinge; jd.localAnchorA = (b2Vec2){0, 3.5f};
    jd.enableLimit = true; jd.lowerAngle = -0.25f * 2.0f * B2_PI; jd.upperAngle = 0.25f * 2.0f * B2_PI;
    b2CreateRevoluteJoint(w, &jd);

    b2Polygon small = b2MakeBox(0.3f, 0.3f);
    bd = b2DefaultBodyDef(); bd.type = b2_dynamicBody; bd.enableSleep = false;
    bd.position = (b2Vec2){8, 3}; bd.name = "distance";
    b2BodyId distanceBody = b2CreateBody(w, &bd);
    b2CreatePolygonShape(distanceBody, &sd, &small);
    b2DistanceJointDef dd = b2DefaultDistanceJointDef();
    dd.bodyIdA = ground; dd.bodyIdB = distanceBody;
    dd.localAnchorA = (b2Vec2){8, 5.5f};
    dd.length = 2;
    dd.enableLimit = true; dd.minLength = 1; dd.maxLength = 3;
    b2CreateDistanceJoint(w, &dd);

    bd = b2DefaultBodyDef(); bd.type = b2_dynamicBody; bd.enableSleep = false;
    bd.position = (b2Vec2){16, 6}; bd.name = "prismatic";
    b2BodyId prismaticBody = b2CreateBody(w, &bd);
    b2CreatePolygonShape(prismaticBody, &sd, &box);
    b2PrismaticJointDef pd = b2DefaultPrismaticJointDef();
    pd.bodyIdA = ground; pd.bodyIdB = prismaticBody;
    pd.localAnchorA = (b2Vec2){16, 3.5f};
    pd.localAxisA = (b2Vec2){0, 1};
    pd.enableLimit = true; pd.lowerTranslation = 0; pd.upperTranslation = 3;
    b2CreatePrismaticJoint(w, &pd);

    bd = b2DefaultBodyDef(); bd.type = b2_dynamicBody; bd.enableSleep = false;
    bd.position = (b2Vec2){24, 3}; bd.name = "wheel";
    b2BodyId wheelBody = b2CreateBody(w, &bd);
    b2CreatePolygonShape(wheelBody, &sd, &small);
    b2WheelJointDef wjd = b2DefaultWheelJointDef();
    wjd.bodyIdA = ground; wjd.bodyIdB = wheelBody;
    wjd.localAnchorA = (b2Vec2){24, 3.5f};
    wjd.enableMotor = true; wjd.motorSpeed = 2.0f * B2_PI; wjd.maxMotorTorque = 10;
    b2CreateWheelJoint(w, &wjd);

    bd = b2DefaultBodyDef(); bd.type = b2_dynamicBody; bd.isAwake = false;
    bd.position = (b2Vec2){32, 0.25f}; bd.name = "capsule";
    b2Capsule cap = {{-0.5f, 0}, {0.5f, 0}, 0.25f};
    b2CreateCapsuleShape(b2CreateBody(w, &bd), &sd, &cap);

    bd = b2DefaultBodyDef();
    bd.position = (b2Vec2){40, 5};
    b2Segment seg = {{-1, 0}, {1, 0}};
    b2CreateSegmentShape(b2CreateBody(w, &bd), &sd, &seg);

    bd = b2DefaultBodyDef();
    bd.position = (b2Vec2){48, 3}; bd.name = "sensor";
    b2ShapeDef sensorSd = b2DefaultShapeDef();
    sensorSd.isSensor = true;
    b2CreatePolygonShape(b2CreateBody(w, &bd), &sensorSd, &box);

    for (int i = 0; i < 90; ++i) b2World_Step(w, 1.0f / 60.0f, 4);

    b2DebugDraw d = b2DefaultDebugDraw();
    d.DrawPolygonFcn = polygon; d.DrawSolidPolygonFcn = solidPolygon;
    d.DrawCircleFcn = circle; d.DrawSolidCircleFcn = solidCircle;
    d.DrawSolidCapsuleFcn = capsule; d.DrawSegmentFcn = segment;
    d.DrawTransformFcn = drawTransform; d.DrawPointFcn = point; d.DrawStringFcn = string;
    d.drawShapes = d.drawJoints = d.drawJointExtras = d.drawBounds = d.drawMass = true;
    d.drawBodyNames = d.drawContacts = d.drawGraphColors = d.drawContactNormals = true;
    d.drawContactImpulses = d.drawContactFeatures = d.drawFrictionImpulses = d.drawIslands = true;
    puts("all"); b2World_Draw(w, &d);
    d.useDrawingBounds = true;
    d.drawingBounds = (b2AABB){{-36, -1}, {4, 5}};
    puts("bounded"); b2World_Draw(w, &d);
    b2DestroyWorld(w);
    return 0;
}
