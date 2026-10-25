#include "check.h"

#include "character/CharacterBody.h"
#include "collision/CollisionShape.h"

#include <kinetix/core/Math.h>

using kinetix::f32;
using kinetix::u32;
using kinetix::Physics::CharacterBody;
using kinetix::Physics::CharacterSweepHit;
using kinetix::Physics::TrimeshShape;
using kinetix::Physics::TrimeshSweepSource;

namespace
{

const f32 kStep = 1.0f / 60.0f;

// Floor quad at y = 0 facing +y, and a wall at x = 1 facing -x.
struct Room
{
    kinetix::Math::vec3 vertices[8] = {{-10, 0, -10}, {10, 0, -10}, {10, 0, 10}, {-10, 0, 10},
                                       {1, -1, -10},  {1, -1, 10},  {1, 5, 10},  {1, 5, -10}};
    u32 indices[12] = {0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7};
    TrimeshShape mesh{vertices, 8, indices, 12};
};

// An overlap's negative t is a world depth along the normal, at any scale.
void overlapPushOutIsInWorldUnits()
{
    for (f32 s : {1.0f, 100.0f})
    {
        kinetix::Math::vec3 vertices[4] = {
            {1 * s, -10 * s, -10 * s}, {1 * s, -10 * s, 10 * s}, {1 * s, 10 * s, 10 * s}, {1 * s, 10 * s, -10 * s}};
        u32 indices[6] = {0, 1, 2, 0, 2, 3};
        TrimeshShape mesh(vertices, 4, indices, 6);
        TrimeshSweepSource source(mesh, kinetix::Math::mat4(1.0f));

        // Radius 0.3*s, centre 0.1*s inside the wall.
        const kinetix::Math::vec3 centre(0.8f * s, 0.0f, 0.0f);
        const kinetix::Math::vec3 radii(0.3f * s, 0.9f * s, 0.3f * s);
        CharacterSweepHit hit;
        CHECK(source.sweepEllipsoid(centre, kinetix::Math::mat3(1.0f), radii,
                                    kinetix::Math::vec3(0.01f * s, 0.0f, 0.0f), hit));
        CHECK_NEAR(hit.t, -0.1f * s, 1.0e-3f * s);

        CharacterBody body;
        body.setShape(0.3f * s, 1.2f * s);
        body.setSkinWidth(0.0f);
        body.setStepOffset(0.0f);
        body.setPosition(centre);
        body.move(kinetix::Math::vec3(0.01f * s, 0.0f, 0.0f), source);
        CHECK_NEAR(body.position().x, 0.7f * s, 1.0e-3f * s);
    }
}

// Sweeping through the mesh transform must match triangles baked into world space, including uneven scale
// where the transpose of the linear part is not its inverse.
void transformedMeshMatchesBakedMesh()
{
    const kinetix::Math::vec3 local[4] = {{-2, 0, -2}, {2, 0, -2}, {2, 0, 2}, {-2, 0, 2}};
    const u32 indices[6] = {0, 2, 1, 0, 3, 2};

    kinetix::Math::mat4 transforms[3];
    transforms[0] = kinetix::Math::translate(kinetix::Math::mat4(1.0f), kinetix::Math::vec3(0.5f, -0.2f, 0.1f)) *
                    kinetix::Math::rotate(kinetix::Math::mat4(1.0f), 0.7f, kinetix::Math::vec3(0, 1, 0)) *
                    kinetix::Math::scale(kinetix::Math::mat4(1.0f), kinetix::Math::vec3(2.0f));
    transforms[1] = kinetix::Math::translate(kinetix::Math::mat4(1.0f), kinetix::Math::vec3(0.0f, 0.3f, 0.0f)) *
                    kinetix::Math::rotate(kinetix::Math::mat4(1.0f), kinetix::Math::radians(25.0f),
                                          kinetix::Math::vec3(0, 0, 1));
    transforms[2] = kinetix::Math::rotate(kinetix::Math::mat4(1.0f), kinetix::Math::radians(20.0f),
                                          kinetix::Math::vec3(1, 0, 1)) *
                    kinetix::Math::scale(kinetix::Math::mat4(1.0f), kinetix::Math::vec3(3.0f, 1.0f, 0.5f));

    for (const kinetix::Math::mat4& transform : transforms)
    {
        kinetix::Math::vec3 baked[4];
        for (u32 i = 0; i < 4; ++i)
            baked[i] = kinetix::Math::vec3(transform * kinetix::Math::vec4(local[i], 1.0f));
        TrimeshShape localMesh(local, 4, indices, 6);
        TrimeshShape bakedMesh(baked, 4, indices, 6);
        TrimeshSweepSource throughTransform(localMesh, transform);
        TrimeshSweepSource identity(bakedMesh, kinetix::Math::mat4(1.0f));

        const kinetix::Math::vec3 radii(0.3f, 0.9f, 0.3f);
        const kinetix::Math::vec3 starts[2] = {{0.2f, 2.0f, 0.1f}, {0.0f, 0.5f, 0.0f}};
        for (const kinetix::Math::vec3& start : starts)
        {
            CharacterSweepHit a;
            CharacterSweepHit b;
            const kinetix::Math::vec3 velocity(0.3f, -3.0f, -0.2f);
            const bool hitA = throughTransform.sweepEllipsoid(start, kinetix::Math::mat3(1.0f), radii, velocity, a);
            const bool hitB = identity.sweepEllipsoid(start, kinetix::Math::mat3(1.0f), radii, velocity, b);
            CHECK(hitA);
            CHECK(hitB);
            CHECK_NEAR(a.t, b.t, 1.0e-3f);
            CHECK_NEAR(kinetix::Math::dot(a.normal, b.normal), 1.0f, 1.0e-4f);
        }
    }
}

// With up along +x the capsule stands along x: resting on a wall facing +x
// its centre sits a half-height off it, not a radius.
void capsuleStandsAlongUpDirection()
{
    kinetix::Math::vec3 vertices[4] = {{0, -10, -10}, {0, 10, -10}, {0, 10, 10}, {0, -10, 10}};
    u32 indices[6] = {0, 1, 2, 0, 2, 3};
    TrimeshShape mesh(vertices, 4, indices, 6);

    CharacterBody body;
    body.setShape(0.3f, 1.2f);
    body.setUpDirection(kinetix::Math::vec3(1.0f, 0.0f, 0.0f));
    body.setPosition(kinetix::Math::vec3(1.2f, 0.0f, 0.0f));
    for (int i = 0; i < 60; ++i)
        body.update(kStep, mesh, kinetix::Math::mat4(1.0f));
    CHECK(body.isOnFloor());
    CHECK_NEAR(body.position().x, 0.9f, 0.021f);

    const kinetix::Math::mat4 tilt =
        kinetix::Math::rotate(kinetix::Math::mat4(1.0f), 0.4f, kinetix::Math::vec3(0, 1, 1));
    const kinetix::Math::vec3 up = kinetix::Math::normalize(kinetix::Math::vec3(tilt * kinetix::Math::vec4(1, 0, 0, 0)));
    CharacterBody tilted;
    tilted.setShape(0.3f, 1.2f);
    tilted.setUpDirection(up);
    tilted.setPosition(up * 1.2f);
    for (int i = 0; i < 60; ++i)
        tilted.update(kStep, mesh, tilt);
    CHECK(tilted.isOnFloor());
    CHECK_NEAR(kinetix::Math::dot(tilted.position(), up), 0.9f, 0.021f);
}

// A frame's travel at 1 m/s is less than the skin width.
void wallContactPersists()
{
    Room room;
    for (f32 along : {0.0f, 1.0f})
    {
        CharacterBody body;
        body.setShape(0.3f, 1.2f);
        body.setPosition(kinetix::Math::vec3(0.0f, 0.92f, 0.0f));
        for (int i = 0; i < 10; ++i)
            body.update(kStep, room.mesh, kinetix::Math::mat4(1.0f));
        body.setMoveInput(kinetix::Math::vec3(1.0f, 0.0f, along));

        int onWall = 0;
        int onFloor = 0;
        const int frames = 120;
        for (int i = 0; i < 60 + frames; ++i)
        {
            body.update(kStep, room.mesh, kinetix::Math::mat4(1.0f));
            if (i >= 60)
            {
                onWall += body.isOnWall() ? 1 : 0;
                onFloor += body.isOnFloor() ? 1 : 0;
            }
        }
        CHECK(onWall == frames);
        CHECK(onFloor == frames);
        CHECK(body.position().x <= 0.7f + 1.0e-4f);
        CHECK(body.position().x > 0.7f - 0.021f);
        CHECK_NEAR(body.velocity().x, 0.0f, 1.0e-3f);
    }
}

void wallMinSlideAngle()
{
    Room room;
    for (f32 angle : {0.0f, 10.0f, 75.0f})
    {
        CharacterBody body;
        body.setShape(0.3f, 1.2f);
        body.setPosition(kinetix::Math::vec3(0.6f, 0.92f, 0.0f));
        body.setWallMinSlideAngle(angle);
        for (int i = 0; i < 5; ++i)
            body.update(kStep, room.mesh, kinetix::Math::mat4(1.0f));

        // About 63 degrees off head-on.
        body.setMoveInput(kinetix::Math::vec3(1.0f, 0.0f, 2.0f));
        for (int i = 0; i < 10; ++i)
            body.update(kStep, room.mesh, kinetix::Math::mat4(1.0f));
        const f32 z0 = body.position().z;
        for (int i = 0; i < 30; ++i)
            body.update(kStep, room.mesh, kinetix::Math::mat4(1.0f));
        const f32 slid = body.position().z - z0;

        CHECK(body.isOnWall());
        if (angle < 63.0f)
        {
            CHECK_NEAR(slid, 1.0f, 0.05f);
            CHECK_NEAR(body.velocity().z, 2.0f, 0.05f);
        }
        else
        {
            CHECK_NEAR(slid, 0.0f, 1.0e-4f);
            CHECK_NEAR(body.velocity().z, 0.0f, 1.0e-4f);
        }
    }
}

void slideOnCeiling()
{
    const f32 c = cosf(kinetix::Math::radians(30.0f));
    const f32 sn = sinf(kinetix::Math::radians(30.0f));
    // A plane through (0, 2, 0) with normal (sin30, -cos30, 0).
    const kinetix::Math::vec3 tangent(c, sn, 0.0f);
    const kinetix::Math::vec3 point(0.0f, 2.0f, 0.0f);
    kinetix::Math::vec3 vertices[4] = {point - tangent * 10.0f + kinetix::Math::vec3(0, 0, -10),
                                       point - tangent * 10.0f + kinetix::Math::vec3(0, 0, 10),
                                       point + tangent * 10.0f + kinetix::Math::vec3(0, 0, 10),
                                       point + tangent * 10.0f + kinetix::Math::vec3(0, 0, -10)};
    u32 indices[6] = {0, 2, 1, 0, 3, 2};
    TrimeshShape mesh(vertices, 4, indices, 6);
    TrimeshSweepSource source(mesh, kinetix::Math::mat4(1.0f));
    CharacterSweepHit probe;
    CHECK(source.sweepEllipsoid(kinetix::Math::vec3(0.0f), kinetix::Math::mat3(1.0f),
                                kinetix::Math::vec3(0.3f, 0.9f, 0.3f), kinetix::Math::vec3(0, 3, 0), probe));
    CHECK(probe.normal.y < -0.8f && probe.normal.x > 0.4f);

    for (bool slide : {true, false})
    {
        CharacterBody body;
        body.setShape(0.3f, 1.2f);
        body.setSlideOnCeiling(slide);
        body.setPosition(kinetix::Math::vec3(0.0f, 0.0f, 0.0f));
        body.setVelocity(kinetix::Math::vec3(0.0f, 5.0f, 0.0f));
        bool ceiling = false;
        for (int i = 0; i < 30 && !ceiling; ++i)
        {
            body.moveAndSlide(kStep, source);
            ceiling = body.isOnCeiling();
            if (!ceiling)
                body.setVelocity(kinetix::Math::vec3(0.0f, 5.0f, 0.0f));
        }
        CHECK(ceiling);
        if (slide)
            CHECK(body.velocity().y > 0.5f);
        else
            CHECK(body.velocity().y <= 1.0e-4f);
        CHECK(body.velocity().x > 1.0f);
    }
}

void moveInputFollowsUpDirection()
{
    CharacterBody body;
    body.setUpDirection(kinetix::Math::vec3(1.0f, 0.0f, 0.0f));
    body.setMoveInput(kinetix::Math::vec3(3.0f, 4.0f, 2.0f));
    CHECK_NEAR(body.moveInput().x, 0.0f, 1.0e-6f);
    CHECK_NEAR(body.moveInput().y, 4.0f, 1.0e-6f);
    CHECK_NEAR(body.moveInput().z, 2.0f, 1.0e-6f);

    CharacterBody later;
    later.setMoveInput(kinetix::Math::vec3(3.0f, 0.0f, 2.0f));
    later.setUpDirection(kinetix::Math::vec3(1.0f, 0.0f, 0.0f));
    CHECK_NEAR(later.moveInput().x, 0.0f, 1.0e-6f);
    CHECK_NEAR(later.moveInput().z, 2.0f, 1.0e-6f);
}

void walkingStillWorks()
{
    Room room;
    CharacterBody body;
    body.setShape(0.3f, 1.2f);
    body.setPosition(kinetix::Math::vec3(-5.0f, 0.92f, 0.0f));
    for (int i = 0; i < 10; ++i)
        body.update(kStep, room.mesh, kinetix::Math::mat4(1.0f));
    body.setMoveInput(kinetix::Math::vec3(0.0f, 0.0f, 4.0f));
    int grounded = 0;
    for (int i = 0; i < 60; ++i)
    {
        body.update(kStep, room.mesh, kinetix::Math::mat4(1.0f));
        grounded += body.isOnFloor() ? 1 : 0;
    }
    CHECK(grounded == 60);
    CHECK_NEAR(body.position().z, 4.0f, 0.05f);
    CHECK_NEAR(body.position().y, 0.92f, 1.0e-3f);

    kinetix::Math::vec3 stepVertices[8] = {{-10, 0, -10}, {10, 0, -10}, {10, 0, 1}, {-10, 0, 1},
                                           {-10, 0.2f, 1}, {10, 0.2f, 1}, {10, 0.2f, 10}, {-10, 0.2f, 10}};
    u32 stepIndices[18] = {0, 2, 1, 0, 3, 2, 3, 5, 2, 3, 4, 5, 4, 6, 5, 4, 7, 6};
    TrimeshShape stairs(stepVertices, 8, stepIndices, 18);
    CharacterBody climber;
    climber.setShape(0.3f, 1.2f);
    climber.setPosition(kinetix::Math::vec3(0.0f, 0.92f, 0.0f));
    climber.setMoveInput(kinetix::Math::vec3(0.0f, 0.0f, 2.0f));
    for (int i = 0; i < 90; ++i)
        climber.update(kStep, stairs, kinetix::Math::mat4(1.0f));
    CHECK(climber.isOnFloor());
    CHECK(climber.position().z > 2.5f);
    CHECK_NEAR(climber.position().y, 1.1f, 0.025f);
}


// A staircase like a building's: risers of 0.25 m and treads of 0.30 m, climbed by the capsule a
// character body stands in (radius 0.3, height 1.1). The step edge touches the lower curve of the
// ellipsoid, so the climb has to start from a resting contact with that edge.
void climbsStaircase()
{
    const int steps = 6;
    const f32 rise = 0.25f;
    const f32 tread = 0.30f;
    kinetix::Math::vec3 vertices[4 + steps * 8];
    u32 indices[6 + steps * 12];
    int v = 0;
    int n = 0;
    auto quad = [&](kinetix::Math::vec3 a, kinetix::Math::vec3 b, kinetix::Math::vec3 c, kinetix::Math::vec3 d)
    {
        vertices[v] = a;
        vertices[v + 1] = b;
        vertices[v + 2] = c;
        vertices[v + 3] = d;
        const u32 o = static_cast<u32>(v);
        const u32 q[6] = {o, o + 2, o + 1, o, o + 3, o + 2};
        for (int i = 0; i < 6; ++i)
            indices[n + i] = q[i];
        v += 4;
        n += 6;
    };
    quad({-2, 0, -5}, {2, 0, -5}, {2, 0, 1}, {-2, 0, 1});
    for (int k = 0; k < steps; ++k)
    {
        const f32 z = 1.0f + tread * static_cast<f32>(k);
        const f32 y0 = rise * static_cast<f32>(k);
        const f32 y1 = y0 + rise;
        const f32 far = k + 1 == steps ? z + 5.0f : z + tread;
        quad({-2, y0, z}, {2, y0, z}, {2, y1, z}, {-2, y1, z});
        quad({-2, y1, z}, {2, y1, z}, {2, y1, far}, {-2, y1, far});
    }
    TrimeshShape stairs(vertices, static_cast<u32>(v), indices, static_cast<u32>(n));
    CharacterBody climber;
    climber.setShape(0.3f, 1.1f);
    climber.setPosition(kinetix::Math::vec3(0.0f, 0.86f, -1.0f));
    for (int i = 0; i < 10; ++i)
        climber.update(kStep, stairs, kinetix::Math::mat4(1.0f));
    climber.setMoveInput(kinetix::Math::vec3(0.0f, 0.0f, 4.0f));
    for (int i = 0; i < 120; ++i)
        climber.update(kStep, stairs, kinetix::Math::mat4(1.0f));
    CHECK(climber.position().z > 1.0f + tread * steps);
    CHECK(climber.position().y > rise * steps + 0.5f);

    // The same climb driven the way a game drives moveAndSlide: a small downward speed while on
    // the floor, gravity piling up while not (and pressed against a step edge it is not).
    TrimeshSweepSource source(stairs, kinetix::Math::mat4(1.0f));
    CharacterBody walker;
    walker.setShape(0.3f, 1.1f);
    walker.setPosition(kinetix::Math::vec3(0.0f, 0.86f, -1.0f));
    f32 vertical = 0.0f;
    for (int i = 0; i < 150; ++i)
    {
        vertical = walker.isOnFloor() ? -0.5f : vertical - 18.0f * kStep;
        walker.setVelocity(kinetix::Math::vec3(0.0f, vertical, i < 10 ? 0.0f : 4.0f));
        walker.moveAndSlide(kStep, source);
    }
    CHECK(walker.position().z > 1.0f + tread * steps);
    CHECK(walker.position().y > rise * steps + 0.5f);
}

}

int main()
{
    overlapPushOutIsInWorldUnits();
    transformedMeshMatchesBakedMesh();
    capsuleStandsAlongUpDirection();
    wallContactPersists();
    wallMinSlideAngle();
    slideOnCeiling();
    moveInputFollowsUpDirection();
    walkingStillWorks();
    climbsStaircase();
    return TEST_RESULT();
}
