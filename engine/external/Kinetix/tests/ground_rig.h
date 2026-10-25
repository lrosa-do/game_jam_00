#ifndef KINETIX_TESTS_GROUND_RIG_H
#define KINETIX_TESTS_GROUND_RIG_H

#include <collision/CollisionShape.h>
#include <dynamics/PhysicsWorld.h>
#include <dynamics/RigidBody.h>
#include <ground/Ground.h>

#include <ct/vector.hpp>

#include <math.h>
#include <stdio.h>

namespace ground_test
{

using namespace kinetix::Physics;
using kinetix::f32;
using kinetix::s32;
using kinetix::u32;
namespace KM = kinetix::Math;

constexpr f32 kPi = 3.14159265358979f;
constexpr f32 kDeg = kPi / 180.0f;
constexpr f32 kG = 9.81f;
// Vehicle bodies live in this collision group; the wheel rays skip it, so a
// trailer's rays never land on its tractor.
constexpr u32 kVehicleGroup = 1u << 4;

inline bool finiteVec(const KM::vec3& v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

inline KM::quat headingRotation(f32 heading)
{
    return KM::angleAxis(-heading, KM::vec3(0.0f, 1.0f, 0.0f));
}

template <typename HeightFn>
TrimeshShape* makeHeightfield(int cells, f32 cell, HeightFn height)
{
    ct::Vector<KM::vec3> vertices;
    ct::Vector<u32> indices;
    const f32 half = 0.5f * cells * cell;
    for (int j = 0; j <= cells; ++j)
        for (int i = 0; i <= cells; ++i)
        {
            const f32 x = -half + i * cell;
            const f32 z = -half + j * cell;
            vertices.push_back(KM::vec3(x, height(x, z), z));
        }
    for (int j = 0; j < cells; ++j)
        for (int i = 0; i < cells; ++i)
        {
            const u32 a = static_cast<u32>(j * (cells + 1) + i);
            const u32 b = a + 1;
            const u32 c = a + static_cast<u32>(cells + 1);
            const u32 d = c + 1;
            // Wound counter-clockwise seen from above (normals up).
            indices.push_back(a);
            indices.push_back(c);
            indices.push_back(b);
            indices.push_back(b);
            indices.push_back(c);
            indices.push_back(d);
        }
    return new TrimeshShape(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));
}

struct Scene
{
    PhysicsWorld world;
    ct::Vector<RigidBody*> statics;
    ct::Vector<CollisionShape*> shapes;

    explicit Scene(bool flatFloor = true)
    {
        world.setGravity(KM::vec3(0.0f, -kG, 0.0f));
        if (flatFloor)
            addStaticBox(KM::vec3(0.0f, -1.0f, 0.0f), KM::vec3(2000.0f, 1.0f, 2000.0f), KM::quat());
    }

    ~Scene()
    {
        for (RigidBody* b : statics)
            delete b;
        for (CollisionShape* s : shapes)
            delete s;
    }

    u32 addStatic(CollisionShape* shape, const KM::vec3& position, const KM::quat& orientation)
    {
        RigidBody* body = new RigidBody();
        body->setBodyType(BodyType::Static);
        body->setPosition(position);
        body->setOrientation(orientation);
        body->calculateDerivedData();
        BodyEntry entry;
        entry.body = body;
        entry.shape = shape;
        entry.friction = 0.8f;
        const u32 id = world.addBody(entry);
        world.markStaticBroadphaseDirty();
        statics.push_back(body);
        shapes.push_back(shape);
        return id;
    }

    u32 addStaticBox(const KM::vec3& centre, const KM::vec3& halfExtents, const KM::quat& orientation)
    {
        return addStatic(new BoxShape(halfExtents), centre, orientation);
    }
};

struct Rig
{
    RigidBody body;
    BoxShape shape{KM::vec3(1.0f)};
    GroundVehicle vehicle;
    GroundVehicleLayout layout;
    u32 id = 0xFFFFFFFFu;

    void place(Scene& scene, const KM::vec3& at, f32 heading = 0.0f)
    {
        body.setPosition(KM::vec3(at.x, at.y + layout.restHeight, at.z));
        body.setOrientation(headingRotation(heading));
        body.setVelocity(KM::vec3(0.0f));
        body.setAngularVelocity(KM::vec3(0.0f));
        body.calculateDerivedData();
        addToWorld(scene);
    }

    void addToWorld(Scene& scene)
    {
        shape = BoxShape(layout.collisionHalfExtents);
        BodyEntry entry;
        entry.body = &body;
        entry.shape = &shape;
        entry.friction = 0.5f;
        entry.filter.group = kVehicleGroup;
        id = scene.world.addBody(entry);
        vehicle.setBody(&body, id);
        vehicle.setWorld(&scene.world);
        QueryFilter rays;
        rays.collision.group = 0xFFFFFFFFu;
        rays.collision.mask = ~kVehicleGroup;
        vehicle.setRayFilter(rays);
    }

    void launch(f32 speed)
    {
        body.setVelocity(body.directionToWorld(KM::vec3(0.0f, 0.0f, speed)));
        for (u32 i = 0; i < vehicle.wheelCount(); ++i)
            vehicle.setWheelAngularVelocity(i, speed / vehicle.axle(i / 2).wheel.radius);
    }

    f32 lowestUp() const
    {
        return body.directionToWorld(KM::vec3(0.0f, 1.0f, 0.0f)).y;
    }
};

inline void stepAll(Scene& scene, Rig* const* rigs, u32 count, f32 dt)
{
    for (u32 i = 0; i < count; ++i)
        rigs[i]->vehicle.applyForces(dt);
    scene.world.step(dt);
}

inline void step(Scene& scene, Rig& rig, f32 dt)
{
    Rig* rigs[1] = {&rig};
    stepAll(scene, rigs, 1, dt);
}

inline f32 horizontalDistance(const KM::vec3& a, const KM::vec3& b)
{
    const f32 dx = a.x - b.x;
    const f32 dz = a.z - b.z;
    return sqrtf(dx * dx + dz * dz);
}

// Least-squares circle through points in the XZ plane (Kasa fit): centre
// and radius.
inline void fitCircle(const ct::Vector<KM::vec3>& points, f32& cx, f32& cz, f32& radius)
{
    double sx = 0, sz = 0, sxx = 0, szz = 0, sxz = 0, sxxx = 0, szzz = 0, sxzz = 0, sxxz = 0;
    const double n = static_cast<double>(points.size());
    for (const KM::vec3& p : points)
    {
        const double x = p.x, z = p.z;
        sx += x;
        sz += z;
        sxx += x * x;
        szz += z * z;
        sxz += x * z;
        sxxx += x * x * x;
        szzz += z * z * z;
        sxzz += x * z * z;
        sxxz += x * x * z;
    }
    // Solve [sxx sxz sx; sxz szz sz; sx sz n] [a b c]^T = -[sxxx+sxzz; sxxz+szzz; sxx+szz]
    // for x^2 + z^2 + a x + b z + c = 0.
    double m[3][4] = {{sxx, sxz, sx, -(sxxx + sxzz)}, {sxz, szz, sz, -(sxxz + szzz)}, {sx, sz, n, -(sxx + szz)}};
    for (int col = 0; col < 3; ++col)
    {
        int pivot = col;
        for (int r = col + 1; r < 3; ++r)
            if (fabs(m[r][col]) > fabs(m[pivot][col]))
                pivot = r;
        for (int k = 0; k < 4; ++k)
        {
            const double t = m[col][k];
            m[col][k] = m[pivot][k];
            m[pivot][k] = t;
        }
        for (int r = 0; r < 3; ++r)
        {
            if (r == col)
                continue;
            const double f = m[r][col] / m[col][col];
            for (int k = 0; k < 4; ++k)
                m[r][k] -= f * m[col][k];
        }
    }
    const double a = m[0][3] / m[0][0];
    const double b = m[1][3] / m[1][1];
    const double c = m[2][3] / m[2][2];
    cx = static_cast<f32>(-0.5 * a);
    cz = static_cast<f32>(-0.5 * b);
    radius = static_cast<f32>(sqrt(fmax(0.25 * (a * a + b * b) - c, 0.0)));
}

inline f32 bumpyHeight(f32 x, f32 z)
{
    return 0.05f * sinf(0.9f * x + 0.3f) * sinf(0.7f * z) + 0.03f * sinf(2.1f * x + 1.0f) * cosf(1.7f * z + 0.5f);
}

struct StabilityRun
{
    f32 lowestUp = 1.0f;
    f32 fastest = 0.0f;
    f32 farthest = 0.0f;
    f32 worstSeparation = 0.0f;
    f32 fastestWheel = 0.0f;
    f32 stoppedSpeed = 1.0e9f;
    bool finite = true;
};

inline void stabilityDrive(Scene& scene, Rig* const* rigs, u32 count, TrailerCoupling* coupling, f32 rate,
                           f32 cruise, f32 steer, s32 gear, StabilityRun& out)
{
    const f32 dt = 1.0f / rate;
    const int steps = static_cast<int>(60.0f * rate + 0.5f);
    Rig& lead = *rigs[0];
    const KM::vec3 start = lead.body.position();
    for (int i = 0; i < steps; ++i)
    {
        const f32 t = i * dt;
        VehicleControls c;
        if (t >= 2.0f && i == static_cast<int>(2.0f * rate))
            lead.vehicle.selectGear(gear);
        const f32 v = lead.vehicle.telemetry().forwardSpeed;
        if (t >= 2.0f && t < 40.0f)
        {
            c.throttle = kinetix::Clamp(0.3f + 0.4f * (cruise - v), 0.0f, 1.0f);
            c.steering = steer * (0.85f + 0.15f * sinf(0.4f * t));
        }
        else if (t >= 40.0f && t < 50.0f)
        {
            c.brake = 1.0f;
            c.steering = 0.0f;
        }
        else if (t >= 50.0f)
        {
            c.throttle = kinetix::Clamp(0.3f + 0.4f * (0.5f * cruise - v), 0.0f, 1.0f);
            c.steering = -steer;
        }
        lead.vehicle.setControls(c);
        stepAll(scene, rigs, count, dt);
        for (u32 r = 0; r < count; ++r)
        {
            const Rig& rig = *rigs[r];
            const VehicleTelemetry& tel = rig.vehicle.telemetry();
            out.finite = out.finite && finiteVec(rig.body.position()) && finiteVec(rig.body.velocity()) &&
                         finiteVec(rig.body.angularVelocity()) && isfinite(tel.engineRpm) && isfinite(tel.forwardSpeed);
            out.lowestUp = fminf(out.lowestUp, rig.lowestUp());
            out.fastest = fmaxf(out.fastest, KM::length(rig.body.velocity()));
            for (u32 w = 0; w < rig.vehicle.wheelCount(); ++w)
            {
                const WheelTelemetry& wt = rig.vehicle.wheel(w);
                out.finite = out.finite && isfinite(wt.angularVelocity) && isfinite(wt.load) &&
                             isfinite(wt.longitudinalForce) && isfinite(wt.lateralForce);
                out.fastestWheel = fmaxf(out.fastestWheel, fabsf(wt.angularVelocity));
            }
        }
        out.farthest = fmaxf(out.farthest, horizontalDistance(lead.body.position(), start));
        if (coupling)
            out.worstSeparation = fmaxf(out.worstSeparation, coupling->separation());
        if (t > 48.0f && t < 50.0f)
            out.stoppedSpeed = fminf(out.stoppedSpeed, fabsf(lead.vehicle.telemetry().forwardSpeed));
        if (!out.finite)
            return;
    }
}

inline void reportStability(const char* name, f32 rate, bool bumpy, const StabilityRun& r)
{
    printf("%s, %s, %.0f Hz: finite %d, lowest up %.3f, fastest %.1f m/s, fastest wheel %.1f rad/s, "
           "farthest %.0f m, worst coupling separation %.2f mm, speed when braked %.3f m/s\n",
           name, bumpy ? "bumpy trimesh" : "flat", double(rate), r.finite, double(r.lowestUp), double(r.fastest),
           double(r.fastestWheel), double(r.farthest), double(1000.0f * r.worstSeparation), double(r.stoppedSpeed));
}

#ifdef KINETIX_TESTS_CHECK_H
// Checks are made here (check.h must be included first).
inline void truckStabilityMinute(bool bumpy, f32 rate)
{
    Scene scene(!bumpy);
    if (bumpy)
        scene.addStatic(makeHeightfield(200, 1.0f, bumpyHeight), KM::vec3(0.0f), KM::quat());
    Rig truck;
    Rig trailer;
    truck.layout = buildTruck6x4(truck.vehicle, truck.body);
    trailer.layout = buildSemiTrailer(trailer.vehicle, trailer.body);
    truck.place(scene, KM::vec3(0.0f, bumpy ? bumpyHeight(0.0f, 0.0f) : 0.0f, 20.0f));
    CouplingParameters cp;
    cp.type = CouplingType::FifthWheel;
    cp.towPoint = truck.layout.towPoint;
    cp.trailerPoint = trailer.layout.couplingPoint;
    TrailerCoupling::placeTrailer(truck.body, trailer.body, cp);
    trailer.addToWorld(scene);
    TrailerCoupling coupling;
    CHECK(coupling.attach(scene.world, truck.vehicle, trailer.vehicle, cp));
    Rig* rigs[2] = {&truck, &trailer};
    StabilityRun r;
    stabilityDrive(scene, rigs, 2, &coupling, rate, 7.0f, 0.45f, 1, r);
    reportStability("truck and semi-trailer", rate, bumpy, r);
    CHECK(r.finite);
    CHECK(r.lowestUp > 0.9f);
    CHECK(r.fastest < 12.0f);
    CHECK(r.fastestWheel < 40.0f);
    CHECK(r.farthest < 80.0f);
    CHECK(r.worstSeparation < 0.01f);
    CHECK(r.stoppedSpeed < 0.05f);
    CHECK(fabsf(coupling.articulationAngle()) < 60.0f * kDeg);
}
#endif

}

#endif // KINETIX_TESTS_GROUND_RIG_H
