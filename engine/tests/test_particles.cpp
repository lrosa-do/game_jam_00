#include "Check.h"

#include "prisma3d/core/Random.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/ParticleCollision3D.h"
#include "prisma3d/scene/ParticleSystem3D.h"
#include "prisma3d/scene/SceneTree.h"

#include <math.h>

using namespace prisma3d;

namespace
{

const float kTolerance = 1e-4f;
const float kPi = 3.14159265358979323846f;

bool near(float a, float b, float tolerance = kTolerance)
{
    return fabsf(a - b) <= tolerance;
}

bool nearVec3(const Math::Vec3& a, float x, float y, float z, float tolerance = kTolerance)
{
    return near(a.x, x, tolerance) && near(a.y, y, tolerance) && near(a.z, z, tolerance);
}

bool nearVec4(const Math::Vec4& a, float x, float y, float z, float w)
{
    return near(a.x, x) && near(a.y, y) && near(a.z, z) && near(a.w, w);
}

struct Scene
{
    SceneTree tree;
    ParticleSystem3D* particles;

    explicit Scene(int maxParticles = 1000)
            : tree(),
              particles(nullptr)
    {
        tree.initialize();
        particles = tree.getRoot()->createChild<ParticleSystem3D>(maxParticles);
        particles->setLifetime(100.0f)->setSpeed(0.0f)->setSize(1.0f);
    }
};

void testRandom()
{
    Random a(42);
    Random b(42);
    CHECK(a.getSeed() == 42);
    for (int i = 0; i < 64; ++i) CHECK(a.rand() == b.rand());

    Random c(42);
    CHECK(c.rand() == 492690617u);
    CHECK(c.rand() == 1919685028u);
    CHECK(c.rand() == 3561993920u);
    CHECK(c.rand() == 683038915u);

    Random d;
    CHECK(d.getSeed() == Random::kDefaultSeed);
    CHECK(d.rand() == 3161026589u);
    CHECK(d.rand() == 2668139190u);

    c.seed(42);
    CHECK(c.rand() == 492690617u);

    Random e(42);
    CHECK(near(e.randf(), 0.1183701902627945f, 1e-7f));
    CHECK(near(e.randf(), 0.65903240442276f, 1e-7f));
    CHECK(near(e.randf(), 0.2981868088245392f, 1e-7f));

    Random f(42);
    CHECK(f.rand(6) == 5u);

    Random g(7);
    for (int i = 0; i < 10000; ++i)
    {
        float v = g.randf();
        CHECK(v >= 0.0f && v <= 1.0f);
        float r = g.randfRange(-3.0f, 5.0f);
        CHECK(r >= -3.0f && r <= 5.0f);
        double dv = g.randd();
        CHECK(dv >= 0.0 && dv <= 1.0);
        int n = g.randiRange(-4, 9);
        CHECK(n >= -4 && n <= 9);
        int m = g.randiRange(9, -4);
        CHECK(m >= -4 && m <= 9);
        CHECK(g.rand(10) < 10u);
    }
    CHECK(g.randiRange(5, 5) == 5);

    bool seen[4] = { false, false, false, false };
    for (int i = 0; i < 1000; ++i) seen[g.randiRange(0, 3)] = true;
    CHECK(seen[0] && seen[1] && seen[2] && seen[3]);

    Random h(42);
    CHECK(near(h.randfRange(10.0f, 20.0f), 10.0f + 0.1183701902627945f * 10.0f, 1e-5f));

    Random state(5);
    std::uint64_t saved = state.getState();
    std::uint32_t first = state.rand();
    state.setState(saved);
    CHECK(state.rand() == first);

    Random r1;
    r1.randomize();
    CHECK(r1.getSeed() != Random::kDefaultSeed);
}

void testDefaults()
{
    SceneTree tree;
    tree.initialize();
    ParticleSystem3D* p = tree.getRoot()->createChild<ParticleSystem3D>();

    CHECK(p->castTo<ParticleSystem3D>() == p);
    CHECK(p->castTo<VisualInstance3D>() == p);
    CHECK(p->castTo<MeshInstance3D>() == nullptr);
    CHECK(!p->getCastShadows());
    CHECK(p->getMaxParticles() == 1000);
    CHECK(p->getParticles().size() == 1000);
    CHECK(p->getActiveCount() == 0);
    CHECK(p->getBlendMode() == ParticleBlendMode::Additive);
    CHECK(p->getEmissionMode() == EmissionMode::Continuous);
    CHECK(near(p->getEmissionRate(), 10.0f));
    CHECK(p->getBurstCount() == 10);
    CHECK(near(p->getBurstInterval(), 1.0f));
    CHECK(p->getOneShotCount() == 10);
    CHECK(near(p->getPulseRate(), 1.0f));
    CHECK(p->getParticlesPerPulse() == 5);
    CHECK(p->getEmitterShape() == EmitterShape::Point);
    CHECK(near(p->getShapeRadius(), 1.0f));
    CHECK(near(p->getShapeInnerRadius(), 0.5f));
    CHECK(near(p->getShapeConeAngleDeg(), 0.785f * (180.0f / 3.14159265f)));
    CHECK(nearVec3(p->getShapeBoxSize(), 1.0f, 1.0f, 1.0f));
    CHECK(nearVec3(p->getEmissionOffset(), 0.0f, 0.0f, 0.0f));
    CHECK(nearVec3(p->getEmissionDirection(), 0.0f, 1.0f, 0.0f));
    CHECK(near(p->getSpreadAngleDeg(), 0.0f));
    CHECK(near(p->getLifetimeMin(), 1.0f) && near(p->getLifetimeMax(), 3.0f));
    CHECK(near(p->getSpeedMin(), 1.0f) && near(p->getSpeedMax(), 5.0f));
    CHECK(near(p->getSizeStart().x, 0.5f) && near(p->getSizeEnd().y, 0.1f));
    CHECK(nearVec4(p->getColorStart(), 1.0f, 1.0f, 1.0f, 1.0f));
    CHECK(nearVec4(p->getColorEnd(), 1.0f, 1.0f, 1.0f, 0.0f));
    CHECK(near(p->getRotSpeedMin(), 0.0f) && near(p->getRotSpeedMax(), 0.0f));
    CHECK(nearVec3(p->getGravity(), 0.0f, 0.0f, 0.0f));
    CHECK(near(p->getDrag(), 0.0f));
    CHECK(near(p->getDuration(), -1.0f));
    CHECK(p->getLoop());
    CHECK(!p->isUsingAtlas());
    CHECK(p->getAtlasCols() == 1 && p->getAtlasRows() == 1);
    CHECK(p->getAffectorCount() == 0);
    CHECK(!p->getTexture().valid());
    CHECK(p->getGeometry().positions.empty());

    const Particle fresh;
    CHECK(!fresh.active);
    CHECK(near(fresh.lifetime, 1.0f) && near(fresh.timeAlive, 0.0f));
    CHECK(nearVec4(fresh.texRect, 0.0f, 0.0f, 1.0f, 1.0f));
    CHECK(nearVec4(fresh.colorEnd, 1.0f, 1.0f, 1.0f, 0.0f));
    CHECK(near(fresh.sizeEnd.x, 0.5f));

    p->setBlendMode(ParticleBlendMode::Screen);
    CHECK(p->getBlendMode() == ParticleBlendMode::Screen);

    ParticleSystem3D* chain = p->setShapeSphere(2.0f)->setLifetime(0.5f, 0.75f)->setSpeed(3.0f);
    CHECK(chain == p);
    CHECK(near(p->getShapeRadius(), 2.0f));
    CHECK(p->getEmitterShape() == EmitterShape::Sphere);
    CHECK(near(p->getLifetimeMin(), 0.5f) && near(p->getLifetimeMax(), 0.75f));
    CHECK(near(p->getSpeedMin(), 3.0f) && near(p->getSpeedMax(), 3.0f));
    p->setShapeCone(45.0f, 2.0f)->setSpreadAngle(30.0f);
    CHECK(near(p->getShapeConeAngleDeg(), 45.0f, 1e-3f));
    CHECK(near(p->getSpreadAngleDeg(), 30.0f, 1e-3f));
    p->setShapeRing(5.0f, 2.0f);
    CHECK(near(p->getShapeRadius(), 5.0f) && near(p->getShapeInnerRadius(), 2.0f));
    p->setEmissionDirection(Math::Vec3(0.0f, 4.0f, 0.0f));
    CHECK(nearVec3(p->getEmissionDirection(), 0.0f, 1.0f, 0.0f));
}

void testEmissionModes()
{
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        CHECK(p->getState() == EmitterState::Playing);
        p->setContinuous(4.0f);
        p->update(1.0);
        CHECK(p->getActiveCount() == 4);
        p->update(0.5);
        CHECK(p->getActiveCount() == 6);
        p->update(0.0);
        CHECK(p->getActiveCount() == 6);
    }
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setContinuous(1000.0f);
        p->update(1.0);
        CHECK(p->getActiveCount() == 101);
    }
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setBurst(5, 1.0f);
        p->update(0.5);
        CHECK(p->getActiveCount() == 0);
        p->update(0.5);
        CHECK(p->getActiveCount() == 5);
        p->update(0.5);
        CHECK(p->getActiveCount() == 5);
        p->update(0.5);
        CHECK(p->getActiveCount() == 10);
        CHECK(p->getState() == EmitterState::Playing);
    }
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setBurst(5, 1.0f)->setLoop(false);
        p->update(1.0);
        CHECK(p->getActiveCount() == 0);
        CHECK(p->getState() == EmitterState::Stopped);
    }
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setOneShot(7);
        p->play();
        p->update(0.0);
        CHECK(p->getActiveCount() == 7);
        p->play();
        p->update(0.0);
        CHECK(p->getActiveCount() == 7);
        p->update(2.0);
        CHECK(p->getActiveCount() == 7);

        p->setDuration(1.0f);
        p->update(1.0);
        p->play();
        p->update(0.0);
        CHECK(p->getActiveCount() == 14);
    }
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setPulse(2.0f, 3);
        p->update(0.5);
        CHECK(p->getActiveCount() == 3);
        p->update(0.25);
        CHECK(p->getActiveCount() == 3);
        p->update(0.25);
        CHECK(p->getActiveCount() == 6);
    }
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setEmissionMode(EmissionMode::Continuous);
        p->emitBurst(9);
        p->update(0.0);
        CHECK(p->getActiveCount() == 9);
    }
}

void testMaxParticlesAndExpiry()
{
    {
        Scene scene(5);
        ParticleSystem3D* p = scene.particles;
        p->setLifetime(1.0f);
        p->emitBurst(10);
        p->update(0.0);
        CHECK(p->getActiveCount() == 5);
        p->setContinuous(100.0f);
        p->update(0.1);
        CHECK(p->getActiveCount() == 5);

        p->setEmissionMode(EmissionMode::OneShot);
        p->update(0.85);
        CHECK(p->getActiveCount() == 5);
        float before = p->getParticles()[0].timeAlive;
        CHECK(before > 0.9f);
        p->emitBurst(2);
        int fresh = 0;
        for (std::size_t i = 0; i < p->getParticles().size(); ++i)
            if (p->getParticles()[i].timeAlive == 0.0f) ++fresh;
        CHECK(fresh == 2);
        p->update(0.0);
        CHECK(p->getActiveCount() == 5);
    }
    {
        Scene scene(5);
        ParticleSystem3D* p = scene.particles;
        p->setLifetime(100.0f);
        p->emitBurst(5);
        p->update(1.0);
        p->emitBurst(3);
        int fresh = 0;
        for (std::size_t i = 0; i < p->getParticles().size(); ++i)
            if (p->getParticles()[i].timeAlive == 0.0f) ++fresh;
        CHECK(fresh == 0);
        CHECK(p->getParticles().size() == 5);
    }
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setOneShot(3)->setLifetime(1.0f);
        p->play();
        p->update(0.5);
        CHECK(p->getActiveCount() == 3);
        p->update(0.5);
        CHECK(p->getActiveCount() == 0);
        CHECK(p->getGeometry().positions.empty());
    }
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setLifetime(0.5f, 1.5f);
        p->emitBurst(300);
        for (std::size_t i = 0; i < p->getParticles().size(); ++i)
        {
            const Particle& particle = p->getParticles()[i];
            if (!particle.active) continue;
            CHECK(particle.lifetime >= 0.5f && particle.lifetime <= 1.5f);
            CHECK(particle.rotation >= 0.0f && particle.rotation <= 2.0f * kPi + 1e-4f);
        }
        p->update(2.0);
        CHECK(p->getActiveCount() == 0);
    }
}

void testShapes()
{
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setShapePoint()->setEmissionOffset(Math::Vec3(1.0f, 2.0f, 3.0f));
        p->emitBurst(50);
        for (int i = 0; i < 50; ++i)
            CHECK(nearVec3(p->getParticles()[static_cast<std::size_t>(i)].position, 1.0f, 2.0f,
                    3.0f));
    }
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setShapeSphere(2.0f);
        p->emitBurst(500);
        int outer = 0;
        for (int i = 0; i < 500; ++i)
        {
            float length = p->getParticles()[static_cast<std::size_t>(i)].position.Length();
            CHECK(length <= 2.0f + 1e-4f);
            if (length > 1.0f) ++outer;
        }
        CHECK(outer > 250);
    }
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setShapeBox(Math::Vec3(2.0f, 4.0f, 6.0f));
        p->emitBurst(500);
        bool wide = false;
        for (int i = 0; i < 500; ++i)
        {
            const Math::Vec3& v = p->getParticles()[static_cast<std::size_t>(i)].position;
            CHECK(fabsf(v.x) <= 1.0f && fabsf(v.y) <= 2.0f && fabsf(v.z) <= 3.0f);
            if (fabsf(v.z) > 2.0f) wide = true;
        }
        CHECK(wide);
    }
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setShapeCircle(3.0f);
        p->emitBurst(500);
        bool far = false;
        for (int i = 0; i < 500; ++i)
        {
            const Math::Vec3& v = p->getParticles()[static_cast<std::size_t>(i)].position;
            CHECK(near(v.z, 0.0f));
            float radius = sqrtf(v.x * v.x + v.y * v.y);
            CHECK(radius <= 3.0f + 1e-4f);
            if (radius > 2.0f) far = true;
        }
        CHECK(far);
    }
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setShapeCone(30.0f, 2.0f);
        p->emitBurst(300);
        for (int i = 0; i < 300; ++i)
        {
            const Math::Vec3& v = p->getParticles()[static_cast<std::size_t>(i)].position;
            CHECK(near(v.z, 0.0f));
            CHECK(sqrtf(v.x * v.x + v.y * v.y) <= 2.0f + 1e-4f);
        }
        Scene flat;
        flat.particles->setShapeCone(30.0f);
        flat.particles->emitBurst(20);
        for (int i = 0; i < 20; ++i)
            CHECK(nearVec3(flat.particles->getParticles()[static_cast<std::size_t>(i)].position,
                    0.0f, 0.0f, 0.0f));
    }
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setShapeRing(5.0f, 2.0f);
        p->emitBurst(500);
        for (int i = 0; i < 500; ++i)
        {
            const Math::Vec3& v = p->getParticles()[static_cast<std::size_t>(i)].position;
            CHECK(near(v.z, 0.0f));
            float radius = sqrtf(v.x * v.x + v.y * v.y);
            CHECK(radius >= 2.0f - 1e-4f && radius <= 5.0f + 1e-4f);
        }
    }
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setPosition(Math::Vec3(10.0f, 0.0f, 0.0f));
        p->setShapePoint()->setEmissionOffset(Math::Vec3(0.0f, 1.0f, 0.0f));
        p->emitBurst(5);
        for (int i = 0; i < 5; ++i)
            CHECK(nearVec3(p->getParticles()[static_cast<std::size_t>(i)].position, 10.0f, 1.0f,
                    0.0f));
    }
}

void testVelocity()
{
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setSpeed(2.0f)->setEmissionDirection(Math::Vec3(0.0f, 1.0f, 0.0f));
        p->emitBurst(10);
        for (int i = 0; i < 10; ++i)
            CHECK(nearVec3(p->getParticles()[static_cast<std::size_t>(i)].velocity, 0.0f, 2.0f,
                    0.0f));
    }
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setSpeed(1.5f, 3.0f)->setSpreadAngle(30.0f);
        p->emitBurst(500);
        float widest = 0.0f;
        for (int i = 0; i < 500; ++i)
        {
            const Math::Vec3& v = p->getParticles()[static_cast<std::size_t>(i)].velocity;
            float speed = v.Length();
            CHECK(speed >= 1.5f - 1e-4f && speed <= 3.0f + 1e-4f);
            float angle = acosf(v.y / speed);
            CHECK(angle <= 30.0f * kPi / 180.0f + 1e-3f);
            if (angle > widest) widest = angle;
        }
        CHECK(widest > 20.0f * kPi / 180.0f);
    }
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setSpeed(2.0f)->setSpreadAngle(10.0f)->setEmissionDirection(
                Math::Vec3(0.0f, 0.0f, 1.0f));
        p->emitBurst(200);
        for (int i = 0; i < 200; ++i)
        {
            const Math::Vec3& v = p->getParticles()[static_cast<std::size_t>(i)].velocity;
            CHECK(near(v.Length(), 2.0f, 1e-3f));
            CHECK(v.z / 2.0f >= cosf(10.0f * kPi / 180.0f) - 1e-3f);
        }
    }
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setRotationDegrees(Math::Vec3(0.0f, 0.0f, 90.0f));
        p->setSpeed(2.0f)->setEmissionDirection(Math::Vec3(0.0f, 1.0f, 0.0f));
        p->emitBurst(3);
        for (int i = 0; i < 3; ++i)
            CHECK(nearVec3(p->getParticles()[static_cast<std::size_t>(i)].velocity, -2.0f, 0.0f,
                    0.0f, 1e-3f));
    }
}

void testSimulation()
{
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setOneShot(1)->setGravity(Math::Vec3(0.0f, -10.0f, 0.0f));
        p->play();
        p->update(0.1);
        const Particle& particle = p->getParticles()[0];
        CHECK(nearVec3(particle.velocity, 0.0f, -1.0f, 0.0f));
        CHECK(nearVec3(particle.position, 0.0f, -0.1f, 0.0f));
        CHECK(nearVec3(particle.acceleration, 0.0f, -10.0f, 0.0f));
        CHECK(near(particle.timeAlive, 0.1f));
    }
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setOneShot(1)->setDrag(0.5f)->setSpeed(2.0f);
        p->play();
        p->update(0.1);
        CHECK(nearVec3(p->getParticles()[0].velocity, 0.0f, 1.9f, 0.0f));
        CHECK(nearVec3(p->getParticles()[0].position, 0.0f, 0.19f, 0.0f));
    }
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setOneShot(1)->addGravity(Math::Vec3(0.0f, -10.0f, 0.0f));
        p->play();
        p->update(0.1);
        CHECK(nearVec3(p->getParticles()[0].position, 0.0f, 0.0f, 0.0f));
        CHECK(nearVec3(p->getParticles()[0].acceleration, 0.0f, -10.0f, 0.0f));
        p->update(0.1);
        CHECK(nearVec3(p->getParticles()[0].velocity, 0.0f, -1.0f, 0.0f));
        CHECK(nearVec3(p->getParticles()[0].position, 0.0f, -0.1f, 0.0f));
    }
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setOneShot(1)->setRotationSpeed(2.0f, 2.0f);
        p->play();
        float before = p->getParticles()[0].rotation;
        CHECK(near(p->getParticles()[0].rotationSpeed, 2.0f));
        p->update(0.1);
        CHECK(near(p->getParticles()[0].rotation, before + 0.2f));
    }
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setOneShot(1)->setLifetime(4.0f);
        p->setColor(Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f), Math::Vec4(0.0f, 0.0f, 1.0f, 0.0f));
        p->setSize(Math::Vec2(1.0f, 1.0f), Math::Vec2(3.0f, 5.0f));
        p->play();
        CHECK(nearVec4(p->getParticles()[0].color, 1.0f, 0.0f, 0.0f, 1.0f));
        CHECK(near(p->getParticles()[0].size.x, 1.0f));
        p->update(1.0);
        CHECK(nearVec4(p->getParticles()[0].color, 0.75f, 0.0f, 0.25f, 0.75f));
        CHECK(near(p->getParticles()[0].size.x, 1.5f) && near(p->getParticles()[0].size.y, 2.0f));
        p->update(2.0);
        CHECK(nearVec4(p->getParticles()[0].color, 0.25f, 0.0f, 0.75f, 0.25f));
        CHECK(near(p->getParticles()[0].size.x, 2.5f) && near(p->getParticles()[0].size.y, 4.0f));
    }
}

void testAffectors()
{
    {
        GravityAffector affector(Math::Vec3(0.0f, -10.0f, 0.0f));
        Particle p;
        p.acceleration = Math::Vec3(1.0f, 2.0f, 3.0f);
        affector.apply(p, 0.1f);
        CHECK(nearVec3(p.acceleration, 1.0f, -8.0f, 3.0f));
        affector.enabled = false;
        affector.apply(p, 0.1f);
        CHECK(nearVec3(p.acceleration, 1.0f, -8.0f, 3.0f));
    }
    {
        DragAffector affector(0.5f);
        Particle p;
        p.velocity = Math::Vec3(2.0f, 4.0f, 0.0f);
        affector.apply(p, 0.1f);
        CHECK(nearVec3(p.velocity, 1.9f, 3.8f, 0.0f));
        affector.enabled = false;
        affector.apply(p, 0.1f);
        CHECK(nearVec3(p.velocity, 1.9f, 3.8f, 0.0f));
    }
    {
        VortexAffector affector(Math::Vec3(0.0f, 0.0f, 0.0f), 2.0f, 4.0f);
        Particle p;
        p.position = Math::Vec3(2.0f, 0.0f, 0.0f);
        affector.apply(p, 0.1f);
        CHECK(nearVec3(p.acceleration, 0.0f, 0.0f, -1.0f));
        Particle outside;
        outside.position = Math::Vec3(5.0f, 0.0f, 0.0f);
        affector.apply(outside, 0.1f);
        CHECK(nearVec3(outside.acceleration, 0.0f, 0.0f, 0.0f));
        Particle onCenter;
        affector.apply(onCenter, 0.1f);
        CHECK(nearVec3(onCenter.acceleration, 0.0f, 0.0f, 0.0f));
        Particle high;
        high.position = Math::Vec3(0.0f, 3.0f, 2.0f);
        VortexAffector axis(Math::Vec3(0.0f, 3.0f, 0.0f), 2.0f, 4.0f);
        axis.apply(high, 0.1f);
        CHECK(nearVec3(high.acceleration, 1.0f, 0.0f, 0.0f));
    }
    {
        AttractorAffector pull(Math::Vec3(0.0f, 0.0f, 0.0f), 4.0f, 4.0f);
        Particle p;
        p.position = Math::Vec3(2.0f, 0.0f, 0.0f);
        pull.apply(p, 0.1f);
        CHECK(nearVec3(p.acceleration, -2.0f, 0.0f, 0.0f));

        AttractorAffector push(Math::Vec3(0.0f, 0.0f, 0.0f), 4.0f, 4.0f, true);
        Particle q;
        q.position = Math::Vec3(2.0f, 0.0f, 0.0f);
        push.apply(q, 0.1f);
        CHECK(nearVec3(q.acceleration, 2.0f, 0.0f, 0.0f));

        Particle far;
        far.position = Math::Vec3(9.0f, 0.0f, 0.0f);
        pull.apply(far, 0.1f);
        CHECK(nearVec3(far.acceleration, 0.0f, 0.0f, 0.0f));
    }
    {
        TurbulenceAffector affector(2.0f, 1.0f);
        Particle p;
        affector.apply(p, 0.5f);
        CHECK(near(affector.time, 0.5f));
        CHECK(nearVec3(p.acceleration, sinf(0.5f) * 2.0f, sinf(0.5f * 1.3f) * 2.0f,
                cosf(0.5f * 0.7f) * 2.0f));
        Particle moved;
        moved.position = Math::Vec3(1.0f, 2.0f, 3.0f);
        affector.apply(moved, 0.5f);
        CHECK(near(affector.time, 1.0f));
        CHECK(nearVec3(moved.acceleration, sinf(1.0f + 1.0f) * 2.0f, sinf(2.0f + 1.3f) * 2.0f,
                cosf(3.0f + 0.7f) * 2.0f));
    }
    {
        ColorOverLifetimeAffector affector(Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f),
                Math::Vec4(0.0f, 0.0f, 1.0f, 0.0f));
        Particle p;
        p.timeAlive = 1.0f;
        p.lifetime = 4.0f;
        affector.apply(p, 0.1f);
        CHECK(nearVec4(p.color, 0.75f, 0.0f, 0.25f, 0.75f));
    }
    {
        SizeOverLifetimeAffector affector(Math::Vec2(1.0f, 1.0f), Math::Vec2(3.0f, 5.0f));
        Particle p;
        p.timeAlive = 1.0f;
        p.lifetime = 4.0f;
        affector.apply(p, 0.1f);
        CHECK(near(p.size.x, 1.5f) && near(p.size.y, 2.0f));
    }
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->addGravity(Math::Vec3(0.0f, -1.0f, 0.0f))
                ->addDrag(0.1f)
                ->addVortex(Math::Vec3(), 1.0f, 1.0f);
        p->addAttractor(Math::Vec3(), 1.0f, 1.0f)->addTurbulence(1.0f, 1.0f);
        p->addColorOverLifetime(Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f), Math::Vec4())
                ->addSizeOverLifetime(Math::Vec2(1.0f, 1.0f), Math::Vec2());
        CHECK(p->getAffectorCount() == 7);
        CHECK(p->getAffector(7) == nullptr);
        CHECK(p->getAffector(0) != nullptr);

        p->clearAffectors();
        CHECK(p->getAffectorCount() == 0);
        p->addGravity(Math::Vec3(0.0f, -1.0f, 0.0f));
        CHECK(p->getAffectorCount() == 1);

        p->setOneShot(1);
        p->play();
        p->getAffector(0)->enabled = false;
        p->update(0.1);
        CHECK(nearVec3(p->getParticles()[0].acceleration, 0.0f, 0.0f, 0.0f));
        p->getAffector(0)->enabled = true;
        p->update(0.1);
        CHECK(nearVec3(p->getParticles()[0].acceleration, 0.0f, -1.0f, 0.0f));
    }
}

void testPlayback()
{
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setContinuous(4.0f);
        p->update(1.0);
        CHECK(p->getActiveCount() == 4);
        float age = p->getParticles()[0].timeAlive;

        p->pause();
        CHECK(p->getState() == EmitterState::Paused);
        p->update(1.0);
        CHECK(p->getActiveCount() == 4);
        CHECK(p->getParticles()[0].timeAlive == age);

        p->play();
        CHECK(p->getState() == EmitterState::Playing);
        p->update(1.0);
        CHECK(p->getActiveCount() == 8);
        CHECK(near(p->getParticles()[0].timeAlive, age + 1.0f));

        p->stop();
        CHECK(p->getState() == EmitterState::Stopped);
        CHECK(p->getActiveCount() == 0);
        for (std::size_t i = 0; i < p->getParticles().size(); ++i)
            CHECK(!p->getParticles()[i].active);
        p->update(1.0);
        CHECK(p->getActiveCount() == 0);

        p->play();
        p->update(1.0);
        CHECK(p->getActiveCount() == 4);
    }
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setContinuous(4.0f)->setDuration(1.0f)->setLoop(false);
        p->update(1.0);
        CHECK(p->getState() == EmitterState::Stopped);
        CHECK(p->getActiveCount() == 0);
    }
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setContinuous(4.0f)->setDuration(1.0f)->setLoop(true);
        p->update(1.0);
        CHECK(p->getState() == EmitterState::Playing);
        CHECK(p->getActiveCount() == 4);
        p->update(1.0);
        CHECK(p->getActiveCount() == 8);
    }
}

void testGeometry()
{
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->update(0.0);
        CHECK(p->getGeometry().positions.empty());
        CHECK(p->getGeometry().indices.empty());
    }
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setSize(Math::Vec2(2.0f, 4.0f), Math::Vec2(2.0f, 4.0f));
        p->setColor(Math::Vec4(0.25f, 0.5f, 0.75f, 0.5f));
        p->setEmissionOffset(Math::Vec3(0.0f, 0.0f, -5.0f));
        p->setOneShot(3);
        p->play();
        p->update(0.0);

        const SurfaceData& g = p->getGeometry();
        CHECK(p->getActiveCount() == 3);
        CHECK(g.positions.size() == 12);
        CHECK(g.normals.size() == 12);
        CHECK(g.colors.size() == 12);
        CHECK(g.uvs.size() == 12);
        CHECK(g.indices.size() == 18);

        const std::uint32_t expected[6] = { 0, 1, 2, 0, 2, 3 };
        for (std::uint32_t quad = 0; quad < 3; ++quad)
            for (int k = 0; k < 6; ++k) CHECK(g.indices[quad * 6 + k] == quad * 4 + expected[k]);

        std::size_t quad = 0;
        for (std::size_t i = 0; i < p->getParticles().size(); ++i)
        {
            const Particle& particle = p->getParticles()[i];
            if (!particle.active) continue;
            float c = cosf(particle.rotation);
            float s = sinf(particle.rotation);
            Math::Vec3 right(c, -s, 0.0f);
            Math::Vec3 up(s, c, 0.0f);
            Math::Vec3 hr = right * 1.0f;
            Math::Vec3 hu = up * 2.0f;
            const Math::Vec3 centre(0.0f, 0.0f, -5.0f);
            const Math::Vec3* v = &g.positions[quad * 4];
            CHECK(nearVec3(v[0], centre.x - hr.x - hu.x, centre.y - hr.y - hu.y, -5.0f));
            CHECK(nearVec3(v[1], centre.x + hr.x - hu.x, centre.y + hr.y - hu.y, -5.0f));
            CHECK(nearVec3(v[2], centre.x + hr.x + hu.x, centre.y + hr.y + hu.y, -5.0f));
            CHECK(nearVec3(v[3], centre.x - hr.x + hu.x, centre.y - hr.y + hu.y, -5.0f));
            CHECK(near((v[1] - v[0]).Length(), 2.0f));
            CHECK(near((v[3] - v[0]).Length(), 4.0f));
            for (int corner = 0; corner < 4; ++corner)
            {
                CHECK(nearVec3(g.normals[quad * 4 + corner], 0.0f, 0.0f, 1.0f));
                CHECK(nearVec4(g.colors[quad * 4 + corner], 0.25f, 0.5f, 0.75f, 0.5f));
            }
            CHECK(near(g.uvs[quad * 4].x, 0.0f) && near(g.uvs[quad * 4].y, 0.0f));
            CHECK(near(g.uvs[quad * 4 + 1].x, 1.0f) && near(g.uvs[quad * 4 + 1].y, 0.0f));
            CHECK(near(g.uvs[quad * 4 + 2].x, 1.0f) && near(g.uvs[quad * 4 + 2].y, 1.0f));
            CHECK(near(g.uvs[quad * 4 + 3].x, 0.0f) && near(g.uvs[quad * 4 + 3].y, 1.0f));
            ++quad;
        }
        CHECK(quad == 3);
    }
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setSize(2.0f)->setOneShot(1);
        Camera3D* camera = scene.tree.getRoot()->createChild<Camera3D>();
        camera->setPosition(Math::Vec3(0.0f, 0.0f, 10.0f));
        camera->setRotationDegrees(Math::Vec3(0.0f, 90.0f, 0.0f));
        camera->makeCurrent();
        const Math::Vec3 right = Math::Vec3(camera->getGlobalTransform().col0).Normalized();
        const Math::Vec3 up = Math::Vec3(camera->getGlobalTransform().col1).Normalized();
        CHECK(near(fabsf(right.z), 1.0f) && near(right.x, 0.0f, 1e-3f));
        CHECK(nearVec3(up, 0.0f, 1.0f, 0.0f, 1e-3f));

        p->play();
        p->update(0.0);
        const Particle& particle = p->getParticles()[0];
        const SurfaceData& g = p->getGeometry();
        CHECK(g.positions.size() == 4);
        float c = cosf(particle.rotation);
        float s = sinf(particle.rotation);
        Math::Vec3 pr = right * c - up * s;
        Math::Vec3 pu = right * s + up * c;
        CHECK(nearVec3(g.positions[0], -pr.x - pu.x, -pr.y - pu.y, -pr.z - pu.z, 1e-3f));
        CHECK(nearVec3(g.positions[2], pr.x + pu.x, pr.y + pu.y, pr.z + pu.z, 1e-3f));
        const Math::Vec3 facing = Math::Vec3::Cross(right, up).Normalized();
        for (std::size_t corner = 0; corner < 4; ++corner)
        {
            CHECK(near(g.positions[corner].x, 0.0f, 1e-3f));
            CHECK(nearVec3(g.normals[corner], facing.x, facing.y, facing.z, 1e-3f));
        }
    }
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setAtlasGrid(2, 2)->setOneShot(1);
        CHECK(p->isUsingAtlas());
        CHECK(p->getAtlasCols() == 2 && p->getAtlasRows() == 2);
        p->play();
        p->update(0.0);
        CHECK(nearVec4(p->getParticles()[0].texRect, 0.0f, 0.0f, 0.5f, 0.5f));
        const SurfaceData& g = p->getGeometry();
        CHECK(near(g.uvs[0].x, 0.0f) && near(g.uvs[0].y, 0.0f));
        CHECK(near(g.uvs[1].x, 0.5f) && near(g.uvs[1].y, 0.0f));
        CHECK(near(g.uvs[2].x, 0.5f) && near(g.uvs[2].y, 0.5f));
        CHECK(near(g.uvs[3].x, 0.0f) && near(g.uvs[3].y, 0.5f));

        p->setAtlasGrid(4, 2);
        p->emitBurst(1);
        bool found = false;
        for (std::size_t i = 0; i < p->getParticles().size(); ++i)
            if (nearVec4(p->getParticles()[i].texRect, 0.0f, 0.0f, 0.25f, 0.5f)) found = true;
        CHECK(found);

        p->clearAtlas();
        CHECK(!p->isUsingAtlas());
        p->emitBurst(1);
        int full = 0;
        for (std::size_t i = 0; i < p->getParticles().size(); ++i)
            if (p->getParticles()[i].active && nearVec4(p->getParticles()[i].texRect, 0, 0, 1, 1))
                ++full;
        CHECK(full == 1);
    }
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setPosition(Math::Vec3(10.0f, 0.0f, 0.0f));
        p->setSize(2.0f)->setOneShot(1);
        p->play();
        p->update(0.0);
        const Particle& particle = p->getParticles()[0];
        CHECK(nearVec3(particle.position, 10.0f, 0.0f, 0.0f));
        const SurfaceData& g = p->getGeometry();
        CHECK(g.positions.size() == 4);
        CHECK(nearVec3((g.positions[0] + g.positions[2]) * 0.5f, 0.0f, 0.0f, 0.0f));
        CHECK(nearVec3((g.positions[1] + g.positions[3]) * 0.5f, 0.0f, 0.0f, 0.0f));
        CHECK(near((g.positions[1] - g.positions[0]).Length(), 2.0f));
    }
    {
        Scene scene(4);
        ParticleSystem3D* p = scene.particles;
        p->emitBurst(10);
        p->update(0.0);
        CHECK(p->getGeometry().positions.size() == 16);
        CHECK(p->getGeometry().indices.size() == 24);
    }
}

} // namespace

void testColliders()
{
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();
    Math::Vec3 normal;
    float depth = 0.0f;

    ParticleCollisionSphere3D* sphere = root->createChild<ParticleCollisionSphere3D>();
    CHECK(near(sphere->getRadius(), 1.0f));
    CHECK(tree.getParticleColliders().size() == 1);
    CHECK(tree.getParticleColliders()[0] == sphere);
    CHECK(sphere->castTo<ParticleCollision3D>() == sphere);
    CHECK(sphere->castTo<ParticleCollisionBox3D>() == nullptr);

    CHECK(sphere->collide(Math::Vec3(0.0f, 1.005f, 0.0f), 0.01f, normal, depth));
    CHECK(nearVec3(normal, 0.0f, 1.0f, 0.0f) && near(depth, 0.005f));
    CHECK(sphere->collide(Math::Vec3(0.0f, 1.0105f, 0.0f), 0.01f, normal, depth));
    CHECK(near(depth, -0.0005f));
    CHECK(!sphere->collide(Math::Vec3(0.0f, 1.02f, 0.0f), 0.01f, normal, depth));
    CHECK(sphere->collide(Math::Vec3(-0.5f, 0.0f, 0.0f), 0.0f, normal, depth));
    CHECK(nearVec3(normal, -1.0f, 0.0f, 0.0f) && near(depth, 0.5f));

    sphere->setRadius(0.5f);
    sphere->setScale(Math::Vec3(2.0f, 4.0f, 6.0f));
    sphere->setPosition(Math::Vec3(10.0f, 0.0f, 0.0f));
    CHECK(sphere->collide(Math::Vec3(10.0f, 0.0f, 1.9f), 0.0f, normal, depth));
    CHECK(nearVec3(normal, 0.0f, 0.0f, 1.0f) && near(depth, 0.1f));
    CHECK(!sphere->collide(Math::Vec3(10.0f, 0.0f, 2.1f), 0.0f, normal, depth));

    ParticleCollisionBox3D* box = root->createChild<ParticleCollisionBox3D>();
    CHECK(nearVec3(box->getSize(), 2.0f, 2.0f, 2.0f));
    CHECK(tree.getParticleColliders().size() == 2);
    CHECK(box->collide(Math::Vec3(0.0f, 1.005f, 0.0f), 0.01f, normal, depth));
    CHECK(nearVec3(normal, 0.0f, 1.0f, 0.0f) && near(depth, 0.005f));
    CHECK(box->collide(Math::Vec3(0.0f, -1.005f, 0.0f), 0.01f, normal, depth));
    CHECK(nearVec3(normal, 0.0f, -1.0f, 0.0f) && near(depth, 0.005f));
    CHECK(!box->collide(Math::Vec3(0.0f, 1.5f, 0.0f), 0.01f, normal, depth));
    CHECK(box->collide(Math::Vec3(1.003f, 1.004f, 0.0f), 0.01f, normal, depth));
    CHECK(nearVec3(normal, 0.6f, 0.8f, 0.0f) && near(depth, 0.005f));

    CHECK(box->collide(Math::Vec3(0.2f, 0.9f, 0.1f), 0.01f, normal, depth));
    CHECK(nearVec3(normal, 0.0f, 1.0f, 0.0f) && near(depth, 0.11f));
    CHECK(box->collide(Math::Vec3(-0.95f, 0.2f, 0.1f), 0.01f, normal, depth));
    CHECK(nearVec3(normal, -1.0f, 0.0f, 0.0f) && near(depth, 0.06f));
    CHECK(box->collide(Math::Vec3(0.2f, 0.1f, -0.9f), 0.0f, normal, depth));
    CHECK(nearVec3(normal, 0.0f, 0.0f, -1.0f) && near(depth, 0.1f));

    box->setSize(Math::Vec3(2.0f, 4.0f, 2.0f));
    CHECK(nearVec3(box->getSize(), 2.0f, 4.0f, 2.0f));
    box->setRotationDegrees(Math::Vec3(0.0f, 0.0f, 90.0f));
    CHECK(box->collide(Math::Vec3(-2.005f, 0.0f, 0.0f), 0.01f, normal, depth));
    CHECK(nearVec3(normal, -1.0f, 0.0f, 0.0f) && near(depth, 0.005f));
    CHECK(!box->collide(Math::Vec3(0.0f, 1.5f, 0.0f), 0.01f, normal, depth));

    box->setRotationDegrees(Math::Vec3(0.0f, 0.0f, 0.0f));
    box->setScale(Math::Vec3(1.0f, 0.25f, 1.0f));
    CHECK(box->collide(Math::Vec3(0.0f, 0.505f, 0.0f), 0.01f, normal, depth));
    CHECK(nearVec3(normal, 0.0f, 1.0f, 0.0f) && near(depth, 0.005f));
    CHECK(!box->collide(Math::Vec3(0.0f, 0.6f, 0.0f), 0.01f, normal, depth));

    sphere->queueFree();
    tree.process(1.0 / 60.0);
    CHECK(tree.getParticleColliders().size() == 1);
    CHECK(tree.getParticleColliders()[0] == box);
    box->queueFree();
    tree.process(1.0 / 60.0);
    CHECK(tree.getParticleColliders().empty());
}

struct GroundScene : Scene
{
    ParticleCollisionBox3D* ground;

    GroundScene()
            : Scene(),
              ground(nullptr)
    {
        ground = tree.getRoot()->createChild<ParticleCollisionBox3D>();
        ground->setSize(Math::Vec3(20.0f, 2.0f, 20.0f));
        ground->setPosition(Math::Vec3(0.0f, -1.0f, 0.0f));
        particles->setEmissionOffset(Math::Vec3(0.0f, 0.005f, 0.0f));
        particles->setEmissionDirection(Math::Vec3(0.0f, -1.0f, 0.0f));
        particles->setSpeed(4.0f);
        particles->setOneShot(1);
    }
};

void testCollision()
{
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        CHECK(p->getCollisionMode() == ParticleCollisionMode::Disabled);
        CHECK(near(p->getCollisionFriction(), 0.0f));
        CHECK(near(p->getCollisionBounce(), 0.0f));
        CHECK(near(p->getCollisionBaseSize(), 0.01f));
        CHECK(!p->getCollisionUseScale());
        CHECK(p->getAlign() == ParticleAlign::Billboard);
        CHECK(!p->hasScaleOverVelocity());
    }
    {
        GroundScene scene;
        ParticleSystem3D* p = scene.particles;
        p->play();
        p->update(0.1);
        CHECK(nearVec3(p->getParticles()[0].position, 0.0f, -0.395f, 0.0f));
        CHECK(nearVec3(p->getParticles()[0].velocity, 0.0f, -4.0f, 0.0f));
    }
    {
        GroundScene scene;
        ParticleSystem3D* p = scene.particles;
        p->setCollisionMode(ParticleCollisionMode::Rigid)->setCollisionBounce(0.5f);
        CHECK(p->getCollisionMode() == ParticleCollisionMode::Rigid);
        CHECK(near(p->getCollisionBounce(), 0.5f));
        p->play();
        p->update(0.1);
        CHECK(nearVec3(p->getParticles()[0].velocity, 0.0f, 2.0f, 0.0f));
        CHECK(nearVec3(p->getParticles()[0].position, 0.0f, 0.21f, 0.0f));
    }
    {
        GroundScene scene;
        ParticleSystem3D* p = scene.particles;
        p->setCollisionMode(ParticleCollisionMode::Rigid)->setCollisionBounce(0.5f);
        p->setSpeed(1.0f);
        p->play();
        p->update(0.1);
        CHECK(nearVec3(p->getParticles()[0].velocity, 0.0f, 0.0f, 0.0f));
        CHECK(nearVec3(p->getParticles()[0].position, 0.0f, 0.01f, 0.0f));
    }
    {
        GroundScene scene;
        ParticleSystem3D* p = scene.particles;
        p->setCollisionMode(ParticleCollisionMode::Rigid)->setCollisionFriction(0.25f);
        CHECK(near(p->getCollisionFriction(), 0.25f));
        p->setEmissionDirection(Math::Vec3(1.0f, -1.0f, 0.0f));
        p->play();
        p->update(0.1);
        const float tangent = 4.0f * sqrtf(0.5f) * 0.75f;
        CHECK(nearVec3(p->getParticles()[0].velocity, tangent, 0.0f, 0.0f));
        CHECK(nearVec3(p->getParticles()[0].position, tangent * 0.1f, 0.01f, 0.0f));
    }
    {
        GroundScene scene;
        ParticleSystem3D* p = scene.particles;
        p->setCollisionMode(ParticleCollisionMode::HideOnContact);
        p->play();
        CHECK(p->getParticles()[0].active);
        p->update(0.1);
        CHECK(p->getActiveCount() == 0);
        CHECK(!p->getParticles()[0].active);
    }
    {
        GroundScene scene;
        ParticleSystem3D* p = scene.particles;
        p->setCollisionMode(ParticleCollisionMode::Rigid);
        scene.ground->hide();
        p->play();
        p->update(0.1);
        CHECK(nearVec3(p->getParticles()[0].position, 0.0f, -0.395f, 0.0f));
    }
    {
        GroundScene scene;
        ParticleSystem3D* p = scene.particles;
        p->setCollisionMode(ParticleCollisionMode::Rigid);
        p->setEmissionOffset(Math::Vec3(0.0f, 0.03f, 0.0f));
        p->setSpeed(0.0f)->setSize(4.0f);
        p->play();
        p->update(0.1);
        CHECK(nearVec3(p->getParticles()[0].position, 0.0f, 0.03f, 0.0f));
        p->setCollisionUseScale(true);
        CHECK(p->getCollisionUseScale());
        p->update(0.1);
        CHECK(nearVec3(p->getParticles()[0].position, 0.0f, 0.04f, 0.0f));
        p->setCollisionUseScale(false);
        p->setCollisionBaseSize(0.1f);
        CHECK(near(p->getCollisionBaseSize(), 0.1f));
        p->update(0.1);
        CHECK(nearVec3(p->getParticles()[0].position, 0.0f, 0.1f, 0.0f));
    }
    {
        GroundScene scene;
        ParticleSystem3D* p = scene.particles;
        ParticleCollisionBox3D* wall = scene.tree.getRoot()->createChild<ParticleCollisionBox3D>();
        wall->setSize(Math::Vec3(2.0f, 20.0f, 20.0f));
        wall->setPosition(Math::Vec3(-1.0f, 0.0f, 0.0f));
        p->setCollisionMode(ParticleCollisionMode::Rigid);
        p->setEmissionOffset(Math::Vec3(0.005f, 0.005f, 0.0f));
        p->setSpeed(0.0f);
        p->play();
        p->update(0.1);
        CHECK(nearVec3(p->getParticles()[0].position, 0.01f, 0.01f, 0.0f));
    }
}

void testAlign()
{
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setSize(Math::Vec2(2.0f, 4.0f), Math::Vec2(2.0f, 4.0f));
        p->setEmissionDirection(Math::Vec3(1.0f, 0.0f, 0.0f));
        p->setSpeed(5.0f);
        p->setRotationSpeed(1.0f, 1.0f);
        p->setAlign(ParticleAlign::YToVelocity);
        CHECK(p->getAlign() == ParticleAlign::YToVelocity);
        p->setOneShot(1);
        p->play();
        p->update(0.0);

        const SurfaceData& g = p->getGeometry();
        CHECK(g.positions.size() == 4);
        CHECK(nearVec3(g.positions[0], -2.0f, 1.0f, 0.0f));
        CHECK(nearVec3(g.positions[1], -2.0f, -1.0f, 0.0f));
        CHECK(nearVec3(g.positions[2], 2.0f, -1.0f, 0.0f));
        CHECK(nearVec3(g.positions[3], 2.0f, 1.0f, 0.0f));

        p->setScaleOverVelocity(0.0f, 10.0f, Math::Vec2(1.0f, 1.0f), Math::Vec2(1.0f, 3.0f));
        CHECK(p->hasScaleOverVelocity());
        p->update(0.0);
        CHECK(nearVec3(p->getGeometry().positions[0], -4.0f, 1.0f, 0.0f));
        CHECK(nearVec3(p->getGeometry().positions[2], 4.0f, -1.0f, 0.0f));

        p->setScaleOverVelocity(0.0f, 2.0f, Math::Vec2(1.0f, 1.0f), Math::Vec2(0.5f, 3.0f));
        p->update(0.0);
        CHECK(nearVec3(p->getGeometry().positions[0], -6.0f, 0.5f, 0.0f));
        CHECK(nearVec3(p->getGeometry().positions[2], 6.0f, -0.5f, 0.0f));

        p->setScaleOverVelocity(5.0f, 15.0f, Math::Vec2(1.0f, 1.0f), Math::Vec2(1.0f, 3.0f));
        p->update(0.0);
        CHECK(nearVec3(p->getGeometry().positions[0], -4.0f, 1.0f, 0.0f));

        p->clearScaleOverVelocity();
        CHECK(!p->hasScaleOverVelocity());
        p->update(0.0);
        CHECK(nearVec3(p->getGeometry().positions[0], -2.0f, 1.0f, 0.0f));
    }
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setSize(Math::Vec2(2.0f, 4.0f), Math::Vec2(2.0f, 4.0f));
        p->setEmissionDirection(Math::Vec3(0.0f, 0.0f, 1.0f));
        p->setSpeed(5.0f);
        p->setAlign(ParticleAlign::YToVelocity);
        p->setOneShot(1);
        p->play();
        p->update(0.0);
        const SurfaceData& g = p->getGeometry();
        CHECK(nearVec3(g.positions[0], -1.0f, -2.0f, 0.0f));
        CHECK(nearVec3(g.positions[2], 1.0f, 2.0f, 0.0f));
    }
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setSize(Math::Vec2(2.0f, 4.0f), Math::Vec2(2.0f, 4.0f));
        p->setEmissionDirection(Math::Vec3(0.0f, -1.0f, 1.0f));
        p->setSpeed(5.0f);
        p->setAlign(ParticleAlign::YToVelocity);
        p->setOneShot(1);
        p->play();
        p->update(0.0);
        const SurfaceData& g = p->getGeometry();
        CHECK(nearVec3(g.positions[0], 1.0f, 2.0f, 0.0f));
        CHECK(nearVec3(g.positions[1], -1.0f, 2.0f, 0.0f));
        CHECK(nearVec3(g.positions[2], -1.0f, -2.0f, 0.0f));
    }
}

void testSheetAnimation()
{
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        CHECK(near(p->getAnimSpeedMin(), 0.0f) && near(p->getAnimSpeedMax(), 0.0f));
        CHECK(near(p->getAnimOffsetMin(), 0.0f) && near(p->getAnimOffsetMax(), 0.0f));
        CHECK(!p->getAnimLoop());

        p->setAtlasGrid(4, 2)->setLifetime(1.0f)->setOneShot(1);
        p->setAnimSpeed(1.0f);
        CHECK(near(p->getAnimSpeedMin(), 1.0f) && near(p->getAnimSpeedMax(), 1.0f));
        p->play();
        const Particle& particle = p->getParticles()[0];
        CHECK(near(particle.animSpeed, 1.0f) && near(particle.animOffset, 0.0f));
        CHECK(nearVec4(particle.texRect, 0.0f, 0.0f, 0.25f, 0.5f));
        p->update(0.3);
        CHECK(nearVec4(particle.texRect, 0.5f, 0.0f, 0.25f, 0.5f));
        p->update(0.3);
        CHECK(nearVec4(particle.texRect, 0.0f, 0.5f, 0.25f, 0.5f));
        p->update(0.35);
        CHECK(nearVec4(particle.texRect, 0.75f, 0.5f, 0.25f, 0.5f));

        const SurfaceData& g = p->getGeometry();
        CHECK(near(g.uvs[0].x, 0.75f) && near(g.uvs[0].y, 0.5f));
        CHECK(near(g.uvs[2].x, 1.0f) && near(g.uvs[2].y, 1.0f));
    }
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setAtlasGrid(4, 2)->setLifetime(1.0f)->setOneShot(1);
        p->setAnimSpeed(2.0f);
        p->play();
        p->update(0.6);
        CHECK(nearVec4(p->getParticles()[0].texRect, 0.75f, 0.5f, 0.25f, 0.5f));
    }
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setAtlasGrid(4, 2)->setLifetime(1.0f)->setOneShot(1);
        p->setAnimSpeed(2.0f)->setAnimLoop(true);
        CHECK(p->getAnimLoop());
        p->play();
        p->update(0.6);
        CHECK(nearVec4(p->getParticles()[0].texRect, 0.25f, 0.0f, 0.25f, 0.5f));
    }
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setAtlasGrid(4, 2)->setLifetime(1.0f)->setOneShot(1);
        p->setAnimOffset(0.5f);
        CHECK(near(p->getAnimOffsetMin(), 0.5f) && near(p->getAnimOffsetMax(), 0.5f));
        p->play();
        CHECK(nearVec4(p->getParticles()[0].texRect, 0.0f, 0.5f, 0.25f, 0.5f));
        p->update(0.5);
        CHECK(nearVec4(p->getParticles()[0].texRect, 0.0f, 0.5f, 0.25f, 0.5f));
    }
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setAtlasGrid(4, 2)->setOneShot(64);
        p->setAnimOffset(0.25f, 0.75f)->setAnimSpeed(1.0f, 3.0f);
        p->play();
        int seen = 0;
        for (std::size_t i = 0; i < p->getParticles().size(); ++i)
        {
            const Particle& particle = p->getParticles()[i];
            if (!particle.active) continue;
            ++seen;
            CHECK(particle.animOffset >= 0.25f && particle.animOffset <= 0.75f);
            CHECK(particle.animSpeed >= 1.0f && particle.animSpeed <= 3.0f);
            CHECK(particle.texRect.x >= 0.0f && particle.texRect.y >= 0.0f);
            CHECK(particle.texRect.x + particle.texRect.z <= 1.0001f);
        }
        CHECK(seen == 64);
    }
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        p->setLifetime(1.0f)->setOneShot(1);
        p->setAnimSpeed(1.0f);
        p->play();
        p->update(0.5);
        CHECK(nearVec4(p->getParticles()[0].texRect, 0.0f, 0.0f, 1.0f, 1.0f));
    }
}

struct SubScene : GroundScene
{
    ParticleSystem3D* sub;

    SubScene()
            : GroundScene(),
              sub(nullptr)
    {
        particles->setName("source");
        sub = tree.getRoot()->createChild<ParticleSystem3D>(100);
        sub->setName("sub");
        sub->setOneShot(0)->setLifetime(100.0f)->setSpeed(0.0f);
        particles->setSubEmitter("../sub");
    }

    int subCount() const
    {
        int count = 0;
        for (std::size_t i = 0; i < sub->getParticles().size(); ++i)
            if (sub->getParticles()[i].active) ++count;
        return count;
    }
};

void testSubEmitters()
{
    {
        Scene scene;
        ParticleSystem3D* p = scene.particles;
        CHECK(p->getSubEmitter().empty());
        CHECK(p->getSubEmitterMode() == ParticleSubEmitterMode::Disabled);
        CHECK(near(p->getSubEmitterFrequency(), 4.0f));
        CHECK(p->getSubEmitterAmountAtEnd() == 1);
        CHECK(p->getSubEmitterAmountAtCollision() == 1);
        CHECK(p->getSubEmitterAmountAtStart() == 1);
        CHECK(!p->getSubEmitterKeepVelocity());
    }
    {
        Scene scene(2);
        ParticleSystem3D* p = scene.particles;
        p->setSpeed(3.0f);
        CHECK(p->emitParticle(Math::Vec3(1.0f, 2.0f, 3.0f)));
        CHECK(p->getParticles()[0].active);
        CHECK(nearVec3(p->getParticles()[0].position, 1.0f, 2.0f, 3.0f));
        CHECK(nearVec3(p->getParticles()[0].velocity, 0.0f, 3.0f, 0.0f));
        CHECK(p->emitParticle(Math::Vec3(4.0f, 5.0f, 6.0f), Math::Vec3(5.0f, 0.0f, 0.0f)));
        CHECK(nearVec3(p->getParticles()[1].position, 4.0f, 5.0f, 6.0f));
        CHECK(nearVec3(p->getParticles()[1].velocity, 5.0f, 0.0f, 0.0f));
        CHECK(!p->emitParticle(Math::Vec3(0.0f, 0.0f, 0.0f)));
    }
    {
        SubScene scene;
        ParticleSystem3D* p = scene.particles;
        p->setSubEmitterMode(ParticleSubEmitterMode::AtStart)->setSubEmitterAmountAtStart(3);
        CHECK(p->getSubEmitterMode() == ParticleSubEmitterMode::AtStart);
        CHECK(p->getSubEmitterAmountAtStart() == 3);
        CHECK(p->getSubEmitter() == "../sub");
        p->setEmissionOffset(Math::Vec3(1.0f, 5.0f, 0.0f))->setSpeed(0.0f)->setOneShot(2);
        p->play();
        CHECK(scene.subCount() == 0);
        p->update(0.1);
        CHECK(scene.subCount() == 6);
        CHECK(nearVec3(scene.sub->getParticles()[0].position, 1.0f, 5.0f, 0.0f));
        CHECK(nearVec3(scene.sub->getParticles()[5].position, 1.0f, 5.0f, 0.0f));
        p->update(0.1);
        CHECK(scene.subCount() == 6);
    }
    {
        SubScene scene;
        ParticleSystem3D* p = scene.particles;
        p->setSubEmitterMode(ParticleSubEmitterMode::AtEnd)->setSubEmitterAmountAtEnd(2);
        CHECK(p->getSubEmitterAmountAtEnd() == 2);
        p->setEmissionOffset(Math::Vec3(0.0f, 5.0f, 0.0f));
        p->setEmissionDirection(Math::Vec3(0.0f, 1.0f, 0.0f));
        p->setSpeed(1.0f)->setLifetime(0.25f);
        p->play();
        p->update(0.1);
        p->update(0.1);
        CHECK(scene.subCount() == 0);
        p->update(0.1);
        CHECK(scene.subCount() == 2);
        CHECK(!p->getParticles()[0].active);
        CHECK(nearVec3(scene.sub->getParticles()[0].position, 0.0f, 5.2f, 0.0f));
        CHECK(nearVec3(scene.sub->getParticles()[0].velocity, 0.0f, 0.0f, 0.0f));
        p->update(0.1);
        CHECK(scene.subCount() == 2);
    }
    {
        SubScene scene;
        ParticleSystem3D* p = scene.particles;
        p->setSubEmitterMode(ParticleSubEmitterMode::Constant)->setSubEmitterFrequency(4.0f);
        p->setEmissionOffset(Math::Vec3(0.0f, 5.0f, 0.0f))->setSpeed(0.0f);
        p->play();
        for (int i = 0; i < 3; ++i) p->update(0.0625);
        CHECK(scene.subCount() == 0);
        p->update(0.0625);
        CHECK(scene.subCount() == 1);
        for (int i = 0; i < 3; ++i) p->update(0.0625);
        CHECK(scene.subCount() == 1);
        p->update(0.0625);
        CHECK(scene.subCount() == 2);
        p->setSubEmitterFrequency(0.0f);
        for (int i = 0; i < 8; ++i) p->update(0.0625);
        CHECK(scene.subCount() == 2);
    }
    {
        SubScene scene;
        ParticleSystem3D* p = scene.particles;
        p->setCollisionMode(ParticleCollisionMode::Rigid)->setCollisionBounce(0.5f);
        p->setSubEmitterMode(ParticleSubEmitterMode::AtCollision);
        p->setSubEmitterAmountAtCollision(4)->setSubEmitterKeepVelocity(true);
        CHECK(p->getSubEmitterAmountAtCollision() == 4);
        CHECK(p->getSubEmitterKeepVelocity());
        p->play();
        p->update(0.1);
        CHECK(scene.subCount() == 4);
        CHECK(nearVec3(scene.sub->getParticles()[3].position, 0.0f, 0.21f, 0.0f));
        CHECK(nearVec3(scene.sub->getParticles()[3].velocity, 0.0f, 2.0f, 0.0f));
        p->update(0.1);
        CHECK(scene.subCount() == 4);
    }
    {
        SubScene scene;
        ParticleSystem3D* p = scene.particles;
        scene.sub->setSpeed(3.0f);
        p->setCollisionMode(ParticleCollisionMode::HideOnContact);
        p->setSubEmitterMode(ParticleSubEmitterMode::AtCollision);
        p->setSubEmitterAmountAtCollision(2);
        p->play();
        p->update(0.1);
        CHECK(!p->getParticles()[0].active);
        CHECK(scene.subCount() == 2);
        CHECK(nearVec3(scene.sub->getParticles()[0].position, 0.0f, 0.005f, 0.0f));
        CHECK(nearVec3(scene.sub->getParticles()[0].velocity, 0.0f, 3.0f, 0.0f));
    }
    {
        SubScene scene;
        ParticleSystem3D* p = scene.particles;
        p->setSubEmitterAmountAtStart(3);
        p->play();
        p->update(0.1);
        CHECK(scene.subCount() == 0);

        p->setSubEmitterMode(ParticleSubEmitterMode::AtStart);
        p->setSubEmitter("../source");
        p->emitBurst(1);
        p->update(0.1);
        CHECK(scene.subCount() == 0);
        CHECK(p->getActiveCount() == 2);

        p->setSubEmitter("../ground");
        scene.ground->setName("ground");
        p->emitBurst(1);
        p->update(0.1);
        CHECK(scene.subCount() == 0);
        CHECK(p->getActiveCount() == 3);
    }
}

struct Contacts
{
    int count = 0;
    int wasActive = 0;
    Math::Vec3 position = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Vec3 velocity = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Vec3 normal = Math::Vec3(0.0f, 0.0f, 0.0f);
};

void listen(ParticleSystem3D* particles, Contacts& contacts)
{
    particles->collided = [&contacts](const Particle& particle, const Math::Vec3& normal)
    {
        ++contacts.count;
        contacts.wasActive += particle.active ? 1 : 0;
        contacts.position = particle.position;
        contacts.velocity = particle.velocity;
        contacts.normal = normal;
    };
}

void testCollisionCallback()
{
    {
        GroundScene scene;
        ParticleSystem3D* p = scene.particles;
        CHECK(!p->collided);
        Contacts contacts;
        listen(p, contacts);
        CHECK(static_cast<bool>(p->collided));
        p->play();
        p->update(0.1);
        CHECK(contacts.count == 0);
    }
    {
        GroundScene scene;
        ParticleSystem3D* p = scene.particles;
        p->setCollisionMode(ParticleCollisionMode::Rigid);
        scene.ground->hide();
        Contacts contacts;
        listen(p, contacts);
        p->play();
        p->update(0.1);
        CHECK(contacts.count == 0);
    }
    {
        GroundScene scene;
        ParticleSystem3D* p = scene.particles;
        p->setCollisionMode(ParticleCollisionMode::Rigid)->setCollisionBounce(0.5f);
        Contacts contacts;
        listen(p, contacts);
        p->play();
        p->update(0.1);
        CHECK(contacts.count == 1);
        CHECK(contacts.wasActive == 1);
        CHECK(nearVec3(contacts.position, 0.0f, 0.01f, 0.0f));
        CHECK(nearVec3(contacts.velocity, 0.0f, -4.0f, 0.0f));
        CHECK(nearVec3(contacts.normal, 0.0f, 1.0f, 0.0f));
        CHECK(nearVec3(p->getParticles()[0].velocity, 0.0f, 2.0f, 0.0f));
        CHECK(nearVec3(p->getParticles()[0].position, 0.0f, 0.21f, 0.0f));
        p->update(0.1);
        CHECK(contacts.count == 1);
        CHECK(nearVec3(p->getParticles()[0].position, 0.0f, 0.41f, 0.0f));
    }
    {
        GroundScene scene;
        ParticleSystem3D* p = scene.particles;
        p->setCollisionMode(ParticleCollisionMode::Rigid)->setCollisionBounce(0.5f);
        p->setOneShot(3);
        Contacts contacts;
        listen(p, contacts);
        p->play();
        p->update(0.1);
        CHECK(p->getActiveCount() == 3);
        CHECK(contacts.count == 3);
        CHECK(nearVec3(contacts.position, 0.0f, 0.01f, 0.0f));
    }
    {
        GroundScene scene;
        ParticleSystem3D* p = scene.particles;
        p->setCollisionMode(ParticleCollisionMode::HideOnContact);
        Contacts contacts;
        listen(p, contacts);
        p->play();
        p->update(0.1);
        CHECK(contacts.count == 1);
        CHECK(contacts.wasActive == 1);
        CHECK(nearVec3(contacts.position, 0.0f, 0.01f, 0.0f));
        CHECK(nearVec3(contacts.velocity, 0.0f, -4.0f, 0.0f));
        CHECK(nearVec3(contacts.normal, 0.0f, 1.0f, 0.0f));
        CHECK(p->getActiveCount() == 0);
        p->update(0.1);
        CHECK(contacts.count == 1);
    }
    {
        SubScene scene;
        ParticleSystem3D* p = scene.particles;
        p->setCollisionMode(ParticleCollisionMode::HideOnContact);
        p->setSubEmitterMode(ParticleSubEmitterMode::AtCollision);
        p->setSubEmitterAmountAtCollision(2);
        Contacts contacts;
        listen(p, contacts);
        p->play();
        p->update(0.1);
        CHECK(contacts.count == 1);
        CHECK(scene.subCount() == 2);
    }
    {
        GroundScene scene;
        ParticleSystem3D* p = scene.particles;
        scene.ground->hide();
        ParticleCollisionBox3D* wall = scene.tree.getRoot()->createChild<ParticleCollisionBox3D>();
        wall->setSize(Math::Vec3(2.0f, 20.0f, 20.0f));
        wall->setPosition(Math::Vec3(-1.0f, 0.0f, 0.0f));
        p->setCollisionMode(ParticleCollisionMode::HideOnContact);
        p->setEmissionOffset(Math::Vec3(0.005f, 0.005f, 0.0f));
        p->setSpeed(0.0f);
        Contacts contacts;
        listen(p, contacts);
        p->play();
        p->update(0.1);
        CHECK(contacts.count == 1);
        CHECK(nearVec3(contacts.position, 0.01f, 0.005f, 0.0f));
        CHECK(nearVec3(contacts.normal, 1.0f, 0.0f, 0.0f));
    }
    {
        GroundScene scene;
        ParticleSystem3D* p = scene.particles;
        p->setCollisionMode(ParticleCollisionMode::Rigid)->setCollisionBounce(0.5f);
        Contacts contacts;
        listen(p, contacts);
        p->collided = nullptr;
        CHECK(!p->collided);
        p->play();
        p->update(0.1);
        CHECK(contacts.count == 0);
        CHECK(nearVec3(p->getParticles()[0].position, 0.0f, 0.21f, 0.0f));
    }
}

int main()
{
    testRandom();
    testDefaults();
    testEmissionModes();
    testMaxParticlesAndExpiry();
    testShapes();
    testVelocity();
    testSimulation();
    testAffectors();
    testPlayback();
    testGeometry();
    testColliders();
    testCollision();
    testAlign();
    testSheetAnimation();
    testSubEmitters();
    testCollisionCallback();

    if (failures != 0)
    {
        printf("%d checks failed\n", failures);
        return 1;
    }
    printf("particles: all checks passed\n");
    return 0;
}
