#include "prisma3d/scene/ImpactFx3D.h"

#include "NodeFactory.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/ParticleSystem3D.h"

#include <math.h>

namespace prisma3d
{

namespace
{

constexpr int kSparksPerFrame = 60;
constexpr int kSmokePerFrame = 3;
constexpr int kFlamePerFrame = 14;

TextureId softDisc()
{
    TextureStore* textures = Main::textures();
    if (!textures) return TextureId();
    const int size = 64;
    Pixmap pixmap(size, size, 4);
    for (int y = 0; y < size; ++y)
        for (int x = 0; x < size; ++x)
        {
            const float dx = (static_cast<float>(x) + 0.5f) / size * 2.0f - 1.0f;
            const float dy = (static_cast<float>(y) + 0.5f) / size * 2.0f - 1.0f;
            const float edge = fmaxf(0.0f, 1.0f - sqrtf(dx * dx + dy * dy));
            const float alpha = edge * edge * (3.0f - 2.0f * edge);
            pixmap.set_pixel(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y), 255, 255,
                    255, static_cast<std::uint8_t>(alpha * 255.0f + 0.5f));
        }
    return textures->createFromPixmap(pixmap, true, true, "impact disc");
}

} // namespace

ImpactFx3D::ImpactFx3D()
        : Node3D(),
          mDisc(),
          mSparks(nullptr),
          mSmoke(nullptr),
          mFlame(nullptr),
          mBlack(nullptr),
          mFlameBudget(kFlamePerFrame),
          mSparkBudget(kSparksPerFrame),
          mSmokeBudget(kSmokePerFrame),
          mCrashSounds(),
          mSounds(),
          mNextSound(0),
          mSoundsLoaded(false)
{
    setProcess(true);
}

ImpactFx3D::~ImpactFx3D()
{
    release();
}

void ImpactFx3D::build()
{
    if (mSparks) return;
    mDisc = softDisc();

    mSparks = createChild<ParticleSystem3D>(1500);
    mSparks->setInternal(true);
    mSparks->setContinuous(0.0f);
    mSparks->setTexture(mDisc);
    mSparks->setBlendMode(ParticleBlendMode::Additive);
    mSparks->setAlign(ParticleAlign::YToVelocity);
    mSparks->setGravity(Math::Vec3(0.0f, -9.8f, 0.0f));
    mSparks->setLifetime(0.3f, 0.8f);
    mSparks->setSize(Math::Vec2(0.03f, 0.22f), Math::Vec2(0.012f, 0.06f));
    mSparks->setColor(Math::Vec4(1.0f, 0.78f, 0.35f, 1.0f), Math::Vec4(1.0f, 0.25f, 0.05f, 0.0f));

    mSmoke = createChild<ParticleSystem3D>(1500);
    mSmoke->setInternal(true);
    mSmoke->setContinuous(0.0f);
    mSmoke->setTexture(mDisc);
    mSmoke->setBlendMode(ParticleBlendMode::Alpha);
    mSmoke->setGravity(Math::Vec3(0.0f, 0.7f, 0.0f));
    mSmoke->setLifetime(0.7f, 1.4f);
    mSmoke->setSize(Math::Vec2(0.25f, 0.25f), Math::Vec2(1.1f, 1.1f));
    mSmoke->setColor(Math::Vec4(0.5f, 0.5f, 0.52f, 0.28f), Math::Vec4(0.7f, 0.7f, 0.72f, 0.0f));
    mSmoke->setRotationSpeed(-0.6f, 0.6f);

    mFlame = createChild<ParticleSystem3D>(1500);
    mFlame->setInternal(true);
    mFlame->setContinuous(0.0f);
    mFlame->setTexture(mDisc);
    mFlame->setBlendMode(ParticleBlendMode::Additive);
    mFlame->setGravity(Math::Vec3(0.0f, 3.2f, 0.0f));
    mFlame->setLifetime(0.4f, 0.9f);
    mFlame->setSize(Math::Vec2(0.3f, 0.3f), Math::Vec2(0.9f, 0.9f));
    mFlame->setColor(Math::Vec4(1.0f, 0.7f, 0.22f, 0.85f), Math::Vec4(0.9f, 0.12f, 0.02f, 0.0f));
    mFlame->setRotationSpeed(-1.0f, 1.0f);

    mBlack = createChild<ParticleSystem3D>(1200);
    mBlack->setInternal(true);
    mBlack->setContinuous(0.0f);
    mBlack->setTexture(mDisc);
    mBlack->setBlendMode(ParticleBlendMode::Alpha);
    mBlack->setGravity(Math::Vec3(0.0f, 1.6f, 0.0f));
    mBlack->setLifetime(1.8f, 3.2f);
    mBlack->setSize(Math::Vec2(0.5f, 0.5f), Math::Vec2(2.4f, 2.4f));
    mBlack->setColor(Math::Vec4(0.06f, 0.06f, 0.06f, 0.42f), Math::Vec4(0.22f, 0.22f, 0.22f, 0.0f));
    mBlack->setRotationSpeed(-0.5f, 0.5f);
}

void ImpactFx3D::release()
{
    ParticleSystem3D* systems[4] = { mSparks, mSmoke, mFlame, mBlack };
    for (ParticleSystem3D* system: systems)
        if (system) NodeFactory::destroy(system);
    mSparks = nullptr;
    mSmoke = nullptr;
    mFlame = nullptr;
    mBlack = nullptr;
    TextureStore* textures = Main::textures();
    if (textures && mDisc.valid()) textures->destroy(mDisc);
    mDisc = TextureId();
}

void ImpactFx3D::flame(const Math::Vec3& point, int count)
{
    if (!mFlame) return;
    Random& random = mFlame->getRandom();
    for (int i = 0; i < count && mFlameBudget > 0; ++i, --mFlameBudget)
    {
        const Math::Vec3 at =
                point + Math::Vec3(random.randfRange(-0.35f, 0.35f), random.randfRange(0.0f, 0.2f),
                                random.randfRange(-0.35f, 0.35f));
        mFlame->emitParticle(at,
                Math::Vec3(random.randfRange(-0.4f, 0.4f), random.randfRange(0.8f, 2.0f),
                        random.randfRange(-0.4f, 0.4f)));
        if ((i & 1) == 0)
            mBlack->emitParticle(at + Math::Vec3(0.0f, 0.3f, 0.0f),
                    Math::Vec3(random.randfRange(-0.3f, 0.3f), random.randfRange(1.0f, 2.2f),
                            random.randfRange(-0.3f, 0.3f)));
    }
}

void ImpactFx3D::blast(const Math::Vec3& point)
{
    if (!mFlame) return;
    Random& random = mFlame->getRandom();
    mFlame->setSize(Math::Vec2(0.9f, 0.9f), Math::Vec2(2.8f, 2.8f));
    mFlame->setLifetime(0.5f, 1.2f);
    for (int i = 0; i < 170; ++i)
    {
        Math::Vec3 v(random.randfRange(-1.0f, 1.0f), random.randfRange(-0.2f, 1.0f),
                random.randfRange(-1.0f, 1.0f));
        mFlame->emitParticle(point + v * 0.3f, v.NormalizedSafe() * random.randfRange(2.5f, 11.0f));
    }
    mFlame->setSize(Math::Vec2(0.3f, 0.3f), Math::Vec2(0.9f, 0.9f));
    mFlame->setLifetime(0.4f, 0.9f);
    for (int i = 0; i < 24; ++i)
    {
        Math::Vec3 v(random.randfRange(-1.0f, 1.0f), random.randfRange(0.2f, 1.0f),
                random.randfRange(-1.0f, 1.0f));
        mBlack->emitParticle(point + v * 0.5f, v.NormalizedSafe() * random.randfRange(1.5f, 6.0f));
    }
    for (int i = 0; i < 120; ++i)
    {
        Math::Vec3 v(random.randfRange(-1.0f, 1.0f), random.randfRange(0.0f, 1.0f),
                random.randfRange(-1.0f, 1.0f));
        mSparks->emitParticle(point, v.NormalizedSafe() * random.randfRange(5.0f, 16.0f));
    }
}

Math::Vec3 ImpactFx3D::scatter(float lift)
{
    Random& random = mSparks->getRandom();
    Math::Vec3 v(random.randfRange(-1.0f, 1.0f), random.randfRange(0.0f, 1.0f) + lift,
            random.randfRange(-1.0f, 1.0f));
    return v.NormalizedSafe();
}

void ImpactFx3D::impact(const Math::Vec3& point, float impulse)
{
    if (!mSparks) return;
    Random& random = mSparks->getRandom();
    int sparks = static_cast<int>(impulse / 70.0f);
    if (sparks < 3) sparks = 3;
    if (sparks > 30) sparks = 30;
    for (int i = 0; i < sparks && mSparkBudget > 0; ++i, --mSparkBudget)
        mSparks->emitParticle(point, scatter(0.3f) * random.randfRange(3.0f, 11.0f));
    int puffs = static_cast<int>(impulse / 1500.0f);
    if (puffs < 1) puffs = 1;
    if (puffs > 2) puffs = 2;
    smoke(point, puffs);
}

void ImpactFx3D::smoke(const Math::Vec3& point, int count)
{
    if (!mSmoke) return;
    Random& random = mSmoke->getRandom();
    for (int i = 0; i < count && mSmokeBudget > 0; ++i, --mSmokeBudget)
        mSmoke->emitParticle(point + scatter(0.0f) * 0.25f,
                scatter(0.8f) * random.randfRange(0.5f, 1.8f));
}

void ImpactFx3D::setCrashSounds(const ct::String& paths)
{
    mCrashSounds = paths;
    mSounds.clear();
    mSoundsLoaded = false;
}

const ct::String& ImpactFx3D::getCrashSounds() const
{
    return mCrashSounds;
}

void ImpactFx3D::loadSounds()
{
    mSoundsLoaded = true;
    mSounds.clear();
    AudioServer* audio = Main::audio();
    if (!audio) return;
    std::size_t start = 0;
    for (std::size_t i = 0; i <= mCrashSounds.size(); ++i)
    {
        if (i < mCrashSounds.size() && mCrashSounds[i] != ',') continue;
        const ct::String path = mCrashSounds.substr(start, i - start).trimmed();
        start = i + 1;
        if (path.empty()) continue;
        SoundId sound = audio->findSoundByPath(path);
        if (!audio->validSound(sound)) sound = audio->createSoundFromFile(path);
        if (audio->validSound(sound)) mSounds.push_back(sound);
    }
}

void ImpactFx3D::crash(const Math::Vec3& point, float volume)
{
    if (!mSoundsLoaded) loadSounds();
    AudioServer* audio = Main::audio();
    if (!audio || mSounds.empty()) return;
    const SoundId sound = mSounds[mNextSound++ % mSounds.size()];
    audio->playOneShotAt(sound, point, volume, 1.0f, 4.0f, 150.0f);
}

void ImpactFx3D::notificationHandler(int what)
{
    Node3D::notificationHandler(what);
    switch (what)
    {
        case kNotificationEnterTree:
            build();
            break;
        case kNotificationProcess:
            mSparkBudget = kSparksPerFrame;
            mFlameBudget = kFlamePerFrame;
            mSmokeBudget = kSmokePerFrame;
            break;
        default:
            break;
    }
}

void ImpactFx3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = ImpactFx3D;
    DB::addProperty<P>(PropertyInfo(PropertyType::String, "crash_sounds"), &P::setCrashSounds,
            &P::getCrashSounds);
}

} // namespace prisma3d
