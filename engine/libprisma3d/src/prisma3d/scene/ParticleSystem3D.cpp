#include "prisma3d/scene/ParticleSystem3D.h"

#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/ParticleCollision3D.h"
#include "prisma3d/scene/ResourceProperties.h"
#include "prisma3d/scene/SceneJson.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Viewport.h"

#include <ct/json.hpp>

#include <math.h>

namespace prisma3d
{

namespace
{

const char* const kMeshName = "particles";

constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = kPi * 2.f;

Math::Vec4 lerp4(const Math::Vec4& a, const Math::Vec4& b, float t)
{
    return Math::Vec4(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t,
            a.w + (b.w - a.w) * t);
}

Math::Vec2 lerp2(const Math::Vec2& a, const Math::Vec2& b, float t)
{
    return Math::Vec2(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
}

Math::Vec3 axisFromColumn(const Math::Vec4& column)
{
    return Math::Vec3(column.x, column.y, column.z).Normalized();
}

} // namespace

void GravityAffector::apply(Particle& particle, float)
{
    if (enabled) particle.acceleration += gravity;
}

void DragAffector::apply(Particle& particle, float dt)
{
    if (enabled) particle.velocity = particle.velocity * (1.f - drag * dt);
}

void VortexAffector::apply(Particle& particle, float)
{
    if (!enabled) return;
    Math::Vec3 toCenter = center - particle.position;
    float d = toCenter.Length();
    if (d < radius && d > 0.001f)
    {
        Math::Vec3 dir = toCenter * (1.f / d);
        Math::Vec3 tangent(-dir.z, 0.f, dir.x);
        particle.acceleration += tangent * (strength * (1.f - d / radius));
    }
}

void AttractorAffector::apply(Particle& particle, float)
{
    if (!enabled) return;
    Math::Vec3 to = position - particle.position;
    float d = to.Length();
    if (d < radius && d > 0.001f)
        particle.acceleration +=
                (to * (1.f / d)) * (strength * (1.f - d / radius) * (repulse ? -1.f : 1.f));
}

void TurbulenceAffector::apply(Particle& particle, float dt)
{
    if (!enabled) return;
    time += dt;
    particle.acceleration += Math::Vec3(sinf(particle.position.x * frequency + time) * strength,
            sinf(particle.position.y * frequency + time * 1.3f) * strength,
            cosf(particle.position.z * frequency + time * 0.7f) * strength);
}

void ColorOverLifetimeAffector::apply(Particle& particle, float)
{
    if (enabled)
        particle.color = lerp4(startColor, endColor, particle.timeAlive / particle.lifetime);
}

void SizeOverLifetimeAffector::apply(Particle& particle, float)
{
    if (enabled) particle.size = lerp2(startSize, endSize, particle.timeAlive / particle.lifetime);
}

ParticleSystem3D::ParticleSystem3D(int maxParticles)
        : VisualInstance3D(),
          mParticles(),
          mAffectors(),
          mMaxParticles(maxParticles),
          mActiveCount(0),
          mEmissionMode(EmissionMode::Continuous),
          mEmitterShape(EmitterShape::Point),
          mState(EmitterState::Stopped),
          mEmissionOffset(0.0f, 0.0f, 0.0f),
          mEmissionDirection(0.0f, 1.0f, 0.0f),
          mEmissionRate(10.0f),
          mBurstInterval(1.0f),
          mBurstCount(10),
          mOneShotCount(10),
          mPulseRate(1.0f),
          mParticlesPerPulse(5),
          mAtlasFrames(),
          mAtlasCols(1),
          mAtlasRows(1),
          mUseAtlas(false),
          mAnimSpeedMin(0.0f),
          mAnimSpeedMax(0.0f),
          mAnimOffsetMin(0.0f),
          mAnimOffsetMax(0.0f),
          mAnimLoop(false),
          mRadius(1.0f),
          mInnerRadius(0.5f),
          mConeAngle(0.785f),
          mConeAngleDeg(0.785f * (180.f / 3.14159265f)),
          mBoxSize(1.0f, 1.0f, 1.0f),
          mLifetimeMin(1.0f),
          mLifetimeMax(3.0f),
          mSpeedMin(1.0f),
          mSpeedMax(5.0f),
          mSizeStart(0.5f, 0.5f),
          mSizeEnd(0.1f, 0.1f),
          mColorStart(1.0f, 1.0f, 1.0f, 1.0f),
          mColorEnd(1.0f, 1.0f, 1.0f, 0.0f),
          mRotSpeedMin(0.0f),
          mRotSpeedMax(0.0f),
          mSpreadAngle(0.0f),
          mSpreadAngleDeg(0.0f),
          mGravity(0.0f, 0.0f, 0.0f),
          mDrag(0.0f),
          mDuration(-1.0f),
          mLoop(true),
          mEmissionTimer(0.0f),
          mTimeAlive(0.0f),
          mBurstTimer(0.0f),
          mPulseTimer(0.0f),
          mHasEmittedOneShot(false),
          mAlign(ParticleAlign::Billboard),
          mScaleOverVelocity(false),
          mScaleVelocityMin(0.0f),
          mScaleVelocityMax(5.0f),
          mScaleAtMinVelocity(1.0f, 1.0f),
          mScaleAtMaxVelocity(1.0f, 1.0f),
          mCollisionMode(ParticleCollisionMode::Disabled),
          mCollisionFriction(0.0f),
          mCollisionBounce(0.0f),
          mCollisionBaseSize(0.01f),
          mCollisionUseScale(false),
          mSubEmitter(),
          mSubEmitterMode(ParticleSubEmitterMode::Disabled),
          mSubEmitterFrequency(4.0f),
          mSubEmitterAmountAtEnd(1),
          mSubEmitterAmountAtCollision(1),
          mSubEmitterAmountAtStart(1),
          mSubEmitterKeepVelocity(false),
          mRandom(),
          mBlend(ParticleBlendMode::Additive),
          mTexture(),
          mGeometry(),
          mMesh(),
          mSurfaceReady(false),
          mUploadedEmpty(true),
          mDefaultMaterial()
{

    mRandom.randomize();
    mParticles.resize(static_cast<std::size_t>(maxParticles > 0 ? maxParticles : 0));

    setCastShadows(false);

    if (MaterialStore* materials = MaterialStore::singleton())
    {
        mDefaultMaterial = materials->create(kMeshName);
        materials->setShading(mDefaultMaterial, MaterialShading::Unshaded);
        materials->setCull(mDefaultMaterial, MaterialCull::Disabled);
        applyMaterial();
    }
}

ParticleSystem3D::~ParticleSystem3D()
{
    disconnectPreDraw();

    if (RenderingServer* server = RenderingServer::singleton())
    {
        server->instanceSetMaterialOverride(getInstance(), MaterialId());
        server->instanceSetBase(getInstance(), MeshId());
    }
    if (MeshStore* store = MeshStore::singleton())
        if (mMesh.valid()) store->destroy(mMesh);
    if (MaterialStore* materials = MaterialStore::singleton())
        if (mDefaultMaterial.valid()) materials->destroy(mDefaultMaterial);

    clearAffectors();
}

void ParticleSystem3D::setTexture(TextureId texture)
{
    mTexture = texture;
    applyMaterial();
}

TextureId ParticleSystem3D::getTexture() const
{
    return mTexture;
}

void ParticleSystem3D::setBlendMode(ParticleBlendMode mode)
{
    mBlend = mode;
    applyMaterial();
}

ParticleBlendMode ParticleSystem3D::getBlendMode() const
{
    return mBlend;
}

ParticleSystem3D* ParticleSystem3D::setEmissionMode(EmissionMode mode)
{
    const bool changed = mEmissionMode != mode;
    mEmissionMode = mode;
    if (changed) notifyPropertyListChanged();
    return this;
}

void ParticleSystem3D::validateProperty(PropertyInfo& info) const
{
    VisualInstance3D::validateProperty(info);
    const ct::String& name = info.name;
    bool shown = true;
    if (name == "emission_rate") shown = mEmissionMode == EmissionMode::Continuous;
    else if (name == "burst_count" || name == "burst_interval")
        shown = mEmissionMode == EmissionMode::Burst;
    else if (name == "one_shot_count")
        shown = mEmissionMode == EmissionMode::OneShot;
    else if (name == "pulse_rate" || name == "particles_per_pulse")
        shown = mEmissionMode == EmissionMode::Pulse;
    else if (name == "shape_radius")
        shown = mEmitterShape != EmitterShape::Point && mEmitterShape != EmitterShape::Box;
    else if (name == "shape_inner_radius")
        shown = mEmitterShape == EmitterShape::Ring;
    else if (name == "shape_cone_angle")
        shown = mEmitterShape == EmitterShape::Cone;
    else if (name == "shape_box_size")
        shown = mEmitterShape == EmitterShape::Box;
    else if (name.starts_with("atlas_") && name != "atlas_enabled")
        shown = mUseAtlas;
    if (!shown) info.usage = PropertyUsage::NoEditor;
}

ParticleSystem3D* ParticleSystem3D::setContinuous(float particlesPerSecond)
{
    const bool changed = mEmissionMode != EmissionMode::Continuous;
    mEmissionMode = EmissionMode::Continuous;
    if (changed) notifyPropertyListChanged();
    mEmissionRate = particlesPerSecond;
    return this;
}

ParticleSystem3D* ParticleSystem3D::setBurst(int count, float interval)
{
    const bool changed = mEmissionMode != EmissionMode::Burst;
    mEmissionMode = EmissionMode::Burst;
    if (changed) notifyPropertyListChanged();
    mBurstCount = count;
    mBurstInterval = interval;
    return this;
}

ParticleSystem3D* ParticleSystem3D::setOneShot(int count)
{
    const bool changed = mEmissionMode != EmissionMode::OneShot;
    mEmissionMode = EmissionMode::OneShot;
    if (changed) notifyPropertyListChanged();
    mOneShotCount = count;
    return this;
}

ParticleSystem3D* ParticleSystem3D::setPulse(float particlesPerSecond, int perPulse)
{
    const bool changed = mEmissionMode != EmissionMode::Pulse;
    mEmissionMode = EmissionMode::Pulse;
    if (changed) notifyPropertyListChanged();
    mPulseRate = particlesPerSecond;
    mParticlesPerPulse = perPulse;
    return this;
}

ParticleSystem3D* ParticleSystem3D::setShapePoint()
{
    const bool changed = mEmitterShape != EmitterShape::Point;
    mEmitterShape = EmitterShape::Point;
    if (changed) notifyPropertyListChanged();
    return this;
}

ParticleSystem3D* ParticleSystem3D::setShapeSphere(float radius)
{
    const bool changed = mEmitterShape != EmitterShape::Sphere;
    mEmitterShape = EmitterShape::Sphere;
    if (changed) notifyPropertyListChanged();
    mRadius = radius;
    return this;
}

ParticleSystem3D* ParticleSystem3D::setShapeBox(const Math::Vec3& size)
{
    const bool changed = mEmitterShape != EmitterShape::Box;
    mEmitterShape = EmitterShape::Box;
    if (changed) notifyPropertyListChanged();
    mBoxSize = size;
    return this;
}

ParticleSystem3D* ParticleSystem3D::setShapeCone(float degrees, float baseRadius)
{
    const bool changed = mEmitterShape != EmitterShape::Cone;
    mEmitterShape = EmitterShape::Cone;
    if (changed) notifyPropertyListChanged();
    mConeAngle = degrees * (kPi / 180.f);
    mConeAngleDeg = degrees;
    mRadius = baseRadius;
    return this;
}

ParticleSystem3D* ParticleSystem3D::setShapeCircle(float radius)
{
    const bool changed = mEmitterShape != EmitterShape::Circle;
    mEmitterShape = EmitterShape::Circle;
    if (changed) notifyPropertyListChanged();
    mRadius = radius;
    return this;
}

ParticleSystem3D* ParticleSystem3D::setShapeRing(float outer, float inner)
{
    const bool changed = mEmitterShape != EmitterShape::Ring;
    mEmitterShape = EmitterShape::Ring;
    if (changed) notifyPropertyListChanged();
    mRadius = outer;
    mInnerRadius = inner;
    return this;
}

ParticleSystem3D* ParticleSystem3D::setEmissionOffset(const Math::Vec3& offset)
{
    mEmissionOffset = offset;
    return this;
}

ParticleSystem3D* ParticleSystem3D::setEmissionDirection(const Math::Vec3& direction)
{
    mEmissionDirection = direction.Normalized();
    return this;
}

ParticleSystem3D* ParticleSystem3D::setSpreadAngle(float degrees)
{
    mSpreadAngle = degrees * (kPi / 180.f);
    mSpreadAngleDeg = degrees;
    return this;
}

ParticleSystem3D* ParticleSystem3D::setLifetime(float minimum, float maximum)
{
    mLifetimeMin = minimum;
    mLifetimeMax = maximum;
    return this;
}

ParticleSystem3D* ParticleSystem3D::setLifetime(float lifetime)
{
    return setLifetime(lifetime, lifetime);
}

ParticleSystem3D* ParticleSystem3D::setSpeed(float minimum, float maximum)
{
    mSpeedMin = minimum;
    mSpeedMax = maximum;
    return this;
}

ParticleSystem3D* ParticleSystem3D::setSpeed(float speed)
{
    return setSpeed(speed, speed);
}

ParticleSystem3D* ParticleSystem3D::setSize(const Math::Vec2& start, const Math::Vec2& end)
{
    mSizeStart = start;
    mSizeEnd = end;
    return this;
}

ParticleSystem3D* ParticleSystem3D::setSize(float size)
{
    return setSize(Math::Vec2(size, size), Math::Vec2(size, size));
}

ParticleSystem3D* ParticleSystem3D::setColor(const Math::Vec4& start, const Math::Vec4& end)
{
    mColorStart = start;
    mColorEnd = end;
    return this;
}

ParticleSystem3D* ParticleSystem3D::setColor(const Math::Vec4& color)
{
    return setColor(color, color);
}

ParticleSystem3D* ParticleSystem3D::setRotationSpeed(float minimum, float maximum)
{
    mRotSpeedMin = minimum;
    mRotSpeedMax = maximum;
    return this;
}

ParticleSystem3D* ParticleSystem3D::setGravity(const Math::Vec3& gravity)
{
    mGravity = gravity;
    return this;
}

ParticleSystem3D* ParticleSystem3D::setDrag(float drag)
{
    mDrag = drag;
    return this;
}

ParticleSystem3D* ParticleSystem3D::setDuration(float seconds)
{
    mDuration = seconds;
    return this;
}

ParticleSystem3D* ParticleSystem3D::setLoop(bool loop)
{
    mLoop = loop;
    return this;
}

ParticleSystem3D* ParticleSystem3D::setAlign(ParticleAlign align)
{
    mAlign = align;
    return this;
}

ParticleAlign ParticleSystem3D::getAlign() const
{
    return mAlign;
}

ParticleSystem3D* ParticleSystem3D::setScaleOverVelocity(float minVelocity, float maxVelocity,
        const Math::Vec2& scaleAtMin, const Math::Vec2& scaleAtMax)
{
    mScaleOverVelocity = true;
    mScaleVelocityMin = minVelocity;
    mScaleVelocityMax = maxVelocity;
    mScaleAtMinVelocity = scaleAtMin;
    mScaleAtMaxVelocity = scaleAtMax;
    return this;
}

void ParticleSystem3D::clearScaleOverVelocity()
{
    mScaleOverVelocity = false;
}

bool ParticleSystem3D::hasScaleOverVelocity() const
{
    return mScaleOverVelocity;
}

ParticleSystem3D* ParticleSystem3D::setScaleOverVelocityEnabled(bool enabled)
{
    mScaleOverVelocity = enabled;
    return this;
}

ParticleSystem3D* ParticleSystem3D::setScaleVelocityMin(float velocity)
{
    mScaleVelocityMin = velocity;
    return this;
}

float ParticleSystem3D::getScaleVelocityMin() const
{
    return mScaleVelocityMin;
}

ParticleSystem3D* ParticleSystem3D::setScaleVelocityMax(float velocity)
{
    mScaleVelocityMax = velocity;
    return this;
}

float ParticleSystem3D::getScaleVelocityMax() const
{
    return mScaleVelocityMax;
}

ParticleSystem3D* ParticleSystem3D::setScaleAtMinVelocity(const Math::Vec2& scale)
{
    mScaleAtMinVelocity = scale;
    return this;
}

const Math::Vec2& ParticleSystem3D::getScaleAtMinVelocity() const
{
    return mScaleAtMinVelocity;
}

ParticleSystem3D* ParticleSystem3D::setScaleAtMaxVelocity(const Math::Vec2& scale)
{
    mScaleAtMaxVelocity = scale;
    return this;
}

const Math::Vec2& ParticleSystem3D::getScaleAtMaxVelocity() const
{
    return mScaleAtMaxVelocity;
}

ParticleSystem3D* ParticleSystem3D::setCollisionMode(ParticleCollisionMode mode)
{
    mCollisionMode = mode;
    return this;
}

ParticleCollisionMode ParticleSystem3D::getCollisionMode() const
{
    return mCollisionMode;
}

ParticleSystem3D* ParticleSystem3D::setCollisionFriction(float friction)
{
    mCollisionFriction = friction;
    return this;
}

float ParticleSystem3D::getCollisionFriction() const
{
    return mCollisionFriction;
}

ParticleSystem3D* ParticleSystem3D::setCollisionBounce(float bounce)
{
    mCollisionBounce = bounce;
    return this;
}

float ParticleSystem3D::getCollisionBounce() const
{
    return mCollisionBounce;
}

ParticleSystem3D* ParticleSystem3D::setCollisionBaseSize(float size)
{
    mCollisionBaseSize = size;
    return this;
}

float ParticleSystem3D::getCollisionBaseSize() const
{
    return mCollisionBaseSize;
}

ParticleSystem3D* ParticleSystem3D::setCollisionUseScale(bool useScale)
{
    mCollisionUseScale = useScale;
    return this;
}

bool ParticleSystem3D::getCollisionUseScale() const
{
    return mCollisionUseScale;
}

ParticleSystem3D* ParticleSystem3D::setSubEmitter(const ct::String& path)
{
    mSubEmitter = path;
    return this;
}

const ct::String& ParticleSystem3D::getSubEmitter() const
{
    return mSubEmitter;
}

ParticleSystem3D* ParticleSystem3D::setSubEmitterMode(ParticleSubEmitterMode mode)
{
    mSubEmitterMode = mode;
    return this;
}

ParticleSubEmitterMode ParticleSystem3D::getSubEmitterMode() const
{
    return mSubEmitterMode;
}

ParticleSystem3D* ParticleSystem3D::setSubEmitterFrequency(float frequency)
{
    mSubEmitterFrequency = frequency;
    return this;
}

float ParticleSystem3D::getSubEmitterFrequency() const
{
    return mSubEmitterFrequency;
}

ParticleSystem3D* ParticleSystem3D::setSubEmitterAmountAtEnd(int amount)
{
    mSubEmitterAmountAtEnd = amount;
    return this;
}

int ParticleSystem3D::getSubEmitterAmountAtEnd() const
{
    return mSubEmitterAmountAtEnd;
}

ParticleSystem3D* ParticleSystem3D::setSubEmitterAmountAtCollision(int amount)
{
    mSubEmitterAmountAtCollision = amount;
    return this;
}

int ParticleSystem3D::getSubEmitterAmountAtCollision() const
{
    return mSubEmitterAmountAtCollision;
}

ParticleSystem3D* ParticleSystem3D::setSubEmitterAmountAtStart(int amount)
{
    mSubEmitterAmountAtStart = amount;
    return this;
}

int ParticleSystem3D::getSubEmitterAmountAtStart() const
{
    return mSubEmitterAmountAtStart;
}

ParticleSystem3D* ParticleSystem3D::setSubEmitterKeepVelocity(bool keepVelocity)
{
    mSubEmitterKeepVelocity = keepVelocity;
    return this;
}

bool ParticleSystem3D::getSubEmitterKeepVelocity() const
{
    return mSubEmitterKeepVelocity;
}

ParticleSystem3D* ParticleSystem3D::setAtlasGrid(int columns, int rows)
{
    mAtlasFrames.clear();
    float cw = 1.f / columns;
    float ch = 1.f / rows;
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < columns; ++c)
            mAtlasFrames.push_back(Math::Vec4(c * cw, r * ch, cw, ch));
    mAtlasCols = columns;
    mAtlasRows = rows;
    const bool changed = !mUseAtlas;
    mUseAtlas = true;
    if (changed) notifyPropertyListChanged();
    return this;
}

void ParticleSystem3D::clearAtlas()
{
    mAtlasFrames.clear();
    const bool changed = mUseAtlas;
    mUseAtlas = false;
    if (changed) notifyPropertyListChanged();
}

ParticleSystem3D* ParticleSystem3D::setAnimSpeed(float minimum, float maximum)
{
    mAnimSpeedMin = minimum;
    mAnimSpeedMax = maximum;
    return this;
}

ParticleSystem3D* ParticleSystem3D::setAnimSpeed(float speed)
{
    return setAnimSpeed(speed, speed);
}

ParticleSystem3D* ParticleSystem3D::setAnimOffset(float minimum, float maximum)
{
    mAnimOffsetMin = minimum;
    mAnimOffsetMax = maximum;
    return this;
}

ParticleSystem3D* ParticleSystem3D::setAnimOffset(float offset)
{
    return setAnimOffset(offset, offset);
}

ParticleSystem3D* ParticleSystem3D::setAnimLoop(bool loop)
{
    mAnimLoop = loop;
    return this;
}

float ParticleSystem3D::getAnimSpeedMin() const
{
    return mAnimSpeedMin;
}

float ParticleSystem3D::getAnimSpeedMax() const
{
    return mAnimSpeedMax;
}

float ParticleSystem3D::getAnimOffsetMin() const
{
    return mAnimOffsetMin;
}

float ParticleSystem3D::getAnimOffsetMax() const
{
    return mAnimOffsetMax;
}

bool ParticleSystem3D::getAnimLoop() const
{
    return mAnimLoop;
}

void ParticleSystem3D::play()
{
    mState = EmitterState::Playing;
    if (mEmissionMode == EmissionMode::OneShot && !mHasEmittedOneShot)
    {
        emitBurst(mOneShotCount);
        mHasEmittedOneShot = true;
    }
}

void ParticleSystem3D::stop()
{
    mState = EmitterState::Stopped;
    for (std::size_t i = 0; i < mParticles.size(); ++i) mParticles[i].active = false;
    mActiveCount = 0;
}

void ParticleSystem3D::pause()
{
    mState = EmitterState::Paused;
}

EmitterState ParticleSystem3D::getState() const
{
    return mState;
}

ParticleSystem3D* ParticleSystem3D::addGravity(const Math::Vec3& gravity)
{
    mAffectors.push_back(new GravityAffector(gravity));
    return this;
}

ParticleSystem3D* ParticleSystem3D::addDrag(float strength)
{
    mAffectors.push_back(new DragAffector(strength));
    return this;
}

ParticleSystem3D* ParticleSystem3D::addVortex(const Math::Vec3& center, float strength,
        float radius)
{
    mAffectors.push_back(new VortexAffector(center, strength, radius));
    return this;
}

ParticleSystem3D* ParticleSystem3D::addAttractor(const Math::Vec3& position, float strength,
        float radius, bool repulse)
{
    mAffectors.push_back(new AttractorAffector(position, strength, radius, repulse));
    return this;
}

ParticleSystem3D* ParticleSystem3D::addTurbulence(float strength, float frequency)
{
    mAffectors.push_back(new TurbulenceAffector(strength, frequency));
    return this;
}

ParticleSystem3D* ParticleSystem3D::addColorOverLifetime(const Math::Vec4& start,
        const Math::Vec4& end)
{
    mAffectors.push_back(new ColorOverLifetimeAffector(start, end));
    return this;
}

ParticleSystem3D* ParticleSystem3D::addSizeOverLifetime(const Math::Vec2& start,
        const Math::Vec2& end)
{
    mAffectors.push_back(new SizeOverLifetimeAffector(start, end));
    return this;
}

void ParticleSystem3D::clearAffectors()
{
    for (std::size_t i = 0; i < mAffectors.size(); ++i) delete mAffectors[i];
    mAffectors.clear();
}

std::size_t ParticleSystem3D::getAffectorCount() const
{
    return mAffectors.size();
}

ParticleAffector* ParticleSystem3D::getAffector(std::size_t index) const
{
    return index < mAffectors.size() ? mAffectors[index] : nullptr;
}

int ParticleSystem3D::getMaxParticles() const
{
    return mMaxParticles;
}

void ParticleSystem3D::setMaxParticles(int maxParticles)
{
    if (maxParticles < 0) maxParticles = 0;
    if (maxParticles == mMaxParticles) return;
    mMaxParticles = maxParticles;
    mParticles.resize(static_cast<std::size_t>(maxParticles));
    mActiveCount = 0;
    for (std::size_t i = 0; i < mParticles.size(); ++i)
        if (mParticles[i].active) ++mActiveCount;
    mGeometry.positions.clear();
    mGeometry.normals.clear();
    mGeometry.colors.clear();
    mGeometry.uvs.clear();
    mGeometry.indices.clear();

    if (RenderingServer* server = RenderingServer::singleton())
        server->instanceSetBase(getInstance(), MeshId());
    if (MeshStore* store = MeshStore::singleton())
        if (mMesh.valid()) store->destroy(mMesh);
    mMesh = MeshId();
    mSurfaceReady = false;
    mUploadedEmpty = true;
}

int ParticleSystem3D::getActiveCount() const
{
    return mActiveCount;
}

const ct::Vector<Particle>& ParticleSystem3D::getParticles() const
{
    return mParticles;
}

Random& ParticleSystem3D::getRandom()
{
    return mRandom;
}

EmissionMode ParticleSystem3D::getEmissionMode() const
{
    return mEmissionMode;
}

float ParticleSystem3D::getEmissionRate() const
{
    return mEmissionRate;
}

int ParticleSystem3D::getBurstCount() const
{
    return mBurstCount;
}

float ParticleSystem3D::getBurstInterval() const
{
    return mBurstInterval;
}

int ParticleSystem3D::getOneShotCount() const
{
    return mOneShotCount;
}

float ParticleSystem3D::getPulseRate() const
{
    return mPulseRate;
}

int ParticleSystem3D::getParticlesPerPulse() const
{
    return mParticlesPerPulse;
}

EmitterShape ParticleSystem3D::getEmitterShape() const
{
    return mEmitterShape;
}

float ParticleSystem3D::getShapeRadius() const
{
    return mRadius;
}

float ParticleSystem3D::getShapeInnerRadius() const
{
    return mInnerRadius;
}

float ParticleSystem3D::getShapeConeAngleDeg() const
{
    return mConeAngleDeg;
}

const Math::Vec3& ParticleSystem3D::getShapeBoxSize() const
{
    return mBoxSize;
}

const Math::Vec3& ParticleSystem3D::getEmissionOffset() const
{
    return mEmissionOffset;
}

const Math::Vec3& ParticleSystem3D::getEmissionDirection() const
{
    return mEmissionDirection;
}

float ParticleSystem3D::getSpreadAngleDeg() const
{
    return mSpreadAngleDeg;
}

float ParticleSystem3D::getLifetimeMin() const
{
    return mLifetimeMin;
}

float ParticleSystem3D::getLifetimeMax() const
{
    return mLifetimeMax;
}

float ParticleSystem3D::getSpeedMin() const
{
    return mSpeedMin;
}

float ParticleSystem3D::getSpeedMax() const
{
    return mSpeedMax;
}

const Math::Vec2& ParticleSystem3D::getSizeStart() const
{
    return mSizeStart;
}

const Math::Vec2& ParticleSystem3D::getSizeEnd() const
{
    return mSizeEnd;
}

const Math::Vec4& ParticleSystem3D::getColorStart() const
{
    return mColorStart;
}

const Math::Vec4& ParticleSystem3D::getColorEnd() const
{
    return mColorEnd;
}

float ParticleSystem3D::getRotSpeedMin() const
{
    return mRotSpeedMin;
}

float ParticleSystem3D::getRotSpeedMax() const
{
    return mRotSpeedMax;
}

const Math::Vec3& ParticleSystem3D::getGravity() const
{
    return mGravity;
}

float ParticleSystem3D::getDrag() const
{
    return mDrag;
}

float ParticleSystem3D::getDuration() const
{
    return mDuration;
}

bool ParticleSystem3D::getLoop() const
{
    return mLoop;
}

bool ParticleSystem3D::isUsingAtlas() const
{
    return mUseAtlas;
}

int ParticleSystem3D::getAtlasCols() const
{
    return mAtlasCols;
}

int ParticleSystem3D::getAtlasRows() const
{
    return mAtlasRows;
}

void ParticleSystem3D::simulate(float dt)
{
    if (mState == EmitterState::Paused) return;
    mTimeAlive += dt;
    if (mState == EmitterState::Playing)
    {
        switch (mEmissionMode)
        {
            case EmissionMode::Continuous:
                emitContinuous(dt);
                break;
            case EmissionMode::Burst:
                emitBurstMode(dt);
                break;
            case EmissionMode::Pulse:
                emitPulseMode(dt);
                break;
            case EmissionMode::OneShot:
                break;
        }
        if (mDuration > 0.f && mTimeAlive >= mDuration)
        {
            if (mLoop)
            {
                mTimeAlive = 0.f;
                mHasEmittedOneShot = false;
            }
            else
                stop();
        }
    }
    updateParticles(dt);
    applyAffectors(dt);
    mActiveCount = 0;
    for (std::size_t i = 0; i < mParticles.size(); ++i)
        if (mParticles[i].active) ++mActiveCount;
}

void ParticleSystem3D::emitContinuous(float dt)
{
    mEmissionTimer += dt;
    float interval = 1.f / mEmissionRate;
    int count = 0;
    while (mEmissionTimer >= interval)
    {
        ++count;
        mEmissionTimer -= interval;
        if (count > 100)
        {
            mEmissionTimer = 0;
            break;
        }
    }
    for (int i = 0; i < count; ++i)
    {
        Particle* particle = getFreeParticle();
        if (particle) initParticle(particle);
    }
}

void ParticleSystem3D::emitBurstMode(float dt)
{
    mBurstTimer += dt;
    if (mBurstTimer >= mBurstInterval)
    {
        emitBurst(mBurstCount);
        mBurstTimer = 0;
        if (!mLoop) stop();
    }
}

void ParticleSystem3D::emitPulseMode(float dt)
{
    mPulseTimer += dt;
    float interval = 1.f / mPulseRate;
    if (mPulseTimer >= interval)
    {
        emitBurst(mParticlesPerPulse);
        mPulseTimer = 0;
    }
}

void ParticleSystem3D::emitBurst(int count)
{
    for (int i = 0; i < count; ++i)
    {
        Particle* particle = getFreeParticle();
        if (particle) initParticle(particle);
    }
}

Particle* ParticleSystem3D::getFreeParticle()
{
    for (std::size_t i = 0; i < mParticles.size(); ++i)
        if (!mParticles[i].active) return &mParticles[i];
    Particle* oldest = nullptr;
    float maxProgress = 0.9f;
    for (std::size_t i = 0; i < mParticles.size(); ++i)
    {
        Particle& particle = mParticles[i];
        float progress = particle.timeAlive / particle.lifetime;
        if (progress > maxProgress)
        {
            maxProgress = progress;
            oldest = &particle;
        }
    }
    return oldest;
}

void ParticleSystem3D::initParticle(Particle* particle)
{
    particle->position = calcEmissionPosition();
    particle->velocity = calcEmissionVelocity();
    particle->acceleration = mGravity;
    particle->lifetime = rnd(mLifetimeMin, mLifetimeMax);
    particle->timeAlive = 0;
    particle->sizeStart = mSizeStart;
    particle->sizeEnd = mSizeEnd;
    particle->size = mSizeStart;
    particle->colorStart = mColorStart;
    particle->colorEnd = mColorEnd;
    particle->color = mColorStart;
    particle->rotation = rnd(0, kTwoPi);
    particle->rotationSpeed = rnd(mRotSpeedMin, mRotSpeedMax);
    particle->animSpeed = 0.0f;
    particle->animOffset = 0.0f;
    particle->texRect = Math::Vec4(0, 0, 1, 1);
    if (mUseAtlas && !mAtlasFrames.empty())
    {
        particle->animOffset = rnd(mAnimOffsetMin, mAnimOffsetMax);
        particle->animSpeed = rnd(mAnimSpeedMin, mAnimSpeedMax);
        particle->texRect = atlasRect(particle->animOffset);
    }
    particle->active = true;
}

Math::Vec3 ParticleSystem3D::calcEmissionPosition()
{
    const Math::Mat4 world = getGlobalTransform();
    Math::Vec4 wp4 =
            world * Math::Vec4(mEmissionOffset.x, mEmissionOffset.y, mEmissionOffset.z, 1.f);
    Math::Vec3 wp(wp4.x, wp4.y, wp4.z);
    Math::Vec3 off(0, 0, 0);
    switch (mEmitterShape)
    {
        case EmitterShape::Point:
            break;
        case EmitterShape::Sphere:
        {
            Math::Vec3 dir = randomUnitVector();
            float r = cbrtf(mRandom.randf()) * mRadius;
            off = dir * r;
            break;
        }
        case EmitterShape::Box:
        {
            float x = rnd(-mBoxSize.x * .5f, mBoxSize.x * .5f);
            float y = rnd(-mBoxSize.y * .5f, mBoxSize.y * .5f);
            float z = rnd(-mBoxSize.z * .5f, mBoxSize.z * .5f);
            off = Math::Vec3(x, y, z);
            break;
        }
        case EmitterShape::Cone:
        case EmitterShape::Circle:
        {
            if (mRadius > 0.f)
            {
                float a = rnd(0, kTwoPi);
                float r = sqrtf(mRandom.randf()) * mRadius;
                off = axisFromColumn(world.col0) * (cosf(a) * r) +
                      axisFromColumn(world.col1) * (sinf(a) * r);
            }
            break;
        }
        case EmitterShape::Ring:
        {
            float a = rnd(0, kTwoPi);
            float r = rnd(mInnerRadius, mRadius);
            off = axisFromColumn(world.col0) * (cosf(a) * r) +
                  axisFromColumn(world.col1) * (sinf(a) * r);
            break;
        }
    }
    if (Math::Vec3::Dot(off, off) > 0.0001f)
    {
        Math::Vec4 o4 = world * Math::Vec4(off.x, off.y, off.z, 0.f);
        wp += Math::Vec3(o4.x, o4.y, o4.z);
    }
    return wp;
}

Math::Vec3 ParticleSystem3D::calcEmissionVelocity()
{
    float speed = rnd(mSpeedMin, mSpeedMax);
    Math::Vec4 d4 = getGlobalTransform() * Math::Vec4(mEmissionDirection.x, mEmissionDirection.y,
                                                   mEmissionDirection.z, 0.f);
    Math::Vec3 dir = Math::Vec3(d4.x, d4.y, d4.z).Normalized();
    if (mSpreadAngle > 0.f)
    {
        float theta = rnd(0, kTwoPi);
        float phi = rnd(0, mSpreadAngle);
        Math::Vec3 u(0, 1, 0);
        if (fabsf(Math::Vec3::Dot(dir, u)) > 0.99f) u = Math::Vec3(1, 0, 0);
        Math::Vec3 r2 = Math::Vec3::Cross(dir, u).Normalized();
        u = Math::Vec3::Cross(r2, dir).Normalized();
        dir = (dir * cosf(phi) + (r2 * cosf(theta) + u * sinf(theta)) * sinf(phi)).Normalized();
    }
    return dir * speed;
}

void ParticleSystem3D::updateParticles(float dt)
{
    ParticleSystem3D* subEmitter = findSubEmitter();
    for (std::size_t i = 0; i < mParticles.size(); ++i)
    {
        Particle& p = mParticles[i];
        if (!p.active) continue;
        const bool justSpawned = p.timeAlive == 0.0f;
        bool touched = false;
        p.timeAlive += dt;
        if (p.timeAlive >= p.lifetime)
        {
            if (mSubEmitterMode == ParticleSubEmitterMode::AtEnd)
                emitSubParticles(subEmitter, p, mSubEmitterAmountAtEnd);
            p.active = false;
            continue;
        }
        p.velocity += p.acceleration * dt;
        if (mDrag > 0.f) p.velocity = p.velocity * (1.f - mDrag * dt);
        if (mCollisionMode != ParticleCollisionMode::Disabled)
        {
            Math::Vec3 normal;
            float depth = 0.0f;
            if (findCollision(p, normal, depth))
            {
                touched = true;
                if (collided)
                {
                    Particle contact = p;
                    contact.position += normal * depth;
                    collided(contact, normal);
                }
                if (mCollisionMode == ParticleCollisionMode::HideOnContact)
                {
                    if (mSubEmitterMode == ParticleSubEmitterMode::AtCollision)
                        emitSubParticles(subEmitter, p, mSubEmitterAmountAtCollision);
                    p.active = false;
                    continue;
                }
                const float response = Math::Vec3::Dot(normal, p.velocity);
                const float edge = 2.0f / fminf(fmaxf(mCollisionBounce + 1.0f, 1.0f), 2.0f);
                const float bounceTrigger = fabsf(response) < edge ? 0.0f : 1.0f;
                p.position += normal * depth;
                p.velocity -= normal * response;
                p.velocity = p.velocity * (1.0f - fminf(fmaxf(mCollisionFriction, 0.0f), 1.0f));
                p.velocity -= normal * (response * mCollisionBounce * bounceTrigger);
            }
        }
        p.position += p.velocity * dt;
        p.rotation += p.rotationSpeed * dt;
        float t = p.timeAlive / p.lifetime;
        p.color = lerp4(p.colorStart, p.colorEnd, t);
        p.size = lerp2(p.sizeStart, p.sizeEnd, t);
        if (mUseAtlas && !mAtlasFrames.empty())
            p.texRect = atlasRect(p.animOffset + t * p.animSpeed);
        p.acceleration = mGravity;

        switch (mSubEmitterMode)
        {
            case ParticleSubEmitterMode::Constant:
                if (mSubEmitterFrequency > 0.0f)
                {
                    const float period = 1.0f / mSubEmitterFrequency;
                    const float from = p.timeAlive - dt;
                    const float remaining = period - (from - period * floorf(from / period));
                    if (dt >= remaining) emitSubParticles(subEmitter, p, 1);
                }
                break;
            case ParticleSubEmitterMode::AtCollision:
                if (touched) emitSubParticles(subEmitter, p, mSubEmitterAmountAtCollision);
                break;
            case ParticleSubEmitterMode::AtStart:
                if (justSpawned) emitSubParticles(subEmitter, p, mSubEmitterAmountAtStart);
                break;
            default:
                break;
        }
    }
}

ParticleSystem3D* ParticleSystem3D::findSubEmitter() const
{
    if (mSubEmitterMode == ParticleSubEmitterMode::Disabled || mSubEmitter.empty() ||
            !isInsideTree())
        return nullptr;
    Node* node = getNode(mSubEmitter);
    ParticleSystem3D* target = node ? node->castTo<ParticleSystem3D>() : nullptr;
    return target != this ? target : nullptr;
}

void ParticleSystem3D::emitSubParticles(ParticleSystem3D* target, const Particle& particle,
        int count) const
{
    if (!target) return;
    for (int i = 0; i < count; ++i)
    {
        if (mSubEmitterKeepVelocity) target->emitParticle(particle.position, particle.velocity);
        else
            target->emitParticle(particle.position);
    }
}

bool ParticleSystem3D::emitParticle(const Math::Vec3& position)
{
    Particle* particle = getFreeParticle();
    if (!particle) return false;
    initParticle(particle);
    particle->position = position;
    return true;
}

bool ParticleSystem3D::emitParticle(const Math::Vec3& position, const Math::Vec3& velocity)
{
    Particle* particle = getFreeParticle();
    if (!particle) return false;
    initParticle(particle);
    particle->position = position;
    particle->velocity = velocity;
    return true;
}

bool ParticleSystem3D::findCollision(const Particle& particle, Math::Vec3& normal,
        float& depth) const
{
    const SceneTree* tree = getTree();
    if (!tree) return false;

    float particleSize = mCollisionBaseSize;
    if (mCollisionUseScale) particleSize *= (particle.size.x + particle.size.y) * 0.5f;

    bool collided = false;
    const ct::Vector<ParticleCollision3D*>& colliders = tree->getParticleColliders();
    for (std::size_t i = 0; i < colliders.size(); ++i)
    {
        if (!colliders[i]->isVisibleInTree()) continue;
        Math::Vec3 colliderNormal;
        float colliderDepth = 0.0f;
        if (!colliders[i]->collide(particle.position, particleSize, colliderNormal, colliderDepth))
            continue;
        if (!collided)
        {
            collided = true;
            normal = colliderNormal;
            depth = colliderDepth;
            continue;
        }
        Math::Vec3 push = normal * depth;
        push += colliderNormal * fmaxf(0.0f, colliderDepth - Math::Vec3::Dot(colliderNormal, push));
        depth = push.Length();
        normal = depth > 0.0f ? push * (1.0f / depth) : colliderNormal;
    }
    return collided;
}

Math::Vec4 ParticleSystem3D::atlasRect(float phase) const
{
    const float total = static_cast<float>(mAtlasFrames.size());
    float frame = floorf(phase * total);
    if (mAnimLoop) frame = frame - total * floorf(frame / total);
    else
        frame = fminf(fmaxf(frame, 0.0f), total - 1.0f);
    return mAtlasFrames[static_cast<std::size_t>(frame)];
}

Math::Vec2 ParticleSystem3D::velocityScale(const Particle& particle) const
{
    if (!mScaleOverVelocity) return Math::Vec2(1.0f, 1.0f);
    const float range = mScaleVelocityMax - mScaleVelocityMin;
    float t = range != 0.0f ? particle.velocity.Length() / range : 0.0f;
    t = fminf(fmaxf(t, 0.0f), 1.0f);
    return lerp2(mScaleAtMinVelocity, mScaleAtMaxVelocity, t);
}

void ParticleSystem3D::applyAffectors(float dt)
{
    if (mAffectors.empty()) return;
    for (std::size_t i = 0; i < mParticles.size(); ++i)
    {
        Particle& p = mParticles[i];
        if (!p.active) continue;
        for (std::size_t a = 0; a < mAffectors.size(); ++a)
            if (mAffectors[a] && mAffectors[a]->enabled) mAffectors[a]->apply(p, dt);
    }
}

float ParticleSystem3D::rnd(float a, float b)
{
    return a + mRandom.randf() * (b - a);
}

Math::Vec3 ParticleSystem3D::randomUnitVector()
{
    float z = rnd(-1, 1);
    float a = rnd(0, kTwoPi);
    float r = sqrtf(1 - z * z);
    return Math::Vec3(r * cosf(a), r * sinf(a), z);
}

void ParticleSystem3D::update(double delta)
{
    simulate(static_cast<float>(delta));
    buildGeometry();
    uploadGeometry();
}

const SurfaceData& ParticleSystem3D::getGeometry() const
{
    return mGeometry;
}

void ParticleSystem3D::buildGeometry()
{
    mGeometry.positions.clear();
    mGeometry.normals.clear();
    mGeometry.colors.clear();
    mGeometry.uvs.clear();
    mGeometry.indices.clear();

    Math::Vec3 camRight(1.0f, 0.0f, 0.0f);
    Math::Vec3 camUp(0.0f, 1.0f, 0.0f);
    Math::Vec3 camBack(0.0f, 0.0f, 1.0f);
    {
        Viewport* viewport = Viewport::findViewport(this);
        Camera3D* camera = viewport ? viewport->getCamera3D() : nullptr;
        if (camera)
        {
            const Math::Mat4 cameraTransform = camera->getGlobalTransform();
            camRight = axisFromColumn(cameraTransform.col0);
            camUp = axisFromColumn(cameraTransform.col1);
            camBack = axisFromColumn(cameraTransform.col2);
        }
    }

    int q = 0;
    for (std::size_t i = 0; i < mParticles.size(); ++i)
    {
        const Particle& p = mParticles[i];
        if (!p.active || q >= mMaxParticles) continue;
        Math::Vec3 pr = camRight;
        Math::Vec3 pu = camUp;
        if (mAlign == ParticleAlign::YToVelocity)
        {
            Math::Vec3 screenVelocity = p.velocity - camBack * Math::Vec3::Dot(camBack, p.velocity);
            if (screenVelocity.Length() == 0.0f) screenVelocity = camUp;
            pu = screenVelocity.Normalized();
            pr = Math::Vec3::Cross(pu, camBack).Normalized();
        }
        else if (p.rotation != 0.f)
        {
            float c = cosf(p.rotation);
            float s = sinf(p.rotation);
            Math::Vec3 nr = pr * c - pu * s;
            Math::Vec3 nu = pr * s + pu * c;
            pr = nr;
            pu = nu;
        }
        const Math::Vec2 scale = velocityScale(p);
        Math::Vec3 hr = pr * (p.size.x * scale.x * .5f);
        Math::Vec3 hu = pu * (p.size.y * scale.y * .5f);
        float u0 = p.texRect.x;
        float v0 = p.texRect.y;
        float u1 = p.texRect.x + p.texRect.z;
        float v1 = p.texRect.y + p.texRect.w;

        mGeometry.positions.push_back(p.position - hr - hu);
        mGeometry.positions.push_back(p.position + hr - hu);
        mGeometry.positions.push_back(p.position + hr + hu);
        mGeometry.positions.push_back(p.position - hr + hu);
        mGeometry.uvs.push_back(Math::Vec2(u0, v0));
        mGeometry.uvs.push_back(Math::Vec2(u1, v0));
        mGeometry.uvs.push_back(Math::Vec2(u1, v1));
        mGeometry.uvs.push_back(Math::Vec2(u0, v1));
        for (int corner = 0; corner < 4; ++corner) mGeometry.colors.push_back(p.color);

        const std::uint32_t base = static_cast<std::uint32_t>(q * 4);
        mGeometry.indices.push_back(base);
        mGeometry.indices.push_back(base + 1);
        mGeometry.indices.push_back(base + 2);
        mGeometry.indices.push_back(base);
        mGeometry.indices.push_back(base + 2);
        mGeometry.indices.push_back(base + 3);
        ++q;
    }

    if (!mGeometry.positions.empty())
    {
        const Math::Mat4 inverse = getGlobalTransform().InverseAffine();
        const Math::Vec3 normal =
                inverse.TransformVector(Math::Vec3::Cross(camRight, camUp).Normalized())
                        .Normalized();
        for (std::size_t i = 0; i < mGeometry.positions.size(); ++i)
        {
            mGeometry.positions[i] = inverse.TransformPoint(mGeometry.positions[i]);
            mGeometry.normals.push_back(normal);
        }
    }
}

void ParticleSystem3D::uploadGeometry()
{
    MeshStore* store = MeshStore::singleton();
    if (!store || mMaxParticles <= 0) return;

    if (!store->valid(mMesh))
    {
        mMesh = store->create(kMeshName);
        mSurfaceReady = false;
        setBase(mMesh);
    }

    if (!mSurfaceReady)
    {
        SurfaceLayout layout;
        layout.attributeMask = kVertexPosition | kVertexNormal | kVertexColor | kVertexUv;
        layout.vertexCount = static_cast<std::uint32_t>(mMaxParticles) * 4u;
        layout.indexCount = static_cast<std::uint32_t>(mMaxParticles) * 6u;
        mSurfaceReady = store->addDynamicSurface(mMesh, layout, kMeshName) >= 0;
        mUploadedEmpty = true;
    }

    const bool empty = mGeometry.positions.empty();
    if (mSurfaceReady && !(empty && mUploadedEmpty))
    {
        store->updateSurface(mMesh, 0, mGeometry);
        mUploadedEmpty = empty;
    }
}

void ParticleSystem3D::applyMaterial()
{
    MaterialStore* materials = MaterialStore::singleton();
    if (materials && mDefaultMaterial.valid())
    {
        switch (mBlend)
        {
            case ParticleBlendMode::Alpha:
                materials->setBlendMode(mDefaultMaterial, MaterialBlend::Mix);
                materials->setTransparency(mDefaultMaterial, MaterialTransparency::AlphaBlend);
                break;
            case ParticleBlendMode::Additive:
                materials->setBlendMode(mDefaultMaterial, MaterialBlend::Add);
                materials->setTransparency(mDefaultMaterial, MaterialTransparency::Opaque);
                break;
            case ParticleBlendMode::Screen:
                materials->setBlendMode(mDefaultMaterial, MaterialBlend::Screen);
                materials->setTransparency(mDefaultMaterial, MaterialTransparency::Opaque);
                break;
            case ParticleBlendMode::Multiply:
                materials->setBlendMode(mDefaultMaterial, MaterialBlend::Mul);
                materials->setTransparency(mDefaultMaterial, MaterialTransparency::Opaque);
                break;
        }
        materials->setTexture(mDefaultMaterial, MaterialTexture::Albedo, mTexture);
    }

    RenderingServer* server = RenderingServer::singleton();
    if (server) server->instanceSetMaterialOverride(getInstance(), mDefaultMaterial);
}

void ParticleSystem3D::notificationHandler(int what)
{
    VisualInstance3D::notificationHandler(what);

    switch (what)
    {
        case kNotificationEnterTree:
            connectPreDraw();
            break;
        case kNotificationExitTree:
            disconnectPreDraw();
            break;
        case kNotificationReady:
            play();
            break;
        default:
            break;
    }
}

void ParticleSystem3D::connectPreDraw()
{
    RenderingServer* server = RenderingServer::singleton();
    if (!server) return;
    server->framePreDrawConnect(this,
            [this]()
            {
                SceneTree* tree = getTree();
                update(tree ? tree->getProcessDeltaTime() : 0.0);
            });
}

void ParticleSystem3D::disconnectPreDraw()
{
    RenderingServer* server = RenderingServer::singleton();
    if (server) server->framePreDrawDisconnect(this);
}

namespace
{

void applyShape(ParticleSystem3D* system, EmitterShape shape)
{
    switch (shape)
    {
        case EmitterShape::Point:
            system->setShapePoint();
            break;
        case EmitterShape::Sphere:
            system->setShapeSphere(system->getShapeRadius());
            break;
        case EmitterShape::Box:
            system->setShapeBox(system->getShapeBoxSize());
            break;
        case EmitterShape::Cone:
            system->setShapeCone(system->getShapeConeAngleDeg(), system->getShapeRadius());
            break;
        case EmitterShape::Circle:
            system->setShapeCircle(system->getShapeRadius());
            break;
        case EmitterShape::Ring:
            system->setShapeRing(system->getShapeRadius(), system->getShapeInnerRadius());
            break;
    }
}

} // namespace

void ParticleSystem3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = ParticleSystem3D;
    auto real = [](const char* name, double minimum, double maximum, double step)
    { return PropertyInfo::range(PropertyType::Float, name, minimum, maximum, step, true); };
    auto realSigned = [](const char* name, double minimum, double maximum, double step)
    { return PropertyInfo::range(PropertyType::Float, name, minimum, maximum, step, true, true); };
    auto whole = [](const char* name, double minimum, double maximum)
    { return PropertyInfo::range(PropertyType::Int, name, minimum, maximum, 1.0, true); };

    resourceprop::add<P, TextureId>(PropertyInfo::resource("texture", "Texture2D"), &P::setTexture,
            &P::getTexture);
    DB::addProperty<P>(whole("max_particles", 0.0, 1000000.0), &P::setMaxParticles,
            &P::getMaxParticles);
    DB::addProperty<P>(PropertyInfo::enumeration("blend_mode", "Alpha,Additive,Multiply,Screen"),
            &P::setBlendMode, &P::getBlendMode);
    DB::addProperty<P>(PropertyInfo::enumeration("align", "Billboard,Y To Velocity"), &P::setAlign,
            &P::getAlign);

    DB::addPropertyGroup<P>("Emission", "emission_");
    DB::addProperty<P>(
            PropertyInfo::enumeration("emission_mode", "Continuous,Burst,One Shot,Pulse"),
            &P::setEmissionMode, &P::getEmissionMode);
    DB::addProperty<P>(
            real("emission_rate", 0.0, 100000.0, 0.01),
            [](P* system, const PropertyValue& value)
            {
                const EmissionMode mode = system->getEmissionMode();
                system->setContinuous(static_cast<float>(value.asFloat()));
                system->setEmissionMode(mode);
            },
            &P::getEmissionRate);
    DB::addProperty<P>(
            whole("burst_count", 0.0, 100000.0),
            [](P* system, const PropertyValue& value)
            {
                const EmissionMode mode = system->getEmissionMode();
                system->setBurst(static_cast<int>(value.asInt()), system->getBurstInterval());
                system->setEmissionMode(mode);
            },
            &P::getBurstCount);
    DB::addProperty<P>(
            real("burst_interval", 0.0, 3600.0, 0.01),
            [](P* system, const PropertyValue& value)
            {
                const EmissionMode mode = system->getEmissionMode();
                system->setBurst(system->getBurstCount(), static_cast<float>(value.asFloat()));
                system->setEmissionMode(mode);
            },
            &P::getBurstInterval);
    DB::addProperty<P>(
            whole("one_shot_count", 0.0, 100000.0),
            [](P* system, const PropertyValue& value)
            {
                const EmissionMode mode = system->getEmissionMode();
                system->setOneShot(static_cast<int>(value.asInt()));
                system->setEmissionMode(mode);
            },
            &P::getOneShotCount);
    DB::addProperty<P>(
            real("pulse_rate", 0.0, 100000.0, 0.01),
            [](P* system, const PropertyValue& value)
            {
                const EmissionMode mode = system->getEmissionMode();
                system->setPulse(static_cast<float>(value.asFloat()),
                        system->getParticlesPerPulse());
                system->setEmissionMode(mode);
            },
            &P::getPulseRate);
    DB::addProperty<P>(
            whole("particles_per_pulse", 0.0, 100000.0),
            [](P* system, const PropertyValue& value)
            {
                const EmissionMode mode = system->getEmissionMode();
                system->setPulse(system->getPulseRate(), static_cast<int>(value.asInt()));
                system->setEmissionMode(mode);
            },
            &P::getParticlesPerPulse);
    DB::addProperty<P>(PropertyInfo(PropertyType::Vec3, "emission_offset"), &P::setEmissionOffset,
            &P::getEmissionOffset);
    DB::addProperty<P>(PropertyInfo(PropertyType::Vec3, "emission_direction"),
            &P::setEmissionDirection, &P::getEmissionDirection);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "emission_spread_angle", 0.0, 180.0, 0.01),
            &P::setSpreadAngle, &P::getSpreadAngleDeg);
    DB::addProperty<P>(realSigned("duration", -1.0, 3600.0, 0.01), &P::setDuration,
            &P::getDuration);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "loop"), &P::setLoop, &P::getLoop);

    DB::addPropertyGroup<P>("Shape", "shape_");
    DB::addProperty<P>(
            PropertyInfo::enumeration("shape_type", "Point,Sphere,Box,Cone,Circle,Ring"),
            [](P* system, const PropertyValue& value)
            { applyShape(system, static_cast<EmitterShape>(value.asInt())); }, &P::getEmitterShape);
    DB::addProperty<P>(
            real("shape_radius", 0.0, 10000.0, 0.001),
            [](P* system, const PropertyValue& value)
            {
                const EmitterShape shape = system->getEmitterShape();
                system->setShapeCircle(static_cast<float>(value.asFloat()));
                applyShape(system, shape);
            },
            &P::getShapeRadius);
    DB::addProperty<P>(
            real("shape_inner_radius", 0.0, 10000.0, 0.001),
            [](P* system, const PropertyValue& value)
            {
                const EmitterShape shape = system->getEmitterShape();
                system->setShapeRing(system->getShapeRadius(), static_cast<float>(value.asFloat()));
                applyShape(system, shape);
            },
            &P::getShapeInnerRadius);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "shape_cone_angle", 0.0, 180.0, 0.01),
            [](P* system, const PropertyValue& value)
            {
                const EmitterShape shape = system->getEmitterShape();
                system->setShapeCone(static_cast<float>(value.asFloat()), system->getShapeRadius());
                applyShape(system, shape);
            },
            &P::getShapeConeAngleDeg);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Vec3, "shape_box_size", 0.0, 10000.0, 0.001, true),
            [](P* system, const PropertyValue& value)
            {
                const EmitterShape shape = system->getEmitterShape();
                system->setShapeBox(PropertyConvert<Math::Vec3>::from(value));
                applyShape(system, shape);
            },
            &P::getShapeBoxSize);

    DB::addPropertyGroup<P>("Particle", "particle_");
    DB::addProperty<P>(
            real("particle_lifetime_min", 0.0, 3600.0, 0.01),
            [](P* system, const PropertyValue& value)
            { system->setLifetime(static_cast<float>(value.asFloat()), system->getLifetimeMax()); },
            &P::getLifetimeMin);
    DB::addProperty<P>(
            real("particle_lifetime_max", 0.0, 3600.0, 0.01),
            [](P* system, const PropertyValue& value)
            { system->setLifetime(system->getLifetimeMin(), static_cast<float>(value.asFloat())); },
            &P::getLifetimeMax);
    DB::addProperty<P>(
            realSigned("particle_speed_min", -10000.0, 10000.0, 0.01),
            [](P* system, const PropertyValue& value)
            { system->setSpeed(static_cast<float>(value.asFloat()), system->getSpeedMax()); },
            &P::getSpeedMin);
    DB::addProperty<P>(
            realSigned("particle_speed_max", -10000.0, 10000.0, 0.01),
            [](P* system, const PropertyValue& value)
            { system->setSpeed(system->getSpeedMin(), static_cast<float>(value.asFloat())); },
            &P::getSpeedMax);
    DB::addProperty<P>(
            PropertyInfo(PropertyType::Vec2, "particle_size_start"),
            [](P* system, const PropertyValue& value)
            { system->setSize(value.asVec2(), system->getSizeEnd()); }, &P::getSizeStart);
    DB::addProperty<P>(
            PropertyInfo(PropertyType::Vec2, "particle_size_end"),
            [](P* system, const PropertyValue& value)
            { system->setSize(system->getSizeStart(), value.asVec2()); }, &P::getSizeEnd);
    DB::addProperty<P>(
            PropertyInfo::color("particle_color_start"), [](P* system, const PropertyValue& value)
            { system->setColor(value.asColor(), system->getColorEnd()); }, &P::getColorStart);
    DB::addProperty<P>(
            PropertyInfo::color("particle_color_end"), [](P* system, const PropertyValue& value)
            { system->setColor(system->getColorStart(), value.asColor()); }, &P::getColorEnd);
    DB::addProperty<P>(
            realSigned("particle_rotation_speed_min", -100000.0, 100000.0, 0.01),
            [](P* system, const PropertyValue& value) {
                system->setRotationSpeed(static_cast<float>(value.asFloat()),
                        system->getRotSpeedMax());
            },
            &P::getRotSpeedMin);
    DB::addProperty<P>(
            realSigned("particle_rotation_speed_max", -100000.0, 100000.0, 0.01),
            [](P* system, const PropertyValue& value) {
                system->setRotationSpeed(system->getRotSpeedMin(),
                        static_cast<float>(value.asFloat()));
            },
            &P::getRotSpeedMax);

    DB::addPropertyGroup<P>("Scale Over Velocity", "scale_velocity_");
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "scale_velocity_enabled"),
            &P::setScaleOverVelocityEnabled, &P::hasScaleOverVelocity);
    DB::addProperty<P>(realSigned("scale_velocity_min", -10000.0, 10000.0, 0.01),
            &P::setScaleVelocityMin, &P::getScaleVelocityMin);
    DB::addProperty<P>(realSigned("scale_velocity_max", -10000.0, 10000.0, 0.01),
            &P::setScaleVelocityMax, &P::getScaleVelocityMax);
    DB::addProperty<P>(PropertyInfo(PropertyType::Vec2, "scale_velocity_at_min"),
            &P::setScaleAtMinVelocity, &P::getScaleAtMinVelocity);
    DB::addProperty<P>(PropertyInfo(PropertyType::Vec2, "scale_velocity_at_max"),
            &P::setScaleAtMaxVelocity, &P::getScaleAtMaxVelocity);

    DB::addPropertyGroup<P>("Physics", "");
    DB::addProperty<P>(PropertyInfo(PropertyType::Vec3, "gravity"), &P::setGravity, &P::getGravity);
    DB::addProperty<P>(realSigned("drag", -100.0, 100.0, 0.001), &P::setDrag, &P::getDrag);

    DB::addPropertyGroup<P>("Collision", "collision_");
    DB::addProperty<P>(
            PropertyInfo::enumeration("collision_mode", "Disabled,Rigid,Hide On Contact"),
            &P::setCollisionMode, &P::getCollisionMode);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "collision_friction", 0.0, 1.0, 0.01, true),
            &P::setCollisionFriction, &P::getCollisionFriction);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "collision_bounce", 0.0, 1.0, 0.01, true),
            &P::setCollisionBounce, &P::getCollisionBounce);
    DB::addProperty<P>(real("collision_base_size", 0.0, 100.0, 0.001), &P::setCollisionBaseSize,
            &P::getCollisionBaseSize);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "collision_use_scale"),
            &P::setCollisionUseScale, &P::getCollisionUseScale);

    DB::addPropertyGroup<P>("Sub Emitter", "sub_emitter_");
    DB::addProperty<P>(PropertyInfo::nodePath("sub_emitter", "ParticleSystem3D"), &P::setSubEmitter,
            &P::getSubEmitter);
    DB::addProperty<P>(PropertyInfo::enumeration("sub_emitter_mode",
                               "Disabled,Constant,At End,At Collision,At Start"),
            &P::setSubEmitterMode, &P::getSubEmitterMode);
    DB::addProperty<P>(real("sub_emitter_frequency", 0.0, 100000.0, 0.01),
            &P::setSubEmitterFrequency, &P::getSubEmitterFrequency);
    DB::addProperty<P>(whole("sub_emitter_amount_at_end", 0.0, 100000.0),
            &P::setSubEmitterAmountAtEnd, &P::getSubEmitterAmountAtEnd);
    DB::addProperty<P>(whole("sub_emitter_amount_at_collision", 0.0, 100000.0),
            &P::setSubEmitterAmountAtCollision, &P::getSubEmitterAmountAtCollision);
    DB::addProperty<P>(whole("sub_emitter_amount_at_start", 0.0, 100000.0),
            &P::setSubEmitterAmountAtStart, &P::getSubEmitterAmountAtStart);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "sub_emitter_keep_velocity"),
            &P::setSubEmitterKeepVelocity, &P::getSubEmitterKeepVelocity);

    DB::addPropertyGroup<P>("Atlas", "atlas_");
    DB::addProperty<P>(
            PropertyInfo(PropertyType::Bool, "atlas_enabled"),
            [](P* system, const PropertyValue& value)
            {
                if (value.asBool())
                    system->setAtlasGrid(system->getAtlasCols(), system->getAtlasRows());
                else
                    system->clearAtlas();
            },
            &P::isUsingAtlas);
    DB::addProperty<P>(
            whole("atlas_columns", 1.0, 1024.0),
            [](P* system, const PropertyValue& value)
            {
                const bool used = system->isUsingAtlas();
                const int columns = static_cast<int>(value.asInt());
                system->setAtlasGrid(columns < 1 ? 1 : columns, system->getAtlasRows());
                if (!used) system->clearAtlas();
            },
            &P::getAtlasCols);
    DB::addProperty<P>(
            whole("atlas_rows", 1.0, 1024.0),
            [](P* system, const PropertyValue& value)
            {
                const bool used = system->isUsingAtlas();
                const int rows = static_cast<int>(value.asInt());
                system->setAtlasGrid(system->getAtlasCols(), rows < 1 ? 1 : rows);
                if (!used) system->clearAtlas();
            },
            &P::getAtlasRows);
    DB::addProperty<P>(
            realSigned("atlas_anim_speed_min", -1000.0, 1000.0, 0.01),
            [](P* system, const PropertyValue& value) {
                system->setAnimSpeed(static_cast<float>(value.asFloat()),
                        system->getAnimSpeedMax());
            },
            &P::getAnimSpeedMin);
    DB::addProperty<P>(
            realSigned("atlas_anim_speed_max", -1000.0, 1000.0, 0.01),
            [](P* system, const PropertyValue& value) {
                system->setAnimSpeed(system->getAnimSpeedMin(),
                        static_cast<float>(value.asFloat()));
            },
            &P::getAnimSpeedMax);
    DB::addProperty<P>(
            realSigned("atlas_anim_offset_min", -1000.0, 1000.0, 0.01),
            [](P* system, const PropertyValue& value) {
                system->setAnimOffset(static_cast<float>(value.asFloat()),
                        system->getAnimOffsetMax());
            },
            &P::getAnimOffsetMin);
    DB::addProperty<P>(
            realSigned("atlas_anim_offset_max", -1000.0, 1000.0, 0.01),
            [](P* system, const PropertyValue& value) {
                system->setAnimOffset(system->getAnimOffsetMin(),
                        static_cast<float>(value.asFloat()));
            },
            &P::getAnimOffsetMax);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "atlas_anim_loop"), &P::setAnimLoop,
            &P::getAnimLoop);
}

namespace
{

constexpr std::int64_t kParticleSystemDataVersion = 1;

const char* affectorName(ParticleAffectorType type)
{
    switch (type)
    {
        case ParticleAffectorType::Gravity:
            return "Gravity";
        case ParticleAffectorType::Drag:
            return "Drag";
        case ParticleAffectorType::Vortex:
            return "Vortex";
        case ParticleAffectorType::Attractor:
            return "Attractor";
        case ParticleAffectorType::Turbulence:
            return "Turbulence";
        case ParticleAffectorType::ColorOverLifetime:
            return "ColorOverLifetime";
        case ParticleAffectorType::SizeOverLifetime:
            return "SizeOverLifetime";
    }
    return "";
}

} // namespace

bool ParticleSystem3D::saveData(ct::Json& data, SceneResult& result, const ct::String& path) const
{
    (void) result;
    (void) path;
    if (mAffectors.empty()) return false;

    data["version"] = ct::Json(static_cast<long long>(kParticleSystemDataVersion));
    ct::Json affectors = ct::Json::array();
    for (std::size_t i = 0; i < mAffectors.size(); ++i)
    {
        const ParticleAffector* affector = mAffectors[i];
        ct::Json entry = ct::Json::object();
        entry["type"] = ct::Json(affectorName(affector->getType()));
        entry["enabled"] = ct::Json(affector->enabled);
        switch (affector->getType())
        {
            case ParticleAffectorType::Gravity:
                entry["gravity"] =
                        scenejson::fromVec3(static_cast<const GravityAffector*>(affector)->gravity);
                break;
            case ParticleAffectorType::Drag:
                entry["drag"] =
                        scenejson::fromFloat(static_cast<const DragAffector*>(affector)->drag);
                break;
            case ParticleAffectorType::Vortex:
            {
                const VortexAffector* vortex = static_cast<const VortexAffector*>(affector);
                entry["center"] = scenejson::fromVec3(vortex->center);
                entry["strength"] = scenejson::fromFloat(vortex->strength);
                entry["radius"] = scenejson::fromFloat(vortex->radius);
                break;
            }
            case ParticleAffectorType::Attractor:
            {
                const AttractorAffector* attractor =
                        static_cast<const AttractorAffector*>(affector);
                entry["position"] = scenejson::fromVec3(attractor->position);
                entry["strength"] = scenejson::fromFloat(attractor->strength);
                entry["radius"] = scenejson::fromFloat(attractor->radius);
                entry["repulse"] = ct::Json(attractor->repulse);
                break;
            }
            case ParticleAffectorType::Turbulence:
            {
                const TurbulenceAffector* turbulence =
                        static_cast<const TurbulenceAffector*>(affector);
                entry["strength"] = scenejson::fromFloat(turbulence->strength);
                entry["frequency"] = scenejson::fromFloat(turbulence->frequency);
                break;
            }
            case ParticleAffectorType::ColorOverLifetime:
            {
                const ColorOverLifetimeAffector* color =
                        static_cast<const ColorOverLifetimeAffector*>(affector);
                entry["start"] = scenejson::fromVec4(color->startColor);
                entry["end"] = scenejson::fromVec4(color->endColor);
                break;
            }
            case ParticleAffectorType::SizeOverLifetime:
            {
                const SizeOverLifetimeAffector* size =
                        static_cast<const SizeOverLifetimeAffector*>(affector);
                entry["start"] = scenejson::fromVec2(size->startSize);
                entry["end"] = scenejson::fromVec2(size->endSize);
                break;
            }
        }
        affectors.push_back(entry);
    }
    data["affectors"] = affectors;
    return true;
}

bool ParticleSystem3D::loadData(const ct::Json& data, SceneResult& result, const ct::String& path)
{
    if (!scenejson::checkVersion(data, kParticleSystemDataVersion, result, path)) return false;

    clearAffectors();
    const ct::Json* affectors = scenejson::arrayField(data, "affectors", result, path);
    if (!affectors) return true;

    for (std::size_t i = 0; i < affectors->size(); ++i)
    {
        const ct::Json& entry = affectors->at(i);
        const ct::String entryPath = scenejson::element(path, "affectors", i);
        if (!entry.is_object())
        {
            result.addError(entryPath, "expected an object");
            continue;
        }
        const ct::String type = scenejson::getString(entry, "type", ct::String());
        const std::size_t before = mAffectors.size();
        Math::Vec3 vector(0.0f, 0.0f, 0.0f);
        if (type == "Gravity")
        {
            if (scenejson::readVec3Field(entry, "gravity", vector, result, entryPath))
                addGravity(vector);
            else if (!entry.contains("gravity"))
                result.addError(entryPath, "missing gravity");
        }
        else if (type == "Drag")
        {
            addDrag(scenejson::getFloat(entry, "drag", 0.0f));
        }
        else if (type == "Vortex")
        {
            if (scenejson::readVec3Field(entry, "center", vector, result, entryPath))
                addVortex(vector, scenejson::getFloat(entry, "strength", 0.0f),
                        scenejson::getFloat(entry, "radius", 1.0f));
            else if (!entry.contains("center"))
                result.addError(entryPath, "missing center");
        }
        else if (type == "Attractor")
        {
            if (scenejson::readVec3Field(entry, "position", vector, result, entryPath))
                addAttractor(vector, scenejson::getFloat(entry, "strength", 0.0f),
                        scenejson::getFloat(entry, "radius", 1.0f),
                        scenejson::getBool(entry, "repulse", false));
            else if (!entry.contains("position"))
                result.addError(entryPath, "missing position");
        }
        else if (type == "Turbulence")
        {
            addTurbulence(scenejson::getFloat(entry, "strength", 0.0f),
                    scenejson::getFloat(entry, "frequency", 1.0f));
        }
        else if (type == "ColorOverLifetime")
        {
            Math::Vec4 start(1.0f, 1.0f, 1.0f, 1.0f);
            Math::Vec4 end(1.0f, 1.0f, 1.0f, 0.0f);
            scenejson::readVec4Field(entry, "start", start, result, entryPath);
            scenejson::readVec4Field(entry, "end", end, result, entryPath);
            addColorOverLifetime(start, end);
        }
        else if (type == "SizeOverLifetime")
        {
            Math::Vec2 start(1.0f, 1.0f);
            Math::Vec2 end(0.5f, 0.5f);
            const ct::Json* startField = entry.find("start");
            const ct::Json* endField = entry.find("end");
            if ((startField && !scenejson::toVec2(*startField, start)) ||
                    (endField && !scenejson::toVec2(*endField, end)))
                result.addError(entryPath, "expected [x, y] for start and end");
            addSizeOverLifetime(start, end);
        }
        else
        {
            result.addError(scenejson::field(entryPath, "type"),
                    ct::String("unknown affector '") + type + "'");
            continue;
        }
        if (mAffectors.size() > before)
            mAffectors.back()->enabled = scenejson::getBool(entry, "enabled", true);
    }
    return true;
}

} // namespace prisma3d
