#pragma once

#include "prisma3d/core/Random.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/TextureStore.h"
#include "prisma3d/scene/VisualInstance3D.h"

#include <ct/function.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

namespace prisma3d
{

enum class ParticleBlendMode
{
    Alpha,
    Additive,
    Multiply,
    Screen
};

enum class EmissionMode
{
    Continuous,
    Burst,
    OneShot,
    Pulse
};

enum class EmitterShape
{
    Point,
    Sphere,
    Box,
    Cone,
    Circle,
    Ring
};

enum class ParticleAlign
{
    Billboard,
    YToVelocity
};

enum class ParticleCollisionMode
{
    Disabled,
    Rigid,
    HideOnContact
};

enum class ParticleSubEmitterMode
{
    Disabled,
    Constant,
    AtEnd,
    AtCollision,
    AtStart
};

enum class EmitterState
{
    Stopped,
    Playing,
    Paused
};

struct Particle
{
    Math::Vec3 position = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Vec3 velocity = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Vec3 acceleration = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Vec4 color = Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f);
    Math::Vec2 size = Math::Vec2(1.0f, 1.0f);
    float rotation = 0.0f;
    float rotationSpeed = 0.0f;
    Math::Vec4 texRect = Math::Vec4(0.0f, 0.0f, 1.0f, 1.0f);
    float lifetime = 1.0f;
    float timeAlive = 0.0f;
    bool active = false;
    Math::Vec4 colorStart = Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f);
    Math::Vec4 colorEnd = Math::Vec4(1.0f, 1.0f, 1.0f, 0.0f);
    Math::Vec2 sizeStart = Math::Vec2(1.0f, 1.0f);
    Math::Vec2 sizeEnd = Math::Vec2(0.5f, 0.5f);
    float animSpeed = 0.0f;
    float animOffset = 0.0f;
};

enum class ParticleAffectorType : std::uint8_t
{
    Gravity,
    Drag,
    Vortex,
    Attractor,
    Turbulence,
    ColorOverLifetime,
    SizeOverLifetime
};

class ParticleAffector
{
public:
    bool enabled = true;

    virtual ~ParticleAffector() = default;
    virtual ParticleAffectorType getType() const = 0;
    virtual void apply(Particle& particle, float dt) = 0;
};

class GravityAffector : public ParticleAffector
{
public:
    ParticleAffectorType getType() const override
    {
        return ParticleAffectorType::Gravity;
    }

    Math::Vec3 gravity;

    explicit GravityAffector(const Math::Vec3& g)
            : gravity(g)
    {
    }

    void apply(Particle& particle, float dt) override;
};

class DragAffector : public ParticleAffector
{
public:
    ParticleAffectorType getType() const override
    {
        return ParticleAffectorType::Drag;
    }

    float drag;

    explicit DragAffector(float d)
            : drag(d)
    {
    }

    void apply(Particle& particle, float dt) override;
};

class VortexAffector : public ParticleAffector
{
public:
    ParticleAffectorType getType() const override
    {
        return ParticleAffectorType::Vortex;
    }

    Math::Vec3 center;
    float strength;
    float radius;

    VortexAffector(const Math::Vec3& c, float s, float r)
            : center(c),
              strength(s),
              radius(r)
    {
    }

    void apply(Particle& particle, float dt) override;
};

class AttractorAffector : public ParticleAffector
{
public:
    ParticleAffectorType getType() const override
    {
        return ParticleAffectorType::Attractor;
    }

    Math::Vec3 position;
    float strength;
    float radius;
    bool repulse;

    AttractorAffector(const Math::Vec3& p, float s, float r, bool rep = false)
            : position(p),
              strength(s),
              radius(r),
              repulse(rep)
    {
    }

    void apply(Particle& particle, float dt) override;
};

class TurbulenceAffector : public ParticleAffector
{
public:
    ParticleAffectorType getType() const override
    {
        return ParticleAffectorType::Turbulence;
    }

    float strength;
    float frequency;
    float time = 0.0f;

    TurbulenceAffector(float s, float f)
            : strength(s),
              frequency(f)
    {
    }

    void apply(Particle& particle, float dt) override;
};

class ColorOverLifetimeAffector : public ParticleAffector
{
public:
    ParticleAffectorType getType() const override
    {
        return ParticleAffectorType::ColorOverLifetime;
    }

    Math::Vec4 startColor;
    Math::Vec4 endColor;

    ColorOverLifetimeAffector(const Math::Vec4& s, const Math::Vec4& e)
            : startColor(s),
              endColor(e)
    {
    }

    void apply(Particle& particle, float dt) override;
};

class SizeOverLifetimeAffector : public ParticleAffector
{
public:
    ParticleAffectorType getType() const override
    {
        return ParticleAffectorType::SizeOverLifetime;
    }

    Math::Vec2 startSize;
    Math::Vec2 endSize;

    SizeOverLifetimeAffector(const Math::Vec2& s, const Math::Vec2& e)
            : startSize(s),
              endSize(e)
    {
    }

    void apply(Particle& particle, float dt) override;
};

class ParticleSystem3D : public VisualInstance3D
{
public:
    PRISMA3D_NODE_CLASS(ParticleSystem3D, VisualInstance3D);
    static void bindProperties();

    bool saveData(ct::Json& data, SceneResult& result, const ct::String& path) const override;
    bool loadData(const ct::Json& data, SceneResult& result, const ct::String& path) override;

    ct::Function<void(const Particle&, const Math::Vec3&)> collided;

    explicit ParticleSystem3D(int maxParticles = 1000);
    ~ParticleSystem3D() override;

    void setTexture(TextureId texture);
    TextureId getTexture() const;
    void setBlendMode(ParticleBlendMode mode);
    ParticleBlendMode getBlendMode() const;

    ParticleSystem3D* setEmissionMode(EmissionMode mode);
    ParticleSystem3D* setContinuous(float particlesPerSecond);
    ParticleSystem3D* setBurst(int count, float interval = 1.0f);
    ParticleSystem3D* setOneShot(int count);
    ParticleSystem3D* setPulse(float particlesPerSecond, int perPulse);

    ParticleSystem3D* setShapePoint();
    ParticleSystem3D* setShapeSphere(float radius);
    ParticleSystem3D* setShapeBox(const Math::Vec3& size);
    ParticleSystem3D* setShapeCone(float degrees, float baseRadius = 0.0f);
    ParticleSystem3D* setShapeCircle(float radius);
    ParticleSystem3D* setShapeRing(float outer, float inner);

    ParticleSystem3D* setEmissionOffset(const Math::Vec3& offset);
    ParticleSystem3D* setEmissionDirection(const Math::Vec3& direction);
    ParticleSystem3D* setSpreadAngle(float degrees);

    ParticleSystem3D* setLifetime(float minimum, float maximum);
    ParticleSystem3D* setLifetime(float lifetime);
    ParticleSystem3D* setSpeed(float minimum, float maximum);
    ParticleSystem3D* setSpeed(float speed);
    ParticleSystem3D* setSize(const Math::Vec2& start, const Math::Vec2& end);
    ParticleSystem3D* setSize(float size);
    ParticleSystem3D* setColor(const Math::Vec4& start, const Math::Vec4& end);
    ParticleSystem3D* setColor(const Math::Vec4& color);
    ParticleSystem3D* setRotationSpeed(float minimum, float maximum);

    ParticleSystem3D* setGravity(const Math::Vec3& gravity);
    ParticleSystem3D* setDrag(float drag);
    ParticleSystem3D* setDuration(float seconds);
    ParticleSystem3D* setLoop(bool loop);

    ParticleSystem3D* setAlign(ParticleAlign align);
    ParticleAlign getAlign() const;
    ParticleSystem3D* setScaleOverVelocity(float minVelocity, float maxVelocity,
            const Math::Vec2& scaleAtMin, const Math::Vec2& scaleAtMax);
    void clearScaleOverVelocity();
    bool hasScaleOverVelocity() const;
    ParticleSystem3D* setScaleOverVelocityEnabled(bool enabled);
    ParticleSystem3D* setScaleVelocityMin(float velocity);
    float getScaleVelocityMin() const;
    ParticleSystem3D* setScaleVelocityMax(float velocity);
    float getScaleVelocityMax() const;
    ParticleSystem3D* setScaleAtMinVelocity(const Math::Vec2& scale);
    const Math::Vec2& getScaleAtMinVelocity() const;
    ParticleSystem3D* setScaleAtMaxVelocity(const Math::Vec2& scale);
    const Math::Vec2& getScaleAtMaxVelocity() const;

    ParticleSystem3D* setCollisionMode(ParticleCollisionMode mode);
    ParticleCollisionMode getCollisionMode() const;
    ParticleSystem3D* setCollisionFriction(float friction);
    float getCollisionFriction() const;
    ParticleSystem3D* setCollisionBounce(float bounce);
    float getCollisionBounce() const;
    ParticleSystem3D* setCollisionBaseSize(float size);
    float getCollisionBaseSize() const;
    ParticleSystem3D* setCollisionUseScale(bool useScale);
    bool getCollisionUseScale() const;

    ParticleSystem3D* setSubEmitter(const ct::String& path);
    const ct::String& getSubEmitter() const;
    ParticleSystem3D* setSubEmitterMode(ParticleSubEmitterMode mode);
    ParticleSubEmitterMode getSubEmitterMode() const;
    ParticleSystem3D* setSubEmitterFrequency(float frequency);
    float getSubEmitterFrequency() const;
    ParticleSystem3D* setSubEmitterAmountAtEnd(int amount);
    int getSubEmitterAmountAtEnd() const;
    ParticleSystem3D* setSubEmitterAmountAtCollision(int amount);
    int getSubEmitterAmountAtCollision() const;
    ParticleSystem3D* setSubEmitterAmountAtStart(int amount);
    int getSubEmitterAmountAtStart() const;
    ParticleSystem3D* setSubEmitterKeepVelocity(bool keepVelocity);
    bool getSubEmitterKeepVelocity() const;

    ParticleSystem3D* setAtlasGrid(int columns, int rows);
    void clearAtlas();
    ParticleSystem3D* setAnimSpeed(float minimum, float maximum);
    ParticleSystem3D* setAnimSpeed(float speed);
    ParticleSystem3D* setAnimOffset(float minimum, float maximum);
    ParticleSystem3D* setAnimOffset(float offset);
    ParticleSystem3D* setAnimLoop(bool loop);
    float getAnimSpeedMin() const;
    float getAnimSpeedMax() const;
    float getAnimOffsetMin() const;
    float getAnimOffsetMax() const;
    bool getAnimLoop() const;

    void play();
    void stop();
    void pause();
    void emitBurst(int count);
    bool emitParticle(const Math::Vec3& position);
    bool emitParticle(const Math::Vec3& position, const Math::Vec3& velocity);
    EmitterState getState() const;

    ParticleSystem3D* addGravity(const Math::Vec3& gravity);
    ParticleSystem3D* addDrag(float strength);
    ParticleSystem3D* addVortex(const Math::Vec3& center, float strength, float radius);
    ParticleSystem3D* addAttractor(const Math::Vec3& position, float strength, float radius,
            bool repulse = false);
    ParticleSystem3D* addTurbulence(float strength, float frequency);
    ParticleSystem3D* addColorOverLifetime(const Math::Vec4& start, const Math::Vec4& end);
    ParticleSystem3D* addSizeOverLifetime(const Math::Vec2& start, const Math::Vec2& end);
    void clearAffectors();
    std::size_t getAffectorCount() const;
    ParticleAffector* getAffector(std::size_t index) const;

    int getMaxParticles() const;
    void setMaxParticles(int maxParticles);
    int getActiveCount() const;
    const ct::Vector<Particle>& getParticles() const;
    Random& getRandom();
    EmissionMode getEmissionMode() const;
    float getEmissionRate() const;
    int getBurstCount() const;
    float getBurstInterval() const;
    int getOneShotCount() const;
    float getPulseRate() const;
    int getParticlesPerPulse() const;
    EmitterShape getEmitterShape() const;
    float getShapeRadius() const;
    float getShapeInnerRadius() const;
    float getShapeConeAngleDeg() const;
    const Math::Vec3& getShapeBoxSize() const;
    const Math::Vec3& getEmissionOffset() const;
    const Math::Vec3& getEmissionDirection() const;
    float getSpreadAngleDeg() const;
    float getLifetimeMin() const;
    float getLifetimeMax() const;
    float getSpeedMin() const;
    float getSpeedMax() const;
    const Math::Vec2& getSizeStart() const;
    const Math::Vec2& getSizeEnd() const;
    const Math::Vec4& getColorStart() const;
    const Math::Vec4& getColorEnd() const;
    float getRotSpeedMin() const;
    float getRotSpeedMax() const;
    const Math::Vec3& getGravity() const;
    float getDrag() const;
    float getDuration() const;
    bool getLoop() const;
    bool isUsingAtlas() const;
    int getAtlasCols() const;
    int getAtlasRows() const;

    void simulate(float dt);
    void update(double delta);
    const SurfaceData& getGeometry() const;

protected:
    void notificationHandler(int what) override;
    void validateProperty(PropertyInfo& info) const override;

private:
    void emitContinuous(float dt);
    void emitBurstMode(float dt);
    void emitPulseMode(float dt);
    Particle* getFreeParticle();
    void initParticle(Particle* particle);
    Math::Vec3 calcEmissionPosition();
    Math::Vec3 calcEmissionVelocity();
    void updateParticles(float dt);
    bool findCollision(const Particle& particle, Math::Vec3& normal, float& depth) const;
    Math::Vec2 velocityScale(const Particle& particle) const;
    Math::Vec4 atlasRect(float phase) const;
    ParticleSystem3D* findSubEmitter() const;
    void emitSubParticles(ParticleSystem3D* target, const Particle& particle, int count) const;
    void applyAffectors(float dt);
    float rnd(float a, float b);
    Math::Vec3 randomUnitVector();
    void buildGeometry();
    void uploadGeometry();
    void applyMaterial();
    void connectPreDraw();
    void disconnectPreDraw();

    ct::Vector<Particle> mParticles;
    ct::Vector<ParticleAffector*> mAffectors;
    int mMaxParticles;
    int mActiveCount;

    EmissionMode mEmissionMode;
    EmitterShape mEmitterShape;
    EmitterState mState;
    Math::Vec3 mEmissionOffset;
    Math::Vec3 mEmissionDirection;
    float mEmissionRate;
    float mBurstInterval;
    int mBurstCount;
    int mOneShotCount;
    float mPulseRate;
    int mParticlesPerPulse;
    ct::Vector<Math::Vec4> mAtlasFrames;
    int mAtlasCols;
    int mAtlasRows;
    bool mUseAtlas;
    float mAnimSpeedMin;
    float mAnimSpeedMax;
    float mAnimOffsetMin;
    float mAnimOffsetMax;
    bool mAnimLoop;
    float mRadius;
    float mInnerRadius;
    float mConeAngle;
    float mConeAngleDeg;
    Math::Vec3 mBoxSize;
    float mLifetimeMin;
    float mLifetimeMax;
    float mSpeedMin;
    float mSpeedMax;
    Math::Vec2 mSizeStart;
    Math::Vec2 mSizeEnd;
    Math::Vec4 mColorStart;
    Math::Vec4 mColorEnd;
    float mRotSpeedMin;
    float mRotSpeedMax;
    float mSpreadAngle;
    float mSpreadAngleDeg;
    Math::Vec3 mGravity;
    float mDrag;
    float mDuration;
    bool mLoop;
    float mEmissionTimer;
    float mTimeAlive;
    float mBurstTimer;
    float mPulseTimer;
    bool mHasEmittedOneShot;
    ParticleAlign mAlign;
    bool mScaleOverVelocity;
    float mScaleVelocityMin;
    float mScaleVelocityMax;
    Math::Vec2 mScaleAtMinVelocity;
    Math::Vec2 mScaleAtMaxVelocity;
    ParticleCollisionMode mCollisionMode;
    float mCollisionFriction;
    float mCollisionBounce;
    float mCollisionBaseSize;
    bool mCollisionUseScale;
    ct::String mSubEmitter;
    ParticleSubEmitterMode mSubEmitterMode;
    float mSubEmitterFrequency;
    int mSubEmitterAmountAtEnd;
    int mSubEmitterAmountAtCollision;
    int mSubEmitterAmountAtStart;
    bool mSubEmitterKeepVelocity;
    Random mRandom;

    ParticleBlendMode mBlend;
    TextureId mTexture;
    SurfaceData mGeometry;
    MeshId mMesh;
    bool mSurfaceReady;
    bool mUploadedEmpty;
    MaterialId mDefaultMaterial;
};

} // namespace prisma3d
