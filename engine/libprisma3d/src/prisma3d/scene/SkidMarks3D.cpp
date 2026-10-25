#include "prisma3d/scene/SkidMarks3D.h"

#include "NodeFactory.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/VisualInstance3D.h"

namespace prisma3d
{

namespace
{

constexpr float kStep = 0.7f;
constexpr float kUploadEvery = 0.07f;
constexpr float kOver = 0.03f;

} // namespace

SkidMarks3D::SkidMarks3D()
        : Node3D(),
          mSurface(),
          mStrength(),
          mBorn(),
          mMesh(),
          mMaterial(),
          mShown(nullptr),
          mColor(0.17f, 0.10f, 0.06f, 1.0f),
          mLife(12.0f),
          mFade(5.0f),
          mRolling(0.16f),
          mClaimed(0),
          mNext(0),
          mTime(0.0f),
          mSinceUpload(0.0f),
          mSinceFade(0.0f),
          mDirty(false),
          mReady(false)
{
    setProcess(true);
}

SkidMarks3D::~SkidMarks3D()
{
    release();
}

void SkidMarks3D::setColor(const Math::Vec4& color)
{
    mColor = color;
}

const Math::Vec4& SkidMarks3D::getColor() const
{
    return mColor;
}

void SkidMarks3D::setLife(float seconds)
{
    mLife = seconds < 0.1f ? 0.1f : seconds;
}

float SkidMarks3D::getLife() const
{
    return mLife;
}

void SkidMarks3D::setFade(float seconds)
{
    mFade = seconds < 0.01f ? 0.01f : seconds;
}

float SkidMarks3D::getFade() const
{
    return mFade;
}

void SkidMarks3D::setRollingStrength(float strength)
{
    mRolling = strength < 0.0f ? 0.0f : (strength > 1.0f ? 1.0f : strength);
}

float SkidMarks3D::getRollingStrength() const
{
    return mRolling;
}

int SkidMarks3D::claimWheels(int count)
{
    if (count <= 0 || mClaimed + count > kWheels) return -1;
    const int first = mClaimed;
    mClaimed += count;
    return first;
}

void SkidMarks3D::build()
{
    if (mReady) return;
    MaterialStore* materials = Main::materials();
    MeshStore* meshes = Main::meshes();
    if (!materials || !meshes) return;
    mSurface.positions.resize(kQuads * 4, Math::Vec3(0.0f, 0.0f, 0.0f));
    mSurface.normals.resize(kQuads * 4, Math::Vec3(0.0f, 1.0f, 0.0f));
    mSurface.colors.resize(kQuads * 4, Math::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
    mSurface.uvs.resize(kQuads * 4, Math::Vec2(0.0f, 0.0f));
    mSurface.indices.resize(kQuads * 6);
    for (int q = 0; q < kQuads; ++q)
    {
        const std::uint32_t v = static_cast<std::uint32_t>(q) * 4;
        const std::uint32_t order[6] = { 0, 1, 2, 0, 2, 3 };
        for (int i = 0; i < 6; ++i)
            mSurface.indices[static_cast<std::size_t>(q) * 6 + i] = v + order[i];
    }
    mStrength.resize(kQuads * 4, 0.0f);
    mBorn.resize(kQuads, -1000.0f);

    mMaterial = materials->create("skid marks");
    materials->setShading(mMaterial, MaterialShading::Unshaded);
    materials->setTransparency(mMaterial, MaterialTransparency::AlphaBlend);
    materials->setBlendMode(mMaterial, MaterialBlend::Mix);
    materials->setCull(mMaterial, MaterialCull::Disabled);

    mMesh = meshes->create("skid marks");
    SurfaceLayout layout;
    layout.attributeMask = kVertexPosition | kVertexNormal | kVertexColor | kVertexUv;
    layout.vertexCount = kQuads * 4;
    layout.indexCount = kQuads * 6;
    layout.indexFormat = prisma::IndexFormat::UInt32;
    layout.aabb = Math::Box(Math::Vec3(-8000.0f, -2000.0f, -8000.0f),
            Math::Vec3(8000.0f, 2000.0f, 8000.0f));
    mReady = meshes->addDynamicSurface(mMesh, layout, "skid marks") >= 0;
    if (!mReady) return;
    meshes->setSurfaceMaterial(mMesh, 0, mMaterial);
    meshes->updateSurface(mMesh, 0, mSurface);

    mShown = createChild<MeshInstance3D>();
    mShown->setInternal(true);
    mShown->setMesh(mMesh);
    mShown->setCastShadows(false);
}

void SkidMarks3D::release()
{
    if (mShown) NodeFactory::destroy(mShown);
    mShown = nullptr;
    MeshStore* meshes = Main::meshes();
    if (meshes && meshes->valid(mMesh)) meshes->destroy(mMesh);
    MaterialStore* materials = Main::materials();
    if (materials && materials->valid(mMaterial)) materials->destroy(mMaterial);
    mMesh = MeshId();
    mMaterial = MaterialId();
    mReady = false;
}

Math::Vec3 SkidMarks3D::onGround(const Math::Vec3& place, PhysicsBodyId exclude) const
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (server)
    {
        PhysicsRayParameters ray;
        ray.from = place + Math::Vec3(0.0f, 0.6f, 0.0f);
        ray.to = place - Math::Vec3(0.0f, 1.0f, 0.0f);
        ray.exclude = exclude;
        PhysicsRayResult hit;
        if (server->intersectRay(ray, hit)) return hit.position + hit.normal * kOver;
    }
    return place + Math::Vec3(0.0f, kOver, 0.0f);
}

void SkidMarks3D::touch(int wheel, const Math::Vec3& contact, const Math::Vec3& halfAcross,
        float strength, PhysicsBodyId exclude)
{
    if (!mReady || wheel < 0 || wheel >= kWheels) return;
    Wheel& state = mWheels[wheel];
    const float moved = (contact - state.place).Length();
    if (state.down && moved < kStep) return;
    const Math::Vec3 left = onGround(contact - halfAcross, exclude);
    const Math::Vec3 right = onGround(contact + halfAcross, exclude);
    if (!state.down || moved > 4.0f)
    {
        state.down = true;
        state.left = left;
        state.right = right;
        state.place = contact;
        state.strength = strength;
        return;
    }

    const std::size_t v = static_cast<std::size_t>(mNext) * 4;
    mSurface.positions[v] = state.left;
    mSurface.positions[v + 1] = state.right;
    mSurface.positions[v + 2] = right;
    mSurface.positions[v + 3] = left;
    mStrength[v] = mStrength[v + 1] = state.strength;
    mStrength[v + 2] = mStrength[v + 3] = strength;
    mBorn[static_cast<std::size_t>(mNext)] = mTime;
    mNext = (mNext + 1) % kQuads;
    mDirty = true;

    state.left = left;
    state.right = right;
    state.place = contact;
    state.strength = strength;
}

void SkidMarks3D::lift(int wheel)
{
    if (wheel >= 0 && wheel < kWheels) mWheels[wheel].down = false;
}

void SkidMarks3D::clear()
{
    for (std::size_t q = 0; q < mBorn.size(); ++q) mBorn[q] = -1000.0f;
    for (int w = 0; w < kWheels; ++w) mWheels[w].down = false;
    mDirty = true;
    mSinceFade = 1.0f;
}

void SkidMarks3D::update(float step)
{
    if (!mReady) return;
    mTime += step;
    mSinceUpload += step;
    mSinceFade += step;
    if (!(mDirty && mSinceUpload >= kUploadEvery) && mSinceFade < 0.25f) return;
    const bool fading = mSinceFade >= 0.25f;
    for (int q = 0; q < kQuads; ++q)
    {
        const float age = mTime - mBorn[static_cast<std::size_t>(q)];
        if (!fading && age > 1.0f) continue;
        float fade = age < mLife - mFade ? 1.0f : (mLife - age) / mFade;
        fade = fade < 0.0f ? 0.0f : fade;
        for (int i = 0; i < 4; ++i)
        {
            const std::size_t v = static_cast<std::size_t>(q) * 4 + i;
            mSurface.colors[v] =
                    Math::Vec4(mColor.x, mColor.y, mColor.z, mColor.w * mStrength[v] * fade);
        }
    }
    Main::meshes()->updateSurface(mMesh, 0, mSurface);
    mDirty = false;
    mSinceUpload = 0.0f;
    if (fading) mSinceFade = 0.0f;
}

void SkidMarks3D::notificationHandler(int what)
{
    Node3D::notificationHandler(what);
    switch (what)
    {
        case kNotificationEnterTree:
            build();
            break;
        case kNotificationProcess:
            update(static_cast<float>(getProcessDeltaTime()));
            break;
        default:
            break;
    }
}

void SkidMarks3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = SkidMarks3D;
    DB::addProperty<P>(PropertyInfo::color("color"), &P::setColor, &P::getColor);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "life", 0.1, 600.0, 0.1, true),
            &P::setLife, &P::getLife);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "fade", 0.01, 600.0, 0.01, true),
            &P::setFade, &P::getFade);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "rolling_strength", 0.0, 1.0, 0.01),
            &P::setRollingStrength, &P::getRollingStrength);
}

} // namespace prisma3d
