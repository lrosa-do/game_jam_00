#include "prisma3d/scene/Tree3D.h"

#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/ResourceProperties.h"

namespace prisma3d
{

namespace
{

const char* const kMeshName = "tree";
const char* const kBarkName = "tree bark";
const char* const kLeafName = "tree leaves";
const char* const kShadowName = "tree shadow";
constexpr float kLeafScissor = 0.4f;

} // namespace

Tree3D::Tree3D()
        : VisualInstance3D(),
          mParams(),
          mGeometry(),
          mDirty(true),
          mBuildCount(0),
          mMesh(),
          mShadowMesh(),
          mShadowGeometry(),
          mShadowLeafStep(6),
          mSurfacesReady(false),
          mDefaultBark(),
          mDefaultLeaf(),
          mBarkMaterial(),
          mLeafMaterial()
{

    if (MaterialStore* materials = MaterialStore::singleton())
    {
        mDefaultBark = materials->create(kBarkName);
        materials->setFloat(mDefaultBark, MaterialParam::Roughness, 0.9f);
        mBarkMaterial = mDefaultBark;

        mDefaultLeaf = materials->create(kLeafName);
        materials->setFloat(mDefaultLeaf, MaterialParam::Roughness, 0.7f);
        materials->setFloat(mDefaultLeaf, MaterialParam::AlphaScissorThreshold, kLeafScissor);
        materials->setTransparency(mDefaultLeaf, MaterialTransparency::AlphaScissor);
        materials->setCull(mDefaultLeaf, MaterialCull::Back);
        mLeafMaterial = mDefaultLeaf;
    }
}

Tree3D::~Tree3D()
{
    disconnectPreDraw();

    if (RenderingServer* server = RenderingServer::singleton())
        server->instanceSetBase(getInstance(), MeshId());
    if (MeshStore* store = MeshStore::singleton())
    {
        if (mMesh.valid()) store->destroy(mMesh);
        if (mShadowMesh.valid()) store->destroy(mShadowMesh);
    }
    if (MaterialStore* materials = MaterialStore::singleton())
    {
        if (mDefaultBark.valid()) materials->destroy(mDefaultBark);
        if (mDefaultLeaf.valid()) materials->destroy(mDefaultLeaf);
    }
}

void Tree3D::setParams(const TreeParams& params)
{
    if (params == mParams) return;
    mParams = params;
    markDirty();
}

const TreeParams& Tree3D::getParams() const
{
    return mParams;
}

void Tree3D::setSeed(std::int32_t seed)
{
    if (seed == mParams.seed) return;
    mParams.seed = seed;
    markDirty();
}

std::int32_t Tree3D::getSeed() const
{
    return mParams.seed;
}

void Tree3D::setBarkMaterial(MaterialId material)
{
    mBarkMaterial = material.valid() ? material : mDefaultBark;
    applyMaterials();
}

MaterialId Tree3D::getBarkMaterial() const
{
    return mBarkMaterial;
}

MaterialId Tree3D::getAssignedBarkMaterial() const
{
    return mBarkMaterial == mDefaultBark ? MaterialId() : mBarkMaterial;
}

void Tree3D::setLeafMaterial(MaterialId material)
{
    mLeafMaterial = material.valid() ? material : mDefaultLeaf;
    applyMaterials();
}

MaterialId Tree3D::getLeafMaterial() const
{
    return mLeafMaterial;
}

MaterialId Tree3D::getAssignedLeafMaterial() const
{
    return mLeafMaterial == mDefaultLeaf ? MaterialId() : mLeafMaterial;
}

bool Tree3D::rebuild()
{
    if (!mDirty) return false;

    mDirty = false;
    ++mBuildCount;
    TreeGenerator::generate(mParams, mGeometry);
    upload();
    return true;
}

bool Tree3D::isDirty() const
{
    return mDirty;
}

int Tree3D::getBuildCount() const
{
    return mBuildCount;
}

const SurfaceData& Tree3D::getBarkGeometry() const
{
    return mGeometry.bark;
}

const SurfaceData& Tree3D::getLeafGeometry() const
{
    return mGeometry.leaves;
}

void Tree3D::setShadowLeafStep(int step)
{
    if (step < 1) step = 1;
    if (step == mShadowLeafStep) return;
    mShadowLeafStep = step;
    markDirty();
}

int Tree3D::getShadowLeafStep() const
{
    return mShadowLeafStep;
}

const TreeGeometry& Tree3D::getShadowGeometry() const
{
    return mShadowGeometry;
}

bool Tree3D::buildCollision(CollisionMesh& out) const
{
    out.clear();
    if (mGeometry.bark.positions.empty() || mGeometry.trunkIndices.empty()) return false;

    out.append(mGeometry.bark.positions.data(),
            static_cast<std::uint32_t>(mGeometry.bark.positions.size()),
            mGeometry.trunkIndices.data(),
            static_cast<std::uint32_t>(mGeometry.trunkIndices.size()));
    return out.finish();
}

Math::Box Tree3D::getAabb() const
{
    const bool bark = !mGeometry.bark.positions.empty();
    const bool leaves = !mGeometry.leaves.positions.empty();
    if (bark && leaves)
        return MeshStore::computeAabb(mGeometry.bark.positions)
                .Union(MeshStore::computeAabb(mGeometry.leaves.positions));
    if (bark) return MeshStore::computeAabb(mGeometry.bark.positions);
    return MeshStore::computeAabb(mGeometry.leaves.positions);
}

void Tree3D::notificationHandler(int what)
{
    VisualInstance3D::notificationHandler(what);

    switch (what)
    {
        case kNotificationEnterTree:
            markDirty();
            connectPreDraw();
            break;
        case kNotificationExitTree:
            disconnectPreDraw();
            break;
        default:
            break;
    }
}

void Tree3D::upload()
{
    MeshStore* store = MeshStore::singleton();
    if (!store) return;

    if (!store->valid(mMesh))
    {
        mMesh = store->create(kMeshName);
        setBase(mMesh);
    }
    else
    {
        store->clearSurfaces(mMesh);
    }

    const bool bark = store->addSurface(mMesh, mGeometry.bark, kBarkName);
    const bool leaves = store->addSurface(mMesh, mGeometry.leaves, kLeafName);
    mSurfacesReady = bark && leaves;
    applyMaterials();

    mShadowGeometry.clear();
    if (mShadowLeafStep <= 1 || !mSurfacesReady)
    {
        store->setShadowMesh(mMesh, MeshId());
        if (store->valid(mShadowMesh)) store->clearSurfaces(mShadowMesh);
        return;
    }
    TreeGenerator::buildShadowGeometry(mGeometry, mShadowLeafStep, mShadowGeometry);
    if (!store->valid(mShadowMesh)) mShadowMesh = store->create(kShadowName);
    else
        store->clearSurfaces(mShadowMesh);
    const bool shadowBark = !mShadowGeometry.bark.indices.empty() &&
                            store->addSurface(mShadowMesh, mShadowGeometry.bark, kShadowName);
    const bool shadowLeaves = shadowBark && !mShadowGeometry.leaves.positions.empty() &&
                              store->addSurface(mShadowMesh, mShadowGeometry.leaves, kShadowName);
    store->setShadowMesh(mMesh, shadowBark && shadowLeaves ? mShadowMesh : MeshId());
}

void Tree3D::applyMaterials()
{
    MeshStore* store = MeshStore::singleton();
    if (!store || !mSurfacesReady || !store->valid(mMesh)) return;

    if (mBarkMaterial.valid()) store->setSurfaceMaterial(mMesh, 0, mBarkMaterial);
    if (mLeafMaterial.valid()) store->setSurfaceMaterial(mMesh, 1, mLeafMaterial);
}

void Tree3D::markDirty()
{
    mDirty = true;
}

void Tree3D::connectPreDraw()
{
    RenderingServer* server = RenderingServer::singleton();
    if (!server) return;
    server->framePreDrawConnect(this, [this]() { rebuild(); });
}

void Tree3D::disconnectPreDraw()
{
    RenderingServer* server = RenderingServer::singleton();
    if (server) server->framePreDrawDisconnect(this);
}

void Tree3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = Tree3D;
    using T = TreeParams;
    auto params = &P::getParams;
    auto setParams = &P::setParams;
    auto real = [](const char* name, double minimum, double maximum, double step)
    { return PropertyInfo::range(PropertyType::Float, name, minimum, maximum, step, true, true); };

    DB::addProperty<P>(PropertyInfo(PropertyType::Int, "seed"), &P::setSeed, &P::getSeed);
    resourceprop::add<P, MaterialId>(PropertyInfo::resource("bark_material", "Material"),
            &P::setBarkMaterial, &P::getAssignedBarkMaterial);
    resourceprop::add<P, MaterialId>(PropertyInfo::resource("leaf_material", "Material"),
            &P::setLeafMaterial, &P::getAssignedLeafMaterial);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Int, "shadow_leaf_step", 1.0, 64.0, 1.0, true),
            &P::setShadowLeafStep, &P::getShadowLeafStep);

    DB::addPropertyGroup<P>("Trunk", "trunk_");
    DB::addFieldProperty<P>(real("trunk_length", 0.0, 100.0, 0.01), params, setParams,
            &T::trunkLength);
    DB::addFieldProperty<P>(real("trunk_kink", -1.0, 1.0, 0.01), params, setParams, &T::trunkKink);
    DB::addFieldProperty<P>(PropertyInfo::range(PropertyType::Int, "trunk_steps", 0.0, 128.0, 1.0),
            params, setParams, &T::trunkSteps);
    DB::addFieldProperty<P>(real("max_radius", 0.0, 10.0, 0.001), params, setParams, &T::maxRadius);
    DB::addFieldProperty<P>(real("taper_rate", 0.01, 1.0, 0.001), params, setParams, &T::taperRate);
    DB::addFieldProperty<P>(real("twist_rate", -100.0, 100.0, 0.01), params, setParams,
            &T::twistRate);

    DB::addPropertyGroup<P>("Branches", "branch_");
    DB::addFieldProperty<P>(PropertyInfo::range(PropertyType::Int, "levels", 1.0, 12.0, 1.0),
            params, setParams, &T::levels);
    DB::addFieldProperty<P>(PropertyInfo::range(PropertyType::Int, "segments", 4.0, 64.0, 2.0),
            params, setParams, &T::segments);
    DB::addFieldProperty<P>(real("branch_factor", 0.0, 10.0, 0.01), params, setParams,
            &T::branchFactor);
    DB::addFieldProperty<P>(real("initial_branch_length", 0.0, 10.0, 0.01), params, setParams,
            &T::initialBranchLength);
    DB::addFieldProperty<P>(real("length_falloff_factor", 0.0, 2.0, 0.01), params, setParams,
            &T::lengthFalloffFactor);
    DB::addFieldProperty<P>(real("length_falloff_power", 0.0, 8.0, 0.01), params, setParams,
            &T::lengthFalloffPower);
    DB::addFieldProperty<P>(real("radius_falloff_rate", 0.01, 2.0, 0.01), params, setParams,
            &T::radiusFalloffRate);
    DB::addFieldProperty<P>(real("clump_max", 0.0, 2.0, 0.01), params, setParams, &T::clumpMax);
    DB::addFieldProperty<P>(real("clump_min", 0.0, 2.0, 0.01), params, setParams, &T::clumpMin);
    DB::addFieldProperty<P>(real("climb_rate", -10.0, 10.0, 0.01), params, setParams,
            &T::climbRate);
    DB::addFieldProperty<P>(real("drop_amount", -10.0, 10.0, 0.01), params, setParams,
            &T::dropAmount);
    DB::addFieldProperty<P>(real("grow_amount", -10.0, 10.0, 0.01), params, setParams,
            &T::growAmount);
    DB::addFieldProperty<P>(real("sweep_amount", -10.0, 10.0, 0.01), params, setParams,
            &T::sweepAmount);

    DB::addPropertyGroup<P>("Texture", "");
    DB::addFieldProperty<P>(real("v_multiplier", -10.0, 10.0, 0.01), params, setParams,
            &T::vMultiplier);
    DB::addFieldProperty<P>(real("twig_scale", 0.0, 100.0, 0.01), params, setParams, &T::twigScale);
}

} // namespace prisma3d
