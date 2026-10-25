#include "prisma3d/scene/Forest3D.h"

#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/SceneJson.h"
#include "prisma3d/scene/SceneResources.h"

#include "NodeFactory.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/Terrain3D.h"
#include "prisma3d/scene/Viewport.h"

#include "platform.h"

#include <ct/json.hpp>

#include <math.h>
#include <string.h>

namespace prisma3d
{

namespace
{

constexpr float kDefaultSwapDistance = 120.0f;
constexpr float kDitherLimit = 0.99f;

bool sameMatrix(const Math::Mat4& a, const Math::Mat4& b)
{
    return a.col0 == b.col0 && a.col1 == b.col1 && a.col2 == b.col2 && a.col3 == b.col3;
}

bool validBox(const Math::Box& box)
{
    return box.max.x >= box.min.x && box.max.y >= box.min.y && box.max.z >= box.min.z;
}

} // namespace

Forest3D::Forest3D()
        : Node3D(),
          mSpecies(),
          mInstances(),
          mSwapDistance(kDefaultSwapDistance),
          mDrawDistance(500.0f),
          mImpostorsEnabled(true),
          mFrustumCulling(true),
          mShadowCullMargin(2.0f),
          mCastShadows(true),
          mWritten(0),
          mConnected(false),
          mSet(),
          mWorld(Math::Mat4::Identity()),
          mTerrain(nullptr),
          mTerrainPath(),
          mPlacement(),
          mTerrainOffset(0.0f),
          mReseated(0),
          mReseatMs(0.0f)
{
}

Forest3D::~Forest3D()
{
    disconnectPreDraw();
    exitWorld();
    if (mTerrain) mTerrain->removeObserver(this);
    for (std::size_t i = 0; i < mSpecies.size(); ++i)
    {
        Species* species = mSpecies[i];
        species->meshNode = nullptr;
        releaseSpecies(*species);
        delete species;
    }
    mSpecies.clear();
}

void Forest3D::releaseSpecies(Species& species)
{
    RenderingServer* server = RenderingServer::singleton();
    if (server && species.impostor.valid()) server->impostorFree(species.impostor);
    species.impostor = ImpostorId();
}

int Forest3D::addSpecies(const ForestSpeciesDesc& desc)
{
    RenderingServer* server = RenderingServer::singleton();
    if (!server) return -1;
    MeshStore& meshes = server->meshes();
    if (!meshes.valid(desc.mesh))
    {
        log_error("forest: the species has no mesh");
        return -1;
    }
    if (desc.shadowMesh.valid()) meshes.setShadowMesh(desc.mesh, desc.shadowMesh);

    Species* species = new Species();
    species->desc = desc;
    species->impostor = server->impostorCreate(desc.mesh, desc.materials, fadeDistance());

    species->meshNode = createChild<MultiMeshInstance3D>();
    species->meshNode->setInternal(true);
    species->meshNode->setMesh(desc.mesh);
    species->meshNode->setUseCustomData(true);
    species->meshNode->getMultiMesh()->setDissolve(true);
    for (std::size_t i = 0; i < desc.materials.size(); ++i)
        if (desc.materials[i].valid())
            species->meshNode->setSurfaceMaterial(static_cast<std::uint32_t>(i), desc.materials[i]);
    species->meshNode->setCastShadows(mCastShadows);

    mSpecies.push_back(species);
    return static_cast<int>(mSpecies.size()) - 1;
}

void Forest3D::removeSpecies(int species)
{
    if (species < 0 || static_cast<std::size_t>(species) >= mSpecies.size()) return;
    Species* removed = mSpecies[static_cast<std::size_t>(species)];
    if (removed->meshNode) NodeFactory::destroy(removed->meshNode);
    removed->meshNode = nullptr;
    releaseSpecies(*removed);
    delete removed;
    mSpecies.erase(mSpecies.begin() + species);

    ct::Vector<Instance> kept;
    for (std::size_t i = 0; i < mInstances.size(); ++i)
    {
        if (static_cast<int>(mInstances[i].species) == species) continue;
        Instance instance = mInstances[i];
        if (static_cast<int>(instance.species) > species) --instance.species;
        kept.push_back(instance);
    }
    mInstances.clear();
    for (std::size_t i = 0; i < mSpecies.size(); ++i) mSpecies[i]->instances.clear();
    for (std::size_t i = 0; i < kept.size(); ++i)
    {
        Instance instance = kept[i];
        Species& owner = *mSpecies[instance.species];
        instance.local = static_cast<std::uint32_t>(owner.instances.size());
        owner.instances.push_back(static_cast<std::uint32_t>(mInstances.size()));
        mInstances.push_back(instance);
    }
    for (std::size_t i = 0; i < mSpecies.size(); ++i) mSpecies[i]->allocated = false;
    writeRecords();
}

int Forest3D::getSpeciesCount() const
{
    return static_cast<int>(mSpecies.size());
}

bool Forest3D::setSpeciesMeshPath(int species, const ct::String& path)
{
    const int count = getSpeciesCount();
    if (species < 0 || species > count || species >= kSpeciesSlots) return false;
    if (path.empty())
    {
        if (species == count) return true;
        removeSpecies(species);
        notifyPropertyListChanged();
        return true;
    }
    if (species < count && getSpeciesMeshPath(species) == sceneres::normalizePath(path))
        return true;

    sceneres::clearError();
    ForestSpeciesDesc desc;
    desc.mesh = sceneres::loadMesh(path);
    if (!desc.mesh.valid())
    {
        log_error("forest: %s", sceneres::takeError().c_str());
        return false;
    }
    if (const MeshStore* meshes = MeshStore::singleton())
        if (const MeshInfo* info = meshes->get(desc.mesh))
            for (std::size_t i = 0; i < info->surfaces.size(); ++i)
                desc.materials.push_back(info->surfaces[i].material);

    if (species == count)
    {
        if (addSpecies(desc) < 0) return false;
        notifyPropertyListChanged();
        return true;
    }

    RenderingServer* server = RenderingServer::singleton();
    if (!server) return false;
    Species& target = *mSpecies[static_cast<std::size_t>(species)];
    releaseSpecies(target);
    if (target.meshNode) NodeFactory::destroy(target.meshNode);
    target.desc = desc;
    target.impostor = server->impostorCreate(desc.mesh, desc.materials, fadeDistance());
    target.meshNode = createChild<MultiMeshInstance3D>();
    target.meshNode->setInternal(true);
    target.meshNode->setMesh(desc.mesh);
    target.meshNode->setUseCustomData(true);
    target.meshNode->getMultiMesh()->setDissolve(true);
    for (std::size_t i = 0; i < desc.materials.size(); ++i)
        if (desc.materials[i].valid())
            target.meshNode->setSurfaceMaterial(static_cast<std::uint32_t>(i), desc.materials[i]);
    target.meshNode->setCastShadows(mCastShadows);
    target.allocated = false;
    for (std::size_t i = 0; i < mInstances.size(); ++i)
        if (static_cast<int>(mInstances[i].species) == species) refreshBounds(mInstances[i]);
    writeRecords();
    return true;
}

bool Forest3D::setSpeciesShadowMeshPath(int species, const ct::String& path)
{
    if (species < 0 || static_cast<std::size_t>(species) >= mSpecies.size()) return false;
    Species& target = *mSpecies[static_cast<std::size_t>(species)];
    RenderingServer* server = RenderingServer::singleton();
    if (!server) return false;
    MeshStore& meshes = server->meshes();
    if (path.empty())
    {
        if (!target.desc.shadowMesh.valid()) return true;
        target.desc.shadowMesh = MeshId();
        meshes.setShadowMesh(target.desc.mesh, MeshId());
        return true;
    }
    if (getSpeciesShadowMeshPath(species) == sceneres::normalizePath(path)) return true;
    sceneres::clearError();
    const MeshId shadow = sceneres::loadMesh(path);
    if (!shadow.valid())
    {
        log_error("forest: %s", sceneres::takeError().c_str());
        return false;
    }
    target.desc.shadowMesh = shadow;
    meshes.setShadowMesh(target.desc.mesh, shadow);
    return true;
}

ct::String Forest3D::getSpeciesShadowMeshPath(int species) const
{
    if (species < 0 || static_cast<std::size_t>(species) >= mSpecies.size()) return ct::String();
    return sceneres::pathOf(mSpecies[static_cast<std::size_t>(species)]->desc.shadowMesh);
}

ct::String Forest3D::getSpeciesMeshPath(int species) const
{
    if (species < 0 || static_cast<std::size_t>(species) >= mSpecies.size()) return ct::String();
    return sceneres::pathOf(mSpecies[static_cast<std::size_t>(species)]->desc.mesh);
}

void Forest3D::validateProperty(PropertyInfo& info) const
{
    Node3D::validateProperty(info);
    if (info.name.size() < 10 || !info.name.starts_with("species_") || info.name[9] != '/') return;
    const char digit = info.name[8];
    if (digit < '0' || digit > '9') return;
    if (digit - '0' > getSpeciesCount()) info.usage = PropertyUsage::NoEditor;
}

ImpostorId Forest3D::getImpostor(int species) const
{
    if (species < 0 || static_cast<std::size_t>(species) >= mSpecies.size()) return ImpostorId();
    return mSpecies[static_cast<std::size_t>(species)]->impostor;
}

ImpostorSetId Forest3D::getImpostorSet() const
{
    return mSet;
}

MultiMeshInstance3D* Forest3D::getMeshNode(int species) const
{
    if (species < 0 || static_cast<std::size_t>(species) >= mSpecies.size()) return nullptr;
    return mSpecies[static_cast<std::size_t>(species)]->meshNode;
}

int Forest3D::addInstance(int species, const Math::Mat4& transform)
{
    if (species < 0 || static_cast<std::size_t>(species) >= mSpecies.size()) return -1;
    syncWorld();
    Species& owner = *mSpecies[static_cast<std::size_t>(species)];
    Instance instance;
    instance.transform = transform;
    instance.position = transform.GetTranslation();
    instance.species = static_cast<std::uint32_t>(species);
    instance.local = static_cast<std::uint32_t>(owner.instances.size());
    refreshBounds(instance);
    owner.instances.push_back(static_cast<std::uint32_t>(mInstances.size()));
    owner.allocated = false;
    mInstances.push_back(instance);
    const std::size_t index = mInstances.size() - 1;
    if (mSet.valid())
    {
        RenderingServer* server = RenderingServer::singleton();
        if (server) server->impostorSetResize(mSet, static_cast<std::uint32_t>(mInstances.size()));
        writeRecord(index);
    }
    return static_cast<int>(index);
}

int Forest3D::addInstance(int species, const Math::Mat4& transform, bool onTerrain)
{
    const int index = addInstance(species, transform);
    if (index >= 0) mInstances[static_cast<std::size_t>(index)].onTerrain = onTerrain;
    return index;
}

int Forest3D::removeInstancesInRadius(const Math::Vec3& worldCentre, float radius)
{
    if (mInstances.empty() || !(radius > 0.0f)) return 0;
    const Math::Mat4 world = getGlobalTransform();
    ct::Vector<Instance> kept;
    int removed = 0;
    for (std::size_t i = 0; i < mInstances.size(); ++i)
    {
        const Math::Vec3 position = world.TransformPoint(mInstances[i].position);
        const float dx = position.x - worldCentre.x;
        const float dz = position.z - worldCentre.z;
        if (dx * dx + dz * dz <= radius * radius)
        {
            ++removed;
            continue;
        }
        kept.push_back(mInstances[i]);
    }
    if (removed == 0) return 0;
    mInstances.clear();
    for (std::size_t i = 0; i < mSpecies.size(); ++i) mSpecies[i]->instances.clear();
    for (std::size_t i = 0; i < kept.size(); ++i)
    {
        Instance instance = kept[i];
        Species& owner = *mSpecies[instance.species];
        instance.local = static_cast<std::uint32_t>(owner.instances.size());
        owner.instances.push_back(static_cast<std::uint32_t>(mInstances.size()));
        mInstances.push_back(instance);
    }
    for (std::size_t i = 0; i < mSpecies.size(); ++i) mSpecies[i]->allocated = false;
    if (mSet.valid())
    {
        RenderingServer* server = RenderingServer::singleton();
        if (server) server->impostorSetResize(mSet, static_cast<std::uint32_t>(mInstances.size()));
    }
    writeRecords();
    return removed;
}

int Forest3D::plant(int species, const Math::Vec3& position, float yaw, float scale)
{
    const Math::Mat4 transform = Math::Mat4::Translation(position) * Math::Mat4::RotationY(yaw) *
                                 Math::Mat4::Scale(Math::Vec3(scale, scale, scale));
    return addInstance(species, transform);
}

void Forest3D::clearInstances()
{
    mInstances.clear();
    for (std::size_t i = 0; i < mSpecies.size(); ++i)
    {
        mSpecies[i]->instances.clear();
        mSpecies[i]->allocated = false;
    }
    writeRecords();
}

void Forest3D::setTerrain(Terrain3D* terrain)
{
    mTerrainPath = ct::String();
    if (terrain == mTerrain) return;
    if (mTerrain) mTerrain->removeObserver(this);
    mTerrain = terrain;
    if (mTerrain) mTerrain->addObserver(this);
}

Terrain3D* Forest3D::getTerrain() const
{
    return mTerrain;
}

void Forest3D::setTerrainPath(const ct::String& path)
{
    mTerrainPath = path;
    resolveTerrain();
}

ct::String Forest3D::getTerrainPath() const
{
    if (mTerrain) return getPathTo(mTerrain);
    return mTerrainPath;
}

void Forest3D::resolveTerrain()
{
    Terrain3D* terrain = nullptr;
    if (!mTerrainPath.empty())
    {
        Node* node = getNode(mTerrainPath);
        terrain = node ? node->castTo<Terrain3D>() : nullptr;
    }
    if (terrain)
    {
        setTerrain(terrain);
        return;
    }
    const ct::String pending = mTerrainPath;
    setTerrain(nullptr);
    mTerrainPath = pending;
}

void Forest3D::setTerrainOffset(float offset)
{
    mTerrainOffset = offset;
}

float Forest3D::getTerrainOffset() const
{
    return mTerrainOffset;
}

void Forest3D::setMaxSlope(float degrees)
{
    mPlacement.setMaxSlope(degrees);
}

float Forest3D::getMaxSlope() const
{
    return mPlacement.getMaxSlope();
}

void Forest3D::addExclusion(const TerrainPath& path, float margin)
{
    mPlacement.addExclusion(path, margin);
}

void Forest3D::clearExclusions()
{
    mPlacement.clearExclusions();
}

int Forest3D::plantOnTerrain(int species, float x, float z, float yaw, float scale)
{
    if (!mTerrain || !mTerrain->isLoaded()) return -1;
    const Math::Mat4 world = getGlobalTransform();
    const Math::Vec3 flat = world.TransformPoint(Math::Vec3(x, 0.0f, z));
    const Math::Vec3 ground(flat.x, mTerrain->getWorldHeightAt(flat), flat.z);
    if (!mPlacement.accepts(*mTerrain, ground)) return -1;
    const float y = world.InverseAffine().TransformPoint(ground).y + mTerrainOffset;
    const int instance = plant(species, Math::Vec3(x, y, z), yaw, scale);
    if (instance >= 0) mInstances[static_cast<std::size_t>(instance)].onTerrain = true;
    return instance;
}

int Forest3D::getReseatedCount() const
{
    return mReseated;
}

float Forest3D::getReseatMilliseconds() const
{
    return mReseatMs;
}

void Forest3D::onTerrainDestroyed()
{
    mTerrain = nullptr;
}

void Forest3D::onTerrainChanged(const Math::Box& worldBox)
{
    if (!mTerrain || !mTerrain->isLoaded()) return;
    const std::uint64_t started = Main::device() ? Main::device()->getTicksUsec() : 0;
    syncWorld();
    const Math::Mat4 world = mWorld;
    const Math::Mat4 inverse = world.InverseAffine();
    int reseated = 0;
    for (std::size_t i = 0; i < mInstances.size(); ++i)
    {
        Instance& instance = mInstances[i];
        if (!instance.onTerrain) continue;
        const Math::Vec3 at = world.TransformPoint(instance.position);
        if (at.x < worldBox.min.x || at.x > worldBox.max.x || at.z < worldBox.min.z ||
                at.z > worldBox.max.z)
            continue;
        const Math::Vec3 ground(at.x, mTerrain->getWorldHeightAt(at), at.z);
        const float y = inverse.TransformPoint(ground).y + mTerrainOffset;
        ++reseated;
        if (y == instance.position.y) continue;
        instance.position.y = y;
        instance.transform.col3.y = y;
        refreshBounds(instance);
        writeRecord(i);
        Species& species = *mSpecies[instance.species];
        if (!species.allocated) continue;
        const std::int32_t meshSlot = species.meshList.slotOf[instance.local];
        if (meshSlot >= 0) species.meshNode->setInstanceTransform(meshSlot, instance.transform);
    }
    const std::uint64_t finished = Main::device() ? Main::device()->getTicksUsec() : 0;
    mReseated = reseated;
    mReseatMs = static_cast<float>(finished - started) / 1000.0f;
}

int Forest3D::getInstanceCount() const
{
    return static_cast<int>(mInstances.size());
}

bool Forest3D::isInstanceOnTerrain(int instance) const
{
    if (instance < 0 || static_cast<std::size_t>(instance) >= mInstances.size()) return false;
    return mInstances[static_cast<std::size_t>(instance)].onTerrain;
}

int Forest3D::getInstanceSpecies(int instance) const
{
    if (instance < 0 || static_cast<std::size_t>(instance) >= mInstances.size()) return -1;
    return static_cast<int>(mInstances[static_cast<std::size_t>(instance)].species);
}

Math::Mat4 Forest3D::getInstanceTransform(int instance) const
{
    if (instance < 0 || static_cast<std::size_t>(instance) >= mInstances.size())
        return Math::Mat4::Identity();
    return mInstances[static_cast<std::size_t>(instance)].transform;
}

Math::Vec3 Forest3D::getInstanceCentre(int instance) const
{
    if (instance < 0 || static_cast<std::size_t>(instance) >= mInstances.size())
        return Math::Vec3(0.0f, 0.0f, 0.0f);
    return mInstances[static_cast<std::size_t>(instance)].centre;
}

float Forest3D::getInstanceRadius(int instance) const
{
    if (instance < 0 || static_cast<std::size_t>(instance) >= mInstances.size()) return 0.0f;
    return mInstances[static_cast<std::size_t>(instance)].radius;
}

float Forest3D::fadeDistance() const
{
    return mDrawDistance < mSwapDistance ? mDrawDistance : mSwapDistance;
}

void Forest3D::applyFade()
{
    RenderingServer* server = RenderingServer::singleton();
    if (!server) return;
    const float fade = fadeDistance();
    for (std::size_t i = 0; i < mSpecies.size(); ++i)
        if (mSpecies[i]->impostor.valid())
            server->impostorSetSwapInDistance(mSpecies[i]->impostor, fade);
}

void Forest3D::refreshBounds(Instance& instance) const
{
    const Math::Mat4 placed = mWorld * instance.transform;
    const MeshStore* store = MeshStore::singleton();
    const MeshInfo* info = store && instance.species < mSpecies.size()
                                   ? store->get(mSpecies[instance.species]->desc.mesh)
                                   : nullptr;
    if (!info || !validBox(info->aabb))
    {
        instance.centre = placed.GetTranslation();
        instance.radius = 0.0f;
        return;
    }
    const Math::Box box = info->aabb.Transformed(placed);
    const Math::Vec3 half = box.Extents();
    instance.centre = box.Center();
    instance.radius = sqrtf(half.x * half.x + half.y * half.y + half.z * half.z);
}

void Forest3D::writeRecord(std::size_t index)
{
    if (!mSet.valid() || index >= mInstances.size()) return;
    RenderingServer* server = RenderingServer::singleton();
    if (!server) return;
    const Instance& instance = mInstances[index];
    server->impostorSetInstance(mSet, static_cast<std::uint32_t>(index),
            mSpecies[instance.species]->impostor, mWorld * instance.transform,
            Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
}

void Forest3D::writeRecords()
{
    if (!mSet.valid()) return;
    RenderingServer* server = RenderingServer::singleton();
    if (!server) return;
    server->impostorSetResize(mSet, static_cast<std::uint32_t>(mInstances.size()));
    for (std::size_t i = 0; i < mInstances.size(); ++i) writeRecord(i);
}

bool Forest3D::syncWorld()
{
    const Math::Mat4 world = getGlobalTransform();
    if (sameMatrix(world, mWorld)) return false;
    mWorld = world;
    for (std::size_t i = 0; i < mInstances.size(); ++i) refreshBounds(mInstances[i]);
    writeRecords();
    return true;
}

void Forest3D::enterWorld()
{
    RenderingServer* server = RenderingServer::singleton();
    Viewport* viewport = Viewport::findViewport(this);
    if (!server || !viewport) return;
    exitWorld();
    mSet = server->impostorSetCreate(viewport->getScenario());
    server->impostorSetEnabled(mSet, mImpostorsEnabled);
    if (!syncWorld()) writeRecords();
}

void Forest3D::exitWorld()
{
    RenderingServer* server = RenderingServer::singleton();
    if (server && mSet.valid()) server->impostorSetFree(mSet);
    mSet = ImpostorSetId();
}

void Forest3D::setSwapDistance(float distance)
{
    mSwapDistance = distance > 0.0f ? distance : 0.0f;
    applyFade();
}

float Forest3D::getSwapDistance() const
{
    return mSwapDistance;
}

void Forest3D::setDrawDistance(float distance)
{
    mDrawDistance = distance > 0.0f ? distance : 0.0f;
    applyFade();
}

float Forest3D::getDrawDistance() const
{
    return mDrawDistance;
}

void Forest3D::setImpostorsEnabled(bool enabled)
{
    mImpostorsEnabled = enabled;
    RenderingServer* server = RenderingServer::singleton();
    if (server && mSet.valid()) server->impostorSetEnabled(mSet, enabled);
}

bool Forest3D::isImpostorsEnabled() const
{
    return mImpostorsEnabled;
}

void Forest3D::setCastShadows(bool cast)
{
    mCastShadows = cast;
    for (std::size_t i = 0; i < mSpecies.size(); ++i)
        if (mSpecies[i]->meshNode) mSpecies[i]->meshNode->setCastShadows(cast);
}

bool Forest3D::getCastShadows() const
{
    return mCastShadows;
}

bool Forest3D::meshDissolve(float distance, float fade, float radius, float& dither)
{
    const float beyond = distance - fade;
    dither = radius > 0.0f ? (beyond > 0.0f ? beyond : 0.0f) / radius : 0.0f;
    return distance <= fade + radius && dither <= kDitherLimit;
}

void Forest3D::allocate(Species& species)
{
    const int count = static_cast<int>(species.instances.size());
    species.meshNode->setInstanceCount(count);
    species.meshNode->setVisibleInstanceCount(0);
    species.meshList.items.clear();
    species.meshList.slotOf.clear();
    species.meshList.slotOf.resize(species.instances.size(), -1);
    species.allocated = true;
}

bool Forest3D::update(const Math::Vec3& cameraPosition)
{
    return update(cameraPosition, nullptr);
}

void Forest3D::setFrustumCullingEnabled(bool enabled)
{
    mFrustumCulling = enabled;
}

bool Forest3D::isFrustumCullingEnabled() const
{
    return mFrustumCulling;
}

void Forest3D::setShadowCullMargin(float margin)
{
    mShadowCullMargin = margin < 0.0f ? 0.0f : margin;
}

float Forest3D::getShadowCullMargin() const
{
    return mShadowCullMargin;
}

bool Forest3D::update(const Math::Vec3& cameraPosition, const Math::Frustum* frustum)
{
    mWritten = 0;
    bool changed = false;
    syncWorld();
    const float impostorFade = fadeDistance();
    for (std::size_t s = 0; s < mSpecies.size(); ++s)
    {
        Species& species = *mSpecies[s];
        if (!species.allocated) allocate(species);
        const bool impostors = mImpostorsEnabled && species.impostor.valid();
        const float fade = impostors ? impostorFade : mDrawDistance;
        const std::size_t meshBefore = species.meshList.items.size();
        const int cardsBefore = species.cards;
        int writes = 0;
        int cards = 0;

        for (std::size_t local = 0; local < species.instances.size(); ++local)
        {
            const std::uint32_t index = species.instances[local];
            Instance& instance = mInstances[index];
            if (instance.radius <= 0.0f)
            {
                refreshBounds(instance);
                if (instance.radius > 0.0f) writeRecord(index);
            }
            const float dx = instance.centre.x - cameraPosition.x;
            const float dy = instance.centre.y - cameraPosition.y;
            const float dz = instance.centre.z - cameraPosition.z;
            const float distance = sqrtf(dx * dx + dy * dy + dz * dz);
            float dither = 0.0f;
            bool want = meshDissolve(distance, fade, instance.radius, dither);
            if (want && frustum &&
                    !frustum->IntersectsSphere(instance.centre,
                            instance.radius * (1.0f + mShadowCullMargin)))
                want = false;
            if (impostors && distance >= fade - instance.radius &&
                    (!frustum || frustum->IntersectsSphere(instance.centre, instance.radius)))
                ++cards;

            SlotList& list = species.meshList;
            MultiMeshInstance3D* node = species.meshNode;
            std::int32_t slot = list.slotOf[local];
            if (want && slot < 0)
            {
                slot = static_cast<std::int32_t>(list.items.size());
                list.items.push_back(static_cast<std::uint32_t>(local));
                list.slotOf[local] = slot;
                node->setInstanceTransform(slot, instance.transform);
                node->setInstanceCustomData(slot, Math::Vec4(dither, 0.0f, 0.0f, 0.0f));
                instance.written = dither;
                ++writes;
            }
            else if (!want && slot >= 0)
            {
                const std::size_t last = list.items.size() - 1;
                if (static_cast<std::size_t>(slot) != last)
                {
                    const std::uint32_t moved = list.items[last];
                    const Instance& shifted = mInstances[species.instances[moved]];
                    list.items[static_cast<std::size_t>(slot)] = moved;
                    list.slotOf[moved] = slot;
                    node->setInstanceTransform(slot, shifted.transform);
                    node->setInstanceCustomData(slot,
                            Math::Vec4(shifted.written, 0.0f, 0.0f, 0.0f));
                    ++writes;
                }
                list.items.pop_back();
                list.slotOf[local] = -1;
            }
            else if (want && dither != instance.written)
            {
                node->setInstanceCustomData(slot, Math::Vec4(dither, 0.0f, 0.0f, 0.0f));
                instance.written = dither;
                ++writes;
            }
        }

        species.cards = cards;
        if (species.meshList.items.size() != meshBefore)
            species.meshNode->setVisibleInstanceCount(
                    static_cast<int>(species.meshList.items.size()));
        mWritten += writes;
        if (writes > 0 || species.meshList.items.size() != meshBefore || cards != cardsBefore)
            changed = true;
    }
    return changed;
}

int Forest3D::getMeshInstanceCount() const
{
    int count = 0;
    for (std::size_t i = 0; i < mSpecies.size(); ++i)
        count += static_cast<int>(mSpecies[i]->meshList.items.size());
    return count;
}

int Forest3D::getImpostorInstanceCount() const
{
    int count = 0;
    for (std::size_t i = 0; i < mSpecies.size(); ++i) count += mSpecies[i]->cards;
    return count;
}

int Forest3D::getMeshInstanceCount(int species) const
{
    if (species < 0 || static_cast<std::size_t>(species) >= mSpecies.size()) return 0;
    return static_cast<int>(mSpecies[static_cast<std::size_t>(species)]->meshList.items.size());
}

int Forest3D::getImpostorInstanceCount(int species) const
{
    if (species < 0 || static_cast<std::size_t>(species) >= mSpecies.size()) return 0;
    return mSpecies[static_cast<std::size_t>(species)]->cards;
}

int Forest3D::getWrittenInstanceCount() const
{
    return mWritten;
}

void Forest3D::notificationHandler(int what)
{
    Node3D::notificationHandler(what);

    switch (what)
    {
        case kNotificationEnterTree:
            if (!mTerrainPath.empty()) resolveTerrain();
            connectPreDraw();
            break;
        case kNotificationExitTree:
            disconnectPreDraw();
            break;
        case kNotificationEnterWorld:
            enterWorld();
            break;
        case kNotificationExitWorld:
            exitWorld();
            break;
        case kNotificationTransformChanged:
            syncWorld();
            break;
        default:
            break;
    }
}

void Forest3D::updateFromCamera()
{
    Viewport* viewport = Viewport::findViewport(this);
    if (!viewport) return;
    Camera3D* camera = viewport->getCamera3D();
    if (!camera) return;
    if (!mFrustumCulling)
    {
        update(camera->getGlobalPosition());
        return;
    }
    const Math::Frustum frustum = Math::Frustum::FromViewProjection(
            camera->getCameraProjection() * camera->getGlobalTransform().InverseAffine());
    update(camera->getGlobalPosition(), &frustum);
}

void Forest3D::connectPreDraw()
{
    RenderingServer* server = RenderingServer::singleton();
    if (!server || mConnected) return;
    server->framePreDrawConnect(this, [this]() { updateFromCamera(); });
    mConnected = true;
}

void Forest3D::disconnectPreDraw()
{
    RenderingServer* server = RenderingServer::singleton();
    if (server && mConnected) server->framePreDrawDisconnect(this);
    mConnected = false;
}

void Forest3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = Forest3D;
    DB::addProperty<P>(PropertyInfo::nodePath("terrain", "Terrain3D"), &P::setTerrainPath,
            &P::getTerrainPath);
    DB::addPropertyGroup<P>("Species", "species_");
    const auto editorOnly = [](PropertyInfo info)
    {
        info.usage = PropertyUsage::Editor;
        return info;
    };
    static const char* const kSpeciesLabels[kSpeciesSlots] = { "Species 1", "Species 2",
        "Species 3", "Species 4", "Species 5", "Species 6", "Species 7", "Species 8" };
    static const char* const kSpeciesPrefixes[kSpeciesSlots] = { "species_0/", "species_1/",
        "species_2/", "species_3/", "species_4/", "species_5/", "species_6/", "species_7/" };
    static const char* const kSpeciesMeshNames[kSpeciesSlots] = { "species_0/mesh",
        "species_1/mesh", "species_2/mesh", "species_3/mesh", "species_4/mesh", "species_5/mesh",
        "species_6/mesh", "species_7/mesh" };
    static const char* const kSpeciesShadowNames[kSpeciesSlots] = { "species_0/shadow_mesh",
        "species_1/shadow_mesh", "species_2/shadow_mesh", "species_3/shadow_mesh",
        "species_4/shadow_mesh", "species_5/shadow_mesh", "species_6/shadow_mesh",
        "species_7/shadow_mesh" };
    for (int i = 0; i < kSpeciesSlots; ++i)
    {
        DB::addPropertySubgroup<P>(kSpeciesLabels[i], kSpeciesPrefixes[i]);
        DB::addProperty(P::classNameStatic(),
                editorOnly(PropertyInfo::resource(kSpeciesMeshNames[i], "Mesh")),
                PropertySetter([i](Node* node, const PropertyValue& value)
                        { static_cast<P*>(node)->setSpeciesMeshPath(i, value.asString()); }),
                PropertyGetter(
                        [i](const Node* node) {
                            return PropertyValue::fromResourcePath(
                                    static_cast<const P*>(node)->getSpeciesMeshPath(i));
                        }));
        DB::addProperty(P::classNameStatic(),
                editorOnly(PropertyInfo::resource(kSpeciesShadowNames[i], "Mesh")),
                PropertySetter([i](Node* node, const PropertyValue& value)
                        { static_cast<P*>(node)->setSpeciesShadowMeshPath(i, value.asString()); }),
                PropertyGetter(
                        [i](const Node* node)
                        {
                            return PropertyValue::fromResourcePath(
                                    static_cast<const P*>(node)->getSpeciesShadowMeshPath(i));
                        }));
    }
    DB::addProperty<P>(PropertyInfo(PropertyType::Float, "terrain_offset"), &P::setTerrainOffset,
            &P::getTerrainOffset);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "max_slope", 0.0, 90.0, 0.1),
            &P::setMaxSlope, &P::getMaxSlope);

    DB::addPropertyGroup<P>("Distance", "");
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "swap_distance", 0.0, 100000.0, 0.1, true),
            &P::setSwapDistance, &P::getSwapDistance);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "draw_distance", 0.0, 100000.0, 0.1, true),
            &P::setDrawDistance, &P::getDrawDistance);

    DB::addPropertyGroup<P>("Rendering", "");
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "impostors_enabled"),
            &P::setImpostorsEnabled, &P::isImpostorsEnabled);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "cast_shadows"), &P::setCastShadows,
            &P::getCastShadows);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "frustum_culling_enabled"),
            &P::setFrustumCullingEnabled, &P::isFrustumCullingEnabled);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "shadow_cull_margin", 0.0, 1000.0, 0.01, true),
            &P::setShadowCullMargin, &P::getShadowCullMargin);
}

namespace
{

constexpr std::int64_t kForestDataVersion = 1;

}

bool Forest3D::saveData(ct::Json& data, SceneResult& result, const ct::String& path) const
{
    const bool hasExclusions = mPlacement.getExclusionCount() > 0;
    if (mSpecies.empty() && !hasExclusions) return false;

    data["version"] = ct::Json(static_cast<long long>(kForestDataVersion));

    ct::Vector<std::int32_t> remap;
    remap.resize(mSpecies.size(), -1);
    ct::Json species = ct::Json::array();
    for (std::size_t i = 0; i < mSpecies.size(); ++i)
    {
        const ForestSpeciesDesc& desc = mSpecies[i]->desc;
        const ct::String meshPath = sceneres::pathOf(desc.mesh);
        if (meshPath.empty())
        {
            result.addWarning(scenejson::element(path, "species", i),
                    "the mesh has no path; the species and its instances were not saved");
            continue;
        }
        remap[i] = static_cast<std::int32_t>(species.size());
        ct::Json entry = ct::Json::object();
        entry["mesh"] = ct::Json(meshPath);
        if (desc.shadowMesh.valid())
        {
            const ct::String shadowPath = sceneres::pathOf(desc.shadowMesh);
            if (shadowPath.empty())
                result.addWarning(scenejson::element(path, "species", i),
                        "the shadow mesh has no path; it was not saved");
            else
                entry["shadow_mesh"] = ct::Json(shadowPath);
        }
        ct::Json materials = ct::Json::array();
        for (std::size_t m = 0; m < desc.materials.size(); ++m)
        {
            const ct::String ref = sceneres::pathOf(desc.materials[m]);
            if (desc.materials[m].valid() && ref.empty())
                result.addWarning(scenejson::element(path, "species", i),
                        "a material has no path; it was not saved");
            materials.push_back(ct::Json(ref));
        }
        entry["materials"] = materials;
        species.push_back(entry);
    }
    data["species"] = species;

    ct::Vector<std::uint32_t> owners;
    ct::Vector<float> transforms;
    ct::Json onTerrain = ct::Json::array();
    std::size_t skipped = 0;
    for (std::size_t i = 0; i < mInstances.size(); ++i)
    {
        const Instance& instance = mInstances[i];
        if (instance.species >= remap.size() || remap[instance.species] < 0)
        {
            ++skipped;
            continue;
        }
        if (instance.onTerrain)
            onTerrain.push_back(ct::Json(static_cast<long long>(owners.size())));
        owners.push_back(static_cast<std::uint32_t>(remap[instance.species]));
        const float* values = instance.transform.Data();
        for (int v = 0; v < 16; ++v) transforms.push_back(values[v]);
    }
    if (skipped > 0)
        result.addWarning(scenejson::field(path, "instances"),
                "instances of species that were not saved were left out");
    if (!owners.empty())
    {
        ct::Json instances = ct::Json::object();
        instances["count"] = ct::Json(static_cast<long long>(owners.size()));
        instances["species"] = scenejson::fromU32Block(owners.data(), owners.size());
        instances["transforms"] = scenejson::fromFloatBlock(transforms.data(), transforms.size());
        if (onTerrain.size() > 0) instances["on_terrain"] = onTerrain;
        data["instances"] = instances;
    }
    if (hasExclusions) data["exclusions"] = scenejson::fromExclusions(mPlacement);
    return true;
}

bool Forest3D::loadData(const ct::Json& data, SceneResult& result, const ct::String& path)
{
    if (!scenejson::checkVersion(data, kForestDataVersion, result, path)) return false;

    ct::Vector<std::int32_t> remap;
    if (const ct::Json* species = scenejson::arrayField(data, "species", result, path))
    {
        for (std::size_t i = 0; i < species->size(); ++i)
        {
            const ct::Json& entry = species->at(i);
            const ct::String entryPath = scenejson::element(path, "species", i);
            remap.push_back(-1);
            const ct::String meshPath = entry.is_object()
                                                ? scenejson::getString(entry, "mesh", ct::String())
                                                : ct::String();
            if (meshPath.empty())
            {
                result.addError(entryPath, "missing mesh");
                continue;
            }
            ForestSpeciesDesc desc;
            sceneres::clearError();
            desc.mesh = sceneres::loadMesh(meshPath);
            if (!desc.mesh.valid())
            {
                result.addWarning(scenejson::field(entryPath, "mesh"),
                        sceneres::takeError() + "; the species was left out");
                continue;
            }
            const ct::String shadowPath = scenejson::getString(entry, "shadow_mesh", ct::String());
            if (!shadowPath.empty())
            {
                sceneres::clearError();
                desc.shadowMesh = sceneres::loadMesh(shadowPath);
                if (!desc.shadowMesh.valid())
                    result.addWarning(scenejson::field(entryPath, "shadow_mesh"),
                            sceneres::takeError());
            }
            if (const ct::Json* materials =
                            scenejson::arrayField(entry, "materials", result, entryPath))
            {
                for (std::size_t m = 0; m < materials->size(); ++m)
                {
                    const ct::Json& ref = materials->at(m);
                    MaterialId material;
                    if (ref.is_string() && !ref.str().empty())
                    {
                        sceneres::clearError();
                        material = sceneres::loadMaterial(ref.str());
                        if (!material.valid())
                            result.addWarning(scenejson::element(entryPath, "materials", m),
                                    sceneres::takeError());
                    }
                    desc.materials.push_back(material);
                }
            }
            const int index = addSpecies(desc);
            if (index < 0)
            {
                result.addError(entryPath, "the species could not be added");
                continue;
            }
            remap.back() = index;
        }
    }

    if (const ct::Json* instances = scenejson::objectField(data, "instances", result, path))
    {
        const ct::String instancesPath = scenejson::field(path, "instances");
        const std::int64_t count = scenejson::getInt(*instances, "count", 0);
        ct::Vector<std::uint32_t> owners;
        ct::Vector<float> transforms;
        const ct::Json* ownersField = instances->find("species");
        const ct::Json* transformsField = instances->find("transforms");
        if (count < 0 || !ownersField || !transformsField ||
                !scenejson::toU32Block(*ownersField, owners) ||
                !scenejson::toFloatBlock(*transformsField, transforms) ||
                owners.size() != static_cast<std::size_t>(count) ||
                transforms.size() != static_cast<std::size_t>(count) * 16)
        {
            result.addError(instancesPath, "needs count, species and 16 transform numbers for "
                                           "each instance");
        }
        else
        {
            ct::Vector<std::uint8_t> flags;
            flags.resize(owners.size(), 0);
            if (const ct::Json* onTerrain = instances->find("on_terrain"))
            {
                for (std::size_t i = 0; onTerrain->is_array() && i < onTerrain->size(); ++i)
                {
                    std::int64_t index = 0;
                    if (scenejson::toInt(onTerrain->at(i), index) && index >= 0 &&
                            static_cast<std::size_t>(index) < flags.size())
                        flags[static_cast<std::size_t>(index)] = 1;
                }
            }
            std::size_t left = 0;
            for (std::size_t i = 0; i < owners.size(); ++i)
            {
                if (owners[i] >= remap.size() || remap[owners[i]] < 0)
                {
                    ++left;
                    continue;
                }
                Math::Mat4 transform;
                memcpy(transform.Data(), &transforms[i * 16], 16 * sizeof(float));
                const int index = addInstance(remap[owners[i]], transform);
                if (index >= 0 && flags[i])
                    mInstances[static_cast<std::size_t>(index)].onTerrain = true;
            }
            if (left > 0)
                result.addWarning(instancesPath,
                        "instances of species that could not be loaded were left out");
        }
    }

    if (const ct::Json* exclusions = data.find("exclusions"))
        scenejson::toExclusions(*exclusions, mPlacement, result,
                scenejson::field(path, "exclusions"));
    return true;
}

} // namespace prisma3d
