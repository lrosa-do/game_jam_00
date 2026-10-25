#include "prisma3d/scene/Dismember3D.h"

#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/Skeleton3D.h"
#include "prisma3d/scene/VisualInstance3D.h"

#include <ct/hashmap.hpp>

namespace prisma3d
{

namespace
{

constexpr std::size_t kParts = static_cast<std::size_t>(PhysicsRagdollPart::Count);

void collectSkinned(Node* node, const Skeleton3D* skeleton, ct::Vector<MeshInstance3D*>& out)
{
    MeshInstance3D* instance = node->castTo<MeshInstance3D>();
    if (instance && instance->getSkeleton() == skeleton && instance->getMesh().valid())
        out.push_back(instance);
    for (std::size_t i = 0; i < node->getChildCount(); ++i)
        collectSkinned(node->getChild(i), skeleton, out);
}

int dominantPart(const SurfaceData& in, const std::uint32_t* corners,
        const ct::Vector<int>& partOfBone)
{
    float score[kParts] = {};
    for (int c = 0; c < 3; ++c)
    {
        const std::uint32_t packed = in.joints[corners[c]];
        const Math::Vec4& weight = in.weights[corners[c]];
        const float weights[4] = { weight.x, weight.y, weight.z, weight.w };
        for (int j = 0; j < 4; ++j)
        {
            const std::uint32_t bone = (packed >> (8 * j)) & 0xFFu;
            if (weights[j] > 0.0f && bone < partOfBone.size())
                score[static_cast<std::size_t>(partOfBone[bone])] += weights[j];
        }
    }
    int best = 0;
    for (std::size_t p = 1; p < kParts; ++p)
        if (score[p] > score[static_cast<std::size_t>(best)]) best = static_cast<int>(p);
    return best;
}

void copyVertex(const SurfaceData& in, std::uint32_t vertex, int part,
        const ct::Vector<int>& partOfBone, int partBone, SurfaceData& out)
{
    out.positions.push_back(in.positions[vertex]);
    if (!in.normals.empty()) out.normals.push_back(in.normals[vertex]);
    if (!in.tangents.empty()) out.tangents.push_back(in.tangents[vertex]);
    if (!in.colors.empty()) out.colors.push_back(in.colors[vertex]);
    if (!in.uvs.empty()) out.uvs.push_back(in.uvs[vertex]);
    if (!in.uvs2.empty()) out.uvs2.push_back(in.uvs2[vertex]);

    const std::uint32_t packed = in.joints[vertex];
    const Math::Vec4& weight = in.weights[vertex];
    const float weights[4] = { weight.x, weight.y, weight.z, weight.w };
    std::uint32_t joints = 0;
    float kept[4] = {};
    float sum = 0.0f;
    for (int j = 0; j < 4; ++j)
    {
        const std::uint32_t bone = (packed >> (8 * j)) & 0xFFu;
        if (weights[j] <= 0.0f || bone >= partOfBone.size() || partOfBone[bone] != part) continue;
        joints |= bone << (8 * j);
        kept[j] = weights[j];
        sum += weights[j];
    }
    if (sum > 0.0f)
    {
        out.joints.push_back(joints);
        out.weights.push_back(
                Math::Vec4(kept[0] / sum, kept[1] / sum, kept[2] / sum, kept[3] / sum));
    }
    else if (partBone >= 0)
    {
        out.joints.push_back(static_cast<std::uint32_t>(partBone));
        out.weights.push_back(Math::Vec4(1.0f, 0.0f, 0.0f, 0.0f));
    }
    else
    {
        out.joints.push_back(packed);
        out.weights.push_back(weight);
    }
}

} // namespace

Dismember3D::Dismember3D()
        : Node3D(),
          mRagdoll(nullptr),
          mMeshCut(false),
          mRadius(3.0f),
          mForce(8.0f)
{
}

Dismember3D::~Dismember3D()
{
    destroyCuts();
}

void Dismember3D::setRagdoll(Ragdoll3D* ragdoll)
{
    mRagdollPath = ct::String();
    mRagdoll = ragdoll;
}

Ragdoll3D* Dismember3D::getRagdoll() const
{
    return mRagdoll;
}

void Dismember3D::setRagdollPath(const ct::String& path)
{
    mRagdollPath = path;
    resolveRagdoll();
}

ct::String Dismember3D::getRagdollPath() const
{
    if (mRagdoll) return getPathTo(mRagdoll);
    return mRagdollPath;
}

void Dismember3D::resolveRagdoll()
{
    Ragdoll3D* ragdoll = nullptr;
    if (!mRagdollPath.empty())
    {
        Node* node = getNode(mRagdollPath);
        ragdoll = node ? node->castTo<Ragdoll3D>() : nullptr;
    }
    const ct::String pending = mRagdollPath;
    setRagdoll(ragdoll);
    if (!ragdoll) mRagdollPath = pending;
}

void Dismember3D::setRadius(float radius)
{
    mRadius = radius < 0.0f ? 0.0f : radius;
}

float Dismember3D::getRadius() const
{
    return mRadius;
}

void Dismember3D::setForce(float force)
{
    mForce = force < 0.0f ? 0.0f : force;
}

float Dismember3D::getForce() const
{
    return mForce;
}

bool Dismember3D::ensureActive()
{
    if (!mRagdoll) return false;
    return mRagdoll->isActive() || mRagdoll->start();
}

bool Dismember3D::detach(PhysicsRagdollPart part)
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server || part == PhysicsRagdollPart::Pelvis || part >= PhysicsRagdollPart::Count ||
            !ensureActive())
        return false;
    const PhysicsRagdollId ragdoll = mRagdoll->getRagdoll();
    bool separated[kParts];
    for (std::size_t i = 0; i < kParts; ++i)
        separated[i] = server->ragdollIsDetached(ragdoll, static_cast<PhysicsRagdollPart>(i));
    separated[static_cast<std::size_t>(part)] = true;
    cutMeshes(separated);
    return server->ragdollDetach(ragdoll, part);
}

bool Dismember3D::isDetached(PhysicsRagdollPart part) const
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server || !mRagdoll) return false;
    return server->ragdollIsDetached(mRagdoll->getRagdoll(), part);
}

int Dismember3D::getDetachedCount() const
{
    int count = 0;
    for (std::size_t i = 0; i < kParts; ++i)
        if (isDetached(static_cast<PhysicsRagdollPart>(i))) ++count;
    return count;
}

bool Dismember3D::explode(const Math::Vec3& origin)
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server || !ensureActive()) return false;
    bool separated[kParts];
    for (std::size_t i = 0; i < kParts; ++i) separated[i] = true;
    cutMeshes(separated);
    const PhysicsRagdollId ragdoll = mRagdoll->getRagdoll();
    for (std::size_t i = 0; i < kParts; ++i)
        server->ragdollDetach(ragdoll, static_cast<PhysicsRagdollPart>(i));
    for (std::size_t i = 0; i < kParts; ++i)
    {
        const PhysicsRagdollPart part = static_cast<PhysicsRagdollPart>(i);
        const Math::Vec3 center = server->ragdollGetPartTransform(ragdoll, part).GetTranslation();
        Math::Vec3 direction = center - origin;
        const float distance = direction.Length();
        if (mRadius > 0.0f && distance >= mRadius) continue;
        const float falloff = mRadius > 0.0f ? 1.0f - distance / mRadius : 1.0f;
        direction = distance > 1e-3f ? direction / distance : Math::Vec3(0.0f, 1.0f, 0.0f);
        const float mass = server->ragdollGetPartMass(ragdoll, part);
        server->ragdollApplyImpulse(ragdoll, part, direction * (mForce * falloff * mass),
                Math::Vec3(0.0f, 0.0f, 0.0f));
    }
    return true;
}

bool Dismember3D::isMeshCut() const
{
    return mMeshCut;
}

MeshId Dismember3D::cutMesh(MeshId source, const ct::Vector<int>& partOfBone,
        const int* partBones) const
{
    MeshStore* store = MeshStore::singleton();
    const MeshInfo* info = store ? store->get(source) : nullptr;
    if (!info || info->surfaces.empty()) return MeshId();
    const std::size_t surfaceCount = info->surfaces.size();
    ct::Vector<MaterialId> materials;
    for (std::size_t s = 0; s < surfaceCount; ++s)
    {
        const SurfaceData* in = store->surfaceSource(source, static_cast<std::uint32_t>(s));
        if (!in || in->topology != prisma::Topology::Triangles || in->joints.empty() ||
                in->weights.size() != in->joints.size())
            return MeshId();
        materials.push_back(info->surfaces[s].material);
    }
    const ct::String name = info->name + " (cut)";

    const MeshId mesh = store->create(name.c_str());
    for (std::size_t s = 0; s < surfaceCount; ++s)
    {
        const SurfaceData& in = *store->surfaceSource(source, static_cast<std::uint32_t>(s));
        const std::size_t count = in.indices.empty() ? in.positions.size() : in.indices.size();
        SurfaceData out;
        out.topology = in.topology;
        ct::HashMap<std::uint64_t, std::uint32_t> remap;
        for (std::size_t t = 0; t + 2 < count; t += 3)
        {
            std::uint32_t corners[3];
            for (int c = 0; c < 3; ++c)
                corners[c] = in.indices.empty() ? static_cast<std::uint32_t>(t + c)
                                                : in.indices[t + static_cast<std::size_t>(c)];
            const int part = dominantPart(in, corners, partOfBone);
            for (int c = 0; c < 3; ++c)
            {
                const std::uint64_t key = (static_cast<std::uint64_t>(corners[c]) << 8) |
                                          static_cast<std::uint64_t>(part);
                const std::uint32_t* found = remap.find(key);
                if (found)
                {
                    out.indices.push_back(*found);
                    continue;
                }
                const std::uint32_t index = static_cast<std::uint32_t>(out.positions.size());
                copyVertex(in, corners[c], part, partOfBone, partBones[part], out);
                remap[key] = index;
                out.indices.push_back(index);
            }
        }
        if (!store->addSurface(mesh, out, name.c_str()))
        {
            store->destroy(mesh);
            return MeshId();
        }
        store->setSurfaceMaterial(mesh, static_cast<std::uint32_t>(s), materials[s]);
    }
    return mesh;
}

void Dismember3D::cutMeshes(const bool* separated)
{
    if (!mRagdoll) return;
    Skeleton3D* skeleton = mRagdoll->getSkeleton();
    if (!skeleton) return;

    int partBones[kParts];
    for (std::size_t p = 0; p < kParts; ++p)
        partBones[p] = mRagdoll->getPartBone(static_cast<PhysicsRagdollPart>(p));
    const int boneCount = static_cast<int>(skeleton->getBoneCount());
    ct::Vector<int> partOfBone;
    partOfBone.resize(static_cast<std::size_t>(boneCount));
    for (int bone = 0; bone < boneCount; ++bone)
    {
        int part = 0;
        for (int walk = bone; walk >= 0 && part == 0; walk = skeleton->getBoneParent(walk))
            for (std::size_t p = 1; p < kParts; ++p)
                if (partBones[p] == walk)
                {
                    part = static_cast<int>(p);
                    break;
                }
        partOfBone[static_cast<std::size_t>(bone)] = part;
    }

    int parentPart[kParts];
    for (std::size_t p = 0; p < kParts; ++p)
    {
        const int parentBone = partBones[p] >= 0 ? skeleton->getBoneParent(partBones[p]) : -1;
        parentPart[p] =
                p == 0 || parentBone < 0 ? 0 : partOfBone[static_cast<std::size_t>(parentBone)];
    }
    int group[kParts];
    for (std::size_t p = 0; p < kParts; ++p)
    {
        int walk = static_cast<int>(p);
        for (std::size_t guard = 0; walk != 0 && !separated[walk] && guard < kParts; ++guard)
            walk = parentPart[walk];
        group[p] = separated[walk] ? walk : 0;
    }
    ct::Vector<int> groupOfBone;
    groupOfBone.resize(partOfBone.size());
    for (std::size_t bone = 0; bone < partOfBone.size(); ++bone)
        groupOfBone[bone] = group[static_cast<std::size_t>(partOfBone[bone])];

    MeshStore* store = MeshStore::singleton();
    if (mMeshCut)
    {
        for (std::size_t i = 0; i < mCuts.size(); ++i)
        {
            Node* node = getNode(mCuts[i].path);
            MeshInstance3D* instance = node ? node->castTo<MeshInstance3D>() : nullptr;
            if (!instance || instance->getMesh() != mCuts[i].cut) continue;
            const MeshId cut = cutMesh(mCuts[i].original, groupOfBone, partBones);
            if (!cut.valid()) continue;
            instance->setMesh(cut);
            if (store) store->destroy(mCuts[i].cut);
            mCuts[i].cut = cut;
        }
        return;
    }
    mMeshCut = true;

    ct::Vector<MeshInstance3D*> instances;
    collectSkinned(skeleton, skeleton, instances);
    for (std::size_t i = 0; i < instances.size(); ++i)
    {
        MeshInstance3D* instance = instances[i];
        const MeshId original = instance->getMesh();
        const MeshId cut = cutMesh(original, groupOfBone, partBones);
        if (!cut.valid()) continue;
        instance->setMesh(cut);
        Cut entry;
        entry.path = getPathTo(instance);
        entry.original = original;
        entry.cut = cut;
        mCuts.push_back(entry);
    }
}

void Dismember3D::restoreMeshes()
{
    for (std::size_t i = 0; i < mCuts.size(); ++i)
    {
        Node* node = getNode(mCuts[i].path);
        MeshInstance3D* instance = node ? node->castTo<MeshInstance3D>() : nullptr;
        if (instance && instance->getMesh() == mCuts[i].cut) instance->setMesh(mCuts[i].original);
    }
    destroyCuts();
    mMeshCut = false;
}

void Dismember3D::destroyCuts()
{
    MeshStore* store = MeshStore::singleton();
    if (store)
        for (std::size_t i = 0; i < mCuts.size(); ++i) store->destroy(mCuts[i].cut);
    mCuts.clear();
}

void Dismember3D::notificationHandler(int what)
{
    Node3D::notificationHandler(what);
    if (what == kNotificationEnterTree && !mRagdollPath.empty()) resolveRagdoll();
    if (what == kNotificationExitTree) restoreMeshes();
}

void Dismember3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = Dismember3D;
    DB::addProperty<P>(PropertyInfo::nodePath("ragdoll", "Ragdoll3D"), &P::setRagdollPath,
            &P::getRagdollPath);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "radius", 0.0, 100.0, 0.01, true),
            &P::setRadius, &P::getRadius);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "force", 0.0, 1000.0, 0.01, true),
            &P::setForce, &P::getForce);
}

} // namespace prisma3d
