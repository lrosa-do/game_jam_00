#include "prisma3d/scene/ModelInstance.h"

#include "NodeFactory.h"
#include "platform.h"
#include "prisma3d/core/FileSystem.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/import/ModelImporter.h"
#include "prisma3d/scene/AnimationPlayer.h"
#include "prisma3d/scene/BoneAttachment3D.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/CollisionShape3D.h"
#include "prisma3d/scene/LightmapGI.h"
#include "prisma3d/scene/PhysicsBody3D.h"
#include "prisma3d/scene/Skeleton3D.h"
#include "prisma3d/scene/VisualInstance3D.h"

#include <math.h>
#include <stdio.h>

namespace prisma3d
{

namespace
{

const float kRadiansToDegrees = 57.29577951308232f;

ct::String baseName(const ct::String& path)
{
    std::size_t start = 0;
    for (std::size_t i = 0; i < path.size(); ++i)
        if (path[i] == '/' || path[i] == '\\') start = i + 1;
    std::size_t end = path.size();
    for (std::size_t i = path.size(); i > start; --i)
    {
        if (path[i - 1] == '.')
        {
            end = i - 1;
            break;
        }
    }
    ct::String name;
    for (std::size_t i = start; i < end; ++i) name.push_back(path[i]);
    return name;
}

ct::String indexedName(const char* prefix, std::size_t index)
{
    char text[32];
    snprintf(text, sizeof(text), "%s%u", prefix, static_cast<unsigned>(index));
    return ct::String(text);
}

float srgbToLinear(std::uint8_t value)
{
    const float c = static_cast<float>(value) / 255.0f;
    return c <= 0.04045f ? c / 12.92f : powf((c + 0.055f) / 1.055f, 2.4f);
}

struct LightmapUser
{
    MeshInstance3D* instance;
    int node;
    int image;
};

struct LightmapPage
{
    int image;
    Pixmap* pixmap;
};

class Builder
{
public:
    Builder(const ModelData& document, const ct::Vector<MeshId>& meshes, int collisionType,
            int physicsShape, bool kinematicBody, float lightmapEnergy,
            const ModelImporter* importer, float scale, const ct::String& collidePrefixes,
            const ct::String& hidePrefixes)
            : mDocument(document),
              mMeshes(meshes),
              mCollisionType(collisionType),
              mPhysicsShape(physicsShape),
              mKinematicBody(kinematicBody),
              mLightmapEnergy(lightmapEnergy),
              mImporter(importer),
              mScale(scale),
              mCollidePrefixes(collidePrefixes),
              mHidePrefixes(hidePrefixes)
    {
        for (std::size_t m = 0; m < document.meshes.size(); ++m)
            if (document.meshes[m].lightmapImage >= 0) mHasLightmaps = true;
        mJointOf.resize(document.nodes.size());
        for (std::size_t i = 0; i < mJointOf.size(); ++i) mJointOf[i] = jointSkeleton(i);
        mAttachments.resize(document.nodes.size(), nullptr);

        mSkeletons.resize(document.skeletons.size());
        mSkeletonParent.resize(document.skeletons.size());
        for (std::size_t s = 0; s < document.skeletons.size(); ++s)
        {
            Skeleton3D* node = NodeFactory::make<Skeleton3D>();
            node->setName(document.skeletons.size() == 1 ? ct::String("Skeleton3D")
                                                         : indexedName("Skeleton3D_", s));
            node->setSkeleton(document.skeletons[s]);
            mSkeletons[s] = node;
            mSkeletonParent[s] = skeletonParentNode(s);
        }
    }

    Node3D* build()
    {
        Node3D* root = NodeFactory::make<Node3D>();
        const ct::String name = baseName(mDocument.path);
        root->setName(name.empty() ? ct::String("Model") : name);
        attachSkeletons(root, -1);
        for (std::size_t i = 0; i < mDocument.roots.size(); ++i)
            buildNode(mDocument.roots[i], root, -1);
        for (std::size_t s = 0; s < mSkeletons.size(); ++s)
            if (!mSkeletons[s]->getParent()) NodeFactory::attach(root, mSkeletons[s]);
        addPlayers();
        addLightmap(root);
        return root;
    }

private:
    int jointSkeleton(std::size_t node) const
    {
        const ModelNodeData& data = mDocument.nodes[node];
        const ct::String& name = data.name;
        if (name.empty()) return -1;
        // A mesh hung under a bone often takes the bone's name ("Bomb" under the bone "Bomb");
        // the joint is then the node without a mesh, and the mesh keeps its own transform.
        if (data.mesh >= 0 || data.camera >= 0)
            for (std::size_t n = 0; n < mDocument.nodes.size(); ++n)
                if (n != node && mDocument.nodes[n].name == name && mDocument.nodes[n].mesh < 0 &&
                        mDocument.nodes[n].camera < 0)
                    return -1;
        for (std::size_t s = 0; s < mDocument.skeletons.size(); ++s)
            if (mDocument.skeletons[s].findBone(name.c_str()) >= 0) return static_cast<int>(s);
        return -1;
    }

    int skeletonParentNode(std::size_t skeleton) const
    {
        const Skeleton& bones = mDocument.skeletons[skeleton];
        for (std::uint32_t b = 0; b < bones.boneCount(); ++b)
        {
            if (bones.bone(b).parent >= 0) continue;
            for (std::size_t n = 0; n < mDocument.nodes.size(); ++n)
            {
                if (mJointOf[n] != static_cast<int>(skeleton)) continue;
                if (mDocument.nodes[n].name != bones.bone(b).name) continue;
                return mDocument.nodes[n].parent;
            }
            break;
        }
        return -1;
    }

    bool validNode(int index) const
    {
        return index >= 0 && static_cast<std::size_t>(index) < mDocument.nodes.size();
    }

    bool pureJoint(int index) const
    {
        if (!validNode(index)) return false;
        const ModelNodeData& data = mDocument.nodes[static_cast<std::size_t>(index)];
        if (mJointOf[static_cast<std::size_t>(index)] < 0 || data.mesh >= 0 || data.camera >= 0)
            return false;
        for (std::size_t s = 0; s < mSkeletonParent.size(); ++s)
            if (mSkeletonParent[s] == index) return false;
        for (std::size_t i = 0; i < data.children.size(); ++i)
            if (!pureJoint(data.children[i])) return false;
        return true;
    }

    bool skinnedNode(const ModelNodeData& data) const
    {
        if (data.mesh < 0 || data.skeleton < 0) return false;
        if (static_cast<std::size_t>(data.skeleton) >= mSkeletons.size()) return false;
        if (static_cast<std::size_t>(data.mesh) >= mDocument.meshes.size()) return false;
        const ModelMeshData& mesh = mDocument.meshes[static_cast<std::size_t>(data.mesh)];
        for (std::size_t p = 0; p < mesh.primitives.size(); ++p)
            if (mesh.primitives[p].skinned) return true;
        return false;
    }

    bool needsPlaceholder(int index) const
    {
        const ModelNodeData& data = mDocument.nodes[static_cast<std::size_t>(index)];
        for (std::size_t i = 0; i < data.children.size(); ++i)
            if (!pureJoint(data.children[i])) return true;
        for (std::size_t s = 0; s < mSkeletonParent.size(); ++s)
            if (mSkeletonParent[s] == index) return true;
        return false;
    }

    void attachSkeletons(Node3D* parent, int index)
    {
        for (std::size_t s = 0; s < mSkeletons.size(); ++s)
            if (mSkeletonParent[s] == index && !mSkeletons[s]->getParent())
                NodeFactory::attach(parent, mSkeletons[s]);
    }

    bool hasSkeletonChild(int index) const
    {
        for (std::size_t s = 0; s < mSkeletons.size(); ++s)
            if (mSkeletonParent[s] == index && !mSkeletons[s]->getParent()) return true;
        return false;
    }

    BoneAttachment3D* attachmentFor(int jointNode)
    {
        const std::size_t slot = static_cast<std::size_t>(jointNode);
        if (mAttachments[slot]) return mAttachments[slot];

        const ModelNodeData& data = mDocument.nodes[slot];
        BoneAttachment3D* attachment = NodeFactory::make<BoneAttachment3D>();
        attachment->setName(data.name);
        NodeFactory::attach(mSkeletons[static_cast<std::size_t>(mJointOf[slot])], attachment);
        attachment->setBoneName(data.name);
        mAttachments[slot] = attachment;
        return attachment;
    }

    MeshInstance3D* makeMeshInstance(const ModelNodeData& data) const
    {
        MeshInstance3D* instance = NodeFactory::make<MeshInstance3D>();
        if (static_cast<std::size_t>(data.mesh) < mMeshes.size())
            instance->setMesh(mMeshes[static_cast<std::size_t>(data.mesh)]);
        if (mCollisionType != 0) instance->setCollisionType(mCollisionType);
        if (mLightmapEnergy > 0.0f &&
                static_cast<std::size_t>(data.mesh) < mDocument.meshes.size() &&
                mDocument.meshes[static_cast<std::size_t>(data.mesh)].lightmapImage >= 0)
        {
            LightmapUser user;
            user.instance = instance;
            user.node = static_cast<int>(&data - mDocument.nodes.data());
            user.image = mDocument.meshes[static_cast<std::size_t>(data.mesh)].lightmapImage;
            mLightmapUsers.push_back(user);
            instance->setCastShadows(false);
        }
        else if (mLightmapEnergy > 0.0f && mHasLightmaps)
            instance->setGiMode(GiMode::Dynamic);
        return instance;
    }

    Node3D* makeStaticMesh(const ModelNodeData& data) const
    {
        PhysicsBody3D* body =
                mKinematicBody ? static_cast<PhysicsBody3D*>(NodeFactory::make<AnimatableBody3D>())
                               : static_cast<PhysicsBody3D*>(NodeFactory::make<StaticBody3D>());
        MeshInstance3D* instance = makeMeshInstance(data);
        instance->setName("Mesh");
        NodeFactory::attach(body, instance);
        CollisionShape3D* shape = NodeFactory::make<CollisionShape3D>();
        shape->setName("Shape");
        NodeFactory::attach(body, shape);
        const MeshId mesh = static_cast<std::size_t>(data.mesh) < mMeshes.size()
                                    ? mMeshes[static_cast<std::size_t>(data.mesh)]
                                    : MeshId();
        const bool made = mPhysicsShape == 2 ? shape->setConcaveFromMesh(mesh)
                                             : shape->setConvexFromMesh(mesh);
        if (!made)
            log_warn("model: no physics shape for mesh '%s' of '%s'", data.name.c_str(),
                    mDocument.path.c_str());
        return body;
    }

    static bool matchesPrefix(const ct::String& name, const ct::String& prefixes)
    {
        std::size_t begin = 0;
        while (begin <= prefixes.size())
        {
            std::size_t end = prefixes.find(',', begin);
            if (end == ct::String::npos) end = prefixes.size();
            if (end > begin && name.starts_with(prefixes.substr(begin, end - begin))) return true;
            begin = end + 1;
        }
        return false;
    }

    void buildNode(int index, Node3D* parent, int boneNode)
    {
        if (!validNode(index)) return;
        const ModelNodeData& data = mDocument.nodes[static_cast<std::size_t>(index)];
        if (!mHidePrefixes.empty() && matchesPrefix(data.name, mHidePrefixes)) return;

        Node3D* node = nullptr;
        if (skinnedNode(data))
        {
            MeshInstance3D* instance = makeMeshInstance(data);
            if (!data.name.empty()) instance->setName(data.name);
            instance->setSkeletonPath("..");
            NodeFactory::attach(mSkeletons[static_cast<std::size_t>(data.skeleton)], instance);
            if (!needsPlaceholder(index)) return;
            node = NodeFactory::make<Node3D>();
            if (data.name.empty())
                node->setName(indexedName("node_", static_cast<std::size_t>(index)));
        }
        else if (data.mesh >= 0)
        {
            const bool collides =
                    mPhysicsShape != 0 &&
                    (mCollidePrefixes.empty() || matchesPrefix(data.name, mCollidePrefixes));
            node = collides ? makeStaticMesh(data) : makeMeshInstance(data);
        }
        else if (data.camera >= 0 &&
                 static_cast<std::size_t>(data.camera) < mDocument.cameras.size())
        {
            const ModelCameraData& source =
                    mDocument.cameras[static_cast<std::size_t>(data.camera)];
            Camera3D* camera = NodeFactory::make<Camera3D>();
            if (source.perspective)
                camera->setPerspective(source.yfov * kRadiansToDegrees, source.zNear, source.zFar);
            else
                camera->setOrthogonal(source.ymag * 2.0f, source.zNear, source.zFar);
            node = camera;
        }

        if (mJointOf[static_cast<std::size_t>(index)] >= 0)
        {
            if (!node)
            {
                if (hasSkeletonChild(index)) attachSkeletons(attachmentFor(index), index);
                for (std::size_t i = 0; i < data.children.size(); ++i)
                    buildNode(data.children[i], parent, index);
                return;
            }
            if (!data.name.empty()) node->setName(data.name);
            NodeFactory::attach(attachmentFor(index), node);
        }
        else
        {
            if (!node) node = NodeFactory::make<Node3D>();
            if (!data.name.empty()) node->setName(data.name);
            place(node, data, boneNode >= 0 ? attachmentFor(boneNode) : parent);
        }

        attachSkeletons(node, index);
        for (std::size_t i = 0; i < data.children.size(); ++i)
            buildNode(data.children[i], node, -1);
    }

    static void place(Node3D* node, const ModelNodeData& data, Node3D* parent)
    {
        NodeFactory::attach(parent, node);
        node->setPosition(data.translation);
        node->setQuaternion(data.rotation);
        node->setScale(data.scale);
    }

    void addPlayers()
    {
        for (std::size_t s = 0; s < mSkeletons.size(); ++s)
        {
            AnimationPlayer* player = nullptr;
            for (std::size_t a = 0; a < mDocument.animations.size(); ++a)
            {
                const AnimationClip& clip = mDocument.animations[a];
                if (clip.skeleton() != static_cast<int>(s)) continue;
                if (!player)
                {
                    player = NodeFactory::make<AnimationPlayer>();
                    player->setName("AnimationPlayer");
                    player->setSkeletonPath("..");
                    NodeFactory::attach(mSkeletons[s], player);
                }
                ct::String name = clip.name().empty() ? indexedName("animation_", a) : clip.name();
                if (player->hasAnimation(name)) name = name + indexedName("_", a);
                player->addAnimation(name, &clip);
            }
        }
    }

    struct Placement
    {
        int image;
        int page;
        int x;
        int y;
        int width;
        int height;
    };

    static Math::Vec3 sampleLightmap(const ct::Vector<std::uint16_t>& texels,
            const Placement& placement, std::uint32_t width, std::uint32_t height,
            const Math::Vec2& uv)
    {
        const float u = fminf(fmaxf(uv.x, 0.0f), 1.0f);
        const float v = fminf(fmaxf(uv.y, 0.0f), 1.0f);
        const int x =
                placement.x + static_cast<int>(u * static_cast<float>(placement.width - 1) + 0.5f);
        const int y =
                placement.y + static_cast<int>(v * static_cast<float>(placement.height - 1) + 0.5f);
        const std::size_t index =
                ((static_cast<std::size_t>(placement.page) * height + static_cast<std::size_t>(y)) *
                                width +
                        static_cast<std::size_t>(x)) *
                4;
        if (index + 2 >= texels.size()) return Math::Vec3(0.0f, 0.0f, 0.0f);
        return Math::Vec3(LightmapGIData::halfToFloat(texels[index]),
                LightmapGIData::halfToFloat(texels[index + 1]),
                LightmapGIData::halfToFloat(texels[index + 2]));
    }

    static void shBasis(const Math::Vec3& d, float out[9])
    {
        out[0] = 0.282095f;
        out[1] = 0.488603f * d.y;
        out[2] = 0.488603f * d.z;
        out[3] = 0.488603f * d.x;
        out[4] = 1.092548f * d.x * d.y;
        out[5] = 1.092548f * d.y * d.z;
        out[6] = 0.315392f * (3.0f * d.z * d.z - 1.0f);
        out[7] = 1.092548f * d.x * d.z;
        out[8] = 0.546274f * (d.x * d.x - d.y * d.y);
    }

    void generateProbes(LightmapGIData& data, const ct::Vector<LightmapPage>& pages,
            const ct::Vector<Placement>& placements, const ct::Vector<std::uint16_t>& texels,
            std::uint32_t width, std::uint32_t height, const Math::Box& bounds) const
    {
        const Math::Vec3 size = bounds.Size();
        const float extent = fmaxf(size.x, fmaxf(size.y, size.z));
        if (extent <= 0.0f) return;
        const float cell = fminf(fmaxf(extent / 28.0f, 1.0f), 3.0f);
        std::uint32_t dims[3];
        const float sizes[3] = { size.x, size.y, size.z };
        for (int a = 0; a < 3; ++a)
        {
            const float count = ceilf(sizes[a] / cell) + 1.0f;
            dims[a] = static_cast<std::uint32_t>(fminf(fmaxf(count, 2.0f), 64.0f));
        }
        const Math::Vec3 origin = bounds.min;
        const std::size_t cells = static_cast<std::size_t>(dims[0]) * dims[1] * dims[2];
        ct::Vector<float> sh;
        sh.resize(cells * 27, 0.0f);
        ct::Vector<float> coverage;
        coverage.resize(cells, 0.0f);
        const float radiance = 2.0f / 3.14159265f * mLightmapEnergy;
        const int reach = 2;

        auto scatter =
                [&](const Math::Vec3& p, const Math::Vec3& n, float area, const Math::Vec3& light)
        {
            const float gx = (p.x - origin.x) / cell;
            const float gy = (p.y - origin.y) / cell;
            const float gz = (p.z - origin.z) / cell;
            const int x0 = static_cast<int>(floorf(gx)) - reach;
            const int y0 = static_cast<int>(floorf(gy)) - reach;
            const int z0 = static_cast<int>(floorf(gz)) - reach;
            for (int z = z0; z <= z0 + reach * 2 + 1; ++z)
            {
                if (z < 0 || z >= static_cast<int>(dims[2])) continue;
                for (int y = y0; y <= y0 + reach * 2 + 1; ++y)
                {
                    if (y < 0 || y >= static_cast<int>(dims[1])) continue;
                    for (int x = x0; x <= x0 + reach * 2 + 1; ++x)
                    {
                        if (x < 0 || x >= static_cast<int>(dims[0])) continue;
                        const Math::Vec3 q(origin.x + static_cast<float>(x) * cell,
                                origin.y + static_cast<float>(y) * cell,
                                origin.z + static_cast<float>(z) * cell);
                        const Math::Vec3 d = q - p;
                        const float dist2 = d.x * d.x + d.y * d.y + d.z * d.z;
                        if (dist2 < 0.04f) continue;
                        const float dist = sqrtf(dist2);
                        const float facing = (n.x * d.x + n.y * d.y + n.z * d.z) / dist;
                        if (facing <= 0.0f) continue;
                        const float omega = fminf(area * facing / dist2, 0.3f);
                        const Math::Vec3 dir(-d.x / dist, -d.y / dist, -d.z / dist);
                        float basis[9];
                        shBasis(dir, basis);
                        const std::size_t index = (static_cast<std::size_t>(z) * dims[1] +
                                                          static_cast<std::size_t>(y)) *
                                                          dims[0] +
                                                  static_cast<std::size_t>(x);
                        float* target = sh.data() + index * 27;
                        for (int c = 0; c < 9; ++c)
                        {
                            target[c * 3 + 0] += light.x * omega * basis[c];
                            target[c * 3 + 1] += light.y * omega * basis[c];
                            target[c * 3 + 2] += light.z * omega * basis[c];
                        }
                        coverage[index] += omega;
                    }
                }
            }
        };

        std::size_t sampleCount = 0;
        for (std::size_t u = 0; u < mLightmapUsers.size(); ++u)
        {
            const LightmapUser& user = mLightmapUsers[u];
            std::size_t slot = pages.size();
            for (std::size_t p = 0; p < pages.size(); ++p)
                if (pages[p].image == user.image) slot = p;
            if (slot == pages.size()) continue;
            const Placement& placement = placements[slot];
            const ModelNodeData& nodeData = mDocument.nodes[static_cast<std::size_t>(user.node)];
            if (nodeData.mesh < 0) continue;
            const Math::Mat4 world = mDocument.nodeGlobalTransform(user.node);
            const ModelMeshData& mesh = mDocument.meshes[static_cast<std::size_t>(nodeData.mesh)];
            for (std::size_t pi = 0; pi < mesh.primitives.size(); ++pi)
            {
                SurfaceData surface;
                if (!mImporter->readPrimitive(mDocument, mesh.primitives[pi], surface)) continue;
                if (surface.uvs2.size() != surface.positions.size()) continue;
                const std::size_t triangles = surface.indices.empty() ? surface.positions.size() / 3
                                                                      : surface.indices.size() / 3;
                for (std::size_t t = 0; t < triangles; ++t)
                {
                    std::uint32_t i0 = static_cast<std::uint32_t>(t * 3);
                    std::uint32_t i1 = i0 + 1;
                    std::uint32_t i2 = i0 + 2;
                    if (!surface.indices.empty())
                    {
                        i0 = surface.indices[t * 3];
                        i1 = surface.indices[t * 3 + 1];
                        i2 = surface.indices[t * 3 + 2];
                    }
                    if (i0 >= surface.positions.size() || i1 >= surface.positions.size() ||
                            i2 >= surface.positions.size())
                        continue;
                    const Math::Vec3 p0 = world.TransformPoint(surface.positions[i0] * mScale);
                    const Math::Vec3 p1 = world.TransformPoint(surface.positions[i1] * mScale);
                    const Math::Vec3 p2 = world.TransformPoint(surface.positions[i2] * mScale);
                    const Math::Vec3 e1 = p1 - p0;
                    const Math::Vec3 e2 = p2 - p0;
                    Math::Vec3 normal = Math::Vec3::Cross(e1, e2);
                    const float doubleArea = normal.Length();
                    if (doubleArea < 1e-6f) continue;
                    normal = normal * (1.0f / doubleArea);
                    const float area = doubleArea * 0.5f;
                    const int k =
                            static_cast<int>(fminf(fmaxf(ceilf(sqrtf(area) / 0.5f), 1.0f), 10.0f));
                    const float sampleArea = area / static_cast<float>(k * (k + 1) / 2);
                    const Math::Vec2& uv0 = surface.uvs2[i0];
                    const Math::Vec2& uv1 = surface.uvs2[i1];
                    const Math::Vec2& uv2 = surface.uvs2[i2];
                    for (int a = 0; a < k; ++a)
                    {
                        for (int b = 0; b < k - a; ++b)
                        {
                            const float bu =
                                    (static_cast<float>(a) + 1.0f / 3.0f) / static_cast<float>(k);
                            const float bv =
                                    (static_cast<float>(b) + 1.0f / 3.0f) / static_cast<float>(k);
                            const float bw = 1.0f - bu - bv;
                            const Math::Vec3 p = p0 + e1 * bu + e2 * bv;
                            const Math::Vec2 uv(uv0.x * bw + uv1.x * bu + uv2.x * bv,
                                    uv0.y * bw + uv1.y * bu + uv2.y * bv);
                            const Math::Vec3 light =
                                    sampleLightmap(texels, placement, width, height, uv) * radiance;
                            scatter(p, normal, sampleArea, light);
                            ++sampleCount;
                        }
                    }
                }
            }
        }

        for (int pass = 0; pass < 3; ++pass)
        {
            ct::Vector<float> filled = sh;
            ct::Vector<float> filledCoverage = coverage;
            for (std::uint32_t z = 0; z < dims[2]; ++z)
                for (std::uint32_t y = 0; y < dims[1]; ++y)
                    for (std::uint32_t x = 0; x < dims[0]; ++x)
                    {
                        const std::size_t index =
                                (static_cast<std::size_t>(z) * dims[1] + y) * dims[0] + x;
                        if (coverage[index] > 0.0f) continue;
                        float sum[27] = {};
                        int found = 0;
                        for (int dz = -1; dz <= 1; ++dz)
                            for (int dy = -1; dy <= 1; ++dy)
                                for (int dx = -1; dx <= 1; ++dx)
                                {
                                    const int nx = static_cast<int>(x) + dx;
                                    const int ny = static_cast<int>(y) + dy;
                                    const int nz = static_cast<int>(z) + dz;
                                    if (nx < 0 || ny < 0 || nz < 0 ||
                                            nx >= static_cast<int>(dims[0]) ||
                                            ny >= static_cast<int>(dims[1]) ||
                                            nz >= static_cast<int>(dims[2]))
                                        continue;
                                    const std::size_t neighbour =
                                            (static_cast<std::size_t>(nz) * dims[1] +
                                                    static_cast<std::size_t>(ny)) *
                                                    dims[0] +
                                            static_cast<std::size_t>(nx);
                                    if (coverage[neighbour] <= 0.0f) continue;
                                    for (int c = 0; c < 27; ++c) sum[c] += sh[neighbour * 27 + c];
                                    ++found;
                                }
                        if (found == 0) continue;
                        for (int c = 0; c < 27; ++c)
                            filled[index * 27 + c] = sum[c] / static_cast<float>(found);
                        filledCoverage[index] = 1e-6f;
                    }
            sh = filled;
            coverage = filledCoverage;
        }

        if (data.setProbeGrid(origin, Math::Vec3(cell, cell, cell), dims, sh))
            log_info("model: lightmap probes of '%s': %ux%ux%u cells of %.2f m from %zu samples",
                    mDocument.path.c_str(), dims[0], dims[1], dims[2], cell, sampleCount);
    }


    void addLightmap(Node3D* root)
    {
        if (mLightmapUsers.empty()) return;
        ct::Vector<LightmapPage> pages;
        std::uint32_t width = 0;
        std::uint32_t height = 0;
        for (std::size_t u = 0; u < mLightmapUsers.size(); ++u)
        {
            const int image = mLightmapUsers[u].image;
            bool known = false;
            for (std::size_t p = 0; p < pages.size() && !known; ++p)
                known = pages[p].image == image;
            if (known) continue;
            const ModelImageData& imageData = mDocument.images[static_cast<std::size_t>(image)];
            const ct::String& path = imageData.path;
            ct::Vector<std::uint8_t> bytes;
            if (imageData.embedded() && mImporter)
                mImporter->readImage(mDocument, imageData, bytes);
            else
                bytes = FileSystem::getSingleton().readBinary(path);
            Pixmap* pixmap = new Pixmap();
            bool ok = !bytes.empty() && pixmap->load_from_memory(bytes.data(),
                                                static_cast<std::uint32_t>(bytes.size()));
            if (ok && pixmap->components != 4)
            {
                Pixmap* converted = pixmap->convert_to_rgba();
                delete pixmap;
                pixmap = converted;
                ok = converted != nullptr;
            }
            if (!ok || pixmap->width <= 0 || pixmap->height <= 0)
            {
                log_warn("model: could not read the lightmap '%s' of '%s'", path.c_str(),
                        mDocument.path.c_str());
                delete pixmap;
                continue;
            }
            LightmapPage page;
            page.image = image;
            page.pixmap = pixmap;
            pages.push_back(page);
            if (static_cast<std::uint32_t>(pixmap->width) > width)
                width = static_cast<std::uint32_t>(pixmap->width);
            if (static_cast<std::uint32_t>(pixmap->height) > height)
                height = static_cast<std::uint32_t>(pixmap->height);
        }
        if (pages.empty()) return;
        log_info("model: '%s': lightmap with %d pages for %d meshes", mDocument.path.c_str(),
                static_cast<int>(pages.size()), static_cast<int>(mLightmapUsers.size()));

        const int gutter = 2;
        ct::Vector<std::size_t> order;
        for (std::size_t p = 0; p < pages.size(); ++p) order.push_back(p);
        for (std::size_t i = 1; i < order.size(); ++i)
        {
            const std::size_t value = order[i];
            std::size_t k = i;
            while (k > 0 && pages[order[k - 1]].pixmap->height < pages[value].pixmap->height)
            {
                order[k] = order[k - 1];
                --k;
            }
            order[k] = value;
        }
        ct::Vector<Placement> placements;
        placements.resize(pages.size());
        LightmapGI::AtlasCursor cursor;
        const int pageWidth = static_cast<int>(width);
        const int pageHeight = static_cast<int>(height);
        for (std::size_t i = 0; i < order.size(); ++i)
        {
            const Pixmap& pixmap = *pages[order[i]].pixmap;
            int x = 0;
            int y = 0;
            LightmapGI::placeBlock(cursor, pixmap.width, pixmap.height, pageWidth, pageHeight,
                    gutter, x, y);
            Placement& placement = placements[order[i]];
            placement.image = pages[order[i]].image;
            placement.page = cursor.layer;
            placement.x = x;
            placement.y = y;
            placement.width = pixmap.width;
            placement.height = pixmap.height;
        }
        const int layerCount = cursor.layer + 1;

        const std::size_t layerTexels = static_cast<std::size_t>(width) * height * 4;
        ct::Vector<std::uint16_t> texels;
        texels.resize(layerTexels * static_cast<std::size_t>(layerCount), 0);
        const std::uint16_t one = LightmapGIData::floatToHalf(1.0f);
        for (std::size_t p = 0; p < pages.size(); ++p)
        {
            const Pixmap& pixmap = *pages[p].pixmap;
            const Placement& placement = placements[p];
            std::uint16_t* layer =
                    texels.data() + layerTexels * static_cast<std::size_t>(placement.page);
            for (int y = -1; y <= placement.height; ++y)
            {
                const int targetY = placement.y + y;
                if (targetY < 0 || targetY >= pageHeight) continue;
                const int sourceY = y < 0 ? 0 : (y >= placement.height ? placement.height - 1 : y);
                const std::uint8_t* row =
                        pixmap.pixels + static_cast<std::size_t>(sourceY) * pixmap.width * 4;
                std::uint16_t* out = layer + static_cast<std::size_t>(targetY) * width * 4;
                for (int x = -1; x <= placement.width; ++x)
                {
                    const int targetX = placement.x + x;
                    if (targetX < 0 || targetX >= pageWidth) continue;
                    const int sourceX =
                            x < 0 ? 0 : (x >= placement.width ? placement.width - 1 : x);
                    const std::uint8_t* texel = row + sourceX * 4;
                    std::uint16_t* target = out + targetX * 4;
                    target[0] =
                            LightmapGIData::floatToHalf(srgbToLinear(texel[0]) * mLightmapEnergy);
                    target[1] =
                            LightmapGIData::floatToHalf(srgbToLinear(texel[1]) * mLightmapEnergy);
                    target[2] =
                            LightmapGIData::floatToHalf(srgbToLinear(texel[2]) * mLightmapEnergy);
                    target[3] = one;
                }
            }
        }

        ct::Unique<LightmapGIData> data = ct::make_unique<LightmapGIData>();
        if (!data->setLightImages(width, height, static_cast<std::uint32_t>(layerCount),
                    texels.data()))
        {
            for (std::size_t p = 0; p < pages.size(); ++p) delete pages[p].pixmap;
            return;
        }
        data->setUsesSphericalHarmonics(false);
        data->setBakedExposure(1.0f);

        LightmapGI* lightmap = NodeFactory::make<LightmapGI>();
        lightmap->setName("Lightmap");
        NodeFactory::attach(root, lightmap);
        Math::Box bounds;
        bool haveBounds = false;
        for (std::size_t u = 0; u < mLightmapUsers.size(); ++u)
        {
            const LightmapUser& user = mLightmapUsers[u];
            std::size_t slot = pages.size();
            for (std::size_t p = 0; p < pages.size(); ++p)
                if (pages[p].image == user.image) slot = p;
            if (slot == pages.size()) continue;
            const Placement& placement = placements[slot];
            const Math::Vec4 uvScale(static_cast<float>(placement.x) / static_cast<float>(width),
                    static_cast<float>(placement.y) / static_cast<float>(height),
                    static_cast<float>(placement.width) / static_cast<float>(width),
                    static_cast<float>(placement.height) / static_cast<float>(height));
            data->addUser(lightmap->getPathTo(user.instance), uvScale, placement.page);
            const ModelNodeData& nodeData = mDocument.nodes[static_cast<std::size_t>(user.node)];
            if (nodeData.mesh < 0) continue;
            const Math::Mat4 world = mDocument.nodeGlobalTransform(user.node);
            const ModelMeshData& mesh = mDocument.meshes[static_cast<std::size_t>(nodeData.mesh)];
            for (std::size_t p = 0; p < mesh.primitives.size(); ++p)
            {
                const ModelPrimitiveData& primitive = mesh.primitives[p];
                if (!primitive.hasAabb) continue;
                for (int corner = 0; corner < 8; ++corner)
                {
                    const Math::Vec3 local((corner & 1) ? primitive.aabb.max.x
                                                        : primitive.aabb.min.x,
                            (corner & 2) ? primitive.aabb.max.y : primitive.aabb.min.y,
                            (corner & 4) ? primitive.aabb.max.z : primitive.aabb.min.z);
                    const Math::Vec3 point = world.TransformPoint(local);
                    if (!haveBounds)
                    {
                        bounds = Math::Box(point, point);
                        haveBounds = true;
                        continue;
                    }
                    bounds.min = Math::Vec3(point.x < bounds.min.x ? point.x : bounds.min.x,
                            point.y < bounds.min.y ? point.y : bounds.min.y,
                            point.z < bounds.min.z ? point.z : bounds.min.z);
                    bounds.max = Math::Vec3(point.x > bounds.max.x ? point.x : bounds.max.x,
                            point.y > bounds.max.y ? point.y : bounds.max.y,
                            point.z > bounds.max.z ? point.z : bounds.max.z);
                }
            }
        }
        if (haveBounds) data->setBounds(bounds);
        if (haveBounds && mImporter)
            generateProbes(*data, pages, placements, texels, width, height, bounds);
        for (std::size_t p = 0; p < pages.size(); ++p) delete pages[p].pixmap;
        lightmap->setLightData(static_cast<ct::Unique<LightmapGIData>&&>(data));
        lightmap->setModelAtlas(cursor);
        log_info("model: lightmap of '%s': %zu images in %d page(s) of %ux%u",
                mDocument.path.c_str(), pages.size(), layerCount, width, height);
    }

    const ModelData& mDocument;
    const ct::Vector<MeshId>& mMeshes;
    int mCollisionType;
    int mPhysicsShape;
    bool mKinematicBody;
    float mLightmapEnergy;
    const ModelImporter* mImporter;
    bool mHasLightmaps = false;
    float mScale;
    ct::String mCollidePrefixes;
    ct::String mHidePrefixes;
    mutable ct::Vector<LightmapUser> mLightmapUsers;
    ct::Vector<int> mJointOf;
    ct::Vector<Skeleton3D*> mSkeletons;
    ct::Vector<int> mSkeletonParent;
    ct::Vector<BoneAttachment3D*> mAttachments;
};

} // namespace

Node3D* instantiateModel(Node* parent, const ModelData& document, const ct::Vector<MeshId>& meshes,
        int collisionType)
{
    if (!parent) return nullptr;
    Builder builder(document, meshes, collisionType, 0, false, 0.0f, nullptr, 1.0f, ct::String(),
            ct::String());
    Node3D* root = builder.build();
    NodeFactory::attach(parent, root);
    return root;
}

Node3D* buildModel(ModelId model)
{
    AssetLoader* loader = AssetLoader::singleton();
    if (!loader) return nullptr;
    const ModelInfo* info = loader->model(model);
    if (!info || !info->parsed) return nullptr;
    Builder builder(info->document, info->meshes, info->collisionType, info->physicsShape,
            info->kinematicBody, info->lightmapEnergy, info->importer, info->scale,
            info->collidePrefixes, info->hidePrefixes);
    return builder.build();
}

Node3D* instantiateModel(Node* parent, ModelId model)
{
    if (!parent) return nullptr;
    Node3D* root = buildModel(model);
    if (root) NodeFactory::attach(parent, root);
    return root;
}

} // namespace prisma3d
