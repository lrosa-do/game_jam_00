#include "Check.h"

#include "prisma3d/Device.h"
#include "prisma3d/core/FileSystem.h"
#include "prisma3d/import/ModelImporter.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/render/AssetLoader.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/AnimationPlayer.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/ModelInstance.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Skeleton3D.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/VisualInstance3D.h"

#include "platform.h"

#include <math.h>
#include <string.h>

#include <ct/string.hpp>
#include <ct/vector.hpp>

using namespace prisma3d;

namespace
{

typedef ct::Vector<std::uint8_t> Bytes;

const int kWidth = 64;
const int kHeight = 48;
const float kTolerance = 1e-3f;

bool near(float a, float b)
{
    return fabsf(a - b) <= kTolerance;
}

bool nearVec3(const Math::Vec3& a, const Math::Vec3& b)
{
    return near(a.x, b.x) && near(a.y, b.y) && near(a.z, b.z);
}

void putU32(Bytes& bytes, std::uint32_t value)
{
    for (int i = 0; i < 4; ++i) bytes.push_back(static_cast<std::uint8_t>(value >> (8 * i)));
}

void putF32(Bytes& bytes, float value)
{
    std::uint32_t raw;
    memcpy(&raw, &value, sizeof(raw));
    putU32(bytes, raw);
}

void putVec3(Bytes& bytes, float x, float y, float z)
{
    putF32(bytes, x);
    putF32(bytes, y);
    putF32(bytes, z);
}

void putQuat(Bytes& bytes, float x, float y, float z, float w)
{
    putF32(bytes, x);
    putF32(bytes, y);
    putF32(bytes, z);
    putF32(bytes, w);
}

void buildSkinBin(Bytes& bin)
{
    bin.clear();
    putVec3(bin, 0.0f, 0.0f, 0.0f);
    putVec3(bin, 1.0f, 0.0f, 0.0f);
    putVec3(bin, 0.0f, 1.0f, 0.0f);

    const std::uint8_t joints[3][4] = { { 0, 1, 0, 0 }, { 1, 0, 0, 0 }, { 0, 1, 0, 0 } };
    for (int i = 0; i < 3; ++i)
        for (int c = 0; c < 4; ++c) bin.push_back(joints[i][c]);

    const std::uint8_t weights[3][4] = { { 255, 0, 0, 0 }, { 204, 51, 0, 0 }, { 100, 50, 0, 0 } };
    for (int i = 0; i < 3; ++i)
        for (int c = 0; c < 4; ++c) bin.push_back(weights[i][c]);

    for (int m = 0; m < 2; ++m)
    {
        for (int c = 0; c < 4; ++c)
        {
            for (int r = 0; r < 4; ++r)
            {
                float value = r == c ? 1.0f : 0.0f;
                if (m == 1 && c == 3 && r == 1) value = -1.0f;
                putF32(bin, value);
            }
        }
    }

    putF32(bin, 1.0f);
    putF32(bin, 1.5f);
    putF32(bin, 2.0f);
    putF32(bin, 1.0f);
    putF32(bin, 2.0f);

    putVec3(bin, 0.0f, 1.0f, 0.0f);
    putVec3(bin, 0.0f, 2.0f, 0.0f);
    putVec3(bin, 0.0f, 3.0f, 0.0f);

    putQuat(bin, 0.0f, 0.0f, 0.0f, 1.0f);
    putQuat(bin, 0.0f, 0.0f, 0.38268343f, 0.92387953f);
    putQuat(bin, 0.0f, 0.0f, 0.70710678f, 0.70710678f);

    putQuat(bin, 0.0f, 0.0f, 0.0f, 1.0f);
    putQuat(bin, 0.0f, 0.0f, 0.70710678f, 0.70710678f);

    putVec3(bin, 9.0f, 9.0f, 9.0f);
    putVec3(bin, 0.0f, 5.0f, 0.0f);
    putVec3(bin, 9.0f, 9.0f, 9.0f);
    putVec3(bin, 9.0f, 9.0f, 9.0f);
    putVec3(bin, 0.0f, 7.0f, 0.0f);
    putVec3(bin, 9.0f, 9.0f, 9.0f);
}

ct::String buildSkinJson()
{
    return ct::String(
            "{\"asset\":{\"version\":\"2.0\"},"
            "\"scene\":0,\"scenes\":[{\"nodes\":[0,1]}],"
            "\"nodes\":["
            "{\"name\":\"body\",\"mesh\":0,\"skin\":0},"
            "{\"name\":\"root\",\"scale\":[2,2,2],\"children\":[2]},"
            "{\"name\":\"spine\",\"translation\":[0,1,0]}],"
            "\"skins\":[{\"joints\":[1,2],\"inverseBindMatrices\":3}],"
            "\"meshes\":[{\"name\":\"skinned\",\"primitives\":[{\"attributes\":{\"POSITION\":0,"
            "\"JOINTS_0\":1,\"WEIGHTS_0\":2}}]}],"
            "\"animations\":["
            "{\"name\":\"wave\",\"samplers\":["
            "{\"input\":4,\"output\":6},{\"input\":4,\"output\":7},{\"input\":5,\"output\":8}],"
            "\"channels\":["
            "{\"sampler\":0,\"target\":{\"node\":2,\"path\":\"translation\"}},"
            "{\"sampler\":1,\"target\":{\"node\":2,\"path\":\"rotation\"}},"
            "{\"sampler\":2,\"target\":{\"node\":1,\"path\":\"rotation\"}},"
            "{\"sampler\":0,\"target\":{\"node\":0,\"path\":\"translation\"}}]}"
            "],\"buffers\":[{\"byteLength\":396}],"
            "\"bufferViews\":["
            "{\"buffer\":0,\"byteOffset\":0,\"byteLength\":36},"
            "{\"buffer\":0,\"byteOffset\":36,\"byteLength\":12},"
            "{\"buffer\":0,\"byteOffset\":48,\"byteLength\":12},"
            "{\"buffer\":0,\"byteOffset\":60,\"byteLength\":128},"
            "{\"buffer\":0,\"byteOffset\":188,\"byteLength\":12},"
            "{\"buffer\":0,\"byteOffset\":200,\"byteLength\":8},"
            "{\"buffer\":0,\"byteOffset\":208,\"byteLength\":36},"
            "{\"buffer\":0,\"byteOffset\":244,\"byteLength\":48},"
            "{\"buffer\":0,\"byteOffset\":292,\"byteLength\":32},"
            "{\"buffer\":0,\"byteOffset\":324,\"byteLength\":72}],"
            "\"accessors\":["
            "{\"bufferView\":0,\"componentType\":5126,\"count\":3,\"type\":\"VEC3\","
            "\"min\":[0,0,0],\"max\":[1,1,0]},"
            "{\"bufferView\":1,\"componentType\":5121,\"count\":3,\"type\":\"VEC4\"},"
            "{\"bufferView\":2,\"componentType\":5121,\"normalized\":true,\"count\":3,\"type\":"
            "\"VEC4\"},"
            "{\"bufferView\":3,\"componentType\":5126,\"count\":2,\"type\":\"MAT4\"},"
            "{\"bufferView\":4,\"componentType\":5126,\"count\":3,\"type\":\"SCALAR\"},"
            "{\"bufferView\":5,\"componentType\":5126,\"count\":2,\"type\":\"SCALAR\"},"
            "{\"bufferView\":6,\"componentType\":5126,\"count\":3,\"type\":\"VEC3\"},"
            "{\"bufferView\":7,\"componentType\":5126,\"count\":3,\"type\":\"VEC4\"},"
            "{\"bufferView\":8,\"componentType\":5126,\"count\":2,\"type\":\"VEC4\"},"
            "{\"bufferView\":9,\"componentType\":5126,\"count\":6,\"type\":\"VEC3\"}]}");
}

void buildGlb(const ct::String& json, const Bytes& bin, Bytes& glb)
{
    glb.clear();
    const std::uint32_t jsonLength =
            static_cast<std::uint32_t>((json.size() + 3) & ~std::size_t(3));
    const std::uint32_t binLength = static_cast<std::uint32_t>((bin.size() + 3) & ~std::size_t(3));
    putU32(glb, 0x46546C67u);
    putU32(glb, 2);
    putU32(glb, 12 + 8 + jsonLength + 8 + binLength);
    putU32(glb, jsonLength);
    putU32(glb, 0x4E4F534Au);
    for (std::size_t i = 0; i < json.size(); ++i) glb.push_back(static_cast<std::uint8_t>(json[i]));
    for (std::size_t i = json.size(); i < jsonLength; ++i) glb.push_back(' ');
    putU32(glb, binLength);
    putU32(glb, 0x004E4942u);
    for (std::size_t i = 0; i < bin.size(); ++i) glb.push_back(bin[i]);
    for (std::size_t i = bin.size(); i < binLength; ++i) glb.push_back(0);
}

ct::String path(const char* directory, const char* name)
{
    ct::String result(directory);
    result += "/";
    result += name;
    return result;
}

struct Frame
{
    explicit Frame(SceneTree& tree)
    {
        tree.process(1.0 / 60.0);
        Main::rendering()->renderViewports();
    }

    ~Frame()
    {
        Main::rendering()->endFrame(true);
    }

    const RenderStats& stats() const
    {
        return Main::rendering()->getStats();
    }
};

struct Reference
{
    Math::Vec3 positions[3];
};

Reference referenceAt(const ModelData& document, float time, bool animated)
{
    Reference result;
    const Skeleton& skeleton = document.skeletons[0];
    ct::Vector<LocalPose> pose;
    skeleton.bindPose(pose);
    if (animated) document.animations[0].sample(time, pose);
    ct::Vector<Math::Mat4> globals;
    ct::Vector<Math::Mat4> palette;
    skeleton.evaluate(pose, globals, palette);

    const Math::Vec3 positions[3] = { Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(1.0f, 0.0f, 0.0f),
        Math::Vec3(0.0f, 1.0f, 0.0f) };
    const int joints[3][2] = { { 0, 1 }, { 1, 0 }, { 0, 1 } };
    const float weights[3][2] = { { 1.0f, 0.0f }, { 0.8f, 0.2f }, { 2.0f / 3.0f, 1.0f / 3.0f } };
    for (int v = 0; v < 3; ++v)
    {
        Math::Vec3 sum(0.0f, 0.0f, 0.0f);
        for (int j = 0; j < 2; ++j)
            sum += palette[static_cast<std::size_t>(joints[v][j])].TransformPoint(positions[v]) *
                   weights[v][j];
        result.positions[v] = sum;
    }
    return result;
}

bool readSkinned(InstanceId instance, const SurfaceInfo& surface, Math::Vec3* out)
{
    prisma::Driver* driver = Main::device()->driver();
    const prisma::BufferHandle buffer = Main::rendering()->instanceSkinnedVertexBuffer(instance, 0);
    if (!buffer.valid()) return false;
    Bytes bytes;
    bytes.resize(surface.vertexCount * surface.vertexStride);
    if (!driver->readBuffer(buffer, 0, static_cast<std::uint32_t>(bytes.size()), bytes.data()))
        return false;
    const std::uint32_t offset = MeshStore::attributeOffset(surface.attributeMask, kVertexPosition);
    for (std::uint32_t v = 0; v < surface.vertexCount && v < 3; ++v)
    {
        const float* p =
                reinterpret_cast<const float*>(bytes.data() + v * surface.vertexStride + offset);
        out[v] = Math::Vec3(p[0], p[1], p[2]);
    }
    return true;
}

bool checkSkinned(InstanceId instance, const SurfaceInfo& surface, const Reference& expected,
        const char* label)
{
    Math::Vec3 actual[3];
    if (!readSkinned(instance, surface, actual))
    {
        printf("%s: skinned buffer could not be read\n", label);
        return false;
    }
    bool ok = true;
    for (int v = 0; v < 3; ++v)
    {
        if (nearVec3(actual[v], expected.positions[v])) continue;
        printf("%s: vertex %d is %f %f %f, expected %f %f %f\n", label, v, actual[v].x, actual[v].y,
                actual[v].z, expected.positions[v].x, expected.positions[v].y,
                expected.positions[v].z);
        ok = false;
    }
    return ok;
}

bool loadModel(AssetLoader& loader, ModelId id)
{
    const ModelInfo* info = loader.model(id);
    if (!info) return false;
    const std::uint64_t start = time_nanos();
    for (int i = 0; i < 10000; ++i)
    {
        if (time_nanos() - start > 20000000000ull) break;
        loader.pump(64);
        if (info->status != LoadStatus::InProgress) break;
        time_sleep(1);
    }
    return info->status == LoadStatus::Loaded;
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "model skin test";
    desc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    desc.width = kWidth;
    desc.height = kHeight;
    desc.headless = true;

    CHECK(Main::setup(desc));
    if (!Main::device()) return 1;

    const char* directory =
            vulkan ? PRISMA3D_TEST_DIR "/model_skin_vulkan" : PRISMA3D_TEST_DIR "/model_skin";
    fs_create_directory(directory);
    FileSystem::getSingleton().addSearchPath(directory);

    SceneTree tree;
    CHECK(Main::start(&tree));
    Node* root = tree.getRoot();
    Window* window = root ? root->castTo<Window>() : nullptr;
    CHECK(window != nullptr);
    if (!window) return 1;
    window->setClearColor(Math::Vec4(0.1f, 0.2f, 0.3f, 1.0f));
    Node3D holder;
    Camera3D* camera = root->createChild<Camera3D>();
    camera->setPosition(Math::Vec3(0.0f, 1.0f, 8.0f));
    camera->makeCurrent();

    AssetLoader* loader = Main::assets();
    CHECK(loader != nullptr);
    if (!loader) return 1;

    Bytes bin;
    Bytes glb;
    buildSkinBin(bin);
    buildGlb(buildSkinJson(), bin, glb);
    const ct::String file = path(directory, "skin.glb");
    CHECK(file_write(file.c_str(), glb.data(), glb.size()));

    const ModelId id = loader->requestModel(file);
    CHECK(id.valid());
    CHECK(loadModel(*loader, id));
    const ModelInfo* info = loader->model(id);
    CHECK(info != nullptr && info->parsed);
    if (!info || !info->parsed) return 1;
    CHECK(info->document.skeletons.size() == 1);
    CHECK(info->document.animations.size() == 1);
    CHECK(info->meshes.size() == 1);
    if (info->meshes.size() != 1) return 1;
    const MeshInfo* meshInfo = Main::meshes()->get(info->meshes[0]);
    CHECK(meshInfo != nullptr && meshInfo->surfaces.size() == 1);
    if (!meshInfo || meshInfo->surfaces.size() != 1) return 1;
    const SurfaceInfo& surface = meshInfo->surfaces[0];
    CHECK(surface.skinned());
    CHECK(surface.boneAabbs.size() == 2);

    Node3D* model = instantiateModel(&holder, id);
    CHECK(model != nullptr);
    if (!model) return 1;
    CHECK(model->getName() == "skin");
    CHECK(model->getChildCount() == 1);
    Skeleton3D* skeleton =
            model->getChildCount() == 1 ? model->getChild(0)->castTo<Skeleton3D>() : nullptr;
    CHECK(skeleton != nullptr);
    if (!skeleton) return 1;
    CHECK(skeleton->getName() == "Skeleton3D");
    CHECK(skeleton->getBoneCount() == 2);
    CHECK(skeleton->findBone("root") == 0 && skeleton->findBone("spine") == 1);
    CHECK(nearVec3(skeleton->getBoneRestPose(0).scale, Math::Vec3(2.0f, 2.0f, 2.0f)));
    CHECK(skeleton->getChildCount() == 2);
    MeshInstance3D* body = skeleton->findChild("body")
                                   ? skeleton->findChild("body")->castTo<MeshInstance3D>()
                                   : nullptr;
    AnimationPlayer* player =
            skeleton->findChild("AnimationPlayer")
                    ? skeleton->findChild("AnimationPlayer")->castTo<AnimationPlayer>()
                    : nullptr;
    CHECK(body != nullptr && player != nullptr);
    if (!body || !player) return 1;
    CHECK(body->getMesh() == info->meshes[0]);
    CHECK(body->getTransform() == Math::Mat4::Identity());
    CHECK(body->getSkeletonPath() == "..");
    CHECK(player->getAnimationCount() == 1);
    CHECK(player->hasAnimation("wave"));
    CHECK(player->getAnimation("wave") == &info->document.animations[0]);
    CHECK(model->findChild("root") == nullptr && model->findChild("spine") == nullptr);

    model->reparent(root);
    CHECK(body->getSkeleton() == skeleton);
    CHECK(player->getSkeleton() == skeleton);
    RenderingServer* server = Main::rendering();
    CHECK(server->instanceGetSkeleton(body->getInstance()) == skeleton->getSkeletonId());

    {
        Frame frame(tree);
        CHECK(frame.stats().skinDispatches == 1);
        CHECK(frame.stats().skinnedSurfaces == 1);
    }
    const Reference rest = referenceAt(info->document, 0.0f, false);
    CHECK(nearVec3(rest.positions[1], Math::Vec3(2.0f, 0.0f, 0.0f)));
    CHECK(checkSkinned(body->getInstance(), surface, rest, "rest pose"));
    {
        Frame frame(tree);
        CHECK(frame.stats().skinDispatches == 0);
    }

    player->play("wave");
    {
        Frame frame(tree);
        CHECK(frame.stats().skinDispatches == 1);
    }
    CHECK(checkSkinned(body->getInstance(), surface, referenceAt(info->document, 0.0f, true),
            "animation start"));

    CHECK(near(static_cast<float>(player->getCurrentAnimationLength()), 1.0f));
    player->seek(2.0, true);
    {
        Frame frame(tree);
        CHECK(frame.stats().skinDispatches == 1);
    }
    CHECK(near(static_cast<float>(player->getCurrentAnimationPosition()), 1.0f));
    const Reference end = referenceAt(info->document, 1.0f, true);
    CHECK(checkSkinned(body->getInstance(), surface, end, "animation end"));
    CHECK(!nearVec3(end.positions[1], rest.positions[1]));
    const Math::Box box = server->instanceGetAabb(body->getInstance());
    for (int v = 0; v < 3; ++v) CHECK(box.Expanded(kTolerance).Contains(end.positions[v]));
    CHECK(!box.Contains(Math::Vec3(2.0f, 0.0f, 0.0f)));
    CHECK(body->getAabb().Contains(Math::Vec3(1.0f, 1.0f, 0.0f)));

    player->pause();
    {
        Frame frame(tree);
        CHECK(frame.stats().skinDispatches == 0);
    }

    model->reparent(&holder);
    CHECK(!server->instanceGetSkeleton(body->getInstance()).valid());
    model->queueFree();
    {
        Frame frame(tree);
        CHECK(frame.stats().skinDispatches == 0);
        CHECK(frame.stats().skinnedSurfaces == 0);
    }
    CHECK(loader->freeModel(id));

    ModelOptions options;
    options.collisionType = 2;
    const ModelId solid = loader->requestModel(file, options);
    CHECK(solid.valid());
    CHECK(loadModel(*loader, solid));
    const ModelInfo* solidInfo = loader->model(solid);
    CHECK(solidInfo != nullptr && solidInfo->meshes.size() == 1);
    if (solidInfo && solidInfo->meshes.size() == 1)
    {
        const MeshInfo* solidMesh = Main::meshes()->get(solidInfo->meshes[0]);
        const CollisionMesh* collision = Main::meshes()->collision(solidInfo->meshes[0]);
        CHECK(solidMesh != nullptr && solidMesh->surfaces.size() == 1);
        CHECK(collision != nullptr && collision->valid());
        if (solidMesh && collision && solidMesh->surfaces.size() == 1)
            CHECK(collision->triangleCount() == solidMesh->surfaces[0].vertexCount / 3);
        Node3D* solidModel = instantiateModel(&holder, solid);
        CHECK(solidModel != nullptr);
        if (solidModel)
        {
            Node* solidSkeleton =
                    solidModel->getChildCount() == 1 ? solidModel->getChild(0) : nullptr;
            Node* solidBody = solidSkeleton ? solidSkeleton->findChild("body") : nullptr;
            MeshInstance3D* instance = solidBody ? solidBody->castTo<MeshInstance3D>() : nullptr;
            CHECK(instance != nullptr && instance->getCollisionType() == 2);
            CHECK(instance != nullptr && instance->getCollisionShape() == CollisionShapeType::Mesh);
            solidModel->queueFree();
            tree.process(1.0 / 60.0);
        }
    }
    CHECK(loader->freeModel(solid));

    fs_remove_path(file.c_str());
    camera->queueFree();
    tree.finalize();
    Main::cleanup();
    fs_remove_path(directory);

    if (failures == 0) printf("ok\n");
    return failures == 0 ? 0 : 1;
}
