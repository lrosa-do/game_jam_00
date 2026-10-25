#include "Check.h"
#include "ColouredBox.h"
#include "TestFiles.h"

#include "prisma3d/Device.h"
#include "prisma3d/audio/AudioServer.h"
#include "prisma3d/core/FileSystem.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/render/AssetLoader.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/TextureStore.h"
#include "prisma3d/scene/AudioStreamPlayer.h"
#include "prisma3d/scene/AudioStreamPlayer3D.h"
#include "prisma3d/scene/Decal3D.h"
#include "prisma3d/scene/Destructible3D.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/PrimitiveMeshes.h"
#include "prisma3d/scene/RibbonTrail3D.h"
#include "prisma3d/scene/SceneResources.h"
#include "prisma3d/scene/SceneSerializer.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/VisualInstance3D.h"

#include "platform.h"

#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <math.h>
#include <string.h>

using namespace prisma3d;

namespace
{

ct::String gBase;

struct Row
{
    const char* className;
    const char* nodeName;
    const char* property;
    const char* path;
};

const char kPack[] = "models/pack.glb";
const char kSphere[] = "primitive:sphere?radius=0.5&rings=8&slices=12";

const Row kRows[] = {
    { "MeshInstance3D", "mesh", "mesh", "models/pack.glb::Body" },
    { "MeshInstance3D", "mesh", "material_override", "libs/paint.mat::wood" },
    { "MultiMeshInstance3D", "multi", "mesh", kSphere },
    { "MultiMeshInstance3D", "multi", "material_override", "models/pack.glb::paint" },
    { "AudioStreamPlayer", "sound", "stream", "audio/tone.wav" },
    { "AudioStreamPlayer3D", "sound3d", "stream", "audio/tone.wav" },
    { "Decal3D", "decal", "texture_albedo", "tex/albedo.png" },
    { "Decal3D", "decal", "texture_normal", "tex/normal.png" },
    { "Decal3D", "decal", "texture_orm", "tex/orm.png" },
    { "Decal3D", "decal", "texture_emission", "tex/emission.png" },
    { "DestructibleBody3D", "crate", "outer_material", "libs/paint.mat::wood" },
    { "DestructibleBody3D", "crate", "inner_material", "libs/paint.mat::metal" },
    { "Hair3D", "hair", "material", "libs/paint.mat::metal" },
    { "Hair3D", "hair", "texture", "models/pack.glb::image0" },
    { "ParticleSystem3D", "smoke", "texture", "tex/particle.png" },
    { "RibbonTrail3D", "ribbon", "material", "libs/paint.mat::wood" },
    { "Road3D", "road", "material", "libs/paint.mat::metal" },
    { "Terrain3D", "ground", "material", "libs/paint.mat::wood" },
    { "Tree3D", "tree", "bark_material", "libs/paint.mat::wood" },
    { "Tree3D", "tree", "leaf_material", "libs/paint.mat::metal" },
    { "Water3D", "water", "normal_texture", "tex/water.png" },
    { "WorldEnvironment", "env", "sky_texture", "tex/sky.png" },
    { "WorldEnvironment", "env", "glow_map", "tex/glow.png" },
    { "Node3D", "solid", "collision_mesh", "models/pack.glb::#1" },
};

const size_t kRowCount = sizeof(kRows) / sizeof(kRows[0]);

bool hasDiagnostic(const SceneResult& result, SceneDiagnosticSeverity severity, const char* path,
        const char* fragment)
{
    for (size_t i = 0; i < result.diagnostics.size(); ++i)
    {
        const SceneDiagnostic& diagnostic = result.diagnostics[i];
        if (diagnostic.severity != severity) continue;
        if (path && strcmp(diagnostic.path.c_str(), path) != 0) continue;
        if (!fragment || strstr(diagnostic.message.c_str(), fragment)) return true;
    }
    return false;
}

void expectClean(const SceneResult& result, const char* what)
{
    if (result.diagnostics.size() == 0) return;
    printf("%s:\n%s", what, result.describe().c_str());
    ++failures;
}

ct::String under(const char* relative)
{
    return gBase + "/" + relative;
}

bool writePng(const char* relative, int width, int height, int red, int green, int blue)
{
    Pixmap pixmap(width, height, 4);
    pixmap.fill(static_cast<std::uint8_t>(red), static_cast<std::uint8_t>(green),
            static_cast<std::uint8_t>(blue), 255);
    return pixmap.save(under(relative).c_str());
}

bool writeHeightmap(const char* relative)
{
    Pixmap pixmap(8, 8, 4);
    for (int y = 0; y < 8; ++y)
    {
        for (int x = 0; x < 8; ++x)
        {
            const std::uint8_t value = static_cast<std::uint8_t>(x * 16 + y * 8);
            pixmap.set_pixel(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y), value,
                    value, value, 255);
        }
    }
    return pixmap.save(under(relative).c_str());
}

void put16(ct::Vector<std::uint8_t>& out, std::uint32_t value)
{
    out.push_back(static_cast<std::uint8_t>(value & 0xffu));
    out.push_back(static_cast<std::uint8_t>((value >> 8) & 0xffu));
}

void put32(ct::Vector<std::uint8_t>& out, std::uint32_t value)
{
    put16(out, value & 0xffffu);
    put16(out, value >> 16);
}

void putTag(ct::Vector<std::uint8_t>& out, const char* tag)
{
    for (int i = 0; i < 4; ++i) out.push_back(static_cast<std::uint8_t>(tag[i]));
}

ct::Vector<std::uint8_t> makeWav()
{
    const std::uint32_t rate = 22050;
    const std::uint32_t frames = 2205;
    ct::Vector<std::uint8_t> bytes;
    putTag(bytes, "RIFF");
    put32(bytes, 36 + frames * 2);
    putTag(bytes, "WAVE");
    putTag(bytes, "fmt ");
    put32(bytes, 16);
    put16(bytes, 1);
    put16(bytes, 1);
    put32(bytes, rate);
    put32(bytes, rate * 2);
    put16(bytes, 2);
    put16(bytes, 16);
    putTag(bytes, "data");
    put32(bytes, frames * 2);
    for (std::uint32_t i = 0; i < frames; ++i)
    {
        const float sample = 0.4f * sinf(2.0f * 3.14159265f * 440.0f * static_cast<float>(i) /
                                            static_cast<float>(rate));
        put16(bytes,
                static_cast<std::uint32_t>(static_cast<std::int16_t>(sample * 32767.0f)) & 0xffffu);
    }
    return bytes;
}

bool writeBytes(const char* relative, const ct::Vector<std::uint8_t>& bytes)
{
    return file_write(under(relative).c_str(), bytes.data(), bytes.size());
}

bool writeGlb(const char* relative, const ct::Vector<std::uint8_t>& png)
{
    ct::Vector<std::uint8_t> bin;
    const float positions[9] = { 0, 0, 0, 1, 0, 0, 0, 1, 0 };
    for (int i = 0; i < 9; ++i)
    {
        std::uint32_t raw;
        memcpy(&raw, &positions[i], sizeof(raw));
        put32(bin, raw);
    }
    const size_t imageOffset = bin.size();
    for (size_t i = 0; i < png.size(); ++i) bin.push_back(png[i]);
    while (bin.size() % 4 != 0) bin.push_back(0);

    ct::String json =
            "{\"asset\":{\"version\":\"2.0\"},\"scene\":0,\"scenes\":[{\"nodes\":[0,1,2]}],"
            "\"nodes\":[{\"mesh\":0},{\"mesh\":1},{\"mesh\":2}],"
            "\"meshes\":["
            "{\"name\":\"Body\",\"primitives\":[{\"attributes\":{\"POSITION\":0},"
            "\"material\":0}]},"
            "{\"primitives\":[{\"attributes\":{\"POSITION\":0}}]},"
            "{\"name\":\"Body\",\"primitives\":[{\"attributes\":{\"POSITION\":0}}]}],"
            "\"materials\":[{\"name\":\"paint\",\"pbrMetallicRoughness\":{\"baseColorFactor\":"
            "[1,0,0,1],\"baseColorTexture\":{\"index\":0}}}],"
            "\"textures\":[{\"source\":0}],"
            "\"images\":[{\"bufferView\":1,\"mimeType\":\"image/png\"}],"
            "\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":3,"
            "\"type\":\"VEC3\",\"min\":[0,0,0],\"max\":[1,1,0]}],"
            "\"bufferViews\":[{\"buffer\":0,\"byteOffset\":0,\"byteLength\":36},"
            "{\"buffer\":0,\"byteOffset\":";
    json.append_number(static_cast<unsigned long long>(imageOffset));
    json += ",\"byteLength\":";
    json.append_number(static_cast<unsigned long long>(png.size()));
    json += "}],\"buffers\":[{\"byteLength\":";
    json.append_number(static_cast<unsigned long long>(bin.size()));
    json += "}]}";
    while (json.size() % 4 != 0) json += ' ';

    ct::Vector<std::uint8_t> file;
    putTag(file, "glTF");
    put32(file, 2);
    put32(file, static_cast<std::uint32_t>(12 + 8 + json.size() + 8 + bin.size()));
    put32(file, static_cast<std::uint32_t>(json.size()));
    putTag(file, "JSON");
    for (size_t i = 0; i < json.size(); ++i) file.push_back(static_cast<std::uint8_t>(json[i]));
    put32(file, static_cast<std::uint32_t>(bin.size()));
    file.push_back('B');
    file.push_back('I');
    file.push_back('N');
    file.push_back(0);
    for (size_t i = 0; i < bin.size(); ++i) file.push_back(bin[i]);
    return writeBytes(relative, file);
}

bool writeFiles()
{
    const char* folders[] = { "tex", "models", "libs", "audio", "scenes" };
    for (const char* folder: folders)
        if (!fs_create_directory(under(folder).c_str())) return false;
    bool ok = writePng("tex/albedo.png", 4, 4, 200, 120, 60);
    ok = ok && writePng("tex/normal.png", 4, 4, 128, 128, 255);
    ok = ok && writePng("tex/orm.png", 4, 4, 255, 128, 0);
    ok = ok && writePng("tex/emission.png", 4, 4, 255, 255, 0);
    ok = ok && writePng("tex/particle.png", 4, 4, 255, 255, 255);
    ok = ok && writePng("tex/water.png", 4, 4, 100, 100, 255);
    ok = ok && writePng("tex/sky.png", 8, 4, 60, 100, 200);
    ok = ok && writePng("tex/glow.png", 4, 4, 30, 30, 30);
    ok = ok && writePng("tex/embedded.png", 2, 2, 10, 220, 30);
    ok = ok && writeHeightmap("tex/height.png");
    if (!ok) return false;
    ct::Vector<std::uint8_t> png = FileSystem::getSingleton().readBinary(under("tex/embedded.png"));
    if (png.empty() || !writeGlb("models/pack.glb", png)) return false;
    if (!writeBytes("audio/tone.wav", makeWav())) return false;
    return FileSystem::writeText(under("libs/paint.mat"),
            "material \"wood\" {\n    roughness = 0.4\n    color = (0.5, 0.25, 0.1, 1)\n}\n"
            "material \"metal\" {\n    metallic = 1\n    roughness = 0.2\n}\n");
}

struct Loaded
{
    MeshId meshes[4];
    MaterialId materials[4];
    TextureId textures[4];
    SoundId sound;
};

Node* findNode(Node* scene, const char* name)
{
    return scene ? scene->findChild(name) : nullptr;
}

Node* buildWorld(Node* root)
{
    Node* scene = root->createChild("Node3D");
    scene->setName("scene");
    const char* lastName = "";
    Node* node = nullptr;
    for (size_t i = 0; i < kRowCount; ++i)
    {
        const Row& row = kRows[i];
        if (strcmp(lastName, row.nodeName) != 0)
        {
            node = scene->createChild(row.className);
            CHECK(node != nullptr);
            if (!node) return scene;
            node->setName(row.nodeName);
            lastName = row.nodeName;
            if (strcmp(row.nodeName, "solid") == 0)
                node->set("collision_shape", PropertyValue::fromInt(2));
        }
        CHECK(node->set(row.property, PropertyValue::fromResourcePath(row.path)));
    }
    return scene;
}

void checkRows(Node* scene, const char* what)
{
    for (size_t i = 0; i < kRowCount; ++i)
    {
        const Row& row = kRows[i];
        Node* node = findNode(scene, row.nodeName);
        CHECK(node != nullptr);
        if (!node) continue;
        bool valid = false;
        const PropertyValue value = node->get(row.property, &valid);
        const bool same = valid && value == PropertyValue::fromResourcePath(row.path);
        if (!same)
        {
            printf("FAIL %s: %s.%s is '%s', expected '%s'\n", what, row.nodeName, row.property,
                    valid ? value.asString().c_str() : "(unreadable)", row.path);
            ++failures;
        }
    }
}

void testRoundTrip(Node* root, SceneTree& tree)
{
    MeshStore& meshes = *Main::meshes();
    TextureStore& textures = *Main::textures();
    MaterialStore& materials = *Main::materials();
    AudioServer& audio = *Main::audio();
    AssetLoader& assets = *Main::assets();

    Node* scene = buildWorld(root);
    checkRows(scene, "after setting");

    MeshInstance3D* instance = static_cast<MeshInstance3D*>(findNode(scene, "mesh"));
    CHECK(instance && instance->getMesh().valid());
    const MeshId oldMesh = instance ? instance->getMesh() : MeshId();
    const MaterialId wood = instance ? instance->getMaterialOverride() : MaterialId();
    CHECK(wood.valid());
    AudioStreamPlayer* player = static_cast<AudioStreamPlayer*>(findNode(scene, "sound"));
    AudioStreamPlayer3D* player3d = static_cast<AudioStreamPlayer3D*>(findNode(scene, "sound3d"));
    CHECK(player && player3d && player->getSound().valid());
    CHECK(player && player3d && player->getSound() == player3d->getSound());
    CHECK(audio.getSoundCount() == 1);
    const SoundId oldSound = player ? player->getSound() : SoundId();
    Decal3D* decal = static_cast<Decal3D*>(findNode(scene, "decal"));
    const TextureId oldAlbedo = decal ? decal->getTexture(DecalTexture::Albedo) : TextureId();
    CHECK(oldAlbedo.valid());
    CHECK(sceneres::loadTexture("tex/albedo.png", true) == oldAlbedo);
    CHECK(sceneres::loadTexture("tex//./albedo.png", true) == oldAlbedo);
    CHECK(sceneres::loadTexture("tex/albedo.png", false) != oldAlbedo);
    CHECK(textures.get(oldAlbedo)->srgb);
    CHECK(decal && !textures.get(decal->getTexture(DecalTexture::Normal))->srgb);
    CHECK(sceneres::loadMaterial("libs/paint.mat::wood") == wood);
    DestructibleBody3D* crate = static_cast<DestructibleBody3D*>(findNode(scene, "crate"));
    RibbonTrail3D* ribbon = static_cast<RibbonTrail3D*>(findNode(scene, "ribbon"));
    CHECK(crate && crate->getOuterMaterial() == wood);
    CHECK(ribbon && ribbon->getAssignedMaterial() == wood);
    CHECK(crate && crate->getInnerMaterial() != wood && crate->getInnerMaterial().valid());
    CHECK(sceneres::loadMesh("models/pack.glb::Body") == oldMesh);
    CHECK(sceneres::loadMesh("models/pack.glb") == oldMesh);
    CHECK(sceneres::pathOf(oldMesh) == "models/pack.glb::Body");
    const MeshId second = sceneres::loadMesh("models/pack.glb::#2");
    CHECK(second.valid() && second != oldMesh && sceneres::pathOf(second) == "models/pack.glb::#2");
    CHECK(!sceneres::loadMesh("models/pack.glb::Body2").valid());
    CHECK(sceneres::takeError().contains("models/pack.glb::Body2"));
    CHECK(sceneres::pathOf(wood) == "libs/paint.mat::wood");
    const MaterialId paint = sceneres::loadMaterial("models/pack.glb::paint");
    CHECK(paint.valid() && materials.get(paint)->path == kPack);
    CHECK(materials.get(paint)->textures[0].valid());
    CHECK(sceneres::pathOf(materials.get(paint)->textures[0]) == "models/pack.glb::image0");

    SceneResult saved;
    ct::String text;
    CHECK(SceneSerializer::save(scene, text, &saved));
    expectClean(saved, "save");
    for (size_t i = 0; i < kRowCount; ++i) CHECK(text.contains(kRows[i].path));
    CHECK(SceneSerializer::saveFile(scene, under("scenes/world.scene.json"), &saved));
    expectClean(saved, "save file");

    scene->queueFree();
    tree.process(1.0 / 60.0);
    meshes.clear();
    textures.clear();
    materials.clear();
    audio.destroySound(oldSound);
    CHECK(!sceneres::alive(oldMesh) && !sceneres::alive(wood) && !sceneres::alive(oldAlbedo));
    CHECK(!sceneres::alive(oldSound));
    CHECK(meshes.count() == 0 && materials.count() == 0 && audio.getSoundCount() == 0);

    SceneResult loadedResult;
    Node* loaded = SceneSerializer::loadFile(under("scenes/world.scene.json"), root, loadedResult);
    expectClean(loadedResult, "load");
    CHECK(loaded != nullptr);
    checkRows(loaded, "after loading");
    if (!loaded) return;

    MeshInstance3D* again = static_cast<MeshInstance3D*>(findNode(loaded, "mesh"));
    CHECK(again && again->getMesh().valid() && again->getMesh() != oldMesh);
    CHECK(again && sceneres::alive(again->getMesh()) &&
            sceneres::alive(again->getMaterialOverride()));
    CHECK(again && sceneres::pathOf(again->getMesh()) == "models/pack.glb::Body");
    CHECK(sceneres::pathOf(again->getMaterialOverride()) == "libs/paint.mat::wood");
    AudioStreamPlayer* playerAgain = static_cast<AudioStreamPlayer*>(findNode(loaded, "sound"));
    AudioStreamPlayer3D* player3dAgain =
            static_cast<AudioStreamPlayer3D*>(findNode(loaded, "sound3d"));
    CHECK(playerAgain && player3dAgain && playerAgain->getSound().valid());
    CHECK(playerAgain && player3dAgain && playerAgain->getSound() == player3dAgain->getSound());
    CHECK(audio.getSoundCount() == 1);
    CHECK(sceneres::pathOf(playerAgain->getSound()) == "audio/tone.wav");
    Node3D* solid = static_cast<Node3D*>(findNode(loaded, "solid"));
    CHECK(solid && solid->getStoredCollisionShape() == CollisionShapeType::Box);
    CHECK(solid && sceneres::pathOf(solid->getStoredCollisionMesh()) == "models/pack.glb::#1");
    Decal3D* decalAgain = static_cast<Decal3D*>(findNode(loaded, "decal"));
    CHECK(decalAgain && textures.get(decalAgain->getTexture(DecalTexture::Albedo))->srgb);
    CHECK(decalAgain && !textures.get(decalAgain->getTexture(DecalTexture::Normal))->srgb);
    CHECK(decalAgain && !textures.get(decalAgain->getTexture(DecalTexture::Orm))->srgb);
    CHECK(decalAgain && textures.get(decalAgain->getTexture(DecalTexture::Emission))->srgb);

    const size_t meshesBefore = meshes.count();
    const size_t texturesBefore = textures.count();
    const size_t materialsBefore = materials.count();
    const size_t soundsBefore = audio.getSoundCount();
    const size_t ownedMeshes = meshesBefore - 4;
    const size_t ownedTextures = texturesBefore - 9;
    const size_t ownedMaterials = materialsBefore - 3;
    CHECK(ownedMaterials > 0);

    const std::uint32_t requests = assets.totalRequests();
    SceneResult secondResult;
    Node* secondScene =
            SceneSerializer::loadFile(under("scenes/world.scene.json"), root, secondResult);
    expectClean(secondResult, "load again");
    CHECK(secondScene != nullptr);
    if (meshes.count() != meshesBefore + ownedMeshes ||
            textures.count() != texturesBefore + ownedTextures ||
            materials.count() != materialsBefore + ownedMaterials ||
            audio.getSoundCount() != soundsBefore)
        printf("FAIL: meshes %zu -> %zu, textures %zu -> %zu, materials %zu -> %zu, sounds %zu -> "
               "%zu\n",
                meshesBefore, meshes.count(), texturesBefore, textures.count(), materialsBefore,
                materials.count(), soundsBefore, audio.getSoundCount());
    CHECK(meshes.count() == meshesBefore + ownedMeshes);
    CHECK(textures.count() == texturesBefore + ownedTextures);
    CHECK(materials.count() == materialsBefore + ownedMaterials);
    CHECK(audio.getSoundCount() == soundsBefore);
    CHECK(assets.totalRequests() == requests);
    checkRows(secondScene, "after loading twice");
    MeshInstance3D* twice = static_cast<MeshInstance3D*>(findNode(secondScene, "mesh"));
    CHECK(twice && again && twice->getMesh() == again->getMesh());
    CHECK(twice && again && twice->getMaterialOverride() == again->getMaterialOverride());

    if (secondScene) secondScene->queueFree();
    loaded->queueFree();
    tree.process(1.0 / 60.0);
}

struct PrimitiveCase
{
    MeshId mesh;
    ct::String path;
    std::uint32_t vertices = 0;
    std::uint32_t indices = 0;
    Math::Box box;
};

void testPrimitives(Node* root, SceneTree& tree)
{
    MeshStore& meshes = *Main::meshes();
    ct::Vector<PrimitiveCase> cases;
    ct::Vector<MeshId> made;
    made.push_back(createBoxMesh(meshes, Math::Vec3(1.0f, 2.0f, 3.0f)));
    made.push_back(createPlaneMesh(meshes, 10.0f, 6.0f, 4, 3, 2.0f));
    made.push_back(createSphereMesh(meshes, 0.75f, 10, 14));
    made.push_back(createCylinderMesh(meshes, 0.5f, 2.5f, 12));
    made.push_back(createConeMesh(meshes, 1.25f, 3.0f, 9));
    made.push_back(createCapsuleMesh(meshes, 0.3f, 1.4f, 5, 10));
    made.push_back(createTorusMesh(meshes, 2.0f, 0.4f, 16, 8));
    made.push_back(
            createHillsPlaneMeshFromFile(meshes, 8.0f, 6.0f, 5, 4, "tex/height.png", 2.5f, 3.0f));
    made.push_back(createHeightfieldMeshFromFile(meshes, "tex/height.png", 0.5f, 3.0f, 2.0f));

    const char* expected[] = {
        "primitive:box?size=1,2,3",
        "primitive:plane?width=10&depth=6&segments_x=4&segments_z=3&uv_tiles=2",
        "primitive:sphere?radius=0.75&rings=10&slices=14",
        "primitive:cylinder?radius=0.5&height=2.5&slices=12",
        "primitive:cone?radius=1.25&height=3&slices=9",
        "primitive:capsule?radius=0.3&height=1.4&rings=5&slices=10",
        "primitive:torus?major_radius=2&minor_radius=0.4&major_segments=16&minor_segments=8",
        ("primitive:hills_plane?width=8&depth=6&segments_x=5&segments_z=4&heightmap=tex/"
         "height.png&height_scale=2.5&uv_tiles=3"),
        "primitive:heightfield?heightmap=tex/height.png&cell_size=0.5&height_scale=3&uv_tiles=2",
    };

    Node* scene = root->createChild("Node3D");
    scene->setName("shapes");
    for (size_t i = 0; i < made.size(); ++i)
    {
        PrimitiveCase entry;
        entry.mesh = made[i];
        CHECK(made[i].valid());
        const MeshInfo* info = meshes.get(made[i]);
        CHECK(info != nullptr);
        if (!info) continue;
        entry.path = info->path;
        if (entry.path != expected[i])
        {
            printf("FAIL: primitive %zu has the path '%s', expected '%s'\n", i, entry.path.c_str(),
                    expected[i]);
            ++failures;
        }
        for (size_t s = 0; s < info->surfaces.size(); ++s)
        {
            entry.vertices += info->surfaces[s].vertexCount;
            entry.indices += info->surfaces[s].indexCount;
        }
        entry.box = info->aabb;
        CHECK(entry.vertices > 0 && entry.indices > 0);
        cases.push_back(entry);

        Node* node = scene->createChild("MeshInstance3D");
        ct::String name = "shape";
        name.append_number(static_cast<unsigned long long>(i));
        node->setName(name);
        CHECK(node->set("mesh", PropertyValue::fromResourcePath(info->path)));
        CHECK(static_cast<MeshInstance3D*>(node)->getMesh() == made[i]);
    }

    SceneResult saved;
    ct::String text;
    CHECK(SceneSerializer::save(scene, text, &saved));
    expectClean(saved, "primitives save");
    for (size_t i = 0; i < cases.size(); ++i) CHECK(text.contains(cases[i].path.c_str()));

    scene->queueFree();
    tree.process(1.0 / 60.0);
    meshes.clear();
    CHECK(meshes.count() == 0);

    SceneResult loadedResult;
    Node* loaded = SceneSerializer::load(text, root, loadedResult);
    expectClean(loadedResult, "primitives load");
    CHECK(loaded != nullptr);
    for (size_t i = 0; loaded && i < cases.size(); ++i)
    {
        ct::String name = "shape";
        name.append_number(static_cast<unsigned long long>(i));
        MeshInstance3D* node = static_cast<MeshInstance3D*>(findNode(loaded, name.c_str()));
        CHECK(node != nullptr);
        if (!node) continue;
        const MeshInfo* info = meshes.get(node->getMesh());
        CHECK(info != nullptr);
        if (!info) continue;
        std::uint32_t vertices = 0;
        std::uint32_t indices = 0;
        for (size_t s = 0; s < info->surfaces.size(); ++s)
        {
            vertices += info->surfaces[s].vertexCount;
            indices += info->surfaces[s].indexCount;
        }
        if (vertices != cases[i].vertices || indices != cases[i].indices ||
                info->aabb.min.x != cases[i].box.min.x || info->aabb.min.y != cases[i].box.min.y ||
                info->aabb.min.z != cases[i].box.min.z || info->aabb.max.x != cases[i].box.max.x ||
                info->aabb.max.y != cases[i].box.max.y || info->aabb.max.z != cases[i].box.max.z ||
                info->path != cases[i].path)
        {
            printf("FAIL: primitive %zu came back different (%u/%u vertices, %u/%u indices)\n", i,
                    vertices, cases[i].vertices, indices, cases[i].indices);
            ++failures;
        }
    }
    const size_t meshCount = meshes.count();
    CHECK(sceneres::loadMesh(expected[0]) == sceneres::loadMesh(expected[0]));
    CHECK(sceneres::loadMesh("primitive:box?size=1,2,3") == sceneres::loadMesh(expected[0]));
    CHECK(sceneres::loadMesh("primitive:box?size=1.0,2.00,3") == sceneres::loadMesh(expected[0]));
    CHECK(meshes.count() == meshCount);

    const MeshId plain = sceneres::loadMesh("primitive:sphere");
    CHECK(plain.valid() && sceneres::pathOf(plain) == "primitive:sphere?radius=0.5&rings=16&"
                                                      "slices=24");
    CHECK(sceneres::loadMesh("primitive:sphere?radius=0.5&rings=16&slices=24") == plain);

    const char* bad[] = {
        "primitive:cube?size=1",
        "primitive:box?size=1,2",
        "primitive:box?size=1,2,3,4",
        "primitive:box?size=a,b,c",
        "primitive:sphere?radius=-1",
        "primitive:sphere?rings=2.5",
        "primitive:sphere?rings=1",
        "primitive:sphere?slices=99999",
        "primitive:sphere?bogus=1",
        "primitive:sphere?radius=1&radius=2",
        "primitive:sphere?radius",
        "primitive:sphere?radius=nan",
        "primitive:plane?segments_x=4096&segments_z=4096",
        "primitive:hills_plane?width=1",
        "primitive:hills_plane?heightmap=tex/none.png",
        "primitive:heightfield?heightmap=%zz",
    };
    for (const char* path: bad)
    {
        sceneres::clearError();
        CHECK(!sceneres::loadMesh(path).valid());
        const ct::String error = sceneres::takeError();
        if (!error.contains(path))
        {
            printf("FAIL: no error naming '%s': '%s'\n", path, error.c_str());
            ++failures;
        }
    }

    if (loaded) loaded->queueFree();
    tree.process(1.0 / 60.0);
}

void testMissing(Node* root, SceneTree& tree)
{
    const char* text =
            "{\"format\":\"prisma3d-scene\",\"version\":1,\"root\":{\"class\":\"Node3D\","
            "\"name\":\"gaps\",\"children\":["
            "{\"class\":\"MeshInstance3D\",\"name\":\"a\",\"properties\":{"
            "\"mesh\":\"models/nope.glb::x\",\"material_override\":\"libs/none.mat::x\","
            "\"position\":[1,2,3]}},"
            "{\"class\":\"Decal3D\",\"name\":\"b\",\"properties\":{"
            "\"texture_albedo\":\"tex/none.png\",\"texture_normal\":\"tex/normal.png\"}},"
            "{\"class\":\"AudioStreamPlayer\",\"name\":\"c\",\"properties\":{"
            "\"stream\":\"audio/none.wav\",\"pitch_scale\":2}},"
            "{\"class\":\"MeshInstance3D\",\"name\":\"d\",\"properties\":{"
            "\"mesh\":\"models/pack.glb::Nope\",\"material_override\":\"libs/paint.mat::nope\"}},"
            "{\"class\":\"MeshInstance3D\",\"name\":\"e\",\"properties\":{"
            "\"mesh\":\"primitive:cube\"},\"data\":{\"version\":1,\"surface_materials\":["
            "{\"surface\":0,\"material\":\"libs/none.mat::hook\"}]}},"
            "{\"class\":\"Node3D\",\"name\":\"ok\",\"properties\":{\"position\":[4,5,6]}}]}}";

    SceneResult result;
    Node* gaps = SceneSerializer::load(text, root, result);
    CHECK(gaps != nullptr);
    CHECK(result.success());
    CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Warning,
            "root.children[0].properties.mesh", "models/nope.glb::x"));
    CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Warning,
            "root.children[0].properties.material_override", "libs/none.mat::x"));
    CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Warning,
            "root.children[1].properties.texture_albedo", "tex/none.png"));
    CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Warning,
            "root.children[2].properties.stream", "audio/none.wav"));
    CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Warning,
            "root.children[3].properties.mesh", "models/pack.glb::Nope"));
    CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Warning,
            "root.children[3].properties.material_override", "libs/paint.mat::nope"));
    CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Warning,
            "root.children[4].properties.mesh", "primitive:cube"));
    CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Warning,
            "root.children[4].data.surface_materials[0].material", "libs/none.mat::hook"));
    CHECK(result.warningCount() == 8);
    if (gaps)
    {
        MeshInstance3D* a = static_cast<MeshInstance3D*>(findNode(gaps, "a"));
        CHECK(a && !a->getMesh().valid() && !a->getMaterialOverride().valid());
        CHECK(a && a->getPosition() == Math::Vec3(1.0f, 2.0f, 3.0f));
        Decal3D* b = static_cast<Decal3D*>(findNode(gaps, "b"));
        CHECK(b && !b->getTexture(DecalTexture::Albedo).valid());
        CHECK(b && b->getTexture(DecalTexture::Normal).valid());
        AudioStreamPlayer* c = static_cast<AudioStreamPlayer*>(findNode(gaps, "c"));
        CHECK(c && !c->getSound().valid() && c->getPitch() == 2.0f);
        Node3D* ok = static_cast<Node3D*>(findNode(gaps, "ok"));
        CHECK(ok && ok->getPosition() == Math::Vec3(4.0f, 5.0f, 6.0f));
        gaps->queueFree();
        tree.process(1.0 / 60.0);
    }
}

void testWithoutPath(Node* root, SceneTree& tree)
{
    MeshStore& meshes = *Main::meshes();
    TextureStore& textures = *Main::textures();
    MaterialStore& materials = *Main::materials();
    AudioServer& audio = *Main::audio();

    Node* scene = root->createChild("Node3D");
    scene->setName("anonymous");

    const MeshId mesh = meshes.create("handmade");
    CHECK(meshes.addSurface(mesh, colouredBox()));
    const MaterialId material = materials.create("handmade");
    const TextureId texture = textures.createEmpty(4, 4, true, false, "handmade");
    const ct::Vector<std::uint8_t> wav = makeWav();
    const SoundId sound = audio.createSoundFromMemory(wav.data(), wav.size());
    CHECK(mesh.valid() && material.valid() && texture.valid() && sound.valid());
    CHECK(sceneres::pathOf(mesh).empty() && sceneres::pathOf(material).empty());
    CHECK(sceneres::pathOf(texture).empty() && sceneres::pathOf(sound).empty());

    Node* instance = scene->createChild("MeshInstance3D");
    instance->setName("instance");
    static_cast<MeshInstance3D*>(instance)->setMesh(mesh);
    static_cast<MeshInstance3D*>(instance)->setMaterialOverride(material);
    Node* decal = scene->createChild("Decal3D");
    decal->setName("decal");
    static_cast<Decal3D*>(decal)->setTexture(DecalTexture::Albedo, texture);
    Node* player = scene->createChild("AudioStreamPlayer");
    player->setName("player");
    static_cast<AudioStreamPlayer*>(player)->setSound(sound);
    CHECK(instance->get("mesh") == PropertyValue::fromResourcePath(""));

    SceneResult result;
    ct::String text;
    CHECK(SceneSerializer::save(scene, text, &result));
    CHECK(result.success());
    CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Warning,
            "root.children[0].properties.mesh", "no path"));
    CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Warning,
            "root.children[0].properties.material_override", "no path"));
    CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Warning,
            "root.children[1].properties.texture_albedo", "no path"));
    CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Warning,
            "root.children[2].properties.stream", "no path"));
    CHECK(result.warningCount() == 4);
    CHECK(!text.contains("handmade"));
    CHECK(!text.contains("\"mesh\""));

    scene->queueFree();
    tree.process(1.0 / 60.0);

    Node* defaults = root->createChild("Node3D");
    defaults->setName("defaults");
    const char* classes[] = { "Road3D", "Terrain3D", "Tree3D", "Hair3D", "RibbonTrail3D", "Water3D",
        "ParticleSystem3D", "WorldEnvironment", "DestructibleBody3D", "Decal3D", "MeshInstance3D",
        "MultiMeshInstance3D", "AudioStreamPlayer", "AudioStreamPlayer3D" };
    for (const char* className: classes)
    {
        Node* node = defaults->createChild(className);
        CHECK(node != nullptr);
        if (node) node->setName(className);
    }
    SceneResult fresh;
    ct::String freshText;
    CHECK(SceneSerializer::save(defaults, freshText, &fresh));
    expectClean(fresh, "nodes with default resources");
    CHECK(!freshText.contains("\"properties\""));
    defaults->queueFree();
    tree.process(1.0 / 60.0);

    audio.destroySound(sound);
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    gBase = PRISMA3D_TEST_DIR;
    gBase += vulkan ? "/resource_paths_vulkan" : "/resource_paths";
    removeTree(gBase);
    CHECK(fs_create_directory(gBase.c_str()));
    FileSystem::getSingleton().addSearchPath(gBase);
    CHECK(writeFiles());

    DeviceDesc desc;
    desc.title = "resource paths test";
    desc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    desc.width = 64;
    desc.height = 48;
    desc.headless = true;

    CHECK(Main::setup(desc));
    if (!Main::device()) return 1;
    printf("%s\n", vulkan ? "Vulkan" : "OpenGL");

    PhysicsServer3D physics;
    SceneTree tree;
    CHECK(Main::start(&tree));
    Node* root = tree.getRoot();
    CHECK(root != nullptr);
    if (!root) return 1;
    CHECK(Main::audio() && Main::audio()->isReady());

    testRoundTrip(root, tree);
    testPrimitives(root, tree);
    testMissing(root, tree);
    testWithoutPath(root, tree);

    tree.process(1.0 / 60.0);
    FileSystem::getSingleton().removeSearchPath(gBase);
    removeTree(gBase);

    if (failures)
    {
        printf("%d failures\n", failures);
        return 1;
    }
    printf("resource paths ok\n");
    return 0;
}
