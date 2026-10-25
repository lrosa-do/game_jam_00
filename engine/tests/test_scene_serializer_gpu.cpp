#include "Check.h"
#include "ColouredBox.h"

#include "prisma3d/Device.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/render/TextureStore.h"
#include "prisma3d/scene/CollisionShape3D.h"
#include "prisma3d/scene/Forest3D.h"
#include "prisma3d/scene/Grass3D.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/ManualMesh3D.h"
#include "prisma3d/scene/MultiMeshInstance3D.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/SceneSerializer.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Terrain3D.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/VisualInstance3D.h"
#include "prisma3d/terrain/TerrainLayerBlendMap.h"

#include <ct/json.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <math.h>
#include <string.h>

using namespace prisma3d;

namespace
{

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

bool sameVec3(const Math::Vec3& a, const Math::Vec3& b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

bool sameMatrix(const Math::Mat4& a, const Math::Mat4& b)
{
    return memcmp(a.Data(), b.Data(), 16 * sizeof(float)) == 0;
}

bool sameSurface(const SurfaceData& a, const SurfaceData& b)
{
    if (a.topology != b.topology || a.positions.size() != b.positions.size() ||
            a.normals.size() != b.normals.size() || a.colors.size() != b.colors.size() ||
            a.uvs.size() != b.uvs.size() || a.indices.size() != b.indices.size())
        return false;
    for (size_t i = 0; i < a.positions.size(); ++i)
        if (!sameVec3(a.positions[i], b.positions[i])) return false;
    for (size_t i = 0; i < a.normals.size(); ++i)
        if (!sameVec3(a.normals[i], b.normals[i])) return false;
    for (size_t i = 0; i < a.indices.size(); ++i)
        if (a.indices[i] != b.indices[i]) return false;
    for (size_t i = 0; i < a.colors.size(); ++i)
        if (memcmp(&a.colors[i], &b.colors[i], sizeof(Math::Vec4)) != 0) return false;
    return true;
}

MeshId namedMesh(const char* name)
{
    MeshStore& meshes = *Main::meshes();
    const MeshId mesh = meshes.create(name);
    meshes.setPath(mesh, name);
    return mesh;
}

MaterialId namedMaterial(const char* name)
{
    MaterialStore& materials = *Main::materials();
    const MaterialId material = materials.create(name);
    materials.setPath(material, "library.mat");
    return material;
}

TextureId namedTexture(std::uint32_t width, std::uint32_t height, bool srgb, const char* name)
{
    TextureStore& textures = *Main::textures();
    const TextureId texture = textures.createEmpty(width, height, srgb, false, name);
    textures.setPath(texture, name);
    return texture;
}

Node* copyOf(Node* source, Node* parent, SceneResult& result, ct::String* textOut = nullptr)
{
    ct::String text;
    SceneResult saved;
    CHECK(SceneSerializer::save(source, text, &saved));
    for (size_t i = 0; i < saved.diagnostics.size(); ++i)
        result.diagnostics.push_back(saved.diagnostics[i]);
    if (textOut) *textOut = text;
    return SceneSerializer::load(text, parent, result);
}

void testSurfacesAndMaterials(Node* root)
{
    MeshStore& meshes = *Main::meshes();
    MaterialStore& materials = *Main::materials();

    const MeshId box = namedMesh("box");
    CHECK(meshes.addSurface(box, colouredBox()));
    const MaterialId wood = namedMaterial("wood");
    const MaterialId metal = namedMaterial("metal");

    MeshInstance3D* instance = root->createChild<MeshInstance3D>();
    instance->setName("mesh");
    instance->setMesh(box);
    instance->setSurfaceMaterial(0, wood);
    instance->setSurfaceMaterial(2, metal);
    instance->setSurfaceVisible(1, false);
    instance->setSurfaceVisible(3, false);

    SceneResult result;
    MeshInstance3D* copy = static_cast<MeshInstance3D*>(copyOf(instance, root, result));
    CHECK(copy != nullptr);
    expectClean(result, "mesh instance");
    if (copy)
    {
        CHECK(copy->getSurfaceMaterial(0) == wood);
        CHECK(!copy->getSurfaceMaterial(1).valid());
        CHECK(copy->getSurfaceMaterial(2) == metal);
        CHECK(!copy->isSurfaceVisible(1));
        CHECK(copy->isSurfaceVisible(0));
        CHECK(copy->isSurfaceVisible(2));
        CHECK(!copy->isSurfaceVisible(3));
    }

    const MaterialId anonymous = materials.create();
    instance->setSurfaceMaterial(4, anonymous);
    ct::String text;
    SceneResult warned;
    CHECK(SceneSerializer::save(instance, text, &warned));
    CHECK(hasDiagnostic(warned, SceneDiagnosticSeverity::Warning, "root.data.surface_materials[4]",
            "no path"));
    CHECK(warned.success());

    materials.destroy(metal);
    SceneResult missing;
    Node* partial = SceneSerializer::load(text, root, missing);
    CHECK(partial != nullptr);
    CHECK(missing.success());
    CHECK(hasDiagnostic(missing, SceneDiagnosticSeverity::Warning,
            "root.data.surface_materials[1].material", "could not be loaded"));
    if (partial)
    {
        MeshInstance3D* loaded = static_cast<MeshInstance3D*>(partial);
        CHECK(loaded->getSurfaceMaterial(0) == wood);
        CHECK(!loaded->isSurfaceVisible(1));
    }

    if (copy) copy->queueFree();
    if (partial) partial->queueFree();
    instance->queueFree();
}

void testMultiMesh(Node* root)
{
    MeshStore& meshes = *Main::meshes();
    const MeshId box = namedMesh("multi box");
    CHECK(meshes.addSurface(box, colouredBox()));

    MultiMeshInstance3D* instance = root->createChild<MultiMeshInstance3D>();
    instance->setMesh(box);
    CHECK(instance->setUseColors(true));
    CHECK(instance->setUseCustomData(true));
    instance->setInstanceCount(5);
    for (int i = 0; i < 5; ++i)
    {
        const float f = static_cast<float>(i);
        instance->setInstanceTransform(i, Math::Mat4::Translation(Math::Vec3(f, f * 0.5f, -f)) *
                                                  Math::Mat4::RotationY(f * 0.3f));
        instance->setInstanceColor(i, Math::Vec4(f * 0.1f, 0.5f, 1.0f - f * 0.1f, 1.0f));
        instance->setInstanceCustomData(i, Math::Vec4(f, f * 2.0f, 3.0f, 4.0f));
    }
    CHECK(instance->setVisibleInstanceCount(3));

    SceneResult result;
    MultiMeshInstance3D* copy = static_cast<MultiMeshInstance3D*>(copyOf(instance, root, result));
    CHECK(copy != nullptr);
    expectClean(result, "multimesh");
    if (copy)
    {
        CHECK(copy->getInstanceCount() == 5);
        CHECK(copy->getVisibleInstanceCount() == 3);
        CHECK(copy->getUseColors() && copy->getUseCustomData());
        for (int i = 0; i < 5; ++i)
        {
            CHECK(sameMatrix(copy->getInstanceTransform(i), instance->getInstanceTransform(i)));
            const Math::Vec4 color = copy->getInstanceColor(i);
            const Math::Vec4 expected = instance->getInstanceColor(i);
            CHECK(color.x == expected.x && color.y == expected.y && color.z == expected.z &&
                    color.w == expected.w);
            const Math::Vec4 custom = copy->getInstanceCustomData(i);
            const Math::Vec4 expectedCustom = instance->getInstanceCustomData(i);
            CHECK(custom.x == expectedCustom.x && custom.y == expectedCustom.y &&
                    custom.z == expectedCustom.z && custom.w == expectedCustom.w);
        }
    }

    ct::String text;
    SceneSerializer::save(instance, text);
    ct::Json::Error error;
    ct::Json document = ct::Json::parse(text, &error);
    CHECK(!error);
    document["root"]["data"]["buffer"] = ct::Json("AAAAAA==");
    SceneResult badBuffer;
    Node* broken = SceneSerializer::load(document.dump(), root, badBuffer);
    CHECK(broken != nullptr);
    CHECK(hasDiagnostic(badBuffer, SceneDiagnosticSeverity::Error, "root.data.buffer",
            "does not fit"));

    if (broken) broken->queueFree();
    if (copy) copy->queueFree();
    instance->queueFree();
}

void testManualMesh(Node* root)
{
    const MaterialId paint = namedMaterial("paint");

    ManualMesh3D* mesh = root->createChild<ManualMesh3D>();
    mesh->setName("manual");
    const SurfaceData first = colouredBox();
    SurfaceData second;
    second.positions.push_back(Math::Vec3(0.0f, 0.0f, 0.0f));
    second.positions.push_back(Math::Vec3(1.0f, 0.0f, 0.0f));
    second.positions.push_back(Math::Vec3(0.0f, 1.0f, 0.0f));
    second.normals.push_back(Math::Vec3(0.0f, 0.0f, 1.0f));
    second.normals.push_back(Math::Vec3(0.0f, 0.0f, 1.0f));
    second.normals.push_back(Math::Vec3(0.0f, 0.0f, 1.0f));
    second.uvs.push_back(Math::Vec2(0.0f, 0.0f));
    second.uvs.push_back(Math::Vec2(1.0f, 0.0f));
    second.uvs.push_back(Math::Vec2(0.0f, 1.0f));
    second.indices.push_back(0);
    second.indices.push_back(1);
    second.indices.push_back(2);
    CHECK(mesh->addSurface(first, paint) == 0);
    CHECK(mesh->addSurface(second) == 1);

    SceneResult result;
    ManualMesh3D* copy = static_cast<ManualMesh3D*>(copyOf(mesh, root, result));
    CHECK(copy != nullptr);
    expectClean(result, "manual mesh");
    if (copy)
    {
        CHECK(copy->getSurfaceCount() == 2);
        if (copy->getSurfaceCount() == 2)
        {
            CHECK(sameSurface(*copy->getSurfaceData(0), first));
            CHECK(sameSurface(*copy->getSurfaceData(1), second));
            CHECK(copy->getSurfaceMaterial(0) == paint);
            CHECK(!copy->getSurfaceMaterial(1).valid());
        }
    }

    ManualMesh3D* dynamic = root->createChild<ManualMesh3D>();
    dynamic->setDynamic(64, 96);
    CHECK(dynamic->addSurface(second) == 0);
    ManualMesh3D* dynamicCopy = static_cast<ManualMesh3D*>(copyOf(dynamic, root, result));
    CHECK(dynamicCopy != nullptr);
    if (dynamicCopy)
    {
        CHECK(dynamicCopy->isDynamic());
        CHECK(dynamicCopy->getVertexCapacity() == 64 && dynamicCopy->getIndexCapacity() == 96);
        CHECK(dynamicCopy->getSurfaceCount() == 1);
        if (dynamicCopy->getSurfaceCount() == 1)
            CHECK(sameSurface(*dynamicCopy->getSurfaceData(0), second));
    }

    if (dynamicCopy) dynamicCopy->queueFree();
    dynamic->queueFree();
    if (copy) copy->queueFree();
    mesh->queueFree();
}

void testLensFlare(Node* root)
{
    TextureStore& textures = *Main::textures();
    const TextureId a = namedTexture(4, 4, true, "flare/a.png");
    const TextureId b = namedTexture(4, 4, true, "flare/b.png");
    const TextureId unnamed = textures.createEmpty(4, 4, true, false);
    CHECK(a.valid() && b.valid() && unnamed.valid());

    OmniLight3D* light = root->createChild<OmniLight3D>();
    light->setName("sun");
    ct::Vector<TextureId> flare;
    flare.push_back(a);
    flare.push_back(b);
    light->setLensFlare(flare);

    SceneResult result;
    OmniLight3D* copy = static_cast<OmniLight3D*>(copyOf(light, root, result));
    CHECK(copy != nullptr);
    expectClean(result, "lens flare");
    if (copy)
    {
        CHECK(copy->getLensFlareCount() == 2);
        CHECK(copy->getLensFlareTexture(0) == a);
        CHECK(copy->getLensFlareTexture(1) == b);
    }

    flare.push_back(unnamed);
    light->setLensFlare(flare);
    SceneResult warned;
    ct::String text;
    CHECK(SceneSerializer::save(light, text, &warned));
    CHECK(hasDiagnostic(warned, SceneDiagnosticSeverity::Warning, "root.data.lens_flare[2]",
            "no path"));

    textures.destroy(b);
    SceneResult missing;
    Node* partial = SceneSerializer::load(text, root, missing);
    CHECK(partial != nullptr);
    CHECK(hasDiagnostic(missing, SceneDiagnosticSeverity::Warning, "root.data.lens_flare[1]",
            "could not be loaded"));
    if (partial)
    {
        CHECK(static_cast<OmniLight3D*>(partial)->getLensFlareCount() == 1);
        partial->queueFree();
    }
    if (copy) copy->queueFree();
    light->queueFree();
}

void testWorld(Node* root)
{
    MeshStore& meshes = *Main::meshes();

    const char* names[] = { "terrain/grass_d.png", "terrain/grass_n.png", "terrain/rock_d.png",
        "terrain/rock_n.png", "grass/blades.png" };
    TextureId layerTextures[5];
    for (int i = 0; i < 5; ++i)
    {
        layerTextures[i] = namedTexture(16, 16, i % 2 == 0, names[i]);
        CHECK(layerTextures[i].valid());
    }

    const MeshId treeMesh = namedMesh("trees/pine.mesh");
    CHECK(meshes.addSurface(treeMesh, colouredBox()));
    const MeshId otherMesh = namedMesh("trees/birch.mesh");
    CHECK(meshes.addSurface(otherMesh, colouredBox()));
    const MeshId shadowMesh = namedMesh("trees/pine_shadow.mesh");
    CHECK(meshes.addSurface(shadowMesh, colouredBox()));
    const MaterialId bark = namedMaterial("bark");
    const MaterialId leaves = namedMaterial("leaves");

    Node3D* world = root->createChild<Node3D>();
    world->setName("world");
    world->setPosition(Math::Vec3(10.0f, 2.0f, -5.0f));

    Terrain3D* terrain = world->createChild<Terrain3D>();
    terrain->setName("ground");
    terrain->setLightmapEnabled(false);
    terrain->setMaxPixelError(5.5f);
    terrain->setCastShadows(false);

    const std::uint16_t size = 129;
    ct::Vector<float> hills;
    hills.resize(static_cast<size_t>(size) * size);
    for (int y = 0; y < size; ++y)
        for (int x = 0; x < size; ++x)
            hills[static_cast<size_t>(y) * size + x] = 6.0f * sinf(static_cast<float>(x) * 0.21f) *
                                                       cosf(static_cast<float>(y) * 0.17f);
    TerrainImportData data;
    data.terrainSize = size;
    data.minBatchSize = 17;
    data.maxBatchSize = 33;
    data.worldSize = 128.0f;
    data.inputFloat = hills.data();
    data.skirtSize = 12.0f;
    data.useRayBoxDistanceCalculation = true;
    data.layerBlendMapSize = 64;
    data.lightMapSize = 64;
    data.compositeMapSize = 64;
    data.defaultGlobalColourMapSize = 64;
    data.lightMapDirection = Math::Vec3(0.3f, -0.9f, 0.1f);
    data.compositeMapAmbient = Math::Vec3(0.2f, 0.3f, 0.4f);
    data.compositeMapDiffuse = Math::Vec3(0.9f, 0.8f, 0.7f);
    TerrainLayerInstance grass;
    grass.worldSize = 12.0f;
    grass.textureNames.push_back(names[0]);
    grass.textureNames.push_back(names[1]);
    TerrainLayerInstance rock;
    rock.worldSize = 7.5f;
    rock.textureNames.push_back(names[2]);
    rock.textureNames.push_back(names[3]);
    data.layerList.push_back(grass);
    data.layerList.push_back(rock);
    CHECK(terrain->load(data));
    CHECK(terrain->isLoaded());
    TerrainLayerBlendMap* rockMap = terrain->getLayerBlendMap(1);
    CHECK(rockMap != nullptr);
    if (rockMap)
    {
        for (int y = 0; y < 64; ++y)
            for (int x = 0; x < 64; ++x)
                rockMap->setBlendValue(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y),
                        static_cast<float>((x * 3 + y) % 11) / 10.0f);
        rockMap->update();
    }
    terrain->setGlobalColourMapEnabled(true, 64);
    Pixmap colours(64, 64, 3);
    for (int y = 0; y < 64; ++y)
        for (int x = 0; x < 64; ++x)
            colours.set_pixel(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y),
                    static_cast<std::uint8_t>(x * 4), static_cast<std::uint8_t>(y * 4),
                    static_cast<std::uint8_t>((x + y) * 2), 255);
    CHECK(terrain->setGlobalColourMap(colours));

    Forest3D* forest = world->createChild<Forest3D>();
    forest->setName("trees");
    forest->setTerrain(terrain);
    forest->setTerrainOffset(0.25f);
    forest->setMaxSlope(40.0f);
    ForestSpeciesDesc pine;
    pine.mesh = treeMesh;
    pine.shadowMesh = shadowMesh;
    pine.materials.push_back(bark);
    pine.materials.push_back(leaves);
    ForestSpeciesDesc birch;
    birch.mesh = otherMesh;
    CHECK(forest->addSpecies(pine) == 0);
    CHECK(forest->addSpecies(birch) == 1);
    for (int i = 0; i < 6; ++i)
    {
        const float f = static_cast<float>(i);
        CHECK(forest->plant(i % 2, Math::Vec3(f * 3.0f - 8.0f, 0.5f * f, f * 2.0f), f * 0.4f,
                      1.0f + f * 0.1f) >= 0);
    }
    CHECK(forest->plantOnTerrain(0, 4.0f, 6.0f, 0.3f, 1.5f) >= 0);
    TerrainPath exclusion;
    exclusion.add(Math::Vec3(0.0f, 0.0f, 0.0f), 2.0f);
    exclusion.add(Math::Vec3(30.0f, 0.0f, 10.0f), 3.0f);
    exclusion.add(Math::Vec3(50.0f, 0.0f, -10.0f), 2.5f);
    exclusion.closed = true;
    forest->addExclusion(exclusion, 1.5f);

    Grass3D* grassNode = world->createChild<Grass3D>();
    grassNode->setName("grass");
    grassNode->setTerrain(terrain);
    grassNode->setExtent(-32.0f, -32.0f, 32.0f, 32.0f);
    GrassTypeDesc blades;
    blades.texture = layerTextures[4];
    blades.height = 0.9f;
    blades.width = 0.4f;
    blades.scaleMin = 0.5f;
    blades.scaleMax = 1.5f;
    blades.colourA = Math::Vec3(0.1f, 0.5f, 0.2f);
    blades.colourB = Math::Vec3(0.9f, 0.8f, 0.3f);
    blades.alphaCut = 0.2f;
    blades.densityScale = 0.75f;
    CHECK(grassNode->addType(blades) == 0);
    GrassTypeDesc flowers = blades;
    flowers.height = 0.3f;
    flowers.densityScale = 0.1f;
    CHECK(grassNode->addType(flowers) == 1);
    CHECK(grassNode->addTypeRegion(0, 0, 0, 8, 16, 1.0f, 2.0f));
    CHECK(grassNode->addTypeRegion(0, 8, 0, 8, 16, 0.5f, 1.0f));
    CHECK(grassNode->createMask(8, 8));
    grassNode->fill(0, 0.5f);
    grassNode->fill(1, 0.25f);
    grassNode->addExclusion(exclusion, 0.5f);

    CollisionShape3D* field = world->createChild<CollisionShape3D>();
    field->setName("field");
    CHECK(field->setHeightMapFromTerrain(*terrain));
    CHECK(field->getKind() == CollisionShapeKind::HeightMap);

    SceneResult result;
    ct::String text;
    Node3D* copies = root->createChild<Node3D>();
    copies->setName("copies");
    Node3D* copy = static_cast<Node3D*>(copyOf(world, copies, result, &text));
    CHECK(copy != nullptr);
    expectClean(result, "world");
    if (!copy) return;
    CHECK(copy != world);
    CHECK(copy->getPosition().x == 10.0f && copy->getPosition().y == 2.0f);

    Terrain3D* loadedTerrain = static_cast<Terrain3D*>(copy->findChild("ground"));
    Forest3D* loadedForest = static_cast<Forest3D*>(copy->findChild("trees"));
    Grass3D* loadedGrass = static_cast<Grass3D*>(copy->findChild("grass"));
    CHECK(loadedTerrain && loadedForest && loadedGrass);
    if (!loadedTerrain || !loadedForest || !loadedGrass) return;

    CHECK(loadedTerrain->isLoaded());
    const Terrain& a = terrain->getTerrain();
    const Terrain& b = loadedTerrain->getTerrain();
    CHECK(a.getSize() == b.getSize() && a.getWorldSize() == b.getWorldSize());
    CHECK(a.getMaxBatchSize() == b.getMaxBatchSize() && a.getMinBatchSize() == b.getMinBatchSize());
    CHECK(a.getSkirtSize() == b.getSkirtSize() &&
            a.getUseRayBoxDistanceCalculation() == b.getUseRayBoxDistanceCalculation());
    CHECK(a.getMaxPixelError() == b.getMaxPixelError());
    CHECK(a.getLayerBlendMapSize() == b.getLayerBlendMapSize());
    CHECK(sameVec3(a.getLightMapDirection(), b.getLightMapDirection()));
    CHECK(sameVec3(a.getCompositeMapAmbient(), b.getCompositeMapAmbient()));
    CHECK(sameVec3(a.getCompositeMapDiffuse(), b.getCompositeMapDiffuse()));
    CHECK(memcmp(a.getHeightData(), b.getHeightData(),
                  static_cast<size_t>(size) * size * sizeof(float)) == 0);
    CHECK(a.getLayerCount() == 2 && b.getLayerCount() == 2);
    for (std::uint8_t i = 0; i < a.getLayerCount() && i < b.getLayerCount(); ++i)
    {
        CHECK(a.getLayerWorldSize(i) == b.getLayerWorldSize(i));
        CHECK(a.getLayerTextureName(i, 0) == b.getLayerTextureName(i, 0));
        CHECK(a.getLayerTextureName(i, 1) == b.getLayerTextureName(i, 1));
    }
    CHECK(a.getBlendTextureCount() == b.getBlendTextureCount());
    for (std::uint8_t i = 0; i < a.getBlendTextureCount() && i < b.getBlendTextureCount(); ++i)
        CHECK(a.getBlendTextureData(i) == b.getBlendTextureData(i));
    CHECK(b.getGlobalColourMapEnabled() && b.getGlobalColourMapSize() == 64);
    CHECK(a.getGlobalColourMapData() == b.getGlobalColourMapData());
    CHECK(!loadedTerrain->getCastShadows());
    CHECK(!loadedTerrain->getLightmapEnabled());

    CHECK(loadedForest->getTerrain() == loadedTerrain);
    CHECK(loadedGrass->getTerrain() == loadedTerrain);
    CHECK(loadedForest->getTerrainOffset() == 0.25f && loadedForest->getMaxSlope() == 40.0f);
    CHECK(loadedForest->getSpeciesCount() == 2);
    CHECK(loadedForest->getMeshNode(0) && loadedForest->getMeshNode(0)->getMesh() == treeMesh);
    CHECK(loadedForest->getMeshNode(1) && loadedForest->getMeshNode(1)->getMesh() == otherMesh);
    CHECK(loadedForest->getMeshNode(0)->getSurfaceMaterial(0) == bark);
    CHECK(loadedForest->getMeshNode(0)->getSurfaceMaterial(1) == leaves);
    CHECK(loadedForest->getInstanceCount() == forest->getInstanceCount());
    for (int i = 0; i < forest->getInstanceCount() && i < loadedForest->getInstanceCount(); ++i)
    {
        CHECK(loadedForest->getInstanceSpecies(i) == forest->getInstanceSpecies(i));
        CHECK(sameMatrix(loadedForest->getInstanceTransform(i), forest->getInstanceTransform(i)));
        CHECK(loadedForest->isInstanceOnTerrain(i) == forest->isInstanceOnTerrain(i));
    }
    CHECK(loadedForest->isInstanceOnTerrain(6));
    CHECK(!loadedForest->isInstanceOnTerrain(0));

    CollisionShape3D* loadedField = static_cast<CollisionShape3D*>(copy->findChild("field"));
    CHECK(loadedField && loadedField->getKind() == CollisionShapeKind::HeightMap);
    if (loadedField)
    {
        PhysicsServer3D* server = PhysicsServer3D::singleton();
        ct::Vector<float> expected;
        ct::Vector<float> actual;
        CHECK(server->shapeGetHeightMap(field->getShape(), expected));
        CHECK(server->shapeGetHeightMap(loadedField->getShape(), actual));
        CHECK(actual.size() == expected.size() &&
                actual.size() == static_cast<size_t>(size) * size);
        CHECK(actual == expected);
        CHECK(server->shapeGetHeightMapWidth(loadedField->getShape()) ==
                server->shapeGetHeightMapWidth(field->getShape()));
        CHECK(sameVec3(server->shapeGetHeightMapScale(loadedField->getShape()),
                server->shapeGetHeightMapScale(field->getShape())));
        CHECK(sameVec3(server->shapeGetHeightMapOffset(loadedField->getShape()),
                server->shapeGetHeightMapOffset(field->getShape())));
    }

    CHECK(loadedGrass->getTypeCount() == 2);
    for (int t = 0; t < 2; ++t)
    {
        GrassTypeDesc expected;
        GrassTypeDesc actual;
        CHECK(grassNode->getType(t, expected) && loadedGrass->getType(t, actual));
        CHECK(actual.texture == expected.texture);
        CHECK(actual.height == expected.height && actual.width == expected.width);
        CHECK(actual.scaleMin == expected.scaleMin && actual.scaleMax == expected.scaleMax);
        CHECK(sameVec3(actual.colourA, expected.colourA) &&
                sameVec3(actual.colourB, expected.colourB));
        CHECK(actual.alphaCut == expected.alphaCut && actual.densityScale == expected.densityScale);
        CHECK(loadedGrass->getTypeRegionCount(t) == grassNode->getTypeRegionCount(t));
        for (int r = 0; r < grassNode->getTypeRegionCount(t); ++r)
        {
            GrassRegion left;
            GrassRegion right;
            CHECK(grassNode->getTypeRegion(t, r, left) && loadedGrass->getTypeRegion(t, r, right));
            CHECK(memcmp(&left.texMulAdd, &right.texMulAdd, sizeof(Math::Vec4)) == 0);
            CHECK(left.size == right.size && left.aspect == right.aspect &&
                    left.weight == right.weight);
        }
    }
    CHECK(loadedGrass->hasMask() && loadedGrass->getMaskWidth() == 8 &&
            loadedGrass->getMaskDepth() == 8);
    CHECK(loadedGrass->getMaskDensity(0, 1.0f, 1.0f) == grassNode->getMaskDensity(0, 1.0f, 1.0f));
    CHECK(loadedGrass->getMaskDensity(1, -9.0f, 5.0f) == grassNode->getMaskDensity(1, -9.0f, 5.0f));

    ct::String again;
    CHECK(SceneSerializer::save(copy, again));
    if (again != text)
    {
        printf("FAIL: saving the loaded world gives another text\n");
        ++failures;
    }

    const size_t meshCount = meshes.count();
    copy->queueFree();
    world->queueFree();
    (void) meshCount;
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "scene serializer test";
    desc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    desc.width = 64;
    desc.height = 48;
    desc.headless = true;

    CHECK(Main::setup(desc));
    if (!Main::device()) return 1;

    PhysicsServer3D physics;
    SceneTree tree;
    CHECK(Main::start(&tree));
    Node* root = tree.getRoot();
    CHECK(root != nullptr);
    if (!root) return 1;

    testSurfacesAndMaterials(root);
    testMultiMesh(root);
    testManualMesh(root);
    testLensFlare(root);
    testWorld(root);

    tree.process(1.0 / 60.0);

    if (failures)
    {
        printf("%d failures\n", failures);
        return 1;
    }
    printf("scene serializer gpu ok\n");
    return 0;
}
