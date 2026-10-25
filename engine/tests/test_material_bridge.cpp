#include "Check.h"
#include "MaterialFields.h"

#include "prisma/rhi/Driver.h"
#include "prisma3d/core/FileSystem.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/import/ObjImporter.h"
#include "prisma3d/render/AnimatedTexture.h"
#include "prisma3d/render/AssetLoader.h"
#include "prisma3d/render/LoadedMaterialLibrary.h"

#include "platform.h"

#include <string.h>

using namespace prisma3d;

namespace
{

const char* const kRoot = PRISMA3D_TEST_DIR "/material_bridge";

const char* const kCube = "mtllib cube.mtl\n"
                          "v 0 0 0\n"
                          "v 1 0 0\n"
                          "v 1 1 0\n"
                          "v 0 1 0\n"
                          "vt 0 0\n"
                          "vt 1 0\n"
                          "vt 1 1\n"
                          "vt 0 1\n"
                          "vn 0 0 1\n"
                          "usemtl red\n"
                          "f 1/1/1 2/2/1 3/3/1 4/4/1\n"
                          "usemtl blue\n"
                          "f -2/-2 -3/-3 -4/-4\n";

const char* const kCubeMaterials = "newmtl red\n"
                                   "Kd 1.0 0.0 0.0\n"
                                   "map_Kd skin.png\n"
                                   "\n"
                                   "newmtl blue\n"
                                   "Kd 0 0 1\n";

struct Context
{
    MaterialStore* materials = nullptr;
    MeshStore* meshes = nullptr;
    AssetLoader* loader = nullptr;
};

ct::String under(const char* directory, const char* name)
{
    return ct::String(directory) + "/" + name;
}

bool writeText(const ct::String& path, const char* text)
{
    return file_write(path.c_str(), text, strlen(text));
}

bool savePng(const ct::String& path, std::uint8_t red)
{
    Pixmap pixmap(2, 2, 4);
    pixmap.fill(red, 20, 30, 255);
    return pixmap.save(path.c_str());
}

bool drain(AssetLoader& loader)
{
    const std::uint64_t start = time_nanos();
    while (loader.pendingRequests() != 0)
    {
        if (time_nanos() - start > 20000000000ull) return false;
        loader.pump();
        time_sleep(1);
    }
    loader.pump();
    return true;
}

ModelId loadModel(Context& context, const ct::String& path, bool sidecar)
{
    ModelOptions options;
    options.materialSidecar = sidecar;
    const ModelId model = context.loader->requestModel(path, options);
    CHECK(drain(*context.loader));
    CHECK(context.loader->modelStatus(model) == LoadStatus::Loaded);
    return model;
}

MaterialId materialNamed(Context& context, ModelId model, const char* name)
{
    const ModelInfo* info = context.loader->model(model);
    if (!info) return MaterialId();
    for (std::size_t i = 0; i < info->materials.size(); ++i)
    {
        const MaterialInfo* material = context.materials->get(info->materials[i]);
        if (material && material->name == name) return info->materials[i];
    }
    return MaterialId();
}

void makeCube(const char* directory)
{
    CHECK(fs_create_directory(directory));
    fs_remove_path(under(directory, "cube.mat").c_str());
    CHECK(writeText(under(directory, "cube.obj"), kCube));
    CHECK(writeText(under(directory, "cube.mtl"), kCubeMaterials));
    CHECK(savePng(under(directory, "skin.png"), 200));
}

bool sameFloat(float a, float b)
{
    return a == b;
}

void testPathHelpers()
{
    CHECK(materialSidecarPath("a/b/model.gltf") == "a/b/model.mat");
    CHECK(materialSidecarPath("model.obj") == "model.mat");
    CHECK(materialSidecarPath("a.b/model") == "a.b/model.mat");
    CHECK(materialSidecarPath("x\\y.z\\w.glb") == "x\\y.z\\w.mat");
    CHECK(materialSidecarPath("model.tar.gz") == "model.tar.mat");

    CHECK(resolveMaterialTexturePath("a/b", "c.png") == "a/b/c.png");
    CHECK(resolveMaterialTexturePath("a/b", "../tex/c.png") == "a/tex/c.png");
    CHECK(resolveMaterialTexturePath("a/b", "./x/../c.png") == "a/b/c.png");
    CHECK(resolveMaterialTexturePath("", "c.png") == "c.png");
    CHECK(resolveMaterialTexturePath("a/b", "/abs/c.png") == "/abs/c.png");
    CHECK(resolveMaterialTexturePath("a/b", "") == "");
    CHECK(resolveMaterialTexturePath("a\\b", "c\\d.png") == "a/b/c/d.png");

    CHECK(relativeMaterialTexturePath("a/b/lib.mat", "a/b/c.png") == "c.png");
    CHECK(relativeMaterialTexturePath("a/b/lib.mat", "a/b/t/c.png") == "t/c.png");
    CHECK(relativeMaterialTexturePath("a/b/lib.mat", "a/tex/c.png") == "../tex/c.png");
    CHECK(relativeMaterialTexturePath("a/b/lib.mat", "x/c.png") == "../../x/c.png");
    CHECK(relativeMaterialTexturePath("lib.mat", "tex/c.png") == "tex/c.png");
    CHECK(relativeMaterialTexturePath("/r/m/lib.mat", "/r/t/c.png") == "../t/c.png");
    CHECK(relativeMaterialTexturePath("/r/lib.mat", "/q/c.png") == "/q/c.png");
    CHECK(relativeMaterialTexturePath("a/lib.mat", "/q/c.png") == "/q/c.png");
    CHECK(relativeMaterialTexturePath("a/b/lib.mat", "") == "");
}

void setEverything(Context& context, MaterialId id, const ct::String* paths)
{
    MaterialStore& store = *context.materials;
    const MaterialInfo m = everythingMaterial();
    store.setColor(id, MaterialParam::Albedo, m.albedo);
    store.setFloat(id, MaterialParam::Metallic, m.metallic);
    store.setFloat(id, MaterialParam::Specular, m.specular);
    store.setFloat(id, MaterialParam::Roughness, m.roughness);
    store.setColor(id, MaterialParam::Emission,
            Math::Vec4(m.emission.x, m.emission.y, m.emission.z, 1.0f));
    store.setFloat(id, MaterialParam::EmissionEnergy, m.emissionEnergy);
    store.setFloat(id, MaterialParam::NormalScale, m.normalScale);
    store.setFloat(id, MaterialParam::AoLightAffect, m.aoLightAffect);
    store.setFloat(id, MaterialParam::Clearcoat, m.clearcoat);
    store.setFloat(id, MaterialParam::ClearcoatRoughness, m.clearcoatRoughness);
    store.setFloat(id, MaterialParam::AlphaScissorThreshold, m.alphaScissorThreshold);
    store.setVec3(id, MaterialParam::Uv1Scale, m.uv1Scale);
    store.setVec3(id, MaterialParam::Uv1Offset, m.uv1Offset);
    store.setVec3(id, MaterialParam::Uv2Scale, m.uv2Scale);
    store.setVec3(id, MaterialParam::Uv2Offset, m.uv2Offset);
    store.setDetail(id, m.detail);
    store.setDetailBlendMode(id, m.detailBlend);
    store.setDetailUv(id, m.detailUv);
    store.setTextureChannel(id, MaterialTexture::Metallic, m.metallicChannel);
    store.setTextureChannel(id, MaterialTexture::Roughness, m.roughnessChannel);
    store.setTextureChannel(id, MaterialTexture::AmbientOcclusion, m.aoChannel);
    store.setTransparency(id, m.transparency);
    store.setAlphaAntialiasing(id, m.alphaAntialiasing, m.alphaAntialiasingEdge);
    store.setCull(id, m.cull);
    store.setEmissionOperator(id, m.emissionOperator);
    store.setShading(id, m.shading);
    store.setBlendMode(id, m.blend);
    store.setDepthTest(id, m.depthTest);
    store.setGrass(id, m.grass, m.grassDrawDistance, m.grassCameraBend, m.grassTintA, m.grassTintB);
    store.setGrassWind(id, m.grassWindDirection, m.grassWindAmplitude, m.grassWindSpeed);
    store.setLeafWind(id, m.leafWind, m.leafWindHeight, m.leafWindStrength, m.leafWindSpeed,
            m.leafWindDirection);
    store.setBillboard(id, m.billboard, m.billboardKeepScale);
    store.setReceiveShadows(id, m.receiveShadows);
    store.setBacklight(id, m.backlight);
    store.setTerrainMorph(id, m.terrainMorph);
    store.setFloat(id, MaterialParam::Refraction, m.refraction);
    store.setTextureChannel(id, MaterialTexture::Refraction, m.refractionChannel);
    store.setWater(id, m.water, m.waterData);
    for (std::size_t slot = 0; slot < kMaterialTextureCount; ++slot)
    {
        const MaterialTexture texture = static_cast<MaterialTexture>(slot);
        const bool srgb = texture == MaterialTexture::Albedo ||
                          texture == MaterialTexture::Emission ||
                          texture == MaterialTexture::DetailAlbedo;
        store.setTexture(id, texture, context.loader->requestTexture(paths[slot], srgb, true));
    }
}

void testEveryFieldRoundTrip(Context& context)
{
    const ct::String textures = under(kRoot, "round/tex");
    const ct::String library = under(kRoot, "round/lib");
    CHECK(fs_create_directory(textures.c_str()));
    CHECK(fs_create_directory(library.c_str()));
    const char* const names[kMaterialTextureCount] = { "albedo.png", "metal.png", "rough.png",
        "emission.png", "normal.png", "ao.png", "coat.png", "orm.png", "detail_albedo.png",
        "detail_normal.png", "detail_mask.png", "refraction.png" };
    ct::String paths[kMaterialTextureCount];
    for (std::size_t slot = 0; slot < kMaterialTextureCount; ++slot)
    {
        paths[slot] = under(textures.c_str(), names[slot]);
        CHECK(savePng(paths[slot], static_cast<std::uint8_t>(10 + slot * 20)));
    }

    MaterialStore& store = *context.materials;
    const MaterialId source = store.create("everything");
    setEverything(context, source, paths);
    CHECK(drain(*context.loader));

    const MaterialInfo* original = store.get(source);
    CHECK(original != nullptr);
    if (!original) return;
    CHECK(materialFieldDifferences(*original, MaterialInfo()) == kMaterialFieldCount);
    CHECK(materialFieldDifferences(*original, everythingMaterial()) == 0);
    for (std::size_t slot = 0; slot < kMaterialTextureCount; ++slot)
        CHECK(original->textures[slot].valid());

    MaterialDef captured;
    CHECK(captureMaterial(store, source, captured));
    CHECK(materialFieldDifferences(captured.info, *original) == 0);
    CHECK(captured.info.name == "everything");
    for (std::size_t slot = 0; slot < kMaterialTextureCount; ++slot)
        CHECK(captured.paths[slot] == paths[slot]);

    const ct::String file = under(library.c_str(), "everything.mat");
    ct::Vector<MaterialId> ids;
    ids.push_back(source);
    ct::String error;
    CHECK(saveMaterials(store, ids, file, error));
    CHECK(error.empty());

    MaterialLibraryData written;
    CHECK(MaterialLibraryFormat::load(file, written, error));
    CHECK(written.materials.size() == 1);
    if (written.materials.size() == 1)
    {
        const char* const relative[kMaterialTextureCount] = { "../tex/albedo.png",
            "../tex/metal.png", "../tex/rough.png", "../tex/emission.png", "../tex/normal.png",
            "../tex/ao.png", "../tex/coat.png", "../tex/orm.png", "../tex/detail_albedo.png",
            "../tex/detail_normal.png", "../tex/detail_mask.png", "../tex/refraction.png" };
        for (std::size_t slot = 0; slot < kMaterialTextureCount; ++slot)
            CHECK(written.materials[0].paths[slot] == relative[slot]);
        CHECK(materialFieldDifferences(written.materials[0].info, *original) == 0);
    }

    LoadedMaterialLibrary loaded(store, *context.loader);
    CHECK(loaded.load(file, error));
    CHECK(loaded.count() == 1);
    const MaterialId copy = loaded.find("everything");
    CHECK(copy.valid() && copy != source);
    const MaterialInfo* restored = store.get(copy);
    CHECK(restored != nullptr);
    if (!restored) return;
    CHECK(materialFieldDifferences(*restored, *original) == 0);
    CHECK(restored->name == "everything");
    CHECK(restored->path == file);
    for (std::size_t slot = 0; slot < kMaterialTextureCount; ++slot)
        CHECK(restored->textures[slot] == original->textures[slot]);
    CHECK(drain(*context.loader));
    store.destroy(source);
}

void testReloadInPlace(Context& context)
{
    const ct::String directory = under(kRoot, "reload");
    const ct::String textures = under(directory.c_str(), "tex");
    CHECK(fs_create_directory(textures.c_str()));
    CHECK(savePng(under(textures.c_str(), "a.png"), 90));
    CHECK(savePng(under(textures.c_str(), "b.png"), 190));
    const ct::String file = under(directory.c_str(), "set.mat");

    const std::size_t baseline = context.materials->count();
    ct::String error;
    {
        LoadedMaterialLibrary library(*context.materials, *context.loader);
        CHECK(!library.reload(error));
        CHECK(error == "no library has been loaded");
        CHECK(!library.load(under(directory.c_str(), "absent.mat"), error));
        CHECK(error == under(directory.c_str(), "absent.mat") + ": not found or empty");
        CHECK(library.count() == 0 && library.path().empty());

        CHECK(writeText(file,
                "material \"wood\" {\n    roughness = 0.5\n    albedo = \"tex/a.png\"\n}\n"
                "material \"metal\" {\n    metallic = 1\n    roughness = 0.1\n}\n"));
        CHECK(library.load(file, error));
        CHECK(library.count() == 2);
        CHECK(library.path() == file);
        CHECK(context.materials->count() == baseline + 2);
        const MaterialId wood = library.find("wood");
        const MaterialId metal = library.find("metal");
        CHECK(wood.valid() && metal.valid() && wood != metal);
        CHECK(!library.find("glass").valid());
        CHECK(!library.find(nullptr).valid());
        CHECK(context.materials->get(wood)->roughness == 0.5f);
        CHECK(context.materials->get(wood)->textures[0].valid());
        CHECK(!context.materials->get(metal)->textures[0].valid());
        const TextureId woodTexture = context.materials->get(wood)->textures[0];
        const TextureInfo* info =
                context.loader ? TextureStore::singleton()->get(woodTexture) : nullptr;
        CHECK(info != nullptr && info->path == under(textures.c_str(), "a.png"));
        CHECK(info != nullptr && info->srgb);

        CHECK(library.load(file, error));
        CHECK(context.materials->count() == baseline + 2);
        CHECK(library.find("wood") == wood && library.find("metal") == metal);
        CHECK(context.materials->get(wood)->textures[0] == woodTexture);

        CHECK(writeText(file,
                "material \"wood\" {\n    roughness = 0.25\n    albedo = \"tex/b.png\"\n"
                "    normal = \"tex/b.png\"\n}\n"
                "material \"glass\" {\n    transparency = Blend\n}\n"));
        CHECK(library.reload(error));
        CHECK(library.count() == 2);
        CHECK(library.find("wood") == wood);
        CHECK(context.materials->get(wood)->roughness == 0.25f);
        CHECK(context.materials->get(wood)->textures[0] != woodTexture);
        CHECK(context.materials->get(wood)->textures[0].valid());
        CHECK(context.materials->get(wood)
                        ->textures[static_cast<std::size_t>(MaterialTexture::Normal)]
                        .valid());
        const MaterialId glass = library.find("glass");
        CHECK(glass.valid());
        CHECK(context.materials->get(glass)->transparency == MaterialTransparency::AlphaBlend);
        CHECK(library.find("metal") == metal);
        CHECK(context.materials->get(metal)->roughness == 0.1f);
        CHECK(context.materials->count() == baseline + 3);

        CHECK(writeText(file, "material \"wood\" {\n    roughness = 0.9\n    cull = Bogus\n}\n"));
        CHECK(!library.reload(error));
        CHECK(error == file + ": line 3: 'cull': unknown value 'Bogus'");
        CHECK(context.materials->get(wood)->roughness == 0.25f);
        CHECK(library.count() == 2);

        CHECK(writeText(file, "material \"wood\" {\n    roughness = 0.6\n}\n"
                              "material \"wood\" {\n    roughness = 0.7\n}\n"
                              "material \"far\" {\n    albedo = \"tex/missing.png\"\n}\n"));
        CHECK(library.reload(error));
        CHECK(library.count() == 2);
        CHECK(context.materials->get(wood)->roughness == 0.6f);
        CHECK(library.data().warnings.size() == 1);
        CHECK(library.data().warnings[0] ==
                "duplicate material name 'wood' ignored, the first is used");
        const MaterialId far = library.find("far");
        CHECK(far.valid());
        CHECK(!context.materials->get(far)->textures[0].valid());
        CHECK(drain(*context.loader));
    }
    CHECK(context.materials->count() == baseline);
}

void testBinding(Context& context)
{
    const ct::String directory = under(kRoot, "bind");
    makeCube(directory.c_str());
    const ct::String file = under(directory.c_str(), "looks.mat");
    CHECK(writeText(file,
            "material \"red\" {\n    roughness = 0.2\n    color = (0.5, 0.5, 0.5, 1)\n}\n"
            "material \"green\" {\n    metallic = 1\n}\n"));

    const ModelId model = loadModel(context, under(directory.c_str(), "cube.obj"), false);
    const ModelInfo* info = context.loader->model(model);
    CHECK(info != nullptr && info->meshes.size() == 1 && info->materials.size() == 2);
    if (!info || info->meshes.size() != 1) return;

    LoadedMaterialLibrary library(*context.materials, *context.loader);
    ct::String error;
    CHECK(library.load(file, error));
    const MaterialId red = library.find("red");
    CHECK(red.valid());

    const MaterialId ownRed = materialNamed(context, model, "red");
    const MaterialId ownBlue = materialNamed(context, model, "blue");
    CHECK(ownRed.valid() && ownBlue.valid());

    const MeshInfo* mesh = context.meshes->get(info->meshes[0]);
    CHECK(mesh != nullptr && mesh->surfaces.size() == 2);
    if (!mesh || mesh->surfaces.size() != 2) return;
    CHECK(mesh->surfaces[0].material == ownRed && mesh->surfaces[1].material == ownBlue);

    CHECK(library.bind(*context.meshes, *context.loader, model) == 1);
    CHECK(mesh->surfaces[0].material == red);
    CHECK(mesh->surfaces[1].material == ownBlue);
    CHECK(library.bind(*context.meshes, *context.loader, model) == 0);
    CHECK(library.bind(*context.meshes, info->meshes[0]) == 0);
    CHECK(library.bind(*context.meshes, MeshId()) == 0);
    CHECK(library.bind(*context.meshes, *context.loader, ModelId()) == 0);

    CHECK(writeText(file, "material \"red\" {\n    roughness = 0.8\n}\n"));
    CHECK(library.reload(error));
    CHECK(mesh->surfaces[0].material == red);
    CHECK(context.materials->get(mesh->surfaces[0].material)->roughness == 0.8f);

    context.loader->freeModel(model);
}

void testSidecarByName(Context& context)
{
    const ct::String directory = under(kRoot, "side_name");
    makeCube(directory.c_str());
    CHECK(writeText(under(directory.c_str(), "cube.mat"),
            "material \"blue\" {\n    roughness = 0.3\n    lit = false\n}\n"
            "material \"red\" {\n    roughness = 0.7\n    color = (0.25, 0.5, 0.75, 1)\n"
            "    albedo = \"skin.png\"\n}\n"));
    const ModelId model = loadModel(context, under(directory.c_str(), "cube.obj"), true);
    const MaterialId red = materialNamed(context, model, "red");
    const MaterialId blue = materialNamed(context, model, "blue");
    CHECK(red.valid() && blue.valid());
    if (!red.valid() || !blue.valid()) return;
    const MaterialInfo* redInfo = context.materials->get(red);
    const MaterialInfo* blueInfo = context.materials->get(blue);
    CHECK(sameFloat(redInfo->roughness, 0.7f));
    CHECK(redInfo->albedo.x == 0.25f && redInfo->albedo.z == 0.75f);
    CHECK(redInfo->shading == MaterialShading::Lit);
    CHECK(sameFloat(blueInfo->roughness, 0.3f));
    CHECK(blueInfo->shading == MaterialShading::Unshaded);
    CHECK(redInfo->path == under(directory.c_str(), "cube.obj"));
    CHECK(redInfo->textures[0].valid() && !blueInfo->textures[0].valid());
    const TextureInfo* texture = TextureStore::singleton()->get(redInfo->textures[0]);
    CHECK(texture != nullptr && texture->path == under(directory.c_str(), "skin.png"));
    context.loader->freeModel(model);
}

void testSidecarByOrder(Context& context)
{
    const ct::String directory = under(kRoot, "side_order");
    makeCube(directory.c_str());
    CHECK(writeText(under(directory.c_str(), "cube.mat"),
            "material \"first\" {\n    roughness = 0.11\n}\n"
            "material \"second\" {\n    roughness = 0.22\n}\n"));
    const ModelId model = loadModel(context, under(directory.c_str(), "cube.obj"), true);
    const ModelInfo* info = context.loader->model(model);
    CHECK(info != nullptr && info->materials.size() == 2);
    if (!info || info->materials.size() != 2) return;
    const MaterialInfo* first = context.materials->get(info->materials[0]);
    const MaterialInfo* second = context.materials->get(info->materials[1]);
    CHECK(first->name == "first" && sameFloat(first->roughness, 0.11f));
    CHECK(second->name == "second" && sameFloat(second->roughness, 0.22f));
    context.loader->freeModel(model);

    const ct::String mixed = under(kRoot, "side_mixed");
    makeCube(mixed.c_str());
    CHECK(writeText(under(mixed.c_str(), "cube.mat"),
            "material \"red\" {\n    roughness = 0.4\n}\n"
            "material \"other\" {\n    roughness = 0.6\n}\n"));
    const ModelId second_model = loadModel(context, under(mixed.c_str(), "cube.obj"), true);
    const ModelInfo* mixedInfo = context.loader->model(second_model);
    CHECK(mixedInfo != nullptr && mixedInfo->materials.size() == 2);
    if (mixedInfo && mixedInfo->materials.size() == 2)
    {
        const MaterialInfo* red = context.materials->get(mixedInfo->materials[0]);
        const MaterialInfo* blue = context.materials->get(mixedInfo->materials[1]);
        CHECK(red->name == "red" && sameFloat(red->roughness, 0.4f));
        CHECK(blue->name == "other" && sameFloat(blue->roughness, 0.6f));
    }
    context.loader->freeModel(second_model);
}

void testSidecarUnmatched(Context& context)
{
    const ct::String directory = under(kRoot, "side_none");
    makeCube(directory.c_str());
    CHECK(writeText(under(directory.c_str(), "cube.mat"),
            "material \"x\" {\n    roughness = 0.11\n}\n"
            "material \"y\" {\n    roughness = 0.22\n}\n"
            "material \"z\" {\n    roughness = 0.33\n}\n"));
    const ModelId model = loadModel(context, under(directory.c_str(), "cube.obj"), true);
    const ModelInfo* info = context.loader->model(model);
    CHECK(info != nullptr && info->materials.size() == 2);
    if (info && info->materials.size() == 2)
    {
        const MaterialInfo* red = context.materials->get(info->materials[0]);
        const MaterialInfo* blue = context.materials->get(info->materials[1]);
        CHECK(red->name == "red" && blue->name == "blue");
        CHECK(sameFloat(red->roughness, 1.0f) && sameFloat(blue->roughness, 1.0f));
        CHECK(red->path == under(directory.c_str(), "cube.obj"));
    }
    context.loader->freeModel(model);
}

void testSidecarSwitchAndOtherExtension(Context& context)
{
    const ct::String directory = under(kRoot, "side_off");
    makeCube(directory.c_str());
    CHECK(writeText(under(directory.c_str(), "cube.mat"),
            "material \"red\" {\n    roughness = 0.5\n}\n"));
    const ModelId off = loadModel(context, under(directory.c_str(), "cube.obj"), false);
    CHECK(context.materials->get(materialNamed(context, off, "red"))->roughness == 1.0f);
    context.loader->freeModel(off);
    const ModelId on = loadModel(context, under(directory.c_str(), "cube.obj"), true);
    CHECK(context.materials->get(materialNamed(context, on, "red"))->roughness == 0.5f);
    context.loader->freeModel(on);

    const ct::String other = under(kRoot, "side_material_ext");
    makeCube(other.c_str());
    CHECK(writeText(under(other.c_str(), "cube.material"), "this is not a material library {{{"));
    const ModelId model = loadModel(context, under(other.c_str(), "cube.obj"), true);
    CHECK(context.materials->get(materialNamed(context, model, "red"))->roughness == 1.0f);
    context.loader->freeModel(model);

    const ct::String broken = under(kRoot, "side_broken");
    makeCube(broken.c_str());
    CHECK(writeText(under(broken.c_str(), "cube.mat"), "material \"red\" {\n  cull = Bogus\n}\n"));
    const ModelId failed = loadModel(context, under(broken.c_str(), "cube.obj"), true);
    CHECK(materialNamed(context, failed, "red").valid());
    CHECK(context.materials->get(materialNamed(context, failed, "red"))->cull ==
            MaterialCull::Back);
    context.loader->freeModel(failed);
}

void testSidecarMissingTexture(Context& context)
{
    const ct::String directory = under(kRoot, "side_missing");
    makeCube(directory.c_str());
    CHECK(writeText(under(directory.c_str(), "cube.mat"),
            "material \"red\" {\n    roughness = 0.45\n    albedo = \"absent.png\"\n"
            "    normal = \"skin.png\"\n}\n"));
    const ModelId model = loadModel(context, under(directory.c_str(), "cube.obj"), true);
    const MaterialId red = materialNamed(context, model, "red");
    CHECK(red.valid());
    if (red.valid())
    {
        const MaterialInfo* info = context.materials->get(red);
        CHECK(sameFloat(info->roughness, 0.45f));
        CHECK(!info->textures[static_cast<std::size_t>(MaterialTexture::Albedo)].valid());
        CHECK(info->textures[static_cast<std::size_t>(MaterialTexture::Normal)].valid());
    }
    context.loader->freeModel(model);
}

void testSidecarWrittenFromModel(Context& context)
{
    const ct::String directory = under(kRoot, "side_write");
    makeCube(directory.c_str());
    const ct::String modelPath = under(directory.c_str(), "cube.obj");
    const ModelId model = loadModel(context, modelPath, true);
    const MaterialId red = materialNamed(context, model, "red");
    const MaterialId blue = materialNamed(context, model, "blue");
    CHECK(red.valid() && blue.valid());
    if (!red.valid() || !blue.valid()) return;

    MaterialStore& store = *context.materials;
    store.setFloat(red, MaterialParam::Roughness, 0.35f);
    store.setFloat(red, MaterialParam::Metallic, 0.6f);
    store.setColor(red, MaterialParam::Albedo, Math::Vec4(0.2f, 0.4f, 0.6f, 1.0f));
    store.setShading(blue, MaterialShading::Unshaded);
    store.setCull(blue, MaterialCull::Disabled);
    store.setTransparency(blue, MaterialTransparency::AlphaScissor);
    store.setFloat(blue, MaterialParam::AlphaScissorThreshold, 0.4f);
    store.setGrass(blue, true, 33.0f, 0.2f);

    ct::String error;
    CHECK(!saveModelMaterials(store, *context.loader, ModelId(), error));
    CHECK(error == "the model is not loaded");
    CHECK(saveModelMaterials(store, *context.loader, model, error));
    const ct::String sidecar = under(directory.c_str(), "cube.mat");
    CHECK(FileSystem::getSingleton().exists(sidecar));

    MaterialLibraryData written;
    CHECK(MaterialLibraryFormat::load(sidecar, written, error));
    CHECK(written.materials.size() == 2);
    const MaterialDef* writtenRed = written.find("red");
    CHECK(writtenRed != nullptr);
    if (writtenRed) CHECK(writtenRed->paths[0] == "skin.png");

    const MaterialInfo savedRed = *store.get(red);
    const MaterialInfo savedBlue = *store.get(blue);
    context.loader->freeModel(model);

    const ModelId again = loadModel(context, modelPath, true);
    const MaterialId red2 = materialNamed(context, again, "red");
    const MaterialId blue2 = materialNamed(context, again, "blue");
    CHECK(red2.valid() && blue2.valid());
    if (red2.valid() && blue2.valid())
    {
        CHECK(materialFieldDifferences(*store.get(red2), savedRed) == 0);
        CHECK(materialFieldDifferences(*store.get(blue2), savedBlue) == 0);
        CHECK(store.get(red2)->textures[0] == savedRed.textures[0]);
        CHECK(store.get(red2)->path == modelPath);
    }
    context.loader->freeModel(again);

    const ct::String archived = under(kRoot, "side_nofile");
    ct::Vector<MaterialId> none;
    CHECK(!saveMaterials(store, none, under(archived.c_str(), "x.mat"), error));
}

void testSequenceRoundTrip(Context& context)
{
    const ct::String textures = under(kRoot, "sequence/tex");
    const ct::String library = under(kRoot, "sequence/lib");
    CHECK(fs_create_directory(under(kRoot, "sequence").c_str()));
    CHECK(fs_create_directory(textures.c_str()));
    CHECK(fs_create_directory(library.c_str()));
    ct::String frames[3];
    const char* const names[3] = { "f0.png", "f1.png", "f2.png" };
    for (int i = 0; i < 3; ++i)
    {
        frames[i] = under(textures.c_str(), names[i]);
        CHECK(savePng(frames[i], static_cast<std::uint8_t>(40 + i * 60)));
    }
    const ct::String file = under(library.c_str(), "water.mat");
    CHECK(writeText(file, "material \"a\" {\n"
                          "    albedo = [\"../tex/f0.png\", \"../tex/f1.png\", \"../tex/f2.png\"]\n"
                          "    albedo_fps = 4\n"
                          "    normal = [\"../tex/f2.png\", \"../tex/f0.png\"]\n"
                          "}\n"
                          "material \"b\" {\n"
                          "    albedo = [\"../tex/f0.png\", \"../tex/f1.png\", \"../tex/f2.png\"]\n"
                          "    albedo_fps = 4\n"
                          "}\n"
                          "material \"c\" {\n"
                          "    albedo = [\"../tex/f0.png\", \"../tex/f1.png\", \"../tex/f2.png\"]\n"
                          "    albedo_fps = 8\n"
                          "}\n"
                          "material \"d\" {\n"
                          "    albedo = [\"../tex/f0.png\", \"../tex/missing.png\"]\n"
                          "}\n"
                          "material \"e\" {\n"
                          "    detail = true\n"
                          "    detail_albedo = [\"../tex/f0.png\", \"../tex/f1.png\"]\n"
                          "    detail_albedo_fps = 2\n"
                          "    detail_mask = \"../tex/f2.png\"\n"
                          "}\n"));

    MaterialStore& store = *context.materials;
    TextureStore& textureStore = *TextureStore::singleton();
    ct::String error;
    {
        LoadedMaterialLibrary loaded(store, *context.loader);
        CHECK(loaded.load(file, error));
        CHECK(loaded.count() == 5);
        CHECK(drain(*context.loader));
        const MaterialInfo* a = store.get(loaded.find("a"));
        const MaterialInfo* b = store.get(loaded.find("b"));
        const MaterialInfo* c = store.get(loaded.find("c"));
        const MaterialInfo* d = store.get(loaded.find("d"));
        const MaterialInfo* e = store.get(loaded.find("e"));
        CHECK(a && b && c && d && e);
        if (!(a && b && c && d && e)) return;
        const std::size_t albedo = static_cast<std::size_t>(MaterialTexture::Albedo);
        const std::size_t normal = static_cast<std::size_t>(MaterialTexture::Normal);
        const TextureId proxy = a->textures[albedo];
        CHECK(proxy.valid() && textureStore.isProxy(proxy));
        CHECK(b->textures[albedo] == proxy);
        CHECK(c->textures[albedo].valid() && c->textures[albedo] != proxy);
        CHECK(textureStore.isProxy(c->textures[albedo]));
        CHECK(!d->textures[albedo].valid());
        CHECK(a->textures[normal].valid() && a->textures[normal] != proxy);

        const TextureId detailProxy =
                e->textures[static_cast<std::size_t>(MaterialTexture::DetailAlbedo)];
        CHECK(e->detail && detailProxy.valid() && textureStore.isProxy(detailProxy));
        CHECK(detailProxy != proxy);
        const AnimatedTexture* detailAnimated = context.loader->animatedTexture(detailProxy);
        CHECK(detailAnimated != nullptr && detailAnimated->getFrames() == 2);
        if (detailAnimated)
        {
            CHECK(detailAnimated->getFrameDuration(0) == 0.5f);
            CHECK(textureStore.get(detailAnimated->getFrameTexture(0))->srgb);
        }
        const TextureId maskTexture =
                e->textures[static_cast<std::size_t>(MaterialTexture::DetailMask)];
        CHECK(maskTexture.valid() && !textureStore.isProxy(maskTexture));
        CHECK(!textureStore.get(maskTexture)->srgb);

        AnimatedTexture* animated = context.loader->animatedTexture(proxy);
        CHECK(animated != nullptr);
        CHECK(context.loader->animatedTexture(c->textures[albedo]) != animated);
        CHECK(context.loader->animatedTexture(TextureId()) == nullptr);
        if (animated)
        {
            CHECK(animated->getFrames() == 3);
            CHECK(animated->getFrameDuration(0) == 0.25f && animated->getFrameDuration(2) == 0.25f);
            CHECK(animated->getCurrentFrame() == 0);
            const TextureId first = context.loader->requestTexture(frames[0], true, true);
            CHECK(first.valid() && animated->getFrameTexture(0) == first);
            CHECK(textureStore.proxyTarget(proxy) == first);
            CHECK(textureStore.get(first)->srgb);
            CHECK(textureStore.get(animated->getFrameTexture(2))->srgb);
            animated->setAutoUpdate(false);
            animated->update(0.3f);
            CHECK(animated->getCurrentFrame() == 1);
            CHECK(textureStore.proxyTarget(proxy) == animated->getFrameTexture(1));
            CHECK(textureStore.proxyTarget(b->textures[albedo]) == animated->getFrameTexture(1));
            CHECK(textureStore.get(proxy)->sequence.size() == 3);
            CHECK(textureStore.get(proxy)->sequenceFps == 4.0f);
        }
        const AnimatedTexture* second = context.loader->animatedTexture(a->textures[normal]);
        CHECK(second != nullptr && second->getFrames() == 2);
        if (second)
        {
            CHECK(second->getFrameDuration(0) == 1.0f);
            CHECK(!textureStore.get(second->getFrameTexture(0))->srgb);
        }

        MaterialDef captured;
        CHECK(captureMaterial(store, loaded.find("a"), captured));
        CHECK(captured.paths[albedo].empty());
        CHECK(captured.sequences[albedo].size() == 3);
        CHECK(captured.sequences[albedo].size() == 3 && captured.sequences[albedo][1] == frames[1]);
        CHECK(captured.sequenceFps[albedo] == 4.0f);
        CHECK(captured.sequences[normal].size() == 2 && captured.sequenceFps[normal] == 0.0f);

        MaterialDef missing;
        CHECK(captureMaterial(store, loaded.find("d"), missing));
        CHECK(missing.sequences[albedo].empty() && missing.paths[albedo].empty());

        const ct::String saved = under(library.c_str(), "water_saved.mat");
        ct::Vector<MaterialId> ids;
        ids.push_back(loaded.find("a"));
        ids.push_back(loaded.find("b"));
        ids.push_back(loaded.find("c"));
        CHECK(saveMaterials(store, ids, saved, error));
        MaterialLibraryData written;
        CHECK(MaterialLibraryFormat::load(saved, written, error));
        CHECK(written.materials.size() == 3);
        if (written.materials.size() == 3)
        {
            CHECK(written.materials[0].sequences[albedo].size() == 3);
            CHECK(written.materials[0].sequences[albedo].size() == 3 &&
                    written.materials[0].sequences[albedo][0] == "../tex/f0.png");
            CHECK(written.materials[0].sequenceFps[albedo] == 4.0f);
            CHECK(written.materials[0].sequences[normal].size() == 2);
            CHECK(written.materials[2].sequenceFps[albedo] == 8.0f);
            CHECK(written.materials[1].sequences[normal].empty());
        }

        LoadedMaterialLibrary again(store, *context.loader);
        CHECK(again.load(saved, error));
        const MaterialInfo* a2 = store.get(again.find("a"));
        const MaterialInfo* c2 = store.get(again.find("c"));
        CHECK(a2 && c2);
        if (a2 && c2)
        {
            CHECK(a2->textures[albedo] == proxy);
            CHECK(a2->textures[normal] == a->textures[normal]);
            CHECK(c2->textures[albedo] == c->textures[albedo]);
        }
        CHECK(drain(*context.loader));
    }
}

void testCaptureNames(Context& context)
{
    MaterialStore& store = *context.materials;
    const MaterialId unnamed = store.create();
    const MaterialId quoted = store.create("a\"b\nc");
    const MaterialId first = store.create("same");
    const MaterialId second = store.create("same");
    ct::Vector<MaterialId> ids;
    ids.push_back(unnamed);
    ids.push_back(quoted);
    ids.push_back(first);
    ids.push_back(second);
    ids.push_back(MaterialId());

    MaterialLibraryData data;
    CHECK(captureMaterials(store, ids, data));
    CHECK(data.materials.size() == 3);
    CHECK(data.warnings.size() == 1);
    CHECK(data.warnings[0] == "duplicate material name 'same' skipped");
    if (data.materials.size() == 3)
    {
        CHECK(data.materials[0].info.name == "material_0");
        CHECK(data.materials[1].info.name == "a_b_c");
        CHECK(data.materials[2].info.name == "same");
    }
    ct::String text;
    MaterialLibraryFormat::write(data, text);
    MaterialLibraryData back;
    ct::String error;
    CHECK(MaterialLibraryFormat::parse(text.c_str(), text.size(), back, error));
    CHECK(back.materials.size() == 3);

    MaterialDef def;
    CHECK(!captureMaterial(store, MaterialId(), def));
    CHECK(!applyMaterial(store, *context.loader, MaterialId(), def, ct::String()));
    store.destroy(unnamed);
    store.destroy(quoted);
    store.destroy(first);
    store.destroy(second);
}

} // namespace

int main()
{
    testPathHelpers();

    prisma::DriverDesc desc;
    desc.type = prisma::DriverType::Null;
    prisma::DriverError driverError = prisma::DriverError::None;
    prisma::Driver* driver = prisma::createDriver(desc, &driverError);
    CHECK(driver != nullptr);
    if (!driver) return 1;

    CHECK(fs_create_directory(kRoot));
    {
        TextureStore textures(driver);
        MeshStore meshes(driver);
        MaterialStore materials(textures);
        ModelLoader importers;
        importers.addImporter(new ObjImporter());
        {
            AssetLoader loader(textures, materials, meshes, importers, 2);
            Context context;
            context.materials = &materials;
            context.meshes = &meshes;
            context.loader = &loader;

            testEveryFieldRoundTrip(context);
            testReloadInPlace(context);
            testBinding(context);
            testSidecarByName(context);
            testSidecarByOrder(context);
            testSidecarUnmatched(context);
            testSidecarSwitchAndOtherExtension(context);
            testSidecarMissingTexture(context);
            testSidecarWrittenFromModel(context);
            testSequenceRoundTrip(context);
            testCaptureNames(context);
            loader.waitAll();
        }
    }
    prisma::destroyDriver(driver);
    return failures ? 1 : 0;
}
