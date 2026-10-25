#include "Check.h"
#include "MaterialFields.h"
#include "PMeshSupport.h"

#include "prisma/rhi/Driver.h"
#include "prisma3d/core/FileSystem.h"
#include "prisma3d/import/GltfImporter.h"
#include "prisma3d/import/ObjImporter.h"
#include "prisma3d/import/OgreMeshImporter.h"
#include "prisma3d/import/PMeshImporter.h"
#include "prisma3d/import/PMeshWriter.h"
#include "prisma3d/render/AssetLoader.h"
#include "prisma3d/render/LoadedMaterialLibrary.h"

#include "miniz.h"
#include "platform.h"

#include <stdio.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const char* const kRoot = PRISMA3D_TEST_DIR "/pmesh_loader";
const char* const kAssets = PRISMA3D_ASSETS_DIR "/models";

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

bool drain(AssetLoader& loader)
{
    const std::uint64_t start = time_nanos();
    while (loader.pendingRequests() != 0)
    {
        if (time_nanos() - start > 60000000000ull) return false;
        loader.pump();
        time_sleep(1);
    }
    loader.pump();
    return true;
}

ModelId loadModel(Context& context, const ct::String& path)
{
    const ModelId model = context.loader->requestModel(path);
    CHECK(drain(*context.loader));
    CHECK(context.loader->modelStatus(model) == LoadStatus::Loaded);
    return model;
}

SurfaceData quad()
{
    SurfaceData s;
    const float xs[4] = { 0.0f, 1.0f, 1.0f, 0.0f };
    const float ys[4] = { 0.0f, 0.0f, 1.0f, 1.0f };
    for (int i = 0; i < 4; ++i)
    {
        s.positions.push_back(Math::Vec3(xs[i], ys[i], 0.0f));
        s.normals.push_back(Math::Vec3(0.0f, 0.0f, 1.0f));
        s.uvs.push_back(Math::Vec2(xs[i], ys[i]));
    }
    const std::uint32_t indices[6] = { 0, 1, 2, 0, 2, 3 };
    for (std::uint32_t index: indices) s.indices.push_back(index);
    return s;
}

void addPrimitive(ModelData& model, MemoryImporter& importer, const char* meshName, int material)
{
    ModelMeshData mesh;
    mesh.name = meshName;
    ModelPrimitiveData primitive;
    const SurfaceData surface = quad();
    primitive.material = material;
    primitive.attributeMask = kVertexPosition | kVertexNormal | kVertexUv;
    primitive.vertexCount = 4;
    primitive.indexCount = 6;
    primitive.source = static_cast<std::uint32_t>(importer.surfaces.size());
    primitive.aabb = MeshStore::computeAabb(surface.positions);
    primitive.hasAabb = true;
    mesh.primitives.push_back(primitive);
    importer.surfaces.push_back(surface);
    model.meshes.push_back(mesh);
    ModelNodeData node;
    node.name = meshName;
    node.mesh = static_cast<int>(model.meshes.size() - 1);
    model.nodes.push_back(node);
    model.roots.push_back(static_cast<int>(model.nodes.size() - 1));
}

void addMaterial(ModelData& model, const char* name)
{
    ModelMaterialData material;
    material.name = name;
    model.materials.push_back(material);
}

const MaterialInfo* materialNamed(Context& context, ModelId model, const char* name)
{
    const ModelInfo* info = context.loader->model(model);
    if (!info) return nullptr;
    for (std::size_t i = 0; i < info->materials.size(); ++i)
    {
        const MaterialInfo* material = context.materials->get(info->materials[i]);
        if (material && material->name == name) return material;
    }
    return nullptr;
}

void testSidecarByName(Context& context)
{
    const ct::String directory = under(kRoot, "names");
    CHECK(fs_create_directory(directory.c_str()));
    MemoryImporter source;
    ModelData model;
    addMaterial(model, "a");
    addMaterial(model, "b");
    addPrimitive(model, source, "first", 0);
    addPrimitive(model, source, "second", 1);
    ct::String error;
    CHECK(savePMesh(model, source, under(directory.c_str(), "box.pmesh"), PMeshSaveOptions(),
            error));

    const ct::String path = under(directory.c_str(), "box.pmesh");
    fs_remove_path(under(directory.c_str(), "box.mat").c_str());
    {
        const ModelId id = loadModel(context, path);
        const ModelInfo* info = context.loader->model(id);
        CHECK(info != nullptr && info->materials.size() == 2 && info->meshes.size() == 2);
        CHECK(info != nullptr && info->unmatchedMaterials.empty());
        const MaterialInfo* a = materialNamed(context, id, "a");
        CHECK(a != nullptr && a->roughness == 1.0f);
        context.loader->freeModel(id);
    }

    CHECK(writeText(under(directory.c_str(), "box.mat"),
            "material \"x\" {\n    roughness = 0.31\n}\n"
            "material \"b\" {\n    roughness = 0.42\n    metallic = 0.5\n}\n"
            "material \"y\" {\n    roughness = 0.53\n}\n"));
    {
        const ModelId id = loadModel(context, path);
        const ModelInfo* info = context.loader->model(id);
        CHECK(info != nullptr && info->materials.size() == 2);
        const MaterialInfo* a = materialNamed(context, id, "a");
        const MaterialInfo* b = materialNamed(context, id, "b");
        CHECK(a != nullptr && b != nullptr);
        if (a && b)
        {
            CHECK(a->roughness == 1.0f && a->metallic == 1.0f);
            CHECK(b->roughness == 0.42f && b->metallic == 0.5f);
            CHECK(b->path == under(directory.c_str(), "box.mat"));
        }
        CHECK(info != nullptr && info->unmatchedMaterials.size() == 1 &&
                info->unmatchedMaterials[0] == "a");
        if (info && info->meshes.size() == 2)
        {
            const MeshInfo* second = context.meshes->get(info->meshes[1]);
            CHECK(second != nullptr && second->surfaces.size() == 1);
            if (second && !second->surfaces.empty() && b)
                CHECK(context.materials->get(second->surfaces[0].material) == b);
        }
        context.loader->freeModel(id);
    }

    ModelOptions off;
    off.materialSidecar = false;
    const ModelId id = context.loader->requestModel(path, off);
    CHECK(drain(*context.loader));
    const MaterialInfo* b = materialNamed(context, id, "b");
    CHECK(b != nullptr && b->roughness == 1.0f);
    const ModelInfo* info = context.loader->model(id);
    CHECK(info != nullptr && info->unmatchedMaterials.empty());
    context.loader->freeModel(id);
}

void testShadowMesh(Context& context)
{
    const ct::String directory = under(kRoot, "shadow");
    CHECK(fs_create_directory(directory.c_str()));
    MemoryImporter source;
    ModelData model;
    addMaterial(model, "m");
    addPrimitive(model, source, "tree", 0);
    SurfaceData shadow;
    shadow.positions.push_back(Math::Vec3(0.0f, 0.0f, 0.0f));
    shadow.positions.push_back(Math::Vec3(1.0f, 0.0f, 0.0f));
    shadow.positions.push_back(Math::Vec3(0.0f, 1.0f, 0.0f));
    shadow.indices.push_back(0);
    shadow.indices.push_back(1);
    shadow.indices.push_back(2);
    model.meshes[0].shadowSurfaces.push_back(shadow);
    ct::String error;
    CHECK(savePMesh(model, source, under(directory.c_str(), "tree.pmesh"), PMeshSaveOptions(),
            error));
    const ModelId id = loadModel(context, under(directory.c_str(), "tree.pmesh"));
    const ModelInfo* info = context.loader->model(id);
    CHECK(info != nullptr && info->meshes.size() == 1);
    if (info && info->meshes.size() == 1)
    {
        const MeshId shadowMesh = context.meshes->getShadowMesh(info->meshes[0]);
        CHECK(shadowMesh.valid());
        const MeshInfo* shadowInfo = context.meshes->get(shadowMesh);
        CHECK(shadowInfo != nullptr && shadowInfo->surfaces.size() == 1);
        if (shadowInfo && shadowInfo->surfaces.size() == 1)
        {
            CHECK(shadowInfo->surfaces[0].vertexCount == 3 &&
                    shadowInfo->surfaces[0].indexCount == 3);
        }
    }
    context.loader->freeModel(id);
}

bool buildZip(const char* path, const ct::Vector<ct::String>& names,
        const ct::Vector<ct::Vector<std::uint8_t>>& contents)
{
    mz_zip_archive zip;
    memset(&zip, 0, sizeof(zip));
    if (!mz_zip_writer_init_heap(&zip, 0, 0)) return false;
    bool ok = true;
    for (std::size_t i = 0; ok && i < names.size(); ++i)
        ok = mz_zip_writer_add_mem(&zip, names[i].c_str(), contents[i].data(), contents[i].size(),
                     MZ_BEST_COMPRESSION) != 0;
    void* data = nullptr;
    size_t size = 0;
    ok = ok && mz_zip_writer_finalize_heap_archive(&zip, &data, &size);
    ok = ok && file_write(path, data, size);
    mz_zip_writer_end(&zip);
    if (data) free(data);
    return ok;
}

ct::Vector<std::uint8_t> textBytes(const char* text)
{
    ct::Vector<std::uint8_t> bytes;
    for (const char* c = text; *c; ++c) bytes.push_back(static_cast<std::uint8_t>(*c));
    return bytes;
}

void testArchive(Context& context)
{
    MemoryImporter source;
    ModelData model;
    addMaterial(model, "skin");
    addMaterial(model, "metal");
    addPrimitive(model, source, "body", 0);
    addPrimitive(model, source, "gun", 1);
    Skeleton skeleton;
    CHECK(skeleton.addBone("root", -1, Math::Mat4::Identity(), Math::Mat4::Identity()));
    CHECK(skeleton.addBone("tip", 0, Math::Mat4::Identity(), Math::Mat4::Identity()));
    CHECK(skeleton.finalize());
    model.skeletons.push_back(skeleton);
    AnimationClip clip;
    clip.setName("idle");
    clip.setDuration(1.0f);
    clip.setSkeleton(0);
    BoneTrack track;
    track.bone = 1;
    track.times.push_back(0.0f);
    track.positions.push_back(Math::Vec3(0.0f, 1.0f, 0.0f));
    track.rotations.push_back(Math::Quaternion(0.0f, 0.0f, 0.0f, 1.0f));
    track.scales.push_back(Math::Vec3(1.0f, 1.0f, 1.0f));
    clip.tracks().push_back(track);
    model.animations.push_back(clip);

    PMeshFiles files;
    ct::String error;
    CHECK(buildPMesh(model, source, "hero", PMeshSaveOptions(), files, error));

    ct::Vector<ct::String> names;
    ct::Vector<ct::Vector<std::uint8_t>> contents;
    names.push_back("arch/hero.pmesh");
    contents.push_back(files.mesh);
    names.push_back("arch/hero.pskel");
    contents.push_back(files.skeletons[0]);
    names.push_back("arch/hero.panim");
    contents.push_back(files.animations);
    names.push_back("arch/hero.mat");
    contents.push_back(textBytes("material \"skin\" {\n    roughness = 0.35\n}\n"));

    CHECK(fs_create_directory(kRoot));
    const ct::String zip = under(kRoot, "models.zip");
    CHECK(buildZip(zip.c_str(), names, contents));
    FileSystem& fileSystem = FileSystem::getSingleton();
    CHECK(fileSystem.mountArchive(zip));
    CHECK(!fileSystem.resolve("arch/hero.pmesh").empty() || fileSystem.exists("arch/hero.pmesh"));

    const ModelId id = loadModel(context, "arch/hero.pmesh");
    const ModelInfo* info = context.loader->model(id);
    CHECK(info != nullptr && info->materials.size() == 2 && info->meshes.size() == 2);
    CHECK(info != nullptr && info->document.skeletons.size() == 1 &&
            info->document.skeletons[0].boneCount() == 2);
    CHECK(info != nullptr && info->document.animations.size() == 1 &&
            info->document.animations[0].name() == "idle");
    const MaterialInfo* skin = materialNamed(context, id, "skin");
    const MaterialInfo* metal = materialNamed(context, id, "metal");
    CHECK(skin != nullptr && skin->roughness == 0.35f);
    CHECK(metal != nullptr && metal->roughness == 1.0f);
    CHECK(info != nullptr && info->unmatchedMaterials.size() == 1 &&
            info->unmatchedMaterials[0] == "metal");
    context.loader->freeModel(id);
    fileSystem.unmountAll();
}

void testConvert(Context& context)
{
    const ct::String source = ct::String(kAssets) + "/DamagedHelmet/DamagedHelmet.gltf";
    const ct::String directory = under(kRoot, "helmet");
    CHECK(fs_create_directory(directory.c_str()));
    const ct::String exact = under(directory.c_str(), "Helmet.pmesh");
    ct::String error;
    PMeshConvertOptions options;
    options.bakeTangents = false;
    CHECK(convertModelToPMesh(*context.loader, *context.materials, source, exact, options, error));
    CHECK(error.empty());
    CHECK(FileSystem::getSingleton().exists(exact));
    CHECK(FileSystem::getSingleton().exists(under(directory.c_str(), "Helmet.mat")));

    const ModelId original = loadModel(context, source);
    const ModelId converted = loadModel(context, exact);
    const ModelInfo* a = context.loader->model(original);
    const ModelInfo* b = context.loader->model(converted);
    CHECK(a != nullptr && b != nullptr);
    if (a && b)
    {
        CHECK(modelDifferences(a->document, *a->importer, b->document, *b->importer, false) == 0);
        CHECK(a->materials.size() == b->materials.size());
        for (std::size_t i = 0; i < a->materials.size() && i < b->materials.size(); ++i)
        {
            const MaterialInfo* x = context.materials->get(a->materials[i]);
            const MaterialInfo* y = context.materials->get(b->materials[i]);
            CHECK(x != nullptr && y != nullptr);
            if (!x || !y) continue;
            CHECK(x->name == y->name);
            CHECK(materialFieldDifferences(*x, *y) == 0);
            int textures = 0;
            for (std::size_t s = 0; s < kMaterialTextureCount; ++s)
                textures += x->textures[s].valid() != y->textures[s].valid();
            CHECK(textures == 0);
            CHECK(y->path == under(directory.c_str(), "Helmet.mat"));
        }
        CHECK(b->unmatchedMaterials.empty());
    }
    context.loader->freeModel(original);
    context.loader->freeModel(converted);

    const ct::String baked = under(directory.c_str(), "HelmetBaked.pmesh");
    PMeshConvertOptions bakeOptions;
    CHECK(convertModelToPMesh(*context.loader, *context.materials, source, baked, bakeOptions,
            error));
    PMeshImporter reader;
    ModelData bakedData;
    CHECK(reader.import(baked, bakedData));
    bool tangents = !bakedData.meshes.empty();
    for (std::size_t m = 0; m < bakedData.meshes.size(); ++m)
        for (std::size_t p = 0; p < bakedData.meshes[m].primitives.size(); ++p)
            tangents = tangents &&
                       (bakedData.meshes[m].primitives[p].attributeMask & kVertexTangent) != 0;
    CHECK(tangents);

    CHECK(!convertModelToPMesh(*context.loader, *context.materials,
            under(directory.c_str(), "nothing.gltf"), exact, options, error));
    CHECK(!error.empty());

    const ct::String sinbad = ct::String(kAssets) + "/sinbad/Sinbad.mesh";
    const ct::String sinbadTarget = under(directory.c_str(), "Sinbad.pmesh");
    CHECK(convertModelToPMesh(*context.loader, *context.materials, sinbad, sinbadTarget, options,
            error));
    CHECK(FileSystem::getSingleton().exists(under(directory.c_str(), "Sinbad.pskel")));
    CHECK(FileSystem::getSingleton().exists(under(directory.c_str(), "Sinbad.panim")));
    const ModelId skinned = loadModel(context, sinbadTarget);
    const ModelId first = loadModel(context, sinbad);
    const ModelInfo* sk = context.loader->model(skinned);
    const ModelInfo* fo = context.loader->model(first);
    CHECK(sk != nullptr && fo != nullptr);
    if (sk && fo)
    {
        CHECK(modelDifferences(fo->document, *fo->importer, sk->document, *sk->importer, false) ==
                0);
        CHECK(sk->document.skeletons.size() == 1 &&
                sk->document.animations.size() == fo->document.animations.size());
    }
    context.loader->freeModel(skinned);
    context.loader->freeModel(first);
}

std::uint64_t median(ct::Vector<std::uint64_t>& values)
{
    for (std::size_t i = 1; i < values.size(); ++i)
        for (std::size_t j = i; j > 0 && values[j] < values[j - 1]; --j)
        {
            const std::uint64_t t = values[j];
            values[j] = values[j - 1];
            values[j - 1] = t;
        }
    return values[values.size() / 2];
}

struct Timing
{
    std::uint64_t importNanos = 0;
    std::uint64_t surfaceNanos = 0;
    std::uint64_t vertices = 0;
    std::uint64_t indices = 0;
};

bool timeOnce(ModelLoader& loader, const ct::String& path, Timing& timing)
{
    ModelData data;
    const std::uint64_t t0 = time_nanos();
    if (!loader.load(path, data)) return false;
    const std::uint64_t t1 = time_nanos();
    ModelImporter* importer = loader.importerFor(path);
    std::uint64_t vertices = 0;
    std::uint64_t indices = 0;
    for (std::size_t m = 0; m < data.meshes.size(); ++m)
        for (std::size_t p = 0; p < data.meshes[m].primitives.size(); ++p)
        {
            SurfaceData surface;
            if (!importer->readPrimitive(data, data.meshes[m].primitives[p], surface)) return false;
            vertices += surface.positions.size();
            indices += surface.indices.size();
        }
    const std::uint64_t t2 = time_nanos();
    timing.importNanos = t1 - t0;
    timing.surfaceNanos = t2 - t1;
    timing.vertices = vertices;
    timing.indices = indices;
    return true;
}

void measure()
{
    ModelLoader loader;
    loader.addImporter(new ObjImporter());
    loader.addImporter(new PMeshImporter());
    const ct::String source = ct::String(kAssets) + "/powerplant/Powerplant.obj";
    const ct::String directory = under(kRoot, "measure");
    CHECK(fs_create_directory(directory.c_str()));
    const ct::String target = under(directory.c_str(), "Powerplant.pmesh");

    ModelData original;
    CHECK(loader.load(source, original));
    ct::String error;
    CHECK(savePMesh(original, *loader.importerFor(source), target, PMeshSaveOptions(), error));
    const std::uint64_t objBytes = FileSystem::getSingleton().readBinary(source).size();
    const std::uint64_t pmeshBytes = FileSystem::getSingleton().readBinary(target).size();

    const ct::String paths[2] = { source, target };
    const char* names[2] = { "Powerplant.obj", "Powerplant.pmesh" };
    const std::uint64_t sizes[2] = { objBytes, pmeshBytes };
    const int runs = 5;
    for (int f = 0; f < 2; ++f)
    {
        Timing warm;
        CHECK(timeOnce(loader, paths[f], warm));
        ct::Vector<std::uint64_t> imports;
        ct::Vector<std::uint64_t> surfaces;
        ct::Vector<std::uint64_t> totals;
        Timing last;
        for (int r = 0; r < runs; ++r)
        {
            CHECK(timeOnce(loader, paths[f], last));
            imports.push_back(last.importNanos);
            surfaces.push_back(last.surfaceNanos);
            totals.push_back(last.importNanos + last.surfaceNanos);
        }
        printf("%s: %llu bytes, %llu vertices, %llu indices; median of %d runs after one warm-up: "
               "import %.1f ms, surfaces %.1f ms, total %.1f ms\n",
                names[f], static_cast<unsigned long long>(sizes[f]),
                static_cast<unsigned long long>(last.vertices),
                static_cast<unsigned long long>(last.indices), runs,
                static_cast<double>(median(imports)) / 1e6,
                static_cast<double>(median(surfaces)) / 1e6,
                static_cast<double>(median(totals)) / 1e6);
    }
}

} // namespace

int main()
{
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
        importers.addImporter(new GltfImporter());
        importers.addImporter(new OgreMeshImporter());
        importers.addImporter(new PMeshImporter());
        {
            AssetLoader loader(textures, materials, meshes, importers, 2);
            Context context;
            context.materials = &materials;
            context.meshes = &meshes;
            context.loader = &loader;

            testSidecarByName(context);
            testShadowMesh(context);
            testArchive(context);
            testConvert(context);
            loader.waitAll();
        }
    }
    prisma::destroyDriver(driver);
    measure();
    return failures ? 1 : 0;
}
