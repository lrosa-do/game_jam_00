#include "Check.h"

#include "prisma3d/import/ObjImporter.h"

#include "platform.h"

#include <math.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const float kTolerance = 1e-4f;
const char* const kDirectory = PRISMA3D_TEST_DIR "/obj";

bool near(float a, float b)
{
    return fabsf(a - b) <= kTolerance;
}

bool nearVec3(const Math::Vec3& a, float x, float y, float z)
{
    return near(a.x, x) && near(a.y, y) && near(a.z, z);
}

bool nearVec2(const Math::Vec2& a, float x, float y)
{
    return near(a.x, x) && near(a.y, y);
}

ct::String pathOf(const char* name)
{
    return ct::String(kDirectory) + "/" + name;
}

bool writeText(const char* name, const char* text)
{
    return file_write(pathOf(name).c_str(), text, strlen(text));
}

const char* const kCube = "# quad and triangle\n"
                          "mtllib cube.mtl\n"
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

const char* const kCubeMaterials = "# materials\n"
                                   "newmtl red\n"
                                   "Ka 0.1 0.1 0.1\n"
                                   "Kd 1.0 0.0 0.0\n"
                                   "map_Kd skin.png\n"
                                   "\n"
                                   "newmtl blue\n"
                                   "Kd 0 0 1\n";

const char* const kLeaves = "mtllib leaves.mtl\n"
                            "v 0 0 0\n"
                            "v 1 0 0\n"
                            "v 0 1 0\n"
                            "v 1 1 0\n"
                            "usemtl red\n"
                            "f 1 2 3\n"
                            "usemtl leaf\n"
                            "f 2 3 4\n"
                            "usemtl red\n"
                            "f 3 2 4\n";

const char* const kLeavesMaterials = "newmtl red\r\n"
                                     "Kd 0.5 0.25 1e0\r\n"
                                     "map_Kd textures\\red.png\r\n"
                                     "newmtl leaf\r\n"
                                     "map_Kd leaf.png\r\n"
                                     "map_bump leaf_n.png\r\n"
                                     "map_d leaf.png\r\n";

void checkCube(const ModelImporter& importer, const ModelData& model, const char* nodeName)
{
    CHECK(model.error.empty());
    CHECK(model.nodes.size() == 1);
    CHECK(model.roots.size() == 1 && model.roots[0] == 0);
    CHECK(model.meshes.size() == 1);
    CHECK(model.primitiveCount == 2);
    CHECK(model.materials.size() == 2);
    if (model.nodes.size() != 1 || model.meshes.size() != 1 || model.materials.size() != 2) return;

    CHECK(model.nodes[0].name == nodeName);
    CHECK(model.nodes[0].parent == -1);
    CHECK(model.nodes[0].mesh == 0);
    CHECK(model.nodes[0].skeleton == -1);
    CHECK(model.meshes[0].name == nodeName);
    CHECK(model.meshes[0].primitives.size() == 2);
    if (model.meshes[0].primitives.size() != 2) return;

    CHECK(model.materials[0].name == "red");
    CHECK(near(model.materials[0].baseColor.x, 1.0f));
    CHECK(near(model.materials[0].baseColor.y, 0.0f));
    CHECK(near(model.materials[0].baseColor.z, 0.0f));
    CHECK(near(model.materials[0].baseColor.w, 1.0f));
    CHECK(model.materials[0].alphaMode == ModelAlphaMode::Opaque);
    CHECK(!model.materials[0].doubleSided);
    CHECK(model.materials[0].baseColorTexture == 0);
    CHECK(model.materials[0].normalTexture == -1);
    CHECK(model.materials[1].name == "blue");
    CHECK(near(model.materials[1].baseColor.z, 1.0f));
    CHECK(model.materials[1].baseColorTexture == -1);

    CHECK(model.textures.size() == 1);
    CHECK(model.images.size() == 1);
    if (model.textures.size() == 1 && model.images.size() == 1)
    {
        CHECK(model.textures[0].image == 0);
        CHECK(model.images[0].path == pathOf("skin.png"));
        CHECK(model.images[0].name == "skin.png");
        CHECK(!model.images[0].embedded());
        ct::Vector<std::uint8_t> bytes;
        bytes.push_back(7);
        CHECK(!importer.readImage(model, model.images[0], bytes));
        CHECK(bytes.empty());
    }

    const ModelPrimitiveData& quad = model.meshes[0].primitives[0];
    const ModelPrimitiveData& triangle = model.meshes[0].primitives[1];
    CHECK(quad.material == 0);
    CHECK(quad.vertexCount == 4);
    CHECK(quad.indexCount == 6);
    CHECK(quad.hasAabb);
    CHECK(quad.topology == prisma::Topology::Triangles);
    CHECK(!quad.skinned);
    CHECK((quad.attributeMask & kVertexNormal) != 0);
    CHECK((quad.attributeMask & kVertexUv) != 0);
    CHECK(triangle.material == 1);
    CHECK(triangle.vertexCount == 3);
    CHECK(triangle.indexCount == 3);
    CHECK(model.geometryBytes == quad.sourceBytes + triangle.sourceBytes);

    SurfaceData surface;
    CHECK(importer.readPrimitive(model, quad, surface));
    CHECK(surface.positions.size() == 4);
    CHECK(surface.normals.size() == 4);
    CHECK(surface.uvs.size() == 4);
    CHECK(surface.indices.size() == 6);
    if (surface.positions.size() == 4 && surface.normals.size() == 4 && surface.uvs.size() == 4 &&
            surface.indices.size() == 6)
    {
        const float positions[4][3] = { { 0, 0, 0 }, { 1, 0, 0 }, { 1, 1, 0 }, { 0, 1, 0 } };
        const float uvs[4][2] = { { 0, 1 }, { 1, 1 }, { 1, 0 }, { 0, 0 } };
        const std::uint32_t indices[6] = { 0, 1, 2, 0, 2, 3 };
        for (std::size_t i = 0; i < 4; ++i)
        {
            CHECK(nearVec3(surface.positions[i], positions[i][0], positions[i][1],
                    positions[i][2]));
            CHECK(nearVec3(surface.normals[i], 0.0f, 0.0f, 1.0f));
            CHECK(nearVec2(surface.uvs[i], uvs[i][0], uvs[i][1]));
        }
        for (std::size_t i = 0; i < 6; ++i) CHECK(surface.indices[i] == indices[i]);
    }
    CHECK(near(quad.aabb.min.x, 0.0f) && near(quad.aabb.max.x, 1.0f));
    CHECK(near(quad.aabb.min.y, 0.0f) && near(quad.aabb.max.y, 1.0f));

    SurfaceData second;
    CHECK(importer.readPrimitive(model, triangle, second));
    CHECK(second.positions.size() == 3);
    CHECK(second.normals.size() == 3);
    CHECK(second.uvs.size() == 3);
    CHECK(second.indices.size() == 3);
    if (second.positions.size() == 3 && second.normals.size() == 3 && second.uvs.size() == 3 &&
            second.indices.size() == 3)
    {
        CHECK(nearVec3(second.positions[0], 1, 1, 0));
        CHECK(nearVec3(second.positions[1], 1, 0, 0));
        CHECK(nearVec3(second.positions[2], 0, 0, 0));
        for (std::size_t i = 0; i < 3; ++i) CHECK(nearVec3(second.normals[i], 0.0f, 0.0f, -1.0f));
        CHECK(nearVec2(second.uvs[0], 1, 0));
        CHECK(nearVec2(second.uvs[1], 1, 1));
        CHECK(nearVec2(second.uvs[2], 0, 1));
        CHECK(second.indices[0] == 0 && second.indices[1] == 1 && second.indices[2] == 2);
    }

    ModelPrimitiveData invalid = triangle;
    invalid.source = 99;
    CHECK(!importer.readPrimitive(model, invalid, surface));
}

void checkLeaves(const ModelImporter& importer, const ModelData& model)
{
    CHECK(model.nodes.size() == 1);
    CHECK(model.meshes.size() == 1);
    CHECK(model.materials.size() == 2);
    if (model.meshes.size() != 1 || model.materials.size() != 2) return;
    CHECK(model.meshes[0].primitives.size() == 2);
    if (model.meshes[0].primitives.size() != 2) return;

    CHECK(model.meshes[0].primitives[0].material == 0);
    CHECK(model.meshes[0].primitives[0].indexCount == 6);
    CHECK(model.meshes[0].primitives[1].material == 1);
    CHECK(model.meshes[0].primitives[1].indexCount == 3);
    CHECK(model.meshes[0].primitives[0].vertexCount == 4);
    CHECK((model.meshes[0].primitives[0].attributeMask & kVertexUv) == 0);

    CHECK(near(model.materials[0].baseColor.x, 0.5f));
    CHECK(near(model.materials[0].baseColor.y, 0.25f));
    CHECK(near(model.materials[0].baseColor.z, 1.0f));
    CHECK(model.materials[0].alphaMode == ModelAlphaMode::Opaque);
    CHECK(model.materials[1].alphaMode == ModelAlphaMode::Mask);
    CHECK(model.materials[1].doubleSided);
    CHECK(near(model.materials[1].baseColor.x, 0.75f));
    CHECK(model.images.size() == 3);
    CHECK(model.textures.size() == 3);
    if (model.images.size() != 3) return;
    CHECK(model.images[0].path == pathOf("textures\\red.png"));
    CHECK(model.images[1].path == pathOf("leaf.png"));
    CHECK(model.images[2].path == pathOf("leaf_n.png"));
    CHECK(model.materials[0].baseColorTexture == 0);
    CHECK(model.materials[1].baseColorTexture == 1);
    CHECK(model.materials[1].normalTexture == 2);

    SurfaceData surface;
    CHECK(importer.readPrimitive(model, model.meshes[0].primitives[0], surface));
    CHECK(surface.uvs.empty());
    CHECK(surface.normals.size() == 4);
    CHECK(surface.indices.size() == 6);
    if (surface.normals.size() == 4 && surface.positions.size() == 4)
    {
        CHECK(nearVec3(surface.positions[3], 1, 1, 0));
        CHECK(nearVec3(surface.normals[0], 0.0f, 0.0f, 1.0f));
        CHECK(nearVec3(surface.normals[1], 0.0f, 0.0f, 1.0f));
        CHECK(nearVec3(surface.normals[2], 0.0f, 0.0f, 1.0f));
        CHECK(nearVec3(surface.normals[3], 0.0f, 0.0f, 0.0f));
    }
}

void checkSmoothing()
{
    const char* text = "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\n"
                       "f 1 2 3\nf 1 3 4\n";
    ObjImporter importer;
    ModelData model;
    CHECK(importer.importMemory(reinterpret_cast<const std::uint8_t*>(text), strlen(text),
            pathOf("shared.obj"), model));
    CHECK(model.materials.size() == 1);
    CHECK(model.materials.size() == 1 && model.materials[0].name == "default");
    CHECK(model.materials.size() == 1 && near(model.materials[0].baseColor.x, 0.75f));
    CHECK(model.meshes.size() == 1 && model.meshes[0].primitives.size() == 1);
    if (model.meshes.size() != 1 || model.meshes[0].primitives.size() != 1) return;
    CHECK(model.meshes[0].primitives[0].vertexCount == 4);
    CHECK(model.meshes[0].primitives[0].indexCount == 6);
    SurfaceData surface;
    CHECK(importer.readPrimitive(model, model.meshes[0].primitives[0], surface));
    if (surface.normals.size() != 4) return;
    const float inv = 1.0f / sqrtf(2.0f);
    CHECK(nearVec3(surface.normals[0], inv, 0.0f, inv));
    CHECK(nearVec3(surface.normals[1], 0.0f, 0.0f, 1.0f));
    CHECK(nearVec3(surface.normals[2], inv, 0.0f, inv));
    CHECK(nearVec3(surface.normals[3], 1.0f, 0.0f, 0.0f));
}

void checkPolygon()
{
    const char* text = "v 0 0 0\nv 1 0 0\nv 2 1 0\nv 1 2 0\nv 0 1 0\n"
                       "f 1 2 3 4 5\n";
    ObjImporter importer;
    ModelData model;
    CHECK(importer.importMemory(reinterpret_cast<const std::uint8_t*>(text), strlen(text),
            pathOf("pentagon.obj"), model));
    CHECK(model.meshes.size() == 1 && model.meshes[0].primitives.size() == 1);
    if (model.meshes.size() != 1 || model.meshes[0].primitives.size() != 1) return;
    SurfaceData surface;
    CHECK(importer.readPrimitive(model, model.meshes[0].primitives[0], surface));
    CHECK(surface.positions.size() == 5);
    const std::uint32_t expected[9] = { 0, 1, 2, 0, 2, 3, 0, 3, 4 };
    CHECK(surface.indices.size() == 9);
    if (surface.indices.size() != 9) return;
    for (std::size_t i = 0; i < 9; ++i) CHECK(surface.indices[i] == expected[i]);
    for (std::size_t i = 0; i < 5; ++i) CHECK(nearVec3(surface.normals[i], 0.0f, 0.0f, 1.0f));
}

void checkRejected(ObjImporter& importer, const char* text, const char* name)
{
    ModelData model;
    CHECK(!importer.importMemory(reinterpret_cast<const std::uint8_t*>(text), strlen(text),
            pathOf(name), model));
    CHECK(!model.error.empty());
}

} // namespace

int main()
{
    CHECK(fs_create_directory(kDirectory));
    CHECK(writeText("cube.obj", kCube));
    CHECK(writeText("cube.mtl", kCubeMaterials));
    CHECK(writeText("leaves.obj", kLeaves));
    CHECK(writeText("leaves.mtl", kLeavesMaterials));

    ObjImporter importer;
    CHECK(strcmp(importer.name(), "Wavefront OBJ") == 0);
    CHECK(importer.supports("obj"));
    CHECK(!importer.supports("mtl"));
    CHECK(!importer.supports("3ds"));
    CHECK(!importer.supports(nullptr));

    ModelData model;
    CHECK(importer.import(pathOf("cube.obj"), model));
    CHECK(model.path == pathOf("cube.obj"));
    CHECK(model.directory == pathOf(""));
    checkCube(importer, model, "cube");

    ModelData memoryModel;
    CHECK(importer.importMemory(reinterpret_cast<const std::uint8_t*>(kCube), strlen(kCube),
            pathOf("memory.obj"), memoryModel));
    checkCube(importer, memoryModel, "memory");

    ModelLoader loader;
    loader.addImporter(new ObjImporter());
    CHECK(loader.importerFor(pathOf("cube.obj")) != nullptr);
    CHECK(loader.importerFor(pathOf("cube.OBJ")) != nullptr);
    ModelData loaded;
    CHECK(loader.load(pathOf("cube.obj"), loaded));
    checkCube(*loader.importerFor(pathOf("cube.obj")), loaded, "cube");
    ModelData loadedMemory;
    CHECK(loader.loadFromMemory(reinterpret_cast<const std::uint8_t*>(kCube), strlen(kCube),
            pathOf("loader.obj"), loadedMemory));
    checkCube(*loader.importer(0), loadedMemory, "loader");

    ModelData leaves;
    CHECK(importer.import(pathOf("leaves.obj"), leaves));
    checkLeaves(importer, leaves);

    ModelData withoutMaterials;
    CHECK(importer.importMemory(reinterpret_cast<const std::uint8_t*>(kCube), strlen(kCube),
            "nowhere/cube.obj", withoutMaterials));
    CHECK(withoutMaterials.error.empty());
    CHECK(withoutMaterials.materials.size() == 2);
    CHECK(withoutMaterials.meshes.size() == 1 && withoutMaterials.meshes[0].primitives.size() == 2);
    CHECK(withoutMaterials.images.empty());
    CHECK(withoutMaterials.textures.empty());
    if (withoutMaterials.materials.size() == 2)
    {
        CHECK(withoutMaterials.materials[0].name == "red");
        CHECK(withoutMaterials.materials[1].name == "blue");
        CHECK(near(withoutMaterials.materials[0].baseColor.x, 0.75f));
        CHECK(near(withoutMaterials.materials[0].baseColor.y, 0.75f));
        CHECK(near(withoutMaterials.materials[1].baseColor.z, 0.75f));
        CHECK(withoutMaterials.materials[0].baseColorTexture == -1);
    }

    checkSmoothing();
    checkPolygon();

    checkRejected(importer, "v 0 0 0\nv 1 0 0\nv 0 1 0\n", "nofaces.obj");
    checkRejected(importer, "# nothing\n", "comment.obj");
    checkRejected(importer, "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2\nf 1\n", "degenerate.obj");
    checkRejected(importer, "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 4\n", "missingvertex.obj");
    checkRejected(importer, "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 -4\n", "negativevertex.obj");
    checkRejected(importer, "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 0\n", "zerovertex.obj");
    checkRejected(importer, "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 x\n", "garbage.obj");
    checkRejected(importer, "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1/1 2/1 3/1\n", "missinguv.obj");
    checkRejected(importer, "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1//1 2//1 3//1\n", "missingnormal.obj");
    checkRejected(importer, "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1/ 2/ 3/x\n", "slashes.obj");

    ModelData failed;
    CHECK(!importer.importMemory(nullptr, 0, pathOf("empty.obj"), failed));
    CHECK(!failed.error.empty());
    CHECK(!importer.import(pathOf("absent.obj"), failed));
    CHECK(!failed.error.empty());
    CHECK(writeText("nofaces.obj", "v 0 0 0\nv 1 0 0\nv 0 1 0\n"));
    CHECK(!importer.import(pathOf("nofaces.obj"), failed));
    CHECK(!failed.error.empty());

    const std::size_t length = strlen(kCube);
    for (std::size_t cut = 1; cut < length; ++cut)
    {
        ModelData partial;
        const bool ok = importer.importMemory(reinterpret_cast<const std::uint8_t*>(kCube), cut,
                pathOf("partial.obj"), partial);
        CHECK(ok == partial.error.empty());
        if (ok) CHECK(!partial.meshes.empty() && partial.primitiveCount > 0);
    }

    const char* hugeNumbers = "v 1e99999 -1e99999 99999999999999999999999999999999999999999999999\n"
                              "v 1 0 0\nv 0 1 0\nf 99999999999999999999 2 3\n";
    checkRejected(importer, hugeNumbers, "huge.obj");

    const char* noTrailingNewline = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3";
    ModelData tail;
    CHECK(importer.importMemory(reinterpret_cast<const std::uint8_t*>(noTrailingNewline),
            strlen(noTrailingNewline), pathOf("tail.obj"), tail));
    CHECK(tail.primitiveCount == 1);

    if (failures == 0) printf("obj: ok\n");
    return failures == 0 ? 0 : 1;
}
