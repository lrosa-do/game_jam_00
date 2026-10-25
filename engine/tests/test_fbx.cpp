#include "Check.h"

#include "prisma3d/import/FbxImporter.h"

#include "platform.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const float kTolerance = 1e-3f;
const char* const kDirectory = PRISMA3D_TEST_DIR "/fbx";

bool near(float a, float b)
{
    return fabsf(a - b) <= kTolerance;
}

bool nearVec3(const Math::Vec3& a, float x, float y, float z)
{
    return near(a.x, x) && near(a.y, y) && near(a.z, z);
}

ct::String pathOf(const char* name)
{
    return ct::String(kDirectory) + "/" + name;
}

bool writeText(const char* name, const ct::String& text)
{
    fs_create_directory(kDirectory);
    return file_write(pathOf(name).c_str(), text.c_str(), text.size());
}

const char* const kIdentity = "1.0,0.0,0.0,0.0, 0.0,1.0,0.0,0.0, 0.0,0.0,1.0,0.0, 0.0,0.0,0.0,1.0";
const char* const kChildLink = "1.0,0.0,0.0,0.0, 0.0,1.0,0.0,0.0, 0.0,0.0,1.0,0.0, 0.0,1.0,0.0,1.0";

ct::String geometryText()
{
    return "\tGeometry: 100, \"Geometry::quad\", \"Mesh\" {\n"
           "\t\tVertices: *12 {\n"
           "\t\t\ta: -1.0,0.0,-1.0, 1.0,0.0,-1.0, 1.0,0.0,1.0, -1.0,0.0,1.0\n"
           "\t\t}\n"
           "\t\tPolygonVertexIndex: *6 {\n"
           "\t\t\ta: 0,1,-3, 0,2,-4\n"
           "\t\t}\n"
           "\t\tLayerElementNormal: 0 {\n"
           "\t\t\tVersion: 101\n"
           "\t\t\tName: \"\"\n"
           "\t\t\tMappingInformationType: \"ByPolygonVertex\"\n"
           "\t\t\tReferenceInformationType: \"Direct\"\n"
           "\t\t\tNormals: *18 {\n"
           "\t\t\t\ta: 0.0,1.0,0.0, 0.0,1.0,0.0, 0.0,1.0,0.0, 0.0,1.0,0.0, 0.0,1.0,0.0, "
           "0.0,1.0,0.0\n"
           "\t\t\t}\n"
           "\t\t}\n"
           "\t\tLayerElementUV: 0 {\n"
           "\t\t\tVersion: 101\n"
           "\t\t\tName: \"map1\"\n"
           "\t\t\tMappingInformationType: \"ByPolygonVertex\"\n"
           "\t\t\tReferenceInformationType: \"IndexToDirect\"\n"
           "\t\t\tUV: *8 {\n"
           "\t\t\t\ta: 0.0,0.0, 1.0,0.0, 1.0,1.0, 0.0,1.0\n"
           "\t\t\t}\n"
           "\t\t\tUVIndex: *6 {\n"
           "\t\t\t\ta: 0,1,2, 0,2,3\n"
           "\t\t\t}\n"
           "\t\t}\n"
           "\t\tLayerElementMaterial: 0 {\n"
           "\t\t\tVersion: 101\n"
           "\t\t\tName: \"\"\n"
           "\t\t\tMappingInformationType: \"ByPolygon\"\n"
           "\t\t\tReferenceInformationType: \"IndexToDirect\"\n"
           "\t\t\tMaterials: *2 {\n"
           "\t\t\t\ta: 0,1\n"
           "\t\t\t}\n"
           "\t\t}\n"
           "\t}\n";
}

ct::String modelAndMaterialsText()
{
    return "\tModel: 200, \"Model::quad\", \"Mesh\" {\n"
           "\t\tVersion: 232\n"
           "\t\tProperties70:  {\n"
           "\t\t\tP: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",2.0,0.0,0.0\n"
           "\t\t}\n"
           "\t}\n"
           "\tMaterial: 300, \"Material::red\", \"\" {\n"
           "\t\tVersion: 102\n"
           "\t\tShadingModel: \"phong\"\n"
           "\t\tProperties70:  {\n"
           "\t\t\tP: \"DiffuseColor\", \"Color\", \"\", \"A\",1.0,0.0,0.0\n"
           "\t\t}\n"
           "\t}\n"
           "\tMaterial: 301, \"Material::blue\", \"\" {\n"
           "\t\tVersion: 102\n"
           "\t\tShadingModel: \"phong\"\n"
           "\t\tProperties70:  {\n"
           "\t\t\tP: \"DiffuseColor\", \"Color\", \"\", \"A\",0.0,0.0,1.0\n"
           "\t\t}\n"
           "\t}\n"
           "\tTexture: 350, \"Texture::albedo\", \"\" {\n"
           "\t\tType: \"TextureVideoClip\"\n"
           "\t\tVersion: 202\n"
           "\t\tFileName: \"C:\\\\somewhere\\\\albedo.png\"\n"
           "\t\tRelativeFilename: \"tex/albedo.png\"\n"
           "\t}\n";
}

ct::String skeletonText()
{
    ct::String text;
    text += "\tModel: 600, \"Model::root\", \"LimbNode\" {\n"
            "\t\tVersion: 232\n"
            "\t\tProperties70:  {\n"
            "\t\t\tP: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",0.0,0.0,0.0\n"
            "\t\t}\n"
            "\t}\n"
            "\tModel: 601, \"Model::child\", \"LimbNode\" {\n"
            "\t\tVersion: 232\n"
            "\t\tProperties70:  {\n"
            "\t\t\tP: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",0.0,1.0,0.0\n"
            "\t\t}\n"
            "\t}\n"
            "\tDeformer: 500, \"Deformer::skin\", \"Skin\" {\n"
            "\t\tVersion: 101\n"
            "\t}\n"
            "\tDeformer: 510, \"SubDeformer::root\", \"Cluster\" {\n"
            "\t\tVersion: 100\n"
            "\t\tIndexes: *2 {\n"
            "\t\t\ta: 0,1\n"
            "\t\t}\n"
            "\t\tWeights: *2 {\n"
            "\t\t\ta: 1.0,1.0\n"
            "\t\t}\n"
            "\t\tTransform: *16 {\n"
            "\t\t\ta: ";
    text += kIdentity;
    text += "\n\t\t}\n"
            "\t\tTransformLink: *16 {\n"
            "\t\t\ta: ";
    text += kIdentity;
    text += "\n\t\t}\n"
            "\t}\n"
            "\tDeformer: 511, \"SubDeformer::child\", \"Cluster\" {\n"
            "\t\tVersion: 100\n"
            "\t\tIndexes: *2 {\n"
            "\t\t\ta: 2,3\n"
            "\t\t}\n"
            "\t\tWeights: *2 {\n"
            "\t\t\ta: 1.0,1.0\n"
            "\t\t}\n"
            "\t\tTransform: *16 {\n"
            "\t\t\ta: ";
    text += kIdentity;
    text += "\n\t\t}\n"
            "\t\tTransformLink: *16 {\n"
            "\t\t\ta: ";
    text += kChildLink;
    text += "\n\t\t}\n"
            "\t}\n"
            "\tAnimationStack: 700, \"AnimStack::walk\", \"\" {\n"
            "\t\tProperties70:  {\n"
            "\t\t\tP: \"LocalStop\", \"KTime\", \"Time\", \"\",46186158000\n"
            "\t\t}\n"
            "\t}\n"
            "\tAnimationLayer: 710, \"AnimLayer::base\", \"\" {\n"
            "\t}\n"
            "\tAnimationCurveNode: 720, \"AnimCurveNode::R\", \"\" {\n"
            "\t\tProperties70:  {\n"
            "\t\t\tP: \"d|X\", \"Number\", \"\", \"A\",0.0\n"
            "\t\t\tP: \"d|Y\", \"Number\", \"\", \"A\",0.0\n"
            "\t\t\tP: \"d|Z\", \"Number\", \"\", \"A\",0.0\n"
            "\t\t}\n"
            "\t}\n"
            "\tAnimationCurve: 730, \"AnimCurve::\", \"\" {\n"
            "\t\tDefault: 0.0\n"
            "\t\tKeyVer: 4008\n"
            "\t\tKeyTime: *2 {\n"
            "\t\t\ta: 0,46186158000\n"
            "\t\t}\n"
            "\t\tKeyValueFloat: *2 {\n"
            "\t\t\ta: 0.0,90.0\n"
            "\t\t}\n"
            "\t\tKeyAttrFlags: *1 {\n"
            "\t\t\ta: 24840\n"
            "\t\t}\n"
            "\t}\n";
    return text;
}

ct::String fbxText(bool skinned)
{
    ct::String text;
    text += "; FBX 7.4.0 project file\n"
            "FBXHeaderExtension:  {\n"
            "\tFBXHeaderVersion: 1003\n"
            "\tFBXVersion: 7400\n"
            "}\n"
            "GlobalSettings:  {\n"
            "\tVersion: 1000\n"
            "\tProperties70:  {\n"
            "\t\tP: \"UpAxis\", \"int\", \"Integer\", \"\",1\n"
            "\t}\n"
            "}\n"
            "Objects:  {\n";
    text += geometryText();
    text += modelAndMaterialsText();
    if (skinned) text += skeletonText();
    text += "}\n"
            "Connections:  {\n"
            "\tC: \"OO\",200,0\n"
            "\tC: \"OO\",100,200\n"
            "\tC: \"OO\",300,200\n"
            "\tC: \"OO\",301,200\n"
            "\tC: \"OP\",350,300, \"DiffuseColor\"\n";
    if (skinned)
        text += "\tC: \"OO\",600,0\n"
                "\tC: \"OO\",601,600\n"
                "\tC: \"OO\",500,100\n"
                "\tC: \"OO\",510,500\n"
                "\tC: \"OO\",511,500\n"
                "\tC: \"OO\",600,510\n"
                "\tC: \"OO\",601,511\n"
                "\tC: \"OO\",710,700\n"
                "\tC: \"OO\",720,710\n"
                "\tC: \"OP\",720,601, \"Lcl Rotation\"\n"
                "\tC: \"OP\",730,720, \"d|Z\"\n";
    text += "}\n";
    return text;
}

void checkGeometry(const FbxImporter& importer, const ModelData& model, bool skinned)
{
    CHECK(model.meshes.size() == 1);
    CHECK(model.nodes.size() == 1);
    CHECK(model.roots.size() == 1);
    if (model.meshes.size() != 1 || model.nodes.size() != 1) return;
    CHECK(model.meshes[0].name == "quad");
    CHECK(model.nodes[0].mesh == 0);
    CHECK(model.nodes[0].skeleton == (skinned ? 0 : -1));
    CHECK(model.meshes[0].primitives.size() == 2);
    CHECK(model.primitiveCount == 2);
    if (model.meshes[0].primitives.size() != 2) return;

    const float shift = skinned ? 0.0f : 2.0f;
    for (std::size_t p = 0; p < 2; ++p)
    {
        const ModelPrimitiveData& primitive = model.meshes[0].primitives[p];
        std::uint32_t mask = kVertexPosition | kVertexNormal | kVertexUv;
        if (skinned) mask |= kVertexJoints | kVertexWeights;
        CHECK(primitive.attributeMask == mask);
        CHECK(primitive.material == static_cast<int>(p));
        CHECK(primitive.vertexCount == 3);
        CHECK(primitive.indexCount == 3);
        CHECK(primitive.skinned == skinned);
        CHECK(primitive.hasAabb);
        SurfaceData surface;
        CHECK(importer.readPrimitive(model, primitive, surface));
        CHECK(surface.positions.size() == 3 && surface.indices.size() == 3);
        CHECK(surface.normals.size() == 3 && surface.uvs.size() == 3);
        CHECK(surface.tangents.empty() && surface.colors.empty());
        if (surface.positions.size() != 3) continue;
        CHECK(nearVec3(surface.positions[0], -1.0f + shift, 0.0f, -1.0f));
        CHECK(nearVec3(surface.normals[0], 0.0f, 1.0f, 0.0f));
        CHECK(near(surface.uvs[0].x, 0.0f) && near(surface.uvs[0].y, 1.0f));
        if (p == 0)
        {
            CHECK(nearVec3(surface.positions[1], 1.0f + shift, 0.0f, -1.0f));
            CHECK(nearVec3(surface.positions[2], 1.0f + shift, 0.0f, 1.0f));
            CHECK(near(surface.uvs[2].x, 1.0f) && near(surface.uvs[2].y, 0.0f));
        }
        else
        {
            CHECK(nearVec3(surface.positions[1], 1.0f + shift, 0.0f, 1.0f));
            CHECK(nearVec3(surface.positions[2], -1.0f + shift, 0.0f, 1.0f));
        }
        CHECK(surface.indices[0] == 0 && surface.indices[1] == 1 && surface.indices[2] == 2);
        if (skinned)
        {
            CHECK(surface.joints.size() == 3 && surface.weights.size() == 3);
            if (surface.joints.size() != 3) continue;
            CHECK(surface.joints[0] == 0);
            CHECK(near(surface.weights[0].x, 1.0f) && near(surface.weights[0].y, 0.0f));
            const std::uint32_t last = surface.joints[p == 0 ? 2 : 1];
            CHECK((last & 0xffu) == 1u && ((last >> 8) & 0xffu) == 1u);
        }
        else
            CHECK(surface.joints.empty() && surface.weights.empty());
    }

    CHECK(model.materials.size() == 2);
    if (model.materials.size() == 2)
    {
        CHECK(model.materials[0].name == "red");
        CHECK(near(model.materials[0].baseColor.x, 1.0f) &&
                near(model.materials[0].baseColor.z, 0.0f));
        CHECK(model.materials[0].metallic == 0.0f);
        CHECK(model.materials[0].baseColorTexture == 0);
        CHECK(model.materials[0].normalTexture == -1);
        CHECK(model.materials[1].name == "blue");
        CHECK(near(model.materials[1].baseColor.z, 1.0f));
        CHECK(model.materials[1].baseColorTexture == -1);
    }
    CHECK(model.textures.size() == 1 && model.images.size() == 1);
    if (model.images.size() == 1)
    {
        CHECK(model.images[0].name == "albedo.png");
        CHECK(!model.images[0].embedded());
        CHECK(model.images[0].path == model.directory + "tex/albedo.png");
        ct::Vector<std::uint8_t> bytes;
        CHECK(!importer.readImage(model, model.images[0], bytes));
    }
}

void checkSkeleton(const ModelData& model)
{
    CHECK(model.skeletons.size() == 1);
    if (model.skeletons.size() != 1) return;
    const Skeleton& skeleton = model.skeletons[0];
    CHECK(skeleton.boneCount() == 2);
    CHECK(skeleton.finalized());
    if (skeleton.boneCount() != 2) return;
    CHECK(skeleton.bone(0).name == "root");
    CHECK(skeleton.bone(0).parent == -1);
    CHECK(skeleton.bone(1).name == "child");
    CHECK(skeleton.bone(1).parent == 0);
    CHECK(nearVec3(Math::Vec3(skeleton.bone(1).bindLocal.col3.x, skeleton.bone(1).bindLocal.col3.y,
                           skeleton.bone(1).bindLocal.col3.z),
            0.0f, 1.0f, 0.0f));
    CHECK(near(skeleton.bone(1).inverseBind.col3.y, -1.0f));
    CHECK(skeleton.findBone("child") == 1);
}

void checkAnimation(const ModelData& model)
{
    CHECK(model.animations.size() == 1);
    if (model.animations.size() != 1) return;
    const AnimationClip& clip = model.animations[0];
    CHECK(clip.name() == "walk");
    CHECK(clip.skeleton() == 0);
    CHECK(near(clip.duration(), 1.0f));
    CHECK(clip.tracks().size() == 2);
    const BoneTrack* child = nullptr;
    for (std::size_t i = 0; i < clip.tracks().size(); ++i)
        if (clip.tracks()[i].bone == 1) child = &clip.tracks()[i];
    CHECK(child != nullptr);
    if (!child) return;
    CHECK(child->times.size() == 2);
    if (child->times.size() != 2) return;
    CHECK(near(child->times[0], 0.0f) && near(child->times[1], 1.0f));
    CHECK(nearVec3(child->positions[0], 0.0f, 1.0f, 0.0f));
    CHECK(nearVec3(child->positions[1], 0.0f, 1.0f, 0.0f));
    CHECK(near(child->rotations[0].w, 1.0f));
    const float half = sinf(45.0f * 3.14159265f / 180.0f);
    CHECK(near(child->rotations[1].z, half) && near(child->rotations[1].w, half));
    CHECK(near(child->rotations[1].x, 0.0f) && near(child->rotations[1].y, 0.0f));
    CHECK(nearVec3(child->scales[1], 1.0f, 1.0f, 1.0f));
}

void testFailures(FbxImporter& importer)
{
    ModelData model;
    const char* garbage = "this is not an fbx file at all {{{";
    CHECK(!importer.importMemory(reinterpret_cast<const std::uint8_t*>(garbage), strlen(garbage),
            "garbage.fbx", model));
    CHECK(!model.error.empty());

    const char* emptyScene = "; FBX 7.4.0 project file\nObjects:  {\n}\nConnections:  {\n}\n";
    CHECK(!importer.importMemory(reinterpret_cast<const std::uint8_t*>(emptyScene),
            strlen(emptyScene), "empty.fbx", model));
    CHECK(model.error == "no usable geometry");

    CHECK(!importer.importMemory(nullptr, 0, "none.fbx", model));
    CHECK(!importer.import(pathOf("missing.fbx"), model));
    CHECK(model.error == "cannot read file");
}

} // namespace

void testTextureSearch(FbxImporter& importer)
{
    ct::String text = fbxText(false);
    const ct::String relative = "RelativeFilename: \"tex/albedo.png\"";
    const std::size_t at = text.find(relative);
    CHECK(at != ct::String::npos);
    if (at == ct::String::npos) return;
    text = ct::String(text.c_str(), at) + "RelativeFilename: \"..\\\\sourceimages\\\\albedo.png\"" +
           ct::String(text.c_str() + at + relative.size());

    const ct::String root = ct::String(kDirectory) + "/pack";
    fs_remove_path((root + "/Models/albedo.png").c_str());
    fs_remove_path((root + "/Textures/albedo.png").c_str());
    fs_create_directory(root.c_str());
    fs_create_directory((root + "/Models").c_str());
    fs_create_directory((root + "/Textures").c_str());
    CHECK(file_write((root + "/Models/block.fbx").c_str(), text.c_str(), text.size()));

    ModelData missing;
    CHECK(importer.import(root + "/Models/block.fbx", missing));
    CHECK(missing.images.size() == 1);
    if (missing.images.size() == 1)
    {
        CHECK(missing.images[0].name == "albedo.png");
        CHECK(missing.images[0].path == root + "/sourceimages/albedo.png");
    }

    const char png[] = "png";
    CHECK(file_write((root + "/Textures/albedo.png").c_str(), png, sizeof(png)));
    ModelData found;
    CHECK(importer.import(root + "/Models/block.fbx", found));
    CHECK(found.images.size() == 1);
    if (found.images.size() == 1) CHECK(found.images[0].path == root + "/Textures/albedo.png");

    CHECK(file_write((root + "/Models/albedo.png").c_str(), png, sizeof(png)));
    ModelData beside;
    CHECK(importer.import(root + "/Models/block.fbx", beside));
    if (beside.images.size() == 1) CHECK(beside.images[0].path == root + "/Models/albedo.png");
}

void testSecondUvSet(FbxImporter& importer)
{
    printf("a second UV set is kept as the lightmap UV\n");
    ct::String text = fbxText(false);
    const char* anchor = "\t\tLayerElementMaterial: 0 {\n";
    const std::size_t at = text.find(anchor);
    CHECK(at != ct::String::npos);
    if (at == ct::String::npos) return;
    ct::String second = "\t\tLayerElementUV: 1 {\n"
                        "\t\t\tVersion: 101\n"
                        "\t\t\tName: \"map2\"\n"
                        "\t\t\tMappingInformationType: \"ByPolygonVertex\"\n"
                        "\t\t\tReferenceInformationType: \"IndexToDirect\"\n"
                        "\t\t\tUV: *8 {\n"
                        "\t\t\t\ta: 0.25,0.25, 0.75,0.25, 0.75,0.75, 0.25,0.75\n"
                        "\t\t\t}\n"
                        "\t\t\tUVIndex: *6 {\n"
                        "\t\t\t\ta: 0,1,2, 0,2,3\n"
                        "\t\t\t}\n"
                        "\t\t}\n";
    text = text.substr(0, at) + second + text.substr(at);
    CHECK(writeText("quad_uv2.fbx", text));

    ModelData model;
    CHECK(importer.import(pathOf("quad_uv2.fbx"), model));
    CHECK(model.error.empty());
    CHECK(model.meshes.size() == 1 && model.meshes[0].primitives.size() == 2);
    if (model.meshes.size() != 1 || model.meshes[0].primitives.size() != 2) return;
    for (std::size_t p = 0; p < 2; ++p)
    {
        const ModelPrimitiveData& primitive = model.meshes[0].primitives[p];
        CHECK(primitive.attributeMask ==
                (kVertexPosition | kVertexNormal | kVertexUv | kVertexUv2));
        SurfaceData surface;
        CHECK(importer.readPrimitive(model, primitive, surface));
        CHECK(surface.uvs.size() == 3 && surface.uvs2.size() == 3);
        if (surface.uvs2.size() != 3) continue;
        CHECK(near(surface.uvs[0].x, 0.0f) && near(surface.uvs[0].y, 1.0f));
        CHECK(near(surface.uvs2[0].x, 0.25f) && near(surface.uvs2[0].y, 0.75f));
        if (p == 0)
            CHECK(near(surface.uvs2[1].x, 0.75f) && near(surface.uvs2[1].y, 0.75f) &&
                    near(surface.uvs2[2].x, 0.75f) && near(surface.uvs2[2].y, 0.25f));
    }
}

int main()
{
    FbxImporter importer;
    CHECK(strcmp(importer.name(), "FBX") == 0);
    CHECK(importer.supports("fbx"));
    CHECK(!importer.supports("obj"));
    CHECK(!importer.supports(nullptr));

    const ct::String staticText = fbxText(false);
    const ct::String skinnedText = fbxText(true);
    CHECK(writeText("quad.fbx", staticText));
    CHECK(writeText("hero.fbx", skinnedText));

    ModelData staticModel;
    CHECK(importer.import(pathOf("quad.fbx"), staticModel));
    CHECK(staticModel.error.empty());
    checkGeometry(importer, staticModel, false);
    CHECK(staticModel.skeletons.empty() && staticModel.animations.empty());

    ModelData hero;
    CHECK(importer.import(pathOf("hero.fbx"), hero));
    CHECK(hero.error.empty());
    checkGeometry(importer, hero, true);
    checkSkeleton(hero);
    testTextureSearch(importer);
    checkAnimation(hero);

    ModelData fromMemory;
    CHECK(importer.importMemory(reinterpret_cast<const std::uint8_t*>(skinnedText.c_str()),
            skinnedText.size(), "memory/hero.fbx", fromMemory));
    CHECK(fromMemory.directory == "memory/");
    checkSkeleton(fromMemory);
    checkAnimation(fromMemory);

    testSecondUvSet(importer);
    testFailures(importer);

    if (failures) printf("%d check(s) failed\n", failures);
    else
        printf("all checks passed\n");
    return failures ? 1 : 0;
}
