#include "Check.h"

#include "prisma3d/import/ModelImporter.h"

#include <math.h>
#include <string.h>

using namespace prisma3d;

namespace
{

int sDestroyed = 0;

class FakeImporter : public ModelImporter
{
public:
    FakeImporter(const char* tag, const char* extension)
            : mTag(tag),
              mExtension(extension),
              mImports(0)
    {
    }

    ~FakeImporter() override
    {
        ++sDestroyed;
    }

    const char* name() const override
    {
        return "fake";
    }

    bool supports(const char* extension) const override
    {
        return strcmp(extension, mExtension) == 0;
    }

    bool import(const ct::String& path, ModelData& out) override
    {
        ++mImports;
        out = ModelData();
        out.path = path;
        fill(out);
        return true;
    }

    bool importMemory(const std::uint8_t* data, std::size_t size, const ct::String& virtualName,
            ModelData& out) override
    {
        (void) data;
        ++mImports;
        out = ModelData();
        out.path = virtualName;
        out.geometryBytes = size;
        fill(out);
        return true;
    }

    bool readPrimitive(const ModelData& model, const ModelPrimitiveData& primitive,
            SurfaceData& out) const override
    {
        (void) model;
        (void) primitive;
        (void) out;
        return false;
    }

    bool readImage(const ModelData& model, const ModelImageData& image,
            ct::Vector<std::uint8_t>& out) const override
    {
        (void) model;
        (void) image;
        (void) out;
        return false;
    }

    int imports() const
    {
        return mImports;
    }

private:
    void fill(ModelData& out) const
    {
        out.nodes.resize(2);
        out.nodes[0].name = mTag;
        out.nodes[0].children.push_back(1);
        out.nodes[1].name = "child";
        out.nodes[1].parent = 0;
        out.roots.push_back(0);
    }

    const char* mTag;
    const char* mExtension;
    int mImports;
};

class FailingImporter : public ModelImporter
{
public:
    ~FailingImporter() override
    {
        ++sDestroyed;
    }

    const char* name() const override
    {
        return "failing";
    }

    bool supports(const char* extension) const override
    {
        return strcmp(extension, "bad") == 0;
    }

    bool import(const ct::String& path, ModelData& out) override
    {
        (void) path;
        out.error = "broken on purpose";
        return false;
    }

    bool importMemory(const std::uint8_t* data, std::size_t size, const ct::String& virtualName,
            ModelData& out) override
    {
        (void) data;
        (void) size;
        return import(virtualName, out);
    }

    bool readPrimitive(const ModelData& model, const ModelPrimitiveData& primitive,
            SurfaceData& out) const override
    {
        (void) model;
        (void) primitive;
        (void) out;
        return false;
    }

    bool readImage(const ModelData& model, const ModelImageData& image,
            ct::Vector<std::uint8_t>& out) const override
    {
        (void) model;
        (void) image;
        (void) out;
        return false;
    }
};

bool near(float a, float b)
{
    return fabsf(a - b) < 1e-4f;
}

bool nearVec3(const Math::Vec3& v, float x, float y, float z)
{
    return near(v.x, x) && near(v.y, y) && near(v.z, z);
}

void test_extension()
{
    CHECK(ModelLoader::extensionOf("a/b.GLB") == "glb");
    CHECK(ModelLoader::extensionOf("model.gltf") == "gltf");
    CHECK(ModelLoader::extensionOf("a.b/c.Obj") == "obj");
    CHECK(ModelLoader::extensionOf("a\\b.c\\d.PNG") == "png");
    CHECK(ModelLoader::extensionOf("a/b").empty());
    CHECK(ModelLoader::extensionOf("model").empty());
    CHECK(ModelLoader::extensionOf("a.b/c").empty());
    CHECK(ModelLoader::extensionOf("a.b\\c").empty());
    CHECK(ModelLoader::extensionOf("model.").empty());
    CHECK(ModelLoader::extensionOf("").empty());
}

void test_registry()
{
    sDestroyed = 0;
    {
        ModelLoader loader;
        CHECK(loader.importerCount() == 0);
        CHECK(loader.importer(0) == nullptr);
        loader.addImporter(nullptr);
        CHECK(loader.importerCount() == 0);

        FakeImporter* first = new FakeImporter("first", "fake");
        FakeImporter* second = new FakeImporter("second", "fake");
        FakeImporter* other = new FakeImporter("other", "other");
        FailingImporter* failing = new FailingImporter();
        loader.addImporter(first);
        loader.addImporter(second);
        loader.addImporter(other);
        loader.addImporter(failing);
        CHECK(loader.importerCount() == 4);
        CHECK(loader.importer(0) == first);
        CHECK(loader.importer(3) == failing);
        CHECK(loader.importer(4) == nullptr);

        CHECK(loader.importerFor("dir/model.fake") == first);
        CHECK(loader.importerFor("dir/model.FAKE") == first);
        CHECK(loader.importerFor("dir/model.other") == other);
        CHECK(loader.importerFor("dir/model.bad") == failing);
        CHECK(loader.importerFor("dir/model.unknown") == nullptr);
        CHECK(loader.importerFor("dir/model") == nullptr);
        CHECK(loader.importerFor("") == nullptr);

        ModelData model;
        CHECK(loader.load("dir/model.fake", model));
        CHECK(model.path == "dir/model.fake");
        CHECK(model.nodes.size() == 2);
        if (model.nodes.size() == 2)
        {
            CHECK(model.nodes[0].name == "first");
            CHECK(model.nodes[1].parent == 0);
        }
        CHECK(first->imports() == 1);
        CHECK(second->imports() == 0);

        ModelData otherModel;
        CHECK(loader.load("x/y.other", otherModel));
        CHECK(otherModel.nodes.size() == 2);
        if (otherModel.nodes.size() == 2) CHECK(otherModel.nodes[0].name == "other");
        CHECK(other->imports() == 1);

        ModelData untouched;
        untouched.nodes.resize(1);
        untouched.nodes[0].name = "kept";
        CHECK(!loader.load("dir/model.unknown", untouched));
        CHECK(!untouched.error.empty());
        CHECK(untouched.nodes.size() == 1 && untouched.nodes[0].name == "kept");

        ModelData noExtension;
        CHECK(!loader.load("dir/model", noExtension));
        CHECK(!noExtension.error.empty());

        ModelData broken;
        broken.nodes.resize(1);
        CHECK(!loader.load("dir/model.bad", broken));
        CHECK(broken.error == "broken on purpose");
        CHECK(broken.nodes.size() == 1);

        const std::uint8_t bytes[5] = { 1, 2, 3, 4, 5 };
        ModelData memory;
        CHECK(loader.loadFromMemory(bytes, sizeof(bytes), "virtual/thing.Fake", memory));
        CHECK(memory.path == "virtual/thing.Fake");
        CHECK(memory.geometryBytes == sizeof(bytes));
        CHECK(memory.nodes.size() == 2);
        if (memory.nodes.size() == 2) CHECK(memory.nodes[0].name == "first");
        CHECK(first->imports() == 2);
        CHECK(second->imports() == 0);

        ModelData memoryFailure;
        CHECK(!loader.loadFromMemory(bytes, sizeof(bytes), "thing.unknown", memoryFailure));
        CHECK(!memoryFailure.error.empty());
        CHECK(!loader.loadFromMemory(bytes, sizeof(bytes), "thing", memoryFailure));
        CHECK(!memoryFailure.error.empty());
        memoryFailure.error.clear();
        CHECK(!loader.loadFromMemory(nullptr, 0, "thing.fake", memoryFailure));
        CHECK(!memoryFailure.error.empty());
        memoryFailure.error.clear();
        CHECK(!loader.loadFromMemory(bytes, sizeof(bytes), "thing.bad", memoryFailure));
        CHECK(memoryFailure.error == "broken on purpose");

        CHECK(sDestroyed == 0);
    }
    CHECK(sDestroyed == 4);
}

void test_singleton()
{
    CHECK(ModelLoader::singleton() == nullptr);
    {
        ModelLoader a;
        CHECK(ModelLoader::singleton() == &a);
        {
            ModelLoader b;
            CHECK(ModelLoader::singleton() == &b);
        }
        CHECK(ModelLoader::singleton() == nullptr);
    }
    CHECK(ModelLoader::singleton() == nullptr);
}

void test_transforms()
{
    ModelData model;
    model.nodes.resize(3);
    model.nodes[0].translation = Math::Vec3(1.0f, 2.0f, 3.0f);
    model.nodes[0].scale = Math::Vec3(2.0f, 2.0f, 2.0f);
    model.nodes[0].children.push_back(1);
    model.nodes[1].parent = 0;
    model.nodes[1].translation = Math::Vec3(1.0f, 0.0f, 0.0f);
    model.nodes[1].children.push_back(2);
    model.nodes[2].parent = 1;
    model.nodes[2].translation = Math::Vec3(0.0f, 1.0f, 0.0f);

    CHECK(nearVec3(model.nodeLocalTransform(0).GetTranslation(), 1.0f, 2.0f, 3.0f));
    CHECK(nearVec3(model.nodeLocalTransform(1).GetTranslation(), 1.0f, 0.0f, 0.0f));
    CHECK(nearVec3(model.nodeGlobalTransform(0).GetTranslation(), 1.0f, 2.0f, 3.0f));
    CHECK(nearVec3(model.nodeGlobalTransform(1).GetTranslation(), 3.0f, 2.0f, 3.0f));
    CHECK(nearVec3(model.nodeGlobalTransform(2).GetTranslation(), 3.0f, 4.0f, 3.0f));
    const Math::Vec3 point =
            model.nodeGlobalTransform(1).TransformPoint(Math::Vec3(1.0f, 1.0f, 1.0f));
    CHECK(nearVec3(point, 5.0f, 4.0f, 5.0f));
}

} // namespace

int main()
{
    test_extension();
    test_registry();
    test_singleton();
    test_transforms();

    if (failures) printf("%d check(s) failed\n", failures);
    else
        printf("all checks passed\n");
    return failures != 0;
}
