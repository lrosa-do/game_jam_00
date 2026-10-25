#include "prisma3d/import/C3DSImporter.h"

#include "prisma3d/core/FileSystem.h"

#include <ct/hashmap.hpp>

#include <string.h>

namespace prisma3d
{

namespace
{

const std::uint16_t kChunkMain = 0x4D4D;
const std::uint16_t kChunkEdit = 0x3D3D;
const std::uint16_t kChunkEditMaterial = 0xAFFF;
const std::uint16_t kChunkEditObject = 0x4000;
const std::uint16_t kChunkMatName = 0xA000;
const std::uint16_t kChunkMatDiffuse = 0xA020;
const std::uint16_t kChunkMatTexMap = 0xA200;
const std::uint16_t kChunkMatMapFile = 0xA300;
const std::uint16_t kChunkObjTriMesh = 0x4100;
const std::uint16_t kChunkTriVert = 0x4110;
const std::uint16_t kChunkTriFace = 0x4120;
const std::uint16_t kChunkTriFaceMat = 0x4130;
const std::uint16_t kChunkTriUv = 0x4140;
const std::uint16_t kChunkColRgb = 0x0010;
const std::uint16_t kChunkColTru = 0x0011;
const std::uint16_t kChunkColLin24 = 0x0012;
const std::uint16_t kChunkColLinF = 0x0013;

const std::size_t kChunkHeaderSize = 6;

bool finite(float value)
{
    return value - value == 0.0f;
}

struct Cursor
{
    const std::uint8_t* data = nullptr;
    std::size_t size = 0;
    std::size_t pos = 0;
    const char* error = nullptr;

    bool fail(const char* message)
    {
        if (!error) error = message;
        return false;
    }

    bool canRead(std::size_t count) const
    {
        return !error && pos <= size && size - pos >= count;
    }

    bool take(void* out, std::size_t count)
    {
        if (!canRead(count))
        {
            pos = size;
            return fail("data is truncated");
        }
        memcpy(out, data + pos, count);
        pos += count;
        return true;
    }

    std::uint8_t readU8()
    {
        std::uint8_t value = 0;
        take(&value, 1);
        return value;
    }

    std::uint16_t readU16()
    {
        std::uint8_t raw[2] = { 0, 0 };
        take(raw, 2);
        return static_cast<std::uint16_t>(raw[0] | (raw[1] << 8));
    }

    std::uint32_t readU32()
    {
        std::uint8_t raw[4] = { 0, 0, 0, 0 };
        take(raw, 4);
        return static_cast<std::uint32_t>(raw[0]) | (static_cast<std::uint32_t>(raw[1]) << 8) |
               (static_cast<std::uint32_t>(raw[2]) << 16) |
               (static_cast<std::uint32_t>(raw[3]) << 24);
    }

    float readF32()
    {
        const std::uint32_t raw = readU32();
        float value;
        memcpy(&value, &raw, sizeof(value));
        return value;
    }

    void seek(std::size_t target)
    {
        pos = target > size ? size : target;
    }
};

struct ChunkHeader
{
    std::uint16_t id = 0;
    std::uint32_t length = 0;
};

bool readChunkHeader(Cursor& r, std::size_t parentEnd, ChunkHeader& header, std::size_t& childEnd)
{
    if (r.error) return false;
    if (parentEnd > r.size || r.pos > parentEnd || parentEnd - r.pos < kChunkHeaderSize)
        return r.fail("chunk header is truncated");
    header.id = r.readU16();
    header.length = r.readU32();
    if (header.length < kChunkHeaderSize) return r.fail("chunk length is invalid");
    const std::size_t payload = static_cast<std::size_t>(header.length) - kChunkHeaderSize;
    if (payload > parentEnd - r.pos) return r.fail("chunk exceeds its parent");
    childEnd = r.pos + payload;
    return true;
}

bool hasChunk(const Cursor& r, std::size_t endPos)
{
    return !r.error && r.pos <= endPos && endPos - r.pos >= kChunkHeaderSize;
}

bool readCString(Cursor& r, std::size_t endPos, ct::String& value)
{
    value.clear();
    if (r.error) return false;
    if (endPos > r.size || r.pos > endPos) return r.fail("string is out of range");
    const std::uint8_t* begin = r.data + r.pos;
    const void* terminator = memchr(begin, 0, endPos - r.pos);
    if (!terminator) return r.fail("string is not terminated");
    const std::size_t length =
            static_cast<std::size_t>(static_cast<const std::uint8_t*>(terminator) - begin);
    value = ct::String(reinterpret_cast<const char*>(begin), length);
    r.pos += length + 1;
    return true;
}

struct Face3DS
{
    std::uint16_t a = 0;
    std::uint16_t b = 0;
    std::uint16_t c = 0;
};

struct FaceGroup3DS
{
    ct::String matName;
    ct::Vector<std::uint16_t> faces;
};

struct Object3DS
{
    ct::String name;
    ct::Vector<Math::Vec3> verts;
    ct::Vector<Math::Vec2> uvs;
    ct::Vector<Face3DS> faces;
    ct::Vector<FaceGroup3DS> groups;
};

struct Material3DS
{
    ct::String name;
    Math::Vec3 diffuse = Math::Vec3(1.0f, 1.0f, 1.0f);
    ct::String texFile;
};

bool readColorChunk(Cursor& r, std::size_t endPos, Math::Vec3& color)
{
    if (!hasChunk(r, endPos)) return true;
    ChunkHeader header;
    std::size_t childEnd = 0;
    if (!readChunkHeader(r, endPos, header, childEnd)) return false;
    if (header.id == kChunkColRgb || header.id == kChunkColLinF)
    {
        if (childEnd - r.pos < sizeof(float) * 3) return r.fail("colour chunk is truncated");
        Math::Vec3 value;
        value.x = r.readF32();
        value.y = r.readF32();
        value.z = r.readF32();
        if (!finite(value.x) || !finite(value.y) || !finite(value.z))
            return r.fail("colour is invalid");
        color = value;
    }
    else if (header.id == kChunkColTru || header.id == kChunkColLin24)
    {
        if (childEnd - r.pos < 3) return r.fail("colour chunk is truncated");
        const float red = static_cast<float>(r.readU8()) / 255.0f;
        const float green = static_cast<float>(r.readU8()) / 255.0f;
        const float blue = static_cast<float>(r.readU8()) / 255.0f;
        color = Math::Vec3(red, green, blue);
    }
    r.seek(childEnd);
    return !r.error;
}

bool readMaterialChunk(Cursor& r, std::size_t matEnd, ct::Vector<Material3DS>& out)
{
    Material3DS material;
    std::uint16_t texSection = 0;

    while (hasChunk(r, matEnd))
    {
        ChunkHeader header;
        std::size_t childEnd = 0;
        if (!readChunkHeader(r, matEnd, header, childEnd)) return false;

        switch (header.id)
        {
            case kChunkMatName:
                if (!readCString(r, childEnd, material.name)) return false;
                r.seek(childEnd);
                break;
            case kChunkMatDiffuse:
                if (!readColorChunk(r, childEnd, material.diffuse)) return false;
                r.seek(childEnd);
                break;
            case kChunkMatTexMap:
                texSection = header.id;
                break;
            case kChunkMatMapFile:
                if (texSection == kChunkMatTexMap && !readCString(r, childEnd, material.texFile))
                    return false;
                r.seek(childEnd);
                break;
            default:
                r.seek(childEnd);
                break;
        }
    }
    out.push_back(static_cast<Material3DS&&>(material));
    return !r.error;
}

bool readTriVert(Cursor& r, std::size_t endPos, Object3DS& obj)
{
    if (endPos - r.pos < sizeof(std::uint16_t)) return r.fail("vertex list is truncated");
    const std::size_t count = r.readU16();
    if (count > (endPos - r.pos) / (sizeof(float) * 3)) return r.fail("vertex list is truncated");
    obj.verts.resize(count);
    for (std::size_t i = 0; i < count; ++i)
    {
        Math::Vec3 v;
        v.x = r.readF32();
        v.y = r.readF32();
        v.z = r.readF32();
        if (!finite(v.x) || !finite(v.y) || !finite(v.z)) return r.fail("vertex is invalid");
        obj.verts[i] = v;
    }
    return !r.error;
}

bool readTriUv(Cursor& r, std::size_t endPos, Object3DS& obj)
{
    if (endPos - r.pos < sizeof(std::uint16_t)) return r.fail("uv list is truncated");
    const std::size_t count = r.readU16();
    if (count > (endPos - r.pos) / (sizeof(float) * 2)) return r.fail("uv list is truncated");
    obj.uvs.resize(count);
    for (std::size_t i = 0; i < count; ++i)
    {
        const float u = r.readF32();
        const float v = r.readF32();
        if (!finite(u) || !finite(v)) return r.fail("uv is invalid");
        obj.uvs[i] = Math::Vec2(u, 1.0f - v);
    }
    return !r.error;
}

bool readFaceGroup(Cursor& r, std::size_t endPos, Object3DS& obj)
{
    FaceGroup3DS group;
    if (!readCString(r, endPos, group.matName)) return false;
    if (endPos - r.pos < sizeof(std::uint16_t)) return r.fail("face group is truncated");
    const std::size_t count = r.readU16();
    if (count > (endPos - r.pos) / sizeof(std::uint16_t)) return r.fail("face group is truncated");
    group.faces.resize(count);
    for (std::size_t i = 0; i < count; ++i) group.faces[i] = r.readU16();
    obj.groups.push_back(static_cast<FaceGroup3DS&&>(group));
    return !r.error;
}

bool readTriFace(Cursor& r, std::size_t endPos, Object3DS& obj)
{
    if (endPos - r.pos < sizeof(std::uint16_t)) return r.fail("face list is truncated");
    const std::size_t count = r.readU16();
    if (count > (endPos - r.pos) / (sizeof(std::uint16_t) * 4))
        return r.fail("face list is truncated");
    obj.faces.resize(count);
    for (std::size_t i = 0; i < count; ++i)
    {
        Face3DS face;
        face.a = r.readU16();
        face.b = r.readU16();
        face.c = r.readU16();
        r.readU16();
        obj.faces[i] = face;
    }

    while (hasChunk(r, endPos))
    {
        ChunkHeader header;
        std::size_t childEnd = 0;
        if (!readChunkHeader(r, endPos, header, childEnd)) return false;
        if (header.id == kChunkTriFaceMat && !readFaceGroup(r, childEnd, obj)) return false;
        r.seek(childEnd);
    }
    return !r.error;
}

bool readTriMesh(Cursor& r, std::size_t endPos, Object3DS& obj)
{
    while (hasChunk(r, endPos))
    {
        ChunkHeader header;
        std::size_t childEnd = 0;
        if (!readChunkHeader(r, endPos, header, childEnd)) return false;

        bool ok = true;
        switch (header.id)
        {
            case kChunkTriVert:
                ok = readTriVert(r, childEnd, obj);
                break;
            case kChunkTriUv:
                ok = readTriUv(r, childEnd, obj);
                break;
            case kChunkTriFace:
                ok = readTriFace(r, childEnd, obj);
                break;
            default:
                break;
        }
        if (!ok) return false;
        r.seek(childEnd);
    }
    return !r.error;
}

bool readEditObject(Cursor& r, std::size_t endPos, ct::Vector<Object3DS>& objects)
{
    Object3DS obj;
    if (!readCString(r, endPos, obj.name)) return false;

    while (hasChunk(r, endPos))
    {
        ChunkHeader header;
        std::size_t childEnd = 0;
        if (!readChunkHeader(r, endPos, header, childEnd)) return false;
        if (header.id == kChunkObjTriMesh && !readTriMesh(r, childEnd, obj)) return false;
        r.seek(childEnd);
    }

    if (!obj.faces.empty()) objects.push_back(static_cast<Object3DS&&>(obj));
    return !r.error;
}

bool readEdit3DS(Cursor& r, std::size_t endPos, ct::Vector<Object3DS>& objects,
        ct::Vector<Material3DS>& materials)
{
    while (hasChunk(r, endPos))
    {
        ChunkHeader header;
        std::size_t childEnd = 0;
        if (!readChunkHeader(r, endPos, header, childEnd)) return false;

        if (header.id == kChunkEditMaterial)
        {
            if (!readMaterialChunk(r, childEnd, materials)) return false;
        }
        else if (header.id == kChunkEditObject)
        {
            if (!readEditObject(r, childEnd, objects)) return false;
        }
        r.seek(childEnd);
    }
    return !r.error;
}

bool parse3DS(const std::uint8_t* data, std::size_t size, ct::Vector<Object3DS>& objects,
        ct::Vector<Material3DS>& materials, const char*& error)
{
    Cursor r;
    r.data = data;
    r.size = size;

    const bool parsed = [&]()
    {
        ChunkHeader main;
        std::size_t mainEnd = 0;
        if (!readChunkHeader(r, r.size, main, mainEnd)) return false;
        if (main.id != kChunkMain) return r.fail("not a 3DS file");

        while (hasChunk(r, mainEnd))
        {
            ChunkHeader header;
            std::size_t childEnd = 0;
            if (!readChunkHeader(r, mainEnd, header, childEnd)) return false;
            if (header.id == kChunkEdit && !readEdit3DS(r, childEnd, objects, materials))
                return false;
            r.seek(childEnd);
        }
        if (r.error) return false;
        if (objects.empty()) return r.fail("3DS has no geometry");
        return true;
    }();

    if (!parsed)
    {
        error = r.error ? r.error : "3DS is invalid";
        return false;
    }
    return true;
}

ct::String directoryOf(const ct::String& path)
{
    std::size_t slash = ct::String::npos;
    for (std::size_t i = 0; i < path.size(); ++i)
    {
        if (path[i] == '/' || path[i] == '\\') slash = i;
    }
    if (slash == ct::String::npos) return ct::String();
    return ct::String(path.c_str(), slash + 1);
}

ct::String fileNameOf(const ct::String& path)
{
    std::size_t begin = 0;
    for (std::size_t i = 0; i < path.size(); ++i)
    {
        if (path[i] == '/' || path[i] == '\\') begin = i + 1;
    }
    return ct::String(path.c_str() + begin, path.size() - begin);
}

ct::String baseNameOf(const ct::String& path)
{
    const ct::String name = fileNameOf(path);
    std::size_t end = name.size();
    for (std::size_t i = name.size(); i > 0; --i)
    {
        if (name[i - 1] == '.')
        {
            end = i - 1;
            break;
        }
    }
    return ct::String(name.c_str(), end);
}

ct::String joinPath(const ct::String& directory, const ct::String& name)
{
    if (name.empty() || directory.empty()) return name;
    if (name[0] == '/' || name[0] == '\\' || (name.size() > 1 && name[1] == ':')) return name;
    return directory + name;
}

std::uint64_t surfaceBytes(const SurfaceData& surface)
{
    std::uint64_t total = 0;
    total += surface.positions.size() * sizeof(Math::Vec3);
    total += surface.normals.size() * sizeof(Math::Vec3);
    total += surface.uvs.size() * sizeof(Math::Vec2);
    total += surface.indices.size() * sizeof(std::uint32_t);
    return total;
}

struct C3DSSource : ModelSource
{
    ct::Vector<SurfaceData> surfaces;
};

bool fail(ModelData& out, const char* message)
{
    out.error = message;
    return false;
}

int addTexture(ModelData& out, const ct::String& path)
{
    for (std::size_t i = 0; i < out.images.size(); ++i)
    {
        if (out.images[i].path == path) return static_cast<int>(i);
    }
    ModelImageData image;
    image.name = fileNameOf(path);
    image.path = path;
    out.images.push_back(static_cast<ModelImageData&&>(image));
    ModelTextureData texture;
    texture.image = static_cast<int>(out.images.size() - 1);
    out.textures.push_back(texture);
    return static_cast<int>(out.textures.size() - 1);
}

struct MaterialSlots
{
    const ct::Vector<Material3DS>& materials;
    const ct::String& directory;
    ModelData& out;
    ct::HashMap<ct::String, int> slots;
    int defaultSlot = -1;

    MaterialSlots(const ct::Vector<Material3DS>& materials, const ct::String& directory,
            ModelData& out)
            : materials(materials),
              directory(directory),
              out(out)
    {
    }

    int slotFor(const ct::String& name)
    {
        if (name.empty())
        {
            if (defaultSlot < 0)
            {
                ModelMaterialData material;
                material.name = "default";
                material.metallic = 0.0f;
                material.roughness = 1.0f;
                out.materials.push_back(static_cast<ModelMaterialData&&>(material));
                defaultSlot = static_cast<int>(out.materials.size() - 1);
            }
            return defaultSlot;
        }
        const int* found = slots.find(name);
        if (found) return *found;

        ModelMaterialData material;
        material.name = name;
        material.metallic = 0.0f;
        material.roughness = 1.0f;
        for (std::size_t i = 0; i < materials.size(); ++i)
        {
            if (materials[i].name != name) continue;
            material.baseColor = Math::Vec4(materials[i].diffuse.x, materials[i].diffuse.y,
                    materials[i].diffuse.z, 1.0f);
            if (!materials[i].texFile.empty())
                material.baseColorTexture =
                        addTexture(out, joinPath(directory, materials[i].texFile));
            break;
        }
        out.materials.push_back(static_cast<ModelMaterialData&&>(material));
        const int slot = static_cast<int>(out.materials.size() - 1);
        slots[name] = slot;
        return slot;
    }
};

bool buildGroup(const Object3DS& obj, const FaceGroup3DS& group, SurfaceData& surface,
        const char*& error)
{
    ct::HashMap<std::uint16_t, std::uint32_t> remap;
    surface = SurfaceData();
    surface.topology = prisma::Topology::Triangles;
    const bool hasUv = !obj.uvs.empty();

    for (std::size_t f = 0; f < group.faces.size(); ++f)
    {
        const std::size_t faceIndex = group.faces[f];
        if (faceIndex >= obj.faces.size())
        {
            error = "face group references a missing face";
            return false;
        }
        const Face3DS& face = obj.faces[faceIndex];
        const std::uint16_t corners[3] = { face.a, face.b, face.c };
        for (std::size_t c = 0; c < 3; ++c)
        {
            const std::uint16_t vi = corners[c];
            if (vi >= obj.verts.size())
            {
                error = "face references a missing vertex";
                return false;
            }
            const std::uint32_t* found = remap.find(vi);
            std::uint32_t local;
            if (found) local = *found;
            else
            {
                local = static_cast<std::uint32_t>(surface.positions.size());
                remap[vi] = local;
                surface.positions.push_back(obj.verts[vi]);
                surface.normals.push_back(Math::Vec3(0.0f, 0.0f, 0.0f));
                if (hasUv)
                    surface.uvs.push_back(
                            vi < obj.uvs.size() ? obj.uvs[vi] : Math::Vec2(0.0f, 0.0f));
            }
            surface.indices.push_back(local);
        }
    }

    for (std::size_t i = 0; i + 2 < surface.indices.size(); i += 3)
    {
        const std::uint32_t a = surface.indices[i];
        const std::uint32_t b = surface.indices[i + 1];
        const std::uint32_t c = surface.indices[i + 2];
        const Math::Vec3 faceNormal = Math::Vec3::Cross(surface.positions[b] - surface.positions[a],
                surface.positions[c] - surface.positions[a]);
        surface.normals[a] += faceNormal;
        surface.normals[b] += faceNormal;
        surface.normals[c] += faceNormal;
    }
    for (std::size_t i = 0; i < surface.normals.size(); ++i)
    {
        Math::Vec3& normal = surface.normals[i];
        if (Math::Vec3::Dot(normal, normal) > 0.0f) normal = normal.Normalized();
        else
            normal = Math::Vec3(0.0f, 1.0f, 0.0f);
    }
    return true;
}

bool build(const std::uint8_t* data, std::size_t size, const ct::String& path, ModelData& out)
{
    out = ModelData();
    out.path = path;
    out.directory = directoryOf(path);

    const char* error = "";
    ct::Vector<Object3DS> objects;
    ct::Vector<Material3DS> materials;
    if (!parse3DS(data, size, objects, materials, error)) return fail(out, error);

    ct::Unique<C3DSSource> owned = ct::make_unique<C3DSSource>();
    MaterialSlots slots(materials, out.directory, out);

    ModelNodeData root;
    root.name = baseNameOf(path);
    out.nodes.push_back(static_cast<ModelNodeData&&>(root));
    out.roots.push_back(0);

    for (std::size_t o = 0; o < objects.size(); ++o)
    {
        const Object3DS& obj = objects[o];

        ct::Vector<FaceGroup3DS> fallback;
        const ct::Vector<FaceGroup3DS>* groups = &obj.groups;
        if (obj.groups.empty())
        {
            FaceGroup3DS all;
            all.faces.resize(obj.faces.size());
            for (std::size_t i = 0; i < obj.faces.size(); ++i)
                all.faces[i] = static_cast<std::uint16_t>(i);
            fallback.push_back(static_cast<FaceGroup3DS&&>(all));
            groups = &fallback;
        }

        ModelMeshData mesh;
        mesh.name = obj.name;
        for (std::size_t g = 0; g < groups->size(); ++g)
        {
            const FaceGroup3DS& group = (*groups)[g];
            if (group.faces.empty()) continue;

            SurfaceData surface;
            if (!buildGroup(obj, group, surface, error)) return fail(out, error);

            ModelPrimitiveData primitive;
            primitive.material = slots.slotFor(group.matName);
            primitive.attributeMask = kVertexPosition | kVertexNormal;
            if (!surface.uvs.empty()) primitive.attributeMask |= kVertexUv;
            primitive.vertexCount = static_cast<std::uint32_t>(surface.positions.size());
            primitive.indexCount = static_cast<std::uint32_t>(surface.indices.size());
            primitive.sourceBytes = surfaceBytes(surface);
            primitive.source = static_cast<std::uint32_t>(owned->surfaces.size());
            primitive.aabb = MeshStore::computeAabb(surface.positions);
            primitive.hasAabb = true;

            out.geometryBytes += primitive.sourceBytes;
            ++out.primitiveCount;
            mesh.primitives.push_back(primitive);
            owned->surfaces.push_back(static_cast<SurfaceData&&>(surface));
        }
        if (mesh.primitives.empty()) continue;

        out.meshes.push_back(static_cast<ModelMeshData&&>(mesh));
        ModelNodeData node;
        node.name = obj.name;
        node.parent = 0;
        node.mesh = static_cast<int>(out.meshes.size() - 1);
        out.nodes.push_back(static_cast<ModelNodeData&&>(node));
        out.nodes[0].children.push_back(static_cast<int>(out.nodes.size() - 1));
    }

    if (out.meshes.empty()) return fail(out, "3DS produced no triangles");

    out.source = ct::Unique<ModelSource>(static_cast<ct::Unique<C3DSSource>&&>(owned));
    return true;
}

} // namespace

const char* C3DSImporter::name() const
{
    return "3D Studio 3DS";
}

bool C3DSImporter::supports(const char* extension) const
{
    return extension && strcmp(extension, "3ds") == 0;
}

bool C3DSImporter::import(const ct::String& path, ModelData& out)
{
    out = ModelData();
    const ct::Vector<std::uint8_t> bytes = FileSystem::getSingleton().readBinary(path);
    if (bytes.empty()) return fail(out, "cannot read file");
    return build(bytes.data(), bytes.size(), path, out);
}

bool C3DSImporter::importMemory(const std::uint8_t* data, std::size_t size,
        const ct::String& virtualName, ModelData& out)
{
    out = ModelData();
    if (!data || size == 0) return fail(out, "empty 3DS data");
    return build(data, size, virtualName, out);
}

bool C3DSImporter::readPrimitive(const ModelData& model, const ModelPrimitiveData& primitive,
        SurfaceData& out) const
{
    const C3DSSource* source = static_cast<const C3DSSource*>(model.source.get());
    if (!source || primitive.source >= source->surfaces.size()) return false;
    out = source->surfaces[primitive.source];
    return true;
}

bool C3DSImporter::readImage(const ModelData&, const ModelImageData&,
        ct::Vector<std::uint8_t>& out) const
{
    out.clear();
    return false;
}

} // namespace prisma3d
