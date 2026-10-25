#include "prisma3d/import/ObjImporter.h"

#include "prisma3d/core/FileSystem.h"

#include "platform.h"

#include <ct/hashmap.hpp>

#include <math.h>
#include <string.h>

namespace prisma3d
{

namespace
{

const double kMaxValue = 3.0e38;
const int kMaxExponent = 10000;
const int kMaxIndexValue = 1000000000;
const std::uint32_t kUnmapped = 0xFFFFFFFFu;

bool isSpace(char c)
{
    return c == ' ' || c == '\t' || c == '\r';
}

const char* skipSpaces(const char* p, const char* end)
{
    while (p < end && isSpace(*p)) ++p;
    return p;
}

const char* lineEnd(const char* p, const char* end)
{
    while (p < end && *p != '\n' && *p != '\r') ++p;
    return p;
}

const char* nextLine(const char* p, const char* end)
{
    while (p < end && *p != '\n') ++p;
    return p < end ? p + 1 : p;
}

bool startsWith(const char* p, const char* end, const char* word)
{
    while (*word)
    {
        if (p >= end || *p++ != *word++) return false;
    }
    return true;
}

bool keyword(const char* p, const char* end, const char* word, std::size_t length)
{
    return startsWith(p, end, word) && p + length < end && isSpace(p[length]);
}

float parseFloat(const char*& p, const char* end)
{
    p = skipSpaces(p, end);
    bool negative = false;
    if (p < end && (*p == '-' || *p == '+'))
    {
        negative = *p == '-';
        ++p;
    }

    double value = 0.0;
    while (p < end && *p >= '0' && *p <= '9')
    {
        if (value < 1.0e300) value = value * 10.0 + static_cast<double>(*p - '0');
        ++p;
    }
    if (p < end && *p == '.')
    {
        ++p;
        double fraction = 0.1;
        while (p < end && *p >= '0' && *p <= '9')
        {
            value += static_cast<double>(*p++ - '0') * fraction;
            fraction *= 0.1;
        }
    }
    if (p < end && (*p == 'e' || *p == 'E'))
    {
        ++p;
        bool exponentNegative = false;
        if (p < end && (*p == '-' || *p == '+'))
        {
            exponentNegative = *p == '-';
            ++p;
        }
        int exponent = 0;
        while (p < end && *p >= '0' && *p <= '9')
        {
            if (exponent < kMaxExponent) exponent = exponent * 10 + (*p - '0');
            ++p;
        }
        value *= pow(10.0, exponentNegative ? -exponent : exponent);
    }
    if (value != value) value = 0.0;
    if (value > kMaxValue) value = kMaxValue;
    return static_cast<float>(negative ? -value : value);
}

int parseInt(const char*& p, const char* end)
{
    p = skipSpaces(p, end);
    bool negative = false;
    if (p < end && (*p == '-' || *p == '+'))
    {
        negative = *p == '-';
        ++p;
    }
    int value = 0;
    while (p < end && *p >= '0' && *p <= '9')
    {
        if (value < kMaxIndexValue) value = value * 10 + (*p - '0');
        ++p;
    }
    return negative ? -value : value;
}

ct::String lineValue(const char* p, const char* end)
{
    p = skipSpaces(p, end);
    const char* last = lineEnd(p, end);
    while (last > p && isSpace(last[-1])) --last;
    return ct::String(p, static_cast<std::size_t>(last - p));
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

struct RawMaterial
{
    ct::String name;
    Math::Vec3 diffuse = Math::Vec3(0.75f, 0.75f, 0.75f);
    ct::String albedo;
    ct::String normalMap;
    bool cutout = false;
};

void parseMtl(const ct::String& filename, ct::Vector<RawMaterial>& materials)
{
    const ct::Vector<std::uint8_t> bytes = FileSystem::getSingleton().readBinary(filename);
    if (bytes.empty())
    {
        log_warn("ObjImporter: material file '%s' was not found", filename.c_str());
        return;
    }

    const char* p = reinterpret_cast<const char*>(bytes.data());
    const char* end = p + bytes.size();
    RawMaterial* current = nullptr;
    while (p < end)
    {
        p = skipSpaces(p, end);
        if (keyword(p, end, "newmtl", 6))
        {
            materials.push_back(RawMaterial());
            current = &materials[materials.size() - 1];
            current->name = lineValue(p + 6, end);
        }
        else if (current && keyword(p, end, "Kd", 2))
        {
            p += 2;
            current->diffuse.x = parseFloat(p, end);
            current->diffuse.y = parseFloat(p, end);
            current->diffuse.z = parseFloat(p, end);
        }
        else if (current && keyword(p, end, "map_Kd", 6))
            current->albedo = lineValue(p + 6, end);
        else if (current && (keyword(p, end, "map_bump", 8) || keyword(p, end, "map_Bump", 8)))
            current->normalMap = lineValue(p + 8, end);
        else if (current && keyword(p, end, "map_Disp", 8))
        {
            if (current->normalMap.empty()) current->normalMap = lineValue(p + 8, end);
        }
        else if (current && keyword(p, end, "bump", 4))
            current->normalMap = lineValue(p + 4, end);
        else if (current && keyword(p, end, "map_d", 5))
            current->cutout = true;
        p = nextLine(p, end);
    }
}

struct VertexKey
{
    int position = -1;
    int uv = -1;
    int normal = -1;

    bool operator==(const VertexKey& other) const
    {
        return position == other.position && uv == other.uv && normal == other.normal;
    }

    std::uint64_t hash() const
    {
        const ct::Hash<std::uint64_t> mix;
        std::uint64_t h = mix(static_cast<std::uint64_t>(position + 1));
        h = ct::hash_combine(h, mix(static_cast<std::uint64_t>(uv + 1)));
        return ct::hash_combine(h, mix(static_cast<std::uint64_t>(normal + 1)));
    }
};

struct Part
{
    std::uint32_t material = 0;
    ct::Vector<std::uint32_t> indices;
};

struct ParsedObj
{
    ct::Vector<Math::Vec3> sourcePositions;
    ct::Vector<Math::Vec3> sourceNormals;
    ct::Vector<Math::Vec2> sourceUvs;
    ct::Vector<RawMaterial> materials;
    ct::Vector<Math::Vec3> positions;
    ct::Vector<Math::Vec3> normals;
    ct::Vector<Math::Vec2> uvs;
    ct::Vector<std::uint8_t> missingNormals;
    ct::Vector<Part> parts;
};

int resolveIndex(int index, std::size_t count)
{
    if (index > 0) return index - 1;
    if (index < 0) return static_cast<int>(count) + index;
    return -1;
}

bool readOptionalIndex(const char*& cursor, const char* faceEnd, std::size_t count, int& resolved)
{
    const int raw = parseInt(cursor, faceEnd);
    if (raw == 0) return true;
    resolved = resolveIndex(raw, count);
    return resolved >= 0 && resolved < static_cast<int>(count);
}

bool parseFace(ParsedObj& obj, ct::HashMap<VertexKey, std::uint32_t>& vertices, Part& part,
        const char* cursor, const char* faceEnd, const char*& error)
{
    std::uint32_t first = 0;
    std::uint32_t previous = 0;
    std::uint32_t corners = 0;
    while ((cursor = skipSpaces(cursor, faceEnd)) < faceEnd)
    {
        const int position = resolveIndex(parseInt(cursor, faceEnd), obj.sourcePositions.size());
        int uv = -1;
        int normal = -1;
        if (position < 0 || position >= static_cast<int>(obj.sourcePositions.size()))
        {
            error = "face references a missing position";
            return false;
        }
        if (cursor < faceEnd && *cursor == '/')
        {
            ++cursor;
            if (cursor < faceEnd && *cursor != '/' &&
                    !readOptionalIndex(cursor, faceEnd, obj.sourceUvs.size(), uv))
            {
                error = "face references a missing texture coordinate";
                return false;
            }
            if (cursor < faceEnd && *cursor == '/')
            {
                ++cursor;
                if (!readOptionalIndex(cursor, faceEnd, obj.sourceNormals.size(), normal))
                {
                    error = "face references a missing normal";
                    return false;
                }
            }
        }

        VertexKey key;
        key.position = position;
        key.uv = uv;
        key.normal = normal;
        std::uint32_t vertex;
        const std::uint32_t* found = vertices.find(key);
        if (found) vertex = *found;
        else
        {
            vertex = static_cast<std::uint32_t>(obj.positions.size());
            vertices[key] = vertex;
            obj.positions.push_back(obj.sourcePositions[static_cast<std::size_t>(position)]);
            obj.normals.push_back(normal >= 0 ? obj.sourceNormals[static_cast<std::size_t>(normal)]
                                              : Math::Vec3(0.0f, 0.0f, 0.0f));
            obj.missingNormals.push_back(normal < 0 ? 1 : 0);
            obj.uvs.push_back(
                    uv >= 0 ? obj.sourceUvs[static_cast<std::size_t>(uv)] : Math::Vec2(0.0f, 0.0f));
        }

        if (corners == 0) first = vertex;
        else if (corners >= 2)
        {
            part.indices.push_back(first);
            part.indices.push_back(previous);
            part.indices.push_back(vertex);
        }
        previous = vertex;
        ++corners;
    }
    return true;
}

bool parseObj(const std::uint8_t* data, std::size_t size, const ct::String& directory,
        ParsedObj& obj, const char*& error)
{
    ct::HashMap<ct::String, std::uint32_t> materialIndices;
    ct::HashMap<VertexKey, std::uint32_t> vertices;
    ct::HashMap<std::uint32_t, std::size_t> mergedParts;
    std::uint32_t currentMaterial = 0;

    RawMaterial fallback;
    fallback.name = "default";
    obj.materials.push_back(fallback);
    materialIndices[fallback.name] = 0;

    const char* p = reinterpret_cast<const char*>(data);
    const char* end = p + size;
    while (p < end)
    {
        p = skipSpaces(p, end);
        if (p >= end) break;

        if (*p == 'v' && p + 1 < end && isSpace(p[1]))
        {
            ++p;
            const float x = parseFloat(p, end);
            const float y = parseFloat(p, end);
            const float z = parseFloat(p, end);
            obj.sourcePositions.push_back(Math::Vec3(x, y, z));
        }
        else if (keyword(p, end, "vn", 2))
        {
            p += 2;
            const float x = parseFloat(p, end);
            const float y = parseFloat(p, end);
            const float z = parseFloat(p, end);
            obj.sourceNormals.push_back(Math::Vec3(x, y, z));
        }
        else if (keyword(p, end, "vt", 2))
        {
            p += 2;
            const float x = parseFloat(p, end);
            const float y = parseFloat(p, end);
            obj.sourceUvs.push_back(Math::Vec2(x, 1.0f - y));
        }
        else if (keyword(p, end, "mtllib", 6))
        {
            const ct::String mtlName = lineValue(p + 6, end);
            const ct::String mtl = joinPath(directory, mtlName);
            const std::size_t first = obj.materials.size();
            if (!mtlName.empty()) parseMtl(mtl, obj.materials);
            for (std::size_t i = first; i < obj.materials.size(); ++i)
            {
                obj.materials[i].albedo = joinPath(directory, obj.materials[i].albedo);
                obj.materials[i].normalMap = joinPath(directory, obj.materials[i].normalMap);
                materialIndices[obj.materials[i].name] = static_cast<std::uint32_t>(i);
            }
        }
        else if (keyword(p, end, "usemtl", 6))
        {
            const ct::String name = lineValue(p + 6, end);
            const std::uint32_t* found = materialIndices.find(name);
            if (!found)
            {
                RawMaterial material;
                material.name = name;
                currentMaterial = static_cast<std::uint32_t>(obj.materials.size());
                obj.materials.push_back(material);
                materialIndices[name] = currentMaterial;
            }
            else
                currentMaterial = *found;
        }
        else if (*p == 'f' && p + 1 < end && isSpace(p[1]))
        {
            std::size_t partIndex;
            const std::size_t* merged = mergedParts.find(currentMaterial);
            if (merged) partIndex = *merged;
            else
            {
                partIndex = obj.parts.size();
                Part part;
                part.material = currentMaterial;
                obj.parts.push_back(part);
                mergedParts[currentMaterial] = partIndex;
            }
            if (!parseFace(obj, vertices, obj.parts[partIndex], p + 1, lineEnd(p + 1, end), error))
                return false;
        }
        p = nextLine(p, end);
    }

    bool any = false;
    for (std::size_t i = 0; i < obj.parts.size(); ++i)
    {
        if (!obj.parts[i].indices.empty()) any = true;
    }
    if (!any)
    {
        error = "OBJ has no faces";
        return false;
    }

    for (std::size_t i = 0; i < obj.parts.size(); ++i)
    {
        const ct::Vector<std::uint32_t>& indices = obj.parts[i].indices;
        for (std::size_t k = 0; k + 2 < indices.size(); k += 3)
        {
            const std::uint32_t a = indices[k];
            const std::uint32_t b = indices[k + 1];
            const std::uint32_t c = indices[k + 2];
            const Math::Vec3 face = Math::Vec3::Cross(obj.positions[b] - obj.positions[a],
                    obj.positions[c] - obj.positions[a]);
            if (obj.missingNormals[a]) obj.normals[a] += face;
            if (obj.missingNormals[b]) obj.normals[b] += face;
            if (obj.missingNormals[c]) obj.normals[c] += face;
        }
    }
    for (std::size_t i = 0; i < obj.normals.size(); ++i)
    {
        if (obj.missingNormals[i] && Math::Vec3::Dot(obj.normals[i], obj.normals[i]) > 0.0f)
            obj.normals[i] = obj.normals[i].Normalized();
    }
    return true;
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

struct ObjSource : ModelSource
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

bool build(const std::uint8_t* data, std::size_t size, const ct::String& path, ModelData& out)
{
    out = ModelData();
    out.path = path;
    out.directory = directoryOf(path);

    const char* error = "";
    ParsedObj obj;
    if (!parseObj(data, size, out.directory, obj, error)) return fail(out, error);

    bool defaultUsed = false;
    for (std::size_t i = 0; i < obj.parts.size(); ++i)
    {
        if (!obj.parts[i].indices.empty() && obj.parts[i].material == 0) defaultUsed = true;
    }

    ct::Vector<int> materialMap;
    materialMap.resize(obj.materials.size(), -1);
    for (std::size_t i = defaultUsed ? 0 : 1; i < obj.materials.size(); ++i)
    {
        const RawMaterial& raw = obj.materials[i];
        ModelMaterialData material;
        material.name = raw.name;
        material.baseColor = Math::Vec4(raw.diffuse.x, raw.diffuse.y, raw.diffuse.z, 1.0f);
        material.metallic = 0.0f;
        material.roughness = 1.0f;
        if (raw.cutout)
        {
            material.alphaMode = ModelAlphaMode::Mask;
            material.doubleSided = true;
        }
        if (!raw.albedo.empty()) material.baseColorTexture = addTexture(out, raw.albedo);
        if (!raw.normalMap.empty()) material.normalTexture = addTexture(out, raw.normalMap);
        out.materials.push_back(static_cast<ModelMaterialData&&>(material));
        materialMap[i] = static_cast<int>(out.materials.size() - 1);
    }

    ct::Unique<ObjSource> owned = ct::make_unique<ObjSource>();
    const ct::String baseName = baseNameOf(path);
    const bool hasUv = !obj.sourceUvs.empty();

    ct::Vector<std::uint32_t> remap;
    remap.resize(obj.positions.size(), kUnmapped);
    ct::Vector<std::uint32_t> touched;

    ModelMeshData mesh;
    mesh.name = baseName;
    for (std::size_t i = 0; i < obj.parts.size(); ++i)
    {
        const Part& part = obj.parts[i];
        if (part.indices.empty()) continue;

        SurfaceData surface;
        surface.topology = prisma::Topology::Triangles;
        touched.clear();
        for (std::size_t k = 0; k < part.indices.size(); ++k)
        {
            const std::uint32_t global = part.indices[k];
            if (remap[global] == kUnmapped)
            {
                remap[global] = static_cast<std::uint32_t>(surface.positions.size());
                touched.push_back(global);
                surface.positions.push_back(obj.positions[global]);
                surface.normals.push_back(obj.normals[global]);
                if (hasUv) surface.uvs.push_back(obj.uvs[global]);
            }
            surface.indices.push_back(remap[global]);
        }
        for (std::size_t k = 0; k < touched.size(); ++k) remap[touched[k]] = kUnmapped;

        ModelPrimitiveData primitive;
        primitive.material = materialMap[part.material];
        primitive.attributeMask = kVertexPosition | kVertexNormal;
        if (hasUv) primitive.attributeMask |= kVertexUv;
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
    out.meshes.push_back(static_cast<ModelMeshData&&>(mesh));

    ModelNodeData node;
    node.name = baseName;
    node.mesh = 0;
    out.nodes.push_back(static_cast<ModelNodeData&&>(node));
    out.roots.push_back(0);

    out.source = ct::Unique<ModelSource>(static_cast<ct::Unique<ObjSource>&&>(owned));
    return true;
}

} // namespace

const char* ObjImporter::name() const
{
    return "Wavefront OBJ";
}

bool ObjImporter::supports(const char* extension) const
{
    return extension && strcmp(extension, "obj") == 0;
}

bool ObjImporter::import(const ct::String& path, ModelData& out)
{
    out = ModelData();
    const ct::Vector<std::uint8_t> bytes = FileSystem::getSingleton().readBinary(path);
    if (bytes.empty()) return fail(out, "cannot read file");
    return build(bytes.data(), bytes.size(), path, out);
}

bool ObjImporter::importMemory(const std::uint8_t* data, std::size_t size,
        const ct::String& virtualName, ModelData& out)
{
    out = ModelData();
    if (!data || size == 0) return fail(out, "empty OBJ data");
    return build(data, size, virtualName, out);
}

bool ObjImporter::readPrimitive(const ModelData& model, const ModelPrimitiveData& primitive,
        SurfaceData& out) const
{
    const ObjSource* source = static_cast<const ObjSource*>(model.source.get());
    if (!source || primitive.source >= source->surfaces.size()) return false;
    out = source->surfaces[primitive.source];
    return true;
}

bool ObjImporter::readImage(const ModelData&, const ModelImageData&,
        ct::Vector<std::uint8_t>& out) const
{
    out.clear();
    return false;
}

} // namespace prisma3d
