#include "prisma3d/editor/LightmapUnwrapper.h"

#include "xatlas.h"

#include "platform.h"

#include <math.h>

namespace prisma3d
{

namespace
{

const std::uint32_t kInvalid = 0xFFFFFFFFu;

struct ProgressContext
{
    LightmapUnwrapSettings::ProgressCallback callback = nullptr;
    void* userData = nullptr;
    bool cancelled = false;
};

bool xatlasProgress(xatlas::ProgressCategory category, int progress, void* userData)
{
    ProgressContext* context = static_cast<ProgressContext*>(userData);
    if (!context || !context->callback) return true;
    const int clamped = progress < 0 ? 0 : (progress > 100 ? 100 : progress);
    const bool keepGoing = context->callback(xatlas::StringForEnum(category),
            static_cast<std::uint32_t>(clamped), context->userData);
    if (!keepGoing) context->cancelled = true;
    return keepGoing;
}

bool fail(ct::String* error, const char* message)
{
    log_error("LightmapUnwrapper: %s", message);
    if (error) *error = message;
    return false;
}

bool sameCount(const ct::Vector<SurfaceData>& surfaces, bool normals)
{
    for (std::size_t s = 0; s < surfaces.size(); ++s)
    {
        const std::size_t count = surfaces[s].positions.size();
        const std::size_t have = normals ? surfaces[s].normals.size() : surfaces[s].uvs.size();
        if (have != count) return false;
    }
    return true;
}

} // namespace

bool LightmapUnwrapper::unwrap(const ct::Vector<SurfaceData>& input,
        ct::Vector<SurfaceData>& output, const LightmapUnwrapSettings& settings,
        LightmapUnwrapResult* atlasResult, ct::String* error) const
{
    output.clear();
    if (atlasResult) *atlasResult = LightmapUnwrapResult();
    if (error) error->clear();

    if (input.empty()) return fail(error, "the mesh has no surfaces");
    std::size_t vertexTotal = 0;
    std::size_t indexTotal = 0;
    for (std::size_t s = 0; s < input.size(); ++s)
    {
        const SurfaceData& surface = input[s];
        if (surface.topology != prisma::Topology::Triangles)
            return fail(error, "only triangle surfaces can be unwrapped");
        if (!surface.joints.empty() || !surface.weights.empty())
            return fail(error, "skinned meshes are not supported");
        if (surface.positions.empty() || surface.indices.empty())
            return fail(error, "a surface is empty");
        if (surface.indices.size() % 3 != 0)
            return fail(error, "the index count is not a multiple of three");
        for (std::size_t i = 0; i < surface.indices.size(); ++i)
            if (surface.indices[i] >= surface.positions.size())
                return fail(error, "an index is out of range");
        vertexTotal += surface.positions.size();
        indexTotal += surface.indices.size();
    }

    const bool hasNormals = sameCount(input, true);
    const bool hasUvs = sameCount(input, false);

    ct::Vector<Math::Vec3> positions;
    ct::Vector<Math::Vec3> normals;
    ct::Vector<Math::Vec2> uvs;
    ct::Vector<std::uint32_t> indices;
    ct::Vector<std::uint32_t> vertexBase;
    ct::Vector<std::uint32_t> indexBase;
    positions.reserve(vertexTotal);
    indices.reserve(indexTotal);
    if (hasNormals) normals.reserve(vertexTotal);
    if (hasUvs) uvs.reserve(vertexTotal);
    for (std::size_t s = 0; s < input.size(); ++s)
    {
        const SurfaceData& surface = input[s];
        vertexBase.push_back(static_cast<std::uint32_t>(positions.size()));
        indexBase.push_back(static_cast<std::uint32_t>(indices.size()));
        for (std::size_t i = 0; i < surface.positions.size(); ++i)
        {
            const Math::Vec3& p = surface.positions[i];
            positions.push_back(Math::Vec3(p.x * settings.scale.x, p.y * settings.scale.y,
                    p.z * settings.scale.z));
            if (hasNormals) normals.push_back(surface.normals[i]);
            if (hasUvs) uvs.push_back(surface.uvs[i]);
        }
        for (std::size_t i = 0; i < surface.indices.size(); ++i)
            indices.push_back(surface.indices[i] + vertexBase.back());
    }
    vertexBase.push_back(static_cast<std::uint32_t>(positions.size()));
    indexBase.push_back(static_cast<std::uint32_t>(indices.size()));

    xatlas::Atlas* atlas = xatlas::Create();
    if (!atlas) return fail(error, "xatlas could not create an atlas");

    ProgressContext progress;
    progress.callback = settings.progress;
    progress.userData = settings.progressUserData;
    if (settings.progress) xatlas::SetProgressCallback(atlas, xatlasProgress, &progress);

    xatlas::MeshDecl decl;
    decl.vertexCount = static_cast<std::uint32_t>(positions.size());
    decl.vertexPositionData = positions.data();
    decl.vertexPositionStride = sizeof(Math::Vec3);
    if (hasNormals)
    {
        decl.vertexNormalData = normals.data();
        decl.vertexNormalStride = sizeof(Math::Vec3);
    }
    if (hasUvs)
    {
        decl.vertexUvData = uvs.data();
        decl.vertexUvStride = sizeof(Math::Vec2);
    }
    decl.indexCount = static_cast<std::uint32_t>(indices.size());
    decl.indexData = indices.data();
    decl.indexFormat = xatlas::IndexFormat::UInt32;

    const xatlas::AddMeshError added = xatlas::AddMesh(atlas, decl);
    if (added != xatlas::AddMeshError::Success)
    {
        ct::String message =
                ct::String("xatlas could not take the mesh (") + xatlas::StringForEnum(added) + ")";
        xatlas::Destroy(atlas);
        return fail(error, message.c_str());
    }

    xatlas::ChartOptions chartOptions;
    chartOptions.useInputMeshUvs = hasUvs;
    chartOptions.fixWinding = true;

    xatlas::PackOptions packOptions;
    packOptions.resolution = settings.resolution;
    packOptions.padding = settings.padding;
    packOptions.texelsPerUnit = settings.texelsPerUnit;
    packOptions.bilinear = true;
    packOptions.blockAlign = true;

    xatlas::Generate(atlas, chartOptions, packOptions);

    if (progress.cancelled)
    {
        xatlas::Destroy(atlas);
        if (error) *error = "cancelled";
        return false;
    }
    if (atlas->meshCount != 1 || atlas->width == 0 || atlas->height == 0 || atlas->atlasCount == 0)
    {
        xatlas::Destroy(atlas);
        return fail(error, "xatlas produced an unusable atlas");
    }
    if (atlas->atlasCount != 1)
    {
        ct::String message = ct::String("the atlas needs ");
        message.append_number(static_cast<unsigned long long>(atlas->atlasCount));
        message += " pages: lower the texels per unit or raise the resolution until it fits one";
        xatlas::Destroy(atlas);
        return fail(error, message.c_str());
    }

    const xatlas::Mesh& result = atlas->meshes[0];
    if (result.indexCount != indices.size())
    {
        xatlas::Destroy(atlas);
        return fail(error, "xatlas changed the triangle count");
    }

    const float width = static_cast<float>(atlas->width);
    const float height = static_cast<float>(atlas->height);
    ct::Vector<std::uint32_t> remap;
    remap.resize(result.vertexCount, kInvalid);
    ct::Vector<std::uint32_t> touched;

    output.resize(input.size());
    for (std::size_t s = 0; s < input.size(); ++s)
    {
        const SurfaceData& source = input[s];
        SurfaceData& target = output[s];
        target.topology = source.topology;
        const std::uint32_t first = indexBase[s];
        const std::uint32_t last = indexBase[s + 1];
        const std::uint32_t low = vertexBase[s];
        const std::uint32_t high = vertexBase[s + 1];
        touched.clear();
        for (std::uint32_t i = first; i < last; ++i)
        {
            const std::uint32_t outIndex = result.indexArray[i];
            if (outIndex >= result.vertexCount)
            {
                xatlas::Destroy(atlas);
                output.clear();
                return fail(error, "xatlas returned an out-of-range index");
            }
            if (remap[outIndex] == kInvalid)
            {
                const xatlas::Vertex& vertex = result.vertexArray[outIndex];
                if (vertex.xref < low || vertex.xref >= high)
                {
                    xatlas::Destroy(atlas);
                    output.clear();
                    return fail(error, "xatlas mixed vertices of two surfaces");
                }
                const std::size_t local = vertex.xref - low;
                remap[outIndex] = static_cast<std::uint32_t>(target.positions.size());
                touched.push_back(outIndex);
                target.positions.push_back(source.positions[local]);
                if (!source.normals.empty()) target.normals.push_back(source.normals[local]);
                if (!source.tangents.empty()) target.tangents.push_back(source.tangents[local]);
                if (!source.colors.empty()) target.colors.push_back(source.colors[local]);
                if (!source.uvs.empty()) target.uvs.push_back(source.uvs[local]);
                target.uvs2.push_back(Math::Vec2(vertex.uv[0] / width, vertex.uv[1] / height));
            }
            target.indices.push_back(remap[outIndex]);
        }
        for (std::size_t i = 0; i < touched.size(); ++i) remap[touched[i]] = kInvalid;
    }

    log_info("LightmapUnwrapper: atlas %ux%u, %u charts, %zu surfaces", atlas->width, atlas->height,
            atlas->chartCount, input.size());
    if (atlasResult)
    {
        atlasResult->width = atlas->width;
        atlasResult->height = atlas->height;
        atlasResult->chartCount = atlas->chartCount;
    }
    xatlas::Destroy(atlas);
    return true;
}

float LightmapUnwrapper::worldSurfaceArea(const ct::Vector<SurfaceData>& surfaces,
        const Math::Vec3& scale)
{
    double area = 0.0;
    for (std::size_t s = 0; s < surfaces.size(); ++s)
    {
        const SurfaceData& surface = surfaces[s];
        for (std::size_t i = 0; i + 2 < surface.indices.size(); i += 3)
        {
            const std::uint32_t a = surface.indices[i];
            const std::uint32_t b = surface.indices[i + 1];
            const std::uint32_t c = surface.indices[i + 2];
            if (a >= surface.positions.size() || b >= surface.positions.size() ||
                    c >= surface.positions.size())
                continue;
            const Math::Vec3 p0 = surface.positions[a] * scale;
            const Math::Vec3 p1 = surface.positions[b] * scale;
            const Math::Vec3 p2 = surface.positions[c] * scale;
            area += static_cast<double>((p1 - p0).Cross(p2 - p0).Length()) * 0.5;
        }
    }
    return static_cast<float>(area);
}

float LightmapUnwrapper::fittedTexelsPerUnit(float area, std::uint32_t atlasResolution)
{
    if (area <= 0.0f) return 0.0f;
    return static_cast<float>(atlasResolution) / sqrtf(area) * 0.8f;
}

} // namespace prisma3d
