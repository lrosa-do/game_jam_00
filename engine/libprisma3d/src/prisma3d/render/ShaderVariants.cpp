#include "prisma3d/render/ShaderVariants.h"

#include <ct/vector.hpp>

namespace prisma3d
{

namespace
{

const char* const kSceneVertexFile = "scene_forward_clustered_vertex.glsl";
const char* const kSceneFragmentFile = "scene_forward_clustered_fragment.glsl";

const char* const kSceneBase =
        "#define MATERIAL_UNIFORMS_USED\n#define UV_USED\n#define COLOR_USED\n"
        "#define SPECULAR_OCCLUSION_DISABLED\n#define FEATURE_LIGHTS\n#define FEATURE_DFG\n"
        "#define FEATURE_REFLECTION_COLOR\n#define FEATURE_RADIANCE\n"
        "#define FEATURE_DIRECTIONAL_SHADOWS\n#define USE_RADIANCE_OCTMAP_ARRAY\n"
        "#define MAX_ROUGHNESS_LOD 7.0\n#define DIFFUSE_BURLEY\n#define DO_SIDE_CHECK\n";
const char* const kSceneUnshaded = "#define MODE_UNSHADED\n#define SHADOWS_DISABLED\n";
const char* const kSceneLit =
        "#define LIGHT_BACKLIGHT_USED\n#define FEATURE_CLUSTER\n"
        "#define subgroupBroadcastFirst(value) (value)\n#define subgroupMin(value) (value)\n"
        "#define subgroupMax(value) (value)\n#define subgroupOr(value) (value)\n";
const char* const kAlbedo = "#define MATERIAL_ALBEDO_TEXTURE\n";
const char* const kPacked =
        "#define MATERIAL_METALLIC_TEXTURE\n#define MATERIAL_ROUGHNESS_TEXTURE\n"
        "#define MATERIAL_AO_TEXTURE\n";
const char* const kEmission = "#define MATERIAL_EMISSION_TEXTURE\n";
const char* const kNormal = "#define NORMAL_MAP_USED\n#define TANGENT_USED\n";
const char* const kScissor = "#define ALPHA_SCISSOR_USED\n";
const char* const kDetail = "#define FEATURE_DETAIL\n#define UV2_USED\n";
const char* const kLightmap = "#define FEATURE_GI\n#define USE_LIGHTMAP\n";
const char* const kDirectionalShadows = "#define FEATURE_DIRECTIONAL_SHADOWS\n";
const char* const kShadowsDisabled = "#define SHADOWS_DISABLED\n";
const char* const kCoverage = "#define ALPHA_ANTIALIASING_EDGE_USED\n";
const char* const kDepthBase = "#define MODE_RENDER_DEPTH\n#define SHADOWS_DISABLED\n"
                               "#define DIFFUSE_BURLEY\n";
const char* const kDualParaboloid = "#define MODE_DUAL_PARABOLOID\n";

ct::String sceneDefines(unsigned key)
{
    ct::String defines(kSceneBase);
    const bool albedo = (key & 1u) != 0;
    const bool packed = ((key >> 1) & 1u) != 0;
    const bool emission = ((key >> 2) & 1u) != 0;
    const bool normal = ((key >> 3) & 1u) != 0;
    const bool scissor = ((key >> 4) & 1u) != 0;
    const bool unshaded = ((key >> 5) & 1u) != 0;
    const bool detail = ((key >> 6) & 1u) != 0;
    const bool lightmap = ((key >> 7) & 1u) != 0;
    defines += unshaded ? kSceneUnshaded : kSceneLit;
    if (albedo) defines += kAlbedo;
    if (packed) defines += kPacked;
    if (emission) defines += kEmission;
    if (normal) defines += kNormal;
    if (scissor) defines += kScissor;
    if (detail) defines += kDetail;
    if (lightmap) defines += kLightmap;
    return defines;
}

ct::String withoutDirectionalShadows(const ct::String& defines)
{
    const std::size_t at = defines.find(kDirectionalShadows);
    if (at == ct::String::npos) return defines;
    ct::String result = defines.substr(0, at);
    result += defines.substr(at + ct::String(kDirectionalShadows).size());
    return result;
}

ct::String multimeshDefines(unsigned flags)
{
    const unsigned color = flags & 1u;
    const unsigned custom = (flags >> 1) & 1u;
    ct::String extra("#define FEATURE_MULTIMESH\n#define SC_PACKED_1 ");
    extra.append_number(static_cast<unsigned long long>(1u + color * 4u + custom * 8u));
    extra += "u\n";
    return extra;
}

ct::String normalRoughnessDefines(unsigned key)
{
    ct::String defines("#define MATERIAL_UNIFORMS_USED\n#define UV_USED\n#define COLOR_USED\n"
                       "#define MODE_RENDER_DEPTH\n#define MODE_RENDER_NORMAL_ROUGHNESS\n"
                       "#define SHADOWS_DISABLED\n#define DIFFUSE_BURLEY\n");
    if (key & 1u) defines += kAlbedo;
    if ((key >> 1) & 1u) defines += "#define MATERIAL_ROUGHNESS_TEXTURE\n";
    if ((key >> 3) & 1u) defines += kNormal;
    if ((key >> 4) & 1u) defines += kScissor;
    if ((key >> 6) & 1u) defines += kDetail;
    return defines;
}

struct Registry
{
    ct::Vector<ShaderVariant*> owned;

    const ShaderVariant* sceneVertex[kSceneVariantCount];
    const ShaderVariant* sceneFragment[kSceneVariantCount];
    const ShaderVariant* sceneNoShadow[kSceneVariantCount];
    const ShaderVariant* sceneCoverage[kSceneVariantCount];
    const ShaderVariant* sceneCoverageNoShadow[kSceneVariantCount];
    const ShaderVariant* skyVertex;
    const ShaderVariant* skyFragment[kSkyVariantCount];
    const ShaderVariant* octmap[2];
    const ShaderVariant* shadowVertex;
    const ShaderVariant* shadowDualVertex;
    const ShaderVariant* terrainComposite;
    const ShaderVariant* clusterRender;
    const ShaderVariant* multimeshScene[3][4];
    const ShaderVariant* multimeshShadow[4];
    const ShaderVariant* multimeshShadowDual[4];
    const ShaderVariant* depthVertex[2][5];
    const ShaderVariant* depthFragment[2][2];
    const ShaderVariant* depthCoverage[2];
    const ShaderVariant* ssaoDownsample[6];
    const ShaderVariant* ssaoGather[3];
    const ShaderVariant* ssaoImportance[3];
    const ShaderVariant* ssaoBlur[3];
    const ShaderVariant* ssaoInterleave[3];
    const ShaderVariant* ssrCopy[3];
    const ShaderVariant* ssrDownsample[4];
    const ShaderVariant* ssrHiz[4];
    const ShaderVariant* ssrTrace[1];
    const ShaderVariant* ssrFilter[1];
    const ShaderVariant* fsrUpscale[1];
    const ShaderVariant* ssrResolve[1];
    const ShaderVariant* dofCompute[5];
    const ShaderVariant* dofRaster[7];
    const ShaderVariant* normalVertex[4];
    const ShaderVariant* normalMultimesh[2][4];
    const ShaderVariant* normalFragment[128];
    const ShaderVariant* billboard[3][3][5];
    const ShaderVariant* impostorVertex[4];
    const ShaderVariant* impostorFragment[128];
    const ShaderVariant* impostorCardVertex;
    const ShaderVariant* impostorCardFragment[3];
    const ShaderVariant* terrainVertex[3];
    const ShaderVariant* terrainShadowVertex[2];
    const ShaderVariant* terrainFragment[2];
    const ShaderVariant* ocean[2];
    const ShaderVariant* depthGrass[2];
    const ShaderVariant* depthLeafWind[5];
    const ShaderVariant* singles[38];

    const ShaderVariant* make(const char* file, prisma::ShaderStage stage,
            const ct::String& defines)
    {
        ShaderVariant* variant = new ShaderVariant();
        variant->file = file;
        variant->stage = stage;
        variant->defines = defines;
        owned.push_back(variant);
        return variant;
    }

    const ShaderVariant* vertex(const ct::String& defines)
    {
        return make(kSceneVertexFile, prisma::ShaderStage::Vertex, defines);
    }

    const ShaderVariant* fragment(const ct::String& defines)
    {
        return make(kSceneFragmentFile, prisma::ShaderStage::Fragment, defines);
    }

    const ShaderVariant* compute(const char* file, const ct::String& defines)
    {
        return make(file, prisma::ShaderStage::Compute, defines);
    }

    Registry()
    {
        buildScene();
        buildSky();
        buildOctmap();
        buildShadow();
        buildMultimesh();
        buildDepth();
        buildScreenSpace();
        buildSsr();
        buildFsr();
        buildDof();
        buildNormalRoughness();
        buildBillboard();
        buildImpostor();
        buildTerrain();
        buildOcean();
        buildGrassAndLeafDepth();
        struct Single
        {
            const char* file;
            prisma::ShaderStage stage;
        };
        static const Single kSingles[38] = {
            { "gui.vert", prisma::ShaderStage::Vertex },
            { "gui.frag", prisma::ShaderStage::Fragment },
            { "skeleton.comp", prisma::ShaderStage::Compute },
            { "impostor_prepare.comp", prisma::ShaderStage::Compute },
            { "integrate_dfg.comp", prisma::ShaderStage::Compute },
            { "octmap.vert", prisma::ShaderStage::Vertex },
            { "octmap_roughness.frag", prisma::ShaderStage::Fragment },
            { "shadow.frag", prisma::ShaderStage::Fragment },
            { "shadow_dp.frag", prisma::ShaderStage::Fragment },
            { "cube_to_dp.frag", prisma::ShaderStage::Fragment },
            { "batch.vert", prisma::ShaderStage::Vertex },
            { "batch.frag", prisma::ShaderStage::Fragment },
            { "tonemap.vert", prisma::ShaderStage::Vertex },
            { "tonemap.frag", prisma::ShaderStage::Fragment },
            { "glow_blur.frag", prisma::ShaderStage::Fragment },
            { "depth_read.frag", prisma::ShaderStage::Fragment },
            { "atlas_copy.frag", prisma::ShaderStage::Fragment },
            { "lensflare.vert", prisma::ShaderStage::Vertex },
            { "lensflare.frag", prisma::ShaderStage::Fragment },
            { "hair_simulate.comp", prisma::ShaderStage::Compute },
            { "hair_wind.comp", prisma::ShaderStage::Compute },
            { "screen_copy.frag", prisma::ShaderStage::Fragment },
            { "ocean_simulate.comp", prisma::ShaderStage::Compute },
            { "ocean_fft.comp", prisma::ShaderStage::Compute },
            { "ocean_fft_v2.comp", prisma::ShaderStage::Compute },
            { "ocean_displacement.comp", prisma::ShaderStage::Compute },
            { "ocean_gradient.comp", prisma::ShaderStage::Compute },
            { "best_fit_normal.comp", prisma::ShaderStage::Compute },
            { "cluster_render.vert", prisma::ShaderStage::Vertex },
            { "cluster_store.comp", prisma::ShaderStage::Compute },
            { "cluster_clear.comp", prisma::ShaderStage::Compute },
            { "upscale.frag", prisma::ShaderStage::Fragment },
            { "smaa_edge_detection.vert", prisma::ShaderStage::Vertex },
            { "smaa_edge_detection.frag", prisma::ShaderStage::Fragment },
            { "smaa_weight_calculation.vert", prisma::ShaderStage::Vertex },
            { "smaa_weight_calculation.frag", prisma::ShaderStage::Fragment },
            { "smaa_blending.vert", prisma::ShaderStage::Vertex },
            { "smaa_blending.frag", prisma::ShaderStage::Fragment },
        };
        for (std::size_t i = 0; i < 38; ++i)
            singles[i] = make(kSingles[i].file, kSingles[i].stage, ct::String());
        terrainComposite = make("terrain_composite_fragment.glsl", prisma::ShaderStage::Fragment,
                ct::String());
        clusterRender = make("cluster_render_fragment.glsl", prisma::ShaderStage::Fragment,
                ct::String("#define USE_ATTACHMENT\n#define subgroupBroadcastFirst(value) (value)\n"
                           "#define subgroupBallot(value) uvec4((value) ? 1u : 0u, 0u, 0u, 0u)\n"
                           "#define subgroupBallotExclusiveBitCount(value) 0u\n"
                           "#define subgroupOr(value) (value)\n"));
    }

    ~Registry()
    {
        for (std::size_t i = 0; i < owned.size(); ++i) delete owned[i];
    }

    void buildScene()
    {
        for (unsigned key = 0; key < kSceneVariantCount; ++key)
        {
            sceneVertex[key] = sceneFragment[key] = sceneNoShadow[key] = sceneCoverage[key] =
                    sceneCoverageNoShadow[key] = nullptr;
            const bool unshaded = ((key >> 5) & 1u) != 0;
            const bool litOnly = ((key & 14u) | (key >> 7)) != 0;
            const bool lightmap = ((key >> 7) & 1u) != 0;
            const bool lightmapDetail = (((key >> 6) & 1u) != 0) && lightmap;
            if ((unshaded && litOnly) || lightmapDetail) continue;
            ct::String defines = sceneDefines(key);
            if (!unshaded) defines += "#define FEATURE_SSAO\n#define FEATURE_SSR\n";
            const bool scissor = ((key >> 4) & 1u) != 0;
            if (scissor && !lightmap) sceneCoverage[key] = fragment(defines + kCoverage);
            if (!(unshaded || !scissor || lightmap))
                sceneCoverageNoShadow[key] =
                        fragment(withoutDirectionalShadows(defines) + kShadowsDisabled + kCoverage);
            if (!unshaded)
                sceneNoShadow[key] =
                        fragment(withoutDirectionalShadows(defines) + kShadowsDisabled);
            sceneVertex[key] = vertex(defines);
            sceneFragment[key] = fragment(defines);
        }
    }

    void buildSky()
    {
        const ct::String base("#define MATERIAL_UNIFORMS_USED\n"
                              "#define MAX_DIRECTIONAL_LIGHT_DATA_STRUCTS 4\n");
        skyVertex = make("sky_vertex.glsl", prisma::ShaderStage::Vertex, base);
        for (unsigned key = 0; key < kSkyVariantCount; ++key)
        {
            ct::String defines = base;
            const unsigned material = key >> 2;
            if (material == 3) defines += "#define SKY_MATERIAL_FLAT\n";
            else if (material == 1)
                defines += "#define SKY_MATERIAL_PANORAMA\n";
            else if (material == 2)
                defines += "#define SKY_MATERIAL_CUBEMAP\n";
            if (key & 1u) defines += "#define USE_DEBANDING\n";
            if ((key >> 1) & 1u) defines += "#define USE_CUBEMAP_PASS\n";
            skyFragment[key] = make("sky_fragment.glsl", prisma::ShaderStage::Fragment, defines);
        }
    }

    void buildOctmap()
    {
        octmap[0] = make("octmap_downsample_fragment.glsl", prisma::ShaderStage::Fragment,
                ct::String());
        octmap[1] = make("octmap_downsample_fragment.glsl", prisma::ShaderStage::Fragment,
                ct::String("#define OCTMAP_SOURCE_ARRAY\n"));
    }

    void buildShadow()
    {
        shadowVertex = vertex(ct::String(kDepthBase));
        shadowDualVertex = vertex(ct::String("#define MODE_RENDER_DEPTH\n") + kDualParaboloid +
                                  "#define SHADOWS_DISABLED\n#define DIFFUSE_BURLEY\n");
    }

    void buildMultimesh()
    {
        const unsigned types[3] = { 0, 8, 32 };
        for (unsigned t = 0; t < 3; ++t)
        {
            const ct::String base = sceneDefines(types[t]);
            for (unsigned flags = 0; flags < 4; ++flags)
                multimeshScene[t][flags] = vertex(base + multimeshDefines(flags));
        }
        for (unsigned flags = 0; flags < 4; ++flags)
        {
            multimeshShadow[flags] = vertex(ct::String(kDepthBase) + multimeshDefines(flags));
            multimeshShadowDual[flags] = vertex(
                    ct::String("#define MODE_RENDER_DEPTH\n") + kDualParaboloid +
                    "#define SHADOWS_DISABLED\n#define DIFFUSE_BURLEY\n" + multimeshDefines(flags));
        }
    }

    void buildDepth()
    {
        const ct::String base(
                "#define MATERIAL_UNIFORMS_USED\n#define UV_USED\n#define COLOR_USED\n"
                "#define MODE_RENDER_DEPTH\n#define SHADOWS_DISABLED\n"
                "#define DIFFUSE_BURLEY\n");
        for (unsigned dual = 0; dual < 2; ++dual)
        {
            const ct::String mode = dual ? ct::String(kDualParaboloid) : ct::String();
            for (unsigned mm = 0; mm < 5; ++mm)
            {
                const ct::String extra = mm > 0 ? multimeshDefines(mm - 1) : ct::String();
                depthVertex[dual][mm] = vertex(base + mode + extra);
            }
            for (unsigned albedo = 0; albedo < 2; ++albedo)
            {
                ct::String extra(kScissor);
                if (albedo) extra += kAlbedo;
                depthFragment[dual][albedo] = fragment(base + mode + extra);
            }
        }
        for (unsigned albedo = 0; albedo < 2; ++albedo)
        {
            ct::String extra = ct::String(kScissor) + kCoverage;
            if (albedo) extra += kAlbedo;
            depthCoverage[albedo] = fragment(base + extra);
        }
    }

    void buildScreenSpace()
    {
        for (unsigned mode = 0; mode < 6; ++mode)
        {
            ct::String defines;
            if (mode == 1 || mode == 5) defines += "#define USE_HALF_SIZE\n";
            if (mode == 2 || mode == 3) defines += "#define GENERATE_MIPS\n";
            if (mode == 3) defines += "#define USE_HALF_SIZE\n";
            if (mode == 4 || mode == 5) defines += "#define USE_HALF_BUFFERS\n";
            ssaoDownsample[mode] = compute("ssao_downsample_compute.glsl", defines);
        }
        const char* const gather[3] = { "", "#define SSAO_BASE\n", "#define ADAPTIVE\n" };
        const char* const importance[3] = { "#define GENERATE_MAP\n", "#define PROCESS_MAPA\n",
            "#define PROCESS_MAPB\n" };
        const char* const blur[3] = { "#define MODE_NON_SMART\n", "#define MODE_SMART\n",
            "#define MODE_WIDE\n" };
        const char* const interleave[3] = { "#define MODE_NON_SMART\n", "#define MODE_SMART\n",
            "#define MODE_HALF\n" };
        for (unsigned i = 0; i < 3; ++i)
        {
            ssaoGather[i] = compute("ssao_gather_compute.glsl", ct::String(gather[i]));
            ssaoImportance[i] =
                    compute("ssao_importance_map_compute.glsl", ct::String(importance[i]));
            ssaoBlur[i] = compute("ssao_blur_compute.glsl", ct::String(blur[i]));
            ssaoInterleave[i] = compute("ssao_interleave_compute.glsl", ct::String(interleave[i]));
        }
    }

    void buildFsr()
    {
        fsrUpscale[0] = compute("fsr_upscale_compute.glsl",
                ct::String("#define MODE_FSR_UPSCALE_FALLBACK\n"));
    }

    void buildSsr()
    {
        const char* const copy[3] = { "#define MODE_SIMPLE_COPY\n",
            "#define MODE_SIMPLE_COPY_DEPTH\n", "#define MODE_MIPMAP\n" };
        for (unsigned i = 0; i < 3; ++i)
            ssrCopy[i] = compute("ssr_copy_compute.glsl", ct::String(copy[i]));
        const char* const odd[4] = { "", "#define MODE_ODD_WIDTH\n", "#define MODE_ODD_HEIGHT\n",
            "#define MODE_ODD_WIDTH\n#define MODE_ODD_HEIGHT\n" };
        for (unsigned i = 0; i < 4; ++i)
        {
            ssrDownsample[i] = compute("ssr_downsample_compute.glsl", ct::String(odd[i]));
            ssrHiz[i] = compute("ssr_hiz_compute.glsl", ct::String(odd[i]));
        }
        ssrTrace[0] = compute("ssr_trace_compute.glsl", ct::String());
        ssrFilter[0] = compute("ssr_filter_compute.glsl", ct::String());
        ssrResolve[0] = compute("ssr_resolve_compute.glsl", ct::String());
    }

    void buildDof()
    {
        const char* const computeModes[5] = { "#define MODE_GEN_BLUR_SIZE\n",
            "#define MODE_BOKEH_BOX\n", "#define MODE_BOKEH_HEXAGONAL\n",
            "#define MODE_BOKEH_CIRCULAR\n", "#define MODE_COMPOSITE_BOKEH\n" };
        for (unsigned i = 0; i < 5; ++i)
            dofCompute[i] = compute("bokeh_dof_compute.glsl", ct::String(computeModes[i]));
        const char* const rasterModes[7] = { "#define MODE_GEN_BLUR_SIZE\n",
            "#define MODE_BOKEH_BOX\n#define OUTPUT_WEIGHT\n", "#define MODE_BOKEH_BOX\n",
            "#define MODE_BOKEH_HEXAGONAL\n#define OUTPUT_WEIGHT\n",
            "#define MODE_BOKEH_HEXAGONAL\n",
            "#define MODE_BOKEH_CIRCULAR\n#define OUTPUT_WEIGHT\n",
            "#define MODE_COMPOSITE_BOKEH\n" };
        for (unsigned i = 0; i < 7; ++i)
            dofRaster[i] = make("bokeh_dof_raster_fragment.glsl", prisma::ShaderStage::Fragment,
                    ct::String(rasterModes[i]));
    }

    void buildNormalRoughness()
    {
        for (unsigned key = 0; key < 128; ++key)
        {
            normalFragment[key] = nullptr;
            const bool albedo = (key & 1u) != 0;
            const bool unused = (key & 36u) != 0;
            const bool normal = ((key >> 3) & 1u) != 0;
            const bool scissor = ((key >> 4) & 1u) != 0;
            const bool detail = ((key >> 6) & 1u) != 0;
            if (unused || (albedo && !scissor) || (detail && !normal)) continue;
            normalFragment[key] = fragment(normalRoughnessDefines(key));
        }
        for (unsigned index = 0; index < 4; ++index)
        {
            normalVertex[index] = nullptr;
            const unsigned normal = index & 1u;
            const unsigned detail = index >> 1;
            if (detail && !normal) continue;
            normalVertex[index] = vertex(normalRoughnessDefines(normal * 8u + detail * 64u));
        }
        for (unsigned normal = 0; normal < 2; ++normal)
        {
            const ct::String defines = normalRoughnessDefines(normal * 8u);
            for (unsigned flags = 0; flags < 4; ++flags)
                normalMultimesh[normal][flags] = vertex(defines + multimeshDefines(flags));
        }
    }

    void buildBillboard()
    {
        const unsigned modes[3] = { 1, 2, 5 };
        const unsigned types[3] = { 0, 8, 32 };
        for (unsigned m = 0; m < 3; ++m)
        {
            for (unsigned t = 0; t < 3; ++t)
            {
                const ct::String base = sceneDefines(types[t]);
                for (unsigned mm = 0; mm < 5; ++mm)
                {
                    billboard[m][t][mm] = nullptr;
                    if (modes[m] == 2 && (mm != 4 || types[t] != 0)) continue;
                    ct::String extra("#define FEATURE_BILLBOARD\n");
                    if (modes[m] == 2)
                        extra = "#define FEATURE_GRASS\n#define VERTEX_WORLD_COORDS_USED\n";
                    if (modes[m] == 5)
                        extra = "#define FEATURE_LEAF_WIND\n#define VERTEX_WORLD_COORDS_USED\n";
                    if (mm > 0) extra += multimeshDefines(mm - 1);
                    billboard[m][t][mm] = vertex(base + extra);
                }
            }
        }
    }

    void buildImpostor()
    {
        const ct::String base(
                "#define MATERIAL_UNIFORMS_USED\n#define UV_USED\n#define COLOR_USED\n"
                "#define SPECULAR_OCCLUSION_DISABLED\n#define DIFFUSE_BURLEY\n"
                "#define MODE_UNSHADED\n#define SHADOWS_DISABLED\n"
                "#define FEATURE_IMPOSTOR_CAPTURE\n#define MAX_ROUGHNESS_LOD 7.0\n"
                "#define FOG_DISABLED\n");
        for (unsigned key = 0; key < 128; ++key)
        {
            impostorFragment[key] = nullptr;
            if ((key & ~(1u | 2u | 8u | 16u | 64u)) != 0) continue;
            ct::String defines = base;
            if (key & 1u) defines += kAlbedo;
            if ((key >> 1) & 1u) defines += kPacked;
            if ((key >> 3) & 1u) defines += kNormal;
            if ((key >> 4) & 1u) defines += kScissor;
            if ((key >> 6) & 1u) defines += kDetail;
            impostorFragment[key] = fragment(defines);
        }
        for (unsigned flags = 0; flags < 4; ++flags)
        {
            ct::String defines = base;
            if (flags & 1u) defines += kNormal;
            if ((flags >> 1) & 1u) defines += kDetail;
            impostorVertex[flags] = vertex(defines);
        }
        impostorCardVertex = make("impostor_card.vert", prisma::ShaderStage::Vertex,
                ct::String("#define SHADOWS_DISABLED\n#define DIFFUSE_BURLEY\n"));
        ct::String lit = sceneDefines(0);
        const ct::String uniforms("#define MATERIAL_UNIFORMS_USED\n");
        const std::size_t at = lit.find(uniforms);
        if (at != ct::String::npos) lit = lit.substr(0, at) + lit.substr(at + uniforms.size());
        lit += "#define FEATURE_IMPOSTOR_CARD\n";
        impostorCardFragment[0] = fragment(lit + "#define IMPOSTOR_CARD_CUT\n");
        impostorCardFragment[1] = fragment(lit + "#define IMPOSTOR_CARD_COVERAGE\n");
        impostorCardFragment[2] = fragment(
                ct::String("#define UV_USED\n#define COLOR_USED\n#define MODE_RENDER_DEPTH\n"
                           "#define SHADOWS_DISABLED\n#define DIFFUSE_BURLEY\n#define "
                           "FEATURE_IMPOSTOR_CARD\n"));
    }

    void buildTerrain()
    {
        const unsigned types[3] = { 0, 8, 32 };
        for (unsigned t = 0; t < 3; ++t)
            terrainVertex[t] = vertex(sceneDefines(types[t]) + "#define FEATURE_TERRAIN_MORPH\n");
        for (unsigned dual = 0; dual < 2; ++dual)
        {
            ct::String defines("#define COLOR_USED\n#define MODE_RENDER_DEPTH\n");
            if (dual) defines += kDualParaboloid;
            defines += "#define SHADOWS_DISABLED\n#define DIFFUSE_BURLEY\n"
                       "#define FEATURE_TERRAIN_MORPH\n";
            terrainShadowVertex[dual] = vertex(defines);
        }
        for (unsigned type = 0; type < 2; ++type)
            terrainFragment[type] = fragment(sceneDefines(type) + "#define FEATURE_TERRAIN\n");
    }

    void buildOcean()
    {
        const ct::String base = sceneDefines(0);
        ocean[0] = vertex(base + "#define FEATURE_OCEAN\n#define VERTEX_WORLD_COORDS_USED\n");
        ocean[1] = fragment(base + "#define FEATURE_OCEAN\n");
    }

    void buildGrassAndLeafDepth()
    {
        for (unsigned dual = 0; dual < 2; ++dual)
        {
            ct::String defines(
                    "#define MATERIAL_UNIFORMS_USED\n#define UV_USED\n#define COLOR_USED\n"
                    "#define MODE_RENDER_DEPTH\n");
            if (dual) defines += kDualParaboloid;
            defines +=
                    "#define SHADOWS_DISABLED\n#define DIFFUSE_BURLEY\n"
                    "#define FEATURE_MULTIMESH\n#define SC_PACKED_1 13u\n#define FEATURE_GRASS\n";
            depthGrass[dual] = vertex(defines);
        }
        const ct::String base(
                "#define MATERIAL_UNIFORMS_USED\n#define UV_USED\n#define COLOR_USED\n"
                "#define MODE_RENDER_DEPTH\n#define SHADOWS_DISABLED\n"
                "#define DIFFUSE_BURLEY\n#define FEATURE_LEAF_WIND\n"
                "#define VERTEX_WORLD_COORDS_USED\n");
        for (unsigned mm = 0; mm < 5; ++mm)
            depthLeafWind[mm] = vertex(base + (mm > 0 ? multimeshDefines(mm - 1) : ct::String()));
    }
};

Registry& registry()
{
    static Registry instance;
    return instance;
}

Registry& sRegistry = registry();

} // namespace

const ShaderVariant* const* const kSceneVertexBlobs = sRegistry.sceneVertex;
const ShaderVariant* const* const kSceneFragmentBlobs = sRegistry.sceneFragment;
const ShaderVariant* const* const kSceneNoShadowFragmentBlobs = sRegistry.sceneNoShadow;
const ShaderVariant* const* const kSceneCoverageFragmentBlobs = sRegistry.sceneCoverage;
const ShaderVariant* const* const kSceneCoverageNoShadowFragmentBlobs =
        sRegistry.sceneCoverageNoShadow;
const ShaderVariant* const kSkyVertexBlob = sRegistry.skyVertex;
const ShaderVariant* const* const kSkyFragmentBlobs = sRegistry.skyFragment;
const ShaderVariant* const* const kOctmapDownsampleBlobs = sRegistry.octmap;
const ShaderVariant& gui_vert = *sRegistry.singles[0];
const ShaderVariant& gui_frag = *sRegistry.singles[1];
const ShaderVariant& skeleton_comp = *sRegistry.singles[2];
const ShaderVariant& impostor_prepare_comp = *sRegistry.singles[3];
const ShaderVariant& integrate_dfg_comp = *sRegistry.singles[4];
const ShaderVariant& octmap_vert = *sRegistry.singles[5];
const ShaderVariant& octmap_roughness_frag = *sRegistry.singles[6];
const ShaderVariant& shadow_frag = *sRegistry.singles[7];
const ShaderVariant& shadow_dp_frag = *sRegistry.singles[8];
const ShaderVariant& cube_to_dp_frag = *sRegistry.singles[9];
const ShaderVariant& batch_vert = *sRegistry.singles[10];
const ShaderVariant& batch_frag = *sRegistry.singles[11];
const ShaderVariant& tonemap_vert = *sRegistry.singles[12];
const ShaderVariant& tonemap_frag = *sRegistry.singles[13];
const ShaderVariant& glow_blur_frag = *sRegistry.singles[14];
const ShaderVariant& depth_read_frag = *sRegistry.singles[15];
const ShaderVariant& atlas_copy_frag = *sRegistry.singles[16];
const ShaderVariant& lensflare_vert = *sRegistry.singles[17];
const ShaderVariant& lensflare_frag = *sRegistry.singles[18];
const ShaderVariant& hair_simulate_comp = *sRegistry.singles[19];
const ShaderVariant& hair_wind_comp = *sRegistry.singles[20];
const ShaderVariant& screen_copy_frag = *sRegistry.singles[21];
const ShaderVariant& ocean_simulate_comp = *sRegistry.singles[22];
const ShaderVariant& ocean_fft_comp = *sRegistry.singles[23];
const ShaderVariant& ocean_fft_v2_comp = *sRegistry.singles[24];
const ShaderVariant& ocean_displacement_comp = *sRegistry.singles[25];
const ShaderVariant& ocean_gradient_comp = *sRegistry.singles[26];
const ShaderVariant& best_fit_normal_comp = *sRegistry.singles[27];
const ShaderVariant& cluster_render_vert = *sRegistry.singles[28];
const ShaderVariant& cluster_store_comp = *sRegistry.singles[29];
const ShaderVariant& cluster_clear_comp = *sRegistry.singles[30];
const ShaderVariant& upscale_frag = *sRegistry.singles[31];
const ShaderVariant& smaa_edge_detection_vert = *sRegistry.singles[32];
const ShaderVariant& smaa_edge_detection_frag = *sRegistry.singles[33];
const ShaderVariant& smaa_weight_calculation_vert = *sRegistry.singles[34];
const ShaderVariant& smaa_weight_calculation_frag = *sRegistry.singles[35];
const ShaderVariant& smaa_blending_vert = *sRegistry.singles[36];
const ShaderVariant& smaa_blending_frag = *sRegistry.singles[37];
const ShaderVariant& shadow_vert = *sRegistry.shadowVertex;
const ShaderVariant& shadow_dp_vert = *sRegistry.shadowDualVertex;
const ShaderVariant& terrain_composite_frag = *sRegistry.terrainComposite;
const ShaderVariant& cluster_render_frag = *sRegistry.clusterRender;
const ShaderVariant* const (*const kSceneMultiMeshVertexBlobs)[4] = sRegistry.multimeshScene;
const ShaderVariant* const* const kShadowMultiMeshVertexBlobs = sRegistry.multimeshShadow;
const ShaderVariant* const* const kShadowDualMultiMeshVertexBlobs = sRegistry.multimeshShadowDual;
const ShaderVariant* const (*const kDepthVertexBlobs)[5] = sRegistry.depthVertex;
const ShaderVariant* const (*const kDepthFragmentBlobs)[2] = sRegistry.depthFragment;
const ShaderVariant* const* const kDepthCoverageFragmentBlobs = sRegistry.depthCoverage;
const ShaderVariant* const* const kSsaoDownsampleBlobs = sRegistry.ssaoDownsample;
const ShaderVariant* const* const kSsaoGatherBlobs = sRegistry.ssaoGather;
const ShaderVariant* const* const kSsaoImportanceBlobs = sRegistry.ssaoImportance;
const ShaderVariant* const* const kSsaoBlurBlobs = sRegistry.ssaoBlur;
const ShaderVariant* const* const kSsaoInterleaveBlobs = sRegistry.ssaoInterleave;
const ShaderVariant* const* const kSsrCopyBlobs = sRegistry.ssrCopy;
const ShaderVariant* const* const kSsrDownsampleBlobs = sRegistry.ssrDownsample;
const ShaderVariant* const* const kSsrHizBlobs = sRegistry.ssrHiz;
const ShaderVariant* const* const kSsrTraceBlobs = sRegistry.ssrTrace;
const ShaderVariant* const* const kFsrUpscaleBlobs = sRegistry.fsrUpscale;
const ShaderVariant* const* const kSsrFilterBlobs = sRegistry.ssrFilter;
const ShaderVariant* const* const kSsrResolveBlobs = sRegistry.ssrResolve;
const ShaderVariant* const* const kDofComputeBlobs = sRegistry.dofCompute;
const ShaderVariant* const* const kDofRasterBlobs = sRegistry.dofRaster;
const ShaderVariant* const* const kNormalVertexBlobs = sRegistry.normalVertex;
const ShaderVariant* const (*const kNormalMultiMeshVertexBlobs)[4] = sRegistry.normalMultimesh;
const ShaderVariant* const* const kNormalFragmentBlobs = sRegistry.normalFragment;
const ShaderVariant* const (*const kSceneBillboardVertexBlobs)[3][5] = sRegistry.billboard;
const ShaderVariant* const* const kImpostorCaptureVertexBlobs = sRegistry.impostorVertex;
const ShaderVariant* const* const kImpostorCaptureFragmentBlobs = sRegistry.impostorFragment;
const ShaderVariant* const kImpostorCardVertexBlob = sRegistry.impostorCardVertex;
const ShaderVariant* const* const kImpostorCardFragmentBlobs = sRegistry.impostorCardFragment;
const ShaderVariant* const* const kSceneTerrainVertexBlobs = sRegistry.terrainVertex;
const ShaderVariant* const* const kShadowTerrainVertexBlobs = sRegistry.terrainShadowVertex;
const ShaderVariant* const* const kSceneTerrainFragmentBlobs = sRegistry.terrainFragment;
const ShaderVariant* const* const kSceneOceanBlobs = sRegistry.ocean;
const ShaderVariant* const* const kDepthGrassVertexBlobs = sRegistry.depthGrass;
const ShaderVariant* const* const kDepthLeafWindVertexBlobs = sRegistry.depthLeafWind;

} // namespace prisma3d
