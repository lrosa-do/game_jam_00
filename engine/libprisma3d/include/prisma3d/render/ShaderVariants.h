#pragma once

#include "prisma3d/render/ShaderCompiler.h"

namespace prisma3d
{

constexpr unsigned kSceneVariantCount = 256;
constexpr unsigned kSkyVariantCount = 13;

extern const ShaderVariant* const* const kSceneVertexBlobs;
extern const ShaderVariant* const* const kSceneFragmentBlobs;
extern const ShaderVariant* const* const kSceneNoShadowFragmentBlobs;
extern const ShaderVariant* const* const kSceneCoverageFragmentBlobs;
extern const ShaderVariant* const* const kSceneCoverageNoShadowFragmentBlobs;

extern const ShaderVariant* const kSkyVertexBlob;
extern const ShaderVariant* const* const kSkyFragmentBlobs;

extern const ShaderVariant* const* const kOctmapDownsampleBlobs;

extern const ShaderVariant& gui_vert;
extern const ShaderVariant& gui_frag;
extern const ShaderVariant& skeleton_comp;
extern const ShaderVariant& impostor_prepare_comp;
extern const ShaderVariant& integrate_dfg_comp;
extern const ShaderVariant& octmap_vert;
extern const ShaderVariant& octmap_roughness_frag;
extern const ShaderVariant& shadow_frag;
extern const ShaderVariant& shadow_dp_frag;
extern const ShaderVariant& cube_to_dp_frag;
extern const ShaderVariant& batch_vert;
extern const ShaderVariant& batch_frag;
extern const ShaderVariant& tonemap_vert;
extern const ShaderVariant& tonemap_frag;
extern const ShaderVariant& glow_blur_frag;
extern const ShaderVariant& depth_read_frag;
extern const ShaderVariant& atlas_copy_frag;
extern const ShaderVariant& lensflare_vert;
extern const ShaderVariant& lensflare_frag;
extern const ShaderVariant& hair_simulate_comp;
extern const ShaderVariant& hair_wind_comp;
extern const ShaderVariant& screen_copy_frag;
extern const ShaderVariant& ocean_simulate_comp;
extern const ShaderVariant& ocean_fft_comp;
extern const ShaderVariant& ocean_fft_v2_comp;
extern const ShaderVariant& ocean_displacement_comp;
extern const ShaderVariant& ocean_gradient_comp;
extern const ShaderVariant& best_fit_normal_comp;
extern const ShaderVariant& cluster_render_vert;
extern const ShaderVariant& cluster_store_comp;
extern const ShaderVariant& cluster_clear_comp;
extern const ShaderVariant& upscale_frag;
extern const ShaderVariant& smaa_edge_detection_vert;
extern const ShaderVariant& smaa_edge_detection_frag;
extern const ShaderVariant& smaa_weight_calculation_vert;
extern const ShaderVariant& smaa_weight_calculation_frag;
extern const ShaderVariant& smaa_blending_vert;
extern const ShaderVariant& smaa_blending_frag;
extern const ShaderVariant& shadow_vert;
extern const ShaderVariant& shadow_dp_vert;
extern const ShaderVariant& terrain_composite_frag;
extern const ShaderVariant& cluster_render_frag;

extern const ShaderVariant* const (*const kSceneMultiMeshVertexBlobs)[4];
extern const ShaderVariant* const* const kShadowMultiMeshVertexBlobs;
extern const ShaderVariant* const* const kShadowDualMultiMeshVertexBlobs;

extern const ShaderVariant* const (*const kDepthVertexBlobs)[5];
extern const ShaderVariant* const (*const kDepthFragmentBlobs)[2];
extern const ShaderVariant* const* const kDepthCoverageFragmentBlobs;

extern const ShaderVariant* const* const kSsaoDownsampleBlobs;
extern const ShaderVariant* const* const kSsaoGatherBlobs;
extern const ShaderVariant* const* const kSsaoImportanceBlobs;
extern const ShaderVariant* const* const kSsaoBlurBlobs;
extern const ShaderVariant* const* const kSsaoInterleaveBlobs;

extern const ShaderVariant* const* const kSsrCopyBlobs;
extern const ShaderVariant* const* const kSsrDownsampleBlobs;
extern const ShaderVariant* const* const kSsrHizBlobs;
extern const ShaderVariant* const* const kSsrTraceBlobs;
extern const ShaderVariant* const* const kFsrUpscaleBlobs;
extern const ShaderVariant* const* const kSsrFilterBlobs;
extern const ShaderVariant* const* const kSsrResolveBlobs;

extern const ShaderVariant* const* const kDofComputeBlobs;
extern const ShaderVariant* const* const kDofRasterBlobs;

extern const ShaderVariant* const* const kNormalVertexBlobs;
extern const ShaderVariant* const (*const kNormalMultiMeshVertexBlobs)[4];
extern const ShaderVariant* const* const kNormalFragmentBlobs;

extern const ShaderVariant* const (*const kSceneBillboardVertexBlobs)[3][5];

extern const ShaderVariant* const* const kImpostorCaptureVertexBlobs;
extern const ShaderVariant* const* const kImpostorCaptureFragmentBlobs;
extern const ShaderVariant* const kImpostorCardVertexBlob;
extern const ShaderVariant* const* const kImpostorCardFragmentBlobs;

extern const ShaderVariant* const* const kSceneTerrainVertexBlobs;
extern const ShaderVariant* const* const kShadowTerrainVertexBlobs;
extern const ShaderVariant* const* const kSceneTerrainFragmentBlobs;

extern const ShaderVariant* const* const kSceneOceanBlobs;
extern const ShaderVariant* const* const kDepthGrassVertexBlobs;
extern const ShaderVariant* const* const kDepthLeafWindVertexBlobs;

} // namespace prisma3d
