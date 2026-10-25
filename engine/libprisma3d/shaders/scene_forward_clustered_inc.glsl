#define M_PI 3.14159265359
#define M_TAU 6.28318530718
#define ROUGHNESS_MAX_LOD 5

#define MAX_VOXEL_GI_INSTANCES 8
#define MAX_VIEWS 2


#ifdef FEATURE_CLUSTER
#include "cluster_data_inc.glsl"
#include "decal_data_inc.glsl"
#endif
#include "oct_inc.glsl"
#include "scene_data_inc.glsl"

#if !defined(MODE_RENDER_DEPTH) || defined(MODE_RENDER_MATERIAL) || defined(MODE_RENDER_SDF) || defined(MODE_RENDER_NORMAL_ROUGHNESS) || defined(MODE_RENDER_VOXEL_GI) || defined(TANGENT_USED) || defined(NORMAL_MAP_USED) || defined(BENT_NORMAL_MAP_USED) || defined(LIGHT_ANISOTROPY_USED)
#ifndef NORMAL_USED
#define NORMAL_USED
#endif
#endif

#if !defined(TANGENT_USED) && (defined(NORMAL_MAP_USED) || defined(BENT_NORMAL_MAP_USED) || defined(LIGHT_ANISOTROPY_USED))
#define TANGENT_USED
#endif

layout(set = 0, binding = 1, std140) uniform DrawCall {
	uint instance_index;
	uint uv_offset;
	uint multimesh_motion_vectors_current_offset;
	uint multimesh_motion_vectors_previous_offset;
	vec4 instance_color;
	uvec4 planar_reflection;
}
draw_call;

const bool sc_emulate_point_size = false;

#ifdef POINT_SIZE_USED

#define VERTEX_INDEX (sc_emulate_point_size ? gl_InstanceIndex : gl_VertexIndex)
#define INSTANCE_INDEX (sc_emulate_point_size ? (gl_VertexIndex / 6) : gl_InstanceIndex)

#else

#define VERTEX_INDEX gl_VertexIndex
#define INSTANCE_INDEX gl_InstanceIndex

#endif

#define REFLECTION_MULTIPLIER 1.0

#define SDFGI_MAX_CASCADES 8

/* Set 0: Base Pass (never changes) */

#ifdef FEATURE_LIGHTS
#include "light_data_inc.glsl"
#endif

#define INSTANCE_FLAGS_DYNAMIC (1 << 3)
#define INSTANCE_FLAGS_NON_UNIFORM_SCALE (1 << 4)
#define INSTANCE_FLAGS_USE_GI_BUFFERS (1 << 5)
#define INSTANCE_FLAGS_USE_SDFGI (1 << 6)
#define INSTANCE_FLAGS_USE_LIGHTMAP_CAPTURE (1 << 7)
#define INSTANCE_FLAGS_USE_LIGHTMAP (1 << 8)
#define INSTANCE_FLAGS_USE_SH_LIGHTMAP (1 << 9)
#define INSTANCE_FLAGS_USE_VOXEL_GI (1 << 10)
#define INSTANCE_FLAGS_PARTICLES (1 << 11)
#define INSTANCE_FLAGS_PARTICLE_TRAIL_SHIFT 16
#define INSTANCE_FLAGS_FADE_SHIFT 24
//3 bits of stride
#define INSTANCE_FLAGS_PARTICLE_TRAIL_MASK 0xFF

#define SCREEN_SPACE_EFFECTS_FLAGS_USE_SSAO (1 << 0)
#define SCREEN_SPACE_EFFECTS_FLAGS_USE_SSIL (1 << 1)
#define SCREEN_SPACE_EFFECTS_FLAGS_USE_SSR (1 << 2)
#define SCREEN_SPACE_EFFECTS_FLAGS_RESOLVE_SSR (1 << 3)
#define SCREEN_SPACE_EFFECTS_FLAGS_USE_SSCS (1 << 4)

#ifdef FEATURE_LIGHTS
#define MAX_DIRECTIONAL_LIGHT_DATA_STRUCTS 8

layout(set = 0, binding = 3, std140) uniform LightCountsBlock {
	uint omni;
	uint spot;
	uint pad0;
	uint pad1;
}
light_counts;

layout(set = 0, binding = 4, std140) uniform DirectionalLights {
	DirectionalLightData data[MAX_DIRECTIONAL_LIGHT_DATA_STRUCTS];
}
directional_lights;

layout(set = 2, binding = 1, std430) restrict readonly buffer OmniLights {
	LightData data[];
}
omni_lights;

layout(set = 2, binding = 2, std430) restrict readonly buffer SpotLights {
	LightData data[];
}
spot_lights;
#endif

#ifdef FEATURE_LIGHT_RESOURCES




layout(set = 0, binding = 5, std430) restrict readonly buffer AreaLights {
	LightData data[];
}
area_lights;

layout(set = 0, binding = 6, std430) restrict readonly buffer ReflectionProbeData {
	ReflectionData data[];
}
reflections;



#define LIGHTMAP_FLAG_USE_DIRECTION 1
#define LIGHTMAP_FLAG_USE_SPECULAR_DIRECTION 2

#define LIGHTMAP_SHADOWMASK_MODE_NONE 0
#define LIGHTMAP_SHADOWMASK_MODE_REPLACE 1
#define LIGHTMAP_SHADOWMASK_MODE_OVERLAY 2
#define LIGHTMAP_SHADOWMASK_MODE_ONLY 3

struct Lightmap {
	mat3x4 normal_xform_and_specular_intensity; // "normal_xform" is the 3x3 matrix. "specular_intensity" is the 4th row of the 1st column.
	vec2 light_texture_size;
	float exposure_normalization;
	uint flags;
};

layout(set = 0, binding = 8, std140) restrict readonly buffer Lightmaps {
	Lightmap data[];
}
lightmaps;

struct LightmapCapture {
	vec4 sh[9];
};

layout(set = 0, binding = 9, std140) restrict readonly buffer LightmapCaptures {
	LightmapCapture data[];
}
lightmap_captures;

layout(set = 0, binding = 10) uniform texture2D decal_atlas;
layout(set = 0, binding = 11) uniform texture2D decal_atlas_srgb;

layout(set = 0, binding = 12, std430) restrict readonly buffer Decals {
	DecalData data[];
}
decals;

layout(set = 0, binding = 13, std430) restrict readonly buffer GlobalShaderUniformData {
	vec4 data[];
}
global_shader_uniforms;

struct SDFVoxelGICascadeData {
	vec3 position;
	float to_probe;
	ivec3 probe_world_offset;
	float to_cell; // 1/bounds * grid_size
	vec3 pad;
	float exposure_normalization;
};

layout(set = 0, binding = 14, std140) uniform SDFGI {
	vec3 grid_size;
	uint max_cascades;

	bool use_occlusion;
	int probe_axis_size;
	float probe_to_uvw;
	float normal_bias;

	vec3 lightprobe_tex_pixel_size;
	float energy;

	vec3 lightprobe_uv_offset;
	float y_mult;

	vec3 occlusion_clamp;
	uint pad3;

	vec3 occlusion_renormalize;
	uint pad4;

	vec3 cascade_probe_size;
	uint pad5;

	SDFVoxelGICascadeData cascades[SDFGI_MAX_CASCADES];
}
sdfgi;

layout(set = 0, binding = 15) uniform sampler DEFAULT_SAMPLER_LINEAR_WITH_MIPMAPS_CLAMP;

layout(set = 0, binding = 16) uniform texture2D best_fit_normal_texture;

layout(set = 0, binding = 18) uniform texture2D ltc_lut1;

layout(set = 0, binding = 19) uniform texture2D ltc_lut2;

layout(set = 0, binding = 20) uniform texture2D area_light_atlas;
#endif
/* Set 1: Render Pass (changes per render pass) */

layout(set = 0, binding = 0, std140) uniform SceneDataBlock {
	SceneData data;
	SceneData prev_data;
}
scene_data_block;

/* Specialization Constants */

#ifndef SC_PACKED_0
#define SC_PACKED_0 0u
#endif
#ifndef SC_PACKED_1
#define SC_PACKED_1 0u
#endif

const uint pso_sc_packed_0 = SC_PACKED_0;
const uint pso_sc_packed_1 = SC_PACKED_1;

uint sc_packed_0() {
	return scene_data_block.data.packed_constants;
}

uint sc_packed_1() {
	return pso_sc_packed_1;
}

bool sc_use_forward_gi() {
	return ((sc_packed_0() >> 0) & 1U) != 0;
}

bool sc_use_light_projector() {
	return ((sc_packed_0() >> 1) & 1U) != 0;
}

bool sc_use_light_soft_shadows() {
	return ((sc_packed_0() >> 2) & 1U) != 0;
}

bool sc_use_directional_soft_shadows() {
	return ((sc_packed_0() >> 3) & 1U) != 0;
}

bool sc_decal_use_mipmaps() {
	return ((sc_packed_0() >> 4) & 1U) != 0;
}

bool sc_projector_use_mipmaps() {
	return ((sc_packed_0() >> 5) & 1U) != 0;
}

bool sc_use_depth_fog() {
	return ((sc_packed_0() >> 6) & 1U) != 0;
}

bool sc_use_lightmap_bicubic_filter() {
	return ((sc_packed_0() >> 7) & 1U) != 0;
}

uint sc_soft_shadow_samples() {
	return (sc_packed_0() >> 8) & 63U;
}

uint sc_penumbra_shadow_samples() {
	return (sc_packed_0() >> 14) & 63U;
}

uint sc_directional_soft_shadow_samples() {
	return (sc_packed_0() >> 20) & 63U;
}

uint sc_directional_penumbra_shadow_samples() {
	return (sc_packed_0() >> 26) & 63U;
}

bool sc_multimesh() {
	return ((sc_packed_1() >> 0) & 1U) != 0;
}

bool sc_multimesh_format_2d() {
	return ((sc_packed_1() >> 1) & 1U) != 0;
}

bool sc_multimesh_has_color() {
	return ((sc_packed_1() >> 2) & 1U) != 0;
}

bool sc_multimesh_has_custom_data() {
	return ((sc_packed_1() >> 3) & 1U) != 0;
}

bool sc_fog_use_legacy_blending() {
	return ((sc_packed_1() >> 4) & 1U) != 0;
}

bool sc_cluster_has_area_light() {
	return ((sc_packed_1() >> 5) & 1U) != 0;
}

bool sc_use_lightmap_specular() {
	return bool(scene_data_block.data.flags & SCENE_DATA_FLAGS_USE_LIGHTMAP_SPECULAR);
}

bool sc_material_feedback() {
	return ((sc_packed_1() >> 7) & 1U) != 0;
}

float sc_luminance_multiplier() {
	// Not used in clustered renderer but we share some code with the mobile renderer that requires this.
	return 1.0;
}


#ifdef FEATURE_CLUSTER
struct ImplementationData {
	uint cluster_shift;
	uint cluster_width;
	uint cluster_type_size;
	uint max_cluster_element_count_div_32;

	uint ss_effects_flags;
	float ssao_light_affect;
	float ssao_ao_affect;
	uint pad1;

	mat4 sdf_to_bounds;

	ivec3 sdf_offset;
	uint pad2;

	ivec3 sdf_size;
	bool gi_upscale_for_msaa;

	bool volumetric_fog_enabled;
	float volumetric_fog_inv_length;
	float volumetric_fog_detail_spread;
	uint volumetric_fog_pad;
};

layout(set = 0, binding = 5, std140) uniform ImplementationDataBlock {
	ImplementationData data;
}
implementation_data_block;

#define implementation_data implementation_data_block.data

layout(set = 2, binding = 3, std430) buffer restrict readonly ClusterBuffer {
	uint data[];
}
cluster_buffer;

layout(set = 2, binding = 4, std430) restrict readonly buffer Decals {
	DecalData data[];
}
decals;

layout(set = 1, binding = 10) uniform sampler2D decal_atlas;
layout(set = 1, binding = 11) uniform sampler2D decal_atlas_srgb;
#define PRISMA_TEXTURE_decal_atlas decal_atlas
#define PRISMA_TEXTURE_decal_atlas_srgb decal_atlas_srgb
#define PRISMA_TEXTURE_radiance_octmap radiance_octmap
#define PRISMA_TEXTURE_directional_shadow_atlas directional_shadow_depth
#define PRISMA_TEXTURE_shadow_atlas shadow_atlas_depth
#ifdef FEATURE_SSAO
layout(set = 1, binding = 21) uniform sampler2D ao_buffer;
#define PRISMA_TEXTURE_ao_buffer ao_buffer
#endif
#ifdef FEATURE_SSR
layout(set = 1, binding = 22) uniform sampler2D ssr_buffer;
layout(set = 1, binding = 23) uniform sampler2D ssr_mip_level_buffer;
#define PRISMA_TEXTURE_ssr_buffer ssr_buffer
#define PRISMA_TEXTURE_ssr_mip_level_buffer ssr_mip_level_buffer
#endif
#define sampler2D(texture_name, sampler_name) PRISMA_TEXTURE_##texture_name
#endif

#ifdef MODE_RENDER_NORMAL_ROUGHNESS
layout(set = 1, binding = 22) uniform sampler2D best_fit_normal_texture;
#define PRISMA_TEXTURE_best_fit_normal_texture best_fit_normal_texture
#define sampler2D(texture_name, sampler_name) PRISMA_TEXTURE_##texture_name
#endif

#include "instance_data_inc.glsl"

layout(set = 2, binding = 0, std430) buffer restrict readonly InstanceDataBuffer {
	InstanceData data[];
}
instances;

#ifdef USE_LIGHTMAP
#define LIGHTMAP_FLAG_USE_DIRECTION 1
#define LIGHTMAP_FLAG_USE_SPECULAR_DIRECTION 2

#define LIGHTMAP_SHADOWMASK_MODE_NONE 0
#define LIGHTMAP_SHADOWMASK_MODE_REPLACE 1
#define LIGHTMAP_SHADOWMASK_MODE_OVERLAY 2
#define LIGHTMAP_SHADOWMASK_MODE_ONLY 3

struct Lightmap {
	mat3x4 normal_xform_and_specular_intensity; // "normal_xform" is the 3x3 matrix. "specular_intensity" is the 4th row of the 1st column.
	vec2 light_texture_size;
	float exposure_normalization;
	uint flags;
};

layout(set = 2, binding = 6, std140) restrict readonly buffer Lightmaps {
	Lightmap data[];
}
lightmaps;

struct LightmapCapture {
	vec4 sh[9];
};

layout(set = 2, binding = 7, std140) restrict readonly buffer LightmapCaptures {
	LightmapCapture data[];
}
lightmap_captures;

layout(set = 1, binding = 24) uniform sampler2DArray lightmap_light_texture;
layout(set = 1, binding = 25) uniform sampler2DArray lightmap_shadowmask_texture;
#endif

#ifdef FEATURE_DIRECTIONAL_SHADOWS
layout(set = 1, binding = 8) uniform sampler2DShadow directional_shadow_atlas;
layout(set = 1, binding = 9) uniform sampler2D directional_shadow_depth;
#define sampler2DShadow(texture_name, sampler_name) texture_name
#endif

#ifndef SHADOWS_DISABLED
layout(set = 1, binding = 12) uniform sampler2DShadow shadow_atlas;
layout(set = 1, binding = 13) uniform sampler2D shadow_atlas_depth;
#endif

#ifdef FEATURE_RADIANCE
layout(set = 1, binding = 7) uniform sampler2DArray radiance_octmap;
#define sampler2DArray(texture_name, sampler_name) texture_name
#endif

#ifdef FEATURE_RENDER_RESOURCES
#ifdef USE_RADIANCE_OCTMAP_ARRAY

layout(set = 1, binding = 3) uniform texture2DArray radiance_octmap;

#else

layout(set = 1, binding = 3) uniform texture2D radiance_octmap;

#endif

layout(set = 1, binding = 4) uniform texture2DArray reflection_atlas;

layout(set = 1, binding = 5) uniform texture2D shadow_atlas;

layout(set = 1, binding = 6) uniform texture2D directional_shadow_atlas;

layout(set = 1, binding = 7) uniform texture2DArray lightmap_textures[MAX_LIGHTMAP_TEXTURES * 2];

layout(set = 1, binding = 8) uniform texture3D voxel_gi_textures[MAX_VOXEL_GI_INSTANCES];

layout(set = 1, binding = 9, std430) buffer restrict readonly ClusterBuffer {
	uint data[];
}
cluster_buffer;

layout(set = 1, binding = 10) uniform sampler decal_sampler;

layout(set = 1, binding = 11) uniform sampler light_projector_sampler;

layout(set = 1, binding = 12 + 0) uniform sampler SAMPLER_NEAREST_CLAMP;
layout(set = 1, binding = 12 + 1) uniform sampler SAMPLER_LINEAR_CLAMP;
layout(set = 1, binding = 12 + 2) uniform sampler SAMPLER_NEAREST_WITH_MIPMAPS_CLAMP;
layout(set = 1, binding = 12 + 3) uniform sampler SAMPLER_LINEAR_WITH_MIPMAPS_CLAMP;
layout(set = 1, binding = 12 + 4) uniform sampler SAMPLER_NEAREST_WITH_MIPMAPS_ANISOTROPIC_CLAMP;
layout(set = 1, binding = 12 + 5) uniform sampler SAMPLER_LINEAR_WITH_MIPMAPS_ANISOTROPIC_CLAMP;
layout(set = 1, binding = 12 + 6) uniform sampler SAMPLER_NEAREST_REPEAT;
layout(set = 1, binding = 12 + 7) uniform sampler SAMPLER_LINEAR_REPEAT;
layout(set = 1, binding = 12 + 8) uniform sampler SAMPLER_NEAREST_WITH_MIPMAPS_REPEAT;
layout(set = 1, binding = 12 + 9) uniform sampler SAMPLER_LINEAR_WITH_MIPMAPS_REPEAT;
layout(set = 1, binding = 12 + 10) uniform sampler SAMPLER_NEAREST_WITH_MIPMAPS_ANISOTROPIC_REPEAT;
layout(set = 1, binding = 12 + 11) uniform sampler SAMPLER_LINEAR_WITH_MIPMAPS_ANISOTROPIC_REPEAT;

#ifdef MODE_RENDER_SDF

layout(r16ui, set = 1, binding = 24) uniform restrict writeonly uimage3D albedo_volume_grid;
layout(r32ui, set = 1, binding = 25) uniform restrict writeonly uimage3D emission_grid;
layout(r32ui, set = 1, binding = 26) uniform restrict writeonly uimage3D emission_aniso_grid;
layout(r32ui, set = 1, binding = 27) uniform restrict uimage3D geom_facing_grid;

//still need to be present for shaders that use it, so remap them to something
#define depth_buffer shadow_atlas
#define color_buffer shadow_atlas
#define normal_roughness_buffer shadow_atlas

#define multiviewSampler sampler2D
#else

#ifdef USE_MULTIVIEW
layout(set = 1, binding = 24) uniform texture2DArray depth_buffer;
layout(set = 1, binding = 25) uniform texture2DArray color_buffer;
layout(set = 1, binding = 26) uniform texture2DArray normal_roughness_buffer;
layout(set = 1, binding = 27) uniform texture2DArray ao_buffer;
layout(set = 1, binding = 28) uniform texture2DArray ambient_buffer;
layout(set = 1, binding = 29) uniform texture2DArray reflection_buffer;
#define multiviewSampler sampler2DArray
#else // USE_MULTIVIEW
layout(set = 1, binding = 24) uniform texture2D depth_buffer;
layout(set = 1, binding = 25) uniform texture2D color_buffer;
layout(set = 1, binding = 26) uniform texture2D normal_roughness_buffer;
layout(set = 1, binding = 27) uniform texture2D ao_buffer;
layout(set = 1, binding = 28) uniform texture2D ambient_buffer;
layout(set = 1, binding = 29) uniform texture2D reflection_buffer;
#define multiviewSampler sampler2D
#endif
layout(set = 1, binding = 30) uniform texture2DArray sdfgi_lightprobe_texture;
layout(set = 1, binding = 31) uniform texture3D sdfgi_occlusion_cascades;

struct VoxelGIData {
	mat4 xform; // 64 - 64

	vec3 bounds; // 12 - 76
	float dynamic_range; // 4 - 80

	float bias; // 4 - 84
	float normal_bias; // 4 - 88
	bool blend_ambient; // 4 - 92
	uint mipmaps; // 4 - 96

	vec3 pad; // 12 - 108
	float exposure_normalization; // 4 - 112
};

layout(set = 1, binding = 32, std140) uniform VoxelGIs {
	VoxelGIData data[MAX_VOXEL_GI_INSTANCES];
}
voxel_gi_instances;

layout(set = 1, binding = 33) uniform texture3D volumetric_fog_texture;

#ifdef USE_MULTIVIEW
layout(set = 1, binding = 34) uniform texture2DArray ssil_buffer;
layout(set = 1, binding = 35) uniform texture2DArray ssr_buffer;
layout(set = 1, binding = 36) uniform texture2DArray ssr_mip_level_buffer;
layout(set = 1, binding = 37) uniform texture2DArray sscs_buffer;
#else
layout(set = 1, binding = 34) uniform texture2D ssil_buffer;
layout(set = 1, binding = 35) uniform texture2D ssr_buffer;
layout(set = 1, binding = 36) uniform texture2D ssr_mip_level_buffer;
layout(set = 1, binding = 37) uniform texture2DArray sscs_buffer;
#endif // USE_MULTIVIEW

#endif

#endif

vec4 normal_roughness_compatibility(vec4 p_normal_roughness) {
	float roughness = p_normal_roughness.w;
	if (roughness > 0.5) {
		roughness = 1.0 - roughness;
	}
	roughness /= (127.0 / 255.0);
	return vec4(normalize(p_normal_roughness.xyz * 2.0 - 1.0) * 0.5 + 0.5, roughness);
}

#ifdef FEATURE_DFG
layout(set = 1, binding = 6) uniform sampler2D dfg;

// https://google.github.io/filament/Filament.html#toc5.3.4.7
// Note: We have an inverted Y-axis in the DFG; therefore, the roughness value must be inverted.
vec2 prefiltered_dfg(float lod, float NoV) {
	return textureLod(dfg, vec2(NoV, 1.0 - lod), 0.0).rg;
}

#endif

// Compute multiscatter compensation
// https://google.github.io/filament/Filament.html#listing_energycompensationimpl
vec3 get_energy_compensation(vec3 f0, float env) {
	return 1.0 + f0 * (1.0 / env - 1.0);
}

#ifdef TEXTURE_STREAMING
// Texture streaming material feedback buffer access
layout(set = 1, binding = 38, std430) buffer restrict MaterialFeedbackBuffer {
	uint data[];
}
material_feedback;
#endif

/* Set 2 Skeleton & Instancing (can change per item) */

#ifdef FEATURE_MULTIMESH
layout(set = 2, binding = 5, std430) restrict readonly buffer Transforms {
	vec4 data[];
}
transforms;
#endif

/* Set 3 User Material */
