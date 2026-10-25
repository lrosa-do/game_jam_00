#ifdef MATERIAL_ALBEDO_TEXTURE
layout(set = 1, binding = 0) uniform sampler2D texture_albedo;
#endif
#ifdef MATERIAL_METALLIC_TEXTURE
layout(set = 1, binding = 1) uniform sampler2D texture_metallic;
#endif
#ifdef MATERIAL_ROUGHNESS_TEXTURE
layout(set = 1, binding = 2) uniform sampler2D texture_roughness;
#endif
#ifdef MATERIAL_EMISSION_TEXTURE
layout(set = 1, binding = 3) uniform sampler2D texture_emission;
#endif
#ifdef NORMAL_MAP_USED
layout(set = 1, binding = 4) uniform sampler2D texture_normal;

vec3 water_normal_tap(vec2 uv) {
	vec3 ts_normal;
	ts_normal.xy = texture(texture_normal, uv).xy * 2.0 - 1.0;
	ts_normal.z = sqrt(max(0.0, 1.0 - ts_normal.x * ts_normal.x - ts_normal.y * ts_normal.y));
	return ts_normal;
}
#endif
#ifdef MATERIAL_AO_TEXTURE
layout(set = 1, binding = 5) uniform sampler2D texture_ambient_occlusion;
#endif

#ifdef FEATURE_DETAIL
layout(set = 1, binding = 14) uniform sampler2D texture_detail_albedo;
#ifdef NORMAL_MAP_USED
layout(set = 1, binding = 15) uniform sampler2D texture_detail_normal;
#endif
layout(set = 1, binding = 16) uniform sampler2D texture_detail_mask;
#endif

#define SCENE_DATA_FLAGS_DEBUG_NORMAL (1 << 9)

layout(location = 17) in flat float dissolve_interp;

const float bayer_matrix_8[8][8] = float[8][8](
		float[8](1.0 / 65.0, 49.0 / 65.0, 13.0 / 65.0, 61.0 / 65.0, 4.0 / 65.0, 52.0 / 65.0, 16.0 / 65.0, 64.0 / 65.0),
		float[8](33.0 / 65.0, 17.0 / 65.0, 45.0 / 65.0, 29.0 / 65.0, 36.0 / 65.0, 20.0 / 65.0, 48.0 / 65.0, 32.0 / 65.0),
		float[8](9.0 / 65.0, 57.0 / 65.0, 5.0 / 65.0, 53.0 / 65.0, 12.0 / 65.0, 60.0 / 65.0, 8.0 / 65.0, 56.0 / 65.0),
		float[8](41.0 / 65.0, 25.0 / 65.0, 37.0 / 65.0, 21.0 / 65.0, 44.0 / 65.0, 28.0 / 65.0, 40.0 / 65.0, 24.0 / 65.0),
		float[8](3.0 / 65.0, 51.0 / 65.0, 15.0 / 65.0, 63.0 / 65.0, 2.0 / 65.0, 50.0 / 65.0, 14.0 / 65.0, 62.0 / 65.0),
		float[8](35.0 / 65.0, 19.0 / 65.0, 47.0 / 65.0, 31.0 / 65.0, 34.0 / 65.0, 18.0 / 65.0, 46.0 / 65.0, 30.0 / 65.0),
		float[8](11.0 / 65.0, 59.0 / 65.0, 7.0 / 65.0, 55.0 / 65.0, 10.0 / 65.0, 58.0 / 65.0, 6.0 / 65.0, 54.0 / 65.0),
		float[8](43.0 / 65.0, 27.0 / 65.0, 39.0 / 65.0, 23.0 / 65.0, 42.0 / 65.0, 26.0 / 65.0, 38.0 / 65.0, 22.0 / 65.0));

float dither_mask8(uvec2 pixel) {
	return bayer_matrix_8[pixel.x % 8u][pixel.y % 8u];
}

#ifdef FEATURE_IMPOSTOR_CARD
layout(set = 1, binding = 14) uniform sampler2DArray impostor_array;
layout(location = 15) in flat uint impostor_slice;
layout(location = 16) in flat float impostor_dither;
#endif

#ifdef FEATURE_TERRAIN
layout(set = 1, binding = 14) uniform sampler2D terrain_normal_map;
layout(set = 1, binding = 15) uniform sampler2D terrain_lightmap;
layout(set = 1, binding = 16) uniform sampler2D terrain_colour_map;
layout(set = 1, binding = 17) uniform sampler2D terrain_blend;
layout(set = 1, binding = 18) uniform sampler2D terrain_diffuse0;
layout(set = 1, binding = 19) uniform sampler2D terrain_diffuse1;
layout(set = 1, binding = 20) uniform sampler2D terrain_diffuse2;
layout(set = 1, binding = 21) uniform sampler2D terrain_diffuse3;
layout(set = 1, binding = 22) uniform sampler2D terrain_diffuse4;
layout(set = 1, binding = 23) uniform sampler2D terrain_normalheight0;
layout(set = 1, binding = 1) uniform sampler2D terrain_normalheight1;
layout(set = 1, binding = 2) uniform sampler2D terrain_normalheight2;
layout(set = 1, binding = 3) uniform sampler2D terrain_normalheight3;
layout(set = 1, binding = 4) uniform sampler2D terrain_normalheight4;
#include "terrain_layers_inc.glsl"
#endif

#ifdef FEATURE_OCEAN
#define PLANAR_REFLECTION_AVAILABLE
layout(set = 1, binding = 18) uniform sampler2D screen_texture;
layout(set = 1, binding = 19) uniform sampler2D depth_texture;
layout(set = 1, binding = 20) uniform sampler2D planar_reflection;
layout(set = 1, binding = 21) uniform sampler2D ocean_displacement;
layout(set = 1, binding = 22) uniform sampler2D ocean_gradient;
layout(set = 1, binding = 23) uniform sampler2D ocean_perlin;
layout(set = 0, binding = 7, std140) uniform PlanarReflections {
	vec4 plane_vs[4];
	mat4 projection[4];
	vec4 params[4];
}
planar_reflections;

vec2 ocean_hash_simplex_2d(vec2 p) {
	p = vec2(dot(p, vec2(127.1, 311.7)), dot(p, vec2(269.5, 183.3)));
	return -1.0 + 2.0 * fract(sin(p) * 43758.5453123);
}

float ocean_noise_simplex_2d(vec2 p) {
	const float K1 = 0.366025404;
	const float K2 = 0.211324865;

	vec2 i = floor(p + (p.x + p.y) * K1);
	vec2 a = p - i + (i.x + i.y) * K2;
	float m = step(a.y, a.x);
	vec2 o = vec2(m, 1.0 - m);
	vec2 b = a - o + K2;
	vec2 c = a - 1.0 + 2.0 * K2;
	vec3 h = max(0.5 - vec3(dot(a, a), dot(b, b), dot(c, c)), 0.0);
	vec3 n = h * h * h * h * vec3(dot(a, ocean_hash_simplex_2d(i + 0.0)), dot(b, ocean_hash_simplex_2d(i + o)), dot(c, ocean_hash_simplex_2d(i + 1.0)));
	return dot(n, vec3(70.0));
}

vec2 ocean_hash_voronoi(vec2 p) {
	p = vec2(dot(p, vec2(127.1, 311.7)), dot(p, vec2(269.5, 183.3)));
	return fract(sin(p) * 18.5453);
}

vec2 ocean_noise_voronoi(vec2 x, float seed) {
	vec2 n = floor(x);
	vec2 f = fract(x);

	vec3 m = vec3(8.0);
	for (int j = -1; j <= 1; j++) {
		for (int i = -1; i <= 1; i++) {
			vec2 g = vec2(float(i), float(j));
			vec2 o = ocean_hash_voronoi(n + g);
			vec2 r = g - f + (0.5 + 0.5 * sin(seed + 6.2831 * o));
			float d = dot(r, r);
			if (d < m.x) {
				m = vec3(d, o);
			}
		}
	}

	return vec2(sqrt(m.x), m.y + m.z);
}

vec3 ocean_reconstruct_view(mat4 inverse_projection, vec2 uv, float depth) {
	vec4 position = inverse_projection * vec4(uv * 2.0 - 1.0, depth, 1.0);
	return position.xyz / position.w;
}

float ocean_find_nearest_depth(vec2 uv, int radius) {
	vec2 dim_rcp = 1.0 / vec2(textureSize(depth_texture, 0));
	float ret = 1.0;
	for (int x = -radius; x <= radius; ++x) {
		for (int y = -radius; y <= radius; ++y) {
			ret = min(ret, textureLod(depth_texture, uv + vec2(float(x), float(y)) * dim_rcp, 0.0).r);
		}
	}
	return ret;
}
#endif

#if !defined(FEATURE_TERRAIN) && !defined(FEATURE_IMPOSTOR_CARD) && !defined(MODE_RENDER_DEPTH) && !defined(MODE_UNSHADED) && !defined(FEATURE_OCEAN)
#define MATERIAL_REFRACTION_AVAILABLE
#define PLANAR_REFLECTION_AVAILABLE
layout(set = 1, binding = 17) uniform sampler2D texture_refraction;
layout(set = 1, binding = 18) uniform sampler2D screen_texture;
layout(set = 1, binding = 19) uniform sampler2D depth_texture;
layout(set = 1, binding = 20) uniform sampler2D planar_reflection;
layout(set = 0, binding = 7, std140) uniform PlanarReflections {
	vec4 plane_vs[4];
	mat4 projection[4];
	vec4 params[4];
}
planar_reflections;
#endif
