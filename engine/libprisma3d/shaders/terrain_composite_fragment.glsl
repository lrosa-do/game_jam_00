layout(location = 0) in vec2 uv_interp;

layout(set = 0, binding = 0, std140) uniform TerrainCompositeParams {
	vec4 uv_mul0;
	vec4 uv_mul1;
	vec4 light_direction;
	vec4 ambient;
	vec4 diffuse;
	vec4 params;
}
composite;

layout(set = 1, binding = 0) uniform sampler2D terrain_normal_map;
layout(set = 1, binding = 1) uniform sampler2D terrain_lightmap;
layout(set = 1, binding = 2) uniform sampler2D terrain_colour_map;
layout(set = 1, binding = 3) uniform sampler2D terrain_blend;
layout(set = 1, binding = 4) uniform sampler2D terrain_diffuse0;
layout(set = 1, binding = 5) uniform sampler2D terrain_diffuse1;
layout(set = 1, binding = 6) uniform sampler2D terrain_diffuse2;
layout(set = 1, binding = 7) uniform sampler2D terrain_diffuse3;
layout(set = 1, binding = 8) uniform sampler2D terrain_diffuse4;
layout(set = 1, binding = 9) uniform sampler2D terrain_normalheight0;
layout(set = 1, binding = 10) uniform sampler2D terrain_normalheight1;
layout(set = 1, binding = 11) uniform sampler2D terrain_normalheight2;
layout(set = 1, binding = 12) uniform sampler2D terrain_normalheight3;
layout(set = 1, binding = 13) uniform sampler2D terrain_normalheight4;

#include "terrain_layers_inc.glsl"

layout(location = 0) out vec4 frag_color;

void main() {
	int layer_count = int(composite.params.x + 0.5);
	bool normal_mapping = composite.params.y > 0.5;
	bool use_lightmap = composite.params.z > 0.5;
	bool use_colour_map = composite.params.w > 0.5;

	mat3 normal_matrix = mat3(1.0);
	vec3 normal = normal_matrix * terrain_fetch_normal(terrain_normal_map, uv_interp);

	mat3 tbn = mat3(1.0);
	vec3 ts_normal = vec3(0.0, 0.0, 1.0);
	vec4 diffuse_spec = vec4(0.0);
	if (normal_mapping) {
		terrain_calculate_tbn(normal, normal_matrix, tbn);
	}
	terrain_shade_layers(uv_interp, layer_count, composite.uv_mul0, composite.uv_mul1.x, normal_mapping, 0, 0.0, vec3(0.0), tbn, ts_normal, diffuse_spec);
	if (normal_mapping) {
		terrain_transform_to_ts(ts_normal, normal_matrix, normal);
	}

	float shadow_factor = 1.0;
	if (use_lightmap) {
		shadow_factor = min(shadow_factor, texture(terrain_lightmap, uv_interp).x);
	}

	vec3 albedo = diffuse_spec.rgb;
	if (composite.uv_mul1.y > 0.5) {
		albedo = layer_count > 0 ? terrain_detail_factor(uv_interp, composite.uv_mul0.x, composite.uv_mul1.z) : vec3(0.0);
	}
	if (use_colour_map) {
		albedo *= texture(terrain_colour_map, uv_interp).rgb;
	}

	float n_dot_l = clamp(dot(normalize(normal), -composite.light_direction.xyz), 0.0, 1.0);
	vec3 lit = composite.ambient.rgb + composite.diffuse.rgb * (n_dot_l * shadow_factor);

	frag_color = vec4(albedo * lit, 1.0);
}
