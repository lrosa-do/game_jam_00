#define TERRAIN_MIN_BLEND_WEIGHT 0.0039

vec3 terrain_fetch_normal(sampler2D s, vec2 uv) {
	return 2.0 * texture(s, uv).xyz - 1.0;
}

void terrain_calculate_tbn(vec3 normal, mat3 normal_matrix, out mat3 tbn) {
	vec3 tangent = vec3(1.0, 0.0, 0.0);
	normal = normalize(normal);
	tangent = normalize(normal_matrix * tangent);
	vec3 binormal = cross(tangent, normal);
	tangent = cross(normal, binormal);
	tbn = mat3(tangent, binormal, normal);
}

void terrain_transform_to_ts(vec3 ts_normal, mat3 normal_matrix, inout vec3 normal) {
	vec3 tangent = vec3(1.0, 0.0, 0.0);
	normal = normalize(normal);
	tangent = normalize(normal_matrix * tangent);
	vec3 binormal = cross(tangent, normal);
	tangent = cross(normal, binormal);
	mat3 tbn = mat3(tangent, binormal, normal);

	normal = tbn * ts_normal;
}

vec2 terrain_parallax_texcoord(sampler2D normal_height_map, vec2 tex_coord, vec3 view_pos, float height_scale, mat3 tbn, int mode) {
	vec3 eye_vec = -view_pos * tbn;
	eye_vec = normalize(eye_vec);

	vec2 new_tex_coord = tex_coord;

	if (mode == 1) {
		float height = 1.0 - texture(normal_height_map, new_tex_coord).a;
		vec2 p = eye_vec.xy * (height * height_scale);
		new_tex_coord = new_tex_coord - p;
	} else {
		if (abs(view_pos.z) > 400.0) {
			return new_tex_coord;
		}

		const int layer_count = 32;
		float layer_depth = 1.0 / float(layer_count);
		float current_layer_depth = 0.0;
		vec2 parallax_shift = eye_vec.xy * height_scale;
		vec2 delta_tex_coords = parallax_shift / float(layer_count);

		float current_depth_map_value = 1.0 - texture(normal_height_map, new_tex_coord).a;

		for (int current_layer_id = 0; current_layer_id < layer_count; current_layer_id++) {
			new_tex_coord -= delta_tex_coords;

			current_depth_map_value = 1.0 - texture(normal_height_map, new_tex_coord).a;

			if (current_layer_depth > current_depth_map_value) {
				break;
			}

			current_layer_depth += layer_depth;
		}

		vec2 prev_tex_coords = new_tex_coord + delta_tex_coords;

		float after_depth = current_depth_map_value - current_layer_depth;
		float before_depth = (1.0 - texture(normal_height_map, prev_tex_coords).a) - current_layer_depth + layer_depth;

		float weight = after_depth / (after_depth - before_depth);
		new_tex_coord = mix(new_tex_coord, prev_tex_coords, weight);
	}
	return new_tex_coord;
}

void terrain_blend_layer(float blend_weight, vec2 uv0, float uv_mul, bool normal_mapping, int parallax_mode, vec3 view_pos, float scale_bias, mat3 tbn, sampler2D normtex, inout vec3 normal, sampler2D difftex, inout vec4 diffuse_spec) {
	if (blend_weight < TERRAIN_MIN_BLEND_WEIGHT) {
		return;
	}

	vec2 uv = mod(uv0 * uv_mul, 1.0);

	if (normal_mapping && parallax_mode != 0) {
		uv = terrain_parallax_texcoord(normtex, uv, view_pos, scale_bias, tbn, parallax_mode);
	}

	vec4 diffuse_spec_tex = texture(difftex, uv);
	diffuse_spec = mix(diffuse_spec, diffuse_spec_tex, blend_weight);

	if (normal_mapping) {
		vec3 ts_normal = terrain_fetch_normal(normtex, uv);
		normal = normalize(vec3(mix(normal.xy * ts_normal.z, ts_normal.xy * normal.z, blend_weight), ts_normal.z * normal.z));
	}
}

vec3 terrain_srgb_encode(vec3 linear_value) {
	vec3 c = clamp(linear_value, vec3(0.0), vec3(1.0));
	return mix(1.055 * pow(c, vec3(1.0 / 2.4)) - 0.055, c * 12.92, lessThanEqual(c, vec3(0.0031308)));
}

vec3 terrain_detail_factor(vec2 uv, float uv_mul, float strength) {
	vec3 detail = terrain_srgb_encode(texture(terrain_diffuse0, uv * uv_mul).rgb);
	return mix(vec3(1.0), detail * 2.0, strength);
}

void terrain_shade_layers(vec2 uv, int layer_count, vec4 uv_mul0, float uv_mul4, bool normal_mapping, int parallax_mode, float scale_bias, vec3 view_pos, mat3 tbn, out vec3 r_ts_normal, out vec4 r_diffuse_spec) {
	vec4 blend_weights = texture(terrain_blend, uv);
	vec4 diffuse_spec = vec4(0.0);
	vec3 ts_normal = vec3(0.0, 0.0, 1.0);
	if (layer_count > 0) {
		terrain_blend_layer(1.0, uv, uv_mul0.x, normal_mapping, parallax_mode, view_pos, scale_bias, tbn, terrain_normalheight0, ts_normal, terrain_diffuse0, diffuse_spec);
	}
	if (layer_count > 1) {
		terrain_blend_layer(blend_weights.x, uv, uv_mul0.y, normal_mapping, parallax_mode, view_pos, scale_bias, tbn, terrain_normalheight1, ts_normal, terrain_diffuse1, diffuse_spec);
	}
	if (layer_count > 2) {
		terrain_blend_layer(blend_weights.y, uv, uv_mul0.z, normal_mapping, parallax_mode, view_pos, scale_bias, tbn, terrain_normalheight2, ts_normal, terrain_diffuse2, diffuse_spec);
	}
	if (layer_count > 3) {
		terrain_blend_layer(blend_weights.z, uv, uv_mul0.w, normal_mapping, parallax_mode, view_pos, scale_bias, tbn, terrain_normalheight3, ts_normal, terrain_diffuse3, diffuse_spec);
	}
	if (layer_count > 4) {
		terrain_blend_layer(blend_weights.w, uv, uv_mul4, normal_mapping, parallax_mode, view_pos, scale_bias, tbn, terrain_normalheight4, ts_normal, terrain_diffuse4, diffuse_spec);
	}
	r_ts_normal = ts_normal;
	r_diffuse_spec = diffuse_spec;
}
