/* Include half precision types. */
#include "half_inc.glsl"
#include "scene_forward_clustered_inc.glsl"

#define SHADER_IS_SRGB false
#define SHADER_SPACE_FAR 0.0

#ifdef USE_MULTIVIEW
#define OUTPUT_IS_MULTIVIEW true
#else
#define OUTPUT_IS_MULTIVIEW false
#endif

/* INPUT ATTRIBS */

// Always contains vertex position in XYZ, can contain tangent angle in W.
layout(location = 0) in vec4 vertex_angle_attrib;

//only for pure render depth when normal is not used

#if defined(NORMAL_USED) || defined(TANGENT_USED)
// Contains Normal/Axis in RG, can contain tangent in BA.
layout(location = 1) in vec4 axis_tangent_attrib;
#endif

// Location 2 is unused.

#ifdef TANGENT_USED
layout(location = 2) in vec4 tangent_attrib;
#endif

#if defined(COLOR_USED)
layout(location = 3) in vec4 color_attrib;
#endif

#ifdef UV_USED
layout(location = 4) in vec2 uv_attrib;
#endif

#if defined(UV2_USED) || defined(USE_LIGHTMAP) || defined(MODE_RENDER_MATERIAL)
layout(location = 5) in vec2 uv2_attrib;
#endif

#if defined(CUSTOM0_USED)
layout(location = 6) in vec4 custom0_attrib;
#endif

#if defined(CUSTOM1_USED)
layout(location = 7) in vec4 custom1_attrib;
#endif

#if defined(CUSTOM2_USED)
layout(location = 8) in vec4 custom2_attrib;
#endif

#if defined(CUSTOM3_USED)
layout(location = 9) in vec4 custom3_attrib;
#endif

#if defined(BONES_USED) || defined(USE_PARTICLE_TRAILS)
layout(location = 10) in uvec4 bone_attrib;
#endif

#if defined(WEIGHTS_USED) || defined(USE_PARTICLE_TRAILS)
layout(location = 11) in vec4 weight_attrib;
#endif

#ifdef MOTION_VECTORS
layout(location = 12) in vec4 previous_vertex_attrib;

#if defined(NORMAL_USED) || defined(TANGENT_USED)
layout(location = 13) in vec4 previous_normal_attrib;
#endif

#endif // MOTION_VECTORS

void axis_angle_to_tbn(vec3 axis, float angle, out vec3 tangent, out vec3 binormal, out vec3 normal) {
	float c = cos(angle);
	float s = sin(angle);
	vec3 omc_axis = (1.0 - c) * axis;
	vec3 s_axis = s * axis;
	tangent = omc_axis.xxx * axis + vec3(c, -s_axis.z, s_axis.y);
	binormal = omc_axis.yyy * axis + vec3(s_axis.z, c, -s_axis.x);
	normal = omc_axis.zzz * axis + vec3(-s_axis.y, s_axis.x, c);
}

/* Varyings */

layout(location = 0) out vec3 vertex_interp;

#ifdef NORMAL_USED
layout(location = 1) out vec3 normal_interp;
#endif

#if defined(COLOR_USED)
layout(location = 2) out vec4 color_interp;
#endif

#ifdef UV_USED
layout(location = 3) out vec2 uv_interp;
#endif

#if defined(UV2_USED) || defined(USE_LIGHTMAP)
layout(location = 4) out vec2 uv2_interp;
#endif

#ifdef TANGENT_USED
layout(location = 5) out vec3 tangent_interp;
layout(location = 6) out vec3 binormal_interp;
#endif

#ifdef MOTION_VECTORS
layout(location = 7) out vec4 screen_position;
layout(location = 8) out vec4 prev_screen_position;
#endif

#ifdef MATERIAL_UNIFORMS_USED
/* clang-format off */
layout(set = 0, binding = 2, std140) uniform MaterialUniforms {
#include "scene_material_uniforms_inc.glsl"
} material;
/* clang-format on */
#endif

#ifdef FEATURE_OCEAN
layout(set = 1, binding = 21) uniform sampler2D ocean_displacement;
#endif

#define global_time scene_data.time

#ifdef MODE_DUAL_PARABOLOID

layout(location = 9) out float dp_clip;

#endif

layout(location = 10) out flat uint instance_index_interp;

layout(location = 17) out flat float dissolve_interp;

#ifdef USE_MULTIVIEW
#extension GL_EXT_multiview : enable
#define ViewIndex gl_ViewIndex

vec3 multiview_uv(vec2 uv) {
	return vec3(uv, ViewIndex);
}
ivec3 multiview_uv(ivec2 uv) {
	return ivec3(uv, int(ViewIndex));
}
layout(location = 11) out vec4 combined_projected;
#else // USE_MULTIVIEW
#define ViewIndex 0
vec2 multiview_uv(vec2 uv) {
	return uv;
}
ivec2 multiview_uv(ivec2 uv) {
	return uv;
}
#endif //USE_MULTIVIEW

#if !defined(MODE_RENDER_DEPTH) && !defined(MODE_UNSHADED) && defined(USE_VERTEX_LIGHTING)
layout(location = 12) out vec4 diffuse_light_interp;
layout(location = 13) out vec4 specular_light_interp;

#include "scene_forward_vertex_lights_inc.glsl"

void cluster_get_item_range(uint p_offset, out uint item_min, out uint item_max, out uint item_from, out uint item_to) {
	uint item_min_max = cluster_buffer.data[p_offset];
	uint range_min = item_min_max & 0xFFFFu;
	uint range_max = item_min_max >> 16;
	item_min = range_min;
	item_max = range_max;

	item_from = range_min >> 5;
	item_to = (range_max == 0) ? 0 : ((range_max - 1) >> 5) + 1; //side effect of how it is stored, as item_max 0 means no elements
}

uint cluster_get_range_clip_mask(uint i, uint z_min, uint z_max) {
	int local_min = clamp(int(z_min) - int(i) * 32, 0, 31);
	int mask_width = min(int(z_max) - int(z_min), 32 - local_min);
	return bitfieldInsert(uint(0), uint(0xFFFFFFFF), local_min, mask_width);
}
#endif // !defined(MODE_RENDER_DEPTH) && !defined(MODE_UNSHADED) && defined(USE_VERTEX_LIGHTING)

#if defined(POINT_SIZE_USED) && defined(POINT_COORD_USED)
layout(location = 14) out vec2 point_coord_interp;
#endif

invariant gl_Position;


#ifdef USE_DOUBLE_PRECISION
// Helper functions for emulating double precision when adding floats.
vec3 quick_two_sum(vec3 a, vec3 b, out vec3 out_p) {
	vec3 s = a + b;
	out_p = b - (s - a);
	return s;
}

vec3 two_sum(vec3 a, vec3 b, out vec3 out_p) {
	vec3 s = a + b;
	vec3 v = s - a;
	out_p = (a - (s - v)) + (b - v);
	return s;
}

vec3 double_add_vec3(vec3 base_a, vec3 prec_a, vec3 base_b, vec3 prec_b, out vec3 out_precision) {
	vec3 s, t, se, te;
	s = two_sum(base_a, base_b, se);
	t = two_sum(prec_a, prec_b, te);
	se += t;
	s = quick_two_sum(s, se, se);
	se += te;
	s = quick_two_sum(s, se, out_precision);
	return s;
}
#endif

uint multimesh_stride() {
	uint stride = sc_multimesh_format_2d() ? 2 : 3;
	stride += sc_multimesh_has_color() ? 1 : 0;
	stride += sc_multimesh_has_custom_data() ? 1 : 0;
	return stride;
}

void vertex_shader(vec3 vertex_input,
#ifdef NORMAL_USED
		in vec3 normal_input,
#endif
#ifdef TANGENT_USED
		in vec3 tangent_input,
		in vec3 binormal_input,
#endif
		in uint instance_index, in uint multimesh_offset, in SceneData scene_data, in mat3x4 in_model_matrix,
#ifdef USE_DOUBLE_PRECISION
		in vec3 model_precision,
#endif
		out vec4 screen_pos) {
	vec4 instance_custom = vec4(0.0);
#if defined(COLOR_USED)
	color_interp = color_attrib;
#endif

	mat4 inv_view_matrix = transpose(mat4(scene_data.inv_view_matrix[0],
			scene_data.inv_view_matrix[1],
			scene_data.inv_view_matrix[2],
			vec4(0.0, 0.0, 0.0, 1.0)));

	mat4 model_matrix = transpose(mat4(in_model_matrix[0],
			in_model_matrix[1],
			in_model_matrix[2],
			vec4(0.0, 0.0, 0.0, 1.0)));

#ifdef USE_DOUBLE_PRECISION
	vec3 view_precision = scene_data.inv_view_precision.xyz;
#endif

	mat3 model_normal_matrix;
	if (bool(instances.data[instance_index].flags & INSTANCE_FLAGS_NON_UNIFORM_SCALE)) {
		model_normal_matrix = transpose(inverse(mat3(model_matrix)));
	} else {
		model_normal_matrix = mat3(model_matrix);
	}

	mat4 matrix;
	mat4 read_model_matrix = model_matrix;

#ifdef FEATURE_MULTIMESH
	if (sc_multimesh()) {
		//multimesh, instances are for it

#ifdef USE_PARTICLE_TRAILS
		uint trail_size = (instances.data[instance_index].flags >> INSTANCE_FLAGS_PARTICLE_TRAIL_SHIFT) & INSTANCE_FLAGS_PARTICLE_TRAIL_MASK;
		uint stride = 3 + 1 + 1; //particles always uses this format

		uint offset = trail_size * stride * INSTANCE_INDEX;

#ifdef COLOR_USED
		vec4 pcolor;
#endif
		{
			uint boffset = offset + bone_attrib.x * stride;
			matrix = mat4(transforms.data[boffset + 0], transforms.data[boffset + 1], transforms.data[boffset + 2], vec4(0.0, 0.0, 0.0, 1.0)) * weight_attrib.x;
#ifdef COLOR_USED
			pcolor = transforms.data[boffset + 3] * weight_attrib.x;
#endif
		}
		if (weight_attrib.y > 0.001) {
			uint boffset = offset + bone_attrib.y * stride;
			matrix += mat4(transforms.data[boffset + 0], transforms.data[boffset + 1], transforms.data[boffset + 2], vec4(0.0, 0.0, 0.0, 1.0)) * weight_attrib.y;
#ifdef COLOR_USED
			pcolor += transforms.data[boffset + 3] * weight_attrib.y;
#endif
		}
		if (weight_attrib.z > 0.001) {
			uint boffset = offset + bone_attrib.z * stride;
			matrix += mat4(transforms.data[boffset + 0], transforms.data[boffset + 1], transforms.data[boffset + 2], vec4(0.0, 0.0, 0.0, 1.0)) * weight_attrib.z;
#ifdef COLOR_USED
			pcolor += transforms.data[boffset + 3] * weight_attrib.z;
#endif
		}
		if (weight_attrib.w > 0.001) {
			uint boffset = offset + bone_attrib.w * stride;
			matrix += mat4(transforms.data[boffset + 0], transforms.data[boffset + 1], transforms.data[boffset + 2], vec4(0.0, 0.0, 0.0, 1.0)) * weight_attrib.w;
#ifdef COLOR_USED
			pcolor += transforms.data[boffset + 3] * weight_attrib.w;
#endif
		}

		instance_custom = transforms.data[offset + 4];

#ifdef COLOR_USED
		color_interp *= pcolor;
#endif

#else
		uint stride = multimesh_stride();
		uint offset = stride * (INSTANCE_INDEX + multimesh_offset);

		if (sc_multimesh_format_2d()) {
			matrix = mat4(transforms.data[offset + 0], transforms.data[offset + 1], vec4(0.0, 0.0, 1.0, 0.0), vec4(0.0, 0.0, 0.0, 1.0));
			offset += 2;
		} else {
			matrix = mat4(transforms.data[offset + 0], transforms.data[offset + 1], transforms.data[offset + 2], vec4(0.0, 0.0, 0.0, 1.0));
			offset += 3;
		}

		if (sc_multimesh_has_color()) {
#ifdef COLOR_USED
			color_interp *= transforms.data[offset];
#endif
			offset += 1;
		}

		if (sc_multimesh_has_custom_data()) {
			instance_custom = transforms.data[offset];
		}

#endif
		//transpose
		matrix = transpose(matrix);
#if !defined(USE_DOUBLE_PRECISION) || defined(SKIP_TRANSFORM_USED) || defined(VERTEX_WORLD_COORDS_USED) || defined(MODEL_MATRIX_USED)
		// Normally we can bake the multimesh transform into the model matrix, but when using double precision
		// we avoid baking it in so we can emulate high precision.
		read_model_matrix = model_matrix * matrix;
#if !defined(USE_DOUBLE_PRECISION) || defined(SKIP_TRANSFORM_USED) || defined(VERTEX_WORLD_COORDS_USED)
		model_matrix = read_model_matrix;
#endif // !defined(USE_DOUBLE_PRECISION) || defined(SKIP_TRANSFORM_USED) || defined(VERTEX_WORLD_COORDS_USED)
#endif // !defined(USE_DOUBLE_PRECISION) || defined(SKIP_TRANSFORM_USED) || defined(VERTEX_WORLD_COORDS_USED) || defined(MODEL_MATRIX_USED)
		model_normal_matrix = model_normal_matrix * mat3(matrix);
	}
#endif // FEATURE_MULTIMESH

	vec3 vertex = vertex_input;
#ifdef NORMAL_USED
	vec3 normal_highp = normal_input;
#endif

#ifdef TANGENT_USED
	vec3 tangent = tangent_input;
	vec3 binormal = binormal_input;
#endif

#ifdef UV_USED
	uv_interp = uv_attrib;
#endif

#if defined(UV2_USED) || defined(USE_LIGHTMAP)
	uv2_interp = uv2_attrib;
#endif

	vec4 uv_scale = instances.data[instance_index].uv_scale;

	if (uv_scale != vec4(0.0)) { // Compression enabled
#ifdef UV_USED
		uv_interp = (uv_interp - 0.5) * uv_scale.xy;
#endif
#if defined(UV2_USED) || defined(USE_LIGHTMAP)
		uv2_interp = (uv2_interp - 0.5) * uv_scale.zw;
#endif
	}

#ifdef OVERRIDE_POSITION
	vec4 position = vec4(1.0);
#endif

#ifdef USE_MULTIVIEW
	mat4 combined_projection = scene_data.projection_matrix;
	mat4 projection_matrix = scene_data.projection_matrix_view[ViewIndex];
	mat4 inv_projection_matrix = scene_data.inv_projection_matrix_view[ViewIndex];
	vec3 eye_offset = scene_data.eye_offset[ViewIndex].xyz;
#else
	mat4 projection_matrix = scene_data.projection_matrix;
	mat4 inv_projection_matrix = scene_data.inv_projection_matrix;
	vec3 eye_offset = vec3(0.0, 0.0, 0.0);
#endif //USE_MULTIVIEW

//using world coordinates
#if !defined(SKIP_TRANSFORM_USED) && defined(VERTEX_WORLD_COORDS_USED)

	vertex = (model_matrix * vec4(vertex, 1.0)).xyz;

#ifdef NORMAL_USED
	// For correct non-uniform scale handling, normal has to be transformed by normal matrix, but tangent vectors need to use model matrix as is
	normal_highp = model_normal_matrix * normal_highp;
#endif

#ifdef TANGENT_USED
	// For non-uniform scale, this produces non-orthogonal TBNs; ideally binormal should be reconstructed in fragment shader with cross
	tangent = mat3(model_matrix) * tangent;
	binormal = mat3(model_matrix) * binormal;
#endif
#endif

#ifdef Z_CLIP_SCALE_USED
	float z_clip_scale = 1.0;
#endif

	float roughness_highp = 1.0;

	mat4 read_view_matrix = transpose(mat4(scene_data.view_matrix[0],
			scene_data.view_matrix[1],
			scene_data.view_matrix[2],
			vec4(0.0, 0.0, 0.0, 1.0)));

#ifdef USE_DOUBLE_PRECISION
	mat4 modelview = read_view_matrix * model_matrix;

	// We separate the basis from the origin because the basis is fine with single point precision.
	// Then we combine the translations from the model matrix and the view matrix using emulated doubles.
	// We add the result to the vertex and ignore the final lost precision.
	vec3 model_origin = model_matrix[3].xyz;
	if (sc_multimesh()) {
		modelview = modelview * matrix;

		vec3 instance_origin = mat3(model_matrix) * matrix[3].xyz;
		model_origin = double_add_vec3(model_origin, model_precision, instance_origin, vec3(0.0), model_precision);
	}

	// Overwrite the translation part of modelview with improved precision.
	vec3 temp_precision; // Will be ignored.
	modelview[3].xyz = double_add_vec3(model_origin, model_precision, inv_view_matrix[3].xyz, view_precision, temp_precision);
	modelview[3].xyz = mat3(read_view_matrix) * modelview[3].xyz;
#else
	mat4 modelview = read_view_matrix * model_matrix;
#endif
	mat3 modelview_normal = mat3(read_view_matrix) * model_normal_matrix;
	vec2 read_viewport_size = scene_data.viewport_size;

#ifdef POINT_SIZE_USED
	float point_size = 1.0;
#endif

	{
#include "scene_material_vertex_inc.glsl"
	}

	float roughness = roughness_highp;
#ifdef NORMAL_USED
	vec3 normal = normal_highp;
#endif

// using local coordinates (default)
#if !defined(SKIP_TRANSFORM_USED) && !defined(VERTEX_WORLD_COORDS_USED)

	vertex = (modelview * vec4(vertex, 1.0)).xyz;

#ifdef NORMAL_USED
	// For correct non-uniform scale handling, normal has to be transformed by normal matrix, but tangent vectors need to use model matrix as is
	normal = modelview_normal * normal;
#endif

#ifdef TANGENT_USED
	// For non-uniform scale, this produces non-orthogonal TBNs; ideally binormal should be reconstructed in fragment shader with cross
	tangent = mat3(modelview) * tangent;
	binormal = mat3(modelview) * binormal;
#endif
#endif // !defined(SKIP_TRANSFORM_USED) && !defined(VERTEX_WORLD_COORDS_USED)

//using world coordinates
#if !defined(SKIP_TRANSFORM_USED) && defined(VERTEX_WORLD_COORDS_USED)

	vertex = (read_view_matrix * vec4(vertex, 1.0)).xyz;
#ifdef NORMAL_USED
	normal = (read_view_matrix * vec4(normal, 0.0)).xyz;
#endif

#ifdef TANGENT_USED
	binormal = (read_view_matrix * vec4(binormal, 0.0)).xyz;
	tangent = (read_view_matrix * vec4(tangent, 0.0)).xyz;
#endif
#endif

	vertex_interp = vertex;

	// Normalize TBN vectors to account for model/normal transforms that may have scale
#ifdef NORMAL_USED
	normal_interp = normalize(normal);
#endif

#ifdef TANGENT_USED
	tangent_interp = normalize(tangent);
	binormal_interp = normalize(binormal);
#endif

#ifdef MODE_RENDER_DEPTH

#ifdef MODE_DUAL_PARABOLOID

	vertex_interp.z *= scene_data.dual_paraboloid_side;

	dp_clip = vertex_interp.z; //this attempts to avoid noise caused by objects sent to the other parabolloid side due to bias

	//for dual paraboloid shadow mapping, this is the fastest but least correct way, as it curves straight edges

	vec3 vtx = vertex_interp;
	float distance = length(vtx);
	vtx = normalize(vtx);
	vtx.xy /= 1.0 - vtx.z;
	vtx.z = (distance / scene_data.z_far);
	vtx.z = vtx.z * 2.0 - 1.0;
	vertex_interp = vtx;

#endif

#endif //MODE_RENDER_DEPTH

#ifdef OVERRIDE_POSITION
	gl_Position = position;
#else
	gl_Position = projection_matrix * vec4(vertex_interp, 1.0);
#endif

#ifdef USE_MULTIVIEW
	combined_projected = combined_projection * vec4(vertex_interp, 1.0);
#endif

#ifdef MOTION_VECTORS
	screen_pos = gl_Position;
#else
	screen_pos = vec4(0.0);
#endif

#if !defined(MODE_RENDER_DEPTH) && !defined(MODE_UNSHADED) && defined(USE_VERTEX_LIGHTING)
	diffuse_light_interp = vec4(0.0);
	specular_light_interp = vec4(0.0);

#ifdef USE_MULTIVIEW
	vec3 view = -normalize(vertex_interp - eye_offset);
	vec2 clip_pos = clamp((combined_projected.xy / combined_projected.w) * 0.5 + 0.5, 0.0, 1.0);
#else
	vec3 view = -normalize(vertex_interp);
	vec2 clip_pos = clamp((gl_Position.xy / gl_Position.w) * 0.5 + 0.5, 0.0, 1.0);
#endif

	uvec2 screen_size = uvec2(1.0 / scene_data.screen_pixel_size);
	uvec2 screen_pixel = clamp(uvec2(clip_pos * vec2(screen_size)), uvec2(0), screen_size - uvec2(1));
	uvec2 cluster_pos = screen_pixel >> implementation_data.cluster_shift;
	uint cluster_offset = (implementation_data.cluster_width * cluster_pos.y + cluster_pos.x) * (implementation_data.max_cluster_element_count_div_32 + 32);
	uint cluster_z = uint(clamp((-vertex_interp.z / scene_data.z_far) * 32.0, 0.0, 31.0));

	{ //omni lights

		uint cluster_omni_offset = cluster_offset;

		uint item_min;
		uint item_max;
		uint item_from;
		uint item_to;

		cluster_get_item_range(cluster_omni_offset + implementation_data.max_cluster_element_count_div_32 + cluster_z, item_min, item_max, item_from, item_to);

		for (uint i = item_from; i < item_to; i++) {
			uint mask = cluster_buffer.data[cluster_omni_offset + i];
			mask &= cluster_get_range_clip_mask(i, item_min, item_max);
			uint merged_mask = mask;

			while (merged_mask != 0) {
				uint bit = findMSB(merged_mask);
				merged_mask &= ~(1u << bit);
				uint light_index = 32 * i + bit;

				if (!bool(omni_lights.data[light_index].mask & instances.data[instance_index].layer_mask)) {
					continue; //not masked
				}

				if (omni_lights.data[light_index].bake_mode == LIGHT_BAKE_STATIC && bool(instances.data[instance_index].flags & INSTANCE_FLAGS_USE_LIGHTMAP)) {
					continue; // Statically baked light and object uses lightmap, skip
				}

				light_process_omni_vertex(light_index, vertex, view, normal, roughness,
						diffuse_light_interp.rgb, specular_light_interp.rgb);
			}
		}
	}

	{ //spot lights
		uint cluster_spot_offset = cluster_offset + implementation_data.cluster_type_size;

		uint item_min;
		uint item_max;
		uint item_from;
		uint item_to;

		cluster_get_item_range(cluster_spot_offset + implementation_data.max_cluster_element_count_div_32 + cluster_z, item_min, item_max, item_from, item_to);

		for (uint i = item_from; i < item_to; i++) {
			uint mask = cluster_buffer.data[cluster_spot_offset + i];
			mask &= cluster_get_range_clip_mask(i, item_min, item_max);
			uint merged_mask = mask;

			while (merged_mask != 0) {
				uint bit = findMSB(merged_mask);
				merged_mask &= ~(1u << bit);

				uint light_index = 32 * i + bit;

				if (!bool(spot_lights.data[light_index].mask & instances.data[instance_index].layer_mask)) {
					continue; //not masked
				}

				if (spot_lights.data[light_index].bake_mode == LIGHT_BAKE_STATIC && bool(instances.data[instance_index].flags & INSTANCE_FLAGS_USE_LIGHTMAP)) {
					continue; // Statically baked light and object uses lightmap, skip
				}

				light_process_spot_vertex(light_index, vertex, view, normal, roughness,
						diffuse_light_interp.rgb, specular_light_interp.rgb);
			}
		}
	}

	{ // Directional light.

		// We process the first directional light separately as it may have shadows.
		vec3 directional_diffuse = vec3(0.0);
		vec3 directional_specular = vec3(0.0);

		for (uint i = 0; i < scene_data.directional_light_count; i++) {
			if (!bool(directional_lights.data[i].mask & instances.data[instance_index].layer_mask)) {
				continue; // Not masked, skip.
			}

			if (directional_lights.data[i].bake_mode == LIGHT_BAKE_STATIC && bool(instances.data[instance_index].flags & INSTANCE_FLAGS_USE_LIGHTMAP)) {
				continue; // Statically baked light and object uses lightmap, skip.
			}
			if (i == 0) {
				light_compute_vertex(normal, directional_lights.data[0].direction, view,
						directional_lights.data[0].color * directional_lights.data[0].energy,
						true, roughness,
						directional_diffuse,
						directional_specular);
			} else {
				light_compute_vertex(normal, directional_lights.data[i].direction, view,
						directional_lights.data[i].color * directional_lights.data[i].energy,
						true, roughness,
						diffuse_light_interp.rgb,
						specular_light_interp.rgb);
			}
		}

		// Calculate the contribution from the shadowed light so we can scale the shadows accordingly.
		float diff_avg = dot(diffuse_light_interp.rgb, vec3(0.33333));
		float diff_dir_avg = dot(directional_diffuse, vec3(0.33333));
		if (diff_avg > 0.0) {
			diffuse_light_interp.a = diff_dir_avg / (diff_avg + diff_dir_avg);
		} else {
			diffuse_light_interp.a = 1.0;
		}

		diffuse_light_interp.rgb += directional_diffuse;

		float spec_avg = dot(specular_light_interp.rgb, vec3(0.33333));
		float spec_dir_avg = dot(directional_specular, vec3(0.33333));
		if (spec_avg > 0.0) {
			specular_light_interp.a = spec_dir_avg / (spec_avg + spec_dir_avg);
		} else {
			specular_light_interp.a = 1.0;
		}

		specular_light_interp.rgb += directional_specular;
	}

#endif //!defined(MODE_RENDER_DEPTH) && !defined(MODE_UNSHADED) && defined(USE_VERTEX_LIGHTING)

#ifdef MODE_RENDER_DEPTH
	if (bool(scene_data.flags & SCENE_DATA_FLAGS_USE_PANCAKE_SHADOWS)) {
		if (gl_Position.z <= 0.0001) {
			gl_Position.z = 0.0001;
		}
	}
#endif
#ifdef MODE_RENDER_MATERIAL
	if (bool(scene_data.flags & SCENE_DATA_FLAGS_USE_UV2_MATERIAL)) {
		vec2 uv_dest_attrib;
		if (uv_scale != vec4(0.0)) {
			uv_dest_attrib = (uv2_attrib.xy - 0.5) * uv_scale.zw;
		} else {
			uv_dest_attrib = uv2_attrib.xy;
		}

		vec2 uv_offset = unpackHalf2x16(draw_call.uv_offset);
		gl_Position.xy = (uv_dest_attrib + uv_offset) * 2.0 - 1.0;
		gl_Position.z = 0.00001;
		gl_Position.w = 1.0;
	}
#endif

#ifdef Z_CLIP_SCALE_USED
	if (!bool(scene_data.flags & SCENE_DATA_FLAGS_IN_SHADOW_PASS)) {
		gl_Position.z = mix(gl_Position.w, gl_Position.z, z_clip_scale);
	}
#endif

#ifdef POINT_SIZE_USED
	if (sc_emulate_point_size) {
		vec2 point_coords[6] = vec2[](
				vec2(0, 1),
				vec2(0, 0),
				vec2(1, 1),
				vec2(0, 0),
				vec2(1, 0),
				vec2(1, 1));

		vec2 point_coord = point_coords[gl_VertexIndex % 6];
		gl_Position.xy += (point_coord * 2.0 - 1.0) * (point_size / scene_data.viewport_size) * gl_Position.w;

#ifdef POINT_COORD_USED
		point_coord_interp = point_coord;
#endif
	} else {
		gl_PointSize = point_size;
	}
#endif
}

void _unpack_vertex_attributes(vec4 p_vertex_in, vec3 p_compressed_aabb_position, vec3 p_compressed_aabb_size,
#if defined(NORMAL_USED) || defined(TANGENT_USED)
		vec4 p_normal_in, vec4 p_tangent_in,
#ifdef NORMAL_USED
		out vec3 r_normal,
#endif
		out vec3 r_tangent,
		out vec3 r_binormal,
#endif
		out vec3 r_vertex) {
	r_vertex = p_vertex_in.xyz;
#ifdef NORMAL_USED
	r_normal = p_normal_in.xyz;
#endif
#ifdef TANGENT_USED
	r_tangent = p_tangent_in.xyz;
	r_binormal = normalize(cross(p_normal_in.xyz, p_tangent_in.xyz) * p_tangent_in.w);
#elif defined(NORMAL_USED)
	r_tangent = vec3(0.0);
	r_binormal = vec3(0.0);
#endif
}

void main() {
	uint instance_index = draw_call.instance_index;
	if (!sc_multimesh()) {
		instance_index += INSTANCE_INDEX;
	}

	instance_index_interp = instance_index;

#ifdef MOTION_VECTORS
	// Previous vertex.
	vec3 prev_vertex;
#ifdef NORMAL_USED
	vec3 prev_normal;
#endif
#if defined(NORMAL_USED) || defined(TANGENT_USED)
	vec3 prev_tangent;
	vec3 prev_binormal;
#endif

	_unpack_vertex_attributes(
			previous_vertex_attrib,
			instances.data[instance_index].compressed_aabb_position_pad.xyz,
			instances.data[instance_index].compressed_aabb_size_pad.xyz,

#if defined(NORMAL_USED) || defined(TANGENT_USED)
			previous_normal_attrib,
#ifdef NORMAL_USED
			prev_normal,
#endif
			prev_tangent,
			prev_binormal,
#endif
			prev_vertex);

	vertex_shader(prev_vertex,
#ifdef NORMAL_USED
			prev_normal,
#endif
#ifdef TANGENT_USED
			prev_tangent,
			prev_binormal,
#endif
			instance_index, draw_call.multimesh_motion_vectors_previous_offset, scene_data_block.prev_data, instances.data[instance_index].prev_transform,
#ifdef USE_DOUBLE_PRECISION
			instances.data[instance_index].prev_model_precision.xyz,
#endif
			prev_screen_position);
#else
	// Unused output.
	vec4 screen_position;
#endif

	vec3 vertex;
#ifdef NORMAL_USED
	vec3 normal;
#endif
#if defined(NORMAL_USED) || defined(TANGENT_USED)
	vec3 tangent;
	vec3 binormal;
#endif

	_unpack_vertex_attributes(
			vertex_angle_attrib,
			instances.data[instance_index].compressed_aabb_position_pad.xyz,
			instances.data[instance_index].compressed_aabb_size_pad.xyz,
#if defined(NORMAL_USED) || defined(TANGENT_USED)
			axis_tangent_attrib,
#ifdef TANGENT_USED
			tangent_attrib,
#else
			vec4(0.0),
#endif
#ifdef NORMAL_USED
			normal,
#endif
			tangent,
			binormal,
#endif
			vertex);

	// Current vertex.
	vertex_shader(vertex,
#ifdef NORMAL_USED
			normal,
#endif
#ifdef TANGENT_USED
			tangent,
			binormal,
#endif
			instance_index, draw_call.multimesh_motion_vectors_current_offset, scene_data_block.data, instances.data[instance_index].transform,
#ifdef USE_DOUBLE_PRECISION
			instances.data[instance_index].model_precision.xyz,
#endif

			screen_position);
}
