#ifdef UV_USED
	uv_interp = uv_interp * material.uv1_scale_offset.xy + material.uv1_scale_offset.zw;
#endif

#if defined(FEATURE_DETAIL) && !defined(USE_LIGHTMAP)
	if (bool(instances.data[instance_index].flags & (1 << 12))) {
		uv2_interp = vec2(0.0);
	}
	uv2_interp = uv2_interp * material.uv2_scale_offset.xy + material.uv2_scale_offset.zw;
#endif

#ifdef FEATURE_BILLBOARD
	mat4 main_cam_inv_view_matrix = scene_data.main_cam_inv_view_matrix;
	vec3 billboard_right = normalize(cross(vec3(0.0, 1.0, 0.0), main_cam_inv_view_matrix[2].xyz));
	vec3 billboard_front = normalize(cross(main_cam_inv_view_matrix[0].xyz, vec3(0.0, 1.0, 0.0)));
	modelview = read_view_matrix * mat4(
			vec4(billboard_right, 0.0),
			vec4(0.0, 1.0, 0.0, 0.0),
			vec4(billboard_front, 0.0),
			model_matrix[3]);
	if (material.billboard.x > 0.5) {
		modelview = modelview * mat4(
				vec4(length(model_matrix[0].xyz), 0.0, 0.0, 0.0),
				vec4(0.0, length(model_matrix[1].xyz), 0.0, 0.0),
				vec4(0.0, 0.0, length(model_matrix[2].xyz), 0.0),
				vec4(0.0, 0.0, 0.0, 1.0));
	}
	modelview_normal = mat3(modelview);
#endif

#ifdef FEATURE_GRASS
	{
		mat3 node_basis = mat3(transpose(mat4(in_model_matrix[0], in_model_matrix[1], in_model_matrix[2], vec4(0.0, 0.0, 0.0, 1.0))));
		mat4 main_cam_inv_view_matrix = scene_data.main_cam_inv_view_matrix;
		vec3 camera_position = main_cam_inv_view_matrix[3].xyz;
		vec3 camera_up = main_cam_inv_view_matrix[1].xyz;

		vec3 root = model_matrix[3].xyz;
		vec2 seed = root.xz + vec2(root.y * 3.7);
		float tint_hash = fract(sin(dot(seed + vec2(55.3), vec2(127.1, 311.7))) * 43758.5453);

		float draw_distance = material.grass.x;
		float fade = 1.0 - smoothstep(draw_distance * 0.7, draw_distance, length(root - camera_position));

		vec3 up = normalize(model_matrix[1].xyz);
		float height_t = vertex_input.y;
		float height = length(model_matrix[1].xyz) * fade;

		vec4 region = color_interp;
		uv_interp = uv_interp * region.xy + region.zw;

		float stem = max(instance_custom.w, 0.001);
		vec3 wind_direction = normalize(vec3(material.grass_wind.x, 0.0, material.grass_wind.y));
		float wind_time = global_time * material.grass_wind.w;
		float wind_wave = sin(wind_time * 1.6 + root.x * 0.16 + root.z * 0.11) + 0.5 * sin(wind_time * 2.7 + root.x * 0.29);
		vec3 tip = up * stem + node_basis * instance_custom.xyz + wind_direction * (wind_wave * material.grass_wind.z);
		float tip_length = length(tip);
		tip = tip_length > 1e-5 ? tip * (stem / tip_length) : up * stem;
		float below = dot(tip, up);
		if (below < 0.0) {
			tip -= up * below;
			float below_length = length(tip);
			if (below_length > 1e-5) {
				tip *= stem / below_length;
			}
		}
		vec3 deviation = tip - up * height;
		vec3 pos = root + (vertex - root) * fade + deviation * (height_t * height_t);
		float edge_on = 1.0 - abs(dot(camera_up, up));
		pos += camera_up * (edge_on * material.grass.y * height_t * height);
		if (fade <= 0.001) {
			pos = root - up * 1000.0;
		}

#ifdef NORMAL_USED
		vec3 quad_normal = normalize(normal_highp);
		float facing = dot(quad_normal, camera_position - pos) >= 0.0 ? 1.0 : -1.0;
		normal_highp = facing * normalize(mix(facing * quad_normal, up, 0.6));
#endif
		vertex = pos;

		vec3 tint = mix(material.grass_tint_a.rgb, material.grass_tint_b.rgb, tint_hash);
		color_interp = vec4(tint * mix(0.45, 1.0, height_t), 1.0);
	}
#endif

#ifdef FEATURE_LEAF_WIND
	{
		float model_height = max(material.leaf_wind_model.x, 0.001);
		float height01 = clamp(vertex_input.y / model_height, 0.0, 1.0);
		vec3 raw_position = vertex_input / model_height * 2.0;
		float axis_distance = length(vertex_input.xz) * length(model_matrix[0].xyz);
		vec3 instance_position = model_matrix[3].xyz;
		float phase = dot(raw_position, vec3(0.71, 0.33, 0.57)) + instance_position.x * 0.12 + instance_position.z * 0.09;
		float wind_time = global_time * material.leaf_wind.w;
		float wave = sin(wind_time * 1.1 + phase) + 0.5 * sin(wind_time * 1.9 + phase * 1.7);
		vec3 wind_direction = normalize(vec3(material.leaf_wind.x, 0.0, material.leaf_wind.y));
		vec3 offset = wind_direction * wave + vec3(0.0, 0.25 * wave * wave, 0.0);
		vertex += offset * (material.leaf_wind.z * height01 * axis_distance * 0.06);
	}
#endif

#ifdef FEATURE_TERRAIN_MORPH
	{
		vec2 terrain_delta = color_attrib.xy;
		float terrain_packed = draw_call.instance_color.a * 64.0 - 2.0;
		float terrain_target = floor(terrain_packed * 0.5);
		vec2 lod_morph = vec2(terrain_packed - terrain_target * 2.0, terrain_target);
		float to_morph = -min(0.0, sign(terrain_delta.y - lod_morph.y));
		vertex.y += terrain_delta.x * to_morph * lod_morph.x;
		color_interp = vec4(1.0);
	}
#endif

#ifdef FEATURE_OCEAN
	{
		vec2 ocean_grid = vertex_input.xz;
		vec4 ocean_clip = vec4(ocean_grid * 2.0 - 1.0, 1.0, 1.0);
		ocean_clip.xy *= max(1.0, material.ocean_params.z);

		vec4 unproj_near = inv_projection_matrix * vec4(ocean_clip.xy, 0.0, 1.0);
		unproj_near.xyz /= unproj_near.w;
		unproj_near.xyz = (inv_view_matrix * vec4(unproj_near.xyz, 1.0)).xyz;
		vec4 unproj_far = inv_projection_matrix * vec4(ocean_clip.xy, 1.0, 1.0);
		unproj_far.xyz /= unproj_far.w;
		unproj_far.xyz = (inv_view_matrix * vec4(unproj_far.xyz, 1.0)).xyz;
		vec3 ocean_d = normalize(unproj_far.xyz - unproj_near.xyz);
		vec3 ocean_o = unproj_near.xyz;

		float ocean_height = material.ocean_extinction.w;
		float ocean_plane_dist = (ocean_height - ocean_o.y) / ocean_d.y;
		vec3 ocean_world;
		if (ocean_plane_dist > 0.0) {
			ocean_world = ocean_o + ocean_d * ocean_plane_dist;
		} else {
			ocean_world = vec3(ocean_o.x, ocean_height, ocean_o.z) + normalize(vec3(ocean_d.x, 0.0, ocean_d.z)) * scene_data.z_far;
		}

		vec2 ocean_uv = ocean_world.xz * material.ocean_params.x;
		vec2 ocean_edge = material.ocean_misc.yz * 0.5;
		if (ocean_grid.x > ocean_edge.x && ocean_grid.x < 1.0 - ocean_edge.x && ocean_grid.y > ocean_edge.y && ocean_grid.y < 1.0 - ocean_edge.y) {
			vec3 ocean_displace = textureLod(ocean_displacement, ocean_uv, 0.0).xzy;

			float ocean_dist = length(ocean_world - unproj_near.xyz);
			float ocean_fade = smoothstep(0.08, 0.8, clamp(ocean_dist * 0.005, 0.0, 1.0));
			ocean_displace = mix(ocean_displace, vec3(0.0), ocean_fade);

			ocean_world += ocean_displace;
		}

		vertex = ocean_world;
#ifdef NORMAL_USED
		normal_highp = vec3(0.0, 1.0, 0.0);
#endif
		uv_interp = ocean_uv;
	}
#endif

	dissolve_interp = 0.0;
	if (bool(instances.data[instance_index].flags & (1 << 13)) && !bool(scene_data.flags & SCENE_DATA_FLAGS_IN_SHADOW_PASS)) {
		dissolve_interp = instance_custom.x;
	}
