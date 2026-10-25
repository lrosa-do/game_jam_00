#ifdef FEATURE_IMPOSTOR_CARD
	vec4 impostor_albedo = texture(impostor_array, vec3(uv, float(impostor_slice)));
	float impostor_alpha = impostor_albedo.a;
#if defined(IMPOSTOR_CARD_CUT) || defined(MODE_RENDER_DEPTH)
	{
		float cutout = impostor_alpha;
		if (dither_mask8(uvec2(gl_FragCoord.xy)) - impostor_dither < 0.0) {
			cutout = 0.0;
		}
		cutout -= 0.75;
		if (!(cutout > 0.0)) {
			discard;
		}
	}
#endif
#ifndef MODE_RENDER_DEPTH
	albedo_highp = (impostor_albedo * color).rgb;
	vec4 impostor_surface = texture(impostor_array, vec3(uv, float(impostor_slice + 2u)));
	ao = impostor_surface.r;
	roughness_highp = impostor_surface.g;
	metallic_highp = impostor_surface.b;
	specular = impostor_surface.a;
	emission = vec3(0.0);
#ifdef NORMAL_USED
	vec3 impostor_world_normal = normalize(texture(impostor_array, vec3(uv, float(impostor_slice + 1u))).rgb * 2.0 - 1.0);
	normal_highp = mat3(read_view_matrix) * impostor_world_normal;
#endif
#endif
	alpha_highp = impostor_alpha;
#ifdef IMPOSTOR_CARD_COVERAGE
	alpha_highp -= impostor_dither;
#endif
#else
	vec2 base_uv = uv;
#ifdef FEATURE_DETAIL
	vec2 base_uv2 = uv2;
#endif

	vec4 albedo_tex = vec4(1.0);
#ifdef MATERIAL_ALBEDO_TEXTURE
	albedo_tex = texture(texture_albedo, base_uv);
#endif
	albedo_tex *= color;
	vec3 albedo_base = material.albedo.rgb * albedo_tex.rgb;
#ifdef FEATURE_DETAIL
	vec2 detail_uv = material.detail.y > 0.5 ? base_uv2 : base_uv;
	vec4 detail_tex = texture(texture_detail_albedo, detail_uv);
	vec4 detail_mask_tex = texture(texture_detail_mask, base_uv);
	vec3 detail_applied = detail_tex.rgb;
	if (material.detail.x > 2.5) {
		detail_applied = albedo_base * detail_tex.rgb;
	} else if (material.detail.x > 1.5) {
		detail_applied = albedo_base - detail_tex.rgb;
	} else if (material.detail.x > 0.5) {
		detail_applied = albedo_base + detail_tex.rgb;
	}
	vec3 detail = mix(albedo_base, detail_applied, detail_tex.a);
	albedo_base = mix(albedo_base, detail, detail_mask_tex.r);
#endif
	albedo_highp = albedo_base * draw_call.instance_color.rgb;

	float metallic_tex = 1.0;
#ifdef MATERIAL_METALLIC_TEXTURE
	metallic_tex = dot(texture(texture_metallic, base_uv), material.metallic_texture_channel);
#endif
	metallic_highp = metallic_tex * material.surface.x;
	specular = material.surface.y;

	float roughness_tex = 1.0;
#ifdef MATERIAL_ROUGHNESS_TEXTURE
	roughness_tex = dot(texture(texture_roughness, base_uv), material.roughness_texture_channel);
#endif
	roughness_highp = roughness_tex * material.surface.z;

#ifdef NORMAL_MAP_USED
#ifdef FEATURE_DETAIL
	if (material.detail.w > 0.5) {
#endif
		if (material.water.x > 0.5) {
			vec2 water_uv = base_uv;
			float water_timer = global_time;
			vec2 water_wind_dir = material.water_wind.xy;
			float water_wind_speed = material.water_wind.z * 0.32;
			float water_wind_speed2 = material.water_wind.z * 2.0;
			float water_wave_chop = material.water.z * 0.01;
			float water_wave_scale = -material.water.y;
			vec3 water_normal0 = water_normal_tap(water_uv * water_wave_scale * 0.05 + water_wind_dir * water_timer * water_wind_speed * 0.04 + water_wind_speed2 * water_timer * vec2(-0.015, -0.005));
			vec3 water_normal1 = water_normal_tap(water_uv * water_wave_scale * 0.1 + water_wind_dir * water_timer * water_wind_speed * 0.08 - (water_normal0.xy / water_normal0.zz) * water_wave_chop + water_wind_speed2 * water_timer * vec2(0.020, 0.015));
			vec3 water_normal2 = water_normal_tap(water_uv * water_wave_scale * 0.25 + water_wind_dir * water_timer * water_wind_speed * 0.07 - (water_normal1.xy / water_normal1.zz) * water_wave_chop + water_wind_speed2 * water_timer * vec2(-0.04, -0.03));
			vec3 water_normal3 = water_normal_tap(water_uv * water_wave_scale * 0.5 + water_wind_dir * water_timer * water_wind_speed * 0.09 - (water_normal2.xy / water_normal2.zz) * water_wave_chop + water_wind_speed2 * water_timer * vec2(0.03, 0.04));
			vec3 water_normal4 = water_normal_tap(water_uv * water_wave_scale * 1.0 + water_wind_dir * water_timer * water_wind_speed * 0.4 - (water_normal3.xy / water_normal3.zz) * water_wave_chop + water_wind_speed2 * water_timer * vec2(-0.02, 0.1));
			vec3 water_normal5 = water_normal_tap(water_uv * water_wave_scale * 2.0 + water_wind_dir * water_timer * water_wind_speed * 0.7 - (water_normal4.xy / water_normal4.zz) * water_wave_chop + water_wind_speed2 * water_timer * vec2(0.1, -0.06));
			vec3 water_n6 = water_normal0 * material.water_waves.x + water_normal1 * material.water_waves.y +
					water_normal2 * material.water_waves.z + water_normal3 * material.water_waves.w +
					water_normal4 * material.water_waves2.x + water_normal5 * material.water_waves2.y;
			vec3 water_n1 = normalize(vec3(water_n6.x * material.water.w, water_n6.y * material.water.w, water_n6.z));
			vec3 water_n2 = normalize(vec3(water_n6.x * material.water_wind.w, water_n6.y * material.water_wind.w, water_n6.z));
			vec3 water_normal = normalize(mix(water_n1, water_n2, material.water_waves2.z));
			normal_map = vec3(water_normal.xy * 0.5 + 0.5, water_normal.z);
			normal_map_depth = 1.0;
		} else {
			normal_map = texture(texture_normal, base_uv).rgb;
			normal_map_depth = material.surface.w;
		}
#ifdef FEATURE_DETAIL
	}
	{
		vec4 detail_norm_tex = texture(texture_detail_normal, detail_uv);
		vec3 detail_norm = mix(normal_map, detail_norm_tex.rgb, detail_tex.a);
		normal_map = mix(normal_map, detail_norm, detail_mask_tex.r);
	}
#endif
#endif

#ifdef MATERIAL_EMISSION_TEXTURE
	vec3 emission_tex = texture(texture_emission, base_uv).rgb;
	emission = mix(material.emission.rgb + emission_tex, material.emission.rgb * emission_tex, material.emission.a) * material.surface2.y;
#else
	emission = material.emission.rgb * material.surface2.y;
#endif

#ifdef MATERIAL_AO_TEXTURE
	ao = dot(texture(texture_ambient_occlusion, base_uv), material.ao_texture_channel);
#endif
	ao_light_affect = material.surface2.x;
#ifdef LIGHT_BACKLIGHT_USED
	backlight = material.backlight.rgb;
#endif

#ifdef MATERIAL_REFRACTION_AVAILABLE
	if (material.refraction.y > 0.5) {
		vec3 ref_normal = normal_highp;
#ifdef NORMAL_MAP_USED
		vec3 unpacked_normal = normal_map;
		unpacked_normal.xy = unpacked_normal.xy * 2.0 - 1.0;
		unpacked_normal.z = sqrt(max(0.0, 1.0 - dot(unpacked_normal.xy, unpacked_normal.xy)));
		ref_normal = normalize(mix(
				normal_highp,
				tangent * unpacked_normal.x + binormal * unpacked_normal.y + normal_highp * unpacked_normal.z,
				normal_map_depth));
#endif
		vec2 ref_ofs = screen_uv - ref_normal.xy * dot(texture(texture_refraction, base_uv), material.refraction_texture_channel) * material.refraction.x;

		float ref_amount = 1.0 - material.albedo.a * albedo_tex.a;

		float refraction_depth_tex = textureLod(depth_texture, ref_ofs, 0.0).r;
		vec4 refraction_ndc = vec4(screen_uv * 2.0 - 1.0, refraction_depth_tex, 1.0);
		vec4 refraction_view_pos = inv_projection_matrix * refraction_ndc;
		refraction_view_pos.xyz /= refraction_view_pos.w;

		vec3 refraction_color = textureLod(screen_texture, mix(screen_uv, ref_ofs, smoothstep(0.0, 1.0, vertex.z - refraction_view_pos.z)), roughness_highp * 8.0).rgb;
		float water_shore = 1.0;
		if (material.water.x > 0.5) {
			float water_depth_tex = textureLod(depth_texture, screen_uv, 0.0).r;
			vec4 water_ndc = vec4(screen_uv * 2.0 - 1.0, water_depth_tex, 1.0);
			vec4 water_view_pos = inv_projection_matrix * water_ndc;
			water_view_pos.xyz /= water_view_pos.w;
			float water_depth = dot(normalize(normal_highp), vertex - water_view_pos.xyz);
			float water_d = min(40.0, max(0.0, 3.0 + 10.0 * water_depth * material.water_depth.w));
			refraction_color *= max(vec3(0.0), vec3(1.0) - water_d * material.water_depth.rgb);
			water_shore = max(0.0, min(1.0, water_depth * material.water_waves2.w));
		}

		emission += refraction_color * ref_amount;
		albedo_highp *= 1.0 - ref_amount;
		alpha_highp = draw_call.instance_color.a * water_shore;
	} else {
		alpha_highp *= material.albedo.a * albedo_tex.a * draw_call.instance_color.a;
	}
#else
	alpha_highp *= material.albedo.a * albedo_tex.a * draw_call.instance_color.a;
#endif
	if (dissolve_interp > 0.0) {
		alpha_highp *= 1.0 - dissolve_interp;
#ifdef ALPHA_ANTIALIASING_EDGE_USED
		alpha_highp -= dissolve_interp;
#else
		if (dither_mask8(uvec2(gl_FragCoord.xy)) - dissolve_interp < 0.0) {
			discard;
		}
#endif
	}
#ifdef ALPHA_SCISSOR_USED
	alpha_scissor_threshold = material.surface2.z;
#endif
#ifdef ALPHA_ANTIALIASING_EDGE_USED
	alpha_antialiasing_edge = material.surface2.w;
#ifdef MATERIAL_ALBEDO_TEXTURE
	alpha_texture_coordinate = uv * vec2(textureSize(texture_albedo, 0));
#endif
#endif

#ifdef FEATURE_OCEAN
	{
		mat4 ocean_inv_view = transpose(mat4(scene_data_block.data.inv_view_matrix[0],
				scene_data_block.data.inv_view_matrix[1],
				scene_data_block.data.inv_view_matrix[2],
				vec4(0.0, 0.0, 0.0, 1.0)));
		mat4 ocean_inv_projection = inv_projection_matrix;
		vec4 ocean_color = material.ocean_color;
		float ocean_height = material.ocean_extinction.w;
		float ocean_patch_recip = material.ocean_params.x;
		float lineardepth = -vertex.z;

		if (lineardepth < max(scene_data_block.data.z_near + 1.0, 1.0)) {
			discard;
		}

		float dist = length(vertex);
		vec3 ocean_p = (ocean_inv_view * vec4(vertex, 1.0)).xyz;
		vec3 ocean_v = mat3(ocean_inv_view) * (-vertex / dist);

		vec4 gradient = texture(ocean_gradient, uv);

		const int detail_count = 3;
		vec2 gradient_detail = vec2(0.0);
		for (int i = 0; i < detail_count; ++i) {
			gradient_detail += texture(ocean_gradient, uv * pow(2.0, float(i + 1))).rg;
		}
		gradient_detail /= float(detail_count);
		gradient.rg += gradient_detail;

		const float g_PerlinSize = 1.0;
		const vec2 g_UVBase = vec2(0.0);
		const vec3 g_PerlinOctave = vec3(1.12, 0.59, 0.23);
		float g_PerlinMovement = -global_time * material.ocean_misc.x * 0.06;
		const vec3 g_PerlinGradient = vec3(1.4, 1.6, 2.2);

		vec2 perlin_tc = uv * g_PerlinSize + g_UVBase;
		vec2 perlin_tc0 = perlin_tc * g_PerlinOctave.x + g_PerlinMovement;
		vec2 perlin_tc1 = perlin_tc * g_PerlinOctave.y + g_PerlinMovement;
		vec2 perlin_tc2 = perlin_tc * g_PerlinOctave.z + g_PerlinMovement;

		vec2 perlin_0 = texture(ocean_perlin, perlin_tc0).xy;
		vec2 perlin_1 = texture(ocean_perlin, perlin_tc1).xy;
		vec2 perlin_2 = texture(ocean_perlin, perlin_tc2).xy;

		vec2 perlin = (perlin_0 * g_PerlinGradient.x + perlin_1 * g_PerlinGradient.y + perlin_2 * g_PerlinGradient.z);
		perlin = perlin * 0.000005 * material.ocean_params.w;

		float gradient_fade = smoothstep(0.08, 0.8, clamp(dist * 0.005, 0.0, 1.0));
		gradient.rg = mix(gradient.rg, perlin, gradient_fade);

		const float bump_strength = 0.1;

		vec3 ocean_n = normalize(vec3(gradient.x, material.ocean_params.y * 2.0, gradient.y));
		normal_highp = mat3(read_view_matrix) * ocean_n;
		albedo_highp = ocean_color.rgb;
		specular = 0.35355339;
		roughness_highp = 0.1;
		metallic_highp = 0.0;
		vec3 ocean_extinction = material.ocean_extinction.rgb;
		float ocean_fresnel = 0.02 + 0.98 * pow(1.0 - clamp(dot(ocean_n, ocean_v), 0.0, 1.0), 5.0);

		vec4 ocean_refraction = vec4(0.0);
		float water_depth;
		{
			vec2 refraction_uv = screen_uv + ocean_n.xz * bump_strength;
			float refraction_depth = ocean_find_nearest_depth(refraction_uv, 2);
			vec3 refraction_position = (ocean_inv_view * vec4(ocean_reconstruct_view(ocean_inv_projection, refraction_uv, refraction_depth), 1.0)).xyz;
			water_depth = ocean_height - refraction_position.y;
			water_depth += textureLod(ocean_displacement, refraction_position.xz * ocean_patch_recip, 0.0).z;
			if (water_depth <= 0.0) {
				refraction_uv = screen_uv;
			} else {
				refraction_uv = screen_uv + ocean_n.xz * bump_strength * clamp(1.0 - exp(-water_depth), 0.0, 1.0);
			}
			ocean_refraction.rgb = textureLod(screen_texture, 1.0 - abs(1.0 - abs(refraction_uv)), 0.0).rgb;
			refraction_depth = textureLod(depth_texture, refraction_uv, 0.0).r;
			refraction_position = (ocean_inv_view * vec4(ocean_reconstruct_view(ocean_inv_projection, refraction_uv, refraction_depth), 1.0)).xyz;
			water_depth = max(water_depth, ocean_height - refraction_position.y);
			water_depth += textureLod(ocean_displacement, refraction_position.xz * ocean_patch_recip, 0.0).z;
			float waterfog = clamp(exp(-water_depth * ocean_color.a), 0.0, 1.0);
			vec3 transmittance = clamp(exp(-water_depth * ocean_extinction * ocean_color.a), 0.0, 1.0);
			ocean_refraction.a = waterfog;
			ocean_refraction.rgb *= transmittance;
		}

		{
			float scene_lineardepth = -ocean_reconstruct_view(ocean_inv_projection, screen_uv, textureLod(depth_texture, screen_uv, 0.0).r).z;
			float water_depth_diff = abs(scene_lineardepth - lineardepth);
			float foam_shore = clamp(exp(-water_depth_diff * 2.0), 0.0, 1.0);
			float foam_wave = pow(clamp(gradient.a * (1.0 - gradient_fade), 0.0, 1.0), 4.0) * clamp(exp(-water_depth * 0.1), 0.0, 1.0);
			float foam_combined = clamp(foam_shore + foam_wave, 0.0, 1.0);
			float foam = smoothstep(0.5, 0.6, clamp(foam_combined + 0.1, 0.0, 1.0));
			if (foam_combined > 0.002) {
				float foam_simplex = 0.0;
				foam_simplex += smoothstep(0.0, 0.8, ocean_noise_simplex_2d(ocean_p.xz * 1.0 + global_time));
				foam_simplex += smoothstep(0.0, 0.8, ocean_noise_simplex_2d(ocean_p.xz * 2.0 + global_time));
				foam_simplex += smoothstep(0.0, 0.8, ocean_noise_simplex_2d(ocean_p.zx * 4.0 - global_time * 2.0));
				float foam_voronoi = 0.0;
				foam_voronoi += smoothstep(0.5, 0.8, ocean_noise_voronoi(ocean_p.xz * 1.0, global_time).x);
				foam_voronoi += smoothstep(0.5, 0.8, ocean_noise_voronoi(ocean_p.xz * 2.0, global_time).x);
				foam_voronoi += smoothstep(0.5, 0.8, ocean_noise_voronoi(ocean_p.xz * 4.0, global_time).x);
				foam += foam_voronoi * foam_simplex * foam_combined;
			}
			foam *= 2.0;
			foam = clamp(foam, 0.0, 1.0);
			albedo_highp = mix(albedo_highp, vec3(0.6), foam);
			ocean_refraction.a *= 1.0 - foam;
			ocean_refraction.a = clamp(ocean_refraction.a + clamp(exp(-water_depth_diff * 4.0), 0.0, 1.0), 0.0, 1.0);
		}

		emission = ocean_refraction.rgb * (1.0 - ocean_fresnel) * ocean_refraction.a;
		albedo_highp *= 1.0 - ocean_refraction.a;

		float far_fade = clamp(1.0 - clamp(dist / scene_data_block.data.z_far - 0.8, 0.0, 1.0) * 5.0, 0.0, 1.0);
		alpha_highp = far_fade;
	}
#endif

#ifdef FEATURE_TERRAIN
	{
		vec2 terrain_scale = material.uv1_scale_offset.xy;
		terrain_scale.x = abs(terrain_scale.x) > 1e-6 ? terrain_scale.x : 1.0;
		terrain_scale.y = abs(terrain_scale.y) > 1e-6 ? terrain_scale.y : 1.0;
		vec2 terrain_uv = (uv_interp - material.uv1_scale_offset.zw) / terrain_scale;

		mat3 terrain_normal_matrix = mat3(read_view_matrix) * model_normal_matrix;
		vec3 terrain_normal = terrain_normal_matrix * terrain_fetch_normal(terrain_normal_map, terrain_uv);

		int terrain_layer_count = int(material.terrain_params.x + 0.5);
		bool terrain_normal_mapping = material.terrain_params.y > 0.5;
		int terrain_parallax = int(material.terrain_params.z + 0.5);
		int terrain_flags = int(material.terrain_params.w + 0.5);

		if (terrain_layer_count > 0 && (terrain_flags & 4) != 0) {
			albedo_highp = material.albedo.rgb * draw_call.instance_color.rgb * terrain_detail_factor(terrain_uv, material.terrain_uv0.x, material.terrain_uv1.z);
			specular = material.surface.y;
		} else if (terrain_layer_count > 0) {
			mat3 terrain_tbn = mat3(1.0);
			vec3 terrain_ts_normal = vec3(0.0, 0.0, 1.0);
			vec4 terrain_diffuse_spec = vec4(0.0);
			if (terrain_normal_mapping) {
				terrain_calculate_tbn(terrain_normal, terrain_normal_matrix, terrain_tbn);
			}
			terrain_shade_layers(terrain_uv, terrain_layer_count, material.terrain_uv0, material.terrain_uv1.x, terrain_normal_mapping, terrain_parallax, material.terrain_uv1.y, vertex, terrain_tbn, terrain_ts_normal, terrain_diffuse_spec);
			if (terrain_normal_mapping) {
				terrain_transform_to_ts(terrain_ts_normal, terrain_normal_matrix, terrain_normal);
			}
			albedo_highp = material.albedo.rgb * terrain_diffuse_spec.rgb * draw_call.instance_color.rgb;
			specular = terrain_diffuse_spec.a * material.surface.y;
		}

		if ((terrain_flags & 2) != 0) {
			albedo_highp *= texture(terrain_colour_map, terrain_uv).rgb;
		}
		if ((terrain_flags & 1) != 0) {
			ao = min(ao, texture(terrain_lightmap, terrain_uv).x);
			ao_light_affect = 1.0;
		}
		normal_highp = terrain_normal;
	}
#endif

#ifdef NORMAL_USED
	if (bool(scene_data.flags & SCENE_DATA_FLAGS_DEBUG_NORMAL)) {
		albedo_highp = normalize(normal_highp) * 0.5 + 0.5;
		metallic_highp = 0.0;
		emission = vec3(0.0);
		ao = 1.0;
		ao_light_affect = 0.0;
	}
#endif
#endif
