	float v_angle = clamp(cube_normal.y, -1.0, 1.0);
	vec3 sky = mix(material.sky_top_color.rgb, material.sky_horizon_color.rgb, clamp(pow(1.0 - v_angle, material.inv_sky_curve), 0.0, 1.0));

	for (int light = 0; light < MAX_DIRECTIONAL_LIGHT_DATA_STRUCTS; light++) {
		if (directional_lights.data[light].enabled) {
			float sun_angle = dot(directional_lights.data[light].direction_energy.xyz, cube_normal);
			float sun_size = cos(directional_lights.data[light].color_size.w);
			vec3 sun_color = directional_lights.data[light].color_size.xyz * directional_lights.data[light].direction_energy.w;
			if (sun_angle > sun_size) {
				sky = sun_color;
			} else if (sun_angle > material.sun_angle_max) {
				float c2 = (sun_size - sun_angle) / (sun_size - material.sun_angle_max);
				sky = mix(sky, sun_color, clamp(pow(1.0 - c2, material.inv_sun_curve), 0.0, 1.0));
			}
		}
	}

	vec3 ground = mix(material.ground_bottom_color.rgb, material.ground_horizon_color.rgb, clamp(pow(1.0 + v_angle, material.inv_ground_curve), 0.0, 1.0));

	color = mix(ground, sky, step(0.0, cube_normal.y)) * material.exposure;
