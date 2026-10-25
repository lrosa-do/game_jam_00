layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;

#define FLAG_FLIP_Y (1 << 5)
#define FLAG_FORCE_LUMINANCE (1 << 6)
#define FLAG_COPY_ALL_SOURCE (1 << 7)
#define FLAG_ALPHA_TO_ONE (1 << 8)
#define FLAG_SANITIZE_INF_NAN (1 << 9)

layout(set = 0, binding = 0, std140) uniform Params {
	ivec4 section;
	ivec2 target;
	uint flags;
	float luminance_multiplier;
}
params;

layout(set = 1, binding = 0) uniform sampler2D source_color;

#ifdef MODE_SIMPLE_COPY_DEPTH
layout(r32f, set = 3, binding = 0) uniform restrict writeonly image2D dest_buffer;

vec4 reverse_depth(vec4 depth) {
	return vec4(1.0 - depth.x, depth.yzw);
}

#define texelFetch(sampler, coord, lod) reverse_depth(texelFetch(sampler, coord, lod))
#else
layout(rgba16f, set = 3, binding = 0) uniform restrict writeonly image2D dest_buffer;
#endif

void main() {
	// Pixel being shaded
	ivec2 pos = ivec2(gl_GlobalInvocationID.xy);

#ifndef MODE_GAUSSIAN_BLUR // Gaussian blur needs the extra threads
	if (any(greaterThanEqual(pos, params.section.zw))) { //too large, do nothing
		return;
	}
#endif

#ifdef MODE_MIPMAP

	ivec2 base_pos = (pos + params.section.xy) << 1;
	vec4 color = texelFetch(source_color, base_pos, 0);
	color += texelFetch(source_color, base_pos + ivec2(0, 1), 0);
	color += texelFetch(source_color, base_pos + ivec2(1, 0), 0);
	color += texelFetch(source_color, base_pos + ivec2(1, 1), 0);
	color /= 4.0;
	color = mix(color, vec4(100.0, 100.0, 100.0, 1.0), isinf(color));
	color = mix(color, vec4(100.0, 100.0, 100.0, 1.0), isnan(color));

	imageStore(dest_buffer, pos + params.target, color);
#endif

#ifdef MODE_SIMPLE_COPY

	vec4 color;
	if (bool(params.flags & FLAG_COPY_ALL_SOURCE)) {
		vec2 uv = vec2(pos) / vec2(params.section.zw);
		if (bool(params.flags & FLAG_FLIP_Y)) {
			uv.y = 1.0 - uv.y;
		}
		color = textureLod(source_color, uv, 0.0);

	} else {
		color = texelFetch(source_color, pos + params.section.xy, 0);

		if (bool(params.flags & FLAG_FLIP_Y)) {
			pos.y = params.section.w - pos.y - 1;
		}
	}

	if (bool(params.flags & FLAG_FORCE_LUMINANCE)) {
		color.rgb = vec3(max(max(color.r, color.g), color.b));
	}

	if (bool(params.flags & FLAG_ALPHA_TO_ONE)) {
		color.a = 1.0;
	}

	if (bool(params.flags & FLAG_SANITIZE_INF_NAN)) {
		color = mix(color, vec4(100.0, 100.0, 100.0, 1.0), isinf(color));
		color = mix(color, vec4(100.0, 100.0, 100.0, 1.0), isnan(color));
	}

	imageStore(dest_buffer, pos + params.target, color);

#endif // MODE_SIMPLE_COPY

#ifdef MODE_SIMPLE_COPY_DEPTH

	vec4 color = texelFetch(source_color, pos + params.section.xy, 0);

	if (bool(params.flags & FLAG_FLIP_Y)) {
		pos.y = params.section.w - pos.y - 1;
	}

	imageStore(dest_buffer, pos + params.target, vec4(color.r));

#endif // MODE_SIMPLE_COPY_DEPTH
}
