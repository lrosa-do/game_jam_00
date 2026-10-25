#version 450

layout(location = 0) in vec2 uv_interp;

#define FLAG_HORIZONTAL (1 << 0)
#define FLAG_GLOW_FIRST_PASS (1 << 4)

layout(set = 0, binding = 0, std140) uniform Params {
	vec2 dest_pixel_size;
	uint flags;
	float source_lod;

	float glow_strength;
	float glow_bloom;
	float glow_hdr_threshold;
	float glow_hdr_scale;

	float glow_exposure;
	float glow_white;
	float glow_luminance_cap;
	float glow_auto_exposure_scale;
}
params;

layout(set = 1, binding = 0) uniform sampler2D source_color;

layout(location = 0) out vec4 frag_color;

void main() {
	const float kernel[5] = { 0.2024, 0.1790, 0.1240, 0.0672, 0.0285 };

	bool horizontal = bool(params.flags & FLAG_HORIZONTAL);
	bool first_pass = bool(params.flags & FLAG_GLOW_FIRST_PASS);

	vec2 step_uv = horizontal ? vec2(params.dest_pixel_size.x, 0.0) : vec2(0.0, params.dest_pixel_size.y);
	vec2 uv_min = params.dest_pixel_size * 0.5;
	vec2 uv_max = vec2(1.0) - uv_min;

	vec3 tonemap_col = vec3(0.299, 0.587, 0.114) / max(params.glow_luminance_cap, 6.0);

	vec4 color = vec4(0.0);
	for (int i = -4; i <= 4; i++) {
		vec4 tap = textureLod(source_color, clamp(uv_interp + step_uv * float(i), uv_min, uv_max), params.source_lod);
		if (horizontal && first_pass) {
			// Tonemap initial samples to reduce weight of fireflies: https://graphicrants.blogspot.com/2013/12/tone-mapping.html
			tap /= 1.0 + dot(tap.rgb, tonemap_col);
		}
		color += tap * kernel[abs(i)];
	}

	if (!horizontal) {
		if (first_pass) {
			// Undo tonemap to restore range: https://graphicrants.blogspot.com/2013/12/tone-mapping.html
			color /= 1.0 - dot(color.rgb, tonemap_col);
		}

		color *= params.glow_strength;

		if (first_pass) {
			color *= params.glow_exposure;

			float max_value = max(color.r, max(color.g, color.b));
			float feedback = max(smoothstep(params.glow_hdr_threshold, params.glow_hdr_threshold + params.glow_hdr_scale, max_value), params.glow_bloom);

			color = min(color * feedback, vec4(params.glow_luminance_cap));
		}
	}

	frag_color = color;
}
