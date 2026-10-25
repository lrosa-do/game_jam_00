#version 450

layout(location = 0) out vec2 uv_interp;
layout(location = 1) out float opacity_interp;

layout(set = 0, binding = 0, std140) uniform LensFlareParams {
	vec4 light_screen_pos;
	vec4 offset_size;
	vec4 screen_size_rcp;
}
params;

layout(set = 1, binding = 0) uniform sampler2D source_depth;

const vec2 BILLBOARD[4] = vec2[4](vec2(-1.0, -1.0), vec2(1.0, -1.0), vec2(-1.0, 1.0), vec2(1.0, 1.0));

void main() {
	uv_interp = BILLBOARD[gl_VertexIndex] * 0.5 + 0.5;

	const vec3 light_screen_pos = params.light_screen_pos.xyz;
	const float occlusion_range = params.light_screen_pos.w;

	const float ref_depth = min(light_screen_pos.z, 0.999999);
	const vec2 step = params.screen_size_rcp.xy;
	float samples = 0.0;
	float visible = 0.0;

	for (float y = -occlusion_range; y <= occlusion_range; y += 1.0) {
		for (float x = -occlusion_range; x <= occlusion_range; x += 1.0) {
			vec2 uv = light_screen_pos.xy + vec2(x, y) * step;
			samples += 1.0;
			if (uv.x < 0.0 || uv.x > 1.0 || uv.y < 0.0 || uv.y > 1.0) {
				visible += 1.0;
				continue;
			}
			visible += (textureLod(source_depth, uv, 0.0).r >= ref_depth) ? 1.0 : 0.0;
		}
	}
	float opacity = visible / max(1.0, samples);

	const vec2 pos = (light_screen_pos.xy - 0.5) * vec2(2.0, 2.0);
	const vec2 mod_pos = pos * params.offset_size.x;

	opacity *= clamp(1.0 - length(pos - mod_pos), 0.0, 1.0);

	opacity_interp = opacity;

	gl_Position = vec4(mod_pos + BILLBOARD[gl_VertexIndex] * params.offset_size.yz * params.screen_size_rcp.xy, 0.0, 1.0);
}
