#version 450

layout(location = 0) in vec2 uv_interp;

layout(set = 0, binding = 0, std140) uniform AtlasCopyParams {
	vec4 source_rect;
}
params;

layout(set = 1, binding = 0) uniform sampler2D source_color;

layout(location = 0) out vec4 frag_color;

void main() {
	frag_color = textureLod(source_color, params.source_rect.xy + uv_interp * params.source_rect.zw, 0.0);
}
