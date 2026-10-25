#version 450

layout(location = 0) in vec2 uv_interp;

layout(set = 0, binding = 0, std140) uniform DepthReadParams {
	vec2 uv;
	vec2 pad;
}
params;

layout(set = 1, binding = 0) uniform sampler2D source_depth;

layout(location = 0) out vec4 frag_color;

void main() {
	uint bits = floatBitsToUint(texture(source_depth, params.uv).r);
	frag_color = vec4(float(bits & 255u), float((bits >> 8) & 255u), float((bits >> 16) & 255u), float(bits >> 24)) / 255.0;
}
