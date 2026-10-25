#version 450

// Puts the 3D picture (drawn at its own size, viewport scaling_3d_scale or scaling_3d_size, and
// already scaled by FSR when that mode is on) on the rectangle it fills in the window with one
// linear sample per pixel. Window pixels outside the picture (the bars of scaling_3d_aspect
// Keep) are black.

layout(location = 0) in vec2 uv_interp;

layout(set = 0, binding = 0, std140) uniform UpscaleParams {
	vec2 uv_scale;
	vec2 uv_offset;
}
params;

layout(set = 1, binding = 0) uniform sampler2D source;

layout(location = 0) out vec4 frag_color;

void main() {
	vec2 uv = uv_interp * params.uv_scale + params.uv_offset;
	if (any(lessThan(uv, vec2(0.0))) || any(greaterThan(uv, vec2(1.0)))) {
		frag_color = vec4(0.0, 0.0, 0.0, 1.0);
		return;
	}
	frag_color = vec4(textureLod(source, uv, 0.0).rgb, 1.0);
}
