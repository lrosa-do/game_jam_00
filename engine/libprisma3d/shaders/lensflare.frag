#version 450

layout(location = 0) in vec2 uv_interp;
layout(location = 1) in float opacity_interp;

layout(set = 1, binding = 1) uniform sampler2D flare_texture;

layout(location = 0) out vec4 frag_color;

void main() {
	vec4 c = texture(flare_texture, uv_interp);
	frag_color = vec4(c.rgb * c.a * opacity_interp, 1.0);
}
