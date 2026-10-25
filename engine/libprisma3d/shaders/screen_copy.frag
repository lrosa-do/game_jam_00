#version 450

layout(location = 0) in vec2 uv_interp;

layout(set = 1, binding = 0) uniform sampler2D source;

layout(location = 0) out vec4 frag_color;

void main() {
	frag_color = texture(source, uv_interp);
}
