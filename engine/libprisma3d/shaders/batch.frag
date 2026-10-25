#version 450

layout(location = 0) in vec2 v_texcoord;
layout(location = 1) in vec4 v_color;

layout(set = 1, binding = 0) uniform sampler2D u_texture;

layout(location = 0) out vec4 FragColor;

void main()
{
    FragColor = texture(u_texture, v_texcoord) * v_color;
}
