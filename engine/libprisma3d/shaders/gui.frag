#version 450

layout(location = 0) in vec2 vUv;
layout(location = 1) in vec4 vColor;

layout(set = 1, binding = 0) uniform sampler2D uTexture;

layout(location = 0) out vec4 oColor;

void main()
{
    oColor = vColor * texture(uTexture, vUv);
}
