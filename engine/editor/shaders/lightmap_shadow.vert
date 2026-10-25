#version 450

layout(location = 0) in vec3 aPosition;

layout(set = 0, binding = 0, std140) uniform BakeBlock
{
    mat4 uShadowVP;
    mat4 uModel;
    vec4 uLightDirection;
    vec4 uLightColor;
};

void main() { gl_Position = uShadowVP * uModel * vec4(aPosition, 1.0); }
