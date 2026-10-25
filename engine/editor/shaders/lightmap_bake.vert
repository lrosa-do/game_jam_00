#version 450

layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aUV2;

layout(set = 0, binding = 0, std140) uniform BakeBlock
{
    mat4 uShadowVP;
    mat4 uModel;
    vec4 uLightDirection;
    vec4 uLightColor;
    vec4 uParams;
    vec4 uAmbientSky;
    vec4 uJitter;
};

layout(location = 0) centroid out vec3 vWorldPosition;
layout(location = 1) centroid out vec3 vNormal;
layout(location = 2) centroid out vec4 vShadowPosition;

void main()
{
    vec4 world = uModel * vec4(aPosition, 1.0);
    vWorldPosition = world.xyz;
    vNormal = mat3(uModel) * aNormal;
    vShadowPosition = uShadowVP * world;
    gl_Position = vec4(aUV2 * 2.0 - 1.0 + uJitter.xy, 0.0, 1.0);
}
