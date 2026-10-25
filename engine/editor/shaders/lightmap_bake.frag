#version 450

layout(set = 1, binding = 0) uniform sampler2D uShadowMap;

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
#define uBias uParams.x
#define uAmbientGround uParams.y
#define uFilterRadius uParams.z
#define uSampleWeight uParams.w

layout(location = 0) centroid in vec3 vWorldPosition;
layout(location = 1) centroid in vec3 vNormal;
layout(location = 2) centroid in vec4 vShadowPosition;
layout(location = 0) out vec4 FragColor;

void main()
{
    vec3 n = normalize(vNormal);
    vec3 lightToSurface = normalize(uLightDirection.xyz);
    float direct = max(dot(n, -lightToSurface), 0.0);
    vec3 shadowCoord = vShadowPosition.xyz / vShadowPosition.w;
    shadowCoord.xy = shadowCoord.xy * 0.5 + 0.5;
    float shadow = 1.0;
    if (shadowCoord.x >= 0.0 && shadowCoord.x <= 1.0 && shadowCoord.y >= 0.0 &&
        shadowCoord.y <= 1.0 && shadowCoord.z >= 0.0 && shadowCoord.z <= 1.0)
    {
        vec2 texelSize = uFilterRadius / vec2(textureSize(uShadowMap, 0));
        shadow = 0.0;
        float taps = 0.0;
        for (int y = -1; y <= 1; ++y)
            for (int x = -1; x <= 1; ++x)
            {
                vec2 uv = shadowCoord.xy + vec2(x, y) * texelSize;
                float depth = texture(uShadowMap, uv).r;
                shadow += shadowCoord.z - uBias <= depth ? 1.0 : 0.0;
                taps += 1.0;
            }
        shadow /= taps;
    }
    vec3 ambient = mix(uAmbientSky.rgb * uAmbientGround, uAmbientSky.rgb, 0.5 + 0.5 * n.y);
    FragColor = vec4((ambient + direct * shadow * uLightColor.rgb) * uSampleWeight, 1.0);
}
