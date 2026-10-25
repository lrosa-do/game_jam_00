#version 450

layout(location = 0) in vec2 uv_interp;

layout(set = 0, binding = 0, std140) uniform PackBlock
{
    ivec4 uSize;
};

layout(set = 1, binding = 0) uniform sampler2D uLightmap;

layout(location = 0) out vec4 FragColor;

void main()
{
    ivec2 pixel = ivec2(uv_interp * vec2(uSize.zw));
    ivec2 texel = ivec2(pixel.x >> 1, pixel.y);
    vec4 c = texelFetch(uLightmap, texel, 0);
    uint bits = (pixel.x & 1) == 0 ? packHalf2x16(c.rg) : packHalf2x16(c.ba);
    FragColor = vec4(float(bits & 255u), float((bits >> 8) & 255u), float((bits >> 16) & 255u),
                    float(bits >> 24)) / 255.0;
}
