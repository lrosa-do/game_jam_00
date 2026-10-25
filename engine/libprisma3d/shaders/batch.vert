#version 450

layout(location = 0) in vec3 a_position;
layout(location = 1) in vec2 a_texcoord;
layout(location = 2) in vec4 a_color;

layout(set = 0, binding = 0, std140) uniform BatchUniforms
{
    mat4 u_mvp;
    vec4 u_params;
};

layout(location = 0) out vec2 v_texcoord;
layout(location = 1) out vec4 v_color;

vec3 srgb_to_linear(vec3 color)
{
    return mix(pow((color + vec3(0.055)) * (1.0 / 1.055), vec3(2.4)), color * (1.0 / 12.92), lessThan(color, vec3(0.04045)));
}

void main()
{
    gl_Position = u_mvp * vec4(a_position, 1.0);
    v_texcoord = a_texcoord;
    v_color = a_color;
    if (u_params.x > 0.5)
        v_color.rgb = srgb_to_linear(a_color.rgb);
}
